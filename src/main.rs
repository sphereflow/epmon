#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(generic_const_exprs)]

use crate::ringbuffer::RingBuffer;
use command::Command;
use core::mem::MaybeUninit;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpAddress, IpListenEndpoint, Stack, StackResources};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{with_timeout, Duration, Timer};
use embedded_svc::io::asynch::Write;
use esp_backtrace as _;
use esp_hal::analog::adc::{Adc, AdcConfig, AdcPin, Attenuation};
use esp_hal::clock::Clocks;
use esp_hal::gpio::{GpioPin, Io, Level, Output};
use esp_hal::peripherals::{ADC1, RADIO_CLK, RNG, TIMG0, WIFI};
use esp_hal::rmt::Rmt;
use esp_hal::rng::Rng;
use esp_hal::timer::{ErasedTimer, OneShotTimer, PeriodicTimer};
use esp_hal::uart::Uart;
use esp_hal::Blocking;
use esp_hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, system::SystemControl};
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use esp_println::println;
use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState};
use esp_wifi::EspWifiInitFor;
use heapless::Vec;
use max485::Max485Modbus;
use smart_leds::{SmartLedsWrite, RGB8};

extern crate alloc;

pub mod command;
pub mod max485;
pub mod ringbuffer;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

static ADC_READINGS: Mutex<CriticalSectionRawMutex, Option<AdcReadings>> = Mutex::new(None);

const SSID: &str = env!("WIFI_SSID");
const PASSWORD: &str = env!("WIFI_PASS");
const PORT: u16 = 8900;
const RING_BUFFER_SIZE: usize = 20000;
const INTERVAL_MS: u16 = 20;
const RX_BUFFER_SIZE: usize = 1024;
const TX_BUFFER_SIZE: usize = 1024;
// static buffers to not need a huge task-arena
static mut RX_BUFFER: [u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE];
static mut TX_BUFFER: [u8; TX_BUFFER_SIZE] = [0; TX_BUFFER_SIZE];

type LedT = SmartLedsAdapter<esp_hal::rmt::Channel<Blocking, 0>, 25>;
static LED: Mutex<CriticalSectionRawMutex, Option<LedT>> = Mutex::new(None);
static MAX485_MODBUS: Mutex<CriticalSectionRawMutex, Option<Max485Modbus>> = Mutex::new(None);

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();
    init_heap();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    // set up smart_led
    // set up adc
    let mut adc_config = AdcConfig::new();
    let adc_pin0 =
        adc_config.enable_pin_with_cal::<_, AdcCal>(io.pins.gpio4, Attenuation::Attenuation0dB);
    let adc_pin1 =
        adc_config.enable_pin_with_cal::<_, AdcCal>(io.pins.gpio5, Attenuation::Attenuation0dB);
    let adc_pin2 =
        adc_config.enable_pin_with_cal::<_, AdcCal>(io.pins.gpio6, Attenuation::Attenuation0dB);
    let adc1 = Adc::new(peripherals.ADC1, adc_config);
    {
        let mut adc_readings = ADC_READINGS.lock().await;
        adc_readings.replace(AdcReadings::default());
    }
    {
        let rmt = Rmt::new(peripherals.RMT, 80.MHz(), &clocks, None).unwrap();
        let rmt_buffer = smartLedBuffer!(1);
        let led_adapter = SmartLedsAdapter::new(rmt.channel0, io.pins.gpio8, rmt_buffer, &clocks);
        let mut led = LED.lock().await;
        led.replace(led_adapter);
        led.as_mut()
            .map(|l| l.write(Some(RGB8::new(130, 0, 0))).ok());
    }
    {
        if let Ok(uart_peripheral) =
            Uart::new_async(peripherals.UART1, &clocks, io.pins.gpio18, io.pins.gpio10)
        {
            let mut modbus = MAX485_MODBUS.lock().await;
            modbus.replace(Max485Modbus::new(
                Output::new(io.pins.gpio2, Level::Low),
                uart_peripheral,
            ));
        }
    }

    let systimer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    let timers = [OneShotTimer::new(systimer.alarm0.into())];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
    esp_hal_embassy::init(&clocks, timers);
    if let Err(err) = spawner.spawn(aquire_adc_readings_task(adc1, adc_pin0, adc_pin1, adc_pin2)) {
        log::error!("could not spawn adc task");
        log::error!("{err:?}");
    }

    let stack = init_wifi(
        peripherals.TIMG0,
        peripherals.RNG,
        peripherals.RADIO_CLK,
        peripherals.WIFI,
        &clocks,
        &spawner,
    )
    .await;
    spawner
        .spawn(network_handler(stack))
        .expect("could not spawn network_handler");

    loop {
        let mut print_buffer: [u16; 20] = [0; 20];
        {
            let mut adc_readings_guard = ADC_READINGS.lock().await;
            if let Some(adc_readings) = adc_readings_guard.as_mut() {
                adc_readings.ring_buffers[0]
                    .get_range(RING_BUFFER_SIZE - 20..RING_BUFFER_SIZE, &mut print_buffer);
                log::info!("{:?}", print_buffer);
                adc_readings.ring_buffers[1]
                    .get_range(RING_BUFFER_SIZE - 20..RING_BUFFER_SIZE, &mut print_buffer);
                log::info!("{:?}", print_buffer);
                adc_readings.ring_buffers[2]
                    .get_range(RING_BUFFER_SIZE - 20..RING_BUFFER_SIZE, &mut print_buffer);
                log::info!("{:?}", print_buffer);
            }
        }

        Timer::after_secs(1).await;
    }
}

type AdcCal = esp_hal::analog::adc::AdcCalBasic<ADC1>;
// type AdcCal = esp_hal::analog::adc::AdcCalLine<ADC1>;
// type AdcCal = esp_hal::analog::adc::AdcCalCurve<ADC1>;
type PIN0 = AdcPin<GpioPin<4>, ADC1, AdcCal>;
type PIN1 = AdcPin<GpioPin<5>, ADC1, AdcCal>;
type PIN2 = AdcPin<GpioPin<6>, ADC1, AdcCal>;

#[embassy_executor::task]
async fn aquire_adc_readings_task(
    mut adc1: Adc<'static, ADC1>,
    mut pin0: PIN0,
    mut pin1: PIN1,
    mut pin2: PIN2,
) {
    let mut r0;
    let mut r1;
    let mut r2;
    loop {
        let mut res = adc1.read_oneshot(&mut pin0);
        while res.is_err() {
            res = adc1.read_oneshot(&mut pin0);
            Timer::after_micros(100).await;
        }
        r0 = res.unwrap();
        res = adc1.read_oneshot(&mut pin1);
        while res.is_err() {
            res = adc1.read_oneshot(&mut pin1);
            Timer::after_micros(100).await;
        }
        r1 = res.unwrap();
        res = adc1.read_oneshot(&mut pin2);
        while res.is_err() {
            res = adc1.read_oneshot(&mut pin2);
            Timer::after_micros(100).await;
        }
        r2 = res.unwrap();
        // println!("r012: {r0:?}, {r1:?}, {r2:?}");
        {
            let mut adc_readings_guard = ADC_READINGS.lock().await;
            if let Some(adc_readings) = (*adc_readings_guard).as_mut() {
                adc_readings.push_value(0_usize, r0);
                adc_readings.push_value(1_usize, r1);
                adc_readings.push_value(2_usize, r2);
            }
        }
        Timer::after_millis(INTERVAL_MS as u64).await;
    }
}

#[derive(Default, Debug)]
struct AdcReadings {
    pub ring_buffers: [RingBuffer<RING_BUFFER_SIZE>; 3],
}

impl AdcReadings {
    fn push_value(&mut self, ix: usize, val: u16) {
        self.ring_buffers[ix].push(val);
    }
}

async fn init_wifi(
    timg0: TIMG0,
    rng: RNG,
    radio_clk: RADIO_CLK,
    wifi: WIFI,
    clocks: &Clocks<'_>,
    spawner: &Spawner,
) -> &'static Stack<WifiDevice<'static, WifiStaDevice>> {
    let timer = PeriodicTimer::new(
        esp_hal::timer::timg::TimerGroup::new(timg0, clocks, None)
            .timer0
            .into(),
    );

    let init = esp_wifi::initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(rng),
        radio_clk,
        clocks,
    )
    .unwrap();

    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiStaDevice).unwrap();
    let config = embassy_net::Config::dhcpv4(Default::default());

    let seed = 1234; // very random, very secure seed

    // Init network stack
    let stack = &*mk_static!(
        Stack<WifiDevice<'_, WifiStaDevice>>,
        Stack::new(
            wifi_interface,
            config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed
        )
    );
    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(stack)).ok();
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after_millis(100).await;
    }

    println!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            break;
        }
        Timer::after_millis(100).await;
    }
    stack
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    println!("start connection task");
    println!("Device capabilities: {:?}", controller.get_capabilities());
    loop {
        if esp_wifi::wifi::get_wifi_state() == WifiState::StaConnected {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            Timer::after_secs(5).await
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config =
                esp_wifi::wifi::Configuration::Client(esp_wifi::wifi::ClientConfiguration {
                    ssid: SSID.try_into().unwrap(),
                    password: PASSWORD.try_into().unwrap(),
                    ..Default::default()
                });
            controller.set_configuration(&client_config).unwrap();
            println!("Starting wifi");
            controller.start().await.unwrap();
            println!("Wifi started!");
        }
        println!("About to connect...");

        match controller.connect().await {
            Ok(_) => println!("Wifi connected!"),
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
                Timer::after_secs(5).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    stack.run().await
}

#[embassy_executor::task]
async fn network_handler(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    let mut last_addr_byte = [0];
    let mut rx_buffer = [1];
    let mut rx_meta = [PacketMetadata::EMPTY];
    // connect / reconnect loop
    loop {
        {
            let mut led = LED.lock().await;
            led.as_mut().map(|l| l.write(Some(RGB8::new(0, 0, 50))));
        }
        let mut udp_socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buffer, &mut [], &mut []);
        match udp_socket.bind(IpListenEndpoint {
            addr: Some(IpAddress::v4(0, 0, 0, 0)),
            port: 9900,
        }) {
            Ok(()) => log::info!("udp socket is bound"),
            Err(e) => log::error!("{e:?}"),
        }
        match udp_socket.recv_from(&mut last_addr_byte).await {
            Ok((size, _)) => log::info!("received: {size} byte(s) on udp socket"),
            Err(e) => log::error!("{e:?}"),
        }
        udp_socket.close();
        log::info!("connecting to tcp_socket");
        let mut socket = TcpSocket::new(
            stack,
            unsafe { &mut *core::ptr::addr_of_mut!(RX_BUFFER) },
            unsafe { &mut *core::ptr::addr_of_mut!(TX_BUFFER) },
        );

        let addr: IpAddress = IpAddress::v4(192, 168, 178, last_addr_byte[0]);
        println!("checking addr: {addr:?}");
        match with_timeout(Duration::from_millis(500), socket.connect((addr, PORT))).await {
            Ok(Ok(())) => {
                println!("Found server at {addr:?}");
            }
            Ok(Err(e)) => {
                log::error!("{e:?}");
                continue;
            }
            Err(e) => {
                log::error!("{e:?}");
                continue;
            }
        }
        {
            let mut led = LED.lock().await;
            led.as_mut().map(|l| l.write(Some(RGB8::new(0, 128, 50))));
        }

        // send receive loop
        let mut command_buf = [0; 4];
        let mut send_buf: [u8; 1024] = [0; 1024];
        loop {
            if socket.read(&mut command_buf).await.is_err() {
                log::error!("socket.read failed");
                break;
            }
            if let Ok(command) = command_buf[..].try_into() {
                match command {
                    Command::GetIntervalms => {
                        send_buf[..2].clone_from_slice(&INTERVAL_MS.to_be_bytes());
                        log::info!("reply to Command::GetIntervalms => {:?}", &send_buf[..2]);
                        if socket.write_all(&send_buf[..2]).await.is_err() {
                            break;
                        }
                        log::info!("reply sent");
                    }
                    Command::GetVoltageBufferSize => {
                        let size = core::mem::size_of::<usize>();
                        send_buf[..size].clone_from_slice(&RING_BUFFER_SIZE.to_be_bytes());
                        log::info!("usize is {size} bytes long");
                        log::info!(
                            "reply to Command::GetVoltageBufferSize => {:?}",
                            &send_buf[..size]
                        );
                        if socket.write_all(&send_buf[..size]).await.is_err() {
                            break;
                        }
                        log::info!("reply sent");
                    }
                    Command::GetBattery1Buffer => {
                        let mut adc_readings_guard = ADC_READINGS.lock().await;
                        if let Some(adc_readings) = adc_readings_guard.as_mut() {
                            if adc_readings.ring_buffers[0]
                                .send_diff(&mut socket, &mut send_buf)
                                .await
                                .is_err()
                            {
                                log::info!("Error: send_diff");
                                break;
                            }
                        }
                    }
                    Command::GetBatteryPackBuffer => {
                        let mut adc_readings_guard = ADC_READINGS.lock().await;
                        if let Some(adc_readings) = adc_readings_guard.as_mut() {
                            if adc_readings.ring_buffers[1]
                                .send_diff(&mut socket, &mut send_buf)
                                .await
                                .is_err()
                            {
                                break;
                            }
                        }
                    }
                    Command::GetPVBuffer => {
                        let mut adc_readings_guard = ADC_READINGS.lock().await;
                        if let Some(adc_readings) = adc_readings_guard.as_mut() {
                            if adc_readings.ring_buffers[2]
                                .send_diff(&mut socket, &mut send_buf)
                                .await
                                .is_err()
                            {
                                break;
                            }
                        }
                    }
                    Command::RetransmitBuffers => {
                        let mut adc_readings_guard = ADC_READINGS.lock().await;
                        if let Some(adc_readings) = adc_readings_guard.as_mut() {
                            adc_readings.ring_buffers[0].retransmit_whole_buffer_on_next_transmit();
                            adc_readings.ring_buffers[1].retransmit_whole_buffer_on_next_transmit();
                            adc_readings.ring_buffers[2].retransmit_whole_buffer_on_next_transmit();
                        }
                    }
                    Command::ModbusGetHoldings {
                        register_address,
                        size,
                    } => {
                        let mut modbus = MAX485_MODBUS.lock().await;
                        if let Some(modbus) = modbus.as_mut() {
                            log::info!("trying to get holding values for register_address: {:?}, and size: {}", register_address, size);
                            if let Ok(Ok(values)) = with_timeout(
                                Duration::from_millis(100),
                                modbus.get_holdings(register_address, size),
                            )
                            .await
                            {
                                let bytes: Vec<u8, 256> =
                                    values.iter().flat_map(|val| val.to_be_bytes()).collect();
                                log::info!("holding values: {:?}", bytes);
                                if socket.write_all(bytes.as_slice()).await.is_err() {
                                    break;
                                }
                            } else {
                                log::error!("modbus error => sending empty buffer");
                                for _ in 0..(size * 2) {
                                    if socket.write(&[0]).await.is_err() {
                                        break;
                                    }
                                }
                            }
                        } else {
                            log::error!("no modbus device");
                        }
                    }
                    Command::ModbusGetInputRegisters {
                        register_address,
                        size,
                    } => {
                        let mut modbus = MAX485_MODBUS.lock().await;
                        if let Some(modbus) = modbus.as_mut() {
                            log::info!("trying to get holding values for register_address: {:?}, and size: {}", register_address, size);
                            if let Ok(Ok(values)) = with_timeout(
                                Duration::from_millis(100),
                                modbus.get_input_registers(register_address, size),
                            )
                            .await
                            {
                                let bytes: Vec<u8, 256> =
                                    values.iter().flat_map(|val| val.to_be_bytes()).collect();
                                log::info!("holding values: {:?}", bytes);
                                if socket.write_all(bytes.as_slice()).await.is_err() {
                                    break;
                                }
                            } else {
                                log::error!("modbus error => sending empty buffer");
                                for _ in 0..(size * 2) {
                                    if socket.write(&[0]).await.is_err() {
                                        break;
                                    }
                                }
                            }
                        } else {
                            log::error!("no modbus device");
                        }
                    }
                }
            } else {
                log::info!("could not recognize command");
            }
        }
        log::info!("tcp socket error => reconnecting with new tcp socket");
    }
}
