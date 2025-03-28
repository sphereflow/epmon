#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(impl_trait_in_assoc_type)]

use command::{BufferType, Command, COMMAND_SIZE};
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpAddress, IpListenEndpoint, Runner, Stack, StackResources};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{with_timeout, Duration, Ticker, Timer};
use embedded_io_async::*;
use esp_alloc::{self as _, HeapStats};
use esp_backtrace as _;
use esp_hal::analog::adc::{Adc, AdcCalScheme, AdcChannel, AdcConfig, AdcPin, Attenuation};
use esp_hal::gpio::{GpioPin, Level, Output, OutputConfig};
use esp_hal::peripherals::{ADC1, RADIO_CLK, RNG, TIMG0, WIFI};
use esp_hal::rmt::Rmt;
use esp_hal::rng::Rng;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::Uart;
use esp_hal::{Async, Blocking, Config};
use esp_hal_embassy::main;
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use esp_println::println;
use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiState};
use esp_wifi::EspWifiController;
use heapless::Vec;
use max485::Max485Modbus;
use ringbuffer::RingBuffer;
use smart_leds::colors::*;
use smart_leds::{SmartLedsWrite, RGB8};
use static_cell::StaticCell;

pub mod command;
pub mod max485;
pub mod ringbuffer;
pub mod string_logger;

static ADC_READINGS: Mutex<CriticalSectionRawMutex, Option<AdcReadings>> = Mutex::new(None);
static POWER_READINGS: Mutex<CriticalSectionRawMutex, Option<PowerReadings>> = Mutex::new(None);

const SSID: &str = env!("WIFI_SSID");
const PASSWORD: &str = env!("WIFI_PASS");
const PORT: u16 = 8900;
const RING_BUFFER_SIZE: usize = 12000;
const VOLTAGE_INTERVAL_MS: u16 = 150;
const POWER_INTERVAL_MS: u16 = 10000;
const RX_BUFFER_SIZE: usize = 1024;
const TX_BUFFER_SIZE: usize = 1024;
// static buffers to not need a huge task-arena
static mut RX_BUFFER: [u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE];
static mut TX_BUFFER: [u8; TX_BUFFER_SIZE] = [0; TX_BUFFER_SIZE];

type LedT = SmartLedsAdapter<esp_hal::rmt::Channel<Blocking, 0>, 25>;
type LedMutex = Mutex<CriticalSectionRawMutex, LedT>;
type ModbusMutex = Mutex<NoopRawMutex, Max485Modbus<'static>>;

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
    esp_alloc::heap_allocator!(size: 72 * 1024);
    esp_println::logger::init_logger_from_env();
    // string_logger::init_string_logger();
    let peripherals = esp_hal::init(Config::default());

    // set up adc
    let mut adc_config = AdcConfig::new();
    let adc_pin0 =
        adc_config.enable_pin_with_cal::<_, AdcCal>(peripherals.GPIO4, Attenuation::_0dB);
    let adc_pin1 =
        adc_config.enable_pin_with_cal::<_, AdcCal>(peripherals.GPIO5, Attenuation::_0dB);
    let adc_pin2 =
        adc_config.enable_pin_with_cal::<_, AdcCal>(peripherals.GPIO6, Attenuation::_0dB);
    let adc1 = Adc::new(peripherals.ADC1, adc_config);
    {
        let mut adc_readings = ADC_READINGS.lock().await;
        adc_readings.replace(AdcReadings::default());
    }
    {
        let mut power_readings = POWER_READINGS.lock().await;
        power_readings.replace(PowerReadings::default());
    }

    // set up smartled
    static LED: StaticCell<LedMutex> = StaticCell::new();
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).unwrap();
    let rmt_buffer = smartLedBuffer!(1);
    let led_adapter = SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO8, rmt_buffer);
    let led_mutex = LED.init(Mutex::new(led_adapter));
    {
        let mut led = led_mutex.lock().await;
        led.write(Some(RGB8::new(130, 0, 0))).ok();
    }

    static MODBUS: StaticCell<ModbusMutex> = StaticCell::new();
    let uart_config = esp_hal::uart::Config::default();
    if let Ok(uart_peripheral) = Uart::new(peripherals.UART1, uart_config) {
        let uart_async: Uart<'static, Async> = uart_peripheral
            .with_tx(peripherals.GPIO18)
            .with_rx(peripherals.GPIO10)
            .into_async();
        let modbus = Max485Modbus::new(
            Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default()),
            uart_async,
        );
        let modbus_mutex = MODBUS.init(Mutex::new(modbus));

        let timg0 = TimerGroup::new(peripherals.TIMG1);
        esp_hal_embassy::init(timg0.timer0);
        if let Err(err) =
            spawner.spawn(aquire_adc_readings_task(adc1, adc_pin0, adc_pin1, adc_pin2))
        {
            log::error!("could not spawn adc task");
            log::error!("{err:?}");
        }

        if let Err(err) = spawner.spawn(aquire_power_readings_task(modbus_mutex)) {
            log::error!("could not spawn power task");
            log::error!("{err:?}");
        }

        let stack = init_wifi(
            peripherals.TIMG0,
            peripherals.RNG,
            peripherals.RADIO_CLK,
            peripherals.WIFI,
            &spawner,
        )
        .await;
        spawner
            .spawn(network_handler(stack, modbus_mutex, led_mutex))
            .expect("could not spawn network_handler");
    }

    loop {
        // run_tests().await;
        let stats = esp_alloc::HEAP.stats();
        println!("{stats}");
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
    mut adc1: Adc<'static, ADC1, Blocking>,
    mut pin0: PIN0,
    mut pin1: PIN1,
    mut pin2: PIN2,
) {
    let mut r0;
    let mut r1;
    let mut r2;
    let mut sub_ticker = Ticker::every(Duration::from_millis(VOLTAGE_INTERVAL_MS as u64 / 30));
    loop {
        let mut r0_acc = 0;
        let mut r1_acc = 0;
        let mut r2_acc = 0;
        // accumulate values
        for _ in 0..10 {
            r0_acc += adc1.read_adc(&mut pin0).await;
            sub_ticker.next().await;
            r1_acc += adc1.read_adc(&mut pin1).await;
            sub_ticker.next().await;
            r2_acc += adc1.read_adc(&mut pin2).await;
            sub_ticker.next().await;
        }
        // average them out
        r0 = r0_acc / 10;
        r1 = r1_acc / 10;
        r2 = r2_acc / 10;
        {
            if let Some(adc_readings) = (*ADC_READINGS.lock().await).as_mut() {
                adc_readings.push_value(0_usize, r0);
                adc_readings.push_value(1_usize, r1);
                adc_readings.push_value(2_usize, r2);
            }
        }
    }
}

trait ReadAdc {
    type ADC: esp_hal::analog::adc::RegisterAccess;
    async fn read_adc<const GPIO_NUM: u8>(
        &mut self,
        pin: &mut AdcPin<GpioPin<GPIO_NUM>, Self::ADC, AdcCal>,
    ) -> u16
    where
        GpioPin<GPIO_NUM>: AdcChannel;
}

impl<'a, ADC: esp_hal::analog::adc::RegisterAccess> ReadAdc for Adc<'a, ADC, Blocking>
where
    AdcCal: AdcCalScheme<ADC>,
{
    type ADC = ADC;
    async fn read_adc<const GPIO_NUM: u8>(
        &mut self,
        pin: &mut AdcPin<GpioPin<GPIO_NUM>, ADC, AdcCal>,
    ) -> u16
    where
        GpioPin<GPIO_NUM>: AdcChannel,
    {
        loop {
            if let Ok(val) = self.read_oneshot(pin) {
                return val;
            }
            Timer::after_micros(100).await;
        }
    }
}

#[embassy_executor::task]
async fn aquire_power_readings_task(modbus_mutex: &'static ModbusMutex) {
    loop {
        let mut power_pv_acc = 0;
        let mut interval_ticker =
            Ticker::every(Duration::from_millis(POWER_INTERVAL_MS as u64 / 10));
        for _ in 0..10 {
            let register = {
                let mut modbus = modbus_mutex.lock().await;
                with_timeout(
                    Duration::from_millis(100),
                    modbus.get_input_registers(0x3102, 2),
                )
                .await
            };
            if let Ok(Ok(values)) = register {
                let power = (values[0] as u32) + ((values[1] as u32) << 16);
                power_pv_acc += power;
            } else {
                log::error!("aquire_power_readings_task: timeout or modbus error");
            }
            interval_ticker.next().await;
        }
        {
            if let Some(power_readings) = (*POWER_READINGS.lock().await).as_mut() {
                power_readings.push_value(0, (power_pv_acc / 1000) as u16);
            }
        }
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

#[derive(Default, Clone, Debug)]
struct PowerReadings {
    pub ring_buffers: [RingBuffer<RING_BUFFER_SIZE>; 2],
}

impl PowerReadings {
    fn push_value(&mut self, ix: usize, val: u16) {
        self.ring_buffers[ix].push(val);
    }
}

async fn init_wifi(
    timg0: TIMG0,
    rng: RNG,
    radio_clk: RADIO_CLK,
    wifi: WIFI,
    spawner: &Spawner,
) -> Stack<'static> {
    let init = &*mk_static!(
        EspWifiController<'static>,
        esp_wifi::init(TimerGroup::new(timg0).timer0, Rng::new(rng), radio_clk).unwrap()
    );

    let (wifi_controller, interfaces) = esp_wifi::wifi::new(init, wifi).unwrap();
    let wifi_interface = interfaces.sta;

    let config = embassy_net::Config::dhcpv4(Default::default());

    let seed = 1234; // very random, very secure seed

    // Init network stack
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );
    spawner.spawn(connection(wifi_controller)).ok();
    spawner.spawn(net_task(runner)).ok();
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
    println!("Device capabilities: {:?}", controller.capabilities());
    loop {
        if esp_wifi::wifi::wifi_state() == WifiState::StaConnected {
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
            controller.start().unwrap();
            println!("Wifi started!");
        }
        println!("About to connect...");

        match controller.connect() {
            Ok(_) => println!("Wifi connected!"),
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
                Timer::after_secs(5).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut stack: Runner<'static, WifiDevice<'static>>) {
    stack.run().await
}

#[embassy_executor::task]
async fn network_handler(
    stack: Stack<'static>,
    modbus_mutex: &'static ModbusMutex,
    led_mutex: &'static LedMutex,
) {
    let mut last_addr_byte = [0];
    let mut rx_buffer = [1];
    let mut rx_meta = [PacketMetadata::EMPTY];
    // connect / reconnect loop
    loop {
        change_led_color(RGB8::new(0, 0, 50), led_mutex).await;
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
        change_led_color(RGB8::new(0, 128, 50), led_mutex).await;

        // send receive loop
        let mut command_buf = [0; COMMAND_SIZE];
        let mut send_buf: [u8; 1024] = [0; 1024];
        loop {
            let send_receive_loop_result = with_timeout(
                Duration::from_secs(5),
                send_receive_loop(&mut socket, &mut command_buf, &mut send_buf, modbus_mutex),
            )
            .await;
            match send_receive_loop_result {
                Ok(Ok(())) => {}
                Err(e) => {
                    log::error!("Send Receive loop timeout: {:?}", e);
                    break;
                }
                Ok(Err(e)) => {
                    log::error!("Send Receive loop error: {:?}", e);
                    break;
                }
            }
        }
        log::error!("tcp socket error => reconnecting with new tcp socket");
    }
}

async fn send_receive_loop<'a>(
    socket: &mut TcpSocket<'a>,
    command_buf: &mut [u8],
    send_buf: &mut [u8],
    modbus_mutex: &'static ModbusMutex,
) -> Result<(), embassy_net::tcp::Error> {
    socket.read(command_buf).await?;
    if let Ok(command) = command_buf[..].try_into() {
        log::info!("received command: {:?}", command);
        match command {
            Command::GetVoltageIntervalms => {
                send_buf[..2].clone_from_slice(&VOLTAGE_INTERVAL_MS.to_be_bytes());
                log::info!("reply to Command::GetIntervalms => {:?}", &send_buf[..2]);
                socket.write_all(&send_buf[..2]).await?;
                log::info!("reply sent");
            }
            Command::GetPowerIntervalms => {
                send_buf[..2].clone_from_slice(&POWER_INTERVAL_MS.to_be_bytes());
                log::info!("reply to Command::GetIntervalms => {:?}", &send_buf[..2]);
                socket.write_all(&send_buf[..2]).await?;
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
                socket.write_all(&send_buf[..size]).await?;
                log::info!("reply sent");
            }
            Command::GetBuffer(BufferType::Battery1Voltage) => {
                if let Some(adc_readings) = (*ADC_READINGS.lock().await).as_mut() {
                    adc_readings.ring_buffers[0]
                        .send_diff(socket, send_buf)
                        .await?;
                }
            }
            Command::GetBuffer(BufferType::BatteryPackVoltage) => {
                if let Some(adc_readings) = (*ADC_READINGS.lock().await).as_mut() {
                    adc_readings.ring_buffers[1]
                        .send_diff(socket, send_buf)
                        .await?;
                }
            }
            Command::GetBuffer(BufferType::PVVoltage) => {
                if let Some(adc_readings) = (*ADC_READINGS.lock().await).as_mut() {
                    adc_readings.ring_buffers[2]
                        .send_diff(socket, send_buf)
                        .await?;
                }
            }
            Command::GetBuffer(BufferType::PVPower) => {
                if let Some(power_readings) = (*POWER_READINGS.lock().await).as_mut() {
                    power_readings.ring_buffers[0]
                        .send_diff(socket, send_buf)
                        .await?;
                }
            }
            Command::GetBuffer(BufferType::InverterPower) => {
                if let Some(power_readings) = (*POWER_READINGS.lock().await).as_mut() {
                    power_readings.ring_buffers[1]
                        .send_diff(socket, send_buf)
                        .await?;
                }
            }
            Command::RetransmitBuffers => {
                if let Some(adc_readings) = (*ADC_READINGS.lock().await).as_mut() {
                    adc_readings.ring_buffers[0].retransmit_whole_buffer_on_next_transmit();
                    adc_readings.ring_buffers[1].retransmit_whole_buffer_on_next_transmit();
                    adc_readings.ring_buffers[2].retransmit_whole_buffer_on_next_transmit();
                }
                if let Some(power_readings) = (*POWER_READINGS.lock().await).as_mut() {
                    power_readings.ring_buffers[0].retransmit_whole_buffer_on_next_transmit();
                    power_readings.ring_buffers[1].retransmit_whole_buffer_on_next_transmit();
                }
            }
            Command::ModbusGetHoldings {
                register_address,
                size,
            } => {
                let register = {
                    let mut modbus = modbus_mutex.lock().await;
                    log::info!(
                        "trying to get holding values for register_address: {:?}, and size: {}",
                        register_address,
                        size
                    );
                    with_timeout(
                        Duration::from_millis(100),
                        modbus.get_holdings(register_address, size),
                    )
                    .await
                };
                if let Ok(Ok(values)) = register {
                    let bytes: Vec<u8, 256> =
                        values.iter().flat_map(|val| val.to_be_bytes()).collect();
                    log::info!("holding values: {:?}", bytes);
                    socket.write_all(bytes.as_slice()).await?;
                } else {
                    log::error!("modbus error => sending empty buffer");
                    for _ in 0..(size * 2) {
                        socket.write(&[0]).await?;
                    }
                }
            }
            Command::ModbusGetInputRegisters {
                register_address,
                size,
            } => {
                let register = {
                    let mut modbus = modbus_mutex.lock().await;
                    log::info!(
                    "trying to get input register values for register_address: {:?}, and size: {}",
                    register_address,
                    size
                );
                    with_timeout(
                        Duration::from_millis(100),
                        modbus.get_input_registers(register_address, size),
                    )
                    .await
                };
                if let Ok(Ok(values)) = register {
                    let bytes: Vec<u8, 256> =
                        values.iter().flat_map(|val| val.to_be_bytes()).collect();
                    log::info!("register values: {:?}", bytes);
                    socket.write_all(bytes.as_slice()).await?;
                } else {
                    log::error!("modbus error => sending empty buffer");
                    for _ in 0..(size * 2) {
                        socket.write(&[0]).await?;
                    }
                }
            }
            Command::ModbusSetHoldings {
                register_address,
                new_holding_values,
            } => {
                let register = {
                    let mut modbus = modbus_mutex.lock().await;
                    modbus
                        .set_holdings(register_address, &new_holding_values)
                        .await
                };
                if register.is_err() {
                    log::error!("failed to set holding values");
                }
            }
            Command::GetLastLogMessage => {
                if let Some(message) = string_logger::LOG.try_take() {
                    socket.write_all(message.as_bytes()).await?;
                } else {
                    socket.write_all("Could not get log".as_bytes()).await?;
                }
            }
        }
    } else {
        log::error!("could not recognize command");
    }
    Ok(())
}

pub async fn change_led_color(color: RGB8, led_mutex: &'static LedMutex) {
    let mut led = led_mutex.lock().await;
    led.write(Some(color)).ok();
}

#[allow(dead_code)]
async fn run_tests(modbus_mutex: &'static ModbusMutex, led_mutex: &'static LedMutex) {
    run_modbus_test(modbus_mutex, led_mutex).await;
    run_uart_loopback_test(modbus_mutex).await;
    run_adc_test().await;
}

#[allow(dead_code)]
async fn run_modbus_test(modbus_mutex: &'static ModbusMutex, led_mutex: &'static LedMutex) {
    let mut modbus = modbus_mutex.lock().await;
    match with_timeout(Duration::from_millis(100), modbus.test_holding()).await {
        Ok(Ok(true)) => {
            log::info!("test modbus => success");
            change_led_color(GREEN, led_mutex).await;
        }
        Ok(Ok(false)) => log::error!("test modbus => buffers are not equal"),
        Ok(Err(e)) => {
            log::error!("test modbus => modbus error: {:?}", e);
            change_led_color(ORANGE, led_mutex).await;
        }
        Err(_) => {
            log::error!("test modbus => timeout");
            change_led_color(PURPLE, led_mutex).await;
        }
    }
}

#[allow(dead_code)]
async fn run_uart_loopback_test(modbus_mutex: &'static ModbusMutex) {
    let mut modbus = modbus_mutex.lock().await;
    match with_timeout(Duration::from_millis(100), modbus.test_loopback()).await {
        Ok(Ok(true)) => log::info!("test loopback => success"),
        Ok(Ok(false)) => log::error!("test loopback => buffers are not equal"),
        _ => log::error!("test loopback => sth went wrong"),
    }
}

#[allow(dead_code)]
async fn run_adc_test() {
    let mut print_buffer: [u16; 20] = [0; 20];
    if let Some(adc_readings) = (*ADC_READINGS.lock().await).as_mut() {
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
