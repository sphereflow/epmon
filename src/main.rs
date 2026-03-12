#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(impl_trait_in_assoc_type)]

use crate::smartled::SmartLedsAdapter;
use adc_readings::{AdcCal, AdcReadings, aquire_adc_readings_task};
use command::{BufferType, COMMAND_SIZE, Command};
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpAddress, IpListenEndpoint, Stack};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer, with_timeout};
use embedded_io_async::*;
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::rmt::Rmt;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::Uart;
use esp_hal::{Async, Config};
use esp_println::println;
use esp_rtos::main;
use heapless::Vec;
use max485::{Device, Max485Modbus};
use net_log::NetLog;
use power_readings::PowerReadings;
use smart_leds::{RGB8, SmartLedsWrite};
use static_cell::StaticCell;

pub mod adc_readings;
pub mod command;
pub mod init_network;
pub mod max485;
pub mod net_log;
pub mod power_readings;
pub mod ringbuffer;
pub mod smartled;
pub mod string_logger;
pub mod tests;

static ADC_READINGS: Mutex<CriticalSectionRawMutex, Option<AdcReadings>> = Mutex::new(None);
static POWER_READINGS: Mutex<CriticalSectionRawMutex, Option<PowerReadings>> = Mutex::new(None);

const PORT: u16 = 8900;
const RING_BUFFER_SIZE: usize = 12000;
const VOLTAGE_INTERVAL_MS: u16 = 150;
const POWER_INTERVAL_MS: u16 = 10000;
const MODBUS_TIMEOUT_MS: u64 = 200;
const RX_BUFFER_SIZE: usize = 1024;
const TX_BUFFER_SIZE: usize = 1024;
// static buffers to not need a huge task-arena
static mut RX_BUFFER: [u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE];
static mut TX_BUFFER: [u8; TX_BUFFER_SIZE] = [0; TX_BUFFER_SIZE];

type LedT = SmartLedsAdapter<'static, 25>;
type LedMutex = Mutex<CriticalSectionRawMutex, LedT>;
type ModbusMutex = Mutex<NoopRawMutex, Max485Modbus<'static>>;
type NetLogMutex = Mutex<NoopRawMutex, NetLog>;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[main]
async fn main(spawner: Spawner) -> ! {
    esp_alloc::heap_allocator!(size: 60 * 1024);
    esp_println::logger::init_logger_from_env();
    // string_logger::init_string_logger();
    log::info!("entered main");
    log::info!("entered main");
    let peripherals = esp_hal::init(Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    static NET_LOG: StaticCell<NetLogMutex> = StaticCell::new();
    let net_log_mutex = NET_LOG.init(Mutex::new(NetLog::new()));

    // set up adc
    let mut adc_config = AdcConfig::new();
    let adc_pin0 =
        adc_config.enable_pin_with_cal::<_, AdcCal>(peripherals.GPIO4, Attenuation::_0dB);
    let adc_pin1 =
        adc_config.enable_pin_with_cal::<_, AdcCal>(peripherals.GPIO5, Attenuation::_0dB);
    let adc_pin2 =
        adc_config.enable_pin_with_cal::<_, AdcCal>(peripherals.GPIO6, Attenuation::_11dB);
    let adc1 = Adc::new(peripherals.ADC1, adc_config).into_async();
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
    let rmt_buffer = smart_led_buffer!(1);
    let led_adapter = SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO8, rmt_buffer);
    let led_mutex = LED.init(Mutex::new(led_adapter));
    {
        let mut led = led_mutex.lock().await;
        led.write(Some(RGB8::new(130, 0, 0))).ok();
    }

    log::info!("configuring UART");
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

        if let Err(err) =
            spawner.spawn(aquire_adc_readings_task(adc1, adc_pin0, adc_pin1, adc_pin2))
        {
            log::error!("could not spawn adc task");
            log::error!("{err:?}");
        }

        if let Err(err) = spawner.spawn(power_readings::aquire_power_readings_task(
            modbus_mutex,
            net_log_mutex,
        )) {
            log::error!("could not spawn power task");
            log::error!("{err:?}");
        }

        let stack = init_network::init_wifi(peripherals.WIFI, &spawner).await;
        spawner
            .spawn(network_handler(
                stack,
                net_log_mutex,
                modbus_mutex,
                led_mutex,
            ))
            .expect("could not spawn network_handler");

        // wait for some ADC readings to come in
        // Timer::after_secs(2).await;
        // run_tests(modbus_mutex, led_mutex).await;
    }

    loop {
        let stats = esp_alloc::HEAP.stats();
        println!("{stats}");
        Timer::after_secs(60).await;
    }
}

#[embassy_executor::task]
async fn network_handler(
    stack: Stack<'static>,
    net_log_mutex: &'static NetLogMutex,
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
                send_receive_loop(
                    &mut socket,
                    &mut command_buf,
                    &mut send_buf,
                    net_log_mutex,
                    modbus_mutex,
                ),
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
    net_log_mutex: &'static NetLogMutex,
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
            Command::GetBuffer(BufferType::InverterInputPower) => {
                if let Some(power_readings) = (*POWER_READINGS.lock().await).as_mut() {
                    power_readings.ring_buffers[1]
                        .send_diff(socket, send_buf)
                        .await?;
                }
            }
            Command::GetBuffer(BufferType::InverterOutputPower) => {
                if let Some(power_readings) = (*POWER_READINGS.lock().await).as_mut() {
                    power_readings.ring_buffers[2]
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
                    power_readings.ring_buffers[2].retransmit_whole_buffer_on_next_transmit();
                }
            }
            Command::ModbusTracerGetHoldings {
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
                        Duration::from_millis(MODBUS_TIMEOUT_MS),
                        modbus.get_holdings(Device::Tracer, register_address, size),
                    )
                    .await
                };
                let mut net_log = net_log_mutex.lock().await;
                *net_log = NetLog::from_timeout_modbus_result(&register);
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
            Command::ModbusTracerGetInputRegisters {
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
                        Duration::from_millis(MODBUS_TIMEOUT_MS),
                        modbus.get_input_registers(Device::Tracer, register_address, size),
                    )
                    .await
                };
                let mut net_log = net_log_mutex.lock().await;
                *net_log = NetLog::from_timeout_modbus_result(&register);
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
            Command::ModbusTracerSetHoldings {
                register_address,
                new_holding_values,
            } => {
                let register = {
                    let mut modbus = modbus_mutex.lock().await;
                    modbus
                        .set_holdings(Device::Tracer, register_address, &new_holding_values)
                        .await
                };
                let mut net_log = net_log_mutex.lock().await;
                *net_log = NetLog::from_timeout_modbus_result(&Ok(register));
                if register.is_err() {
                    log::error!("failed to set holding values");
                }
            }
            Command::ModbusInverterGetHoldings {
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
                        Duration::from_millis(MODBUS_TIMEOUT_MS),
                        modbus.get_holdings(Device::Inverter, register_address, size),
                    )
                    .await
                };
                let mut net_log = net_log_mutex.lock().await;
                *net_log = NetLog::from_timeout_modbus_result(&register);
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
            Command::ModbusInverterGetInputRegisters {
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
                        Duration::from_millis(MODBUS_TIMEOUT_MS),
                        modbus.get_input_registers(Device::Inverter, register_address, size),
                    )
                    .await
                };
                let mut net_log = net_log_mutex.lock().await;
                *net_log = NetLog::from_timeout_modbus_result(&register);
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
            Command::ModbusInverterSetHoldings {
                register_address,
                new_holding_values,
            } => {
                let register = {
                    let mut modbus = modbus_mutex.lock().await;
                    modbus
                        .set_holdings(Device::Inverter, register_address, &new_holding_values)
                        .await
                };
                let mut net_log = net_log_mutex.lock().await;
                *net_log = NetLog::from_timeout_modbus_result(&Ok(register));
                if register.is_err() {
                    log::error!("failed to set holding values");
                }
            }
            Command::GetLastLogMessage => {
                // if let Some(message) = string_logger::LOG.try_take() {
                //     socket.write_all(message.as_bytes()).await?;
                // } else {
                //     socket.write_all("Could not get log".as_bytes()).await?;
                // }
                let mut net_log = net_log_mutex.lock().await;
                let modbus = modbus_mutex.lock().await;
                net_log.append_log(&modbus.net_log);
                net_log.send(socket, send_buf).await?;
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
