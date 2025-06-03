use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources};
use embassy_time::Timer;
use esp_hal::{
    peripherals::{RADIO_CLK, RNG, TIMG0, WIFI},
    rng::Rng,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_wifi::{
    wifi::{WifiController, WifiDevice, WifiEvent, WifiState},
    EspWifiController,
};

const SSID: &str = env!("WIFI_SSID");
const PASSWORD: &str = env!("WIFI_PASS");

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

pub async fn init_wifi(
    timg0: TIMG0<'static>,
    rng: RNG<'_>,
    radio_clk: RADIO_CLK<'static>,
    wifi: WIFI<'static>,
    spawner: &Spawner,
) -> Stack<'static> {
    let init = &*mk_static!(
        EspWifiController<'static>,
        esp_wifi::init(TimerGroup::new(timg0).timer0, Rng::new(rng), radio_clk).unwrap()
    );

    let (mut wifi_controller, interfaces) = esp_wifi::wifi::new(init, wifi).unwrap();
    wifi_controller
        .set_power_saving(esp_wifi::config::PowerSaveMode::None)
        .expect("wifi_controller.set_power_saving(...) failed");
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
                    ssid: SSID.into(),
                    password: PASSWORD.into(),
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
