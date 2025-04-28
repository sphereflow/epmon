use embassy_time::{with_timeout, Duration};
use smart_leds::colors::*;

use crate::{change_led_color, LedMutex, ModbusMutex, ADC_READINGS, RING_BUFFER_SIZE};

#[allow(dead_code)]
pub async fn run_tests(modbus_mutex: &'static ModbusMutex, led_mutex: &'static LedMutex) {
    run_modbus_test(modbus_mutex, led_mutex).await;
    run_uart_loopback_test(modbus_mutex).await;
    run_adc_test().await;
}

#[allow(dead_code)]
pub async fn run_modbus_test(modbus_mutex: &'static ModbusMutex, led_mutex: &'static LedMutex) {
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
pub async fn run_uart_loopback_test(modbus_mutex: &'static ModbusMutex) {
    let mut modbus = modbus_mutex.lock().await;
    match with_timeout(Duration::from_millis(100), modbus.test_loopback()).await {
        Ok(Ok(true)) => log::info!("test loopback => success"),
        Ok(Ok(false)) => log::error!("test loopback => buffers are not equal"),
        _ => log::error!("test loopback => sth went wrong"),
    }
}

#[allow(dead_code)]
pub async fn run_adc_test() {
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
