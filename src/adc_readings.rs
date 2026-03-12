use embassy_time::{Duration, Ticker};
use esp_hal::{
    Async,
    analog::adc::{Adc, AdcPin},
    peripherals::{ADC1, GPIO4, GPIO5, GPIO6},
};

use crate::{ADC_READINGS, RING_BUFFER_SIZE, VOLTAGE_INTERVAL_MS, ringbuffer::RingBuffer};

pub type AdcCal = esp_hal::analog::adc::AdcCalBasic<ADC1<'static>>;
// pub type AdcCal = esp_hal::analog::adc::AdcCalLine<ADC1>;
// pub type AdcCal = esp_hal::analog::adc::AdcCalCurve<ADC1>;
type PIN0 = AdcPin<GPIO4<'static>, ADC1<'static>, AdcCal>;
type PIN1 = AdcPin<GPIO5<'static>, ADC1<'static>, AdcCal>;
type PIN2 = AdcPin<GPIO6<'static>, ADC1<'static>, AdcCal>;

#[derive(Default, Debug)]
pub struct AdcReadings {
    pub ring_buffers: [RingBuffer<RING_BUFFER_SIZE>; 3],
}

impl AdcReadings {
    fn push_value(&mut self, ix: usize, val: u16) {
        self.ring_buffers[ix].push(val);
    }
}

#[embassy_executor::task]
pub async fn aquire_adc_readings_task(
    mut adc1: Adc<'static, ADC1<'static>, Async>,
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
            r0_acc += adc1.read_oneshot(&mut pin0).await;
            sub_ticker.next().await;
            r1_acc += adc1.read_oneshot(&mut pin1).await;
            sub_ticker.next().await;
            r2_acc += adc1.read_oneshot(&mut pin2).await;
            sub_ticker.next().await;
        }
        // average them out
        r0 = r0_acc / 10;
        r1 = r1_acc / 10;
        // PV Voltage attenuation is 11 dB which is a factor of ~ 3.546 ( so multiply by 35 before
        // dividing by 10)
        r2 = 35 * (r2_acc as u32) / 10;
        {
            if let Some(adc_readings) = (*ADC_READINGS.lock().await).as_mut() {
                adc_readings.push_value(0_usize, r0);
                adc_readings.push_value(1_usize, r1);
                adc_readings.push_value(2_usize, r2 as u16);
            }
        }
    }
}
