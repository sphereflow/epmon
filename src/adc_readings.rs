use embassy_time::{Duration, Ticker, Timer};
use esp_hal::{
    analog::adc::{Adc, AdcCalScheme, AdcChannel, AdcPin, RegisterAccess},
    gpio::GpioPin,
    peripherals::ADC1,
    Blocking,
};

use crate::{ringbuffer::RingBuffer, ADC_READINGS, RING_BUFFER_SIZE, VOLTAGE_INTERVAL_MS};

pub type AdcCal = esp_hal::analog::adc::AdcCalBasic<ADC1>;
// pub type AdcCal = esp_hal::analog::adc::AdcCalLine<ADC1>;
// pub type AdcCal = esp_hal::analog::adc::AdcCalCurve<ADC1>;
type PIN0 = AdcPin<GpioPin<4>, ADC1, AdcCal>;
type PIN1 = AdcPin<GpioPin<5>, ADC1, AdcCal>;
type PIN2 = AdcPin<GpioPin<6>, ADC1, AdcCal>;

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

impl<'a, ADC: RegisterAccess> ReadAdc for Adc<'a, ADC, Blocking>
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
