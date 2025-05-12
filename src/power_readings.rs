use crate::{
    net_log::NetLog, ringbuffer::RingBuffer, ModbusMutex, NetLogMutex, MODBUS_TIMEOUT_MS,
    POWER_INTERVAL_MS, POWER_READINGS, RING_BUFFER_SIZE,
};
use embassy_time::{with_timeout, Duration, Ticker};

#[embassy_executor::task]
pub async fn aquire_power_readings_task(
    modbus_mutex: &'static ModbusMutex,
    last_error_mutex: &'static NetLogMutex,
) {
    loop {
        let mut power_pv_acc = 0;
        let mut interval_ticker =
            Ticker::every(Duration::from_millis(POWER_INTERVAL_MS as u64 / 10));
        for _ in 0..10 {
            let register = {
                let mut modbus = modbus_mutex.lock().await;
                with_timeout(
                    Duration::from_millis(MODBUS_TIMEOUT_MS),
                    modbus.get_input_registers(0x3102, 2),
                )
                .await
            };
            if let Ok(Ok(values)) = register {
                let power = (values[0] as u32) + ((values[1] as u32) << 16);
                power_pv_acc += power;
            } else {
                let mut last_error = last_error_mutex.lock().await;
                *last_error = NetLog::from_timeout_modbus_result(&register);
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

#[derive(Default, Clone, Debug)]
pub struct PowerReadings {
    pub ring_buffers: [RingBuffer<RING_BUFFER_SIZE>; 2],
}

impl PowerReadings {
    fn push_value(&mut self, ix: usize, val: u16) {
        self.ring_buffers[ix].push(val);
    }
}
