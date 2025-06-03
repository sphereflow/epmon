use crate::{
    max485::Device, net_log::NetLog, ringbuffer::RingBuffer, ModbusMutex, NetLogMutex,
    MODBUS_TIMEOUT_MS, POWER_INTERVAL_MS, POWER_READINGS, RING_BUFFER_SIZE,
};
use embassy_time::{with_timeout, Duration, Ticker};

#[embassy_executor::task]
pub async fn aquire_power_readings_task(
    modbus_mutex: &'static ModbusMutex,
    last_error_mutex: &'static NetLogMutex,
) {
    loop {
        let mut power_accs = [0; 3];
        // number of successfully read values
        let mut n_success = [0; 3];
        let mut interval_ticker =
            Ticker::every(Duration::from_millis(POWER_INTERVAL_MS as u64 / 10));
        let register_device_addresses = [
            (Device::Tracer, 0x3102),
            (Device::Inverter, 0x310A),
            (Device::Inverter, 0x310E),
        ];
        for _ in 0..10 {
            for (i, (device, address)) in register_device_addresses.iter().enumerate() {
                let register = {
                    let mut modbus = modbus_mutex.lock().await;
                    with_timeout(
                        Duration::from_millis(MODBUS_TIMEOUT_MS),
                        modbus.get_input_registers(*device, *address, 2),
                    )
                    .await
                };
                if let Ok(Ok(values)) = register {
                    let power = (values[0] as u32) + ((values[1] as u32) << 16);
                    n_success[i] += 1;
                    power_accs[i] += power;
                } else {
                    let mut last_error = last_error_mutex.lock().await;
                    *last_error = NetLog::from_timeout_modbus_result(&register);
                    log::error!("aquire_power_readings_task: timeout or modbus error");
                }
            }
            interval_ticker.next().await;
        }
        {
            if let Some(power_readings) = (*POWER_READINGS.lock().await).as_mut() {
                for (ix, successes) in n_success.iter().enumerate() {
                    if *successes != 0 {
                        power_readings.push_value(ix, (power_accs[ix] / (100 * successes)) as u16);
                    } else {
                        power_readings.push_value(ix, 0);
                    }
                }
            }
        }
    }
}

#[derive(Default, Clone, Debug)]
pub struct PowerReadings {
    pub ring_buffers: [RingBuffer<RING_BUFFER_SIZE>; 3],
}

impl PowerReadings {
    fn push_value(&mut self, ix: usize, val: u16) {
        self.ring_buffers[ix].push(val);
    }
}
