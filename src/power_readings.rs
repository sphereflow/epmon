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
        let mut power_pv_acc = 0;
        let mut power_inverter_input_acc = 0;
        let mut power_inverter_output_acc = 0;
        // number of successfully read values
        let mut n_success = [0; 3];
        let mut interval_ticker =
            Ticker::every(Duration::from_millis(POWER_INTERVAL_MS as u64 / 10));
        for _ in 0..10 {
            let register_tracer = {
                let mut modbus = modbus_mutex.lock().await;
                with_timeout(
                    Duration::from_millis(MODBUS_TIMEOUT_MS),
                    modbus.get_input_registers(Device::Tracer, 0x3102, 2),
                )
                .await
            };
            let register_inverter_input = {
                let mut modbus = modbus_mutex.lock().await;
                with_timeout(
                    Duration::from_millis(MODBUS_TIMEOUT_MS),
                    modbus.get_input_registers(Device::Inverter, 0x310A, 2),
                )
                .await
            };
            let register_inverter_output = {
                let mut modbus = modbus_mutex.lock().await;
                with_timeout(
                    Duration::from_millis(MODBUS_TIMEOUT_MS),
                    modbus.get_input_registers(Device::Inverter, 0x310E, 2),
                )
                .await
            };
            if let Ok(Ok(values)) = register_tracer {
                let power = (values[0] as u32) + ((values[1] as u32) << 16);
                n_success[0] += 1;
                power_pv_acc += power;
            } else {
                let mut last_error = last_error_mutex.lock().await;
                *last_error = NetLog::from_timeout_modbus_result(&register_tracer);
                log::error!("aquire_power_readings_task: timeout or modbus error");
            }
            if let Ok(Ok(values)) = register_inverter_input {
                let power = (values[0] as u32) + ((values[1] as u32) << 16);
                n_success[1] += 1;
                power_inverter_input_acc += power;
            } else {
                let mut last_error = last_error_mutex.lock().await;
                *last_error = NetLog::from_timeout_modbus_result(&register_inverter_input);
                log::error!("aquire_power_readings_task: timeout or modbus error");
            }
            if let Ok(Ok(values)) = register_inverter_output {
                let power = (values[0] as u32) + ((values[1] as u32) << 16);
                n_success[2] += 1;
                power_inverter_output_acc += power;
            } else {
                let mut last_error = last_error_mutex.lock().await;
                *last_error = NetLog::from_timeout_modbus_result(&register_inverter_output);
                log::error!("aquire_power_readings_task: timeout or modbus error");
            }
            interval_ticker.next().await;
        }
        {
            if let Some(power_readings) = (*POWER_READINGS.lock().await).as_mut() {
                if n_success[0] != 0 {
                    power_readings.push_value(0, (power_pv_acc / (100 * n_success[0])) as u16);
                } else {
                    power_readings.push_value(0, 0);
                }
                if n_success[1] != 0 {
                    power_readings
                        .push_value(1, (power_inverter_input_acc / (100 * n_success[1])) as u16);
                } else {
                    power_readings.push_value(1, 0);
                }
                if n_success[2] != 0 {
                    power_readings
                        .push_value(2, (power_inverter_output_acc / (100 * n_success[2])) as u16);
                } else {
                    power_readings.push_value(2, 0);
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
