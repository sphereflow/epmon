use embassy_time::Duration;
use embedded_io::ReadExactError;
use embedded_svc::io::asynch::{Read, Write};
use esp_hal::{
    gpio::{GpioPin, Output},
    peripherals::UART1,
    uart::Uart,
    Async,
};
use heapless::Vec;
use rmodbus::{client::ModbusRequest, guess_response_frame_len};

pub struct Max485Modbus {
    uart: Uart<'static, UART1, Async>,
    rw_pin: Output<'static, GpioPin<2>>,
    unit_id: u8,
}

impl Max485Modbus {
    pub fn new(rw_pin: Output<'static, GpioPin<2>>, uart: Uart<'static, UART1, Async>) -> Self {
        Max485Modbus {
            rw_pin,
            uart,
            // for EPEVER Tracer AN the unit id appears to be: 1
            unit_id: 1,
        }
    }

    // a holding is a 16 bit register on the device

    // write consecutive registers
    pub async fn set_holdings(
        &mut self,
        reg_address: u16,
        register_values: &[u16],
    ) -> Result<(), Max485ModbusError> {
        let mut modbus_request = ModbusRequest::new(self.unit_id, rmodbus::ModbusProto::Rtu);
        let mut request_buffer: heapless::Vec<u8, 256> = heapless::Vec::new();
        modbus_request
            .generate_set_holdings_bulk(reg_address, register_values, &mut request_buffer)
            .map_err(|_e| Max485ModbusError {})?;
        self.write_all(&request_buffer).await?;

        // reuse the request_buffer for the response buffer
        request_buffer.clear();

        // get a response if the value was successfully set
        let mut response_buffer = request_buffer;
        let _ = response_buffer.resize(3, 0);
        self.read_exact(&mut response_buffer).await?;
        let response_frame_len =
            guess_response_frame_len(&response_buffer, rmodbus::ModbusProto::Rtu)
                .map_err(|_error| Max485ModbusError {})?;
        let _ = response_buffer
            .resize(response_frame_len as usize, 0)
            .map_err(|_error| Max485ModbusError {});
        self.read_exact(&mut response_buffer[3..]).await?;
        modbus_request
            .parse_ok(&response_buffer)
            .map_err(|_error| Max485ModbusError {})?;
        Ok(())
    }

    pub async fn get_holdings(
        &mut self,
        reg_address: u16,
        holding_count: u8,
    ) -> Result<Vec<u16, 128>, Max485ModbusError> {
        let mut modbus_request = ModbusRequest::new(self.unit_id, rmodbus::ModbusProto::Rtu);
        let mut request_buffer: Vec<u8, 256> = Vec::new();
        modbus_request
            .generate_get_holdings(reg_address, holding_count as u16, &mut request_buffer)
            .map_err(|_error| Max485ModbusError {})?;
        self.write_all(&request_buffer).await?;

        // reuse the request_buffer for the response buffer
        request_buffer.clear();

        // get a response if the value was successfully set
        let mut response_buffer = request_buffer;
        response_buffer
            .resize(3, 0)
            .map_err(|_error| Max485ModbusError {})?;
        self.read_exact(&mut response_buffer).await?;
        let response_frame_len =
            guess_response_frame_len(&response_buffer, rmodbus::ModbusProto::Rtu)
                .map_err(|_error| Max485ModbusError {})?;
        response_buffer
            .resize(response_frame_len as usize, 0)
            .map_err(|_error| Max485ModbusError {})?;
        self.read_exact(&mut response_buffer[3..]).await?;
        let mut val_array: Vec<u16, 128> = Vec::new();
        modbus_request
            .parse_u16(&response_buffer, &mut val_array)
            .map_err(|_error| Max485ModbusError {})?;
        Ok(val_array)
    }

    pub async fn get_input_registers(
        &mut self,
        reg_address: u16,
        register_count: u8,
    ) -> Result<Vec<u16, 128>, Max485ModbusError> {
        let mut modbus_request = ModbusRequest::new(self.unit_id, rmodbus::ModbusProto::Rtu);
        let mut request_buffer: Vec<u8, 256> = Vec::new();
        modbus_request
            .generate_get_inputs(reg_address, register_count as u16, &mut request_buffer)
            .map_err(|_error| Max485ModbusError {})?;
        self.write_all(&request_buffer).await?;

        // reuse the request_buffer for the response buffer
        request_buffer.clear();

        // get a response if the value was successfully set
        let mut response_buffer = request_buffer;
        response_buffer
            .resize(3, 0)
            .map_err(|_error| Max485ModbusError {})?;
        self.read_exact(&mut response_buffer).await?;
        let response_frame_len =
            guess_response_frame_len(&response_buffer, rmodbus::ModbusProto::Rtu)
                .map_err(|_error| Max485ModbusError {})?;
        response_buffer
            .resize(response_frame_len as usize, 0)
            .map_err(|_error| Max485ModbusError {})?;
        self.read_exact(&mut response_buffer[3..]).await?;
        let mut val_array: Vec<u16, 128> = Vec::new();
        modbus_request
            .parse_u16(&response_buffer, &mut val_array)
            .map_err(|_error| Max485ModbusError {})?;
        Ok(val_array)
    }
    // a coil is a single bit on the device

    pub async fn get_coils(
        &mut self,
        reg_address: u16,
        count: u16,
    ) -> Result<u8, Max485ModbusError> {
        let mut modbus_request = ModbusRequest::new(self.unit_id, rmodbus::ModbusProto::Rtu);
        let mut request_buffer: heapless::Vec<u8, 256> = heapless::Vec::new();
        modbus_request
            .generate_get_coils(reg_address, count.max(8), &mut request_buffer)
            .map_err(|_error| Max485ModbusError {})?;
        self.write_all(&request_buffer).await?;

        // reuse the request_buffer for the response buffer
        request_buffer.clear();

        // get a response if the value was successfully set
        let mut response_buffer = request_buffer;
        response_buffer
            .resize(3, 0)
            .map_err(|_error| Max485ModbusError {})?;
        self.read_exact(&mut response_buffer).await?;
        let response_frame_len =
            guess_response_frame_len(&response_buffer, rmodbus::ModbusProto::Rtu)
                .map_err(|_error| Max485ModbusError {})?;
        response_buffer
            .resize(response_frame_len as usize, 0)
            .map_err(|_error| Max485ModbusError {})?;
        self.read_exact(&mut response_buffer[3..]).await?;
        modbus_request
            .parse_ok(&response_buffer)
            .map_err(|_error| Max485ModbusError {})?;
        let byte_count = response_buffer[2];
        let val = response_buffer[3];
        if byte_count != 1 {
            return Err(Max485ModbusError {});
        }
        Ok(val)
    }
}

impl Write for Max485Modbus {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.rw_pin.set_high();
        embassy_time::block_for(Duration::from_micros(3));
        let bytes_written = self.uart.write(buf).await?;
        self.flush().await?;
        self.rw_pin.set_low();
        Ok(bytes_written)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.uart.flush().await.map_err(|e| e.into())
    }

    async fn write_all(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.rw_pin.set_high();
        embassy_time::block_for(Duration::from_micros(3));
        self.uart.write_all(buf).await?;
        self.flush().await?;
        self.rw_pin.set_low();
        Ok(())
    }
}

impl Read for Max485Modbus {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rw_pin.set_low();
        embassy_time::block_for(Duration::from_micros(3));
        self.uart.read(buf).await.map_err(|e| e.into())
    }

    async fn read_exact(
        &mut self,
        buf: &mut [u8],
    ) -> Result<(), embedded_io::ReadExactError<Self::Error>> {
        self.rw_pin.set_low();
        embassy_time::block_for(Duration::from_micros(3));
        self.uart
            .read_exact(buf)
            .await
            .map_err(|_e| ReadExactError::Other(Max485ModbusError {}))
    }
}

impl embedded_svc::io::asynch::ErrorType for Max485Modbus {
    type Error = Max485ModbusError;
}

#[derive(Debug)]
pub struct Max485ModbusError {}

impl embedded_svc::io::Error for Max485ModbusError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl From<esp_hal::uart::Error> for Max485ModbusError {
    fn from(_value: esp_hal::uart::Error) -> Self {
        Max485ModbusError {}
    }
}

impl<E> From<embedded_io::ReadExactError<E>> for Max485ModbusError {
    fn from(_value: embedded_io::ReadExactError<E>) -> Self {
        Max485ModbusError {}
    }
}
