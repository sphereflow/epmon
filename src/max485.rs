use crate::net_log::NetLog;
use embassy_time::Duration;
use embassy_time::Timer;
use embedded_io::ReadExactError;
use embedded_io_async::Read;
use embedded_io_async::Write;
use esp_hal::uart::IoError;
use esp_hal::Async;
use esp_hal::{gpio::Output, uart::Uart};
use heapless::String;
use heapless::Vec;
use rmodbus::{client::ModbusRequest, guess_response_frame_len};

pub struct Max485Modbus<'a> {
    uart: Uart<'a, Async>,
    rw_pin: Output<'a>,
    unit_ids: [u8; 2],
    pub net_log: NetLog,
}

impl<'a> Max485Modbus<'a> {
    pub fn new(rw_pin: Output<'static>, uart: Uart<'static, Async>) -> Self {
        Max485Modbus {
            rw_pin,
            uart,
            // for EPEVER Tracer AN the unit id appears to be: 1
            unit_ids: [1, 3],
            net_log: NetLog::new(),
        }
    }

    async fn do_request<'r>(
        &mut self,
        device: Device,
        reg_address: u16,
        request: Request<'r>,
        request_buffer: &mut heapless::Vec<u8, 256>,
    ) -> Result<ModbusRequest, Max485ModbusError> {
        self.net_log = NetLog::new();
        // wait for a short period so that a received and an immediatly afterwards transmitted package don't look like one package
        Timer::after_millis(5).await;
        // logs rx errors from previous requests and 'clears' them
        if let Err(e) = self.uart.check_for_rx_errors() {
            log::error!("Max485Modbus::do_request(...) -> RxError: {:?}", e);
            let mut err_string: String<128> = String::new();
            core::fmt::Write::write_fmt(&mut err_string, format_args!("previous RxError: {e}\n"))
                .expect("do_request: could not append to net_log");
            self.net_log.append(&err_string);
        }
        let mut modbus_request =
            ModbusRequest::new(self.unit_ids[device as usize], rmodbus::ModbusProto::Rtu);
        // read leftover bytes from last modbus transfer
        if self.uart.read_ready() {
            request_buffer.resize(256, 0)?;
            let num_bytes = self.read(request_buffer).await?;
            log::warn!("uart had leftovers from last meal! size: {}", num_bytes);
            request_buffer.clear();
            self.net_log.append("read_ready returned true\n");
        }
        match request {
            Request::SetHoldings { register_values } => {
                modbus_request.generate_set_holdings_bulk(
                    reg_address,
                    register_values,
                    request_buffer,
                )?;
                log::info!("set_holdings write request_buffer: {:?}", &request_buffer);
            }
            Request::GetHoldings { holding_count } => {
                modbus_request.generate_get_holdings(reg_address, holding_count, request_buffer)?;
            }
            Request::GetRegister { register_count } => {
                modbus_request.generate_get_inputs(reg_address, register_count, request_buffer)?;
            }
            Request::ReadCoils { coil_count } => {
                modbus_request.generate_get_coils(
                    reg_address,
                    coil_count.max(8),
                    request_buffer,
                )?;
            }
        }
        self.write_all(request_buffer).await?;
        Ok(modbus_request)
    }

    // a holding is a 16 bit register on the device

    // write consecutive registers
    pub async fn set_holdings(
        &mut self,
        device: Device,
        reg_address: u16,
        register_values: &[u16],
    ) -> Result<(), Max485ModbusError> {
        let mut request_buffer: heapless::Vec<u8, 256> = heapless::Vec::new();
        let modbus_request = self
            .do_request(
                device,
                reg_address,
                Request::SetHoldings { register_values },
                &mut request_buffer,
            )
            .await?;

        // reuse the request_buffer for the response buffer
        request_buffer.clear();

        // get a response if the value was successfully set
        let mut response_buffer = request_buffer;
        let _ = response_buffer.resize(3, 0);
        self.read_exact(&mut response_buffer).await?;
        let response_frame_len =
            guess_response_frame_len(&response_buffer, rmodbus::ModbusProto::Rtu)?;
        response_buffer.resize(response_frame_len as usize, 0)?;
        self.read_exact(&mut response_buffer[3..]).await?;
        log::info!("set_holdings response_buffer: {:?}", &response_buffer);
        modbus_request.parse_ok(&response_buffer)?;
        Ok(())
    }

    pub async fn get_holdings(
        &mut self,
        device: Device,
        reg_address: u16,
        holding_count: u8,
    ) -> Result<Vec<u16, 128>, Max485ModbusError> {
        let mut request_buffer: Vec<u8, 256> = Vec::new();
        let modbus_request = self
            .do_request(
                device,
                reg_address,
                Request::GetHoldings {
                    holding_count: holding_count as u16,
                },
                &mut request_buffer,
            )
            .await?;
        self.net_log
            .append("Max485Modbus::get_holdings => Max485Modbus::do_request(...) successful\n");

        // reuse the request_buffer for the response buffer
        request_buffer.clear();

        // get a response if the value was successfully set
        let mut response_buffer = request_buffer;
        response_buffer.resize(3, 0)?;
        self.read_exact(&mut response_buffer).await?;
        self.net_log.append(
            "Max485Modbus::get_holdings => first part of the response frame successfully read\n",
        );
        log::info!("got response frame: {:?}", response_buffer);
        let response_frame_len =
            guess_response_frame_len(&response_buffer, rmodbus::ModbusProto::Rtu)?;
        log::info!("calculated response frame len: {}", response_frame_len);
        response_buffer.resize(response_frame_len as usize, 0)?;
        self.read_exact(&mut response_buffer[3..]).await?;
        self.net_log.append(
            "Max485Modbus::get_holdings => second part of the response frame successfully read\n",
        );
        log::info!("got response frame: {:?}", response_buffer);
        let mut val_array: Vec<u16, 128> = Vec::new();
        modbus_request.parse_u16(&response_buffer, &mut val_array)?;
        Ok(val_array)
    }

    pub async fn get_input_registers(
        &mut self,
        device: Device,
        reg_address: u16,
        register_count: u8,
    ) -> Result<Vec<u16, 128>, Max485ModbusError> {
        let mut request_buffer: Vec<u8, 256> = Vec::new();
        let modbus_request = self
            .do_request(
                device,
                reg_address,
                Request::GetRegister {
                    register_count: register_count as u16,
                },
                &mut request_buffer,
            )
            .await?;
        self.net_log.append(
            "Max485Modbus::get_input_registers => Max485Modbus::do_request(...) successful\n",
        );

        // reuse the request_buffer for the response buffer
        request_buffer.clear();

        // get a response if the value was successfully set
        let mut response_buffer = request_buffer;
        response_buffer.resize(3, 0)?;
        self.read_exact(&mut response_buffer).await?;
        self.net_log.append(
            "Max485Modbus::get_input_registers => first part of the response frame successfully read\n",
        );
        let response_frame_len =
            guess_response_frame_len(&response_buffer, rmodbus::ModbusProto::Rtu)?;
        response_buffer.resize(response_frame_len as usize, 0)?;
        self.read_exact(&mut response_buffer[3..]).await?;
        self.net_log.append(
            "Max485Modbus::get_input_registers => second part of the response frame successfully read\n",
        );
        let mut val_array: Vec<u16, 128> = Vec::new();
        modbus_request.parse_u16(&response_buffer, &mut val_array)?;
        Ok(val_array)
    }
    // a coil is a single bit on the device

    pub async fn get_coils(
        &mut self,
        device: Device,
        reg_address: u16,
        coil_count: u16,
    ) -> Result<u8, Max485ModbusError> {
        let mut request_buffer: heapless::Vec<u8, 256> = heapless::Vec::new();
        let modbus_request = self
            .do_request(
                device,
                reg_address,
                Request::ReadCoils { coil_count },
                &mut request_buffer,
            )
            .await?;

        // reuse the request_buffer for the response buffer
        request_buffer.clear();

        // get a response if the value was successfully set
        let mut response_buffer = request_buffer;
        response_buffer.resize(3, 0)?;
        self.read_exact(&mut response_buffer).await?;
        let response_frame_len =
            guess_response_frame_len(&response_buffer, rmodbus::ModbusProto::Rtu)?;
        response_buffer.resize(response_frame_len as usize, 0)?;
        self.read_exact(&mut response_buffer[3..]).await?;
        modbus_request.parse_ok(&response_buffer)?;
        let byte_count = response_buffer[2];
        let val = response_buffer[3];
        if byte_count != 1 {
            return Err(Max485ModbusError::ByteCountError);
        }
        Ok(val)
    }

    pub async fn test_loopback(&mut self) -> Result<bool, Max485ModbusError> {
        let tx_buf = [1, 2, 3, 4, 5, 6, 7];
        self.uart.write_all(&tx_buf).await?;
        let mut rx_buf: [u8; 7] = [0; 7];
        Timer::after_millis(50).await;
        embedded_io_async::Read::read_exact(&mut self.uart, &mut rx_buf).await?;
        let equals = rx_buf == tx_buf;
        Ok(equals)
    }

    pub async fn test_holding(&mut self) -> Result<bool, Max485ModbusError> {
        let tx_buf = [1, 3, 144, 3, 0, 1, 89, 10];
        let required_response = [1, 3, 2, 11, 84, 190, 139];
        self.rw_pin.set_high();
        self.uart.write_all(&tx_buf).await?;
        self.uart.flush_async().await?;
        self.rw_pin.set_low();
        let mut rx_buf: [u8; 7] = [0; 7];
        embedded_io_async::Read::read(&mut self.uart, &mut rx_buf).await?;
        log::info!("reading successful : {:?}", rx_buf);
        let equals = rx_buf == required_response;
        Ok(equals)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
enum Request<'a> {
    GetHoldings { holding_count: u16 },
    SetHoldings { register_values: &'a [u16] },
    GetRegister { register_count: u16 },
    ReadCoils { coil_count: u16 },
}

impl<'a> Write for Max485Modbus<'a> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.rw_pin.set_high();
        embassy_time::block_for(Duration::from_micros(3));
        let bytes_written = embedded_io_async::Write::write(&mut self.uart, buf).await?;
        self.flush().await?;
        self.rw_pin.set_low();
        Ok(bytes_written)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.uart.flush_async().await?;

        // for debugging purposes only
        unsafe {
            let txfifo_cnt: u8 = esp32c6::UART0::steal().status().read().txfifo_cnt().bits();
            let mut err_string: String<128> = String::new();
            core::fmt::Write::write_fmt(
                &mut err_string,
                format_args!("flush(): txfifo_cnt after flush: {txfifo_cnt}\n"),
            )
            .expect("flush(): could not append to net_log");
            self.net_log.append(&err_string);
        };
        Ok(())
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

impl<'a> embedded_io::Read for Max485Modbus<'a> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rw_pin.set_low();
        let count = embedded_io::Read::read(&mut self.uart, buf)?;
        Ok(count)
    }

    fn read_exact(
        &mut self,
        buf: &mut [u8],
    ) -> Result<(), embedded_io::ReadExactError<Self::Error>> {
        self.rw_pin.set_low();
        embedded_io::Read::read_exact(&mut self.uart, buf).map_err(|e| match e {
            ReadExactError::UnexpectedEof => ReadExactError::UnexpectedEof,
            ReadExactError::Other(e) => ReadExactError::Other(e.into()),
        })
    }
}

impl<'a> embedded_io_async::Read for Max485Modbus<'a> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rw_pin.set_low();
        embedded_io_async::Read::read(&mut self.uart, buf)
            .await
            .map_err(|e| e.into())
    }

    async fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), ReadExactError<Self::Error>> {
        self.rw_pin.set_low();
        embedded_io_async::Read::read_exact(&mut self.uart, buf)
            .await
            .map_err(|e| match e {
                ReadExactError::UnexpectedEof => ReadExactError::UnexpectedEof,
                ReadExactError::Other(e) => ReadExactError::Other(e.into()),
            })
    }
}

impl<'a> embedded_svc::io::asynch::ErrorType for Max485Modbus<'a> {
    type Error = Max485ModbusError;
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Max485ModbusError {
    UartRxError(esp_hal::uart::RxError),
    UartTxError(esp_hal::uart::TxError),
    IoError(esp_hal::uart::IoError),
    ModbusError(rmodbus::ErrorKind),
    BufferResizeError,
    ByteCountError,
    ReadExactError(embedded_io_async::ReadExactError<IoError>),
    UnexpectedEof,
}

impl embedded_svc::io::Error for Max485ModbusError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl From<esp_hal::uart::RxError> for Max485ModbusError {
    fn from(value: esp_hal::uart::RxError) -> Self {
        Max485ModbusError::UartRxError(value)
    }
}

impl From<esp_hal::uart::TxError> for Max485ModbusError {
    fn from(value: esp_hal::uart::TxError) -> Self {
        Max485ModbusError::UartTxError(value)
    }
}

impl From<esp_hal::uart::IoError> for Max485ModbusError {
    fn from(value: esp_hal::uart::IoError) -> Self {
        Max485ModbusError::IoError(value)
    }
}

impl From<embedded_io::ReadExactError<IoError>> for Max485ModbusError {
    fn from(value: embedded_io::ReadExactError<IoError>) -> Self {
        Max485ModbusError::ReadExactError(value)
    }
}

impl From<embedded_io::ReadExactError<Max485ModbusError>> for Max485ModbusError {
    fn from(value: embedded_io::ReadExactError<Max485ModbusError>) -> Self {
        match value {
            ReadExactError::UnexpectedEof => Max485ModbusError::UnexpectedEof,
            ReadExactError::Other(e) => e,
        }
    }
}

impl From<()> for Max485ModbusError {
    fn from(_value: ()) -> Self {
        Max485ModbusError::BufferResizeError
    }
}

impl From<rmodbus::ErrorKind> for Max485ModbusError {
    fn from(value: rmodbus::ErrorKind) -> Self {
        Max485ModbusError::ModbusError(value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Device {
    Tracer,
    Inverter,
}
