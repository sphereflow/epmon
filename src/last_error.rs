use crate::max485::Max485ModbusError;
use core::{fmt::Write, str::FromStr};
use embassy_net::tcp::TcpSocket;
use embassy_time::TimeoutError;
use heapless::{String, Vec};

pub struct LastError {
    msg: Option<String<256>>,
}

impl LastError {
    pub fn new() -> Self {
        LastError {
            msg: Some(
                String::from_str("first log message")
                    .expect("LastError::new() : could not convert str to String"),
            ),
        }
    }

    pub fn from_timeout_get_register_or_holding(
        possible_err: &Result<Result<Vec<u16, 128>, Max485ModbusError>, TimeoutError>,
    ) -> Self {
        let msg = match possible_err {
            Ok(Ok(_)) => None,
            Ok(Err(e)) => {
                let mut msg = String::new();
                msg.write_fmt(format_args!("{:?}", e))
                    .expect("from_timeout_get_register_or_holding => could not format string");
                Some(msg)
            }
            Err(e) => {
                let mut msg = String::new();
                msg.write_fmt(format_args!("{:?}", e))
                    .expect("from_timeout_get_register_or_holding => could not format string");
                Some(msg)
            }
        };
        LastError { msg }
    }

    pub async fn send(
        &mut self,
        socket: &mut TcpSocket<'_>,
        send_buf: &mut [u8],
    ) -> Result<(), embassy_net::tcp::Error> {
        if let Some(msg) = self.msg.take() {
            let bytes = msg.into_bytes();
            let transmission_size = bytes.len();
            send_buf[..4].clone_from_slice(&transmission_size.to_be_bytes());
            embedded_io_async::Write::write_all(socket, &send_buf[..4]).await?;
            embedded_io_async::Write::write_all(socket, &bytes[..]).await?;
        } else {
            let transmission_size = 0_usize;
            send_buf[..4].clone_from_slice(&transmission_size.to_be_bytes());
            embedded_io_async::Write::write_all(socket, &send_buf[..4]).await?;
        }
        Ok(())
    }
}

impl Default for LastError {
    fn default() -> Self {
        Self::new()
    }
}
