use crate::max485::Max485ModbusError;
use core::{fmt::Write, str::FromStr};
use embassy_net::tcp::TcpSocket;
use embassy_time::TimeoutError;
use heapless::String;

pub struct NetLog {
    msg: Option<String<2048>>,
}

impl NetLog {
    pub fn new() -> Self {
        NetLog {
            msg: Some(
                String::from_str("---\n").expect("NetLog::new() : could not convert str to String"),
            ),
        }
    }

    pub fn from_timeout_modbus_result<T>(
        possible_err: &Result<Result<T, Max485ModbusError>, TimeoutError>,
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
        NetLog { msg }
    }

    pub fn append(&mut self, s: &str) {
        if let Some(m) = self.msg.as_mut() {
            m.push_str(s).expect("NetLog::append : failed");
        } else {
            let m = String::from_str(s).expect("NetLog::append : failed");
            self.msg = Some(m)
        }
    }

    pub fn append_log(&mut self, other: &NetLog) {
        if let Some(m) = &other.msg {
            self.append(m)
        }
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

impl Default for NetLog {
    fn default() -> Self {
        Self::new()
    }
}
