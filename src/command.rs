pub const COMMAND_SIZE: usize = 33;

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum Command {
    GetVoltageIntervalms = 0x0,
    GetPowerIntervalms,
    GetVoltageBufferSize,
    GetBuffer(BufferType),
    RetransmitBuffers,
    ModbusTracerGetHoldings {
        register_address: u16,
        size: u8,
    },
    ModbusTracerGetInputRegisters {
        register_address: u16,
        size: u8,
    },
    /// sets all the holding values at once
    ModbusTracerSetHoldings {
        register_address: u16,
        new_holding_values: [u16; 15],
    },
    ModbusInverterGetHoldings {
        register_address: u16,
        size: u8,
    },
    ModbusInverterGetInputRegisters {
        register_address: u16,
        size: u8,
    },
    /// sets all the holding values at once
    ModbusInverterSetHoldings {
        register_address: u16,
        new_holding_values: [u16; 8],
    },
    GetLastLogMessage,
}

impl Command {
    pub fn to_bytes(&self) -> [u8; COMMAND_SIZE] {
        let mut res = [0; COMMAND_SIZE];
        res[0] = self.discriminant();
        match self {
            Command::ModbusTracerGetInputRegisters {
                register_address,
                size,
            }
            | Command::ModbusTracerGetHoldings {
                register_address,
                size,
            }
            | Command::ModbusInverterGetInputRegisters {
                register_address,
                size,
            }
            | Command::ModbusInverterGetHoldings {
                register_address,
                size,
            } => {
                let [b1, b2] = register_address.to_be_bytes();
                let b3 = *size;
                res[1] = b1;
                res[2] = b2;
                res[3] = b3;
            }
            Command::ModbusTracerSetHoldings {
                register_address,
                new_holding_values,
            } => {
                let [b1, b2] = register_address.to_be_bytes();
                res[1] = b1;
                res[2] = b2;
                let holding_bytes = &mut res[3..];
                for (ix, val) in new_holding_values.iter().enumerate() {
                    let chunk = val.to_be_bytes();
                    holding_bytes[ix * 2] = chunk[0];
                    holding_bytes[ix * 2 + 1] = chunk[1];
                }
            }
            Command::ModbusInverterSetHoldings {
                register_address,
                new_holding_values,
            } => {
                let [b1, b2] = register_address.to_be_bytes();
                res[1] = b1;
                res[2] = b2;
                let holding_bytes = &mut res[3..];
                for (ix, val) in new_holding_values.iter().enumerate() {
                    let chunk = val.to_be_bytes();
                    holding_bytes[ix * 2] = chunk[0];
                    holding_bytes[ix * 2 + 1] = chunk[1];
                }
            }
            Command::GetBuffer(buffer_type) => {
                res[1] = *buffer_type as u8;
            }
            _ => {}
        }
        res
    }

    pub fn size(&self) -> u8 {
        match self {
            Command::ModbusTracerGetHoldings { size, .. } => *size,
            Command::ModbusTracerGetInputRegisters { size, .. } => *size,
            Command::ModbusInverterGetHoldings { size, .. } => *size,
            Command::ModbusInverterGetInputRegisters { size, .. } => *size,
            _ => 0,
        }
    }

    fn discriminant(&self) -> u8 {
        // WHYYYYYY !!!!!!!!!!!!!!!!!!!!!!
        unsafe { *(self as *const Self as *const u8) }
    }
}

impl TryFrom<&[u8]> for Command {
    type Error = ();

    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        match value {
            [0, ..] => Ok(Command::GetVoltageIntervalms),
            [1, ..] => Ok(Command::GetPowerIntervalms),
            [2, ..] => Ok(Command::GetVoltageBufferSize),
            [3, 0, ..] => Ok(Command::GetBuffer(BufferType::PVVoltage)),
            [3, 1, ..] => Ok(Command::GetBuffer(BufferType::PVPower)),
            [3, 2, ..] => Ok(Command::GetBuffer(BufferType::Battery1Voltage)),
            [3, 3, ..] => Ok(Command::GetBuffer(BufferType::BatteryPackVoltage)),
            [3, 4, ..] => Ok(Command::GetBuffer(BufferType::InverterInputPower)),
            [3, 5, ..] => Ok(Command::GetBuffer(BufferType::InverterOutputPower)),
            [4, ..] => Ok(Command::RetransmitBuffers),
            [5, h1, h2, h3, ..] => Ok(Command::ModbusTracerGetHoldings {
                register_address: u16::from_be_bytes([*h1, *h2]),
                size: *h3,
            }),
            [6, h1, h2, h3, ..] => Ok(Command::ModbusTracerGetInputRegisters {
                register_address: u16::from_be_bytes([*h1, *h2]),
                size: *h3,
            }),
            [7, h1, h2, new_values @ ..] => {
                let mut new_holding_values = [0; 15];
                for (ix, chunk) in new_values.chunks(2).enumerate() {
                    new_holding_values[ix] = u16::from_be_bytes([chunk[0], chunk[1]]);
                }
                Ok(Command::ModbusTracerSetHoldings {
                    register_address: u16::from_be_bytes([*h1, *h2]),
                    new_holding_values,
                })
            }
            [8, h1, h2, h3, ..] => Ok(Command::ModbusInverterGetHoldings {
                register_address: u16::from_be_bytes([*h1, *h2]),
                size: *h3,
            }),
            [9, h1, h2, h3, ..] => Ok(Command::ModbusInverterGetInputRegisters {
                register_address: u16::from_be_bytes([*h1, *h2]),
                size: *h3,
            }),
            [10, h1, h2, new_values @ ..] => {
                let mut new_holding_values = [0; 8];
                for (ix, chunk) in new_values.chunks(2).enumerate() {
                    new_holding_values[ix] = u16::from_be_bytes([chunk[0], chunk[1]]);
                }
                Ok(Command::ModbusInverterSetHoldings {
                    register_address: u16::from_be_bytes([*h1, *h2]),
                    new_holding_values,
                })
            }
            [11, ..] => Ok(Command::GetLastLogMessage),
            _ => Err(()),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BufferType {
    PVVoltage,
    PVPower,
    Battery1Voltage,
    BatteryPackVoltage,
    InverterInputPower,
    InverterOutputPower,
}
