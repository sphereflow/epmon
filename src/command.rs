#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum Command {
    GetIntervalms = 0x0,
    GetVoltageBufferSize,
    GetBattery1Buffer,
    GetBatteryPackBuffer,
    GetPVBuffer,
    RetransmitBuffers,
    ModbusGetHoldings {
        register_address: u16,
        size: u8,
    },
    ModbusGetInputRegisters {
        register_address: u16,
        size: u8,
    },
    ModbusSetHolding {
        register_address: u16,
        new_holding_value: u16,
    },
}

impl Command {
    pub fn to_bytes(&self) -> [u8; 5] {
        let b0 = self.discriminant();
        let [b1, b2, b3, b4] = match self {
            Command::ModbusGetInputRegisters {
                register_address,
                size,
            }
            | Command::ModbusGetHoldings {
                register_address,
                size,
            } => {
                let [b1, b2] = register_address.to_be_bytes();
                let b3 = *size;
                [b1, b2, b3, 0]
            }
            Command::ModbusSetHolding {
                register_address,
                new_holding_value,
            } => {
                let [b1, b2] = register_address.to_be_bytes();
                let [b3, b4] = new_holding_value.to_be_bytes();
                [b1, b2, b3, b4]
            }
            _ => [0, 0, 0, 0],
        };
        [b0, b1, b2, b3, b4]
    }

    pub fn size(&self) -> u8 {
        match self {
            Command::ModbusGetHoldings { size, .. } => *size,
            Command::ModbusGetInputRegisters { size, .. } => *size,
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
        log::info!("command: {:?}", value);
        match value {
            [0, ..] => Ok(Command::GetIntervalms),
            [1, ..] => Ok(Command::GetVoltageBufferSize),
            [2, ..] => Ok(Command::GetBattery1Buffer),
            [3, ..] => Ok(Command::GetBatteryPackBuffer),
            [4, ..] => Ok(Command::GetPVBuffer),
            [5, ..] => Ok(Command::RetransmitBuffers),
            [6, h1, h2, h3, _h4] => Ok(Command::ModbusGetHoldings {
                register_address: u16::from_be_bytes([*h1, *h2]),
                size: *h3,
            }),
            [7, h1, h2, h3, _h4] => Ok(Command::ModbusGetInputRegisters {
                register_address: u16::from_be_bytes([*h1, *h2]),
                size: *h3,
            }),
            [8, h1, h2, h3, h4] => Ok(Command::ModbusSetHolding {
                register_address: u16::from_be_bytes([*h1, *h2]),
                new_holding_value: u16::from_be_bytes([*h3, *h4]),
            }),
            _ => Err(()),
        }
    }
}
