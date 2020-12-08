use embedded_hal::blocking::i2c;

pub struct SimpleMotorController<T> {
    bus: T,
    address: u8,
}


#[derive(Debug, Copy, Clone)]
pub enum Variable {
    ErrorStatus = 0,
    ErrorsOccurred = 1,
    SerialErrorsOccurred = 2,
    LimitStatus = 3,

    RC1UnlimitedRaw = 4,
    RC1Raw = 5,
    RC1Scaled = 6,
    RC2UnlimitedRaw = 8,
    RC2Raw = 9,
    RC2Scaled = 10,

    AN1UnlimitedRaw = 12,
    AN1Raw = 13,
    AN1Scaled = 14,
    AN2UnlimitedRaw = 16,
    AN2Raw = 17,
    AN2Scaled = 18,

    TargetSpeed = 20,
    Speed = 21,
    BrakeAmount = 22,
    InputVoltage = 23,
    TemperatureA = 24,
    TemperatureB = 25,
    RCPeriod = 26,
    BaudRate = 27,
    UptimeLow = 28,
    UptimeHigh = 29,

    MaxSpeedFwd = 30,
    MaxAccFwd = 31,
    MaxDecFwd = 32,
    BrakeDurationFwd = 33,
    StartingSpeedFwd = 34,
    MaxSpeedRev = 36,
    MaxAccRev = 37,
    MaxDecRev = 38,
    BrakeDurationRev = 39,
    StartingSpeedRev = 40,
    CurrentLimit = 42,
    RawCurrent = 43,
    Current = 44,
    CurrentLimitingConsecutiveCount = 45,
    CurrentLimitingOccurenceCount = 46,

    ResetFlags = 127,
}

#[derive(Debug)]
pub enum ResetSource {
    NRstPulledLow,
    PowerLow,
    SoftwareReset,
    WatchdogTimer,
    Unknown,
}

#[derive(Debug)]
pub enum VariableValue {
    Errors {
        safe_start_violation: bool,
        required_channel_invalid: bool,
        serial_error: bool,
        command_timeout: bool,
        limit_kill_switch: bool,
        low_vin: bool,
        high_vin: bool,
        over_temperature: bool,
        motor_driver_error: bool,
        err_line_high: bool,
    },
    SerialErrors {
        frame: bool,
        noise: bool,
        rx_overrun: bool,
        format: bool,
        crc: bool,
    },
    Limits {
        motor_not_allowed_to_run: bool,
        temperature_reducing_speed: bool,
        max_speed: bool,
        below_starting_speed: bool,
        speed_limited_acc_dec_brakeduration: bool,
        rc1_killswitch_active: bool,
        rc2_killswitch_active: bool,
        an1_killswitch_active: bool,
        an2_killswitch_active: bool,
        usb_killswitch_active: bool,
    },
    ResetSource(ResetSource),
    U16(u16),
    I16(i16),
}

fn is_bit_set(byte: u16, bit: u8) -> bool {
    byte & (1 << bit) != 0
}

impl Variable {
    pub fn get_value(&self, response: u16) -> VariableValue {
        use Variable::*;
        match self {
            ErrorStatus | ErrorsOccurred => VariableValue::Errors {
                safe_start_violation: is_bit_set(response, 0),
                required_channel_invalid: is_bit_set(response, 1),
                serial_error: is_bit_set(response, 2),
                command_timeout: is_bit_set(response, 3),
                limit_kill_switch: is_bit_set(response, 4),
                low_vin: is_bit_set(response, 5),
                high_vin: is_bit_set(response, 6),
                over_temperature: is_bit_set(response, 7),
                motor_driver_error: is_bit_set(response, 0),
                err_line_high: is_bit_set(response, 1),
            },
            SerialErrorsOccurred => VariableValue::SerialErrors {
                frame: is_bit_set(response, 1),
                noise: is_bit_set(response, 2),
                rx_overrun: is_bit_set(response, 3),
                format: is_bit_set(response, 4),
                crc: is_bit_set(response, 5),
            },
            LimitStatus => VariableValue::Limits {
                motor_not_allowed_to_run: is_bit_set(response, 0),
                temperature_reducing_speed: is_bit_set(response, 1),
                max_speed: is_bit_set(response, 2),
                below_starting_speed: is_bit_set(response, 3),
                speed_limited_acc_dec_brakeduration: is_bit_set(response, 4),
                rc1_killswitch_active: is_bit_set(response, 5),
                rc2_killswitch_active: is_bit_set(response, 6),
                an1_killswitch_active: is_bit_set(response, 7),
                an2_killswitch_active: is_bit_set(response, 8),
                usb_killswitch_active: is_bit_set(response, 9),
            },
            ResetFlags => match response {
                0x04 => VariableValue::ResetSource(ResetSource::NRstPulledLow),
                0x0c => VariableValue::ResetSource(ResetSource::PowerLow),
                0x14 => VariableValue::ResetSource(ResetSource::SoftwareReset),
                0x24 => VariableValue::ResetSource(ResetSource::WatchdogTimer),
                _ => VariableValue::ResetSource(ResetSource::Unknown),
            },
            RC1Scaled | RC2Scaled | AN1Scaled | AN2Scaled | TargetSpeed | Speed => {
                VariableValue::I16(response as i16)
            }
            _ => VariableValue::U16(response),
        }
    }
}

#[allow(unused_variables)]
pub fn get_command_id(cmd: &Command) -> u8 {
    match cmd {
        Command::ExitSafeStart => 0x83,
        Command::MotorFwd { speed } => 0x85,
        Command::MotorRev { speed } => 0x86,
        Command::MotorFwd7bit { speed } => 0x89,
        Command::MotorRev7bit { speed } => 0x8a,
        Command::MotorBrake { brake_amount } => 0x92,
        Command::SetCurrentLimit { value } => 0x91,
        Command::StopMotor => 0xE0,
    }
}

#[derive(Debug)]
pub enum Command {
    ExitSafeStart,
    MotorFwd { speed: u16 },
    MotorRev { speed: u16 },
    MotorFwd7bit { speed: u8 },
    MotorRev7bit { speed: u8 },
    MotorBrake { brake_amount: u8 },
    SetCurrentLimit { value: u16 },
    StopMotor,
}

pub enum MotorLimit {
    MaxSpeed = 0,
    MaxAcc = 1,
    MaxDec = 2,
    BrakeDuration = 3,
    MaxSpeedFwd = 4,
    MaxAccFwd = 5,
    MaxDecFwd = 6,
    BrakeDurationFwd = 7,
    MaxSpeedRev = 8,
    MaxAccRev = 9,
    MaxDecRev = 10,
    BrakeDurationRev = 11,
}

#[derive(Debug)]
pub enum MotorLimitResponse {
    Ok,
    UnableForward,
    UnableReverse,
    Unable,
    Unknown,
}

#[derive(Debug)]
pub struct FirmwareVersion {
    product_id: u16,
    minor_bcd: u8,
    major_bcd: u8,
}

impl<T> SimpleMotorController<T> {
    pub fn new(bus: T, address: u8) -> SimpleMotorController<T> {
        SimpleMotorController { bus, address }
    }
}
impl<T> SimpleMotorController<T>
where
    T: i2c::Write,
{
    pub fn send_command(&mut self, cmd: Command) -> Result<(), T::Error> {
        let cmd_id = get_command_id(&cmd);
        match cmd {
            Command::ExitSafeStart => self
                .bus
                .write(self.address, &[cmd_id])?,
            Command::MotorFwd { speed } => self.bus.write(
                self.address,
                &[
                    cmd_id,
                    (speed % 32) as u8,
                    (speed / 32) as u8,
                ],
            )?,
            Command::MotorRev { speed } => self.bus.write(
                self.address,
                &[
                    cmd_id,
                    (speed % 32) as u8,
                    (speed / 32) as u8,
                ],
            )?,
            Command::MotorFwd7bit { speed } => self
                .bus
                .write(self.address, &[cmd_id, speed])?,
            Command::MotorRev7bit { speed } => self
                .bus
                .write(self.address, &[cmd_id, speed])?,
            Command::MotorBrake { brake_amount } => self
                .bus
                .write(self.address, &[cmd_id, brake_amount])?,
            Command::SetCurrentLimit { value } => self.bus.write(
                self.address,
                &[
                    cmd_id,
                    (value % 128) as u8,
                    (value / 128) as u8,
                ],
            )?,
            Command::StopMotor => self
                .bus
                .write(self.address, &[cmd_id])?,
        };
        Ok(())
    }
}

impl<T> SimpleMotorController<T>
where
    T: i2c::WriteRead,
{
    pub fn get_variable(&mut self, variable: Variable) -> Result<VariableValue, T::Error> {
        let mut buf: [u8; 2] = [0, 0];
        self.bus.write_read(
            self.address,
            &[0xa1, variable as u8],
            &mut buf,
        )?;

        return Ok(variable.get_value(buf[0] as u16 | (buf[1] as u16) << 8));
    }

    pub fn set_motor_limit(
        &mut self,
        limit: MotorLimit,
        value: u16,
    ) -> Result<MotorLimitResponse, T::Error> {
        let mut buf: [u8; 1] = [0];
        self.bus.write_read(
            self.address,
            &[
                0xa2,
                limit as u8,
                (value % 128) as u8,
                (value / 128) as u8,
            ],
            &mut buf,
        )?;

        match buf[0] {
            0 => Ok(MotorLimitResponse::Ok),
            1 => Ok(MotorLimitResponse::UnableForward),
            2 => Ok(MotorLimitResponse::UnableReverse),
            3 => Ok(MotorLimitResponse::Unable),
            _ => Ok(MotorLimitResponse::Unknown),
        }
    }

    pub fn get_firmware_version(&mut self) -> Result<FirmwareVersion, T::Error> {
        let mut buf: [u8; 4] = [0, 0, 0, 0];
        self.bus.write_read(
            self.address,
            &[0xc2],
            &mut buf,
        )?;

        Ok(FirmwareVersion {
            product_id: (buf[0] as u16 + (buf[1] as u16) << 8),
            major_bcd: buf[2],
            minor_bcd: buf[3],
        })
    }
}

