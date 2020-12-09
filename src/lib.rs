//! This crate provides a user-friendly driver for the Pololu Simple Motor Controller G2, for use on embedded hardware.
//!
//! It uses traits from the [`embedded_hal`] crate in order to be independent from end user hardware.
//! Currently only the I<sup>2</sup>C protocol is implemented.
//!
//! This documentation is heavily based on the official [user's guide for the controller](https://www.pololu.com/docs/pdf/0J77/simple_motor_controller_g2.pdf), published by Pololu.
//! The aim is for this documentation to contain the basic information required to use the controller, but please consult the user's guide
//! in order to fully understand the capabilities and limitations of the hardware and the various protocols.
//!
//! Example
//!```
//! # use embedded_hal::blocking::i2c::{Write, WriteRead};
//! # #[derive(Debug)]
//! # struct Error {}
//! # struct Test {}
//! # impl Write for Test {
//! #     type Error = Error;
//! #     fn write(&mut self, _: u8, _: &[u8]) -> std::result::Result<(), <Self as Write>::Error> {Ok(())}
//! # }
//! # impl WriteRead for Test {
//! #     type Error = Error;
//! #     fn write_read(&mut self, _: u8, _: &[u8], _: &mut [u8]) -> std::result::Result<(), <Self as WriteRead>::Error> {Ok(())}
//! # }
//! # use pololu_smc::VariableValue;
//! use pololu_smc::{SimpleMotorController, Variable, Command};
//!
//! ...
//!
//! # fn main() -> std::result::Result<(), Error> {
//! # let mut interface = Test{};
//! let mut controller = SimpleMotorController::new(interface, 0x12);
//!
//! let errors = controller.get_variable(Variable::ErrorStatus)?;
//! # match errors {
//! #   VariableValue::Errors{safe_start_violation, required_channel_invalid, serial_error, command_timeout, limit_kill_switch, low_vin, high_vin, over_temperature, motor_driver_error, err_line_high} => {   
//! #       assert_eq!(safe_start_violation, false);
//! #       assert_eq!(required_channel_invalid, false);
//! #       assert_eq!(serial_error, false);
//! #       assert_eq!(command_timeout, false);
//! #       assert_eq!(limit_kill_switch, false);
//! #       assert_eq!(low_vin, false);
//! #       assert_eq!(high_vin, false);
//! #       assert_eq!(over_temperature, false);
//! #       assert_eq!(motor_driver_error, false);
//! #       assert_eq!(err_line_high, false);
//! #   },
//! #   _ => {},
//! # };
//!
//! controller.send_command(Command::ExitSafeStart)?;
//! controller.send_command(Command::MotorFwd{speed: 500})?;
//! # Ok(())
//! # }
//!```
#![warn(missing_docs)]
#![no_std]

/// Represents a single physical motor controller.
///
/// The type parameter `T` should be a type that implements the appropriate traits for the desired communications interface.
///   * For I<sup>2</sup>C:
///     - [`embedded_hal::blocking::i2c::Write`]
///     - [`embedded_hal::blocking::i2c::WriteRead`]
pub struct SimpleMotorController<T> {
    /// The [`embedded_hal`] interface to use for communication.
    pub interface: T,
    /// The device number of this controller, in order to separate different controllers using the same bus.
    pub device_number: u8,
}

/// Identifies the variables that can be read from the controller.
///
/// Read using [`SimpleMotorController::get_variable`]
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum Variable {
    /// Indicates the errors that are currently stopping the motor.
    /// Returns a value of [`VariableValue::Errors`]
    ErrorStatus = 0,
    /// Indicates which errors have occurred since last reading this variable.
    /// Returns a value of [`VariableValue::Errors`]
    ErrorsOccurred = 1,
    /// Indicates which serial errors have occurred since last reading this variable.
    /// Returns a value of [`VariableValue::SerialErrors`]
    SerialErrorsOccurred = 2,
    /// Indicates things that are currently limiting the motor controller.
    /// Returns a value of [`VariableValue::Limits`]
    LimitStatus = 3,

    /// Positive pulse width of signal on RC channel 1. Unit: 0.25 μs. 0xFFFF if no valid signal detected.
    RC1UnlimitedRaw = 4,
    /// Positive pulse width of signal on RC channel 1. Unit: 0.25 μs. 0xFFFF if no valid signal detected or if outside error max/error min channel calibration settings.
    RC1Raw = 5,
    /// Scaled version of the RC1 raw value (based on RC channel 1 calibration settings). 0 if the raw value is 0xFFFF, otherwise from -3200 to 3200.
    RC1Scaled = 6,
    /// Positive pulse width of signal on RC channel 2. Unit: 0.25 μs. 0xFFFF if no valid signal detected.
    RC2UnlimitedRaw = 8,
    /// Positive pulse width of signal on RC channel 2. Unit: 0.25 μs. 0xFFFF if no valid signal detected or if outside error max/error min channel calibration settings.
    RC2Raw = 9,
    /// Scaled version of the RC2 raw value (based on RC channel 2 calibration settings). 0 if the raw value is 0xFFFF, otherwise from -3200 to 3200.
    RC2Scaled = 10,

    /// The 12-bit ADC reading of analog channel 1. 0 (0 V) to 4095 (3.3 V). 0xFFFF if the input is disconnected.
    AN1UnlimitedRaw = 12,
    /// The 12-bit ADC reading of analog channel 1. 0 (0 V) to 4095 (3.3 V). 0xFFFF if the input is disconnected or outside of the error max/error min channel calibration settings.
    AN1Raw = 13,
    /// The scaled version of the AN1 raw value (based on analog channel 1 calibration settings). 0 if the raw value is 0xFFFF, otherwise from -3200 to 3200.
    AN1Scaled = 14,
    /// The 12-bit ADC reading of analog channel 2. 0 (0 V) to 4095 (3.3 V). 0xFFFF if the input is disconnected.
    AN2UnlimitedRaw = 16,
    /// The 12-bit ADC reading of analog channel 2. 0 (0 V) to 4095 (3.3 V). 0xFFFF if the input is disconnected or outside of the error max/error min channel calibration settings.
    AN2Raw = 17,
    /// The scaled version of the AN2 raw value (based on analog channel 2 calibration settings). 0 if the raw value is 0xFFFF, otherwise from -3200 to 3200.
    AN2Scaled = 18,

    /// Motor target speed (-3200 to 3200).
    TargetSpeed = 20,
    /// Current speed of the motor (-3200 to 3200).
    Speed = 21,
    /// When speed=0, this variable indicates whether the controller is braking or not.
    BrakeAmount = 22,
    /// Measured voltage on the VIN pin, in mV.
    InputVoltage = 23,
    /// Board temperature in 0.1 °C, measured at a separate location from [`Variable::TemperatureB`]. Temperatures below freezing are reported as 0. Errors measuring temperature are reported as 3000.
    TemperatureA = 24,
    /// Board temperature in 0.1 °C, measured at a separate location from [`Variable::TemperatureA`]. Temperatures below freezing are reported as 0. Errors measuring temperature are reported as 3000.
    TemperatureB = 25,
    /// If there is a valid signal on RC1, this contains the signal period in 0.1ms. Otherwise, this has value 0.
    RCPeriod = 26,
    /// Value of the controller's baud rate register, in seconds per 72 000 000 bits.
    BaudRate = 27,
    /// The two lower bytes of the number of milliseconds that have elapsed since the controller was last reset or powered up.
    UptimeLow = 28,
    /// The two upper bytes of the number of milliseconds that have elapsed since the controller was last reset or powered up.
    UptimeHigh = 29,

    /// Maximum allowed motor speed in the forward direction, 0 to 3200.
    MaxSpeedFwd = 30,
    /// Maximum allowed motor acceleration in the forward direction, in Δspeed per update period. Value range 0 to 3200. 0 means no limit.
    MaxAccFwd = 31,
    /// Maximum allowed motor deceleration from the forward direction, in Δspeed per update period. Value range 0 to 3200. 0 means no limit.
    MaxDecFwd = 32,
    /// Time spent braking (at speed 0) in ms when transitioning from forward to reverse.
    BrakeDurationFwd = 33,
    /// Minimum allowed motor speed in the forward direction, 0 to 3200.
    StartingSpeedFwd = 34,
    /// Maximum allowed motor speed in the reverse direction, 0 to 3200.
    MaxSpeedRev = 36,
    /// Maximum allowed motor acceleration in the reverse direction, in Δspeed per update period. Value range 0 to 3200. 0 means no limit.
    MaxAccRev = 37,
    /// Maximum allowed motor deceleration from the reverse direction, in Δspeed per update period. Value range 0 to 3200. 0 means no limit.
    MaxDecRev = 38,
    /// Time spent braking (at speed 0) in ms when transitioning from reverse to forward.
    BrakeDurationRev = 39,
    /// Minimum allowed motor speed in the reverse direction, 0 to 3200.
    StartingSpeedRev = 40,
    /// The hardware current limit currently being used. See section 5.2 in the official user's guide for instructions on how to interpret the units.
    CurrentLimit = 42,
    /// Raw motor current measurement. See section 5.2 in the official user's guide for instructions on how to interpret the units.
    RawCurrent = 43,
    /// Measurement of the motor current in mA.
    Current = 44,
    /// The number of consecutive 10ms time periods in which the hardware current limiting has activated.
    CurrentLimitingConsecutiveCount = 45,
    /// The number of 10 ms time periods in which the hardware current limit has activated since the last time this
    /// variable was read.
    CurrentLimitingOccurenceCount = 46,

    /// Flags indicating the source of the last board reset.
    /// Returns a value of [`VariableValue::ResetSource`]
    ResetFlags = 127,
}

/// Reasons for why the controller board was reset
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum ResetSource {
    /// <span style="text-decoration:overline">RST</span> pin was pulled low by external source
    NRstPulledLow,
    /// Power reset (VIN got too low or was disconnected)
    PowerLow,
    /// Software reset (by firmware upgrade process)
    SoftwareReset,
    /// Watchdog timer reset (should never happen; this could indicate a firmware bug)
    WatchdogTimer,
    /// Unrecognized value returned from controller
    Unknown,
}

/// The values that can be returned as the value for [`Variable::BrakeAmount`].
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum BrakeAmount {
    /// Motor is coasting.
    Coasting,
    /// Motor is braking.
    Braking,
    /// Speed is not zero, so no brake amount is reported.
    NotAvailable,
    /// Controller returned a value not specified in the offical user's guide.
    Unknown,
}

/// Representations of the various types of values of the variables available to be read from the controller.
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum VariableValue {
    /// Error flags that are stopping the motor from running.
    ///
    /// The motor can only be driven when all of these flags are `false`
    Errors {
        /// Safe start violation.
        safe_start_violation: bool,
        /// Required channel invalid.
        required_channel_invalid: bool,
        /// Serial error.
        serial_error: bool,
        /// Command timeout.
        command_timeout: bool,
        /// Limit/kill switch.
        limit_kill_switch: bool,
        /// Low VIN.
        low_vin: bool,
        /// High VIN.
        high_vin: bool,
        /// Over temperature.
        over_temperature: bool,
        /// Motor driver error.
        motor_driver_error: bool,
        /// ERR line high.
        err_line_high: bool,
    },
    /// Serial errors
    SerialErrors {
        /// Frame
        frame: bool,
        /// Noise
        noise: bool,
        /// RX overrun
        rx_overrun: bool,
        /// Format
        format: bool,
        /// CRC
        crc: bool,
    },
    /// Flags that indicate things that are currently limiting the motor controller.
    Limits {
        /// Motor is not allowed to run due to an error or safe-start violation.
        motor_not_allowed_to_run: bool,
        /// Temperate is actively reducing target speed.
        temperature_reducing_speed: bool,
        /// Max speed limit is actively reducing target speed (target speed > max speed).
        max_speed: bool,
        /// Starting speed limit is actively reducing target speed to zero (target speed < starting speed).
        below_starting_speed: bool,
        /// Motor speed is not equal to target speed because of acceleration, deceleration, or brake duration limits.
        speed_limited_acc_dec_brakeduration: bool,
        /// RC1 is configured as a limit/kill switch and the switch is active (scaled value >= 1600).
        rc1_killswitch_active: bool,
        /// RC2 is configured as a limit/kill switch and the switch is active (scaled value >= 1600).
        rc2_killswitch_active: bool,
        /// AN1 is configured as a limit/kill switch and the switch is active (scaled value >= 1600).
        an1_killswitch_active: bool,
        /// AN2 is configured as a limit/kill switch and the switch is active (scaled value >= 1600).
        an2_killswitch_active: bool,
        /// USB kill switch is active.
        usb_killswitch_active: bool,
    },
    /// Flags indicating the source of the last board reset.
    /// This variable does not change when the controller is running.
    ResetSource(ResetSource),
    /// Indicates if the motor is braking or not.
    BrakeAmount(BrakeAmount),
    /// An unsigned 16-bit integer.
    U16(u16),
    /// A signed 16-bit integer.
    I16(i16),
}

fn is_bit_set(byte: u16, bit: u8) -> bool {
    byte & (1 << bit) != 0
}

impl Variable {
    /// Converts a provided two-byte response from the controller into the proper value type for this variant.
    pub fn get_value(&self, response: u16) -> VariableValue {
        match self {
            Variable::ErrorStatus | Variable::ErrorsOccurred => VariableValue::Errors {
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
            Variable::SerialErrorsOccurred => VariableValue::SerialErrors {
                frame: is_bit_set(response, 1),
                noise: is_bit_set(response, 2),
                rx_overrun: is_bit_set(response, 3),
                format: is_bit_set(response, 4),
                crc: is_bit_set(response, 5),
            },
            Variable::LimitStatus => VariableValue::Limits {
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
            Variable::ResetFlags => match response {
                0x04 => VariableValue::ResetSource(ResetSource::NRstPulledLow),
                0x0c => VariableValue::ResetSource(ResetSource::PowerLow),
                0x14 => VariableValue::ResetSource(ResetSource::SoftwareReset),
                0x24 => VariableValue::ResetSource(ResetSource::WatchdogTimer),
                _ => VariableValue::ResetSource(ResetSource::Unknown),
            },
            Variable::RC1Scaled
            | Variable::RC2Scaled
            | Variable::AN1Scaled
            | Variable::AN2Scaled
            | Variable::TargetSpeed
            | Variable::Speed => VariableValue::I16(response as i16),
            Variable::BrakeAmount => match response {
                0 => VariableValue::BrakeAmount(BrakeAmount::Coasting),
                32 => VariableValue::BrakeAmount(BrakeAmount::Braking),
                0xff => VariableValue::BrakeAmount(BrakeAmount::NotAvailable),
                _ => VariableValue::BrakeAmount(BrakeAmount::Unknown),
            },
            _ => VariableValue::U16(response),
        }
    }
}

#[allow(unused_variables)]
fn get_command_id(cmd: &Command) -> u8 {
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

/// Representations of the commands that can be sent to the controller.
///
/// These only contain the commands which do not provide any response, which can thus be used in [`SimpleMotorController::send_command`]. There are some additional commands available that <b>do</b> provide a response. These are handled
/// separately by the methods [`SimpleMotorController::set_motor_limit`] and [`SimpleMotorController::get_firmware_version`].
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum Command {
    /// If safe-start protection is enabled, this command is required before the motor can run.
    ExitSafeStart,
    /// Sets the full-resolution motor target speed in the forward direction.
    MotorFwd {
        /// Target speed, 0 (motor stopped) to 3200 (full speed).
        speed: u16,
    },
    /// Sets the full-resolution motor target speed in the reverse direction.
    MotorRev {
        /// Target speed, 0 (motor stopped) to 3200 (full speed).
        speed: u16,
    },
    /// Sets the low-resolution motor target speed in the forward direction.
    MotorFwd7bit {
        /// Target speed, 0 (motor stopped) to 127 (full speed).
        speed: u8,
    },

    /// Sets the low-resolution motor target speed in the reverse direction.
    MotorRev7bit {
        /// Target speed, 0 (motor stopped) to 127 (full speed).
        speed: u8,
    },
    /// Causes the motor to immediately brake or coast (configured deceleration limits are ignored).
    MotorBrake {
        /// The amount of braking action. 0 (coasting) to 32.
        brake_amount: u8,
    },
    /// This command lets you change the hardware current limit temporarily until the next reset.
    SetCurrentLimit {
        /// The current limit in internal units. See section 5.2 in the official user's guide
        /// for instructions on how to convert mA into these internal units.
        value: u16,
    },
    /// Sets the motor target speed to zero, stopping the motor respecting the configured deceleration limits.
    /// This also makes the controller susceptible to a safe-start violation error is safe-start is enabled.
    StopMotor,
}

/// All the various ways to configure a motor limit.
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum MotorLimit {
    /// Max speed, both forward and reverse.
    MaxSpeed = 0,
    /// Max acceleration, both forward and reverse.
    MaxAcc = 1,
    /// Max deceleration, both forward and reverse.
    MaxDec = 2,
    /// Brake duration, both forward and reverse.
    BrakeDuration = 3,
    /// Max speed forward.
    MaxSpeedFwd = 4,
    /// Max acceleration forward.
    MaxAccFwd = 5,
    /// Max deceleration forward.
    MaxDecFwd = 6,
    /// Brake duration forward.
    BrakeDurationFwd = 7,
    /// Max speed reverse.
    MaxSpeedRev = 8,
    /// Max acceleration reverse.
    MaxAccRev = 9,
    /// Max deceleration reverse.
    MaxDecRev = 10,
    /// Brake duration reverse.
    BrakeDurationRev = 11,
}

/// All the responses that the controller can provide after setting the motor limits.
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum MotorLimitResponse {
    /// No problems setting the limit.
    Ok,
    /// Unable to set forward limit to the specified value because of hard motor limit settings.
    UnableForward,
    /// Unable to set reverse limit to the specified value because of hard motor limit settings.
    UnableReverse,
    /// Unable to set forward and reverse limits to the specified value because of hard motor limit settings.
    Unable,
    /// The response code has not been specified in the official user's guide.
    Unknown,
}

/// Represents the firmware version and product id.
///
/// Note that the raw version values in this struct are encoded using BCD (Binary Encoded Decimal).
/// Use the [`FirmwareVersion::get_numerical_version_numbers`] method to get a tuple of real decimal version numbers.
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Default)]
pub struct FirmwareVersion {
    /// The product id of this controller's version.
    pub product_id: u16,
    /// Major version number, encoded as BCD (binary encoded decimal).
    pub minor_bcd: u8,
    /// Minor version number, encoded as BCD (binary encoded decimal).
    pub major_bcd: u8,
}

impl FirmwareVersion {
    /// Converts the raw firmware version into a pair of decimal numbers, (major, minor).
    pub fn get_numerical_version_numbers(&self) -> (u8, u8) {
        (
            (self.major_bcd & 0xf0) * 10 + (self.major_bcd & 0x0f),
            (self.minor_bcd & 0xf0) * 10 + (self.minor_bcd & 0x0f),
        )
    }
}

impl<T> SimpleMotorController<T> {
    /// Constructs a new controller using a specicic interface and device number.
    pub fn new(interface: T, device_number: u8) -> SimpleMotorController<T> {
        SimpleMotorController {
            interface,
            device_number,
        }
    }
}
impl<T> SimpleMotorController<T>
where
    T: embedded_hal::blocking::i2c::Write,
{
    /// Sends a given command to the controller. No response is provided.
    pub fn send_command(&mut self, cmd: Command) -> Result<(), T::Error> {
        let cmd_id = get_command_id(&cmd);
        match cmd {
            Command::ExitSafeStart | Command::StopMotor => {
                self.interface.write(self.device_number, &[cmd_id])?
            }
            Command::MotorFwd { speed } | Command::MotorRev { speed } => self.interface.write(
                self.device_number,
                &[cmd_id, (speed % 32) as u8, (speed / 32) as u8],
            )?,
            Command::MotorFwd7bit { speed } | Command::MotorRev7bit { speed } => {
                self.interface.write(self.device_number, &[cmd_id, speed])?
            }
            Command::MotorBrake { brake_amount } => self
                .interface
                .write(self.device_number, &[cmd_id, brake_amount])?,
            Command::SetCurrentLimit { value } => self.interface.write(
                self.device_number,
                &[cmd_id, (value % 128) as u8, (value / 128) as u8],
            )?,
        };
        Ok(())
    }
}

impl<T> SimpleMotorController<T>
where
    T: embedded_hal::blocking::i2c::WriteRead,
{
    /// Sends a request for the value of a certain variable to the controller, and returns
    /// a user-friendly interpretation of the variable value.
    pub fn get_variable(&mut self, variable: Variable) -> Result<VariableValue, T::Error> {
        let mut buf: [u8; 2] = [0, 0];
        self.interface
            .write_read(self.device_number, &[0xa1, variable as u8], &mut buf)?;

        Ok(variable.get_value(buf[0] as u16 | (buf[1] as u16) << 8))
    }

    /// Sends a command to temporarily set a certain motor limit to some value, until the board is reset.
    pub fn set_motor_limit(
        &mut self,
        limit: MotorLimit,
        value: u16,
    ) -> Result<MotorLimitResponse, T::Error> {
        let mut buf: [u8; 1] = [0];
        self.interface.write_read(
            self.device_number,
            &[0xa2, limit as u8, (value % 128) as u8, (value / 128) as u8],
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

    /// Sends a command to read the firmware version and product id from the controller.
    pub fn get_firmware_version(&mut self) -> Result<FirmwareVersion, T::Error> {
        let mut buf: [u8; 4] = [0, 0, 0, 0];
        self.interface
            .write_read(self.device_number, &[0xc2], &mut buf)?;

        Ok(FirmwareVersion {
            product_id: buf[0] as u16 + ((buf[1] as u16) << 8),
            major_bcd: buf[2],
            minor_bcd: buf[3],
        })
    }
}
