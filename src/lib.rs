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
//! use pololu_smc::{SimpleMotorController, command::Command};
//!
//! // ...
//!
//! # fn main() -> std::result::Result<(), Error> {
//! # let mut interface = Test{};
//! let mut controller = SimpleMotorController::new(interface, 0x12);
//!
//! let errors = controller.get_error_status()?;
//! # assert_eq!(errors.safe_start_violation, false);
//! # assert_eq!(errors.required_channel_invalid, false);
//! # assert_eq!(errors.serial_error, false);
//! # assert_eq!(errors.command_timeout, false);
//! # assert_eq!(errors.limit_kill_switch, false);
//! # assert_eq!(errors.low_vin, false);
//! # assert_eq!(errors.high_vin, false);
//! # assert_eq!(errors.over_temperature, false);
//! # assert_eq!(errors.motor_driver_error, false);
//! # assert_eq!(errors.err_line_high, false);
//!
//! controller.send_command(Command::ExitSafeStart)?;
//! controller.send_command(Command::MotorFwd{speed: 500})?;
//! # Ok(())
//! # }
//!```
#![warn(missing_docs)]
#![no_std]

pub mod command;
pub mod variable_types;

use command::{Command, FirmwareVersion, MotorLimitKind, MotorLimitResponse};
use variable_types::{BrakeAmount, Errors, Limits, ResetSource};

/// Represents a single physical motor controller.
///
/// The type parameter `T` should be a type that implements the appropriate traits for the desired communications interface.
///   * For I<sup>2</sup>C:
///     - [`embedded_hal::blocking::i2c::Write`]
///     - [`embedded_hal::blocking::i2c::WriteRead`]
pub struct SimpleMotorController<T> {
    /// The [`embedded_hal`] interface to use for communication.
    interface: T,
    /// The device number of this controller, in order to separate different controllers using the same bus.
    pub device_number: u8,
}

/// Maps the variables that can be read from the controller to their identifiers in the protocol.
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
enum Variable {
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

/// A channel of measurement, either RC or analog
#[allow(missing_docs)]
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum Channel {
    Ch1,
    Ch2,
}

/// A direction of movement
#[allow(missing_docs)]
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum Direction {
    Forward,
    Reverse,
}

impl<T> SimpleMotorController<T> {
    /// Constructs a new controller using a specific interface and device number.
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
        match cmd {
            Command::ExitSafeStart | Command::StopMotor => {
                self.interface.write(self.device_number, &[cmd.get_id()])?
            }
            Command::MotorFwd { speed } | Command::MotorRev { speed } => self.interface.write(
                self.device_number,
                &[cmd.get_id(), (speed % 32) as u8, (speed / 32) as u8],
            )?,
            Command::MotorFwd7bit { speed } | Command::MotorRev7bit { speed } => self
                .interface
                .write(self.device_number, &[cmd.get_id(), speed])?,
            Command::MotorBrake { brake_amount } => self
                .interface
                .write(self.device_number, &[cmd.get_id(), brake_amount])?,
            Command::SetCurrentLimit { value } => self.interface.write(
                self.device_number,
                &[cmd.get_id(), (value % 128) as u8, (value / 128) as u8],
            )?,
        };
        Ok(())
    }
}

impl<T> SimpleMotorController<T>
where
    T: embedded_hal::blocking::i2c::WriteRead,
{
    /// Performs a request for a raw u16 value of a variable.
    fn get_variable_raw(&mut self, variable: Variable) -> Result<u16, T::Error> {
        let mut buf: [u8; 2] = [0, 0];
        self.interface
            .write_read(self.device_number, &[0xa1, variable as u8], &mut buf)?;

        Ok(u16::from(buf[0]) | (u16::from(buf[1])) << 8)
    }

    /// Returns the errors that are currently stopping the motor
    pub fn get_error_status(&mut self) -> Result<Errors, T::Error> {
        Ok(Errors::from(self.get_variable_raw(Variable::ErrorStatus)?))
    }

    /// Like [`SimpleMotorController::get_error_status`], but only returns the new errors that occurred after the
    /// last time this variable was read.
    pub fn get_new_errors(&mut self) -> Result<Errors, T::Error> {
        Ok(Errors::from(
            self.get_variable_raw(Variable::ErrorsOccurred)?,
        ))
    }

    /// Returns the new serial errors that occurred after the
    /// last time this variable was read.
    pub fn get_new_serial_errors(&mut self) -> Result<Errors, T::Error> {
        Ok(Errors::from(
            self.get_variable_raw(Variable::SerialErrorsOccurred)?,
        ))
    }

    /// Returns the things that are currently limiting the operation of the motor controller in some way.
    pub fn get_limit_status(&mut self) -> Result<Limits, T::Error> {
        Ok(Limits::from(self.get_variable_raw(Variable::LimitStatus)?))
    }

    /// Returns positive pulse width of signal on an RC channel. Unit: 0.25 μs. 0xFFFF if no valid signal detected.
    pub fn get_rc_unlimited_raw(&mut self, ch: Channel) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(match ch {
            Channel::Ch1 => Variable::RC1UnlimitedRaw,
            Channel::Ch2 => Variable::RC2UnlimitedRaw,
        })?)
    }

    /// Returns positive pulse width of signal on an RC channel. Unit: 0.25 μs. 0xFFFF if no valid signal detected or if outside error max/error min channel calibration settings.
    pub fn get_rc_raw(&mut self, ch: Channel) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(match ch {
            Channel::Ch1 => Variable::RC1Raw,
            Channel::Ch2 => Variable::RC2Raw,
        })?)
    }

    /// Returns scaled version of the RC raw value (based on RC channel calibration settings). 0 if the raw value is 0xFFFF, otherwise from -3200 to 3200.
    pub fn get_rc_scaled(&mut self, ch: Channel) -> Result<i16, T::Error> {
        Ok(self.get_variable_raw(match ch {
            Channel::Ch1 => Variable::RC1Scaled,
            Channel::Ch2 => Variable::RC2Scaled,
        })? as i16)
    }

    /// Returns the 12-bit ADC reading of an analog channel. 0 (0 V) to 4095 (3.3 V). 0xFFFF if the input is disconnected.
    pub fn get_analog_unlimited_raw(&mut self, ch: Channel) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(match ch {
            Channel::Ch1 => Variable::AN1UnlimitedRaw,
            Channel::Ch2 => Variable::AN2UnlimitedRaw,
        })?)
    }

    /// Returns the 12-bit ADC reading of an analog channel. 0 (0 V) to 4095 (3.3 V). 0xFFFF if the input is disconnected or outside of the error max/error min channel calibration settings.
    pub fn get_analog_raw(&mut self, ch: Channel) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(match ch {
            Channel::Ch1 => Variable::AN1Raw,
            Channel::Ch2 => Variable::AN2Raw,
        })?)
    }

    /// Returns the scaled version of the analog raw value (based on analog channel calibration settings). 0 if the raw value is 0xFFFF, otherwise from -3200 to 3200.
    pub fn get_analog_scaled(&mut self, ch: Channel) -> Result<i16, T::Error> {
        Ok(self.get_variable_raw(match ch {
            Channel::Ch1 => Variable::AN1Scaled,
            Channel::Ch2 => Variable::AN2Scaled,
        })? as i16)
    }

    /// Returns motor target speed (-3200 to 3200).
    pub fn get_target_speed(&mut self) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(Variable::TargetSpeed)?)
    }

    /// Returns current speed of the motor (-3200 to 3200).
    pub fn get_speed(&mut self) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(Variable::Speed)?)
    }

    /// When speed is 0, the returnev value indicates whether the controller is braking or not.
    pub fn get_brake_amount(&mut self) -> Result<BrakeAmount, T::Error> {
        Ok(BrakeAmount::from(
            self.get_variable_raw(Variable::BrakeAmount)?,
        ))
    }

    /// Returns measured voltage on the VIN pin, in mV.
    pub fn get_input_voltage(&mut self) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(Variable::InputVoltage)?)
    }

    /// Returns board temperature in 0.1 °C, measured at a separate location from [`SimpleMotorController::get_temperature_b`]. Temperatures below freezing are reported as 0. Errors measuring temperature are reported as 3000.
    pub fn get_temperature_a(&mut self) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(Variable::TemperatureA)?)
    }

    /// Returns board temperature in 0.1 °C, measured at a separate location from [`SimpleMotorController::get_temperature_a`]. Temperatures below freezing are reported as 0. Errors measuring temperature are reported as 3000.
    pub fn get_temperature_b(&mut self) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(Variable::TemperatureB)?)
    }

    /// If there is a valid signal on RC1, this returns the signal period in 0.1ms. Otherwise, this returns value 0.
    pub fn get_rc_period(&mut self) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(Variable::RCPeriod)?)
    }

    /// Returns the value of the controller's baud rate register, in seconds per 72 000 000 bits.
    pub fn get_baud_rate(&mut self) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(Variable::BaudRate)?)
    }

    /// Returns the two lower bytes of the number of milliseconds that have elapsed since the controller was last reset or powered up.
    pub fn get_uptime_low(&mut self) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(Variable::UptimeLow)?)
    }

    /// Returns the two upper bytes of the number of milliseconds that have elapsed since the controller was last reset or powered up.
    pub fn get_uptime_high(&mut self) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(Variable::UptimeHigh)?)
    }

    /// Returns the number of milliseconds since the last reset, by combining the results of [`SimpleMotorController::get_uptime_low`] and [`SimpleMotorController::get_uptime_high`].
    pub fn get_uptime(&mut self) -> Result<u32, T::Error> {
        Ok((u32::from(self.get_uptime_high()?) << 16) + u32::from(self.get_uptime_low()?))
    }

    /// Returns maximum allowed motor speed in direction `dir`, 0 to 3200.
    pub fn get_max_speed(&mut self, dir: Direction) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(match dir {
            Direction::Forward => Variable::MaxSpeedFwd,
            Direction::Reverse => Variable::MaxSpeedRev,
        })?)
    }

    /// Returns maximum allowed motor acceleration in direction `dir`, in Δspeed per update period. Value range 0 to 3200. 0 means no limit.
    pub fn get_max_acceleration(&mut self, dir: Direction) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(match dir {
            Direction::Forward => Variable::MaxAccFwd,
            Direction::Reverse => Variable::MaxAccRev,
        })?)
    }

    /// Returns maximum allowed motor deceleration from direction `dir`, in Δspeed per update period. Value range 0 to 3200. 0 means no limit.
    pub fn get_max_deceleration(&mut self, dir: Direction) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(match dir {
            Direction::Forward => Variable::MaxDecFwd,
            Direction::Reverse => Variable::MaxDecRev,
        })?)
    }

    /// Returns time spent braking (at speed 0) in ms when transitioning from direction `dir` to the other.
    pub fn get_brake_duration(&mut self, dir: Direction) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(match dir {
            Direction::Forward => Variable::BrakeDurationFwd,
            Direction::Reverse => Variable::BrakeDurationRev,
        })?)
    }

    /// Returns minimum allowed motor speed in direction `dir`, 0 to 3200.
    pub fn get_starting_speed(&mut self, dir: Direction) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(match dir {
            Direction::Forward => Variable::StartingSpeedFwd,
            Direction::Reverse => Variable::StartingSpeedRev,
        })?)
    }

    /// Returns the hardware current limit currently being used. See section 5.2 in the official user's guide for instructions on how to interpret the units.
    pub fn get_current_limit(&mut self) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(Variable::CurrentLimit)?)
    }

    /// Returns raw motor current measurement. See section 5.2 in the official user's guide for instructions on how to interpret the units.
    pub fn get_raw_current(&mut self) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(Variable::RawCurrent)?)
    }

    /// Returns a measurement of the motor current in mA.
    pub fn get_current(&mut self) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(Variable::Current)?)
    }

    /// Returns the number of consecutive 10ms time periods in which the hardware current limiting has activated.
    pub fn get_current_limiting_consecutive_count(&mut self) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(Variable::CurrentLimitingConsecutiveCount)?)
    }

    /// Returns the number of 10 ms time periods in which the hardware current limit has activated since the last time 
    /// this variable was read.
    pub fn get_current_limiting_occurence_count(&mut self) -> Result<u16, T::Error> {
        Ok(self.get_variable_raw(Variable::CurrentLimitingOccurenceCount)?)
    }

    /// Indicates the source of the last board reset.
    pub fn get_reset_flags(&mut self) -> Result<ResetSource, T::Error> {
        Ok(ResetSource::from(
            self.get_variable_raw(Variable::ResetFlags)?,
        ))
    }

    /// Sends a command to temporarily set a certain motor limit to some value, until the board is reset.
    pub fn set_motor_limit(
        &mut self,
        limit: MotorLimitKind,
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
            product_id: u16::from(buf[0]) + ((u16::from(buf[1])) << 8),
            major_bcd: buf[2],
            minor_bcd: buf[3],
        })
    }
}
