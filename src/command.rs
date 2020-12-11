
//! Defines types that handle sending commands to the controller.

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
pub enum MotorLimitKind {
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

impl Command {
    #[allow(unused_variables)]
    pub(crate) fn get_id(self) -> u8 {
        match self {
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
}

impl FirmwareVersion {
    /// Converts the raw firmware version into a pair of decimal numbers, (major, minor).
    pub fn get_numerical_version_numbers(self) -> (u8, u8) {
        (
            (self.major_bcd & 0xf0) * 10 + (self.major_bcd & 0x0f),
            (self.minor_bcd & 0xf0) * 10 + (self.minor_bcd & 0x0f),
        )
    }
}