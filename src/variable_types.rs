//! Defines the types that represent the values of non-trivial and non-numerical variables.

/// Error flags that are stopping the motor from running.
///
/// The motor can only be driven when all of these flags are `false`
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub struct Errors {
    /// Safe start violation.
    pub safe_start_violation: bool,
    /// Required channel invalid.
    pub required_channel_invalid: bool,
    /// Serial error.
    pub serial_error: bool,
    /// Command timeout.
    pub command_timeout: bool,
    /// Limit/kill switch.
    pub limit_kill_switch: bool,
    /// Low VIN.
    pub low_vin: bool,
    /// High VIN.
    pub high_vin: bool,
    /// Over temperature.
    pub over_temperature: bool,
    /// Motor driver error.
    pub motor_driver_error: bool,
    /// ERR line high.
    pub err_line_high: bool,
}

/// Serial errors
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub struct SerialErrors {
    /// Frame
    pub frame: bool,
    /// Noise
    pub noise: bool,
    /// RX overrun
    pub rx_overrun: bool,
    /// Format
    pub format: bool,
    /// CRC
    pub crc: bool,
}

/// Flags that indicate things that are currently limiting the motor controller.
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub struct Limits {
    /// Motor is not allowed to run due to an error or safe-start violation.
    pub motor_not_allowed_to_run: bool,
    /// Temperate is actively reducing target speed.
    pub temperature_reducing_speed: bool,
    /// Max speed limit is actively reducing target speed (target speed > max speed).
    pub max_speed: bool,
    /// Starting speed limit is actively reducing target speed to zero (target speed < starting speed).
    pub below_starting_speed: bool,
    /// Motor speed is not equal to target speed because of acceleration, deceleration, or brake duration limits.
    pub speed_limited_acc_dec_brakeduration: bool,
    /// RC1 is configured as a limit/kill switch and the switch is active (scaled value >= 1600).
    pub rc1_killswitch_active: bool,
    /// RC2 is configured as a limit/kill switch and the switch is active (scaled value >= 1600).
    pub rc2_killswitch_active: bool,
    /// AN1 is configured as a limit/kill switch and the switch is active (scaled value >= 1600).
    pub an1_killswitch_active: bool,
    /// AN2 is configured as a limit/kill switch and the switch is active (scaled value >= 1600).
    pub an2_killswitch_active: bool,
    /// USB kill switch is active.
    pub usb_killswitch_active: bool,
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
    /// Controller returned a value not specified in the offical user's guide.
    Unknown,
}

/// If the motor is braking or coasting.
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



fn is_bit_set(byte: u16, bit: u8) -> bool {
    byte & (1 << bit) != 0
}

impl From<u16> for Errors {
    fn from(response: u16) -> Self {
        Self {
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
        }
    }
}

impl From<u16> for SerialErrors {
    fn from(response: u16) -> Self {
        Self {
            frame: is_bit_set(response, 1),
            noise: is_bit_set(response, 2),
            rx_overrun: is_bit_set(response, 3),
            format: is_bit_set(response, 4),
            crc: is_bit_set(response, 5),
        }
    }
}

impl From<u16> for Limits {
    fn from(response: u16) -> Self {
        Self {
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
        }
    }
}

impl From<u16> for ResetSource {
    fn from(response: u16) -> Self {
        match response {
            0x04 => ResetSource::NRstPulledLow,
            0x0c => ResetSource::PowerLow,
            0x14 => ResetSource::SoftwareReset,
            0x24 => ResetSource::WatchdogTimer,
            _ => ResetSource::Unknown,
        }
    }
}

impl From<u16> for BrakeAmount {
    fn from(response: u16) -> Self {
        match response {
            0 => BrakeAmount::Coasting,
            32 => BrakeAmount::Braking,
            0xff => BrakeAmount::NotAvailable,
            _ => BrakeAmount::Unknown,
        }
    }
}

