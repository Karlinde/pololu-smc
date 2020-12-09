
# `pololu-smc`

A hardware-independent driver for the Pololu Simple Motor Controller G2 (https://www.pololu.com/category/94/pololu-simple-motor-controllers).
Note that only the new generation of controllers, with the "G2" suffix, are supported.

This crate aims to provide a user-friendly interface to the motor controller. Commands and
variables are represented as structs and enums, to avoid having to bother the user with the details of the low-level communication protocols.

Currently only the I<sup>2</sup>C protocol is implemented. It should however be easy to add
support for the serial protocol as well.

## Example
```rust
use pololu_smc::{SimpleMotorController, Variable, Command};

...

let mut controller = SimpleMotorController::new(interface, 0x12);

let errors = controller.get_variable(Variable::ErrorStatus)?;

controller.send_command(Command::ExitSafeStart)?;
controller.send_command(Command::MotorFwd{speed: 500})?;
```
