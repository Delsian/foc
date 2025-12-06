# FOC Motor Control Module

This FOC (Field-Oriented Control) implementation is based on the Arduino-FOC library architecture and adapted for Zephyr RTOS on STM32G431.

## Overview

The FOC module provides advanced motor control capabilities including:
- **Clarke Transform**: Converts 3-phase ABC coordinates to 2-phase Alpha-Beta stationary frame
- **Park Transform**: Converts Alpha-Beta to DQ rotating reference frame
- **Space Vector Modulation (SVM)**: Efficient PWM generation for motor control
- **PID Controllers**: Cascaded PID control for position, velocity, and current
- **Multiple Control Modes**: Voltage, velocity, position, and torque control

## Architecture

### Core Components

1. **foc_math.c/h** - Mathematical transforms and utilities
   - Clarke and Park transforms (forward and inverse)
   - Space Vector Modulation
   - Fast trigonometric functions with lookup tables
   - Angle normalization and electrical angle calculation

2. **foc_pid.c/h** - PID controller implementation
   - Anti-windup integral limiting
   - Configurable proportional, integral, derivative gains
   - Output saturation and clamping

3. **foc_motor.c/h** - Main motor control logic
   - State machine for motor control
   - Multiple control modes (voltage, velocity, position)
   - Sensor integration (MT6701 magnetic encoder)
   - PWM output management
   - Velocity estimation with low-pass filtering

## File Structure

```
src/
├── foc_math.h         # FOC mathematical transforms
├── foc_math.c
├── foc_pid.h          # PID controller
├── foc_pid.c
├── foc_motor.h        # Motor control state machine
├── foc_motor.c
├── mt6701.h           # Magnetic encoder driver
├── mt6701.c
└── main.c             # Example integration
```

## Hardware Requirements

### Minimum Requirements
- STM32G431 microcontroller (or compatible)
- 3-phase BLDC motor
- Position sensor (MT6701 magnetic encoder)
- 3-phase motor driver with PWM inputs
- Power supply suitable for your motor

### Pin Configuration
You need to configure the following in your device tree:
- **I2C**: For MT6701 encoder communication
- **PWM**: 3 channels for motor phases A, B, C (typically TIM1 or similar advanced timer)

## Usage

### 1. Configure Device Tree

Add PWM configuration to your device tree overlay:

```dts
&timers1 {
    status = "okay";
    pwm1: pwm {
        status = "okay";
        pinctrl-0 = <&tim1_ch1_pa8 &tim1_ch2_pa9 &tim1_ch3_pa10>;
        pinctrl-names = "default";
    };
};
```

### 2. Motor Parameters

Update motor parameters in [main.c](src/main.c):

```c
#define MOTOR_POLE_PAIRS        7      // Your motor's pole pairs
#define MOTOR_PHASE_RESISTANCE  0.5f   // Ohms (measure with multimeter)
#define MOTOR_PHASE_INDUCTANCE  0.0001f // Henries
#define MOTOR_VOLTAGE_SUPPLY    12.0f   // Supply voltage
#define MOTOR_VOLTAGE_LIMIT     10.0f   // Max voltage to apply
#define MOTOR_VELOCITY_LIMIT    100.0f  // Max velocity (rad/s)
```

### 3. Initialize FOC Motor

```c
#include "foc_motor.h"

struct foc_motor motor;
struct foc_motor_params params = {
    .pole_pairs = 7,
    .phase_resistance = 0.5f,
    .voltage_limit = 10.0f,
    // ... other parameters
};

struct foc_pwm_output pwm_config = {
    .pwm_dev = pwm_dev,
    .channel_a = 1,
    .channel_b = 2,
    .channel_c = 3,
    .pwm_period_ns = 20000,  // 50 kHz PWM
};

foc_motor_init(&motor, &params, &encoder, &pwm_config);
```

### 4. Control Modes

#### Voltage Control (Open-Loop)
```c
foc_motor_set_mode(&motor, FOC_MODE_VOLTAGE);
foc_motor_set_voltage(&motor, 5.0f, 0.0f);  // Uq=5V, Ud=0V
```

#### Velocity Control
```c
foc_motor_set_mode(&motor, FOC_MODE_VELOCITY);
foc_motor_set_velocity_pid(&motor, 0.5f, 5.0f, 0.0f);  // Kp, Ki, Kd
foc_motor_set_velocity(&motor, 10.0f);  // 10 rad/s target
```

#### Position Control
```c
foc_motor_set_mode(&motor, FOC_MODE_POSITION);
foc_motor_set_position_pid(&motor, 20.0f, 0.0f, 0.1f);  // Kp, Ki, Kd
foc_motor_set_position(&motor, M_PI);  // π radians target
```

### 5. Control Loop

Run the FOC control loop at high frequency (1-10 kHz):

```c
while (1) {
    foc_motor_loop(&motor);
    k_sleep(K_USEC(1000));  // 1 kHz
}
```

## API Reference

### Motor Control Functions

- `foc_motor_init()` - Initialize motor controller
- `foc_motor_enable()` - Enable motor
- `foc_motor_disable()` - Disable motor
- `foc_motor_set_mode()` - Set control mode
- `foc_motor_loop()` - Run FOC control loop
- `foc_motor_calibrate_sensor()` - Calibrate encoder alignment

### Setpoint Functions

- `foc_motor_set_velocity()` - Set velocity target
- `foc_motor_set_position()` - Set position target
- `foc_motor_set_voltage()` - Set direct Uq/Ud voltages

### Tuning Functions

- `foc_motor_set_velocity_pid()` - Configure velocity PID gains
- `foc_motor_set_position_pid()` - Configure position PID gains

### Status Functions

- `foc_motor_get_state()` - Get motor state
- `foc_motor_get_velocity()` - Get current velocity
- `foc_motor_get_position()` - Get current position

## PID Tuning Guide

### Velocity Control Tuning

1. Start with low gains: Kp=0.1, Ki=0.5, Kd=0
2. Increase Kp until you get quick response with some oscillation
3. Increase Ki to eliminate steady-state error
4. Add small Kd if needed to reduce overshoot

Typical values: Kp=0.5, Ki=5.0, Kd=0

### Position Control Tuning

1. Tune velocity loop first (position controller outputs velocity target)
2. Start with proportional gain: Kp=10, Ki=0, Kd=0
3. Increase Kp for faster response
4. Add derivative Kd to reduce overshoot
5. Add small Ki if steady-state error exists

Typical values: Kp=20.0, Ki=0.0, Kd=0.1

## Coordinate Frames

The FOC algorithm uses multiple coordinate transformations:

1. **ABC Frame** (3-phase stationary)
   - Natural motor coordinates: Phase A, B, C

2. **Alpha-Beta Frame** (2-phase stationary)
   - Simplified 2-axis representation
   - Clarke Transform: ABC → Alpha-Beta

3. **DQ Frame** (2-phase rotating)
   - D-axis: Direct (flux-producing)
   - Q-axis: Quadrature (torque-producing)
   - Park Transform: Alpha-Beta → DQ
   - **Advantage**: DC quantities in steady-state, easy to control with PI

## Troubleshooting

### Motor doesn't spin
- Check PWM configuration in device tree
- Verify motor driver connections
- Check voltage supply
- Ensure sensor is reading correctly
- Start with voltage mode to test basic operation

### Motor vibrates/oscillates
- Reduce PID gains (especially Kp)
- Increase control loop frequency
- Check sensor alignment calibration
- Verify pole pairs setting

### Position/velocity drifts
- Increase integral gain (Ki)
- Reduce velocity limit
- Check for mechanical friction
- Verify voltage supply is stable

### Sensor reading errors
- Check I2C connections
- Verify sensor I2C address (default: 0x06)
- Run sensor calibration: `foc_motor_calibrate_sensor()`

## References

- [Arduino-FOC Library](https://github.com/simplefoc/Arduino-FOC)
- [FOC Theory](https://www.ti.com/lit/an/spra588/spra588.pdf)
- Zephyr RTOS Documentation
- STM32G431 Reference Manual

## License

Apache-2.0

Copyright (c) 2025 FOC Project
