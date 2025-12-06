# FOC Current Sensing Module

This module provides ADC-based current sensing for FOC motor control, enabling advanced control modes like torque control and current-limited operation.

## Overview

The current sensing module measures motor currents using 2 ADC channels and applies Clarke and Park transforms to obtain DQ-frame currents for FOC control.

### Key Features

- **2-channel current measurement** via ADC
- **Automatic third phase calculation** using Kirchhoff's law
- **Automatic offset calibration** for zero-current detection
- **Clarke and Park transforms** integrated for DQ current calculation
- **Configurable amplifier parameters** (gain, shunt resistance, offset)
- **Raw ADC and calibrated current readings** in ABC and DQ frames
- **Seamless integration** with FOC motor module

## Hardware Requirements

### Sensing Configuration

This module uses **2-channel sensing**:
- Measures phases A and B only
- Calculates phase C using: **Ic = -(Ia + Ib)**
- Saves one ADC channel and amplifier per motor
- Accurate for balanced three-phase systems

### Current Sense Amplifiers

For each measured motor phase, you need:
1. **Shunt resistor** in series with motor phase (typically 0.01-0.1 Ω)
2. **Op-amp current sense amplifier** (bidirectional)
3. **ADC input** connected to amplifier output

### Typical Circuit (2-Channel)

```
Motor Phase A ──┬── Shunt Resistor ──┬── Motor Driver
                │                     │
                └── Current Sense ────┴── ADC Channel 1
                    Amplifier              (to MCU)

Motor Phase B ──┬── Shunt Resistor ──┬── Motor Driver
                │                     │
                └── Current Sense ────┴── ADC Channel 2
                    Amplifier              (to MCU)

Motor Phase C ──── (No sensing, calculated from A and B)
```

### Amplifier Requirements

- **Bidirectional**: Must measure both positive and negative currents
- **Rail-to-rail output**: For full ADC range utilization
- **Fast response**: Bandwidth > 10 kHz recommended
- **Mid-supply offset**: Output centered at Vref/2 for zero current

### Common Current Sense ICs

- INA240 (Texas Instruments) - Enhanced PWM rejection
- INA181 (Texas Instruments) - Low-cost bidirectional
- MAX9918 (Analog Devices) - High-side current sensing
- LMV358 (generic op-amp) - DIY amplifier circuits

## Configuration

### Amplifier Parameters

Configure in [src/main.c:26-30](src/main.c#L26-L30):

```c
#define CURRENT_SENSE_GAIN      20.0f  /* Op-amp gain (V/V) */
#define CURRENT_SENSE_SHUNT     0.01f  /* Shunt resistor (Ohms) */
#define CURRENT_SENSE_VREF      3.3f   /* ADC reference voltage */
#define CURRENT_SENSE_OFFSET    1.65f  /* Amplifier mid-point voltage */
```

#### Parameter Calculation

**Gain**: Amplifier voltage gain
- Example: INA240A1 has fixed gain of 20 V/V

**Shunt Resistance**: Value of current sense resistor
- Lower values = less power loss but lower signal
- Higher values = better signal but more heating
- Typical: 0.01 Ω for 10A motors

**Vref**: ADC reference voltage
- Typically 3.3V for STM32
- Check your MCU datasheet

**Offset**: Zero-current voltage level
- Usually Vref/2 for bidirectional sensing
- Verify with oscilloscope or calibration

### Device Tree Configuration

Add 2-channel ADC configuration to your device tree overlay:

```dts
&adc1 {
    status = "okay";
    #address-cells = <1>;
    #size-cells = <0>;

    channel@1 {
        reg = <1>;
        zephyr,gain = "ADC_GAIN_1";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,resolution = <12>;
    };

    channel@2 {
        reg = <2>;
        zephyr,gain = "ADC_GAIN_1";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,resolution = <12>;
    };
};
```

Update pin assignments in your board's pinctrl (phases A and B only):

```dts
&pinctrl {
    adc1_in1_pa0: adc1_in1_pa0 {
        pinmux = <STM32_PINMUX('A', 0, ANALOG)>;
    };
    adc1_in2_pa1: adc1_in2_pa1 {
        pinmux = <STM32_PINMUX('A', 1, ANALOG)>;
    };
};
```

## API Usage

### 1. Initialize Current Sensing

```c
#include "foc_current_sense.h"

struct current_sense_dev current_sense;

const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));

struct current_sense_amp_config amp_config = {
    .gain = 20.0f,
    .shunt_resistance = 0.01f,
    .offset_voltage = 1.65f,
    .vref = 3.3f,
};

/* Initialize 2-channel current sensing (phases A and B) */
int ret = current_sense_init(&current_sense, adc_dev,
                              1, 2,  /* ADC channels A, B */
                              &amp_config);
```

### 2. Calibrate Zero Offset

**IMPORTANT**: Calibrate when motor is NOT powered (zero current).

```c
/* Calibrate with 100 samples */
ret = current_sense_calibrate(&current_sense, 100);
if (ret == 0) {
    LOG_INF("Current sense calibrated");
}
```

### 3. Link to Motor Controller

```c
foc_motor_link_current_sense(&motor, &current_sense);
```

### 4. Read Currents

#### ABC Frame (Three-Phase)
```c
struct abc_frame currents_abc;
current_sense_read_abc(&current_sense, &currents_abc);
LOG_INF("Ia=%.2fA Ib=%.2fA Ic=%.2fA",
        currents_abc.a, currents_abc.b, currents_abc.c);
```

#### DQ Frame (Rotating Reference)
```c
struct dq_frame currents_dq;
float electrical_angle = /* from encoder */;
current_sense_read_dq(&current_sense, electrical_angle, &currents_dq);
LOG_INF("Id=%.2fA Iq=%.2fA", currents_dq.d, currents_dq.q);
```

## Control Modes with Current Sensing

### Torque Control Mode

Direct current control for precise torque:

```c
foc_motor_set_mode(&motor, FOC_MODE_TORQUE);
foc_motor_set_current(&motor, 5.0f, 0.0f);  /* Iq=5A, Id=0A */
```

Torque is proportional to Q-axis current (Iq):
```
Torque = (3/2) × pole_pairs × Ψ × Iq
```

### Current-Limited Velocity Control

Velocity control with automatic current limiting:

```c
/* Current sensing automatically enables cascaded control:
 * Velocity PID → Current Target → Voltage
 */
foc_motor_set_mode(&motor, FOC_MODE_VELOCITY);
foc_motor_set_velocity(&motor, 50.0f);  /* 50 rad/s */

/* Current automatically limited by MOTOR_CURRENT_LIMIT */
```

## Calibration

### Automatic Calibration

Run during startup when motor is unpowered:

```c
current_sense_calibrate(&current_sense, 100);
```

This measures and stores ADC offset values for zero current.

### Manual Verification

Check calibration offsets:

```c
LOG_INF("Offset A: %.1f", current_sense.offset_a);
LOG_INF("Offset B: %.1f", current_sense.offset_b);
```

Expected values: ~2048 for 12-bit ADC at mid-range (Vref/2)

Note: Phase C offset is not stored since it's calculated from phases A and B.

### Re-calibration

Re-calibrate if:
- Temperature changes significantly
- Power supply voltage changes
- Amplifier offset drifts

## Troubleshooting

### No Current Readings (Always Zero)

**Check:**
1. ADC configured in device tree and enabled
2. ADC channels match hardware connections
3. Amplifier powered and functioning
4. Shunt resistor installed correctly

**Test:**
```c
int16_t raw_a, raw_b;
current_sense_read_raw(&current_sense, &raw_a, &raw_b);
LOG_INF("Raw ADC A: %d, B: %d", raw_a, raw_b);
```

### Current Readings Stuck at Offset

**Possible causes:**
- Motor not powered (normal during calibration)
- Shunt resistor not in circuit path
- Amplifier output saturated

### Noisy Current Readings

**Solutions:**
1. Add low-pass filter to amplifier output (10 nF capacitor)
2. Increase ADC acquisition time
3. Use hardware oversampling if available
4. Software filtering:
   ```c
   // Add to motor loop
   current_filtered = 0.9f * current_filtered + 0.1f * current_measured;
   ```

### Incorrect Current Magnitude

**Check configuration:**
- Gain setting matches amplifier datasheet
- Shunt resistor value correct
- Vref matches ADC reference

**Formula check:**
```
Expected ADC value = (Vref/2) + (Current × R_shunt × Gain)

Example: 5A current, 0.01Ω shunt, 20x gain, 3.3V ref
ADC = 1.65V + (5 × 0.01 × 20) = 2.65V
ADC raw = (2.65 / 3.3) × 4095 = 3286
```

### Current Polarity Inverted

Swap ADC channel assignments or invert shunt resistor connections.

## Advanced Features

### Current Limiting

Automatic current limiting protects motor and driver:

```c
motor.params.current_limit = 10.0f;  /* 10A max */
```

Current PIDs automatically clamp output to this limit.

### Field Weakening

For speeds above base speed, apply negative D-axis current:

```c
if (motor.velocity > base_velocity) {
    float id_target = -2.0f;  /* Field weakening */
    motor.current_d_target = id_target;
}
```

### Power Calculation

Calculate instantaneous power:

```c
struct abc_frame currents, voltages;
current_sense_read_abc(&current_sense, &currents);
/* voltages from motor.voltage_* */

float power = voltages.a * currents.a +
              voltages.b * currents.b +
              voltages.c * currents.c;
```

## Performance Considerations

### ADC Sampling Frequency

- **Minimum**: 2× FOC loop frequency
- **Recommended**: Same as FOC loop (1-10 kHz)
- **Maximum**: Limited by ADC conversion time

### ADC Triggering

For best performance, trigger ADC conversion from PWM timer to sample during stable PWM period.

### CPU Load

Typical overhead per control loop iteration:
- 2× ADC reads: ~20 μs
- Clarke transform: ~5 μs
- Park transform: ~10 μs
- **Total**: ~35 μs @ 170 MHz STM32G4

## References

- [Current Sensing for Motor Control](https://www.ti.com/lit/an/slyt665/slyt665.pdf)
- STM32G4 ADC Application Note
- [FOC Current Control Theory](https://www.ti.com/lit/an/bpra073/bpra073.pdf)

## File Structure

```
inc/
└── foc_current_sense.h  # Current sensing API
src/foc/
└── foc_current_sense.c  # Current sensing implementation
```

## License

Apache-2.0

Copyright (c) 2025 FOC Project
