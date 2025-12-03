# MKS DUAL FOC V3.2 PLUS Pinout

**MCU:** STM32G431CB (LQFP48 package)
**Board:** MKS DUAL FOC V3.2 PLUS
**Clock:** 144 MHz (8 MHz HSE × PLL)

---

## Motor 0 Control

### PWM Outputs (TIM1 - Advanced Timer with Complementary Outputs)

| Phase | High-Side | Low-Side | Timer Channel |
|-------|-----------|----------|---------------|
| A     | PA8       | PB13     | TIM1_CH1 / CH1N |
| B     | PA9       | PB14     | TIM1_CH2 / CH2N |
| C     | PA10      | PB15     | TIM1_CH3 / CH3N |

**Features:**
- Full 6-PWM complementary outputs
- Dead-time insertion for synchronous rectification
- Suitable for advanced FOC algorithms
- Hardware protection with break input capability

### Current Sensing (ADC1)

| Phase | Pin | ADC Channel | Notes |
|-------|-----|-------------|-------|
| A     | PA0 | ADC1_IN1    | 12-bit, 3.3V reference |
| B     | PA1 | ADC1_IN2    | 12-bit, 3.3V reference |

### Control Signals

| Function | Pin | Direction | Active Level |
|----------|-----|-----------|--------------|
| Enable   | PC14| Output    | HIGH         |
| Fault    | PC15| Input     | LOW          |



---

## Motor 1 Control

### PWM Outputs (TIM4 - General Purpose Timer)

| Phase | Pin  | Timer Channel | Notes |
|-------|------|---------------|-------|
| A     | PB6  | TIM4_CH1      | High-side only |
| B     | PB7  | TIM4_CH2      | High-side only |
| C     | PB9  | TIM4_CH4      | High-side only |

**Features:**
- 3-PWM unipolar outputs (no complementary outputs)
- Suitable for 6-step commutation or unipolar PWM control
- TIM4_CH3 (PB8) not used due to BOOT0 conflict

**Limitations:**
- No hardware dead-time insertion
- No complementary outputs for low-side control
- Requires external low-side control or diode rectification

### Current Sensing (ADC1)

| Phase | Pin | ADC Channel | Notes |
|-------|-----|-------------|-------|
| A     | PB0 | ADC1_IN15   | 12-bit, 3.3V reference |
| B     | PB1 | ADC1_IN12   | 12-bit, 3.3V reference |

**Note:** These pins conflict with TIM8 low-side outputs (CH2N, CH3N), which is why TIM8 is not used.

### Control Signals

| Function | Pin | Direction | Active Level |
|----------|-----|-----------|--------------|
| Enable   | PA13| Output    | HIGH         |
| Fault    | PA14| Input     | LOW          |


**Compatible Encoders:**
- Magnetic encoders with SPI interface (AS5047, AS5048, etc.)
- Optical encoders with SPI interface

---

## Communication Interfaces

### USB CDC-ACM

| Function | Pin | Notes |
|----------|-----|-------|
| USB_DP   | PA12| USB D+ |
| USB_DM   | PA11| USB D- |

**Features:**
- Virtual COM port over USB
- USB 2.0 Full-Speed (12 Mbps)
- Can be disabled to save ~34KB flash and ~10KB RAM

**Configuration:** Set `CONFIG_ENABLE_USB=y` in prj.conf

### USART2 Console

| Function | Pin | Baudrate |
|----------|-----|----------|
| TX       | PA2 | 115200   |
| RX       | PA15| 115200   |

**Features:**
- Debug console output
- Command line interface
- LOG output via minimal logging mode
- **Note:** PA15 conflicts with I2C1_SCL

### I2C1 (OLED Display)

**Status:** DISABLED

**Reason:** PA15 is used by USART2_RX, creating a conflict with I2C1_SCL

| Function | Pin | Notes |
|----------|-----|-------|
| SCL      | PA15| Used by USART2_RX |
| SDA      | PB7 | - |

**Connected Device:** SSD1306 OLED (0x3C address, 72×40 pixels)

**⚠️ DISABLED:** I2C1 is disabled because PA15 is required for USART2_RX
- OLED display is not available in this configuration
- MT6701 encoders are not available in this configuration

---

## User Interface

### Status LED

| Function | Pin | Active Level |
|----------|-----|--------------|
| LED0     | PC6 | HIGH         |

### Buttons

| Function | Pin | Active Level | Notes |
|----------|-----|--------------|-------|
| User     | PC13| HIGH (pull-down) | General purpose button |
| Boot0    | PB8 | HIGH         | Bootloader entry, conflicts with TIM4_CH3 |

---

## Alternative Timer Configuration (TIM8 - Not Used)

**Status:** DISABLED due to conflicts

### Why TIM8 is Not Used:

| Pin | TIM8 Function | Conflict With |
|-----|---------------|---------------|
| PA7 | TIM8_CH1N     | Could be used for ADC or other functions |
| PB0 | TIM8_CH2N     | ADC1_IN15 (Motor 1 Phase A current) |
| PB1 | TIM8_CH3N     | ADC1_IN12 (Motor 1 Phase B current) |
| PB6 | TIM8_CH1      | TIM4_CH1 (Motor 1 Phase A PWM) |
| PB8 | TIM8_CH2      | BOOT0 button |
| PB9 | TIM8_CH3      | TIM4_CH4 (Motor 1 Phase C PWM) |

**If you need TIM8 instead of TIM4:**
1. Disable current sensing on Motor 1 (PB0, PB1)
2. Disable BOOT0 button functionality
3. Modify device tree to enable `&timers8` and disable `&timers4`

---

## ADC Configuration

### ADC1 Channels (Synchronous mode, prescaler = 4)

| Channel | Pin | Function | Resolution |
|---------|-----|----------|------------|
| IN1     | PA0 | Motor 0 Phase A Current | 12-bit |
| IN2     | PA1 | Motor 0 Phase B Current | 12-bit |
| IN4     | PA3 | Motor 1 Phase A Current | 12-bit |
| IN12    | PB1 | Motor 1 Phase B Current | 12-bit |

**ADC Settings:**
- Reference: Internal 3.3V
- Acquisition time: Default
- Gain: 1×

### ADC2 Channels (Not configured for motor control)

| Channel | Pin | Function | Notes |
|---------|-----|----------|-------|
| IN12    | PB2 | VBUS sensing | Currently used for USB-C PD (disabled) |

---

## Pin Conflicts and Design Considerations

### Known Conflicts

1. **PA15:** USART2_RX ↔ I2C1_SCL
   - USART2 console takes priority
   - **Impact:** I2C1 disabled, OLED and encoders unavailable
   - **Solution:** Use USB CDC-ACM for console instead, or use different UART pins

2. **PA2:** USART2_TX ↔ ADC1_IN3
   - USART2 console takes priority
   - **Impact:** ADC moved to PA3 (ADC1_IN4)

3. **PB8:** BOOT0 button ↔ TIM8_CH2
   - TIM8 uses CH1, CH2, CH3 (CH2 conflicts with BOOT0)
   - **Impact:** BOOT0 button unavailable when Motor 1 (TIM8) is running

### Available Unused Pins

| Pin | Possible Alternative Use |
|-----|--------------------------|
| PA4 | SPI3_CS, DAC, etc. |
| PA5 | ADC2_IN13, SPI1 |
| PA6 | ADC1_IN3, TIM3_CH1 |
| PA7 | ADC1_IN4, TIM3_CH2, TIM8_CH1N |
| PC10-12 | General GPIO (not available in LQFP48?) |
| PF0-1 | HSE oscillator (in use) |

---

## Memory Usage

**With all features enabled (USB + Logging + 2 Motors):**
- Flash: 60,312 bytes (46.01% of 128 KB)
- RAM: 15,744 bytes (48.05% of 32 KB)

**USB Disabled (CONFIG_ENABLE_USB=n):**
- Flash: 23,652 bytes (18.05%)
- RAM: 4,992 bytes (15.23%)
- **Savings:** ~34 KB flash, ~10 KB RAM

---

## Hardware Recommendations

### For Dual Motor FOC Control:

1. **Motor 0 (TIM1):** Full 6-PWM FOC control
   - Use for high-performance motor requiring synchronous rectification
   - Implement sine-wave FOC algorithms
   - Hall sensors or sensorless control (no encoder)

2. **Motor 1 (TIM4):** 3-PWM trapezoidal or 6-step control
   - Use for less demanding motor
   - 6-step commutation with hall sensors
   - Encoder feedback via SPI3

### Alternative Single Motor Configuration:

If only one motor is needed:
- Use Motor 0 (TIM1) for full FOC capability
- Repurpose Motor 1 pins for:
  - Additional ADC channels
  - Encoder quadrature input
  - CAN bus communication
  - Additional I2C/SPI devices

---

## Device Tree Location

Full configuration: `/home/eug/work/foc/foc.git/fw/boards/arm/foc/foc_431.dts`

**To modify pinout:**
1. Edit the device tree file
2. Rebuild: `west build -b foc_431 -p`
3. Flash: `west flash`

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12-02 | Initial pinout documentation for MKS DUAL FOC V3.2 PLUS |

---

## References

- STM32G431CB Datasheet: [STMicroelectronics](https://www.st.com/en/microcontrollers-microprocessors/stm32g431cb.html)
- Zephyr RTOS Documentation: [docs.zephyrproject.org](https://docs.zephyrproject.org/)
- Device Tree Specification: Zephyr devicetree bindings
