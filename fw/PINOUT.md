# FOC STM32G431 Final Pinout Configuration

## Overview
This document describes the final pin allocation for the FOC (Field-Oriented Control) board based on STM32G431CB (UFQFPN48 package).

## Pin Allocation Table

### Power and System Pins
| Pin | Function | Notes |
|-----|----------|-------|
| PA13 | SWDIO | Debug interface (reserved) |
| PA14 | SWCLK | Debug interface (reserved) |
| PC13 | USER Button | GPIO input with pull-down |
| PC6 | Status LED | GPIO output, active high |
| PB8-BOOT0 | BOOT0 Button | GPIO input |

### Motor 0 - TIM2 (3-Phase High-Side PWM)
| Pin | Function | Timer Channel | Notes |
|-----|----------|---------------|-------|
| PA5 | PWM_M0_CH1 | TIM2_CH1 | Phase A |
| PB3 | PWM_M0_CH2 | TIM2_CH2 | Phase B |
| PB10 | PWM_M0_CH3 | TIM2_CH3 | Phase C |

### Motor 1 - TIM3 (3-Phase High-Side PWM)
| Pin | Function | Timer Channel | Notes |
|-----|----------|---------------|-------|
| PA4 | PWM_M1_CH2 | TIM3_CH2 | Phase A |
| PB0 | PWM_M1_CH3 | TIM3_CH3 | Phase B |
| PB1 | PWM_M1_CH4 | TIM3_CH4 | Phase C |

### ADC2 - Current Sensing (12-bit, 3.3V VREF)
| Pin | Function | ADC Channel | Notes |
|-----|----------|-------------|-------|
| PA0 | ADC2_IN1 | Channel 1 | |
| PA1 | ADC2_IN2 | Channel 2 | |
| PA6 | ADC2_IN3 | Channel 3 | |
| PA7 | ADC2_IN4 | Channel 4 | |

**Note:** ADC pins PA0, PA1, PA6 share GPIO with timer PWM outputs. ADC sampling must be coordinated with PWM timing to avoid conflicts.

### USART2 - Debug Console
| Pin | Function | Baud Rate | Notes |
|-----|----------|-----------|-------|
| PA2 | USART2_TX | 115200 | Console output |
| PA3 | USART2_RX | 115200 | Console input |

### I2C1 - Encoder/Sensor Bus #1
| Pin | Function | Speed | Notes |
|-----|----------|-------|-------|
| PA15 | I2C1_SCL | 400 kHz | Fast mode |
| PB7 | I2C1_SDA | 400 kHz | Fast mode |

**I2C1 Devices:**
- SSD1306 OLED display at address 0x3C
- MT6701 encoder (Motor 0) at address 0x06

### I2C2 - Encoder/Sensor Bus #2
| Pin | Function | Speed | Notes |
|-----|----------|-------|-------|
| PC4 | I2C2_SCL | 400 kHz | Fast mode |
| PA8 | I2C2_SDA | 400 kHz | Fast mode |

**I2C2 Devices:**
- MT6701 encoder (Motor 1) at address 0x06

**I2C Configuration:**
- Two independent I2C buses for dual MT6701 magnetic encoders
- Clock frequency: 400 kHz (I2C Fast Mode)
- Pull-up resistors required externally (typically 4.7kΩ)
- MT6701 14-bit magnetic angle encoders (16384 steps per revolution)

### USB - CDC-ACM (Virtual COM Port) **[DISABLED]**
| Pin | Function | Notes |
|-----|----------|-------|
| PA11 | USB_DM | USB D- data line (hardware enabled, software disabled) |
| PA12 | USB_DP | USB D+ data line (hardware enabled, software disabled) |

**USB Status:** Currently disabled in software (CONFIG_ENABLE_USB=n)

### USB-C Reserved Pins (UCPD1)
| Pin | Function | Notes |
|-----|----------|-------|
| PB4 | CC1 | USB-C Configuration Channel 1 (reserved, not available for GPIO) |
| PB6 | CC2 | USB-C Configuration Channel 2 (reserved, not available for GPIO) |
| PA9 | VBUS_SENSE | USB-C VBUS detection (reserved for future use) |
| PA10 | VBUS_DISCHARGE | USB-C VBUS discharge (reserved for future use) |
| PB2 | VBUS_ADC | Connected to VBUS via voltage divider for monitoring |

**Notes:**
- UCPD1 pins PB4/PB6 cannot be used for other functions even when UCPD is disabled. These are hardware-dedicated USB-C PD controller pins.
- PB2 can be used as ADC input (ADC2_IN12) to monitor VBUS voltage through the voltage divider

### FDCAN1 - CAN FD Bus
| Pin | Function | Notes |
|-----|----------|-------|
| PB8 | FDCAN1_RX | CAN receive |
| PB9 | FDCAN1_TX | CAN transmit |

**FDCAN Configuration:**
- Clock divider: 1
- Message RAM: Standard configuration
- Supports CAN FD mode

## Disabled Peripherals

### I2C
- **I2C3**: Disabled

**Enabled I2C:**
- **I2C1**: Enabled on PA15 (SCL) + PB7 (SDA)
- **I2C2**: Enabled on PC4 (SCL) + PA8 (SDA)

### SPI
- **SPI1**: Disabled
- **SPI2**: Disabled
- **SPI3**: Disabled

### Timers
- **TIM1**: Disabled
- **TIM4**: Disabled
- **TIM8**: Disabled

### Other
- **ADC1**: Disabled (using ADC2 instead)
- **UCPD1**: Disabled (USB-C PD pins not suitable)
- **USB**: Hardware enabled in device tree, software disabled (CONFIG_ENABLE_USB=n)

## Clock Configuration

### System Clocks
- **HSE**: 8 MHz crystal oscillator
- **PLL**: 144 MHz (HSE / 2 * 72 / 2)
- **SYSCLK**: 144 MHz
- **AHB**: 144 MHz (prescaler = 1)
- **APB1**: 144 MHz (prescaler = 1)
- **APB2**: 144 MHz (prescaler = 1)

### Low-Power Clocks
- **LSE**: 32.768 kHz (for RTC)
- **LSI**: Enabled
- **LPTIM1**: Clocked from LSE (Zephyr low-power tick source)

## Pin Conflicts and Sharing

### Known Conflicts
1. **PB3**: Used by TIM2_CH2, conflicts with USART2_TX (USART2 uses PA2 instead)
2. **ADC2 pins**: PA0, PA1, PA6, PA7 share GPIO with timer channels
   - Requires careful timing coordination between PWM and ADC sampling
3. **PA8**: Used by I2C2_SDA, conflicts with TIM1_CH1 alternate function
4. **PC4**: Used by I2C2_SCL
5. **PA15**: Used by I2C1_SCL, conflicts with JTDI debug pin (SWD only, JTAG disabled)

### Reserved Pins
- **PA13, PA14**: SWD debug interface (cannot be reassigned)
- **PB8**: Also used as BOOT0 button (shared with FDCAN1_RX)
- **PB2**: Connected to VBUS via voltage divider (ADC2_IN12 for VBUS monitoring)
- **PB4, PB6**: Reserved for USB-C CC1/CC2 (UCPD1, not usable for GPIO)
- **PA9, PA10**: Reserved for USB-C VBUS (future use)

### Pin Sharing Summary
| Pin | Primary Function | Alternate/Shared Function |
|-----|------------------|---------------------------|
| PA0 | ADC2_IN1 | TIM2_CH1 (not used) |
| PA1 | ADC2_IN2 | TIM2_CH2 (not used) |
| PA6 | ADC2_IN3 | TIM3_CH1 (not used) |
| PA8 | I2C2_SDA | TIM1_CH1 (disabled) |
| PA9 | Reserved (USB-C VBUS) | TIM1_CH2 (cannot use) |
| PA10 | Reserved (USB-C VBUS) | TIM1_CH3 (cannot use) |
| PA15 | I2C1_SCL | JTDI (JTAG disabled) |
| PB2 | VBUS_ADC (ADC2_IN12) | TIM2_CH4 (not used) |
| PB4 | Reserved (USB-C CC1) | TIM3_CH1 (cannot use) |
| PB6 | Reserved (USB-C CC2) | - |
| PB8 | FDCAN1_RX | BOOT0 button |
| PC4 | I2C2_SCL | - |

## Memory Usage
- **Flash**: ~19-20 KB / 128 KB (15-16%)
- **RAM**: ~4.5 KB / 32 KB (14%)

## Build Configuration

### Device Tree
- Main file: `boards/arm/foc/foc_431.dts`
- All peripheral configurations defined in device tree

### Kconfig
- `CONFIG_ENABLE_USB=n` - USB disabled in software (hardware ready)
- `CONFIG_I2C=y` - I2C support enabled
- `CONFIG_GPIO=y` - GPIO support enabled

### Build Command
```bash
west build -b foc_431
```

## Hardware Considerations

### PWM Output
- All PWM outputs are **high-side only** (no complementary low-side outputs)
- External gate drivers required for motor control
- Dead-time insertion must be handled externally

### ADC Sampling
- ADC2 operates in synchronous mode (SYNC) with prescaler = 4
- Acquisition time: Default
- Resolution: 12-bit
- Reference voltage: 3.3V internal

### CAN Bus
- FDCAN1 supports both CAN 2.0 and CAN FD modes
- External CAN transceiver required (e.g., TJA1051, SN65HVD230)
- Termination resistor may be needed depending on bus topology

### I2C Bus
- Two independent I2C buses (I2C1 and I2C2) for dual encoder support
- External pull-up resistors required (typically 4.7kΩ for 400 kHz operation)
- MT6701 magnetic encoders can be connected (one per bus)
- Bus addresses must be configured to avoid conflicts if using devices on the same bus

## Notes
- This configuration supports dual-motor FOC with:
  - Motor 0: TIM2 channels 1-3 (PA5, PB3, PB10)
  - Motor 1: TIM3 channels 2-4 (PA4, PB0, PB1)
  - 4× ADC channels for current sensing
  - 2× I2C buses for magnetic encoders (MT6701)
  - FDCAN for vehicle bus communication
  - USART2 for debug console
- USB hardware is ready but disabled in software (can be enabled if needed)
- All peripherals validated for STM32G431CB UFQFPN48 package
- **Note:** TIM3_CH1 not used (PB4 reserved for USB-C CC1 - UCPD1)

## Package Information
- **MCU**: STM32G431CB
- **Package**: UFQFPN48 (7×7 mm, 0.5 mm pitch)
- **Pinctrl File**: `stm32g431c(6-8-b)ux-pinctrl.dtsi`
- **Flash**: 128 KB
- **RAM**: 32 KB
- **Note**: This is a different package than LQFP48 (TX suffix) - pin availability differs
