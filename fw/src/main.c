#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <string.h>
#include "oled.h"
#include "motor_control.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Helper macros to get encoder I2C address from device tree */
#if DT_NODE_EXISTS(DT_ALIAS(encoder_motor0))
#define ENCODER_MOTOR0_ADDR  DT_REG_ADDR(DT_ALIAS(encoder_motor0))
#else
#define ENCODER_MOTOR0_ADDR  MT6701_I2C_ADDR_DEFAULT
#endif

#if DT_NODE_EXISTS(DT_ALIAS(encoder_motor1))
#define ENCODER_MOTOR1_ADDR  DT_REG_ADDR(DT_ALIAS(encoder_motor1))
#else
#define ENCODER_MOTOR1_ADDR  MT6701_I2C_ADDR_DEFAULT
#endif

/* Motor configuration parameters */
#define MOTOR_POLE_PAIRS        7
#define MOTOR_PHASE_RESISTANCE  0.5f    /* Ohms */
#define MOTOR_PHASE_INDUCTANCE  0.0001f /* Henries */
#define MOTOR_VOLTAGE_SUPPLY    12.0f   /* Volts */
#define MOTOR_VOLTAGE_LIMIT     10.0f   /* Volts */
#define MOTOR_VELOCITY_LIMIT    100.0f  /* rad/s */
#define MOTOR_CURRENT_LIMIT     10.0f   /* Amps */

/* PWM configuration */
#define PWM_PERIOD_NS           20000   /* 50 kHz PWM frequency */

/* Current sense amplifier configuration */
#define CURRENT_SENSE_GAIN      20.0f  /* Op-amp gain (V/V) */
#define CURRENT_SENSE_SHUNT     0.01f  /* Shunt resistor (Ohms) */
#define CURRENT_SENSE_VREF      3.3f   /* ADC reference voltage */
#define CURRENT_SENSE_OFFSET    1.65f  /* Amplifier mid-point voltage */

/* Control loop frequency */
#define CONTROL_LOOP_US         1000   /* 1 kHz control loop */

/* Thread priority */
#define MOTOR_THREAD_PRIORITY   7

/* Motor contexts for 2 motors */
static struct motor_control_ctx motor0_ctx;
static struct motor_control_ctx motor1_ctx;

/* Common motor configuration */
static const struct motor_control_config motor_config = {
	.pole_pairs = MOTOR_POLE_PAIRS,
	.phase_resistance = MOTOR_PHASE_RESISTANCE,
	.phase_inductance = MOTOR_PHASE_INDUCTANCE,
	.voltage_supply = MOTOR_VOLTAGE_SUPPLY,
	.voltage_limit = MOTOR_VOLTAGE_LIMIT,
	.velocity_limit = MOTOR_VELOCITY_LIMIT,
	.current_limit = MOTOR_CURRENT_LIMIT,
	.pwm_period_ns = PWM_PERIOD_NS,
	.current_sense_gain = CURRENT_SENSE_GAIN,
	.current_sense_shunt = CURRENT_SENSE_SHUNT,
	.current_sense_vref = CURRENT_SENSE_VREF,
	.current_sense_offset = CURRENT_SENSE_OFFSET,
	.control_loop_us = CONTROL_LOOP_US,
};

int main(void)
{
	int ret;
	uint32_t val = 0;

	LOG_INF("FOC STM32G431 Starting - Dual Motor Controller");

	/* Initialize OLED display */
	oled_init();

	/* Get hardware devices */
	const struct device *i2c1_dev = NULL;
	const struct device *i2c2_dev = NULL;
	const struct device *pwm1_dev = NULL;
	const struct device *pwm2_dev = NULL;
	const struct device *adc_dev = NULL;

	/* Get I2C1 for motor 0 encoder */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay)
	i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	if (!device_is_ready(i2c1_dev)) {
		LOG_ERR("I2C1 device not ready");
		return -1;
	}
#else
	LOG_ERR("I2C1 not configured in device tree");
	return -1;
#endif

	/* Get I2C2 for motor 1 encoder (optional) */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c2), okay)
	i2c2_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));
	if (!device_is_ready(i2c2_dev)) {
		LOG_WRN("I2C2 device not ready - motor 1 encoder disabled");
		i2c2_dev = NULL;
	}
#else
	LOG_INF("I2C2 not configured - motor 1 will use I2C1");
#endif

	/* Get PWM for Motor 0 using alias */
#if DT_NODE_EXISTS(DT_ALIAS(pwm_motor0))
	pwm1_dev = DEVICE_DT_GET(DT_ALIAS(pwm_motor0));
	if (!device_is_ready(pwm1_dev)) {
		LOG_ERR("PWM Motor 0 device not ready");
		return -1;
	}
#else
	LOG_ERR("PWM Motor 0 not configured in device tree");
	return -1;
#endif

	/* Get PWM for Motor 1 using alias */
#if DT_NODE_EXISTS(DT_ALIAS(pwm_motor1))
	pwm2_dev = DEVICE_DT_GET(DT_ALIAS(pwm_motor1));
	if (!device_is_ready(pwm2_dev)) {
		LOG_WRN("PWM Motor 1 device not ready - motor 1 disabled");
		pwm2_dev = NULL;
	}
#else
	LOG_WRN("PWM Motor 1 not configured in device tree - motor 1 disabled");
#endif

	/* Get ADC2 for both motors' current sensing (4 channels available) */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(adc2), okay)
	adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc2));
	if (!device_is_ready(adc_dev)) {
		LOG_WRN("ADC2 not ready - current sensing disabled for both motors");
		adc_dev = NULL;
	} else {
		LOG_INF("ADC2 ready: 4 channels available (CH1-2 for M0, CH3-4 for M1)");
	}
#else
	LOG_WRN("ADC2 not configured - current sensing disabled");
#endif

	/* Initialize Motor 0
	 *
	 * Hardware Configuration (from device tree aliases):
	 * - Encoder: encoder-motor0 (MT6701 on I2C1, address from DT)
	 * - PWM: pwm-motor0 (TIM2 CH1/2/3 on PA5, PB3, PB10)
	 * - Current: ADC2 CH1/2 (PA0, PA1)
	 */
	LOG_INF("Initializing Motor 0...");

	uint8_t motor0_addr;
	ret = get_encoder_address("encoder-motor0", &motor0_addr);
	if (ret < 0) {
		LOG_ERR("Failed to get encoder-motor0 address from device tree");
		motor0_addr = MT6701_I2C_ADDR_DEFAULT;  /* Fallback */
	}
	LOG_INF("Motor 0: Encoder I2C address 0x%02X (from device tree)", motor0_addr);

	ret = motor_control_init(&motor0_ctx,
	                         0,                        /* motor_id */
	                         i2c1_dev,                 /* I2C for encoder */
	                         motor0_addr,              /* Address from device tree */
	                         pwm1_dev,                 /* PWM: pwm-motor0 alias */
	                         1, 2, 3,                  /* PWM channels A, B, C */
	                         adc_dev,                  /* ADC2 */
	                         1, 2,                     /* ADC channels 1, 2 */
	                         &motor_config);
	if (ret < 0) {
		LOG_ERR("Failed to initialize motor 0: %d", ret);
	} else {
		/* Start motor 0 control thread */
		ret = motor_control_start_thread(&motor0_ctx, 0, MOTOR_THREAD_PRIORITY);
		if (ret < 0) {
			LOG_ERR("Failed to start motor 0 thread: %d", ret);
		} else {
			LOG_INF("Motor 0 thread started successfully");
			/* Set initial velocity target */
			motor_control_set_velocity(&motor0_ctx, 10.0f); /* 10 rad/s */
		}
	}

	/* Initialize Motor 1 (if PWM is available)
	 *
	 * Hardware Configuration (from device tree aliases):
	 * - Encoder: encoder-motor1 (MT6701 on I2C2, address from DT)
	 * - PWM: pwm-motor1 (TIM3 CH2/3/4 on PA4, PB0, PB1)
	 * - Current: ADC2 CH3/4 (PA6, PA7)
	 *
	 * Note: Uses I2C2 if available, otherwise falls back to I2C1 with alternate address
	 */
	if (pwm2_dev != NULL) {
		LOG_INF("Initializing Motor 1...");

		/* Use I2C2 if available, otherwise fall back to I2C1 */
		const struct device *motor1_i2c = (i2c2_dev != NULL) ? i2c2_dev : i2c1_dev;

		/* Get encoder address from device tree */
		uint8_t motor1_addr;
		ret = get_encoder_address("encoder-motor1", &motor1_addr);
		if (ret < 0) {
			LOG_ERR("Failed to get encoder-motor1 address from device tree");
			/* Fallback: use alternate address if on same I2C bus as motor 0 */
			motor1_addr = (i2c2_dev != NULL) ? MT6701_I2C_ADDR_DEFAULT : MT6701_I2C_ADDR_ALTERNATE;
		}

		if (i2c2_dev != NULL) {
			LOG_INF("Motor 1: Using I2C2 with address 0x%02X (from device tree)", motor1_addr);
		} else {
			LOG_INF("Motor 1: Using I2C1 with address 0x%02X (fallback)", motor1_addr);
		}

		ret = motor_control_init(&motor1_ctx,
		                         1,                          /* motor_id */
		                         motor1_i2c,                 /* I2C for encoder */
		                         motor1_addr,                /* Address from device tree */
		                         pwm2_dev,                   /* PWM: pwm-motor1 alias */
		                         2, 3, 4,                    /* PWM channels (TIM3_CH2/3/4) */
		                         adc_dev,                    /* ADC2 */
		                         3, 4,                       /* ADC channels 3, 4 */
		                         &motor_config);
		if (ret < 0) {
			LOG_ERR("Failed to initialize motor 1: %d", ret);
		} else {
			/* Start motor 1 control thread */
			ret = motor_control_start_thread(&motor1_ctx, 0, MOTOR_THREAD_PRIORITY);
			if (ret < 0) {
				LOG_ERR("Failed to start motor 1 thread: %d", ret);
			} else {
				LOG_INF("Motor 1 thread started successfully");
				/* Set initial velocity target */
				motor_control_set_velocity(&motor1_ctx, 10.0f); /* 10 rad/s */
			}
		}
	} else {
		LOG_INF("Motor 1 disabled (PWM not available)");
	}

	LOG_INF("Motor initialization complete");

	/* Main loop - display status */
	while (1) {
		/* Get motor 0 status */
		float vel0 = motor_control_get_velocity(&motor0_ctx);
		float pos0 = motor_control_get_position(&motor0_ctx);
		enum foc_motor_state state0 = motor_control_get_state(&motor0_ctx);

		LOG_INF("M0: vel=%.2f rad/s, pos=%.2f rad, state=%d",
		        vel0, pos0, state0);

		/* Get motor 1 status if available */
		if (pwm2_dev != NULL) {
			float vel1 = motor_control_get_velocity(&motor1_ctx);
			float pos1 = motor_control_get_position(&motor1_ctx);
			enum foc_motor_state state1 = motor_control_get_state(&motor1_ctx);

			LOG_INF("M1: vel=%.2f rad/s, pos=%.2f rad, state=%d",
			        vel1, pos1, state1);
		}

		oled_write(val++);
		k_sleep(K_MSEC(500));
	}

	return 0;
}
