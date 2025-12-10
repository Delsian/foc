#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <stdlib.h>
#include "console.h"
#include "mt6701.h"
#include "adc_dma.h"
#include "pwm_bldc.h"

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

/* Get encoder devices from device tree aliases */
static const struct device *get_encoder_device(uint8_t id)
{
	switch (id) {
	case 0:
		return DEVICE_DT_GET(DT_ALIAS(encoder_motor0));
	case 1:
		return DEVICE_DT_GET(DT_ALIAS(encoder_motor1));
	default:
		return NULL;
	}
}

/* Get PWM motor devices from device tree aliases */
static const struct device *get_pwm_motor_device(uint8_t id)
{
	switch (id) {
	case 0:
		return pwm_bldc_get_device("pwm-motor0");
	case 1:
		return pwm_bldc_get_device("pwm-motor1");
	default:
		return NULL;
	}
}

static int cmd_version(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "FOC STM32G431 Firmware");
	shell_print(sh, "Version: 1.0.0");
	shell_print(sh, "Build: %s %s", __DATE__, __TIME__);

	return 0;
}

static int cmd_encoder_read(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	uint8_t encoder_id = 0;
	uint16_t raw_angle;
	float angle_deg;
	float angle_rad;
	int ret;

	/* Parse encoder ID argument (optional, default is 0) */
	if (argc >= 2) {
		encoder_id = (uint8_t)atoi(argv[1]);
		if (encoder_id > 1) {
			shell_error(sh, "Invalid encoder ID: %d (valid: 0 or 1)", encoder_id);
			return -EINVAL;
		}
	}

	/* Get encoder device */
	dev = get_encoder_device(encoder_id);
	if (!dev) {
		shell_error(sh, "Encoder %d device not found", encoder_id);
		return -ENODEV;
	}

	if (!device_is_ready(dev)) {
		shell_error(sh, "Encoder %d device not ready", encoder_id);
		return -ENODEV;
	}

	/* Read raw angle */
	ret = mt6701_read_angle_raw(dev, &raw_angle);
	if (ret < 0) {
		shell_error(sh, "Failed to read encoder %d: error %d", encoder_id, ret);
		return ret;
	}

	/* Read angle in degrees */
	ret = mt6701_read_angle_deg(dev, &angle_deg);
	if (ret < 0) {
		shell_error(sh, "Failed to convert angle: error %d", ret);
		return ret;
	}

	/* Read angle in radians */
	ret = mt6701_read_angle_rad(dev, &angle_rad);
	if (ret < 0) {
		shell_error(sh, "Failed to convert angle: error %d", ret);
		return ret;
	}

	/* Display results - no float printf, split into integer and fractional parts */
	int deg_int = (int)angle_deg;
	int deg_frac = (int)((angle_deg - deg_int) * 100);
	int rad_int = (int)angle_rad;
	int rad_frac = (int)((angle_rad - rad_int) * 10000);

	shell_print(sh, "Encoder %d [%s]:", encoder_id, dev->name);
	shell_print(sh, "  Raw:     %5u (0x%04X)", raw_angle, raw_angle);
	shell_print(sh, "  Degrees: %3d.%02d deg", deg_int, deg_frac);
	shell_print(sh, "  Radians: %d.%04d rad", rad_int, rad_frac);

	return 0;
}

static int cmd_i2c_scan(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *i2c_dev;
	uint8_t bus_id = 1;
	int devices_found = 0;

	/* Parse bus ID argument (optional, default is 1) */
	if (argc >= 2) {
		bus_id = (uint8_t)atoi(argv[1]);
	}

	/* Get I2C device */
	if (bus_id == 1) {
		i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	} else if (bus_id == 2) {
		i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));
	} else {
		shell_error(sh, "Invalid I2C bus: %d (valid: 1 or 2)", bus_id);
		return -EINVAL;
	}

	if (!device_is_ready(i2c_dev)) {
		shell_error(sh, "I2C%d device not ready", bus_id);
		return -ENODEV;
	}

	shell_print(sh, "Scanning I2C%d bus...", bus_id);
	shell_print(sh, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");

	for (uint8_t addr = 0; addr <= 0x7F; addr++) {
		if ((addr % 16) == 0) {
			shell_print(sh, "");
			shell_fprintf(sh, SHELL_NORMAL, "%02x: ", addr);
		}

		/* Try to write 0 bytes to check if device responds */
		struct i2c_msg msgs[1];
		uint8_t dummy = 0;

		msgs[0].buf = &dummy;
		msgs[0].len = 0;
		msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

		int ret = i2c_transfer(i2c_dev, &msgs[0], 1, addr);

		if (ret == 0) {
			shell_fprintf(sh, SHELL_NORMAL, "%02x ", addr);
			devices_found++;
		} else {
			shell_fprintf(sh, SHELL_NORMAL, "-- ");
		}
	}

	shell_print(sh, "\n");
	shell_print(sh, "Found %d device(s) on I2C%d", devices_found, bus_id);

	return 0;
}

static int cmd_adc_read(const struct shell *sh, size_t argc, char **argv)
{
#if defined(CONFIG_FOC_ADC_DMA)
	uint16_t values[CONFIG_FOC_ADC_NUM_CHANNELS];
	int ret;

	/* Read all ADC channels */
	ret = adc_dma_get_all_channels(values, CONFIG_FOC_ADC_NUM_CHANNELS);
	if (ret < 0) {
		shell_error(sh, "Failed to read ADC channels: %d", ret);
		return ret;
	}

	shell_print(sh, "ADC Channels:");
	for (uint8_t i = 0; i < CONFIG_FOC_ADC_NUM_CHANNELS; i++) {
		uint32_t mv = adc_dma_raw_to_mv(values[i]);
		int mv_int = mv / 1000;
		int mv_frac = (mv % 1000);
		shell_print(sh, "  CH%d: %4u (0x%03X) = %d.%03d V",
			    i + 1, values[i], values[i], mv_int, mv_frac);
	}

	return 0;
#else
	shell_error(sh, "ADC DMA not enabled");
	return -ENOTSUP;
#endif
}

static int cmd_pwm_set(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	uint8_t motor_id = 0;
	float duty_a, duty_b, duty_c;
	int ret;

	/* Check argument count */
	if (argc < 4) {
		shell_error(sh, "Usage: pwm set <motor_id> <duty_a> <duty_b> <duty_c>");
		shell_error(sh, "  motor_id: 0 or 1");
		shell_error(sh, "  duty_a/b/c: 0.0 to 100.0 (percent)");
		return -EINVAL;
	}

	/* Parse arguments */
	motor_id = (uint8_t)atoi(argv[1]);
	duty_a = (float)atof(argv[2]);
	duty_b = (float)atof(argv[3]);
	duty_c = (float)atof(argv[4]);

	/* Validate motor ID */
	if (motor_id > 1) {
		shell_error(sh, "Invalid motor ID: %d (valid: 0 or 1)", motor_id);
		return -EINVAL;
	}

	/* Get PWM device */
	dev = get_pwm_motor_device(motor_id);
	if (!dev) {
		shell_error(sh, "PWM motor %d device not found", motor_id);
		return -ENODEV;
	}

	if (!device_is_ready(dev)) {
		shell_error(sh, "PWM motor %d device not ready", motor_id);
		return -ENODEV;
	}

	/* Set duty cycles */
	ret = pwm_bldc_set_duty(dev, duty_a, duty_b, duty_c);
	if (ret < 0) {
		shell_error(sh, "Failed to set PWM duty: %d", ret);
		return ret;
	}

	/* Display confirmation */
	int a_int = (int)duty_a;
	int a_frac = (int)((duty_a - a_int) * 10);
	int b_int = (int)duty_b;
	int b_frac = (int)((duty_b - b_int) * 10);
	int c_int = (int)duty_c;
	int c_frac = (int)((duty_c - c_int) * 10);

	shell_print(sh, "Motor %d PWM set:", motor_id);
	shell_print(sh, "  Phase A: %3d.%01d%%", a_int, a_frac);
	shell_print(sh, "  Phase B: %3d.%01d%%", b_int, b_frac);
	shell_print(sh, "  Phase C: %3d.%01d%%", c_int, c_frac);

	return 0;
}

static int cmd_pwm_disable(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	uint8_t motor_id = 0;
	int ret;

	/* Parse motor ID argument (optional, default is 0) */
	if (argc >= 2) {
		motor_id = (uint8_t)atoi(argv[1]);
		if (motor_id > 1) {
			shell_error(sh, "Invalid motor ID: %d (valid: 0 or 1)", motor_id);
			return -EINVAL;
		}
	}

	/* Get PWM device */
	dev = get_pwm_motor_device(motor_id);
	if (!dev) {
		shell_error(sh, "PWM motor %d device not found", motor_id);
		return -ENODEV;
	}

	if (!device_is_ready(dev)) {
		shell_error(sh, "PWM motor %d device not ready", motor_id);
		return -ENODEV;
	}

	/* Disable PWM */
	ret = pwm_bldc_disable(dev);
	if (ret < 0) {
		shell_error(sh, "Failed to disable PWM: %d", ret);
		return ret;
	}

	shell_print(sh, "Motor %d PWM disabled (all phases set to 0%%)", motor_id);

	return 0;
}

static int cmd_pwm_vector(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	uint8_t motor_id = 0;
	float angle, amplitude;
	int ret;

	/* Check argument count */
	if (argc < 4) {
		shell_error(sh, "Usage: pwm vector <motor_id> <angle> <amplitude>");
		shell_error(sh, "  motor_id: 0 or 1");
		shell_error(sh, "  angle: 0.0 to 360.0 degrees (electrical angle)");
		shell_error(sh, "  amplitude: 0.0 to 100.0 (percent)");
		return -EINVAL;
	}

	/* Parse arguments */
	motor_id = (uint8_t)atoi(argv[1]);
	angle = (float)atof(argv[2]);
	amplitude = (float)atof(argv[3]);

	/* Validate motor ID */
	if (motor_id > 1) {
		shell_error(sh, "Invalid motor ID: %d (valid: 0 or 1)", motor_id);
		return -EINVAL;
	}

	/* Get PWM device */
	dev = get_pwm_motor_device(motor_id);
	if (!dev) {
		shell_error(sh, "PWM motor %d device not found", motor_id);
		return -ENODEV;
	}

	if (!device_is_ready(dev)) {
		shell_error(sh, "PWM motor %d device not ready", motor_id);
		return -ENODEV;
	}

	/* Set vector */
	ret = pwm_bldc_set_vector(dev, angle, amplitude);
	if (ret < 0) {
		shell_error(sh, "Failed to set PWM vector: %d", ret);
		return ret;
	}

	/* Display confirmation */
	int angle_int = (int)angle;
	int angle_frac = (int)((angle - angle_int) * 10);
	int amp_int = (int)amplitude;
	int amp_frac = (int)((amplitude - amp_int) * 10);

	shell_print(sh, "Motor %d vector set:", motor_id);
	shell_print(sh, "  Angle: %3d.%01d degrees", angle_int, angle_frac);
	shell_print(sh, "  Amplitude: %3d.%01d%%", amp_int, amp_frac);

	return 0;
}

static int cmd_pwm_phase(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	uint8_t motor_id = 0;
	uint8_t phase;
	float duty;
	int ret;

	/* Check argument count */
	if (argc < 4) {
		shell_error(sh, "Usage: pwm phase <motor_id> <phase> <duty>");
		shell_error(sh, "  motor_id: 0 or 1");
		shell_error(sh, "  phase: 0 (A), 1 (B), or 2 (C)");
		shell_error(sh, "  duty: 0.0 to 100.0 (percent)");
		return -EINVAL;
	}

	/* Parse arguments */
	motor_id = (uint8_t)atoi(argv[1]);
	phase = (uint8_t)atoi(argv[2]);
	duty = (float)atof(argv[3]);

	/* Validate arguments */
	if (motor_id > 1) {
		shell_error(sh, "Invalid motor ID: %d (valid: 0 or 1)", motor_id);
		return -EINVAL;
	}

	if (phase > 2) {
		shell_error(sh, "Invalid phase: %d (valid: 0-2)", phase);
		return -EINVAL;
	}

	/* Get PWM device */
	dev = get_pwm_motor_device(motor_id);
	if (!dev) {
		shell_error(sh, "PWM motor %d device not found", motor_id);
		return -ENODEV;
	}

	if (!device_is_ready(dev)) {
		shell_error(sh, "PWM motor %d device not ready", motor_id);
		return -ENODEV;
	}

	/* Set phase duty cycle */
	ret = pwm_bldc_set_phase_duty(dev, phase, duty);
	if (ret < 0) {
		shell_error(sh, "Failed to set PWM phase: %d", ret);
		return ret;
	}

	/* Display confirmation */
	const char *phase_name[] = {"A", "B", "C"};
	int duty_int = (int)duty;
	int duty_frac = (int)((duty - duty_int) * 10);

	shell_print(sh, "Motor %d Phase %s set to %d.%01d%%",
		    motor_id, phase_name[phase], duty_int, duty_frac);

	return 0;
}

static int cmd_help_custom(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "FOC Controller Commands:");
	shell_print(sh, "  version              - Show firmware version");
	shell_print(sh, "  encoder read [id]    - Read encoder angle (id: 0 or 1, default: 0)");
	shell_print(sh, "  adc read             - Read all ADC channels");
	shell_print(sh, "  i2c scan [bus]       - Scan I2C bus (bus: 1 or 2, default: 1)");
	shell_print(sh, "  pwm vector <id> <a> <m>  - Set 3-phase vector (120deg spacing)");
	shell_print(sh, "  pwm set <id> <a> <b> <c> - Set PWM duty cycles (0-100%%)");
	shell_print(sh, "  pwm phase <id> <p> <d>   - Set single phase duty cycle");
	shell_print(sh, "  pwm disable [id]     - Disable PWM outputs (id: 0 or 1, default: 0)");
	shell_print(sh, "");
	shell_print(sh, "Built-in commands:");
	shell_print(sh, "  help                 - Show all available commands");
	shell_print(sh, "  clear                - Clear screen");
	shell_print(sh, "  history              - Show command history");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_encoder,
	SHELL_CMD_ARG(read, NULL, "Read encoder angle [id: 0|1]", cmd_encoder_read, 1, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_adc,
	SHELL_CMD_ARG(read, NULL, "Read all ADC channels", cmd_adc_read, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_i2c,
	SHELL_CMD_ARG(scan, NULL, "Scan I2C bus [1|2]", cmd_i2c_scan, 1, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_pwm,
	SHELL_CMD_ARG(vector, NULL, "Set 3-phase vector <id> <angle> <amp>", cmd_pwm_vector, 4, 0),
	SHELL_CMD_ARG(set, NULL, "Set PWM duty <id> <a> <b> <c>", cmd_pwm_set, 5, 0),
	SHELL_CMD_ARG(phase, NULL, "Set phase duty <id> <phase> <duty>", cmd_pwm_phase, 4, 0),
	SHELL_CMD_ARG(disable, NULL, "Disable PWM [id: 0|1]", cmd_pwm_disable, 1, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(version, NULL, "Show firmware version", cmd_version);
SHELL_CMD_REGISTER(encoder, &sub_encoder, "Encoder commands", NULL);
SHELL_CMD_REGISTER(adc, &sub_adc, "ADC commands", NULL);
SHELL_CMD_REGISTER(i2c, &sub_i2c, "I2C commands", NULL);
SHELL_CMD_REGISTER(pwm, &sub_pwm, "PWM motor control commands", NULL);
SHELL_CMD_REGISTER(foc, NULL, "Show FOC-specific help", cmd_help_custom);
