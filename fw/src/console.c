#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <stdlib.h>
#include "console.h"
#include "mt6701.h"
#include "adc_dma.h"

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

static int cmd_help_custom(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "FOC Controller Commands:");
	shell_print(sh, "  version              - Show firmware version");
	shell_print(sh, "  encoder read [id]    - Read encoder angle (id: 0 or 1, default: 0)");
	shell_print(sh, "  adc read             - Read all ADC channels");
	shell_print(sh, "  i2c scan [bus]       - Scan I2C bus (bus: 1 or 2, default: 1)");
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

SHELL_CMD_REGISTER(version, NULL, "Show firmware version", cmd_version);
SHELL_CMD_REGISTER(encoder, &sub_encoder, "Encoder commands", NULL);
SHELL_CMD_REGISTER(adc, &sub_adc, "ADC commands", NULL);
SHELL_CMD_REGISTER(i2c, &sub_i2c, "I2C commands", NULL);
SHELL_CMD_REGISTER(foc, NULL, "Show FOC-specific help", cmd_help_custom);
