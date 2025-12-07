#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include "console.h"

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

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
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Encoder read command");
	shell_print(sh, "Note: Encoder reading requires device initialization");
	shell_print(sh, "This feature will be implemented when encoder devices are set up");

	return 0;
}

static int cmd_help_custom(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "FOC Controller Commands:");
	shell_print(sh, "  version              - Show firmware version");
	shell_print(sh, "  encoder read <id>    - Read encoder angle (id: 0 or 1)");
	shell_print(sh, "");
	shell_print(sh, "Built-in commands:");
	shell_print(sh, "  help                 - Show all available commands");
	shell_print(sh, "  clear                - Clear screen");
	shell_print(sh, "  history              - Show command history");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_encoder,
	SHELL_CMD(read, NULL, "Read encoder angle", cmd_encoder_read),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(version, NULL, "Show firmware version", cmd_version);
SHELL_CMD_REGISTER(encoder, &sub_encoder, "Encoder commands", NULL);
SHELL_CMD_REGISTER(foc, NULL, "Show FOC-specific help", cmd_help_custom);