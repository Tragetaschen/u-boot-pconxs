// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 Kai Ruhnau <kai.ruhnau@target-sg.com>
 */

#include <asm/arch/mx6-pins.h>
#include <asm/mach-imx/sys_proto.h>
#include <power/pmic.h>
#include <power/rn5t618_pmic.h>
#include <led.h>
#include <fuse.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE | \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm  | PAD_CTL_SRE_FAST | PAD_CTL_HYS)
static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_GPIO1_IO04__UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_GPIO1_IO05__UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

int board_early_init_f(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	return 0;
}

int power_init_board(void)
{
	struct udevice *dev;
	int ret;

	ret = led_get_by_label("power-hold", &dev);
	if (ret != 0)
		return ret;
	ret = led_set_state(dev, LEDST_ON);
	if (ret != 0)
		return ret;

	ret = led_get_by_label("reset-off", &dev);
	if (ret != 0)
		return ret;
	ret = led_set_state(dev, LEDST_ON);
	if (ret != 0)
		return ret;

	ret = led_get_by_label("battery-communication-enable", &dev);
	if (ret != 0)
		return ret;
	ret = led_set_state(dev, LEDST_ON);
	if (ret != 0)
		return ret;

	ret = pmic_get("rn5t618@32", &dev);
	if (ret != 0)
		return ret;

	/* Disable auto power-off */
	ret = pmic_reg_write(dev, RN5T618_NOETIMSETCNT, 0);
	if (ret != 0)
		return ret;
	ret = pmic_reg_write(dev, RN5T618_PWRONTIMSET, 0x88);
	if (ret != 0)
		return ret;

	return 0;
}

int checkboard(void)
{
	puts("Board: K&K Trizeps VII SX\n");

	return 0;
}

#include "../common.c"

