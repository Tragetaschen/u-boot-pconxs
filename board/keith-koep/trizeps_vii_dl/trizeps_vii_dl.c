// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 Kai Ruhnau <kai.ruhnau@target-sg.com>
 */

#include <asm/arch/mx6-pins.h>
#include <asm/mach-imx/sys_proto.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../../freescale/common/pfuze.h"
#include <led.h>
#include <fuse.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE | \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm  | PAD_CTL_SRE_FAST | PAD_CTL_HYS)
static iomux_v3_cfg_t const uart2_pads[] = {
	MX6_PAD_SD3_DAT5__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT4__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

int board_early_init_f(void)
{
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
	return 0;
}

int power_init_board(void)
{
	struct udevice *dev;
	int ret;

	ret = pmic_get("pfuze100@8", &dev);
	if (ret != 0)
		return ret;

	return pfuze_mode_init(dev, APS_PFM);
}

int checkboard(void)
{
	puts("Board: K&K Trizeps VII DL\n");

	return 0;
}

#include "../common.c"

