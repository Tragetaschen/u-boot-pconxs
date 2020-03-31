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

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

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

static int reset_fpga(void)
{
	struct udevice *dev;
	int ret;

	ret = led_get_by_label("fpga-reset", &dev);
	if (ret != 0)
		return ret;

	ret = led_set_state(dev, LEDST_OFF);
	if (ret != 0)
		return ret;

	udelay(50);

	ret = led_set_state(dev, LEDST_ON);
	return ret;
}

int board_init(void)
{
	int ret;

	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	ret = reset_fpga();
	if (ret != 0)
		return ret;

	return 0;
}

int checkboard(void)
{
	puts("Board: K&K Trizeps VII SX\n");

	return 0;
}

static int extract_mac_address(void)
{
	u32 value;
	unsigned char mac[6];
	char buf[ARP_HLEN_ASCII + 1];
	int ret;

	ret = fuse_read(4, 2, &value);
	if (ret != 0)
		return ret;

	mac[0] = value;
	mac[1] = value >> 8;
	mac[2] = value >> 16;
	mac[3] = value >> 24;

	ret = fuse_read(4, 3, &value);
	if (ret != 0)
		return ret;

	mac[4] = value;
	mac[5] = value >> 8;

	sprintf(buf, "%pM", mac);

	return env_set("ethaddr", buf);
}

int board_late_init(void)
{
	return extract_mac_address();
}
