/*
 * Copyright (C) 2017 Target Systemelektronik
 *
 * Author: Kai Ruhnau <kai.ruhnau@target-sg.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <asm/arch/crm_regs.h>

#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../../freescale/common/pfuze.h"

#include <linux/fb.h>
#include "../drivers/video/mxcfb.h"
#include <ipu_pixfmt.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS \
)

#define USDHC_PAD_CTRL (\
	PAD_CTL_PUS_47K_UP | \
	PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_80ohm | \
	PAD_CTL_SRE_FAST  | \
	PAD_CTL_HYS \
)
#define GPIO_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS \
)

#define I2C_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_HYS | \
	PAD_CTL_ODE | \
	PAD_CTL_SRE_FAST \
)

#define PWM_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS \
)

int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};
static iomux_v3_cfg_t const uart2_pads[] = {
	MX6_PAD_SD3_DAT5__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT4__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};
static iomux_v3_cfg_t const uart4_pads[] = {
	MX6_PAD_KEY_COL0__UART4_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_KEY_ROW0__UART4_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
	imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
}

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
/* I2C1 for PMIC */
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_CSI0_DAT9__GPIO5_IO27 | PC,
		.gp = IMX_GPIO_NR(5, 27),
	},
	.sda = {
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_CSI0_DAT8__GPIO5_IO26 | PC,
		.gp = IMX_GPIO_NR(5, 26),
	},
};

static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12),
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | PC,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | PC,
		.gp = IMX_GPIO_NR(4, 13),
	},
};

static void periphery_reset(void)
{
#define PERIPHERY_RESET_OUT IMX_GPIO_NR(1, 6)
	gpio_direction_output(PERIPHERY_RESET_OUT, 0);
	gpio_direction_output(PERIPHERY_RESET_OUT, 1);
}

int board_early_init_f(void)
{
	setup_iomux_uart();
	periphery_reset();

	return 0;
}

static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX6_PAD_SD1_CLK__SD1_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__SD1_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT0__SD1_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT1__SD1_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT2__SD1_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT3__SD1_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_KEY_COL2__GPIO4_IO10 	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_GPIO_2__GPIO1_IO02 	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__SD4_DATA4	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__SD4_DATA5	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__SD4_DATA6	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__SD4_DATA7	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_ALE__SD4_RESET	| MUX_PAD_CTRL(USDHC_PAD_CTRL)
};

struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

int board_mmc_init(bd_t *bis)
{
	int ret;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

	for(index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
			case 0:
				imx_iomux_v3_setup_multiple_pads(usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
#define TRIZEPS7_WLAN_SHUTDOWN IMX_GPIO_NR(4, 10)
#define TRIZEPS7_WLAN_RESET IMX_GPIO_NR(1, 2)
				gpio_direction_output(TRIZEPS7_WLAN_RESET, 0);
				udelay(10);
				gpio_direction_output(TRIZEPS7_WLAN_SHUTDOWN, 0);
				udelay(20);
				gpio_direction_output(TRIZEPS7_WLAN_SHUTDOWN, 1);
				udelay(10);
				gpio_direction_output(TRIZEPS7_WLAN_RESET, 1);
				break;
			case 1:
				imx_iomux_v3_setup_multiple_pads(usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
				break;	
			default:
				printf("Warning: you configured more USDHC controllers"
				       "(%d) than supported by the board (%d)\n",
				       index + 1, CONFIG_SYS_FSL_USDHC_NUM);
				return -EINVAL;
		}
		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
		if (ret)
			return ret;
	}
	return 0;
}

static void reset_fpga(void)
{
	// pci nconfig
	gpio_direction_output(IMX_GPIO_NR(6, 7), 0);
	udelay(50);
	gpio_direction_output(IMX_GPIO_NR(6, 7), 1);
}

int board_init(void)
{
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

	reset_fpga();

	return 0;
}

static iomux_v3_cfg_t const power_pads[] = {
	MX6_PAD_EIM_D28__GPIO3_IO28 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

int power_init_board(void)
{
	imx_iomux_v3_setup_multiple_pads(power_pads, ARRAY_SIZE(power_pads));
	gpio_direction_output(IMX_GPIO_NR(3, 28), 1);

	struct pmic *p;

	p = pfuze_common_init(1);
	if (!p)
		return -ENODEV;

	return pfuze_mode_init(p, APS_PFM);
}

int checkboard(void)
{
	puts("Board: K&K Trizeps VII DL\n");

	return 0;
}

