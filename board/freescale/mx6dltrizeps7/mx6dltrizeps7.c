/*
 * Copyright (C) 2015 Target Systemelektronik
 *
 * Author: Kai Ruhnau <kai.ruhnau@target-sg.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/imx-common/spi.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <asm/imx-common/spi.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <asm/arch/crm_regs.h>

#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../common/pfuze.h"

#include <linux/fb.h>
#include "../drivers/video/mxcfb.h"
#include <ipu_pixfmt.h>

DECLARE_GLOBAL_DATA_PTR;

#define CLKCTL_CCGR2 0x70

static u32 system_rev;

static void usbotg_init(void);

#ifdef CONFIG_CMD_I2C
static void setup_iomux_i2c(unsigned int);
#endif
static void setup_iomux_gpio(void);
#ifdef CONFIG_VIDEO_IPUV3
static void setup_iomux_ipu1(void);
#endif
static void setup_iomux_pwm(void);
static void periphery_reset(void);

#ifdef CONFIG_VIDEO_IPUV3
static void enable_lvds(struct display_info_t const* dev);
#endif

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);
	return 0;
}

#define UART_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS \
)

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

#ifdef CONFIG_FSL_ESDHC

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

#endif

static iomux_v3_cfg_t const power_pads[] = {
	// i2c_batt_enable
	MX6_PAD_EIM_D28__GPIO3_IO28 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	// power on
	MX6_PAD_EIM_D29__GPIO3_IO29 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

#define I2C_PMIC 1
int power_init_board(void)
{
	struct pmic *p;
	unsigned int ret;

	imx_iomux_v3_setup_multiple_pads(power_pads, ARRAY_SIZE(power_pads));
	gpio_direction_output(IMX_GPIO_NR(3, 29), 1);

	p = pfuze_common_init(I2C_PMIC);
	if (!p)
		return -ENODEV;

	ret = pfuze_mode_init(p, APS_PFM);
	if (ret < 0)
		return ret;

	gpio_direction_output(IMX_GPIO_NR(3, 28), 1);

	return 0;
}

u32 get_board_rev(void)
{
	system_rev = 0x63000;
	return system_rev;
}

int board_early_init_f(void)
{
	setup_iomux_gpio();
#ifdef CONFIG_VIDEO_IPUV3
	setup_iomux_ipu1();
#endif
	setup_iomux_pwm();
	setup_iomux_uart();
#ifdef CONFIG_CMD_I2C
	setup_iomux_i2c(I2C1_BASE_ADDR);
	setup_iomux_i2c(I2C2_BASE_ADDR);
#endif
	periphery_reset();
	usbotg_init();
	return 0;
}

static void periphery_reset(void)
{
#define PERIPHERY_RESET_OUT IMX_GPIO_NR(1, 6)
	gpio_direction_output(PERIPHERY_RESET_OUT, 0);
	gpio_direction_output(PERIPHERY_RESET_OUT, 1);
}

static iomux_v3_cfg_t const usb_otg_pads[] = {
	MX6_PAD_NANDF_D0__GPIO2_IO00 | MUX_PAD_CTRL(GPIO_PAD_CTRL), // usb_otg_pen
	MX6_PAD_NANDF_RB0__GPIO6_IO10 | MUX_PAD_CTRL(GPIO_PAD_CTRL), // usb_otg_oc
	MX6_PAD_GPIO_1__USB_OTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL), // usb_otg_id
};

static void usbotg_init(void) {
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

	clrsetbits_le32(&iomuxc_regs->gpr[1],
			IOMUXC_GPR1_OTG_ID_MASK,
			IOMUXC_GPR1_OTG_ID_GPIO1);

	imx_iomux_v3_setup_multiple_pads(usb_otg_pads, ARRAY_SIZE(usb_otg_pads));

	gpio_direction_output(IMX_GPIO_NR(2, 0), 1);
}

int board_init(void)
{
	gd->bd->bi_arch_number = CONFIG_MACH_TYPE;
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"esdhc2", MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{NULL,   0},
};
#endif

static void fpga_reset(void)
{
	// pci nconfig
	gpio_direction_output(IMX_GPIO_NR(6, 7), 0);
	gpio_direction_output(IMX_GPIO_NR(7, 8), 0);
	udelay(50);
	gpio_direction_output(IMX_GPIO_NR(6, 7), 1);
	gpio_direction_output(IMX_GPIO_NR(7, 8), 1);
	udelay(50);
}

int board_late_init(void)
{
	fpga_reset();

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	return 0;
}

#ifdef CONFIG_CMD_I2C

#define I2C_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_HYS | \
	PAD_CTL_ODE | \
	PAD_CTL_SRE_FAST \
)

static void setup_iomux_i2c(unsigned int module_base) {
	switch (module_base) {
		case I2C1_BASE_ADDR:
			imx_iomux_v3_setup_pad(MX6_PAD_CSI0_DAT8__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL));
			imx_iomux_v3_setup_pad(MX6_PAD_CSI0_DAT9__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL));
			break;
		case I2C2_BASE_ADDR:
			imx_iomux_v3_setup_pad(MX6_PAD_KEY_ROW3__I2C2_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL));
			imx_iomux_v3_setup_pad(MX6_PAD_KEY_COL3__I2C2_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL));
			break;
		default:
			break;
	}
}
#endif

int checkboard(void)
{
	printf("Board: " CONFIG_BOARD_NAME "\n");
	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)
size_t  display_count = 1;
struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect = NULL,
	.enable = &enable_lvds,
	.mode	= {
		.name		= "New-DISP",
		.refresh	= 75,
		.xres		= 480,
		.yres		= 800,
		.pixclock	= 32552,
		.vsync_len	= 2,
		.upper_margin	= 2,
		.lower_margin	= 2,
		.hsync_len	= 10,
		.left_margin	= 10,
		.right_margin	= 8,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
		.flag		= 0
	}
}};

static iomux_v3_cfg_t const display_spi_pads[] = {
	MX6_PAD_EIM_EB2__GPIO2_IO30 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_EIM_EB3__GPIO2_IO31 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_EIM_A25__GPIO5_IO02 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	// backlight
	MX6_PAD_EIM_A21__GPIO2_IO17 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	// reset
	MX6_PAD_EIM_A20__GPIO2_IO18 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};
#define DISPLAY_MOSI IMX_GPIO_NR(2, 30)
#define DISPLAY_CLK IMX_GPIO_NR(2, 31)
#define DISPLAY_CS IMX_GPIO_NR(5, 2)

static u8 const display_enable_extended_commands[] = { 0xb9, 0xff, 0x83, 0x63, };
static u8 const display_set_power1[] = { 0xb1, 0x81, 0x24, 0x04, 0x02, 0x02, 0x03, 0x10, 0x10, 0x34, 0x3c, 0x3f, 0x3f, };
static u8 const display_sleep_out[] = { 0x11, };
// wait 5 msec
static u8 const display_display_inversion_off[] = { 0x20, };
static u8 const display_memory_access_control[] = { 0x36, 0x00, };
static u8 const display_interface_pixel_format[] = { 0x3a, 0x70 };
// wait 120 msec
static u8 const display_set_power2[] = { 0xb1, 0x78, 0x24, 0x04, 0x02, 0x02, 0x03, 0x10, 0x10, 0x34, 0x3c, 0x3f, 0x3f, };
static u8 const display_set_rgb_interface_related_register[] = { 0xb3, 0x01, };
static u8 const display_set_display_waveform_cycle[] = { 0xb4, 0x00, 0x08, 0x56, 0x07, 0x01, 0x01, 0x4d, 0x01, 0x42 };
static u8 const display_set_panel[] = { 0xcc, 0x0b, };
static u8 const display_set_gamma_curve_related_setting[] = { 0xe0, 0x01, 0x48, 0x4d, 0x4e, 0x58, 0xf6, 0x0b, 0x4e, 0x12, 0xd5, 0x15, 0x95, 0x55, 0x8e, 0x11, 0x01, 0x48, 0x4d, 0x55, 0x5f, 0xfd, 0x0a, 0x4e, 0x51, 0xd3, 0x17, 0x95, 0x96, 0x4e, 0x11, };
// wait 5 msec
static u8 const display_display_on[] = { 0x29 };

static void send_display_command(u8 const* bytes, int length)
{
	int i, bit;
	for (i=0; i<length; ++i)
	{
		udelay(50);
		gpio_direction_output(DISPLAY_CLK, 0);
		gpio_direction_output(DISPLAY_MOSI, i != 0);
		udelay(50);
		gpio_direction_output(DISPLAY_CLK, 1);
		for (bit=7; bit >= 0; --bit)
		{
			udelay(50);
			gpio_direction_output(DISPLAY_CLK, 0);
			gpio_direction_output(DISPLAY_MOSI, (bytes[i] & (1<<bit))>>bit);
			udelay(50);
			gpio_direction_output(DISPLAY_CLK, 1);
		}
	}
}

#define SEND(array) send_display_command(array, ARRAY_SIZE(array))

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT | IOMUXC_GPR2_DATA_WIDTH_CH1_24BIT;
	writel(reg, &iomux->gpr[2]);

	imx_iomux_v3_setup_multiple_pads(display_spi_pads, ARRAY_SIZE(display_spi_pads));
	
	// Wait 2 frames or more
	udelay(50000);
	gpio_direction_output(DISPLAY_CS, 0);

	SEND(display_enable_extended_commands);
	SEND(display_set_power1);
	SEND(display_sleep_out);
	udelay(5000);
	SEND(display_display_inversion_off);
	SEND(display_memory_access_control);
	SEND(display_interface_pixel_format);
	udelay(120000);
	SEND(display_set_power2);
	SEND(display_set_rgb_interface_related_register);
	SEND(display_set_display_waveform_cycle);
	SEND(display_set_panel);
	SEND(display_set_gamma_curve_related_setting);
	udelay(5000);
	SEND(display_display_on);

	gpio_direction_output(DISPLAY_CS, 1);

	// backlight on
	gpio_direction_output(IMX_GPIO_NR(2, 17), 1);
}

#endif

int overwrite_console(void)
{
	return 1;
}

static iomux_v3_cfg_t const gpio_pads[] = {
	// Keyboard
	MX6_PAD_CSI0_DAT11__GPIO5_IO29 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_EIM_WAIT__GPIO5_IO00 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_GPIO_8__GPIO1_IO08 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_GPIO_7__GPIO1_IO07 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	// LEDs
	MX6_PAD_EIM_A24__GPIO5_IO04 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_EIM_A23__GPIO6_IO06 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_EIM_A22__GPIO2_IO16 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	// buzzer enable
	MX6_PAD_EIM_D27__GPIO3_IO27 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	// vibrator
	MX6_PAD_EIM_D30__GPIO3_IO30 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	// PCIe
	MX6_PAD_NANDF_CLE__GPIO6_IO07 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_SD3_RST__GPIO7_IO08 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_EIM_DA13__GPIO3_IO13 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_EIM_DA2__GPIO3_IO02 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	// 3ax int1
	MX6_PAD_NANDF_D5__GPIO2_IO05 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	// reset
	MX6_PAD_GPIO_6__GPIO1_IO06 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	// temp int
	MX6_PAD_EIM_DA5__GPIO3_IO05 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	// debug
	MX6_PAD_EIM_D23__GPIO3_IO23 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_EIM_D24__GPIO3_IO24 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_EIM_D25__GPIO3_IO25 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_EIM_D26__GPIO3_IO26 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	// OLED Reset
	MX6_PAD_KEY_COL4__GPIO4_IO14 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	

	// Unknown

	MX6_PAD_NANDF_D1__GPIO2_IO01 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_D7__GPIO2_IO07 | MUX_PAD_CTRL(GPIO_PAD_CTRL),

	MX6_PAD_GPIO_19__GPIO4_IO05 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_KEY_COL1__GPIO4_IO08 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_KEY_ROW1__GPIO4_IO09 | MUX_PAD_CTRL(GPIO_PAD_CTRL),

	MX6_PAD_NANDF_CS0__GPIO6_IO11 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_CS3__GPIO6_IO16 | MUX_PAD_CTRL(GPIO_PAD_CTRL),

	MX6_PAD_SD3_DAT2__GPIO7_IO06 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_GPIO_16__GPIO7_IO11 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_GPIO_18__GPIO7_IO13 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static void setup_iomux_gpio() {
	imx_iomux_v3_setup_multiple_pads(gpio_pads, ARRAY_SIZE(gpio_pads));

	gpio_direction_output(IMX_GPIO_NR(5, 4), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 6), 1);
	gpio_direction_output(IMX_GPIO_NR(2, 16), 1);
	// buzzer enable
	gpio_direction_output(IMX_GPIO_NR(3, 27), 0);
	// vibrator disable
	gpio_direction_output(IMX_GPIO_NR(3, 30), 0);
	// fpga reset
	gpio_direction_output(IMX_GPIO_NR(6, 7), 1);
	gpio_direction_output(IMX_GPIO_NR(7, 8), 1);
}

#ifdef CONFIG_VIDEO_IPUV3

#define IPU1_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS \
)

static iomux_v3_cfg_t const ipu1_pads[] = {

	MX6_PAD_DISP0_DAT0__IPU1_DISP0_DATA00 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT1__IPU1_DISP0_DATA01 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT2__IPU1_DISP0_DATA02 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT3__IPU1_DISP0_DATA03 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT4__IPU1_DISP0_DATA04 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT5__IPU1_DISP0_DATA05 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT6__IPU1_DISP0_DATA06 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT7__IPU1_DISP0_DATA07 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT8__IPU1_DISP0_DATA08 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT9__IPU1_DISP0_DATA09 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT10__IPU1_DISP0_DATA10 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT11__IPU1_DISP0_DATA11 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT12__IPU1_DISP0_DATA12 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT13__IPU1_DISP0_DATA13 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT14__IPU1_DISP0_DATA14 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT15__IPU1_DISP0_DATA15 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT16__IPU1_DISP0_DATA16 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT17__IPU1_DISP0_DATA17 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT18__IPU1_DISP0_DATA18 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT19__IPU1_DISP0_DATA19 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT20__IPU1_DISP0_DATA20 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT21__IPU1_DISP0_DATA21 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT22__IPU1_DISP0_DATA22 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DISP0_DAT23__IPU1_DISP0_DATA23 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
	MX6_PAD_DI0_PIN4__IPU1_DI0_PIN04 | MUX_PAD_CTRL(IPU1_PAD_CTRL),
};

static void setup_iomux_ipu1() {
	imx_iomux_v3_setup_multiple_pads(ipu1_pads, ARRAY_SIZE(ipu1_pads));
}
#endif

#define PWM_PAD_CTRL (\
	PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS \
)

static iomux_v3_cfg_t const pwm_pads[] = {
	MX6_PAD_GPIO_9__PWM1_OUT | MUX_PAD_CTRL(PWM_PAD_CTRL),
};

static void setup_iomux_pwm() {
	imx_iomux_v3_setup_multiple_pads(pwm_pads, ARRAY_SIZE(pwm_pads));
}

