#ifndef __MX6DLTRIZEPS7EVAL_CONFIG_H
#define  __MX6DLTRIZEPS7EVAL_CONFIG_H

#define CONFIG_MX6

#include "mx6_common.h"
#include <linux/sizes.h>

#define CONFIG_BOARD_NAME "MX6DL-TRIZEPS7-pConXS"
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define CONFIG_ARCH_CPU_INIT
#define CONFIG_SYS_VSNPRINTF

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define CONFIG_SYS_GENERIC_BOARD

//http://www.arm.linux.org.uk/developer/machines/list.php?id=4634
#define CONFIG_MACH_TYPE 4634

#define CONFIG_SYS_MX6_HCLK 24000000
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT

#define CONFIG_CMDLINE_TAG
#define CONFIG_REVISION_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG

#define CONFIG_SYS_MALLOC_LEN (2*SZ_1M)
#define CONFIG_SYS_GBL_DATA_SIZE 128
#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_PROMPT "Target U-Boot> "
#define CONFIG_SYS_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE 256
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT)+16)
#define CONFIG_SYS_MAXARGS 16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START 0x10000000
#define CONFIG_SYS_MEMTEST_END (CONFIG_SYS_MEMTEST_START + SZ_64K)

#define CONFIG_LOADADDR 0x12000000
#define CONFIG_SYS_LOAD_ADDR CONFIG_LOADADDR

#define CONFIG_SYS_HZ 1000

/* Serial port */
#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE UART2_BASE
#define CONFIG_BAUDRATE 115200
#define CONFIG_SYS_BAUDRATE_TABLE {9600, 19200, 38400, 57600, 115200}
/* /Serial port */

#define CONFIG_CMDLINE_EDITING
#define CONFIG_SYS_HUSH_PARSER 1
#define CONFIG_SYS_PROMPT_HUSH_PS2 "> "
#define CONFIG_ENV_OVERWRITE

/* skip android fastbbot settings */
/*
 * CONFIG_USB_DEVICE
 * ...
 * CONFIG_EXTRA_ENV_SETTINGS
**/

#include <config_cmd_default.h>
#define CONFIG_CMD_I2C
#define CONFIG_CMD_MMC
#define CONFIG_CMD_ENV
#define CONFIG_CMD_BMODE
#define CONFIG_CMD_FUSE

#undef CONFIG_CMD_SATA
#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY -1
#define CONFIG_ETHPRIME "FEC"


/* Ethernet */
#define CONFIG_TARGET_NATIVE_PHY_ACTIVE 1
#define CONFIG_TARGET_REVERSE_MAC_ADDRESS
#define CONFIG_FEC_MXC
#define CONFIG_FEC_MXC_PHYADDR 1
#define IMX_FEC_BASE ENET_BASE_ADDR
#define CONFIG_PHYLIB 
#if CONFIG_TARGET_NATIVE_PHY_ACTIVE
	#define CONFIG_PHY_SMSC
#else
	#define CONFIG_PHY_BROADCOM
#endif
#define CONFIG_MII
#define CONFIG_RMII
#define CONFIG_FEC_XCV_TYPE RMII
#define CONFIG_IPADDR 10.199.0.155
#define CONFIG_SERVERIP 10.199.0.78
#define CONFIG_NETMASK 255.255.0.0
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_ARP_TIMEOUT 200UL
#define CONFIG_NET_RETRY_COUNT 100
#define CONFIG_NET_MULTI 1
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_DNS
/* /Ethernet */

#define CONFIG_MXC_OCOTP
#define CONFIG_MXC_GPIO

#ifdef CONFIG_CMD_I2C
	#define CONFIG_SYS_I2C
	#define CONFIG_SYS_I2C_MXC
	#define CONFIG_HARD_I2C
	#define CONFIG_SYS_I2C_BASE I2C2_BASE_ADDR
	#define CONFIG_SYS_I2C_SPEED 100000
	#define CONFIG_SYS_I2C_SLAVE 0x1f
#endif

#ifdef CONFIG_CMD_MMC
	#define CONFIG_FSL_ESDHC
	#define CONFIG_FSL_USDHC
	#define CONFIG_SYS_FSL_ESDHC_ADDR 0

	#define CONFIG_MMC
	#define CONFIG_GENERIC_MMC
	#define CONFIG_BOUNCE_BUFFER

	#define CONFIG_SYS_FSL_USDHC_NUM 2
	#define CONFIG_DOS_PARTITION
	#define CONFIG_CMD_FAT
	#define CONFIG_CMD_EXT2

#endif

#define CONFIG_STACKSIZE SZ_128K

#define CONFIG_NR_DRAM_BANKS 1
#define PHYS_SDRAM MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM_SIZE SZ_1G
#define CONFIG_SYS_SDRAM_BASE PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define iomem_valid_addr(addr, size) \
	(addr >= PHYS_SDRAM && addr <= (PHYS_SDRAM + PHYS_SDRAM_SIZE))

#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_SECT_SIZE (8 * SZ_1K)
#define CONFIG_ENV_SIZE CONFIG_ENV_SECT_SIZE
#define CONFIG_ENV_OFFSET (768 * SZ_1K)
#define CONFIG_SYS_MMC_ENV_DEV 1

#define CONFIG_SYS_TEXT_BASE 0x17800000

#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_IMX_VIDEO_SKIP
#define CONFIG_IPUV3_CLK 260000000
#define CONFIG_VIDEO_LOGO
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE

#define CONFIG_OF_LIBFDT

#define CONFIG_DEFAULT_FDT_FILE		"imx6dl-trizeps7-pconxs.dtb"
#define CONFIG_CONSOLE_DEV		"ttymxc1"
#define CONFIG_MMCROOT			"/dev/mmcblk0p2"

#define CONFIG_EXTRA_ENV_SETTINGS \
	"autoload=0\0" \
	"script=boot.scr\0" \
	"uimage=uImage\0" \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"fdt_addr=0x18000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=1\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0" \
	"mmcargs=setenv bootargs "\
		"consoleblank=0 "\
		"console=${console},${baudrate} " \
		"root=${mmcroot}\0" \
	"loadbootscript=" \
		"fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loaduimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${uimage}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootm ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootm; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootm; " \
		"fi;\0"

#define CONFIG_BOOTCOMMAND \
	"mmc dev ${mmcdev};" \
	"if mmc rescan; then " \
		"if run loaduimage; then " \
			"run mmcboot; " \
		"fi; " \
	"fi"
#endif
