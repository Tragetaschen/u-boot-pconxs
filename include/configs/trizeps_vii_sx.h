/*FIXME
 * Copyright (C) 2016 Marek Vasut <marex@denx.de>
 *
 * Configuration settings for the Trizeps VII i.MX6SX SoM
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_TRIZEPS_VII_SX_CONFIG_H__
#define __CONFIG_TRIZEPS_VII_SX_CONFIG_H__

#include "mx6_common.h"

#ifdef CONFIG_SPL
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT
#include "imx6_spl.h"
#endif

#define CONFIG_BOARD_EARLY_INIT_F

/* Memory configuration */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM_SIZE			SZ_1G
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_MEMTEST_START	PHYS_SDRAM
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x10000)
#define CONFIG_STACKSIZE		SZ_128K
#define CONFIG_SYS_MALLOC_LEN		(3 * SZ_1M)

#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FEC Ethernet */
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x1
#define CONFIG_FEC_MXC_FUSE_MAC_REVERSED
#define CONFIG_FEC_XCV_TYPE             RGMII
#define CONFIG_ETHPRIME                 "FEC"

#define CONFIG_PHYLIB
#define CONFIG_PHY_SMSC

/* I2C */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_SPEED		100000

/* I2C EEPROM */
#define CONFIG_CMD_EEPROM
#define CONFIG_SYS_I2C_EEPROM_BUS		1
#define CONFIG_SYS_I2C_EEPROM_ADDR		0x51
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN		1
#define CONFIG_SYS_I2C_EEPROM_ADDR_OVERFLOW	1
#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS	5
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS	20

/* MMC */
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC4_BASE_ADDR
#define CONFIG_SYS_FSL_USDHC_NUM	1

#define CONFIG_ENV_OFFSET		(8 * SZ_64K)
#define CONFIG_ENV_SIZE			SZ_8K
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV		0	/* USDHC4 */

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_RN5T618
#define CONFIG_POWER_RN5T618_I2C_ADDR	0x32

/* QSPI */
#define CONFIG_FSL_QSPI
#ifdef CONFIG_FSL_QSPI
#define CONFIG_SYS_FSL_QSPI_AHB
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED		40000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#define FSL_QSPI_FLASH_SIZE		SZ_32M
#define FSL_QSPI_FLASH_NUM		2
#endif

/* Thermal */
#define CONFIG_IMX_THERMAL

/* UART */
#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

/* USB */
#ifdef CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2
#endif

/* Boot Linux */
#define CONFIG_BOOTFILE		"fitImage"
#define CONFIG_BOOTARGS		"console=ttymxc0,115200"
#define CONFIG_BOOTCOMMAND	"run mmc_mmc"

/* Extra Environments */
#define CONFIG_PREBOOT		"run try_bootscript"
#define CONFIG_HOSTNAME		trizeps_vii_sx

#define CONFIG_EXTRA_ENV_SETTINGS					\
	"consdev=ttymxc0\0"						\
	"baudrate=115200\0"						\
	"fdt_high=0xffffffff\0"						\
	"bootscript=boot.scr\0"						\
	"bootdev=/dev/mmcblk1p1\0"					\
	"rootdev=/dev/mmcblk1p2\0"					\
	"netdev=eth0\0"							\
	"kernel_addr_r=0x82000000\0"					\
	"videomode=video=ctfb:x:480,y:800,depth:24,mode:0,pclk:32552,"	\
		"le:0,ri:256,up:2,lo:2,hs:2,vs:2,sync:100663296,"	\
		"vmode:0\0"						\
	"addcons="							\
		"setenv bootargs ${bootargs} "				\
		"console=${consdev},${baudrate}\0"			\
	"addip="							\
		"setenv bootargs ${bootargs} "				\
		"ip=${ipaddr}:${serverip}:${gatewayip}:"		\
			"${netmask}:${hostname}:${netdev}:off\0"	\
	"addmisc="							\
		"setenv bootargs ${bootargs} ${miscargs}\0"		\
	"addargs=run addcons addmisc\0"					\
	"mmcload="							\
		"mmc rescan ; "						\
		"load mmc 0:1 ${kernel_addr_r} ${bootfile}\0"		\
	"netload="							\
		"tftp ${kernel_addr_r} ${hostname}/${bootfile}\0"	\
	"miscargs=nohlt panic=1 quiet vt.global_cursor_default=0 "	\
		"systemd.mask=getty@tty1.service\0"			\
	"mmcargs=setenv bootargs root=${rootdev} rw rootwait "		\
		"rootfstype=ext4\0"					\
	"nfsargs="							\
		"test -z \"${nfsserverip}\" && "			\
			"setenv nfsserverip ${serverip} ; "		\
		"setenv bootargs root=/dev/nfs rw "			\
			"nfsroot=${nfsserverip}:${rootpath},v3,tcp\0"	\
	"mmc_mmc="							\
		"run mmcload mmcargs addargs ; "			\
		"bootm ${kernel_addr_r}\0"				\
	"mmc_nfs="							\
		"run mmcload nfsargs addip addargs ; "			\
		"bootm ${kernel_addr_r}\0"				\
	"net_mmc="							\
		"run netload mmcargs addargs ; "			\
		"bootm ${kernel_addr_r}\0"				\
	"net_nfs="							\
		"run netload nfsargs addip addargs ; "			\
		"bootm ${kernel_addr_r}\0"				\
	"try_bootscript="						\
		"mmc rescan;"						\
		"if test -e mmc 0:1 ${bootscript} ; then "		\
		"if load mmc 0:1 ${kernel_addr_r} ${bootscript};"	\
		"then ; "						\
			"echo Running bootscript... ; "			\
			"source ${kernel_addr_r} ; "			\
		"fi ; "							\
		"fi\0"

#endif		/* __CONFIG_TRIZEPS_VII_SX_CONFIG_H__ */
