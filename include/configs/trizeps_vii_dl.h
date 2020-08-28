/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2020 Kai Ruhnau <kai.ruhnau@target-sg.com>
 *
 * Configuration settings for the Trizeps VII i.MX6DL SoM
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CONFIG_SYS_MALLOC_LEN (3 * SZ_1M)

#define CONFIG_MXC_UART_BASE UART2_BASE

#define CONFIG_EXTRA_ENV_SETTINGS				\
	"fdt_high=0xffffffff\0"					\
	"rootdev=/dev/mmcblk0p2\0"				\
	"mmcload=load mmc 3:1 ${loadaddr} fitImage\0"		\
	"mmcargs=setenv bootargs root=${rootdev} rw rootwait "	\
		"rootfstype=ext4\0"				\
	"miscargs=setenv bootargs ${bootargs} "			\
		"nohlt panic=1 quiet consoleblank=0 "		\
		"vt.global_cursor_default=0 "			\
		"systemd.mask=getty@tty1.service "		\
		"console=ttymxc1,${baudrate}\0"			\
	"mmc_mmc="						\
		"run mmcload mmcargs miscargs ; "		\
		"bootm ${loadaddr}\0"

#define CONFIG_BOOTCOMMAND "run mmc_mmc"

#define CONFIG_SYS_MEMTEST_START PHYS_SDRAM
#define CONFIG_SYS_MEMTEST_END (CONFIG_SYS_MEMTEST_START + 0x10000)

#define PHYS_SDRAM MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM_SIZE SZ_1G

#define CONFIG_SYS_SDRAM_BASE PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_SYS_FSL_ESDHC_ADDR USDHC4_BASE_ADDR

#define CONFIG_SYS_MMC_ENV_DEV 3

#endif /* __CONFIG_H */
