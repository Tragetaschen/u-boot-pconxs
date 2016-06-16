// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 Kai Ruhnau <kai.ruhnau@target-sg.com>
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <fdtdec.h>
#include <linux/libfdt.h>
#include <power/rn5t618_pmic.h>
#include <power/pmic.h>

static int rn5t618_reg_count(struct udevice *dev)
{
	return RN5T618_NUM_OF_REGS;
}

static int rn5t618_write(struct udevice *dev, uint reg, const uint8_t *buff,
			  int len)
{
	int ret;

	ret = dm_i2c_write(dev, reg, buff, len);
	if (ret) {
		debug("write error to device: %p register: %#x!\n", dev, reg);
		return ret;
	}

	return 0;
}

static int rn5t618_read(struct udevice *dev, uint reg, uint8_t *buff, int len)
{
	int ret;

	ret = dm_i2c_read(dev, reg, buff, len);
	if (ret) {
		debug("read error from device: %p register: %#x!\n", dev, reg);
		return ret;
	}

	return 0;
}

static struct dm_pmic_ops rn5t618_ops = {
	.reg_count = rn5t618_reg_count,
	.read = rn5t618_read,
	.write = rn5t618_write,
};

static const struct udevice_id rn5t618_ids[] = {
	{ .compatible = "ricoh,rn5t618" },
	{ }
};

U_BOOT_DRIVER(pmic_rn5t618) = {
	.name = "rn5t618-pmic",
	.id = UCLASS_PMIC,
	.of_match = rn5t618_ids,
	.ops = &rn5t618_ops,
};
