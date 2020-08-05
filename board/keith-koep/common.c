int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

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

