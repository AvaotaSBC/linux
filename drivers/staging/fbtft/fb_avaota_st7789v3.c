// SPDX-License-Identifier: GPL-2.0+
/* Avaota A1 onboard ZJY114TC-IG01-HSD / ST7789V3 SPI status display. */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <video/mipi_display.h>

#include "fbtft.h"

#define DRVNAME "avaota-st7789"
#define AVAOTA_WIDTH 240
#define AVAOTA_HEIGHT 135
#define AVAOTA_MADCTL 0xa0

struct avaota_panel {
	u32 x_offset;
	u32 y_offset;
};

struct avaota_command {
	u8 command;
	u8 count;
	u8 data[14];
};

/* Avaota A1 hardware documentation, "onboard SPI display" initialization. */
static const struct avaota_command avaota_init[] = {
	{ 0x36, 1, { AVAOTA_MADCTL } },
	{ 0x3a, 1, { 0x05 } },
	{ 0xb2, 5, { 0x0c, 0x0c, 0x00, 0x33, 0x33 } },
	{ 0xb7, 1, { 0x35 } },
	{ 0xbb, 1, { 0x19 } },
	{ 0xc0, 1, { 0x2c } },
	{ 0xc2, 1, { 0x01 } },
	{ 0xc3, 1, { 0x12 } },
	{ 0xc4, 1, { 0x20 } },
	{ 0xc6, 1, { 0x0f } },
	{ 0xd0, 2, { 0xa4, 0xa1 } },
	{ 0xe0, 14, { 0xd0, 0x04, 0x0d, 0x11, 0x13, 0x2b, 0x3f,
		       0x54, 0x4c, 0x18, 0x0d, 0x0b, 0x1f, 0x23 } },
	{ 0xe1, 14, { 0xd0, 0x04, 0x0c, 0x11, 0x13, 0x2c, 0x3f,
		       0x44, 0x51, 0x2f, 0x1f, 0x1f, 0x20, 0x23 } },
	{ MIPI_DCS_ENTER_INVERT_MODE, 0, { } },
	{ MIPI_DCS_SET_DISPLAY_ON, 0, { } },
};

/* Use the core's kmalloc buffer, and propagate SPI failures during probe. */
static int avaota_write_command(struct fbtft_par *par, u8 command,
				const u8 *data, size_t count)
{
	int ret;

	par->buf[0] = command;
	ret = fbtft_write_buf_dc(par, par->buf, 1, 0);
	if (ret < 0 || !count)
		return ret;

	memcpy(par->buf, data, count);
	return fbtft_write_buf_dc(par, par->buf, count, 1);
}

static int avaota_set_var(struct fbtft_par *par)
{
	/* The official 0xa0 scan direction is already landscape. */
	if (par->info->var.xres != AVAOTA_WIDTH ||
	    par->info->var.yres != AVAOTA_HEIGHT ||
	    par->info->var.bits_per_pixel != 16 ||
	    par->info->var.rotate || par->bgr)
		return -EINVAL;

	return 0;
}

static int avaota_init_display(struct fbtft_par *par)
{
	struct device *dev = par->info->device;
	struct avaota_panel *panel;
	unsigned int i;
	int ret;

	if (!par->spi || !par->gpio.reset || !par->gpio.dc)
		return -EINVAL;
	ret = avaota_set_var(par);
	if (ret)
		return ret;

	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	/* Board manufacturer's fb_st7789v.c uses (40,52) with MADCTL=0xa0.
	 * These are the 240x135 visible window within the controller RAM.
	 */
	panel->x_offset = 40;
	panel->y_offset = 52;
	device_property_read_u32(dev, "avaota,x-offset", &panel->x_offset);
	device_property_read_u32(dev, "avaota,y-offset", &panel->y_offset);
	if (panel->x_offset > 320 - AVAOTA_WIDTH ||
	    panel->y_offset > 240 - AVAOTA_HEIGHT)
		return -EINVAL;
	par->extra = panel;

	/* Reset GPIO uses flags GPIO_ACTIVE_HIGH: logical 0 is physical low. */
	gpiod_set_value_cansleep(par->gpio.reset, 0);
	msleep(100);
	gpiod_set_value_cansleep(par->gpio.reset, 1);
	msleep(100);

	ret = avaota_write_command(par, MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0);
	if (ret < 0)
		return ret;
	msleep(120);

	for (i = 0; i < ARRAY_SIZE(avaota_init); i++) {
		ret = avaota_write_command(par, avaota_init[i].command,
					   avaota_init[i].data,
					   avaota_init[i].count);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "initialization command 0x%02x failed\n",
					     avaota_init[i].command);
	}

	dev_info(dev, "ST7789V3 240x135 RGB565, window offset (%u,%u)\n",
		 panel->x_offset, panel->y_offset);
	return 0;
}

static void avaota_set_addr_win(struct fbtft_par *par, int xs, int ys,
				int xe, int ye)
{
	struct avaota_panel *panel = par->extra;

	xs += panel->x_offset;
	xe += panel->x_offset;
	ys += panel->y_offset;
	ye += panel->y_offset;
	write_reg(par, MIPI_DCS_SET_COLUMN_ADDRESS,
		  (xs >> 8) & 0xff, xs & 0xff, (xe >> 8) & 0xff, xe & 0xff);
	write_reg(par, MIPI_DCS_SET_PAGE_ADDRESS,
		  (ys >> 8) & 0xff, ys & 0xff, (ye >> 8) & 0xff, ye & 0xff);
	write_reg(par, MIPI_DCS_WRITE_MEMORY_START);
}

static int avaota_blank(struct fbtft_par *par, bool on)
{
	return avaota_write_command(par, on ? MIPI_DCS_SET_DISPLAY_OFF :
				    MIPI_DCS_SET_DISPLAY_ON, NULL, 0);
}

static struct fbtft_display display = {
	.regwidth = 8,
	.buswidth = 8,
	.width = AVAOTA_WIDTH,
	.height = AVAOTA_HEIGHT,
	.bpp = 16,
	.fps = 10,
	.backlight = 1,
	.fbtftops = {
		.init_display = avaota_init_display,
		.set_var = avaota_set_var,
		.set_addr_win = avaota_set_addr_win,
		.blank = avaota_blank,
	},
};

FBTFT_REGISTER_SPI_DRIVER(DRVNAME, "avaota", "a1-tft114", &display);

MODULE_DESCRIPTION("Avaota A1 onboard ST7789V3 240x135 SPI display");
MODULE_LICENSE("GPL");
