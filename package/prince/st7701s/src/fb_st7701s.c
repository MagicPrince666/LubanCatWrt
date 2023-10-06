#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <video/mipi_display.h>

#include "fbtft.h"

#define DRVNAME "fb_st7701s"

#define DEFAULT_GAMMA \
	"70 2C 2E 15 10 09 48 33 53 0B 19 18 20 25\n" \
	"70 2C 2E 15 10 09 48 33 53 0B 19 18 20 25"


enum st7701s_command {
	PORCTRL = 0xB2,
	GCTRL = 0xB7,
	VCOMS = 0xBB,
	VDVVRHEN = 0xC2,
	VRHS = 0xC3,
	VDVS = 0xC4,
	VCMOFSET = 0xC5,
	PWCTRL1 = 0xD0,
	PVGAMCTRL = 0xE0,
	NVGAMCTRL = 0xE1,
};

#define MADCTL_BGR BIT(3) /* bitmask for RGB/BGR order */
#define MADCTL_MV BIT(5) /* bitmask for page/column order */
#define MADCTL_MX BIT(6) /* bitmask for column address order */
#define MADCTL_MY BIT(7) /* bitmask for page address order */

/* 60Hz for 16.6ms, configured as 2*16.6ms */
#define PANEL_TE_TIMEOUT_MS  33

static struct completion panel_te; /* completion for panel TE line */
static int irq_te; /* Linux IRQ for LCD TE line */

static irqreturn_t panel_te_handler(int irq, void *data)
{
	complete(&panel_te);
	return IRQ_HANDLED;
}

/*
 * init_tearing_effect_line() - init tearing effect line.
 * @par: FBTFT parameter object.
 *
 * Return: 0 on success, or a negative error code otherwise.
 */
static int init_tearing_effect_line(struct fbtft_par *par)
{
	struct device *dev = par->info->device;
	struct gpio_desc *te;
	int rc, irq;

	te = gpiod_get_optional(dev, "te", GPIOD_IN);
	if (IS_ERR(te))
		return dev_err_probe(dev, PTR_ERR(te), "Failed to request te GPIO\n");

	/* if te is NULL, indicating no configuration, directly return success */
	if (!te) {
		irq_te = 0;
		return 0;
	}

	irq = gpiod_to_irq(te);

	/* GPIO is locked as an IRQ, we may drop the reference */
	gpiod_put(te);

	if (irq < 0) {
		return irq;
	}

	irq_te = irq;
	init_completion(&panel_te);

	/* The effective state is high and lasts no more than 1000 microseconds */
	rc = devm_request_irq(dev, irq_te, panel_te_handler,
			      IRQF_TRIGGER_RISING, "TE_GPIO", par);
	if (rc) {
		return dev_err_probe(dev, rc, "TE IRQ request failed.\n");
	}

	disable_irq_nosync(irq_te);

	return 0;
}

static int init_display(struct fbtft_par *par)
{
	int rc;

	par->fbtftops.reset(par);

	rc = init_tearing_effect_line(par);
	if (rc) {
		return rc;
	}

	/* turn off sleep mode */
	write_reg(par, MIPI_DCS_EXIT_SLEEP_MODE);
	mdelay(120);

	write_reg(par, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x10);
	write_reg(par, 0xC0, 0x63, 0x00);
	write_reg(par, 0xC1, 0x11, 0x02);
	write_reg(par, 0xC2, 0x31, 0x08);
	write_reg(par, 0xCC, 0x10);
	write_reg(par, 0xCD, 0x08);
	write_reg(par, 0xB0, 0x40, 0x01, 0x46, 0x0D, 0x13, 0x09, 0x05, 0x09, 0x09, 0x1B, 0x07, 0x15, 0x12, 0x4C, 0x10, 0xC8);
	write_reg(par, 0xB1, 0x40, 0x02, 0x86, 0x0D, 0x13, 0x09, 0x05, 0x09, 0x09, 0x1F, 0x07, 0x15, 0x12, 0x15, 0x19, 0x08);
	write_reg(par, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x11, 0xB0, 0x50);
	write_reg(par, 0xB1, 0x75);
	write_reg(par, 0xB2, 0x07, 0xB3, 0x80, 0xB5, 0x47, 0xB7, 0x85, 0xB8, 0x21, 0xB9, 0x10, 0xC1, 0x78, 0xC2, 0x78, 0xD0,0x88);
	mdelay(120);

	write_reg(par, 0xE0, 0x00, 0x00, 0x02);
	write_reg(par, 0xE1, 0x08, 0x00, 0x0A, 0x00, 0x07, 0x00, 0x09, 0x00, 0x00, 0x33, 0x33);
	write_reg(par, 0xE2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	write_reg(par, 0xE3, 0x00, 0x00, 0x33, 0x33);
	write_reg(par, 0xE4, 0x44, 0x44);
	write_reg(par, 0xE5, 0x0E, 0x2D, 0xA0, 0xA0, 0x10, 0x2D, 0xA0, 0xA0, 0x0A, 0x2D, 0xA0, 0xA0, 0x0C, 0x2D, 0xA0, 0xA0);
	write_reg(par, 0xE6, 0x00, 0x00, 0x33, 0x33);
	write_reg(par, 0xE7, 0x44, 0x44);
	write_reg(par, 0xE8, 0x0D, 0x2D, 0xA0, 0xA0, 0x0F, 0x2D, 0xA0, 0xA0, 0x09, 0x2D, 0xA0, 0xA0, 0x0B, 0x2D, 0xA0, 0xA0);
	write_reg(par, 0xEB, 0x02, 0x01, 0xE4, 0xE4, 0x44, 0x00, 0x40);
	write_reg(par, 0xEC, 0x02, 0x01);
	write_reg(par, 0xED, 0xAB, 0x89, 0x76, 0x54, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x10, 0x45, 0x67, 0x98, 0xBA);
	write_reg(par, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x00);
	write_reg(par, 0x3A, 0x77);
	write_reg(par, 0x36, 0x00);

	write_reg(par, 0x29);
	return 0;
}

/**
 * set_var() - apply LCD properties like rotation and BGR mode
 *
 * @par: FBTFT parameter object
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int set_var(struct fbtft_par *par)
{
	u8 madctl_par = 0;

	if (par->bgr)
		madctl_par |= MADCTL_BGR;
	switch (par->info->var.rotate) {
	case 0:
		break;
	case 90:
		madctl_par |= (MADCTL_MV | MADCTL_MY);
		break;
	case 180:
		madctl_par |= (MADCTL_MX | MADCTL_MY);
		break;
	case 270:
		madctl_par |= (MADCTL_MV | MADCTL_MX);
		break;
	default:
		return -EINVAL;
	}
	write_reg(par, MIPI_DCS_SET_ADDRESS_MODE, madctl_par);
	return 0;
}

/**
 * set_gamma() - set gamma curves
 *
 * @par: FBTFT parameter object
 * @curves: gamma curves
 *
 * Before the gamma curves are applied, they are preprocessed with a bitmask
 * to ensure syntactically correct input for the display controller.
 * This implies that the curves input parameter might be changed by this
 * function and that illegal gamma values are auto-corrected and not
 * reported as errors.
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int set_gamma(struct fbtft_par *par, u32 *curves)
{
	int i;
	int j;
	int c; /* curve index offset */

	/*
	 * Bitmasks for gamma curve command parameters.
	 * The masks are the same for both positive and negative voltage
	 * gamma curves.
	 */
	static const u8 gamma_par_mask[] = {
		0xFF, /* V63[3:0], V0[3:0]*/
		0x3F, /* V1[5:0] */
		0x3F, /* V2[5:0] */
		0x1F, /* V4[4:0] */
		0x1F, /* V6[4:0] */
		0x3F, /* J0[1:0], V13[3:0] */
		0x7F, /* V20[6:0] */
		0x77, /* V36[2:0], V27[2:0] */
		0x7F, /* V43[6:0] */
		0x3F, /* J1[1:0], V50[3:0] */
		0x1F, /* V57[4:0] */
		0x1F, /* V59[4:0] */
		0x3F, /* V61[5:0] */
		0x3F, /* V62[5:0] */
	};

	for (i = 0; i < par->gamma.num_curves; i++) {
		c = i * par->gamma.num_values;
		for (j = 0; j < par->gamma.num_values; j++) {
			curves[c + j] &= gamma_par_mask[j];
		}
		write_reg(
			par, PVGAMCTRL + i,
			curves[c + 0], curves[c + 1], curves[c + 2],
			curves[c + 3], curves[c + 4], curves[c + 5],
			curves[c + 6], curves[c + 7], curves[c + 8],
			curves[c + 9], curves[c + 10], curves[c + 11],
			curves[c + 12], curves[c + 13]);
	}
	return 0;
}

/**
 * blank() - blank the display
 *
 * @par: FBTFT parameter object
 * @on: whether to enable or disable blanking the display
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int blank(struct fbtft_par *par, bool on)
{
	if (on) {
		write_reg(par, MIPI_DCS_SET_DISPLAY_OFF);
	} else {
		write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
	}
	return 0;
}

/*
 * write_vmem() - write data to display.
 * @par: FBTFT parameter object.
 * @offset: offset from screen_buffer.
 * @len: the length of data to be writte.
 *
 * Return: 0 on success, or a negative error code otherwise.
 */
static int write_vmem(struct fbtft_par *par, size_t offset, size_t len)
{
	struct device *dev = par->info->device;
	int ret;

	if (irq_te) {
		enable_irq(irq_te);
		reinit_completion(&panel_te);
		ret = wait_for_completion_timeout(&panel_te,
						  msecs_to_jiffies(PANEL_TE_TIMEOUT_MS));
		if (ret == 0){
			dev_err(dev, "wait panel TE timeout\n");
		}

		disable_irq(irq_te);
	}

	switch (par->pdata->display.buswidth) {
	case 8:
		ret = fbtft_write_vmem16_bus8(par, offset, len);
		break;
	case 9:
		ret = fbtft_write_vmem16_bus9(par, offset, len);
		break;
	case 16:
		ret = fbtft_write_vmem16_bus16(par, offset, len);
		break;
	default:
		dev_err(dev, "Unsupported buswidth %d\n",
			par->pdata->display.buswidth);
		ret = 0;
		break;
	}

	return ret;
}

static struct fbtft_display display = {
	.regwidth = 8,
	.width = 480,
	.height = 800,
	.gamma_num = 2,
	.gamma_len = 14,
	.gamma = DEFAULT_GAMMA,
	.fbtftops = {
		.init_display = init_display,
		.write_vmem = write_vmem,
		.set_var = set_var,
		.set_gamma = set_gamma,
		.blank = blank,
	},
};

FBTFT_REGISTER_DRIVER(DRVNAME, "sitronix,st7701s", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:st7701s");
MODULE_ALIAS("platform:st7701s");
MODULE_LICENSE("GPL");