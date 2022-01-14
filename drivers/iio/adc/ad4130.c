// SPDX-License-Identifier: GPL-2.0+
/*
 * AD4130 SPI ADC driver
 *
 * Copyright 2018 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/adc/ad_sigma_delta.h>
#include <linux/iio/sysfs.h>

#define AD4130_8_NAME			"ad4130-8"

#define AD4130_REG_STATUS		0x00
#define AD4130_STATUS_POR_FLAG_MSK	BIT(4)

#define AD4130_REG_ADC_CONTROL		0x01
#define AD4130_CSB_EN_MASK		BIT(9)

#define AD4130_REG_DATA			0x02

#define AD4130_REG_IO_CONTROL		0x03
#define AD4130_INT_PIN_SEL_MASK		GENMASK(9, 8)
#define AD4130_INT_PIN_P1		0x2

#define AD4130_REG_ID			0x05

#define AD4130_REG_CHANNEL_X(x)		(0x09 + (x))
#define AD4130_CHANNEL_EN_MASK		BIT(23)

#define AD4130_MAX_CHANNELS		16
#define AD4130_RESET_CLK_COUNT		64

static const unsigned int ad4130_reg_size[] = {
	[AD4130_REG_STATUS] = 1,
	[AD4130_REG_ADC_CONTROL] = 2,
	[AD4130_REG_DATA] = 2,
	[AD4130_REG_IO_CONTROL] = 2,
	[AD4130_REG_ID] = 1,
	[
		AD4130_REG_CHANNEL_X(0)
		...
		AD4130_REG_CHANNEL_X(AD4130_MAX_CHANNELS)
	] = 3,
};

enum ad4130_id {
	ID_AD4130_8_24_WLCSP,
	ID_AD4130_8_24_LFCSP,
	ID_AD4130_8_16_WLCSP,
	ID_AD4130_8_16_LFCSP,
};

struct ad4130_chip_info {
	const char *name;
	u8 resolution;
};

struct ad4130_state {
	const struct ad4130_chip_info	*chip_info;
	struct ad_sigma_delta		sd;
};

static int ad4130_read(struct ad4130_state *st, unsigned int reg,
		       unsigned int *val)
{
	if (reg >= ARRAY_SIZE(ad4130_reg_size))
		return -EINVAL;

	return ad_sd_read_reg(&st->sd, reg, ad4130_reg_size[reg], val);
}

static int ad4130_write(struct ad4130_state *st, unsigned int reg,
			unsigned int val)
{
	if (reg >= ARRAY_SIZE(ad4130_reg_size))
		return -EINVAL;

	return ad_sd_write_reg(&st->sd, reg, ad4130_reg_size[reg], val);
}

static int ad4130_update_bits(struct ad4130_state *st, unsigned int reg,
			      unsigned int mask, unsigned int val)
{
	unsigned int tmp;
	int ret;

	ret = ad4130_read(st, reg, &tmp);
	if (ret)
		return ret;

	tmp &= ~mask;
	val &= mask;
	tmp |= val;

	return ad4130_write(st, reg, tmp);
}

#define ad4130_update_field_bits(st, reg, mask, val) \
	ad4130_update_bits(st, reg, mask, FIELD_PREP(mask, val))

static int ad4130_set_channel_enable(struct ad4130_state *st,
				     unsigned int channel, bool status)
{
	return ad4130_update_field_bits(st, AD4130_REG_CHANNEL_X(channel),
					AD4130_CHANNEL_EN_MASK,
					status);
}

static int ad4130_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad4130_state *st = iio_priv(indio_dev);

	if (readval)
		return ad4130_read(st, reg, readval);

	return ad4130_write(st, reg, writeval);
}

static int ad4130_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *scan_mask)
{
	struct ad4130_state *st = iio_priv(indio_dev);
	bool status;
	int ret;
	int i;

	for (i = 0; i < indio_dev->num_channels; i++) {
		status = test_bit(i, scan_mask);
		ret = ad4130_set_channel_enable(st, i, status);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct iio_info ad4130_info = {
	.update_scan_mode = ad4130_update_scan_mode,
	.debugfs_reg_access = ad4130_reg_access,
};

static const struct ad_sigma_delta_info ad4130_sigma_delta_info = {
	.has_registers = true,
	.addr_shift = 0,
	.read_mask = BIT(6),
	.data_reg = AD4130_REG_DATA,
	.irq_flags = IRQF_TRIGGER_FALLING,
};

static int ad4130_setup(struct ad4130_state *st)
{
	int ret;

	/*
	 * It is possible to select DOUT, CLK, P1, or INT (on WLCSP packaged
	 * chips) as interrupt pin.
	 *
	 * DOUT cannot be used as it is claimed by the SPI interface, and it
	 * cannot be used for FIFO interrupts.
	 *
	 * CLK can be used but it would prevent using external clocks.
	 *
	 * P1 can be used but it prevents using it as a GPIO.
	 * INT can be used on WLCSP but it uses the same configration as DOUT on
	 * LFCSP which cannot be used for FIFO interrupts.
	 *
	 * P1 was chosen because it exhibits the same behavior across
	 * all chip variants and it does not need any special handling.
	 */
	ret = ad4130_update_field_bits(st, AD4130_REG_IO_CONTROL,
				       AD4130_INT_PIN_SEL_MASK,
				       AD4130_INT_PIN_P1);
	if (ret)
		return ret;

	/*
	 * Switch to SPI 4-wire mode.
	 */
	ret = ad4130_update_field_bits(st, AD4130_REG_ADC_CONTROL,
				       AD4130_CSB_EN_MASK, 1);
	if (ret)
		return ret;

	return 0;
}

static int ad4130_soft_reset(struct ad4130_state *st)
{
	unsigned int val, timeout;
	int ret;

	ret = ad_sd_reset(&st->sd, AD4130_RESET_CLK_COUNT);
	if (ret)
		return ret;

	usleep_range(2000, 3000);

	timeout = 100;
	do {
		ret = ad4130_read(st, AD4130_REG_STATUS, &val);
		if (ret)
			return ret;

		if (!(val & AD4130_STATUS_POR_FLAG_MSK))
			return 0;

		usleep_range(2000, 3000);
	} while (--timeout);

	dev_err(&st->sd.spi->dev, "Soft reset failed\n");

	return -EIO;
}

static int ad4130_probe(struct spi_device *spi)
{
	const struct ad4130_chip_info *info;
	struct ad4130_state *st;
	struct iio_dev *indio_dev;
	int ret;

	info = device_get_match_data(&spi->dev);
	if (!info)
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->chip_info = info;

	ad_sd_init(&st->sd, indio_dev, spi, &ad4130_sigma_delta_info);
	st->sd.num_slots = AD4130_MAX_CHANNELS;
	spi_set_drvdata(spi, indio_dev);

	indio_dev->name = st->chip_info->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad4130_info;

	ret = ad4130_soft_reset(st);
	if (ret)
		return ret;

	ret = ad4130_setup(st);
	if (ret)
		return ret;

	ret = devm_ad_sd_setup_buffer_and_trigger(&spi->dev, indio_dev);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct ad4130_chip_info ad4130_chip_info_tbl[] = {
	[ID_AD4130_8_24_WLCSP] = {
		.name = AD4130_8_NAME,
		.resolution = 24,
	},
	[ID_AD4130_8_24_LFCSP] = {
		.name = AD4130_8_NAME,
		.resolution = 24,
	},
	[ID_AD4130_8_16_WLCSP] = {
		.name = AD4130_8_NAME,
		.resolution = 16,
	},
	[ID_AD4130_8_16_LFCSP] = {
		.name = AD4130_8_NAME,
		.resolution = 16,
	},
};

static const struct of_device_id ad4130_of_match[] = {
	{
		.compatible = "adi,ad4130-8-24-wlcsp",
		.data = &ad4130_chip_info_tbl[ID_AD4130_8_24_WLCSP],
	},
	{
		.compatible = "adi,ad4130-8-24-lfcsp",
		.data = &ad4130_chip_info_tbl[ID_AD4130_8_24_LFCSP],
	},
	{
		.compatible = "adi,ad4130-8-16-wlcsp",
		.data = &ad4130_chip_info_tbl[ID_AD4130_8_16_WLCSP],
	},
	{
		.compatible = "adi,ad4130-8-16-lfcsp",
		.data = &ad4130_chip_info_tbl[ID_AD4130_8_16_LFCSP],
	},
	{ },
};
MODULE_DEVICE_TABLE(of, ad4130_of_match);

static struct spi_driver ad4130_driver = {
	.driver = {
		.name = "ad4130",
		.of_match_table = ad4130_of_match,
	},
	.probe = ad4130_probe,
};
module_spi_driver(ad4130_driver);

MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4130 SPI driver");
MODULE_LICENSE("GPL");
