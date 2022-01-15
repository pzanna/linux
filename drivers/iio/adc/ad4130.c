// SPDX-License-Identifier: GPL-2.0+
/*
 * AD4130 SPI ADC driver
 *
 * Copyright 2018 Analog Devices Inc.
 */
#include <asm/unaligned.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/adc/ad_sigma_delta.h>
#include <linux/iio/sysfs.h>

#define AD4130_8_NAME			"ad4130-8"

#define AD4130_REG_COMMS		0x00
#define AD4130_COMMS_READ_MASK		BIT(6)
#define AD4130_COMMS_REG_MASK		GENMASK(5, 0)

#define AD4130_REG_STATUS		0x00
#define AD4130_STATUS_POR_FLAG_MSK	BIT(4)

#define AD4130_REG_ADC_CONTROL		0x01
#define AD4130_CSB_EN_MASK		BIT(9)
#define AD4130_MCLK_SEL_MASK		GENMASK(1, 0)
#define AD4130_MCLK_76_8KHZ		0x0
#define AD4130_MCLK_76_8KHZ_OUT		0x1
#define AD4130_MCLK_76_8KHZ_EXT		0x2
#define AD4130_MCLK_153_6KHZ_EXT	0x3

#define AD4130_REG_DATA			0x02

#define AD4130_REG_IO_CONTROL		0x03
#define AD4130_INT_PIN_SEL_MASK		GENMASK(9, 8)
#define AD4130_INT_PIN_DOUT_OR_INT	0x0
#define AD4130_INT_PIN_CLK		0x1
#define AD4130_INT_PIN_P1		0x2
#define AD4130_INT_PIN_DOUT		0x3
#define AD4130_GPIO_DATA_MASK		GENMASK(7, 4)
#define AD4130_GPIO_CTRL_MASK		GENMASK(3, 0)

#define AD4130_REG_ID			0x05

#define AD4130_REG_CHANNEL_X(x)		(0x09 + (x))
#define AD4130_CHANNEL_EN_MASK		BIT(23)

#define AD4130_REG_MISC			0x39
#define AD4130_STBY_OUT_EN_MASK		BIT(8)

#define AD4130_MAX_GPIOS		4
#define AD4130_P1_INDEX			0
#define AD4130_P3_INDEX			2

#define AD4130_MAX_CHANNELS		16
#define AD4130_RESET_CLK_COUNT		64
#define AD4130_RESET_BUF_SIZE		(AD4130_RESET_CLK_COUNT / 8)
#define AD4130_SOFT_RESET_SLEEP		2000
#define AD4130_SOFT_RESET_TIMEOUT	(AD4130_SOFT_RESET_SLEEP * 100)

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
	[AD4130_REG_MISC] = 2,
};

enum ad4130_id {
	ID_AD4130_8_24_WLCSP,
	ID_AD4130_8_24_LFCSP,
	ID_AD4130_8_16_WLCSP,
	ID_AD4130_8_16_LFCSP,
};

struct ad4130_chip_info {
	const char	*name;
	u8		resolution;
	bool		has_int_pin;
};

struct ad4130_state {
	const struct ad4130_chip_info	*chip_info;
	struct spi_device		*spi;
	struct regmap			*regmap;
	struct clk			*mclk;

	struct gpio_chip		gc;
	unsigned int			gpio_offsets[AD4130_MAX_GPIOS];
	unsigned int			num_gpios;

	u32			int_pin_sel;
	bool			dout_int_pin;
	u32			mclk_sel;
	bool			standby_out_en;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	u8			reset_buf[AD4130_RESET_BUF_SIZE];
	u8			reg_write_tx_buf[4] ____cacheline_aligned;
	u8			reg_read_tx_buf[1];
	u8			reg_read_rx_buf[3];
};

static const struct iio_chan_spec ad4130_channel_template = {
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.differential = 1,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	.scan_type = {
		.sign = 'u',
		.realbits = 24,
		.storagebits = 32,
		.endianness = IIO_BE,
	},
};

static int ad4130_get_reg_size(struct ad4130_state *st, unsigned int reg,
			       unsigned int *size)
{
	if (reg >= ARRAY_SIZE(ad4130_reg_size))
		return -EINVAL;

	*size = ad4130_reg_size[reg];

	return 0;
}

static int ad4130_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct ad4130_state *st = context;
	struct spi_transfer t = {
		.tx_buf = st->reg_write_tx_buf,
	};
	unsigned int size;
	int ret;

	ret = ad4130_get_reg_size(st, reg, &size);
	if (ret)
		return ret;

	st->reg_write_tx_buf[0] = FIELD_PREP(AD4130_COMMS_REG_MASK, reg);
	t.len = size + 1;

	switch (size) {
	case 3:
		put_unaligned_be24(val, &st->reg_write_tx_buf[1]);
		break;
	case 2:
		put_unaligned_be16(val, &st->reg_write_tx_buf[1]);
		break;
	case 1:
		st->reg_write_tx_buf[1] = val;
		break;
	default:
		return -EINVAL;
	}

	return spi_sync_transfer(st->spi, &t, 1);
}

static int ad4130_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct ad4130_state *st = context;
	struct spi_transfer t[2] = {
		{
			.tx_buf = st->reg_read_tx_buf,
			.len = sizeof(st->reg_read_tx_buf),
		},
		{
			.rx_buf = st->reg_read_rx_buf,
		},
	};
	unsigned int size;
	int ret;

	ret = ad4130_get_reg_size(st, reg, &size);
	if (ret)
		return ret;

	st->reg_read_tx_buf[0] = AD4130_COMMS_READ_MASK |
				 FIELD_PREP(AD4130_COMMS_REG_MASK, reg);
	t[1].len = size;

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret)
		return ret;

	switch (size) {
	case 3:
		*val = get_unaligned_be24(st->reg_read_rx_buf);
		break;
	case 2:
		*val = get_unaligned_be16(st->reg_read_rx_buf);
		break;
	case 1:
		*val = st->reg_read_rx_buf[0];
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return 0;
}

static const struct regmap_config ad4130_regmap_config = {
	.reg_read = ad4130_reg_read,
	.reg_write = ad4130_reg_write,
};

static int ad4130_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	return 0;
}

static int ad4130_gpio_direction_input(struct gpio_chip *gc,
				       unsigned int offset)
{
	return 0;
}

static int ad4130_gpio_direction_output(struct gpio_chip *gc,
					unsigned int offset, int value)
{
	return 0;
}

static int ad4130_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	return 0;
}

static int ad4130_gpio_get_multiple(struct gpio_chip *gc, unsigned long *mask,
				    unsigned long *bits)
{
	return 0;
}

static void ad4130_gpio_set(struct gpio_chip *gc, unsigned int offset,
			    int value)
{
}

static void ad4130_gpio_set_multiple(struct gpio_chip *gc, unsigned long *mask,
				     unsigned long *bits)
{
}

static int ad4130_set_channel_enable(struct ad4130_state *st,
				     unsigned int channel, bool status)
{
	return regmap_update_bits(st->regmap, AD4130_REG_CHANNEL_X(channel),
				  AD4130_CHANNEL_EN_MASK,
				  FIELD_PREP(AD4130_CHANNEL_EN_MASK, status));
}

static irqreturn_t ad4130_irq_handler(int irq, void *private)
{
	return IRQ_HANDLED;
}

static int ad4130_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_RAW:
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad4130_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad4130_state *st = iio_priv(indio_dev);

	if (readval)
		return ad4130_reg_read(st, reg, readval);

	return ad4130_reg_write(st, reg, writeval);
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
	.read_raw = ad4130_read_raw,
	.update_scan_mode = ad4130_update_scan_mode,
	.debugfs_reg_access = ad4130_reg_access,
};

static int ad4310_parse_fw(struct ad4130_state *st)
{
	bool disabled_gpios[AD4130_MAX_GPIOS] = {0};
	struct device *dev = &st->spi->dev;
	int ret;
	int i;

	st->int_pin_sel = AD4130_INT_PIN_CLK;
	device_property_read_u32(dev, "adi,int-pin-sel", &st->int_pin_sel);

	if (st->int_pin_sel < AD4130_INT_PIN_DOUT_OR_INT ||
	    st->int_pin_sel > AD4130_INT_PIN_DOUT) {
		dev_err(dev, "Invalid interrupt pin %u\n", st->int_pin_sel);
		return -EINVAL;
	}

	if (st->int_pin_sel == AD4130_INT_PIN_DOUT ||
	    (st->int_pin_sel == AD4130_INT_PIN_DOUT_OR_INT &&
	     !st->chip_info->has_int_pin)) {
		dev_err(dev, "FIFO functionality disabled\n");
		st->dout_int_pin = true;
	}

	if (st->int_pin_sel == AD4130_INT_PIN_P1)
		disabled_gpios[AD4130_P1_INDEX] = true;

	st->standby_out_en = device_property_read_bool(dev, "adi,stby-out-en");

	if (st->standby_out_en)
		disabled_gpios[AD4130_P3_INDEX] = true;

	ret = device_property_read_u32(dev, "adi,mclk-sel", &st->mclk_sel);
	if (ret)
		st->mclk_sel = AD4130_MCLK_76_8KHZ;

	if (st->mclk_sel < AD4130_MCLK_76_8KHZ ||
	    st->mclk_sel > AD4130_MCLK_153_6KHZ_EXT) {
		dev_err(dev, "Invalid clock %u\n",
			st->mclk_sel);
		return -EINVAL;
	}

	if (st->int_pin_sel == AD4130_INT_PIN_CLK
	    && st->mclk_sel != AD4130_MCLK_76_8KHZ) {
		dev_err(dev, "Invalid clock %u for interrupt pin %u\n",
			st->mclk_sel, st->int_pin_sel);
		return -EINVAL;
	}

	for (i = 0; i < AD4130_MAX_GPIOS; i++)
		if (!disabled_gpios[i])
			st->gpio_offsets[st->num_gpios++] = i;

	return 0;
}

static int ad4130_setup(struct ad4130_state *st)
{
	int ret;

	/* Switch to SPI 4-wire mode. */
	ret = regmap_update_bits(st->regmap, AD4130_REG_ADC_CONTROL,
				 AD4130_CSB_EN_MASK, AD4130_CSB_EN_MASK);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, AD4130_REG_ADC_CONTROL,
				 AD4130_MCLK_SEL_MASK,
				 FIELD_PREP(AD4130_MCLK_SEL_MASK,
					    st->mclk_sel));
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, AD4130_REG_IO_CONTROL,
				 AD4130_INT_PIN_SEL_MASK,
				 FIELD_PREP(AD4130_INT_PIN_SEL_MASK,
					    st->int_pin_sel));
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, AD4130_REG_MISC,
				 AD4130_STBY_OUT_EN_MASK,
				 st->standby_out_en ? AD4130_STBY_OUT_EN_MASK
						    : 0);
	if (ret)
		return ret;

	/* First channel starts out as enabled, disable it. */
	ret = ad4130_set_channel_enable(st, 0, false);
	if (ret)
		return ret;

	return 0;
}

static int ad4130_soft_reset(struct ad4130_state *st)
{
	unsigned int val;
	int ret;

	ret = spi_write(st->spi, st->reset_buf, sizeof(st->reset_buf));
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(st->regmap, AD4130_REG_STATUS, val,
				       !(val & AD4130_STATUS_POR_FLAG_MSK),
				       AD4130_SOFT_RESET_SLEEP,
				       AD4130_SOFT_RESET_TIMEOUT);
	if (ret) {
		dev_err(&st->spi->dev, "Soft reset failed\n");
		return ret;
	}

	return 0;
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

	memset(st->reset_buf, 0xff, AD4130_RESET_BUF_SIZE);
	st->chip_info = info;
	st->spi = spi;

	indio_dev->name = st->chip_info->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad4130_info;

	st->regmap = devm_regmap_init(&spi->dev, NULL, st,
				      &ad4130_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	ret = ad4130_soft_reset(st);
	if (ret)
		return ret;

	st->mclk = devm_clk_get(&spi->dev, "mclk");
	if (IS_ERR(st->mclk))
		return PTR_ERR(st->mclk);

	ret = ad4310_parse_fw(st);
	if (ret)
		return ret;

	ret = ad4130_setup(st);
	if (ret)
		return ret;

	if (st->num_gpios) {
		st->gc.owner = THIS_MODULE;
		st->gc.label = st->chip_info->name;
		st->gc.base = -1;
		st->gc.ngpio = AD4130_MAX_GPIOS;
		st->gc.parent = &st->spi->dev;
		st->gc.can_sleep = true;
		st->gc.get_direction = ad4130_gpio_get_direction;
		st->gc.direction_input = ad4130_gpio_direction_input;
		st->gc.direction_output = ad4130_gpio_direction_output;
		st->gc.get = ad4130_gpio_get;
		st->gc.get_multiple = ad4130_gpio_get_multiple;
		st->gc.set = ad4130_gpio_set;
		st->gc.set_multiple = ad4130_gpio_set_multiple;

		ret = devm_gpiochip_add_data(&spi->dev, &st->gc, st);
		if (ret)
			return ret;
	}

	irq_set_status_flags(spi->irq, IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
					ad4130_irq_handler, IRQF_ONESHOT,
					indio_dev->name, indio_dev);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "Failed to request irq\n");

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct ad4130_chip_info ad4130_chip_info_tbl[] = {
	[ID_AD4130_8_24_WLCSP] = {
		.name = AD4130_8_NAME,
		.resolution = 24,
		.has_int_pin = true,
	},
	[ID_AD4130_8_24_LFCSP] = {
		.name = AD4130_8_NAME,
		.resolution = 24,
		.has_int_pin = false,
	},
	[ID_AD4130_8_16_WLCSP] = {
		.name = AD4130_8_NAME,
		.resolution = 16,
		.has_int_pin = true,
	},
	[ID_AD4130_8_16_LFCSP] = {
		.name = AD4130_8_NAME,
		.resolution = 16,
		.has_int_pin = false,
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
