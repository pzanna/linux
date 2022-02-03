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
#define AD4130_MODE_MASK		GENMASK(5, 2)

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
#define AD4130_SETUP_MASK		GENMASK(22, 20)
#define AD4130_AINP_MASK		GENMASK(17, 13)
#define AD4130_AINM_MASK		GENMASK(12, 8)
#define AD4130_IOUT1_MASK		GENMASK(7, 4)
#define AD4130_IOUT2_MASK		GENMASK(3, 0)

#define AD4130_REG_CONFIG_X(x)		(0x19 + (x))
#define AD4130_IOUT1_VAL_MASK		GENMASK(15, 13)
#define AD4130_IOUT2_VAL_MASK		GENMASK(12, 10)
#define AD4130_IOUT_OFF			0x0
#define AD4130_IOUT_10000NA		0x1
#define AD4130_IOUT_20000NA		0x2
#define AD4130_IOUT_50000NA		0x3
#define AD4130_IOUT_100000NA		0x4
#define AD4130_IOUT_150000NA		0x5
#define AD4130_IOUT_200000NA		0x6
#define AD4130_IOUT_100NA		0x7
#define AD4130_BURNOUT_MASK		GENMASK(9, 8)
#define AD4130_BURNOUT_OFF		0x0
#define AD4130_BURNOUT_500NA		0x1
#define AD4130_BURNOUT_2000NA		0x2
#define AD4130_BURNOUT_4000NA		0x3
#define AD4130_REF_BUFP			BIT(7)
#define AD4130_REF_BUFM			BIT(6)
#define AD4130_REF_SEL			GENMASK(5, 4)

#define AD4130_MAX_GPIOS		4
#define AD4130_MAX_SETUPS		8
#define AD4130_MAX_CHANNELS		16
#define AD4130_MAX_ANALOG_PINS		16
#define AD4130_MAX_DIFF_INPUTS		30

#define AD4130_AIN_2_P1			0x2
#define AD4130_AIN_3_P2			0x3
#define AD4130_AIN_4_P3			0x4
#define AD4130_AIN_5_P4			0x5

#define AD4130_RESET_CLK_COUNT		64
#define AD4130_RESET_BUF_SIZE		(AD4130_RESET_CLK_COUNT / 8)
#define AD4130_SOFT_RESET_SLEEP		2000
#define AD4130_SOFT_RESET_TIMEOUT	(AD4130_SOFT_RESET_SLEEP * 100)

static const unsigned int ad4130_reg_size[] = {
	[AD4130_REG_STATUS] = 1,
	[AD4130_REG_ADC_CONTROL] = 2,
	[AD4130_REG_IO_CONTROL] = 2,
	[AD4130_REG_ID] = 1,
	[
		AD4130_REG_CHANNEL_X(0)
		...
		AD4130_REG_CHANNEL_X(AD4130_MAX_CHANNELS)
	] = 3,
};

static const unsigned int ad4130_iout_current_na_tbl[] = {
	[AD4130_IOUT_OFF] = 0,
	[AD4130_IOUT_100NA] = 100,
	[AD4130_IOUT_10000NA] = 10000,
	[AD4130_IOUT_20000NA] = 20000,
	[AD4130_IOUT_50000NA] = 50000,
	[AD4130_IOUT_100000NA] = 100000,
	[AD4130_IOUT_150000NA] = 150000,
	[AD4130_IOUT_200000NA] = 200000,
};

static const unsigned int ad4130_burnout_current_na_tbl[] = {
	[AD4130_BURNOUT_OFF] = 0,
	[AD4130_BURNOUT_500NA] = 500,
	[AD4130_BURNOUT_2000NA] = 2000,
	[AD4130_BURNOUT_4000NA] = 4000,
};

enum ad4130_id {
	ID_AD4130_8_24_WLCSP,
	ID_AD4130_8_24_LFCSP,
	ID_AD4130_8_16_WLCSP,
	ID_AD4130_8_16_LFCSP,
};

enum ad4130_mode {
	AD4130_MODE_CONTINUOUS = 0b0000,
	AD4130_MODE_SINGLE = 0b0001,
	AD4130_MODE_IDLE = 0b0100,
};

enum ad4130_pin_function {
	AD4130_PIN_FN_NONE,
	AD4130_PIN_FN_SPECIAL,
};

struct ad4130_chip_info {
	const char	*name;
	u8		resolution;
	bool		has_int_pin;
};

struct ad4130_chan_info {
	u32		setup;
	u32		iout0;
	u32		iout1;
};

struct ad4130_setup_info {
	unsigned int	iout0_val;
	unsigned int	iout1_val;
	unsigned int	burnout;
	bool		buffered_positive;
	bool		buffered_negative;
	u32		reference_select;
};

struct ad4130_state {
	const struct ad4130_chip_info	*chip_info;
	struct spi_device		*spi;
	struct regmap			*regmap;
	struct clk			*mclk;

	/*
	 * Synchronize access to members of driver state, and ensure atomicity
	 * of consecutive regmap operations.
	 */
	struct mutex			lock;
	struct completion		completion;

	struct iio_chan_spec		chans[AD4130_MAX_CHANNELS];
	struct ad4130_chan_info		chans_info[AD4130_MAX_CHANNELS];
	struct ad4130_setup_info	setups_info[AD4130_MAX_SETUPS];
	enum ad4130_pin_function	pins_fn[AD4130_MAX_ANALOG_PINS];
	struct gpio_chip		gc;
	unsigned int			gpio_offsets[AD4130_MAX_GPIOS];
	unsigned int			num_gpios;

	u32			int_pin_sel;
	u32			mclk_sel;

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
		.endianness = IIO_BE,
	},
};

static int ad4130_get_reg_size(struct ad4130_state *st, unsigned int reg,
			       unsigned int *size)
{
	if (reg >= ARRAY_SIZE(ad4130_reg_size))
		return -EINVAL;

	if (reg == AD4130_REG_DATA) {
		*size = st->chip_info->resolution / 8;
		return 0;
	}

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
	return GPIO_LINE_DIRECTION_OUT;
}

static void ad4130_gpio_set(struct gpio_chip *gc, unsigned int offset,
			    int value)
{
	struct ad4130_state *st = gpiochip_get_data(gc);
	unsigned int real_offset = st->gpio_offsets[offset];
	unsigned int mask = FIELD_PREP(AD4130_GPIO_DATA_MASK, BIT(real_offset));

	regmap_update_bits(st->regmap, AD4130_REG_IO_CONTROL, mask, mask);
}

static int ad4130_set_mode(struct ad4130_state *st, enum ad4130_mode mode)
{
	return regmap_update_bits(st->regmap, AD4130_REG_ADC_CONTROL,
				  AD4130_MODE_MASK,
				  FIELD_PREP(AD4130_MODE_MASK, mode));
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
	struct iio_dev *indio_dev = private;
	struct ad4130_state *st = iio_priv(indio_dev);

	complete(&st->completion);

	return IRQ_HANDLED;
}

static int _ad4130_read_sample(struct ad4130_state *st,
			       struct iio_chan_spec const *chan,
			       int *val)
{
	int ret;

	reinit_completion(&st->completion);

	ret = ad4130_set_channel_enable(st, chan->scan_index, true);
	if (ret)
		return ret;

	ret = ad4130_set_mode(st, AD4130_MODE_SINGLE);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&st->completion,
					  msecs_to_jiffies(1000));
	if (!ret)
		return -ETIMEDOUT;

	ret = regmap_read(st->regmap, AD4130_REG_DATA, val);
	if (ret)
		return ret;

	ret = ad4130_set_mode(st, AD4130_MODE_IDLE);
	if (ret)
		return ret;

	ret = ad4130_set_channel_enable(st, chan->scan_index, false);
	if (ret)
		return ret;

	return IIO_VAL_INT;
}

static int ad4130_read_sample(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val)
{
	struct ad4130_state *st = iio_priv(indio_dev);
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	ret = _ad4130_read_sample(st, chan, val);
	mutex_unlock(&st->lock);

	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int ad4130_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_RAW:
		return ad4130_read_sample(indio_dev, chan, val);
	default:
		return -EINVAL;
	}
}

static int ad4130_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad4130_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static int ad4130_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *scan_mask)
{
	struct ad4130_state *st = iio_priv(indio_dev);
	int ret;
	int i;

	for (i = 0; i < indio_dev->num_channels; i++) {
		ret = ad4130_set_channel_enable(st, i, test_bit(i, scan_mask));
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

static int ad4130_validate_diff_channel(struct ad4130_state *st, u32 pin)
{
	struct device *dev = &st->spi->dev;
	enum ad4130_pin_function pin_fn;

	if (pin >= AD4130_MAX_DIFF_INPUTS) {
		dev_err(dev, "Invalid diffreential channel %u\n", pin);
		return -EINVAL;
	}

	if (pin >= AD4130_MAX_ANALOG_PINS)
		return 0;

	pin_fn = st->pins_fn[pin];

	if (pin_fn == AD4130_PIN_FN_SPECIAL) {
		dev_err(dev, "Pin %u already used with fn %u\n",
			pin, pin_fn);
		return -EINVAL;
	}

	return 0;
}

static int ad4130_validate_diff_channels(struct ad4130_state *st,
					 u32 *pins, unsigned int len)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < len; i++) {
		ret = ad4130_validate_diff_channel(st, pins[i]);
		if (ret)
			break;
	}

	return ret;
}

static int ad4130_validate_excitation_pin(struct ad4130_state *st, u32 pin)
{
	struct device *dev = &st->spi->dev;
	enum ad4130_pin_function pin_fn;

	if (pin >= AD4130_MAX_ANALOG_PINS) {
		dev_err(dev, "Invalid excitation pin %u\n", pin);
		return -EINVAL;
	}

	pin_fn = st->pins_fn[pin];

	if (pin_fn == AD4130_PIN_FN_SPECIAL) {
		dev_err(dev, "Pin %u already used with fn %u\n",
			pin, pin_fn);
		return -EINVAL;
	}

	return 0;
}

static int ad4130_parse_fw_channel(struct iio_dev *indio_dev,
				   struct fwnode_handle *child)
{
	struct ad4130_state *st = iio_priv(indio_dev);
	unsigned int index = indio_dev->num_channels++;
	struct iio_chan_spec *chan = &st->chans[index];
	struct device *dev = &st->spi->dev;
	struct ad4130_chan_info *chan_info;
	u32 pins[2], reg;
	int ret;

	*chan = ad4130_channel_template;
	chan->scan_type.realbits = st->chip_info->resolution;
	chan->scan_type.storagebits = st->chip_info->resolution;

	ret = fwnode_property_read_u32(child, "reg", &reg);
	if (ret) {
		dev_err(dev, "Missing channel index\n");
		return ret;
	}

	if (reg >= AD4130_MAX_CHANNELS) {
		dev_err(dev, "Channel index %u invalid\n", reg);
		return -EINVAL;
	}

	chan_info = &st->chans_info[reg];
	chan->scan_index = reg;

	ret = fwnode_property_read_u32_array(child, "adi,diff-channels", pins,
					     ARRAY_SIZE(pins));
	if (ret)
		return ret;

	ret = ad4130_validate_diff_channels(st, pins, ARRAY_SIZE(pins));
	if (ret)
		return ret;

	chan->channel = pins[0];
	chan->channel2 = pins[1];

	fwnode_property_read_u32(child, "adi,setup", &chan_info->setup);
	if (chan_info->setup >= AD4130_MAX_SETUPS) {
		dev_err(dev, "Invalid config setup %u\n", chan_info->setup);
		return -EINVAL;
	}

	fwnode_property_read_u32(child, "adi,excitation-pin-0",
				 &chan_info->iout0);
	ret = ad4130_validate_excitation_pin(st, chan_info->iout0);
	if (ret)
		return ret;

	fwnode_property_read_u32(child, "adi,excitation-pin-1",
				 &chan_info->iout1);
	ret = ad4130_validate_excitation_pin(st, chan_info->iout1);
	if (ret)
		return ret;

	return 0;
}

static int ad4130_validate_excitation_current(struct ad4130_state *st,
					      unsigned int *iout_val,
					      u32 current_na)
{
	struct device *dev = &st->spi->dev;
	int i;

	for (i = 0; i < ARRAY_SIZE(ad4130_iout_current_na_tbl); i++)
		if (ad4130_iout_current_na_tbl[i] == current_na) {
			*iout_val = i;
			return 0;
		}

	dev_err(dev, "Invalid excitation current %unA\n", current_na);

	return -EINVAL;
}

static int ad4130_validate_burnout_current(struct ad4130_state *st,
					   unsigned int *burnout,
					   u32 current_na)
{
	struct device *dev = &st->spi->dev;
	int i;

	for (i = 0; i < ARRAY_SIZE(ad4130_burnout_current_na_tbl); i++)
		if (ad4130_burnout_current_na_tbl[i] == current_na) {
			*burnout = i;
			return 0;
		}

	dev_err(dev, "Invalid excitation current %unA\n", current_na);

	return -EINVAL;
}

static int ad4130_parse_fw_setup(struct iio_dev *indio_dev,
				 struct fwnode_handle *child)
{
	struct ad4130_state *st = iio_priv(indio_dev);
	struct ad4130_setup_info *setup_info;
	struct device *dev = &st->spi->dev;
	u32 reg, current_na;
	int ret;

	ret = fwnode_property_read_u32(child, "reg", &reg);
	if (ret) {
		dev_err(dev, "Missing setup index\n");
		return ret;
	}

	setup_info = &st->setups_info[reg];

	fwnode_property_read_u32(child, "adi,excitation-current-0-nanoamps",
				 &current_na);
	ret = ad4130_validate_excitation_current(st, &setup_info->iout0_val,
						 current_na);
	if (ret)
		return ret;

	fwnode_property_read_u32(child, "adi,excitation-current-1-nanoamps",
				 &current_na);
	ret = ad4130_validate_excitation_current(st, &setup_info->iout1_val,
						 current_na);
	if (ret)
		return ret;

	fwnode_property_read_u32(child, "adi,burnout-current-nanoamps",
				 &current_na);
	ret = ad4130_validate_burnout_current(st, &setup_info->burnout,
					      current_na);
	if (ret)
		return ret;

	setup_info->buffered_positive =
		fwnode_property_read_bool(child, "adi,buffered-positive");

	setup_info->buffered_negative =
		fwnode_property_read_bool(child, "adi,buffered-negative");

	return 0;
}

static int ad4130_parse_fw_children(struct iio_dev *indio_dev)
{
	struct ad4130_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct fwnode_handle *child;
	int ret;

	indio_dev->channels = st->chans;

	fwnode_for_each_available_child_node(fwnode, child) {
		const char *name = fwnode_get_name(child);

		if (!strcmp(name, "channel")) {
			ret = ad4130_parse_fw_channel(indio_dev, child);
		} else if (strcmp(name, "setup")) {
			ret = ad4130_parse_fw_setup(indio_dev, child);
		} else {
			dev_err(dev, "Invalid child name %s\n", name);
			ret = -EINVAL;
		}

		if (ret)
			break;
	}

	fwnode_handle_put(child);

	return ret;
}

static int ad4310_parse_fw(struct iio_dev *indio_dev)
{
	struct ad4130_state *st = iio_priv(indio_dev);
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
		dev_err(dev, "Cannot use DOUT as interrupt pin\n");
		return -EINVAL;
	}

	if (st->int_pin_sel == AD4130_INT_PIN_P1)
		st->pins_fn[AD4130_AIN_2_P1] = AD4130_PIN_FN_SPECIAL;

	st->mclk_sel = AD4130_MCLK_76_8KHZ;
	device_property_read_u32(dev, "adi,mclk-sel", &st->mclk_sel);

	if (st->mclk_sel < AD4130_MCLK_76_8KHZ ||
	    st->mclk_sel > AD4130_MCLK_153_6KHZ_EXT) {
		dev_err(dev, "Invalid clock %u\n", st->mclk_sel);
		return -EINVAL;
	}

	if (st->int_pin_sel == AD4130_INT_PIN_CLK
	    && st->mclk_sel != AD4130_MCLK_76_8KHZ) {
		dev_err(dev, "Invalid clock %u for interrupt pin %u\n",
			st->mclk_sel, st->int_pin_sel);
		return -EINVAL;
	}

	ret = ad4130_parse_fw_children(indio_dev);
	if (ret)
		return ret;

	for (i = 0; i < AD4130_MAX_GPIOS; i++) {
		if (st->pins_fn[i + AD4130_AIN_2_P1] == AD4130_PIN_FN_NONE)
			st->gpio_offsets[st->num_gpios++] = i;
	}

	return 0;
}

static int ad4130_setup(struct iio_dev *indio_dev)
{
	struct ad4130_state *st = iio_priv(indio_dev);
	unsigned int i;
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

	/* ADC starts out in single conversion mode, switch to idle. */
	ret = ad4130_set_mode(st, AD4130_MODE_IDLE);
	if (ret)
		return ret;

	/* Switch unused GPIOs to output mode. */
	for (i = 0; i < st->num_gpios; i++) {
		unsigned int real_offset = st->gpio_offsets[i];
		return regmap_update_bits(st->regmap, AD4130_REG_IO_CONTROL,
					  BIT(real_offset), BIT(real_offset));
	}

	/* Setup channels. */
	for (i = 0; i < indio_dev->num_channels; i++) {
		struct iio_chan_spec *chan = &st->chans[i];
		unsigned int channel = chan->scan_index;
		struct ad4130_chan_info *chan_info = &st->chans_info[channel];
		unsigned int val;

		val = FIELD_PREP(AD4130_SETUP_MASK, chan_info->setup) |
		      FIELD_PREP(AD4130_AINP_MASK, chan->channel) |
		      FIELD_PREP(AD4130_AINM_MASK, chan->channel2) |
		      FIELD_PREP(AD4130_IOUT1_MASK, chan_info->iout0) |
		      FIELD_PREP(AD4130_IOUT2_MASK, chan_info->iout1);

		ret = regmap_write(st->regmap, AD4130_REG_CHANNEL_X(channel),
				   val);
		if (ret)
			return ret;
	}

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
	init_completion(&st->completion);
	mutex_init(&st->lock);
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

	ret = ad4310_parse_fw(indio_dev);
	if (ret)
		return ret;

	ret = ad4130_setup(indio_dev);
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
		st->gc.set = ad4130_gpio_set;

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
