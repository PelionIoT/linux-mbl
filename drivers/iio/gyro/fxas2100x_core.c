/*
 * Driver for NXP FXAS2100x Gyroscope - Core
 *
 * Copyright (C) 2017 Linaro Ltd.
 *
 */

#include <linux/module.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/regmap.h>

#include "fxas2100x.h"

#define FXAS21002_CHIP_ID_1	0xD6
#define FXAS21002_CHIP_ID_2	0xD7

#define FXAS2100X_MODE_STANDBY	0x00
#define FXAS2100X_MODE_READY	0x01
#define FXAS2100X_MODE_ACTIVE	0x02

#define FXAS2100X_STANDBY_ACTIVE_TIME_MS	62
#define FXAS2100X_READY_ACTIVE_TIME_MS		7

#define FXAS2100X_ODR_LIST_MAX		10

#define FXAS2100X_SCALE_FRACTIONAL	32
#define FXAS2100X_RANGE_LIMIT_DOUBLE	2000

#define FXAS2100X_AXIS_TO_REG(axis)	(FXAS2100X_REG_OUT_X_MSB + (axis * 2))

static const int fxas2100x_odr_values[] = {
	800, 400, 200, 100, 50, 25, 12, 12
};

/*
 * This values are taken from the low-pass filter cuttoff frequency calculated
 * ODR * 0.lpf_values. So, for ODR = 800Hz with a lpf_value = 0.32
 * => LPF cuttof frequency = 800 * 0.32 = 256 Hz
 */
static const int fxas2100x_lpf_values[] = {
	32, 16, 8
};

/*
 * This values are taken from the high-pass filter cuttoff frequency calculated
 * ODR * 0.0hpf_values. So, for ODR = 800Hz with a hpf_value = 0.018750
 * => HPF cuttof frequency = 800 * 0.018750 = 15 Hz
 */
static const int fxas2100x_hpf_values[] = {
	18750, 9625, 4875, 2475
};

static const int fxas2100x_range_values[] = {
	4000, 2000, 1000, 500, 250
};

struct fxas2100x_data {
	u8 chip_id;
	u8 mode;
	u8 prev_mode;
	struct regmap *regmap;
	struct regmap_field *regmap_fields[F_MAX_FIELDS];
	struct mutex mutex;		/* protect access */
	int irq;
};

enum fxas2100x_channel_index {
	CHANNEL_SCAN_INDEX_X,
	CHANNEL_SCAN_INDEX_Y,
	CHANNEL_SCAN_INDEX_Z,
	CHANNEL_SCAN_MAX,
};

static int fxas2100x_odr_hz_from_value(struct fxas2100x_data *data, u8 value)
{
	int odr_value_max = ARRAY_SIZE(fxas2100x_odr_values) - 1;

	value = min_t(u8, value, odr_value_max);

	return fxas2100x_odr_values[value];
}

static int fxas2100x_odr_value_from_hz(struct fxas2100x_data *data,
				       unsigned int hz)
{
	int odr_table_size = ARRAY_SIZE(fxas2100x_odr_values);
	int i;

	for (i = 0; i < odr_table_size; i++)
		if (fxas2100x_odr_values[i] == hz)
			return i;

	return -EINVAL;
}

static int fxas2100x_lpf_bw_from_value(struct fxas2100x_data *data, u8 value)
{
	int lpf_value_max = ARRAY_SIZE(fxas2100x_lpf_values) - 1;

	value = min_t(u8, value, lpf_value_max);

	return fxas2100x_lpf_values[value];
}

static int fxas2100x_lpf_value_from_bw(struct fxas2100x_data *data,
				       unsigned int hz)
{
	int lpf_table_size = ARRAY_SIZE(fxas2100x_lpf_values);
	int i;

	for (i = 0; i < lpf_table_size; i++)
		if (fxas2100x_lpf_values[i] == hz)
			return i;

	return -EINVAL;
}

static int fxas2100x_hpf_sel_from_value(struct fxas2100x_data *data, u8 value)
{
	int hpf_value_max = ARRAY_SIZE(fxas2100x_hpf_values) - 1;

	value = min_t(u8, value, hpf_value_max);

	return fxas2100x_hpf_values[value];
}

static int fxas2100x_hpf_value_from_sel(struct fxas2100x_data *data,
					unsigned int hz)
{
	int hpf_table_size = ARRAY_SIZE(fxas2100x_hpf_values);
	int i;

	for (i = 0; i < hpf_table_size; i++)
		if (fxas2100x_hpf_values[i] == hz)
			return i;

	return -EINVAL;
}

static int fxas2100x_range_fs_from_value(struct fxas2100x_data *data,
					 u8 value)
{
	int range_value_max = ARRAY_SIZE(fxas2100x_range_values) - 1;
	unsigned int fs_double;
	int ret;

	/* We need to check if FS_DOUBLE is enabled to offset the value */
        ret = regmap_field_read(data->regmap_fields[F_FS_DOUBLE], &fs_double);
	if (ret < 0)
		return ret;

	value = min_t(u8, value, range_value_max);

	if (!fs_double)
		value += 1;

	return fxas2100x_range_values[value];
}

static int fxas2100x_range_value_from_fs(struct fxas2100x_data *data,
					 unsigned int range)
{
	int range_table_size = ARRAY_SIZE(fxas2100x_range_values);
	bool found = false;
	int ret;
	int i;

	for (i = 0; i < range_table_size; i++)
		if (fxas2100x_range_values[i] == range)
			found = true;
	if (!found)
		return -EINVAL;

	if (range > FXAS2100X_RANGE_LIMIT_DOUBLE)
		ret = regmap_field_write(data->regmap_fields[F_FS_DOUBLE], 1);
	else
		ret = regmap_field_write(data->regmap_fields[F_FS_DOUBLE], 0);

	return i;
}

static int fxas2100x_mode_get(struct fxas2100x_data *data)
{
	unsigned int active;
	unsigned int ready;
	int ret;

	ret = regmap_field_read(data->regmap_fields[F_ACTIVE], &active);
	if (ret < 0)
		return ret;
	if (active)
		return FXAS2100X_MODE_ACTIVE;

	ret = regmap_field_read(data->regmap_fields[F_READY], &ready);
	if (ret < 0)
		return ret;
	if (ready)
		return FXAS2100X_MODE_READY;

	return FXAS2100X_MODE_STANDBY;
}

static int fxas2100x_mode_set(struct fxas2100x_data *data, u8 mode)
{
	int ret;

	if (mode > FXAS2100X_MODE_ACTIVE)
		return -EINVAL;

	if (mode == data->mode)
		return 0;

	if (mode == FXAS2100X_MODE_READY)
		ret = regmap_field_write(data->regmap_fields[F_READY], 1);
	else
		ret = regmap_field_write(data->regmap_fields[F_READY], 0);
	if (ret < 0)
		return ret;

	if (mode == FXAS2100X_MODE_ACTIVE)
		ret = regmap_field_write(data->regmap_fields[F_ACTIVE], 1);
	else
		ret = regmap_field_write(data->regmap_fields[F_ACTIVE], 0);
	if (ret < 0)
		return ret;

	/* if going to active wait the setup times */
	if (mode == FXAS2100X_MODE_ACTIVE)
		if (data->mode == FXAS2100X_MODE_STANDBY)
			msleep_interruptible(FXAS2100X_STANDBY_ACTIVE_TIME_MS);
	if (data->mode == FXAS2100X_MODE_READY)
		msleep_interruptible(FXAS2100X_READY_ACTIVE_TIME_MS);

	data->prev_mode = data->mode;
	data->mode = mode;

	return ret;
}

static int fxas2100x_pre_write(struct fxas2100x_data *data)
{
	int actual_mode;

	actual_mode = fxas2100x_mode_get(data);

	if (actual_mode < 0)
		return actual_mode;

	return fxas2100x_mode_set(data, FXAS2100X_MODE_READY);
}

static int fxas2100x_post_write(struct fxas2100x_data *data)
{
	return fxas2100x_mode_set(data, data->prev_mode);
}

static int  fxas2100x_pm_get(struct fxas2100x_data *data)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret;

	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		pm_runtime_put_noidle(dev);

	return ret;
}

static int  fxas2100x_pm_put(struct fxas2100x_data *data)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret;

	pm_runtime_mark_last_busy(dev);
	ret = pm_runtime_put_autosuspend(dev);

	return ret;
}

static int fxas2100x_temp_get(struct fxas2100x_data *data, int *val)
{
	struct device *dev = regmap_get_device(data->regmap);
	unsigned int temp;
	int ret;

	mutex_lock(&data->mutex);
	ret = fxas2100x_pm_get(data);
	if (ret < 0)
		goto data_unlock;

	ret = regmap_field_read(data->regmap_fields[F_TEMP], &temp);
	if (ret < 0) {
		dev_err(dev, "failed to read temp: %d\n", ret);
		goto data_unlock;
	}

	*val = sign_extend32(temp, 7);

	ret = fxas2100x_pm_put(data);
	if (ret < 0)
		goto data_unlock;

	ret = IIO_VAL_INT;

data_unlock:
	mutex_unlock(&data->mutex);

	return ret;
}

static int fxas2100x_axis_get(struct fxas2100x_data *data, int index, int *val)
{
	struct device *dev = regmap_get_device(data->regmap);
	__be16 axis_be;
	int ret;

	mutex_lock(&data->mutex);
	ret = fxas2100x_pm_get(data);
	if (ret < 0)
		goto data_unlock;

	ret = regmap_bulk_read(data->regmap, FXAS2100X_AXIS_TO_REG(index),
			       &axis_be, sizeof(axis_be));
	if (ret < 0) {
		dev_err(dev, "failed to read axis: %d: %d\n", index, ret);
		goto data_unlock;
	}

	*val = sign_extend32(be16_to_cpu(axis_be), 15);

	ret = fxas2100x_pm_put(data);
	if (ret < 0)
		goto data_unlock;

	ret = IIO_VAL_INT;

data_unlock:
	mutex_unlock(&data->mutex);

	return ret;
}

static int fxas2100x_odr_get(struct fxas2100x_data *data, int *odr)
{
	unsigned int odr_bits;
	int ret;

	mutex_lock(&data->mutex);
	ret = regmap_field_read(data->regmap_fields[F_DR], &odr_bits);
	if (ret < 0)
		goto data_unlock;

	*odr = fxas2100x_odr_hz_from_value(data, odr_bits);

	ret = IIO_VAL_INT;
data_unlock:
	mutex_unlock(&data->mutex);

	return ret;
}

static int fxas2100x_odr_set(struct fxas2100x_data *data, int odr)
{
	int odr_bits;
	int ret;

	odr_bits = fxas2100x_odr_value_from_hz(data, odr);

	if (odr_bits < 0)
		return odr_bits;

	mutex_lock(&data->mutex);
	ret = fxas2100x_pre_write(data);
	if (ret < 0)
		goto post_write;

	ret = regmap_field_write(data->regmap_fields[F_DR], odr_bits);
	if (ret < 0)
		goto post_write;

post_write:
	ret = fxas2100x_post_write(data);

	mutex_unlock(&data->mutex);

	return ret;
}

static int fxas2100x_lpf_get(struct fxas2100x_data *data, int *val2)
{
	unsigned int bw_bits;
	int ret;

	mutex_lock(&data->mutex);
	ret = regmap_field_read(data->regmap_fields[F_BW], &bw_bits);
	if (ret < 0)
		goto data_unlock;

	*val2 = fxas2100x_lpf_bw_from_value(data, bw_bits) * 10000;

	ret = IIO_VAL_INT_PLUS_MICRO;

data_unlock:
	mutex_unlock(&data->mutex);

	return ret;
}

static int fxas2100x_lpf_set(struct fxas2100x_data *data, int bw)
{
	int bw_bits;
	int odr;
	int ret;

	bw_bits = fxas2100x_lpf_value_from_bw(data, bw);

	if (bw_bits < 0)
		return bw_bits;

	/*
	 * From table 33 of the device spec, for ODR = 25Hz and 12.5 value 0.08
	 * is not allowed and for ODR = 12.5 value 0.16 is also not allowed
	 */
	ret = fxas2100x_odr_get(data, &odr);
	if (ret < 0)
		return -EINVAL;

	if ((odr == 25 && bw_bits > 0x01) || (odr == 12 && bw_bits > 0))
		return -EINVAL;

	mutex_lock(&data->mutex);
	ret = fxas2100x_pre_write(data);
	if (ret < 0)
		goto post_write;

	ret = regmap_field_write(data->regmap_fields[F_BW], bw_bits);
	if (ret < 0)
		goto post_write;

post_write:
	ret = fxas2100x_post_write(data);

	mutex_unlock(&data->mutex);

	return ret;
}

static int fxas2100x_hpf_get(struct fxas2100x_data *data, int *val2)
{
	unsigned int sel_bits;
	int ret;

	mutex_lock(&data->mutex);
	ret = regmap_field_read(data->regmap_fields[F_SEL], &sel_bits);
	if (ret < 0)
		goto data_unlock;

	*val2 = fxas2100x_hpf_sel_from_value(data, sel_bits);

	ret = IIO_VAL_INT_PLUS_MICRO;

data_unlock:
	mutex_unlock(&data->mutex);

	return ret;
}

static int fxas2100x_hpf_set(struct fxas2100x_data *data, int sel)
{
	int sel_bits;
	int ret;

	sel_bits = fxas2100x_hpf_value_from_sel(data, sel);

	if (sel_bits < 0)
		return sel_bits;

	mutex_lock(&data->mutex);
	ret = fxas2100x_pre_write(data);
	if (ret < 0)
		goto post_write;

	ret = regmap_field_write(data->regmap_fields[F_SEL], sel_bits);
	if (ret < 0)
		goto post_write;

post_write:
	ret = fxas2100x_post_write(data);

	mutex_unlock(&data->mutex);

	return ret;
}

static int fxas2100x_scale_get(struct fxas2100x_data *data, int *val)
{
	int fs_bits;
	int scale;
	int ret;

	mutex_lock(&data->mutex);
	ret = regmap_field_read(data->regmap_fields[F_FS], &fs_bits);
	if (ret < 0)
		goto data_unlock;

	scale = fxas2100x_range_fs_from_value(data, fs_bits);

	*val = scale;

	ret = IIO_VAL_FRACTIONAL;

data_unlock:
	mutex_unlock(&data->mutex);

	return ret;
}

static int fxas2100x_scale_set(struct fxas2100x_data *data, int range)
{
	int fs_bits;
	int ret;

	fs_bits = fxas2100x_range_value_from_fs(data, range);

	if (fs_bits < 0)
		return fs_bits;

	mutex_lock(&data->mutex);
	ret = fxas2100x_pre_write(data);
	if (ret < 0)
		goto post_write;

	ret = regmap_field_write(data->regmap_fields[F_FS], fs_bits);
	if (ret < 0)
		goto post_write;

post_write:
	ret = fxas2100x_post_write(data);

	mutex_unlock(&data->mutex);

	return ret;
}

static int fxas2100x_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan, int *val,
			      int *val2, long mask)
{
	struct fxas2100x_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_TEMP:
			return fxas2100x_temp_get(data, val);
		case IIO_ANGL_VEL:
			return fxas2100x_axis_get(data, chan->scan_index, val);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			*val2 = FXAS2100X_SCALE_FRACTIONAL;
			return fxas2100x_scale_get(data, val);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		*val = 0;
		return fxas2100x_lpf_get(data, val2);
	case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		*val = 0;
		return fxas2100x_hpf_get(data, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val2 = 0;
		return fxas2100x_odr_get(data, val);
	default:
		return -EINVAL;
	}
}

static int fxas2100x_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan, int val,
			       int val2, long mask)
{
	struct fxas2100x_data *data = iio_priv(indio_dev);
	int range;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return fxas2100x_odr_set(data, val);
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		val2 = val2 / 10000;
		return fxas2100x_lpf_set(data, val2);
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			range = (((val * 1000 + val2 / 1000) *
				  FXAS2100X_SCALE_FRACTIONAL) / 1000);
			return fxas2100x_scale_set(data, range);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		return fxas2100x_hpf_set(data, val2);
	default:
		return -EINVAL;
	}
}

static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("12.5 25 50 100 200 400 800");

static IIO_CONST_ATTR(in_anglvel_filter_low_pass_3db_frequency_available,
		      "0.32 0.16 0.08");

static IIO_CONST_ATTR(in_anglvel_filter_high_pass_3db_frequency_available,
		      "0.018750 0.009625 0.004875 0.002475");

static IIO_CONST_ATTR(in_anglvel_scale_available,
		      "125.0 62.5 31.25 15.625 7.8130");

static struct attribute *fxas2100x_attributes[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_anglvel_filter_low_pass_3db_frequency_available.dev_attr.attr,
	&iio_const_attr_in_anglvel_filter_high_pass_3db_frequency_available.dev_attr.attr,
	&iio_const_attr_in_anglvel_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group fxas2100x_attrs_group = {
	.attrs = fxas2100x_attributes,
};

#define FXAS2100X_CHANNEL(_axis) {					\
	.type = IIO_ANGL_VEL,						\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |		\
		BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY) |	\
		BIT(IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY) |	\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),				\
	.scan_index = CHANNEL_SCAN_INDEX_##_axis,			\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,						\
		.storagebits = 16,					\
		.endianness = IIO_BE,					\
	},								\
}

static const struct iio_chan_spec fxas2100x_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = -1,
	},
	FXAS2100X_CHANNEL(X),
	FXAS2100X_CHANNEL(Y),
	FXAS2100X_CHANNEL(Z),
};

static const struct iio_info fxas2100x_info = {
	.driver_module		= THIS_MODULE,
	.attrs			= &fxas2100x_attrs_group,
	.read_raw		= &fxas2100x_read_raw,
	.write_raw		= &fxas2100x_write_raw,
};

static int fxas2100x_chip_init(struct fxas2100x_data *data)
{
	struct device *dev = regmap_get_device(data->regmap);
	unsigned int chip_id;
	int ret;

	ret = regmap_field_read(data->regmap_fields[F_WHO_AM_I], &chip_id);
	if (ret < 0)
		return ret;

	if (chip_id != FXAS21002_CHIP_ID_1 && chip_id != FXAS21002_CHIP_ID_2) {
		dev_err(dev, "chip id %d is not supported\n", chip_id);
		return -EINVAL;
	}

	data->chip_id = chip_id;

	ret = fxas2100x_mode_set(data, FXAS2100X_MODE_STANDBY);
	if (ret < 0)
		return ret;

	/* Set ODR to 200HZ as default */
	ret = fxas2100x_odr_set(data, 200);
	if (ret < 0)
		dev_err(dev, "failed to set ODR: %d\n", ret);

	return ret;
}

int fxas2100x_core_probe(struct device *dev, struct regmap *regmap, int irq,
			 const char *name)
{
	struct fxas2100x_data *data;
	struct iio_dev *indio_dev;
	struct regmap_field *f;
	int i;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);
	data->irq = irq;
	data->regmap = regmap;

	for (i = 0; i < F_MAX_FIELDS; i++) {
		f = devm_regmap_field_alloc(dev, data->regmap,
					    fxas2100x_reg_fields[i]);
		if (IS_ERR(f)) {
			dev_err(dev, "failed to alloc regmap field %d: %ld\n",
				i, PTR_ERR(f));
			return PTR_ERR(f);
		}
		data->regmap_fields[i] = f;
	}

	mutex_init(&data->mutex);

	ret = fxas2100x_chip_init(data);
	if (ret < 0)
		return ret;

	indio_dev->dev.parent = dev;
	indio_dev->channels = fxas2100x_channels;
	indio_dev->num_channels = ARRAY_SIZE(fxas2100x_channels);
	indio_dev->name = name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &fxas2100x_info;

	ret = pm_runtime_set_active(dev);
	if (ret)
		return ret;

	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 2000);
	pm_runtime_use_autosuspend(dev);

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(dev, "unable to register iio device: %d\n", ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(fxas2100x_core_probe);

void fxas2100x_core_remove(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fxas2100x_data *data = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_put_noidle(dev);

	fxas2100x_mode_set(data, FXAS2100X_MODE_STANDBY);
}
EXPORT_SYMBOL_GPL(fxas2100x_core_remove);

#ifdef CONFIG_PM_SLEEP
static int fxas2100x_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fxas2100x_data *data = iio_priv(indio_dev);

	fxas2100x_mode_set(data, FXAS2100X_MODE_STANDBY);

	return 0;
}

static int fxas2100x_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fxas2100x_data *data = iio_priv(indio_dev);

	fxas2100x_mode_set(data, data->prev_mode);

	return 0;
}
#endif

#ifdef CONFIG_PM
static int fxas2100x_runtime_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fxas2100x_data *data = iio_priv(indio_dev);
	int ret;

	ret = fxas2100x_mode_set(data, FXAS2100X_MODE_READY);
	if (ret < 0)
		return -EAGAIN;

	return 0;
}

static int fxas2100x_runtime_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fxas2100x_data *data = iio_priv(indio_dev);
	int ret;

	ret = fxas2100x_mode_set(data, FXAS2100X_MODE_ACTIVE);
	if (ret < 0)
		return -EAGAIN;

	return 0;
}
#endif

const struct dev_pm_ops fxas2100x_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(fxas2100x_suspend, fxas2100x_resume)
	SET_RUNTIME_PM_OPS(fxas2100x_runtime_suspend,
			   fxas2100x_runtime_resume, NULL)
};
EXPORT_SYMBOL_GPL(fxas2100x_pm_ops);

MODULE_AUTHOR("Rui Miguel Silva <rui.silva@linaro.org>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("FXAS2100X Gyro driver");
