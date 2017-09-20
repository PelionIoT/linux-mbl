/*
 * Driver for NXP FXOS8700 Accelerometer and Magnetometer - Core
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

#include "fxos8700.h"

#define FXOS8700_CHIP_ID	0xC4
#define FXOS8700_PRE_CHIP_ID	0xC7

#define FXOS8700_MODE_STANDBY	0x00
#define FXOS8700_MODE_ACTIVE	0x01

#define FXOS8700_ACCEL_ON	0x00
#define FXOS8700_MAGN_ON	0x01
#define FXOS8700_HYBRID_ON	0x03

struct fxos8700_data {
	u8 chip_id;
	u8 mode;
	u8 prev_mode;
	bool accel_enabled;
	bool magn_enabled;
	struct regmap *regmap;
	struct regmap_field *regmap_fields[F_MAX_FIELDS];
	struct mutex mutex;	/* protect data */
};

static int  fxos8700_pm_get(struct fxos8700_data *data)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret;

	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		pm_runtime_put_noidle(dev);

	return ret;
}

static int  fxos8700_pm_put(struct fxos8700_data *data)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret;

	pm_runtime_mark_last_busy(dev);
	ret = pm_runtime_put_autosuspend(dev);

	return ret;
}

static int fxos8700_mode_get(struct fxos8700_data *data)
{
	unsigned int mode;
	int ret;

	ret = regmap_field_read(data->regmap_fields[F_ACTIVE], &mode);
	if (ret < 0)
		return ret;

	return mode;
}

static int fxos8700_mode_set(struct fxos8700_data *data, u8 mode)
{
	int ret;

	if (mode > FXOS8700_MODE_ACTIVE)
		return -EINVAL;

	if (mode == FXOS8700_MODE_ACTIVE)
		ret = regmap_field_write(data->regmap_fields[F_ACTIVE], 1);
	else
		ret = regmap_field_write(data->regmap_fields[F_ACTIVE], 0);
	if (ret < 0)
		return ret;

	data->prev_mode = data->mode;
	data->mode = mode;

	return ret;
}

static int fxos8700_pre_write(struct fxos8700_data *data)
{
	int actual_mode;
	int ret;

	actual_mode = fxos8700_mode_get(data);

	if (actual_mode < 0)
		return actual_mode;

	ret = fxos8700_mode_set(data, FXOS8700_MODE_STANDBY);
	if (ret < 0)
		return ret;

	/* going from active to standby varies from 0 to 300ms */
	msleep_interruptible(300);

	return ret;
}

static int fxos8700_post_write(struct fxos8700_data *data)
{
	return fxos8700_mode_set(data, data->prev_mode);
}

static int fxos8700_axis_get(struct fxos8700_data *data, unsigned long addr,
			     int type, int *val)
{
	struct device *dev = regmap_get_device(data->regmap);
	__be16 axis_be;
	unsigned int reg;
	int bits;
	int ret_pm;
	int ret;

	mutex_lock(&data->mutex);
	ret = fxos8700_pm_get(data);
	if (ret < 0)
		goto data_unlock;

	reg = addr;

	/* For magnetometer we need to first read OUT_X_MSB to update the other
	 * axis, not needed of course if we are getting OUT_X
	 */
	if (type == IIO_MAGN && reg != FXOS8700_REG_OUT_X_MSB) {
		ret = regmap_bulk_read(data->regmap, FXOS8700_REG_OUT_X_MSB,
				       &axis_be, sizeof(axis_be));
		if (ret < 0) {
			dev_err(dev, "failed to read axis: %d\n", ret);
			goto pm_put;
		}
	}

	ret = regmap_bulk_read(data->regmap, reg, &axis_be, sizeof(axis_be));
	if (ret < 0) {
		dev_err(dev, "failed to read axis: %d\n", ret);
		goto pm_put;
	}

	/* For accelerometer only use a complement by 2' 14 bits */
	if (type == IIO_ACCEL) {
		bits = 14;
		axis_be = (axis_be >> 2);
	} else {
		bits = 16;
	}

	*val = sign_extend32(be16_to_cpu(axis_be), bits);

	ret = IIO_VAL_INT;

pm_put:
	ret_pm = fxos8700_pm_put(data);
	if (ret_pm < 0)
		ret = ret_pm;

data_unlock:
	mutex_unlock(&data->mutex);

	return ret;
}

static int fxos8700_hms_get(struct fxos8700_data *data, int type, int *val)
{
	unsigned int hms;
	int ret_pm;
	int ret;

	mutex_lock(&data->mutex);
	ret = fxos8700_pm_get(data);
	if (ret < 0)
		goto data_unlock;

	ret = regmap_field_read(data->regmap_fields[F_M_HMS], &hms);
	if (ret < 0)
		goto pm_put;

	*val = 0;

	if (type == IIO_MAGN) {
		if ((hms == FXOS8700_MAGN_ON) || (hms == FXOS8700_HYBRID_ON))
			*val = 1;
	} else {
		if ((hms == FXOS8700_ACCEL_ON) || (hms == FXOS8700_HYBRID_ON))
			*val = 1;
	}

	ret = IIO_VAL_INT;

pm_put:
	ret_pm = fxos8700_pm_put(data);
	if (ret_pm < 0)
		ret = ret_pm;

data_unlock:
	mutex_unlock(&data->mutex);

	return ret;
}

static int fxos8700_hms_set(struct fxos8700_data *data, u8 hms)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret;

	mutex_lock(&data->mutex);
	ret = fxos8700_pre_write(data);
	if (ret < 0)
		goto post_write;

	ret = regmap_field_write(data->regmap_fields[F_M_HMS], hms);
	if (ret < 0)
		goto post_write;

	if (hms == FXOS8700_HYBRID_ON) {
		data->magn_enabled = true;
		data->accel_enabled = true;
	} else if (hms == FXOS8700_ACCEL_ON) {
		data->magn_enabled = false;
		data->accel_enabled = true;
	} else if (hms == FXOS8700_MAGN_ON) {
		data->accel_enabled = false;
		data->magn_enabled = true;
	}

post_write:
	fxos8700_post_write(data);

	mutex_unlock(&data->mutex);

	dev_info(dev, "accelerometer: %s magnetometer: %s\n",
		 data->accel_enabled ? "on" : "off",
		 data->magn_enabled ? "on" : "off");

	return ret;
}

static int fxos8700_osr_set(struct fxos8700_data *data, u8 osr)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = fxos8700_pre_write(data);
	if (ret < 0)
		goto post_write;

	ret = regmap_field_write(data->regmap_fields[F_M_OS], osr);
	if (ret < 0)
		goto post_write;

post_write:
	fxos8700_post_write(data);

	mutex_unlock(&data->mutex);

	return ret;
}

static int fxos8700_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int *val,
			     int *val2, long mask)
{
	struct fxos8700_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_MAGN:
		case IIO_ACCEL:
			return fxos8700_axis_get(data, chan->address,
						 chan->type, val);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_ENABLE:
		return fxos8700_hms_get(data, chan->type, val);
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_MAGN:
			*val = 0;
			*val2 = 1;
			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_ACCEL:
			*val = 0;
			*val2 = 244; /* at +/-2g is 244mg/LSB */
			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int fxos8700_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan, int val,
			      int val2, long mask)
{
	struct fxos8700_data *data = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(data->regmap);
	u8 hms;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		if (chan->type == IIO_MAGN) {
			if (data->magn_enabled == !!val)
				return 0;

			data->magn_enabled = !!val;
		}

		if (chan->type == IIO_ACCEL) {
			if (data->accel_enabled == !!val)
				return 0;

			data->accel_enabled = !!val;
		}

		if (data->magn_enabled && data->accel_enabled) {
			hms = FXOS8700_HYBRID_ON;
		} else if (data->magn_enabled) {
			hms = FXOS8700_MAGN_ON;
		} else if (data->accel_enabled) {
			hms = FXOS8700_ACCEL_ON;
		} else {
			dev_err(dev, "both accel and  magn can not be off at the same time\n");
			return -EINVAL;
		}

		return fxos8700_hms_set(data, hms);
	default:
		return -EINVAL;
	}
}

#define FXOS8700_ACCEL_CHANNEL(_axis) {					\
	.type = IIO_ACCEL,						\
	.address = FXOS8700_REG_OUT_##_axis##_MSB,			\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |		\
				BIT(IIO_CHAN_INFO_ENABLE),		\
}

#define FXOS8700_MAGN_CHANNEL(_axis) {					\
	.type = IIO_MAGN,						\
	.address = FXOS8700_REG_M_OUT_##_axis##_MSB,			\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |		\
				BIT(IIO_CHAN_INFO_ENABLE),		\
}

static const struct iio_chan_spec fxos8700_channels[] = {
	FXOS8700_ACCEL_CHANNEL(X),
	FXOS8700_ACCEL_CHANNEL(Y),
	FXOS8700_ACCEL_CHANNEL(Z),
	FXOS8700_MAGN_CHANNEL(X),
	FXOS8700_MAGN_CHANNEL(Y),
	FXOS8700_MAGN_CHANNEL(Z),
};

static const struct iio_info fxos8700_info = {
	.driver_module		= THIS_MODULE,
	.read_raw		= &fxos8700_read_raw,
	.write_raw		= &fxos8700_write_raw,
};

static int fxos8700_chip_init(struct fxos8700_data *data)
{
	struct device *dev = regmap_get_device(data->regmap);
	unsigned int chip_id;
	int ret;

	ret = regmap_field_read(data->regmap_fields[F_WHO_AM_I], &chip_id);
	if (ret < 0)
		return ret;

	if (chip_id != FXOS8700_CHIP_ID && chip_id != FXOS8700_PRE_CHIP_ID) {
		dev_err(dev, "chip id %d is not supported\n", chip_id);
		return -EINVAL;
	}

	data->chip_id = chip_id;

	ret = fxos8700_mode_set(data, FXOS8700_MODE_STANDBY);
	if (ret < 0)
		return ret;

	/* Hybrid mode set - accel on and magnet on */
	ret = fxos8700_hms_set(data, FXOS8700_HYBRID_ON);
	if (ret < 0) {
		dev_err(dev, "failed to set hybrid mode: %d\n", ret);
		return ret;
	}

	/* Set oversampling to 8x for ODR 200Hz */
	ret = fxos8700_osr_set(data, 0x07);
	if (ret < 0)
		dev_err(dev, "failed to set OSR: %d\n", ret);

	return ret;
}

int fxos8700_core_probe(struct device *dev, struct regmap *regmap,
			const char *name)
{
	struct fxos8700_data *data;
	struct iio_dev *indio_dev;
	struct regmap_field *f;
	int ret;
	int i;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);
	data->regmap = regmap;

	for (i = 0; i < F_MAX_FIELDS; i++) {
		f = devm_regmap_field_alloc(dev, data->regmap,
					    fxos8700_reg_fields[i]);
		if (IS_ERR(f)) {
			dev_err(dev, "failed to alloc regmap field %d: %ld\n",
				i, PTR_ERR(f));
			return PTR_ERR(f);
		}
		data->regmap_fields[i] = f;
	}

	mutex_init(&data->mutex);

	ret = fxos8700_chip_init(data);
	if (ret < 0)
		return ret;

	indio_dev->dev.parent = dev;
	indio_dev->channels = fxos8700_channels;
	indio_dev->num_channels = ARRAY_SIZE(fxos8700_channels);
	indio_dev->name = name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &fxos8700_info;

	ret = pm_runtime_set_active(dev);
	if (ret)
		return ret;

	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 4000);
	pm_runtime_use_autosuspend(dev);

	fxos8700_mode_set(data, FXOS8700_MODE_ACTIVE);

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(dev, "unable to register iio device: %d\n", ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(fxos8700_core_probe);

void fxos8700_core_remove(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fxos8700_data *data = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_put_noidle(dev);

	fxos8700_mode_set(data, FXOS8700_MODE_STANDBY);
}
EXPORT_SYMBOL_GPL(fxos8700_core_remove);

#ifdef CONFIG_PM_SLEEP
static int fxos8700_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fxos8700_data *data = iio_priv(indio_dev);

	fxos8700_mode_set(data, FXOS8700_MODE_STANDBY);

	return 0;
}

static int fxos8700_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fxos8700_data *data = iio_priv(indio_dev);

	fxos8700_mode_set(data, data->prev_mode);

	return 0;
}
#endif

#ifdef CONFIG_PM
static int fxos8700_runtime_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fxos8700_data *data = iio_priv(indio_dev);
	int ret;

	ret = fxos8700_mode_set(data, FXOS8700_MODE_STANDBY);
	if (ret < 0)
		return -EAGAIN;

	return 0;
}

static int fxos8700_runtime_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fxos8700_data *data = iio_priv(indio_dev);
	int ret;

	ret = fxos8700_mode_set(data, FXOS8700_MODE_ACTIVE);
	if (ret < 0)
		return -EAGAIN;

	return 0;
}
#endif

const struct dev_pm_ops fxos8700_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(fxos8700_suspend, fxos8700_resume)
	SET_RUNTIME_PM_OPS(fxos8700_runtime_suspend, fxos8700_runtime_resume,
			   NULL)
};
EXPORT_SYMBOL_GPL(fxos8700_pm_ops);

MODULE_AUTHOR("Rui Miguel Silva <rui.silva@linaro.org>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("FXOS8700 Accel/Magn driver");
