/*
 * Driver for NXP Fxos8700 Accelerometer and Magnetometer - I2C
 *
 * Copyright (C) 2017 Linaro Ltd.
 *
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

#include "fxos8700.h"

static const struct regmap_config fxos8700_regmap_i2c_conf = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FXOS8700_REG_NVM_DATA_BNK0,
};

static int fxos8700_i2c_probe(struct i2c_client *i2c,
			      const struct i2c_device_id *id)
{
	struct regmap *regmap;
	const char *name = NULL;

	regmap = devm_regmap_init_i2c(i2c, &fxos8700_regmap_i2c_conf);
	if (IS_ERR(regmap)) {
		dev_err(&i2c->dev, "Failed to register i2c regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	if (id)
		name = id->name;

	return fxos8700_core_probe(&i2c->dev, regmap, name);
}

static int fxos8700_i2c_remove(struct i2c_client *i2c)
{
	fxos8700_core_remove(&i2c->dev);

	return 0;
}

static const struct i2c_device_id fxos8700_i2c_id[] = {
	{ "fxos8700", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fxos8700_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id fxos8700_i2c_of_match[] = {
	{ .compatible = "nxp,fxos8700", },
	{ },
};
MODULE_DEVICE_TABLE(of, fxos8700_i2c_of_match);
#endif

static struct i2c_driver fxos8700_i2c_driver = {
	.driver = {
		.name = "fxos8700_i2c",
		.pm = &fxos8700_pm_ops,
		.of_match_table = of_match_ptr(fxos8700_i2c_of_match),
	},
	.probe		= fxos8700_i2c_probe,
	.remove		= fxos8700_i2c_remove,
	.id_table	= fxos8700_i2c_id,
};
module_i2c_driver(fxos8700_i2c_driver);

MODULE_AUTHOR("Rui Miguel Silva <rui.silva@linaro.org>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("FXOS8700 I2C Accel and Magnet driver");
