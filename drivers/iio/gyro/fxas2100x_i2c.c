/*
 * Driver for NXP FXAS2100x Gyroscope - I2C
 *
 * Copyright (C) 2017 Linaro Ltd.
 *
 */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

#include "fxas2100x.h"

static const struct regmap_config fxas2100x_regmap_i2c_conf = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FXAS2100X_REG_CTRL3,
};

static int fxas2100x_i2c_probe(struct i2c_client *i2c,
			       const struct i2c_device_id *id)
{
	struct regmap *regmap;
	const char *name = NULL;

	regmap = devm_regmap_init_i2c(i2c, &fxas2100x_regmap_i2c_conf);
	if (IS_ERR(regmap)) {
		dev_err(&i2c->dev, "Failed to register i2c regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	if (id)
		name = id->name;

	return fxas2100x_core_probe(&i2c->dev, regmap, i2c->irq, name);
}

static int fxas2100x_i2c_remove(struct i2c_client *i2c)
{
	fxas2100x_core_remove(&i2c->dev);

	return 0;
}

static const struct i2c_device_id fxas2100x_i2c_id[] = {
	{ "fxas2100x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fxas2100x_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id fxas2100x_i2c_of_match[] = {
	{ .compatible = "nxp,fxas2100x", },
	{ },
};
MODULE_DEVICE_TABLE(of, fxas2100x_i2c_of_match);
#endif

static struct i2c_driver fxas2100x_i2c_driver = {
	.driver = {
		.name = "fxas2100x_i2c",
		.pm = &fxas2100x_pm_ops,
		.of_match_table = of_match_ptr(fxas2100x_i2c_of_match),
	},
	.probe		= fxas2100x_i2c_probe,
	.remove		= fxas2100x_i2c_remove,
	.id_table	= fxas2100x_i2c_id,
};
module_i2c_driver(fxas2100x_i2c_driver);

MODULE_AUTHOR("Rui Miguel Silva <rui.silva@linaro.org>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("FXAS2100X I2C Gyro driver");
