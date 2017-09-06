/*
 * Driver for NXP FXAS2100x Gyroscope - SPI
 *
 * Copyright (C) 2017 Linaro Ltd.
 *
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include "fxas2100x.h"

static const struct regmap_config fxas2100x_regmap_spi_conf = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FXAS2100X_REG_CTRL3,
};

static int fxas2100x_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &fxas2100x_regmap_spi_conf);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return fxas2100x_core_probe(&spi->dev, regmap, spi->irq, id->name);
}

static int fxas2100x_spi_remove(struct spi_device *spi)
{
	fxas2100x_core_remove(&spi->dev);

	return 0;
}

static const struct spi_device_id fxas2100x_spi_id[] = {
	{ "fxas2100x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, fxas2100x_spi_id);

#ifdef CONFIG_OF
static const struct of_device_id fxas2100x_spi_of_match[] = {
	{ .compatible = "nxp,fxas2100x", },
	{ },
};
MODULE_DEVICE_TABLE(of, fxas2100x_spi_of_match);
#endif

static struct spi_driver fxas2100x_spi_driver = {
	.driver = {
		.name = "fxas2100x_spi",
		.pm = &fxas2100x_pm_ops,
		.of_match_table = of_match_ptr(fxas2100x_spi_of_match),
	},
	.probe		= fxas2100x_spi_probe,
	.remove		= fxas2100x_spi_remove,
	.id_table	= fxas2100x_spi_id,
};
module_spi_driver(fxas2100x_spi_driver);

MODULE_AUTHOR("Rui Miguel Silva <rui.silva@linaro.org>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("FXAS2100X SPI Gyro driver");
