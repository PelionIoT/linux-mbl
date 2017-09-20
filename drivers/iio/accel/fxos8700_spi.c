/*
 * Driver for NXP Fxos8700 Accelerometer and Magnetometer - SPI
 *
 * Copyright (C) 2017 Linaro Ltd.
 *
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include "fxos8700.h"

static const struct regmap_config fxos8700_regmap_spi_conf = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FXOS8700_REG_NVM_DATA_BNK0,
};

static int fxos8700_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &fxos8700_regmap_spi_conf);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return fxos8700_core_probe(&spi->dev, regmap, id->name);
}

static int fxos8700_spi_remove(struct spi_device *spi)
{
	fxos8700_core_remove(&spi->dev);

	return 0;
}

static const struct spi_device_id fxos8700_spi_id[] = {
	{ "fxos8700", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, fxos8700_spi_id);

#ifdef CONFIG_OF
static const struct of_device_id fxos8700_spi_of_match[] = {
	{ .compatible = "nxp,fxos8700", },
	{ },
};
MODULE_DEVICE_TABLE(of, fxos8700_spi_of_match);
#endif

static struct spi_driver fxos8700_spi_driver = {
	.driver = {
		.name = "fxos8700_spi",
		.pm = &fxos8700_pm_ops,
		.of_match_table = of_match_ptr(fxos8700_spi_of_match),
	},
	.probe		= fxos8700_spi_probe,
	.remove		= fxos8700_spi_remove,
	.id_table	= fxos8700_spi_id,
};
module_spi_driver(fxos8700_spi_driver);

MODULE_AUTHOR("Rui Miguel Silva <rui.silva@linaro.org>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("FXOS8700 SPI Accel and Magnet driver");
