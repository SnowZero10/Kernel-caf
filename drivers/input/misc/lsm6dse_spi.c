/*
 * STMicroelectronics lsm6dse i2c driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * v 1.1.0
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>

#include	"lsm6dse.h"
#include	"lsm6dse_core.h"
#define SENSORS_SPI_READ			0x80

static int lsm6dse_spi_read(struct lsm6dse_data *cdata, u8 reg_addr, int len,
							u8 *data, bool b_lock)
{
	int err;

	struct spi_transfer xfers[] = {
		{
			.tx_buf = cdata->tb.tx_buf,
			.bits_per_word = 8,
			.len = 1,
		},
		{
			.rx_buf = cdata->tb.rx_buf,
			.bits_per_word = 8,
			.len = len,
		}
	};

	if (b_lock)
		mutex_lock(&cdata->bank_registers_lock);

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = reg_addr | SENSORS_SPI_READ;

	err = spi_sync_transfer(to_spi_device(cdata->dev),
						xfers, ARRAY_SIZE(xfers));
	if (err)
		goto acc_spi_read_error;

	memcpy(data, cdata->tb.rx_buf, len*sizeof(u8));
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->bank_registers_lock);

	return len;

acc_spi_read_error:
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->bank_registers_lock);

	return err;
}

static int lsm6dse_spi_write(struct lsm6dse_data *cdata, u8 reg_addr, int len,
							u8 *data, bool b_lock)
{
	int err;

	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LSM6DSE_RX_MAX_LENGTH)
		return -ENOMEM;

	if (b_lock)
		mutex_lock(&cdata->bank_registers_lock);

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = reg_addr;

	memcpy(&cdata->tb.tx_buf[1], data, len);

	err = spi_sync_transfer(to_spi_device(cdata->dev), &xfers, 1);
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->bank_registers_lock);

	return err;
}


static const struct lsm6dse_transfer_function lsm6dse_tf_spi = {
	.write = lsm6dse_spi_write,
	.read = lsm6dse_spi_read,
};

static int lsm6dse_spi_probe(struct spi_device *spi)
{
	int err;
	struct lsm6dse_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->name = spi->modalias;
	cdata->tf = &lsm6dse_tf_spi;
	spi_set_drvdata(spi, cdata);

	/*err = lsm6dse_common_probe(cdata, spi->irq, BUS_SPI);*/
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);

	return err;
}

static int lsm6dse_spi_remove(struct spi_device *spi)
{
	/* TODO: check the function */
	struct lsm6dse_data *cdata = spi_get_drvdata(spi);

	lsm6dse_common_remove(cdata, spi->irq);
	dev_info(cdata->dev, "%s: removed\n", LSM6DSE_ACC_GYR_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM
static int lsm6dse_suspend(struct device *dev)
{
	struct lsm6dse_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lsm6dse_common_suspend(cdata);
}

static int lsm6dse_resume(struct device *dev)
{
	struct lsm6dse_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lsm6dse_common_resume(cdata);
}

static const struct dev_pm_ops lsm6dse_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm6dse_suspend, lsm6dse_resume)
};

#define LSM6DSE_PM_OPS		(&lsm6dse_pm_ops)
#else /* CONFIG_PM */
#define LSM6DSE_PM_OPS		NULL
#endif /* CONFIG_PM */

#ifdef CONFIG_OF
static const struct spi_device_id lsm6dse_ids[] = {
	{"lsm6dse", 0},
	{ }
};
MODULE_DEVICE_TABLE(spi, lsm6dse_ids);

static const struct of_device_id lsm6dse_id_table[] = {
	{.compatible = "st,lsm6dse", },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm6dse_id_table);
#endif

static struct spi_driver lsm6dse_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LSM6DSE_ACC_GYR_DEV_NAME,
		.pm = LSM6DSE_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lsm6dse_id_table,
#endif
	},
	.probe    = lsm6dse_spi_probe,
	.remove   = lsm6dse_spi_remove,
	.id_table = lsm6dse_ids,
};

module_spi_driver(lsm6dse_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lsm6dse spi driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_LICENSE("GPL v2");
