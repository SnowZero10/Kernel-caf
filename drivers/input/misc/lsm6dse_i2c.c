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
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>

#include	"lsm6dse.h"
#include	"lsm6dse_core.h"

static int lsm6dse_i2c_read(struct lsm6dse_data *cdata, u8 reg_addr, int len,
							u8 *data, bool b_lock)
{
	int err = 0;
	struct i2c_msg msg[2];
	struct i2c_client *client = to_i2c_client(cdata->dev);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = &reg_addr;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	if (b_lock) {
		mutex_lock(&cdata->bank_registers_lock);
		err = i2c_transfer(client->adapter, msg, 2);
		mutex_unlock(&cdata->bank_registers_lock);
	} else
		err = i2c_transfer(client->adapter, msg, 2);

	return err;
}

static int lsm6dse_i2c_write(struct lsm6dse_data *cdata, u8 reg_addr, int len,
							u8 *data, bool b_lock)
{
	int err = 0;
	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(cdata->dev);

	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));
	len++;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len;
	msg.buf = send;

	if (b_lock) {
		mutex_lock(&cdata->bank_registers_lock);
		err = i2c_transfer(client->adapter, &msg, 1);
		mutex_unlock(&cdata->bank_registers_lock);
	} else
		err = i2c_transfer(client->adapter, &msg, 1);

	return err;
}


static const struct lsm6dse_transfer_function lsm6dse_tf_i2c = {
	.write = lsm6dse_i2c_write,
	.read = lsm6dse_i2c_read,
};

static int sensor_regulator_configure(struct lsm6dse_data *data, bool on)
{
	int rc;

	if (!on) {

		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				3300000);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				1800000);

		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				2600000, 3300000);
			if (rc) {
				dev_err(&data->client->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				1800000, 1800000);
			if (rc) {
				dev_err(&data->client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;
reg_vio_put:
	regulator_put(data->vio);

reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, 3300000);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

static int sensor_regulator_power_on(struct lsm6dse_data *data, bool on)
{
	int rc = 0;

	if (on) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			regulator_disable(data->vdd);
			return rc;
		}
	} else {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			rc = regulator_enable(data->vdd);
			dev_err(&data->client->dev,
					"Regulator vio re-enabled rc=%d\n", rc);

			if (!rc) {
				rc = -EBUSY;
				goto enable_delay;
			}
		}
		return rc;
	}

enable_delay:
/*	msleep(130);*/
	dev_dbg(&data->client->dev,
		"Sensor regulator power on =%d\n", on);
	return rc;
}

static int lsm6dse_i2c_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	int err;
	struct lsm6dse_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->name = client->name;
	cdata->tf = &lsm6dse_tf_i2c;
	i2c_set_clientdata(client, cdata);
	cdata->client = client;
	sensor_regulator_configure(cdata, 1);
	sensor_regulator_power_on(cdata, 1);
	err = lsm6dse_common_probe(cdata, client->irq, BUS_I2C);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);
	return err;
}

static int lsm6dse_i2c_remove(struct i2c_client *client)
{
	/* TODO: check the function */
	struct lsm6dse_data *cdata = i2c_get_clientdata(client);

	lsm6dse_common_remove(cdata, client->irq);
	dev_info(cdata->dev, "%s: removed\n", LSM6DSE_ACC_GYR_DEV_NAME);
	kfree(cdata);
	return 0;
}

#ifdef CONFIG_PM
static int lsm6dse_suspend(struct device *dev)
{
	struct lsm6dse_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lsm6dse_common_suspend(cdata);
}

static int lsm6dse_resume(struct device *dev)
{
	struct lsm6dse_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lsm6dse_common_resume(cdata);
}

static const struct dev_pm_ops lsm6dse_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm6dse_suspend, lsm6dse_resume)
};

#define LSM6DSE_PM_OPS		(&lsm6dse_pm_ops)
#else /* CONFIG_PM */
#define LSM6DSE_PM_OPS		NULL
#endif /* CONFIG_PM */


static const struct i2c_device_id lsm6dse_ids[] = {
	{"lsm6dse", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, lsm6dse_ids);

#ifdef CONFIG_OF
static const struct of_device_id lsm6dse_id_table[] = {
	{.compatible = "st,lsm6dse", },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm6dse_id_table);
#endif

static struct i2c_driver lsm6dse_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LSM6DSE_ACC_GYR_DEV_NAME,
		.pm = LSM6DSE_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lsm6dse_id_table,
#endif
	},
	.probe    = lsm6dse_i2c_probe,
	.remove   = lsm6dse_i2c_remove,
	.id_table = lsm6dse_ids,
};

module_i2c_driver(lsm6dse_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics lsm6dse i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_LICENSE("GPL v2");
