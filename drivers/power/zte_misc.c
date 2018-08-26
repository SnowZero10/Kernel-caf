/*
 * Driver for zte misc functions
 * function1: used for translate hardware GPIO to SYS GPIO number
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/power_supply.h>/* for charging eanble/disable conrotl */

#if defined(CONFIG_BOARD_HELEN)
#include <linux/module.h>
#define GPIO_RED_LED_CONTROL	1
#endif

#if defined(CONFIG_BOARD_CHAPEL)
#define GPIO_CHG_EN		0
#endif

#include "zte_misc.h"

struct zte_gpio_info {
	int sys_num;			/* system pin number */
	const char *name;
};

#define MAX_SUPPORT_GPIOS 16
struct zte_gpio_info zte_gpios[MAX_SUPPORT_GPIOS];

static const struct of_device_id zte_misc_of_match[] = {
	{ .compatible = "zte-misc", },
	{ },
};
MODULE_DEVICE_TABLE(of, zte_misc_of_match);

#define CHARGER_BUF_SIZE 0x32

int get_sysnumber_byname(char *name)
{
	int i;

	for (i = 0; i < MAX_SUPPORT_GPIOS; i++) {
		if (zte_gpios[i].name) {
			if (!strcmp(zte_gpios[i].name, name))
				return zte_gpios[i].sys_num;
		}
	}
	return -ENODEV;
}

static int get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	int count = -1;

	pr_info("zte_misc: translate hardware pin to system pin\n");
	node = dev->of_node;
	if (node == NULL)
		return -ENODEV;

	pp = NULL;
	while ((pp = of_get_next_child(node, pp))) {
		if (!of_find_property(pp, "label", NULL)) {
			dev_warn(dev, "Found without labels\n");
			continue;
		}
		count++;
		zte_gpios[count].name = kstrdup(of_get_property(pp, "label", NULL),
								GFP_KERNEL);
		zte_gpios[count].sys_num = of_get_gpio(pp, 0);

		pr_info("zte_misc: sys_number=%d name=%s\n", zte_gpios[count].sys_num, zte_gpios[count].name);
	}
	return 0;
}

/* get hareware version ( board id ) by the gpios defined in "projec name.dts"
*/
#define HW_VER_ID_0  "hw_ver_id_0"
#define HW_VER_ID_1  "hw_ver_id_1"
static int hw_ver_id_0_en = -1;
static int hw_ver_id_1_en = -1;
int	hw_version = -1;
int zte_get_board_ver(void)
{
	return hw_version;
}
static int zte_hw_ver_init(void)
{
	int rc;

	hw_ver_id_0_en = get_sysnumber_byname(HW_VER_ID_0);
	hw_ver_id_1_en = get_sysnumber_byname(HW_VER_ID_1);

	rc = gpio_request(hw_ver_id_0_en, HW_VER_ID_0);
	if (rc) {
		pr_info("unable to request gpio HW_VER_ID_0(%d)\n", rc);
	} else {
		pr_info("success\n");
	}

	rc = gpio_request(hw_ver_id_1_en, HW_VER_ID_1);
	if (rc) {
		pr_info("unable to request gpio HW_VER_ID_0(%d)\n", rc);
	} else {
		pr_info("success\n");
	}
	/* gpio_direction_output(gpio_batt_switch_en,0); */

	return 0;
}

static int zte_get_hw_version(void)
{
	int id, id0, id1;

	if ((hw_ver_id_0_en == -1) || (hw_ver_id_1_en == -1)) {
		pr_info("%s:not support hw_version gpios\n", __func__);
		return -ENODEV;
	}

	gpio_direction_input(hw_ver_id_0_en);
	gpio_direction_input(hw_ver_id_1_en);

	id0 = gpio_get_value(hw_ver_id_0_en);
	id1 = gpio_get_value(hw_ver_id_1_en);

	id = (id0<<1) | id1;
	pr_info("hw_version:0x%02x (id0:%d,id1:%d)\n", id, id0, id1);
	return id;
}


/*battery switch functions*/
static int poweroff_bs = 0;
static int factory_mode = 0;
static int is_factory_mode = 0;
#define BATTERY_SWITCH		"battery_switch"
static int gpio_batt_switch_en = -1;
static int is_use_ti_internal_bs = 0;

static int zte_batt_switch_init(void)
{
	int rc;

	if (is_use_ti_internal_bs)
		return 0;

	gpio_batt_switch_en = get_sysnumber_byname(BATTERY_SWITCH);

	rc = gpio_request(gpio_batt_switch_en, BATTERY_SWITCH);
	if (rc) {
		pr_info("unable to request battery_switch(%d)\n", rc);
	} else {
		pr_info("success\n");
	}
	/* gpio_direction_output(gpio_batt_switch_en,0); */

	return 0;
}
/*Note, 2 kinds of battery switch:
 * (1)fairchild: turn off batt_switch 7.3s later when enable pin given low to high (rising edge)
 *				 and kept high for at least 1 ms
 * (2)TI charger internal battery switch controllled by I2C
*/
/*extern int ti2419x_turn_off_battery_switch(void);*/
static int zte_turn_off_batt_switch(void)
{
	pr_info("%s:turning off battery switch...\n", __func__);
#if 0
	if (is_use_ti_internal_bs)
		ti2419x_turn_off_battery_switch();
	else{
		gpio_direction_output(gpio_batt_switch_en, 0);
		msleep(10);
		gpio_direction_output(gpio_batt_switch_en, 1);
	}
#endif
	return 0;
}

static int poweroff_bs_set(const char *val, struct kernel_param *kp)
{
	int ret;

	if (gpio_batt_switch_en == -1 && is_use_ti_internal_bs == 0) {
		pr_info("%s:not support battery sitch\n", __func__);
		return -ENODEV;
	}

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	if (poweroff_bs != 1)
		return -ENODEV;

	zte_turn_off_batt_switch();
	return 0;
}
module_param_call(poweroff_bs, poweroff_bs_set, NULL,
			&poweroff_bs, 0644);

static int factory_mode_set(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	if (factory_mode == 0)
		is_factory_mode = 0;
	else
		is_factory_mode = 1;

	return 0;
}

static int factory_mode_get(char *buffer, struct kernel_param *kp)
{
	return	snprintf(buffer, sizeof(buffer)-1, "%d", is_factory_mode);
}
module_param_call(factory_mode, factory_mode_set, factory_mode_get,
			&factory_mode, 0644);

int battery_switch_enable(void)
{
	if (gpio_batt_switch_en == -1 && is_use_ti_internal_bs == 0) {
		pr_info("%s:not support battery sitch\n", __func__);
		return -ENODEV;
	}

	if (!is_factory_mode)
		return -ENODEV;

	zte_turn_off_batt_switch();

	return 0;
}
EXPORT_SYMBOL(battery_switch_enable);

/*
  *Emode function to enable/disable 0% shutdown
  */
/* extern int enable_to_shutdown;//defined in ****-charger.c */
int enable_to_shutdown = 1;
static int set_enable_to_shutdown(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	pr_warn("set_enable_to_shutdown to %d\n", enable_to_shutdown);
	return 0;
}

module_param_call(enable_to_shutdown, set_enable_to_shutdown, param_get_uint,
					&enable_to_shutdown, 0644);

static int zte_misc_charging_enabled;/* defined in ****-charger.c */
static int zte_misc_control_charging(const char *val, struct kernel_param *kp)
{
	struct power_supply	*batt_psy;
	int rc;
	const union power_supply_propval enable = {1,};
	const union power_supply_propval disable = {0,};

	rc = param_set_int(val, kp);
	if (rc) {
		pr_err("%s: error setting value %d\n", __func__, rc);
		return rc;
	}

	batt_psy = power_supply_get_by_name("battery");
	if (batt_psy) {
		if (zte_misc_charging_enabled != 0) {
			rc = batt_psy->set_property(batt_psy,
					POWER_SUPPLY_PROP_CHARGING_ENABLED, &enable);
			pr_info("%s: enable charging\n", __func__);
		} else {
			rc = batt_psy->set_property(batt_psy,
					POWER_SUPPLY_PROP_CHARGING_ENABLED, &disable);
			pr_info("%s: disable charging\n", __func__);
		}
		if (rc) {
			pr_err("battery does not export CHARGING_ENABLED: %d\n", rc);
		}
	} else
		pr_err("%s: batt_psy is NULL\n", __func__);

	return 0;
}

module_param_call(charging_enabled, zte_misc_control_charging, param_get_uint,
					&zte_misc_charging_enabled, 0644);

#if defined(GPIO_RED_LED_CONTROL)
#define GPIO_RED_LED_NAME		"zte_red_led"

void zte_misc_red_led_control(bool value)
{
	int gpio_red_led_num = 0;

	gpio_red_led_num = get_sysnumber_byname(GPIO_RED_LED_NAME);
	pr_info("The gpio_red_led_num is %d, And the value is %d\n", gpio_red_led_num, value);

	if (value == true) {
		gpio_direction_output(gpio_red_led_num, 0);
	} else {
		gpio_direction_output(gpio_red_led_num, 1);
	}
}
EXPORT_SYMBOL(zte_misc_red_led_control);

static int red_led_mode = 0;
static int set_red_led_mode(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("Fail to setting value %d\n", ret);
		return ret;
	}

	if (red_led_mode == 0) {
		zte_misc_red_led_control(false);
	} else if (red_led_mode == 1) {
		zte_misc_red_led_control(true);
	}
	pr_info("red_led_mode is %d\n", red_led_mode);
	return 0;
}
module_param_call(red_led_mode, set_red_led_mode, param_get_int, &red_led_mode, 0644);
#endif

#if defined(CONFIG_BOARD_CHAPEL)
#define GPIO_CHG_EN_NAME		"zte_chg_en"
void zte_misc_chg_enable(void)
{
	int gpio_chg_en_num = 0;

	gpio_chg_en_num = get_sysnumber_byname(GPIO_CHG_EN_NAME);
	pr_info("The gpio_chg_en_num is %d\n", gpio_chg_en_num);
	gpio_direction_output(gpio_chg_en_num, 1);
}
#endif

#ifdef CONFIG_ZTE_BATTERY_CAPACITY_MAH
static int design_capacity = -1;
static int zte_misc_get_design_capacity(char *val, struct kernel_param *kp)
{
	struct power_supply *bms_psy;
	union power_supply_propval prop = {0,};
	int zte_design_capacity = 0;
	int rc;

	bms_psy = power_supply_get_by_name("bms");
	if (bms_psy) {
		rc = bms_psy->get_property(bms_psy,
				POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &prop);
		if (rc) {
			pr_err("failed to get battery capacity, error:%d.\n", rc);
			goto out;
		} else {
			zte_design_capacity = prop.intval;
			goto out;
		}
	} else {
		pr_err("bms_psy is NULL.\n");
	}

	bms_psy = power_supply_get_by_name("ti_bms");
	if (bms_psy) {
		rc = bms_psy->get_property(bms_psy,
				POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &prop);
		if (rc) {
			pr_err("failed to get battery capacity, error:%d.\n", rc);
			goto out;
		} else {
			zte_design_capacity = prop.intval;
			goto out;
		}
	} else {
		pr_err("bms_psy is NULL.\n");
	}

	bms_psy = power_supply_get_by_name("on_bms");
	if (bms_psy) {
		rc = bms_psy->get_property(bms_psy,
				POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &prop);
		if (rc) {
			pr_err("failed to get battery capacity, error:%d.\n", rc);
			goto out;
		} else {
			zte_design_capacity = prop.intval;
			goto out;
		}
	} else {
		pr_err("bms_psy is NULL.\n");
	}

	bms_psy = power_supply_get_by_name("max17055_bms");
	if (bms_psy) {
		rc = bms_psy->get_property(bms_psy,
				POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &prop);
		if (rc) {
			pr_err("failed to get battery capacity, error:%d.\n", rc);
			goto out;
		} else {
			zte_design_capacity = prop.intval;
			goto out;
		}
	} else {
		pr_err("batt_psy is NULL.\n");
	}
	pr_err("zty_debug the capacity is %d\n", zte_design_capacity);

out:
	return snprintf(val, CHARGER_BUF_SIZE, "%d", zte_design_capacity);
}
module_param_call(design_capacity, NULL, zte_misc_get_design_capacity,
			&design_capacity, 0644);
#endif

/*
#if defined(CONFIG_BOARD_SWEET)
extern int schedule_update_heartbeat_work(void);
static int store_mode = 0;
static int set_store_mode(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("Fail to setting value %d\n", ret);
		return ret;
	}

	schedule_update_heartbeat_work();

	pr_debug("The store_mode is %d\n", store_mode);
	return 0;
}
module_param_call(store_mode, set_store_mode, param_get_int, &store_mode, 0644);

int get_store_mode(void)
{
	pr_debug("Get_store_mode the store_mode is %d\n", store_mode);
	return store_mode;
}
#endif
*/

struct charging_policy_ops *g_charging_policy_ops = NULL;

static int demo_charging_policy = 0;
static int demo_charging_policy_set(const char *val, struct kernel_param *kp)
{
	int rc;

	rc = param_set_int(val, kp);
	if (rc) {
		pr_err("%s: error setting value %d\n", __func__, rc);
		return rc;
	}

	if (g_charging_policy_ops && g_charging_policy_ops->charging_policy_demo_sts_set)
		g_charging_policy_ops->charging_policy_demo_sts_set(g_charging_policy_ops, demo_charging_policy);
	else
		pr_err("can not set demo charging policy\n");

	pr_info("demo_charging_policy is %d\n", demo_charging_policy);

	return 0;
}
static int demo_charging_policy_get(char *val, struct kernel_param *kp)
{
	if (g_charging_policy_ops && g_charging_policy_ops->charging_policy_demo_sts_get)
		demo_charging_policy = g_charging_policy_ops->charging_policy_demo_sts_get(g_charging_policy_ops);
	else
		pr_err("can not get demo charging policy\n");

	pr_info("demo_charging_policy is %d\n", demo_charging_policy);
	return snprintf(val, 0x2, "%d",  demo_charging_policy);
}
module_param_call(demo_charging_policy, demo_charging_policy_set,
	demo_charging_policy_get, &demo_charging_policy, 0644);

static int expired_charging_policy = 0;
static int expired_charging_policy_set(const char *val, struct kernel_param *kp)
{
	int rc;

	pr_info("expired_charging_policy_set.\n");
	rc = param_set_int(val, kp);
	if (rc) {
		pr_err("%s: error setting value %d\n", __func__, rc);
		return rc;
	}
	pr_info("expired_charging_policy is %d\n", expired_charging_policy);

	return 0;
}
static int expired_charging_policy_get(char *val, struct kernel_param *kp)
{
	if (g_charging_policy_ops && g_charging_policy_ops->charging_policy_expired_sts_get)
		expired_charging_policy = g_charging_policy_ops->charging_policy_expired_sts_get(g_charging_policy_ops);
	else
		pr_err("can not get expired charging policy\n");

	pr_info("expired_charging_policy is %d\n", expired_charging_policy);
	return snprintf(val, 0x2, "%d",  expired_charging_policy);
}
module_param_call(expired_charging_policy, expired_charging_policy_set,
	expired_charging_policy_get, &expired_charging_policy, 0644);

static int charging_time_sec = 0;
static int expired_charging_policy_sec_set(const char *val, struct kernel_param *kp)
{
	int rc;

	rc = param_set_int(val, kp);
	if (rc) {
		pr_err("%s: error setting value %d\n", __func__, rc);
		return rc;
	}

	if (g_charging_policy_ops && g_charging_policy_ops->charging_policy_expired_sec_set)
		g_charging_policy_ops->charging_policy_expired_sec_set(g_charging_policy_ops,
			charging_time_sec);
	else
		pr_err("can not set expired charging policy sec\n");

	pr_info("expired_charging_policy_sec_set is %d\n", charging_time_sec);

	return 0;
}
static int expired_charging_policy_sec_get(char *val, struct kernel_param *kp)
{
	if (g_charging_policy_ops && g_charging_policy_ops->charging_policy_expired_sec_get)
		charging_time_sec =
			g_charging_policy_ops->charging_policy_expired_sec_get(g_charging_policy_ops);
	else
		pr_err("can not get expired charging policy\n");

	pr_info("expired_charging_policy is %d\n", charging_time_sec);
	return snprintf(val, 0x2, "%d",  charging_time_sec);
}
module_param_call(charging_time_sec, expired_charging_policy_sec_set,
	expired_charging_policy_sec_get, &charging_time_sec, 0644);

int zte_misc_register_charging_policy_ops(struct charging_policy_ops *ops)
{
	int ret = 0;

	pr_info("%s.\n", __func__);
	g_charging_policy_ops = ops;

	return ret;
}

/*
vendor_id    02    03    04    17    10   12    15    20      20
vendor_name  ATL   cos   BYD   BAK   LG   SONY  SANYO samsung samsung
resistance   10K   20K   33K   82K   180K 240K  330K  430K    470K
*/
#define BATTERY_VENDOR_NUM 9
int battery_vendor_id[BATTERY_VENDOR_NUM] = {02, 03, 04, 17, 10, 12, 15, 20, 20};
int resistance_kohm[BATTERY_VENDOR_NUM] = {10, 20, 33, 82, 180, 240, 330, 430, 470};
static int battery_module_pack_vendor = 0;
static int battery_module_pack_vendor_get(char *val, struct kernel_param *kp)
{
	struct power_supply *batt_psy;
	union power_supply_propval prop = {0,};
	int resistance = 0, i, rc;

	batt_psy = power_supply_get_by_name("battery");
	if (batt_psy) {
		rc = batt_psy->get_property(batt_psy,
				POWER_SUPPLY_PROP_RESISTANCE_ID, &prop);
		if (rc)
			pr_err("failed to battery module pack vendor, error:%d.\n", rc);
		else {
			resistance = prop.intval / 1000;
			if (resistance > (resistance_kohm[BATTERY_VENDOR_NUM - 1] * 109 / 100)
				|| resistance < (resistance_kohm[0] * 91 / 100)) {
				pr_err("resistance is out of range, %dkohm\n", resistance);
			} else {
				for (i = 0; i < (BATTERY_VENDOR_NUM - 1); i++) {
					if (resistance < resistance_kohm[i] * 109 / 100
						&& resistance > resistance_kohm[i] * 91 / 100) {
						battery_module_pack_vendor = battery_vendor_id[i];
						break;
					}
				}
				pr_info("battery resistance is %dkohm, battery_vendor_id = %2d\n",
					resistance, battery_module_pack_vendor);
			}
		}
	} else
		pr_err("batt_psy is NULL.\n");

	return snprintf(val, 0x4, "%2d", battery_module_pack_vendor);
}
module_param_call(battery_module_pack_vendor, NULL,
	battery_module_pack_vendor_get, &battery_module_pack_vendor, 0644);

/* static int __devinit zte_misc_probe(struct platform_device *pdev) */
static int zte_misc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	int error;

	pr_info("%s +++++\n", __func__);

	error = get_devtree_pdata(dev);
	if (error)
		return error;

	/*if use ti_nternel battery switch,enable it by I2C command not GPIO*/
	is_use_ti_internal_bs = of_property_read_bool(node, "zte,use-ti-charger-internal-battery-switch");
	pr_info("is_use_ti_internal_bs=%d\n", is_use_ti_internal_bs);

	zte_batt_switch_init();

#if defined(CONFIG_BOARD_CHAPEL)
	zte_misc_chg_enable();
#endif

	zte_hw_ver_init();
	hw_version = zte_get_hw_version();
	zte_get_board_ver();
	pr_info("%s ----\n", __func__);
	return 0;
}

/* static int __devexit zte_misc_remove(struct platform_device *pdev) */
static int	zte_misc_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver zte_misc_device_driver = {
	.probe		= zte_misc_probe,
	/* .remove	= __devexit_p(zte_misc_remove),  //zte jiangzhineng changed */
	.remove    = zte_misc_remove,
	.driver		= {
		.name	= "zte-misc",
		.owner	= THIS_MODULE,
		.of_match_table = zte_misc_of_match,
	}
};

int __init zte_misc_init(void)
{
	return platform_driver_register(&zte_misc_device_driver);
}

static void __exit zte_misc_exit(void)
{
	platform_driver_unregister(&zte_misc_device_driver);
}
fs_initcall(zte_misc_init);
module_exit(zte_misc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Misc driver for zte");
MODULE_ALIAS("platform:zte-misc");
