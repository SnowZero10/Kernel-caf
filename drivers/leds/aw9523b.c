/*
 * leds-aw9523b
 */
#define pr_fmt(fmt) "[AW9523B]%s: " fmt, __func__
#define DEBUG

#include  <linux/err.h>
#include  <linux/errno.h>
#include  <linux/delay.h>
#include  <linux/fs.h>
#include  <linux/i2c.h>
#include  <linux/input.h>
#include  <linux/uaccess.h>
#include  <linux/workqueue.h>
#include  <linux/irq.h>
#include  <linux/gpio.h>
#include  <linux/interrupt.h>
#include  <linux/slab.h>
#include  <linux/pm.h>
#include  <linux/module.h>
#include  <linux/regulator/consumer.h>
#include  <linux/of_gpio.h>
#include  <linux/leds.h>
#include  <linux/ctype.h>
#include  <linux/timer.h>
#include  <linux/wakelock.h>

#define    P0_LEVEL         0x00
#define    P1_LEVEL         0x01
#define    P0_DIRECTION         0x04
#define    P1_DIRECTION         0x05
#define    P0_INTERRUPT_ENABLE         0x06
#define    P1_INTERRUPT_ENABLE         0x07
#define    AW_ID             0x10
#define    MODE_P0         0x12
#define    MODE_P1         0x13
#define    LEDCTLP1_0         0x20
#define    LEDCTLP1_1         0x21
#define    LEDCTLP1_2         0x22
#define    LEDCTLP1_3         0x23
#define    LEDCTLP0_0         0x24
#define    LEDCTLP0_1         0x25
#define    LEDCTLP0_2         0x26
#define    LEDCTLP0_3         0x27
#define    LEDCTLP0_4         0x28
#define    LEDCTLP0_5         0x29
#define    LEDCTLP0_6         0x2A
#define    LEDCTLP0_7         0x2B
#define    LEDCTLP1_4         0x2C
#define    LEDCTLP1_5         0x2D
#define    LEDCTLP1_6         0x2E
#define    LEDCTLP1_7         0x2F

#define    SW_RSTN             0x7F


#define   I2C_RETRY_DELAY        5
#define   I2C_RETRIES          5

/*#define     LED_RST        "led_rst"*/
#define     AW9523B_MODE_RED_ALLON 0x00
#define     AW9523B_MODE_GREEN_ALLON 0x01
#define     AW9523B_MODE_ORANGE_ALLON 0x03
/*static int led_rst = 0;*/


#define AW9523B_LED_ON_MESC_INVAILD -1
#define AW9523B_LED_OFF_MESC_INVAILD -1
enum aw9523b_number {
	AW9523B_NUMBER_0,
	AW9523B_NUMBER_1,
	AW9523B_NUMBER_2,
	AW9523B_NUMBER_3,
	AW9523B_NUMBER_4,
	AW9523B_NUMBER_5,
	AW9523B_NUMBER_6,
	AW9523B_NUMBER_7,
	AW9523B_NUMBER_8,
	AW9523B_NUMBER_9,
	AW9523B_NUMBER_10,
	AW9523B_NUMBER_11,
	AW9523B_NUMBER_12,
	AW9523B_NUMBER_13,
	AW9523B_NUMBER_14,
	AW9523B_NUMBER_15,
	AW9523B_NUMBER_INVAILD,
};


enum aw9523b_mode {
	AW9523B_MODE_ON = 0,          /* on */
	AW9523B_MODE_OFF,             /* off */
	AW9523B_MODE_BLINK,
	AW9523B_MODE_ALL_ON,
	AW9523B_MODE_ALL_OFF,
	AW9523B_MODE_INVAILD,
};

struct wake_lock aw9523b_wake_lock;

struct aw9523b_mode_map {
	const char *mode;
	enum aw9523b_mode mode_val;
};
static struct aw9523b_mode_map mode_map[] = {
	{ "on", AW9523B_MODE_ON },
	{ "off", AW9523B_MODE_OFF },
	{ "blink", AW9523B_MODE_BLINK },
	{ "allon", AW9523B_MODE_ALL_ON },
	{ "alloff", AW9523B_MODE_ALL_OFF },
	{ "invaild", AW9523B_MODE_INVAILD },
};
struct zte_gpio_info {
	int sys_num;/*system pin number*/
	const char *name;
};
#define MAX_SUPPORT_GPIOS 16
struct zte_gpio_info zte_led_gpios[MAX_SUPPORT_GPIOS];


struct aw9523b_data {
	struct i2c_client *client;
	struct led_classdev cdev;
	struct regulator *regulator;

	atomic_t enabled;
	enum aw9523b_mode mode;
	enum aw9523b_number number;

	int gpio_rst;
	int led_on_mesc;
	int led_off_mesc;
	u8 led_brightness;
	u8 reg_addr;

	u8 mode_data[32];
	u8 number_data[32];
	int led_on_mesc_data[32];
	int led_off_mesc_data[32];
	u8 led_brightness_data[32];
	u8 reg_addr_data[32];

	bool led0_led_enable;
	bool led1_led_enable;
	bool led2_led_enable;
	bool led3_led_enable;

	bool led4_led_enable;
	bool led5_led_enable;
	bool led6_led_enable;
	bool led7_led_enable;

	bool led8_led_enable;
	bool led9_led_enable;
	bool led10_led_enable;
	bool led11_led_enable;

	bool led12_led_enable;
	bool led13_led_enable;
	bool led14_led_enable;
	bool led15_led_enable;

};
struct aw9523b_data	*aw9523b_blink_data = NULL;

static void led_timer_func0(unsigned long data);
static void led_timer_ftm_func(unsigned long data);

static struct work_struct timerwork0;

static struct work_struct timerwork_ftm;

static struct workqueue_struct *led_timer_wq0;

static struct workqueue_struct *led_timer_wq_ftm;


static DEFINE_TIMER(led_timer0, led_timer_func0, 0, 0);

static DEFINE_TIMER(led_ftm_timer, led_timer_ftm_func, 0, 0);

static int aw9523b_i2c_write(struct aw9523b_data *drvdata, u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = drvdata->client->addr,
			.flags = drvdata->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(drvdata->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&drvdata->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}
static int aw9523b_i2c_read(struct i2c_client *client, u8 reg, u8 *value)
{
	int tmp;

	tmp = i2c_smbus_read_byte_data(client, reg);
	if (tmp < 0)
		return tmp;

	*value = tmp;

	return 0;
}
static int aw9523b_register_write(struct aw9523b_data *drvdata, u8 *buf,
	u8 reg_address, u8 new_value)
{
	int err = -1;
	u8 buf_temp[7];
	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
		buf_temp[0] = reg_address;
		buf_temp[1] = new_value;
	err = aw9523b_i2c_write(drvdata, buf_temp, 1);
		if (err < 0)
			return err;
	return err;
}

static int aw9523b_i2c_test(struct aw9523b_data *drvdata)
{
	int err = 0;
	u8 val = 0;

	err = aw9523b_i2c_read(drvdata->client, P0_LEVEL, &val);
	pr_info("P0_LEVEL 0x00 is 0x%x\n", val);
	err = aw9523b_i2c_read(drvdata->client, P1_LEVEL, &val);
	pr_info("P1_LEVEL 0x01 is %x\n", val);
	err = aw9523b_i2c_read(drvdata->client, P0_DIRECTION, &val);
	pr_info("P0_DIRECTION 0x04 is %x\n", val);
	err = aw9523b_i2c_read(drvdata->client, P1_DIRECTION, &val);
	pr_info("P1_DIRECTION 0x05 is %x\n", val);
	err = aw9523b_i2c_read(drvdata->client, P0_INTERRUPT_ENABLE, &val);
	pr_info("P0_INTERRUPT_ENABLE 0x06 is %x\n", val);
	err = aw9523b_i2c_read(drvdata->client, P1_INTERRUPT_ENABLE, &val);
	pr_info("P1_INTERRUPT_ENABLE 0x07 is %x\n", val);
	err = aw9523b_i2c_read(drvdata->client, AW_ID, &val);
	pr_info("AW_IADR 0x10 is 0x%x\n", val);
	err = aw9523b_i2c_read(drvdata->client, MODE_P0, &val);
	pr_info("MODE_P1 0x12 is 0x%x\n", val);
	err = aw9523b_i2c_read(drvdata->client, MODE_P1, &val);
	pr_info("MODE_P1 0x13 is 0x%x\n", val);


	return err;
}
#if 0
 /* HW reset*/
static int aw9523b_hw_init(struct aw9523b_data *drvdata)
{
	int err = 0;
	u8 buf[7];

	gpio_direction_output(led_rst, 1);

	gpio_set_value(led_rst, 0);
	mdelay(20);/*msleep*/
	gpio_set_value(led_rst, 1);
	mdelay(8);

	aw9523b_register_write(drvdata, buf, SW_RSTN, 0x00);
	aw9523b_register_write(drvdata, buf, MODE_P0, 0x00);
	aw9523b_register_write(drvdata, buf, MODE_P1, 0x00);
	mdelay(8);
	return err;
}
#endif
static void led_timer_func0(unsigned long data)
{
	queue_work(led_timer_wq0, &timerwork0);
}
static void led_timer_ftm_func(unsigned long data)
{
	queue_work(led_timer_wq_ftm, &timerwork_ftm);
}
static void led_timer_handler0(struct work_struct *work)
{
	static u8 val = 0;
	static u8 val_backup = 0;
	u8 buf[7];
	int i = 0;

	int delay_time;

	struct aw9523b_data *drvdata;

	drvdata = aw9523b_blink_data;

	pr_info("entry  led_timer_handler0\n");
	if (val_backup == 0) {
		for (i = 0; i < 16; i++) {
			if (drvdata->mode_data[i] == AW9523B_MODE_BLINK) {
				val = drvdata->led_brightness_data[i];
				pr_info("entry  led_timer_handler1 blink val = %d ,reg_addr = %d,drvdata->brightness = %d\n",
					val, drvdata->reg_addr_data[i], drvdata->led_brightness_data[i]);
				aw9523b_register_write(drvdata, buf, drvdata->reg_addr_data[i], val);
				delay_time = drvdata->led_off_mesc_data[i];
				pr_info("entry  led_timer_handler1 blink led_off_mesc_data = %d\n", delay_time);
			}
		}
	val_backup = val;
	} else {

		for (i = 0; i < 16; i++) {
			if (drvdata->mode_data[i] == AW9523B_MODE_BLINK) {
			pr_info("entry  led_timer_handler1 blink val = %d ,reg_addr = %d,number = %d\n",
					val, drvdata->reg_addr_data[i], i);
			aw9523b_register_write(drvdata, buf, drvdata->reg_addr_data[i], 0X00);
			delay_time = drvdata->led_on_mesc_data[i];
			pr_info("entry  led_timer_handler1 blink led_on_mesc_data = %d\n", delay_time);
			}
		}
	val_backup = 0;
	}
	mod_timer(&led_timer0, jiffies + ((unsigned long)delay_time/10));

	pr_info("entry  led_timer0 val_backup = %d\n", val_backup);

}
static void led_timer_ftm_handler (struct work_struct *work)
{
	u8 buf[7];

	int mode;
	struct aw9523b_data *drvdata;

	drvdata = aw9523b_blink_data;

	mode = drvdata->mode_data[31];
	pr_info("entry  AW9523Bdrvdata->mode_data[31]= %d\n", drvdata->mode_data[31]);
	del_timer_sync(&led_timer0);
	if (mode == AW9523B_MODE_RED_ALLON) {
		pr_info("entry  AW9523B_MODE_RED_ALLON\n");
		aw9523b_register_write(drvdata, buf, LEDCTLP1_3, 0xFF);/*3*/
		aw9523b_register_write(drvdata, buf, LEDCTLP0_1, 0xFF);/*5*/
		aw9523b_register_write(drvdata, buf, LEDCTLP0_3, 0xFF);/*7*/
		aw9523b_register_write(drvdata, buf, LEDCTLP0_5, 0xFF);/*9*/
		aw9523b_register_write(drvdata, buf, LEDCTLP0_7, 0xFF);/*11*/
		aw9523b_register_write(drvdata, buf, LEDCTLP1_5, 0xFF);/*13*/
		aw9523b_register_write(drvdata, buf, LEDCTLP1_7, 0xFF);/*15*/

		aw9523b_register_write(drvdata, buf, LEDCTLP0_0, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_0, 0x00);/*0*/
		aw9523b_register_write(drvdata, buf, LEDCTLP1_1, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_2, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_2, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_4, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_6, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_4, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_6, 0x00);
	} else if (mode == AW9523B_MODE_GREEN_ALLON) {
		pr_info("entry  AW9523B_MODE_GREEN_ALLON\n");
		aw9523b_register_write(drvdata, buf, LEDCTLP1_0, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_1, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_2, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_0, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_2, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_4, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_6, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_4, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_6, 0xFF);

		aw9523b_register_write(drvdata, buf, LEDCTLP1_3, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_1, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_3, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_5, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_7, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_5, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_7, 0x00);
	} else if (mode == AW9523B_MODE_ORANGE_ALLON) {
		pr_info("entry  AW9523B_MODE_ORANGE_ALLON\n");
		aw9523b_register_write(drvdata, buf, LEDCTLP1_2, 0x7D);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_3, 0x19);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_0, 0x7D);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_1, 0x19);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_2, 0x7D);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_3, 0x19);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_4, 0x7D);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_5, 0x19);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_6, 0x7D);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_7, 0x19);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_4, 0x7D);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_5, 0x19);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_6, 0x7D);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_7, 0x19);

		aw9523b_register_write(drvdata, buf, LEDCTLP1_0, 0);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_1, 0);
	} else {
		pr_info("entry  AW9523B_MODE_DEFAULT\n");
		aw9523b_register_write(drvdata, buf, LEDCTLP1_0, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_1, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_2, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_3, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_0, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_1, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_2, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_3, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_4, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_5, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_6, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_7, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_4, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_5, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_6, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_7, 0xFF);
	}
}
static int aw9523b_set_registers(struct aw9523b_data *drvdata)
{
	int err = 0;
	u8 buf[7];
	int i = 0;
	int j = 0;
	int delay_time;
	bool del_timer = true;
	static bool set_timer = false;

	i = drvdata->number;

	delay_time = drvdata->led_on_mesc_data[i];

	if (drvdata->mode == AW9523B_MODE_ALL_ON) {
		pr_info("entry  AW9523B_MODE_ALL_ON\n");
		del_timer_sync(&led_timer0);
		set_timer = false;
		aw9523b_register_write(drvdata, buf, LEDCTLP1_0, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_1, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_2, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_3, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_0, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_1, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_2, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_3, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_4, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_5, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_6, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_7, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_4, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_5, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_6, 0xFF);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_7, 0xFF);
	} else if (drvdata->mode == AW9523B_MODE_ALL_OFF) {
		pr_info("entry  AW9523B_MODE_ALL_OFF\n");
		del_timer_sync(&led_timer0);
		set_timer = false;
		aw9523b_register_write(drvdata, buf, LEDCTLP1_0, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_1, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_2, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_3, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_0, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_1, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_2, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_3, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_4, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_5, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_6, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_7, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_4, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_5, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_6, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_7, 0x00);
	} else if ((drvdata->mode_data[i] == AW9523B_MODE_OFF) || (drvdata->mode_data[i] == AW9523B_MODE_ON)) {
		for (j = 0; j < 16; j++) {
				if (drvdata->mode_data[j] == AW9523B_MODE_BLINK) {
						del_timer = false;
				}
		}
		if (del_timer) {
				del_timer_sync(&led_timer0);
				set_timer = false;

		}
		pr_info("entry  aw9523b_MODE_ON number = %d ,reg_addr = %d\n",
				drvdata->number, drvdata->reg_addr_data[i]);
		aw9523b_register_write(drvdata, buf, drvdata->reg_addr_data[i], drvdata->led_brightness_data[i]);
	} else if (drvdata->mode_data[i] == AW9523B_MODE_BLINK) {
		pr_info("entry  aw9523b_MODE_BLINK number = %d\n", drvdata->number);
		if (!set_timer) {
				mod_timer(&led_timer0, jiffies + (unsigned long)delay_time/10);
				set_timer = true;
		}
	} else {
		pr_info("entry  default\n");
		aw9523b_register_write(drvdata, buf, LEDCTLP1_0, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_1, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_2, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_3, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_0, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_1, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_2, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_3, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_4, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_5, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_6, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP0_7, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_4, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_5, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_6, 0x00);
		aw9523b_register_write(drvdata, buf, LEDCTLP1_7, 0x00);
		}
		return err;
}

static ssize_t attr_aw9523b_set_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct aw9523b_data *drvdata = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	drvdata->mode = AW9523B_MODE_OFF;
	aw9523b_set_registers(drvdata);
	if (val) {
		atomic_set(&drvdata->enabled, 1);
	} else {
		atomic_set(&drvdata->enabled, 0);
	}
	return size;
}
static ssize_t attr_aw9523b_get_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct aw9523b_data *drvdata = dev_get_drvdata(dev);
	int val = atomic_read(&drvdata->enabled);

	return snprintf(buf, sizeof(val) + 2, "%d\n", val);

}
static int aw9523b_get_mode_from_str(const char *str)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mode_map); i++) {
		if (sysfs_streq(str, mode_map[i].mode)) {
			pr_debug("attr_aw9523b_set_mode,mode_map = %d\n", mode_map[i].mode_val);
			return mode_map[i].mode_val;
		}
	}
	pr_debug("attr_aw9523b_set_mode error here\n");
	return -EINVAL;
}
static int hexval(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	else if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	else if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return 0;
}
long atol(const char *num)
{
	long value = 0;
	int neg = 0;

	if (num[0] == '0' && num[1] == 'x') {
		num += 2;
		while (*num && isxdigit(*num))
		value = value * 16 + hexval(*num++);
	} else {
		if (num[0] == '-') {
			neg = 1;
			num++;
		}
	while (*num && isdigit(*num))
		value = value * 10 + *num++  - '0';
	}

	if (neg)
		value = -value;

	return value;
}

int atoi(const char *num)
{
	return atol(num);
}
static ssize_t attr_aw9523b_set_ftm_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)

{
	struct aw9523b_data *drvdata = dev_get_drvdata(dev);
	int led_ftm_mode = AW9523B_MODE_INVAILD;
	int val = atomic_read(&drvdata->enabled);
	int delay_time;
	u8 buf_led[7];

	drvdata = aw9523b_blink_data;

	drvdata->led_off_mesc_data[31] = 1000;
	delay_time = drvdata->led_off_mesc_data[31];
	pr_debug("attr_aw9523b_set_mode\n");
	if (val == 0) {
		pr_info("atomic_cmpxchg is disabled\n");
		return AW9523B_MODE_OFF;
	}

	led_ftm_mode = atoi(buf);
	drvdata->mode_data[31]  = led_ftm_mode;
	if ((led_ftm_mode == AW9523B_MODE_RED_ALLON) || (led_ftm_mode == AW9523B_MODE_GREEN_ALLON)
	|| (led_ftm_mode == AW9523B_MODE_ORANGE_ALLON)) {
		aw9523b_register_write(drvdata, buf_led, LEDCTLP1_0, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP1_1, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP1_2, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP1_3, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP0_0, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP0_1, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP0_2, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP0_3, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP0_4, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP0_5, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP0_6, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP0_7, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP1_4, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP1_5, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP1_6, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP1_7, 0x00);
		mod_timer(&led_ftm_timer, jiffies + (unsigned long)delay_time/10);
		pr_debug("attr_aw9523b_complete\n");
	} else {
		aw9523b_register_write(drvdata, buf_led, LEDCTLP1_0, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP1_1, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP1_2, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP1_3, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP0_0, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP0_1, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP0_2, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP0_3, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP0_4, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP0_5, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP0_6, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP0_7, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP1_4, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP1_5, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP1_6, 0x00);
		aw9523b_register_write(drvdata, buf_led, LEDCTLP1_7, 0x00);
	}

	pr_debug("attr_aw9523b_set_ftm_mode !!!!!!!_mode = %d\n", led_ftm_mode);
	return size;
}
static ssize_t attr_aw9523b_get_ftm_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct aw9523b_data *drvdata = dev_get_drvdata(dev);

	int val = drvdata->mode_data[31];

	pr_info("get aw9523 drvdata->ftm_mode =%d\n", drvdata->mode_data[31]);

	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}
bool atcmd_test = false;
static ssize_t attr_aw9523b_set_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct aw9523b_data *drvdata = dev_get_drvdata(dev);
	int mode = AW9523B_MODE_INVAILD;
	int val = atomic_read(&drvdata->enabled);

	int i = drvdata->number;

	pr_debug("attr_aw9523b_set_mode\n");
	if (val == 0) {
		pr_info("atomic_cmpxchg is disabled\n");
		return AW9523B_MODE_OFF;
	}
	mode = aw9523b_get_mode_from_str(buf);
	if (mode < 0) {
		dev_err(dev, "Invalid mode\n");
		return mode;
	}
	if ((mode == AW9523B_MODE_ALL_ON) || (mode == AW9523B_MODE_ALL_OFF)) {
		atcmd_test = true;
	}
	if (atcmd_test) {
		if ((mode == AW9523B_MODE_ALL_ON) || (mode == AW9523B_MODE_ALL_OFF)) {
			drvdata->mode_data[i]  = mode;
			drvdata->mode = mode;
		} else {
			return size;
		}
	} else {
		drvdata->mode_data[i]  = mode;
		drvdata->mode = mode;
	}
	pr_debug("attr_aw9523b_set_mode,drvdata->mode = %d\n", mode);

	return size;
}
static ssize_t attr_aw9523b_get_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct aw9523b_data *drvdata = dev_get_drvdata(dev);

	int i = drvdata->number;

	int val = drvdata->mode_data[i];

	pr_info("get aw9523 drvdata->mode =%d\n", drvdata->mode_data[i]);

	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}


static ssize_t attr_aw9523b_set_number(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct aw9523b_data *drvdata = dev_get_drvdata(dev);
	int number = AW9523B_NUMBER_INVAILD;

	number = atoi(buf);
	if (number < 0) {
		dev_err(dev, "Invalid mode\n");
	return number;
	}
	if (atcmd_test) {
		return size;
	}
	drvdata->number = number;
	pr_info("set aw9523 drvdata->number =%d\n", drvdata->number);
	switch (drvdata->number) {
	case AW9523B_NUMBER_0:
		drvdata->reg_addr_data[0] = LEDCTLP1_0;
		pr_info("get drvdata->reg_addr_data0 =%d\n", drvdata->reg_addr_data[0]);
		break;
	case AW9523B_NUMBER_1:
		drvdata->reg_addr_data[1]  = LEDCTLP1_1;
		pr_info("get drvdata->reg_addr_data1 =%d\n", drvdata->reg_addr_data[1]);
		break;
	case AW9523B_NUMBER_2:
		drvdata->reg_addr_data[2] = LEDCTLP1_2;
		break;
	case AW9523B_NUMBER_3:
		drvdata->reg_addr_data[3] = LEDCTLP1_3;
		break;
	case AW9523B_NUMBER_4:
		drvdata->reg_addr_data[4] = LEDCTLP0_0;
		break;
	case AW9523B_NUMBER_5:
		drvdata->reg_addr_data[5] = LEDCTLP0_1;
		break;
	case AW9523B_NUMBER_6:
		drvdata->reg_addr_data[6] = LEDCTLP0_2;
		break;
	case AW9523B_NUMBER_7:
		drvdata->reg_addr_data[7] = LEDCTLP0_3;
		break;
	case AW9523B_NUMBER_8:
		drvdata->reg_addr_data[8] = LEDCTLP0_4;
		break;
	case AW9523B_NUMBER_9:
		drvdata->reg_addr_data[9] = LEDCTLP0_5;
		break;
	case AW9523B_NUMBER_10:
		drvdata->reg_addr_data[10] = LEDCTLP0_6;
		break;
	case AW9523B_NUMBER_11:
		drvdata->reg_addr_data[11] = LEDCTLP0_7;
		break;
	case AW9523B_NUMBER_12:
		drvdata->reg_addr_data[12] = LEDCTLP1_4;
		break;
	case AW9523B_NUMBER_13:
		drvdata->reg_addr_data[13] = LEDCTLP1_5;
		break;
	case AW9523B_NUMBER_14:
		drvdata->reg_addr_data[14] = LEDCTLP1_6;
		break;
	case AW9523B_NUMBER_15:
		drvdata->reg_addr_data[15] = LEDCTLP1_7;
		break;
	case AW9523B_NUMBER_INVAILD:
		break;
	}
	return size;
}
static ssize_t attr_aw9523b_get_number(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct aw9523b_data *drvdata = dev_get_drvdata(dev);

	int val = drvdata->number;

	pr_info("get drvdata->number =%d\n", drvdata->number);

	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}
static ssize_t attr_aw9523b_set_led_on_mesc(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct aw9523b_data *drvdata = dev_get_drvdata(dev);

	int led_on_mesc = AW9523B_LED_ON_MESC_INVAILD;
	int i = 0;

	led_on_mesc = atoi(buf);
	if (led_on_mesc < 0) {
		dev_err(dev, "Invalid led_on_mesc\n");
		return led_on_mesc;
	}
	i = drvdata->number;
	drvdata->led_on_mesc_data[i] = led_on_mesc;
	pr_info("get drvdata->led_on_mesc =%d\n", drvdata->led_on_mesc_data[i]);

	return size;
}
static ssize_t attr_aw9523b_get_led_on_mesc(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct aw9523b_data *drvdata = dev_get_drvdata(dev);

	int i = drvdata->number;

	int val = drvdata->led_on_mesc_data[i];

	pr_info("get drvdata->led_on_mesc =%d\n", drvdata->led_on_mesc_data[i]);

	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}
static ssize_t attr_aw9523b_set_led_off_mesc(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct aw9523b_data *drvdata = dev_get_drvdata(dev);

	int led_off_mesc = AW9523B_LED_OFF_MESC_INVAILD;
	int i = 0;

	led_off_mesc = atoi(buf);

	if (led_off_mesc < 0) {
		dev_err(dev, "Invalid led_off\n");
		return led_off_mesc;
	}
	i = drvdata->number;
	drvdata->led_off_mesc_data[i] = led_off_mesc;
	pr_info("set drvdata->led_off_mesc =%d\n", drvdata->led_off_mesc_data[i]);
	return size;
}
static ssize_t attr_aw9523b_get_led_off_mesc(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct aw9523b_data *drvdata = dev_get_drvdata(dev);

	int i = drvdata->number;

	int val = drvdata->led_off_mesc_data[i];

	pr_info("drvdata->led_off =%d\n", drvdata->led_off_mesc_data[i]);

	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}

static DEVICE_ATTR(enable, 0644, attr_aw9523b_get_enable, attr_aw9523b_set_enable);
static DEVICE_ATTR(mode, 0644, attr_aw9523b_get_mode, attr_aw9523b_set_mode);
static DEVICE_ATTR(number, 0644, attr_aw9523b_get_number, attr_aw9523b_set_number);
static DEVICE_ATTR(led_on_mesc, 0644, attr_aw9523b_get_led_on_mesc, attr_aw9523b_set_led_on_mesc);
static DEVICE_ATTR(led_off_mesc, 0644, attr_aw9523b_get_led_off_mesc, attr_aw9523b_set_led_off_mesc);
static DEVICE_ATTR(led_ftm_mode, 0644, attr_aw9523b_get_ftm_mode, attr_aw9523b_set_ftm_mode);

static void aw9523b_brightness_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	struct aw9523b_data *drvdata;

	int err = 0;
	int i = 0;

	drvdata = container_of(led_cdev, struct aw9523b_data, cdev);

	i = drvdata->number;

	drvdata->led_brightness_data[i] = value;

	pr_debug("%s: aw9523b entry and brightness will be %d\n",  __func__, value);
	pr_debug("aw9523b entry and brightness mode = %d, led_on_mesc = %d, led_off_mesc = %d, number = %d\n",
		drvdata->mode_data[i], drvdata->led_on_mesc_data[i],
		drvdata->led_off_mesc_data[i], drvdata->number);
	if ((drvdata->mode_data[i] == AW9523B_MODE_ON) || (drvdata->mode_data[i] == AW9523B_MODE_OFF)) {
		if ((drvdata->mode_data[i] != AW9523B_MODE_INVAILD)
		&& (drvdata->number !=  AW9523B_NUMBER_INVAILD)) {
			err = aw9523b_set_registers(drvdata);
			pr_debug("aw9523b_set_registers  success = %d\n", err);
		}
	} else {
		err = aw9523b_set_registers(drvdata);
		pr_debug("aw9523b_set_registers  success= %d\n", err);
	}

	aw9523b_blink_data = drvdata;

}

static int aw9523b_power_Off(struct aw9523b_data *drvdata)
{

	int err = -1;

	pr_info("entry\n");
	msleep(200);
	return err;
}

int notifyLight_parse_dt_aw9523b(struct device *dev,
		struct aw9523b_data *drvdata)
{
	struct device_node *np = dev->of_node;

	int rc;
	u32 temp_val;

	drvdata->gpio_rst = -1;

	rc  = of_property_read_u32(np, "aw9523b_mode", &temp_val);
	pr_info("aw9523b temp_val is %d\n", temp_val);

	drvdata->led0_led_enable = of_property_read_bool(np, "aw9523b,led0");
	drvdata->led1_led_enable = of_property_read_bool(np, "aw9523b,led1");
	drvdata->led2_led_enable = of_property_read_bool(np, "aw9523b,led2");
	drvdata->led3_led_enable = of_property_read_bool(np, "aw9523b,led3");
	drvdata->led4_led_enable = of_property_read_bool(np, "aw9523b,led4");
	drvdata->led5_led_enable = of_property_read_bool(np, "aw9523b,led5");
	drvdata->led6_led_enable = of_property_read_bool(np, "aw9523b,led6");
	drvdata->led7_led_enable = of_property_read_bool(np, "aw9523b,led7");
	drvdata->led8_led_enable = of_property_read_bool(np, "aw9523b,led8");
	drvdata->led9_led_enable = of_property_read_bool(np, "aw9523b,led9");
	drvdata->led10_led_enable = of_property_read_bool(np, "aw9523b,led10");
	drvdata->led11_led_enable = of_property_read_bool(np, "aw9523b,led11");
	drvdata->led12_led_enable = of_property_read_bool(np, "aw9523b,led12");
	drvdata->led13_led_enable = of_property_read_bool(np, "aw9523b,led13");
	drvdata->led14_led_enable = of_property_read_bool(np, "aw9523b,led14");
	drvdata->led15_led_enable = of_property_read_bool(np, "aw9523b,led15");

	if (rc < 0)
		goto parse_error;
	drvdata->mode = temp_val;
	return 0;
parse_error:
	dev_err(dev, "parse property is failed, rc = %d\n", rc);
	return -EINVAL;
}

static int aw9523b_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct aw9523b_data *drvdata;
	int err = -1;
	int rc = 0;

	pr_info("aw9523b entry\n");
	if (client == NULL) {
		pr_info("i2c client error\n");
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENXIO;
		pr_info("error1: i2c check error\n");
		goto exit_check_functionality_failed;
	}
	pr_info("i2c check success\n");
	drvdata = kzalloc(sizeof(struct aw9523b_data), GFP_KERNEL);

	if (drvdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for module data: "
			"%d\n", err);
		pr_info("aw9523b error2: drvdata is null\n");
		goto exit_check_functionality_failed;
	}
	drvdata->led0_led_enable = false;
	drvdata->led1_led_enable = false;
	drvdata->led2_led_enable = false;

	if (client->dev.of_node) {
		err = notifyLight_parse_dt_aw9523b(&client->dev, drvdata);
		if (err) {
			dev_err(&client->dev, "Failed to parse device tree\n");
			err = -EINVAL;
			kfree(drvdata);
			return err;
		}
	} else {
			dev_err(&client->dev, "No valid platform data. exiting.\n");
			err = -ENODEV;
			kfree(drvdata);
			return err;
	}

	drvdata->client = client;
	i2c_set_clientdata(client, drvdata);

     /*get reset resource*/
#if 0
	led_rst = of_get_named_gpio(client->dev.of_node, "zte_led,gpio_reset", 0);
	if (!gpio_is_valid(led_rst)) {
		pr_info("RESET GPIO is invalid.\n");
	}
	rc = gpio_request(led_rst, "zte_led");
	if (rc) {
		dev_err(&client->dev, "Failed to request RESET GPIO. rc = %d\n", rc);

		gpio_free(led_rst);
		pr_info("remove reset_gpio success\n");
	}

	err = aw9523b_hw_init(drvdata);
#endif
	aw9523b_i2c_test(drvdata);
	atomic_set(&drvdata->enabled, 1);
	drvdata->mode = AW9523B_MODE_OFF;
	err = aw9523b_set_registers(drvdata);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);

	}

/*class node*/
	drvdata->cdev.brightness = LED_OFF;
	drvdata->cdev.name = "aw9523b_led";
	drvdata->cdev.brightness_set = aw9523b_brightness_set;
	drvdata->cdev.max_brightness = LED_FULL;
	rc = led_classdev_register(&client->dev, &drvdata->cdev);
	if (rc) {
		dev_err(&client->dev, "STATUS_LED: led_classdev_register failed\n");
		goto err_led_classdev_register_failed;
	}
	pr_info("led_classdev_register\n");
	aw9523b_blink_data = drvdata;
	wake_lock_init(&aw9523b_wake_lock, WAKE_LOCK_SUSPEND, "aw9523b-wakelock");
	wake_lock(&aw9523b_wake_lock);
/*device node*/
	rc = device_create_file(&client->dev, &dev_attr_enable);
	if (rc) {
		dev_err(&client->dev, "STATUS_LED: create dev_attr_enable failed\n");
		goto err_out_attr_enable;
	}

	rc = device_create_file(&client->dev, &dev_attr_mode);
	if (rc) {
		dev_err(&client->dev, "STATUS_LED: create dev_attr_mode failed\n");
		goto err_out_attr_mode;
	}
	rc = device_create_file(&client->dev, &dev_attr_number);
	if (rc) {
		dev_err(&client->dev, "STATUS_LED: create dev_attr_number failed\n");
		goto err_out_attr_number;
	}
	rc = device_create_file(&client->dev, &dev_attr_led_on_mesc);
	rc = device_create_file(&client->dev, &dev_attr_led_off_mesc);
	rc = device_create_file(&client->dev, &dev_attr_led_ftm_mode);

	INIT_WORK(&timerwork0, led_timer_handler0);
	INIT_WORK(&timerwork_ftm, led_timer_ftm_handler);
	return 0;

err_out_attr_enable:
	device_remove_file(&client->dev, &dev_attr_enable);
err_out_attr_mode:
	device_remove_file(&client->dev, &dev_attr_mode);
err_led_classdev_register_failed:
	led_classdev_unregister(&drvdata->cdev);
err_out_attr_number:
	device_remove_file(&client->dev, &dev_attr_number);

exit_check_functionality_failed:
	dev_err(&client->dev, "%s: Driver Init failed\n", "aw9523b");
	return err;
}

static int aw9523b_remove(struct i2c_client *client)
{
	struct aw9523b_data *drvdata = i2c_get_clientdata(client);
	int err;

	if (gpio_is_valid(drvdata->gpio_rst)) {
		gpio_free(drvdata->gpio_rst);
	}
	err = aw9523b_power_Off(drvdata);

	kfree(drvdata);

	return err;
}

static int aw9523b_resume(struct i2c_client *client)
{
	int err = 0;

	return err;
}

static int aw9523b_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int err = 0;
	return err;
}

static const struct i2c_device_id aw9523b_id[] = {
	{ "aw9523b", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, aw9523b_id);


static struct of_device_id aw9523b_match_table[] = {
	{ .compatible = "aw9523b", },
	{ },
};

static struct i2c_driver aw9523b_driver = {
	.driver = {
		.name = "aw9523b",
		.owner = THIS_MODULE,
		.of_match_table = aw9523b_match_table,
	},
	.probe     = aw9523b_probe,
	.remove  = aw9523b_remove,
	.resume = aw9523b_resume,
	.suspend = aw9523b_suspend,
	.id_table  = aw9523b_id,
};

static int __init aw9523b_init(void)
{
	led_timer_wq0 = alloc_workqueue("led-timer-handler0", WQ_UNBOUND | WQ_HIGHPRI, 1);
	led_timer_wq_ftm = alloc_workqueue("led-timer-ftm_handler", WQ_UNBOUND | WQ_HIGHPRI, 1);
	return i2c_add_driver(&aw9523b_driver);
}

static void __exit aw9523b_exit(void)
{
	return i2c_del_driver(&aw9523b_driver);
}

module_init(aw9523b_init);
module_exit(aw9523b_exit);

MODULE_DESCRIPTION("LED Class Interface");
MODULE_AUTHOR("Liukejing@zte.com.cn");
MODULE_LICENSE("GPL");

