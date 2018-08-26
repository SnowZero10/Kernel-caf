/*
 * STMicroelectronics lsm6ds3 driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * v 1.2.0
 * Licensed under the GPL-2.
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#include "lsm6ds3.h"
#include "lsm6ds3_core.h"
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h> /* lijiangshuo add for proc system */

/* The proper range for sensors calibration */
#define CAL_TIME		10
#define CAL_HZ			100
#define ACC_CAL_XY_RANGE_MIN		-2950 /* 180mg  //1.76m/s2 */
#define ACC_CAL_XY_RANGE_MAX		2950
#define ACC_CAL_Z_RANGE_MIN		13434
#define ACC_CAL_Z_RANGE_MAX		19334
#define GYRO_CAL_RANGE_MIN		-40000 /* 40dps // 0.7rad/s */
#define GYRO_CAL_RANGE_MAX		40000

/* COMMON VALUES FOR ACCEL-GYRO SENSORS */
#define LSM6DS3_WHO_AM_I			0x0f
#define LSM6DS3_WHO_AM_I_DEF			0x69
#define LSM6DSE_WHO_AM_I_DEF			0x6a
#define LSM6DS3_AXIS_EN_MASK			0x38
#define LSM6DS3_INT1_CTRL_ADDR			0x0d
#define LSM6DS3_INT2_CTRL_ADDR			0x0e
#define LSM6DS3_INT1_FULL			0x20
#define LSM6DS3_INT1_FTH			0x08
#define LSM6DS3_MD1_ADDR			0x5e
#define LSM6DS3_ODR_LIST_NUM			9
#define LSM6DS3_ODR_POWER_OFF_VAL		0x00
#define LSM6DS3_ODR_13HZ_VAL			0x01
#define LSM6DS3_ODR_26HZ_VAL			0x02
#define LSM6DS3_ODR_52HZ_VAL			0x03
#define LSM6DS3_ODR_104HZ_VAL			0x04
#define LSM6DS3_ODR_208HZ_VAL			0x05
#define LSM6DS3_ODR_416HZ_VAL			0x06
#define LSM6DS3_ODR_833HZ_VAL			0x07
#define LSM6DS3_ODR_1660HZ_VAL		0x08
#define LSM6DS3_ODR_3330HZ_VAL		0x09
#define LSM6DS3_ODR_6660HZ_VAL		0x0A
#define LSM6DS3_FS_LIST_NUM			4
#define LSM6DS3_BDU_ADDR			0x12
#define LSM6DS3_BDU_MASK			0x40
#define LSM6DS3_EN_BIT				0x01
#define LSM6DS3_DIS_BIT				0x00
#define LSM6DS3_FUNC_EN_ADDR			0x19
#define LSM6DS3_FUNC_EN_MASK			0x04
#define LSM6DS3_FUNC_CFG_ACCESS_ADDR		0x01
#define LSM6DS3_FUNC_CFG_ACCESS_MASK		0x01
#define LSM6DS3_FUNC_CFG_ACCESS_MASK2		0x04
#define LSM6DS3_FUNC_CFG_REG2_MASK		0x80
#define LSM6DS3_FUNC_CFG_START1_ADDR		0x62
#define LSM6DS3_FUNC_CFG_START2_ADDR		0x63
#define LSM6DS3_SELFTEST_ADDR			0x14
#define LSM6DS3_SELFTEST_ACCEL_MASK		0x03
#define LSM6DS3_SELFTEST_GYRO_MASK		0x0c
#define LSM6DS3_SELF_TEST_DISABLED_VAL		0x00
#define LSM6DS3_SELF_TEST_POS_SIGN_VAL		0x01
#define LSM6DS3_SELF_TEST_NEG_ACCEL_SIGN_VAL	0x02
#define LSM6DS3_SELF_TEST_NEG_GYRO_SIGN_VAL	0x03
#define LSM6DS3_LIR_ADDR			0x58
#define LSM6DS3_LIR_MASK			0x01
#define LSM6DS3_TIMER_EN_ADDR			0x58
#define LSM6DS3_TIMER_EN_MASK			0x80
#define LSM6DS3_PEDOMETER_EN_ADDR		0x58
#define LSM6DS3_PEDOMETER_EN_MASK		0x40
#define LSM6DS3_INT2_ON_INT1_ADDR		0x13
#define LSM6DS3_INT2_ON_INT1_MASK		0x20
#define LSM6DS3_MIN_DURATION_MS			1638
#define LSM6DS3_ROUNDING_ADDR			0x16
#define LSM6DS3_ROUNDING_MASK			0x04
#define LSM6DS3_FIFO_MODE_ADDR			0x0a
#define LSM6DS3_FIFO_MODE_MASK			0x07
#define LSM6DS3_FIFO_MODE_BYPASS		0x00
#define LSM6DS3_FIFO_MODE_CONTINUOS		0x06
#define LSM6DS3_FIFO_THRESHOLD_IRQ_MASK		0x08
#define LSM6DS3_FIFO_ODR_ADDR			0x0a
#define LSM6DS3_FIFO_ODR_MASK			0x78
#define LSM6DS3_FIFO_ODR_MAX			0x07
#define LSM6DS3_FIFO_ODR_MAX_HZ			800
#define LSM6DS3_FIFO_ODR_OFF			0x00
#define LSM6DS3_FIFO_CTRL3_ADDR			0x08
#define LSM6DS3_FIFO_ACCEL_DECIMATOR_MASK	0x07
#define LSM6DS3_FIFO_GYRO_DECIMATOR_MASK	0x38
#define LSM6DS3_FIFO_CTRL4_ADDR			0x09
#define LSM6DS3_FIFO_STEP_C_DECIMATOR_MASK	0x38
#define LSM6DS3_FIFO_THR_L_ADDR			0x06
#define LSM6DS3_FIFO_THR_H_ADDR			0x07
#define LSM6DS3_FIFO_THR_H_MASK			0x0f
#define LSM6DS3_FIFO_THR_IRQ_MASK		0x08
#define LSM6DS3_FIFO_PEDO_E_ADDR		0x07
#define LSM6DS3_FIFO_PEDO_E_MASK		0x80
#define LSM6DS3_FIFO_STEP_C_FREQ		25

/* CUSTOM VALUES FOR ACCEL SENSOR */
#define LSM6DS3_ACCEL_ODR_ADDR			0x10
#define LSM6DS3_ACCEL_ODR_MASK			0xf0
#define LSM6DS3_ACCEL_FS_ADDR			0x10
#define LSM6DS3_ACCEL_FS_MASK			0x0c
#define LSM6DS3_ACCEL_FS_2G_VAL			0x00
#define LSM6DS3_ACCEL_FS_4G_VAL			0x02
#define LSM6DS3_ACCEL_FS_8G_VAL			0x03
#define LSM6DS3_ACCEL_FS_16G_VAL		0x01
#define LSM6DS3_ACCEL_FS_2G_GAIN		61
#define LSM6DS3_ACCEL_FS_4G_GAIN		122
#define LSM6DS3_ACCEL_FS_8G_GAIN		244
#define LSM6DS3_ACCEL_FS_16G_GAIN		488
#define LSM6DS3_ACCEL_OUT_X_L_ADDR		0x28
#define LSM6DS3_ACCEL_OUT_Y_L_ADDR		0x2a
#define LSM6DS3_ACCEL_OUT_Z_L_ADDR		0x2c
#define LSM6DS3_ACCEL_AXIS_EN_ADDR		0x18
#define LSM6DS3_ACCEL_DRDY_IRQ_MASK		0x01
#define LSM6DS3_ACCEL_STD			1
#define LSM6DS3_ACCEL_STD_FROM_PD		2

/* CUSTOM VALUES FOR GYRO SENSOR */
#define LSM6DS3_GYRO_ODR_ADDR			0x11
#define LSM6DS3_GYRO_ODR_MASK			0xf0
#define LSM6DS3_GYRO_FS_ADDR			0x11
#define LSM6DS3_GYRO_FS_MASK			0x0c
#define LSM6DS3_GYRO_FS_245_VAL			0x00
#define LSM6DS3_GYRO_FS_500_VAL			0x01
#define LSM6DS3_GYRO_FS_1000_VAL		0x02
#define LSM6DS3_GYRO_FS_2000_VAL		0x03
#define LSM6DS3_GYRO_FS_245_GAIN		8750
#define LSM6DS3_GYRO_FS_500_GAIN		17500
#define LSM6DS3_GYRO_FS_1000_GAIN		35000
#define LSM6DS3_GYRO_FS_2000_GAIN		70000
#define LSM6DS3_GYRO_OUT_X_L_ADDR		0x22
#define LSM6DS3_GYRO_OUT_Y_L_ADDR		0x24
#define LSM6DS3_GYRO_OUT_Z_L_ADDR		0x26
#define LSM6DS3_GYRO_AXIS_EN_ADDR		0x19
#define LSM6DS3_GYRO_DRDY_IRQ_MASK		0x02
#define LSM6DS3_GYRO_STD					4
#define LSM6DS3_GYRO_STD_FROM_PD		1

#define LSM6DS3_OUT_XYZ_SIZE			8

/* CUSTOM VALUES FOR SIGNIFICANT MOTION SENSOR */
#define LSM6DS3_SIGN_MOTION_EN_ADDR		0x19
#define LSM6DS3_SIGN_MOTION_EN_MASK		0x01
#define LSM6DS3_SIGN_MOTION_DRDY_IRQ_MASK	0x40

/* CUSTOM VALUES FOR STEP DETECTOR SENSOR */
#define LSM6DS3_STEP_DETECTOR_DRDY_IRQ_MASK	0x80

/* CUSTOM VALUES FOR STEP COUNTER SENSOR */
#define LSM6DS3_STEP_COUNTER_DRDY_IRQ_MASK	0x80
#define LSM6DS3_STEP_COUNTER_OUT_L_ADDR		0x4b
#define LSM6DS3_STEP_COUNTER_OUT_SIZE		2
#define LSM6DS3_STEP_COUNTER_RES_ADDR		0x19
#define LSM6DS3_STEP_COUNTER_RES_MASK		0x06
#define LSM6DS3_STEP_COUNTER_RES_ALL_EN		0x03
#define LSM6DS3_STEP_COUNTER_RES_FUNC_EN	0x02
#define LSM6DS3_STEP_COUNTER_DURATION_ADDR	0x15
#define LSM6DS3_STEP_COUNTER_PEDO_THS_REG	0x0f
#define LSM6DS3_STEP_COUNTER_PEDO_DEB_REG	0x14

/* CUSTOM VALUES FOR TILT SENSOR */
#define LSM6DS3_TILT_EN_ADDR			0x58
#define LSM6DS3_TILT_EN_MASK			0x20
#define LSM6DS3_TILT_DRDY_IRQ_MASK		0x02

#define LSM6DS3_ENABLE_AXIS			0x07
#define LSM6DS3_FIFO_DIFF_L			0x3a
#define LSM6DS3_FIFO_DIFF_MASK			0x0fff
#define LSM6DS3_FIFO_DATA_OUT_L			0x3e
#define LSM6DS3_FIFO_ELEMENT_LEN_BYTE		6
#define LSM6DS3_FIFO_BYTE_FOR_CHANNEL		2
#define LSM6DS3_FIFO_DATA_OVR_2REGS		0x4000
#define LSM6DS3_FIFO_DATA_OVR			0x40

#define LSM6DS3_SRC_FUNC_ADDR			0x53
#define LSM6DS3_FIFO_DATA_AVL_ADDR		0x3b

#define LSM6DS3_SRC_SIGN_MOTION_DATA_AVL	0x40
#define LSM6DS3_SRC_STEP_DETECTOR_DATA_AVL	0x10
#define LSM6DS3_SRC_TILT_DATA_AVL		0x20
#define LSM6DS3_SRC_STEP_COUNTER_DATA_AVL	0x80
#define LSM6DS3_FIFO_DATA_AVL			0x80
#define LSM6DS3_RESET_ADDR			0x12
#define LSM6DS3_RESET_MASK			0x01

#define G_MAX		16000

static int change_freq = 0;

static const struct lsm6ds3_sensor_name {
	const char *name;
	const char *description;
} lsm6ds3_sensor_name[LSM6DS3_SENSORS_NUMB] = {
	[LSM6DS3_ACCEL] = {
		.name = "accel",
		.description = "lsm6ds3-accel",
	},
	[LSM6DS3_GYRO] = {
		.name = "gyro",
		.description = "lsm6ds3-gyro",
	},
	[LSM6DS3_SIGN_MOTION] = {
		.name = "sign_m",
		.description = "ST LSM6DS3 Significant Motion Sensor",
	},
	[LSM6DS3_STEP_COUNTER] = {
		.name = "step_c",
		.description = "lsm6ds3-stepcounter",
	},
	[LSM6DS3_STEP_DETECTOR] = {
		.name = "step_d",
		.description = "ST LSM6DS3 Step Detector Sensor",
	},
	[LSM6DS3_TILT] = {
		.name = "tilt",
		.description = "ST LSM6DS3 Tilt Sensor",
	},
};

struct lsm6ds3_odr_reg {
	u32 hz;
	u8 value;
};

static const struct lsm6ds3_odr_table {
	u8 addr[2];
	u8 mask[2];
	struct lsm6ds3_odr_reg odr_avl[10];
} lsm6ds3_odr_table = {
	.addr[LSM6DS3_ACCEL] = LSM6DS3_ACC_ODR_ADDR,
	.mask[LSM6DS3_ACCEL] = LSM6DS3_ACC_ODR_MASK,
	.addr[LSM6DS3_GYRO] = LSM6DS3_GYR_ODR_ADDR,
	.mask[LSM6DS3_GYRO] = LSM6DS3_GYR_ODR_MASK,
	.odr_avl[0] = { .hz = 26, .value = LSM6DS3_ODR_26HZ_VAL },
	.odr_avl[1] = { .hz = 52, .value = LSM6DS3_ODR_52HZ_VAL },
	.odr_avl[2] = { .hz = 104, .value = LSM6DS3_ODR_104HZ_VAL },
	.odr_avl[3] = { .hz = 208, .value = LSM6DS3_ODR_208HZ_VAL },
	.odr_avl[4] = { .hz = 416, .value = LSM6DS3_ODR_416HZ_VAL },
	.odr_avl[5] = { .hz = 833, .value = LSM6DS3_ODR_833HZ_VAL },
	.odr_avl[6] = { .hz = 1660, .value = LSM6DS3_ODR_1660HZ_VAL },
	.odr_avl[7] = { .hz = 3330, .value = LSM6DS3_ODR_3330HZ_VAL },
	.odr_avl[8] = { .hz = 6660, .value = LSM6DS3_ODR_6660HZ_VAL },
};

struct lsm6ds3_fs_reg {
	unsigned int gain;
	u8 value;
	int urv;
};

static struct lsm6ds3_fs_table {
	u8 addr;
	u8 mask;
	struct lsm6ds3_fs_reg fs_avl[LSM6DS3_FS_LIST_NUM];
} lsm6ds3_fs_table[LSM6DS3_SENSORS_NUMB] = {
	[LSM6DS3_ACCEL] = {
		.addr = LSM6DS3_ACCEL_FS_ADDR,
		.mask = LSM6DS3_ACCEL_FS_MASK,
		.fs_avl[0] = { .gain = LSM6DS3_ACCEL_FS_2G_GAIN,
					.value = LSM6DS3_ACCEL_FS_2G_VAL,
					.urv = 2, },
		.fs_avl[1] = { .gain = LSM6DS3_ACCEL_FS_4G_GAIN,
					.value = LSM6DS3_ACCEL_FS_4G_VAL,
					.urv = 4, },
		.fs_avl[2] = { .gain = LSM6DS3_ACCEL_FS_8G_GAIN,
					.value = LSM6DS3_ACCEL_FS_8G_VAL,
					.urv = 8, },
		.fs_avl[3] = { .gain = LSM6DS3_ACCEL_FS_16G_GAIN,
					.value = LSM6DS3_ACCEL_FS_16G_VAL,
					.urv = 16, },
	},
	[LSM6DS3_GYRO] = {
		.addr = LSM6DS3_GYRO_FS_ADDR,
		.mask = LSM6DS3_GYRO_FS_MASK,
		.fs_avl[0] = { .gain = LSM6DS3_GYRO_FS_245_GAIN,
					.value = LSM6DS3_GYRO_FS_245_VAL,
					.urv = 245, },
		.fs_avl[1] = { .gain = LSM6DS3_GYRO_FS_500_GAIN,
					.value = LSM6DS3_GYRO_FS_500_VAL,
					.urv = 500, },
		.fs_avl[2] = { .gain = LSM6DS3_GYRO_FS_1000_GAIN,
					.value = LSM6DS3_GYRO_FS_1000_VAL,
					.urv = 1000, },
		.fs_avl[3] = { .gain = LSM6DS3_GYRO_FS_2000_GAIN,
					.value = LSM6DS3_GYRO_FS_2000_VAL,
					.urv = 2000, },
	}
};

static struct sensors_classdev accel_cdev = {
	.name = "lsm6ds3-accel",
	.vendor = "ST Corporation",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "156.8",
	.resolution = "0.00781",
	.sensor_power = "0.13",
	.min_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_calibrate = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,
};

static struct sensors_classdev gyro_cdev = {
	.name = "lsm6ds3-gyro",
	.vendor = "ST Corporation",
	.version = 1,
	.handle = SENSORS_GYROSCOPE_HANDLE,
	.type = SENSOR_TYPE_GYROSCOPE,
	.max_range = "35",
	.resolution = "0.06",
	.sensor_power = "0.13",
	.min_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_calibrate = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,
};

static struct sensors_classdev stepcounter_cdev = {
	.name = "lsm6ds3-stepcounter",
	.vendor = "ST Corporation",
	.version = 1,
	.handle = SENSORS_STEP_COUNTER_HANDLE,
	.type = SENSOR_TYPE_STEP_COUNTER,
	.max_range = "35",
	.resolution = "0.06",
	.sensor_power = "0.13",
	.min_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_calibrate = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,
};

/* lijiangshuo add for proc file start */
static int chip_id = 0xff;
static struct proc_dir_entry *lsm6ds3_info_proc_file_accel;
static struct proc_dir_entry *lsm6ds3_info_proc_file_gyro;
/* lijiangshuo add for proc file end*/

static struct workqueue_struct *lsm6ds3_workqueue = 0;

static inline void lsm6ds3_flush_works(void)
{
	flush_workqueue(lsm6ds3_workqueue);
}

#if 0
static inline int64_t lsm6ds3_get_time_ns(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);
	return timespec_to_ns(&ts);
}
#endif

static int lsm6ds3_write_data_with_mask(struct lsm6ds3_data *cdata,
				u8 reg_addr, u8 mask, u8 data, bool b_lock)
{
	int err;
	u8 new_data = 0x00, old_data = 0x00;

	err = cdata->tf->read(cdata, reg_addr, 1, &old_data, b_lock);
	if (err < 0)
		return err;

	new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));

	if (new_data == old_data)
		return 1;

	return cdata->tf->write(cdata, reg_addr, 1, &new_data, b_lock);
}

static int lsm6ds3_input_init(struct lsm6ds3_sensor_data *sdata, u16 bustype,
							const char *description)
{
	int err = 0;

	sdata->input_dev = input_allocate_device();
	if (!sdata->input_dev) {
		dev_err(sdata->cdata->dev, "failed to allocate input device");
		return -ENOMEM;
	}

	sdata->input_dev->name = lsm6ds3_sensor_name[sdata->sindex].description;

	sdata->input_dev->id.bustype = bustype;
	sdata->input_dev->dev.parent = sdata->cdata->dev;
	sdata->input_dev->name = description;
	input_set_drvdata(sdata->input_dev, sdata);

	if (sdata->sindex == LSM6DS3_ACCEL) {
		set_bit(EV_ABS, sdata->input_dev->evbit);
		input_set_abs_params(sdata->input_dev, ABS_X, -G_MAX, G_MAX, 0, 0);
		input_set_abs_params(sdata->input_dev, ABS_Y, -G_MAX, G_MAX, 0, 0);
		input_set_abs_params(sdata->input_dev, ABS_Z, -G_MAX, G_MAX, 0, 0);
	} else if (sdata->sindex == LSM6DS3_GYRO) {
		set_bit(EV_ABS, sdata->input_dev->evbit);
		input_set_abs_params(sdata->input_dev, ABS_RX, -G_MAX, G_MAX, 0, 0);
		input_set_abs_params(sdata->input_dev, ABS_RY, -G_MAX, G_MAX, 0, 0);
		input_set_abs_params(sdata->input_dev, ABS_RZ, -G_MAX, G_MAX, 0, 0);
	} else if (sdata->sindex == LSM6DS3_STEP_COUNTER) {
		set_bit(EV_ABS, sdata->input_dev->evbit);
		input_set_abs_params(sdata->input_dev, ABS_MISC, 0, 65535, 0, 0);
	} else {
		__set_bit(INPUT_EVENT_TYPE, sdata->input_dev->evbit);
		__set_bit(INPUT_EVENT_TIME_MSB, sdata->input_dev->mscbit);
		__set_bit(INPUT_EVENT_TIME_LSB, sdata->input_dev->mscbit);
		__set_bit(INPUT_EVENT_X, sdata->input_dev->mscbit);
	}

	err = input_register_device(sdata->input_dev);
	if (err) {
		dev_err(sdata->cdata->dev, "unable to register sensor %s\n",
								sdata->name);
		input_free_device(sdata->input_dev);
	}

	return err;
}

static void lsm6ds3_input_cleanup(struct lsm6ds3_sensor_data *sdata)
{
	input_unregister_device(sdata->input_dev);
	input_free_device(sdata->input_dev);
}

static void lsm6ds3_report_3axes_event(struct lsm6ds3_sensor_data *sdata,
							s32 *xyz, ktime_t timestamp)
{
	struct input_dev  *input = sdata->input_dev;

	if (!sdata->enabled)
		return;

	switch (sdata->sindex) {
	case LSM6DS3_ACCEL:
		input_event(input, EV_ABS, ABS_X, xyz[0]);
		input_event(input, EV_ABS, ABS_Y, xyz[1]);
		input_event(input, EV_ABS, ABS_Z, xyz[2]);
		input_event(input, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
		input_event(input, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
		input_sync(input);
		break;
	case LSM6DS3_GYRO:
		input_event(input, EV_ABS, ABS_RX, xyz[0]);
		input_event(input, EV_ABS, ABS_RY, xyz[1]);
		input_event(input, EV_ABS, ABS_RZ, xyz[2]);
		input_event(input, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
		input_event(input, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
		input_sync(input);
		break;
	default:
		input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
		input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
		input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
		input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
								ktime_to_timespec(timestamp).tv_sec);
		input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
								ktime_to_timespec(timestamp).tv_nsec);
		input_sync(input);
		break;
	}
}

static void lsm6ds3_report_single_event(struct lsm6ds3_sensor_data *sdata,
							s32 data, ktime_t timestamp)
{
	struct input_dev  *input = sdata->input_dev;

	if (!sdata->enabled)
		return;

	if (sdata->sindex == LSM6DS3_STEP_COUNTER) {
		input_event(input, EV_ABS, ABS_MISC, data);
		/* input_event(input, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec); */
		/* input_event(input, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec); */
		input_sync(input);
	} else {
		input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_X, data);
		input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
								ktime_to_timespec(timestamp).tv_sec);
		input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
								ktime_to_timespec(timestamp).tv_nsec);
		input_sync(input);
	}
}

enum hrtimer_restart lsm6ds3_poll_function_read(struct hrtimer *timer)
{
	struct lsm6ds3_sensor_data *sdata;

	sdata = container_of((struct hrtimer *)timer, struct lsm6ds3_sensor_data,
							hr_timer);

	sdata->timestamp = ktime_get_boottime();
	queue_work(lsm6ds3_workqueue, &sdata->input_work);

	return HRTIMER_NORESTART;
}

static int lsm6ds3_get_step_c_data(struct lsm6ds3_sensor_data *sdata, u16 *steps)
{
	u8 data[2];
	int err = 0;

	err = sdata->cdata->tf->read(sdata->cdata,
					LSM6DS3_STEP_COUNTER_OUT_L_ADDR,
					LSM6DS3_STEP_COUNTER_OUT_SIZE,
					data, true);
	if (err < 0)
		return err;

	*steps = data[0] | (data[1] << 8);

	return 0;
}

static int lsm6ds3_get_poll_data(struct lsm6ds3_sensor_data *sdata, u8 *data)
{
	int err = 0;
	u8 reg_addr;

	switch (sdata->sindex) {
	case LSM6DS3_ACCEL:
		reg_addr = LSM6DS3_ACCEL_OUT_X_L_ADDR;
		break;
	case LSM6DS3_GYRO:
		reg_addr = LSM6DS3_GYRO_OUT_X_L_ADDR;
		break;
	default:
		dev_err(sdata->cdata->dev, "invalid polling mode for sensor %s\n",
								sdata->name);
		err = -1;
		return err;
	}

	err = sdata->cdata->tf->read(sdata->cdata, reg_addr, LSM6DS3_OUT_XYZ_SIZE,
								data, true);

	return err;
}

static void step_counter_poll_function_work(struct work_struct *input_work)
{
	struct lsm6ds3_sensor_data *sdata;
	struct lsm6ds3_data *cdata;
	int err = 0;
	u16 steps_c;

	sdata = container_of((struct work_struct *)input_work, struct lsm6ds3_sensor_data, input_work);
	cdata = sdata->cdata;

	err = lsm6ds3_get_step_c_data(sdata, &steps_c);
	if (err < 0) {
		dev_err(cdata->dev, "error while reading step counter data\n");
		hrtimer_start(&sdata->hr_timer, sdata->ktime, HRTIMER_MODE_REL);
		return;
	}

	lsm6ds3_report_single_event(sdata, steps_c, sdata->timestamp);
	cdata->steps_c = steps_c;

	hrtimer_start(&sdata->hr_timer, sdata->ktime, HRTIMER_MODE_REL);

	pr_info("lsm6ds3 %s steps_c=%d\n", __func__, steps_c);
}

static void poll_function_work(struct work_struct *input_work)
{
	struct lsm6ds3_sensor_data *sdata;
	int xyz[3] = { 0 }, xyz_final[3] = { 0 };
	u8 data[6];
	int err;
	unsigned int set_ms;

	sdata = container_of((struct work_struct *)input_work,
			struct lsm6ds3_sensor_data, input_work);

	hrtimer_start(&sdata->hr_timer, sdata->ktime, HRTIMER_MODE_REL);
	if (sdata->sample_to_discard) {
		sdata->sample_to_discard--;
		return;
	}

	err = lsm6ds3_get_poll_data(sdata, data);
	if (err < 0)
		dev_err(sdata->cdata->dev, "get %s data failed %d\n",
								sdata->name, err);
	else {
		xyz[0] = (s32)((s16)(data[0] | (data[1] << 8)));
		xyz[1] = (s32)((s16)(data[2] | (data[3] << 8)));
		xyz[2] = (s32)((s16)(data[4] | (data[5] << 8)));

		if (sdata->sindex == LSM6DS3_ACCEL) {
			xyz[0] *= sdata->sensitivity;
			xyz[1] *= sdata->sensitivity;
			xyz[2] *= sdata->sensitivity;
		}

		if (sdata->sindex == LSM6DS3_GYRO) {
			xyz[0] *= sdata->c_gain;
			xyz[1] *= sdata->c_gain;
			xyz[2] *= sdata->c_gain;
			if (chip_id ==  LSM6DSE_WHO_AM_I_DEF) {
				xyz[0] /= 10000;
				xyz[1] /= 10000;
				xyz[2] /= 10000;
			} else {
				xyz[0] /= 1000;
				xyz[1] /= 1000;
				xyz[2] /= 1000;
			}
		}

#ifdef CONFIG_BOARD_ELIN
		if (sdata->sindex == LSM6DS3_ACCEL) {
			xyz_final[0] = sdata->cdata->negate_x ?
				-xyz[sdata->cdata->axis_map_x] : xyz[sdata->cdata->axis_map_x];
			xyz_final[1] = sdata->cdata->negate_y ?
				-xyz[sdata->cdata->axis_map_y] : xyz[sdata->cdata->axis_map_y];
			xyz_final[2] = sdata->cdata->negate_z ?
				-xyz[sdata->cdata->axis_map_z] : xyz[sdata->cdata->axis_map_z];
		} else if (sdata->sindex == LSM6DS3_GYRO) {
			xyz_final[0] = sdata->cdata->negate_x ?
				xyz[sdata->cdata->axis_map_x] : -xyz[sdata->cdata->axis_map_x];
			xyz_final[1] = sdata->cdata->negate_y ?
				-xyz[sdata->cdata->axis_map_y] : xyz[sdata->cdata->axis_map_y];
			xyz_final[2] = sdata->cdata->negate_z ?
				xyz[sdata->cdata->axis_map_z] : -xyz[sdata->cdata->axis_map_z];
		}
#else
		xyz_final[0] = sdata->cdata->negate_x ? -xyz[sdata->cdata->axis_map_x] : xyz[sdata->cdata->axis_map_x];
		xyz_final[1] = sdata->cdata->negate_y ? -xyz[sdata->cdata->axis_map_y] : xyz[sdata->cdata->axis_map_y];
		xyz_final[2] = sdata->cdata->negate_z ? -xyz[sdata->cdata->axis_map_z] : xyz[sdata->cdata->axis_map_z];
#endif
		xyz_final[0] -= sdata->cal_x;
		xyz_final[1] -= sdata->cal_y;
		xyz_final[2] -= sdata->cal_z;

		if (sdata->cdata->delay_time == 1000 && change_freq == 0) {
			dev_err(sdata->cdata->dev, "change gyro freq\n");
			set_ms = 500;
			sdata->cdata->sensors[LSM6DS3_GYRO].ktime = ktime_set(0, MS_TO_NS(set_ms));
			change_freq = 1;
		}

		sdata->timestamp = ktime_get_boottime();
		lsm6ds3_report_3axes_event(sdata, xyz_final, sdata->timestamp);
	}
}

int lsm6ds3_set_drdy_irq(struct lsm6ds3_sensor_data *sdata, bool state)
{
	u8 reg_addr, mask, value;

	if (state)
		value = LSM6DS3_EN_BIT;
	else
		value = LSM6DS3_DIS_BIT;

	switch (sdata->sindex) {
	case LSM6DS3_ACCEL:
	case LSM6DS3_GYRO:
	case LSM6DS3_STEP_COUNTER:
		return 0;

	case LSM6DS3_SIGN_MOTION:
	case LSM6DS3_STEP_DETECTOR:
		if ((sdata->cdata->sensors[LSM6DS3_STEP_DETECTOR].enabled) ||
				(sdata->cdata->sensors[LSM6DS3_SIGN_MOTION].enabled))
			return 0;

		reg_addr = LSM6DS3_INT1_CTRL_ADDR;
		mask = LSM6DS3_STEP_DETECTOR_DRDY_IRQ_MASK;
		break;
	case LSM6DS3_TILT:
		reg_addr = LSM6DS3_MD1_ADDR;
		mask = LSM6DS3_TILT_DRDY_IRQ_MASK;
		break;
	default:
		return -EINVAL;
	}

	return lsm6ds3_write_data_with_mask(sdata->cdata, reg_addr, mask, value,
									true);
}

static int lsm6ds3_set_fs(struct lsm6ds3_sensor_data *sdata, u32 gain)
{
	int err, i;

	for (i = 0; i < LSM6DS3_FS_LIST_NUM; i++) {
		if (lsm6ds3_fs_table[sdata->sindex].fs_avl[i].gain == gain)
			break;
	}

	if (i == LSM6DS3_FS_LIST_NUM)
		return -EINVAL;

	switch (i) {
	case 0:
		sdata->sensitivity = 1;
		break;
	case 1:
		sdata->sensitivity = 2;
		break;
	case 2:
		sdata->sensitivity = 4;
		break;
	case 3:
		sdata->sensitivity = 8;
		break;
	}

	err = lsm6ds3_write_data_with_mask(sdata->cdata,
				lsm6ds3_fs_table[sdata->sindex].addr,
				lsm6ds3_fs_table[sdata->sindex].mask,
				lsm6ds3_fs_table[sdata->sindex].fs_avl[i].value,
				true);
	if (err < 0)
		return err;

	sdata->c_gain = gain;

	return 0;
}

irqreturn_t lsm6ds3_save_timestamp(int irq, void *private)
{
	struct lsm6ds3_data *cdata = (struct lsm6ds3_data *)private;

	cdata->timestamp = ktime_get_boottime();
	queue_work(lsm6ds3_workqueue, &cdata->input_work);

	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

static int lsm6ds3_disable_sensors(struct lsm6ds3_sensor_data *sdata);

static void lsm6ds3_irq_management(struct work_struct *input_work)
{
	struct lsm6ds3_data *cdata;
	u8 src_value = 0x00, src_fifo = 0x00;
	struct lsm6ds3_sensor_data *sdata;
	u16 steps_c;
	int err;

	cdata = container_of((struct work_struct *)input_work,
						struct lsm6ds3_data, input_work);

	cdata->tf->read(cdata, LSM6DS3_SRC_FUNC_ADDR, 1, &src_value, true);
	cdata->tf->read(cdata, LSM6DS3_FIFO_DATA_AVL_ADDR, 1, &src_fifo, true);

	if (src_value & LSM6DS3_SRC_STEP_COUNTER_DATA_AVL) {
		sdata = &cdata->sensors[LSM6DS3_STEP_COUNTER];
		sdata->timestamp = cdata->timestamp;
		err = lsm6ds3_get_step_c_data(sdata, &steps_c);
		if (err < 0) {
			dev_err(cdata->dev,
				"error while reading step counter data\n");
			enable_irq(cdata->irq);

			return;
		}

		lsm6ds3_report_single_event(&cdata->sensors[LSM6DS3_STEP_COUNTER],
						steps_c,
						cdata->sensors[LSM6DS3_STEP_COUNTER].timestamp);
		cdata->steps_c = steps_c;
	}

	if (src_value & LSM6DS3_SRC_STEP_DETECTOR_DATA_AVL) {
		sdata = &cdata->sensors[LSM6DS3_STEP_DETECTOR];
		sdata->timestamp = cdata->timestamp;
		lsm6ds3_report_single_event(sdata, 1, sdata->timestamp);

		if (cdata->sign_motion_event_ready) {
			sdata = &cdata->sensors[LSM6DS3_SIGN_MOTION];
			sdata->timestamp = cdata->timestamp;
			lsm6ds3_report_single_event(sdata, 1, sdata->timestamp);
			cdata->sign_motion_event_ready = false;
			lsm6ds3_disable_sensors(sdata);
		}
	}

	if (src_value & LSM6DS3_SRC_TILT_DATA_AVL) {
		sdata = &cdata->sensors[LSM6DS3_TILT];
		sdata->timestamp = cdata->timestamp;
		lsm6ds3_report_single_event(sdata, 1, sdata->timestamp);
	}

	enable_irq(cdata->irq);
}

int lsm6ds3_allocate_workqueue(struct lsm6ds3_data *cdata)
{
	int err;

	if (!lsm6ds3_workqueue)
		lsm6ds3_workqueue = create_workqueue(cdata->name);

	if (!lsm6ds3_workqueue)
		return -EINVAL;

	INIT_WORK(&cdata->input_work, lsm6ds3_irq_management);

	err = request_threaded_irq(cdata->irq, lsm6ds3_save_timestamp, NULL,
			IRQF_TRIGGER_HIGH, cdata->name, cdata);
	if (err)
		return err;

	return 0;
}

static int lsm6ds3_set_extra_dependency(struct lsm6ds3_sensor_data *sdata,
								bool enable)
{
	int err;

	if (!(sdata->cdata->sensors[LSM6DS3_SIGN_MOTION].enabled |
			sdata->cdata->sensors[LSM6DS3_STEP_COUNTER].enabled |
			sdata->cdata->sensors[LSM6DS3_STEP_DETECTOR].enabled |
			sdata->cdata->sensors[LSM6DS3_TILT].enabled)) {
		if (enable) {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
						LSM6DS3_FUNC_EN_ADDR,
						LSM6DS3_FUNC_EN_MASK,
						LSM6DS3_EN_BIT, true);
			if (err < 0)
				return err;
		} else {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
						LSM6DS3_FUNC_EN_ADDR,
						LSM6DS3_FUNC_EN_MASK,
						LSM6DS3_DIS_BIT, true);
			if (err < 0)
				return err;
		}
	}

	if (!sdata->cdata->sensors[LSM6DS3_ACCEL].enabled) {
		if (enable) {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
				lsm6ds3_odr_table.addr[LSM6DS3_ACCEL],
				lsm6ds3_odr_table.mask[LSM6DS3_ACCEL],
				lsm6ds3_odr_table.odr_avl[0].value, true);
			if (err < 0)
				return err;
		} else {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
				lsm6ds3_odr_table.addr[LSM6DS3_ACCEL],
				lsm6ds3_odr_table.mask[LSM6DS3_ACCEL],
				LSM6DS3_ODR_POWER_OFF_VAL, true);
			if (err < 0)
				return err;
		}
	}

	return 0;
}

static int lsm6ds3_enable_pedometer(struct lsm6ds3_sensor_data *sdata,
								bool enable)
{
	int err = 0;
	u8 value = LSM6DS3_DIS_BIT;

	pr_info("lsm6ds3 %s\n", __func__);

	if (sdata->cdata->sensors[LSM6DS3_STEP_COUNTER].enabled &&
			sdata->cdata->sensors[LSM6DS3_STEP_DETECTOR].enabled)
		return 0;

	if (enable)
		value = LSM6DS3_EN_BIT;

	err = lsm6ds3_write_data_with_mask(sdata->cdata,
						LSM6DS3_FIFO_PEDO_E_ADDR,
						LSM6DS3_FIFO_PEDO_E_MASK,
						value, true);
	if (err < 0)
		return err;

	return lsm6ds3_write_data_with_mask(sdata->cdata,
						LSM6DS3_PEDOMETER_EN_ADDR,
						LSM6DS3_PEDOMETER_EN_MASK,
						value, true);

}

static int lsm6ds3_enable_sensors(struct lsm6ds3_sensor_data *sdata)
{
	int err, i;

	pr_info("ljs %s index=%d\n", __func__, sdata->sindex);

	if (sdata->enabled)
		return 0;

	switch (sdata->sindex) {
	case LSM6DS3_ACCEL:
	case LSM6DS3_GYRO:
		for (i = 0; i < LSM6DS3_ODR_LIST_NUM; i++) {
			if (lsm6ds3_odr_table.odr_avl[i].hz == sdata->c_odr)
				break;
		}
		if (i == LSM6DS3_ODR_LIST_NUM)
			return -EINVAL;

		if (sdata->sindex == LSM6DS3_ACCEL)
			sdata->sample_to_discard = LSM6DS3_ACCEL_STD +
						LSM6DS3_ACCEL_STD_FROM_PD;

		sdata->cdata->sensors[LSM6DS3_GYRO].sample_to_discard =
						LSM6DS3_GYRO_STD +
						LSM6DS3_GYRO_STD_FROM_PD;

		err = lsm6ds3_write_data_with_mask(sdata->cdata,
				lsm6ds3_odr_table.addr[sdata->sindex],
				lsm6ds3_odr_table.mask[sdata->sindex],
				lsm6ds3_odr_table.odr_avl[i].value, true);
		if (err < 0)
			return err;

		hrtimer_start(&sdata->hr_timer, sdata->ktime, HRTIMER_MODE_REL);
		sdata->c_odr = lsm6ds3_odr_table.odr_avl[i].hz;

		break;
	case LSM6DS3_SIGN_MOTION:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
						LSM6DS3_SIGN_MOTION_EN_ADDR,
						LSM6DS3_SIGN_MOTION_EN_MASK,
						LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

		if ((sdata->cdata->sensors[LSM6DS3_STEP_COUNTER].enabled) ||
				(sdata->cdata->sensors[LSM6DS3_STEP_DETECTOR].enabled)) {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
						LSM6DS3_PEDOMETER_EN_ADDR,
						LSM6DS3_PEDOMETER_EN_MASK,
						LSM6DS3_DIS_BIT, true);
			if (err < 0)
				return err;

			err = lsm6ds3_write_data_with_mask(sdata->cdata,
						LSM6DS3_PEDOMETER_EN_ADDR,
						LSM6DS3_PEDOMETER_EN_MASK,
						LSM6DS3_EN_BIT, true);
			if (err < 0)
				return err;
		} else {
			err = lsm6ds3_enable_pedometer(sdata, true);
			if (err < 0)
				return err;
		}

		sdata->cdata->sign_motion_event_ready = true;

		break;
	case LSM6DS3_STEP_COUNTER:
	case LSM6DS3_STEP_DETECTOR:
		err = lsm6ds3_enable_pedometer(sdata, true);
		if (err < 0)
			return err;

		hrtimer_start(&sdata->hr_timer, sdata->ktime, HRTIMER_MODE_REL);

		break;
	case LSM6DS3_TILT:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
					LSM6DS3_TILT_EN_ADDR,
					LSM6DS3_TILT_EN_MASK,
					LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

		break;
	default:
		return -EINVAL;
	}

	err = lsm6ds3_set_extra_dependency(sdata, true);
	if (err < 0)
		return err;


	err = lsm6ds3_set_drdy_irq(sdata, true);
	if (err < 0)
		return err;

	sdata->enabled = true;
	pr_info("ljs %s out. index=%d\n", __func__, sdata->sindex);

	return 0;
}

static int lsm6ds3_disable_sensors(struct lsm6ds3_sensor_data *sdata)
{
	int err;

	if (!sdata->enabled)
		return 0;

	switch (sdata->sindex) {
	case LSM6DS3_ACCEL:
		if (sdata->cdata->sensors[LSM6DS3_SIGN_MOTION].enabled |
			sdata->cdata->sensors[LSM6DS3_STEP_COUNTER].enabled |
			sdata->cdata->sensors[LSM6DS3_STEP_DETECTOR].enabled |
			sdata->cdata->sensors[LSM6DS3_TILT].enabled) {

			err = lsm6ds3_write_data_with_mask(sdata->cdata,
				lsm6ds3_odr_table.addr[LSM6DS3_ACCEL],
				lsm6ds3_odr_table.mask[LSM6DS3_ACCEL],
				lsm6ds3_odr_table.odr_avl[0].value, true);
		} else {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
				lsm6ds3_odr_table.addr[LSM6DS3_ACCEL],
				lsm6ds3_odr_table.mask[LSM6DS3_ACCEL],
				LSM6DS3_ODR_POWER_OFF_VAL, true);
		}
		if (err < 0)
			return err;

		cancel_work_sync(&sdata->input_work);
		hrtimer_cancel(&sdata->hr_timer);

		break;
	case LSM6DS3_GYRO:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
				lsm6ds3_odr_table.addr[LSM6DS3_GYRO],
				lsm6ds3_odr_table.mask[LSM6DS3_GYRO],
				LSM6DS3_ODR_POWER_OFF_VAL, true);
		if (err < 0)
			return err;

		cancel_work_sync(&sdata->input_work);
		hrtimer_cancel(&sdata->hr_timer);
		if (change_freq == 1) {
			pr_info("%s reset gyro freq\n", __func__);
			change_freq = 0;
		}

		break;
	case LSM6DS3_SIGN_MOTION:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
				LSM6DS3_SIGN_MOTION_EN_ADDR,
				LSM6DS3_SIGN_MOTION_EN_MASK,
				LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;

		err = lsm6ds3_enable_pedometer(sdata, false);
		if (err < 0)
			return err;

		sdata->cdata->sign_motion_event_ready = false;

		break;
	case LSM6DS3_STEP_COUNTER:
	case LSM6DS3_STEP_DETECTOR:
		err = lsm6ds3_enable_pedometer(sdata, false);
		if (err < 0)
			return err;

		cancel_work_sync(&sdata->input_work);
		hrtimer_cancel(&sdata->hr_timer);
		break;
	case LSM6DS3_TILT:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
				LSM6DS3_TILT_EN_ADDR,
				LSM6DS3_TILT_EN_MASK,
				LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;

		break;
	default:
		return -EINVAL;
	}

	err = lsm6ds3_set_extra_dependency(sdata, false);
	if (err < 0)
		return err;

	err = lsm6ds3_set_drdy_irq(sdata, false);
	if (err < 0)
		return err;

	sdata->enabled = false;
	pr_info("ljs %s out. index=%d\n", __func__, sdata->sindex);

	return 0;
}

static int lsm6ds3_reset_steps(struct lsm6ds3_data *cdata)
{
	int err;
	u8 reg_value = 0x00;

	err = cdata->tf->read(cdata,
			LSM6DS3_STEP_COUNTER_RES_ADDR, 1, &reg_value, true);
	if (err < 0)
		return err;

	if (reg_value & LSM6DS3_FUNC_EN_MASK)
		reg_value = LSM6DS3_STEP_COUNTER_RES_FUNC_EN;
	else
		reg_value = LSM6DS3_DIS_BIT;

	err = lsm6ds3_write_data_with_mask(cdata,
				LSM6DS3_STEP_COUNTER_RES_ADDR,
				LSM6DS3_STEP_COUNTER_RES_MASK,
				LSM6DS3_STEP_COUNTER_RES_ALL_EN, true);
	if (err < 0)
		return err;

	err = lsm6ds3_write_data_with_mask(cdata,
				LSM6DS3_STEP_COUNTER_RES_ADDR,
				LSM6DS3_STEP_COUNTER_RES_MASK,
				reg_value, true);
	if (err < 0)
		return err;

	cdata->reset_steps = true;

	return 0;
}

static int lsm6ds3_init_sensors(struct lsm6ds3_data *cdata)
{
	int err, i;
	u8 default_reg_value = 0;
	u8 pedo_ths_reg = 0x94; /* threshold: 20*32mg = 640mg; 4G-Scale */
	u8 pedo_deb_reg = 0x5f; /* debounce time: 11*80ms = 880ms; debounce step = 7 */
	u8 reg_value = 0xff;
	struct lsm6ds3_sensor_data *sdata;

	mutex_init(&cdata->tb.buf_lock);

	for (i = 0; i < LSM6DS3_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];

		err = lsm6ds3_disable_sensors(sdata);
		if (err < 0)
			return err;
	}

	hrtimer_init(&cdata->sensors[LSM6DS3_ACCEL].hr_timer, CLOCK_MONOTONIC,
						HRTIMER_MODE_REL);
	hrtimer_init(&cdata->sensors[LSM6DS3_GYRO].hr_timer, CLOCK_MONOTONIC,
						HRTIMER_MODE_REL);
	cdata->sensors[LSM6DS3_ACCEL].hr_timer.function =
						&lsm6ds3_poll_function_read;
	cdata->sensors[LSM6DS3_GYRO].hr_timer.function =
						&lsm6ds3_poll_function_read;

	hrtimer_init(&cdata->sensors[LSM6DS3_STEP_COUNTER].hr_timer, CLOCK_MONOTONIC,
						HRTIMER_MODE_REL);
	cdata->sensors[LSM6DS3_STEP_COUNTER].hr_timer.function =
						&lsm6ds3_poll_function_read;
	cdata->steps_c = 0;
	cdata->reset_steps = false;

	err = lsm6ds3_write_data_with_mask(cdata, LSM6DS3_RESET_ADDR,
				LSM6DS3_RESET_MASK, LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_LIR_ADDR,
					LSM6DS3_LIR_MASK,
					LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_TIMER_EN_ADDR,
					LSM6DS3_TIMER_EN_MASK,
					LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_BDU_ADDR,
					LSM6DS3_BDU_MASK,
					LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_ROUNDING_ADDR,
					LSM6DS3_ROUNDING_MASK,
					LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_INT2_ON_INT1_ADDR,
					LSM6DS3_INT2_ON_INT1_MASK,
					LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = lsm6ds3_reset_steps(cdata);
	if (err < 0)
		return err;

	mutex_lock(&cdata->bank_registers_lock);
	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					LSM6DS3_FUNC_CFG_REG2_MASK,
					LSM6DS3_EN_BIT, false);
	if (err < 0)
		goto lsm6ds3_init_sensor_mutex_unlock;

	err = cdata->tf->write(cdata,
					LSM6DS3_STEP_COUNTER_DURATION_ADDR,
					1,
					&default_reg_value, false);
	if (err < 0)
		goto lsm6ds3_init_sensor_mutex_unlock;

	err = cdata->tf->write(cdata,
					LSM6DS3_STEP_COUNTER_PEDO_THS_REG,
					1,
					&pedo_ths_reg, false);
	if (err < 0)
		goto lsm6ds3_init_sensor_mutex_unlock;

	err = cdata->tf->write(cdata,
					LSM6DS3_STEP_COUNTER_PEDO_DEB_REG,
					1,
					&pedo_deb_reg, false);
	if (err < 0)
		goto lsm6ds3_init_sensor_mutex_unlock;

	cdata->tf->read(cdata, LSM6DS3_STEP_COUNTER_PEDO_THS_REG, 1, &reg_value, false);
	pr_info("%s pedo_ths_reg 0x%x(addr 0x%x)\n", __func__, reg_value, LSM6DS3_STEP_COUNTER_PEDO_THS_REG);

	cdata->tf->read(cdata, LSM6DS3_STEP_COUNTER_DURATION_ADDR, 1, &reg_value, false);
	pr_info("%s duration_reg 0x%x(addr 0x%x)\n", __func__, reg_value, LSM6DS3_STEP_COUNTER_DURATION_ADDR);

	cdata->tf->read(cdata, LSM6DS3_STEP_COUNTER_PEDO_DEB_REG, 1, &reg_value, false);
	pr_info("%s pedo_deb_reg 0x%x(addr 0x%x)\n", __func__, reg_value, LSM6DS3_STEP_COUNTER_PEDO_DEB_REG);


	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					LSM6DS3_FUNC_CFG_REG2_MASK,
					LSM6DS3_DIS_BIT, false);
	if (err < 0)
		goto lsm6ds3_init_sensor_mutex_unlock;

	mutex_unlock(&cdata->bank_registers_lock);

	err = lsm6ds3_set_fs(&cdata->sensors[LSM6DS3_ACCEL], cdata->sensors[LSM6DS3_ACCEL].c_gain);
	if (err < 0)
		return err;

	err = lsm6ds3_set_fs(&cdata->sensors[LSM6DS3_GYRO], cdata->sensors[LSM6DS3_GYRO].c_gain);
	if (err < 0)
		return err;

	cdata->sensors[LSM6DS3_ACCEL].ktime = ktime_set(0,
			MS_TO_NS(cdata->sensors[LSM6DS3_ACCEL].poll_interval));
	cdata->sensors[LSM6DS3_GYRO].ktime = ktime_set(0,
			MS_TO_NS(cdata->sensors[LSM6DS3_GYRO].poll_interval));
	INIT_WORK(&cdata->sensors[LSM6DS3_ACCEL].input_work, poll_function_work);
	INIT_WORK(&cdata->sensors[LSM6DS3_GYRO].input_work, poll_function_work);

	cdata->sensors[LSM6DS3_STEP_COUNTER].ktime = ktime_set(0,
			MS_TO_NS(cdata->sensors[LSM6DS3_STEP_COUNTER].poll_interval));
	INIT_WORK(&cdata->sensors[LSM6DS3_STEP_COUNTER].input_work, step_counter_poll_function_work);

	return 0;

lsm6ds3_init_sensor_mutex_unlock:
	mutex_unlock(&cdata->bank_registers_lock);
	return err;
}

static int lsm6ds3_set_odr(struct lsm6ds3_sensor_data *sdata, u32 odr)
{
	int err = 0, i;

	for (i = 0; i < LSM6DS3_ODR_LIST_NUM; i++) {
		if (lsm6ds3_odr_table.odr_avl[i].hz >= odr)
			break;
	}
	if (i == LSM6DS3_ODR_LIST_NUM) {
		pr_info("%s ODR too small!\n", __func__);
		return -EINVAL;
	}

	if (sdata->c_odr == lsm6ds3_odr_table.odr_avl[i].hz)
		return 0;

	if (sdata->enabled) {
		disable_irq(sdata->cdata->irq);
		lsm6ds3_flush_works();

		if (sdata->sindex == LSM6DS3_ACCEL)
			sdata->cdata->sensors[LSM6DS3_ACCEL].sample_to_discard +=
							LSM6DS3_ACCEL_STD;

		if (sdata->cdata->sensors[LSM6DS3_GYRO].enabled)
			sdata->cdata->sensors[LSM6DS3_GYRO].sample_to_discard +=
							LSM6DS3_GYRO_STD;

		err = lsm6ds3_write_data_with_mask(sdata->cdata,
				lsm6ds3_odr_table.addr[sdata->sindex],
				lsm6ds3_odr_table.mask[sdata->sindex],
				lsm6ds3_odr_table.odr_avl[i].value, true);
		if (err < 0) {
			enable_irq(sdata->cdata->irq);

			return err;
		}

		sdata->c_odr = lsm6ds3_odr_table.odr_avl[i].hz;
		enable_irq(sdata->cdata->irq);
	} else
		sdata->c_odr = lsm6ds3_odr_table.odr_avl[i].hz;

	return err;
}

int run_accel_calibration(struct lsm6ds3_data *cdata)
{
	int i, sum[3] = {0}, avg[3] = {0}, xyz[3] = {0}, xyz_final[3] = {0};
	u8 data[6] = {0};
	int err, pre_enable, odr_changed = 0;
	int discard_num = LSM6DS3_ACCEL_STD + LSM6DS3_ACCEL_STD_FROM_PD;
	int ret;

	mutex_lock(&cdata->sensors[LSM6DS3_ACCEL].input_dev->mutex);

	pre_enable = cdata->sensors[LSM6DS3_ACCEL].enabled;
	if (!pre_enable)
		lsm6ds3_enable_sensors(&cdata->sensors[LSM6DS3_ACCEL]);

	if (cdata->sensors[LSM6DS3_ACCEL].c_odr < CAL_HZ) { /* set rate to 100HZ for fast-read data */
		err = lsm6ds3_set_odr(&cdata->sensors[LSM6DS3_ACCEL], CAL_HZ);
		odr_changed = 1;
	}

	mdelay(1000/CAL_HZ*discard_num);

	for (i = 0; i < CAL_TIME; i++) {
		err = cdata->tf->read(cdata, LSM6DS3_ACC_OUT_X_L_ADDR, LSM6DS3_OUT_XYZ_SIZE, data, true);
		if (err < 0) {
			pr_info("I2C read accel raw data error!\n");
			ret = -1;
			return ret;
		}
		xyz[0] = (s32)((s16)(data[0] | (data[1] << 8)));
		xyz[1] = (s32)((s16)(data[2] | (data[3] << 8)));
		xyz[2] = (s32)((s16)(data[4] | (data[5] << 8)));
		xyz[0] *= cdata->sensors[LSM6DS3_ACCEL].sensitivity;
		xyz[1] *= cdata->sensors[LSM6DS3_ACCEL].sensitivity;
		xyz[2] *= cdata->sensors[LSM6DS3_ACCEL].sensitivity;
		xyz_final[0] = cdata->negate_x ? -xyz[cdata->axis_map_x] : xyz[cdata->axis_map_x];
		xyz_final[1] = cdata->negate_y ? -xyz[cdata->axis_map_y] : xyz[cdata->axis_map_y];
		xyz_final[2] = cdata->negate_z ? -xyz[cdata->axis_map_z] : xyz[cdata->axis_map_z];
		pr_info("%d: x:%d y:%d z:%d\n", i, xyz[0], xyz[1], xyz[2]);
		sum[0] += xyz_final[0];
		sum[1] += xyz_final[1];
		sum[2] += xyz_final[2];
		mdelay(1000/CAL_HZ);
	}

	avg[0] = sum[0] / CAL_TIME;
	avg[1] = sum[1] / CAL_TIME;
	avg[2] = sum[2] / CAL_TIME;

	pr_info("%s accel avg data x:%d y:%d z:%d\n", __func__, avg[0], avg[1], avg[2]);

	/* acc_h_max = (0.5*1024.0/9.8); */
	/* acc_h_min =  (-acc_h_max); */
	/* acc_v_max =  (10.3*1024.0/9.8); */
	/* acc_v_min =  (9.3*1024.0/9.8); */

	/* the right data is avg[0]=0 avg[1]=0 avg[2]=1024 */
	if ((avg[0] > ACC_CAL_XY_RANGE_MAX || avg[0] < ACC_CAL_XY_RANGE_MIN) ||
		(avg[1] > ACC_CAL_XY_RANGE_MAX || avg[1] < ACC_CAL_XY_RANGE_MIN) ||
		(avg[2] > ACC_CAL_Z_RANGE_MAX || avg[2] < ACC_CAL_Z_RANGE_MIN)) {
		pr_info("%s Please keep the machine almost horizontal when being calibrated\n", __func__);
		mutex_unlock(&cdata->sensors[LSM6DS3_ACCEL].input_dev->mutex);
		if (!pre_enable)
			lsm6ds3_disable_sensors(&cdata->sensors[LSM6DS3_ACCEL]);

		cdata->sensors[LSM6DS3_ACCEL].cal_x = 0;
		cdata->sensors[LSM6DS3_ACCEL].cal_y = 0;
		cdata->sensors[LSM6DS3_ACCEL].cal_z = 0;
		cdata->sensors[LSM6DS3_ACCEL].cal_result = -1;
		memset(cdata->sensors[LSM6DS3_ACCEL].calibrate_buf, 0,
			sizeof(cdata->sensors[LSM6DS3_ACCEL].calibrate_buf));
		cdata->accel_cdev.params = cdata->sensors[LSM6DS3_ACCEL].calibrate_buf;
		ret = -1;
		return ret;
	}

	cdata->sensors[LSM6DS3_ACCEL].cal_x = avg[0];
	cdata->sensors[LSM6DS3_ACCEL].cal_y = avg[1];
	cdata->sensors[LSM6DS3_ACCEL].cal_z = avg[2] - 16384;
	cdata->sensors[LSM6DS3_ACCEL].cal_result = 0;

	snprintf(cdata->sensors[LSM6DS3_ACCEL].calibrate_buf, sizeof(cdata->sensors[LSM6DS3_ACCEL].calibrate_buf),
		"%d,%d,%d",
		cdata->sensors[LSM6DS3_ACCEL].cal_x,
		cdata->sensors[LSM6DS3_ACCEL].cal_y,
		cdata->sensors[LSM6DS3_ACCEL].cal_z);
	cdata->accel_cdev.params = cdata->sensors[LSM6DS3_ACCEL].calibrate_buf;

	pr_info("acc cal_x:%d cal_y:%d cal_z:%d\n", cdata->sensors[LSM6DS3_ACCEL].cal_x,
		cdata->sensors[LSM6DS3_ACCEL].cal_y, cdata->sensors[LSM6DS3_ACCEL].cal_z);

	mutex_unlock(&cdata->sensors[LSM6DS3_ACCEL].input_dev->mutex);

	if (odr_changed)
		err = lsm6ds3_set_odr(&cdata->sensors[LSM6DS3_ACCEL], cdata->sensors[LSM6DS3_ACCEL].c_odr);
	if (!pre_enable)
		lsm6ds3_disable_sensors(&cdata->sensors[LSM6DS3_ACCEL]);

	return 0;
}

int run_gyro_calibration(struct lsm6ds3_data *cdata)
{
	int i, sum[3] = {0}, avg[3] = {0}, xyz[3] = {0}, xyz_final[3] = {0};
	u8 data[6] = {0};
	int err, pre_enable, odr_changed = 0;
	int discard_num = 10;
	int ret;

	mutex_lock(&cdata->sensors[LSM6DS3_GYRO].input_dev->mutex);

	pre_enable = cdata->sensors[LSM6DS3_GYRO].enabled;
	if (!pre_enable)
		lsm6ds3_enable_sensors(&cdata->sensors[LSM6DS3_GYRO]);

	if (cdata->sensors[LSM6DS3_GYRO].c_odr < CAL_HZ)  { /* set rate to 100HZ for fast-read data */
		err = lsm6ds3_set_odr(&cdata->sensors[LSM6DS3_GYRO], CAL_HZ);
		odr_changed = 1;
	}

	/* throw away the first 10 gyro data for it is not stable */
	mdelay(1000/CAL_HZ*discard_num);

	for (i = 0; i < CAL_TIME; i++) {
		err = cdata->tf->read(cdata, LSM6DS3_GYRO_OUT_X_L_ADDR, LSM6DS3_OUT_XYZ_SIZE, data, true);
		if (err < 0) {
			pr_info("I2C read gyro data error!\n");
			err = -1;
			return ret;
		}
		xyz[0] = (s32)((s16)(data[0] | (data[1] << 8)));
		xyz[1] = (s32)((s16)(data[2] | (data[3] << 8)));
		xyz[2] = (s32)((s16)(data[4] | (data[5] << 8)));
		xyz[0] *= cdata->sensors[LSM6DS3_GYRO].c_gain;
		xyz[1] *= cdata->sensors[LSM6DS3_GYRO].c_gain;
		xyz[2] *= cdata->sensors[LSM6DS3_GYRO].c_gain;
		xyz[0] /= 1000;
		xyz[1] /= 1000;
		xyz[2] /= 1000;
		xyz_final[0] = cdata->negate_x ? -xyz[cdata->axis_map_x] : xyz[cdata->axis_map_x];
		xyz_final[1] = cdata->negate_y ? -xyz[cdata->axis_map_y] : xyz[cdata->axis_map_y];
		xyz_final[2] = cdata->negate_z ? -xyz[cdata->axis_map_z] : xyz[cdata->axis_map_z];
		pr_info("%d: x:%d y:%d z:%d\n", i, xyz[0], xyz[1], xyz[2]);
		sum[0] += xyz_final[0];
		sum[1] += xyz_final[1];
		sum[2] += xyz_final[2];
		mdelay(1000/CAL_HZ);
	}

	avg[0] = sum[0] / CAL_TIME;
	avg[1] = sum[1] / CAL_TIME;
	avg[2] = sum[2] / CAL_TIME;

	pr_info("%s gyro avg data x:%d y:%d z:%d\n", __func__, avg[0], avg[1], avg[2]);

	/* gyro_max = (0.2*1000.0*180.0/3.14); */
	/* gyro_min = (-gyro_max); */

	if ((avg[0] > GYRO_CAL_RANGE_MAX || avg[0] < GYRO_CAL_RANGE_MIN)  ||
		(avg[1] > GYRO_CAL_RANGE_MAX || avg[1] < GYRO_CAL_RANGE_MIN) ||
		(avg[2] > GYRO_CAL_RANGE_MAX || avg[2] < GYRO_CAL_RANGE_MIN)) {
		pr_info("%s Please keep the machine almost horizontal when being calibrated\n", __func__);
		mutex_unlock(&cdata->sensors[LSM6DS3_GYRO].input_dev->mutex);
		if (!pre_enable)
			lsm6ds3_disable_sensors(&cdata->sensors[LSM6DS3_GYRO]);

		cdata->sensors[LSM6DS3_GYRO].cal_x = 0;
		cdata->sensors[LSM6DS3_GYRO].cal_y = 0;
		cdata->sensors[LSM6DS3_GYRO].cal_z = 0;
		cdata->sensors[LSM6DS3_GYRO].cal_result = -1;
		memset(cdata->sensors[LSM6DS3_GYRO].calibrate_buf, 0,
			sizeof(cdata->sensors[LSM6DS3_GYRO].calibrate_buf));
		cdata->gyro_cdev.params = cdata->sensors[LSM6DS3_GYRO].calibrate_buf;
		ret = -1;
		return ret;
	}

	cdata->sensors[LSM6DS3_GYRO].cal_x = avg[0];
	cdata->sensors[LSM6DS3_GYRO].cal_y = avg[1];
	cdata->sensors[LSM6DS3_GYRO].cal_z = avg[2];
	cdata->sensors[LSM6DS3_GYRO].cal_result = 0;

	snprintf(cdata->sensors[LSM6DS3_GYRO].calibrate_buf, sizeof(cdata->sensors[LSM6DS3_GYRO].calibrate_buf),
		"%d,%d,%d",
		cdata->sensors[LSM6DS3_GYRO].cal_x,
		cdata->sensors[LSM6DS3_GYRO].cal_y,
		cdata->sensors[LSM6DS3_GYRO].cal_z);
	cdata->gyro_cdev.params = cdata->sensors[LSM6DS3_GYRO].calibrate_buf;

	pr_info("gyro cal_x:%d cal_y:%d cal_z:%d\n", cdata->sensors[LSM6DS3_GYRO].cal_x,
		cdata->sensors[LSM6DS3_GYRO].cal_y, cdata->sensors[LSM6DS3_GYRO].cal_z);

	mutex_unlock(&cdata->sensors[LSM6DS3_GYRO].input_dev->mutex);

	if (odr_changed)
		err = lsm6ds3_set_odr(&cdata->sensors[LSM6DS3_GYRO], cdata->sensors[LSM6DS3_GYRO].c_odr);
	if (!pre_enable)
		lsm6ds3_disable_sensors(&cdata->sensors[LSM6DS3_GYRO]);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id lsm6ds3_dt_id[] = {
	{.compatible = "st,lsm6ds3",},
	{},
};
MODULE_DEVICE_TABLE(of, lsm6ds3_dt_id);

static u32 lsm6ds3_parse_dt(struct lsm6ds3_data *cdata)
{
	struct device_node *np;
	u32 temp_val;
	int rc;

	np = cdata->dev->of_node;
	if (!np)
		return -EINVAL;

	rc = of_property_read_u32(np, "st,axis-map-x", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(cdata->dev, "Unable to read axis-map_x\n");
		return rc;
	}
	cdata->axis_map_x = (u8)temp_val;

	rc = of_property_read_u32(np, "st,axis-map-y", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(cdata->dev, "Unable to read axis_map_y\n");
		return rc;
	}
	cdata->axis_map_y = (u8)temp_val;

	rc = of_property_read_u32(np, "st,axis-map-z", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(cdata->dev, "Unable to read axis-map-z\n");
		return rc;
	}
	cdata->axis_map_z = (u8)temp_val;

	cdata->negate_x = of_property_read_bool(np, "st,negate-x");
	cdata->negate_y = of_property_read_bool(np, "st,negate-y");
	cdata->negate_z = of_property_read_bool(np, "st,negate-z");

	return rc;
}

#else
#endif

static int lsm6ds3_accel_poll_delay(struct sensors_classdev *sensors_cdev,
				unsigned int delay_ms)
{
	int err;
	unsigned int set_ms;
	struct lsm6ds3_data *cdata = container_of(sensors_cdev,
					struct lsm6ds3_data, accel_cdev);
	pr_info("ljs %s delay_ms=%d\n", __func__, delay_ms);

	mutex_lock(&cdata->sensors[LSM6DS3_ACCEL].input_dev->mutex);
	/*
	* Polling interval is in msec, then we have to convert it in Hz to
	* configure ODR through lsm6ds3_set_odr
	*/
	err = lsm6ds3_set_odr(&cdata->sensors[LSM6DS3_ACCEL], 1000 / delay_ms);
	if (delay_ms == 1000)
		set_ms = 500;
	else
		set_ms = delay_ms;
	if (!(err < 0)) {
		cdata->sensors[LSM6DS3_ACCEL].poll_interval = delay_ms;
		cdata->sensors[LSM6DS3_ACCEL].ktime = ktime_set(0, MS_TO_NS(set_ms));
	}
	mutex_unlock(&cdata->sensors[LSM6DS3_ACCEL].input_dev->mutex);

	return 0;
}

static int lsm6ds3_cdev_enable_accel(struct sensors_classdev *sensors_cdev,
				 unsigned int enable)
{
	int err;
	struct lsm6ds3_data *client_data = container_of(sensors_cdev,
					struct lsm6ds3_data, accel_cdev);

	pr_info("ljs %s en=%d\n", __func__, enable);
	if (enable)
		err = lsm6ds3_enable_sensors(&client_data->sensors[LSM6DS3_ACCEL]);
	else
		err = lsm6ds3_disable_sensors(&client_data->sensors[LSM6DS3_ACCEL]);

	return err;
}

static int lsm6ds3_acc_calibrate(struct sensors_classdev *sensors_cdev, int axis, int apply_now)
{
	int err = -1;
	struct lsm6ds3_data *cdata = container_of(sensors_cdev, struct lsm6ds3_data, accel_cdev);

	err = run_accel_calibration(cdata);
	if (err < 0)
		pr_info("accel_calibration failed\n");

	return err;
}

static int lsm6ds3_acc_write_calibrate(struct sensors_classdev *sensors_cdev,
		struct cal_result_t *cal_result)
{
	struct lsm6ds3_data *cdata =
		container_of(sensors_cdev, struct lsm6ds3_data, accel_cdev);

	cdata->sensors[LSM6DS3_ACCEL].cal_x = cal_result->offset_x;
	cdata->sensors[LSM6DS3_ACCEL].cal_y = cal_result->offset_y;
	cdata->sensors[LSM6DS3_ACCEL].cal_z = cal_result->offset_z;

	pr_info("%s cal_x:%d cal_y:%d cal_z:%d\n", __func__,
		cdata->sensors[LSM6DS3_ACCEL].cal_x,
		cdata->sensors[LSM6DS3_ACCEL].cal_y,
		cdata->sensors[LSM6DS3_ACCEL].cal_z);

	return 0;
}

static int lsm6ds3_gyro_poll_delay(struct sensors_classdev *sensors_cdev,
				unsigned int delay_ms)
{
	int err;
	unsigned int set_ms;
	struct lsm6ds3_data *cdata = container_of(sensors_cdev,
					struct lsm6ds3_data, gyro_cdev);
	pr_info("ljs %s delay_ms=%d\n", __func__, delay_ms);
	mutex_lock(&cdata->sensors[LSM6DS3_GYRO].input_dev->mutex);
	/*
	 * Polling interval is in msec, then we have to convert it in Hz to
	 * configure ODR through lsm6ds3_set_odr
	 */
	err = lsm6ds3_set_odr(&cdata->sensors[LSM6DS3_GYRO], 1000 / delay_ms);
	if (delay_ms == 1000)
		set_ms = 200;
	else
		set_ms = delay_ms;

	if (!(err < 0)) {
		cdata->sensors[LSM6DS3_GYRO].poll_interval = delay_ms;
		cdata->sensors[LSM6DS3_GYRO].ktime = ktime_set(0, MS_TO_NS(set_ms));
		cdata->delay_time = delay_ms;
	}
	mutex_unlock(&cdata->sensors[LSM6DS3_GYRO].input_dev->mutex);

	return 0;
}

static int lsm6ds3_cdev_enable_gyro(struct sensors_classdev *sensors_cdev,
				 unsigned int enable)
{
	int err;
	struct lsm6ds3_data *client_data = container_of(sensors_cdev,
					struct lsm6ds3_data, gyro_cdev);
	pr_info("ljs %s en=%d\n", __func__, enable);

	if (enable)
		err = lsm6ds3_enable_sensors(&client_data->sensors[LSM6DS3_GYRO]);
	else
		err = lsm6ds3_disable_sensors(&client_data->sensors[LSM6DS3_GYRO]);

	return err;
}

static int lsm6ds3_gyro_calibrate(struct sensors_classdev *sensors_cdev,
		int axis, int apply_now)
{
	int err = -1;
	struct lsm6ds3_data *cdata =
		container_of(sensors_cdev, struct lsm6ds3_data, gyro_cdev);

	err = run_gyro_calibration(cdata);
	if (err < 0)
		pr_info("gyro_calibration failed\n");

	return err;

}

static int lsm6ds3_gyro_write_calibrate(struct sensors_classdev *sensors_cdev,
		struct cal_result_t *cal_result)
{
	struct lsm6ds3_data *cdata =
		container_of(sensors_cdev, struct lsm6ds3_data, gyro_cdev);

	cdata->sensors[LSM6DS3_GYRO].cal_x = cal_result->offset_x;
	cdata->sensors[LSM6DS3_GYRO].cal_y = cal_result->offset_y;
	cdata->sensors[LSM6DS3_GYRO].cal_z = cal_result->offset_z;

	pr_info("%s cal_x:%d cal_y:%d cal_z:%d\n", __func__,
		cdata->sensors[LSM6DS3_GYRO].cal_x,
		cdata->sensors[LSM6DS3_GYRO].cal_y,
		cdata->sensors[LSM6DS3_GYRO].cal_z);

	return 0;
}

static int lsm6ds3_stepcounter_poll_delay(
			struct sensors_classdev *sensors_cdev, unsigned int delay_ms)
{
	struct lsm6ds3_data *cdata = container_of(sensors_cdev,
					struct lsm6ds3_data, stepcounter_cdev);

	pr_info("ljs %s delay_ms=%d\n", __func__, delay_ms);

	mutex_lock(&cdata->sensors[LSM6DS3_STEP_COUNTER].input_dev->mutex);

	cdata->sensors[LSM6DS3_STEP_COUNTER].poll_interval = delay_ms;
	cdata->sensors[LSM6DS3_STEP_COUNTER].ktime = ktime_set(0, MS_TO_NS(delay_ms));

	mutex_unlock(&cdata->sensors[LSM6DS3_STEP_COUNTER].input_dev->mutex);

	return 0;
}

static int lsm6ds3_cdev_enable_stepcounter(
			struct sensors_classdev *sensors_cdev, unsigned int enable)
{
	int err;
	struct lsm6ds3_data *client_data = container_of(sensors_cdev,
					struct lsm6ds3_data, stepcounter_cdev);

	pr_info("ljs %s en=%d\n", __func__, enable);

	if (enable)
		err = lsm6ds3_enable_sensors(&client_data->sensors[LSM6DS3_STEP_COUNTER]);
	else
		err = lsm6ds3_disable_sensors(&client_data->sensors[LSM6DS3_STEP_COUNTER]);

	return err;
}


/* lijiangshuo add for proc system start */
static int lsm6ds3_proc_show(struct seq_file *m, void *v)
{
	return seq_printf(m, "0x%x\n", chip_id);
}
static int lsm6ds3_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, lsm6ds3_proc_show, NULL);
}

static const struct file_operations lsm6ds3_proc_fops = {
	.open	= lsm6ds3_proc_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};

static void create_lsm6ds3_info_proc_file(void)
{
	lsm6ds3_info_proc_file_accel =
		proc_create("driver/accel", 0644, NULL, &lsm6ds3_proc_fops);
	if (!lsm6ds3_info_proc_file_accel)
		pr_info("lsm6ds3 accel proc file create failed!\n");

	lsm6ds3_info_proc_file_gyro =
		proc_create("driver/gyro", 0644, NULL, &lsm6ds3_proc_fops);
	if (!lsm6ds3_info_proc_file_gyro)
		pr_info("lsm6ds3 gyro proc file create failed!\n");
}

static void remove_lsm6ds3_info_proc_file(void)
{
	if (lsm6ds3_info_proc_file_accel) {
		remove_proc_entry("driver/accel", NULL);
		lsm6ds3_info_proc_file_accel = NULL;
	}
	if (lsm6ds3_info_proc_file_gyro) {
		remove_proc_entry("driver/gyro", NULL);
		lsm6ds3_info_proc_file_gyro = NULL;
	}

}
/* lijiangshuo add for proc system end */

int lsm6ds3_common_probe(struct lsm6ds3_data *cdata, int irq, u16 bustype)
{
	/* TODO: add errors management */
	int32_t err, i;
	u8 wai = 0x00;
	struct lsm6ds3_sensor_data *sdata;

	mutex_init(&cdata->bank_registers_lock);

	err = cdata->tf->read(cdata, LSM6DS3_WHO_AM_I, 1, &wai, true);
	if (err < 0) {
		dev_err(cdata->dev, "failed to read Who-Am-I register.\n");
		return err;
	}
	if ((wai != LSM6DS3_WHO_AM_I_DEF) && (wai != LSM6DSE_WHO_AM_I_DEF)) {
		dev_err(cdata->dev, "Who-Am-I value not valid.\n");
		return -ENODEV;
	}

	chip_id = wai;

	mutex_init(&cdata->lock);

	err = lsm6ds3_parse_dt(cdata);
	if (err < 0)
		return err;

	if (irq > 0) {
#ifdef CONFIG_OF
		err = lsm6ds3_parse_dt(cdata);
		if (err < 0)
			return err;
#else /* CONFIG_OF */
		if (cdata->dev->platform_data) {
			cdata->drdy_int_pin = ((struct lsm6ds3_platform_data *)
					cdata->dev->platform_data)->drdy_int_pin;

			if ((cdata->drdy_int_pin > 2) ||
						(cdata->drdy_int_pin < 1))
				cdata->drdy_int_pin = 1;
		} else
			cdata->drdy_int_pin = 1;
#endif /* CONFIG_OF */

		dev_info(cdata->dev, "driver use DRDY int pin %d\n",
							cdata->drdy_int_pin);
	}

	for (i = 0; i < LSM6DS3_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];
		sdata->enabled = false;
		sdata->cdata = cdata;
		sdata->sindex = i;
		sdata->name = lsm6ds3_sensor_name[i].name;
		sdata->cal_result = -1;
		sdata->cal_x = 0;
		sdata->cal_y = 0;
		sdata->cal_z = 0;
		if ((i == LSM6DS3_ACCEL) || (i == LSM6DS3_GYRO)) {
			sdata->c_odr = lsm6ds3_odr_table.odr_avl[0].hz;
			sdata->c_gain = lsm6ds3_fs_table[i].fs_avl[1].gain;
			sdata->poll_interval = 1000 / sdata->c_odr;
		}
		if (i == LSM6DS3_STEP_COUNTER) {
			sdata->c_odr = LSM6DS3_MIN_DURATION_MS;
			sdata->poll_interval = 1000 / sdata->c_odr;
		}

		lsm6ds3_input_init(sdata, bustype,
					lsm6ds3_sensor_name[i].description);
	}

	/*to do*/
	cdata->accel_cdev = accel_cdev;
	cdata->accel_cdev.sensors_enable = lsm6ds3_cdev_enable_accel;
	cdata->accel_cdev.sensors_poll_delay = lsm6ds3_accel_poll_delay;
	cdata->accel_cdev.sensors_calibrate = lsm6ds3_acc_calibrate;
	cdata->accel_cdev.sensors_write_cal_params = lsm6ds3_acc_write_calibrate;
	err = sensors_classdev_register(&cdata->sensors[LSM6DS3_ACCEL].input_dev->dev,
			 &cdata->accel_cdev);
	if (err) {
		dev_err(cdata->dev, "accel sensors class register failed.\n");
		return err;
	}

	cdata->gyro_cdev = gyro_cdev;
	cdata->gyro_cdev.sensors_enable = lsm6ds3_cdev_enable_gyro;
	cdata->gyro_cdev.sensors_poll_delay = lsm6ds3_gyro_poll_delay;
	cdata->gyro_cdev.sensors_calibrate = lsm6ds3_gyro_calibrate;
	cdata->gyro_cdev.sensors_write_cal_params = lsm6ds3_gyro_write_calibrate;
	err = sensors_classdev_register(&cdata->sensors[LSM6DS3_GYRO].input_dev->dev,
			&cdata->gyro_cdev);
	if (err) {
		dev_err(cdata->dev, "gyro sensors class register failed.\n");
		return err;
	}

	cdata->stepcounter_cdev = stepcounter_cdev;
	cdata->stepcounter_cdev.sensors_enable = lsm6ds3_cdev_enable_stepcounter;
	cdata->stepcounter_cdev.sensors_poll_delay = lsm6ds3_stepcounter_poll_delay;
	err = sensors_classdev_register(
		&cdata->sensors[LSM6DS3_STEP_COUNTER].input_dev->dev,
		&cdata->stepcounter_cdev);
	if (err) {
		dev_err(cdata->dev, "step counter sensors class register failed.\n");
		return err;
	}


	if (lsm6ds3_workqueue == 0)
		lsm6ds3_workqueue = create_workqueue("lsm6ds3_workqueue");

	err = lsm6ds3_init_sensors(cdata);
	if (err < 0)
		return err;
	if (irq > 0)
		cdata->irq = irq;

	if (irq > 0) {
		err = lsm6ds3_allocate_workqueue(cdata);
		if (err < 0)
			return err;
	}

	if (chip_id != 0xff)
		create_lsm6ds3_info_proc_file();

	dev_info(cdata->dev, "%s: probed\n", LSM6DS3_ACC_GYR_DEV_NAME);
	return 0;
}
EXPORT_SYMBOL(lsm6ds3_common_probe);

void lsm6ds3_common_remove(struct lsm6ds3_data *cdata, int irq)
{
	u8 i;

	for (i = 0; i < LSM6DS3_SENSORS_NUMB; i++) {
		lsm6ds3_disable_sensors(&cdata->sensors[i]);
		lsm6ds3_input_cleanup(&cdata->sensors[i]);
	}

	if (!lsm6ds3_workqueue) {
		flush_workqueue(lsm6ds3_workqueue);
		destroy_workqueue(lsm6ds3_workqueue);
	}

	if (chip_id != 0xff)
		remove_lsm6ds3_info_proc_file();
}
EXPORT_SYMBOL(lsm6ds3_common_remove);

#ifdef CONFIG_PM
int lsm6ds3_common_suspend(struct lsm6ds3_data *cdata)
{
	pr_info("%s in\n", __func__);
	if (cdata->sensors[LSM6DS3_STEP_COUNTER].enabled == 1) {
		cancel_work_sync(&cdata->sensors[LSM6DS3_STEP_COUNTER].input_work);
		hrtimer_cancel(&cdata->sensors[LSM6DS3_STEP_COUNTER].hr_timer);
	}

	return 0;
}
EXPORT_SYMBOL(lsm6ds3_common_suspend);

int lsm6ds3_common_resume(struct lsm6ds3_data *cdata)
{
	pr_info("%s in, ktimer=%lld\n", __func__,
		cdata->sensors[LSM6DS3_STEP_COUNTER].ktime.tv64);

	if (cdata->sensors[LSM6DS3_STEP_COUNTER].enabled == 1)
		hrtimer_start(&cdata->sensors[LSM6DS3_STEP_COUNTER].hr_timer,
			cdata->sensors[LSM6DS3_STEP_COUNTER].ktime, HRTIMER_MODE_REL);

	return 0;
}
EXPORT_SYMBOL(lsm6ds3_common_resume);
#endif /* CONFIG_PM */

