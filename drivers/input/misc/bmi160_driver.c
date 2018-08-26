/*!
 * @section LICENSE
 * (C) Copyright 2011~2016 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http: www.fsf.org/copyleft/gpl.html
 *
 * @filename bmi160_driver.c
 * @date     2015/08/17 14:40
 * @id       "09afbe6"
 * @version  1.4
 *
 * @brief
 * The core code of BMI160 device driver
 *
 * @detail
 * This file implements the core code of BMI160 device driver,
 * which includes hardware related functions, input device register,
 * device attribute files, etc.
*/
#include "bmi160_driver.h"
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/proc_fs.h> /* lijiangshuo add for proc system */

#define I2C_BURST_READ_MAX_LEN      (256)
#define BMI160_STORE_COUNT  (6000)
#define LMADA     (1)
uint64_t g_current_apts_us;
/* static unsigned char g_fifo_data_arr[2048]; 1024 + 12*4 */

/*BMI power supply VDD 1.71V-3.6V VIO 1.2-3.6V */
#define BMI160_VDD_MIN_UV       1750000
#define BMI160_VDD_MAX_UV       3600000
#define BMI160_VIO_MIN_UV       1200000
#define BMI160_VIO_MAX_UV       3600000

/* The proper range for sensors calibration */
#define	ACC_CAL_XY_RANGE_MIN		-2950 /* 180mg  //1.76m/s2 */
#define	ACC_CAL_XY_RANGE_MAX		2950
#define	ACC_CAL_Z_RANGE_MIN		13434
#define	ACC_CAL_Z_RANGE_MAX		19334
#define	GYRO_CAL_RANGE_MIN		-40000 /* 40dps // 0.7rad/s */
#define	GYRO_CAL_RANGE_MAX		40000

static struct sensors_classdev accel_cdev = {
	.name = "bmi160-accel",
	.vendor = "Bosch Corporation",
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
	.name = "bmi160-gyro",
	.vendor = "Bosch Corporation",
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
	.name = "bmi160-stepcounter",
	.vendor = "Bosch Corporation",
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
enum BMI_SENSOR_INT_T {
	/* Interrupt enable0*/
	BMI_ANYMO_X_INT = 0,
	BMI_ANYMO_Y_INT,
	BMI_ANYMO_Z_INT,
	BMI_D_TAP_INT,
	BMI_S_TAP_INT,
	BMI_ORIENT_INT,
	BMI_FLAT_INT,
	/* Interrupt enable1*/
	BMI_HIGH_X_INT,
	BMI_HIGH_Y_INT,
	BMI_HIGH_Z_INT,
	BMI_LOW_INT,
	BMI_DRDY_INT,
	BMI_FFULL_INT,
	BMI_FWM_INT,
	/* Interrupt enable2 */
	BMI_NOMOTION_X_INT,
	BMI_NOMOTION_Y_INT,
	BMI_NOMOTION_Z_INT,
	BMI_STEP_DETECTOR_INT,
	INT_TYPE_MAX
};

/*bmi fifo sensor type combination*/
enum BMI_SENSOR_FIFO_COMBINATION {
	BMI_FIFO_A = 0,
	BMI_FIFO_G,
	BMI_FIFO_M,
	BMI_FIFO_G_A,
	BMI_FIFO_M_A,
	BMI_FIFO_M_G,
	BMI_FIFO_M_G_A,
	BMI_FIFO_COM_MAX
};

/*bmi fifo analyse return err status*/
enum BMI_FIFO_ANALYSE_RETURN_T {
	FIFO_OVER_READ_RETURN = -10,
	FIFO_SENSORTIME_RETURN = -9,
	FIFO_SKIP_OVER_LEN = -8,
	FIFO_M_G_A_OVER_LEN = -7,
	FIFO_M_G_OVER_LEN = -6,
	FIFO_M_A_OVER_LEN = -5,
	FIFO_G_A_OVER_LEN = -4,
	FIFO_M_OVER_LEN = -3,
	FIFO_G_OVER_LEN = -2,
	FIFO_A_OVER_LEN = -1
};

/*!bmi sensor generic power mode enum */
enum BMI_DEV_OP_MODE {
	SENSOR_PM_NORMAL = 0,
	SENSOR_PM_LP1,
	SENSOR_PM_SUSPEND,
	SENSOR_PM_LP2
};

/*! bmi acc sensor power mode enum */
enum BMI_ACC_PM_TYPE {
	BMI_ACC_PM_NORMAL = 0,
	BMI_ACC_PM_LP1,
	BMI_ACC_PM_SUSPEND,
	BMI_ACC_PM_LP2,
	BMI_ACC_PM_MAX
};

/*! bmi gyro sensor power mode enum */
enum BMI_GYRO_PM_TYPE {
	BMI_GYRO_PM_NORMAL = 0,
	BMI_GYRO_PM_FAST_START,
	BMI_GYRO_PM_SUSPEND,
	BMI_GYRO_PM_MAX
};

/*! bmi mag sensor power mode enum */
enum BMI_MAG_PM_TYPE {
	BMI_MAG_PM_NORMAL = 0,
	BMI_MAG_PM_LP1,
	BMI_MAG_PM_SUSPEND,
	BMI_MAG_PM_LP2,
	BMI_MAG_PM_MAX
};


/*! bmi sensor support type*/
enum BMI_SENSOR_TYPE {
	BMI_ACC_SENSOR,
	BMI_GYRO_SENSOR,
	BMI_MAG_SENSOR,
	BMI_SENSOR_TYPE_MAX
};

/*!bmi sensor generic power mode enum */
enum BMI_AXIS_TYPE {
	X_AXIS = 0,
	Y_AXIS,
	Z_AXIS,
	AXIS_MAX
};

/*!bmi sensor generic intterrupt enum */
enum BMI_INT_TYPE {
	BMI160_INT0 = 0,
	BMI160_INT1,
	BMI160_INT_MAX
};

/*! bmi sensor time resolution definition*/
enum BMI_SENSOR_TIME_RS_TYPE {
	TS_0_78_HZ = 1,/*0.78HZ*/
	TS_1_56_HZ,/*1.56HZ*/
	TS_3_125_HZ,/*3.125HZ*/
	TS_6_25_HZ,/*6.25HZ*/
	TS_12_5_HZ,/*12.5HZ*/
	TS_25_HZ,/*25HZ, odr=6*/
	TS_50_HZ,/*50HZ*/
	TS_100_HZ,/*100HZ*/
	TS_200_HZ,/*200HZ*/
	TS_400_HZ,/*400HZ*/
	TS_800_HZ,/*800HZ*/
	TS_1600_HZ,/*1600HZ*/
	TS_MAX_HZ
};

/*! bmi sensor interface mode */
enum BMI_SENSOR_IF_MODE_TYPE {
	/*primary interface:autoconfig/secondary interface off*/
	P_AUTO_S_OFF = 0,
	/*primary interface:I2C/secondary interface:OIS*/
	P_I2C_S_OIS,
	/*primary interface:autoconfig/secondary interface:Magnetometer*/
	P_AUTO_S_MAG,
	/*interface mode reseved*/
	IF_MODE_RESEVED

};

/*! bmi160 acc/gyro calibration status in H/W layer */
enum BMI_CALIBRATION_STATUS_TYPE {
	/*BMI FAST Calibration ready x/y/z status*/
	BMI_ACC_X_FAST_CALI_RDY = 0,
	BMI_ACC_Y_FAST_CALI_RDY,
	BMI_ACC_Z_FAST_CALI_RDY
};

unsigned int reg_op_addr;

static const int bmi_pmu_cmd_acc_arr[BMI_ACC_PM_MAX] = {
	/*!bmi pmu for acc normal, low power1,
	 * suspend, low power2 mode command */
	CMD_PMU_ACC_NORMAL,
	CMD_PMU_ACC_LP1,
	CMD_PMU_ACC_SUSPEND,
	CMD_PMU_ACC_LP2
};

static const int bmi_pmu_cmd_gyro_arr[BMI_GYRO_PM_MAX] = {
	/*!bmi pmu for gyro normal, fast startup,
	 * suspend mode command */
	CMD_PMU_GYRO_NORMAL,
	CMD_PMU_GYRO_FASTSTART,
	CMD_PMU_GYRO_SUSPEND
};

static const int bmi_pmu_cmd_mag_arr[BMI_MAG_PM_MAX] = {
	/*!bmi pmu for mag normal, low power1,
	 * suspend, low power2 mode command */
	CMD_PMU_MAG_NORMAL,
	CMD_PMU_MAG_LP1,
	CMD_PMU_MAG_SUSPEND,
	CMD_PMU_MAG_LP2
};

/* static const char *bmi_axis_name[AXIS_MAX] = {"x", "y", "z"}; */

static const int bmi_interrupt_type[] = {
	/*!bmi interrupt type */
	/* Interrupt enable0 , index=0~6*/
	BMI160_ANY_MOTION_X_ENABLE,
	BMI160_ANY_MOTION_Y_ENABLE,
	BMI160_ANY_MOTION_Z_ENABLE,
	BMI160_DOUBLE_TAP_ENABLE,
	BMI160_SINGLE_TAP_ENABLE,
	BMI160_ORIENT_ENABLE,
	BMI160_FLAT_ENABLE,
	/* Interrupt enable1, index=7~13*/
	BMI160_HIGH_G_X_ENABLE,
	BMI160_HIGH_G_Y_ENABLE,
	BMI160_HIGH_G_Z_ENABLE,
	BMI160_LOW_G_ENABLE,
	BMI160_DATA_RDY_ENABLE,
	BMI160_FIFO_FULL_ENABLE,
	BMI160_FIFO_WM_ENABLE,
	/* Interrupt enable2, index = 14~17*/
	BMI160_NOMOTION_X_ENABLE,
	BMI160_NOMOTION_Y_ENABLE,
	BMI160_NOMOTION_Z_ENABLE,
	BMI160_STEP_DETECTOR_EN
};

/*! bmi sensor time depend on ODR*/
struct bmi_sensor_time_odr_tbl {
	u32 ts_duration_lsb;
	u32 ts_duration_us;
	u32 ts_delat;/*sub current delat fifo_time*/
};

struct bmi160_axis_data_t {
	int x;
	int y;
	int z;
};
struct bmi160_value_t {
	struct bmi160_axis_data_t acc;
	struct bmi160_axis_data_t gyro;
#ifdef BMI160_MAG_INTERFACE_SUPPORT
	struct bmi160_axis_data_t mag;
#endif
	int64_t ts_intvl;
};
struct bmi160_type_mapping_type {

	/*! bmi16x sensor chip id */
	uint16_t chip_id;

	/*! bmi16x chip revision code */
	uint16_t revision_id;

	/*! bmi160 sensor name */
	const char *sensor_name;
};

struct bmi160_store_info_t {
	uint8_t current_frm_cnt;
	uint64_t current_apts_us[2];
	uint8_t fifo_ts_total_frmcnt;
	uint64_t fifo_time;
};
static struct workqueue_struct *reportdata_wq;
#define FIFO_READ_LENGTH_RECOMMENDED    (67)
#define FIFO_SENSORTIME_OVERFLOW_MASK   (0x1000000)
#define FIFO_SENSORTIME_RESOLUTION              (390625)

static uint8_t s_fifo_data_buf[FIFO_DATA_BUFSIZE * 2] = {0};

#if defined(BMI160_ENABLE_INT1) || defined(BMI160_ENABLE_INT2)
static uint64_t sensor_time_old = 0;
static uint64_t sensor_time_new = 0;
static uint64_t host_time_old = 0;
static uint64_t host_time_new = 0;
#endif

/* #define BMI_RING_BUF_SIZE 100 */

/* static struct bmi160_value_t bmi_ring_buf[BMI_RING_BUF_SIZE]; */
/* static int s_ring_buf_head = 0; */
/* static int s_ring_buf_tail = 0; */

/* lijiangshuo add for proc file start */
static int chip_id = 0xff;
static struct proc_dir_entry *bmi160_info_proc_file_accel;
static struct proc_dir_entry *bmi160_info_proc_file_gyro;
/* lijiangshuo add for proc file end*/

static struct workqueue_struct *bmi160_workqueue = 0;

uint64_t get_current_timestamp(void)
{
	uint64_t ts;
	struct timeval tv;

	do_gettimeofday(&tv);
	ts = (uint64_t)tv.tv_sec * 1000000 + tv.tv_usec;

	return ts;
}


/*! sensor support type map */
static const struct bmi160_type_mapping_type sensor_type_map[] = {

	{SENSOR_CHIP_ID_BMI, SENSOR_CHIP_REV_ID_BMI, "BMI160/162AB"},
	{SENSOR_CHIP_ID_BMI_C2, SENSOR_CHIP_REV_ID_BMI, "BMI160C2"},
	{SENSOR_CHIP_ID_BMI_C3, SENSOR_CHIP_REV_ID_BMI, "BMI160C3"},

};

/*!bmi160 sensor time depends on ODR */
static const struct bmi_sensor_time_odr_tbl
		sensortime_duration_tbl[TS_MAX_HZ] = {
	{0x010000, 2560000, 0x00ffff},/*2560ms, 0.39hz, odr=resver*/
	{0x008000, 1280000, 0x007fff},/*1280ms, 0.78hz, odr_acc=1*/
	{0x004000, 640000, 0x003fff},/*640ms, 1.56hz, odr_acc=2*/
	{0x002000, 320000, 0x001fff},/*320ms, 3.125hz, odr_acc=3*/
	{0x001000, 160000, 0x000fff},/*160ms, 6.25hz, odr_acc=4*/
	{0x000800, 80000,  0x0007ff},/*80ms, 12.5hz*/
	{0x000400, 40000, 0x0003ff},/*40ms, 25hz, odr_acc = odr_gyro =6*/
	{0x000200, 20000, 0x0001ff},/*20ms, 50hz, odr = 7*/
	{0x000100, 10000, 0x0000ff},/*10ms, 100hz, odr=8*/
	{0x000080, 5000, 0x00007f},/*5ms, 200hz, odr=9*/
	{0x000040, 2500, 0x00003f},/*2.5ms, 400hz, odr=10*/
	{0x000020, 1250, 0x00001f},/*1.25ms, 800hz, odr=11*/
	{0x000010, 625, 0x00000f},/*0.625ms, 1600hz, odr=12*/

};

static void bmi_dump_reg(struct bmi_client_data *client_data)
{
	#define REG_MAX0 0x24
	#define REG_MAX1 0x56
	int i;
	u8 dbg_buf0[REG_MAX0];
	u8 dbg_buf1[REG_MAX1];
	u8 dbg_buf_str0[REG_MAX0 * 3 + 1] = "";
	u8 dbg_buf_str1[REG_MAX1 * 3 + 1] = "";

	dev_notice(client_data->dev, "\nFrom 0x00:\n");

	client_data->device.bus_read(client_data->device.dev_addr,
			BMI_REG_NAME(USER_CHIP_ID), dbg_buf0, REG_MAX0);
	for (i = 0; i < REG_MAX0; i++) {
		snprintf(dbg_buf_str0 + i * 3, 8, "%02x%c", dbg_buf0[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	dev_notice(client_data->dev, "%s\n", dbg_buf_str0);

	client_data->device.bus_read(client_data->device.dev_addr,
			BMI160_USER_ACCEL_CONFIG_ADDR, dbg_buf1, REG_MAX1);
	dev_notice(client_data->dev, "\nFrom 0x40:\n");
	for (i = 0; i < REG_MAX1; i++) {
		snprintf(dbg_buf_str1 + i * 3, 8, "%02x%c", dbg_buf1[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	dev_notice(client_data->dev, "\n%s\n", dbg_buf_str1);

}

/*!
* BMI160 sensor remapping function
* need to give some parameter in BSP files first.
*/
static const struct bosch_sensor_axis_remap
	bst_axis_remap_tab_dft[MAX_AXIS_REMAP_TAB_SZ] = {
	/* src_x src_y src_z  sign_x  sign_y  sign_z */
	{  0,	 1,    2,	  1,	  1,	  1 }, /* P0 */
	{  1,	 0,    2,	  1,	 -1,	  1 }, /* P1 */
	{  0,	 1,    2,	 -1,	 -1,	  1 }, /* P2 */
	{  1,	 0,    2,	 -1,	  1,	  1 }, /* P3 */

	{  0,	 1,    2,	 -1,	  1,	 -1 }, /* P4 */
	{  1,	 0,    2,	 -1,	 -1,	 -1 }, /* P5 */
	{  0,	 1,    2,	  1,	 -1,	 -1 }, /* P6 */
	{  1,	 0,    2,	  1,	  1,	 -1 }, /* P7 */
};

static int bmi160_power_ctl(struct bmi_client_data *data, bool enable)
{
	int ret = 0;
	int err = 0;

	pr_info("%s power_enabled=%d enable=%d\n", __func__, data->power_enabled, enable);

	if (!enable && data->power_enabled) {
		ret = regulator_disable(data->vdd);
		if (ret) {
			dev_err(&data->i2c->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			dev_err(&data->i2c->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			err = regulator_enable(data->vdd);
			return ret;
		}
		data->power_enabled = enable;
	} else if (enable && !data->power_enabled) {
		ret = regulator_enable(data->vdd);
		if (ret) {
			dev_err(&data->i2c->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(data->vio);
		if (ret) {
			dev_err(&data->i2c->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			err = regulator_disable(data->vdd);
			return ret;
		}
		data->power_enabled = enable;
	} else {
		dev_info(&data->i2c->dev,
				"Power on=%d. enabled=%d\n",
				enable, data->power_enabled);
	}

	return ret;
}

static int bmi160_power_init(struct bmi_client_data *data)
{
	int ret;

	data->vdd = regulator_get(&data->i2c->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		ret = PTR_ERR(data->vdd);
		dev_err(&data->i2c->dev,
			"Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		ret = regulator_set_voltage(data->vdd,
				BMI160_VDD_MIN_UV,
				BMI160_VDD_MAX_UV);
		if (ret) {
			dev_err(&data->i2c->dev,
				"Regulator set failed vdd ret=%d\n",
				ret);
			goto reg_vdd_put;
		}
	}

	data->vio = regulator_get(&data->i2c->dev, "vio");
	if (IS_ERR(data->vio)) {
		ret = PTR_ERR(data->vio);
		dev_err(&data->i2c->dev,
			"Regulator get failed vio ret=%d\n", ret);
		goto reg_vdd_set;
	}

	if (regulator_count_voltages(data->vio) > 0) {
			ret = regulator_set_voltage(data->vio,
				BMI160_VIO_MIN_UV,
				BMI160_VIO_MAX_UV);
		if (ret) {
			dev_err(&data->i2c->dev,
			"Regulator set failed vio ret=%d\n", ret);
			goto reg_vio_put;
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, BMI160_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}

static int bmi160_power_deinit(struct bmi_client_data *data)
{
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd,
				0, BMI160_VDD_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vio) > 0)
		regulator_set_voltage(data->vio,
				0, BMI160_VIO_MAX_UV);

	regulator_put(data->vio);

	return 0;
}

static void bst_remap_sensor_data(struct bosch_sensor_data *data,
			const struct bosch_sensor_axis_remap *remap)
{
	struct bosch_sensor_data tmp;

	tmp.x = data->v[remap->src_x] * remap->sign_x;
	tmp.y = data->v[remap->src_y] * remap->sign_y;
	tmp.z = data->v[remap->src_z] * remap->sign_z;

	memcpy(data, &tmp, sizeof(*data));
}

static void bst_remap_sensor_data_dft_tab(struct bosch_sensor_data *data,
			int place)
{
/* sensor with place 0 needs not to be remapped */
	if ((place <= 0) || (place >= MAX_AXIS_REMAP_TAB_SZ))
		return;
	bst_remap_sensor_data(data, &bst_axis_remap_tab_dft[place]);
}

static void bmi_remap_sensor_data(struct bmi160_axis_data_t *val,
		struct bmi_client_data *client_data)
{
	struct bosch_sensor_data bsd;

	if ((client_data->bst_pd == NULL) ||
			(BOSCH_SENSOR_PLACE_UNKNOWN
			 == client_data->bst_pd->place))
		return;

	bsd.x = val->x;
	bsd.y = val->y;
	bsd.z = val->z;

	bst_remap_sensor_data_dft_tab(&bsd,
			client_data->bst_pd->place);

	val->x = bsd.x;
	val->y = bsd.y;
	val->z = bsd.z;

}

static void bmi_fifo_frame_bytes_extend_calc(
	struct bmi_client_data *client_data,
	unsigned int *fifo_frmbytes_extend)
{

	switch (client_data->fifo_data_sel) {
	case BMI_FIFO_A_SEL:
	case BMI_FIFO_G_SEL:
		*fifo_frmbytes_extend = 7;
		break;
	case BMI_FIFO_G_A_SEL:
		*fifo_frmbytes_extend = 13;
		break;
	case BMI_FIFO_M_SEL:
		*fifo_frmbytes_extend = 9;
		break;
	case BMI_FIFO_M_A_SEL:
	case BMI_FIFO_M_G_SEL:
		/*8(mag) + 6(gyro or acc) +1(head) = 15*/
		*fifo_frmbytes_extend = 15;
		break;
	case BMI_FIFO_M_G_A_SEL:
		/*8(mag) + 6(gyro or acc) + 6 + 1 = 21*/
		*fifo_frmbytes_extend = 21;
		break;
	default:
		*fifo_frmbytes_extend = 0;
		break;

	};

}

static int bmi_input_init(struct bmi_client_data *client_data)
{
	struct input_dev *dev;
	int err = 0;

	dev = devm_input_allocate_device(&client_data->i2c->dev);
	if (dev == NULL)
		return -ENOMEM;

	dev->name = BMI160_ACCEL_INPUT_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN, ABSMAX, 0, 0);

	input_set_drvdata(dev, client_data);
	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		dev_notice(client_data->dev, "bmi160 accel input free!\n");
		return err;
	}
	client_data->input_accel = dev;
	dev_notice(client_data->dev,
		"bmi160 accel input register successfully, %s!\n",
		client_data->input_accel->name);

	dev = devm_input_allocate_device(&client_data->i2c->dev);
	if (dev == NULL)
		return -ENOMEM;

	dev->name = BMI160_GYRO_INPUT_NAME;
	dev->id.bustype = BUS_I2C;


	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_RX, GYRO_MIN_VALUE, GYRO_MAX_VALUE, 0, 0);
	input_set_abs_params(dev, ABS_RY, GYRO_MIN_VALUE, GYRO_MAX_VALUE, 0, 0);
	input_set_abs_params(dev, ABS_RZ, GYRO_MIN_VALUE, GYRO_MAX_VALUE, 0, 0);
	input_set_drvdata(dev, client_data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		dev_notice(client_data->dev, "bmi160 accel input free!\n");
		return err;
	}
	client_data->input_gyro = dev;
	dev_notice(client_data->dev,
		"bmi160 gyro input register successfully, %s!\n",
		client_data->input_gyro->name);

	dev = devm_input_allocate_device(&client_data->i2c->dev);
	if (dev == NULL)
		return -ENOMEM;

	dev->name = BMI160_STEPCOUNTER_INPUT_NAME;
	dev->id.bustype = BUS_I2C;

	set_bit(EV_ABS, dev->evbit);
	input_set_abs_params(dev, ABS_MISC, 0, 65535, 0, 0);

	input_set_drvdata(dev, client_data);
	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		dev_notice(client_data->dev, "bmi160 stepcounter input free!\n");
		return err;
	}
	client_data->input_stepcounter = dev;
	dev_notice(client_data->dev,
		"bmi160 stepcounter input register successfully, %s!\n",
		client_data->input_stepcounter->name);

	return err;
}


static void bmi_input_destroy(struct bmi_client_data *client_data)
{
	struct input_dev *dev = client_data->input_accel;

	input_unregister_device(dev);
	input_free_device(dev);

	dev = client_data->input_gyro;
	input_unregister_device(dev);
	input_free_device(dev);
}

static int bmi_check_chip_id(struct bmi_client_data *client_data)
{
	int8_t err = 0;
	int8_t i = 0;
	uint8_t chip_id_ = 0;
	uint8_t read_count = 0;
	u8 bmi_sensor_cnt = sizeof(sensor_type_map) / sizeof(struct bmi160_type_mapping_type);

	/* read and check chip id */
	while (read_count++ < CHECK_CHIP_ID_TIME_MAX) {
		if (client_data->device.bus_read(client_data->device.dev_addr,
			BMI_REG_NAME(USER_CHIP_ID), &chip_id_, 1) < 0) {
			dev_err(client_data->dev,
				"Bosch Sensortec Device not found"
				" %d read chip_id:%d\n", read_count, chip_id_);
			mdelay(1);
			continue;
		} else {
			for (i = 0; i < bmi_sensor_cnt; i++) {
				if (sensor_type_map[i].chip_id == chip_id_) {
					client_data->chip_id = chip_id_;
					chip_id = chip_id_;
					dev_notice(client_data->dev,
						"Bosch Sensortec Device detected, "
						"HW IC name: %s\n", sensor_type_map[i].sensor_name);
					break;
				}
			}
			if (i < bmi_sensor_cnt)
				break;

			dev_err(client_data->dev,
				"Failed!Bosch Sensortec Device not found"
				" mismatch chip_id:%d\n", chip_id_);
			err = -ENODEV;
			return err;

		}
	}

	if (read_count == (CHECK_CHIP_ID_TIME_MAX+1)) {
		dev_err(client_data->dev,
			"Failed!Bosch Sensortec Device not found"
			" mismatch chip_id:%d\n", chip_id_);
		err = -ENODEV;
		return err;
	}

	pr_info("bmi160 chip_id=%d\n", chip_id);
	return err;
}

static int bmi_pmu_set_suspend(struct bmi_client_data *client_data)
{
	int err = 0;

	if (client_data == NULL)
		return -EINVAL;

	err += BMI_CALL_API(set_command_register)
			(bmi_pmu_cmd_acc_arr[SENSOR_PM_SUSPEND]);
	err += BMI_CALL_API(set_command_register)
			(bmi_pmu_cmd_gyro_arr[SENSOR_PM_SUSPEND]);
	err += BMI_CALL_API(set_command_register)
			(bmi_pmu_cmd_mag_arr[SENSOR_PM_SUSPEND]);
	client_data->pw.acc_pm = BMI_ACC_PM_SUSPEND;
	client_data->pw.gyro_pm = BMI_GYRO_PM_SUSPEND;
	client_data->pw.mag_pm = BMI_MAG_PM_SUSPEND;

	return err;
}

static int bmi_get_err_status(struct bmi_client_data *client_data)
{
	int err = 0;

	err = BMI_CALL_API(get_error_status)(&client_data->err_st.fatal_err,
		&client_data->err_st.err_code, &client_data->err_st.i2c_fail,
	&client_data->err_st.drop_cmd, &client_data->err_st.mag_drdy_err);
	return err;
}


static enum hrtimer_restart reportdata_timer_fun(
	struct hrtimer *hrtimer)
{
	struct bmi_client_data *client_data =
		container_of(hrtimer, struct bmi_client_data, timer);
	int32_t delay = 0;
	/* delay = atomic_read(&client_data->delay); */
	delay = client_data->accel_delay;
	queue_work(reportdata_wq, &(client_data->report_data_work));
	client_data->work_delay_kt = ns_to_ktime(delay*1000000);
	hrtimer_forward(hrtimer, ktime_get(), client_data->work_delay_kt);

	return HRTIMER_RESTART;
}

static void bmi_work_func(struct work_struct *work)
{
	struct bmi_client_data *client_data =
		container_of((struct work_struct *)work, struct bmi_client_data, accel_work);
	struct bmi160_accel_t data;
	struct bmi160_axis_data_t bmi160_udata;
	int err;
	ktime_t timestamp = ktime_get_boottime();

	/* schedule_delayed_work(&client_data->work, delay); */
	hrtimer_start(&client_data->accel_timer, client_data->accel_ktime, HRTIMER_MODE_REL);

	err = BMI_CALL_API(read_accel_xyz)(&data);
	if (err < 0)
		return;

	bmi160_udata.x = data.x;
	bmi160_udata.y = data.y;
	bmi160_udata.z = data.z;

	bmi_remap_sensor_data(&bmi160_udata, client_data);

	bmi160_udata.x -= client_data->acc_cal_x;
	bmi160_udata.y -= client_data->acc_cal_y;
	bmi160_udata.z -= client_data->acc_cal_z;

	/*report current frame via input event*/
	input_event(client_data->input_accel, EV_ABS, ABS_X, bmi160_udata.x);
	input_event(client_data->input_accel, EV_ABS, ABS_Y, bmi160_udata.y);
	input_event(client_data->input_accel, EV_ABS, ABS_Z, bmi160_udata.z);
	input_event(client_data->input_accel, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
	input_event(client_data->input_accel, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
	input_sync(client_data->input_accel);
}

static void bmi_gyro_work_func(struct work_struct *work)
{
	struct bmi_client_data *client_data =
		container_of((struct work_struct *)work, struct bmi_client_data, gyro_work);
	struct bmi160_gyro_t data;
	struct bmi160_axis_data_t bmi160_udata;
	int err;
	ktime_t timestamp = ktime_get_boottime();

	/* schedule_delayed_work(&client_data->gyro_work, delay); */
	hrtimer_start(&client_data->gyro_timer, client_data->gyro_ktime, HRTIMER_MODE_REL);

	err = BMI_CALL_API(read_gyro_xyz)(&data);
	if (err < 0)
		return;

	bmi160_udata.x = data.x;
	bmi160_udata.y = data.y;
	bmi160_udata.z = data.z;

	/* LSB to mdps */
	bmi160_udata.x = bmi160_udata.x*61; /* 1000000/16400; */
	bmi160_udata.y = bmi160_udata.y*61; /* 1000000/16400; */
	bmi160_udata.z = bmi160_udata.z*61; /* 1000000/16400; */

	bmi_remap_sensor_data(&bmi160_udata, client_data);

	bmi160_udata.x -= client_data->gyro_cal_x;
	bmi160_udata.y -= client_data->gyro_cal_y;
	bmi160_udata.z -= client_data->gyro_cal_z;

	/*report current frame via input event*/
	input_event(client_data->input_gyro, EV_ABS, ABS_RX, bmi160_udata.x);
	input_event(client_data->input_gyro, EV_ABS, ABS_RY, bmi160_udata.y);
	input_event(client_data->input_gyro, EV_ABS, ABS_RZ, bmi160_udata.z);
	input_event(client_data->input_gyro, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
	input_event(client_data->input_gyro, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
	input_sync(client_data->input_gyro);
}

static void bmi_stepcounter_work_func(struct work_struct *work)
{
	struct bmi_client_data *client_data =
		container_of((struct work_struct *)work, struct bmi_client_data, stepcounter_work);
	int err;
	s16 data;

	/* schedule_delayed_work(&client_data->stepcounter_work, delay); */
	hrtimer_start(&client_data->stepcounter_timer, client_data->stepcounter_ktime, HRTIMER_MODE_REL);

	err = BMI_CALL_API(read_step_count)(&data);
	if (err < 0) {
		pr_info("%s read step counter failed!\n", __func__);
		/* schedule_delayed_work(&client_data->stepcounter_work, delay); */
		hrtimer_start(&client_data->stepcounter_timer, client_data->stepcounter_ktime, HRTIMER_MODE_REL);
	}

	/*report current frame via input event*/
	input_event(client_data->input_stepcounter, EV_ABS, ABS_MISC, data);
	input_sync(client_data->input_stepcounter);

	pr_info("%s step=%d\n", __func__, data);
}

static uint8_t dbg_buf_str[2048] = "";
static void bmi_hrtimer_work_func(struct work_struct *work)
{
	/* struct bmi_client_data *client_data =
		container_of((struct delayed_work *)work, struct bmi_client_data, work); */
	struct bmi_client_data *client_data =
		container_of((struct work_struct *)work, struct bmi_client_data, accel_work);
	unsigned int fifo_len0 = 0;
	unsigned int fifo_frmbytes_ext = 0;
	unsigned int fifo_read_len = 0;
	int i, err;

	bmi_fifo_frame_bytes_extend_calc(client_data, &fifo_frmbytes_ext);

	err = BMI_CALL_API(fifo_length)(&fifo_len0);
	client_data->fifo_bytecount = fifo_len0;

	if (client_data->fifo_bytecount == 0 || err) {
		return;
	}

	fifo_read_len = client_data->fifo_bytecount + fifo_frmbytes_ext;
	if (fifo_read_len > FIFO_DATA_BUFSIZE) {
		fifo_read_len = FIFO_DATA_BUFSIZE;
	}

	if (!err) {
		err = bmi_burst_read_wrapper(client_data->device.dev_addr,
			BMI160_USER_FIFO_DATA__REG, s_fifo_data_buf,
			fifo_read_len);
	}

	for (i = 0; i < fifo_read_len; i++) {
		snprintf(dbg_buf_str + i * 3, 8, "%02x%c", s_fifo_data_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	pr_info("%s\n", dbg_buf_str);
}

static int bmi160_set_acc_op_mode(struct bmi_client_data *client_data,
				unsigned long op_mode)
{
	int err = 0;
	unsigned char stc_enable;
	unsigned char std_enable;

	mutex_lock(&client_data->mutex_op_mode);

	if (op_mode < BMI_ACC_PM_MAX) {
		switch (op_mode) {
		case BMI_ACC_PM_NORMAL:
			err = BMI_CALL_API(set_command_register)
			(bmi_pmu_cmd_acc_arr[BMI_ACC_PM_NORMAL]);
			client_data->pw.acc_pm = BMI_ACC_PM_NORMAL;
			mdelay(10);
			break;
		case BMI_ACC_PM_LP1:
			err = BMI_CALL_API(set_command_register)
			(bmi_pmu_cmd_acc_arr[BMI_ACC_PM_LP1]);
			client_data->pw.acc_pm = BMI_ACC_PM_LP1;
			mdelay(3);
			break;
		case BMI_ACC_PM_SUSPEND:
			BMI_CALL_API(get_step_counter_enable)(&stc_enable);
			BMI_CALL_API(get_step_detector_enable)(&std_enable);
			if ((stc_enable == 0) && (std_enable == 0) &&
				(client_data->sig_flag == 0)) {
				err = BMI_CALL_API(set_command_register)
				(bmi_pmu_cmd_acc_arr[BMI_ACC_PM_SUSPEND]);
				client_data->pw.acc_pm = BMI_ACC_PM_SUSPEND;
				mdelay(10);
			}
			break;
		case BMI_ACC_PM_LP2:
			err = BMI_CALL_API(set_command_register)
			(bmi_pmu_cmd_acc_arr[BMI_ACC_PM_LP2]);
			client_data->pw.acc_pm = BMI_ACC_PM_LP2;
			mdelay(3);
			break;
		default:
			mutex_unlock(&client_data->mutex_op_mode);
			return -EINVAL;
		}
	} else {
		mutex_unlock(&client_data->mutex_op_mode);
		return -EINVAL;
	}

	mutex_unlock(&client_data->mutex_op_mode);

	return err;
}

static int bmi160_set_gyro_op_mode(struct bmi_client_data *client_data,
					unsigned long op_mode)
{
	int err = 0;

	mutex_lock(&client_data->mutex_op_mode);

	if (op_mode < BMI_GYRO_PM_MAX) {
		switch (op_mode) {
		case BMI_GYRO_PM_NORMAL:
			err = BMI_CALL_API(set_command_register)
			(bmi_pmu_cmd_gyro_arr[BMI_GYRO_PM_NORMAL]);
			client_data->pw.gyro_pm = BMI_GYRO_PM_NORMAL;
			msleep(60);
			break;
		case BMI_GYRO_PM_FAST_START:
			err = BMI_CALL_API(set_command_register)
			(bmi_pmu_cmd_gyro_arr[BMI_GYRO_PM_FAST_START]);
			client_data->pw.gyro_pm = BMI_GYRO_PM_FAST_START;
			msleep(60);
			break;
		case BMI_GYRO_PM_SUSPEND:
			err = BMI_CALL_API(set_command_register)
			(bmi_pmu_cmd_gyro_arr[BMI_GYRO_PM_SUSPEND]);
			client_data->pw.gyro_pm = BMI_GYRO_PM_SUSPEND;
			msleep(60);
			break;
		default:
			mutex_unlock(&client_data->mutex_op_mode);
			return -EINVAL;
		}
	} else {
		mutex_unlock(&client_data->mutex_op_mode);
		return -EINVAL;
	}

	mutex_unlock(&client_data->mutex_op_mode);
	return err;
}

static void bmi_delay(u32 msec)
{
	mdelay(msec);
}

#if defined(BMI160_ENABLE_INT1) || defined(BMI160_ENABLE_INT2)
static void bmi_slope_interrupt_handle(struct bmi_client_data *client_data)
{
	/* anym_first[0..2]: x, y, z */
	u8 anym_first[3] = {0};
	u8 status2;
	u8 anym_sign;
	u8 i = 0;

	client_data->device.bus_read(client_data->device.dev_addr,
				BMI160_USER_INTR_STAT_2_ADDR, &status2, 1);
	anym_first[0] = BMI160_GET_BITSLICE(status2,
				BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X);
	anym_first[1] = BMI160_GET_BITSLICE(status2,
				BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y);
	anym_first[2] = BMI160_GET_BITSLICE(status2,
				BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z);
	anym_sign = BMI160_GET_BITSLICE(status2,
				BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN);

	for (i = 0; i < 3; i++) {
		if (anym_first[i]) {
			/*1: negative*/
			if (anym_sign)
				dev_notice(client_data->dev,
				"Anymotion interrupt happened!"
				"%s axis, negative sign\n", bmi_axis_name[i]);
			else
				dev_notice(client_data->dev,
				"Anymotion interrupt happened!"
				"%s axis, postive sign\n", bmi_axis_name[i]);
		}
	}
}

static uint64_t bmi_get_alarm_timestamp_ns(void)
{
	uint64_t ts_ap;
	struct timespec tmp_time;

	get_monotonic_boottime(&tmp_time);
	ts_ap = (uint64_t)tmp_time.tv_sec * 1000000000L + tmp_time.tv_nsec;
	return ts_ap;
}

static int bmi_ring_buf_full(void)
{
	return (s_ring_buf_tail + 1) % BMI_RING_BUF_SIZE == s_ring_buf_head;
}

static int bmi_ring_buf_put(struct bmi160_value_t data)
{
	if (bmi_ring_buf_full())
		return 0;

	bmi_ring_buf[s_ring_buf_tail].acc.x = data.acc.x;
	bmi_ring_buf[s_ring_buf_tail].acc.y = data.acc.y;
	bmi_ring_buf[s_ring_buf_tail].acc.z = data.acc.z;
	bmi_ring_buf[s_ring_buf_tail].gyro.x = data.gyro.x;
	bmi_ring_buf[s_ring_buf_tail].gyro.y = data.gyro.y;
	bmi_ring_buf[s_ring_buf_tail].gyro.z = data.gyro.z;
#ifdef BMI160_MAG_INTERFACE_SUPPORT
	bmi_ring_buf[s_ring_buf_tail].mag.x = data.mag.x;
	bmi_ring_buf[s_ring_buf_tail].mag.y = data.mag.y;
	bmi_ring_buf[s_ring_buf_tail].mag.z = data.mag.z;
#endif
	bmi_ring_buf[s_ring_buf_tail].ts_intvl = data.ts_intvl;
	s_ring_buf_tail = (s_ring_buf_tail + 1) % BMI_RING_BUF_SIZE;

	return 1;
}

static int bmi_fifo_get_decode_data(struct bmi160_axis_data_t *acc,
				    struct bmi160_axis_data_t *gyro,
				    struct bmi160_axis_data_t *mag,
				    u8 *fifo_data,
				    u16 fifo_index)
{
	/* array index to get elements from fifo data*/
	u16 array_index = 0;

	/* get xyzr axis mag data */
	if (mag != NULL) {
		struct bmi160_mag_xyzr_t mag_valid;
		struct bmi160_mag_xyzr_t mag_data;
		struct bmi160_mag_xyz_s32_t mag_comp_xyz;

		mag_data.x = fifo_data[fifo_index + 0]
					| fifo_data[fifo_index + 1] << 8;
		mag_data.y = fifo_data[fifo_index + 2]
					| fifo_data[fifo_index + 3] << 8;
		mag_data.z = fifo_data[fifo_index + 4]
					| fifo_data[fifo_index + 5] << 8;
		mag_data.r = fifo_data[fifo_index + 6]
					| fifo_data[fifo_index + 7] << 8;
		fifo_index += 8;
		mag_valid.x = mag_data.x >> 3;
		mag_valid.y = mag_data.y >> 3;
		mag_valid.z = mag_data.z >> 1;
		mag_valid.r = mag_data.r >> 2;
		bmi160_bmm150_mag_compensate_xyz_raw(&mag_comp_xyz, mag_valid);
		mag->x = mag_comp_xyz.x;
		mag->y = mag_comp_xyz.y;
		mag->z = mag_comp_xyz.z;
	}

	/* get xyz axis gyro data */
	if (gyro != NULL) {
		gyro->x = fifo_data[fifo_index + 0]
					| fifo_data[fifo_index + 1] << 8;
		gyro->y = fifo_data[fifo_index + 2]
					| fifo_data[fifo_index + 3] << 8;
		gyro->z = fifo_data[fifo_index + 4]
					| fifo_data[fifo_index + 5] << 8;
		fifo_index += 6;
	}

	/* get xyz axis accel data */
	if (acc != NULL) {
		acc->x = fifo_data[fifo_index + 0]
					| fifo_data[fifo_index + 1] << 8;
		acc->y = fifo_data[fifo_index + 2]
					| fifo_data[fifo_index + 3] << 8;
		acc->z = fifo_data[fifo_index + 4]
					| fifo_data[fifo_index + 5] << 8;
		fifo_index += 6;
	}

	return array_index;

}

static int bmi_pow(int x, int y)
{
	int result = 1;

	if (y == 1)
		return x;


	if (y > 0) {
		while (--y) {
			result *= x;
		}
	} else {
		result = 0;
	}

	return result;
}

/*
* Decoding of FIFO frames (only exemplary subset). Calculates accurate timestamps based on the
* drift information. Returns value of the SENSORTIME frame.
*
* fifo_data_buf - pointer to fetched FIFO buffer
* fifo_length - number of valid bytes in the FIFO buffer
* drift - clock drift value, needed to calculate accurate timestamps
* timestamp - initial timestamp for the first sample of the FIFO chunk
* acc_odr - current value of the ACC_CONF.acc_odr register field
*/
static uint64_t bmi_fifo_data_decode(uint8_t *fifo_data_buf, uint16_t fifo_length,
					int drift, uint64_t timestamp, int odr_pow)
{
	int idx = 0;
	int ret;
	uint8_t header;
	uint64_t fifo_time = 0;
	uint64_t timestamp_next = timestamp;
	struct bmi160_value_t bmi160_value;

	while (idx < fifo_length) {
		header = fifo_data_buf[idx];
		memset(&bmi160_value, 0, sizeof(bmi160_value));
		switch (header) {
		case FIFO_HEAD_SENSOR_TIME:
			/* sensortime frame, length = 4 bytes*/
			if (idx + 3 < fifo_length) {
				fifo_time = fifo_data_buf[idx + 1] |
						fifo_data_buf[idx + 2] << 8 |
						fifo_data_buf[idx + 3] << 16;
				return fifo_time;
			}
			idx += 4;
			break;
		case FIFO_HEAD_A:
			/* accel frame, length = 7 bytes */
			idx += 1;
			if (idx + 6 < fifo_length) {
				bmi_fifo_get_decode_data(
					&bmi160_value.acc,
					 NULL, NULL,
					 fifo_data_buf, idx);
				bmi160_value.ts_intvl =
				 (int64_t)(625 * odr_pow * (drift > 0 ? drift : 1000));
				if (bmi_ring_buf_put(bmi160_value) == 0)
					pr_info("bmi ring buf full head %d, tail %d",
						s_ring_buf_head, s_ring_buf_tail);
				if (drift > 0)
					timestamp_next += (uint64_t)(625 * odr_pow * drift);
				else
					timestamp_next += (uint64_t)(625 * odr_pow * 1000);
			}
			idx += 6;
			break;
		case FIFO_HEAD_G:
			/* gyro frame, length = 7 bytes */
			idx += 1;
			if (idx + 6 < fifo_length) {
				bmi_fifo_get_decode_data(NULL,
					 &bmi160_value.gyro,
					 NULL, fifo_data_buf, idx);
				bmi160_value.ts_intvl =
				 (int64_t)(625 * odr_pow * (drift > 0 ? drift : 1000));
				if (bmi_ring_buf_put(bmi160_value) == 0)
					pr_info("bmi ring buf full head %d, tail %d",
						s_ring_buf_head, s_ring_buf_tail);
			}

			idx += 6;
			break;
		case FIFO_HEAD_G_A:
			idx += 1;
			if (idx + 12 < fifo_length) {
				bmi_fifo_get_decode_data(&bmi160_value.acc,
					 &bmi160_value.gyro,
					 NULL, fifo_data_buf, idx);
				bmi160_value.ts_intvl =
				 (int64_t)(625 * odr_pow * (drift > 0 ? drift : 1000));
				if (bmi_ring_buf_put(bmi160_value) == 0)
					pr_info("bmi ring buf full head %d, tail %d",
						s_ring_buf_head, s_ring_buf_tail);
			}

			idx += 12;
			break;
#ifdef BMI160_MAG_INTERFACE_SUPPORT
		case FIFO_HEAD_M:
			idx += 1;
			if (idx + 8 < fifo_length) {
				bmi_fifo_get_decode_data(NULL, NULL,
					 &bmi160_value.mag,
					 fifo_data_buf, idx);
				bmi160_value.ts_intvl =
				 (int64_t)(625 * odr_pow * (drift > 0 ? drift : 1000));
				if (bmi_ring_buf_put(bmi160_value) == 0)
					pr_info("bmi ring buf full head %d, tail %d",
						s_ring_buf_head, s_ring_buf_tail);
			}

			idx += 8;
			break;
		case FIFO_HEAD_M_A:
			idx += 1;
			if (idx + 14 < fifo_length) {
				bmi_fifo_get_decode_data(&bmi160_value.acc, NULL,
					 &bmi160_value.mag, fifo_data_buf, idx);
				bmi160_value.ts_intvl =
				 (int64_t)(625 * odr_pow * (drift > 0 ? drift : 1000));
				if (bmi_ring_buf_put(bmi160_value) == 0)
					pr_info("bmi ring buf full head %d, tail %d",
						s_ring_buf_head, s_ring_buf_tail);
			}

			idx += 14;
			break;

		case FIFO_HEAD_M_G:
			idx += 1;
			if (idx + 14 < fifo_length) {
				bmi_fifo_get_decode_data(NULL, &bmi160_value.gyro,
					 &bmi160_value.mag, fifo_data_buf, idx);
				bmi160_value.ts_intvl =
				 (int64_t)(625 * odr_pow * (drift > 0 ? drift : 1000));
				if (bmi_ring_buf_put(bmi160_value) == 0)
					pr_info("bmi ring buf full head %d, tail %d",
						s_ring_buf_head, s_ring_buf_tail);
			}

			idx += 14;
			break;
		case FIFO_HEAD_M_G_A:
			idx += 1;
			if (idx + 20 < fifo_length) {
				bmi_fifo_get_decode_data(&bmi160_value.acc,
						 &bmi160_value.gyro,
						 &bmi160_value.mag, fifo_data_buf, idx);
				bmi160_value.ts_intvl =
				 (int64_t)(625 * odr_pow * (drift > 0 ? drift : 1000));
				if (bmi_ring_buf_put(bmi160_value) == 0)
					pr_info("bmi ring buf full head %d, tail %d",
						s_ring_buf_head, s_ring_buf_tail);
			}

			idx += 20;
			pr_info("second interface mag");
			break;
#endif
		case FIFO_HEAD_OVER_READ_LSB:
			/* end of fifo chunk */
			idx = fifo_length;
			break;
		default:
			pr_info("ERROR parsing FIFO!! header 0x%x", header);
			idx = fifo_length;
			break;
		}
	}

	ret = -1;
	return ret;
}

static uint64_t bmi_fifo_next_timestamp_calc(uint64_t host_time_new,
						uint64_t sensor_time_new,
						int drift, int odr_pow,
						uint8_t bmi_odr)
{
	uint64_t time_age = ((sensor_time_new & (0xFFFF >> bmi_odr))
				 * (drift > 0 ? drift : 1000)) * 390625;
	return ((host_time_new - div64_u64(time_age, 10000000)
		 + ((drift > 0 ? drift : 1000) * (625 * odr_pow))));
}

static int bmi_fifo_time_drift_calc(uint64_t host_time_new,
				 uint64_t sensor_time_new,
				uint64_t host_time_old,
				 uint64_t sensor_time_old)
{
	uint64_t delta_st, delta_ht;
	int drift = 0;

	if (host_time_old > 0) {
		delta_st = sensor_time_new >= sensor_time_old ?
			(sensor_time_new - sensor_time_old) :
			(sensor_time_new + FIFO_SENSORTIME_OVERFLOW_MASK - sensor_time_old);
		delta_ht = host_time_new - host_time_old;
		if (delta_st != 0)
			drift = (int)div64_u64(delta_ht * 10000,
			 (delta_st * FIFO_SENSORTIME_RESOLUTION));
		else
			drift = 0;

	} else {
		drift = 0;
	}

	return drift;
}

static void bmi_fifo_watermark_interrupt_handle
				(struct bmi_client_data *client_data)
{
	int err = 0;
	unsigned int fifo_len0 = 0;
	unsigned int  fifo_frmbytes_ext = 0;
	static int time_drift = 0;
	static uint64_t timestamp_next = 0;
	uint8_t bmi_odr = client_data->odr.acc_odr;
	int odr_pow = bmi_pow(2, 12 - bmi_odr + 1);
	/*TO DO*/

	if (client_data->fifo_data_sel == 4) {
		bmi_odr = client_data->odr.mag_odr;
		odr_pow = bmi_pow(2, 12 - bmi_odr + 1);
	}

	bmi_fifo_frame_bytes_extend_calc(client_data, &fifo_frmbytes_ext);

	if (client_data->pw.acc_pm == 2 && client_data->pw.gyro_pm == 2
					&& client_data->pw.mag_pm == 2)
		pr_info("pw_acc: %d, pw_gyro: %d\n",
			client_data->pw.acc_pm, client_data->pw.gyro_pm);
	if (!client_data->fifo_data_sel)
		pr_info("no selsect sensor fifo, fifo_data_sel:%d\n",
						client_data->fifo_data_sel);

	err = BMI_CALL_API(fifo_length)(&fifo_len0);
	client_data->fifo_bytecount = fifo_len0;

	if (client_data->fifo_bytecount == 0 || err) {
		pr_info("fifo_bytecount is 0!!");
		return;
	}
	if (client_data->fifo_bytecount + fifo_frmbytes_ext > FIFO_DATA_BUFSIZE)
		client_data->fifo_bytecount = FIFO_DATA_BUFSIZE;
	/* need give attention for the time of burst read*/
	memset(s_fifo_data_buf, 0, sizeof(s_fifo_data_buf));
	if (!err) {
		err = bmi_burst_read_wrapper(client_data->device.dev_addr,
			BMI160_USER_FIFO_DATA__REG, s_fifo_data_buf,
			client_data->fifo_bytecount + fifo_frmbytes_ext);
		/* store host timestamp after reading fifo */
		host_time_new = bmi_get_alarm_timestamp_ns();
		if (timestamp_next == 0) {
			sensor_time_new = bmi_fifo_data_decode(s_fifo_data_buf,
				client_data->fifo_bytecount + fifo_frmbytes_ext,
				time_drift, timestamp_next, odr_pow);
		} else {
			sensor_time_new = bmi_fifo_data_decode(s_fifo_data_buf,
				client_data->fifo_bytecount + fifo_frmbytes_ext,
				time_drift, host_time_new, odr_pow);
		}
		if (sensor_time_new >= 0) {
			time_drift = bmi_fifo_time_drift_calc(host_time_new,
				sensor_time_new, host_time_old, sensor_time_old);
			timestamp_next = bmi_fifo_next_timestamp_calc(host_time_new,
				sensor_time_new, time_drift, odr_pow, bmi_odr);
			/* store current timestamps as the old ones
			for next delta calculation */
			host_time_old = host_time_new;
			sensor_time_old = sensor_time_new;
		}
	} else {
		dev_err(client_data->dev, "read fifo leght err");
	}

	if (err)
		dev_err(client_data->dev, "brust read fifo err\n");

}

static void bmi_signification_motion_interrupt_handle(
		struct bmi_client_data *client_data)
{
	pr_info("bmi_signification_motion_interrupt_handle\n");
	input_event(client_data->input_accel, EV_MSC, INPUT_EVENT_SGM, 1);
	input_sync(client_data->input_accel);
	input_event(client_data->input_gyro, EV_MSC, INPUT_EVENT_SGM, 1);
	input_sync(client_data->input_gyro);
	bmi160_set_command_register(CMD_RESET_INT_ENGINE);

}
static void bmi_stepdetector_interrupt_handle(
	struct bmi_client_data *client_data)
{
	u8 current_step_dector_st = 0;

	client_data->pedo_data.wkar_step_detector_status++;
	current_step_dector_st =
		client_data->pedo_data.wkar_step_detector_status;
	client_data->std = ((current_step_dector_st == 1) ? 0 : 1);
}

static void bmi_irq_work_func(struct work_struct *work)
{
	struct bmi_client_data *client_data =
		container_of((struct work_struct *)work,
			struct bmi_client_data, irq_work);

	unsigned char int_status[4] = {0, 0, 0, 0};

	client_data->device.bus_read(client_data->device.dev_addr,
				BMI160_USER_INTR_STAT_0_ADDR, int_status, 4);

	if (BMI160_GET_BITSLICE(int_status[0],
					BMI160_USER_INTR_STAT_0_ANY_MOTION))
		bmi_slope_interrupt_handle(client_data);

	if (BMI160_GET_BITSLICE(int_status[0],
			BMI160_USER_INTR_STAT_0_STEP_INTR))
		bmi_stepdetector_interrupt_handle(client_data);
	if (BMI160_GET_BITSLICE(int_status[1],
			BMI160_USER_INTR_STAT_1_FIFO_WM_INTR))
		bmi_fifo_watermark_interrupt_handle(client_data);

	/* Clear ALL inputerrupt status after handler sig mition*/
	/* Put this commads intot the last one*/
	if (BMI160_GET_BITSLICE(int_status[0],
		BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR))
		bmi_signification_motion_interrupt_handle(client_data);

}

static irqreturn_t bmi_irq_handler(int irq, void *handle)
{
	struct bmi_client_data *client_data = handle;

	if (client_data == NULL)
		return IRQ_HANDLED;
	if (client_data->dev == NULL)
		return IRQ_HANDLED;
	schedule_work(&client_data->irq_work);

	return IRQ_HANDLED;
}
#endif /* defined(BMI_ENABLE_INT1BMI_ENABLE_INT1)||defined(BMI_ENABLE_INT2) */

static void bmi160_set_acc_enable(struct device *dev, unsigned int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bmi_client_data *client_data = i2c_get_clientdata(client);
	int acc_pre_enable = atomic_read(&client_data->wkqueue_en);
	int gyro_current_enable;

	mutex_lock(&client_data->mutex_enable);
	if (enable) {
		if (acc_pre_enable == 0) {
			if (!client_data->power_enabled) {
				if (bmi160_power_ctl(client_data, true)) {
					dev_err(dev, "power up sensor failed.\n");
					goto mutex_exit;
				}
			}

			bmi160_set_acc_op_mode(client_data, BMI_ACC_PM_NORMAL);
			/* schedule_delayed_work(&client_data->work,
				msecs_to_jiffies(atomic_read(&client_data->delay))); */
			hrtimer_start(&client_data->accel_timer, client_data->accel_ktime, HRTIMER_MODE_REL);
			atomic_set(&client_data->wkqueue_en, 1);
		}
	} else {
		if ((acc_pre_enable == 1) && client_data->power_enabled) {
			if (!client_data->stc_enable)
				bmi160_set_acc_op_mode(client_data, BMI_ACC_PM_SUSPEND);
			/* cancel_delayed_work_sync(&client_data->work); */
			hrtimer_cancel(&client_data->accel_timer);
			cancel_work_sync(&client_data->accel_work);
			atomic_set(&client_data->wkqueue_en, 0);
			gyro_current_enable = atomic_read(&client_data->gyro_en);
			if (!gyro_current_enable && !client_data->stc_enable) {
				if (bmi160_power_ctl(client_data, false)) {
					dev_err(dev, "power down sensor failed.\n");
					goto mutex_exit;
				}
			}
		}
	}

mutex_exit:
	mutex_unlock(&client_data->mutex_enable);
	dev_notice(dev,
		"acc_enable en_state=%d\n",
		 atomic_read(&client_data->wkqueue_en));
}


static void bmi160_set_gyro_enable(struct device *dev, unsigned int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bmi_client_data *client_data = i2c_get_clientdata(client);
	int gyro_pre_enable = atomic_read(&client_data->gyro_en);
	int acc_current_enable;

	pr_info("%s power_enabled=%d pre_enable=%d enable=%d\n",
		__func__, client_data->power_enabled, gyro_pre_enable, enable);

	mutex_lock(&client_data->mutex_enable);
	if (enable) {
		if (gyro_pre_enable == 0) {
			if (!client_data->power_enabled) {
				if (bmi160_power_ctl(client_data, true)) {
					dev_err(dev, "power up sensor failed.\n");
					goto mutex_exit;
				}
			}
			bmi160_set_gyro_op_mode(client_data, BMI_GYRO_PM_NORMAL);
			/* schedule_delayed_work(&client_data->gyro_work,
				msecs_to_jiffies(atomic_read(&client_data->gyro_delay))); */
			hrtimer_start(&client_data->gyro_timer, client_data->gyro_ktime, HRTIMER_MODE_REL);
			atomic_set(&client_data->gyro_en, 1);
		}
	} else {
		if ((gyro_pre_enable == 1) && client_data->power_enabled) {
			bmi160_set_gyro_op_mode(client_data, BMI_GYRO_PM_SUSPEND);
			/* cancel_delayed_work_sync(&client_data->gyro_work); */
			hrtimer_cancel(&client_data->gyro_timer);
			cancel_work_sync(&client_data->gyro_work);
			atomic_set(&client_data->gyro_en, 0);
			acc_current_enable = atomic_read(&client_data->wkqueue_en);
			if (!acc_current_enable && !client_data->stc_enable) {
				if (bmi160_power_ctl(client_data, false)) {
					dev_err(dev, "power down sensor failed.\n");
					goto mutex_exit;
				}
			}
		}
	}

mutex_exit:
	mutex_unlock(&client_data->mutex_enable);
	/* dev_notice(&client->dev, "gyro_enable en_state=%d\n", */
	/* atomic_read(&client_data->gyro_en)); */
}

static int bmi160_accel_poll_delay(struct sensors_classdev *sensors_cdev,
				unsigned int delay_ms)
{
	struct bmi_client_data *data = container_of(sensors_cdev,
					struct bmi_client_data, accel_cdev);

	if (delay_ms < 10)
		delay_ms = 10;

	/* atomic_set(&data->delay, delay_ms); */
	data->accel_delay = delay_ms;
	data->accel_ktime = ktime_set(0, MS_TO_NS(delay_ms));
	return 0;
}

static int bmi160_cdev_enable_accel(struct sensors_classdev *sensors_cdev,
				 unsigned int enable)
{
	struct bmi_client_data *client_data = container_of(sensors_cdev,
					struct bmi_client_data, accel_cdev);
	bmi160_set_acc_enable(&client_data->i2c->dev, enable);
	return 0;
}

static int bmi160_gyro_poll_delay(struct sensors_classdev *sensors_cdev,
				unsigned int delay_ms)
{
	struct bmi_client_data *data = container_of(sensors_cdev,
					struct bmi_client_data, gyro_cdev);

	if (delay_ms < 10)
		delay_ms = 10;

	/* atomic_set(&data->gyro_delay, delay_ms); */
	data->gyro_delay = delay_ms;
	data->gyro_ktime = ktime_set(0, MS_TO_NS(delay_ms));
	return 0;
}


static int bmi160_cdev_enable_gyro(struct sensors_classdev *sensors_cdev,
				 unsigned int enable)
{
	struct bmi_client_data *client_data = container_of(sensors_cdev,
					struct bmi_client_data, gyro_cdev);
	bmi160_set_gyro_enable(&client_data->i2c->dev, enable);
	return 0;
}
static int bmi160_stepcounter_poll_delay(struct sensors_classdev *sensors_cdev,
				unsigned int delay_ms)
{
	struct bmi_client_data *data = container_of(sensors_cdev,
					struct bmi_client_data, stepcounter_cdev);

	if (delay_ms < 10)
		delay_ms = 10;

	/* atomic_set(&data->stepcounter_delay, delay_ms); */
	data->stepcounter_delay = delay_ms;
	data->stepcounter_ktime = ktime_set(0, MS_TO_NS(delay_ms));
	return 0;
}

static int bmi160_cdev_enable_stepcounter(struct sensors_classdev *sensors_cdev,
				 unsigned int enable)
{
	struct bmi_client_data *client_data = container_of(sensors_cdev,
					struct bmi_client_data, stepcounter_cdev);
	int err;

	pr_info("%s current_stc_enable=%d enable=%d\n", __func__, client_data->stc_enable, enable);

	if (enable && !client_data->stc_enable) {
		if (!client_data->power_enabled) {
			err = bmi160_power_ctl(client_data, true);
			if (err) {
				pr_info("%s power up sensor failed.\n", __func__);
				return err;
			}
			bmi160_set_acc_op_mode(client_data, BMI_ACC_PM_NORMAL);
		}
		err = BMI_CALL_API(set_step_counter_enable)(1);
		if (err < 0)
			pr_info("%s enable stepcounter failed!\n", __func__);

		/* schedule_delayed_work(&client_data->stepcounter_work,
			msecs_to_jiffies(atomic_read(&client_data->gyro_delay))); */
		hrtimer_start(&client_data->stepcounter_timer, client_data->stepcounter_ktime, HRTIMER_MODE_REL);
		client_data->stc_enable = 1;
	} else if (!enable && client_data->stc_enable) {
		err = BMI_CALL_API(set_step_counter_enable)(0);
		if (err < 0)
			pr_info("%s disable stepcounter failed!\n", __func__);

		/* cancel_delayed_work_sync(&client_data->stepcounter_work); */
		hrtimer_cancel(&client_data->stepcounter_timer);
		cancel_work_sync(&client_data->stepcounter_work);
		client_data->stc_enable = 0;
		if (!atomic_read(&client_data->wkqueue_en))
			bmi160_set_acc_op_mode(client_data, BMI_ACC_PM_SUSPEND);
		if (!atomic_read(&client_data->gyro_en) && !atomic_read(&client_data->wkqueue_en)) {
			err = bmi160_power_ctl(client_data, false);
			if (err) {
				pr_info("%s power down sensor failed.\n", __func__);
				return err;
			}
		}
	} else
		pr_info("%s: Do Nothing!\n", __func__);

	return err;
}

static int bmi160_accel_calibrate(struct sensors_classdev *sensors_cdev, int axis, int apply_now)
{
	struct bmi_client_data *client_data = container_of(sensors_cdev, struct bmi_client_data, accel_cdev);
	int i, sum[3] = {0}, avg[3] = {0};
	struct bmi160_accel_t data;
	struct bmi160_axis_data_t bmi160_udata;
	int err, pre_enable;
	int discard_num = 5;
	int ret;

	pre_enable = atomic_read(&client_data->wkqueue_en);
	if (!pre_enable)
		bmi160_set_acc_enable(&client_data->i2c->dev, 1);
	mdelay(10*discard_num);

	for (i = 0; i < 10; i++) {
		err = BMI_CALL_API(read_accel_xyz)(&data);
		if (err < 0) {
			pr_info("%s I2C read accel raw data error!\n", __func__);
			ret = -1;
			return ret;
		}

		bmi160_udata.x = data.x;
		bmi160_udata.y = data.y;
		bmi160_udata.z = data.z;
		bmi_remap_sensor_data(&bmi160_udata, client_data);
		pr_info("%s %d: x:%d y:%d z:%d\n", __func__, i, bmi160_udata.x, bmi160_udata.y, bmi160_udata.z);

		sum[0] += bmi160_udata.x;
		sum[1] += bmi160_udata.y;
		sum[2] += bmi160_udata.z;
		mdelay(10);
	}

	avg[0] = sum[0] / 10;
	avg[1] = sum[1] / 10;
	avg[2] = sum[2] / 10;

	pr_info("%s accel avg data x:%d y:%d z:%d\n", __func__, avg[0], avg[1], avg[2]);

	if ((avg[0] > ACC_CAL_XY_RANGE_MAX || avg[0] < ACC_CAL_XY_RANGE_MIN) ||
		(avg[1] > ACC_CAL_XY_RANGE_MAX || avg[1] < ACC_CAL_XY_RANGE_MIN) ||
		(avg[2] > ACC_CAL_Z_RANGE_MAX || avg[2] < ACC_CAL_Z_RANGE_MIN)) {
		pr_info("%s Please keep the machine almost horizontal when being calibrated\n", __func__);
		if (!pre_enable)
			bmi160_set_acc_enable(&client_data->i2c->dev, 0);

		client_data->acc_cal_x = 0;
		client_data->acc_cal_y = 0;
		client_data->acc_cal_z = 0;
		client_data->acc_cal_result = -1;
		memset(client_data->acc_calibrate_buf, 0, sizeof(client_data->acc_calibrate_buf));
		client_data->accel_cdev.params = client_data->acc_calibrate_buf;
		ret = -1;
		return ret;
	}

	client_data->acc_cal_x = avg[0];
	client_data->acc_cal_y = avg[1];
	client_data->acc_cal_z = avg[2] - 16384;
	client_data->acc_cal_result = 0;

	snprintf(client_data->acc_calibrate_buf, sizeof(client_data->acc_calibrate_buf), "%d,%d,%d",
		client_data->acc_cal_x, client_data->acc_cal_y, client_data->acc_cal_z);
	client_data->accel_cdev.params = client_data->acc_calibrate_buf;

	pr_info("acc cal_x:%d cal_y:%d cal_z:%d\n",
		client_data->acc_cal_x, client_data->acc_cal_y, client_data->acc_cal_z);

	if (!pre_enable)
		bmi160_set_acc_enable(&client_data->i2c->dev, 0);

	ret = 0;
	return ret;
}

static int bmi160_accel_write_calibrate(struct sensors_classdev *sensors_cdev,
		struct cal_result_t *cal_result)
{
	struct bmi_client_data *client_data = container_of(sensors_cdev, struct bmi_client_data, accel_cdev);

	client_data->acc_cal_x = cal_result->offset_x;
	client_data->acc_cal_y = cal_result->offset_y;
	client_data->acc_cal_z = cal_result->offset_z;

	pr_info("%s cal_x:%d cal_y:%d cal_z:%d\n", __func__,
		client_data->acc_cal_x, client_data->acc_cal_y, client_data->acc_cal_z);

	return 0;
}

static int bmi160_gyro_calibrate(struct sensors_classdev *sensors_cdev, int axis, int apply_now)
{
	struct bmi_client_data *client_data = container_of(sensors_cdev, struct bmi_client_data, gyro_cdev);
	int i, sum[3] = {0}, avg[3] = {0};
	struct bmi160_gyro_t data;
	struct bmi160_axis_data_t bmi160_udata;
	int err, pre_enable;
	int discard_num = 5;
	int ret;

	pre_enable = atomic_read(&client_data->gyro_en);
	if (!pre_enable)
		bmi160_set_gyro_enable(&client_data->i2c->dev, 1);
	mdelay(10*discard_num);

	for (i = 0; i < 10; i++) {
		err = BMI_CALL_API(read_gyro_xyz)(&data);
		if (err < 0) {
			pr_info("%s I2C read gyro raw data error!\n", __func__);
			ret = -1;
			return ret;
		}

		bmi160_udata.x = data.x;
		bmi160_udata.y = data.y;
		bmi160_udata.z = data.z;
		/* LSB to mdps */
		bmi160_udata.x = bmi160_udata.x*61; /* 1000000/16400; */
		bmi160_udata.y = bmi160_udata.y*61; /* 1000000/16400; */
		bmi160_udata.z = bmi160_udata.z*61; /* 1000000/16400; */
		bmi_remap_sensor_data(&bmi160_udata, client_data);
		pr_info("%s %d: x:%d y:%d z:%d\n", __func__, i, bmi160_udata.x, bmi160_udata.y, bmi160_udata.z);

		sum[0] += bmi160_udata.x;
		sum[1] += bmi160_udata.y;
		sum[2] += bmi160_udata.z;
		mdelay(10);
	}

	avg[0] = sum[0] / 10;
	avg[1] = sum[1] / 10;
	avg[2] = sum[2] / 10;

	pr_info("%s gyro avg data x:%d y:%d z:%d\n", __func__, avg[0], avg[1], avg[2]);

	if ((avg[0] > GYRO_CAL_RANGE_MAX || avg[0] < GYRO_CAL_RANGE_MIN)  ||
		(avg[1] > GYRO_CAL_RANGE_MAX || avg[1] < GYRO_CAL_RANGE_MIN) ||
		(avg[2] > GYRO_CAL_RANGE_MAX || avg[2] < GYRO_CAL_RANGE_MIN)) {
		pr_info("%s Please keep the machine almost horizontal when being calibrated\n", __func__);
		if (!pre_enable)
			bmi160_set_gyro_enable(&client_data->i2c->dev, 0);

		client_data->gyro_cal_x = 0;
		client_data->gyro_cal_y = 0;
		client_data->gyro_cal_z = 0;
		client_data->gyro_cal_result = -1;
		memset(client_data->gyro_calibrate_buf, 0, sizeof(client_data->gyro_calibrate_buf));
		client_data->gyro_cdev.params = client_data->gyro_calibrate_buf;
		ret = -1;
		return ret;
	}

	client_data->gyro_cal_x = avg[0];
	client_data->gyro_cal_y = avg[1];
	client_data->gyro_cal_z = avg[2];
	client_data->gyro_cal_result = 0;

	snprintf(client_data->gyro_calibrate_buf, sizeof(client_data->gyro_calibrate_buf), "%d,%d,%d",
		client_data->gyro_cal_x, client_data->gyro_cal_y, client_data->gyro_cal_z);
	client_data->gyro_cdev.params = client_data->gyro_calibrate_buf;

	pr_info("acc cal_x:%d cal_y:%d cal_z:%d\n",
		client_data->gyro_cal_x, client_data->gyro_cal_y, client_data->gyro_cal_z);

	if (!pre_enable)
		bmi160_set_gyro_enable(&client_data->i2c->dev, 0);

	return 0;
}

static int bmi160_gyro_write_calibrate(struct sensors_classdev *sensors_cdev,
		struct cal_result_t *cal_result)
{
	struct bmi_client_data *client_data = container_of(sensors_cdev, struct bmi_client_data, gyro_cdev);

	client_data->gyro_cal_x = cal_result->offset_x;
	client_data->gyro_cal_y = cal_result->offset_y;
	client_data->gyro_cal_z = cal_result->offset_z;

	pr_info("%s cal_x:%d cal_y:%d cal_z:%d\n", __func__,
		client_data->gyro_cal_x, client_data->gyro_cal_y, client_data->gyro_cal_z);

	return 0;
}

static void bmi160_parse_dt(struct device *dev, struct bosch_sensor_specific *bst_pd)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "bosch,place", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read min-interval\n");
	} else {
		bst_pd->place = temp_val;
	}
}

/* lijiangshuo add for proc system start */
static int bmi160_proc_show(struct seq_file *m, void *v)
{
	return seq_printf(m, "0x%x\n", chip_id);
}

static int bmi160_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, bmi160_proc_show, NULL);
}

static const struct file_operations bmi160_proc_fops = {
	.open	= bmi160_proc_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};

static void create_bmi160_info_proc_file(void)
{
	bmi160_info_proc_file_accel =
		proc_create("driver/accel", 0644, NULL, &bmi160_proc_fops);
	if (!bmi160_info_proc_file_accel) {
		pr_info("bmi160 accel proc file create failed!\n");
	}

	bmi160_info_proc_file_gyro =
		proc_create("driver/gyro", 0644, NULL, &bmi160_proc_fops);
	if (!bmi160_info_proc_file_gyro) {
		pr_info("bmi160 gyro proc file create failed!\n");
	}
}

static void remove_bmi160_info_proc_file(void)
{
	if (bmi160_info_proc_file_accel) {
		remove_proc_entry("driver/accel", NULL);
		bmi160_info_proc_file_accel = NULL;
	}
	if (bmi160_info_proc_file_gyro) {
		remove_proc_entry("driver/gyro", NULL);
		bmi160_info_proc_file_gyro = NULL;
	}
}
/* lijiangshuo add for proc system end */

enum hrtimer_restart bmi160_accel_timer_function(struct hrtimer *timer)
{
	struct bmi_client_data *client_data =
		container_of((struct hrtimer *)timer, struct bmi_client_data, accel_timer);
	queue_work(bmi160_workqueue, &client_data->accel_work);
	return HRTIMER_NORESTART;
}

enum hrtimer_restart bmi160_gyro_timer_function(struct hrtimer *timer)
{
	struct bmi_client_data *client_data =
		container_of((struct hrtimer *)timer, struct bmi_client_data, gyro_timer);
	queue_work(bmi160_workqueue, &client_data->gyro_work);
	return HRTIMER_NORESTART;
}

enum hrtimer_restart bmi160_stepcounter_timer_function(struct hrtimer *timer)
{
	struct bmi_client_data *client_data = container_of((struct hrtimer *)timer,
		struct bmi_client_data, stepcounter_timer);
	queue_work(bmi160_workqueue, &client_data->stepcounter_work);
	return HRTIMER_NORESTART;
}

int bmi_probe(struct bmi_client_data *client_data, struct device *dev)
{
	int err = 0;
#ifdef BMI160_MAG_INTERFACE_SUPPORT
	u8 mag_dev_addr;
	u8 mag_urst_len;
	u8 mag_op_mode;
#endif
	err = bmi160_power_init(client_data);
	if (err) {
		dev_err(&client_data->i2c->dev,
			 "Failed to get sensor regulators\n");
		err = -EINVAL;
		goto exit_err_clean;
	}
	err = bmi160_power_ctl(client_data, true);
	if (err) {
		dev_err(&client_data->i2c->dev,
			 "Failed to enable sensor power\n");
		err = -EINVAL;
		goto deinit_power_exit;
	}

	/* check chip id */
	err = bmi_check_chip_id(client_data);
	if (err)
		goto disable_power_exit;

	dev_set_drvdata(dev, client_data);
	client_data->dev = dev;

	mutex_init(&client_data->mutex_enable);
	mutex_init(&client_data->mutex_op_mode);
	mutex_init(&client_data->mutex_ring_buf);

	/* input device init */
	err = bmi_input_init(client_data);
	if (err < 0)
		goto disable_power_exit;

	/*to do*/
	client_data->accel_cdev = accel_cdev;
	client_data->accel_cdev.sensors_enable = bmi160_cdev_enable_accel;
	client_data->accel_cdev.sensors_poll_delay = bmi160_accel_poll_delay;
	client_data->accel_cdev.sensors_calibrate = bmi160_accel_calibrate;
	client_data->accel_cdev.sensors_write_cal_params =
		bmi160_accel_write_calibrate;
	err = sensors_classdev_register(&client_data->input_accel->dev,
			 &client_data->accel_cdev);
	if (err) {
		dev_err(&client_data->i2c->dev, "sensors class register failed.\n");
		return err;
	}

	client_data->gyro_cdev = gyro_cdev;
	client_data->gyro_cdev.sensors_enable = bmi160_cdev_enable_gyro;
	client_data->gyro_cdev.sensors_poll_delay = bmi160_gyro_poll_delay;
	client_data->gyro_cdev.sensors_calibrate = bmi160_gyro_calibrate;
	client_data->gyro_cdev.sensors_write_cal_params = bmi160_gyro_write_calibrate;
	err = sensors_classdev_register(&client_data->input_gyro->dev,
			&client_data->gyro_cdev);
	if (err) {
		dev_err(&client_data->i2c->dev, "sensors class register failed.\n");
		return err;
	}

	client_data->stepcounter_cdev = stepcounter_cdev;
	client_data->stepcounter_cdev.sensors_enable = bmi160_cdev_enable_stepcounter;
	client_data->stepcounter_cdev.sensors_poll_delay =
		bmi160_stepcounter_poll_delay;
	err = sensors_classdev_register(&client_data->input_stepcounter->dev,
			&client_data->stepcounter_cdev);
	if (err) {
		dev_err(&client_data->i2c->dev, "sensors class register failed.\n");
		return err;
	}
	if (dev->of_node != NULL) {
		client_data->bst_pd = kzalloc(sizeof(*client_data->bst_pd),
				GFP_KERNEL);
		bmi160_parse_dt(dev, client_data->bst_pd);
	}

	/* workqueue init */
	client_data->accel_delay = BMI_DELAY_DEFAULT;
	client_data->gyro_delay = BMI_DELAY_DEFAULT;
	client_data->stepcounter_delay = BMI_DELAY_DEFAULT;
	bmi160_workqueue = create_workqueue("bmi160_workqueue");
	hrtimer_init(&client_data->accel_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrtimer_init(&client_data->gyro_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrtimer_init(&client_data->stepcounter_timer, CLOCK_MONOTONIC,
		HRTIMER_MODE_REL);
	client_data->accel_timer.function = &bmi160_accel_timer_function;
	client_data->gyro_timer.function = &bmi160_gyro_timer_function;
	client_data->stepcounter_timer.function = &bmi160_stepcounter_timer_function;
	client_data->accel_ktime = ktime_set(0, MS_TO_NS(client_data->accel_delay));
	client_data->gyro_ktime = ktime_set(0, MS_TO_NS(client_data->gyro_delay));
	client_data->stepcounter_ktime = ktime_set(0,
		MS_TO_NS(client_data->stepcounter_delay));
	INIT_WORK(&client_data->accel_work, bmi_work_func);
	INIT_WORK(&client_data->gyro_work, bmi_gyro_work_func);
	INIT_WORK(&client_data->stepcounter_work, bmi_stepcounter_work_func);
	atomic_set(&client_data->wkqueue_en, 0);
	atomic_set(&client_data->gyro_en, 0);
	client_data->stc_enable = 0;
	client_data->acc_cal_x = 0;
	client_data->acc_cal_y = 0;
	client_data->acc_cal_z = 0;
	client_data->acc_cal_result = -1;
	memset(client_data->acc_calibrate_buf, 0,
		sizeof(client_data->acc_calibrate_buf));
	client_data->gyro_cal_x = 0;
	client_data->gyro_cal_y = 0;
	client_data->gyro_cal_z = 0;
	client_data->gyro_cal_result = -1;
	memset(client_data->gyro_calibrate_buf, 0,
		sizeof(client_data->gyro_calibrate_buf));

	/* h/w init */
	client_data->device.delay_msec = bmi_delay;
	err = BMI_CALL_API(init)(&client_data->device);

	/*workqueue init*/
	INIT_WORK(&client_data->report_data_work, bmi_hrtimer_work_func);
	reportdata_wq = create_singlethread_workqueue("bmi160_wq");
	if (reportdata_wq == NULL) {
		pr_info("fail to create the reportdta_wq %d", -ENOMEM);
	}
	hrtimer_init(&client_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	client_data->timer.function = reportdata_timer_fun;
	client_data->work_delay_kt = ns_to_ktime(10000000);
	client_data->is_timer_running = 0;
	client_data->time_odr = 500000; /*200Hz*/

	bmi_dump_reg(client_data);

	/*power on detected*/
	/*or softrest(cmd 0xB6) */
	/*fatal err check*/
	/*soft reset*/
	err += BMI_CALL_API(set_command_register)(CMD_RESET_USER_REG);
	mdelay(3);
	if (err)
		dev_err(dev, "Failed soft reset, er=%d", err);
	/*usr data config page*/
	err += BMI_CALL_API(set_target_page)(USER_DAT_CFG_PAGE);
	if (err)
		dev_err(dev, "Failed cffg page, er=%d", err);
	err += bmi_get_err_status(client_data);
	if (err) {
		dev_err(dev, "Failed to bmi16x init!err_st=0x%x\n",
				client_data->err_st.err_st_all);
		goto exit_err_sysfs;
	}

#ifdef BMI160_MAG_INTERFACE_SUPPORT
	err += bmi160_set_command_register(MAG_MODE_NORMAL);
	mdelay(2);
	err += bmi160_get_mag_power_mode_stat(&mag_op_mode);
	mdelay(2);
	err += BMI_CALL_API(get_i2c_device_addr)(&mag_dev_addr);
	mdelay(2);
#ifdef BMI160_AKM09912_SUPPORT
	err += BMI_CALL_API(set_i2c_device_addr)(BMI160_AKM09912_I2C_ADDRESS);
	/*do not need to check the return value for mag 2nd interface*/
	bmi160_bst_akm_mag_interface_init(BMI160_AKM09912_I2C_ADDRESS);
#else
	err += BMI_CALL_API(set_i2c_device_addr)(BMI160_AUX_BMM150_I2C_ADDRESS);
	/*do not need to check the return value for mag 2nd interface*/
	bmi160_bmm150_mag_interface_init();
#endif
	err += bmi160_set_mag_burst(3);
	err += bmi160_get_mag_burst(&mag_urst_len);
	dev_info(client_data->dev,
		"BMI160 mag_urst_len:%d, mag_add:0x%x, mag_op_mode:%d\n",
		mag_urst_len, mag_dev_addr, mag_op_mode);
#endif
	if (err < 0)
		goto exit_err_sysfs;

#ifdef BMI160_ENABLE_INT1
	client_data->gpio_pin = of_get_named_gpio_flags(dev->of_node,
				"bosch,gpio-int1", 0, NULL);
	dev_info(client_data->dev, "BMI160 qpio number:%d\n",
				client_data->gpio_pin);
	err += gpio_request_one(client_data->gpio_pin,
				GPIOF_IN, "bmi160_int");
	err += gpio_direction_input(client_data->gpio_pin);
	client_data->IRQ = gpio_to_irq(client_data->gpio_pin);
	if (err) {
		dev_err(client_data->dev,
			"can not request gpio to irq number\n");
		client_data->gpio_pin = 0;
	}
	/* maps interrupt to INT1/InT2 pin */
	BMI_CALL_API(set_intr_any_motion)(BMI_INT0, ENABLE);
	BMI_CALL_API(set_intr_fifo_wm)(BMI_INT0, ENABLE);
	/*BMI_CALL_API(set_int_drdy)(BMI_INT0, ENABLE);*/

	/*Set interrupt trige level way */
	BMI_CALL_API(set_intr_edge_ctrl)(BMI_INT0, BMI_INT_LEVEL);
	bmi160_set_intr_level(BMI_INT0, 1);
	/*set interrupt latch temporary, 5 ms*/
	/*bmi160_set_latch_int(5);*/

	BMI_CALL_API(set_output_enable)(
	BMI160_INTR1_OUTPUT_ENABLE, ENABLE);
	sigmotion_init_interrupts(BMI160_MAP_INTR1);
	BMI_CALL_API(map_step_detector_intr)(BMI160_MAP_INTR1);
	/*close step_detector in init function*/
	BMI_CALL_API(set_step_detector_enable)(0);
#endif

#ifdef BMI160_ENABLE_INT2
	client_data->gpio_pin = of_get_named_gpio_flags(dev->of_node,
				"bosch,gpio-int2", 0, NULL);
	dev_info(client_data->dev, "BMI160 qpio number:%d\n",
				client_data->gpio_pin);
	err += gpio_request_one(client_data->gpio_pin,
					GPIOF_IN, "bmi160_int");
	err += gpio_direction_input(client_data->gpio_pin);
	client_data->IRQ = gpio_to_irq(client_data->gpio_pin);
	if (err) {
		dev_err(client_data->dev,
			"can not request gpio to irq number\n");
		client_data->gpio_pin = 0;
	}
	/* maps interrupt to INT1/InT2 pin */
	BMI_CALL_API(set_intr_any_motion)(BMI_INT1, ENABLE);
	BMI_CALL_API(set_intr_fifo_wm)(BMI_INT1, ENABLE);
	BMI_CALL_API(set_int_drdy)(BMI_INT1, ENABLE);

	/*Set interrupt trige level way */
	BMI_CALL_API(set_intr_edge_ctrl)(BMI_INT1, BMI_INT_LEVEL);
	bmi160_set_intr_level(BMI_INT1, 1);
	/*set interrupt latch temporary, 5 ms*/
	/*bmi160_set_latch_int(5);*/

	BMI_CALL_API(set_output_enable)(
	BMI160_INTR2_OUTPUT_ENABLE, ENABLE);
	sigmotion_init_interrupts(BMI160_MAP_INTR2);
	BMI_CALL_API(map_step_detector_intr)(BMI160_MAP_INTR2);
	/*close step_detector in init function*/
	BMI_CALL_API(set_step_detector_enable)(0);
#endif

#if defined(BMI160_ENABLE_INT1) || defined(BMI160_ENABLE_INT2)
	err = request_irq(client_data->IRQ, bmi_irq_handler,
			IRQF_TRIGGER_RISING, "bmi160", client_data);
	if (err)
		dev_err(client_data->dev, "could not request irq\n");
	INIT_WORK(&client_data->irq_work, bmi_irq_work_func);
#endif

	client_data->selftest = 0;

	client_data->fifo_data_sel = 0;
	BMI_CALL_API(get_accel_output_data_rate)(&client_data->odr.acc_odr);
	BMI_CALL_API(get_gyro_output_data_rate)(&client_data->odr.gyro_odr);
	BMI_CALL_API(get_mag_output_data_rate)(&client_data->odr.mag_odr);
	BMI_CALL_API(set_fifo_time_enable)(1);
	BMI_CALL_API(get_accel_range)(&client_data->range.acc_range);
	BMI_CALL_API(get_gyro_range)(&client_data->range.gyro_range);
	/* now it's power on which is considered as resuming from suspend */

	/* set sensor PMU into suspend power mode for all */
	if (bmi_pmu_set_suspend(client_data) < 0) {
		dev_err(dev, "Failed to set BMI160 to suspend power mode\n");
		goto exit_err_sysfs;
	}

	bmi160_power_ctl(client_data, false);

	dev_notice(dev, "sensor_time:%d, %d",
		sensortime_duration_tbl[0].ts_delat,
		sensortime_duration_tbl[0].ts_duration_lsb);
	dev_notice(dev, "sensor %s probed successfully", SENSOR_NAME);

/* lijiangshuo add for proc system start */
	if (chip_id != 0xff)
		create_bmi160_info_proc_file();
/* lijiangshuo add for proc system end */

	return 0;

exit_err_sysfs:
	if (err)
		bmi_input_destroy(client_data);
disable_power_exit:
	bmi160_power_ctl(client_data, false);
deinit_power_exit:
	bmi160_power_deinit(client_data);
exit_err_clean:
	if (err) {
		if (client_data != NULL) {
			if (client_data->bst_pd != NULL) {
				kfree(client_data->bst_pd);
				client_data->bst_pd = NULL;
			}
		}
	}

	return err;
}
EXPORT_SYMBOL(bmi_probe);

/*!
 * @brief remove bmi client
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
int bmi_remove(struct device *dev)
{
	int err = 0;
	struct bmi_client_data *client_data = dev_get_drvdata(dev);

	if (client_data != NULL) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&client_data->early_suspend_handler);
#endif
		mutex_lock(&client_data->mutex_enable);
		if (client_data->pw.acc_pm == BMI_ACC_PM_NORMAL ||
			client_data->pw.gyro_pm == BMI_GYRO_PM_NORMAL ||
				client_data->pw.mag_pm == BMI_MAG_PM_NORMAL) {
			/* cancel_delayed_work_sync(&client_data->work); */
			hrtimer_cancel(&client_data->accel_timer);
			cancel_work_sync(&client_data->accel_work);
		}
		mutex_unlock(&client_data->mutex_enable);

		err = bmi_pmu_set_suspend(client_data);

		mdelay(5);

		bmi_input_destroy(client_data);

		if (client_data->bst_pd != NULL) {
			kfree(client_data->bst_pd);
			client_data->bst_pd = NULL;
		}
	kfree(client_data);
	}

	if (!bmi160_workqueue) {
		flush_workqueue(bmi160_workqueue);
		destroy_workqueue(bmi160_workqueue);
	}

/* lijiangshuo add for proc system start */
	if (chip_id != 0xff)
		remove_bmi160_info_proc_file();
/* lijiangshuo add for proc system end */

	return err;
}
EXPORT_SYMBOL(bmi_remove);

static int bmi_post_resume(struct bmi_client_data *client_data)
{
	int err = 0;

	mutex_lock(&client_data->mutex_enable);

	if (atomic_read(&client_data->wkqueue_en) == 1) {
		bmi160_set_acc_op_mode(client_data, BMI_ACC_PM_NORMAL);
		hrtimer_start(&client_data->accel_timer, client_data->accel_ktime,
			HRTIMER_MODE_REL);
	}
	mutex_unlock(&client_data->mutex_enable);

	if (client_data->stc_enable == 1)
		hrtimer_start(&client_data->stepcounter_timer,
			client_data->stepcounter_ktime, HRTIMER_MODE_REL);

	if (client_data->is_timer_running) {
		hrtimer_start(&client_data->timer, ns_to_ktime(client_data->time_odr),
			HRTIMER_MODE_REL);
		client_data->base_time = 0;
		client_data->timestamp = 0;
		client_data->is_timer_running = 1;
	}

	return err;
}


int bmi_suspend(struct device *dev)
{
	int err = 0;
	struct bmi_client_data *client_data = dev_get_drvdata(dev);
	unsigned char stc_enable;
	unsigned char std_enable;

	dev_err(client_data->dev, "bmi suspend function entrance");

	if (client_data->is_timer_running) {
		hrtimer_cancel(&client_data->timer);
		client_data->base_time = 0;
		client_data->timestamp = 0;
		client_data->fifo_time = 0;
	}

	if (atomic_read(&client_data->wkqueue_en) == 1) {
		bmi160_set_acc_op_mode(client_data, BMI_ACC_PM_SUSPEND);
		/* cancel_delayed_work_sync(&client_data->work); */
		hrtimer_cancel(&client_data->accel_timer);
		cancel_work_sync(&client_data->accel_work);
	}

	if (client_data->stc_enable == 1) {
		/* cancel_delayed_work_sync(&client_data->stepcounter_work); */
		hrtimer_cancel(&client_data->stepcounter_timer);
		cancel_work_sync(&client_data->stepcounter_work);
	}

	BMI_CALL_API(get_step_counter_enable)(&stc_enable);
	BMI_CALL_API(get_step_detector_enable)(&std_enable);
	if (client_data->pw.acc_pm != BMI_ACC_PM_SUSPEND &&
		(stc_enable != 1) && (std_enable != 1) &&
		(client_data->sig_flag != 1)) {
		err += BMI_CALL_API(set_command_register)
				(bmi_pmu_cmd_acc_arr[BMI_ACC_PM_SUSPEND]);
		/*client_data->pw.acc_pm = BMI_ACC_PM_SUSPEND;*/
		mdelay(3);
	}
	if (client_data->pw.gyro_pm != BMI_GYRO_PM_SUSPEND) {
		err += BMI_CALL_API(set_command_register)
				(bmi_pmu_cmd_gyro_arr[BMI_GYRO_PM_SUSPEND]);
		/*client_data->pw.gyro_pm = BMI_GYRO_PM_SUSPEND;*/
		mdelay(3);
	}

	if (client_data->pw.mag_pm != BMI_MAG_PM_SUSPEND) {
#ifdef BMI160_AKM09912_SUPPORT
		err += bmi160_set_bst_akm_and_secondary_if_powermode
					(BMI160_MAG_SUSPEND_MODE);
#else
		err += bmi160_set_bmm150_mag_and_secondary_if_power_mode
					(BMI160_MAG_SUSPEND_MODE);
#endif
		/*client_data->pw.gyro_pm = BMI160_MAG_SUSPEND_MODE;*/
		mdelay(3);
	}

	return err;
}
EXPORT_SYMBOL(bmi_suspend);

int bmi_resume(struct device *dev)
{
	int err = 0;
	struct bmi_client_data *client_data = dev_get_drvdata(dev);

	dev_err(client_data->dev, "bmi resume function entrance");

	if (client_data->pw.acc_pm != BMI_ACC_PM_SUSPEND) {
		err += BMI_CALL_API(set_command_register)
				(bmi_pmu_cmd_acc_arr[BMI_ACC_PM_NORMAL]);
		mdelay(3);
	}
	if (client_data->pw.gyro_pm != BMI_GYRO_PM_SUSPEND) {
		err += BMI_CALL_API(set_command_register)
				(bmi_pmu_cmd_gyro_arr[BMI_GYRO_PM_NORMAL]);
		mdelay(3);
	}

	if (client_data->pw.mag_pm != BMI_MAG_PM_SUSPEND) {
#ifdef BMI160_AKM09912_SUPPORT
		err += bmi160_set_bst_akm_and_secondary_if_powermode
					(BMI160_MAG_FORCE_MODE);
#else
		err += bmi160_set_bmm150_mag_and_secondary_if_power_mode
					(BMI160_MAG_FORCE_MODE);
#endif
		mdelay(3);
	}
	/* post resume operation */
	err += bmi_post_resume(client_data);

	return err;
}
EXPORT_SYMBOL(bmi_resume);

