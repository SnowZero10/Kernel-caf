/*
 * FILE:__TSP_FW_CLASS_H_INCLUDED
 *
 */
#ifndef __TPD_FW_H_INCLUDED
#define __TPD_FW_H_INCLUDED

#include <linux/device.h>
#include <linux/rwsem.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/proc_fs.h>
#include <linux/printk.h>

#ifdef TPD_DMESG
#undef TPD_DMESG
#endif
#define TPD_DMESG(a, arg...) pr_notice("tpd: " a, ##arg)

#ifdef CONFIG_ZTE_TPD_SYS
#define CONFIG_CREATE_TPD_SYS_INTERFACE
#endif

#define CMD_WRITE_IMAGE	1
#define CMD_WRITE_FINISH	2
#define CMD_CLEAN_BUF	3
#define CMD_READ_IMAGE	4
#define CMD_PERFORM_UPG	5
#define CMD_PORCE_UPG	6
#define CMD_SET_PART_ID	7
#define CMD_COMPARE_FIRMWARE	8

#define STATUS_BUF_NULL	1
#define STATUS_BUF_ING	2
#define STATUS_BUF_RDY	3

#define STATUS_UPG_ING	4
#define STATUS_UPG_SUCC	5
#define STATUS_UPG_FAILED	6
#define STATUS_UPG_NONEED	7
#define STATUS_FILE_FAILED	8

#define STATUS_NULL   STATUS_BUF_NULL
#define STATUS_OK	STATUS_UPG_SUCC

/*gesture for tp, begin*/
#define  KEY_GESTURE_DOUBLE_CLICK 214

/**#define  KEY_GESTURE_UP 218
#define  KEY_GESTURE_DOWN 216
#define  KEY_GESTURE_LEFT 222
#define  KEY_GESTURE_RIGHT 223*/

/*#define  KEY_GESTURE_C 219
#define  KEY_GESTURE_E 220
#define  KEY_GESTURE_M 221
#define  KEY_GESTURE_L KEY_RESERVED
#define  KEY_GESTURE_O 230
#define  KEY_GESTURE_S 227
#define  KEY_GESTURE_W 228
#define  KEY_GESTURE_U KEY_RESERVED
#define  KEY_GESTURE_V KEY_RESERVED
#define  KEY_GESTURE_Z KEY_RESERVED

#define KEY_UNLOCK		249
#define KEY_MESSAGE		250
#define KEY_UNDEFINE_S		251
#define KEY_UNDEFINE_W		252
#define KEY_POWER_ON		253
#define KEY_UNDEFINE_O		254	*/
/*gesture for tp, end*/

struct tpvendor_t {
	int vendor_id;
	char *vendor_name;
};
/*chip_model_id synaptics 1,atmel 2,cypress 3,focal 4,goodix 5,mefals 6,mstar
7,himax 8;
 *
 */
struct tpd_tpinfo_t {
	unsigned int chip_model_id;
	unsigned int chip_part_id;
	unsigned int chip_ver;
	unsigned int module_id;
	unsigned int firmware_ver;
	unsigned int config_ver;
	unsigned int i2c_addr;
	unsigned int i2c_type;
	char tp_name[20];
	char vendor_name[20];
};

struct firmware_t {
	unsigned char *data;
	unsigned int data_len;
	unsigned int length;
};

struct tpd_classdev_t {
	const char *name;
	int cmd;
	int size;
	int status;
	int b_fwloader;
	int b_film_mode_enable;
	int b_force_upgrade;
	int fw_compare_result;
	int b_gesture_enable;
	bool TP_have_registered;

	int (*read_block)(struct tpd_classdev_t *cdev, u16 addr, u8 *buf, int len);
	int (*write_block)(struct tpd_classdev_t *cdev, u16 addr, u8 *buf, int len);
	int (*flash_fw)(struct tpd_classdev_t *cdev, unsigned char *data, unsigned int size, int force_upg);
	int (*compare_tp_version)(struct tpd_classdev_t *cdev, unsigned char *data);
	int (*wake_film_mode)(struct tpd_classdev_t *cdev, int enable);
	int (*get_tpinfo)(struct tpd_classdev_t *cdev);
	int (*get_gesture)(struct tpd_classdev_t *cdev);
	int (*wake_gesture)(struct tpd_classdev_t *cdev, int enable);
	void *private;
	void *test_node;    /**added for tp test.*/

	/**for tpd test*/
	int (*tpd_test_set_save_filepath)(struct tpd_classdev_t *cdev, const char *buf);
	int (*tpd_test_get_save_filepath)(struct tpd_classdev_t *cdev, char *buf);
	int (*tpd_test_set_ini_filepath)(struct tpd_classdev_t *cdev, const char *buf);
	int (*tpd_test_get_ini_filepath)(struct tpd_classdev_t *cdev, char *buf);
	int (*tpd_test_set_filename)(struct tpd_classdev_t *cdev, const char *buf);
	int (*tpd_test_get_filename)(struct tpd_classdev_t *cdev, char *buf);
	int (*tpd_test_set_cmd)(struct tpd_classdev_t *cdev, const char *buf);
	int (*tpd_test_get_cmd)(struct tpd_classdev_t *cdev, char *buf);
	int (*tpd_test_set_node_data_type)(struct tpd_classdev_t *cdev, const char *buf);
	int (*tpd_test_get_node_data)(struct tpd_classdev_t *cdev, char *buf);
	int (*tpd_test_get_channel_info)(struct tpd_classdev_t *cdev, char *buf);
	int (*tpd_test_get_result)(struct tpd_classdev_t *cdev, char *buf);

	struct mutex upgrade_mutex;
	struct mutex cmd_mutex;

	struct tpd_tpinfo_t ic_tpinfo;
	struct tpd_tpinfo_t file_tpinfo;
	struct firmware_t tp_fw;
	struct workqueue_struct *tpd_upgrade_wq;
	struct work_struct  tpd_upgrade_w;

	char lcm_info[64];
	char lcm_chip_info[64];

	struct device		*dev;
	struct list_head	 node;
};

extern int tpd_power_already_on;
extern struct tpd_classdev_t tpd_fw_cdev;
extern int tpd_classdev_register(struct device *parent, struct tpd_classdev_t
*tsp_fw_cdev);
#endif	/* __TSP_FW_CLASS_H_INCLUDED */

