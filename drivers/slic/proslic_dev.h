#ifndef _PROSLIC_DEV_H
#define _PROSLIC_DEV_H

#include <linux/slic.h>
#include "proslic_api.h"
#include "proslic_config.h"

#define FSK_WAIT_TIME           (12)
#define FSK_DEPTH_TRIG          (1)

#ifdef CONFIG_PROSLIC_RST_LOW
#define SLIC_RESET_ENABLE       (0)
#define SLIC_RESET_DISABLE      (1)
#else
#define SLIC_RESET_ENABLE       (1)
#define SLIC_RESET_DISABLE      (0)
#endif

struct proslic_status {
	unsigned char   hook_event;
	int             hook_int;
	unsigned long   onhook_timestamp;
	int             key_int;
	unsigned char   key_event;
	int             fsk_int;
	int             fsk_err_num;
	int             fsk_irq_num;
	vdaaRingDetectStatusType vdaa_status;
};

struct proslic_work {
	struct proslic_dev *proslic;
	struct delayed_work work;
	int irq_err;
};

struct tone_work {
	struct delayed_work work;
	struct proslic_dev *proslic;
	struct tone_set *tone;
	int curr_list;
	int curr_times;
};

struct proslic_dev {
	struct slic_dev         sdev;
	struct spi_device       *spi;
	controlInterfaceType    *hw;
	proslicDeviceType_ptr   device;
	proslicChanType_ptr     chan;
	struct proslic_status   state;
	struct delayed_work     hook_work;
	struct delayed_work     dtmf_work;
	struct tone_work        tone_work;
	atomic_t                cid_ack;
	wait_queue_head_t       fsk_wait;
	wait_queue_head_t       ack_wait;
	struct mutex            lock;
	struct fxs_config       fxs_config;
	struct fxo_config       fxo_config;
	int                     num_slic;
	int                     irq_gpio;
	int                     irq_gpio_cp;
	int                     rst_gpio;
	int                     type;
	int                     initialized;
};
#endif
