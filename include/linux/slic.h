#ifndef _SLIC_H
#define _SLIC_H

#include <linux/device.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/spinlock.h>

#define SLIC_IOC_MAGIC              0xF3
#define SLIC_PCM_START              _IO(SLIC_IOC_MAGIC, 0)
#define SLIC_PCM_STOP               _IO(SLIC_IOC_MAGIC, 1)
#define SLIC_TONE_START             _IO(SLIC_IOC_MAGIC, 2)
#define SLIC_TONE_STOP              _IO(SLIC_IOC_MAGIC, 3)
#define SLIC_RING_START             _IO(SLIC_IOC_MAGIC, 4)
#define SLIC_RING_STOP              _IO(SLIC_IOC_MAGIC, 5)
#define SLIC_CID_OFFHOOK            _IO(SLIC_IOC_MAGIC, 6)
#define SLIC_VM_START               _IO(SLIC_IOC_MAGIC, 7)
#define SLIC_VM_STOP                _IO(SLIC_IOC_MAGIC, 8)
#define SLIC_SET_TX_MUTE            _IO(SLIC_IOC_MAGIC, 10)
#define SLIC_SET_RX_MUTE            _IO(SLIC_IOC_MAGIC, 11)
#define SLIC_SET_TX_GAIN            _IO(SLIC_IOC_MAGIC, 12)
#define SLIC_SET_RX_GAIN            _IO(SLIC_IOC_MAGIC, 13)
#define SLIC_SET_REG                _IO(SLIC_IOC_MAGIC, 14)
#define SLIC_SET_RAM                _IO(SLIC_IOC_MAGIC, 15)
#define SLIC_SET_DEBUG_MODE         _IO(SLIC_IOC_MAGIC, 16)
#define SLIC_GET_EVENT              _IO(SLIC_IOC_MAGIC, 20)
#define SLIC_GET_TYPE               _IO(SLIC_IOC_MAGIC, 21)
#define SLIC_GET_STATUS             _IO(SLIC_IOC_MAGIC, 22)
#define SLIC_GET_REG                _IO(SLIC_IOC_MAGIC, 23)
#define SLIC_GET_RAM                _IO(SLIC_IOC_MAGIC, 24)
#define SLIC_PRINT_REG              _IO(SLIC_IOC_MAGIC, 25)
#define SLIC_PRINT_RAM              _IO(SLIC_IOC_MAGIC, 26)
#define SLIC_GET_DEBUG_MODE         _IO(SLIC_IOC_MAGIC, 27)
#define SLIC_POWER_ON               _IO(SLIC_IOC_MAGIC, 30)
#define SLIC_POWER_OFF              _IO(SLIC_IOC_MAGIC, 31)
#define SLIC_POWER_RESET            _IO(SLIC_IOC_MAGIC, 32)
#define SLIC_POWER_SUSPEND          _IO(SLIC_IOC_MAGIC, 33)
#define SLIC_POWER_RESUME           _IO(SLIC_IOC_MAGIC, 34)
#define SLIC_DAA_ONHOOK             _IO(SLIC_IOC_MAGIC, 40)
#define SLIC_DAA_OFFHOOK            _IO(SLIC_IOC_MAGIC, 41)
#define SLIC_CONFIG_FLASH_TIME      _IO(SLIC_IOC_MAGIC, 50)
#define SLIC_CONFIG_RX_TIMESLOT     _IO(SLIC_IOC_MAGIC, 51)
#define SLIC_CONFIG_TX_TIMESLOT     _IO(SLIC_IOC_MAGIC, 52)
#define SLIC_CONFIG_PCM             _IO(SLIC_IOC_MAGIC, 53)
#define SLIC_CONFIG_RING            _IO(SLIC_IOC_MAGIC, 54)
#define SLIC_CONFIG_DCFEED          _IO(SLIC_IOC_MAGIC, 55)
#define SLIC_CONFIG_IMPEDANCE       _IO(SLIC_IOC_MAGIC, 56)

struct slic_event {
int dev_id;     /* slic_dev->id */
int event_type; /* slic event type */
};

/* slic event type */
enum {
SLIC_EVENT_FXS_ONHOOK = 1,
SLIC_EVENT_FXS_OFFHOOK,
SLIC_EVENT_FXS_FLASH,
SLIC_EVENT_FXO_RING,
SLIC_EVENT_FXO_RING_STOP,
SLIC_EVENT_FXO_CID,
SLIC_EVENT_KEY_1,
SLIC_EVENT_KEY_2,
SLIC_EVENT_KEY_3,
SLIC_EVENT_KEY_4,
SLIC_EVENT_KEY_5,
SLIC_EVENT_KEY_6,
SLIC_EVENT_KEY_7,
SLIC_EVENT_KEY_8,
SLIC_EVENT_KEY_9,
SLIC_EVENT_KEY_0,
SLIC_EVENT_KEY_STAR,
SLIC_EVENT_KEY_POUND,
SLIC_EVENT_KEY_END,
SLIC_EVENT_MAX,
};

/* slic tone type */
enum {
SLIC_TONE_DIAL,
SLIC_TONE_BUSY,
SLIC_TONE_CONGESTION,
SLIC_TONE_RINGBACK,
SLIC_TONE_HOWLER,
SLIC_TONE_PIN,
SLIC_TONE_PUK,
SLIC_TONE_SERVICE_SUCCES,
SLIC_TONE_SERVICE_FAIL,
SLIC_TONE_ALERTING,
SLIC_TONE_CALL_WAITING,
SLIC_TONE_LOW_POWERE,
SLIC_TONE_VOICE_MAIL,
SLIC_TONE_TONE_MAX,
};

/* caller id type */
enum {
CALL_ID_FSK,
CALL_ID_DTMF,
CID_VOICEMAIL_ON,
CID_VOICEMAIL_OFF,
};

struct caller_id_data {
unsigned char cid_type;
unsigned char date[8];
unsigned char number[36];
unsigned char number_len;
unsigned char name[36];
unsigned char name_len;
};

enum {
SLIC_AUDIO_MUTE_OFF,
SLIC_AUDIO_MUTE_ON,
};

enum {
SLIC_AUDIO_GAIN_LEVEL_0,
SLIC_AUDIO_GAIN_LEVEL_1,
SLIC_AUDIO_GAIN_LEVEL_2,
SLIC_AUDIO_GAIN_LEVEL_3,
SLIC_AUDIO_GAIN_LEVEL_4,
SLIC_AUDIO_GAIN_LEVEL_5,
SLIC_AUDIO_GAIN_LEVEL_6,
};

/* data type for read/write memory */
struct slic_mem_cmd {
unsigned int addr;
unsigned int value;
};

/* slic type */
enum {
SLIC_TYPE_UNKNOWN,
SLIC_TYPE_FXS,
SLIC_TYPE_FXO,
};

/* slic linefeed status */
enum {
SLIC_FXS_ONHOOK,
SLIC_FXS_OFFHOOK,
SLIC_FXS_RING,
SLIC_FXO_ONHOOK,
SLIC_FXO_OFFHOOK,
SLIC_FXO_RING,
};

struct slic_dev {
struct device *dev;
int id;
int phone_event;
void *control_data;
struct mutex event_lock;
struct list_head device_entry;
struct fasync_struct *async_queue;
int (*ioctl)(struct slic_dev *slic, unsigned int cmd, void *cmd_data);
};

struct slic_platform_data {
int num_slic;
int irq_gpio;
int rst_gpio;
int power_gpio;
};

extern int slic_dev_register(struct device *parent, struct slic_dev *slic);
extern void slic_dev_unregister(struct slic_dev *slic);
extern int slic_report_event(struct slic_dev *slic, int event_type);

#endif
