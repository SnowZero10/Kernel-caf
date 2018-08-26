/*
**si3217x driver
**Author:haozhiwei
**date:2015-02-28
*/
#define DEBUG

#include <linux/init.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/param.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include "proslic_dev.h"
#include "proslic_tone.h"
#include <linux/platform_device.h>

static struct proslic_work si3217x_irq_data;
static spinlock_t si3217x_irq_spinlock;

#ifdef CONFIG_SLIC_IRQ_POLL
static struct proslic_work si3217x_poll_data;
#define IRQ_POLL_PERIOD         (1000)
#endif

DEFINE_MUTEX(spi_mutex);

int irq_cp;
unsigned char si3217x_spi_read(struct spi_device* spi,
				unsigned char cid,
				unsigned char reg)
{
	unsigned char value = 0;
	int ret;

	ret = spi_write(spi, &cid, 1);
	if (ret != 0){
		dev_err(&spi->dev, "%s spi_write() err %d\n", __func__, ret);
		return ret;
	}
	ret = spi_write(spi, &reg, 1);
	if (ret != 0){
		dev_err(&spi->dev, "%s spi_write() err %d\n", __func__, ret);
		return ret;
	}
	ret = spi_read(spi, &value, 1);
	if (ret != 0){
		dev_err(&spi->dev, "%s spi_read() err %d\n", __func__, ret);
		return ret;
	}
	//dev_dbg(&spi->dev, "%s cid %x reg %d val 0x%x\n", __func__, cid, reg, value);
	return (value);
}

int si3217x_spi_write(struct spi_device* spi,
				unsigned char cid,
				unsigned char reg,
				unsigned char value)
{
	int ret = 0;

	ret = spi_write(spi, &cid, 1);
	if (ret != 0){
		dev_err(&spi->dev, "%s spi_write() err %d\n", __func__, ret);
		return ret;
	}
	ret = spi_write(spi, &reg, 1);
	if (ret != 0){
		dev_err(&spi->dev, "%s spi_write() err %d\n", __func__, ret);
		return ret;
	}
	ret = spi_write(spi, &value, 1);
	if (ret != 0){
		dev_err(&spi->dev, "%s spi_write() err %d\n", __func__, ret);
		return ret;
	}
	//dev_dbg(&spi->dev, "%s cid %x reg %d val 0x%x\n", __func__, cid, reg, value);
return ret;
}

unsigned char si3217x_spi_direct_read(struct spi_device* spi,
				unsigned char channel,
				unsigned char reg)
{
	unsigned char val;
	unsigned char cid;

	/*
	* Bit reverse: Since the CID[0:4] is the LSB first, so we need to do the bit reverse
	*/
	channel &= 0x1F;
	channel = ((channel & 0x01) << 4)
		| ((channel & 0x02) << 2)
		|  (channel & 0x04)
		| ((channel & 0x08) >> 2)
		| ((channel & 0x10) >> 4);
	/*
	* Send Control byte, bit 6 set to 1 for reading
	*/
	cid = (0x60 | channel);
	val = si3217x_spi_read(spi, cid, reg);
	return (val);
}

int si3217x_spi_direct_write(struct spi_device* spi,
				unsigned char channel,
				unsigned char reg,
				unsigned char data)
{
	unsigned char cid;
	/*
	* Configure the bit7 in the control byte to 1 if channel is in the broadcast mode.
	* The CID[4:0] need to be configured to 1, too.
	*/
	if (0xFF == channel) {
		cid = 0xBF;
	} else {
		/*
		* Bit reverse: Since the CID[0:4] is the LSB first, so we need to do the bit reverse
		*/
		channel &= 0x1F;
		channel = ((channel & 0x01) << 4)
			| ((channel & 0x02) << 2)
			|  (channel & 0x04)
			| ((channel & 0x08) >> 2)
			| ((channel & 0x10) >> 4);
		/*
		* Send Control byte, bit 6 set to 0 for writing
		*/
		cid = (0x20 | channel);
	}
	return si3217x_spi_write(spi, cid, reg, data);
}

int si3217x_spi_indirect_write(struct spi_device* spi,
				unsigned char channel,
				unsigned short int addr,
				unsigned long data32)
{
	volatile unsigned char data;
	volatile unsigned char status;
	volatile unsigned char addr_hi;
	volatile unsigned char addr_lo;

	addr_hi = (addr >> 3) & 0xE0;
	addr_lo = addr & 0xFF;

	/* Wait for RAM status register to be ready. */
	do {
		status = si3217x_spi_direct_read(spi, channel, SI3217X_COM_REG_RAMSTAT);
		status &= 0x01;
	} while (0 != status);

	/* Write the high part of address */
	si3217x_spi_direct_write(spi, channel, SI3217X_COM_REG_RAM_ADDR_HI, addr_hi);

	/* Write data to ram */
	data = (unsigned char)((data32 << 3) & 0xFF);
	si3217x_spi_direct_write(spi, channel, SI3217X_COM_REG_RAM_DATA_B0, data);

	data = (unsigned char)((data32 >> 5) & 0xFF);
	si3217x_spi_direct_write(spi, channel, SI3217X_COM_REG_RAM_DATA_B1, data);

	data = (unsigned char)((data32 >> 13) & 0xFF);
	si3217x_spi_direct_write(spi, channel, SI3217X_COM_REG_RAM_DATA_B2, data);

	data = (unsigned char)((data32 >> 21) & 0xFF);
	si3217x_spi_direct_write(spi, channel, SI3217X_COM_REG_RAM_DATA_B3, data);

	/* Write the low part of address */
	si3217x_spi_direct_write(spi, channel, SI3217X_COM_REG_RAM_ADDR_LO, addr_lo);

	/* Wait for RAM status register to be ready. */
	do {
		status = si3217x_spi_direct_read(spi, channel, SI3217X_COM_REG_RAMSTAT);
		status &= 0x01;
	} while (0 != status);

	return 0;
}

unsigned long si3217x_spi_indirect_read(struct spi_device* spi,
				unsigned char channel,
				unsigned short int addr)
{
	volatile unsigned char status;
	volatile unsigned char addr_hi;
	volatile unsigned char addr_lo;
	unsigned long  data;
	unsigned long  val;

	addr_hi = (addr >> 3) & 0xE0;
	addr_lo = addr & 0xFF;

	/* Wait for RAM status register to be ready. */
	do {
		status = si3217x_spi_direct_read(spi, channel, SI3217X_COM_REG_RAMSTAT);
		status &= 0x01;
	} while (0 != status);

	/* Write the high part of address */
	si3217x_spi_direct_write(spi, channel, SI3217X_COM_REG_RAM_ADDR_HI, addr_hi);

	/* Write the low part of address */
	si3217x_spi_direct_write(spi, channel, SI3217X_COM_REG_RAM_ADDR_LO, addr_lo);

	/*
	*  Above actions initiates the READ transation, cause the RAM_STAT
	*  bit to set.
	*  Wait for RAM status register to be ready.
	*/
	do {
		status = si3217x_spi_direct_read(spi, channel, SI3217X_COM_REG_RAMSTAT);
		status &= 0x01;
	} while (0 != status);

	/* Read bit 4 ~ 0 */
	val = si3217x_spi_direct_read(spi, channel, SI3217X_COM_REG_RAM_DATA_B0);
	val = (val >> 3) & 0xFF;

	/* Read bit 12 ~ 5 */
	data = si3217x_spi_direct_read(spi, channel, SI3217X_COM_REG_RAM_DATA_B1);
	val = val | (data << 5);

	/* Read bit 20 ~ 13 */
	data = si3217x_spi_direct_read(spi, channel, SI3217X_COM_REG_RAM_DATA_B2);
	val = val | (data << 13);

	/* Read bit 28 ~ 21 */
	data = si3217x_spi_direct_read(spi, channel, SI3217X_COM_REG_RAM_DATA_B3);
	val = val | (data << 21);

	return (val);
}

int si3217x_reset(struct spi_device * spi, int status)
{
	struct proslic_dev *proslic = spi_get_drvdata(spi);

	dev_info(&spi->dev, "%s %d\n", __func__, status);
	if (status)
		gpio_direction_output(proslic->rst_gpio, SLIC_RESET_ENABLE);
	else
		gpio_direction_output(proslic->rst_gpio, SLIC_RESET_DISABLE);
	return 0;
}

uInt8 ctrl_ReadRegisterWrapper(void *hSpiGci, uInt8 channel, uInt8 regAddr)
{
	uInt8 ret;

	mutex_lock(&spi_mutex);
	ret = si3217x_spi_direct_read(hSpiGci, channel, regAddr);
	mutex_unlock(&spi_mutex);
	return ret;
}

int ctrl_WriteRegisterWrapper(void *hSpiGci, uInt8 channel, uInt8 regAddr, uInt8 data)
{
	int ret;

	mutex_lock(&spi_mutex);
	ret = si3217x_spi_direct_write(hSpiGci, channel, regAddr, data);
	mutex_unlock(&spi_mutex);
	return ret;
}

ramData ctrl_ReadRAMWrapper(void *hSpiGci, uInt8 channel, uInt16 ramAddr)
{
	uInt32 ret;

	mutex_lock(&spi_mutex);
	ret = si3217x_spi_indirect_read(hSpiGci, channel, ramAddr);
	mutex_unlock(&spi_mutex);
	return ret;
}

int ctrl_WriteRAMWrapper(void *hSpiGci, uInt8 channel, uInt16 ramAddr, ramData data)
{
	int ret;

	mutex_lock(&spi_mutex);
	ret = si3217x_spi_indirect_write(hSpiGci, channel, ramAddr, data);
	mutex_unlock(&spi_mutex);
	return ret;
}

int time_DelayWrapper(void *hTimer, int timeInMs)
{
	mdelay(timeInMs);
	return 0;
}

int time_TimeElapsedWrapper(void *hTimer, void *startTime, int *timeInMs)
{
	return 0;
}

int time_GetTimeWrapper(void *hTimer, void *time)
{
	return 0;
}

int ctrl_ResetWrapper(void *hSpiGci, int status)
{
	return si3217x_reset((struct spi_device *)hSpiGci, status);
}


#define WRITEREG(pProslic,address,value)        pProslic->deviceId->ctrlInterface->WriteRegister_fptr(pProslic->deviceId->ctrlInterface->hCtrl,pProslic->channel,address,value)
#define READREG(pProslic,address)               pProslic->deviceId->ctrlInterface->ReadRegister_fptr(pProslic->deviceId->ctrlInterface->hCtrl,pProslic->channel,address)
#define WRITERAM(pProslic,address,value)        pProslic->deviceId->ctrlInterface->WriteRAM_fptr(pProslic->deviceId->ctrlInterface->hCtrl,pProslic->channel,address,value)
#define READRAM(pProslic,address)               pProslic->deviceId->ctrlInterface->ReadRAM_fptr(pProslic->deviceId->ctrlInterface->hCtrl,pProslic->channel,address)


static int si3217x_read_reg(struct proslic_dev *si3217x,
				struct slic_mem_cmd *data)
{
	struct slic_dev *slic = &si3217x->sdev;
	proslicChanType_ptr chan = si3217x->chan;

	data->value = READREG(chan, data->addr);
	dev_info(slic->dev, "REG: %d = 0x%X\n", data->addr, data->value);
	return 0;
}

static int si3217x_read_ram(struct proslic_dev *si3217x,
				struct slic_mem_cmd *data)
{
	struct slic_dev *slic = &si3217x->sdev;
	proslicChanType_ptr chan = si3217x->chan;

	if (data->addr >= 1024)
        ProSLIC_SetUserMode(chan, TRUE, FALSE);

	data->value = READRAM(chan, data->addr);

	if (data->addr >= 1024)
        ProSLIC_SetUserMode(chan, FALSE, FALSE);

	dev_info(slic->dev, "RAM: %d = 0x%X\n", data->addr, data->value);
	return 0;
}

static int si3217x_write_reg(struct proslic_dev *si3217x,
				struct slic_mem_cmd *data)
{
	struct slic_dev *slic = &si3217x->sdev;
	proslicChanType_ptr chan = si3217x->chan;
	unsigned long before = 0;
	unsigned long after = 0;
	int ret = 0;

	before = READREG(chan, data->addr);
	ret = WRITEREG(chan, data->addr, data->value);
	after = READREG(chan, data->addr);

	dev_info(slic->dev, "%s addr %d = 0x%x, from 0x%x to 0x%x\n",
			__func__, (int)data->addr,
			(int)data->value, (int)before, (int)after);
	return ret;
}

static int si3217x_write_ram(struct proslic_dev *si3217x,
				struct slic_mem_cmd *data)
{
	struct slic_dev *slic = &si3217x->sdev;
	proslicChanType_ptr chan = si3217x->chan;
	unsigned long before = 0;
	unsigned long after = 0;
	int ret = 0;

	if (data->addr >= 1024)
        ProSLIC_SetUserMode(chan, TRUE, FALSE);

	before = READRAM(chan, data->addr);
	ret = WRITERAM(chan, data->addr, data->value);
	after = READRAM(chan, data->addr);

	if (data->addr >= 1024)
        ProSLIC_SetUserMode(chan, FALSE, FALSE);

	dev_info(slic->dev, "%s addr %d = 0x%x, from 0x%x to 0x%x\n",
			__func__, (int)data->addr,
			(int)data->value, (int)before, (int)after);
	return ret;
}

static int si3217x_read_hookstatus(struct proslic_dev *si3217x)
{
	unsigned char hook_state;

	ProSLIC_ReadHookStatus(si3217x->chan, &hook_state);
	dev_dbg(si3217x->sdev.dev, "HOOK STATE = %0X\n", hook_state);

	switch (hook_state) {
	case PROSLIC_OFFHOOK:
		return SLIC_FXS_OFFHOOK;
	case PROSLIC_ONHOOK:
		return SLIC_FXS_ONHOOK;
	default:
		return -1;
	}
}

static int si3217x_pcm_start(struct proslic_dev *si3217x)
{
	ProSLIC_PCMStart(si3217x->chan);
	return 0;
}

static int si3217x_pcm_stop(struct proslic_dev *si3217x)
{
	ProSLIC_PCMStop(si3217x->chan);
	return 0;
}

static int si3217x_ring_start(struct proslic_dev *si3217x)
{
	ProSLIC_RingStart(si3217x->chan);
	return 0;
}

static int si3217x_ring_stop(struct proslic_dev *si3217x)
{
	ProSLIC_RingStop(si3217x->chan);
	return 0;
}

static void si3217x_play_tone(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct tone_work *tw = container_of(dw, struct tone_work, work);
	struct proslic_dev *si3217x = tw->proslic;
	struct slic_dev *slic = &si3217x->sdev;
	struct tone_set *toneset = tw->tone;
	struct tone_list *playlist;

	if (TONE_HW_CTL == toneset->ctl_type) {
		tw->curr_list = 0;
		tw->curr_times = 0;
		playlist = &toneset->playlist[tw->curr_list];
		proslic_play_tonelist(si3217x, playlist);
		dev_dbg(slic->dev, "%s: hardware play preset %d\n",
				__func__, playlist->preset);
	} else if (TONE_SW_CTL == toneset->ctl_type) {
		playlist = &toneset->playlist[tw->curr_list];
		if (toneset->play_times > 0) {
			if (tw->curr_times < toneset->play_times) {
				proslic_play_tonelist(si3217x, playlist);
				if((tw->curr_list + 1) >= toneset->list_num)
					tw->curr_times = tw->curr_times + 1;
				tw->curr_list = (tw->curr_list + 1) % (toneset->list_num);
			} else {
				tw->curr_list = 0;
				tw->curr_times = 0;
				dev_dbg(slic->dev,
					"%s: software play %d times and stop\n",
					__func__, toneset->play_times);
				return;
			}
		} else {
			proslic_play_tonelist(si3217x, playlist);
			tw->curr_list = (tw->curr_list + 1) % (toneset->list_num);
		}
		schedule_delayed_work(&tw->work, msecs_to_jiffies(playlist->time));
	}
}

static int si3217x_tone_start(struct proslic_dev *si3217x, int type)
{
	struct slic_dev *slic = &si3217x->sdev;
	struct tone_work *tw = &(si3217x->tone_work);
	int ret;

	cancel_delayed_work_sync(&tw->work);
	tw->curr_list = 0;
	tw->curr_times = 0;

	ret = proslic_find_toneset(&tw->tone, type);
	if (ret < 0) {
		dev_err(slic->dev, "%s: not support tone %d\n", __func__, type);
		return -1;
	}
	dev_dbg(slic->dev, "%s: play tone %d\n", __func__, type);
	schedule_delayed_work(&tw->work, 0);
	return 0;
}

static int si3217x_tone_stop(struct proslic_dev *si3217x)
{
	struct tone_work *tw = &(si3217x->tone_work);

	cancel_delayed_work_sync(&tw->work);
	tw->curr_list = 0;
	tw->curr_times = 0;
	ProSLIC_ToneGenStop(si3217x->chan);
	return 0;
}

static int si3217x_wait_fskbuf(struct proslic_dev *si3217x)
{
	struct slic_dev *slic = &si3217x->sdev;
	proslicChanType_ptr chan = si3217x->chan;
	int ret;

	if (SLIC_FXS_OFFHOOK == si3217x_read_hookstatus(si3217x)) {
		ProSLIC_DisableCID(chan);
		dev_err(slic->dev, "%s: off-hook to disable cid\n", __func__);
		return (-1);
	}

	ret = wait_event_timeout(si3217x->fsk_wait,
	si3217x->state.fsk_int != 0, msecs_to_jiffies(FSK_WAIT_TIME));
	si3217x->state.fsk_int = 0;
	if (0 == ret)
		si3217x->state.fsk_err_num++;
	return 0;
}

static inline unsigned char si3217x_check_sum(unsigned char *string)
{
	int i = 0;
	unsigned char sum = 0;

	while (string[i] != 0)
		sum += string[i++];
	return -sum ;
}

static int si3217x_get_cid_msg(struct caller_id_data *cid,
				unsigned char *message)
{
	int total_len = 0;
	int date_len, number_len, name_len;
	unsigned char *date = cid->date;
	unsigned char *number = cid->number;
	unsigned char *name = cid->name;
	unsigned char cid_type = cid->cid_type;
	int i;

	if (date[0] == 0) {
		date_len = 0;
	} else {
		date_len = 8;
		total_len += date_len + 2;
	}

	if (number[0] == 0) {
		number_len = 0;
	} else {
		if (0 == cid->number_len)
			number_len = strlen(cid->number);
		else
			number_len = cid->number_len;
		total_len += number_len + 2;
	}

	if (name[0] == 0) {
		name_len = 0;
	} else {
		if (0 == cid->name_len)
			name_len = strlen(cid->name);
		else
			name_len = cid->name_len;
		total_len += name_len + 2;
	}

	if ((cid_type == CID_VOICEMAIL_ON) || (cid_type == CID_VOICEMAIL_OFF))
		total_len += 3;

	message[0] = 0x80;
	message[1] = (unsigned char)total_len;
	i = 2;

	if(date_len > 0){
		message[i++] = 0x01;
		message[i++] = 0x08;
		memcpy(&message[i], date, date_len);
		i += date_len;
	}

	if(number_len > 0){
		message[i++] = 0x02;
		message[i++] = number_len;
		memcpy(&message[i], number, number_len);
		i += number_len;
	}

	if(name_len > 0){
		message[i++] = 0x07;
		message[i++] = name_len;
		memcpy(&message[i], name, name_len);
		i += name_len;
	}

	if (cid->cid_type == CID_VOICEMAIL_ON) {
		message[i++] = 0x0B;
		message[i++] = 1;
		message[i++] = 0xFF;
	}

	if (cid->cid_type == CID_VOICEMAIL_OFF) {
		message[i++] = 0x0B;
		message[i++] = 1;
		message[i++] = 0x0;
	}

	message[i] = 0;
	message[i] = si3217x_check_sum(message);
	for (i = 0; i < total_len + 3; i++)
		pr_info("cid message[%d] = 0x%0x\n", i, message[i]);
	return 0;
}

static int si3217x_offhook_fsk(struct proslic_dev *si3217x,
				struct caller_id_data *cid)
{
	struct slic_dev *slic = &si3217x->sdev;
	proslicChanType_ptr chan = si3217x->chan;
	unsigned char message[96];
	int i, ret;

	dev_dbg(slic->dev, "%s Sending CID <%s,%s,%8s> to SLIC[%d]\n",
			__func__, cid->number, cid->name, cid->date, slic->id);
	ProSLIC_PCMStop(chan);
	atomic_set(&si3217x->cid_ack, 1);
	proslic_play_cas_tone(si3217x);
	msleep(83);
	ProSLIC_ToneGenStop(chan);
	ret = wait_event_timeout(si3217x->ack_wait,
				(2 == atomic_read(&si3217x->cid_ack)),
				msecs_to_jiffies(240));
	if (0 != ret) {
		si3217x_get_cid_msg(cid, message);
		dev_dbg(slic->dev, "%s Start Message (len = %d)\n",
				__func__, (message[1]+3));
		si3217x->state.fsk_err_num = 0;
		si3217x->state.fsk_irq_num = 0;
		ProSLIC_FSKSetup(chan, si3217x->fxs_config.fsk);
		ProSLIC_EnableCID(chan);
		msleep(68);
		for (i = 0; i < (message[1]+3); i++) {
			ProSLIC_SendCID(chan, &(message[i]), 1);
			ret = wait_event_timeout(si3217x->fsk_wait,
					si3217x->state.fsk_int != 0,
					msecs_to_jiffies(FSK_WAIT_TIME));
			si3217x->state.fsk_int = 0;
			if (0 == ret)
				si3217x->state.fsk_err_num++;
		}
		msleep(100);
		ProSLIC_DisableCID(chan);
		dev_dbg(slic->dev, "%s finish, %d errors, %d irqs\n",
				__func__, si3217x->state.fsk_err_num,
				si3217x->state.fsk_irq_num);
	}
	else {
		dev_dbg(slic->dev, "%s: ack not detected, cid_ack = %d\n",
				__func__, atomic_read(&si3217x->cid_ack));
	}
	atomic_set(&si3217x->cid_ack, 0);
	ProSLIC_PCMStart(chan);
	return 0;
}

static int si3217x_fsk_cid(struct proslic_dev * si3217x,
				struct caller_id_data *cid)
{
	struct slic_dev *slic = &si3217x->sdev;
	struct fxs_config *config = &si3217x->fxs_config;
	proslicChanType_ptr chan = si3217x->chan;
	unsigned char preamble[] ={'U'};
	unsigned char message[96];
	int i, num, index;

	dev_info(slic->dev, "%s Sending CID <%s,%s,%8s> to SLIC[%d]\n",
			__func__, cid->number, cid->name, cid->date, slic->id);
	si3217x_get_cid_msg(cid, message);
	si3217x->state.fsk_err_num = 0;
	si3217x->state.fsk_irq_num = 0;
	msleep(config->fsk_onhook_wait);

	if (SLIC_FXS_OFFHOOK == si3217x_read_hookstatus(si3217x)) {
		dev_err(slic->dev, "%s: off-hook to disable cid\n", __func__);
		goto err;
	}
	ProSLIC_FSKSetup(chan, config->fsk);
	ProSLIC_EnableCID(chan);
	dev_info(slic->dev, "%s begin, %d errors, %d irqs\n", __func__,
	si3217x->state.fsk_err_num, si3217x->state.fsk_irq_num);

	//Send preamble
	num = config->fsk_onhook_preamble - FSK_DEPTH_TRIG;
	for (i = 0; i <= num; i += FSK_DEPTH_TRIG) {
		ProSLIC_SendCID(chan, preamble, FSK_DEPTH_TRIG);
		if (si3217x_wait_fskbuf(si3217x))
			goto err;
	}
	if (30%FSK_DEPTH_TRIG) {
		ProSLIC_SendCID(chan, preamble, 30%FSK_DEPTH_TRIG);
		if (si3217x_wait_fskbuf(si3217x))
			goto err;
	}
	dev_info(slic->dev, "%s preamble end, %d errors, %d irqs\n", __func__,
			si3217x->state.fsk_err_num, si3217x->state.fsk_irq_num);

	//Send mark, 150ms delay for 180 mark bits(1200 baud)
	msleep(config->fsk_onhook_mark);
	dev_info(slic->dev, "%s mark end\n", __func__);

	//Send message
	num = (message[1]+3) - FSK_DEPTH_TRIG;
	for (i = 0; i <= num; i += FSK_DEPTH_TRIG) {
		ProSLIC_SendCID(chan, &(message[i]), FSK_DEPTH_TRIG);
		if (si3217x_wait_fskbuf(si3217x))
			goto err;
	}
	index = ((message[1]+3) / FSK_DEPTH_TRIG) * FSK_DEPTH_TRIG;
	num = (message[1]+3) % FSK_DEPTH_TRIG;
	if (num) {
		ProSLIC_SendCID(chan, &(message[index]), num);
		if (si3217x_wait_fskbuf(si3217x))
			goto err;
	}
	dev_info(slic->dev, "%s message end, %d errors, %d irqs\n", __func__,
			si3217x->state.fsk_err_num, si3217x->state.fsk_irq_num);

	msleep(200);
	ProSLIC_DisableCID(chan);
	dev_info(slic->dev, "%s finish, %d errors, %d irqs\n", __func__,
			si3217x->state.fsk_err_num, si3217x->state.fsk_irq_num);
	return 0;
err:
	return (-1);
}

static int si3217x_voice_mail(struct proslic_dev *si3217x,
				struct caller_id_data *cid, int enable)
{
	int ret = -1;

	if (SLIC_FXS_OFFHOOK == si3217x_read_hookstatus(si3217x))
		return ret;

	memset(cid, 0, sizeof(struct caller_id_data));
	if (enable)
		cid->cid_type = CID_VOICEMAIL_ON;
	else
		cid->cid_type = CID_VOICEMAIL_OFF;
	ProSLIC_SetLinefeedStatus(si3217x->chan, LF_FWD_OHT);
	ret = si3217x_fsk_cid(si3217x, cid);
	ProSLIC_SetLinefeedStatus(si3217x->chan, LF_FWD_ACTIVE);
	return ret;
}

static int si3217x_dtmf_cid(struct proslic_dev *si3217x,
				struct caller_id_data *cid)
{
	proslicChanType_ptr chan = si3217x->chan;
	unsigned int i;

	ProSLIC_SetPowersaveMode(chan, PWRSAVE_DISABLE);
	msleep(10);
	ProSLIC_SetLinefeedStatus(chan, LF_FWD_OHT);
	ProSLIC_PolRev(chan, 0, 1);
	msleep(300);
	for (i = 0; i < cid->number_len; i++) {
		msleep(160);
		ProSLIC_ToneGenStop(chan);
		ProSLIC_ToneGenSetup(chan, (cid->number[i]-48+TONE_DTMF_0));
		ProSLIC_ToneGenStart(chan, 1);
	}
	msleep(80);
	ProSLIC_ToneGenStop(chan);
	msleep(100);
	ProSLIC_PolRev(chan, 0, 0);
	ProSLIC_SetLinefeedStatus(chan, LF_FWD_ACTIVE);
	ProSLIC_SetPowersaveMode(chan, PWRSAVE_ENABLE);
	msleep(50);
	return 0;
}

static int si3217x_gain_level(int level)
{
	int gain = 0;

	switch (level) {
	case SLIC_AUDIO_GAIN_LEVEL_0:
		gain = -18;
		break;
	case SLIC_AUDIO_GAIN_LEVEL_1:
		gain = -12;
		break;
	case SLIC_AUDIO_GAIN_LEVEL_2:
		gain = -6;
		break;
	case SLIC_AUDIO_GAIN_LEVEL_3:
		gain = -3;
		break;
	case SLIC_AUDIO_GAIN_LEVEL_4:
		gain = 0;
		break;
	case SLIC_AUDIO_GAIN_LEVEL_5:
		gain = 3;
		break;
	case SLIC_AUDIO_GAIN_LEVEL_6:
		gain = 6;
		break;
	}
	return gain;
}

static int si3217x_audio_gain(struct proslic_dev *si3217x)
{
	struct fxs_config *config = &si3217x->fxs_config;

	if (si3217x->chan->channelType == PROSLIC) {
		ProSLIC_AudioGainSetup(si3217x->chan,
				config->audio_gain_rx,
				config->audio_gain_tx,
				config->impedance);
		dev_dbg(si3217x->sdev.dev, "%s: set rx gain %d, tx gain %d\n",
				__func__, config->audio_gain_rx,
				config->audio_gain_tx);
	}
	return 0;
}

static int si3217x_pcm_enable(struct proslic_dev *si3217x)
{
	return ProSLIC_PLLFreeRunStop(si3217x->chan);
}

static int si3217x_pcm_disable(struct proslic_dev *si3217x)
{
	return ProSLIC_PLLFreeRunStart(si3217x->chan);
}

static void si3217x_is_onhook(struct work_struct *work)
{
	struct delayed_work * dw = container_of(work, struct delayed_work, work);
	struct proslic_dev * si3217x =
			container_of(dw, struct proslic_dev, hook_work);
	struct slic_dev * slic = &si3217x->sdev;

	if (si3217x->state.onhook_timestamp != 0) {
		slic_report_event(slic, SLIC_EVENT_FXS_ONHOOK);
		dev_dbg(slic->dev, "Onhook\n");
#ifdef CONFIG_SLIC_POLL_DTMF
		cancel_delayed_work(&si3217x->dtmf_work);
#endif
	}
}

#ifdef CONFIG_SLIC_POLL_DTMF
/*
** Detection of SLIC costs 13.33ms at least and 26ms at most
** Polling assert costs 10ms at most
** SLIC REG read operation need about 2ms
** ADSP mute operation need about 3ms
** SLIC mute operation need about 3ms
*/
#define DTMF_DETECT_PERIOD        10  //10ms

#ifdef CONFIG_SND_SOC_QDSP6
extern uint16_t voc_get_session_id(char *name);
extern int voc_set_tx_mute(uint16_t session_id, uint32_t dir,
				uint32_t mute);

static int si3217x_set_voc_tx_mute(int mute)
{
	voc_set_tx_mute(voc_get_session_id("Voice session"), 1, mute);
	voc_set_tx_mute(voc_get_session_id("VoLTE session"), 1, mute);
	return 0;
}
#elif defined(CONFIG_SND_SOC_QDSP6V2)
extern uint16_t voc_get_session_id(char *name);
extern int voc_set_tx_mute(uint32_t session_id, uint32_t dir,
				uint32_t mute, uint32_t ramp_duration);

static int si3217x_set_voc_tx_mute(int mute)
{
	voc_set_tx_mute(voc_get_session_id("Voice session"), 1, mute, 0);
	voc_set_tx_mute(voc_get_session_id("VoLTE session"), 1, mute, 0);
	return 0;
}
#else
static int si3217x_set_voc_tx_mute(int mute)
{
	return 0;
}
#endif
#endif

static int si3217x_set_mute_status(struct proslic_dev * si3217x,
				int path, int mute)
{
	u8 data = 0;

	if (si3217x->chan->channelType != PROSLIC) {
		return RC_CHANNEL_TYPE_ERR;
	}

	switch (mute) {
	case SLIC_AUDIO_MUTE_OFF:
		if (path & PROSLIC_MUTE_RX)
			data &= ~1;
		if (path & PROSLIC_MUTE_TX)
			data &= ~2;
		break;
	case SLIC_AUDIO_MUTE_ON:
		if (path & PROSLIC_MUTE_RX)
			data |= 1;
		if (path & PROSLIC_MUTE_TX)
			data |= 2;
		break;
	default:
		return -1;
	}

	if (data != (READREG(si3217x->chan, SI3217X_COM_REG_DIGCON) & 0x03)) {
		dev_dbg(si3217x->sdev.dev, "%s %d\n", __func__, data);
		WRITEREG(si3217x->chan, SI3217X_COM_REG_DIGCON, data);
	}
	return 0;
}

static void si3217x_dtmf_work(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct proslic_dev *si3217x =
			container_of(dw, struct proslic_dev, dtmf_work);
	struct slic_dev *slic = &si3217x->sdev;
	unsigned char data;
	unsigned char key_event;
	int event_type;
	unsigned long timestamp;
	unsigned int period;

#ifdef CONFIG_SLIC_POLL_DTMF
	static unsigned int pass_cnt = 0;
	static unsigned int fail_cnt = 0;
	static unsigned int dtmf_pass_first_time = 0;
	static unsigned int dtmf_fail_first_time = 0;
	static unsigned int pass_report_flag = 0;
	static unsigned int fail_report_flag = 0;

	unsigned long now = jiffies;

	data = READREG(si3217x->chan, SI3217X_COM_REG_TONDTMF);
	if (data & 0x30) {
		dev_dbg(si3217x->sdev.dev, "pass TONDTMF = %0X\n", data);
		if (0 == pass_cnt) {
			dtmf_pass_first_time = now;
			++pass_cnt;
			pass_report_flag = 0;
			si3217x_set_voc_tx_mute(1);
			si3217x_set_mute_status(si3217x,
					PROSLIC_MUTE_TX, SLIC_AUDIO_MUTE_ON);
		}
		timestamp = (dtmf_pass_first_time + msecs_to_jiffies(40));
		if (!pass_report_flag && time_after_eq(now, timestamp)) {
			ProSLIC_DTMFReadDigit(si3217x->chan, &key_event);
			si3217x->state.key_event = key_event;
			if (1 == atomic_read(&si3217x->cid_ack)) {
				dev_dbg(slic->dev, "FSK ACK DTMF Event %x\n",
						key_event);
				atomic_set(&si3217x->cid_ack, 2);
				wake_up(&si3217x->ack_wait);
			}
			else {
				dev_dbg(slic->dev, "DTMF Digit: %d\n", key_event);
				event_type = (SLIC_EVENT_KEY_1 + key_event -1);
				slic_report_event(slic, event_type);
			}
			pass_report_flag = 1;
		}
		fail_cnt = 0;
	} else {
		if (0 == fail_cnt) {
			dtmf_fail_first_time = now;
			++fail_cnt;
			fail_report_flag = 0;
		}
		period = si3217x->fxs_config.dtmf_mute_period;
		timestamp = (dtmf_fail_first_time + msecs_to_jiffies(period));
		if (time_after_eq(now, timestamp)) {
			if (!fail_report_flag) {
				si3217x_set_mute_status(si3217x,
						PROSLIC_MUTE_TX,
						SLIC_AUDIO_MUTE_OFF);
				si3217x_set_voc_tx_mute(0);
				fail_report_flag = 1;
			}
		} else {
			/* dev_dbg(si3217x->sdev.dev, "fail TONDTMF = %0X\n", data); */
			if ((pass_cnt > 0) && !pass_report_flag ) {
				si3217x_set_mute_status(si3217x,
						PROSLIC_MUTE_TX,
						SLIC_AUDIO_MUTE_OFF);
				si3217x_set_voc_tx_mute(0);
			}
		}
		pass_cnt = 0;
	}
	schedule_delayed_work(&si3217x->dtmf_work,
	msecs_to_jiffies(DTMF_DETECT_PERIOD));
#else
	data = READREG(si3217x->chan, SI3217X_COM_REG_TONDTMF);
	dev_dbg(slic->dev, "TONDTMF = %0X\n", data);
	if (data & 0x30) {
		schedule_delayed_work(&si3217x->dtmf_work, msecs_to_jiffies(40));
	} else{
		slic_report_event(slic, SLIC_EVENT_KEY_END);
	}
#endif
}


char * daa_int_str [] =
{
"POLI","TGDI","LCSOI","DODI","BTDI","FDTI","ROVI","RDTI","CVI"
};

char * fxs_int_str [] =
{
"OSC1_T1","OSC1_T2","OSC2_T1","OSC2_T2","RING_T1","RING_T2","PM_T1","PM_T2","FSKBUF_AVAIL",
"VBAT","RING_TRIP","LOOP_STATUS","LONG_STAT","VOC_TRACK","DTMF","INDIRECT","TXMDM","RXMDM",
"PQ1","PQ2","PQ3","PQ4","PQ5","PQ6","RING_FAIL","CM_BAL",
"USER_IRQ0","USER_IRQ1","USER_IRQ2","USER_IRQ3","USER_IRQ4","USER_IRQ5","USER_IRQ6","USER_IRQ7",
"IRQ_DSP","IRQ_MADC_FS","IRQ_P_HVIC","IRQ_P_THERM","IRQ_P_OFFLD"
};

char * chip_type_str[] =
{
"SI3210","SI3215","SI3216","SI3211","SI3212","SI3210M","SI3215M","SI3216M",
"SI3240","SI3241","SI3242","SI3243","SI3245","SI3244","SI3246","SI3247",
"SI3220","SI3225","SI3226","SI3227",
"SI32171","SI32172","SI32174","SI32175","SI32176","SI32177","SI32178","SI32179",
"SI32260","SI32261","SI32262","SI32263","SI32264","SI32265","SI32266","SI32267","SI32268","SI32269",
"SI32360","SI32361",
};

char * chip_rev_str[] =
{
"A","B","C","D","E","F","G",
};

char * slic_type_str[] =
{
"Unknown", "FXS", "FXO",
};

static int si3217x_get_interrupts(struct proslic_dev * si3217x)
{
	struct slic_dev *slic = &si3217x->sdev;
	proslicChanType_ptr chan = si3217x->chan;
	int i;
	ProslicInt fxs_irqs[32];
	vdaaInt daa_irqs[9];
	proslicIntType fxs_int_data;
	vdaaIntType daa_int_data;
	int valid_irq_num = 0;
	unsigned char status;

	fxs_int_data.irqs = fxs_irqs;
	daa_int_data.irqs = daa_irqs;
	if (chan->channelType == PROSLIC) {
		ProSLIC_GetInterrupts(chan, &fxs_int_data);
		dev_dbg(slic->dev, "%d interrupts\n", fxs_int_data.number);
#ifdef CONFIG_SLIC_IRQ_TRIG_LOW
    if (irq_cp==0)
        enable_irq(gpio_to_irq(si3217x->irq_gpio));
    else
        enable_irq(gpio_to_irq(si3217x->irq_gpio_cp));
#endif
		for (i = 0; i < fxs_int_data.number; i++) {
			switch (fxs_int_data.irqs[i]) {
			case IRQ_LOOP_STATUS:
				ProSLIC_ReadHookStatus(chan, &status);
				si3217x->state.hook_event = status;
				si3217x->state.hook_int = 1;
				dev_dbg(slic->dev, "FXS INTERRUPT: %s\n",
					fxs_int_str[fxs_int_data.irqs[i]]);
				valid_irq_num ++;
				break;
			case IRQ_DTMF:
				si3217x->state.key_int = 1;
				dev_dbg(slic->dev, "FXS INTERRUPT: %s\n",
					fxs_int_str[fxs_int_data.irqs[i]]);
				valid_irq_num ++;
				break;
			case IRQ_FSKBUF_AVAIL:
				valid_irq_num ++;
				dev_dbg(si3217x->sdev.dev, "fsk irq\n");
				si3217x->state.fsk_int = 1;
				si3217x->state.fsk_irq_num++;
				wake_up(&si3217x->fsk_wait);
				break;
			default:
				dev_err(si3217x->sdev.dev, "FXS INTERRUPT: %s\n",
					fxs_int_str[fxs_int_data.irqs[i]]);
				break;
			}
		}
	}
	else if (chan->channelType == DAA) {
		Vdaa_GetInterrupts(chan, &daa_int_data);
		for (i = 0; i < daa_int_data.number; i++) {
			if (daa_int_data.irqs[i] == RDTI) {
				Vdaa_ReadRingDetectStatus(chan, &(si3217x->state.vdaa_status));
				//si3217x->state.vdaa_status.ringDetected = 1;
				//dev_dbg(slic->dev, "DAA INTERRUPT: %s\n",
				//	daa_int_str[daa_int_data.irqs[i]]);
				valid_irq_num ++;
			}
		}
	}
	return valid_irq_num;
}

static void si3217x_irq_work(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct proslic_work *irq_data = container_of(dw, struct proslic_work, work);
	struct proslic_dev *proslic = irq_data->proslic;
	struct proslic_dev *si3217x;
	struct slic_dev *slic;
	struct proslic_status *state;
	int i;
	unsigned long now = jiffies;
	int flash_time_min, flash_time_max;
	int total_irq_num = 0;
	unsigned long period, timestamp;

	//pr_debug("%s\n", __func__);
	for (i = 0; i < proslic->num_slic; i++)
		total_irq_num += si3217x_get_interrupts(&proslic[i]);

	if (0 == total_irq_num) {
		irq_data->irq_err ++;
		pr_err("si3217x interrupt has not been found for %d times\n",
		irq_data->irq_err);
		return ;
	}

	for (i = 0; i < proslic->num_slic; i++) {
		si3217x = &proslic[i];
		state = &si3217x->state;
		slic = &si3217x->sdev;
		if (state->hook_int) {
			flash_time_max = si3217x->fxs_config.flash_time_max;
			flash_time_min = si3217x->fxs_config.flash_time_min;
			if (PROSLIC_OFFHOOK == state->hook_event) {
				period = msecs_to_jiffies(flash_time_max);
				timestamp = state->onhook_timestamp + period;
				if (time_before(now, timestamp)) {
					cancel_delayed_work_sync(&si3217x->hook_work);
					period = msecs_to_jiffies(flash_time_min);
					timestamp = state->onhook_timestamp + period;
					if (time_after_eq(now, timestamp)) {
						slic_report_event(slic, SLIC_EVENT_FXS_FLASH);
						dev_dbg(slic->dev, "Flash\n");
					}
					else {
						dev_dbg(slic->dev, "Offhook not Reported\n");
					}
				} else {
					slic_report_event(slic, SLIC_EVENT_FXS_OFFHOOK);
					dev_dbg(slic->dev, "Offhook\n");
#ifdef CONFIG_SLIC_POLL_DTMF
					schedule_delayed_work(&si3217x->dtmf_work, 0);
#endif
				}
				state->onhook_timestamp = 0;
			}
			else if (PROSLIC_ONHOOK == state->hook_event) {
				schedule_delayed_work(&si3217x->hook_work,
				msecs_to_jiffies(flash_time_max));
				state->onhook_timestamp = now;
				dev_dbg(slic->dev, "Onhook Waiting\n");
			}
			state->hook_int = 0;
		}

		if (state->key_int) {
#ifdef CONFIG_SLIC_POLL_DTMF
#else
			if (1 == atomic_read(&si3217x->cid_ack)) {
				atomic_set(&si3217x->cid_ack, 2);
				dev_dbg(slic->dev, "DTMF Digit : %x\n", state->key_event);
				wake_up(&si3217x->ack_wait);
			} else {
				schedule_delayed_work(&si3217x->dtmf_work, msecs_to_jiffies(40));
				slic_report_event(slic, (SLIC_EVENT_KEY_1 + state->key_event -1));
				dev_dbg(slic->dev, "DTMF Digit : %x\n", state->key_event);
			}
#endif
			state->key_int = 0;
		}
	}
	return;
}

static irqreturn_t si3217x_irq(int irq, void *dev_id)
{
	unsigned long flags;

	//pr_debug("%s\n", __func__);
#ifdef CONFIG_SLIC_IRQ_TRIG_LOW
	disable_irq_nosync(irq);
#endif
	spin_lock_irqsave(&si3217x_irq_spinlock, flags);
	schedule_delayed_work(&si3217x_irq_data.work, 0);
	spin_unlock_irqrestore(&si3217x_irq_spinlock, flags);
	return IRQ_HANDLED;
}
/*
static irqreturn_t si3217x_irq_cp(int irq, void *dev_id)
{
	unsigned long flags;
	irq_cp =1;
	//pr_debug("%s\n", __func__);
#ifdef CONFIG_SLIC_IRQ_TRIG_LOW
	disable_irq_nosync(irq);
#endif
	spin_lock_irqsave(&si3217x_irq_spinlock, flags);
	schedule_delayed_work(&si3217x_irq_data.work, 0);
	spin_unlock_irqrestore(&si3217x_irq_spinlock, flags);
	return IRQ_HANDLED;
}
*/
#ifdef CONFIG_SLIC_IRQ_POLL
static void si3217x_poll_work(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct proslic_work *poll_data = container_of(dw, struct proslic_work, work);
	struct proslic_dev *proslic = poll_data->proslic;
	static int last = 1; //high default
	int val;

	val = gpio_get_value_cansleep(proslic->irq_gpio);
	if ((val == last) && ((0 == val))) { //if low state lasts 1000ms
		schedule_delayed_work(&si3217x_irq_data.work, 0);
	}
	last = val;
	schedule_delayed_work(&si3217x_poll_data.work,
			msecs_to_jiffies(IRQ_POLL_PERIOD));
}
#endif

static int si3217x_create_proslic(struct proslic_dev *si3217x, int i)
{
#ifndef DISABLE_MALLOC
	int ret = 0;
	ret = ProSLIC_createControlInterface(&si3217x->hw);
	if (ret)
		return -ENOMEM;

	ret = ProSLIC_createDevice (&si3217x->device);
	if (ret) {
		ProSLIC_destroyControlInterface(&si3217x->hw);
		return -ENOMEM;
	}

	ret = ProSLIC_createChannel(&si3217x->chan);
	if (ret) {
		ProSLIC_destroyDevice(&si3217x->device);
		ProSLIC_destroyControlInterface(&si3217x->hw);
		return -ENOMEM;
	}
#else
	si3217x->hw = kzalloc(sizeof(controlInterfaceType), GFP_KERNEL);
	if (!si3217x->hw) {
		return -ENOMEM;
	}

	si3217x->device = kzalloc(sizeof(ProslicDeviceType), GFP_KERNEL);
	if (!si3217x->device) {
		kfree(si3217x->hw);
		return -ENOMEM;
	}

	si3217x->chan = kzalloc(sizeof(proslicChanType), GFP_KERNEL);
	if (!si3217x->chan) {
		kfree(si3217x->device);
		kfree(si3217x->hw);
		return -ENOMEM;
	}
#endif
	ProSLIC_SWInitChan(si3217x->chan, i, si3217x->type, si3217x->device, si3217x->hw);
	ProSLIC_setSWDebugMode(si3217x->chan, 0);
	ProSLIC_setControlInterfaceCtrlObj (si3217x->hw, si3217x->spi);
	ProSLIC_setControlInterfaceReset (si3217x->hw, ctrl_ResetWrapper);
	ProSLIC_setControlInterfaceWriteRegister (si3217x->hw, ctrl_WriteRegisterWrapper);
	ProSLIC_setControlInterfaceReadRegister (si3217x->hw, ctrl_ReadRegisterWrapper);
	ProSLIC_setControlInterfaceWriteRAM (si3217x->hw, ctrl_WriteRAMWrapper);
	ProSLIC_setControlInterfaceReadRAM (si3217x->hw, ctrl_ReadRAMWrapper);
	ProSLIC_setControlInterfaceTimerObj (si3217x->hw, NULL);
	ProSLIC_setControlInterfaceDelay (si3217x->hw, time_DelayWrapper);
	ProSLIC_setControlInterfaceTimeElapsed (si3217x->hw, time_TimeElapsedWrapper);
	ProSLIC_setControlInterfaceGetTime (si3217x->hw, time_GetTimeWrapper);
	ProSLIC_setControlInterfaceSemaphore (si3217x->hw, NULL);
	return 0;
}

static void si3217x_destory_proslic(struct proslic_dev *si3217x)
{
#ifndef DISABLE_MALLOC
	ProSLIC_destroyChannel(&si3217x->chan);
	ProSLIC_destroyDevice(&si3217x->device);
	ProSLIC_destroyControlInterface(&si3217x->hw);
#else
	kfree(si3217x->chan);
	kfree(si3217x->device);
	kfree(si3217x->hw);
#endif
}

static int si3217x_pbx_setup(struct proslic_dev *si3217x)
{
	proslicChanType_ptr chan = si3217x->chan;
	struct fxs_config *fxs = &si3217x->fxs_config;
	struct fxo_config *fxo = &si3217x->fxo_config;

	switch (chan->channelType) {
	case PROSLIC:
		ProSLIC_DCFeedSetup(chan, fxs->dcfeed);
		ProSLIC_RingSetup(chan, fxs->ring);
		ProSLIC_ZsynthSetup(chan, fxs->impedance);
		ProSLIC_AudioGainSetup(chan,
		fxs->audio_gain_rx, fxs->audio_gain_tx, fxs->impedance);
		ProSLIC_PCMTimeSlotSetup(chan,
		fxs->pcm_timeslot_rx, fxs->pcm_timeslot_tx);
		ProSLIC_PCMSetup(chan, fxs->pcm_config);
		ProSLIC_FSKSetup(chan, fxs->fsk);
		ProSLIC_SetLinefeedStatus(chan, LF_FWD_ACTIVE);
		ProSLIC_EnableInterrupts(chan);
		ProSLIC_PCMStart(chan); //make sure PCM_OUT/PCM_IN in certain state
		msleep(1000);
		ProSLIC_PCMStop(chan);
#ifdef CONFIG_SLIC_POLL_DTMF
		WRITEREG(chan, SI3217X_COM_REG_TONEN, 0xE0); //26ms
#else
		WRITEREG(chan, SI3217X_COM_REG_TONEN, 0xF0); //40ms
#endif
		WRITERAM(chan, SI3217X_COM_RAM_DTMINPTH, 0x50AD1C);
		break;
	case DAA:
		Vdaa_CountrySetup(chan, fxo->country);
		Vdaa_PCMTimeSlotSetup(chan, fxo->pcm_timeslot_rx,
				fxo->pcm_timeslot_tx);
		Vdaa_PCMSetup(chan, fxo->pcm_config);
		Vdaa_RingDetectSetup(chan, fxo->ring);
		Vdaa_PCMStart(chan);
		Vdaa_SetInterruptMask(chan,0x0080);
		break;
	default:
		break;
	}
	return 0;
}

static int si3217x_pbx_init (struct proslic_dev *si3217x)
{
	int num = si3217x->num_slic;
	struct slic_dev *slic = &si3217x->sdev;
	struct proslic_dev *proslic = (struct proslic_dev *)slic->control_data;
	int i, err;
	int ret = 0;

	proslicChanType_ptr chans[num];

	if (proslic->initialized)
		return ret;

	/* ProSLIC_Init is for all channels */
	for (i = 0; i < num; i++) {
		chans[i] = proslic[i].chan;
	}
	ProSLIC_Reset(chans[0]);
	ProSLIC_Init_MultiBOM(chans, num, 4);
	Vdaa_Init(chans, num);

	for (i = 0; i < num; i++) {
		switch (chans[i]->channelType) {
		case PROSLIC:
		case DAA:
			if (chans[i]->error) {
				ProSLIC_getErrorFlag(chans[i], &err);
				ret = -err;
				dev_err(slic->dev, "SLIC[%d] err %d\n", i, err);
				goto err;
			}
			si3217x_pbx_setup(&proslic[i]);
			dev_info(slic->dev,
				"SLIC[%d] is %s, Type %s, Rev %s\n", i,
				slic_type_str[chans[i]->channelType],
				chip_type_str[chans[i]->deviceId->chipType],
				chip_rev_str[chans[i]->deviceId->chipRev]);
			break;
		default:
			dev_err(slic->dev, "SLIC[%d] unknown type\n", i);
			ret = -ENODEV;
			goto err;
		}
	}

#ifdef CONFIG_SLIC_IRQ_TRIG_LOW
	ret = request_irq(gpio_to_irq(proslic->irq_gpio), si3217x_irq,
			IRQF_SHARED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_LOW,
			"si3217x", proslic);

#else
	ret = request_irq(gpio_to_irq(proslic->irq_gpio), si3217x_irq,
			IRQF_SHARED | IRQF_TRIGGER_FALLING,
			"si3217x", proslic);

#endif
	if (ret < 0) {
		dev_err(slic->dev, "request irq faild\n");
		goto err;
	}

	proslic->initialized = 1;
	dev_info(slic->dev, "SLIC Power Up\n");
	return ret;

err:
	for (i = 0; i < num; i++) {
		ProSLIC_SWInitChan(chans[i], chans[i]->channel,
				si3217x->type, si3217x->device,
				si3217x->hw);
	}
	return ret;
}

static int si3217x_pbx_exit(struct proslic_dev *si3217x)
{
	struct slic_dev *slic = &si3217x->sdev;
	struct proslic_dev *proslic = (struct proslic_dev *)slic->control_data;
	int irq = gpio_to_irq(proslic->irq_gpio);
	int irq_cp = gpio_to_irq(proslic->irq_gpio_cp);
	int i;

	for (i = 0; i < proslic->num_slic; i++)
		ProSLIC_ShutdownChannel(proslic[i].chan);
	free_irq(irq, proslic);
	free_irq(irq_cp, proslic);
	proslic->initialized = 0;
	dev_info(slic->dev, "SLIC Power Down\n");
	return 0;
}

static int si3217x_slic_ioctl(struct slic_dev *slic,
				unsigned int cmd, void *arg)
{
	struct proslic_dev *proslic = (struct proslic_dev *)slic->control_data;
	struct proslic_dev *si3217x = &proslic[slic->id];
	int ret = 0;
	int *val = arg;
	struct caller_id_data *cid_data = arg;
	struct slic_mem_cmd *mem_data = arg;

#ifdef CONFIG_SLIC_IRQ_POLL
	cancel_delayed_work(&si3217x_poll_data.work);
#endif
	mutex_lock(&si3217x->lock);
	switch (cmd) {
	case SLIC_PCM_START:
		ret = si3217x_pcm_start(si3217x);
		break;
	case SLIC_PCM_STOP:
		ret = si3217x_pcm_stop(si3217x);
		break;
	case SLIC_TONE_START:
		ret = si3217x_tone_start(si3217x, *val);
		break;
	case SLIC_TONE_STOP:
		ret = si3217x_tone_stop(si3217x);
		break;
	case SLIC_RING_START:
		if (cid_data->cid_type == CALL_ID_FSK) {
			si3217x_ring_start(si3217x);
			ret = si3217x_fsk_cid(si3217x, cid_data);
	      mdelay(1000);
		} else {
			ret = si3217x_dtmf_cid(si3217x, cid_data);
			si3217x_ring_start(si3217x);
		}
		break;
	case SLIC_RING_STOP:
		ret = si3217x_ring_stop(si3217x);
		break;
	case SLIC_CID_OFFHOOK:
		ret = si3217x_offhook_fsk(si3217x, cid_data);
		break;
	case SLIC_VM_START:
		ret = si3217x_voice_mail(si3217x, cid_data, 1);
		dev_info(slic->dev, "si3217x_voice_mail SLIC_VM_START\n");
		break;
	case SLIC_VM_STOP:
		ret = si3217x_voice_mail(si3217x, cid_data, 0);
		dev_info(slic->dev, "si3217x_voice_mail SLIC_VM_STOP\n");
		break;
	case SLIC_SET_TX_MUTE:
		ret = si3217x_set_mute_status(si3217x, PROSLIC_MUTE_TX, *val);
		break;
	case SLIC_SET_RX_MUTE:
		ret = si3217x_set_mute_status(si3217x, PROSLIC_MUTE_RX, *val);
		break;
	case SLIC_SET_TX_GAIN:
		si3217x->fxs_config.audio_gain_tx = si3217x_gain_level(*val);
		ret = si3217x_audio_gain(si3217x);
		break;
	case SLIC_SET_RX_GAIN:
		si3217x->fxs_config.audio_gain_rx = si3217x_gain_level(*val);
		ret = si3217x_audio_gain(si3217x);
		break;
	case SLIC_SET_REG:
		ret = si3217x_write_reg(si3217x, mem_data);
		break;
	case SLIC_SET_RAM:
		ret = si3217x_write_ram(si3217x, mem_data);
		break;
	case SLIC_SET_DEBUG_MODE:
		ProSLIC_setSWDebugMode(si3217x->chan, *val?1:0);
		break;
	case SLIC_GET_EVENT:
		*val = slic->phone_event;
		break;
	case SLIC_GET_TYPE:
		if (PROSLIC == si3217x->chan->channelType)
			*val = SLIC_TYPE_FXS;
		else if (DAA == si3217x->chan->channelType)
			*val = SLIC_TYPE_FXO;
		else
			*val = SLIC_TYPE_UNKNOWN;
		break;
	case SLIC_GET_STATUS:
		*val = si3217x_read_hookstatus(si3217x);
		break;
	case SLIC_GET_REG:
		si3217x_read_reg(si3217x, mem_data);
		break;
	case SLIC_GET_RAM:
		si3217x_read_ram(si3217x, mem_data);
		break;
	case SLIC_PRINT_REG:
		ProSLIC_PrintDebugReg(si3217x->chan);
		break;
	case SLIC_PRINT_RAM:
		ProSLIC_PrintDebugRAM(si3217x->chan);
		break;
	case SLIC_GET_DEBUG_MODE:
		dev_info(slic->dev, "DebugMode: %d\n", si3217x->chan->debugMode);
		break;
	case SLIC_POWER_ON:
		ret = si3217x_pbx_init(si3217x);
		break;
	case SLIC_POWER_OFF:
		ret = si3217x_pbx_exit(si3217x);
		break;
	case SLIC_POWER_RESET:
		si3217x_pbx_exit(si3217x);
		ret = si3217x_pbx_init(si3217x);
		break;
	case SLIC_POWER_SUSPEND:
		ret = si3217x_pcm_disable(si3217x);
		break;
	case SLIC_POWER_RESUME:
		ret = si3217x_pcm_enable(si3217x);
		break;
	case SLIC_DAA_ONHOOK:
		Vdaa_SetHookStatus(si3217x->chan, VDAA_ONHOOK);
		break;
	case SLIC_DAA_OFFHOOK:
		Vdaa_SetHookStatus(si3217x->chan, VDAA_OFFHOOK);
		break;
	case SLIC_CONFIG_FLASH_TIME:
		si3217x->fxs_config.flash_time_max = *val;
		break;
	case SLIC_CONFIG_RX_TIMESLOT:
		si3217x->fxs_config.audio_gain_rx= *val;
		break;
	case SLIC_CONFIG_TX_TIMESLOT:
		si3217x->fxs_config.audio_gain_tx= *val;
		break;
	case SLIC_CONFIG_PCM:
		si3217x->fxs_config.pcm_config= *val;
		break;
	case SLIC_CONFIG_RING:
		si3217x->fxs_config.ring= *val;
		break;
	case SLIC_CONFIG_DCFEED:
		si3217x->fxs_config.dcfeed= *val;
		break;
	case SLIC_CONFIG_IMPEDANCE:
		si3217x->fxs_config.impedance= *val;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&si3217x->lock);

#ifdef CONFIG_SLIC_IRQ_POLL
	schedule_delayed_work(&si3217x_poll_data.work, 0);
#endif
	return ret;
}

static int si3217x_spi_probe(struct spi_device *spi)
{
	const struct slic_platform_data *pdata = spi->dev.platform_data;
	struct proslic_dev *proslic, *si3217x;
	struct slic_dev *slic;
	int i, ret = 0;
	int rst_gpio, irq_gpio, power_gpio;
	int irq_gpio_cp;
	int num_slic = 0;

        pr_info("si3217x_spi_probe is enter 1102_1....................................\n");

	if (pdata) {
		rst_gpio = pdata->rst_gpio;
		irq_gpio = pdata->irq_gpio;
                power_gpio = pdata->power_gpio;
		num_slic = pdata->num_slic;
	} else {
		rst_gpio = of_get_named_gpio(spi->dev.of_node, "rst-gpio", 0);
		irq_gpio = of_get_named_gpio(spi->dev.of_node, "irq-gpio", 0);
                power_gpio =  of_get_named_gpio(spi->dev.of_node, "power-gpio", 0);
		irq_gpio_cp = of_get_named_gpio(spi->dev.of_node, "irq-gpio-cp", 0);
		of_property_read_u32(spi->dev.of_node, "num_slic", &num_slic);

	}
	if (num_slic <= 0) {
		dev_err(&spi->dev, "%s: slic number <= 0\n", __func__);
		return -EINVAL;
	}
	dev_info(&spi->dev,
		"irq_gpio is %d, irq_gpio_cp is %d,rst_gpio is %d, num_slic is %d\n",
		irq_gpio,irq_gpio_cp, rst_gpio, num_slic);

#if 1
        if (power_gpio >= 0) {
            ret = gpio_request(power_gpio, "si3217x power");
            if (ret) {
                dev_err(&spi->dev, "%s : request power gpio failed\n", __func__);
                return -EINVAL;
            }
            gpio_direction_output(power_gpio, 1);  //only test 20161028:high is valid
        }
        else {
            dev_err(&spi->dev, "%s : power gpio < 0\n", __func__);
            return -EINVAL;
        }
#endif

	if (rst_gpio >= 0) {
		ret = gpio_request(rst_gpio, "si3217x reset");
		if (ret) {
			dev_err(&spi->dev,
				"%s: request rst gpio failed\n", __func__);
			return -EINVAL;
		}
		gpio_direction_output(rst_gpio, SLIC_RESET_DISABLE);
	} else {
		dev_err(&spi->dev, "%s: reset gpio < 0\n", __func__);
		return -EINVAL;
	}

	if (irq_gpio >= 0) {
		ret = gpio_request(irq_gpio, "si3217x irq");
		if (ret) {
			dev_err(&spi->dev,
				"%s: request irq gpio failed\n", __func__);
			ret = -EINVAL;
			goto err_gpio;
		}
		gpio_direction_input(irq_gpio);
	} else {
		dev_err(&spi->dev, "%s: irq gpio < 0\n", __func__);
		//return -EINVAL;
	}

	if (irq_gpio_cp >= 0) {
		ret = gpio_request(irq_gpio_cp, "si3217x irq_cp");
		if (ret) {
			dev_err(&spi->dev,
				"%s: request irq gpio cp failed\n", __func__);
			ret = -EINVAL;
			goto err_gpio;
		}
		gpio_direction_input(irq_gpio_cp);
	} else {
		dev_err(&spi->dev, "%s: irq gpio cp < 0\n", __func__);
		//return -EINVAL;
	}
	if (irq_gpio < 0&&irq_gpio_cp<0)
	{
		return -EINVAL;
	}

	proslic = kcalloc(num_slic, sizeof(struct proslic_dev), GFP_KERNEL);
	if (!proslic) {
		dev_err(&spi->dev, "%s: failed to alloc memory\n", __func__);
		ret = -ENOMEM;
		goto err_mem;
	}
	proslic->initialized = 0;
	spi_set_drvdata(spi, proslic);

	for (i = 0; i < num_slic; i++) {
		si3217x = &proslic[i];
		si3217x->spi = spi;
		si3217x->num_slic = num_slic;
		if (irq_gpio >= 0)
		    si3217x->irq_gpio = irq_gpio;

		if (irq_gpio_cp >= 0)
		    si3217x->irq_gpio_cp = irq_gpio_cp;
		si3217x->rst_gpio = rst_gpio;
		si3217x->type = SI3217X_TYPE;
		ret = si3217x_create_proslic(si3217x, i);
		if (ret) {
			dev_err(&spi->dev, "failed to alloc proslic memory\n");
			ret = -ENOMEM;
			goto err_create;
		}
		si3217x->tone_work.proslic = si3217x;
		si3217x->state.onhook_timestamp = jiffies;
		INIT_DELAYED_WORK(&(si3217x->hook_work), si3217x_is_onhook);
		INIT_DELAYED_WORK(&(si3217x->dtmf_work), si3217x_dtmf_work);
		INIT_DELAYED_WORK(&(si3217x->tone_work.work), si3217x_play_tone);
		atomic_set(&si3217x->cid_ack, 0);
		init_waitqueue_head(&si3217x->fsk_wait);
		init_waitqueue_head(&si3217x->ack_wait);
		mutex_init(&si3217x->lock);
		proslic_config_init(&si3217x->fxs_config, &si3217x->fxo_config);
	}

	for (i = 0; i < num_slic; i++) {
		slic = &(proslic[i].sdev);
		slic->control_data = proslic;
		slic->id = i;
		slic->ioctl = si3217x_slic_ioctl;
		ret = slic_dev_register(&spi->dev, slic);
		if (ret) {
			dev_err(&spi->dev, "register slic dev failed\n");
			goto err_device;
		}
	}

	INIT_DELAYED_WORK(&(si3217x_irq_data.work), si3217x_irq_work);
	si3217x_irq_data.proslic = proslic;
	si3217x_irq_data.irq_err = 0;
	spin_lock_init(&si3217x_irq_spinlock);

#ifdef CONFIG_SLIC_IRQ_POLL
	INIT_DELAYED_WORK(&(si3217x_poll_data.work), si3217x_poll_work);
	si3217x_poll_data.proslic = proslic;
	schedule_delayed_work(&si3217x_poll_data.work,
			msecs_to_jiffies(IRQ_POLL_PERIOD));
#endif
	dev_info(&spi->dev, "%s done", __func__);
	return ret;

err_device:
	while(i)
		slic_dev_unregister(&(proslic[--i].sdev));
	for (i = 0; i < num_slic; i++)
		si3217x_destory_proslic(&proslic[i]);
err_create:
	while(i)
		si3217x_destory_proslic(&proslic[--i]);
	kfree(proslic);
err_mem:
	gpio_free(irq_gpio);
	gpio_free(irq_gpio_cp);
err_gpio:
	gpio_free(rst_gpio);
	return ret;
}

static int si3217x_spi_remove(struct spi_device *spi)
{
	struct proslic_dev *proslic = spi_get_drvdata(spi);
	int i, irq = gpio_to_irq(proslic->irq_gpio);
       int irq_cp = gpio_to_irq(proslic->irq_gpio_cp);
	free_irq(irq, proslic);
	free_irq(irq_cp, proslic);
	flush_scheduled_work();
	for (i = 0; i < proslic->num_slic; i++) {
		slic_dev_unregister(&(proslic[i].sdev));
		si3217x_destory_proslic(&proslic[i]);
	}
	kfree(proslic);
	gpio_free(proslic->irq_gpio);
	gpio_free(proslic->irq_gpio_cp);
	gpio_free(proslic->rst_gpio);
	dev_info(&spi->dev, "%s", __func__);
	return 0;
}


static struct of_device_id slic_match_table[] = {
{ .compatible = "si3217x", },
{ },
};
MODULE_DEVICE_TABLE(of, slic_match_table);

static struct spi_driver si3217x_spi_driver = {
	.driver = {
		.name    = "si3217x",
		.owner    = THIS_MODULE,
        .of_match_table	= slic_match_table,
	},
	.probe    = si3217x_spi_probe,
	.remove    = si3217x_spi_remove,
};

static int __init si3217x_init(void)
{
	int ret = 0;
    pr_info("si3217x_init is enter 2016_101....................................\n");

	ret = spi_register_driver(&si3217x_spi_driver);
	if (ret) {
		pr_err("register si3217x spi driver failed\n");
		ret = -ENODEV;
		return ret;
	}
	return ret;
}

static void __exit si3217x_exit(void)
{
	spi_unregister_driver(&si3217x_spi_driver);
	pr_info("remove si3217x spi driver\n");
}

module_init(si3217x_init);
module_exit(si3217x_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("HaoZhiwei <hao.zhiwei@zte.com.cn>");
