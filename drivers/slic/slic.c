/*
**slic driver
**Author:haozhiwei
**date:2015-02-28
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/uaccess.h>
#include <linux/signal.h>
#include <linux/delay.h>
#include <linux/slic.h>
//#include <linux/zte_bsp_netlink.h>

static int slic_major = 0,g_key_value=0;;
static struct class *slic_class;

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static ssize_t slic_ioctl_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct slic_dev *slic = dev_get_drvdata(dev);
    int number = -1;
    unsigned int ctl_cmd;
    int val = 0;
    u32 val2 = 0;
    struct slic_mem_cmd mem_data;
    struct caller_id_data cid_data;
    int ret = 0;

    if (sscanf(buf, "%d %d %x", &number, &val, &val2) < 1)
        return -EINVAL;

    if (slic->ioctl) {
        ctl_cmd = _IO(SLIC_IOC_MAGIC, number);
        switch (ctl_cmd) {
        case SLIC_RING_START:
        case SLIC_CID_OFFHOOK:
            memset(&cid_data, 0, sizeof(struct caller_id_data));
            cid_data.cid_type = val;
            memcpy(cid_data.date, "12120001", 8);
            memcpy(cid_data.number, "0123456789", 10);
            memcpy(cid_data.name, "ProSLIC CALLING", 15);
            ret = slic->ioctl(slic, ctl_cmd, &cid_data);
            break;
        case SLIC_SET_REG:
        case SLIC_SET_RAM:
            mem_data.addr = val;
            mem_data.value = val2;
            ret = slic->ioctl(slic, ctl_cmd, &mem_data);
            break;
        case SLIC_GET_REG:
        case SLIC_GET_RAM:
            mem_data.addr = val;
            ret = slic->ioctl(slic, ctl_cmd, &mem_data);
            pr_info("command get value 0x%0x\n", mem_data.value);
            break;
        case SLIC_GET_EVENT:
        case SLIC_GET_TYPE:
        case SLIC_GET_STATUS:
            ret = slic->ioctl(slic, ctl_cmd, &val);
            pr_info("command get value %d\n", val);
            break;
		case SLIC_VM_START:
        case SLIC_VM_STOP:
			memset(&cid_data, 0, sizeof(struct caller_id_data));
			ret = slic->ioctl(slic, ctl_cmd, &cid_data);
			pr_info("VM command %d\n", ctl_cmd);
			break;
        default:
            ret = slic->ioctl(slic, ctl_cmd, &val);
            break;
        }
        if (ret)
            pr_info("command %d return %d\n", number, ret);
    }
    return size;
}

static ssize_t slic_ioctl_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    return 0;
}

static ssize_t slic_key_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size);

static ssize_t slic_key_show(struct device *dev,
                        struct device_attribute *attr,
                        char *buf);



static struct device_attribute slic_class_attrs[] = {
    __ATTR(ioctl, 0644, slic_ioctl_show, slic_ioctl_store),
        __ATTR(key, 0644, slic_key_show, slic_key_store),
};

void slic_init_attrs(struct slic_dev *slic)
{
    int i, ret;

    for (i = 0; i < ARRAY_SIZE(slic_class_attrs); i++) {
        ret = device_create_file(slic->dev, &slic_class_attrs[i]);
        if (ret)
            dev_err(slic->dev, "Failed to create attribute %s\n",
                        slic_class_attrs[i].attr.name);
    }
}

static ssize_t slic_key_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct slic_dev *slic = dev_get_drvdata(dev);
    char cmd[size];

    scnprintf(cmd, size, buf);
    dev_info(dev, "cmd : %s\n", cmd);

    if (!strncmp(cmd, "FF", strlen("FF"))) {
        g_key_value=SLIC_EVENT_MAX;
        sysfs_notify(&slic->dev->kobj, NULL, slic_class_attrs[1].attr.name);
    }

    return 0;
}

static ssize_t slic_key_show(struct device *dev,
                        struct device_attribute *attr,
                        char *buf)
{
    int tmp_value=g_key_value;
    g_key_value=0;
    return scnprintf(buf, PAGE_SIZE, "%i\n", tmp_value);
}



static int slic_netlink_send(struct slic_dev * slic,struct slic_event *event)
{
    g_key_value=event->event_type;
    sysfs_notify(&slic->dev->kobj, NULL, slic_class_attrs[1].attr.name);
    return 0;
}

int slic_report_event(struct slic_dev * slic, int event_type)
{
    struct slic_event event;

    mutex_lock(&slic->event_lock);
    slic->phone_event = event_type;
    event.dev_id = slic->id;
    event.event_type = event_type;
    slic_netlink_send(slic,&event);
    mutex_unlock(&slic->event_lock);
    //if (slic->async_queue)
    //  kill_fasync(&slic->async_queue, SIGIO, POLL_IN);
    return 0;
}
EXPORT_SYMBOL_GPL(slic_report_event);

int slic_fasync(int fd, struct file *filp, int mode)
{
    struct slic_dev *slic = filp->private_data;
    return fasync_helper(fd, filp, mode, &slic->async_queue);
}

int slic_open(struct inode *inode, struct file *filp)
{
    struct slic_dev *slic;
    int status = -ENXIO;

    mutex_lock(&device_list_lock);
    list_for_each_entry(slic, &device_list, device_entry) {
        if (slic->dev->devt == inode->i_rdev) {
            status = 0;
            break;
        }
    }
    if (status == 0) {
        filp->private_data = slic;
        nonseekable_open(inode, filp);
        status = -ENODEV;
        if (slic->ioctl)
            status = slic->ioctl(slic, SLIC_POWER_ON, 0);
    }
    mutex_unlock(&device_list_lock);
    return status;
}

int slic_release(struct inode *inode, struct file *filp)
{
    slic_fasync(-1, filp, 0);
    filp->private_data = NULL;
    return 0;
}

long slic_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
    struct slic_dev *slic = filp->private_data;
    long ret = -EPERM;
    struct slic_mem_cmd mem_data;
    struct caller_id_data cid_data;
    int val;

    if (slic->ioctl) {
        switch (cmd) {
        case SLIC_RING_START:
        case SLIC_CID_OFFHOOK:
        case SLIC_VM_START:
        case SLIC_VM_STOP:
            memset(&cid_data, 0, sizeof(struct caller_id_data));
            if (copy_from_user(&cid_data, (unsigned char __user*)arg,
                    sizeof(struct caller_id_data))) {
                ret = -EFAULT;
            } else {
                ret = slic->ioctl(slic, cmd, &cid_data);
            }
            break;
        case SLIC_SET_REG:
        case SLIC_SET_RAM:
        case SLIC_GET_REG:
        case SLIC_GET_RAM:
            memset(&mem_data, 0, sizeof(struct slic_mem_cmd));
            if (copy_from_user(&mem_data, (void __user*)arg,
                    sizeof(struct slic_mem_cmd))) {
                ret = -EFAULT;
            } else {
                ret = slic->ioctl(slic, cmd, &mem_data);
                if (ret)
                    break;
            }
            ret = copy_to_user((void __user *)arg, &mem_data,
            sizeof(struct slic_mem_cmd));
            break;
        case SLIC_GET_EVENT:
        case SLIC_GET_TYPE:
        case SLIC_GET_STATUS:
            ret = slic->ioctl(slic, cmd, &val);
            if (ret)
                break;
            ret = put_user(val, (int __user *)arg);
            break;
        default:
            val = (int) arg;
            ret = slic->ioctl(slic, cmd, &val);
            break;
        }
    }
    return ret;
}

struct file_operations slic_fops = {
    .owner   = THIS_MODULE,
    .open    = slic_open,
    .release = slic_release,
    .unlocked_ioctl  = slic_ioctl,
    .fasync  = slic_fasync,
};

static int slic_suspend(struct device *dev, pm_message_t state)
{
    struct slic_dev *slic = dev_get_drvdata(dev);
    int ret = 0;

    if (slic->ioctl)
        ret = slic->ioctl(slic, SLIC_POWER_SUSPEND, 0);
    return ret;
}

static int slic_resume(struct device *dev)
{
    struct slic_dev *slic = dev_get_drvdata(dev);
    int ret = 0;

    if (slic->ioctl)
        ret = slic->ioctl(slic, SLIC_POWER_RESUME, 0);
    return ret;
}

int slic_dev_register(struct device *parent, struct slic_dev * slic)
{
    slic->dev = device_create(slic_class, parent, MKDEV(slic_major, slic->id), slic,
                "slic-%d", slic->id);
    if (IS_ERR(slic->dev))
        return PTR_ERR(slic->dev);

    INIT_LIST_HEAD(&slic->device_entry);
    mutex_lock(&device_list_lock);
    list_add(&slic->device_entry, &device_list);
    mutex_unlock(&device_list_lock);

    mutex_init(&slic->event_lock);
    slic_init_attrs(slic);
    dev_dbg(slic->dev, "Registered slic device %ul(%d,%d)\n", slic->dev->devt, slic_major, slic->id);
    return 0;
}
EXPORT_SYMBOL_GPL(slic_dev_register);

void slic_dev_unregister(struct slic_dev *slic)
{
    mutex_destroy(&slic->event_lock);
    mutex_lock(&device_list_lock);
    list_del(&slic->device_entry);
    mutex_unlock(&device_list_lock);
    device_destroy(slic_class, slic->dev->devt);
}
EXPORT_SYMBOL_GPL(slic_dev_unregister);

static int __init slic_init(void)
{
    int ret = 0;

    slic_major = register_chrdev(0, "slic", &slic_fops);
    if (slic_major < 0) {
        pr_err("%s : can't get major, err %d\n", __func__, slic_major);
        return -ENODEV;
    }

    slic_class = class_create(THIS_MODULE, "slic");
    if (IS_ERR(slic_class)) {
        pr_err("%s : register slic class failed\n",__func__);
        ret =  PTR_ERR(slic_class);
        goto error_class;
    }
    slic_class->suspend = slic_suspend;
    slic_class->resume = slic_resume;
    return ret;

error_class:
    unregister_chrdev(slic_major,"slic");
    return ret;
}

static void __exit slic_exit(void)
{
    class_destroy(slic_class);
    unregister_chrdev(slic_major,"slic");
}

subsys_initcall(slic_init);
module_exit(slic_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("HaoZhiwei <hao.zhiwei@zte.com.cn>");
