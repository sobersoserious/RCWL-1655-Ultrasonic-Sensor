/**
 * @file   rcwl1655.c
 * @author zgs
 * @brief  超声波传感器驱动
 * @version 2.1
 * @date   2025-12-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/types.h>

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/version.h> 
#include <linux/mod_devicetable.h> 
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/platform_device.h>

#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/idr.h>

#include "rcwl1655.h"

#define RCWL1655_NAME   "rcwl1655"
#define RCWL1655_MAX_DEVICES 32

static DEFINE_IDA(rcwl1655_ida);
static dev_t rcwl1655_base_devid;
static struct class *rcwl1655_class;

enum rcwl1655_state {
    RCWL_IDLE,
    RCWL_MEASURING,
};
struct rcwl1655_dev {
    struct i2c_client *client;
    struct mutex lock;
    
    int id;
    dev_t devid;
    struct cdev cdev;
    struct device *device;

    u32 distance_raw;
    u32 frame_id;
    u64 last_sample_ns;

    struct delayed_work poll_work;
    wait_queue_head_t wq;

    enum rcwl1655_state state;
    bool RCWL_RUNNING;
    int open_count;

    u32 poll_interval_ms;
};

static ssize_t poll_interval_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct rcwl1655_dev *rcwl_dev = dev_get_drvdata(dev);

    return sysfs_emit(buf, "%u\n", rcwl_dev->poll_interval_ms);
}

static ssize_t poll_interval_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct rcwl1655_dev *rcwl_dev = dev_get_drvdata(dev);
    unsigned long val;
    int ret;

    ret = kstrtoul(buf, 10, &val);
    if (ret < 0) {
        return ret;
    }

    if (val < 5 || val > 1000) {
        return -EINVAL;
    }

    mutex_lock(&rcwl_dev->lock);
    rcwl_dev->poll_interval_ms = val;
    mutex_unlock(&rcwl_dev->lock);

    return count;
}

static ssize_t distance_raw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct rcwl1655_dev *rcwl_dev = dev_get_drvdata(dev);
    u32 val;

    mutex_lock(&rcwl_dev->lock);
    val = rcwl_dev->distance_raw;
    mutex_unlock(&rcwl_dev->lock);

    return sysfs_emit(buf, "%u\n", val);
}

static DEVICE_ATTR_RW(poll_interval);
static DEVICE_ATTR_RO(distance_raw);

static struct attribute *rcwl1655_attrs[] = {
    &dev_attr_poll_interval.attr,
    &dev_attr_distance_raw.attr,
    NULL,
};

static const struct attribute_group rcwl1655_attr_group = {
    .attrs = rcwl1655_attrs,
};

static int rcwl1655_write_cmd(struct i2c_client *client, u8 cmd)
{
    int ret;
    
    ret = i2c_master_send(client, &cmd, 1);
    if (ret < 0)
      return ret;

    if (ret != 1) {
        return -EIO;
    }   

    return 0;
}

static int rcwl1655_start(struct rcwl1655_dev *rcwl_dev)
{
    int ret;

    mutex_lock(&rcwl_dev->lock);
    if (!rcwl_dev->RCWL_RUNNING) {
        ret = -ESHUTDOWN;
        goto out_unlock;
    }

    ret = rcwl1655_write_cmd(rcwl_dev->client, RCWL1655_READ_CMD);
    if (ret < 0) {
        dev_err(&rcwl_dev->client->dev, "write command failed: %d\n", ret);
        goto out_unlock;
    }

    rcwl_dev->state = RCWL_MEASURING;
    schedule_delayed_work(&rcwl_dev->poll_work, msecs_to_jiffies(rcwl_dev->poll_interval_ms));

out_unlock:
    mutex_unlock(&rcwl_dev->lock);

    return ret;
}

static void rcwl1655_poll_work(struct work_struct *work)
{
    struct rcwl1655_dev *rcwl_dev = container_of(to_delayed_work(work), struct rcwl1655_dev, poll_work);
    int ret;
    u8 buf[3];

    mutex_lock(&rcwl_dev->lock);
    if (rcwl_dev->RCWL_RUNNING != true) {
        mutex_unlock(&rcwl_dev->lock);
        return;
    }

    if (rcwl_dev->state == RCWL_IDLE) {
        mutex_unlock(&rcwl_dev->lock);
        ret = rcwl1655_start(rcwl_dev);
        if (ret < 0) {
            mutex_lock(&rcwl_dev->lock);
            if (rcwl_dev->RCWL_RUNNING)
                schedule_delayed_work(&rcwl_dev->poll_work, msecs_to_jiffies(rcwl_dev->poll_interval_ms));
            mutex_unlock(&rcwl_dev->lock);
        }
        return;
    }

    ret = i2c_master_recv(rcwl_dev->client, buf, sizeof(buf));
    if (ret != sizeof(buf)) {
        schedule_delayed_work(&rcwl_dev->poll_work, msecs_to_jiffies(rcwl_dev->poll_interval_ms));
        mutex_unlock(&rcwl_dev->lock);
        return;
    }

    rcwl_dev->distance_raw = (buf[0] << 16) | (buf[1] << 8) | buf[2];
    rcwl_dev->frame_id++;
    rcwl_dev->last_sample_ns = ktime_get_ns();
    rcwl_dev->state = RCWL_IDLE;
    wake_up_interruptible(&rcwl_dev->wq);
    schedule_delayed_work(&rcwl_dev->poll_work, msecs_to_jiffies(rcwl_dev->poll_interval_ms));
    mutex_unlock(&rcwl_dev->lock);
}

static int rcwl1655_open(struct inode *inode, struct file *filp)
{
    struct rcwl1655_dev *rcwl_dev = container_of(inode->i_cdev, struct rcwl1655_dev, cdev);
    int ret;

    filp->private_data = rcwl_dev;

    mutex_lock(&rcwl_dev->lock);
    if (rcwl_dev->open_count != 0) {
        mutex_unlock(&rcwl_dev->lock);
        pr_err("%s: open failed. \n", __func__);
        return -EBUSY;
    }
    rcwl_dev->open_count = 1;
    rcwl_dev->RCWL_RUNNING = true;
    rcwl_dev->frame_id = 0;
    rcwl_dev->last_sample_ns = 0;
    mutex_unlock(&rcwl_dev->lock);

    ret = rcwl1655_start(rcwl_dev);
    if (ret < 0) {
        mutex_lock(&rcwl_dev->lock);
        rcwl_dev->open_count = 0;
        rcwl_dev->RCWL_RUNNING = false;
        rcwl_dev->state = RCWL_IDLE;
        mutex_unlock(&rcwl_dev->lock);
        pr_err("%s: rcwl1655 start failed.\n", __func__);
        return ret;
    }

    return 0;
}

static ssize_t rcwl1655_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
    struct rcwl1655_dev *rcwl_dev = filp->private_data;
    struct rcwl1655_sample sample;
    int ret;
    unsigned long uncopied;

    if (cnt < sizeof(sample)) {
        dev_err(&rcwl_dev->client->dev, "%s: read buffer too small\n", __func__);
        return -EINVAL;
    }

    mutex_lock(&rcwl_dev->lock);
    if (rcwl_dev->RCWL_RUNNING != true) {
        mutex_unlock(&rcwl_dev->lock);
        dev_err(&rcwl_dev->client->dev, "%s: device already stopped\n", __func__);
        return -EFAULT;
    }
    mutex_unlock(&rcwl_dev->lock);

    ret = wait_event_interruptible(rcwl_dev->wq, rcwl_dev->state == RCWL_IDLE);
    if (ret < 0) {
        dev_err(&rcwl_dev->client->dev, "%s: wait interrupted\n", __func__);
        return ret;
    }

    mutex_lock(&rcwl_dev->lock);
    sample.distance_raw = rcwl_dev->distance_raw;
    sample.frame_id = rcwl_dev->frame_id;
    sample.timestamp_ns = rcwl_dev->last_sample_ns;
    mutex_unlock(&rcwl_dev->lock);

    uncopied = copy_to_user(buf, &sample, sizeof(sample));
    if (uncopied != 0) {
        dev_err(&rcwl_dev->client->dev, "%s: copy_to_user error\n", __func__);
        return -EFAULT;
    }

    return sizeof(sample);
}

static int rcwl1655_release(struct inode *inode, struct file *filp)
{
    struct rcwl1655_dev *rcwl_dev = filp->private_data;

    mutex_lock(&rcwl_dev->lock);
    rcwl_dev->open_count = 0;
    rcwl_dev->RCWL_RUNNING = false;
    mutex_unlock(&rcwl_dev->lock);

    wake_up_interruptible_all(&rcwl_dev->wq);
    cancel_delayed_work_sync(&rcwl_dev->poll_work);

    filp->private_data = NULL;

    return 0;
}

static __poll_t rcwl1655_poll(struct file *filp, struct poll_table_struct *wait)
{
    struct rcwl1655_dev *rcwl_dev = filp->private_data;
    unsigned int mask = 0;
    
    poll_wait(filp, &rcwl_dev->wq, wait);
    
    mutex_lock(&rcwl_dev->lock);
    if (rcwl_dev->state == RCWL_IDLE) {
        mask |= POLLIN | POLLRDNORM;
    }
    mutex_unlock(&rcwl_dev->lock);

    return mask;
}

static const struct file_operations rcwl1655_ops = {
	.owner = THIS_MODULE,
	.open = rcwl1655_open,
	.read = rcwl1655_read,
	.release = rcwl1655_release,
    .poll = rcwl1655_poll,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static int rcwl1655_i2c_probe(struct i2c_client *client)
#else
static int rcwl1655_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
    int ret;
    struct rcwl1655_dev *rcwl_dev = NULL;
    const char *label = NULL;
    bool has_label;
    const char *name;

    rcwl_dev = devm_kzalloc(&client->dev, sizeof(*rcwl_dev), GFP_KERNEL);
    if (!rcwl_dev) {
        dev_err(&client->dev, "%s: kzalloc err.\n", __func__);
        return -ENOMEM;
    }

    rcwl_dev->id = ida_alloc_max(&rcwl1655_ida, RCWL1655_MAX_DEVICES - 1, GFP_KERNEL);
    if (rcwl_dev->id < 0) {
        dev_err(&client->dev, "%s: ida_alloc_max Failed\n", __func__);
        ret = rcwl_dev->id;
        return ret;
    }

    has_label = !of_property_read_string(client->dev.of_node, "label", &label);
    if (has_label)
        name = devm_kasprintf(&client->dev, GFP_KERNEL, "%s", label);
    else
        name = devm_kasprintf(&client->dev, GFP_KERNEL, "rcwl1655-%d", rcwl_dev->id);

    rcwl_dev->client = client;
    mutex_init(&rcwl_dev->lock);

    init_waitqueue_head(&rcwl_dev->wq);
    INIT_DELAYED_WORK(&rcwl_dev->poll_work, rcwl1655_poll_work);
    rcwl_dev->RCWL_RUNNING = false;
    rcwl_dev->state = RCWL_IDLE;
    rcwl_dev->poll_interval_ms = 10;
    rcwl_dev->open_count = 0;
    rcwl_dev->distance_raw = 0;
    rcwl_dev->frame_id = 0;
    rcwl_dev->last_sample_ns = 0;

    rcwl_dev->devid = MKDEV(MAJOR(rcwl1655_base_devid), rcwl_dev->id);

    /* Registered cdev */
    cdev_init(&rcwl_dev->cdev, &rcwl1655_ops);
    ret = cdev_add(&rcwl_dev->cdev, rcwl_dev->devid, 1);
    if (ret < 0) {
        dev_err(&client->dev, "%s: cdev_add Failed\n", __func__);
        goto err_free_ida;
    }

    /* created device */
    rcwl_dev->device = device_create(rcwl1655_class, &client->dev, rcwl_dev->devid, rcwl_dev, "%s", name);
    if (IS_ERR(rcwl_dev->device)) {
        ret = PTR_ERR(rcwl_dev->device);
        dev_err(&client->dev, "%s: device_create Failed\n", __func__);
        goto err_cdev_del;
    }

    dev_set_drvdata(rcwl_dev->device, rcwl_dev);

    ret = sysfs_create_group(&rcwl_dev->device->kobj, &rcwl1655_attr_group);
    if (ret < 0) {
        dev_err(&client->dev, "%s: sysfs_create_group Failed\n", __func__);
        goto err_device_destroy;
    }

    i2c_set_clientdata(client, rcwl_dev);

    dev_info(&client->dev, "%s: Succeed (id=%d)\n", __func__, rcwl_dev->id);
    return 0;

// err_sysfs_remove:
//     sysfs_remove_group(&rcwl_dev->device->kobj, &rcwl1655_attr_group);
err_device_destroy:
    device_destroy(rcwl1655_class, rcwl_dev->devid);
err_cdev_del:
    cdev_del(&rcwl_dev->cdev);
err_free_ida:
    ida_free(&rcwl1655_ida, rcwl_dev->id);
    return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void rcwl1655_i2c_remove(struct i2c_client *client)
#else
static int rcwl1655_i2c_remove(struct i2c_client *client)
#endif
{
    struct rcwl1655_dev *rcwl_dev = i2c_get_clientdata(client);

    mutex_lock(&rcwl_dev->lock);
    rcwl_dev->RCWL_RUNNING = false;
    mutex_unlock(&rcwl_dev->lock);

    wake_up_interruptible_all(&rcwl_dev->wq);
    cancel_delayed_work_sync(&rcwl_dev->poll_work);

    sysfs_remove_group(&rcwl_dev->device->kobj, &rcwl1655_attr_group);

    device_destroy(rcwl1655_class, rcwl_dev->devid);
    cdev_del(&rcwl_dev->cdev);
    ida_free(&rcwl1655_ida, rcwl_dev->id);

    dev_info(&client->dev, "%s: I2C Removed (id=%d)\n", __func__, rcwl_dev->id);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
    return 0;
#endif
}

static const struct of_device_id rcwl1655_of_match[] = {
    { .compatible = "rcwl,rcwl1655" },
    { /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, rcwl1655_of_match);

static const struct i2c_device_id rcwl1655_id[] = {
    { RCWL1655_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, rcwl1655_id);

static struct i2c_driver rcwl1655_driver = {
    .probe = rcwl1655_i2c_probe,
    .remove = rcwl1655_i2c_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = RCWL1655_NAME,
        .of_match_table = rcwl1655_of_match,
    },
    .id_table = rcwl1655_id,
};

static int __init rcwl1655_init_driver(void)
{
    int ret;

    ret = alloc_chrdev_region(&rcwl1655_base_devid, 0, RCWL1655_MAX_DEVICES, RCWL1655_NAME);
    if (ret < 0) {
        pr_err("rcwl1655: alloc_chrdev_region Failed\n");
        return ret;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
    rcwl1655_class = class_create(RCWL1655_NAME);
#else
    rcwl1655_class = class_create(THIS_MODULE, RCWL1655_NAME);
#endif
    if (IS_ERR(rcwl1655_class)) {
        ret = PTR_ERR(rcwl1655_class);
        rcwl1655_class = NULL;
        pr_err("%s: class create failed.\n", __func__);
        goto err_unregister_chrdev;
    }

    ret = i2c_add_driver(&rcwl1655_driver);
    if (ret < 0) {
        pr_err("%s: i2c add failed.\n", __func__);
        goto err_class_destroy;
    }
    return ret;

err_class_destroy:
    class_destroy(rcwl1655_class);
    rcwl1655_class = NULL;
err_unregister_chrdev:
    unregister_chrdev_region(rcwl1655_base_devid, RCWL1655_MAX_DEVICES);
    return ret;
}

static void __exit rcwl1655_exit_driver(void)
{
    i2c_del_driver(&rcwl1655_driver);
    class_destroy(rcwl1655_class);
    unregister_chrdev_region(rcwl1655_base_devid, RCWL1655_MAX_DEVICES);
}

module_init(rcwl1655_init_driver);
module_exit(rcwl1655_exit_driver);
// module_i2c_driver(rcwl1655_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zgs");
MODULE_DESCRIPTION("RCWL1655 sensor driver");
