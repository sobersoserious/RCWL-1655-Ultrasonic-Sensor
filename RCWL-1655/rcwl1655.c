/**
 * @file   rcwl1655.c
 * @author zgs
 * @brief  超声波传感器驱动
 * @version 1.0
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

#include "rcwl1655.h"

#define RCWL1655_NAME   "rcwl1655"

enum rcwl1655_state {
    RCWL_IDLE,
    RCWL_MEASURING,
};

struct rcwl1655_dev {
    struct i2c_client *client;
    struct mutex lock;

    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;

    u32 distance_raw;

    struct delayed_work poll_work;
    wait_queue_head_t wq;

    enum rcwl1655_state state;
    bool RCWL_RUNNING;

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
    if (ret != 1) {
        return -EIO;
    }

    return 0;
}

static int rcwl1655_start(struct rcwl1655_dev *rcwl_dev)
{
    int ret;

    mutex_lock(&rcwl_dev->lock);
    if (rcwl_dev->RCWL_RUNNING != true) {
        rcwl_dev->RCWL_RUNNING = true;
        ret = rcwl1655_write_cmd(rcwl_dev->client, RCWL1655_READ_CMD);
        if (ret < 0) {
            mutex_unlock(&rcwl_dev->lock);
            pr_err("rcwl1655_write_cmd first error\n");
            return -EIO;
        }
        rcwl_dev->state = RCWL_MEASURING;
        schedule_delayed_work(&rcwl_dev->poll_work, msecs_to_jiffies(rcwl_dev->poll_interval_ms));
    }
    else if (rcwl_dev->RCWL_RUNNING == true) {
        ret = rcwl1655_write_cmd(rcwl_dev->client, RCWL1655_READ_CMD);
        if (ret < 0) {
            mutex_unlock(&rcwl_dev->lock);
            pr_err("rcwl1655_write_cmd error\n");
            return -EIO;
        }
        rcwl_dev->state = RCWL_MEASURING;
        schedule_delayed_work(&rcwl_dev->poll_work, msecs_to_jiffies(rcwl_dev->poll_interval_ms));
    }
    mutex_unlock(&rcwl_dev->lock);
    
    return 0;
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
        rcwl1655_start(rcwl_dev);
        return;
    }

    ret = i2c_master_recv(rcwl_dev->client, buf, 3);
    if (ret < 0) {
        schedule_delayed_work(&rcwl_dev->poll_work, msecs_to_jiffies(rcwl_dev->poll_interval_ms));
        mutex_unlock(&rcwl_dev->lock);
        pr_err("rcwl1655 i2c_master_recv error\n");
        return;
    }
    if (ret == 3) {
        rcwl_dev->distance_raw = (buf[0] << 16) | (buf[1] << 8) | buf[2];
        rcwl_dev->state = RCWL_IDLE;
        wake_up_interruptible(&rcwl_dev->wq);
        schedule_delayed_work(&rcwl_dev->poll_work, msecs_to_jiffies(rcwl_dev->poll_interval_ms));
        mutex_unlock(&rcwl_dev->lock);
        return;
    }
    mutex_unlock(&rcwl_dev->lock);
}

static int rcwl1655_open(struct inode *inode, struct file *filp)
{
    struct rcwl1655_dev *rcwl_dev = container_of(inode->i_cdev, struct rcwl1655_dev, cdev);
    filp->private_data = rcwl_dev;

    return 0;
}

static ssize_t rcwl1655_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
    struct rcwl1655_dev *rcwl_dev = filp->private_data;
    int ret;
    unsigned long uncopied;

    if (cnt < sizeof(rcwl_dev->distance_raw)) {
        pr_err("rcwl1655 recived cnt do not enough\n");
        return -EINVAL;
    }

    mutex_lock(&rcwl_dev->lock);
    if (rcwl_dev->RCWL_RUNNING != true) {
        mutex_unlock(&rcwl_dev->lock);
        pr_err("rcwl1655 is already Stopped\n");
        return -EFAULT;
    }
    mutex_unlock(&rcwl_dev->lock);

    ret = wait_event_interruptible(rcwl_dev->wq, rcwl_dev->state == RCWL_IDLE);
    if (ret < 0) {
        pr_err("rcwl1655 Sudden Stopped\n");
        return ret;
    }

    mutex_lock(&rcwl_dev->lock);
    uncopied = copy_to_user(buf, &rcwl_dev->distance_raw, sizeof(u32));
    if (uncopied != 0) {
        mutex_unlock(&rcwl_dev->lock);
        pr_err("rcwl1655 copy_to_user error\n");
        return -EFAULT;
    }
    mutex_unlock(&rcwl_dev->lock);

    pr_info("rcwl1655 copy_to_user success\n");

    return sizeof(u32);
}

static int rcwl1655_release(struct inode *inode, struct file *filp)
{
    filp->private_data = NULL;
    
    return 0;
}

static const struct file_operations rcwl1655_ops = {
	.owner = THIS_MODULE,
	.open = rcwl1655_open,
	.read = rcwl1655_read,
	.release = rcwl1655_release,
};

static int rcwl1655_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    struct rcwl1655_dev *rcwl_dev = NULL;

    rcwl_dev = devm_kzalloc(&client->dev, sizeof(*rcwl_dev), GFP_KERNEL);
    if (!rcwl_dev) {
        return -ENOMEM;
    }

    rcwl_dev->client = client;
    mutex_init(&rcwl_dev->lock);

    /* Registered cdevid */
    ret = alloc_chrdev_region(&rcwl_dev->devid, 0, 1, RCWL1655_NAME);
    if (ret < 0) {
        dev_err(&client->dev, "alloc_chrdev_region Failed\n");
        return ret;
    }

    /* Registered cdev */
    cdev_init(&rcwl_dev->cdev, &rcwl1655_ops);
    ret = cdev_add(&rcwl_dev->cdev, rcwl_dev->devid, 1);
    if (ret < 0) {
        dev_err(&client->dev, "cdev_add Failed\n");
        goto err_unregister_chrdev;
    }

    /* created class */
    rcwl_dev->class = class_create(THIS_MODULE, RCWL1655_NAME);
    if (IS_ERR(rcwl_dev->class)) {
        ret = PTR_ERR(rcwl_dev->class);
        dev_err(&client->dev, "class_create Failed\n");
        goto err_cdev_del;
    }

    /*creared device*/
    rcwl_dev->device = device_create(rcwl_dev->class, NULL, rcwl_dev->devid, NULL, RCWL1655_NAME);
    if (IS_ERR(rcwl_dev->device)) {
        ret = PTR_ERR(rcwl_dev->device);
        dev_err(&client->dev, "device_create Failed\n");
        goto err_class_destroy;
    }

    dev_set_drvdata(rcwl_dev->device, rcwl_dev);

    ret = sysfs_create_group(&rcwl_dev->device->kobj, &rcwl1655_attr_group);
    if (ret < 0) {
        dev_err(&client->dev, "sysfs_create_group Failed\n");
        goto err_device_destroy;
    }
    
    i2c_set_clientdata(client, rcwl_dev);

    init_waitqueue_head(&rcwl_dev->wq);
    INIT_DELAYED_WORK(&rcwl_dev->poll_work, rcwl1655_poll_work);
    rcwl_dev->RCWL_RUNNING = false;
    rcwl_dev->state = RCWL_IDLE;
    rcwl_dev->poll_interval_ms = 10;

    rcwl1655_start(rcwl_dev);

    dev_info(&client->dev, "rcwl1655_i2c_probe Succeed\n");
    return 0;

err_device_destroy:
    device_destroy(rcwl_dev->class, rcwl_dev->devid);
err_class_destroy:
    class_destroy(rcwl_dev->class);
err_cdev_del:
    cdev_del(&rcwl_dev->cdev);
err_unregister_chrdev:
    unregister_chrdev_region(rcwl_dev->devid, 1);
    return ret;
}

static int rcwl1655_i2c_remove(struct i2c_client *client)
{
    struct rcwl1655_dev *rcwl_dev = i2c_get_clientdata(client);    //unuse will make err

    mutex_lock(&rcwl_dev->lock);
    rcwl_dev->RCWL_RUNNING = false;
    mutex_unlock(&rcwl_dev->lock);

    wake_up_interruptible_all(&rcwl_dev->wq);
    cancel_delayed_work_sync(&rcwl_dev->poll_work);

    sysfs_remove_group(&rcwl_dev->device->kobj, &rcwl1655_attr_group);

    device_destroy(rcwl_dev->class, rcwl_dev->devid);
    class_destroy(rcwl_dev->class);
    cdev_del(&rcwl_dev->cdev);
    unregister_chrdev_region(rcwl_dev->devid, 1);

    dev_info(&client->dev, "RCWL1655 I2C Removed\n");

    return 0;
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

    ret = i2c_add_driver(&rcwl1655_driver);
    return ret;
}

static void __exit rcwl1655_exit_driver(void)
{
    i2c_del_driver(&rcwl1655_driver);
}

module_init(rcwl1655_init_driver);
module_exit(rcwl1655_exit_driver);
// module_i2c_driver(rcwl1655_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zgs");
MODULE_DESCRIPTION("RCWL1655 sensor driver");