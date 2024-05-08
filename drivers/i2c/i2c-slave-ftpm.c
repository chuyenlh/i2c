// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2017 - 2018, Intel Corporation.

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>

#define MQ_MSGBUF_SIZE		CONFIG_I2C_SLAVE_FTPM_MESSAGE_SIZE

struct ftpm_msg {
	int	len;
	u8	*buf;
};

struct ftpm_data {
	struct bin_attribute	bin;
	struct kernfs_node	*kn;
	spinlock_t		lock; /* spinlock for queue index handling */
	int			truncated; /* drop current if truncated */
	struct ftpm_msg		buffer_read;
	struct ftpm_msg		buffer_write;
	bool is_read;
	u16 buffer_write_idx;
	u16 buffer_read_idx;
};

static int i2c_slave_ftpm_callback(struct i2c_client *client,
				     enum i2c_slave_event event, u8 *val)
{
	struct ftpm_data *ftpm = i2c_get_clientdata(client);
	struct ftpm_msg *msg_read = &ftpm->buffer_read;
	struct ftpm_msg *msg_wr = &ftpm->buffer_write;
	int ret = 0;

	switch (event) {
	case I2C_SLAVE_WRITE_REQUESTED:
		ftpm->is_read = false;
		ftpm->truncated = 0;
		ftpm->buffer_read_idx = 0;
		msg_read->len = 0;
		break;

	case I2C_SLAVE_WRITE_RECEIVED:
		if (msg_read->len < MQ_MSGBUF_SIZE) {
			msg_read->buf[msg_read->len++] = *val;
		} else {
			dev_err(&client->dev, "message is truncated!\n");
			ftpm->truncated = 1;
			ret = -EINVAL;
		}
		break;
	
	case I2C_SLAVE_READ_PROCESSED:
		if (ftpm->buffer_write_idx < msg_wr->len) {
			*val = msg_wr->buf[ftpm->buffer_write_idx++];
		} else {
			*val = 0xFF;
		}

		break;

	case I2C_SLAVE_READ_REQUESTED:
		ftpm->is_read = true;
		if (ftpm->buffer_write_idx < msg_wr->len) {
			*val = msg_wr->buf[ftpm->buffer_write_idx++];
		} else {
			*val = 0xFF;
		}
		break;

	case I2C_SLAVE_STOP:
		if (!ftpm->is_read) {
			if (unlikely(ftpm->truncated || msg_read->len < 2))
				break;
			kernfs_notify(ftpm->kn);
		} else {
			spin_lock(&ftpm->lock);
			if (ftpm->buffer_write_idx == msg_wr->len) {
				msg_wr->len = 0;
				ftpm->buffer_write_idx = 0;
			}
			spin_unlock(&ftpm->lock);
		}
		break;

	default:
		*val = 0xFF;
		break;
	}

	return ret;
}

static ssize_t i2c_slave_ftpm_bin_read(struct file *filp,
					 struct kobject *kobj,
					 struct bin_attribute *attr,
					 char *buf, loff_t pos, size_t count)
{
	struct ftpm_data *mq;
	struct ftpm_msg *msg;
	unsigned long flags;
	ssize_t ret = 0;

	mq = dev_get_drvdata(container_of(kobj, struct device, kobj));

	spin_lock_irqsave(&mq->lock, flags);
	msg = &mq->buffer_read;

	if (msg->len - mq->buffer_read_idx <= count) {
		ret = msg->len - mq->buffer_read_idx;
	} else {
		ret = count; /* Drop this HUGE one. */
	}
	memcpy(buf, msg->buf + mq->buffer_read_idx, ret);
	mq->buffer_read_idx += ret;
	spin_unlock_irqrestore(&mq->lock, flags);
	return ret;
}

static ssize_t i2c_slave_ftpm_bin_write(struct file *filp,
					 struct kobject *kobj,
					 struct bin_attribute *attr,
					 char *buf, loff_t pos, size_t count)
{
	struct ftpm_data *mq;
	struct ftpm_msg *msg;
	unsigned long flags;

	ssize_t ret = 0;

	mq = dev_get_drvdata(container_of(kobj, struct device, kobj));

	spin_lock_irqsave(&mq->lock, flags);

	mq->buffer_write_idx = 0;
	msg = &mq->buffer_write;
	if (count <= MQ_MSGBUF_SIZE) {
		msg->len = count;
		memcpy(msg->buf, buf, msg->len);
		ret = msg->len;
	} else {
		ret = -EOVERFLOW; /* Drop this HUGE one. */
	}

	spin_unlock_irqrestore(&mq->lock, flags);

	return ret;
}

static int i2c_slave_ftpm_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ftpm_data *mq;
	int ret;
	void *buf;
	void *buf_wr;

	mq = devm_kzalloc(dev, sizeof(*mq), GFP_KERNEL);
	if (!mq)
		return -ENOMEM;


	buf = devm_kmalloc(dev, MQ_MSGBUF_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	mq->buffer_read.buf = buf;
	
	buf_wr = devm_kmalloc(dev, MQ_MSGBUF_SIZE, GFP_KERNEL);
	if (!buf_wr)
		return -ENOMEM;

	mq->buffer_write.buf = buf_wr;

	i2c_set_clientdata(client, mq);

	spin_lock_init(&mq->lock);
	sysfs_bin_attr_init(&mq->bin);
	mq->bin.attr.name = "slave-ftpm";
	mq->bin.attr.mode = S_IRUSR | S_IWUSR;;
	mq->bin.read = i2c_slave_ftpm_bin_read;
	mq->bin.write = i2c_slave_ftpm_bin_write;
	mq->bin.size = MQ_MSGBUF_SIZE + MQ_MSGBUF_SIZE;

	ret = sysfs_create_bin_file(&dev->kobj, &mq->bin);
	if (ret)
		return ret;

	mq->kn = kernfs_find_and_get(dev->kobj.sd, mq->bin.attr.name);
	if (!mq->kn) {
		sysfs_remove_bin_file(&dev->kobj, &mq->bin);
		return -EFAULT;
	}

	ret = i2c_slave_register(client, i2c_slave_ftpm_callback);
	if (ret) {
		kernfs_put(mq->kn);
		sysfs_remove_bin_file(&dev->kobj, &mq->bin);
		return ret;
	}

	return 0;
}

static int i2c_slave_ftpm_remove(struct i2c_client *client)
{
	struct ftpm_data *mq = i2c_get_clientdata(client);

	i2c_slave_unregister(client);

	kernfs_put(mq->kn);
	sysfs_remove_bin_file(&client->dev.kobj, &mq->bin);

	return 0;
}

static const struct i2c_device_id i2c_slave_ftpm_id[] = {
	{ "slave-ftpm", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, i2c_slave_ftpm_id);

static struct i2c_driver i2c_slave_ftpm_driver = {
	.driver = {
		.name	= "i2c-slave-ftpm",
	},
	.probe		= i2c_slave_ftpm_probe,
	.remove		= i2c_slave_ftpm_remove,
	.id_table	= i2c_slave_ftpm_id,
};
module_i2c_driver(i2c_slave_ftpm_driver);

MODULE_AUTHOR("SecEdge");
MODULE_DESCRIPTION("I2C slave mode FTPM");
MODULE_LICENSE("GPL v2");
