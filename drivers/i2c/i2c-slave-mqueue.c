// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2017 - 2018, Intel Corporation.

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>

#define MQ_MSGBUF_SIZE		CONFIG_I2C_SLAVE_MQUEUE_MESSAGE_SIZE
#define MQ_QUEUE_SIZE		CONFIG_I2C_SLAVE_MQUEUE_QUEUE_SIZE
#define MQ_QUEUE_NEXT(x)	(((x) + 1) & (MQ_QUEUE_SIZE - 1))

struct mq_msg {
	int	len;
	u8	*buf;
};

struct mq_queue {
	struct bin_attribute	bin;
	struct kernfs_node	*kn;

	spinlock_t		lock; /* spinlock for queue index handling */
	int			in;
	int			out;

	struct mq_msg		*curr;
	int			truncated; /* drop current if truncated */
	struct mq_msg		queue[MQ_QUEUE_SIZE];
	int			in_wr;
	int			out_wr;
	struct mq_msg		*curr_wr;
	int			truncated_wr; /* drop current if truncated */
	struct mq_msg		queue_wr[MQ_QUEUE_SIZE];
	bool is_read;
	u16 buffer_idx;
};

static int i2c_slave_mqueue_callback(struct i2c_client *client,
				     enum i2c_slave_event event, u8 *val)
{
	struct mq_queue *mq = i2c_get_clientdata(client);
	struct mq_msg *msg = mq->curr;
	struct mq_msg *msg_wr = mq->curr_wr;
	struct device *dev = &client->dev;
	int ret = 0;

	switch (event) {
	case I2C_SLAVE_WRITE_REQUESTED:
		mq->is_read = false;
		mq->truncated = 0;

		msg->len = 1;
		msg->buf[0] = client->addr << 1;
		break;

	case I2C_SLAVE_WRITE_RECEIVED:
		if (msg->len < MQ_MSGBUF_SIZE) {
			msg->buf[msg->len++] = *val;
		} else {
			dev_err(&client->dev, "message is truncated!\n");
			mq->truncated = 1;
			ret = -EINVAL;
		}
		break;
	
	case I2C_SLAVE_READ_PROCESSED:
		if (mq->out_wr < mq->in_wr) {
			if (mq->buffer_idx < msg_wr->len) {
				*val = msg_wr->buf[mq->buffer_idx++];
				ret = msg_wr->len;
			} else {
				*val = 0xFF;
			}
		}
		break;

	case I2C_SLAVE_READ_REQUESTED:
		dev_info(dev, "Got I2C_SLAVE_READ_REQUESTED event!\n");
		dev_info(dev, "out_wr: %d ----- in_wr %d \n", mq->out_wr, mq->in_wr);
		dev_info(dev, "buffer idx: %d ----- len %d \n", mq->buffer_idx, msg_wr->len);
		
		mq->is_read = true;
		if (mq->out_wr >= mq->in_wr) {
			dev_info(dev, "No Data to read\n");
		} else {
			if (mq->buffer_idx < msg_wr->len) {
				*val = msg_wr->buf[mq->buffer_idx++];
			} else {
				*val = 0xFF;
			}
		}
		break;

	case I2C_SLAVE_STOP:
		if (!mq->is_read) {
			dev_info(dev, "Got I2C_SLAVE_STOP READ event!\n");
			if (unlikely(mq->truncated || msg->len < 2))
				break;

			spin_lock(&mq->lock);
			mq->in = MQ_QUEUE_NEXT(mq->in);
			mq->curr = &mq->queue[mq->in];
			mq->curr->len = 0;

			/* Flush the oldest message */
			if (mq->out == mq->in)
				mq->out = MQ_QUEUE_NEXT(mq->out);
			spin_unlock(&mq->lock);

			kernfs_notify(mq->kn);
		} else {
			dev_info(dev, "Got I2C_SLAVE_STOP WRITE event!\n");
			spin_lock(&mq->lock);
			dev_info(dev, "buffer idx: %d ----- len %d \n", mq->buffer_idx, msg_wr->len);
			if (mq->buffer_idx == msg_wr->len) {
				if (mq->out_wr < mq->in_wr) {
					mq->out_wr = MQ_QUEUE_NEXT(mq->out_wr);
					mq->curr_wr = &mq->queue_wr[mq->out_wr];
				}
				mq->buffer_idx = 0;
			}

			dev_info(dev, "out_wr: %d ----- in_wr %d \n", mq->out_wr, mq->in_wr);
			
			spin_unlock(&mq->lock);
		}

		break;

	default:
		*val = 0xFF;
		break;
	}

	return ret;
}

static ssize_t i2c_slave_mqueue_bin_read(struct file *filp,
					 struct kobject *kobj,
					 struct bin_attribute *attr,
					 char *buf, loff_t pos, size_t count)
{
	struct mq_queue *mq;
	struct mq_msg *msg;
	unsigned long flags;
	bool more = false;
	ssize_t ret = 0;

	mq = dev_get_drvdata(container_of(kobj, struct device, kobj));

	spin_lock_irqsave(&mq->lock, flags);
	if (mq->out != mq->in) {
		msg = &mq->queue[mq->out];

		if (msg->len <= count) {
			ret = msg->len;
			memcpy(buf, msg->buf, ret);
		} else {
			ret = -EOVERFLOW; /* Drop this HUGE one. */
		}

		mq->out = MQ_QUEUE_NEXT(mq->out);
		if (mq->out != mq->in)
			more = true;
	}
	spin_unlock_irqrestore(&mq->lock, flags);

	if (more)
		kernfs_notify(mq->kn);

	return ret;
}

static ssize_t i2c_slave_mqueue_bin_write(struct file *filp,
					 struct kobject *kobj,
					 struct bin_attribute *attr,
					 char *buf, loff_t pos, size_t count)
{
	struct mq_queue *mq;
	struct mq_msg *msg;
	unsigned long flags;

	ssize_t ret = 0;

	mq = dev_get_drvdata(container_of(kobj, struct device, kobj));

	spin_lock_irqsave(&mq->lock, flags);

	msg = &mq->queue_wr[mq->in_wr];

	if (count <= MQ_MSGBUF_SIZE) {
		msg->len = count;
		memcpy(msg->buf, buf, msg->len);
		ret = msg->len;
	} else {
		ret = -EOVERFLOW; /* Drop this HUGE one. */
	}

	mq->in_wr = MQ_QUEUE_NEXT(mq->in_wr);

	spin_unlock_irqrestore(&mq->lock, flags);

	return ret;
}

static int i2c_slave_mqueue_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct mq_queue *mq;
	int ret, i;
	void *buf;
	void *buf_wr;

	mq = devm_kzalloc(dev, sizeof(*mq), GFP_KERNEL);
	if (!mq)
		return -ENOMEM;

	BUILD_BUG_ON(!is_power_of_2(MQ_QUEUE_SIZE));

	buf = devm_kmalloc_array(dev, MQ_QUEUE_SIZE, MQ_MSGBUF_SIZE,
				 GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	for (i = 0; i < MQ_QUEUE_SIZE; i++)
		mq->queue[i].buf = buf + i * MQ_MSGBUF_SIZE;
	
	buf_wr = devm_kmalloc_array(dev, MQ_QUEUE_SIZE, MQ_MSGBUF_SIZE,
				 GFP_KERNEL);
	if (!buf_wr)
		return -ENOMEM;

	for (i = 0; i < MQ_QUEUE_SIZE; i++)
		mq->queue_wr[i].buf = buf_wr + i * MQ_MSGBUF_SIZE;

	i2c_set_clientdata(client, mq);

	spin_lock_init(&mq->lock);
	mq->curr = &mq->queue[0];
	mq->curr_wr = &mq->queue_wr[0];

	sysfs_bin_attr_init(&mq->bin);
	mq->bin.attr.name = "slave-mqueue";
	mq->bin.attr.mode = S_IRUSR | S_IWUSR;;
	mq->bin.read = i2c_slave_mqueue_bin_read;
	mq->bin.write = i2c_slave_mqueue_bin_write;
	mq->bin.size = MQ_MSGBUF_SIZE * MQ_QUEUE_SIZE + MQ_MSGBUF_SIZE * MQ_QUEUE_SIZE;

	ret = sysfs_create_bin_file(&dev->kobj, &mq->bin);
	if (ret)
		return ret;

	mq->kn = kernfs_find_and_get(dev->kobj.sd, mq->bin.attr.name);
	if (!mq->kn) {
		sysfs_remove_bin_file(&dev->kobj, &mq->bin);
		return -EFAULT;
	}

	ret = i2c_slave_register(client, i2c_slave_mqueue_callback);
	if (ret) {
		kernfs_put(mq->kn);
		sysfs_remove_bin_file(&dev->kobj, &mq->bin);
		return ret;
	}

	return 0;
}

static int i2c_slave_mqueue_remove(struct i2c_client *client)
{
	struct mq_queue *mq = i2c_get_clientdata(client);

	i2c_slave_unregister(client);

	kernfs_put(mq->kn);
	sysfs_remove_bin_file(&client->dev.kobj, &mq->bin);

	return 0;
}

static const struct i2c_device_id i2c_slave_mqueue_id[] = {
	{ "slave-mqueue", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, i2c_slave_mqueue_id);

static struct i2c_driver i2c_slave_mqueue_driver = {
	.driver = {
		.name	= "i2c-slave-mqueue",
	},
	.probe		= i2c_slave_mqueue_probe,
	.remove		= i2c_slave_mqueue_remove,
	.id_table	= i2c_slave_mqueue_id,
};
module_i2c_driver(i2c_slave_mqueue_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Haiyue Wang <haiyue.wang@linux.intel.com>");
MODULE_DESCRIPTION("I2C slave mode for receiving and queuing messages");
