// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SecEdge I2C TPM - AST2600 fTPM
 *
 * Copyright (C) 2024 SecEdge
 *
 * TGC status/locality/etc functions seen in the LPC implementation do not
 * seem to be present.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include "tpm.h"

#define I2C_DRIVER_NAME "tpm_i2c_secedge"

#define TPM_I2C_SHORT_TIMEOUT  750     /* ms */
#define TPM_I2C_LONG_TIMEOUT   2000    /* 2 sec */

#define SECEDGE_STS_OK 1

struct priv_data {
	size_t len;
	/* This is the amount we read on the first try. 25 was chosen to fit a
	 * fair number of read responses in the buffer so a 2nd retry can be
	 * avoided in small message cases. */
	u8 buffer[sizeof(struct tpm_header)];
};

static int i2c_secedge_send(struct tpm_chip *chip, u8 *buf, size_t len)
{
	struct priv_data *priv = dev_get_drvdata(&chip->dev);
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	s32 status;

	priv->len = 0;

	if (len <= 2)
		return -EIO;

	dev_info(&chip->dev,
		"%s(buf=%*ph len=%0zx) -> sts=%d\n", __func__,
		(int)min_t(size_t, 64, len), buf, len, status);

	{
	  u32 arg[3] = {0x0,0x0, 0x0};
	  memcpy(arg, buf, len < 10 ? len : 12);
	  printk(KERN_ERR "%s(%d): 0x%08x 0x%08x 0x%08x\n",
		 __func__, __LINE__,
		 arg[0], arg[1], arg[2]);
         }

	status = i2c_master_send(client, buf, len);

	dev_info(&chip->dev,
		"%s(buf=%*ph len=%0zx) -> sts=%d\n", __func__,
		(int)min_t(size_t, 64, len), buf, len, status);

	if (status < 0)
		return status;

	/* The upper layer does not support incomplete sends. */
	if (status != len)
		return -E2BIG;

	return 0;
}

# define HDR_LEN TPM_HEADER_SIZE /* 10 bytes in header */

static int i2c_secedge_recv(struct tpm_chip *chip, u8 *buf, size_t count)
{
	struct priv_data *priv = dev_get_drvdata(&chip->dev);
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	struct tpm_header *hdr = (struct tpm_header *)priv->buffer;
	u32 expected_len;
	int rc;

	if (priv->len == 0)
		return -EIO;

	/* Get the message size from the message header, if we didn't get the
	 * whole message in read_status then we need to re-read the
	 * message. */
	expected_len = be32_to_cpu(hdr->length);
	if (expected_len > count)
		return -ENOMEM;

	if (priv->len >= expected_len) {
		dev_info(&chip->dev,
			"%s early(buf=%*ph count=%0zx) -> ret=%d\n", __func__,
			(int)min_t(size_t, 64, expected_len), buf, count,
			expected_len);
		memcpy(buf, priv->buffer, expected_len);
		return expected_len;
	}

	memcpy(buf, priv->buffer, priv->len); /* take the buffered header etc */
	rc = i2c_master_recv(client, &buf[priv->len], expected_len - priv->len);
	dev_info(&chip->dev,
		"%s reread(buf=%*ph count=%0zx) -> ret=%d\n", __func__,
		(int)min_t(size_t, 64, expected_len), buf, count,
		expected_len);
	if (rc >= 0) {
	  rc += priv->len;
	}
	return rc;
}

static void i2c_secedge_cancel(struct tpm_chip *chip)
{
	dev_err(&chip->dev, "TPM operation cancellation was requested, but is not supported");
}

static u8 i2c_secedge_read_status(struct tpm_chip *chip)
{
	struct priv_data *priv = dev_get_drvdata(&chip->dev);
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	int rc;

	/* The TPM fails the I2C read until it is ready, so we do the entire
	 * transfer here and buffer it locally. This way the common code can
	 * properly handle the timeouts. */
	priv->len = 0;
	memset(priv->buffer, 0, sizeof(priv->buffer));

	/* 
	  attempt read and spin until receive response other than 0xff
	*/
	for (;;) {
	  u8 waste[2] = {0xff, 0xff};

	  rc = i2c_master_recv(client, &waste[0], 1);
	  if (rc <= 0) {
	    return 0;
	  }
	  if (waste[0] == 0xff) {
	    msleep(50); /* wait 50mS */
	    continue;
	  }
	  priv->buffer[0] = waste[0];
	  break;
	}

	/*
	 * Buffer the header only
	 * Account for remainder to be read in _recv(): might contain trailing 0xff
	 */
	rc = i2c_master_recv(client, &priv->buffer[1], sizeof(priv->buffer) - 1);

	if (rc <= 0)
		return 0;

	priv->len = rc + 1;
	dev_info(&chip->dev,
		"%s: sts=%d", __func__, rc);

# if 0
	{
	  u32 expected_len;
	  struct tpm_header *hdr = (struct tpm_header *)priv->buffer;

	  expected_len = be32_to_cpu(hdr->length);
	  if (expected_len == HDR_LEN) { /* picked it all up */
	    priv->len = HDR_LEN;
	  }
	}
# endif
	return SECEDGE_STS_OK;
}

static bool i2c_secedge_req_canceled(struct tpm_chip *chip, u8 status)
{
	return 0;
}

static const struct tpm_class_ops i2c_secedge_tpm_ops = {
	.flags = TPM_OPS_AUTO_STARTUP,
	.status = i2c_secedge_read_status,
	.recv = i2c_secedge_recv,
	.send = i2c_secedge_send,
	.cancel = i2c_secedge_cancel,
	.req_complete_mask = 0 /*SECEDGE_STS_OK*/,
	.req_complete_val = 0 /*SECEDGE_STS_OK*/,
	.req_canceled = i2c_secedge_req_canceled,
};

static int i2c_secedge_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct tpm_chip *chip;
	struct device *dev = &client->dev;
	struct priv_data *priv;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	chip = tpmm_chip_alloc(dev, &i2c_secedge_tpm_ops);
	if (IS_ERR(chip))
		return PTR_ERR(chip);

	priv = devm_kzalloc(dev, sizeof(struct priv_data), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Default timeouts */
	chip->flags |= TPM_CHIP_FLAG_TPM2;
# if 0 /* for V2 ? */
	chip->timeout_a = msecs_to_jiffies(TPM_I2C_SHORT_TIMEOUT);
	chip->timeout_b = msecs_to_jiffies(TPM_I2C_LONG_TIMEOUT);
	chip->timeout_c = msecs_to_jiffies(TPM_I2C_SHORT_TIMEOUT);
	chip->timeout_d = msecs_to_jiffies(TPM_I2C_SHORT_TIMEOUT);
# endif

	dev_set_drvdata(&chip->dev, priv);

	/* There is no known way to probe for this device, and all version
	 * information seems to be read via TPM commands. Thus we rely on the
	 * TPM startup process in the common code to detect the device. */

	return tpm_chip_register(chip);
}

static int i2c_secedge_remove(struct i2c_client *client)
{
	struct device *dev = &(client->dev);
	struct tpm_chip *chip = dev_get_drvdata(dev);
	tpm_chip_unregister(chip);
	return 0;
}

static const struct i2c_device_id i2c_secedge_id[] = {
	{I2C_DRIVER_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, i2c_secedge_id);

#ifdef CONFIG_OF
static const struct of_device_id i2c_secedge_of_match[] = {
	{.compatible = "secedge,ast2600"},
	{},
};
MODULE_DEVICE_TABLE(of, i2c_secedge_of_match);
#endif

static SIMPLE_DEV_PM_OPS(i2c_secedge_pm_ops, tpm_pm_suspend, tpm_pm_resume);

static struct i2c_driver i2c_secedge_driver = {
	.id_table = i2c_secedge_id,
	.probe = i2c_secedge_probe,
	.remove = i2c_secedge_remove,
	.driver = {
		.name = I2C_DRIVER_NAME,
		.pm = &i2c_secedge_pm_ops,
		.of_match_table = of_match_ptr(i2c_secedge_of_match),
	},
};

module_i2c_driver(i2c_secedge_driver);

MODULE_AUTHOR("SecEdge");
MODULE_DESCRIPTION("Secedge TPM I2C Driver");
MODULE_LICENSE("GPL");
