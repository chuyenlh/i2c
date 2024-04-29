// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASPEED AST2600 new register set I2C controller driver
 *
 * Copyright (C) ASPEED Technology Inc.
 */
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/i2c-smbus.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include "i2c-ast2600-global.h"

/* 0x00 : I2CC Master/Slave Function Control Register  */
#define AST2600_I2CC_FUN_CTRL		0x00
#define AST2600_I2CC_SLAVE_ADDR_RX_EN		BIT(20)
#define AST2600_I2CC_MASTER_RETRY_MASK		GENMASK(19, 18)
#define AST2600_I2CC_MASTER_RETRY(x)		(((x) & GENMASK(1, 0)) << 18)
#define AST2600_I2CC_BUS_AUTO_RELEASE		BIT(17)
#define AST2600_I2CC_M_SDA_LOCK_EN			BIT(16)
#define AST2600_I2CC_MULTI_MASTER_DIS		BIT(15)
#define AST2600_I2CC_M_SCL_DRIVE_EN			BIT(14)
#define AST2600_I2CC_MSB_STS				BIT(9)
#define AST2600_I2CC_SDA_DRIVE_1T_EN		BIT(8)
#define AST2600_I2CC_M_SDA_DRIVE_1T_EN		BIT(7)
#define AST2600_I2CC_M_HIGH_SPEED_EN		BIT(6)
/* reserver 5 : 2 */
#define AST2600_I2CC_SLAVE_EN			BIT(1)
#define AST2600_I2CC_MASTER_EN			BIT(0)

/* 0x04 : I2CC Master/Slave Clock and AC Timing Control Register #1 */
#define AST2600_I2CC_AC_TIMING		0x04
#define AST2600_I2CC_TTIMEOUT(x)			(((x) & GENMASK(4, 0)) << 24)
#define AST2600_I2CC_TCKHIGHMIN(x)			(((x) & GENMASK(3, 0)) << 20)
#define AST2600_I2CC_TCKHIGH(x)			(((x) & GENMASK(3, 0)) << 16)
#define AST2600_I2CC_TCKLOW(x)			(((x) & GENMASK(3, 0)) << 12)
#define AST2600_I2CC_THDDAT(x)			(((x) & GENMASK(1, 0)) << 10)
#define AST2600_I2CC_TOUTBASECLK(x)			(((x) & GENMASK(1, 0)) << 8)
#define AST2600_I2CC_TBASECLK(x)			((x) & GENMASK(3, 0))

/* 0x08 : I2CC Master/Slave Transmit/Receive Byte Buffer Register */
#define AST2600_I2CC_STS_AND_BUFF		0x08
#define AST2600_I2CC_TX_DIR_MASK			GENMASK(31, 29)
#define AST2600_I2CC_SDA_OE				BIT(28)
#define AST2600_I2CC_SDA_O				BIT(27)
#define AST2600_I2CC_SCL_OE				BIT(26)
#define AST2600_I2CC_SCL_O				BIT(25)

#define AST2600_I2CC_SCL_LINE_STS			BIT(18)
#define AST2600_I2CC_SDA_LINE_STS			BIT(17)
#define AST2600_I2CC_BUS_BUSY_STS			BIT(16)

#define AST2600_I2CC_GET_RX_BUFF(x)			(((x) >> 8) & GENMASK(7, 0))

/* 0x0C : I2CC Master/Slave Pool Buffer Control Register  */
#define AST2600_I2CC_BUFF_CTRL		0x0C
#define AST2600_I2CC_GET_RX_BUF_LEN(x)      (((x) & GENMASK(29, 24)) >> 24)
#define AST2600_I2CC_SET_RX_BUF_LEN(x)		(((((x) - 1) & GENMASK(4, 0)) << 16) | BIT(0))
#define AST2600_I2CC_SET_TX_BUF_LEN(x)		(((((x) - 1) & GENMASK(4, 0)) << 8) | BIT(0))
#define AST2600_I2CC_GET_TX_BUF_LEN(x)      ((((x) & GENMASK(12, 8)) >> 8) + 1)

/* 0x10 : I2CM Master Interrupt Control Register */
#define AST2600_I2CM_IER			0x10
/* 0x14 : I2CM Master Interrupt Status Register   : WC */
#define AST2600_I2CM_ISR			0x14

#define AST2600_I2CM_PKT_TIMEOUT			BIT(18)
#define AST2600_I2CM_PKT_ERROR			BIT(17)
#define AST2600_I2CM_PKT_DONE			BIT(16)

#define AST2600_I2CM_BUS_RECOVER_FAIL		BIT(15)
#define AST2600_I2CM_SDA_DL_TO			BIT(14)
#define AST2600_I2CM_BUS_RECOVER			BIT(13)
#define AST2600_I2CM_SMBUS_ALT			BIT(12)

#define AST2600_I2CM_SCL_LOW_TO			BIT(6)
#define AST2600_I2CM_ABNORMAL			BIT(5)
#define AST2600_I2CM_NORMAL_STOP			BIT(4)
#define AST2600_I2CM_ARBIT_LOSS			BIT(3)
#define AST2600_I2CM_RX_DONE			BIT(2)
#define AST2600_I2CM_TX_NAK				BIT(1)
#define AST2600_I2CM_TX_ACK				BIT(0)

/* 0x18 : I2CM Master Command/Status Register   */
#define AST2600_I2CM_CMD_STS		0x18
#define AST2600_I2CM_PKT_ADDR(x)			(((x) & GENMASK(6, 0)) << 24)
#define AST2600_I2CM_PKT_EN				BIT(16)
#define AST2600_I2CM_SDA_OE_OUT_DIR			BIT(15)
#define AST2600_I2CM_SDA_O_OUT_DIR			BIT(14)
#define AST2600_I2CM_SCL_OE_OUT_DIR			BIT(13)
#define AST2600_I2CM_SCL_O_OUT_DIR			BIT(12)
#define AST2600_I2CM_RECOVER_CMD_EN			BIT(11)

#define AST2600_I2CM_RX_DMA_EN			BIT(9)
#define AST2600_I2CM_TX_DMA_EN			BIT(8)
/* Command Bit */
#define AST2600_I2CM_RX_BUFF_EN			BIT(7)
#define AST2600_I2CM_TX_BUFF_EN			BIT(6)
#define AST2600_I2CM_STOP_CMD			BIT(5)
#define AST2600_I2CM_RX_CMD_LAST			BIT(4)
#define AST2600_I2CM_RX_CMD				BIT(3)

#define AST2600_I2CM_TX_CMD				BIT(1)
#define AST2600_I2CM_START_CMD			BIT(0)

/* 0x1C : I2CM Master DMA Transfer Length Register	 */
#define AST2600_I2CM_DMA_LEN		0x1C
/* Tx Rx support length 1 ~ 4096 */
#define AST2600_I2CM_SET_RX_DMA_LEN(x)	((((x) & GENMASK(11, 0)) << 16) | BIT(31))
#define AST2600_I2CM_SET_TX_DMA_LEN(x)	(((x) & GENMASK(11, 0)) | BIT(15))

/* 0x20 : I2CS Slave Interrupt Control Register   */
#define AST2600_I2CS_IER			0x20
/* 0x24 : I2CS Slave Interrupt Status Register	 */
#define AST2600_I2CS_ISR			0x24

#define AST2600_I2CS_ADDR_INDICATE_MASK	GENMASK(31, 30)
#define AST2600_I2CS_SLAVE_PENDING			BIT(29)

#define AST2600_I2CS_WAIT_TX_DMA			BIT(25)
#define AST2600_I2CS_WAIT_RX_DMA			BIT(24)

#define AST2600_I2CS_ADDR3_NAK			BIT(22)
#define AST2600_I2CS_ADDR2_NAK			BIT(21)
#define AST2600_I2CS_ADDR1_NAK			BIT(20)

#define AST2600_I2CS_ADDR_MASK			GENMASK(19, 18)
#define AST2600_I2CS_PKT_ERROR			BIT(17)
#define AST2600_I2CS_PKT_DONE			BIT(16)
#define AST2600_I2CS_INACTIVE_TO			BIT(15)

#define AST2600_I2CS_SLAVE_MATCH			BIT(7)
#define AST2600_I2CS_ABNOR_STOP			BIT(5)
#define AST2600_I2CS_STOP				BIT(4)
#define AST2600_I2CS_RX_DONE_NAK			BIT(3)
#define AST2600_I2CS_RX_DONE			BIT(2)
#define AST2600_I2CS_TX_NAK				BIT(1)
#define AST2600_I2CS_TX_ACK				BIT(0)

/* 0x28 : I2CS Slave CMD/Status Register   */
#define AST2600_I2CS_CMD_STS		0x28
#define AST2600_I2CS_ACTIVE_ALL			GENMASK(18, 17)
#define AST2600_I2CS_PKT_MODE_EN			BIT(16)
#define AST2600_I2CS_AUTO_NAK_NOADDR		BIT(15)
#define AST2600_I2CS_AUTO_NAK_EN			BIT(14)

#define AST2600_I2CS_ALT_EN				BIT(10)
#define AST2600_I2CS_RX_DMA_EN			BIT(9)
#define AST2600_I2CS_TX_DMA_EN			BIT(8)
#define AST2600_I2CS_RX_BUFF_EN			BIT(7)
#define AST2600_I2CS_TX_BUFF_EN			BIT(6)
#define AST2600_I2CS_RX_CMD_LAST			BIT(4)

#define AST2600_I2CS_TX_CMD				BIT(2)

#define AST2600_I2CS_DMA_LEN		0x2C
#define AST2600_I2CS_SET_RX_DMA_LEN(x)	(((((x) - 1) & GENMASK(11, 0)) << 16) | BIT(31))
#define AST2600_I2CS_RX_DMA_LEN_MASK	(GENMASK(11, 0) << 16)

#define AST2600_I2CS_SET_TX_DMA_LEN(x)	((((x) - 1) & GENMASK(11, 0)) | BIT(15))
#define AST2600_I2CS_TX_DMA_LEN_MASK	GENMASK(11, 0)

/* I2CM Master DMA Tx Buffer Register   */
#define AST2600_I2CM_TX_DMA			0x30
/* I2CM Master DMA Rx Buffer Register	*/
#define AST2600_I2CM_RX_DMA			0x34
/* I2CS Slave DMA Tx Buffer Register   */
#define AST2600_I2CS_TX_DMA			0x38
/* I2CS Slave DMA Rx Buffer Register   */
#define AST2600_I2CS_RX_DMA			0x3C

#define AST2600_I2CS_ADDR_CTRL		0x40

#define	AST2600_I2CS_ADDR3_MASK		GENMASK(22, 16)
#define	AST2600_I2CS_ADDR2_MASK		GENMASK(14, 8)
#define	AST2600_I2CS_ADDR1_MASK		GENMASK(6, 0)

#define AST2600_I2CM_DMA_LEN_STS		0x48
#define AST2600_I2CS_DMA_LEN_STS		0x4C

#define AST2600_I2C_GET_TX_DMA_LEN(x)		((x) & GENMASK(12, 0))
#define AST2600_I2C_GET_RX_DMA_LEN(x)        (((x) & GENMASK(28, 16)) >> 16)

/* 0x40 : Slave Device Address Register */
#define AST2600_I2CS_ADDR3_ENABLE			BIT(23)
#define AST2600_I2CS_ADDR3(x)			((x) << 16)
#define AST2600_I2CS_ADDR2_ENABLE			BIT(15)
#define AST2600_I2CS_ADDR2(x)			((x) << 8)
#define AST2600_I2CS_ADDR1_ENABLE			BIT(7)
#define AST2600_I2CS_ADDR1(x)			(x)

#define I2C_SLAVE_MSG_BUF_SIZE		256

#define AST2600_I2C_DMA_SIZE		4096

#define MASTER_TRIGGER_LAST_STOP	(AST2600_I2CM_RX_CMD_LAST | AST2600_I2CM_STOP_CMD)
#define SLAVE_TRIGGER_CMD	(AST2600_I2CS_ACTIVE_ALL | AST2600_I2CS_PKT_MODE_EN)

#define AST_I2C_TIMEOUT_CLK		0x2

enum xfer_mode {
	BYTE_MODE,
	BUFF_MODE,
	DMA_MODE,
};

struct ast2600_i2c_bus {
	struct i2c_adapter	adap;
	struct device		*dev;
	void __iomem		*reg_base;
	struct regmap		*global_reg;
	int					irq;
	/* 0: dma, 1: pool, 2:byte */
	enum xfer_mode		mode;
	/* 0: old mode, 1: new mode */
	int					clk_div_mode;
	struct clk			*clk;
	u32					apb_clk;
	u32					bus_frequency;
	int					slave_operate;
	u32					timeout;
	/* smbus alert */
	int					alert_enable;
	struct i2c_smbus_alert_setup alert_data;
	struct i2c_client *ara;
	/* Multi-master */
	bool				multi_master;
	/* master structure */
	int					cmd_err;
	struct completion	cmd_complete;
	/* smbus send */
	bool				smbus_protocol;
	struct i2c_msg		*msgs;	/* cur xfer msgs */
	size_t				buf_index;	/* buffer mode idx */
	/* cur xfer msgs index*/
	int					msgs_index;
	int					msgs_count;	/* total msgs */
	u8					*master_safe_buf;
	dma_addr_t			master_dma_addr;
	/*total xfer count */
	int					master_xfer_cnt;
	int					master_xfer_tx_cnt;
	int					master_xfer_rx_cnt;
	/* Buffer mode */
	void __iomem		*buf_base;
	size_t				buf_size;
	/* Slave structure */
	int					slave_xfer_len;
	int					slave_xfer_cnt;
#ifdef CONFIG_I2C_SLAVE
	unsigned char		*slave_dma_buf;
	dma_addr_t			slave_dma_addr;
	struct i2c_client	*slave;
#endif
};

static u32 ast2600_select_i2c_clock(struct ast2600_i2c_bus *i2c_bus)
{
	unsigned long base_clk[16];
	int baseclk_idx;
	u32 clk_div_reg;
	u32 scl_low;
	u32 scl_high;
	int divisor = 0;
	u32 data;
	int i;

	regmap_read(i2c_bus->global_reg, AST2600_I2CG_CLK_DIV_CTRL, &clk_div_reg);
	for (i = 0; i < 16; i++) {
		if (i == 0)
			base_clk[i] = i2c_bus->apb_clk;
		else if ((i > 0) || (i < 5))
			base_clk[i] = (i2c_bus->apb_clk * 2) /
				(((clk_div_reg >> ((i - 1) * 8)) & GENMASK(7, 0)) + 2);
		else
			base_clk[i] = base_clk[4] / (1 << (i - 5));

		if ((base_clk[i] / i2c_bus->bus_frequency) <= 32) {
			baseclk_idx = i;
			divisor = DIV_ROUND_UP(base_clk[i], i2c_bus->bus_frequency);
			break;
		}
	}
	baseclk_idx = min(baseclk_idx, 15);
	divisor = min(divisor, 32);
	scl_low = min(divisor * 9 / 16 - 1, 15);
	scl_high = (divisor - scl_low - 2) & GENMASK(3, 0);
	data = (scl_high - 1) << 20 | scl_high << 16 | scl_low << 12 | baseclk_idx;
	if (i2c_bus->timeout) {
		data |= AST2600_I2CC_TOUTBASECLK(AST_I2C_TIMEOUT_CLK);
		data |= AST2600_I2CC_TTIMEOUT(i2c_bus->timeout);
	}

	return data;
}

static u8 ast2600_i2c_recover_bus(struct ast2600_i2c_bus *i2c_bus)
{
	u32 state = readl(i2c_bus->reg_base + AST2600_I2CC_STS_AND_BUFF);
	int ret = 0;
	u32 ctrl;
	int r;

	dev_dbg(i2c_bus->dev, "%d-bus recovery bus [%x]\n", i2c_bus->adap.nr, state);

	ctrl = readl(i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);

	/* Disable master/slave mode */
	writel(ctrl & ~(AST2600_I2CC_MASTER_EN | AST2600_I2CC_SLAVE_EN),
	       i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);

	/* Enable master mode only */
	writel(readl(i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL) | AST2600_I2CC_MASTER_EN,
	       i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);

	reinit_completion(&i2c_bus->cmd_complete);
	i2c_bus->cmd_err = 0;

	/* Check 0x14's SDA and SCL status */
	state = readl(i2c_bus->reg_base + AST2600_I2CC_STS_AND_BUFF);
	if (!(state & AST2600_I2CC_SDA_LINE_STS) && (state & AST2600_I2CC_SCL_LINE_STS)) {
		writel(AST2600_I2CM_RECOVER_CMD_EN, i2c_bus->reg_base + AST2600_I2CM_CMD_STS);
		r = wait_for_completion_timeout(&i2c_bus->cmd_complete, i2c_bus->adap.timeout);
		if (r == 0) {
			dev_dbg(i2c_bus->dev, "recovery timed out\n");
			ret = -ETIMEDOUT;
		} else {
			if (i2c_bus->cmd_err) {
				dev_dbg(i2c_bus->dev, "recovery error\n");
				ret = -EPROTO;
			}
		}
	}

	/* Recovery done */
	state = readl(i2c_bus->reg_base + AST2600_I2CC_STS_AND_BUFF);
	if (state & AST2600_I2CC_BUS_BUSY_STS) {
		dev_dbg(i2c_bus->dev, "Can't recover bus [%x]\n", state);
		ret = -EPROTO;
	}

	/* restore original master/slave setting */
	writel(ctrl, i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);
	return ret;
}

#ifdef CONFIG_I2C_SLAVE
static void ast2600_i2c_slave_packet_dma_irq(struct ast2600_i2c_bus *i2c_bus, u32 sts)
{
	int slave_rx_len;
	u32 cmd = 0;
	u8 value;
	int i;

	sts &= ~(AST2600_I2CS_SLAVE_PENDING);
	/* Handle i2c slave timeout condition */
	if (AST2600_I2CS_INACTIVE_TO & sts) {
		cmd = SLAVE_TRIGGER_CMD;
		cmd |= AST2600_I2CS_RX_DMA_EN;
		writel(AST2600_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_MSG_BUF_SIZE),
		       i2c_bus->reg_base + AST2600_I2CS_DMA_LEN);
		writel(cmd, i2c_bus->reg_base + AST2600_I2CS_CMD_STS);
		writel(AST2600_I2CS_PKT_DONE, i2c_bus->reg_base + AST2600_I2CS_ISR);
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_STOP, &value);
		return;
	}

	sts &= ~(AST2600_I2CS_PKT_DONE | AST2600_I2CS_PKT_ERROR);

	switch (sts) {
	case AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_RX_DONE | AST2600_I2CS_WAIT_RX_DMA:
	case AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_WAIT_RX_DMA:
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_REQUESTED, &value);
		slave_rx_len = AST2600_I2C_GET_RX_DMA_LEN(readl(i2c_bus->reg_base +
						      AST2600_I2CS_DMA_LEN_STS));
		for (i = 0; i < slave_rx_len; i++) {
			i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_RECEIVED,
					&i2c_bus->slave_dma_buf[i]);
		}
		writel(AST2600_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_MSG_BUF_SIZE),
		       i2c_bus->reg_base + AST2600_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST2600_I2CS_RX_DMA_EN;
		break;
	case AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_STOP:
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_STOP, &value);
		writel(AST2600_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_MSG_BUF_SIZE),
		       i2c_bus->reg_base + AST2600_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST2600_I2CS_RX_DMA_EN;
		break;
	case AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_RX_DONE_NAK |
			AST2600_I2CS_RX_DONE | AST2600_I2CS_STOP:
	case AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_WAIT_RX_DMA |
			AST2600_I2CS_RX_DONE | AST2600_I2CS_STOP:
	case AST2600_I2CS_RX_DONE_NAK | AST2600_I2CS_RX_DONE | AST2600_I2CS_STOP:
	case AST2600_I2CS_RX_DONE | AST2600_I2CS_WAIT_RX_DMA | AST2600_I2CS_STOP:
	case AST2600_I2CS_RX_DONE | AST2600_I2CS_STOP:
	case AST2600_I2CS_RX_DONE | AST2600_I2CS_WAIT_RX_DMA:
	case AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_RX_DONE | AST2600_I2CS_STOP:
		if (sts & AST2600_I2CS_SLAVE_MATCH)
			i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_REQUESTED, &value);

		slave_rx_len = AST2600_I2C_GET_RX_DMA_LEN(readl(i2c_bus->reg_base +
						      AST2600_I2CS_DMA_LEN_STS));
		for (i = 0; i < slave_rx_len; i++) {
			i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_RECEIVED,
					&i2c_bus->slave_dma_buf[i]);
		}
		writel(AST2600_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_MSG_BUF_SIZE),
		       i2c_bus->reg_base + AST2600_I2CS_DMA_LEN);
		if (sts & AST2600_I2CS_STOP)
			i2c_slave_event(i2c_bus->slave, I2C_SLAVE_STOP, &value);
		cmd = SLAVE_TRIGGER_CMD | AST2600_I2CS_RX_DMA_EN;
		break;

	/* it is Mw data Mr coming -> it need send tx */
	case AST2600_I2CS_RX_DONE | AST2600_I2CS_WAIT_TX_DMA:
	case AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_RX_DONE | AST2600_I2CS_WAIT_TX_DMA:
		/* it should be repeat start read */
		if (sts & AST2600_I2CS_SLAVE_MATCH)
			i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_REQUESTED, &value);

		slave_rx_len = AST2600_I2C_GET_RX_DMA_LEN(readl(i2c_bus->reg_base +
						      AST2600_I2CS_DMA_LEN_STS));
		for (i = 0; i < slave_rx_len; i++) {
			i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_RECEIVED,
					&i2c_bus->slave_dma_buf[i]);
		}
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_READ_REQUESTED,
				&i2c_bus->slave_dma_buf[0]);
		writel(0, i2c_bus->reg_base + AST2600_I2CS_DMA_LEN_STS);
		writel(AST2600_I2CS_SET_TX_DMA_LEN(1),
		       i2c_bus->reg_base + AST2600_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST2600_I2CS_TX_DMA_EN;
		break;
	case AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_WAIT_TX_DMA:
		/* First Start read */
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_READ_REQUESTED,
				&i2c_bus->slave_dma_buf[0]);
		writel(AST2600_I2CS_SET_TX_DMA_LEN(1),
		       i2c_bus->reg_base + AST2600_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST2600_I2CS_TX_DMA_EN;
		break;
	case AST2600_I2CS_WAIT_TX_DMA:
		/* it should be next start read */
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_READ_PROCESSED,
				&i2c_bus->slave_dma_buf[0]);
		writel(0, i2c_bus->reg_base + AST2600_I2CS_DMA_LEN_STS);
		writel(AST2600_I2CS_SET_TX_DMA_LEN(1),
		       i2c_bus->reg_base + AST2600_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST2600_I2CS_TX_DMA_EN;
		break;
	case AST2600_I2CS_TX_NAK | AST2600_I2CS_STOP:
		/* it just tx complete */
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_STOP, &value);
		writel(0, i2c_bus->reg_base + AST2600_I2CS_DMA_LEN_STS);
		writel(AST2600_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_MSG_BUF_SIZE),
		       i2c_bus->reg_base + AST2600_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST2600_I2CS_RX_DMA_EN;
		break;
	case AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_RX_DONE:
		cmd = 0;
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_REQUESTED, &value);
		break;
	case AST2600_I2CS_STOP:
		cmd = 0;
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_STOP, &value);
		break;
	default:
		dev_dbg(i2c_bus->dev, "unhandled slave isr case %x, sts %x\n", sts,
			readl(i2c_bus->reg_base + AST2600_I2CC_STS_AND_BUFF));
		break;
	}

	if (cmd)
		writel(cmd, i2c_bus->reg_base + AST2600_I2CS_CMD_STS);
	writel(AST2600_I2CS_PKT_DONE, i2c_bus->reg_base + AST2600_I2CS_ISR);
	readl(i2c_bus->reg_base + AST2600_I2CS_ISR);
	dev_dbg(i2c_bus->dev, "cmd %x\n", cmd);
}

static void ast2600_i2c_slave_packet_buff_irq(struct ast2600_i2c_bus *i2c_bus, u32 sts)
{
	int slave_rx_len = 0;
	u32 cmd = 0;
	u8 value;
	int i;

	/* due to master slave is common buffer, so need force the master stop not issue */
	if (readl(i2c_bus->reg_base + AST2600_I2CM_CMD_STS) & GENMASK(15, 0)) {
		writel(0, i2c_bus->reg_base + AST2600_I2CM_CMD_STS);
		i2c_bus->cmd_err = -EBUSY;
		writel(0, i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
		complete(&i2c_bus->cmd_complete);
	}

	/* Handle i2c slave timeout condition */
	if (AST2600_I2CS_INACTIVE_TO & sts) {
		writel(SLAVE_TRIGGER_CMD, i2c_bus->reg_base + AST2600_I2CS_CMD_STS);
		writel(AST2600_I2CS_PKT_DONE, i2c_bus->reg_base + AST2600_I2CS_ISR);
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_STOP, &value);
		i2c_bus->slave_operate = 0;
		return;
	}

	sts &= ~(AST2600_I2CS_PKT_DONE | AST2600_I2CS_PKT_ERROR);

	if (sts & AST2600_I2CS_SLAVE_MATCH)
		i2c_bus->slave_operate = 1;

	switch (sts) {
	case AST2600_I2CS_SLAVE_PENDING | AST2600_I2CS_WAIT_RX_DMA |
		 AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_RX_DONE | AST2600_I2CS_STOP: // #1 1010004 -> 21010094
	case AST2600_I2CS_SLAVE_PENDING |
		 AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_RX_DONE | AST2600_I2CS_STOP: // #1 1010004 ->20010094
	case AST2600_I2CS_SLAVE_PENDING |
		 AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_STOP: // #1 1010004 ->20010090
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_STOP, &value);
		fallthrough;
	case AST2600_I2CS_SLAVE_PENDING |
		 AST2600_I2CS_WAIT_RX_DMA | AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_RX_DONE: // #1 10014 -> 21010084
	case AST2600_I2CS_WAIT_RX_DMA | AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_RX_DONE: // #1 21010014 -> 1010084
	// fix this : with when stop is coming and clr rx len set trigger rx buff size  = 0
	case AST2600_I2CS_WAIT_RX_DMA | AST2600_I2CS_SLAVE_MATCH:	// #1 1010080	//normal coming
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_REQUESTED, &value);
		cmd = SLAVE_TRIGGER_CMD;
		if (sts & AST2600_I2CS_RX_DONE) {
			slave_rx_len = AST2600_I2CC_GET_RX_BUF_LEN(readl(i2c_bus->reg_base +
							       AST2600_I2CC_BUFF_CTRL));
			for (i = 0; i < slave_rx_len; i++) {
				value = readb(i2c_bus->buf_base + 0x10 + i);
				dev_dbg(i2c_bus->dev, "%02x ", value);
				i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_RECEIVED, &value);
			}
		}
		if (readl(i2c_bus->reg_base + AST2600_I2CS_CMD_STS) & AST2600_I2CS_RX_BUFF_EN)
			cmd = 0;
		else
			cmd = SLAVE_TRIGGER_CMD | AST2600_I2CS_RX_BUFF_EN;

		writel(AST2600_I2CC_SET_RX_BUF_LEN(i2c_bus->buf_size),
					i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
		break;
	case AST2600_I2CS_WAIT_RX_DMA | AST2600_I2CS_RX_DONE: //#2 1010004
		cmd = SLAVE_TRIGGER_CMD;
		slave_rx_len = AST2600_I2CC_GET_RX_BUF_LEN(readl(i2c_bus->reg_base +
						       AST2600_I2CC_BUFF_CTRL));
		for (i = 0; i < slave_rx_len; i++) {
			value = readb(i2c_bus->buf_base + 0x10 + i);
			dev_dbg(i2c_bus->dev, "%02x ", value);
			i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_RECEIVED, &value);
		}
		cmd |= AST2600_I2CS_RX_BUFF_EN;
		writel(AST2600_I2CC_SET_RX_BUF_LEN(i2c_bus->buf_size),
						i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
		break;
	case AST2600_I2CS_SLAVE_PENDING | AST2600_I2CS_WAIT_RX_DMA |
				AST2600_I2CS_RX_DONE | AST2600_I2CS_STOP: ///#3 21010014
		// D | P | S
		cmd = SLAVE_TRIGGER_CMD;
		slave_rx_len = AST2600_I2CC_GET_RX_BUF_LEN(readl(i2c_bus->reg_base +
						       AST2600_I2CC_BUFF_CTRL));
		for (i = 0; i < slave_rx_len; i++) {
			value = readb(i2c_bus->buf_base + 0x10 + i);
			dev_dbg(i2c_bus->dev, "%02x ", value);
			i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_RECEIVED, &value);
		}
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_STOP, &value);
		cmd |= AST2600_I2CS_RX_BUFF_EN;
		writel(AST2600_I2CC_SET_RX_BUF_LEN(i2c_bus->buf_size),
						i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
		break;
	case AST2600_I2CS_SLAVE_PENDING | AST2600_I2CS_RX_DONE | AST2600_I2CS_STOP:   /////#3 pre isr 1010004 -> 20010014
		cmd = SLAVE_TRIGGER_CMD;
		slave_rx_len = AST2600_I2CC_GET_RX_BUF_LEN(readl(i2c_bus->reg_base +
							   AST2600_I2CC_BUFF_CTRL));
		for (i = 0; i < slave_rx_len; i++) {
			value = readb(i2c_bus->buf_base + 0x10 + i);
			dev_dbg(i2c_bus->dev, "%02x ", value);
			i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_RECEIVED, &value);
		}
		/* workaround for avoid next start with len != 0 */
		writel(BIT(0), i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_STOP, &value);
		break;
	case AST2600_I2CS_RX_DONE | AST2600_I2CS_STOP:   /////#3 1010014
		cmd = SLAVE_TRIGGER_CMD;
		slave_rx_len = AST2600_I2CC_GET_RX_BUF_LEN(readl(i2c_bus->reg_base +
							   AST2600_I2CC_BUFF_CTRL));
		for (i = 0; i < slave_rx_len; i++) {
			value = readb(i2c_bus->buf_base + 0x10 + i);
			dev_dbg(i2c_bus->dev, "%02x ", value);
			i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_RECEIVED, &value);
		}
		//workaround for avoid next start with len != 0
		writel(BIT(0), i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_STOP, &value);
		break;
	case AST2600_I2CS_WAIT_TX_DMA | AST2600_I2CS_SLAVE_MATCH://2010080
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_READ_REQUESTED, &value);
		dev_dbg(i2c_bus->dev, "tx : %02x ", value);
		writeb(value, i2c_bus->buf_base);
		writel(AST2600_I2CC_SET_TX_BUF_LEN(1),
				i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
		cmd = SLAVE_TRIGGER_CMD | AST2600_I2CS_TX_BUFF_EN;
		break;
	case AST2600_I2CS_WAIT_TX_DMA | AST2600_I2CS_RX_DONE:	//1010080 -> 2010004
	case AST2600_I2CS_WAIT_TX_DMA:	//2010004 -> 2010000
		if (sts & AST2600_I2CS_RX_DONE) {
			slave_rx_len = AST2600_I2CC_GET_RX_BUF_LEN(readl(i2c_bus->reg_base +
							AST2600_I2CC_BUFF_CTRL));
			for (i = 0; i < slave_rx_len; i++) {
				value = readb(i2c_bus->buf_base + 0x10 + i);
				dev_dbg(i2c_bus->dev, "%02x ", value);
				i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_RECEIVED, &value);
			}
			i2c_slave_event(i2c_bus->slave, I2C_SLAVE_READ_REQUESTED, &value);
		} else {
			i2c_slave_event(i2c_bus->slave, I2C_SLAVE_READ_PROCESSED, &value);
		}
		dev_dbg(i2c_bus->dev, "tx : %02x ", value);
		writeb(value, i2c_bus->buf_base);
		writel(AST2600_I2CC_SET_TX_BUF_LEN(1),
				i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
		cmd = SLAVE_TRIGGER_CMD | AST2600_I2CS_TX_BUFF_EN;
		break;
	/* workaround : trigger the cmd twice to fix next state keep 1000000 */
	case AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_RX_DONE: //#3 10014 -> 10084
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_REQUESTED, &value);
		cmd = SLAVE_TRIGGER_CMD | AST2600_I2CS_RX_BUFF_EN;
		writel(cmd, i2c_bus->reg_base + AST2600_I2CS_CMD_STS);
		break;

	case AST2600_I2CS_TX_NAK | AST2600_I2CS_STOP://2010000 -> 10012
	case AST2600_I2CS_STOP:
		cmd = SLAVE_TRIGGER_CMD;
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_STOP, &value);
		break;
	default:
		dev_dbg(i2c_bus->dev, "unhandled slave isr case %x, sts %x\n", sts,
			readl(i2c_bus->reg_base + AST2600_I2CC_STS_AND_BUFF));
		break;
	}

	if (cmd)
		writel(cmd, i2c_bus->reg_base + AST2600_I2CS_CMD_STS);
	writel(AST2600_I2CS_PKT_DONE, i2c_bus->reg_base + AST2600_I2CS_ISR);
	readl(i2c_bus->reg_base + AST2600_I2CS_ISR);

	if ((sts & AST2600_I2CS_STOP) && !(sts & AST2600_I2CS_SLAVE_PENDING))
		i2c_bus->slave_operate = 0;

	dev_dbg(i2c_bus->dev, "slave_rx_len %d, cmd %x\n", slave_rx_len, cmd);
}

static void ast2600_i2c_slave_byte_irq(struct ast2600_i2c_bus *i2c_bus, u32 sts)
{
	u32 i2c_buff = readl(i2c_bus->reg_base + AST2600_I2CC_STS_AND_BUFF);
	u32 cmd = AST2600_I2CS_ACTIVE_ALL;
	u8 byte_data;
	u8 value;

	switch (sts) {
	case AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_RX_DONE | AST2600_I2CS_WAIT_RX_DMA:
		dev_dbg(i2c_bus->dev, "S : Sw|D\n");
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_REQUESTED, &value);
		/* first address match is address */
		byte_data = AST2600_I2CC_GET_RX_BUFF(i2c_buff);
		dev_dbg(i2c_bus->dev, "addr [%x]", byte_data);
		break;
	case AST2600_I2CS_RX_DONE | AST2600_I2CS_WAIT_RX_DMA:
		dev_dbg(i2c_bus->dev, "S : D\n");
		byte_data = AST2600_I2CC_GET_RX_BUFF(i2c_buff);
		dev_dbg(i2c_bus->dev, "rx [%x]", byte_data);
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_WRITE_RECEIVED, &byte_data);
		break;
	case AST2600_I2CS_SLAVE_MATCH | AST2600_I2CS_RX_DONE | AST2600_I2CS_WAIT_TX_DMA:
		cmd |= AST2600_I2CS_TX_CMD;
		dev_dbg(i2c_bus->dev, "S : Sr|D\n");
		byte_data = AST2600_I2CC_GET_RX_BUFF(i2c_buff);
		dev_dbg(i2c_bus->dev, "addr : [%02x]", byte_data);
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_READ_REQUESTED, &byte_data);
		dev_dbg(i2c_bus->dev, "tx: [%02x]\n", byte_data);
		writel(byte_data, i2c_bus->reg_base + AST2600_I2CC_STS_AND_BUFF);
		break;
	case AST2600_I2CS_TX_ACK | AST2600_I2CS_WAIT_TX_DMA:
		cmd |= AST2600_I2CS_TX_CMD;
		dev_dbg(i2c_bus->dev, "S : D\n");
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_READ_PROCESSED, &byte_data);
		dev_dbg(i2c_bus->dev, "tx: [%02x]\n", byte_data);
		writel(byte_data, i2c_bus->reg_base + AST2600_I2CC_STS_AND_BUFF);
		break;
	case AST2600_I2CS_STOP:
	case AST2600_I2CS_STOP | AST2600_I2CS_TX_NAK:
		dev_dbg(i2c_bus->dev, "S : P\n");
		i2c_slave_event(i2c_bus->slave, I2C_SLAVE_STOP, &value);
		break;
	default:
		dev_dbg(i2c_bus->dev, "unhandled pkt isr %x\n", sts);
		break;
	}
	writel(cmd, i2c_bus->reg_base + AST2600_I2CS_CMD_STS);
	writel(sts, i2c_bus->reg_base + AST2600_I2CS_ISR);
	readl(i2c_bus->reg_base + AST2600_I2CS_ISR);
}

static int ast2600_i2c_slave_irq(struct ast2600_i2c_bus *i2c_bus)
{
	u32 ier = readl(i2c_bus->reg_base + AST2600_I2CS_IER);
	u32 isr = readl(i2c_bus->reg_base + AST2600_I2CS_ISR);

	if (!(isr & ier))
		return 0;

	/*
	 * Slave interrupt coming after Master package done
	 * So need handle master first.
	 */
	if (readl(i2c_bus->reg_base + AST2600_I2CM_ISR) & AST2600_I2CM_PKT_DONE)
		return 0;

	dev_dbg(i2c_bus->dev, "isr %x\n", isr);

	isr &= ~(AST2600_I2CS_ADDR_INDICATE_MASK);

	if (AST2600_I2CS_ADDR1_NAK & isr)
		isr &= ~AST2600_I2CS_ADDR1_NAK;

	if (AST2600_I2CS_ADDR2_NAK & isr)
		isr &= ~AST2600_I2CS_ADDR2_NAK;

	if (AST2600_I2CS_ADDR3_NAK & isr)
		isr &= ~AST2600_I2CS_ADDR3_NAK;

	if (AST2600_I2CS_ADDR_MASK & isr)
		isr &= ~AST2600_I2CS_ADDR_MASK;

	if (AST2600_I2CS_PKT_DONE & isr) {
		if (i2c_bus->mode == DMA_MODE)
			ast2600_i2c_slave_packet_dma_irq(i2c_bus, isr);
		else
			ast2600_i2c_slave_packet_buff_irq(i2c_bus, isr);
	} else {
		ast2600_i2c_slave_byte_irq(i2c_bus, isr);
	}

	return 1;
}
#endif

static int ast2600_smbus_do_start(struct ast2600_i2c_bus *i2c_bus)
{
	struct i2c_msg *msg = &i2c_bus->msgs[i2c_bus->msgs_index];
	struct i2c_msg *msgr = &i2c_bus->msgs[i2c_bus->msgs_index + 1];

	int xfer_len = 0;
	int i = 0;
	u32 cmd;

	cmd = AST2600_I2CM_PKT_EN | AST2600_I2CM_PKT_ADDR(msg->addr) | AST2600_I2CM_START_CMD;

	/* send start */

	dev_dbg(i2c_bus->dev, "SMBUS msgs_count (%d)", i2c_bus->msgs_count);

	dev_dbg(i2c_bus->dev, "SMBUS msg0 %sing %d byte%s %s 0x%02x\n",
		msg->flags & I2C_M_RD ? "read" : "write",
		msg->len, msg->len > 1 ? "s" : "",
		msg->flags & I2C_M_RD ? "from" : "to", msg->addr);

	dev_dbg(i2c_bus->dev, "SMBUS msg1 %sing %d byte%s %s 0x%02x\n",
		msgr->flags & I2C_M_RD ? "read" : "write",
		msgr->len, msgr->len > 1 ? "s" : "",
		msgr->flags & I2C_M_RD ? "from" : "to", msgr->addr);

	/*local record tx / rx count*/
	i2c_bus->master_xfer_tx_cnt = 0;
	i2c_bus->master_xfer_rx_cnt = 0;

	i2c_bus->buf_index = 0;

	for (i = 0; i < i2c_bus->msgs_count; i++) {
		if (msg->flags & I2C_M_RD) {
			cmd |= (AST2600_I2CM_RX_CMD | AST2600_I2CM_RX_BUFF_EN);

			if (msg->flags & I2C_M_RECV_LEN) {
				dev_dbg(i2c_bus->dev, "smbus read\n");
				xfer_len = 1;
				cmd &= ~(AST2600_I2CM_STOP_CMD);
			} else {
				if (msg->len > i2c_bus->buf_size) {
					xfer_len = i2c_bus->buf_size;
					cmd &= ~(AST2600_I2CM_STOP_CMD);
				} else {
					xfer_len = msg->len;
					cmd |= MASTER_TRIGGER_LAST_STOP;
				}
			}
			writel(AST2600_I2CC_SET_RX_BUF_LEN(xfer_len),
			       i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
		} else {
			u8 wbuf[4];
			/* buff mode */
			if (msg->len > i2c_bus->buf_size) {
				xfer_len = i2c_bus->buf_size;
			} else {
				xfer_len = msg->len;
				cmd |= AST2600_I2CM_STOP_CMD;
			}
			if (xfer_len) {
				cmd |= AST2600_I2CM_TX_BUFF_EN | AST2600_I2CM_TX_CMD;
				if (readl(i2c_bus->reg_base + AST2600_I2CS_ISR))
					return -ENOMEM;
				writel(AST2600_I2CC_SET_TX_BUF_LEN(xfer_len),
				       i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
				if (readl(i2c_bus->reg_base + AST2600_I2CS_ISR))
					return -ENOMEM;
				for (i = 0; i < xfer_len; i++) {
					wbuf[i % 4] = msg->buf[i];
					if (i % 4 == 3)
						writel(*(u32 *)wbuf, i2c_bus->buf_base + i - 3);
					dev_dbg(i2c_bus->dev, "[%02x]\n", msg->buf[i]);
				}
				if (--i % 4 != 3)
					writel(*(u32 *)wbuf, i2c_bus->buf_base + i - (i % 4));
			}
			if (readl(i2c_bus->reg_base + AST2600_I2CS_ISR))
				return -ENOMEM;
		}
		msg++;
	}

	dev_dbg(i2c_bus->dev, "len %d , cmd %x\n", xfer_len, cmd);
	writel(cmd, i2c_bus->reg_base + AST2600_I2CM_CMD_STS);
	return 0;
}

static int ast2600_i2c_do_start(struct ast2600_i2c_bus *i2c_bus)
{
	struct i2c_msg *msg = &i2c_bus->msgs[i2c_bus->msgs_index];
	int xfer_len = 0;
	int i = 0;
	u32 cmd;

	cmd = AST2600_I2CM_PKT_EN | AST2600_I2CM_PKT_ADDR(msg->addr) | AST2600_I2CM_START_CMD;

	/* send start */
	dev_dbg(i2c_bus->dev, "[%d] %sing %d byte%s %s 0x%02x\n",
		i2c_bus->msgs_index, msg->flags & I2C_M_RD ? "read" : "write",
		msg->len, msg->len > 1 ? "s" : "",
		msg->flags & I2C_M_RD ? "from" : "to", msg->addr);

	i2c_bus->master_xfer_cnt = 0;
	i2c_bus->buf_index = 0;

	if (msg->flags & I2C_M_RD) {
		cmd |= AST2600_I2CM_RX_CMD;
		if (i2c_bus->mode == DMA_MODE) {
			/* dma mode */
			cmd |= AST2600_I2CM_RX_DMA_EN;

			if (msg->flags & I2C_M_RECV_LEN) {
				dev_dbg(i2c_bus->dev, "smbus read\n");
				xfer_len = 1;
			} else {
				if (msg->len > AST2600_I2C_DMA_SIZE) {
					xfer_len = AST2600_I2C_DMA_SIZE;
				} else {
					xfer_len = msg->len;
					if (i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) {
						dev_dbg(i2c_bus->dev, "last stop\n");
						cmd |= MASTER_TRIGGER_LAST_STOP;
					}
				}
			}
			writel(AST2600_I2CM_SET_RX_DMA_LEN(xfer_len - 1),
			       i2c_bus->reg_base + AST2600_I2CM_DMA_LEN);
			i2c_bus->master_safe_buf = i2c_get_dma_safe_msg_buf(msg, 1);
			if (!i2c_bus->master_safe_buf)
				return -ENOMEM;
			i2c_bus->master_dma_addr =
				dma_map_single(i2c_bus->dev, i2c_bus->master_safe_buf, msg->len, DMA_FROM_DEVICE);
			if (dma_mapping_error(i2c_bus->dev, i2c_bus->master_dma_addr)) {
				i2c_put_dma_safe_msg_buf(i2c_bus->master_safe_buf, msg, false);
				i2c_bus->master_safe_buf = NULL;
				return -ENOMEM;
			}
			writel(i2c_bus->master_dma_addr, i2c_bus->reg_base + AST2600_I2CM_RX_DMA);
		} else if (i2c_bus->mode == BUFF_MODE) {
			/* buff mode */
			cmd |= AST2600_I2CM_RX_BUFF_EN;
			if (msg->flags & I2C_M_RECV_LEN) {
				dev_dbg(i2c_bus->dev, "smbus read\n");
				xfer_len = 1;
			} else {
				if (msg->len > i2c_bus->buf_size) {
					xfer_len = i2c_bus->buf_size;
				} else {
					xfer_len = msg->len;
					if (i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) {
						dev_dbg(i2c_bus->dev, "last stop\n");
						cmd |= MASTER_TRIGGER_LAST_STOP;
					}
				}
			}
			writel(AST2600_I2CC_SET_RX_BUF_LEN(xfer_len),
			       i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
		} else {
			/* byte mode */
			xfer_len = 1;
			if (msg->flags & I2C_M_RECV_LEN) {
				dev_dbg(i2c_bus->dev, "smbus read\n");
			} else {
				if (i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) {
					if (msg->len == 1) {
						dev_dbg(i2c_bus->dev, "last stop\n");
						cmd |= MASTER_TRIGGER_LAST_STOP;
					}
				}
			}
		}
	} else {
		if (i2c_bus->mode == DMA_MODE) {
			/* dma mode */
			if (msg->len > AST2600_I2C_DMA_SIZE) {
				xfer_len = AST2600_I2C_DMA_SIZE;
			} else {
				if (i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) {
					dev_dbg(i2c_bus->dev, "with stop\n");
					cmd |= AST2600_I2CM_STOP_CMD;
				}
				xfer_len = msg->len;
			}

			if (xfer_len) {
				cmd |= AST2600_I2CM_TX_DMA_EN | AST2600_I2CM_TX_CMD;
				writel(AST2600_I2CM_SET_TX_DMA_LEN(xfer_len - 1),
				       i2c_bus->reg_base + AST2600_I2CM_DMA_LEN);
				i2c_bus->master_safe_buf = i2c_get_dma_safe_msg_buf(msg, 1);
				if (!i2c_bus->master_safe_buf)
					return -ENOMEM;
				i2c_bus->master_dma_addr =
					dma_map_single(i2c_bus->dev, i2c_bus->master_safe_buf, msg->len,
						       DMA_TO_DEVICE);
				if (dma_mapping_error(i2c_bus->dev, i2c_bus->master_dma_addr)) {
					i2c_put_dma_safe_msg_buf(i2c_bus->master_safe_buf, msg, false);
					i2c_bus->master_safe_buf = NULL;
					return -ENOMEM;
				}
				writel(i2c_bus->master_dma_addr,
				       i2c_bus->reg_base + AST2600_I2CM_TX_DMA);
			}
		} else if (i2c_bus->mode == BUFF_MODE) {
			u8 wbuf[4];
			/* buff mode */
			if (msg->len > i2c_bus->buf_size) {
				xfer_len = i2c_bus->buf_size;
			} else {
				if (i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) {
					dev_dbg(i2c_bus->dev, "with stop\n");
					cmd |= AST2600_I2CM_STOP_CMD;
				}
				xfer_len = msg->len;
			}
			if (xfer_len) {
				cmd |= AST2600_I2CM_TX_BUFF_EN | AST2600_I2CM_TX_CMD;
				if (readl(i2c_bus->reg_base + AST2600_I2CS_ISR))
					return -ENOMEM;
				writel(AST2600_I2CC_SET_TX_BUF_LEN(xfer_len),
				       i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
				if (readl(i2c_bus->reg_base + AST2600_I2CS_ISR))
					return -ENOMEM;
				for (i = 0; i < xfer_len; i++) {
					wbuf[i % 4] = msg->buf[i];
					if (i % 4 == 3)
						writel(*(u32 *)wbuf, i2c_bus->buf_base + i - 3);
					dev_dbg(i2c_bus->dev, "[%02x]\n", msg->buf[i]);
				}
				if (--i % 4 != 3)
					writel(*(u32 *)wbuf, i2c_bus->buf_base + i - (i % 4));
			}
			if (readl(i2c_bus->reg_base + AST2600_I2CS_ISR))
				return -ENOMEM;
		} else {
			/* byte mode */
			if ((i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) && (msg->len <= 1)) {
				dev_dbg(i2c_bus->dev, "with stop\n");
				cmd |= AST2600_I2CM_STOP_CMD;
			}

			if (msg->len) {
				cmd |= AST2600_I2CM_TX_CMD;
				xfer_len = 1;
				dev_dbg(i2c_bus->dev, "w [0] : %02x\n", msg->buf[0]);
				writel(msg->buf[0], i2c_bus->reg_base + AST2600_I2CC_STS_AND_BUFF);
			} else {
				xfer_len = 0;
			}
		}
	}
	dev_dbg(i2c_bus->dev, "len %d , cmd %x\n", xfer_len, cmd);
	writel(cmd, i2c_bus->reg_base + AST2600_I2CM_CMD_STS);
	return 0;
}

static int ast2600_i2c_irq_err_to_errno(u32 irq_status)
{
	if (irq_status & AST2600_I2CM_ARBIT_LOSS)
		return -EAGAIN;
	if (irq_status & (AST2600_I2CM_SDA_DL_TO | AST2600_I2CM_SCL_LOW_TO))
		return -EBUSY;
	if (irq_status & (AST2600_I2CM_ABNORMAL))
		return -EPROTO;

	return 0;
}

static void ast2600_i2c_smbus_package_irq(struct ast2600_i2c_bus *i2c_bus, u32 sts)
{
	struct i2c_msg *msg = &i2c_bus->msgs[i2c_bus->msgs_index];
	u32 cmd = AST2600_I2CM_PKT_EN;
	int xfer_len_tx;
	int xfer_len_rx;
	int i;

	sts &= ~AST2600_I2CM_PKT_DONE;
	writel(AST2600_I2CM_PKT_DONE, i2c_bus->reg_base + AST2600_I2CM_ISR);
	switch (sts) {
	case AST2600_I2CM_PKT_ERROR:
		dev_dbg(i2c_bus->dev, "M : ERROR only\n");
		i2c_bus->cmd_err = -EAGAIN;
		complete(&i2c_bus->cmd_complete);
		break;
	case AST2600_I2CM_PKT_ERROR | AST2600_I2CM_TX_NAK: /* a0 fix for issue */
		fallthrough;
	case AST2600_I2CM_PKT_ERROR | AST2600_I2CM_TX_NAK | AST2600_I2CM_NORMAL_STOP:
		dev_dbg(i2c_bus->dev, "M : TX NAK | NORMAL STOP\n");
		i2c_bus->cmd_err = -ENXIO;
		complete(&i2c_bus->cmd_complete);
		break;
	case AST2600_I2CM_NORMAL_STOP:
		/* write 0 byte only have stop isr */
		dev_dbg(i2c_bus->dev, "M clear isr: AST2600_I2CM_NORMAL_STOP = %x\n", sts);
		i2c_bus->msgs_index++;
		if (i2c_bus->msgs_index < i2c_bus->msgs_count) {
			if (ast2600_i2c_do_start(i2c_bus)) {
				i2c_bus->cmd_err = -ENOMEM;
				complete(&i2c_bus->cmd_complete);
			}
		} else {
			i2c_bus->cmd_err = i2c_bus->msgs_index;
			complete(&i2c_bus->cmd_complete);
		}
		break;
	case AST2600_I2CM_TX_ACK:
		//dev_dbg(i2c_bus->dev, "M : AST2600_I2CM_TX_ACK = %x\n", sts);
	case AST2600_I2CM_TX_ACK | AST2600_I2CM_NORMAL_STOP:
		if (i2c_bus->mode == DMA_MODE)
			xfer_len_tx = AST2600_I2C_GET_TX_DMA_LEN(readl(i2c_bus->reg_base +
							  AST2600_I2CM_DMA_LEN_STS));
		else if (i2c_bus->mode == BUFF_MODE)
			xfer_len_tx = AST2600_I2CC_GET_TX_BUF_LEN(readl(i2c_bus->reg_base +
							   AST2600_I2CC_BUFF_CTRL));

		dev_dbg(i2c_bus->dev,
			"M : AST2600_I2CM_TX_ACK | AST2600_I2CM_NORMAL_STOP= %x (%d)\n", sts, xfer_len_tx);
		i2c_bus->master_xfer_tx_cnt += xfer_len_tx;

		if (i2c_bus->master_xfer_tx_cnt == msg->len) {
			if (i2c_bus->mode == DMA_MODE) {
				dma_unmap_single(i2c_bus->dev, i2c_bus->master_dma_addr, msg->len,
						 DMA_TO_DEVICE);
				i2c_put_dma_safe_msg_buf(i2c_bus->master_safe_buf, msg, true);
				i2c_bus->master_safe_buf = NULL;
			}
			i2c_bus->msgs_index++;
			if (i2c_bus->msgs_index == i2c_bus->msgs_count) {
				i2c_bus->cmd_err = i2c_bus->msgs_index;
				complete(&i2c_bus->cmd_complete);
			} else {
				if (ast2600_i2c_do_start(i2c_bus)) {
					i2c_bus->cmd_err = -ENOMEM;
					complete(&i2c_bus->cmd_complete);
				}
			}
		} else {
			/* do next tx */
			cmd |= AST2600_I2CM_TX_CMD;
			xfer_len_tx = msg->len - i2c_bus->master_xfer_tx_cnt;

			if (i2c_bus->mode == DMA_MODE) {
				cmd |= AST2600_I2CM_TX_DMA_EN;
				if (xfer_len_tx > AST2600_I2C_DMA_SIZE) {
					xfer_len_tx = AST2600_I2C_DMA_SIZE;
				} else {
					if (i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) {
						dev_dbg(i2c_bus->dev, "M: STOP\n");
						cmd |= AST2600_I2CM_STOP_CMD;
					}
				}
				writel(AST2600_I2CM_SET_TX_DMA_LEN(xfer_len_tx - 1),
				       i2c_bus->reg_base + AST2600_I2CM_DMA_LEN);
				dev_dbg(i2c_bus->dev, "next tx xfer_len: %d, offset %d\n",
					xfer_len_tx, i2c_bus->master_xfer_tx_cnt);
				writel(i2c_bus->master_dma_addr + i2c_bus->master_xfer_tx_cnt,
				       i2c_bus->reg_base + AST2600_I2CM_TX_DMA);
			} else if (i2c_bus->mode == BUFF_MODE) {
				u8 wbuf[4];

				cmd |= AST2600_I2CM_TX_BUFF_EN;
				if (xfer_len_tx > i2c_bus->buf_size) {
					xfer_len_tx = i2c_bus->buf_size;
				} else {
					if (i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) {
						dev_dbg(i2c_bus->dev, "M: STOP\n");
						cmd |= AST2600_I2CM_STOP_CMD;
					}
				}
				for (i = 0; i < xfer_len_tx; i++) {
					wbuf[i % 4] = msg->buf[i2c_bus->master_xfer_tx_cnt + i];
					if (i % 4 == 3)
						writel(*(u32 *)wbuf, i2c_bus->buf_base + i - 3);
					dev_dbg(i2c_bus->dev, "[%02x]\n",
						msg->buf[i2c_bus->master_xfer_cnt + i]);
				}
				if (--i % 4 != 3)
					writel(*(u32 *)wbuf, i2c_bus->buf_base + i - (i % 4));
				writel(AST2600_I2CC_SET_TX_BUF_LEN(xfer_len_tx),
				       i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
			}

			dev_dbg(i2c_bus->dev, "next tx %d cmd: %x\n", xfer_len_tx, cmd);
			writel(cmd, i2c_bus->reg_base + AST2600_I2CM_CMD_STS);
		}
		break;
	case AST2600_I2CM_RX_DONE:
#ifdef CONFIG_I2C_SLAVE
		/* Workaround for master/slave package mode enable rx done stuck issue
		 * When master go for first read (RX_DONE), slave mode will also effect
		 * Then controller will send nack, not operate anymore.
		 */
		if (readl(i2c_bus->reg_base + AST2600_I2CS_CMD_STS) & AST2600_I2CS_PKT_MODE_EN) {
			u32 slave_cmd = readl(i2c_bus->reg_base + AST2600_I2CS_CMD_STS);

			writel(0, i2c_bus->reg_base + AST2600_I2CS_CMD_STS);
			writel(slave_cmd, i2c_bus->reg_base + AST2600_I2CS_CMD_STS);
		}
		fallthrough;
#endif
	case AST2600_I2CM_RX_DONE | AST2600_I2CM_NORMAL_STOP:
		/* do next rx */
		if (i2c_bus->mode == DMA_MODE) {
			xfer_len_rx = AST2600_I2C_GET_RX_DMA_LEN(readl(i2c_bus->reg_base +
						AST2600_I2CM_DMA_LEN_STS));
		} else if (i2c_bus->mode == BUFF_MODE) {
			xfer_len_rx = AST2600_I2CC_GET_RX_BUF_LEN(readl(i2c_bus->reg_base +
						AST2600_I2CC_BUFF_CTRL));
			for (i = 0; i < xfer_len_rx; i++)
				msg->buf[i2c_bus->master_xfer_rx_cnt + i] =
					readb(i2c_bus->buf_base + 0x10 + i);
		}

		dev_dbg(i2c_bus->dev,
			"M : AST2600_I2CM_RX_DONE | AST2600_I2CM_NORMAL_STOP = %x (%d)\n", sts, xfer_len_rx);

		if (msg->flags & I2C_M_RECV_LEN) {
			if (unlikely(msg->buf[0] > I2C_SMBUS_BLOCK_MAX))
				dev_dbg(i2c_bus->dev, "smbus len = %x is over max length\n", msg->buf[0]);
			else
				dev_dbg(i2c_bus->dev, "smbus first len = %x\n", msg->buf[0]);

			msg->len = min_t(unsigned int, msg->buf[0], I2C_SMBUS_BLOCK_MAX);
			msg->len += ((msg->flags & I2C_CLIENT_PEC) ? 2 : 1);
			msg->flags &= ~I2C_M_RECV_LEN;
		}
		i2c_bus->master_xfer_rx_cnt += xfer_len_rx;
		dev_dbg(i2c_bus->dev, "master_xfer_cnt [%d/%d]\n", i2c_bus->master_xfer_rx_cnt,
			msg->len);

		if (i2c_bus->master_xfer_rx_cnt == msg->len) {
			if (i2c_bus->mode == DMA_MODE) {
				dma_unmap_single(i2c_bus->dev, i2c_bus->master_dma_addr, msg->len,
						 DMA_FROM_DEVICE);
				i2c_put_dma_safe_msg_buf(i2c_bus->master_safe_buf, msg, true);
				i2c_bus->master_safe_buf = NULL;
			}

			for (i = 0; i < msg->len; i++)
				dev_dbg(i2c_bus->dev, "M: r %d:[%x]\n", i, msg->buf[i]);
			i2c_bus->msgs_index++;
			if (i2c_bus->msgs_index == i2c_bus->msgs_count) {
				i2c_bus->cmd_err = i2c_bus->msgs_index;
				complete(&i2c_bus->cmd_complete);
			} else {
				if (ast2600_i2c_do_start(i2c_bus)) {
					i2c_bus->cmd_err = -ENOMEM;
					complete(&i2c_bus->cmd_complete);
				}
			}
		} else {
			/* next rx */
			cmd |= AST2600_I2CM_RX_CMD;
			xfer_len_rx = msg->len - i2c_bus->master_xfer_rx_cnt;

			if (i2c_bus->mode == DMA_MODE) {
				cmd |= AST2600_I2CM_RX_DMA_EN;
				if (xfer_len_rx > AST2600_I2C_DMA_SIZE) {
					xfer_len_rx = AST2600_I2C_DMA_SIZE;
				} else {
					if (i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) {
						dev_dbg(i2c_bus->dev, "last stop\n");
						cmd |= MASTER_TRIGGER_LAST_STOP;
					}
				}
				dev_dbg(i2c_bus->dev, "M: next rx len [%d/%d] , cmd %x\n", xfer_len_rx,
					msg->len, cmd);
				writel(AST2600_I2CM_SET_RX_DMA_LEN(xfer_len_rx - 1),
				       i2c_bus->reg_base + AST2600_I2CM_DMA_LEN);
				writel(i2c_bus->master_dma_addr + i2c_bus->master_xfer_rx_cnt,
				       i2c_bus->reg_base + AST2600_I2CM_RX_DMA);
			} else if (i2c_bus->mode == BUFF_MODE) {
				cmd |= AST2600_I2CM_RX_BUFF_EN;
				if (xfer_len_rx > i2c_bus->buf_size) {
					xfer_len_rx = i2c_bus->buf_size;
				} else {
					if (i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) {
						dev_dbg(i2c_bus->dev, "last stop\n");
						cmd |= MASTER_TRIGGER_LAST_STOP;
					}
				}
				writel(AST2600_I2CC_SET_RX_BUF_LEN(xfer_len_rx),
				       i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
			}

			dev_dbg(i2c_bus->dev, "M: next rx len %d, cmd %x\n", xfer_len_rx, cmd);
			writel(cmd, i2c_bus->reg_base + AST2600_I2CM_CMD_STS);
		}
		break;
	/* smbus normal case */
	case AST2600_I2CM_TX_ACK | AST2600_I2CM_RX_DONE: /*I2C_M_RECV_LEN case*/
	case AST2600_I2CM_TX_ACK | AST2600_I2CM_RX_DONE | AST2600_I2CM_NORMAL_STOP:
		if (i2c_bus->mode == DMA_MODE)
			xfer_len_tx = AST2600_I2C_GET_TX_DMA_LEN(readl(i2c_bus->reg_base +
							  AST2600_I2CM_DMA_LEN_STS));
		else if (i2c_bus->mode == BUFF_MODE)
			xfer_len_tx = AST2600_I2CC_GET_TX_BUF_LEN(readl(i2c_bus->reg_base +
							   AST2600_I2CC_BUFF_CTRL));

		dev_dbg(i2c_bus->dev,
			"M : AST2600_I2CM_TX_ACK | AST2600_I2CM_NORMAL_STOP= %x (%d)\n", sts, xfer_len_tx);
		i2c_bus->master_xfer_tx_cnt += xfer_len_tx;

		if (i2c_bus->master_xfer_tx_cnt == msg->len) {
			if (i2c_bus->mode == DMA_MODE) {
				dma_unmap_single(i2c_bus->dev, i2c_bus->master_dma_addr, msg->len,
						 DMA_TO_DEVICE);
				i2c_put_dma_safe_msg_buf(i2c_bus->master_safe_buf, msg, true);
				i2c_bus->master_safe_buf = NULL;
			}
			i2c_bus->msgs_index++;
		}

		msg = &i2c_bus->msgs[i2c_bus->msgs_index];

		/* handle rx */
		if (i2c_bus->mode == DMA_MODE) {
			xfer_len_rx = AST2600_I2C_GET_RX_DMA_LEN(readl(i2c_bus->reg_base +
						AST2600_I2CM_DMA_LEN_STS));
		} else if (i2c_bus->mode == BUFF_MODE) {
			xfer_len_rx = AST2600_I2CC_GET_RX_BUF_LEN(readl(i2c_bus->reg_base +
						AST2600_I2CC_BUFF_CTRL));
			for (i = 0; i < xfer_len_rx; i++)
				msg->buf[i2c_bus->master_xfer_rx_cnt + i] =
					readb(i2c_bus->buf_base + 0x10 + i);
		}

		dev_dbg(i2c_bus->dev,
			"M : AST2600_I2CM_RX_DONE | AST2600_I2CM_NORMAL_STOP = %x (%d)\n", sts, xfer_len_rx);

		if (msg->flags & I2C_M_RECV_LEN) {
			if (unlikely(msg->buf[0] > I2C_SMBUS_BLOCK_MAX))
				dev_dbg(i2c_bus->dev, "smbus len = %x is over max length\n", msg->buf[0]);
			else
				dev_dbg(i2c_bus->dev, "smbus first len = %x\n", msg->buf[0]);

			msg->len = min_t(unsigned int, msg->buf[0], I2C_SMBUS_BLOCK_MAX);
			msg->len += ((msg->flags & I2C_CLIENT_PEC) ? 2 : 1);
			msg->flags &= ~I2C_M_RECV_LEN;
		}
		i2c_bus->master_xfer_rx_cnt += xfer_len_rx;
		dev_dbg(i2c_bus->dev, "master_xfer_cnt [%d/%d]\n", i2c_bus->master_xfer_rx_cnt,
			msg->len);

		if (i2c_bus->master_xfer_rx_cnt == msg->len) {
			if (i2c_bus->mode == DMA_MODE) {
				dma_unmap_single(i2c_bus->dev, i2c_bus->master_dma_addr, msg->len,
						 DMA_FROM_DEVICE);
				i2c_put_dma_safe_msg_buf(i2c_bus->master_safe_buf, msg, true);
				i2c_bus->master_safe_buf = NULL;
			}

			for (i = 0; i < msg->len; i++)
				dev_dbg(i2c_bus->dev, "M: r %d:[%x]\n", i, msg->buf[i]);
			i2c_bus->msgs_index++;
			if (i2c_bus->msgs_index == i2c_bus->msgs_count) {
				i2c_bus->cmd_err = i2c_bus->msgs_index;
				complete(&i2c_bus->cmd_complete);
			} else {
				if (ast2600_i2c_do_start(i2c_bus)) {
					i2c_bus->cmd_err = -ENOMEM;
					complete(&i2c_bus->cmd_complete);
				}
			}
		} else {
			/* next rx */
			cmd |= AST2600_I2CM_RX_CMD;
			xfer_len_rx = msg->len - i2c_bus->master_xfer_rx_cnt;

			if (i2c_bus->mode == DMA_MODE) {
				cmd |= AST2600_I2CM_RX_DMA_EN;
				if (xfer_len_rx > AST2600_I2C_DMA_SIZE) {
					xfer_len_rx = AST2600_I2C_DMA_SIZE;
				} else {
					if (i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) {
						dev_dbg(i2c_bus->dev, "last stop\n");
						cmd |= MASTER_TRIGGER_LAST_STOP;
					}
				}
				dev_dbg(i2c_bus->dev, "M: next rx len [%d/%d] , cmd %x\n", xfer_len_rx,
					msg->len, cmd);
				writel(AST2600_I2CM_SET_RX_DMA_LEN(xfer_len_rx - 1),
				       i2c_bus->reg_base + AST2600_I2CM_DMA_LEN);
				writel(i2c_bus->master_dma_addr + i2c_bus->master_xfer_rx_cnt,
				       i2c_bus->reg_base + AST2600_I2CM_RX_DMA);
			} else if (i2c_bus->mode == BUFF_MODE) {
				cmd |= AST2600_I2CM_RX_BUFF_EN;
				if (xfer_len_rx > i2c_bus->buf_size) {
					xfer_len_rx = i2c_bus->buf_size;
				} else {
					if (i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) {
						dev_dbg(i2c_bus->dev, "last stop\n");
						cmd |= MASTER_TRIGGER_LAST_STOP;
					}
				}
				writel(AST2600_I2CC_SET_RX_BUF_LEN(xfer_len_rx),
				       i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
			}

			dev_dbg(i2c_bus->dev, "M: next rx len %d, cmd %x\n", xfer_len_rx, cmd);
			writel(cmd, i2c_bus->reg_base + AST2600_I2CM_CMD_STS);
		}
		break;

	default:
		dev_dbg(i2c_bus->dev, "todo care sts %x\n", sts);
		break;
	}
}

static void ast2600_i2c_master_package_irq(struct ast2600_i2c_bus *i2c_bus, u32 sts)
{
	struct i2c_msg *msg = &i2c_bus->msgs[i2c_bus->msgs_index];
	u32 cmd = AST2600_I2CM_PKT_EN;
	int xfer_len;
	int i;

	sts &= ~AST2600_I2CM_PKT_DONE;
	writel(AST2600_I2CM_PKT_DONE, i2c_bus->reg_base + AST2600_I2CM_ISR);
	switch (sts) {
	case AST2600_I2CM_PKT_ERROR:
		dev_dbg(i2c_bus->dev, "M : ERROR only\n");
		i2c_bus->cmd_err = -EAGAIN;
		complete(&i2c_bus->cmd_complete);
		break;
	case AST2600_I2CM_PKT_ERROR | AST2600_I2CM_TX_NAK: /* a0 fix for issue */
		fallthrough;
	case AST2600_I2CM_PKT_ERROR | AST2600_I2CM_TX_NAK | AST2600_I2CM_NORMAL_STOP:
		dev_dbg(i2c_bus->dev, "M : TX NAK | NORMAL STOP\n");
		i2c_bus->cmd_err = -ENXIO;
		complete(&i2c_bus->cmd_complete);
		break;
	case AST2600_I2CM_NORMAL_STOP:
		/* write 0 byte only have stop isr */
		dev_dbg(i2c_bus->dev, "M clear isr: AST2600_I2CM_NORMAL_STOP = %x\n", sts);
		i2c_bus->msgs_index++;
		if (i2c_bus->msgs_index < i2c_bus->msgs_count) {
			if (ast2600_i2c_do_start(i2c_bus)) {
				i2c_bus->cmd_err = -ENOMEM;
				complete(&i2c_bus->cmd_complete);
			}
		} else {
			i2c_bus->cmd_err = i2c_bus->msgs_index;
			complete(&i2c_bus->cmd_complete);
		}
		break;
	case AST2600_I2CM_TX_ACK:
		//dev_dbg(i2c_bus->dev, "M : AST2600_I2CM_TX_ACK = %x\n", sts);
	case AST2600_I2CM_TX_ACK | AST2600_I2CM_NORMAL_STOP:
		if (i2c_bus->mode == DMA_MODE)
			xfer_len = AST2600_I2C_GET_TX_DMA_LEN(readl(i2c_bus->reg_base +
							  AST2600_I2CM_DMA_LEN_STS));
		else if (i2c_bus->mode == BUFF_MODE)
			xfer_len = AST2600_I2CC_GET_TX_BUF_LEN(readl(i2c_bus->reg_base +
							   AST2600_I2CC_BUFF_CTRL));
		else
			xfer_len = 1;

		dev_dbg(i2c_bus->dev,
			"M : AST2600_I2CM_TX_ACK | AST2600_I2CM_NORMAL_STOP= %x (%d)\n", sts, xfer_len);
		i2c_bus->master_xfer_cnt += xfer_len;

		if (i2c_bus->master_xfer_cnt == msg->len) {
			if (i2c_bus->mode == DMA_MODE) {
				dma_unmap_single(i2c_bus->dev, i2c_bus->master_dma_addr, msg->len,
						 DMA_TO_DEVICE);
				i2c_put_dma_safe_msg_buf(i2c_bus->master_safe_buf, msg, true);
				i2c_bus->master_safe_buf = NULL;
			}
			i2c_bus->msgs_index++;
			if (i2c_bus->msgs_index == i2c_bus->msgs_count) {
				i2c_bus->cmd_err = i2c_bus->msgs_index;
				complete(&i2c_bus->cmd_complete);
			} else {
				if (ast2600_i2c_do_start(i2c_bus)) {
					i2c_bus->cmd_err = -ENOMEM;
					complete(&i2c_bus->cmd_complete);
				}
			}
		} else {
			/* do next tx */
			cmd |= AST2600_I2CM_TX_CMD;
			if (i2c_bus->mode == DMA_MODE) {
				cmd |= AST2600_I2CM_TX_DMA_EN;
				xfer_len = msg->len - i2c_bus->master_xfer_cnt;
				if (xfer_len > AST2600_I2C_DMA_SIZE) {
					xfer_len = AST2600_I2C_DMA_SIZE;
				} else {
					if (i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) {
						dev_dbg(i2c_bus->dev, "M: STOP\n");
						cmd |= AST2600_I2CM_STOP_CMD;
					}
				}
				writel(AST2600_I2CM_SET_TX_DMA_LEN(xfer_len - 1),
				       i2c_bus->reg_base + AST2600_I2CM_DMA_LEN);
				dev_dbg(i2c_bus->dev, "next tx xfer_len: %d, offset %d\n",
					xfer_len, i2c_bus->master_xfer_cnt);
				writel(i2c_bus->master_dma_addr + i2c_bus->master_xfer_cnt,
				       i2c_bus->reg_base + AST2600_I2CM_TX_DMA);
			} else if (i2c_bus->mode == BUFF_MODE) {
				u8 wbuf[4];

				cmd |= AST2600_I2CM_TX_BUFF_EN;
				xfer_len = msg->len - i2c_bus->master_xfer_cnt;
				if (xfer_len > i2c_bus->buf_size) {
					xfer_len = i2c_bus->buf_size;
				} else {
					if (i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) {
						dev_dbg(i2c_bus->dev, "M: STOP\n");
						cmd |= AST2600_I2CM_STOP_CMD;
					}
				}
				for (i = 0; i < xfer_len; i++) {
					wbuf[i % 4] = msg->buf[i2c_bus->master_xfer_cnt + i];
					if (i % 4 == 3)
						writel(*(u32 *)wbuf, i2c_bus->buf_base + i - 3);
					dev_dbg(i2c_bus->dev, "[%02x]\n",
						msg->buf[i2c_bus->master_xfer_cnt + i]);
				}
				if (--i % 4 != 3)
					writel(*(u32 *)wbuf, i2c_bus->buf_base + i - (i % 4));
				writel(AST2600_I2CC_SET_TX_BUF_LEN(xfer_len),
				       i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
			} else {
				/* byte */
				if ((i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) &&
				    ((i2c_bus->master_xfer_cnt + 1) == msg->len)) {
					dev_dbg(i2c_bus->dev, "M: STOP\n");
					cmd |= AST2600_I2CM_STOP_CMD;
				}
				dev_dbg(i2c_bus->dev, "tx buff[%x]\n",
					msg->buf[i2c_bus->master_xfer_cnt]);
				writel(msg->buf[i2c_bus->master_xfer_cnt],
				       i2c_bus->reg_base + AST2600_I2CC_STS_AND_BUFF);
			}
			dev_dbg(i2c_bus->dev, "next tx %d cmd: %x\n", xfer_len, cmd);
			writel(cmd, i2c_bus->reg_base + AST2600_I2CM_CMD_STS);
		}
		break;
	case AST2600_I2CM_RX_DONE:
#ifdef CONFIG_I2C_SLAVE
		/* Workaround for master/slave package mode enable rx done stuck issue
		 * When master go for first read (RX_DONE), slave mode will also effect
		 * Then controller will send nack, not operate anymore.
		 */
		if (readl(i2c_bus->reg_base + AST2600_I2CS_CMD_STS) & AST2600_I2CS_PKT_MODE_EN) {
			u32 slave_cmd = readl(i2c_bus->reg_base + AST2600_I2CS_CMD_STS);

			writel(0, i2c_bus->reg_base + AST2600_I2CS_CMD_STS);
			writel(slave_cmd, i2c_bus->reg_base + AST2600_I2CS_CMD_STS);
		}
		fallthrough;
#endif
	case AST2600_I2CM_RX_DONE | AST2600_I2CM_NORMAL_STOP:
		/* do next rx */
		if (i2c_bus->mode == DMA_MODE) {
			xfer_len = AST2600_I2C_GET_RX_DMA_LEN(readl(i2c_bus->reg_base +
							  AST2600_I2CM_DMA_LEN_STS));
		} else if (i2c_bus->mode == BUFF_MODE) {
			xfer_len = AST2600_I2CC_GET_RX_BUF_LEN(
						readl(i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL));
			for (i = 0; i < xfer_len; i++)
				msg->buf[i2c_bus->master_xfer_cnt + i] =
					readb(i2c_bus->buf_base + 0x10 + i);
		} else {
			xfer_len = 1;
			msg->buf[i2c_bus->master_xfer_cnt] =
				AST2600_I2CC_GET_RX_BUFF(readl(i2c_bus->reg_base +
						     AST2600_I2CC_STS_AND_BUFF));
		}

		dev_dbg(i2c_bus->dev,
			"M : AST2600_I2CM_RX_DONE | AST2600_I2CM_NORMAL_STOP = %x (%d)\n", sts, xfer_len);

		if (msg->flags & I2C_M_RECV_LEN) {
			if (unlikely(msg->buf[0] > I2C_SMBUS_BLOCK_MAX))
				dev_dbg(i2c_bus->dev, "smbus len = %x is over max length\n", msg->buf[0]);
			else
				dev_dbg(i2c_bus->dev, "smbus first len = %x\n", msg->buf[0]);

			msg->len = min_t(unsigned int, msg->buf[0], I2C_SMBUS_BLOCK_MAX);
			msg->len += ((msg->flags & I2C_CLIENT_PEC) ? 2 : 1);
			msg->flags &= ~I2C_M_RECV_LEN;
		}
		i2c_bus->master_xfer_cnt += xfer_len;
		dev_dbg(i2c_bus->dev, "master_xfer_cnt [%d/%d]\n", i2c_bus->master_xfer_cnt,
			msg->len);

		if (i2c_bus->master_xfer_cnt == msg->len) {
			if (i2c_bus->mode == DMA_MODE) {
				dma_unmap_single(i2c_bus->dev, i2c_bus->master_dma_addr, msg->len,
						 DMA_FROM_DEVICE);
				i2c_put_dma_safe_msg_buf(i2c_bus->master_safe_buf, msg, true);
				i2c_bus->master_safe_buf = NULL;
			}

			for (i = 0; i < msg->len; i++)
				dev_dbg(i2c_bus->dev, "M: r %d:[%x]\n", i, msg->buf[i]);
			i2c_bus->msgs_index++;
			if (i2c_bus->msgs_index == i2c_bus->msgs_count) {
				i2c_bus->cmd_err = i2c_bus->msgs_index;
				complete(&i2c_bus->cmd_complete);
			} else {
				if (ast2600_i2c_do_start(i2c_bus)) {
					i2c_bus->cmd_err = -ENOMEM;
					complete(&i2c_bus->cmd_complete);
				}
			}
		} else {
			/* next rx */
			cmd |= AST2600_I2CM_RX_CMD;
			if (i2c_bus->mode == DMA_MODE) {
				cmd |= AST2600_I2CM_RX_DMA_EN;
				xfer_len = msg->len - i2c_bus->master_xfer_cnt;
				if (xfer_len > AST2600_I2C_DMA_SIZE) {
					xfer_len = AST2600_I2C_DMA_SIZE;
				} else {
					if (i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) {
						dev_dbg(i2c_bus->dev, "last stop\n");
						cmd |= MASTER_TRIGGER_LAST_STOP;
					}
				}
				dev_dbg(i2c_bus->dev, "M: next rx len [%d/%d] , cmd %x\n", xfer_len,
					msg->len, cmd);
				writel(AST2600_I2CM_SET_RX_DMA_LEN(xfer_len - 1),
				       i2c_bus->reg_base + AST2600_I2CM_DMA_LEN);
				writel(i2c_bus->master_dma_addr + i2c_bus->master_xfer_cnt,
				       i2c_bus->reg_base + AST2600_I2CM_RX_DMA);
			} else if (i2c_bus->mode == BUFF_MODE) {
				cmd |= AST2600_I2CM_RX_BUFF_EN;
				xfer_len = msg->len - i2c_bus->master_xfer_cnt;
				if (xfer_len > i2c_bus->buf_size) {
					xfer_len = i2c_bus->buf_size;
				} else {
					if (i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) {
						dev_dbg(i2c_bus->dev, "last stop\n");
						cmd |= MASTER_TRIGGER_LAST_STOP;
					}
				}
				writel(AST2600_I2CC_SET_RX_BUF_LEN(xfer_len),
				       i2c_bus->reg_base + AST2600_I2CC_BUFF_CTRL);
			} else {
				if ((i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) &&
				    ((i2c_bus->master_xfer_cnt + 1) == msg->len)) {
					dev_dbg(i2c_bus->dev, "last stop\n");
					cmd |= MASTER_TRIGGER_LAST_STOP;
				}
			}
			dev_dbg(i2c_bus->dev, "M: next rx len %d, cmd %x\n", xfer_len, cmd);
			writel(cmd, i2c_bus->reg_base + AST2600_I2CM_CMD_STS);
		}
		break;
	default:
		dev_dbg(i2c_bus->dev, "todo care sts %x\n", sts);
		break;
	}
}

static int ast2600_i2c_master_irq(struct ast2600_i2c_bus *i2c_bus)
{
	u32 sts = readl(i2c_bus->reg_base + AST2600_I2CM_ISR);
	u32 ier = readl(i2c_bus->reg_base + AST2600_I2CM_IER);
	u32 ctrl;

	dev_dbg(i2c_bus->dev, "M sts %x\n", sts);
	if (!i2c_bus->alert_enable)
		sts &= ~AST2600_I2CM_SMBUS_ALT;

	if (AST2600_I2CM_BUS_RECOVER_FAIL & sts) {
		dev_dbg(i2c_bus->dev, "M clear isr: AST2600_I2CM_BUS_RECOVER_FAIL= %x\n", sts);
		writel(AST2600_I2CM_BUS_RECOVER_FAIL, i2c_bus->reg_base + AST2600_I2CM_ISR);
		ctrl = readl(i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);
		writel(0, i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);
		writel(ctrl, i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);
		i2c_bus->cmd_err = -EPROTO;
		complete(&i2c_bus->cmd_complete);
		return 1;
	}

	if (AST2600_I2CM_BUS_RECOVER & sts) {
		dev_dbg(i2c_bus->dev, "M clear isr: AST2600_I2CM_BUS_RECOVER= %x\n", sts);
		writel(AST2600_I2CM_BUS_RECOVER, i2c_bus->reg_base + AST2600_I2CM_ISR);
		i2c_bus->cmd_err = 0;
		complete(&i2c_bus->cmd_complete);
		return 1;
	}

	if (AST2600_I2CM_SMBUS_ALT & sts) {
		if (ier & AST2600_I2CM_SMBUS_ALT) {
			dev_dbg(i2c_bus->dev, "M clear isr: AST2600_I2CM_SMBUS_ALT= %x\n", sts);
			/* Disable ALT INT */
			writel(ier & ~AST2600_I2CM_SMBUS_ALT, i2c_bus->reg_base + AST2600_I2CM_IER);
			i2c_handle_smbus_alert(i2c_bus->ara);
			writel(AST2600_I2CM_SMBUS_ALT, i2c_bus->reg_base + AST2600_I2CM_ISR);
			dev_err(i2c_bus->dev,
				"ast2600_master_alert_recv bus id %d, Disable Alt, Please Imple\n",
				i2c_bus->adap.nr);
			return 1;
		}
	}

	i2c_bus->cmd_err = ast2600_i2c_irq_err_to_errno(sts);
	if (i2c_bus->cmd_err) {
		dev_dbg(i2c_bus->dev, "received error interrupt: 0x%02x\n", sts);
		writel(AST2600_I2CM_PKT_DONE, i2c_bus->reg_base + AST2600_I2CM_ISR);
		complete(&i2c_bus->cmd_complete);
		return 1;
	}

	if (AST2600_I2CM_PKT_DONE & sts) {
		if (i2c_bus->smbus_protocol)
			ast2600_i2c_smbus_package_irq(i2c_bus, sts);
		else
			ast2600_i2c_master_package_irq(i2c_bus, sts);
		return 1;
	}

	if (readl(i2c_bus->reg_base + AST2600_I2CM_ISR)) {
		dev_dbg(i2c_bus->dev, "master TODO care sts %x\n",
			readl(i2c_bus->reg_base + AST2600_I2CM_ISR));
		writel(readl(i2c_bus->reg_base + AST2600_I2CM_ISR),
				i2c_bus->reg_base + AST2600_I2CM_ISR);
	}

	return 0;
}

static irqreturn_t ast2600_i2c_bus_irq(int irq, void *dev_id)
{
	struct ast2600_i2c_bus *i2c_bus = dev_id;

#ifdef CONFIG_I2C_SLAVE
	if (readl(i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL) & AST2600_I2CC_SLAVE_EN) {
		if (ast2600_i2c_slave_irq(i2c_bus)) {
//			dev_dbg(i2c_bus->dev, "bus-%d.slave handle\n", i2c_bus->adap.nr);
			return IRQ_HANDLED;
		}
	}
#endif
	return ast2600_i2c_master_irq(i2c_bus) ? IRQ_HANDLED : IRQ_NONE;
}

static void ast2600_smbus_try_get_dmabuf(struct i2c_msg *msg, u8 init_val)
{
	bool is_read = msg->flags & I2C_M_RD;
	unsigned char *dma_buf;

	dma_buf = kzalloc(I2C_SMBUS_BLOCK_MAX + (is_read ? 2 : 3), GFP_KERNEL);
	if (!dma_buf)
		return;

	msg->buf = dma_buf;
	msg->flags |= I2C_M_DMA_SAFE;

	if (init_val)
		msg->buf[0] = init_val;
}

/* Assume a 7-bit address, which is reasonable for SMBus */
static u8 ast2600_smbus_msg_pec(u8 pec, struct i2c_msg *msg)
{
	/* The address will be sent first */
	u8 addr = i2c_8bit_addr_from_msg(msg);

	pec = i2c_smbus_pec(pec, &addr, 1);

	/* The data buffer follows */
	return i2c_smbus_pec(pec, msg->buf, msg->len);
}

/* Used for write only transactions */
static inline void ast2600_smbus_add_pec(struct i2c_msg *msg)
{
	msg->buf[msg->len] = ast2600_smbus_msg_pec(0, msg);
	msg->len++;
}

/* Return <0 on CRC error
 * If there was a write before this read (most cases) we need to take the
 * partial CRC from the write part into account.
 * Note that this function does modify the message (we need to decrease the
 * message length to hide the CRC byte from the caller).
 */
static int ast2600_smbus_check_pec(u8 cpec, struct i2c_msg *msg)
{
	u8 rpec = msg->buf[--msg->len];

	cpec = ast2600_smbus_msg_pec(cpec, msg);

	if (rpec != cpec) {
		pr_debug("Bad PEC 0x%02x vs. 0x%02x\n", rpec, cpec);
		return -EBADMSG;
	}
	return 0;
}

static int ast2600_i2c_smbus_xfer(struct i2c_adapter *adap, u16 addr,
				  unsigned short flags,
				  char read_write, u8 command, int size,
				  union i2c_smbus_data *data)
{
	struct ast2600_i2c_bus *i2c_bus = i2c_get_adapdata(adap);
	unsigned long timeout;
	unsigned char msgbuf0[I2C_SMBUS_BLOCK_MAX + 3];
	unsigned char msgbuf1[I2C_SMBUS_BLOCK_MAX + 2];
	int nmsgs = read_write == I2C_SMBUS_READ ? 2 : 1;
	u8 partial_pec = 0;
	int status;

	struct i2c_msg msg[2] = {
		{
			.addr = addr,
			.flags = flags,
			.len = 1,
			.buf = msgbuf0,
		}, {
			.addr = addr,
			.flags = flags | I2C_M_RD,
			.len = 0,
			.buf = msgbuf1,
		},
	};

	bool wants_pec = ((flags & I2C_CLIENT_PEC) &&
		size != I2C_SMBUS_QUICK &&
		size != I2C_SMBUS_I2C_BLOCK_DATA);

	msgbuf0[0] = command;
	switch (size) {
	case I2C_SMBUS_QUICK:
		msg[0].len = 0;
		/* Special case: The read/write field is used as data */
		msg[0].flags = flags | (read_write == I2C_SMBUS_READ ?
					I2C_M_RD : 0);
		nmsgs = 1;
		break;
	case I2C_SMBUS_BYTE:
		if (read_write == I2C_SMBUS_READ) {
			/* Special case: only a read! */
			msg[0].flags = I2C_M_RD | flags;
			nmsgs = 1;
		}
		break;
	case I2C_SMBUS_BYTE_DATA:
		if (read_write == I2C_SMBUS_READ) {
			msg[1].len = 1;
		} else {
			msg[0].len = 2;
			msgbuf0[1] = data->byte;
		}
		break;
	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_READ) {
			msg[1].len = 2;
		} else {
			msg[0].len = 3;
			msgbuf0[1] = data->word & 0xff;
			msgbuf0[2] = data->word >> 8;
		}
		break;
	case I2C_SMBUS_PROC_CALL:
		nmsgs = 2; /* Special case */
		read_write = I2C_SMBUS_READ;
		msg[0].len = 3;
		msg[1].len = 2;
		msgbuf0[1] = data->word & 0xff;
		msgbuf0[2] = data->word >> 8;
		break;
	case I2C_SMBUS_BLOCK_DATA:
		if (read_write == I2C_SMBUS_READ) {
			msg[1].flags |= I2C_M_RECV_LEN;
			msg[1].len = 1; /* block length will be added */
			ast2600_smbus_try_get_dmabuf(&msg[1], 0);
		} else {
			msg[0].len = data->block[0] + 2;
			if (msg[0].len > I2C_SMBUS_BLOCK_MAX + 2) {
				dev_err(&adap->dev,
					"Invalid block write size %d\n",
					data->block[0]);
				return -EINVAL;
			}

			ast2600_smbus_try_get_dmabuf(&msg[0], command);
			memcpy(msg[0].buf + 1, data->block, msg[0].len - 1);
		}
		break;
	case I2C_SMBUS_BLOCK_PROC_CALL:
		nmsgs = 2; /* Another special case */
		read_write = I2C_SMBUS_READ;
		if (data->block[0] > I2C_SMBUS_BLOCK_MAX) {
			dev_err(&adap->dev,
				"Invalid block write size %d\n",
				data->block[0]);
			return -EINVAL;
		}

		msg[0].len = data->block[0] + 2;
		ast2600_smbus_try_get_dmabuf(&msg[0], command);
		memcpy(msg[0].buf + 1, data->block, msg[0].len - 1);

		msg[1].flags |= I2C_M_RECV_LEN;
		msg[1].len = 1; /* block length will be added */
		ast2600_smbus_try_get_dmabuf(&msg[1], 0);
		break;
	case I2C_SMBUS_I2C_BLOCK_DATA:
		if (data->block[0] > I2C_SMBUS_BLOCK_MAX) {
			dev_err(&adap->dev, "Invalid block %s size %d\n",
				read_write == I2C_SMBUS_READ ? "read" : "write",
				data->block[0]);
			return -EINVAL;
		}

		if (read_write == I2C_SMBUS_READ) {
			msg[1].len = data->block[0];
			ast2600_smbus_try_get_dmabuf(&msg[1], 0);
		} else {
			msg[0].len = data->block[0] + 1;

			ast2600_smbus_try_get_dmabuf(&msg[0], command);
			memcpy(msg[0].buf + 1, data->block + 1, data->block[0]);
		}
		break;
	default:
		dev_err(&adap->dev, "Unsupported transaction %d\n", size);
		return -EOPNOTSUPP;
	}

	if (wants_pec) {
		/* Compute PEC if first message is a write */
		if (!(msg[0].flags & I2C_M_RD)) {
			if (nmsgs == 1) /* Write only */
				ast2600_smbus_add_pec(&msg[0]);
			else /* Write followed by read */
				partial_pec = ast2600_smbus_msg_pec(0, &msg[0]);
		}
		/* Ask for PEC if last message is a read */
		if (msg[nmsgs - 1].flags & I2C_M_RD)
			msg[nmsgs - 1].len++;
	}

	i2c_bus->cmd_err = 0;
	i2c_bus->msgs = msg;
	i2c_bus->msgs_index = 0;
	i2c_bus->msgs_count = nmsgs;
	i2c_bus->smbus_protocol = 1;
	reinit_completion(&i2c_bus->cmd_complete);
	status = ast2600_smbus_do_start(i2c_bus);
	timeout = wait_for_completion_timeout(&i2c_bus->cmd_complete, i2c_bus->adap.timeout);
	if (timeout == 0) {
		u32 ctrl = readl(i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);

		dev_dbg(i2c_bus->dev, "timeout isr[%x], sts[%x]\n",
			readl(i2c_bus->reg_base + AST2600_I2CM_ISR),
			readl(i2c_bus->reg_base + AST2600_I2CC_STS_AND_BUFF));

		writel(0, i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);
		writel(ctrl, i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);
#ifdef CONFIG_I2C_SLAVE
		if (ctrl & AST2600_I2CC_SLAVE_EN) {
			u32 cmd = SLAVE_TRIGGER_CMD;

			if (i2c_bus->mode == DMA_MODE) {
				cmd |= AST2600_I2CS_RX_DMA_EN;
				writel(i2c_bus->slave_dma_addr, i2c_bus->reg_base + AST2600_I2CS_RX_DMA);
				writel(i2c_bus->slave_dma_addr, i2c_bus->reg_base + AST2600_I2CS_TX_DMA);
				writel(AST2600_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_MSG_BUF_SIZE),
				       i2c_bus->reg_base + AST2600_I2CS_DMA_LEN);
			} else if (i2c_bus->mode == BUFF_MODE) {
				cmd = SLAVE_TRIGGER_CMD;
			} else {
				cmd &= ~AST2600_I2CS_PKT_MODE_EN;
			}
			dev_dbg(i2c_bus->dev, "slave trigger [%x]\n", cmd);
			writel(cmd, i2c_bus->reg_base + AST2600_I2CS_CMD_STS);
		}
#endif
		if (i2c_bus->multi_master &&
		    (readl(i2c_bus->reg_base + AST2600_I2CC_STS_AND_BUFF) &
		    AST2600_I2CC_BUS_BUSY_STS))
			ast2600_i2c_recover_bus(i2c_bus);

		status = -ETIMEDOUT;
	} else {
		status = i2c_bus->cmd_err;
	}

	if (status < 0)
		goto cleanup;
	if (status != nmsgs) {
		status = -EIO;
		goto cleanup;
	}

	status = 0;

	/* Check PEC if last message is a read */
	if (wants_pec && (msg[nmsgs - 1].flags & I2C_M_RD)) {
		status = ast2600_smbus_check_pec(partial_pec, &msg[nmsgs - 1]);
		if (status < 0)
			goto cleanup;
	}

	if (read_write == I2C_SMBUS_READ)
		switch (size) {
		case I2C_SMBUS_BYTE:
			data->byte = msgbuf0[0];
			break;
		case I2C_SMBUS_BYTE_DATA:
			data->byte = msgbuf1[0];
			break;
		case I2C_SMBUS_WORD_DATA:
		case I2C_SMBUS_PROC_CALL:
			data->word = msgbuf1[0] | (msgbuf1[1] << 8);
			break;
		case I2C_SMBUS_I2C_BLOCK_DATA:
			memcpy(data->block + 1, msg[1].buf, data->block[0]);
			break;
		case I2C_SMBUS_BLOCK_DATA:
		case I2C_SMBUS_BLOCK_PROC_CALL:
			if (msg[1].buf[0] > I2C_SMBUS_BLOCK_MAX) {
				dev_err(&adap->dev,
					"Invalid block size returned: %d\n",
					msg[1].buf[0]);
				status = -EPROTO;
				goto cleanup;
			}
			memcpy(data->block, msg[1].buf, msg[1].buf[0] + 1);
			break;
		}

cleanup:
	if (msg[0].flags & I2C_M_DMA_SAFE)
		kfree(msg[0].buf);
	if (msg[1].flags & I2C_M_DMA_SAFE)
		kfree(msg[1].buf);

	return status;
}

static int ast2600_i2c_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct ast2600_i2c_bus *i2c_bus = i2c_get_adapdata(adap);
	unsigned long timeout;
	int ret;

	/* If bus is busy in a single master environment, attempt recovery. */
	if (!i2c_bus->multi_master &&
	    (readl(i2c_bus->reg_base + AST2600_I2CC_STS_AND_BUFF) & AST2600_I2CC_BUS_BUSY_STS)) {
		ret = ast2600_i2c_recover_bus(i2c_bus);
		if (ret)
			return ret;
	}

#ifdef CONFIG_I2C_SLAVE
	if (i2c_bus->mode == BUFF_MODE) {
		if (i2c_bus->slave_operate)
			return -EBUSY;
		/* disable slave isr */
		writel(0, i2c_bus->reg_base + AST2600_I2CS_IER);
		if (readl(i2c_bus->reg_base + AST2600_I2CS_ISR) || i2c_bus->slave_operate) {
			writel(AST2600_I2CS_PKT_DONE, i2c_bus->reg_base + AST2600_I2CS_IER);
			return -EBUSY;
		}
	}
#endif

	i2c_bus->cmd_err = 0;
	i2c_bus->msgs = msgs;
	i2c_bus->msgs_index = 0;
	i2c_bus->msgs_count = num;
	i2c_bus->smbus_protocol = 0;
	reinit_completion(&i2c_bus->cmd_complete);
	ret = ast2600_i2c_do_start(i2c_bus);
#ifdef CONFIG_I2C_SLAVE
	/* avoid race condication slave is wait and master wait 1st slave operate */
	if (i2c_bus->mode == BUFF_MODE)
		writel(AST2600_I2CS_PKT_DONE, i2c_bus->reg_base + AST2600_I2CS_IER);
#endif
	if (ret)
		goto master_out;
	timeout = wait_for_completion_timeout(&i2c_bus->cmd_complete, i2c_bus->adap.timeout);
	if (timeout == 0) {
		u32 ctrl = readl(i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);

		dev_dbg(i2c_bus->dev, "timeout isr[%x], sts[%x]\n",
			readl(i2c_bus->reg_base + AST2600_I2CM_ISR),
			readl(i2c_bus->reg_base + AST2600_I2CC_STS_AND_BUFF));

		writel(0, i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);
		writel(ctrl, i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);
#ifdef CONFIG_I2C_SLAVE
		if (ctrl & AST2600_I2CC_SLAVE_EN) {
			u32 cmd = SLAVE_TRIGGER_CMD;

			if (i2c_bus->mode == DMA_MODE) {
				cmd |= AST2600_I2CS_RX_DMA_EN;
				writel(i2c_bus->slave_dma_addr, i2c_bus->reg_base + AST2600_I2CS_RX_DMA);
				writel(i2c_bus->slave_dma_addr, i2c_bus->reg_base + AST2600_I2CS_TX_DMA);
				writel(AST2600_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_MSG_BUF_SIZE),
				       i2c_bus->reg_base + AST2600_I2CS_DMA_LEN);
			} else if (i2c_bus->mode == BUFF_MODE) {
				cmd = SLAVE_TRIGGER_CMD;
			} else {
				cmd &= ~AST2600_I2CS_PKT_MODE_EN;
			}
			dev_dbg(i2c_bus->dev, "slave trigger [%x]\n", cmd);
			writel(cmd, i2c_bus->reg_base + AST2600_I2CS_CMD_STS);
		}
#endif
		if (i2c_bus->multi_master &&
		    (readl(i2c_bus->reg_base + AST2600_I2CC_STS_AND_BUFF) &
		    AST2600_I2CC_BUS_BUSY_STS))
			ast2600_i2c_recover_bus(i2c_bus);

		ret = -ETIMEDOUT;
	} else {
		ret = i2c_bus->cmd_err;
	}

	dev_dbg(i2c_bus->dev, "bus%d-m: %d end\n", i2c_bus->adap.nr, i2c_bus->cmd_err);

master_out:
	if (i2c_bus->mode == DMA_MODE) {
		kfree(i2c_bus->master_safe_buf);
	    i2c_bus->master_safe_buf = NULL;
	}

	return ret;
}

static void ast2600_i2c_init(struct ast2600_i2c_bus *i2c_bus)
{
	struct platform_device *pdev = to_platform_device(i2c_bus->dev);
	u32 fun_ctrl = AST2600_I2CC_BUS_AUTO_RELEASE | AST2600_I2CC_MASTER_EN;

	/* I2C Reset */
	writel(0, i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);

	i2c_bus->multi_master = device_property_read_bool(&pdev->dev, "multi-master");
	if (!i2c_bus->multi_master)
		fun_ctrl |= AST2600_I2CC_MULTI_MASTER_DIS;

	/* Enable Master Mode */
	writel(fun_ctrl, i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);
	/* disable slave address */
	writel(0, i2c_bus->reg_base + AST2600_I2CS_ADDR_CTRL);

	/* Set AC Timing */
	writel(ast2600_select_i2c_clock(i2c_bus), i2c_bus->reg_base + AST2600_I2CC_AC_TIMING);

	/* Clear Interrupt */
	writel(GENMASK(27, 0), i2c_bus->reg_base + AST2600_I2CM_ISR);

#ifdef CONFIG_I2C_SLAVE
	/* for memory buffer initial */
	if (i2c_bus->mode == DMA_MODE) {
		i2c_bus->slave_dma_buf =
			dmam_alloc_coherent(i2c_bus->dev, I2C_SLAVE_MSG_BUF_SIZE,
					    &i2c_bus->slave_dma_addr, GFP_KERNEL);
		if (!i2c_bus->slave_dma_buf)
			return;
	}

	writel(GENMASK(27, 0), i2c_bus->reg_base + AST2600_I2CS_ISR);

	if (i2c_bus->mode == BYTE_MODE) {
		writel(GENMASK(15, 0), i2c_bus->reg_base + AST2600_I2CS_IER);
	} else {
		/* Set interrupt generation of I2C slave controller */
		writel(AST2600_I2CS_PKT_DONE, i2c_bus->reg_base + AST2600_I2CS_IER);
	}
#endif
}

#ifdef CONFIG_I2C_SLAVE
static int ast2600_i2c_reg_slave(struct i2c_client *client)
{
	struct ast2600_i2c_bus *i2c_bus = i2c_get_adapdata(client->adapter);
	u32 cmd = SLAVE_TRIGGER_CMD;

	if (i2c_bus->slave)
		return -EINVAL;

	dev_dbg(i2c_bus->dev, "slave addr %x\n", client->addr);

	writel(0, i2c_bus->reg_base + AST2600_I2CS_ADDR_CTRL);
	writel(AST2600_I2CC_SLAVE_EN | readl(i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL),
	       i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);

	/* trigger rx buffer */
	if (i2c_bus->mode == DMA_MODE) {
		cmd |= AST2600_I2CS_RX_DMA_EN;
		writel(i2c_bus->slave_dma_addr, i2c_bus->reg_base + AST2600_I2CS_RX_DMA);
		writel(i2c_bus->slave_dma_addr, i2c_bus->reg_base + AST2600_I2CS_TX_DMA);
		writel(AST2600_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_MSG_BUF_SIZE),
		       i2c_bus->reg_base + AST2600_I2CS_DMA_LEN);
	} else if (i2c_bus->mode == BUFF_MODE) {
		cmd = SLAVE_TRIGGER_CMD;
	} else {
		cmd &= ~AST2600_I2CS_PKT_MODE_EN;
	}

	writel(cmd, i2c_bus->reg_base + AST2600_I2CS_CMD_STS);
	i2c_bus->slave = client;
	/* Set slave addr. */
	writel(client->addr | AST2600_I2CS_ADDR1_ENABLE,
	       i2c_bus->reg_base + AST2600_I2CS_ADDR_CTRL);

	return 0;
}

static int ast2600_i2c_unreg_slave(struct i2c_client *slave)
{
	struct ast2600_i2c_bus *i2c_bus = i2c_get_adapdata(slave->adapter);

	/* Turn off slave mode. */
	writel(~AST2600_I2CC_SLAVE_EN & readl(i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL),
	       i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);
	writel(readl(i2c_bus->reg_base + AST2600_I2CS_ADDR_CTRL) & ~AST2600_I2CS_ADDR1_MASK,
	       i2c_bus->reg_base + AST2600_I2CS_ADDR_CTRL);

	i2c_bus->slave = NULL;

	return 0;
}
#endif

static u32 ast2600_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL_ALL;
}

static struct i2c_algorithm i2c_ast2600_algorithm = {
	.master_xfer = ast2600_i2c_master_xfer,
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	.reg_slave = ast2600_i2c_reg_slave,
	.unreg_slave = ast2600_i2c_unreg_slave,
#endif
	.functionality = ast2600_i2c_functionality,
};

static const struct of_device_id ast2600_i2c_bus_of_table[] = {
	{
		.compatible = "aspeed,ast2600-i2c-bus",
	},
	{}
};
MODULE_DEVICE_TABLE(of, ast2600_i2c_bus_of_table);

static int ast2600_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ast2600_i2c_bus *i2c_bus;
	const struct of_device_id *match;
	struct resource *res;
	u32 global_ctrl;
	int ret;

	i2c_bus = devm_kzalloc(dev, sizeof(*i2c_bus), GFP_KERNEL);
	if (!i2c_bus)
		return -ENOMEM;

	i2c_bus->global_reg = syscon_regmap_lookup_by_compatible("aspeed,ast2600-i2c-global");
	if (IS_ERR(i2c_bus->global_reg)) {
		dev_err(&pdev->dev, "failed to find ast2600 i2c global regmap\n");
		ret = -ENOMEM;
		goto free_mem;
	}

	/* get global control register */
	regmap_read(i2c_bus->global_reg, AST2600_I2CG_CTRL, &global_ctrl);

	if (global_ctrl & AST2600_I2CG_CTRL_NEW_CLK_DIV)
		i2c_bus->clk_div_mode = 1;

	if (!(global_ctrl & AST2600_I2CG_CTRL_NEW_REG)) {
		ret = -ENOENT;
		/* this driver only supports new reg mode. */
		dev_err(&pdev->dev, "Expect I2CG0C[2] = 1 (new reg mode)\n");
		goto free_mem;
	}

	i2c_bus->mode = DMA_MODE;
	i2c_bus->slave_operate = 0;
	i2c_bus->dev = dev;

	if (of_property_read_bool(pdev->dev.of_node, "byte-mode"))
		i2c_bus->mode = BYTE_MODE;

	if (of_property_read_bool(pdev->dev.of_node, "buff-mode")) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (res && resource_size(res) >= 2)
			i2c_bus->buf_base = devm_ioremap_resource(&pdev->dev, res);

		if (!IS_ERR_OR_NULL(i2c_bus->buf_base))
			i2c_bus->buf_size = resource_size(res)/2;

		i2c_bus->mode = BUFF_MODE;
	}

	/* support smbus xfer if it's not byte mode */
	if (i2c_bus->mode != BYTE_MODE)
		i2c_ast2600_algorithm.smbus_xfer = ast2600_i2c_smbus_xfer;

	/*
	 * i2c timeout counter: use base clk4 1Mhz,
	 * per unit: 1/(1000/4096) = 4096us
	 */
	ret = device_property_read_u32(dev, "i2c-scl-clk-low-timeout-us", &i2c_bus->timeout);
	if (!ret)
		i2c_bus->timeout /= 4096;

	init_completion(&i2c_bus->cmd_complete);

	i2c_bus->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2c_bus->reg_base)) {
		ret = PTR_ERR(i2c_bus->reg_base);
		goto free_mem;
	}

	i2c_bus->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (i2c_bus->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -i2c_bus->irq;
		goto free_irq;
	}

	match = of_match_node(ast2600_i2c_bus_of_table, pdev->dev.of_node);
	if (!match) {
		ret = -ENOENT;
		goto free_irq;
	}

	platform_set_drvdata(pdev, i2c_bus);

	i2c_bus->clk = devm_clk_get(i2c_bus->dev, NULL);
	if (IS_ERR(i2c_bus->clk)) {
		dev_err(i2c_bus->dev, "no clock defined\n");
		ret = -ENODEV;
		goto free_irq;
	}
	i2c_bus->apb_clk = clk_get_rate(i2c_bus->clk);
	dev_dbg(i2c_bus->dev, "i2c_bus->apb_clk %d\n", i2c_bus->apb_clk);

	ret = of_property_read_u32(pdev->dev.of_node, "bus-frequency", &i2c_bus->bus_frequency);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not read bus-frequency property\n");
		i2c_bus->bus_frequency = 100000;
	}

	/* Initialize the I2C adapter */
	i2c_bus->adap.owner = THIS_MODULE;
	i2c_bus->adap.algo = &i2c_ast2600_algorithm;
	i2c_bus->adap.retries = 0;
	i2c_bus->adap.dev.parent = i2c_bus->dev;
	i2c_bus->adap.dev.of_node = pdev->dev.of_node;
	i2c_bus->adap.algo_data = i2c_bus;
	strscpy(i2c_bus->adap.name, pdev->name, sizeof(i2c_bus->adap.name));
	i2c_set_adapdata(&i2c_bus->adap, i2c_bus);

	ast2600_i2c_init(i2c_bus);

	ret = devm_request_irq(dev, i2c_bus->irq, ast2600_i2c_bus_irq, 0,
			       dev_name(dev), i2c_bus);
	if (ret < 0)
		goto unmap;

	i2c_bus->alert_enable = device_property_read_bool(dev, "smbus-alert");
	if (i2c_bus->alert_enable) {
		i2c_bus->ara = i2c_new_smbus_alert_device(&i2c_bus->adap, &i2c_bus->alert_data);
		if (!i2c_bus->ara)
			dev_warn(dev, "Failed to register ARA client\n");

		writel(AST2600_I2CM_PKT_DONE | AST2600_I2CM_BUS_RECOVER | AST2600_I2CM_SMBUS_ALT,
		       i2c_bus->reg_base + AST2600_I2CM_IER);
	} else {
		i2c_bus->alert_enable = false;
		/* Set interrupt generation of I2C master controller */
		writel(AST2600_I2CM_PKT_DONE | AST2600_I2CM_BUS_RECOVER,
		       i2c_bus->reg_base + AST2600_I2CM_IER);
	}

	ret = i2c_add_adapter(&i2c_bus->adap);
	if (ret < 0)
		goto unmap;

	dev_info(dev, "%s [%d]: adapter [%d khz] mode [%d]\n",
		 dev->of_node->name, i2c_bus->adap.nr, i2c_bus->bus_frequency / 1000,
		 i2c_bus->mode);

	return 0;

unmap:
	free_irq(i2c_bus->irq, i2c_bus);
free_irq:
	devm_iounmap(&pdev->dev, i2c_bus->reg_base);
free_mem:
	kfree(i2c_bus);
	return ret;
}

static int ast2600_i2c_remove(struct platform_device *pdev)
{
	struct ast2600_i2c_bus *i2c_bus = platform_get_drvdata(pdev);

	/* Disable everything. */
	writel(0, i2c_bus->reg_base + AST2600_I2CC_FUN_CTRL);
	writel(0, i2c_bus->reg_base + AST2600_I2CM_IER);

	devm_free_irq(&pdev->dev, i2c_bus->irq, i2c_bus);

	i2c_del_adapter(&i2c_bus->adap);

#ifdef CONFIG_I2C_SLAVE
	/* for memory buffer initial */
	if (i2c_bus->mode == DMA_MODE)
		dma_free_coherent(i2c_bus->dev, I2C_SLAVE_MSG_BUF_SIZE,
				i2c_bus->slave_dma_buf, i2c_bus->slave_dma_addr);
#endif

	return 0;
}

static struct platform_driver ast2600_i2c_bus_driver = {
	.probe = ast2600_i2c_probe,
	.remove = ast2600_i2c_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = ast2600_i2c_bus_of_table,
	},
};
module_platform_driver(ast2600_i2c_bus_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED AST2600 I2C Controller Driver");
MODULE_LICENSE("GPL");
