/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2023 Aspeed Technology Inc.
 */
#ifndef _AST2500_ESPI_H_
#define _AST2500_ESPI_H_

#include <linux/bits.h>
#include "aspeed-espi-comm.h"

/* registers */
#define ESPI_CTRL				0x000
#define   ESPI_CTRL_FLASH_TX_SW_RST		BIT(31)
#define   ESPI_CTRL_FLASH_RX_SW_RST		BIT(30)
#define   ESPI_CTRL_OOB_TX_SW_RST		BIT(29)
#define   ESPI_CTRL_OOB_RX_SW_RST		BIT(28)
#define   ESPI_CTRL_PERIF_NP_TX_SW_RST		BIT(27)
#define   ESPI_CTRL_PERIF_NP_RX_SW_RST		BIT(26)
#define   ESPI_CTRL_PERIF_PC_TX_SW_RST		BIT(25)
#define   ESPI_CTRL_PERIF_PC_RX_SW_RST		BIT(24)
#define   ESPI_CTRL_FLASH_TX_DMA_EN		BIT(23)
#define   ESPI_CTRL_FLASH_RX_DMA_EN		BIT(22)
#define   ESPI_CTRL_OOB_TX_DMA_EN		BIT(21)
#define   ESPI_CTRL_OOB_RX_DMA_EN		BIT(20)
#define   ESPI_CTRL_PERIF_NP_TX_DMA_EN		BIT(19)
#define   ESPI_CTRL_PERIF_PC_TX_DMA_EN		BIT(17)
#define   ESPI_CTRL_PERIF_PC_RX_DMA_EN		BIT(16)
#define   ESPI_CTRL_FLASH_SAFS_SW_MODE		BIT(10)
#define   ESPI_CTRL_VW_GPIO_SW			BIT(9)
#define   ESPI_CTRL_FLASH_SW_RDY		BIT(7)
#define   ESPI_CTRL_OOB_SW_RDY			BIT(4)
#define   ESPI_CTRL_VW_SW_RDY			BIT(3)
#define   ESPI_CTRL_PERIF_SW_RDY		BIT(1)
#define ESPI_STS				0x004
#define ESPI_INT_STS				0x008
#define   ESPI_INT_STS_RST_DEASSERT		BIT(31)
#define   ESPI_INT_STS_OOB_RX_TMOUT		BIT(23)
#define   ESPI_INT_STS_VW_SYSEVT1		BIT(22)
#define   ESPI_INT_STS_FLASH_TX_ERR		BIT(21)
#define   ESPI_INT_STS_OOB_TX_ERR		BIT(20)
#define   ESPI_INT_STS_FLASH_TX_ABT		BIT(19)
#define   ESPI_INT_STS_OOB_TX_ABT		BIT(18)
#define   ESPI_INT_STS_PERIF_NP_TX_ABT		BIT(17)
#define   ESPI_INT_STS_PERIF_PC_TX_ABT		BIT(16)
#define   ESPI_INT_STS_FLASH_RX_ABT		BIT(15)
#define   ESPI_INT_STS_OOB_RX_ABT		BIT(14)
#define   ESPI_INT_STS_PERIF_NP_RX_ABT		BIT(13)
#define   ESPI_INT_STS_PERIF_PC_RX_ABT		BIT(12)
#define   ESPI_INT_STS_PERIF_NP_TX_ERR		BIT(11)
#define   ESPI_INT_STS_PERIF_PC_TX_ERR		BIT(10)
#define   ESPI_INT_STS_VW_GPIO			BIT(9)
#define   ESPI_INT_STS_VW_SYSEVT		BIT(8)
#define   ESPI_INT_STS_FLASH_TX_CMPLT		BIT(7)
#define   ESPI_INT_STS_FLASH_RX_CMPLT		BIT(6)
#define   ESPI_INT_STS_OOB_TX_CMPLT		BIT(5)
#define   ESPI_INT_STS_OOB_RX_CMPLT		BIT(4)
#define   ESPI_INT_STS_PERIF_NP_TX_CMPLT	BIT(3)
#define   ESPI_INT_STS_PERIF_PC_TX_CMPLT	BIT(1)
#define   ESPI_INT_STS_PERIF_PC_RX_CMPLT	BIT(0)
#define ESPI_INT_EN				0x00c
#define   ESPI_INT_EN_RST_DEASSERT		BIT(31)
#define   ESPI_INT_EN_OOB_RX_TMOUT		BIT(23)
#define   ESPI_INT_EN_VW_SYSEVT1		BIT(22)
#define   ESPI_INT_EN_FLASH_TX_ERR		BIT(21)
#define   ESPI_INT_EN_OOB_TX_ERR		BIT(20)
#define   ESPI_INT_EN_FLASH_TX_ABT		BIT(19)
#define   ESPI_INT_EN_OOB_TX_ABT		BIT(18)
#define   ESPI_INT_EN_PERIF_NP_TX_ABT		BIT(17)
#define   ESPI_INT_EN_PERIF_PC_TX_ABT		BIT(16)
#define   ESPI_INT_EN_FLASH_RX_ABT		BIT(15)
#define   ESPI_INT_EN_OOB_RX_ABT		BIT(14)
#define   ESPI_INT_EN_PERIF_NP_RX_ABT		BIT(13)
#define   ESPI_INT_EN_PERIF_PC_RX_ABT		BIT(12)
#define   ESPI_INT_EN_PERIF_NP_TX_ERR		BIT(11)
#define   ESPI_INT_EN_PERIF_PC_TX_ERR		BIT(10)
#define   ESPI_INT_EN_VW_GPIO			BIT(9)
#define   ESPI_INT_EN_VW_SYSEVT			BIT(8)
#define   ESPI_INT_EN_FLASH_TX_CMPLT		BIT(7)
#define   ESPI_INT_EN_FLASH_RX_CMPLT		BIT(6)
#define   ESPI_INT_EN_OOB_TX_CMPLT		BIT(5)
#define   ESPI_INT_EN_OOB_RX_CMPLT		BIT(4)
#define   ESPI_INT_EN_PERIF_NP_TX_CMPLT		BIT(3)
#define   ESPI_INT_EN_PERIF_PC_TX_CMPLT		BIT(1)
#define   ESPI_INT_EN_PERIF_PC_RX_CMPLT		BIT(0)
#define ESPI_PERIF_PC_RX_DMA			0x010
#define ESPI_PERIF_PC_RX_CTRL			0x014
#define   ESPI_PERIF_PC_RX_CTRL_SERV_PEND	BIT(31)
#define   ESPI_PERIF_PC_RX_CTRL_LEN		GENMASK(23, 12)
#define   ESPI_PERIF_PC_RX_CTRL_TAG		GENMASK(11, 8)
#define   ESPI_PERIF_PC_RX_CTRL_CYC		GENMASK(7, 0)
#define ESPI_PERIF_PC_RX_DATA			0x018
#define ESPI_PERIF_PC_TX_DMA			0x020
#define ESPI_PERIF_PC_TX_CTRL			0x024
#define	  ESPI_PERIF_PC_TX_CTRL_TRIG_PEND	BIT(31)
#define	  ESPI_PERIF_PC_TX_CTRL_LEN		GENMASK(23, 12)
#define	  ESPI_PERIF_PC_TX_CTRL_TAG		GENMASK(11, 8)
#define	  ESPI_PERIF_PC_TX_CTRL_CYC		GENMASK(7, 0)
#define ESPI_PERIF_PC_TX_DATA			0x028
#define ESPI_PERIF_NP_TX_DMA			0x030
#define ESPI_PERIF_NP_TX_CTRL			0x034
#define   ESPI_PERIF_NP_TX_CTRL_TRIG_PEND	BIT(31)
#define	  ESPI_PERIF_NP_TX_CTRL_LEN		GENMASK(23, 12)
#define	  ESPI_PERIF_NP_TX_CTRL_TAG		GENMASK(11, 8)
#define	  ESPI_PERIF_NP_TX_CTRL_CYC		GENMASK(7, 0)
#define ESPI_PERIF_NP_TX_DATA			0x038
#define ESPI_OOB_RX_DMA				0x040
#define ESPI_OOB_RX_CTRL			0x044
#define	  ESPI_OOB_RX_CTRL_SERV_PEND		BIT(31)
#define	  ESPI_OOB_RX_CTRL_LEN			GENMASK(23, 12)
#define	  ESPI_OOB_RX_CTRL_TAG			GENMASK(11, 8)
#define	  ESPI_OOB_RX_CTRL_CYC			GENMASK(7, 0)
#define ESPI_OOB_RX_DATA			0x048
#define ESPI_OOB_TX_DMA				0x050
#define ESPI_OOB_TX_CTRL			0x054
#define	  ESPI_OOB_TX_CTRL_TRIG_PEND		BIT(31)
#define	  ESPI_OOB_TX_CTRL_LEN			GENMASK(23, 12)
#define	  ESPI_OOB_TX_CTRL_TAG			GENMASK(11, 8)
#define	  ESPI_OOB_TX_CTRL_CYC			GENMASK(7, 0)
#define ESPI_OOB_TX_DATA			0x058
#define ESPI_FLASH_RX_DMA			0x060
#define ESPI_FLASH_RX_CTRL			0x064
#define	  ESPI_FLASH_RX_CTRL_SERV_PEND		BIT(31)
#define	  ESPI_FLASH_RX_CTRL_LEN		GENMASK(23, 12)
#define	  ESPI_FLASH_RX_CTRL_TAG		GENMASK(11, 8)
#define	  ESPI_FLASH_RX_CTRL_CYC		GENMASK(7, 0)
#define ESPI_FLASH_RX_DATA			0x068
#define ESPI_FLASH_TX_DMA			0x070
#define ESPI_FLASH_TX_CTRL			0x074
#define	  ESPI_FLASH_TX_CTRL_TRIG_PEND		BIT(31)
#define	  ESPI_FLASH_TX_CTRL_LEN		GENMASK(23, 12)
#define	  ESPI_FLASH_TX_CTRL_TAG		GENMASK(11, 8)
#define	  ESPI_FLASH_TX_CTRL_CYC		GENMASK(7, 0)
#define ESPI_FLASH_TX_DATA			0x078
#define ESPI_PERIF_MCYC_SADDR			0x084
#define ESPI_PERIF_MCYC_TADDR			0x088
#define ESPI_PERIF_MCYC_MASK			0x08c
#define ESPI_FLASH_SAFS_TADDR			0x090
#define   ESPI_FLASH_SAFS_TADDR_BASE		GENMASK(31, 24)
#define   ESPI_FLASH_SAFS_TADDR_MASK		GENMASK(15, 8)
#define ESPI_VW_SYSEVT_INT_EN			0x094
#define   ESPI_VW_SYSEVT_INT_EN_HOST_RST_WARN	BIT(8)
#define   ESPI_VW_SYSEVT_INT_EN_OOB_RST_WARN	BIT(6)
#define ESPI_VW_SYSEVT				0x098
#define   ESPI_VW_SYSEVT_HOST_RST_ACK		BIT(27)
#define   ESPI_VW_SYSEVT_SLV_BOOT_STS		BIT(23)
#define   ESPI_VW_SYSEVT_SLV_BOOT_DONE		BIT(20)
#define   ESPI_VW_SYSEVT_OOB_RST_ACK		BIT(16)
#define   ESPI_VW_SYSEVT_HOST_RST_WARN		BIT(8)
#define   ESPI_VW_SYSEVT_OOB_RST_WARN		BIT(6)
#define ESPI_VW_GPIO_VAL			0x09c
#define ESPI_GEN_CAP_N_CONF			0x0a0
#define ESPI_CH0_CAP_N_CONF			0x0a4
#define ESPI_CH1_CAP_N_CONF			0x0a8
#define ESPI_CH2_CAP_N_CONF			0x0ac
#define ESPI_CH3_CAP_N_CONF			0x0b0
#define ESPI_CH3_CAP_N_CONF2			0x0b4
#define ESPI_VW_GPIO_DIR			0x0c0
#define ESPI_VW_GPIO_GRP			0x0c4
#define ESPI_VW_SYSEVT1_INT_EN			0x100
#define   ESPI_VW_SYSEVT1_INT_EN_SUSPEND_WARN	BIT(0)
#define ESPI_VW_SYSEVT1				0x104
#define   ESPI_VW_SYSEVT1_SUSPEND_ACK		BIT(20)
#define   ESPI_VW_SYSEVT1_SUSPEND_WARN		BIT(0)
#define ESPI_VW_SYSEVT_INT_T0			0x110
#define ESPI_VW_SYSEVT_INT_T1			0x114
#define ESPI_VW_SYSEVT_INT_T2			0x118
#define   ESPI_VW_SYSEVT_INT_T2_HOST_RST_WARN	BIT(8)
#define   ESPI_VW_SYSEVT_INT_T2_OOB_RST_WARN	BIT(6)
#define ESPI_VW_SYSEVT_INT_STS			0x11c
#define   ESPI_VW_SYSEVT_INT_STS_HOST_RST_WARN	BIT(8)
#define   ESPI_VW_SYSEVT_INT_STS_OOB_RST_WARN	BIT(6)
#define ESPI_VW_SYSEVT1_INT_T0			0x120
#define   ESPI_VW_SYSEVT1_INT_T0_SUSPEND_WARN	BIT(0)
#define ESPI_VW_SYSEVT1_INT_T1			0x124
#define ESPI_VW_SYSEVT1_INT_T2			0x128
#define ESPI_VW_SYSEVT1_INT_STS			0x12c
#define   ESPI_VW_SYSEVT1_INT_STS_SUSPEND_WARN	BIT(0)

/* collect ESPI_INT_EN bits for convenience */
#define ESPI_INT_EN_PERIF			\
	(ESPI_INT_EN_PERIF_NP_TX_ABT |		\
	 ESPI_INT_EN_PERIF_PC_TX_ABT |		\
	 ESPI_INT_EN_PERIF_NP_RX_ABT |		\
	 ESPI_INT_EN_PERIF_PC_RX_ABT |		\
	 ESPI_INT_EN_PERIF_NP_TX_ERR |		\
	 ESPI_INT_EN_PERIF_PC_TX_ERR |		\
	 ESPI_INT_EN_PERIF_NP_TX_CMPLT |	\
	 ESPI_INT_EN_PERIF_PC_TX_CMPLT |	\
	 ESPI_INT_EN_PERIF_PC_RX_CMPLT)

#define ESPI_INT_EN_VW			\
	(ESPI_INT_EN_VW_SYSEVT1 |	\
	 ESPI_INT_EN_VW_GPIO    |	\
	 ESPI_INT_EN_VW_SYSEVT)

#define ESPI_INT_EN_OOB		\
	(ESPI_INT_EN_OOB_RX_TMOUT |	\
	 ESPI_INT_EN_OOB_TX_ERR |	\
	 ESPI_INT_EN_OOB_TX_ABT |	\
	 ESPI_INT_EN_OOB_RX_ABT |	\
	 ESPI_INT_EN_OOB_TX_CMPLT |	\
	 ESPI_INT_EN_OOB_RX_CMPLT)

#define ESPI_INT_EN_FLASH		\
	(ESPI_INT_EN_FLASH_TX_ERR |	\
	 ESPI_INT_EN_FLASH_TX_ABT |	\
	 ESPI_INT_EN_FLASH_RX_ABT |	\
	 ESPI_INT_EN_FLASH_TX_CMPLT |	\
	 ESPI_INT_EN_FLASH_RX_CMPLT)

/* collect ESPI_INT_EN bits for convenience */
#define ESPI_INT_EN_PERIF			\
	(ESPI_INT_EN_PERIF_NP_TX_ABT |		\
	 ESPI_INT_EN_PERIF_PC_TX_ABT |		\
	 ESPI_INT_EN_PERIF_NP_RX_ABT |		\
	 ESPI_INT_EN_PERIF_PC_RX_ABT |		\
	 ESPI_INT_EN_PERIF_NP_TX_ERR |		\
	 ESPI_INT_EN_PERIF_PC_TX_ERR |		\
	 ESPI_INT_EN_PERIF_NP_TX_CMPLT |	\
	 ESPI_INT_EN_PERIF_PC_TX_CMPLT |	\
	 ESPI_INT_EN_PERIF_PC_RX_CMPLT)

#define ESPI_INT_EN_VW			\
	(ESPI_INT_EN_VW_SYSEVT1 |	\
	 ESPI_INT_EN_VW_GPIO    |	\
	 ESPI_INT_EN_VW_SYSEVT)

#define ESPI_INT_EN_OOB		\
	(ESPI_INT_EN_OOB_RX_TMOUT |	\
	 ESPI_INT_EN_OOB_TX_ERR |	\
	 ESPI_INT_EN_OOB_TX_ABT |	\
	 ESPI_INT_EN_OOB_RX_ABT |	\
	 ESPI_INT_EN_OOB_TX_CMPLT |	\
	 ESPI_INT_EN_OOB_RX_CMPLT)

#define ESPI_INT_EN_FLASH		\
	(ESPI_INT_EN_FLASH_TX_ERR |	\
	 ESPI_INT_EN_FLASH_TX_ABT |	\
	 ESPI_INT_EN_FLASH_RX_ABT |	\
	 ESPI_INT_EN_FLASH_TX_CMPLT |	\
	 ESPI_INT_EN_FLASH_RX_CMPLT)

/* collect ESPI_INT_STS bits for convenience */
#define ESPI_INT_STS_PERIF			\
	(ESPI_INT_STS_PERIF_NP_TX_ABT |		\
	 ESPI_INT_STS_PERIF_PC_TX_ABT |		\
	 ESPI_INT_STS_PERIF_NP_RX_ABT |		\
	 ESPI_INT_STS_PERIF_PC_RX_ABT |		\
	 ESPI_INT_STS_PERIF_NP_TX_ERR |		\
	 ESPI_INT_STS_PERIF_PC_TX_ERR |		\
	 ESPI_INT_STS_PERIF_NP_TX_CMPLT |	\
	 ESPI_INT_STS_PERIF_PC_TX_CMPLT |	\
	 ESPI_INT_STS_PERIF_PC_RX_CMPLT)

#define ESPI_INT_STS_VW			\
	(ESPI_INT_STS_VW_SYSEVT1 |	\
	 ESPI_INT_STS_VW_GPIO    |	\
	 ESPI_INT_STS_VW_SYSEVT)

#define ESPI_INT_STS_OOB		\
	(ESPI_INT_STS_OOB_RX_TMOUT |	\
	 ESPI_INT_STS_OOB_TX_ERR |	\
	 ESPI_INT_STS_OOB_TX_ABT |	\
	 ESPI_INT_STS_OOB_RX_ABT |	\
	 ESPI_INT_STS_OOB_TX_CMPLT |	\
	 ESPI_INT_STS_OOB_RX_CMPLT)

#define ESPI_INT_STS_FLASH		\
	(ESPI_INT_STS_FLASH_TX_ERR |	\
	 ESPI_INT_STS_FLASH_TX_ABT |	\
	 ESPI_INT_STS_FLASH_RX_ABT |	\
	 ESPI_INT_STS_FLASH_TX_CMPLT |	\
	 ESPI_INT_STS_FLASH_RX_CMPLT)

/* consistent with DTS property "flash-safs-mode" */
enum ast2500_safs_mode {
	SAFS_MODE_MIX = 0x0,
	SAFS_MODE_SW,
	SAFS_MODES,
};

#endif
