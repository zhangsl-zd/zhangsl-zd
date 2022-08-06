/* SPDX-License-Identifier: GPL-2.0 */
/* Phytium CAN controller driver
 *
 * Copyright (C) 2021 Phytium Technology Co., Ltd.
 */
#ifndef __PHYTIUM_CAN_H__
#define __PHYTIUM_CAN_H__

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/led.h>
#include <linux/atomic.h>

#define DRIVER_NAME "phytium_can"

/* CAN registers offset */
enum phytium_can_reg {
	FTCAN_CTRL_OFFSET           = 0x00,  /* Global control register */
	FTCAN_INTR_OFFSET           = 0x04,  /* Interrupt register */
	FTCAN_ARB_RATE_CTRL_OFFSET	= 0x08,  /* Arbitration rate control register */
	FTCAN_DAT_RATE_CTRL_OFFSET	= 0x0C,  /* Data rate control register */
	FTCAN_ACC_ID0_OFFSET        = 0x10,  /* Acceptance identifier0 register */
	FTCAN_ACC_ID1_OFFSET        = 0x14,  /* Acceptance identifier1 register */
	FTCAN_ACC_ID2_OFFSET        = 0x18,  /* Acceptance identifier2 register */
	FTCAN_ACC_ID3_OFFSET        = 0x1C,  /* Acceptance identifier3 register */
	FTCAN_ACC_ID0_MASK_OFFSET   = 0x20,  /* Acceptance identifier0 mask register */
	FTCAN_ACC_ID1_MASK_OFFSET   = 0x24,  /* Acceptance identifier1 mask register */
	FTCAN_ACC_ID2_MASK_OFFSET   = 0x28,  /* Acceptance identifier2 mask register */
	FTCAN_ACC_ID3_MASK_OFFSET   = 0x2C,  /* Acceptance identifier3 mask register */
	FTCAN_XFER_STS_OFFSET       = 0x30,  /* Transfer status register */
	FTCAN_ERR_CNT_OFFSET        = 0x34,  /* Error counter register */
	FTCAN_FIFO_CNT_OFFSET       = 0x38,  /* FIFO counter register */
	FTCAN_DMA_CTRL_OFFSET       = 0x3C,  /* DMA request control register */
	FTCAN_FRM_INFO_OFFSET       = 0x48,  /* Frame valid number register */
	FTCAN_TX_FIFO_OFFSET        = 0x100, /* TX FIFO shadow register */
	FTCAN_RX_FIFO_OFFSET        = 0x200, /* RX FIFO shadow register */
};

/*---------------------------------------------------------------------------*/
/* CAN register bit masks - FTCAN_<REG>_<BIT>_MASK                           */
/*---------------------------------------------------------------------------*/
/* FTCAN_CTRL mask */
#define FTCAN_CTRL_XFER_MASK   (0x1 << 0)  /*Transfer enable*/
#define FTCAN_CTRL_TXREQ_MASK  (0x1 << 1)  /*Transmit request*/
#define FTCAN_CTRL_AIME_MASK   (0x1 << 2)  /*Acceptance identifier mask enable*/
#define FTCAN_CTRL_RST_MASK   (0x1 << 7)  /* Soft rest 1:reset and auto clear */
#define FTCAN_CTRL_FDCRC_MASK (0x1 << 11)  /* Stuff count */
#define FTCAN_CTRL_IOF_MASK   (0x1 << 10)  /* send over frame */
/* FTCAN_INTR mask */
#define FTCAN_INTR_STATUS_MASK (0xFF << 0) /*the interrupt status*/
#define FTCAN_INTR_BOIS_MASK   (0x1 << 0)  /*Bus off interrupt status*/
#define FTCAN_INTR_PWIS_MASK   (0x1 << 1)  /*Passive warning interrupt status*/
#define FTCAN_INTR_PEIS_MASK   (0x1 << 2)  /*Passive error interrupt status*/
#define FTCAN_INTR_RFIS_MASK   (0x1 << 3)  /*RX FIFO full interrupt status*/
#define FTCAN_INTR_TFIS_MASK   (0x1 << 4)  /*TX FIFO empty interrupt status*/
#define FTCAN_INTR_REIS_MASK   (0x1 << 5)  /*RX frame end interrupt status*/
#define FTCAN_INTR_TEIS_MASK   (0x1 << 6)  /*TX frame end interrupt status*/
#define FTCAN_INTR_EIS_MASK    (0x1 << 7)  /*Error interrupt status*/

#define FTCAN_INTR_EN_MASK     (0xFF << 8) /*the interrupt enable*/
#define FTCAN_INTR_BOIE_MASK   (0x1 << 8)  /*Bus off interrupt enable*/
#define FTCAN_INTR_PWIE_MASK   (0x1 << 9)  /*Passive warning interrupt enable*/
#define FTCAN_INTR_PEIE_MASK   (0x1 << 10) /*Passive error interrupt enable*/
#define FTCAN_INTR_RFIE_MASK   (0x1 << 11) /*RX FIFO full interrupt enable*/
#define FTCAN_INTR_TFIE_MASK   (0x1 << 12) /*TX FIFO empty interrupt enable*/
#define FTCAN_INTR_REIE_MASK   (0x1 << 13) /*RX frame end interrupt enable*/
#define FTCAN_INTR_TEIE_MASK   (0x1 << 14) /*TX frame end interrupt enable*/
#define FTCAN_INTR_EIE_MASK    (0x1 << 15) /*Error interrupt enable*/

#define FTCAN_INTR_BOIC_MASK   (0x1 << 16) /*Bus off interrupt clear*/
#define FTCAN_INTR_PWIC_MASK   (0x1 << 17) /*Passive warning interrupt clear*/
#define FTCAN_INTR_PEIC_MASK   (0x1 << 18) /*Passive error interrupt clear*/
#define FTCAN_INTR_RFIC_MASK   (0x1 << 19) /*RX FIFO full interrupt clear*/
#define FTCAN_INTR_TFIC_MASK   (0x1 << 20) /*TX FIFO empty interrupt clear*/
#define FTCAN_INTR_REIC_MASK   (0x1 << 21) /*RX frame end interrupt clear*/
#define FTCAN_INTR_TEIC_MASK   (0x1 << 22) /*TX frame end interrupt clear*/
#define FTCAN_INTR_EIC_MASK    (0x1 << 23) /*Error interrupt clear*/

/* FTCAN_ACC_ID(0-3)_MASK mask */
#define FTCAN_ACC_IDN_MASK      0x1FFFFFFF /*don’t care the matching */

/* FTCAN_ERR_CNT_OFFSET mask */
#define FTCAN_ERR_CNT_RFN_MASK (0xFF << 0) /*Receive error counter*/
#define FTCAN_ERR_CNT_TFN_MASK (0xFF << 16)/*Transmit error counter*/

/* FTCAN_FIFO_CNT_OFFSET mask */
#define FTCAN_FIFO_CNT_RFN_MASK (0xFF << 0) /*Receive FIFO valid data number*/
#define FTCAN_FIFO_CNT_TFN_MASK (0xFF << 16)/*Transmit FIFO valid data number*/

#define FTCAN_ERR_CNT_TFN_SHIFT	  16  /* Tx Error Count shift */
#define FTCAN_FIFO_CNT_TFN_SHIFT  16  /* Tx FIFO Count shift*/
#define FTCAN_IDR_ID1_SHIFT       21  /* Standard Messg Identifier */
#define FTCAN_IDR_ID2_SHIFT       1   /* Extended Message Identifier */
#define FTCAN_IDR_SDLC_SHIFT      14
#define FTCAN_IDR_EDLC_SHIFT      26

#define FTCANFD_IDR1_SDLC_SHIFT     11
#define FTCANFD_IDR_SDLC_SHIFT      17
#define FTCANFD_IDR_EDLC_SHIFT      24
#define FTCANFD_IDR_GET_EDLC_SHIFT  12

#define FTCAN_IDR_ID2_MASK	0x0007FFFE /* Extended message ident */
#define FTCAN_IDR_ID1_MASK	0xFFE00000 /* Standard msg identifier */
#define FTCAN_IDR_IDE_MASK	0x00080000 /* Identifier extension */
#define FTCAN_IDR_SRR_MASK	0x00100000 /* Substitute remote TXreq */
#define FTCAN_IDR_RTR_MASK	0x00000001 /* Extended frames remote TX request */
#define FTCAN_IDR_DLC_MASK	0x0003C000 /* Standard msg dlc */
#define FTCAN_IDR_PAD_MASK	0x00003FFF /* Standard msg padding 1 */
#define FTCAN_ID2_EDLC_MASK	0x3C000000 /* Can Extended msg dlc */

#define FTCANFD_ID1_FDL_MASK	0x00040000 /* CANFD Standard FDL */
#define FTCANFD_ID1_BRS_MASK	0x00010000 /* CANFD Standard BRS */
#define FTCANFD_ID1_ESI_MASK	0x00008000 /* CANFD Standard ESI */
#define FTCANFD_ID1_SDLC_MASK	0x00007800 /* CANFD Standard msg dlc */

#define FTCANFD_ID2_FDL_MASK	0x80000000 /* CANFD Extended FDL */
#define FTCANFD_ID2_BRS_MASK	0x20000000 /* CANFD Extended BRS */
#define FTCANFD_ID2_ESI_MASK	0x10000000 /* CANFD Extended ESI */
#define FTCANFD_ID2_EDLC_MASK	0x0F000000 /* CANFD Extended msg dlc */
#define FTCANFD_IDR_PAD_MASK	0x000007FF /* CANFD Standard msg padding 1 */

#define FTCAN_INTR_EN (FTCAN_INTR_TEIE_MASK | FTCAN_INTR_REIE_MASK \
			| FTCAN_INTR_RFIE_MASK)

#define FTCAN_INTR_DIS      0x00000000
#define FTCAN_NAPI_WEIGHT	64

enum phytium_can_ip_type {
	PHYTIUM_CAN = 0,
	PHYTIUM_CANFD,
};

struct phytium_can_devtype {
	enum phytium_can_ip_type cantype;
	const struct can_bittiming_const *bittiming_const;
};

/**
 * struct phytium_can_priv - This definition define CAN driver instance
 * @can:		CAN private data structure.
 * @tx_head:		Tx CAN packets ready to send on the queue
 * @tx_tail:		Tx CAN packets successfully sended on the queue
 * @tx_max:		Maximum number packets the driver can send
 * @napi:		NAPI structure
 * @read_reg:		For reading data from CAN registers
 * @write_reg:		For writing data to CAN registers
 * @set_reg_bits:	For writing data to CAN registers bit
 * @clr_reg_bits:	For writing 0 to CAN registers bit
 * @dev:		Network device data structure
 * @reg_base:		Ioremapped address to registers
 * @irq_flags:		For request_irq()
 * @can_clk:		Pointer to struct clk
 * @lock:		The spin lock flag
 * @isr:		The interrupt status
 */
struct phytium_can_priv {
	struct can_priv can;
	unsigned int tx_head;
	unsigned int tx_tail;
	unsigned int tx_max;
	struct napi_struct napi;
	u32 (*read_reg)(const struct phytium_can_priv *priv,
			enum phytium_can_reg reg);
	void (*write_reg)(const struct phytium_can_priv *priv,
			  enum phytium_can_reg reg, u32 val);
	void (*set_reg_bits)(const struct phytium_can_priv *priv,
			     enum phytium_can_reg reg, u32 bs);
	void (*clr_reg_bits)(const struct phytium_can_priv *priv,
			     enum phytium_can_reg reg, u32 bs);
	struct device *dev;
	void __iomem *reg_base;
	unsigned long irq_flags;
	struct clk *can_clk;
	spinlock_t lock; /* spin lock */
	u32 isr;
	u32 quota;
	u32 fdmode;
};

int phytium_canfd_register(struct net_device *ndev);
int phytium_canfd_unregister(struct net_device *ndev);

#endif /*  __PHYTIUM_CAN_H__  */
