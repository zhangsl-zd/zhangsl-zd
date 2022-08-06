// SPDX-License-Identifier: GPL-2.0
/* CAN bus driver for Phytium CAN controller
 *
 * Copyright (C) 2021 Phytium Technology Co., Ltd.
 */

#include <linux/bitfield.h>

#include "phytium_can.h"

/* register definition */
enum phytium_can_reg {
	CAN_CTRL		= 0x00,		/* Global control register */
	CAN_INTR		= 0x04,		/* Interrupt register */
	CAN_ARB_RATE_CTRL	= 0x08,		/* Arbitration rate control register */
	CAN_DAT_RATE_CTRL	= 0x0c,		/* Data rate control register */
	CAN_ACC_ID0		= 0x10,		/* Acceptance identifier0 register */
	CAN_ACC_ID1		= 0x14,		/* Acceptance identifier1 register */
	CAN_ACC_ID2		= 0x18,		/* Acceptance identifier2 register */
	CAN_ACC_ID3		= 0x1c,		/* Acceptance identifier3 register */
	CAN_ACC_ID0_MASK	= 0x20,		/* Acceptance identifier0 mask register */
	CAN_ACC_ID1_MASK	= 0x24,		/* Acceptance identifier1 mask register */
	CAN_ACC_ID2_MASK	= 0x28,		/* Acceptance identifier2 mask register */
	CAN_ACC_ID3_MASK	= 0x2c,		/* Acceptance identifier3 mask register */
	CAN_XFER_STS		= 0x30,		/* Transfer status register */
	CAN_ERR_CNT		= 0x34,		/* Error counter register */
	CAN_FIFO_CNT		= 0x38,		/* FIFO counter register */
	CAN_DMA_CTRL		= 0x3c,		/* DMA request control register */
	CAN_XFER_EN		= 0x40,		/* Transfer enable register */
	CAN_INTR1		= 0x44,		/* Interrupt register 1 */
	CAN_FRM_INFO		= 0x48,		/* Frame valid number register */
	CAN_TIME_OUT		= 0x4c,		/* Timeout register */
	CAN_TIME_OUT_CNT	= 0x50,		/* Timeout counter register */
	CAN_INTR2		= 0x54,		/* Interrupt register 2 */
	CAN_TX_FIFO		= 0x100,	/* TX FIFO shadow register */
	CAN_RX_FIFO		= 0x200,	/* RX FIFO shadow register */
	CAN_RX_INFO_FIFO	= 0x300,	/* RX information FIFO shadow register */
	CAN_PIDR4		= 0xfd0,	/* Peripheral Identification Register 4 */
	CAN_PIDR0		= 0xfe0,	/* Peripheral Identification Register 0 */
	CAN_PIDR1		= 0xfe4,	/* Peripheral Identification Register 1 */
	CAN_PIDR2		= 0xfe8,	/* Peripheral Identification Register 2 */
	CAN_PIDR3		= 0xfec,	/* Peripheral Identification Register 3 */
	CAN_CIDR0		= 0xff0,	/* Component Identification Register 0 */
	CAN_CIDR1		= 0xff4,	/* Component Identification Register 1 */
	CAN_CIDR2		= 0xff8,	/* Component Identification Register 2 */
	CAN_CIDR3		= 0xffc,	/* Component Identification Register 3 */
};

/* Global control register (CTRL) */
#define CTRL_XFER		BIT(0)	/* Transfer enable */
#define CTRL_TXREQ		BIT(1)	/* Transmit request */
#define CTRL_AIME		BIT(2)	/* Acceptance identifier mask enable */
#define CTRL_TTS		BIT(3)	/* Transmit trigger strategy */
#define CTRL_RST		BIT(7)	/* Write 1 to soft reset and self clear */
#define CTRL_RFEIDF		BIT(8)	/* Allow RX frame end interrupt during ID filtered frame */
#define CTRL_RFEDT		BIT(9)	/* Allow RX frame end interrupt during TX frame */
#define CTRL_IOF		BIT(10)	/* Ignore overload flag internally */
#define CTRL_FDCRC		BIT(11)	/* CANFD CRC mode */

/* Interrupt register (INTR) */
#define INTR_BOIS	BIT(0)  /* Bus off interrupt status */
#define INTR_PWIS	BIT(1)  /* Passive warning interrupt status */
#define INTR_PEIS	BIT(2)  /* Passive error interrupt status */
#define INTR_RFIS	BIT(3)  /* RX FIFO full interrupt status */
#define INTR_TFIS	BIT(4)  /* TX FIFO empty interrupt status */
#define INTR_REIS	BIT(5)  /* RX frame end interrupt status */
#define INTR_TEIS	BIT(6)  /* TX frame end interrupt status */
#define INTR_EIS	BIT(7)  /* Error interrupt status */
#define INTR_BOIE	BIT(8)  /* Bus off interrupt enable */
#define INTR_PWIE	BIT(9)  /* Passive warning interrupt enable */
#define INTR_PEIE	BIT(10) /* Passive error interrupt enable */
#define INTR_RFIE	BIT(11) /* RX FIFO full interrupt enable */
#define INTR_TFIE	BIT(12) /* TX FIFO empty interrupt enable */
#define INTR_REIE	BIT(13) /* RX frame end interrupt enable */
#define INTR_TEIE	BIT(14) /* TX frame end interrupt enable */
#define INTR_EIE	BIT(15) /* Error interrupt enable */
#define INTR_BOIC	BIT(16) /* Bus off interrupt clear */
#define INTR_PWIC	BIT(17) /* Passive warning interrupt clear */
#define INTR_PEIC	BIT(18) /* Passive error interrupt clear */
#define INTR_RFIC	BIT(19) /* RX FIFO full interrupt clear */
#define INTR_TFIC	BIT(20) /* TX FIFO empty interrupt clear */
#define INTR_REIC	BIT(21) /* RX frame end interrupt clear */
#define INTR_TEIC	BIT(22) /* TX frame end interrupt clear */
#define INTR_EIC	BIT(23) /* Error interrupt clear */

#define INTR_STATUS_MASK (INTR_BOIS | INTR_PWIS | INTR_PEIS | INTR_RFIS | \
			  INTR_TFIS | INTR_REIS | INTR_TEIS | INTR_EIS)
#define INTR_EN_MASK	 (INTR_BOIE | INTR_PWIE | INTR_PEIE | INTR_RFIE | \
			  INTR_REIE | INTR_TEIE | INTR_EIE)
#define INTR_CLEAR_MASK	 (INTR_BOIC | INTR_PWIC | INTR_PEIC | INTR_RFIC | \
			  INTR_TFIC | INTR_REIC | INTR_TEIC | INTR_EIC)

/* Arbitration rate control register (ARB_RATE_CTRL) */
#define ARB_RATE_CTRL_ARJW	GENMASK(1, 0)	/* Arbitration field resync jump width */
#define ARB_RATE_CTRL_APRS	GENMASK(4, 2)	/* Arbitration field propagation segment */
#define ARB_RATE_CTRL_APH1S	GENMASK(7, 5)	/* Arbitration field phase1 segment */
#define ARB_RATE_CTRL_APH2S	GENMASK(10, 8)	/* Arbitration field phase2 segment */
#define ARB_RATE_CTRL_APD	GENMASK(28, 16)	/* Arbitration field prescaler divider */

/* Data rate control register (DAT_RATE_CTRL) */
#define DAT_RATE_CTRL_DRJW	GENMASK(1, 0)	/* Data field resync jump width */
#define DAT_RATE_CTRL_DPRS	GENMASK(4, 2)	/* Data field propagation segment */
#define DAT_RATE_CTRL_DPH1S	GENMASK(7, 5)	/* Data field phase1 segment */
#define DAT_RATE_CTRL_DPH2S	GENMASK(10, 8)	/* Data field phase2 segment */
#define DAT_RATE_CTRL_DPD	GENMASK(28, 16)	/* Data field prescaler divider */

/* Acceptance identifierX register (ACC_IDX) */
#define ACC_IDX_AID_MASK	GENMASK(28, 0)	/* Acceptance identifier */

/* Acceptance identifier0 mask register (ACC_ID0_MASK) */
#define ACC_IDX_MASK_AID_MASK	GENMASK(28, 0)	/* Acceptance identifier mask */

/* Transfer status register (XFER_STS) */
#define XFER_STS_FRAS		GENMASK(2, 0)	/* Frame status */
#define XFER_STS_FIES		GENMASK(7, 3)	/* Field status */
#define  XFER_STS_FIES_IDLE		(0x0)	/* idle */
#define  XFER_STS_FIES_ARBITRATION	(0x1)	/* arbitration */
#define  XFER_STS_FIES_TX_CTRL		(0x2)	/* transmit control */
#define  XFER_STS_FIES_TX_DATA		(0x3)	/* transmit data */
#define  XFER_STS_FIES_TX_CRC		(0x4)	/* transmit crc */
#define  XFER_STS_FIES_TX_FRM		(0x5)	/* transmit frame */
#define  XFER_STS_FIES_RX_CTRL		(0x6)	/* receive control */
#define  XFER_STS_FIES_RX_DATA		(0x7)	/* receive data */
#define  XFER_STS_FIES_RX_CRC		(0x8)	/* receive crc */
#define  XFER_STS_FIES_RX_FRM		(0x9)	/* receive frame */
#define  XFER_STS_FIES_INTERMISSION	(0xa)	/* intermission */
#define  XFER_STS_FIES_TX_SUSPD		(0xb)	/* transmit suspend */
#define  XFER_STS_FIES_BUS_IDLE		(0xc)	/* bus idle */
#define  XFER_STS_FIES_OVL_FLAG		(0xd)	/* overload flag */
#define  XFER_STS_FIES_OVL_DLM		(0xe)	/* overload delimiter */
#define  XFER_STS_FIES_ERR_FLAG		(0xf)	/* error flag */
#define  XFER_STS_FIES_ERR_DLM		(0x10)	/* error delimiter */
#define  XFER_STS_FIES_BUS_OFF		(0x11)	/* bus off */
#define XFER_STS_TS			BIT(8)	/* Transmit status */
#define XFER_STS_RS			BIT(9)	/* Receive status */
#define XFER_STS_XFERS			BIT(10)	/* Transfer status */

/* Error counter register (ERR_CNT) */
#define ERR_CNT_REC		GENMASK(8, 0)	/* Receive error counter */
#define ERR_CNT_TEC		GENMASK(24, 16)	/* Transmit error counter */

/* FIFO counter register (FIFO_CNT) */
#define FIFO_CNT_RFN		GENMASK(6, 0)	/* Receive FIFO valid data number */
#define FIFO_CNT_TFN		GENMASK(22, 16)	/* Transmit FIFO valid data number */

/* DMA request control register (DMA_CTRL) */
#define DMA_CTRL_RFTH		GENMASK(5, 0)	/* Receive FIFO DMA request threshold */
#define DMA_CTRL_RFRE		BIT(6)		/* Receive FIFO DMA request enable */
#define DMA_CTRL_TFTH		GENMASK(21, 16)	/* Transmit FIFO DMA request threshold */
#define DMA_CTRL_TFRE		BIT(22)		/* Transmit FIFO DMA request enable */

/* Transfer enable register (XFER_EN) */
#define XFER_EN_XFER		BIT(0)		/* Transfer enable */

/* Interrupt register 1 (INTR1) */
#define INTR1_RF1IS	BIT(0)	/* RX FIFO 1/4 interrupt status */
#define INTR1_RF2IS	BIT(1)	/* RX FIFO 1/2 interrupt status */
#define INTR1_RF3IS	BIT(2)	/* RX FIFO 3/4 interrupt status */
#define INTR1_RF4IS	BIT(3)	/* RX FIFO full interrupt status */
#define INTR1_TF1IS	BIT(4)	/* TX FIFO 1/4 interrupt status */
#define INTR1_TF2IS	BIT(5)	/* TX FIFO 1/2 interrupt status */
#define INTR1_TF3IS	BIT(6)	/* TX FIFO 3/4 interrupt status */
#define INTR1_TF4IS	BIT(7)	/* TX FIFO full interrupt status */
#define INTR1_RF1IE	BIT(8)	/* RX FIFO 1/4 interrupt enable */
#define INTR1_RF2IE	BIT(9)	/* RX FIFO 1/2 interrupt enable */
#define INTR1_RF3IE	BIT(10)	/* RX FIFO 3/4 interrupt enable */
#define INTR1_RF4IE	BIT(11)	/* RX FIFO full interrupt enable */
#define INTR1_TF1IE	BIT(12)	/* TX FIFO 1/4 interrupt enable */
#define INTR1_TF2IE	BIT(13)	/* TX FIFO 1/2 interrupt enable */
#define INTR1_TF3IE	BIT(14)	/* TX FIFO 3/4 interrupt enable */
#define INTR1_TF4IE	BIT(15)	/* TX FIFO full interrupt enable */
#define INTR1_RF1IC	BIT(16)	/* RX FIFO 1/4 interrupt clear */
#define INTR1_RF2IC	BIT(17)	/* RX FIFO 1/2 interrupt clear */
#define INTR1_RF3IC	BIT(18)	/* RX FIFO 3/4 interrupt clear */
#define INTR1_RF4IC	BIT(19)	/* RX FIFO full interrupt clear */
#define INTR1_TF1IC	BIT(20)	/* TX FIFO 1/4 interrupt clear */
#define INTR1_TF2IC	BIT(21)	/* TX FIFO 1/2 interrupt clear */
#define INTR1_TF3IC	BIT(22)	/* TX FIFO 3/4 interrupt clear */
#define INTR1_TF4IC	BIT(23)	/* TX FIFO full interrupt clear */
#define INTR1_RF1RIS	BIT(24)	/* RX FIFO 1/4 raw interrupt status */
#define INTR1_RF2RIS	BIT(25)	/* RX FIFO 1/2 raw interrupt status */
#define INTR1_RF3RIS	BIT(26)	/* RX FIFO 3/4 raw interrupt status */
#define INTR1_RF4RIS	BIT(27)	/* RX FIFO full raw interrupt status */
#define INTR1_TF1RIS	BIT(28)	/* TX FIFO 1/4 raw interrupt status */
#define INTR1_TF2RIS	BIT(29)	/* TX FIFO 1/2 raw interrupt status */
#define INTR1_TF3RIS	BIT(30)	/* TX FIFO 3/4 raw interrupt status */
#define INTR1_TF4RIS	BIT(31)	/* TX FIFO full raw interrupt status */

/* Frame valid number register (FRM_INFO) */
#define FRM_INFO_RXFC	GENMASK(5, 0)	/* Valid frame number in RX FIFO */
#define FRM_INFO_SSPD	GENMASK(31, 16)	/* Secondary sample point delay */

/* Interrupt register 2 (INTR2) */
#define INTR2_TOIS	BIT(0)	/* RX FIFO time out interrupt status */
#define INTR2_TOIM	BIT(8)	/* RX FIFO time out interrupt mask */
#define INTR2_TOIC	BIT(16)	/* RX FIFO time out interrupt clear */
#define INTR2_TORIS	BIT(24)	/* RX FIFO time out raw interrupt status */

/* RX information FIFO shadow register (RX_INFO_FIFO) */
#define RX_INFO_FIFO_WNORF	GENMASK(4, 0)	/* Word (4-byte) number of current receive frame */
#define RX_INFO_FIFO_RORF	BIT(5)		/* RTR value of current receive frame */
#define RX_INFO_FIFO_FORF	BIT(6)		/* FDF value of current receive frame */
#define RX_INFO_FIFO_IORF	BIT(7)		/* IDE value of current receive frame */

/* Arbitration Bits */
#define CAN_ID1_MASK		GENMASK(10, 0)	/* Base identifer */
#define CAN_RTR_MASK		BIT(11)		/* Remote Transmission Request */
#define CAN_IDE_MASK		BIT(12)		/* IDentifier extension flag */
#define CAN_ID2_MASK		GENMASK(30, 13)	/* Identifier extension */
#define CAN_ID2_RTR_MASK	BIT(31)

#define CAN_ID1_EDL_MASK	BIT(13)
#define CAN_ID1_DLC_MASK	GENMASK(17, 14)
#define CANFD_ID1_BRS_MASK	BIT(15)
#define CANFD_ID1_ESI_MASK	BIT(16)
#define CANFD_ID1_DLC_MASK	GENMASK(20, 17)

#define CAN_ID2_EDL_MASK	BIT(0)
#define CAN_ID2_DLC_MASK	GENMASK(5, 2)
#define CANFD_ID2_BRS_MASK	BIT(2)
#define CANFD_ID2_ESI_MASK	BIT(3)
#define CANFD_ID2_DLC_MASK	GENMASK(7, 4)

#define CAN_ID1_DLC_OFF		14
#define CANFD_ID1_DLC_OFF	17
#define CAN_ID2_DLC_OFF		2
#define CANFD_ID2_DLC_OFF	4

static inline u32 phytium_can_read(const struct phytium_can_dev *cdev, enum phytium_can_reg reg)
{
	return readl(cdev->base + reg);
}

static inline void phytium_can_write(const struct phytium_can_dev *cdev, enum phytium_can_reg reg,
			      u32 val)
{
	writel(val, cdev->base + reg);
}

static inline void phytium_can_enable_all_interrupts(struct phytium_can_dev *cdev)
{
	phytium_can_write(cdev, CAN_INTR, INTR_EN_MASK);
}

static inline void phytium_can_disable_all_interrupt(struct phytium_can_dev *cdev)
{
	phytium_can_write(cdev, CAN_INTR, 0x0);
}

static int phytium_can_get_berr_counter(const struct net_device *dev,
					struct can_berr_counter *bec)
{
	struct phytium_can_dev *cdev = netdev_priv(dev);

	bec->rxerr = phytium_can_read(cdev, CAN_ERR_CNT) & ERR_CNT_REC;
	bec->txerr = (phytium_can_read(cdev, CAN_ERR_CNT) & ERR_CNT_TEC) >> 16;

	return 0;
}

static int phytium_can_state_change(struct net_device *dev, enum can_state new_state)
{
	struct phytium_can_dev *cdev = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	struct can_berr_counter bec;

	switch (new_state) {
	case CAN_STATE_BUS_OFF:
		cdev->can.state = CAN_STATE_BUS_OFF;
		phytium_can_disable_all_interrupt(cdev);
		cdev->can.can_stats.bus_off++;
		can_bus_off(dev);
		break;
	case CAN_STATE_ERROR_PASSIVE:
		cdev->can.state = CAN_STATE_ERROR_PASSIVE;
		cdev->can.can_stats.error_passive++;
		break;
	case CAN_STATE_ERROR_WARNING:
		cdev->can.state = CAN_STATE_ERROR_WARNING;
		cdev->can.can_stats.error_warning++;
		break;
	default:
		break;
	}

	/* propagate the error condition to the CAN stack */
	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	phytium_can_get_berr_counter(dev, &bec);

	switch (new_state) {
	case CAN_STATE_BUS_OFF:
		cf->can_id |= CAN_ERR_BUSOFF;
		break;
	case CAN_STATE_ERROR_PASSIVE:
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = (bec.rxerr > 127) ?
			      CAN_ERR_CRTL_RX_PASSIVE :
			      CAN_ERR_CRTL_TX_PASSIVE;
		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;
		break;
	case CAN_STATE_ERROR_WARNING:
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = (bec.txerr > bec.rxerr) ?
			      CAN_ERR_CRTL_TX_PASSIVE :
			      CAN_ERR_CRTL_RX_PASSIVE;
		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;
		break;
	default:
		break;
	}

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_receive_skb(skb);

	return 1;
}

static int phytium_can_handle_state_errors(struct net_device *dev, u32 isr)
{
	struct phytium_can_dev *cdev = netdev_priv(dev);
	int work_done = 0;

	/* Handle states changes */
	if ((isr & INTR_PWIS) && cdev->can.state != CAN_STATE_ERROR_WARNING) {
		netdev_dbg(dev, "entered error warning state\n");
		work_done += phytium_can_state_change(dev, CAN_STATE_ERROR_WARNING);
	}

	if ((isr & INTR_PEIS) && cdev->can.state != CAN_STATE_ERROR_PASSIVE) {
		netdev_dbg(dev, "entered error passive state\n");
		work_done += phytium_can_state_change(dev, CAN_STATE_ERROR_PASSIVE);
	}

	if ((isr & INTR_BOIS) && cdev->can.state != CAN_STATE_BUS_OFF) {
		netdev_dbg(dev, "entered error bus off state\n");
		work_done += phytium_can_state_change(dev, CAN_STATE_BUS_OFF);
	}

	return work_done;
}

static int phytium_can_handle_lost_msg(struct net_device *dev)
{
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;

	netdev_err(dev, "message lost in RX FIFO\n");

	stats->rx_errors++;
	stats->rx_over_errors++;

	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	cf->can_id |= CAN_ERR_CRTL;
	cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;

	netif_receive_skb(skb);

	return 1;
}

static int phytium_can_read_fifo(struct net_device *dev)
{
	struct net_device_stats *stats = &dev->stats;
	struct phytium_can_dev *cdev = netdev_priv(dev);
	struct canfd_frame *cf;
	struct sk_buff *skb;
	u32 id, dlc, i;

	/* Read the frame header from FIFO */
	id = phytium_can_read(cdev, CAN_RX_FIFO);
	if (id & CAN_IDE_MASK) {
		/* Received an extended frame */
		dlc = cpu_to_be32(phytium_can_read(cdev, CAN_RX_FIFO));

		if (dlc & CAN_ID2_EDL_MASK)
			skb = alloc_canfd_skb(dev, &cf);
		else
			skb = alloc_can_skb(dev, (struct can_frame **)&cf);

		if (unlikely(!skb)) {
			stats->rx_dropped++;
			return 0;
		}

		if (dlc & CAN_ID2_EDL_MASK) {
			/* CAN FD extended frame */
			if (dlc & CANFD_ID2_BRS_MASK)
				cf->flags |= CANFD_BRS;
			if (dlc & CANFD_ID2_ESI_MASK)
				cf->flags |= CANFD_ESI;
			cf->len = can_dlc2len((dlc & CANFD_ID2_DLC_MASK) >> CANFD_ID2_DLC_OFF);
		} else {
			/* CAN extended frame */
			cf->len = get_can_dlc((dlc & CAN_ID2_DLC_MASK) >> CAN_ID2_DLC_OFF);
		}

		cf->can_id = (id & CAN_ID1_MASK) << 18;
		cf->can_id |= (id & CAN_ID2_MASK) >> 13;
		cf->can_id |= CAN_EFF_FLAG;

		if (id & CAN_ID2_RTR_MASK)
			cf->can_id |= CAN_RTR_FLAG;
	} else {
		/* Received a standard frame */
		if (id & CAN_ID1_EDL_MASK)
			skb = alloc_canfd_skb(dev, &cf);
		else
			skb = alloc_can_skb(dev, (struct can_frame **)&cf);

		if (unlikely(!skb)) {
			stats->rx_dropped++;
			return 0;
		}

		if (id & CAN_ID1_EDL_MASK) {
			/* CAN FD extended frame */
			if (id & CANFD_ID1_BRS_MASK)
				cf->flags |= CANFD_BRS;
			if (id & CANFD_ID1_ESI_MASK)
				cf->flags |= CANFD_ESI;
			cf->len = can_dlc2len((id & CANFD_ID1_DLC_MASK) >> CANFD_ID1_DLC_OFF);
		} else {
			/* CAN extended frame */
			cf->len = get_can_dlc((id & CAN_ID1_DLC_MASK) >> CAN_ID1_DLC_OFF);
		}

		if (id & CAN_RTR_MASK)
			cf->can_id |= CAN_RTR_FLAG;
	}

	if (!(cf->can_id & CAN_RTR_FLAG))
		/* Receive data frames */
		for (i = 0; i < cf->len; i += 4)
			*(__be32 *)(cf->data + i) = phytium_can_read(cdev, CAN_RX_FIFO);

	stats->rx_packets++;
	stats->rx_bytes += cf->len;
	netif_receive_skb(skb);

	return 0;
}

static int phytium_can_do_rx_poll(struct net_device *dev, int quota)
{
	struct phytium_can_dev *cdev = netdev_priv(dev);
	u32 rxfs, pkts = 0;
	int err;

	rxfs = phytium_can_read(cdev, CAN_FIFO_CNT) & FIFO_CNT_RFN;
	if (!rxfs) {
		netdev_dbg(dev, "no messages in RX FIFO\n");
		return 0;
	}

	while ((rxfs > 0) && (quota > 0)) {
		err = phytium_can_read_fifo(dev);
		if (err)
			return err;

		quota--;
		pkts++;
		rxfs = phytium_can_read(cdev, CAN_FIFO_CNT) & FIFO_CNT_RFN;
	}

	if (pkts)
		can_led_event(dev, CAN_LED_EVENT_RX);

	return pkts;
}

static int phytium_can_rx_handler(struct net_device *dev, int quota)
{
	struct phytium_can_dev *cdev = netdev_priv(dev);
	int work_done = 0;
	u32 isr;

	isr = cdev->isr | phytium_can_read(cdev, CAN_INTR);
	if (!isr)
		goto end;

	/* Handle error states IRQs */
	if (isr & (INTR_BOIS | INTR_PWIS | INTR_PEIS))
		work_done += phytium_can_handle_state_errors(dev, isr);

	if (isr & INTR_EIS)
		netdev_err(dev, "Detect error interrupt.\n");

	/* Handle RX FIFO full (overflow) IRQ */
	if (isr & INTR_RFIS)
		work_done += phytium_can_handle_lost_msg(dev);

	/* Handle RX IRQ */
	if (isr & INTR_REIS) {
		int rx_work_or_err;

		rx_work_or_err = phytium_can_do_rx_poll(dev, (quota - work_done));
		if (rx_work_or_err < 0)
			return rx_work_or_err;

		work_done += rx_work_or_err;
	}

end:
	return work_done;
}

static int phytium_can_poll(struct napi_struct *napi, int quota)
{
	struct net_device *dev = napi->dev;
	struct phytium_can_dev *cdev = netdev_priv(dev);
	int work_done;

	work_done = phytium_can_rx_handler(dev, quota);

	/* Don't re-enable interrupts if the driver had a fatal error
	 * (e.g., FIFO read failure)
	 */
	if (work_done >= 0 && work_done < quota) {
		napi_complete_done(napi, work_done);
		phytium_can_enable_all_interrupts(cdev);
	}

	return work_done;
}

static void phytium_can_write_frame(struct phytium_can_dev *cdev)
{
	struct canfd_frame *cf = (struct canfd_frame *)cdev->tx_skb->data;
	struct net_device *dev = cdev->net;
	struct sk_buff *skb = cdev->tx_skb;
	u32 i, id, dlc = 0;

	cdev->tx_skb = NULL;

	if (cf->can_id & CAN_EFF_FLAG) {
		/* Extended CAN ID format */
		id = (cf->can_id & GENMASK(17, 0)) << 13;
		id |= (cf->can_id & CAN_EFF_MASK) >> 18;

		/* Set SRR and IDE bit of extended frame */
		id |= CAN_RTR_MASK | CAN_IDE_MASK;

		/* Set RTR bit for remote TX request */
		if (cf->can_id & CAN_RTR_FLAG)
			id |= CAN_ID2_RTR_MASK;

		if (cdev->can.ctrlmode & CAN_CTRLMODE_FD) {
			dlc = can_len2dlc(cf->len) << CANFD_ID2_DLC_OFF;
			dlc |=  CAN_ID2_EDL_MASK;
			if (can_is_canfd_skb(skb)) {
				if (cf->flags & CANFD_BRS)
					dlc |= CANFD_ID2_BRS_MASK;
				if (cf->flags & CANFD_ESI)
					dlc |= CANFD_ID2_ESI_MASK;
			}
		} else {
			dlc = can_len2dlc(cf->len) << CAN_ID2_DLC_OFF;
		}
	} else {
		/* Standard CAN ID format */
		id = cf->can_id & CAN_SFF_MASK;

		/* Set RTR bit for remote TX request */
		if (cf->can_id & CAN_RTR_FLAG)
			id |= CAN_RTR_MASK;

		if (cdev->can.ctrlmode & CAN_CTRLMODE_FD) {
			id |= can_len2dlc(cf->len) << CANFD_ID1_DLC_OFF;
			id |= CAN_ID1_EDL_MASK;
			if (can_is_canfd_skb(skb)) {
				if (cf->flags & CANFD_BRS)
					id |= CANFD_ID1_BRS_MASK;
				if (cf->flags & CANFD_ESI)
					id |= CANFD_ID1_ESI_MASK;
			}
		} else {
			id |= can_len2dlc(cf->len) << CAN_ID1_DLC_OFF;
		}
	}

	can_put_echo_skb(skb, dev, 0);

	phytium_can_write(cdev, CAN_TX_FIFO, id);
	if (dlc)
		phytium_can_write(cdev, CAN_TX_FIFO, dlc);

	for (i = 0; i < cf->len; i += 4)
		phytium_can_write(cdev, CAN_TX_FIFO, i);
}

static netdev_tx_t phytium_can_tx_handler(struct phytium_can_dev *cdev)
{
	struct canfd_frame *cf = (struct canfd_frame *)cdev->tx_skb->data;
	struct net_device *dev = cdev->net;
	u32 tx_len, tx_fifo_used;

	tx_len = ((cf->can_id & CAN_EFF_FLAG) ? 8 : 4) + round_up(cf->len, 4);
	tx_fifo_used = (phytium_can_read(cdev, CAN_FIFO_CNT) & FIFO_CNT_TFN) >> 16;

	/* Check if the FIFO has enough space to transmit this skb. */
	if (tx_len > (cdev->tx_fifo_depth - tx_fifo_used)) {
		netdev_err(dev, "TX FIFO full when queue awake!\n");
		netif_stop_queue(dev);
		return NETDEV_TX_BUSY;
	}

	phytium_can_write_frame(cdev);

	/* If free space in TX FIFO is less than the max size of a frame,
	 * netif_stop_queue() to prevent TX FIFO overflow.
	 *
	 * Max posssible size of a frame:
	 * - standard CAN:	4 + 8 = 12 Bytes
	 * - extended CAN:	8 + 8 = 16 Bytes
	 * - standard CAN FD:	4 + 64 = 68 Bytes
	 * - extended CAN FD:	8 + 64 = 72 Bytes
	 */
	tx_fifo_used = (phytium_can_read(cdev, CAN_FIFO_CNT) & FIFO_CNT_TFN) >> 16;
	if (((cdev->can.ctrlmode & CAN_CTRLMODE_FD) &&
	     ((cdev->tx_fifo_depth - tx_fifo_used) < 72)) ||
	     ((cdev->tx_fifo_depth - tx_fifo_used) < 16))
		netif_stop_queue(dev);

	return NETDEV_TX_OK;
}

static irqreturn_t phytium_can_isr(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct phytium_can_dev *cdev = netdev_priv(dev);
	u32 isr;

	if (pm_runtime_suspended(cdev->dev))
		return IRQ_NONE;

	/* Get the interrupt status */
	isr = phytium_can_read(cdev, CAN_INTR) & INTR_STATUS_MASK;
	if (!isr)
		return IRQ_NONE;

	/* Ack all irqs */
	phytium_can_write(cdev, CAN_INTR, INTR_CLEAR_MASK);

	/* Schedule NAPI in case of
	 * - rx IRQ
	 *   - RFIS: RX FIFO Full
	 *   - REIS: RX frame end
	 * - error IRQ (BOIS|PWIS|PEIS|EIS)
	 */
	if (isr & (INTR_BOIS | INTR_PWIS | INTR_PEIS | INTR_EIS | INTR_RFIS | INTR_REIS)) {
		cdev->isr = isr;
		phytium_can_disable_all_interrupt(cdev);
		napi_schedule(&cdev->napi);
	}

	/* Handle tx IRQ
	 * - TFIS: TX FIFO full
	 * - TEIS: TX frame end
	 */
	if (isr & INTR_TEIS) {
		struct net_device_stats *stats = &dev->stats;

		/* Update TX stats */
		stats->tx_bytes += can_get_echo_skb(dev, 0);
		stats->tx_packets++;

		can_led_event(dev, CAN_LED_EVENT_TX);
		if (!(isr & INTR_TFIS))
			netif_wake_queue(dev);
	}

	return IRQ_HANDLED;
}

static int phytium_can_set_bittiming(struct net_device *dev)
{
	struct phytium_can_dev *cdev = netdev_priv(dev);
	const struct can_bittiming *bt = &cdev->can.bittiming;
	const struct can_bittiming *dbt = &cdev->can.data_bittiming;
	u32 arbr, datr;

	arbr = FIELD_PREP(ARB_RATE_CTRL_ARJW, (bt->sjw - 1));
	arbr |= FIELD_PREP(ARB_RATE_CTRL_APRS, (bt->prop_seg - 1));
	arbr |= FIELD_PREP(ARB_RATE_CTRL_APH1S, (bt->phase_seg1 - 1));
	arbr |= FIELD_PREP(ARB_RATE_CTRL_APH2S, (bt->phase_seg2 - 1));
	arbr |= FIELD_PREP(ARB_RATE_CTRL_APD, (bt->brp - 1));

	if (cdev->can.ctrlmode & CAN_CTRLMODE_FD) {
		datr = FIELD_PREP(DAT_RATE_CTRL_DRJW, (dbt->sjw - 1));
		datr = FIELD_PREP(DAT_RATE_CTRL_DPRS, (dbt->prop_seg - 1));
		datr = FIELD_PREP(DAT_RATE_CTRL_DPH1S, (dbt->phase_seg1 - 1));
		datr = FIELD_PREP(DAT_RATE_CTRL_DPH2S, (dbt->phase_seg2 - 1));
		datr = FIELD_PREP(DAT_RATE_CTRL_DPD, (dbt->brp - 1));
	} else {
		datr = arbr;
	}

	phytium_can_write(cdev, CAN_ARB_RATE_CTRL, arbr);
	phytium_can_write(cdev, CAN_DAT_RATE_CTRL, datr);

	return 0;
}

static void phytium_can_start(struct net_device *dev)
{
	struct phytium_can_dev *cdev = netdev_priv(dev);
	u32 ctrl;

	/* Disable transfer */
	ctrl = phytium_can_read(cdev, CAN_CTRL);
	ctrl &= ~CTRL_XFER;
	phytium_can_write(cdev, CAN_CTRL, ctrl);

	/* XXX: If CANFD, reset the controller */
	phytium_can_write(cdev, CAN_CTRL, (ctrl | CTRL_RST));

	/* Bittiming setup */
	phytium_can_set_bittiming(dev);

	/* Acceptance identifier mask setup */
	phytium_can_write(cdev, CAN_ACC_ID0_MASK, ACC_IDX_MASK_AID_MASK);
	phytium_can_write(cdev, CAN_ACC_ID1_MASK, ACC_IDX_MASK_AID_MASK);
	phytium_can_write(cdev, CAN_ACC_ID2_MASK, ACC_IDX_MASK_AID_MASK);
	phytium_can_write(cdev, CAN_ACC_ID3_MASK, ACC_IDX_MASK_AID_MASK);
	ctrl |= CTRL_AIME;

	if (cdev->can.ctrlmode & CAN_CTRLMODE_FD)
		ctrl |= CTRL_IOF | CTRL_FDCRC;

	phytium_can_write(cdev, CAN_CTRL, ctrl);

	cdev->can.state = CAN_STATE_ERROR_ACTIVE;

	phytium_can_enable_all_interrupts(cdev);

	if (cdev->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
		ctrl |= CTRL_XFER;
	else
		ctrl |= CTRL_XFER | CTRL_TXREQ;

	phytium_can_write(cdev, CAN_CTRL, ctrl);
}

static void phytium_can_stop(struct net_device *dev)
{
	struct phytium_can_dev *cdev = netdev_priv(dev);
	u32 ctrl;

	/* Disable all interrupts */
	phytium_can_disable_all_interrupt(cdev);

	/* Disable transfer and switch to receive-only mode */
	ctrl = phytium_can_read(cdev, CAN_CTRL);
	ctrl &= ~(CTRL_XFER | CTRL_TXREQ);
	phytium_can_write(cdev, CAN_CTRL, ctrl);

	/* Set the state as STOPPED */
	cdev->can.state = CAN_STATE_STOPPED;
}

static void phytium_can_clean(struct net_device *dev)
{
	struct phytium_can_dev *cdev = netdev_priv(dev);

	if (cdev->tx_skb) {
		dev->stats.tx_errors++;
		can_free_echo_skb(cdev->net, 0);
		cdev->tx_skb = NULL;
	}
}

static int phytium_can_set_mode(struct net_device *dev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		phytium_can_clean(dev);
		phytium_can_start(dev);
		netif_wake_queue(dev);
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

static int phytium_can_open(struct net_device *dev)
{
	struct phytium_can_dev *cdev = netdev_priv(dev);
	int ret;

	/* Start clock */
	ret = pm_runtime_resume(cdev->dev);
	if (ret)
		return ret;

	/* Open the CAN device */
	ret = open_candev(dev);
	if (ret) {
		netdev_err(dev, "failed to open can device\n");
		goto disable_clk;
	}

	/* Register interrupt handler */
	ret = devm_request_irq(cdev->dev, dev->irq, phytium_can_isr,
			       IRQF_SHARED, dev->name, dev);
	if (ret < 0) {
		netdev_err(dev, "failed to request interrupt\n");
		goto fail;
	}

	/* Start the controller */
	phytium_can_start(dev);

	can_led_event(dev, CAN_LED_EVENT_OPEN);
	napi_enable(&cdev->napi);
	netif_start_queue(dev);

	return 0;

fail:
	close_candev(dev);
disable_clk:
	pm_runtime_put_sync(cdev->dev);
	return ret;
}

static int phytium_can_close(struct net_device *dev)
{
	struct phytium_can_dev *cdev = netdev_priv(dev);

	netif_stop_queue(dev);
	napi_disable(&cdev->napi);

	phytium_can_stop(dev);
	pm_runtime_put_sync(cdev->dev);

	close_candev(dev);
	can_led_event(dev, CAN_LED_EVENT_STOP);

	return 0;
}

static netdev_tx_t phytium_can_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct phytium_can_dev *cdev = netdev_priv(dev);

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	cdev->tx_skb = skb;

	return phytium_can_tx_handler(cdev);
}

static const struct net_device_ops phytium_can_netdev_ops = {
	.ndo_open = phytium_can_open,
	.ndo_stop = phytium_can_close,
	.ndo_start_xmit = phytium_can_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};

static int register_phytium_can_dev(struct net_device *dev)
{
	dev->flags |= IFF_ECHO;
	dev->netdev_ops = &phytium_can_netdev_ops;

	return register_candev(dev);
}

static int phytium_can_dev_setup(struct phytium_can_dev *cdev)
{
	struct net_device *dev = cdev->net;

	netif_napi_add(dev, &cdev->napi, phytium_can_poll, 64);

	cdev->can.do_set_mode = phytium_can_set_mode;
	cdev->can.do_get_berr_counter = phytium_can_get_berr_counter;

	cdev->can.ctrlmode_supported = CAN_CTRLMODE_LISTENONLY |
				       CAN_CTRLMODE_BERR_REPORTING;
	cdev->can.bittiming_const = cdev->bit_timing;

	if (cdev->fdmode) {
		cdev->can.ctrlmode_supported |= CAN_CTRLMODE_FD;
		can_set_static_ctrlmode(dev, CAN_CTRLMODE_FD);
		cdev->can.data_bittiming_const = cdev->bit_timing;
	}

	return 0;
}

struct phytium_can_dev *phytium_can_allocate_dev(struct device *dev, int sizeof_priv,
						 int tx_fifo_depth)
{
	struct phytium_can_dev *cdev = NULL;
	struct net_device *net_dev;

	/* Allocate the can device struct */
	net_dev = alloc_candev(sizeof_priv, tx_fifo_depth);
	if (!net_dev) {
		dev_err(dev, "Failed to allocate CAN device.\n");
		goto out;
	}

	cdev = netdev_priv(net_dev);
	cdev->net = net_dev;
	cdev->dev = dev;
	SET_NETDEV_DEV(net_dev, dev);

out:
	return cdev;
}
EXPORT_SYMBOL(phytium_can_allocate_dev);

void phytium_can_free_dev(struct net_device *net)
{
	free_candev(net);
}
EXPORT_SYMBOL(phytium_can_free_dev);

int phytium_can_register(struct phytium_can_dev *cdev)
{
	int ret;

	ret = pm_runtime_resume(cdev->dev);
	if (ret)
		return ret;

	ret = phytium_can_dev_setup(cdev);
	if (ret)
		goto fail;

	ret = register_phytium_can_dev(cdev->net);
	if (ret) {
		dev_err(cdev->dev, "registering %s failed (err=%d)\n",
			cdev->net->name, ret);
		goto fail;
	}

	devm_can_led_init(cdev->net);

	dev_info(cdev->dev, "%s device registered (irq=%d)\n",
		 KBUILD_MODNAME, cdev->net->irq);

	/* Probe finished
	 * Stop clocks. They will be reactivated once the device is opened.
	 */
	pm_runtime_put_sync(cdev->dev);

	return 0;

fail:
	pm_runtime_put_sync(cdev->dev);
	return ret;
}
EXPORT_SYMBOL(phytium_can_register);

void phytium_can_unregister(struct phytium_can_dev *cdev)
{
	unregister_candev(cdev->net);
}
EXPORT_SYMBOL(phytium_can_unregister);

int phytium_can_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct phytium_can_dev *cdev = netdev_priv(ndev);

	if (netif_running(ndev)) {
		netif_stop_queue(ndev);
		netif_device_detach(ndev);
		phytium_can_stop(ndev);
		pm_runtime_put_sync(cdev->dev);
	}

	cdev->can.state = CAN_STATE_SLEEPING;

	return 0;
}
EXPORT_SYMBOL(phytium_can_suspend);

int phytium_can_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct phytium_can_dev *cdev = netdev_priv(ndev);
	int ret;

	cdev->can.state = CAN_STATE_ERROR_ACTIVE;

	if (netif_running(ndev)) {
		ret = pm_runtime_resume(cdev->dev);
		if (ret)
			return ret;

		phytium_can_start(ndev);
		netif_device_attach(ndev);
		netif_start_queue(ndev);
	}

	return 0;
}
EXPORT_SYMBOL(phytium_can_resume);

MODULE_AUTHOR("Cheng Quan <chengquan@phytium.com.cn>");
MODULE_AUTHOR("Chen Baozi <chenbaozi@phytium.com.cn>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN bus driver for Phytium CAN controller");

