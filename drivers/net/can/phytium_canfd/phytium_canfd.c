// SPDX-License-Identifier: GPL-2.0
/* CAN bus driver for Phytium CAN controller
 *
 * Copyright (C) 2021 Phytium Technology Co., Ltd.
 */
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
#include "phytium_canfd.h"

/**
 * phytium_can_write_reg - Write a value to the device register
 * @priv:	Driver private data structure
 * @reg:	Register offset
 * @val:	Value to write at the Register offset
 *
 * Write data to the paricular CAN register
 */
static void
phytium_can_write_reg(const struct phytium_can_priv *priv, enum phytium_can_reg reg, u32 val)
{
	writel(val, priv->reg_base + reg);
}

/**
 * phytium_can_read_reg - Read a value from the device register
 * @priv:	Driver private data structure
 * @reg:	Register offset
 *
 * Read data from the particular CAN register
 * Return: value read from the CAN register
 */
static u32 phytium_can_read_reg(const struct phytium_can_priv *priv, enum phytium_can_reg reg)
{
	return readl(priv->reg_base + reg);
}

/**
 * phytium_can_set_reg_bits - set a bit value to the device register
 * @priv:	Driver private data structure
 * @reg:	Register offset
 * @bs:     The bit mask
 *
 * Read data from the particular CAN register
 * Return: value read from the CAN register
 */
static void
phytium_can_set_reg_bits(const struct phytium_can_priv *priv, enum phytium_can_reg reg, u32 bs)
{
	u32 val = readl(priv->reg_base + reg);

	val |= bs;
	writel(val, priv->reg_base + reg);
}

/**
 * phytium_can_clr_reg_bits - clear a bit value to the device register
 * @priv:	Driver private data structure
 * @reg:	Register offset
 * @bs:     The bit mask
 *
 * Read data from the particular CAN register
 * Return: value read from the CAN register
 */
static void
phytium_can_clr_reg_bits(const struct phytium_can_priv *priv, enum phytium_can_reg reg, u32 bs)
{
	u32 val = readl(priv->reg_base + reg);

	val &= ~bs;
	writel(val, priv->reg_base + reg);
}

/**
 * phytium_can_set_bittiming - CAN set bit timing routine
 * @ndev:	Pointer to net_device structure
 *
 * This is the driver set bittiming  routine.
 * Return: 0 on success and failure value on error
 */
static int phytium_can_set_bittiming(struct net_device *ndev)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);
	struct can_bittiming *bt = &priv->can.bittiming;
	struct can_bittiming *dbt = &priv->can.data_bittiming;
	u32 btr, dbtr;
	u32 is_config_mode;

	/* Check whether Phytium CAN is in configuration mode.
	 * It cannot set bit timing if Phytium CAN is not in configuration mode.
	 */
	is_config_mode = (priv->read_reg(priv, FTCAN_CTRL_OFFSET) &
			  FTCAN_CTRL_XFER_MASK);
	if (is_config_mode) {
		netdev_alert(ndev, "BUG! Cannot set bittiming - CAN is not in config mode\n");
		return -EPERM;
	}

	/* Setting Baud Rate prescalar value in BRPR Register */
	btr = (bt->brp - 1) << 16;

	/* Setting Time Segment 1 in BTR Register */
	btr |= (bt->prop_seg - 1) << 2;

	btr |= (bt->phase_seg1 - 1) << 5;

	/* Setting Time Segment 2 in BTR Register */
	btr |= (bt->phase_seg2 - 1) << 8;

	/* Setting Synchronous jump width in BTR Register */
	btr |= (bt->sjw - 1);

	dbtr = (dbt->brp - 1) << 16;
	dbtr |= (dbt->prop_seg - 1) << 2;
	dbtr |= (dbt->phase_seg1 - 1) << 5;
	dbtr |= (dbt->phase_seg2 - 1) << 8;
	dbtr |= (dbt->sjw - 1);

	if (btr && dbtr) {
		priv->write_reg(priv, FTCAN_ARB_RATE_CTRL_OFFSET, btr);
		priv->write_reg(priv, FTCAN_DAT_RATE_CTRL_OFFSET, dbtr);
	} else {
		priv->write_reg(priv, FTCAN_ARB_RATE_CTRL_OFFSET, btr);
		priv->write_reg(priv, FTCAN_DAT_RATE_CTRL_OFFSET, btr);
	}

	netdev_dbg(ndev, "DAT=0x%08x, ARB=0x%08x\n",
		   priv->read_reg(priv, FTCAN_DAT_RATE_CTRL_OFFSET),
		   priv->read_reg(priv, FTCAN_ARB_RATE_CTRL_OFFSET));

	return 0;
}

/**
 * phytium_can_chip_start - This the drivers start routine
 * @ndev:	Pointer to net_device structure
 *
 * This is the drivers start routine.
 * Based on the State of the CAN device it puts
 * the CAN device into a proper mode.
 *
 * Return: 0 on success and failure value on error
 */
static int phytium_can_chip_start(struct net_device *ndev)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);
	int err;

	/*reset can controller*/
	priv->set_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_RST_MASK);

	/*Disable Transfer*/
	priv->clr_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_XFER_MASK);
	priv->can.state = CAN_STATE_STOPPED;

	err = phytium_can_set_bittiming(ndev);
	if (err < 0)
		return err;

	/* Identifier mask enable */
	priv->set_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_AIME_MASK | FTCAN_CTRL_IOF_MASK);
	priv->write_reg(priv, FTCAN_ACC_ID0_MASK_OFFSET, FTCAN_ACC_IDN_MASK);
	priv->write_reg(priv, FTCAN_ACC_ID1_MASK_OFFSET, FTCAN_ACC_IDN_MASK);
	priv->write_reg(priv, FTCAN_ACC_ID2_MASK_OFFSET, FTCAN_ACC_IDN_MASK);
	priv->write_reg(priv, FTCAN_ACC_ID3_MASK_OFFSET, FTCAN_ACC_IDN_MASK);

	/* Enable interrupts */
	priv->write_reg(priv, FTCAN_INTR_OFFSET, FTCAN_INTR_EN);

	if (priv->can.ctrlmode & CAN_CTRLMODE_FD)
		priv->set_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_FDCRC_MASK);
	else
		priv->clr_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_FDCRC_MASK);

	/*Enable Transfer*/
	priv->set_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_XFER_MASK);

	netdev_dbg(ndev, "status:#x%08x\n", priv->read_reg(priv, FTCAN_XFER_STS_OFFSET));

	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	return 0;
}

/**
 * phytium_can_do_set_mode - This sets the mode of the driver
 * @ndev:	Pointer to net_device structure
 * @mode:	Tells the mode of the driver
 *
 * This check the drivers state and calls the
 * the corresponding modes to set.
 *
 * Return: 0 on success and failure value on error
 */
static int phytium_can_do_set_mode(struct net_device *ndev, enum can_mode mode)
{
	int ret;

	switch (mode) {
	case CAN_MODE_START:
		ret = phytium_can_chip_start(ndev);
		if (ret < 0) {
			netdev_err(ndev, "xcan_chip_start failed!\n");
			return ret;
		}
		netif_wake_queue(ndev);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

/**
 * phytium_can_start_xmit - Starts the transmission
 * @skb:	sk_buff pointer that contains data to be Txed
 * @ndev:	Pointer to net_device structure
 *
 * This function is invoked from upper layers to initiate transmission. This
 * function uses the next available free txbuff and populates their fields to
 * start the transmission.
 *
 * Return: 0 on success and failure value on error
 */
static int phytium_can_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct canfd_frame *cf = (struct canfd_frame *)skb->data;
	u32 id, dlc, data_len, frame_head[2] = {0, 0};
	u32 tx_fifo_cnt;
	int i;

	if (can_dropped_invalid_skb(ndev, skb))
		return NETDEV_TX_OK;

	/* Check if the TX buffer is full */
	tx_fifo_cnt = (priv->read_reg(priv, FTCAN_FIFO_CNT_OFFSET) >> FTCAN_FIFO_CNT_TFN_SHIFT);
	if (tx_fifo_cnt == priv->tx_max) {
		netif_stop_queue(ndev);
		netdev_err(ndev, "BUG!, TX FIFO full when queue awake!\n");
		return NETDEV_TX_BUSY;
	}

	if (priv->tx_head == priv->tx_tail) {
		priv->tx_head = 0;
		priv->tx_tail = 0;
	}

	data_len = can_len2dlc(cf->len);
	netdev_dbg(ndev, "Get CAN controller data length %d, len:%d\n", data_len, cf->len);

	/* Watch carefully on the bit sequence */
	if (cf->can_id & CAN_EFF_FLAG) {
		/* Extended CAN ID format */
		id = ((cf->can_id & CAN_EFF_MASK) << FTCAN_IDR_ID2_SHIFT) &
			FTCAN_IDR_ID2_MASK;
		id |= (((cf->can_id & CAN_EFF_MASK) >>
			(CAN_EFF_ID_BITS - CAN_SFF_ID_BITS)) <<
			FTCAN_IDR_ID1_SHIFT) & FTCAN_IDR_ID1_MASK;

		/* The substibute remote TX request bit should be "1"
		 * for extended frames as in the Phytium CAN datasheet
		 */
		id |= FTCAN_IDR_IDE_MASK | FTCAN_IDR_SRR_MASK;

		if (cf->can_id & CAN_RTR_FLAG)
			/* Extended frames remote TX request */
			id |= FTCAN_IDR_RTR_MASK;
		if ((priv->can.ctrlmode & CAN_CTRLMODE_FD) && can_is_canfd_skb(skb))
			dlc = data_len << FTCANFD_IDR_EDLC_SHIFT;
		else
			dlc = data_len << FTCAN_IDR_EDLC_SHIFT;

		if (priv->can.ctrlmode & CAN_CTRLMODE_FD) {
			dlc |= FTCANFD_ID2_FDL_MASK;
			if (cf->flags & CANFD_BRS)
				dlc |= FTCANFD_ID2_BRS_MASK;
			if (cf->flags & CANFD_ESI)
				dlc |= FTCANFD_ID2_ESI_MASK;
		}

		frame_head[0] = cpu_to_be32p(&id);
		frame_head[1] = cpu_to_be32p(&dlc);

		/* Write the Frame to Phytium CAN TX FIFO */
		priv->write_reg(priv, FTCAN_TX_FIFO_OFFSET, frame_head[0]);
		priv->write_reg(priv, FTCAN_TX_FIFO_OFFSET, frame_head[1]);
		netdev_dbg(ndev, "Write atbitration field [0]:%x [1]:%x\n",
			   frame_head[0], frame_head[1]);
	} else {
		/* Standard CAN ID format */
		id = ((cf->can_id & CAN_SFF_MASK) << FTCAN_IDR_ID1_SHIFT) & FTCAN_IDR_ID1_MASK;

		if (cf->can_id & CAN_RTR_FLAG)
			/* Standard frames remote TX request */
			id |= FTCAN_IDR_SRR_MASK;

		if (priv->can.ctrlmode & CAN_CTRLMODE_FD)
			dlc = (data_len << FTCANFD_IDR1_SDLC_SHIFT) | FTCANFD_IDR_PAD_MASK;
		else
			dlc = (data_len << FTCAN_IDR_SDLC_SHIFT) | FTCAN_IDR_PAD_MASK;

		id |= dlc;

		if (priv->can.ctrlmode & CAN_CTRLMODE_FD) {
			id |= FTCANFD_ID1_FDL_MASK;
			if (cf->flags & CANFD_BRS)
				id |= FTCANFD_ID1_BRS_MASK;
			if (cf->flags & CANFD_ESI)
				id |= FTCANFD_ID1_ESI_MASK;
		}
		frame_head[0] =  cpu_to_be32p(&id);
		/* Write the Frame to Phytium CAN TX FIFO */
		priv->write_reg(priv, FTCAN_TX_FIFO_OFFSET, frame_head[0]);
		netdev_dbg(ndev, "Write atbitration field [0] %x\n", frame_head[0]);
	}

	if (!(cf->can_id & CAN_RTR_FLAG)) {
		netdev_dbg(ndev, "Write CAN data frame\n");
		for (i = 0; i < cf->len; i += 4) {
			priv->write_reg(priv, FTCAN_TX_FIFO_OFFSET, *(__be32 *)(cf->data + i));
			netdev_dbg(ndev, "[%d]:%x\n", i, *(__be32 *)(cf->data + i));
		}
		netdev_dbg(ndev, "\n-------------------\n");
	}
	stats->tx_bytes += cf->len;
	can_put_echo_skb(skb, ndev, priv->tx_head % priv->tx_max);
	priv->tx_head++;

	netif_stop_queue(ndev);
	/* triggers tranmission */
	priv->clr_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_XFER_MASK);
	priv->set_reg_bits(priv, FTCAN_CTRL_OFFSET,
			   FTCAN_CTRL_TXREQ_MASK | FTCAN_CTRL_XFER_MASK);

	netdev_dbg(ndev, "Trigger send message!\n");
	return NETDEV_TX_OK;
}

/**
 * phytium_can_rx -  Is called from CAN isr to complete the received
 *		frame  processing
 * @ndev:	Pointer to net_device structure
 *
 * This function is invoked from the CAN isr(poll) to process the Rx frames. It
 * does minimal processing and invokes "netif_receive_skb" to complete further
 * processing.
 * Return: 1 on success and 0 on failure.
 */
static int phytium_can_rx(struct net_device *ndev)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct canfd_frame *cf;
	struct sk_buff *skb;
	u32 id_ftcan, dlc;
	int i;

	/* Read a frame from Phytium CAN */
	id_ftcan = priv->read_reg(priv, FTCAN_RX_FIFO_OFFSET);
	id_ftcan = be32_to_cpup(&id_ftcan);

	/* Change Phytium CAN ID format to socketCAN ID format */
	if (id_ftcan & FTCAN_IDR_IDE_MASK) {
		/* The received frame is an Extended format frame */
		dlc = priv->read_reg(priv, FTCAN_RX_FIFO_OFFSET);
		dlc = be32_to_cpup(&dlc);
		if (dlc & FTCANFD_ID2_FDL_MASK) {
			skb = alloc_canfd_skb(ndev, &cf);
			dlc = (dlc & FTCANFD_ID2_EDLC_MASK) >> FTCANFD_IDR_EDLC_SHIFT;
		} else {
			skb = alloc_can_skb(ndev, (struct can_frame **)&cf);
			dlc = (dlc & FTCAN_ID2_EDLC_MASK) >> FTCAN_IDR_EDLC_SHIFT;
		}
		if (unlikely(!skb)) {
			stats->rx_dropped++;
			return 0;
		}
		if (can_is_canfd_skb(skb) && (dlc & FTCANFD_ID2_BRS_MASK))
			cf->flags |= CANFD_BRS;
		if (can_is_canfd_skb(skb) && (dlc & FTCANFD_ID2_ESI_MASK))
			cf->flags |= CANFD_ESI;

		cf->can_id = (id_ftcan & FTCAN_IDR_ID1_MASK) >> 3;
		cf->can_id |= (id_ftcan & FTCAN_IDR_ID2_MASK) >> FTCAN_IDR_ID2_SHIFT;
		cf->can_id |= CAN_EFF_FLAG;
		if (id_ftcan & FTCAN_IDR_RTR_MASK)
			cf->can_id |= CAN_RTR_FLAG;
		netdev_dbg(ndev, "Extended frame flag :0x%x\n", cf->flags);
	} else {
		if (id_ftcan & FTCANFD_ID1_FDL_MASK) {
			skb = alloc_canfd_skb(ndev, &cf);
			dlc = ((id_ftcan & FTCANFD_ID1_SDLC_MASK) >> FTCANFD_IDR1_SDLC_SHIFT);
		} else {
			skb = alloc_can_skb(ndev, (struct can_frame **)&cf);
			dlc = ((id_ftcan & FTCAN_IDR_DLC_MASK) >> FTCAN_IDR_SDLC_SHIFT);
		}
		if (unlikely(!skb)) {
			stats->rx_dropped++;
			return 0;
		}

		if (can_is_canfd_skb(skb) && (id_ftcan & FTCANFD_ID1_BRS_MASK))
			cf->flags |= CANFD_BRS;
		if (can_is_canfd_skb(skb) && (id_ftcan & FTCANFD_ID1_ESI_MASK))
			cf->flags |= CANFD_ESI;

		/* The received frame is a standard format frame */
		cf->can_id = (id_ftcan & FTCAN_IDR_ID1_MASK) >> FTCAN_IDR_ID1_SHIFT;
		if (id_ftcan & FTCAN_IDR_SRR_MASK)
			cf->can_id |= CAN_RTR_FLAG;
		netdev_dbg(ndev, "Standard frame flag :0x%x\n", cf->flags);
	}

	if (can_is_canfd_skb(skb))
		cf->len = can_dlc2len(dlc);
	else
		cf->len = get_can_dlc(dlc);

	netdev_dbg(ndev, "Frame:%s, len:%d, dlc:%d, Id:%x\n",
		   can_is_canfd_skb(skb) ? "FD" : "CAN", cf->len, dlc, cf->can_id);

	if (!(cf->can_id & CAN_RTR_FLAG)) {
		/* Change Phytium CAN data format to socketCAN data format */
		netdev_dbg(ndev, "Read CAN data frame\n");
		for (i = 0; i < cf->len; i += 4) {
			*(__be32 *)(cf->data + i) = priv->read_reg(priv, FTCAN_RX_FIFO_OFFSET);
			netdev_dbg(ndev, "[%d]:%x ", i, *(__be32 *)(cf->data + i));
		}
		netdev_dbg(ndev, "\n-------------------\n");
	}

	stats->rx_bytes += cf->len;
	stats->rx_packets++;
	netif_receive_skb(skb);

	return 1;
}

/**
 * phytium_can_err_interrupt - error frame Isr
 * @ndev:	net_device pointer
 *
 * This is the CAN error interrupt and it will
 * check the the type of error and forward the error
 * frame to upper layers.
 */
static void phytium_can_err_interrupt(struct net_device *ndev, u32 isr)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	u32  txerr = 0, rxerr = 0;

	skb = alloc_can_err_skb(ndev, &cf);

	rxerr = priv->read_reg(priv, FTCAN_ERR_CNT_OFFSET) & FTCAN_ERR_CNT_RFN_MASK;
	txerr = ((priv->read_reg(priv, FTCAN_ERR_CNT_OFFSET) &
		FTCAN_ERR_CNT_TFN_MASK) >> FTCAN_ERR_CNT_TFN_SHIFT);

	if (isr & FTCAN_INTR_BOIS_MASK) {
		priv->can.state = CAN_STATE_BUS_OFF;
		priv->can.can_stats.bus_off++;
		/* Leave device in Config Mode in bus-off state */
		can_bus_off(ndev);
		if (skb)
			cf->can_id |= CAN_ERR_BUSOFF;
	} else if ((isr & FTCAN_INTR_PEIS_MASK) == FTCAN_INTR_PEIS_MASK) {
		priv->can.state = CAN_STATE_ERROR_PASSIVE;
		priv->can.can_stats.error_passive++;
		if (skb) {
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] = (rxerr > 127) ?
					CAN_ERR_CRTL_RX_PASSIVE :
					CAN_ERR_CRTL_TX_PASSIVE;
			cf->data[6] = txerr;
			cf->data[7] = rxerr;
		}
	} else if (isr & FTCAN_INTR_PWIS_MASK) {
		priv->can.state = CAN_STATE_ERROR_WARNING;
		priv->can.can_stats.error_warning++;
		if (skb) {
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] |= (txerr > rxerr) ?
					CAN_ERR_CRTL_TX_WARNING :
					CAN_ERR_CRTL_RX_WARNING;
			cf->data[6] = txerr;
			cf->data[7] = rxerr;
		}
	}

	/* Check for RX FIFO Overflow interrupt */
	if (isr & FTCAN_INTR_RFIS_MASK) {
		stats->rx_over_errors++;
		stats->rx_errors++;

		if (skb) {
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
		}
	}

	if (skb) {
		stats->rx_packets++;
		stats->rx_bytes += cf->can_dlc;
		netif_rx(skb);
	}

	netdev_dbg(ndev, "%s: error status :0x%x\n", __func__,
		   priv->read_reg(priv, FTCAN_INTR_OFFSET) & FTCAN_INTR_STATUS_MASK);
}

/**
 * phytium_can_rx_poll - Poll routine for rx packets (NAPI)
 * @napi:	napi structure pointer
 * @quota:	Max number of rx packets to be processed.
 *
 * This is the poll routine for rx part.
 * It will process the packets maximux quota value.
 *
 * Return: number of packets received
 */
static int phytium_can_rx_poll(struct napi_struct *napi, int quota)
{
	struct net_device *ndev = napi->dev;
	struct phytium_can_priv *priv = netdev_priv(ndev);
	u32 isr, rx_fifo_cnt;
	int work_done = 0;

	priv->quota = quota;
	isr = priv->isr;

	rx_fifo_cnt = (priv->read_reg(priv, FTCAN_FIFO_CNT_OFFSET) & FTCAN_FIFO_CNT_RFN_MASK);

	netdev_dbg(ndev, "Received  %d frame\n", rx_fifo_cnt);
	while ((rx_fifo_cnt != 0) && (work_done < quota)) {
		if (isr & FTCAN_INTR_REIS_MASK)
			work_done += phytium_can_rx(ndev);
		else
			break;
		rx_fifo_cnt = (priv->read_reg(priv, FTCAN_FIFO_CNT_OFFSET) &
						FTCAN_FIFO_CNT_RFN_MASK);
		netdev_dbg(ndev, "Next received %d frame again.\n", rx_fifo_cnt);
	}

	netdev_dbg(ndev, "Net can receive %d frame, quota:%d\n", work_done, quota);
	if (work_done)
		can_led_event(ndev, CAN_LED_EVENT_RX);

	if (work_done < quota) {
		priv->isr = 0;
		napi_complete_done(napi, work_done);
		priv->set_reg_bits(priv, FTCAN_INTR_OFFSET, FTCAN_INTR_REIE_MASK);
	}
	return work_done;
}

/**
 * phytium_can_tx_interrupt - Tx Done Isr
 * @ndev:	net_device pointer
 * @isr:	Interrupt status register value
 */
static void phytium_can_tx_interrupt(struct net_device *ndev, u32 isr)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;

	while ((priv->tx_head - priv->tx_tail > 0) && (isr & FTCAN_INTR_TEIS_MASK)) {
		priv->set_reg_bits(priv, FTCAN_INTR_OFFSET,
				   FTCAN_INTR_TEIC_MASK | FTCAN_INTR_REIC_MASK);
		can_get_echo_skb(ndev, priv->tx_tail % priv->tx_max);
		priv->tx_tail++;
		stats->tx_packets++;
		isr = (priv->read_reg(priv, FTCAN_INTR_OFFSET) &
		       FTCAN_INTR_STATUS_MASK);
	}

	priv->clr_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_XFER_MASK);
	priv->clr_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_TXREQ_MASK);
	priv->set_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_XFER_MASK);
	netdev_dbg(ndev, "Finish transform packets %lu\n", stats->tx_packets);

	can_led_event(ndev, CAN_LED_EVENT_TX);
	netif_wake_queue(ndev);
}

/**
 * phytium_can_interrupt - CAN Isr
 * @irq:	irq number
 * @dev_id:	device id poniter
 *
 * This is the phytium CAN Isr. It checks for the type of interrupt
 * and invokes the corresponding ISR.
 *
 * Return:
 * IRQ_NONE - If CAN device is in sleep mode, IRQ_HANDLED otherwise
 */
static irqreturn_t phytium_can_interrupt(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct phytium_can_priv *priv = netdev_priv(ndev);
	u32 isr;

	/* Get the interrupt status from Phytium CAN */
	isr = (priv->read_reg(priv, FTCAN_INTR_OFFSET) & FTCAN_INTR_STATUS_MASK);
	if (!isr)
		return IRQ_NONE;

	/* Check for FIFO full interrupt and alarm */
	if ((isr & FTCAN_INTR_RFIS_MASK)) {
		netdev_dbg(ndev, "rx_fifo is full!.\n");
		isr &= (~FTCAN_INTR_RFIS_MASK);
		priv->clr_reg_bits(priv, FTCAN_INTR_OFFSET, FTCAN_INTR_RFIE_MASK);
		priv->set_reg_bits(priv, FTCAN_INTR_OFFSET, FTCAN_INTR_RFIC_MASK);
		napi_schedule(&priv->napi);
	}

	/* Check for Tx interrupt and Processing it */
	if ((isr & FTCAN_INTR_TEIS_MASK)) {
		isr &= (~FTCAN_INTR_REIS_MASK);
		phytium_can_tx_interrupt(ndev, isr);
	}

	/* Check for the type of error interrupt and Processing it */
	if (isr & (FTCAN_INTR_EIS_MASK | FTCAN_INTR_RFIS_MASK |
			FTCAN_INTR_BOIS_MASK | FTCAN_INTR_PEIS_MASK)) {
		priv->clr_reg_bits(priv, FTCAN_INTR_OFFSET,
				   (FTCAN_INTR_EIC_MASK | FTCAN_INTR_RFIC_MASK |
				    FTCAN_INTR_BOIC_MASK | FTCAN_INTR_PEIC_MASK));
		phytium_can_err_interrupt(ndev, isr);
		priv->set_reg_bits(priv, FTCAN_INTR_OFFSET,
				   (FTCAN_INTR_EIC_MASK | FTCAN_INTR_RFIC_MASK |
				   FTCAN_INTR_BOIC_MASK | FTCAN_INTR_PEIC_MASK));
		priv->set_reg_bits(priv, FTCAN_INTR_OFFSET, FTCAN_INTR_EN);
		return IRQ_HANDLED;
	}

	/* Check for the type of receive interrupt and Processing it */
	if (isr & (FTCAN_INTR_REIS_MASK)) {
		priv->isr = (isr & FTCAN_INTR_REIS_MASK);
		priv->clr_reg_bits(priv, FTCAN_INTR_OFFSET, FTCAN_INTR_REIE_MASK);
		priv->set_reg_bits(priv, FTCAN_INTR_OFFSET, FTCAN_INTR_REIC_MASK);
		napi_schedule(&priv->napi);
	}

	return IRQ_HANDLED;
}

/**
 * phytium_can_chip_stop - Driver stop routine
 * @ndev:	Pointer to net_device structure
 *
 * This is the drivers stop routine. It will disable the
 * interrupts and put the device into configuration mode.
 */
static void phytium_can_chip_stop(struct net_device *ndev)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);
	u32 ier;

	/* Disable interrupts and leave the can in configuration mode */
	ier = (FTCAN_INTR_DIS & FTCAN_INTR_EN_MASK);
	priv->clr_reg_bits(priv, FTCAN_INTR_OFFSET, ier);

	/*Disable Transfer*/
	priv->clr_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_XFER_MASK);
	priv->clr_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_TXREQ_MASK);
	/*reset canfd*/
	priv->set_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_RST_MASK);
	priv->can.state = CAN_STATE_STOPPED;
}

/**
 * phytium_can_open - Driver open routine
 * @ndev:	Pointer to net_device structure
 *
 * This is the driver open routine.
 * Return: 0 on success and failure value on error
 */
static int phytium_can_open(struct net_device *ndev)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);
	int ret;

	ret = request_irq(ndev->irq, phytium_can_interrupt, priv->irq_flags,
			  ndev->name, ndev);
	if (ret < 0) {
		netdev_err(ndev, "irq allocation for CAN failed\n");
		goto err;
	}

	/* Common open */
	ret = open_candev(ndev);
	if (ret)
		goto err_irq;

	ret = phytium_can_chip_start(ndev);
	if (ret < 0) {
		netdev_err(ndev, "phytium_can_chip_start failed!\n");
		goto err_candev;
	}

	can_led_event(ndev, CAN_LED_EVENT_OPEN);
	napi_enable(&priv->napi);
	netif_start_queue(ndev);

	return 0;

err_candev:
	close_candev(ndev);
err_irq:
	free_irq(ndev->irq, ndev);
err:
	return ret;
}

/**
 * phytium_can_close - Driver close routine
 * @ndev:	Pointer to net_device structure
 *
 * Return: 0 always
 */
static int phytium_can_close(struct net_device *ndev)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);

	netif_stop_queue(ndev);
	napi_disable(&priv->napi);
	phytium_can_chip_stop(ndev);
	free_irq(ndev->irq, ndev);
	close_candev(ndev);

	can_led_event(ndev, CAN_LED_EVENT_STOP);
	priv->tx_head = 0;
	priv->tx_tail = 0;
	return 0;
}

/**
 * phytium_can_get_berr_counter - error counter routine
 * @ndev:	Pointer to net_device structure
 * @bec:	Pointer to can_berr_counter structure
 *
 * This is the driver error counter routine.
 * Return: 0 on success and failure value on error
 */
static int phytium_can_get_berr_counter(const struct net_device *ndev,
					struct can_berr_counter *bec)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);

	bec->rxerr = priv->read_reg(priv, FTCAN_ERR_CNT_OFFSET) &
					FTCAN_ERR_CNT_RFN_MASK;
	bec->txerr = ((priv->read_reg(priv, FTCAN_ERR_CNT_OFFSET) &
					FTCAN_ERR_CNT_TFN_MASK) >> FTCAN_ERR_CNT_TFN_SHIFT);
	return 0;
}

static const struct net_device_ops phytium_can_netdev_ops = {
	.ndo_open	= phytium_can_open,
	.ndo_stop	= phytium_can_close,
	.ndo_start_xmit	= phytium_can_start_xmit,
	.ndo_change_mtu	= can_change_mtu,
};

#define phytium_can_dev_pm_ops NULL

/**
 * phytium_canfd_register - Platform registration call
 * @ndev:	Handle to the net device structure
 *
 * This function does  registration for the CAN device.
 *
 * Return: 0 on success and failure value on error
 */
int phytium_canfd_register(struct net_device *ndev)
{
	struct phytium_can_priv *priv;
	int ret;

	priv = netdev_priv(ndev);
	priv->can.do_set_mode = phytium_can_do_set_mode;
	priv->can.do_get_berr_counter = phytium_can_get_berr_counter;

	if (priv->fdmode) {
		priv->can.ctrlmode_supported |= CAN_CTRLMODE_FD;
		ndev->mtu = CANFD_MTU;
		priv->can.ctrlmode = CAN_CTRLMODE_FD;
		priv->can.data_bittiming_const	= priv->can.bittiming_const;
		netdev_dbg(ndev, "Support CANFD mode\n");
	}
	priv->can.ctrlmode_supported |= CAN_CTRLMODE_BERR_REPORTING;
	priv->tx_head = 0;
	priv->tx_tail = 0;
	spin_lock_init(&priv->lock);

	ndev->netdev_ops = &phytium_can_netdev_ops;

	priv->write_reg = phytium_can_write_reg;
	priv->read_reg = phytium_can_read_reg;
	priv->set_reg_bits = phytium_can_set_reg_bits;
	priv->clr_reg_bits = phytium_can_clr_reg_bits;

	netif_napi_add(ndev, &priv->napi, phytium_can_rx_poll, FTCAN_NAPI_WEIGHT);

	ret = register_candev(ndev);
	if (ret) {
		dev_err(priv->dev, "fail to register failed (err=%d)\n", ret);
		goto err_disableclks;
	}

	devm_can_led_init(ndev);
	netdev_dbg(ndev, "reg_base=0x%p irq=%d clock=%d, tx fifo depth:%d\n",
		   priv->reg_base, ndev->irq, priv->can.clock.freq, priv->tx_max);

	return 0;

err_disableclks:
	free_candev(ndev);
	return ret;
}
EXPORT_SYMBOL(phytium_canfd_register);

/**
 * phytium_canfd_unregister - Unregister the device after releasing the resources
 * @pdev:	Handle to the platform device structure
 *
 * This function frees all the resources allocated to the device.
 * Return: 0 always
 */
int phytium_canfd_unregister(struct net_device *ndev)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);

	unregister_candev(ndev);
	netif_napi_del(&priv->napi);
	free_candev(ndev);
	return 0;
}
EXPORT_SYMBOL(phytium_canfd_unregister);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cheng Quan<chengquan@phytium.com.cn>");
MODULE_DESCRIPTION("Phytium CAN Controller");
