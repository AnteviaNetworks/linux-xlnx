// SPDX-License-Identifier: GPL-2.0-only
/*
 * Antevia Axi Ethernet device driver add-on for shared DMA
 *
 * Copyright (c) 2023 - 2023 Antevia Networks, Ltd. All rights reserved.
 *
 * This is a driver for the Xilinx Axi Ethernet using shared AXI-MCDMA
 *
 * TODO:
 */

#include <linux/clk.h>
#include <linux/circ_buf.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/phy.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/iopoll.h>
#include <linux/ptp_classify.h>
#include <linux/net_tstamp.h>
#include <linux/random.h>
#include <net/sock.h>
#include <linux/xilinx_phy.h>
#include <linux/clk.h>

#include "xilinx_axienet.h"

static DEFINE_SPINLOCK(list_lock);
static struct axienet_local *head = NULL;
static size_t axienet_local_count = 0;

static int axienet_count_macs(void)
{
	return axienet_local_count;
}

static int axienet_all_macs_loaded(void)
{
	struct axienet_local *mac;
	int loaded = 1;
	for(mac = head; mac != NULL; mac = mac->next)
		loaded &= mac->state == ST_LOADED;
	return loaded;
}

static void axienet_disable_channel_interrupts(struct axienet_dma_q *q)
{
	u32 cr;
	cr = axienet_dma_in32(q, XMCDMA_CHAN_CR_OFFSET(q->chan_id));
	/* Disable coalesce, delay timer and error interrupts */
	cr &= (~XMCDMA_IRQ_ALL_MASK);
	/* Finally write to the Tx channel control register */
	axienet_dma_out32(q, XMCDMA_CHAN_CR_OFFSET(q->chan_id), cr);

	cr = axienet_dma_in32(q, XMCDMA_CHAN_CR_OFFSET(q->chan_id) +
			      q->rx_offset);
	/* Disable coalesce, delay timer and error interrupts */
	cr &= (~XMCDMA_IRQ_ALL_MASK);
	/* write to the Rx channel control register */
	axienet_dma_out32(q, XMCDMA_CHAN_CR_OFFSET(q->chan_id) +
			  q->rx_offset, cr);
}

static void axienet_reset_other_mac(struct axienet_local *mac)
{
	int i;
	switch(mac->state)
	{
	case ST_UNLOADED:
	case ST_LOADED:
		break;
	case ST_OPENED:
		for_each_rx_dma_queue(mac, i) {
			axienet_disable_channel_interrupts(mac->dq[i]);
			tasklet_schedule(&mac->dma_err_tasklet[i]);
		}
		mac->state = ST_RESET;
		break;
	case ST_ERROR:
	case ST_CLOSED:
		mac->state = ST_LOADED;
		break;
	case ST_RESET:
		break;
	}
}

static void axienet_reset_all_other_macs(struct axienet_local *lp)
{
	struct axienet_local *mac;
	for(mac = head; mac != NULL; mac = mac->next)
		if(mac != lp)
			axienet_reset_other_mac(mac);
}

void axienet_shared_mcdma_mac_add(struct axienet_local *lp)
{       
	struct axienet_local **tail;
	lp->next = NULL;
	spin_lock(&list_lock);
	for(tail = &head;
		*tail != lp && *tail != NULL;
		tail = &((*tail)->next));
	if(*tail == NULL)
	{
		*tail = lp;
		lp->state = ST_LOADED;
		++axienet_local_count;
	}
	spin_unlock(&list_lock);
}

void axienet_shared_mcdma_mac_remove(struct axienet_local *lp)
{
	struct axienet_local **tail;
	spin_lock(&list_lock);
	for(tail = &head;
		*tail != lp && *tail != NULL;
		tail = &((*tail)->next));
	if(*tail != NULL)
	{
		*tail = lp->next;
		lp->next = NULL;
		lp->state = ST_UNLOADED;
		--axienet_local_count;
	}
	spin_unlock(&list_lock);
}

void axienet_shared_mcdma_event(enum axienet_event event,
				struct axienet_local *lp)
{
	spin_lock(&list_lock);
	switch(event)
	{
	case EVT_MAC_OPEN_COMPLETE:
		netdev_info(lp->ndev,
			"MCDMA SM %s event EVT_MAC_OPEN_COMPLETE\n",
			lp->ndev->name);
		lp->state = ST_OPENED;
		break;
	case EVT_DMA_ERROR_RESET_COMPLETE:
		netdev_info(lp->ndev,
			"MCDMA SM %s event EVT_DMA_ERROR_RESET_COMPLETE\n",
			lp->ndev->name);
		lp->state = ST_OPENED;
		break;
	case EVT_MAC_CLOSED:
		netdev_info(lp->ndev,
			"MCDMA SM %s event EVT_MAC_CLOSED\n",
			lp->ndev->name);
		lp->state = ST_CLOSED;
		break;
	case EVT_DMA_ERROR:
		if(lp->state != ST_RESET)
		{
			netdev_info(lp->ndev,
				"MCDMA SM %s event EVT_DMA_ERROR\n",
				lp->ndev->name);
			lp->state = ST_ERROR;
		}
		else
		{
			netdev_info(lp->ndev,
				"MCDMA SM %s event EVT_DMA_ERROR ignored while in RESET state\n",
				lp->ndev->name);
		}
		break;
	}
	spin_unlock(&list_lock);
}

int axienet_shared_mcdma_should_reset(struct axienet_local *lp)
{
	int reset_flg = 0;
	netdev_info(lp->ndev, "MCDMA SM %s checking reset state\n", lp->ndev->name);
	// If Only 1 device - just reset as normal, this is
	// the same as a dedicated MCDMA
	if(axienet_count_macs() < 2) {
		netdev_info(lp->ndev,
			"MCDMA SM %s only one instance\n",
			lp->ndev->name);
		return 1;
	}

	spin_lock(&list_lock);
	// resets will only occur in the LOADED, CLOSED, ERROR and RESET
	// states.
	// UNLOADED - transitory state, do not reset
	// LOADED - only reset if all other known MACs are also in the LOADED
	//          state. (i.e. first one opened resets the DMA)
	// OPENED - reset should never occur in this state, Reset occurs at
	//          the start of the axienet_open() function. At this time the
	//          the state is either LOADED or CLOSED. The OPENED state is
	//          entered at the end of the axienet_open() function.
	//          OPENED state is also re-entered at the end of handling a
	//          DMA error interrupt.
	// ERROR - The state is entered when a DMA error tasklet runs caused
	//         by a DMA error interrupt. The first request to reset in this
	//         state causes a DMA reset. This should clear all other error
	//         interrupts on the other DMA channels. We set all other macs
	//         to the RESET state and request the other DMA channel error
	//         handler tasklets to run to reset the DMA BD queues for the
	//         other MACs.
	// CLOSED - The MAC has been opened, then subsequently closed. To get
	//          the channel working again, the DMA needs resetting. All
	//          other MACs will be placed in the RESET state and the DMA
	//          error handler tasklet will be scheduled to reset the DMA
	//          BD queues for the MAC.
	// RESET - An internal state that the mac enters when this state engine
	//         schedules the DMA error handler tasklet to reset the DMA
	//         BD queues for other MACs. We do not reset the DMA in this
	//         state.

	switch(lp->state)
	{
		case ST_UNLOADED:
			// This state should not be seen as it is transitory
			// prior to device driver being removed.
			netdev_info(lp->ndev,
				"MCDMA SM %s UNLOADED\n",
				lp->ndev->name);
			break;
		case ST_LOADED:
			// Device has been probed, but never opened
			// Or a previous reset occurred in the CLOSED
			// or ERROR states that moved the state back to LOADED
			netdev_info(lp->ndev,
				"MCDMA SM %s LOADED\n",
				lp->ndev->name);
			if(axienet_all_macs_loaded())
			{
				netdev_info(lp->ndev,
					"MCDMA SM %s first channel reset on LOADED\n",
					lp->ndev->name);
				reset_flg = 1;
			}
			break;
		case ST_OPENED:
			// Device is open, we should not get a reset in this
			// state. 
			netdev_info(lp->ndev,
				"MCDMA SM %s OPENED\n",
				lp->ndev->name);
			break;
		case ST_ERROR:
			// Reset the DMA, request the dma error handler run on
			// the other MACs to reset the DMA BD queues
			netdev_info(lp->ndev,
				"MCDMA SM %s ERROR\n",
				lp->ndev->name);
			axienet_reset_all_other_macs(lp);
			reset_flg = 1;
			break;
		case ST_CLOSED:
			// Reset the DMA, request the dma error handler run on
			// the other MACs to reset the DMA BD queues
			netdev_info(lp->ndev,
				"MCDMA SM %s CLOSED\n",
				lp->ndev->name);
			axienet_reset_all_other_macs(lp);
			reset_flg = 1;
			break;
		case ST_RESET:
			netdev_info(lp->ndev,
				"MCDMA SM %s RESET\n",
				lp->ndev->name);
			break;
	}
	spin_unlock(&list_lock);
	return reset_flg;
}
