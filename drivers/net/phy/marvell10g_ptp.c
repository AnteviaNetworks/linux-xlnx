// SPDX-License-Identifier: GPL-2.0+
/*
 * Marvell 10G 88x3540 PHY PTP Hardware Clock driver
 * and TAI timestamper.
 */
#define DEBUG
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/net_tstamp.h>
#include <linux/marvell_phy.h>
#include <linux/phy.h>
#include <linux/ptp_classify.h>
#include <linux/ptp_clock_kernel.h>                                                                  
#include "marvell10g_ptp.h"


#define MV_SKB_CB(skb)		((struct mv3540_ptp_skb_cb *)(skb)->cb)
#define SKB_TS_TIMEOUT		10			/* jiffies */

/*
 * Missing from ptp_classify.h in v5.10 kernel
 */
#define PTP_MSGTYPE_SYNC        0x0
#define PTP_MSGTYPE_DELAY_REQ   0x1
#define PTP_MSGTYPE_PDELAY_REQ  0x2
#define PTP_MSGTYPE_PDELAY_RESP 0x3

enum {
	MV_PTP_RDPLUS_ENA       = 0xd96e,
	MV_PTP_ENABLE           = 0x8000,
	MV_PTP_REGSET_PORT      = 0x0000,
	MV_PTP_REGSET_TAI       = 0x6000,
	MV_PTP_REGSET_GLOBAL    = 0x7000,
	MV_PTP_REGSET_MASK      = 0x7000,
	MV_PTP_REGADDR_MASK     = 0x001f,
	MV_PTP_RDPLUS_DATA      = 0xd96f,
	/*
	 * Start register for atomic
	 * readplus of the 4 departure
	 * timestamp registers
	 */
	MV_PTP_DEP_TS_ISR       = 0xd910,
	MV_PTP_DEP_TS_IS_MASK   = 0x0006,
	MV_PTP_DEP_TS_VALID     = 0x0001,
	MV_PTP_DEP_TS_VALID_MASK= 0x0001,
	/*
	 * Registers at offset 11,12, and 13
	 * read atomically with readplus operation
	 */
	MV_PTP_DEP_TS_LO        = 0xd911,
	MV_PTP_DEP_TS_HI        = 0xd912,
	MV_PTP_DEP_TS_SEQID     = 0xd913,
};

struct mv3540_departure_ts {
	ktime_t hwtstamp;
	u16 seq_id;
};

struct mv3540_ptp_skb_cb {
	unsigned long timeout;
	u16 seq_id;
	bool discard;
};

static int mv3540_readplus_start(struct phy_device *phydev, int regset, u16 reg)
{
	u16 val = MV_PTP_ENABLE | (regset & MV_PTP_REGSET_MASK) | (reg & MV_PTP_REGADDR_MASK);
	return phy_write_mmd(phydev, MDIO_MMD_PCS, MV_PTP_RDPLUS_ENA, val);
}

static int mv3540_readplus(struct phy_device *phydev)
{
	return phy_read_mmd(phydev, MDIO_MMD_PCS, MV_PTP_RDPLUS_DATA);
}

static int mv3540_readplus_end(struct phy_device *phydev)
{
	u16 val = 0;
	return phy_write_mmd(phydev, MDIO_MMD_PCS, MV_PTP_RDPLUS_ENA, val);
}

/*
 * MV88X3540 Timestamping Interface
 */

static bool mv3540_rxtstamp(struct mii_timestamper *mii_ts, struct sk_buff *skb, int type)
{
        struct mv3540_ptp_priv *priv = container_of(mii_ts, struct mv3540_ptp_priv,
	                                        mii_ts);
	struct ptp_header *header;
	struct skb_shared_hwtstamps *shhwtstamps;
	u32 ts_lower;
	u64 time64;

	if (priv->hwts_rx_en) {
		header = ptp_parse_header(skb, type);
		if (!header)
			return false;
		shhwtstamps = skb_hwtstamps(skb);
		ts_lower = be32_to_cpu(header->reserved2);
		if (ts_lower < priv->ts_arr_lower)
			++priv->ts_arr_upper;
		priv->ts_arr_lower = ts_lower;
		time64 = ((u64)priv->ts_arr_upper << 32) | priv->ts_arr_lower;
		shhwtstamps->hwtstamp = ns_to_ktime(time64);
	}
	return false;
}

static void mv3540_txtstamp(struct mii_timestamper *mii_ts, struct sk_buff *skb, int type)
{
        struct mv3540_ptp_priv *priv = container_of(mii_ts, struct mv3540_ptp_priv,
	                                        mii_ts);
	struct ptp_header *hdr;
	bool discard = false;
	int msgtype;

	hdr = ptp_parse_header(skb, type);
	if (!hdr)
		goto out;
	msgtype = ptp_get_msgtype(hdr, type);

	switch (priv->hwts_tx_en)
	{
	case HWTSTAMP_TX_ONESTEP_P2P:
		if (msgtype == PTP_MSGTYPE_PDELAY_RESP)
			discard = true;
		fallthrough;
	case HWTSTAMP_TX_ONESTEP_SYNC:
		if (msgtype == PTP_MSGTYPE_SYNC)
			discard = true;
		fallthrough;
	case HWTSTAMP_TX_ON:
		MV_SKB_CB(skb)->timeout = jiffies + SKB_TS_TIMEOUT;
		MV_SKB_CB(skb)->seq_id = be16_to_cpu(hdr->sequence_id);
		MV_SKB_CB(skb)->discard = discard;
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		skb_queue_tail(&priv->tx_skb_queue, skb);
		ptp_schedule_worker(priv->ptp_clock, 0);
		return;
	default:
		break;
	}

out:
	kfree_skb(skb);
}

static int mv3540_hwtstamp(struct mii_timestamper *mii_ts, struct ifreq *ifr)
{
        struct mv3540_ptp_priv *priv = container_of(mii_ts, struct mv3540_ptp_priv,
	                                        mii_ts);

	struct hwtstamp_config cfg;

	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;

	if (cfg.flags) /* reserved for future extensions */
		return -EINVAL;

	if (cfg.tx_type < 0 || cfg.tx_type > HWTSTAMP_TX_ONESTEP_SYNC)
		return -ERANGE;

	priv->hwts_tx_en = cfg.tx_type;

	switch (cfg.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		priv->hwts_rx_en = 0;
		priv->ptp_transport = 0;
		priv->ptp_version = 0;
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
		priv->hwts_rx_en = 1;
		priv->ptp_transport = PTP_CLASS_L4;
		priv->ptp_version = PTP_CLASS_V1;
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_EVENT;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
		priv->hwts_rx_en = 1;
		priv->ptp_transport = PTP_CLASS_L4;
		priv->ptp_version = PTP_CLASS_V2;
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
		priv->hwts_rx_en = 1;
		priv->ptp_transport = PTP_CLASS_L2;
		priv->ptp_version = PTP_CLASS_V2;
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V2_L2_EVENT;
		break;
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		priv->hwts_rx_en = 1;
		priv->ptp_transport = PTP_CLASS_L4 | PTP_CLASS_L2;
		priv->ptp_version = PTP_CLASS_V2;
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		break;
	default:
		return -ERANGE;
	}
	
	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}

static int mv3540_ts_info(struct mii_timestamper *mii_ts, struct ethtool_ts_info *info)
{
        struct mv3540_ptp_priv *priv = container_of(mii_ts, struct mv3540_ptp_priv,
	                                        mii_ts);

	info->so_timestamping =
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;

	info->phc_index = ptp_clock_index(priv->ptp_clock);
	info->tx_types =
		(1 << HWTSTAMP_TX_OFF) |
		(1 << HWTSTAMP_TX_ON) ;
      	info->rx_filters =
                (1 << HWTSTAMP_FILTER_NONE) |
                (1 << HWTSTAMP_FILTER_PTP_V2_L2_EVENT) |
                (1 << HWTSTAMP_FILTER_PTP_V2_L4_EVENT);
	return 0;
}

/*
 * MV88X3540 PTP Hardware Clock Interface
 */

static int mv3540_read_departure_ts (struct mv3540_ptp_priv *priv, struct mv3540_departure_ts *ts)
{
	u16 departure_ts[4];
	u32 ts_lower;
	u64 time64;

	mv3540_readplus_start(priv->phydev, MV_PTP_REGSET_PORT, MV_PTP_DEP_TS_ISR);
	departure_ts[0] = mv3540_readplus(priv->phydev);
	departure_ts[1] = mv3540_readplus(priv->phydev);
	departure_ts[2] = mv3540_readplus(priv->phydev);
	departure_ts[3] = mv3540_readplus(priv->phydev);
	mv3540_readplus_end(priv->phydev);
	phy_clear_bits_mmd(priv->phydev, MDIO_MMD_PCS, MV_PTP_DEP_TS_ISR,
		MV_PTP_DEP_TS_VALID_MASK);
	if((departure_ts[0] & MV_PTP_DEP_TS_IS_MASK) == 0x0000 &&
		(departure_ts[0] & MV_PTP_DEP_TS_VALID_MASK) == MV_PTP_DEP_TS_VALID) {
		ts_lower = ((u32)departure_ts[2] << 16) | departure_ts[1];
		if (ts_lower < priv->ts_dep_lower)
			++priv->ts_dep_upper;
		priv->ts_dep_lower = ts_lower;
		time64 = ((u64)priv->ts_dep_upper << 32) | priv->ts_dep_lower;
		ts->hwtstamp = ns_to_ktime(time64);
		ts->seq_id = departure_ts[3];
		return true;
	}
	return false;
}

static void mv3540_ptp_match_tstamp(struct mv3540_ptp_priv *priv,
				 struct mv3540_departure_ts *ts)
{
	struct skb_shared_hwtstamps hwts;
	struct sk_buff *skb, *ts_skb;
	unsigned long flags;
	bool first = false;

	ts_skb = NULL;
	spin_lock_irqsave(&priv->tx_skb_queue.lock, flags);
	skb_queue_walk(&priv->tx_skb_queue, skb) {
		if (MV_SKB_CB(skb)->seq_id == ts->seq_id) {
			first = skb_queue_is_first(&priv->tx_skb_queue, skb);
			__skb_unlink(skb, &priv->tx_skb_queue);
			ts_skb = skb;
			break;
		}
	}
	spin_unlock_irqrestore(&priv->tx_skb_queue.lock, flags);

	/* TX captures one-step packets, discard them if needed. */
	if (ts_skb) {
		if (MV_SKB_CB(ts_skb)->discard) {
			kfree_skb(ts_skb);
		} else {
			memset(&hwts, 0, sizeof(hwts));
			hwts.hwtstamp = ts->hwtstamp;
			skb_complete_tx_timestamp(ts_skb, &hwts);
		}
	}

	/* not first match, try and expire entries */
	if (!first) {
		while ((skb = skb_dequeue(&priv->tx_skb_queue))) {
			if (!time_after(jiffies, MV_SKB_CB(skb)->timeout)) {
				skb_queue_head(&priv->tx_skb_queue, skb);
				break;
			}
			kfree_skb(skb);
		}
	}
}

static int mv3540_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
        struct mv3540_ptp_priv *priv = container_of(ptp, struct mv3540_ptp_priv,
	                                        ptp_clock_info);
	int err = 0;

        phydev_info(priv->phydev, "adjust PHC time: %lld ns\n", delta);

	return err;
}

static int mv3540_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{	
        struct mv3540_ptp_priv *priv = container_of(ptp, struct mv3540_ptp_priv,
	                                        ptp_clock_info);
	int err = 0;

        phydev_info(priv->phydev, "adjust PHC time: %ld scaled PPM in units of 2^-16\n", scaled_ppm);

	return err;
}

static int mv3540_gettimex(struct ptp_clock_info *ptp,
		       struct timespec64 *ts,
		       struct ptp_system_timestamp *sts)
{
        struct mv3540_ptp_priv *priv = container_of(ptp, struct mv3540_ptp_priv,
	                                        ptp_clock_info);
	int err = 0;

        phydev_info(priv->phydev, "get PHC time (with bounding system timestamps)\n");

	return err;
}

static int mv3540_settime(struct ptp_clock_info *ptp, const struct timespec64 *ts)
{
        struct mv3540_ptp_priv *priv = container_of(ptp, struct mv3540_ptp_priv,
	                                        ptp_clock_info);
	int err = 0;

        phydev_info(priv->phydev, "set PHC time to %lld%09ld secs\n", ts->tv_sec, ts->tv_nsec);

	return err;
}

static long mv3540_do_aux_work(struct ptp_clock_info *ptp)
{
        struct mv3540_ptp_priv *priv = container_of(ptp, struct mv3540_ptp_priv,
	                                        ptp_clock_info);
	struct mv3540_departure_ts ts;
	bool reschedule = false;

	while (!skb_queue_empty_lockless(&priv->tx_skb_queue)) {
		if (!mv3540_read_departure_ts(priv, &ts)) {
			reschedule = true;
			break;
		}
		mv3540_ptp_match_tstamp(priv, &ts);
	}

	return reschedule ? 1 : -1;
}
static struct ptp_clock_info mv3540_ptp_clock_info = {
	.owner          = THIS_MODULE,
	.name           = "MV88X3540_PHC",
	.max_adj        = 100000000,
	.n_alarm        = 0,
	.n_pins         = 0,
	.n_ext_ts       = 0,
	.n_per_out      = 0,
	.pps            = 0,
	.adjtime        = mv3540_adjtime,
	.adjfine        = mv3540_adjfine,
	.gettimex64	= mv3540_gettimex,
	.settime64      = mv3540_settime,
	.do_aux_work	= mv3540_do_aux_work,
};

int mv3540_ptp_probe(struct phy_device *phydev, struct mv3540_ptp_priv *priv)
{
	int ret;

	priv->phydev = phydev;
	priv->ptp_clock_info = mv3540_ptp_clock_info;
	priv->ptp_clock = ptp_clock_register(&priv->ptp_clock_info,
						&phydev->mdio.dev);
	if (IS_ERR(priv->ptp_clock)) {
		ret = PTR_ERR(priv->ptp_clock);
		phydev_err(phydev, "Failed to register ptp clock\n");
		return ret;
	}
	priv->mii_ts.rxtstamp = mv3540_rxtstamp,
	priv->mii_ts.txtstamp = mv3540_txtstamp,
	priv->mii_ts.hwtstamp = mv3540_hwtstamp,
	priv->mii_ts.ts_info = mv3540_ts_info,
	priv->mii_ts.device = &phydev->mdio.dev;
	phydev->mii_ts = &priv->mii_ts;

	// SKB queues
	skb_queue_head_init(&priv->tx_skb_queue);

	return 0;
}

int mv3540_ptp_config_init(struct mv3540_ptp_priv *priv)
{
	return 0;
}

void mv3540_ptp_remove(struct mv3540_ptp_priv *priv)
{
	if (priv->ptp_clock != NULL) {
		ptp_clock_unregister(priv->ptp_clock);
		priv->ptp_clock = NULL;
	}
}
