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

#define PTP_DOMAINNUMBER        24

enum {
	PTP_TYPE_SYNC = 0x00,
	PTP_TYPE_DELAY_REQ,
	PTP_TYPE_PDELAY_REQ,
	PTP_TYPE_PDELAY_RESP,
	PTP_TYPE_FOLLOW_UP = 0x08,
	PTP_TYPE_DELAY_RESP,
	PTP_TYPE_PDELAY_RESP_FOLLOW_UP,
	PTP_TYPE_ANNOUNCE,
	PTP_TYPE_SIGNALING,
	PTP_TYPE_MANAGEMENT,
};

enum transspec {
	TRANSSPEC_IEEE_1588,
	TRANSSPEC_802_1AS,
	TRANSSPEC_EITHER,
};

enum {
	/* PMA/PMD REGISTERS */
	MV_PMAPMD_CR1                         = 0x0000,
	MV_PMAPMD_CR1_SWRST                   = BIT(15),
	/* PTP TOP LEVEL REGISTERS */
	MV_PTP_TOP_CR1                        = 0xd823,
	MV_PTP_TOP_CR1_BYPASS                 = BIT(0),
	MV_PTP_TOP_CR1_SRST                   = BIT(1),
	MV_PTP_TOP_CR1_ONESTEP_INPUT          = BIT(12),
	/* PTP PORT REGISTERS */
	MV_PTP_PORT_CFG_TRANSSPEC             = 0xd900,
	MV_PTP_PORT_CFG_TRANSSPEC_DISTS       = BIT(0),
	MV_PTP_PORT_CFG_TRANSSPEC_802_1AS     = BIT(12),
	MV_PTP_PORT_CFG_TRANSSPEC_DIS_CHECK   = BIT(11),
	MV_PTP_PORT_TRANSSPEC_TRANSSPEC_MASK  = 0xf800,
	MV_PTP_PORT_CFG_ARR_MODE              = 0xd902,
	MV_PTP_PORT_CFG_DEP_INT_ENA           = 0x0002,
	MV_PTP_PORT_CFG_DEP_KEEP_SA           = 0x0020,
	MV_PTP_PORT_CFG_ARR_MODE_IN_PKT       = 0x1000,
	MV_PTP_GLOB_MTYPE_ENA                 = 0xd961,
	MV_PTP_RDPLUS_ENA                     = 0xd96e,
	MV_PTP_ENABLE                         = 0x8000,
	MV_PTP_REGSET_PORT                    = 0x0000,
	MV_PTP_REGSET_TAI                     = 0x0e00,
	MV_PTP_REGSET_GLOBAL                  = 0x0f00,
	MV_PTP_REGSET_MASK                    = 0x7f00,
	MV_PTP_REGADDR_MASK                   = 0x001f,
	MV_PTP_RDPLUS_DATA                    = 0xd96f,
	/*
	 * Start register for atomic
	 * readplus of the 4 departure
	 * timestamp registers
	 */
	MV_PTP_DEP_TS_ISR                     = 0xd910,
	MV_PTP_DEP_TS_VALID                   = 0x0001,
	MV_PTP_DEP_INT_STATUS_NORMAL          = 0x0000,
	MV_PTP_DEP_INT_STATUS_LOST_OVERWRITE  = 0x0002,
	MV_PTP_DEP_INT_STATUS_LOST_DISCARD    = 0x0004,
	MV_PTP_DEP_TS_IS_MASK                 = 0x0006,
	/*
	 * Registers at offset 11,12, and 13
	 * read atomically with readplus operation
	 */
	MV_PTP_DEP_TS_LO                      = 0xd911,
	MV_PTP_DEP_TS_HI                      = 0xd912,
	MV_PTP_DEP_TS_SEQID                   = 0xd913,
	/* PTP GLOBAL REGISTERS */
	MV_PTP_GLOB_CONFIG_UPDATE             = 0xd967,
	MV_PTP_GLOB_CONFIG_UPDATE_WR          = BIT(15),
	MV_PTP_GLOB_CONFIG_UPDATE_MODE_IDX    = 0x0000,
	MV_PTP_GLOB_CONFIG_UPDATE_IDX_MASK    = 0x7f00,
	MV_PTP_GLOB_CONFIG_UPDATE_DATA_MASK   = 0x00ff,
	/* ToD Load point Registers */
	MV_PTP_GLOB_TOD_LOAD_POINT_15_0       = 0xd970,
	MV_PTP_GLOB_TOD_LOAD_POINT_31_16      = 0xd971,
	/* Time Array (ToD) load and capture */
	MV_PTP_GLOB_TIME_ARR                  = 0xd972,
	MV_PTP_GLOB_TIME_ARR_TOD_BUSY         = 0x8000,
	MV_PTP_GLOB_TIME_ARR_TOD_BUSY_MASK    = 0x8000,
	/* Only clean ops defined (resets AccErr) */
	MV_PTP_GLOB_TIME_ARR_OP_STORE_COMP    = 0x0000,
	MV_PTP_GLOB_TIME_ARR_OP_STORE_ALL     = 0x3000,
	MV_PTP_GLOB_TIME_ARR_OP_CAPTURE       = 0x4000,
	MV_PTP_GLOB_TIME_ARR_RESERVED_MASK    = 0x0080,
	MV_PTP_GLOB_TIME_ARR_IDX              = 0x0000,
	MV_PTP_GLOB_TIME_ARR_ACTIVE           = 0x0010,
	MV_PTP_GLOB_TIME_ARR_ACTIVE_MASK      = 0x0010,
	MV_PTP_GLOB_TIME_ARR_DOMAIN           = PTP_DOMAINNUMBER,
	MV_PTP_GLOB_TIME_ARR_DOMAIN_MASK      = 0x00ff,
	/* ToD Registers */
	MV_PTP_GLOB_TOD_NS_15_0               = 0xd973,
	MV_PTP_GLOB_TOD_NS_31_16              = 0xd974,
	MV_PTP_GLOB_TOD_SECS_15_0             = 0xd975,
	MV_PTP_GLOB_TOD_SECS_31_16            = 0xd976,
	MV_PTP_GLOB_TOD_SECS_47_32            = 0xd977,
	/*
	* 1722 timestamp registers
	*/
	MV_PTP_GLOB_1722_NS_15_0              = 0xd978,
	MV_PTP_GLOB_1722_NS_31_16             = 0xd979,
	MV_PTP_GLOB_1722_NS_47_32             = 0xd97a,
	MV_PTP_GLOB_1722_NS_63_48             = 0xd97b,
	/*
	 * Freq. Compensation in units of 465.661 zs
	 * 1zs = 10^-21 seconds
	 */
	MV_PTP_GLOB_TOD_COMP_15_0             = 0xd978,
	MV_PTP_GLOB_TOD_COMP_31_16            = 0xd979,
};

struct mv3540_departure_ts {
	ktime_t hwtstamp;
	u16 seq_id;
	bool found;
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

static int mv3540_config_lock(struct mv3540_ptp_priv *priv, u32 sleep_us, u64 timeout_us)
{
	int ret;
	int data;
	timeout_us = ktime_add_us(ktime_get(), timeout_us);
	for (;;) {
		ret = mv3540_readplus_start(priv->phydev, MV_PTP_REGSET_GLOBAL, MV_PTP_GLOB_CONFIG_UPDATE);
		data = mv3540_readplus(priv->phydev);
		if ((data & MV_PTP_GLOB_CONFIG_UPDATE_WR) == 0)
			break;
		if (timeout_us && ktime_compare(ktime_get(), timeout_us) > 0) {
			(void)mv3540_readplus_end(priv->phydev);
			break;
		}
		if (sleep_us)
			usleep_range((sleep_us >> 2) + 1, sleep_us);
	}
	return (data & MV_PTP_GLOB_CONFIG_UPDATE_WR) == 0 ? 0 : -ETIMEDOUT; \
}

static void mv3540_config_unlock(struct mv3540_ptp_priv *priv)
{
	(void)mv3540_readplus_end(priv->phydev);
}

static int mv3540_get_config(struct mv3540_ptp_priv *priv, int index, u8 *data)
{
	int ret;
	int val = (index & MV_PTP_GLOB_CONFIG_UPDATE_IDX_MASK);
	ret = mv3540_config_lock(priv, 5000, 100000);
	if(ret) {
		phydev_err(priv->phydev, "PTP global config lock failed err=%d\n", ret);
		return ret;
	}
	ret = phy_write_mmd(priv->phydev, MDIO_MMD_PCS, MV_PTP_GLOB_CONFIG_UPDATE, val);
	if (ret)
		goto out;
	ret = phy_read_mmd(priv->phydev, MDIO_MMD_PCS, MV_PTP_GLOB_CONFIG_UPDATE);
	*data = (u8)ret & MV_PTP_GLOB_CONFIG_UPDATE_DATA_MASK;
out:
	mv3540_config_unlock(priv);
	return 0;
}

static int mv3540_set_config(struct mv3540_ptp_priv *priv, int index, u8 data)
{
	int ret;
	int val = MV_PTP_GLOB_CONFIG_UPDATE_WR |
		  (index & MV_PTP_GLOB_CONFIG_UPDATE_IDX_MASK) |
		  (data & MV_PTP_GLOB_CONFIG_UPDATE_DATA_MASK);
	ret = mv3540_config_lock(priv, 5000, 100000);
	if(ret) {
		phydev_err(priv->phydev, "PTP global config lock failed err=%d\n", ret);
		return ret;
	}
	ret = phy_write_mmd(priv->phydev, MDIO_MMD_PCS, MV_PTP_GLOB_CONFIG_UPDATE, val);
	mv3540_config_unlock(priv);
	return ret;
}

static int mv3540_enable_one_step(struct mv3540_ptp_priv *priv, bool one_step) {
	u8 data;
	int ret;

	ret = mv3540_get_config(priv, MV_PTP_GLOB_CONFIG_UPDATE_MODE_IDX, &data);
	if (ret) {
		phydev_err(priv->phydev, "unable to read config for PTP mode: failed err=%d\n", ret);
		return ret;
	}
	if(one_step)
		data |= 0x0004;
	else
		data &= ~0x0004;
	return mv3540_set_config(priv, MV_PTP_GLOB_CONFIG_UPDATE_MODE_IDX, data);
}

static int mv3540_set_rx_filter(struct mv3540_ptp_priv *priv, u16 filter)
{
	/* Enable message types */
	return phy_write_mmd(priv->phydev, MDIO_MMD_PCS,
			    MV_PTP_GLOB_MTYPE_ENA,
			    filter);
}

static int mv3540_set_ptp_transport_specific(struct mv3540_ptp_priv *priv, enum transspec transport)
{
	u16 transspecval = 0;

	switch (transport)
	{
	case TRANSSPEC_IEEE_1588:
		break;
	case TRANSSPEC_802_1AS:
		transspecval |= MV_PTP_PORT_CFG_TRANSSPEC_802_1AS;
		break;
	case TRANSSPEC_EITHER:
		transspecval |= MV_PTP_PORT_CFG_TRANSSPEC_DIS_CHECK;
		break;
	}
	/* Transport Spec = 0x00 - IEEE 1588v2 0x01 - 802.1AS, or disable check */
	return phy_modify_mmd(priv->phydev, MDIO_MMD_PCS,
			     MV_PTP_PORT_CFG_TRANSSPEC,
			     MV_PTP_PORT_TRANSSPEC_TRANSSPEC_MASK,
			     transspecval);
}

static int mv3540_pmd_reset(struct mv3540_ptp_priv *priv)
{
	int val, ret;
	ret = phy_set_bits_mmd(priv->phydev, MDIO_MMD_PMAPMD, MV_PMAPMD_CR1,
				MV_PMAPMD_CR1_SWRST);
	if (ret)
		return ret;
	ret = phy_read_mmd_poll_timeout(priv->phydev, MDIO_MMD_PMAPMD,
					MV_PMAPMD_CR1, val,
					!(val & MV_PMAPMD_CR1_SWRST),
					5000, 100000, true);
	return ret;
}

static int mv3540_ptp_reset(struct mv3540_ptp_priv *priv)
{
	int ret;
	ret = phy_set_bits_mmd(priv->phydev, MDIO_MMD_PCS, MV_PTP_TOP_CR1,
				MV_PTP_TOP_CR1_SRST);
	if (ret)
		return ret;
	ret = phy_clear_bits_mmd(priv->phydev, MDIO_MMD_PCS, MV_PTP_TOP_CR1,
				MV_PTP_TOP_CR1_SRST);
	return ret;
}

static int mv3540_enable_ptp(struct mv3540_ptp_priv *priv)
{
	int ret;

	ret = phy_clear_bits_mmd(priv->phydev, MDIO_MMD_PCS, MV_PTP_TOP_CR1,
			       MV_PTP_TOP_CR1_BYPASS);
	if (ret)
		return ret;
	ret = phy_set_bits_mmd(priv->phydev, MDIO_MMD_PMAPMD, 0xc04a, 0x0001);

	ret = mv3540_pmd_reset(priv);
	if (ret) {
		phydev_err(priv->phydev, "PMD reset failed err=%d\n", ret);
		return ret;
	}
	ret = mv3540_ptp_reset(priv);
	/*
	 * Always enable one-step input control to PTP block when the
	 * PTP block is enabled.
	 */
	ret = phy_set_bits_mmd(priv->phydev, MDIO_MMD_PCS, MV_PTP_TOP_CR1,
			       MV_PTP_TOP_CR1_ONESTEP_INPUT);
	return ret;
}

static int mv3540_disable_ptp(struct mv3540_ptp_priv *priv)
{
	int ret;
	ret = phy_set_bits_mmd(priv->phydev, MDIO_MMD_PCS, MV_PTP_TOP_CR1,
			       MV_PTP_TOP_CR1_BYPASS);
	if (ret)
		return ret;
	ret = phy_clear_bits_mmd(priv->phydev, MDIO_MMD_PMAPMD, 0xc04a, 0x0001);
	if (ret)
		return ret;
	ret = mv3540_pmd_reset(priv);
	if (ret) {
		phydev_err(priv->phydev, "PMD reset failed err=%d\n", ret);
		return ret;
	}
	ret = mv3540_ptp_reset(priv);
	/*
	 * Clear input control to PTP block when the
	 * PTP block is disabled.
	 */
	ret = phy_clear_bits_mmd(priv->phydev, MDIO_MMD_PCS, MV_PTP_TOP_CR1,
			       MV_PTP_TOP_CR1_ONESTEP_INPUT);
	return ret;
}

static int mv3540_start_ptp(struct mv3540_ptp_priv *priv)
{
	return phy_clear_bits_mmd(priv->phydev, MDIO_MMD_PCS,
			     MV_PTP_PORT_CFG_TRANSSPEC,
			     MV_PTP_PORT_CFG_TRANSSPEC_DISTS);
}

static int mv3540_stop_ptp(struct mv3540_ptp_priv *priv)
{
	return phy_set_bits_mmd(priv->phydev, MDIO_MMD_PCS,
				MV_PTP_PORT_CFG_TRANSSPEC,
				MV_PTP_PORT_CFG_TRANSSPEC_DISTS);
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

	if (priv->hwts_rx_en) {
		header = ptp_parse_header(skb, type);
		if (!header)
			return false;
		shhwtstamps = skb_hwtstamps(skb);
		ts_lower = be32_to_cpu(header->reserved2);
		shhwtstamps->hwtstamp = ns_to_ktime(ts_lower);
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
	int err;
	bool start_ptp = false;
	bool one_step = false;
	u16 filter = 0xffff;
	enum transspec transport = TRANSSPEC_IEEE_1588;

	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;

	if (cfg.flags) /* reserved for future extensions */
		return -EINVAL;

	switch (cfg.tx_type) {
	case HWTSTAMP_TX_OFF:
		start_ptp = false;
		break;
	case HWTSTAMP_TX_ON:
		start_ptp = true;
		break;
	case HWTSTAMP_TX_ONESTEP_SYNC:
	case HWTSTAMP_TX_ONESTEP_P2P:
		one_step = true;
		break;
	default:
		return -ERANGE;
	}
	err = mv3540_enable_one_step(priv, one_step);
	if (err) {
		phydev_err(priv->phydev, "setting PTP %s-step failed: err=%d\n", one_step ? "one" : "two", err);
		return -EFAULT;
	}
	if (cfg.rx_filter == HWTSTAMP_FILTER_NONE) {
		filter = 0x0;
	}
	err = mv3540_set_rx_filter(priv, filter);
	if (err) {
		phydev_err(priv->phydev, "setting PTP rx-filter failed: err=%d\n", err);
		return -EFAULT;
	}

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
		transport = TRANSSPEC_802_1AS;
		break;
	case HWTSTAMP_FILTER_ALL:
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		priv->hwts_rx_en = 1;
		priv->ptp_transport = PTP_CLASS_L4 | PTP_CLASS_L2;
		priv->ptp_version = PTP_CLASS_V2;
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		transport = TRANSSPEC_EITHER;
		break;
	default:
		return -ERANGE;
	}
	err = mv3540_set_ptp_transport_specific(priv, transport);
	if (err) {
		phydev_err(priv->phydev, "setting PTP transport specific check failed: err=%d\n", err);
		return -EFAULT;
	}
	if(start_ptp)
		err = mv3540_start_ptp(priv);
	else
		err = mv3540_stop_ptp(priv);
	if (err) {
		phydev_err(priv->phydev, "failed to %s PTP timestamping: err=%d\n",
			start_ptp ? "start" : "stop", err);
		return -EFAULT;
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

	mv3540_readplus_start(priv->phydev, MV_PTP_REGSET_PORT, MV_PTP_DEP_TS_ISR);
	departure_ts[0] = mv3540_readplus(priv->phydev);
	departure_ts[1] = mv3540_readplus(priv->phydev);
	departure_ts[2] = mv3540_readplus(priv->phydev);
	departure_ts[3] = mv3540_readplus(priv->phydev);
	phy_clear_bits_mmd(priv->phydev, MDIO_MMD_PCS, MV_PTP_DEP_TS_ISR,
		MV_PTP_DEP_TS_IS_MASK | MV_PTP_DEP_TS_VALID);
	mv3540_readplus_end(priv->phydev);
	if((departure_ts[0] & MV_PTP_DEP_TS_VALID) == MV_PTP_DEP_TS_VALID) {
		ts_lower = ((u32)departure_ts[2] << 16) | departure_ts[1];
		ts->hwtstamp = ns_to_ktime(ts_lower);
		ts->seq_id = departure_ts[3];
		ts->found = true;
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
	if (ts->found) {
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
	}

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
			phydev_err(priv->phydev, "timed out TX timestamp discarding tag: 0x%x\n", MV_SKB_CB(skb)->seq_id);
			kfree_skb(skb);
		}
	}
}

static int mv3540_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct mv3540_ptp_priv *priv = container_of(ptp, struct mv3540_ptp_priv,
						ptp_clock_info);
	int err = 0;

	phydev_info(priv->phydev, "not implemented - adjust PHC time delta %lldns\n", delta);

	return err;
}

static int mv3540_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{	
	struct mv3540_ptp_priv *priv = container_of(ptp, struct mv3540_ptp_priv,
						ptp_clock_info);
	int err = 0;

	phydev_info(priv->phydev, "not implemented - adjust PHC time: %ld scaled PPM in units of 2^-16\n", scaled_ppm);

	return err;
}

static int mv3540_gettime(struct ptp_clock_info *ptp,
		       struct timespec64 *ts)
{
	struct mv3540_ptp_priv *priv = container_of(ptp, struct mv3540_ptp_priv,
						ptp_clock_info);
	int err = 0;

	phydev_info(priv->phydev, "not implemented - get PHC get time\n");

	return err;
}

static int mv3540_settime(struct ptp_clock_info *ptp, const struct timespec64 *ts)
{
	struct mv3540_ptp_priv *priv = container_of(ptp, struct mv3540_ptp_priv,
						ptp_clock_info);
	int err = 0;

	phydev_info(priv->phydev, "not implemented - set PHC time to %lld.%09ld secs\n", ts->tv_sec, ts->tv_nsec);

	return err;
}

static long mv3540_do_aux_work(struct ptp_clock_info *ptp)
{
	struct mv3540_ptp_priv *priv = container_of(ptp, struct mv3540_ptp_priv,
						ptp_clock_info);
	struct mv3540_departure_ts ts = {
		.found = false,
	};
	bool reschedule = false;

	while (!skb_queue_empty_lockless(&priv->tx_skb_queue) && !reschedule) {
		if (!mv3540_read_departure_ts(priv, &ts)) {
			reschedule = true;
		}
		mv3540_ptp_match_tstamp(priv, &ts);
	}

	return reschedule ? 1 : -1;
}

static struct ptp_clock_info mv3540_ptp_clock_info = {
	.owner          = THIS_MODULE,
	.name           = "mv88x3540",
	.max_adj        = 2147483647,
	.n_alarm        = 0,
	.n_ext_ts       = 0,
	.n_per_out      = 0,
	.n_pins         = 0,
	.pps            = 0,
	.adjtime        = mv3540_adjtime,
	.adjfine        = mv3540_adjfine,
	.gettime64	= mv3540_gettime,
	.settime64      = mv3540_settime,
	.do_aux_work	= mv3540_do_aux_work,
};

int mv3540_ptp_probe(struct phy_device *phydev, struct mv3540_ptp_priv *priv)
{
	int ret;

	priv->phydev = phydev;
	priv->ptp_clock_info = mv3540_ptp_clock_info;
	snprintf(priv->ptp_clock_info.name, sizeof(priv->ptp_clock_info.name), "%s@%d",
		mv3540_ptp_clock_info.name,
		phydev->mdio.addr);
	
	priv->ptp_clock = ptp_clock_register(&priv->ptp_clock_info,
						&phydev->mdio.dev);
	if (IS_ERR(priv->ptp_clock)) {
		ret = PTR_ERR(priv->ptp_clock);
		phydev_err(phydev, "Failed to register ptp clock\n");
		return ret;
	}
	phydev_info(phydev, "PHY PTP clock registered successfully index: %d\n", ptp_clock_index(priv->ptp_clock));
	priv->mii_ts.rxtstamp = mv3540_rxtstamp,
	priv->mii_ts.txtstamp = mv3540_txtstamp,
	priv->mii_ts.hwtstamp = mv3540_hwtstamp,
	priv->mii_ts.ts_info = mv3540_ts_info,
	priv->mii_ts.device = &phydev->mdio.dev;
	phydev->mii_ts = &priv->mii_ts;

	// SKB queues
	skb_queue_head_init(&priv->tx_skb_queue);

	/* Disable timestamping on this port */
	ret = mv3540_disable_ptp(priv);
	if (ret)
		return ret;

	phydev_info(phydev, "PHY PTP probed successfully\n");
	return 0;
}

int mv3540_ptp_config_init(struct mv3540_ptp_priv *priv)
{
	int ret;
	struct timespec64 ts;
	struct timespec64 ts2 = {0};

	/* Enable timestamping on this port */
	ret = mv3540_enable_ptp(priv);
	if (ret)
		return ret;

	/* Initialize current time */
	ts = ns_to_timespec64(ktime_to_ns(ktime_get_real()));
	mv3540_settime(&priv->ptp_clock_info, &ts);

	mv3540_gettime(&priv->ptp_clock_info, &ts2);
	phydev_info(priv->phydev, "get init PHC time to %lld.%09ld secs\n", ts2.tv_sec, ts2.tv_nsec);

	/*
	 * Enable departure timestamps captured in registers
	 *        Arrival timestamps placed in packet
	 */
	return phy_write_mmd(priv->phydev, MDIO_MMD_PCS,
			    MV_PTP_PORT_CFG_ARR_MODE,
			    MV_PTP_PORT_CFG_DEP_INT_ENA |
			    MV_PTP_PORT_CFG_DEP_KEEP_SA |
			    MV_PTP_PORT_CFG_ARR_MODE_IN_PKT);
	return 0;
}

void mv3540_ptp_remove(struct mv3540_ptp_priv *priv)
{
	if (priv->ptp_clock != NULL) {
		ptp_clock_unregister(priv->ptp_clock);
		priv->ptp_clock = NULL;
	}
	mv3540_disable_ptp(priv);
}
