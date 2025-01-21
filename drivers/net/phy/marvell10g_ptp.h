// SPDX-License-Identifier: GPL-2.0+
/*
 * Marvell 10G 88x3540 PHY PTP Hardware Clock driver
 * and TAI timestamper.
 */
#include <linux/mii_timestamper.h>
#include <linux/ptp_clock_kernel.h>

struct phy_device;
struct ptp_clock;
struct mii_timestamper;

struct mv3540_ptp_priv {
	struct phy_device *phydev;
        struct mii_timestamper mii_ts;
	struct ptp_clock_info  ptp_clock_info;

        struct ptp_clock        *ptp_clock;

	struct sk_buff_head tx_skb_queue;

	int hwts_tx_en;
	int hwts_rx_en;
	int ptp_transport;
	int ptp_version;
};

int mv3540_ptp_probe(struct phy_device *phydev, struct mv3540_ptp_priv *priv);
int mv3540_ptp_config_init(struct mv3540_ptp_priv *priv);
void mv3540_ptp_remove(struct mv3540_ptp_priv *priv);

