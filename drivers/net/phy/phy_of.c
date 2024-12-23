#include <linux/device.h>
#include <linux/err.h>
#include <linux/phy.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include "phy-core.h"

#if IS_ENABLED(CONFIG_OF_DYNAMIC)

/* Extract the clause 22 phy ID from the compatible string of the form
 * ethernet-phy-idAAAA.BBBB */
static int of_get_phy_id(struct device_node *device, u32 *phy_id)
{
	struct property *prop;
	const char *cp;
	unsigned int upper, lower;

	of_property_for_each_string(device, "compatible", prop, cp) {
		if (sscanf(cp, "ethernet-phy-id%4x.%4x", &upper, &lower) == 2) {
			*phy_id = ((upper & 0xFFFF) << 16) | (lower & 0xFFFF);
			return 0;
		}
	}
	return -EINVAL;
}

static struct mii_timestamper *of_find_mii_timestamper(struct device_node *node)
{
	struct of_phandle_args arg;
	int err;

	err = of_parse_phandle_with_fixed_args(node, "timestamper", 1, 0, &arg);

	if (err == -ENOENT)
		return NULL;
	else if (err)
		return ERR_PTR(err);

	if (arg.args_count != 1)
		return ERR_PTR(-EINVAL);

	return register_mii_timestamper(arg.np, arg.args[0]);
}

static int of_mdiobus_register_phy(struct mii_bus *mdio,
				    struct device_node *child, u32 addr)
{
	struct mii_timestamper *mii_ts;
	struct phy_device *phy;
	bool is_c45;
	int rc;
	u32 phy_id;

	mii_ts = of_find_mii_timestamper(child);
	if (IS_ERR(mii_ts))
		return PTR_ERR(mii_ts);

	is_c45 = of_device_is_compatible(child,
					 "ethernet-phy-ieee802.3-c45");

	if (!is_c45 && !of_get_phy_id(child, &phy_id))
		phy = phy_device_create(mdio, addr, phy_id, 0, NULL);
	else
		phy = get_phy_device(mdio, addr, is_c45);
	if (IS_ERR(phy)) {
		if (mii_ts)
			unregister_mii_timestamper(mii_ts);
		return PTR_ERR(phy);
	}

	rc = of_mdiobus_phy_device_register(mdio, phy, child, addr);
	if (rc) {
		if (mii_ts)
			unregister_mii_timestamper(mii_ts);
		phy_device_free(phy);
		return rc;
	}

	/* phy->mii_ts may already be defined by the PHY driver. A
	 * mii_timestamper probed via the device tree will still have
	 * precedence.
	 */
	if (mii_ts)
		phy->mii_ts = mii_ts;

	return 0;
}

static int of_phy_notify(struct notifier_block *nb, unsigned long action,
                         void *arg)
{
	struct of_reconfig_data *rd = arg;
	struct mii_bus *mdio;
	struct phy_device *phy;
	int rc, addr;

	switch (of_reconfig_get_state_change(action, rd)) {
	case OF_RECONFIG_CHANGE_ADD:
		if(!of_mdiobus_child_is_phy(rd->dn))
			break;
		mdio = of_mdio_find_bus(rd->dn->parent);
		if(mdio == NULL)
			break;
		addr = of_mdio_parse_addr(&mdio->dev, rd->dn);
		if(addr < 0) {
			dev_info(&mdio->dev, "phy %pOFfp missing address - add reg property\n",
						 rd->dn);
			put_device(&mdio->dev);
			break;
		}
		rc = of_mdiobus_register_phy(mdio, rd->dn, addr);
		if (rc == -ENODEV) {
			dev_err(&mdio->dev,
				"MDIO device at address %d is missing.\n",
				addr);
			put_device(&mdio->dev);
			break;
		}
		if (of_node_test_and_set_flag(rd->dn, OF_POPULATED)) {
			put_device(&mdio->dev);
			break;
		}
		dev_info(&mdio->dev, "add phy %pOFf\n", rd->dn);
		put_device(&mdio->dev);
		return NOTIFY_OK;

	case OF_RECONFIG_CHANGE_REMOVE:
		/* already depopulated? */
		if (!of_node_check_flag(rd->dn, OF_POPULATED))
			break;
		if(!of_mdiobus_child_is_phy(rd->dn))
			break;
		phy = of_phy_find_device(rd->dn);
		if(phy == NULL)
			break;
		if(phy->mdio.bus == NULL) {
			put_device(&phy->mdio.dev);
			break;
		}
		if(phy->mdio.bus->dev.of_node != rd->dn->parent) {
			put_device(&phy->mdio.dev);
			break;
		}
		phy_device_remove(phy);
		dev_info(&phy->mdio.dev, "remove phy %pOFf\n", rd->dn);
		put_device(&phy->mdio.dev);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

struct notifier_block phy_of_notifier = {
        .notifier_call = of_phy_notify,
};
#endif /* CONFIG_OF_DYNAMIC */

void phy_of_notifier_register(void)
{
#if IS_ENABLED(CONFIG_OF_DYNAMIC)
	WARN_ON(of_reconfig_notifier_register(&phy_of_notifier));
#endif /* CONFIG_OF_DYNAMIC */
}

void phy_of_notifier_unregister(void)
{
#if IS_ENABLED(CONFIG_OF_DYNAMIC)
	WARN_ON(of_reconfig_notifier_unregister(&phy_of_notifier));
#endif /* CONFIG_OF_DYNAMIC */
}

