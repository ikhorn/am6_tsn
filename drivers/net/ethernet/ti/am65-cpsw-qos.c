// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments K3 AM65 Ethernet QoS submodule
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *
 * quality of service module includes:
 * Enhanced Scheduler Traffic (EST - P802.1Qbv/D2.2)
 */

#include "am65-cpsw-qos.h"
#include "am65-cpsw-nuss.h"
#include "am65-cpts.h"
#include <linux/pm_runtime.h>
#include <linux/time.h>

#define AM65_CPSW_REG_CTL			0x004
#define AM65_CPSW_PN_REG_CTL			0x004

/* AM65_CPSW_REG_CTL register fields */
#define AM65_CPSW_CTL_EST_EN			BIT(18)

/* AM65_CPSW_PN_REG_CTL register fields */
#define AM65_CPSW_PN_CTL_EST_PORT_EN		BIT(17)

#if IS_ENABLED(CONFIG_NET_SCH_TAPRIO)

static int am65_cpsw_port_est_enabled(struct am65_cpsw_port *port)
{
	return port->qbv.est_oper || port->qbv.est_admin;
}

static void am65_cpsw_est_enable(struct am65_cpsw_common *common, int enable)
{
	u32 val;

	val = readl(common->cpsw_base + AM65_CPSW_REG_CTL);

	if (enable)
		val |= AM65_CPSW_CTL_EST_EN;
	else
		val &= ~AM65_CPSW_CTL_EST_EN;

	writel(val, common->cpsw_base + AM65_CPSW_REG_CTL);
	common->est_enabled = enable;
}

static void am65_cpsw_port_est_enable(struct am65_cpsw_port *port, int enable)
{
	u32 val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_CTL);
	if (enable)
		val |= AM65_CPSW_PN_CTL_EST_PORT_EN;
	else
		val &= ~AM65_CPSW_PN_CTL_EST_PORT_EN;

	writel(val, port->port_base + AM65_CPSW_PN_REG_CTL);
}

static void am65_cpsw_est_set(struct net_device *ndev, int enable)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = port->common;
	int common_enable = 0;
	int i;

	am65_cpsw_port_est_enable(port, enable);

	for (i = 0; i < common->port_num; i++)
		common_enable |= am65_cpsw_port_est_enabled(&common->ports[i]);

	common_enable |= enable;
	am65_cpsw_est_enable(common, common_enable);
}

/* This update is supposed to be used in any routine before getting real state
 * of oper -> admin transition, particularly it's supposed to be used in some
 * generic routine for providing real state to Taprio Qdisc.
 */
static void am65_cpsw_est_update_state(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u64 cur_time;

	if (!port->qbv.est_admin)
		return;

	cur_time = am65_cpts_ns_gettime(port->common->cpts);

	if (cur_time < port->qbv.est_admin->taprio.base_time)
		return;

	if (port->qbv.est_oper)
		devm_kfree(&ndev->dev, port->qbv.est_oper);

	port->qbv.est_oper = port->qbv.est_admin;
	port->qbv.est_admin = NULL;
}

static int am65_cpsw_configure_taprio(struct net_device *ndev,
				      struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = port->common;

	am65_cpsw_est_update_state(ndev);

	if (est_new->taprio.enable && common->pf_p0_rx_ptype_rrobin) {
		netdev_err(ndev,
			   "p0-rx-ptype-rrobin flag conflicts with taprio qdisc\n");
		return -EINVAL;
	}

	am65_cpsw_est_set(ndev, est_new->taprio.enable);

	return 0;
}

static void am65_cpsw_cp_taprio(struct tc_taprio_qopt_offload *from,
				struct tc_taprio_qopt_offload *to)
{
	int i;

	*to = *from;
	for (i = 0; i < from->num_entries; i++)
		to->entries[i] = from->entries[i];
}

int am65_cpsw_set_taprio(struct net_device *ndev, void *type_data)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct tc_taprio_qopt_offload *taprio = type_data;
	struct am65_cpsw_common *common = port->common;
	struct am65_cpsw_est *est_new;
	size_t size;
	int ret;

	size = sizeof(struct tc_taprio_sched_entry) * taprio->num_entries +
	       sizeof(struct am65_cpsw_est);

	ret = pm_runtime_get_sync(common->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(common->dev);
		return ret;
	}

	est_new = devm_kzalloc(&ndev->dev, size, GFP_KERNEL);
	if (!est_new) {
		ret = -ENOMEM;
		goto err;
	}

	am65_cpsw_cp_taprio(taprio, &est_new->taprio);
	ret = am65_cpsw_configure_taprio(ndev, est_new);
	if (!ret) {
		if (port->qbv.est_admin)
			devm_kfree(&ndev->dev, port->qbv.est_admin);

		if (!taprio->enable) {
			if (port->qbv.est_oper)
				devm_kfree(&ndev->dev, port->qbv.est_oper);

			devm_kfree(&ndev->dev, est_new);
			port->qbv.est_oper = NULL;
			port->qbv.est_admin = NULL;
		} else {
			port->qbv.est_admin = est_new;
		}
	} else {
		devm_kfree(&ndev->dev, est_new);
	}
err:
	pm_runtime_put(common->dev);
	return ret;
}

static int am65_cpsw_recalc_taprio(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_est *est_admin;
	struct am65_cpsw_est *est_oper;
	int ret;

	if (!am65_cpsw_port_est_enabled(port))
		return 0;

	am65_cpsw_est_update_state(ndev);

	est_oper = port->qbv.est_oper;
	est_admin = port->qbv.est_admin;
	port->qbv.est_oper = NULL;
	port->qbv.est_admin = NULL;

	if (est_oper) {
		ret = am65_cpsw_set_taprio(ndev, &est_oper->taprio);
		if (ret)
			goto err;
	}

	if (est_admin) {
		ret = am65_cpsw_set_taprio(ndev, &est_admin->taprio);
		if (ret)
			goto err;
	}

	if (est_oper)
		devm_kfree(&ndev->dev, est_oper);

	if (est_admin)
		devm_kfree(&ndev->dev, est_admin);

	return 0;
err:

	port->qbv.est_oper = est_oper;
	port->qbv.est_admin = est_admin;
	return ret;
}

void am65_cpsw_qbv_adj_link(struct net_device *ndev)
{
	am65_cpsw_recalc_taprio(ndev);
}

#endif
