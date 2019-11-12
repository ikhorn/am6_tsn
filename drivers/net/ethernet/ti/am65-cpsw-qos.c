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
#define AM65_CPSW_PN_REG_FIFO_STATUS		0x050
#define AM65_CPSW_PN_REG_EST_CTL		0x060

/* AM65_CPSW_REG_CTL register fields */
#define AM65_CPSW_CTL_EST_EN			BIT(18)

/* AM65_CPSW_PN_REG_CTL register fields */
#define AM65_CPSW_PN_CTL_EST_PORT_EN		BIT(17)

/* AM65_CPSW_PN_REG_EST_CTL register fields */
#define AM65_CPSW_PN_EST_ONEBUF			BIT(0)
#define AM65_CPSW_PN_EST_BUFSEL			BIT(1)
#define AM65_CPSW_PN_EST_TS_EN			BIT(2)
#define AM65_CPSW_PN_EST_TS_FIRST		BIT(3)
#define AM65_CPSW_PN_EST_ONEPRI			BIT(4)
#define AM65_CPSW_PN_EST_TS_PRI_MSK		GENMASK(7, 5)

/* AM65_CPSW_PN_REG_FIFO_STATUS register fields */
#define AM65_CPSW_PN_FST_TX_PRI_ACTIVE_MSK	GENMASK(7, 0)
#define AM65_CPSW_PN_FST_TX_E_MAC_ALLOW_MSK	GENMASK(15, 8)
#define AM65_CPSW_PN_FST_EST_CNT_ERR		BIT(16)
#define AM65_CPSW_PN_FST_EST_ADD_ERR		BIT(17)
#define AM65_CPSW_PN_FST_EST_BUFACT		BIT(18)

/* EST FETCH COMMAND RAM */
#define AM65_CPSW_PORT_RAM_BASE			0x10000
#define AM65_CPSW_FETCH_RAM_CMD_NUM		0x80
#define AM65_CPSW_FETCH_CNT_MSK			GENMASK(22, 8)
#define AM65_CPSW_FETCH_CNT_MAX			(AM65_CPSW_FETCH_CNT_MSK >> 8)
#define AM65_CPSW_FETCH_CNT_OFFSET		8
#define AM65_CPSW_FETCH_ALLOW_MSK		GENMASK(7, 0)
#define AM65_CPSW_FETCH_ALLOW_MAX		AM65_CPSW_FETCH_ALLOW_MSK

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

/* target new EST RAM buffer, actual toggle happens after cycle completion */
static void am65_cpsw_port_est_assign_buf_num(struct net_device *ndev,
					      int buf_num)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u32 val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_EST_CTL);
	if (buf_num)
		val |= AM65_CPSW_PN_EST_BUFSEL;
	else
		val &= ~AM65_CPSW_PN_EST_BUFSEL;

	writel(val, port->port_base + AM65_CPSW_PN_REG_EST_CTL);
}

/* This should be called with preemption/interrupt disabled to avoid long
 * delays while flushing.
 */
static int am65_cpsw_port_est_get_free_buf_num(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	int oper_buf, admin_buf;
	int try = 2;
	u32 val;

	while (try--) {
		/* can't toggle buffer if prev. toggle is not completed */
		val = readl(port->port_base + AM65_CPSW_PN_REG_FIFO_STATUS);
		oper_buf = !!(val & AM65_CPSW_PN_FST_EST_BUFACT);

		val = readl(port->port_base + AM65_CPSW_PN_REG_EST_CTL);
		admin_buf = !!(val & AM65_CPSW_PN_EST_BUFSEL);
		if (admin_buf == oper_buf)
			return !oper_buf;

		/* flush transition as it's not allowed to touch memory
		 * in-flight
		 */
		am65_cpsw_port_est_assign_buf_num(ndev, oper_buf);

		dev_info(&ndev->dev,
			 "Prev. EST admin cycle is in transit %d -> %d\n",
			 oper_buf, admin_buf);
	}

	return admin_buf;
}

static void am65_cpsw_port_est_get_buf_num(struct net_device *ndev,
					   struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u32 val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_EST_CTL);
	if (est_new->one_buf)
		val |= AM65_CPSW_PN_EST_ONEBUF;
	else
		val &= ~AM65_CPSW_PN_EST_ONEBUF;

	writel(val, port->port_base + AM65_CPSW_PN_REG_EST_CTL);

	if (!est_new->one_buf)
		est_new->buf = am65_cpsw_port_est_get_free_buf_num(ndev);
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

static void am65_cpsw_admin_to_oper(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	if (port->qbv.est_oper)
		devm_kfree(&ndev->dev, port->qbv.est_oper);

	port->qbv.est_oper = port->qbv.est_admin;
	port->qbv.est_admin = NULL;
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

	am65_cpsw_admin_to_oper(ndev);
}

static void am65_cpsw_catch_buf_swap(struct net_device *ndev,
				     struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	if (port->qbv.est_oper && port->qbv.est_oper->one_buf)
		return;

	/* rolled buf num == changed buf while configuration */
	if (port->qbv.est_oper && port->qbv.est_admin &&
	    est_new->buf == port->qbv.est_oper->buf)
		am65_cpsw_admin_to_oper(ndev);
}

/* Fetch command count it's number of bytes in Gigabit mode or nibbles in
 * 10/100Mb mode. So, having speed and time in ns, recalculate ns to number of
 * bytes/nibbles that can be sent while transmission on given speed.
 */
static int am65_est_cmd_ns_to_cnt(u64 ns, int link_speed)
{
	u64 temp;

	temp = ns * link_speed;
	if (link_speed < SPEED_1000)
		temp <<= 1;

	return DIV_ROUND_UP(temp, 8 * 1000);
}

static void __iomem *am65_cpsw_est_set_sched_cmds(void __iomem *addr,
						  int fetch_cnt,
						  int fetch_allow)
{
	u32 prio_mask, cmd_fetch_cnt, cmd;

	do {
		if (fetch_cnt > AM65_CPSW_FETCH_CNT_MAX) {
			fetch_cnt -= AM65_CPSW_FETCH_CNT_MAX;
			cmd_fetch_cnt = AM65_CPSW_FETCH_CNT_MAX;
		} else {
			cmd_fetch_cnt = fetch_cnt;
			/* fetch count can't be less than 16? */
			if (cmd_fetch_cnt && cmd_fetch_cnt < 16)
				cmd_fetch_cnt = 16;

			fetch_cnt = 0;
		}

		prio_mask = fetch_allow & AM65_CPSW_FETCH_ALLOW_MSK;
		cmd = (cmd_fetch_cnt << AM65_CPSW_FETCH_CNT_OFFSET) | prio_mask;

		writel(cmd, addr);
		addr += 4;
	} while (fetch_cnt);

	return addr;
}

static int am65_cpsw_est_calc_cmd_num(struct tc_taprio_qopt_offload *taprio,
				      int link_speed)
{
	int i, cmd_cnt, cmd_sum = 0;
	u32 fetch_cnt;

	for (i = 0; i < taprio->num_entries; i++) {
		fetch_cnt = am65_est_cmd_ns_to_cnt(taprio->entries[i].interval,
						   link_speed);

		if (fetch_cnt > AM65_CPSW_FETCH_CNT_MAX) {
			cmd_cnt = fetch_cnt / AM65_CPSW_FETCH_CNT_MAX;
			if (cmd_cnt * AM65_CPSW_FETCH_CNT_MAX < fetch_cnt)
				cmd_cnt++;
		} else {
			cmd_cnt = 1;
		}

		cmd_sum += cmd_cnt;

		if (!fetch_cnt)
			break;
	}

	return cmd_sum;
}

static int am65_cpsw_est_get_buf_mode(struct net_device *ndev, int link_speed,
				      struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	int cmd_num;

	cmd_num = am65_cpsw_est_calc_cmd_num(&est_new->taprio, link_speed);
	if (cmd_num > AM65_CPSW_FETCH_RAM_CMD_NUM) {
		netdev_err(ndev, "No fetch RAM");
		return -ENOMEM;
	}

	if (port->qbv.est_oper && port->qbv.est_oper->one_buf) {
		netdev_err(ndev,
			   "One buf fetch RAM was used, stop taprio first");
		return -ENOMEM;
	}

	if (port->qbv.est_oper &&
	    cmd_num > AM65_CPSW_FETCH_RAM_CMD_NUM / 2) {
		netdev_err(ndev,
			   "Failed to toggle fetch RAM, stop taprio first");
		return -ENOMEM;
	}

	/* Use two buffer operation to toggle configuration easily, it's
	 * supposed to cover mostly all cases. But if requested more memory
	 * than one buffer can allow and no running EST schedule for the port
	 * the one_buf mode can be used.
	 */
	if (cmd_num > AM65_CPSW_FETCH_RAM_CMD_NUM / 2) {
		est_new->one_buf = 1;
		dev_info(&ndev->dev,
			 "One buffer mode is used, no runtime schedule swap is allowed afterwards");
	} else {
		est_new->one_buf = 0;
	}

	return 0;
}

static void am65_cpsw_est_set_sched_list(struct net_device *ndev,
					 int link_speed,
					 struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	void __iomem *ram_addr, *max_ram_addr;
	struct tc_taprio_sched_entry *entry;
	u32 fetch_cnt, fetch_allow;
	int i, ram_size;

	/* use port base as it includes port offset? */
	ram_addr = port->port_base + AM65_CPSW_PORT_RAM_BASE;
	if (est_new->one_buf) {
		ram_size = AM65_CPSW_FETCH_RAM_CMD_NUM * 4;
	} else {
		ram_size = AM65_CPSW_FETCH_RAM_CMD_NUM * 2;
		ram_addr += est_new->buf * ram_size;
	}

	max_ram_addr = ram_size + ram_addr;
	for (i = 0; i < est_new->taprio.num_entries; i++) {
		entry = &est_new->taprio.entries[i];

		fetch_cnt = am65_est_cmd_ns_to_cnt(entry->interval, link_speed);
		fetch_allow = entry->gate_mask;
		if (fetch_allow > AM65_CPSW_FETCH_ALLOW_MAX)
			dev_dbg(&ndev->dev, "fetch_allow > 8 bits: %d\n",
				fetch_allow);

		ram_addr = am65_cpsw_est_set_sched_cmds(ram_addr, fetch_cnt,
							fetch_allow);

		if (!fetch_cnt && i < est_new->taprio.num_entries - 1) {
			dev_info(&ndev->dev,
				 "next scheds after %d have no impact", i + 1);
			break;
		}
	}

	/* end null cmd ? */
	if (ram_addr < max_ram_addr)
		writel(0, ram_addr);
}

static int am65_cpsw_configure_taprio(struct net_device *ndev,
				      struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = port->common;
	struct ethtool_link_ksettings ecmd;
	int ret, link_speed;

	am65_cpsw_est_update_state(ndev);

	if (!est_new->taprio.enable) {
		am65_cpsw_est_set(ndev, est_new->taprio.enable);
		return 0;
	}

	if (common->pf_p0_rx_ptype_rrobin) {
		netdev_err(ndev,
			   "p0-rx-ptype-rrobin flag conflicts with taprio qdisc\n");
		return -EINVAL;
	}

	ret = __ethtool_get_link_ksettings(ndev, &ecmd);
	if (ret < 0)
		return ret;

	if (ecmd.base.speed && ecmd.base.speed != SPEED_UNKNOWN)
		link_speed = ecmd.base.speed;
	else
		link_speed = SPEED_10;

	ret = am65_cpsw_est_get_buf_mode(ndev, link_speed, est_new);
	if (ret < 0)
		return ret;

	am65_cpsw_port_est_get_buf_num(ndev, est_new);
	am65_cpsw_catch_buf_swap(ndev, est_new);
	am65_cpsw_est_set_sched_list(ndev, link_speed, est_new);
	am65_cpsw_port_est_assign_buf_num(ndev, est_new->buf);

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

	if (taprio->cycle_time_extension) {
		netdev_err(ndev, "Failed to set cycle time extension");
		return -EOPNOTSUPP;
	}

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
