// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments K3 AM65 Ethernet QoS submodule
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *
 * quality of service module includes:
 * Enhanced Scheduler Traffic (EST - P802.1Qbv/D2.2)
 * Intersperced Express Traffic (IET – P802.3br/D2.0)
 */

#include "am65-cpsw-qos.h"
#include "am65-cpsw-nuss.h"
#include "am65-cpts.h"
#include <linux/pm_runtime.h>
#include <linux/time.h>

#define AM65_CPSW_REG_CTL			0x004
#define AM65_CPSW_PN_REG_CTL			0x004
#define AM65_CPSW_PN_REG_MAX_BLKS		0x008
#define AM65_CPSW_PN_REG_IET_CTRL		0x040
#define AM65_CPSW_PN_REG_IET_STATUS		0x044
#define AM65_CPSW_PN_REG_IET_VERIFY		0x049
#define AM65_CPSW_PN_REG_FIFO_STATUS		0x050
#define AM65_CPSW_PN_REG_EST_CTL		0x060
#define AM65_CPSW_PN_REG_MAC_TX_GAP		0x3A4

/* AM65_CPSW_REG_CTL register fields */
#define AM65_CPSW_CTL_IET_EN			BIT(17)
#define AM65_CPSW_CTL_EST_EN			BIT(18)

/* AM65_CPSW_PN_REG_CTL register fields */
#define AM65_CPSW_PN_CTL_IET_PORT_EN		BIT(16)
#define AM65_CPSW_PN_CTL_EST_PORT_EN		BIT(17)

/* AM65_CPSW_PN_REG_MAX_BLKS register vals */
#define AM65_CPSW_PN_MAX_BLKS_DEF		0x00001004
#define AM65_CPSW_PN_MAX_BLKS_IET		0x00000D07

/* AM65_CPSW_PN_REG_EST_CTL register fields */
#define AM65_CPSW_PN_EST_ONEBUF			BIT(0)
#define AM65_CPSW_PN_EST_BUFSEL			BIT(1)
#define AM65_CPSW_PN_EST_TS_EN			BIT(2)
#define AM65_CPSW_PN_EST_TS_FIRST		BIT(3)
#define AM65_CPSW_PN_EST_ONEPRI			BIT(4)
#define AM65_CPSW_PN_EST_TS_PRI_MSK		GENMASK(7, 5)
#define AM65_CPSW_PN_EST_FILL_EN		BIT(8)
#define AM65_CPSW_PN_EST_FILL_MARGIN_MSK	GENMASK(25, 16)
#define AM65_CPSW_PN_EST_FILL_MARGIN_OFFSET	16

/* AM65_CPSW_PN_REG_IET_CTRL register fields */
#define AM65_CPSW_PN_IET_PENABLE		BIT(0)
#define AM65_CPSW_PN_IET_MACHOLD		BIT(1)
#define AM65_CPSW_PN_IET_VERIFY_DBL		BIT(2)
#define AM65_CPSW_PN_IET_LINK_FAIL		BIT(3)
#define AM65_CPSW_PN_IET_PREMPT_MASK		GENMASK(23, 16)
#define AM65_CPSW_PN_IET_PREMPT_OFFSET		16

/* AM65_CPSW_PN_REG_IET_STATUS register fields */
#define AM65_CPSW_PN_MAC_VERIFIED		BIT(0)
#define AM65_CPSW_PN_MAC_VERIFY_FAIL		BIT(1)
#define AM65_CPSW_PN_MAC_RESPOND_ERR		BIT(2)
#define AM65_CPSW_PN_MAC_VERIFY_ERR		BIT(3)

/* AM65_CPSW_PN_REG_IET_VERIFY register fields */
#define AM65_CPSW_IET_VERIFY_1000		0x001312D0

/* AM65_CPSW_PN_REG_FIFO_STATUS register fields */
#define AM65_CPSW_PN_FST_TX_PRI_ACTIVE_MSK	GENMASK(7, 0)
#define AM65_CPSW_PN_FST_TX_E_MAC_ALLOW_MSK	GENMASK(15, 8)
#define AM65_CPSW_PN_FST_EST_CNT_ERR		BIT(16)
#define AM65_CPSW_PN_FST_EST_ADD_ERR		BIT(17)
#define AM65_CPSW_PN_FST_EST_BUFACT		BIT(18)

/* EST FETCH COMMAND RAM */
#define AM65_CPSW_FETCH_RAM_CMD_NUM		0x80
#define AM65_CPSW_FETCH_CNT_MSK			GENMASK(22, 8)
#define AM65_CPSW_FETCH_CNT_MAX			(AM65_CPSW_FETCH_CNT_MSK >> 8)
#define AM65_CPSW_FETCH_CNT_OFFSET		8
#define AM65_CPSW_FETCH_ALLOW_MSK		GENMASK(7, 0)
#define AM65_CPSW_FETCH_ALLOW_MAX		AM65_CPSW_FETCH_ALLOW_MSK

/* AM65_CPSW_PN_REG_MAC_TX_GAP */
#define AM65_CPSW_TX_GAP_GMII_MSK		GENMASK(7, 0)
#define AM65_CPSW_TX_GAP_XGMII_MSK		GENMASK(15, 0)
#define AM65_CPSW_TX_GAP_SHORT			12

enum pf_act {
	PF_ENABLE,
	PF_DISABLE,
	PF_COLLISION,
};

enum timer_act {
	TACT_PROG,		/* need program timer */
	TACT_NEED_STOP,		/* need stop first */
	TACT_SKIP_PROG,		/* just buffer can be updated */
};

static int am65_cpsw_port_est_enabled(struct am65_cpsw_port *port)
{
	return port->qos.est_oper || port->qos.est_admin;
}

/* IET */

void am65_cpsw_iet_set_mac_hold(struct net_device *ndev, int on)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u32 val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_IET_CTRL);

	 /* don't schedule verification, for sure */
	val |= AM65_CPSW_PN_IET_LINK_FAIL;
	if (on)
		val |= AM65_CPSW_PN_IET_MACHOLD;
	else
		val &= ~AM65_CPSW_PN_IET_MACHOLD;

	writel(val, port->port_base + AM65_CPSW_PN_REG_IET_CTRL);
}

static void am65_cpsw_iet_enable(struct am65_cpsw_common *common)
{
	int common_enable = 0;
	u32 val;
	int i;

	for (i = 0; i < common->port_num; i++)
		common_enable |= !!common->ports[i].qos.iet_mask;

	val = readl(common->cpsw_base + AM65_CPSW_REG_CTL);

	if (common_enable)
		val |= AM65_CPSW_CTL_IET_EN;
	else
		val &= ~AM65_CPSW_CTL_IET_EN;

	writel(val, common->cpsw_base + AM65_CPSW_REG_CTL);
	common->iet_enabled = common_enable;
}

static void am65_cpsw_port_iet_enable(struct am65_cpsw_port *port,
				      int link_speed, u32 mask)
{
	u32 max_blks_val;
	u32 val;

	/* set verify count depending on link speed */
	val = DIV_ROUND_UP(SPEED_1000, link_speed);
	val = val * AM65_CPSW_IET_VERIFY_1000;
	writel(val, port->port_base + AM65_CPSW_PN_REG_IET_VERIFY);

	val = readl(port->port_base + AM65_CPSW_PN_REG_CTL);
	if (mask) {
		val |= AM65_CPSW_PN_CTL_IET_PORT_EN;
		max_blks_val = AM65_CPSW_PN_MAX_BLKS_IET;
	} else {
		val &= ~AM65_CPSW_PN_CTL_IET_PORT_EN;
		max_blks_val = AM65_CPSW_PN_MAX_BLKS_DEF;
	}

	writel(max_blks_val, port->port_base + AM65_CPSW_PN_REG_MAX_BLKS);
	writel(val, port->port_base + AM65_CPSW_PN_REG_CTL);
	port->qos.iet_mask = mask;
}

static int am65_cpsw_iet_verify(struct net_device *ndev, int link_speed)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	int try;
	u32 val;

	/* supposes link fail bit cleaned before
	 * 10 ms - 1Gb
	 */
	try = DIV_ROUND_UP(SPEED_1000, link_speed) + 1;
	while (try--) {
		val = readl(port->port_base + AM65_CPSW_PN_REG_IET_STATUS);
		if (val & AM65_CPSW_PN_MAC_VERIFIED)
			break;

		if (val & AM65_CPSW_PN_MAC_VERIFY_FAIL) {
			dev_err(&ndev->dev, "IET MAC verify failed");
			return -1;
		}

		if (val & AM65_CPSW_PN_MAC_RESPOND_ERR) {
			dev_err(&ndev->dev, "IET MAC respond error");
			return -1;
		}

		if (val & AM65_CPSW_PN_MAC_VERIFY_ERR) {
			dev_err(&ndev->dev, "IET MAC verify error");
			return -1;
		}

		msleep(10);
	}

	if (try < 0) {
		dev_err(&ndev->dev, "IET MAC verify timeout");
		return -1;
	}

	return 0;
}

static void am65_cpsw_iet_set_mask(struct am65_cpsw_port *port, u32 mask)
{
	u32 val;

	val = mask << AM65_CPSW_PN_IET_PREMPT_OFFSET;
	val &= AM65_CPSW_PN_IET_PREMPT_MASK;

	 /* don't schedule verification */
	val |= AM65_CPSW_PN_IET_LINK_FAIL;

	/* no mask - no need in IET */
	if (mask)
		val |= AM65_CPSW_PN_IET_PENABLE;

	writel(val, port->port_base + AM65_CPSW_PN_REG_IET_CTRL);
}

int am65_cpsw_iet_set(struct net_device *ndev, int link_speed, u32 mask)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = port->common;
	int ret, verify;
	u32 val;

	if (mask && am65_cpsw_port_est_enabled(port)) {
		netdev_err(ndev,
			   "Frame preemption has to be enabled before EST");
		return -EINVAL;
	}

	/* TODO: xmit shouldn't send packet to premt queue till the moment
	 * preemption is configured, so stop sending packets to appropriate
	 * qeues and flush & suspend them? Not cool but skip for now.
	 */

	am65_cpsw_port_iet_enable(port, link_speed, mask);

	/* verify only if not verified, that is after IET off */
	val = readl(port->port_base + AM65_CPSW_PN_REG_IET_STATUS);
	verify = mask && !(val & AM65_CPSW_PN_MAC_VERIFIED);

	/* clean link fail bit to enable verification */
	if (verify)
		writel(AM65_CPSW_PN_IET_PENABLE,
		       port->port_base + AM65_CPSW_PN_REG_IET_CTRL);

	am65_cpsw_iet_enable(common);

	if (verify) {
		ret = am65_cpsw_iet_verify(ndev, link_speed);
		if (ret)
			goto err;
	}

	am65_cpsw_iet_set_mask(port, mask);
	return 0;
err:
	am65_cpsw_iet_set_mask(port, 0);
	am65_cpsw_port_iet_enable(port, link_speed, 0);
	am65_cpsw_iet_enable(common);
	return ret;
}

#if IS_ENABLED(CONFIG_NET_SCH_TAPRIO)

/* EST */

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

static int am65_cpsw_port_est_is_swapped(struct net_device *ndev, int *oper,
					 int *admin)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u32 val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_FIFO_STATUS);
	*oper = !!(val & AM65_CPSW_PN_FST_EST_BUFACT);

	val = readl(port->port_base + AM65_CPSW_PN_REG_EST_CTL);
	*admin = !!(val & AM65_CPSW_PN_EST_BUFSEL);

	return *admin == *oper;
}

/* This should be called with preemption/interrupt disabled to avoid long
 * delays while flushing.
 */
static int am65_cpsw_port_est_get_free_buf_num(struct net_device *ndev)
{
	int oper, admin;
	int roll = 2;

	while (roll--) {
		if (am65_cpsw_port_est_is_swapped(ndev, &oper, &admin))
			return !oper;

		/* admin is not set, so hinder transition as it's not allowed
		 * to touch memory in-flight, by targeting same oper buf.
		 */
		am65_cpsw_port_est_assign_buf_num(ndev, oper);

		dev_info(&ndev->dev,
			 "Prev. EST admin cycle is in transit %d -> %d\n",
			 oper, admin);
	}

	return admin;
}

static void am65_cpsw_admin_to_oper(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	if (port->qos.est_oper)
		devm_kfree(&ndev->dev, port->qos.est_oper);

	port->qos.est_oper = port->qos.est_admin;
	port->qos.est_admin = NULL;
}

static void am65_cpsw_port_est_get_buf_num(struct net_device *ndev,
					   struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u32 val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_EST_CTL);
	val &= ~AM65_CPSW_PN_EST_ONEBUF;

	writel(val, port->port_base + AM65_CPSW_PN_REG_EST_CTL);

	est_new->buf = am65_cpsw_port_est_get_free_buf_num(ndev);

	/* rolled buf num means changed buf while configuring */
	if (port->qos.est_oper && port->qos.est_admin &&
	    est_new->buf == port->qos.est_oper->buf)
		am65_cpsw_admin_to_oper(ndev);
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
 * of admin -> oper transition, particularly it's supposed to be used in some
 * generic routine for providing real state to Taprio Qdisc.
 */
static void am65_cpsw_est_update_state(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	int oper, admin;

	if (!port->qos.est_admin)
		return;

	if (!am65_cpsw_port_est_is_swapped(ndev, &oper, &admin))
		return;

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

static int am65_cpsw_est_calc_cmd_num(struct net_device *ndev,
				      struct tc_taprio_qopt_offload *taprio,
				      int link_speed)
{
	int i, cmd_cnt, cmd_sum = 0;
	u32 fetch_cnt;

	for (i = 0; i < taprio->num_entries; i++) {
		if (taprio->entries[i].command != TC_TAPRIO_CMD_SET_GATES) {
			dev_err(&ndev->dev, "Only SET command is supported");
			return -EINVAL;
		}

		fetch_cnt = am65_est_cmd_ns_to_cnt(taprio->entries[i].interval,
						   link_speed);

		cmd_cnt = DIV_ROUND_UP(fetch_cnt, AM65_CPSW_FETCH_CNT_MAX);
		if (!cmd_cnt)
			cmd_cnt++;

		cmd_sum += cmd_cnt;

		if (!fetch_cnt)
			break;
	}

	return cmd_sum;
}

static int am65_cpsw_est_check_shceds(struct net_device *ndev,
				      struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	int cmd_num;

	cmd_num = am65_cpsw_est_calc_cmd_num(ndev, &est_new->taprio,
					     port->qos.link_speed);
	if (cmd_num < 0)
		return cmd_num;

	if (cmd_num > AM65_CPSW_FETCH_RAM_CMD_NUM / 2) {
		dev_err(&ndev->dev, "No fetch RAM");
		return -ENOMEM;
	}

	return 0;
}

static void am65_cpsw_est_set_sched_list(struct net_device *ndev,
					 struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u32 fetch_cnt, fetch_allow, all_fetch_allow = 0;
	void __iomem *ram_addr, *max_ram_addr;
	struct tc_taprio_sched_entry *entry;
	int i, ram_size;

	ram_addr = port->fram_base;
	ram_size = AM65_CPSW_FETCH_RAM_CMD_NUM * 2;
	ram_addr += est_new->buf * ram_size;

	max_ram_addr = ram_size + ram_addr;
	for (i = 0; i < est_new->taprio.num_entries; i++) {
		entry = &est_new->taprio.entries[i];

		fetch_cnt = am65_est_cmd_ns_to_cnt(entry->interval,
						   port->qos.link_speed);
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

		all_fetch_allow |= fetch_allow;
	}

	/* end cmd, enabling non-timed queues for potential over cycle time */
	if (ram_addr < max_ram_addr)
		writel(~all_fetch_allow & AM65_CPSW_FETCH_ALLOW_MSK, ram_addr);
}

/**
 * Enable ESTf periodic output, set cycle start time and interval.
 */
static int am65_cpsw_timer_set(struct net_device *ndev,
			       struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = port->common;
	struct am65_cpts *cpts = common->cpts;
	struct am65_cpts_estf_req req;

	req.ns_period = est_new->taprio.cycle_time;
	req.idx = port->port_id - 1;
	req.ns_start = est_new->taprio.base_time;
	req.on = est_new->taprio.enable;

	return am65_cpts_estf_enable(cpts, &req);
}

static void am65_cpsw_timer_stop(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpts *cpts = port->common->cpts;
	struct am65_cpts_estf_req req;

	req.idx = port->port_id - 1;
	req.on = 0;

	am65_cpts_estf_enable(cpts, &req);
}

static enum timer_act am65_cpsw_timer_act(struct net_device *ndev,
					  struct am65_cpsw_est *est_new)
{
	struct tc_taprio_qopt_offload *taprio_oper, *taprio_new;
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpts *cpts = port->common->cpts;
	u64 cur_time;
	s64 diff;

	if (!port->qos.est_oper)
		return TACT_PROG;

	taprio_new = &est_new->taprio;
	taprio_oper = &port->qos.est_oper->taprio;

	if (taprio_new->cycle_time != taprio_oper->cycle_time)
		return TACT_NEED_STOP;

	/* in order to avoid timer reset get base_time form oper taprio */
	if (!taprio_new->base_time && taprio_oper)
		taprio_new->base_time = taprio_oper->base_time;

	if (taprio_new->base_time == taprio_oper->base_time)
		return TACT_SKIP_PROG;

	/* base times are cycle synchronized */
	diff = taprio_new->base_time - taprio_oper->base_time;
	diff = diff < 0 ? -diff : diff;
	if (diff % taprio_new->cycle_time)
		return TACT_NEED_STOP;

	cur_time = am65_cpts_ns_gettime(cpts);
	if (taprio_new->base_time <= cur_time + taprio_new->cycle_time)
		return TACT_SKIP_PROG;

	/* Here hrtimer should be requested to prog admin conf within last
	 * cycle of oper conf if hypothetical time allows ofc, say
	 * taprio_new->base_time - cur_time > 40 ms and 2 buffer mode
	 * is allowed. For this, new enum value can be returned here, kind of
	 * TACT_HRTIMER_NEEDED, as for now involve a user to stop.
	 */
	return TACT_NEED_STOP;
}

static u32 am65_cpsw_port_ipg_ns(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u32 val, ipg;

	val = readl(port->port_base + AM65_CPSW_PN_REG_MAC_TX_GAP);

	if (port->qos.link_speed <= SPEED_1000) {
		val &= AM65_CPSW_TX_GAP_GMII_MSK;
		/* bytes to ns, for 1Gb, 12B ~ 96 ns */
		ipg = val * 8 * NSEC_PER_USEC;
	} else {
		/* val &= AM65_CPSW_TX_GAP_XGMII_MSK
		 * But it's rate of short gap, short gap overrides increased
		 * gap value and ipg is still min default - 12? If so then no
		 * matter on short gap rate, ipg is still 12B.
		 */
		ipg = AM65_CPSW_TX_GAP_SHORT * 8 * NSEC_PER_USEC;
	}

	return DIV_ROUND_UP(ipg, port->qos.link_speed);
}

static u32 am65_cpsw_port_calc_fill(struct net_device *ndev,
				    struct am65_cpsw_est *est)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct tc_taprio_qopt_offload *taprio;
	u32 max_pg, fill_margin, ipg;
	u64 cmds_interval = 0;
	int i, zero_allow = 0;

	/* if no zero allow and no interval for non timed queues,
	 * fill margin =  0
	 */
	taprio = &est->taprio;
	for (i = 0; i < taprio->num_entries; i++) {
		cmds_interval += taprio->entries[i].interval;

		/* skip non zero allow scheds */
		if (taprio->entries[i].gate_mask)
			continue;

		zero_allow = 1;
	}

	if (!zero_allow && taprio->cycle_time == cmds_interval)
		return 0;

	/* Use maximum packet size, but should be maximum packet size got from
	 * all non timed queues in above loop, but as at this moment no way to
	 * configure it, set it to be generic maximum for all queues of given
	 * interface.
	 */
	max_pg = ndev->max_mtu * 8 * NSEC_PER_USEC;
	max_pg = DIV_ROUND_UP(max_pg, port->qos.link_speed);

	/* inter packet gap */
	ipg = am65_cpsw_port_ipg_ns(ndev);

	fill_margin = max_pg + ipg;

	return fill_margin;
}

static void am65_cpsw_port_pf_set(struct net_device *ndev,
				  struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u32 margin, margin_cnt, val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_EST_CTL);
	val &= ~AM65_CPSW_PN_EST_FILL_MARGIN_MSK;

	if (est_new->pf_enable) {
		margin = am65_cpsw_port_calc_fill(ndev, est_new);
		val |= AM65_CPSW_PN_EST_FILL_EN;
		/* here has to be another conversion function, use below
		 * am65_est_cmd_ns_to_cnt() for stub till clarification
		 * about TBD cloks.
		 */
		margin_cnt = am65_est_cmd_ns_to_cnt(margin,
						    port->qos.link_speed);
		margin_cnt <<= AM65_CPSW_PN_EST_FILL_MARGIN_OFFSET;
		val |= (margin_cnt & AM65_CPSW_PN_EST_FILL_MARGIN_MSK);
	} else {
		val &= ~AM65_CPSW_PN_EST_FILL_EN;
	}

	writel(val, port->port_base + AM65_CPSW_PN_REG_EST_CTL);
}

static enum pf_act
am65_cpsw_port_pf_check(struct net_device *ndev,
			struct tc_taprio_qopt_offload *taprio, u32 fill_margin)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u64 interval, cmds_interval = 0;
	int i, next, pf_enable = 0;
	u32 ipg, min_pg;
	s32 fill_window;

	min_pg = ndev->min_mtu * 8 * NSEC_PER_USEC;
	min_pg = DIV_ROUND_UP(min_pg, port->qos.link_speed);
	ipg = am65_cpsw_port_ipg_ns(ndev);

	/* it make sense only if zero allow is used */
	for (i = 0; i < taprio->num_entries; i++) {
		cmds_interval += taprio->entries[i].interval;

		/* skip non zero allow scheds */
		if (taprio->entries[i].gate_mask)
			continue;

		/* Once frame preemption feature is added and enabled
		 * here should be check if previous schedule has express and
		 * preempt queues. It can't have both, thus PF can't be
		 * enabled?
		 */

		/* if next is also zero allow, then no harm and fill_window
		 * is more. Can be included also next + (1..n), but this
		 * should be enough.
		 */
		interval = taprio->entries[i].interval;
		next = (i == taprio->num_entries - 1) ? 0 : i + 1;
		if (!taprio->entries[i].gate_mask)
			interval += taprio->entries[next].interval;

		/* packet can't overlap with next fetch RAM command */
		fill_window = interval - fill_margin;
		if (fill_window < 0)
			return PF_COLLISION;

		if (fill_window < min_pg + ipg) {
			dev_info(&ndev->dev,
				 "zero allow [%d] is too short for ESTPF", i);
			continue;
		}

		pf_enable = 1;
	}

	if (pf_enable)
		return PF_ENABLE;

	/* PF can be enabled for reset of cycle if present */
	if (taprio->cycle_time - cmds_interval > fill_margin + min_pg + ipg)
		return PF_ENABLE;

	return PF_DISABLE;
}

/* It's supposed that Packet Fill (PF) is applied only if zero fetch schedule
 * is present in cycle configuration, so does oper cycle or doesn't have PF
 * enabled for fetch zero allow, shouldn't be harm to enable it. In case zero
 * allow interval is too short and fill margin overlaps for admin cycle or oper
 * cycle, the PF is disabled because it can corrupt it. Also, disable PF if it's
 * really no need and enable if configuration allows or leave enabled if oper
 * cycle needs it and no harm for admin cycle. For oper cycle it's no harm to
 * disable PF for last cycle as it has impact only on non timed queues.
 */
static int am65_cpsw_port_pf_get(struct net_device *ndev,
				 struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_est *est_oper = port->qos.est_oper;
	u32 fill_margin_new, fill_margin_oper;
	int pf_new, pf_oper;

	if (!port->qos.pf_allow)
		return 0;

	/* fill margin can be changed while oper -> admin so should be verified
	 * for oper cycl later if no collisions before enabling it
	 */
	fill_margin_new = am65_cpsw_port_calc_fill(ndev, est_new);

	if (est_oper)
		fill_margin_oper =
			am65_cpsw_port_calc_fill(ndev, est_oper);

	pf_new = am65_cpsw_port_pf_check(ndev, &est_new->taprio,
					 fill_margin_new);

	/* check if no collision with oper cycle */
	if (est_oper)
		pf_oper = am65_cpsw_port_pf_check(ndev, &est_oper->taprio,
						  fill_margin_new);

	est_new->pf_enable = (pf_new == PF_ENABLE) &&
			     (!est_oper || ((pf_oper != PF_COLLISION) &&
			     (!est_oper->pf_enable ||
			      fill_margin_new >= fill_margin_oper)));

	if (!est_new->pf_enable && pf_new == PF_ENABLE) {
		/* here can be started hrtimer to enable PF once admin
		 * becomes oper, for now just don't enable it and inform
		 */
		dev_info(&ndev->dev, "Can't enable ESTPF feature");
	}

	if (est_oper && est_oper->pf_enable) {
		/* here can be started hrtimer to disable PF once admin
		 * becomes oper (only for case pf_new == PF_DISABLE, for
		 * collision better stop it before), for now just disable it
		 * and inform.
		 */
		if (!est_new->pf_enable)
			dev_info(&ndev->dev, "Disabling ESTPF feature for admin");
	}

	return 0;
}

static void am65_cpsw_stop_est(struct net_device *ndev)
{
	am65_cpsw_timer_stop(ndev);
	am65_cpsw_est_set(ndev, 0);
}

static void am65_cpsw_purge_est(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	am65_cpsw_stop_est(ndev);

	if (port->qos.est_admin)
		devm_kfree(&ndev->dev, port->qos.est_admin);

	if (port->qos.est_oper)
		devm_kfree(&ndev->dev, port->qos.est_oper);

	port->qos.est_oper = NULL;
	port->qos.est_admin = NULL;
}

static int am65_cpsw_configure_taprio(struct net_device *ndev,
				      struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);
	struct am65_cpts *cpts = common->cpts;
	int ret, tact = TACT_PROG;

	am65_cpsw_est_update_state(ndev);

	if (!est_new->taprio.enable) {
		am65_cpsw_stop_est(ndev);
		return 0;
	}

	ret = am65_cpsw_est_check_shceds(ndev, est_new);
	if (ret < 0)
		return ret;

	tact = am65_cpsw_timer_act(ndev, est_new);
	if (tact == TACT_NEED_STOP) {
		dev_err(&ndev->dev,
			"Can't toggle estf timer, stop taprio first");
		return -EINVAL;
	}

	if (tact == TACT_PROG)
		am65_cpsw_timer_stop(ndev);

	if (!est_new->taprio.base_time)
		est_new->taprio.base_time = am65_cpts_ns_gettime(cpts);

	am65_cpsw_port_est_get_buf_num(ndev, est_new);

	am65_cpsw_port_pf_get(ndev, est_new);

	am65_cpsw_est_set_sched_list(ndev, est_new);
	am65_cpsw_port_est_assign_buf_num(ndev, est_new->buf);
	am65_cpsw_port_pf_set(ndev, est_new);

	am65_cpsw_est_set(ndev, est_new->taprio.enable);

	if (tact == TACT_PROG) {
		ret = am65_cpsw_timer_set(ndev, est_new);
		if (ret) {
			dev_err(&ndev->dev, "Failed to set cycle time");
			return ret;
		}
	}

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

static int am65_cpsw_set_taprio(struct net_device *ndev, void *type_data)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct tc_taprio_qopt_offload *taprio = type_data;
	struct am65_cpsw_common *common = port->common;
	struct am65_cpsw_est *est_new;
	size_t size;
	int ret;

	if (taprio->cycle_time_extension) {
		dev_err(&ndev->dev, "Failed to set cycle time extension");
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
		if (taprio->enable) {
			if (port->qos.est_admin)
				devm_kfree(&ndev->dev, port->qos.est_admin);

			port->qos.est_admin = est_new;
		} else {
			devm_kfree(&ndev->dev, est_new);
			am65_cpsw_purge_est(ndev);
		}
	} else {
		devm_kfree(&ndev->dev, est_new);
	}
err:
	pm_runtime_put(common->dev);
	return ret;
}

static int am65_cpsw_reconf_taprio(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_est *est_admin;
	struct am65_cpsw_est *est_oper;
	int ret;

	if (!am65_cpsw_port_est_enabled(port))
		return 0;

	am65_cpsw_est_update_state(ndev);

	est_oper = port->qos.est_oper;
	est_admin = port->qos.est_admin;
	port->qos.est_oper = NULL;
	port->qos.est_admin = NULL;

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

	port->qos.est_oper = est_oper;
	port->qos.est_admin = est_admin;
	return ret;
}

static void am65_cpsw_est_link_up(struct net_device *ndev, int link_speed)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u64 cur_time;

	if (!am65_cpsw_port_est_enabled(port))
		return;

	if (port->qos.link_down_time) {
		if (link_speed != port->qos.link_speed) {
			dev_err(&ndev->dev,
				"Link speed is changed, stopping TAS");
			goto purge_est;
		}

		cur_time = ktime_get_real_ns();
		if (cur_time - port->qos.link_down_time > NSEC_PER_SEC) {
			dev_err(&ndev->dev,
				"Link has been lost too long, stopping TAS");
			goto purge_est;
		}
	}

	if (am65_cpsw_reconf_taprio(ndev))
		goto purge_est;

	return;

purge_est:
	am65_cpsw_purge_est(ndev);
}

int am65_cpsw_setup_taprio(struct net_device *ndev, void *type_data)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = port->common;
	struct ethtool_link_ksettings ecmd;
	int ret;

	if (!netif_running(ndev)) {
		dev_err(&ndev->dev, "interface is down, link speed unknown\n");
		return -ENETDOWN;
	}

	if (common->pf_p0_rx_ptype_rrobin) {
		dev_err(&ndev->dev,
			"p0-rx-ptype-rrobin flag conflicts with taprio qdisc\n");
		return -EINVAL;
	}

	ret = __ethtool_get_link_ksettings(ndev, &ecmd);
	if (ret < 0)
		return ret;

	if (ecmd.base.speed && ecmd.base.speed == SPEED_UNKNOWN)
		return -ENOLINK;

	port->qos.link_speed = ecmd.base.speed;

	return am65_cpsw_set_taprio(ndev, type_data);
}

#else

static int am65_cpsw_setup_taprio(struct net_device *ndev, void *type_data)
{
	return -EOPNOTSUPP;
}

static void am65_cpsw_est_link_up(struct net_device *ndev, int link_speed)
{
}

#endif

int am65_cpsw_qos_ndo_setup_tc(struct net_device *ndev, enum tc_setup_type type,
			       void *type_data)
{
	switch (type) {
	case TC_SETUP_QDISC_TAPRIO:
		return am65_cpsw_setup_taprio(ndev, type_data);
	default:
		return -EOPNOTSUPP;
	}
}

void am65_cpsw_qos_link_up(struct net_device *ndev, int link_speed)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	am65_cpsw_est_link_up(ndev, link_speed);
	port->qos.link_down_time = 0;
}

void am65_cpsw_qos_link_down(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	if (!port->qos.link_down_time)
		port->qos.link_down_time = ktime_get_real_ns();
}
