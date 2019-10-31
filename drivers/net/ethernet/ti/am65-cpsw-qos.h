/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 */

#ifndef AM65_CPSW_QOS_H_
#define AM65_CPSW_QOS_H_

#include <linux/netdevice.h>
#include <net/pkt_sched.h>

struct am65_cpsw_est {
	int pf_enable;
	int buf;
	/* has to be the last one */
	struct tc_taprio_qopt_offload taprio;
};

struct am65_cpsw_qos {
	struct am65_cpsw_est *est_admin;
	struct am65_cpsw_est *est_oper;
	u64 link_down_time;
	int link_speed;
	int pf_allow;
	int iet_mask;
	int iet_mac_hold;
	int mqprio_hw;
};

int am65_cpsw_qos_ndo_setup_tc(struct net_device *ndev, enum tc_setup_type type,
			       void *type_data);

void am65_cpsw_qos_link_up(struct net_device *ndev, int link_speed);
void am65_cpsw_qos_link_down(struct net_device *ndev);

int am65_cpsw_iet_set(struct net_device *ndev, int link_speed, u32 mask);
void am65_cpsw_iet_set_mac_hold(struct net_device *ndev, int on);
int am65_cpsw_set_mqprio(struct net_device *ndev, void *type_data);

#endif /* AM65_CPSW_QOS_H_ */
