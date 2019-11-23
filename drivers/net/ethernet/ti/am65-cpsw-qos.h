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
	int one_buf;
	int buf;
	/* has to be the last one */
	struct tc_taprio_qopt_offload taprio;
};

struct am65_cpsw_qbv {
	struct am65_cpsw_est *est_admin;
	struct am65_cpsw_est *est_oper;
};

#if IS_ENABLED(CONFIG_NET_SCH_TAPRIO)

int am65_cpsw_set_taprio(struct net_device *ndev, void *type_data);
void am65_cpsw_qbv_adj_link(struct net_device *ndev);

#else

static inline int am65_cpsw_set_taprio(struct net_device *ndev, void *type_data)
{
	return -EOPNOTSUPP;
}

static inline void am65_cpsw_qbv_adj_link(struct net_device *ndev)
{
}

#endif

#endif /* AM65_CPSW_QOS_H_ */
