/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016-2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 */

#ifndef AM65_CPSW_QOS_H_
#define AM65_CPSW_QOS_H_

#include <linux/netdevice.h>

int am65_cpsw_qos_ndo_setup_tc(struct net_device *ndev, enum tc_setup_type type,
			       void *type_data);
int am65_cpsw_qos_init(struct net_device *ndev);

#endif /* AM65_CPSW_QOS_H_ */
