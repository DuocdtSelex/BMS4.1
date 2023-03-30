/*
 * soc_est.h
 *
 *  Created on: Aug 22, 2020
 *      Author: quangnd
 */

#ifndef SERVICE_SOH_EST_SOH_EST_H_
#define SERVICE_SOH_EST_SOH_EST_H_

#include "stdint.h"
#include "estimator.h"
typedef struct SOH_Est_t SOH_Est;
typedef struct SOH_Est_State_Vector_t SOH_Est_State_Vector;

struct SOH_Est_State_Vector_t{
	uint32_t current;
	uint32_t voltage;
	uint32_t temp;
};

struct SOH_Est_t{
	Estimator base;
};

SOH_Est* soh_est_create(void);

#endif /* SERVICE_SOH_EST_SOH_EST_H_ */
