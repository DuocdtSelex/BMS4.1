/*
 * soc_est.h
 *
 *  Created on: Aug 22, 2020
 *      Author: quangnd
 */

#ifndef SERVICE_SOC_EST_SOC_EST_H_
#define SERVICE_SOC_EST_SOC_EST_H_

#include "stdint.h"
#include "estimator.h"
typedef struct SOC_Est_t SOC_Est;
typedef struct SOC_Est_State_Vector_t SOC_Est_State_Vector;

struct SOC_Est_State_Vector_t{
	int32_t current;
	uint32_t voltage;
	uint32_t temp;
};

struct SOC_Est_t{
	Estimator base;
};

SOC_Est* soc_est_create(void);

#endif /* SERVICE_SOC_EST_SOC_EST_H_ */
