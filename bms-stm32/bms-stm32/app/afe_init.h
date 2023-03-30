/*
 * afe_init.h
 *
 *  Created on: Aug 21, 2020
 *      Author: quangnd
 */

#ifndef AFE_INIT_H_
#define AFE_INIT_H_

#include "bq769x0.h"
#include "bq769x2.h"

extern BQ769x2 bq;
extern AFE app_afe;
void afe_init(void);

#endif /* AFE_INIT_H_ */
