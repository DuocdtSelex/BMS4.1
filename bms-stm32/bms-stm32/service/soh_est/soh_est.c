/*
 * soc_est.c
 *
 *  Created on: Aug 22, 2020
 *      Author: quangnd
 */

#include "soh_est.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"

static void soh_update_impl(Estimator* p_est,void* inputs);
static void soh_est_init_impl(Estimator* p_est);

SOH_Est* soh_est_create(void){
	SOH_Est* this=(SOH_Est*) malloc(sizeof(SOH_Est));
	while(this==NULL){};
	SOH_Est_State_Vector* p_vector=malloc(sizeof(SOH_Est_State_Vector));
	while(p_vector==NULL){};
	this->base.output=malloc(sizeof(uint32_t));
	this->base.init=soh_est_init_impl;
	this->base.update=soh_update_impl;
	return this;
}

static void soh_est_init_impl(Estimator* p_est){

	SOH_Est* p_soh_est=(SOH_Est*)p_est;
	p_soh_est->base.init=soh_est_init_impl;
	p_soh_est->base.update=soh_update_impl;
}

static void soh_update_impl(Estimator* p_est,void* inputs){

	SOH_Est* p_soh_est=(SOH_Est*)p_est;
	SOH_Est_State_Vector* p_vector=(SOH_Est_State_Vector*) inputs;
	uint32_t* p_soh=(uint32_t*) p_est->output;

	/* run update code */
#if 0
	float soc_avg, cc_avg, Var_SoC, Var_cc, deltaSoC_square;
	int32_t soh_uart, c1_uart, c2_uart, c3_uart, soc_uart, k_uart;
	//	deltaSoC_square = (SoC_arr[10]-SoC_arr[1])*(SoC_arr[10]-SoC_arr[1]);
	number = 0;
	counter_timer_temp = 0;
	//	if(deltaSoC_square < 0.0025){  // avoid affecting c1->c6
	//		is_dis = true;   //active for again
	//		return;
	//	}
	Total_SoC -= SoC_arr[0];    // delete data 1th => 11-1 = 10 data
	Total_cc -= cc_arr[0];
	soc_avg = Total_SoC / 50;
	cc_avg = Total_cc / 50;
	Total_SoC = 0;
	Total_cc = 0;
	Var_SoC = 0;
	Var_cc = 0;
	for (int i = 1; i < 51; i++) {
		Var_SoC += (SoC_arr[i] - soc_avg) * (SoC_arr[i] - soc_avg);
		Var_cc += (cc_arr[i] - cc_avg) * (cc_arr[i] - cc_avg);
	}
	Var_SoC /= 49; //n-1
	Var_cc /= 49;
	for (int i = 1, j = 2; i < 50; i++, j++) {
		x = SoC_arr[j] - SoC_arr[i];
		y = (cc_arr[j] - cc_arr[i]);
		c1 += x * x / Var_cc;
		c2 += x * y / Var_cc;
		c3 += y * y / Var_cc;
		/*		c4 += x*x/Var_SoC;
		 c5 += x*y/Var_SoC;
		 c6 += y*y/Var_SoC;   */
	}
	k = sqrt(Var_SoC / Var_cc); //if k 5 loop is same => TLS method ....else AWTLS method
	Q_estimate = (double) (-c1 + k * k * c3
			+ sqrt((c1 - k * k * c3) * (c1 - k * k * c3) + 4 * k * k * c2 * c2))
			/ (2 * k * k * c2);
	p_bms->soh = (double) Q_estimate * 100.0 / Qnom;
#if DEBUG_UART_SOH
			 	       itoa(time_soc1,buffer,10);USART_Write_String(buffer);USART_Write_String("\t");
			 	       time_soc1 += 25;  //25 second
					soh_uart = (int32_t)(p_bms->soh*1000);
					               itoa(soh_uart/1000,buffer,10);USART_Write_String(buffer);USART_Write_String(".");
						 		   itoa((soh_uart%1000)/100,buffer,10);USART_Write_String(buffer);
						 		   itoa((soh_uart%100)/10,buffer,10);USART_Write_String(buffer);
						 		   itoa(soh_uart%10,buffer,10);USART_Write_String(buffer);
						 		  USART_Write_String("\t");
				k_uart = (int32_t)(k*1000);
									itoa(k_uart/1000,buffer,10);USART_Write_String(buffer);USART_Write_String(".");
									 itoa((k_uart%1000)/100,buffer,10);USART_Write_String(buffer);
									 itoa((k_uart%100)/10,buffer,10);USART_Write_String(buffer);
									itoa(k_uart%10,buffer,10);USART_Write_String(buffer);
									USART_Write_String("\t");
						 		  // send SoC:
				    soc_uart = (int32_t)(p_bms->soc*1000);
						           itoa(soc_uart/1000,buffer,10);USART_Write_String(buffer);USART_Write_String(".");
						 		   itoa((soc_uart%1000)/100,buffer,10);USART_Write_String(buffer);
						 		   itoa((soc_uart%100)/10,buffer,10);USART_Write_String(buffer);
						 		   itoa(soc_uart%10,buffer,10);USART_Write_String(buffer);
						 		   USART_Write_String("\n\r");

				    c1_uart = (int32_t)(c1*1000);
						 		   itoa(c1_uart/1000,buffer,10);USART_Write_String(buffer);USART_Write_String(".");
						 		   itoa((c1_uart%1000)/100,buffer,10);USART_Write_String(buffer);
						 		   itoa((c1_uart%100)/10,buffer,10);USART_Write_String(buffer);
						 		   itoa(c1_uart%10,buffer,10);USART_Write_String(buffer);
						 		   USART_Write_String("\t\t");
	                c2_uart = (int32_t)(c2*1000);
								   itoa(c2_uart/1000,buffer,10);USART_Write_String(buffer);USART_Write_String(".");
								   itoa((c2_uart%1000)/100,buffer,10);USART_Write_String(buffer);
								   itoa((c2_uart%100)/10,buffer,10);USART_Write_String(buffer);
								   itoa(c2_uart%10,buffer,10);USART_Write_String(buffer);
								   USART_Write_String("\t\t");
					c3_uart = (int32_t)(c3*1000);
								   itoa(c3_uart/1000,buffer,10);USART_Write_String(buffer);USART_Write_String(".");
								   itoa((c3_uart%1000)/100,buffer,10);USART_Write_String(buffer);
								   itoa((c3_uart%100)/10,buffer,10);USART_Write_String(buffer);
								   itoa(c3_uart%10,buffer,10);USART_Write_String(buffer);
								   USART_Write_String("\n\r");
#endif
#endif
}

