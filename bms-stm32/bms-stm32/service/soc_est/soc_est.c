/*
 * soc_est.c
 *
 *  Created on: Aug 22, 2020
 *      Author: quangnd
 */

#include "soc_est.h"
#include "stdint.h"
#include "stdlib.h"

static void soc_update_impl(Estimator* p_est,void* inputs);
static void soc_est_init_impl(Estimator* p_est);

SOC_Est* soc_est_create(void){
	SOC_Est* this=(SOC_Est*) malloc(sizeof(SOC_Est));
	while(this==NULL){};
	SOC_Est_State_Vector* p_vector=malloc(sizeof(SOC_Est_State_Vector));
	while(p_vector==NULL){};
	this->base.input=p_vector;
	this->base.output=malloc(sizeof(uint32_t));
	this->base.init=soc_est_init_impl;
	this->base.update=soc_update_impl;
	return this;
}


static void soc_est_init_impl(Estimator* p_est){

	SOC_Est* p_soc_est=(SOC_Est*)p_est;
	p_soc_est->base.output=malloc(sizeof(uint32_t));
	p_soc_est->base.init=soc_est_init_impl;
	p_soc_est->base.update=soc_update_impl;
}

static void soc_update_impl(Estimator* p_est,void* inputs){

	SOC_Est* p_soc_est=(SOC_Est*)p_est;
	SOC_Est_State_Vector* p_vector=(SOC_Est_State_Vector*) inputs;
	uint32_t* p_soc=(uint32_t*) p_est->output;

	/* run update code */

	#if 0
	float Yk, L1, L2, L1_TEMP, deltaSoc, deltaUd, P1_TEMP, P2_TEMP, P3_TEMP;
	float A2, A4, C2;
	static float A1, A3, C1;
	static float P1 = 0.005, P2 = 0.005, P3 = 0.005, P4 = 0.005;
	int32_t SoC_estimate, Ud_tmp, Ud_tmp_minus, Yk_send, SoC_true;
	// Predictor time update:
	SoC[0] = SoC[0] + (float) (*p_bms->current * UPDATE_SOC_STAMP / total_capacity);
	Ud[count_update_soc + 1] = a1 * Ud[count_update_soc] + b1 * Ik / 1000 + 0.001;  //e-3
	Udk_run[count_update_soc + 1] = a1 * Udk_run[count_update_soc] + b1 * Ik / 1000;  //+ 0.001;  //e-3
	SoC[count_update_soc + 1] = SoC[count_update_soc] + (float) (*p_bms->current * UPDATE_SOC_STAMP * 2 / total_capacity) + 0.0001;   // e-4
	Uoc = bms_estimate_ocv(SoC[count_update_soc + 1]) * 15;  //15cell
	// f_1[k] = SoC[k] + delta(t)*Ik/total_capacity;
	// f_2[k] = -a1*Ud[k]+ b1*Ik; h[k] = Uoc[k]+Ud[k]
	// h[k] = Uoc[k]+Ud[k]

	if (count_update_soc == 1) {
		h[2] = Uoc + Ud[2];
		f_1[2] = SoC[2] + (float) (*p_bms->current * UPDATE_SOC_STAMP * 2 / total_capacity);
		f_2[2]= a1 * Ud[2] + b1 * Ik / 1000;
	}
	Ik = (float) *p_bms->current;  // (mA)
	Yk = Uoc + Ud[count_update_soc + 1] + 0.1;
// Corrector measure update:
	if (count_update_soc == 2) {
// the derivative matrices:
		SoC[1] = SoC[3];
		deltaSoc = (SoC[3] - SoC[2]);
		deltaUd = (Ud[3] - Ud[2]);
		if (deltaSoc == 0) {
			deltaSoc = 0.00001;
		}
		if (deltaUd == 0) {
			deltaUd = 0.0008856;
		}
		f_2[3]= a1 * Ud[3] + b1 * Ik / 1000;
		f_1[3] = SoC[3] + (float) (*p_bms->current * UPDATE_SOC_STAMP * 2 / total_capacity);
		Ik = (float) *p_bms->current;

		A1 = (f_1[3] - f_1[2]) / deltaSoc;
		A2 = (f_1[3] - f_1[2]) / deltaUd;


		A3 = (f_2[3] - f_2[2]) / deltaSoc;
		A4 = (f_2[3] - f_2[2]) / deltaUd;


		h[3] = Uoc + Ud[3];
		C1 = (h[3] - h[2]) / deltaSoc;
		C2 = (h[3] - h[2]) / deltaUd;


		A2 = -0.000001;
		A1 = 0.8;
		P1_TEMP = P1;
		P2_TEMP = P2;
		P3_TEMP = P3;  // SAVE VALUE CURRENT
		P1 = A1 * (A1 * P1 + A2 * P3) + A2 * (A1 * P2 + A2 * P4) + 0.0001;
		P2 = A3 * (A1 * P1_TEMP + A2 * P3) + A4 * (A1 * P2_TEMP + A2 * P4); // Qk = diag(0.000001,0.001)
		P3 = A1 * (A3 * P1_TEMP + A4 * P3_TEMP) + A2 * (A3 * P2_TEMP + A4 * P4);
		P4 = A3 * (A3 * P1_TEMP + A4 * P3_TEMP) + A4 * (A3 * P2_TEMP + A4 * P4)
				+ 0.001;
// calculate L
		L1_TEMP = C1 * (C1 * P1 + C2 * P3) + C2 * (C1 * P2 + C2 * P4) + 0.1;
		L1 = (P1 * C1 + P2 * C2) / L1_TEMP;
		L2 = (P3 * C1 + P4 * C2) / L1_TEMP;
// measurement update:
		SoC[1] = SoC[1] + L1 * (p_bms->pack_voltage * 1.0 / 1000 - Yk);
		Udk_run[1] = Udk_run[3];
//		Ud[1] = Ud[3];
		P1_TEMP = P1;
		P2_TEMP = P2;
		P3_TEMP = P3;  // SAVE VALUE CURRENT
		P1 = P1 * (1 - L1 * C1) + P3 * L1 * C2;
		P2 = P2 * (1 - L1 * C1) + P4 * L1 * C2;
		P3 = L2 * C1 * P1_TEMP + P3_TEMP * (1 - L2 * C2);
		P4 = L2 * C1 * P2_TEMP + P4 * (1 - L2 * C2);
		///////////////////////////////////////////////
		// send USART
		Yk_send = (int32_t) (Yk * 1000);
		p_bms->soc = SoC[1];  //UP
		SoC_estimate = (int32_t) (p_bms->soc * 1000);
		SoC_true = (int32_t) (SoC[0] * 1000);
		Ud_tmp = (int32_t) (Udk_run[1] * 1000);
		if (Ud_tmp < 0) {
			Ud_tmp_minus = Ud_tmp;
			Ud_tmp = -Ud_tmp;
		}     //time:
#if DEBUG_UART_SOC
		itoa(time_soc, buffer, 10);
		USART_Write_String(buffer);
		USART_Write_String("\t");
		time_soc += 2;   // over-> time_soc = 0;  [...] //0.4second
		// send SoC:
		USART_Write_String("Soc_est: ");
		itoa(SoC_estimate / 1000, buffer, 10);
		USART_Write_String(buffer);
		USART_Write_String(".");
		itoa((SoC_estimate % 1000) / 100, buffer, 10);
		USART_Write_String(buffer);
		itoa((SoC_estimate % 100) / 10, buffer, 10);
		USART_Write_String(buffer);
		itoa(SoC_estimate % 10, buffer, 10);
		USART_Write_String(buffer);
		USART_Write_String("\t");
		// send SoC_true:
		USART_Write_String("Soc_meas: ");
		itoa(SoC_true / 1000, buffer, 10);
		USART_Write_String(buffer);
		USART_Write_String(".");
		itoa((SoC_true % 1000) / 100, buffer, 10);
		USART_Write_String(buffer);
		itoa((SoC_true % 100) / 10, buffer, 10);
		USART_Write_String(buffer);
		itoa(SoC_true % 10, buffer, 10);
		USART_Write_String(buffer);
		USART_Write_String("\t");
		// send Ud:
		USART_Write_String("U_dk: ");
		if (Ud_tmp_minus < 0) {
			USART_Write_String("-");
		}
		itoa(Ud_tmp / 1000, buffer, 10);
		USART_Write_String(buffer);
		USART_Write_String(".");
		itoa((Ud_tmp % 1000) / 100, buffer, 10);
		USART_Write_String(buffer);
		itoa((Ud_tmp % 100) / 10, buffer, 10);
		USART_Write_String(buffer);
		itoa(Ud_tmp % 10, buffer, 10);
		USART_Write_String(buffer);
		USART_Write_String("\t");
		// send Ik:
		USART_Write_String("Ik: ");
		itoa(*p_bms->current, buffer, 10);
		USART_Write_String(buffer);
		USART_Write_String("\t");
		//send Yk:
		USART_Write_String("est_vol: ");
		itoa(Yk_send / 1000, buffer, 10);
		USART_Write_String(buffer);
		USART_Write_String(".");
		itoa((Yk_send % 1000) / 100, buffer, 10);
		USART_Write_String(buffer);
		itoa((Yk_send % 100) / 10, buffer, 10);
		USART_Write_String(buffer);
		itoa(Yk_send % 10, buffer, 10);
		USART_Write_String(buffer);
		USART_Write_String("\t");
		//send pack voltage :
		USART_Write_String("pack_vol: ");
		itoa(p_bms->pack_voltage / 1000, buffer, 10);
		USART_Write_String(buffer);
		USART_Write_String(".");
		itoa((p_bms->pack_voltage % 1000) / 100, buffer, 10);
		USART_Write_String(buffer);
		itoa((p_bms->pack_voltage % 100) / 10, buffer, 10);
		USART_Write_String(buffer);
		itoa(p_bms->pack_voltage % 10, buffer, 10);
		USART_Write_String(buffer);
		USART_Write_String("\n\r");
#endif
		count_update_soc -= 2;
	}
	count_update_soc++;
#endif

}

