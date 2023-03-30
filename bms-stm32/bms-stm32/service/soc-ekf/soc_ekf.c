/*
 * soc_ekf.c
 *
 *  Created on: Feb 4, 2021
 *      Author: Admin
 */
#if 0

#include "soc_ekf.h"
#include "model.h"
#include <stdio.h>
#include "stdbool.h"

void ekf_soc_init(ekf_data *p_ekf){
   p_ekf->input_current = 0;
   //p_ekf->input_voltage = 0;
   p_ekf->xhat_ir = ir0;
   p_ekf->xhat_hk = hk0;
   p_ekf->sigmaV = SigmaV;
   p_ekf->sigmaW = SigmaW;
   p_ekf->priorI = 0;
   p_ekf->signIk = 0;
   p_ekf->sigmaX_11 = SigmaX_11; p_ekf->sigmaX_12 = 0;         p_ekf->sigmaX_13= 0;
   p_ekf->sigmaX_21 = 0;         p_ekf->sigmaX_22 = SigmaX_22; p_ekf->sigmaX_33= 0;
   p_ekf->sigmaX_31 = 0;         p_ekf->sigmaX_32 = 0;         p_ekf->sigmaX_33= SigmaX_33;
}

// binary search
static int binary_search(float data, const float* array, int max_size){
    int lower_bound = 0;
    int upper_bound = max_size-1;
    int mid_point = -1;
    int index = -1;

    while(lower_bound <= upper_bound){
      // compute the mid point
      // midPoint = (lowerBound + upperBound) / 2;
      mid_point = lower_bound + (upper_bound - lower_bound) / 2;
      // data found
      if(array[mid_point] == data) {
         index = mid_point;
         break;
      } else {
         // if data is larger
         if(array[mid_point] > data) {
            // data is in upper half
            lower_bound = mid_point + 1;
         }
         // data is smaller
         else {
            // data is in lower half
            upper_bound = mid_point -1;
         }
      }
    }
    if (lower_bound > upper_bound){
    	index = lower_bound;
    }
   return index;
}

/* round floating point
uint32_t soc_is_round(float soc){
	uint32_t soc_rounded;
	 if((soc-(uint32_t)soc) <= 0.5){
		 soc_rounded = (uint32_t)soc;
	 }
	 else{
		 soc_rounded = (uint32_t)soc + 1;
	 }
	return soc_rounded;
}
*/
float SOC_from_OCV(float vk){
	int index = 0;
	if (vk > ocv_lut_values[0]){
		vk = ocv_lut_values[0];
	}
	if (vk < ocv_lut_values[EKF_OCV_LUT_SIZE-1]){
		vk = ocv_lut_values[EKF_OCV_LUT_SIZE-1];
	}

    index = binary_search(vk, ocv_lut_values, EKF_OCV_LUT_SIZE);
	return soc_lut_values[index];
}

static float OCV_from_SOC(float soc){
	int index = 0;
	if (soc > soc_lut_values[0]){
		soc = soc_lut_values[0];
	}
	if (soc < soc_lut_values[EKF_SOC_LUT_SIZE-1]){
		soc = soc_lut_values[EKF_SOC_LUT_SIZE-1];
	}

	index = binary_search(soc, soc_lut_values, EKF_SOC_LUT_SIZE);
	return ocv_lut_values[index];
}

static float dOCV_from_SOC(float soc){
	int index = 0;
	if (soc > soc_lut_values[0]){
		soc = soc_lut_values[0];
	}
	if (soc < soc_lut_values[EKF_SOC_LUT_SIZE-1]){
		soc = soc_lut_values[EKF_SOC_LUT_SIZE-1];
	}

	while((soc <= soc_lut_values[index]) && (index < EKF_SOC_LUT_SIZE)){
		index++;
	}
	return dOCV_LUT_TABLES[index];
}

//helper function
static float get_min(float value1, float value2){
    float min;
    min = (value1 < value2) ? value1 : value2;
	return min;
}
static float get_max(float value1, float value2){
	float max;
	max = (value1 > value2) ? value1: value2;
	return max;
}
// initial parameter
// Get data stored in ekfData structure
static void update_ekf_data(ekf_data* p_ekf){
	//p_ekf->xhat_zk = SOC_from_OCV(p_ekf->input_voltage);
	//p_ekf->priorI = p_ekf->input_current;
	 if(p_ekf->input_current < 0.0f){
	   p_ekf->input_current = p_ekf->input_current*ETAParam;
	 }
    p_ekf->signIk = (p_ekf->input_current >0.0f) ? 1.0f: 0.0f;
	p_ekf->vk = p_ekf->input_voltage;
}
// EKF step 0: Compute matrix Ahat and Bhat
static void init_Ahat(ekf_data* p_ekf){
	p_ekf->Ahat.Ahat_ir = RC;
	p_ekf->Ahat.Ahat_hk = Ah;
	p_ekf->Ahat.Ahat_zk= 1.0f;
}
static void init_B(ekf_data* p_ekf){
//	int signI;
//	signI = (ekf->priorI >0) ? 0 :1;
	p_ekf->Bhat.Bhat_ir = 1.0f-RC;
	//Bhat(hkInd) = -abs(G*deltat/(3600*Q))*Ah*(1+sign(I)*xhat(hkInd))
	p_ekf->Bhat.Bhat_hk = G_deltat_Q* Ah*(1.0f+ p_ekf->signIk*p_ekf->xhat_hk);
	p_ekf->Bhat.Bhat_zk = Bhat_zkInd;
	p_ekf->Bhat.B_hk2 = Ah-1.0f;
}
// EKF step 1a: State-prediction update
static void ekf_step1a(ekf_data* p_ekf){
   p_ekf->xhat_ir = p_ekf->Ahat.Ahat_ir*p_ekf->xhat_ir + p_ekf->Bhat.Bhat_ir*p_ekf->priorI;
   p_ekf->xhat_hk = p_ekf->Ahat.Ahat_hk*p_ekf->xhat_hk + p_ekf->Bhat.Bhat_hk*p_ekf->priorI + p_ekf->Bhat.B_hk2*p_ekf->signIk;
   p_ekf->xhat_zk = p_ekf->Ahat.Ahat_zk*p_ekf->xhat_zk + p_ekf->Bhat.Bhat_zk*p_ekf->priorI;
   //help maintain robust
   p_ekf->xhat_hk = get_min(1.0f, get_max(-1.0f, p_ekf->xhat_hk));
   p_ekf->xhat_zk = get_min(1.0f, get_max(0.0f, p_ekf->xhat_zk));
}
// EKF step 1b: Error-covariance time update
static void ekf_step1b(ekf_data* p_ekf){
	p_ekf->sigmaX_11 = p_ekf->Ahat.Ahat_ir*p_ekf->sigmaX_11*p_ekf->Ahat.Ahat_ir + p_ekf->Bhat.Bhat_ir*p_ekf->sigmaW*p_ekf->Bhat.Bhat_ir;
	p_ekf->sigmaX_12 = p_ekf->Ahat.Ahat_ir*p_ekf->sigmaX_12*p_ekf->Ahat.Ahat_hk + p_ekf->Bhat.Bhat_ir*p_ekf->sigmaW*p_ekf->Bhat.Bhat_hk;
	p_ekf->sigmaX_13 = p_ekf->Ahat.Ahat_ir*p_ekf->sigmaX_13*p_ekf->Ahat.Ahat_zk + p_ekf->Bhat.Bhat_ir*p_ekf->sigmaW*p_ekf->Bhat.Bhat_zk;

	p_ekf->sigmaX_21 = p_ekf->Ahat.Ahat_hk*p_ekf->sigmaX_21*p_ekf->Ahat.Ahat_ir + p_ekf->Bhat.Bhat_hk*p_ekf->sigmaW*p_ekf->Bhat.Bhat_ir;
	p_ekf->sigmaX_22 = p_ekf->Ahat.Ahat_hk*p_ekf->sigmaX_22*p_ekf->Ahat.Ahat_hk + p_ekf->Bhat.Bhat_hk*p_ekf->sigmaW*p_ekf->Bhat.Bhat_hk;
	p_ekf->sigmaX_23 = p_ekf->Ahat.Ahat_hk*p_ekf->sigmaX_23*p_ekf->Ahat.Ahat_zk + p_ekf->Bhat.Bhat_hk*p_ekf->sigmaW*p_ekf->Bhat.Bhat_hk;

    p_ekf->sigmaX_31 = p_ekf->Ahat.Ahat_zk*p_ekf->sigmaX_31*p_ekf->Ahat.Ahat_ir + p_ekf->Bhat.Bhat_zk*p_ekf->sigmaW*p_ekf->Bhat.Bhat_ir;
    p_ekf->sigmaX_32 = p_ekf->Ahat.Ahat_zk*p_ekf->sigmaX_32*p_ekf->Ahat.Ahat_hk + p_ekf->Bhat.Bhat_zk*p_ekf->sigmaW*p_ekf->Bhat.Bhat_hk;//bug
    p_ekf->sigmaX_33 = p_ekf->Ahat.Ahat_zk*p_ekf->sigmaX_33*p_ekf->Ahat.Ahat_zk + p_ekf->Bhat.Bhat_zk*p_ekf->sigmaW*p_ekf->Bhat.Bhat_zk;
}
// EKF step 1c: Predict system output
static void ekf_step1c(ekf_data* p_ekf){
    p_ekf->yhat = OCV_from_SOC(p_ekf->xhat_zk) + M0Param*p_ekf->signIk + MParam* p_ekf->xhat_hk
    		    - RParam*p_ekf->xhat_ir - R0Param*p_ekf->input_current;
}
// EKF step 2a: Estimate gain matrix Lk
static void init_Chat(ekf_data* p_ekf){
	p_ekf->Chat.Chat_ir = -RParam;
	p_ekf->Chat.Chat_hk = MParam;
	p_ekf->Chat.Chat_zk = dOCV_from_SOC(p_ekf->xhat_zk);
}
static void compute_sigmaY(ekf_data* p_ekf){
	p_ekf->sigmaY = (p_ekf->Chat.Chat_ir*p_ekf->sigmaX_11 + p_ekf->Chat.Chat_hk*p_ekf->sigmaX_21 + p_ekf->Chat.Chat_zk*p_ekf->sigmaX_31)*p_ekf->Chat.Chat_ir
			      + (p_ekf->Chat.Chat_ir*p_ekf->sigmaX_12 + p_ekf->Chat.Chat_hk*p_ekf->sigmaX_22 + p_ekf->Chat.Chat_zk*p_ekf->sigmaX_32)*p_ekf->Chat.Chat_hk
				  + (p_ekf->Chat.Chat_ir*p_ekf->sigmaX_13 + p_ekf->Chat.Chat_hk*p_ekf->sigmaX_23 + p_ekf->Chat.Chat_zk*p_ekf->sigmaX_33)*p_ekf->Chat.Chat_zk
				  + Dhat*p_ekf->sigmaV*Dhat;
	//help maintain robustness
	p_ekf->sigmaY = get_min(0.3f, get_max(0.18f, p_ekf->sigmaY));
}
static void ekf_step2a(ekf_data* p_ekf){
	init_Chat(p_ekf);
	compute_sigmaY(p_ekf);
	p_ekf->L.L1 = (p_ekf->sigmaX_11*p_ekf->Chat.Chat_ir + p_ekf->sigmaX_12*p_ekf->Chat.Chat_hk + p_ekf->sigmaX_13*p_ekf->Chat.Chat_zk)/p_ekf->sigmaY;
	p_ekf->L.L2 = (p_ekf->sigmaX_21*p_ekf->Chat.Chat_ir + p_ekf->sigmaX_22*p_ekf->Chat.Chat_hk + p_ekf->sigmaX_23*p_ekf->Chat.Chat_zk)/p_ekf->sigmaY;
	p_ekf->L.L3 = (p_ekf->sigmaX_31*p_ekf->Chat.Chat_ir + p_ekf->sigmaX_32*p_ekf->Chat.Chat_hk + p_ekf->sigmaX_33*p_ekf->Chat.Chat_zk)/p_ekf->sigmaY;
	//help maintain robustness
	p_ekf->L.L3 = get_min(0.05f, get_max(0.0f, p_ekf->L.L3));

}
// EKF step 2b: State-estimate measurement update
static void ekf_step2b(ekf_data* p_ekf){
	p_ekf->r = p_ekf->vk - p_ekf->yhat;
	if ((p_ekf->r*p_ekf->r) > 100.0f*p_ekf->sigmaY){
		p_ekf->L.L1 =0.0f;
		p_ekf->L.L2 =0.0f;
		p_ekf->L.L3 =0.0f;
	}
	p_ekf->r = get_min(2.5f, get_max(-2.5f, p_ekf->r));

	p_ekf->xhat_ir = p_ekf->xhat_ir + p_ekf->L.L1*p_ekf->r;
	p_ekf->xhat_hk = p_ekf->xhat_hk + p_ekf->L.L2*p_ekf->r;
	p_ekf->xhat_zk = p_ekf->xhat_zk + p_ekf->L.L3*p_ekf->r;
	   //help maintain robustness
    p_ekf->xhat_hk = get_min(1.0f, get_max(-1.0f, p_ekf->xhat_hk));
    p_ekf->xhat_zk = get_min(1.0f, get_max(0.0f, p_ekf->xhat_zk));
}
// EKF step 2c: Error-covariance measurement update
static void ekf_step2c(ekf_data* p_ekf){
	p_ekf->sigmaX_11 = p_ekf->sigmaX_11 - p_ekf->L.L1*p_ekf->sigmaY*p_ekf->L.L1;
	p_ekf->sigmaX_12 = p_ekf->sigmaX_12 - p_ekf->L.L1*p_ekf->sigmaY*p_ekf->L.L2;
	p_ekf->sigmaX_13 = p_ekf->sigmaX_13 - p_ekf->L.L1*p_ekf->sigmaY*p_ekf->L.L3;

	p_ekf->sigmaX_21 = p_ekf->sigmaX_21 - p_ekf->L.L2*p_ekf->sigmaY*p_ekf->L.L1;
	p_ekf->sigmaX_22 = p_ekf->sigmaX_22 - p_ekf->L.L2*p_ekf->sigmaY*p_ekf->L.L2;
	p_ekf->sigmaX_23 = p_ekf->sigmaX_23 - p_ekf->L.L2*p_ekf->sigmaY*p_ekf->L.L3;

	p_ekf->sigmaX_31 = p_ekf->sigmaX_31 - p_ekf->L.L3*p_ekf->sigmaY*p_ekf->L.L1;
	p_ekf->sigmaX_32 = p_ekf->sigmaX_32 - p_ekf->L.L3*p_ekf->sigmaY*p_ekf->L.L2;
	p_ekf->sigmaX_33 = p_ekf->sigmaX_33 - p_ekf->L.L3*p_ekf->sigmaY*p_ekf->L.L3;

	if ((p_ekf->r*p_ekf->r) > 4*p_ekf->sigmaY){
		p_ekf->sigmaX_33 = p_ekf->sigmaX_33*QBUMP;
		//printf("SigmaX Bump");
	}
	//help maintain robustness
	//p_ekf->sigmaX_11 = (p_ekf->sigmaX_11 + p_ekf->sigmaX_11)/2;
	p_ekf->sigmaX_12 = (p_ekf->sigmaX_12 + p_ekf->sigmaX_21)/2;
	p_ekf->sigmaX_13 = (p_ekf->sigmaX_13 + p_ekf->sigmaX_31)/2;

	p_ekf->sigmaX_21 = p_ekf->sigmaX_12;
	//p_ekf->sigmaX_22 = (p_ekf->sigmaX_22 + p_ekf->sigmaX_22)/2;
	p_ekf->sigmaX_23 = (p_ekf->sigmaX_23 + p_ekf->sigmaX_32)/2;

	p_ekf->sigmaX_31 = p_ekf->sigmaX_13;
	p_ekf->sigmaX_32 = p_ekf->sigmaX_23;
	//p_ekf->sigmaX_33 = (p_ekf->sigmaX_33 + p_ekf->sigmaX_33)/2;
}
// Save data in ekf_data structure for next time
static void save_data(ekf_data* p_ekf){
	p_ekf->priorI = p_ekf->input_current;
}
// run iterEKF
void ekf_soc_iter(ekf_data* p_ekf){
	update_ekf_data(p_ekf);
	init_Ahat(p_ekf);
	init_B(p_ekf);
	ekf_step1a(p_ekf);
	ekf_step1b(p_ekf);
	ekf_step1c(p_ekf);
	ekf_step2a(p_ekf);
	ekf_step2b(p_ekf);
	ekf_step2c(p_ekf);
	save_data(p_ekf);
}

#endif
