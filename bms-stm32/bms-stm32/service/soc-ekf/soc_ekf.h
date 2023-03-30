/*
 * soc-ekf.h
 *
 *  Created on: Feb 4, 2021
 *      Author: Admin
 */
#if 0

#ifndef SERVICE_SOC_EKF_SOC_EKF_H_
#define SERVICE_SOC_EKF_SOC_EKF_H_

#include "model.h"
#include <stdio.h>
#include "stdbool.h"

//initial parameter for compute EKF
#define ir0     0.0f
#define hk0 	0.0f
#define QBUMP   1.03f
#define SigmaV  0.2f //Uncertainty of voltage sensor, output equation
#define SigmaW  0.2f //Uncertainty of current sensor, state equation
#define SigmaX_11 0.000001f   // uncertainty of initial state
#define SigmaX_22 0.00000001f
#define SigmaX_33 0.0002f
#define deltat 0.62f     // time interval: deltat = time(2) - time(1)
#define RC     0.7732f // RC = exp(-deltat/RCParam)
#define Ah     1.0f    // hysteresis factor Ah = exp(-abs(G*I*deltat)/(3600*Q))
#define Bhat_zkInd  -0.000011f //-deltat/(3600*Q);
#define Dhat  1.0f
#define G_deltat_Q 0.0f //abs(G*deltat/(3600*Q))

typedef struct ekf_data_t ekf_data;
struct ekf_data_t{
	//input
	float input_current;
	float input_voltage;
	// Covariance matrix values
	float sigmaX_11, sigmaX_12, sigmaX_13;
	float sigmaX_21, sigmaX_22, sigmaX_23;
	float sigmaX_31, sigmaX_32, sigmaX_33;
	float sigmaW, sigmaV;
	//previous value of current
	float priorI;
	float signIk;
	float vk;
	// ir hk zk vector for diffusion current, hysteresis, and SOC
	float xhat_ir, xhat_hk, xhat_zk;
	float yhat;
	float sigmaY;
    L  L;
    Ahat Ahat;
    Bhat Bhat;
    Chat Chat;
    float r; // measurement innovation
};
//uint32_t soc_is_round(float soc);
void ekf_soc_init(ekf_data *p_ekf);
void ekf_soc_iter(ekf_data* p_ekf);
float SOC_from_OCV(float vk);
#endif /* SERVICE_SOC_EKF_SOC_EKF_H_ */

#endif
