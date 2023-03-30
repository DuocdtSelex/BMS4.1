/*
 * soc_spkf.h
 *
 *  Created on: Aug 27, 2021
 *      Author: Admin
 */

#ifndef SERVICE_SOC_SPKF_SOC_SPKF_H_
#define SERVICE_SOC_SPKF_SOC_SPKF_H_

#include <stdio.h>
#include "stdbool.h"
#include "model_battery.h"

/*
 *  Initial parameters for compute SPKF
 */
#define ir0         0.0f
#define hk0         0.0f
#define QBUMP       5
#define SigmaV      0.2f 			 //Uncertainty of voltage sensor, output equation
#define SigmaW      0.2f			 //Uncertainty of current sensor, state equation
#define SigmaX_11   0.000001f  		 // uncertainty of initial state
#define SigmaX_22   0.00000001f
#define SigmaX_33   0.0002f
#define Snoise   	0.4472f

/*
 *  SPKF specific parameters
 */
#define Nx 			3 				/* state vector length */
#define Ny 			1				/* measurement vector length */
#define Nu 			1				/* input-vector length */
#define Nw  		1				/* process-noise-vector length */
#define Nv			1				/* sensor-noise-vector length */
#define Na 			5				/* augmented-state-vector length */
#define H_Param	1.73205f		/* SPFK/CDKF tuning factor */
#define Weight1 	(H_Param*H_Param - Na)/(H_Param*H_Param)/* weighting factors when computing mean */
#define Weight2		1/(2*H_Param*H_Param)		/* and covariance */

/*
 *  parameters model
 */
#define deltat      1.0f     		// time interval: deltat = time(2) - time(1)
#define RC          0.66046f 		// RC = exp(-deltat/RCParam)

float *cholesky(int n,float A[n][n]);

typedef struct sigma_Xa_t sigma_Xa;
struct sigma_Xa_t{
	float sigma_Xa_11;
	float sigma_Xa_22;
	float simga_Xa_21;
	float simga_Xa_31;
	float sigma_Xa_32;
	float sigma_Xa_33;
	float sigma_Xa_44;
	float sigma_Xa_55;
	float p;
};

typedef struct spkf_data_t spkf_data;
struct spkf_data_t{
	float input_voltage;
	float input_current;
	float vk;
    float priorI;
    float signIk;
    float r;
    sigma_Xa sigma_Xa;
	float yhat_matrix[1][11];
	float yhat[1][1];
	float xhat[3][1];
	float sigmaX[3][3];
    float sigmaV, sigmaW;
    float S_noise_11;
    float S_noise_22;
    float Wm[11][1];
    float Wc[11][1];
    float Xa[5][11];
    float Xx[3][11];
    float Xs[3][11];
    float diagWc[11][11];
    float xnoise[1][11];
    float ynoise[1][11];
    float L[3][1];
    float sigmaY[1][1];
};


void spkf_soc_init(spkf_data *p_spkf);
void spkf_soc_iter(spkf_data *p_spkf);
float get_soc_from_ocv(float vk);

#endif /* SERVICE_SOC_SPKF_SOC_SPKF_H_ */
