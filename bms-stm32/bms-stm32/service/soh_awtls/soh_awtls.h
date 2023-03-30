/*
 * soh_awtls.h
 *
 *  Created on: Nov 4, 2021
 *      Author: Admin
 */

#ifndef SERVICE_SOH_AWTLS_SOH_AWTLS_H_
#define SERVICE_SOH_AWTLS_SOH_AWTLS_H_

#define sigma_X0_square    0.000196f
#define sigma_Y0_square    0.00000121f
#define sigma_X0 		   0.014f
#define sigma_Y0           0.0011f
#define K 				   12.7273f
#define Q_NOMINAL_CAPACITY 4.0f
#define eta      		   1
#define DELTAT_SOH		   1


typedef struct soh_awtls_t soh_awtls;

struct soh_awtls_t{
    float measureX; // delta soc = z[k2] - z[k1]
    float measureY; // accumlated I*deltat
    float sigma_X;
    float sigma_Y;
    float C1;
    float C2;
    float C3;
    float C4;
    float C5;
    float C6;
    float A,B,C,D,E;
    float root[4];
    float result[4];
    float root_min;
    float Q_hat;
    float Q_nom;
    float Sigma_Q;
    float soh_estimate;
};

typedef struct complex_t complex;
struct complex_t{
	float real;
	float image;
};

void soh_awtls_init(soh_awtls* p_data);
void soh_awtls_iter(soh_awtls* p_data);


#endif /* SERVICE_SOH_AWTLS_SOH_AWTLS_H_ */
