/*
 * soc_spkf.c
 *
 *  Created on: Aug 27, 2021
 *      Author: Admin
 */

#include <math.h>
#include "stdlib.h"
#include "model.h"
#include "stdint.h"
#include "soc_spkf.h"

#define ir 0
#define hk 1
#define zk 2
// helper function

static void spkf_step_1a_1(spkf_data* p_spkf);
static void spkf_step_1a_2(spkf_data *p_spkf);
static void spkf_step_1a_3(spkf_data *p_spkf);

static void spkf_step_1a(spkf_data *p_spkf);
static void spkf_step_1b(spkf_data *p_spkf);
static void spkf_step_1c(spkf_data *p_spkf);
static void spkf_step_2a(spkf_data *p_spkf);
static void spkf_step_2b(spkf_data *p_spkf);
static void spkf_step_2c(spkf_data *p_spkf);

static void update_data(spkf_data* p_spkf);
static void spkf_save_data(spkf_data *p_spkf);
static void state_equation(spkf_data *p_spkf, uint8_t n, uint8_t m, float xold[n][m], uint8_t x, uint8_t y, float xnoise[x][y], float xnew[n][m]);
static void output_equation(spkf_data *p_spkf, uint8_t n, uint8_t m, float xhat[n][m], uint8_t x, uint8_t y, float xnoise[x][y], float ynoise[x][y]);

/*
 * helper function
 */
static float get_min(float value1, float value2);
static float get_max(float value1, float value2);
static float get_ocv_from_soc(float soc);
//static float get_soc_from_ocv(float vk);

static int binary_search(float data, const float* array, int max_size);
float *cholesky(int n,float A[n][n]);
static float square_root(float x);
static float exponent(float x);
static int sign(float value);
static void multiply_matrix(uint8_t n, uint8_t m, uint8_t p, float matrix1[n][m], float matrix2[m][p],float result[n][p]);
static void scalar_multiply_matrix(uint8_t n, uint8_t m, float number, float matrix[n][m],float result[n][m]);
static void additon_matrix(uint8_t n, uint8_t m, float matrix1[n][m], float matrix2[n][m], float result[n][m]);
static void truncate_matrix(uint8_t n, uint8_t m, float src_matrix[n][m], uint8_t x, uint8_t y, float dst_matrix[x][y], uint8_t begin_row, uint8_t begin_col);
static void generate_matrix(uint8_t m, uint8_t n, float matrix[n][m], float value);
static void transpose_matrix(uint8_t n, uint8_t m, float src_matrix[n][m], float dst_matrix[m][n]);
static void diag_matrix_col(uint8_t m, float src_matrix[1][m], float dst_matrix[m][m]);

float absolute(float x)
{
     if (x < 0)
         x = -x;
     return x;
}

static float square_root(float x)
{
    float guess = 1;

    while(absolute((guess*guess)/ x - 1) >= 0.0001 )
        guess = ((x/guess) + guess) / 2;

    return guess;
}

#if 0
/*
 "safe square root
 */
static void sqrt_safe(float x)
{
	x = square_root(get_max(0,x));
}
#endif
static float exponent(float x)
{
    return (1 + x + x*x/2);             // Taylor approximate
}

static int sign(float value)
{
	return (value == 0) ? 0 :((value >0) ? 1:-1);
}
static float get_min(float value1, float value2)
{
    float min;
    min = (value1 < value2) ? value1 : value2;
	return min;
}
static float get_max(float value1, float value2)
{
	float max;
	max = (value1 > value2) ? value1: value2;
	return max;
}

static int binary_search(float data, const float* array, int max_size)
{
    int lower_bound = 0;
    int upper_bound = max_size-1;
    int mid_point = -1;
    int index = -1;

    while(lower_bound <= upper_bound)
    {
      // compute the mid point
      // midPoint = (lowerBound + upperBound) / 2;
      mid_point = lower_bound + (upper_bound - lower_bound) / 2;
      // data found
      if(array[mid_point] == data)
      {
         index = mid_point;
         break;
      } else
      {
         // if data is larger
         if(array[mid_point] > data)
         {
            // data is in upper half
            lower_bound = mid_point + 1;
         }
         // data is smaller
         else
         {
            // data is in lower half
            upper_bound = mid_point -1;
         }
      }
    }
    if (lower_bound > upper_bound)
    {
    	index = upper_bound;
    }
   return index;
}

float *cholesky(int n,float A[n][n])
{
    uint8_t i, j, k;
    float *L = (float*)calloc(n * n, sizeof(float));
    if (L == NULL)
        return 0;

    for (i = 0; i < n; i++)
        for (j = 0; j < (i+1); j++)
    {
            float s = 0;
            for ( k = 0; k < j; k++)
            {
                s += L[i * n + k] * L[j * n + k];
            }
            L[i * n + j] = (i == j) ? square_root(A[i][j] - s) : (1.0 / L[j * n + j] * (A[i][j] - s));
    }

    return L;
}

/*
 * 	Ensure all value of matrix is initial
 *  Function multiply two matrix with argument:
 * 		 n: number of row 1 	 m: number of column 1 	 p: number of column2
 *  	 matrix1: matrix n*m
 *  	 matrix2: matrix m*p
 *  	 result: matrix reuslt n*p
 */
static void multiply_matrix(uint8_t n, uint8_t m, uint8_t p, float matrix1[n][m], float matrix2[m][p],float result[n][p])
{
	uint8_t i,j,k;
	float sum;
    for(i = 0; i < n; ++i)
    {
        for(j=0; j < p; ++j)
        {
            sum=0;
            for(k =0; k < m; ++k)
            {
                sum+=matrix1[i][k]*matrix2[k][j];
            }
            result[i][j] = sum;
        }
    }

}
static void scalar_multiply_matrix(uint8_t n, uint8_t m, float number, float matrix[n][m],float result[n][m])
{
	uint8_t i,j;
    for(i = 0; i < n; ++i)
    {
        for(j = 0; j < m; ++j)
        {
            result[i][j]= number*matrix[i][j];
        }
    }

}
/*
 * 	Ensure all value of matrix is initial
 *  function matrix addition with argument:
 *  n: number of row, m: number of column
 *  matrix1: matrix n*m
 *  matrix2: matrix n*m
 *  result: matrix reuslt n*m
 */
static void additon_matrix(uint8_t n, uint8_t m, float matrix1[n][m], float matrix2[n][m], float result[n][m])
{
	uint8_t i,j;
    for (i=0; i < n; i++)
    {
      for (j =0; j <m; j++)
      {
    	  //result[i][j] =0; // ensure value of matrix is initial
          result[i][j] = matrix1[i][j] + matrix2[i][j];
      }
    }
}

static void truncate_matrix(uint8_t n, uint8_t m, float src_matrix[n][m], uint8_t x, uint8_t y, float dst_matrix[x][y], uint8_t begin_row, uint8_t begin_col)
{
	uint8_t i, j;
	for (i = begin_row; i < (begin_row+x); i++)
	{
		for (j = begin_col; j <(begin_col+ y); j++)
		{
			dst_matrix[i-begin_row][j-begin_col] = src_matrix[i][j];
		}
	}
}

static void transpose_matrix(uint8_t n, uint8_t m, float src_matrix[n][m], float dst_matrix[m][n]){
	uint8_t i,j;
	for(i = 0; i< n; i++)
	{
		for(j = 0; j < m; j++)
		{
			dst_matrix[j][i] = src_matrix[i][j];
		}
	}
}

static void diag_matrix_col(uint8_t m, float src_matrix[m][1], float dst_matrix[m][m])
{
	uint8_t i,j;
	for(i = 0; i< m; i++){
		for(j = 0; j< m; j++)
		{
			dst_matrix[i][j] = (i==j)?src_matrix[i][0]:0;
		}
	}
}

/*
 * generate_matrix with each component is same value
 */
static void generate_matrix(uint8_t m, uint8_t n, float matrix[n][m], float value)
{
	uint8_t i,j;
    for(i=0; i <n; i++)
    {
        for(j=0;j<m;j++)
        {
            matrix[i][j] = value;
        }
    }
}

//static float get_soc_from_ocv(float vk)
float get_soc_from_ocv(float vk){
    int index = 0;
    if (vk > ocv_lut_values[0])
    {
        vk = ocv_lut_values[0];
    }
    if (vk < ocv_lut_values[SPKF_OCV_LUT_SIZE-1])
    {
        vk = ocv_lut_values[SPKF_OCV_LUT_SIZE-1];
    }

    index = binary_search(vk, ocv_lut_values, SPKF_OCV_LUT_SIZE);
    return soc_lut_values[index];
}

static float get_ocv_from_soc(float soc){
    int index = 0;
    if (soc > soc_lut_values[0]){
        soc = soc_lut_values[0];
    }
    if (soc < soc_lut_values[SPKF_SOC_LUT_SIZE-1]){
        soc = soc_lut_values[SPKF_SOC_LUT_SIZE-1];
    }

    index = binary_search(soc, soc_lut_values, SPKF_SOC_LUT_SIZE);
    return ocv_lut_values[index];
}
static void update_data(spkf_data* p_spkf){
	 if(p_spkf->input_current < 0.0f){
	   p_spkf->input_current = p_spkf->input_current*ETAParam;
	 }
    p_spkf->signIk = (p_spkf->input_current >0.0f) ? 1.0f: 0.0f;
    p_spkf->vk = p_spkf->input_voltage;
}


void spkf_soc_init(spkf_data* p_spkf){
    p_spkf->xhat[ir][0] =0;
    p_spkf->xhat[hk][0] =0;
    p_spkf->xhat[zk][0] = get_soc_from_ocv(p_spkf->input_voltage);
    p_spkf->input_current =0;
    // covariance value
    p_spkf->sigmaV = SigmaV;
    p_spkf->sigmaW = SigmaW;
    p_spkf->sigmaX[ir][ir] = SigmaX_11;     p_spkf->sigmaX[0][1] =0;                 p_spkf->sigmaX[0][2] =0;
    p_spkf->sigmaX[1][0] =0;                p_spkf->sigmaX[hk][hk] = SigmaX_22;      p_spkf->sigmaX[1][2] =0;
    p_spkf->sigmaX[2][0] =0;                p_spkf->sigmaX[2][1] =0;                 p_spkf->sigmaX[zk][zk] = SigmaX_33;
    p_spkf->S_noise_11 = Snoise;
    p_spkf->S_noise_22 = Snoise;
    //SPFK specific parameters
    p_spkf->Wm[0][0] = Weight1;
    p_spkf->Wc[0][0] = Weight1;
    uint8_t i;
    for(i =1; i <(2*Na+1); i++){
        p_spkf->Wm[i][0] = Weight2;
        p_spkf->Wc[i][0] = Weight2;
    }
    //previous value of current
    p_spkf->priorI = 0;
    p_spkf->signIk = 0;
}

// Get data stored in spkfData structure

 /*
  % Step 1a: State estimate time update
  %          - Create xhatminus augmented SigmaX points
  %          - Extract xhatminus state SigmaX points
  %          - Compute weighted average xhatminus(k)
  */

/*
 % Step 1a-1: Create augmented SigmaX and xhat
 */
static void spkf_step_1a_1(spkf_data* p_spkf)
{
	float *c1 = cholesky(3, p_spkf->sigmaX);
	p_spkf->sigma_Xa.sigma_Xa_11 = *(c1);
	p_spkf->sigma_Xa.simga_Xa_21 = *(c1+3);
	p_spkf->sigma_Xa.sigma_Xa_22 = *(c1+4);
	p_spkf->sigma_Xa.simga_Xa_31 = *(c1+5);
	p_spkf->sigma_Xa.sigma_Xa_32 = *(c1+6);
	p_spkf->sigma_Xa.sigma_Xa_33 = *(c1+8);
	p_spkf->sigma_Xa.p = *(c1+2);
	free(c1);

	if(p_spkf->sigma_Xa.p >0 )
	{
		p_spkf->sigma_Xa.sigma_Xa_11 = square_root(p_spkf->sigmaX[ir][ir]);
		p_spkf->sigma_Xa.sigma_Xa_22 = square_root(p_spkf->sigmaX[hk][hk]);
		p_spkf->sigma_Xa.sigma_Xa_33 = square_root(p_spkf->sigmaX[zk][zk]);
	}

	p_spkf->sigma_Xa.sigma_Xa_44 = Snoise;
	p_spkf->sigma_Xa.sigma_Xa_55 = Snoise;

}

/*
  % Step 1a-2: Calculate SigmaX points (strange indexing of xhata to
  % avoid "repmat" call, which is very inefficient in MATLAB)
 */
static void spkf_step_1a_2(spkf_data *p_spkf)
{
	//	spkf_step_1a_2
    float matrix_xhata[5][1] = {{p_spkf->xhat[ir][0]},{p_spkf->xhat[hk][0]},{p_spkf->xhat[zk][0]},{0},{0}};
    float matrix_ones[1][11] = {{1,1,1,1,1,1,1,1,1,1,1}};
    float Xc[5][11] = { {0, p_spkf->sigma_Xa.sigma_Xa_11,0, 0, 0, 0,-p_spkf->sigma_Xa.sigma_Xa_11, 0, 0, 0, 0},
    					{0, 0, p_spkf->sigma_Xa.sigma_Xa_22,0, 0, 0, 0,-p_spkf->sigma_Xa.sigma_Xa_22, 0, 0, 0},
						{0, 0, 0, p_spkf->sigma_Xa.sigma_Xa_33,0, 0, 0, 0,-p_spkf->sigma_Xa.sigma_Xa_33, 0, 0},
						{0, 0, 0, 0, p_spkf->sigma_Xa.sigma_Xa_44,0, 0, 0, 0,-p_spkf->sigma_Xa.sigma_Xa_44, 0},
						{0, 0, 0, 0, 0, p_spkf->sigma_Xa.sigma_Xa_55,0, 0, 0, 0,-p_spkf->sigma_Xa.sigma_Xa_55}};
    scalar_multiply_matrix(5, 11, H_Param, Xc, Xc);
    multiply_matrix(5, 1, 11, matrix_xhata, matrix_ones, p_spkf->Xa);
    additon_matrix(5, 11, p_spkf->Xa, Xc, p_spkf->Xa);
}
/*
 % Step 1a-3: Time update from last iteration until now
  %     stateEqn(xold,current,xnoise)
 */
static void spkf_step_1a_3(spkf_data *p_spkf)
{
	float xold[3][11] ={0};
	truncate_matrix(5, 11, p_spkf->Xa, 3, 11, xold, 0, 0);
	truncate_matrix(5, 11, p_spkf->Xa, 1, 11, p_spkf->xnoise, 3, 0);
	state_equation(p_spkf, 3, 11, xold, 1, 11, p_spkf->xnoise, p_spkf->Xx); //Xx bug
	multiply_matrix(3, 11, 1, p_spkf->Xx, p_spkf->Wm, p_spkf->xhat);
    //help maintain robust
    p_spkf->xhat[hk][0] = get_min(1.0f, get_max(-1.0f, p_spkf->xhat[hk][0]));
    p_spkf->xhat[zk][0] = get_min(1.0f, get_max(0.0f, p_spkf->xhat[zk][0]));

}

void spkf_step_1a(spkf_data *p_spkf)
{
	spkf_step_1a_1(p_spkf);
	spkf_step_1a_2(p_spkf);
	spkf_step_1a_3(p_spkf);
}

/*
  % Step 1b: Error covariance time update
  %          - Compute weighted covariance sigmaminus(k)
  %            (strange indexing of xhat to avoid "repmat" call)
 */
void spkf_step_1b(spkf_data *p_spkf)
{
	float X_temp[1][11] ={0};
	float Xs_transpose[11][3] ={0};
	float sigmaX_temp[3][11] ={0};
	//float X_temp[1][11] = {{1,1,1,1,1,1,1,1,1,1,1,}}; //
	generate_matrix(1, 11, X_temp, -1);
	multiply_matrix(3, 1, 11, p_spkf->xhat, X_temp, p_spkf->Xs);
	additon_matrix(3, 11, p_spkf->Xx, p_spkf->Xs, p_spkf->Xs);

	diag_matrix_col(11, p_spkf->Wc, p_spkf->diagWc);
	transpose_matrix(3, 11, p_spkf->Xs, Xs_transpose);
	multiply_matrix(3, 11, 11, p_spkf->Xs, p_spkf->diagWc, sigmaX_temp);
	multiply_matrix(3, 11, 3, sigmaX_temp, Xs_transpose, p_spkf->sigmaX);
}

/*
 % Step 1c: Output estimate
  %          - Compute weighted output estimate yhat(k)
 */
void spkf_step_1c(spkf_data *p_spkf)
{
	truncate_matrix(5, 11, p_spkf->Xa, 1, 11, p_spkf->ynoise, 4, 0);
	output_equation(p_spkf, 3, 11, p_spkf->Xx, 1, 11, p_spkf->xnoise, p_spkf->ynoise);
	multiply_matrix(1, 11, 1, p_spkf->yhat_matrix, p_spkf->Wm, p_spkf->yhat);
}
/*
 % Step 2a: Estimator gain matrix
 */
static void spkf_step_2a(spkf_data *p_spkf)
{
	float Yhat_temp[1][11] ={0};
	float Ys[1][11] ={0};
	float sigmaXY[3][1] ={0};
	float sigmaXY_temp[3][11] ={0};
	float sigmaY_temp[1][11] = {0};
	float Ys_transpose[11][1] ={0};

	generate_matrix(1, 11, Yhat_temp, 1); // addtion
	scalar_multiply_matrix(1, 11, -p_spkf->yhat[0][0], Yhat_temp, Yhat_temp); //bug
	additon_matrix(1, 11, p_spkf->yhat_matrix, Yhat_temp, Ys);

	transpose_matrix(11, 1, Ys, Ys_transpose);
	multiply_matrix(3, 11, 11, p_spkf->Xs, p_spkf->diagWc, sigmaXY_temp);
	multiply_matrix(3, 11, 1, sigmaXY_temp, Ys_transpose, sigmaXY);

	multiply_matrix(1, 11, 11, Ys, p_spkf->diagWc, sigmaY_temp);
	multiply_matrix(1, 11, 1, sigmaY_temp, Ys_transpose, p_spkf->sigmaY);

	scalar_multiply_matrix(3, 1, 1.0/p_spkf->sigmaY[0][0], sigmaXY, p_spkf->L);
}

/*
 % Step 2b: State estimate measurement update
 */
static void spkf_step_2b(spkf_data *p_spkf)
{
	float L_r_temp[3][1] ={0};
	p_spkf->r = p_spkf->vk - p_spkf->yhat[0][0];
	if(p_spkf->r*p_spkf->r > 100*p_spkf->sigmaY[0][0])
	{
		p_spkf->L[ir][0] =0;
		p_spkf->L[hk][0] =0;
		p_spkf->L[zk][0] =0;
	}
	scalar_multiply_matrix(3, 1, p_spkf->r, p_spkf->L, L_r_temp);
	additon_matrix(3, 1, p_spkf->xhat, L_r_temp, p_spkf->xhat);
	   //help maintain robustness
	p_spkf->xhat[zk][0] = get_min(1.0f, get_max(-1.0f, p_spkf->xhat[zk][0]));
	p_spkf->xhat[zk][0] = get_min(1.0f, get_max(0.0f, p_spkf->xhat[zk][0]));
}

/*
  % Step 2c: Error covariance measurement update
  */
static void spkf_step_2c(spkf_data *p_spkf)
{
	float L_transpose[1][3] ={0};
	transpose_matrix(3, 1, p_spkf->L, L_transpose);
	uint8_t i,j;
	for (i=0; i < 3; i++)
	{
		for(j =0; j <3; j++)
		{
			p_spkf->sigmaX[i][j] = p_spkf->sigmaX[i][j] -p_spkf->L[i][0]*p_spkf->sigmaY[0][0]*L_transpose[0][j];
		}
	}
	//help maintain robustness

	p_spkf->sigmaX[0][1] = (p_spkf->sigmaX[0][1] + p_spkf->sigmaX[1][0])/2;
	p_spkf->sigmaX[0][2] = (p_spkf->sigmaX[0][2] + p_spkf->sigmaX[2][0])/2;

	p_spkf->sigmaX[1][0] = p_spkf->sigmaX[0][1];

	p_spkf->sigmaX[1][2] = (p_spkf->sigmaX[1][2] + p_spkf->sigmaX[2][1])/2;

	p_spkf->sigmaX[2][0] = p_spkf->sigmaX[0][2];
	p_spkf->sigmaX[2][1] = p_spkf->sigmaX[1][2];

	//  Q-bump code
	if ((p_spkf->r*p_spkf->r) > 4*p_spkf->sigmaY[0][0])
	{
		p_spkf->sigmaX[zk][zk] = p_spkf->sigmaX[zk][zk]*QBUMP;
	}
}

/*
 % Save data in spkfData structure for next time...
 */
static void spkf_save_data(spkf_data *p_spkf)
{
	p_spkf->priorI = p_spkf->input_current;
}

/*
% Calculate new states for all of the old state vectors in xold.
*/
static void state_equation(spkf_data *p_spkf, uint8_t n, uint8_t m, float xold[n][m], uint8_t x, uint8_t y, float xnoise[x][y], float xnew[n][m])
{
	float current[1][11] ={0};
	float Ah[1][11] ={0};
	generate_matrix(x, y, current, p_spkf->priorI);
	generate_matrix(n, m, xnew, 0);
	additon_matrix(x, y, current, xnoise, current);
	uint8_t i;
	for (i =0; i < m; i++)
	{
		xnew[0][i] = RC*xold[0][i] + (1-RC)*current[0][i];
		Ah[0][i]   = exponent(-absolute(current[0][i]*GParam*deltat/(3600*QParam)));
		xnew[1][i] = Ah[0][i]*xold[1][i] + (Ah[0][i] - 1)* sign(current[0][i]);
		xnew[2][i] = xold[2][i] - current[0][i]*deltat/(3600*QParam);
	}
}

/*
  % Calculate cell output voltage for all of state vectors in xhat
  */
static void output_equation(spkf_data *p_spkf, uint8_t n, uint8_t m, float xhat[n][m], uint8_t x, uint8_t y, float xnoise[x][y], float ynoise[x][y])
{
	float current[1][11] ={0};
	uint8_t i;
	generate_matrix(x, y, current, p_spkf->input_current);
	additon_matrix(x, y, current, xnoise, current);
	p_spkf->yhat_matrix[0][0] = get_ocv_from_soc(xhat[zk][3]);
	generate_matrix(x, y, p_spkf->yhat_matrix,p_spkf->yhat_matrix[0][0]);
	for(i = 0; i < 11; i++)
	{
	//	p_spkf->yhat_matrix[0][i] = get_ocv_from_soc(xhat[zk][i]);
		p_spkf->yhat_matrix[0][i] = p_spkf->yhat_matrix[0][i] + MParam*xhat[hk][i] + M0Param*p_spkf->signIk+ ynoise[0][i];
		p_spkf->yhat_matrix[0][i] = p_spkf->yhat_matrix[0][i] -RParam*xhat[ir][i] - R0Param*current[0][i];
	}
}


void spkf_soc_iter(spkf_data *p_spkf)
{
	update_data(p_spkf);
	spkf_step_1a(p_spkf);
	spkf_step_1b(p_spkf);
	spkf_step_1c(p_spkf);
	spkf_step_2a(p_spkf);
	spkf_step_2b(p_spkf);
	spkf_step_2c(p_spkf);
	spkf_save_data(p_spkf);

}
