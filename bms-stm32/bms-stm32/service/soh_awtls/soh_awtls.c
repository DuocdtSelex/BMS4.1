/*
 * soh_awtls.c
 *
 *  Created on: Nov 4, 2021
 *      Author: Admin
 */

#include "soh_awtls.h"

static void iter_soh_update_parameter(soh_awtls* p_data);
static void slove_quartic(soh_awtls* p_data);
static float absolute(float x);
static float square_root(float x);
static float diff(float x, float y);
static float cube_root(float x);
static float get_min(float value1, float value2);
static float get_max(float value1, float value2);
static void compute_Qhat(soh_awtls* p_data);


static float absolute(float x)
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

static float diff(float x, float y)
{
    if (x > (y*y*y))
        return (x-(y*y*y));
    else
        return ((y*y*y) - x);
}

static float cube_root(float x)
{
    float start=0, end=x;
    float mid = (start + end)/2;
    float error = diff(x, mid);
    if (x>0)
    {
    	while (error > 0.001)
    	{
		// If mid*mid*mid is greater than x set
		// end = mid
    		if ((mid*mid*mid) > x)
			{
				end = mid;
			}
			// If mid*mid*mid is less than x set
			// start = mid
			else
			{
				start = mid;
			}
			mid = (start + end)/2;
			error = diff(x, mid);
       }
		// If error is less than e then mid is
		// our answer so return mid
		if (error <= 0.001)
		{
			return mid;
		}
    }
    else
    {
    	while (error > 0.001)
			{
				// If mid*mid*mid is greater than x set  end = mid
				if ((mid*mid*mid) < x)
				{
					end = mid;
				}
				// If mid*mid*mid is less than x set start = mid
				else
				{
					start = mid;
				}
				mid = (start + end)/2;
				error = diff(x, mid);
			}
			// If error is less than e then mid is our answer so return mid
		if (error <= 0.001)
			{
				return mid;
			}
    }
    return mid;
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

void soh_awtls_init(soh_awtls* p_data)
{
    p_data->sigma_X = sigma_X0_square;
    p_data->sigma_Y = sigma_Y0_square;
    p_data->Q_nom   = Q_NOMINAL_CAPACITY;
    p_data->C1 = 0;
    p_data->C3 = 0;
    p_data->C4 = 0;
    p_data->C5 = 0;
    p_data->C6 = 0;

    p_data->C1 = 1.0f/(K*K*p_data->sigma_Y);
    p_data->C2 = (K*p_data->Q_nom)/(K*K*p_data->sigma_Y);
    p_data->C3 = (K*K*p_data->Q_nom*p_data->Q_nom)/(K*K*p_data->sigma_Y);
    p_data->C4 = 1.0f/(p_data->sigma_X);
    p_data->C5 = (K*p_data->Q_nom)/p_data->sigma_X;
    p_data->C6 = (K*K*p_data->Q_nom*p_data->Q_nom)/(p_data->sigma_X);
}

static void iter_soh_update_parameter(soh_awtls* p_data)
{
    // Update parameter
    p_data->C1 = eta*p_data->C1 + (p_data->measureX*p_data->measureX)/(K*K*p_data->sigma_Y);
    p_data->C2 = eta*p_data->C2 + (K*p_data->measureX*p_data->measureY)/(K*K*p_data->sigma_Y);
    p_data->C3 = eta*p_data->C3 + (K*K*p_data->measureY*p_data->measureY)/(K*K*p_data->sigma_Y);
    p_data->C4 = eta*p_data->C4 + (p_data->measureX*p_data->measureX)/(p_data->sigma_X);
    p_data->C5 = eta*p_data->C5 + (K*p_data->measureX*p_data->measureY)/(p_data->sigma_X);
    p_data->C6 = eta*p_data->C6 + (K*K*p_data->measureX*p_data->measureX)/(p_data->sigma_X);

    //update quartic parameter
    p_data->A = p_data->C5;
    p_data->B = -p_data->C1 + 2*p_data->C4 - p_data->C6;
    p_data->C = 3*p_data->C2 - 3*p_data->C5;
    p_data->D = p_data->C1 -2*p_data->C3 + p_data->C6;
    p_data->E = -p_data->C2;
}


void soh_awtls_iter(soh_awtls* p_data)
{
    iter_soh_update_parameter(p_data);
    slove_quartic(p_data);
    compute_Qhat(p_data);
}
static void slove_quartic(soh_awtls* p_data)
{
    float alpha = 0;
    float beta = 0;
    float gamma = 0;
    float P = 0;
    float Q_1 = 0;
    float r =0;
    float q =0;
    float delta = 0;

    alpha = p_data->C/p_data->A - 3*p_data->B*p_data->B/(8*p_data->A*p_data->A);
    beta = (p_data->B*p_data->B*p_data->B)/(8*p_data->A*p_data->A*p_data->A)
    		-(p_data->B*p_data->C)/(2*p_data->A*p_data->A) + p_data->D/p_data->A;
    gamma = (-3*p_data->B*p_data->B*p_data->B*p_data->B)/(256*p_data->A*p_data->A*p_data->A*p_data->A)
    		+(p_data->C*p_data->B*p_data->B)/(16*p_data->A*p_data->A*p_data->A)
			- (p_data->B*p_data->D)/(4*p_data->A*p_data->A) + p_data->E/p_data->A;

    P = -(alpha*alpha)/12.0f - gamma;
    Q_1 = -(alpha*alpha*alpha)/108.0f + (alpha*gamma)/3.0f - (beta*beta)/8.0f;

    q = P/3.0f;
    r = -Q_1/2.0f;
    delta = q*q*q+r*r;
    complex s,t,rho;
    float theta =0;
    float temp1, temp2;
#if 1
    if(delta >0)
    {
        temp1 = r + square_root(delta);
        temp2 = r - square_root(delta);
    	s.real = cube_root(temp1);
    	s.image = 0;
    	t.real = cube_root(temp2);
    	t.image = 0;
    }
    else
    {
        temp1 = r + square_root(-delta);
        temp2 = r - square_root(-delta);
    	s.real = cube_root(temp1);
    	s.image = 0;
    	t.real = cube_root(temp2);
    	t.image = 0;
    }
#endif

#if 0
    if(delta >0)
    {
        temp1 = r + square_root(delta);
        temp2 = r - square_root(delta);
    	s.real = 1.0f/(temp1) *1.0f/(temp1)*1.0/(temp1);
    	s.image = 0;
    	t.real = 1.0f/(temp2) *1.0f/(temp2)*1.0/(temp2);
    	t.image = 0;
    }
    else
    {
        temp1 = r + square_root(-delta);
        temp2 = r - square_root(-delta);
    	s.real = 1.0f/(temp1) *1.0f/(temp1)*1.0/(temp1);
    	s.image = 0;
    	t.real = 1.0f/(temp2) *1.0f/(temp2)*1.0/(temp2);
    	t.image = 0;
    }
#endif

    complex r1;
    float y = 0, W = 0;
    r1.real = s.real + t.real;
    r1.image = s.image + t.image;

//    r2.real = -0.5f*(s.real + t.real) + (-(square_root(3)/2.0)*(s.image-t.image));
//    r2.image = -0.5f*(s.image + t.image) + (square_root(3)/2.0)*(s.real-t.real);
//
//    r3.real = -0.5f*(s.real + t.real) - (-(square_root(3)/2.0)*(s.image-t.image));
//    r3.image = -0.5f*(s.image + t.image) - (square_root(3)/2.0)*(s.real-t.real);

    // workaround
    r = r1.real;
    y = (-5.0f/6.0)*alpha + r;
    W = square_root((alpha + 2*y));
    p_data->root[0] = 0;
    p_data->root[1] = 0;
    p_data->root[2] = 0;
    p_data->root[3] = 0;
    float temp3 = -(3*alpha + 2*y + 2*beta/W);
    float temp4 = -(3*alpha + 2*y - 2*beta/W);
    float temp5 = (-p_data->B)/(4*p_data->A);

    if(temp3 >0)
    {
    	p_data->root[0] = temp5 + (W + square_root(temp3))/2;
    	p_data->root[2] = temp5 + (W - square_root(temp3))/2;
    }

    if(temp4 >0)
    {
    	p_data->root[1]  = temp5 + (-W + square_root(temp4))/2;
    	p_data->root[3]  = temp5 + (-W - square_root(temp4))/2;
    }
}

static void compute_Qhat(soh_awtls* p_data)
{
	// discard negative roots
	int i =0;
	float Jr[4] ={10000,10000,10000,10000};
	float J_min=0, Q=0, H =0;
	for (i=0; i<4; i++)
	{
		if(p_data->root[i] > 0)
		{
			p_data->result[i] = p_data->root[i];

			Jr[i] =	(1/(p_data->result[i]*p_data->result[i]+1))*(p_data->result[i]*p_data->result[i]*p_data->result[i]*p_data->result[i]*p_data->C4
					-2*p_data->C5*p_data->result[i]*p_data->result[i]*p_data->result[i] + (p_data->C1+ p_data->C6)*p_data->result[i]*p_data->result[i]
					-2*p_data->C2*p_data->result[i] + p_data->C3);
		}
		else
		{
			p_data->result[i] = 0;
		}
	}
	// compute Jr
//    Jr = ((1./(r.^2+1).^2).*(r.^4*C4-2*C5*r.^3+(C1+C6)*r.^2-2*C2*r+C3))';
//    J = min(Jr);
//    Q = r(Jr==J); % keep Q that minimizes cost function
	J_min = Jr[0];
    Q=p_data->result[0];
	for(i =1; i <4; i++)
	{
		if(J_min > Jr[i])
		{
			J_min=Jr[i];
            Q=p_data->result[i];
		}
	}

	H = (2/((Q*Q+1)*(Q*Q+1)*(Q*Q+1)*(Q*Q+1)))*(-2*p_data->C5*Q*Q*Q*Q*Q+(3*p_data->C1-6*p_data->C4+3*p_data->C6)*Q*Q*Q*Q
			-(12*p_data->C2+15*p_data->C5)*Q*Q*Q + (-8*p_data->C1+10*p_data->C3+6*p_data->C4-8*p_data->C6)*Q*Q
			+(12*p_data->C2-6*p_data->C5)*Q + (p_data->C1 -2*p_data->C3 +p_data->C6));
	p_data->Q_hat = Q/K;
	p_data->Sigma_Q = 2/H/(K*K);

	p_data->soh_estimate = get_min(1.0f, get_max(0.0f, p_data->Q_hat/Q_NOMINAL_CAPACITY));
//    H = (2/(Q^2+1)^4)*(-2*C5*Q^5+(3*C1-6*C4+3*C6)*Q^4+(-12*C2+16*C5)*Q^3 ...
//          +(-8*C1+10*C3+6*C4-8*C6)*Q^2+(12*C2-6*C5)*Q+(C1-2*C3+C6));
//    Qhat(iter,1) = Q/K;
//    SigmaQ(iter,1) = 2/H/K^2;

}

