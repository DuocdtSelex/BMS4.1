/*
 * ntc.c
 *
 *  Created on: Sep 30, 2020
 *      Author: quangnd
 */

#include "ntc.h"

static int binary_search(const uint16_t data, const uint16_t* array, const uint16_t max_size);

static int16_t ntc_get_temp_from_impedance(const uint16_t impedance,const uint16_t* const p_lut,
		const uint16_t size,const int16_t min){

	int index = 0;
    index = binary_search(impedance, p_lut, size);
	return index+min;
#if 0
	int16_t id=0;
    while((impedance < p_lut[id]) &&(id<size)){
   	 id++;
    }
	return(id+min);
#endif
}

void ntc_update_temp(NTC* p_ntc){

	ntc_update_impedance(p_ntc);

	p_ntc->temp=ntc_get_temp_from_impedance(p_ntc->impedance,p_ntc->lut,p_ntc->lut_size,
			p_ntc->min_temp)/10;
}

void ntc_update_temp_from_bq(NTC* p_ntc){
	ntc_update_temperature(p_ntc);
}

static int binary_search(const uint16_t data, const uint16_t* array, const uint16_t max_size){
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
      }
      else
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
    if (lower_bound > upper_bound){
    	index = lower_bound;
    }
   return index;
}
