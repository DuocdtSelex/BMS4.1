#if 0
#include "soc-ocv.h"
#include "stdint.h"

 void soc_ocv_interpoltae(SOC_OCV *soc_ocv) {
	 uint8_t vol_id_arr = 0;
	 		while (soc_ocv->vol <= ocv_values[vol_id_arr]) {
	 			vol_id_arr++;
	 		}
	 		if (vol_id_arr > (SOC_LOOKUPS_TABLE_SIZE - 1))
	 			vol_id_arr = (SOC_LOOKUPS_TABLE_SIZE - 1);
	 		soc_ocv->soc = (soc_values[vol_id_arr - 1] * (soc_ocv->vol - ocv_values[vol_id_arr])
	 				+ soc_values[vol_id_arr] * (ocv_values[vol_id_arr - 1] - soc_ocv->vol))
	 				/ ( ocv_values[vol_id_arr - 1] -ocv_values[vol_id_arr] );
 }
//////////////////// test NTL ... from state of charge value to calculate voltage of cell
 void ocv_soc_interpoltae(SOC_OCV *soc_ocv){
	 uint8_t soc_id_arr = 0;
	 while(soc_ocv->soc <= soc_values[soc_id_arr]){
		 soc_id_arr++;
	 }
	// soc_id_arr--;
	 soc_ocv->vol = (ocv_values[soc_id_arr-1]*(soc_ocv->soc - soc_values[soc_id_arr])
			 + ocv_values[soc_id_arr]* (soc_values[soc_id_arr-1]- soc_ocv->soc))
			 /(soc_values[soc_id_arr-1]-soc_values[soc_id_arr]);
 }
 /////////////////////////
int32_t get_soc_from_ocv(uint32_t voltage, int32_t current){
	 	int32_t id=0;
	 	uint32_t OCV;
	 	uint32_t abs_current;
	 	abs_current = (current <0) ? (-current) : (current);
	 	OCV = voltage + (abs_current * INTERNAL_RESISTANCE_mOhm)/1000;
       // ensures that only OCVs within the bounds of the polynomial fit are used
	 	if(OCV > ocv_values_pack[0]){
	 		OCV = ocv_values_pack[0];
	 	}
	 	if(OCV < ocv_values_pack[100]){
	 		OCV = ocv_values_pack[100];
	 	}
	     while((OCV <= ocv_values_pack[id]) &&(id<OCV_LUT_SIZE)){
	    	 id++;
	     }
	 	return soc_values_pack[id];
	 }
#endif
