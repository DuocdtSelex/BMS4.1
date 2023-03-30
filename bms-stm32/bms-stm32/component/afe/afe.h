/*
 * afe.h
 *
 *  Created on: Aug 19, 2020
 *      Author: quangnd
 */

#ifndef COMPONENT_AFE_AFE_H_
#define COMPONENT_AFE_AFE_H_

#include "cell_array.h"

#define AFE_ERROR_NO							0L
#define AFE_ERROR_COM_LINK						-1L
#define AFE_ERROR_READ_MISMATCH						-2L

#define AFE_STS_HW_ERROR						(1<<16)
#define AFE_STS_OCD							(1<<0)
#define AFE_STS_SCD							(1<<1)
#define AFE_STS_OV 	    			        		(1<<2)
#define AFE_STS_UV			 	    			(1<<3)
#define AFE_STS_OT				      			(1<<9)
#define AFE_STS_UT				      			(1<<10)

#define AFE_MAX_GROUP							4

typedef struct AFE_t AFE;
typedef struct AFE_Interface_t AFE_Interface;
typedef struct Permanent_Fail_t Permanent_Fail;
typedef struct Safety_t	Safety;

struct AFE_Interface_t{
	void (*disable_cell_balancing)(AFE* this);
	void (*enable_cell_balancing)(AFE* this,const uint32_t target_vol);
	void (*set_occ_threshold_mA)(AFE* this, uint32_t th);
	void (*set_scd_threshold_mA)(AFE* this, uint32_t th);
	void (*set_cuv_threshold_mV)(AFE* this, uint32_t th);
	void (*set_cov_threshold_mV)(AFE* this, uint32_t th);
	void (*set_ocd1_threshold_mA)(AFE* this, uint32_t th);
	void (*set_ocd2_threshold_mA)(AFE* this, uint32_t th);
	void (*set_occ_delay_ms)(AFE* this, uint32_t ms);
	void (*set_scd_delay_us)(AFE* this, uint32_t us);
	void (*set_cuv_delay_ms)(AFE* this, uint32_t ms);
	void (*set_cov_delay_ms)(AFE* this, uint32_t ms);
	void (*set_ocd1_delay_ms)(AFE* this, uint32_t ms);
	void (*set_ocd2_delay_ms)(AFE* this, uint32_t ms);
	void (*set_cov_recovery_hysteresis)(AFE* this, uint32_t th);
	void (*set_cuv_recovery_hysteresis)(AFE* this, uint32_t th);
	void (*set_scd_recovery_time_s)(AFE* this, uint32_t s);
	void (*set_occ_recovery_threshold_mA)(AFE* this, int32_t th);
	void (*set_ocd_recovery_threshold_mA)(AFE* this, int32_t th);
	void (*update_cell_voltage)(AFE* this);
	void (*update_pack_voltage)(AFE* this);
	void (*update_status)(AFE* this);
	void (*update_battery_status)(AFE* this);
	void (*reset_error)(AFE* this);
	void (*set_shutdown_mode)(AFE* this);
	void (*update_current_cA)(AFE* this);
	void (*update_fet_status)(AFE* this);
	void (*update_safety_alert_status)(AFE* this);
	void (*afe_update_permanent_fail_state)(AFE* this);
	void (*afe_set_otd_threshold_C)(AFE* this, uint8_t th);
	void (*afe_set_otc_threshold_C)(AFE* this, uint8_t th);
	void (*afe_set_otint_threshold_C)(AFE* this, uint8_t th);
	void (*afe_set_otf_threshold_C)(AFE* this, uint8_t th);
	void (*afe_set_otc_recovery_C)(AFE* this, uint8_t th);
	void (*afe_set_utd_threshold_C)(AFE* this, uint8_t th);
	void (*afe_set_utc_threshold_C)(AFE* this, uint8_t th);
	void (*afe_set_utint_threshold_C)(AFE* this, uint8_t th);
	void (*afe_update_int_temperature_C)(AFE* this);
};

struct Permanent_Fail_t{
	uint8_t pf_alert_a;
	uint8_t pf_alert_b;
	uint8_t pf_alert_c;
	uint8_t pf_alert_d;
	uint8_t pf_status_a;
	uint8_t pf_status_b;
	uint8_t pf_status_c;
	uint8_t pf_status_d;
};

struct Safety_t{
	uint8_t safety_status_a;
	uint8_t safety_status_b;
	uint8_t safety_status_c;
	uint8_t safety_alert_a;
	uint8_t safety_alert_b;
	uint8_t safety_alert_c;
};
struct AFE_t{
	Cell_Array* cell_array;
	const AFE_Interface* interface;
	uint32_t pack_voltage;
	int32_t error;
	uint32_t status;
	uint32_t battery_status;
	int32_t current;
	uint8_t fet_status;
	uint8_t is_balancing;
	uint16_t balancing_mask;
	Safety safety;
	Permanent_Fail permanent_fail;
	int16_t int_temperature;
};

AFE* afe_create(void);

static inline int32_t afe_get_last_error(const AFE* const p_afe){
	return p_afe->error;
}

static inline void afe_set_error(AFE* const p_afe,const int32_t error){
	p_afe->error=error;
	p_afe->status|= AFE_STS_HW_ERROR;
}

void afe_set_interface(AFE* const p_afe,const AFE_Interface* const p_interface);

static inline void afe_update_pack_voltate(AFE* p_afe){
 	p_afe->interface->update_pack_voltage(p_afe);
}

static inline void afe_update_cell_voltage(AFE* p_afe){
	p_afe->interface->update_cell_voltage(p_afe);
}

static inline void afe_update_status(AFE* this){
	this->interface->update_status(this);
}

static inline void afe_update_battery_status(AFE* this){
	this->interface->update_battery_status(this);
}

static inline void afe_update_current_cA(AFE* this){
	this->interface->update_current_cA(this);
}

static inline void afe_update_fet_status(AFE* this){
	this->interface->update_fet_status(this);
}

static inline void 	afe_update_safety_alert_status(AFE* this){
	this->interface->update_safety_alert_status(this);
}

static inline void afe_update_permanent_fail_state(AFE* this){
	this->interface->afe_update_permanent_fail_state(this);
}

static inline void afe_set_otd_threshold_C(AFE* this, uint8_t th){
	this->interface->afe_set_otd_threshold_C(this, th);
}

static inline void afe_set_otc_threshold_C(AFE* this, uint8_t th){
	this->interface->afe_set_otc_threshold_C(this, th);
}

static inline void afe_set_otint_threshold_C(AFE* this, uint8_t th){
	this->interface->afe_set_otint_threshold_C(this, th);
}

static inline void afe_set_otf_threshold_C(AFE* this, uint8_t th){
	this->interface->afe_set_otf_threshold_C(this, th);
}

static inline void afe_set_otc_recovery_C(AFE* this, uint8_t th){
	this->interface->afe_set_otc_recovery_C(this, th);
}
static inline void afe_set_utd_threshold_C(AFE* this, uint8_t th){
	this->interface->afe_set_utd_threshold_C(this, th);
}

static inline void afe_set_utc_threshold_C(AFE* this, uint8_t th){
	this->interface->afe_set_utc_threshold_C(this, th);
}

static inline void afe_set_utint_threshold_C(AFE* this, uint8_t th){
	this->interface->afe_set_utint_threshold_C(this, th);
}

static inline void afe_update_int_temperature_C(AFE* this){
	this->interface->afe_update_int_temperature_C(this);
}

static inline void afe_set_occ_threshold_mA(AFE* this, uint32_t th){
	this->interface->set_occ_threshold_mA(this,th);
};

static inline void afe_set_scd_threshold_mA(AFE* this, uint32_t th){
	this->interface->set_scd_threshold_mA(this,th);
};

static inline void afe_set_cuv_threshold_mV(AFE* this, uint32_t th){
	this->interface->set_cuv_threshold_mV(this,th);
};

static inline void afe_set_cov_threshold_mV(AFE* this, uint32_t th){
	this->interface->set_cov_threshold_mV(this,th);
};

static inline void afe_set_ocd1_threshold_mA(AFE* this, uint32_t th){
	this->interface->set_ocd1_threshold_mA(this,th);
}

static inline void afe_set_ocd2_threshold_mA(AFE* this, uint32_t th){
	this->interface->set_ocd2_threshold_mA(this,th);
}

static inline void afe_set_occ_delay_ms(AFE* this, uint32_t ms){
	this->interface->set_occ_delay_ms(this,ms);
};

static inline void afe_set_scd_delay_us(AFE* this, uint32_t us){
	this->interface->set_scd_delay_us(this,us);
};

static inline void afe_set_cuv_delay_ms(AFE* this, uint32_t ms){
	this->interface->set_cuv_delay_ms(this,ms);
};

static inline void afe_set_cov_delay_ms(AFE* this, uint32_t ms){
	this->interface->set_cov_delay_ms(this,ms);
};

static inline void afe_set_ocd1_delay_ms(AFE*this, uint32_t ms){
	this->interface->set_ocd1_delay_ms(this,ms);
}

static inline void afe_set_ocd2_delay_ms(AFE*this, uint32_t ms){
	this->interface->set_ocd2_delay_ms(this,ms);
}

static inline void afe_set_cov_recovery_hysteresis(AFE* this, uint32_t th){
	this->interface->set_cov_recovery_hysteresis(this,th);
}

static inline void afe_set_cuv_recovery_hysteresis(AFE* this, uint32_t th){
	this->interface->set_cuv_recovery_hysteresis(this,th);
}

static inline void afe_set_scd_recovery_time_s(AFE* this, uint32_t th){
	this->interface->set_scd_recovery_time_s(this, th);
}

static inline void afe_set_occ_recovery_threshold_mA(AFE* this, int32_t th){
	this->interface->set_occ_recovery_threshold_mA(this, th);
}

static inline void afe_set_ocd_recovery_threshold_mA(AFE* this, int32_t th){
	this->interface->set_ocd_recovery_threshold_mA(this, th);
}
static inline void afe_disable_cell_balancing(AFE* this){
	this->interface->disable_cell_balancing(this);
}

static inline void afe_enable_cell_balancing(AFE* this,const uint32_t target_vol){
	this->interface->enable_cell_balancing(this,target_vol);
}
static inline void afe_set_cell_array(AFE* const p_afe,Cell_Array* const p_ca){
	p_afe->cell_array=p_ca;
}

static inline Cell_Array* afe_get_cell_array(const AFE* const p_afe){
	return p_afe->cell_array;
}

static inline uint32_t afe_get_pack_voltage(const AFE* const p_afe){

	return p_afe->pack_voltage;
}

static inline uint32_t afe_get_cell_voltage(const AFE* const p_afe,const uint8_t cell_id){

	return p_afe->cell_array->cells[cell_id].voltage;
}

static inline uint32_t afe_get_status(const AFE* const p_afe){
	return p_afe->status;
}

static inline uint32_t afe_get_battery_status(const AFE* const p_afe){
	return p_afe->battery_status;
}

static inline int32_t afe_get_current(const AFE* const p_afe){
	return p_afe->current;
}

static inline int32_t afe_get_int_temperature(const AFE* const p_afe){
	return p_afe->int_temperature;
}

static inline uint32_t afe_get_fet_status(const AFE* const p_afe){
	return p_afe->fet_status;
}
static inline void afe_reset_error(AFE* this){
	this->interface->reset_error(this);
}

static inline void afe_set_shutdown_mode(AFE* this){
	this->interface->set_shutdown_mode(this);
}
#endif /* COMPONENT_AFE_AFE_H_ */
