/*
 * estimator.h
 *
 *  Created on: Aug 24, 2020
 *      Author: quangnd
 */

#ifndef SERVICE_ESTIMATOR_ESTIMATOR_H_
#define SERVICE_ESTIMATOR_ESTIMATOR_H_

typedef struct Estimator_t Estimator;

struct Estimator_t{
	void* input;
	void* output;
	void (*init)(Estimator* p_est);
	void (*update)(Estimator* p_est);
};

static inline void* est_get_ouput(const Estimator* const p_est){
	return p_est->output;
}

static inline void est_update(Estimator* p_est){
	p_est->update(p_est);
}

static inline void est_init(Estimator* p_est){
	p_est->init(p_est);
}

static inline void* est_get_input_buffer(const Estimator* const p_est){
	return p_est->input;
}

#endif /* SERVICE_ESTIMATOR_ESTIMATOR_H_ */
