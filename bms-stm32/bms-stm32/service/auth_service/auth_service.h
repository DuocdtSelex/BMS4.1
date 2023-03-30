/*
 * auth_service.h
 *
 *  Created on: Nov 3, 2020
 *      Author: quangnd
 */

#ifndef SERVICE_AUTH_SERVICE_AUTH_SERVICE_H_
#define SERVICE_AUTH_SERVICE_AUTH_SERVICE_H_

#include "stdint.h"

typedef enum AUTH_SV_STATE_t{
        AUTH_ST_NOT_REG=0,
        AUTH_ST_PROCESS,
        AUTH_ST_PASS,
        AUTH_ST_FAIL
}AUTH_SV_STATE;

typedef struct Auth_SV_t Auth_SV;

struct Auth_SV_t{

        AUTH_SV_STATE state;
        void (*authenticate)(Auth_SV* p_sv,void* data,const uint32_t time);
        uint32_t auth_time_counter;
        uint32_t auth_timeout_ms;
        uint32_t session_time;
        uint32_t session_timeout;
};

void auth_sv_init(Auth_SV* p_sv);

#endif /* SERVICE_AUTH_SERVICE_AUTH_SERVICE_H_ */
