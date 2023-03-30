/*
 * auth_service.c
 *
 *  Created on: Nov 3, 2020
 *      Author: quangnd
 */

#include "auth_service.h"

void auth_sv_init(Auth_SV* p_sv){

        p_sv->auth_time_counter=0;
        p_sv->auth_timeout_ms=3000;
        p_sv->state=AUTH_ST_NOT_REG;
        p_sv->session_time=0;
        p_sv->session_timeout=60000;
}
