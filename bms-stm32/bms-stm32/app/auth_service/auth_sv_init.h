/*
 * auth_sv_init.h
 *
 *  Created on: Nov 3, 2020
 *      Author: quangnd
 */

#ifndef APP_AUTH_SERVICE_AUTH_SV_INIT_H_
#define APP_AUTH_SERVICE_AUTH_SV_INIT_H_

#include "hc05.h"
#include "auth_service.h"

#define AUTH_CMD_BUFFER_SIZE    16
#define AUTH_SV_TIMEOUT_mS                      3000

typedef struct Cmd_Buffer_t Cmd_Buffer;
struct Cmd_Buffer_t{
        char data[AUTH_CMD_BUFFER_SIZE];
        uint16_t index;
};


static inline void buffer_reset(Cmd_Buffer* p_buff){
        p_buff->index=0;
}

extern Auth_SV bms_auth_sv;
void app_auth_service_init(void);


#endif /* APP_AUTH_SERVICE_AUTH_SV_INIT_H_ */
