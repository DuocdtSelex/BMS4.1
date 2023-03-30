/*
 * auth_sv_init.c
 *
 *  Created on: Nov 3, 2020
 *      Author: quangnd
 */


#include "auth_sv_init.h"
#include "auth_service.h"
#include "hc05.h"
#include "stdint.h"

static Cmd_Buffer cmd_rx_buffer;
Auth_SV bms_auth_sv;
static Hc05 bms_ble;
static void auth_sv_check_impl(Auth_SV* p_sv,void* data,const uint32_t time);
static void auth_sv_receive_data(const char c);

const char* key_white_list[]={
                "Selex123",
                "admin",
                "root"
};

void app_auth_service_init(void){

        cmd_rx_buffer.index=0;
        auth_sv_init(&bms_auth_sv);
        bms_auth_sv.authenticate=auth_sv_check_impl;
        hc05_init(&bms_ble,&hc05_port);
        hc05_set_receive_handle(auth_sv_receive_data);
}

static void process_authenticate_command(Auth_SV* p_sv,Cmd_Buffer* p_buff);

static void auth_sv_check_impl(Auth_SV* p_sv,void* data,const uint32_t time){

        if(p_sv->state==AUTH_ST_PASS){
                p_sv->session_time+=time;
                if(p_sv->session_time>=p_sv->session_timeout){
                        p_sv->state=AUTH_ST_NOT_REG;
                        return;
                }
        }

        if(p_sv->state==AUTH_ST_PROCESS){
                p_sv->auth_time_counter+=time;
                if(p_sv->auth_time_counter>=p_sv->auth_timeout_ms){
                        /* avoid overflow */
                        p_sv->auth_time_counter=p_sv->auth_timeout_ms;
                        p_sv->state=AUTH_ST_FAIL;
                        return;
                }
        }

        Cmd_Buffer* p_buff=(Cmd_Buffer*) data;

        /* only part of command received */
        if(p_buff->index<=2) return;

        if((p_buff->data[p_buff->index-2]==0x0D) &&
                        (p_buff->data[p_buff->index-1]==0x0A)){

                process_authenticate_command(p_sv,p_buff);
        }
}

static void process_authenticate_command(Auth_SV* p_sv,Cmd_Buffer* p_buff){

        char cmd=p_buff->data[0];
        char password[32];
        char* pass_byte=&p_buff->data[1];
        char* content=&password[0];
        int i = 0;
        const char *key = key_white_list[0];
        switch (cmd) {
        case 'I':
                p_sv->auth_time_counter = 0;
                p_sv->auth_timeout_ms = AUTH_SV_TIMEOUT_mS;
                p_sv->state = AUTH_ST_PROCESS;
                hc05_sends(&bms_ble, "OK\r\n");
                break;
        case 'R':
                if (p_sv->state != AUTH_ST_PROCESS) {
                        hc05_sends(&bms_ble, "ERR\r\n");
                        break;
                }
                while (*pass_byte != '\r') {
                        password[i] = *pass_byte;
                        pass_byte++;
                        i++;
                }

                while (*content == *key) {
                        content++;
                        key++;
                        if ((*key == '\0') || (*content == '\0'))
                                break;
                }
                ;
                if ((*content == '\0') && (*key == '\0')) {
                        p_sv->session_time = 0;
                        p_sv->state = AUTH_ST_PASS;
                        hc05_sends(&bms_ble, "OK\r\n");
                } else {
                        p_sv->state = AUTH_ST_FAIL;
                        hc05_sends(&bms_ble, "ERR\r\n");
                }
                break;
        case 'O':
                p_sv->state = AUTH_ST_NOT_REG;
                hc05_sends(&bms_ble,"OK\r\n");
                break;
        default:
                break;
        }

                buffer_reset(p_buff);
}

static void auth_sv_receive_data(const char c){
        cmd_rx_buffer.data[cmd_rx_buffer.index++]=c;
        cmd_rx_buffer.data[cmd_rx_buffer.index]='\0';
        if(cmd_rx_buffer.index>=AUTH_CMD_BUFFER_SIZE){
                cmd_rx_buffer.index=0;
        }
}

