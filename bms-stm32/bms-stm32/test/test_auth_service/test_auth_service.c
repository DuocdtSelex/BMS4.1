/*
 * test_auth_service.c
 *
 *  Created on: Nov 3, 2020
 *      Author: quangnd
 */

#include "auth_sv_init.h"
#include "auth_service.h"

Cmd_Buffer test_buff;

static void buff_receive(Cmd_Buffer* p_buff,const char c){
        p_buff->data[p_buff->index++]=c;
        p_buff->data[p_buff->index]='\0';
        if(p_buff->index>=AUTH_CMD_BUFFER_SIZE){
                p_buff->index=0;
        }
}

static void buff_receives(Cmd_Buffer* p_buff,const char* s){
        while(*s){
                buff_receive(p_buff,*s);
                s++;
        }
}

static void test_log_in_success(void){
        buff_receives(&test_buff,"I\r\n");
        bms_auth_sv.authenticate(&bms_auth_sv,(void*)&test_buff,0);
        buff_receives(&test_buff,"RSelex123\r\n");
        bms_auth_sv.authenticate(&bms_auth_sv,(void*)&test_buff,0);
}

static void test_wrong_password(void){

        buff_receives(&test_buff,"I\r\n");
        bms_auth_sv.authenticate(&bms_auth_sv,(void*)&test_buff,0);
        buff_receives(&test_buff,"RSelex124\r\n");
        bms_auth_sv.authenticate(&bms_auth_sv,(void*)&test_buff,0);
}

static void test_timeout(void){

        uint32_t timestamp=0;
        buff_receives(&test_buff,"I\r\n");
        bms_auth_sv.authenticate(&bms_auth_sv,(void*)&test_buff,timestamp);
        buff_receives(&test_buff,"RS");
        for(int i=0;i<4;i++){
                timestamp+=1000;
                bms_auth_sv.authenticate(&bms_auth_sv,(void*)&test_buff,timestamp);
        }

        buff_receives(&test_buff,"RSelex123\r\n");
        bms_auth_sv.authenticate(&bms_auth_sv,(void*)&test_buff,0);
}

static void test_log_out(void){
        test_log_in_success();
        buff_receives(&test_buff,"O\r\n");
        bms_auth_sv.authenticate(&bms_auth_sv,(void*)&test_buff,0);
}

int main(void){

        app_auth_service_init();
        test_log_in_success();
        test_log_out();
        test_timeout();
}




