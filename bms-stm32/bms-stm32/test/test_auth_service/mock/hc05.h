/*
 * hc05_mock.h
 *
 *  Created on: Nov 3, 2020
 *      Author: quangnd
 */

#ifndef TEST_TEST_AUTH_SERVICE_MOCK_HC05_H_
#define TEST_TEST_AUTH_SERVICE_MOCK_HC05_H_

typedef struct HC05_HW_t HC05_HW;
struct HC05_HW_t{

};

typedef struct Hc05_t Hc05;

struct Hc05_t{
};

typedef void (*Hc05_Receive_Handle)(const char c);
static HC05_HW hc05_port;
void hc05_sends(Hc05* p_hc,const char* s);
void hc05_init(Hc05* p_hc,HC05_HW* p_port);
void hc05_set_receive_handle(Hc05_Receive_Handle handle);

#endif /* TEST_TEST_AUTH_SERVICE_MOCK_HC05_H_ */
