#include "stdint.h"
void string_swap(const uint8_t* p_buffer,uint8_t* p_msg,const uint8_t len);
int long_to_string(const uint32_t data,uint8_t* s);
int slong_to_string(const int32_t data,uint8_t* s);
void byte_to_hex_ascii(const uint8_t s,uint8_t* hex);
