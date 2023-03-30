#include "string_util.h"
static const uint8_t digit_char[10]={'0','1','2','3','4','5','6','7','8','9'};
static const uint8_t hex_digit[] ={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

void string_swap(const uint8_t* p_buffer,uint8_t* p_msg,const uint8_t len){
	uint8_t i=0;
	for(i=0;i<len;i++){
		p_msg[i]=p_buffer[len-i-1];
	}
}

int long_to_string(const uint32_t data,uint8_t* s){
	uint32_t remain=data;
	uint8_t digit=0;
	uint8_t index=0;
        uint8_t buffer[16]= {0};
	while(remain >=10){
		digit = remain%10;
		buffer[index]=digit_char[digit];
		index++;
		remain =(uint32_t)(remain/10);
	}
	buffer[index]=digit_char[remain];
        string_swap(buffer,s,index+1);
	s[index+1]='\0';
	return index+1;
}

int slong_to_string(const int32_t data,uint8_t* s){
	int32_t temp;
	if(data >=0){
		return long_to_string(data,s);
	}

	temp=-data;
	*s='-';
	return (long_to_string(temp,s+1)+1);
}

void byte_to_hex_ascii(const uint8_t s,uint8_t* hex){
    uint8_t low_digit= s & 0x0f;
    uint8_t high_digit=(s & 0xf0)>>4;
    hex[0] = hex_digit[low_digit];
    hex[1]= hex_digit[high_digit];
}

