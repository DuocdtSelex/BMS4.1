/*
 * afe.c
 *
 *  Created on: Aug 19, 2020
 *      Author: quangnd
 */

#include "afe.h"
#include "stdlib.h"
#include "stdio.h"
void afe_set_interface(AFE* const p_afe,const AFE_Interface* const p_interface){
	p_afe->interface=p_interface;
}

AFE* afe_create(void){

	AFE* this=(AFE*)malloc(sizeof(AFE));
	while(this==NULL){

	};
	return this;
}
