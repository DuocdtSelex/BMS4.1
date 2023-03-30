#include "debug_com_port_hal.h"
#include "delay_hw.h"
#include "core_hal.h"
#include "can_hal.h"

static CanTxMsg tx_msg;
static void can_rx_impl(CAN_Hw* p_hw);

//static void test_fail(void){
//		debug_sends(&debug_port,(uint8_t*)"FAIL\n");
//}
//
//static void test_pass(void){
//		debug_sends(&debug_port,(uint8_t*)"PASS\n");
//}
//
//static void test_equal(int32_t src,int32_t dest){
//	if(src !=dest){
//			test_fail();
//	}else{
//			test_pass();
//	}
//}

static void test_setup(void){
	core_hw_init();
	//debug_com_hw_init();
	can_hardware_init();
	tx_msg.DLC=2;
	tx_msg.Data[0]='A';
	tx_msg.Data[1]='B';
	tx_msg.StdId=0x33;
	can_set_receive_handle(&can1,can_rx_impl);
}

int main(void)
{
	hw_delay_ms(500);
	test_setup();
	while(1){
		can_send(&can1,&tx_msg);
		hw_delay_ms(1000);
	}
	return 0;
}

static void can_rx_impl(CAN_Hw* p_hw){
	int a =0;
	if(p_hw->rx_msg.StdId ==1){
		a=1;
	}
//
//	debug_sends(&debug_port,p_hw->rx_msg.Data);
}
void SysTick_Handler(void){

}
