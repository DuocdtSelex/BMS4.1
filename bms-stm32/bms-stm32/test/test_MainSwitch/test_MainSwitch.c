#include "debug_com_port_hal.h"
#include "delay_hw.h"
#include "core_hal.h"
#include "bq_hal.h"

static void test_fail(void){
		debug_sends(&debug_port,(uint8_t*)"FAIL\n");
}

static void test_pass(void){
		debug_sends(&debug_port,(uint8_t*)"PASS\n");
}

static void test_equal(int32_t src,int32_t dest){
	if(src !=dest){
			test_fail();
	}else{
			test_pass();
	}
}

static uint8_t test_states[]={0x00,0x01,0x02,0x03};
//static uint8_t test_states[]={0x00,0x00,0x00,0x00};
static uint8_t step=0;

static void test_setup(void){
	core_hw_init();
	debug_com_hw_init();
	afe_hardware_init();
}

static void test_switch_state(BQ_Hw* p_hw,uint8_t sw_state){

	int32_t state=0;
	debug_sends(&debug_port,(uint8_t*)"TEST SET STATE: ");
	state =bq_write_reg_byte_with_crc(p_hw,0x05,sw_state);
	test_equal(state,0);
	debug_sends(&debug_port,(uint8_t*)"\n");
}

int main(void)
{
	hw_delay_ms(500);
	test_setup();
	while(1){
		test_switch_state(&upper_bq_hw,test_states[step]);
		test_switch_state(&lower_bq_hw,test_states[step]);
		step++;
		if(step>3){
			step=0;
		}
		hw_delay_ms(2000);
	}
	return 0;
}

void SysTick_Handler(void){

}

void EXTI4_15_IRQHandler(void){
	BQ_INT_CLEAR_FLAG;
}

void TIM7_IRQHandler(void) {
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET) {
	}
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
 }

