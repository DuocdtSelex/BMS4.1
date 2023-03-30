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

static void test_setup(void){
	core_hw_init();
	debug_com_hw_init();
	afe_hardware_init();
}

static void test_bq_i2c_link(BQ_Hw* p_hw){
		int32_t state=0;
		uint8_t reg_write_value=0x55;
		uint8_t reg_data=0;

		debug_sends(&debug_port,(uint8_t*)"--TEST WRITE: ");
		state=bq_write_reg_byte_with_crc(p_hw,0x0A,reg_write_value);
		test_equal(state,0);

		debug_sends(&debug_port,(uint8_t*)"--TEST READ: ");
		state=bq_read_reg_byte_with_crc(p_hw,0x0A,&reg_data);

		test_equal(state,0);

		debug_sends(&debug_port,(uint8_t*)"--TEST VERIFY: ");
		test_equal(reg_data,reg_write_value);

		debug_sends(&debug_port,(uint8_t*)"\n");
}

int main(void)
{
	hw_delay_ms(500);
	test_setup();
	while(1){

		debug_sends(&debug_port,(uint8_t*)"TEST LOWER I2C:\n");
		test_bq_i2c_link(&lower_bq_hw);

		debug_sends(&debug_port,(uint8_t*)"TEST UPPER I2C:\n");
		test_bq_i2c_link(&upper_bq_hw);

		hw_delay_ms(1000);
	}
	return 0;
}

void SysTick_Handler(void){

}

void TIM7_IRQHandler(void) {
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET) {
	}
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
 }

