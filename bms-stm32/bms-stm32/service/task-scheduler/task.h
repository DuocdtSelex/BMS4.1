#ifndef TASK_H_
#define TASK_H_
#include "board.h"

typedef struct Task Task;
typedef enum TASK_RUN_POLICY TASK_RUN_POLICY;
typedef enum TASK_STATE TASK_STATE;
typedef void (*Task_Run_Handle)();

void task_init(Task* p_task,
		const uint32_t next_run,
		const uint32_t run_interval,
		const uint32_t time_out,
		TASK_RUN_POLICY run_policy,
		TASK_STATE state,
	       	Task_Run_Handle handle);
void task_set_state(Task* p_task,const TASK_STATE state);

enum TASK_RUN_POLICY{
	TASK_ONESHOT	=0x00,
	TASK_PERIODIC	=0x01
};

enum TASK_STATE{
	TASK_RUNABLE		=0x00,
	TASK_SUSPENDING		=0x01,
	TASK_STOPPED		=0x02	
};

struct Task{
	uint32_t next_run_timestamp;
	uint32_t run_interval;
	uint16_t time_out;
	TASK_STATE state;
	TASK_RUN_POLICY run_policy;
	uint8_t id;
	Task_Run_Handle run;
};

#endif
