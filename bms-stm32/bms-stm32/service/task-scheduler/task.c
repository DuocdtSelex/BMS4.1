#include "task.h"
void task_init(Task* p_task,
		const uint32_t next_run,
		const uint32_t run_interval,
		const uint32_t time_out,
		TASK_RUN_POLICY run_policy,
		TASK_STATE state,
	       	Task_Run_Handle handle){

	p_task->next_run_timestamp = next_run;
	p_task->run_interval= run_interval;
	p_task->time_out=time_out;
	p_task->run_policy = run_policy;
	p_task->state=TASK_RUNABLE;
	p_task->run = handle;
}

void task_set_state(Task* p_task,const TASK_STATE state){
	p_task->state=state;
}
