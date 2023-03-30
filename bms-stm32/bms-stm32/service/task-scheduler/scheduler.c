#include "scheduler.h"
#include "task.h"
#include "stdio.h"
#include "stdlib.h"
#include "../../board/stm32_bsp/stm32f0xx_it.h"

//extern volatile uint32_t system_time_stamp;
void scheduler_init(Scheduler *p_sch, uint32_t *p_time_stamp) {
	p_sch->task_counter = 0;
	p_sch->time_stamp = p_time_stamp;
}
void scheduler_start(Scheduler *p_sch) {
	scheduler_reset_time(p_sch);
}

void scheduler_reset_time(Scheduler *p_sch) {
	*(p_sch->time_stamp) = 0;
}
void scheduler_stop(Scheduler *p_sch) {

}

void scheduler_add_task(Scheduler *p_sch, Task *p_task) {
	if (p_sch->task_counter >= MAX_SYSTEM_TASKS)
		return;

	p_sch->system_tasks[p_sch->task_counter] = p_task;
	p_task->id = p_sch->task_counter;
	p_sch->task_counter++;
}

void scheduler_dispatch_task(Scheduler *p_sch) {
	uint8_t task_index;
	Task *p_task = NULL;
	for (task_index = 0; task_index < p_sch->task_counter; task_index++) {
		p_task = p_sch->system_tasks[task_index];
		if (p_task->state != TASK_RUNABLE) {
			continue;
		}
		if (p_task->next_run_timestamp <= *(p_sch->time_stamp)) {
			p_task->run();
			if (p_task->run_policy == TASK_ONESHOT) {
				p_task->state = TASK_STOPPED;
			} else {
				p_task->next_run_timestamp += p_task->run_interval;
			}
		}
	}
}
