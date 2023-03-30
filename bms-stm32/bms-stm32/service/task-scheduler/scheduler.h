#ifndef SCHEDULER_H_
#define SCHEDULER_H_
#include "task.h"
#define MAX_SYSTEM_TASKS	6

typedef struct Scheduler Scheduler;

void scheduler_init(Scheduler* p_sch,uint32_t* p_time_stamp);
void scheduler_start(Scheduler* p_sch);
void scheduler_stop(Scheduler* p_sch);
void scheduler_add_task(Scheduler* p_sch,Task* p_task);
void scheduler_dispatch_task(Scheduler* p_sch);
void scheduler_reset_time(Scheduler* p_sch);

struct Scheduler{
	uint8_t task_counter;
	Task* system_tasks[MAX_SYSTEM_TASKS];
    uint32_t* time_stamp;
};

#endif
