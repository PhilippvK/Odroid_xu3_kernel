#ifndef _DOMINIKS_GOV_GENEREL_SCHED_H_
#define _DOMINIKS_GOV_GENEREL_SCHED_H_

#define SIZE_WORKLOAD_HISTORY 21
//#define SIZE_WORKLOAD_HISTORY 11

typedef struct {
	pid_t pid;
	int64_t workload_history[SIZE_WORKLOAD_HISTORY];
	int64_t autocorr_max;
	int autocorr_shift;
	int64_t prediction;
	int64_t prediction_cycles;
	int64_t cpu_time;
	short allocated_core;
    struct mutex lock;
}task_struct_expansion;

#endif
