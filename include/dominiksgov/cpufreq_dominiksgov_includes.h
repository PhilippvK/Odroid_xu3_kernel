#ifndef _DOMINIKS_GOV_GENEREL_H_
#define _DOMINIKS_GOV_GENEREL_H_

#include "dominiksgov/unit_tests.h"
#include <linux/types.h>
#include <dominiksgov/cpufreq_dominiksgov_sched.h>
#include <dominiksgov/cpufreq_dominiksgov_eglapi.h>

#ifdef UNIT_TEST
	#include <sys/types.h>
	#include <stdint.h>
	int64_t div64_s64(int64_t a, int64_t b);
#endif

/*
 * defines for configuration of governor
 */

//define to enable logging
#define DO_LOGGING
#ifdef DO_LOGGING
	#define LOGFILE "/data/local/GameOptimized_Gov_LOG"
	#pragma message( "Compiled with logging flag!")
#endif

//define to enable powergating
#define POWERGATING_GAMEOPTIMIZED_GOV

//define for debugging purposes
//#define GOV_PRINT_SPACES

//define to enable thread name logging
//#define THREAD_NAME_LOGGING
#ifdef THREAD_NAME_LOGGING
	#define LOGFILE_THREAD_NAME "/data/local/GameOptimized_Gov_Thread_Name_LOG"
	#pragma message("Compiled with thread name logging flag!")
#endif

#define CPUCYCLE_COVERSION_FACTOR 201

#define AGGRESSIVENESS_FACTOR_UP 20
#define AGGRESSIVENESS_FACTOR_DOWN 10
#define FRAME_HIT_REDUCTION_FACTOR 5

#define FRAME_RATE_UPPER_BOUND 33
#define FRAME_RATE_LOWER_BOUND 27

#define SHUTDOWN_LIMIT 10

#define REPORT_DEBUG    0
#define REPORT_ERROR    1
#define REPORT_VERBOSE  0
#define REPORT_WARNING 	1

#define FIRST_MINOR 0
#define MINOR_CNT 1
/*
 * end config
 */

extern void autocorr( const int64_t *data_in, int64_t *outcorr_out, int tau, int length );
int64_t corr_own(const int64_t *data_in, int a, int length);
int64_t mean(const int64_t *data_in, int length);
int64_t WMA( const int64_t *cpu_time_history, int length );
void get_max(const int64_t *a, int length, int64_t *max, int *pos);
int64_t WMA_hybrid_predictor(const int64_t *data, int length, int64_t autocorr_max, int pos);




#define KERNEL_DEBUG_MSG(...) \
            do { if (REPORT_DEBUG) printk(KERN_INFO __VA_ARGS__); } while (0)

#define KERNEL_ERROR_MSG(...) \
            do { if (REPORT_ERROR) printk(KERN_ERR __VA_ARGS__); } while (0)

#define KERNEL_LOGGG_MSG(...) \
            do { if (REPORT_LOGGG) printk(KERN_ERR __VA_ARGS__); } while (0)

#define KERNEL_VERBOSE_MSG(...) \
            do { if (REPORT_VERBOSE) printk(KERN_INFO __VA_ARGS__); } while (0)

#define KERNEL_WARNING_MSG(...) \
            do { if (REPORT_WARNING) printk(KERN_WARNING __VA_ARGS__); } while (0)

#define DOMINIKSGOV_CHAR_DEVICE_NAME "dominiksgov_device"
#define IOCTL_MAJOR 240

//static char *Version ="V0.1";

typedef struct {
	uint64_t time; //holds the time of the last update
	const uint64_t update_interval; //in ns
	short timer_expired; //var to check if timer has expired (1==yes)
}timer_struct;

typedef struct{
	int64_t nr;
	int64_t a7_freq;
	int64_t a15_freq;
	uint64_t frame_rate;
	uint64_t time_in;
	int64_t fr_error;
	short cpu4_online;
	short cpu5_online;
	short cpu6_online;
	short cpu7_online;
	int number_allocation_chances;
	int nr_tasks_on_cpu[8];
}log_struct;

#endif
