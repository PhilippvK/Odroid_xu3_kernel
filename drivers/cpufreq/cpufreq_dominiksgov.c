/*
 *  linux/drivers/cpufreq/cpufreq_powersave.c
 *
 *  Copyright (C) 2015 - 2016 Dominik Füß <dominik.fuess@tum.de>
 *  Copyright (C) 2019 Philipp van Kempen <philipp.van-kempen@tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#define DO_DEBUG

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/kthread.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/init.h>

#include <linux/fs.h>
#include <linux/version.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/stat.h>
#include <linux/fcntl.h>
#include <linux/unistd.h>

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/syscalls.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/pid.h>
#include <linux/random.h>
#include <linux/slab.h>

#include <dominiksgov/cpufreq_dominiksgov_includes.h>
#include <dominiksgov/cpufreq_dominiksgov.h>


//global vars for character file
static dev_t dev;
static struct cdev c_dev;
static struct class *cl;

//number of cpus managed by the governor
static short number_cpus=0;
static int a7ismanaged=0;
static int a15ismanaged=0;

int64_t nr_write_threads2=0;
#ifdef DO_DEBUG
//count function calls and number threads ->needed for debugging
static int thread_count=0;
static int number_allocation_chances=0;
#endif

//mutex definition
static DEFINE_MUTEX(hotplug_mutex);
#ifdef DO_LOGGING
DEFINE_MUTEX(logfile_mutex);
DEFINE_MUTEX(logstruct_mutex);
#endif

#ifdef THREAD_NAME_LOGGING
DEFINE_MUTEX(logfile_thread_name_mutex);
DEFINE_MUTEX(logstruct_thread_name_mutex);
#endif

//variables for task allocation
unsigned int a15frequency=1200e3; //in kHz!!! - variable holds the current processor frequency
unsigned int a7frequency=1000e3; //in kHZ!!!

int64_t a7space[4]={0, 0, 0, 0}; //space left (in cpu cycles) of each core
int64_t a15space[4]={0, 0, 0, 0};
const int64_t a7available_frequencies[5]={1000, 1100, 1200, 1300, 1400}; //in MHz!!!
const int64_t a15available_frequencies[9]={1200, 1300, 1400, 1500,  1600, 1700, 1800, 1900, 2000}; //in MHz!!!
short curr_frequ_a7_nr; //nr of target frequency (range: 0-4) ->convert to 'real' frequency with a7available_frequencies[curr_frequ_a7_nr]
short curr_frequ_a15_nr; //nr of target frequency (range: 0-8)
int64_t a7_space_increase_per_frequency=0; //as frequency always increases in steps of 0.1GHz the space increase (in cycles) is constant and can be computed once to save time
int64_t a15_space_increase_per_frequency=0; 

short shut_down_core=0;
short a15_core_online[4]={1};
short A15_online=1;

//counting threads assigned to the a15
int nr_tasks_a15;
int shutdown_counter_a15=0;

short logging_file_open =0;
short logging_file_thread_name_open=0;

//int for_tests_a15frequency_to_be_set=1200000;

struct file *fp_thread_name_logging=NULL;

#ifdef DO_LOGGING
struct file *fp_loggin_file=NULL;
//struct to be written in the logfile
log_struct log_str= {
    .nr=0,
    .a7_freq=0,
    .a15_freq=0,
    .frame_rate=0,
    .time_in=0,
    .fr_error=0,
    .cpu4_online=7,
    .cpu5_online=7,
    .cpu6_online=7,
    .cpu7_online=7,
};
#endif

//timer initialization 
static timer_struct autocorr_timer = {
    .time=0, //holds the time of the last update
    .update_interval=10e9, //update every 10 sec
    .timer_expired=1
};

//called if events occur
static int cpufreq_governor_dominiksgov(struct cpufreq_policy *policy, unsigned int event){
    mm_segment_t old_fs;
    KERNEL_DEBUG_MSG("GOV|CPU: %u, last CPU: %u\n", policy->cpu, policy->last_cpu);
    switch (event) {
        //event if governor has started->for each cpu independently
        case CPUFREQ_GOV_START:
            // TODO: REMOVE
            KERNEL_ERROR_MSG("GOV|GOV START Event called for CPU: %d, CPU online: %d\n", policy->cpu, cpu_online(policy->cpu));

            /*
             * shut_down_core is set if the cores are shutdown or rebooted. This causes a call of the
             * governor function with the start and stop events. These calls have to be ignored
             */
            if(shut_down_core==1){
                if (policy->cpu==4) {
                    number_cpus++;
                }
                break;
            }
            //set the is managed flag for the respective cpu
            if (policy->cpu==0 || policy->cpu==1 || policy->cpu==2 || policy->cpu==3){
                a7ismanaged=1;
            }
            else if(policy->cpu==4 || policy->cpu==5 || policy->cpu==6 || policy->cpu==7){
                a15ismanaged=1;
            }
            else{
                KERNEL_ERROR_MSG("GOV|ERROR: Unknown CPU-Nr.: %u \n", policy->cpu);
            }

            //increment number of used cpus
            number_cpus++;

            //initialize char device (only for first init)
            if (number_cpus==1) {
                if (IoctlInit()!=0){
                    KERNEL_ERROR_MSG("GOV|ERROR: could not initialize char device!\n");
                }
                else {
                    KERNEL_DEBUG_MSG("GOV|char device, successfully initialized!\n");
                }
            }

#ifdef DO_LOGGING
            /*
             * open logfile
             */
            old_fs = get_fs();
            set_fs(KERNEL_DS);

            mutex_lock(&logfile_mutex);
            if(logging_file_open==0){
                fp_loggin_file = filp_open(LOGFILE, (O_CREAT | O_TRUNC | O_WRONLY), (S_IRWXU | S_IRWXG | S_IRWXO));
            }
            mutex_unlock(&logfile_mutex);

            if (fp_loggin_file == NULL) {
                KERNEL_ERROR_MSG("GOV|Can't  open Logging File \n");
            }
            else{
                if(logging_file_open==0){
                    KERNEL_DEBUG_MSG("GOV|Opened Logging File \n");
                    logging_file_open=1;
                }
            }

            set_fs(old_fs);
#endif

#ifdef THREAD_NAME_LOGGING
            /*
             * open logfile
             */
            old_fs=get_fs();
            set_fs(KERNEL_DS);

            mutex_lock(&logfile_thread_name_mutex);
            if(logging_file_thread_name_open==0){
                fp_thread_name_logging=filp_open(LOGFILE_THREAD_NAME, (O_CREAT | O_TRUNC | O_WRONLY), (S_IRWXU | S_IRWXG | S_IRWXO));
            }
            mutex_unlock(&logfile_thread_name_mutex);

            if(fp_thread_name_logging==NULL){
                KERNEL_ERROR_MSG("GOV|Can't  open Thread Name Logging File \n");
            }
            else{
                if(logging_file_thread_name_open==0){
                    KERNEL_DEBUG_MSG("GOV|Opened Thread Name Logging File \n");
                    logging_file_thread_name_open=1;
                }
            }
            set_fs(old_fs);
#endif


            KERNEL_DEBUG_MSG("GOV|setting to %u kHz because of event %u\n", policy->min, event);
            __cpufreq_driver_target(policy, policy->min, CPUFREQ_RELATION_L); //start with min frequency
            break;
        case CPUFREQ_GOV_LIMITS:
            break;

            //governor has been stopped
        case CPUFREQ_GOV_STOP:
            KERNEL_ERROR_MSG("GOV|GOV STOP Event called for CPU: %d, CPU online: %d\n", policy->cpu, cpu_online(policy->cpu));


            //reset ismanaged flag
            if ((policy->cpu==0 || policy->cpu==1 || policy->cpu==2 || policy->cpu==3) && cpu_online(policy->cpu) && shut_down_core == 0){
                a7ismanaged=0;
                //decrement numbers of cpus managed
                number_cpus--;
            }
            else if((policy->cpu==4 || policy->cpu==5 || policy->cpu==6 || policy->cpu==7) && cpu_online(policy->cpu) && shut_down_core == 0){
                a15ismanaged=0;
                KERNEL_ERROR_MSG("GOV|A15 not managed !\n");
                //decrement numbers of cpus managed
                number_cpus--;
            }
            else if (shut_down_core == 1){
                KERNEL_ERROR_MSG("GOV|GOV STOP MSG because of core shutdown \n");
                if (policy->cpu==4) {
                    number_cpus--;
                }
            }
            else{
                KERNEL_ERROR_MSG("GOV|ERROR: Unknown CPU-Nr.: %u\n", policy->cpu);
            }
            //remove char device if no cpu is managed anymore
            if (number_cpus==0) {
                IoctlExit();
#ifdef DO_LOGGING
                /*
                 * close logfile
                 */
                old_fs = get_fs();
                set_fs(KERNEL_DS);

                mutex_lock(&logfile_mutex);
                filp_close(fp_loggin_file, 0);
                logging_file_open=0;
                KERNEL_DEBUG_MSG("GOV|Closed Logging File \n");
                mutex_unlock(&logfile_mutex);

                set_fs(old_fs);
#endif

#ifdef THREAD_NAME_LOGGING
                /*
                 * close logfile
                 */
                old_fs = get_fs();
                set_fs(KERNEL_DS);

                mutex_lock(&logfile_thread_name_mutex);
                filp_close(fp_thread_name_logging, 0);
                logging_file_thread_name_open=0;
                KERNEL_DEBUG_MSG("GOV|Closed Thread Name Logging File \n");
                mutex_unlock(&logfile_thread_name_mutex);

                set_fs(old_fs);
#endif
            }

            KERNEL_DEBUG_MSG("GOV|Governor stopped");
            break;
        default:
            break;
    }
    return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_DOMINIKSGOV
static
#endif
struct cpufreq_governor cpufreq_gov_dominiksgov = {
    .name		= "dominiksgov",
    .governor	= cpufreq_governor_dominiksgov,
    .owner		= THIS_MODULE,
};

//register and unregister governor
static int __init cpufreq_gov_dominiksgov_init(void)
{
    return cpufreq_register_governor(&cpufreq_gov_dominiksgov);
}

static void __exit cpufreq_gov_dominiksgov_exit(void)
{
    cpufreq_unregister_governor(&cpufreq_gov_dominiksgov);
}

MODULE_AUTHOR("Dominik Füß <dominik.fuess@mytum.de>");
MODULE_DESCRIPTION("CPUfreq governor 'DominiksGov'");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_DOMINIKSGOV
fs_initcall(cpufreq_gov_dominiksgov_init);
#else
module_init(cpufreq_gov_dominiksgov_init);
#endif
module_exit(cpufreq_gov_dominiksgov_exit);


//---------------------------------------------------------------------
// IOCTL - Functions 
//---------------------------------------------------------------------


static long MyIOctl( struct file *File,unsigned int cmd, unsigned long arg  )
{	struct task_struct* task, *group_leader_task, *task_buffer;
    uint64_t frame_rate=0;
    static int64_t frame_rate_error=0;
    int64_t error_buff;
    static int64_t target_time=0;
    static int64_t target_time_upper_bound=0;
    static int64_t target_time_lower_bound=0;
    int64_t a7space_decrement=0;
    int64_t a15space_decrement=0;
    static uint64_t time_buf=0;
    ioctl_struct_new_frame ioctl_arg;
    static short iciotl_new_frame_inuse=0;
    static int64_t a7space_init=0;
    static int64_t a15space_init=0;
    int i;
    static short is_init=0;
    int nr_tasks_on_cpu[8]={0};

    struct cpufreq_policy *policy;
    int a;

    switch (cmd){
        //ioctl command from opengl library that a new frame is about to be processed
        case IOCTL_CMD_NEW_FRAME:
            // TODO: preempt_disable?
            //prevent overlaps in ioctl calls
            if(iciotl_new_frame_inuse==1){
                KERNEL_ERROR_MSG("GOV|ERROR: IOCTL overlap! \n");
                if (copy_from_user(&ioctl_arg, (ioctl_struct_new_frame *)arg, sizeof(ioctl_struct_new_frame))){
                    KERNEL_ERROR_MSG("GOV|ERROR: Copy from user failed! \n");
                    return -EACCES;
                }
                time_buf=ioctl_arg.time;
                break;
            }
            iciotl_new_frame_inuse=1;

            //copy arguments from user space
            if (copy_from_user(&ioctl_arg, (ioctl_struct_new_frame *)arg, sizeof(ioctl_struct_new_frame))){
                KERNEL_ERROR_MSG("GOV|ERROR: Copy from user failed! \n");
                //preempt_enable(); // TODO: comment in and test
                return -EACCES;
            }

            //calculate actual frame rate
            frame_rate = div64_u64(10e9, ioctl_arg.time-time_buf); //framerate=10^9/(time_now-time_last_call) ->10e9 to get higher precission (305=30.5fps)
            if (frame_rate>1000){ //sometimes framerates over 100 occur (mostly if game or level is loading)
                KERNEL_WARNING_MSG("GOV|WARNING: Invalid Frame Rate: %llu\n", frame_rate);
                frame_rate=1000; // handle too high fps like a lower framerate
                time_buf=ioctl_arg.time-div64_u64(10e9, 100); // TODO: test
                //goto exit;
            }

            //calculate frame rate error
            if (target_time==0){
                target_time=div_s64(1e9, TARGET_FRAME_RATE);
            }
            if (target_time_upper_bound==0){
                target_time_upper_bound=div_s64(1e9, FRAME_RATE_UPPER_BOUND);
            }
            if (target_time_lower_bound==0){
                target_time_lower_bound=div_s64(1e9, FRAME_RATE_LOWER_BOUND);
            }
            //time for processing was bigger than the target time -> frame rate too low
            if(is_init){
                error_buff=(((int64_t)ioctl_arg.time)-time_buf)- (int64_t)ioctl_arg.sleep_time -target_time;
                //if (error_buff < target_time){
                if (error_buff < target_time*2){ // TODO: test 7.5 fps limit
                    if(error_buff>(target_time - target_time_upper_bound)){ //FRAME MISS
                        frame_rate_error+=div_s64(error_buff*AGGRESSIVENESS_FACTOR_UP, 100);
                    }
                    else if(error_buff < (target_time - target_time_lower_bound)){ //FRAME RATE TOO HIGH
                        frame_rate_error+=div_s64(error_buff*AGGRESSIVENESS_FACTOR_DOWN, 100);
                    }
                    else{ //FRAME HIT
                        frame_rate_error=div_s64(frame_rate_error * (100 - FRAME_HIT_REDUCTION_FACTOR), 100); //reduce framerate error by 5 percent
                    }
                    if(frame_rate_error<0){
                        frame_rate_error=0;
                    }
                    else if(frame_rate_error > target_time){
                        frame_rate_error=target_time;
                    }
                    a7space_decrement= div_s64(frame_rate_error*a7available_frequencies[0], 1e3); //10^-9 s * 10^6 1/s = 10^-3
                    a15space_decrement=div_s64(frame_rate_error*a15available_frequencies[0], 1e3);
                    a15space_decrement=convert_a15cpucycles_to_a7(a15space_decrement);
                    //}
                }
            }

            time_buf=ioctl_arg.time;//holds the last time the function was called
            number_allocation_chances=0;

            //check if both cpus are managed by the governor
            if(!(a7ismanaged==1 && a15ismanaged==1)){
                KERNEL_ERROR_MSG("GOV|ERROR: Not all CPUs are managed: A7: %d, A15: %d \n", a7ismanaged, a15ismanaged);
                goto exit;
            }

            //update timer
            update_timer(&autocorr_timer, ioctl_arg.time);

            //INIT task allocation varibales
            curr_frequ_a7_nr=0;//start with minimum frequency
            curr_frequ_a15_nr=0;

            //compute a7space_init and once to save time; space_init->the initial space (in cpu cycles) with minimum frequency
            if(a7space_init==0 || a15space_init==0){
                a7space_init=div_s64(a7available_frequencies[0]*1e6, TARGET_FRAME_RATE); //space_init = freuency (in GHz) / target frame rate
                a15space_init=div_s64(a15available_frequencies[0]*1e6, TARGET_FRAME_RATE);
                a15space_init=convert_a15cpucycles_to_a7(a15space_init); //a15 can do more computations within the same time -> multiply with constant factor
            }

            //compute space_increase_per frequency once to save time
            if(a7_space_increase_per_frequency==0 || a15_space_increase_per_frequency==0){
                a7_space_increase_per_frequency=div_s64(1e8, TARGET_FRAME_RATE) - div_s64(frame_rate_error, 10); //0.1 *10^9 / FrAME_RATE - 0.1 (1/s * 10^9) * error (s *10^-9)
                a15_space_increase_per_frequency=convert_a15cpucycles_to_a7(a7_space_increase_per_frequency);
            }

            //init the 4 space-slots of both processors
            for(i=0; i<4; i++){
                a7space[i]=a7space_init-a7space_decrement;
                a15space[i]=a15space_init-a15space_decrement;
            }

            //reset counter
            nr_tasks_a15=0;

            for(a=0;a<8; a++){
                nr_tasks_on_cpu[a]=0;
            }

            if(!is_init) {
                char buf[TASK_COMM_LEN];
                get_task_comm(buf, current);
                KERNEL_ERROR_MSG("GOV|CURRENT: pid=%i, comm=%s",current->pid, buf);
            }

            //find the froup leader task
            group_leader_task=current->group_leader;

            // TODO: test
            while (nr_write_threads2 > 0) {
                KERNEL_ERROR_MSG("GOV|nr_write_threads2 > 0 (%lld)!\n",nr_write_threads2);
                udelay(100);
            }
            //iterate through all tasks
            list_for_each_entry_safe(task, task_buffer ,&(group_leader_task->thread_group), thread_group){
                //if(task->pid == current->pid) {
                //    // TODO: handle?
                //}
                //else {
                    process_task(task);
                    nr_tasks_on_cpu[task->task_informations->allocated_core]+=1;
                //}
            }

            //process group leader separately as it will not appear in the list_for_each_entry_safe loop
            process_task(group_leader_task);
            nr_tasks_on_cpu[group_leader_task->task_informations->allocated_core]+=1;

            //disable a15 if no tasks are assigned to it
            if (nr_tasks_a15==0 && (cpu_online(4) || cpu_online(5) || cpu_online(6) || cpu_online(7))){
                //shutting down cores take some time -> start a new kernel thread
                kthread_run(&shutdown_a15, NULL, "shutdown_thread");
            }

            //set the cpu frequencies
            policy=cpufreq_cpu_get(0);
            if (policy) {
                cpufreq_set(policy, a7available_frequencies[curr_frequ_a7_nr]*1000); //cpufreq_set wants to have the frequency in kHz
                cpufreq_cpu_put(policy);
            } else {
                KERNEL_ERROR_MSG("GOV|Policy for A7 not available!\n");
            }

            //!!!!always use cpufreq_cpu_put after cpufreq_cpu_get to release resources!!!!!!
            if(cpu_online(4) && cpu_online(5) && cpu_online(6) && cpu_online(7) && A15_online==1){
                mutex_lock(&hotplug_mutex);
                policy = cpufreq_cpu_get(4);
                if (policy) {
                    cpufreq_set(policy, a15available_frequencies[curr_frequ_a15_nr]*1000);
                    cpufreq_cpu_put(policy);
                } else {
                    KERNEL_ERROR_MSG("GOV|Policy for A15 not available!\n");
                }
                mutex_unlock(&hotplug_mutex);
            }

#ifdef DO_LOGGING
            //copy data to log struct
            mutex_lock(&logfile_mutex);
            log_str.nr++;
            log_str.a7_freq=a7frequency;
            log_str.a15_freq=a15frequency;
            log_str.frame_rate=frame_rate;
            log_str.time_in=ioctl_arg.time;
            log_str.fr_error=frame_rate_error;
            log_str.cpu4_online=cpu_online(4);
            log_str.cpu5_online=cpu_online(5);
            log_str.cpu6_online=cpu_online(6);
            log_str.cpu7_online=cpu_online(7);
            for(a=0; a<8; a++){
                log_str.nr_tasks_on_cpu[a]=nr_tasks_on_cpu[a];
            }
            log_str.number_allocation_chances=number_allocation_chances;
            mutex_unlock(&logfile_mutex);

            //start a new kthread to write the log
            kthread_run(&write_log, &log_str, "log_thread");
#endif
            is_init=1;


exit:
            iciotl_new_frame_inuse=0;
            break;
        default:
            KERNEL_ERROR_MSG("GOV|ERROR:  IOCTL CMD not known!!\n");
            return(-1);
    }
    //preempt_enable(); // TODO: comment in and test
    return( 0 );
}

//functions called if char device is opened or closes -> don't do anything
static int MyOpen(struct inode *i, struct file *f)
{
    return 0;
}
static int MyClose(struct inode *i, struct file *f)
{
    return 0;
}

static ssize_t MyRead(struct file *file, char *buffer, size_t length, loff_t *offset)
{
    KERNEL_VERBOSE_MSG("GOV|device_read(%p,%p,%d)\n", file, buffer, length);
    return 0;
}


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,2,0)
static ssize_t MyWrite(struct file *file,
        const char *buffer,
        size_t length,
        loff_t *offset)
#else
static int device_write(struct inode *inode,
        struct file *file,
        const char *buffer,
        int length)
#endif
{
    KERNEL_VERBOSE_MSG ("GOV|Entered message: %s \n", buffer);
    KERNEL_VERBOSE_MSG ("GOV|device_write(%p,%s,%d)", file, buffer, length);

    return length;
}

static struct file_operations ioctlFops =
{
    .owner = THIS_MODULE,
    .open = MyOpen,
    .release = MyClose,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
    .ioctl = MyIOctl,
#else
    .unlocked_ioctl = MyIOctl,
#endif
    .read=MyRead,
    .write=MyWrite,
};

static int IoctlInit(void)
{//initialization of char device
    int ret;
    struct device *dev_ret;
    //int wfd;

    if ((ret = alloc_chrdev_region(&dev, FIRST_MINOR, MINOR_CNT, DOMINIKSGOV_CHAR_DEVICE_NAME)) < 0){
        return ret;
    }

    cdev_init(&c_dev, &ioctlFops);

    if ((ret = cdev_add(&c_dev, dev, MINOR_CNT)) < 0)
    {
        return ret;
    }

    if (IS_ERR(cl = class_create(THIS_MODULE, DOMINIKSGOV_CHAR_DEVICE_NAME "char")))
    {
        cdev_del(&c_dev);
        unregister_chrdev_region(dev, MINOR_CNT);
        return PTR_ERR(cl);
    }

    //cl->dev_uevent=my_dev_uevent;
    //cl->devnode=tty_devnode;
    //cl->dev_attrs->attr.mode=0666;

    if (IS_ERR(dev_ret = device_create(cl, NULL, dev, NULL, DOMINIKSGOV_CHAR_DEVICE_NAME)))
    {
        class_destroy(cl);
        cdev_del(&c_dev);
        unregister_chrdev_region(dev, MINOR_CNT);
        return PTR_ERR(dev_ret);
    }


    return 0;
}

static void IoctlExit(void)
{//remove char device
    KERNEL_VERBOSE_MSG("GOV|Cleanup_module called\n");
    device_destroy(cl, dev);
    class_destroy(cl);
    cdev_del(&c_dev);
    unregister_chrdev_region(dev, MINOR_CNT);
}

//--------------------------End of ioctl interface------------------------------------

//set cpu frequency
static int cpufreq_set(struct cpufreq_policy *policy, unsigned int freq)
{
    int ret = -EINVAL;
    KERNEL_VERBOSE_MSG("GOV|cpufreq_set for cpu %u, freq %u kHz\n", policy->cpu, freq);

    ret = __cpufreq_driver_target(policy, freq, CPUFREQ_RELATION_L);

    if(!ret){
        if(policy->cpu<4 && policy->cpu>=0){//setting frequency of the A7
            a7frequency=freq;
        }
        else if(policy->cpu>=4 && policy->cpu<8) //setting frequency of the A15
        {
            a15frequency=freq;
        }
        else {
            KERNEL_ERROR_MSG("GOV|ERROR: Set invalid core %u to frequency: %u \n", policy->cpu, freq);
        }
    }
    else{
        KERNEL_ERROR_MSG("GOV|ERROR: Setting frequency to %u for cpu: %d failed!\n", freq, policy->cpu);
    }

    return ret;
}

//task processing
void inline process_task(struct task_struct *task){
    //if task was not initialized yet -> initialize
    if(task->task_struct_expansion_is_initialized==0) {
        init_task_struct_expansion(task);
    }
    //if task was cloned from another task -> do new initialization
    if(task->task_informations->pid!=task->pid) {
        init_task_struct_expansion(task);
    }

    mutex_lock(&task->task_informations->lock);

    //update the workload history of the task
    update_workload_history(task);

    //if autocorrtimer has expired -> update the autocorellation of the task
    if(autocorr_timer.timer_expired==1){
        update_autocorr(task);
    }

    //perform the workload prediction for the task
    perform_workload_prediction(task);

    //allocate the task to a core and (if needed) increase the cpu frequency (variable)
    perform_task_allocation(task);

    thread_count++;

    mutex_unlock(&task->task_informations->lock);
}

//set the affinity of a task to a core (core_nr: 0-7)
long inline sched_setaffinity_own(struct task_struct *task, short core_nr){
    //test for valid core_nr
    int count_loop=0;

    if(core_nr>=0 && core_nr<nr_cpu_ids){
        if(task->task_informations->allocated_core != core_nr){
            number_allocation_chances+=1;
        }
        task->task_informations->allocated_core=core_nr;
        //enable a15 before assigning a task to it
        if(core_nr>3){
            shutdown_counter_a15=0;
            if(!cpu_online(core_nr)){
                kthread_run(&enable_a15, NULL, "enable_a15_thread");
            }
            nr_tasks_a15++;

            //wait until core is available
            while(!cpu_online(core_nr)){
                count_loop++;
                schedule();
                udelay(100);

                //check number of iterations to not get an infinity loop!
                if(count_loop>10000){
                    KERNEL_ERROR_MSG("GOV|Timeout for boot of core nr %d (Time > 1s)\n Can't assign task!\n", core_nr);
                    return -998;
                }
            }
            if(count_loop>0) {
                KERNEL_WARNING_MSG("GOV|WARNING:Waiting for core took %d iterations\n", count_loop);
            }
        }

        //set task affinity
        return sched_setaffinity(task->pid, get_cpu_mask((unsigned int)core_nr));}
    else {
        KERNEL_ERROR_MSG("GOV|ERROR: Wrong CPU-Number: %d\n", core_nr);
        return -999;
    }
}

//initialization of expansions of the task_struct
void inline init_task_struct_expansion(struct task_struct *task){
    int i;
    static short core_init=0;
    //char buf[TASK_COMM_LEN];
    //if (task->task_struct_expansion_is_initialized!=0 && task->task_informations->pid!=task->pid) {
    //    get_task_comm(buf, task);
    //    KERNEL_ERROR_MSG("GOV|CLONE of Task %d (%s)\n", task->pid, buf);
    //}
    //allocate memory for the task_struct_expansion struct
    //
    //if(!(task->task_struct_expansion_is_initialized!=0 && task->task_informations->pid!=task->pid && task->pid == current->pid)) {
        task->task_informations=(task_struct_expansion *)kmalloc(sizeof(task_struct_expansion), GFP_KERNEL);
        if(task->task_informations==NULL){
            KERNEL_ERROR_MSG("GOV|ERROR:  INIT of Task: %d  FAILED\n", task->pid);
            return;
        }
        
        //init all vars inside the struct
        mutex_init(&task->task_informations->lock);
        task->task_informations->pid=task->pid;
        task->task_struct_expansion_is_initialized=1;
        for (i=0; i<SIZE_WORKLOAD_HISTORY; i++){
            task->task_informations->workload_history[i]=0;
        }
        task->task_informations->autocorr_max=0;
        task->task_informations->autocorr_shift=0;
        task->task_informations->prediction=0;
        task->task_informations->prediction_cycles=0;
        task->task_informations->cpu_time=0;
        task->task_informations->allocated_core=core_init;

        core_init++;
        //spread the initial core affinity over all cores of the A7
        if(core_init>3){
            core_init=0;
        }
    //} else {
    //    if(task->task_informations==NULL){
    //        KERNEL_ERROR_MSG("GOV|ERROR: CLONE of Task: %d  FAILED\n", task->pid);
    //        return;
    //    }
    //    task->task_informations->pid=task->pid;
    //}
}

//get the workload of a task of the last frame and update the workload history
void inline update_workload_history(struct task_struct *task){
    int i;
    int64_t time_new;

    //shift history by one
    for (i=SIZE_WORKLOAD_HISTORY-1; i>0; i--){
        task->task_informations->workload_history[i]=task->task_informations->workload_history[i-1];
    }
    //get new total runtime
    time_new=(int64_t)task->se.sum_exec_runtime;//se.sum_exec_runtime total runtime in user and kernel space (in ns!!!)
    task->task_informations->workload_history[0]=time_new - task->task_informations->cpu_time;//save new workload at the beginning of the history
    task->task_informations->cpu_time=time_new; //save runtime for the calculation of the next workload
}

//predict the next workload of a task
void inline perform_workload_prediction(struct task_struct *task){
    //prediction using the hybrid WMA predictor
    task->task_informations->prediction=WMA_hybrid_predictor(task->task_informations->workload_history, SIZE_WORKLOAD_HISTORY, task->task_informations->autocorr_max, task->task_informations->autocorr_shift);
    //convert the prediction from time to processor cycles
    if(task->task_informations->allocated_core<4 && task->task_informations->allocated_core>=0){//task was allocated to one of the A7 cores
        task->task_informations->prediction_cycles= div_s64(task->task_informations->prediction*a7frequency, 1e6);//prediction in cycles=(prediction * frequency)/10^6
        //                       10^-9s  *  10^3(1/s) => 10^⁻6
    }
    else if(task->task_informations->allocated_core>=4 && task->task_informations->allocated_core<8) //task was allocated to one of the A15 cores
    {
        task->task_informations->prediction_cycles= div_s64(task->task_informations->prediction*a15frequency, 1e6);
        task->task_informations->prediction_cycles=convert_a15cpucycles_to_a7(task->task_informations->prediction_cycles);//convert to unified processor cycles
    }
    else {
        KERNEL_ERROR_MSG("GOV|ERROR: Task %d was allocated to a invalid core Nr: %d \n", task->pid, task->task_informations->allocated_core);
    }
}

//update a timer
void update_timer(timer_struct *tr, uint64_t new_time){
    //reset timer_expired var
    if (tr->timer_expired==1){
        tr->timer_expired=0;
    }
    //check if timer has expired
    if ((new_time - tr->time) > tr->update_interval){
        tr->timer_expired=1;
        tr->time=new_time;
    }
}

//update the autocorrelation informations of a task
void inline update_autocorr(struct task_struct *task){
    int64_t corr[SIZE_WORKLOAD_HISTORY]={-1};
    int maxpos;
    int64_t corrmax;
    //compute the autocorrelation for 20 lags
    autocorr(task->task_informations->workload_history, corr, SIZE_WORKLOAD_HISTORY-1, SIZE_WORKLOAD_HISTORY);
    //get maximum of the autocors and the shift of the maximum
    get_max(&corr[1], SIZE_WORKLOAD_HISTORY-1, &corrmax, &maxpos);
    task->task_informations->autocorr_max=corrmax;
    task->task_informations->autocorr_shift=maxpos;

#ifdef THREAD_NAME_LOGGING
    kthread_run(&write_thread_name_log, task, "thread_name_log_thread");
#endif
}

//convert processor cycles of the a15 to unified processor cycles
int64_t inline convert_a15cpucycles_to_a7(int64_t cycles_in){
    int64_t cycles_out;
    cycles_out=div_s64((cycles_in*CPUCYCLE_COVERSION_FACTOR), 100);//new_cycles=old_cycles * 1.71
    return cycles_out;
}

//allocate the task to a sufficient core and update the cpu-frequency variables accordingly
void inline perform_task_allocation(struct task_struct *task){
    //nr of core with maximum space left
    short a7core_max_space;
    short a15core_max_space;
    short curr_frequ_a7_nr_buff;
    int64_t a7space_buff[4];
    int i;
    //--------------------------------------------------
    //if prediction is zero: keep the old configuration-
    //--------------------------------------------------

    if (task->task_informations->prediction_cycles==0){
        return;
    }

    //--------------------------------------------
    //Try to assign to A7-------------------------
    //--------------------------------------------
    a7core_max_space=get_max_spaceA7(NULL);
    //buffer the A7 frequency informations -> if the task can't be assigned to the A7, the old values have to be restored
    curr_frequ_a7_nr_buff=curr_frequ_a7_nr;
    for(i=0; i<4; i++){
        a7space_buff[i]=a7space[i];
    }

    while(1){
        //check if task was assigned to A7, if yes->check if old core has enough space for task
        if(task->task_informations->allocated_core<4){
            if(check_space_left_and_assignA7(task, task->task_informations->allocated_core)){
                return;
            }
        }

        //try to assign to core with max space left
        if(check_space_left_and_assignA7(task, a7core_max_space)){
            return;
        }

        //if maximum frequency is reached -> frequency can't be increased anymore -> task has to be assigned to the A15
        if (curr_frequ_a7_nr==4){
            break;
        }
        else{ //(theoretically) increase the frequency of the A7
            curr_frequ_a7_nr++;
            //increase the A7 space
            for(i=0; i<4; i++){
                a7space[i]+=a7_space_increase_per_frequency;
            }
        }
    }

    //reset target frequency as no frequency increase is needed
    curr_frequ_a7_nr = curr_frequ_a7_nr_buff;
    for(i=0; i<4; i++){
        a7space[i]=a7space_buff[i];
    }

    //--------------------------------------------
    //Assign to A15-------------------------------
    //--------------------------------------------
    a15core_max_space=get_max_spaceA15(NULL);

    while(1){
        //check if task was assigned to A15, if yes->check if old core has enough space for task
        if(task->task_informations->allocated_core>=4){
            if(check_space_left_and_assignA15(task, task->task_informations->allocated_core)){
                return;
            }
        }

        //try to assign to core with max space left
        if(check_space_left_and_assignA15(task, a15core_max_space)){
            return;
        }


        if(curr_frequ_a15_nr==8){ //no space found!! -> assign to core with most space left
            a15space[a15core_max_space-4]=a15space[a15core_max_space-4] - task->task_informations->prediction_cycles;
            sched_setaffinity_own(task, a15core_max_space);
            return;
        }
        else{
            curr_frequ_a15_nr++;
            for(i=0; i<4; i++){
                a15space[i]+=a15_space_increase_per_frequency;
            }
        }
    }
}

//check if there is enough space on a core, if yes -> assign to that core
short inline check_space_left_and_assignA7(struct task_struct *task, short core_nr){
    if (core_nr>=4 || core_nr < 0){
        KERNEL_ERROR_MSG("GOV|ERROR: Wrong A7 core number: %d \n", core_nr);
    }
    if(a7space[core_nr] >= task->task_informations->prediction_cycles){
        a7space[core_nr] -= task->task_informations->prediction_cycles;
        sched_setaffinity_own(task, core_nr);
        return 1;
    }
    else{
        return (0);//0 -> no success
    }
}

short inline get_max_spaceA7(void *pointer){
    int i;
    short core=0;
    //int64_t buf=a7space[0];

    for (i=1; i<4; i++){
        if(a7space[i] > a7space[core]){
            core = i;
        }
    }
    return core;
}

short inline get_max_spaceA15(void *pointer){
    int i;
    short core=0;
    //int64_t buf=a7space[0];

    for (i=1; i<4; i++){
        if(a15space[i] > a15space[core]){
            core = i;
        }
    }
    return core+4;
}

short inline check_space_left_and_assignA15(struct task_struct *task, short core_nr){//core number ={4-7}
    if (core_nr<4 || core_nr > 7){
        KERNEL_ERROR_MSG("GOV|ERROR: Wrong A15 core number: %d \n", core_nr);
    }
    if(a15space[core_nr-4] >= task->task_informations->prediction_cycles){
        a15space[core_nr-4] -= task->task_informations->prediction_cycles;
        sched_setaffinity_own(task, core_nr);
        return 1;
    }
    else{
        return (0);//0 -> no success
    }
}



int shutdown_a15(void * in){
    int a=0, ret_shutdown=0;
    shut_down_core=1;
    shutdown_counter_a15++;
    cpu_hotplug_driver_lock();
    mutex_lock(&hotplug_mutex);
#ifdef CONFIG_HOTPLUG_CPU
#ifdef POWERGATING_GAMEOPTIMIZED_GOV
    //only shut down after shutdown_counter_a15 exceeds the shutdown limit
    if(shutdown_counter_a15>SHUTDOWN_LIMIT){
        A15_online=0;
        shutdown_counter_a15=0;
        KERNEL_ERROR_MSG("GOV|DISABLE A15 \n");
        for (a=4; a<8; a++){
            if (cpu_online(a)){
                //shut down cpu
                ret_shutdown += cpu_down(a);
                if (ret_shutdown){
                    KERNEL_ERROR_MSG("GOV|Can't shut down CPU %d \n", a);
                }
            }
        }
    }
#endif
#endif
    mutex_unlock(&hotplug_mutex);
    cpu_hotplug_driver_unlock();
    shut_down_core=0;
    do_exit(0);
    return ret_shutdown;
}

int __cpuinit enable_a15(void * in){
    int a=0, ret_enable=0;
    A15_online=1;
    shut_down_core=1;
    cpu_hotplug_driver_lock();
    mutex_lock(&hotplug_mutex);
#ifdef CONFIG_HOTPLUG_CPU
#ifdef POWERGATING_GAMEOPTIMIZED_GOV
    KERNEL_ERROR_MSG("GOV|ENABLE A15 \n");
    for (a=4; a<8; a++){
        if (!cpu_online(a)){
            ret_enable += cpu_up(a);
            if (ret_enable){
                KERNEL_ERROR_MSG("GOV|Can't bring up CPU %d \n", a);
            }
        }
    }
#endif
#endif
    mutex_unlock(&hotplug_mutex);
    cpu_hotplug_driver_unlock();
    shut_down_core=0;
    do_exit(0);
    return ret_enable;
}
//#endif

int64_t nr_write_threads=0;


#ifdef DO_LOGGING
int write_log(void *in){
    log_struct log_str_own;
    mm_segment_t old_fs;

    if (in==NULL || fp_loggin_file==NULL){
        KERNEL_ERROR_MSG("GOV|Can't write log beacuse file is closed\n");
        do_exit(-99);
        return -99;
    }

    nr_write_threads++;

    mutex_lock(&logstruct_mutex);
    log_str_own=*(log_struct*)in;
    mutex_unlock(&logstruct_mutex);
    old_fs = get_fs();
    set_fs(KERNEL_DS);

    mutex_lock(&logfile_mutex);
    fp_loggin_file->f_op->write(fp_loggin_file, (char *)&log_str_own, sizeof(log_str_own), &fp_loggin_file->f_pos);
    mutex_unlock(&logfile_mutex);

    set_fs(old_fs);

    nr_write_threads--;

    do_exit(0);
}
#endif

#ifdef THREAD_NAME_LOGGING
int write_thread_name_log(void *in){
    mm_segment_t old_fs;
    struct task_struct *ts;
    char buf[16]={"\0"};
    int a; //, b;

    ts=(struct task_struct*)in;

    if (in==NULL || fp_thread_name_logging==NULL){
        KERNEL_ERROR_MSG("GOV|Can't write log because file is closed\n");
        do_exit(-99);
        return -99;
    }

    //KERNEL_ERROR_MSG("GOV|Writing Thread Name Log, Nr Threads waiting: %lld\n", nr_write_threads2);
    //udelay(100);
    nr_write_threads2++;
    old_fs = get_fs();
    set_fs(KERNEL_DS);

    mutex_lock(&logfile_thread_name_mutex);
    fp_thread_name_logging->f_op->write(fp_thread_name_logging, ts->comm, TASK_COMM_LEN, &fp_thread_name_logging->f_pos);
    mutex_lock(&ts->task_informations->lock);
    fp_thread_name_logging->f_op->write(fp_thread_name_logging, "; ", 2, &fp_thread_name_logging->f_pos);
    sprintf(buf, "%lld", ts->task_informations->autocorr_max );
    fp_thread_name_logging->f_op->write(fp_thread_name_logging, buf, strlen(buf), &fp_thread_name_logging->f_pos);
    for(a=0; a<SIZE_WORKLOAD_HISTORY ; a++){
        //for(b=0; b<16; b++){
        //	buf[b]='\0';
        //}

        fp_thread_name_logging->f_op->write(fp_thread_name_logging, "; ", 2, &fp_thread_name_logging->f_pos);
        sprintf(buf, "%lld", ts->task_informations->workload_history[a]);
        fp_thread_name_logging->f_op->write(fp_thread_name_logging, buf, strlen(buf), &fp_thread_name_logging->f_pos);
    }


    fp_thread_name_logging->f_op->write(fp_thread_name_logging, " \n", 2, &fp_thread_name_logging->f_pos);

    //KERNEL_ERROR_MSG("GOV|ACorr: %lld \n", ts->task_informations->autocorr_max);
    nr_write_threads2--;
    mutex_unlock(&ts->task_informations->lock);
    mutex_unlock(&logfile_thread_name_mutex);
    set_fs(old_fs);

    do_exit(0);



}
#endif



