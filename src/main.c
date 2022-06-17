/** \file main.c
* \brief The system does a basic processing of an analog signal. It reads the input voltage
from an analog sensor, digitally filters the signal and outputs it.
*
* The system toogles the duty cycle os the pwm signal that is connected to the LED1(output) in function of the concurrent voltage os the 
*potenciometer(input). The information is shared between the treads using Semaphores and Shared Memory
* 
*
* \author Goncalo Moniz, Joao Oliveira, Diogo Leao
* \date 31-5-2022
* \bug There are no bugs
*/

/**
@mainpage ASSIGNMENT 4 PART 1
@author Goncalo Moniz, Joao Oliveira, Diogo Leao

The aim of this assignment is to learn how to implement a set of cooperative real-time tasks in
Zephyr. Replicating the typical structure of embedded software, a mix of periodic and sporadic
tasks will be considered.

The system to implement does a basic processing of an analog signal. It reads the input voltage
from an analog sensor, digitally filters the signal and outputs it.

. Input sensor: Emulated by a 10 kO potentiometer, supplied by the DevKit 3 V supply
(VDD).

. Digital filter: moving average filter, with a window size of 10 samples. Removes the
outliers (10% or high deviation from average) and computes the average of the remaining
samples. 

. Output: pwm signal applied to one of the DevKit leds.

The system shall be structured with at least three tasks, matching the basic processing blocks,
namely one task for acquiring the sample, one for filtering and the other to output the signal.
The sampling task is periodic, while the other two are sporadic, being activated when new data is
available.


*/
#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <string.h>
#include <devicetree.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <timing/timing.h>
#include <stdio.h>
#include <sys/printk.h>
#include <hal/nrf_saadc.h>
#include "ADC.h"
#include "Filter.h"
#include "PWM.h"
#include "Placa.h"
#include "uart.h"


//----------------THREAD CONFIG


/* Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024
/* Thread scheduling priority */
#define thread_A_prio 1
#define thread_B_prio 1
#define thread_C_prio 1
#define thread_D_prio 1
/* Create thread stack space */
K_THREAD_STACK_DEFINE(thread_A_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_B_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_C_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_D_stack, STACK_SIZE);
/* Create variables for thread data */
struct k_thread thread_A_data;
struct k_thread thread_B_data;
struct k_thread thread_C_data;
struct k_thread thread_D_data;
/* Create task IDs */
k_tid_t thread_A_tid;
k_tid_t thread_B_tid;
k_tid_t thread_C_tid;
k_tid_t thread_D_tid;
/* Thread code prototypes */
void thread_A_code(void *, void *, void *);
void thread_B_code(void *, void *, void *);
void thread_C_code(void *, void *, void *);
void thread_D_code(void *, void *, void *);

//------------------SEMS CONFIG + SHARED MEMORY


struct k_sem sem_ab;
struct k_sem sem_bc;
struct k_sem sem_cd;
/** Shared memory betwen ADC and Filter.
 */ 
uint16_t ab;
/** Shared memory betwen Fiter and PWM.
 */ 
uint16_t bc;

uint16_t cd;


//----------------MAIN

/**brief Main function
* 
* Inicialization of Semaphores and Threads
* 
* \author Goncalo Moniz, Joao Oliveira, Diogo Leao
* \param[in,out] 
*void
*void
* \return void
* \date 31-5-2022
*/
void main(void)
{
 
    /* Create/Init Semaphores */
    k_sem_init(&sem_ab,0,1);
    k_sem_init(&sem_bc,0,1);
    k_sem_init(&sem_cd,0,1);
    /* Then create the task */
    thread_A_tid = k_thread_create(&thread_A_data, thread_A_stack,
    K_THREAD_STACK_SIZEOF(thread_A_stack), thread_A_code,
    NULL, NULL, NULL, thread_A_prio, 0, K_NO_WAIT);

    thread_B_tid = k_thread_create(&thread_B_data, thread_B_stack,
    K_THREAD_STACK_SIZEOF(thread_B_stack), thread_B_code,
    NULL, NULL, NULL, thread_B_prio, 0, K_NO_WAIT);

    thread_C_tid = k_thread_create(&thread_C_data, thread_C_stack,
    K_THREAD_STACK_SIZEOF(thread_C_stack), thread_C_code,
    NULL, NULL, NULL, thread_C_prio, 0, K_NO_WAIT);

    thread_D_tid = k_thread_create(&thread_D_data, thread_D_stack,
    K_THREAD_STACK_SIZEOF(thread_D_stack), thread_D_code,
    NULL, NULL, NULL, thread_D_prio, 0, K_NO_WAIT);
}