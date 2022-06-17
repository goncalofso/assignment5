/** \file ADC.h
* \brief ADC Input Sensor
* 
* Process the input signal using an adc
* Input sensor: Emulated by a 10 kO potentiometer, supplied by the DevKit 3 V supply(VDD).
* 
* \author Goncalo Moniz, Joao Oliveira, Diogo Leao
* \date 31-5-2022
* \bug There are no bugs
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

#ifndef _ADC_H
#define _ADC_H

/** Sampling Period
*/
#define SAMP_PERIOD_MS 10


#define ADC_NID DT_NODELABEL(adc) 
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_CHANNEL_ID 1  

//-----------------ADC CONFIG

#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1 
#define BUFFER_SIZE 1
#define TIMER_INTERVAL_MSEC 50 /* Interval between ADC samples */

extern uint16_t ab;
extern struct k_sem sem_ab;

/**Thread A 
* 
* Thread for the ADC
* \author Goncalo Moniz, Joao Oliveira, Diogo Leao
* \param[in,out] 
*void
*void
* \return void
* \date 31-5-2022
*/
void thread_A_code(void *argA , void *argB, void *argC);

#endif