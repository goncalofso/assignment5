/** \file PWM.h
* \brief PWM Output
* 
* Process the data of tread B using pwm
* Output: pwm signal applied to one of the DevKit leds.
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

#ifndef _PWM_H
#define _PWM_H

//---------------PWM CONFIG

#define BOARDLED_PIN DT_PROP(PWM0_NID, ch0_pin)
#include <drivers/pwm.h>
/* Refer to dts file */
#define GPIO0_NID DT_NODELABEL(gpio0) 
#define PWM0_NID DT_NODELABEL(pwm0) 

extern uint16_t bc;
extern uint16_t cd;

extern struct k_sem sem_bc;
extern struct k_sem sem_cd;

/**Thread C
* 
* Thread for the PWM
* \author Goncalo Moniz, Joao Oliveira, Diogo Leao
* \param[in,out] 
*void
*void
* \return void
* \date 31-5-2022
*/
void thread_C_code(void *argA , void *argB, void *argC);


#endif