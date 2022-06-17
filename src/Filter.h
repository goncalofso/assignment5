/** \file Filter.h
* \brief Digital Filter
* 
* Process the data from the tread A using a filter 
* Digital filter: moving average filter, with a window size of 10 samples. Removes the
* outliers (10% or high deviation from average) and computes the average of the remaining
* samples.
*
* \author Goncalo Moniz, Joao Oliveira, Diogo Leao
* \date 31-5-2022
* \bug There are no bugs
*/

#include <zephyr.h>
#include <device.h>
#include <drivers/uart.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifndef _FILTER_H
#define _FILTER_H

extern uint16_t ab,bc;

extern struct k_sem sem_ab;
extern struct k_sem sem_bc;

/**Thread B 
* 
* Thread for the Filter
* \author Goncalo Moniz, Joao Oliveira, Diogo Leao
* \param[in,out] 
* void
* void
* \return void
* \date 31-5-2022
*/
void thread_B_code(void *argA , void *argB, void *argC);

#endif