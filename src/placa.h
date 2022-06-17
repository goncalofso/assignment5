/** @file Placa.h
 * @brief Variaveis e programação da placa
 *
 * Este modulo inicia as portas da placa que queremos usar e verifica
 * se não ocorreu nenhum erro
 * 
 * 
 * @author Goncalo Moniz, João Oliveira, Diogo Leão
 * @bug No known bugs.
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>
#include <timing/timing.h>
#include <stdio.h>
#include <string.h>

#ifndef _PLACA_H
#define _PLACA_H

/* Refer to dts file */
#define GPIO0_NID DT_NODELABEL(gpio0) 

#define button1 0xb
#define button2 0xc
#define button3 0x18
#define button4 0x19

#define SAMP_PERIOD_MS 100

extern volatile int dcToggleFlag;

extern uint16_t cd;
extern struct k_sem sem_cd;

void button1_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void button2_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void button3_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void button4_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/**
 * @brief Inicia o Modulo
 * 
 * Esta função inicializa o modulo
 */
void InitConfig(void);

void thread_D_code(void *argA , void *argB, void *argC);

#endif