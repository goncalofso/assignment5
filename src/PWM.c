#include "PWM.h"



#include <zephyr.h>
#include <device.h>
#include <drivers/uart.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define MAIN_SLEEP_TIME_MS 10 /* Time between main() activations */ 

#define FATAL_ERR -1 /* Fatal error return code, app terminates */

#define UART_NID DT_NODELABEL(uart0)    /* UART Node label, see dts */
#define RXBUF_SIZE 60                   /* RX buffer size */
#define TXBUF_SIZE 60                   /* TX buffer size */
#define RX_TIMEOUT 1000                  /* Inactivity period after the instant when last char was received that triggers an rx event (in us) */

/* Struct for UART configuration (if using default valuies is not needed) */
const struct uart_config uart_cfg = {
		.baudrate = 115200,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};

/* UAR related variables */
const struct device *uart_dev;          /* Pointer to device struct */ 
static uint8_t rx_buf[RXBUF_SIZE];      /* RX buffer, to store received data */
static uint8_t rx_chars[RXBUF_SIZE];    /* chars actually received  */
volatile int uart_rx_rdy_flag;          /* Flag to signal main() that a message is available */

/* UART callback function prototype */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);



void thread_C_code(void *argA , void *argB, void *argC)
{
    
    /* Local variables */
    int ret=0;      
    int duty=100;
    const struct device *gpio0_dev;         /* Pointer to GPIO device structure */
    const struct device *pwm0_dev;          /* Pointer to PWM device structure */
    unsigned int pwmPeriod_us = 1000;       /* PWM priod in us */

    int mode = 2;
    
/* Bind to GPIO 0 and PWM0 */
    gpio0_dev = device_get_binding(DT_LABEL(GPIO0_NID));
    if (gpio0_dev == NULL) {
        printk("Error: Failed to bind to GPIO0\n\r");        
	return;
    }
    else {
        printk("Bind to GPIO0 successfull \n\r");        
    }
    
    pwm0_dev = device_get_binding(DT_LABEL(PWM0_NID));
    if (pwm0_dev == NULL) {
	printk("Error: Failed to bind to PWM0\n r");
	return;
    }
    else  {
        printk("Bind to PWM0 successful\n\r");            
    }

    printk("Thread C init (sporadic, waits on a semaphore by task A)\n");

    uint16_t Ti, kp, integ=0 , saidaPI,ref,diff=0;
    uint16_t erro,erro2=0;
    uint16_t valorinserido = 1023;
    ref=1500; 
    int cnt1,soma1,array1[20];
    int tensaoMV;
    int tensaoambiente=(uint16_t)(1000*bc*((float)3/1023));







    /* Local vars */    
    int err=0; /* Generic error variable */
    uint8_t welcome_mesg[] = "UART demo: Type a few chars in a row and then pause for a little while ...\n\r"; 
    uint8_t rep_mesg[TXBUF_SIZE];

    /* Bind to UART */
    uart_dev= device_get_binding(DT_LABEL(UART_NID));
    if (uart_dev == NULL) {
        printk("device_get_binding() error for device %s!\n\r", DT_LABEL(UART_NID));
        return;
    }
    else {
        printk("UART binding successful\n\r");
    }

    /* Configure UART */
    err = uart_configure(uart_dev, &uart_cfg);
    if (err == -ENOSYS) { /* If invalid configuration */
        printk("uart_configure() error. Invalid configuration\n\r");
        return; 
    }

    /* Register callback */
    err = uart_callback_set(uart_dev, uart_cb, NULL);
    if (err) {
        printk("uart_callback_set() error. Error code:%d\n\r",err);
        return;
    }
		
    /* Enable data reception */
    err =  uart_rx_enable(uart_dev ,rx_buf,sizeof(rx_buf),RX_TIMEOUT);
    if (err) {
        printk("uart_rx_enable() error. Error code:%d\n\r",err);
        return;
    }

    /* Send a welcome message */ 
    /* Last arg is timeout. Only relevant if flow controll is used */
    err = uart_tx(uart_dev, welcome_mesg, sizeof(welcome_mesg), SYS_FOREVER_MS);
    if (err) {
        printk("uart_tx() error. Error code:%d\n\r",err);
        return;
    }
    printk("Escreva o valor pretendido\n\n");








    while(1) 
    {
        k_sem_take(&sem_cd, K_FOREVER);        
        
        
        printk("Botao: %d\r\n",cd);           //Escolher o modo, (Mode 1 => Manual), (Mode 2 => Automático)

        if(cd == 1) mode = 1;
        
        if(cd == 2) mode = 2;
        

        if(mode == 1)
        {
            printk("Modo Manual\r\n");

            if(cd == 3)
            {
                duty = duty + 10;
            }

            else if(cd == 4)
            {
                duty = duty - 10;
            }

            else if (duty > 100)
            {
                duty = 100;
            }

            else if (duty < 0)
            {
                duty = 0;
            }
             tensaoMV=(uint16_t)(1000*bc*((float)3/1023));printk("Tensao no sensor => %4u mv\n\r",tensaoMV); 
        }
         
        else
        {
            printk("Modo Automatico    REF->UP/DOWN\r\n");            
            tensaoMV=(uint16_t)(1000*bc*((float)3/1023));printk("Tensao no sensor => %4u mv\n\r",tensaoMV);   
                           
            
            //PI CONTROLER
             Ti=1.5;
             int Td=0.5;
             kp=0.8;
             if(cd == 3)
             {
               ref = ref + 100;
             }
             else if(cd == 4)
             {
                ref = ref - 100;
             }
             if(ref>=2500)
             {
               ref = 2500;
             }
             if(ref<=900)
             {
                ref = 900;
             }
             printk("(PID) ref = %4u mv\r\n",ref);
             erro2=erro;
             erro = ref-tensaoMV;
          
             if (erro>3000) 
             {  erro = 65535-erro;
                printk("(PID) erro = -%4u mv\r\n",erro);
                integ = integ - (erro2+erro)/2; printk("(PID) integ = %4u mv\r\n",integ);  
                diff=diff + (erro2+erro)/2; printk("(PID) diff = %4u mv\r\n",diff);             
             }
             else
             {
                printk("(PID) erro = %4u mv\r\n",erro);
                integ = integ +  (erro2+erro)/2; printk("(PID) integ = %4u mv\r\n",integ);   
                diff=diff - (erro2+erro)/2; printk("(PID) diff = %4u mv\r\n",diff);             
             } 
                        
             saidaPI=kp*erro+(1/Ti)*integ+Td*diff;               
               
             if (saidaPI > 3000)
             {
                 saidaPI = 65535-saidaPI ;
                 printk("(PID) saidaPI = %4u mv\r\n",saidaPI);    
             }
             else printk("(PID) saidaPI = %4u mv\r\n",saidaPI);    
             duty=(uint16_t)100-(saidaPI)/30;
             
             if (duty > 100)
             {
                 duty = 100;
             }

             if (duty < 0)
             {
                 duty = 0;
             }
             
        }  
        printk("PWM DC value set to %u %%\n\n\r",duty);
        printk("Escreva o valor pretendido =>\r\n\n");
        ret = pwm_pin_set_usec(pwm0_dev, BOARDLED_PIN,pwmPeriod_us,(unsigned int)((pwmPeriod_us*duty)/100), PWM_POLARITY_NORMAL);
        cd = 0;
        
  }
}
/* UART callback implementation */
/* Note that callback functions are executed in the scope of interrupt handlers. */
/* They run asynchronously after hardware/software interrupts and have a higher priority than all threads */
/* Should be kept as short and simple as possible. Heavier processing should be deferred to a task with suitable priority*/
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    int err;

    switch (evt->type) {
	
        case UART_TX_DONE:
		//printk("UART_TX_DONE event \n\r");
                break;

	case UART_TX_ABORTED:
		printk("UART_TX_ABORTED event \n\r");
		break;
		
	case UART_RX_RDY:
		//printk("UART_RX_RDY event \n\r");
                /* Just copy data to a buffer. Usually it is preferable to use e.g. a FIFO to communicate with a task that shall process the messages*/
                memcpy(rx_chars,&(rx_buf[evt->data.rx.offset]),evt->data.rx.len); 
                rx_chars[evt->data.rx.len]=0; /* Terminate the string */
                uart_rx_rdy_flag = 1;
		break;

	case UART_RX_BUF_REQUEST:
		printk("UART_RX_BUF_REQUEST event \n\r");
		break;

	case UART_RX_BUF_RELEASED:
		printk("UART_RX_BUF_RELEASED event \n\r");
		break;
		
	case UART_RX_DISABLED: 
                /* When the RX_BUFF becomes full RX is is disabled automaticaly.  */
                /* It must be re-enabled manually for continuous reception */
                printk("UART_RX_DISABLED event \n\r");
		err =  uart_rx_enable(uart_dev ,rx_buf,sizeof(rx_buf),RX_TIMEOUT);
                if (err) {
                    printk("uart_rx_enable() error. Error code:%d\n\r",err);
                    exit(FATAL_ERR);                
                }
		break;

	case UART_RX_STOPPED:
		printk("UART_RX_STOPPED event \n\r");
		break;
		
	default:
                printk("UART: unknown event \n\r");
		break;
    }

}