#include "placa.h"

static struct gpio_callback button1_cb_data;
static struct gpio_callback button2_cb_data;
static struct gpio_callback button3_cb_data;
static struct gpio_callback button4_cb_data;

volatile int dcToggleFlag = 0;


void button1_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
        dcToggleFlag=1;
}

void button2_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
        dcToggleFlag=2;
}
void button3_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
        dcToggleFlag=3;
}
void button4_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
        dcToggleFlag=4;
}

void InitConfig(void){
      /* Local vars */    
    const struct device *gpio0_dev;         /* Pointer to GPIO device structure */
    int ret=0;                              /* Generic return value variable */


    /* Bind to GPIO 0 */
    gpio0_dev = device_get_binding(DT_LABEL(GPIO0_NID));
    if (gpio0_dev == NULL) {
        printk("Failed to bind to GPIO0\n\r");        
	return;
    }
    else {
        printk("Bind to GPIO0 successfull \n\r");        
    }
    
    ret = gpio_pin_configure(gpio0_dev, button1, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("gpio_pin_configure() failed with error %d\n\r", ret);        
	return;
    }
   ret = gpio_pin_configure(gpio0_dev, button2, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("gpio_pin_configure() failed with error %d\n\r", ret);        
	return;
    }
    ret = gpio_pin_configure(gpio0_dev, button3, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("gpio_pin_configure() failed with error %d\n\r", ret);        
	return;
    }
    ret = gpio_pin_configure(gpio0_dev, button4, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("gpio_pin_configure() failed with error %d\n\r", ret);        
	return;
    }

    
    	ret = gpio_pin_interrupt_configure(gpio0_dev,button1,GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		return;
	}
    	ret = gpio_pin_interrupt_configure(gpio0_dev,button2,GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		return;
	}
    	ret = gpio_pin_interrupt_configure(gpio0_dev,button3,GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		return;
	}
     	ret = gpio_pin_interrupt_configure(gpio0_dev,button4,GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		return;
	}

	gpio_init_callback(&button1_cb_data, button1_pressed, BIT(button1));
	gpio_add_callback(gpio0_dev, &button1_cb_data);

	gpio_init_callback(&button2_cb_data, button2_pressed, BIT(button2));
	gpio_add_callback(gpio0_dev, &button2_cb_data);

        gpio_init_callback(&button3_cb_data, button3_pressed, BIT(button3));
	gpio_add_callback(gpio0_dev, &button3_cb_data);

        gpio_init_callback(&button4_cb_data, button4_pressed, BIT(button4));

	gpio_add_callback(gpio0_dev, &button4_cb_data);    
        printk("Config botoes feita\r\n");
        return;

}

void thread_D_code(void *argA , void *argB, void *argC)
{
	InitConfig();
	int64_t fin_time=0, release_time=0;  
	release_time = k_uptime_get() + SAMP_PERIOD_MS;
   /* Timing variables to control task periodicity */    
	while(true)
    {
    	if(dcToggleFlag == 0)
    	{
    		k_sem_give(&sem_cd);
    	}
    	else
    	{
    		k_sem_give(&sem_cd);
    		cd = dcToggleFlag;
    		dcToggleFlag = 0;
    	}
    	/* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) 
        {
            k_msleep(release_time - fin_time);
            release_time += SAMP_PERIOD_MS;
        }

        /* Stop timing functions */
        timing_stop();

    }
}