#include "avr_i2c_driver_appl.h"

#include "ds1307_rtc_app.h"
#include "ina219_appl.h"

#include "avr/interrupt.h"

i2c_drv_st i2c = { 0 };

prj_com_op_res_et avr_i2c_driver_appl_init (void)
{   /* The current version does not support multiple instances, 
	   so the speed and peripheral are hardcoded in the driver module */
	return i2c_drv_init (&i2c);
}

void i2c_drv_data_received_cmplt(i2c_drv_st* drv, uint8_t addr, uint8_t size, uint8_t uuid, uint8_t *data)
{  /* Distributes the received data among modules depending on the address */	
	switch(addr){
		case DS1307_IC_I2C_ADDR: 
			ds1307_data_received(&rtc, data, uuid, size);
			break;
	}
	
}

ISR (TWI_vect)
{
	i2c_drv_irq_handler(&i2c);
}