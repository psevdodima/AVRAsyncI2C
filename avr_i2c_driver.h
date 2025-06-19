#ifndef AVR_I2C_DRIVER_H_
#define AVR_I2C_DRIVER_H_

#define I2C_TRANSMIT_BUF_SIZE   32
#define I2C_RECEIVE_BUF_SIZE	32

#include <avr/io.h>
#include "ubuf.h"
#include "prj_com.h"

#ifndef AVR_I2C_DRV_DEBUG 
	#define AVR_I2C_DRV_DEBUG 0
#endif

#define F_I2C 100000UL 

typedef uint8_t i2c_addr_t;

typedef struct i2c_drv_s
{
	// Buffers
	CyclicBuffer _transmit_buf;
	CyclicBuffer _receive_buf;
	uint8_t _cbuf_tx_ram_mem[I2C_TRANSMIT_BUF_SIZE];
	uint8_t _cbuf_rx_ram_mem[I2C_RECEIVE_BUF_SIZE];
	// Queue
	volatile uint8_t _op_queue_elm_cnt;
	volatile uint8_t _op_queue_elm_size;
	volatile uint8_t _in_queue_elm_cnt;
	// State info
	volatile prj_com_state_et _state;
	uint8_t _prev_addr;
}i2c_drv_st;

//------------------------------------------------- NOT FOR DIRECT USE ------------------------------------------------------
prj_com_op_res_et operate_start(i2c_drv_st* drv, i2c_addr_t addr, const uint8_t* data, uint8_t size, bool is_read, uint8_t tr_uuid);
//---------------------------------------------------------------------------------------------------------------------------
#define i2c_drv_read(drv, addr, size, uuid) operate_start(drv, addr, NULL, size, true, uuid)
#define i2c_drv_write(drv, addr, data, size) operate_start(drv, addr, data, size, false, NULL)
prj_com_op_res_et i2c_drv_trans_plus_byte (i2c_drv_st* drv, i2c_addr_t addr, uint8_t* data, uint8_t size, uint8_t byte);
void i2c_drv_run(i2c_drv_st* drv);// Call in main loop

void i2c_drv_data_received_cmplt (i2c_drv_st* drv, uint8_t addr, uint8_t size, uint8_t uuid, uint8_t *data);
void i2c_drv_target_not_recognize_clbk (i2c_drv_st* drv, i2c_addr_t addr);
void i2c_drv_target_data_nack (i2c_drv_st* drv, uint16_t dat_lef);
void i2c_drv_bus_error (i2c_drv_st* drv);

prj_com_op_res_et i2c_drv_init (i2c_drv_st* drv);

void i2c_drv_irq_handler(i2c_drv_st* drv);

#endif /* AVR_I2C_DRIVER_H_ */