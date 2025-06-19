#define TWSR_MASK				0xF8
#define TWI_SUCCESS				0xFF
#define TWI_NO_STATE			0x01
#define TWI_START				0x08
#define TWI_REP_START			0x10
#define TWI_MTX_ADR_ACK			0x18
#define TWI_MRX_ADR_ACK			0x40
#define TWI_MTX_DATA_ACK		0x28
#define TWI_MRX_DATA_ACK		0x50
#define TWI_MTX_ADR_NACK		0x20
#define TWI_MRX_ADR_NACK		0x48
#define TWI_MTX_DATA_NACK		0x30
#define TWI_MRX_DATA_NACK		0x58
#define TWI_BUS_ERROR			0x00
#define TWI_ARB_LOST			0x38

#define MSG_HEADER_SIZE 2 // Addr -> 1 byte, DataSize -> 1 byte
                   
#include "avr_i2c_driver.h"
#include <string.h>

#define TWBR_VALUE (((F_CPU)/(F_I2C)-16)/2)
#if ((TWBR_VALUE > 255) || (TWBR_VALUE == 0)) //TWI (TWBR)
#error "TWBR value is not correct"
#endif

prj_com_op_res_et i2c_drv_trans_plus_byte (i2c_drv_st* drv, i2c_addr_t addr, uint8_t* data, uint8_t size, uint8_t byte)
{
	CBufWriteByte(&drv->_transmit_buf, size + 1);
	#if AVR_I2C_DRV_DEBUG
		printf("I2C Start Queue WRITE plus byte (0x%X) OPERATION...\r\n", addr);
	#endif
	CBufWriteByte(&drv->_transmit_buf, addr & (~0x01));
	CBufWriteByte(&drv->_transmit_buf, byte);
	CBufWrite(&drv->_transmit_buf, data, size);
	drv->_op_queue_elm_cnt ++;
	return OP_OK;
}

prj_com_op_res_et operate_start (i2c_drv_st* drv, i2c_addr_t addr, const uint8_t* data, uint8_t size, bool is_read, uint8_t tr_uuid)
{
	CBufWriteByte(&drv->_transmit_buf, size);
	if(is_read){
		#if AVR_I2C_DRV_DEBUG
			printf("I2C Start Queue READ (0x%X) OPERATION...\r\n", addr);
		#endif
		CBufWriteByte(&drv->_transmit_buf, addr | (0x01));
		CBufWriteByte(&drv->_transmit_buf, tr_uuid);
	} else {
		#if AVR_I2C_DRV_DEBUG
			printf("I2C Start Queue WRITE (0x%X) OPERATION...\r\n", addr);
		#endif
		CBufWriteByte(&drv->_transmit_buf, addr & (~0x01));
		CBufWrite(&drv->_transmit_buf, data, size);
	}
	drv->_op_queue_elm_cnt ++;
	return OP_OK;
}

static bool go_to_next_packet(i2c_drv_st* drv)
{
	drv->_op_queue_elm_cnt --;
	if(drv->_op_queue_elm_cnt){
		cb_size_t exp_pos = drv->_transmit_buf.rpos + 1;
		cb_size_t size_pos = (exp_pos < drv->_transmit_buf.size) ? exp_pos : exp_pos - drv->_transmit_buf.size;
		if((drv->_transmit_buf.data[size_pos] & 0xFE) != drv->_prev_addr){
			return false;
		} else {
			drv->_op_queue_elm_size = CBufReadByteNoRes(&drv->_transmit_buf);
			#if AVR_I2C_DRV_DEBUG
				printf("I2C IRQ Queue element size: %d bytes started\r\n", drv->_op_queue_elm_size);
			#endif
			return true;
		}	
	}
	return false;
}

prj_com_op_res_et i2c_drv_init(i2c_drv_st* drv){
	if(!drv){
		return OP_ERROR_NULL;
	}
	CBufInit(&drv->_transmit_buf, drv->_cbuf_tx_ram_mem, I2C_TRANSMIT_BUF_SIZE);
	CBufInit(&drv->_receive_buf, drv->_cbuf_rx_ram_mem, I2C_RECEIVE_BUF_SIZE);
	TWBR = TWBR_VALUE;
	TWSR = 0;
	drv->_state = STATE_READY;
	return OP_OK;
}

void i2c_drv_run(i2c_drv_st* drv)
{
	static uint8_t message_arena[I2C_RECEIVE_BUF_SIZE] = { 0 };
	if(drv->_in_queue_elm_cnt){
		uint8_t header[3];	
		CBufRead(&drv->_receive_buf, header, 3);
		CBufRead(&drv->_receive_buf, message_arena, header[1]);
		#if AVR_I2C_DRV_DEBUG
			printf("I2C Call handler for id: %d\r\n", header[2]);		
		#endif
		i2c_drv_data_received_cmplt(drv, header[0], header[1], header[2], message_arena);
		drv->_in_queue_elm_cnt--; // Only one message processed at a time
	}
	if((drv->_state == STATE_READY) && (!(TWCR & (1<<TWSTO)))){ // Ready to next operation
		if(drv->_op_queue_elm_cnt){
			drv->_state = STATE_BUSY;
			drv->_op_queue_elm_size = CBufReadByteNoRes(&drv->_transmit_buf); // Get data count
			#if AVR_I2C_DRV_DEBUG
				printf("I2C Queue element size: %d bytes started\r\n", drv->_op_queue_elm_size);
			#endif
			TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA); // Start
		}
	}
}

void i2c_drv_irq_handler(i2c_drv_st* drv)
{
	uint8_t state = TWSR & TWSR_MASK;
	switch (state){
		case TWI_START:
		case TWI_REP_START: { // After start we need to send device addr			
			TWDR = CBufReadByteNoRes(&drv->_transmit_buf);
			drv->_prev_addr = TWDR & 0xFE;
			#if AVR_I2C_DRV_DEBUG
				if(TWDR & 0x01){
					printf("I2C IRQ Start READ to 0x%X...\r\n",  TWDR);
				} else {
					printf("I2C IRQ Start WRITE to 0x%X...\r\n", TWDR);
				}
			#endif
			TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT);
			return;
		}
		
		case TWI_MTX_ADR_ACK:   // Previous transmitted addr - is transmit addr, so we need trasmit data
			#if AVR_I2C_DRV_DEBUG
			printf("I2C IRQ TX addr acknowledged SUCCESS\r\n");
			#endif 
			// Fall -->
		case TWI_MTX_DATA_ACK:  // Previous trasmited data is ACK by device, trasmit next data if need or stop
			if(drv->_op_queue_elm_size){
				TWDR = CBufReadByteNoRes(&drv->_transmit_buf);
				TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT);
				drv->_op_queue_elm_size--;
				drv->_state = STATE_TRANSMIT;
			} else { // Start new transaction or stop if no data in  TX buffer
				#if AVR_I2C_DRV_DEBUG
					printf("I2C IRQ TX SUCCESS\r\n");
				#endif
				if(go_to_next_packet(drv)){// Have full packet in TX buf
					drv->_state = STATE_BUSY;
					#if AVR_I2C_DRV_DEBUG
						printf("I2C IRQ Continue with same address\r\n");
					#endif
					TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA); // Repeated start
					return;
				}
				TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWSTO)|(0<<TWIE);
				drv->_state = STATE_READY;
			}
			return;
		
		case TWI_MRX_ADR_ACK:   
			CBufWriteByte(&drv->_receive_buf, drv->_prev_addr); //Add addr device, from which the data was obtained
			CBufWriteByte(&drv->_receive_buf, drv->_op_queue_elm_size); // Add pkt size into RX buf
			uint8_t op_uuid = CBufReadByteNoRes(&drv->_transmit_buf);
			CBufWriteByte(&drv->_receive_buf, op_uuid); // Read transaction UUID
			drv->_state = STATE_RECEIVE;
			#if AVR_I2C_DRV_DEBUG
				printf("I2C IRQ RX addr acknowledged SUCCESS (UUID:%d)\r\n", op_uuid);
			#endif
			// Fall -->				
		case TWI_MRX_DATA_ACK:	
			if(state == TWI_MRX_DATA_ACK){
				drv->_op_queue_elm_size--;
				CBufWriteByte(&drv->_receive_buf, TWDR);
			}
			if(drv->_op_queue_elm_size - 1){// If last byte then start receive and send NACK after
				TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA); // Continue... Send ACK
			} else {
				TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA); // Continue... Send NACK (Last byte)
			}
			return;
								
		case TWI_MRX_DATA_NACK: 
			CBufWriteByte(&drv->_receive_buf, TWDR); // Save last received byte
			drv->_in_queue_elm_cnt++;
			#if AVR_I2C_DRV_DEBUG
				printf("I2C IRQ RX SUCCESS\r\n");
			#endif
			if(go_to_next_packet(drv)){// Have full packet in TX buf
				#if AVR_I2C_DRV_DEBUG
					printf("I2C IRQ Continue with same address\r\n");
				#endif
				drv->_state = STATE_BUSY;
				TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA); // Repeated start
				return;
			}
			TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWSTO)|(0<<TWIE);
			drv->_state = STATE_READY;
			return;
		
		case TWI_ARB_LOST:
			TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA);
			#if AVR_I2C_DRV_DEBUG
				printf("I2C IRQ Arbiter lost\r\n");
			#endif
			return;
		
		case TWI_MTX_ADR_NACK:// Target not response ACK on his addr
			CBufMarkAsRead(&drv->_transmit_buf, drv->_op_queue_elm_size);
			#if AVR_I2C_DRV_DEBUG
				printf("I2C IRQ ERR tx address not acknowledged\r\n");
			#endif
			i2c_drv_target_not_recognize_clbk(drv, TWDR); 
			break; //--> To error handler
			
		case TWI_MRX_ADR_NACK: 
			CBufMarkAsRead(&drv->_transmit_buf, 1); // Read UUID from transmit buf
			#if AVR_I2C_DRV_DEBUG
				printf("I2C IRQ ERR rx address not acknowledged\r\n");
			#endif
			break;//--> To error handler
			
		case TWI_MTX_DATA_NACK: 
			i2c_drv_target_data_nack(drv, drv->_op_queue_elm_size); 
			CBufMarkAsRead(&drv->_transmit_buf, drv->_op_queue_elm_size); // Mark remaining bytes as read
			#if AVR_I2C_DRV_DEBUG
				printf("I2C IRQ ERR data not acknowledged\r\n");
			#endif
			break;//--> To error handler
		
		case TWI_BUS_ERROR:
		default:
			i2c_drv_bus_error(drv);
			CBufClear(&drv->_transmit_buf);
			CBufClear(&drv->_receive_buf);
			drv->_op_queue_elm_size = 0;
			drv->_op_queue_elm_cnt = 0;
			drv->_prev_addr = 0;
			drv->_state = STATE_READY;
			TWCR = (1<<TWINT); // Clear irq flag and turn off TWI
			#if AVR_I2C_DRV_DEBUG
				printf("I2C IRQ FATAL ERROR (Bus error)\r\n");
			#endif
			return;
	}
	/* Error handle */
	if(go_to_next_packet(drv)){// If have another operations 
		drv->_state = STATE_BUSY;
		TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA); // Repeated start, if possible
		return;
	}
	TWCR = (1<<TWINT); // Clear irq flag
	drv->_state = STATE_READY;
}

__attribute__((weak)) void i2c_drv_target_not_recognize_clbk(i2c_drv_st* drv, i2c_addr_t addr){		
}
__attribute__((weak)) void i2c_drv_target_data_nack(i2c_drv_st* drv, uint16_t dat_lef){	
}
__attribute__((weak)) void i2c_drv_bus_error(i2c_drv_st* drv){	
}
__attribute__((weak)) void i2c_drv_data_received_cmplt(i2c_drv_st* drv, uint8_t addr, uint8_t size, uint8_t uuid, uint8_t *data){
}

