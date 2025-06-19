
#ifndef PRJ_COM_H_
#define PRJ_COM_H_

#define _GINT_DIS(buf)  do{ buf = (bool)(SREG & (1<<7)); asm("cli"); } while(0)
#define _GINT_EN(buf)   do{ if(buf) asm("sei"); } while(0)

#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
#include "application_config.h"

typedef enum prj_com_op_res_e
{
	OP_OK,
	OP_BUSY,
	OP_ERROR,
	OP_ERROR_NULL,
	OP_ERROR_NOT_INITED,
	OP_ERROR_INVALID_PARAMS,
	OP_ERROR_FORBIDEN,
	OP_NO_MEM,
	OP_IVALID_STATE
} prj_com_op_res_et;

typedef enum prj_com_state_e
{
	STATE_UNKNOWN,
	STATE_READY,
	STATE_TRANSMIT,
	STATE_RECEIVE,
	STATE_BUSY,
	STATE_ERROR,
} prj_com_state_et;

#define RETURN_ON_ERROR(call)\
do{\
	prj_com_op_res_et res = (call);\
	if(res != OP_OK){\
		return res;\
	}\
} while(0)

#ifdef REALIBLE_CODE
	#define RETURN_ON_ERROR_SENSETIVE(call)\
	do{\
		prj_com_op_res_et res = (call);\
		if(res != OP_OK){\
			return res;\
		}\
	} while(0);
#else
	#define RETURN_ON_ERROR_SENSETIVE(call) (call);
#endif
	
#endif
