/*
 * BTMessageParse.c
 *
 * Created: 3/5/2016 2:50:50 PM
 *  Author: Drew
 */ 



#include <stdint.h>
#include <ioport.h>
#include <user_board.h>
#include "BTMessageParse.h"
#include <string.h>

// init
void buf_init()
{
	pIn = pOut = buf;       // init to any slot in buffer
	pEnd = &buf[BUFSIZE];   // past last valid slot in buffer
	rxBufferFull = 0;               // buffer is empty
	bytes_rx = 0;
	commands_in_buf = 0;
}

// add char 'c' to buffer
int buf_put(char c)
{
	if (pIn == pOut  &&  rxBufferFull)
	return 0;				// buffer overrun

	*pIn++ = c;             // insert c into buffer
	if (pIn >= pEnd)        // end of circular buffer?
	pIn = buf;				// wrap around

	if (pIn == pOut)        // did we run into the output ptr?
	rxBufferFull = 1;       // can't add any more data into buffer
	return 1;               // all OK
}

// get a char from circular buffer
int buf_get(char *pc)
{
	if (pIn == pOut  &&  !rxBufferFull)
	return 0;				// buffer empty  FAIL

	*pc = *pOut++;          // pick up next char to be returned
	if (pOut >= pEnd)       // end of circular buffer?
	pOut = buf;				// wrap around

	rxBufferFull = 0;       // there is at least 1 slot
	return 1;               // *pc has the data to be returned
}

void buf_clear(void)
{
	memset(&buf[0], 0, BUFSIZE);
	pIn = pOut = buf;       // reset buffer pointers
	rxBufferFull = 0;       // set buffer full flag to 0
	bytes_rx = 0;			// Empty everything
	commands_in_buf = 0;	// Empty everything
	}

void parseCommand(void)
{
	char byte1;
	char byte2;
	char byte3;
	char byte4;
	
	buf_get(&byte1);
	buf_get(&byte2);
	buf_get(&byte3);
	buf_get(&byte4);
	
	if(byte1 & CMD_TYPE_bm){
		CMDstate = Speed;
		if(byte1 & ML_DIR_bm){
			ioport_set_pin_level(PE0,1);
		}//if
		else{
			ioport_set_pin_level(PE0,0);
		}//else
		mlDutyCycle = byte2;
		
		if(byte1 & MR_DIR_bm){
			ioport_set_pin_level(PE2,0);
		}//if
		else{
			ioport_set_pin_level(PE2,1);
		}//else
		mrDutyCycle = byte3;
		
	}//if
	else{
		if(byte1 & POS_GET){
			sendPositionFlag = true;	
		}//if
		else{
			if(byte1 & ML_POS_bm){
				CMDstate = Position;
				if(byte2 <= 192){
					mlDestination = byte2;
				}//if
			}//if
			if(byte1 & MR_POS_bm){
				CMDstate = Position;
				if(byte3 <= 192){
					mrDestination = byte3;
				}//if
			}//if
		}//else
	}//else
}//parseCommand