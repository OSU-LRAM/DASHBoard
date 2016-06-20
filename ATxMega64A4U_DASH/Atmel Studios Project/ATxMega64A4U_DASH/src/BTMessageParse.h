/*
 * BTMessageParse.h
 *
 * Created: 3/5/2016 2:50:18 PM
 *  Author: Drew
 */ 


#ifndef BTMESSAGEPARSE_H_
#define BTMESSAGEPARSE_H_

/* Command Byte (First byte) */
#define CMD_TYPE_bm (0x80)
#define INFO_TYPE_bm (0x00)

#define ML_DIR_bm (0x40)
#define MR_DIR_bm (0x20)
#define FWD 1
#define REV 0

#define POS_GET (0x10)
#define ML_POS_bm (0x40)
#define MR_POS_bm (0x20)

//bits 0-4 are not currently used

#define MESSAGE_LEN 4

#define ECHO 0xFE	//Whenever this byte is received, it is immediately echoed back to the Bluetooth Module
#define RESET 0xFF	//Whenever this byte is received, the circular buffer is emptied and the DASH 

//Circular buffer
#define BUFSIZE 100
char buf[BUFSIZE];
char *pIn, *pOut, *pEnd;
char rxBufferFull;
uint8_t bytes_rx;
uint8_t commands_in_buf;

/* THIS IS BAD PRACTICE! THIS STUFF DOESN'T BELONG HERE */
/*These variables really shouldn't be declared here but I need access to them in parseCommand. The better practice would be to set flags
for the variables that we want to change. Then the variables that are declared below are only declared/modified in main.c 
A implementation similar to the sendPositionFlag should be used.	*/

enum states{
	Speed,
	Position
	} CMDstate;

uint8_t mrDutyCycle;
uint8_t mlDutyCycle; 

uint8_t mlDestination;
uint8_t mrDestination;

/* THIS IS BAD PRACTICE! THIS STUFF DOESN'T BELONG HERE */

bool sendPositionFlag;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* FUNCTION PROTOTYPES */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Circular buffer
void buf_init(void);
int buf_put(char c);
int buf_get(char *pc);
void buf_clear(void);

//Command functions
void parseCommand(void);


#endif /* BTMESSAGEPARSE_H_ */