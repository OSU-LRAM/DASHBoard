/**
 * \file
 *
 * \brief User board initialization template
 *
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>

void board_init(void)
{
	/* This function is meant to contain board-specific initialization code
	 * for, e.g., the I/O pins. The initialization can rely on application-
	 * specific board configuration, found in conf_board.h.
	 */
	
	//BUTTON	TODO: Change GPIO_PUSH_BUTTON_0 to IOPORT_CREATE_PIN(port,pin)
	//ioport_configure_pin(GPIO_PUSH_BUTTON_0, IOPORT_DIR_INPUT
		//| IOPORT_LEVEL | IOPORT_PULL_UP);
	
	//USARTD0
	ioport_configure_pin(IOPORT_CREATE_PIN(PORTD, 3), IOPORT_DIR_OUTPUT
		| IOPORT_INIT_HIGH);
	ioport_configure_pin(IOPORT_CREATE_PIN(PORTD, 2), IOPORT_DIR_INPUT);
	
	//SPI
	//Set the pin used for slave select as output high
	ioport_configure_port_pin(&PORTC, PIN4_bm, IOPORT_INIT_HIGH | IOPORT_DIR_OUTPUT);
		
	//Set MOSI and SCK as output high, and set MISO as input
	ioport_configure_port_pin(&PORTC, PIN5_bm, IOPORT_INIT_HIGH | IOPORT_DIR_OUTPUT); //MOSI SDA
	ioport_configure_port_pin(&PORTC, PIN6_bm, IOPORT_DIR_INPUT); //MISO SCO
	ioport_configure_port_pin(&PORTC, PIN7_bm, IOPORT_INIT_HIGH | IOPORT_DIR_OUTPUT); //SCK
	
	ioport_configure_pin(IOPORT_CREATE_PIN(PORTE, 0), IOPORT_DIR_OUTPUT);
	ioport_configure_pin(IOPORT_CREATE_PIN(PORTE, 2), IOPORT_DIR_OUTPUT);
	
	
}
