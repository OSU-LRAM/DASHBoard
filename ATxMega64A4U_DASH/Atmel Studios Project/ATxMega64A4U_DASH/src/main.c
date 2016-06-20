//#define GYRO_SPI
#define USART_to_BT
#define PWM
#define QDEC
//#define PID
#define BANGBANG

#include <asf.h>
#include <avr/io.h>
#include <string.h>
#include <BTMessageParse.h>

#ifdef PID
#include <PID.h>
#endif /*PID*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* PID */	//Not currently in use.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef PID

#define K_P		0.53 //0.53 calculated to get output 100 at error 190  (output = k_p * error)
#define K_I		0.00
#define K_D		0.00

//TIME_INTERVAL = (desired interval[sec] )*( frequency[Hz]) / 255

#define TIME_INTERVAL 100

struct GLOBAL_FLAGS {
	uint8_t pidTimer:1;
	uint8_t dummy:7;
	}	gFlags = {0, 0};

	struct PID_DATA pidData;

#endif /*PID*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Bang-Bang */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef BANGBANG

//TIME_INTERVAL = (desired interval[sec] )*( frequency[Hz]) / 255
#define TIME_INTERVAL 100
#define POS_TOLERANCE 3	//192 total positions
#define POS_SPEED	50
#define leftCurrentPos TCC0_CNT
#define rightCurrentPos TCD0_CNT
bool bang = false;

#endif /*BANGBANG*/
	
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* USART_to_BT */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef USART_to_BT
#define USART_SERIAL                     &USARTD0
#define USART_SERIAL_BAUDRATE            921600
#define USART_SERIAL_CHAR_LENGTH         USART_CHSIZE_8BIT_gc
#define USART_SERIAL_PARITY              USART_PMODE_DISABLED_gc
#define USART_SERIAL_STOP_BIT            true

static usart_rs232_options_t USART_SERIAL_OPTIONS = {
	.baudrate = USART_SERIAL_BAUDRATE,
	.charlength = USART_SERIAL_CHAR_LENGTH,
	.paritytype = USART_SERIAL_PARITY,
	.stopbits = USART_SERIAL_STOP_BIT
};

#endif /* USART */

// PWM config struct
struct pwm_config mr_pwm_cfg;
struct pwm_config ml_pwm_cfg;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
												/* FUNCTION PROTOTYPES */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//button debounce
int8_t debounce_switch(uint8_t button);

//GYRO_SPI Init
void spi_init_module(void);

//send ACK over USART
void sendACK(void);

//init qdec
static void qdec_init(void);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
												/* MAIN */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main (void)
{
	sysclk_init();

	board_init();
	
	#ifdef QDEC
	qdec_init();
	#endif /* QDEC */
	
	#ifdef GYRO_SPI
	spi_init_module();
	#endif /* GYRO_SPI */
	
	#ifdef USART_to_BT
	sysclk_enable_module(SYSCLK_PORT_D, PR_USART0_bm);
	usart_init_rs232(USART_SERIAL, &USART_SERIAL_OPTIONS);
	buf_init();
	#endif /* USART */
	
	/* Control Timer */
	sysclk_enable_module(SYSCLK_PORT_D, PR_TC1_bm); //Enable TCD1

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Interrupt Setup */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* Programmable Interrupt Controller Level Enable */
	PMIC.CTRL = (PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm);
	
	/* UART RX Complete Interrupt */
	USARTD0.CTRLA |= (USART_RXCINTLVL0_bm | USART_RXCINTLVL1_bm);
	
	/* QDEC Timer Counter Overflow Interrupt */
	TCC0.INTCTRLA |= (TC0_OVFINTLVL0_bm | TC0_OVFINTLVL1_bm);
	
	/* QDEC Timer Counter Compare Interrupt */
	TCC0.INTCTRLB |= (TC0_CCBINTLVL0_bm | TC0_CCBINTLVL1_bm);

	/* Control Timer Enable */
	TCD1.CTRLA |= (TC1_CLKSEL1_bm); //no clock prescalar
	TCD1.INTCTRLA |= (TC1_OVFINTLVL0_bm | TC1_OVFINTLVL1_bm); //overflow interrupt level enable
	TCD1.PER = 0x00FF; //Change 16-bit timer to 8-bit by making the top value 0x00FF
	
	sei(); //Enable global interrupts
	
	#ifdef PID
	pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR, K_D * SCALING_FACTOR, &pidData);
	#endif /* PID */
	
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* PWM Init and start */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	#ifdef PWM
	//PWM_1 - RIGHT MOTOR
	pwm_init(&mr_pwm_cfg, PWM_TCE0, PWM_CH_B, 50000); //Port E Pin 1
	
	//PWM_2 - LEFT MOTOR
	pwm_init(&ml_pwm_cfg, PWM_TCE0, PWM_CH_D, 50000); //Port E Pin 3
	
	//When PWM is active, the state of PE0 and PE2 toggle left and right motor direction in the following manner
	//                       | Forward | Reverse |
	//                       ---------------------
	//  | LEFT MOTOR  -> PE0 |    1    |    0    |
	//                       ---------------------
	//  | RIGHT MOTOR -> PE2 |    0    |    1    |
	//                       ---------------------
	
	
	gpio_set_pin_high(IOPORT_CREATE_PIN(PORTE, 0)); //Sets motor direction forward
					
	
	mrDutyCycle = 0;
	mlDutyCycle = 0;
	mlDestination = 0;
	mrDestination = 0;
	sendPositionFlag = false;
	
	#endif /* PWM */
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	while(1){
		pwm_start(&mr_pwm_cfg, mrDutyCycle);
		pwm_start(&ml_pwm_cfg, mlDutyCycle);
		
		if(commands_in_buf > 0){
			parseCommand();
			commands_in_buf--;
		}
		
		if(sendPositionFlag){							//If postion request commmand was received
			usart_putchar(&USARTD0, leftCurrentPos);			//Send back position of left motor
			usart_putchar(&USARTD0, rightCurrentPos);			//Send back position of right motor
			sendPositionFlag = false;									
		}
		else if((CMDstate == Position)&&(bang)){		/* If we are going to position, and the control loop timer
															flag is set */
			
			/* LEFT MOTOR CONTROL LOOP */
			if(leftCurrentPos + POS_TOLERANCE < mlDestination){
				if((mlDestination - leftCurrentPos) > 96){
					ioport_set_pin_level(PE2,0);
				}
				else{
					ioport_set_pin_level(PE2,1);
				}
				mlDutyCycle = POS_SPEED;
				bang = false;
			}
			else if(leftCurrentPos > mlDestination + POS_TOLERANCE){
				if((leftCurrentPos - mlDestination) > 96){
					ioport_set_pin_level(PE2,1);
					}
					else{
					ioport_set_pin_level(PE2,0);
				}
				mlDutyCycle = POS_SPEED;
				bang = false;
			}
			else{
				mlDutyCycle = 0;
				bang = false;
			}
			/* LEFT MOTOR CONTROL LOOP */

			/* RIGHT MOTOR CONTROL LOOP */
			if(rightCurrentPos + POS_TOLERANCE < mrDestination){
				if((mrDestination - rightCurrentPos) > 96){
					ioport_set_pin_level(PE0,1);
					}
					else{
					ioport_set_pin_level(PE0,0);
				}
				mrDutyCycle = POS_SPEED;
				bang = false;
			}
			else if(rightCurrentPos > mrDestination + POS_TOLERANCE){
					if((rightCurrentPos - mrDestination) > 96){
						ioport_set_pin_level(PE0,0);
					}
					else{
						ioport_set_pin_level(PE0,1);
					}
				mrDutyCycle = POS_SPEED;
				bang = false;
				}
				else{
				mrDutyCycle = 0;
				bang = false;
				}
		}
	}
}
		

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
												/* Functions */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* QDEC */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void qdec_init(void)
{
	qdec_config_t QDEC_CH1;	//LEFT SIDE ENCODER
	qdec_config_t QDEC_CH2; //RIGHT SIDE ENCODER UNTESTED
	
	qdec_get_config_defaults(&QDEC_CH1);
	qdec_config_phase_pins(&QDEC_CH1, &PORTB, 0, false, 50);
	qdec_config_revolution(&QDEC_CH1, (12*16));	//12 counts per revolution of magnetic encoder wheel. 16:1 Gear ratio between encoder and legs.
	//qdec_config_event_channel(&QDEC_CH1, 0);
	//qdec_config_tc(&QDEC_CH1, &TCC0);
	
	//////////////////////////////////////////////
	//qdec_config_enable_freq(&QDEC_CH1,(3881/12));
	qdec_enabled(&QDEC_CH1);
	
	qdec_get_config_defaults(&QDEC_CH2);
	qdec_config_phase_pins(&QDEC_CH2, &PORTC, 0, false, 50);
	qdec_config_revolution(&QDEC_CH2, (12*16)); //12 counts per revolution of magnetic encoder wheel. 16:1 Gear ratio between encoder and legs.
	qdec_config_event_channel(&QDEC_CH2, 2);
	qdec_config_tc(&QDEC_CH2, &TCD0);
	
	//////////////////////////////////////////////
	//qdec_config_enable_freq(&QDEC_CH2,3881);
	qdec_enabled(&QDEC_CH2);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* GYRO_SPI */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef GYRO_SPI

void spi_init_module(void)
{
	struct spi_device device= {
		.id = IOPORT_CREATE_PIN(PORTC, 4)
		//.id = SPIC_SS //PC4 (J1 - SS)
	};

	spi_master_init(&SPIC);
	spi_master_setup_device(&SPIC, &device, SPI_MODE_3, 1000000, 0);
	spi_enable(&SPIC);
}

#endif /* GYRO */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* USART_to_BT */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef USART_to_BT

void sendACK(void)
{
	usart_putchar(&USARTD0, 'O');
	usart_putchar(&USARTD0, 'K');
	bytes_rx = 0;
}
#endif /* USART_to_BT */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* This function can be used to debounce the but*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int8_t debounce_switch(uint8_t button) {
	static uint16_t state = 0;
	state = (state <<1) | (! gpio_pin_is_low(button)) | 0xE000 ;
	if(state == 0xF000){
		return 1;
	}
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
												/* ISRs */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef USART_to_BT
ISR(USARTD0_RXC_vect){			//This ISR executes when the UART receives a byte
	uint8_t message = usart_getchar(&USARTD0);

	if(message == RESET){
		buf_clear();
		sendACK();
	}
	else if(message == ECHO){
		usart_putchar(&USARTD0, message);
	}
	else{
		buf_put(message);
		bytes_rx++;
		if(bytes_rx >= 4){
			commands_in_buf++;
			sendACK();
		}
	}
	
}
#endif /* USART_to_BT */

/* Control Interval Timer Interrupt */
ISR(TCD1_OVF_vect){			//I COMMENTED OUT THE ISR IN tc.c
	static uint16_t i = 0;
	if(i < TIME_INTERVAL){
		i++;
	}
	else{
		bang = true;
		//gFlags.pidTimer = TRUE; //Flag used for PID control
		i = 0;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////