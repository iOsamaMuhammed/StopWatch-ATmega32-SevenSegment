
/*
 * Stop watch Project Description:
 *
 * This code implements a stopwatch using an ATmega32 micro-controller.
 * Timer1 is configured with a pre-scaler of 64,
 * Three external interrupts (INT0, INT1, and INT2) are used for stopwatch controls.
 * The elapsed time is displayed on a multiplexed seven-segment display in the format of hours:minutes:seconds.
 *
 * Features:
 * - Timer1 with Pre-scaler 64 for accurate timing measurements.
 * - External interrupts for stop watch controls (INT0: Reset, INT1: Stop, INT2: Resume).
 * - Multiplexed seven-segment display to show hours, minutes, and seconds.
 * Author: Osama M.Shehata
 * Date: 30 May 2023
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned int time =0;

unsigned char sec1 =0;
unsigned char sec2 =0;

unsigned char min1 =0;
unsigned char min2 =0;

unsigned char hour1 =0;
unsigned char hour2 =0;

#define ENABLEBIT(Bit) PORTA= 0x00 |(1<<Bit)
#define DATADISP(data) (PORTC = (PORTC & 0xF0) |((data)&0x0F))
#define TIMER1_COMPARE_VLAUE 15625

/* Description:
 * For System Clock = 1Mhz and pre-scaler F_CPU/64.
 * Timer frequency will be around 15.625Khz, Timer = 64us
 * For compare value equals to 15,625 the timer will generate compare match interrupt every 1 second.
 * Compare interrupt will be generated every 1 second, so we need 1 compare interrupts to count 1 second.
 */

/***************** Interrupts Service Routines *****************/
ISR(INT0_vect)
{
	/********************* INT0 Service Routine *********************
	 * Function : If a Falling edge detected the Stop Watch time should be reset
	 * Method: Reset the Timer1 counter value TCNT1 = 0
	 */
	TCNT1 = 0; 	//Reset counter value
	time = 0; 	//Reset time

}
ISR(INT1_vect)
{
	/********************* INT1 Service Routine *********************
	 * Function : If a raising edge detected the Stop Watch time should be paused.
	 * Method: CS12-CS10=0 , No clock source (Timer/Counter stopped).
	 */
	TCCR1B &= ~(1 << CS12);
	TCCR1B &= ~(1 << CS11);
	TCCR1B &= ~(1 << CS10);
}

ISR(INT2_vect)
{
	/********************* INT2 Service Routine *********************
	 * Function : If a falling edge detected the Stop Watch time should be resumed.
	 * Method: Set pre-scalar value to be 64 (Resume clock)
	 */
	TCCR1B = (1 << CS11) | (1 << CS10) | (1 << WGM12);
}

ISR(TIMER1_COMPA_vect)
{
	/********************* Timer1 Service Routine *********************
	 * Function : Count time in seconds.
	 * Method: Increment time variable every compare match
	 */
	time++;

}

void INT0_Init(void)
{
	/********************* INT0 Configuration *********************
	 * 1. Configure External Interrupt INT0 with falling edge ---> Reset Stop Watch PD2
	 * 2. Enable external interrupt pin INT0
	 * 3. Enable internal pull-up resistor for INT0
	 */
	DDRD &= ~(1 << PD2);              	// Configure INT0/PD2 as input pin
	PORTD |= (1 << INT0);				// Enable internal pull-up resistor for INT0
	MCUCR |= (1 << ISC01);   			// Trigger INT0 with the falling edge
	GICR |= (1 << INT0);                // Enable external interrupt pin INT0
}

void INT1_Init(void)
{
	/********************* INT1 Configuration *********************
	 * 1. Configure External Interrupt INT1 with rising edge
	 * 2. Connect a push button with the internal pull-down resistor
	 */
	DDRD &= ~(1 << INT1);               	// Configure INT1/PD3 as input pin
	MCUCR = (1 << ISC10) | (1 << ISC11);   	// Trigger INT0 with the rising edge
	GICR |= (1 << INT1);                 	// Enable external interrupt pin INT1
}
void INT2_Init(void)
{
	/********************* INT2 Configuration *********************
	 * 1. Configure External Interrupt INT2 with falling edge (ISC2 = 0), MCUCSR
	 * If ISC2 is written to zero, a falling edge on INT2 activates the interrupt.
	 * If ISC2 is written to one, a rising edge on INT2 activates the interrupt.
	 *
	 * 2. Connect a push button with the internal pull-up resistor on INT2 in PORTB
	 */
	DDRB &= ~(1 << INT2); 		// Configure INT2 on PORTB as input pin
	PORTB |= (1 << INT2); 		// Enable internal pull-up resistor for INT2
	GICR |= (1 << INT2);		// Enable external interrupt pin INT2
}
void Timer1_Init(void)
{
	/********************* Timer1 Configuration *********************
	 * 1. Set waveform generation mode to CTC (Clear timer on compare match) WGM12 =1
	 * 2. Set compare output to toggle OC01 on compare watch COM1A0 =1
	 * 3. Set the pre-scalar to 64  ( CS12 = 1 , CS11 = 0 , CS
	 * Used a pre-scalar of 64 instead of 1064 to make it more accurate
	 */
	TCCR1B = (1 << CS11) | (1 << CS10) | (1 << WGM12);

	OCR1A = TIMER1_COMPARE_VLAUE; 	//Set compare value = 1000
	TIMSK |= (1 << OCIE1A); 		// Enable Compare Match A Interrupt

}
void updateTime(void){
	/* This function calculates the value of time in hours : minutes : seconds */
	hour1 = (time / 3600) % 10;
	hour2 = (time / 3600) / 10;
	unsigned int remainingSeconds = (time % 3600);
	min2 = (remainingSeconds / 60) / 10;
	min1 = (remainingSeconds / 60) % 10;
	sec2 = (remainingSeconds % 60) / 10;
	sec1 = (remainingSeconds % 60) % 10;
}
int main(void)
{
	DDRC = 0x0F; 					// Configure the first four pins in PORTC as output pins connected  to 7447 decoder
	PORTC &= 0xF0;    				// Initialize the 7-segment display zero at the beginning.

	DDRA = 0x3F; 					// First 6-pins in PORTA as the enable/disable pins for the six 7-segments
	PORTA = 0x3F;				    // Enable all 7-segments at the beginning

	SREG |= (1 << 7);				// Enable global interrupts by setting I-bit

	//Call initialization function of Timers & interrupts
	Timer1_Init(); 					// Start Timer 1
	INT0_Init(); 					// Start Interrupt 0
	INT1_Init();					// Start Interrupt 1
	INT2_Init();					// Start Interrupt 2
	while (1) {
		updateTime();
		ENABLEBIT(PA0);				//  Enable the display for this digit
		DATADISP(sec1); 			//  Display Seconds Units
		_delay_us(2);
		ENABLEBIT(PA1);
		DATADISP(sec2); 			//  Display Seconds Tens
		_delay_us(2);
		ENABLEBIT(PA2);
		DATADISP(min1); 			//  Display Minutes units
		_delay_us(2);
		ENABLEBIT(PA3);
		DATADISP(min2); 			//  Display Minutes Tens
		_delay_us(2);
		ENABLEBIT(PA4);
		DATADISP(hour1);; 			//  Display Hours units
		_delay_us(2);
		ENABLEBIT(PA5);
		DATADISP(hour2); 			//  Display Hours Tens
		_delay_us(2);
	}
}
