/* Name: main.c
 * Project: blastAMP - Infrared Remote Learning Device/Blaster
 * Author: David Arvelo
 * Creation Date: 2011-02-26
 *
 * License: GNU GPL v2
 * This Revision: 2011-03-10
 */

/*

The pin configuration on the ATmega88PA should be as follows.
For a description of modes, or to change how modes work, check "MODES INFORMATION" in the defines below.

01: PC6 - RESET w/ 1.5k+ resistor			28: PC5  - LED negative lead (mode 2)
02: PD0 - RXD - UART receive				27: PC4  - LED negative lead (mode 1)
03: PD1 - TXD - UART transmit				26: PC3  - LED negative lead (mode 0)
04: PD2 - LED negative lead (reprogram LED)		25: PC2  - pushbutton switch positive lead (Reprogram Switch)
05: PD3 - LED negative lead (infrared LED)		24: PC1  - pushbutton switch positive lead (Action Switch)
06: PD4 - LED negative lead (mode 3)			23: PC0  - center tap of  5k+ potentiometer
07: VCC - 5v						22: GND
08: GND							21: AREF - 0.1uF capacitor
09: PB6 - 12 MHz crystal/22pf capacitor			20: AVCC - VCC
10: PB7 - 12 MHz crystal/22pf capacitor			19: PB5  - SCK  (ISP)
11: PD5 - LED negative lead (mode 4)			18: PB4  - MISO (ISP)
12: PD6 - LED negative lead (mode 5)			17: PB3  - MOSI (ISP)
13: PD7 - LED negative lead (mode 6)			16: PB2  - NC (not connected)
14: PB0 - IR demodulator output pin			15: PB1  - NC

*/

#include <avr/io.h>
#include <stdio.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/boot.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/pgmspace.h> 


#include "uart.h"
#include "defines.h"

#ifndef uint
#define uint            unsigned int
#endif

#ifndef uchar
#define uchar           unsigned char
#endif

#define getstata        1
#define getcount        2
#define getirdata       3
#define next_run        4

#define IR_ReadPort	PORTB
#define IR_PortDDR      DDRB

//**  Stuff setup on PORTD
#define IO_Port_Setup	DDRD
#define IO_Port		PORTD
#define IO_Pin		PIND

#define reprogramLED	_BV(PD2)
#define IRLED		_BV(PD3)

//** Stuff setup on PORTC
#define Input_Port_Setup	DDRC
#define Input_Port		PORTC
#define Input_Pin		PINC


// MODES INFORMATION
// -- comp2piano and piano2comp are macros that utilize Function and volUp/volDown
// -- the rest are codes learned verbatim from the remote control
// -- each mode is tied to an LED on the device and (except c2p, p2c) a specific block of flash memory and EEPROM
// -- modes are switched while running by the ADC value of a potentiometer
#define NUM_OF_MODES	7
#define Comp2Piano      0
#define Piano2Comp	1
#define Power		2
#define Function	3
#define volUp		4
#define volDown		5
#define Special		6

#define SpecialPressed			bit_is_clear(Input_Pin, PC1)
#define ReprogramPressed		bit_is_clear(Input_Pin, PC2)

#define DEBOUNCE_TIME  50  // amount of time for button switch signal to stabilize

//==== GLOBAL VARIABLES ========================== 

// these EEMEM values represent word size (two bytes per count) on value [2] because they represent ints.
// Values [0] and [1] are, combined, an address location to search in flash
uint8_t EEMEM volUpAddress[3] = { 0x16, 0x00, 32 };
uint8_t EEMEM volDownAddress[3] = { 0x16, 0x80, 32 };
uint8_t EEMEM FunctionAddress[3] = { 0x17, 0x00, 32 };
uint8_t EEMEM SpecialAddress[3] = { 0x17, 0x80, 32 };
uint8_t EEMEM PowerAddress[3] = { 0x18, 0x00, 32 };
uchar timer0_count = 0;
uchar timer2_count = 0;
uint  irdata[128];
uchar irdataindex;
uchar readindex;
uchar timer1Strike = 0;
uchar mode = 255;  // bogus value to force mode check and correct LED lighting on start

//==== FUNCTION PROTOTYPES =========================
void infiniteLight(void);
void infiniteLight2(void);

void boot_program_page (uint32_t page, uint8_t *buf) BOOTLOADER_SECTION;

void timer0_capture_start(void);
void timer0_pulse_start(void);
void timer1_capture_start(void);
void timer1_pulse_init(void);
void timer1_pulse_start(void);
void timer2_pulse_init(void);
void timer2_pulse_start(void);
void checkMode(void);
unsigned char checkButtonPress(int callingFunction); 
void pulse_IR_LED(unsigned char buttonCode, int numPulses);
void pulseComp2Piano(void);
void pulsePiano2Comp(void);
void reprogramButton(void);
void capture_IR_signal(void);
void read_IR_signal_data(unsigned char buttonCode);
void store_IR_signal_data(unsigned char buttonCode);



/*------------------- FOR 12MHz CLOCK -------------------*/
ISR(TIMER0_OVF_vect)
{
    TCNT0 = 0x50;       //set ~15ms  (@ 12MHz clock, 175 timing events at 1024 prescaler => start at 80 (0x50) to count to 255  (255-80 = 175))
	timer0_count++;
	if (timer0_count >= 3)
		TCCR0B = 0x00;  // stop timer
}

ISR(TIMER1_COMPA_vect)
{
	
	timer1Strike = 1;  // set flag for pulse_IR_LED function
	TCNT1 = OCR1A;//irdata[readindex]; OCR1A will do as long as TIMER2_COMPA_vect interrupt isn't enabled... dunno why.

}

// timer 2 compares at 160kHz to start. the first cycle OC2B will be high.
// at next compare, set timer for three elapsed 160kHz cycles with OC2B low
// this simulates a 40kHz wave with 1/4 duty cycle
ISR(TIMER2_COMPB_vect)
{
	if (OCR2B == 0x4A) {  // if this compare occurred after first 160kHz cycle (OC2B high)
		OCR2B = 0xE0;	  //	 set timer for three elapsed 160kHz cycles with OC2B low
	} else {			  // else, three cycles have already elapsed (OC2B low)
		OCR2B = 0x4A;	  //	 set for one 160kHz cycle with OC2B high
	}
	TCNT2 = 0x00;		  // reset counter


}

/*-------------------------------------------------------*/



/* ------------------- FOR 4MHz CLOCK----------------------
ISR(TIMER0_OVF_vect)
{
    TCNT0 = 0x16;                    //set 15ms
	timer0_count++;
	if (timer0_count >= 3)
		TCCR0B = 0x00;  // stop timer
}

ISR(TIMER1_COMPA_vect)
{
	TCNT1 = irdata[readindex];// OCR1A;

	timer1Strike = 1;
}

ISR(TIMER2_COMPB_vect)
{
	if (OCR2B == 0x18) {
		OCR2B = 0x48;
	} else {
		OCR2B = 0x18;
	}
	TCNT2 = 0x00;
}

/*------------------------------------------------------*/

// I think I had originally set compare A interrupt on.  I've since turned it off, but this is here for compatibility.
ISR(TIMER2_COMPA_vect)
{
	printf("AHHHHHHHHHHHHH!!!!!!!!!!!!");  // this shouldn't happen, but if it does and no ISR is declared... CRASH!
}



// these two functions both test for unwanted infinite loops.
// infiniteLight2 displays a specified 8-bit binary value on an LED for debugging in case no UART is present

void infiniteLight(void)
{		
	while(1){
		_delay_ms(1000);
		IO_Port &= ~(reprogramLED);  // turn LED on
		_delay_ms(1000);
		IO_Port |= reprogramLED;  // turn LED off
	}
}

void infiniteLight2(void)
{		
	int i;
//	while(1){
		_delay_ms(2000);
		IO_Port &= ~(reprogramLED);  // turn LED on
		_delay_ms(5000);
		IO_Port |= reprogramLED;  // turn LED off

		_delay_ms(5000);
		for (;;) {
			for (i=0; i<8; i++)  // for all 8 bits
			{
				IO_Port |= reprogramLED;
				_delay_ms(200);
				if ((irdataindex >> i) & 0x01) // if bit is 1, shine light twice quickly
				{
					IO_Port &= ~(reprogramLED);
					_delay_ms(200);
					IO_Port |= reprogramLED;
					_delay_ms(200);
					IO_Port &= ~(reprogramLED);
					_delay_ms(200);
					IO_Port |= reprogramLED;
					_delay_ms(200);
				}else{						   // if bit is 0, shine light once slowly
					IO_Port &= ~(reprogramLED);
					_delay_ms(1000);
					IO_Port |= reprogramLED;
					_delay_ms(1000);
				}

				_delay_ms(1000);			  // delay before next bit
			}

			_delay_ms(5000);  // delay a lot before looping function over again
		}

//	}
}

/************************************************

			Bootloader Reprogramming Section

	Bootloader section is set at 0xF00 (word addressing)
	If you add more functions to the bootloader make sure
	it doesn't overflow in memory past microcontroller limits.
	Make sure all other programmed code stays before this address
	by checking the amount of data to be flashed in simulator.

****************************************************/


void boot_program_page (uint32_t page, uint8_t *buf)
{
        uint16_t i;
        uint8_t sreg;

        // Disable interrupts.

        sreg = SREG;
        cli();
    
        eeprom_busy_wait ();

        boot_page_erase (page);
        boot_spm_busy_wait ();      // Wait until the memory is erased.

        for (i=0; i<SPM_PAGESIZE; i+=2)
        {
            // Set up little-endian word.

            uint16_t w = *buf++;
            w += (*buf++) << 8;
        
            boot_page_fill (page + i, w);
        }

        boot_page_write (page);     // Store buffer in flash page.
        boot_spm_busy_wait();       // Wait until the memory is written.

        // Reenable RWW-section again. We need this if we want to jump back
        // to the application after bootloading.

        boot_rww_enable ();

        // Re-enable interrupts (if they were ever enabled).

        SREG = sreg;

} //endcode 


/* -------------------FOR 12MHz CLOCK---------------------------------- */

void timer0_capture_start(void)
{
    TCCR0B = 0x00;                    //stop counter
    TCNT0 = 0x50;                    //set ~15ms  (@ 12MHz clock, 175 timing events at 1024 prescaler => start at 80 (0x50) to count to 255  (255-80 = 175))
    TIMSK0 = 0x00; //TIMSK0 & (~(1 << TOIE0)); <= original AVRUSB_IR code //no interrupt
    TIFR0  = TIFR0 | (1 << TOV0);      //clear int flag
    TCCR0B = ((1<<CS02) | (1<<CS00));  //start timer with 1024 prescaler (@ 12MHz clock)
}

void timer0_pulse_start(void)
{
    TCCR0B = 0x00;                    //stop counter
    TCNT0 = 0x50;                    //set ~15ms  (@ 12MHz clock, 175 timing events at 1024 prescaler => start at 80 (0x50) to count to 255  (255-80 = 175))
    TIFR0  = TIFR0 | (1 << TOV0);      //clear int flag
    TIMSK0 = (1 << TOIE0); // overflow interrupt in effect
    TCCR0B = ((1<<CS02) | (1<<CS00));                   //start timer with 1024 prescaler (@ 12MHz clock)
}

void timer1_capture_start(void)
{
//  TCCR1A = 0x00;
    TCCR1B = 0x00;                                      //stop timer
    TIMSK1  = 0x00; //TIMSK1 & (~((1 << TOIE1) | ~(1 << OCIE1A) | (1 << ICIE1))); <= original AVRUSB_IR code // Timer1 interrupts disable
 
	// my code  TIFR1  = ((1<<ICF1) | (1<<OCF1A) | (1 << TOV1));   //clear T1 overflow flag/input capture flag, clear possible mess from pulse
	TIFR1 = TIFR1 | (1 << OCF1A);

	//original code
	TIFR1  = TIFR1 | (1 << TOV1);                        //clear T1 overflow flag
    TIFR1  = TIFR1 | (1 << ICF1);                        //clear T1 input capture flag

    TCNT1 = 0x0000;                                      //starts from 0, 666nS per count @ 12MHz clock

    TCCR1B = 0x80;                                      // noise surpress mode, first capture negative edge (ICES1 bit)
	TCCR1B |= ((1<<CS11));	// start timer with prescaler of 8 (666nS per count @ 12MHz clock)
}

void timer1_pulse_init(void)
{
	TCCR1A = 0x00; // clear COM and WGM settings
	TCCR1B = (1<<WGM12);  // set CTC mode with WGM bits from A and B
	TIMSK1 = (1<<OCIE1A);  // enable output compare interrupt on timer A
	TIFR1 = ((1<<ICF1) | (1<<OCF1A) | (1<<TOV1)); // clean up any mess possibly left by capture operation, reset output compare flag

	OCR1B = 0xFFFF;  // this is to prevent a possible match on side B messing up the timer
	TCNT1 = 0x0000;  // set counter to 0
}

void timer1_pulse_start(void)
{
	TCCR1B |= ((1<<CS11));	// start timer with prescaler of 8 (666nS per count @ 12MHz clock)
}
//==================================================
//

// timer 2 compares at 160kHz to start. the first cycle OC2B will be high.
// at next compare, set timer for three elapsed 160kHz cycles with OC2B low
// this simulates a 40kHz wave with 1/4 duty cycle
void timer2_pulse_init(void)
{
	TCCR2A = ((1<<COM2B1) | (1<<WGM21)) ; // set CTC mode, OC2B to output low at start (forced compare match below)
	TCCR2B = (1<<FOC2B);  // force compare match to set set pin OC2B low, also turn off clock
	TIMSK2 = (1<<OCIE2B); // enable output compare interrupt on timer side B to handle 1/4 duty cycle of 40kHz wave

	OCR2B = 0x4A;  //  first 160kHz cycle @ 12MHZ, 0x4B signifies 1/4 mark
	OCR2A = 0xFF;  // this is to prevent a possible match on side A messing up the timer
	TCNT2 = 0x00;  // set counter to 0

	TCCR2A = ((1<<COM2B0) | (1<<WGM21)); // set CTC mode, OC2B to toggle on compare match
}

//==================================================
//
void timer2_pulse_start(void)
{	
	TCCR2B = ((1<<FOC2B) | (1<<CS20));  // force compare to bring pin high, start clock no prescaler
}

/*-----------------------END OF FOR 12MHz CLOCK---------------------------------*/




/*--------------------------------FOR 4MHz CLOCK----------------------------------------

void timer0_capture_start(void)
{
    TCCR0B = 0x00;                    //stop counter
    TCNT0 = 0x16;                    //set ~15ms  (@ 4MHz clock, 234 timing events = 234-1 total time clocked, so start at 22->255)
    TIMSK0 = 0x00; //TIMSK0 & (~(1 << TOIE0)); //no interrupt
    TIFR0  = TIFR0 | (1 << TOV0);      //clear int flag
    TCCR0B = (1<<CS02);                    //start timer with 256 prescaler (@ 4MHz clock)
}

void timer0_pulse_start(void)
{
    TCCR0B = 0x00;                    //stop counter
    TCNT0 = 0x16;                    //set ~15ms  (@ 4MHz clock, 234 timing events = 234-1 total time clocked, so start at 22->255)
    TIFR0  = TIFR0 | (1 << TOV0);      //clear int flag
    TIMSK0 = (1 << TOIE0); // overflow interrupt in effect
    TCCR0B = (1<<CS02);                   //start timer with 256 prescaler (@ 4MHz clock)
}

void timer1_pulse_init(void) 
{ 
   TCCR1A = 0x00; // clear COM and WGM settings 
   TCCR1B = (1<<WGM12);  // set CTC mode with WGM bits from A and B 
   TIMSK1 = (1<<OCIE1A);  // enable output compare interrupt on timer A 
   TIFR1 = ((1<<ICF1) | (1<<OCF1A) | (1<<TOV1)); // clean up any mess possibly left by capture operation, reset output compare flag 

   OCR1B = 0xFFFF;  // this is to prevent a possible match on side B messing up the timer 
   TCNT1 = 0x0000; // reset counter 
} 

void timer1_pulse_start(void) 
{    
   TCCR1B |= ((1<<CS11));  // start timer with prescaler of 8 (2uS per count on 4MHz system clock) 
} 

//================================================== 

void timer2_pulse_init(void) 
{ 
   OCR2B = 0x18;    // set timer to 80kHz (@ 4MHz clock) which is twice the 40kHz signal we need to pulse 40kHz waves 
   OCR2A = 0xFF;   // this is to prevent a possible match on side A messing up the timer 
   TCNT2 = 0x00;  // reset counter 

   TCCR2A = ((1<<COM2B1) | (1<<WGM21)) ; // set CTC mode, pin OC2B to output low at start (forced compare match below) 
   TIMSK2 = (1<<OCIE2B);  // enable output compare interrupt on timer B 

   TCCR2B = (1<<FOC2B);  // force compare match to set pin OC2B low, turn off clock 

   TCCR2A = ((1<<COM2B0) | (1<<WGM21)); // set CTC mode, OC2B to toggle on compare match 
} 

void timer2_pulse_start(void) 
{    
   TCCR2B = ((1<<FOC2B) | (1<<CS20));  // force compare to bring pin high to start 
}
/*------------------------------END OF FOR 4MHz CLOCK-----------------------------------------------------*/


// checks potentiometer with ADC to change function modes when user turns the knob
void checkMode(void) 
{
//	ADCSRA |= (1 << ADSC);  // start first conversion.  not needed if ADATE is set (free running mode)
//	while (bit_is_set(ADCSRA, ADSC));  // wait for conversion complete; also not needed if ADATE is set

	uchar temp = ADCH/((256/NUM_OF_MODES)+1);  // read ADC value from potentiometer 0-255
	
	if (mode == temp) return;    // 256 divided by 7 modes = 36.57 ~ 37.  each mode has approx 37 range on ADC (last one has 34).

	mode = temp;

	IO_PORT |= 0xF8; // disable all LEDs on PORT D
	Input_Port |= 0x38;  // disable all LEDs on PORT C

	if (mode == 0) {
		Input_Port &= ~(_BV(PC3)); // PC3 LED turn on
	} else if (mode == 1) {
		Input_Port &= ~(_BV(PC4)); // PC4 LED turn on
	} else if (mode == 2) {
		Input_Port &= ~(_BV(PC5)); // PC5 LED turn on
	} else if (mode == 3) {
		IO_Port &= ~(_BV(PD4)); // PC4 LED turn on
	} else if (mode == 4) {
		IO_Port &= ~(_BV(PD5)); // PD5 LED turn on
	} else if (mode == 5) {
		IO_Port &= ~(_BV(PD6)); // PD6 LED turn on
	} else if (mode == 6) {
		IO_Port &= ~(_BV(PD7)); // PD7 LED turn on
	}	
}

int main(void)
{ 	

	timer2_pulse_init(); // must occur before DDRD setup, sets up OC2B

	uart_init();
	FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
	// I think this is only necessary for an error output
	FILE lcd_str = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
    stderr = &lcd_str;
	stdout = stdin = &uart_str;


	printf("CLKPR value: %d", CLKPR);

		ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0) | (1<< ADATE);   // auto-trigger, 64 prescaler (datasheet specifies need between 50 kHz and 200 kHz). 
		ADCSRB = 0;  // Free Running mode
		ADMUX |= (1<< ADLAR) | (1 << REFS0);   // set AVcc as reference voltage, ADC0 is automatically multiplexed
											// highest 8 bits in ADCH register (ADLAR -left adjusted bit set)
		ADCSRA |= (1 << ADEN);  // enable ADC hardware



	Input_Port_Setup = 0x38; // set up all (3) LEDs, (2) button switches, and (1) ADC on PC0
	Input_Port |= 0x07;  // activate all pull-ups on button switches

	IO_Port_Setup = 0xFF; // Set all LEDs to output
	IO_Port |= 0xFF; // set all LEDs to off

    IR_ReadPort |= 0xff; /* activate all pull-ups on IR Read port */
    IR_PortDDR = 0x02;    /* all pins except OC1A are input */
 
	readindex   = 0;
    irdataindex = 0;
      
	// not sure if these are necessary but they were in the initial capture code from another project
    TIFR0        = TIFR0 | (1 << TOV0); //clear interrupt flag
    TIFR1        = TIFR1 | (1 << TOV1); //clear T1 overflow flag�
    TIFR1        = TIFR1 | (1 << ICF1); //clear input capture flag
	

	ADCSRA |= (1 << ADSC);  // start first conversion

	sei();   // enable global interrupts

	char val;
    for(;;)
	{
		checkMode();
		printf("Mode: %u, ADC value: %u\r\n", mode, ADCH);
		val = checkButtonPress(0);
//		printf("made it out\r\n");
    } // end infinite for loop

} // end main

unsigned char checkButtonPress(int callingFunction) 
{

	if (callingFunction == 0)
	{
		if (SpecialPressed) {
			printf("yep i'm in...");
			_delay_ms(DEBOUNCE_TIME);
			if (!SpecialPressed) return (0xFF);

			ADCSRA &= ~(1<<ADEN);  // disable free-running ADC if button press confirmed

			while(SpecialPressed);

			printf("specialpressed");

			if (mode > 1) {     		// if mode is a verbatim remote control signal
				pulse_IR_LED(mode, 10);
			} else if (mode == 0) {
				pulseComp2Piano();
			} else if (mode == 1) {
				pulsePiano2Comp();
			}

			ADCSRA |= (1 << ADEN);  // enable ADC hardware
			ADCSRA |= (1 << ADSC);  // start first conversion
			return (1);	

		} else if (ReprogramPressed) {
			_delay_ms(DEBOUNCE_TIME);
			if (!ReprogramPressed) return (0xFF);
			while(ReprogramPressed);
			reprogramButton();
			return (2);		



		}else{
			return (0xFF);
		}

		return (0xFE);  // probably not needed, but catch just in case

	} else {

		if (SpecialPressed) {
			printf("yep i'm in...");
			_delay_ms(DEBOUNCE_TIME);
			if (!SpecialPressed) return (0xFF);

			while(SpecialPressed);

			printf("specialpressed");
			printf("Mode: %u, ADC value: %u\r\n", mode, ADCH);

			if (mode > 1) {      	// if mode is a verbatim remote control signal
				return mode;		// return signal code number
			} else {
				return (0xFF);
			}

		} else if (ReprogramPressed) {
			_delay_ms(DEBOUNCE_TIME);
			if (!ReprogramPressed) return (0xFF);
			while(ReprogramPressed);
			return (0xD0);		


		}else{
			return (0xFF);
		}

		return (0xFF);  // probably not needed, but catch just in case

	}
}

void pulse_IR_LED(unsigned char buttonCode, int numPulses)
{
	int i;

	read_IR_signal_data(buttonCode); // this stores the IR code into irdata array from flash.  irdataindex is also used (EEPROM)

	printf("IR Signal Read data.  %d:\r\n", irdataindex);
	for (i = 0; i < irdataindex; i++)
		printf("%u, ", irdata[i]);

	// pulse IR signal numPulses times and wait a total of 45ms between each full signal (including signal time)
	for (i=0; i<numPulses; i++)
	{	
		timer0_count = 0;
		readindex = 1;
		OCR1A = irdata[1];  // load second value into timer (since first value is zero, we can skip it because it's handled by timer2_pulse_start)

		timer1_pulse_init();  	// timer1 controls when OC2B signals IR data 0 or 1 by turning OC2B/timer2 on/off at interrupt
		timer2_pulse_init();    // initialize OC2B to low, set timer to zero
		timer2_pulse_start();	// set OC2B high for start of signal
		timer1_pulse_start(); 	// OC2B is high on start, at next timer1 interrupt (index = 1, not 0), OC2B goes low

		timer0_pulse_start();	// start ~15ms timer


//		printf("irdataindex = %d.  Start operation.\r\n", irdataindex);

		while(readindex < irdataindex) { //printf("Still running %d: %d, %d...", timer0_count, TCNT0, OCR1A);	// send all data pulses before continuing
			if (timer1Strike) {	  // timer1 interrupt flag
				timer1Strike = 0;
				
				if (readindex % 2 == 0)  // even index, set OC2B high.  odd, low.
				{
					timer2_pulse_start();
						
				} else {
					timer2_pulse_init();  //  was going to use TCCR2B = 0x00 but this forces a low OC2B pin which is what i want
				}

				readindex++;

				OCR1A = irdata[readindex];   // load next value

			}
			asm volatile("nop" ::);  // keep loop going (compiler might be "optimizing this loop away" since the while loop may appear to not do anything)

		}
		
		TCCR1B = 0x00;
		timer2_pulse_init();
//		printf("Send operation finished %d.\r\n", timer0_count);

		// timer0 is shut off by its interrupt after 3 interrupts go by
		while(timer0_count < 3) printf("x"); //printf("Still running %d, %d...", timer0_count, TCNT0); // after ~45ms (~15ms timer * 3), be ready to repeat total operation


//		_delay_ms(25);  // I'm using this instead of timer0 for now ---- need to test timer0 instead  by enabling line above

//		printf("Send operation finished2.\r\n");
	}

}

void pulseComp2Piano(void)
{
	pulse_IR_LED(Function, 10);
	pulse_IR_LED(Function, 10);
	pulse_IR_LED(Function, 10);
	_delay_ms(30);
	pulse_IR_LED(volUp, 60);
}

void pulsePiano2Comp(void)
{
	pulse_IR_LED(Function, 10);
	pulse_IR_LED(Function, 10);
	pulse_IR_LED(Function, 10);
	pulse_IR_LED(Function, 10);
	_delay_ms(30);
	pulse_IR_LED(volDown, 55);
}

void reprogramButton(void)
{

	unsigned char buttonCode = 0xFF;

	while (buttonCode == 0xFF) {

		IO_Port &= ~(reprogramLED);  // turn LED on
		_delay_ms(100);
		IO_Port |= reprogramLED;  // turn LED off
		_delay_ms(400);
		
		checkMode();
		printf("Mode: %u, ADC value: %u\r\n", mode, ADCH);
		buttonCode = checkButtonPress(1);
		if (buttonCode == 0xD0) {     // cancel reprogram
			IO_Port |= reprogramLED;  // turn LED off
			return;
		}
	}
	
	ADCSRA &= ~(1<<ADEN);  // disable free-running ADC

	IO_Port &= ~(reprogramLED);  // turn on LED


	//work on reprogramming using button code
	capture_IR_signal();
	store_IR_signal_data(buttonCode);


	// before you leave, shut off the lights
	IO_Port |= reprogramLED;  // turn LED off

	ADCSRA |= (1 << ADEN);  // enable ADC hardware
	ADCSRA |= (1 << ADSC);  // start first conversion


}

void capture_IR_signal(void)
{
	int p;
    unsigned int itemp;
	irdataindex = 0;
//	printf("Capture now beginning.\r\n");

	timer1_capture_start();
	while (1)
	{
		if (TIFR1 & (1 << ICF1))// && (busy == 1))   //with IR, save counts and initialise timer to 0, change capture edge
        {

            if (irdataindex == 0) //first capture, T1 starts from 0
            {
               TCNT1H    = 0x00;
                TCNT1L    = 0x00; //clear to start from 0
                irdata[0] = 0;
                irdataindex++;

                TIFR1 = TIFR1 | (1 << ICF1); //clear capture INT flag

                timer0_capture_start();
            }
            else               //IR not first edge
            {
               itemp = ICR1L; //save IR timing
                itemp = itemp | (ICR1H << 8);
                irdata[irdataindex] = itemp;
                irdataindex++;

                TIFR1 = TIFR1 | (1 << ICF1); //clear input capture INT flag

                timer0_capture_start();

                if (irdataindex > 127)     // index over-ranged 
                {
                  TCCR0B  = 0x00;               //stop timer0
                  TIFR0   = TIFR0 | (1 << TOV0); //clear interrupt flag
                  TCCR1B = 0x00;               //stop timer
                  
/*				  if (irdataindex > 0) {
				  	printf("Index too big.  %d:\r\n", irdataindex);
				  	for (p = 0; p < irdataindex; p++)
						printf("%d, ", irdata[p]);
            	  }
*/                break;
				} 
            }
      
	        TCCR1B ^= (1<<ICES1);             // toggle capture edge 
	     	  
	    }
        if (TIFR0 & (1 << TOV0))          //If T0>15ms, then IR signal ended
        {
            TCCR0B  = 0x00;               //stop timer0
            TIFR0   = TIFR0 | (1 << TOV0); //clear interrupt flag
            TCCR1B = 0x00;               //stop timer

			if (irdataindex > 0) {
				printf("Timer overflow.  Signal ended.  %d:\r\n", irdataindex);
				for (p = 0; p < irdataindex; p++)
					printf("%u, ", irdata[p]);
           	}
            break;
        }
	}
}

void read_IR_signal_data(unsigned char buttonCode)
{	

	uint8_t i = 0;
	uint8_t CodeAddress[3];
	
	if (buttonCode == volUp) eeprom_read_block( (void *)&CodeAddress, (const void *)&volUpAddress, 3);
	else if (buttonCode == volDown) eeprom_read_block( (void *)&CodeAddress, (const void *)&volDownAddress, 3);
	else if (buttonCode == Function) eeprom_read_block( (void *)&CodeAddress, (const void *)&FunctionAddress, 3);
	else if (buttonCode == Special) eeprom_read_block( (void *)&CodeAddress, (const void *)&SpecialAddress, 3);
	else if (buttonCode == Power) eeprom_read_block( (void *)&CodeAddress, (const void *)&PowerAddress, 3);
	else return;

	irdataindex = CodeAddress[2];
	uint32_t startAddress = (CodeAddress[0] << 8) | CodeAddress[1];

	// irdata is an integer array, requires two bytes to fill
	for (i=0; i<CodeAddress[2]; i++)
		irdata[i] = ((pgm_read_byte(startAddress + 2*i) << 8) | (pgm_read_byte(startAddress + 2*i + 1)));

}

void store_IR_signal_data(unsigned char buttonCode)
{
	/*-----------------------------------------------------------------------*
	| Addresses of program codes as well as their 
	| size are stored in EEPROM with the format:	Byte 1: Address High Byte
	|												Byte 2: Address Low Byte
	|												Byte 3: Size of Code Data
	\*-----------------------------------------------------------------------*/


	unsigned int j, num_pages;

	uint8_t CodeAddress[3];
	
	if (buttonCode == volUp) eeprom_read_block( (void *)&CodeAddress, (const void *)&volUpAddress, 3);
	else if (buttonCode == volDown) eeprom_read_block( (void *)&CodeAddress, (const void *)&volDownAddress, 3);
	else if (buttonCode == Function) eeprom_read_block( (void *)&CodeAddress, (const void *)&FunctionAddress, 3);
	else if (buttonCode == Special) eeprom_read_block( (void *)&CodeAddress, (const void *)&SpecialAddress, 3);
	else if (buttonCode == Power) eeprom_read_block( (void *)&CodeAddress, (const void *)&PowerAddress, 3);
	else return;

	uint32_t firstPage = (CodeAddress[0] << 8) | CodeAddress[1];
	uint8_t pageBuf[SPM_PAGESIZE];

	CodeAddress[2] = (unsigned int)irdataindex;


	// STORE DATA IN FLASH!!
	readindex = 0; j = 0; num_pages = 0;
	while (readindex < irdataindex)
	{
		if (j < SPM_PAGESIZE)     // if buffer not full
		{

			pageBuf[j] = (irdata[readindex]) >> 8;  // store another byte in buffer
			pageBuf[j+1] = irdata[readindex];
			readindex++;
			j+=2;

			if (readindex >= irdataindex)
			{
				while (j < SPM_PAGESIZE)
				{
					pageBuf[j] = 0xFF;
					j++;
				}
				
				boot_program_page(firstPage, pageBuf);
				firstPage += SPM_PAGESIZE;
				num_pages++;
				j = 0;
				break;
			}

		} else {    // buffer is full. store page.
			boot_program_page(firstPage, pageBuf);
			firstPage += SPM_PAGESIZE;				// it seems that the first page gets overwritten with FFs in the sim
			num_pages++;
			j = 0;

	 		if (num_pages >= 4)  // only flash up to 4 pages
				break;
		}
	}

	//EEPROM write parameters
	if (buttonCode == volUp) eeprom_write_block( (const void *)&CodeAddress, (void *)&volUpAddress, 3);
	if (buttonCode == volDown) eeprom_write_block( (const void *)&CodeAddress, (void *)&volDownAddress, 3);
	if (buttonCode == Function) eeprom_write_block( (const void *)&CodeAddress, (void *)&FunctionAddress, 3);
	if (buttonCode == Special) eeprom_write_block( (const void *)&CodeAddress, (void *)&SpecialAddress, 3);
	if (buttonCode == Power) eeprom_write_block( (const void *)&CodeAddress, (void *)&PowerAddress, 3);
	else return;
	
		

}


