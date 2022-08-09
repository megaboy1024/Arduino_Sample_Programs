#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#define ADC_SAMPLES			16
#define ICR_MAX				200UL
#define ICR_MIN				20UL
#define set_bit(Rn, bn)		Rn |= (1<<bn)
#define clear_bit(Rn, bn)	Rn &=~(1<<bn)
#define toggle_bit(Rn, bn)	Rn ^= (1<<bn)
#define ADC_Start()			set_bit(ADCSRA, ADSC)

/************************************************************************/
/* global variables                                                     */
/************************************************************************/
volatile uint16_t sa[ADC_SAMPLES];
volatile uint16_t sb;
volatile uint16_t* pa;
volatile uint16_t avga;
volatile uint16_t x, y, z;

volatile uint8_t count, update = 0;

void ADC_init(
	uint8_t ch,		//ADC channel(default 0)
	uint8_t psbits,	//ADC PS2..PS0 bits
	uint8_t csbits	//TC0 clock select bits
	)
{
	/***
	 * setup ADC
	 * single conversation mode
	 * AVCC with external capacitor at VREF
	 */
	ADMUX  = (0x40|(ch&0x07));
	ADCSRA = (0x80|(psbits&0x07));
	
	ADC_Start();					//perform first conversation
	while((ADCSRA&(1<<ADSC)))	;	//wait while conversation is in progress
	
	pa = &sa[0];	//set pointer
	
	/***
	 * setup TC0
	 * normal operation mode
	 * OC0x pins are disconnected
	 * overflow interrupt enabled
	 */
	TCCR0A = 0x00;
	TCCR0B = (csbits&0x07);
	set_bit(TIMSK0, TOIE0);
}

void PWM_init(
	uint8_t csbits	//TC1 clock select bits
	)
{
	/***
	 * setup TC0 
	 * phase and frequency correct mode
	 * non-inverting output
	 */
	
	set_bit(DDRB, PB1);	//activate output drive
	set_bit(DDRB, PB2);	//activate output drive
	
	//initial values
	ICR1  = 0x0080;
	OCR1A = 0x0020;
	OCR1B = 0x0040;
	
	TCCR1A = 0xA0;					//compare output mode
	TCCR1B = (0x10|(csbits&0x07));	//waveform generation settings and clock select bits 
}

/************************************************************************/
/* interrupt sub-routines                                               */
/************************************************************************/
ISR(TIMER0_OVF_vect)
{
	/***
	 * TC0 overflow interrupt sub-routine
	 * capture ADC result and start next conversation
	 */
	
	if(count<ADC_SAMPLES)
		{
			*pa = ADC;	//capture data
			 pa++;		//inc pointer
			
			count++;	//inc counter
			
			if(!(count<ADC_SAMPLES))
				set_bit(ADMUX, MUX0);
		}
	else
		{
			sb = ADC;		//capture data
			
			pa = &sa[0];	//reset pointer
			count = 0;		//reset sample counter
			update = 1;		//set update flag
			
			clear_bit(ADMUX, MUX0);
			clear_bit(TIMSK0, TOIE0);
		}
	
	ADC_Start();	//start next conversation
	return;			//return
}

ISR(TIMER1_OVF_vect)
{
	/***
	 * TC1 overflow
	 * update PWM generator parameters
	 */
	
	ICR1  = x;	//TOP
	OCR1A = y;	//OCRA
	OCR1B = z;	//OCRB
	
	clear_bit(TIMSK1, TOIE1);	//disable TC1 overflow interrupt
	set_bit(TIMSK0, TOIE0);		//enable TC0 overflow interrupt
	return;
}

/************************************************************************/
/* program entry point                                                  */
/************************************************************************/
int main(void)
{
	PWM_init(2);
	ADC_init(0, 7, 2);
	sei();
	
	while(1)
		{
			if(update)
				{
					update = 0;		//reset update flag
									
					avga = 0;
					for(uint8_t i=0; i<16; i++)	
						avga += sa[i];	//accumulate samples
					avga = (avga>>4);	//divide by sixteen
					
					x = (avga*ICR_MAX)/1024UL;
					if(x < ICR_MIN)	x = ICR_MIN;
					if(x > ICR_MAX) x = ICR_MAX;
					
					y = sb * x;
					y = y / 1024UL;
					z = y - ((y*10)/100);
					
					set_bit(TIMSK1, TOIE1);			//enable TC1 overflow interrupt
				}
		}
}