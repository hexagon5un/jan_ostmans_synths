// dsp-D8 Drum Chip (c) DSP Synthesizers 2015
// Free for non commercial use

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "drum_samples.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 

// Standard Arduino Pins
#define digitalPinToPortReg(P) \
	(((P) >= 0 && (P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define digitalPinToDDRReg(P) \
	(((P) >= 0 && (P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define digitalPinToPINReg(P) \
	(((P) >= 0 && (P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define digitalPinToBit(P) \
	(((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P) - 8 : (P) - 14))

#define digitalReadFast(P) bitRead(*digitalPinToPINReg(P), digitalPinToBit(P))

#define digitalWriteFast(P, V) bitWrite(*digitalPinToPortReg(P), digitalPinToBit(P), (V))

const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);




//--------- Ringbuf parameters ----------
uint8_t Ringbuffer[256];
uint8_t RingWrite=0;
uint8_t RingRead=0;
volatile uint8_t RingCount=0;

//-----------------------------------------
ISR(TIMER1_COMPA_vect) {

	//-------------------  Ringbuffer handler -------------------------

	if (RingCount) {                            //If entry in FIFO..
		OCR2A = Ringbuffer[(RingRead++)];          //Output LSB of 16-bit DAC
		RingCount--;
	}

	//-----------------------------------------------------------------

}



void setup() 
{
	OSCCAL=0xFF;

	//Drumtrigger inputs
	pinMode(2,INPUT_PULLUP);
	pinMode(3,INPUT_PULLUP);
	pinMode(4,INPUT_PULLUP);
	pinMode(5,INPUT_PULLUP);
	pinMode(6,INPUT_PULLUP);
	pinMode(7,INPUT_PULLUP);
	pinMode(8,INPUT_PULLUP);
	pinMode(9,INPUT_PULLUP);
	pinMode(10,INPUT_PULLUP);

	//8-bit PWM DAC pin
	pinMode(11,OUTPUT);

	// Set up Timer 1 to send a sample every interrupt.
	cli();
	// Set CTC mode
	// Have to set OCR1A *after*, otherwise it gets reset to 0!
	TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
	TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));    
	// No prescaler
	TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
	// Set the compare register (OCR1A).
	// OCR1A is a 16-bit register, so we have to do this with
	// interrupts disabled to be safe.
	//OCR1A = F_CPU / SAMPLE_RATE; 
	// Enable interrupt when TCNT1 == OCR1A
	TIMSK1 |= _BV(OCIE1A);   
	sei();
	OCR1A = 400; //40KHz Samplefreq

	// Set up Timer 2 to do pulse width modulation on D11

	// Use internal clock (datasheet p.160)
	ASSR &= ~(_BV(EXCLK) | _BV(AS2));

	// Set fast PWM mode  (p.157)
	TCCR2A |= _BV(WGM21) | _BV(WGM20);
	TCCR2B &= ~_BV(WGM22);

	// Do non-inverting PWM on pin OC2A (p.155)
	// On the Arduino this is pin 11.
	TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
	TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
	// No prescaler (p.158)
	TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

	// Set initial pulse width to the first sample.
	OCR2A = 128;

	//set timer0 interrupt at 61Hz
	TCCR0A = 0;// set entire TCCR0A register to 0
	TCCR0B = 0;// same for TCCR0B
	TCNT0  = 0;//initialize counter value to 0
	// set compare match register for 62hz increments
	OCR0A = 255;// = 61Hz
	// turn on CTC mode
	TCCR0A |= (1 << WGM01);
	// Set CS01 and CS00 bits for prescaler 1024
	TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00);  //1024 prescaler 

	TIMSK0=0;


	// set up the ADC
	ADCSRA &= ~PS_128;  // remove bits set by Arduino library
	// Choose prescaler PS_128.
	ADCSRA |= PS_128;
	ADMUX = 64;
	sbi(ADCSRA, ADSC);

	Serial.begin(9600);
	Serial.println("Modified [Jan Ostman] dsp-D8:");
	Serial.println("https://janostman.wordpress.com/the-dsp-d8-drum-chip-source-code/");
	Serial.println("Press asdfjkl; to play the drums.");
}

void loop() 
{  
	uint8_t phaccBD,phaccCH,phaccCL,phaccCR,phaccOH,phaccRD,phaccRS,phaccSD;
	uint8_t pitchBD=128;
	uint8_t pitchCH=64;
	uint8_t pitchCL=64;
	uint8_t pitchCR=16;
	uint8_t pitchOH=64;
	uint8_t pitchRD=16;
	uint8_t pitchRS=64;
	uint8_t pitchSD=64;
	uint16_t samplecntBD,samplecntCH,samplecntCL,samplecntCR,samplecntOH,samplecntRD,samplecntRS,samplecntSD;
	uint16_t samplepntBD,samplepntCH,samplepntCL,samplepntCR,samplepntOH,samplepntRD,samplepntRS,samplepntSD;

	int16_t total;
	uint8_t oldPORTB;
	uint8_t oldPORTD;

	uint8_t divider;
	uint8_t MUX=0;
	char ser=' ';
	while(1) { 

		//------ Add current sample word to ringbuffer FIFO --------------------  

		if (RingCount<255) {  //if space in ringbuffer
			total=0;
			if (samplecntBD) {
				phaccBD+=pitchBD;
				if (phaccBD & 128) {
					phaccBD &= 127;
					samplepntBD++;
					samplecntBD--;

				}
				total+=(pgm_read_byte_near(BD + samplepntBD)-128);
			}
			if (samplecntSD) {
				phaccSD+=pitchSD;
				if (phaccSD & 128) {
					phaccSD &= 127;
					samplepntSD++;
					samplecntSD--;

				}
				total+=(pgm_read_byte_near(SN + samplepntSD)-128);
			}
			if (samplecntCL) {
				phaccCL+=pitchCL;
				if (phaccCL & 128) {
					phaccCL &= 127;
					samplepntCL++;
					samplecntCL--;

				}
				total+=(pgm_read_byte_near(CL + samplepntCL)-128);
			}
			if (samplecntRS) {
				phaccRS+=pitchRS;
				if (phaccRS & 128) {
					phaccRS &= 127;
					samplepntRS++;
					samplecntRS--;

				}
				total+=(pgm_read_byte_near(RS + samplepntRS)-128);
			}
			if (samplecntCH) {
				phaccCH+=pitchCH;
				if (phaccCH & 128) {
					phaccCH &= 127;
					samplepntCH++;
					samplecntCH--;

				}
				total+=(pgm_read_byte_near(CH + samplepntCH)-128);
			}    
			if (samplecntOH) {
				phaccOH+=pitchOH;
				if (phaccOH & 128) {
					phaccOH &= 127;
					samplepntOH++;
					samplecntOH--;

				}
				total+=(pgm_read_byte_near(OH + samplepntOH)-128);
			}  
			if (samplecntCR) {
				phaccCR+=pitchCR;
				if (phaccCR & 128) {
					phaccCR &= 127;
					samplepntCR++;
					samplecntCR--;

				}
				total+=(pgm_read_byte_near(CR + samplepntCR)-128);
			}  
			if (samplecntRD) {
				phaccRD+=pitchRD;
				if (phaccRD & 128) {
					phaccRD &= 127;
					samplepntRD++;
					samplecntRD--;

				}
				total+=(pgm_read_byte_near(RD + samplepntRD)-128);
			}  
			total>>=1;  
			if (!(PINB&4)) total>>=1;
			total+=128;  
			if (total>255) total=255;

			cli();
			Ringbuffer[RingWrite]=total;
			RingWrite++;
			RingCount++;
			sei();
		}

		//----------------------------------------------------------------------------

		//----------------- Handle Triggers ------------------------------
		if (Serial.available() > 0) {
			ser = Serial.read();
			Serial.write(ser);
			switch (ser){
				case('a'):	
					samplepntBD=0;
					samplecntBD=2154;
					break;
				case('s'): 
					samplepntSD=0;
					samplecntSD=3482;
					break;
				case('d'): 
					samplepntCH=0;
					samplecntCH=482;
					break;
				case('f'): 
					samplepntOH=0;
					samplecntOH=2572;
					break;
				case('j'): 
					samplepntRS=0;
					samplecntRS=1160;
					break;
				case('k'): 
					samplepntCL=0;
					samplecntCL=2384;
					break;
				case('l'): 
					samplepntRD=0;
					samplecntRD=5066;
					break;
				case(';'): 
					samplepntCR=0;
					samplecntCR=5414;
					break;
				default:
					break;
			}
		}

		//-----------------------------------------------------------------

	}
}
