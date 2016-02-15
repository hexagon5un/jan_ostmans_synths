// O2 Minipops rhythm box (c) DSP Synthesizers 2016
// Free for non commercial use

// http://janostman.wordpress.com


#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "O2_data.h"

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
volatile uint16_t SFREQ;
//-----------------------------------------

ISR(TIMER1_COMPA_vect) {

	//-------------------  Ringbuffer handler -------------------------

	if (RingCount) {                            //If entry in FIFO..
		OCR2A = Ringbuffer[(RingRead++)];          //Output LSB of 16-bit DAC
		RingCount--;
	}

	//-----------------------------------------------------------------

}



void setup() {

	OSCCAL=0xFF;

	//Drum mute inputs
	pinMode(2,INPUT_PULLUP);
	pinMode(3,INPUT_PULLUP);
	pinMode(4,INPUT_PULLUP);
	pinMode(5,INPUT_PULLUP);
	pinMode(6,INPUT_PULLUP);
	pinMode(7,INPUT_PULLUP);
	pinMode(8,INPUT_PULLUP);
	pinMode(9,INPUT_PULLUP);

	pinMode(10,INPUT_PULLUP); //RUN - Stop input
	pinMode(12,OUTPUT);       //Reset output
	pinMode(13,OUTPUT);       //Clock output

	pinMode(14,OUTPUT);       //SEQ cnt output
	pinMode(15,OUTPUT); 
	pinMode(16,OUTPUT); 
	pinMode(17,OUTPUT); 

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
	OCR1A = 800; //40KHz Samplefreq

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
	SFREQ=analogRead(0);
	ADCSRA &= ~PS_128;  // remove bits set by Arduino library
	// Choose prescaler PS_128.
	ADCSRA |= PS_128;
	ADMUX = 64;
	sbi(ADCSRA, ADSC);

	Serial.begin(9600);

	Serial.println("Modified Jan Ostman O2 Minipops!");
	Serial.println("[j] for next pattern, [k] for previous");
	Serial.println("[f] to speed up, [d] to slow down.  [space] to pause.");
}




void loop() {  

	uint16_t samplecntBD,samplecntBG2,samplecntCL,samplecntCW,samplecntCY,samplecntGU,samplecntMA,samplecntQU;

	samplecntBD=0;
	samplecntBG2=0;
	samplecntCL=0;
	samplecntCW=0;
	samplecntCY=0;
	samplecntGU=0;
	samplecntMA=0;
	samplecntQU=0;

	uint16_t samplepntBD,samplepntBG2,samplepntCL,samplepntCW,samplepntCY,samplepntGU,samplepntMA,samplepntQU;

	int16_t total;

	uint8_t stepcnt=0;
	uint16_t tempo=3500;
	uint16_t tempocnt=1;
	uint8_t MUX=4;
	uint8_t playing = 1;

	uint8_t patselect=0;
	uint8_t patlength=pgm_read_byte_near(patlen + patselect);

	while(1) { 

		//------ Add current sample word to ringbuffer FIFO --------------------  

		if (RingCount<255) {  //if space in ringbuffer
			total=0;
			if (samplecntBD) {
				total+=(pgm_read_byte_near(BD + samplepntBD++)-128);
				samplecntBD--;
			}
			if (samplecntBG2) {
				total+=(pgm_read_byte_near(BG2 + samplepntBG2++)-128);
				samplecntBG2--;
			}
			if (samplecntCL) {
				total+=(pgm_read_byte_near(CL + samplepntCL++)-128);
				samplecntCL--;
			}
			if (samplecntCW) {
				total+=(pgm_read_byte_near(CW + samplepntCW++)-128);
				samplecntCW--;
			}
			if (samplecntCY) {
				total+=(pgm_read_byte_near(CY + samplepntCY++)-128);
				samplecntCY--;
			}
			if (samplecntGU) {
				total+=(pgm_read_byte_near(GU + samplepntGU++)-128);
				samplecntGU--;
			}
			if (samplecntMA) {
				total+=(pgm_read_byte_near(MA + samplepntMA++)-128);
				samplecntMA--;
			}
			if (samplecntQU) {
				total+=(pgm_read_byte_near(QU + samplepntQU++)-128);
				samplecntQU--;
			}
			if (total<-127) total=-127;
			if (total>127) total=127;                           
			cli();
			Ringbuffer[RingWrite]=total+128;
			RingWrite++;
			RingCount++;
			sei();


			//----------------------------------------------------------------------------

			//--------- sequencer block ----------------------------------------------
			if (playing) {  
				if (!(tempocnt--)) {
					tempocnt=tempo;
					digitalWriteFast(13,HIGH); //Clock out Hi
					uint8_t trig=pgm_read_byte_near(pattern + (patselect<<4) + stepcnt++);
					PORTC=stepcnt;
					uint8_t mask=(PIND>>2)|((PINB&3)<<6);
					trig&=mask;
					if (stepcnt>patlength) stepcnt=0;
					if (stepcnt==0) digitalWriteFast(12,HIGH); //Reset out Hi
					if (stepcnt!=0) digitalWriteFast(12,LOW); //Reset out Lo
					if (trig & 1) {
						samplepntQU=0;
						samplecntQU=7712;
					}
					if (trig & 2) {
						samplepntCY=0;
						samplecntCY=9434;
					}
					if (trig & 4) {
						samplepntMA=0;
						samplecntMA=568;
					}
					if (trig & 8) {
						samplepntCW=0;
						samplecntCW=830;
					}
					if (trig & 16) {
						samplepntCL=0;
						samplecntCL=752;
					}
					if (trig & 32) {
						samplepntBD=0;
						samplecntBD=1076;
					}
					if (trig & 64) {
						samplepntBG2=0;
						samplecntBG2=1136;
					}
					if (trig & 128) {
						samplepntGU=0;
						samplecntGU=2816;
					}
				}
				digitalWriteFast(13,LOW); //Clock out Lo
			}
		}

		//------------------------------------------------------------------------

		// Serial keyboard control block
		if (Serial.available()) {
			char c = Serial.read();
			switch (c){
				case('j'):  /* next pattern */
					++patselect;
					patselect &= 15;   /* only 16 patterns */
					patlength=pgm_read_byte_near(patlen + patselect);
					break;
				case('k'):  /* previous pattern */
					--patselect;
					patselect &= 15;   /* only 16 patterns */
					patlength=pgm_read_byte_near(patlen + patselect);
					break;
				case(' '):
					if (playing){
						playing=0;
					} else {
						playing=1;
					}
					break; 
				case('f'):
					if (tempo > 100){
						tempo -= 100;
					}
					Serial.println(tempo);
					break;
				case('d'):
					tempo += 100;
					Serial.println(tempo);
					break;
				default:
					break;
			}

		}

	}

}
