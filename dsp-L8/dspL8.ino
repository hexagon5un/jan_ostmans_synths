// dsp-L8 Latin Perc Chip (c) DSP Synthesizers 2015
// Free for non commercial use

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "samples.h"



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
	// set up the ADC
	SFREQ=analogRead(0);
	ADCSRA &= ~PS_128;  // remove bits set by Arduino library
	// Choose prescaler PS_128.
	ADCSRA |= PS_128;
	ADMUX = 64;
	sbi(ADCSRA, ADSC);
	Serial.begin(9600);
	Serial.println("Modified [Jan Ostman] dsp-L8: ");
	Serial.println("https://janostman.wordpress.com/dsp-l8-source-code/");
	Serial.println("Press asdfjkl; to play the drums.");
}



uint8_t phaccAG,phaccCA,phaccMA,phaccWH,phaccTI,phaccCH,phaccQU,phaccBO;
uint8_t pitchAG=32;
uint8_t pitchCA=64;
uint8_t pitchMA=64;
uint8_t pitchWH=32;
uint8_t pitchTI=64;
uint8_t pitchCH=16;
uint8_t pitchQU=16;
uint8_t pitchBO=64;
uint16_t samplecntAG,samplecntCA,samplecntMA,samplecntWH,samplecntTI,samplecntCH,samplecntQU,samplecntBO;
uint16_t samplepntAG,samplepntCA,samplepntMA,samplepntWH,samplepntTI,samplepntCH,samplepntQU,samplepntBO;

void loop() {  
	int16_t total;
	uint8_t oldPORTB;
	uint8_t oldPORTD;

	uint8_t divider;
	uint8_t MUX=0;

	char ser='o';
	//------ Add current sample word to ringbuffer FIFO --------------------  
	if (RingCount<255) {  //if space in ringbuffer
		total=0;
		if (samplecntAG) {
			phaccAG+=pitchAG;
			if (phaccAG & 128) {
				phaccAG &= 127;
				total+=(pgm_read_byte_near(AG + samplepntAG)-128);
				samplepntAG++;
				samplecntAG--;

			}
		}
		if (samplecntBO) {
			phaccBO+=pitchBO;
			if (phaccBO & 128) {
				phaccBO &= 127;
				total+=(pgm_read_byte_near(BO + samplepntBO)-128);
				samplepntBO++;
				samplecntBO--;

			}
		}
		if (samplecntMA) {
			phaccMA+=pitchMA;
			if (phaccMA & 128) {
				phaccMA &= 127;
				total+=(pgm_read_byte_near(MA + samplepntMA)-128);
				samplepntMA++;
				samplecntMA--;

			}
		}
		if (samplecntQU) {
			phaccQU+=pitchQU;
			if (phaccQU & 128) {
				phaccQU &= 127;
				total+=(pgm_read_byte_near(QU + samplepntQU)-128);
				samplepntQU++;
				samplecntQU--;

			}
		}
		if (samplecntCA) {
			phaccCA+=pitchCA;
			if (phaccCA & 128) {
				phaccCA &= 127;
				total+=(pgm_read_byte_near(CA + samplepntCA)-128);
				samplepntCA++;
				samplecntCA--;

			}
		}    
		if (samplecntTI) {
			phaccTI+=pitchTI;
			if (phaccTI & 128) {
				phaccTI &= 127;
				total+=(pgm_read_byte_near(TI + samplepntTI)-128);
				samplepntTI++;
				samplecntTI--;

			}
		}  
		if (samplecntWH) {
			phaccWH+=pitchWH;
			if (phaccWH & 128) {
				phaccWH &= 127;
				total+=(pgm_read_byte_near(WH + samplepntWH)-128);
				samplepntWH++;
				samplecntWH--;

			}
		}  
		if (samplecntCH) {
			phaccCH+=pitchCH;
			if (phaccCH & 128) {
				phaccCH &= 127;
				total+=(pgm_read_byte_near(CH + samplepntCH)-128);
				samplepntCH++;
				samplecntCH--;

			}
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

	if (Serial.available()) {
		ser = Serial.read();
		Serial.write(ser);
		switch(ser){
			case('a'): 
				samplepntBO=0;
				samplecntBO=1474;
				break;
			case('s'): 
				samplepntAG=0;
				samplecntAG=1720;
				break;
			case('d'): 
				samplepntTI=0;
				samplecntTI=3680;
				break;
			case('f'): 
				samplepntQU=0;
				samplecntQU=6502;
				break;
			case('j'): 
				samplepntMA=0;
				samplecntMA=394;
				break;
			case('k'): 
				samplepntCH=0;
				samplecntCH=5596;
				break;

			case('l'): 
				samplepntCA=0;
				samplecntCA=2174;
				break;
			case(';'): 
				samplepntWH=0;
				samplecntWH=2670;
				break;
			default:
				break;
		}

	}

}
