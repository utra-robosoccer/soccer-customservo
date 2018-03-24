/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/eeprom.h>

#include "constants.h"

uint16_t BUFF[BUFF_SIZE >> 1];
volatile uint16_t buff_idx = 0;

volatile uint8_t last_int = 0;
volatile int16_t encoder = 0;
volatile uint8_t buff_full = 0;
volatile uint8_t triggered = 0;
// volatile uint8_t on = 0; // consult OC0A

volatile uint8_t pwm;
volatile uint8_t phase = 2;

const uint8_t PHASES[] = {
	0b01011100,
	0b01001101,
	0b01000111,
	// OFF_MASK,
	// OFF_MASK
};
const uint8_t adc[] = {
	1, 1, 2
};
const uint8_t RESTING[] = {
	0b01010100,
	0b01000101,
	0b01000101
};

void put_commutation(uint8_t c) {
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		PORTD = (PORTD & ~(0b111 << 5)) | ((c & 0b111) << 5);
		PORTB = (PORTB & ~(0b111)) | ((c >> 3) & 0b111);
	}
}

ISR(TIMER0_OVF_vect) {
	put_commutation(OFF_MASK); // de-energize for a bit
	if(!buff_full)
		put_commutation(PHASES[phase]);
	
	TCNT1 = 0; // reset threshold timer
	TIFR0 |= 1;
	triggered = 0;
	pwm++;
	
	ADMUX = (ADMUX & ~(0b11)) | adc[phase];
}
ISR(TIMER0_COMPA_vect) {
	put_commutation(RESTING[phase]);
	phase = (phase + 1) % 3;
}

// void quadrature(uint8_t vec) {
	
// 	last_int = vec;
// 	// PORTB ^= 1; // signal threshold
// }

volatile uint8_t last_Q0 = 0;
volatile uint8_t last_Q90 = 0;

ISR(INT0_vect) {
	uint8_t current_Q0 = (PORTD & 0b100) >> 2;
	if(last_Q0 != current_Q0) {
		if(last_Q0 != last_Q90)
			encoder++;
		else
			encoder--;
	}
	last_Q0 = current_Q0;
	EIFR |= 1;
}
ISR(INT1_vect) {
	uint8_t current_Q90 = (PORTD & 0b1000) >> 3;
	if(last_Q0 != current_Q90) {
		if(last_Q90 == last_Q0)
			encoder++;
		else
			encoder--;
	}
	last_Q90 = current_Q90;
	EIFR |= 0b10;
}

ISR(ANALOG_COMP_vect) {
	if(pwm % PWM_CYCLE_SKIP == 0 && !triggered) {
		PORTB ^= 0b1000;
		// ATOMIC_BLOCK(ATOMIC_FORCEON) {
			if(buff_idx < BUFF_SIZE) {
				BUFF[buff_idx++] = TCNT1; // stash threshold time
				BUFF[buff_idx++] = encoder; // stash encoder value
				BUFF[buff_idx++] = phase;
			}
			else {
				buff_full = 1;
				// OCR0A = 0;
			}
		// }
		triggered = 1;
	}
	ACSR |= 1 << 4;
}

// volatile uint8_t triggered = 0;
// ISR(ADC_vect) {
// 	if((PORTD & 0b1000000 /* OC0A */) && !triggered && ADCL < ADC_TRIGGER && ) {
// 		ADCSRA |= 1 << 6; // start a new conversion
// 		triggered = 1;
// 		BUFF[buff_idx++] = ADCH; // stash result
// 	}
// 	ADCSRA |= 0b10000; // clear ADC interrupt
// }
// void adc_init() {
// 	ADMUX = 0b11 << 6 | // internal 1.1V bandgap
// 	        1 << 5 | // ADCLAR: left-adjust result and take MSBs
// 	        0b0000; // ADC0 <- CH0
// 	ADCSRA = 1 << 7 | // ADC enable
// 	         // 1 << 5 | // ADATE: allow external triggers
// 	         1 << 3 | // ACD interrupt enable
// 	         0b000; // 2x prescaling
// 	// ADCSRB |= 0b100; //  Timer0 overflow triggers ADC conversion
// }

void port_init() {
	DDRD = 0xF0; // output all except TxRx and encoder
	DDRB = 0x0F | (1 << 5); // output all motor phases and on pin 13
	PORTB &= ~(1 << 5);
	
	EICRA = 0b01 << 2 | // logic-level change on INT1 generates INT1
	        0b01;  // logic-level change on INT0 generates INT0
	EIMSK |= 0b11; // enable INT1 and INT0
}
void timer_init() {
	TCCR0A = 0b00 << 6 | // don't touch OC0A // 0b11 << 6 | // OC0A inverted PWM (NMOS control)
	         0b00 << 4 | // don't touch OC0B
	         0b11; // fast PWM (needed to detect on-off cycles)
	TCCR0B = 0 << 3 | // fast PWM (continued)
	         0b011; // 64x prescaling (1024Hz)
	OCR0A = 42;
	TIMSK0 = 0b011; // enable compare match A; enable overflow interrupt
	
	TCCR1A = 0b00 << 6 | // don't touch OC0A
	         0b00 << 4 | // don't touch OC0B
	         0b00000; // Normal mode
	TCCR1B |= 0b01; // no prescaling
}
void ac_init() {
	// ADC0 implicitly from ADMUX & 0xF = 0
	ADCSRB = 1 << 6 | // ADC0 input to comparator
	         0b000; // free-running mode
	ACSR = 1 << 6 | // use internal bandgap (assume 1.136V)
	       1 << 3 | // enable interrupts
	       0b11; // toggle triggers interrupt
}

int main(void)
{
	port_init();
	timer_init();
	ac_init();
	sei();
	
	uint8_t persisted = 0;
	
	// put_commutation(OFF_MASK);
	for(;;) {
		if(buff_full && !persisted) {
			for(uint16_t i = 0; i < BUFF_SIZE; i++) {
				eeprom_write_word((uint16_t*)(i << 1), BUFF[i]);
			}
			persisted = 1;
			PORTB |= 1 << 5;
		}
	}
	return 0;   /* never reached */
}
