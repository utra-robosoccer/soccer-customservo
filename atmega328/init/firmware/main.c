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
#include <stdlib.h>

#include "constants.h"
#include "init.h"
#include "pin_util.h"

uint16_t BUFF[BUFF_SIZE >> 1];
volatile uint16_t buff_idx = 0;

volatile uint8_t last_int = 0;
volatile int16_t encoder = 0;
volatile uint8_t buff_full = 0;
volatile uint8_t triggered = 0;
// volatile uint8_t on = 0; // consult OC0A

volatile uint8_t phase = 2;

typedef enum program_state_t
{
	FIRST_ENCODER_TICK = 1,
	INDUCTANCE_ANGLE = 2,
	RUNNING = 3
} program_state_t;

volatile program_state_t PROGRAM_STATE = FIRST_ENCODER_TICK;

const uint8_t PHASES[] = {
	0b01011100,
	0b01001101,
	0b01000111,
	// OFF_MASK,
	// OFF_MASK
};
const uint8_t adc[] = {
	4, 4, 5
};
const uint8_t RESTING[] = {
	0b01010100,
	0b01000101,
	0b01000101
};
int8_t SPEED = 60;

ISR(TIMER0_OVF_vect) {
	if(PROGRAM_STATE == INDUCTANCE_ANGLE) {
		phase = (phase + 1) % 3;
		
		TCNT1 = 0; // reset threshold timer
		triggered = 0;
		
		TIMSK1 &= ~(1 << 5); // disable input capture interrupt
		ADMUX = (ADMUX & ~(0b11)) | adc[phase];
	}
	
	put_commutation(OFF_MASK); // de-energize for a bit to avoid short
	if(PROGRAM_STATE != INDUCTANCE_ANGLE || !buff_full)
		put_commutation(PHASES[phase]);
	
	if(PROGRAM_STATE == INDUCTANCE_ANGLE)
		TIMSK1 |= 1 << 5; // re-enable input capture interrupt
	
	TIFR0 |= 1;
}
ISR(TIMER0_COMPA_vect) {
	put_commutation(RESTING[phase]);
}

volatile uint8_t last_Q0 = 0;
volatile uint8_t last_Q90 = 0;
volatile uint8_t speed_buffer[SPEED_BUFFER_SIZE];
volatile uint8_t speed_buffer_idx = 0;
volatile uint8_t valid_speed_buffer_samples = 0;

ISR(INT0_vect) {
	uint8_t current_Q0 = (PIND & 0b100) >> 2;
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
	uint8_t current_Q90 = (PIND & 0b1000) >> 3;
	if(last_Q90 != current_Q90) {

		if(last_Q90 == last_Q0)
			encoder++;
		else
			encoder--;
	}
	last_Q90 = current_Q90;
	EIFR |= 0b10;
}

typedef struct saturation_timing_buffer_t {
	uint16_t buffer[INDUCTANCE_ANGLE_SAMPLES];
	volatile uint16_t counter;
} saturation_timing_buffer_t;
volatile saturation_timing_buffer_t sat_timing_buffers[3];

// ISR(ANALOG_COMP_vect) {
ISR(TIMER1_CAPT_vect) {
	if(!triggered && sat_timing_buffers[phase].counter < INDUCTANCE_ANGLE_SAMPLES) {
		// PORTB ^= 0b1000;
		sat_timing_buffers[phase].buffer[sat_timing_buffers[phase].counter++] = ICR1;
		triggered = 1;
	}
	ACSR |= 1 << 4;
}

int main(void)
{
	port_init();
	timer_init();
	ac_init();
	sei();
	
	uint8_t persisted = 0;
	
	uint8_t sine_idx = 0;
	while(abs(encoder) < 18) {
		// one tick on a 64 CPR encoder
		if(sine_idx == 127) {
			sine_idx = 0;
			phase = (phase + 1) % 3;
		}
		OCR0A = HALF_SINE[sine_idx++] >> 2; // max 25% duty
		_delay_ms(50);
	}
	
	// TCCR0B = (TCCR0B & ~(0b111)) | 0b101; // 1024x prescaling
	TCCR0B = (TCCR0B & ~(0b010)) | 0b101; // 8x prescaling
	OCR0A = 150; // 100us pulse width for 8x prescaler
	PROGRAM_STATE = INDUCTANCE_ANGLE;
	
	uint8_t all_buffers_full;
	do {
		all_buffers_full = 1;
		for(uint8_t i = 0; i < 3; i++) {
			all_buffers_full &= sat_timing_buffers[i].counter == INDUCTANCE_ANGLE_SAMPLES;
		}
	}
	while(!all_buffers_full);
	
	PROGRAM_STATE = RUNNING;
	
	// put_commutation(OFF_MASK);
	for(;;) {
		if(valid_speed_buffer_samples < SPEED_BUFFER_SIZE)
			valid_speed_buffer_samples++;
		
		speed_buffer_idx = (speed_buffer_idx + 1) % SPEED_BUFFER_SIZE;
		speed_buffer[speed_buffer_idx] = TCNT1 / encoder;
		TCNT1 = 0;
		encoder = 0;
		// if(buff_full && !persisted) {
		// 	for(uint16_t i = 0; i < BUFF_SIZE; i++) {
		// 		eeprom_write_word((uint16_t*)(i << 1), BUFF[i]);
		// 	}
		// 	persisted = 1;
		// 	PORTB |= 1 << 5;
		// }
	}
	return 0;   /* never reached */
}
