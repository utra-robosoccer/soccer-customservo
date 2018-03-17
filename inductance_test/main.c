/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <math.h>

#include "constants.h"

#include "init.h"

typedef enum program_state_t
{
	FIRST_ENCODER_TICK = 1,
	INDUCTANCE_ANGLE = 2,
	RUNNING = 3
} program_state_t;

volatile program_state_t PROGRAM_STATE = FIRST_ENCODER_TICK;

volatile short pwm = 0; // at most 1kB cycles; assuming 10s test, 1kHz PWM, 10 samples per pulse, must skip every 100th PWM cycle
// volatile uint8_t adc_count = 0;
volatile int on_flag = 0;

volatile uint8_t phase = 0;
const uint8_t PHASES[] = {
	0b01010011,
	0b01010001, // resting
	// OFF_MASK, OFF_MASK,
	0b01001101, 0b01000101, // resting
	// OFF_MASK, OFF_MASK,
	0b01110100,
	0b01010100 // resting
};

// #define BUFF_SIZE 4096
#define BUFF_SIZE 2048
uint16_t BUFF[BUFF_SIZE];
volatile uint16_t buff_addr = 0x0;
volatile int buff_full = 0;
volatile int thresholded = 0;

ISR(PORTD_INT0_vect) {
	PORTD.INTCTRL &= ~(0b11); // clear interrupts on INT0
	if(PROGRAM_STATE == FIRST_ENCODER_TICK) {
		// PROGRAM_STATE = INDUCTANCE_ANGLE;
		// TCC1.CTRLB &= ~(0b0001 << 4); // disable PWM driving motor towards first tick
		
		// TCC0.CTRLA = INDUCTANCE_ANGLE_SCHEDULER_PRESCALER;
	}
}

ISR(TCC0_OVF_vect) {
	on_flag = 1;
	if(1) { // PROGRAM_STATE == INDUCTANCE_ANGLE) {
		PORTC_OUT = OFF_MASK; // protect against stray energization
		PORTC_OUT = PHASES[phase << 1];
		
		PORTD.OUT |= 0x10;
		
		phase = (phase + 1) % 3;
		if(++pwm % PWM_CYCLE_SKIP == 0) {
			TCC1.CNT = 0; // reset posedge timer
			thresholded = 0;
			ADCB.CH0.INTCTRL |= 0b10; // med-pri interrupts for below comparison
		}
	}
	TCC0.INTFLAGS |= 1;
}

ISR(TCC0_CCB_vect) {
	ADCB_CH0_INTCTRL &= ~(0b10); // disable ADC interrupts foremost
	
	on_flag = 0;
	
	PORTD.OUT &= ~(0x10);
	
	PORTC_OUT = PHASES[1 | (phase << 1)];
	ADCB_CH0_MUXCTRL = phase << 3;
	TCC0_INTFLAGS |= 0b10000;
}

typedef struct saturation_timing_buffer_t {
	uint16_t buffer[INDUCTANCE_ANGLE_SAMPLES];
	volatile uint16_t counter;
} saturation_timing_buffer_t;
volatile saturation_timing_buffer_t sat_timing_buffers[3];

ISR(ADCB_CH0_vect) {
	uint16_t t = TCC1.CNT;
	uint16_t encoder_v = TCD0.CNT;
	
	if(t < SATURATION_TIME_THRESHOLD) return; // false alarms
	
	if(on_flag && !thresholded) {
		PORTD.OUT &= ~(0x10);
		ADCB_CH0_INTCTRL &= ~(0b10);
		thresholded = 1;
		
		if(buff_addr < BUFF_SIZE - 3) {
			BUFF[buff_addr++] = t;
			BUFF[buff_addr++] = encoder_v;
			BUFF[buff_addr++] = phase;
		}
		else {
			buff_full = 1;
			TCC0.CTRLA = 0;
		}
		
		// if(sat_timing_buffers[phase].counter < INDUCTANCE_ANGLE_SAMPLES)
		// 	sat_timing_buffers[phase].buffer[sat_timing_buffers[phase].counter++] = t;
	}
	ADCB_CH0_INTFLAGS = 1;
}

int main(void)
{
	/*
	Timers
		C0: 
	*/
	clock_init();
	port_init();
	timer_init();
	adc_init();
	interrupt_init();
	sei();
	
	//////////////////////////////////////////////
	///////////// FIRST ENCODER TICK /////////////
	//////////////////////////////////////////////
	
	/*
	Drive the motor in open loop until the encoder hits the first tick
	*/
	
	// PORTC.OUT = 0b01010001;
	// uint16_t CC = OPEN_LOOP_DRIVE_THRESHOLD * 0.85f;
	// // TCC1.CTRLA = 0b001; // 1x prescaler
	// while(PROGRAM_STATE == FIRST_ENCODER_TICK) {
	// 	TCC1.CCB = OPEN_LOOP_DRIVE_THRESHOLD - CC;
	// 	// _delay_ms(5);
	// 	// CC >>= 1; // ramp up the duty cycle if we still haven't hit an encoder tick yet
	// }
	
	TCC1.CTRLB &= ~(0b0010 << 4);
	
	///////////////////////////////////////////////////
	///////////// PHASE ANGLE MEASUREMENT /////////////
	///////////////////////////////////////////////////
	
	/*
	Identify the phase angle by timing the transistor saturation delay,
	averaging over some number of rectangular pulses
	
	The measuring is entirely orchestrated by the interrupts, so just idle
	in main until the results are out
	*/
	
	// TEMP
	TCC0.CTRLA = INDUCTANCE_ANGLE_SCHEDULER_PRESCALER;
	
	// while(PROGRAM_STATE == INDUCTANCE_ANGLE) {
	int persisted = 0;
	for(;;) {
		if(buff_full && !persisted) {
			PORTC.OUT = OFF_MASK;
			for(uint16_t i = 0; i < BUFF_SIZE; i++) {
				eeprom_write_word((uint16_t*)(i << 1), BUFF[i]);
			}
			persisted = 1;
			PORTD.OUT &= ~(0x30);
		}
		
		// int all_buffers_full = 1;
		// for(uint8_t i = 0; i < 3; i++)
		// 	if(sat_timing_buffers[i].counter < INDUCTANCE_ANGLE_SAMPLES)
		// 		all_buffers_full = 0;
		// if(all_buffers_full) {
		// 	PROGRAM_STATE = RUNNING;
		// 	break;
		// }
	}
	TCC1.CTRLA = 0; // no more inductance cycles
	PORTC.OUT = OFF_MASK;
	
	// find angle relative to A phase (LSB)
	float a_hat, b_hat, c_hat;
	for(uint16_t i = 0; i < INDUCTANCE_ANGLE_SAMPLES; i++) {
		a_hat += sat_timing_buffers[0].buffer[i], b_hat += sat_timing_buffers[1].buffer[i], c_hat += sat_timing_buffers[2].buffer[i];
	}
	float total_avg = (a_hat + b_hat + c_hat) / 3;
	a_hat = (a_hat - total_avg) / INDUCTANCE_ANGLE_SAMPLES;
	b_hat = (b_hat - total_avg) / INDUCTANCE_ANGLE_SAMPLES;
	
	float theta = acos(SQRT_THREE_OVER_TWO * a_hat / sqrt(a_hat * a_hat + a_hat * b_hat + b_hat * b_hat));
	
	///////////////////////////////////
	///////////// RUNNING /////////////
	///////////////////////////////////
	
	/*
	For now, just let the interrupts handle everything
	*/
	
	while(PROGRAM_STATE == RUNNING);
}
