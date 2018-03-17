#include "init.h"
#include "constants.h"


void clock_init() {
	CCP = 0xD8;
	OSC.CTRL = 0x02; // OSC_RC32MEN_bm;
	
	while(!(OSC_STATUS & 0x02));
	
	CCP = 0xD8;
	CLK.CTRL = 0x01;
}

void port_init() {
	// BLDC DRIVER DEFAULTS
	PORTC.DIR = 0xFF;
	PORTC.OUT = OFF_MASK;
	
	PORTD.DIR &= ~(0b11); // force quadrature pins to be high impedance
	PORTD.PIN0CTRL |= 0b11; // below
	PORTD.PIN1CTRL |= 0b11; // below
	PORTD.DIR |= 0x30; // D5 out, D4 out
	PORTD.OUT |= 0x20;
	
	PORTD.INTCTRL = 0b10; // medium-pri interrupt
	PORTD.INT1MASK = 0b1100; // pins 2 and 3 can generate interrupts
	PORTD.PIN2CTRL |= PORT_ISC_BOTHEDGES_gc;
}


void timer_init() {
	//////////////////////////////////////////
	///// C0: INDUCTANCE ANGLE SCHEDULER /////
	//////////////////////////////////////////
	
	// TCC0.CTRLB = // (1 << 5) | // override PC1 (CCB)
	//              0b000; // normal mode, generate interrupts
	// TCC0.CTRLA = 0b100; // 8x prescaler
	
	// TCC0.PER = 1 << 13; // 12-bit timer on 1MHz peripheral clock -> 256Hz PWM
	// TCC0.INTCTRLA |= 0b11;
	// TCC0.INTCTRLB |= 0b11 << 2; // CCB trigger
	
	// short cc = TCC0.PER * 0.05f;
	// // short cc = TCC0.PER * (1 - duty);
	// TCC0.CCA = cc;
	// TCC0.CCB = cc;
	
	// EVSYS.CH0MUX = (0b1100 << 4) | // TCC0 writes event channel 0
	// (0b0100); // CCA trigger
	
	// EVSYS.CH1MUX = (0b1100 << 4) | // TCC0 writes to event channel 1
	// (0b0000); // overflow trigger
	
	///////////////////////////////////
	///// C1: OPEN-LOOP DRIVE PWM /////
	///////////////////////////////////
	
	// TCC1.CTRLA = 0b001; // 1x prescaler
	TCC0.CTRLB = // 0b0010 << 4 | // override C1
	             0b101; // dual-slope PWM
	TCC0.CTRLA = OPEN_LOOP_DRIVE_PRESCALER;
	TCC0.PER = OPEN_LOOP_DRIVE_PER;
	TCC0.CCA = 0;
	TCC0.CCB = 0;
	TCC0.CCC = 0;
	
	//////////////////////////////////
	///// D0: QUADRATURE DECODER /////
	//////////////////////////////////
	
	TCD0.CTRLA = 0b0001; // 1x prescaler
	TCD0.CTRLD = (0b011 << 5) | // quadrature decode from events...
	(0b1010); // grab quadrature events from channel 2
	
	EVSYS.CH2MUX = (0b01101 << 3) | // PORTD generates interrupts
	(0b000); // pin 0
	EVSYS.CH2CTRL = (0b1 << 3) | // enable quadrature decoding on event channel
	(0b011); // four-sample digital filtering
	
	////////////////////////////////
	///// D1: SATURATION TIMER /////
	////////////////////////////////
	
	TCD1.CTRLB = 0b000;
	TCD1.CTRLA = 0b011; // 4x prescaler
}


void adc_init() {
	ADCB.CTRLA = 0b1; // enable
	ADCB.CTRLB = (1 << 7) | // low-impedance source
	(1 << 3) | // free-run
	0b10 << 1; // 8-bit result for faster conversion
	ADCB.PRESCALER = 0b0; // 4x prescaled, that's really low enough impedance
	ADCB.CMP = 200; // assume Vtn = 3.0V => anything below 1.00V * 220 / (1k + 220) really
	
	ADCB.CH0.CTRL = 0b01; // single-ended external input
	ADCB.CH0.MUXCTRL = 0b0000 << 3; // PB0
	
	ADCB.CH0.INTCTRL = 0b01 << 2; // below threshold comparison interrupt mode
	
	
	// ADCB: BEMF measurement
	// ADCB.PRESCALER = 0b0; // try at max clock
	// // unsure if I need channel sweep control: if it's asking which channels generate interrupts then I want to constraint to CH0 (only one zero crossing of interest)
	// ADCB.CTRLB |= 1 << 4; // signed for differential
	// ADCB.CMP = 0; // looking for zero-crossings; might offset for noise immunity
	// ADCB.CH0.CTRL = 0b10; // diff without gain
	// ADCB.CH0.MUXCTRL = (4 << 3) | 0b000; // differential across which pins (B4 against B0)
	// ADCB.CH0.INTCTRL = 0b11; // posedge
}

void interrupt_init() {
	PMIC.CTRL |= 0b111;
}