#include <avr/io.h>
#include <avr/interrupt.h>

namespace Encoder {
	extern volatile uint8_t last_Q0; 
	extern volatile uint8_t last_Q90;
	extern volatile uint8_t speed_buffer_idx;
	extern volatile uint8_t valid_speed_buffer_samples; 
	extern volatile int16_t position;
}