#include "Encoder.h"

namespace Encoder {
	volatile uint8_t last_Q0 = 0;
	volatile uint8_t last_Q90 = 0;
	volatile uint8_t speed_buffer_idx = 0;
	volatile uint8_t valid_speed_buffer_samples = 0;
	volatile int16_t position = 0;
	
	ISR(INT0_vect) {
		// hit_registry(INT0_vect_num);
			
		uint8_t current_Q0 = (PIND & 0b100) >> 2;
		if(last_Q0 != current_Q0) {
			if(last_Q0 != last_Q90)
				position++;
			else
				position--;
		}
		last_Q0 = current_Q0;
		EIFR |= 1;
	}
	ISR(INT1_vect) {
		// hit_registry(INT1_vect_num);
		
		uint8_t current_Q90 = (PIND & 0b1000) >> 3;
		if(last_Q90 != current_Q90) {
			if(last_Q90 == last_Q0)
				position++;
			else
				position--;
		}
		last_Q90 = current_Q90;
		EIFR |= 0b10;
	}
}