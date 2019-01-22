#define F_CPU 8000000

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "i2c/i2c_slave_defs.h"
#include "i2c/i2c_machine.h"

// Theory of operation:
//  Set up timer0 to overflow every ~20ms
//  Set up timer0 to generate interrupt after ~0.5ms
//  In timer0 event interrupt, trigger timer1
//  Timer1 set up with 128 prescaler
//

#define SERVO_PORT  PORTB
#define SERVO_DDR   DDRB
#define SERVO_PIN_A (1 << 3)
#define SERVO_PIN_B (1 << 4)

// No idea why...
#define SERVO_MIN 2

volatile uint8_t i2c_reg[I2C_N_REG] = {
	0x0,        // CONTROL
	0x80,       // SERVO_A
	SERVO_MIN,  // SERVO_A_MIN
	0xFF,       // SERVO_A_MAX
	0x80,       // SERVO_B
	SERVO_MIN, // SERVO_B_MIN
	0xFF,       // SERVO_B_MAX
};
const uint8_t eeprom[] EEMEM = { 0x0, 0x80, SERVO_MIN, 0xFF, 0x80, SERVO_MIN, 0xFF };

const uint8_t i2c_w_mask[I2C_N_REG] = { 0x83, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

enum servo_id {
	SERVO_A = 0,
	SERVO_B,
};

static volatile uint8_t servos_enabled_pins;

// Sets the PWM pin low, and disables the high-res timer if needed
static void servo_done(uint8_t pin) {
	static uint8_t servos_done = 0;

	SERVO_PORT &= ~(pin);

	servos_done |= pin;
	if (servos_done == servos_enabled_pins) {
		TCCR1 = 0;
		TCNT1 = 0;
		servos_done = 0;
	}
}

// Runs at the start of each PWM cycle
ISR(TIMER0_COMPA_vect) {
	SERVO_PORT |= servos_enabled_pins;
}

// Runs 0.5 ms into the cycle, starts the high-res "turn off" counter
ISR(TIMER0_COMPB_vect) {
	// clk/64. 8us/tick
	TCCR1 = (0x7 << CS00);
}

ISR(TIMER1_COMPA_vect) {
	servo_done(SERVO_PIN_A);
}

ISR(TIMER1_COMPB_vect) {
	servo_done(SERVO_PIN_B);
}

void servo_set(enum servo_id servo, uint8_t pos) {
	uint8_t min, max;
	uint16_t tmp;

	switch (servo) {
	case SERVO_A:
		min = REG_SERVO_A_MIN;
		max = REG_SERVO_A_MAX;
		tmp = (max - min) * pos;

		OCR1A = (tmp >> 8) + min;
		break;
	case SERVO_B:
		min = REG_SERVO_B_MIN;
		max = REG_SERVO_B_MAX;
		tmp = (max - min) * pos;

		OCR1B = (tmp >> 8) + min;
		break;
	}
}

static void servo_enable(enum servo_id servo) {
	cli();
	switch (servo) {
	case SERVO_A:
		TIMSK |= (1 << OCIE1A);
		servos_enabled_pins |= SERVO_PIN_A;
		break;
	case SERVO_B:
		TIMSK |= (1 << OCIE1B);
		servos_enabled_pins |= SERVO_PIN_B;
		break;
	}
	sei();
}

static void servo_disable(enum servo_id servo) {
	cli();
	switch (servo) {
	case SERVO_A:
		TIMSK &= ~(1 << OCIE1A);
		servo_done(SERVO_PIN_A);
		servos_enabled_pins &= ~(SERVO_PIN_A);
		break;
	case SERVO_B:
		TIMSK &= ~(1 << OCIE1B);
		servo_done(SERVO_PIN_B);
		servos_enabled_pins &= ~(SERVO_PIN_B);
		break;
	}
	sei();
}

static void save_to_eeprom() {
	eeprom_write_block(i2c_reg, (void *)eeprom, I2C_N_REG);
}

static void load_from_eeprom() {
	uint8_t i;

	eeprom_read_block(i2c_reg, (void *)eeprom, I2C_N_REG);
	for (i = 0; i < I2C_N_REG; i++) {
		i2c_reg[i] = i2c_reg[i] & i2c_w_mask[i];
	}
}

void main(void)
{
	DDRB |= SERVO_PIN_A | SERVO_PIN_B;

	GTCCR = (1 << TSM) | (1 << PSR0);
	// CTC mode - overflow at OCR0A
	TCCR0A = (0x2 << WGM00);
	// Divide by 1024. 0.000128 s/tick
	TCCR0B = (0x5 << CS00);
	// Reset every 9.98ms
	OCR0A = 78;
	// OC interrupt after 0.512ms
	OCR0B = 4;

	TIMSK |= (1 << OCIE0B) | (1 << OCIE0A);

	GTCCR = 0;

	load_from_eeprom();
	i2c_init();

	sei();

	if (REG_CONTROL & (1 << SERVO_A)) {
		servo_set(SERVO_A, REG_SERVO_A);
		servo_enable(SERVO_A);
	}
	if (REG_CONTROL & (1 << SERVO_B)) {
		servo_set(SERVO_B, REG_SERVO_B);
		servo_enable(SERVO_B);
	}

	uint8_t ctl = REG_CONTROL;
	for (;;) {
		if (i2c_check_stop()) {
			if (REG_SERVO_A_MIN < SERVO_MIN) {
				REG_SERVO_A_MIN = SERVO_MIN;
			}

			if (REG_SERVO_B_MIN < SERVO_MIN) {
				REG_SERVO_B_MIN = SERVO_MIN;
			}

			if (REG_SERVO_A_MAX < REG_SERVO_A_MIN) {
				REG_SERVO_A_MAX = REG_SERVO_A_MIN;
			}

			if (REG_SERVO_B_MAX < REG_SERVO_B_MIN) {
				REG_SERVO_B_MAX = REG_SERVO_B_MIN;
			}

			if (REG_CONTROL & (1 << SERVO_A)) {
				servo_set(SERVO_A, REG_SERVO_A);
				if (!(ctl & (1 << SERVO_A))) {
					servo_enable(SERVO_A);
				}
			} else {
				if (ctl & (1 << SERVO_A)) {
					servo_disable(SERVO_A);
				}
			}

			if (REG_CONTROL & (1 << SERVO_B)) {
				servo_set(SERVO_B, REG_SERVO_B);
				if (!(ctl & (1 << SERVO_B))) {
					servo_enable(SERVO_B);
				}
			} else {
				if (ctl & (1 << SERVO_B)) {
					servo_disable(SERVO_B);
				}
			}
			ctl = REG_CONTROL;

			if (ctl & 1 << 7) {
				REG_CONTROL &= ~(1 << 7);
				save_to_eeprom();
			}
		}
		sleep_mode();
	};
}
