#define F_CPU 8000000

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "i2c/i2c_slave_defs.h"
#include "i2c/i2c_machine.h"

// Theory of operation:
//  - Set up timer0 to overflow every N milliseconds (whatever servo period you
//    want), and to trigger compare-match after 0.5ms
//  - When timer0 overflows, set the servo pins high
//  - When the timer0 compare match triggers:
//    - Set up timer1 to compare-match after (pulse_width - 0.5) ms
//    - Start timer1
//  - When timer1 compare-match fires, set the servo pin low
//
// This gives us high-precision on the servo pulse width. If we just used timer1
// to generate the PWM signal directly, we'd waste 0.5 ms worth of our resolution
// at the start of the cycle (the pulse is *always* at least 0.5ms long)
//
// The pins are controlled from SW rather than via the compare-match HW, because
// the timer lacks an appropriate compare-match mode to prevent a glitch at the
// 0.5 ms point.
//
// With this approach, we can generate pulses between 0.5-2.0 ms, with 8 us precision.
//
//     <-- 0.5ms --> <------- OCR1x * 8 us -------->
//    |`````````````````````````````````````````````|                       |
// ___|                                             |__________...._________|
//
//    ^- timer0 overflow                                                    ^- timer0 overflow
//                ^- timer0 compare-match           ^- timer1 compare-match
//                   start timer1
//
//     <---------------------- Full cycle, 20 ms or whatever -------------->
//

#define SERVO_PORT  PORTB
#define SERVO_DDR   DDRB
#define SERVO_PIN_A (1 << 3)
#define SERVO_PIN_B (1 << 1)
#define LASER_PIN (1 << 4)

// No idea why...
#define SERVO_MIN 2

volatile uint8_t i2c_reg[I2C_N_REG] = {
	0x0,        // CONTROL
	0x80,       // SERVO_A
	0x80,       // SERVO_B
	SERVO_MIN,  // SERVO_A_MIN
	0xFF,       // SERVO_A_MAX
	SERVO_MIN, // SERVO_B_MIN
	0xFF,       // SERVO_B_MAX
};
const uint8_t eeprom[] EEMEM = { 0x0, 0x80, 0x80, SERVO_MIN, 0xFF, SERVO_MIN, 0xFF };
// 0, 0, 0, 128, 75, 128, 180

const uint8_t i2c_w_mask[I2C_N_REG] = { 0x87, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

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

static uint8_t calc_pos(uint8_t start, uint8_t end, uint8_t pos) {
	int16_t range = end - start;
	int16_t offs = (int32_t)(range * pos) >> 8;

	return start + offs;
}

void servo_set(enum servo_id servo, uint8_t pos) {

	switch (servo) {
	case SERVO_A:
		pos = calc_pos(REG_SERVO_A_MIN, REG_SERVO_A_MAX, pos);
		OCR1A = pos;
		break;
	case SERVO_B:
		pos = calc_pos(REG_SERVO_B_MIN, REG_SERVO_B_MAX, pos);
		OCR1B = pos;
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

static void save_to_eeprom(void) {
	eeprom_write_block((const void *)i2c_reg, (void *)eeprom, I2C_N_REG);
}

static void load_from_eeprom(void) {
	uint8_t i;

	eeprom_read_block((void *)i2c_reg, (void *)eeprom, I2C_N_REG);
	for (i = 0; i < I2C_N_REG; i++) {
		i2c_reg[i] = i2c_reg[i] & i2c_w_mask[i];
	}
}

static void laser_on(void) {
	DDRB |= LASER_PIN;
}

static void laser_off(void) {
	DDRB &= ~(LASER_PIN);
}

void main(void)
{
	DDRB |= SERVO_PIN_A | SERVO_PIN_B;
	PORTB &= ~(LASER_PIN);
	laser_off();

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
			uint8_t tmp = ctl ^ REG_CONTROL;

			if (REG_SERVO_A_MIN < SERVO_MIN) {
				REG_SERVO_A_MIN = SERVO_MIN;
			}

			if (REG_SERVO_B_MIN < SERVO_MIN) {
				REG_SERVO_B_MIN = SERVO_MIN;
			}

			if (REG_SERVO_A_MAX < SERVO_MIN) {
				REG_SERVO_A_MAX = SERVO_MIN;
			}

			if (REG_SERVO_B_MAX < SERVO_MIN) {
				REG_SERVO_B_MAX = SERVO_MIN;
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

			if (tmp & (1 << 2)) {
				if (REG_CONTROL & (1 << 2)) {
					laser_on();
				} else {
					laser_off();
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
