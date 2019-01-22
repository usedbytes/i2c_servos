#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

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
	switch (servo) {
	case SERVO_A:
		OCR1A = pos;
		break;
	case SERVO_B:
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

void main()
{
	DDRB |= (1 << 3);

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

	sei();

	servo_set(SERVO_A, 128);
	servo_enable(SERVO_A);
	for (;;) {
		servo_set(SERVO_A, 50);
		_delay_ms(1000);
		servo_set(SERVO_A, 200);
		_delay_ms(1000);
	};
}
