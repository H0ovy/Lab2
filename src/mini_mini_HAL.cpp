#include "mini_mini_HAL.h"

Mini_mini_HAL::Mini_mini_HAL()
{

}

void Mini_mini_HAL::PinMode(Uint8_t pin, Uint8_t mode)
{
	Uint8_t bit = digitalPinToBitMask(pin);
	Uint8_t port = digitalPinToPort(pin);
	volatile Uint8_t *reg, *out;

	if (port == NOT_A_PIN) return;

	reg = portModeRegister(port);
	out = portOutputRegister(port);

	if (mode == INPUT) 
    { 
		Uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out &= ~bit;
		SREG = oldSREG;
	} 
    else if (mode == INPUT_PULLUP) 
    {
		Uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out |= bit;
		SREG = oldSREG;
	} 
    else 
    {
		Uint8_t oldSREG = SREG;
                cli();
		*reg |= bit;
		SREG = oldSREG;
	}
}

#define _LPM_word_enhanced__(addr)         \
(__extension__({                            \
    uint16_t __addr16 = (uint16_t)(addr);   \
    uint16_t __result;                      \
    __asm__ __volatile__                    \
    (                                       \
        "lpm %A0, Z+"   "\n\t"              \
        "lpm %B0, Z"    "\n\t"              \
        : "=r" (__result), "=z" (__addr16)  \
        : "1" (__addr16)                    \
    );                                      \
    __result;                               \
}))
const Uint16_t PROGMEM Port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

volatile Uint8_t * Mini_mini_HAL::PortOutputRegister(Uint8_t P)
{
	return (volatile Uint8_t *)( _LPM_word_enhanced__( Port_to_output_PGM + (P)));
}

const Uint8_t PROGMEM Digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER, /* 0 - port D */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	// on the ATmega168, digital pin 3 has hardware pwm
#if defined(__AVR_ATmega8__)
	NOT_ON_TIMER,
#else
	TIMER2B,
#endif
	NOT_ON_TIMER,
	// on the ATmega168, digital pins 5 and 6 have hardware pwm
#if defined(__AVR_ATmega8__)
	NOT_ON_TIMER,
	NOT_ON_TIMER,
#else
	TIMER0B,
	TIMER0A,
#endif
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 8 - port B */
	TIMER1A,
	TIMER1B,
#if defined(__AVR_ATmega8__)
	TIMER2,
#else
	TIMER2A,
#endif
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 14 - port C */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
};



#define _LPM_enhanced__(addr)  \
(__extension__({                \
    uint16_t __addr16 = (uint16_t)(addr); \
    uint8_t __result;           \
    __asm__ __volatile__        \
    (                           \
        "lpm %0, Z" "\n\t"      \
        : "=r" (__result)       \
        : "z" (__addr16)        \
    );                          \
    __result;                   \
}))

Uint8_t Mini_mini_HAL::DigitalPinToTimer(Uint8_t P)
{
	return _LPM_enhanced__(Digital_pin_to_timer_PGM + (P));
}

const Uint8_t PROGMEM Digital_pin_to_bit_mask_PGM[] = {
	_BV(0), /* 0, port D */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0), /* 8, port B */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(0), /* 14, port C */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
};

Uint8_t Mini_mini_HAL::DigitalPinToBitMask(Uint8_t P)
{
	return _LPM_enhanced__(Digital_pin_to_bit_mask_PGM + (P));
}

const Uint8_t PROGMEM Digital_pin_to_port_PGM[] = {
        4, /* 0 */
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        2, /* 8 */
        2,
        2,
        2,
        2,
        2,
        3, /* 14 */
        3,
        3,
        3,
        3,
        3,
    };

Uint8_t Mini_mini_HAL::DigitalPinToPort(Uint8_t P)
{
	return _LPM_enhanced__(Digital_pin_to_port_PGM + (P));
}

void Mini_mini_HAL::DigitalWrite(Uint8_t pin, Uint8_t val)
{
    Uint8_t timer = DigitalPinToTimer(pin);
	Uint8_t bit = DigitalPinToBitMask(pin);
	Uint8_t port = DigitalPinToPort(pin);
	volatile Uint8_t *out;

	if (port == 0) 
		return;

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if (timer != 0) 
		TurnOffPWM(timer);

	out = PortOutputRegister(port);

	Uint8_t oldSREG = SREG;
	cli();

	if (val == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}

	SREG = oldSREG;
}

void Mini_mini_HAL::TurnOffPWM(Uint8_t timer)
{
	switch (timer)
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:   cbi(TCCR1A, COM1C1);    break;
		#endif
		
		#if defined(TCCR2) && defined(COM21)
		case  TIMER2:   cbi(TCCR2, COM21);      break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0A1)
		case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0B1)
		case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2A1)
		case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2B1)
		case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
		#endif
		
		#if defined(TCCR3A) && defined(COM3A1)
		case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3C1)
		case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
		#endif

		#if defined(TCCR4A) && defined(COM4A1)
		case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
		#endif					
		#if defined(TCCR4A) && defined(COM4B1)
		case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
		#endif
		#if defined(TCCR4A) && defined(COM4C1)
		case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
		#endif			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:	cbi(TCCR4C, COM4D1);	break;
		#endif			
			
		#if defined(TCCR5A)
		case  TIMER5A:  cbi(TCCR5A, COM5A1);    break;
		case  TIMER5B:  cbi(TCCR5A, COM5B1);    break;
		case  TIMER5C:  cbi(TCCR5A, COM5C1);    break;
		#endif
	}
}


int Mini_mini_HAL::DigitalRead(Uint8_t pin)
{
	Uint8_t timer = digitalPinToTimer(pin);
	Uint8_t bit = digitalPinToBitMask(pin);
	Uint8_t port = digitalPinToPort(pin);

	if (port == NOT_A_PIN) return LOW;

	// If the pin that support PWM output, we need to turn it off
	// before getting a digital reading.
	if (timer != NOT_ON_TIMER) TurnOffPWM(timer);

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}