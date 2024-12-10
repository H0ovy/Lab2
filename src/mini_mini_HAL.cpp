#include "mini_mini_HAL.h"

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

void Mini_mini_HAL::DigitalWrite(Uint8_t pin, Uint8_t val)
{
    Uint8_t timer = digitalPinToTimer(pin);
	Uint8_t bit = digitalPinToBitMask(pin);
	Uint8_t port = digitalPinToPort(pin);
	volatile Uint8_t *out;

	if (port == NOT_A_PIN) return;

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if (timer != NOT_ON_TIMER) TurnOffPWM(timer);

	out = portOutputRegister(port);

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

// void Mini_mini_HAL::Delay(unsigned long ms)
// {
// 	uint32_t start = micros();

// 	while (ms > 0) {
// 		yield();
// 		while ( ms > 0 && (micros() - start) >= 1000) {
// 			ms--;
// 			start += 1000;
// 		}
// 	}
// }

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