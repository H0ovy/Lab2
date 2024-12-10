#ifndef MINI_MINI_HAL_H
#define MINI_MINI_HAL_H

#include <Arduino.h>
// #include "wiring_private.h"
// #include "pins_arduino.h"

typedef signed int Int8_t __attribute__((__mode__(__QI__)));
typedef unsigned int Uint8_t __attribute__((__mode__(__QI__)));
typedef signed int Int16_t __attribute__ ((__mode__ (__HI__)));
typedef unsigned int Uint16_t __attribute__ ((__mode__ (__HI__)));
typedef signed int Int32_t __attribute__ ((__mode__ (__SI__)));
typedef unsigned int Uint32_t __attribute__ ((__mode__ (__SI__)));

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))

// #define _MMIO_BYTE(mem_addr) (*(volatile uint8_t *)(mem_addr))

// #define HIGH 0x1
// #define LOW  0x0

// #define INPUT 0x0
// #define OUTPUT 0x1
// #define INPUT_PULLUP 0x2

// void setup(void);
// void loop(void);

// #define __SFR_OFFSET 0x20

// #define _SFR_IO8(io_addr) _MMIO_BYTE((io_addr) + __SFR_OFFSET)

// #define SREG _SFR_IO8(0x3F)

// #define cli()  __asm__ __volatile__ ("cli" ::: "memory")

// #define NOT_A_PIN 0
// #define NOT_A_PORT 0
// #define NOT_ON_TIMER 0


// #define __LPM_word_enhanced__(addr)         \
// (__extension__({                            \
//     uint16_t __addr16 = (uint16_t)(addr);   \
//     uint16_t __result;                      \
//     __asm__ __volatile__                    \
//     (                                       \
//         "lpm %A0, Z+"   "\n\t"              \
//         "lpm %B0, Z"    "\n\t"              \
//         : "=r" (__result), "=z" (__addr16)  \
//         : "1" (__addr16)                    \
//     );                                      \
//     __result;                               \
// }))
// #define __LPM_enhanced__(addr)  \
// (__extension__({                \
//     uint16_t __addr16 = (uint16_t)(addr); \
//     uint8_t __result;           \
//     __asm__ __volatile__        \
//     (                           \
//         "lpm %0, Z" "\n\t"      \
//         : "=r" (__result)       \
//         : "z" (__addr16)        \
//     );                          \
//     __result;                   \
// }))
// #define __LPM_word(addr)    __LPM_word_enhanced__(addr)
// #define __LPM(addr)         __LPM_enhanced__(addr)
// #define pgm_read_byte_near(address_short) __LPM((uint16_t)(address_short))
// #define pgm_read_word_near(address_short) __LPM_word((uint16_t)(address_short))
// #define pgm_read_byte(address_short)    pgm_read_byte_near(address_short)
// #define pgm_read_word(address_short)    pgm_read_word_near(address_short)


// // #define PORTB   _SFR_IO8(0x18)
// // #define PORTC  _SFR_IO8(0x08)
// // #define PORTD   _SFR_IO8(0x0B)

// // #define DDRB      _SFR_IO8(0x17)
// // #define DDRC    _SFR_IO8(0x07)
// // #define DDRD    _SFR_IO8(0x0A)

// #define __ATTR_PROGMEM__ __attribute__((__progmem__))

// #define PROGMEM __ATTR_PROGMEM__

// extern const uint16_t PROGMEM port_to_mode_PGM[];
// extern const uint16_t PROGMEM port_to_input_PGM[];
// extern const uint16_t PROGMEM port_to_output_PGM[];

// extern const uint8_t PROGMEM digital_pin_to_port_PGM[];
// // extern const uint8_t PROGMEM digital_pin_to_bit_PGM[];
// extern const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[];
// extern const uint8_t PROGMEM digital_pin_to_timer_PGM[];

// #define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
// #define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
// #define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )
// #define analogInPinToBit(P) (P)
// #define portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )
// #define portInputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (P))) )
// #define portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )

// volatile unsigned long timer0_overflow_count = 0;

// #define TCNT0 _SFR_IO8(0x26)
// #define TIFR0 _SFR_IO8(0x15)

// #define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )

// #define _BV(bit) (1 << (bit))
// #define TOV0 0

// void yield(void);

// unsigned long micros() {
// 	unsigned long m;
// 	uint8_t oldSREG = SREG, t;
	
// 	cli();
// 	m = timer0_overflow_count;
// #if defined(TCNT0)
// 	t = TCNT0;
// #elif defined(TCNT0L)
// 	t = TCNT0L;
// #else
// 	#error TIMER 0 not defined
// #endif

// #ifdef TIFR0
// 	if ((TIFR0 & _BV(TOV0)) && (t < 255))
// 		m++;
// #else
// 	if ((TIFR & _BV(TOV0)) && (t < 255))
// 		m++;
// #endif

// 	SREG = oldSREG;
	
// 	return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
// }

class Mini_mini_HAL
{
public:
    void DigitalWrite(uint8_t pin, uint8_t val);
    void PinMode(uint8_t pin, uint8_t mode);
    int DigitalRead(uint8_t pin);
    static void TurnOffPWM(uint8_t timer);
    void Delay(unsigned long ms);

private:
    

};

#endif