#ifndef MINI_MINI_HAL_H
#define MINI_MINI_HAL_H

#include <Arduino.h>

typedef signed int Int8_t __attribute__((__mode__(__QI__)));
typedef unsigned int Uint8_t __attribute__((__mode__(__QI__)));
typedef signed int Int16_t __attribute__ ((__mode__ (__HI__)));
typedef unsigned int Uint16_t __attribute__ ((__mode__ (__HI__)));
typedef signed int Int32_t __attribute__ ((__mode__ (__SI__)));
typedef unsigned int Uint32_t __attribute__ ((__mode__ (__SI__)));

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))

class Mini_mini_HAL
{
public:
    Mini_mini_HAL();
    void DigitalWrite(uint8_t pin, uint8_t val);
    void PinMode(uint8_t pin, uint8_t mode);
    int DigitalRead(uint8_t pin);
    static void TurnOffPWM(uint8_t timer);
    void Delay(unsigned long ms);
    volatile Uint8_t * PortOutputRegister(Uint8_t P);
    Uint8_t DigitalPinToTimer(Uint8_t P);
    Uint8_t DigitalPinToBitMask(Uint8_t P);
    Uint8_t DigitalPinToPort(Uint8_t P);

private:

};

#endif