#include "mini_mini_HAL.h"
#define LED 2

Mini_mini_HAL HAL;

void setup() {
  HAL.PinMode(LED, OUTPUT);

}

void loop() {
  HAL.DigitalWrite(LED, HIGH);
  delay(500);
  HAL.DigitalWrite(LED, LOW);
  delay(500);
}

