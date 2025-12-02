#ifndef KEYPAD_H__
#define KEYPAD_H__

#include "stm32f4xx_hal.h"
#define R1_Pin GPIO_PIN_0   // PA0
#define R2_Pin GPIO_PIN_1   // PA1
#define R3_Pin GPIO_PIN_0   // PB0
#define R4_Pin GPIO_PIN_10  // PB10
#define C1_Pin GPIO_PIN_4   // PA4
#define C2_Pin GPIO_PIN_5   // PA5
#define C3_Pin GPIO_PIN_6   // PA6
#define C4_Pin GPIO_PIN_7   // PA7
// ?? ??
char keypad_get_key(void);
void keypad_init(void);

#endif // KEYPAD_H__
