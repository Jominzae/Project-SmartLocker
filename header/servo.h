
#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx_hal.h"
void servo_init(void);
void servo_set_angle(uint16_t angle);
void servo_unlock(void);
void servo_lock(void);

#endif
