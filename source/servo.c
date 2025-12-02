#include "servo.h"

extern TIM_HandleTypeDef htim1;

void servo_init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    servo_lock();
    HAL_Delay(500);
}

void servo_set_angle(uint16_t angle)
{
    if(angle > 180) angle = 180;


    uint32_t pulse_width;

    pulse_width = 1000 + (angle * 1000) / 180;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_width);

    HAL_Delay(500);
}

void servo_unlock(void)
{
    servo_set_angle(90);
}

void servo_lock(void)
{
    servo_set_angle(0);
}


void servo_manual_control(uint32_t pulse_width)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_width);
}
