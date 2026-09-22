/*
 * servo_motor.c
 *
 *  Created on: 2026年8月9日
 *      Author: twyyd
 */

#include "servo_motor.h"
#include "stm32f429xx.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"

void servo_init() {
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
}
void servo_stop() {
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 500);
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 500);
  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 500);
  HAL_Delay(1);
  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
}
static uint32_t angle_to_pulse(uint16_t angle, uint16_t min, uint16_t max) {
  angle = (angle > max) ? max : (angle < min) ? min : angle;
  uint32_t pulse;
  pulse = (float)(angle - min) / (float)(max - min) * (servo_max - servo_min) +
          servo_min;
  return pulse;
}
void servo_set_angle(Servo_ID id, uint16_t angle) {
  switch (id) {
  case SERVO_SG90:
    TIM8->CCR3 = angle_to_pulse(angle, sg90_min, sg90_max);
    break;
  case SERVO_XH270:
    TIM9->CCR1 = angle_to_pulse(angle, XH270_min, XH270_max);
    break;
  case SERVO_XH360:
    TIM12->CCR1 = angle_to_pulse(angle, XH360_min, XH360_max);
    break;
  }
}
