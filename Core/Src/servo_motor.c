/*
 * servo_motor.c
 *
 *  Created on: 2026年8月9日
 *      Author: twyyd
 */

#include "servo_motor.h"
#include "stm32f429xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include <stdint.h>
#include <stdlib.h>
#include <sys/_intsup.h>

uint16_t sg90_old = 0;
uint16_t XH270_old = 0;
uint16_t XH360_old = 0;

void servo_init() {
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
}
void servo_stop() {
  // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 500);
  // __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 500);
  // __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 500);
  // HAL_Delay(1);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
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
    if (angle > 70)
      angle = 70;
    TIM4->CCR1 = angle_to_pulse(angle, sg90_min, sg90_max);
    break;
  case SERVO_XH270:
    if (angle > 125)
      angle = 125;
    int old = XH270_old;
    int target = angle;
    if (old == 0) {
      TIM9->CCR1 = angle_to_pulse(angle, XH270_min, XH270_max);
      XH270_old = target;
      break;
    }

    if (old == target) {
      break;
    }

    int step = (target > old) ? 1 : -1;
    int steps = abs(target - old);

    for (int i = 0; i < steps; i++) {
      old += step;
      TIM9->CCR1 = angle_to_pulse((uint16_t)old, XH270_min, XH270_max);
      HAL_Delay(8);
    }

    XH270_old = (uint16_t)target;
    break;
  case SERVO_XH360: {
    if (angle > 340)
      angle = 340;
    int old = XH360_old;
    int target = angle;
    if (old == 0) {
      TIM12->CCR1 = angle_to_pulse(angle, XH360_min, XH360_max);
      XH360_old = target;
      break;
    }

    if (old == target) {
      break;
    }

    int step = (target > old) ? 1 : -1;
    int steps = abs(target - old);

    for (int i = 0; i < steps; i++) {
      old += step;
      TIM12->CCR1 = angle_to_pulse((uint16_t)old, XH360_min, XH360_max);
      HAL_Delay(8);
    }

    XH360_old = (uint16_t)target;
    break;
  }
  }
}
