/*
 * servo_motor.c
 *
 *  Created on: 2026年8月9日
 *      Author: twyyd
 */

#include "servo_motor.h"
#include "lub_cat.h"
#include "stm32f429xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/_intsup.h>

uint16_t sg90_old = 0;
uint16_t XH270_old = 0;
uint16_t XH360_old = 0;
float oldx_sum = 0;
float oldy_sum = 0;

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
    if (angle < 5) {
      angle = 5;
    }
    TIM4->CCR1 = angle_to_pulse(angle, sg90_min, sg90_max);
    break;
  case SERVO_XH270:
    if (angle > 125)
      angle = 125;
    if (angle < 5) {
      angle = 5;
    }
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
    if (angle < 30) {
      angle = 30;
    }
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
void servo_set_angle_quk(Servo_ID id, uint16_t angle) {
  switch (id) {
  case SERVO_SG90:
    if (angle > 70)
      angle = 70;
    if (angle < 5) {
      angle = 5;
    }
    TIM4->CCR1 = angle_to_pulse(angle, sg90_min, sg90_max);
    break;
  case SERVO_XH270:
    if (angle > 125)
      angle = 125;
    if (angle < 5) {
      angle = 5;
    }
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
      HAL_Delay(1);
    }

    XH270_old = (uint16_t)target;
    break;
  case SERVO_XH360: {
    if (angle > 340)
      angle = 340;
    if (angle < 30) {
      angle = 30;
    }
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
      HAL_Delay(1);
    }

    XH360_old = (uint16_t)target;
    break;
  }
  }
}
void servo_set_angle_pid(Servo_ID id, uint16_t angle) {
  switch (id) {
  case SERVO_SG90:
    if (angle > 70)
      angle = 70;
    if (angle < 5) {
      angle = 5;
    }
    TIM4->CCR1 = angle_to_pulse(angle, sg90_min, sg90_max);
    break;
  case SERVO_XH270:
    if (angle > 125)
      angle = 125;
    if (angle < 5) {
      angle = 5;
    }
    int target = angle;
    TIM9->CCR1 = angle_to_pulse(angle, XH270_min, XH270_max);
    XH270_old = (uint16_t)target;
    break;
  case SERVO_XH360: {
    if (angle > 340)
      angle = 340;
    if (angle < 30) {
      angle = 30;
    }
    int target = angle;
    TIM12->CCR1 = angle_to_pulse(angle, XH360_min, XH360_max);
    XH360_old = (uint16_t)target;
    break;
  }
  }
}
void pid_armservo(int16_t px, int16_t py) {
  int16_t center_angle = XH360_old - 277;
  oldx_sum += px;
  oldy_sum += py;
  px = sin(center_angle) * py + cos(center_angle) * px;
  py = cos(center_angle) * py - sin(center_angle) * px;
  if ((px < 0 && oldx_sum > 0) || (px > 0 && oldx_sum < 0)) {
    oldx_sum = 0;
  }
  if ((py < 0 && oldy_sum > 0) || (py > 0 && oldy_sum < 0)) {
    oldy_sum = 0;
  }
  int16_t angle_3servo = 0.05 * px + 0.0001 * oldx_sum;
  int16_t angle_2servo = 0.05 * py + 0.0001 * oldy_sum;
  angle_3servo = angle_3servo > 5 ? 5 : angle_3servo < -5 ? -5 : angle_3servo;
  angle_2servo = angle_2servo > 3 ? 3 : angle_2servo < -3 ? -3 : angle_2servo;
  int a = XH360_old - angle_3servo;
  int b = XH270_old - angle_2servo;
  servo_set_angle_pid(SERVO_XH360, a);
  servo_set_angle_pid(SERVO_XH270, b);
}