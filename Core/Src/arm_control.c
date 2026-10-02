/*
 * arm_control.c
 *
 *  Created on: 2026年10月01日
 *      Author: twyyd
 */

#include "arm_control.h"
#include "motor.h"
#include "servo_motor.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <sys/types.h>

void grab_ground() {
  send_motor_place_absolute(0, 150);
  HAL_Delay(500);
  servo_set_angle(SERVO_SG90, servo_place);
  servo_set_angle(SERVO_XH360, 277);
  HAL_Delay(1000);
  send_motor_place_absolute(1, 350);
  HAL_Delay(2000);
  servo_set_angle(SERVO_SG90, servo_grab);
  HAL_Delay(1000);
  send_motor_place_absolute(0, 345);
  HAL_Delay(1000);
}
void grab_plate(uint8_t id) {
  switch (id) {
  case 1: {
    send_motor_place_absolute(0, 205);
    HAL_Delay(1000);
    servo_set_angle(SERVO_SG90, servo_place);
    HAL_Delay(1000);
    servo_set_angle(SERVO_XH360, plate_one);
    HAL_Delay(1000);
    servo_set_angle(SERVO_XH270, 73);
    HAL_Delay(500);
    servo_set_angle(SERVO_SG90, servo_grab);
    HAL_Delay(1000);
    send_motor_place_absolute(0, 345);
    HAL_Delay(1000);
    break;
  }
  case 2: {
    send_motor_place_absolute(0, 205);
    HAL_Delay(1000);
    servo_set_angle(SERVO_SG90, servo_place);
    HAL_Delay(1000);
    servo_set_angle(SERVO_XH360, plate_two);
    HAL_Delay(1000);
    servo_set_angle(SERVO_XH270, 75);
    HAL_Delay(500);
    servo_set_angle(SERVO_SG90, servo_grab);
    HAL_Delay(1000);
    send_motor_place_absolute(0, 345);
    HAL_Delay(1000);
    break;
  }
  case 3: {
    send_motor_place_absolute(0, 205);
    HAL_Delay(1000);
    servo_set_angle(SERVO_SG90, servo_place);
    HAL_Delay(1000);
    servo_set_angle(SERVO_XH360, plate_thr);
    HAL_Delay(1000);
    servo_set_angle(SERVO_XH270, 73);
    HAL_Delay(500);
    servo_set_angle(SERVO_SG90, servo_grab);
    HAL_Delay(1000);
    send_motor_place_absolute(0, 345);
    HAL_Delay(1000);
    break;
  }
  }
}
void place_plate(uint8_t id) {
  switch (id) {
  case 1: {
    send_motor_place_absolute(0, 345);
    HAL_Delay(600);
    servo_set_angle(SERVO_XH360, plate_one);
    HAL_Delay(1000);
    servo_set_angle(SERVO_XH270, 73);
    HAL_Delay(500);
    send_motor_place_absolute(0, 290);
    HAL_Delay(1000);
    servo_set_angle(SERVO_SG90, servo_place);
    HAL_Delay(1000);
    break;
  }
  case 2: {
    send_motor_place_absolute(0, 345);
    HAL_Delay(600);
    servo_set_angle(SERVO_XH360, plate_two);
    HAL_Delay(1000);
    servo_set_angle(SERVO_XH270, 78);
    HAL_Delay(500);
    send_motor_place_absolute(0, 290);
    HAL_Delay(1000);
    servo_set_angle(SERVO_SG90, servo_place);
    HAL_Delay(1000);
    break;
  }
  case 3: {
    send_motor_place_absolute(0, 345);
    HAL_Delay(600);
    servo_set_angle(SERVO_XH360, plate_thr);
    HAL_Delay(1000);
    servo_set_angle(SERVO_XH270, 73);
    HAL_Delay(500);
    send_motor_place_absolute(0, 290);
    HAL_Delay(1000);
    servo_set_angle(SERVO_SG90, servo_place);
    HAL_Delay(1000);
    break;
  }
  }
}
void place_ground() {
  send_motor_place_absolute(0, 345);
  HAL_Delay(1000);
  servo_set_angle(SERVO_XH360, 277);
  HAL_Delay(1000);
  send_motor_place_absolute(1, 335);
  HAL_Delay(2000);
  servo_set_angle(SERVO_SG90, servo_place);
  HAL_Delay(500);
  send_motor_place_absolute(0, 150);
  HAL_Delay(1000);
}
void place_block() {
  send_motor_place_absolute(0, 345);
  HAL_Delay(1000);
  servo_set_angle(SERVO_XH360, 277);
  HAL_Delay(1000);
  send_motor_place_absolute(1, 0);
  HAL_Delay(2000);
  servo_set_angle(SERVO_SG90, servo_place);
  HAL_Delay(1000);
  send_motor_place_absolute(0, 150);
  HAL_Delay(500);
}
bool to_yolo(uint8_t id) {
  send_motor_place_absolute(0, 345);
  HAL_Delay(1000);
  servo_set_angle(SERVO_XH360, 277);
  HAL_Delay(1000);
  servo_set_angle(SERVO_XH270, 73);
  HAL_Delay(500);
  servo_set_angle(SERVO_SG90, servo_place);
  HAL_Delay(500);
  send_motor_place_absolute(0, 150);
  HAL_Delay(1000);
  Lub_Cat_send_yolo(id);
  if (cat_centre_calibrate()) {
    // if (current_r == 180)
    //   set_cur_pos(180, 180, 1050);
    HAL_UART_Transmit(&huart1, (uint8_t *)"对齐", sizeof("对齐") - 1,
                      HAL_MAX_DELAY);
    return true;
  }
  return false;
}
bool to_ring() {
  servo_set_angle(SERVO_SG90, servo_place);
  send_motor_place_absolute(0, 150);
  Lub_Cat_send_ring();
  if (cat_centre_calibrate()) {
    HAL_UART_Transmit(&huart1, (uint8_t *)"对齐", sizeof("对齐") - 1,
                      HAL_MAX_DELAY);
    return true;
  }
  return false;
}
bool to_material(uint8_t color) {
  send_motor_place_absolute(0, 345);
  HAL_Delay(1000);
  servo_set_angle(SERVO_XH360, 277);
  HAL_Delay(1000);
  servo_set_angle(SERVO_SG90, servo_place);
  HAL_Delay(500);
  send_motor_place_absolute(0, 150);
  HAL_Delay(1000);
  Lub_Cat_send_material(color);
  if (cat_centre_calibrate()) {
    HAL_UART_Transmit(&huart1, (uint8_t *)"对齐", sizeof("对齐") - 1,
                      HAL_MAX_DELAY);
    return true;
  }
  return false;
}