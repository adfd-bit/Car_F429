/*
 * arm_control.c
 *
 *  Created on: 2026年10月01日
 *      Author: twyyd
 */

#include "arm_control.h"
#include "lub_cat.h"
#include "motor.h"
#include "servo_motor.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <sys/types.h>
void arm_free() {
  send_motor_place_absolute(motor_up, arm_ullimit);
  servo_set_angle(SERVO_SG90, servo_place);
  HAL_Delay(500);
  servo_set_angle(SERVO_XH270, grab2_2servo);
  servo_set_angle(SERVO_XH360, plate_two);
  HAL_Delay(500);
}
void grab_platform() {
  send_motor_place_absolute(motor_up, grab_platform_motor);
  HAL_Delay(200);
  servo_set_angle(SERVO_SG90, servo_grab);
  HAL_Delay(500);
  send_motor_place_absolute(motor_up, arm_ullimit);
  HAL_Delay(600);
}
void grab_ground() {
  send_motor_place_absolute(motor_up, yolo_motor);
  HAL_Delay(500);
  servo_set_angle(SERVO_SG90, servo_place);
  servo_set_angle_quk(SERVO_XH360, yolo_3servo);
  HAL_Delay(1000);
  send_motor_place_absolute(motor_down, arm_dwlimit);
  HAL_Delay(1000);
  servo_set_angle(SERVO_SG90, servo_grab);
  HAL_Delay(800);
  send_motor_place_absolute(motor_up, arm_ullimit);
  HAL_Delay(1000);
}
void grab_ground_id(uint8_t id) {
  switch (id) {
  case 1: {
    servo_set_angle(SERVO_SG90, servo_place);
    servo_set_angle_quk(SERVO_XH360, place1_3servo);
    servo_set_angle(SERVO_XH270, place1_2servo);
    HAL_Delay(600);
    send_motor_place_absolute(motor_down, arm_dwlimit);
    HAL_Delay(1000);
    servo_set_angle(SERVO_SG90, servo_grab);
    HAL_Delay(800);
    send_motor_place_absolute(motor_up, arm_ullimit);
    HAL_Delay(1000);
    break;
  }
  case 2: {
    servo_set_angle(SERVO_SG90, servo_place);
    servo_set_angle_quk(SERVO_XH360, yolo_3servo);
    servo_set_angle(SERVO_XH270, yolo_2servo);
    HAL_Delay(600);
    send_motor_place_absolute(motor_down, arm_dwlimit);
    HAL_Delay(1000);
    servo_set_angle(SERVO_SG90, servo_grab);
    HAL_Delay(800);
    send_motor_place_absolute(motor_up, arm_ullimit);
    HAL_Delay(1000);
    break;
  }
  case 3: {
    servo_set_angle(SERVO_SG90, servo_place);
    servo_set_angle_quk(SERVO_XH360, place3_3servo);
    servo_set_angle(SERVO_XH270, place3_2servo);
    HAL_Delay(600);
    send_motor_place_absolute(motor_down, arm_dwlimit);
    HAL_Delay(1000);
    servo_set_angle(SERVO_SG90, servo_grab);
    HAL_Delay(800);
    send_motor_place_absolute(motor_up, arm_ullimit);
    HAL_Delay(1000);
    break;
  }
  }
}
void grab_plate(uint8_t id, uint8_t mod) {
  switch (id) {
  case 1: {
    send_motor_place_absolute(motor_up, grab_motor);
    HAL_Delay(300);
    servo_set_angle(SERVO_SG90, servo_place);
    servo_set_angle(SERVO_XH270, grab13_2servo);
    HAL_Delay(500);
    servo_set_angle_quk(SERVO_XH360, plate_one);
    HAL_Delay(700);
    servo_set_angle(SERVO_SG90, servo_grab);
    HAL_Delay(500);
    send_motor_place_absolute(motor_up, arm_ullimit);
    HAL_Delay(500);
    if (mod == servo2_savemod) {
      servo_set_angle(SERVO_XH270, grab_save_2servo);
      HAL_Delay(300);
    }

    break;
  }
  case 2: {
    send_motor_place_absolute(motor_up, grab_motor);
    HAL_Delay(300);
    servo_set_angle(SERVO_SG90, servo_place);
    servo_set_angle(SERVO_XH270, grab2_2servo);
    HAL_Delay(500);
    servo_set_angle_quk(SERVO_XH360, plate_two);
    HAL_Delay(700);
    servo_set_angle(SERVO_SG90, servo_grab);
    HAL_Delay(500);
    send_motor_place_absolute(motor_up, arm_ullimit);
    HAL_Delay(500);
    if (mod == servo2_savemod) {
      servo_set_angle(SERVO_XH270, grab_save_2servo);
      HAL_Delay(300);
    }
    break;
  }
  case 3: {
    send_motor_place_absolute(motor_up, grab_motor);
    HAL_Delay(300);
    servo_set_angle(SERVO_SG90, servo_place);
    servo_set_angle(SERVO_XH270, grab13_2servo);
    HAL_Delay(500);
    servo_set_angle_quk(SERVO_XH360, plate_thr);
    HAL_Delay(700);
    servo_set_angle(SERVO_SG90, servo_grab);
    HAL_Delay(500);
    send_motor_place_absolute(motor_up, arm_ullimit);
    HAL_Delay(500);
    if (mod == servo2_savemod) {
      servo_set_angle(SERVO_XH270, grab_save_2servo);
      HAL_Delay(300);
    }
    break;
  }
  }
}
void place_plate(uint8_t id) {
  switch (id) {
  case 1: {
    servo_set_angle(SERVO_XH270, grab13_2servo);
    servo_set_angle(SERVO_XH360, plate_one);
    HAL_Delay(500);
    // send_motor_place_absolute(motor_up, 290);
    // HAL_Delay(1000);
    servo_set_angle(SERVO_SG90, servo_place);
    HAL_Delay(300);
    break;
  }
  case 2: {
    servo_set_angle(SERVO_XH270, grab2_2servo);
    servo_set_angle(SERVO_XH360, plate_two);
    HAL_Delay(500);
    // send_motor_place_absolute(motor_up, 290);
    // HAL_Delay(1000);
    servo_set_angle(SERVO_SG90, servo_place);
    HAL_Delay(300);
    break;
  }
  case 3: {
    servo_set_angle(SERVO_XH270, grab13_2servo);
    servo_set_angle(SERVO_XH360, plate_thr);
    HAL_Delay(500);
    // send_motor_place_absolute(motor_up, 290);
    // HAL_Delay(1000);
    servo_set_angle(SERVO_SG90, servo_place);
    HAL_Delay(300);
    break;
  }
  }
}
void place_ground(uint8_t id) {
  switch (id) {
  case 1: {
    servo_set_angle(SERVO_XH360, place1_3servo);
    HAL_Delay(200);
    servo_set_angle(SERVO_XH270, place1_2servo);
    send_motor_place_absolute(motor_down, arm_place);
    HAL_Delay(800);
    servo_set_angle(SERVO_SG90, servo_place);
    HAL_Delay(500);
    // send_motor_place_absolute(motor_up, yolo_motor);
    // HAL_Delay(800);
    break;
  }
  case 2: {
    servo_set_angle(SERVO_XH360, yolo_3servo);
    HAL_Delay(300);
    servo_set_angle(SERVO_XH270, yolo_2servo);
    send_motor_place_absolute(motor_down, arm_place);
    HAL_Delay(800);
    servo_set_angle(SERVO_SG90, servo_place);
    HAL_Delay(500);
    // send_motor_place_absolute(motor_up, yolo_motor);
    // HAL_Delay(800);
    break;
  }
  case 3: {
    servo_set_angle(SERVO_XH360, place3_3servo);
    HAL_Delay(300);
    servo_set_angle(SERVO_XH270, place3_2servo);
    send_motor_place_absolute(motor_down, arm_place);
    HAL_Delay(800);
    servo_set_angle(SERVO_SG90, servo_place);
    HAL_Delay(500);
    // send_motor_place_absolute(motor_up, yolo_motor);
    // HAL_Delay(800);
    break;
  }
  }
}
void place_block(uint8_t id) {
  switch (id) {
  case 1: {
    servo_set_angle(SERVO_XH360, place1_3servo);
    HAL_Delay(500);
    servo_set_angle(SERVO_XH270, place1_2servo);
    send_motor_place_absolute(motor_down, place_stack);
    HAL_Delay(800);
    servo_set_angle(SERVO_SG90, servo_place);
    HAL_Delay(500);
    send_motor_place_absolute(motor_up, yolo_motor);
    HAL_Delay(500);
    break;
  }
  case 2: {

    servo_set_angle(SERVO_XH360, yolo_3servo);
    HAL_Delay(500);
    servo_set_angle(SERVO_XH270, yolo_2servo);
    send_motor_place_absolute(motor_down, place_stack);
    HAL_Delay(800);
    servo_set_angle(SERVO_SG90, servo_place);
    HAL_Delay(500);
    send_motor_place_absolute(motor_up, yolo_motor);
    HAL_Delay(500);
    break;
  }
  case 3: {

    servo_set_angle(SERVO_XH270, grab_save_2servo);
    HAL_Delay(100);
    servo_set_angle(SERVO_XH360, place3_3servo);
    HAL_Delay(500);
    servo_set_angle(SERVO_XH270, place3_2servo);
    send_motor_place_absolute(motor_down, place_stack);
    HAL_Delay(800);
    servo_set_angle(SERVO_SG90, servo_place);
    HAL_Delay(500);
    send_motor_place_absolute(motor_up, yolo_motor);
    HAL_Delay(500);
    break;
  }
  }
}
bool to_yolo(CAT_Yolo_t id) {
  servo_set_angle(SERVO_SG90, servo_place);
  HAL_Delay(600);
  servo_set_angle_quk(SERVO_XH360, yolo_3servo);
  HAL_Delay(1000);
  send_motor_place_absolute(motor_up, yolo_motor);
  servo_set_angle(SERVO_XH270, yolo_2servo);
  HAL_Delay(500);
  Lub_Cat_send_yolo(id);
  refresh_lubcat();
  if (cat_centre_calibrate()) {
    // if (current_r == 180)
    //   set_cur_pos(180, 180, 1050);
    refresh_lubcat();
    HAL_UART_Transmit(&huart1, (uint8_t *)"对齐", sizeof("对齐") - 1,
                      HAL_MAX_DELAY);
    return true;
  }
  return false;
}
bool to_ring() {
  servo_set_angle(SERVO_SG90, servo_place);
  send_motor_place_absolute(motor_up, yolo_motor);
  Lub_Cat_send_ring();
  if (cat_centre_calibrate()) {
    refresh_lubcat();
    HAL_UART_Transmit(&huart1, (uint8_t *)"对齐", sizeof("对齐") - 1,
                      HAL_MAX_DELAY);
    return true;
  }
  return false;
}
bool to_material(CAT_Color_t color) {
  motor_stop();
  servo_set_angle(SERVO_SG90, servo_place);
  HAL_Delay(500);
  servo_set_angle(SERVO_XH360, yolo_3servo);
  HAL_Delay(300);
  servo_set_angle(SERVO_XH270, yolo_2servo);
  send_motor_place_absolute(motor_up, yolo_motor);
  HAL_Delay(800);
  refresh_lubcat();
  Lub_Cat_send_material(color);
  refresh_lubcat();
  if (cat_centre_calibrate()) {
    refresh_lubcat();
    HAL_UART_Transmit(&huart1, (uint8_t *)"对齐", sizeof("对齐") - 1,
                      HAL_MAX_DELAY);
    return true;
  }
  return false;
}
bool to_material_platform(CAT_Color_t color) {
  motor_stop();
  servo_set_angle(SERVO_SG90, servo_place);
  HAL_Delay(500);
  servo_set_angle_quk(SERVO_XH360, yolo_3servo);
  HAL_Delay(300);
  servo_set_angle(SERVO_XH270, yolo_platform_2servo);
  send_motor_place_absolute(motor_up, arm_ullimit);
  HAL_Delay(800);
  refresh_lubcat();
  Lub_Cat_send_material(color);
  refresh_lubcat();
  if (cat_centre_calibrate()) {
    refresh_lubcat();
    HAL_UART_Transmit(&huart1, (uint8_t *)"对齐", sizeof("对齐") - 1,
                      HAL_MAX_DELAY);
    return true;
  }
  return false;
}