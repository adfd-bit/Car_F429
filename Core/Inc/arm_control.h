/*
 * arm_control.h
 *
 *  Created on: 2026年10月01日
 *      Author: twyyd
 */

#ifndef INC_ARM_CONTROL_H_
#define INC_ARM_CONTROL_H_

#include "arm_control.h"
#include "lub_cat.h"
#include "motor.h"
#include "ops9.h"
#include "servo_motor.h"
#include <stdbool.h>
#include <stdint.h>

#define motor_up 0   // 机械臂上升
#define motor_down 1 // 机械臂下降
#define servo2_savemod 1

#define arm_ullimit 360 // 机械臂上极限位置
#define arm_dwlimit 380 // 机械臂下极限位置
#define arm_place 370   // 机械臂放置位置
#define place_stack 0   // 机械臂叠放位置

#define servo_place 5 // 爪子松
#define servo_grab 65 // 爪子紧

#define yolo_2servo 40          // yolo2 2舵机位置
#define yolo_3servo 277         //  yolo2 3舵机位置
#define yolo_motor 160          // yolo2 电机位置
#define yolo_platform_2servo 80 // 工作台2舵机位置

#define plate_one 115 // 1号盘位置
#define plate_two 89  // 2号盘位置
#define plate_thr 59  // 3号盘位置

#define place1_3servo 320   // 放1号3舵机位置
#define place1_2servo 122   // 放1号2舵机位置
#define place3_3servo 230   // 放3号3舵机位置
#define place3_2servo 88    // 放3号2舵机位置
#define grab_save_2servo 20 // 抓取安全位置

#define grab_platform_motor 240 // 抓工作台物料
#define grab2_2servo 82         // 抓2盘中物料
#define grab13_2servo 73        // 抓13盘中物料
#define grab_motor 215          // 抓取物料电机位置

extern uint8_t ar_material_order[6];
extern uint8_t ar_number_order[6];

void arm_free();
void grab_platform();
void grab_ground();
void grab_ground_id(uint8_t id);
void grab_plate(uint8_t id, uint8_t mod);
void place_plate(uint8_t id);
void place_ground(uint8_t id);
void place_block(uint8_t id);

bool to_yolo(CAT_Yolo_t id);
bool to_ring();
bool to_material(CAT_Color_t color);
bool to_material_platform(CAT_Color_t color);

#endif /* INC_ARM_CONTROL_H_ */
