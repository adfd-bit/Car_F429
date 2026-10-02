/*
 * arm_control.h
 *
 *  Created on: 2026年10月01日
 *      Author: twyyd
 */

#ifndef INC_ARM_CONTROL_H_
#define INC_ARM_CONTROL_H_

#include "lub_cat.h"
#include "motor.h"
#include "ops9.h"
#include "servo_motor.h"
#include <stdbool.h>
#include <stdint.h>


#define servo_place 5
#define servo_grab 65
#define plate_one 115
#define plate_two 88
#define plate_thr 58

void grab_ground();
void grab_plate(uint8_t id);
void place_plate(uint8_t id);
void place_ground();
void place_block();
bool to_yolo(uint8_t id);
bool to_ring();
bool to_material(uint8_t color);

#endif /* INC_ARM_CONTROL_H_ */
