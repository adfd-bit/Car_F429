/*
 * OPS9.h
 *
 *  Created on: 2026年6月21日
 *      Author: twyyd
 */

#ifndef INC_OPS9_H_
#define INC_OPS9_H_

#include "stdbool.h"
#include "string.h"
#include "usart.h"

#define OPS9_data_len 30 /* OPS9 接收缓冲区大小（28字节数据帧 + 2字节余量） */
extern uint8_t OPS_redata[OPS9_data_len];

extern volatile float
    OPS_angle; /* 定义在 main.c，另见 motor.h:31，两处须逐字一致 */
extern volatile float
    OPS_X; /* 定义在 main.c，另见 motor.h:32，两处须逐字一致 */
extern volatile float
    OPS_Y; /* 定义在 main.c，另见 motor.h:33，两处须逐字一致 */
extern volatile float current_x;
extern volatile float current_y;
extern volatile float
    current_r; /* 定义在 motor.c，另见 lub_cat.h:49，两处须逐字一致 */
extern uint32_t ALL_time; /* 定义在 main.c，另见 motor.h:29，两处须逐字一致 */

void ops9_receive_start();
void ops9_receive_stop();

void set_cur_pos_x(float x);
void set_cur_pos_y(float y);
void set_cur_pos_angle(float angle);
void set_cur_pos(float angle, float x, float y);

#endif /* INC_OPS9_H_ */
