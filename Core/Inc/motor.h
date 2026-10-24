/*
 * motor.h
 *
 *  Created on: 2026年6月21日
 *      Author: twyyd
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include "math.h"
#include "path_plan.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "tim.h"
#include "usart.h"

#define PI 3.14159265358979323846
#define ErrTol 10.0
#define ErrTol_r 2.0
#define kI_max 20000
#define KI_min -20000
#define mov_max 210
#define mov_min -210
#define rad_max 600
#define rad_min -600
#define va_max 80

/* 180° 自旋拆成两个 90°（实现在 motor.c 的 pid_turn_split_180）。
 * 原地一次转 180° 时起步扭矩要翻转，容易打滑丢定位，拆成两段更稳。 */
#define SPLIT_180_ERR_DEG   150.0f /* 角差超过这个值才当作"掉头" */
#define SPLIT_180_POS_TOL_MM 50.0f /* XY 目标在这个范围内算原地自旋 */
#define SPLIT_180_PAUSE_MS   500U  /* 两段之间的停顿，让车停稳 */

extern uint32_t ALL_time; /* 定义在 main.c，另见 OPS9.h:24，两处须逐字一致 */

extern volatile float
    OPS_angle;               /* 定义在 main.c，另见 OPS9.h:18，两处须逐字一致 */
extern volatile float OPS_X; /* 定义在 main.c，另见 OPS9.h:19，两处须逐字一致 */
extern volatile float OPS_Y; /* 定义在 main.c，另见 OPS9.h:20，两处须逐字一致 */
extern float angle_OLD;
extern float X_OLD;
extern float Y_OLD;
extern volatile bool send_data_state;
extern int path_length;
extern PathPoint path[25];
extern Node nodes[MAP_SIZE][MAP_SIZE];
static uint16_t arrive_time __attribute__((unused)) = 0;
extern volatile bool opsready;
bool Critical_judgment(float x, float y);
void send_motor_place_absolute(uint8_t dir, uint32_t pulse);
void send_motor_place_relative(uint8_t dir, uint32_t pulse);
void send_motor_speed(uint8_t id);
void motor_go();
void motor_enable(uint8_t id);
void motor_en();
void motor_p();
void motor_stop();
void Speed_Conversion(double *vx, double *vy);
void control_v(double vx, double offs_vx, double vy, double offs_vy, double wv,
               double offs_wv);
bool pid_to_x_y(float X_target, float Y_target);
bool pid_to_w(float angle_taget);
bool pid_to_v(float X_target, float Y_target, float angle_taget);
bool pid_to_goal(float X_target, float Y_target, float angle_taget);
bool pid_to_goal_relative(float X_target, float Y_target, float angle_taget);
bool pid_to_path(int sta_x, int sta_y, int goal_x, int goal_y);

void send_motor_place(uint8_t dir, uint32_t pulse);
#endif /* INC_MOTOR_H_ */
