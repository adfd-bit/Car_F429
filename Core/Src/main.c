/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "gpio.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OPS9.h"
#include "ar_screen.h"
#include "arm_control.h"
#include "cy_z.h"
#include "lub_cat.h"
#include "math.h"
#include "motor.h"
#include "path_plan.h"
#include "servo_motor.h"
#include "stdio.h"
#include "string.h"
#include <complex.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/_intsup.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef enum {
  STATE_INIT,
  STATE_NAV,
  STATE_STOP,
} SystemState_t;
//-----------------------------------全局变量----------------------------------*/
uint8_t OPS_redata[OPS9_data_len];
volatile float OPS_angle = 0.0f;
volatile float OPS_X = 0.0f;
volatile float OPS_Y = 0.0f;
uint8_t CAT_redata[CAT_data_len];
volatile int16_t CAT_x = 0;
volatile int16_t CAT_y = 0;
uint8_t ar_data[30];
uint8_t ar_material_order[6] = {};
uint8_t ar_number_order[6] = {};
uint8_t cyz_redata[16];
volatile float cyz_angle = 0;
uint8_t cyz_cmd[8] = {0xA5, 0x5A, 0x01, 0x02, 0x02, 0xF8, 0x91, 0x5A};
uint8_t cyz_clear[8] = {0xA5, 0x5A, 0x01, 0x01, 0x01, 0x30, 0x00, 0x5A};

/*----------------------------------------------调试变量(可删)----------------------------*/
uint8_t receive;
uint8_t pathl[4];
int pt[4];
bool pt_sta = false;
uint8_t hc_os[17];
int goal_x = 0, goal_y = 0, goal_w = 0;
volatile bool re_sta = false;
uint8_t car_debug[13];
uint8_t car_debug_dalen = 0;
bool car_debug_state = false;
char cd[20] = {};
/*-----------------------------------状态变量----------------------------------*/
uint32_t ALL_time = 0;
uint8_t posit_state = 0; // 状态机
uint8_t arm_state = 0;
volatile bool send_data_state = true;
volatile bool opsready = false;
volatile bool catready = false;
volatile bool ar_screen_sta =
    false; /* true = AR_Screen 接收数据完成标志，主循环用 */
volatile bool cyz_state = false;
SystemState_t currentState = STATE_INIT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*-----------------------------------电机控制=----------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim1) { // 发送数据至电机
    static int id = 0;
    id++;
    if (id == 5) {
      id = 0;
      motor_go();
      HAL_TIM_Base_Stop_IT(&htim1);
      __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
      __HAL_TIM_SET_COUNTER(&htim1, 0);
      send_data_state = true;
    } else
      send_motor_speed(id);
  } else if (htim == &htim3) {
    static uint8_t a = 0;
    a++;
    if (a > 13) {
      a = 0;
      HAL_TIM_Base_Stop_IT(htim);
      currentState = STATE_NAV;
    }
  }
}

/*-------------------------------------串口接收事件回调函数-------------------------*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  //  没有检测到 AF FA 91 CF FC
  if (huart == &huart4) { // 鲁班猫数据处理
    if (Size == 5) {
      if (CAT_redata[0] == 0xAF && CAT_redata[1] == 0xFA &&
          CAT_redata[2] == 0x91 && CAT_redata[3] == 0xCF &&
          CAT_redata[4] == 0xFC) {
      }
      catready = false;
    }
    if (CAT_redata[0] == 0xAF && CAT_redata[1] == 0xFA &&
        CAT_redata[2] == 0x0B) {
      if (CAT_redata[9] == 0xCF && CAT_redata[10] == 0xFC) {
        CAT_x = *(int16_t *)&CAT_redata[4];
        CAT_y = *(int16_t *)&CAT_redata[6];
        catready = true;
        // HAL_UART_Transmit(&huart4, (uint8_t *)"luban_ready",
        //                   sizeof("luban_ready") - 1, HAL_MAX_DELAY);
      }
    } else if (CAT_redata[0] == 0xAF && CAT_redata[1] == 0XFA &&
               CAT_redata[2] == 0x0C) {
      if (CAT_redata[10] == 0xCF && CAT_redata[11] == 0xFC) {
        CAT_x = *(int16_t *)&CAT_redata[5];
        CAT_y = *(int16_t *)&CAT_redata[7];
        catready = true;
        // HAL_UART_Transmit(&huart4, (uint8_t *)"luban_ready",
        //                   sizeof("luban_ready") - 1, HAL_MAX_DELAY);
      }
    }
    /* DMA_NORMAL 模式收完一包即停，必须重装接收，否则坐标只收一次 */
    Lub_Cat_receive_start();
  } else if (huart == &huart2) { // OPS9数据处理
    if (Size == 28 && OPS_redata[0] == 0x0D && OPS_redata[1] == 0x0A &&
        OPS_redata[26] == 0x0A && OPS_redata[27] == 0x0D) {
      // if (cyz_state == true) {
      //   cyz_state = false;
      //   OPS_angle = cyz_angle;
      // }
      OPS_angle = *(float *)&OPS_redata[2];
      OPS_X = *(float *)&OPS_redata[14];
      OPS_Y = *(float *)&OPS_redata[18];
      if (posit_state >= 2) {
        OPS_angle += 1;
      }
      if (posit_state >= 5) {
        OPS_angle += 2;
      }
      if (OPS_angle >= 180) {
        OPS_angle = 180;
      }
      float temp = OPS_X;
      OPS_X = OPS_Y;
      OPS_Y = -temp;
      /* 这里不要再去改 OPS_angle 本身。
       * 原来那 4 行在 current_r==180 时把 -178 加成 182，紧跟着下面这个 ±180
       * 校验立刻就不认它了 → opsready 再也置不起来 → pid_to_v 一直 return false
       * →
       * 不再下发新指令，驱动板保持上一条速度把车带走，也就是"跑到180度后跑偏"。
       * 就近方向的判断已挪到 motor.c 的
       * ang_diff()，用局部量做，全局读数保持原样。 */

      if (OPS_X >= -20000.0f && OPS_X <= 20000.0f && OPS_Y >= -20000.0f &&
          OPS_Y <= 20000.0f && OPS_angle >= -180.0f && OPS_angle <= 180.0f) {
        opsready = true;
      }
    }
    /* DMA_NORMAL 模式收完一包即停，必须重装接收，否则坐标只收一次 */
    ops9_receive_start();
  } else if (huart == &huart5) { // 陀螺仪
    if (Size == 16) {
      if (cyz_redata[0] == 0xAA && cyz_redata[1] == 0x55) {
        cyz_state = true;
        cyz_angle = *(float *)&cyz_redata[4];
      }
    }
    cyz_receive_start();
  } else if (huart == &huart7) { // AR_Screen数据处理
    if (Size > 14) {
      ar_screen_sta = true;
    }
  } else if (huart == &huart1) {
    car_debug_state = true;
    car_debug_dalen = Size;
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, car_debug, 13);
  }
}
/*-------------------------------------串口外设---------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  // if (huart == &huart1) { // 全局定位
  //     re_sta = true;
  //     HAL_UART_Receive_IT(&huart1, hc_os, 17);
  // }

  // if(huart == &huart1){//路径规划
  // 	pt_sta = 1;
  // 	pt[0] = pathl[0]-48;
  // 	pt[1] = pathl[1]-48;
  // 	pt[2] = pathl[2]-48;
  // 	pt[3] = pathl[3]-48;
  // 	currentState = STATE_NAV;
  // 	HAL_UART_Receive_IT(&huart1, pathl, sizeof(pathl));//路径规划
  // }

  // if (huart == &huart1) { // 机械臂调试
  //   arm_state = true;
  //   HAL_UART_Receive_IT(&huart1, arm_control_data, 5);
  // }

  // if (huart == &huart1) { // 鲁班猫
  //   lubanready = true;
  //   HAL_UART_Receive_IT(&huart1, lub_cat_re, 4);
  // }
}
/*------------------------------------数据接收错误重启--------------------------------__*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
    // 无条件清除所有错误标志
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_NE |
                                     UART_FLAG_ORE);
    // 清空HAL库错误标记
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    // 重启串口接收（此时 HAL 已将 RxState 置为 READY，可安全重装）
    ops9_receive_start();
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM12_Init();
  MX_TIM4_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
  //--------------------------------初始化----------------------------------
  HAL_UART_Transmit(&huart5, cyz_cmd, 8, HAL_MAX_DELAY); // 陀螺仪
  servo_init();                                          // 舵机初始化
  motor_en();                                            // 电机使能
  ops9_receive_start(); /* 启动 OPS9 接收 DMA */
  Lub_Cat_receive_start();
  //--------------------------------调试----------------------------------
  //	HAL_UART_Receive_IT(&huart1, pathl, sizeof(pathl));//路径规划
  //	HAL_UART_Receive_IT(&huart1, &receive, 1);
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, car_debug, 13); // 整车调试
  uint8_t stack[3];
  // uint8_t stack2[3]; // 抓取相应位置的物块
  uint8_t stack3[3];
  // cyz_receive_start();
  // HAL_TIM_Base_Start_IT(&htim3); // OPS9启动
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // AR_Screen_Start();
    //-----------------调试------------------------
    if (car_debug_state) {
      car_debug_state = false;
      if (car_debug_dalen == 3) {
        if (car_debug[0] == 'g' && car_debug[1] == 'g') {
          grab_ground_id(car_debug[2] - 48);
        } else if (car_debug[0] == 'g' && car_debug[1] == 'p') {
          grab_plate(car_debug[2] - 48, 1);
        } else if (car_debug[0] == 'p' && car_debug[1] == 'g') {
          place_ground(car_debug[2] - 48);
        } else if (car_debug[0] == 'p' && car_debug[1] == 'p') {
          place_plate(car_debug[2] - 48);
        } else if (car_debug[0] == 'p' && car_debug[1] == 'b') {
          place_block(car_debug[2] - 48);
        }
      } else if (car_debug_dalen == 4) { // 鲁班猫调试
        //         色环检测	AF FA 07 01 01 CF FC
        // 物料-红色	AF FA 08 02 01 03 CF FC
        // 物料-黄色	AF FA 08 02 02 00 CF FC
        // 物料-蓝色	AF FA 08 02 03 01 CF FC
        // 物料-绿色	AF FA 08 02 04 06 CF FC
        // 物料-黑色	AF FA 08 02 05 07 CF FC
        // 物料-浅蓝色	AF FA 08 02 06 04 CF FC
        Lub_Cat_receive_start();
        if (car_debug[0] == 'f' && car_debug[3] == 'f') {
          if (car_debug[1] == '1') {
            Lub_Cat_send_yolo(car_debug[2] - 48);
            if (cat_centre_calibrate()) {
              HAL_UART_Transmit(&huart1, (uint8_t *)"对齐", sizeof("对齐") - 1,
                                HAL_MAX_DELAY);
            }
          } else if (car_debug[1] == '2') {
            Lub_Cat_send_ring();
            if (cat_centre_calibrate()) {
              HAL_UART_Transmit(&huart1, (uint8_t *)"对齐", sizeof("对齐") - 1,
                                HAL_MAX_DELAY);
            }
          } else if (car_debug[1] == '3') {
            Lub_Cat_send_material(car_debug[2] - 48);
            if (cat_centre_calibrate()) {

              HAL_UART_Transmit(&huart1, (uint8_t *)"对齐", sizeof("对齐") - 1,
                                HAL_MAX_DELAY);
            }
          } else if (car_debug[1] == '0' && car_debug[2] == '0') {
            Lub_Cat_send_exit();
          }
        } else if (car_debug[0] == 'f' && car_debug[3] == 'a') {
          if (car_debug[1] == '3') {
            Lub_Cat_send_material(car_debug[2] - 48);
            if (cat_centre_calibrate_arm()) {

              HAL_UART_Transmit(&huart1, (uint8_t *)"对齐", sizeof("对齐") - 1,
                                HAL_MAX_DELAY);
            }
          }
        }
      } else if (car_debug_dalen == 5) { // 机械臂调试
        if (car_debug[0] == 'f') {
          int pulse = (car_debug[2] - 48) * 100 + (car_debug[3] - 48) * 10 +
                      car_debug[4] - 48;
          servo_set_angle(car_debug[1] - 48, pulse);
        } else if (car_debug[0] == '+') { // 升降
          int num = (car_debug[2] - 48) * 100 + (car_debug[3] - 48) * 10 +
                    car_debug[4] - 48;
          if (car_debug[1] == '0') {
            send_motor_place_absolute(0, (uint32_t)num);
          } else {
            send_motor_place_relative(0, (uint32_t)num);
          }
        } else if (car_debug[0] == '-') { // 升降
          int num = (car_debug[2] - 48) * 100 + (car_debug[3] - 48) * 10 +
                    car_debug[4] - 48;
          if (car_debug[1] == '0') {
            send_motor_place_absolute(1, (uint32_t)num);
          } else {
            send_motor_place_relative(1, (uint32_t)num);
          }
        }
      } else if (car_debug_dalen == 6) {
        Lub_Cat_receive_start();
        HAL_Delay(500);
        pid_to_goal(200, 200, 0);
      } else if (car_debug_dalen == 7) {
        if (car_debug[0] == 'g' && car_debug[1] == 'o') {
          currentState = STATE_NAV;
        }
      } else if (car_debug_dalen == 13) { // 定位
        goal_x = 0;
        goal_y = 0;
        goal_w = 0;
        goal_x = (car_debug[1] - 48) * 1000 + (car_debug[2] - 48) * 100 +
                 (car_debug[3] - 48) * 10;
        goal_y = (car_debug[5] - 48) * 1000 + (car_debug[6] - 48) * 100 +
                 (car_debug[7] - 48) * 10;
        goal_w = (car_debug[9] - 48) * 100 + (car_debug[10] - 48) * 10 +
                 (car_debug[11] - 48);
        if (car_debug[4] == '-') {
          goal_x = 0 - goal_x;
        }
        if (car_debug[8] == '-') {
          goal_y = 0 - goal_y;
        }
        if (car_debug[12] == '-') {
          goal_w = 0 - goal_w;
        }
        if (car_debug[0] == 'A') {
          if (pid_to_goal(goal_x, goal_y, goal_w)) {
            HAL_UART_Transmit(&huart1, (uint8_t *)"全局定位okk",
                              sizeof("全局定位okk") - 1, HAL_MAX_DELAY);
          }
        } else if (car_debug[0] == 'R') {
          if (pid_to_goal_relative(goal_x, goal_y, goal_w)) {
            HAL_UART_Transmit(&huart1, (uint8_t *)"相对移动okk",
                              sizeof("相对移动okk") - 1, HAL_MAX_DELAY);
          }
        } else if (car_debug[0] == 'S') {
          set_cur_pos(goal_w, goal_x, goal_y);
        }
      }
    }

    //-----------------状态机------------------------
    switch (currentState) { // 初始化，
    case STATE_INIT:
      continue;
    case STATE_NAV: {
      HAL_UART_Transmit(&huart5, cyz_clear, 8, HAL_MAX_DELAY);
      if (posit_state == 0 && pid_to_goal(200, 200, 0)) {
        posit_state = 1;
      }
      if (posit_state == 1 && pid_to_path(0, 0, 2, 0)) { // 路径规划
        pid_to_goal(current_x, current_y, 0);
        AR_Screen_Start();
        HAL_Delay(100);
        AR_screen_stop();
        for (int pd = 0; pd < 3; pd++) {
          if (ar_number_order[pd] == 1) {
            stack[0] = pd;
          } else if (ar_number_order[pd] == 2) {
            stack[1] = pd;
          } else if (ar_number_order[pd] == 3) {
            stack[2] = pd;
          }
        }
        // for (int pd = 3; pd < 6; pd++) {
        //   if (ar_material_order[pd] == ar_material_order[stack[0]]) {
        //     stack2[0] = pd;
        //   } else if (ar_material_order[pd] == ar_material_order[stack[1]]) {
        //     stack2[1] = pd;
        //   } else if (ar_material_order[pd] == ar_material_order[stack[2]]) {
        //     stack2[2] = pd;
        //   }
        // }
        for (int pd = 0; pd < 3; pd++) {
          if (ar_material_order[3] == ar_material_order[pd]) {
            stack3[0] = pd;
          } else if (ar_material_order[4] == ar_material_order[pd]) {
            stack3[1] = pd;
          } else if (ar_material_order[5] == ar_material_order[pd]) {
            stack3[2] = pd;
          }
        }
        posit_state = 2;
      }
      if (posit_state == 2 && pid_to_path(2, 0, 4, 2)) { // 1抓
        pid_to_goal(current_x, current_y, 0);
        send_motor_place_absolute(motor_up, arm_ullimit);
        HAL_Delay(50);
        while (1) {
          pid_to_goal(current_x, current_y, current_r);
          if (arm_state == 0 && to_material_platform(ar_material_order[0])) {
            grab_platform();
            place_plate(3);
            arm_state = 1;
          }
          if (arm_state == 1 && to_material_platform(ar_material_order[1])) {
            grab_platform();
            place_plate(2);
            arm_state = 2;
          }
          if (arm_state == 2 && to_material_platform(ar_material_order[2])) {
            grab_platform();
            place_plate(1);
            arm_free();
            arm_state = 0;
            break;
          }
        }

        posit_state = 3;
      }
      if (posit_state == 3 && pid_to_path(4, 2, 0, 2)) { // 放1
        pid_to_goal_relative(20, 0, 0);
        pid_to_goal(current_x, current_y, 180);
        if (to_yolo(CAT_YOLO_TWO)) {
          to_ring();
          grab_plate(3, 1);
          place_ground(ar_number_order[0]);
          grab_plate(2, 1);
          place_ground(ar_number_order[1]);
          grab_plate(1, 0);
          place_ground(ar_number_order[2]);
          send_motor_place_absolute(motor_up, 0);
          HAL_Delay(100);
          grab_ground_id(ar_number_order[0]);
          place_plate(3);
          grab_ground_id(ar_number_order[1]);
          place_plate(2);
          grab_ground_id(ar_number_order[2]);
          place_plate(1);
          arm_free();
          refresh_lubcat();
        }
        posit_state = 4;
      }
      if (posit_state == 4 && pid_to_path(0, 2, 2, 4)) { // 放2
        pid_to_goal_relative(0, -30, 0);
        if (to_yolo(CAT_YOLO_TWO)) {
          to_ring();
          grab_plate(3, 1);
          place_ground(ar_number_order[0]);
          grab_plate(2, 1);
          place_ground(ar_number_order[1]);
          grab_plate(1, 0);
          place_ground(ar_number_order[2]);
          arm_free();
          refresh_lubcat();
        }
        posit_state = 5;
      }
      if (posit_state == 5 && pid_to_path(2, 4, 4, 2)) { // 2抓

        pid_to_goal(current_x, current_y, 0);
        pid_to_goal_relative(-40, -20, 0);
        while (1) {
          pid_to_goal(current_x, current_y, current_r);
          if (arm_state == 0 && to_material_platform(ar_material_order[3])) {
            grab_platform();
            place_plate(3);
            arm_state = 1;
          }
          if (arm_state == 1 && to_material_platform(ar_material_order[4])) {
            grab_platform();
            place_plate(2);
            arm_state = 2;
          }

          if (arm_state == 2 && to_material_platform(ar_material_order[5])) {
            grab_platform();
            place_plate(1);
            arm_free();
            arm_state = 0;
            break;
          }
        }

        posit_state = 6;
      }
      if (posit_state == 6 && pid_to_path(4, 2, 0, 2)) { // 放1

        pid_to_goal_relative(20, 0, 0);
        pid_to_goal(current_x, current_y, 180);
        if (to_yolo(CAT_YOLO_TWO)) {
          to_ring();
          grab_plate(3, 1);
          place_ground(ar_number_order[3]);
          grab_plate(2, 1);
          place_ground(ar_number_order[4]);
          grab_plate(1, 0);
          place_ground(ar_number_order[5]);
          send_motor_place_absolute(motor_up, 0);
          HAL_Delay(100);
          grab_ground_id(ar_number_order[3]);
          place_plate(3);
          grab_ground_id(ar_number_order[4]);
          place_plate(2);
          grab_ground_id(ar_number_order[5]);
          place_plate(1);
          arm_free();
        }
        posit_state = 7;
      }
      if (posit_state == 7 && pid_to_path(0, 2, 2, 4)) {

        pid_to_goal_relative(0, -30, 0);
        if (current_r == -180 || current_r == +180) {
          pid_to_goal_relative(0, 0, -90);
        }
        if (current_r == -90) {
          pid_to_goal_relative(0, 0, 90);
          HAL_Delay(500);
          pid_to_goal_relative(0, 0, 90);
        }

        if (to_material(ar_material_order[stack[1]])) {

          grab_plate(3, 1);
          place_block(ar_number_order[stack3[0]]);
          grab_plate(2, 1);
          if (stack3[0] == 2 && stack3[1] == 1) {
            servo_set_angle(SERVO_XH270, 0);
          }
          place_block(ar_number_order[stack3[1]]);
          grab_plate(1, 1);
          if (stack3[2] == 1) {
            servo_set_angle(SERVO_XH270, 0);
          }
          place_block(ar_number_order[stack3[2]]);
          servo_set_angle(SERVO_SG90, servo_place);
          HAL_Delay(500);
          servo_set_angle(SERVO_XH270, grab2_2servo);
          servo_set_angle(SERVO_XH360, plate_two);
          HAL_Delay(500);
        }
        posit_state = 8;
      }
      if (posit_state == 8 && pid_to_path(2, 4, 0, 0)) {
        posit_state = 9;
        if (pid_to_goal(-130, 0, 0))
          posit_state = 0;
        currentState = STATE_STOP;
      } else
        continue;
      break;
    }
    case STATE_STOP: {
      motor_stop();
      // 注意删除
      HAL_UART_Transmit(&huart1, (uint8_t *)"over", sizeof("over") - 1,
                        HAL_MAX_DELAY);
      currentState = STATE_INIT;
      break;
    }
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
