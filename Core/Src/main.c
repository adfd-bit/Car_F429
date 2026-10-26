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
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OPS9.h"
#include "ar_screen.h"
#include "arm_control.h"
#include "cy_z.h"
#include "ldrobot_stl.h"
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
#define compensation_angle_1 1
#define compensation_angle_2 2
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
typedef enum {
  start_stop_zone_1 = 0x01,
  start_stop_zone_2 = 0x02,
} start_stop_zone;
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
uint8_t volatile current_task_m = 255;
start_stop_zone map_zone;
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
/* 雷达障碍物只投影一次的一次性标志。
 * 进 STATE_NAV 那一刻车还静止，位姿最准；投影完就冻结，之后不再重建
 * （否则车开出去后还会用新位姿去套起步时的旧读数，障碍物整体平移）。 */
static bool radar_painted = false;
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
  } else if (htim == &htim2) {
    if (current_task_m == 255) {
      return;
    } else if (current_task_m < 3) {
      Lub_Cat_send_yolo(current_task_m);
    } else if (current_task_m < 9) {
      Lub_Cat_send_material(current_task_m - 2);
    } else if (current_task_m == 9) {
      Lub_Cat_send_ring();
    }
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
      static uint8_t cat_error_num = 0;
      if (CAT_redata[0] == 0xAF && CAT_redata[1] == 0xFA &&
          CAT_redata[2] == 0x91 && CAT_redata[3] == 0xCF &&
          CAT_redata[4] == 0xFC) {
        cat_error_num++;
        catready = false;
      }
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
  } else if (huart == &huart6) { // 乐动 STL-06N 激光雷达
    ldrobot_stl_rx_event_callback(huart, Size);
  } else if (huart == &huart1) { // 调试
    car_debug_state = true;
    car_debug_dalen = Size;
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, car_debug, 13);
  }
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
  if (huart == &huart6) { // 乐动 STL-06N 激光雷达
    ldrobot_stl_rx_error_callback(huart);
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //--------------------------------初始化----------------------------------
  // HAL_UART_Transmit(&huart5, cyz_cmd, 8, HAL_MAX_DELAY); // 陀螺仪
  servo_init(); // 舵机初始化
  // motor_en();           // 电机使能
  motor_noen();
  ops9_receive_start(); /* 启动 OPS9 接收 DMA */
  Lub_Cat_receive_start();
  ldrobot_stl_init(); /* 启动 STL-06N 激光雷达 USART6 接收 */
  /* 雷达上电默认就是 10Hz（3600 度/秒），这里主动下发一次 0xA2 显式设成
   * 10Hz，万一把转速设低了或者跑偏，这条指令能拉回默认值。 */
  ldrobot_stl_set_speed(LDR_STL_SCAN_SPEED_DPS);
  HAL_TIM_Base_Start_IT(&htim2);
  // HAL_TIM_Base_Start_IT(&htim3); // OPS9启动
  uint8_t stack[3];
  uint8_t stack3[3];
  map_zone = start_stop_zone_1;
  //--------------------------------调试----------------------------------
  //	HAL_UART_Receive_IT(&huart1, pathl, sizeof(pathl));//路径规划
  //	HAL_UART_Receive_IT(&huart1, &receive, 1);
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, car_debug, 13); // 整车调试

  /* 起 TIM3：PSC=1799 / ARR=49999，APB1 定时器时钟 90MHz
   *   → 90M / 1800 / 50000 = 1Hz，1 秒一次中断。
   * 回调里数到 14 次（a > 13）才转 STATE_NAV —— 所以实际是 14 秒。
   * 这 14 秒主循环在 STATE_INIT 空转，只跑 ldrobot_stl_process() 收雷达，
   * 等 OPS9 起来。这就是"先扫 14 秒，再导航"。 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
          } else if (car_debug[1] == '4') {
            Lub_Cat_send_material(car_debug[2] - 48);
            if (cat_centre_calibrate_material()) {

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
    case STATE_INIT: {
      ldrobot_stl_process();
      continue;
    }

    case STATE_NAV: {
      if (!radar_painted) { /* 只在跳变那一瞬间画一次：此刻车还没动，
                             * 静止位姿投影最准。之后冻结。 */
        radar_painted = true;
        (void)PathPlan_UpdateFromRadar();
      }
      if (map_zone == start_stop_zone_2) {

        HAL_UART_Transmit(&huart5, cyz_clear, 8, HAL_MAX_DELAY);
        if (posit_state == 0 && pid_to_goal(200, 200, 0)) {
          posit_state = 1;
        }
        if (posit_state == 1 && pid_to_path(0, 0, 2, 0)) { // 路径规划
          pid_to_goal(current_x, current_y, 0);
          pid_to_goal_relative(0, 50, 0);
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
          //   } else if (ar_material_order[pd] == ar_material_order[stack[1]])
          //   {
          //     stack2[1] = pd;
          //   } else if (ar_material_order[pd] == ar_material_order[stack[2]])
          //   {
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
          HAL_Delay(100);
          motor_stop();
          HAL_Delay(100);
          while (1) {
            pid_to_goal(current_x, current_y, current_r);
            HAL_Delay(100);
            motor_stop();
            HAL_Delay(100);
            if (arm_state == 0 && to_material_platform(ar_material_order[0])) {
              // set_cur_pos(1, 1920, 1050);
              grab_platform();
              place_plate(3);
              AR_Screen_SendData(1, 0);
              arm_state = 1;
            }
            if (arm_state == 1 && to_material_platform(ar_material_order[1])) {
              grab_platform();
              place_plate(2);
              AR_Screen_SendData(2, 0);
              arm_state = 2;
            }
            if (arm_state == 2 && to_material_platform(ar_material_order[2])) {
              grab_platform();
              place_plate(1);
              arm_free();
              AR_Screen_SendData(3, 0);
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
            set_cur_pos(179, 170, 1070);
            grab_plate(3, 1);
            place_ground(ar_number_order[0]);
            AR_Screen_SendData(3, 1);
            grab_plate(2, 1);
            place_ground(ar_number_order[1]);
            AR_Screen_SendData(3, 2);
            grab_plate(1, 0);
            place_ground(ar_number_order[2]);
            AR_Screen_SendData(3, 3);
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
          pid_to_goal(current_x, current_y, 90);
          if (to_yolo(CAT_YOLO_TWO)) {
            set_cur_pos(89, 1070, 1920);
            to_ring();
            grab_plate(3, 1);
            place_ground_two(ar_number_order[0]);
            AR_Screen_SendData(3, 1);
            grab_plate(2, 1);
            place_ground_two(ar_number_order[1]);
            AR_Screen_SendData(3, 2);
            grab_plate(1, 0);
            place_ground_two(ar_number_order[2]);
            AR_Screen_SendData(3, 3);
            arm_free();
            refresh_lubcat();
          }
          posit_state = 5;
        }
        if (posit_state == 5 && pid_to_path(2, 4, 4, 2)) { // 2抓

          pid_to_goal(current_x, current_y, 0);
          pid_to_goal_relative(-30, -25, 0);
          while (1) {
            pid_to_goal(current_x, current_y, current_r);
            if (arm_state == 0 && to_material_platform(ar_material_order[3])) {
              grab_platform();
              place_plate(3);
              AR_Screen_SendData(4, 3);
              arm_state = 1;
            }
            if (arm_state == 1 && to_material_platform(ar_material_order[4])) {
              grab_platform();
              place_plate(2);
              AR_Screen_SendData(5, 3);
              arm_state = 2;
            }

            if (arm_state == 2 && to_material_platform(ar_material_order[5])) {
              grab_platform();
              place_plate(1);
              arm_free();
              AR_Screen_SendData(6, 3);
              arm_state = 0;
              break;
            }
          }

          posit_state = 6;
        }
        if (posit_state == 6 && pid_to_path(4, 2, 0, 2)) { // 放1

          pid_to_goal(current_x, current_y, 180);
          pid_to_goal_relative(-55, 0, 0);

          if (to_yolo(CAT_YOLO_TWO)) {
            to_ring();
            set_cur_pos(177, 180, 1070);
            grab_plate(3, 1);
            place_ground(ar_number_order[3]);
            AR_Screen_SendData(6, 4);
            grab_plate(2, 1);
            place_ground(ar_number_order[4]);
            AR_Screen_SendData(6, 5);
            grab_plate(1, 0);
            place_ground(ar_number_order[5]);
            AR_Screen_SendData(6, 6);
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
          pid_to_goal(current_x, current_y, 90);
          pid_to_goal_relative(0, -30, 0);

          while (1) {
            pid_to_goal(current_x, current_y, current_r);
            if (to_material(ar_material_order[stack[1]])) {
              set_cur_pos(87, 1070, 1930);
              grab_plate(3, 1);
              place_block(ar_number_order[stack3[0]]);
              AR_Screen_SendData(6, 4);
              grab_plate(2, 1);
              if (stack3[0] == 2 && stack3[1] == 1) {
                servo_set_angle(SERVO_XH270, 10);
                pid_to_goal_relative(0, -50, 0);
                servo_set_angle(SERVO_XH360, place1_3servo);
              }
              place_block(ar_number_order[stack3[1]]);
              AR_Screen_SendData(6, 5);
              if (stack3[2] == 1) {
                pid_to_goal_relative(-70, 0, 0);
                if (to_material(ar_material_order[stack[0]])) {
                  grab_plate(1, 1);
                  place_block(2);
                  AR_Screen_SendData(6, 6);
                  arm_free();
                  posit_state = 8;
                  break;
                }
              }
              grab_plate(1, 1);
              place_block(ar_number_order[stack3[2]]);
              AR_Screen_SendData(6, 6);
              servo_set_angle(SERVO_SG90, servo_place);
              HAL_Delay(500);
              servo_set_angle(SERVO_XH270, grab2_2servo);
              servo_set_angle(SERVO_XH360, plate_two);
              HAL_Delay(500);
              break;
            }
          }

          posit_state = 8;
        }
        if (posit_state == 8 && pid_to_path(2, 4, 0, 0)) {
          posit_state = 9;
          if (pid_to_goal(-130, 65, 0))
            posit_state = 0;
          currentState = STATE_STOP;
        } else
          continue;
      }
      //--------------------------启停区1----------------------------
      else if (map_zone == start_stop_zone_1) {
        HAL_UART_Transmit(&huart5, cyz_clear, 8, HAL_MAX_DELAY);
        if (posit_state == 0 && pid_to_goal(200, 200, 0)) {
          posit_state = 1;
        }
        if (posit_state == 1 && pid_to_path(0, 0, 0, 2)) {
          pid_to_goal(current_x, current_y, -90);
          pid_to_goal_relative(50, 0, 0);
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
          //   } else if (ar_material_order[pd] == ar_material_order[stack[1]])
          //   {
          //     stack2[1] = pd;
          //   } else if (ar_material_order[pd] == ar_material_order[stack[2]])
          //   {
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
        if (posit_state == 2 && pid_to_path(0, 2, 2, 0)) { // 1抓
          pid_to_goal(current_x, current_y, -90);
          pid_to_goal_relative(0, -45, 0);
          HAL_Delay(100);
          motor_stop();
          HAL_Delay(100);
          while (1) {
            pid_to_goal(current_x, current_y, current_r);
            HAL_Delay(100);
            motor_stop();
            HAL_Delay(100);
            if (arm_state == 0 && to_material_platform(ar_material_order[0])) {
              // set_cur_pos(1, 1920, 1050);
              grab_platform();
              place_plate(3);
              AR_Screen_SendData(1, 0);
              arm_state = 1;
            }
            if (arm_state == 1 && to_material_platform(ar_material_order[1])) {
              grab_platform();
              place_plate(2);
              AR_Screen_SendData(2, 0);
              arm_state = 2;
            }
            if (arm_state == 2 && to_material_platform(ar_material_order[2])) {
              grab_platform();
              place_plate(1);
              arm_free();
              AR_Screen_SendData(3, 0);
              arm_state = 0;
              break;
            }
          }

          posit_state = 3;
        }
        if (posit_state == 3 && pid_to_path(2, 0, 2, 4)) { // 放1
          pid_to_goal_relative(0, -20, 0);
          pid_to_goal(current_x, current_y, 90);
          if (to_yolo(CAT_YOLO_TWO)) {
            to_ring();
            set_cur_pos(89, 1070, 1920);
            grab_plate(3, 1);
            place_ground(ar_number_order[0]);
            AR_Screen_SendData(3, 1);
            grab_plate(2, 1);
            place_ground(ar_number_order[1]);
            AR_Screen_SendData(3, 2);
            grab_plate(1, 0);
            place_ground(ar_number_order[2]);
            AR_Screen_SendData(3, 3);
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
        if (posit_state == 4 && pid_to_path(2, 4, 4, 2)) { // 放2
          pid_to_goal_relative(-45, 0, 0);
          pid_to_goal(current_x, current_y, 0);
          if (to_yolo(CAT_YOLO_TWO)) {
            set_cur_pos(1, 1900, 1050);
            to_ring();
            grab_plate(3, 1);
            place_ground_two(ar_number_order[0]);
            AR_Screen_SendData(3, 1);
            grab_plate(2, 1);
            place_ground_two(ar_number_order[1]);
            AR_Screen_SendData(3, 2);
            grab_plate(1, 0);
            place_ground_two(ar_number_order[2]);
            AR_Screen_SendData(3, 3);
            arm_free();
            refresh_lubcat();
          }
          posit_state = 5;
        }
        if (posit_state == 5 && pid_to_path(4, 2, 2, 0)) { // 2抓
          pid_to_goal_relative(25, 30, 0);
          pid_to_goal(current_x, current_y, -90);

          while (1) {
            pid_to_goal(current_x, current_y, current_r);
            if (arm_state == 0 && to_material_platform(ar_material_order[3])) {
              grab_platform();
              place_plate(3);
              AR_Screen_SendData(4, 3);
              arm_state = 1;
            }
            if (arm_state == 1 && to_material_platform(ar_material_order[4])) {
              grab_platform();
              place_plate(2);
              AR_Screen_SendData(5, 3);
              arm_state = 2;
            }

            if (arm_state == 2 && to_material_platform(ar_material_order[5])) {
              grab_platform();
              place_plate(1);
              arm_free();
              AR_Screen_SendData(6, 3);
              arm_state = 0;
              break;
            }
          }

          posit_state = 6;
        }
        if (posit_state == 6 && pid_to_path(2, 0, 2, 4)) { // 放1
          pid_to_goal_relative(0, -30, 0);
          pid_to_goal(current_x, current_y, 90);

          if (to_yolo(CAT_YOLO_TWO)) {
            to_ring();
            set_cur_pos(87, 1050, 1900);
            grab_plate(3, 1);
            place_ground(ar_number_order[3]);
            AR_Screen_SendData(6, 4);
            grab_plate(2, 1);
            place_ground(ar_number_order[4]);
            AR_Screen_SendData(6, 5);
            grab_plate(1, 0);
            place_ground(ar_number_order[5]);
            AR_Screen_SendData(6, 6);
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
        if (posit_state == 7 && pid_to_path(2, 4, 4, 2)) { // 放2
          pid_to_goal(current_x, current_y, 0);
          pid_to_goal_relative(-45, 0, 0);

          while (1) {
            pid_to_goal(current_x, current_y, current_r);
            if (to_material(ar_material_order[stack[1]])) {
              set_cur_pos(3, 1900, 1050);
              grab_plate(3, 1);
              place_block(ar_number_order[stack3[0]]);
              AR_Screen_SendData(6, 4);
              grab_plate(2, 1);
              if (stack3[0] == 2 && stack3[1] == 1) {
                servo_set_angle(SERVO_XH270, 10);
                pid_to_goal_relative(-50, 0, 0);
                servo_set_angle(SERVO_XH360, place1_3servo);
              }
              place_block(ar_number_order[stack3[1]]);
              AR_Screen_SendData(6, 5);
              if (stack3[2] == 1) {
                pid_to_goal_relative(-70, 0, 0);
                if (to_material(ar_material_order[stack[0]])) {
                  grab_plate(1, 1);
                  place_block(2);
                  AR_Screen_SendData(6, 6);
                  arm_free();
                  posit_state = 8;
                  break;
                }
              }
              grab_plate(1, 1);
              place_block(ar_number_order[stack3[2]]);
              AR_Screen_SendData(6, 6);
              servo_set_angle(SERVO_SG90, servo_place);
              HAL_Delay(500);
              servo_set_angle(SERVO_XH270, grab2_2servo);
              servo_set_angle(SERVO_XH360, plate_two);
              HAL_Delay(500);
              break;
            }
          }

          posit_state = 8;
        }
        if (posit_state == 8 && pid_to_path(4, 2, 0, 0)) {
          posit_state = 9;
          if (pid_to_goal(-130, 55, 0))
            posit_state = 0;
          currentState = STATE_STOP;
        } else
          continue;
      }
    }
    case STATE_STOP: {
      motor_stop();
      /* 必须在这里重开 TIM3。回调里到点会 HAL_TIM_Base_Stop_IT(htim) 把它停掉，
       * 不重开的话第二轮会卡死在 STATE_INIT 永远进不了 STATE_NAV —— 车看着像
       * 上电了但一直不动。回调里的计数 a 在停机那一次已经清 0，所以直接 Start
       * 就是从 0 开始完整数 14 秒。 */
      radar_painted = false; /* 第二轮进 STATE_NAV 时要重新投影一次 */
      // HAL_TIM_Base_Start_IT(&htim3);
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
