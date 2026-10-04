/*
 * OPS9.c
 *
 *  Created on: 2026年6月21日
 *      Author: twyyd
 */

#include "OPS9.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
#include <math.h>
#include <stdint.h>
void ops9_receive_start() {
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, OPS_redata, OPS9_data_len);
}
void ops9_receive_stop() {
  // 中止接收：HAL 内部会禁 DMAR、abort DMA、关 IDLE/RXNE/PE/ERR 中断、复位状态
  HAL_UART_AbortReceive(&huart2);
}
void set_cur_pos_x(float x) {
  current_x = x;
  uint8_t setdata[8] = {};
  setdata[0] = 'A';
  setdata[1] = 'C';
  setdata[2] = 'T';
  setdata[3] = 'Y';
  memcpy(&setdata[4], &x, sizeof(float));
  HAL_UART_Transmit(&huart2, setdata, 8, HAL_MAX_DELAY);
}
void set_cur_pos_y(float y) {
  current_y = y;
  y = -y;
  uint8_t setdata[8] = {};
  setdata[0] = 'A';
  setdata[1] = 'C';
  setdata[2] = 'T';
  setdata[3] = 'X';
  memcpy(&setdata[4], &y, sizeof(float));
  HAL_UART_Transmit(&huart2, setdata, 8, HAL_MAX_DELAY);
}
void set_cur_pos_angle(float angle) {
  current_r = angle;
  uint8_t setdata[8] = {};
  setdata[0] = 'A';
  setdata[1] = 'C';
  setdata[2] = 'T';
  setdata[3] = 'J';
  memcpy(&setdata[4], &angle, sizeof(float));
  HAL_UART_Transmit(&huart2, setdata, 8, HAL_MAX_DELAY);
}
void set_cur_pos(float angle, float x, float y) {
  set_cur_pos_x(x);
  HAL_Delay(100);
  set_cur_pos_y(y);
  HAL_Delay(100);
  set_cur_pos_angle(angle);
  HAL_Delay(100);
}
