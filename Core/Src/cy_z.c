/*
 * cy_z.c
 *
 *  Created on: 2026年10月05日
 *      Author: twyyd
 */

#include "cy_z.h"
#include "OPS9.h"
#include <stdbool.h>
#include <stdint.h>

float cyz_updata[10];
bool cyz_finish = false;
void cyz_receive_start() {
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, cyz_redata, 18); // 陀螺仪
}
void cyz_receive_stop() { HAL_UART_AbortReceive(&huart5); }
void cyz_data_enter(float data) {
  static uint8_t i = 0;
  cyz_updata[i] = data;
  if (i == 9) {
    cyz_finish = true;
    i = 0;
  }
  i++;
}
float cyz_updata_angle() {
  if (cyz_finish == true) {
    cyz_finish = false;
    float num = 0;
    for (int j = 0; j < 10; j++) {
      num += cyz_updata[j];
    }
    num /= 10;
    return num;
  } else
    return -999;
}