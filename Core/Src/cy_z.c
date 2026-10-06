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
void cyz_receive_start() {
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, cyz_redata, 18); // 陀螺仪
}
void cyz_receive_stop() { HAL_UART_AbortReceive(&huart5); }
