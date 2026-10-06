/*
 * ar_screen.c
 *
 *  Created on: 2026年8月29日
 *      Author: twyyd
 */
#include "ar_screen.h"
#include "arm_control.h"
#include "usart.h"

/* 在此添加函数实现 */
void AR_Screen_Receive() {
  HAL_UARTEx_ReceiveToIdle_DMA(&huart7, ar_data, 30);
  __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
}
void AR_screen_stop() { HAL_UART_AbortReceive(&huart7); }
void AR_Screen_SendData() {
  HAL_UART_Transmit(&huart8, ar_data, sizeof(ar_data), HAL_MAX_DELAY);
}
void AR_Screen_Start() {
  if (!ar_screen_sta) {
    AR_Screen_Receive();
    while (1) {
      if (ar_screen_sta) {
        AR_Screen_SendData();
        ar_material_order[0] = ar_data[0] - 48;
        ar_material_order[1] = ar_data[1] - 48;
        ar_material_order[2] = ar_data[2] - 48;
        ar_material_order[3] = ar_data[8] - 48;
        ar_material_order[4] = ar_data[9] - 48;
        ar_material_order[6] = ar_data[10] - 48;

        ar_number_order[0] = ar_data[4] - 48;
        ar_number_order[1] = ar_data[5] - 48;
        ar_number_order[2] = ar_data[6] - 48;
        ar_number_order[3] = ar_data[12] - 48;
        ar_number_order[4] = ar_data[13] - 48;
        ar_number_order[5] = ar_data[14] - 48;
        break;
      }
    }
  }
}