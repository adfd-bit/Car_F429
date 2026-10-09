/*
 * ar_screen.c
 *
 *  Created on: 2026年8月29日
 *      Author: twyyd
 */
#include "ar_screen.h"
#include "arm_control.h"
#include "motor.h"
#include "usart.h"
#include <stdint.h>

uint8_t save_data[12] = {};

/* 在此添加函数实现 */
void AR_Screen_Receive() {
  HAL_UARTEx_ReceiveToIdle_DMA(&huart7, ar_data, 30);
  __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
}
void AR_screen_stop() { HAL_UART_AbortReceive(&huart7); }
void AR_Screen_SendData(uint16_t grab, uint16_t place) {
  uint8_t num = 0;
  uint8_t send_data[24];
  send_data[0] = 0xAA;
  send_data[1] = 0x55;
  send_data[2] = 0x01;
  send_data[3] = 0x11;
  for (int i = 0; i < 12; i++) {
    send_data[i + 4] = save_data[i];
  }
  send_data[16] = grab;
  send_data[17] = grab >> 8;
  send_data[18] = place;
  send_data[19] = place >> 8;
  send_data[20] = 0x01;
  for (int j = 2; j < 21; j++) {
    num += send_data[j];
  }
  send_data[21] = num;
  send_data[22] = 0x0D;
  send_data[23] = 0x0A;
  HAL_UART_Transmit(&huart8, send_data, sizeof(send_data), HAL_MAX_DELAY);
}
void AR_Screen_Start() {
  if (!ar_screen_sta) {
    AR_Screen_Receive();
    while (1) {
      if (ar_screen_sta) {
        ar_material_order[0] = ar_data[0] - 48;
        ar_material_order[1] = ar_data[1] - 48;
        ar_material_order[2] = ar_data[2] - 48;
        ar_material_order[3] = ar_data[8] - 48;
        ar_material_order[4] = ar_data[9] - 48;
        ar_material_order[5] = ar_data[10] - 48;

        ar_number_order[0] = ar_data[4] - 48;
        ar_number_order[1] = ar_data[5] - 48;
        ar_number_order[2] = ar_data[6] - 48;
        ar_number_order[3] = ar_data[12] - 48;
        ar_number_order[4] = ar_data[13] - 48;
        ar_number_order[5] = ar_data[14] - 48;
        for (int i = 0; i < 12; i++) {
          if (i < 3) {
            save_data[i] = ar_material_order[i % 3];
          } else if (i < 6) {
            save_data[i] = ar_number_order[i % 3];
          } else if (i < 9) {
            save_data[i] = ar_material_order[i % 6 + 3];
          } else {
            save_data[i] = ar_number_order[i % 6];
          }
        }
        AR_Screen_SendData(0, 0);
        break;
      }
    }
  }
}
