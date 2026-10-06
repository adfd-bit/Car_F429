/*
 * cy_z.h
 *
 *  Created on: 2026年10月05日
 *      Author: twyyd
 */

#ifndef INC_CY_Z_H_
#define INC_CY_Z_H_

#include "stdbool.h"
#include "usart.h"
#include <stdint.h>

extern uint8_t cyz_redata[16];
void cyz_receive_start();
void cyz_receive_stop();
#endif /* INC_CY_Z_H_ */
