/*
 * bsp.h
 *
 *  Created on: 28 abr. 2021
 *      Author: agust
 */

#ifndef BSP_H_
#define BSP_H_

#include <stdint.h>

void BSP_Init(void);

void CONSOLE_SendMsg(uint8_t *pData, uint16_t Size);

void LED_on(void *led);
void LED_off(void *led);
void LED_toggle(void *led);
void LED_blinky(void *led, uint16_t ton, uint16_t toff, uint16_t times);

float SENSTEMP_getTemperature(void);



#endif /* BSP_H_ */
