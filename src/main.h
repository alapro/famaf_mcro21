/*
 * main.h
 *
 *  Created on: 23 abr. 2021
 *      Author: agust
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include "../BSP/bsp.h"

typedef enum {	Idle,
				Cycle1,
				Cycle2,
				Cycle3
}smState_TypeDef;

typedef enum {	None,
				TimeOut,
				SW_Press
}event_TypeDef;


void TIM2_Init(void);
void SystemClock_Config(void);
void Error_Handler(void);

void App_1msPeriod(void);


#endif /* MAIN_H_ */
