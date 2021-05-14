/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "main.h"

void *ledRed;
void *ledBlue;
void *ledOrange;
void *ledGreen;

float t;
struct{
	event_TypeDef Semaphore;
	event_TypeDef Term;
}event;

struct{
	smState_TypeDef Semaphore;
	uint8_t Term;
}smState;

uint32_t semaphoreTimeOut;



int main(void)
{

	BSP_Init();

	event.Semaphore = None;
	event.Term = None;

	while(1){

		if(event.Semaphore != None){
			if (smState.Semaphore == Idle && event.Semaphore == SW_Press){
				smState.Semaphore = Cycle1;
				LED_blinky(ledRed, 500, 500, 6);
				semaphoreTimeOut = 5000;
			} else if (smState.Semaphore == Cycle1 && event.Semaphore == TimeOut){
				smState.Semaphore = Cycle2;
				LED_on(ledRed);
				LED_off(ledBlue);
				semaphoreTimeOut = 10000;
			}

			event.Semaphore = None;
		}

		if(event.Term != None){

			t = SENSTEMP_getTemperature();

			event.Term = None;
		}
	}
}


void App_1msPeriod(void){
	if(semaphoreTimeOut){
		semaphoreTimeOut--;
		if(!semaphoreTimeOut){
			event.Semaphore = TimeOut;
		}
	}

}

void SW_PressEvent(void){
	event.Semaphore = SW_Press;
}


