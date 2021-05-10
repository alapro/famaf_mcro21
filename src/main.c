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

int main(void)
{

	BSP_Init();

	LED_blinky(ledBlue, 100, 900, 10);

	while(1){

		t = SENSTEMP_getTemperature();
		t++;

	}
}


void App_1msPeriod(void){

	static uint16_t cont = 1;

	cont--;
	if(!cont){
		cont = 300;
		LED_toggle(ledOrange);
	}
}


