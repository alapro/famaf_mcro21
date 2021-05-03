/*
 * led.h
 *
 *  Created on: 28 abr. 2021
 *      Author: agust
 */

#ifndef LED_H_
#define LED_H_

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} GPIOLED_TypeDef;

void LED_init(void);

void LED_blinkyIRQ(void);

#endif /* LED_H_ */
