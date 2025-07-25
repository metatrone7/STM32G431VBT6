/*
 * SocketBoardTest.h
 *
 *  Created on: Jul 24, 2025
 *      Author: ricky kim
 */
#ifndef SOCKETBOARDTEST_H
#define SOCKETBOARDTEST_H

/* Private includes ----------------------------------------------------------*/
#include "stm32g431xx.h"
#include "stm32g4xx_ll_gpio.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    GPIO_TypeDef* port;
    uint32_t pin;
} PinDef;


/* Private define ------------------------------------------------------------*/
#define PWM_PERIOD_CYCLES       17000 // 170 000 000/10000 >> 10k
#define GPIO_TEST_PIN_CNT 		28 //28 //sizeof(pins) / sizeof(pins[0]);

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
extern void GPIO_TestProcess(uint8_t* _cTestResult);

#endif
