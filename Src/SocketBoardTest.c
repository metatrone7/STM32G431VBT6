/*
 * SocketBoardTest.c
 *
 *  Created on: Jul 24, 2025
 *      Author: ricky kim
 */

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>
#include "SocketBoardTest.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
PinDef pins[] = {
    {GPIOA, LL_GPIO_PIN_0},//0
    {GPIOA, LL_GPIO_PIN_4},//1
    {GPIOA, LL_GPIO_PIN_8},//2
    {GPIOA, LL_GPIO_PIN_9},//3
    {GPIOA, LL_GPIO_PIN_10},//4
    {GPIOA, LL_GPIO_PIN_11},//5
    {GPIOA, LL_GPIO_PIN_12},//6
    {GPIOA, LL_GPIO_PIN_15},//7 실패, PA15 J2 R3 remove 후에도 실패.
    {GPIOB, LL_GPIO_PIN_3},//8
    {GPIOB, LL_GPIO_PIN_4},//9  실패 PB4 J2 R2 remove 후에 성공.
    {GPIOB, LL_GPIO_PIN_5},//10
    {GPIOB, LL_GPIO_PIN_6},//11
    {GPIOB, LL_GPIO_PIN_7},//12
    {GPIOB, LL_GPIO_PIN_8},//13
    {GPIOB, LL_GPIO_PIN_9},//14
    {GPIOB, LL_GPIO_PIN_10},//15
    {GPIOC, LL_GPIO_PIN_0},//16
    {GPIOC, LL_GPIO_PIN_1},//17
    {GPIOC, LL_GPIO_PIN_2},//18
    {GPIOC, LL_GPIO_PIN_3},//19
    {GPIOC, LL_GPIO_PIN_4},//20
    {GPIOC, LL_GPIO_PIN_5},//21
    {GPIOC, LL_GPIO_PIN_13},//22
    {GPIOC, LL_GPIO_PIN_14},//23
    {GPIOC, LL_GPIO_PIN_15},//24
    {GPIOD, LL_GPIO_PIN_2},//25 실패 J2 관련 저항 수정 및 인풋 풀다운 설정 후 성공.
    {GPIOF, LL_GPIO_PIN_0},//26
    {GPIOF, LL_GPIO_PIN_1}//27
};

/* Private functions define --------------------------------------------------*/
void GPIO_TestProcess(uint8_t* _cTestResult);
void GPIO_SetPinInput(GPIO_TypeDef *GPIOx, uint32_t Pin);
void GPIO_SetPinOutput(GPIO_TypeDef *GPIOx, uint32_t Pin);
void Delay(volatile uint32_t count);


/* Private functions ---------------------------------------------------------*/
// 초기화 시 모두 성공(1) 또는 실패(0)로 설정 후, 실패 발견 시 기록하는 방식 선택 가능
// 여기서는 모두 성공(1)로 초기화 후 실패 시 0으로 변경하는 예시
void GPIO_TestProcess(uint8_t* _cTestResult)
{
    // 모든 핀을 입력 모드로 초기화 및 결과 초기화 (초기값 1: 성공)
    for (int i = 0; i < GPIO_TEST_PIN_CNT; i++)
    {
        GPIO_SetPinInput(pins[i].port, pins[i].pin);
        _cTestResult[i] = 1; // 초기값 1 (성공)
    }

    // 한 핀씩 출력 모드로 변경 후 High/Low 신호 검사
    for (int i = 0; i < GPIO_TEST_PIN_CNT; i++)
    {
        GPIO_SetPinOutput(pins[i].port, pins[i].pin);

        // 출력 핀을 High로 설정
        LL_GPIO_SetOutputPin(pins[i].port, pins[i].pin);
        Delay(1000000);

        // 모든 입력 핀 검사
        for (int j = 0; j < GPIO_TEST_PIN_CNT; j++)
        {
            if (j == i) continue; // 출력 핀 제외

            if (LL_GPIO_IsInputPinSet(pins[j].port, pins[j].pin) == 0)
            {
                _cTestResult[j] = 0; // j 핀 실패 기록
            }
            else
            {
//                _cTestResult[j] = 1; // j 핀 성공 기록
            }
        }

        // 출력 핀을 Low로 설정
        LL_GPIO_ResetOutputPin(pins[i].port, pins[i].pin);
        Delay(1000000);

        // 모든 입력 핀 검사
        for (int j = 0; j < GPIO_TEST_PIN_CNT; j++)
        {
            if (j == i) continue;

            if (LL_GPIO_IsInputPinSet(pins[j].port, pins[j].pin) != 0)
            {
                _cTestResult[j] = 0; // j 핀 실패 기록
            }
            else
            {
//                _cTestResult[j] = 1; // j 핀 성공 기록
            }
        }

        // 출력 핀 다시 입력 모드로 변경
        GPIO_SetPinInput(pins[i].port, pins[i].pin);
    }
}

void GPIO_SetPinInput(GPIO_TypeDef *GPIOx, uint32_t Pin) {
	// 모드 레지스터에서 해당 핀 2비트 클리어 후 입력(00) 설정
	uint32_t pos = 0;
	while (((Pin >> pos) & 1) == 0)
		pos++;

	uint32_t mode_reg = GPIOx->MODER;
	mode_reg &= ~(0x3 << (pos * 2)); // 2비트 클리어
	// 입력 모드(00)이므로 클리어만 하면 됨
	GPIOx->MODER = mode_reg;

//	// 풀업/풀다운 없음
//	uint32_t pupd_reg = GPIOx->PUPDR;
//	pupd_reg &= ~(0x3 << (pos * 2));
//	GPIOx->PUPDR = pupd_reg;

	// 풀업 설정
	uint32_t pupd_reg = GPIOx->PUPDR;
	pupd_reg &= ~(0x3 << (pos * 2));      // 해당 핀의 PUPD 비트 클리어
	pupd_reg |=  (0x1 << (pos * 2));      // 0x1: 풀업 설정 (01)
	GPIOx->PUPDR = pupd_reg;

//	// 풀다운 설정
//	uint32_t pupd_reg = GPIOx->PUPDR;
//	pupd_reg &= ~(0x3 << (pos * 2));      // 해당 핀의 PUPD 비트 클리어
//	pupd_reg |=  (0x2 << (pos * 2));      // 0x2: 풀다운 설정 (10)
//	GPIOx->PUPDR = pupd_reg;
}

void GPIO_SetPinOutput(GPIO_TypeDef *GPIOx, uint32_t Pin) {
	// 모드 레지스터에서 해당 핀 2비트 클리어 후 출력(01) 설정
	uint32_t pos = 0;
	while (((Pin >> pos) & 1) == 0)
		pos++;

	uint32_t mode_reg = GPIOx->MODER;
	mode_reg &= ~(0x3 << (pos * 2));
	mode_reg |= (0x1 << (pos * 2)); // 출력 모드(01)
	GPIOx->MODER = mode_reg;

	// 출력 속도 낮음 설정
	uint32_t ospeed_reg = GPIOx->OSPEEDR;
	ospeed_reg &= ~(0x3 << (pos * 2));
	ospeed_reg |= (0x0 << (pos * 2)); // Low speed
	GPIOx->OSPEEDR = ospeed_reg;

	// 풀업 설정 - 출력 모드에서는 PUPDR 설정이 무시됩니다.
	uint32_t pupd_reg = GPIOx->PUPDR;
	pupd_reg &= ~(0x3 << (pos * 2));      // 해당 핀의 PUPD 비트 클리어
	pupd_reg |=  (0x1 << (pos * 2));      // 0x1: 풀업 설정 (01)
	GPIOx->PUPDR = pupd_reg;

//	// 풀다운 설정 - 출력 모드에서는 PUPDR 설정이 무시됩니다.
//	uint32_t pupd_reg = GPIOx->PUPDR;
//	pupd_reg &= ~(0x3 << (pos * 2));      // 해당 핀의 PUPD 비트 클리어
//	pupd_reg |=  (0x2 << (pos * 2));      // 0x2: 풀다운 설정 (10)
//	GPIOx->PUPDR = pupd_reg;

	// 출력 타입 푸시풀 설정
	uint32_t otype_reg = GPIOx->OTYPER;
	otype_reg &= ~(1 << pos); // 0: Push-pull
	GPIOx->OTYPER = otype_reg;
}

void Delay(volatile uint32_t count) {
	while (count--)
		__NOP();
}
