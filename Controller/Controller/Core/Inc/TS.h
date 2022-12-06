/*
 * LCD.h
 *
 *  Created on: Dec. 1, 2022
 *      Author: broderio
 */


#ifndef INC_TS_H_
#define INC_TS_H_

#include "stm32l4xx_hal.h"
#include "LCD.h"

#define X_A		-0.0979
#define Y_A		-0.0006
#define X_B   	 -0.0004
#define Y_B		-0.1352
#define X_C  	343.1929
#define Y_C		514.2382

#define X_PORT GPIOC
#define Y_PORT GPIOC

#define YPOS_PIN GPIO_PIN_0
#define XPOS_PIN GPIO_PIN_1
#define YNEG_PIN GPIO_PIN_2
#define XNEG_PIN GPIO_PIN_3

#define XPOS_ADC_CHANNEL ADC_CHANNEL_0
#define YPOS_ADC_CHANNEL ADC_CHANNEL_1
#define XNEG_ADC_CHANNEL ADC_CHANNEL_2
#define YNEG_ADC_CHANNEL ADC_CHANNEL_3

#define ADC_SAMPLE_COUNT 10


int map(int x, int in_min, int in_max, int out_min, int out_max);
void ADC_Select(ADC_HandleTypeDef *hadc, uint32_t channel, GPIO_TypeDef* port, uint32_t pin);
double TS_readX(ADC_HandleTypeDef *hadc);
double TS_readY(ADC_HandleTypeDef *hadc);
double TS_readPressure(ADC_HandleTypeDef *hadc);
void transform(double x, double y, double* x_out, double* y_out);

#endif /* INC_TS_H_ */
