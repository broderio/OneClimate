/*
 * TS.c
 *
 *  Created on: Dec 1, 2022
 *      Author: broderio
 */

#include <TS.h>
#include "stm32l4xx_hal.h"
#include <stdlib.h>

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void transform(double x, double y, double* x_out, double* y_out)
{
	*x_out = X_A * x + X_B * y + X_C;
	*y_out = Y_A * x + Y_B * y + Y_C;
}

void ADC_Select(ADC_HandleTypeDef *hadc, uint32_t channel, GPIO_TypeDef* port, uint32_t pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(port, &GPIO_InitStruct);

	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(hadc, &sConfig);
}

double TS_readY(ADC_HandleTypeDef *hadc)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	// Pull Y+ high and Y- low to get voltage differential
	GPIO_InitStruct.Pin = YPOS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Y_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(Y_PORT, YPOS_PIN, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = YNEG_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Y_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(Y_PORT, YNEG_PIN, GPIO_PIN_RESET);

	// Set proper ADC channel
	ADC_Select(hadc, XNEG_ADC_CHANNEL, X_PORT, XNEG_PIN);

	// Read ADC
	uint32_t y = 0;
	for (int i = 0; i < ADC_SAMPLE_COUNT; ++i)
	{
		HAL_ADC_Start(hadc);
		HAL_ADC_PollForConversion(hadc, 100);
		y += HAL_ADC_GetValue(hadc);
		HAL_ADC_Stop(hadc);
	}
	y /= ADC_SAMPLE_COUNT;

	// Return Y+/- to high impedance state
	ADC_Select(hadc, YPOS_ADC_CHANNEL, Y_PORT, YPOS_PIN);
	ADC_Select(hadc, YNEG_ADC_CHANNEL, Y_PORT, YNEG_PIN);

	return y;
}

double TS_readX(ADC_HandleTypeDef *hadc)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Pull X+ high and X- low to get voltage differential
	GPIO_InitStruct.Pin = XPOS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(X_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(X_PORT, XPOS_PIN, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = XNEG_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(X_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(X_PORT, XNEG_PIN, GPIO_PIN_RESET);

	// Set proper ADC channel
	ADC_Select(hadc, YPOS_ADC_CHANNEL, Y_PORT, YPOS_PIN);

	// Read ADC
	uint32_t x = 0;
	for (int i = 0; i < ADC_SAMPLE_COUNT; ++i)
	{
		HAL_ADC_Start(hadc);
		HAL_ADC_PollForConversion(hadc, 100);
		x += HAL_ADC_GetValue(hadc);
		HAL_ADC_Stop(hadc);
	}
	x /= ADC_SAMPLE_COUNT;

	// Return X to high impedance state
	ADC_Select(hadc, XPOS_ADC_CHANNEL, X_PORT, XPOS_PIN);
	ADC_Select(hadc, XNEG_ADC_CHANNEL, X_PORT, XNEG_PIN);

	return x;
}

double TS_readPressure(ADC_HandleTypeDef *hadc)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

		// Pull X+ low  and Y- high to get voltage differential
		GPIO_InitStruct.Pin = XPOS_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(X_PORT, &GPIO_InitStruct);
		HAL_GPIO_WritePin(X_PORT, XPOS_PIN, GPIO_PIN_RESET);

		GPIO_InitStruct.Pin = YNEG_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(Y_PORT, &GPIO_InitStruct);
		HAL_GPIO_WritePin(Y_PORT, YNEG_PIN, GPIO_PIN_SET);

		// Set proper ADC channel
		ADC_Select(hadc, XNEG_ADC_CHANNEL, X_PORT, XPOS_PIN);

		// Read ADC
		int32_t z1 = 0, z2 = 0;
		for (int i = 0; i < ADC_SAMPLE_COUNT; ++i)
		{
			HAL_ADC_Start(hadc);
			HAL_ADC_PollForConversion(hadc, 100);
			z1 += HAL_ADC_GetValue(hadc);
			HAL_ADC_Stop(hadc);
		}
		z1 /= ADC_SAMPLE_COUNT;

		// Set proper ADC channel
		ADC_Select(hadc, YPOS_ADC_CHANNEL, Y_PORT, YPOS_PIN);

		for (int i = 0; i < ADC_SAMPLE_COUNT; ++i)
		{
			HAL_ADC_Start(hadc);
			HAL_ADC_PollForConversion(hadc, 100);
			z2 += HAL_ADC_GetValue(hadc);
			HAL_ADC_Stop(hadc);
		}
		z2 /= ADC_SAMPLE_COUNT;

		// Return X+ and Y- to high impedance state
		ADC_Select(hadc, XPOS_ADC_CHANNEL, X_PORT, XPOS_PIN);
		ADC_Select(hadc, YNEG_ADC_CHANNEL, Y_PORT, YNEG_PIN);

		return z2 - z1;
}
