/*
 * dcmotor_config.c
 *
 *  Created on: Mar 16, 2022
 *      Author: User
 */

#include "main.h"
#include "dcmotor.h"
#include "tim.h"

const DC_MOTOR_CfgType DC_MOTOR_CfgParam[DC_MOTOR_UNITS] =
{
	// DC MOTOR 1 Configurations
    {
	    &htim3,
		TIM_CHANNEL_2,
		TIM_CHANNEL_1,
		&htim16,
		TIM_CHANNEL_1,
		&hdma_tim16_ch1,
		TIM_DMA_CC1,
		(uint32_t) &(TIM3->CCR2),
		(uint32_t) &(TIM3->CCR1)
	}
};
