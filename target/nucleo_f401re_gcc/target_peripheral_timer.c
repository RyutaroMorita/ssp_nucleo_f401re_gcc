/*
 * target_peripheral_timer.c
 *
 *  Created on: 2020/02/24
 *      Author: Ryutaro Morita
 */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

extern HAL_StatusTypeDef HAL_InitTick2(uint32_t TickPriority);
extern TIM_HandleTypeDef htim5;

void target_peripheral_timer_initialize(intptr_t exinf)
{
	/* Set Interrupt Group Priority */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/* Use systick as time base source and configure 1ms tick (default clock after Reset is HSI) */
	HAL_InitTick(TICK_INT_PRIORITY);
}

void tim5_handler(void)
{
	if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET) {
		if (__HAL_TIM_GET_IT_SOURCE(&htim5, TIM_IT_UPDATE) != RESET) {
			__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
			HAL_IncTick();
		}
	}
}
