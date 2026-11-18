/*
 * target_peripheral_timer.h
 *
 *  Created on: 2020/02/24
 *      Author: ryuta
 */

#ifndef TARGET_STM32F4DISCOVERY_GCC_TARGET_PERIPHERAL_TIMER_H_
#define TARGET_STM32F4DISCOVERY_GCC_TARGET_PERIPHERAL_TIMER_H_

#define INHNO_TIM5		(TIM5_IRQn + 16)		/* 50 */
#define INTNO_TIM5		(TIM5_IRQn + 16)		/* 50 */
#define INTATR_TIM5		(0)		/* 割込み属性	*/
#define INTPRI_TIM5		(-1)						/* 割込み優先度	*/

extern void target_peripheral_timer_initialize(intptr_t exinf);

#endif /* TARGET_STM32F4DISCOVERY_GCC_TARGET_PERIPHERAL_TIMER_H_ */
