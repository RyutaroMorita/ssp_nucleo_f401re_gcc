/*
 *  TOPPERS/SSP Kernel
 *      Smallest Set Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2007 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012 Meika Sugimoto
 *  Copyright (C) 2015 by Naoki Saito
 *              Nagoya Municipal Industrial Research Institute, JAPAN
 * 
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 * 
 */

/*
 * ターゲット依存モジュール（NUCLEO_F401RE用）
 */
#include "kernel_impl.h"
#include <sil.h>
#include "stm32f4xx_hal.h"
#include "target_serial.h"
#include "target_syssvc.h"


/*
 *  バーナ出力用のUARTの初期化
 */
static void usart_early_init(void);

/*
 *  初期化時のエラー発生時の処理
 */
extern void Error_Handler(void);

#if 0
/*
 * GPIOレジスタ操作関数
 */
Inline void set_cr_mode(uint32_t reg, uint_t p, int_t v)
{
	if (p < 8) {
		sil_andw((void*)GPIO_CRL(reg), ~CR_MODE_MASK(p));
		sil_orw((void*)GPIO_CRL(reg), CR_MODE(p, v));
	} else if (8 <= p && p < 16) {
		sil_andw((void*)GPIO_CRH(reg), ~CR_MODE_MASK(p - 8));
		sil_orw((void*)GPIO_CRH(reg), CR_MODE(p - 8, v));
	}
}

Inline void set_cr_cnf(uint32_t reg, uint_t p, int_t v)
{
	if (p < 8) {
		sil_andw((void*)GPIO_CRL(reg), ~CR_CNF_MASK(p));
		sil_orw((void*)GPIO_CRL(reg), CR_CNF(p, v));
	} else if (8 <= p && p < 16) {
		sil_andw((void*)GPIO_CRH(reg), ~CR_CNF_MASK(p - 8));
		sil_orw((void*)GPIO_CRH(reg), CR_CNF(p - 8, v));
	}
}

Inline void set_port_pull(uint32_t reg, uint_t p, bool_t up)
{
	if (up) {
		sil_wrw_mem((void*)GPIO_BSRR(reg), 0x01 << p);
	} else {
		sil_wrw_mem((void*)GPIO_BRR(reg), 0x01 << p);
	}
}
#endif

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * ターゲット依存部　初期化処理
 */
void target_initialize(void)
{
#if 0
	/*
	 *  プロセッサクロック(RCC)の初期化
	 */
	/* HSEの有効化 */
	sil_orw((void*)RCC_CR, CR_HSE_ON);

	/* HSE有効待ち */
	while ((sil_rew_mem((void*)RCC_CR) & CR_HSE_RDY) == 0)
		;

	/* FLASH ROMは2wait待ち */
	sil_andw((void*)FLASH_ACR, ~ACR_LATENCY_MASK);
	sil_orw((void*)FLASH_ACR, ACR_LATENCY_TWO);

	/* HCLK = SYSCLK, PCLK2 = HCLK, PCLK1 = HCLK/2  */
	sil_orw((void*)RCC_CFGR, 0x00 | (0x00 << 11) | (0x04 << 8))
		;

	/* PLLCLK = 8MHz x 9 = 72MHz, HSE as PLL clock */
	sil_orw((void*)RCC_CFGR, 0x07 << 18);
	sil_orw((void*)RCC_CFGR, CFGR_PLL_SRC);

	/* PLLの有効化 */
	sil_orw((void*)RCC_CR, CR_PLL_ON);

	/* PLL有効待ち */
	while ((sil_rew_mem((void*)RCC_CR) & CR_PLL_RDY) == 0)
		;

	/* PLLをシステムクロックに選択 */
	sil_orw((void*)RCC_CFGR, CFGR_SW_PLL);

	/* PLLのシステムクロック選択待ち */
	while ((sil_rew_mem((void*)RCC_CFGR) & CFGR_SWS_MASK) != (CFGR_SW_PLL << 2))
		;
#endif
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/*
	 *  プロセッサ依存部の初期化
	 */
	prc_initialize();

#if 0
	/*
	 *  ペリフェラルの有効化
	 */
	sil_orw((void*)RCC_APB2ENR, APB2ENR_USART1_EN | APB2ENR_IOPA_EN |
			APB2ENR_IOPC_EN | APB2ENR_AFIO_EN);
#if (TNUM_PORT >= 2)
	sil_orw((void*)RCC_APB2ENR, APB2ENR_IOPD_EN);
	sil_orw((void*)RCC_APB1ENR, APB1ENR_USART2_EN);
#endif
	/*
	 *  I/Oポートの初期化
	 */
	/* USART1(RX)  プルアップ */
	set_cr_mode(GPIOA_BASE, 10, MODE_INPUT);
	set_cr_cnf(GPIOA_BASE, 10, CNF_IN_FLOATING);

	/* USART1(TX) */
	set_cr_mode(GPIOA_BASE, 9, MODE_OUTPUT_50MHZ);
	set_cr_cnf(GPIOA_BASE, 9, CNF_OUT_AF_PP);

#if (TNUM_PORT >= 2)
	/* USART2(RX)  プルアップ */
	set_cr_mode(GPIOD_BASE, 6, MODE_INPUT);
	set_cr_cnf(GPIOD_BASE, 6, CNF_IN_FLOATING);

	/* USART2(TX) */
	set_cr_mode(GPIOD_BASE, 5, MODE_OUTPUT_50MHZ);
	set_cr_cnf(GPIOD_BASE, 5, CNF_OUT_AF_PP);

	/* USART2ポートのリマップ（PD5,6） */
	sil_orw((void*)AFIO_MAPR, MAPR_USART2_REMAP);
#endif
	/* LEDポート */
	set_cr_mode(GPIOC_BASE, 6, MODE_OUTPUT_50MHZ);
	set_cr_cnf(GPIOC_BASE, 6, CNF_OUT_GP_PP);
#endif

	/*
	 *  バナー出力用のシリアル初期化
	 */
	//target_low_output_init(SIO_PORTID);
	usart_early_init();
}

/*
 * ターゲット依存部 終了処理
 */
void target_exit(void)
{
	/* プロセッサ依存部の終了処理 */
	prc_terminate();
	
	while(true)
		;
}

static UART_HandleTypeDef UartHandle;

static void usart_early_init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	__HAL_RCC_USART2_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**USART2 GPIO Configuration
	PA2     ------> USART2_TX
	PA3     ------> USART2_RX
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	UartHandle.Instance = USART2;
	UartHandle.Init.BaudRate = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits = UART_STOPBITS_1;
	UartHandle.Init.Parity = UART_PARITY_NONE;
	UartHandle.Init.Mode = UART_MODE_TX_RX;
	UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		Error_Handler();
	}
}

/*
 * システムログの低レベル出力のための文字出力
 */
void target_fput_log(char c)
{
	char cr = '\r';
	if (c == '\n') {
		//sio_pol_snd_chr('\r', SIO_PORTID);
		HAL_UART_Transmit(&UartHandle, (uint8_t *)&cr, 1, 0xFFFF);
	}
	//sio_pol_snd_chr(c, SIO_PORTID);
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&c, 1, 0xFFFF);
}

/*
 * 初期化時のエラー発生時の処理
 */
void
Error_Handler(void){
	volatile int loop;
	while(1){
		for(loop = 0; loop < 0x100000; loop++);
	}
}

