/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Комбинированный проект ЛР4–6.
  *                  Таймеры, ШИМ, UART, прерывания, DMA, кнопка.
  *                  STM32F303RE (ARM Cortex-M4, 72 МГц).
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "protocol.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
static volatile uint8_t s_rx_byte;
static volatile uint8_t s_tx_byte;
static volatile bool    s_tx_busy  = false;
static volatile bool    s_dma_busy = false;
static volatile bool    s_rx_dma_mode = false;

TIM_HandleTypeDef htim6;

/* TIM6-debounce: pin «вооружён» (последний триггер EXTI). */
static volatile uint16_t s_btn_pending_pin = 0;
static volatile GPIO_TypeDef *s_btn_pending_port = NULL;
static volatile bool s_tim6_running = false;

/* ЛР4: отключаемое автодемо (sweep duty) — кнопка PC13 (USER) переключает */
static volatile bool     s_lab4_active      = true;
static volatile bool     s_lab4_toggle_req  = false;
static volatile uint32_t s_last_pc13_tick   = 0;

static const uint8_t s_lab4_cfg[][8] = {
    { 1, 3, 0xE8, 0x03, 0, 5, 100, 0 },  /* ch1 PC6  (LD0) */
    { 2, 3, 0xE8, 0x03, 0, 5, 100, 0 },  /* ch2 PC9  (LD1) */
    { 3, 3, 0xE8, 0x03, 0, 5, 100, 0 },  /* ch3 PC7  (LD2) */
    { 4, 3, 0xE8, 0x03, 0, 5, 100, 0 },  /* ch4 PC8  (LD3) */
    { 5, 3, 0xE8, 0x03, 0, 5, 100, 0 },  /* ch5 PA9  (LD4) */
    { 6, 3, 0xE8, 0x03, 0, 5, 100, 0 },  /* ch6 PA8  (LD5) */
    { 7, 3, 0xE8, 0x03, 0, 5, 100, 0 },  /* ch7 PA11 (LD6) */
    { 8, 3, 0xE8, 0x03, 0, 5, 100, 0 },  /* ch8 PA10 (LD7) */
};

static void lab4_start(void) {
    s_lab4_active = true;
    config_apply_buf((const uint8_t *)s_lab4_cfg,
                     sizeof(s_lab4_cfg) / 8u);
}

static void lab4_stop(void) {
    s_lab4_active = false;
    for (uint8_t ch = 1; ch <= PWM_CHANNELS; ++ch) {
        g_sweep[ch - 1].active = false;
        g_pwm[ch - 1].enabled  = false;
        hw_pwm_apply_state(ch);
    }
}

static const struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t timclk;
} CH_MAP[PWM_CHANNELS] = {
    /* timclk = 72 МГц: HSI(8)/PREDIV(1)*PLLMUL(9) = 72 МГц SYSCLK = HCLK = TIMxCLK */
    /* 1 */ { &htim3, TIM_CHANNEL_1, 72000000UL },
    /* 2 */ { &htim3, TIM_CHANNEL_4, 72000000UL },
    /* 3 */ { &htim3, TIM_CHANNEL_2, 72000000UL },
    /* 4 */ { &htim3, TIM_CHANNEL_3, 72000000UL },
    /* 5 */ { &htim1, TIM_CHANNEL_2, 72000000UL },
    /* 6 */ { &htim1, TIM_CHANNEL_1, 72000000UL },
    /* 7 */ { &htim1, TIM_CHANNEL_4, 72000000UL },
    /* 8 */ { &htim1, TIM_CHANNEL_3, 72000000UL },
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t pwm_ccer_bit(uint32_t channel) {
    switch (channel) {
        case TIM_CHANNEL_1: return TIM_CCER_CC1E;
        case TIM_CHANNEL_2: return TIM_CCER_CC2E;
        case TIM_CHANNEL_3: return TIM_CCER_CC3E;
        case TIM_CHANNEL_4: return TIM_CCER_CC4E;
        default: return 0;
    }
}

static void pwm_set_period(TIM_HandleTypeDef *htim, uint32_t timclk, uint32_t freq) {
    if (freq == 0) freq = 1;
    uint32_t psc = (timclk / freq) >> 16;
    uint32_t arr = (timclk / (freq * (psc + 1))) - 1;
    if (arr > 0xFFFEu) arr = 0xFFFEu;
    __HAL_TIM_SET_PRESCALER(htim, psc);
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    /* Принудительное обновление теневых регистров PSC и ARR */
    htim->Instance->EGR = TIM_EGR_UG;
}

static void pwm_apply_duty(TIM_HandleTypeDef *htim, uint32_t ch, uint8_t pct) {
    if (pct > 100u) pct = 100u;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t ccr = ((uint32_t)pct * (arr + 1u)) / 100u;
    if (ccr > 0xFFFFu) ccr = 0xFFFFu;
    __HAL_TIM_SET_COMPARE(htim, ch, ccr);
}

void hw_pwm_apply_freq(uint8_t ch) {
    if (ch < 1 || ch > PWM_CHANNELS) return;
    pwm_set_period(CH_MAP[ch - 1].htim, CH_MAP[ch - 1].timclk, g_pwm[ch - 1].freq_hz);
    pwm_apply_duty(CH_MAP[ch - 1].htim, CH_MAP[ch - 1].channel, g_pwm[ch - 1].duty_pct);
}

void hw_pwm_apply_duty(uint8_t ch) {
    if (ch < 1 || ch > PWM_CHANNELS) return;
    pwm_apply_duty(CH_MAP[ch - 1].htim, CH_MAP[ch - 1].channel, g_pwm[ch - 1].duty_pct);
}

void hw_pwm_apply_state(uint8_t ch) {
    if (ch < 1 || ch > PWM_CHANNELS) return;
    TIM_TypeDef *tim = CH_MAP[ch - 1].htim->Instance;
    uint32_t bit = pwm_ccer_bit(CH_MAP[ch - 1].channel);
    if (g_pwm[ch - 1].enabled) {
        tim->CCER |= bit;
    } else {
        tim->CCER &= ~bit;
    }
}

void hw_pwm_apply_invert(uint8_t ch, bool invert) {
    if (ch < 1 || ch > PWM_CHANNELS) return;
    TIM_TypeDef *tim = CH_MAP[ch - 1].htim->Instance;
    uint32_t channel = CH_MAP[ch - 1].channel;
    volatile uint32_t *ccmr = (channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2)
                              ? &tim->CCMR1 : &tim->CCMR2;
    uint32_t shift = (channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_3) ? 4u : 12u;
    uint32_t mask = 0x7u << shift;
    uint32_t mode = invert ? 0x7u : 0x6u;
    *ccmr = (*ccmr & ~mask) | (mode << shift);
}

void hw_uart_apply_baud(uint32_t baud) {
    HAL_UART_DeInit(&huart3);
    huart3.Init.BaudRate = baud;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&s_rx_byte, 1);
}

void hw_uart_kick_tx(void) {
    if (s_tx_busy || s_dma_busy || proto_dma_active()) return;
    __disable_irq();
    bool got = ring_get(&g_tx_ring, (uint8_t *)&s_tx_byte);
    if (got) s_tx_busy = true;
    __enable_irq();
    if (got) HAL_UART_Transmit_IT(&huart3, (uint8_t *)&s_tx_byte, 1);
}

void hw_uart_tx_flush(void) {
    while (!ring_empty(&g_tx_ring) || s_tx_busy) hw_uart_kick_tx();
    while (!(huart3.Instance->ISR & USART_ISR_TC)) { /* spin */ }
}

bool hw_dma_send(const uint8_t *data, uint16_t size) {
    if (HAL_UART_Transmit_DMA(&huart3, (uint8_t *)data, size) == HAL_OK) {
        s_dma_busy = true;
        return true;
    }
    return false;
}

void hw_dma_abort(void) {
    HAL_UART_DMAStop(&huart3);
    s_dma_busy = false;
}

bool hw_uart_arm_rx_dma(uint8_t *buf, uint16_t size) {
    HAL_UART_AbortReceive(&huart3);
    s_rx_dma_mode = true;
    if (HAL_UART_Receive_DMA(&huart3, buf, size) != HAL_OK) {
        s_rx_dma_mode = false;
        HAL_UART_Receive_IT(&huart3, (uint8_t *)&s_rx_byte, 1);
        return false;
    }
    return true;
}

void hw_uart_disarm_rx_dma(void) {
    if (s_rx_dma_mode) {
        HAL_UART_AbortReceive(&huart3);
        s_rx_dma_mode = false;
    }
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&s_rx_byte, 1);
}

uint32_t hw_tick_ms(void) {
    return HAL_GetTick();
}

/* ────────────────────────── ЛР6: Антидребезг TIM6 ────────────────────────── */

static void tim6_init(void) {
    __HAL_RCC_TIM6_CLK_ENABLE();
    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = 7199;    /* 72 МГц / 7200 = 10 кГц */
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 299;     /* 300 × 0.1 мс = 30 мс */
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();
    __HAL_TIM_URS_ENABLE(&htim6);
    SET_BIT(htim6.Instance->CR1, TIM_CR1_OPM);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void TIM6_DAC_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        HAL_TIM_Base_Stop_IT(&htim6);
        s_tim6_running = false;
        if (s_btn_pending_port) {
            /* Все кнопки: falling edge, pull-up → нажата = пин LOW */
            if (HAL_GPIO_ReadPin((GPIO_TypeDef *)s_btn_pending_port,
                                 s_btn_pending_pin) == GPIO_PIN_RESET) {
                proto_on_button();
            }
        }
        s_btn_pending_port = NULL;
        s_btn_pending_pin  = 0;
    }
}

/* ────────────────────────── ЛР5: UART callbacks ──────────────────────────── */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        if (s_rx_dma_mode) {
            s_rx_dma_mode = false;
            proto_on_config_rx_complete();
        } else {
            proto_feed(s_rx_byte);
            HAL_UART_Receive_IT(huart, (uint8_t *)&s_rx_byte, 1);
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        if (s_dma_busy) {
            s_dma_busy = false;
            proto_on_dma_complete();
        } else {
            s_tx_busy = false;
        }
        hw_uart_kick_tx();
    }
}

/* ────────────────────────── ЛР6: EXTI / Кнопка ───────────────────────────── */

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    /* PC13 (USER button) — переключение автодемо ЛР4 */
    if (pin == GPIO_PIN_13) {
        uint32_t now = HAL_GetTick();
        if (now - s_last_pc13_tick >= 300u) {
            s_last_pc13_tick = now;
            s_lab4_toggle_req = true;
        }
        return;
    }

    /* PC12 (BTN2) — протокол кнопки ЛР6 (антидребезг TIM6) */
    if (pin != GPIO_PIN_12) return;

    if (s_tim6_running) return;
    s_btn_pending_pin  = pin;
    s_btn_pending_port = GPIOC;
    s_tim6_running = true;
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    HAL_TIM_Base_Start_IT(&htim6);
}

static void buttons_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Speed = GPIO_SPEED_FREQ_LOW;

    /* BTN0 (PB3), BTN1 (PD2), BTN3 (PA15) — falling edge, pull-up */
    g.Mode = GPIO_MODE_IT_FALLING;
    g.Pull = GPIO_PULLUP;
    g.Pin = GPIO_PIN_3;  HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_2;  HAL_GPIO_Init(GPIOD, &g);
    g.Pin = GPIO_PIN_15; HAL_GPIO_Init(GPIOA, &g);

    /* BTN2 (PC12) — Вариант 1: falling edge, pull-up
     * (нач. сост. = лог. 1, нажатие = 0) */
    g.Mode = GPIO_MODE_IT_FALLING;
    g.Pull = GPIO_PULLUP;
    g.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &g);

    HAL_NVIC_SetPriority(EXTI3_IRQn,     2, 0);  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, 2, 0);  HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief  PWM running light при запуске — проверка всех 8 каналов.
  */
static void boot_scan(void) {
    for (uint8_t ch = 1; ch <= PWM_CHANNELS; ++ch) {
        TIM_HandleTypeDef *htim = CH_MAP[ch - 1].htim;
        uint32_t channel       = CH_MAP[ch - 1].channel;
        pwm_set_period(htim, CH_MAP[ch - 1].timclk, 1000);
        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
        __HAL_TIM_SET_COMPARE(htim, channel, (arr + 1u) / 2u);
        htim->Instance->EGR = TIM_EGR_UG;
        HAL_TIM_PWM_Start(htim, channel);
        HAL_Delay(120);
        htim->Instance->CCER &= ~pwm_ccer_bit(channel);
        g_pwm[ch - 1].duty_pct = 50;
        g_pwm[ch - 1].enabled  = false;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */

  proto_init();
  for (uint8_t ch = 1; ch <= PWM_CHANNELS; ++ch) {
      hw_pwm_apply_freq(ch);
      /* Запускаем ШИМ на всех каналах: CEN + CCxE. */
      HAL_TIM_PWM_Start(CH_MAP[ch - 1].htim, CH_MAP[ch - 1].channel);
  }
  tim6_init();
  buttons_init();
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&s_rx_byte, 1);
  boot_scan();

  /* ─── ЛР4: Автозапуск демонстрации sweep duty ─── */
  /* Нажатие USER (PC13) переключает: вкл ↔ выкл.
   * Либо полностью отключите, закомментировав строку ниже. */
  lab4_start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ЛР4: обработка переключения автодемо по кнопке PC13 */
    if (s_lab4_toggle_req) {
        s_lab4_toggle_req = false;
        if (s_lab4_active) lab4_stop();
        else               lab4_start();
    }
    proto_task();
    proto_sweep_tick();
    hw_uart_kick_tx();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

/**
  * @brief TIM1 Initialization — каналы 5..8 (PA8, PA9, PA10, PA11)
  */
static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM3 Initialization — каналы 1..4 (PC6, PC7, PC8, PC9)
  */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART3 Initialization — UART 9600 8N1, PC10 TX / PC11 RX
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}

/**
  * @brief DMA Initialization
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
  * @brief GPIO Initialization
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* PC13 — USER button */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Error_Handler(void)
{
  while (1);
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */