/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nlg5_brusa.h"
#include "sdc_can_pwt_db_v1_0.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_CAN1_Init();
    /* USER CODE BEGIN 2 */
    HAL_CAN_Start(&hcan1);

    bool charging = 1;
    float maxvolt = 0;
    float maxcurr = 0;
    uint32_t mailbox;
    uint32_t time = 0;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        CAN_RxHeaderTypeDef rxmsg;
        uint8_t data[8] = {0};

        if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0))
        {
            if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxmsg, data) == HAL_OK)
            {
                if (rxmsg.StdId == SDC_CAN_PWT_DB_V1_0_FBMS_CHARGE_FRAME_ID && rxmsg.DLC == SDC_CAN_PWT_DB_V1_0_FBMS_CHARGE_LENGTH)
                {
                    struct sdc_can_pwt_db_v1_0_fbms_charge_t status;
                    sdc_can_pwt_db_v1_0_fbms_charge_unpack(&status, data, rxmsg.DLC);

                    // charging = sdc_can_pwt_db_v1_0_fbms_charge_fbms_charge_sts_decode(status.fbms_charge_sts);
                    maxvolt = sdc_can_pwt_db_v1_0_fbms_charge_fbms_charge_max_volt_decode(status.fbms_charge_max_volt);
                    maxcurr = sdc_can_pwt_db_v1_0_fbms_charge_fbms_charge_max_curr_decode(status.fbms_charge_max_curr);
                }
            }
        }

        if (HAL_GetTick() - time > 100)
        {
            charging = HAL_GPIO_ReadPin(PON_GPIO_Port, PON_Pin);

            struct nlg5_brusa_nlg5_ctl_t brusa;
            brusa.nlg5_c_c_en = nlg5_brusa_nlg5_ctl_nlg5_c_c_en_encode(charging);
            brusa.nlg5_mc_max = nlg5_brusa_nlg5_ctl_nlg5_mc_max_encode(16);
            brusa.nlg5_ov_com = nlg5_brusa_nlg5_ctl_nlg5_ov_com_encode(maxvolt);
            brusa.nlg5_oc_com = nlg5_brusa_nlg5_ctl_nlg5_oc_com_encode(maxcurr);

            CAN_TxHeaderTypeDef txh = {.DLC = NLG5_BRUSA_NLG5_CTL_LENGTH, .StdId = NLG5_BRUSA_NLG5_CTL_FRAME_ID};
            nlg5_brusa_nlg5_ctl_pack(data, &brusa, NLG5_BRUSA_NLG5_CTL_LENGTH);
            HAL_CAN_AddTxMessage(&hcan1, &txh, data, &mailbox);
            time = HAL_GetTick();
        }

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

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
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 32;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
