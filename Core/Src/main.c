/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 *@file           : main.c
 *@brief          : Main program body
 ******************************************************************************
 *@attention
 *
 *Copyright (c) 2022 STMicroelectronics.
 *All rights reserved.
 *
 *This software is licensed under terms that can be found in the LICENSE file
 *in the root directory of this software component.
 *If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

#include "../../modbus/mb.h"
#include "../../modbus/mbport.h"
#define REG_HOLDING_START 1
#define REG_HOLDING_NREGS 22
static USHORT usRegHoldingStart = REG_HOLDING_START;
static USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

enum
{
    HREG_DCMOTOR_RUN,
    HREG_DCMOTOR_DIRECTION,
    HREG_DCMOTOR_SET_ELAPSED_TIME,
    HREG_DCMOTOR_SET_SPEED,
    HREG_DCMOTOR_SET_LIMIT_VOLTAGE,
    HREG_DCMOTOR_SET_LIMIT_CURRENT,
    HREG_DCMOTOR_GET_CURRENT_SPEED,
    HREG_DCMOTOR_GET_VOLTAGE,
    HREG_DCMOTOR_GET_CURRENT,
    HREG_DCMOTOR_GET_RESISTANCE,
    HREG_DCMOTOR_GET_TEMPERATURE,
    HREG_MODBUS_ADDRESS,
    HREG_MODBUS_BAUDRATE
} holdingRegs_t;

#define SECTOR_MODBUS_ADDRESS 1
#include "../../w25qxx/w25qxx.h"

#include "../../MedianFilter/MedianFilter.h"

/**
 *@brief Get the Modbus Device Address object. 
 *If not present, default address = 80.
 * 
 *@return int
 */
int getDeviceAddress()
{
    uint8_t mb_default_address[] = "80";
    uint8_t sector_cont[sizeof(mb_default_address) - 1];
    if (W25qxx_IsEmptySector(SECTOR_MODBUS_ADDRESS, sizeof(mb_default_address), sizeof(mb_default_address) - 1) == false)
        W25qxx_ReadSector(sector_cont, SECTOR_MODBUS_ADDRESS, sizeof(mb_default_address), sizeof(mb_default_address) - 1);

    else memcpy(sector_cont, mb_default_address, sizeof(mb_default_address) - 1);

    uint8_t v[sizeof(sector_cont)];
    memcpy(v, sector_cont, sizeof(sector_cont));
    return atoi(v);
}

/**
 *@brief Write the Modbus Device Address object.
 * 
 *@param new_address 
 */
void writeDeviceAddress(char new_address[])
{
    W25qxx_EraseSector(SECTOR_MODBUS_ADDRESS);
    W25qxx_WriteSector(new_address, SECTOR_MODBUS_ADDRESS, SECTOR_MODBUS_ADDRESS + strlen(new_address), strlen(new_address));

    HAL_Delay(50);
   	Error_Handler();
}

uint32_t millis()
{
    return HAL_GetTick();
}

// void millis(uint16_t us)
// {
//     __HAL_TIM_SET_COUNTER(&htim17, 0);
//     while (__HAL_TIM_GET_COUNTER(&htim17) < us);
// }

uint16_t rawCurrent, rawVoltage;
volatile int adcSwitch = 1;

#include "../../DCMotor/DCMotor.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if(htim->Instance == TIM17)
    {
        adcSwitch = !adcSwitch;
        if (adcSwitch == 1)
        {
            rawVoltage = HAL_ADC_GetValue(&sensor_adc);
            HAL_ADC_Stop(&sensor_adc);
            DCMotor_GetSensor(0, 100);
        }
        else
        {
            rawCurrent = HAL_ADC_GetValue(&sensor_adc);
            HAL_ADC_Stop(&sensor_adc);
            DCMotor_GetSensor(1, 100);
        }
    }
}

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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

    // printf("hello\r\n");
    HAL_ADCEx_Calibration_Start(&hadc1);
    W25qxx_Init();

    HAL_GPIO_WritePin(GATE_PWR_ENA_GPIO_Port, GATE_PWR_ENA_Pin, 1);

    DCMotor_Init();
    // DCMotor_CalibrateCurrent(30, 50);

    // int i_max = DCMotor_GetMaxCurrentFilter();
    // DCMotor_SetCurrentLimit(i_max);
    // printf("i max: %ld\r\n", i_max);
    // HAL_Delay(2000);

    // MedianFilter_In(motor.current_filter, HAL_ADC_GetValue(&sensor_adc));

    for (size_t i = 0; i < REG_HOLDING_NREGS; i++) usRegHoldingBuf[i] = 0;
    eMBInit(MB_RTU, 2, 3, 9600, MB_PAR_NONE);
    eMBEnable();

    // DCMotor_SetAcceleration(1000);

    // DCMotor_SetDirection(0);
    // DCMotor_SetSpeed(30);
    // DCMotor_Update();
    // DCMotor_Stop();

    // DCMotor_SetDirection(1);
    // DCMotor_SetSpeed(30);
    // DCMotor_Update();
    // DCMotor_Stop();

    // HAL_Delay(10);
    // HAL_TIM_Base_Start_IT(&htim17);
    // uint32_t previousMillis = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        // printf("adc0 = %d\tadc1 = %d\r\n", rawCurrent, rawVoltage);

        // MedianFilter_In(motor.current_filter, DCMotor_GetCurrentValue());
        // int i_out = MedianFilter_Out(motor.current_filter);
        // printf("v:%ld\t i:%ld\t i_max:%ld\r\n", DCMotor_GetVoltageValue(), i_out, i_max);

        // if (i_max < i_out)
        // {
        //     DCMotor_Brake();
        //     printf("hit max current\r\n");
        //     break;
        // }

        // HAL_Delay(50);

        // uint32_t currentMillis = millis();
        // if (currentMillis - previousMillis >= 1000)
        // {
        //     previousMillis = currentMillis;
        //     printf("fil curr: %i\r\n", filter->out(filter));
        //     // printf("%i\r\n", filter->getMax(filter));
        //     printf("nofil curr: %f\r\n", res);
        //     printf("\r\n");
        // }

        // // printf("\r\n");
        // HAL_Delay(100);

        eMBPoll();
        DCMotor_Update();

        // if (usRegHoldingBuf[HREG_DCMOTOR_RUN] == 1) DCMotor_Update();
        // else
        // {
        //     DCMotor_Stop();
        //     DCMotor_Reset();
        // }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 *@brief Fetching modbus input register data.
 *
 *@param iRegIndex
 *@return uint8_t 
 */
uint8_t fetchInputRegsData(int iRegIndex)
{
    uint8_t numUs = 1;
    return numUs;
}

/**
 *@brief Fetching modbus holding register data.
 *
 *@param iRegIndex 
 *@return uint8_t 
 */
uint8_t fetchHoldingRegsData(int iRegIndex)
{
    uint8_t numUs = 1;
    switch (iRegIndex)
    {
        case HREG_DCMOTOR_RUN:
            if (usRegHoldingBuf[HREG_DCMOTOR_RUN] == 0)
            {
                DCMotor_Stop();
            }
        case HREG_DCMOTOR_DIRECTION:
            usRegHoldingBuf[HREG_DCMOTOR_DIRECTION] = DCMotor_GetDirectionValue();
            break;
        case HREG_DCMOTOR_SET_ELAPSED_TIME:
            usRegHoldingBuf[HREG_DCMOTOR_SET_ELAPSED_TIME] = DCMotor_GetElapsedTime();
            break;
        case HREG_DCMOTOR_SET_SPEED:
            break;
        case HREG_DCMOTOR_SET_LIMIT_VOLTAGE:
            usRegHoldingBuf[HREG_DCMOTOR_SET_LIMIT_VOLTAGE] = DCMotor_GetAcceleration();
            break;
        case HREG_DCMOTOR_SET_LIMIT_CURRENT:
            usRegHoldingBuf[HREG_DCMOTOR_SET_LIMIT_CURRENT] = DCMotor_GetFilteredCurrent();
            break;
        case HREG_DCMOTOR_GET_CURRENT_SPEED:
            usRegHoldingBuf[HREG_DCMOTOR_GET_CURRENT_SPEED] = DCMotor_GetCurrentSpeed();
            break;
        case HREG_DCMOTOR_GET_VOLTAGE:
            usRegHoldingBuf[HREG_DCMOTOR_GET_VOLTAGE] = DCMotor_GetVoltageValue();
            break;
        case HREG_DCMOTOR_GET_CURRENT:
            usRegHoldingBuf[HREG_DCMOTOR_GET_CURRENT] = DCMotor_GetFilteredCurrent();
            break;
        case HREG_DCMOTOR_GET_RESISTANCE:
            usRegHoldingBuf[HREG_DCMOTOR_GET_RESISTANCE] = DCMotor_GetResistanceValue();
            break;
        case HREG_DCMOTOR_GET_TEMPERATURE:
            break;
        case HREG_MODBUS_ADDRESS:
            break;
        case HREG_MODBUS_BAUDRATE:
            break;
        default:
            numUs = 0;
            break;
    }
    return numUs;
}

/**
 *@brief Writing modbus holding registers.
 *
 *@param iRegIndex 
 *@param tempReg
 */
void writeHoldingRegs(int iRegIndex, uint16_t tempReg)
{
    switch (iRegIndex)
    {
        case HREG_DCMOTOR_RUN:
            if (tempReg > 1) break;
            usRegHoldingBuf[HREG_DCMOTOR_RUN] = tempReg;
            DCMotor_Start(tempReg);
            break;
        case HREG_DCMOTOR_DIRECTION:
            if (tempReg > 1) break;
            usRegHoldingBuf[HREG_DCMOTOR_DIRECTION] = tempReg;
            DCMotor_SetDirection(tempReg);
            DCMotor_Stop();
            break;
        case HREG_DCMOTOR_SET_ELAPSED_TIME:
            usRegHoldingBuf[HREG_DCMOTOR_SET_ELAPSED_TIME] = tempReg;
            DCMotor_SetElapsedTime(tempReg);
            break;
        case HREG_DCMOTOR_SET_SPEED:
            if (tempReg > 100) break;
            usRegHoldingBuf[HREG_DCMOTOR_SET_SPEED] = tempReg;
            DCMotor_SetSpeed(tempReg);
            break;
        case HREG_DCMOTOR_SET_LIMIT_VOLTAGE:
            // usRegHoldingBuf[HREG_DCMOTOR_SET_LIMIT_VOLTAGE] = tempReg;
            break;
        case HREG_DCMOTOR_SET_LIMIT_CURRENT:
            if (tempReg == 1)
            {
                dc.motor_current_limit = 1;
            }
            usRegHoldingBuf[HREG_DCMOTOR_SET_LIMIT_CURRENT] = tempReg;
            break;
        // case HREG_MODBUS_ADDRESS:
        //     break;
        // case HREG_MODBUS_BAUDRATE:
        //     break;

        // case HREG_NEW_MODBUS_ADDRESS:
        //     if (tempReg < 10 || tempReg > 99) break;
        //     usRegHoldingBuf[HREG_NEW_MODBUS_ADDRESS] = tempReg;
        //     char value[3];
        //    	writeDeviceAddress(itoa(usRegHoldingBuf[HREG_NEW_MODBUS_ADDRESS], value, 10));
        //     break;
        
        default:
           	// usRegHoldingBuf[iRegIndex] = tempReg;
            break;
    }
}

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    if (eMode == MB_REG_READ)
    {
        if ((usAddress >= REG_HOLDING_START) &&
            (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
        {
            iRegIndex = (int)(usAddress - usRegHoldingStart);
            while (usNRegs > 0)
            {
                uint8_t numUs = fetchHoldingRegsData(iRegIndex);
                if (numUs < 1)
                {
                    return MB_ENORES;
                }

                for (size_t i = 0; i < numUs; ++i)
                {
                    if (usNRegs > 0)
                    {
                        *pucRegBuffer++ = (unsigned char)(usRegHoldingBuf[iRegIndex] >> 8);
                        *pucRegBuffer++ = (unsigned char)(usRegHoldingBuf[iRegIndex] &0xFF);
                        iRegIndex++;
                        usNRegs--;
                    }
                    else
                    {
                        return MB_ENORES;
                    }
                }
            }
        }
        else
        {
            eStatus = MB_ENOREG;
        }
    }

    if (eMode == MB_REG_WRITE)
    {
        if ((usAddress >= REG_HOLDING_START) && (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
        {
            iRegIndex = (int)(usAddress - usRegHoldingStart);

            while (usNRegs > 0)
            {
                writeHoldingRegs(iRegIndex, (USHORT)(((unsigned int) *pucRegBuffer << 8) | ((unsigned int) *(pucRegBuffer + 1))));
                pucRegBuffer += 2;
                usNRegs--;
                iRegIndex++;
            }
        }
        else eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    return MB_ENOREG;
}

eMBErrorCode eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
    return MB_ENOREG;
}

eMBErrorCode eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
    return MB_ENOREG;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /*User can add his own implementation to report the HAL error return state */
    __disable_irq();
    HAL_NVIC_SystemReset();
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
    /*User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
