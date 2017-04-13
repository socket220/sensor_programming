// pins for i2c:SCL SDA
//				PB8 PB9

#ifdef __cplusplus
extern "C"
{
#endif
	#include <sys/stat.h>
	#include <usbd_core.h>
	#include <usbd_cdc.h>
	#include "usbd_cdc_if.h"
	#include <usbd_desc.h>
	#include "stm32f4xx_hal_tim.h"
	
	#include "tm_stm32_mpu9250.h"

	USBD_HandleTypeDef USBD_Device;
	void SysTick_Handler(void);
	void OTG_FS_IRQHandler(void);
	void OTG_HS_IRQHandler(void);
	extern PCD_HandleTypeDef hpcd;
	
	//int VCP_read(void *pBuffer, int size);
	//int VCP_write(const void *pBuffer, int size);
	uint8_t CDC_Receive(uint8_t* Buf, int Len);
	uint8_t CDC_Transmit(void *Buf, int Len);
	extern char g_VCPInitialized;
	
	TM_MPU9250_t MPU9250; // sensor structure
	
}
#include "MahonyAHRS.h"
#ifdef __cplusplus

#endif

static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	__PWR_CLK_ENABLE();
	//__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;//8
	RCC_OscInitStruct.PLL.PLLN = 336;//336
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;//7
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK |
	                               RCC_CLOCKTYPE_HCLK |
	                               RCC_CLOCKTYPE_PCLK1 |
	                               RCC_CLOCKTYPE_PCLK2);

	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;//1
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;//2
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);//5
	
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 3000);//8000//wtf???????
	
	//HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	
	//HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}



void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

#ifdef USE_USB_FS
void OTG_FS_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&hpcd);
}
#elif defined(USE_USB_HS)
void OTG_HS_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&hpcd);
}
#else
#error USB peripheral type not defined
#endif

/*
	This is a basic USB CDC device sample based on the ST USB library.
	To test it out:
		1. Install the ST VCP drivers (http://www.st.com/web/en/catalog/tools/PF257938)
		2. Connect to the virtual COM port using SmarTTY or any other terminal program
		3. Type some characters and observe the output
	Read more about the sample: http://visualgdb.com/tutorials/arm/stm32/usb/
*/

static TIM_HandleTypeDef htim4;

void TIM4_Init(void)
{
	
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
	TIM_OC_InitTypeDef sConfigOC;
 
	__TIM4_CLK_ENABLE();
	
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 7;//Prescaler = ClockSource/(PWM frequency*PWM resolution) – 1 // source is apb1 i thnink
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 10000;//
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&htim4);
 
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);
 
	HAL_TIM_PWM_Init(&htim4);
 
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);
 
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	HAL_TIMEx_ConfigBreakDeadTime(&htim4, &sBreakDeadTimeConfig);
 
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 525;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);
 
}

int main(void)
{
	
	HAL_Init();
	SystemClock_Config();
	USBD_Init(&USBD_Device, &VCP_Desc, 0);

	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_sensor_programming_fops);
	USBD_Start(&USBD_Device);
	
	TM_MPU9250_Init(&MPU9250, TM_MPU9250_Device_0); //zbs
	
	Mahony filter;
	filter.begin(10);

	char byte;
	uint8_t count = 0;
	char str[30];
	
	/*
	__GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = GPIO_PIN_15;

	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	*/
	
	for (;;)
	{
		
		TM_MPU9250_ReadAcce(&MPU9250);
		HAL_Delay(10);
		TM_MPU9250_ReadGyro(&MPU9250);
		HAL_Delay(10);
		TM_MPU9250_ReadMag(&MPU9250);
		
		//count  = sprintf(str, "%.3f %.3f %.3f | %.3f %.3f %.3f | %.4i %.4i %.4i \r\n", MPU9250.Gx, MPU9250.Gy, MPU9250.Gz,
		//MPU9250.Ax, MPU9250.Ay, MPU9250.Az, MPU9250.Mx_Raw, MPU9250.My_Raw, MPU9250.Mz_Raw);
		
		filter.update(MPU9250.Gx, MPU9250.Gy, MPU9250.Gz, MPU9250.Ax, MPU9250.Ay, MPU9250.Az, MPU9250.Mx, MPU9250.My, MPU9250.Mz);
		//filter.updateIMU(MPU9250.Gx, MPU9250.Gy, MPU9250.Gz, MPU9250.Ax, MPU9250.Ay, MPU9250.Az);
		
		count = sprintf(str, "%.2f %.2f %.2f \r\n", filter.getRoll(), filter.getPitch(), filter.getYaw());
		
		CDC_Transmit(&str, count);
		HAL_Delay(80);
		
		/*
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_Delay(1000);
		*/
	}
}