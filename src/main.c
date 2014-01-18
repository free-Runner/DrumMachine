/**
  ******************************************************************************
  * @file    LIS302DL/main.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    15-May-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "main.h"
#include "stm32f4xx_conf.h"
#include "utils.h"
#include "Audio.h"
#include "mp3dec.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_lis302dl.h"

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup LIS302DL_Example
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t time_var1, time_var2;
MP3FrameInfo mp3FrameInfo;
HMP3Decoder hMP3Decoder;
int reset_Y = 0;
int reset_X = 0;
int reset_Q = 0;
uint16_t PrescalerValue = 0;
uint8_t Buffer[6];
__IO uint32_t TimingDelay = 0;
__IO int8_t XOffset;
__IO int8_t YOffset;

__IO uint8_t SingleClickDetect = 0x00;
//__IO uint8_t isYPositive = 0x00;
__IO uint8_t play_Y = 0x00;
__IO uint8_t play_X = 0x00;
// External variables
extern uint8_t ClickReg;
extern const char mp3_data1[];
extern const char mp3_data2[];
extern const char mp3_data3[];
static const char *read_ptr;
static int bytes_left;
int offset;

#define MP3_SIZE1  9636
#define MP3_SIZE2  12528
#define MP3_SIZE3   8920
#define BUTTON    (GPIOA->IDR & GPIO_Pin_0)


/* Private function prototypes -----------------------------------------------*/
static void TIM_Config(void);
void AudioCallback(void *context,int buffer);
void Delay(volatile uint32_t nCount);
void init();

/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  uint8_t ctrl = 0;
  
  LIS302DL_InitTypeDef  LIS302DL_InitStruct;
  LIS302DL_InterruptConfigTypeDef LIS302DL_InterruptStruct;  
  
  /* SysTick end of count event each 10ms */
  SysTick_Config(SystemCoreClock/ 100);
  
  /* Set configuration of LIS302DL*/
  LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
  LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
  LIS302DL_InitStruct.Axes_Enable = LIS302DL_X_ENABLE | LIS302DL_Y_ENABLE | LIS302DL_Z_ENABLE;
  LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
  LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
  LIS302DL_Init(&LIS302DL_InitStruct);
    
  /* Set configuration of Internal High Pass Filter of LIS302DL*/
  LIS302DL_InterruptStruct.Latch_Request = LIS302DL_INTERRUPTREQUEST_LATCHED;
  LIS302DL_InterruptStruct.SingleClick_Axes = LIS302DL_CLICKINTERRUPT_Z_ENABLE;
  LIS302DL_InterruptStruct.DoubleClick_Axes = LIS302DL_DOUBLECLICKINTERRUPT_Z_ENABLE;
  LIS302DL_InterruptConfig(&LIS302DL_InterruptStruct);

  /* Required delay for the MEMS Accelerometre: Turn-on time = 3/Output data Rate 
                                                             = 3/100 = 30ms */
  Delay(30);
  
  /* Configure Interrupt control register: enable Click interrupt1 */
  ctrl = 0x07;
  LIS302DL_Write(&ctrl, LIS302DL_CTRL_REG3_ADDR, 1);
  
  /* Enable Interrupt generation on click/double click on Z axis */
  ctrl = 0x70;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_CFG_REG_ADDR, 1);
  
  /* Configure Click Threshold on X/Y axis (10 x 0.5g) */
  ctrl = 0xAA;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_THSY_X_REG_ADDR, 1);
  
  /* Configure Click Threshold on Z axis (10 x 0.5g) */
  ctrl = 0x0A;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_THSZ_REG_ADDR, 1);
  
  /* Configure Time Limit */
  ctrl = 0x03;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_TIMELIMIT_REG_ADDR, 1);
    
  /* Configure Latency */
  ctrl = 0x7F;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_LATENCY_REG_ADDR, 1);
  
  /* Configure Click Window */
  ctrl = 0x7F;
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_WINDOW_REG_ADDR, 1);
  
  /* TIM configuration -------------------------------------------------------*/
  TIM_Config(); 
  LIS302DL_Read(Buffer, LIS302DL_OUT_X_ADDR, 6);
                  
  XOffset = Buffer[0];
  YOffset = Buffer[2];

  // Play mp3
  hMP3Decoder = MP3InitDecoder();
  InitializeAudio(Audio44100HzSettings);
  SetAudioVolume(0x00);
  PlayAudioWithCallback(AudioCallback, 0);
  //Aman: This calls the callback function that starts the song
  
    
  while(1)
  {

    if (play_X != 0){
        // play_X = 0;
        //reset_X = 0;
        SetAudioVolume(0x00);
        //reset_Y= 1;
        read_ptr = mp3_data1;
        bytes_left = MP3_SIZE1;
        offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
        SetAudioVolume(0x0);
        SetAudioVolume(0xCF);
        play_X = 0;
    }
    else if (play_Y != 0){
        // play_Y = 0;
        // reset_Y = 0;
        SetAudioVolume(0x0);
        //reset_X= 1;
		uint8_t volume;
        read_ptr = mp3_data2;
        bytes_left = MP3_SIZE2;
        offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
        SetAudioVolume(0x0);
		if (play_Y > (0x50))
			volume = 0xFF;
		else
			volume = 0xAF+play_Y;
        SetAudioVolume(volume);
        play_Y = 0;
    }

    // if (SingleClickDetect != 0)
    // //if (*BSRRL)
    // {

    //   //Delay(50);
    //   /* Read click status register */
    //   //LIS302DL_Read(&ClickReg, LIS302DL_CLICK_SRC_REG_ADDR, 1); 
    //     /*
    //     /* Enable TIM4 Capture Compare Channel 1 */
    //     TIM_CCxCmd(TIM4, TIM_Channel_1, ENABLE);
    //     /* Sets the TIM4 Capture Compare1 Register value */
    //     TIM_SetCompare1(TIM4, TIM_CCR/12);
    //     /* Enable TIM4 Capture Compare Channel 2 */
    //     TIM_CCxCmd(TIM4, TIM_Channel_2, ENABLE);
    //     /* Sets the TIM4 Capture Compare2 Register value */
    //     TIM_SetCompare2(TIM4, TIM_CCR/12);
    //     /* Enable TIM4 Capture Compare Channel 3 */
    //     TIM_CCxCmd(TIM4, TIM_Channel_3, ENABLE);
    //     /* Sets the TIM4 Capture Compare3 Register value */
    //     TIM_SetCompare3(TIM4, TIM_CCR/12);
    //     /* Enable TIM4 Capture Compare Channel 4 */
    //     TIM_CCxCmd(TIM4, TIM_Channel_4, ENABLE);
    //     /* Sets the TIM4 Capture Compare4 Register value */
    //     TIM_SetCompare4(TIM4, TIM_CCR/12);
        
    //     /* Time base configuration */
    //     TIM_SetAutoreload(TIM4, TIM_ARR/12);
    //     //Delay(50);  
        
    //     /* Clear the click status register by reading it */
    //     LIS302DL_Read(&ClickReg, LIS302DL_CLICK_SRC_REG_ADDR, 1);
    //     //SetAudioVolume(0xCF);
    //     //reset=1;
        
    //     /* Reset the single click detect */
    //     SingleClickDetect = 0x00;
    // }
  }
}

/*
 * Called by the audio driver when it is time to provide data to
 * one of the audio buffers (while the other buffer is sent to the
 * CODEC using DMA). One mp3 frame is decoded at a time and
 * provided to the audio driver.
 */
void AudioCallback(void *context, int buffer) {
  static int16_t audio_buffer0[4096];
  static int16_t audio_buffer1[4096];

  int err;
  int outOfData = 0;
  // static const char *read_ptr;
  // static int bytes_left;

  int16_t *samples;

  if (buffer) {
    samples = audio_buffer0;
    GPIO_SetBits(GPIOD, GPIO_Pin_13);
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
  } else {
    samples = audio_buffer1;
    GPIO_SetBits(GPIOD, GPIO_Pin_14);
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);
  }

  offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
  bytes_left -= offset;

  /* Original code that repeats mp3 continuously
  if (bytes_left <= 10000) {
    read_ptr = mp3_data;
    bytes_left = MP3_SIZE;
    offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
  }
  */

  if (bytes_left <= 1000) {

    SetAudioVolume(0x00); //Set valuem to 0   
    //reset_X = 1;
    reset_Q = 1;
  }

  // if (reset_Y){
  //   reset_Y = 0;
  //   read_ptr = mp3_data1;
  //   bytes_left = MP3_SIZE1;
  //   offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
  // }
  // else if (reset_X){
  //   reset_X = 0;
  //   read_ptr = mp3_data2;
  //   bytes_left = MP3_SIZE2;
  //   offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
  // }
  if (reset_Q){
    reset_Q = 0;
    read_ptr = mp3_data3;
    bytes_left = MP3_SIZE3;
    offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
  }
  

  read_ptr += offset;
  err = MP3Decode(hMP3Decoder, (unsigned char**)&read_ptr, &bytes_left, samples, 0);

  if (err) {
    /* error occurred */
    switch (err) {
    case ERR_MP3_INDATA_UNDERFLOW:
      outOfData = 1;
      break;
    case ERR_MP3_MAINDATA_UNDERFLOW:
      /* do nothing - next call to decode will provide more mainData */
      break;
    case ERR_MP3_FREE_BITRATE_SYNC:
    default:
      outOfData = 1;
      break;
    }
  } else {
    /* no error */
    MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);
  }

  if (!outOfData) {
    ProvideAudioBuffer(samples, mp3FrameInfo.outputSamps);

  }
}

void init() {
  GPIO_InitTypeDef  GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  // ---------- SysTick timer -------- //
  //if (SysTick_Config(SystemCoreClock / 1000)) {
    // Capture error
  //  while (1){};
  //}

  // Enable full access to FPU (Should be done automatically in system_stm32f4xx.c):
  //SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  // set CP10 and CP11 Full Access

  // GPIOD Periph clock enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  // Configure PD12, PD13, PD14 and PD15 in output pushpull mode
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);


  // ------ UART ------ //

  // Clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  // IO
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART1);

  // Conf
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART2, &USART_InitStructure);

  // Enable
  USART_Cmd(USART2, ENABLE);
}

/**
  * @brief  Configures the TIM Peripheral.
  * @param  None
  * @retval None
  */
static void TIM_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  
  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  
  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
  /*-------------------------- GPIO Configuration ----------------------------*/
  /* GPIOD Configuration: Pins 12, 13, 14 and 15 in output push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Connect TIM4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); 
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4); 
  
    /* -----------------------------------------------------------------------
    TIM4 Configuration: Output Compare Timing Mode:
    
    In this example TIM4 input clock (TIM4CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1 (APB1 Prescaler = 4, see system_stm32f4xx.c file).
      TIM4CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM4CLK = 2*(HCLK / 4) = HCLK/2 = SystemCoreClock/2
         
    To get TIM4 counter clock at 2 KHz, the prescaler is computed as follows:
       Prescaler = (TIM4CLK / TIM1 counter clock) - 1
       Prescaler = (168 MHz/(2 * 2 KHz)) - 1 = 41999
                                        
    To get TIM4 output clock at 1 Hz, the period (ARR)) is computed as follows:
       ARR = (TIM4 counter clock / TIM4 output clock) - 1
           = 1999
                    
    TIM4 Channel1 duty cycle = (TIM4_CCR1/ TIM4_ARR)* 100 = 50%
    TIM4 Channel2 duty cycle = (TIM4_CCR2/ TIM4_ARR)* 100 = 50%
    TIM4 Channel3 duty cycle = (TIM4_CCR3/ TIM4_ARR)* 100 = 50%
    TIM4 Channel4 duty cycle = (TIM4_CCR4/ TIM4_ARR)* 100 = 50%
    
    ==> TIM4_CCRx = TIM4_ARR/2 = 1000  (where x = 1, 2, 3 and 4).
  
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
  ----------------------------------------------------------------------- */ 
    
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 2000) - 1;
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = TIM_ARR;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  /* Enable TIM4 Preload register on ARR */
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  
  /* TIM PWM1 Mode configuration: Channel */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = TIM_CCR;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  
  /* Output Compare PWM1 Mode configuration: Channel1 */
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_CCxCmd(TIM4, TIM_Channel_1, DISABLE);
  
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
  
  /* Output Compare PWM1 Mode configuration: Channel2 */
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_CCxCmd(TIM4, TIM_Channel_2, DISABLE);
  
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
  /* Output Compare PWM1 Mode configuration: Channel3 */
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_CCxCmd(TIM4, TIM_Channel_3, DISABLE);
  
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  
  /* Output Compare PWM1 Mode configuration: Channel4 */
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_CCxCmd(TIM4, TIM_Channel_4, DISABLE);
  
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
  
  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

/**
  * @brief  MEMS accelerometre management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LIS302DL_TIMEOUT_UserCallback(void)
{
  /* MEMS Accelerometer Timeout error occured */
  while (1)
  {   
  }
}

/*
 * Dummy function to avoid compiler error
 */
void _init() {

}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}


#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
