/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_audio_if.c
  * @version        : v1.0_Cube
  * @brief          : Generic media access layer.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usbd_audio_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_AUDIO_IF
  * @{
  */

/** @defgroup USBD_AUDIO_IF_Private_TypesDefinitions USBD_AUDIO_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Defines USBD_AUDIO_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Macros USBD_AUDIO_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Variables USBD_AUDIO_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Exported_Variables USBD_AUDIO_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_FunctionPrototypes USBD_AUDIO_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t AUDIO_Init_FS(uint32_t AudioFreq, uint32_t Volume, uint32_t options);
static int8_t AUDIO_DeInit_FS(uint32_t options);
static int8_t AUDIO_AudioCmd_FS(uint8_t* pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_VolumeCtl_FS(uint8_t vol);
static int8_t AUDIO_MuteCtl_FS(uint8_t cmd);
static int8_t AUDIO_PeriodicTC_FS(uint8_t *pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_GetState_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

#define nsa (AUDIO_OUT_PACKET*3)
#define OSR 14
/*#define period 174
  #define period3 173*/
#define period 124
#define period3 125
#define NDA 3
//#define period (500/OSR-1)
#define period2 ((period+2)>>1)
volatile uint16_t data[NDA*nsa*OSR];
volatile int iii=0;
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops_FS =
{
  AUDIO_Init_FS,
  AUDIO_DeInit_FS,
  AUDIO_AudioCmd_FS,
  AUDIO_VolumeCtl_FS,
  AUDIO_MuteCtl_FS,
  AUDIO_PeriodicTC_FS,
  AUDIO_GetState_FS,
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the AUDIO media low layer over USB FS IP
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  options: Reserved for future use
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_Init_FS(uint32_t AudioFreq, uint32_t Volume, uint32_t options)
{
  /* USER CODE BEGIN 0 */

 for(int i=0;i<nsa*OSR;i++)
   data[i*NDA]=data[i*NDA+1]=data[i*NDA+2]=period2;
   //   dDAata[dDA*i+0]=period,data[NDA*i+2]=data[NDA*i+3]=data[NDA*i+4]=period2,data[NDA*i+5]=i*NDA;

    RCC->AHB1ENR|=(1<<21)|(1<<22); //dma1 & dma2 clock enable
    RCC->AHB1ENR|=1<<0;//GPIOA enable
    RCC->AHB1ENR|=(1<<1);//GPIOB GPIOCenable
    
  RCC->APB1ENR|=1<<1;//tim3 enable
  RCC->APB2ENR|=1<<0;//tim1 enable
  GPIOA->MODER|=(2<<14)|(2<<16)|(2<<18)|(2<<20)|(2<<12);//|(2<<22);// PA8-PA10
  GPIOA->AFR[0]|=(2<<24)|(2<<28);
  GPIOA->AFR[1]|=(1<<0)|(1<<4)|(1<<8);//|(1<<12);// PA8-PA10 AF1 TIM1
  GPIOB->MODER|=(2<<26)|(2<<28)|(2<<30);// PB13 and PB15
  GPIOB->AFR[1]|=(1<<20)|(1<<24)|(1<<28);// PA6,PA7 AF2 TIM2...3


   TIM1->DMAR=0;

  //TIM1->CR2=1<<4;// MMS2 001 ENABLE
  TIM1->PSC=0;
  TIM1->ARR=period;
  TIM1->CCR1=TIM1->CCR2=TIM1->CCR3=period>>1;
  TIM1->DIER=0x1f<<8;// TIM3_U DMA ENABLE
  //TIM1->DIER|=1<<12;// TIM3_CC1
  TIM1->CCMR1=(6<<4)|(6<<12)|(1<<3);//|(1<<11); // OCM1 PWM1 MODE
  TIM1->CCMR2=(6<<4)|(6<<12)|(1<<3);//|(1<<11); // OCM1 PWM1 MODE
  //  TIM1->CCER=(1<<0);//CC1enable
  TIM1->CCER=(5<<0)|(5<<4)|(5<<8);//|(1<<12);//CC2ENABLE cc2Pol
  TIM1->DCR=((NDA-1)<<8); // DBL DMA BURST LENGTH 5=DBL+1 
  TIM1->DCR|=(0xD<<0);//DMA BASE ADDRESS 0xB ARR, 0xd OCR1, 0xe ocr2
  TIM1->BDTR|=1<<15;
  TIM1->CR1=1<<7; //AUTO-reload preload enable (buffered ARR)
  TIM1->CR1|=1;


  
  TIM3->PSC=0;
  TIM3->ARR=874;
  //  TIM3->CCR1=TIM1->CCR2=TIM1->CCR3=period>>1;
  //  TIM->DIER=0x1f<<8;// TIM3_U DMA ENABLE
  //TIM1->DIER|=1<<12;// TIM3_CC1
  TIM3->CCMR1=(6<<4)|(6<<12)|(1<<3);//|(1<<11); // OCM1 PWM1 MODE
  TIM3->CCMR2=(6<<4)|(6<<12)|(1<<3);//|(1<<11); // OCM1 PWM1 MODE
  //  TIM1->CCER=(1<<0);//CC1enable
  TIM3->CCER=(5<<0)|(5<<4)|(5<<8);//|(1<<12);//CC2ENABLE cc2Pol
  //  TIM1->DCR=((NDA-1)<<8); // DBL DMA BURST LENGTH 5=DBL+1 
  //  TIM1->DCR|=(0xD<<0);//DMA BASE ADDRESS 0xB ARR, 0xd OCR1, 0xe ocr2
  //  TIM1->BDTR|=1<<15;
  TIM3->CR1=1<<7; //AUTO-reload preload enable (buffered ARR)
  TIM3->CR1|=1;

  
DMA2_Stream3->NDTR=nsa*NDA*OSR  ; //N
DMA2_Stream3->PAR=(uint32_t)&TIM1->DMAR; //
DMA2_Stream3->M0AR=(uint32_t)data;
DMA2_Stream3->M1AR=(uint32_t)&data[nsa*NDA*OSR];
DMA2_Stream3->CR=6<<25;  //chan6 tim1_ch1
// DMA2_Stream3->CR|=1<<18; //double buffered
DMA2_Stream3->CR|=1<<4; //enable TC interrupt
DMA2_Stream3->CR|=(1<<13)|(1<<11); //MSIZE 1 and PSIZE 1 16bit
DMA2_Stream3->CR|=1<<10; //MINC 1 increment memory
DMA2_Stream3->CR|=1<<8;//CIRC circular mode
DMA2_Stream3->CR|=1<<6;//DIR Memory to peripheral
DMA2_Stream3->CR|=1; // enable DMA2_Stream2

NVIC_SetPriority(DMA2_Stream3_IRQn,3);
NVIC_EnableIRQ(DMA2_Stream3_IRQn);

 TIM3->CCR2=TIM3->ARR+1;
  UNUSED(AudioFreq);
  UNUSED(Volume);
  UNUSED(options);
  return (USBD_OK);
  /* USER CODE END 0 */
}


/**
  * @brief  De-Initializes the AUDIO media low layer
  * @param  options: Reserved for future use
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_DeInit_FS(uint32_t options)
{
  /* USER CODE BEGIN 1 */

  UNUSED(options);
  return (USBD_OK);
  /* USER CODE END 1 */
}

/**
  * @brief  Handles AUDIO command.
  * @param  pbuf: Pointer to buffer of data to be sent
  * @param  size: Number of data to be sent (in bytes)
  * @param  cmd: Command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_AudioCmd_FS(uint8_t* pbuf, uint32_t size, uint8_t cmd)
{
  /* USER CODE BEGIN 2 */
  switch(cmd)
  {
    case AUDIO_CMD_START:
    break;

    case AUDIO_CMD_PLAY:
    break;
  }
  UNUSED(pbuf);
  UNUSED(size);
  UNUSED(cmd);
  return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  Controls AUDIO Volume.
  * @param  vol: volume level (0..100)
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_VolumeCtl_FS(uint8_t vol)
{
  /* USER CODE BEGIN 3 */
  UNUSED(vol);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  Controls AUDIO Mute.
  * @param  cmd: command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_MuteCtl_FS(uint8_t cmd)
{
  /* USER CODE BEGIN 4 */
  UNUSED(cmd);
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  AUDIO_PeriodicT_FS
  * @param  cmd: Command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */

int yy[4];
//volatile float  xxx[24]={0,0,0,0,
//  0,0,0,0,0,0,0,0,0,0,00,0,0,0,0};
//volatile int  xxx[24]={0,0,0,0,
//0,0,0,0,0,0,0,0,0,0,00,0,0,0,0};

float xxx[30]={0,0,0,0,
  0,0,0,0,
  0,0,0,0,
  1.58561962155 ,0.20206855018 , 1.98716270916 ,-1.98716270916,
  -1.79793144982, 0.40154308761,1.4e-3,0.980,0.015};

float xxx2[40]={0,0,0,0,0,
  0,0,0,0,0,
  0,0,0,0,
  1.4e-3, 0.98, 0.015, //x[14]=1.2e-3,x[15]=0.98,
 2.3446273e+00,  3.4857350e-01 , 7.3589825e-02,2.976912e+00, -2.976912e+00,//x[17]
 -3.7397556e+00,  3.3042870e-01,  6.9759148e-02,-3.1403835e+00,3.1403835e+00,//x[22]
 -4.33083217954e-01, -8.49306487776e-01,  3.01856615214e-01};//x[27]



/* volatile float  xxx[30]={0,0,0,0, */
/*   0,0,0,0,0, */
/*   0,0,0,0,0, */
/*   2.34462735360e+00, 3.48573500619e-01 ,7.35898259147e-02, */
/*   2.97691267066e+00, -2.97691267066e+00, */
/*   -3.73975564129e+00, 3.30428701846e-01,6.97591486528e-02, */
/*   -3.14038358306e+00 ,3.14038358306e+00, */
/*   -4.33083217954e-01,-8.49306487776e-01,3.01856615214e-01}; */

/*      /\*     [[ 1.58561962155  0.20206855018  1.98716270916 -1.98716270916] */
/*  [-1.79793144982  0.40154308761 -1.             1.           ]] */

uint32_t datos[8]={0,0,0,0,0,0,0,0};
static int8_t AUDIO_PeriodicTC_FS(uint8_t *pbuf, uint32_t size, uint8_t cmd)
{
  /* USER CODE BEGIN 5 */
  int16_t *audio=(int16_t*)pbuf;
  //  int period0=period;
  iii=iii%(nsa*OSR*NDA);//Subwoofer
   datos[0]=(uint32_t)xxx;
  datos[1]=(uint32_t)audio;
  datos[2]=size>>1;
  datos[3]=(uint32_t)data;
  datos[4]=iii*2;
  datos[5]=(uint32_t)&iii;
  do_sf3(datos);


  /* datos[0]=(uint32_t)xxx2; */
  /* datos[1]=(uint32_t)audio; */
  /* datos[2]=size>>1; */
  /* datos[3]=(uint32_t)data; */
  /* datos[4]=iii*2; */
  /* datos[5]=(uint32_t)&iii; */
  /* do_sf4(datos); */

  
  UNUSED(pbuf);
  UNUSED(size);
  UNUSED(cmd);
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Gets AUDIO State.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_GetState_FS(void)
{
  /* USER CODE BEGIN 6 */
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  Manages the DMA full transfer complete event.
  * @retval None
  */
void TransferComplete_CallBack_FS(void)
{
  /* USER CODE BEGIN 7 */
  USBD_AUDIO_Sync(&hUsbDeviceFS, AUDIO_OFFSET_FULL);
  /* USER CODE END 7 */
}

/**
  * @brief  Manages the DMA Half transfer complete event.
  * @retval None
  */
void HalfTransfer_CallBack_FS(void)
{
  /* USER CODE BEGIN 8 */
  USBD_AUDIO_Sync(&hUsbDeviceFS, AUDIO_OFFSET_HALF);
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
volatile uint32_t ii1=0;
extern  void DMA2_Stream3_IRQHandler(void) {
  if (DMA2->LISR & (1<<27)){
    DMA2->LIFCR |= (1<<27);

    //    TIM1->ARR=174;
    if(iii<3500)
      TIM1->ARR=period3;
    if(iii>(nsa*OSR*NDA-3500))
      TIM1->ARR=period;
     TIM3->CCR1=iii>>5;

      //TIM3->CCR1=ii1;
  }

}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

