/**
  ******************************************************************************
  * @file    stm32vldiscovery.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    09/13/2010
  * @brief   STM32VLDISCOVERY abstraction layer. 
  *          This file should be added to the main application to use the provided
  *          functions that manage the Leds LD3 and LD4 and the USER push-button.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 
  
/* Includes ------------------------------------------------------------------*/
#include "STM32vldiscovery.h"

/** @defgroup STM32vldiscovery_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM32vldiscovery_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM32vldiscovery_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM32vldiscovery_Private_Variables
  * @{
  */ 

//const uint16_t BUTTON_PIN_SOURCE[BUTTONn] = {USER_BUTTON_EXTI_PIN_SOURCE};

//const uint16_t BUTTON_PORT_SOURCE[BUTTONn] = {USER_BUTTON_EXTI_PORT_SOURCE};

//GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT}; 

//const uint16_t BUTTON_PIN[BUTTONn] = {USER_BUTTON_PIN}; 

//const uint32_t BUTTON_CLK[BUTTONn] = {USER_BUTTON_GPIO_CLK};

//const uint16_t BUTTON_EXTI_LINE[BUTTONn] = {USER_BUTTON_EXTI_LINE};

//const uint16_t BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn};					 

/** @defgroup STM32vldiscovery_Private_FunctionPrototypes
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM32vldiscovery_Private_Functions
  * @{
  */ 

/*
void STM32vldiscovery_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // Enable the BUTTON Clock 
  RCC_APB2PeriphClockCmd(BUTTON_CLK[Button] | RCC_APB2Periph_AFIO, ENABLE);

  // Configure Button pin as input floating 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = BUTTON_PIN[Button];
  GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStructure);

  if (Button_Mode == BUTTON_MODE_EXTI)
  {
    // Connect Button EXTI Line to Button GPIO Pin 
    GPIO_EXTILineConfig(BUTTON_PORT_SOURCE[Button], BUTTON_PIN_SOURCE[Button]);

    // Configure Button EXTI line 
    EXTI_InitStructure.EXTI_Line = BUTTON_EXTI_LINE[Button];
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;

    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  

    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Enable and set Button EXTI Interrupt to the lowest priority 
    NVIC_InitStructure.NVIC_IRQChannel = BUTTON_IRQn[Button];
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure); 
  }
}


 uint32_t STM32vldiscovery_PBGetState(Button_TypeDef Button)
{
  return GPIO_ReadInputDataBit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
} */



// **** Aldo Salvatore Coraggio  03/12/2011*********
void GPIO_PinInit( GPIO_TypeDef* port , uint16_t pin  , GPIOMode_TypeDef mod , GPIOSpeed_TypeDef speed )
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  if ( port == GPIOA ) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  if ( port == GPIOB ) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  if ( port == GPIOC ) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  if ( port == GPIOD ) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
 
  GPIO_InitStructure.GPIO_Pin = pin;
  GPIO_InitStructure.GPIO_Mode = mod;
  GPIO_InitStructure.GPIO_Speed = speed;
  GPIO_Init( port , &GPIO_InitStructure );
  
 
}

void STM32F100PinSet( GPIO_TypeDef* port , uint16_t pin , bool On)
{
  if ( On )  port->BSRR = pin;
   else      port->BRR = pin;
}

void STM32F100PinToggle( GPIO_TypeDef* port , uint16_t pin )
{
  port->ODR ^= pin;
}

void EXTI_PinInit(uint8_t port,uint16_t pin,uint32_t pinMask , EXTIMode_TypeDef EXTI_Mode , EXTITrigger_TypeDef EXTI_Trigger , FunctionalState EXTI_LineCmd)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_TypeDef*  gport;
  
  if ( port == GPIO_PortSourceGPIOA ) {
           RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
           gport = GPIOA;}
  if ( port == GPIO_PortSourceGPIOB ) {
           RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
           gport=GPIOB;}
  if ( port == GPIO_PortSourceGPIOC ) {
           RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
           gport=GPIOC;}
    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = pin;
  GPIO_Init(gport, &GPIO_InitStructure);

  
  //GPIO_EXTILineConfig(port, pin);

  AFIO->EXTICR[0]=0x1000;  // seleziona i pin da cui arriva l'interrupt pin 3 porta B
  AFIO->EXTICR[1]=0x0001;  // seleziona i pin da cui arriva l'interrupt pin 4 porta B
  /* Configure Button EXTI line */
  EXTI_InitStructure.EXTI_Line = pinMask;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode;

  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger;  

  EXTI_InitStructure.EXTI_LineCmd = EXTI_LineCmd;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  //NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  if (pinMask == EXTI_Line0 ) NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  if (pinMask == EXTI_Line1 ) NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  if (pinMask == EXTI_Line2 ) NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  if (pinMask == EXTI_Line3 ) NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
  if (pinMask == EXTI_Line4 ) NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  if (pinMask == EXTI_Line15 ) NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure); 

}


// **************************************************


/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter can be one of following parameters:   
  *     @arg BUTTON_USER: USER Push Button 
  * @param  Button_Mode: Specifies Button mode.
  *   This parameter can be one of following parameters:   
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO 
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                     generation capability  
  * @retval None
  */

/**
  * @}
  */ 

/**
  * @}
  */ 


/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */
    
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
