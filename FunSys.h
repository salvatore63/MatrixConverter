/**
  ******************************************************************************
  * @file    EXTI/stm32f10x_it.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    09/13/2010
  * @brief   This file contains the headers of the interrupt handlers.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_IT_H
#define __STM32F10x_IT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

//#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define SetDDram(val)	(GPIOB->BRR)=(GPIO_Pin_14) , (SPI1->DR) =(0x80|(val)),(Pdisp=TRUE)
#define SetCGram(val)	((GPIOB->BRR)=(GPIO_Pin_14)) , ((SPI1->DR) =(0x40|(val))),(Pdisp=TRUE)
#define DisplayOn()     ((GPIOB->BRR)=(GPIO_Pin_14)) , ((SPI1->DR) =(0x0C)),(Pdisp=TRUE)
#define ClearDisplay()  ((GPIOB->BRR)=(GPIO_Pin_14)) , ((SPI1->DR) =(0x01)),(Pdisp=TRUE)
#define SetDisplay()    ((GPIOB->BRR)=(GPIO_Pin_14)) , ((SPI1->DR) =(0x38)),(Pdisp=TRUE)
#define ShowCursor()    ((GPIOB->BRR)=(GPIO_Pin_14)) , ((SPI1->DR) =(0x0F)),(Pdisp=TRUE)
#define HideCursor()    ((GPIOB->BRR)=(GPIO_Pin_14)) , ((SPI1->DR) =(0x0C)),(Pdisp=TRUE)
#define SetValDisp(val)	((GPIOB->BSRR)=(GPIO_Pin_14)) , ((SPI1->DR) =(val)),(Pdisp=TRUE)
#define LoadLine(n)     for(i=n*16;i<(n+1)*16;i++) { ; }

/* Exported functions ------------------------------------------------------- */


void visualizza( void);
void Display (int z,uint8_t pos);
void seleziona_messaggio(uint32_t sect,uint8_t display);

#endif /* __STM32F10x_IT_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
