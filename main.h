/**
  ******************************************************************************
  * @file    SysTick/main.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    09/13/2010
  * @brief   Header for main.c module
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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "stm32f10x.h"
#include "STM32vldiscovery.h"
#include "rtc.h"
#include "diskio.h"



/* Exported types ------------------------------------------------------------*/

static __IO uint32_t registroC,registroB;

#define AND_C                   ((uint32_t)0xFFFFE000)  /* mask per registro C */
#define AND_B                   ((uint32_t)0xFFFFC3FF)  /* mask per registro B */

#define SELECT()        STM32F100PinSet( GPIOB , GPIO_Pin_0 , FALSE)    /* MMC CS = L */
#define DESELECT()      STM32F100PinSet( GPIOB , GPIO_Pin_0 , TRUE)      /* MMC CS = H */

#define  CH1_PC0  0    //  CH10 PC0 ->  ch1
#define  CH2_PC1  1    //  CH11 PC1 ->  ch2
#define  CH3_PC2  2    //  CH12_ PC2 -> ch3
#define  CH4_PC3  3    //  CH13_PC3 ->  ch4
#define  CH5_PC4  4    //  CH14_PC4 ->   ch5
#define  CH6_PC5  5    //  CH15_PC5 ->   ch6
#define  CH7_PA0  6    //  CH0_PA0 ->   ch7
#define  CH8_PA1  7    //  CH1_PA1 ->   ch8
#define  CH9_PA2  8    //  CH2_PA2 ->   ch9
#define  CH10_PA3 9    //  CH3_PA3 ->  ch10
#define  CHT      10   //  CH16 temperatur sensor -> ch11

static uint16_t  n_medie,n_medie_cur;
static uint32_t cum_medie,T_media;
static float_t TReal,tempR; 
static uint8_t Tint,Tdec;

static uint32_t P[78],Pcalc,DnumFlash;  // Parametri da inizializzare da flash
static uint8_t numFlash,writePar,Ptemp;

static __IO uint32_t TimingDelay;
static  uint16_t tim1,tim2,tim3;
static  uint16_t val_tim1,val_tim2,val_tim3;

static __IO uint8_t bitcount;

//static __IO uint16_t ccyc,ccyc_min,ccyc_max; // conteggio numero cicli in un millisecondo

//static uint8_t ni;

static  uint8_t command;
static   bool  impulsi,ENanalog,avanti,req_ram_write,newSect2write;
static   bool TimeDate;
static __IO uint16_t AutRelTIM16,DutCycTIM16,DTimeTIM16;
static __IO uint16_t AutRelTIM17,DutCycTIM17,DTimeTIM17;
static __IO uint16_t AutRelTIM1,DutCycTIM1,DTimeTIM1;
static uint8_t posFasi,posFasiPrec,contSync,fasesyn,possyn,testpos,roti,rotn;
static __IO uint32_t   prima,seconda,terza,quarta,quinta;
static __IO uint32_t   primaD,secondaD,terzaD,quartaD,quintaD;

static uint8_t   ram_data[512],*MaxRamD,*minRamD,*d;
static uint32_t curSectDati,curSectTesti,curRigaWrite;;

static RTC_t  *DataOra=0,V_time;
const static RTC_t *Data0;
static uint16_t yearP=0;
static uint8_t  monthP=0;
static uint8_t  mdayP=0;
static uint8_t  hourP=0;
static uint8_t  minP=0;
static uint8_t  secP=0;
static uint8_t  y4,y3,y2,y1,m2,m1,g2,g1,h2,h1,p2,p1,s2,s1,v4,v3,v2,v1;
const static uint8_t *d_write;


typedef enum
{ 
  SetDDram ,
  SetCGram, 
  DisplayOn,
  ClearDisplay,
  SetDisplay,
  ShowCursor,
  HideCursor,
  SetValDisp
  
} ActFun;

typedef enum
{ 
  ch1,  // CH1_PC0  3.3V alimentazione
  ch2,  // CH2_PC1
  ch3,  // CH3_PC2 
  ch4,  // CH4_PC3  
  ch5,  // CH5_PC4  
  ch6,  // CH6_PC5  
  ch7,  // CH7_PA0  
  ch8,  // CH8_PA1  
  ch9,  // CH9_PA2  
  ch10,  // CH10_PA3  
  ch11  // CHT  ntc temperatura
  
} MyChannel;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void FuncDisp(ActFun fun , uint8_t val);
void visualizza( void);
void Display ( uint32_t z, uint8_t pos);
void seleziona_messaggio(uint8_t display);
void sel_mess_da_flash(uint8_t display);
void TimingDelay_Decrement(void);
void TimingAuxOp(void);
void Leggi_Tastiera(void);
void write_data(MyChannel ch);
uint32_t tastierino(void);

//void disk_timerproc(void);

inline void Epuls(void)
{
  uint8_t k;
  STM32F100PinSet( GPIOD , GPIO_Pin_2 , TRUE);
  for (k=0;k<50;k++){STM32F100PinSet( GPIOD , GPIO_Pin_2 , TRUE);}
  STM32F100PinSet( GPIOD , GPIO_Pin_2 , FALSE);
}

inline void cifre(uint32_t z)
{
  float temp;   
  if ( z > 65535) z = 65535;
     if ( z < 1 ) z = 0;              
     quinta = lrintf(z/10000);
     quarta = lrintf((z-quinta*10000)/1000);
     temp=z/100;
     temp=temp-quinta*100;
     temp=temp-quarta*10;
     terza = lrintf(temp);
     temp=z/10;
     temp=temp-quinta*1000;
     temp=temp-quarta*100;
     temp=temp-terza*10;
     seconda = lrintf(temp);
     temp=z;
     temp=temp-quinta*10000;
     temp=temp-quarta*1000;
     temp=temp-terza*100;
     prima=lrintf(temp-seconda*10);
}

inline void cifreD(uint32_t z)
{
  float temp;   
  if ( z > 65535) z = 65535;
     if ( z < 1 ) z = 0;              
     quintaD = lrintf(z/10000);
     quartaD = lrintf((z-quintaD*10000)/1000);
     temp=z/100;
     temp=temp-quintaD*100;
     temp=temp-quartaD*10;
     terzaD = lrintf(temp);
     temp=z/10;
     temp=temp-quintaD*1000;
     temp=temp-quartaD*100;
     temp=temp-terzaD*10;
     secondaD = lrintf(temp);
     temp=z;
     temp=temp-quintaD*10000;
     temp=temp-quartaD*1000;
     temp=temp-terzaD*100;
     primaD=lrintf(temp-secondaD*10);
}

inline void inc_d(void)
{
  d++;
  if ( d == MaxRamD )
    {  
      disk_write(0,ram_data,curSectDati,1);
      curSectDati++;
      disk_read(0,ram_data,curSectDati,1);    // legge un settore sul drive 0 a partire da sect
      d=minRamD;
    }
  
}
#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
