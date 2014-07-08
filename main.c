/**
  ******************************************************************************
  * @file    EXTI/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    09/13/2010
  * @brief   Main program body
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
//#include <math.h>
#include <stdlib.h>
#include <fenv.h>
#include "stm32f10x.h"
#include "STM32vldiscovery.h"
#include "main.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_flash.h"
#include "ff.h"
#include "diskio.h"
//#include "rtc.h"

//#include "stm32f10x_spi.h"

extern  uint32_t  gl_ptr_mem;         /* memory data pointer     */

// variabili per gestione flash
volatile uint32_t tcalc,tcalc1;
static uint32_t sectFirstFile,sectSecondFile,firstSectFirstFile,sizeFirstFile,sizeSecondFile;
static uint32_t m,k,s,sectP,mreg,                          // numero di settore flash caricato in ram
              SectVarFlash;                 //  numero del settore contenente l'MSB ascii della variabile
                                            //  contenuta  della riga selezionata

//static uint32_t curSectDati,curSectTesti,curRigaWrite;

uint16_t byteSect,byteSectP,curPosSectT;
uint8_t   filler,SD_mount,test1;

size_t sizeRam=128;

static __IO uint8_t sel,n_byte;
static __IO uint16_t peso_tst;
static  uint32_t valore,data_val,data_valP,cifra,cifraP,cifraN;
static  int32_t temp_tst;
static  bool start_T1,start_T2,start_T3;

static __IO bool data_recived;
static __IO unsigned char scandata,n_byte,output;
static __IO uint8_t sel;

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;  //  Flash

#define FLASH_PAGE_SIZE    ((uint16_t)0x400)               // 1K

#define AREA_TEXT_START_ADDR  ((uint32_t)0x08010000)
#define AREA_TEXT_END_ADDR  ((uint32_t)0x08012800)

#define AREA_PAR_START_ADDR  ((uint32_t)0x0801F002)
#define AREA_PAR_END_ADDR    ((uint32_t)0x0801FF02)     // 3 pagine da 1K  */

//__no_init uint32_t arr[4096] @ AREA_PAR_START_ADDR ;

uint32_t FlashCounter = 0x00, Address = 0x00;
uint32_t Data = 0x3210ABCD;
__IO uint32_t NbrOfPage = 0x00;
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
volatile TestStatus MemoryProgramStatus = PASSED;


// Comment or incomment this define to use or not the DMA 
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define BufferLenght       11                        // numero di canali analogici
MyChannel channel;



ADC_InitTypeDef   ADC_InitStructure;
DMA_InitTypeDef   DMA_InitStructure;
static uint16_t ADC1ConvertedValue[BufferLenght];
ErrorStatus HSEStartUpStatus;
    
/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);

// variabili per gestione Display

static  uint8_t   disp_addr_char,disp_data,display,displayP,posiz;
static __IO uint8_t   idisp,i_p,CursorOff,dataDisp,sequenza,sequenzaD,nEdisp;
//static __IO uint32_t   prima,seconda,terza,quarta,quinta;
//static  uint8_t   *n,*MaxRam,*minRam,riga[34],*pRiga; 
static uint8_t   *ram,*n,*MaxRam,*minRam,riga[34],*pRiga,*pBoot,*pVar,*FATSz32;
//static uint8_t   ram_data[512],*MaxRamD,*minRamD,*d;
static __IO uint8_t   *AddrVarRam; //  contiene l'indirizzo dell'MSB ascii della variabile 
                                   // contenuta  della riga selezionata

static   bool      Disp_SPI_BSY,caricaDisp,search,found;


static uint16_t Tsesti;  // ( 1/Tsesti ) / 6   =  frequenza del campo rotante

static uint32_t Freq,FreqAct,FreqSet,FreqStart,FreqPrec,LastFreq,
                FreqMax,         // Frequenza massima in decimi di Hertz
	        FreqMin;         // Frequenza minima in decimi di Hertz
			         // con StartArr = 50 l'rrotondamento prende metà rampa 
static int32_t Tramp_acc,     // tempo di accelerazione in decimi di secondo range ( 0 - 9999 )
               Tramp_dec;

 static __IO uint16_t ccyc,ccyc_min,ccyc_max,ccyc_P; // conteggio numero cicli in un millisecondo
 static uint8_t ni,nm,nk;
 

/* Private macro -------------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  
 if (SysTick_Config(SystemCoreClock / 1000))
 { 
   
   
   /* Capture error */ 
    //while (1);
 }
 

 TimingDelay=1000;
 while (TimingDelay != 0);    // aspetta 2sec dopo il reset

//  confifurazione   ingressi - uscite
// su CR : 0 Analog mode; 3 -> output push pull;  4 -> floating input ; 7 -> output open drain
//         8 -> input PU/PD ( PU ODR -> 1 ); B -> Output push-pull alternate func   
  //GPIO_AFIODeInit();
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB| RCC_APB2Periph_GPIOC| RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE); 
  
  //GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable | GPIO_Remap_SWJ_NoJTRST, ENABLE);  
 GPIOA->CRL =    0xB4B40000;
 GPIOA->CRH =    0x4444444B;
 GPIOA->ODR =    0x00000000; 
 GPIOB->CRL =    0xBB488443;
 GPIOB->CRH =    0x83B333BB;
 GPIOC->CRL =    0x77000000;
 GPIOC->CRH =    0x44433333;
 GPIOD->CRL =    0x44444344;
 AFIO->MAPR =    0x02000000;   // disabilita JTAG
  
 EXTI_PinInit(GPIO_PortSourceGPIOB,GPIO_Pin_3,EXTI_Line3,EXTI_Mode_Interrupt,EXTI_Trigger_Falling,ENABLE);
 // configurazione interrupt su PB3  per lettura tastiera
 EXTI_PinInit(GPIO_PortSourceGPIOB,GPIO_Pin_4,EXTI_Line4,EXTI_Mode_Interrupt,EXTI_Trigger_Falling,ENABLE);
 // configurazione interrupt su PB3  per lettura tastiera
 
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
 SPI1->CR1=0x4377; // transmit only
 
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
 TIM1->CCMR1=0x006C;  // pwm1   OC1FastEnable + ARPE bit
 TIM1->CCER=0x0005;   // attiva OC1 ed OC1N
 TIM1->BDTR=0x8C3F;      
 TIM1->EGR=0x01;        //  initialize whit UG bit 
 TIM1->CR1=0x0001;
 
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16,ENABLE);
 TIM16->CCMR1=0x006C;  // pwm1   OC1FastEnable + ARPE bit
 TIM16->CCER=0x0005;   // attiva OC1 ed OC1N
 TIM16->BDTR=0x8C3F;      
 TIM16->EGR=0x01;        //  initialize whit UG bit 
 TIM16->CR1=0x0001;

 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17,ENABLE);
 TIM17->CCMR1=0x006C;  // pwm1   OC1FastEnable + ARPE bit
 TIM17->CCER=0x0005;   // attiva OC1 ed OC1N
 TIM17->BDTR=0x8C3F;      
 TIM17->EGR=0x01;        //  initialize whit UG bit 
 TIM17->CR1=0x0001;

 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
 NVIC_EnableIRQ(TIM7_IRQn);
 TIM7->PSC=64;  // prescaler
 TIM7->ARR=1000;   // autoreload
 TIM7->CR2=0x20;
 TIM7->DIER=0x1;
 TIM7->EGR=0x1;
 TIM7->CR1=0x85;


 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  /* DMA1 channel1 configuration ---------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC1ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = BufferLenght;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);

 TimingDelay=10;
 while (TimingDelay != 0); 
 
/* 
 GPIO_PinLockConfig(GPIOA,0xffff);
 TimingDelay=10;
 while (TimingDelay != 0);   
  
 GPIO_PinLockConfig(GPIOB,0xffff);
 TimingDelay=10;
 while (TimingDelay != 0);   
 
 GPIO_PinLockConfig(GPIOC,0xffff);
 TimingDelay=10;
 while (TimingDelay != 0);  
 
 GPIO_PinLockConfig(GPIOD,0x0005);
*/
 
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);


  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = BufferLenght;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel11, channel3, channel16 and channel17 configurations */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);         //  CH1_PC0 -> ch1  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_239Cycles5);         //  CH2_PC1 -> ch2 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_239Cycles5);         //  CH3_PC2 -> ch3
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_239Cycles5);         //  CH4_PC3 -> ch4
  ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 5, ADC_SampleTime_239Cycles5);         //  CH5_PC4 -> ch5
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 6, ADC_SampleTime_239Cycles5);         //  CH6_PC5 -> ch6  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 7, ADC_SampleTime_239Cycles5);          //  CH7_PA0 -> ch7
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 8, ADC_SampleTime_239Cycles5);          //  CH8_PA1 -> ch8
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 9, ADC_SampleTime_239Cycles5);          //  CH9_PA2 -> ch9
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 10, ADC_SampleTime_41Cycles5);          //  CH10_PA3 -> ch10
  ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 11, ADC_SampleTime_239Cycles5);        //  CH10_PA3 -> ch11 sensore temperatura
  
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
  /* Enable TempSensor and Vrefint channels: channel16 and Channel17 */
  ADC_TempSensorVrefintCmd(ENABLE);

  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);

  
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  
  /* Test on Channel 1 DMA1_FLAG_TC flag */
  while(!DMA_GetFlagStatus(DMA1_FLAG_TC1));
  
  /* Clear Channel 1 DMA1_FLAG_TC flag */
  DMA_ClearFlag(DMA1_FLAG_TC1);	
  
 
//***************  inizializzazioni  ****************
 peso_tst=1;
 scandata=0;
 bitcount=11;
 data_recived=0;
 start_T1=FALSE;
 start_T3=FALSE;
 val_tim1=val_tim2=val_tim3=0;
 DESELECT(); 
 search=0;
 found=0;
 ccyc=0;
 ccyc_max=0;
 ccyc_min=0;  
 ni=0;nm=0;
 n_medie=100;
 n_medie_cur=0;
 cum_medie=0;

 ram = malloc (sizeRam);
// ram_data = malloc (sizeRam);
 
 // ram2write = malloc (sizeRam);
 //riga = malloc (9);
 FATSz32 = malloc (1);
 
 //riga[34]=0;
 n=ram;
 minRam=ram;
 MaxRam=ram;
 for(k=0;k<512;k++) {*n=48;n++;MaxRam++;}

 pRiga=&riga[0];
 for(k=0;k<34;k++)  {riga[k]=48;}  //{*pRiga=48;pRiga++;}
 
 d=ram_data;
 minRamD=ram_data;
 MaxRamD=ram_data;
 for(s=0;s<512;s++) {*d=48;inc_d();MaxRamD++;}
 d_write=&ram_data[0];
 
 //pRiga=riga;
 n=ram;
 d=ram_data;
                 
 m=0;           
 s=0;
 sectP=0;       
 byteSect = byteSectP = 0;  // posizione byte nel settore 
 curPosSectT=0; 
 disp_addr_char=0;
 disp_data=0;
 displayP=1;
 tim2=350;  // timer per display ciclico dati 350msec
 tim3=1000; // timer per visualizzazione orologio
 Disp_SPI_BSY=caricaDisp=FALSE;
 nEdisp=0;
 posiz=11;
 CursorOff=1;
 dataDisp=10;
 sequenza=0;
 sequenzaD=0;
 i_p=35;
 idisp=0;
 TimeDate=FALSE;
 
 ADC1ConvertedValue[0]=0;
 ADC1ConvertedValue[1]=0;
 ADC1ConvertedValue[2]=0;
 ADC1ConvertedValue[3]=0;
 ADC1ConvertedValue[4]=0;
 ADC1ConvertedValue[5]=0;
 ADC1ConvertedValue[6]=0;
 ADC1ConvertedValue[7]=0;
 ADC1ConvertedValue[8]=0;
 ADC1ConvertedValue[9]=0;
 ADC1ConvertedValue[10]=0;
 channel=ch1;
 data_val=0;
 data_valP=0;
 valore=0;
//     STM32F100PinToggle( GPIOC,GPIO_Pin_8); 

 AutRelTIM16=10000;
 TIM16->ARR =  AutRelTIM16;
 DutCycTIM16=8000;
 TIM16->CCR1 =  DutCycTIM16;
 DTimeTIM16=15000;
 
 AutRelTIM17=10000;
 TIM17->ARR =  AutRelTIM17;
 DutCycTIM17=8000;
 TIM17->CCR1 =  DutCycTIM17;
 DTimeTIM17=15000;

 AutRelTIM1=10000;
 TIM1->ARR =  AutRelTIM1;
 DutCycTIM1=8000;
 TIM1->CCR1 =  DutCycTIM1;
 DTimeTIM1=15000;

 FuncDisp(SetDisplay , 0);
 TimingDelay=2;
 while (TimingDelay != 0);    
 //while( (SPI1->SR & SPI_SR_BSY) != 0 );
 Epuls();
 
 FuncDisp(ClearDisplay , 0);
 TimingDelay=2;
 while (TimingDelay != 0);    
 //while( (SPI1->SR & SPI_SR_BSY) != 0 );
 Epuls();
 
  
 FuncDisp(DisplayOn , 0);
 TimingDelay=2;
 while (TimingDelay != 0);    
 //while( (SPI1->SR & SPI_SR_BSY) != 0 );
 Epuls();
 
 Disp_SPI_BSY=FALSE;
      
 SD_mount = disk_initialize(0);             // inizializza SD
 if ( !disk_read(0,ram,m,1) ) {     // legge settore di boot
    pBoot=ram;                            // e calcola settore dati 
    pBoot+=36;
    tcalc = *pBoot;
    pBoot++;
    tcalc+=*pBoot*256;
    pBoot++;
    tcalc+=*pBoot*65356;
    pBoot++;
    tcalc+=*pBoot*16777216;
    tcalc=tcalc*2;
    firstSectFirstFile=32+tcalc;
    sectFirstFile=firstSectFirstFile+4;//tcalc+4;
    if ( !disk_read(0,ram,firstSectFirstFile,1) ) {   // calcola settore testi
         pBoot=ram;                            
         pBoot+=60;         // vedi Procedura di individuazione dati con formattazione FAT32 di  schede SD  su STM32
         tcalc = *pBoot;
         pBoot++;
         tcalc+=*pBoot*256;
         pBoot++;
         tcalc+=*pBoot*65356;
         pBoot++;
         sizeFirstFile=tcalc + ( *pBoot*16777216 );
         tcalc = 1 + (sizeFirstFile/2048);
         tcalc = (uint32_t)trunc(tcalc);
         tcalc *=4;
         sectSecondFile = 4 + tcalc + firstSectFirstFile;
         m=sectFirstFile;    //m=sectSecondFile;
         disk_read(0,ram,m,1);
    } 
  }

  curSectDati=sectSecondFile;//sectFirstFile;
  curSectTesti=sectFirstFile;//sectSecondFile;
  curRigaWrite=0; 
/*  
  V_time.year = (uint16_t)2013;  // impostazione data ora
  V_time.month = (uint8_t)6;
  V_time.mday = (uint8_t)14;
  V_time.hour = (uint8_t)16; 
  V_time.min = (uint8_t)31; 
  V_time.sec =  (uint8_t)0;
 
  Data0 = &V_time;
  DataOra = &V_time;
  cifra=0;
  cifraP=0;
  cifraN=0;
  rtc_init();
*/  
  req_ram_write=0;
  newSect2write=1;

  if ( SD_mount ==  0 ) {  // se scheda SD inizializzata correttamente
    seleziona_messaggio(0);
    FLASH_UnlockBank1();
    
    if ( pRiga[33] == 49 ) { // se è abilitata la memorizzazione dei testi
      
      NbrOfPage = (AREA_TEXT_END_ADDR - AREA_TEXT_START_ADDR) / FLASH_PAGE_SIZE;
      FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	 // Clear All pending flags
      
      for(FlashCounter = 0; (FlashCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); FlashCounter++)
       {
          FLASHStatus = FLASH_ErasePage(AREA_TEXT_START_ADDR + (FLASH_PAGE_SIZE * FlashCounter));  // Erase the FLASH pages 
       }

      Address =  AREA_TEXT_START_ADDR;
      display = 4;
      for( k=1; k <= 90; k++)
       {
         seleziona_messaggio(display);
         for ( nk=0;nk<=12;nk+=4 ) 
          {
            Data=pRiga[nk];Data <<= 8;
            Data |= pRiga[nk+1];Data <<= 8;
            Data |= pRiga[nk+2];Data <<= 8;
            Data |= pRiga[nk+3];
            FLASHStatus = FLASH_BUSY;           
            while( FLASH_ProgramWord(Address, Data) != FLASH_COMPLETE ) ;
            Address  += 4;
            
          }

         for ( nk=18;nk<=30;nk+=4 ) 
          {
            Data=pRiga[nk];Data <<= 8;
            Data |= pRiga[nk+1];Data <<= 8;
            Data |= pRiga[nk+2];Data <<= 8;
            Data |= pRiga[nk+3];
            FLASHStatus = FLASH_BUSY;           
            while( FLASH_ProgramWord(Address, Data) != FLASH_COMPLETE ) ;
            Address += 4;
          }
         
         display += 2;
       }
           
    }
    if ( pRiga[31] == 49 ) { // se è abilitata la memorizzazione dei parametri
      
      NbrOfPage = (AREA_PAR_END_ADDR - AREA_PAR_START_ADDR) / FLASH_PAGE_SIZE;
      FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	 // Clear All pending flags
      
      for(FlashCounter = 0; (FlashCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); FlashCounter++)
       {
          FLASHStatus = FLASH_ErasePage(AREA_PAR_START_ADDR + (FLASH_PAGE_SIZE * FlashCounter));  // Erase the FLASH pages 
       }
      display=234;                          // numero da verificare sul file F1_testi.txt
      Address =  AREA_PAR_START_ADDR;
      
      for (nk=0;nk<=79;nk++)  {
         seleziona_messaggio(display);
         Pcalc=riga[33]-48;
         if ( ( Ptemp = riga[32]-48 ) != 0)  Pcalc += Ptemp*10;
         if ( ( Ptemp = riga[31]-48 ) != 0)  Pcalc += Ptemp*100;
         if ( ( Ptemp = riga[30]-48 ) != 0)  Pcalc += Ptemp*1000;
         if ( ( Ptemp = riga[29]-48 ) != 0)  Pcalc += Ptemp*10000;
         if ( ( Ptemp = riga[28]-48 ) != 0)  Pcalc += Ptemp*100000;
         if ( ( Ptemp = riga[27]-48 )  != 0)  Pcalc += Ptemp*1000000;
         if ( ( Ptemp = riga[26]-48 )  != 0)  Pcalc += Ptemp*10000000;
         if ( ( Ptemp = riga[25]-48 )  != 0)  Pcalc += Ptemp*100000000;
         if ( ( Ptemp = riga[24]-48 )  != 0)  Pcalc += Ptemp*1000000000;
         FLASHStatus = FLASH_BUSY;
         while( FLASH_ProgramWord(Address, Pcalc) != FLASH_COMPLETE ) ;
         Address += 4;
         display += 2;
      
      }
    }  
       
    FLASH_LockBank1();
    seleziona_messaggio(2);  // riga contiene data ed ora
    if ( pRiga[33] == 49 ) { // impostare data ed ora
        Pcalc=0;
        if ( ( Ptemp = riga[6]-48 ) != 0)  Pcalc += Ptemp*10; 
        if ( ( Ptemp = riga[7]-48 ) != 0)  Pcalc += Ptemp;
        V_time.mday = (uint8_t)Pcalc;
        Pcalc=0;
        if ( ( Ptemp = riga[9]-48 ) != 0)  Pcalc += Ptemp*10; 
        if ( ( Ptemp = riga[10]-48 ) != 0)  Pcalc += Ptemp;
        V_time.month = (uint8_t)Pcalc;
        Pcalc=0;
        if ( ( Ptemp = riga[12]-48 ) != 0)  Pcalc += Ptemp*1000;
        if ( ( Ptemp = riga[13]-48 ) != 0)  Pcalc += Ptemp*100;
        if ( ( Ptemp = riga[14]-48 ) != 0)  Pcalc += Ptemp*10; 
        if ( ( Ptemp = riga[15]-48 ) != 0)  Pcalc += Ptemp;
        V_time.year = (uint16_t)Pcalc;
        Pcalc=0;
        if ( ( Ptemp = riga[24]-48 ) != 0)  Pcalc += Ptemp*10; 
        if ( ( Ptemp = riga[25]-48 ) != 0)  Pcalc += Ptemp;
        V_time.hour = (uint8_t)Pcalc;
        Pcalc=0;
        if ( ( Ptemp = riga[27]-48 ) != 0)  Pcalc += Ptemp*10; 
        if ( ( Ptemp = riga[28]-48 ) != 0)  Pcalc += Ptemp;
        V_time.min = (uint8_t)Pcalc;
        Pcalc=0;
        if ( ( Ptemp = riga[30]-48 ) != 0)  Pcalc += Ptemp*10; 
        if ( ( Ptemp = riga[31]-48 ) != 0)  Pcalc += Ptemp;
        V_time.sec = (uint8_t)Pcalc;
        
        Data0 = &V_time;
        DataOra = &V_time;
        cifra=0;
        cifraP=0;
        cifraN=0;
        rtc_init();
        
        rtc_settime(Data0);
              
    }
  }
  
  display=2;
  Address = AREA_PAR_START_ADDR; 
  for ( nk=0;nk<=79;nk++ )  {               // Inizializzazione parametri da flash STM
     P[nk]= (*(__IO uint32_t*) Address); 
     Address += 4;
  }
  numFlash=0;
  writePar=0;
  
   
  while (1)
  {  // *********************************   main loop  *************************
    
   ccyc++;                           // conteggio cicli macchina
       
   if(data_recived)
    {
     // idisp=0;
     data_recived=FALSE;
//     STM32F100PinToggle( GPIOC,GPIO_Pin_9 );   
     if ( n_byte != 15 )  
      {
	  //CursorOff = 1;
        cifra = tastierino();
        if (sel == 0 )
	  cifra = n_byte;  //  se al tasto non è stata assegnata nessuna
                           //  funzione visualizza  scancode  tastiera
      }

    }
   else
    { 
      if ( cifraN != 0)                     // programmazione di comandi seguenziali
      {
        data_recived= TRUE;
        if ( sel == 22 ) n_byte = 251;
      }
    }  
   
    if (byteSectP != byteSect	)  {

     n=ram;
     
     //idisp=0;      // fa partire la visualizzazione dei dati sul display

     byteSectP = byteSect;          

	 while(0 < byteSect-- ) n++;    // n punta al byte selezionato

     byteSect = byteSectP;   // per fare in modo che l'operazione venga 
                             // eseguita una sola volta
     cifra=*n;
    }

   
   if ( ( display != displayP ) || caricaDisp ) 
    {                          //  seleziona dati in ram caricati da flash eprom
      sel_mess_da_flash(display);  
      //seleziona_messaggio(display);      // seleziona messaggio da scheda SD
      caricaDisp=0;
      idisp=0;               // fa partire la visualizzazione dei 32 caratteri sul display
      displayP=display;
    }
   
   
   if ( (idisp < 35) || Disp_SPI_BSY ) {                        //-> visualizza dati selezionati in ram
/*     
      if ( idisp == 0 ) 
       {
         pRiga=ram; 
       }*/
      
      if ((idisp < 35) && (!Disp_SPI_BSY) )
       {
         visualizza();              
         
       }
      
      if ( Disp_SPI_BSY )  //-> generazione impulso 
       {                   //   caricamento dati display
         if ( (nEdisp <10 ) &&  ((SPI1->SR & SPI_SR_BSY) == 0) ) 
          { 
            STM32F100PinSet( GPIOD , GPIO_Pin_2 , TRUE);  //  set E_Disp
            nEdisp++;
          }

         if ( nEdisp == 10 ) 
          {
            STM32F100PinSet( GPIOD , GPIO_Pin_2 , FALSE);  //  res E_Disp
            nEdisp=0;
            Disp_SPI_BSY=FALSE;
          }
         
       }  
    
    }
   
   if ( (cifra != cifraP) && (!Disp_SPI_BSY)  && (idisp >= 35) )  {	 //-> visualizzazione dei dati digitati sul display
      sequenza++;
      Display(cifra,posiz);
      
    } 

   if ( req_ram_write )  {  // condizioni di scrittura dati su ram_data
    	                                   
      sequenzaD++;
      if ( newSect2write ) {
            d=ram_data;
            disk_read(0,d,curSectDati,1); 
            cifraP=-1;
            newSect2write=0;
      }      
      write_data(channel);
    }
   
    
   if ( dataDisp < 10  && !start_T2  ) {     //-> imposta visualizzazione dati ciclica
                                         
    start_T2=TRUE;
    switch (dataDisp)
    {
     case 1: { 
	       if ( sel == 5 )     
                { 
                 cifra=ADC1ConvertedValue[CHT];  // valore analogico sensore  (elemento 2)
                 cifraP =-1;                    // interno temperatura
                 posiz=2; 
                }    
               break;}
     case 2: { 
	       if ( sel == 5 )  {      
                 
                 cifra=ADC1ConvertedValue[CH10_PA3];  // valore analogico su PA3
                 cifraP =-1;
                 posiz=75; 
                }    
	       break;}
     case 3: { 
	       if ( sel == 5 )  {  
                 
                 cifra=ADC1ConvertedValue[CH1_PC0];  // valore analogico sensore
                 cifraP =-1;                    // interno Vcc
                 posiz=66; 
                }    
               break;}
     case 4: { 
	       if ( sel == 5 )  {    
                 
                 cifra=ADC1ConvertedValue[CH3_PC2];  // 
                 cifraP =-1;                    
                 posiz=11; 
	        }
               break;}
/*     case 8: {      
	       if ( sel == 100 )			  
               {
                if ( display == 21 ) {sequenza++;Display(Tramp_acc,74);}
                if ( display == 23 ) {sequenza++;Display(Tramp_dec,74);}
               }       
	       break;} */
     default: ;
    } 
    if (dataDisp ==4 ) dataDisp=0;
    dataDisp++;
    
   }

   
   if ( numFlash  != 0 )  {  // cancella la pagina flash  e riscrive i parametri

    switch (writePar)
    {
      case 0: {
               FLASH_UnlockBank1();
               FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
               FLASHStatus = FLASH_BUSY;
               writePar=1;
               Address = AREA_PAR_START_ADDR;
               break;}

      case 1: {
               if(FLASHStatus != FLASH_COMPLETE) FLASHStatus=FLASH_ErasePage(AREA_PAR_START_ADDR);
               if(FLASHStatus == FLASH_COMPLETE) {writePar=2;caricaDisp=1;}
               break;}

      case 2: {               
               if ( caricaDisp == 0 )
                {
                  Pcalc=riga[33]-48;
                  if ( ( Ptemp = riga[32]-48 ) != 0)  Pcalc += Ptemp*10;
                  if ( ( Ptemp = riga[31]-48 ) != 0)  Pcalc += Ptemp*100;
                  if ( ( Ptemp = riga[30]-48 ) != 0)  Pcalc += Ptemp*1000;
                  if ( ( Ptemp = riga[29]-48 ) != 0)  Pcalc += Ptemp*10000;
                  if ( ( Ptemp = riga[28]-48 ) != 0)  Pcalc += Ptemp*100000;
                  if ( ( Ptemp = riga[27]-48 )  != 0)  Pcalc += Ptemp*1000000;
                  if ( ( Ptemp = riga[26]-48 )  != 0)  Pcalc += Ptemp*10000000;
                  if ( ( Ptemp = riga[25]-48 )  != 0)  Pcalc += Ptemp*100000000;
                  if ( ( Ptemp = riga[24]-48 )  != 0)  Pcalc += Ptemp*1000000000;
                  FLASHStatus = FLASH_BUSY;
                  writePar=3;
                }
               break;}

      case 3: {
               if(FLASHStatus != FLASH_COMPLETE) FLASHStatus = FLASH_ProgramWord(Address, Pcalc);
               if(FLASHStatus == FLASH_COMPLETE) {
                 Address += 4;
                 numFlash--;
                 if (numFlash == 0 )
                   { FLASH_LockBank1();writePar=0;numFlash=0;caricaDisp=1;}
                 else
                 {
                   display += 2;
                   caricaDisp=1;
                   writePar=2;
                 }
               }  
               break;}
                              
      default: ;
    } 
   }    // end if numFlash

   if ( TimeDate ) {
   
      //rtc_gettime(DataOra);
      V_time = *DataOra;
      for(nk=0;nk<=5;nk++) riga[nk]=32;
      cifre((uint32_t)V_time.mday);
      riga[6]=seconda+48;riga[7]=prima+48;riga[8]='/';
      cifre((uint32_t)V_time.month);
      riga[9]=seconda+48;riga[10]=prima+48;riga[11]='/';
      cifre((uint32_t)V_time.year);
      riga[12]=quarta+48;riga[13]=terza+48;riga[14]=seconda+48;riga[15]=prima+48;
      for(nk=18;nk<=23;nk++) riga[nk]=32;
      cifre((uint32_t)V_time.hour);
      riga[24]=seconda+48;riga[25]=prima+48;riga[26]=':';
      cifre((uint32_t)V_time.min);
      riga[27]=seconda+48;riga[28]=prima+48;riga[29]=':';
      cifre((uint32_t)V_time.sec);
      riga[30]=seconda+48;riga[31]=prima+48; 
      riga[32]=32;riga[33]=32;
      TimeDate=FALSE;
      pRiga=riga;
      idisp=0;
                
   }
     
   
   
  } //*********************    end  main loop  *********************************
} 


//  ***************************************************************

uint32_t tastierino(void)
{


  switch (n_byte)
  { 

      
   case 181: {                   // tasto " / " del tastierino
	        sel=1;            
	        valore=AutRelTIM16;
		dataDisp=10;
                display=0;
                posiz=75;
		peso_tst=1;
		temp_tst=11;
	        return AutRelTIM16;
	       }

    case 131: {                   // tasto " * " del tastierino
		sel=2;            
	        valore=DutCycTIM16;
		dataDisp=10;
                display=2;
                posiz=75;
		peso_tst=1;
		temp_tst=11;
                return DutCycTIM16;
	      }

    case 212: {                  // tasto " F " della tastiera
	        sel=3;           // seleziona modifica frequenza
	        valore=Tsesti;      // pwm1   ARR TIM7;

		dataDisp=10;
                posiz=11;
                
		peso_tst=1;
		temp_tst=11;
                return valore;
	      }

    case 228: {                   // tasto " S " della tastiera
	        sel = 4;            // seleziona settore da leggere
	        valore = m;
		peso_tst=1;
		temp_tst=11;
                return valore;
	      }

    case 205: {                   // tasto " B " della tastiera
	        sel=5;            // seleziona posizione del flag debug
	        valore=ADC1ConvertedValue[CH10_PA3];
		dataDisp=2;
		peso_tst=1;
		temp_tst=11;
                return valore;
	      }


    case 211: {                   // tasto " T " della tastiera
	        sel=6;          
                valore=T_media;
                TReal=(float_t)T_media;
                TReal= -0.133*TReal+280.6;
                tempR = trunc(TReal);
                Tint = (uint8_t)tempR;
                Tdec = (uint8_t)trunc(10*(TReal-tempR));
                cifre(Tint);
                riga[30]=seconda+48;riga[31]=prima+48; 
                riga[32]='.'; riga[33]=Tdec+48;
                pRiga=riga;
                idisp=0;
		dataDisp=10;
                display=2;
                //posiz=11;
		peso_tst=1;
		temp_tst=11;
                return cifraP;  // per non visualizzare il valore di ritorno
	       }

    case 196: {                                       // tasto " J " 
	        sel=7;  // seleziona indirizzo DDram
	        valore = ADC1ConvertedValue[CH1_PC0];
 		dataDisp=10;
                posiz=11;
		peso_tst=1;
		temp_tst=11;
	        return valore;
	      }

    case 189: {                   // tasto " K " 
	        sel=0;  // visualizza gli scan code dei tasti premuti
                FuncDisp(ClearDisplay , 0);
                Disp_SPI_BSY = TRUE;
	        dataDisp=10;
		peso_tst=1;
		temp_tst=11;
	        return sel;
	      }
   
    case 221: {                   // tasto " X " 
	        sel=9;  
                valore=byteSect;
		peso_tst=1;
		temp_tst=11;
	        return valore; 
	      }

    case 229: {                   // tasto " Z " 
	        sel=12;  // comando di Clear display
                FuncDisp(ClearDisplay , 0);
                Disp_SPI_BSY = TRUE;
		peso_tst=1;
		temp_tst=11;
	        return sel;
	      }

    case 220: {                    // tasto " D " della tastiera
	        sel=13;            // seleziona messaggio da visualizzare
	        
                //SELECT();
                SD_mount = disk_initialize(0);
                //test = f_mount(0,SDcard);
                peso_tst=1;
		temp_tst=11;
                return SD_mount;
	      }

    case 227: {                    // tasto " A " della tastiera
	        sel=14;            
	        valore=DTimeTIM16;
 		dataDisp=10;
		peso_tst=1;
		temp_tst=11;
                return valore;
	      }

    case 233: {                    // tasto " 1 " della tastiera
	        sel=15;            
	        valore=P[0];
 		peso_tst=1;
		temp_tst=11;
                return valore;
	      }

    case 225: {                    // tasto " 2 " della tastiera
	        sel=15; 
	        valore=P[1];
	        peso_tst=1;
		temp_tst=11;
                return valore;
	      }

    case 217: {                    // tasto " 3 " della tastiera
	        sel=15;            
	        valore=P[2];
		peso_tst=1;
		temp_tst=11;
                ccyc_max=0;
                return valore;  //return valore;
	      }

    case 218: {                    // tasto " 4 " della tastiera
	        sel=15;            
	        	        
	 	peso_tst=1;
		temp_tst=11;
                return 4;  //return valore;
	      }

    case 209: {                    // tasto " 5 " della tastiera
	        sel=15;            
	        valore=ccyc;	
		peso_tst=1;
		temp_tst=11;
                return 5;  //return valore;
	      }

    case 201: {                    // tasto " 6 " della tastiera
	        sel=15;            
	        valore=ccyc_min;
		peso_tst=1;
		temp_tst=11;
                return 6;  //return valore;
	      }

    case 194: {                    // tasto " 7 " della tastiera
	        sel=15;            
	        valore=ccyc_max;
		peso_tst=1;
		temp_tst=11;
                return 7;  //return valore;
	      }

    case 193: {                    // tasto " 8 " della tastiera
	        sel=15;            
	        
		peso_tst=1;
		temp_tst=11;
                return 8;  //return valore;
	      }

    case 185: {                    // tasto " 9 " della tastiera
	        sel=15;           
	        
		peso_tst=1;
		temp_tst=11;
                return 9;  //return valore;
	      }

    case 186: {                    // tasto " 0 " della tastiera
	        sel=15;           
	        
		temp_tst=11;
                return 0;  //return valore;
	      }

    case 226: {                    // tasto " W " della tastiera
	        sel=16;           
	        valore = m;     // scrive ram2write nel settore specificatp
		temp_tst=11;
                return valore;
	      }

    case 213: {                    // tasto " V " della tastiera
	        sel=17;           
	        valore = display;  // visualizza  il contenuto di ram 
		temp_tst=11;
                return valore;
	      }

    case 214: {                       // barra spaziatrice
	                        
	        //valore=SD_mount;
                if ( TIM16->CCER == 0x0001 ) TIM16->CCER=0x0000;   // attiva OC1 ed OC1N  // TIM16->CR1=0x0000;
                else TIM16->CCER=0x0001;               
		peso_tst=1;
		temp_tst=11;
                return cifraP;
	      }


    case 238: {                       // tasto alt sinistro
	                        
	        valore=0;
		
		peso_tst=1;
		temp_tst=11;
                return 0;
	      }

    case 188: {                       // tasto ' i ' 
	        sel=18;
		valore=sel;    
                caricaDisp=1;
		display=2;       // carica terza e quarta riga file
		dataDisp=1;
		peso_tst=1;              // visualizza misure di corrente
		temp_tst=11;
                return valore;
	      }

    case 178: {                    // selezione parametri  tasto "P" tastiera
	        sel=19;           
                caricaDisp=1;
		display=21;       // carica righe 21 e 22
		dataDisp=1;	
		peso_tst=1;
		temp_tst=11;
                return 0;  //return valore;
	      }

  case 250: {                        // F1  scrivi riga su file data con  anno/mese/giorno ora:minuti:secondi canale:valore
	        sel=20;
		req_ram_write=1;
                s=sectSecondFile;
                peso_tst=1;
		temp_tst=11;
	        return cifraP;      // per non visualizzare il valore di ritorno
	      }

  case 249: {                        // F2
		sel=21;            
	        valore=DutCycTIM16;
		dataDisp=10;
                display=4;
                posiz=75;
		peso_tst=1;
		temp_tst=11;
                return DutCycTIM16;
	      }
              
  case 251: {                    // F3  rtc_settime(time);
		sel=22;            
	        //rtc_settime(Data0);
		dataDisp=10;
		peso_tst=1;
		temp_tst=11;
                return Data0->year;
	      }
              
  case 243: {                    // F4  rtc_gettime(time);
		sel=23;            
		dataDisp=10;
		peso_tst=1;
		temp_tst=11;
                if ( start_T3 ) start_T3 = FALSE;
                else start_T3 = TRUE;
                return cifraP; 
                /*rtc_gettime(DataOra);
                cifraP=-1;
                V_time = *DataOra;
                return (*DataOra).sec;//V_time.sec;*/
                
                
	      }
              
  case 210: {                    // tasto " R " della tastiera
	        sel=24;           
	        valore = numFlash;  // Numero di parametri flash da scrivere
		temp_tst=11;
                return valore;
	      }
              
    case 150: { temp_tst = 1;       // tasto 1 tastierino
	            break;}
    case 141: {  temp_tst = 2;      // tasto 2 tastierino  /  tasto freccia in basso
	            break;}
    case 133: { temp_tst = 3;       // tasto 3 tastierino
	            break;}
    case 148: { if(sel==19) {caricaDisp=1;display-=2;dataDisp=1;}   // freccia a sinistra
	              temp_tst = 4; // tasto 4 tastierino
	            break;}
    case 140: { temp_tst = 5;       // tasto 5 tastierino
	            break;}
    case 139: { if(sel==19) {caricaDisp=1;display+=2;dataDisp=1;}   // freccia a destra
                 else
	          temp_tst = 6;     // tasto 6 tastierino
	            break;}
    case 147: { temp_tst = 7;       // tasto 7 tastierino
	            break;}
    case 138: { temp_tst = 8;       // tasto 8 tastierino / tasto freccia in alto
	            break;}
    case 130: { temp_tst = 9;       // tasto 9 tastierino
	            break;}
    case 143: { temp_tst = 0;       // tasto 0 tastierino
	            break;}
    case 165: { temp_tst = 10;
	            break;}
    case 142: { temp_tst = 11;      //tasto "canc" tastierino
                return cifra;
	            break;}

    case 134: {                                            // tasto " + " tastierino
		if ( sel == 2 ) // fase
                 { 
	           
 	           temp_tst = 10;
                 }

		if ( sel == 3 ) // frequenza
                 { 
	           FreqSet+=10;
		   if (FreqSet > 1200 ) FreqSet=1200; 
 	           temp_tst = 11;
                 }

		if ( sel == 19 ) // parametri
                 { 
	           if (display == 21)  // tempo accelerazione
		    { 
                      Tramp_acc+=10;
		      if (Tramp_acc > 1200 ) Tramp_acc=1200; 
                    }
	           if (display == 23)  // tempo decelerazione
		    { 
                      Tramp_dec+=10;
		      if (Tramp_dec > 1200 ) Tramp_dec=1200; 
                    }

 	           temp_tst = 11;
                 }

                break;
	      }

    case 132: {                                           // tasto " - " tastierino
		if ( sel == 2 ) // fase
                 { 
	           
		   temp_tst=10;
                 }

		if ( sel == 3 ) // frequenza
                 {
		   FreqSet-=10;
	           if (FreqSet < 100 ) FreqSet = 100 ;
		   temp_tst = 11;
                 }

		if ( sel == 19 ) // parametri
                 { 
	           if (display == 21)  // tempo accelerazione
		    { 
                      Tramp_acc-=10;
		      if (Tramp_acc < 10 ) Tramp_acc=10; 
                    }
	           if (display == 23)  // tempo decelerazione
		    { 
                      Tramp_dec-=10;
		      if (Tramp_dec < 10 ) Tramp_dec=10; 
                    }
 	           temp_tst = 11;
                 }
                break;
	      }

    default:  { 
	        //if( sel != 0 )  { CursorOff =0; }
	        return cifra;
              }
  } 
  
  if ( temp_tst >= 0 && temp_tst <10) 
   {
     if ( peso_tst == 1    ) { valore=temp_tst; peso_tst=10;return valore;}
     if ( peso_tst == 10   ) { valore=valore*10+temp_tst; peso_tst=100;return valore;}
     if ( peso_tst == 100  ) { valore=valore*10+temp_tst; peso_tst=1000;return valore;}
     if ( peso_tst == 1000 ) { valore=valore*10+temp_tst; return valore;}
     if (valore > 65535 ) valore = 65535;
     return valore;
   }
  
  if ( temp_tst == 10) 
   { 
     peso_tst=1;
     temp_tst=11;
     dataDisp=10;
     
     if ( sel == 1 )   
      {
        AutRelTIM16 = valore;
        TIM16->ARR = AutRelTIM16;
        return valore;
      }

     if ( sel == 2 )   
      {
	DutCycTIM16=valore;
        TIM16->CCR1=DutCycTIM16;
        return valore;
      }

     if ( sel == 3 )   
      {
        Tsesti=valore;
        TIM7->ARR = Tsesti;
	  //if ( dataDisp < 10 )
	  // { 
	  // FreqSet = valore;
          //}

        if ( impulsi == 1 )
	 {
           FreqSet=valore;
         }
	else
	 {
	   LastFreq=valore;
	   FreqSet=valore;
	   //FreqAct=valore;
         }
        return valore;
      }

    if ( sel == 4 )         // tasto " S "
     {
       m = valore;
       disk_read(0,ram,m,1);  
       caricaDisp=1;     // fa partire il caricamento del settore e la visualizzazione dei primi 32 bytes
       //search=1;
       return valore;
     }

    if ( sel == 5 )   
     {
       testpos = valore;
       return testpos;
     }

    if ( sel == 7 )   
     {
       disp_addr_char = valore;
       FuncDisp(SetDDram , disp_addr_char);
       Disp_SPI_BSY = TRUE;
       return disp_addr_char;
     }

    if ( sel == 9 )   
     {
       byteSect = valore;
       return byteSect;
     }

    if ( sel == 13 )   
     {
       display = valore;   
       return display;
     }

    if ( sel == 14 )   
     {
      DTimeTIM16=valore;
      return valore;
     }

    if ( sel == 15 )   
     {
       ccyc_min = valore;   
       return valore;
     }


    if ( sel == 16 )       // tasto " W "
     {
      m=valore; 
      
      n=ram;         
      for(k=0;k<512;k++) {*n=filler;n++;}       // per inizializzare ram

      STM32F100PinToggle( GPIOC,GPIO_Pin_8 );
      valore=disk_write(0,ram,m,1);
      STM32F100PinToggle( GPIOC,GPIO_Pin_8 );
      return valore;
     }

     if ( sel == 17 )       // tasto " V "  visualizza le righe display e display + 1 caricate in ram
     {
      //if ( valore > 28 ) valore = 28;  // per non superare 512
      display=valore;
      return cifraP;   // per bloccare la visualizzazione del return
     }

     if ( sel == 19 )   
     {
       if (display == 21)  // tempo accelerazione
	{ 
	  //write_flash_num(Tramp_acc);
	  valore=Tramp_acc;
         }
	if (display == 23)  // tempo accelerazione
	 { 
	   //write_flash_num(Tramp_dec);
	   valore=Tramp_dec;
         }
	return valore;
     }

     if ( sel == 20 )   
      {
        AutRelTIM17 = valore;
        TIM17->ARR = AutRelTIM17;
        return valore;
      }
 
     if ( sel == 21 )   
      {
	DutCycTIM17=valore;
        TIM17->CCR1=DutCycTIM17;
        return valore;
      }
     
     if ( sel == 24 )       // tasto " R "  attiva la scrittura dei parametri da SD a flash
     {
      numFlash=valore;
      return cifraP;   // per bloccare la visualizzazione del return
     }
     
     
  }
 
}

// ********************  end tastierino  **********************


//***********************************************************************************************************************

void visualizza(void)
{
  if ( idisp == 0 ) 
  { 
   FuncDisp(SetDDram , 0);
  }
 if ( ( idisp > 0) && (idisp <= 16) )
  { 
   FuncDisp(SetValDisp , *pRiga);
   pRiga++;
  }
 if ( idisp == 17 )
  { 
   pRiga++;
   pRiga++;
   FuncDisp(SetDDram , 64);
  }
 if ( ( idisp > 17) && (idisp <= 34) )
  { 
   FuncDisp(SetValDisp , *pRiga);
   pRiga++;
  } 

 idisp++; 

}  

//***********************************************************************************************************************

void FuncDisp(ActFun fun , uint8_t val) 
{
  if ( fun == SetValDisp )  STM32F100PinSet( GPIOB , GPIO_Pin_14 , TRUE);
  else STM32F100PinSet( GPIOB , GPIO_Pin_14 , FALSE);

  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
    
  switch (fun)
  {
    case SetDDram     :  { SPI1->DR =(0x80|val);break;}
    case SetCGram     :  { SPI1->DR =0x40|val;break;}
    case DisplayOn    :  { SPI1->DR =0x0C;break;}
    case ClearDisplay :  { SPI1->DR =0x01;break;}
    case SetDisplay   :  { SPI1->DR =0x38;break;}
    case ShowCursor   :  { SPI1->DR =0x0F;break;}
    case HideCursor   :  { SPI1->DR =0x0C;break;}
    case SetValDisp   :  { SPI1->DR = val;break;}
    
    default: ;
  }
  
  Disp_SPI_BSY=TRUE;
}  

//***********************************************************************************************************************

void Display (uint32_t z,uint8_t pos)        //-> display data entry
{
 
 switch (sequenza)
  {
    case 1: { 
              cifre(z);
              FuncDisp(ShowCursor, 0);
              Disp_SPI_BSY = TRUE;
	      break;}
    case 2: { 
              FuncDisp(SetDDram , pos);               // imposta posizione cursore
              Disp_SPI_BSY = TRUE;
	      break;}
    case 3: { 
              FuncDisp(SetValDisp , quinta+48);           
              Disp_SPI_BSY = TRUE;
	      break;}
    case 4: { 
              FuncDisp(SetValDisp , quarta+48);                
              Disp_SPI_BSY = TRUE;
	      break;}
    case 5: {
              FuncDisp(SetValDisp , terza+48);                
              Disp_SPI_BSY = TRUE;
	      break;}
    case 6: {
              FuncDisp(SetValDisp , seconda+48);                
              Disp_SPI_BSY = TRUE;
	      break;}
    case 7: {
              FuncDisp(SetValDisp , prima+48);                
              Disp_SPI_BSY = TRUE;
	      cifraP=cifra;
	      sequenza=0;
	      break;}
    default: ;
  } 

}

//***********************************************************************************************************************
//***** Legge il settore flash SD e lo memorizza in ram , memorizza in pRiga il messaggio da visualizzare    ************

void seleziona_messaggio(uint8_t disp)  // sect è il settore dove inizia il file contenente i testi
{                                       // ogni riga è costituita da 18 bytes ogni settore da 512 bytes
  
 int temp,temp1;
 
 curSectTesti=sectFirstFile;//sectSecondFile;
    
 if ( (disp != displayP ) ) {
   temp = lrint(disp*18/512); 
   temp1 = (disp*18-temp*512);
   curSectTesti+=temp;
   disk_read(0,ram,curSectTesti,1);                              // legge 1 settore sul drive 0 a partire da sect  
  }
 else
 {  
    //temp=0;
    temp1=curPosSectT;
 } 
 
 n=minRam;
 pRiga=&riga[0];
 
 curPosSectT=temp1;
 
 while(0 <= temp1-- ) 
 {  
   n++;    // n punta al byte selezionato
 }

 temp=0;
 riga[temp]=*(n-1);
 
 while(34 >= temp++ ) 
 {  
   riga[temp]=*n;
   n++;
   if ( n == MaxRam )
    {  
      curSectTesti++;
      disk_read(0,ram,curSectTesti,1);    // legge un settore sul drive 0 a partire da sect
      n=minRam;
    }
 }

 m=curSectTesti;
 AddrVarRam=n-1;
 SectVarFlash=m;
 pRiga=riga;

}

//***************************************************************************************************************
//**  carica in riga il messaggio da visualizzare prelevandolo dalla flash del micro  **************************
void sel_mess_da_flash(uint8_t disp)
{
  uint32_t temp;
  uint8_t i;
  Address =  AREA_TEXT_START_ADDR;
  Address += 16*disp;  
  for( i=0;i<=12;i+=4 )
  {     
    Data= (*(__IO uint32_t*) Address); 
    Address += 4;
    temp = Data;
    riga[i] = temp >> 24;
    temp = Data & 0xFF0000;
    riga[i+1] = temp >> 16;
    temp = Data & 0xFF00;
    riga[i+2] = temp >> 8;
    temp = Data & 0xFF;
    riga[i+3] = temp;
  }
  riga[16]=10;
  riga[17]=13;
  for( i=18;i<=30;i+=4 )
  {     
    Data= (*(__IO uint32_t*) Address); 
    Address += 4;
    temp = Data;
    riga[i] = temp >> 24;
    temp=Data & 0xFF0000;
    riga[i+1] = temp >> 16;
    temp=Data & 0xFF00;
    riga[i+2] = temp >> 8;
    temp = Data & 0xFF;
    riga[i+3] = temp;
  }
 //pRiga=&riga[0]; 
 pRiga=riga;
  
}

//***************************************************************************************************************

void write_data(MyChannel ch) //  scrivi riga su ram_data con  anno/mese/giorno ora:minuti:secondi canale:valore
{ 
 
 
 switch (sequenzaD)   //  cifreD(z);
  {
   
    case 1: { 
              //d=ram_data;
              if (V_time.year != yearP) {
                   cifreD(V_time.year);   // scrivi  anno
                   y4=quartaD+48;
                   y3=terzaD+48;
                   y2=secondaD+48;
                   y1=primaD+48;
                   yearP=V_time.year;
              }    
              *d=y4;inc_d();
              *d=y3;inc_d();
              *d=y2;inc_d();
              *d=y1;inc_d();
              *d='/';inc_d();            
	      break;}
    case 2: { 
              if (V_time.month != monthP) {
                   cifreD(V_time.month);   // scrivi  mese
                   m2=secondaD+48;
                   m1=primaD+48;
                   monthP=V_time.month;
              }    
              *d=m2;inc_d();
              *d=m1;inc_d();
              *d='/';inc_d();            
	      break;}
    case 3: { 
              if (V_time.mday != mdayP) {
                   cifreD(V_time.mday);   // scrivi  giorno
                   g2=secondaD+48;
                   g1=primaD+48;
                   mdayP=V_time.mday;
              }    
              *d=g2;inc_d();
              *d=g1;inc_d();
              *d=' ';inc_d();            
	      break;}
    case 4: { 
              if (V_time.hour != hourP) {
                   cifreD(V_time.hour);   // scrivi  ora
                   h2=secondaD+48;
                   h1=primaD+48;
                   hourP=V_time.hour;
              }    
              *d=h2;inc_d();
              *d=h1;inc_d();
              *d=':';inc_d();            
	      break;}
    case 5: {
              if (V_time.min != minP) {
                   cifreD(V_time.min);   // scrivi  minuti
                   p2=secondaD+48;
                   p1=primaD+48;
                   minP=V_time.min;
              }    
              *d=p2;inc_d();
              *d=p1;inc_d();
              *d=':';inc_d();            
	      break;}
    case 6: {
              if (V_time.sec != secP) {
                   cifreD(V_time.sec);   // scrivi  secondi
                   s2=secondaD+48;
                   s1=primaD+48;
                   secP=V_time.sec;
              }    
              *d=s2;inc_d();
              *d=s1;inc_d();
              *d=' ';inc_d();
              
	      break;}
    case 7: {
              *d='c';inc_d();
              *d='h';inc_d();
              switch (ch)
               {
                    case ch1 :  { data_val=ADC1ConvertedValue[CH1_PC0];*d='1';inc_d();break;}
                    case ch2 :  { data_val=ADC1ConvertedValue[CH2_PC1];*d='2';inc_d();break;}
                    case ch3 :  { data_val=ADC1ConvertedValue[CH3_PC2];*d='3';inc_d();break;}   
                    case ch4 :  { data_val=ADC1ConvertedValue[CH4_PC3];*d='4';inc_d();break;}   
                    case ch5 :  { data_val=ADC1ConvertedValue[CH5_PC4];*d='5';inc_d();break;}   
                    case ch6 :  { data_val=ADC1ConvertedValue[CH6_PC5];*d='6';inc_d();break;}   
                    case ch7 :  { data_val=ADC1ConvertedValue[CH7_PA0];*d='7';inc_d();break;}   
                    case ch8 :  { data_val=ADC1ConvertedValue[CH8_PA1];*d='8';inc_d();break;}   
                    case ch9 :  { data_val=ADC1ConvertedValue[CH9_PA2];*d='9';inc_d();break;}   
                    case ch10 :  { data_val=ADC1ConvertedValue[CH10_PA3];*d='1';inc_d();*d='0';inc_d();break;}   
                    case ch11 :  { data_val=ADC1ConvertedValue[CHT];*d='1';inc_d();*d='1';inc_d();break;} 
                default: ;
               }
              *d=' ';inc_d();
              if (data_val != data_valP) {
                   cifreD(data_val);   // scrivi  valore
                   v4=quartaD+48;
                   v3=terzaD+48;
                   v2=secondaD+48;
                   v1=primaD+48;
                   data_valP=data_val;
              }    
              *d=v4;inc_d();
              *d=v3;inc_d();
              *d=v2;inc_d();
              *d=v1;inc_d();
              
              
              *d=13;inc_d();              
              *d=10;inc_d();              
	      sequenzaD=0;
              req_ram_write=0;
              disk_write(0,ram_data,curSectDati,1);
	      break;}
    default: ;
  } 
  
}

//*****************************************************************************************

void Leggi_Tastiera(void) // eseguita ogni fronte di discesa clk tastiera
{                         // interrupt su PA15
  
  if (!start_T1)
    {
      if( bitcount<11 && bitcount>2 )
      {
        scandata =  scandata>>1;
        if ( GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15) != 0 ) 
          scandata = scandata | 0x80;
      }
 
      if (--bitcount == 0 )
      {
        n_byte = ~scandata;
         if(n_byte != 31 ) 
        { 
         tim1=150;
         start_T1=TRUE;
         data_recived = TRUE;
         AFIO->EXTICR[0]=0x0000; // scollega linea interrupt tastiera
        }
        bitcount = 11;
        scandata=0;
      }
    }      
}


//*****************************************************************************************

//->timer
void TimingDelay_Decrement(void) // eseguita ogni mSec da SysTick_Handler
{                                // interrupt SysTick
  
  
  if(start_T1)           // timer tastiera
  {
   if(val_tim1++  > tim1 ) 
   {
    start_T1=FALSE;
    val_tim1=0;
    AFIO->EXTICR[0]=0x1000; // ricollega interrupt tastiera

   } 
  }

  if(start_T2)           // timer display
  {
   if(val_tim2++  > tim2 ) 
   {
    start_T2=FALSE;
    val_tim2=0;
   } 
  }

   if(val_tim3++  > tim3 )   // tim3 = 1000 per orologio
   {
    rtc_gettime(DataOra);
     //start_T3=FALSE;
    val_tim3=0;
    if(start_T3) TimeDate=TRUE;
   } 

  
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}



//*****************************************************************************************

//->timer
void TimingAuxOp(void) // eseguita ogni mSec da SysTick_Handler
{                                // gestione operazioni ausiliarie
  ni++;
  if ( ni >= 10 ) {
    ni = 0;
    disk_timerproc();
  }  
  
  if( ccyc >= ccyc_max ) ccyc_max=ccyc;
  if( (ccyc == 0) && (ccyc_P == 0 ) ) ccyc_min++;
  ccyc_P=ccyc;
  ccyc=0;
  
  if (20 < nm++ ) {
     nm=0;
     n_medie_cur++;
     cum_medie += ADC1ConvertedValue[CHT];
     if ( n_medie <= n_medie_cur ) {
       n_medie_cur=0;
       T_media=cum_medie/n_medie;
       cum_medie=0;      
     }
  }
  
}