
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "main.h"
#include "STM32vldiscovery.h"
#include "FunSys.h"

extern __IO uint8_t     idisp,sequenza,CursorOff,riga[];
extern __IO uint8_t     *pRiga,*n,*ram,*MaxRam,*minRam,*AddrVarRam;
extern __IO uint8_t     prima,seconda,terza,quarta,cifra,cifraP;
extern __IO uint32_t    sectP,SectVarFlash;


void visualizza(void)
{
 if ( idisp == 0 ) 
  { 
   SetDDram(0);
   idisp++;
   return;
  }
 if ( ( idisp > 0) && (idisp <= 16) )
  { 
   SetValDisp(*pRiga);
   pRiga++;
   idisp++;
   return;
  }
 if ( idisp == 17 )
  { 
   pRiga++;
   pRiga++;
   SetDDram(64);
   idisp++;
   return;
  }
 if ( ( idisp > 17) && (idisp <= 34) )
  { 
   SetValDisp(*pRiga);
   pRiga++;
   idisp++;
   return;
  } 

}


//  ***************************************************************

void Display (int z,uint8_t pos)        // display data entry
{
 switch (sequenza)
  {
    case 1: { 
             if ( z > 9999 ) z = 9999;
             if ( z < 1 ) z = 0;
             quarta = ceil(z/1000);
             terza=ceil((z-quarta*1000)/100);
             seconda=ceil((z-quarta*1000-terza*100)/10);
             prima=z-quarta*1000-terza*100-seconda*10;
 	     ShowCursor();                         // imposta indirizzo iniziale
	     break;}
    case 2: {SetDDram(pos);      // imposta posizione cursore
	         break;}
    case 3: {SetValDisp(quarta+48);
	         break;}
    case 4: {SetValDisp(terza+48); 
	         break;}
    case 5: {SetValDisp(seconda+48);
	         break;}
    case 6: {SetValDisp(prima+48);
	     CursorOff = 1;
	     cifraP=cifra;
	     sequenza=0;
	     break;}
    default: ;
  } 

}

void seleziona_messaggio(uint32_t sect,uint8_t display)
{
  
 int temp,temp1;
 temp = ceil(display*18/512);
 temp1 = (display*18-temp*512);
 
 if(temp != 0)
  {
    while(0 < temp-- ) sect++;

  }
 
 if( sect != sectP )
  {
    //df_df_2_ram(sect,ram);     // legge df scrive in ram
    sectP = sect;
  }
 n=ram;
 pRiga=riga;
 
 while(0 < temp1-- ) 
 {  
   n++;    // n punta al byte selezionato
 }
 
 temp=34;

 while(0 < temp-- ) 
 {  
   *pRiga=*n;
   n++;    
   pRiga++;
   if ( n == MaxRam )
    {  
      sect++;
      //df_df_2_ram(sect,ram);
      n=minRam;
    }
 }

 AddrVarRam=n-1;
 SectVarFlash=sect;
 idisp=0;       // fa partire la visualizzazione dei dati caricati in ram
 pRiga=riga;

}
