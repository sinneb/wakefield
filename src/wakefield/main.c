/*

wakefield synthesizer

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h> /* memset */
#include <math.h>

#include "stm32746g_discovery.h"
#include "stm32746g_discovery_audio.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_sd.h"
#include "stm32f7xx_hal.h"

#include "common/clockconfig.h"

#include "smallwaves.h"

#include "soundpipe.h"

// defines
#define VOLUME 90
#define SAMPLE_RATE 48000
#define AUDIO_DMA_BUFFER_SIZE 4096
#define AUDIO_DMA_BUFFER_SIZE2 (AUDIO_DMA_BUFFER_SIZE >> 1)

/* SOUNDPIPE */

static sp_data *sp;
static sp_ftbl *ft[12];
static sp_ftbl *work;
//static sp_posc3 *posc3[8];
static sp_osc *osc;
static sp_osc *osc2;

// FATFS: sdcard access 
#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
FIL audio_file[13];
FIL wavetable_file[13];
UINT bytes_read[13];
UINT prev_bytes_read;
FATFS* fs;
FATFS SDFatFs;
DWORD fre_clust;
static char usb_drive_path[4];

// datastructures for wavetables
float f_wavetable[13][600];
uint8_t d_wavetable[13][120];
uint16_t vco1wave = 1;
uint16_t vco2wave = 2;
uint16_t vco3wave = 1;
uint16_t vco4wave = 2;
uint16_t vco5wave = 1;
uint16_t vco6wave = 2;
uint16_t vco7wave = 1;
uint16_t vco8wave = 2;
uint8_t audioBufferFile[4][AUDIO_DMA_BUFFER_SIZE];

int teller1, teller2;

// audio buffers
static int16_t int_bufProcessedOut[AUDIO_DMA_BUFFER_SIZE];
static uint8_t audioOutBuf[AUDIO_DMA_BUFFER_SIZE];

extern SAI_HandleTypeDef haudio_out_sai;

// header
void initAudio();
void computeAudio();
void ConfigureADC();
void ConfigureDMA();
void drawVoiceInterface(uint8_t voice);
void drawTemplate(char potfunctions[25], char pagetitle[20]);
void openSCwaveform(uint16_t SCwaveformID, uint16_t filenameID);
void drawWavetable(uint16_t wavetableID, uint16_t x, uint16_t starty);

uint32_t g_ADCValue;
int g_MeasurementNumber;
ADC_HandleTypeDef g_AdcHandle;
DMA_HandleTypeDef  g_DmaHandle;
enum{ ADC_BUFFER_LENGTH = 8192 };
uint32_t g_ADCBuffer[ADC_BUFFER_LENGTH];

uint16_t ADCchannelValues[4];

uint8_t audioReady = 0;
uint8_t process_audio = 1;

static TS_StateTypeDef rawTouchState;
uint16_t runOnce = 0;
char *touchMap = "main";

int main() {
  CPU_CACHE_Enable();
  HAL_Init();
  SystemClock_Config(); 
  ConfigureADC();
  ConfigureDMA();
  HAL_ADC_Start_DMA(&g_AdcHandle, g_ADCBuffer, ADC_BUFFER_LENGTH);
  
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Off(LED_GREEN);
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  

  // Init LCD and Touchscreen
  BSP_LCD_Init();
  if (BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize()) == TS_OK) {
    BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	  BSP_TS_ITConfig();
  }  
  
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    
  // Load SD Driver & mount card
  if (FATFS_LinkDriver(&SD_Driver, usb_drive_path) != 0) {
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)"SD Driver error", CENTER_MODE);
    Error_Handler();
  }

  if(BSP_SD_IsDetected()!=SD_PRESENT) {
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)"No SDcard present", CENTER_MODE);
		Error_Handler();
  }
  
  SD_Driver.disk_initialize(0);
  
	if(f_mount(&SDFatFs, (TCHAR const*)usb_drive_path, 0) != FR_OK) {
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)"SD Mount error", CENTER_MODE);
		Error_Handler();
	}
  
	if(f_getfree((TCHAR const*)usb_drive_path, &fre_clust, &fs) != FR_OK) {
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)"SD access error", CENTER_MODE);
		Error_Handler();
	}
  

  initAudio();
  
  sp_create(&sp);
  sp->sr = 48000;
  
  sp_ftbl_create(sp, &ft[0], 600);
  sp_ftbl_create(sp, &work, 600);

  sp_osc_create(&osc);
  sp_osc_create(&osc2);
  
  sp_osc_init(sp, osc, ft[0], 0);
  osc->freq = 200;
  osc->amp = 0.5;
  
  sp_osc_init(sp, osc2, ft[0], 0);
  osc2->freq = 250;
  osc2->amp = 0.3;
  
  while (1) {		
    // determine audioReady moment
    // reserve 700ms for initialisation
    // the delay needs to match some initialisation timer?
    // somehow this initialisation if different upon reset of coldboot
    // investigate further
    if(audioReady==0 && HAL_GetTick()>700) {
      audioReady=1;
      
      openSCwaveform(0, vco1wave);
  
      for(int i = 0; i < ft[0]->size; i++){
          ft[0]->tbl[i] = f_wavetable[0][i];
      }
      
      drawVoiceInterface(1);
      
    }
    
    //BSP_LED_Toggle(LED_GREEN);
    
    //__disable_irq();
    //openSCwaveform(0, vco1wave);
    //__enable_irq();
    
    //if (HAL_ADC_PollForConversion(&g_AdcHandle, 1000000) == HAL_OK)
    //        {
        //        g_ADCValue = HAL_ADC_GetValue(&g_AdcHandle);
      //          g_MeasurementNumber++;
               // char a[] = "";
 //                uint32_t digu = (uint32_t)(ADCchannelValues[0]);
 //              //uint16_t newval = g_ADCValue - (uint16_t)*blsaw->freq;
 //                      sprintf(a, "%ld", digu);
 //                                      BSP_LCD_DisplayStringAt(50, 50, (uint8_t *)a, LEFT_MODE);
 //                                                             osc->freq=(ADCchannelValues[0] / 10);
 //    // //     }
 //                                //if(g_ADCValue>1000) {
                                //  *blsaw->freq = g_ADCValue;
                                //}
  }

  return 0;
}


void openSCwaveform(uint16_t SCwaveformID, uint16_t filenameID) {
  // open single waveform
  // skip 44 (offset)
  // read 1200 (samplelength = 600 2's complement)
  //char* theFilename = printf("0:single%d.wav", filenameID);
  char theFilename[] = "";
  sprintf(theFilename, "0:AKWF_01%02d.wav", filenameID);
  f_close(&wavetable_file[SCwaveformID]);
  __disable_irq();
  uint16_t openresult = f_open(&wavetable_file[SCwaveformID], theFilename, FA_READ);
  __enable_irq();
  //uint16_t openresult = 3
  if ( openresult == FR_OK) {
  // f_open ok
    f_lseek(&wavetable_file[SCwaveformID], 44);
  } else {
    char a[] = "";
    sprintf(a, "%d", openresult);
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                              (uint8_t *)"wavetable load error", CENTER_MODE);
    Error_Handler();
  }

  // read chunks from USB
  // single use of audiobuffer 0, will be reused in context later on
  // single use of bytes_read 0
  f_read(&wavetable_file[SCwaveformID], audioBufferFile[0], 1200, &bytes_read[0]);
  
  // init drawtable iterator
  int dt_int = 0;

  // 2's-complement signed integers -> short (-32k -> +32k) -> float (-1 -> +1)
  for (int j=0; j < 1200; j=j+2) {

    // // to short
//     // read 2 values for 2's-complement
//     // convert to 1 short value
         short tempshort = (short)(audioBufferFile[0][j+1]<<8 | ((audioBufferFile[0][j]) & 0xFF));
//
//     // to float
//     // use j/2 because of conversion from 2's comp to short
//          float ab = (float)tempshort/32768.f;
//          f_wavetable[SCwaveformID][j/2] = ab + 1;//1000.f;
       f_wavetable[SCwaveformID][j/2] = (float)(tempshort/32768.f);
//
    // // fill drawtable with 120 value
             if(j%10==0) {
                   d_wavetable[SCwaveformID][dt_int] = ((tempshort + 32768) / 656);
                            dt_int++;
                          }
  }
}

void drawWavetable(uint16_t wavetableID, uint16_t x, uint16_t starty) {
  //uint16_t x = xstart;
  //uint16_t starty = ystart;
  uint16_t y=starty-d_wavetable[wavetableID][119];
  for(int i = 0; i < 120; i++) {
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawLine(x,y,x+1,starty-d_wavetable[wavetableID][119-i]);
    x++;
    y=starty-d_wavetable[wavetableID][119-i];
  }
}



void drawWavetableSmall(uint16_t wavetableID, uint16_t x, uint16_t starty) {
  //uint16_t x = xstart;
  //uint16_t starty = ystart;
  // uint16_t y=(starty-d_wavetable[wavetableID][119])/3;
//   for(int i = 0; i < 120; i=i+3) {
//     BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
//     BSP_LCD_DrawLine(x,y,x+1,((starty-d_wavetable[wavetableID][119-i])/3)+1);
//     x++;
//     y=((starty-d_wavetable[wavetableID][119-i])/3)+1;
//   }
  
  uint16_t y = starty + 5 + 50 - sswave[wavetableID][49];
  //uint16_t x = x + 5;
  for (int j=1; j < 50; j++) {
    BSP_LCD_DrawLine(x,y,x+1,starty + 5 + 50 - sswave[wavetableID][49-j]);
    x++;
    y = starty + 5 + 50 - sswave[wavetableID][49-j];
  }
}


void ConfigureADC()
{
    GPIO_InitTypeDef gpioInit;
 
    // MCU pin for A1 = PF10
    // Function for PF10 = ADC3_IN8
    // 5v compatible port
    __GPIOF_CLK_ENABLE();
    __GPIOA_CLK_ENABLE();
    __ADC3_CLK_ENABLE();
 
    gpioInit.Pin = GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_8;
    gpioInit.Mode = GPIO_MODE_ANALOG;
    gpioInit.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &gpioInit);

    gpioInit.Pin = GPIO_PIN_0;
    gpioInit.Mode = GPIO_MODE_ANALOG;
    gpioInit.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpioInit);
 
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
 
    ADC_ChannelConfTypeDef adcChannel;
 
    g_AdcHandle.Instance = ADC3;
 
    g_AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
    g_AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
    g_AdcHandle.Init.ScanConvMode = ENABLE;
    g_AdcHandle.Init.ContinuousConvMode = ENABLE;
    g_AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    g_AdcHandle.Init.NbrOfDiscConversion = 0;
    g_AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    g_AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
    g_AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_AdcHandle.Init.NbrOfConversion = 4;
    g_AdcHandle.Init.DMAContinuousRequests = ENABLE;
    g_AdcHandle.Init.EOCSelection = DISABLE;
 
    HAL_ADC_Init(&g_AdcHandle);
    
    adcChannel.Channel = ADC_CHANNEL_0;
    adcChannel.Rank = 1;
    adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel) != HAL_OK)
    {
      Error_Handler();
    }

      /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
      */
    adcChannel.Channel = ADC_CHANNEL_8;
    adcChannel.Rank = 2;
    if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel) != HAL_OK)
    {
      Error_Handler();
    }

      /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
      */
    adcChannel.Channel = ADC_CHANNEL_7;
    adcChannel.Rank = 3;
    if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel) != HAL_OK)
    {
      Error_Handler();
    }

      /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
      */
    adcChannel.Channel = ADC_CHANNEL_6;
    adcChannel.Rank = 4;
    if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel) != HAL_OK)
    {
      Error_Handler();
    }
    
}

void ConfigureDMA()
{
    __DMA2_CLK_ENABLE(); 
    g_DmaHandle.Instance = DMA2_Stream1;
  
    g_DmaHandle.Init.Channel  = DMA_CHANNEL_2;
    g_DmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_DmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    g_DmaHandle.Init.MemInc = DMA_MINC_ENABLE;
    g_DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    g_DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    g_DmaHandle.Init.Mode = DMA_CIRCULAR;
    g_DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;
    g_DmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;         
    g_DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    g_DmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;
    g_DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE; 
    
    HAL_DMA_Init(&g_DmaHandle);
    
    __HAL_LINKDMA(&g_AdcHandle, DMA_Handle, g_DmaHandle);

    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);   
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
    {
      // uint32_t a2 = 0;
      // for(int i=1000;i<1400;i=i+4) {
      //   a2 = a2 + g_ADCBuffer[i+2];
      // }
      // a2 = a2 / 100;
      
//      if(abs(g_ADCValue-a2)>2) {g_ADCValue = a2;}
      
//      {
        uint32_t channels[4] = {0,0,0,0};
        for(int i=1000;i<1800;i=i+4) {
          channels[0] = channels[0] + g_ADCBuffer[i+0];
          channels[1] = channels[1] + g_ADCBuffer[i+1];
          channels[2] = channels[2] + g_ADCBuffer[i+2];
          channels[3] = channels[3] + g_ADCBuffer[i+3];
        }
      
        channels[0] = channels[0] / 200;
        channels[1] = channels[1] / 200;
        channels[2] = channels[2] / 200;
        channels[3] = channels[3] / 200;

        if(abs(ADCchannelValues[0]-channels[0])>1) {ADCchannelValues[0] = channels[0];}
        if(abs(ADCchannelValues[1]-channels[1])>1) {ADCchannelValues[1] = channels[1];}
        if(abs(ADCchannelValues[2]-channels[2])>1) {ADCchannelValues[2] = channels[2];}
        if(abs(ADCchannelValues[3]-channels[3])>1) {ADCchannelValues[3] = channels[3];}
      
        //g_ADCValue = ADCchannelValues[2];
      
      //g_ADCValue = g_ADCBuffer[2];//std::accumulate(g_ADCBuffer, g_ADCBuffer + ADC_BUFFER_LENGTH, 0) / ADC_BUFFER_LENGTH;
        //g_MeasurementNumber += ADC_BUFFER_LENGTH;
        //if(abs(g_ADCValue - (uint16_t)*blsaw->freq)>5) {
        //  *blsaw->freq = g_ADCValue;
        //}
       // 
    }
 
void DMA2_Stream1_IRQHandler()
{
    HAL_DMA_IRQHandler(&g_DmaHandle);
}

void ADC_IRQHandler()
{
    HAL_ADC_IRQHandler(&g_AdcHandle);
}

void initAudio() {
    if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, VOLUME, SAMPLE_RATE) != 0) {
      BSP_LCD_DisplayStringAt(5, 5, (uint8_t *)"initAudio error", LEFT_MODE);
      Error_Handler();
    }
    
    BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
    BSP_AUDIO_OUT_SetVolume(VOLUME);
    BSP_AUDIO_OUT_Play((uint16_t *)audioOutBuf, AUDIO_DMA_BUFFER_SIZE);

}

void AUDIO_OUT_SAIx_DMAx_IRQHandler(void) {
  HAL_DMA_IRQHandler(haudio_out_sai.hdmatx);
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
  // clear global buffer
  memset(int_bufProcessedOut, 0, sizeof int_bufProcessedOut);
	computeAudio();
	memcpy(audioOutBuf, int_bufProcessedOut, AUDIO_DMA_BUFFER_SIZE2);  
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
  memset(int_bufProcessedOut, 0, sizeof int_bufProcessedOut);
	computeAudio();
	memcpy(&audioOutBuf[AUDIO_DMA_BUFFER_SIZE2], int_bufProcessedOut, AUDIO_DMA_BUFFER_SIZE2);
  
  BSP_LED_Toggle(LED_GREEN);
}

void computeAudio() {
  
  // compute 2048 samples -> 512 audiosamples
  for(int i = 0; i < 1024; i+=2) {
    SPFLOAT tmp = 0, tmp2=0, tmp3=0, tmp4=0, tmp5=0, tmp6=0, mixOut;
    
    if(process_audio==1) {
    
    sp_osc_compute(sp, osc, NULL, &tmp);
    sp_osc_compute(sp, osc2, NULL, &tmp3);
    
    mixOut = (0 * tmp3 + 0.5 * tmp);
    
    } else {
     mixOut = 0;
    }

    // channel outputs in 2's comp / signed int
    int_bufProcessedOut[i] = (mixOut * 32767);
    int_bufProcessedOut[i+1] = (mixOut * 32767);
  
    }
}

void drawTemplate(char potfunctions[25], char pagetitle[20]) {
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(5, 5, (uint8_t *)pagetitle, LEFT_MODE);
  BSP_LCD_DisplayStringAt(5, 5, (uint8_t *)"wakefield  m", RIGHT_MODE);
  BSP_LCD_DrawHLine(0,25,480);
  BSP_LCD_DrawHLine(0,26,480);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(65, 255, (uint8_t *)potfunctions, LEFT_MODE);
  BSP_LCD_DrawHLine(0,248,480);
  BSP_LCD_DrawHLine(0,249,480);
  
  BSP_LCD_FillRect(0,232,40,40);
  BSP_LCD_FillRect(440,232,40,40);
}

void drawVoiceInterface(uint8_t voice) {
  drawTemplate("vol1   vol2   vol3   main", "voice 1: WAVES");
  if(voice==1) {
    BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
    BSP_LCD_FillRect(10,50,140,110);
    BSP_LCD_FillRect(10,166,67,55);
    BSP_LCD_FillRect(83,166,67,55);
    
    BSP_LCD_FillRect(160,50,140,110);
    BSP_LCD_FillRect(310,50,140,110);
    
    drawWavetable(0,20,155);
  }
}

void drawWaveSelector() {  
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_FillRect(10,30,460,237);
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
  BSP_LCD_FillRect(12,32,456,233);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  
  uint8_t wavetableID = 0; 
  for (int row=0; row < 3; row++) {
    for (int col=1; col < 6; col++) {    
      drawWavetableSmall(wavetableID,(col*70)+5,45+(row*70));
      wavetableID++;
    } 
  }
  
  
  //__disable_irq();
  //openSCwaveform(1,1);
  //__enable_irq();
  //BSP_LCD_SetTextColor(LCD_COLOR_RED);
  //BSP_LCD_FillRect(12,12,456,20);
  
  //BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  //BSP_LCD_SetBackColor(LCD_COLOR_RED);
  //BSP_LCD_SetFont(&Font12);
  //BSP_LCD_DisplayStringAt(17, 17, (uint8_t *)"Select wave", LEFT_MODE);
  
  // 18 waves available sofar
  // int8_t sid = 0;
//   for (int row=0; row < 3; row++) {
//     for (int col=0; col < 6; col++) {
//       drawSSample(sid, 20 + col * 70 , 40 + 75 * row );
//       sid++;
//     }
//   }
}


// Interrupt handler shared between:
// SD_DETECT pin, USER_KEY button and touch screen interrupt
void EXTI15_10_IRQHandler(void) {
  if (__HAL_GPIO_EXTI_GET_IT(SD_DETECT_PIN) != RESET) {
    HAL_GPIO_EXTI_IRQHandler(SD_DETECT_PIN | TS_INT_PIN);
  } else {
    // User button event or Touch screen interrupt
    HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
  }
}

// void HAL_GPIO_EXTI_IRQHandler points to this:
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		
	BSP_TS_GetState(&rawTouchState);
	while (rawTouchState.touchDetected) {
    
    if (runOnce == 0) {
      runOnce = 1;
      
      uint16_t touchx = rawTouchState.touchX[0];
      uint16_t touchy = rawTouchState.touchY[0];
      
      if (strcmp(touchMap,"main")==0) {
              // voice 1
              // 10,50,140,110
              if(touchx > 10 && touchx < 150 && touchy > 50 && touchy < 160) {
                //currentVoiceInEdit = 1;
                //drawInterface();
                touchMap = "waveselect";
                drawWaveSelector();
              }
            }
        
            else if (strcmp(touchMap,"waveselect")==0) {
              
              int8_t wavetableID = 0;
              for (int row=0; row < 3; row++) {
                for (int col=1; col < 6; col++) {
                  if(touchx > ((col*70)+5) && touchx < ((col*70)+5+50) && touchy > (45+(row*70)) && touchy < (45+50+(row*70))) {
                    // touch event on wavetable, open this one
                    openSCwaveform(0, wavetableID+1);
                    // and copy contents to wavetable ftbl
                    sp_gen_copy(sp,ft[0],f_wavetable[0]);
                  
                    touchMap = "main";
                    drawVoiceInterface(1);
                    break;
                  } 
                  wavetableID++;
                } 
              }
            }
    }
	  // read state and continue with while
	  BSP_TS_GetState(&rawTouchState);
	} // end while
  // reset touchloop for next time
  runOnce=0;
}
                    // voice 1
                    // 10,50,140,110
                                  //if(touchx > 0 && touchx < 300 && touchy > 0 && touchy < 160) {
                      //currentVoiceInEdit = 1;
                      //drawInterface();
                      //touchMap = "waveselect";
                      //process_audio = 0;
                      //__disable_irq();
                      
                                  // openSCwaveform(0, 3);
 //                                 sp_gen_copy(sp,ft[0],f_wavetable[0]);
                      
                      //sf_readf_float(audioBufferFile[0], ft[0]->tbl, ft[0]->size);
                      
                      //memset(ft[0]->tbl,0,600);
                      //memset(int_bufProcessedOut, 0, sizeof int_bufProcessedOut);
                      
                      // sp_osc_destroy(&osc);
                      // sp_osc_create(&osc);
                      //
                      // sp_osc_init(sp, osc, ft[0], 0);
                      // osc->freq = 200;
                      // osc->amp = 0.5;
                      
                      // sp_osc_create(&osc);
                      // sp_osc_create(&osc2);
                      //
                      // sp_osc_init(sp, osc, ft[0], 0);
                      // osc->freq = 200;
                      // osc->amp = 0.5;
                      
                      //  process_audio=0;
                      // for(int i = 0; i < 600; i++){
                      //   work->tbl[i] = f_wavetable[0][i];
                      // }
                      //                        process_audio=1;
                      //__enable_irq();
                      // sp_osc_init(sp, osc, ft[0], 0);
                      // osc->freq = 200;
                      // osc->amp = 0.5;
                      //process_audio = 1;

                      

		// run once on (continues) touch
//	  if (runOnce == 0) {
//
      

  //    runOnce = 1;
//
//       uint16_t touchx = rawTouchState.touchX[0];
//       uint16_t touchy = rawTouchState.touchY[0];
//
//       // determine and handle current touchMap
//       if (strcmp(touchMap,"main")==0) {
//         // voice 1
//         // 10,50,140,110
//         if(touchx > 10 && touchx < 150 && touchy > 50 && touchy < 160) {
//           //currentVoiceInEdit = 1;
//           //drawInterface();
//           touchMap = "waveselect";
//           drawWaveSelector();
//         }
//       }
//
//       if (strcmp(touchMap,"waveselect")==0) {
//         // voice 1
//         // 10,50,140,110
//         if(touchx > 0 && touchx < 300 && touchy > 0 && touchy < 160) {
//           //currentVoiceInEdit = 1;
//           //drawInterface();
//           //touchMap = "waveselect";
//           //process_audio = 0;
//           openSCwaveform(0, 2);
//
//           // for(int i = 0; i < ft[0]->size; i++){
// //              ft[0]->tbl[i] = f_wavetable[0][i];
// //           }
//           // sp_osc_init(sp, osc, ft[0], 0);
//           // osc->freq = 200;
//           // osc->amp = 0.5;
//           //process_audio = 1;
//
//           touchMap = "main";
//           drawVoiceInterface(1);
//         }
//       }
//
//     }
