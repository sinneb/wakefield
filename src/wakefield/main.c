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

#include "soundpipe.h"

// defines
#define VOLUME 90
#define SAMPLE_RATE 48000
#define AUDIO_DMA_BUFFER_SIZE 4096
#define AUDIO_DMA_BUFFER_SIZE2 (AUDIO_DMA_BUFFER_SIZE >> 1)

/* SOUNDPIPE */

static sp_data *sp;
static sp_ftbl *ft;
//static sp_posc3 *posc3[8];
static sp_osc *osc;
static sp_osc *osc2;

// FATFS: sdcard access 
#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
FIL audio_file[12];
FIL wavetable_file[12];
UINT bytes_read[12];
UINT prev_bytes_read;
FATFS fs;
FATFS SDFatFs;
static char usb_drive_path[4];

// datastructures for wavetables
float f_wavetable[12][600];
uint16_t vco1wave = 4;
uint16_t vco2wave = 2;
uint16_t vco3wave = 1;
uint16_t vco4wave = 2;
uint16_t vco5wave = 1;
uint16_t vco6wave = 2;
uint16_t vco7wave = 1;
uint16_t vco8wave = 2;
static uint8_t audioBufferFile[4][AUDIO_DMA_BUFFER_SIZE];

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

uint32_t g_ADCValue;
int g_MeasurementNumber;
ADC_HandleTypeDef g_AdcHandle;
DMA_HandleTypeDef  g_DmaHandle;
enum{ ADC_BUFFER_LENGTH = 8192 };
uint32_t g_ADCBuffer[ADC_BUFFER_LENGTH];

uint16_t ADCchannelValues[4];

uint8_t audioReady = 0;

int main() {
  CPU_CACHE_Enable();
  HAL_Init();
  SystemClock_Config(); 
  ConfigureADC();
  ConfigureDMA();
  HAL_ADC_Start_DMA(&g_AdcHandle, g_ADCBuffer, ADC_BUFFER_LENGTH);
  
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Off(LED_GREEN);

  // Init LCD and Touchscreen
  BSP_LCD_Init();
  if (BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize()) == TS_OK) {
    BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	  BSP_TS_ITConfig();
  }  
  
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    
  // Load SD Driver & mount card
  if (FATFS_LinkDriver(&SD_Driver, usb_drive_path) != 0) {
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)"SD Driver error", CENTER_MODE);
    Error_Handler();
  }
	if(f_mount(&SDFatFs, (TCHAR const*)usb_drive_path, 0) != FR_OK) {
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)"SD Mount error", CENTER_MODE);
		Error_Handler();
	}

  initAudio();
  drawVoiceInterface(1);
  
  sp_create(&sp);
  sp->sr = 48000;
  
  sp_ftbl_create(sp, &ft, 600);

  sp_osc_create(&osc);
  sp_osc_create(&osc2);
  
  sp_osc_init(sp, osc, ft, 0);
  osc->freq = 200;
  osc->amp = 0.5;
  
  sp_osc_init(sp, osc2, ft, 0);
  osc2->freq = 250;
  osc2->amp = 0.3;
  
  while (1) {		
    // determine audioReady moment
    // reserve 700ms for initialisation
    if(audioReady==0 && HAL_GetTick()>700) {
      audioReady=1;
      
      openSCwaveform(0, vco1wave);
      BSP_LCD_DisplayStringAt(50, 50, (uint8_t *)"Loaded", LEFT_MODE);
  
      for(int i = 0; i < ft->size; i++){
          ft->tbl[i] = f_wavetable[0][i];
      }
    }
    
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
  //__disable_irq();
  f_close(&wavetable_file[SCwaveformID]);
  uint16_t openresult = f_open(&wavetable_file[SCwaveformID], theFilename, FA_READ);
  //__enable_irq();
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

  // 2's-complement signed integers -> short (-32k -> +32k) -> float (-1 -> +1)
  for (int j=0; j < 1200; j=j+2) {

    // to short
    // read 2 values for 2's-complement
    // convert to 1 short value
    short tempshort = (short)((audioBufferFile[0][j+1])<<8 | ((audioBufferFile[0][j]) & 0xFF));

    // to float
    // use j/2 because of conversion from 2's comp to short
    f_wavetable[SCwaveformID][j/2] = ((float)tempshort/32768);
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
    SPFLOAT tmp = 0, tmp2=0, tmp3=0, tmp4=0, tmp5=0, tmp6=0;
    
    sp_osc_compute(sp, osc, NULL, &tmp);
    sp_osc_compute(sp, osc2, NULL, &tmp3);
    
    SPFLOAT mixOut = (0 * tmp3 + 0.5 * tmp);

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
//
//     BSP_LCD_SetTextColor(LCD_COLOR_DARKMAGENTA);
//     BSP_LCD_FillRect(0,0,412,24);
//
//     BSP_LCD_SetFont(&Font16);
//     BSP_LCD_SetBackColor(LCD_COLOR_DARKMAGENTA);
//     BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
//     BSP_LCD_DisplayStringAt(5, 5, (uint8_t *)"voice 1", LEFT_MODE);
  }
}