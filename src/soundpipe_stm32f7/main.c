/*

soundpipe on stm32f7 discovery

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

/* SOUNDPIPE */

static sp_data *sp;
static sp_blsaw *blsaw;
static sp_blsaw *blsaw2;

#define VOLUME 90
#define SAMPLE_RATE 48000
#define AUDIO_DMA_BUFFER_SIZE 4096
#define AUDIO_DMA_BUFFER_SIZE2 (AUDIO_DMA_BUFFER_SIZE >> 1)

// audio buffers
static int16_t int_bufProcessedOut[AUDIO_DMA_BUFFER_SIZE];
static uint8_t audioOutBuf[AUDIO_DMA_BUFFER_SIZE];

extern SAI_HandleTypeDef haudio_out_sai;

// header
void initAudio();
void computeAudio();
void drawInterface();
void ConfigureADC();
void ConfigureDMA();

uint32_t g_ADCValue;
int g_MeasurementNumber;
ADC_HandleTypeDef g_AdcHandle;
DMA_HandleTypeDef  g_DmaHandle;
enum{ ADC_BUFFER_LENGTH = 8192 };
uint32_t g_ADCBuffer[ADC_BUFFER_LENGTH];

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

  initAudio();
  
  drawInterface();
  
  sp_create(&sp);
  sp->sr = 48000;

  sp_blsaw_create(&blsaw);
  sp_blsaw_create(&blsaw2);

  sp_blsaw_init(sp, blsaw);
  *blsaw->freq = 440;
  *blsaw->amp = 0.5f;
  
  sp_blsaw_init(sp, blsaw2);
  *blsaw2->freq = 400;
  *blsaw2->amp = 0.5f;
  

  while (1) {		
    //if (HAL_ADC_PollForConversion(&g_AdcHandle, 1000000) == HAL_OK)
    //        {
        //        g_ADCValue = HAL_ADC_GetValue(&g_AdcHandle);
      //          g_MeasurementNumber++;
            		char a[] = "";
            		sprintf(a, "%ld", g_ADCValue);
                BSP_LCD_DisplayStringAt(50, 50, (uint8_t *)a, LEFT_MODE);
    //        }
  }

  return 0;
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
 
    // adcChannel.Channel = ADC_CHANNEL_8;
    // adcChannel.Rank = 1;
    // adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    // adcChannel.Offset = 0;
    //
    // if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel) != HAL_OK)
    // {
    //     //asm("bkpt 255");
    //   Error_Handler();
    // }
    
    adcChannel.Channel = ADC_CHANNEL_0;
    adcChannel.Rank = 1;
    adcChannel.SamplingTime = ADC_SAMPLETIME_28CYCLES;
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
      g_ADCValue = g_ADCBuffer[2004];//std::accumulate(g_ADCBuffer, g_ADCBuffer + ADC_BUFFER_LENGTH, 0) / ADC_BUFFER_LENGTH;
        g_MeasurementNumber += ADC_BUFFER_LENGTH;
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
    SPFLOAT tmp = 0;
    SPFLOAT tmp2 = 0;
    sp_blsaw_compute(sp, blsaw, NULL, &tmp);
    sp_blsaw_compute(sp, blsaw2, NULL, &tmp2);
    
    SPFLOAT mixOut = (0.5f * tmp + 0.5f * tmp2);

    // channel outputs in 2's comp / signed int
    int_bufProcessedOut[i] = (mixOut * 32767);
    int_bufProcessedOut[i+1] = (mixOut * 32767);
  }
}

void drawInterface() {
  BSP_LCD_Clear(LCD_COLOR_DARKMAGENTA);

  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(5, 5, (uint8_t *)"stm32f7 and soundpipe", LEFT_MODE);

}