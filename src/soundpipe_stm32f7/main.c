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

int main() {
  CPU_CACHE_Enable();
  HAL_Init();
  SystemClock_Config(); 
  
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

  sp_blsaw_init(sp, blsaw);
  *blsaw->freq = 440;
  *blsaw->amp = 0.5f;

  while (1) {		
	  //nothing is done here	
  }

  return 0;
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
    sp_blsaw_compute(sp, blsaw, NULL, &tmp);

    // channel outputs in 2's comp / signed int
    int_bufProcessedOut[i] = (tmp * 32767);
    int_bufProcessedOut[i+1] = (tmp * 32767);
  }
}

void drawInterface() {
  BSP_LCD_Clear(LCD_COLOR_DARKMAGENTA);

  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(5, 5, (uint8_t *)"stm32f7 and soundpipe", LEFT_MODE);

}