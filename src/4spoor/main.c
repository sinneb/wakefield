/*

4spoor mini-sampler
===================

A sampler / sampleplayer capable of:

- Realtime stretching of 4 tracks of streaming audio from the SD-card based on global tempo
- Tap-tempo for global tempo

Technical operation:
4 wav files are opened simultaneously from the SD
Based on the global tempo, a number of byte are read from each file
These bytes are interpolated to the samplerate
The streams are mixed and played

Sample playback is started per sample through a button on the LCD

*/

#include <stdio.h>
#include <stdlib.h>

#include "stm32746g_discovery.h"
#include "stm32746g_discovery_audio.h"
#include "stm32746g_discovery_sd.h"
#include "stm32f7xx_hal.h"

#include "ff.h"
#include "ff_gen_drv.h"
#include "usbh_diskio.h"
#include "sd_diskio.h"

#include "ct-gui/gui_stm32.h"
#include "ct-head/random.h"
#include "common/clockconfig.h"

#include "wavfile.h"

uint8_t currentLCDcolor = 0;
const uint32_t LCDColorarray[] = { LCD_COLOR_YELLOW, LCD_COLOR_GREEN, LCD_COLOR_ORANGE, LCD_COLOR_MAGENTA };

#define VOLUME 60
#define SAMPLE_RATE 44100
#define AUDIO_DMA_BUFFER_SIZE 4096
#define AUDIO_DMA_BUFFER_SIZE2 (AUDIO_DMA_BUFFER_SIZE >> 1)
#define AUDIO_DMA_BUFFER_SIZE4 (AUDIO_DMA_BUFFER_SIZE >> 2)
#define AUDIO_DMA_BUFFER_SIZE8 (AUDIO_DMA_BUFFER_SIZE >> 3)

char *audioFilename[4] = {"0:sound.wav", "0:sound2.wav", "0:sound3.wav", "0:sound4.wav"};

// audio buffers
static uint8_t int_bufProcessedOut[AUDIO_DMA_BUFFER_SIZE];
static uint8_t audioOutBuf[AUDIO_DMA_BUFFER_SIZE];
static uint8_t audioBufferFile[4][AUDIO_DMA_BUFFER_SIZE];
float f_bufPre_left[4][1024];
float f_bufPost_left[4][1024];
float f_bufPre_right[4][1024];
float f_bufPost_right[4][1024];

uint16_t taptempo[4];
uint8_t taptempocounter;
uint32_t taptempo_prev = 0;
float global_tempo = 100;

uint32_t prevTick = 0;

float f_bufPost_mixdown_left[1024];
float f_bufPost_mixdown_right[1024];

short resleft[512];
short resright[512];

extern HCD_HandleTypeDef hhcd;
extern USBH_HandleTypeDef hUSBH;
extern SAI_HandleTypeDef haudio_out_sai;

FIL audio_file[4];
UINT bytes_read[4];
UINT bytes_read2;
UINT bytes_read3;
UINT bytes_read4;
UINT prev_bytes_read;
FATFS fs;
FATFS SDFatFs;

static char usb_drive_path[4];

static TS_StateTypeDef rawTouchState;

uint16_t runOnce = 0;

float getBPM(FIL *audio_file);
static void init_after_USB();
void drawInterface();
static void initAudio();
static void stop_playback();
void computeAudio();
void inter1parray( float aaaa[], int n, float bbbb[], int m );
void interp2array( float aaaa[], int n, float bbbb[], int m );
void interp5( float aaaa[], int n, float bbbbb[], int m );

int main() {
  CPU_CACHE_Enable();
  HAL_Init();
  SystemClock_Config(); 
  
  BSP_LCD_Init();
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Off(LED_GREEN);
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  
  // Init LCD and Touchscreen
  if (BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize()) == TS_OK) {
    BSP_LCD_LayerDefaultInit(1, SDRAM_DEVICE_ADDR);
    BSP_LCD_SelectLayer(1);
	  BSP_TS_ITConfig();
  }  

  // Load SD Driver & mount card
  if (FATFS_LinkDriver(&SD_Driver, usb_drive_path) != 0) {
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)"SD Driver error", CENTER_MODE);
    Error_Handler();
  }
	if(f_mount(&SDFatFs, (TCHAR const*)usb_drive_path, 0) != FR_OK) {
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)"SD Mount error", CENTER_MODE);
		Error_Handler();
	}
		
  init_after_USB();
  
  drawInterface();
  
  while (1) {		
	  //nothing is done here	
  }

  return 0;
}

static void init_after_USB() {
  // open samples
  for (int j=0; j < 4; j++) {
    if (f_open(&audio_file[j], audioFilename[j], FA_READ) == FR_OK) {
		// f_open ok
      f_lseek(&audio_file[j], f_size(&audio_file[j]));
    } else {
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                                (uint8_t *)"Load error", CENTER_MODE);
      Error_Handler();
    }
  }
	
	float bpm = getBPM(&audio_file[1]);
	char a[] = "";
	snprintf(a, 6, "%f",bpm);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)a, CENTER_MODE);
  
  initAudio();
}

float getBPM(FIL *audio_file) {
	uint32_t filesz = f_size(&audio_file[0]);
	uint8_t nbrbeats = 8;
	// filesize / 44100 (samplerate) / 2 (channels) / 2 (16 bits = 2 bytes per channel)
	// this gives playtime in seconds
	// then divide by nbrofbeats to get time per beat in seconds
	// then divide 60 through the time per beat -> BPM
	float bpm = 60 / (((float)filesz / 44100 / 2 / 2) / nbrbeats); 
	return bpm;
}

static void initAudio() {
    if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, VOLUME, SAMPLE_RATE) != 0) {
      Error_Handler();
    }
    BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
    BSP_AUDIO_OUT_SetVolume(VOLUME);
    BSP_AUDIO_OUT_Play((uint16_t *)audioOutBuf, AUDIO_DMA_BUFFER_SIZE);
}

static void stop_playback() {
  BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW);
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
	
	
	if(bytes_read[0]<2048) {
		//memset(audioOutBuf, 0, sizeof audioOutBuf);
		memset(audioBufferFile[0], 0, sizeof audioBufferFile[0]);
		//memset(audioBufferFile[1], 0, sizeof audioBufferFile[1]);
	}
	if(bytes_read[1]<2048) {
		//memset(audioOutBuf, 0, sizeof audioOutBuf);
		//memset(audioBufferFile[0], 0, sizeof audioBufferFile[0]);
		memset(audioBufferFile[1], 0, sizeof audioBufferFile[1]);
	}
	if(bytes_read[2]<2048) {
		//memset(audioOutBuf, 0, sizeof audioOutBuf);
		memset(audioBufferFile[2], 0, sizeof audioBufferFile[2]);
		//memset(audioBufferFile[1], 0, sizeof audioBufferFile[1]);
	}
	if(bytes_read[3]<2048) {
		//memset(audioOutBuf, 0, sizeof audioOutBuf);
		//memset(audioBufferFile[0], 0, sizeof audioBufferFile[0]);
		memset(audioBufferFile[3], 0, sizeof audioBufferFile[3]);
	}
}

void computeAudio() {
	
	float rate = global_tempo / 100;
	if (rate > 2) rate = 2;
	short bufsize = (short)(2048 * rate);
	// garantue a multiple of 4
	bufsize -= bufsize%4;
  
  // for each sample
  for (int i=0; i < 4; i++) {
	
    // clear global buffers
    memset(audioBufferFile[i], 0, sizeof audioBufferFile[i]);
  
  	// read chunks from USB
  	f_read(&audio_file[i], audioBufferFile[i], bufsize, &bytes_read[i]);
	
  	// 2's-complement signed integers -> short (-32k -> +32k) -> float (-1 -> +1)
  	for (int j=0; j < bufsize; j=j+4) {
		
  		// to short
  		short lefty = (short)((audioBufferFile[i][j+1])<<8 | ((audioBufferFile[i][j]) & 0xFF));
  		short righty = (short)((audioBufferFile[i][j+3])<<8 | ((audioBufferFile[i][j+2]) & 0xFF));
		
  		// to float
  		f_bufPre_left[i][j/4] = ((float)lefty/32768);
  		f_bufPre_right[i][j/4] = ((float)righty/32768);
    }
  
    // interpolation
  	inter1parray( f_bufPre_left[i], bufsize / 4, f_bufPost_left[i], 512 );
  	inter1parray( f_bufPre_right[i], bufsize / 4, f_bufPost_right[i], 512 );
  }


	
	// the mix
	// float to short
	for (int k=0; k<2048; k=k+4) {
    
    // mix
    f_bufPost_mixdown_left[k/4] = (0.8 * f_bufPost_left[0][k/4])
                          + (0.8 * f_bufPost_left[1][k/4])
                          + (0.8 * f_bufPost_left[2][k/4])
                          + (0.8 * f_bufPost_left[3][k/4]);
    f_bufPost_mixdown_right[k/4] = (0.8 * f_bufPost_right[0][k/4])
                          + (0.8 * f_bufPost_right[1][k/4])
                          + (0.8 * f_bufPost_right[2][k/4])
                          + (0.8 * f_bufPost_right[3][k/4]);
    
		// to short
		resleft[k/4] = (short)(f_bufPost_mixdown_left[k/4] * 32768);
		resright[k/4] = (short)(f_bufPost_mixdown_right[k/4] * 32768);
		// to 2's-comp
		int_bufProcessedOut[k+1] = resleft[k/4]>>8;
		int_bufProcessedOut[k] = resleft[k/4]&0xff;
		int_bufProcessedOut[k+3] = resright[k/4]>>8;
		int_bufProcessedOut[k+2] = resright[k/4]&0xff;
		
	}
}

void OTG_FS_IRQHandler(void) {
  HAL_HCD_IRQHandler(&hhcd);
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

void drawButton(uint16_t x, uint16_t y, uint8_t * textstring) {
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillRect(x,y,110,100);
  BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
  BSP_LCD_FillRect(x+5,y+5,100,90);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
	BSP_LCD_DisplayStringAt(x+45, y+40, textstring, LEFT_MODE);
}

void drawInterface() {
  BSP_LCD_Clear(LCD_COLOR_BLUE);
    
  drawButton(10,5,(uint8_t *)"1");
  drawButton(125,5,(uint8_t *)"2");
  drawButton(240,5,(uint8_t *)"3");
  drawButton(355,5,(uint8_t *)"4");
}

// void HAL_GPIO_EXTI_IRQHandler points to this:
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// runonce to run this function only once when IRQ fires (fires continuesly)
	runOnce = 0;
  
  //drawInterface();
		
	BSP_TS_GetState(&rawTouchState);
	while (rawTouchState.touchDetected) {
		// run once on (continues) touch
		if (runOnce == 0) {
			
			uint16_t touchx = rawTouchState.touchX[0];
      uint16_t touchy = rawTouchState.touchY[0];
			
			if(touchy > 115) {
			
				int16_t sum = 0;
				for(int i=0; i<4; i++) {
				    sum+=taptempo[i];
				}
				global_tempo = (float)60000 / ((float)sum/4);
			
				uint32_t tikker = HAL_GetTick();
				taptempo[taptempocounter] = tikker - taptempo_prev;
				taptempocounter++;
				if(taptempocounter>4) taptempocounter=0;
				taptempo_prev = tikker;
			
				BSP_LCD_SetBackColor(LCDColorarray[currentLCDcolor]);
			
				currentLCDcolor++;
				if(currentLCDcolor>2) currentLCDcolor=0;			
			
				char a[] = "";
				//sprintf(a, "%ld", tikker - taptempo_prev);
				snprintf(a, 6, "%f",global_tempo);
				BSP_LCD_SetFont(&Font24);
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)a, CENTER_MODE);
			}
			
			else {
        // restart audio based on touch positions
        if(touchx > 0 && touchx < 120) {
          f_lseek(&audio_file[0], 0);
        }
        if(touchx > 120 && touchx < 240) {
          f_lseek(&audio_file[1], 0);
        }
        if(touchx > 240 && touchx < 360) {
          f_lseek(&audio_file[2], 0);
        }
        if(touchx > 360 && touchx < 480) {
          f_lseek(&audio_file[3], 0);
        }
        //} else {
        //  f_lseek(&audio_file[1], 0);
        //}

      }
			
			runOnce = 1;
		}
	// read state and continue with while
	BSP_TS_GetState(&rawTouchState);
	} // end while
}