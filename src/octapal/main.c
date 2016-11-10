/*

octapal
===================

a midi controlled soundmodule

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

#define TIMx TIM3
#define TIMx_CLK_ENABLE() __HAL_RCC_TIM3_CLK_ENABLE()
#define TIMx_IRQn TIM3_IRQn
#define TIMx_IRQHandler TIM3_IRQHandler

TIM_HandleTypeDef timer;
static uint8_t isBlinking = 1;
uint8_t currentStep = 1;

static void initTimer(uint16_t period);

uint8_t currentLCDcolor = 0;
const uint32_t LCDColorarray[] = { LCD_COLOR_YELLOW, LCD_COLOR_GREEN, LCD_COLOR_ORANGE, LCD_COLOR_MAGENTA };

#define VOLUME 60
#define SAMPLE_RATE 96000
#define AUDIO_DMA_BUFFER_SIZE 4096
#define AUDIO_DMA_BUFFER_SIZE2 (AUDIO_DMA_BUFFER_SIZE >> 1)
#define AUDIO_DMA_BUFFER_SIZE4 (AUDIO_DMA_BUFFER_SIZE >> 2)
#define AUDIO_DMA_BUFFER_SIZE8 (AUDIO_DMA_BUFFER_SIZE >> 3)

char *audioFilename[4] = {"0:sound.wav", "0:sound2.wav", "0:sound3.wav", "0:sound4.wav"};

// waves
uint8_t sswave[100][50] = {
  {24, 23, 22, 21, 16, 10, 9, 11, 13, 15, 15, 12, 9, 7, 6, 6, 7, 8, 9, 11, 13, 15, 17, 19, 21, 22, 24, 26, 27, 29, 33, 37, 40, 42, 44, 45, 46, 46, 45, 44, 42, 40, 38, 36, 34, 33, 31, 29, 27, 26},
  {26, 43, 44, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 23, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4},
  {24, 7, 6, 7, 8, 8, 9, 10, 11, 12, 12, 13, 14, 15, 16, 16, 17, 18, 19, 20, 20, 21, 22, 23, 24, 24, 25, 26, 27, 28, 28, 29, 30, 31, 32, 32, 33, 34, 35, 36, 36, 37, 38, 39, 40, 40, 41, 42, 43, 44},
  {25, 27, 30, 32, 35, 37, 39, 40, 42, 43, 43, 43, 43, 43, 42, 40, 39, 37, 35, 33, 31, 28, 26, 24, 22, 20, 18, 16, 15, 14, 13, 13, 12, 12, 13, 14, 10, 9, 10, 10, 10, 10, 11, 12, 13, 14, 16, 18, 20, 22},
  {25, 27, 30, 32, 35, 37, 39, 41, 42, 43, 44, 45, 45, 45, 44, 44, 42, 41, 39, 37, 35, 33, 30, 27, 25, 22, 20, 17, 15, 13, 11, 10, 9, 8, 8, 7, 8, 6, 5, 6, 7, 8, 8, 10, 11, 13, 15, 17, 20, 22},
  {25, 28, 31, 34, 37, 40, 43, 45, 47, 48, 49, 49, 49, 48, 47, 45, 43, 40, 37, 34, 31, 27, 24, 21, 18, 16, 14, 12, 10, 9, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 10, 10, 10, 11, 12, 13, 15, 17, 19, 22},
  {24, 22, 19, 16, 14, 11, 10, 8, 7, 6, 6, 6, 6, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 19, 22, 24, 27, 30, 32, 35, 37, 39, 40, 41, 42, 43, 43, 46, 45, 45, 44, 44, 43, 41, 40, 38, 35, 33, 30, 27},
  {25, 28, 31, 34, 37, 39, 42, 44, 46, 47, 48, 49, 49, 49, 49, 48, 47, 45, 43, 41, 38, 36, 33, 30, 28, 24, 21, 18, 15, 12, 10, 7, 5, 3, 2, 1, 0, 0, 0, 0, 1, 2, 4, 6, 8, 11, 13, 16, 19, 21},
  {25, 27, 29, 32, 34, 36, 38, 39, 41, 42, 43, 44, 45, 45, 46, 45, 45, 43, 42, 40, 37, 35, 32, 30, 27, 24, 22, 19, 17, 15, 13, 11, 10, 8, 7, 6, 5, 4, 4, 3, 4, 4, 6, 7, 9, 12, 14, 17, 19, 22},
  {25, 27, 27, 27, 27, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 26, 26, 26, 27, 33, 24, 22, 22, 22, 22, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 22, 22, 22, 22, 22, 23, 23, 23, 22, 16},
  {24, 6, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 43, 48, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49},
  {27, 44, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 22, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {25, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 44, 45, 46, 17, 6, 4, 5, 6, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 16, 17, 18, 19, 20, 21, 22, 23, 24},
  {24, 15, 12, 12, 14, 16, 16, 16, 17, 20, 23, 25, 17, 8, 4, 3, 5, 7, 8, 7, 8, 10, 13, 16, 20, 23, 27, 30, 34, 37, 40, 40, 40, 41, 44, 47, 49, 41, 32, 28, 27, 29, 32, 32, 32, 32, 34, 37, 41, 36},
  {24, 18, 12, 9, 13, 17, 19, 17, 12, 8, 7, 9, 9, 8, 2, 4, 10, 18, 33, 34, 33, 29, 24, 27, 28, 26, 22, 20, 21, 26, 33, 37, 36, 33, 29, 29, 31, 32, 30, 27, 27, 30, 35, 49, 48, 44, 38, 33, 31, 29},
  {25, 36, 42, 42, 37, 38, 41, 39, 25, 15, 10, 19, 33, 40, 44, 46, 46, 46, 47, 47, 47, 46, 45, 46, 47, 48, 49, 49, 49, 48, 48, 47, 36, 20, 11, 6, 3, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 7},
  {25, 31, 37, 42, 45, 48, 48, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 48, 47, 46, 45, 42, 37, 31, 24, 18, 12, 7, 4, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 3, 4, 7, 12, 18},
  {25, 33, 38, 41, 44, 46, 47, 48, 49, 49, 49, 49, 49, 49, 49, 48, 48, 47, 46, 46, 45, 44, 44, 43, 43, 42, 42, 42, 41, 41, 40, 40, 36, 20, 10, 6, 5, 4, 4, 4, 4, 5, 5, 6, 7, 8, 9, 11, 13, 16}
};

char *wavenames[18] = {"0101","0102","0103","0104","0105","0106","0107","0108","0109","0110","0111","0112","0113","0114","0115","0116","0117","0118"};

// audio buffers
static uint8_t int_bufProcessedOut[AUDIO_DMA_BUFFER_SIZE];
static uint8_t audioOutBuf[AUDIO_DMA_BUFFER_SIZE];
static uint8_t audioBufferFile[4][AUDIO_DMA_BUFFER_SIZE];
//float f_bufPre_left[4][1024];
//float f_bufPost_left[4][1024];
//float f_bufPre_right[4][1024];
//float f_bufPost_right[4][1024];

//uint16_t taptempo[4];
//uint8_t taptempocounter;
//uint32_t taptempo_prev = 0;
//float global_tempo = 100;

uint32_t prevTick = 0;

uint16_t *varToUpdate;
uint16_t waveformToUpdate;

float f_bufPost_mixdown_left[1024];
float f_bufPost_mixdown_right[1024];

float voice_frequency[4] = {0,0,0,0};

int8_t sequence[16] = {0,0,0,0,0,-1,0,-2,0,1,3,-3,2,0,4,5};
uint8_t current_midi_note = 50;

float f_ssample[4][600];
uint8_t active_ssample = 1;
uint8_t receiving_note_on = 0;
// max 32 cycle = 32*600 = 19200
float f_ssample_freq_specific[19200];
float f_ssample_outChannel[4][600];
float f_ssample_outChannel_Volume[4] = {0.3, 0.3, 0.3, 0.3};
float f_ssample_temp[600];
float f_ssample_right[600];
uint16_t ssample_lastpos = 0;
uint16_t ssample_small_lastpos[4] = {0,0,0,0};
uint16_t ssample_samplespertransfer[4] = {300,350,400,450};
int16_t ssamples_remaining_idx[4] = {600,600,600,600};

short resleft[512];
short resright[512];

extern HCD_HandleTypeDef hhcd;
extern USBH_HandleTypeDef hUSBH;
extern SAI_HandleTypeDef haudio_out_sai;

UART_HandleTypeDef uart_config;

uint8_t midicounter=0;
uint16_t mididata[10];
uint8_t rx_byte[1];

uint16_t vco1wave = 0;
uint16_t vco2wave = 1;
uint16_t vco3wave = 2;

FIL audio_file[4];
FIL ssample[4];
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

char *touchMap = "main";

void drawSSample(uint16_t sampleID, uint16_t xstart, uint16_t ystart);
void UART6_Config();
static void init_after_USB();
void drawInterface();
void drawWaveSelector();
static void initAudio();
void computeAudio();
void computeVoice(float freq, uint8_t voiceID);
void computeOscillatorOut(uint8_t ssampleBufferID, uint8_t channelID, uint16_t samplesPerTransfer);
void inter1parray( float aaaa[], int n, float bbbb[], int m );
void interp2array( float aaaa[], int n, float bbbb[], int m );
void interp5( float aaaa[], int n, float bbbbb[], int m );
void openSCwaveform(uint16_t SCwaveformID, uint16_t filenameID);
void drawStepSeqTopBar(uint16_t active);
void drawSequencer();

int main() {
  CPU_CACHE_Enable();
  HAL_Init();
  SystemClock_Config(); 
  
  // config UART for MIDI communication
  UART6_Config();
  
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Off(LED_GREEN);
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  
  initTimer(5000);
  
  // Init LCD and Touchscreen
  BSP_LCD_Init();
  if (BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize()) == TS_OK) {
    BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
    // BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+(BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4));
    // BSP_LCD_SelectLayer(1);
    // BSP_LCD_Clear(LCD_COLOR_TRANSPARENT);
    // BSP_LCD_SetTransparency(1, 100);
    // BSP_LCD_SelectLayer(0);
    // BSP_LCD_Clear(LCD_COLOR_BLACK);
    // BSP_LCD_SetLayerVisible(0,ENABLE);
    // BSP_LCD_SetLayerVisible(1,DISABLE);
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
  
  voice_frequency[0] = 440;
  
  while (1) {		
	  //nothing is done here	
  }

  return 0;
}

// MIDI HANDLING --------------------------------------------

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  BSP_LED_Toggle(LED_GREEN);

  //BSP_LCD_SetFont(&Font12);
  //BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
  //                        (uint8_t *)"MIDI in", CENTER_MODE);
    
  if(midicounter==2) {
    mididata[midicounter]=rx_byte[0];
    char a[] = "";
    sprintf(a, "%d %d %d", mididata[0],mididata[1],mididata[2]);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 20, (uint8_t *)a, CENTER_MODE);
    midicounter=0;
    
    if(mididata[0] == 144) {
      float req_freq = (13.75 * (pow(2,(mididata[1]-9.0)/12.0)));
      char b[10];
      int dingus = req_freq * 1000;
      //sprintf(b, "%f",req_freq);
      //sprintf(b, "bla%g", req_freq );
      snprintf(b, 10, "%d",dingus);
       BSP_LCD_SetFont(&Font24);
       BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
       BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)"         ", CENTER_MODE);
       BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)b, CENTER_MODE);
     
       voice_frequency[0] = req_freq;
     };
     
     if(mididata[0] == 176) {
       if(mididata[1]==71) {
         f_ssample_outChannel_Volume[0] = mididata[2]/127.0;
       }
       if(mididata[1]==72) {
         f_ssample_outChannel_Volume[1] = mididata[2]/127.0;
       }
       if(mididata[1]==73) {
         f_ssample_outChannel_Volume[2] = mididata[2]/127.0;
       }
     }
  }
  
  if(midicounter==1) {
    mididata[midicounter]=rx_byte[0];
    midicounter++;
  }
                            
  // received 144: start receiving note_on (3 bytes)
  // e.g. 144 56 40
  if((rx_byte[0]==144 || rx_byte[0]==176)  && midicounter==0) {
    //receiving_note_on = 1;
    mididata[midicounter]=rx_byte[0];
    midicounter++;
  }  

  // reset interrupt
  HAL_UART_Receive_IT(&uart_config, rx_byte, 1);
}



static void init_after_USB() {
  
  openSCwaveform(0, 2);
  openSCwaveform(1, 3);
  openSCwaveform(2, 3);
  
  initAudio();
}

static void initAudio() {
    if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, VOLUME, SAMPLE_RATE) != 0) {
      Error_Handler();
    }
    BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
    BSP_AUDIO_OUT_SetVolume(VOLUME);
    BSP_AUDIO_OUT_Play((uint16_t *)audioOutBuf, AUDIO_DMA_BUFFER_SIZE);
}

void openSCwaveform(uint16_t SCwaveformID, uint16_t filenameID) {
  // open single waveform
  // skip 44 (offset)
  // read 1200 (samplelength = 600 2's complement)
  //char* theFilename = printf("0:single%d.wav", filenameID);
  char theFilename[] = "";
  sprintf(theFilename, "0:AKWF_01%02d.wav", filenameID);
  f_close(&ssample[SCwaveformID]);
  uint16_t openresult = f_open(&ssample[SCwaveformID], theFilename, FA_READ);
  if ( openresult == FR_OK) {
	// f_open ok
    f_lseek(&ssample[SCwaveformID], 44);
		//char a[] = "";
		//sprintf(a, "%ld", f_size(&ssample));
    //BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)a, CENTER_MODE);
  } else {
		char a[] = "";
		sprintf(a, "%d", openresult);
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                              (uint8_t *)a, CENTER_MODE);
    Error_Handler();
  }

	// read chunks from USB
  // single use of audiobuffer 0, will be reused in context later on
  // single use of bytes_read 0
	f_read(&ssample[SCwaveformID], audioBufferFile[0], 1200, &bytes_read[0]);

	// 2's-complement signed integers -> short (-32k -> +32k) -> float (-1 -> +1)
	for (int j=0; j < 1200; j=j+2) {

		// to short
    // read 2 values for 2's-complement
    // convert to 1 short value
		short tempshort = (short)((audioBufferFile[0][j+1])<<8 | ((audioBufferFile[0][j]) & 0xFF));

		// to float
    // use j/2 because of conversion from 2's comp to short
		f_ssample[SCwaveformID][j/2] = ((float)tempshort/32768);
  }
}

void AUDIO_OUT_SAIx_DMAx_IRQHandler(void) {
  HAL_DMA_IRQHandler(haudio_out_sai.hdmatx);
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
  // clear global buffer
  memset(int_bufProcessedOut, 0, sizeof int_bufProcessedOut);
//  uint32_t tikker = HAL_GetTick();
	computeAudio();
    // char a[] = "";
    // sprintf(a, "%ld", tikker);
    // BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)a, CENTER_MODE);
	memcpy(audioOutBuf, int_bufProcessedOut, AUDIO_DMA_BUFFER_SIZE2);  
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
  memset(int_bufProcessedOut, 0, sizeof int_bufProcessedOut);
	computeAudio();
	memcpy(&audioOutBuf[AUDIO_DMA_BUFFER_SIZE2], int_bufProcessedOut, AUDIO_DMA_BUFFER_SIZE2);
}

void computeAudio() {
  
  for(int k=0; k<4; k++){
    if (voice_frequency[k]!=0) {
      // play voice
      computeVoice(voice_frequency[k], k);
    }
  }
  //computeVoice(440);
  
	// the mix
	// float to short
	for (int k=0; k<2048; k=k+4) {
    
    // mix
    /*
    f_bufPost_mixdown_left[k/4] = (0.8 * f_bufPost_left[0][k/4])
                          + (0.8 * f_bufPost_left[1][k/4])
                          + (0.8 * f_bufPost_left[2][k/4])
                          + (0.8 * f_bufPost_left[3][k/4])
                            + (0.8 * f_ssample_outChannel[0][k/4]);
    f_bufPost_mixdown_right[k/4] = (0.8 * f_bufPost_right[0][k/4])
                          + (0.8 * f_bufPost_right[1][k/4])
                          + (0.8 * f_bufPost_right[2][k/4])
                          + (0.8 * f_bufPost_right[3][k/4]);
    */
    
    f_bufPost_mixdown_left[k/4] =   (f_ssample_outChannel_Volume[0] * f_ssample_outChannel[0][k/4])
                                    + (f_ssample_outChannel_Volume[1] * f_ssample_outChannel[1][k/4])
                                    + (f_ssample_outChannel_Volume[2] * f_ssample_outChannel[2][k/4])
                                    + (f_ssample_outChannel_Volume[3] * f_ssample_outChannel[3][k/4]);
    
    
    
		// to short
		resleft[k/4] = (short)(f_bufPost_mixdown_left[k/4] * 32768);
		//resright[k/4] = (short)(f_bufPost_mixdown_right[k/4] * 32768);
		resright[k/4] = resleft[k/4];
    // to 2's-comp
		int_bufProcessedOut[k+1] = resleft[k/4]>>8;
		int_bufProcessedOut[k] = resleft[k/4]&0xff;
		int_bufProcessedOut[k+3] = resright[k/4]>>8;
		int_bufProcessedOut[k+2] = resright[k/4]&0xff;
		
	}
}

void computeVoice(float freq, uint8_t voiceID) {
  //compute oscillator outputs for note
  // required frequency * 3.2 outputs samples per Transfer
  float samplesPerTrans = freq * 3.2;
  computeOscillatorOut(0, 0, samplesPerTrans); // 109.4 Hz
  computeOscillatorOut(1,1, samplesPerTrans/2); //1250 Hz
  computeOscillatorOut(2,2, samplesPerTrans/4); // 140.3 Hz
  //computeOscillatorOut(3,3, 1408); // 1562.1 Hz
}


// ssampleBufferID = last used sample from main ssample, per oscillator
// channelID = f_ssample_outChannel per oscillator
// samplesPerTranfer = nbr of samples used from main ssample, per processing block
void computeOscillatorOut(uint8_t ssampleBufferID, uint8_t channelID, uint16_t samplesPerTransfer) {
  // clear array
  // set index to 0
  uint16_t ssamples_current_index = 0;
  memset(f_ssample_freq_specific, 0, sizeof f_ssample_freq_specific);
  
  // smaller ssample than 600 samples
  // take part of ssample and interpolate to ssamplesamplespertransfer
  if (samplesPerTransfer < 600) {
    
    if ((600 - ssample_small_lastpos[ssampleBufferID]) > samplesPerTransfer) {  
      // enough samples, take part of ssample
      memcpy(&f_ssample_freq_specific[0], 
             &f_ssample[ssampleBufferID][ssample_small_lastpos[ssampleBufferID]], 
             samplesPerTransfer * 4);
    } else {
      // copy rest
      memcpy(&f_ssample_freq_specific[0], 
             &f_ssample[ssampleBufferID][ssample_small_lastpos[ssampleBufferID]], 
             (600 - ssample_small_lastpos[ssampleBufferID]) * 4); 
      // start new sample, copy rest from that sample, set lastpos
      memcpy(&f_ssample_freq_specific[600 - ssample_small_lastpos[ssampleBufferID]], 
             &f_ssample[ssampleBufferID][0], 
             (ssample_small_lastpos[ssampleBufferID]) * 4);
    }
    
    // interpolate to ssamplesamplespertransfer
    inter1parray( f_ssample_freq_specific, samplesPerTransfer, f_ssample_temp, 600 );
    
    // and interpolate to 512 and send to mixer
    inter1parray( f_ssample_temp, 600, f_ssample_outChannel[channelID], 512 );
    // set ssample_small_lastpos[ssampleBufferID] to lastpos
    ssample_small_lastpos[ssampleBufferID] += samplesPerTransfer; 
    // set overflow
    if (ssample_small_lastpos[ssampleBufferID] > 600) {ssample_small_lastpos[ssampleBufferID] -= 600;}
    
  } else {
  
    // first place remaining samples from last run in f_ssample_freq_specific
    // start copy from ssample_pre_left at idx from last run
    // length = 600 - remaining samples index
    uint16_t ssamples_remaining_length = 600-ssamples_remaining_idx[ssampleBufferID];
    memcpy(&f_ssample_freq_specific[ssamples_current_index], 
           &f_ssample[ssampleBufferID][ssamples_remaining_idx[ssampleBufferID]], 
           ssamples_remaining_length * 4);
    // move index to length
    ssamples_current_index += ssamples_remaining_length;
  
    // build new waveform in f_ssample_freq_specific from ssample which is 600 samples long
    // length of f_ssample_freq_specific is in samplespertransfer
    // determine number of complete waveforms
    // subtract already placed samples from samplespertransfer
    // divide int trough int results in a ceiling rounding
    for(int i=0; i<((samplesPerTransfer - ssamples_remaining_length) / 600); i++) {
      memcpy(&f_ssample_freq_specific[ssamples_current_index], 
             &f_ssample[ssampleBufferID][0], 
             600 * 4);
      ssamples_current_index += 600;
    }
  
    // determine remaining positions
    uint16_t remaining_pos = (samplesPerTransfer - ssamples_current_index);
    // and copy (part of) new ssample into that positions
    memcpy(&f_ssample_freq_specific[ssamples_current_index], 
           &f_ssample[ssampleBufferID][0], 
           remaining_pos * 4); 
  
    // set remaining samples from ssample
    ssamples_remaining_idx[ssampleBufferID] = remaining_pos;
  
    // interpolate to 512 samples
    inter1parray( f_ssample_freq_specific, samplesPerTransfer, f_ssample_outChannel[channelID], 512 ); 
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
  
  BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
  BSP_LCD_FillRect(0,0,480,24);
  
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_DisplayStringAt(5, 5, (uint8_t *)"OCTAPAL", LEFT_MODE);
  
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(5, 60, (uint8_t *)"vco1", LEFT_MODE);
  BSP_LCD_DisplayStringAt(5, 140, (uint8_t *)"vco2", LEFT_MODE);
  BSP_LCD_DisplayStringAt(5, 220, (uint8_t *)"vco3", LEFT_MODE);
  
  drawSSample(vco1wave,40,35);
  drawSSample(vco2wave,40,115);
  drawSSample(vco3wave,40,195);
  
  drawStepSeqTopBar(0);
  
  //BSP_LCD_DrawPolygon(wave1,50);
    
  //drawButton(10,29,(uint8_t *)"1");
  //drawButton(125,29,(uint8_t *)"2");
  //drawButton(240,29,(uint8_t *)"3");
  //drawButton(355,29,(uint8_t *)"4");
}

void drawSequencer() {
  // clear the screen, but not the topbar
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_FillRect(0,24,480,360);
}

void drawStepSeqTopBar(uint16_t active) {
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  for (int i=0; i < 16; i++) {
    BSP_LCD_FillRect(230+(i*12),7,10,10);
  }
  if (active > 0) {
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_FillRect(230+((active-1)*12),7,10,10);
  }
}

void drawWaveSelector() {  
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_FillRect(10,30,460,237);
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
  BSP_LCD_FillRect(12,32,456,233);
  //BSP_LCD_SetTextColor(LCD_COLOR_RED);
  //BSP_LCD_FillRect(12,12,456,20);
  
  //BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  //BSP_LCD_SetBackColor(LCD_COLOR_RED);
  //BSP_LCD_SetFont(&Font12);
  //BSP_LCD_DisplayStringAt(17, 17, (uint8_t *)"Select wave", LEFT_MODE);
  
  // 18 waves available sofar
  int8_t sid = 0;
  for (int row=0; row < 3; row++) {
    for (int col=0; col < 6; col++) {
      drawSSample(sid, 20 + col * 70 , 40 + 75 * row );
      sid++;
    }
  }
}

void drawSSample(uint16_t sampleID, uint16_t xstart, uint16_t ystart) {
  
  BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
  BSP_LCD_FillRect(xstart,ystart,60,60);
  
  BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
  BSP_LCD_DrawLine(xstart,ystart+30,xstart+59,ystart+30);
  
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  uint16_t y = ystart + 50 - sswave[sampleID][0];
  uint16_t x = xstart + 5;
  for (int j=1; j < 50; j++) {
    BSP_LCD_DrawLine(x,y,x+1,ystart + 5 + 50 - sswave[sampleID][j]);
    x++;
    y = ystart + 5 + 50 - sswave[sampleID][j];
  }
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(xstart, ystart+60, (uint8_t *)wavenames[sampleID], LEFT_MODE);
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
      
      // determine and handle current touchMap
      if (strcmp(touchMap,"main")==0) {
        // vco1 select
        if(touchx > 40 && touchx < 100 && touchy > 35 && touchy < 95) {
          touchMap = "waveselect";
          varToUpdate = &vco1wave;
          waveformToUpdate = 0;
          drawWaveSelector();
        }
        
        if(touchx > 40 && touchx < 100 && touchy > 115 && touchy < 175) {
          touchMap = "waveselect";
          varToUpdate = &vco2wave;
          waveformToUpdate = 1;
          drawWaveSelector();
        }
        
        if(touchx > 40 && touchx < 100 && touchy > 195 && touchy < 255) {
          touchMap = "waveselect";
          varToUpdate = &vco3wave;
          waveformToUpdate = 2;
          drawWaveSelector();
        }
        
        // sequencerselect
        if(touchx > 280 && touchx < 480 && touchy > 0 && touchy < 20) {
          touchMap = "waveselect";
          //varToUpdate = &vco3wave;
          //waveformToUpdate = 2;
          drawSequencer();
        }
      }
        
      else if (strcmp(touchMap,"waveselect")==0) {
        
        int8_t sid = 0;
        int8_t once = 0;
        for (int row=0; row < 3; row++) {
          for (int column=0; column < 6; column++) {
            if(touchx > (20 + column * 70) && touchx < (80 + column * 70) && touchy > (40 + 75 * row) && touchy < (100 + 75 * row)) {
                *varToUpdate = sid;
                __disable_irq();
                openSCwaveform(waveformToUpdate, sid+1);
                __enable_irq();
                touchMap = "main";
                drawInterface();      
                break;
            }
            sid++;
          }
        }
      }
        
        // wave select
        //if(touchx > 20 && touchx < 100 && touchy > 35 && touchy < 95) {
          // update pointed to variable (vco1wave, vco2wave or vco3wave)
          // with new waveid
        //  *varToUpdate = 4;
        //  touchMap = "main";
        //  drawInterface();
       // }
        
      //}
			
      /*
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
          voice_frequency[0] = 1408;
          //f_lseek(&audio_file[0], 0);
          //f_ssample_outChannel_Volume[0] += 0.1;
          //  if(f_ssample_outChannel_Volume[0] > 0.6) {f_ssample_outChannel_Volume[0]=0;}
        }
        if(touchx > 120 && touchx < 240) {
          voice_frequency[0] = 700;
          //f_lseek(&audio_file[1], 0);
          //f_ssample_outChannel_Volume[1] += 0.1;
          //  if(f_ssample_outChannel_Volume[1] > 0.6) {f_ssample_outChannel_Volume[1]=0;}
        }
        if(touchx > 240 && touchx < 360) {
          f_ssample_outChannel_Volume[2] += 0.1;
            if(f_ssample_outChannel_Volume[2] > 0.6) {f_ssample_outChannel_Volume[2]=0;}
        }
        if(touchx > 360 && touchx < 480) {
          f_ssample_outChannel_Volume[3] += 0.1;
            if(f_ssample_outChannel_Volume[3] > 0.6) {f_ssample_outChannel_Volume[3]=0;}
        }
        //} else {
        //  f_lseek(&audio_file[1], 0);
        //}

      }*/
			
			runOnce = 1;
		}
	// read state and continue with while
	BSP_TS_GetState(&rawTouchState);
	} // end while
}



// Initialize timer w/ 10kHz resolution and given period
static void initTimer(uint16_t period) {
  uint32_t prescaler           = (uint32_t)((SystemCoreClock / 2) / 10000) - 1;
  timer.Instance               = TIMx;
  timer.Init.Period            = period - 1;
  timer.Init.Prescaler         = prescaler;
  timer.Init.ClockDivision     = 0;
  timer.Init.CounterMode       = TIM_COUNTERMODE_UP;
  timer.Init.RepetitionCounter = 0;

  if (HAL_TIM_Base_Init(&timer) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&timer) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim) {
  // TIMx Peripheral clock enable
  TIMx_CLK_ENABLE();
  // Set the TIMx priority
  HAL_NVIC_SetPriority(TIMx_IRQn, 3, 0);
  // Enable the TIMx global Interrupt
  HAL_NVIC_EnableIRQ(TIMx_IRQn);
}

// Timer interrupt request.
void TIMx_IRQHandler(void) {
  HAL_TIM_IRQHandler(&timer);
}

// Callback function run whenever timer caused interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  //if (isBlinking) {
  //  BSP_LED_Toggle(LED_GREEN);
  //}
  drawStepSeqTopBar(currentStep);
  //sequence[currentStep];
  
  float req_freq = (13.75 * (pow(2,(current_midi_note + sequence[currentStep] -9.0)/12.0)));
  char b[10];
  int dingus = req_freq * 1000;
  //sprintf(b, "%f",req_freq);
  //sprintf(b, "bla%g", req_freq );
  snprintf(b, 10, "%d",dingus);
   BSP_LCD_SetFont(&Font12);
   BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
   BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
   BSP_LCD_DisplayStringAt(7, 7, (uint8_t *)"      ", RIGHT_MODE);
   BSP_LCD_DisplayStringAt(7, 7, (uint8_t *)b, RIGHT_MODE);
 
   voice_frequency[0] = req_freq;
  
  currentStep++;
  if(currentStep>16) {currentStep=1;}
}





void UART6_Config() {
  
//************  UART CONFIG  *****************************//

  __USART6_CLK_ENABLE();

  uart_config.Instance=USART6;

  uart_config.Init.BaudRate=31250;
  uart_config.Init.WordLength=UART_WORDLENGTH_8B;
  uart_config.Init.StopBits=UART_STOPBITS_1;
  uart_config.Init.Parity=UART_PARITY_NONE;
  uart_config.Init.Mode=UART_MODE_TX_RX;
  uart_config.Init.HwFlowCtl=UART_HWCONTROL_NONE;
  
  HAL_UART_Init(&uart_config);

  HAL_NVIC_SetPriority(USART6_IRQn,0,1);
  HAL_NVIC_EnableIRQ(USART6_IRQn);

//**********************************************************//

//************ UART GPIO CONFIG  *********************//

  GPIO_InitTypeDef uart_gpio;

  __GPIOC_CLK_ENABLE();

  uart_gpio.Pin=GPIO_PIN_7;
  uart_gpio.Mode=GPIO_MODE_AF_PP;
  //uart_gpio.Pull=GPIO_NOPULL;
  uart_gpio.Pull=GPIO_PULLUP;
  uart_gpio.Speed=GPIO_SPEED_FAST;
  uart_gpio.Alternate=GPIO_AF8_USART6;

  HAL_GPIO_Init(GPIOC, &uart_gpio);
  HAL_UART_Receive_IT(&uart_config, rx_byte, 1);
}

void USART6_IRQHandler(void)
{
  HAL_UART_IRQHandler(&uart_config);
}
