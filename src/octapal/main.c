/*

octapal
===================

a midi controlled soundmodule

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

#include "soundpipe.h"

// soundpipe globals
#define MY_BUFSIZE 64

static volatile uint8_t nextbuf;
static int16_t buf0[MY_BUFSIZE];
static int16_t buf1[MY_BUFSIZE];

/* SOUNDPIPE */

#define NOSCS 5

static const uint8_t scale[] = {48, 60, 65, 67, 72, 79, 64, 52};

static uint8_t please_play = 0;
static sp_data *sp;
static sp_ftbl *ft;
static sp_osc *osc;
static sp_fosc *fosc[NOSCS];
static sp_revsc *revsc;

uint32_t compute_buffer(int16_t *pbuf, int  bufsize);

__IO uint32_t XferCplt = 0;

// end soundpipe globals

#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"

#include "common/clockconfig.h"

#define TIMx TIM3
#define TIMx_CLK_ENABLE() __HAL_RCC_TIM3_CLK_ENABLE()
#define TIMx_IRQn TIM3_IRQn
#define TIMx_IRQHandler TIM3_IRQHandler
TIM_HandleTypeDef timer;

#define TIMy TIM4
#define TIMy_CLK_ENABLE() __HAL_RCC_TIM4_CLK_ENABLE()
#define TIMy_IRQn TIM4_IRQn
#define TIMy_IRQHandler TIM4_IRQHandler
TIM_HandleTypeDef timer2;

uint8_t currentStep = 1;

static void initTimer(uint16_t period);

uint8_t currentLCDcolor = 0;
const uint32_t LCDColorarray[] = { LCD_COLOR_YELLOW, LCD_COLOR_GREEN, LCD_COLOR_ORANGE, LCD_COLOR_MAGENTA };

#define VOLUME 90
#define SAMPLE_RATE 48000
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

// adsr timings
float adsr_volume_multi = 1;
float adsr[4] = {120,10,0.05,200};
uint32_t adsr_timer_pervoice[4] = {0,0,0,0};
uint8_t midiNoteOn = 0;
uint16_t voiceAllocation[4] = {0,0,0,0};

uint32_t prevTick = 0;
// start editing voice 1
uint8_t currentVoiceInEdit = 1;

uint16_t *varToUpdate;
uint16_t waveformToUpdate;

float f_bufPost_mixdown_left[1024];
float f_bufPost_mixdown_right[1024];

float voice_frequency[4] = {0,0,0,0};

int8_t sequence[16] = {0,0,0,0,0,-1,0,-2,0,1,3,-3,2,0,4,5};
uint8_t current_midi_note = 50;


uint16_t printpos = 5;
uint16_t printypos = 5;


float f_ssample[12][600];
uint8_t active_ssample = 1;
uint8_t receiving_note_on = 0;
// max 32 cycle = 32*600 = 19200
float f_ssample_freq_specific[19200];
float f_ssample_outChannel[12][600];  
float f_ssample_outChannel_Volume[12] = {0.05, 0.05, 0.05, 0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05};
float f_ssample_temp[600];
float f_ssample_right[600];
uint16_t ssample_lastpos = 0;
uint16_t ssample_small_lastpos[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
int16_t ssamples_remaining_idx[12] = {600,600,600,600,600,600,600,600,600,600,600,600};

int32_t adsr_timer = 0;

short resleft[512];
short resright[512];

extern SAI_HandleTypeDef haudio_out_sai;

UART_HandleTypeDef uart_config;

uint8_t midicounter=0;
int16_t mididata[3] = {0,-1,-1};
uint8_t rx_byte[1];

uint16_t vco1wave = 1;
uint16_t vco2wave = 2;
uint16_t vco3wave = 1;
uint16_t vco4wave = 2;
uint16_t vco5wave = 1;
uint16_t vco6wave = 2;
uint16_t vco7wave = 1;
uint16_t vco8wave = 2;

FIL audio_file[12];
FIL ssample[12];
UINT bytes_read[12];
UINT prev_bytes_read;
FATFS fs;
FATFS SDFatFs;

static char usb_drive_path[4];

static TS_StateTypeDef rawTouchState;

uint16_t runOnce = 0;

char *touchMap = "main";

void drawSSample(uint16_t sampleID, uint16_t xstart, uint16_t ystart);
void UART6_Config();
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
void drawStepSeqTopBar();
void drawSequencer();
void drawVolumeIndicators();
void drawVolumeControls();
void handle_midi();
void drawVoiceInterface(uint8_t voice);
static void initNoteOffTimer(uint16_t period);

int main() {
  CPU_CACHE_Enable();
  HAL_Init();
  SystemClock_Config(); 
  
  // config UART for MIDI communication
  UART6_Config();
  
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Off(LED_GREEN);
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  
  // init sequencer
  //initTimer(20000);
  //initNoteOffTimer(10000);
  
  // Init LCD and Touchscreen
  BSP_LCD_Init();
  if (BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize()) == TS_OK) {
    BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
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
		
  // load initial waveforms in oscillators
  openSCwaveform(0, vco1wave);
  openSCwaveform(1, vco2wave);
  openSCwaveform(2, vco3wave);
  openSCwaveform(3, vco4wave);
  openSCwaveform(4, vco5wave);
  openSCwaveform(5, vco6wave);
  openSCwaveform(6, vco7wave);
  openSCwaveform(7, vco8wave);
  // openSCwaveform(8, 10);
  // openSCwaveform(9, 12);
  // openSCwaveform(10, 4);
  // openSCwaveform(11, 6);
  
  initAudio();
  
  sp_create(&sp);
  sp->sr = 48000;

  sp_ftbl_create(sp, &ft, 8192);
  sp_gen_sine(sp, ft);

  sp_osc_create(&osc);

  sp_osc_init(sp, osc, ft, 0);
  osc->freq = 0.2f;
  osc->amp = 1.f;
  uint32_t i;
  
  
  drawInterface();
  
  //voice_frequency[0] = 440;
  
  while (1) {		
	  //nothing is done here	
  }

  return 0;
}

// UART (MIDI) HANDLING --------------------------------------------

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  // check for status (>127) or data
  // does not handle aftertouch yet!
  // e.g. an message with just 1 data byte
  if(rx_byte[0]>127) {
    mididata[0]=rx_byte[0];
  } 
  else
    // data coming in, in two bytes
  {
    if(mididata[1]==-1) {
      mididata[1] = rx_byte[0];
    } else {
      mididata[2] = rx_byte[0];
      handle_midi();
      mididata[1] = -1;
    }
  }
  // reset interrupt
  HAL_UART_Receive_IT(&uart_config, rx_byte, 1);
}  
  
void handle_midi() {
  
  char a[] = "";
  sprintf(a, "%d %d %d", mididata[0],mididata[1],mididata[2]);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAt(300, 7, (uint8_t *)a, LEFT_MODE);
  
  
  // note on if vel (mididata[2]) > 0
  if(mididata[0] == 144 && mididata[2] > 0) {
    
    // check voiceallocation for available voice (if voicemode)
    // start at the back, end up with lowest available voice
    int availableVoice;
    if(voiceAllocation[3]==0) {availableVoice=3;}
    if(voiceAllocation[2]==0) {availableVoice=2;}
    if(voiceAllocation[1]==0) {availableVoice=1;}
    if(voiceAllocation[0]==0) {availableVoice=0;}
  
    // reserve voice
    voiceAllocation[availableVoice] = mididata[1];
    
    // show voices
    char a[] = "";
    sprintf(a, "%d %d %d %d", voiceAllocation[0],voiceAllocation[1],voiceAllocation[2],voiceAllocation[3]);
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DisplayStringAt(200, 7, (uint8_t *)a, LEFT_MODE);
    
    // calculate frequency
    float req_freq = (13.75 * (pow(2,(mididata[1]-9.0)/12.0)));
    
    // show frequency
    char b[10];
    int dingus = req_freq * 1000;
    snprintf(b, 10, "%d",dingus);
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DisplayStringAt(7, 7, (uint8_t *)"      ", RIGHT_MODE);
    BSP_LCD_DisplayStringAt(7, 7, (uint8_t *)b, RIGHT_MODE);
   
    // start new note
    midiNoteOn = 1;
    adsr_timer = 0;
    voice_frequency[availableVoice] = req_freq;
  }
   
   // implicit note off
 if(mididata[0] == 144 && mididata[2] == 0) {
   
   //float req_freq = (13.75 * (pow(2,(mididata[1]-9.0)/12.0)));
  
   //char b[10];
   //int dingus = req_freq * 1000;
   //snprintf(b, 10, "%d",dingus);
   BSP_LCD_SetFont(&Font12);
   BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
   BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
   BSP_LCD_DisplayStringAt(7, 7, (uint8_t *)"    off", RIGHT_MODE);
   //BSP_LCD_DisplayStringAt(7, 7, (uint8_t *)b, RIGHT_MODE);
 
   // reset voice allocation
   int availableVoice;
   if(voiceAllocation[3]==mididata[1]) {voiceAllocation[3]=0;availableVoice=3;}
   if(voiceAllocation[2]==mididata[1]) {voiceAllocation[2]=0;availableVoice=2;}
   if(voiceAllocation[1]==mididata[1]) {voiceAllocation[1]=0;availableVoice=1;}
   if(voiceAllocation[0]==mididata[1]) {voiceAllocation[0]=0;availableVoice=0;}
   
   // show voices
   char a[] = "";
   sprintf(a, "%d %d %d %d", voiceAllocation[0],voiceAllocation[1],voiceAllocation[2],voiceAllocation[3]);
   BSP_LCD_SetFont(&Font12);
   BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
   BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
   BSP_LCD_DisplayStringAt(200, 7, (uint8_t *)a, LEFT_MODE);
   
   // end this note
   midiNoteOn = 0;
   voice_frequency[availableVoice] = 0;
   
   //midiNoteOn = 1;
   //adsr_timer = 0;
   //voice_frequency[0] = req_freq;
  }
   
   if(mididata[0] == 176) {
     if(mididata[1]==71) {
       f_ssample_outChannel_Volume[0] = mididata[2]/127.0;
       drawVolumeIndicators();
     }
     if(mididata[1]==72) {
       f_ssample_outChannel_Volume[1] = mididata[2]/127.0;
       drawVolumeIndicators();
     }
     if(mididata[1]==73) {
       f_ssample_outChannel_Volume[2] = mididata[2]/127.0;
       drawVolumeIndicators();
     }
     if(mididata[1]==74) {
       adsr[0] = mididata[2];
     }
   } 
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
  } else {
		char a[] = "";
		sprintf(a, "%d", openresult);
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                              (uint8_t *)"load error", CENTER_MODE);
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
  
  //   // temp performance test
  //
  //   //CORE_SysTickEn();
  //   float f = 1.01f;
  //   uint32_t it = HAL_GetTick();
  //   float f2 = f * 2.29f;
  //   uint32_t it2 = HAL_GetTick();
  //
  //   BSP_LCD_Clear(LCD_COLOR_BLUE);
  // char a[] = "";
  // sprintf(a, "%ld", it2);
  //     BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
  //                             (uint8_t *)a, CENTER_MODE);
  //   Error_Handler();
  
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
    } else {
      // perform RELEASE
      // clear channel memory
      if (k==0) {
        //if(f_ssample_outChannel_Volume[0] > 0) {
          // RELEASE 
        //  f_ssample_outChannel[0] = f_ssample_outChannel[0] - (1 / adsr[3]);
        //} else {
          memset(f_ssample_outChannel[0], 0, sizeof f_ssample_outChannel[0]);
          memset(f_ssample_outChannel[1], 0, sizeof f_ssample_outChannel[1]);
          adsr_timer_pervoice[0]=0;
          //}
      }
      if (k==1) {
        memset(f_ssample_outChannel[2], 0, sizeof f_ssample_outChannel[2]);
        memset(f_ssample_outChannel[3], 0, sizeof f_ssample_outChannel[3]);
        adsr_timer_pervoice[1]=0;
      }
      if (k==2) {
        memset(f_ssample_outChannel[4], 0, sizeof f_ssample_outChannel[4]);
        memset(f_ssample_outChannel[5], 0, sizeof f_ssample_outChannel[5]);
        adsr_timer_pervoice[2]=0;
      }
      if (k==3) {
        memset(f_ssample_outChannel[6], 0, sizeof f_ssample_outChannel[6]);
        memset(f_ssample_outChannel[7], 0, sizeof f_ssample_outChannel[7]);
        adsr_timer_pervoice[3]=0;
      }
    }
  }
  //computeVoice(440);
  
  // adsr[4] = {140,50,20,20};
  
  // attack
  // adsr_volume_multi goes to 1 at maximum
  
  // Attack, Decay and Sustain because noteon
  // if(midiNoteOn==1) {
  //   if(adsr_timer<adsr[0]) {
  //     adsr_volume_multi = adsr_timer / adsr[0];
  //   }
  //
  //   // decay
  //   uint32_t decaystage = adsr_timer-adsr[0];
  //   if(adsr_timer>adsr[0] && decaystage<adsr[1])
  //   {
  //     //adsr_volume_multi -= 0.01; //1 - (((float)decaystage / adsr[1]) * adsr[2]);
  //     adsr_volume_multi = 1 - ((decaystage / adsr[1]) * (1-adsr[2]));
  //   }
  // } else {
  //   adsr_volume_multi = adsr_volume_multi - (1 / adsr[3]);
  // }
  //
  //
  // if(adsr_volume_multi<=0){//adsr_timer > 30000) {
  //   memset(int_bufProcessedOut, 0, sizeof int_bufProcessedOut);
  // } else {
  //
  	// the mix
  	// float to short
  	for (int k=0; k<2048; k=k+4) {
    
      f_bufPost_mixdown_left[k/4] =   (
                                        (f_ssample_outChannel_Volume[0] * f_ssample_outChannel[0][k/4])
                                        + (f_ssample_outChannel_Volume[1] * f_ssample_outChannel[1][k/4])
                                        + (f_ssample_outChannel_Volume[2] * f_ssample_outChannel[2][k/4])
                                        + (f_ssample_outChannel_Volume[3] * f_ssample_outChannel[3][k/4])
                                        + (f_ssample_outChannel_Volume[4] * f_ssample_outChannel[4][k/4])
                                        + (f_ssample_outChannel_Volume[5] * f_ssample_outChannel[5][k/4])
                                        + (f_ssample_outChannel_Volume[6] * f_ssample_outChannel[6][k/4])
                                        + (f_ssample_outChannel_Volume[7] * f_ssample_outChannel[7][k/4])
//                                           + (f_ssample_outChannel_Volume[8] * f_ssample_outChannel[8][k/4])
//                                           + (f_ssample_outChannel_Volume[9] * f_ssample_outChannel[9][k/4])
//                                           + (f_ssample_outChannel_Volume[10] * f_ssample_outChannel[10][k/4])
//                                           + (f_ssample_outChannel_Volume[11] * f_ssample_outChannel[11][k/4])
                                      ) * adsr_volume_multi;

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
		
    //}
  
  adsr_timer++;
  
}

  //   //CORE_SysTickEn();
  //   float f = 1.01f;
  //   uint32_t it = HAL_GetTick();
  //   float f2 = f * 2.29f;
  //   uint32_t it2 = HAL_GetTick();
  //
  //   BSP_LCD_Clear(LCD_COLOR_BLUE);
  // char a[] = "";
  // sprintf(a, "%ld", it2);
  //     BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
  //                             (uint8_t *)a, CENTER_MODE);
  //   Error_Handler();

void computeVoice(float freq, uint8_t voiceID) {
  //compute oscillator outputs for note
  // required frequency * 3.2 outputs samples per Transfer
  float samplesPerTrans = freq * 3.2;
  
  if(voiceID==0) {
    computeOscillatorOut(0, 0, samplesPerTrans); // 109.4 Hz
    computeOscillatorOut(1,1, samplesPerTrans); //1250 Hz
    
    // ATTACK
    if(adsr_timer_pervoice[0]<adsr[0]) {
      f_ssample_outChannel_Volume[0] = ((adsr_timer / adsr[0]) / 10);
    }
    f_ssample_outChannel_Volume[1] = f_ssample_outChannel_Volume[0];
    
    // DECAY + SUSTAIN
    //uint32_t decaystage = adsr_timer_pervoice[0]-adsr[0];
    //if(adsr_timer_pervoice[0]>adsr[0] && decaystage<adsr[1])
    //{
    //    f_ssample_outChannel_Volume[0] = 1 - ((decaystage / adsr[1]) * (1-adsr[2]));
    //}
    
    
    adsr_timer_pervoice[0]++;
  }
  if(voiceID==1) {
    computeOscillatorOut(2,2, samplesPerTrans); // 109.4 Hz
    computeOscillatorOut(3,3, samplesPerTrans); //1250 Hz
    
    // ATTACK
    if(adsr_timer_pervoice[1]<adsr[0]) {
      f_ssample_outChannel_Volume[2] = ((adsr_timer / adsr[0]) / 10);
    }
    f_ssample_outChannel_Volume[3] = f_ssample_outChannel_Volume[2];
    
    adsr_timer_pervoice[1]++;
  }
  if(voiceID==2) {
    computeOscillatorOut(4,4, samplesPerTrans); // 109.4 Hz
    computeOscillatorOut(5,5, samplesPerTrans); //1250 Hz
    
    // ATTACK
    if(adsr_timer_pervoice[2]<adsr[0]) {
      f_ssample_outChannel_Volume[4] = ((adsr_timer / adsr[0]) / 10);
    }
    f_ssample_outChannel_Volume[5] = f_ssample_outChannel_Volume[4];
    
    adsr_timer_pervoice[2]++;
  }
  if(voiceID==3) {
    computeOscillatorOut(6,6, samplesPerTrans); // 109.4 Hz
    computeOscillatorOut(7,7, samplesPerTrans); //1250 Hz
    
    // ATTACK
    if(adsr_timer_pervoice[3]<adsr[0]) {
      f_ssample_outChannel_Volume[6] = ((adsr_timer / adsr[0]) / 10);
    }
    f_ssample_outChannel_Volume[7] = f_ssample_outChannel_Volume[6];
    
    adsr_timer_pervoice[3]++;
  }
  
  // computeOscillatorOut(2,2, 300); // 140.3 Hz
  // computeOscillatorOut(3,3, 550); // 1562.1 Hz
  // computeOscillatorOut(4,4, 600); // 1562.1 Hz
  // computeOscillatorOut(5,5, 560); // 1562.1 Hz
  // computeOscillatorOut(6,6, 2800); // 1562.1 Hz
  // computeOscillatorOut(7,7, 2000); // 1562.1 Hz
  // computeOscillatorOut(8,4, 2800); // 1562.1 Hz
  // computeOscillatorOut(9,5, 2000); // 1562.1 Hz
  // computeOscillatorOut(10,6, 2230); // 1562.1 Hz
  // computeOscillatorOut(11,7, 2400); // 1562.1 Hz
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
    //inter1parray( f_ssample_freq_specific, samplesPerTransfer, f_ssample_temp, 600 );
    
    // and interpolate to 512 and send to mixer
    //inter1parray( f_ssample_temp, 600, f_ssample_outChannel[channelID], 512 );
    
    // also possible in one go
    inter1parray( f_ssample_freq_specific, samplesPerTransfer,f_ssample_outChannel[channelID], 512);
    
    
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
  BSP_LCD_Clear(LCD_COLOR_DARKMAGENTA);
  
  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_FillRect(0,0,206,136);
	BSP_LCD_SetFont(&Font24);
  BSP_LCD_SetBackColor(LCD_COLOR_RED);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAt(90, 58, (uint8_t *)"1", LEFT_MODE);
  
  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
  BSP_LCD_FillRect(206,0,206,136);
  BSP_LCD_SetBackColor(LCD_COLOR_YELLOW);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAt(296, 58, (uint8_t *)"2", LEFT_MODE);
  
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_FillRect(0,136,206,136);
  BSP_LCD_SetBackColor(LCD_COLOR_GREEN);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAt(90, 194, (uint8_t *)"3", LEFT_MODE);
  
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_FillRect(206,136,206,136);
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAt(296, 194, (uint8_t *)"4", LEFT_MODE);
  
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_FillRect(412,204,68,68);
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(425, 235, (uint8_t *)"<- back", LEFT_MODE);
  
  // voice selectors
  /*
  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_FillRect(0,0,68,68);
	BSP_LCD_SetFont(&Font24);
  BSP_LCD_SetBackColor(LCD_COLOR_RED);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAt(26, 24, (uint8_t *)"1", LEFT_MODE);
  
  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
  BSP_LCD_FillRect(0,68,68,68);
	BSP_LCD_SetFont(&Font24);
  BSP_LCD_SetBackColor(LCD_COLOR_YELLOW);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAt(26, 92, (uint8_t *)"2", LEFT_MODE);
  
  
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_FillRect(0,136,68,68);
	BSP_LCD_SetFont(&Font24);
  BSP_LCD_SetBackColor(LCD_COLOR_GREEN);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAt(28, 160, (uint8_t *)"3", LEFT_MODE);
  
  
  BSP_LCD_SetTextColor(LCD_COLOR_DARKMAGENTA);
  BSP_LCD_FillRect(0,204,68,68);
	BSP_LCD_SetFont(&Font24);
  BSP_LCD_SetBackColor(LCD_COLOR_DARKMAGENTA);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAt(28, 228, (uint8_t *)"4", LEFT_MODE);
  
  
  if(currentVoiceInEdit==1) {
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
  }
  else if(currentVoiceInEdit==2) {
    BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
  }
  BSP_LCD_FillRect(68,0,412,24);
  
  
  
  BSP_LCD_SetFont(&Font20);
  if(currentVoiceInEdit==1) {
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetBackColor(LCD_COLOR_RED);
    BSP_LCD_DisplayStringAt(78, 4, (uint8_t *)"voice 1", LEFT_MODE);
  }
  else if(currentVoiceInEdit==2) {
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_SetBackColor(LCD_COLOR_YELLOW);
    BSP_LCD_DisplayStringAt(78, 4, (uint8_t *)"voice 2", LEFT_MODE);
  }
 
  
  // BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
//   BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
//   BSP_LCD_SetFont(&Font12);
//   BSP_LCD_DisplayStringAt(5, 60, (uint8_t *)"osc1", LEFT_MODE);
//   BSP_LCD_DisplayStringAt(5, 140, (uint8_t *)"osc2", LEFT_MODE);
// //  BSP_LCD_DisplayStringAt(5, 220, (uint8_t *)"osc3", LEFT_MODE);
//
  if(currentVoiceInEdit==1) {
    drawSSample(vco1wave,78,35);
    drawSSample(vco2wave,78,115);
  }  
  
  if(currentVoiceInEdit==2) {
   drawSSample(vco3wave,78,35);
   drawSSample(vco4wave,78,115);
 }
   
   // tuning per osc
   BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
   BSP_LCD_FillRect(150,35,60,60);
   BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
   BSP_LCD_FillRect(150,115,60,60);
   
   // volume per osc
   BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
   BSP_LCD_FillRect(222,35,60,60);
   BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
   BSP_LCD_FillRect(222,115,60,60);
   
   // adsr per voice
   BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
   BSP_LCD_FillRect(294,75,60,60);
      
   // seq per voice
   BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
   for (int i=0; i < 16; i++) {
     BSP_LCD_FillRect(78+(i*12),200,10,10);
   }    
// //  drawSSample(vco3wave,40,195);
//
//   drawVolumeIndicators();
//
//   drawStepSeqTopBar();
//
  //BSP_LCD_DrawPolygon(wave1,50);
    
  //drawButton(10,29,(uint8_t *)"1");
  //drawButton(125,29,(uint8_t *)"2");
  //drawButton(240,29,(uint8_t *)"3");
  //drawButton(355,29,(uint8_t *)"4");
  */
}

void drawVoiceInterface(uint8_t voice) {
  if(voice==1) {
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_FillRect(0,0,412,272);
  
    BSP_LCD_SetTextColor(LCD_COLOR_DARKMAGENTA);
    BSP_LCD_FillRect(0,0,412,24);
  
  	BSP_LCD_SetFont(&Font16);
    BSP_LCD_SetBackColor(LCD_COLOR_DARKMAGENTA);
  	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DisplayStringAt(5, 5, (uint8_t *)"voice 1", LEFT_MODE);
  }
  
  if(voice==2) {
    BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
    BSP_LCD_FillRect(0,0,412,272);
  
    BSP_LCD_SetTextColor(LCD_COLOR_DARKMAGENTA);
    BSP_LCD_FillRect(0,0,412,24);
  
  	BSP_LCD_SetFont(&Font16);
    BSP_LCD_SetBackColor(LCD_COLOR_DARKMAGENTA);
  	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DisplayStringAt(5, 5, (uint8_t *)"voice 2", LEFT_MODE);
  }
  
  BSP_LCD_SetTextColor(LCD_COLOR_DARKMAGENTA);
  BSP_LCD_FillRect(10,50,170,170);
  
  BSP_LCD_SetTextColor(LCD_COLOR_DARKMAGENTA);
  BSP_LCD_FillRect(210,50,170,170);
}

void drawVolumeIndicators() {
  BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
  BSP_LCD_FillRect(110,35,60,60);
  BSP_LCD_FillRect(110,115,60,60);
 // BSP_LCD_FillRect(110,195,60,60);

  // indicator 1
  BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
  BSP_LCD_FillRect(130,45,20,40);
  BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
  BSP_LCD_FillRect(131,46,18,38);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillRect(133,48+(34-f_ssample_outChannel_Volume[0]*34),14,(f_ssample_outChannel_Volume[0]*34)+1);
  
  // indicator 2
  BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
  BSP_LCD_FillRect(130,125,20,40);
  BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
  BSP_LCD_FillRect(131,126,18,38);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillRect(133,128+(34-f_ssample_outChannel_Volume[1]*34),14,(f_ssample_outChannel_Volume[1]*34)+1);
  
  // //indicator 3
  // BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
  // BSP_LCD_FillRect(130,205,20,40);
  // BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
  // BSP_LCD_FillRect(131,206,18,38);
  // BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  // BSP_LCD_FillRect(133,208+(34-f_ssample_outChannel_Volume[2]*34),14,(f_ssample_outChannel_Volume[2]*34)+1);
}

void drawVolumeControls() {
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
  BSP_LCD_FillRect(110,35,300,60);
  BSP_LCD_FillRect(110,115,300,60);
  //BSP_LCD_FillRect(110,195,300,60);
  
  // exit button
  BSP_LCD_FillRect(420,35,52,60);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_DARKBLUE);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(433, 60, (uint8_t *)"exit", LEFT_MODE);
  
  // volume controls per osc
  BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
  BSP_LCD_FillRect(130,45,260,40);
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
  BSP_LCD_FillRect(131,46,258,38);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillRect(134,49,(f_ssample_outChannel_Volume[0] * 252 * 6)+1,32);
  
  BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
  BSP_LCD_FillRect(130,125,260,40);
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
  BSP_LCD_FillRect(131,126,258,38);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillRect(134,129,(f_ssample_outChannel_Volume[1] * 252 * 6 )+1,32);
  
  // BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
  // BSP_LCD_FillRect(130,205,260,40);
  // BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
  // BSP_LCD_FillRect(131,206,258,38);
  // BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  // BSP_LCD_FillRect(134,209,(f_ssample_outChannel_Volume[2] * 252 * 6)+1,32);
}

void drawSequencer() {
  // clear the screen, but not the topbar
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_FillRect(0,24,480,360);
}

void drawStepSeqTopBar() {
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  for (int i=0; i < 16; i++) {
    BSP_LCD_FillRect(230+(i*12),7,10,10);
  }
  // draw current active step
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillRect(230+(currentStep*12),7,10,10);
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
      
      runOnce = 1;
			
			uint16_t touchx = rawTouchState.touchX[0];
      uint16_t touchy = rawTouchState.touchY[0];
      
      // determine and handle current touchMap
      if (strcmp(touchMap,"main")==0) {
        // voice 1
        if(touchx > 0 && touchx < 206 && touchy > 0 && touchy < 136) {
          //currentVoiceInEdit = 1;
          //drawInterface();
          drawVoiceInterface(1);
        }
        
        if(touchx > 206 && touchx < 412 && touchy > 0 && touchy < 136) {
          //currentVoiceInEdit = 1;
          //drawInterface();
          drawVoiceInterface(2);
        }
        
        //(412,204,68,68)
        if(touchx > 412 && touchx < 480 && touchy > 204 && touchy < 272) {
          //currentVoiceInEdit = 1;
          //drawInterface();
          drawInterface();
        }
        
        /*
        // voice select 1
        if(touchx > 0 && touchx < 68 && touchy > 0 && touchy < 68) {
          currentVoiceInEdit = 1;
          drawInterface();
        }
        // voice select 2
        if(touchx > 0 && touchx < 68 && touchy > 68 && touchy < 136) {
          currentVoiceInEdit = 2;
          drawInterface();
        }
        // voice select 3
        if(touchx > 0 && touchx < 68 && touchy > 136 && touchy < 204) {
          
        }
        // voice select 4
        if(touchx > 0 && touchx < 68 && touchy > 204 && touchy < 272) {
          
        }
                
        // vco1 select
        if(touchx > 78 && touchx < 138 && touchy > 35 && touchy < 95) {
          touchMap = "waveselect";
          if(currentVoiceInEdit==1) {
            varToUpdate = &vco1wave;
            waveformToUpdate = 0;
          } 
          if(currentVoiceInEdit==2) {
            varToUpdate = &vco3wave;
            waveformToUpdate = 2;
          } 
          drawWaveSelector();
        }
        
        //vco2 select
        if(touchx > 78 && touchx < 138 && touchy > 115 && touchy < 175) {
          touchMap = "waveselect";
          if(currentVoiceInEdit==1) {
            varToUpdate = &vco2wave;
            waveformToUpdate = 1;
          } 
          if(currentVoiceInEdit==2) {
            varToUpdate = &vco4wave;
            waveformToUpdate = 3;
          } 
          drawWaveSelector();
        }
        
        // //vco3 select
        // if(touchx > 40 && touchx < 100 && touchy > 195 && touchy < 255) {
        //   touchMap = "waveselect";
        //   varToUpdate = &vco3wave;
        //   waveformToUpdate = 2;
        //   drawWaveSelector();
        // }
        
        // // volume control (same interface for all buttons)
//         if(touchx > 110 && touchx < 170 && touchy > 35 && touchy < 95) {
//           touchMap = "volumecontrol";
//           drawVolumeControls();
//         }
//         if(touchx > 110 && touchx < 170 && touchy > 115 && touchy < 175) {
//           touchMap = "volumecontrol";
//           drawVolumeControls();
//         }
        // if(touchx > 110 && touchx < 170 && touchy > 195 && touchy < 255) {
        //   touchMap = "volumecontrol";
        //   drawVolumeControls();
        // }
        
        // sequencerselect
        // if(touchx > 280 && touchx < 480 && touchy > 0 && touchy < 20) {
//           touchMap = "waveselect";
//           drawSequencer();
//         }
        */
      }
        
      else if (strcmp(touchMap,"waveselect")==0) {
        
        int8_t sid = 0;
        //int8_t once = 0;
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
      
      else if (strcmp(touchMap,"volumecontrol")==0) {
        // BSP_LCD_FillRect(130,45,260,40);
        // (130,45,260,40); 125 205
        if(touchx > 130 && touchx < 390 && touchy > 45 && touchy < 85) {
          f_ssample_outChannel_Volume[0] = (((touchx - 130) / 260.0) / 6);
          drawVolumeControls();
        }
        
        if(touchx > 130 && touchx < 390 && touchy > 125 && touchy < 165) {
          f_ssample_outChannel_Volume[1] = (((touchx - 130) / 260.0) / 6);
          drawVolumeControls();
        }
        
        // if(touchx > 130 && touchx < 390 && touchy > 205 && touchy < 245) {
        //   f_ssample_outChannel_Volume[2] = (((touchx - 130) / 260.0) / 6);
        //   drawVolumeControls();
        // }
        //
        // exit
        // 420,35,52,60
        if(touchx > 420 && touchx < 472 && touchy > 35 && touchy < 95) {
          touchMap = "main";
          drawInterface();
        }
      }
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

// Initialize note off timer
static void initNoteOffTimer(uint16_t period) {
  uint32_t prescaler           = (uint32_t)((SystemCoreClock / 2) / 10000) - 1;
  timer2.Instance               = TIMy;
  timer2.Init.Period            = period - 1;
  timer2.Init.Prescaler         = prescaler;
  timer2.Init.ClockDivision     = 0;
  timer2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  timer2.Init.RepetitionCounter = 0;

  if (HAL_TIM_Base_Init(&timer2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&timer2) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim) {
  // check timer instance,
  // TIM3 = sequencer
  if(htim->Instance==TIM3) {
    // TIMx Peripheral clock enable
    TIMx_CLK_ENABLE();
    // Set the TIMx priority
    HAL_NVIC_SetPriority(TIMx_IRQn, 3, 0);
    // Enable the TIMx global Interrupt
    HAL_NVIC_EnableIRQ(TIMx_IRQn);
  }
  
  if(htim->Instance==TIM4) {
    // TIMx Peripheral clock enable
    TIMy_CLK_ENABLE();
    // Set the TIMx priority
    HAL_NVIC_SetPriority(TIMy_IRQn, 3, 0);
    // Enable the TIMx global Interrupt
    HAL_NVIC_EnableIRQ(TIMy_IRQn);
  }
}

// Timer interrupt request.
void TIMx_IRQHandler(void) {
  HAL_TIM_IRQHandler(&timer);
}

// Timer interrupt request.
void TIMy_IRQHandler(void) {
  HAL_TIM_IRQHandler(&timer2);
}

// Sequencer callback function
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  // TIM3 = sequencer timer
  if(htim->Instance==TIM3) {
    currentStep++;
    if(currentStep>15) {currentStep=0;}
    
    int8_t midistatus_buffer = mididata[0];
    mididata[0] = 144;
    mididata[1] = current_midi_note + sequence[currentStep];
    mididata[2] = 100;
    handle_midi();
    mididata[0] = midistatus_buffer;

  
    //drawStepSeqTopBar();
  }
  if(htim->Instance==TIM4) {
    BSP_LED_Toggle(LED_GREEN);
    // disable timer
    HAL_NVIC_DisableIRQ(TIMy_IRQn);
  }
}





void UART6_Config() {
  
//************  UART CONFIG  *****************************//

  __USART6_CLK_ENABLE();

  uart_config.Instance=USART6;

  uart_config.Init.BaudRate=31250;
  uart_config.Init.WordLength=UART_WORDLENGTH_8B;
  uart_config.Init.StopBits=UART_STOPBITS_1;
  uart_config.Init.Parity=UART_PARITY_NONE;
  uart_config.Init.Mode=UART_MODE_RX;
  uart_config.Init.HwFlowCtl=UART_HWCONTROL_NONE;
  //uart_config.Init.OverSampling = UART_OVERSAMPLING_8;
  
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
