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
#define VOLUME 40
#define SAMPLE_RATE 48000
#define AUDIO_DMA_BUFFER_SIZE 4096
#define AUDIO_DMA_BUFFER_SIZE2 (AUDIO_DMA_BUFFER_SIZE >> 1)

/* SOUNDPIPE */

static sp_data *sp;
static sp_ftbl *ft[12];
static sp_osc *osc;
static sp_osc *osc2;
static sp_tadsr *tadsr;
sp_gain *gain;

sp_blsaw *blsaw;
SPFLOAT adsr_trig = 0;

uint8_t audioReady = 0;

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
uint16_t osc1wave = 0;
uint16_t osc2wave = 2;
uint16_t osc3wave = 1;
uint16_t osc4wave = 2;
uint16_t osc5wave = 1;
uint16_t osc6wave = 2;
uint16_t osc7wave = 1;
uint16_t osc8wave = 2;
uint8_t audioBufferFile[4][AUDIO_DMA_BUFFER_SIZE];
uint16_t *varToUpdate;
uint16_t waveformToUpdate = 0;

int teller1, teller2;

// audio buffers
static int16_t int_bufProcessedOut[AUDIO_DMA_BUFFER_SIZE];
static uint8_t audioOutBuf[AUDIO_DMA_BUFFER_SIZE];

// wavetable browser
int16_t startwave = 0;

// timers
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
uint8_t ppqn = 0;
int8_t sequence[16] = {0,10,2,24,20,2,24,6,3,24,10,4};
int8_t sequence_timer = 0;
int16_t sequence_end_note = 0;
int16_t sequence_next_note = 0;
uint8_t current_midi_note = 50;
uint8_t toggleflag = 0;

// midi
UART_HandleTypeDef uart_config;
uint8_t midicounter=0;
int16_t mididata[3] = {0,-1,-1};
uint8_t rx_byte[1];

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
void drawWavetableSmall(uint16_t wavetableID, uint16_t x, uint16_t starty);
void UART6_Config();
void handle_midi();
static void initTimer(uint16_t period);
void drawStepSeqTopBar();
void drawSequencer();

uint32_t g_ADCValue;
int g_MeasurementNumber;
ADC_HandleTypeDef g_AdcHandle;
DMA_HandleTypeDef  g_DmaHandle;
enum{ ADC_BUFFER_LENGTH = 8192 };
uint32_t g_ADCBuffer[ADC_BUFFER_LENGTH];

uint16_t ADCchannelValues[4] = {0,0,0,0};

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
  
  // config UART for MIDI communication
  UART6_Config();
  
  // init sequencer
  // read first sequencer values
  sequence_next_note = sequence[0];
  sequence_end_note = sequence[1];
  initTimer(208);
  
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
  
  // tbls
  sp_ftbl_create(sp, &ft[0], 600);
  sp_ftbl_create(sp, &ft[1], 600);

  // oscs
  sp_osc_create(&osc);
  sp_osc_create(&osc2);
  
  sp_blsaw_create(&blsaw);
  sp_blsaw_init(sp,blsaw);
      *blsaw->freq = 5;
      *blsaw->amp = 1;
  
  sp_osc_init(sp, osc, ft[0], 0);
  osc->freq = 200;
  osc->amp = 0.5;
  
  sp_osc_init(sp, osc2, ft[1], 0);
  osc2->freq = 250;
  osc2->amp = 0.5;
  
  // fltr
  
  sp_gain_create(&gain);
  sp_gain_init(sp, gain);
  gain->freq=500;
  
  // adsr
  sp_tadsr_create(&tadsr);
  sp_tadsr_init(sp, tadsr);
  tadsr->atk = 0.2;
  tadsr->dec = 0.1;
  tadsr->sus = 0.6;
  tadsr->rel = 0.1;
  
  openSCwaveform(0, osc1wave);
  sp_gen_copy(sp,ft[0],f_wavetable[0]);
  
  openSCwaveform(1, osc2wave);
  sp_gen_copy(sp,ft[1],f_wavetable[1]);
  
  drawVoiceInterface(1);
  
  while (1) {
    // determine audioReady moment
    // reserve 700ms for initialisation
    // the delay needs to match some initialisation timer?
    // somehow this initialisation if different upon reset of coldboot
    // investigate further		
    // --> audioReady is not used in the rest of the program!
    // suspect: compiler stuff?
    if(audioReady==0 && HAL_GetTick()>700) {
      audioReady=1;
    }
    
    // if(HAL_GetTick()%5000==0) {
//       adsr_trig = 1;
//       //char a[] = "";
//       //sprintf(a, "%ld", HAL_GetTick());
//       //BSP_LCD_DisplayStringAt(50, 50, (uint8_t *)a, LEFT_MODE);
//       //adsr_trig = 1;
//     }
      
    
    

    // if(audioReady==0 && HAL_GetTick()>700) {
//       audioReady=1;
//
//       openSCwaveform(0, osc1wave);
//
//       for(int i = 0; i < ft[0]->size; i++){
//           ft[0]->tbl[i] = f_wavetable[0][i];
//       }
//
//       drawVoiceInterface(1);
//
//     }
    
    //BSP_LED_Toggle(LED_GREEN);
    
    //__disable_irq();
    //openSCwaveform(0, osc1wave);
    //__enable_irq();
    
    //if (HAL_ADC_PollForConversion(&g_AdcHandle, 1000000) == HAL_OK)
    //        {
        //        g_ADCValue = HAL_ADC_GetValue(&g_AdcHandle);
      //          g_MeasurementNumber++;
    char a[] = "";
    //uint32_t digu = (int32_t)((ADCchannelValues[2]/5.65)-36);
    // int32_t digu = (int32_t)((gain->freq)*1000);
   //uint16_t newval = g_ADCValue - (uint16_t)*blsaw->freq;
           sprintf(a, "%d", ADCchannelValues[1]);
                           BSP_LCD_DisplayStringAt(50, 50, (uint8_t *)a, LEFT_MODE);
 //   //                                                             osc->freq=(ADCchannelValues[0] / 10);
 // //    // //     }
 //                                //if(g_ADCValue>1000) {
                                //  *blsaw->freq = g_ADCValue;
                                //}
                           BSP_LED_Toggle(LED_GREEN);
    
    //ADCchannelValues[1] / 410;
//     gain->res += 0.0001;
//     if(gain->res > 3.9) gain->res=0;
//
  }

  return 0;
}


void openSCwaveform(uint16_t SCwaveformID, uint16_t filenameID) {
  // open single waveform
  // skip 44 (offset)
  // read 1200 (samplelength = 600 2's complement)
  //char* theFilename = printf("0:single%d.wav", filenameID);
  char theFilename[] = "";
  sprintf(theFilename, "0:wt1%03d.wav", filenameID);
  f_close(&wavetable_file[SCwaveformID]);
  __disable_irq();
  uint16_t openresult = f_open(&wavetable_file[SCwaveformID], theFilename, FA_READ);
  
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

    // to short
    // read 2 values for 2's-complement
    // convert to 1 short value
    short tempshort = (short)(audioBufferFile[0][j+1]<<8 | ((audioBufferFile[0][j]) & 0xFF));

    // to float
    // use j/2 because of conversion from 2's comp to short
    f_wavetable[SCwaveformID][j/2] = (float)(tempshort/32768.f);

    // fill drawtable with 120 value
    if(j%10==0) {
      d_wavetable[SCwaveformID][dt_int] = ((tempshort + 32768) / 656);
      dt_int++;
    }
  }
  
  __enable_irq();
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
  
  //BSP_LED_Toggle(LED_GREEN);
}

void computeAudio() {
    //if(audioReady==1) {
  
  // compute 2048 samples -> 512 audiosamples
  // audiorate
  for(int i = 0; i < 1024; i+=2) {
    SPFLOAT tmp1 = 0, tmp2=0, tadsr_out = 0, tmp3=0, mixOut, osctemp, gainout;
    
    sp_blsaw_compute(sp, blsaw, NULL, &tmp3);
    
    //osctemp = osc->freq;
    //osc->freq = osc->freq + (100*tmp3);
    sp_osc_compute(sp, osc, NULL, &tmp1);
    //osc->freq = osctemp;
    
    //osctemp = osc2->freq;
    //osc2->freq = osc2->freq + (100*tmp3);
    sp_osc_compute(sp, osc2, NULL, &tmp2);
    //osc2->freq = osctemp;

    sp_tadsr_compute(sp, tadsr, &adsr_trig, &tadsr_out);
    
    mixOut = (tadsr_out *  (0.5 * tmp1 + 0.5 * tmp2));
    
    //__disable_irq();
    sp_gain_compute(sp, gain, &mixOut, &gainout);
    sp_gain_compute(sp, gain, &mixOut, &gainout);
    sp_gain_compute(sp, gain, &mixOut, &gainout);
    sp_gain_compute(sp, gain, &mixOut, &gainout);
    
    //__enable_irq();

    // channel outputs in 2's comp / signed int
    int_bufProcessedOut[i] = (gainout * 32767);
    int_bufProcessedOut[i+1] = (gainout * 32767);
    
    // reset oscs
    //osc->freq = osctemp;
  }
    
  // controlrate
  
  // reset adsr trig
  adsr_trig = 0;

  //osc->amp = ADCchannelValues[0] / 16384.f;
  //osc2->amp = ADCchannelValues[1] / 16384.f;
  //int8_t relfreq = (ADCchannelValues[1]/55.5f) - 37;
  //float newfreq = osc->freq * powf(1.059463094359,relfreq);
  //osc2->freq = newfreq;
    
    // char a[] = "";
//     sprintf(a, "%d", relfreq);
//     BSP_LCD_DisplayStringAt(150, 50, (uint8_t *)a, LEFT_MODE);
}
//}

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
    BSP_LCD_FillRect(160,166,67,55);
    BSP_LCD_FillRect(233,166,67,55);
    
    
    BSP_LCD_FillRect(310,50,140,110);
    BSP_LCD_FillRect(310,166,67,55);
    BSP_LCD_FillRect(383,166,67,55);
    
    
    drawWavetable(0,20,155);
    drawWavetable(1,170,155);
  }
}

void drawWaveSelector() {  
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_FillRect(10,30,460,237);
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
  BSP_LCD_FillRect(12,32,456,233);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  
  uint8_t wavetableID = startwave; 
  for (int row=0; row < 3; row++) {
    for (int col=1; col < 6; col++) {    
      drawWavetableSmall(wavetableID,(col*70)+5,45+(row*70));
      wavetableID++;
    } 
  }
  
  // page indicator
  BSP_LCD_SetBackColor(LCD_COLOR_DARKBLUE);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font12);
  char a[] = "";
  sprintf(a, "page %d", (startwave/15)+1);
  BSP_LCD_DisplayStringAt(20, 40, (uint8_t *)a, LEFT_MODE);
  
  BSP_LCD_DisplayStringAt(20, 145, (uint8_t *)"<<", LEFT_MODE);
  BSP_LCD_DisplayStringAt(445, 145, (uint8_t *)">>", LEFT_MODE);
}

void drawWavetable(uint16_t wavetableID, uint16_t x, uint16_t starty) {
  //uint16_t x = xstart;
  //uint16_t starty = ystart;
  uint16_t y=starty-d_wavetable[wavetableID][0];
  for(int i = 0; i < 120; i++) {
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawLine(x,y,x+1,starty-d_wavetable[wavetableID][i]);
    x++;
    y=starty-d_wavetable[wavetableID][i];
  }
}



void drawWavetableSmall(uint16_t wavetableID, uint16_t x, uint16_t starty) {
  uint16_t y = starty + 5 + 50 - sswave[wavetableID][0];
  for (int j=1; j < 50; j++) {
    BSP_LCD_DrawLine(x,y,x+1,starty + 5 + 50 - sswave[wavetableID][j]);
    x++;
    y = starty + 5 + 50 - sswave[wavetableID][j];
  }
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
                varToUpdate = &osc1wave;
                waveformToUpdate = 0;
                touchMap = "waveselect";
                drawWaveSelector();
              }
              //160,50,140,110
              if(touchx > 160 && touchx < 300 && touchy > 50 && touchy < 160) {
                //currentVoiceInEdit = 1;
                //drawInterface();
                varToUpdate = &osc2wave;
                waveformToUpdate = 1;
                touchMap = "waveselect";
                drawWaveSelector();
              }
            }
        
            else if (strcmp(touchMap,"waveselect")==0) {
              
              int8_t wavetableID = 0;
              if(touchx<75) {
                startwave = startwave - 15;
                if(startwave < 0) {startwave=0;}
                drawWaveSelector();
                //break;
              }
              if(touchx>405) {
                startwave = startwave + 15;
                if(startwave > 180) {startwave=180;}
                drawWaveSelector();
                //break;
              }
              for (int row=0; row < 3; row++) {
                for (int col=1; col < 6; col++) {
                  if(touchx > ((col*70)+5) && touchx < ((col*70)+5+50) && touchy > (45+(row*70)) && touchy < (45+50+(row*70))) {
                    // touch event on wavetable, open this one
                    openSCwaveform(waveformToUpdate, startwave + wavetableID);
                    // and copy contents to wavetable ftbl
                    sp_gen_copy(sp,ft[waveformToUpdate],f_wavetable[waveformToUpdate]);
                    // set osc var to new id
                    *varToUpdate = startwave + wavetableID;
                  
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
      // float req_freq = (13.75 * (pow(2,(mididata[1]-9.0)/12.0)));
//       osc->freq = req_freq;
//       osc2->freq = (13.75 * (pow(2,(mididata[1]-12.0-9.0)/12.0)));
//       adsr_trig = 1;
      
      mididata[1] = -1;
    }
  }
  // reset interrupt
  HAL_UART_Receive_IT(&uart_config, rx_byte, 1);
}  
  
void handle_midi() {
  
  // char midia[] = "";
  // sprintf(midia, "%d %d %d", mididata[0],mididata[1],mididata[2]);
  // BSP_LCD_SetFont(&Font12);
  // BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
  // BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  // BSP_LCD_DisplayStringAt(300, 7, (uint8_t *)midia, LEFT_MODE);
  
  
  // note on if vel (mididata[2]) > 0
  if(mididata[0] == 144 && mididata[2] > 0) {
 //
 //    // check voiceallocation for available voice (if voicemode)
 //    // start at the back, end up with lowest available voice
 //    int availableVoice;
 //    if(voiceAllocation[3]==0) {availableVoice=3;}
 //    if(voiceAllocation[2]==0) {availableVoice=2;}
 //    if(voiceAllocation[1]==0) {availableVoice=1;}
 //    if(voiceAllocation[0]==0) {availableVoice=0;}
 //
 //    // reserve voice
 //    voiceAllocation[availableVoice] = mididata[1];
 //
 //    // show voices
 //    char a[] = "";
 //    sprintf(a, "%d %d %d %d", voiceAllocation[0],voiceAllocation[1],voiceAllocation[2],voiceAllocation[3]);
 //    BSP_LCD_SetFont(&Font12);
 //    BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
 //    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
 //    BSP_LCD_DisplayStringAt(200, 7, (uint8_t *)a, LEFT_MODE);
 //
    // calculate and set frequency
    float req_freq = (13.75 * (pow(2,(mididata[1]-9.0)/12.0)));
      osc->freq = req_freq;
      osc2->freq = (13.75 * (pow(2,(mididata[1]+24.0-9.0)/12.0)));
 //ADCchannelValues[2]
 //    // show frequency
 //    char b[10];
 //    int dingus = req_freq * 1000;
 //    snprintf(b, 10, "%d",dingus);
 //    BSP_LCD_SetFont(&Font12);
 //    BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
 //    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
 //    BSP_LCD_DisplayStringAt(7, 7, (uint8_t *)"      ", RIGHT_MODE);
 //    BSP_LCD_DisplayStringAt(7, 7, (uint8_t *)b, RIGHT_MODE);
 //
    // start new note
    adsr_trig = 1;
//    midiNoteOn = 1;
 //    adsr_timer = 0;
 //    voice_frequency[availableVoice] = req_freq;
  }
 //
 //   // implicit note off
 if(mididata[0] == 144 && mididata[2] == 0) {
   adsr_trig = 2;
 }
 //
 //   //float req_freq = (13.75 * (pow(2,(mididata[1]-9.0)/12.0)));
 //
 //   //char b[10];
 //   //int dingus = req_freq * 1000;
 //   //snprintf(b, 10, "%d",dingus);
 //   BSP_LCD_SetFont(&Font12);
 //   BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
 //   BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
 //   BSP_LCD_DisplayStringAt(7, 7, (uint8_t *)"    off", RIGHT_MODE);
 //   //BSP_LCD_DisplayStringAt(7, 7, (uint8_t *)b, RIGHT_MODE);
 //
 //   // reset voice allocation
 //   int availableVoice;
 //   if(voiceAllocation[3]==mididata[1]) {voiceAllocation[3]=0;availableVoice=3;}
 //   if(voiceAllocation[2]==mididata[1]) {voiceAllocation[2]=0;availableVoice=2;}
 //   if(voiceAllocation[1]==mididata[1]) {voiceAllocation[1]=0;availableVoice=1;}
 //   if(voiceAllocation[0]==mididata[1]) {voiceAllocation[0]=0;availableVoice=0;}
 //
 //   // show voices
 //   char a[] = "";
 //   sprintf(a, "%d %d %d %d", voiceAllocation[0],voiceAllocation[1],voiceAllocation[2],voiceAllocation[3]);
 //   BSP_LCD_SetFont(&Font12);
 //   BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
 //   BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
 //   BSP_LCD_DisplayStringAt(200, 7, (uint8_t *)a, LEFT_MODE);
 //
 //   // end this note
 //   midiNoteOn = 0;
 //   voice_frequency[availableVoice] = 0;
 //
 //   //midiNoteOn = 1;
 //   //adsr_timer = 0;
 //   //voice_frequency[0] = req_freq;
 //  }
 //
 //   if(mididata[0] == 176) {
 //     if(mididata[1]==71) {
 //       f_ssample_outChannel_Volume[0] = mididata[2]/127.0;
 //       drawVolumeIndicators();
 //     }
 //     if(mididata[1]==72) {
 //       f_ssample_outChannel_Volume[1] = mididata[2]/127.0;
 //       drawVolumeIndicators();
 //     }
 //     if(mididata[1]==73) {
 //       f_ssample_outChannel_Volume[2] = mididata[2]/127.0;
 //       drawVolumeIndicators();
 //     }
 //     if(mididata[1]==74) {
 //       adsr[0] = mididata[2];
 //     }
 //   }
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
    HAL_NVIC_SetPriority(TIMx_IRQn, 4, 0);
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
    
    // part per quarter note timer
    
    // sequencer
    // start note?
    
    // sequence_timer 0: start, 1: length, 2: note
    // START NOTE
    if(sequence_next_note == ppqn) {
      float req_freq = (13.75 * (pow(2,((current_midi_note + sequence[sequence_timer+2])-9.0)/12.0)));
      osc->freq = req_freq;
      osc2->freq = req_freq; //(13.75 * (pow(2,((current_midi_note + sequence[sequence_timer+2])+((ADCchannelValues[2]/5.65)-36)-9.0)/12.0)));
      adsr_trig = 1;
      
      sequence_timer = sequence_timer + 3;
      if(sequence_timer > 11) {
        sequence_timer = 0;
        sequence_next_note = 0;
      }
      // increase next note with value from sequence
      sequence_next_note = sequence_next_note + sequence[sequence_timer];
      // set endnote
      sequence_end_note = ppqn + sequence[sequence_timer+1];
    }
    
    // END NOTE
    if(sequence_end_note == ppqn) {
      adsr_trig = 2;
    }
    
    // sequencer visualiser
    if(ppqn%24==0) {
      currentStep++;
      if(currentStep>15) {currentStep=0;}
      drawStepSeqTopBar();
    }
  
    ppqn++;
    
    if(ppqn==96) {ppqn=0;}
    // if(ppqn==25) {
//       currentStep++;
//       if(currentStep>15) {currentStep=0;}
//
//       //int8_t midistatus_buffer = mididata[0];
//       //mididata[0] = 144;
//       //mididata[1] = current_midi_note + sequence[currentStep];
//       //mididata[2] = 100;
//       //handle_midi();
//       //mididata[0] = midistatus_buffer;
//       float req_freq = (13.75 * (pow(2,((current_midi_note + sequence[currentStep])-9.0)/12.0)));
//       //__disable_irq();
//         osc->freq = req_freq;
//         osc2->freq = req_freq;
//         adsr_trig = 1;
//       //  __enable_irq();
//
//       //toggleflag = 0;
//       //initNoteOffTimer(5000);
//       //__HAL_TIM_SET_AUTORELOAD(htim,5000);
//       //HAL_NVIC_EnableIRQ(TIMy_IRQn);
//
//       drawStepSeqTopBar();
//       ppqn = 1;
//     }
  }
  if(htim->Instance==TIM4) {
    //if(htim->Instance->RepetitionCounter==0) {
      BSP_LED_Toggle(LED_GREEN);
      //}
      //__HAL_TIM_SET_AUTORELOAD(htim,500);
    
    // disable timer
    __HAL_TIM_DISABLE(htim);
    
    if (toggleflag==1) {
      // send note off
     // __disable_irq();
      adsr_trig = 2;
    //  __enable_irq();
    }
    
    toggleflag = 1;
    
  }
}



// Stuff that should be in external files ;)

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
 
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 1);
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

    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 2);   
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
        uint16_t channels_inbetween_res[4] = {0,0,0,0};
        for(int i=1000;i<1800;i=i+4) {
          channels[0] = channels[0] + g_ADCBuffer[i+0];
          channels[1] = channels[1] + g_ADCBuffer[i+1];
          channels[2] = channels[2] + g_ADCBuffer[i+2];
          channels[3] = channels[3] + g_ADCBuffer[i+3];
        }
      
        channels_inbetween_res[0] = channels[0] / 2000;
        channels_inbetween_res[1] = channels[1] / 2000;
        channels_inbetween_res[2] = channels[2] / 2000;
        channels_inbetween_res[3] = channels[3] / 2000;
        
        if(abs(ADCchannelValues[1]-channels_inbetween_res[1])>1)
          {
            ADCchannelValues[1] = channels_inbetween_res[1];
            gain->freq = ADCchannelValues[1] * 20;
          }
//
          if(abs(ADCchannelValues[2]-channels_inbetween_res[2])>1)
            {
              ADCchannelValues[2] = channels_inbetween_res[2];
              gain->res = ADCchannelValues[2] / 100.0;
            }
        
        //gain->freq = 0.1;

        // if(abs(ADCchannelValues[0]-channels[0])>1) {ADCchannelValues[0] = channels[0];}
        // if(abs(ADCchannelValues[1]-channels[1])>1) {ADCchannelValues[1] = channels[1];}
        // if(abs(ADCchannelValues[2]-channels[2])>1) {ADCchannelValues[2] = channels[2];}
        // if(abs(ADCchannelValues[3]-channels[3])>1) {ADCchannelValues[3] = channels[3];}
      //
        // ADCchannelValues[0] = 200;
    //   ADCchannelValues[1] = 200;
    //   ADCchannelValues[2]= 200;
    //       ADCchannelValues[3]= 200;
      
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
  
  HAL_NVIC_SetPriority(USART6_IRQn,10,1);
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

