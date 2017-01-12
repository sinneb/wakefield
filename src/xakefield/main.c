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

#define ARM_MATH_CM7
#include "arm_math.h"

// defines
#define VOLUME 40
#define SAMPLE_RATE 48000
#define AUDIO_DMA_BUFFER_SIZE 4096  // divided by 4 (C array item length) reserves a stereo audio buffer of 1024 bytes
                                    // translating to 2x512 bytes mono
#define AUDIO_DMA_BUFFER_SIZE2 (AUDIO_DMA_BUFFER_SIZE >> 1)
#define NBR_OF_VOICES 6

/* SOUNDPIPE */

static sp_data *sp;
static sp_tadsr *tadsr[NBR_OF_VOICES];
sp_osc *osc[32];
SPFLOAT adsr_trig[NBR_OF_VOICES] = {0,0,0,0,0,0};
static sp_ftbl *ft;
sp_filts *filts;
float oscSetFreq[32];

uint8_t audioReady = 0;
uint8_t activeOsc = 0;
uint8_t player = 0;

// audio buffers
static int16_t int_bufProcessedOut[AUDIO_DMA_BUFFER_SIZE];
static uint8_t audioOutBuf[AUDIO_DMA_BUFFER_SIZE];

// filter buffers and instances
int filt_numStages=1;
int filt_samples=512;
//float32_t pCoeffs[5] = {0.0159660568168987, 0.0319321136337974, 0.0159660568168987, -1.9112261061185025, 0.9750903333860972};
//float32_t  pCoeffs[5] = {1.0f, -1.0f, 1.0f, 1.0f, -1.0f};
//float32_t pCoeffs[5] = {0.9733553098104358, -1.9467106196208717, 0.9733553098104358, 1.9447880283614938, -0.9486332108802497};
//float32_t pCoeffs[5] = {1,2,1,-1.1997,0.5157};
//The coefficients are stored in the array <code>pCoeffs</code> in the following order:   
//<pre>   
//{b10, b11, b12, a11, a12, b20, b21, b22, a21, a22, ...}   
float32_t pCoeffs[5] = {0.004249833583334715, 0.00849966716666943, 0.004249833583334715, 1.9700326795371683, -0.9870320138705072};
float32_t pState[64];
arm_biquad_cascade_df2T_instance_f32 S;

// midi
UART_HandleTypeDef uart_config;
uint8_t midicounter=0;
int16_t mididata[3] = {0,-1,-1};
uint8_t rx_byte[1];
uint16_t voiceAllocation[NBR_OF_VOICES] = {0,0,0,0,0,0};

extern SAI_HandleTypeDef haudio_out_sai;

// header
void initAudio();
void computeAudio();
void UART6_Config();
void handle_midi();


int main() {
  CPU_CACHE_Enable();
  HAL_Init();
  SystemClock_Config(); 
  
  // config UART for MIDI communication
  UART6_Config();
  
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
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);

  initAudio();
  
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)"xakefield", CENTER_MODE);
  
  sp_create(&sp);
  sp->sr = 48000;
  
  sp_ftbl_create(sp, &ft, 600);
  
  float lewave[600] = {0, 0.010467529296875, 0.02093505859375, 0.031402587890625, 0.0418701171875, 0.052337646484375, 0.062774658203125, 0.0732421875, 0.08367919921875, 0.0941162109375, 0.104522705078125, 0.11492919921875, 0.125335693359375, 0.135711669921875, 0.146087646484375, 0.15643310546875, 0.166778564453125, 0.177093505859375, 0.1873779296875, 0.197662353515625, 0.207916259765625, 0.2181396484375, 0.22833251953125, 0.238525390625, 0.248687744140625, 0.258819580078125, 0.2689208984375, 0.27899169921875, 0.289031982421875, 0.299041748046875, 0.30902099609375, 0.318939208984375, 0.328857421875, 0.338714599609375, 0.34857177734375, 0.358367919921875, 0.36810302734375, 0.377838134765625, 0.38751220703125, 0.397125244140625, 0.40673828125, 0.416259765625, 0.42578125, 0.435211181640625, 0.444610595703125, 0.4539794921875, 0.463287353515625, 0.4725341796875, 0.48175048828125, 0.490875244140625, 0.499969482421875, 0.509033203125, 0.51800537109375, 0.526947021484375, 0.535797119140625, 0.54461669921875, 0.553375244140625, 0.56207275390625, 0.570709228515625, 0.579254150390625, 0.5877685546875, 0.596221923828125, 0.604583740234375, 0.612884521484375, 0.621124267578125, 0.629302978515625, 0.63739013671875, 0.64544677734375, 0.653411865234375, 0.661285400390625, 0.669097900390625, 0.676849365234375, 0.684539794921875, 0.692108154296875, 0.69964599609375, 0.70709228515625, 0.714447021484375, 0.72174072265625, 0.72894287109375, 0.736083984375, 0.743133544921875, 0.750091552734375, 0.7569580078125, 0.763763427734375, 0.770477294921875, 0.777130126953125, 0.783660888671875, 0.790130615234375, 0.7965087890625, 0.80279541015625, 0.808990478515625, 0.815093994140625, 0.821136474609375, 0.827056884765625, 0.8328857421875, 0.838653564453125, 0.84429931640625, 0.849853515625, 0.8553466796875, 0.8607177734375, 0.865997314453125, 0.871185302734375, 0.87628173828125, 0.88128662109375, 0.88616943359375, 0.8909912109375, 0.89569091796875, 0.900299072265625, 0.90478515625, 0.909210205078125, 0.91351318359375, 0.917724609375, 0.921844482421875, 0.92584228515625, 0.92974853515625, 0.933563232421875, 0.937255859375, 0.94085693359375, 0.9443359375, 0.94775390625, 0.951019287109375, 0.9542236328125, 0.957275390625, 0.96026611328125, 0.963134765625, 0.96588134765625, 0.96856689453125, 0.971099853515625, 0.973541259765625, 0.97589111328125, 0.978118896484375, 0.980255126953125, 0.982269287109375, 0.984161376953125, 0.9859619140625, 0.9876708984375, 0.989227294921875, 0.99072265625, 0.992095947265625, 0.99334716796875, 0.994476318359375, 0.99554443359375, 0.9964599609375, 0.997283935546875, 0.99798583984375, 0.99859619140625, 0.99908447265625, 0.999481201171875, 0.999755859375, 0.999908447265625, 0.999969482421875, 0.999908447265625, 0.999755859375, 0.999481201171875, 0.99908447265625, 0.99859619140625, 0.99798583984375, 0.997283935546875, 0.9964599609375, 0.99554443359375, 0.994476318359375, 0.99334716796875, 0.992095947265625, 0.99072265625, 0.989227294921875, 0.9876708984375, 0.9859619140625, 0.984161376953125, 0.982269287109375, 0.980255126953125, 0.978118896484375, 0.97589111328125, 0.973541259765625, 0.971099853515625, 0.96856689453125, 0.96588134765625, 0.963134765625, 0.96026611328125, 0.957275390625, 0.9542236328125, 0.951019287109375, 0.94775390625, 0.9443359375, 0.94085693359375, 0.937255859375, 0.933563232421875, 0.92974853515625, 0.92584228515625, 0.921844482421875, 0.917724609375, 0.91351318359375, 0.909210205078125, 0.90478515625, 0.900299072265625, 0.89569091796875, 0.8909912109375, 0.88616943359375, 0.88128662109375, 0.87628173828125, 0.871185302734375, 0.865997314453125, 0.8607177734375, 0.8553466796875, 0.849853515625, 0.84429931640625, 0.838653564453125, 0.8328857421875, 0.827056884765625, 0.821136474609375, 0.815093994140625, 0.808990478515625, 0.80279541015625, 0.7965087890625, 0.790130615234375, 0.783660888671875, 0.777130126953125, 0.770477294921875, 0.763763427734375, 0.7569580078125, 0.750091552734375, 0.743133544921875, 0.736083984375, 0.72894287109375, 0.72174072265625, 0.714447021484375, 0.70709228515625, 0.69964599609375, 0.692108154296875, 0.684539794921875, 0.676849365234375, 0.669097900390625, 0.661285400390625, 0.653411865234375, 0.64544677734375, 0.63739013671875, 0.629302978515625, 0.621124267578125, 0.612884521484375, 0.604583740234375, 0.596221923828125, 0.5877685546875, 0.579254150390625, 0.570709228515625, 0.56207275390625, 0.553375244140625, 0.54461669921875, 0.535797119140625, 0.526947021484375, 0.51800537109375, 0.509033203125, 0.499969482421875, 0.490875244140625, 0.48175048828125, 0.4725341796875, 0.463287353515625, 0.4539794921875, 0.444610595703125, 0.435211181640625, 0.42578125, 0.416259765625, 0.40673828125, 0.397125244140625, 0.38751220703125, 0.377838134765625, 0.36810302734375, 0.358367919921875, 0.34857177734375, 0.338714599609375, 0.328857421875, 0.318939208984375, 0.30902099609375, 0.299041748046875, 0.289031982421875, 0.27899169921875, 0.2689208984375, 0.258819580078125, 0.248687744140625, 0.238525390625, 0.22833251953125, 0.2181396484375, 0.207916259765625, 0.197662353515625, 0.1873779296875, 0.177093505859375, 0.166778564453125, 0.15643310546875, 0.146087646484375, 0.135711669921875, 0.125335693359375, 0.11492919921875, 0.104522705078125, 0.0941162109375, 0.08367919921875, 0.0732421875, 0.062774658203125, 0.052337646484375, 0.0418701171875, 0.031402587890625, 0.02093505859375, 0.010467529296875, 0.0, -0.010467529296875, -0.02093505859375, -0.031402587890625, -0.0418701171875, -0.052337646484375, -0.062774658203125, -0.0732421875, -0.08367919921875, -0.0941162109375, -0.104522705078125, -0.11492919921875, -0.125335693359375, -0.135711669921875, -0.146087646484375, -0.15643310546875, -0.166778564453125, -0.177093505859375, -0.1873779296875, -0.197662353515625, -0.207916259765625, -0.2181396484375, -0.22833251953125, -0.238525390625, -0.248687744140625, -0.258819580078125, -0.2689208984375, -0.27899169921875, -0.289031982421875, -0.299041748046875, -0.30902099609375, -0.318939208984375, -0.328857421875, -0.338714599609375, -0.34857177734375, -0.358367919921875, -0.36810302734375, -0.377838134765625, -0.38751220703125, -0.397125244140625, -0.40673828125, -0.416259765625, -0.42578125, -0.435211181640625, -0.444610595703125, -0.4539794921875, -0.463287353515625, -0.4725341796875, -0.48175048828125, -0.490875244140625, -0.499969482421875, -0.509033203125, -0.51800537109375, -0.526947021484375, -0.535797119140625, -0.54461669921875, -0.553375244140625, -0.56207275390625, -0.570709228515625, -0.579254150390625, -0.5877685546875, -0.596221923828125, -0.604583740234375, -0.612884521484375, -0.621124267578125, -0.629302978515625, -0.63739013671875, -0.64544677734375, -0.653411865234375, -0.661285400390625, -0.669097900390625, -0.676849365234375, -0.684539794921875, -0.692108154296875, -0.69964599609375, -0.70709228515625, -0.714447021484375, -0.72174072265625, -0.72894287109375, -0.736083984375, -0.743133544921875, -0.750091552734375, -0.7569580078125, -0.763763427734375, -0.770477294921875, -0.777130126953125, -0.783660888671875, -0.790130615234375, -0.7965087890625, -0.80279541015625, -0.808990478515625, -0.815093994140625, -0.821136474609375, -0.827056884765625, -0.8328857421875, -0.838653564453125, -0.84429931640625, -0.849853515625, -0.8553466796875, -0.8607177734375, -0.865997314453125, -0.871185302734375, -0.87628173828125, -0.88128662109375, -0.88616943359375, -0.8909912109375, -0.89569091796875, -0.900299072265625, -0.90478515625, -0.909210205078125, -0.91351318359375, -0.917724609375, -0.921844482421875, -0.92584228515625, -0.92974853515625, -0.933563232421875, -0.937255859375, -0.94085693359375, -0.9443359375, -0.94775390625, -0.951019287109375, -0.9542236328125, -0.957275390625, -0.96026611328125, -0.963134765625, -0.96588134765625, -0.96856689453125, -0.971099853515625, -0.973541259765625, -0.97589111328125, -0.978118896484375, -0.980255126953125, -0.982269287109375, -0.984161376953125, -0.9859619140625, -0.9876708984375, -0.989227294921875, -0.99072265625, -0.992095947265625, -0.99334716796875, -0.994476318359375, -0.99554443359375, -0.9964599609375, -0.997283935546875, -0.99798583984375, -0.99859619140625, -0.99908447265625, -0.999481201171875, -0.999755859375, -0.999908447265625, -0.999969482421875, -0.999908447265625, -0.999755859375, -0.999481201171875, -0.99908447265625, -0.99859619140625, -0.99798583984375, -0.997283935546875, -0.9964599609375, -0.99554443359375, -0.994476318359375, -0.99334716796875, -0.992095947265625, -0.99072265625, -0.989227294921875, -0.9876708984375, -0.9859619140625, -0.984161376953125, -0.982269287109375, -0.980255126953125, -0.978118896484375, -0.97589111328125, -0.973541259765625, -0.971099853515625, -0.96856689453125, -0.96588134765625, -0.963134765625, -0.96026611328125, -0.957275390625, -0.9542236328125, -0.951019287109375, -0.94775390625, -0.9443359375, -0.94085693359375, -0.937255859375, -0.933563232421875, -0.92974853515625, -0.92584228515625, -0.921844482421875, -0.917724609375, -0.91351318359375, -0.909210205078125, -0.90478515625, -0.900299072265625, -0.89569091796875, -0.8909912109375, -0.88616943359375, -0.88128662109375, -0.87628173828125, -0.871185302734375, -0.865997314453125, -0.8607177734375, -0.8553466796875, -0.849853515625, -0.84429931640625, -0.838653564453125, -0.8328857421875, -0.827056884765625, -0.821136474609375, -0.815093994140625, -0.808990478515625, -0.80279541015625, -0.7965087890625, -0.790130615234375, -0.783660888671875, -0.777130126953125, -0.770477294921875, -0.763763427734375, -0.7569580078125, -0.750091552734375, -0.743133544921875, -0.736083984375, -0.72894287109375, -0.72174072265625, -0.714447021484375, -0.70709228515625, -0.69964599609375, -0.692108154296875, -0.684539794921875, -0.676849365234375, -0.669097900390625, -0.661285400390625, -0.653411865234375, -0.64544677734375, -0.63739013671875, -0.629302978515625, -0.621124267578125, -0.612884521484375, -0.604583740234375, -0.596221923828125, -0.5877685546875, -0.579254150390625, -0.570709228515625, -0.56207275390625, -0.553375244140625, -0.54461669921875, -0.535797119140625, -0.526947021484375, -0.51800537109375, -0.509033203125, -0.499969482421875, -0.490875244140625, -0.48175048828125, -0.4725341796875, -0.463287353515625, -0.4539794921875, -0.444610595703125, -0.435211181640625, -0.42578125, -0.416259765625, -0.40673828125, -0.397125244140625, -0.38751220703125, -0.377838134765625, -0.36810302734375, -0.358367919921875, -0.34857177734375, -0.338714599609375, -0.328857421875, -0.318939208984375, -0.30902099609375, -0.299041748046875, -0.289031982421875, -0.27899169921875, -0.2689208984375, -0.258819580078125, -0.248687744140625, -0.238525390625, -0.22833251953125, -0.2181396484375, -0.207916259765625, -0.197662353515625, -0.1873779296875, -0.177093505859375, -0.166778564453125, -0.15643310546875, -0.146087646484375, -0.135711669921875, -0.125335693359375, -0.11492919921875, -0.104522705078125, -0.0941162109375, -0.08367919921875, -0.0732421875, -0.062774658203125, -0.052337646484375, -0.0418701171875, -0.031402587890625, -0.02093505859375, -0.010467529296875};
  //sp_gen_composite(sp, ft, "0.5 0.5 270 0.5
  
  for(int i = 0; i < ft->size; i++){
      ft->tbl[i] = lewave[i];
  }
  
  // sp_osc_create(&osc[0]);
  // sp_osc_init(sp, osc[0], ft, 0);
  // osc[0]->freq = 200;
  // osc[0]->amp = 0.5;
  
  for(int i = 0; i < 24; i++) {
    // sp_blsaw_create(&blsaw[i]);
    // sp_blsaw_init(sp,blsaw[i]);

    sp_osc_create(&osc[i]);


    //sp_osc_create(&osc);

    //*blsaw[i]->freq = 110 + (40*i);
    // *blsaw[i]->freq = 55 + rand() % 950;
    // *blsaw[i]->amp = 1;

    sp_osc_init(sp, osc[i], ft, 0);
    osc[i]->freq = 220 + (40*i);
    osc[i]->amp = 0.5;
    oscSetFreq[i] = osc[i]->freq;
  }
  
  osc[2]->freq=220;
  osc[2]->amp=1;
  
  osc[3]->freq=0.1;
  osc[3]->amp=1;
  // fltr
  // fltr
  
  sp_filts_create(&filts);
  sp_filts_init(sp, filts);
  filts->freq=1000;
  filts->res=0;
  
  // init filter
  arm_biquad_cascade_df2T_init_f32(&S, filt_numStages, (float32_t *)&pCoeffs[0], &pState[0]);
  
  // sp_gain_create(&gain);
//   sp_gain_init(sp, gain);
//   gain->freq=0;
//   gain->res=1;
  
  for(int i = 0; i < NBR_OF_VOICES; i++) {

    // adsr
    sp_tadsr_create(&tadsr[i]);
    sp_tadsr_init(sp, tadsr[i]);
    tadsr[i]->atk = 0.5;
    tadsr[i]->dec = 0.2;
    tadsr[i]->sus = 0.6;
    tadsr[i]->rel = 0.8;
  }
  
  while (1) {
    BSP_LED_Toggle(LED_GREEN);
    char a[] = "";
    sprintf(a, "%ld", HAL_GetTick());
    BSP_LCD_DisplayStringAt(50, 50, (uint8_t *)a, LEFT_MODE);
      
      //delay(10);
    if(HAL_GetTick()%500==0) {
      pCoeffs[0]+=0.001;
      if(player==0) {
        mididata[0] = 144; mididata[1] = 60; mididata[2] = 100;
        handle_midi();
        mididata[0] = 0; mididata[1] = -1; mididata[2] = -1;
        player = 1;
      } // else {
//         mididata[0] = 144; mididata[1] = 60; mididata[2] = 0;
//         handle_midi();
//         mididata[0] = 0; mididata[1] = -1; mididata[2] = -1;
//         player = 0;
//       }    
    }
  }

  return 0;
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
}

void doVoice(int voiceID, int action, float thevalue) {
  int startosc = 4 * voiceID;
  
  // amp = 1
  if (action==1) {
    for(int i=startosc;i<startosc+4;i++) {
      osc[i]->amp = thevalue;
    }
  }
  
  // freq = 2
  if (action==2) {
    for(int i=startosc;i<startosc+4;i++) {
      float newfreq = (13.75 * (pow(2,(mididata[1]-9.0-(7*(i-startosc)))/12.0)));
      osc[i]->freq = newfreq;
    }
  }
}

void computeAudio() {

  // compute 1024 samples -> 512 audiosamples, 512 mono to 1024 "stereo"
  // halfbuffer @ audiorate
  
  // 16 buffers with 1024 samples
  float32_t oscbuf[32][1024];
  float32_t oscmixdown[1024];
  float tadsr_out[NBR_OF_VOICES];
  float filtout;
  
  //for(int sampleit=0;sampleit<1024;sampleit+=2) {
  for(int sampleit=0;sampleit<512;sampleit++) {
    
    for(int i = 0; i < NBR_OF_VOICES; i++) {
      // calculate voice ADSR
      sp_tadsr_compute(sp, tadsr[i], &adsr_trig[i], &tadsr_out[i]);
      // set voice amp value
      doVoice(i,1,(tadsr_out[i] / 24));
    }
    
    // calculate 1 monosample for each oscillator for all voices, 6 voices * 4 osc = 24 
    for(int oscit=0; oscit<2; oscit++) {
      sp_osc_compute(sp, osc[oscit], NULL, &oscbuf[oscit][sampleit]);
      
      // apply filter settings
      //sp_filts_compute(sp, filts, &oscbuf[oscit][sampleit], &filtout);
      
      // double the sample to generate "stereo"
      //oscbuf[oscit][sampleit+1] = oscbuf[oscit][sampleit];
    }

    // handle ADSR's
    // non negative (fabsf)
    //osc[2]->amp = fabsf(oscbuf[3][sampleit]);
    
    //osc[2]->amp = tadsr_out; 
    //sp_tadsr_compute(sp, tadsr, &adsr_trig, &tadsr_out);
    //osc[2]->amp = tadsr_out; 
    
    // handle frequency modulation
    //osc[0]->freq = oscSetFreq[0] + (oscbuf[2][sampleit] * 600);
    
  }
  
  // multiply oscbuf[0] with oscbuf[2]
  //arm_mult_f32(oscbuf[0], oscbuf[2], oscbuf[0], 1024);
  
  //add all buffers together
  for(int oscit=1; oscit<2; oscit++) {
    arm_add_f32(oscbuf[0],oscbuf[oscit],oscbuf[0],512);
  }
  
  // filter resulting buffer oscbuf[0]
  
  // calculate filter 
  arm_biquad_cascade_df2T_f32(&S, oscbuf[0], oscbuf[0], 512);
//
  // calculate filter

  // double array mono -> stereo
  int counter = 0;
  for(int sampleit=0;sampleit<1024;sampleit+=2) {
    oscmixdown[sampleit] = oscbuf[0][counter];
    oscmixdown[sampleit+1] = oscbuf[0][counter];
    counter++;
  }
  
  // for(int sampleit=0;sampleit<1024;sampleit+=2) {
  //   sp_filts_compute(sp, filts, &oscbuf[0][sampleit], &filtout);
  //   oscbuf[0][sampleit] = filtout;
  //   oscbuf[0][sampleit+1] = oscbuf[0][sampleit];
  // }
  
  
  // first addition to populate oscmixdown
  //arm_add_f32(oscbuf[0], oscbuf[1], oscbuf[0], 1024);
  
  // add the rest of the osc chuncks 
  // for(int oscit=2; oscit<16; oscit++) {
//     arm_add_f32(oscmixdown, oscbuf[oscit], oscmixdown, 1024);
//   }
  
  
  //osc[2]->amp = tadsr_out
  
  // convert float to q15 / int16
  // send to transferbuffer
  arm_float_to_q15(oscmixdown, int_bufProcessedOut, 1024);

  // for(int i = 0; i < 1024; i+=2) {
//     //SPFLOAT tmp3=0, tmp4=0, tmp[12], mixOut, gainout, oscmix;
//     float tmp[24];
//     // sp_blsaw_compute(sp, blsaw[0], NULL, &tmp3);
// //     oscmix = 0.05 * tmp3;
// //
//     for(int i = 1; i < 16; i++) {
//       //sp_blsaw_compute(sp, blsaw[i], NULL, &tmp[i]);
//       //sp_blsaw_compute(sp, blsaw[i+1], NULL, &tmp4);
//       //oscmix  += 0.05 * tmp[i];
//       //oscmix += 0.2 * tmp3;
//       sp_osc_compute(sp, osc[i], NULL, &tmp[i]);
//     }
//     //osctemp = osc->freq;
//     //osc->freq = osc->freq + (100*tmp3);
//     //sp_osc_compute(sp, osc[0], NULL, &tmp1);
//     //osc->freq = osctemp;
//
//     //osctemp = osc2->freq;
//     //osc2->freq = osc2->freq + (100*tmp3);
//     //sp_osc_compute(sp, osc2, NULL, &tmp2);
//     //osc2->freq = osctemp;
//
//     //sp_tadsr_compute(sp, tadsr, &adsr_trig, &tadsr_out);
//
//     // sp_gain_compute(sp, gain, &oscmix, &gainout);
//
//     float mixOut = tmp[2]; //(gainout);
//
//     // channel outputs in 2's comp / signed int
//     int_bufProcessedOut[i] = (mixOut * 32767);
//     int_bufProcessedOut[i+1] = (mixOut * 32767);
//
//     // reset oscs
//     //osc->freq = osctemp;
//
//     // if(audioReady==0) {
// //       gain->freq+=0.5;
// //     } else {
// //       gain->freq-=0.5;
// //     }
//
//   }
  
  //sp_gain_compute_coeffs(sp,gain);
  
  // if(gain->freq>20000) {audioReady = 1;}
//   if(gain->freq<10) {
//     audioReady= 0;
//     for(int i = 0; i < 8; i++) {
//       *blsaw[i]->freq = 55 + rand() % 400;
//     }
//   }
}


// UART (MIDI) HANDLING --------------------------------------------

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)"le midi", CENTER_MODE);
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
    // check voiceallocation for available voice (if voicemode)
    // start at the back, end up with lowest available voice
    int availableVoice;
    if(voiceAllocation[5]==0) {availableVoice=5;}
    if(voiceAllocation[4]==0) {availableVoice=4;}
    if(voiceAllocation[3]==0) {availableVoice=3;}
    if(voiceAllocation[2]==0) {availableVoice=2;}
    if(voiceAllocation[1]==0) {availableVoice=1;}
    if(voiceAllocation[0]==0) {availableVoice=0;}
 //
    // reserve voice
    voiceAllocation[availableVoice] = mididata[1];
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
   //float req_freq = (13.75 * (pow(2,(mididata[1]-9.0)/12.0)));
   // set voice freq
   doVoice(availableVoice,2,mididata[1]);
   //osc[0]->freq = req_freq;
   //osc[1]->freq = req_freq + 100;
      //osc2->freq = (13.75 * (pow(2,(mididata[1]+24.0-9.0)/12.0)));
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
    adsr_trig[availableVoice] = 1;
//    midiNoteOn = 1;
 //    adsr_timer = 0;
 //    voice_frequency[availableVoice] = req_freq;
  }
 //
  
 //   // implicit note off
  // or note off
 if((mididata[0] == 144 && mididata[2] == 0)  || (mididata[0] == 128)) {
   
   //}
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
   int availableVoice;
   if(voiceAllocation[5]==mididata[1]) {voiceAllocation[5]=0;availableVoice=5;}
   if(voiceAllocation[4]==mididata[1]) {voiceAllocation[4]=0;availableVoice=4;}
   if(voiceAllocation[3]==mididata[1]) {voiceAllocation[3]=0;availableVoice=3;}
   if(voiceAllocation[2]==mididata[1]) {voiceAllocation[2]=0;availableVoice=2;}
   if(voiceAllocation[1]==mididata[1]) {voiceAllocation[1]=0;availableVoice=1;}
   if(voiceAllocation[0]==mididata[1]) {voiceAllocation[0]=0;availableVoice=0;}
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
   adsr_trig[availableVoice] = 2;
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

