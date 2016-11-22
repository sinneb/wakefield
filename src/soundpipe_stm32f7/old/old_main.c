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

uint8_t currentLCDcolor = 0;
const uint32_t LCDColorarray[] = { LCD_COLOR_YELLOW, LCD_COLOR_GREEN, LCD_COLOR_ORANGE, LCD_COLOR_MAGENTA };

#define VOLUME 60
#define SAMPLE_RATE 96000
#define AUDIO_DMA_BUFFER_SIZE 4096
#define AUDIO_DMA_BUFFER_SIZE2 (AUDIO_DMA_BUFFER_SIZE >> 1)
#define AUDIO_DMA_BUFFER_SIZE4 (AUDIO_DMA_BUFFER_SIZE >> 2)
#define AUDIO_DMA_BUFFER_SIZE8 (AUDIO_DMA_BUFFER_SIZE >> 3)

#define BAUDRATE              31250
#define TXPIN                 GPIO_PIN_6
#define RXPIN                 GPIO_PIN_7
#define DATAPORT              GPIOC
#define UART_PRIORITY         6
#define UART_RX_SUBPRIORITY   0
#define MAXCLISTRING          100 // Biggest string the user will type

uint8_t rxBuffer = '\000'; // where we store that one character that just came in
uint8_t rxString[MAXCLISTRING]; // where we build our string from characters coming in
int rxindex = 0; // index for going though rxString

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

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

float f_ssample[4][600];
uint8_t active_ssample = 1;
// max 32 cycle = 32*600 = 19200
float f_ssample_freq_specific[19200];
float f_ssample_outChannel[4][600];
float f_ssample_outChannel_Volume[4] = {0.6, 0.6, 0.6, 0.6};
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

float getBPM(FIL *audio_file);
static void init_after_USB();
void drawInterface();
static void initAudio();
static void stop_playback();
void computeAudio();
void computeVoice(int16_t freq);
void computeOscillatorOut(uint8_t ssampleBufferID, uint8_t channelID, uint16_t samplesPerTransfer);
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
  
  __GPIOC_CLK_ENABLE();
  __USART6_CLK_ENABLE();
  __DMA2_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = TXPIN | RXPIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(DATAPORT, &GPIO_InitStruct);
  


  huart1.Instance = USART6;
  huart1.Init.BaudRate = BAUDRATE;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  HAL_UART_Init(&huart1);
  
  hdma_usart1_rx.Instance = DMA2_Stream2;
  hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
  hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart1_rx.Init.MemInc = DMA_MINC_DISABLE;
  hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
  hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_usart1_rx);

  __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, UART_PRIORITY, UART_RX_SUBPRIORITY);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  
  __HAL_UART_FLUSH_DRREGISTER(&huart1);
  HAL_UART_Receive_DMA(&huart1, &rxBuffer, 1);
  
  //Init LCD and Touchscreen
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

void DMA2_Stream2_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream2_IRQn);
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
    BSP_LED_Toggle(LED_GREEN);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_FLUSH_DRREGISTER(&huart1); // Clear the buffer to prevent overrun

    int i = 0;
    
    BSP_LED_Toggle(LED_GREEN);

    //print(&rxBuffer); // Echo the character that caused this callback so the user can see what they are typing
    

    if (rxBuffer == 8 || rxBuffer == 127) // If Backspace or del
    {
        //print(" \b"); // "\b space \b" clears the terminal character. Remember we just echoced a \b so don't need another one here, just space and \b
        rxindex--; 
        if (rxindex < 0) rxindex = 0;
    }

    else if (rxBuffer == '\n' || rxBuffer == '\r') // If Enter
    {
        //executeSerialCommand(rxString);
        rxString[rxindex] = 0;
        rxindex = 0;
        for (i = 0; i < MAXCLISTRING; i++) rxString[i] = 0; // Clear the string buffer
    }

    else
    {
        rxString[rxindex] = rxBuffer; // Add that character to the string
        rxindex++;
        if (rxindex > MAXCLISTRING) // User typing too much, we can't have commands that big
        {
            rxindex = 0;
            for (i = 0; i < MAXCLISTRING; i++) rxString[i] = 0; // Clear the string buffer
            //print("\r\nConsole> ");
        }
    }
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
  
  // open single waveform
  // skip 44 (offset)
  // read 1200 (samplelength = 600 2's complement)
  if (f_open(&ssample[0], "0:single1.wav", FA_READ) == FR_OK) {
	// f_open ok
    f_lseek(&ssample[0], 44);
		//char a[] = "";
		//sprintf(a, "%ld", f_size(&ssample));
    //BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)a, CENTER_MODE);
  } else {
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                              (uint8_t *)"Load error ssample", CENTER_MODE);
    Error_Handler();
  }
  
	// read chunks from USB
  // single use of audiobuffer 0, will be reused in context later on
  // single use of bytes_read 0
	f_read(&ssample[0], audioBufferFile[0], 1200, &bytes_read[0]);

	// 2's-complement signed integers -> short (-32k -> +32k) -> float (-1 -> +1)
	for (int j=0; j < 1200; j=j+2) {
	
		// to short
		short lefty = (short)((audioBufferFile[0][j+1])<<8 | ((audioBufferFile[0][j]) & 0xFF));
		//short righty = (short)((audioBufferFile[i][j+3])<<8 | ((audioBufferFile[i][j+2]) & 0xFF));
	
		// to float
		f_ssample[0][j/2] = ((float)lefty/32768);
		//f_ssample[0][j/4] = ((float)righty/32768);
  }
  
  // 2
  if (f_open(&ssample[1], "0:single2.wav", FA_READ) == FR_OK) {
	  f_lseek(&ssample[1], 44);
	} else {
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                              (uint8_t *)"Load error ssample 2", CENTER_MODE);
    Error_Handler();
  }
  f_read(&ssample[1], audioBufferFile[0], 1200, &bytes_read[0]);
	for (int j=0; j < 1200; j=j+2) {
		short lefty = (short)((audioBufferFile[0][j+1])<<8 | ((audioBufferFile[0][j]) & 0xFF));
		f_ssample[1][j/2] = ((float)lefty/32768);
  }
  
  // 3
  if (f_open(&ssample[2], "0:single3.wav", FA_READ) == FR_OK) {
	  f_lseek(&ssample[2], 44);
	} else {
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                              (uint8_t *)"Load error ssample 3", CENTER_MODE);
    Error_Handler();
  }
  f_read(&ssample[2], audioBufferFile[0], 1200, &bytes_read[0]);
	for (int j=0; j < 1200; j=j+2) {
		short lefty = (short)((audioBufferFile[0][j+1])<<8 | ((audioBufferFile[0][j]) & 0xFF));
		f_ssample[2][j/2] = ((float)lefty/32768);
  }
  
  
  // 4
  if (f_open(&ssample[3], "0:single4.wav", FA_READ) == FR_OK) {
	  f_lseek(&ssample[3], 44);
	} else {
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                              (uint8_t *)"Load error ssample 4", CENTER_MODE);
    Error_Handler();
  }
  f_read(&ssample[3], audioBufferFile[0], 1200, &bytes_read[0]);
	for (int j=0; j < 1200; j=j+2) {
		short lefty = (short)((audioBufferFile[0][j+1])<<8 | ((audioBufferFile[0][j]) & 0xFF));
		f_ssample[3][j/2] = ((float)lefty/32768);
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
  
  computeVoice(400);
  
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
		resright[k/4] = (short)(f_bufPost_mixdown_right[k/4] * 32768);
		// to 2's-comp
		int_bufProcessedOut[k+1] = resleft[k/4]>>8;
		int_bufProcessedOut[k] = resleft[k/4]&0xff;
		int_bufProcessedOut[k+3] = resright[k/4]>>8;
		int_bufProcessedOut[k+2] = resright[k/4]&0xff;
		
	}
}

void computeVoice(int16_t freq) {
  //compute oscillator outputs for note
  computeOscillatorOut(0,0, 350);
  computeOscillatorOut(1,1, 4000);
  computeOscillatorOut(2,2, 449);
  computeOscillatorOut(3,3, 5000);
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
          //f_lseek(&audio_file[0], 0);
          f_ssample_outChannel_Volume[0] += 0.1;
            if(f_ssample_outChannel_Volume[0] > 0.6) {f_ssample_outChannel_Volume[0]=0;}
        }
        if(touchx > 120 && touchx < 240) {
          //f_lseek(&audio_file[1], 0);
          f_ssample_outChannel_Volume[1] += 0.1;
            if(f_ssample_outChannel_Volume[1] > 0.6) {f_ssample_outChannel_Volume[1]=0;}
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

      }
			
			runOnce = 1;
		}
	// read state and continue with while
	BSP_TS_GetState(&rawTouchState);
	} // end while
}