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

char audioFilename1[] = "0:sound.wav";
char audioFilename2[] = "0:sound2.wav";
char audioFilename3[] = "0:sound3.wav";
char audioFilename4[] = "0:sound4.wav";
//char audioFilename[4][50];
//audioFilename[][0] = "0:sound.wav";
//char audioFilenames[5][100];
//audioFilenames[0] = "0:sound.wav";

static uint8_t int_bufProcessedOut[AUDIO_DMA_BUFFER_SIZE];
static uint8_t audioOutBuf[AUDIO_DMA_BUFFER_SIZE];

static uint8_t audioBufFile1[AUDIO_DMA_BUFFER_SIZE];
static uint8_t audioBufFile2[AUDIO_DMA_BUFFER_SIZE];
static uint8_t audioBufFile3[AUDIO_DMA_BUFFER_SIZE];
static uint8_t audioBufFile4[AUDIO_DMA_BUFFER_SIZE];

uint16_t taptempo[4];
uint8_t taptempocounter;
uint32_t taptempo_prev = 0;
float global_tempo = 100;

uint32_t prevTick = 0;

float f_bufPre_left[1024];
float f_bufPre_right[1024];
float f_bufPost_left[1024];
float f_bufPost_right[1024];

float f_bufPre2_left[1024];
float f_bufPre2_right[1024];
float f_bufPost2_left[1024];
float f_bufPost2_right[1024];

float f_bufPre3_left[1024];
float f_bufPre3_right[1024];
float f_bufPost3_left[1024];
float f_bufPost3_right[1024];

float f_bufPre4_left[1024];
float f_bufPre4_right[1024];
float f_bufPost4_left[1024];
float f_bufPost4_right[1024];

float f_bufPost_mixdown_left[1024];
float f_bufPost_mixdown_right[1024];

short resleft[512];
short resright[512];

extern HCD_HandleTypeDef hhcd;
extern USBH_HandleTypeDef hUSBH;
extern SAI_HandleTypeDef haudio_out_sai;

FIL audio_file[4];
UINT bytes_read;
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
    if (f_open(&audio_file[0], audioFilename1, FA_READ) == FR_OK) {
		// f_open ok
      f_lseek(&audio_file[0], f_size(&audio_file[0]));
    } else {
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                                (uint8_t *)"ERROR", CENTER_MODE);
      Error_Handler();
    }
	
    if (f_open(&audio_file[1], audioFilename2, FA_READ) == FR_OK) {
		// f_open ok
      f_lseek(&audio_file[1], f_size(&audio_file[1]));
    } else {
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                                (uint8_t *)"ERROR2", CENTER_MODE);
      Error_Handler();
    }

        if (f_open(&audio_file[2], audioFilename3, FA_READ) == FR_OK) {
    // f_open ok
          f_lseek(&audio_file[2], f_size(&audio_file[2]));
        } else {
            BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                                    (uint8_t *)"ERROR3", CENTER_MODE);
          Error_Handler();
        }
        if (f_open(&audio_file[3], audioFilename4, FA_READ) == FR_OK) {
    // f_open ok
          f_lseek(&audio_file[3], f_size(&audio_file[3]));
        } else {
            BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                                    (uint8_t *)"ERROR4", CENTER_MODE);
          Error_Handler();
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
	
	
	if(bytes_read<2048) {
		//memset(audioOutBuf, 0, sizeof audioOutBuf);
		memset(audioBufFile1, 0, sizeof audioBufFile1);
		//memset(audioBufFile2, 0, sizeof audioBufFile2);
	}
	if(bytes_read2<2048) {
		//memset(audioOutBuf, 0, sizeof audioOutBuf);
		//memset(audioBufFile1, 0, sizeof audioBufFile1);
		memset(audioBufFile2, 0, sizeof audioBufFile2);
	}
	if(bytes_read3<2048) {
		//memset(audioOutBuf, 0, sizeof audioOutBuf);
		memset(audioBufFile3, 0, sizeof audioBufFile3);
		//memset(audioBufFile2, 0, sizeof audioBufFile2);
	}
	if(bytes_read4<2048) {
		//memset(audioOutBuf, 0, sizeof audioOutBuf);
		//memset(audioBufFile1, 0, sizeof audioBufFile1);
		memset(audioBufFile4, 0, sizeof audioBufFile4);
	}
}

void computeAudio() {
	
	float rate = global_tempo / 100;
	if (rate > 2) rate = 2;
	short bufsize = (short)(2048 * rate);
	// garantue a multiple of 4
	bufsize -= bufsize%4;
	
  // clear global buffers
  memset(audioBufFile1, 0, sizeof audioBufFile1);
  memset(audioBufFile2, 0, sizeof audioBufFile2);
  memset(audioBufFile3, 0, sizeof audioBufFile3);
  memset(audioBufFile4, 0, sizeof audioBufFile4);
  
	// read chunks from USB
	f_read(&audio_file[0], audioBufFile1, bufsize, &bytes_read);
	f_read(&audio_file[1], audioBufFile2, bufsize, &bytes_read2);
	f_read(&audio_file[2], audioBufFile3, bufsize, &bytes_read3);
	f_read(&audio_file[3], audioBufFile4, bufsize, &bytes_read4);
	
	// 2's-complement signed integers -> short (-32k -> +32k) -> float (-1 -> +1)
	for (int j=0; j < bufsize; j=j+4) {
		
		// to short
		short lefty = (short) (audioBufFile1[j+1]<<8 | (audioBufFile1[j] & 0xFF));
		short righty = (short) (audioBufFile1[j+3]<<8 | (audioBufFile1[j+2] & 0xFF));
		
		short lefty2 = (short) (audioBufFile2[j+1]<<8 | (audioBufFile2[j] & 0xFF));
		short righty2 = (short) (audioBufFile2[j+3]<<8 | (audioBufFile2[j+2] & 0xFF));
    
		short lefty3 = (short) (audioBufFile3[j+1]<<8 | (audioBufFile3[j] & 0xFF));
		short righty3 = (short) (audioBufFile3[j+3]<<8 | (audioBufFile3[j+2] & 0xFF));
    
		short lefty4 = (short) (audioBufFile4[j+1]<<8 | (audioBufFile4[j] & 0xFF));
		short righty4 = (short) (audioBufFile4[j+3]<<8 | (audioBufFile4[j+2] & 0xFF));
		//shortleft[j/4] = lefty;
		//shortright[j/4] = righty;
		
		// to float
		f_bufPre_left[j/4] = ((float)lefty/32768);
		f_bufPre_right[j/4] = ((float)righty/32768);
		
		f_bufPre2_left[j/4] = ((float)lefty2/32768);
		f_bufPre2_right[j/4] = ((float)righty2/32768);
    
		f_bufPre3_left[j/4] = ((float)lefty3/32768);
		f_bufPre3_right[j/4] = ((float)righty3/32768);
    
		f_bufPre4_left[j/4] = ((float)lefty4/32768);
		f_bufPre4_right[j/4] = ((float)righty4/32768);
}

	
	inter1parray( f_bufPre_left, bufsize / 4, f_bufPost_left, 512 );
	inter1parray( f_bufPre_right, bufsize / 4, f_bufPost_right, 512 );
	inter1parray( f_bufPre2_left, bufsize / 4, f_bufPost2_left, 512 );
	inter1parray( f_bufPre2_right, bufsize / 4, f_bufPost2_right, 512 );
	inter1parray( f_bufPre3_left, bufsize / 4, f_bufPost3_left, 512 );
	inter1parray( f_bufPre3_right, bufsize / 4, f_bufPost3_right, 512 );
	inter1parray( f_bufPre4_left, bufsize / 4, f_bufPost4_left, 512 );
	inter1parray( f_bufPre4_right, bufsize / 4, f_bufPost4_right, 512 );
	
  
  
  
	// the mix
	// float to short
	for (int k=0; k<2048; k=k+4) {
    
    // mix
    f_bufPost_mixdown_left[k/4] = (0.8 * f_bufPost_left[k/4])
                          + (0.8 * f_bufPost2_left[k/4])
                          + (0.8 * f_bufPost3_left[k/4])
                          + (0.8 * f_bufPost4_left[k/4]);
    f_bufPost_mixdown_right[k/4] = (0.8 * f_bufPost_right[k/4])
                          + (0.8 * f_bufPost2_right[k/4])
                          + (0.8 * f_bufPost3_right[k/4])
                          + (0.8 * f_bufPost4_right[k/4]);
    
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

float elephant (float x, float y[])
{
	float z = x - 1/2.0;
	float even1 = y[1]+y[0], odd1 = y[1]-y[0];
	float even2 = y[2]+y[-1], odd2 = y[2]-y[-1];
	float c0 = even1*0.45868970870461956 + even2*0.04131401926395584;
	float c1 = odd1*0.48068024766578432 + odd2*0.17577925564495955;
	float c2 = even1*-0.246185007019907091 + even2*0.24614027139700284;
	float c3 = odd1*-0.36030925263849456 + odd2*0.10174985775982505;
	return ((c3*z+c2)*z+c1)*z+c0;
}

void interp5( float a[], int n, float b[], int m )
{
    float step = (float)( (float)n - 1 ) / ((float)m - 1);
    for( int j = 0; j < m; j ++ ){
        b[j] = elephant( j*step, a);
    }
}

float interp1( float x, float a[], int n )
{
    if( x <= 0 )  return a[0];
    if( x >= n - 1 )  return a[n-1];
    int j = (int)x;
    return a[j] + (x - j) * (a[j+1] - a[j]);
}

    // linear interpolate array a[] -> array b[]
void inter1parray( float a[], int n, float b[], int m )
{
    float step = (float)( (float)n - 1 ) / ((float)m - 1);
    for( int j = 0; j < m; j ++ ){
        b[j] = interp1( j*step, a, n );
    }
}

//..............................................................................
    // parabola through 3 points, -1 < x < 1

float parabola( float x, float f_1, float f0, float f1 )
{
    if( x <= -1 )  return f_1; 
    if( x >= 1 )  return f1; 
    float l = f0 - x * (f_1 - f0);
    float r = f0 + x * (f1 - f0);
    return (l + r + x * (r - l)) / 2;
}

    // quadratic interpolate x in an array
float interp2( float x, float a[], int n )
{
    if( x <= .5  ||  x >= n - 1.5 )
        return interp1( x, a, n );
    int j = (int)( x + .5 );
    float t = 2 * (x - j);  // -1 .. 1
    return parabola( t, (a[j-1] + a[j]) / 2, a[j], (a[j] + a[j+1]) / 2 );
}

    // quadratic interpolate array a[] -> array b[]
void interp2array( float a[], int n, float b[], int m )
{
    float step = (float)( n - 1 ) / (m - 1);
    for( int j = 0; j < m; j ++ ){
        b[j] = interp2( j*step, a, n );
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

void drawInterface() {
  BSP_LCD_Clear(LCD_COLOR_BLUE);
	
	//BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	//BSP_LCD_DrawVLine(160,0,320);
    
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillRect(5,5,110,100);
  BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
  BSP_LCD_FillRect(10,10,100,90);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
	BSP_LCD_DisplayStringAt(50, 45, (uint8_t *)"1", LEFT_MODE);
  
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillRect(120,5,115,100);
  BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
  BSP_LCD_FillRect(125,10,105,90);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
	BSP_LCD_DisplayStringAt(170, 45, (uint8_t *)"2", LEFT_MODE);
  
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillRect(240,5,115,100);
  BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
  BSP_LCD_FillRect(245,10,105,90);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
	BSP_LCD_DisplayStringAt(290, 45, (uint8_t *)"3", LEFT_MODE);

  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillRect(360,5,115,100);
  BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
  BSP_LCD_FillRect(365,10,105,90);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
	BSP_LCD_DisplayStringAt(410, 45, (uint8_t *)"4", LEFT_MODE);

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