#include "stm32746g_discovery.h"
#include "stm32746g_discovery_audio.h"
#include "stm32f7xx_hal.h"

#include "ff.h"
#include "ff_gen_drv.h"
#include "usbh_diskio.h"

#include "ct-gui/gui_stm32.h"
#include "ct-head/random.h"

#include "common/clockconfig.h"

#include "common/clockconfig.h"
#include "wavfile.h"

#include "common/clockconfig.h"

#include <stdio.h>
#include <stdlib.h>

// #include "stm32fxxx_hal.h"
// #include "defines.h"
// #include "tm_stm32_disco.h"
// #include "tm_stm32_delay.h"
// #include "tm_stm32_usb_device.h"
// #include "tm_stm32_usb_device_msc.h"

uint8_t currentLCDcolor = 0;
const uint32_t LCDColorarray[] = { LCD_COLOR_BLUE, LCD_COLOR_BLACK, LCD_COLOR_GREEN };

#define TIMx TIM3
#define TIMx_CLK_ENABLE() __HAL_RCC_TIM3_CLK_ENABLE()
#define TIMx_IRQn TIM3_IRQn
#define TIMx_IRQHandler TIM3_IRQHandler

TIM_HandleTypeDef timer;
static uint8_t isBlinking = 1;
uint8_t running = 0;
uint8_t playbackactive = 1;

#define VOLUME 50
#define SAMPLE_RATE 44100

//#define AUDIO_DMA_BUFFER_SIZE 4096
//#define AUDIO_DMA_BUFFER_SIZE2 (AUDIO_DMA_BUFFER_SIZE >> 1)

#define AUDIO_DMA_BUFFER_SIZE 4096
#define AUDIO_DMA_BUFFER_SIZE2 (AUDIO_DMA_BUFFER_SIZE >> 1)
#define AUDIO_DMA_BUFFER_SIZE4 (AUDIO_DMA_BUFFER_SIZE >> 2)
#define AUDIO_DMA_BUFFER_SIZE8 (AUDIO_DMA_BUFFER_SIZE >> 3)

static uint8_t temp_audioBuf[AUDIO_DMA_BUFFER_SIZE];
static uint8_t temp_audioBuf2[AUDIO_DMA_BUFFER_SIZE];
static uint8_t audioBuf[AUDIO_DMA_BUFFER_SIZE];

uint8_t audioBufOrg[2048];
uint8_t audioBufNew[2048];

uint16_t taptempo[4];
uint8_t taptempocounter;

uint8_t left_firstbyte;
uint8_t left_secondbyte;
uint8_t right_firstbyte;
uint8_t right_secondbyte;
uint16_t temp_16bit_audio_buffer_left[1024];
uint16_t temp_16bit_audio_buffer_right[1024];
uint16_t temp_16bit_audio_buffer_left2[1024];
uint16_t temp_16bit_audio_buffer_right2[1024];

uint32_t taptempo_prev = 0;
float global_tempo = 100;

float f_bufPre_left[1024];
float f_bufPre_right[1024];
float f_bufPost_left[1024];
float f_bufPost_right[1024];
float f_bufMid_left[1024];
float f_bufMid_right[1024];

float leftdouble[2048];
float rightdouble[2048];
float leftdouble2[2048];
float rightdouble2[2048];

short shortleft[512];
short shortright[512];
short resleft[512];
short resright[512];

BYTE buffer314[128];    /* file copy buffer */


#define WAVEFILENAME "0:sound.wav"
#define WAVEFILENAME2 "0:sound2.wav"
TCHAR szWindowClass[] = ("0:sound.wav");

typedef enum {
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL
} DMABufferState;

extern HCD_HandleTypeDef hhcd;
extern USBH_HandleTypeDef hUSBH;
extern SAI_HandleTypeDef haudio_out_sai;

static uint8_t audio_buf[AUDIO_DMA_BUFFER_SIZE];
static DMABufferState dma_state = BUFFER_OFFSET_NONE;

static uint32_t blink_period = 250;

FIL audio_file;
FIL audio_file2;
FIL fsrc, fdst; 
CTSS_WavHeader audio_format;
CTSS_WavHeader audio_format2;
UINT bytes_read;
UINT bytes_read2;
UINT prev_bytes_read;
FATFS fs;


static char usb_drive_path[4];

typedef struct {
	uint16_t touchX[1];
	uint16_t touchY[1];
	uint32_t lastTouch;
	uint8_t touchDetected;
	uint8_t touchUpdate;
} GUITouchState;

static TS_StateTypeDef rawTouchState;
//static GUITouchState touchState;
uint16_t runOnce = 0;
uint16_t initdone = 0;
uint16_t ccounter = 0;
uint16_t touchx = 0;
uint16_t touchy = 0;
uint16_t countstar = 0;

static void process_usbh_message(USBH_HandleTypeDef *host, uint8_t msg);
static void start_playback();
static void stop_playback();
static void play_wav_file();
static void initAudio();
void computeAudio();
void inter1parray( float aaaa[], int n, float bbbb[], int m );
void interp2array( float aaaa[], int n, float bbbb[], int m );
void interp5( float aaaa[], int n, float bbbbb[], int m );

//static void initTimer(uint16_t period);

static void demoWelcome() {
  BSP_LCD_SetFont(&CTGUI_FONT);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                          (uint8_t *)"STM32F746G", CENTER_MODE);
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 8, (uint8_t *)"llelel",
                          CENTER_MODE);
  //const float w = BSP_LCD_GetXSize() - 5;
  //const float h = BSP_LCD_GetYSize() - 5;

  CT_Smush rnd;
  // seed random number generator
  ct_smush_init(&rnd, 0xdecafbad);

  /*while (1) {
    BSP_LCD_SetTextColor((ct_smush(&rnd) & 0xffffff) | 0xff000000);
    BSP_LCD_FillCircle(ct_smush_minmax(&rnd, 5.f, w),
                       ct_smush_minmax(&rnd, 5.f, h), 4);
    HAL_Delay(1);
  }*/
}

int main() {
  CPU_CACHE_Enable();
  HAL_Init();
  SystemClock_Config();
  BSP_LCD_Init();
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Off(LED_GREEN);
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  
  // only continue if touch screen init ok
  if (BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize()) == TS_OK) {
    BSP_LCD_LayerDefaultInit(1, SDRAM_DEVICE_ADDR);
	BSP_LCD_LayerDefaultInit(2, SDRAM_DEVICE_ADDR);
    BSP_LCD_SelectLayer(1);
	BSP_TS_ITConfig();
  }
  
  demoWelcome();
  
  if (FATFS_LinkDriver(&USBH_Driver, usb_drive_path) == 0) {
    USBH_Init(&hUSBH, process_usbh_message, 0);
    USBH_RegisterClass(&hUSBH, USBH_MSC_CLASS);
    USBH_Start(&hUSBH);
  }
  
  //initTimer(5000);
  
  // if (FATFS_LinkDriver(&USBH_Driver, usb_drive_path) == 0) {
  //   USBH_Init(&hUSBH, process_usbh_message, 0);
  //   USBH_RegisterClass(&hUSBH, USBH_MSC_CLASS);
  //   USBH_Start(&hUSBH);
    while (1) {
		//Application_Process();
		
		if(running == 1) {
			//HAL_Delay(1);
			strcpy(szWindowClass, "0:sound.wav");
			BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
		   (uint8_t *)"main", CENTER_MODE);
		   //start_playback();
			running = 0;
			playbackactive = 1;
			start_playback();
			//play_wav_file();
		}
		USBH_Process(&hUSBH);
		
	    //BSP_LED_Toggle(LED_GREEN);
	    //HAL_Delay(blink_period);
    }
  // } else {
//     Error_Handler();
//   }
//
  //while (1)
  //  ;

  return 0;
}

static void init_after_USB() {
    if (f_open(&audio_file, szWindowClass, FA_READ) == FR_OK) {
      //f_read(&audio_file, &audio_format, sizeof(CTSS_WavHeader), &bytes_read);
      //play_wav_file();
    } else {
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                                (uint8_t *)"ERROR", CENTER_MODE);
      Error_Handler();
    }
	
	/*f_mount(&fs, "", 1);
    if (f_open(&fdst, "dstfile.dat", FA_WRITE  | FA_OPEN_ALWAYS)) {
	  //sprintf(buffer314,"The result was \n", 100);
		const char string[] = "Hallo world\0";
  	  f_write(&fdst, string, 11, &bytes_read);
	  f_sync(&fdst);
  	  f_close(&fdst);
    } else {
      //Error_Handler();
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                              (uint8_t *)"audio", CENTER_MODE);
	  Error_Handler();
    }*/
	
	// skip wave header?
	f_lseek(&audio_file, 0);
	
	uint32_t filesz = f_size(&audio_file);
	uint8_t nbrbeats = 8;
	// filesize / 44100 (samplerate) / 2 (channels) / 2 (16 bits = 2 bytes per channel)
	// this gives playtime in seconds
	// then divide by nbrofbeats to get time per beat in seconds
	// then divide 60 through the time per beat -> BPM
	float bpm = 60 / (((float)filesz / 44100 / 2 / 2) / nbrbeats); 
	//bpm = 30.4;
	char a[] = "";
	//String sfsd(bpm, 5);
	//ftoa2( bpm, a, 2);
	//sprintf(a, "%.4g", bpm );
	snprintf(a, 6, "%f",bpm);
	//sprintf(a, "%d.%02u", bpm);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)a, CENTER_MODE);
	//
  
    initAudio();
}
 
static void process_usbh_message(USBH_HandleTypeDef *host, uint8_t msg) {
  switch (msg) {
    case HOST_USER_SELECT_CONFIGURATION:
      break;

    case HOST_USER_DISCONNECTION:
      f_mount(NULL, (TCHAR const *)"", 0);
      stop_playback();
      break;

    case HOST_USER_CLASS_ACTIVE:
	//start_playback();
	init_after_USB();
      break;

    case HOST_USER_CONNECTION:
      break;

    default:
      break;
  }
}

static void start_playback() {
  if (f_open(&audio_file, szWindowClass, FA_READ) == FR_OK) {
    f_read(&audio_file, &audio_format, sizeof(CTSS_WavHeader), &bytes_read);
    play_wav_file();
  } else {
    Error_Handler();
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                            (uint8_t *)"ERROR", CENTER_MODE);
  }
}

static void initAudio() {
    if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, VOLUME, SAMPLE_RATE) != 0) {
      Error_Handler();
    }
    BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
    BSP_AUDIO_OUT_SetVolume(VOLUME);
    BSP_AUDIO_OUT_Play((uint16_t *)audioBuf, AUDIO_DMA_BUFFER_SIZE);
}

void AUDIO_OUT_SAIx_DMAx_IRQHandler(void) {
  HAL_DMA_IRQHandler(haudio_out_sai.hdmatx);
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
	
	computeAudio();
	memcpy(audioBuf, temp_audioBuf2, AUDIO_DMA_BUFFER_SIZE2);
	
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {

	computeAudio();
	memcpy(&audioBuf[AUDIO_DMA_BUFFER_SIZE2], temp_audioBuf2, AUDIO_DMA_BUFFER_SIZE2);
	
	
	//prev_bytes_read = bytes_read;
	if(bytes_read==0) {
		memset(audioBuf, 0, sizeof audioBuf);
		memset(temp_audioBuf, 0, sizeof temp_audioBuf);
	}
}

void computeAudio() {
	
	float rate = global_tempo / 100;
	if (rate > 2) rate = 2;
	short bufsize = (short)(2048 * rate);
	// garantue a multiple of 4
	bufsize -= bufsize%4;
	
	// read chunk from USB
	f_read(&audio_file, temp_audioBuf, bufsize, &bytes_read);
	//countstar=countstar+10;
	
	// 2's-complement signed integers -> short (-32k -> +32k) -> float (-1 -> +1)
	for (int j=0; j < bufsize; j=j+4) {
		// copy of original audiobuf
		//audioBufOrg[j] = temp_audioBuf[j];
		//audioBufOrg[j+1] = temp_audioBuf[j+1];
		//audioBufOrg[j+2] = temp_audioBuf[j+2];
		//audioBufOrg[j+3] = temp_audioBuf[j+3];
		
		// to short
		short lefty = (short) (temp_audioBuf[j+1]<<8 | (temp_audioBuf[j] & 0xFF));
		short righty = (short) (temp_audioBuf[j+3]<<8 | (temp_audioBuf[j+2] & 0xFF));
		//shortleft[j/4] = lefty;
		//shortright[j/4] = righty;
		
		// to float
		f_bufPre_left[j/4] = ((float)lefty/32768);
		f_bufPre_right[j/4] = ((float)righty/32768);
	}
	
	// processing audio on floats
	//for (int k=0; k<512; k++) {
	//	f_bufPost_left[k] = f_bufPre_left[k];
	//	f_bufPost_right[k] = f_bufPre_right[k];
	//}
	
	inter1parray( f_bufPre_left, bufsize / 4, f_bufPost_left, 512 );
	inter1parray( f_bufPre_right, bufsize / 4, f_bufPost_right, 512 );
	
	//f_bufPost_left[0] = f_bufMid_left[0];
	//f_bufPost_right[0] = f_bufMid_right[0];
	//for(int i=1; i<512; i++){
		//temp_16bit_float_audio_buffer_left2[i] = temp_16bit_float_audio_buffer_left[i];
		
	//	f_bufPost_left[i] = f_bufPost_left[i-1] + (1*(f_bufMid_left[i] - f_bufPost_left[i-1]));
	//	f_bufPost_right[i] = f_bufPost_right[i-1] + (1*(f_bufMid_right[i] - f_bufPost_right[i-1]));
		//f_bufPost_left[i] = f_bufPre_left[i];
		//f_bufPost_right[i] = f_bufPre_right[i];	
	  //  }
	
	
	// float to short
	for (int k=0; k<2048; k=k+4) {
		// to short
		resleft[k/4] = (short)(f_bufPost_left[k/4] * 32768);
		resright[k/4] = (short)(f_bufPost_right[k/4] * 32768);
		// to 2's-comp
		//if(resleft[k/4]<0)resleft[k/4]++;
		//if(resright[k/4]<0)resright[k/4]++;
		temp_audioBuf2[k+1] = resleft[k/4]>>8;
		temp_audioBuf2[k] = resleft[k/4]&0xff;
		temp_audioBuf2[k+3] = resright[k/4]>>8;
		temp_audioBuf2[k+2] = resright[k/4]&0xff;
		
	}
	
	
	//char a[] = "";
	//sprintf(a, "%d", countstar);
	//BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)a, CENTER_MODE);
	
	
	
	/*
	// convert uint8 to uint16
	// split left and right channel
	uint16_t counter=0;
	for( int j = 0; j < 2048; j=j+4 ){
		left_firstbyte = temp_audioBuf[j];
		left_secondbyte = temp_audioBuf[j+1];
		right_firstbyte = temp_audioBuf[j+2];
		right_secondbyte = temp_audioBuf[j+3];
		
		uint16_t left = ((uint16_t)left_secondbyte << 8) | left_firstbyte;
		uint16_t right = ((uint16_t)right_secondbyte << 8) | right_firstbyte;
		
		// convert uint16_t values to float
		// 512 samples in each buffer
		//temp_16bit_float_audio_buffer_left[counter] = ((float)left/32768) - 1;
		//temp_16bit_float_audio_buffer_right[counter] = ((float)right/32768) - 1;
		temp_16bit_float_audio_buffer_left[counter] = ((float)left/65536);
		temp_16bit_float_audio_buffer_right[counter] = ((float)right/65536);
		counter++;
    }
	
	//char a[] = "";
	//char charVal[10]; 
	//float fVal = 0.0256;
	//dtostrf(fVal, 8, 4, charVal);
	//sprintf(a, "%d", temp_16bit_float_audio_buffer_left[100]);
	//BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)a, CENTER_MODE);
	// 4x oversample the float arrays
	for (int i=4;i<2048;i=i+4) {
		rightdouble[i] = temp_16bit_float_audio_buffer_right[i/4];
		rightdouble[i-1] = 0.0f;
		rightdouble[i-2] = 0.0f;
		rightdouble[i-3] = 0.0f;
		
		leftdouble[i] = temp_16bit_float_audio_buffer_left[i/4];
		leftdouble[i-1] = 0.0f;
		leftdouble[i-2] = 0.0f;
		leftdouble[i-3] = 0.0f;
	}
	
	
    //lowpass the resulting arrays
	temp_16bit_float_audio_buffer_left2[0] = temp_16bit_float_audio_buffer_left[0];
	temp_16bit_float_audio_buffer_right2[0] = temp_16bit_float_audio_buffer_right[0];
	//temp_16bit_float_audio_buffer_right2[0] = 0;
	//for(int i=1; i<512; i++){
	//        temp_16bit_float_audio_buffer_left2[i] = temp_16bit_float_audio_buffer_left2[i-1] + (0.05*(temp_16bit_float_audio_buffer_left[i] - temp_16bit_float_audio_buffer_left2[i-1]));
	//    }
	//for(int i=1; i<512; i++){
	//        temp_16bit_float_audio_buffer_right2[i] = temp_16bit_float_audio_buffer_right2[i-1] + (0.05*(temp_16bit_float_audio_buffer_right[i] - temp_16bit_float_audio_buffer_right2[i-1]));
	//    }
		
		
		for(int i=1; i<512; i++){
			temp_16bit_float_audio_buffer_left2[i] = temp_16bit_float_audio_buffer_left[i];
				//temp_16bit_float_audio_buffer_left2[i] = temp_16bit_float_audio_buffer_left2[i-1] + (0.6*(temp_16bit_float_audio_buffer_left[i] - temp_16bit_float_audio_buffer_left2[i-1]));
		    }
		for(int i=1; i<512; i++){
			temp_16bit_float_audio_buffer_right2[i] = temp_16bit_float_audio_buffer_right[i];
				//temp_16bit_float_audio_buffer_right2[i] = temp_16bit_float_audio_buffer_right2[i-1] + (0.6*(temp_16bit_float_audio_buffer_right[i] - temp_16bit_float_audio_buffer_right2[i-1]));
		    }
		
		
	
	
	// 2x oversample the float arrays
	
	//rightdouble[0] = 0;
	//leftdouble[0] = 0;	
	
	//for( int j = 1; j < 512; j++ ){
	//	leftdouble[j*2] = temp_16bit_float_audio_buffer_left[j];
	//	leftdouble[(j*2)-1] = (leftdouble[j*2] + leftdouble[(j*2)-2]) / 2;
	//}
	//for( int j = 1; j < 512; j++ ){
	//	rightdouble[j*2] = temp_16bit_float_audio_buffer_right[j];
	//	rightdouble[(j*2)-1] = (rightdouble[j*2] + rightdouble[(j*2)-2]) / 2;
	//}
	//float rightdouble[1024];
	
	
	// perform stretch algoritm
		inter1parray( leftdouble2, 2048, leftdouble, 2048 );
		inter1parray( rightdouble2, 2048, rightdouble, 2048 );
	
	// 
	for(int i=1; i<512; i++){
	//temp_16bit_float_audio_buffer_left[i] = leftdouble[i*4];
	//temp_16bit_float_audio_buffer_right[i] = rightdouble[i*4];
	}
	
	
	counter=0;
	// extract float arrays
	for( int k = 0; k < 512; k=k+1 ){
		
		// convert from float to uint16_t
		uint16_t samp_left = (uint16_t)((temp_16bit_float_audio_buffer_left2[k]) * 65536);
		uint16_t samp_right = (uint16_t)((temp_16bit_float_audio_buffer_right2[k]) * 65536);
		
		//uint16_t samp_left = (uint16_t)((temp_16bit_float_audio_buffer_left[k] + 1) * 32768);
		//uint16_t samp_right = (uint16_t)((temp_16bit_float_audio_buffer_right[k] + 1) * 32768);
		
		//if(samp_left<30000) samp_left=30000;
		//if(samp_right<30000) samp_right=30000;
		
		
		// convert from uint16_t to uint8_t
		temp_audioBuf2[counter] = samp_left & 0xff;
		temp_audioBuf2[counter+1] = (samp_left >> 8);
		temp_audioBuf2[counter+2] = samp_right & 0xff;
		temp_audioBuf2[counter+3] = (samp_right >> 8);
		counter = counter + 4;
	}
	*/
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

// float interp1( float frac_pos, float a[], int n )
// {
// 	float xm1 = a
// const float    c     = (x1 - xm1) * 0.5f;
// const float    v     = x0 - x1;
// const float    w     = c + v;
// const float    a     = w + v + (x2 - x0) * 0.5f;
// const float    b_neg = w + a;
//
// return ((((a * frac_pos) - b_neg) * frac_pos + c) * frac_pos + x0);
// }

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
	// for(int j=0; j<256; j++) {
	// 	b[j] = a[j];
	// }
	// for(int j=256; j<512; j++) {
	// 	b[j] = a[j];
	// }
}












static void play_wav_file() {
	if(initdone == 0) {
  BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, VOLUME, audio_format.sampleRate);
  BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
   initdone = 1;
}

  f_lseek(&audio_file, 0);
  f_read(&audio_file, audio_buf, AUDIO_DMA_BUFFER_SIZE, &bytes_read);
  uint32_t samples_remaining = audio_format.riffLength - bytes_read;
  BSP_AUDIO_OUT_Play((uint16_t *)audio_buf, AUDIO_DMA_BUFFER_SIZE);
  uint32_t err = 0;
  while (!err && playbackactive) {
    bytes_read = 0;
    if (dma_state == BUFFER_OFFSET_HALF) {
      err = (f_read(&audio_file, audio_buf, AUDIO_DMA_BUFFER_SIZE2,
                    &bytes_read) != FR_OK);
      dma_state = BUFFER_OFFSET_NONE;
    } else if (dma_state == BUFFER_OFFSET_FULL) {
      err = (f_read(&audio_file, &audio_buf[AUDIO_DMA_BUFFER_SIZE2],
                    AUDIO_DMA_BUFFER_SIZE2, &bytes_read) != FR_OK);
      dma_state = BUFFER_OFFSET_NONE;
    }
    if (!err) {
      if (samples_remaining > AUDIO_DMA_BUFFER_SIZE2) {
        samples_remaining -= bytes_read;
      } else {
        f_lseek(&audio_file, 0);
        samples_remaining = audio_format.riffLength - bytes_read;
      }
      //BSP_LED_Toggle(LED_GREEN);
    }
  }
  stop_playback();
}

static void stop_playback() {
  BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW);
}

void OTG_FS_IRQHandler(void) {
  HAL_HCD_IRQHandler(&hhcd);
}

//void AUDIO_OUT_SAIx_DMAx_IRQHandler(void) {
 // HAL_DMA_IRQHandler(haudio_out_sai.hdmatx);
//}

//void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
//  dma_state = BUFFER_OFFSET_HALF;
//}



void BSP_AUDIO_OUT_Error_CallBack(void) {
  Error_Handler();
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                          (uint8_t *)"ERROR", CENTER_MODE);
}

/*
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
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                            (uint8_t *)"ERROR", CENTER_MODE);
  }
  if (HAL_TIM_Base_Start_IT(&timer) != HAL_OK) {
    Error_Handler();
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
                            (uint8_t *)"ERROR", CENTER_MODE);
  }
}*/

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
	// runonce to run this function only once when IRQ fires (fires continuesly)
	runOnce = 0;
	
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DrawVLine(180,0,320);
	
		
	BSP_TS_GetState(&rawTouchState);
	while (rawTouchState.touchDetected) {
		// run once on (continues) touch
		if (runOnce == 0) {
			
			touchx = rawTouchState.touchX[0];
			
			if(touchx > 180) {
			
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
			
			
			
				BSP_LCD_Clear(LCDColorarray[currentLCDcolor]);
			
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
				// restart audio
				//strcpy(szWindowClass, "0:sound3.wav");
				f_lseek(&audio_file, 0);
			}
			
			runOnce = 1;
		}
	// read state and continue with while
	BSP_TS_GetState(&rawTouchState);
	} // end while
}

// Callback function run whenever timer caused interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	
		
	if (isBlinking) {
		BSP_LED_Toggle(LED_GREEN);
	}
	/*BSP_TS_GetState(&rawTouchState);
	 		if (rawTouchState.touchDetected) {
				
				if (fingerdown == 0) {
			   
			   //stop_playback();
			   //if(running==1){
			   //	start_playback();
			   //}
					sprintf(a, "%d", initdone);
						BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
						 (uint8_t *)a, CENTER_MODE);
						 playbackactive = 0;
						 f_close(&audio_file);
						 //f_close(&audio_file);
						 
						 //start_playback();
						 //stop_playback();
						 //stop_playback();
						 //play_wav_file();
						 running = 1;
					
					//f_close(&audio_file);
					//start_playback();
				   //if (f_open(&audio_file2, WAVEFILENAME2, FA_READ) == FR_OK) {
					//   	   	   BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8,
				//			   	   	   (uint8_t *)"ERROR", CENTER_MODE);
								   //}
			    // f_read(&audio_file2, &audio_format2, sizeof(CTSS_WavHeader), &bytes_read2);
						   
				
			   
			   //if (f_open(&audio_file, WAVEFILENAME2, FA_READ) == FR_OK) {
			   //  f_read(&audio_file, &audio_format, sizeof(CTSS_WavHeader), &bytes_read);
			   //  play_wav_file();
							   }
			   //HAL_Delay(1);
			   //start_playback();
			 fingerdown = 1;
			 //}
			}
			
			if (rawTouchState.touchDetected == 0) {
			    fingerdown = 0;
			}

	*/
}