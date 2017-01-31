/*

Wakefield STM32f7 Synthesizer

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

#define ARM_MATH_CM7
#include "arm_math.h"

#include "Heavy_wakefield.h"

// defines
#define VOLUME 50
#define SAMPLE_RATE 48000
#define AUDIO_DMA_BUFFER_SIZE 1024  // divided by 4 (C array item length) reserves a stereo audio buffer of 256 bytes
                                    
#define AUDIO_DMA_BUFFER_SIZE2 (AUDIO_DMA_BUFFER_SIZE >> 1)

// audio buffers
static int16_t int_bufProcessedOut[AUDIO_DMA_BUFFER_SIZE];
static uint8_t audioOutBuf[AUDIO_DMA_BUFFER_SIZE];
extern SAI_HandleTypeDef haudio_out_sai;

// midi
UART_HandleTypeDef uart_config;
uint8_t midicounter=0;
int16_t mididata[3] = {0,-1,-1};
uint8_t rx_byte[1];

// touch
uint8_t runOnce;
static TS_StateTypeDef rawTouchState;

// heavy
double sampleRate = 48000;
HeavyContextInterface *context;

// adc
uint32_t g_ADCValue;
int g_MeasurementNumber;
ADC_HandleTypeDef g_AdcHandle;
DMA_HandleTypeDef  g_DmaHandle;
enum{ ADC_BUFFER_LENGTH = 8192 };
uint32_t g_ADCBuffer[ADC_BUFFER_LENGTH];
uint16_t ADCchannelValues[4];

// header
void initAudio();
void computeAudio();
void UART6_Config();
void handle_midi();
void drawGui();
void printFloat(float lefloat, int x, int y);
void ConfigureADC();
void ConfigureDMA();

void printHook(HeavyContextInterface *thecontext, const char *printLabel, const char *msgString, const HvMessage *themsg) {
  //BSP_LCD_DisplayStringAt(150, 150, (uint8_t *)msgString, LEFT_MODE);
}

int main() {
  CPU_CACHE_Enable();
  HAL_Init();
  SystemClock_Config(); 
  
  // config UART for MIDI communication
  UART6_Config();
  
  // led and pushbutton config
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Off(LED_GREEN);
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  
  // Init LCD and Touchscreen
  BSP_LCD_Init();
  if (BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize()) == TS_OK) {
    BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	  BSP_TS_ITConfig();
  }  
  
  // GUI init
  drawGui();

  // start audio system
  initAudio();
  context = hv_wakefield_new(sampleRate);
  hv_setPrintHook(context, &printHook);

  // init ADC and DMA
  ConfigureADC();
  ConfigureDMA();
  HAL_ADC_Start_DMA(&g_AdcHandle, g_ADCBuffer, ADC_BUFFER_LENGTH);

  // main loop
  //int blah = 0;
  while (1) {

    // do nothing
    //BSP_LED_Toggle(LED_GREEN);
    char a[] = "";
    // //sprintf(a, "%d", (int)filterfreq);
    sprintf(a, "%d", (int)HAL_GetTick());
    // //sprintf(a, "%d", (int)result*1000);
    BSP_LCD_DisplayStringAt(10, 10, (uint8_t *)a, LEFT_MODE);
    //
    // if(HAL_GetTick()>8000 && blah ==0) {
    //   BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)"go", CENTER_MODE);
    //   hv_sendBangToReceiver(context, hv_stringToHash("thebang"));
    //   hv_sendFloatToReceiver(context, hv_stringToHash("thefloat"), 2);
    //   blah = 1;
    // }
    
    //char a[] = "";
    uint32_t digu = (uint32_t)(ADCchannelValues[0]);
   //uint16_t newval = g_ADCValue - (uint16_t)*blsaw->freq;
    sprintf(a, "%ld", digu);
    BSP_LCD_DisplayStringAt(50, 50, (uint8_t *)a, LEFT_MODE);

  }

  return 0;
}

void printFloat(float lefloat, int x, int y) {
  char a[] = "";
  char *tmpSign = (lefloat < 0) ? "-" : "";
  float tmpVal = (lefloat < 0) ? -lefloat : lefloat;

  int tmpInt1 = tmpVal;                  // Get the integer (678).
  float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
  int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

  // Print as parts, note that you need 0-padding for fractional bit.

  sprintf (a, "%s%d.%03d", tmpSign, tmpInt1, tmpInt2);
  //sprintf(a, "%f", testfloat);
  BSP_LCD_DisplayStringAt(x, y, (uint8_t *)a, LEFT_MODE);
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

void computeAudio() {

  //float audioproc[512];
  float32_t mixdown[1024];

  // calculate 512 monosamples
  // for(int sampleit=0;sampleit<512;sampleit++) {
//
//     audioproc[sampleit] = 0;
//
//   }
  
  //computemydsp(dsp, 512, 0, outputs);
  
  hv_processInlineInterleaved(context, NULL, mixdown, 128);
  
  // //double array mono -> stereo
  // int counter = 0;
  // for(int sampleit=0;sampleit<1024;sampleit+=2) {
  //   mixdown[sampleit] = audioproc[counter];
  //   mixdown[sampleit+1] = audioproc[counter];
  //   counter++;
  // }

  // convert float to q15 / int16
  // send to transferbuffer
  arm_float_to_q15(mixdown, int_bufProcessedOut, 512);
}

// UART (MIDI) HANDLING --------------------------------------------

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  //BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)"le midi", CENTER_MODE);
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

  // note on if vel (mididata[2]) > 0
  if(mididata[0] == 144 && mididata[2] > 0) {
    // do midi stuff
  }
  
  // implicit note off
  // or note off
  if((mididata[0] == 144 && mididata[2] == 0)  || (mididata[0] == 128)) {
    // do midi stuff
  }

  // controllers
 if(mididata[0] == 176) {
   if(mididata[1]==71) {
    
   }
   
   if(mididata[1]==72) {
   
   }
 }
}

// x1,y1,x2,y2
// 1 = button
// 2 = toggle
// 3 = slider
// min
// max
// current
// 
// slider -> width 200 (still hardcoded)
// slider -> heigth 20 (hardcoded)
float gui_elements[3][8] = {
// x1  y1  x2  y2  type min max value
  {100,100,300,120,3,  0, 100, 10},
  {200,200,400,220,3,  0, 100, 50},
  { 50, 50,  0,  0,1,  0,   1,  0}
};

// gui name and controller PD element
char *gui_elements_name[2][2] = {
  {"ele1","thefloat"},
  {"ele2","thefloat"}
};

uint8_t nbr_elements = 3;

void drawGui() {
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  BSP_LCD_SetFont(&Font12);
  //BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t *)"faustreceiver", CENTER_MODE);
  
  // iterate through elements and draw
  for (uint8_t i = 0; i < nbr_elements; i++) {
    // slider
    if(gui_elements[i][4]==3) {
      BSP_LCD_DrawRect(gui_elements[i][0], gui_elements[i][1],200,20);
      BSP_LCD_FillRect(gui_elements[i][0]+2, gui_elements[i][1]+2, gui_elements[i][7] * (200.0f / gui_elements[i][6]),17);
      BSP_LCD_DisplayStringAt(gui_elements[i][0]-50, gui_elements[i][1]+5, (uint8_t *)gui_elements_name[i][0], LEFT_MODE);
      printFloat(gui_elements[i][7], gui_elements[i][0]+50, gui_elements[i][1]+5);
    }
    // button
    if(gui_elements[i][4]==1) {
      //BSP_LCD_DisplayStringAt(, (uint8_t *)"slider", LEFT_MODE);
      BSP_LCD_DrawRect(gui_elements[i][0], gui_elements[i][1],20,20);
      // button pressed down
      if(gui_elements[i][7]==1) {
          BSP_LCD_FillRect(gui_elements[i][0]+2, gui_elements[i][1]+2,17,17);
      }
      //BSP_LCD_FillRect(gui_elements[i][0]+2, gui_elements[i][1]+2, gui_elements[i][7] * (200.0f / gui_elements[i][6]),17);
      //BSP_LCD_DisplayStringAt(gui_elements[i][0]-50, gui_elements[i][1]+5, (uint8_t *)gui_elements_name[i][0], LEFT_MODE);
      //printFloat(gui_elements[i][7], gui_elements[i][0]+50, gui_elements[i][1]+5);
    }
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

// void HAL_GPIO_EXTI_IRQHandler points to this:
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// runonce to run this function only once when IRQ fires (fires continuesly)
  // getsecondtouch
  // reset flags
	runOnce = 0;
  //int getSecondTouch = 0;
  
  //drawInterface();
		
	BSP_TS_GetState(&rawTouchState);
	while (rawTouchState.touchDetected) {
		// run once on (continues) touch
		if (runOnce == 0) {
      
      runOnce = 1;
			
			uint16_t touchx = rawTouchState.touchX[0];
      uint16_t touchy = rawTouchState.touchY[0];
      
      // check is element is hit
      for (uint8_t i = 0; i < nbr_elements; i++) {
        // touch on a GUI element?
        if(touchx > gui_elements[i][0] && touchx < gui_elements[i][2] && touchy > gui_elements[i][1] && touchy < gui_elements[i][3]) {
          // check GUI element type
          // button
          if(gui_elements[i][4]==1) {
            // lock the touchevent until finished
            while(rawTouchState.touchDetected == 1) {
              gui_elements[i][7]= 1;
              drawGui();
            }
            // reset button
            gui_elements[i][7]= 0;
          }
          // slider
          if(gui_elements[i][4]==3) {
            // lock the touchevent until finished
            while(rawTouchState.touchDetected == 1) {
        			touchx = rawTouchState.touchX[0];
              touchy = rawTouchState.touchY[0];
              
              //BSP_LCD_FillCircle(touchx,touchy,10);
              // new value = touchx - guielement x
              int newvalue = rawTouchState.touchX[0]-gui_elements[i][0];
              float newscaledvalue = newvalue / (200.0f / gui_elements[i][6]);
              if(newvalue >= gui_elements[i][5] && newscaledvalue <= gui_elements[i][6]) {
                gui_elements[i][7]= newscaledvalue;
                // set linked PD patch value
                hv_sendFloatToReceiver(context, hv_stringToHash(gui_elements_name[i][1]), newscaledvalue);
              }
              drawGui();
              
              BSP_TS_GetState(&rawTouchState);
            } // end touchlock
          } // end slider GUI element 
        } // end touch on GUI event
      }
      
      // //BSP_LCD_DisplayStringAt(200, 200, (uint8_t *)"touchevent", LEFT_MODE);
      // if(touchx < 150 && touchx > 100 && touchy > 100 && touchy < 150) {
      //
      //   // get second touch event
      //   getSecondTouch = 1;
      // }
		}
    
      //     if (getSecondTouch == 1 && rawTouchState.touchDetected == 2) {
      // uint16_t touchx = rawTouchState.touchX[1];
      //       uint16_t touchy = rawTouchState.touchY[1];
      //       BSP_LCD_FillCircle(touchx,touchy,50);
      //     }
	// read state and continue with while
	BSP_TS_GetState(&rawTouchState);
	} // end while
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





// ADC and DMA routines

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

    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);   
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
        for(int i=1000;i<1800;i=i+4) {
          channels[0] = channels[0] + g_ADCBuffer[i+0];
          channels[1] = channels[1] + g_ADCBuffer[i+1];
          channels[2] = channels[2] + g_ADCBuffer[i+2];
          channels[3] = channels[3] + g_ADCBuffer[i+3];
        }
      
        channels[0] = channels[0] / 200;
        channels[1] = channels[1] / 200;
        channels[2] = channels[2] / 200;
        channels[3] = channels[3] / 200;

        if(abs(ADCchannelValues[0]-channels[0])>1) {
          ADCchannelValues[0] = channels[0];
          hv_sendFloatToReceiver(context, hv_stringToHash("thefloat"), ADCchannelValues[0] / 400);
        }
        if(abs(ADCchannelValues[1]-channels[1])>1) {ADCchannelValues[1] = channels[1];}
        if(abs(ADCchannelValues[2]-channels[2])>1) {ADCchannelValues[2] = channels[2];}
        if(abs(ADCchannelValues[3]-channels[3])>1) {ADCchannelValues[3] = channels[3];}
      
      
      BSP_LED_Toggle(LED_GREEN);
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