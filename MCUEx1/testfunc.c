#include "stm32f4xx_hal.h"
#include "testfunc.h"
#include <stdio.h>

uint8_t bLedOn = PIN_SET_OFF;
uint8_t bStartBlink = PIN_SET_OFF;
uint8_t bStartShift = PIN_SET_OFF;
uint8_t bStartShiftRound = PIN_SET_OFF;
uint8_t blinkNum = 0;
uint8_t directLeft = 1;
uint8_t bBlock = 0;

uint8_t TestAppMode = TESTAPP_SHIFT;

void BlinkStart(uint8_t bStart);

void doTurnOnProc(void);
void doTurnOffProc(void);

uint8_t TestLed = 0;
void DoTest1Led(void);
void DoTest2Led(void);

//-----------------------------------------
uint32_t dacVal;
void doDACTest(void);  
void setDACTest(uint8_t bOn);

#define DEF_DAC_VAL 2000



//-----------------------------------------

uint32_t adcVal;
uint32_t ReadAdcVal(void);
void setADC1Test(uint8_t bOn);
void doADC1Test(void);



//-----------------------------------------

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{



}
//-------------------------------------------

uint8_t ReadButton(uint32_t Btn_Pin)
{
  uint8_t ret = BTN_STATE_OFF;
  switch(Btn_Pin)
  {
  case TURN_ON_BTN:
    ret = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15);
    break;
  case TURN_OFF_BTN:
    ret = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12);
    break;
  }
  //HAL_Delay(100);
   return ret;
}

void SetLedOn(uint32_t LED_Pin, uint8_t bOn )
{
  switch(LED_Pin) {
  case 0:
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, (GPIO_PinState)bOn);
      break;
  case 1:
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, (GPIO_PinState)bOn);
      break;
  case 2:
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, (GPIO_PinState)bOn);
      break;
  case 3:
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, (GPIO_PinState)bOn);
      break;
  case 4:
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, (GPIO_PinState)bOn);
      break;
  case 5:
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, (GPIO_PinState)bOn);
      break;
  case 6:
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, (GPIO_PinState)bOn);
      break;
  case 7:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (GPIO_PinState)bOn);
      break;
  }

}


void setAllLed(uint8_t bOn)
{
  SetLedOn(0, bOn);
  SetLedOn(1, bOn);
  SetLedOn(2, bOn);
  SetLedOn(3, bOn);
  SetLedOn(4, bOn);
  SetLedOn(5, bOn);
  SetLedOn(6, bOn);
  SetLedOn(7, bOn);
}

//-------------------------------------------


void doBtnProc(void)
{
    uint8_t bBtnOn = ReadButton(TURN_ON_BTN);
    //printf("bBtnOn:%d\n", bBtnOn);
    if(bBtnOn == BTN_STATE_ON) {
      doTurnOnProc();
    }
    uint8_t bBtnOff = ReadButton(TURN_OFF_BTN);
    //printf("bBtnOff:%d\n", bBtnOff);
    if(bBtnOff == BTN_STATE_ON) {
      doTurnOffProc();
    }

}

//-------------------------------------------

void doTurnOnProc(void)
{
    switch (TestAppMode) { 
    case TESTAPP_ONOFF_ALL:
      bLedOn = PIN_SET_ON;
      setAllLed(bLedOn);
      break; 
    case TESTAPP_SHIFT:
        setLedShiftTest(PIN_SET_ON);
        break;                 
    case TESTAPP_ADC:
        //setADC1Test(PIN_SET_ON);
        break;          
    default:
        setAllLed(PIN_SET_OFF);
        break;          
    }   
}

void doTurnOffProc(void)
{
    switch (TestAppMode) { 
    case TESTAPP_ONOFF_ALL:
      bLedOn = PIN_SET_OFF;
      setAllLed(bLedOn);
      break;
    case TESTAPP_SHIFT:
        setLedShiftTest(PIN_SET_OFF);
        break;                 
    case TESTAPP_ADC:
        //setADC1Test(PIN_SET_OFF);
        break;  
    default:
        setAllLed(PIN_SET_OFF);
        break;          
    }   
}

void doTestProc(void)
{ 
    switch (TestAppMode) { 
    case TESTAPP_ONOFF_ALL:
        setAllLed(bLedOn);
        break;   
    case TESTAPP_SHIFT:
        doLedShiftTest();
        break;             
    case TESTAPP_ADC:
        //doADC1Test();
        break; 
    case TESTAPP_DAC:
        //doDACTest();
        break; 
    default:
        setAllLed(PIN_SET_OFF);
        break;
    }
    bBlock=0;
}

//-------------------------------------------

uint8_t ledvar = 1;

void printbin(uint8_t input)
{
  int mask;
  for(int i =  7; i >=0 ; i--)
  {
    mask = 1<<i;
    printf("%d", input & mask? 1 : 0);
  }
  printf("\n");
}

void setLedBin(uint8_t input)
{
  int mask;
  setAllLed(PIN_SET_OFF);
  for (int i = 7; i >= 0; i--)
  {
      mask = 1 << i;
      if(input & mask)
        SetLedOn(i, PIN_SET_ON);
  }
}
      
void setLedShiftTest(uint8_t bOn)
{ 
  bStartShift = bOn;
  ledvar = 1;
  if(bStartShift == PIN_SET_OFF)
  {
      setAllLed(PIN_SET_OFF);
  }

}

void doLedShiftTest(void)
{
  if(bStartShift == PIN_SET_ON)
  {
      printf("LED : %d\n", ledvar);
      printbin(ledvar);
      setLedBin(ledvar);
      
      if (ledvar < 128) {
          ledvar <<= 1;
      }  
      else {
           ledvar = 1; 
      }

      HAL_Delay(100);   
  }
}

void setLedShiftRoundTest(uint8_t bOn)
{ 

}

void doLedShiftRoundTest(void)
{



























}
  

//---------------------------------------------------------