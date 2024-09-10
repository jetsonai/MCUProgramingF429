#ifndef _TEST_FUNC
#define _TEST_FUNC

#define DEBUG

#ifdef DEBUG
#define PRINTF printf
#else
#define PRINTF(...) do {} while(0)
#endif

#define PIN_SET_ON  GPIO_PIN_SET
#define PIN_SET_OFF GPIO_PIN_RESET

#define BTN_STATE_ON  GPIO_PIN_RESET
#define BTN_STATE_OFF GPIO_PIN_SET

#define TURN_ON_BTN    10
#define TURN_OFF_BTN   11
#define CHANGE_STATE_BTN   12

extern uint8_t TestSetMode;

#define TESTSET_BTN_INTER       100  /*must set btn pin GPIO_INTERRUPT and regenerate code!!"*/
#define TESTSET_BTN_READ        101  /*must set btn pin GPIO_IN andd regenerate code!!"*/

extern uint8_t TestAppMode;

extern uint8_t bStartBlink;

#define TESTAPP_NONE            99u    
#define TESTAPP_ONOFF_ALL       100u   
#define TESTAPP_TOGGLE_ALL      102u
#define TESTAPP_SHIFT           103u
#define TESTAPP_TIMER            110u
#define TESTAPP_UART            111u
#define TESTAPP_I2C            112u
#define TESTAPP_SPI            113u
#define TESTAPP_ADC            114u
#define TESTAPP_DAC            118u
#define TESTAPP_MAX             205u

#define LED_CNT 8


void SetLedOn(uint32_t LED_Pin, uint8_t bOn );

uint8_t ReadButton(uint32_t Btn_Pin);

void setBlinkStart(uint8_t bStart);

void BlinkTest(void);

void setAllLed(uint8_t bOn);

void setAllToggleLed(uint8_t bOn);

void doToggleTest(void);


void setLedShiftTest(uint8_t bOn);
void setLedShiftRoundTest(uint8_t bOn);

void doLedShiftTest(void);

void doLedShiftRoundTest(void);

void doBtnProc(void);

void doTestProc(void);

void printbin(uint8_t input);

void setLedBin(uint8_t input);

void doUartTest1(void);


#endif //_TEST_FUNC