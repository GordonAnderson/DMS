#include "ADC.h"
#include "wiring_private.h"

//
// ADC.cpp
//
// Low-level SAMD51 ADC0/ADC1 driver used for CV scan data acquisition.
//
// During a CV scan the two electrometer channels (positive on ADC0/A0,
// negative on ADC1/A3) are put into free-running "window compare" mode via
// ADCchangeDet(). Each conversion fires the ADCx_1_Handler ISR below, which
// stores the raw result in LastADCval[] and drives a simple debounce filter
// (RepeatCount consecutive readings) before invoking an optional user
// callback (ADC0/1attachInterrupt) when the signal crosses in/out of the
// configured window. This is used elsewhere (DMS.cpp/CVscanISR) purely to
// latch LastADCval[] on every conversion; the window/callback machinery is
// mostly vestigial for that use case but is left in place since it's part
// of the shared ADC driver.
//
// A0 = PA02/AIN0
// A1 = PA05/AIN5

int   ADCmode[2]     = {0,0};
// ADC window modes
// 0 = No window mode
// 1 = ADC value > LowerLimit
// 2 = ADC value < UpperLimit
// 3 = ADC value is within the window defined by UpperLimit and LowerLimit
// 4 = ADC value is outside the window defined by UpperLimit and LowerLimit
// 5 = ADC value changed
int   LowerLimit[2]  = {2000,2000};
int   UpperLimit[2]  = {2500,2500};
int   Threshold[2]   = {6,6};
int   RepeatCount[2] = {6,6};
int   LastADCval[2]  = {0,0};
int   ISRcount[2]    = {0,0};
void  (*ADC0changeFunc)(bool) = NULL;
void  (*ADC1changeFunc)(bool) = NULL;
// ADC acquire setting
int preScaler = 1;
int sampleNum  = 10;
int sampLen    = 20;


// ADC change interrupt call back function
void ADC0attachInterrupt(void (*isr)(bool))
{
  ADC0changeFunc = isr;
}

void ADC1attachInterrupt(void (*isr)(bool))
{
  ADC1changeFunc = isr;
}

// This ADC ISR fires on every ADC conversion. This routine looks at the window flag 
// and will call the attached interrupt when the window condition changes. A repeat count
// filter is applied to filter false signals.
void ADC0_1_Handler(void)
{
   // int (not unsigned) to match RepeatCount[]'s type and avoid
   // sign-compare warnings; values here are always small and non-negative.
   volatile int static count  = 0;
   volatile int static countL = 0;
   int t;

   volatile int intflag = ADC0->INTFLAG.bit.WINMON;
   ADC0sync;
   //ISRcount++;
   LastADCval[0] = ADC0->RESULT.reg;
   //serial->println(LastADCval[0]);
   ADC0sync;
   if(intflag == 1)
   {
      if(++countL >= RepeatCount[0])
      {
        // If here the limit has been exceeded for 
        // RepeatCount readings in a row
        count = 0;
        if(countL == RepeatCount[0])
        {
          if(ADC0changeFunc != NULL) ADC0changeFunc(true);
        }
        if(countL > 2*RepeatCount[0]) countL--;
      }    
   }
   else
   {
      if(++count >= RepeatCount[0])
      {
        // If here the ADC value is within the limit for 
        // RepeatCount readings in a row
        if(ADCmode[0] == 5)
        {
          if((t = LastADCval[0] - Threshold[0]) < 0) t = 0;
          ADC0sync;
          ADC0->WINLT.reg = t; 
          if((t = LastADCval[0] + Threshold[0]) > MAXADC) t = MAXADC;
          ADC0sync;
          ADC0->WINUT.reg = t;
        }
        countL = 0;
        if((count == RepeatCount[0]) && (ADC0changeFunc != NULL)) ADC0changeFunc(false);
        if(count > 2*RepeatCount[0]) count--;
      }    
   }
}

void ADC1_1_Handler(void)
{
   // int (not unsigned) to match RepeatCount[]'s type and avoid
   // sign-compare warnings; values here are always small and non-negative.
   volatile int static count  = 0;
   volatile int static countL = 0;
   int t;

   volatile int intflag = ADC1->INTFLAG.bit.WINMON;
   ADC1sync;
   //ISRcount++;
   LastADCval[1] = ADC1->RESULT.reg;
   //serial->println(LastADCval[1]);
   ADC1sync;
   if(intflag == 1)
   {
      if(++countL >= RepeatCount[1])
      {
        // If here the limit has been exceeded for 
        // RepeatCount readings in a row
        count = 0;
        if(countL == RepeatCount[1])
        {
          if(ADC1changeFunc != NULL) ADC1changeFunc(true);
        }
        if(countL > 2*RepeatCount[1]) countL--;
      }    
   }
   else
   {
      if(++count >= RepeatCount[1])
      {
        // If here the ADC value is within the limit for 
        // RepeatCount readings in a row
        if(ADCmode[1] == 5)
        {
          if((t = LastADCval[1] - Threshold[1]) < 0) t = 0;
          ADC1sync;
          ADC1->WINLT.reg = t; 
          if((t = LastADCval[1] + Threshold[1]) > MAXADC) t = MAXADC;
          ADC1sync;
          ADC1->WINUT.reg = t;
        }
        countL = 0;
        if((count == RepeatCount[1]) && (ADC1changeFunc != NULL)) ADC1changeFunc(false);
        if(count > 2*RepeatCount[1]) count--;
      }    
   }
}

//const PinDescription my_g_APinDescription[]=
//{
//  { PORTA,  5, PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },
//};

// This function enables the ADC change detection system. The variables:
//   ADCmode
//   LowerLimit
//   UpperLimit
// need to be set before calling this function.  
void ADCchangeDet(Adc *adc)
{
   int i = 0;
   
   if(adc == ADC1) i = 1;
   if(adc == ADC0) pinPeripheral(A0, PIO_ANALOG);
   if(adc == ADC1) pinPeripheral(A3, PIO_ANALOG);
   ADCsync;
   adc->CTRLA.bit.ENABLE = 0;
   ADCsync;
   if(adc == ADC0) adc->INPUTCTRL.bit.MUXPOS = 0;
   if(adc == ADC1) adc->INPUTCTRL.bit.MUXPOS = 1;
   ADCsync;
   // Controls conversion rate, minimum value for 12 bits is 2. 4 = 62,500 sps, 2 = 250,000 sps.
   // This assumes clock is 48MHz. 
   // 1.54mS period if Prescale = 1, 1024 samples, 16 bits, SAMPLEN=5
   //
   // Default seetings:
   // PRESCALER = 1
   // SAMPLENUM = 10
   // SAMPLEN = 20
   // Each conversion requires 33 clock, 21 for sampling and 12 for conversion.
   // 2.816mS conversion period. SAMPLEN max is 63 resulting in 64 clock wide sample window
   // and 6.4mS period.
   // Set SAMPLEN to 44 to result in 5mS period, may be ideal for 10mS scan period.
   // Should improve noise performance, 44 = sample window of 3.75uS, still pretty small.

   // Try the following option:
   // PRESCALER = 4, 1.5MHz
   // SAMPLENUM = 7, 128 samples
   // SAMPLEN = 63
   // 6.4mS update rate, 42uS sampling window

   // 
   adc->CTRLA.bit.SLAVEEN = 0;
   ADCsync;
   adc->CTRLA.bit.PRESCALER = preScaler;
   ADCsync;
   adc->CTRLA.bit.ENABLE = 1;
   ADCsync;
   adc->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
   ADCsync;
   adc->AVGCTRL.bit.SAMPLENUM = sampleNum;
   ADCsync;
   adc->AVGCTRL.bit.ADJRES = 0;
   ADCsync;
   adc->CTRLB.bit.FREERUN = 1;
   ADCsync;
   adc->SAMPCTRL.bit.SAMPLEN = sampLen;  // Sampling time in clock pulses
   ADCsync;
   if(ADCmode[i] == 5) adc->CTRLB.bit.WINMODE = 3;
   else adc->CTRLB.bit.WINMODE = ADCmode[i];
   ADCsync;
   adc->INTENSET.bit.RESRDY = 1;
   ADCsync;
   adc->WINLT.reg = LowerLimit[i]; 
   ADCsync;
   adc->WINUT.reg = UpperLimit[i];
   // enable interrupts
   if(adc == ADC0) NVIC_EnableIRQ(ADC0_1_IRQn);
   if(adc == ADC1) NVIC_EnableIRQ(ADC1_1_IRQn);
   // Start adc
   ADCsync;
   adc->SWTRIG.bit.START = 1;
}

void ADCreset(Adc *adc)
{
  if(adc == ADC0) NVIC_DisableIRQ(ADC0_1_IRQn);
  if(adc == ADC1) NVIC_DisableIRQ(ADC1_1_IRQn);
  ADCsync;
  if(adc == ADC0) adc->INPUTCTRL.reg = 0x1806;
  if(adc == ADC1) adc->INPUTCTRL.reg = 0x1800;
  ADCsync;
  adc->CTRLA.reg = 0x400;
  ADCsync;
  adc->CTRLB.reg = 0;
  ADCsync;
  adc->AVGCTRL.reg = 0;
  ADCsync;
  adc->SAMPCTRL.reg = 5;
}

