#include <Arduino.h>
#include "wiring_private.h"
#include "Timers.h"

//
// Timers.cpp
//
// SAMD51 timer/PWM driver used for three unrelated jobs:
//  - pwmWrite()/initPWM(): eight 12-bit PWM channels (TCC0/1/2) that act as
//    the system's "DAC" outputs (drive level, DCB reference, DCB1/DCB2,
//    electrometer offset/zero) — see the DACchan definitions in DMS.cpp.
//  - tc1configure()/setFreqDuty(): TC1 generates the FAIMS RF drive
//    frequency/duty waveform on the PULSE pin.
//  - tc3Configure(): TC3 is a general-purpose periodic interrupt source used
//    to time CV scan steps (see CVscanISR() in DMS.cpp).
//

void pwmWrite(int pin, int value)
{
  Tcc *tcc;
  int ch;

  switch (pin)
  {
    case 4:
      tcc = TCC2;
      ch = 0;
      break;
    case 5:
      tcc = TCC2;
      ch = 1;
      break;
    case 7:
      tcc = TCC1;
      ch = 2;
      break;
    case 9:
      tcc = TCC1;
      ch = 3;
      break;
    case 10:
      tcc = TCC0;
      ch = 0;
      break;
    case 11:
      tcc = TCC0;
      ch = 1;
      break;
    case 12:
      tcc = TCC0;
      ch = 3;
      break;
    case 13:
      tcc = TCC0;
      ch = 2;
      break;
    default:
    return;
  }
  value >>= 4;
  if(value <= 5) value = 5;
  if(value >= 4095) value = 4094;
  while (tcc->SYNCBUSY.bit.ENABLE);
  tcc->CCBUF[ch].reg = value;
}

void configureTCC(Tcc * tcc, int ch, int pin)
{
  if(pin <= 9) pinPeripheral(pin, PIO_TIMER_ALT);
  else pinPeripheral(pin, PIO_TCC_PDEC);
  tcc->CTRLA.bit.ENABLE = 0;
  while (tcc->SYNCBUSY.bit.ENABLE);
  //TCC1->WEXCTRL.bit.OTMX = 2;
  tcc->CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 |        
                    TC_CTRLA_PRESCSYNC_PRESC;                  
  tcc->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  while (tcc->SYNCBUSY.bit.WAVE);
  tcc->PER.reg = 4096;  
  while (tcc->SYNCBUSY.bit.PER);   
  tcc->CC[ch].reg = 2047;
  tcc->CTRLA.bit.ENABLE = 1;
  while (tcc->SYNCBUSY.bit.ENABLE);
}
// Set up the PWM channels used for analog output
// VARIANT_MCK is clock frequency
void initPWM(void)
{
// Configure clock generators for TCCx
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |                // Divide the 120MHz clock source by divisor 1: 120MHz/1 = 120MHz
                         GCLK_GENCTRL_IDC |                   // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |                 // Enable GCLK7
                         //GCLK_GENCTRL_SRC_DFLL;             // Select 48MHz DFLL clock source
                         GCLK_GENCTRL_SRC_DPLL1;              // Select 100MHz DPLL clock source
                         //GCLK_GENCTRL_SRC_DPLL0;            // Select 120MHz DPLL clock source
  GCLK->PCHCTRL[TCC0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |       // Enable the TCC0 perhipheral channel
                                    GCLK_PCHCTRL_GEN_GCLK7;   // Connect generic clock 7 to TCC1
  GCLK->PCHCTRL[TCC1_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |       // Enable the TCC1 perhipheral channel
                                    GCLK_PCHCTRL_GEN_GCLK7;   // Connect generic clock 7 to TCC1
  GCLK->PCHCTRL[TCC2_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |       // Enable the TCC2 perhipheral channel
                                    GCLK_PCHCTRL_GEN_GCLK7;   // Connect generic clock 7 to TCC1

// Configure TCC2/WO[0] for P4
  configureTCC(TCC2, 0, 4);
// Configure TCC2/WO[1] for P5
  configureTCC(TCC2, 1, 5);
// Configure TCC1/WO[2] for P7
  configureTCC(TCC1, 2, 7);
// Configure TCC1/WO[3] for P9
  configureTCC(TCC1, 3, 9);
// Configure TCC0/WO[0] for P10
  configureTCC(TCC0, 0, 10);
// Configure TCC0/WO[1] for P11
  configureTCC(TCC0, 1, 11);
// Configure TCC0/WO[3] for P12
  configureTCC(TCC0, 3, 12);
// Configure TCC0/WO[2] for P13
  configureTCC(TCC0, 2, 13);
}

bool tcIsSyncing(Tc *tc)
{
  return tc->COUNT16.SYNCBUSY.reg & (TC_SYNCBUSY_SWRST | TC_SYNCBUSY_ENABLE | TC_SYNCBUSY_CTRLB | TC_SYNCBUSY_STATUS | TC_SYNCBUSY_COUNT | TC_SYNCBUSY_PER | TC_SYNCBUSY_CC0 | TC_SYNCBUSY_CC1);
}

//This function enables TC and waits for it to be ready
void tcStartCounter(Tc *tc)
{
  tc->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing(tc)); //wait until snyc'd
}

//Reset TC
void tcReset(Tc *tc)
{
  tc->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing(tc));
  while (tc->COUNT16.CTRLA.bit.SWRST);
}

//disable TC
void tcDisable(Tc *tc)
{
  tc->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing(tc));
}

void setFreqDuty(int frequency, int duty)
{
  // Guard against divide-by-zero: SFREQ range-checks dms.Freq before it gets
  // here, but the TC1 debug command passes its argument straight through
  // with no range check, so a bad value (e.g. TC1,0,50) must not reach the
  // division below.
  if(frequency <= 0) return;
  // Set frequency
  TC1->COUNT16.CC[0].reg = VARIANT_MCK / frequency;
  while (tcIsSyncing(TC1));  // was TC2 - TC2 isn't touched here, so that wait was a no-op
  // Set duty cycle
  TC1->COUNT16.CC[1].reg = ((VARIANT_MCK / frequency) * duty) / 100;
  while (tcIsSyncing(TC1));
}

void tc1configure(int frequency, int duty)
{
  pinPeripheral(2, PIO_TIMER);
// Configure clock generators for 120MHz TCCx
  GCLK->GENCTRL[6].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 120MHz clock source by divisor 1: 120MHz/1 = 120MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK6
                         GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
// Enable GCLK for TC1 (timer counter input clock)
  GCLK->PCHCTRL[TC1_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |        // Enable the TC perhipheral channel
                                   GCLK_PCHCTRL_GEN_GCLK6;    // Connect generic clock 6 to TC1

  tcReset(TC1); //reset TC1

  // Set Timer counter Mode to 16 bits
  TC1->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (tcIsSyncing(TC1));
  // Set TC1 mode as match frequency
  TC1->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MPWM_Val;
  while (tcIsSyncing(TC1));
  // Set prescaler to 1
  TC1->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
  while (tcIsSyncing(TC1));
  // Set frequency and duty cycle
  setFreqDuty(frequency,duty);
  // Start the counter
  tcStartCounter(TC1);
  while (tcIsSyncing(TC1));
}

//
// Timer code used to support scan timer interrupt generation.
// Adapted from: https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f
//

void(* callback_func) (void) = NULL;

//this function gets called by the interrupt at <sampleRate>Hertz
void TC3_Handler (void) 
{
  if(callback_func != NULL) callback_func();
  TC3->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
}

//Configures the TC to generate output events at the samplePeriod.
//Configures the TC in Frequency Generation mode, with an event output once
//each period.
void tc3Configure(int samplePeriod, void(* callback) (void))  // samplePeriod in mS
{
 callback_func = callback;
// Configure clock generators for 120MHz TCCx
  GCLK->GENCTRL[6].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 120MHz clock source by divisor 1: 120MHz/1 = 120MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK6
                         GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
// Enable GCLK for TC3 (timer counter input clock)
  GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |        // Enable the TC3 perhipheral channel
                                   GCLK_PCHCTRL_GEN_GCLK6;    // Connect generic clock 6 to TC3

 tcReset(TC3); //reset TC3

 // Set Timer counter Mode to 16 bits
 TC3->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
 // Set TC3 mode as match frequency
 TC3->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ_Val;
 // Determine and set prescaler and enable TC3
 int targetCount = (VARIANT_MCK / 1000) * samplePeriod;
 if((targetCount /= 1) <= 65535) TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
 else if((targetCount /= 2) <= 65535) TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2 | TC_CTRLA_ENABLE;
 else if((targetCount /= 2) <= 65535) TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV4 | TC_CTRLA_ENABLE;
 else if((targetCount /= 2) <= 65535) TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_ENABLE;
 else if((targetCount /= 2) <= 65535) TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16 | TC_CTRLA_ENABLE;
 else if((targetCount /= 4) <= 65535) TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV64 | TC_CTRLA_ENABLE;
 else if((targetCount /= 4) <= 65535) TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256 | TC_CTRLA_ENABLE;
 else if((targetCount /= 4) <= 65535) TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE;
 //set TC3 timer counter
 TC3->COUNT16.CC[0].reg = targetCount; 
 // Configure interrupt request
 NVIC_DisableIRQ(TC3_IRQn);
 NVIC_ClearPendingIRQ(TC3_IRQn);
 NVIC_SetPriority(TC3_IRQn, 0);
 NVIC_EnableIRQ(TC3_IRQn);

 // Enable the TC3 interrupt request
 TC3->COUNT16.INTENSET.bit.MC0 = 1;
 while (tcIsSyncing(TC3)); //wait until TC3 is done syncing 
} 

