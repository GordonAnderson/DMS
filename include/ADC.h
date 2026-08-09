#ifndef ADC_h
#define ADC_h
#include <Arduino.h>

#define ADCsync while(adc->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB);
#define ADC0sync while(ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB);
#define ADC1sync while(ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB);
#define MAXADC 4095

extern int   ADCmode[2];
extern int   LowerLimit[2];
extern int   UpperLimit[2];
extern int   RepeatCount[2];
extern int   Threshold[2];
extern int   LastADCval[2];

// ADC acquire setting
extern int preScaler;
extern int sampleNum;
extern int sampLen;

void ADCchangeDet(Adc *adc);
void ADCreset(Adc *adc);
void ADC0attachInterrupt(void (*isr)(bool));
void ADC1attachInterrupt(void (*isr)(bool));

#endif
