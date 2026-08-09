#ifndef Timers_h
#define Timers_h

void pwmWrite(int pin, int value);
void initPWM(void);
void tc1configure(int frequency, int duty);
void tc3Configure(int samplePeriod, void(* callback) (void));

void tcReset(Tc *tc);
void setFreqDuty(int frequency, int duty);

#endif
