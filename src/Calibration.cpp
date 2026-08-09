#include "Calibration.h"
#include "DMS.h"

//
// Calibration.cpp
//
// Interactive, two-point calibration routines run from the serial command
// interface (CALDCBREF, CALDCB1, CALDCB2, CALVRF, CALDRV2VRF). Each routine
// drives a DAC output to two known setpoints, prompts the operator to enter
// the actual value measured with external test equipment (a voltmeter or
// scope), and computes/stores the resulting linear slope+intercept (m, b)
// calibration pair on the relevant ADCchan/DACchan in `dms`. Results are not
// persisted automatically — use SAVECAL (see saveCalibrations() in DMS.cpp)
// to write them to the flash filesystem.
//

// Pumps the command processor while blocked waiting on operator input
// (cp.userInputFloat's callback), so the command interface stays responsive.
void CalibrateLoop(void)
{
  cp.processStreams();
}

// Drives dacchan (if given) to the requested value *V, prompts the operator
// for the actual measured value (updating *V in place), and returns the
// averaged raw ADC counts read back on adcchan (if given), or 0 if adcchan
// is NULL.
int CalibratePoint(DACchan *dacchan, ADCchan *adcchan, float *V)
{
  // Set value and ask for user to enter actual value read
  if(dacchan !=NULL) DACupdate(*V, dacchan);
  *V = cp.userInputFloat("Enter actual value: ",CalibrateLoop);
  if(adcchan != NULL)
  {
    int sum = 0;
    for(int i=0;i<10;i++) sum += readadc(adcchan->Chan);
    return sum/10;
  }
  return 0;
}

// Generic two-point calibration: drives dacchan to V1 then V2, prompts the
// operator for the actual measured value at each point, and derives/stores
// linear (counts = m*value + b) calibration parameters for dacchan and (if
// given) the associated adcchan readback channel.
void Calibrate(DACchan *dacchan, ADCchan *adcchan, float V1, float V2)
{
  float  val1,val2,m,b;
  int    adcV1, adcV2;
  int    dacV1, dacV2;

  cp.println("Enter values when prompted.");
  // Set to first voltage and ask for user to enter actual voltage
  val1 = V1;
  adcV1 = CalibratePoint(dacchan, adcchan, &val1);
  cp.println(adcV1);
  cp.println(readadc(adcchan->Chan));
  // Set to second voltage and ask for user to enter actual voltage
  val2 = V2;
  adcV2 = CalibratePoint(dacchan, adcchan, &val2);
  cp.println(adcV2);
  cp.println(readadc(adcchan->Chan));
  // Calculate calibration parameters and apply
  dacV1 = Value2Counts(V1, dacchan);
  dacV2 = Value2Counts(V2, dacchan);
  m = (float)(dacV2-dacV1) / (val2-val1);
  b = (float)dacV1 - val1 * m;
  cp.println("DAC channel calibration parameters.");
  cp.print("m = ");
  cp.println(m);
  cp.print("b = ");
  cp.println(b);
  dacchan->m = m;
  dacchan->b = b;
  if(adcchan == NULL) return;
  m = (float)(adcV2-adcV1) / (val2-val1);
  b = (float)adcV1 - val1 * m;
  cp.println("ADC channel calibration parameters.");
  cp.print("m = ");
  cp.println(m);
  cp.print("b = ");
  cp.println(b);
  adcchan->m = m;
  adcchan->b = b;
}

void CalibrateREF(void)
{
  cp.println("Calibrate DCB reference output, monitor with a voltmeter.");
  Calibrate(&dms.DCBREFctrl, NULL, 0.0, 1.25);
  DACupdate(1.25, &dms.DCBREFctrl);
}

void CalibrateDCB1(void)
{
  cp.println("Calibrate DCB1 output, monitor with a voltmeter.");
  Calibrate(&dms.DCB1ctrl, &dms.DCB1RBmon, 0.0, 12.0);
  DACupdate(dms.DCB1v, &dms.DCB1ctrl);
}
void CalibrateDCB2(void)
{
  cp.println("Calibrate DCB2 output, monitor with a voltmeter.");
  Calibrate(&dms.DCB2ctrl, &dms.DCB2RBmon, 0.0, 12.0);
  DACupdate(dms.DCB2v, &dms.DCB2ctrl);
}

void CalibrateVrf(void)
{
  float Vrf1, Vrf2;
  int   adc1,adc2;

  cp.println("Calibrate Vrf readback, monitor Vrf with a scope.");
  DACupdate(10, &dms.DRIVEctrl);
  Vrf1 = cp.userInputFloat("Enter Vrf actual value: ",CalibrateLoop);
  adc1 = 0;
  for(int i = 0;i<1024;i++)
  {
    adc1 += analogRead(RFLEVEL);
    delay(1);
  }
  // Dividing the sum of 1024 raw 12-bit analogRead() samples by 64 (rather
  // than by 1024) computes the average and rescales it to 16 bits in one
  // step: sum/64 == (sum/1024)*16, matching the same x16 (<<4) scaling that
  // readadc() applies elsewhere, with less rounding error than dividing
  // first and multiplying after.
  adc1 /= 64;
// Set drive level to 50% and ask for Vrf voltage
  DACupdate(50, &dms.DRIVEctrl);
  Vrf2 = cp.userInputFloat("Enter Vrf actual value: ",CalibrateLoop);
  adc2 = 0;
  for(int i = 0;i<1024;i++)
  {
    adc2 += analogRead(RFLEVEL);
    delay(1);
  }
  adc2 /= 64;  // Same average+16x-rescale trick as adc1 above.
// Calculate the calibration parameters
  // ADC1 = Vrf1*m + b
  // ADC2 = Vrf2*m + b
  // m = (ADC1 - ADC2) / (Vrf1-Vrf2)
  // b = ADC2 - Vrf2 * m;
  dms.RFLEVELmon.m =  (adc1 - adc2) / (Vrf1-Vrf2);
  dms.RFLEVELmon.b =  adc2 - Vrf2 * dms.RFLEVELmon.m;
  cp.println("ADC channel calibration parameters.");
  cp.print("m = ");
  cp.println(dms.RFLEVELmon.m);
  cp.print("b = ");
  cp.println(dms.RFLEVELmon.b);
// Restore drive level
  DACupdate(dms.Drive, &dms.DRIVEctrl);
  delay(100);
}

// Scan through drive level 0, to Max Drive in 11 steps and record the Vrf
// voltage. This will be used in the step function to quickly set the
// desired Vrf level.
void CalibrateVrf2Drive(void)
{
  if(!dms.Enable) return;   // Exit if system is not enabled
  for(int i=0;i<21;i++)
  {
    // Set the drive level
    DACupdate(i*(dms.MaxDrive/20), &dms.DRIVEctrl);
    // Wait for things to stabalize
    delay(250);
    // Read the Vrf level
    rb.Vrf = -1;
    for(int j=0;j<100;j++) 
    {
      float fval = AnalogIn(readadc, &dms.RFLEVELmon,4);
      rb.Vrf = Filter(rb.Vrf, fval);
    }
    // Save data in lookup table
    dms.LUVrf[i] = rb.Vrf;
    //cp.println(rb.Vrf);
  }
// Restore drive level
  DACupdate(dms.Drive, &dms.DRIVEctrl);
  delay(100);
}
