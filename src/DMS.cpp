//
// DMS
//
// ATSAMD51 32-bit Cortex M4 core running at 120 MHz, 512KB Flash and 192KB RAM.
//
// Hardware details:
//  - The DMS system used PWM to analog output DAC, 12 bit. The TCC timers are used
//    to generate eight 12 resolution output PWM channels.
//  - All analog inputs use the processors 12 bit ADCs.
//  - The ADC for the electrometer channels are setup for over sampling during
//    a CV scan.
//  - Use TinyUBS stack when building application
//  - Use Itsybitsy M4 board when building
//  - Supports flash drive interface allowing the user to read data files from the
//    disk interface. 2MB drive used to save calibration and setup data as well as
//    scan data.
//
// Scanning details
//  - Scanning support two dimensional scans, CV scans over a defined Vrf range.
//  - All scan data is saved in memory.
//  - Scaning saves raw CV adc counts.
//  - Commands alow reading the scan data while it is being acquired. Raw counts are
//    converted to pA when reported to host.
//  - Scan can be aborted.  
//  - Scan can be saved to the file system and then reloaded.
//  - Multiple CV scans can be averaged but only the average is saved.
//
// read scan data function
// - readelecpos,scan num,scan point,numvalues
// - readelecneg,scan num,scan point,numvalues
//    - These read functions apply the calibration parameters
// - numscans
// - scanpoints,scan num
// - scaninfo,returns the status
//
// Scans can be saved to a file
// - savescan,filename
// - loadscan,filenam
// - listscans
//
// To do:
// - need to add scan average option
// - support scheduled scans (done)
// - move all scan code to new file
// - support flash drive emulation to access scan files (done)
// - Add time functions (done)
// - Add battery voltage monitor (done)
// - Add file functions (done)
//    - dir, del
//    - load scan file
// Requests from Dylan
//  - auto zero / calibration of the electrometers.
//  - The idea is when you are continuously scanning you know when you are seeing 
//    compounds with the intensity going up. It would be nice if there was a flag 
//    at those intensity points. 
//  - Additionally another idea could be running in a continuous low resolution scan 
//    mode to reduce the file size but then when it detects compounds it goes into a 
//    high resolution scanning mode so that we can hopefully identify the compound.
//
// Version history
//  1.0, Nov 8, 2025
//    - Orginal release
//  1.1, Dec 9, 2025
//    - Fixed electrometer counts data type, was signed and needed to be un signed. 
//      values over 250pA go negative
//  1.2, Feb 2, 2026
//    - Added the ability to set the ADC acquire parameters
//  1.3, Mar 12, 2026
//    - Added the ability to average CV scans
//    - Changed ADC defaults to optomized values from Dylan
//  1.4, Mar 19, 2026
//    - process the enable command when received, not just set the enable flag
//    - Fixed averaging bug when set to 2


#include "Arduino.h"
#include <variant.h>
#include <wiring_private.h>
#include "DMS.h"
#include "ADC.h"
#include "Timers.h"
#include "Calibration.h"
#include <SPI.h>
#include <Wire.h>
#include <Thread.h>
#include <ThreadController.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_DotStar.h>
#include "RTC_SAMD51.h"
#include "DateTime.h"
#include <iostream>
#include <new> 

#include <RingBuffer.h>
#include <commandProcessor.h>
#include <charAllocate.h>
#include <debug.h>
#include <Errors.h>
#include <Devices.h>
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>
#include "Adafruit_TinyUSB.h"
#include <Adafruit_INA237.h>

#include <stdio.h>

#include <SerialBuffer.h>

#undef  USB_MANUFACTURER
#define USB_MANUFACTURER "GAACE"
#undef  USB_PRODUCT
#define USB_PRODUCT      "DMS Rev 1"

const char *Version = "DMS, version 1.4 Mar 19, 2026";
//Adafruit_BME280 bme(BME280_CS);       // hardware SPI
Adafruit_BME280 bme;                    // hardware TWI

#ifndef NOINA237
Adafruit_INA237 ina237 = Adafruit_INA237();
#endif

DMSdata   dms;
ReadBacks rb;

RTC_SAMD51 rtc;

float power   = 0.0;
float current = 0.0;

// These variables are used for the automated perodic scanning.
bool  acquireEnabled    = false;      // this flag is true when scheduled scanning is in process
char  baseFileName[20]  = "";         // base file name, each scan appends .xxxx to this name where xxxx = scan number
int   numSchScans       = 0;          // number of scan to perform
int   interval          = 15;         // seconds between scans
int   collectedScan     = 0;
bool  scanFlag          = false;      // this flag is true during a scan
bool  scanNow           = false;      // this flag is set to flag a scan to start, monitoried in the main loop

// Pin definitions for the onboard DotStar
// On the ItsyBitsy M4, the DotStar is connected to specific pins.
// Refer to Adafruit documentation for exact pin assignments if needed.
// For the onboard DotStar, these are often defined internally by the library.
#define DATAPIN    8 // Example data pin (check documentation for your specific board)
#define CLOCKPIN   6 // Example clock pin (check documentation for your specific board)
#define NUMPIXELS  1 // Only one LED for the onboard DotStar

// Create the DotStar object
Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

DMSdata Rev_1_dms = {
  sizeof(DMSdata),"DMS",1,
  false,1200000,50,0.0,
  0,false,1.0,0.0,0.0,
  2.5,0.0,0.0,
  // Electrometer settings
  0.0,0.0,0.0,0.0,
  // Limits
  70,3,10,10,
  // ADC acquire parameters
  2,10,63,
  // ADC channels, uses processor ADC inputs
  {DCB1RB,899.00,48185.00},
  {DCB2RB,896.07,48227.00},
  {POSELEC,131.07,0},
  {NEGELEC,131.07,0},
  {RFLEVEL,12.93,-777.76},
  // DAC channels, uses 12 bit PWM outputs
  {DRIVE,-655.35,65535},
  {DCBREF,13107,0},
  {DCB1,586.87,31268.00},
  {DCB2,587.63,31291.00},
  {POSOFF,13107,0},
  {POSZERO,13107,0},
  {NEGOFF,13107,0},
  {NEGZERO,13107,0},
  // Scan parameters
  500,1000,-10,10,100,10,1,10,
  // External step advance
  false,0,
  // External scan trigger options
  0,0,
  // Lookup table for drive level to Vrf calibration
  {329.52,330.26,328.18,329.76,330.03,358.00,401.42,446.39,491.80,537.68,582.62,631.42,678.07,726.23,772.87,819.15,867.06,917.51,966.16,1012.99,1064.74
},
  1013.25,
  SIGNATURE
};

// NOTE: these color names look swapped (RED is green-weighted, GREEN is
// red-weighted) — see TODO.md #5. Left as-is pending hardware verification.
#define RED    strip.Color(0, 150, 0)
#define GREEN  strip.Color(150, 0, 0)
#define BLUE   strip.Color(0, 0, 150)
#define BLACK  strip.Color(0, 0, 0)

// Sets the onboard DotStar status LED, skipping the SPI write if the color
// hasn't changed since the last call. Usage in this file: RED = setup /
// low battery, GREEN = idle/running normally, BLUE = scan in progress.
void statusLED(uint32_t color)
{
  static uint32_t cc = 0;

  if(cc==color) return;
  strip.setPixelColor(0, color);
  strip.show();
  cc = color;
}

void statusLED_RED(void) {statusLED(RED);}
void statusLED_GREEN(void) {statusLED(GREEN);};
void statusLED_BLACK(void) {statusLED(BLACK);};


// Macronix MX25V1635F, 2MiB. Entry maintained locally by GAA —
// not present in upstream Adafruit_SPIFlash as of v5.x.
static const SPIFlash_Device_t MX25V1635 = 
{
    .total_size = (1UL << 21), /* 2 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xc2,                     \
    .memory_type = 0x23, .capacity = 0x15, .max_clock_speed_mhz = 80,          \
    .quad_enable_bit_mask = 0x40, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = true, .is_fram = false,                              \
};

/*
 * Create an array of data structures and fill it with the settings we defined
 * above. We are using two devices, but more can be added if you want.
 */
static const SPIFlash_Device_t my_flash_devices[] = {
     MX25V1635,
};
/*
 * Specify the number of different devices that are listed in the array we just
 * created. If you add more devices to the array, update this value to match.
 */
const int flashDevices = 1;


// Reserve a portion of flash memory to store configuration
// Note: the area of flash memory reserved is lost every time
// the sketch is uploaded on the board.
FlashStorage(flash_DMSdata, DMSdata);

// for flashTransport definition
#include <./FlashFS/flash_config.h>
Adafruit_SPIFlash flash(&flashTransport);
Adafruit_SPIFlash *flashFS = &flash;

// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;

// Check if flash is formatted
bool fs_formatted = false;

// Set to true when PC write to flash
bool fs_changed = true;

// Scan parameters
ScanStatus scanStatus  = SCAN_IDLE;
Scan       *scanBuffer = NULL;
float      CVstep;
float      VRFstep;

// file system object from SdFat
FatVolume fatfs;
File32 file;

const int   rngFREQ[2]     = {500000,1500000};
const int   rngDUTY[2]     = {5,95};
const float rngDRIVE[2]    = {0,100};
const float rngCV[2]       = {-24,24};
const float rngVrf[2]      = {500,1500};
const int   rngDUR[2]      = {1,100};
const int   rngSTEPS[2]    = {10,5000};
const int   rngVrfSTEPS[2] = {1,1000};
const float rngELEC[2]     = {0,5};

const int   rngPRESCALER[2] = {0,7};
const int   rngSAMPLENUM[2] = {0,10};
const int   rngSAMPLELEN[2] = {0,63};
const int   rngAverages[2]  = {1,100};

Command dbsCmds[] =
{
  // Main application commands
  {"GVER",   CMDstr,0,(void *)Version,NULL,                      "Firmware version"},
  {"GNAME", CMDstr, 0, (void *)dms.Name,NULL,                    "Report system name"},
  {"SNAME", CMDstr, 1, (void *)dms.Name,NULL,                    "Set system name"},
  {"SAVE",   CMDfunction, 0, (void *)SaveSettings,NULL,          "Save settings"},
  {"RESTORE",CMDfunction, 0, (void *)RestoreSettings,NULL,       "Restore settings"},
  {"FORMAT", CMDfunction, 0, (void *)FormatFLASH,NULL,           "Format FLASH"},
  {"BLOAD", CMDfunction, 0, (void *)bootloader,NULL,             "Jump to bootloader"},
// Time and date functions
  {"STIMEDATE", CMDfunction, 1, (void *)setTimeDate,NULL,         "Set time or date, hh:mm:ss dd/mm/yyyy"},
  {"GTIME", CMDfunction, 0, (void *)getTime,NULL,                 "Return time, hh:mm:ss"},
  {"GDATE", CMDfunction, 0, (void *)getDate,NULL,                 "Return date, dd/mm/yyyy"},
  {"GTIMEDATE", CMDfunction, 0, (void *)getTimeDate,NULL,         "Return date and time, DDD, DD MMM YYYY hh:mm:ss"},
// SPI flash file IO function
  {"SAVEF", CMDfunction, -1, (void *)saveDefaults,NULL,          "Save current setting to flash FS"},
  {"LOADF", CMDfunction, -1, (void *)loadDefaults,NULL,          "Load current setting from flash FS"},
  {"SAVECAL", CMDfunction, 0, (void *)saveCalibrations,NULL,     "Save current calibration data to flash FS"},
  {"LOADCAL", CMDfunction, 0, (void *)loadCalibrations,NULL,     "Load current calibration from flash FS"},
  {"FORMATFS",CMDfunction, 0, (void *)formatFS,NULL,             "Format the flash FS"},
// DMS general commands
  {"ON",  CMDfunction,   0, (void *)powerON,NULL,                "Set Power on"},
  {"OFF",  CMDfunction,  0, (void *)powerOFF,NULL,               "Set Power off"},
  {"SENA",  CMDfunction, 1, (void *)setEnable,NULL,              "Set to TRUE to enable DMS"},
  {"GENA",  CMDbool,     0, (void *)&dms.Enable,NULL,            "Returns enable status"},
  {"SMODE", CMDbool,     1, (void *)&dms.Mode,NULL,              "Set to TRUE to enable closed loop control of Vrf"},
  {"GMODE", CMDbool,     0, (void *)&dms.Mode,NULL,              "Returns mode control"},
  {"SFREQ", CMDint,      1, (void *)&dms.Freq,(void *)rngFREQ,   "Set DMS frequency, in Hz"},
  {"GFREQ", CMDint,      0, (void *)&dms.Freq,NULL,              "Returns frequency"},
  {"SDUTY", CMDint,      1, (void *)&dms.Duty,(void *)rngDUTY,   "Set DMS duty cycle in percent"},
  {"GDUTY", CMDint,      0, (void *)&dms.Duty,NULL,              "Returns duty cycle"},
  {"SDRV",  CMDfloat,    1, (void *)&dms.Drive,(void *)rngDRIVE, "Set DMS drive level in percent"},
  {"GDRV",  CMDfloat,    0, (void *)&dms.Drive,NULL,             "Returns drive level"},
  {"SVRF",  CMDfunction, 1, (void *)SetVrfCmd,NULL,              "Sets the drive level to achieve the desired voltage"},
  {"SVRFF", CMDfunction, 1, (void *)SetVrfTableCmd,NULL,         "Sets the drive level using the look up table"},
  {"GVRF",  CMDfloat,    0, (void *)&dms.Vrf,NULL,               "Returns the Vrf peak voltage setpoint"},
  {"GVRFV",  CMDfloat,    0, (void *)&rb.Vrf,NULL,               "Returns the Vrf peak voltage readback"},
  {"GVBAT",  CMDfloat,    0, (void *)&rb.Vbat,NULL,              "Returns the battery voltage"},
  {"GPWR",  CMDfloat,    0, (void *)&power,NULL,                 "Returns power in watts"},
  {"GCUR",  CMDfloat,    0, (void *)&current,NULL,               "Returns current in amps"},
// ADC acquire setting
  {"?PRE",  CMDint,-1, (void *)&dms.preScaler,(void *)rngPRESCALER,      "ADC clock prescaler"},
  {"?SNUM", CMDint,-1, (void *)&dms.sampleNum,(void *)rngSAMPLENUM,      "ADC number of sample in average"},
  {"?SLEN", CMDint,-1, (void *)&dms.sampLen,(void *)rngSAMPLELEN,        "ADC sample length in clocks"},
// Electrometer command
  {"RPOSELEC",CMDfloat,  0, (void *)&rb.POSelec,NULL,                    "Report positive electrometer surrent in pA"},
  {"SPOSOFF",CMDfloat,   1, (void *)&dms.ElectPosOffset,(void *)rngELEC, "Set electrometer positive channel offset voltage"},
  {"GPOSOFF",CMDfloat,   0, (void *)&dms.ElectPosOffset,NULL,            "Returns electrometer positive channel offset voltage"},
  {"SPOSZERO",CMDfloat,  1, (void *)&dms.ElectPosZero,(void *)rngELEC,   "Set electrometer positive channel zero voltage"},
  {"GPOSZERO",CMDfloat,  0, (void *)&dms.ElectPosZero,NULL,              "Returns electrometer positive channel zero voltage"},

  {"RNEGELEC",CMDfloat,  0, (void *)&rb.NEGelec,NULL,                    "Report negative electrometer surrent in pA"},
  {"SNEGOFF",CMDfloat,   1, (void *)&dms.ElectNegOffset,(void *)rngELEC, "Set electrometer negative channel offset voltage"},
  {"GNEGOFF",CMDfloat,   0, (void *)&dms.ElectNegOffset,NULL,            "Returns electrometer negative channel offset voltage"},
  {"SNEGZERO",CMDfloat,  1, (void *)&dms.ElectNegZero,(void *)rngELEC,   "Set electrometer negative channel zero voltage"},
  {"GNEGZERO",CMDfloat,  0, (void *)&dms.ElectNegZero,NULL,              "Returns electrometer negative channel zero voltage"},

  {"ELTMTRZERO",CMDfunction,0, (void *)ZeroElectrometer,NULL,             "Zero the electrometer channels"},
// DMS Limits
  {"SMAXDRV",CMDfloat, 1, (void *)&dms.MaxDrive,NULL,          "Set the maximum drive level allowed"},
  {"GMAXDRV",CMDfloat, 0, (void *)&dms.MaxDrive,NULL,          "Returns the maximum drive level"},
  // Need to add power and current limit
// DMS DC bias commands
  {"SCV",    CMDfunction,1, (void *)setCV,NULL,                  "Set the CV voltage"},
  {"GCV",    CMDfloat,   0, (void *)&dms.CV,NULL,                "Returns the CV voltage"},
  {"GCVV",   CMDfloat,   0, (void *)&rb.CV,NULL,                 "Returns the CV actual voltage"},
  {"SBIAS",  CMDfunction,1, (void *)SetBias,NULL,                "Set the Bias voltage"},
  {"GBIAS",  CMDfloat,   0, (void *)&dms.Bias,NULL,              "Returns the Bias voltage"},
  {"GBIASV", CMDfloat,   0, (void *)&rb.BIAS,NULL,               "Returns the Bias actual voltage"},
// DMS scanning commands
  {"SCVSTRT",  CMDfloat, 1, (void *)&dms.CVstart,(void *)rngCV,  "set the CV scan start voltage"},
  {"GCVSTRT",  CMDfloat, 0, (void *)&dms.CVstart,NULL,           "returns the CV scan start voltage"},
  {"SCVEND",   CMDfloat, 1, (void *)&dms.CVend,(void *)rngCV,    "set the CV scan end voltage"},
  {"GCVEND",   CMDfloat, 0, (void *)&dms.CVend,NULL,             "returns the CV scan end voltage"},
  {"SVRFSTRT", CMDfloat, 1, (void *)&dms.VRFstart,(void *)rngVrf,"set the Vrf scan start voltage"},
  {"GVRFSTRT", CMDfloat, 0, (void *)&dms.VRFstart,NULL,          "returns the Vrf scan start voltage"},
  {"SVRFEND",  CMDfloat, 1, (void *)&dms.VRFend,(void *)rngVrf,  "set the Vrf scan end voltage"},
  {"GVRFEND",  CMDfloat, 0, (void *)&dms.VRFend,NULL,            "returns the Vrf scan end voltage"},
  {"SVRFSTEPS",  CMDint, 1, (void *)&dms.VRFsteps,(void *)rngVrfSTEPS,"set the Vrf steps in a scan"},
  {"GVRFSTEPS",  CMDint, 0, (void *)&dms.VRFsteps,NULL,          "returns the Vrf steps in a scan"},
  {"SNUMAVG",  CMDint, 1, (void *)&dms.Averages,(void *)rngAverages,"set the number of CV scan averages"},
  {"GNUMAVG",  CMDint, 0, (void *)&dms.Averages,NULL,          "returns the number of CV scan averages"},

  {"SSTEPDUR",CMDint,1, (void *)&dms.CVstepDuration,(void *)rngDUR,"set the scan step duration in mS"},
  {"GSTEPDUR", CMDint,   0, (void *)&dms.CVstepDuration,NULL,    "returns the scan step duration in mS"},
  {"SNUMSTP",  CMDint,  1, (void *)&dms.CVsteps,(void *)rngSTEPS,"set the scan number of steps"},
  {"GNUMSTP",  CMDint,   0, (void *)&dms.CVsteps,NULL,           "returns the scan number of steps"},
  {"SCNSTRT",  CMDfunction, 0, (void *)StartScan,NULL,           "Start the scan, use internal timing"},
  {"SCNSTP",   CMDfunction, 0, (void *)StopScan,NULL,            "Stop a scan that is in process"},
  {"SCANSTAT", CMDfunction, 0, (void *)ScanStat,NULL,            "Returns the scan status"},
// These commands need to be changed to external trigger to start a scan. External trigger for stepping
// does not make sense
//  {"SEXTSTP",  CMDbool, 1, (void *)&dms.EnableExtStep,NULL,      "Set to TRUE to enable external pin rising edge to advance step"},
//  {"GEXTSTP",  CMDbool, 0, (void *)&dms.EnableExtStep,NULL,      "Returns enable external pin status"},
//  {"SSTPPIN",  CMDint, 1, (void *)&dms.ExtAdvInput,NULL,         "Set pin number to use for external step input"},
//  {"GSTPPIN",  CMDint, 0, (void *)&dms.ExtAdvInput,NULL,         "Returns pin number used for external step input"},
// Scan data read commands
  {"RSCNPOS", CMDfunction, 3, (void *)reportPosElec,NULL,        "Report positive electrometer channel data. scan num,scan point,numvalues"},
  {"RSCNNEG", CMDfunction, 3, (void *)reportNegElec,NULL,        "Report negative electrometer channel data. scan num,scan point,numvalues"},
  {"RNUMSCANS", CMDfunction, 0, (void *)numSchScans,NULL,        "Returns the number of recorded scans"},
  {"RSCNPTS",   CMDfunction, 1, (void *)scanPoints,NULL,         "Returns the number of recorded points in selected scan"},
// Scan file IO command
  {"SSCAN",   CMDfunction, 1, (void *)saveScan,NULL,             "Save scan to file system"},
  {"RSCAN",   CMDfunction, 1, (void *)readScan,NULL,             "Read scan from the file system"},
  {"DIR",   CMDfunction, -1, (void *)listFiles,NULL,             "List files"},
  {"DEL",   CMDfunction, 1, (void *)deleteFile,NULL,             "Delete file"},
  {"MORE",   CMDfunction, 1, (void *)moreFile,NULL,              "List file"},
// Schedule a scan and auto save
  {"SCHEDULE",CMDfunction, 3, (void *)schedule,NULL,             "Schedule collection of scan series, base file name, interval, number"},
  {"STOP",CMDfunction, 0, (void *)stop,NULL,                     "Stop schedule collection of scan series"},
  {"ISSCH", CMDbool, 0, (void *)&acquireEnabled,NULL,            "Returns TRUE is scans are scheduled"},
  {"GSCNNUM", CMDint, 0, (void *)&collectedScan,NULL,            "Returns the number of collected scans"},
// DMS calibration commands
  {"CALDCBREF",CMDfunction, 0, (void *)CalibrateREF,NULL,        "Calibrate the reference voltage"},
  {"CALDCB1",  CMDfunction, 0, (void *)CalibrateDCB1,NULL,       "Calibrate DCB1 voltage"},
  {"CALDCB2",  CMDfunction, 0, (void *)CalibrateDCB2,NULL,       "Calibrate DCB2 voltage"},
  {"CALVRF",   CMDfunction, 0, (void *)CalibrateVrf,NULL,        "Calibrate the Vrf"},
  {"CALDRV2VRF",CMDfunction,0, (void *)CalibrateVrf2Drive,NULL,  "Generate the Vrf lookup table"},
  {"SGAIN",     CMDfloat,   1, (void *)&dms.loopGain,NULL,       "Set the Vrf loop gain"},
  {"GGAIN",     CMDfloat,   0, (void *)&dms.loopGain,NULL,       "Return the Vrf loop gain"},
  // Debug commands
  {"TCC", CMDfunction,2,(void *)setTCC,NULL,                     "Set TCC counter PWM, channel, count"},
  {"TC1", CMDfunction,2,(void *)setTC1,NULL,                     "Set TC2 frequency, duty cycle"},
  {"ADCS",CMDfunction,0,(void *)readADCs,NULL,                   "Read all ADC channel counts"},
  {"RENV",CMDfunction,0,(void *)printBME280,NULL,                "Read enviornment data"},
  {"GTEMP",CMDfunction,0,(void *)getTemp,NULL,                   "Return temoerature in C"},
  {"GPRESS",CMDfunction,0,(void *)getPress,NULL,                 "Return pressure in hPa"},
  {"GHUM",CMDfunction,0,(void *)getHum,NULL,                     "Return humidity in %"},
  {"GALT",CMDfunction,0,(void *)getAlt,NULL,                     "Return altitude in m"},
  {"SALT",CMDfunction,1,(void *)setAltitude,NULL,                "Set altitude in m"},  
  // End of table marker
  {NULL}
};
static CommandList dbsList = {dbsCmds, NULL};

// ThreadController that will control all threads
ThreadController control = ThreadController();
//Threads
Thread SystemThread = Thread();

commandProcessor cp;
debug dbg(&cp);


// This function adjusts the drive level to achieve the desired Vrf level.
// This operation is done by reading the Vrf level and adjusting the drive
// based on the error between current value and desired value, a control loop.
// This function is not fast.
void SetVrf(float Vrf)
{
   float error;

   if(!dms.Enable) return;   // Exit if system is not enabled
   dms.Vrf = Vrf;
   for(int i=0;i<100;i++)
   {
      // Read the current Vrf level
      rb.Vrf = -1;
      for(int j=0;j<10;j++) 
      {
        float fval = AnalogIn(readadc, &dms.RFLEVELmon,4);
        rb.Vrf = Filter(rb.Vrf, fval);
      }
      // Calculate error and adjust the drive level
      error = Vrf - rb.Vrf;
      if(fabs(error) <= 1) return;
      if(fabs(error) <= Vrf * 0.001) return;
      dms.Drive += error * dms.loopGain/100;
      if(dms.Drive > dms.MaxDrive) dms.Drive = dms.MaxDrive;
      if(dms.Drive < 0) dms.Drive = 0;  
      DACupdate(dms.Drive, &dms.DRIVEctrl);
      delay(5);
   }
}
// This function adjusts the drive level to achieve the desired Vrf level.
// This function used the drive level lookup table. This function is very fast 
// and depends on the lookup being accurate. The table has 21 entries, every
// MaxDrive/20 points in drive level. If the Max Drive level is changed then
// The table needs to be regenerated.
void SetVrfTable(float Vrf)
{
  int   i;
  float fStep = dms.MaxDrive / 20;

  if(!dms.Enable) return;   // Exit if system is not enabled
  for(i=0;i<21;i++)
  {
    if(Vrf < dms.LUVrf[i])
    {
      if(i==0) return;
      dms.Drive = i * fStep - ((dms.LUVrf[i] - Vrf) / (dms.LUVrf[i] - dms.LUVrf[i-1])) * fStep;
      DACupdate(dms.Drive, &dms.DRIVEctrl);
      dms.Vrf = Vrf;
      return;
    }
  }
}

/**
 * @brief Sets a new time (hour, minute, second) on an existing DateTime object.
 *
 * This function preserves the current date components of the DateTime object.
 * It works by using the existing getters to fetch the date and the constructor
 * to create a new DateTime instance with the combined old date and new time,
 * assigning the new object back to the reference.
 *
 * @param dt The DateTime object to modify (passed by reference).
 * @param hour The new hour (0-23).
 * @param minute The new minute (0-59).
 * @param second The new second (0-59).
 */
void setDateTimeTime(DateTime &dt, uint8_t hour, uint8_t minute, uint8_t second) {
    // 1. Fetch current date components (year, month, day)
    uint16_t currentYear = dt.year();
    uint8_t currentMonth = dt.month();
    uint8_t currentDay = dt.day();

    // 2. Create a new DateTime object with the old date and new time
    // 3. Assign the new object back to the reference
    dt = DateTime(currentYear, currentMonth, currentDay, hour, minute, second);
}

/**
 * @brief Sets a new date (year, month, day) on an existing DateTime object.
 *
 * This function preserves the current time components of the DateTime object.
 * It works by using the existing getters to fetch the time and the constructor
 * to create a new DateTime instance with the combined new date and old time,
 * assigning the new object back to the reference.
 *
 * @param dt The DateTime object to modify (passed by reference).
 * @param year The new year (e.g., 2024).
 * @param month The new month (1-12).
 * @param day The new day (1-31).
 */
void setDateTimeDate(DateTime &dt, uint16_t year, uint8_t month, uint8_t day) {
    // 1. Fetch current time components (hour, minute, second)
    uint8_t currentHour = dt.hour();
    uint8_t currentMinute = dt.minute();
    uint8_t currentSecond = dt.second();

    // 2. Create a new DateTime object with the new date and old time
    // 3. Assign the new object back to the reference
    dt = DateTime(year, month, day, currentHour, currentMinute, currentSecond);
}

// This function configures general clock 3 to use the external crystal osc
void setupCrystalClock(void)
{
#define GENERIC_CLOCK_GENERATOR_XOSC32K     (3u)
#define GENERIC_CLOCK_GENERATOR_48M		      (1u)
#define GENERIC_CLOCK_GENERATOR_48M_SYNC	  GCLK_SYNCBUSY_GENCTRL1
#define GENERIC_CLOCK_GENERATOR_100M	      (2u)
#define GENERIC_CLOCK_GENERATOR_100M_SYNC	  GCLK_SYNCBUSY_GENCTRL2
#define GENERIC_CLOCK_GENERATOR_12M         (4u)
#define GENERIC_CLOCK_GENERATOR_12M_SYNC    GCLK_SYNCBUSY_GENCTRL4

  /* ----------------------------------------------------------------------------------------------
   * 1) Enable XOSC32K clock (External on-board 32.768Hz oscillator)
   */
  OSC32KCTRL->XOSC32K.reg = OSC32KCTRL_XOSC32K_ENABLE | OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_CGM_XT | OSC32KCTRL_XOSC32K_XTALEN;
  
  while( (OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY) == 0 ){
    /* Wait for oscillator to be ready */
  }
  /* ----------------------------------------------------------------------------------------------
   * 2) Put XOSC32K as source of Generic Clock Generator 3
   */
  GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_XOSC32K].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_XOSC32K) | //generic clock gen 3
    GCLK_GENCTRL_GENEN;

  while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL3 ){
    /* Wait for synchronization */
  }

}

// Hook registered with the debug command set (dbg.registerDebugFunction);
// intentionally empty, kept as a place to drop temporary debug code.
void Debug(void)
{
}

// Applies calibration (DACchan.m/b) to convert an engineering value to raw
// PWM counts and writes it to the corresponding 12-bit PWM "DAC" channel.
void DACupdate(float value, DACchan *chan)
{
  int cnts = Value2Counts(value, chan);
  pwmWrite(chan->Chan, cnts);
}

// Hardware read function passed to AnalogIn()/Devices.cpp. The processor's
// ADC is configured for 12-bit resolution (see analogReadResolution(12) in
// setup()); left-shifting by 4 rescales that to the 16-bit count space used
// by the stored ADCchan/DACchan calibration parameters throughout this file.
int readadc(int8_t chan)
{
   return analogRead(chan) << 4;
}

void VRFcontrolLoop(void)
{
  // If Enable is false exit
  if(!dms.Enable) return;
  // If Mode is false exit
  if(!dms.Mode) return;
  // Update the VRF readback, this gets rid of delay and the loop is faster
  float Vrf = 0;
  for(int j=0;j<10;j++) Vrf += AnalogIn(readadc, &dms.RFLEVELmon);
  Vrf /= 10;
  // Calculate error between actual and setpoint
  float error = dms.Vrf - Vrf;
  if(fabs(error) < 2.0) return;
  dms.Drive += error * dms.loopGain/100;
  if(dms.Drive > dms.MaxDrive) dms.Drive = dms.MaxDrive;
  if(dms.Drive < 0) dms.Drive = 0;  
}

// Process loop, update all DAC outputs and read all ADC readbacks
// This function is called by the thread controller and updates every 
// 100mS
void Update()
{
  float  fval;
  static bool enabled=true;

  if(rb.Vbat < 2.0) statusLED(RED);
  else statusLED(GREEN);
  // Output all the DAC PWM channels
  if(!dms.Enable)
  {
    DACupdate(0, &dms.DRIVEctrl);
    pinMode(PULSE,OUTPUT);
    digitalWrite(PULSE,LOW);
    enabled = false;
  }
  else
  {
    // Test limits
    if(power > dms.MaxPower) dms.Enable = false;
    if(current > dms.MaxCurrent) dms.Enable = false;
    if(dms.Drive > dms.MaxDrive) dms.Drive = dms.MaxDrive;
    DACupdate(dms.Drive, &dms.DRIVEctrl);
    if(!enabled) tc1configure(dms.Freq, dms.Duty);
    enabled = true;
  }
  DACupdate(dms.DCref, &dms.DCBREFctrl);
  DACupdate(dms.DCB1v, &dms.DCB1ctrl);
  DACupdate(dms.DCB2v, &dms.DCB2ctrl);
  DACupdate(dms.ElectPosOffset, &dms.POSOFFctrl);
  DACupdate(dms.ElectPosZero,   &dms.POSZEROctrl);
  DACupdate(dms.ElectNegOffset, &dms.NEGOFFctrl);
  DACupdate(dms.ElectNegZero,   &dms.NEGZEROctrl);

  setFreqDuty(dms.Freq, dms.Duty);
  // Read all the ADC readbacks and filter
  fval = AnalogIn(readadc, &dms.RFLEVELmon);
  rb.Vrf = Filter(rb.Vrf, fval);
  fval = AnalogIn(readadc, &dms.DCB1RBmon);
  rb.DCB1v = Filter(rb.DCB1v, fval);
  fval = AnalogIn(readadc, &dms.DCB2RBmon);
  rb.DCB2v = Filter(rb.DCB2v, fval);
  fval = AnalogIn(readadc, &dms.POSELECmon);
  rb.POSelec = Filter(rb.POSelec, fval);
  fval = AnalogIn(readadc, &dms.NEGELECmon);
  rb.NEGelec = Filter(rb.NEGelec, fval);
  #ifndef NOINA237
  fval = ina237.getBusVoltage_V();
  rb.Vbat = Filter(rb.Vbat, fval);
  current = ina237.getCurrent_mA() / 1000.0;
  fval = current * ina237.getBusVoltage_V();
  power = Filter(power, fval);
  #else
  rb.Vbat = 4.2;
  current = 0;
  power = 0;
  #endif
   // Calculate CV and BIAS from DCB1 and DCB2
  rb.BIAS = (rb.DCB1v + rb.DCB2v)/2;
  rb.CV = rb.DCB1v - rb.DCB2v;

  VRFcontrolLoop();
}

void setup() 
{
  strip.begin();            // Initialize the DotStar strip
  strip.show();             // Turn off all LEDs initially
  strip.setBrightness(20);  // Set the brightness level
  statusLED(RED);           // Turn on red for the setup phase

  // Turn on power FET
  pinMode(POWER,OUTPUT);
  digitalWrite(POWER,LOW);
  // Init flash drive
  flash.begin(my_flash_devices, flashDevices);
  // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("Adafruit", "External Flash", "1.0");
  // Set callback
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  // Set disk size, block size should be 512 regardless of spi flash page size
  usb_msc.setCapacity(flash.size()/512, 512);
  // MSC is ready for read/write
  usb_msc.setUnitReady(true);
  usb_msc.begin();
  // Init file system on the flash
  fs_formatted = fatfs.begin(&flash);
  // Init spi interface
  //SPI.begin();
  // Read the flash config contents and test the signature
  dms = flash_DMSdata.read();
  if(dms.Signature != SIGNATURE) dms = Rev_1_dms;
  //dms = Rev_1_dms;
  // Init test/humitity monitor, use TWI
  bool status = bme.begin(0x76);
  if (!status) 
  {
    // Here with hardware init error
  }
  // Setup power/voltage moniitor
  #ifndef NOINA237
  ina237.begin();
  ina237.setShunt(0.1, 3.2);
  #endif
  // Init the PWM outputs
  initPWM();
  tc1configure(1200000, 10);
  Serial.begin(0);
  //Serial1.begin(9600);
  cp.registerStream(&Serial);
  cp.selectStream(&Serial);
  cp.registerCommands(&dbsList);
  cp.registerCommands(dbg.debugCommands());
  dbg.registerDebugFunction(Debug);
  // Configure Threads
  SystemThread.setName((char *)"Update");
  SystemThread.onRun(Update);
  SystemThread.setInterval(100);
  // Add thread to the controller
  control.add(&SystemThread);
  analogReadResolution(12);
  // Start the real time clock and init to last built time
  rtc.begin();
  DateTime now = DateTime(F(__DATE__), F(__TIME__));
  rtc.adjust(now);
  statusLED(GREEN);   // Turn on green for the run phase
  rtc.now();
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and 
// return number of copied bytes (must be multiple of block size) 
int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return flash.readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and 
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return flash.writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_cb (void)
{
  // sync with flash
  flash.syncBlocks();
  // clear file system's cache to force refresh
  fatfs.cacheClear();
  fs_changed = true;
}

// Runs one unattended scan for the scheduled-acquisition feature (see
// schedule()/scanTime()): powers up, performs a scan using the current
// settings, saves it as "<baseFileName>.<collectedScan, %04d>", then powers
// back down. Called from loop() when scanNow is set by the RTC alarm ISR.
void scanAndSave(void)
{
  char fileName[41];

  // Turn on power and enable
  ON;
  dms.Enable = true;
  Update();
  // Set LED to blue
  statusLED(BLUE);
  // Delay to allow system to stabalize
  delay(100);
  // Perform scan
  if(performScan(false))
  {
    // Build file name
    sprintf(fileName, "%s.%04d",baseFileName,collectedScan);
    // Save data to file, set LED green is ok, red if error
    if(saveScanAscii(fileName)) statusLED(GREEN);
    else statusLED(RED);
  }
  // Disable system and turn power off
  dms.Enable = false;
  Update();
  OFF;
}

void loop() 
{
  // put your main code here, to run repeatedly:
  cp.processStreams();
  cp.processCommands();
  control.run();

  if(scanNow)
  {
    scanAndSave();
    scanNow = false;
    collectedScan++;
    if(collectedScan >= numSchScans) scanFlag = false;
  }
}

// Host command routines

// SAVE: persists the live `dms` settings into the SAMD51's emulated-EEPROM
// flash region via the FlashStorage library. Survives power cycles but is
// lost on the next firmware re-upload. For durable storage across
// re-uploads, use SAVEF instead (saveDefaults(), writes to the external SPI
// flash filesystem).
void SaveSettings(void)
{
  dms.Signature = SIGNATURE;
  flash_DMSdata.write(dms);
  cp.sendACK();
}

// RESTORE: reloads `dms` from the emulated-EEPROM region saved by SAVE.
void RestoreSettings(void)
{
  DMSdata temp;
  // Read the flash config contents into temp and test the signature
  temp = flash_DMSdata.read();
  if(temp.Signature == SIGNATURE) dms = temp;
  else
  {
    cp.sendNAK(ERR_EEPROMWRITE);
    return;
  }
  cp.sendACK();
}

// FORMAT: resets the emulated-EEPROM region to the factory-default
// (Rev_1_dms) settings. Despite the name this does not touch the external
// SPI flash filesystem — see formatFS() (command FORMATFS) for that.
void FormatFLASH(void)
{
  flash_DMSdata.write(Rev_1_dms);
  cp.sendACK();
}

// Debug commands (TCC, TC1, ADCS): bypass the calibrated DACupdate()/
// AnalogIn() paths to drive/read hardware directly in raw counts, for
// bring-up and troubleshooting.

// TCC,<channel>,<count>: writes a raw 12-bit PWM count directly to one of
// the eight PWM "DAC" output channels (pin numbers, see DMS.h).
void setTCC(void)
{
  int ch,count;

  cp.getValue(&ch);
  cp.getValue(&count);
  pwmWrite(ch,count);
}
// TC1,<freq>,<duty>: sets the FAIMS RF drive frequency/duty directly,
// bypassing dms.Freq/dms.Duty.
void setTC1(void)
{
  int freq,duty;

  cp.getValue(&freq);
  cp.getValue(&duty);
  setFreqDuty(freq,duty);
}
// ADCS: prints raw (uncalibrated) analogRead() counts for all five analog
// input pins.
void readADCs(void)
{
  cp.print("A0 = "); cp.println(analogRead(A0));
  cp.print("A1 = "); cp.println(analogRead(A1));
  cp.print("A2 = "); cp.println(analogRead(A2));
  cp.print("A3 = "); cp.println(analogRead(A3));
  cp.print("A4 = "); cp.println(analogRead(A4));
}

// BME280 routines
void printBME280(void) 
{
  cp.sendACK(false);

  cp.print("Temperature = ");
  cp.print(bme.readTemperature());
  cp.println(" *C");

  cp.print("Pressure = ");
  cp.print(bme.readPressure() / 100.0F);
  cp.println(" hPa");

  cp.print("Approx. Altitude = ");
  cp.print(bme.readAltitude(dms.SeaLevelPress));
  cp.println(" m");

  cp.print("Humidity = ");
  cp.print(bme.readHumidity());
  cp.println(" %");

  cp.println("");
}

void getTemp(void)
{
  cp.sendACK(false);
  cp.println(bme.readTemperature());
}

void getPress(void)
{
  cp.sendACK(false);
  cp.println(bme.readPressure() / 100.0F);
}

void getHum(void)
{
  cp.sendACK(false);
  cp.println(bme.readHumidity());
}

void getAlt(void)
{
  cp.sendACK(false);
  cp.println(bme.readAltitude(dms.SeaLevelPress));
}

void setAltitude(void)
{
  float alt;

  if(!cp.getValue(&alt)) return(cp.sendNAK());
  float press = bme.readPressure() / 100.0F;
  dms.SeaLevelPress = bme.seaLevelForAltitude(alt,press);
  cp.sendACK();
}

void SetVrfCmd(void)
{
  if(!cp.getValue(&dms.Vrf,0,2000)) {cp.sendNAK(); return;}
  SetVrf(dms.Vrf);
  cp.sendACK();
}
void SetVrfTableCmd(void)
{
  if(!cp.getValue(&dms.Vrf,0,2000)) {cp.sendNAK(); return;}
  SetVrfTable(dms.Vrf);
  cp.sendACK();
}

void setCV(void)
{
  if(!cp.getValue(&dms.CV,-24,24)) {cp.sendNAK(); return;}
  dms.DCB1v =  dms.CV/2 + dms.Bias;
  dms.DCB2v = -dms.CV/2 + dms.Bias;
  cp.sendACK();
}
void SetBias(void)
{
  if(!cp.getValue(&dms.Bias,-24,24)) {cp.sendNAK(); return;}
  dms.DCB1v =  dms.CV/2 + dms.Bias;
  dms.DCB2v = -dms.CV/2 + dms.Bias;
  cp.sendACK();
}

void freeScanBuffer(void)
{
    if (scanBuffer == NULL) return; // Nothing to free

    // 1. Free each CVscan object
    for (int i = 0; i < scanBuffer->Points; ++i) 
    {
        if (scanBuffer->cvscan[i] != NULL) 
        {
          // Free the data buffers
          if(scanBuffer->cvscan[i]->dataPos != NULL)
          {
            delete[] scanBuffer->cvscan[i]->dataPos;
            scanBuffer->cvscan[i]->dataPos = NULL;
          }
          if(scanBuffer->cvscan[i]->dataNeg != NULL)
          {
            delete[] scanBuffer->cvscan[i]->dataNeg;
            scanBuffer->cvscan[i]->dataNeg = NULL;
          }
          // Each entry was allocated with scalar `new CVscan` (see
          // allocateScanBuffer), so it must be freed with scalar delete,
          // not delete[].
          delete scanBuffer->cvscan[i];
          scanBuffer->cvscan[i] = NULL; // Good practice to nullify dangling pointers
        }
    }
    // 2. Free the Scan structure itself. cvscan is a `new CVscan*[n]` array
    // of pointers, so it needs delete[]; scanBuffer itself was a scalar
    // `new Scan`, so it needs plain delete.
    delete[] scanBuffer->cvscan;
    delete scanBuffer;
    scanBuffer = NULL; // Reset global pointer
}

bool allocateScanBuffer(void)
{
  // Deallocate existing buffer if it exists to prevent memory leaks
  if (scanBuffer != NULL) freeScanBuffer(); // Call the deallocation function
  // Allocate the base scan struct
  scanBuffer = new(std::nothrow) Scan;
  if(scanBuffer == NULL) return false;
  // Init the Scan struct
  scanBuffer->VRFstart = dms.VRFstart;
  scanBuffer->VRFend   = dms.VRFend;
  scanBuffer->Points   = dms.VRFsteps;
  scanBuffer->Acquired = 0;
  scanBuffer->Averages = dms.Averages;

  //size_t CVscanSize = sizeof(CVscan *);
  scanBuffer->cvscan = new(std::nothrow) CVscan *[dms.VRFsteps];

  // Allocate the array of CVscan structs
  for(int i=0;i<dms.VRFsteps;i++)
  {
    scanBuffer->cvscan[i] = new(std::nothrow) CVscan;
    if(scanBuffer->cvscan[i] == NULL) return false;
    // Init the CVscan struct
    scanBuffer->cvscan[i]->CVstart  = dms.CVstart;
    scanBuffer->cvscan[i]->CVend    = dms.CVend;
    scanBuffer->cvscan[i]->Points   = dms.CVsteps;
    scanBuffer->cvscan[i]->Acquired = 0;
  }
  // Allocate the data buffers in each CVscan
  for(int i=0;i<dms.VRFsteps;i++)
  {
    scanBuffer->cvscan[i]->dataPos = new(std::nothrow) uint16_t [scanBuffer->cvscan[i]->Points];
    if(scanBuffer->cvscan[i]->dataPos == NULL) return false;
    scanBuffer->cvscan[i]->dataNeg = new(std::nothrow) uint16_t [scanBuffer->cvscan[i]->Points];
    if(scanBuffer->cvscan[i]->dataNeg == NULL) return false;
  }
  return true;
}

// TC3 timer callback (see tc3Configure in performCVscan), fires once per CV
// step. Records the ADC values latched by the ADC0/1 ISRs (LastADCval[])
// for the current point, then steps the CV/Bias DAC outputs to the next
// point's voltage. Flags scanStatus = SCAN_CVcomplete once all points for
// the current Vrf step have been acquired.
void CVscanISR(void)
{
  // Exit if all points are acquired
  if(scanBuffer->cvscan[scanBuffer->Acquired]->Acquired >= scanBuffer->cvscan[scanBuffer->Acquired]->Points) 
  {
    scanStatus = SCAN_CVcomplete;
    return;
  }
  // Save the voltages in the data scanBuffer
  scanBuffer->cvscan[scanBuffer->Acquired]->dataPos[scanBuffer->cvscan[scanBuffer->Acquired]->Acquired] = LastADCval[0];
  scanBuffer->cvscan[scanBuffer->Acquired]->dataNeg[scanBuffer->cvscan[scanBuffer->Acquired]->Acquired++] = LastADCval[1];
  // Advance the CV voltage to the next point
  float cv = scanBuffer->cvscan[scanBuffer->Acquired]->CVstart;
  cv += CVstep * scanBuffer->cvscan[scanBuffer->Acquired]->Acquired;
  dms.DCB1v =  cv/2 + dms.Bias;
  dms.DCB2v = -cv/2 + dms.Bias;
  DACupdate(dms.DCB1v, &dms.DCB1ctrl);
  DACupdate(dms.DCB2v, &dms.DCB2ctrl); 
}

// Runs one CV sweep (one column of the 2-D scan) at the current Vrf level:
// sets the CV start voltage, starts the TC3 step timer with CVscanISR as
// the callback, then blocks (servicing the command processor so SCNSTP can
// abort) until the sweep completes or is aborted. Returns false if the scan
// buffer isn't ready, the current Vrf step is already fully acquired, or the
// sweep was aborted.
bool performCVscan(void)
{
  // Make sure we have allocated the buffers and we have and empty
  // scan to accept the data
  if(scanBuffer == NULL) return false;
  if(scanBuffer->Acquired >= scanBuffer->Points) return false;
  if(scanBuffer->cvscan[scanBuffer->Acquired] == NULL) return false;
  if(scanBuffer->cvscan[scanBuffer->Acquired]->Acquired >= scanBuffer->cvscan[scanBuffer->Acquired]->Points) return false;
  // Turn on the ADC channels
  preScaler = dms.preScaler;
  sampleNum = dms.sampleNum;
  sampLen = dms.sampLen;
  ADCchangeDet(ADC0);
  ADCchangeDet(ADC1);  
  // Calculate the CV delta voltage between scan points
  if(scanBuffer->cvscan[scanBuffer->Acquired]->Points == 1) CVstep = 0;
  else CVstep = (scanBuffer->cvscan[scanBuffer->Acquired]->CVend - scanBuffer->cvscan[scanBuffer->Acquired]->CVstart)/(scanBuffer->cvscan[scanBuffer->Acquired]->Points - 1);
  // Set the CV start voltage
  dms.DCB1v =  scanBuffer->cvscan[scanBuffer->Acquired]->CVstart/2 + dms.Bias;
  dms.DCB2v = -scanBuffer->cvscan[scanBuffer->Acquired]->CVstart/2 + dms.Bias;
  DACupdate(dms.DCB1v, &dms.DCB1ctrl);
  DACupdate(dms.DCB2v, &dms.DCB2ctrl); 
  // Turn on the timer and attach the ISR
  tc3Configure(dms.CVstepDuration,CVscanISR);
  // Wait for data acquire to complete or an abort signal
  scanStatus = SCAN_CVactive;
  while(true)
  {
    if(scanStatus == SCAN_CVcomplete) break;
    if(scanStatus == SCAN_ABORT) break;
    cp.processStreams();
    cp.processCommands();
  }
  // Stop the timer
  tcReset(TC3);
  // turn off the ADC channels
  ADCreset(ADC0);
  ADCreset(ADC1);
  if(scanStatus == SCAN_ABORT) return false;
  return true;
}

// Called from command processor or scan scheduling function. Performs a scan using current
// system settings. This function will block the update loop during scanning, the command
// processor is call to allow aborting a scan.
// The 
//
// Set the ackFlag true to enable this function to all the host in the case where the function
// is called from the command processor.
bool performScan(bool ackFlag)
{
  uint32_t *dataPos = NULL;
  uint32_t *dataNeg = NULL;

  if(scanStatus == SCAN_CVactive) return false;
  // If scan buffer is not null then free existing buffer
  freeScanBuffer();
  // Allocation scan buffer and initialize
  if(allocateScanBuffer())
  {
    if(ackFlag) cp.sendACK();
    if(scanBuffer->Averages>1)
    {
      // Build temp buffers to hold the sum of the CV scans for the requested averages
      dataPos = new(std::nothrow) uint32_t [scanBuffer->cvscan[0]->Points];
      dataNeg = new(std::nothrow) uint32_t [scanBuffer->cvscan[0]->Points];
    }
    // Calculate Vrf step size
    if(scanBuffer->Points == 1) VRFstep = 0;
    else VRFstep = (scanBuffer->VRFend - scanBuffer->VRFstart)/(scanBuffer->Points - 1);
    // Loop through all Vrf steps
    for(scanBuffer->Acquired = 0;scanBuffer->Acquired < scanBuffer->Points;scanBuffer->Acquired++)
    {
      // Set Vrf
      SetVrf(scanBuffer->VRFstart + VRFstep * scanBuffer->Acquired);
      scanBuffer->cvscan[scanBuffer->Acquired]->Vrf = scanBuffer->VRFstart + VRFstep * scanBuffer->Acquired;
      // Perform CV scan
      if(!performCVscan()) break;
      // If averages is greater then 1 then coadd the remainiing CV scans
      if((scanBuffer->Averages>1) && (dataPos != NULL) && (dataNeg != NULL))
      {
        for(int k=0;k<scanBuffer->cvscan[scanBuffer->Acquired]->Points;k++)
        {
          dataPos[k] = scanBuffer->cvscan[scanBuffer->Acquired]->dataPos[k];
          dataNeg[k] = scanBuffer->cvscan[scanBuffer->Acquired]->dataNeg[k];
        }
        for(int i=2;i<=scanBuffer->Averages;i++)
        {
          scanBuffer->cvscan[scanBuffer->Acquired]->Acquired = 0;  // Clear the pointer
          // Same abort/failure check as the first sweep above: if this repeat
          // is aborted or fails partway through, stop averaging instead of
          // coadding partial/stale data into the running sum.
          if(!performCVscan()) break;
          for(int k=0;k<scanBuffer->cvscan[scanBuffer->Acquired]->Points;k++)
          {
            dataPos[k] += scanBuffer->cvscan[scanBuffer->Acquired]->dataPos[k];
            dataNeg[k] += scanBuffer->cvscan[scanBuffer->Acquired]->dataNeg[k];
          }
        }
        for(int k=0;k<scanBuffer->cvscan[scanBuffer->Acquired]->Points;k++)
        {
          scanBuffer->cvscan[scanBuffer->Acquired]->dataPos[k] = dataPos[k]/scanBuffer->Averages;
          scanBuffer->cvscan[scanBuffer->Acquired]->dataNeg[k] = dataNeg[k]/scanBuffer->Averages;
        }
      }
    }
    if(dataPos != NULL) delete[] dataPos;
    if(dataNeg != NULL) delete[] dataNeg;
    scanStatus = SCAN_FINISHED;
    SetVrf(dms.Vrf = 500);
    return true;
  }
  // Here if allocation failed
  scanStatus = SCAN_ALLOCATIONfailed;
  return false;
}

void StartScan(void)
{
  if(performScan(true)) return;
  else cp.sendNAK();
}

void StopScan(void)
{
  scanStatus = SCAN_ABORT;
  cp.sendACK();
}

void ScanStat(void)
{
  cp.sendACK(false);
  switch (scanStatus)
  {
    case SCAN_IDLE:
      cp.println("Idle");
      break;
    case SCAN_FINISHED:
      cp.println("Finished");
      break;
    case SCAN_CVactive:
      cp.println("Active");
      break;
    case SCAN_CVcomplete:
      cp.println("Complete");
      break;
    case SCAN_ABORT:
      cp.println("Abort");
      break;
    case SCAN_ALLOCATIONfailed:
      cp.println("Allocation failed");
      break;
    default:
      cp.println("Invalid state!");
      break;
  }
}

// Writes the current scanBuffer out to a plain-text, line-oriented file on
// the flash filesystem (see the matching reader, readScan(), for the exact
// format). Electrometer data is written in engineering units (pA) via
// Counts2Value(); readScan() converts back to raw counts on load.
bool saveScanAscii(char *fileName)
{
  if(!FSsetup()) return(false);
  if((file = fatfs.open(fileName,O_WRITE | O_CREAT))!=0)
  {
    file.println("Scan file generated by DMS, do not change anyting after this line.");
    // Here with open file, write the data
    file.print("VRFstart: "); file.println(scanBuffer->VRFstart);
    file.print("VRFend: "); file.println(scanBuffer->VRFend);
    file.print("Acquired: "); file.println(scanBuffer->Acquired);
    file.print("Points: "); file.println(scanBuffer->Points);
    for(int i=0;i<scanBuffer->Acquired;i++)
    {
      file.print("Scan: "); file.println(i+1);
      file.print("Vrf: "); file.println(scanBuffer->cvscan[i]->Vrf);
      file.print("CVstart: "); file.println(scanBuffer->cvscan[i]->CVstart);
      file.print("CVend: "); file.println(scanBuffer->cvscan[i]->CVend);
      file.print("Acquired: "); file.println(scanBuffer->cvscan[i]->Acquired);
      file.print("Points: "); file.println(scanBuffer->cvscan[i]->Points);
      file.println("Pos electrometer data");
      for(int j=0;j<scanBuffer->cvscan[i]->Acquired;j++)
      {
        file.print(Counts2Value(scanBuffer->cvscan[i]->dataPos[j],&dms.POSELECmon));
        if(j < scanBuffer->cvscan[i]->Acquired-1) file.print(",");
      }
      file.println("");
      file.println("Neg electrometer data");
      for(int j=0;j<scanBuffer->cvscan[i]->Acquired;j++)
      {
        file.print(Counts2Value(scanBuffer->cvscan[i]->dataNeg[j],&dms.NEGELECmon));
        if(j < scanBuffer->cvscan[i]->Acquired-1) file.print(",");
      }
      file.println("");
    }
    file.close();
    return(true);
  }
  return(false);
}

// Save current scan to aa ASCII file.
void saveScan(void)
{
  char *fileName;

  if(scanBuffer == NULL) return(cp.sendNAK());
  cp.getValue(&fileName);
  if(saveScanAscii(fileName)) cp.sendACK();
  else cp.sendNAK();
}

// Reads and returns the next ':'/','/'\n'-delimited token from an open scan
// file (the delimiter itself is consumed but not included). Used by
// readScan() to parse the plain-text format written by saveScanAscii().
// Returns the token length, or 0 on an empty token / closed file.
int getToken(File32 *file,char *token, int max)
{
  int i = 0,c;

  if(*file == 0) return 0;
  for(i=0;i<max-1;i++)
  {
    c = file->read();
    if(c=='\n') break;
    if(c==':') break;
    if(c==',') break;
    token[i] = c;
  }
  token[i] = 0;
  return i;
}

// getToken() + string-to-number conversion, overloaded for float/int
// destinations. Returns 0 on success, -1 if no token was available.
int getFileValue(File32 *file,float *fval)
{
  char   token[21];
  String stoken;

  if(getToken(file,token,20)>0)
  {
    stoken = token;
    *fval = stoken.toFloat();
    return 0;
  }
  return -1;
}

int getFileValue(File32 *file,int *ival)
{
  char   token[21];
  String stoken;

  if(getToken(file,token,20)>0)
  {
    stoken = token;
    *ival = stoken.toInt();
    return 0;
  }
  return -1;
}

void readScan(void)
{
  char  *fileName;
  char  buf[81];
  int   scans,points,scanNum;
  float Vrf,fval;
  
  cp.getValue(&fileName);
  if(!FSsetup()) return(cp.sendNAK());
  if((file = fatfs.open(fileName,O_READ))!=0)
  {
    while(file.available() > 0)
    {
      getToken(&file,buf,80);
      if(strncmp(buf,"VRFstart",8)==0)
      {
        // From this point formward the file format must be as expected
        // if not this function will exit with error
        if(getFileValue(&file,&dms.VRFstart)==-1) break;
        getToken(&file,buf,80); if(strncmp(buf,"VRFend",6)!=0) break;
        if(getFileValue(&file,&dms.VRFend)==-1) break;
        getToken(&file,buf,80); if(strncmp(buf,"Acquired",8)!=0) break;
        if(getFileValue(&file,&scans)==-1) break;
        getToken(&file,buf,80); if(strncmp(buf,"Points",6)!=0) break;
        if(getFileValue(&file,&dms.VRFsteps)==-1) break;
        for(int i=0;i<scans;i++)
        {
          getToken(&file,buf,80); if(strncmp(buf,"Scan",6)!=0) break;
          if(getFileValue(&file,&scanNum)==-1) break;
          if(scanNum != i + 1) break;
          getToken(&file,buf,80); if(strncmp(buf,"Vrf",6)!=0) break;
          if(getFileValue(&file,&Vrf)==-1) break;
          getToken(&file,buf,80); if(strncmp(buf,"CVstart",7)!=0) break;
          if(getFileValue(&file,&dms.CVstart)==-1) break;
          getToken(&file,buf,80); if(strncmp(buf,"CVend",5)!=0) break;
          if(getFileValue(&file,&dms.CVend)==-1) break;
          getToken(&file,buf,80); if(strncmp(buf,"Acquired",8)!=0) break;
          if(getFileValue(&file,&points)==-1) break;
          getToken(&file,buf,80); if(strncmp(buf,"Points",6)!=0) break;
          if(getFileValue(&file,&dms.CVsteps)==-1) break;
          if(i==0) 
          {
            if(allocateScanBuffer() == false) break;
            scanBuffer->Acquired = scans;
          }
          // Write the data to the cvscan
          scanBuffer->cvscan[i]->Vrf = Vrf;
          scanBuffer->cvscan[i]->Acquired = points;
          getToken(&file,buf,80); if(strncmp(buf,"Pos electrometer data",21)!=0) break;          
          for(int j=0;j<points;j++)
          {
            if(getFileValue(&file,&fval)==-1)
            {
              file.close();
              cp.sendNAK();
              return;
            }
            // fval is the engineering-unit value written by saveScanAscii()
            // (via Counts2Value); convert it back to raw counts for storage
            // to match how the rest of the code (reportElec, saveScanAscii)
            // expects dataPos/dataNeg to be populated.
            scanBuffer->cvscan[i]->dataPos[j] = Value2Counts(fval,&dms.POSELECmon);
          }
          getToken(&file,buf,80); if(strncmp(buf,"Neg electrometer data",21)!=0) break;          
          for(int j=0;j<points;j++)
          {
            if(getFileValue(&file,&fval)==-1)
            {
              file.close();
              cp.sendNAK();
              return;
            }
            scanBuffer->cvscan[i]->dataNeg[j] = Value2Counts(fval,&dms.NEGELECmon);
          }
          if(scans == i + 1)
          {
            // All data has been read, exit with no errors
            file.close();
            cp.sendACK();
            return;
          }
        }
      }
    }
  }
  file.close();
  cp.sendNAK();
  return;
}

/**
 * Performs basic wildcard matching using '*' as a zero-or-more character match.
 * NOTE: This uses recursion, which can consume stack space on microcontrollers.
 * It is suitable for simple patterns typical in 8.3 filenames (e.g., "*.TXT").
 * @param pattern The wildcard pattern (e.g., "LOG*.TXT").
 * @param text The filename to check (e.g., "LOG001.TXT").
 * @return true if the text matches the pattern.
 */
bool wildcardMatch(const char *pattern, const char *text) {
    if (*pattern == '\0') {
        return *text == '\0'; // Match successful if both strings end
    }

    if (*pattern == '*') {
        // Skip consecutive asterisks in the pattern
        while (*pattern == '*') {
            pattern++;
        }

        // If '*' was the last character, it matches anything remaining in the text
        if (*pattern == '\0') {
            return true;
        }

        // Try to match the rest of the pattern starting from the current text position
        // If the rest of the pattern matches the rest of the text from any point on, it's a match
        while (*text != '\0') {
            if (wildcardMatch(pattern, text)) {
                return true;
            }
            text++;
        }
        return false; // Couldn't find a match for the character following '*'
    }
    
    // Match regular characters
    if (*pattern == *text) {
        return wildcardMatch(pattern + 1, text + 1);
    }

    return false; // Characters don't match
}

// Returns true is fileSpec finds a file match
bool isFile(char *fileSpec)
{
  char   buffer[41];
  File32 file, entry;

  if(!FSsetup()) return(false);
  file = fatfs.open("/", FILE_READ);
  file.rewindDirectory();
  while (true)
  {
    entry = file.openNextFile();
    if (!entry) break;
    entry.getName(buffer,40);
    if(wildcardMatch(fileSpec,buffer))
    {
      // If here we found a file name that matched fileSpec so truen true
      file.close();
      return(true);
    }
  }
  file.close();  
  return(false);
}

// This function deletes a file or a set of files, supports wild
// card characters.
void deleteFile(void)
{
  char   *pattern,buffer[41];
  File32 file, entry;

  if(!FSsetup()) return(cp.sendNAK());
  if(cp.getNumArgs() == 0) return(cp.sendNAK());
  else if(cp.getNumArgs() == 1)
  {
    if(cp.getValue(&pattern) == false) return(cp.sendNAK());
  }
  else return(cp.sendNAK());
  file = fatfs.open("/", FILE_READ);
  file.rewindDirectory();
  cp.sendACK(false);
  while (true)
  {
    entry = file.openNextFile();
    if (!entry) break;
    entry.getName(buffer,40);
    if(wildcardMatch(pattern,buffer))
    {
      if(fatfs.remove(buffer) == false) 
      {
        cp.print("Can't delete file, ");
        cp.println(buffer);
      }
    }
  }
  file.close();  
}

// This function lists file found on the flash drive that match the 
// search pattern string passed. If the pattern is not provided all
// files found are listed.
void listFiles(void)
{
  char   *pattern,buffer[41];
  File32 file, entry;

  if(!FSsetup()) return(cp.sendNAK());
  if(cp.getNumArgs() == 0) pattern = (char *)"*.*";
  else if(cp.getNumArgs() == 1)
  {
    if(cp.getValue(&pattern) == false) return(cp.sendNAK());
  }
  else return(cp.sendNAK());
  file = fatfs.open("/", FILE_READ);
  file.rewindDirectory();
  cp.sendACK(false);
  while (true)
  {
    entry = file.openNextFile();
    if (!entry) break;
    entry.getName(buffer,40);
    if(wildcardMatch(pattern,buffer))
    {
      cp.print(buffer);
      cp.print(", ");
      cp.println(entry.size());
    }
  }
  file.close();  
}

void moreFile(void)
{
  char   *fileName,buf[2];
  File32 file;

  if(!FSsetup()) return(cp.sendNAK());
  if(cp.getValue(&fileName) == false) return(cp.sendNAK());
  if(!fatfs.exists(fileName)) return(cp.sendNAK(ERR_CANTOPENFILE));
  if(!(file = fatfs.open(fileName, FILE_READ))) return(cp.sendNAK(ERR_CANTOPENFILE));
  cp.sendACK(false);
  while(file.available() > 0)
  {
    buf[0] = file.read();
    buf[1] = 0;
    cp.print(buf);
  }
  file.close();
  cp.sendACK();
}

// Returns the number of completed scans
void numScans(void)
{
   cp.sendACK(false);
   // If no array is allocationed return 0
   if(scanBuffer==NULL) return(cp.println(0));
   // Return the number of completed scans
   cp.println(scanBuffer->Acquired);
}
// Returns the number of completed points in the selected scan
void scanPoints(void)
{
  int scn;
  if(!cp.getValue(&scn,1,1000)) return(cp.sendNAK());
  scn--;
  cp.sendACK(false);
   // If no array is allocationed return 0
   if(scanBuffer==NULL) return(cp.println(0));
   // if selected scan is above max return 0. Valid indices are
   // 0..Points-1, so scn == Points is already out of range.
   if(scn >= scanBuffer->Points) return(cp.println(0));
   cp.println(scanBuffer->cvscan[scn]->Acquired);
}

// Reports the number of seleted data points from the scan number and scan index 
// provided. Return 0 if requested data is not avaliable.
// The first value retuned is the number of values that will be reported followed
// by the data.
// Parameters:
//  Scan number
//  Scan index
//  Number of values to return
void reportElec(bool pos)
{
  int scanNum,scanIndex,num;

   if(scanBuffer == NULL) return(cp.sendNAK());
   // Get the input parameters
   if(!cp.getValue(&scanNum,1,scanBuffer->Points)) return(cp.sendNAK());
   if(!cp.getValue(&scanIndex,0,scanBuffer->cvscan[scanNum-1]->Points - 1)) return(cp.sendNAK());
   if(!cp.getValue(&num,1,scanBuffer->cvscan[scanNum-1]->Points)) return(cp.sendNAK());
   // See if data is avalible
   cp.sendACK(false);
   if(scanNum > scanBuffer->Acquired) return(cp.println(0));
   if(scanIndex > scanBuffer->cvscan[scanNum-1]->Acquired) return(cp.println(0));
   if((num + scanIndex) > scanBuffer->cvscan[scanNum-1]->Acquired) return(cp.println(0));
   // Now report the data
   cp.print(num);
   for(int i=scanIndex;i<scanIndex+num;i++)
   {
      cp.print(",");
      if(pos) cp.print(Counts2Value(scanBuffer->cvscan[scanNum-1]->dataPos[i],&dms.POSELECmon));
      else    cp.print(Counts2Value(scanBuffer->cvscan[scanNum-1]->dataNeg[i],&dms.NEGELECmon));
   }
   cp.println("");
}
void reportPosElec(void){reportElec(true);}
void reportNegElec(void){reportElec(false);}

void bootloader(void)
{
  __disable_irq();
 	//THESE MUST MATCH THE BOOTLOADER
	#define DOUBLE_TAP_MAGIC 			      0xf01669efUL
	#define BOOT_DOUBLE_TAP_ADDRESS     (HSRAM_ADDR + HSRAM_SIZE - 4)

	unsigned long *a = (unsigned long *)BOOT_DOUBLE_TAP_ADDRESS;
	*a = DOUBLE_TAP_MAGIC;
	//NVMCTRL->ADDR.reg  = APP_START;
	//NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMD_EB | NVMCTRL_CTRLB_CMDEX_KEY;
	
	// Reset the device
	NVIC_SystemReset() ;

	while (true);
}

// Functions supporting Circuit Python file system (CPFS). This is used to save
// the configuration and calibration data to the SPI flash filesystem.
// Provides non volitial storage of setup data.

// Since SdFat doesn't fully support FAT12 such as format a new flash
// We will use Elm Cham's fatfs f_mkfs() to format
#include "./FlashFS/ff.h"
#include "./FlashFS/diskio.h"
void formatFS(void)
{
  cp.println("SPI Flash FatFs Format");

  // Initialize flash library and check its chip ID.
  if (!flash.begin(my_flash_devices, flashDevices)) 
  //if (!flash.begin()) 
  {
    cp.println("Error, failed to initialize flash chip!");
    return;
  }
  cp.print("Flash chip JEDEC ID: 0x"); cp.println(flash.getJEDECID(), HEX);
  cp.print("Flash size: "); cp.print(flash.size() / 1024); cp.println(" KB");
  char *res = cp.userInput("This will erase all data on the SPI flase, enter YES to continue: ");
  cp.println("");
  if(strcmp(res,"YES")==0)
  {
    cp.println("Formatting...");
    // Format fat12
    uint8_t workbuf[4096];
    // Elm Cham's fatfs objects
    FATFS elmchamFatfs;
    // Make filesystem.
    FRESULT r = f_mkfs("", FM_FAT, 0, workbuf, sizeof(workbuf));
    if (r != FR_OK) {
      cp.print("Error, f_mkfs failed with error code: "); cp.println(r, DEC);
      return;
    }
    // mount to set disk label
    r = f_mount(&elmchamFatfs, "0:", 1);
    if (r != FR_OK) {
      cp.print("Error, f_mount failed with error code: "); cp.println(r, DEC);
      return;
    }
    // Setting label
    cp.println("Setting disk label to: DMS");
    r = f_setlabel("DMS");
    if (r != FR_OK) {
      cp.print("Error, f_setlabel failed with error code: "); cp.println(r, DEC);
      return;
    }
    // unmount
    f_unmount("0:");
    // sync to make sure all data is written to flash
    flash.syncBlocks();
    cp.println("Formatted flash!");
  }
}

bool FSsetup(bool report)
{
  // Init external flash
  if (!flash.begin(my_flash_devices, flashDevices)) {if(report) cp.println("Error, failed to initialize flash chip!");}
  //if (!flash.begin()) {if(report) cp.println("Error, failed to initialize flash chip!");}
  else
  {
    if(report) cp.println("Flash chip initalized!");
    if(!fatfs.begin(&flash)) {if(report) cp.println("Failed to mount filesystem!");}
    else
    {
      if(report) cp.println("Mounted filesystem!");
      return true;
    }
  }
  return false;
}

// This function will save all settings to a file, if no file name is passed
// the data is saved to default.dat, the file name is an optional parameter
void saveDefaults(void)
{
  char *fileName = (char *)"default.dat";

  if(cp.getNumArgs()>1) return(cp.sendNAK());
  if(cp.getNumArgs()==1)
  {
    cp.getValue(&fileName);
  }
  if(!FSsetup()) return(cp.sendNAK());
  if((file = fatfs.open(fileName,O_WRITE | O_CREAT))==0) 
  {
    //cp.println("Can't create default.dat!");
  }
  else
  {
    file.write((void *)&dms,sizeof(DMSdata));
    file.close();
    return(cp.sendACK());
  }
  cp.sendNAK();
}

// This function will load all settings from a file, if no file name is passed
// the data is loaded from default.dat, the file name is an optional parameter
void loadDefaults(void)
{
  DMSdata h;
  char *fileName = (char *)"default.dat";

  if(cp.getNumArgs()>1) return(cp.sendNAK());
  if(cp.getNumArgs()==1)
  {
    cp.getValue(&fileName);
  }
  if(!FSsetup()) return(cp.sendNAK());
  if((file = fatfs.open(fileName,O_READ))==0) 
  {
    //cp.println("Can't open default.dat!");
  }
  else
  {
    int num = file.read((void *)&h,sizeof(DMSdata));
    file.close();
    if((num != sizeof(DMSdata)) || (h.Signature != SIGNATURE))
    {
      //cp.println("Error reading default.dat file!");
      return(cp.sendNAK());
    }
    dms = h;
    //cp.print("default.dat read, number of bytes = ");
    //cp.println(num);
    return(cp.sendACK());
  }
  cp.sendNAK();
}

void saveCalibrations(void)
{
  if(!FSsetup()) return;
  if((file = fatfs.open("cal.dat",O_WRITE | O_CREAT))==0) cp.println("Can't create cal.dat!");
  else
  {
    int num = file.write((void *)&dms.DCB1RBmon,sizeof(ADCchan));
    num += file.write((void *)&dms.DCB2RBmon,sizeof(ADCchan));
    num += file.write((void *)&dms.POSELECmon,sizeof(ADCchan));
    num += file.write((void *)&dms.NEGELECmon,sizeof(ADCchan));
    num += file.write((void *)&dms.RFLEVELmon,sizeof(ADCchan));

    num += file.write((void *)&dms.DRIVEctrl,sizeof(DACchan));
    num += file.write((void *)&dms.DCBREFctrl,sizeof(DACchan));
    num += file.write((void *)&dms.DCB1ctrl,sizeof(DACchan));
    num += file.write((void *)&dms.DCB2ctrl,sizeof(DACchan));
    num += file.write((void *)&dms.POSOFFctrl,sizeof(DACchan));
    num += file.write((void *)&dms.POSZEROctrl,sizeof(DACchan));
    num += file.write((void *)&dms.NEGOFFctrl,sizeof(DACchan));
    num += file.write((void *)&dms.NEGZEROctrl,sizeof(DACchan));

    num += file.write((void *)dms.LUVrf,sizeof(float) * 21);
    file.close();
    cp.print("cal.dat written, number of bytes = ");
    cp.println(num);
  }
}

void loadCalibrations(void)
{
  if(!FSsetup()) return;
  if((file = fatfs.open("cal.dat",O_READ))==0) cp.println("Can't open cal.dat!");
  else
  {
    int num = file.read((void *)&dms.DCB1RBmon,sizeof(ADCchan));
    num += file.read((void *)&dms.DCB2RBmon,sizeof(ADCchan));
    num += file.read((void *)&dms.POSELECmon,sizeof(ADCchan));
    num += file.read((void *)&dms.NEGELECmon,sizeof(ADCchan));
    num += file.read((void *)&dms.RFLEVELmon,sizeof(ADCchan));

    num += file.read((void *)&dms.DRIVEctrl,sizeof(DACchan));
    num += file.read((void *)&dms.DCBREFctrl,sizeof(DACchan));
    num += file.read((void *)&dms.DCB1ctrl,sizeof(DACchan));
    num += file.read((void *)&dms.DCB2ctrl,sizeof(DACchan));
    num += file.read((void *)&dms.POSOFFctrl,sizeof(DACchan));
    num += file.read((void *)&dms.POSZEROctrl,sizeof(DACchan));
    num += file.read((void *)&dms.NEGOFFctrl,sizeof(DACchan));
    num += file.read((void *)&dms.NEGZEROctrl,sizeof(DACchan));

    num += file.read((void *)dms.LUVrf,sizeof(float) * 21);

    file.close();
    cp.print("cal.dat read, number of bytes = ");
    cp.println(num);
  }
}
// End of CPFS

// Adjusts the zero DAC output volts to set the electrometer near zero.
// Note that electrometer is clamped at zero so the algorith sets to near 
// zero. 
void ZeroElectrometer(void)
{
  int i;
  float Ival;

  // Read the positive channel current, average several readings and
  // adjust the zero channel to set to 1 to 5 range
  for(i=0;i<25;i++)
  {
    Ival  = AnalogIn(readadc, &dms.POSELECmon);
    Ival += AnalogIn(readadc, &dms.POSELECmon);
    Ival += AnalogIn(readadc, &dms.POSELECmon);
    Ival += AnalogIn(readadc, &dms.POSELECmon);
    Ival /= 4; 
    //cp.println(Ival);
    if((Ival > 2) && (Ival < 10)) break;
    // Adjust zero voltage
    // cp.print("Zero val :");
    // cp.println(dms.ElectPosZero);
    dms.ElectPosZero += (Ival - 7) * - 0.01;
    if(dms.ElectPosZero < 0.0) dms.ElectPosZero = 0.0;
    if(dms.ElectPosZero > 5.0) dms.ElectPosZero = 5.0;
    DACupdate(dms.ElectPosZero,   &dms.POSZEROctrl);
    delay(25);    
  }
 // Read the negative channel current, average several readings and
 // adjust the zero channel to set to 1 to 5 range
  for(i=0;i<25;i++)
  {
    Ival  = AnalogIn(readadc, &dms.NEGELECmon);
    Ival += AnalogIn(readadc, &dms.NEGELECmon);
    Ival += AnalogIn(readadc, &dms.NEGELECmon);
    Ival += AnalogIn(readadc, &dms.NEGELECmon);
    Ival /= 4; 
    //cp.println(Ival);
    if((Ival > 2) && (Ival < 10)) break;
    // Adjust zero voltage
    //cp.print("Zero val :");
    //cp.println(dms.ElectPosZero);
    dms.ElectNegZero += (Ival - 7) * -0.01;
    if(dms.ElectNegZero < 0.0) dms.ElectNegZero = 0.0;
    if(dms.ElectNegZero > 5.0) dms.ElectNegZero = 5.0;
    DACupdate(dms.ElectNegZero,   &dms.NEGZEROctrl);
    delay(25);    
  }
  cp.sendACK();
}

void setTimeDate(void)
{
  char *td;
  int  h,m,s,d,y;
  DateTime dt = rtc.now();

  cp.getValue(&td);
  // Scan for valid time
  if(sscanf(td,"%2d:%2d:%2d",&h,&m,&s) == 3)
  {
    // Here with valid time format
    setDateTimeTime(dt, h, m, s);
    rtc.adjust(dt);
    cp.sendACK();
    return;
  }
  if(sscanf(td,"%2d/%2d/%4d",&d,&m,&y) == 3)
  {
    // Here with valid date format
    setDateTimeDate(dt, y, m, d);
    rtc.adjust(dt);
    cp.sendACK();
    return;
  }
  // If here on valid format found
  cp.sendNAK();
}

void getTimeDate(void)
{
  char buffer[] = {"DDD, DD MMM YYYY hh:mm:ss"};
  DateTime now = rtc.now();
  cp.sendACK(false);
  cp.println(now.toString(buffer));
}

void getTime(void)
{
  char buffer[] = {"hh:mm:ss"};
  DateTime now = rtc.now();
  cp.sendACK(false);
  cp.println(now.toString(buffer));
}

void getDate(void)
{
  char buffer[] = {"DD/MM/YYYY"};
  DateTime now = rtc.now();
  cp.sendACK(false);
  cp.println(now.toString(buffer));
}

void powerON(void)
{
  ON;
  cp.sendACK();
}

void powerOFF(void)
{
  OFF;
  cp.sendACK();
}

// RTC alarm callback (rtc.attachInterrupt, armed by schedule()). Fires once
// per scheduled interval: flags scanNow so loop() runs the next scan via
// scanAndSave(), then re-arms the RTC alarm for the following interval
// (unless the requested number of scans has already been collected).
void scanTime(uint32_t flag)
{
  if(scanFlag == false) return;
  if(collectedScan >= numSchScans)
  {
    // If here the scans have been collected!
    scanFlag = false;
    scanNow = false;
    return;
  }
  scanNow = true;
  DateTime now = rtc.now();
  TimeSpan ts = interval;
  DateTime alarm = now + ts;
  rtc.setAlarm(0,alarm);
  rtc.enableAlarm(0, rtc.MATCH_HHMMSS); // match Every Day
}

// This command will schedule a set of data collection scans. Its is assumed the system
// parameters are allready set for the scan. The DMS system will be placed in low power mode 
// waiting for the acrquire time, when triggered to acquire it will power up, perform the
// scan, then write the data to flash memory, and finally go back to low power mode.
//
// This command expects the following parameters:
//  Base file name
//  Interval in secs
//  Number of scans to perform
void schedule(void)
{
  char *fp;
  char fileSpec[41];

  // Test current status and return NAK if system is currently in scheduled scan mode
  if(scanFlag) return(cp.sendNAK());
  // Read parameters and validate, test base file name to make sure its not in use
  if(cp.getValue(&fp) == false) return(cp.sendNAK());
  strcpy(baseFileName,fp);
  if(cp.getValue(&interval) == false) return(cp.sendNAK());
  if(cp.getValue(&numSchScans) == false) return(cp.sendNAK());
  // Test if file name has been used
  sprintf(fileSpec,"%s.*",fp);
  if(isFile(fileSpec))
  {
    // Here if file name is in use
    return(cp.sendNAK());
  }
  collectedScan = 0;
  scanFlag = true;
  scanNow = false;
  // Schedule the first scan
  DateTime now = rtc.now();
  TimeSpan ts = interval;
  DateTime alarm = now + ts;
  rtc.setAlarm(0,alarm);
  rtc.enableAlarm(0, rtc.MATCH_HHMMSS);
  rtc.attachInterrupt(scanTime); // callback while alarm is match
  // Turn off and disable system
  dms.Enable = false;
  Update();
  OFF;  
  cp.sendACK();
}

void stop(void)
{
  scanFlag = false;
  scanNow = false;
  cp.sendACK();
}

// SENA,TRUE|FALSE: sets dms.Enable and immediately runs one Update() pass so
// the RF drive is applied/removed right away, rather than waiting for the
// next 100mS tick (see version history note in the file header, v1.4).
void setEnable(void)
{
  char *value;

  if(!cp.getValue(&value,"TRUE,FALSE")) return(cp.sendNAK());
  if(strcmp(value,"TRUE") == 0) dms.Enable = true;
  else dms.Enable = false;
  cp.sendACK();
  Update();
}
