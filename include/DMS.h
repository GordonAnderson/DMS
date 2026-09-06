//
// DMS.h
//
// Pin assignments, the persisted settings structure (DMSdata), scan status
// types, and the function prototypes shared by DMS.cpp, Calibration.cpp and
// external callers. See README.md for the overall firmware architecture.
//
#ifndef DMS_h
#define DMS_h

#include <Devices.h>
#include <commandProcessor.h>

// Magic value stored in DMSdata.Signature; used to detect uninitialized or
// stale (wrong-version) settings data read back from flash/EEPROM.
#define SIGNATURE  0xAA55A5A5

// Macros
#define ON          digitalWrite(POWER,LOW)
//#define OFF         digitalWrite(POWER,HIGH)
// For now disable the power off function, may need a hardware update to make this safe.
// NOTE: as written, OFF is currently identical to ON (see TODO.md #6) — the
// "OFF" command does not actually remove power yet.
#define OFF         digitalWrite(POWER,LOW)
// Disable input voltage monitor
#define NOINA237

// Digitial output line used for 12 bit PWM
#define DRIVE       4
#define DCBREF      5
#define DCB1        7
#define DCB2        9

#define POSOFF      10
#define POSZERO     11
#define NEGOFF      12
#define NEGZERO     13

// Analog input channels
#define POSELEC     A0
#define RFLEVEL     A1
#define DCB1RB      A2
#define NEGELEC     A3
#define DCB2RB      A4

// DIO
#define PULSE       2   // FB transformer pulse
#define POWER       A5  // Turns on and off 3.3V power to DMS electronics

// Global state machine for a scan in progress, tracked in scanStatus
// (DMS.cpp) and reported by the SCANSTAT command (ScanStat()).
typedef enum
{
  SCAN_IDLE,              // No scan started since power-up
  SCAN_FINISHED,          // Last scan (all Vrf steps) completed normally
  SCAN_CVactive,          // A CV sweep is currently running (performCVscan)
  SCAN_CVcomplete,        // The current CV sweep's points are all acquired
  SCAN_ABORT,             // SCNSTP was received; scan is unwinding
  SCAN_FAILED,            // performScan()'s Vrf-step loop broke early for a
                          // reason other than SCAN_ABORT (unexpected)
  SCAN_ALLOCATIONfailed   // Heap allocation for the scan buffer failed
} ScanStatus;

// One CV (compensation voltage) sweep at a single Vrf step. dataPos/dataNeg
// hold raw ADC counts (not engineering units) for the positive/negative
// electrometer channels; apply Counts2Value()/dms.POSELECmon or
// dms.NEGELECmon to convert. Heap-allocated per scan by allocateScanBuffer().
typedef struct
{
  float   Vrf;
  float   CVstart;
  float   CVend;
  int     Points;
  int     Acquired;
  uint16_t *dataPos;
  uint16_t *dataNeg;
} CVscan;

// A full 2-D scan: an outer sweep over Vrf steps, each holding one CVscan.
// cvscan is an array of Points pointers, heap-allocated/freed as a unit by
// allocateScanBuffer()/freeScanBuffer() in DMS.cpp.
typedef struct
{
  float   VRFstart;
  float   VRFend;
  int     Points;
  int     Acquired;
  int     Averages;
  CVscan  **cvscan;
} Scan;

typedef struct
{
  float        POSelec;    // Positive electrometer channel current
  float        NEGelec;    // Negative electrometer channel current
  float        DCB1v;      // DCB channel 1
  float        DCB2v;      // DCB channel 2
  float        CV;         // CV voltage
  float        BIAS;       // Bias voltage
  float        Vbat;       // Battery voltage
  float        Vrf;        // Vrf actual voltage
} ReadBacks;


typedef struct
{
  int16_t       Size;                   // This data structures size in bytes
  char          Name[20];               // Holds the board name, "FAIMSFB"
  int8_t        Rev;                    // Holds the board revision number
  bool          Enable;                 // Turns the FAIMS drive on or off
  int           Freq;                   // FAIMS frequency
  int           Duty;                   // FAIMS duty cycle
  float         Drive;                  // Drive level, in percentage
  float         Vrf;                    // Setpoint voltage
  bool          Mode;                   // True if in closed loop control
  float         loopGain;
  float         CV;                     // CV setpoint
  float         Bias;                   // Bias setpoint
  float         DCref;
  float         DCB1v;
  float         DCB2v;
  // Electrometer settings
  float         ElectPosZero;
  float         ElectNegZero;
  float         ElectPosOffset;
  float         ElectNegOffset;
  // Limits
  float         MaxDrive;
  float         MaxCurrent;
  float         MaxPower;
  float         MaxOnTime;              // Defines the number of hours before the system automatically shuts down 
  // ADC acquire parameters
  int           preScaler;
  int           sampleNum;
  int           sampLen;
  // ADC channels, uses processor ADC inputs
  ADCchan       DCB1RBmon;              // DC bias channel 1 voltage monitor
  ADCchan       DCB2RBmon;              // DC bias channel 2 voltage monitor
  ADCchan       POSELECmon;             // Positive electrometer channel
  ADCchan       NEGELECmon;             // Negative electrometer channel
  ADCchan       RFLEVELmon;             // RF level detector
  // DAC channels, uses 12 bit PWM outputs
  DACchan       DRIVEctrl;              // Drive level control
  DACchan       DCBREFctrl;             // DCB reference voltage
  DACchan       DCB1ctrl;               // DCB1 voltage
  DACchan       DCB2ctrl;               // DCB2 voltage
  DACchan       POSOFFctrl;             // Electrometer positive voltage setpoint
  DACchan       POSZEROctrl;            // Electrometer positive voltage zero control
  DACchan       NEGOFFctrl;             // Electrometer negative voltage setpoint
  DACchan       NEGZEROctrl;            // Electrometer negative voltage zero control
  // Scanning parameters
  float         VRFstart;
  float         VRFend;
  float         CVstart;
  float         CVend;
  // Step based scanning
  int           CVsteps;
  int           CVstepDuration;         // in mSec
  int           Averages;
  int           VRFsteps;
  bool          EnableExtStep;          // Enable the use of external step advance input
  int           ExtAdvInput;            // Define the pin to be used to advance scanning,
  // External scan trigger options
  char          ScanTrigger;            // Trigger input channel
  int8_t        TriggerLevel;           // Trigger level, 0,CHANGE,RISING, or FALLING
  // Lookup table for drive level to Vrf calibration
  float         LUVrf[21];
  // Sea level pressure calibration factor
  float         SeaLevelPress;
  //
  unsigned int  Signature;              // Must be 0xAA55A5A5 for valid data
} DMSdata;

extern commandProcessor cp;
extern DMSdata   dms;
extern ReadBacks rb;

void SetVrfCmd(void);
void SetVrfTableCmd(void);
void setCV(void);
void SetBias(void);
void StartScan(void);
void StopScan(void);

bool FSsetup(bool rerport=false);
void SaveSettings(void);
void RestoreSettings(void);
void FormatFLASH(void);
void saveScan(void);
void readScan(void);
void deleteFile(void);
void listFiles(void);
void moreFile(void);

void setTCC(void);
void setTC1(void);
void readADCs(void);
void printBME280(void);

void bootloader(void);

void saveDefaults(void);
void loadDefaults(void);
void saveCalibrations(void);
void loadCalibrations(void);
void formatFS(void);

void ZeroElectrometer(void);

void ScanStat(void);
void numScans(void);
void scanPoints(void);
void reportPosElec(void);
void reportNegElec(void);

void DACupdate(float value, DACchan *chan);
int readadc(int8_t chan);
void schedule(void);
void stop(void);
void setEnable(void);
void powerON(void);
void powerOFF(void);
void getTimeDate(void);
void getTime(void);
void getDate(void);
void setTimeDate(void);
void getTemp(void);
void getPress(void);
void getHum(void);
void getAlt(void);
void setAltitude(void);

bool saveScanAscii(char *fileName);
bool performScan(bool ackFlag);
int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize);
int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize);
void msc_flush_cb (void);

#endif
