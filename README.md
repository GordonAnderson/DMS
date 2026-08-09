# DMS

Firmware for the DMS (Differential Mobility Spectrometry) controller board. It
drives a FAIMS/DMS RF drive circuit, controls the CV/Bias DC bias electrodes,
reads the positive/negative electrometer channels, and supports both
interactively-triggered and scheduled unattended scanning with on-board data
logging to an external SPI flash chip.

## Hardware

- **MCU:** Microchip/Atmel ATSAMD51G19A (Cortex-M4F @ 120 MHz, 512 KB flash,
  192 KB RAM), on an Adafruit ItsyBitsy M4 board.
- **RF drive:** eight 12-bit analog outputs generated with the SAMD51's TCC
  timers running as PWM-based DACs (see [src/Timers.cpp](src/Timers.cpp)).
- **Analog inputs:** processor 12-bit ADCs, oversampled and windowed for CV
  scanning (see [src/ADC.cpp](src/ADC.cpp)).
- **External storage:** 2 MB SPI NOR flash (Macronix MX25V1635F) exposed to a
  host PC as a USB mass-storage drive, and used internally to hold
  configuration, calibration, and scan data files (FAT filesystem via
  ChaN's FatFs, see [src/FlashFS/](src/FlashFS/)).
- **Environment sensor:** Bosch BME280 (temperature / humidity / pressure) on
  I²C, used for altitude-compensated pressure readings.
- **Power monitor:** TI INA237 bus voltage/current monitor on I²C (can be
  compiled out with `NOINA237`, see [include/DMS.h](include/DMS.h)).
- **RTC:** SAMD51 on-chip RTC (via the Seeed Arduino RTC library) used for
  timestamps and scheduled-scan alarms.
- **Status indicator:** onboard DotStar RGB LED.

## Firmware architecture

- [src/DMS.cpp](src/DMS.cpp) — application entry point (`setup()` / `loop()`),
  the serial command table, scan state machine, calibration file I/O, and USB
  mass-storage callbacks.
- [src/ADC.cpp](src/ADC.cpp) — low-level SAMD51 ADC0/ADC1 configuration and
  window-compare interrupt handlers used during CV scans.
- [src/Timers.cpp](src/Timers.cpp) — SAMD51 TCC/TC timer configuration:
  8-channel 12-bit PWM "DAC" outputs, the FAIMS drive frequency/duty timer
  (TC1), and the CV-scan step timer (TC3).
- [src/Calibration.cpp](src/Calibration.cpp) — interactive two-point
  calibration routines for the DAC/ADC channels and the Vrf drive-level
  lookup table.
- [src/FlashFS/](src/FlashFS/) — a local copy of ChaN's FatFs (`ff.c`/`ff.h`)
  plus the `diskio.cpp` glue that binds it to the `Adafruit_SPIFlash` object
  (`flashFS`) defined in `DMS.cpp`. See the note in
  [TODO.md](TODO.md#build-system) on why this lives here instead of only in
  `GAACE_Core`.
- [include/DMS.h](include/DMS.h) — pin assignments, the persisted `DMSdata`
  settings structure, and shared declarations.

Runtime dependencies (RF drive control, thread scheduling, serial command
dispatch, ADC/DAC calibration helpers) come from the
[GAACE_Core](https://github.com/GordonAnderson/GAACE_Core) library and a set
of Adafruit/third-party libraries, all pulled in via `lib_deps` in
[platformio.ini](platformio.ini).

### Main loop

`setup()` brings up the flash filesystem, USB mass storage, BME280, INA237,
PWM outputs, serial command processor, and RTC, then starts a 100 ms
`ThreadController` tick (`Update()`) that:

1. Applies the current `dms` settings to all DAC outputs (drive level, DCB
   reference, DCB1/DCB2, electrometer offset/zero).
2. Reads back and filters all monitored ADC channels.
3. Runs the closed-loop Vrf control step when `dms.Mode` is enabled.

`loop()` services the command processor and thread controller every pass, and
kicks off a scheduled scan (`scanAndSave()`) when one has been armed with the
`SCHEDULE` command.

### Scanning

A scan is a 2-D sweep: an outer loop over Vrf (RF drive level) steps, and for
each Vrf step, an inner CV (compensation voltage) sweep timed by TC3
(`performCVscan()` / `CVscanISR()`). Scan buffers are heap-allocated per scan
(`allocateScanBuffer()` / `freeScanBuffer()`) sized from the current
`dms.VRFsteps` / `dms.CVsteps` settings, and multiple CV sweeps can be
averaged per Vrf step (`dms.Averages`). Completed scans can be read back over
the command interface point-by-point, or written to the flash filesystem as a
plain-text file (`saveScan` / `readScan`).

### Persisted settings

The `DMSdata` structure (include/DMS.h) holds every tunable parameter —
RF drive settings, DC bias/CV setpoints, electrometer zero/offset,
ADC acquisition parameters, per-channel calibration (`ADCchan`/`DACchan`
slope+intercept pairs), scan parameters, and the Vrf-vs-drive-level lookup
table. It is guarded by a `Signature` field and can be persisted two ways:

- `SAVE`/`RESTORE` — round-trips the whole structure through the SAMD51's
  emulated-EEPROM flash region (`FlashStorage` library). This survives power
  cycles but **not** a firmware re-upload.
- `SAVEF`/`LOADF` (and `SAVECAL`/`LOADCAL` for just the calibration fields) —
  round-trips through a file on the external SPI flash filesystem
  (`default.dat` / `cal.dat` by default). This survives firmware re-uploads.

## Serial command interface

Commands arrive as ASCII over USB CDC serial, one per line, in
`CMD` (get) / `CMD,<args>` (set) form; table entries whose name starts with
`?` accept either form. See `dbsCmds[]` in
[src/DMS.cpp](src/DMS.cpp#L280) for the authoritative, self-documenting list
(each entry carries its own one-line description) — categories include:

- System: `GVER`, `GNAME`/`SNAME`, `SAVE`/`RESTORE`, `FORMAT`, `BLOAD`
- Time/date: `STIMEDATE`, `GTIME`, `GDATE`, `GTIMEDATE`
- Flash filesystem file I/O: `SAVEF`/`LOADF`, `SAVECAL`/`LOADCAL`, `FORMATFS`,
  `DIR`, `DEL`, `MORE`
- Power/enable: `ON`/`OFF`, `SENA`/`GENA`, `SMODE`/`GMODE`
- RF drive: `SFREQ`/`GFREQ`, `SDUTY`/`GDUTY`, `SDRV`/`GDRV`, `SVRF`, `SVRFF`,
  `GVRF`/`GVRFV`
- ADC acquisition tuning: `?PRE`, `?SNUM`, `?SLEN`
- Electrometers: `RPOSELEC`/`RNEGELEC`, `S/GPOSOFF`, `S/GPOSZERO`,
  `S/GNEGOFF`, `S/GNEGZERO`, `ELTMTRZERO`
- DC bias: `SCV`/`GCV`/`GCVV`, `SBIAS`/`GBIAS`/`GBIASV`
- Scanning: `SCVSTRT`/`SCVEND`, `SVRFSTRT`/`SVRFEND`, `SVRFSTEPS`,
  `SNUMAVG`, `SSTEPDUR`, `SNUMSTP`, `SCNSTRT`/`SCNSTP`, `SCANSTAT`,
  `RSCNPOS`/`RSCNNEG`, `RNUMSCANS`, `RSCNPTS`, `SSCAN`/`RSCAN`
- Scheduled/unattended scanning: `SCHEDULE`, `STOP`, `ISSCH`, `GSCNNUM`
- Calibration: `CALDCBREF`, `CALDCB1`, `CALDCB2`, `CALVRF`, `CALDRV2VRF`,
  `SGAIN`/`GGAIN`
- Environment sensor: `RENV`, `GTEMP`, `GPRESS`, `GHUM`, `GALT`, `SALT`
- Debug: `TCC`, `TC1`, `ADCS`

## Building

This is a [PlatformIO](https://platformio.org/) project targeting the
`adafruit_itsybitsy_m4` board.

```sh
pio run              # build
pio run -t upload    # build and flash
pio device monitor   # open a serial terminal to talk to the command interface
```

Dependencies (including `GAACE_Core`) are declared in
[platformio.ini](platformio.ini) and fetched automatically on first build
into `.pio/libdeps/`.

## Known issues

See [TODO.md](TODO.md) for a tracked list of bugs found during review that
still need attention.
