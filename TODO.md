# TODO / Known Issues

Bugs and risk items found during code review passes. Fixed items are kept
here (marked Fixed) as a record of what changed and why, so the reasoning
isn't lost; open items need a decision from someone who can validate against
real hardware, or are flagged so they don't get silently "fixed" by someone
unaware of the context.

## Fixed — 2026-09-05 (v1.5)

### `RNUMSCANS` command crashed when used
**File:** [src/DMS.cpp](src/DMS.cpp) — the `dbsCmds` command table.

The table entry cast the wrong symbol to a function pointer:
```cpp
{"RNUMSCANS", CMDfunction, 0, (void *)numSchScans,NULL, "Returns the number of recorded scans"},
```
`numSchScans` is the `int` variable used by the SCHEDULE feature (number of
scans to perform), not the `numScans()` function that actually reports the
recorded-scan count. The command dispatcher does
`void (*function)(void) = (void (*)(void))c->pointer; function();`, so
sending `RNUMSCANS` jumped execution to whatever small integer value
`numSchScans` held — a near-certain crash. Fixed to `(void *)numScans`.

### `allocateScanBuffer()` could null-deref or free garbage pointers
**File:** [src/DMS.cpp](src/DMS.cpp) — `allocateScanBuffer()`.

`scanBuffer->cvscan = new(std::nothrow) CVscan *[dms.VRFsteps];` was used
without checking the result before the following loop dereferenced it, and
`new T*[n]` does not zero-initialize the array — so if allocation failed
partway through populating entries, the un-populated slots held
indeterminate garbage rather than NULL, which `freeScanBuffer()` would later
try to `delete`. Fixed by value-initializing the array (`...[dms.VRFsteps]()`,
which zeroes every pointer) and checking it for NULL before use.

### `CVscanISR()` commanded one CV step past `CVend`
**File:** [src/DMS.cpp](src/DMS.cpp) — `CVscanISR()`.

On the tick that recorded the last valid point of a CV sweep, the function
still fell through to the step-advance code and drove DCB1/DCB2 to
`CVstart + CVstep*Points`, i.e. `CVend + CVstep` — one step beyond the
configured range — for up to one `CVstepDuration` before the next tick
noticed completion. Fixed: completion is now flagged immediately after
recording the last point, so the sweep never commands a voltage past CVend.

### `setFreqDuty()` waited on the wrong timer, and could divide by zero
**File:** [src/Timers.cpp](src/Timers.cpp) — `setFreqDuty()`.

Wrote `TC1->COUNT16.CC[0].reg` (frequency) but waited on `tcIsSyncing(TC2)`
— TC2 isn't touched by this function, so the wait did nothing to confirm the
frequency write had taken effect before the duty write followed. Also
divided by `frequency` with no guard; `SFREQ` range-checks `dms.Freq` before
it reaches here, but the `TC1,<freq>,<duty>` debug command
([src/DMS.cpp](src/DMS.cpp), `setTC1()`) passed its argument straight
through unchecked, so `TC1,0,50` would divide by zero. Fixed the sync wait
to check `TC1`, and added a `frequency <= 0` guard both in `setFreqDuty()`
and in `setTC1()`.

### `scanStatus`/`scanNow`/`scanFlag` not `volatile`
**Files:** [src/DMS.cpp](src/DMS.cpp).

`scanStatus` is written by `CVscanISR()` (TC3 interrupt context) and polled
in a plain `while(true){...}` loop in `performCVscan()` on the main thread;
`scanNow`/`scanFlag` have the same pattern between the RTC alarm ISR
(`scanTime()`) and `loop()`. None were declared `volatile`, so the compiler
was technically free to cache a read and never observe the ISR's write —
this "worked" only because the loop bodies call external functions the
compiler can't prove are side-effect-free. All three are now `volatile`.

### `RSCNPOS`/`RSCNNEG` could never report data for the scan in progress
**File:** [src/DMS.cpp](src/DMS.cpp) — `reportElec()` (shared by both
commands).

```cpp
if(scanNum > scanBuffer->Acquired) return(cp.println(0));
```
`scanBuffer->Acquired` counts Vrf steps that have *fully* completed; the
step currently being acquired is 0-based index `Acquired` itself and hasn't
been counted yet, so its 1-based `scanNum` is `Acquired + 1`. Since
`Acquired + 1 > Acquired` is always true, this check rejected exactly the
scan that's actively running — `RSCNPOS`/`RSCNNEG` always reported "0
values" for it, no matter how many points had already landed. This
contradicted the file header's own description of the read commands
("Commands alow reading the scan data while it is being acquired") and was
inconsistent with the sibling command `RSCNPTS` (`scanPoints()`), which has
no such gate and correctly reports live progress. Fixed the bound to
`scanNum > scanBuffer->Acquired + 1`; the existing per-point check just
below it already correctly limits what's actually read to points acquired
so far, so no data can be read out ahead of when it's written.

### A failed scan allocation left a stale, unsafe `scanBuffer` around
**File:** [src/DMS.cpp](src/DMS.cpp) — `performScan()`, `freeScanBuffer()`.

If `allocateScanBuffer()` failed partway through (plausible with a large
`VRFsteps`/`CVsteps` request on a 192KB-RAM part), `performScan()` recorded
`SCAN_ALLOCATIONfailed` and returned false, but never freed or NULLed the
global `scanBuffer` — leaving it non-NULL but only partially built, an
invariant nothing else in the file expects (`RSCNPOS`/`RSCNNEG`/`RSCNPTS`
and `freeScanBuffer()`'s own retry path all trust that a non-NULL
`scanBuffer` is fully valid). Depending on exactly where allocation failed,
a subsequent `RSCNPOS`/`RSCNNEG`/`RSCNPTS`, or simply retrying `SCNSTRT`
(which calls `freeScanBuffer()` first), could dereference an unpopulated
`cvscan[i]` slot or index through a NULL top-level `cvscan` array. Fixed by
having `performScan()` call `freeScanBuffer()` on the allocation-failure
path, and hardening `freeScanBuffer()` itself to skip the `cvscan[]` walk
entirely when the top-level array is NULL, so it can safely tear down a
partially-built scan buffer from any failure point.

### Aborting a scan was reported as "Finished"
**File:** [src/DMS.cpp](src/DMS.cpp) — `performScan()`, `StartScan()`,
`ScanStat()`. **File:** [include/DMS.h](include/DMS.h) — `ScanStatus` enum.

At the end of the Vrf-step loop, `performScan()` unconditionally set
`scanStatus = SCAN_FINISHED` and returned `true`, even when the loop broke
early because `SCNSTP` set `scanStatus = SCAN_ABORT` (or some other
unexpected `performCVscan()` failure). So `SCANSTAT` reported "Finished"
after a deliberate abort, and `StartScan()`/`SCNSTRT` never reflected that
the scan didn't actually complete. Fixed: `scanStatus` is now only set to
`SCAN_FINISHED` when the loop reached every Vrf step (`Acquired >= Points`);
an early break preserves `SCAN_ABORT` if that's why it stopped, or sets the
previously-unused `SCAN_FAILED` enum value (added to `ScanStat()`'s switch)
for any other case. `performScan()`'s return value now reflects real
completion too — which also means `scanAndSave()` (the scheduled-scan
feature) no longer saves a scan file for a run that was aborted or failed
partway through. `StartScan()` was adjusted to match: it only sends a NAK
when `performScan()` never got as far as sending its own ACK (i.e.
`SCAN_ALLOCATIONfailed`), since a false return after that point (abort/
failure) must not add a second, unsolicited reply to the same `SCNSTRT`
command — that outcome is reported via `SCANSTAT`.

## Fixed — 2026-08-08 (earlier review pass)

The four items below were found in the 2026-08-08 review and have since
been fixed in the code (this section is kept only as a record — none of
them are still open):
- `readScan()` discarded the result of `Value2Counts()` instead of storing
  it, corrupting reloaded scan data.
- `scanPoints()` had an off-by-one range check (`>` instead of `>=`) that
  let one out-of-range index through.
- `freeScanBuffer()` freed scalar `new` allocations with array `delete[]`.
- The CV-averaging loop in `performScan()` didn't check `performCVscan()`'s
  return value on repeats, so an abort/failure mid-average still got
  coadded into the result.

## Needs hardware verification (not changed)

### `RED`/`GREEN` DotStar color macros may be swapped
**File:** [src/DMS.cpp](src/DMS.cpp), near the `statusLED()` helper:
```cpp
#define RED    strip.Color(0, 150, 0)
#define GREEN  strip.Color(150, 0, 0)
```
`Adafruit_DotStar::Color(r, g, b)` takes logical R/G/B values — the
`DOTSTAR_BRG` argument already passed to the `Adafruit_DotStar` constructor
is what accounts for the physical wire order, so `Color()` itself should not
need the channels pre-swapped. As written, `RED` is weighted toward green and
`GREEN` toward red. Left alone because this is easy to get backwards without
the actual board in hand — please confirm against the physical LED and swap
the two definitions if they're indeed backwards.

## Known limitations (intentional, documented in code, listed here so they stay visible)

### `OFF` doesn't currently turn power off
**File:** [include/DMS.h](include/DMS.h):
```cpp
#define ON          digitalWrite(POWER,LOW)
// For now disable the power off function, may need a hardware update to make this safe
#define OFF         digitalWrite(POWER,LOW)
```
`OFF` (and therefore the `OFF` command / `powerOFF()`) currently does exactly
what `ON` does. This is intentional per the existing comment pending a
hardware change, but is easy to miss when reading `powerOFF()` in isolation —
noting it here so it isn't lost, and so a firmware change elsewhere doesn't
accidentally start relying on `OFF` actually cutting power before the
hardware supports it safely.

### `dms.MaxOnTime` is declared but never implemented
**File:** [include/DMS.h](include/DMS.h):
```cpp
float MaxOnTime; // Defines the number of hours before the system automatically shuts down
```
This field is persisted in `DMSdata` (and read/written by `saveDefaults()`/
`loadDefaults()`), but nothing in the firmware ever reads it — there's no
`SMAXONTIME`/`GMAXONTIME` command and no check against elapsed on-time
anywhere in `Update()` or `loop()`. The auto-shutdown safety behavior the
comment describes does not currently exist. Flagging here rather than
implementing it, since doing so is a feature decision (what "shuts down"
should mean, interacting with the `OFF`-doesn't-cut-power limitation above)
rather than a bug fix.

## Build system

### `src/FlashFS/` is a duplicated copy of `GAACE_Core`'s FatFs sources
`GAACE_Core`'s `library.json` excludes its own `src/FlashFS/` folder from the
library build (`"build": {"srcFilter": ["+<*>", "-<FlashFS>"]}`), and the
library's include path only covers `src/`, not `src/FlashFS/`. That means
neither the `ff.c`/`diskio.cpp` implementations nor the `ff.h`/`diskio.h`
headers were reachable from this project as originally structured, which
previously caused a link failure (`undefined reference to f_mkfs` etc. —
see git history / prior fix). `ff.c`, `ff.h`, `ffconf.h`, `diskio.h`,
`diskio.cpp`, and `flash_config.h` were copied into
[src/FlashFS/](src/FlashFS/) in this project so PlatformIO's default
recursive `src/` build compiles them directly.

This works, but it's a duplicated copy: if `GAACE_Core`'s FlashFS
implementation changes upstream, this project's copy will not pick up the
change automatically, and the two can drift. The durable fix is upstream —
remove the `"-<FlashFS>"` exclusion (and confirm the include path covers the
subfolder) in the `GAACE_Core` repository so any consuming project can depend
on it normally. Flagging here rather than doing it myself since it touches a
separate repository with its own release/versioning.

## Minor / cosmetic (not fixed, low priority)

- `RestoreSettings()` returns `ERR_EEPROMWRITE` when the *read-back* fails a
  signature check — the error code name suggests a write failure. Cosmetic
  only; doesn't affect behavior.
- The two-point calibration loop in `CalibrateVrf()`
  ([src/Calibration.cpp](src/Calibration.cpp)) sums 1024 raw 12-bit
  `analogRead()` samples and divides by 64 rather than 1024. This looks odd
  at first read but is arithmetically intentional: `sum/64 == (sum/1024)*16`,
  which reproduces the same ×16 (12-bit → 16-bit) scaling that `readadc()`
  applies elsewhere via `<< 4`, with less rounding error than dividing first.
  Left a clarifying comment in place; no functional change needed.
