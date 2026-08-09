# TODO / Known Issues

Bugs and risk items found during a code review pass (2026-08-08). None of
these have been fixed unless noted — they need a decision from someone who
can validate against real hardware, or are flagged here specifically so they
don't get silently "fixed" by someone unaware of the context.

## Bugs

### 1. `readScan()` stores the wrong value into scan data buffers
**File:** [src/DMS.cpp](src/DMS.cpp) — inside `readScan()`, the two per-point
read loops (positive and negative electrometer data).

```cpp
Value2Counts(fval,&dms.POSELECmon);
scanBuffer->cvscan[i]->dataPos[j] = fval;
```

The call to `Value2Counts()` converts the ASCII engineering value (`fval`,
e.g. a pA reading written by `saveScanAscii()` via `Counts2Value()`) back to
raw ADC counts — but the **return value is discarded**, and the raw
engineering-unit float is truncated straight into a `uint16_t` counts slot
instead. Every scan reloaded with `RSCAN` ends up with nonsense data (e.g. a
3.45 pA reading becomes a raw count of `3`), and any later `RSCNPOS`/`RSCNNEG`
report or re-save of that scan will apply calibration a second time on top of
already-wrong data.

**Fix:** use the conversion result:
```cpp
scanBuffer->cvscan[i]->dataPos[j] = Value2Counts(fval,&dms.POSELECmon);
...
scanBuffer->cvscan[i]->dataNeg[j] = Value2Counts(fval,&dms.NEGELECmon);
```
Same pattern appears twice (positive and negative loops).

### 2. `scanPoints()` off-by-one lets an out-of-range scan index through
**File:** [src/DMS.cpp](src/DMS.cpp) — `scanPoints()`.

```cpp
scn--;
...
if(scn > scanBuffer->Points) return(cp.println(0));
cp.println(scanBuffer->cvscan[scn]->Acquired);
```

Valid indices into `scanBuffer->cvscan[]` are `0 .. Points-1`. The guard only
rejects `scn > Points`, so `scn == Points` slips through and dereferences one
element past the end of the array (`cvscan[Points]`), reading whatever memory
follows the allocation.

**Fix:** change the comparison to `>=`.

### 3. `freeScanBuffer()` frees scalar allocations with array `delete[]`
**File:** [src/DMS.cpp](src/DMS.cpp) — `freeScanBuffer()`.

`scanBuffer` and each `scanBuffer->cvscan[i]` are allocated with scalar
placement-`new`-style calls (`new(std::nothrow) Scan`, `new(std::nothrow)
CVscan`), but freed with `delete[]`:

```cpp
delete[] scanBuffer->cvscan[i];   // cvscan[i] was `new CVscan`, not `new CVscan[n]`
...
delete[] scanBuffer;              // scanBuffer was `new Scan`, not `new Scan[n]`
```

Mismatched scalar-`new`/array-`delete` is undefined behavior per the C++
standard — the misleading comments above these lines ("we allocated as
unsigned char[]") don't match what the code actually does. It happens to work
on this runtime's allocator today, but should be corrected to plain `delete`
to match the scalar `new`.

### 4. Averaged scans don't stop on abort/failure mid-average
**File:** [src/DMS.cpp](src/DMS.cpp) — `performScan()`.

The first CV sweep for a given Vrf step checks the result and bails out:
```cpp
if(!performCVscan()) break;
```
but the inner averaging loop that repeats the sweep for `dms.Averages > 1`
does not:
```cpp
for(int i=2;i<=scanBuffer->Averages;i++)
{
  scanBuffer->cvscan[scanBuffer->Acquired]->Acquired = 0;
  performCVscan();                     // return value ignored
  for(int k=0; ...) dataPos[k] += scanBuffer->cvscan[...]->dataPos[k];
  ...
}
```
If a scan is aborted (`SCNSTP`) or fails partway through an averaging run,
the loop keeps summing whatever partial/stale data is left in the buffer
into the running total, and the final average silently includes bad data
instead of the scan being reported as failed/aborted.

**Suggested fix:** check `performCVscan()`'s return value here too and break
out of both loops (propagating the abort) the same way the primary call
does.

## Needs hardware verification (not changed)

### 5. `RED`/`GREEN` DotStar color macros may be swapped
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

### 6. `OFF` doesn't currently turn power off
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

## Build system

### 7. `src/FlashFS/` is a duplicated copy of `GAACE_Core`'s FatFs sources
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
