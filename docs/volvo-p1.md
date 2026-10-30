# Volvo P1 (S40 / V50 / C30) Haldex DEM support

This document covers running OpenHaldex-S3 against a Volvo P1 Haldex coupling, including the
common build this support was written for: a **2011 Volvo C30** (factory front-wheel drive)
converted to AWD with the rear subframe, driveshaft, angle gear and Haldex coupling from a
**2005 Volvo S40 AWD**, using either the original **Gen 2 DEM** or a **Gen 3 DEM** fitted as an
upgrade.

It is written in two phases, in the order you should build them:

1. **Standalone controller.** The LilyGo T-2CAN is the DEM's only CAN partner. It generates every
   frame the DEM needs and requests lock from the web UI, with no connection to the car's bus.
2. **C30 CAN input.** The chassis channel taps the C30's HS-CAN in receive-only mode so speed,
   throttle, rpm and the DSTC button can drive Speed / Throttle / Map / RPM modes and mode triggers.

> **Status of the CAN data.** Volvo's HS-CAN matrix is not public and the frames a Volvo DEM
> requires are not published anywhere this project could verify. OpenHaldex-S3 therefore does
> **not** ship a ready-made Volvo frame set. It ships the plumbing: a profile format, a 29-bit
> generator with counters, checksums and lock-value injection, DEM status decode, decode signals
> for CAN View and Setup, and bench tools. You fill the profile from captures of a working P1 AWD
> car. Section 5 explains how.

---

## 1. What is different about a Volvo DEM

| Topic | VW / Audi (existing firmware) | Volvo P1 |
|---|---|---|
| Haldex ECU | Separate VW controller on Gen 1; integrated on Gen 2/4/5 | Integrated **DEM** (Differential Electronic Module) bolted to the coupling |
| Chassis bus | PQ (11-bit) or MQB (11-bit) at 500 kbit/s | Volvo HS-CAN, **29-bit identifiers**, 500 kbit/s |
| Frames the Haldex consumes | Motor/Brakes/LWS frames, known and shipped as DBC tables | Volvo BCM / ECM / CEM frames, **not shipped**; profile-driven |
| Haldex status frame | 0x2C0 (Gen 1/2/4), 0x118 (Gen 5) | DEM frame(s) on 29-bit IDs; configured in the profile |
| Diagnostics | UDS (Gen 5), KWP2000 over TP2.0 (Gen 2/4) | Volvo diagnostic protocol via the CEM; **not implemented** |
| Connector | `1J0-973-713 / 813` pair | Volvo DEM connector; forum reports describe four wires: battery, ground, CAN H, CAN L. Confirm against the S40 wiring diagram |
| Generations | 1, 2, 4, 5 (VAG never used Gen 3) | **2** (factory on P1) or **3** (upgrade) |

The controller hardware is unchanged. Both T-2CAN channels run 500 kbit/s, the Haldex side already
handles extended identifiers, and the chassis side can run in receive-only mode.

## 2. Gen 2 versus Gen 3 DEM on a P1

What the forums document, in short:

- P1 cars (S40/V50 AWD from 2004/2005) shipped with Haldex Gen 2.
- Gen 3 ("PreX" / Volvo "Instant Traction") adds an electric pre-charge pump so pressure is
  available before slip is detected. Volvo fitted it to larger cars (XC90, S80, V70/XC70) from
  roughly 2006-2008.
- The widely shared P1 upgrade swaps the **DEM module, the solenoid and the pressure sensor** from
  a Gen 3 unit (XC90-sourced parts are quoted) onto the P1 coupling. Part numbers quoted in the
  threads include `5WP33504-01 / -03` and `5WP22228-02 / -04` DEMs, and Volvo `36001160`.
- One group reports it as plug-and-play with no software. Others report the car working for a few
  drive cycles and then disabling AWD until the original module is refitted, and some state that a
  DEM needs its own software load. Treat "plug and play" as unverified.

Why this matters for OpenHaldex-S3: a Gen 3 DEM in standalone mode never talks to the Volvo CEM,
so VIDA software loads and vehicle-configuration mismatches cannot disable it. Whatever frames it
needs, the controller supplies. The Gen 2 and Gen 3 DEMs are therefore handled identically in the
firmware, as two **profiles**, and the only thing that differs is the content of those profiles,
which may well differ because the Gen 3 DEM is newer software.

Select **Volvo Gen 2** or **Volvo Gen 3** on the Setup page. The selection chooses the profile
file (`/volvo/gen2.json` or `/volvo/gen3.json`) and nothing else.

Sources used for this section:
[Swedespeed: "Real" AWD upgrade - Gen 3 Haldex](https://www.swedespeed.com/threads/real-awd-upgrade-improved-power-split-gen-3-haldex.558890/),
[Swedespeed: Anyone have success swapping Haldex Gen 2 to Gen 3?](https://www.swedespeed.com/threads/anyone-have-success-swap-haldex-gen-2-to-gen-3.664562/),
[Swedespeed: Does the V50/S40 AWD have Gen 2 or 3?](https://www.swedespeed.com/threads/does-the-v50-and-s40-awd-have-haldex-gen-2-or-3-help-or-info.559634/),
[Volvo Owners Club: Haldex upgrade, can't be this simple surely](https://www.volvoforums.org.uk/archive/index.php/t-281445.html),
[FCP Euro: Volvo AWD systems explained](https://blog.fcpeuro.com/volvo-awd-systems-explained),
[AutomotiveTechInfo: Volvo DEM and Haldex](https://automotivetechinfo.com/2018/04/volvo-dem-and-haldex).

## 3. Wiring

### 3.1 Standalone (phase 1)

```
            +12V (fused, ignition-switched) ----> T-2CAN 12-24V
            Ground --------------------------+--> T-2CAN GND
                                             |
  DEM connector:  B+ (constant, fused) ------|-------> DEM
                  Ground --------------------+-------> DEM
                  CAN H  <-------------------------->  T-2CAN CANHB   (Haldex channel)
                  CAN L  <-------------------------->  T-2CAN CANLB
                  120 ohm across CANHB/CANLB at the T-2CAN end
                  120 ohm across CAN H/L at the DEM end (if the DEM is not internally terminated)
```

- Use the Volvo DEM mating connector or a pigtail from the donor harness. The VW `1J0` connector
  pair in the main README does not apply.
- The controller is the whole "car" on this bus, so **termination is yours to provide**. The
  T-2CAN appears to ship without termination enabled; check the board schematic for pads.
- Power the T-2CAN from an ignition-switched feed. Leave the DEM on constant battery as the
  factory did: it sleeps when the bus goes quiet. Leave the Gen 5 parked-sleep feature **off**.
- Nothing connects to the chassis channel (CANHA/CANLA) in phase 1.

### 3.2 C30 input (phase 2)

Connect the C30 HS-CAN pair to **CANHA/CANLA** and enable **Chassis bus receive-only** on the
Volvo DEM page before plugging in. In receive-only mode the controller never acknowledges or
transmits on the car's bus, so it cannot upset the CEM, BCM or ECM. For logging, the HS-CAN pair
is available on OBD pins 6 and 14; for a permanent install splice at a convenient HS-CAN node per
the C30 wiring diagram. Do **not** add termination to the car's bus.

The DEM stays on its own isolated bus. This is deliberate:

- The C30 was never AWD, so its modules run front-drive software and will not send anything
  AWD-specific.
- A 2005 DEM expects the 2005 frame matrix. P1 had an electrical refresh around MY2008, so a
  MY2011 C30 may not speak the same dialect. Isolation removes that risk.
- The DEM cannot transmit into the C30 bus and the C30 cannot fault on it.

The trade-off is that VIDA cannot see the DEM. DEM fault reading would need Volvo's diagnostic
protocol, which is not implemented.

### 3.3 Two dialects, one controller

P1 had an electrical refresh around model year 2008. Cars from 2004-2007 and cars from 2008 on
share the physical HS-CAN but not the frame matrix, which is why modules do not swap across that
boundary. A 2005 DEM therefore needs the 2005 dialect, while a 2011 C30 speaks the 2008-on dialect.
The profile keeps them apart:

- `frames[]` is the DEM dialect. It comes from captures of the donor-era car and is generated
  toward the DEM.
- `signals[]` is the host-car dialect. It comes from captures of the C30 and is decoded from the
  chassis channel.
- `inject[]` entries inside a frame carry decoded host values across: the controller reads C30
  speed, pedal and rpm through Setup's mapped inputs and re-encodes them with the 2005 scaling
  into the DEM frames. See 5.2.

## 4. Firmware behaviour on the Volvo platform

| Area | Behaviour |
|---|---|
| Setup generation picker | `Volvo Gen 2`, `Volvo Gen 3` (platform `volvo_p1`). VW entries unchanged. |
| Standalone on | The profile generator runs: every enabled frame is sent on the Haldex channel at its period with lock bytes, counters and checksums maintained. Chassis frames are **not** forwarded. |
| Standalone off | Chassis frames are bridged to the DEM **unchanged**. There is no Volvo bridge-mutation strategy, so requested lock is forced to 0. Use this only to experiment with a full pass-through. |
| Chassis receive-only | Chassis TWAI runs in listen-only mode. Broadcast and bridge transmissions toward the car are suppressed. Applies at boot and live from the Volvo DEM page. |
| Lock request | Same pipeline as VW: Home ratio / Speed / Throttle / Map / RPM modes produce `lock_target`; `get_lock_target_adjusted_value(base, invert)` scales each profile lock byte; the learned calibration table applies once DEM status decode works. |
| DEM status | The profile `status` block names the DEM frame and bytes. Engagement feeds the Home page, Diagnostics and the Calibrate routine. |
| Decode signals | The profile `signals` array is the active DBC table: CAN View decodes them and Setup can map them to Speed / Throttle / RPM and to the mode trigger. |
| VW diagnostics | UDS / KWP endpoints return `unsupported_platform`. |
| Parked sleep | Not armed (Gen 5 only). |

Standalone with no chassis input has no speed, throttle or rpm. Fixed-lock modes (FWD, 50/50,
60/40, 70/30, 80/20, 90/10) and the Home ratio work; Speed / Throttle / Map / RPM modes wait for
phase 2 or for mapped signals.

## 5. Building the profile

### 5.1 What you need before writing a single frame

1. **Identify the DEM.** Read the label on the module; confirm Gen 2 or Gen 3 and the part number.
2. **Capture a working P1 AWD car.** Ideally the donor era (S40/V50 AWD 2005-2007), ideally at the
   DEM connector so you see exactly what the DEM receives and answers. The T-2CAN plus CAN View
   (raw table, 30 s dump, diagnostic capture) works; a USB CAN tool and SavvyCAN are easier for
   long logs. Capture ignition-on idle, pull-away, a slip event (wet grass / gravel launch) and
   ignition-off.
3. **Capture the C30's own HS-CAN** at the same time (OBD pins 6 and 14) for phase 2.
4. **Bench the DEM** with the T-2CAN only. Power it, start with the profile empty and watch CAN
   View: a DEM that transmits on power-up gives you the status frame ID for free.

### 5.2 Profile file format

One file per generation in LittleFS, edited on the Volvo DEM page (`/volvo.html`) or via
`GET/POST /api/volvo/profile`. Numbers may be written as JSON integers or hex strings.

```json
{
  "name": "Volvo P1 Gen 2 DEM",
  "generation": 2,
  "status": {
    "enabled": true,
    "id": "0x01234567",
    "extd": true,
    "engagementByte": 1,
    "engagementMin": 0,
    "engagementMax": 255,
    "stateByte": 0
  },
  "frames": [
    {
      "name": "BCM wheel speeds",
      "enabled": true,
      "id": "0x0123ABCD",
      "extd": true,
      "dlc": 8,
      "periodMs": 20,
      "data": ["0x00", "0x00", "0x00", "0x00", "0x00", "0x00", "0x00", "0x00"],
      "counter": { "enabled": true, "byte": 7, "mask": "0x0F", "start": 0, "step": 1 },
      "lock": [{ "byte": 4, "base": "0xFE", "invert": false }],
      "checksum": { "type": "none", "byte": 7, "from": 0, "to": 6 }
    }
  ],
  "signals": [
    {
      "enabled": true,
      "name": "Vehicle speed",
      "id": "0x0123ABCD",
      "extd": true,
      "startBit": 8,
      "length": 16,
      "littleEndian": true,
      "signed": false,
      "factor": 0.01,
      "offset": 0,
      "unit": "km/h"
    }
  ]
}
```

Field notes:

- `frames[].period_ms` is 10..5000; the generator ticks every 10 ms. Up to 32 frames.
- `data` is the static payload. Build order per send is `data`, then `inject`, then `lock`, then
  `counter`, then `checksum`, so a `lock` byte overrides an injected field that shares it.
- `inject[]` re-encodes a live controller value into a bit field: `source` is `speed` (km/h),
  `throttle` (%), `rpm`, `lock` (requested %), `engagement` (DEM-reported %) or `const` (uses
  `value`); `startBit`, `length`, `littleEndian` and `signed` describe the field exactly as a
  `signals[]` entry would; `factor` and `offset` are the DEM-dialect scaling (raw = (value -
  offset) / factor); optional `min`/`max` clamp in engineering units; `lockGain` multiplies the
  value by `1 + lockGain * lock / 100`. Put `lockGain` on the front wheel speeds (or `-lockGain`
  on the rears) to show the DEM slip proportional to the requested lock while still following
  real vehicle speed. Up to 6 per frame.
- `lock[]` entries overwrite a byte on every send with
  `get_lock_target_adjusted_value(base, invert)`: at 0 % requested lock the byte becomes `0x00`
  (`0xFE` when `invert` is true), at 100 % it becomes `base`, in between it follows the learned
  calibration table. Use it exactly where the VW strategy fakes slip or torque: a front wheel
  speed that exceeds the rear, an engine torque or pedal value. Up to 4 per frame.
- `counter` keeps an alive counter in the bits of `mask` within one byte; the rest of the byte is
  preserved. `start` and `step` are in counter units.
- `checksum` is computed last over bytes `from..to` and written to `byte`. Types: `none`, `xor`,
  `sum` (low byte of the sum), `sum_inv` (`0xFF - sum`, the VW steering-angle style),
  `crc8_autosar` (poly 0x2F, init 0xFF, xor-out 0xFF).
- `status` names the DEM frame that reports engagement. `engagementMin/Max` map the raw byte to
  0..100 %. `stateByte` is optional and is shown raw.
- `signals[]` are plain DBC-style definitions (start bit, length, endianness, factor, offset).
  Up to 48. They appear in CAN View and in the Setup signal picker as `chassis|0x…|name|unit`.

Saving validates on the device, writes the file and applies it live. Alive counters of frames
that keep the same ID carry on across a save.

### 5.3 Capturing from a donor car with the driveshaft disconnected

A donor car that drives with the propshaft removed from the coupling is still a good capture
source. Everything the DEM receives is real, so the key-cycle, idle, driving and sleep captures
are unaffected, and on a wet or gravel launch the DEM still sees front-wheel slip and commands
lock. Two caveats:

- A Gen 2 coupling builds its working pressure from the speed difference across the input shaft.
  With the shaft out, commanded lock produces no pressure and the DEM may log a pressure or pump
  fault. A DEM in fault mode can change what it transmits, so clear DEM codes in VIDA before the
  launch run and note whether a fault appeared during it.
- Any "actual engagement" byte in the status frame will read as not engaged in those captures. Use
  them to find the command and status bytes; confirm the engagement scaling later on the bench
  with your own coupling.

### 5.4 Bench procedure

1. Setup: pick `Volvo Gen 2` or `Volvo Gen 3`, Save.
2. Volvo DEM page: tick **Standalone generator**, Apply. Reset to template, then paste your
   capture-derived frames with `enabled: false`.
3. Enable the network-management / keep-alive style frames first. Watch the status pane: once the
   DEM starts answering, fill in `status`.
4. Enable the input frames (wheel speeds, engine, brake/DSTC flags, steering, gear) with neutral
   values. The DEM should sit happily with no fault indication.
5. Add `lock` entries to the slip and torque bytes, put the Home page in 50/50, and watch
   engagement rise. Use the single-frame sender on the Volvo DEM page to probe individual bytes
   before committing them to the profile.
6. Run **Calibrate** on the Setup page so requested and actual lock line up.
7. Keep a copy of the profile (Download button). It is the valuable part of this build.

## 6. Phase 2: C30 input

1. Add the C30 frames you identified (wheel speed, pedal, rpm, brake, DSTC button, reverse) to
   `signals[]` with `enabled: true`.
2. Wire CANHA/CANLA to the C30 HS-CAN, tick **Chassis bus receive-only**, Apply.
3. Setup: map Speed, Throttle and RPM to the new signals and pick the DSTC button signal as the
   mode trigger. Speed / Throttle / Map / RPM modes and the trigger now work exactly as on VW.
4. Add `inject` entries to the DEM frames so the DEM sees the real C30 speed, pedal and rpm in
   its own dialect, with `lockGain` on the slip-relevant fields. The DEM then behaves as if it
   were in a moving 2005 car instead of a stationary one with the lock bytes moving on their own.
   A full bridge mode is not planned: the two dialects differ by design.

## 7. Limitations and open items

- No Volvo frame definitions ship with the firmware; everything depends on your captures.
- No Volvo DEM diagnostics (DTC read/clear, measured values). VIDA cannot reach an isolated DEM.
- Injected values are limited to the controller's mapped inputs (speed, throttle, rpm) plus lock
  and engagement. Brake, reverse, handbrake and DSTC flags from the C30 are not relayed yet; carry
  them as profile constants or as a mode trigger until a flag-injection path exists.
- `Volvo Gen 3` in Setup means "P1 coupling with a Gen 3 DEM fitted". It is not a Gen 3 strategy
  for VW cars; VAG never used Gen 3.
