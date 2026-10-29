#pragma once

// Volvo P1 (S40 / V50 / C30) Haldex DEM support.
//
// Volvo DEMs (Differential Electronic Modules, the Haldex ECU bolted to the coupling) speak
// Volvo's 29-bit, 500 kbit/s HS-CAN matrix, not the VW PQ/MQB frames the rest of this firmware
// generates. The exact frame set a Gen 2 or Gen 3 DEM needs is not published, so this module is
// profile driven: a JSON file in LittleFS describes the periodic frames to generate, where the
// requested-lock value is injected, how alive counters and checksums are maintained, which DEM
// frame reports engagement, and optional decode signals for CAN View / Setup input mapping.
//
// Profiles live at /volvo/gen2.json and /volvo/gen3.json. See docs/volvo-p1.md.

#include <Arduino.h>
#include <ArduinoJson.h>
#include <driver/twai.h>

#include "functions/canview/dbc_common.h"

#define VOLVO_MAX_FRAMES 32
#define VOLVO_MAX_SIGNALS 48
#define VOLVO_MAX_LOCK_BYTES 4
#define VOLVO_NAME_LEN 32
#define VOLVO_UNIT_LEN 12
#define VOLVO_PROFILE_MAX_BYTES 16384

// Load the profile for the current Haldex generation. Call after storageLoad() (LittleFS mounted).
void volvoInit();

// Reload the on-disk profile for the current generation into the live generator.
bool volvoProfileReload();

// Validate a JSON profile, persist it for the current generation, and apply it live.
bool volvoProfileApplyJson(const String& body, String& error);

// Raw JSON text of the on-disk profile for the current generation.
bool volvoProfileReadRaw(String& out);

// Overwrite the current generation's profile with the documented template.
bool volvoProfileWriteTemplate();

String volvoProfilePath();

// Periodic generator. Call every 10 ms from the frame task while the controller is standalone.
void volvoGeneratorTick(uint32_t now_ms);

// Haldex-bus observer. Decodes the DEM status frame into the shared telemetry globals.
// Returns true when the frame was the configured status frame.
bool volvoObserveHaldexFrame(const twai_message_t& frame);

// Decode-only helper used by CAN View. Returns false when status decode is not configured
// or the frame does not match.
bool volvoDecodeStatusFrame(const twai_message_t& frame, uint16_t& engagement_raw, uint8_t& dem_state);
uint32_t volvoStatusFrameId();
bool volvoStatusFrameExtended();

// User-defined decode signals (profile "signals" array) in dbc_signal_t form.
const dbc_signal_t* volvoActiveSignals();
uint16_t volvoActiveSignalCount();

// Send a one-shot frame on the Haldex bus (bench probing).
bool volvoSendRawFrame(uint32_t id, bool extd, uint8_t dlc, const uint8_t* data);

void volvoWriteStatusJson(JsonObject& out, bool include_frames);
