#include "functions/volvo/volvo_p1.h"

#include <LittleFS.h>
#include <string.h>

#include "functions/can/can.h"
#include "functions/can/standalone_can.h"
#include "functions/config/config.h"
#include "functions/core/calcs.h"
#include "functions/core/state.h"
#include "functions/storage/filelog.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ---------------------------------------------------------------------------------------------
// Profile model
// ---------------------------------------------------------------------------------------------

struct volvo_lock_byte_t {
  uint8_t byte;
  uint8_t base;
  bool invert;
};

struct volvo_counter_t {
  bool enabled;
  uint8_t byte;
  uint8_t mask;
  uint8_t start;
  uint8_t step;
};

enum volvo_checksum_type_t : uint8_t {
  VOLVO_CK_NONE = 0,
  VOLVO_CK_XOR,
  VOLVO_CK_SUM,
  VOLVO_CK_SUM_INV,
  VOLVO_CK_CRC8_AUTOSAR,
};

struct volvo_checksum_t {
  volvo_checksum_type_t type;
  uint8_t byte;
  uint8_t from;
  uint8_t to;
};

// Live-value injection: re-encode a mapped controller input into a bit field of a generated frame.
// This is how real host-vehicle data (for example a 2011 C30 bus) is translated into the dialect
// the DEM expects (for example the 2005 S40 matrix).
enum volvo_inject_source_t : uint8_t {
  VOLVO_SRC_SPEED = 0,  // received_vehicle_speed, km/h
  VOLVO_SRC_THROTTLE,   // received_pedal_value, %
  VOLVO_SRC_RPM,        // received_vehicle_rpm
  VOLVO_SRC_LOCK,       // lock_target, % requested lock
  VOLVO_SRC_ENGAGEMENT, // received_haldex_engagement, % reported by the DEM
  VOLVO_SRC_CONST,      // constant engineering value (useful for fixed plausible inputs)
};

struct volvo_inject_t {
  volvo_inject_source_t source;
  uint16_t start_bit;
  uint8_t length;
  bool little_endian;
  bool is_signed;
  float factor;
  float offset;
  float lock_gain; // value *= 1 + lock_gain * lock_target / 100 (fake slip proportional to request)
  float value;     // VOLVO_SRC_CONST only
  float min;       // engineering-unit clamp, ignored when max <= min
  float max;
};

#define VOLVO_MAX_INJECT 6

struct volvo_frame_t {
  char name[VOLVO_NAME_LEN];
  bool enabled;
  uint32_t id;
  bool extd;
  uint8_t dlc;
  uint16_t period_ms;
  uint8_t data[8];
  volvo_counter_t counter;
  uint8_t lock_count;
  volvo_lock_byte_t lock[VOLVO_MAX_LOCK_BYTES];
  uint8_t inject_count;
  volvo_inject_t inject[VOLVO_MAX_INJECT];
  volvo_checksum_t checksum;
  // runtime
  uint32_t last_tx_ms;
  uint32_t tx_count;
  uint8_t counter_value;
};

struct volvo_status_cfg_t {
  bool enabled;
  uint32_t id;
  bool extd;
  uint8_t engagement_byte;
  uint16_t engagement_min;
  uint16_t engagement_max;
  int8_t state_byte; // -1 = none
};

struct volvo_profile_t {
  bool loaded;
  char name[48];
  uint8_t generation;
  uint8_t frame_count;
  volvo_frame_t frames[VOLVO_MAX_FRAMES];
  volvo_status_cfg_t status;
};

static volvo_profile_t g_profile = {};
static SemaphoreHandle_t g_profile_mutex = nullptr;

// Decode signal table is kept contiguous because consumers iterate it as dbc_signal_t[].
static dbc_signal_t g_signal_table[VOLVO_MAX_SIGNALS] = {};
static char g_signal_names[VOLVO_MAX_SIGNALS][VOLVO_NAME_LEN] = {};
static char g_signal_units[VOLVO_MAX_SIGNALS][VOLVO_UNIT_LEN] = {};
static volatile uint16_t g_signal_count = 0;

static String g_last_error = "";
static uint32_t g_last_status_ms = 0;
static uint32_t g_status_count = 0;
static uint32_t g_raw_tx_count = 0;
static uint32_t g_loaded_ms = 0;

static const char* k_profile_dir = "/volvo";

// ---------------------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------------------

static bool lockProfile(uint32_t timeout_ms) {
  if (!g_profile_mutex) {
    return true;
  }
  return xSemaphoreTake(g_profile_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

static void unlockProfile() {
  if (g_profile_mutex) {
    xSemaphoreGive(g_profile_mutex);
  }
}

static void copyName(char* dst, size_t dst_len, const char* src) {
  if (!dst || dst_len == 0) {
    return;
  }
  if (!src) {
    dst[0] = '\0';
    return;
  }
  strncpy(dst, src, dst_len - 1);
  dst[dst_len - 1] = '\0';
}

// Accepts integers or hex/decimal strings ("0x1A", "26").
static bool jsonToU32(JsonVariantConst v, uint32_t& out) {
  if (v.isNull()) {
    return false;
  }
  if (v.is<const char*>()) {
    const char* text = v.as<const char*>();
    if (!text || !*text) {
      return false;
    }
    char* end = nullptr;
    unsigned long parsed = strtoul(text, &end, 0);
    if (end == text || (end && *end != '\0')) {
      return false;
    }
    out = (uint32_t)parsed;
    return true;
  }
  if (v.is<float>() && !v.is<int>()) {
    float f = v.as<float>();
    if (f < 0.0f) {
      return false;
    }
    out = (uint32_t)f;
    return true;
  }
  long long n = v.as<long long>();
  if (n < 0) {
    return false;
  }
  out = (uint32_t)n;
  return true;
}

static bool jsonBoolOr(JsonVariantConst v, bool fallback) {
  if (v.isNull()) {
    return fallback;
  }
  return v.as<bool>();
}

static uint32_t jsonU32Or(JsonVariantConst v, uint32_t fallback) {
  uint32_t out = 0;
  return jsonToU32(v, out) ? out : fallback;
}

static float jsonFloatOr(JsonVariantConst v, float fallback) {
  if (v.isNull()) {
    return fallback;
  }
  return v.as<float>();
}

static const char* jsonStrOr(JsonVariantConst v, const char* fallback) {
  if (v.isNull() || !v.is<const char*>()) {
    return fallback;
  }
  const char* text = v.as<const char*>();
  return text ? text : fallback;
}

static bool jsonToU8(JsonVariantConst v, uint8_t& out, uint8_t fallback) {
  uint32_t tmp = 0;
  if (!jsonToU32(v, tmp)) {
    out = fallback;
    return v.isNull();
  }
  if (tmp > 0xFF) {
    return false;
  }
  out = (uint8_t)tmp;
  return true;
}

static String hexByte(uint8_t value) {
  char buf[5];
  snprintf(buf, sizeof(buf), "0x%02X", value);
  return String(buf);
}

static String hexId(uint32_t id, bool extd) {
  char buf[16];
  if (extd) {
    snprintf(buf, sizeof(buf), "0x%08lX", (unsigned long)(id & 0x1FFFFFFF));
  } else {
    snprintf(buf, sizeof(buf), "0x%03lX", (unsigned long)(id & 0x7FF));
  }
  return String(buf);
}

static bool parseChecksumType(const char* text, volvo_checksum_type_t& out) {
  String token = String(text ? text : "none");
  token.trim();
  token.toLowerCase();
  if (token == "" || token == "none") {
    out = VOLVO_CK_NONE;
  } else if (token == "xor") {
    out = VOLVO_CK_XOR;
  } else if (token == "sum") {
    out = VOLVO_CK_SUM;
  } else if (token == "sum_inv" || token == "suminv" || token == "255-sum") {
    out = VOLVO_CK_SUM_INV;
  } else if (token == "crc8_autosar" || token == "crc8") {
    out = VOLVO_CK_CRC8_AUTOSAR;
  } else {
    return false;
  }
  return true;
}

static const char* checksumTypeName(volvo_checksum_type_t type) {
  switch (type) {
  case VOLVO_CK_XOR:
    return "xor";
  case VOLVO_CK_SUM:
    return "sum";
  case VOLVO_CK_SUM_INV:
    return "sum_inv";
  case VOLVO_CK_CRC8_AUTOSAR:
    return "crc8_autosar";
  default:
    return "none";
  }
}

static bool parseInjectSource(const char* text, volvo_inject_source_t& out) {
  String token = String(text ? text : "");
  token.trim();
  token.toLowerCase();
  if (token == "speed") {
    out = VOLVO_SRC_SPEED;
  } else if (token == "throttle" || token == "pedal") {
    out = VOLVO_SRC_THROTTLE;
  } else if (token == "rpm") {
    out = VOLVO_SRC_RPM;
  } else if (token == "lock") {
    out = VOLVO_SRC_LOCK;
  } else if (token == "engagement") {
    out = VOLVO_SRC_ENGAGEMENT;
  } else if (token == "const" || token == "constant") {
    out = VOLVO_SRC_CONST;
  } else {
    return false;
  }
  return true;
}

static const char* injectSourceName(volvo_inject_source_t source) {
  switch (source) {
  case VOLVO_SRC_SPEED:
    return "speed";
  case VOLVO_SRC_THROTTLE:
    return "throttle";
  case VOLVO_SRC_RPM:
    return "rpm";
  case VOLVO_SRC_LOCK:
    return "lock";
  case VOLVO_SRC_ENGAGEMENT:
    return "engagement";
  default:
    return "const";
  }
}

// Inverse of dbc_extract_raw: write `length` bits of `raw` starting at `start_bit`.
// Big-endian follows the same sequential bit walk dbc_extract_raw uses, so a field that decodes
// with the profile's signal definition packs back identically.
static void packRaw(uint8_t* data, uint16_t start_bit, uint8_t length, bool little_endian, uint64_t raw) {
  if (length == 0 || length > 64) {
    return;
  }
  if (little_endian) {
    for (uint8_t i = 0; i < length; i++) {
      const uint16_t pos = (uint16_t)(start_bit + i);
      if (pos >= 64) {
        break;
      }
      const uint8_t bit = (uint8_t)((raw >> i) & 0x01);
      if (bit) {
        data[pos / 8] |= (uint8_t)(1U << (pos % 8));
      } else {
        data[pos / 8] &= (uint8_t)~(1U << (pos % 8));
      }
    }
    return;
  }
  for (uint8_t i = 0; i < length; i++) {
    const int bit_index = (int)start_bit - (int)i;
    if (bit_index < 0 || bit_index >= 64) {
      break;
    }
    const int byte_index = bit_index / 8;
    const int bit_in_byte = 7 - (bit_index % 8);
    const uint8_t bit = (uint8_t)((raw >> (length - 1 - i)) & 0x01);
    if (bit) {
      data[byte_index] |= (uint8_t)(1U << bit_in_byte);
    } else {
      data[byte_index] &= (uint8_t)~(1U << bit_in_byte);
    }
  }
}

static float injectSourceValue(const volvo_inject_t& inj) {
  switch (inj.source) {
  case VOLVO_SRC_SPEED:
    return (float)received_vehicle_speed;
  case VOLVO_SRC_THROTTLE:
    return received_pedal_value;
  case VOLVO_SRC_RPM:
    return (float)received_vehicle_rpm;
  case VOLVO_SRC_LOCK:
    return lock_target;
  case VOLVO_SRC_ENGAGEMENT:
    return (float)received_haldex_engagement;
  default:
    return inj.value;
  }
}

static void applyInject(const volvo_inject_t& inj, uint8_t* data) {
  float value = injectSourceValue(inj);
  if (inj.lock_gain != 0.0f) {
    float lock = lock_target;
    if (lock < 0.0f) {
      lock = 0.0f;
    }
    if (lock > 100.0f) {
      lock = 100.0f;
    }
    value *= (1.0f + inj.lock_gain * lock / 100.0f);
  }
  if (inj.max > inj.min) {
    if (value < inj.min) {
      value = inj.min;
    }
    if (value > inj.max) {
      value = inj.max;
    }
  }
  const float factor = (inj.factor == 0.0f) ? 1.0f : inj.factor;
  float raw_f = (value - inj.offset) / factor;
  raw_f = (raw_f >= 0.0f) ? (raw_f + 0.5f) : (raw_f - 0.5f);
  const uint64_t field_mask = (inj.length >= 64) ? ~0ULL : ((1ULL << inj.length) - 1ULL);
  uint64_t raw = 0;
  if (inj.is_signed) {
    const int64_t max_v = (int64_t)(field_mask >> 1);
    const int64_t min_v = -max_v - 1;
    int64_t sv = (int64_t)raw_f;
    if (sv > max_v) {
      sv = max_v;
    }
    if (sv < min_v) {
      sv = min_v;
    }
    raw = ((uint64_t)sv) & field_mask;
  } else {
    if (raw_f < 0.0f) {
      raw_f = 0.0f;
    }
    raw = (uint64_t)raw_f;
    if (raw > field_mask) {
      raw = field_mask;
    }
  }
  packRaw(data, inj.start_bit, inj.length, inj.little_endian, raw);
}

static uint8_t counterShift(uint8_t mask) {
  if (mask == 0) {
    return 0;
  }
  uint8_t shift = 0;
  while (((mask >> shift) & 0x01) == 0 && shift < 8) {
    shift++;
  }
  return shift;
}

String volvoProfilePath() {
  String path = String(k_profile_dir) + "/gen" + String((int)haldexGeneration) + ".json";
  return path;
}

// ---------------------------------------------------------------------------------------------
// Parsing
// ---------------------------------------------------------------------------------------------

static bool parseFrame(JsonObjectConst obj, volvo_frame_t& frame, uint8_t index, String& error) {
  frame = volvo_frame_t{};
  copyName(frame.name, sizeof(frame.name), jsonStrOr(obj["name"], ""));
  if (frame.name[0] == '\0') {
    snprintf(frame.name, sizeof(frame.name), "frame%u", (unsigned)index);
  }
  frame.enabled = jsonBoolOr(obj["enabled"], false);
  frame.extd = jsonBoolOr(obj["extd"], true);

  uint32_t id = 0;
  if (!jsonToU32(obj["id"], id)) {
    error = String("frames[") + index + "]: missing or invalid id";
    return false;
  }
  if (frame.extd && id > 0x1FFFFFFF) {
    error = String("frames[") + index + "]: 29-bit id out of range";
    return false;
  }
  if (!frame.extd && id > 0x7FF) {
    error = String("frames[") + index + "]: 11-bit id out of range (set extd true for 29-bit ids)";
    return false;
  }
  frame.id = id;

  const uint32_t dlc = jsonU32Or(obj["dlc"], 8);
  if (dlc > 8) {
    error = String("frames[") + index + "]: dlc must be 0..8";
    return false;
  }
  frame.dlc = (uint8_t)dlc;

  const uint32_t period = jsonU32Or(obj["periodMs"], 100);
  if (period < 10 || period > 5000) {
    error = String("frames[") + index + "]: periodMs must be 10..5000";
    return false;
  }
  frame.period_ms = (uint16_t)period;

  JsonArrayConst data = obj["data"].as<JsonArrayConst>();
  if (!data.isNull()) {
    uint8_t i = 0;
    for (JsonVariantConst v : data) {
      if (i >= 8) {
        break;
      }
      uint8_t byte = 0;
      uint32_t tmp = 0;
      if (!jsonToU32(v, tmp) || tmp > 0xFF) {
        error = String("frames[") + index + "]: data[" + i + "] is not a byte";
        return false;
      }
      byte = (uint8_t)tmp;
      frame.data[i++] = byte;
    }
  }

  JsonObjectConst counter = obj["counter"].as<JsonObjectConst>();
  if (!counter.isNull()) {
    frame.counter.enabled = jsonBoolOr(counter["enabled"], true);
    if (!jsonToU8(counter["byte"], frame.counter.byte, 7) || frame.counter.byte > 7) {
      error = String("frames[") + index + "]: counter.byte must be 0..7";
      return false;
    }
    if (!jsonToU8(counter["mask"], frame.counter.mask, 0x0F)) {
      error = String("frames[") + index + "]: counter.mask is not a byte";
      return false;
    }
    if (frame.counter.mask == 0) {
      frame.counter.mask = 0x0F;
    }
    if (!jsonToU8(counter["start"], frame.counter.start, 0)) {
      error = String("frames[") + index + "]: counter.start is not a byte";
      return false;
    }
    if (!jsonToU8(counter["step"], frame.counter.step, 1)) {
      error = String("frames[") + index + "]: counter.step is not a byte";
      return false;
    }
    if (frame.counter.step == 0) {
      frame.counter.step = 1;
    }
    frame.counter_value = frame.counter.start;
  }

  JsonArrayConst lock = obj["lock"].as<JsonArrayConst>();
  if (!lock.isNull()) {
    for (JsonObjectConst entry : lock) {
      if (frame.lock_count >= VOLVO_MAX_LOCK_BYTES) {
        error = String("frames[") + index + "]: too many lock bytes (max " + VOLVO_MAX_LOCK_BYTES + ")";
        return false;
      }
      volvo_lock_byte_t& lb = frame.lock[frame.lock_count];
      if (!jsonToU8(entry["byte"], lb.byte, 0) || lb.byte > 7) {
        error = String("frames[") + index + "]: lock.byte must be 0..7";
        return false;
      }
      if (!jsonToU8(entry["base"], lb.base, 0xFE)) {
        error = String("frames[") + index + "]: lock.base is not a byte";
        return false;
      }
      lb.invert = jsonBoolOr(entry["invert"], false);
      frame.lock_count++;
    }
  }

  JsonArrayConst inject = obj["inject"].as<JsonArrayConst>();
  if (!inject.isNull()) {
    for (JsonObjectConst entry : inject) {
      if (frame.inject_count >= VOLVO_MAX_INJECT) {
        error = String("frames[") + index + "]: too many inject entries (max " + VOLVO_MAX_INJECT + ")";
        return false;
      }
      volvo_inject_t& inj = frame.inject[frame.inject_count];
      inj = volvo_inject_t{};
      if (!parseInjectSource(jsonStrOr(entry["source"], ""), inj.source)) {
        error = String("frames[") + index + "]: inject.source must be speed|throttle|rpm|lock|engagement|const";
        return false;
      }
      const uint32_t start_bit = jsonU32Or(entry["startBit"], 0);
      const uint32_t length = jsonU32Or(entry["length"], 8);
      if (start_bit > 63 || length == 0 || length > 32) {
        error = String("frames[") + index + "]: inject.startBit must be 0..63 and length 1..32";
        return false;
      }
      inj.start_bit = (uint16_t)start_bit;
      inj.length = (uint8_t)length;
      inj.little_endian = jsonBoolOr(entry["littleEndian"], true);
      inj.is_signed = jsonBoolOr(entry["signed"], false);
      inj.factor = jsonFloatOr(entry["factor"], 1.0f);
      inj.offset = jsonFloatOr(entry["offset"], 0.0f);
      inj.lock_gain = jsonFloatOr(entry["lockGain"], 0.0f);
      inj.value = jsonFloatOr(entry["value"], 0.0f);
      inj.min = jsonFloatOr(entry["min"], 0.0f);
      inj.max = jsonFloatOr(entry["max"], 0.0f);
      frame.inject_count++;
    }
  }

  JsonObjectConst checksum = obj["checksum"].as<JsonObjectConst>();
  frame.checksum.type = VOLVO_CK_NONE;
  if (!checksum.isNull()) {
    if (!parseChecksumType(jsonStrOr(checksum["type"], "none"), frame.checksum.type)) {
      error = String("frames[") + index + "]: checksum.type must be none|xor|sum|sum_inv|crc8_autosar";
      return false;
    }
    if (!jsonToU8(checksum["byte"], frame.checksum.byte, 7) || frame.checksum.byte > 7) {
      error = String("frames[") + index + "]: checksum.byte must be 0..7";
      return false;
    }
    if (!jsonToU8(checksum["from"], frame.checksum.from, 0) || frame.checksum.from > 7) {
      error = String("frames[") + index + "]: checksum.from must be 0..7";
      return false;
    }
    if (!jsonToU8(checksum["to"], frame.checksum.to, 6) || frame.checksum.to > 7) {
      error = String("frames[") + index + "]: checksum.to must be 0..7";
      return false;
    }
    if (frame.checksum.to < frame.checksum.from) {
      error = String("frames[") + index + "]: checksum.to must be >= checksum.from";
      return false;
    }
  }
  return true;
}

static bool parseStatus(JsonObjectConst obj, volvo_status_cfg_t& status, String& error) {
  memset(&status, 0, sizeof(status));
  status.state_byte = -1;
  status.engagement_max = 255;
  if (obj.isNull()) {
    return true;
  }
  status.enabled = jsonBoolOr(obj["enabled"], false);
  status.extd = jsonBoolOr(obj["extd"], true);
  uint32_t id = 0;
  if (!jsonToU32(obj["id"], id)) {
    if (status.enabled) {
      error = "status: missing or invalid id";
      return false;
    }
    id = 0;
  }
  status.id = id;
  if (!jsonToU8(obj["engagementByte"], status.engagement_byte, 1) || status.engagement_byte > 7) {
    error = "status.engagementByte must be 0..7";
    return false;
  }
  uint32_t tmp = 0;
  status.engagement_min = jsonToU32(obj["engagementMin"], tmp) ? (uint16_t)tmp : 0;
  status.engagement_max = jsonToU32(obj["engagementMax"], tmp) ? (uint16_t)tmp : 255;
  if (status.engagement_max <= status.engagement_min) {
    error = "status.engagementMax must be greater than engagementMin";
    return false;
  }
  JsonVariantConst state_byte = obj["stateByte"];
  if (!state_byte.isNull()) {
    uint8_t sb = 0;
    if (!jsonToU8(state_byte, sb, 0) || sb > 7) {
      error = "status.stateByte must be 0..7";
      return false;
    }
    status.state_byte = (int8_t)sb;
  }
  return true;
}

static bool parseSignals(JsonArrayConst arr, dbc_signal_t* table, char (*names)[VOLVO_NAME_LEN],
                         char (*units)[VOLVO_UNIT_LEN], uint16_t& count, String& error) {
  count = 0;
  if (arr.isNull()) {
    return true;
  }
  uint16_t index = 0;
  for (JsonObjectConst obj : arr) {
    const bool enabled = jsonBoolOr(obj["enabled"], true);
    index++;
    if (!enabled) {
      continue;
    }
    if (count >= VOLVO_MAX_SIGNALS) {
      error = String("signals: too many entries (max ") + VOLVO_MAX_SIGNALS + ")";
      return false;
    }
    dbc_signal_t& sig = table[count];
    sig = dbc_signal_t{};
    uint32_t id = 0;
    if (!jsonToU32(obj["id"], id)) {
      error = String("signals[") + (index - 1) + "]: missing or invalid id";
      return false;
    }
    sig.id = id & 0x1FFFFFFF;
    copyName(names[count], VOLVO_NAME_LEN, jsonStrOr(obj["name"], ""));
    if (names[count][0] == '\0') {
      snprintf(names[count], VOLVO_NAME_LEN, "signal%u", (unsigned)count);
    }
    copyName(units[count], VOLVO_UNIT_LEN, jsonStrOr(obj["unit"], ""));
    sig.name = names[count];
    sig.unit = units[count];
    const uint32_t start_bit = jsonU32Or(obj["startBit"], 0);
    const uint32_t length = jsonU32Or(obj["length"], 8);
    if (start_bit > 63 || length == 0 || length > 64) {
      error = String("signals[") + (index - 1) + "]: startBit must be 0..63 and length 1..64";
      return false;
    }
    sig.start_bit = (uint16_t)start_bit;
    sig.length = (uint8_t)length;
    sig.is_little_endian = jsonBoolOr(obj["littleEndian"], true) ? 1 : 0;
    sig.is_signed = jsonBoolOr(obj["signed"], false) ? 1 : 0;
    sig.factor = jsonFloatOr(obj["factor"], 1.0f);
    sig.offset = jsonFloatOr(obj["offset"], 0.0f);
    sig.min = jsonFloatOr(obj["min"], 0.0f);
    sig.max = jsonFloatOr(obj["max"], 0.0f);
    sig.mux = -1;
    count++;
  }
  return true;
}

// Parses into staging storage and swaps in on success.
static bool applyProfileDocument(JsonDocument& doc, String& error) {
  static volvo_profile_t staging;
  static dbc_signal_t staging_signals[VOLVO_MAX_SIGNALS];
  static char staging_names[VOLVO_MAX_SIGNALS][VOLVO_NAME_LEN];
  static char staging_units[VOLVO_MAX_SIGNALS][VOLVO_UNIT_LEN];
  uint16_t staging_signal_count = 0;

  staging = volvo_profile_t{};
  copyName(staging.name, sizeof(staging.name), jsonStrOr(doc["name"], "Volvo P1 profile"));
  const uint32_t generation = jsonU32Or(doc["generation"], haldexGeneration);
  staging.generation = (uint8_t)generation;

  JsonArrayConst frames = doc["frames"].as<JsonArrayConst>();
  if (!frames.isNull()) {
    uint8_t index = 0;
    for (JsonObjectConst obj : frames) {
      if (staging.frame_count >= VOLVO_MAX_FRAMES) {
        error = String("frames: too many entries (max ") + VOLVO_MAX_FRAMES + ")";
        return false;
      }
      if (!parseFrame(obj, staging.frames[staging.frame_count], index, error)) {
        return false;
      }
      staging.frame_count++;
      index++;
    }
  }

  if (!parseStatus(doc["status"].as<JsonObjectConst>(), staging.status, error)) {
    return false;
  }

  if (!parseSignals(doc["signals"].as<JsonArrayConst>(), staging_signals, staging_names, staging_units,
                    staging_signal_count, error)) {
    return false;
  }

  if (!lockProfile(200)) {
    error = "profile busy";
    return false;
  }
  // Preserve runtime counters for frames that keep the same id so a profile edit does not
  // restart alive counters mid-stream.
  for (uint8_t i = 0; i < staging.frame_count; i++) {
    for (uint8_t j = 0; j < g_profile.frame_count; j++) {
      if (g_profile.frames[j].id == staging.frames[i].id && g_profile.frames[j].extd == staging.frames[i].extd) {
        staging.frames[i].tx_count = g_profile.frames[j].tx_count;
        staging.frames[i].counter_value = g_profile.frames[j].counter_value;
        break;
      }
    }
  }
  staging.loaded = true;
  memcpy(&g_profile, &staging, sizeof(g_profile));

  g_signal_count = 0;
  memcpy(g_signal_names, staging_names, sizeof(g_signal_names));
  memcpy(g_signal_units, staging_units, sizeof(g_signal_units));
  memcpy(g_signal_table, staging_signals, sizeof(g_signal_table));
  for (uint16_t i = 0; i < VOLVO_MAX_SIGNALS; i++) {
    g_signal_table[i].name = g_signal_names[i];
    g_signal_table[i].unit = g_signal_units[i];
  }
  g_signal_count = staging_signal_count;
  g_loaded_ms = millis();
  unlockProfile();
  g_last_error = "";
  return true;
}

// ---------------------------------------------------------------------------------------------
// Template
// ---------------------------------------------------------------------------------------------

static String buildTemplateJson() {
  JsonDocument doc;
  doc["name"] = String("Volvo P1 Gen ") + (int)haldexGeneration + " DEM profile (template)";
  doc["generation"] = haldexGeneration;
  JsonArray notes = doc["notes"].to<JsonArray>();
  notes.add("Fill this profile from an HS-CAN capture of a running P1 AWD car. See docs/volvo-p1.md.");
  notes.add("Volvo HS-CAN uses 29-bit identifiers at 500 kbit/s: keep extd=true and write ids as 0x hex.");
  notes.add("Every frame starts disabled. Enable frames one at a time on the bench and watch the DEM.");
  notes.add("lock entries replace a byte with get_lock_target_adjusted_value(base, invert) each send.");
  notes.add("inject entries re-encode a mapped input (speed/throttle/rpm/lock/engagement/const) into a bit field; lockGain scales it with requested lock to fake slip.");
  notes.add("Build order per send: data -> inject -> lock -> counter -> checksum.");
  notes.add("counter writes an alive counter into the masked bits of one byte; checksum is computed last.");

  JsonObject status = doc["status"].to<JsonObject>();
  status["enabled"] = false;
  status["id"] = "0x00000000";
  status["extd"] = true;
  status["engagementByte"] = 1;
  status["engagementMin"] = 0;
  status["engagementMax"] = 255;
  status["stateByte"] = 0;

  JsonArray frames = doc["frames"].to<JsonArray>();
  JsonObject f = frames.add<JsonObject>();
  f["name"] = "example (disabled)";
  f["enabled"] = false;
  f["id"] = "0x00000000";
  f["extd"] = true;
  f["dlc"] = 8;
  f["periodMs"] = 100;
  JsonArray data = f["data"].to<JsonArray>();
  for (uint8_t i = 0; i < 8; i++) {
    data.add("0x00");
  }
  JsonObject counter = f["counter"].to<JsonObject>();
  counter["enabled"] = false;
  counter["byte"] = 7;
  counter["mask"] = "0x0F";
  counter["start"] = 0;
  counter["step"] = 1;
  JsonArray lock = f["lock"].to<JsonArray>();
  JsonObject lb = lock.add<JsonObject>();
  lb["byte"] = 4;
  lb["base"] = "0xFE";
  lb["invert"] = false;
  JsonArray inject = f["inject"].to<JsonArray>();
  JsonObject inj = inject.add<JsonObject>();
  inj["source"] = "speed";
  inj["startBit"] = 0;
  inj["length"] = 16;
  inj["littleEndian"] = true;
  inj["signed"] = false;
  inj["factor"] = 0.01;
  inj["offset"] = 0;
  inj["lockGain"] = 0;
  JsonObject checksum = f["checksum"].to<JsonObject>();
  checksum["type"] = "none";
  checksum["byte"] = 7;
  checksum["from"] = 0;
  checksum["to"] = 6;

  JsonArray signals = doc["signals"].to<JsonArray>();
  JsonObject s = signals.add<JsonObject>();
  s["enabled"] = false;
  s["name"] = "Vehicle speed (example)";
  s["id"] = "0x00000000";
  s["extd"] = true;
  s["startBit"] = 8;
  s["length"] = 16;
  s["littleEndian"] = true;
  s["signed"] = false;
  s["factor"] = 0.01;
  s["offset"] = 0;
  s["unit"] = "km/h";

  String out;
  serializeJsonPretty(doc, out);
  return out;
}

static bool writeProfileFile(const String& path, const String& body) {
  if (!LittleFS.exists(k_profile_dir)) {
    LittleFS.mkdir(k_profile_dir);
  }
  File f = LittleFS.open(path, "w");
  if (!f) {
    return false;
  }
  size_t written = f.print(body);
  f.close();
  return written == body.length();
}

static bool readProfileFile(const String& path, String& out) {
  if (!LittleFS.exists(path)) {
    return false;
  }
  File f = LittleFS.open(path, "r");
  if (!f) {
    return false;
  }
  out = f.readString();
  f.close();
  return true;
}

// ---------------------------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------------------------

void volvoInit() {
  if (!g_profile_mutex) {
    g_profile_mutex = xSemaphoreCreateMutex();
  }
  if (!isVolvoPlatform()) {
    return;
  }
  if (!volvoProfileReload()) {
    LOG_WARN("volvo", "No profile at %s; writing template", volvoProfilePath().c_str());
    (void)volvoProfileWriteTemplate();
  }
}

bool volvoProfileReload() {
  String body;
  const String path = volvoProfilePath();
  if (!readProfileFile(path, body)) {
    g_last_error = "profile file missing: " + path;
    if (lockProfile(200)) {
      g_profile = volvo_profile_t{};
      g_signal_count = 0;
      unlockProfile();
    }
    return false;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err != DeserializationError::Ok) {
    g_last_error = String("profile parse error: ") + err.c_str();
    LOG_ERROR("volvo", "%s", g_last_error.c_str());
    return false;
  }
  String error;
  if (!applyProfileDocument(doc, error)) {
    g_last_error = error;
    LOG_ERROR("volvo", "profile rejected: %s", error.c_str());
    return false;
  }
  LOG_INFO("volvo", "Profile loaded %s frames=%u signals=%u status=%s", path.c_str(), (unsigned)g_profile.frame_count,
           (unsigned)g_signal_count, g_profile.status.enabled ? "on" : "off");
  return true;
}

bool volvoProfileApplyJson(const String& body, String& error) {
  if (body.length() == 0 || body.length() > VOLVO_PROFILE_MAX_BYTES) {
    error = String("profile must be 1..") + VOLVO_PROFILE_MAX_BYTES + " bytes";
    return false;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err != DeserializationError::Ok) {
    error = String("invalid JSON: ") + err.c_str();
    return false;
  }
  if (!applyProfileDocument(doc, error)) {
    g_last_error = error;
    return false;
  }
  String pretty;
  serializeJsonPretty(doc, pretty);
  const String path = volvoProfilePath();
  if (!writeProfileFile(path, pretty)) {
    error = "failed to write " + path;
    g_last_error = error;
    return false;
  }
  filelogLogEvent("volvo", String("profile saved ") + path + " frames=" + (int)g_profile.frame_count +
                             " signals=" + (int)g_signal_count);
  return true;
}

bool volvoProfileReadRaw(String& out) {
  return readProfileFile(volvoProfilePath(), out);
}

bool volvoProfileWriteTemplate() {
  String body = buildTemplateJson();
  String error;
  if (!volvoProfileApplyJson(body, error)) {
    LOG_ERROR("volvo", "template rejected: %s", error.c_str());
    return false;
  }
  return true;
}

static uint8_t computeChecksum(const volvo_frame_t& frame, const uint8_t* data) {
  const uint8_t from = frame.checksum.from;
  uint8_t to = frame.checksum.to;
  if (frame.dlc == 0) {
    return 0;
  }
  if (to >= frame.dlc) {
    to = (uint8_t)(frame.dlc - 1);
  }
  if (from > to) {
    return 0;
  }
  const uint8_t len = (uint8_t)(to - from + 1);
  switch (frame.checksum.type) {
  case VOLVO_CK_XOR: {
    uint8_t x = 0;
    for (uint8_t i = from; i <= to; i++) {
      x ^= data[i];
    }
    return x;
  }
  case VOLVO_CK_SUM: {
    uint16_t sum = 0;
    for (uint8_t i = from; i <= to; i++) {
      sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
  }
  case VOLVO_CK_SUM_INV: {
    uint16_t sum = 0;
    for (uint8_t i = from; i <= to; i++) {
      sum += data[i];
    }
    return (uint8_t)(0xFF - (sum & 0xFF));
  }
  case VOLVO_CK_CRC8_AUTOSAR:
    return crc8_autosar(data + from, len);
  default:
    return 0;
  }
}

static void buildFrame(volvo_frame_t& frame, twai_message_t& msg) {
  msg = {};
  msg.identifier = frame.extd ? (frame.id & 0x1FFFFFFF) : (frame.id & 0x7FF);
  msg.extd = frame.extd ? 1 : 0;
  msg.rtr = 0;
  msg.data_length_code = frame.dlc;
  for (uint8_t i = 0; i < 8; i++) {
    msg.data[i] = frame.data[i];
  }
  for (uint8_t i = 0; i < frame.inject_count; i++) {
    applyInject(frame.inject[i], msg.data);
  }
  for (uint8_t i = 0; i < frame.lock_count; i++) {
    const volvo_lock_byte_t& lb = frame.lock[i];
    if (lb.byte < 8) {
      msg.data[lb.byte] = get_lock_target_adjusted_value(lb.base, lb.invert);
    }
  }
  if (frame.counter.enabled && frame.counter.byte < 8) {
    const uint8_t mask = frame.counter.mask;
    const uint8_t shift = counterShift(mask);
    const uint8_t width_max = (uint8_t)(mask >> shift);
    const uint8_t value = (uint8_t)((frame.counter_value << shift) & mask);
    msg.data[frame.counter.byte] = (uint8_t)((msg.data[frame.counter.byte] & (uint8_t)~mask) | value);
    frame.counter_value = (uint8_t)((frame.counter_value + frame.counter.step) & width_max);
  }
  if (frame.checksum.type != VOLVO_CK_NONE && frame.checksum.byte < 8) {
    msg.data[frame.checksum.byte] = computeChecksum(frame, msg.data);
  }
}

void volvoGeneratorTick(uint32_t now_ms) {
  if (!g_profile.loaded || g_profile.frame_count == 0) {
    return;
  }
  if (!lockProfile(2)) {
    return;
  }
  for (uint8_t i = 0; i < g_profile.frame_count; i++) {
    volvo_frame_t& frame = g_profile.frames[i];
    if (!frame.enabled) {
      continue;
    }
    if (frame.tx_count > 0 && (uint32_t)(now_ms - frame.last_tx_ms) < frame.period_ms) {
      continue;
    }
    twai_message_t msg;
    buildFrame(frame, msg);
    if (haldex_can_send(msg, 0, true)) {
      frame.tx_count++;
    }
    frame.last_tx_ms = now_ms;
  }
  unlockProfile();
}

bool volvoDecodeStatusFrame(const twai_message_t& frame, uint16_t& engagement_raw, uint8_t& dem_state) {
  const volvo_status_cfg_t& status = g_profile.status;
  if (!g_profile.loaded || !status.enabled) {
    return false;
  }
  if ((frame.identifier & 0x1FFFFFFF) != (status.id & 0x1FFFFFFF)) {
    return false;
  }
  if (((bool)frame.extd) != status.extd) {
    return false;
  }
  if (status.engagement_byte >= frame.data_length_code) {
    return false;
  }
  engagement_raw = frame.data[status.engagement_byte];
  dem_state = 0;
  if (status.state_byte >= 0 && status.state_byte < frame.data_length_code) {
    dem_state = frame.data[status.state_byte];
  }
  return true;
}

bool volvoObserveHaldexFrame(const twai_message_t& frame) {
  uint16_t raw = 0;
  uint8_t dem_state = 0;
  if (!volvoDecodeStatusFrame(frame, raw, dem_state)) {
    return false;
  }
  const volvo_status_cfg_t& status = g_profile.status;
  long mapped = map((long)raw, (long)status.engagement_min, (long)status.engagement_max, 0L, 100L);
  received_haldex_engagement_raw = (uint8_t)(raw > 255 ? 255 : raw);
  received_haldex_engagement = (uint8_t)constrain(mapped, 0L, 100L);
  received_haldex_state = dem_state;
  g_last_status_ms = millis();
  g_status_count++;
  return true;
}

uint32_t volvoStatusFrameId() {
  return g_profile.status.enabled ? g_profile.status.id : 0;
}

bool volvoStatusFrameExtended() {
  return g_profile.status.extd;
}

const dbc_signal_t* volvoActiveSignals() {
  return g_signal_table;
}

uint16_t volvoActiveSignalCount() {
  return g_signal_count;
}

bool volvoSendRawFrame(uint32_t id, bool extd, uint8_t dlc, const uint8_t* data) {
  if (dlc > 8) {
    return false;
  }
  if (extd && id > 0x1FFFFFFF) {
    return false;
  }
  if (!extd && id > 0x7FF) {
    return false;
  }
  twai_message_t msg = {};
  msg.identifier = id;
  msg.extd = extd ? 1 : 0;
  msg.rtr = 0;
  msg.data_length_code = dlc;
  for (uint8_t i = 0; i < dlc; i++) {
    msg.data[i] = data[i];
  }
  const bool ok = haldex_can_send(msg, pdMS_TO_TICKS(20), true);
  if (ok) {
    g_raw_tx_count++;
  }
  return ok;
}

void volvoWriteStatusJson(JsonObject& out, bool include_frames) {
  const uint32_t now = millis();
  out["platform"] = vehiclePlatformName(vehiclePlatform);
  out["active"] = isVolvoPlatform();
  out["generatorRunning"] = isVolvoPlatform() && isStandalone && can_ready && g_profile.loaded;
  out["profilePath"] = volvoProfilePath();
  out["loaded"] = g_profile.loaded;
  out["name"] = g_profile.name;
  out["generation"] = g_profile.generation;
  out["haldexGeneration"] = haldexGeneration;
  out["lastError"] = g_last_error;
  out["loadedAgeMs"] = g_loaded_ms ? (now - g_loaded_ms) : 0;
  out["signalCount"] = g_signal_count;
  out["rawTxCount"] = g_raw_tx_count;

  uint8_t enabled_count = 0;
  uint32_t tx_total = 0;
  for (uint8_t i = 0; i < g_profile.frame_count; i++) {
    if (g_profile.frames[i].enabled) {
      enabled_count++;
    }
    tx_total += g_profile.frames[i].tx_count;
  }
  out["frameCount"] = g_profile.frame_count;
  out["enabledFrameCount"] = enabled_count;
  out["txCount"] = tx_total;

  JsonObject status = out["status"].to<JsonObject>();
  status["enabled"] = g_profile.status.enabled;
  status["id"] = hexId(g_profile.status.id, g_profile.status.extd);
  status["extd"] = g_profile.status.extd;
  status["seen"] = g_status_count > 0;
  status["count"] = g_status_count;
  status["ageMs"] = g_last_status_ms ? (now - g_last_status_ms) : 0;
  status["engagementRaw"] = received_haldex_engagement_raw;
  status["engagement"] = received_haldex_engagement;
  status["state"] = hexByte(received_haldex_state);

  if (!include_frames) {
    return;
  }
  JsonArray frames = out["frames"].to<JsonArray>();
  for (uint8_t i = 0; i < g_profile.frame_count; i++) {
    const volvo_frame_t& frame = g_profile.frames[i];
    JsonObject row = frames.add<JsonObject>();
    row["name"] = frame.name;
    row["enabled"] = frame.enabled;
    row["id"] = hexId(frame.id, frame.extd);
    row["extd"] = frame.extd;
    row["dlc"] = frame.dlc;
    row["periodMs"] = frame.period_ms;
    row["lockBytes"] = frame.lock_count;
    row["injectCount"] = frame.inject_count;
    if (frame.inject_count > 0) {
      JsonArray sources = row["inject"].to<JsonArray>();
      for (uint8_t j = 0; j < frame.inject_count; j++) {
        sources.add(injectSourceName(frame.inject[j].source));
      }
    }
    row["counter"] = frame.counter.enabled;
    row["checksum"] = checksumTypeName(frame.checksum.type);
    row["txCount"] = frame.tx_count;
    row["ageMs"] = frame.tx_count ? (now - frame.last_tx_ms) : 0;
  }
}
