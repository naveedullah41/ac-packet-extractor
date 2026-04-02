/*******************************************************************************
 *  KinCony B16 — BASIC/AUX AC RS-485 Sniffer  v8.0
 *
 *  HARDWARE:
 *    Board  : KinCony B16 (ESP32-S3)
 *    RS-485 : TX=GPIO39  RX=GPIO38  9600 8N1
 *    Polarity: A/B corrected (0x7E frames = correct wiring)
 *
 *  CONFIRMED PROTOCOL MAP
 *  ══════════════════════
 *
 *  TYPE 1 — STATUS heartbeat  (10 bytes, repeats 3×/cycle)
 *    7E  F1  [seq]  [room_raw]  [wall_cap]  00  00  21  [seq2]  [cs]  (7E)
 *    byte[3] = WALL CONTROLLER NTC temp → (raw - 0x28) × 0.5 °C
 *    byte[4] = 0x0A  ← wall controller capability byte, never changes
 *
 *  TYPE 2 — TEMP REPORT  (variable, key bytes near start)
 *    7E  01  F1  [01|02]  0D  [mf]  00  00  [set×10]  00  ...
 *    byte[5] = mode + fan combined:
 *      upper nibble → mode:  0x9=COOL  0xA=DRY  0xC=HEAT  0xE=FAN  0x1=OFF
 *      lower nibble → fan:   0x2=HIGH  0x4=MED  0x6=LOW   0xA=AUTO
 *    byte[8] = set temp × 10  (only meaningful in COOL / HEAT / DRY modes)
 *              in FAN-ONLY mode: byte[8] = stale last-mode value, IGNORE IT
 *
 *  TAIL PATTERN (embedded in T2 and larger frames):
 *    [mf]  00  00  [set×10]  00  [coil1×10]  00  [coil2×10]  00  13  [cshi]  [cslo]
 *    coil temps are from sensors on the heat-exchanger, valid in all modes.
 *    In FAN-ONLY mode tail may carry set×10 stale value — same rule: ignore set.
 *
 *  TYPE 3 — SENSOR  (11 bytes)
 *    7E  F1  F1  56  0B  [pipe_raw]  01  00  [comp]  [cs]  (7E)
 *    byte[5] = INDOOR UNIT NTC  → (raw - 0x28) × 0.5 °C
 *              *** This is a DIFFERENT sensor from the wall controller NTC.
 *                  In fan-only mode it shows ~25°C (room ambient at unit),
 *                  wall controller shows ~22.5°C (room ambient at controller).
 *                  Both are correct — just different physical locations.
 *    byte[8]: 0x40 = compressor ON   0x00 = compressor OFF
 *
 *  TYPE 4 — MODE frame  (21 bytes)
 *    7E  F1  00  A1  15  00  [mode_raw]  ...
 *    byte[6]: 0x61=COOL  0x41=DRY  0x81=HEAT  0x00=FAN/OFF
 *
 *  TEMPERATURE DISPLAY LOGIC (matches what LCD shows):
 *    COOL / HEAT / DRY  → LCD shows set temp  = byte[8] ÷ 10
 *    FAN-ONLY           → LCD shows unit NTC  = TYPE 3 byte[5] decoded
 *    OFF                → LCD shows last setpoint or room temp (unit-dependent)
 *
 *  Commands (115200):
 *    status   — print decoded AC state
 *    sniff    — verbose output (default)
 *    quiet    — T2 / state-change lines only
 *    ro / ri  — B16 output / input readback
 *    on N / off N — B16 output control
 *    help
 *******************************************************************************/

#include <Arduino.h>
#include <HardwareSerial.h>

// ─────────────────────────────────────────────────────────────────────────────
//  HARDWARE
// ─────────────────────────────────────────────────────────────────────────────
#define RS485_TX    39
#define RS485_RX    38
#define RS485_BAUD  9600
#define MODBUS_ADDR 0x01

static HardwareSerial bus(1);

// ─────────────────────────────────────────────────────────────────────────────
//  HDLC FRAME COLLECTOR
// ─────────────────────────────────────────────────────────────────────────────
#define HDLC_MAX      128
#define HDLC_FLUSH_MS 200

static uint8_t  hbuf[HDLC_MAX];
static uint8_t  hlen     = 0;
static bool     hinside  = false;
static uint32_t hstart   = 0;
static uint32_t fcount   = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  AC STATE
// ─────────────────────────────────────────────────────────────────────────────
struct AcState {
  // TYPE 1 — wall controller NTC
  bool    room_valid;
  float   room_wall_ntc;      // (raw-0x28)×0.5 — wall panel sensor

  // TYPE 3 — indoor unit NTC
  float   room_unit_ntc;      // (raw-0x28)×0.5 — sensor inside the unit
  bool    compressor_on;
  bool    sensor_valid;

  // TYPE 2 / TAIL — set temp & coils (only for cooling/heating/dry modes)
  bool    settemp_valid;      // false in fan-only mode
  float   set_temp;
  float   coil1, coil2;

  // Mode+fan (from T2 byte[5] / tail anchor)
  uint8_t mf_byte;            // 0x92/94/96=COOL+hi/med/lo  0xAA=DRY  0xE2=FAN  0x16=OFF ...
  bool    mf_valid;

  // TYPE 4
  uint8_t mode4;              // 0x61=COOL 0x41=DRY 0x81=HEAT 0x00=FAN/OFF

  uint32_t ts_room, ts_unit, ts_set, ts_mode;
} ac = {};

static bool verbose_mode = true;

// ─────────────────────────────────────────────────────────────────────────────
//  DECODERS
// ─────────────────────────────────────────────────────────────────────────────
inline bool is_fan_only(uint8_t mf)  { return (mf & 0xF0) == 0xE0; }
inline bool is_off(uint8_t mf)       { return (mf & 0xF0) == 0x10; }
inline bool has_setpoint(uint8_t mf) { return !is_fan_only(mf) && !is_off(mf); }

const char* mode_str(uint8_t mf) {
  switch (mf & 0xF0) {
    case 0x90: return "COOL";
    case 0xA0: return "DRY";
    case 0xC0: return "HEAT";
    case 0xE0: return "FAN-ONLY";
    case 0x10: return "OFF";
    default:   return "?";
  }
}

const char* fan_str(uint8_t mf) {
  if ((mf & 0xF0) == 0xA0 || (mf & 0xF0) == 0xE0 || (mf & 0xF0) == 0x10) return "N/A";
  switch (mf & 0x0F) {
    case 0x02: return "HIGH";
    case 0x04: return "MED";
    case 0x06: return "LOW";
    case 0x0A: return "AUTO";
    default:   return "?";
  }
}

const char* mode4_str(uint8_t r) {
  switch (r) {
    case 0x61: return "COOL";
    case 0x41: return "DRY";
    case 0x81: return "HEAT";
    case 0x00: return "FAN/OFF";
    default:   return "?";
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  MODBUS CRC
// ─────────────────────────────────────────────────────────────────────────────
static uint16_t mbcrc(const uint8_t* d, uint8_t n) {
  uint16_t c = 0xFFFF;
  for (uint8_t i = 0; i < n; i++) {
    c ^= d[i];
    for (uint8_t b = 0; b < 8; b++) c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
  }
  return c;
}
static uint8_t app_crc(uint8_t* b, uint8_t n) {
  uint16_t c = mbcrc(b, n); b[n] = c & 0xFF; b[n+1] = c >> 8; return n+2;
}
static bool ver_crc(const uint8_t* b, uint8_t n) {
  return n >= 4 && mbcrc(b, n-2) == (b[n-2] | ((uint16_t)b[n-1] << 8));
}

// ─────────────────────────────────────────────────────────────────────────────
//  FRAME CLASSIFICATION
// ─────────────────────────────────────────────────────────────────────────────
enum FT : uint8_t { FT_S1, FT_T2, FT_SENS, FT_MODE, FT_MB, FT_UNK };

FT classify(const uint8_t* d, uint8_t n) {
  if (n < 2) return FT_UNK;
  if (n >= 4 && d[0] == MODBUS_ADDR && ver_crc(d, n)) return FT_MB;
  if (d[0] != 0x7E) return FT_UNK;
  if (n==10 && d[1]==0xF1 && (d[2]&0xC0)==0xC0 &&
      d[5]==0x00 && d[6]==0x00 && d[7]==0x21)                return FT_S1;
  if (n>=9 && d[1]==0x01 && d[2]==0xF1 &&
      (d[3]==0x01||d[3]==0x02) && d[4]==0x0D)                return FT_T2;
  if (n==11 && d[1]==0xF1 && d[2]==0xF1 &&
      d[3]==0x56 && d[4]==0x0B)                              return FT_SENS;
  if (n>=7 && d[1]==0xF1 && d[2]==0x00 &&
      d[3]==0xA1 && d[4]==0x15 && d[5]==0x00)                return FT_MODE;
  return FT_UNK;
}

// ─────────────────────────────────────────────────────────────────────────────
//  TAIL SCANNER
//  Pattern: [mf_anchor] 00 00 [set×10] 00 [c1×10] 00 [c2×10] 00 13
//
//  KEY RULE: when mode = FAN-ONLY (mf & 0xF0 == 0xE0), the [set×10] byte is
//  stale data from the previous mode. We read coil temps (always valid) but
//  do NOT update ac.set_temp or ac.settemp_valid.
// ─────────────────────────────────────────────────────────────────────────────
static bool is_mf(uint8_t b) {
  switch (b) {
    // COOL variants
    case 0x92: case 0x94: case 0x96: case 0x9A:
    // DRY
    case 0xAA:
    // HEAT variants
    case 0xC2: case 0xC4: case 0xC6: case 0xCA:
    // FAN-ONLY
    case 0xE2:
    // OFF
    case 0x16:
      return true;
    default: return false;
  }
}

bool scan_tail(const uint8_t* d, uint8_t n, bool print) {
  for (int i = 0; i < (int)n - 9; i++) {
    if (!is_mf(d[i]))   continue;
    if (d[i+1] != 0x00) continue;   // only ONE zero before the 16-bit set temp
    // d[i+2] = set temp HIGH byte (0x00 for ≤25.5°C, 0x01 for 26-35°C, …)
    // d[i+3] = set temp LOW byte
    if (d[i+4] != 0x00 || d[i+6] != 0x00 || d[i+8] != 0x00) continue;
    if (i+9 < (int)n && d[i+9] != 0x13) continue;

    uint8_t  mf = d[i];
    uint16_t sr = ((uint16_t)d[i+2] << 8) | d[i+3];  // ×10 encoding, 16-bit
    uint8_t  c1 = d[i+5];
    uint8_t  c2 = d[i+7];

    float t1 = c1 / 10.0f;
    float t2 = c2 / 10.0f;
    float st = sr / 10.0f;

    // Coil sanity
    if (t1 < 10.0f || t1 > 60.0f) continue;

    // Set-temp sanity gate (skip obviously junk 16-bit values)
    bool fan_only = is_fan_only(mf);
    if (!fan_only && (st < 16.0f || st > 35.0f)) continue;

    // Update coils (always valid)
    ac.coil1    = t1;
    ac.coil2    = t2;
    ac.mf_byte  = mf;
    ac.mf_valid = true;

    // Set temp — only in modes that actually have a setpoint
    if (!fan_only) {
      ac.set_temp      = st;
      ac.settemp_valid = true;
      ac.ts_set        = millis();
    }

    if (print) {
      Serial.println("  ╔═ TAIL ══════════════════════════════════════════════╗");
      Serial.printf ("  ║  Mode      : %s\n", mode_str(mf));
      Serial.printf ("  ║  Fan       : %s  (mf=0x%02X)\n", fan_str(mf), mf);
      if (!fan_only) {
        Serial.printf("  ║  Set temp  : %.1f°C  (raw 0x%02X%02X = %u÷10)\n",
                      st, d[i+2], d[i+3], sr);
      } else {
        Serial.printf("  ║  Set temp  : (FAN-ONLY — LCD shows unit NTC: %.1f°C)\n",
                      ac.room_unit_ntc);
        Serial.printf("  ║  Stale raw : 0x%02X%02X = %.1f°C  (ignore)\n",
                      d[i+2], d[i+3], st);
      }
      Serial.printf ("  ║  Coil 1    : %.1f°C  (raw 0x%02X)\n", t1, c1);
      Serial.printf ("  ║  Coil 2    : %.1f°C  (raw 0x%02X)\n", t2, c2);
      Serial.println("  ╚═════════════════════════════════════════════════════╝");
    }
    return true;
  }
  return false;
}
// ─────────────────────────────────────────────────────────────────────────────
//  STATUS PRINT
// ─────────────────────────────────────────────────────────────────────────────
void print_status() {
  uint32_t now = millis();
  bool fan_only = ac.mf_valid && is_fan_only(ac.mf_byte);

  Serial.println("\n  ┌── AC STATE ──────────────────────────────────────────────┐");
  Serial.printf ("  │  Mode (T4)       : %s  (0x%02X)\n", mode4_str(ac.mode4), ac.mode4);
  if (ac.mf_valid) {
    Serial.printf("  │  Mode+Fan        : %s + %s  (mf=0x%02X)\n",
                  mode_str(ac.mf_byte), fan_str(ac.mf_byte), ac.mf_byte);
  }
  Serial.printf ("  │  Compressor      : %s\n", ac.compressor_on ? "ON" : "OFF");
  Serial.println("  ├──────────────────────────────────────────────────────────┤");

  // Temperature section
  if (fan_only) {
    // FAN-ONLY: LCD shows the indoor unit NTC (TYPE 3 sensor), not a setpoint
    Serial.println("  │  ── FAN-ONLY MODE ─────────────────────────────────────");
    Serial.println("  │  (No active setpoint. LCD displays unit NTC sensor.)");
    if (ac.sensor_valid)
      Serial.printf("  │  Unit NTC (LCD)  : %.1f°C  ← this is what LCD shows\n",
                    ac.room_unit_ntc);
    if (ac.room_valid)
      Serial.printf("  │  Wall NTC        : %.1f°C  ← wall controller sensor\n",
                    ac.room_wall_ntc);
    if (ac.settemp_valid)
      Serial.printf("  │  Last setpoint   : %.1f°C  (from previous mode, not active)\n",
                    ac.set_temp);
  } else {
    // Active cooling / heating / dry
    if (ac.settemp_valid)
      Serial.printf("  │  Set temp        : %.1f°C\n", ac.set_temp);
    else
      Serial.println("  │  Set temp        : (pending)");
    if (ac.room_valid)
      Serial.printf("  │  Room NTC (wall) : %.1f°C\n", ac.room_wall_ntc);
    if (ac.sensor_valid)
      Serial.printf("  │  Room NTC (unit) : %.1f°C\n", ac.room_unit_ntc);
  }

  Serial.println("  ├──────────────────────────────────────────────────────────┤");
  if (ac.coil1 > 0.0f)
    Serial.printf("  │  Coil 1          : %.1f°C\n", ac.coil1);
  if (ac.coil2 > 0.0f)
    Serial.printf("  │  Coil 2          : %.1f°C\n", ac.coil2);

  // Freshness
  Serial.println("  ├──────────────────────────────────────────────────────────┤");
  if (ac.ts_room)   Serial.printf("  │  Room age        : %lu ms\n", now - ac.ts_room);
  if (ac.ts_unit)   Serial.printf("  │  Unit NTC age    : %lu ms\n", now - ac.ts_unit);
  if (ac.ts_set)    Serial.printf("  │  Set temp age    : %lu ms\n", now - ac.ts_set);
  if (ac.ts_mode)   Serial.printf("  │  Mode age        : %lu ms\n", now - ac.ts_mode);
  Serial.println("  └──────────────────────────────────────────────────────────┘");
}

// ─────────────────────────────────────────────────────────────────────────────
//  FRAME DECODER
// ─────────────────────────────────────────────────────────────────────────────
static void decode_frame(const uint8_t* d, uint8_t n) {
  if (n == 0) return;
  fcount++;

  // Polarity check
  uint8_t ff = 0;
  for (int i = 0; i < n; i++) if (d[i] == 0xFF) ff++;
  if (ff > n / 2) {
    Serial.printf("[POLARITY #%lu] %d/%d = 0xFF — swap A/B\n", fcount, ff, n);
    return;
  }

  FT ft = classify(d, n);

  // ── STATUS ──────────────────────────────────────────────────────────────
  if (ft == FT_S1) {
    float rt = (d[3] - 0x28) * 0.5f;
    ac.room_wall_ntc = rt;
    ac.room_valid    = true;
    ac.ts_room       = millis();
    if (verbose_mode)
      Serial.printf("\n[S #%lu] wall_ntc=%.1f°C  seq=0x%02X\n", fcount, rt, d[2]);
    return;
  }

  // ── TEMP REPORT ─────────────────────────────────────────────────────────
  if (ft == FT_T2) {
    uint8_t mf = (n >= 6) ? d[5] : 0;
    uint8_t sr = (n >= 9) ? d[8] : 0;
    bool    fo = is_fan_only(mf);

    // Update mf_byte always
    ac.mf_byte  = mf;
    ac.mf_valid = true;

    // Only update set_temp when mode has a real setpoint
    if (!fo) {
      float st = sr / 10.0f;
      if (st >= 16.0f && st <= 32.0f) {
        ac.set_temp      = st;
        ac.settemp_valid = true;
        ac.ts_set        = millis();
      }
    }
    // In fan-only: DO NOT touch ac.set_temp — keep the last real value

    // Always print T2 (even in quiet mode)
    if (fo) {
      // Fan-only: tell user what LCD is actually showing
      Serial.printf("\n[T2 #%lu] FAN-ONLY  fan=%s  (LCD shows unit NTC: %.1f°C)  raw_b8=0x%02X(stale)\n",
                    fcount, fan_str(mf), ac.room_unit_ntc, sr);
    } else {
      float st = sr / 10.0f;
      Serial.printf("\n[T2 #%lu] %s + %s  set=%.1f°C  mf=0x%02X\n",
                    fcount, mode_str(mf), fan_str(mf), st, mf);
    }

    if (verbose_mode) {
      Serial.print("  HEX: ");
      for (int i = 0; i < n && i < 48; i++) Serial.printf("%02X ", d[i]);
      if (n > 48) Serial.printf("...+%d", n-48);
      Serial.println();
    }
    scan_tail(d, n, verbose_mode);
    return;
  }

  // ── SENSOR ──────────────────────────────────────────────────────────────
  if (ft == FT_SENS) {
    float pt = (d[5] - 0x28) * 0.5f;
    bool  co = (d[8] == 0x40);
    ac.room_unit_ntc  = pt;
    ac.compressor_on  = co;
    ac.sensor_valid   = true;
    ac.ts_unit        = millis();
    if (verbose_mode)
      Serial.printf("\n[SENSOR #%lu] unit_ntc=%.1f°C  compressor=%s\n",
                    fcount, pt, co ? "ON" : "OFF");
    return;
  }

  // ── MODE ────────────────────────────────────────────────────────────────
  if (ft == FT_MODE) {
    uint8_t mr = (n >= 7) ? d[6] : 0;
    ac.mode4   = mr;
    ac.ts_mode = millis();
    if (verbose_mode)
      Serial.printf("\n[MODE #%lu] %s (0x%02X)\n", fcount, mode4_str(mr), mr);
    return;
  }

  // ── MODBUS ──────────────────────────────────────────────────────────────
  if (ft == FT_MB) {
    if (verbose_mode)
      Serial.printf("[MODBUS #%lu] FC=0x%02X len=%d\n", fcount, d[1], n);
    return;
  }

  // ── UNKNOWN: scan for embedded tail ─────────────────────────────────────
  if (n >= 10) scan_tail(d, n, verbose_mode);
  if (verbose_mode) {
    Serial.printf("[? #%lu n=%d] ", fcount, n);
    for (int i = 0; i < n && i < 32; i++) Serial.printf("%02X ", d[i]);
    if (n > 32) Serial.printf("...+%d", n-32);
    Serial.println();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  HDLC RECEIVER
// ─────────────────────────────────────────────────────────────────────────────
static void hdlc_rx() {
  while (bus.available()) {
    uint8_t b = bus.read();

    if (b == 0x7E) {
      if (hinside && hlen > 0) decode_frame(hbuf, hlen);
      hlen    = 0;
      hinside = true;
      hstart  = millis();
      continue;
    }

    if (!hinside) continue;

    if (hlen >= HDLC_MAX) {
      if (verbose_mode)
        Serial.printf("[HDLC] overflow %d bytes, resyncing\n", hlen);
      hlen    = 0;
      hinside = false;
      continue;
    }
    hbuf[hlen++] = b;
  }

  // Fallback flush
  if (hinside && hlen > 0 && (millis() - hstart) >= HDLC_FLUSH_MS) {
    decode_frame(hbuf, hlen);
    hlen    = 0;
    hinside = false;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  MODBUS HELPERS
// ─────────────────────────────────────────────────────────────────────────────
static uint8_t rd_resp(uint8_t* buf, uint8_t mx) {
  uint32_t t = millis(); uint8_t i = 0;
  while (!bus.available() && millis()-t < 100) delay(1);
  while (millis()-t < 100 && i < mx)
    if (bus.available()) { buf[i++] = bus.read(); t = millis(); }
  return i;
}

bool b16_set_out(uint8_t ch, bool on) {
  if (ch < 1 || ch > 16) return false;
  uint8_t r[8];
  r[0]=MODBUS_ADDR; r[1]=0x05; r[2]=0x00; r[3]=ch-1;
  r[4]=on?0xFF:0x00; r[5]=0x00;
  uint8_t n = app_crc(r, 6);
  bus.write(r, n); bus.flush();
  uint8_t rsp[16]; uint8_t rn = rd_resp(rsp, 16);
  return rn >= 6 && ver_crc(rsp, rn) && rsp[1] == 0x05;
}

bool b16_read_outs(bool s[16]) {
  uint8_t r[8];
  r[0]=MODBUS_ADDR; r[1]=0x01; r[2]=0x00; r[3]=0x00; r[4]=0x00; r[5]=0x10;
  uint8_t n = app_crc(r, 6); bus.write(r, n); bus.flush();
  uint8_t rsp[16]; uint8_t rn = rd_resp(rsp, 16);
  if (rn < 5 || !ver_crc(rsp, rn)) return false;
  for (int i=0;i<8;i++) s[i]   = (rsp[3]>>i)&1;
  for (int i=0;i<8;i++) s[8+i] = (rsp[4]>>i)&1;
  return true;
}

bool b16_read_ins(bool s[16]) {
  uint8_t r[8];
  r[0]=MODBUS_ADDR; r[1]=0x02; r[2]=0x00; r[3]=0x00; r[4]=0x00; r[5]=0x10;
  uint8_t n = app_crc(r, 6); bus.write(r, n); bus.flush();
  uint8_t rsp[16]; uint8_t rn = rd_resp(rsp, 16);
  if (rn < 5 || !ver_crc(rsp, rn)) return false;
  for (int i=0;i<8;i++) s[i]   = (rsp[3]>>i)&1;
  for (int i=0;i<8;i++) s[8+i] = (rsp[4]>>i)&1;
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n═══════════════════════════════════════════════════════════════");
  Serial.println("  BASIC/AUX AC Sniffer  v8.0  |  KinCony B16  ESP32-S3");
  Serial.println("  Framing: HDLC (0x7E delimiter)");
  Serial.println("═══════════════════════════════════════════════════════════════");
  Serial.println("  TWO TEMPERATURE SENSORS ON BUS:");
  Serial.println("  • Wall controller NTC (TYPE 1 byte[3]) — sensor at wall panel");
  Serial.println("  • Indoor unit NTC    (TYPE 3 byte[5]) — sensor inside unit");
  Serial.println("  Both are correct; physical location difference explains gap.");
  Serial.println();
  Serial.println("  FAN-ONLY MODE RULE:");
  Serial.println("  • No setpoint is transmitted/displayed.");
  Serial.println("  • LCD shows indoor unit NTC (TYPE 3 byte[5]).");
  Serial.println("  • TYPE 2 byte[8] is stale — ignored by this decoder.");
  Serial.println();
  Serial.println("  COMMANDS: status / sniff / quiet / ro / ri / on N / off N");
  Serial.println("═══════════════════════════════════════════════════════════════\n");
  bus.begin(RS485_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  Serial.println("[OK] Listening...\n");
}

// ─────────────────────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  hdlc_rx();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if      (cmd == "status") { print_status(); }
    else if (cmd == "sniff")  { verbose_mode = true;  Serial.println("[verbose]"); }
    else if (cmd == "quiet")  { verbose_mode = false; Serial.println("[quiet]");   }
    else if (cmd == "ro") {
      bool s[16];
      if (b16_read_outs(s)) for (int i=0;i<16;i++) Serial.printf("  Out%02d: %s\n",i+1,s[i]?"ON":"OFF");
      else Serial.println("[ERR]");
    }
    else if (cmd == "ri") {
      bool s[16];
      if (b16_read_ins(s)) for (int i=0;i<16;i++) Serial.printf("  In%02d: %s\n",i+1,s[i]?"HIT":"idle");
      else Serial.println("[ERR]");
    }
    else if (cmd.startsWith("on "))  {
      int ch = cmd.substring(3).toInt();
      Serial.printf(b16_set_out(ch, true)  ? "[OK] Out%d ON\n"  : "[ERR]\n", ch);
    }
    else if (cmd.startsWith("off ")) {
      int ch = cmd.substring(4).toInt();
      Serial.printf(b16_set_out(ch, false) ? "[OK] Out%d OFF\n" : "[ERR]\n", ch);
    }
    else if (cmd == "help") {
      Serial.println("status / sniff / quiet / ro / ri / on N / off N");
    }
    else if (cmd.length() > 0) Serial.printf("[?] %s\n", cmd.c_str());
  }
}
