/*******************************************************************************
 *  rs485_sniffer.h  —  RS-485 Bus Sniffer & Protocol Identifier  v1.0
 *
 *  Passively listens on an RS-485 bus and attempts to identify which
 *  AC protocol is running:
 *
 *    BASIC / AUX  — HDLC framing, 0x7E flags, proprietary binary frames
 *    GREE         — Modbus RTU master/slave, status word 0x00AA / 0x0055,
 *                   FC 0x03 response with 40 byte-count (20 words)
 *    FAUJI        — Modbus RTU, status word 0x0001 / 0x0000,
 *                   FC 0x03 response with 12 byte-count (6 words)
 *    FISHER       — Unknown / not yet characterised; detected as generic
 *                   Modbus RTU that doesn't match GREE or FAUJI patterns
 *
 *  Additionally prints every raw byte received so you can capture unknown
 *  patterns on the Serial monitor.
 *
 *  HARDWARE:
 *    Same wiring as the production drivers.
 *    RS-485 A/B → KinCony B16 built-in RS-485 (GPIO38=RX, GPIO39=TX)
 *    Baud       : 9600  8N1  (set SNIFF_BAUD below if different)
 *    No DE/RE pin needed — RX-only, never drives the bus.
 *
 *  USAGE IN A STANDALONE SKETCH:
 *    #include "rs485_sniffer.h"
 *    void setup() { sniffer_setup(); }
 *    void loop()  { sniffer_loop();  }
 *
 *  SERIAL COMMANDS (send via Serial Monitor, any baud):
 *    'r'  → toggle raw-hex dump ON / OFF  (default: ON)
 *    'a'  → toggle ASCII printable dump ON / OFF
 *    'c'  → clear frame/score counters
 *    's'  → print current detection summary
 *    'h'  → print this help
 *
 *  OUTPUT EXAMPLE:
 *    [RAW]  7E F1 C1 40 00 00 21 24 7E          (9 bytes)  Δ12ms
 *    [RAW]  01 03 28 00 AA 00 00 03 E8 ...       (45 bytes) Δ2004ms
 *    [DETECT] Confidence: BASIC=8 GREE=0 FAUJI=0 FISHER=0 → BASIC/AUX
 *******************************************************************************/

#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

// ─────────────────────────────────────────────────────────────────────────────
//  CONFIGURATION
// ─────────────────────────────────────────────────────────────────────────────
#define SNIFF_RX_PIN      38
#define SNIFF_TX_PIN      39    // not driven, but UART needs it configured
#define SNIFF_BAUD        9600
#define SNIFF_UART        1

#define SNIFF_IDLE_MS     15    // gap between frames (≥3.5 char-times @ 9600)
#define SNIFF_BUF_MAX     256   // max bytes per frame
#define SNIFF_SCORE_WIN   20    // number of frames in rolling detection window
#define SNIFF_PRINT_MS    5000  // how often to print detection summary

// ─────────────────────────────────────────────────────────────────────────────
//  INTERNAL STATE
// ─────────────────────────────────────────────────────────────────────────────
static HardwareSerial _sniff_bus(SNIFF_UART);

static uint8_t  _sniff_buf[SNIFF_BUF_MAX];
static uint16_t _sniff_len      = 0;
static uint32_t _sniff_last_rx  = 0;
static uint32_t _sniff_frame_ts = 0;   // timestamp of first byte of current frame

static bool _sniff_raw_on   = true;
static bool _sniff_ascii_on = false;

// Detection scores (counts over rolling window)
static int16_t _score_basic  = 0;
static int16_t _score_gree   = 0;
static int16_t _score_fauji  = 0;
static int16_t _score_fisher = 0;
static uint32_t _sniff_frames = 0;

static uint32_t _sniff_last_summary = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  CRC-16 MODBUS  (poly 0xA001, init 0xFFFF, LSB first)
// ─────────────────────────────────────────────────────────────────────────────
static uint16_t _sniff_crc16(const uint8_t* d, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t j = 0; j < len; j++) {
    crc ^= d[j];
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

static bool _sniff_crc_ok(const uint8_t* d, uint16_t len) {
  if (len < 4) return false;
  uint16_t calc = _sniff_crc16(d, len - 2);
  uint16_t recv = (uint16_t)d[len-2] | ((uint16_t)d[len-1] << 8);
  return calc == recv;
}

// ─────────────────────────────────────────────────────────────────────────────
//  PRINT HELPERS
// ─────────────────────────────────────────────────────────────────────────────
static void _sniff_print_hex(const uint8_t* d, uint16_t len, uint32_t dt_ms) {
  Serial.print("[RAW]  ");
  uint16_t show = (len > 32) ? 32 : len;   // cap at 32 bytes on one line
  for (uint16_t i = 0; i < show; i++) {
    if (d[i] < 0x10) Serial.print('0');
    Serial.print(d[i], HEX);
    Serial.print(' ');
  }
  if (len > 32) { Serial.print("... (+"); Serial.print(len - 32); Serial.print(" more)"); }
  Serial.print("  ("); Serial.print(len); Serial.print(" bytes)");
  Serial.print("  Δ"); Serial.print(dt_ms); Serial.println("ms");
}

static void _sniff_print_ascii(const uint8_t* d, uint16_t len) {
  Serial.print("[ASCII] ");
  for (uint16_t i = 0; i < len && i < 64; i++)
    Serial.print((d[i] >= 0x20 && d[i] < 0x7F) ? (char)d[i] : '.');
  Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
//  PROTOCOL FINGERPRINTING
// ─────────────────────────────────────────────────────────────────────────────

// ── BASIC / AUX fingerprints ──────────────────────────────────────────────────
// From basic_ac.h:
//   BAC_S1  : 10 bytes, d[0]=0x7E, d[1]=0xF1, d[2]&0xC0==0xC0, d[5..6]=0x00, d[7]=0x21
//   BAC_T2  : n>=9,     d[0]=0x7E, d[1]=0x01,  d[2]=0xF1, d[3]=0x01/0x02, d[4]=0x0D
//   BAC_SENS: 11 bytes, d[0]=0x7E, d[1]=0xF1,  d[2]=0xF1, d[3]=0x56, d[4]=0x0B
//   BAC_MODE4: n>=7,    d[0]=0x7E, d[1]=0xF1,  d[2]=0x00, d[3]=0xA1, d[4]=0x15
static int8_t _sniff_score_basic(const uint8_t* d, uint16_t n) {
  if (n < 2) return 0;

  // Any HDLC-style 0x7E start is a strong indicator
  if (d[0] != 0x7E) return 0;

  // Too many 0xFF bytes = polarity error, not a real frame
  uint16_t ff = 0;
  for (uint16_t i = 0; i < n; i++) if (d[i] == 0xFF) ff++;
  if (ff > n / 2) return 0;

  // BAC_S1
  if (n == 10 && d[1] == 0xF1 && (d[2] & 0xC0) == 0xC0 &&
      d[5] == 0x00 && d[6] == 0x00 && d[7] == 0x21)
    return 3;

  // BAC_T2 (command / ack frame)
  if (n >= 9 && d[1] == 0x01 && d[2] == 0xF1 &&
      (d[3] == 0x01 || d[3] == 0x02) && d[4] == 0x0D)
    return 3;

  // BAC_SENS
  if (n == 11 && d[1] == 0xF1 && d[2] == 0xF1 &&
      d[3] == 0x56 && d[4] == 0x0B)
    return 3;

  // BAC_MODE4
  if (n >= 7 && d[1] == 0xF1 && d[2] == 0x00 &&
      d[3] == 0xA1 && d[4] == 0x15 && d[5] == 0x00)
    return 3;

  // Generic HDLC 0x7E start — weak signal
  return 1;
}

// ── MODBUS RTU shared check ───────────────────────────────────────────────────
// Returns true if frame has valid Modbus CRC and looks structurally sound.
static bool _sniff_is_modbus(const uint8_t* d, uint16_t n) {
  if (n < 4) return false;
  // Station address 1-247, FC in valid range
  if (d[0] == 0 || d[0] > 247) return false;
  uint8_t fc = d[1];
  if (fc != 0x01 && fc != 0x03 && fc != 0x05 &&
      fc != 0x06 && fc != 0x0F && fc != 0x10 &&
      !(fc >= 0x80 && fc <= 0xFF))  // exception responses
    return false;
  return _sniff_crc_ok(d, n);
}

// ── GREE fingerprints ─────────────────────────────────────────────────────────
// From gree_ac.h:
//   FC 0x03 query : 8 bytes, start=3 (0x0003), count=20 (0x0014)
//   FC 0x03 reply : byte_count = 40 (0x28), status word = 0x00AA or 0x0055
//   FC 0x10 write : start=14 (0x000E), count=9 (0x0009)
//   FC 0x0F coil  : coil_start=16 (0x0010), count=1
static int8_t _sniff_score_gree(const uint8_t* d, uint16_t n) {
  if (!_sniff_is_modbus(d, n)) return 0;
  int8_t score = 1;   // valid Modbus = base point

  uint8_t fc = d[1];

  // FC 0x03 query: start=0x0003, count=0x0014
  if (fc == 0x03 && n == 8) {
    uint16_t start = ((uint16_t)d[2] << 8) | d[3];
    uint16_t count = ((uint16_t)d[4] << 8) | d[5];
    if (start == 0x0003 && count == 0x0014) score += 3;
  }

  // FC 0x03 response: byte_count = 40 (20 words)
  if (fc == 0x03 && n >= 5) {
    if (d[2] == 40) {
      score += 2;
      // Status word at offset 3+0 = 0x00AA (on) or 0x0055 (off)
      if (n >= 7) {
        uint16_t sw = ((uint16_t)d[3] << 8) | d[4];
        if (sw == 0x00AA || sw == 0x0055) score += 3;
      }
    }
  }

  // FC 0x10 write: start=0x000E (14), count=0x0009 (9 words)
  if (fc == 0x10 && n >= 7) {
    uint16_t start = ((uint16_t)d[2] << 8) | d[3];
    uint16_t count = ((uint16_t)d[4] << 8) | d[5];
    if (start == 0x000E && count == 0x0009) score += 3;
  }

  // FC 0x0F coil write: coil_start=0x0010, count=1
  if (fc == 0x0F && n >= 7) {
    uint16_t cs = ((uint16_t)d[2] << 8) | d[3];
    uint16_t cc = ((uint16_t)d[4] << 8) | d[5];
    if (cs == 0x0010 && cc == 0x0001) score += 3;
  }

  return score;
}

// ── FAUJI fingerprints ────────────────────────────────────────────────────────
// From fauji_ac.h:
//   FC 0x03 query : 8 bytes, start=0 (0x0000), count=6 (0x0006)
//   FC 0x03 reply : byte_count = 12 (6 words), status word = 0x0001 or 0x0000
//   FC 0x10 write : start=2 (0x0002), count=4 (0x0004)
static int8_t _sniff_score_fauji(const uint8_t* d, uint16_t n) {
  if (!_sniff_is_modbus(d, n)) return 0;
  int8_t score = 1;

  uint8_t fc = d[1];

  // FC 0x03 query: start=0x0000, count=0x0006
  if (fc == 0x03 && n == 8) {
    uint16_t start = ((uint16_t)d[2] << 8) | d[3];
    uint16_t count = ((uint16_t)d[4] << 8) | d[5];
    if (start == 0x0000 && count == 0x0006) score += 3;
  }

  // FC 0x03 response: byte_count = 12 (6 words)
  if (fc == 0x03 && n >= 5) {
    if (d[2] == 12) {
      score += 2;
      // Status word 0x0001 (on) or 0x0000 (off)
      if (n >= 7) {
        uint16_t sw = ((uint16_t)d[3] << 8) | d[4];
        if (sw == 0x0001 || sw == 0x0000) score += 2;
      }
    }
  }

  // FC 0x10 write: start=0x0002, count=0x0004
  if (fc == 0x10 && n >= 7) {
    uint16_t start = ((uint16_t)d[2] << 8) | d[3];
    uint16_t count = ((uint16_t)d[4] << 8) | d[5];
    if (start == 0x0002 && count == 0x0004) score += 3;
  }

  return score;
}

// ── FISHER fingerprint ────────────────────────────────────────────────────────
// Fisher protocol not yet characterised. We flag it as Modbus RTU that
// doesn't strongly match Gree or Fauji — score 1 for valid Modbus CRC,
// and a small bonus if it smells like an AC (register range 0-30, sane values).
static int8_t _sniff_score_fisher(const uint8_t* d, uint16_t n) {
  if (!_sniff_is_modbus(d, n)) return 0;
  int8_t g = _sniff_score_gree(d, n);
  int8_t f = _sniff_score_fauji(d, n);
  // Only credit Fisher if neither Gree nor Fauji claimed it strongly
  if (g >= 3 || f >= 3) return 0;
  return 1;   // unknown Modbus — could be Fisher
}

// ─────────────────────────────────────────────────────────────────────────────
//  ACCUMULATE SCORES  (decay older frames by halving every SNIFF_SCORE_WIN frames)
// ─────────────────────────────────────────────────────────────────────────────
static void _sniff_accumulate(const uint8_t* d, uint16_t n) {
  _sniff_frames++;

  // Gentle decay every window
  if (_sniff_frames % SNIFF_SCORE_WIN == 0) {
    _score_basic  /= 2;
    _score_gree   /= 2;
    _score_fauji  /= 2;
    _score_fisher /= 2;
  }

  _score_basic  += _sniff_score_basic(d, n);
  _score_gree   += _sniff_score_gree(d, n);
  _score_fauji  += _sniff_score_fauji(d, n);
  _score_fisher += _sniff_score_fisher(d, n);
}

// ─────────────────────────────────────────────────────────────────────────────
//  DETECTION RESULT
// ─────────────────────────────────────────────────────────────────────────────
static const char* _sniff_best() {
  int16_t best = 0;
  const char* name = "UNKNOWN";
  if (_score_basic  > best) { best = _score_basic;  name = "BASIC/AUX"; }
  if (_score_gree   > best) { best = _score_gree;   name = "GREE";      }
  if (_score_fauji  > best) { best = _score_fauji;  name = "FAUJI";     }
  if (_score_fisher > best) { best = _score_fisher; name = "FISHER(?)"; }
  return name;
}

static void _sniff_print_summary() {
  Serial.println();
  Serial.println("══════════════════════════════════════════════════════");
  Serial.print  ("[DETECT] Frames seen : "); Serial.println(_sniff_frames);
  Serial.print  ("[DETECT] Scores      : ");
  Serial.print  ("BASIC=");  Serial.print(_score_basic);
  Serial.print  ("  GREE="); Serial.print(_score_gree);
  Serial.print  ("  FAUJI=");Serial.print(_score_fauji);
  Serial.print  ("  FISHER=");Serial.println(_score_fisher);
  Serial.print  ("[DETECT] Best match  : "); Serial.println(_sniff_best());
  Serial.println("══════════════════════════════════════════════════════");
  Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
//  FRAME ANALYSIS — called once a complete frame is captured
// ─────────────────────────────────────────────────────────────────────────────
static void _sniff_analyse(const uint8_t* d, uint16_t n, uint32_t dt_ms) {
  if (n == 0) return;

  // ── Print raw hex ─────────────────────────────────────────────────────────
  if (_sniff_raw_on)   _sniff_print_hex(d, n, dt_ms);
  if (_sniff_ascii_on) _sniff_print_ascii(d, n);

  // ── Per-frame protocol hint ───────────────────────────────────────────────
  int8_t sb = _sniff_score_basic(d, n);
  int8_t sg = _sniff_score_gree(d, n);
  int8_t sf = _sniff_score_fauji(d, n);
  int8_t sx = _sniff_score_fisher(d, n);

  if (sb > 0 || sg > 0 || sf > 0 || sx > 0) {
    Serial.print("[HINT]  ");
    if (sb >= sg && sb >= sf && sb >= sx) {
      Serial.print("looks like BASIC/AUX  (score="); Serial.print(sb); Serial.print(')');
      // Decode mf-byte if BAC_T2
      if (n >= 6 && d[0] == 0x7E && d[1] == 0x01 && d[2] == 0xF1 &&
          (d[3] == 0x01 || d[3] == 0x02) && d[4] == 0x0D) {
        uint8_t mf = d[5];
        Serial.print("  mf=0x"); Serial.print(mf, HEX);
        uint8_t mode_nib = (mf & 0xF0) >> 4;
        uint8_t fan_nib  = (mf & 0x0F);
        const char* mstr =
          (mode_nib == 0x9) ? "COOL" :
          (mode_nib == 0xA) ? "DRY"  :
          (mode_nib == 0xC) ? "HEAT" :
          (mode_nib == 0xE) ? "FAN"  :
          (mode_nib == 0x1) ? "OFF"  : "?";
        const char* fstr =
          (fan_nib == 0x2) ? "HIGH" :
          (fan_nib == 0x4) ? "MED"  :
          (fan_nib == 0x6) ? "LOW"  :
          (fan_nib == 0xA) ? "AUTO" : "?";
        Serial.print("  mode="); Serial.print(mstr);
        Serial.print("  fan=");  Serial.print(fstr);
        if (n >= 9) {
          uint16_t sr = ((uint16_t)d[7] << 8) | d[8];
          float st = sr / 10.0f;
          if (st >= 16.0f && st <= 35.0f) {
            Serial.print("  setpt="); Serial.print(st, 1); Serial.print("°C");
          }
        }
      }
    } else if (sg >= sf && sg >= sx) {
      Serial.print("looks like GREE       (score="); Serial.print(sg); Serial.print(')');
      // Decode FC 0x03 response
      if (d[1] == 0x03 && n >= 5 && d[2] == 40) {
        bool pwr = (((uint16_t)d[3] << 8) | d[4]) == 0x00AA;
        Serial.print("  power="); Serial.print(pwr ? "ON" : "OFF");
        if (n >= 1 + 1 + 1 + 40 + 2) {
          const uint8_t* p = d + 3;
          auto w = [&](uint8_t off) -> uint16_t {
            return ((uint16_t)p[off*2] << 8) | p[off*2+1];
          };
          // Words relative to GAC_READ_START=3: mode at offset 11, fan at 16, temp at 19
          uint16_t mode_r = w(11);
          uint16_t fan_r  = w(16);
          uint16_t temp_r = w(19);
          const char* mstr =
            (mode_r==1)?"COOL":(mode_r==2)?"HEAT":(mode_r==3)?"DRY":
            (mode_r==4)?"FAN":(mode_r==5)?"AUTO":"?";
          const char* fstr =
            (fan_r==1)?"LOW":(fan_r==2)?"MED":(fan_r==3)?"HIGH":
            (fan_r==0)?"AUTO":"?";
          Serial.print("  mode="); Serial.print(mstr);
          Serial.print("  fan=");  Serial.print(fstr);
          Serial.print("  temp="); Serial.print((int)temp_r); Serial.print("°C");
        }
      }
    } else if (sf >= sx) {
      Serial.print("looks like FAUJI      (score="); Serial.print(sf); Serial.print(')');
      // Decode FC 0x03 response
      if (d[1] == 0x03 && n >= 5 && d[2] == 12) {
        bool pwr = (((uint16_t)d[3] << 8) | d[4]) == 0x0001;
        Serial.print("  power="); Serial.print(pwr ? "ON" : "OFF");
        if (n >= 1 + 1 + 1 + 12 + 2) {
          const uint8_t* p = d + 3;
          auto w = [&](uint8_t off) -> uint16_t {
            return ((uint16_t)p[off*2] << 8) | p[off*2+1];
          };
          uint16_t mode_r = w(2);   // FAC_REG_SET_MODE = 2
          uint16_t fan_r  = w(3);   // FAC_REG_SET_FAN  = 3
          uint16_t temp_r = w(4);   // FAC_REG_SET_TEMP = 4  (×10)
          const char* mstr =
            (mode_r==1)?"COOL":(mode_r==2)?"HEAT":(mode_r==3)?"DRY":
            (mode_r==4)?"FAN":(mode_r==5)?"AUTO":"?";
          const char* fstr =
            (fan_r==1)?"LOW":(fan_r==2)?"MED":(fan_r==3)?"HIGH":
            (fan_r==0)?"AUTO":"?";
          Serial.print("  mode="); Serial.print(mstr);
          Serial.print("  fan=");  Serial.print(fstr);
          Serial.print("  temp="); Serial.print(temp_r / 10.0f, 1); Serial.print("°C");
        }
      }
    } else {
      Serial.print("looks like FISHER(?)  (score="); Serial.print(sx); Serial.print(')');
      Serial.print("  — capture more frames to confirm");
    }
    Serial.println();
  }

  _sniff_accumulate(d, n);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SERIAL COMMAND HANDLER
// ─────────────────────────────────────────────────────────────────────────────
static void _sniff_handle_cmd(char c) {
  switch (c) {
    case 'r':
      _sniff_raw_on = !_sniff_raw_on;
      Serial.print("[CMD] Raw hex dump: "); Serial.println(_sniff_raw_on ? "ON" : "OFF");
      break;
    case 'a':
      _sniff_ascii_on = !_sniff_ascii_on;
      Serial.print("[CMD] ASCII dump: "); Serial.println(_sniff_ascii_on ? "ON" : "OFF");
      break;
    case 'c':
      _score_basic = _score_gree = _score_fauji = _score_fisher = 0;
      _sniff_frames = 0;
      Serial.println("[CMD] Counters cleared.");
      break;
    case 's':
      _sniff_print_summary();
      break;
    case 'h':
      Serial.println();
      Serial.println("[HELP] Commands:");
      Serial.println("  r  — toggle raw hex dump (default ON)");
      Serial.println("  a  — toggle ASCII dump");
      Serial.println("  c  — clear scores and frame counter");
      Serial.println("  s  — print detection summary now");
      Serial.println("  h  — this help");
      break;
    default:
      break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  PUBLIC API
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);

  _sniff_bus.begin(SNIFF_BAUD, SERIAL_8N1, SNIFF_RX_PIN, SNIFF_TX_PIN);
  _sniff_bus.setRxBufferSize(512);
  _sniff_last_rx = millis();

  Serial.println();
  Serial.println("╔══════════════════════════════════════════════════════╗");
  Serial.println("║   RS-485 Sniffer & Protocol Identifier  v1.0        ║");
  Serial.println("╠══════════════════════════════════════════════════════╣");
  Serial.print  ("║  RX GPIO : "); Serial.print(SNIFF_RX_PIN);
  Serial.print  ("   Baud : "); Serial.print(SNIFF_BAUD);
  Serial.println("                     ║");
  Serial.println("║  Detecting : BASIC/AUX  GREE  FAUJI  FISHER        ║");
  Serial.println("║  Send 'h' for commands                               ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  Serial.println();
}

void loop() {
  uint32_t now = millis();

  // ── Read incoming bytes ───────────────────────────────────────────────────
  while (_sniff_bus.available()) {
    uint8_t b = _sniff_bus.read();
    now = millis();

    // New frame? — if bus was idle for SNIFF_IDLE_MS, flush previous frame first
    if (_sniff_len > 0 && (now - _sniff_last_rx) >= SNIFF_IDLE_MS) {
      _sniff_analyse(_sniff_buf, _sniff_len, now - _sniff_frame_ts);
      _sniff_len = 0;
    }

    if (_sniff_len == 0) _sniff_frame_ts = now;

    if (_sniff_len < SNIFF_BUF_MAX) {
      _sniff_buf[_sniff_len++] = b;
    } else {
      // Buffer overflow — flush and restart
      _sniff_analyse(_sniff_buf, _sniff_len, now - _sniff_frame_ts);
      _sniff_len = 0;
      _sniff_buf[_sniff_len++] = b;
      _sniff_frame_ts = now;
    }

    _sniff_last_rx = now;
  }

  // ── Idle flush: no bytes for SNIFF_IDLE_MS → frame is complete ───────────
  if (_sniff_len > 0 && (millis() - _sniff_last_rx) >= SNIFF_IDLE_MS) {
    _sniff_analyse(_sniff_buf, _sniff_len, millis() - _sniff_frame_ts);
    _sniff_len = 0;
  }

  // ── Periodic detection summary ────────────────────────────────────────────
  if ((millis() - _sniff_last_summary) >= SNIFF_PRINT_MS && _sniff_frames > 0) {
    _sniff_last_summary = millis();
    _sniff_print_summary();
  }

  // ── Serial commands from the user ─────────────────────────────────────────
  while (Serial.available()) {
    _sniff_handle_cmd((char)Serial.read());
  }
}