/*******************************************************************************
 *  gree_2wire_capture.ino  —  v1.4  (all-length CMD capture)
 *  - IDL: prints only length + first byte (compact, 1 line)
 *  - CMD: prints full HEX + checksum for ALL packet lengths
 *  - 'b' = trigger, 'c' = clear, 'i' = recal
 *******************************************************************************/

#include <Arduino.h>

#define ADC_PIN_A         7
#define SAMPLE_RATE_HZ    10000
#define PACKET_IDLE_US    5000
#define PULSE_THRESH_HI   200
#define IDLE_LEARN_RATE   0.001f
#define PDM_SHORT_MAX_US  300
#define PDM_SYNC_MIN_US   1000
#define MAX_PKT_BYTES     32
#define CMD_WINDOW_MS     6000

static float     _idle_level    = 2048.0f;
static uint32_t  _last_pulse_us = 0;
static bool      _in_packet     = false;
static uint8_t   _pkt_bits[512];
static uint16_t  _pkt_bit_count = 0;
static uint16_t  _pkt_count     = 0;

static bool      _triggered     = false;
static uint32_t  _trigger_end   = 0;

// per-length last-seen store (for IDL suppression only)
struct LenBucket { uint8_t len; uint8_t data[MAX_PKT_BYTES]; };
static LenBucket _store[16];

static LenBucket* _get_bucket(uint8_t len) {
  int empty = -1;
  for (int i = 0; i < 16; i++) {
    if (_store[i].len == len) return &_store[i];
    if (_store[i].len == 0 && empty < 0) empty = i;
  }
  if (empty >= 0) { _store[empty].len = len; return &_store[empty]; }
  return nullptr;
}

struct PulseEvent { uint32_t us; int16_t delta; uint16_t gap_us; };
static PulseEvent _pulses[128];
static uint8_t    _pulse_count = 0;

static hw_timer_t*       _timer = nullptr;
static volatile uint16_t _adc_sample   = 0;
static volatile bool     _sample_ready = false;
static portMUX_TYPE      _mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR _onTimer() {
  portENTER_CRITICAL_ISR(&_mux);
  _adc_sample   = (uint16_t)analogRead(ADC_PIN_A);
  _sample_ready = true;
  portEXIT_CRITICAL_ISR(&_mux);
}

static void _decode_packet() {
  if (_pulse_count < 2) return;

  _pkt_bit_count = 0;
  for (uint8_t i = 0; i < _pulse_count; i++) {
    PulseEvent& p = _pulses[i];
    if (i == 0 || p.gap_us >= PDM_SYNC_MIN_US) continue;
    if (_pkt_bit_count < sizeof(_pkt_bits))
      _pkt_bits[_pkt_bit_count++] = (p.gap_us > PDM_SHORT_MAX_US) ? 1 : 0;
  }

  uint8_t n = _pkt_bit_count / 8;
  if (n == 0 || n > MAX_PKT_BYTES) return;

  uint8_t dec[MAX_PKT_BYTES];
  for (int i = 0; i < n; i++) {
    uint8_t b = 0;
    for (int j = 0; j < 8; j++) b = (b << 1) | _pkt_bits[i*8+j];
    dec[i] = b;
  }

  uint32_t now = millis();
  bool in_cmd = _triggered && (now < _trigger_end);
  if (_triggered && now >= _trigger_end) {
    _triggered = false;
    Serial.println("\n--- [WINDOW CLOSED] ---");
  }

  _pkt_count++;

  if (in_cmd) {
    // CMD: print EVERYTHING, full detail, regardless of length
    uint8_t xall = 0; uint16_t sum = 0;
    for (int i = 0; i < n; i++) { xall ^= dec[i]; sum += dec[i]; }

    Serial.printf("[CMD#%u] len=%u  HEX:", _pkt_count, n);
    for (int i = 0; i < n; i++) Serial.printf(" %02X", dec[i]);
    Serial.printf("  XA=%02X SM=%02X\n", xall, (uint8_t)sum);

  } else {
    // IDL: only print if different from last seen for this length (suppress repeats)
    LenBucket* b = _get_bucket(n);
    if (b && memcmp(dec, b->data, n) == 0) return;  // identical — silent drop
    if (b) memcpy(b->data, dec, n);

    // compact one-line summary
    Serial.printf("[IDL#%u] len=%u  %02X %02X %02X %02X...\n",
                  _pkt_count, n, dec[0],
                  n>1?dec[1]:0, n>2?dec[2]:0, n>3?dec[3]:0);
  }
}

static void _process_sample(uint16_t adc, uint32_t us) {
  int16_t delta    = (int16_t)adc - (int16_t)_idle_level;
  bool    is_pulse = (abs(delta) > (int16_t)PULSE_THRESH_HI);
  if (!is_pulse)
    _idle_level = _idle_level * (1.0f - IDLE_LEARN_RATE) + adc * IDLE_LEARN_RATE;
  uint32_t gap_us = us - _last_pulse_us;
  if (is_pulse) {
    if (gap_us > PACKET_IDLE_US && _in_packet) {
      _decode_packet(); _pulse_count = _pkt_bit_count = 0;
    }
    if (!_in_packet || gap_us > PACKET_IDLE_US) _in_packet = true;
    if (_pulse_count < 128)
      _pulses[_pulse_count++] = { us, delta, (uint16_t)(_pulse_count > 0 ? gap_us : 0) };
    _last_pulse_us = us;
  } else if (_in_packet && gap_us > PACKET_IDLE_US) {
    _decode_packet(); _pulse_count = _pkt_bit_count = 0; _in_packet = false;
  }
}

static void _calibrate() {
  Serial.println("[CAL] 2s...");
  uint32_t sum = 0, n = 0, end = millis() + 2000;
  while (millis() < end) { sum += analogRead(ADC_PIN_A); n++; delayMicroseconds(100); }
  _idle_level = (float)sum / n;
  Serial.printf("[CAL] Idle=%.1f\n", _idle_level);
}

void setup() {
  Serial.begin(115200); delay(300);
  analogReadResolution(12); analogSetAttenuation(ADC_11db);
  pinMode(ADC_PIN_A, INPUT);
  Serial.println("=== Gree v1.4 — all lengths, CMD detail ===");
  Serial.println("  b=trigger  c=clear  i=recal");
  _calibrate();
  _timer = timerBegin(0, 80, true);
  timerAttachInterrupt(_timer, &_onTimer, true);
  timerAlarmWrite(_timer, 1000000 / SAMPLE_RATE_HZ, true);
  timerAlarmEnable(_timer);
  Serial.println("[INIT] Listening...\n");
}

void loop() {
  static uint32_t sample_us = 0;
  while (true) {
    uint16_t s; bool ready;
    portENTER_CRITICAL(&_mux);
    ready = _sample_ready; s = _adc_sample; _sample_ready = false;
    portEXIT_CRITICAL(&_mux);
    if (!ready) break;
    sample_us += (1000000 / SAMPLE_RATE_HZ);
    _process_sample(s, sample_us);
  }
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'b') {
      _triggered = true; _trigger_end = millis() + CMD_WINDOW_MS;
      Serial.println("\n>>> [TRIGGER — press remote NOW] <<<");
    }
    if (c == 'c') {
      _pkt_count = 0; _triggered = false;
      memset(_store, 0, sizeof(_store));
      Serial.println("[CLR]");
    }
    if (c == 'i') _calibrate();
  }
}