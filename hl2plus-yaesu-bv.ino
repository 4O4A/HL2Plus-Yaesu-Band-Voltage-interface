/*
  HL2+ (Elecraft/KX3-style CAT) -> Yaesu analog band-data voltage
  Arduino Nano (ATmega328P, 16 MHz)

  Wiring
    D2  <- HL2+ CAT TX (3.3 V TTL), SoftwareSerial RX
    D3  -> HL2+ CAT RX via 1.5k/2.2k divider, ONLY if POLL_RADIO is 1
    D9  -> 1 kOhm -> (10 uF to GND) -> amplifier BAND input
    GND <-> HL2+ GND, amplifier GND
    D13    built-in LED, flashes on each valid frequency frame
    USB    115200 baud debug output (Serial Monitor)

  Yaesu band voltages (FT-817/857/897 convention, 0.33 V per step):
    160m 0.33  80m 0.67  40m 1.00  30m 1.33  20m 1.67
     17m 2.00  15m 2.33  12m 2.67  10m 3.00   6m 3.33
*/

#include <SoftwareSerial.h>

// ---------- user settings ----------
const long    RADIO_BAUD = 38400;  // KX3 / HR-50 default; try 9600 or 4800 if nothing decodes
#define       POLL_RADIO   0       // 1 = Nano sends "FA;" every 250 ms (use when the HR-50 is NOT connected)
const float   VCC_VOLTS  = 5.00;   // measure the Nano 5V pin with a DMM and enter the real value
const bool    DEBUG      = true;   // print decoded frequency / band on USB serial

const uint8_t PIN_RX  = 2;
const uint8_t PIN_TX  = 3;
const uint8_t PIN_PWM = 9;         // must stay D9 (Timer1 OC1A)
const uint8_t PIN_LED = 13;

// Band table. Ranges are wide on purpose so slightly out-of-band tuning still selects the LPF.
struct Band { uint32_t lo_khz; uint32_t hi_khz; float volts; const char* name; };
const Band BANDS[] = {
  {  1500,  2500, 0.33, "160m" },
  {  3000,  4500, 0.67, "80m"  },
  {  5000,  5600, 1.00, "60m (40m V)" },  // Yaesu has no 60m level; change to 0.67 if your amp prefers 80m LPF
  {  6500,  7500, 1.00, "40m"  },
  {  9500, 10500, 1.33, "30m"  },
  { 13500, 14500, 1.67, "20m"  },
  { 17500, 18500, 2.00, "17m"  },
  { 20500, 21500, 2.33, "15m"  },
  { 24000, 25500, 2.67, "12m"  },
  { 27500, 30000, 3.00, "10m"  },
  { 49000, 55000, 3.33, "6m"   },
};
const uint8_t N_BANDS = sizeof(BANDS) / sizeof(BANDS[0]);
// -----------------------------------

SoftwareSerial radio(PIN_RX, PIN_TX);

char     frame[40];
uint8_t  frameLen  = 0;
float    lastVolts = -1.0;
uint32_t ledOffAt  = 0;
uint32_t lastPoll  = 0;

void setupPwm() {
  pinMode(PIN_PWM, OUTPUT);
  // Timer1, fast PWM 10-bit (mode 7), no prescaler: 16 MHz / 1024 = 15.6 kHz on OC1A (D9)
  TCCR1A = _BV(COM1A1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM12)  | _BV(CS10);
  OCR1A  = 0;
}

void setVolts(float v) {
  if (v < 0) v = 0;
  if (v > VCC_VOLTS) v = VCC_VOLTS;
  OCR1A = (uint16_t)(v / VCC_VOLTS * 1023.0 + 0.5);
}

void applyFrequency(uint32_t hz) {
  uint32_t khz = hz / 1000UL;
  float v = 0.0;
  const char* name = "out of band -> 0 V";
  for (uint8_t i = 0; i < N_BANDS; i++) {
    if (khz >= BANDS[i].lo_khz && khz <= BANDS[i].hi_khz) { v = BANDS[i].volts; name = BANDS[i].name; break; }
  }
  if (v != lastVolts) {
    setVolts(v);
    lastVolts = v;
    if (DEBUG) { Serial.print(F("f=")); Serial.print(hz); Serial.print(F(" Hz  ")); Serial.print(name);
                 Serial.print(F("  -> ")); Serial.print(v, 2); Serial.println(F(" V")); }
  }
}

// Elecraft frames: "FA00014074000;" (11-digit Hz) or "IF00014074000     +000000 ...;"
void handleFrame(const char* f, uint8_t n) {
  bool fa = (f[0] == 'F' && f[1] == 'A');
  bool iff = (f[0] == 'I' && f[1] == 'F');
  if (!(fa || iff) || n < 13) return;
  if (f[2] != '0' || f[3] != '0') return;          // anything >= 1 GHz is garbage for us
  uint32_t hz = 0;
  for (uint8_t i = 4; i < 13; i++) {
    if (f[i] < '0' || f[i] > '9') return;
    hz = hz * 10UL + (f[i] - '0');
  }
  if (hz < 100000UL || hz > 60000000UL) return;    // 100 kHz .. 60 MHz sanity window
  digitalWrite(PIN_LED, HIGH);
  ledOffAt = millis() + 60;
  applyFrequency(hz);
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  setupPwm();
  Serial.begin(115200);
  radio.begin(RADIO_BAUD);
  if (DEBUG) Serial.println(F("HL2+ CAT -> Yaesu band voltage. Waiting for FA/IF frames..."));
}

void loop() {
  while (radio.available()) {
    char c = radio.read();
    if (c == '\r' || c == '\n') continue;
    if (c == ';') {
      frame[frameLen] = 0;
      handleFrame(frame, frameLen);
      frameLen = 0;
    } else if (frameLen < sizeof(frame) - 1) {
      frame[frameLen++] = c;
    } else {
      frameLen = 0;                                  // overflow: drop and resync
    }
  }

#if POLL_RADIO
  if (millis() - lastPoll > 250) { lastPoll = millis(); radio.print(F("FA;")); }
#endif

  if (ledOffAt && (int32_t)(millis() - ledOffAt) >= 0) { digitalWrite(PIN_LED, LOW); ledOffAt = 0; }
}
