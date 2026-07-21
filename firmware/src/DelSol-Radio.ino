#include <esp_task_wdt.h>
#include <esp_adc_cal.h>
#include <driver/adc.h>
#include <driver/i2s.h>
#include <U8g2lib.h>
#include <BluetoothA2DPSink.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

const char *__version__ = "0.9.1";   // CI greps this literal to stamp firmware.json
uint8_t fw_ver[3] = {0, 0, 0};        // parsed from __version__ at boot, reported over BLE

// where the unit pulls new firmware from when the phone triggers a web update —
// same GitHub Pages origin as the web app, so no cross-host redirects for the ESP to chase
#define WEB_FW_URL "https://rydonz.github.io/Radio-UI/firmware.bin"

struct Biquad;
struct FilteredPot;
enum SettingType { SET_TOGGLE, SET_VARIABLE, SET_LIST };

struct SettingDef {
  const char *name;
  SettingType type;
  int default_val;
  int min_val;
  int max_val;
  int step;
  const char *unit;
  const char * const *labels;
};

struct FilteredPot {
  float smoothed;
  int output;
  bool connected;
  adc1_channel_t channel;
  int raw_value;
  int raw_min;
  int raw_max;
};
bool update_pot(FilteredPot *pot);
int center_snap(int out);
extern FilteredPot pot_fade;
extern FilteredPot pot_bal;

// --- Pin Definitions (labels match the physical board) ---

// I2S Bus 1 — Front DAC
#define I2S1_BCK   26   // D26
#define I2S1_LRCK  25   // D25
#define I2S1_DOUT  27   // D27

// I2S Bus 2 — Rear DAC
#define I2S2_BCK   14   // D14
#define I2S2_LRCK  12   // D12 — strapping pin (MTDI), but PCM5102 LRCK input is
                        // high-Z and the ESP32 internal pull-down wins at boot,
                        // so flash voltage stays at 3.3V default
#define I2S2_DOUT  13   // D13

// Pots (ADC1 only — ADC2 dead when BT active)
#define PIN_VOLUME  36  // VP
#define PIN_BASS    39  // VN
#define PIN_TREBLE  34  // D34
#define PIN_FADE    32  // D32
#define PIN_BALANCE 35  // D35

// Buttons
#define PIN_PLAY    4   // D4  — volume knob push switch
#define PIN_NEXT    16  // RX2
#define PIN_PREV    17  // TX2
#define PIN_MUTE    15  // D15 — volume 0 detent switch

// OLED (SH1122 256x64 via hardware SPI — VSPI bus)
#define OLED_CLK    18  // D18 — VSPI SCK  (module label: SCL)
#define OLED_DIN    23  // D23 — VSPI MOSI (module label: SDA)
#define OLED_CS      5  // D5  — VSPI SS   (module label: CS)
#define OLED_DC     19  // D19             (module label: DC)
#define OLED_RST    22  // D22             (module label: RST)

// display dimensions
#define DISP_W 256
#define DISP_H 64

// --- Audio Config ---

#define SAMPLE_RATE   44100
#define DMA_BUF_COUNT 8
#define DMA_BUF_LEN   128

// --- Bluetooth ---

BluetoothA2DPSink a2dp_sink;
Preferences prefs;

// --- OLED ---

#include <SPI.h>
U8G2_SH1122_256X64_F_4W_HW_SPI oled(U8G2_R0, OLED_CS, OLED_DC, OLED_RST);
portMUX_TYPE spi_mux = portMUX_INITIALIZER_UNLOCKED;

// HW SPI sendBuffer with interrupt guard — prevents BT/I2S ISRs from
// preempting mid-transfer and corrupting the DC line state
void oled_send() {
  oled.sendBuffer();
}

// --- DSP State ---

volatile float volume = 0.5f;
volatile float bass_gain = 0.0f;      // -1.0 to 1.0 = ±6dB
volatile float treble_gain = 0.0f;    // -1.0 to 1.0 = ±6dB
volatile float fade_val = 0.5f;
volatile float balance_val = 0.5f;

volatile bool bt_connected = false;
volatile bool is_paused = false;
volatile bool is_muted = false;
volatile bool autoplay_pending = false;
unsigned long autoplay_time = 0;
bool pots_initialized = false;

// volume pot soft-takeover: when entering settings the pot gets repurposed for
// nav/edit, so on exit its physical position can be way off from the actual
// music volume. Lock until the pot crosses through the current volume so we
// don't blast the speakers.
bool volume_pot_locked = false;
int volume_pot_last_output = -1;

// settings-driven DSP values (volatile — read by audio callback on BT core)
volatile float stereo_width = 1.0f;   // 0=mono, 1=normal, 2=wide
volatile bool loudness_enabled = false;
volatile float bass_shelf_freq = 200.0f;
volatile float treble_shelf_freq = 3000.0f;
volatile bool rear_lr_swap = false;   // rear DAC's L/R outputs are wired backwards on this build

// biquad filter — second-order IIR, much cleaner than single-pole
struct Biquad {
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2; // delay states
};

// per-channel: bass L/R, treble L/R
Biquad bass_L = {}, bass_R = {};
Biquad treble_L = {}, treble_R = {};

// 10-band graphic EQ — ISO octave centers, one peaking biquad per band per channel
#define EQ_BANDS 10
const float eq_freqs[EQ_BANDS] = {31.0f, 62.0f, 125.0f, 250.0f, 500.0f,
                                  1000.0f, 2000.0f, 4000.0f, 8000.0f, 16000.0f};
const float EQ_Q = 1.41f;   // ~1 octave bandwidth
#define EQ_MAX_DB 12

int8_t eq_gains[EQ_BANDS] = {};
Biquad eq_L[EQ_BANDS] = {}, eq_R[EQ_BANDS] = {};

// makeup gain compensating the EQ's peak boost so boosting a band lowers everything
// else instead of driving the output into the limiter
volatile float eq_makeup = 1.0f;

// fc = center frequency, gain_db = boost/cut in dB, fs = sample rate
void calc_low_shelf(Biquad *bq, float fc, float gain_db, float fs) {
  float A = powf(10.0f, gain_db / 40.0f);
  float w0 = 2.0f * M_PI * fc / fs;
  float cosw = cosf(w0);
  float sinw = sinf(w0);
  float alpha = sinw / 2.0f * sqrtf(2.0f); // Q = 0.707 (Butterworth)
  float a0;

  bq->b0 = A * ((A + 1.0f) - (A - 1.0f) * cosw + 2.0f * sqrtf(A) * alpha);
  bq->b1 = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cosw);
  bq->b2 = A * ((A + 1.0f) - (A - 1.0f) * cosw - 2.0f * sqrtf(A) * alpha);
  a0      = (A + 1.0f) + (A - 1.0f) * cosw + 2.0f * sqrtf(A) * alpha;
  bq->a1  = -2.0f * ((A - 1.0f) + (A + 1.0f) * cosw);
  bq->a2  = (A + 1.0f) + (A - 1.0f) * cosw - 2.0f * sqrtf(A) * alpha;

  bq->b0 /= a0; bq->b1 /= a0; bq->b2 /= a0;
  bq->a1 /= a0; bq->a2 /= a0;
}

void calc_high_shelf(Biquad *bq, float fc, float gain_db, float fs) {
  float A = powf(10.0f, gain_db / 40.0f);
  float w0 = 2.0f * M_PI * fc / fs;
  float cosw = cosf(w0);
  float sinw = sinf(w0);
  float alpha = sinw / 2.0f * sqrtf(2.0f);
  float a0;

  bq->b0 = A * ((A + 1.0f) + (A - 1.0f) * cosw + 2.0f * sqrtf(A) * alpha);
  bq->b1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cosw);
  bq->b2 = A * ((A + 1.0f) + (A - 1.0f) * cosw - 2.0f * sqrtf(A) * alpha);
  a0      = (A + 1.0f) - (A - 1.0f) * cosw + 2.0f * sqrtf(A) * alpha;
  bq->a1  = 2.0f * ((A - 1.0f) - (A + 1.0f) * cosw);
  bq->a2  = (A + 1.0f) - (A - 1.0f) * cosw - 2.0f * sqrtf(A) * alpha;

  bq->b0 /= a0; bq->b1 /= a0; bq->b2 /= a0;
  bq->a1 /= a0; bq->a2 /= a0;
}

// RBJ peaking EQ — used for the 10 graphic bands
void calc_peaking(Biquad *bq, float fc, float gain_db, float Q, float fs) {
  float A = powf(10.0f, gain_db / 40.0f);
  float w0 = 2.0f * M_PI * fc / fs;
  float cosw = cosf(w0);
  float alpha = sinf(w0) / (2.0f * Q);
  float a0;

  bq->b0 = 1.0f + alpha * A;
  bq->b1 = -2.0f * cosw;
  bq->b2 = 1.0f - alpha * A;
  a0     = 1.0f + alpha / A;
  bq->a1 = -2.0f * cosw;
  bq->a2 = 1.0f - alpha / A;

  bq->b0 /= a0; bq->b1 /= a0; bq->b2 /= a0;
  bq->a1 /= a0; bq->a2 /= a0;
}

// Soft-knee limiter replacing the old hard clip. Below the threshold the signal is
// untouched; above it, tanh compresses asymptotically toward 1.0. Continuous in value
// and slope at the knee (d/dx tanh(0) = 1), so there's no discontinuity to buzz on.
// Hard clipping is what made loud/EQ-boosted material harsh — it generates high-order
// odd harmonics; this rolls over instead.
#define LIMIT_T 0.75f
static inline float soft_limit(float x) {
  float a = fabsf(x);
  if (a <= LIMIT_T) return x;
  float y = LIMIT_T + (1.0f - LIMIT_T) * tanhf((a - LIMIT_T) / (1.0f - LIMIT_T));
  return (x < 0.0f) ? -y : y;
}

// frequency grid for the makeup-gain calculation — cos/sin depend only on frequency,
// so they're computed once at boot and reused every time coefficients change
#define EQ_GRID 64
float grid_cw[EQ_GRID], grid_sw[EQ_GRID], grid_c2w[EQ_GRID], grid_s2w[EQ_GRID];

void init_response_grid() {
  for (int i = 0; i < EQ_GRID; i++) {
    float t = (float)i / (EQ_GRID - 1);
    float f = 20.0f * powf(1000.0f, t);            // 20 Hz .. 20 kHz, log spaced
    float w = 2.0f * M_PI * f / 44100.0f;
    grid_cw[i]  = cosf(w);       grid_sw[i]  = sinf(w);
    grid_c2w[i] = cosf(2.0f * w); grid_s2w[i] = sinf(2.0f * w);
  }
}

// |H(e^jw)|^2 for one biquad at grid point i
static inline float biquad_mag2(const Biquad *bq, int i) {
  float nr = bq->b0 + bq->b1 * grid_cw[i] + bq->b2 * grid_c2w[i];
  float ni = -(bq->b1 * grid_sw[i] + bq->b2 * grid_s2w[i]);
  float dr = 1.0f + bq->a1 * grid_cw[i] + bq->a2 * grid_c2w[i];
  float di = -(bq->a1 * grid_sw[i] + bq->a2 * grid_s2w[i]);
  float den = dr * dr + di * di;
  if (den < 1e-20f) den = 1e-20f;
  return (nr * nr + ni * ni) / den;
}

static inline float kill_denormal(float x) {
  union { float f; uint32_t i; } u = {x};
  uint32_t exp = u.i & 0x7F800000;
  if (exp == 0) return 0.0f;           // zero / denormal — flush to dodge FPU stalls
  if (exp == 0x7F800000) return 0.0f;  // NaN / Inf — never let it lodge in the filter
                                       // state, or it regenerates full-scale noise forever
  return x;
}

// coefficients only — delay state stays put so live filters don't click on an update
static inline void copy_coeffs(Biquad *dst, const Biquad *src) {
  dst->b0 = src->b0; dst->b1 = src->b1; dst->b2 = src->b2;
  dst->a1 = src->a1; dst->a2 = src->a2;
}

float biquad_process(Biquad *bq, float in) {
  float out = bq->b0 * in + bq->b1 * bq->x1 + bq->b2 * bq->x2
              - bq->a1 * bq->y1 - bq->a2 * bq->y2;
  bq->x2 = bq->x1; bq->x1 = in;
  bq->y2 = kill_denormal(bq->y1); bq->y1 = out;
  return out;
}

static inline int16_t float_to_i16(float x) {
  // Guard NaN/Inf: the range checks below use `>`/`<`, which are both false for NaN,
  // so an unguarded NaN would skip clamping and cast to full-scale garbage — the
  // "boot static" failure. Force any non-finite sample to silence.
  // (No dither: at 16-bit its only benefit is on near-silent fades, inaudible in a
  //  car, while its ±1 LSB noise floor was audible as a constant hiss.)
  if (!isfinite(x)) return 0;
  float scaled = x * 32767.0f;
  if (scaled > 32767.0f) scaled = 32767.0f;
  if (scaled < -32768.0f) scaled = -32768.0f;
  return (int16_t)scaled;
}

// pre-computed coefficients — main loop calculates, callback copies into live filters
Biquad pending_bass = {}, pending_treble = {};
Biquad pending_eq[EQ_BANDS] = {};
volatile float pending_makeup = 1.0f;

// builds new coefficients into pending biquads — called from loop(), not the audio callback
void update_filters() {
  // ±12 dB, not ±24 — the old range guaranteed clipping past a quarter of pot travel
  float bass_db = bass_gain * 12.0f;

  // loudness: boost bass at low volumes to compensate for Fletcher-Munson
  if (loudness_enabled) {
    float vol_factor = 1.0f - (volume * volume);
    bass_db += vol_factor * 6.0f;
  }

  float treble_db = treble_gain * 12.0f;

  calc_low_shelf(&pending_bass, bass_shelf_freq, bass_db, 44100.0f);
  calc_high_shelf(&pending_treble, treble_shelf_freq, treble_db, 44100.0f);

  for (int b = 0; b < EQ_BANDS; b++)
    calc_peaking(&pending_eq[b], eq_freqs[b], (float)eq_gains[b], EQ_Q, 44100.0f);

  // Find the peak of the graphic-EQ curve and pre-attenuate by it, so shaping the EQ
  // changes tone without changing level.
  //
  // Deliberately excludes the bass/treble shelves: those are physical knobs, and having
  // the volume drop as you turn bass up would feel broken. The knobs stay "boost = louder"
  // and are protected by the soft limiter instead.
  float peak2 = 1.0f;
  for (int i = 0; i < EQ_GRID; i++) {
    float m2 = 1.0f;
    for (int b = 0; b < EQ_BANDS; b++) {
      if (eq_gains[b] == 0) continue;      // unity band, skip the math
      m2 *= biquad_mag2(&pending_eq[b], i);
    }
    if (m2 > peak2) peak2 = m2;
  }
  float mk = 1.0f / sqrtf(peak2);
  pending_makeup = (isfinite(mk) && mk > 0.0f) ? mk : 1.0f;  // never publish a bad gain
}

volatile bool filters_dirty = true;
volatile bool filters_ready = false;
volatile bool audio_reset_pending = false;  // set on BT connect → callback flushes stale DSP state

// zero every biquad's delay line — clears any stuck value (incl. a NaN) and avoids a
// pop from stale filter tails when a new stream starts
void zero_filter_states() {
  Biquad *bqs[4] = {&bass_L, &bass_R, &treble_L, &treble_R};
  for (int i = 0; i < 4; i++) bqs[i]->x1 = bqs[i]->x2 = bqs[i]->y1 = bqs[i]->y2 = 0.0f;
  for (int b = 0; b < EQ_BANDS; b++) {
    eq_L[b].x1 = eq_L[b].x2 = eq_L[b].y1 = eq_L[b].y2 = 0.0f;
    eq_R[b].x1 = eq_R[b].x2 = eq_R[b].y1 = eq_R[b].y2 = 0.0f;
  }
}

// DSP cost as a percentage of realtime, reported over BLE — there's no USB port fitted,
// so this is the only way to see whether the EQ is keeping up
volatile uint16_t dsp_load_pct = 0;

// --- Track Metadata ---

char track_title[128] = "";
char track_artist[128] = "";
volatile bool metadata_changed = true;
volatile uint32_t track_duration_ms = 0;
volatile uint32_t track_position_ms = 0;
unsigned long position_sync_time = 0;

#define DISP_TITLE          0
#define DISP_ARTIST         1
#define DISP_TITLE_ARTIST   2
#define DISP_PROGRESS       3

const int top_line_map[] = {DISP_TITLE, DISP_ARTIST, DISP_TITLE_ARTIST};
const int bottom_line_map[] = {DISP_ARTIST, DISP_TITLE, DISP_PROGRESS, DISP_TITLE_ARTIST};

// --- Display State ---

unsigned long last_pot_read = 0;
unsigned long last_display_update = 0;
unsigned long overlay_until = 0;
unsigned long overlay_suppress_until = 0;
int scroll_offset = 0;
unsigned long last_scroll = 0;
unsigned long scroll_pause_until = 0;
int scroll_wrap_at = 0;
const char *overlay_label = NULL;
float overlay_value = 0.0f;
enum OverlayType : uint8_t { OV_PERCENT, OV_CENTERED };
OverlayType overlay_type = OV_PERCENT;
const char *overlay_left_label = "";
const char *overlay_right_label = "";
bool display_dirty = true;

// --- Buttons ---

bool last_play = HIGH;
bool last_next = HIGH;
bool last_prev = HIGH;
bool last_mute_state = false;
unsigned long last_btn_time = 0;

unsigned long play_hold_start = 0;
bool play_held = false;

// --- Settings ---

#define SET_BRIGHTNESS    0
#define SET_LOUDNESS      1
#define SET_BASS_FREQ     2
#define SET_TREBLE_FREQ   3
#define SET_TOP_LINE      4
#define SET_BOTTOM_LINE   5
#define SET_AUTOPLAY      6
#define SET_SEPARATOR     7
#define SET_SCROLL_PAUSE  8
#define SET_AUTO_RECONN   9
#define SET_BOOT_ANIM     10
#define SET_BT_NAME       11
#define SET_SCROLL_SPEED  12
#define SET_OVERLAY_TIME  13
#define SET_STEREO_WIDTH  14
#define SET_FADE_SWAP     15
#define SET_BAL_SWAP      16
#define SET_REAR_LR_SWAP  17
#define SET_POT_DEADZONE  18
#define SET_OTA_UPDATE    19
#define SET_FACTORY_RESET 20
#define SET_DEBUG_VIEW    21
#define SET_CALIBRATE     22
#define SET_EQ_FLAT       23

const char * const bt_name_options[] = {"DelSol", "DelSol Radio", "The Ultra Custom Bestest DelSol Head Unit Of All Time Or Something Like That", "Honda DelSol VTEC"};
const char * const top_line_options[] = {"Title", "Artist", "Title-Artist"};
const char * const bottom_line_options[] = {"Artist", "Title", "Progress", "Title-Artist"};

const SettingDef settings_defs[] = {
  {"Brightness", SET_VARIABLE, 100,  0,    100,  1,    "%",  NULL},
  {"Loudness",   SET_TOGGLE,   0,    0,    1,    1,    NULL, NULL},
  {"Bass Freq",  SET_VARIABLE, 200,  80,   300,  10,   "Hz", NULL},
  {"Treb Freq",  SET_VARIABLE, 3000, 1000, 5000, 100,  "Hz", NULL},
  {"Top Line",   SET_LIST,     0,    0,    2,    1,    NULL, top_line_options},
  {"Bottom Ln",  SET_LIST,     2,    0,    3,    1,    NULL, bottom_line_options},
  {"Autoplay",   SET_TOGGLE,   1,    0,    1,    1,    NULL, NULL},
  {"Separator",  SET_TOGGLE,   1,    0,    1,    1,    NULL, NULL},
  {"Scrl Pause", SET_VARIABLE, 3000, 0,    5000, 500,  "ms", NULL},
  {"Auto Recon", SET_TOGGLE,   1,    0,    1,    1,    NULL, NULL},
  {"Boot Anim",  SET_TOGGLE,   1,    0,    1,    1,    NULL, NULL},
  {"BT Name",    SET_LIST,     0,    0,    3,    1,    NULL, bt_name_options},
  {"Scroll Spd", SET_VARIABLE, 150,  10,  600,  10,   "ms", NULL},
  {"Overlay Tm", SET_VARIABLE, 1500, 500,  5000, 100,  "ms", NULL},
  {"Stereo W.",  SET_VARIABLE, 50,   0,    100,  1,    "%",  NULL},
  {"Fade Swap",  SET_TOGGLE,   0,    0,    1,    1,    NULL, NULL},
  {"Bal Swap",   SET_TOGGLE,   0,    0,    1,    1,    NULL, NULL},
  {"RearLR Swp", SET_TOGGLE,   1,    0,    1,    1,    NULL, NULL},
  {"Pot DeadZn", SET_VARIABLE, 2,    0,    5,    1,    NULL, NULL},
  {"OTA Update", SET_TOGGLE,   0,    0,    1,    1,    NULL, NULL},
  {"Reset All",  SET_TOGGLE,   0,    0,    1,    1,    NULL, NULL},
  {"Debug View", SET_TOGGLE,   0,    0,    1,    1,    NULL, NULL},
  {"Calibrate",  SET_TOGGLE,   0,    0,    1,    1,    NULL, NULL},
  {"EQ Flat",    SET_TOGGLE,   0,    0,    1,    1,    NULL, NULL},
};
#define SETTINGS_COUNT (sizeof(settings_defs) / sizeof(settings_defs[0]))

int settings_values[SETTINGS_COUNT];

bool settings_mode = false;
int settings_page = 0;
bool settings_editing = false;

void apply_setting(int page);
void save_setting(int page);

void save_setting(int page) {
  if (page == SET_FACTORY_RESET || page == SET_OTA_UPDATE || page == SET_CALIBRATE ||
      page == SET_EQ_FLAT) return;
  char key[4];
  snprintf(key, 4, "s%d", page);
  prefs.putInt(key, settings_values[page]);
}

void save_calibration();
void load_calibration();
void clear_calibration();
void reset_pot_observations();

void apply_setting(int page) {
  switch (page) {
    case SET_BRIGHTNESS:
      oled.setContrast(settings_values[SET_BRIGHTNESS] * 255 / 100);
      break;
    case SET_LOUDNESS:
      loudness_enabled = settings_values[SET_LOUDNESS];
      filters_dirty = true;
      break;
    case SET_STEREO_WIDTH:
      stereo_width = settings_values[SET_STEREO_WIDTH] / 50.0f;
      break;
    case SET_BASS_FREQ:
      bass_shelf_freq = (float)settings_values[SET_BASS_FREQ];
      filters_dirty = true;
      break;
    case SET_TREBLE_FREQ:
      treble_shelf_freq = (float)settings_values[SET_TREBLE_FREQ];
      filters_dirty = true;
      break;
    case SET_FADE_SWAP: {
      float fv = (100 - center_snap(pot_fade.output)) / 100.0f;
      if (settings_values[SET_FADE_SWAP]) fv = 1.0f - fv;
      fade_val = fv;
      break;
    }
    case SET_BAL_SWAP: {
      float bv = (100 - center_snap(pot_bal.output)) / 100.0f;
      if (settings_values[SET_BAL_SWAP]) bv = 1.0f - bv;
      balance_val = bv;
      break;
    }
    case SET_REAR_LR_SWAP:
      rear_lr_swap = settings_values[SET_REAR_LR_SWAP];
      break;
    case SET_OTA_UPDATE:
      if (settings_values[SET_OTA_UPDATE]) {
        settings_values[SET_OTA_UPDATE] = 0;
        enter_ota_mode();
      }
      break;
    case SET_FACTORY_RESET:
      if (settings_values[SET_FACTORY_RESET]) {
        clear_calibration();
        for (int i = 0; i < (int)SETTINGS_COUNT; i++) {
          settings_values[i] = settings_defs[i].default_val;
          save_setting(i);
        }
        for (int i = 0; i < (int)SETTINGS_COUNT; i++)
          apply_setting(i);
      }
      break;
    // on-unit escape hatch: zero the EQ without needing the phone
    case SET_EQ_FLAT:
      if (settings_values[SET_EQ_FLAT]) {
        settings_values[SET_EQ_FLAT] = 0;
        memset(eq_gains, 0, sizeof(eq_gains));
        filters_dirty = true;
        save_eq();
      }
      break;
    case SET_CALIBRATE: {
      static bool was_calibrating = false;
      bool now = settings_values[SET_CALIBRATE];
      if (now && !was_calibrating) {
        // start fresh sweep — clear in-memory observations only, keep
        // saved cal in NVS until the user explicitly toggles back off
        reset_pot_observations();
      } else if (!now && was_calibrating) {
        save_calibration();
        // pots that weren't swept stay at sentinels; reload to restore
        // their previously-saved cal so they don't fall back to defaults
        load_calibration();
      }
      was_calibrating = now;
      break;
    }
  }
}

void init_settings() {
  for (int i = 0; i < (int)SETTINGS_COUNT; i++) {
    char key[4];
    snprintf(key, 4, "s%d", i);
    int v = prefs.getInt(key, settings_defs[i].default_val);
    if (v < settings_defs[i].min_val || v > settings_defs[i].max_val)
      v = settings_defs[i].default_val;
    settings_values[i] = v;
  }
  settings_values[SET_OTA_UPDATE] = 0;
  settings_values[SET_FACTORY_RESET] = 0;
  settings_values[SET_CALIBRATE] = 0;
  settings_values[SET_EQ_FLAT] = 0;
  for (int i = 0; i < (int)SETTINGS_COUNT; i++)
    apply_setting(i);
}

// --- I2S Setup ---

void init_i2s(i2s_port_t port, int bck, int lrck, int dout) {
  i2s_config_t config = {};
  config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  config.sample_rate = SAMPLE_RATE;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  config.dma_buf_count = DMA_BUF_COUNT;
  config.dma_buf_len = DMA_BUF_LEN;
  config.use_apll = true;
  config.tx_desc_auto_clear = true;

  esp_err_t err = i2s_driver_install(port, &config, 0, NULL);
  if (err != ESP_OK) Serial.printf("i2s_driver_install(%d) failed: %d\n", port, err);

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = bck;
  pins.ws_io_num = lrck;
  pins.data_out_num = dout;
  pins.data_in_num = I2S_PIN_NO_CHANGE;
  err = i2s_set_pin(port, &pins);
  if (err != ESP_OK) Serial.printf("i2s_set_pin(%d) failed: %d\n", port, err);

  i2s_zero_dma_buffer(port);
  i2s_stop(port);
}

// --- Connection State Callback ---

void connection_state_cb(esp_a2d_connection_state_t state, void *ptr) {
  if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
    i2s_zero_dma_buffer(I2S_NUM_0);
    i2s_zero_dma_buffer(I2S_NUM_1);
    i2s_start(I2S_NUM_0);
    i2s_start(I2S_NUM_1);
    audio_reset_pending = true;   // fresh stream: clear any stale/garbage DSP state
    bt_connected = true;
    if (settings_values[SET_AUTOPLAY]) {
      is_paused = false;
      autoplay_pending = true;
      autoplay_time = millis();
    } else {
      is_paused = true;
    }
    Serial.println("BT connected — ramping volume");
  } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
    bt_connected = false;
    track_title[0] = '\0';
    track_artist[0] = '\0';
    track_duration_ms = 0;
    track_position_ms = 0;
    metadata_changed = true;
    i2s_zero_dma_buffer(I2S_NUM_0);
    i2s_zero_dma_buffer(I2S_NUM_1);
    i2s_stop(I2S_NUM_0);
    i2s_stop(I2S_NUM_1);
    Serial.println("BT disconnected — muted");
  }
}

// --- Audio Callback (runs on BT task, called by A2DP library) ---

void audio_data_callback(const uint8_t *data, uint32_t length) {
  const TickType_t I2S_TIMEOUT = pdMS_TO_TICKS(50);

  if (!bt_connected || is_paused || is_muted) {
    int16_t silence[DMA_BUF_LEN * 2] = {};
    size_t w;
    for (uint32_t off = 0; off < length / 4; off += DMA_BUF_LEN) {
      uint32_t chunk = min((uint32_t)DMA_BUF_LEN, length / 4 - off);
      i2s_write(I2S_NUM_0, silence, chunk * 4, &w, I2S_TIMEOUT);
      i2s_write(I2S_NUM_1, silence, chunk * 4, &w, I2S_TIMEOUT);
    }
    return;
  }


  int16_t *samples = (int16_t *)data;
  uint32_t frame_count = length / 4;

  if (filters_ready) {
    copy_coeffs(&bass_L, &pending_bass);
    copy_coeffs(&bass_R, &pending_bass);
    copy_coeffs(&treble_L, &pending_treble);
    copy_coeffs(&treble_R, &pending_treble);
    for (int b = 0; b < EQ_BANDS; b++) {
      copy_coeffs(&eq_L[b], &pending_eq[b]);
      copy_coeffs(&eq_R[b], &pending_eq[b]);
    }
    eq_makeup = pending_makeup;
    filters_ready = false;
  }

  float mk = eq_makeup;
  if (!isfinite(mk) || mk <= 0.0f) mk = 1.0f;   // defend against a torn/bad makeup value
  float target_vol = volume * volume * volume * 0.6f * mk;
  float fv = fade_val;
  float bv = balance_val;
  float sw = stereo_width;

  float target_front = (fv <= 0.5f) ? 1.0f : 2.0f * (1.0f - fv);
  float target_rear  = (fv >= 0.5f) ? 1.0f : 2.0f * fv;
  float target_left  = (bv <= 0.5f) ? 1.0f : 2.0f * (1.0f - bv);
  float target_right = (bv >= 0.5f) ? 1.0f : 2.0f * bv;

  // per-sample smoothing to avoid clicks from sudden gain changes (~3ms time constant)
  static float s_vol = 0.0f, s_front = 1.0f, s_rear = 1.0f;
  static float s_left = 1.0f, s_right = 1.0f;
  const float GAIN_SMOOTH = 0.007f;

  // fresh stream, or a stuck NaN that would otherwise persist across callbacks: flush the
  // smoothing state and the filter delay lines so audio starts clean and self-heals.
  if (audio_reset_pending || !isfinite(s_vol)) {
    audio_reset_pending = false;
    s_vol = 0.0f; s_front = 1.0f; s_rear = 1.0f; s_left = 1.0f; s_right = 1.0f;
    zero_filter_states();
  }

  int16_t buf_front[DMA_BUF_LEN * 2];
  int16_t buf_rear[DMA_BUF_LEN * 2];

  int64_t dsp_us = 0;

  uint32_t offset = 0;
  while (offset < frame_count) {
    uint32_t chunk = frame_count - offset;
    if (chunk > DMA_BUF_LEN) chunk = DMA_BUF_LEN;

    int64_t t0 = esp_timer_get_time();

    for (uint32_t i = 0; i < chunk; i++) {
      s_vol   += GAIN_SMOOTH * (target_vol - s_vol);
      s_front += GAIN_SMOOTH * (target_front - s_front);
      s_rear  += GAIN_SMOOTH * (target_rear - s_rear);
      s_left  += GAIN_SMOOTH * (target_left - s_left);
      s_right += GAIN_SMOOTH * (target_right - s_right);

      uint32_t idx = (offset + i) * 2;
      float L = samples[idx] / 32768.0f;
      float R = samples[idx + 1] / 32768.0f;

      L = biquad_process(&bass_L, L);
      L = biquad_process(&treble_L, L);
      R = biquad_process(&bass_R, R);
      R = biquad_process(&treble_R, R);

      for (int b = 0; b < EQ_BANDS; b++) {
        L = biquad_process(&eq_L[b], L);
        R = biquad_process(&eq_R[b], R);
      }

      if (sw != 1.0f) {
        float mid = (L + R) * 0.5f;
        float side = (L - R) * 0.5f;
        side *= sw;
        L = mid + side;
        R = mid - side;
      }

      L = soft_limit(L * s_vol);
      R = soft_limit(R * s_vol);

      float Lb = L * s_left;
      float Rb = R * s_right;

      buf_front[i * 2]     = float_to_i16(Lb * s_front);
      buf_front[i * 2 + 1] = float_to_i16(Rb * s_front);
      if (rear_lr_swap) {
        buf_rear[i * 2]      = float_to_i16(Rb * s_rear);
        buf_rear[i * 2 + 1]  = float_to_i16(Lb * s_rear);
      } else {
        buf_rear[i * 2]      = float_to_i16(Lb * s_rear);
        buf_rear[i * 2 + 1]  = float_to_i16(Rb * s_rear);
      }
    }

    // timed before i2s_write — that call blocks on DMA backpressure, which is idle
    // waiting rather than CPU cost and would inflate the figure
    dsp_us += esp_timer_get_time() - t0;

    size_t written;
    i2s_write(I2S_NUM_0, buf_front, chunk * 4, &written, I2S_TIMEOUT);
    i2s_write(I2S_NUM_1, buf_rear, chunk * 4, &written, I2S_TIMEOUT);

    offset += chunk;
  }

  if (frame_count) {
    uint32_t audio_us = (uint32_t)((uint64_t)frame_count * 1000000ULL / SAMPLE_RATE);
    uint32_t load = (uint32_t)(dsp_us * 100 / (audio_us ? audio_us : 1));
    dsp_load_pct = dsp_load_pct ? (uint16_t)((dsp_load_pct * 7 + load) / 8) : (uint16_t)load;
  }
}

// --- AVRCP Metadata Callback ---

void avrc_metadata_cb(uint8_t id, const uint8_t *text) {
  switch (id) {
    case ESP_AVRC_MD_ATTR_TITLE:
      strncpy(track_title, (const char *)text, sizeof(track_title) - 1);
      track_title[sizeof(track_title) - 1] = '\0';
      metadata_changed = true;
      break;
    case ESP_AVRC_MD_ATTR_ARTIST:
      strncpy(track_artist, (const char *)text, sizeof(track_artist) - 1);
      track_artist[sizeof(track_artist) - 1] = '\0';
      metadata_changed = true;
      break;
    case ESP_AVRC_MD_ATTR_PLAYING_TIME:
      track_duration_ms = atol((const char *)text);
      break;
  }
}

void avrc_rn_playstatus_cb(esp_avrc_playback_stat_t playback) {
  if (playback == ESP_AVRC_PLAYBACK_PLAYING) {
    is_paused = false;
  } else if (playback == ESP_AVRC_PLAYBACK_PAUSED || playback == ESP_AVRC_PLAYBACK_STOPPED) {
    is_paused = true;
  }
}

void avrc_play_pos_cb(uint32_t pos_ms) {
  track_position_ms = pos_ms;
  position_sync_time = millis();
}

// --- Pot Reading ---

void show_overlay(const char *label, float val) {
  if (millis() < overlay_suppress_until) return;
  overlay_label = label;
  overlay_value = val;
  overlay_type = OV_PERCENT;
  overlay_until = millis() + settings_values[SET_OVERLAY_TIME];
}

void show_overlay_centered(const char *label, float val, const char *l, const char *r) {
  if (millis() < overlay_suppress_until) return;
  overlay_label = label;
  overlay_value = val;
  overlay_type = OV_CENTERED;
  overlay_left_label = l;
  overlay_right_label = r;
  overlay_until = millis() + settings_values[SET_OVERLAY_TIME];
}

// ESP32 ADC doesn't reach true 0 or 4095 — remap the usable range to 0.0-1.0
#define ADC_MIN 100
#define ADC_MAX 3950

int read_adc_raw(adc1_channel_t ch) {
  int sum = 0;
  for (int i = 0; i < 16; i++) sum += adc1_get_raw(ch);
  return sum / 16;
}

float read_adc(adc1_channel_t ch) {
  int raw = read_adc_raw(ch);
  if (raw <= ADC_MIN) return 1.0f;
  if (raw >= ADC_MAX) return 0.0f;
  return 1.0f - (float)(raw - ADC_MIN) / (float)(ADC_MAX - ADC_MIN);
}

const bool POT_CONNECTED_VOLUME  = true;
const bool POT_CONNECTED_BASS    = true;
const bool POT_CONNECTED_TREBLE  = true;
const bool POT_CONNECTED_FADE    = true;
const bool POT_CONNECTED_BALANCE = true;

const float EMA_ALPHA = 0.12f;

FilteredPot pot_vol  = {0.5f, 50, true,  ADC1_CHANNEL_0, 0, 4096, -1};
FilteredPot pot_bass = {0.5f, 50, true,  ADC1_CHANNEL_3, 0, 4096, -1};
FilteredPot pot_treb = {0.5f, 50, true,  ADC1_CHANNEL_6, 0, 4096, -1};
FilteredPot pot_fade = {0.5f, 50, true,  ADC1_CHANNEL_4, 0, 4096, -1};
FilteredPot pot_bal  = {0.5f, 50, true,  ADC1_CHANNEL_7, 0, 4096, -1};

bool update_pot(FilteredPot *pot) {
  if (!pot->connected) return false;
  pot->raw_value = read_adc_raw(pot->channel);

  // observe extremes only during explicit calibration — keeps the saved
  // range stable during normal use
  if (settings_values[SET_CALIBRATE]) {
    if (pot->raw_value < pot->raw_min) pot->raw_min = pot->raw_value;
    if (pot->raw_value > pot->raw_max) pot->raw_max = pot->raw_value;
  }

  // use the calibrated range with a small inset so jitter at the physical
  // extremes still clamps cleanly to 0/100. Fall back to hardcoded defaults
  // if no calibration has been performed yet.
  int lo = ADC_MIN, hi = ADC_MAX;
  if (pot->raw_max > pot->raw_min + 20) {
    lo = pot->raw_min + 5;
    hi = pot->raw_max - 5;
  }

  float raw;
  if (pot->raw_value <= lo) raw = 1.0f;
  else if (pot->raw_value >= hi) raw = 0.0f;
  else raw = 1.0f - (float)(pot->raw_value - lo) / (float)(hi - lo);
  pot->smoothed += EMA_ALPHA * (raw - pot->smoothed);
  int quantized = (int)(pot->smoothed * 100.0f + 0.5f);
  if (quantized < 0) quantized = 0;
  if (quantized > 100) quantized = 100;
  if (quantized == pot->output) return false;
  if (quantized == 0 || quantized == 100 ||
      abs(quantized - pot->output) > settings_values[SET_POT_DEADZONE]) {
    pot->output = quantized;
    return true;
  }
  return false;
}

void init_pots() {
  FilteredPot *pots[] = {&pot_vol, &pot_bass, &pot_treb, &pot_fade, &pot_bal};
  for (int i = 0; i < 64; i++) {
    for (int p = 0; p < 5; p++) {
      if (pots[p]->connected) {
        float raw = read_adc(pots[p]->channel);
        pots[p]->smoothed += 0.5f * (raw - pots[p]->smoothed);
      }
    }
  }
  for (int p = 0; p < 5; p++) {
    pots[p]->output = (int)(pots[p]->smoothed * 100.0f + 0.5f);
  }
  volume = pot_vol.output / 100.0f;
  bass_gain = (center_snap(pot_bass.output) / 50.0f) - 1.0f;
  treble_gain = (center_snap(pot_treb.output) / 50.0f) - 1.0f;
  fade_val = (100 - center_snap(pot_fade.output)) / 100.0f;
  if (settings_values[SET_FADE_SWAP]) fade_val = 1.0f - fade_val;
  balance_val = (100 - center_snap(pot_bal.output)) / 100.0f;
  if (settings_values[SET_BAL_SWAP]) balance_val = 1.0f - balance_val;
  pots_initialized = true;
}

// pot calibration NVS keys — kept terse to fit Preferences' 15-char limit
static const char *cal_lo_keys[5] = {"cVlo", "cBlo", "cTlo", "cFlo", "cAlo"};
static const char *cal_hi_keys[5] = {"cVhi", "cBhi", "cThi", "cFhi", "cAhi"};

FilteredPot *all_pots[5] = {&pot_vol, &pot_bass, &pot_treb, &pot_fade, &pot_bal};

void save_calibration() {
  for (int i = 0; i < 5; i++) {
    if (all_pots[i]->raw_min < all_pots[i]->raw_max) {
      prefs.putInt(cal_lo_keys[i], all_pots[i]->raw_min);
      prefs.putInt(cal_hi_keys[i], all_pots[i]->raw_max);
    }
  }
}

void load_calibration() {
  for (int i = 0; i < 5; i++) {
    int lo = prefs.getInt(cal_lo_keys[i], -1);
    int hi = prefs.getInt(cal_hi_keys[i], -1);
    if (lo >= 0 && hi > lo) {
      all_pots[i]->raw_min = lo;
      all_pots[i]->raw_max = hi;
    }
  }
}

void clear_calibration() {
  for (int i = 0; i < 5; i++) {
    prefs.remove(cal_lo_keys[i]);
    prefs.remove(cal_hi_keys[i]);
    all_pots[i]->raw_min = 4096;
    all_pots[i]->raw_max = -1;
  }
}

void reset_pot_observations() {
  for (int i = 0; i < 5; i++) {
    all_pots[i]->raw_min = 4096;
    all_pots[i]->raw_max = -1;
  }
}

// snaps a 0-100 pot output to 50 within a center deadband. Bass/treble/fade/
// balance pots physically center near 52-54 instead of 50, so a small band
// around center forces them to exactly mid.
int center_snap(int out) {
  if (out >= 45 && out <= 55) return 50;
  return out;
}

int snap_to_step(const SettingDef *def, int raw) {
  if (def->step <= 1) return raw;
  int steps = (raw - def->min_val + def->step / 2) / def->step;
  int snapped = def->min_val + steps * def->step;
  if (snapped < def->min_val) snapped = def->min_val;
  if (snapped > def->max_val) snapped = def->max_val;
  return snapped;
}

void read_pots() {
  if (update_pot(&pot_vol)) {
    if (settings_mode && settings_editing) {
      const SettingDef *def = &settings_defs[settings_page];
      int range = def->max_val - def->min_val;
      int raw_val = def->min_val + (pot_vol.output * range / 100);
      int new_val = snap_to_step(def, raw_val);
      if (new_val != settings_values[settings_page]) {
        settings_values[settings_page] = new_val;
        apply_setting(settings_page);
        save_setting(settings_page);
        display_dirty = true;
      }
    } else if (settings_mode) {
      int new_page = pot_vol.output * ((int)SETTINGS_COUNT - 1) / 100;
      if (new_page != settings_page) {
        settings_page = new_page;
        display_dirty = true;
      }
    } else {
      if (volume_pot_locked) {
        int target = (int)(volume * 100.0f + 0.5f);
        int prev = volume_pot_last_output;
        int curr = pot_vol.output;
        bool crossed = (prev >= 0) &&
                       ((prev <= target && curr >= target) ||
                        (prev >= target && curr <= target));
        if (crossed) {
          volume_pot_locked = false;
        } else {
          show_overlay("Locked", volume);
        }
      }
      if (!volume_pot_locked) {
        volume = pot_vol.output / 100.0f;
        if (loudness_enabled) filters_dirty = true;
        show_overlay("Volume", volume);
      }
    }
    volume_pot_last_output = pot_vol.output;
  }
  if (update_pot(&pot_bass)) {
    int snapped = center_snap(pot_bass.output);
    bass_gain = (snapped / 50.0f) - 1.0f;
    filters_dirty = true;
    show_overlay("Bass", snapped / 100.0f);
  }
  if (update_pot(&pot_treb)) {
    int snapped = center_snap(pot_treb.output);
    treble_gain = (snapped / 50.0f) - 1.0f;
    filters_dirty = true;
    show_overlay("Treble", snapped / 100.0f);
  }
  if (update_pot(&pot_fade)) {
    int snapped = center_snap(pot_fade.output);
    float fv = (100 - snapped) / 100.0f;
    if (settings_values[SET_FADE_SWAP]) fv = 1.0f - fv;
    fade_val = fv;
    show_overlay_centered("Fade", fv, "F", "R");
  }
  if (update_pot(&pot_bal)) {
    int snapped = center_snap(pot_bal.output);
    float bv = (100 - snapped) / 100.0f;
    if (settings_values[SET_BAL_SWAP]) bv = 1.0f - bv;
    balance_val = bv;
    show_overlay_centered("Balance", bv, "L", "R");
  }
}

// --- Settings Menu ---

void enter_settings() {
  settings_mode = true;
  settings_page = 0;
  settings_editing = false;
  volume_pot_locked = true;
  volume_pot_last_output = pot_vol.output;
  display_dirty = true;
}

void exit_settings() {
  settings_mode = false;
  settings_editing = false;
  int target = (int)(volume * 100.0f + 0.5f);
  if (pot_vol.output == target) volume_pot_locked = false;
  display_dirty = true;
}

// --- Button Handling ---

void process_buttons() {
  // standby: volume knob at zero detent leaves PIN_MUTE HIGH (switch opens at
  // zero, pullup wins). On entry: pause BT and blank the display. On exit:
  // wake the display and resume playback.
  bool mute_now = (digitalRead(PIN_MUTE) == HIGH);
  if (mute_now != last_mute_state) {
    last_mute_state = mute_now;
    if (mute_now) {
      if (bt_connected) a2dp_sink.pause();
      oled.setPowerSave(1);
    } else {
      oled.setPowerSave(0);
      if (bt_connected) a2dp_sink.play();
      display_dirty = true;
    }
  }
  is_muted = mute_now;
  if (is_muted) return;

  bool play_state = digitalRead(PIN_PLAY);
  bool next_state = digitalRead(PIN_NEXT);
  bool prev_state = digitalRead(PIN_PREV);
  unsigned long now = millis();

  if (play_state == LOW) {
    if (play_hold_start == 0) play_hold_start = now;
    if (!play_held && now - play_hold_start >= 2000) {
      play_held = true;
      if (settings_mode) exit_settings(); else enter_settings();
    }
  } else {
    if (play_hold_start > 0 && !play_held && now - last_btn_time >= 50) {
      if (settings_mode) {
        const SettingDef *def = &settings_defs[settings_page];
        if (def->type == SET_TOGGLE) {
          settings_values[settings_page] = !settings_values[settings_page];
          apply_setting(settings_page);
          save_setting(settings_page);
        } else if (def->type == SET_LIST) {
          settings_editing = !settings_editing;
        } else {
          settings_editing = !settings_editing;
        }
        display_dirty = true;
      } else {
        is_paused = !is_paused;
        if (is_paused) a2dp_sink.pause(); else a2dp_sink.play();
        display_dirty = true;
      }
      last_btn_time = now;
    }
    play_hold_start = 0;
    play_held = false;
  }

  if (now - last_btn_time >= 50) {
    if (next_state == LOW && last_next == HIGH) {
      if (settings_mode) {
        if (settings_editing)
          settings_adjust(1);
        else if (settings_page < (int)SETTINGS_COUNT - 1)
          settings_page++;
        display_dirty = true;
      } else {
        is_paused = false;
        a2dp_sink.next();
        scroll_offset = 0;
        scroll_pause_until = now + settings_values[SET_SCROLL_PAUSE];
        overlay_until = 0;
        overlay_suppress_until = now + 300;
        display_dirty = true;
      }
      last_btn_time = now;
    }

    if (prev_state == LOW && last_prev == HIGH) {
      if (settings_mode) {
        if (settings_editing)
          settings_adjust(-1);
        else if (settings_page > 0)
          settings_page--;
        display_dirty = true;
      } else {
        is_paused = false;
        a2dp_sink.previous();
        scroll_offset = 0;
        scroll_pause_until = now + settings_values[SET_SCROLL_PAUSE];
        overlay_until = 0;
        overlay_suppress_until = now + 300;
        display_dirty = true;
      }
      last_btn_time = now;
    }
  }

  last_play = play_state;
  last_next = next_state;
  last_prev = prev_state;
}

void settings_adjust(int delta) {
  const SettingDef *def = &settings_defs[settings_page];
  int val = settings_values[settings_page] + (delta * def->step);
  if (val < def->min_val) val = def->min_val;
  if (val > def->max_val) val = def->max_val;
  settings_values[settings_page] = val;
  apply_setting(settings_page);
  save_setting(settings_page);
}

// --- Display Drawing ---

// pixel-based progress bar, rounded ends
void draw_bar(int x, int y, int w, int h, float val) {
  if (val < 0.0f) val = 0.0f;
  if (val > 1.0f) val = 1.0f;
  int r = h / 2;
  oled.drawRFrame(x, y, w, h, r);
  int inner_h = h - 4;
  int inner_w = (int)((w - 4) * val);
  if (inner_h < 1 || inner_w < inner_h) {
    // too small for rounded box — draw a plain filled rect instead
    if (inner_w >= 1)
      oled.drawBox(x + 2, y + 2, inner_w, inner_h);
    return;
  }
  int ir = inner_h / 2;
  oled.drawRBox(x + 2, y + 2, inner_w, inner_h, ir);
}

void format_time(uint32_t ms, char *buf, int bufsize) {
  int secs = ms / 1000;
  int m = secs / 60;
  int s = secs % 60;
  snprintf(buf, bufsize, "%d:%02d", m, s);
}

uint32_t get_current_position() {
  if (position_sync_time == 0) return 0;
  if (is_paused) return track_position_ms;
  uint32_t elapsed = millis() - position_sync_time;
  uint32_t pos = track_position_ms + elapsed;
  if (track_duration_ms > 0 && pos > track_duration_ms) pos = track_duration_ms;
  return pos;
}

void build_title_artist(char *buf, int bufsize) {
  if (track_artist[0]) {
    snprintf(buf, bufsize, "%s - %s", track_title, track_artist);
  } else {
    strncpy(buf, track_title, bufsize - 1);
    buf[bufsize - 1] = '\0';
  }
}

// returns the pixel width of text that fits within max_w, applying scroll offset
// draws scrolling text at (x,y) clipped to max_w pixels
void draw_scrolling_text(int x, int y, int max_w, const char *text) {
  int text_w = oled.getStrWidth(text);
  if (text_w <= max_w) {
    oled.drawStr(x, y, text);
    return;
  }
  // " ~ " separator between wrapping copies
  const char *sep = "   ";
  int sep_w = oled.getStrWidth(sep);
  int total_w = text_w + sep_w;
  if (total_w > scroll_wrap_at) scroll_wrap_at = total_w;
  int off = scroll_offset % total_w;

  // build a wide enough string from the repeated pattern and draw clipped
  oled.setClipWindow(x, 0, x + max_w - 1, DISP_H - 1);
  oled.drawStr(x - off, y, text);
  oled.drawStr(x - off + text_w, y, sep);
  oled.drawStr(x - off + total_w, y, text);
  oled.setMaxClipWindow();
}

void draw_overlay() {
  oled.setFont(u8g2_font_helvB18_tr);
  oled.drawStr(4, 24, overlay_label);

  if (overlay_type == OV_CENTERED) {
    float diff = overlay_value - 0.5f;
    int pct = (int)(fabsf(diff) * 200.0f + 0.5f);
    if (pct > 100) pct = 100;

    char info[16];
    if (pct < 3) {
      snprintf(info, sizeof(info), "Center");
    } else {
      const char *side = (diff < 0) ? overlay_left_label : overlay_right_label;
      snprintf(info, sizeof(info), "%s %d%%", side, pct);
    }
    oled.setFont(u8g2_font_helvR12_tr);
    int iw = oled.getStrWidth(info);
    oled.drawStr(DISP_W - iw - 6, 24, info);

    oled.setFont(u8g2_font_helvR10_tr);
    int lw = oled.getStrWidth(overlay_left_label);
    int rw = oled.getStrWidth(overlay_right_label);
    int bar_y = 36, bar_h = 18;
    int bar_x = 4 + lw + 6;
    int bar_w = DISP_W - 8 - lw - rw - 12;

    oled.drawStr(4, bar_y + 14, overlay_left_label);
    oled.drawStr(DISP_W - rw - 4, bar_y + 14, overlay_right_label);

    int r = bar_h / 2;
    oled.drawRFrame(bar_x, bar_y, bar_w, bar_h, r);

    int center_x = bar_x + bar_w / 2;
    oled.drawVLine(center_x, bar_y + 2, bar_h - 4);

    int half_inner = (bar_w - 4) / 2;
    int fill_w = (int)(fabsf(diff) * 2.0f * half_inner);
    if (fill_w > half_inner) fill_w = half_inner;
    if (fill_w >= 1) {
      if (diff < 0) {
        oled.drawBox(center_x - fill_w, bar_y + 2, fill_w, bar_h - 4);
      } else {
        oled.drawBox(center_x + 1, bar_y + 2, fill_w, bar_h - 4);
      }
    }
  } else {
    char pct[8];
    snprintf(pct, sizeof(pct), "%d%%", (int)(overlay_value * 100.0f + 0.5f));
    oled.setFont(u8g2_font_helvR12_tr);
    int pw = oled.getStrWidth(pct);
    oled.drawStr(DISP_W - pw - 6, 24, pct);

    draw_bar(4, 36, DISP_W - 8, 18, overlay_value);
  }
}

void draw_idle() {
  oled.setFont(u8g2_font_helvB18_tr);
  const char *name = "DelSol";
  int nw = oled.getStrWidth(name);
  oled.drawStr((DISP_W - nw) / 2, 28, name);

  oled.setFont(u8g2_font_helvR10_tr);
  const char *status;
  if (bt_connected)
    status = is_paused ? "Paused" : "Connected";
  else
    status = "Waiting for Bluetooth...";
  int sw = oled.getStrWidth(status);
  oled.drawStr((DISP_W - sw) / 2, 50, status);
}

void draw_progress_line(int y) {
  uint32_t pos = get_current_position();
  uint32_t dur = track_duration_ms;

  char pos_str[8], dur_str[8];
  format_time(pos, pos_str, sizeof(pos_str));
  format_time(dur, dur_str, sizeof(dur_str));

  oled.setFont(u8g2_font_helvR10_tr);
  int pos_w = oled.getStrWidth(pos_str);
  int dur_w = oled.getStrWidth(dur_str);

  oled.drawStr(4, y + 12, pos_str);
  oled.drawStr(DISP_W - dur_w - 4, y + 12, dur_str);

  int bar_x = pos_w + 12;
  int bar_w = DISP_W - pos_w - dur_w - 24;
  if (bar_w < 20) bar_w = 20;
  float progress = (dur > 0) ? (float)pos / (float)dur : 0.0f;
  draw_bar(bar_x, y + 2, bar_w, 10, progress);
}

void draw_now_playing() {
  int top_mode = top_line_map[settings_values[SET_TOP_LINE]];
  int bottom_mode = bottom_line_map[settings_values[SET_BOTTOM_LINE]];

  // title area: larger font
  oled.setFont(u8g2_font_helvB14_tr);
  switch (top_mode) {
    case DISP_TITLE:
      draw_scrolling_text(4, 18, DISP_W - 8, track_title);
      break;
    case DISP_ARTIST:
      draw_scrolling_text(4, 18, DISP_W - 8, track_artist[0] ? track_artist : " ");
      break;
    case DISP_TITLE_ARTIST: {
      char combined[256];
      build_title_artist(combined, sizeof(combined));
      draw_scrolling_text(4, 18, DISP_W - 8, combined);
      break;
    }
    case DISP_PROGRESS:
      draw_progress_line(2);
      break;
  }

  if (settings_values[SET_SEPARATOR])
    oled.drawHLine(4, 26, DISP_W - 8);

  // bottom area: smaller font for artist, or progress bar
  if (bottom_mode == DISP_PROGRESS) {
    draw_progress_line(30);
  } else {
    oled.setFont(u8g2_font_helvR12_tr);
    switch (bottom_mode) {
      case DISP_TITLE:
        draw_scrolling_text(4, 46, DISP_W - 8, track_title);
        break;
      case DISP_ARTIST:
        draw_scrolling_text(4, 46, DISP_W - 8, track_artist[0] ? track_artist : " ");
        break;
      case DISP_TITLE_ARTIST: {
        char combined[256];
        build_title_artist(combined, sizeof(combined));
        draw_scrolling_text(4, 46, DISP_W - 8, combined);
        break;
      }
    }
  }

  // pause indicator
  if (is_paused && bt_connected) {
    oled.setFont(u8g2_font_helvR08_tr);
    oled.drawStr(DISP_W - 36, 62, "PAUSE");
  }
}

void draw_settings() {
  const SettingDef *def = &settings_defs[settings_page];
  int val = settings_values[settings_page];

  // page indicator dots
  oled.setFont(u8g2_font_helvR08_tr);
  char pg[8];
  snprintf(pg, sizeof(pg), "%d/%d", settings_page + 1, (int)SETTINGS_COUNT);
  int pgw = oled.getStrWidth(pg);
  oled.drawStr(DISP_W - pgw - 4, 10, pg);

  oled.setFont(u8g2_font_helvB14_tr);
  oled.drawStr(4, 16, def->name);

  oled.drawHLine(4, 22, DISP_W - 8);

  if (def->type == SET_VARIABLE && settings_editing) {
    int range = def->max_val - def->min_val;
    float bar_val = range > 0 ? (float)(val - def->min_val) / range : 0.0f;
    draw_bar(4, 30, DISP_W - 8, 16, bar_val);

    char vstr[16];
    if (def->unit)
      snprintf(vstr, sizeof(vstr), "%d %s", val, def->unit);
    else
      snprintf(vstr, sizeof(vstr), "%d", val);
    oled.setFont(u8g2_font_helvB12_tr);
    int vw = oled.getStrWidth(vstr);
    oled.drawStr((DISP_W - vw) / 2, 62, vstr);
  } else {
    char vstr[40];
    if (def->type == SET_TOGGLE) {
      snprintf(vstr, sizeof(vstr), "%s", val ? "ON" : "OFF");
    } else if (def->type == SET_LIST) {
      snprintf(vstr, sizeof(vstr), "%s", def->labels[val]);
    } else {
      if (def->unit)
        snprintf(vstr, sizeof(vstr), "%d %s", val, def->unit);
      else
        snprintf(vstr, sizeof(vstr), "%d", val);
    }
    oled.setFont(u8g2_font_helvR14_tr);
    int vw = oled.getStrWidth(vstr);
    oled.drawStr((DISP_W - vw) / 2, 48, vstr);
  }

  if (settings_editing) {
    oled.setFont(u8g2_font_helvR08_tr);
    oled.drawStr(4, 62, "< PREV    NEXT >");
  }
}

void draw_debug() {
  oled.setFont(u8g2_font_5x7_tf);
  char buf[80];

  if (settings_values[SET_CALIBRATE]) {
    snprintf(buf, sizeof(buf), "CALIBRATE - sweep all pots, toggle off to save");
  } else {
    snprintf(buf, sizeof(buf), "DEBUG v%s  BT=%s  sweep:lo/hi observed",
             __version__, bt_connected ? "Y" : "N");
  }
  oled.drawStr(2, 7, buf);

  FilteredPot *pots[5] = {&pot_vol, &pot_bass, &pot_treb, &pot_fade, &pot_bal};
  const char *names[5] = {"VOL", "BAS", "TRE", "FAD", "BAL"};
  for (int i = 0; i < 5; i++) {
    FilteredPot *p = pots[i];
    int lo = (p->raw_min > p->raw_max) ? 0 : p->raw_min;
    int hi = (p->raw_min > p->raw_max) ? 0 : p->raw_max;
    snprintf(buf, sizeof(buf), "%s r=%4d  lo=%4d hi=%4d  o=%3d v=%.2f",
             names[i], p->raw_value, lo, hi, p->output, p->smoothed);
    oled.drawStr(2, 16 + i * 9, buf);
  }
}

void update_display() {
  oled.clearBuffer();

  if (settings_mode) {
    draw_settings();
  } else if (settings_values[SET_DEBUG_VIEW] || settings_values[SET_CALIBRATE]) {
    draw_debug();
  } else if (millis() < overlay_until && overlay_label != NULL) {
    draw_overlay();
  } else if (track_title[0] == '\0' && track_artist[0] == '\0') {
    draw_idle();
  } else {
    draw_now_playing();
  }

  oled_send();
}

// --- Boot Animation ---

void boot_animation() {
  const char *line1 = "Honda";
  const char *line2 = "DelSol";

  // fade in: draw text at increasing y-offset (slide down into place)
  oled.setFont(u8g2_font_helvB18_tr);
  int w1 = oled.getStrWidth(line1);
  int w2 = oled.getStrWidth(line2);
  int x1 = (DISP_W - w1) / 2;
  int x2 = (DISP_W - w2) / 2;

  for (int i = 0; i <= 10; i++) {
    oled.clearBuffer();
    int offset = 10 - i;
    oled.drawStr(x1, 28 - offset, line1);
    oled.drawStr(x2, 56 + offset, line2);
    oled_send();
    delay(50);
    yield();
  }
  delay(1500);
}

// --- OTA Update Mode ---

void ota_draw(const char *status, int progress = -1) {
  oled.clearBuffer();
  oled.setFont(u8g2_font_helvB14_tr);
  const char *title = "OTA Update";
  oled.drawStr((DISP_W - oled.getStrWidth(title)) / 2, 18, title);

  oled.setFont(u8g2_font_helvR08_tr);
  oled.drawStr((DISP_W - oled.getStrWidth(status)) / 2, 36, status);

  if (progress >= 0) {
    int bar_w = 200;
    int bar_x = (DISP_W - bar_w) / 2;
    oled.drawFrame(bar_x, 46, bar_w, 12);
    oled.drawBox(bar_x + 2, 48, (bar_w - 4) * progress / 100, 8);
  }
  oled_send();
}

// shared OTA logic — assumes OLED is already initialized
void ota_start_and_block() {
  // join Caleb's phone hotspot (has internet → works around Windows
  // deprioritizing internet-less softAPs)
  const char *sta_ssid = "Caleb's S25 Ultra";
  const char *sta_pass = "polly1523";

  ota_draw("Connecting to hotspot...");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(sta_ssid, sta_pass);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
    delay(200);
  }
  if (WiFi.status() != WL_CONNECTED) {
    ota_draw("Hotspot not found");
    delay(3000);
    ESP.restart();
  }
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  char ip_str[32];
  snprintf(ip_str, sizeof(ip_str), "IP: %s", WiFi.localIP().toString().c_str());

  ArduinoOTA.setHostname("delsol-radio");

  ArduinoOTA.onStart([]() {
    ota_draw("Updating...", 0);
  });
  ArduinoOTA.onProgress([](unsigned int prog, unsigned int total) {
    ota_draw("Updating...", prog * 100 / total);
  });
  ArduinoOTA.onEnd([]() {
    ota_draw("Done! Rebooting...");
    delay(1000);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    const char *msg = "Unknown error";
    if (error == OTA_AUTH_ERROR) msg = "Auth failed";
    else if (error == OTA_BEGIN_ERROR) msg = "Begin failed";
    else if (error == OTA_CONNECT_ERROR) msg = "Connect failed";
    else if (error == OTA_RECEIVE_ERROR) msg = "Receive failed";
    else if (error == OTA_END_ERROR) msg = "End failed";
    ota_draw(msg);
    delay(3000);
    ESP.restart();
  });
  ArduinoOTA.begin();

  oled.clearBuffer();
  oled.setFont(u8g2_font_helvB14_tr);
  const char *title = "OTA Update";
  oled.drawStr((DISP_W - oled.getStrWidth(title)) / 2, 16, title);
  oled.setFont(u8g2_font_helvR08_tr);
  char l1[64];
  snprintf(l1, sizeof(l1), "On %s", sta_ssid);
  oled.drawStr((DISP_W - oled.getStrWidth(l1)) / 2, 36, l1);
  oled.drawStr((DISP_W - oled.getStrWidth(ip_str)) / 2, 52, ip_str);
  oled_send();

  for (;;) {
    ArduinoOTA.handle();
    delay(10);
  }
}

// called from settings menu — set NVS flag and reboot so OTA gets a clean
// BT-free start. PCs (Windows especially) won't reliably connect to the AP
// while the BT controller is still holding the radio.
void enter_ota_mode() {
  settings_mode = false;
  ota_draw("Rebooting into OTA...");
  prefs.putBool("ota_pending", true);
  prefs.end();
  delay(500);
  ESP.restart();
}

// called at boot if PREV+NEXT held — nothing else is running yet
void enter_ota_mode_early() {
  SPI.begin(18, -1, 23, -1);
  oled.setBusClock(4000000);
  oled.begin();
  oled.setContrast(255);
  ota_start_and_block();
}

// --- Web OTA (phone-triggered self-update) ---
//
// The phone can't push firmware to us directly — its page is HTTPS and we'd be
// plain HTTP on the LAN (blocked as mixed content). So the phone tells us to
// update over BLE, we drop BT, join the hotspot for internet, and PULL the new
// image from GitHub Pages ourselves. Same one-way flash-slot safety as espota.
void web_ota_run() {
  const char *sta_ssid = "Caleb's S25 Ultra";
  const char *sta_pass = "polly1523";

  ota_draw("Connecting to hotspot...");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(sta_ssid, sta_pass);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) delay(200);
  if (WiFi.status() != WL_CONNECTED) {
    ota_draw("Hotspot not found");
    delay(3000);
    ESP.restart();
  }
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  WiFiClientSecure client;
  client.setInsecure();   // Pages cert isn't pinned on-device; the image itself is verified by the bootloader
  httpUpdate.rebootOnUpdate(false);
  httpUpdate.onProgress([](int cur, int total) {
    ota_draw("Downloading update...", total ? (int)((int64_t)cur * 100 / total) : 0);
  });

  ota_draw("Downloading update...", 0);
  t_httpUpdate_return ret = httpUpdate.update(client, WEB_FW_URL);

  if (ret == HTTP_UPDATE_OK) {
    ota_draw("Done! Rebooting...");
    delay(1200);
    ESP.restart();
  } else {
    char msg[48];
    snprintf(msg, sizeof(msg), "Update failed (%d)", httpUpdate.getLastError());
    ota_draw(msg);
    delay(4000);
    ESP.restart();
  }
}

// set a flag and reboot so the update runs BT-free, same as the espota path
void enter_web_update() {
  settings_mode = false;
  ota_draw("Rebooting to update...");
  prefs.putBool("web_upd", true);
  prefs.end();
  delay(500);
  ESP.restart();
}

void enter_web_update_early() {
  SPI.begin(18, -1, 23, -1);
  oled.setBusClock(4000000);
  oled.begin();
  oled.setContrast(255);
  web_ota_run();
}

// --- BLE Settings Server ---

#define BLE_SVC_UUID   "a1e00001-5b1e-4f2a-9c3d-7e8f0a1b2c3d"
#define BLE_STATE_UUID "a1e00002-5b1e-4f2a-9c3d-7e8f0a1b2c3d"
#define BLE_META_UUID  "a1e00003-5b1e-4f2a-9c3d-7e8f0a1b2c3d"
#define BLE_CMD_UUID   "a1e00004-5b1e-4f2a-9c3d-7e8f0a1b2c3d"

#define BLE_PROTO_VER 1

#define CMD_SET_SETTING   0x01
#define CMD_SET_EQ_BAND   0x02
#define CMD_TRANSPORT     0x03
#define CMD_COMMIT        0x05
#define CMD_FACTORY_RESET 0x06
#define CMD_WEB_UPDATE    0x07

struct __attribute__((packed)) BleState {
  uint8_t  proto;
  uint8_t  flags;          // b0 connected, b1 paused, b2 muted, b3 settings_mode
  uint8_t  volume, bass, treble, fade, balance;   // live pot positions, 0-100
  uint32_t pos_ms, dur_ms;
  int8_t   eq[EQ_BANDS];
  int16_t  makeup_db10;    // EQ makeup attenuation, dB × 10 (negative)
  uint16_t dsp_load_pct;
  uint32_t free_heap;
  int16_t  settings[SETTINGS_COUNT];
  uint8_t  fw_major, fw_minor, fw_patch;   // appended — running firmware version
};

// the web app parses this by hard-coded byte offsets, so a layout change here
// silently corrupts every reading on the phone — fail the build instead
static_assert(sizeof(BleState) == 33 + SETTINGS_COUNT * 2 + 3,
              "BleState layout changed: update STATE_LEN and the offsets in webapp/index.html");

struct BleCmd { uint8_t op; uint8_t idx; int16_t val; };

BLECharacteristic *ble_state_char = NULL;
BLECharacteristic *ble_meta_char = NULL;
QueueHandle_t ble_cmd_queue = NULL;
volatile bool ble_client_connected = false;
bool ble_active = false;

class BleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) { ble_client_connected = true; }
  void onDisconnect(BLEServer *s) {
    ble_client_connected = false;
    BLEDevice::startAdvertising();  // otherwise the phone can never find us again
  }
};

// runs on the BLE task — must not touch the OLED (SPI) or NVS directly, so commands
// are queued and drained by loop() on the main core
class BleCmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) {
    std::string v = c->getValue();
    if (v.length() < 4 || ble_cmd_queue == NULL) return;
    const uint8_t *p = (const uint8_t *)v.data();
    BleCmd cmd = {p[0], p[1], (int16_t)(p[2] | (p[3] << 8))};
    xQueueSend(ble_cmd_queue, &cmd, 0);
  }
};

void ble_start() {
  ble_cmd_queue = xQueueCreate(16, sizeof(BleCmd));

  BLEDevice::init("DelSol Radio");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new BleServerCallbacks());

  BLEService *svc = server->createService(BLE_SVC_UUID);

  ble_state_char = svc->createCharacteristic(
      BLE_STATE_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  ble_state_char->addDescriptor(new BLE2902());

  ble_meta_char = svc->createCharacteristic(
      BLE_META_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  ble_meta_char->addDescriptor(new BLE2902());

  BLECharacteristic *cmd = svc->createCharacteristic(
      BLE_CMD_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  cmd->setCallbacks(new BleCmdCallbacks());

  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SVC_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  ble_active = true;
}

void ble_publish_state() {
  if (!ble_state_char || !ble_client_connected) return;

  BleState s = {};
  s.proto = BLE_PROTO_VER;
  s.flags = (bt_connected ? 1 : 0) | (is_paused ? 2 : 0) | (is_muted ? 4 : 0) |
            (settings_mode ? 8 : 0);
  s.volume  = pot_vol.output;
  s.bass    = pot_bass.output;
  s.treble  = pot_treb.output;
  s.fade    = pot_fade.output;
  s.balance = pot_bal.output;

  // extrapolate between AVRCP position updates so the phone's progress bar runs smoothly
  uint32_t pos = track_position_ms;
  if (bt_connected && !is_paused && position_sync_time)
    pos += millis() - position_sync_time;
  s.pos_ms = pos;
  s.dur_ms = track_duration_ms;

  memcpy(s.eq, eq_gains, sizeof(eq_gains));
  s.makeup_db10  = (int16_t)lroundf(20.0f * log10f(eq_makeup) * 10.0f);
  s.dsp_load_pct = dsp_load_pct;
  s.free_heap    = esp_get_free_heap_size();
  for (int i = 0; i < (int)SETTINGS_COUNT; i++) s.settings[i] = settings_values[i];
  s.fw_major = fw_ver[0]; s.fw_minor = fw_ver[1]; s.fw_patch = fw_ver[2];

  ble_state_char->setValue((uint8_t *)&s, sizeof(s));
  ble_state_char->notify();
}

void ble_publish_meta() {
  if (!ble_meta_char || !ble_client_connected) return;
  uint8_t buf[260];
  size_t n = 0;
  size_t tl = strnlen(track_title, sizeof(track_title));
  size_t al = strnlen(track_artist, sizeof(track_artist));
  if (tl > 128) tl = 128;
  if (al > 128) al = 128;
  memcpy(buf + n, track_title, tl);  n += tl;  buf[n++] = 0;
  memcpy(buf + n, track_artist, al); n += al;  buf[n++] = 0;
  ble_meta_char->setValue(buf, n);
  ble_meta_char->notify();
}

// dragging a slider must not burn one flash write per pixel — changes apply to the DSP
// immediately but only reach NVS once the user stops moving things
#define NVS_DEBOUNCE_MS 2000
uint32_t settings_dirty_mask = 0;
bool eq_dirty = false;
unsigned long settings_save_due = 0;

void save_eq() {
  char key[4];
  for (int i = 0; i < EQ_BANDS; i++) {
    snprintf(key, sizeof(key), "e%d", i);
    prefs.putInt(key, eq_gains[i]);
  }
}

void load_eq() {
  char key[4];
  for (int i = 0; i < EQ_BANDS; i++) {
    snprintf(key, sizeof(key), "e%d", i);
    int v = prefs.getInt(key, 0);
    if (v < -EQ_MAX_DB || v > EQ_MAX_DB) v = 0;
    eq_gains[i] = v;
  }
}

void ble_process_commands() {
  BleCmd c;
  while (ble_cmd_queue && xQueueReceive(ble_cmd_queue, &c, 0) == pdTRUE) {
    switch (c.op) {
      case CMD_SET_SETTING: {
        if (c.idx >= SETTINGS_COUNT) break;
        const SettingDef *d = &settings_defs[c.idx];
        int v = c.val;
        if (v < d->min_val) v = d->min_val;
        if (v > d->max_val) v = d->max_val;
        settings_values[c.idx] = v;
        apply_setting(c.idx);
        settings_dirty_mask |= (1UL << c.idx);
        settings_save_due = millis() + NVS_DEBOUNCE_MS;
        break;
      }
      case CMD_SET_EQ_BAND: {
        if (c.idx >= EQ_BANDS) break;
        int v = c.val;
        if (v < -EQ_MAX_DB) v = -EQ_MAX_DB;
        if (v > EQ_MAX_DB) v = EQ_MAX_DB;
        eq_gains[c.idx] = v;
        filters_dirty = true;
        eq_dirty = true;
        settings_save_due = millis() + NVS_DEBOUNCE_MS;
        break;
      }
      case CMD_TRANSPORT:
        if (!bt_connected) break;
        if (c.idx == 0)      { if (is_paused) a2dp_sink.play(); else a2dp_sink.pause(); }
        else if (c.idx == 1) a2dp_sink.next();
        else if (c.idx == 2) a2dp_sink.previous();
        break;
      case CMD_COMMIT:
        settings_save_due = millis();
        break;
      case CMD_FACTORY_RESET:
        settings_values[SET_FACTORY_RESET] = 1;
        apply_setting(SET_FACTORY_RESET);
        settings_values[SET_FACTORY_RESET] = 0;
        memset(eq_gains, 0, sizeof(eq_gains));
        filters_dirty = true;
        save_eq();
        break;
      case CMD_WEB_UPDATE:
        // drained on the main loop, so it's safe to flag NVS and reboot here
        enter_web_update();
        break;
    }
  }
}

void flush_pending_saves() {
  if (!settings_save_due || (long)(millis() - settings_save_due) < 0) return;
  settings_save_due = 0;
  for (int i = 0; i < (int)SETTINGS_COUNT; i++)
    if (settings_dirty_mask & (1UL << i)) save_setting(i);
  settings_dirty_mask = 0;
  if (eq_dirty) { save_eq(); eq_dirty = false; }
}

// --- Setup & Loop ---

void setup() {
  Serial.begin(115200);
  Serial.printf("DelSol Head Unit v%s\n", __version__);
  sscanf(__version__, "%hhu.%hhu.%hhu", &fw_ver[0], &fw_ver[1], &fw_ver[2]);

  // failsafe OTA: hold NEXT during power-on to enter OTA mode. NEXT/PREV are a single
  // rocker on the OEM faceplate — you physically can't press both — so recovery is one
  // button. Bypasses all application code, so it works even if the firmware is broken.
  pinMode(PIN_NEXT, INPUT_PULLUP);
  delay(50);
  if (digitalRead(PIN_NEXT) == LOW) {
    Serial.println("OTA mode: NEXT held at boot");
    enter_ota_mode_early();
  }

  init_i2s(I2S_NUM_0, I2S1_BCK, I2S1_LRCK, I2S1_DOUT);
  init_i2s(I2S_NUM_1, I2S2_BCK, I2S2_LRCK, I2S2_DOUT);

  SPI.begin(18, -1, 23, -1);
  oled.setBusClock(4000000);
  oled.begin();
  oled.setContrast(255);

  prefs.begin("delsol", false);

  // OTA-on-reboot flag, set by menu-triggered enter_ota_mode()
  if (prefs.getBool("ota_pending", false)) {
    prefs.remove("ota_pending");
    Serial.println("OTA mode: pending flag set");
    enter_ota_mode_early();
  }

  // web-update flag, set by the phone over BLE (enter_web_update)
  if (prefs.getBool("web_upd", false)) {
    prefs.remove("web_upd");
    Serial.println("Web update: pending flag set");
    enter_web_update_early();
  }

  // bump this when settings are reordered to force a reset of stale prefs
  const int SETTINGS_VERSION = 5;
  if (prefs.getInt("sver", 0) != SETTINGS_VERSION) {
    prefs.clear();
    prefs.putInt("sver", SETTINGS_VERSION);
  }

  init_settings();

  if (settings_values[SET_BOOT_ANIM])
    boot_animation();

  oled.clearBuffer();
  oled_send();

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);

  init_pots();
  load_calibration();
  load_eq();
  init_response_grid();
  update_filters();
  // no callback running yet, apply directly
  copy_coeffs(&bass_L, &pending_bass);
  copy_coeffs(&bass_R, &pending_bass);
  copy_coeffs(&treble_L, &pending_treble);
  copy_coeffs(&treble_R, &pending_treble);
  for (int b = 0; b < EQ_BANDS; b++) {
    copy_coeffs(&eq_L[b], &pending_eq[b]);
    copy_coeffs(&eq_R[b], &pending_eq[b]);
  }
  eq_makeup = pending_makeup;
  filters_dirty = false;

  pinMode(PIN_PLAY, INPUT_PULLUP);
  pinMode(PIN_NEXT, INPUT_PULLUP);
  pinMode(PIN_PREV, INPUT_PULLUP);
  pinMode(PIN_MUTE, INPUT_PULLUP);

  const char *bt_name = bt_name_options[settings_values[SET_BT_NAME]];
  bool auto_reconn = settings_values[SET_AUTO_RECONN];

  // dual-mode so the BLE settings server can run alongside A2DP — BLE and BR/EDR share
  // one controller with a common scheduler, unlike WiFi which contends for the antenna
  a2dp_sink.set_default_bt_mode(ESP_BT_MODE_BTDM);

  a2dp_sink.set_stream_reader(audio_data_callback, false);
  a2dp_sink.set_avrc_metadata_attribute_mask(ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | ESP_AVRC_MD_ATTR_PLAYING_TIME);
  a2dp_sink.set_avrc_metadata_callback(avrc_metadata_cb);
  a2dp_sink.set_avrc_rn_play_pos_callback(avrc_play_pos_cb, 1);
  a2dp_sink.set_avrc_rn_playstatus_callback(avrc_rn_playstatus_cb);
  a2dp_sink.set_on_connection_state_changed(connection_state_cb);
  a2dp_sink.start(bt_name, auto_reconn);

  Serial.printf("Bluetooth started — pair your phone to '%s'\n", bt_name);
  Serial.printf("Free heap after A2DP: %u\n", esp_get_free_heap_size());

  ble_start();
  Serial.printf("Free heap after BLE: %u\n", esp_get_free_heap_size());
}

void loop() {
  unsigned long now = millis();

  ble_process_commands();
  flush_pending_saves();

  static unsigned long last_ble_state = 0;
  if (ble_client_connected && now - last_ble_state >= 200) {
    ble_publish_state();
    last_ble_state = now;
  }

  if (autoplay_pending && now - autoplay_time > 500) {
    autoplay_pending = false;
    a2dp_sink.play();
  }

  read_pots();
  process_buttons();

  // compute filter coefficients here (main core) instead of in the audio callback,
  // so trig math can't stall I2S DMA
  if (filters_dirty && !filters_ready) {
    update_filters();
    filters_dirty = false;
    filters_ready = true;
  }

  bool in_overlay = overlay_label != NULL && millis() < overlay_until;

  if (!settings_mode && !in_overlay) {
    if (now > scroll_pause_until && now - last_scroll > (unsigned long)settings_values[SET_SCROLL_SPEED]) {
      scroll_offset += 2;
      if (scroll_wrap_at > 0 && scroll_offset >= scroll_wrap_at) {
        scroll_offset = 0;
        scroll_pause_until = now + settings_values[SET_SCROLL_PAUSE];
      }
      last_scroll = now;
      display_dirty = true;
    }
    int bottom_mode = bottom_line_map[settings_values[SET_BOTTOM_LINE]];
    if (bottom_mode == DISP_PROGRESS && bt_connected)
      display_dirty = true;
  }

  if (in_overlay && now - last_display_update >= 50) {
    display_dirty = true;
  }

  if ((settings_values[SET_DEBUG_VIEW] || settings_values[SET_CALIBRATE]) && !settings_mode) {
    display_dirty = true;
  }

  if (metadata_changed) {
    scroll_offset = 0;
    scroll_wrap_at = 0;
    scroll_pause_until = now + settings_values[SET_SCROLL_PAUSE];
    metadata_changed = false;
    display_dirty = true;
    ble_publish_meta();
  }

  if (display_dirty && !is_muted && now - last_display_update >= 33) {
    update_display();
    last_display_update = now;
    display_dirty = false;
    read_pots(); // catch any changes that happened during the ~60ms sendBuffer
  }
}
