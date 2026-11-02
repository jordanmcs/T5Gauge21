// FuelEdu — bus-side trip computer (instant + windowed average MPG) for T5Gauge21.
//
// Model: fuel flow is derived as a RATE (rpm x final pw), never by counting
// pulses, so integrators need no sync to engine events. dt is always the
// MEASURED gap between samples — irregular arrival is absorbed automatically.
//   gal_h = C_FUEL x Rpm x pw_ms
//   instant_mpg = mph / gal_h   (EMA-smoothed, tau = TAU_MS)
// AVERAGE = fixed ring of N_CELLS per-mile fuel slices (window = N_CELLS x
// CELL_MI miles, rolling). Sensitivity of the display to a new cell is always
// 1/N_CELLS — no early twitchiness, no late calcification. Idling fuel lands
// in the current partial cell (counts honestly, ages out as miles accumulate).
// Reset = refill every cell at AVG_SEED_MPG: display starts at 24.0 and can
// move at most 1/N_CELLS per quarter mile. "--" rules (Jordan): no motion,
// instant over 99, or stale stream.
//
// TWO STREAMS, ONE SCREEN (Jordan 2026-11):
//   liveSt — fed by real ESP-NOW frames, ALWAYS (even while a log plays),
//            persisted to NVS every 1/4 mile and on reset. The trip is real.
//   sandSt — fed only by SD-card playback rows, RAM only, NEVER touches NVS.
//   The display shows sandSt while playback runs ("playback takes over the
//   display" — showing the log's numbers), liveSt otherwise. The banks never
//   interact; stopping playback reveals the live trip exactly as the road
//   left it. There is no flush, no restore, no pause of live collection.
#include "FuelEdu.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <nvs.h>
#include <ui.h>

// glue: playback state (LogSystem.cpp)
bool LogSystem_playbackActive();

// ---- tunables -------------------------------------------------------------
// Fuel: B202 4-cyl with Bosch 0280155968 "Green Giant" 42 lb/hr (440cc) —
// C = 4 cyl x 42 lb/hr / (120 x 1000 x 6.3 lb/gal), flow at 3.0 bar (43.5 psi).
// Refine from a real fill-up (scale by actual_gal/computed_gal); see vault
// "ECU test mule". If regulator pressure differs from 3 bar, scale by sqrt(P/3bar).
#define C_FUEL_GALH  2.222e-4f // gal_h = C x rpm x pw_ms
#define AVG_SEED_MPG 24.0f     // average shown right after reset / refill
#define N_CELLS      64        // window = 64 cells x 0.25 mi = 16 miles
#define CELL_MI      0.25f     // resolution of the rolling window
#define TAU_MS       2000.0f   // instant smoothing time constant
#define STALE_MS     2500UL    // no sample for this long -> freeze + "--"
#define MPG_MAX      99.0f     // above this -> "--"
#define MPH_MIN      1.0f      // below this -> "no motion", instant "--"

// ---- state -----------------------------------------------------------------
static lv_obj_t* page = nullptr;
static lv_obj_t* valLbl = nullptr;   // big instant value
static lv_obj_t* avgLbl = nullptr;   // sub-line: avg (long-press = reset)
static lv_obj_t* tripLbl = nullptr; // miles since reset (odometer crosscheck)

struct TripState {
  float cGal[N_CELLS], cMi[N_CELLS];  // per-cell fuel / distance
  int   head;                          // current partial cell
  float sumGal, sumMi;                 // window sums (ring-total, conserved)
  float tripMi;                        // cumulative miles since reset
  float lastSave;                      // tripMi at last NVS write (-1 none)
  float galE, mphE;                    // instant EMA pair (<0 = unseeded)
  unsigned long tLast;                 // dt base (per bank)
  bool  haveT;
  unsigned long lastSampleMs;          // per-bank freshness for "--" rules
};
static TripState liveSt, sandSt;

static char lastVal[16] = "";
static char lastAvg[24] = "";
static char lastTrip[24] = "";

// ---- persistence (NVS): LIVE BANK ONLY — playback paths never call these.
// Write cadence = one save per CELL_MI driven (cell rollover) or per reset;
// wear-levelled, a few writes per drive. A hard power cut mid-quarter-mile
// loses at most that partial slice.
#define NVS_NS       "fueltrip"
#define NVS_KEY      "trip"
#define TRIP_MAGIC   0x54525031u   // "TRP1"

struct TripBlob {
  uint32_t magic;
  int32_t  head;
  float    cGal[N_CELLS], cMi[N_CELLS];
  float    sumGal, sumMi, tripMi;
  uint32_t cksum;                 // additive over all preceding words
};

static uint32_t blobSum(const TripBlob& b) {
  const uint32_t* w = (const uint32_t*)&b;
  uint32_t s = 0;
  size_t n = (offsetof(TripBlob, cksum)) / sizeof(uint32_t);
  for (size_t i = 0; i < n; i++) s += w[i];
  return s;
}

static void seedTrip(TripState& T);   // fwd

static void tripSave(void) {          // persists liveSt ONLY
  TripBlob b = {};
  b.magic = TRIP_MAGIC;
  b.head = liveSt.head;
  memcpy(b.cGal, liveSt.cGal, sizeof(liveSt.cGal));
  memcpy(b.cMi, liveSt.cMi, sizeof(liveSt.cMi));
  b.sumGal = liveSt.sumGal; b.sumMi = liveSt.sumMi; b.tripMi = liveSt.tripMi;
  b.cksum = blobSum(b);
  nvs_handle_t h;
  if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
    nvs_set_blob(h, NVS_KEY, &b, sizeof(b));
    nvs_commit(h);
    nvs_close(h);
  }
  liveSt.lastSave = liveSt.tripMi;
}

// returns true if a valid live trip was restored (into liveSt)
static bool tripLoad(void) {
  nvs_handle_t h;
  if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
  TripBlob b;
  size_t sz = sizeof(b);
  bool ok = (nvs_get_blob(h, NVS_KEY, &b, &sz) == ESP_OK) && sz == sizeof(b) &&
            b.magic == TRIP_MAGIC && b.cksum == blobSum(b) &&
            b.head >= 0 && b.head < N_CELLS;
  nvs_close(h);
  if (!ok) return false;
  seedTrip(liveSt);              // defaults for fields not persisted
  liveSt.head = b.head;
  memcpy(liveSt.cGal, b.cGal, sizeof(b.cGal));
  memcpy(liveSt.cMi, b.cMi, sizeof(b.cMi));
  liveSt.sumGal = b.sumGal; liveSt.sumMi = b.sumMi; liveSt.tripMi = b.tripMi;
  return true;
}

static void seedTrip(TripState& T) {
  T.sumGal = T.sumMi = 0;
  for (int i = 0; i < N_CELLS; i++) {
    T.cMi[i] = CELL_MI;
    T.cGal[i] = CELL_MI / AVG_SEED_MPG;
    T.sumGal += T.cGal[i];
    T.sumMi += T.cMi[i];
  }
  T.head = 0;
  T.tripMi = 0;
  T.lastSave = -1.0f;
  T.galE = T.mphE = -1.0f;
  T.haveT = false;
  T.tLast = 0;
  T.lastSampleMs = 0;
}

// Add F gal over D miles to bank T; split cell overflow forward
// proportionally, overwriting the oldest cell. Sums stay conserved except
// for the aged-out cells.
static void ringAddTo(TripState& T, float F, float D) {
  T.cGal[T.head] += F;
  T.cMi[T.head] += D;
  T.sumGal += F;
  T.sumMi += D;
  while (T.cMi[T.head] >= CELL_MI) {
    float frac = (T.cMi[T.head] - CELL_MI) / T.cMi[T.head];
    float gCarry = T.cGal[T.head] * frac;
    float mCarry = T.cMi[T.head] - CELL_MI;
    T.cGal[T.head] -= gCarry;
    T.cMi[T.head] = CELL_MI;
    T.head = (T.head + 1) % N_CELLS;      // age out oldest
    T.sumGal += gCarry - T.cGal[T.head];
    T.sumMi += mCarry - T.cMi[T.head];
    T.cGal[T.head] = gCarry;
    T.cMi[T.head] = mCarry;
  }
}

// Integrate one (rpm, pw_ms, kmh) observation into bank T at nowMs.
// dt is measured per bank; gaps >3 s pause integration (never corrupt).
static void sampleTrip(TripState& T, float rpm, float pw, float kmh,
                       unsigned long nowMs) {
  T.lastSampleMs = nowMs;
  if (!T.haveT) { T.tLast = nowMs; T.haveT = true; return; }
  const unsigned long dtms = nowMs - T.tLast;
  T.tLast = nowMs;
  if (dtms == 0 || dtms > 3000) return;

  const float galRate = C_FUEL_GALH * rpm * pw;   // gal/h, raw
  const float mph = kmh * 0.621371f;
  const float dMi = mph * (dtms / 3600000.0f);    // 0 when stopped
  const int headBefore = T.head;
  ringAddTo(T, galRate * (dtms / 3600000.0f), dMi);
  T.tripMi += dMi;

  const float a = dtms / (TAU_MS + dtms);         // EMA, dt-correct
  T.galE = (T.galE < 0) ? galRate : T.galE + a * (galRate - T.galE);
  T.mphE = (T.mphE < 0) ? mph : T.mphE + a * (mph - T.mphE);

  // NVS: LIVE BANK ONLY, once per completed cell / quarter mile of travel.
  if (&T == &liveSt &&
      (headBefore != T.head ||
       (T.lastSave < 0 ? T.tripMi >= 0.25f
                       : T.tripMi - T.lastSave >= 0.25f)))
    tripSave();
}

void FuelEdu_liveSample(float rpm, float pwMs, float kmh) {
  sampleTrip(liveSt, rpm, pwMs, kmh, millis());
}

void FuelEdu_playSample(float rpm, float pwMs, float kmh) {
  sampleTrip(sandSt, rpm, pwMs, kmh, millis());
}

// Playback started: sandbox restarts from a clean prior — RAM only, no NVS.
void FuelEdu_beginSandbox(void) {
  seedTrip(sandSt);
}

// ---- display ---------------------------------------------------------------
static void fmt(char* buf, size_t n, float v) {
  // same whole-vs-decimal convention as updateDisplay()
  if (floorf(v) == v) snprintf(buf, n, "%.0f", v);
  else                snprintf(buf, n, "%.1f", v);
}

static void reset_flash_off_cb(lv_timer_t* t) {
  lv_obj_t* lbl = (lv_obj_t*)t->user_data;  // LVGL 8.3: field, no getter
  lv_obj_set_style_text_color(lbl, lv_color_hex(_ui_theme_color_Green[0]), 0);
  lv_timer_del(t);
}

static void avg_reset_cb(lv_event_t*) { FuelEdu_resetTrip(); }

void FuelEdu_resetTrip(void) {
  // Reset targets the bank ON SCREEN: a playback reset is RAM-only; a live
  // reset must survive power-down.
  TripState& T = LogSystem_playbackActive() ? sandSt : liveSt;
  seedTrip(T);
  if (&T == &liveSt) tripSave();
  // visible feedback: gold for 800 ms, one-shot timer restores theme green
  if (!avgLbl) return;
  lv_obj_set_style_text_color(avgLbl, lv_color_hex(0xDEB600), 0);
  lv_timer_create(reset_flash_off_cb, 800, avgLbl);
}

// ---- page construction (mirrors ui_Gauge component geometry/fonts) ---------
void FuelEdu_init(void) {
  if (!ui_SingleScr) return;
  seedTrip(sandSt);                   // sandbox: RAM only, always fresh
  if (!tripLoad()) seedTrip(liveSt);  // live: restore across power-down, else 24.0

  page = lv_obj_create(ui_SingleScr);
  lv_obj_remove_style_all(page);
  lv_obj_set_size(page, 480, 480);
  lv_obj_clear_flag(page, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

  // NAME — gold seven-seg small, top (like every page)
  lv_obj_t* name = lv_label_create(page);
  lv_obj_set_width(name, LV_SIZE_CONTENT);
  lv_obj_set_height(name, LV_SIZE_CONTENT);
  lv_obj_set_y(name, -140);
  lv_obj_set_align(name, LV_ALIGN_CENTER);
  lv_label_set_text(name, "mpg");
  lv_obj_set_style_text_color(name, lv_color_hex(0xDEB600), 0);
  lv_obj_set_style_text_font(name, &ui_font_SeveSegSm, 0);

  // VALUE — big instant, themed green, center
  valLbl = lv_label_create(page);
  lv_obj_set_width(valLbl, LV_SIZE_CONTENT);
  lv_obj_set_height(valLbl, LV_SIZE_CONTENT);
  lv_obj_set_align(valLbl, LV_ALIGN_CENTER);
  lv_label_set_text(valLbl, "--");
  lv_obj_clear_flag(valLbl, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC |
                            LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN);
  ui_object_set_themeable_style_property(valLbl, LV_PART_MAIN | LV_STATE_DEFAULT,
                                         LV_STYLE_TEXT_COLOR, _ui_theme_color_Green);
  lv_obj_set_style_text_font(valLbl, &ui_font_SeveSegLg, 0);

  // UNIT slot — gray "inst" tag at the component's unit position
  lv_obj_t* unit = lv_label_create(page);
  lv_obj_set_width(unit, LV_SIZE_CONTENT);
  lv_obj_set_height(unit, LV_SIZE_CONTENT);
  lv_obj_set_x(unit, 164);
  lv_obj_set_y(unit, -23);
  lv_obj_set_align(unit, LV_ALIGN_CENTER);
  lv_label_set_text(unit, "inst");
  lv_obj_set_style_text_color(unit, lv_color_hex(0x808080), 0);
  lv_obj_set_style_text_font(unit, &ui_font_SeveSegSm, 0);

  // MAXVALUE slot position — trip average, green small; long-press resets
  avgLbl = lv_label_create(page);
  lv_obj_set_width(avgLbl, LV_SIZE_CONTENT);
  lv_obj_set_height(avgLbl, LV_SIZE_CONTENT);
  lv_obj_set_y(avgLbl, 125);
  lv_obj_set_align(avgLbl, LV_ALIGN_CENTER);
  lv_label_set_text(avgLbl, "24.0");
  lv_obj_clear_flag(avgLbl, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(avgLbl, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_pad_all(avgLbl, 14, 0);  // finger target inside round bezel
  ui_object_set_themeable_style_property(avgLbl, LV_PART_MAIN | LV_STATE_DEFAULT,
                                         LV_STYLE_TEXT_COLOR, _ui_theme_color_Green);
  lv_obj_set_style_text_font(avgLbl, &ui_font_SeveSegSm, 0);
  lv_obj_add_event_cb(avgLbl, avg_reset_cb, LV_EVENT_LONG_PRESSED, nullptr);

  // unit tag for the trip line (avg needs none — page header says mpg)
  lv_obj_t* miTag = lv_label_create(page);
  lv_obj_set_width(miTag, LV_SIZE_CONTENT);
  lv_obj_set_height(miTag, LV_SIZE_CONTENT);
  lv_obj_set_x(miTag, 100);
  lv_obj_set_y(miTag, 178);
  lv_obj_set_align(miTag, LV_ALIGN_CENTER);
  lv_label_set_text(miTag, "Mi");
  lv_obj_set_style_text_color(miTag, lv_color_hex(0x808080), 0);
  lv_obj_set_style_text_font(miTag, &ui_font_SeveSegSm, 0);

  // TRIP — miles since reset, gray, centered under avg
  tripLbl = lv_label_create(page);
  lv_obj_set_width(tripLbl, LV_SIZE_CONTENT);
  lv_obj_set_height(tripLbl, LV_SIZE_CONTENT);
  lv_obj_set_y(tripLbl, 178);
  lv_obj_set_align(tripLbl, LV_ALIGN_CENTER);
  lv_label_set_text(tripLbl, "0.0");
  lv_obj_clear_flag(tripLbl, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_text_color(tripLbl, lv_color_hex(0x808080), 0);
  lv_obj_set_style_text_font(tripLbl, &ui_font_SeveSegSm, 0);
}

// ---- per-loop tick (DISPLAY only; integration happens at sample time) ------
void FuelEdu_tick(unsigned long nowMs) {
  if (!page) return;

  // Playback owns the screen (shows the log's numbers); otherwise live truth.
  const TripState& D = LogSystem_playbackActive() ? sandSt : liveSt;
  const bool fresh = (nowMs - D.lastSampleMs) <= STALE_MS;

  // instant label: "--" when stale, no motion, or out of range
  char vb[16];
  float instMpg = (D.galE > 1e-6f) ? D.mphE / D.galE : 0.0f;
  if (!fresh || D.mphE < MPH_MIN || instMpg > MPG_MAX || instMpg <= 0.0f)
    snprintf(vb, sizeof(vb), "--");
  else
    fmt(vb, sizeof(vb), instMpg);
  if (strcmp(vb, lastVal) != 0) {
    lv_label_set_text(valLbl, vb);
    snprintf(lastVal, sizeof(lastVal), "%s", vb);
  }

  // windowed average from the ring (always ~16 real-or-seeded miles of window)
  float avg = (D.sumMi > 1e-6f) ? D.sumGal / D.sumMi : 1.0f / AVG_SEED_MPG;
  float avgMpg = (avg > 1e-9f) ? 1.0f / avg : 99.9f;
  if (avgMpg > 99.9f) avgMpg = 99.9f;
  char ab[24];
  snprintf(ab, sizeof(ab), "%.1f", avgMpg);
  if (strcmp(ab, lastAvg) != 0) {
    lv_label_set_text(avgLbl, ab);
    snprintf(lastAvg, sizeof(lastAvg), "%s", ab);
  }

  // trip odometer: 0.1 mi resolution to 99.9, whole miles beyond
  char tb[24];
  if (D.tripMi < 100.0f) snprintf(tb, sizeof(tb), "%.1f", D.tripMi);
  else                   snprintf(tb, sizeof(tb), "%.0f", D.tripMi);
  if (strcmp(tb, lastTrip) != 0) {
    lv_label_set_text(tripLbl, tb);
    snprintf(lastTrip, sizeof(lastTrip), "%s", tb);
  }
}
