// FuelEdu — bus-side trip computer (instant + windowed average MPG) for T5Gauge21.
//
// Model: fuel flow is derived as a RATE (rpm x final pw), never by counting
// pulses, so integrators need no sync to engine events. dt is always the
// MEASURED gap between ticks — irregular arrival is absorbed automatically.
//   gal_h = C_FUEL x Rpm x pw_ms
//   instant_mpg = mph / gal_h   (EMA-smoothed, tau = TAU_MS)
// AVERAGE = fixed ring of N_CELLS per-mile fuel slices (window = N_CELLS x
// CELL_MI miles, rolling). Sensitivity of the display to a new cell is always
// 1/N_CELLS — no early twitchiness, no late calcification. Idling fuel lands
// in the current partial cell (counts honestly, ages out as miles accumulate).
// Reset = refill every cell at AVG_SEED_MPG: display starts at 24.0 and can
// move at most 1/N_CELLS per quarter mile — "don't update too quickly" is
// geometry, not weighting. "--" rules (Jordan): no motion, instant over 99,
// or stale stream.
#include "FuelEdu.h"
#include <math.h>
#include <stdio.h>
#include <ui.h>

// ---- glue from T5Gauge21.ino --------------------------------------------
extern float sensorValueFor(const char* id);
extern unsigned long lastDataMs;  // last live/playback ingest (millis)

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
#define STALE_MS     2500UL    // no frame for this long -> freeze + "--"
#define MPG_MAX      99.0f     // above this -> "--"
#define MPH_MIN      1.0f      // below this -> "no motion", instant "--"

// ---- state -----------------------------------------------------------------
static lv_obj_t* page = nullptr;
static lv_obj_t* valLbl = nullptr;   // big instant value
static lv_obj_t* avgLbl = nullptr;   // sub-line: avg (long-press = reset)
static lv_obj_t* tripLbl = nullptr; // miles since reset (odometer crosscheck)

static unsigned long tLast = 0;
static bool haveT = false;
static float galS = -1.0f, mphS = -1.0f;   // EMA states (<0 = unseeded)
static float s_tripMi = 0;                 // cumulative miles since reset

// Rolling window: oldest cell overwritten forever; head = current partial.
static float cGal[N_CELLS], cMi[N_CELLS];
static int head = 0;
static float sumGal = 0, sumMi = 0;
static char lastVal[16] = "";
static char lastAvg[24] = "";

static void seedRing(void) {
  sumGal = sumMi = 0;
  for (int i = 0; i < N_CELLS; i++) {
    cMi[i] = CELL_MI;
    cGal[i] = CELL_MI / AVG_SEED_MPG;
    sumGal += cGal[i];
    sumMi += cMi[i];
  }
  head = 0;
}

// Add fuel D-miles-worth of driving (F gal, D mi); split overflow into new
// cells proportionally, overwriting the oldest. Sums stay conserved except
// for the overwritten (aged-out) cells.
static void ringAdd(float F, float D) {
  cGal[head] += F;
  cMi[head] += D;
  sumGal += F;
  sumMi += D;
  while (cMi[head] >= CELL_MI) {
    float frac = (cMi[head] - CELL_MI) / cMi[head];
    float gCarry = cGal[head] * frac;
    float mCarry = cMi[head] - CELL_MI;
    cGal[head] -= gCarry;
    cMi[head] = CELL_MI;
    head = (head + 1) % N_CELLS;      // age out oldest
    sumGal += gCarry - cGal[head];
    sumMi += mCarry - cMi[head];
    cGal[head] = gCarry;
    cMi[head] = mCarry;
  }
}

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
  seedRing();
  s_tripMi = 0;   // trip odometer resets with the avg window
  // visible feedback: gold for 800 ms, one-shot timer restores theme green
  if (!avgLbl) return;
  lv_obj_set_style_text_color(avgLbl, lv_color_hex(0xDEB600), 0);
  lv_timer_create(reset_flash_off_cb, 800, avgLbl);
}

// ---- page construction (mirrors ui_Gauge component geometry/fonts) ---------
void FuelEdu_init(void) {
  if (!ui_SingleScr) return;
  seedRing();

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

  // TRIP — miles since reset, gray, centered under avg. Long-press anywhere on
  // this page's avg resets it too.
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

// ---- per-loop tick ----------------------------------------------------------
void FuelEdu_tick(unsigned long nowMs) {
  if (!page) return;

  unsigned long dtms = haveT ? (nowMs - tLast) : 0;
  tLast = nowMs;
  haveT = true;
  if (dtms == 0 || dtms > 3000) return;  // boot / paused: pause, never corrupt

  const bool fresh = (nowMs - lastDataMs) <= STALE_MS;
  const float rpm = sensorValueFor("Rpm");
  const float pw = sensorValueFor("Insptid_ms10");
  const float kmh = sensorValueFor("Bil_hast");

  if (fresh) {
    const float galRate = C_FUEL_GALH * rpm * pw;   // gal/h, raw
    const float mph = kmh * 0.621371f;
    const float dMi = mph * (dtms / 3600000.0f);    // 0 when stopped
    ringAdd(galRate * (dtms / 3600000.0f), dMi);
    s_tripMi += dMi;

    const float a = dtms / (TAU_MS + dtms);         // EMA, dt-correct
    galS = (galS < 0) ? galRate : galS + a * (galRate - galS);
    mphS = (mphS < 0) ? mph : mphS + a * (mph - mphS);
  }

  // instant label: "--" when stale, no motion, or out of range
  char vb[16];
  float instMpg = (galS > 1e-6f) ? mphS / galS : 0.0f;
  if (!fresh || mphS < MPH_MIN || instMpg > MPG_MAX || instMpg <= 0.0f)
    snprintf(vb, sizeof(vb), "--");
  else
    fmt(vb, sizeof(vb), instMpg);
  if (strcmp(vb, lastVal) != 0) {
    lv_label_set_text(valLbl, vb);
    snprintf(lastVal, sizeof(lastVal), "%s", vb);
  }

  // windowed average from the ring (always ~16 real-or-seeded miles of window)
  float avg = (sumMi > 1e-6f) ? sumGal / sumMi : 1.0f / AVG_SEED_MPG;
  float avgMpg = (avg > 1e-9f) ? 1.0f / avg : 99.9f;
  if (avgMpg > 99.9f) avgMpg = 99.9f;
  char ab[24];
  snprintf(ab, sizeof(ab), "%.1f", avgMpg);
  if (strcmp(ab, lastAvg) != 0) {
    lv_label_set_text(avgLbl, ab);
    snprintf(lastAvg, sizeof(lastAvg), "%s", ab);
  }

  // trip odometer: 0.1 mi resolution to 99.9, whole miles beyond
  static char lastTrip[24] = "";
  char tb[24];
  if (s_tripMi < 100.0f) snprintf(tb, sizeof(tb), "%.1f", s_tripMi);
  else                   snprintf(tb, sizeof(tb), "%.0f", s_tripMi);
  if (strcmp(tb, lastTrip) != 0) {
    lv_label_set_text(tripLbl, tb);
    snprintf(lastTrip, sizeof(lastTrip), "%s", tb);
  }
}
