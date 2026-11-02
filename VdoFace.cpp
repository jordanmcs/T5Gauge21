// VdoFace implementation — ported from T5Gauge185 face engine.
// Geometry language of the VDO Vision Black dial:
//   270° sweep, needle home at 7:30 (225° cw-from-12), three visual layers
//   on the blade (shadow / color / bevel), silver hub, slotted screws,
//   fixed-width tabular readout between the screws.
// CX/CY/R: 480x480 panel (1.85 used 180/180/170 on its smaller panel).
#include "VdoFace.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CX 240.0f
#define CY 240.0f
#define R_FACE 225.0f
#define VDOSTART 225.0f
#define VADOSPAN 270.0f

typedef struct Pt { float x, y; } Pt;
static inline Pt polarPt(float r, float cwDeg) {
  const float rad = cwDeg * (float)M_PI / 180.0f;
  return { CX + r * sinf(rad), CY - r * cosf(rad) };
}

struct VdoFace {
  VdoFaceSpec spec;
  lv_obj_t* g;
  lv_obj_t* valueLbl;
  lv_obj_t* nBase; lv_point_t nPts[2];
  lv_obj_t* nShadow; lv_point_t sPts[2];
  lv_obj_t* nHi; lv_point_t hPts[2];
  int needleW;
  char lastTxt[16];
};

static void placeLabel(lv_obj_t* lbl, float r, float cwDeg) {
  const Pt p = polarPt(r, cwDeg);
  lv_obj_update_layout(lbl);
  lv_obj_set_pos(lbl, (lv_coord_t)(p.x - lv_obj_get_width(lbl) / 2),
                       (lv_coord_t)(p.y - lv_obj_get_height(lbl) / 2));
}

// radial tick; the lv_line is sized to its own bbox (LVGL clips to the box)
static void makeTick(lv_obj_t* parent, float cwDeg, float rOuter, float rInner,
                     int width, lv_color_t col) {
  const Pt a = polarPt(rInner, cwDeg), b = polarPt(rOuter, cwDeg);
  const float pad = width + 2.0f;
  const float x0 = fminf(a.x, b.x) - pad, y0 = fminf(a.y, b.y) - pad;
  const float x1 = fmaxf(a.x, b.x) + pad, y1 = fmaxf(a.y, b.y) + pad;
  lv_point_t* pts = (lv_point_t*)malloc(2 * sizeof(lv_point_t));
  pts[0].x = (lv_coord_t)(a.x - x0); pts[0].y = (lv_coord_t)(a.y - y0);
  pts[1].x = (lv_coord_t)(b.x - x0); pts[1].y = (lv_coord_t)(b.y - y0);
  lv_obj_t* line = lv_line_create(parent);
  lv_line_set_points(line, pts, 2);
  lv_obj_set_style_line_width(line, width, LV_PART_MAIN);
  lv_obj_set_style_line_color(line, col, LV_PART_MAIN);
  lv_obj_set_style_line_rounded(line, true, LV_PART_MAIN);
  lv_obj_set_size(line, (lv_coord_t)(x1 - x0), (lv_coord_t)(y1 - y0));
  lv_obj_set_pos(line, (lv_coord_t)x0, (lv_coord_t)y0);
  lv_obj_clear_flag(line, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
}

static lv_obj_t* makeTextCol(lv_obj_t* parent, const char* txt,
                             const lv_font_t* font, lv_color_t col) {
  lv_obj_t* l = lv_label_create(parent);
  lv_obj_set_style_text_font(l, font, 0);
  lv_obj_set_style_text_color(l, col, 0);
  lv_label_set_text(l, txt);
  return l;
}

static void fmtNum(char* buf, size_t n, const char* fmt, float v) {
  if (!fmt || strcmp(fmt, "%d") == 0) snprintf(buf, n, "%d", (int)(v + 0.5f));
  else snprintf(buf, n, fmt, (double)v);
}

static float faceAngleOf(const VdoFaceSpec* s, float v) {
  float f = (v - s->min) / (s->max - s->min);
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  return VDOSTART + VADOSPAN * f;
}

static void setPointer(VdoFace* f, float v) {
  const float cw = faceAngleOf(&f->spec, v);
  const float rad = cw * (float)M_PI / 180.0f;
  const float sx = sinf(rad), sy = -cosf(rad);
  const float px = -sy, py = sx;                 // unit perpendicular
  const float rOut = 0.85f * R_FACE, rTail = 0.18f * R_FACE;
  const float piv = (float)lv_obj_get_width(f->nBase) / 2.0f;
  f->nPts[0].x = (lv_coord_t)(piv - sx * rTail); f->nPts[0].y = (lv_coord_t)(piv - sy * rTail);
  f->nPts[1].x = (lv_coord_t)(piv + sx * rOut);  f->nPts[1].y = (lv_coord_t)(piv + sy * rOut);
  lv_line_set_points(f->nBase, f->nPts, 2);
  { // shadow: same blade offset sideways
    const float o = 1.6f;
    f->sPts[0].x = (lv_coord_t)(f->nPts[0].x + px * o); f->sPts[0].y = (lv_coord_t)(f->nPts[0].y + py * o);
    f->sPts[1].x = (lv_coord_t)(f->nPts[1].x + px * o); f->sPts[1].y = (lv_coord_t)(f->nPts[1].y + py * o);
    lv_line_set_points(f->nShadow, f->sPts, 2);
  }
  { // bevel highlight: upper flank, slightly short
    const float o = -(f->needleW / 2.0f - 1.1f);
    f->hPts[0].x = (lv_coord_t)(piv + px * o);
    f->hPts[0].y = (lv_coord_t)(piv + py * o);
    f->hPts[1].x = (lv_coord_t)(piv + sx * rOut * 0.92f + px * o);
    f->hPts[1].y = (lv_coord_t)(piv + sy * rOut * 0.92f + py * o);
    lv_line_set_points(f->nHi, f->hPts, 2);
  }
}

static void sizeNeedle(lv_obj_t* line, float rOut, float width) {
  const float half = rOut + width;
  lv_obj_set_size(line, (lv_coord_t)(2 * half), (lv_coord_t)(2 * half));
  lv_obj_set_pos(line, (lv_coord_t)(CX - half), (lv_coord_t)(CY - half));
  lv_obj_clear_flag(line, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
}

VdoFace* VdoFaceCreate(lv_obj_t* parent, const VdoFaceSpec* spec) {
  VdoFace* f = (VdoFace*)calloc(1, sizeof(VdoFace));
  f->spec = *spec;
  f->needleW = 6;
  const lv_color_t white = lv_color_white();

  f->g = lv_obj_create(parent);
  lv_obj_remove_style_all(f->g);
  lv_obj_set_size(f->g, 480, 480);
  lv_obj_clear_flag(f->g, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_bg_color(f->g, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(f->g, LV_OPA_COVER, 0);
  lv_obj_t* g = f->g;

  // tick ring: minor + major ticks, index math (no float accumulation)
  const int nMinor = (int)((spec->max - spec->min) / spec->minor + 0.5f);
  for (int i = 0; i <= nMinor; i++) {
    const float v = spec->min + spec->minor * i;
    const bool major = spec->major > 0 &&
      (fabsf((v - spec->min) / spec->major - nearbyintf((v - spec->min) / spec->major)) < 1e-3f);
    const float cw = VDOSTART + VADOSPAN * ((float)i / nMinor);
    makeTick(g, cw, 0.94f * R_FACE,
             major ? 0.78f * R_FACE : 0.885f * R_FACE, major ? 3 : 1, white);
  }
  // numerals
  if (spec->numStep > 0) {
    const int nNum = (int)((spec->max - spec->min) / spec->numStep + 0.5f);
    for (int i = 0; i <= nNum; i++) {
      const float v = spec->min + spec->numStep * i;
      char buf[12];
      fmtNum(buf, sizeof(buf), spec->numFmt,
             spec->numDiv > 0 ? v / spec->numDiv : v);
      lv_obj_t* n = makeTextCol(g, buf, &lv_font_montserrat_16, white);
      placeLabel(n, 0.62f * R_FACE,
                 VDOSTART + VADOSPAN * ((float)i / nNum));
    }
  }

  lv_obj_t* nameL = makeTextCol(g, spec->name, &lv_font_montserrat_24, white);
  lv_obj_align(nameL, LV_ALIGN_CENTER, 0, -0.35f * R_FACE);
  lv_obj_t* unitL = makeTextCol(g, spec->unit, &lv_font_montserrat_16, white);
  lv_obj_align(unitL, LV_ALIGN_CENTER, 0, 0.30f * R_FACE);

  // readout between the screws; fixed width + centered => never floats
  f->valueLbl = makeTextCol(g, "--", &lv_font_montserrat_24, white);
  lv_obj_set_width(f->valueLbl, 110);  // screw gap ≈ 118 px at 480 res
  lv_obj_set_style_text_align(f->valueLbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(f->valueLbl, LV_ALIGN_CENTER, 0, 0.62f * R_FACE);

  for (int s = -1; s <= 1; s += 2) {   // slotted screws
    lv_obj_t* screw = lv_obj_create(g);
    lv_obj_set_size(screw, 13, 13);
    lv_obj_clear_flag(screw, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(screw, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(screw, lv_color_hex(0x444444), 0);
    lv_obj_set_style_border_width(screw, 1, 0);
    lv_obj_set_style_border_color(screw, lv_color_hex(0x666666), 0);
    lv_obj_align(screw, LV_ALIGN_CENTER, s * 0.26f * R_FACE, 0.50f * R_FACE);
    lv_obj_t* slot = lv_obj_create(screw);
    lv_obj_set_size(slot, 8, 1);
    lv_obj_set_style_bg_color(slot, lv_color_hex(0x999999), 0);
    lv_obj_set_style_border_width(slot, 0, 0);
    lv_obj_set_style_radius(slot, 0, 0);
    lv_obj_center(slot);
  }

  lv_obj_t* hub = lv_obj_create(g);    // silver hub (drawn under needle? no—above)
  lv_obj_set_size(hub, 24, 24);
  lv_obj_clear_flag(hub, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(hub, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(hub, lv_color_hex(0xb8b8b8), 0);
  lv_obj_set_style_border_width(hub, 1, 0);
  lv_obj_set_style_border_color(hub, lv_color_hex(0x707070), 0);
  lv_obj_center(hub);
  (void)hub;

  // needle trio: shadow, base, bevel (order = z-order)
  struct { lv_obj_t** slot; int w; uint32_t hex; lv_opa_t opa; } parts[3] = {
    { &f->nShadow, f->needleW + 3, 0x000000, LV_OPA_50 },
    { &f->nBase,   f->needleW,     0xe8451e, LV_OPA_COVER },
    { &f->nHi,     LV_MAX(1, f->needleW - 4), 0xff7a60, LV_OPA_80 },
  };
  for (int i = 0; i < 3; i++) {
    lv_obj_t* l = lv_line_create(g);
    lv_obj_set_style_line_width(l, parts[i].w, LV_PART_MAIN);
    lv_obj_set_style_line_color(l, lv_color_hex(parts[i].hex), LV_PART_MAIN);
    lv_obj_set_style_line_opa(l, parts[i].opa, LV_PART_MAIN);
    lv_obj_set_style_line_rounded(l, true, LV_PART_MAIN);
    sizeNeedle(l, 0.85f * R_FACE + f->needleW, f->needleW);
    *parts[i].slot = l;
  }
  setPointer(f, spec->min);
  return f;
}

void VdoFaceSet(VdoFace* f, float value) {
  if (!f) return;
  setPointer(f, value);
  char buf[16];
  fmtNum(buf, sizeof(buf), f->spec.valFmt, value);
  if (strcmp(buf, f->lastTxt) != 0) {
    lv_label_set_text(f->valueLbl, buf);
    snprintf(f->lastTxt, sizeof(f->lastTxt), "%s", buf);
  }
}

void VdoFaceStale(VdoFace* f) {
  if (!f) return;
  if (strcmp("--", f->lastTxt) != 0) {
    lv_label_set_text(f->valueLbl, "--");
    snprintf(f->lastTxt, sizeof(f->lastTxt), "%s", "--");
  }
}

void VdoFaceDestroy(VdoFace* f) {
  if (!f) return;
  lv_obj_del(f->g);
  free(f);
}
