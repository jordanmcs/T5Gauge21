// VdoFace — VDO Vision-style analog dial drawn with pure LVGL primitives
// (ticks, numerals, blade needle w/ shadow+bevel, silver hub, slotted screws).
// Ported from the T5Gauge185 face engine (same design language Jordan
// approved there), geometry scaled to the 2.1 gauge's 480x480 panel and
// parameterized per symbol via VdoFaceSpec. All faces share the VDO arc:
// 270° sweep starting at 7:30 (225° clockwise-from-12).
#pragma once
#include <lvgl.h>

typedef struct {
  const char* id;      // sensorMap symbol id, or "Torque" for the derived one
  const char* name;    // top legend, e.g. "TURBO"
  const char* unit;    // bottom legend, e.g. "kPa"
  float min, max;
  float minor;         // short tick step
  float major;         // long tick step
  float numStep;       // numeral every N (0 = no numerals)
  int   numDiv;        // divide numeral for display (tach: 1000 -> "1..6")
  const char* numFmt;  // numeral printf (NULL = "%d"), e.g. "%.1f"
  const char* valFmt;  // readout printf (NULL = "%d"), e.g. "%.1f"
} VdoFaceSpec;

typedef struct VdoFace VdoFace;

// Draw a face filling `parent` (parent should be 480x480, black bg).
VdoFace* VdoFaceCreate(lv_obj_t* parent, const VdoFaceSpec* spec);
void VdoFaceDestroy(VdoFace* f);
// Move needle + numeric readout. value is in the face's (display) domain.
void VdoFaceSet(VdoFace* f, float value);
// Stream stale: readout shows "--", needle holds (VDO-behavior).
void VdoFaceStale(VdoFace* f);
