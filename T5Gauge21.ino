#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include "RTC_PCF85063.h"
#include "LVGL_Driver.h"
#include <map>
#include <vector>
#include <Preferences.h>  //esp32 stores in non-volitle memory
#include <ui.h>
//#include "globals.h"

Preferences prefs;

const float SHIFT_RPM_THRESHOLD = 3500.0;  //low for testing


struct LabelBinding {
  std::vector<lv_obj_t*> valueLabels;
};


std::map<String, LabelBinding> labelBindings;

struct SensorData {
  String label;
  float value;
  String unit;
};


std::map<String, float> sensorMap;

#include "LogSystem.h"
#include "FuelEdu.h"

// --- LogSystem glue ---
float sensorValueFor(const char* id) {
  auto it = sensorMap.find(String(id));
  return (it != sensorMap.end()) ? it->second : 0.0f;
}

// Freshness clock for FuelEdu: set by BOTH live and playback ingest.
unsigned long lastDataMs = 0;

void logFeed(const char* id, float v) {      // playback -> same path as live
  String k(id);
  sensorMap[k] = v;
  lastDataMs = millis();
  updatePeaks(v, k);
  updateDisplay();
}


float peakAirTemp = -10.0f;       // initialize with very low value
float peakBoostPressure = -1.0f;  // same here

bool newDataAvailable = false;
char jsonBuffer[256];



void startBlink(lv_obj_t* obj) {
  static lv_anim_t blinkAnim;
  lv_anim_init(&blinkAnim);

  lv_anim_set_var(&blinkAnim, obj);
  lv_anim_set_values(&blinkAnim, LV_OPA_100, LV_OPA_0);           // Full to transparent
  lv_anim_set_time(&blinkAnim, 500);                              // 500 ms for one fade
  lv_anim_set_playback_time(&blinkAnim, 500);                     // Fade back in
  lv_anim_set_repeat_count(&blinkAnim, LV_ANIM_REPEAT_INFINITE);  // Repeat forever
  lv_anim_set_exec_cb(&blinkAnim, [](void* var, int32_t v) {
    lv_obj_set_style_opa((lv_obj_t*)var, v, LV_PART_MAIN);
  });

  lv_anim_start(&blinkAnim);
}

void stopBlink(lv_obj_t* obj) {
  lv_anim_del(obj, nullptr);  // remove opacity animation (exec_cb was a lambda in startBlink)
  lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);  // Ensure hidden when stopping
}

void updateClockLabels() {
  // Read time from RTC
  PCF85063_Read_Time(&datetime);

  // Convert 24-hour to 12-hour format and determine AM/PM
  int hour = datetime.hour;
  bool isPM = false;

  if (hour == 0) {
    hour = 12;  // midnight = 12 AM
  } else if (hour == 12) {
    isPM = true;  // noon = 12 PM
  } else if (hour > 12) {
    hour -= 12;
    isPM = true;
  }

  // Format HH:MM string
  char timeStr[6];  // "HH:MM\0"
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d", hour, datetime.minute);

  // Set time text
  lv_label_set_text(uic_clock_value, timeStr);

  // Set AM or PM
  lv_label_set_text(uic_clock_unit, isPM ? "PM" : "AM");
}

void updatePeaks(float& value, const String& id) {
  if (id == "Lufttemp") {  // replace with your actual sensor ID for air temp
    if (value > peakAirTemp) {
      peakAirTemp = value;
    }
  } else if (id == "P_manifold") {  // your sensor ID for boost pressure
    if (value > peakBoostPressure) {
      peakBoostPressure = value;
    }
  }
}
void updatePeakLabels() {
  char buf[16];

  snprintf(buf, sizeof(buf), "%.1f", peakAirTemp);
  lv_label_set_text(ui_aitmaxValLbl, buf);

  snprintf(buf, sizeof(buf), "%.1f", peakBoostPressure);
  lv_label_set_text(ui_boostmaxValLbl, buf);
}

void updateDisplay() {
  for (const auto& pair : sensorMap) {
    const String& id = pair.first;
    float value = pair.second;

    if (!labelBindings.count(id)) continue;
    //aitmaxValLbl
    char buffer[16];
    if (floor(value) == value) {
      // It's a whole number (like 1500.0)
      snprintf(buffer, sizeof(buffer), "%.0f", value);
    } else {
      // It's not a whole number (like 12.3 or 12.34)
      snprintf(buffer, sizeof(buffer), "%.1f", value);  // or "%.2f" if desired
    }

    // Update all linked labels
    for (lv_obj_t* label : labelBindings[id].valueLabels) {
      if (label) lv_label_set_text(label, buffer);
    }

    // 🎯 Update arcs based on sensor ID
    if (id == "Medeltrot" && ui_ThrottleArc) {
      lv_arc_set_value(ui_ThrottleArc, value);  // e.g., 0–100%
    }

    if (id == "AD_sond" && ui_LambdaArc) {
      lv_arc_set_value(ui_LambdaArc, value);  // e.g., convert 0.95 → 95
    }
    // NOTE: TQ is a torque-demand SCALAR (1..106, ~2x load percent), NOT Nm
    // (wiki TQ page, 2026-11 RE) — hp = TQ x rpm / 7120 was computing fiction
    // and is removed. Real hp needs a calibrated torque map; see vault note.


    // Show shift indicator if RPM > threshold
    static bool blinking = false;

    if (id == "Rpm") {
      if (value > SHIFT_RPM_THRESHOLD) {
        if (!blinking) {
          startBlink(ui_ShiftUpLbl);
          startBlink(uic_tach_shiftup);
          blinking = true;
        }
      } else {
        if (blinking) {
          stopBlink(ui_ShiftUpLbl);
          stopBlink(uic_tach_shiftup);

          blinking = false;
        }
      }
    }
  }
  updatePeakLabels();
}

// Wipe every data label to "--" so a page that no log/live frame feeds can
// never show its SquareLine design-time placeholder ("13.6", "12.7", "7:23")
// as if it were a reading. Called on playback start; unit labels are left.
void clearDisplayPlaceholders(void) {
  lv_obj_t* lbls[] = {
    ui_rpmLbl, ui_aitValLbl, ui_boostValLbl, ui_speedLbl,
    uic_tach_value, uic_voltmeter_value, uic_cts_value, uic_ait_value,
    uic_lambda_value, uic_boost_value, uic_ignition_value, uic_throttle_value,
    uic_speed_value, uic_speed_name, uic_torque_value, uic_APC_value,
  };
  for (unsigned i = 0; i < sizeof(lbls) / sizeof(lbls[0]); i++)
    if (lbls[i]) lv_label_set_text(lbls[i], "--");
}

//temp for debugging
void printSensorMap() {
  Serial.println("=== Sensor Map ===");
  for (const auto& pair : sensorMap) {
    const String& id = pair.first;
    const float& value = pair.second;

    Serial.printf("%s : %.2f \n", id.c_str(), value);
  }
  Serial.println("===================");
}

void onReceive(const esp_now_recv_info_t* recvInfo, const uint8_t* incomingData, int len) {
  if (!incomingData || len <= 0) return;
  // Live ingest NEVER pauses for playback — every real frame still feeds the
  // trip integrator (processJSON). Playback only takes over the display.
  size_t copyLen = (size_t)len;
  if (copyLen >= sizeof(jsonBuffer)) copyLen = sizeof(jsonBuffer) - 1;
  memcpy(jsonBuffer, incomingData, copyLen);
  jsonBuffer[copyLen] = '\0';
  newDataAvailable = true;
}
void processJSON(const char* jsonString) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, jsonString);
  if (error) {
    Serial.print("JSON parse error!: ");
    Serial.println(error.f_str());
    return;
  }

  // LIVE collection never stops, even while a log plays: the persisted trip
  // integrates only real frames (containsKey — never mutates doc).
  auto num = [&doc](const char* k) { return doc.containsKey(k) ? (float)doc[k] : 0.0f; };
  FuelEdu_liveSample(num("Rpm"), num("Insptid_ms10"), num("Bil_hast"));

  // Playback owns the display: don't paint over it, but collection is done.
  if (LogSystem_playbackActive()) return;

  for (JsonPair kv : doc.as<JsonObject>()) {
    String id = kv.key().c_str();
    float value = kv.value().as<float>();
    sensorMap[id] = value;  // Insert or update
    lastDataMs = millis();  // FuelEdu freshness

    updateDisplay();
    printSensorMap();
    updatePeaks(value, id);
  }
  rec_sample();  // CSV row if recording (live data only)
}


void WiFi_init(void) {
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(onReceive);
}


// ---- Settings-page wiring (runtime — SquareLine-regen-safe) ----------------
// The generated ui_events.c stubs (toggleNightMode etc.) come back empty on
// every SquareLine export, so real behaviour lives HERE and is attached to
// the switches directly — same precedent as LogSystem's LOGGER button.
#include <nvs.h>
#define BL_DAY    100  // current behaviour
#define BL_NIGHT   25  // first guess — Jordan tunes by eye

static void nightSave(bool night) {
  nvs_handle_t h;
  if (nvs_open("ui", NVS_READWRITE, &h) == ESP_OK) {
    nvs_set_u8(h, "night", night ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
  }
}

static void night_cb(lv_event_t*) {
  const bool night = lv_obj_has_state(ui_nightMode, LV_STATE_CHECKED);
  Set_Backlight(night ? BL_NIGHT : BL_DAY);
  nightSave(night);
}

static void settingsWiring_init(void) {
  if (!ui_nightMode) return;
  nvs_handle_t h;
  uint8_t night = 0;
  if (nvs_open("ui", NVS_READONLY, &h) == ESP_OK) {
    nvs_get_u8(h, "night", &night);
    nvs_close(h);
  }
  // restore BEFORE attaching the cb so boot doesn't re-save
  if (night) {
    lv_obj_add_state(ui_nightMode, LV_STATE_CHECKED);
    Set_Backlight(BL_NIGHT);
  }
  lv_obj_add_event_cb(ui_nightMode, night_cb, LV_EVENT_VALUE_CHANGED, nullptr);
}

void setup() {
  //   pinMode(BACKLIGHT_PIN, OUTPUT);
  //digitalWrite(BACKLIGHT_PIN, HIGH);  // turn on backlight

  Serial.begin(115200);
  //while(!Serial) delay(10);
  WiFi_init();
  I2C_Init();
  TCA9554PWR_Init(0x00);
  Set_EXIO(EXIO_PIN8, Low);
  PCF85063_Init();
  LCD_Init();
  Lvgl_Init();
  ui_init();


  // set 100% to start
  Set_Backlight(100);

  settingsWiring_init();  // Day/Night switch (may dim per saved setting)
  LogSystem_init(logFeed);  // adds LOGGER button to Settings screen
  FuelEdu_init();           // appends "mpg" swipe page after the clock


  // set label bindings for incoming JSON messages
  //dash display
  labelBindings["Rpm"].valueLabels.push_back(ui_rpmLbl);
  labelBindings["Lufttemp"].valueLabels.push_back(ui_aitValLbl);
  labelBindings["P_manifold"].valueLabels.push_back(ui_boostValLbl);
  labelBindings["Bil_hast"].valueLabels.push_back(ui_speedLbl);


  //single display
  labelBindings["Rpm"].valueLabels.push_back(uic_tach_value);
  labelBindings["Batt_volt"].valueLabels.push_back(uic_voltmeter_value);  //single display
  labelBindings["Kyl_temp"].valueLabels.push_back(uic_cts_value);         //single display
  labelBindings["Lufttemp"].valueLabels.push_back(uic_ait_value);         //single display
  labelBindings["AD_sond"].valueLabels.push_back(uic_lambda_value);       //single display
  labelBindings["P_manifold"].valueLabels.push_back(uic_boost_value);     //single display
  labelBindings["Ign_angle"].valueLabels.push_back(uic_ignition_value);   //single display
  labelBindings["Medeltrot"].valueLabels.push_back(uic_throttle_value);   //single display
  labelBindings["Bil_hast"].valueLabels.push_back(uic_speed_value);       //single display
  labelBindings["Gear"].valueLabels.push_back(uic_speed_name);            //single display
  labelBindings["TQ"].valueLabels.push_back(uic_torque_value);            //single display
  labelBindings["PWM_ut10"].valueLabels.push_back(uic_APC_value);         //single display — interpreter sends symbolId "PWM_ut10", not "APC"




  //labelBindings["Medeltrot"].valueLabels.push_back(uic_tps_value);//single display not currently used
}

unsigned long lastUpdate = 0;
unsigned long lastClockUpdate = 0;

const unsigned long updateInterval = 50;  // ms

void loop() {
  Lvgl_Loop();
  LogSystem_tick();
  FuelEdu_tick(millis());

  unsigned long now = millis();
  if (newDataAvailable && (now - lastUpdate >= updateInterval)) {
    newDataAvailable = false;
    lastUpdate = now;
    processJSON(jsonBuffer);
  }
  if (now - lastClockUpdate >= 1000) {
    lastClockUpdate = now;
    updateClockLabels();
  }

  // vTaskDelay(pdMS_TO_TICKS(5));
}
