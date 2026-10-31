// LogSystem — SD datalogging + CSV playback for T5Gauge21 (see LogSystem.h)
#include "LogSystem.h"
#include "ui.h"
#include "SD_Card.h"
#include "SD_MMC.h"
#include "RTC_PCF85063.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <strings.h>


// ---------------- configuration ----------------
// Logged set = current display parameters (labelBindings in T5Gauge21.ino)
static const char* LOG_SYMBOLS[] = {
  "Rpm", "Bil_hast", "P_manifold", "Kyl_temp", "Lufttemp", "Batt_volt",
  "AD_sond", "Ign_angle", "Medeltrot", "Gear", "TQ", "APC"
};
static const int N_SYM = sizeof(LOG_SYMBOLS) / sizeof(LOG_SYMBOLS[0]);

// Legacy LogConverter/CanTraceExt column -> live id (with scale)
struct Remap { const char* col; const char* id; float scale; float offset; };
static const Remap REMAP[] = {
  { "ActualIn.n_Engine",  "Rpm",        1, 0     },
  { "In.v_Vehicle",       "Bil_hast",   1, 0     },
  { "In.p_AirInlet",      "P_manifold", 100, 0   },   // kPa -> x100 raw
  { "P_medel",            "P_manifold", 100, 0   },   // bar rel -> kPa rel (same as p_AirInlet)
  { "ActualIn.T_Engine",  "Kyl_temp",   1, 0     },
  { "ActualIn.T_AirInlet","Lufttemp",   1, 0     },
  { "Out.X_AccPedal",     "Medeltrot",  1, 0     },
  { "Out.fi_Ignition",    "Ign_angle",  1, 0     },
  { "Out.PWM_BoostCntrl", "APC",        1, 0     },
};
static const int N_REMAP = sizeof(REMAP) / sizeof(REMAP[0]);
static const int MAXCOLS = 40;
static const int MAXFILES = 24;
static const char* SDROOT = "/sdcard";

// ---------------- state ----------------
static LogFeedFn feedFn = nullptr;
static lv_obj_t* scr = NULL, * fileList = NULL, * statusLbl = NULL, * progLbl = NULL;
static lv_obj_t* recLbl = NULL, * playLbl = NULL, * speedLbl = NULL, * delLbl = NULL;

enum Mode { MODE_IDLE, MODE_REC, MODE_PLAY };
static Mode mode = MODE_IDLE;
static bool sdReady = false;

// recorder
static File     recFile;
static uint32_t recStartMs = 0, lastFlushMs = 0;

// player
static File     playFile;
static String   colId[MAXCOLS];      // live symbol id per column ("" = skip)
static float    colScale[MAXCOLS], colOff[MAXCOLS];
static int      nCols = 0;
static uint32_t playStartMs = 0, curDueMs = 0, playT0ms = 0;
static bool     playFirstRow = true;
static int      speed = 1;
static bool     rowLoaded = false;   // a row is fed, waiting for its due time

// file list
struct FileInfo { char disp[64]; char path[96]; char nm[40]; char mt[24]; char shortnm[40]; };
static FileInfo files[MAXFILES];
static int  nFiles = 0, selFile = -1;
static char fileBtns[MAXFILES][72];

// ---------------- helpers ----------------
static bool sd_mount() {
  if (sdReady && SD_MMC.cardType() != CARD_NONE) return true;
  SD_Init();                                    // Waveshare: EXIO4 power + SD_MMC 1-bit
  sdReady = (SD_MMC.cardType() != CARD_NONE);
  return sdReady;
}

static const char* mapId(const char* col, float* scale, float* offset) {
  for (int i = 0; i < N_REMAP; i++)
    if (strcmp(REMAP[i].col, col) == 0) { *scale = REMAP[i].scale; *offset = REMAP[i].offset; return REMAP[i].id; }
  for (int i = 0; i < N_SYM; i++)
    if (strcmp(LOG_SYMBOLS[i], col) == 0) { *scale = 1; *offset = 0; return LOG_SYMBOLS[i]; }
  *scale = 1; *offset = 0; return NULL;
}

static void setStatus(const char* txt) {
  if (statusLbl) lv_label_set_text(statusLbl, txt);
  printf("[LOG] %s\n", txt);
}

static String nextToken(String& line) {
  int c = line.indexOf(',');
  String tok = (c < 0) ? line : line.substring(0, c);
  line = (c < 0) ? String("") : line.substring(c + 1);
  tok.trim();
  return tok;
}

// friendly name: strip extension + 8.3 tilde (AUTOTU~1.CSV -> AUTOTU)
static void friendly_name(const char* base, char* out, size_t n) {
  size_t L = strlen(base);
  const char* dot = strrchr(base, '.');
  if (dot && !strncasecmp(dot, ".csv", 4)) L = dot - base;   // drop .csv/.CSV
  size_t j = 0;
  for (size_t i = 0; i < L && j < n - 1; i++)
    if (base[i] != '~') out[j++] = base[i];
  out[j] = 0;
}

// log duration = Time column of the last complete row (tail read, no full scan)
static double csv_duration_s(const char* path) {
  File f = SD_MMC.open(path, FILE_READ);
  if (!f || !f.size()) { if (f) f.close(); return -1; }
  uint32_t sz = f.size(), back = sz > 1536 ? 1536 : sz;
  bool ok = f.seek(sz - back);
  char buf[1537];
  int n = ok ? f.read((uint8_t*)buf, (int)back) : -1;
  f.close();
  if (n <= 0) return -1;
  buf[n] = 0;
  char* e = strrchr(buf, '\n');  if (!e) return -1; *e-- = 0;
  char* s = strrchr(buf, '\n');  if (!s) return -1; s++;
  double t = atof(s);
  return t > 0 ? t : -1;
}

// our own dir lister: Waveshare Folder_retrieval truncates names (sizeof on ptr)
static void list_dir(const char* dir, const char* prefix) {
  File d = SD_MMC.open(dir);
  if (!d) { printf("[LOG] list_dir open FAIL %s\n", dir); return; }
  printf("[LOG] list_dir %s isDir=%d\n", dir, d.isDirectory());
  File f = d.openNextFile();
  while (f && nFiles < MAXFILES) {
    const char* name = f.name();
    if (!f.isDirectory()) {
      // ESP32 FS name() may be full path — keep basename
      const char* base = strrchr(name, '/'); base = base ? base + 1 : name;
      printf("[LOG]   entry: %s\n", base);
      size_t bl = strlen(base);
      if (bl > 4 && !strncasecmp(base + bl - 4, ".csv", 4) &&
          strncmp(base, "PROBE.", 6)) {   // hide self-test file
        snprintf(files[nFiles].path, sizeof(files[nFiles].path), "%s/%s", dir, base);
        // second display line: creation date + log duration
        char meta[48] = "";
        {
          File cf = SD_MMC.open(files[nFiles].path, FILE_READ);
          char dates[24] = "----";
          if (cf) {
            time_t mt = cf.getLastWrite();
            if (mt > 0) strftime(dates, sizeof dates, "%m/%d/%y", localtime(&mt));
            cf.close();
          }
          double dur = csv_duration_s(files[nFiles].path);
          if (dur >= 0) {
            int d = (int)(dur + 0.5);
            snprintf(meta, sizeof meta, "%s  %02d:%02d", dates, d / 60, d % 60);
          } else snprintf(meta, sizeof meta, "%s", dates);
        }
            char fn[40]; friendly_name(base, fn, sizeof fn);
            snprintf(files[nFiles].nm, sizeof files[nFiles].nm, "%s%s", prefix, fn);
            if (strlen(files[nFiles].nm) > 33) {
              strncpy(files[nFiles].shortnm, files[nFiles].nm, 31);
              snprintf(files[nFiles].shortnm + 31, 4, "...");
            } else strcpy(files[nFiles].shortnm, files[nFiles].nm);
            snprintf(files[nFiles].mt, sizeof files[nFiles].mt, "%s", meta);
            snprintf(files[nFiles].disp, sizeof(files[nFiles].disp), "%s\n%s",
                     files[nFiles].nm, files[nFiles].mt);
        nFiles++;
      }
    }
    f = d.openNextFile();
  }
  d.close();
}

// ---------------- recorder ----------------
static void rec_stop() {
  if (recFile) { recFile.flush(); recFile.close(); }
  mode = MODE_IDLE;
  lv_label_set_text(recLbl, "REC");
}

static bool rec_start() {
  if (!sd_mount()) { setStatus("No SD card"); return false; }
  datetime_t dt; PCF85063_Read_Time(&dt);
  printf("[LOG] RTC %04d-%02d-%02d %02d:%02d:%02d\n",
                dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
  errno = 0;
  bool mk = SD_MMC.mkdir("/sdcard/logs");
  printf("[LOG] mkdir /logs -> %d errno=%d, free %llu MB\n", mk, errno,
         (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / 1048576ULL);
  char name[96];
  snprintf(name, sizeof(name), "/sdcard/logs/T5G_%02d%02d%02d_%02d%02d%02d.csv",
           dt.year % 100, dt.month, dt.day, dt.hour, dt.minute, dt.second);
  recFile = SD_MMC.open(name, FILE_WRITE, true);   // create=true REQUIRED by esp32 FS::open
  if (!recFile) {                    // retry at card root (no subdir support?)
    printf("[LOG] open %s failed, retry at root\n", name);
    snprintf(name, sizeof(name), "/sdcard/T5G_%02d%02d%02d_%02d%02d%02d.csv",
             dt.year % 100, dt.month, dt.day, dt.hour, dt.minute, dt.second);
    recFile = SD_MMC.open(name, FILE_WRITE, true);
    if (!recFile) {
      printf("[LOG] root open also failed, errno=%d\n", errno);
      // read-only probe: is the whole FS mounted RO?
      FILE* t = fopen("/sdcard/.wtest", "w");
      printf("[LOG] raw fopen /sdcard/.wtest -> %s errno=%d\n",
             t ? "OK" : "FAIL", errno);
      if (t) fclose(t), remove("/sdcard/.wtest");
      setStatus("Create failed"); return false;
    }
  }
  recFile.print("Time");
  for (int i = 0; i < N_SYM; i++) { recFile.print(','); recFile.print(LOG_SYMBOLS[i]); }
  recFile.println();
  recStartMs = millis();
  mode = MODE_REC;
  lv_label_set_text(recLbl, "STOP");
  char msg[80]; snprintf(msg, sizeof(msg), "Recording %s", strrchr(name, '/') + 1);
  setStatus(msg);
  return true;
}

void rec_sample() {                       // called from processJSON (live only)
  if (mode != MODE_REC || !recFile) return;
  recFile.printf("%.4f", (millis() - recStartMs) / 1000.0f);
  for (int i = 0; i < N_SYM; i++)
    recFile.printf(",%.2f", sensorValueFor(LOG_SYMBOLS[i]));
  recFile.println();
  uint32_t now = millis();
  if (now - lastFlushMs > 1000) { recFile.flush(); lastFlushMs = now; }
}

// ---------------- player ----------------
static bool hdr_read() {
  if (!playFile.available()) return false;
  String line = playFile.readStringUntil('\n'); line.trim();
  nCols = 0;
  while (line.length() && nCols < MAXCOLS) {
    String tok = nextToken(line);
    float sc = 1, off = 0; const char* id = mapId(tok.c_str(), &sc, &off);
    colId[nCols]   = id ? id : "";
    colScale[nCols] = sc;
    colOff[nCols]  = off;
    nCols++;
  }
  return nCols > 1;
}

static bool play_loadRow();   // fwd (defined below)

static void play_stop(const char* why) {
  if (playFile) playFile.close();
  mode = MODE_IDLE; rowLoaded = false;
  lv_label_set_text(playLbl, "PLAY");
  setStatus(why);
}

static bool play_start() {
  if (selFile < 0 || selFile >= nFiles) { setStatus("Select a file first"); return false; }
  if (!sd_mount()) { setStatus("No SD card"); return false; }
  playFile = SD_MMC.open(files[selFile].path, FILE_READ);
  if (!playFile) { setStatus("Open failed"); return false; }
  if (!hdr_read()) { playFile.close(); setStatus("Bad CSV header"); return false; }
  playStartMs = millis(); curDueMs = 0; rowLoaded = false;
  playT0ms = 0; playFirstRow = true;   // anchor pacing to FIRST row's timestamp
  play_loadRow();                    // preload first row; feeds at its timestamp
  mode = MODE_PLAY;
  lv_label_set_text(playLbl, "STOP");
  setStatus(files[selFile].disp);
  lv_scr_load(ui_DashDisplay);      // play back on the live gauge screen
  return true;
}

// feed next row now; set its due time. false = EOF
// load next row into the pending buffer (fed LATER, at its own timestamp)
static String rowSym[MAXCOLS];
static float  rowVal[MAXCOLS];
static int    rowN = 0;
static bool play_loadRow() {
  String line;
  do {
    if (!playFile.available()) return false;
    line = playFile.readStringUntil('\n'); line.trim();
  } while (!line.length() && playFile.available());
  if (!line.length()) return false;
  int col = 0; float t = 0; bool haveT = false; rowN = 0;
  while (line.length() && col < nCols) {
    String tok = nextToken(line);
    if (col == 0) {                          // seconds OR wall-clock timestamp
      if (tok.indexOf(':') >= 0) {
        int hh = 0, mm = 0; float ss = 0;
        const char* cp = strchr(tok.c_str(), ' '); cp = cp ? cp + 1 : tok.c_str();
        sscanf(cp, "%d:%d:%f", &hh, &mm, &ss);
        t = hh * 3600 + mm * 60 + ss;
      } else t = tok.toFloat();
      haveT = true;
    }
    else if (colId[col].length() && rowN < MAXCOLS) {
      rowSym[rowN] = colId[col];
      rowVal[rowN] = tok.toFloat() * colScale[col] + colOff[col];
      rowN++;
    }
    col++;
  }
  if (!haveT) return play_loadRow();
  uint32_t raw = (uint32_t)(t * 1000.0f);
  if (playFirstRow) { playT0ms = raw; playFirstRow = false; }   // logs need not start at 0
  curDueMs = (raw > playT0ms ? raw - playT0ms : 0) / speed;
  rowLoaded = true;
  return true;
}
static void play_emitRow() {
  for (int i = 0; i < rowN; i++) feedFn(rowSym[i].c_str(), rowVal[i]);
  rowLoaded = false;
}

// ---------------- UI ----------------
static void refresh_files();
static void filebtn_cb(lv_event_t* e) {
  int idx = (int)(intptr_t)lv_event_get_user_data(e);
  if (idx < nFiles) { selFile = idx; refresh_files(); }
}
static void refresh_files() {
  nFiles = 0;                       // selFile persists; reset only on screen open
  if (sd_mount()) {
    list_dir("/sdcard/logs", "");          // gauge logs first
    list_dir(SDROOT, "~/");                // root CSVs (Autotune ref etc.)
  }
  lv_obj_clean(fileList);
  for (int i = 0; i < nFiles; i++) {
    // this LVGL port's lv_list forces flex-ROW + content-height on buttons:
    // use a single label with both lines instead; fixed width makes the
    // built-in scroll clip to the name only (second line never scrolls).
    char rowtxt[80];
    snprintf(rowtxt, sizeof rowtxt, "%s\n%s",
             i == selFile ? files[i].nm : files[i].shortnm, files[i].mt);
    lv_obj_t* b = lv_list_add_btn(fileList, NULL, rowtxt);
    lv_obj_set_style_pad_all(b, 4, 0);
    lv_obj_set_height(b, 62);
    lv_obj_add_event_cb(b, filebtn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    for (uint32_t c = 0; c < lv_obj_get_child_cnt(b); c++) {
      lv_obj_t* ch = lv_obj_get_child(b, c);
      if (!lv_obj_check_type(ch, &lv_label_class)) continue;
      lv_obj_set_width(ch, 314);
      if (i == selFile) lv_obj_set_style_text_color(ch, lv_color_white(), 0);
    }
    if (i == selFile)
      lv_obj_set_style_bg_color(b, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
  }
  if (nFiles == 0) lv_list_add_text(fileList, "no csv files");
  setStatus(sdReady ? "Tap file, then PLAY" : "No SD card");
}
static void rec_cb(lv_event_t*) {
  if (mode == MODE_REC)       { rec_stop(); refresh_files(); setStatus("Saved"); }
  else if (mode == MODE_IDLE) rec_start();
}
static void play_cb(lv_event_t*) {
  if (mode == MODE_PLAY)  play_stop("Stopped");
  else if (mode == MODE_IDLE) play_start();
}
static void speed_cb(lv_event_t*) {
  speed = (speed == 1) ? 2 : (speed == 2) ? 4 : 1;
  lv_label_set_text_fmt(speedLbl, "%dx", speed);
}
static void back_cb(lv_event_t*) { lv_scr_load(ui_DashDisplay); }
// two-tap delete: first DEL arms for 3 s, second deletes the selected file
static uint32_t delArmedMs = 0;
static void del_cb(lv_event_t*) {
  bool armed = delArmedMs && millis() - delArmedMs < 5000;
  printf("[LOG] DEL tap sel=%d armed=%d\n", selFile, armed);
  if (mode != MODE_IDLE) { setStatus("Stop playback first"); return; }
  if (selFile < 0 || selFile >= nFiles) { setStatus("Select a file first"); return; }
  if (armed) {
    delArmedMs = 0;
    char name[48]; snprintf(name, sizeof name, "%.40s", files[selFile].disp);
    bool ok = SD_MMC.remove(files[selFile].path);
    printf("[LOG] delete %s -> %d\n", files[selFile].path, ok);
    selFile = -1;
    refresh_files();
    setStatus(ok ? "Deleted" : "Delete failed");
    lv_label_set_text(delLbl, "DEL");
  } else {
    if (delArmedMs) { lv_label_set_text(delLbl, "DEL"); setStatus("Timed out"); }
    delArmedMs = millis();
    lv_label_set_text(delLbl, "SURE?");
    setStatus("Tap DEL again within 5s");
  }
}
static void del_tick(void) {   // disarm visual when window lapses
  if (delArmedMs && millis() - delArmedMs >= 5000) {
    delArmedMs = 0;
    if (delLbl) lv_label_set_text(delLbl, "DEL");
  }
}

static lv_obj_t* mkBtn(lv_obj_t* parent, lv_coord_t w, lv_coord_t h,
                       const char* txt, lv_event_cb_t cb, lv_obj_t** lblOut) {
  lv_obj_t* b = lv_btn_create(parent);
  lv_obj_set_size(b, w, h);
  lv_obj_t* l = lv_label_create(b);
  lv_label_set_text(l, txt);
  lv_obj_center(l);
  lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, NULL);
  if (lblOut) *lblOut = l;
  return b;
}

static void open_cb(lv_event_t*) {
  if (!scr) {
    scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101010), 0);
    lv_obj_t* t = lv_label_create(scr);
    lv_label_set_text(t, "DATALOG");
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 24);

    statusLbl = lv_label_create(scr);
    lv_obj_align(statusLbl, LV_ALIGN_TOP_MID, 0, 48);

    fileList = lv_list_create(scr);
    lv_obj_set_size(fileList, 336, 232);
    lv_obj_align(fileList, LV_ALIGN_CENTER, 0, -16);
    lv_obj_set_style_text_font(fileList, &lv_font_montserrat_16, 0);

    progLbl = lv_label_create(scr);
    lv_obj_align(progLbl, LV_ALIGN_CENTER, 0, 118);

    lv_obj_t* b = mkBtn(scr, 84, 44, "REC", rec_cb, &recLbl);
    lv_obj_align(b, LV_ALIGN_BOTTOM_MID, -128, -64);
    b = mkBtn(scr, 84, 44, "PLAY", play_cb, &playLbl);
    lv_obj_align(b, LV_ALIGN_BOTTOM_MID, -36, -64);
    b = mkBtn(scr, 60, 44, "1x", speed_cb, &speedLbl);
    lv_obj_align(b, LV_ALIGN_BOTTOM_MID, 52, -64);
    b = mkBtn(scr, 56, 44, "DEL", del_cb, &delLbl);
    lv_obj_align(b, LV_ALIGN_BOTTOM_MID, 128, -64);
    b = mkBtn(scr, 70, 40, "BACK", back_cb, NULL);
    lv_obj_align(b, LV_ALIGN_BOTTOM_MID, 0, -4);
  }
  selFile = -1;
  refresh_files();
  lv_scr_load(scr);
}

// ---------------- public ----------------
void LogSystem_init(LogFeedFn feed) {
  feedFn = feed;
  lv_obj_t* entry = lv_btn_create(ui_DashDisplay);   // survives SquareLine regen
  lv_obj_set_size(entry, 70, 34);
  lv_obj_align(entry, LV_ALIGN_TOP_MID, 0, 6);
  lv_obj_t* l = lv_label_create(entry);
  lv_label_set_text(l, "LOG");
  lv_obj_center(l);
  lv_obj_add_event_cb(entry, open_cb, LV_EVENT_CLICKED, NULL);
}

void LogSystem_tick() {
  del_tick();
  if (mode != MODE_PLAY) return;
  uint32_t elapsed = (millis() - playStartMs);
  int budget = 50;
  while (budget-- > 0) {
    if (rowLoaded) {
      if ((uint64_t)elapsed < (uint64_t)curDueMs) break;  // curDue already speed-scaled
      play_emitRow();
    }
    if (!play_loadRow()) {
      play_stop("End of file");
      lv_label_set_text(progLbl, "done");
      return;
    }
  }
  if (playFile.size())
    lv_label_set_text_fmt(progLbl, "%d%%  %dx",
      (int)(100ULL * playFile.position() / playFile.size()), speed);
}

bool LogSystem_playbackActive() { return mode == MODE_PLAY; }

// --- test hooks (unit harness drives the static path without touch) ---
int LogSystem_testFileCount() { return nFiles; }
const char* LogSystem_testFileDisp(int i) { return (i >= 0 && i < nFiles) ? files[i].disp : ""; }
const char* LogSystem_testStatus() { static char s[128];
  snprintf(s, sizeof(s), "mode=%d sel=%d lbl=%s", mode, selFile,
           statusLbl ? lv_label_get_text(statusLbl) : "?"); return s; }
void LogSystem_testOpen()  { open_cb(NULL); }
void LogSystem_testRec()   { rec_cb(NULL); }
void LogSystem_testPlay(int idx) { if (idx >= 0) { selFile = idx; } play_cb(NULL); }
