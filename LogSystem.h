// LogSystem — SD-card datalogging + CSV playback for T5Gauge21
// Format: LogConverter-style CSV ("Time,<SYM>,..." elapsed seconds).
// New logs use live ESP-NOW symbol ids; legacy CanTraceExt columns remap.
#pragma once
#include <Arduino.h>

typedef void (*LogFeedFn)(const char* id, float value);

void LogSystem_init(LogFeedFn feed);   // pins LOG entry button on dash screen
void LogSystem_tick();                 // playback scheduler (call from loop)
void rec_sample();                     // append current values (from processJSON)
bool LogSystem_playbackActive();       // ESP-NOW ingest must pause while true

// test hooks for the host unit harness
int  LogSystem_testFileCount();
const char* LogSystem_testFileDisp(int i);
const char* LogSystem_testStatus();
void LogSystem_testOpen();
void LogSystem_testRec();
void LogSystem_testPlay(int idx);

// implemented by the sketch: current value of one live symbol (0 if unseen)
float sensorValueFor(const char* id);
