// FuelEdu — bus-side trip computer (instant + trip-average MPG) for T5Gauge21.
// Inputs (ESP-NOW bus domain, interpreter-scaled): Rpm = rpm, Insptid_ms10 = ms,
// Bil_hast = km/h. Appends an "mpg" swipe page AFTER ui_clock in ui_SingleScr;
// created at runtime so SquareLine regeneration never eats it (LogSystem precedent).
#ifndef FUELEDU_H
#define FUELEDU_H

#include <Arduino.h>

// Called from setup() AFTER ui_init(); appends the page.
void FuelEdu_init(void);

// Called every loop() with millis(); integrates + refreshes labels.
void FuelEdu_tick(unsigned long nowMs);

// Reseed the trip average to the prior (same as long-press on the avg line).
// Resets the bank currently on screen: playback reset = RAM only, live reset persists.
void FuelEdu_resetTrip(void);

// Live sample hook: call from live ESP-NOW ingest with (Rpm, Insptid_ms10,
// Bil_hast) on EVERY frame — including during playback. Feeds the persisted,
// real trip.
void FuelEdu_liveSample(float rpm, float pwMs, float kmh);

// Playback hook: same triple from a log row; feeds the RAM-only sandbox bank
// the display shows while playback runs. Never touches NVS.
void FuelEdu_playSample(float rpm, float pwMs, float kmh);

// Called when playback starts: sandbox restarts from a clean prior. RAM only.
void FuelEdu_beginSandbox(void);

#endif
