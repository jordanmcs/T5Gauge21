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
void FuelEdu_resetTrip(void);

// Enter/exit playback sandbox: real trip is flushed to NVS, display gets a
// fresh seeded session that never writes flash, and the real trip is restored
// from NVS when playback stops.
void FuelEdu_beginSandbox(void);

#endif
