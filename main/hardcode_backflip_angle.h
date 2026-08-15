// hardcode_backflip_angle.h  --  "Backflip 3"
//
// 6 keyframes taught by hand through the web Teach UI, then hardcoded here
// so they survive a reflash. The taught buffer lives in RAM only: flashing,
// a power cycle or a watchdog reset loses it, which is what this file is for.
//
// DESIGN (calibration-safe):
//   BF3_REF[]     = frame 0, absolute SCS (0..1023)
//   BF3_DELTA[][] = frames 1..N as signed offsets from BF3_REF
//
//   load_bf3() computes  rec_frames[f][id] = BF3_REF[id] + BF3_DELTA[f-1][id]
//
//   After recalibration only BF3_REF needs re-teaching; the deltas are the
//   relative joint motion and do not change.
//
// Play with:  bfload3 then play   (or one-shot: bf3 / the Play backflip 3 button)
// SlowMo must be OFF for BF3_MOVE_MS/BF3_DELAY_MS below to be used at all -
// while it is ON, Play ignores per-frame timing and uses the global play_ms.
#pragma once
#include <stdint.h>

#define BF3_FRAMES 6

// ---- REFERENCE POSE (frame 0) ----
static const uint16_t BF3_REF[13] = {
    /* idx  0     1     2     3     4     5     6     7     8     9    10    11    12 */
              0,    48,   509,   596,    54,   474,   529,    47,   507,   507,    50,   514,   511
};
_Static_assert(sizeof(BF3_REF) / sizeof(BF3_REF[0]) == 13,
               "BF3_REF needs 13 entries: unused [0] + servos 1..12");

// ---- DELTA FRAMES (frames 1..5, relative to BF3_REF) ----
static const int16_t BF3_DELTA[][13] = {
    {    0,     0,  -113,   -33,     0,   120,    88,     1,     4,  -127,     0,   -30,   150},  /* frame 1 */
    {    0,     0,   117,   106,     0,  -125,   -61,     1,    -2,  -129,     0,   -25,   153},  /* frame 2 */
    {    0,     1,   117,   106,     1,  -127,   -60,     1,  -393,   206,     0,   373,  -219},  /* frame 3 */
    {    0,     0,   117,   106,     1,  -127,   -60,     1,  -318,   439,     1,   320,  -470},  /* frame 4 */
    {    0,     2,   281,  -205,     2,  -290,   270,     1,  -210,    21,     1,   162,    25},  /* frame 5 */
};

// BF3_MOVE_MS[f]  = time to MOVE into frame f from the previous pose.
// BF3_DELAY_MS[f] = time to HOLD on frame f after arriving.
// [0] is the approach from the Ini stance into BF3_REF - keep it slow.
static const int BF3_MOVE_MS[]  = {  800,  200,  200,  200,  200,  200 };
static const int BF3_DELAY_MS[] = {  300,    0,    0,    0,    0,    0 };

_Static_assert(sizeof(BF3_DELTA)    / sizeof(BF3_DELTA[0])    == BF3_FRAMES - 1,
               "BF3_DELTA row count must be BF3_FRAMES - 1");
_Static_assert(sizeof(BF3_MOVE_MS)  / sizeof(BF3_MOVE_MS[0])  == BF3_FRAMES,
               "BF3_MOVE_MS must have exactly BF3_FRAMES entries");
_Static_assert(sizeof(BF3_DELAY_MS) / sizeof(BF3_DELAY_MS[0]) == BF3_FRAMES,
               "BF3_DELAY_MS must have exactly BF3_FRAMES entries");
