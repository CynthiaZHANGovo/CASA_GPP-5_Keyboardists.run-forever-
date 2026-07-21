/*
 * ============================================================================
 *  hit_detection_eval.c  —  Simulation-based evaluation of the hit detector
 * ============================================================================
 *
 *  WHAT THIS IS (and honestly, what it is NOT)
 *  ------------------------------------------------------------------------
 *  This program feeds a SYNTHETIC FSR signal through the exact same detection
 *  logic used on the device (EMA low-pass filter + IDLE/ARMED/REFRACT state
 *  machine with hysteresis and a refractory window) and measures how well it
 *  separates real hits from noise, compared against a naive bare-threshold
 *  detector.
 *
 *  It validates the DE-BOUNCE / SIGNAL-PROCESSING logic only. The synthetic
 *  signal is a model of an FSR trace (baseline + noise + hit pulses + glitches),
 *  NOT a real sensor. Absolute physical accuracy of the real FSR must still be
 *  measured on hardware; this quantifies the ALGORITHM, which behaves the same
 *  in simulation as on-device because it is pure DSP + logic.
 *
 *  GROUND TRUTH: because we synthesise the signal ourselves, we know exactly
 *  at which sample indices a real hit was injected. We compare each detector's
 *  reported hits against that truth to count:
 *      - true positives  (real hit correctly detected)
 *      - false negatives (real hit missed)
 *      - false positives (a hit reported where there was only noise)
 *
 *  Build & run (on any PC, no board needed):
 *      cc hit_detection_eval.c -o eval -lm && ./eval
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* ---- must match the on-device tunables in main.cpp --------------------- */
#define EMA_ALPHA        0.30f
#define HIT_THRESHOLD    4.5f     /* force units (0..10) to fire            */
#define RELEASE_THRESH   2.0f     /* must fall below this to re-arm         */
#define REFRACTORY_MS    40       /* ignore re-trigger for this long        */
#define ADC_SPAN         600.0f   /* counts from baseline to "max force"    */
#define SAMPLE_MS        5        /* 200 Hz sampling, same as TaskSensing   */

/* ---- simulation parameters -------------------------------------------- */
#define BASELINE         120      /* resting ADC counts                     */
#define NOISE_AMP        8        /* +/- baseline noise (sensor floor)      */
#define DURATION_SAMPLES 4000     /* 4000 * 5ms = 20 s of data              */
#define TARGET_HIT_PEAK  480      /* real hit peak above baseline (counts)  */
#define GLITCH_AMP       340      /* spurious spike amplitude (counts): a
                                     single-sample spike that DOES cross the
                                     bare threshold and fools a naive detector,
                                     but is too brief to survive EMA smoothing  */
#define GLITCH_COUNT     60       /* number of spurious spikes                 */
#define TOLERANCE_SAMP   6        /* a detection within +/-6 samples of a
                                     ground-truth hit counts as correct     */

/* ---- helpers ---------------------------------------------------------- */
static float counts_to_force(float filtered) {
    float over = filtered - BASELINE;
    if (over < 0) over = 0;
    return (over / ADC_SPAN) * 10.0f;
}

/* deterministic pseudo-noise so runs are reproducible */
static int noise(void) { return (rand() % (2 * NOISE_AMP + 1)) - NOISE_AMP; }

/* ============================================================================
 *  1. SIGNAL GENERATOR
 *     Builds a synthetic FSR trace and records ground-truth hit indices.
 * ========================================================================== */
static int  raw[DURATION_SAMPLES];
static int  truthHits[64];
static int  truthCount = 0;

static void generate_signal(void) {
    /* start from a noisy baseline everywhere */
    for (int i = 0; i < DURATION_SAMPLES; ++i)
        raw[i] = BASELINE + noise();

    /* inject 30 real hits at roughly regular spacing.
       each hit is a fast rise -> peak -> decay pulse (~12 samples wide). */
    for (int h = 0; h < 30; ++h) {
        int center = 120 + h * 125;          /* spread across the trace     */
        if (center + 15 >= DURATION_SAMPLES) break;
        truthHits[truthCount++] = center;
        for (int k = -4; k <= 8; ++k) {
            /* triangular-ish pulse shape */
            float shape = (k <= 0) ? (1.0f + k / 4.0f)      /* rise  */
                                   : (1.0f - k / 8.0f);     /* decay */
            if (shape < 0) shape = 0;
            raw[center + k] += (int)(TARGET_HIT_PEAK * shape);
        }
    }

    /* sprinkle short (1-2 sample) noise glitches that ARE tall enough to cross
       the bare threshold (so a naive detector fires on them) but are too brief
       to survive the EMA low-pass. These are the false-positive bait. */
    for (int g = 0; g < GLITCH_COUNT; ++g) {
        int pos = rand() % (DURATION_SAMPLES - 2);
        int nearHit = 0;
        for (int t = 0; t < truthCount; ++t)
            if (abs(pos - truthHits[t]) < 15) { nearHit = 1; break; }
        if (!nearHit) raw[pos] += GLITCH_AMP;   /* one-sample tall spike */
    }
}

/* ============================================================================
 *  2a. BARE-THRESHOLD DETECTOR (the naive baseline / control group)
 *      Fires whenever a single raw sample crosses the threshold. No filter,
 *      no state machine. This is what the project would do without de-bounce.
 * ========================================================================== */
static int detect_bare(int *reported, int maxRep) {
    int n = 0;
    for (int i = 0; i < DURATION_SAMPLES; ++i) {
        float force = counts_to_force((float)raw[i]);
        if (force >= HIT_THRESHOLD) {
            /* naive: also require the previous sample was below, so one
               sustained pulse isn't counted dozens of times */
            float prev = counts_to_force((float)raw[i ? i - 1 : 0]);
            if (prev < HIT_THRESHOLD && n < maxRep) reported[n++] = i;
        }
    }
    return n;
}

/* ============================================================================
 *  2b. FULL DETECTOR — the real on-device algorithm
 *      EMA low-pass + IDLE/ARMED/REFRACT state machine + refractory window.
 *      This mirrors TaskSensing exactly.
 * ========================================================================== */
enum { IDLE, ARMED, REFRACT };

static int detect_full(int *reported, int maxRep) {
    int   n = 0;
    float ema = BASELINE;
    int   fsm = IDLE;
    int   refractUntil = 0;
    int   refractorySamples = REFRACTORY_MS / SAMPLE_MS;

    for (int i = 0; i < DURATION_SAMPLES; ++i) {
        ema = EMA_ALPHA * raw[i] + (1.0f - EMA_ALPHA) * ema;
        float force = counts_to_force(ema);

        switch (fsm) {
            case IDLE:
                if (force >= HIT_THRESHOLD) {
                    if (n < maxRep) reported[n++] = i;   /* fire */
                    fsm = ARMED;
                    refractUntil = i + refractorySamples;
                }
                break;
            case ARMED:
                if (force < RELEASE_THRESH && i >= refractUntil)
                    fsm = REFRACT;
                break;
            case REFRACT:
                fsm = IDLE;
                break;
        }
    }
    return n;
}

/* ============================================================================
 *  3. SCORING — compare a detector's reported hits against ground truth
 * ========================================================================== */
typedef struct { int truePos, falseNeg, falsePos; } Score;

static Score score(int *reported, int repCount) {
    Score s = {0, 0, 0};
    int matchedTruth[64] = {0};
    int matchedRep[512]  = {0};

    /* match each ground-truth hit to a nearby reported hit */
    for (int t = 0; t < truthCount; ++t) {
        for (int r = 0; r < repCount; ++r) {
            if (!matchedRep[r] && abs(reported[r] - truthHits[t]) <= TOLERANCE_SAMP) {
                matchedTruth[t] = 1;
                matchedRep[r]   = 1;
                s.truePos++;
                break;
            }
        }
    }
    for (int t = 0; t < truthCount; ++t) if (!matchedTruth[t]) s.falseNeg++;
    for (int r = 0; r < repCount;   ++r) if (!matchedRep[r])   s.falsePos++;
    return s;
}

/* ============================================================================
 *  4. REPORT
 * ========================================================================== */
int main(void) {
    srand(42);                 /* fixed seed => reproducible results */
    generate_signal();

    static int bareRep[512], fullRep[512];
    int bareN = detect_bare(bareRep, 512);
    int fullN = detect_full(fullRep, 512);

    Score bare = score(bareRep, bareN);
    Score full = score(fullRep, fullN);

    printf("=== Hit-Detection Evaluation (SIMULATED signal) ===\n");
    printf("Ground-truth real hits injected : %d\n", truthCount);
    printf("Trace length                    : %d samples (%.1f s @ %dms)\n\n",
           DURATION_SAMPLES, DURATION_SAMPLES * SAMPLE_MS / 1000.0, SAMPLE_MS);

    printf("%-22s %10s %10s\n", "", "BARE thr.", "FULL algo");
    printf("%-22s %10d %10d\n", "hits reported",     bareN, fullN);
    printf("%-22s %10d %10d\n", "true positives",    bare.truePos,  full.truePos);
    printf("%-22s %10d %10d\n", "false negatives",   bare.falseNeg, full.falseNeg);
    printf("%-22s %10d %10d\n", "false positives",   bare.falsePos, full.falsePos);

    float bareRecall = 100.0f * bare.truePos / truthCount;
    float fullRecall = 100.0f * full.truePos / truthCount;
    printf("%-22s %9.1f%% %9.1f%%\n", "recall (hit rate)", bareRecall, fullRecall);

    /* precision = TP / (TP + FP): of everything it called a hit, how many real */
    float barePrec = 100.0f * bare.truePos / (bare.truePos + bare.falsePos + 1e-6);
    float fullPrec = 100.0f * full.truePos / (full.truePos + full.falsePos + 1e-6);
    printf("%-22s %9.1f%% %9.1f%%\n", "precision", barePrec, fullPrec);

    printf("\nFalse triggers removed by filtering + refractory: %d -> %d\n",
           bare.falsePos, full.falsePos);
    printf("\nNOTE: validates de-bounce ALGORITHM on synthetic data.\n");
    printf("      Real-sensor absolute accuracy is pending on-hardware test.\n");
    return 0;
}
