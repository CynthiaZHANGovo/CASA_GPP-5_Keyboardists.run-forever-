# Hit-Detection Evaluation (simulation)

Quantifies the de-bounce behaviour of the on-device hit detector (EMA low-pass +
IDLE/ARMED/REFRACT state machine + refractory window) on a **synthetic FSR
signal**, against a naive bare-threshold detector as the control group.

This validates the **signal-processing / de-bounce algorithm**, which behaves
the same in simulation as on-device because it is pure DSP + logic. It does
**not** measure the real sensor's absolute accuracy — that is pending an
on-hardware test.

## Run (no board needed)

```bash
cc hit_detection_eval.c -o eval -lm && ./eval
```

## What it does

1. Generates a 20 s synthetic FSR trace: noisy baseline + 30 injected real hit
   pulses + 60 tall single-sample glitches (false-positive bait). Because the
   signal is synthesised, the exact hit locations are known (ground truth).
2. Runs the trace through two detectors:
   - **bare threshold** — fires on any raw sample crossing the threshold (no
     filter, no state machine): the naive control.
   - **full algorithm** — the real `TaskSensing` logic (EMA + state machine +
     refractory).
3. Scores each against ground truth (true / false positives, misses) and prints
   a comparison.

A fixed RNG seed (42) makes the result reproducible.

## Representative result

| metric              | bare threshold | full algorithm |
|---------------------|---------------:|---------------:|
| hits reported       | 77             | 30             |
| true positives      | 30             | 30             |
| false positives     | 47             | 0              |
| recall (hit rate)   | 100%           | 100%           |
| precision           | 39%            | 100%           |

The filter + refractory window removes the noise-induced false triggers (47 → 0)
while keeping every real hit. The glitches cross the threshold but are too brief
to survive the EMA low-pass.

> Numbers depend on the synthetic noise parameters; they characterise the
> algorithm's de-bounce ability, not real-sensor accuracy.
