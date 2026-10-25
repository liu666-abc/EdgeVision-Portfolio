# Motion/IoU tracking baseline

## 2026-10-22

### Input

- Intel `one-by-one-person-detection.mp4`
- `768x432`, source `10 FPS`
- Published as `/camera/color/image_raw`
- YOLOv8n INT8 on RDK X5

### Tracker

- Kalman constant-velocity prediction
- Class-aware IoU association
- Center-distance fallback for fast-moving boxes
- Two-stage association using high and low score detections
- New tracks displayed after two consecutive hits
- Missed tracks retained for about 2 seconds

### Verified

- Offline tracker self-test passes in Release mode.
- Upper computer shows `track_id`:
  - person `#1` at 87% in the single-person video.
- Pipeline remains around `10 FPS`, matching the source frame rate.
- End-to-end detector timing at 10 FPS sample: about `26-46 ms`.

### Boundary

- This is not a full ByteTrack implementation.
- MOTA, IDF1 and true ID Switch counts require an annotated sequence and are not claimed.
- Dense-crowd `vtest.avi` is retained as a stress case; it is not used as evidence of stable IDs.

### MOT17-mini 8-frame smoke result

The first 8 frames of BoxMOT `MOT17-mini/MOT17-04-FRCNN` were evaluated against
MOTChallenge ground truth with IoU `0.5`:

| Metric | Value |
|---|---:|
| Precision | `100.00%` |
| Recall | `15.48%` |
| MOTA | `15.48%` |
| IDF1 | `26.80%` |
| ID Switch | `0` |
| Unique GT IDs | `8` |
| Unique predicted IDs | `8` |

Full evidence: `evidence/2026-10-22_mot17-mini/`.

**This is not a MOT benchmark and does not mean the MOT target is met.**
The sequence has only 8 frames and dense small-target recall is very low.

The low recall is caused by detecting only 5-8 strong pedestrians in a dense
1920x1080 frame after 640x640 input scaling. It is not evidence that the tracker
changed IDs; the 52 matched outputs retained 8 distinct IDs with zero switches.

### 30-minute soak result

The detector + tracker + dashboard chain completed 30 minutes at forced 25 FPS:

- camera: mean `24.899 FPS`, p95 `25.050 FPS`;
- detector: mean `24.904 FPS`, p95 `25.040 FPS`;
- total latency: mean `24.825 ms`, p95 `26.948 ms`;
- BPU temperature max `49.7 °C`;
- DDR temperature max `51.1 °C`;
- BPU/DDR frequency stayed at `1000/4266 MHz`;
- API failures: `0`;
- memory start/end: `1099/1099 MB`.

Full evidence: `evidence/2026-10-22_soak30/`.

### 10-minute heterogeneous concurrency

The 25 FPS video, BPU detector, CPU tracker, MJPEG dashboard, and a 30 Hz
synthetic depth median/Gaussian workload ran together for 10 minutes:

- CPU utilization: mean `46.16%`, p95 `47.24%`;
- camera/detector mean: `24.896/24.893 FPS`;
- total latency: mean `24.546 ms`, p95 `26.268 ms`;
- deadline misses over 40 ms: `0/120`;
- CPU/BPU/DDR max temperature: `52.6/52.1/53.5 °C`;
- memory delta: `+3 MB`.

Full evidence: `evidence/2026-10-22_hetero10/`.

### Fault recovery

Automated SIGINT fault injection passed:

- detector offline detected: `2.050 s`;
- detector recovered: `18.641 s`;
- video loss detected: `5.790 s`;
- video recovered: `14.865 s`;
- all steps and final restore: PASS.

Recovery is currently triggered by the test script, not a production watchdog.
Full evidence: `evidence/2026-10-22_fault-recovery/`.

### One-command regression

The regression suite passed in `34.939 s`:

- tracker self-test: PASS;
- MOT evaluator self-test: PASS;
- static 10-image counts: exact match `[6,1,4,1,3,2,2,2,4,10]`;
- live camera/detector/tracker/CPU contract: PASS.

Full evidence: `evidence/2026-10-22_regression/`.

### Invalid dual-stream result (do not use)

**The following run was contaminated by a leftover MOT video source. The values
are retained only for audit and cannot be used for scheduling conclusions.**

Two 25 FPS video streams and two detector instances were run together:

- detector A output: `25.060 Hz`;
- detector B output: `14.768 Hz`;
- A camera/detector monitor mean: `24.914/24.918 FPS`;
- CPU: mean `41.828%`;
- BPU: mean `15.133%`;
- frame-budget misses in the monitored A path: `0/60`.

The cause of the low B-stream value in this contaminated run was not isolated.
It must not be attributed to BPU saturation, CPU saturation, or the tracker.

Full evidence: `evidence/2026-10-22_dual-stream5/`.

### Clean dual-stream rerun

The first dual-stream result was contaminated by a leftover MOT video source.
After killing that process and restarting cleanly for 10 minutes:

- camera A/B: both mean `25.011 FPS`;
- detector A/B: mean `22.297/23.925 FPS`;
- detector B/A ratio: mean `1.0767`;
- CPU: mean `54.305%`;
- BPU: mean `17.492%`;
- frame latency p95: A `28.061 ms`, B `27.369 ms`;
- memory: `1076 -> 1080 MB`.

This clean run did not reproduce the earlier B-stream drop to about 15 FPS.
Full evidence: `evidence/2026-10-23_dual-stream10/`.

### Frame-log I/O A/B

Two 2-minute dual-stream runs compared per-frame CSV/track logging:

| Metric | logging on | logging off |
|---|---:|---:|
| detector A mean | `24.7825 FPS` | `24.9806 FPS` |
| detector B mean | `24.7633 FPS` | `24.8694 FPS` |
| CPU mean | `58.104%` | `54.235%` |
| BPU mean | `19.125%` | `18.750%` |

Per-frame logging is a measurable but secondary overhead. It does not explain
the invalid 14.8 Hz run. The demo now defaults to logging off and uses
`EVI_DUAL_LOGGING=1` only when collecting raw evidence.

Full evidence: `evidence/2026-10-23_dual-log-ab/`.

### Decoy causality experiment

A five-phase fenced experiment tested an idle DDS decoy and a continuous 5 FPS
stray video source. The experiment did **not** confirm the earlier 14.8 Hz cause:

- clean baseline detector A/B: `21.194/24.601 FPS`;
- idle decoy: `21.653/24.450 FPS`;
- after removing idle decoy: `14.362/24.840 FPS`;
- continuous 5 FPS decoy: `16.191/23.953 FPS`;
- after removing continuous decoy: `17.573/24.410 FPS`.

The starvation moved to detector A while B remained near 25 FPS. CPU and BPU were
not saturated. The leftover source is a contaminant, but not a sufficient root
cause. QoS depth, CPU affinity, executor fairness, and DDS queueing remain open.

Full evidence: `evidence/2026-10-23_decoy-causality/`.

The QoS variable was isolated in the follow-up experiment below: best-effort
image delivery was the confirmed cause for the reproducible starvation.

### Confirmed QoS root cause

The reproducible dual-stream starvation was isolated to image subscription
reliability:

| Image QoS | detector A | detector B |
|---|---:|---:|
| depth 1, best effort | `24.812` | `7.551` |
| depth 5, best effort | `24.214` | `16.130` |
| depth 5, reliable, 2 min | `25.007` | `25.013` |
| depth 5, reliable, 10 min | `25.011` | `25.014` |

Queue depth alone is not the fix. Reliable delivery is the causal fix for a
reliable video publisher.

Full evidence: `evidence/2026-10-23_qos-root-cause/`.

### Real Astra S QoS verification

On 2026-10-23 the current Astra S color and depth publishers were both
`RELIABLE`. The C++ detector used `depth=5 + reliable` and ran for 2 minutes:

- camera mean: `29.429 FPS`;
- detector mean: `29.602 FPS`;
- total latency p95: `24.364 ms`;
- deadline misses over 40 ms: `0`;
- API failures: `0`.

The scene contained no detected targets, so this is not evidence of recall,
precision, MOT or tracking quality. The camera was attached on USB 2.0 and the
kernel log contained earlier buffer-overrun warnings.

Full evidence: `evidence/2026-10-23_astra-qos/`.

