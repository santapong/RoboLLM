# Validation record — 2026-09-06

Historical record for the original provisional model. The later photo-referenced model, revised claw/setup and fresh checks are documented in [MODEL-REVIEW-2026-09-07.md](MODEL-REVIEW-2026-09-07.md). The frozen evaluator and raw results below remain unchanged.

This record covers software and generated artifacts in `experiment/elexhub-webcam`, based on develop `838eafe`. It is not a physical-robot commissioning certificate.

| Check | Observed result |
|---|---|
| Repository unit suite | 109 passed; two dependency deprecation warnings. See `output/final-unit-tests.txt`. |
| Lint | Changed Python modules pass Ruff; EXE002 excluded because this mount reports all files executable. See `output/final-lint.txt`. |
| Firmware generation | Header matches the disabled physical profile. Unknown pulse endpoints are zero; enable and commissioning flags are false. |
| Arduino Mega compile | AVR compile succeeded: 9,710 bytes program, 1,102 bytes static RAM. No upload occurred. |
| Protocol | PTY handshake, mismatch refusal, explicit enable, bounded setpoints, malformed-state latch, timeout and recovery tested. No real USB port opened. |
| MuJoCo | Native model loads (nq=14, nv=13, nu=6), renders through EGL, agrees with independent FK over 20 sampled joint poses, and completes reference reach/pick-place tests. |
| URDF | Imports into MuJoCo with seven coordinates: five arm hinges and two jaw coordinates. Native MJCF provides the coupled gripper actuation used in the experiment. |
| CAD | FreeCAD 1.1.3: 23 valid solids. FCStd reopened; STEP reimported; volume and world placements agree. `assets/elexhub/provisional/CAD_VALIDATION.json` is the receipt. |
| PDFs | All five A4 pages rendered and visually inspected: three reference pages plus two wiring/measurement pages. No clipped text or overlapping elements observed. |
| Wiring diagram | Archify architecture; showcase validation 9/9, zero errors/warnings at delivery. Browser containment/readability checks pass at 1440×900, 1600×1000, 1920×1080, 2048×1320 and checked dark variants. Four saved screenshots were visually reviewed. |
| Browser console | Visible local-browser preview → clear-workspace confirmation → reach completion checked. Full grasp/lift/release sequence also completed in simulation. Physical enable is absent in simulation mode. |
| Perception runtimes | Downloaded versioned official Hand Landmarker/Vosk models; blank synthetic frame returned zero hands; synthetic silence returned empty text. This does not measure real webcam/speech accuracy. |
| Hardware / Pi | Not operated or deployed. ROS Jazzy is absent on this host, so live ROS action/service integration and real-device behavior remain unverified. |

## Frozen synthetic comparison

The evaluator recorded 100 matched goal seeds for each of A/B/C, preserving 300 rows and a full manifest/source snapshot. It held the 15 mm threshold for 0.5 simulation seconds. Every archived source hash and the paired seed sets were checked.

| Condition | Success | Wilson 95% interval | Median endpoint error | P90 error |
|---|---:|---:|---:|---:|
| A nominal | 100/100 | 96.3–100% | 5.16 mm | 6.88 mm |
| B model compensation | 100/100 | 96.3–100% | 1.80 mm | 2.46 mm |
| C B + bounded feedback | 100/100 | 96.3–100% | 1.80 mm | 2.46 mm |

C reads simulator TCP. B/C endpoint errors are identical for every matched seed. This suite has a success ceiling and demonstrates no incremental C benefit. Paired bootstrap success intervals are degenerate here and must not be interpreted as proof of equivalence. No physical accuracy, RGB-control improvement or sim-to-real transfer is established.

Evidence is under `output/elexhub-evaluation-2026-09-06/`: immutable recorded rows, manifest, source snapshot, exact environment lock and report. Later changes to console diagnostics and host safeguards do not replace that archived evaluator. Run a new evaluation into a new directory; the CLI refuses to overwrite an existing trial file.

## Isolation and remaining gates

This implementation used `/mnt/data/RoboLLM-elexhub`, its dedicated Python 3.12 environment and a separate experiment branch. The original `/home/santapong/RoboLLM` checkout was not edited. A read-only final check found it clean on `experiment/ur5e-vla-bed` at `abac541`, independently advanced from the implementation's initial observation. No Kaggle API, notebook, dataset, checkpoint or training operation was invoked; no statement here establishes the live status of a Kaggle job.

Required next evidence: actual arm dimensions and servo/battery labels; inspected wiring and rated power/cutoff; joint and pulse calibration; measured gripper linkage; webcam intrinsics, base/tool registration and independent held-out pixel-to-table accuracy; live ROS/serial watchdog tests; then the prepared physical trials. `readiness.json` lists the missing profile evidence. The 158 physical trial rows are templates with blank outcomes.

## Diagram delivery identity

Specification SHA-256: `e94e060b7d6878b64914e11c1b76be1e7c7978727c746bb68148813e0c492b44`.

HTML SHA-256: `85fe0fe77a3f7c14490abf5eeaf86ec8711cfe792dbe7e94832c6d1d4b528735` (710,615 bytes).

`wiring.visual-check.json` records automated browser evidence; the visual review described above is a separate image review. No generated HTML was patched after delivery. The checked setup lessons are saved in [LESSONS-2026-09-06.md](LESSONS-2026-09-06.md), alongside the runbook for future retrieval.
