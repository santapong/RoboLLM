# Paper-to-project research brief

Reviewed 2026-09-06. Sources are primary paper versions and official hardware/software documentation. The adaptation below is an engineering proposal for this Elexhub setup, not a reproduction claim or a statement of state of the art.

## Reading and adaptation register

| Source | Relevant finding / method | What this implementation adopts | What does not transfer |
|---|---|---|---|
| [Hajdúch & Kocur, Pointing-Based Object Recognition, v1](https://arxiv.org/abs/2603.15403v1) | RGB pointing-target recognition combines pose, object detection and reconstructed spatial information. | Explicit source-object selection and ambiguity rejection; an RGB-only baseline. | This implementation uses calibrated tabletop projection rather than their reconstructed 3D pipeline. Recognition performance is not robot execution performance. Version 2 exists; this adaptation is tied to the reviewed v1. |
| [Lai et al., Natural Multimodal Fusion-Based Human-Robot Interaction, v3](https://arxiv.org/abs/2501.00785v3) | Voice and deictic selection are temporally associated, with constrained action syntax; physical evaluation uses UR3e. | Separate source/destination latches, short command grammar, preview and explicit confirmation. | The paper's depth sensing, LLM pipeline and industrial-arm execution are not present in this constrained RGB/PWM baseline. See [author code](https://github.com/laiyuzhi/NMM-HRI); root-level repository inspection does not establish a reproduced system. |
| [Kavousian et al., Intuitive Human-Robot Interaction, v1](https://arxiv.org/abs/2604.06073v1) | Evaluates a pointing/click object-selection interface with 20 participants. | Visible target feedback and separate intent accuracy, selection-time and motor-success metrics. | A user-selection study does not establish grasp reliability or safety of autonomous arm motion. |
| [Oaki et al., dynamic parameter identification for a low-cost arm, v2](https://arxiv.org/abs/2605.15949v2) | CRANE-X7 identification uses structured motions, physical-feasibility checks and held-out validation. | Explicit identification records, bounded calibration motions and held-out checks before activating compensation. | Smart-actuator feedback and inverse-dynamics identification cannot be assumed for hobby servos driven by PCA9685. PWM targets alone are not measured torque or angle. |
| [VILAS, v2](https://arxiv.org/abs/2605.02037v2) | Separates perception/policy components, grasping and execution in a robot manipulation stack. | A clean interface between interpretation, planning, hardware execution and observation. | Its robot/compute setup and VLA results do not validate a Pi/Mega/Elexhub implementation. No VLA training or checkpoint transfer is introduced here. |

The papers motivated three useful comparisons: interpretation versus execution failure; nominal commands versus identified actuator compensation; and model compensation versus measured visual correction. They do not provide Elexhub geometry, servo calibration, battery compatibility or a reason to alter the running UR5e training.

## Proposed contribution and evidence needed

The bounded contribution is a reproducible RGB-guided tabletop experiment on a low-cost, uninstrumented servo arm. The central question is whether external tool-marker feedback reduces endpoint error after actuator compensation. First measure the arm and camera; then compare identical target sets and starting poses. Record perception failures separately, retain aborted attempts, and publish continuous error with uncertainty alongside task success.

The delivered synthetic comparison is an internal control/software check. A and B use the same provisional physics model; B compensates gravity from that model. C uses simulator TCP, with a 4 mm stopping threshold and bounded 2 mm corrections. This gives an optimistic, noise-free observation baseline. **It does not test the paper-derived RGB hypothesis.** A future measured experiment must include webcam calibration error, occlusion, servo backlash, delay and load effects, using independently recorded observations.

The first 100 paired simulated goals all pass the 15 mm threshold in A/B/C. B and C have identical endpoint errors in this suite, so it demonstrates no incremental benefit from C. This ceiling effect must remain visible. Do not tighten the existing threshold after seeing results; define a new versioned stress suite or physical protocol before additional comparisons. A degenerate paired bootstrap interval at this ceiling is not proof of equivalence.

## Primary implementation references

- [NXP PCA9685 datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf): PWM output, prescale/timing and OE behavior. It does not supply servo power regulation or position sensing.
- [Adafruit PCA9685 wiring guide](https://learn.adafruit.com/16-channel-pwm-servo-driver/hooking-it-up): separate logic and servo power on a common breakout design. Inspect the actual Elexhub/module wiring before use.
- [MuJoCo modeling documentation](https://mujoco.readthedocs.io/en/stable/modeling.html): MJCF bodies/joints, inertias, actuators and contact modeling.
- [OpenCV 4.10 ChArUco detection](https://docs.opencv.org/4.10.0/df/d4a/tutorial_charuco_detection.html): the printed calibration-board pipeline. Explicit mat coordinates avoid silently adopting a reversed board-normal convention.
- [MediaPipe Hand Landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker/python): local hand-landmark inference. The selected model artifact is recorded by URL and hash.
- [Vosk](https://alphacephei.com/vosk/): offline speech recognition, used only to prepare a constrained, reviewable text instruction.

`tests/unit/test_elexhub.py` verifies the applied contracts: kinematic parity, finite/bounded input, profile/runtime isolation, protocol recovery, freshness and frame conventions, ambiguous selection, preview confirmation and contact-based lift verification. These software checks complement the papers; they are not physical validation.
