# RoboLLM · UR5e VLA sim bed third-party notices

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../README.md) · [Bed README](README.md) · [Specification](SDD.md) · [References](REFERENCES.md) · [Documentation](../../docs/README.md)

RoboLLM is Copyright 2026 Santapong Sondhi and licensed under the Apache
License, Version 2.0 (repository `LICENSE` and `NOTICE`). This page lists what
the UR5e VLA sim bed depends on, under which terms, how it is used, and how
each obligation is met. Pins and ports marked *(P0)* are frozen at Phase 0 of
the [specification](SDD.md) and updated here in the same commit.

Principle: **use upstream unmodified at a pinned version, keep adaptations in
our own files, cite everything, redistribute nothing that is not needed.**

## Register

| Component | Version / pin | License | Copyright | How the bed uses it | Obligation, and how it is met |
|---|---|---|---|---|---|
| MuJoCo | 3.10.0 | Apache-2.0 | DeepMind Technologies | pip dependency (physics, offscreen renderer) | Cite `todorov2012mujoco`. Not redistributed. |
| MuJoCo Menagerie (repository) | commit `e4049d0` 2026-09-01 (confirmed at P0, 3 Sep 2026; sparse checkout of the two model directories, 46 MB) | Apache-2.0 | 2022 DeepMind Technologies Limited | `git clone` into a git-ignored `assets/` directory | Cite `menagerie2022github`. Not redistributed. |
| Menagerie `universal_robots_ur5e` | same commit | BSD-3-Clause | 2018 ROS-Industrial Consortium | `<include>`d unmodified by our overlay scene | Not redistributed, so no notice duty. If vendored: copy the model `LICENSE` beside it (see `examples/talos_mirror/ros2_ws/src/VENDORED.md`); never use the ROS-Industrial name to promote this work. |
| Menagerie `robotiq_2f85` | same commit | BSD-2-Clause | 2013 ROS-Industrial | `<attach>`ed unmodified by our overlay scene | As above. |
| mink | 1.3.0 | Apache-2.0 | Kevin Zakka | pip dependency (differential IK for the expert and the OXE replayer) | Cite `Zakka_Mink_Python_inverse_2026`. Not redistributed. |
| qpsolvers, daqp | 4.13.0, 0.9.1 | LGPL-3.0 (qpsolvers), MIT (daqp) | Stéphane Caron and contributors; Daniel Arnström | pulled by mink; daqp is the QP solver behind `solve_ik` | Not redistributed. |
| viser | 1.1.0 | Apache-2.0 | The viser authors | pip dependency (browser transport for the viewer) | Cite `yi2025viser`. Not redistributed. |
| mjviser | 0.0.14 | Apache-2.0 | 2025 The mjlab Developers | pip dependency (MuJoCo scene in the browser) | Cite `mjviser2026github`. Not redistributed. |
| pyzmq | 27.2.0 | BSD-3-Clause / LGPL (libzmq) | The pyzmq and ZeroMQ authors | pip dependency (lockstep sim server) | Not redistributed. |
| msgpack | 1.2.2 | Apache-2.0 | Inada Naoki and contributors | pip dependency (ZeroMQ payloads) | Not redistributed. |
| NumPy, Pillow | 2.5.2, 12.3.0 (floating) | BSD-3-Clause; MIT-CMU (HPND) | NumPy Developers; Jeffrey A. Clark and contributors | pip dependencies | Not redistributed. |
| LeRobot | 0.6.0 (`lerobot[smolvla]`) | Apache-2.0 | The HuggingFace Inc. team | pip dependency (dataset format, SmolVLA policy, training and evaluation) | Cite `cadene2024lerobot` and `cadenelerobot`. Not redistributed. |
| transformers, accelerate, num2words | 5.5.4, 1.14.0, 0.5.14 | Apache-2.0; Apache-2.0; LGPL-2.1 | Hugging Face; Hugging Face; Virgil Dupras and contributors | pulled by `lerobot[smolvla]` (VLM backbone, inference) | Not redistributed. |
| hf-libero, robosuite, bddl | 0.1.4, 1.4.0, 1.0.1 | MIT code, CC-BY-4.0 LIBERO data; MIT; MIT | LIBERO authors / Hugging Face packaging; ARISE Initiative; Stanford | P2b calibration only, in the separate `.venv-libero` (pins mujoco 3.8.1) | Cite `liu2023libero`. Not redistributed. |
| SmolVLA LIBERO checkpoint `lerobot/smolvla_libero` | model card revision at P2b | Apache-2.0 | LeRobot / Hugging Face | P2b calibration: evaluated on CPU, nothing kept beyond the evaluation JSON | Cite `shukor2025smolvla`. |
| SmolVLA weights `lerobot/smolvla_base` | revision `c83c3163` (downloaded 3 Sep 2026, 907 MB) | **No license tag on the model card** (3 Sep 2026) | LeRobot / Hugging Face | CPU inference measurement (P2) and starting checkpoint for fine-tuning (P4) | **Open item.** Ask on the model card or a LeRobot issue. Fine-tuned checkpoints are research artefacts and are **not published** until this closes. Cite `shukor2025smolvla`. |
| SmolVLM2-500M-Video-Instruct | as pulled by LeRobot | Apache-2.0 | Hugging Face TB | base VLM inside SmolVLA | Cite `smolvlm2_500m_video_instruct`. |
| `lerobot/berkeley_autolab_ur5` | revision `c4e26a697fc4c04776b0558f83e563d14be0109f` (4 Sep 2026): `meta/` + `data/chunk-000/file-000.parquet` only, no videos | CC-BY-4.0 | Berkeley AUTOLAB (Chen, Adebola, Goldberg); conversion by LeRobot | selected episodes downloaded for replay and optional co-training | Attribution: cite `BerkeleyUR5Website`, `lerobot_berkeley_autolab_ur5` and the OXE paper. The P3 side-by-side PNGs under `results/p3/` plot its end-effector paths and are derived from the Berkeley UR5 Demonstration Dataset (Chen, Adebola, Goldberg; CC-BY-4.0), as would any co-trained checkpoint. |
| Open X-Embodiment | paper | per-dataset | Embodiment Collaboration | context and the source of the UR5 dataset | Cite `embodimentcollaboration2025openxembodimentroboticlearning`. |
| LIBERO via `hf-libero` | 0.1.4 (optional P2b) | MIT (code), CC-BY-4.0 (data) | Bo Liu et al. | optional calibration run in its own venv | Cite `liu2023libero`. Not redistributed. |
| OmniSim | v8.1.17 at `19ea166` evaluated; Route O uses an unmodified build | Apache-2.0 (verbatim), with `NOTICE`, `TRADEMARKS.md`, `THIRD_PARTY_NOTICES.md` | OmniLink (derivative of Webots, Cyberbotics Ltd.) | Route O only: unmodified headless build on the workstation; ARM64 build fix contributed upstream | See below. Cite `omnisim2026` and `michel2004cyberbotics`. |

## OmniSim: what is and is not done

Route M contains **no OmniSim code**. The bed's harness, viewer and dataset
tooling are written against MuJoCo and LeRobot directly.

Route O runs OmniSim **unmodified**, so nothing is redistributed and no
modification notice is needed. Wording their trademark policy allows without
permission is the only wording used: "compatible with OmniSim", "runs on
OmniSim". Their name and the orb mark appear on nothing of ours.

The ARM64 build fix (an architecture switch in `scripts/install/qt_linux_installer.sh`
and the Qt package name in `dependencies/Makefile.linux`) is contributed
**upstream** under their Apache-2.0 terms with a `Signed-off-by` trailer (their
CONTRIBUTING uses the Developer Certificate of Origin, no CLA). Their CI checks
Apache headers on source files and requires a measurement for physics, controller
and world changes; a build-only patch's measurement is the Pi build log and
`doctor` reporting READY.

If a file of theirs is ever adapted into this repository, the Apache-2.0
conditions apply in full: keep the file's header, add
"Modifications copyright 2026 Santapong Sondhi", ship their `LICENSE` and
`NOTICE` beside it, mark the changed files, and copy none of the carve-outs
their `NOTICE` reserves (Code2000/2001/2002 fonts, the OmniLink display
typeface, brand artwork under `resources/branding/`).

## Network ports on the Pi *(P0)*

| Service | Port | Bound to |
|---|---|---|
| Viewer (viser websocket + static client) | **8090** (free on 3 Sep 2026; 8080 is nginx) | `100.74.8.82` (Pi tailnet IP) only — verified with `ss -ltnp`: `LISTEN 100.74.8.82:8090` |
| Sim server (ZeroMQ REQ/REP) | **5555** (free on 3 Sep 2026; bound at P5) | Pi tailnet IP only |

Neither service authenticates. No Funnel route, no LAN-wide bind.

## Fine-tuned checkpoints (Phase 4)

Checkpoints written under `artifacts/vla-bed/` (git-ignored) derive from `lerobot/smolvla_base`, whose weights carry no licence tag (SDD §12). They are **not published** and must not be pushed to any hub; the rented host's copy is moved to trash by `gpu/cleanup.sh` and the pod terminated. Evaluation JSONs under `results/p5/` contain no weights.
