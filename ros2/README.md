# ROS 2 physical-arm workspace

This tree contains the installable ROS 2 packages for the physical RoboLLM arm.
Simulation and learning examples remain under `examples/`; hardware-facing code
graduates here only when it is part of the supported physical-arm path.

Current milestone: **v0.2** (`JointTrajectory` -> validated Arduino commands).
The URDF and MoveIt packages will be added after the Phase 0 measurement sheet
is complete so that guessed geometry never becomes a hidden safety assumption.

This milestone is code-complete only at the hardware-free foundation level.
ROS Jazzy build/launch evidence and physical motion acceptance remain open in
`../docs/physical-arm/ROADMAP.md`.
