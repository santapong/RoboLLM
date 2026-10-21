# Demonstration storyboard — target duration 2:45

Use this after the implementation passes its acceptance gates. Record the
actual application, including an NVIDIA/Nebius runtime call, and use English
narration or English captions. Keep secrets outside the capture area. The
continuous 00:20–01:50 segment reserves **90 seconds** for modules operating.
Do not fill that segment with slides alone.

| Time | Picture and operation | Suggested narration |
| --- | --- | --- |
| 00:00–00:20 | Title, target user, a still from the real simulator | “ReachGuard helps robotics developers study what a robot should do after a target changes. This supported humanoid runs in MuJoCo.” |
| 00:20–00:40 | Show a nominal reach, live measured hand position and error | “Classical inverse kinematics controls the arm. We measure the hand position; this is simulation telemetry.” |
| 00:40–01:05 | Move the target in the evaluation scenario; expose the monitor event and hold decision | “The inspection point moved. The execution monitor detects the mismatch and pauses the recovery path.” |
| 01:05–01:30 | Show sanitized live Nebius request metadata, returned model identity and structured decision alongside the simulator | “Nemotron receives the observation and progress summary through Nebius Token Factory. It proposes the next permitted action.” |
| 01:30–01:50 | Show validated recovery executing and its measured endpoint; keep trace visible | “Application code validates the proposal before the robot continues. Here is the resulting reach error and recovery count.” |
| 01:50–02:15 | Run one refusal case: stale observation, invalid proposal or exhausted budget; show held state and reason | “An invalid decision cannot widen the workspace or bypass the recovery limit. This case holds instead of issuing another reach.” |
| 02:15–02:35 | Show actual scenario summary and exact commit, distinguishing fixture from live results | “These are the measured results for this revision. Offline fixture tests exercise the local flow; this recording also includes the live provider path.” |
| 02:35–02:45 | Repository URL and scope card | “This is bounded recovery for supported reaching. It is not a walking, grasping or hardware-safety demonstration.” |

## Recording procedure

1. Use a fresh checkout of the final revision and follow the root README's
   installation and ReachGuard commands. Record exact versions and commands.
   The old humanoid launcher contains machine-specific interpreter defaults;
   do not copy those paths into reviewer instructions.
2. Run the fixture scenario suite first. Preserve its logs separately under a
   clearly labeled fixture run. Check the real rendered frames, not only exit
   status. Verify that the selected Concept 01 asset is available locally.
3. Set credentials in the local environment or an ignored local secret file,
   then use the documented live-provider command. Keep shell history,
   environment output, account pages and authorization headers out of view.
4. Preserve the complete live run trace and render output. Check that it includes
   a completed cloud response and the subsequent validated robotics outcome.
   If the request fails, record the failure and resolve it; do not relabel a
   fixture recording as live.
5. Capture the simulator and event/metrics panel together. Keep at least 90
   seconds of visible application operation in the edit, including the
   displacement, live decision and recovery. Label accelerated playback if used
   and retain a real-time view of the decisive interaction.
6. Add captions with the actual values from the trace. Use original or
   authorized visuals/audio. Export at 2:45, watch the full export and check the
   video metadata. Do not exceed the limit in the official checklist.
7. Upload the final video to YouTube with public visibility. Check playback
   signed out and record the URL and duration in the submission draft. Uploading
   and submitting remain user-facing publication steps.

## Evidence that should be legible

- Selected model and supported-pelvis scope.
- Current observation, measured hand position, error and observation validity.
- Provider mode; returned NVIDIA model identity on the live run.
- Monitor event, model proposal, validation outcome and recovery count.
- Final success or hold criterion, exact code revision and run identifier.

Use only signals implemented by the final runner. If an item is not exposed,
add the missing instrumentation before recording or narrow the narration. A
successful result from one episode is not a general reliability claim.
