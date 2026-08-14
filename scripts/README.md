# Operational scripts

Scripts are grouped by responsibility:

- `launch/` starts MCP, dashboard, and simulation processes.
- `ci/` runs container-bound scenario acceptance.
- `learning/b1/` owns B1 dataset, CPU acceptance, and guarded GPU workflows.
- `setup/` provisions a development workstation.

Scripts must resolve the repository root from their own location and must not
contain personal absolute paths.
