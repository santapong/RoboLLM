# Elexhub lab

The source-checkout entry point is `run.py`. Follow the [setup guide](../../docs/physical-arm/elexhub/README.md) and [commissioning runbook](../../docs/physical-arm/elexhub/COMMISSIONING.md).

```sh
.venv/bin/python examples/elexhub/run.py --help
MUJOCO_GL=egl .venv/bin/python examples/elexhub/run.py serve
```

Default mode is CPU simulation. The physical profile is intentionally incomplete and locked. Nothing here accesses Kaggle training.
