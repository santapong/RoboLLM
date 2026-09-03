"""Goal families, cells, and seeded episode specifications (SDD §7).

Five named regions of the reachable workspace on the +y side of the UR5e base,
each split into a 2×2 grid of cells (20 cells). A target is the cell centre plus
seeded uniform jitter inside the cell; the initial state is the UR5e home pose
plus seeded joint jitter. Train and evaluation seed blocks are disjoint, as in
B1 (`examples/mujoco/reaching.py`).

Reachability is not assumed: `freeze_families()` solves IK to every cell corner
and centre and writes the verified list to `configs/families.json`; the gate
refuses cells that are not in that file (SDD §7, design rule R8).
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

BED_DIR = Path(__file__).resolve().parent
FAMILIES_FILE = BED_DIR / "configs" / "families.json"

# Region boxes in the robot base frame (metres). Home EE is (-0.134, 0.492, 0.332)
# with the tool pointing down; all regions sit on the +y side within ~0.7 m reach.
FAMILY_REGIONS: dict[str, tuple[tuple[float, float], tuple[float, float], tuple[float, float]]] = {
    "front_high": ((-0.15, 0.10), (0.45, 0.62), (0.30, 0.45)),
    "front_low": ((-0.15, 0.10), (0.45, 0.62), (0.08, 0.20)),
    "left": ((-0.38, -0.20), (0.32, 0.55), (0.15, 0.35)),
    "right": ((0.12, 0.30), (0.32, 0.55), (0.15, 0.35)),
    "near": ((-0.15, 0.10), (0.28, 0.40), (0.15, 0.35)),
}
FAMILY_NAMES = tuple(FAMILY_REGIONS)
CELLS_PER_FAMILY = 4  # 2×2 grid along x and y
CELL_IDS = tuple(range(CELLS_PER_FAMILY))

INITIAL_JOINT_JITTER = 0.06  # rad, B1's value
EVALUATION_SEED_OFFSET = 1_000_000_000
TARGET_RELOCATION_SEED_OFFSET = 77_777  # B1's value for the relocation variation


@dataclass(frozen=True)
class Cell:
    family: str
    cell: int
    low: tuple[float, float, float]
    high: tuple[float, float, float]

    @property
    def centre(self) -> np.ndarray:
        return (np.asarray(self.low) + np.asarray(self.high)) / 2.0

    def corners(self) -> np.ndarray:
        lo, hi = np.asarray(self.low), np.asarray(self.high)
        return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def cells(families: tuple[str, ...] = FAMILY_NAMES) -> list[Cell]:
    out: list[Cell] = []
    for family in families:
        (x0, x1), (y0, y1), (z0, z1) = FAMILY_REGIONS[family]
        xm, ym = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        boxes = [((x0, y0), (xm, ym)), ((xm, y0), (x1, ym)), ((x0, ym), (xm, y1)), ((xm, ym), (x1, y1))]
        for cell_id, ((cx0, cy0), (cx1, cy1)) in enumerate(boxes):
            out.append(Cell(family, cell_id, (cx0, cy0, z0), (cx1, cy1, z1)))
    return out


@dataclass(frozen=True)
class EpisodeSpec:
    seed: int
    split: str
    family: str
    cell: int
    target: tuple[float, float, float]
    initial_q: tuple[float, ...]

    def to_dict(self) -> dict:
        return asdict(self)


def _cell(family: str, cell_id: int) -> Cell:
    for c in cells((family,)):
        if c.cell == cell_id:
            return c
    raise KeyError(f"{family}/{cell_id}")


def sample_target(cell: Cell, rng: np.random.Generator) -> np.ndarray:
    return rng.uniform(np.asarray(cell.low), np.asarray(cell.high))


def make_episode_spec(seed: int, family: str, cell_id: int, split: str = "train", home_q=None) -> EpisodeSpec:
    if split not in ("train", "evaluation"):
        raise ValueError(f"unknown split {split!r}")
    if home_q is None:
        from scene.build_scene import HOME_QPOS  # local import keeps this module MuJoCo-free for tests

        home_q = HOME_QPOS
    rng = np.random.default_rng(seed)
    initial_q = np.asarray(home_q, dtype=float) + rng.uniform(-INITIAL_JOINT_JITTER, INITIAL_JOINT_JITTER, size=6)
    target = sample_target(_cell(family, cell_id), rng)
    return EpisodeSpec(
        seed=int(seed),
        split=split,
        family=family,
        cell=int(cell_id),
        target=tuple(float(v) for v in target),
        initial_q=tuple(float(v) for v in initial_q),
    )


def episode_specs(count: int, seed: int, split: str = "train", families: tuple[str, ...] = FAMILY_NAMES, home_q=None) -> list[EpisodeSpec]:
    """Round-robin over cells (family-major) so families and cells stay balanced."""
    offset = 0 if split == "train" else EVALUATION_SEED_OFFSET
    grid = cells(families)
    specs = []
    for index in range(count):
        c = grid[index % len(grid)]
        specs.append(make_episode_spec(seed + offset + index, c.family, c.cell, split, home_q=home_q))
    return specs


def relocated_target(spec: EpisodeSpec) -> np.ndarray:
    """The target_relocation variation: a fresh seeded draw inside the same cell."""
    rng = np.random.default_rng(spec.seed + TARGET_RELOCATION_SEED_OFFSET)
    return sample_target(_cell(spec.family, spec.cell), rng)


def load_frozen() -> dict:
    if not FAMILIES_FILE.exists():
        raise FileNotFoundError(f"{FAMILIES_FILE} missing — run `python families.py --freeze`")
    return json.loads(FAMILIES_FILE.read_text())


def freeze_families(max_residual_m: float = 0.005) -> dict:
    """Verify every cell (centre + 8 corners) with IK and write configs/families.json."""
    from expert import MinkController  # noqa: WPS433 (runtime import: needs MuJoCo + mink)
    from scene import build_scene

    model = build_scene.load_model()
    controller = MinkController(model)
    verified, rejected = [], []
    for c in cells():
        worst = 0.0
        for point in np.vstack([c.centre[None, :], c.corners()]):
            residual = controller.settle_residual(point)
            worst = max(worst, residual)
        entry = {"family": c.family, "cell": c.cell, "low": c.low, "high": c.high, "worst_residual_m": round(worst, 5)}
        (verified if worst <= max_residual_m else rejected).append(entry)
    doc = {
        "schema": "robollm.vla-bed.families.v1",
        "max_residual_m": max_residual_m,
        "regions": {k: v for k, v in FAMILY_REGIONS.items()},
        "verified": verified,
        "rejected": rejected,
    }
    FAMILIES_FILE.parent.mkdir(parents=True, exist_ok=True)
    FAMILIES_FILE.write_text(json.dumps(doc, indent=2) + "\n")
    return doc


if __name__ == "__main__":
    import argparse
    import sys

    sys.path.insert(0, str(BED_DIR))
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--freeze", action="store_true", help="IK-verify all cells and write configs/families.json")
    args = parser.parse_args()
    if args.freeze:
        doc = freeze_families()
        print(f"verified {len(doc['verified'])} cells, rejected {len(doc['rejected'])} → {FAMILIES_FILE}")
        for r in doc["rejected"]:
            print("  REJECTED", r)
    else:
        for c in cells():
            print(c.family, c.cell, c.low, c.high)
