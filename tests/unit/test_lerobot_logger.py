"""Hardware-free contract tests for the minimal LeRobot recorder."""
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np

from lerobot_logger import LeRobotRecorder, lerobot_features


@dataclass
class FakeState:
    q: list[float]
    gripper: float
    t_host: float


class FakeArm:
    config = SimpleNamespace(
        joints=[SimpleNamespace(name=f"joint{i}") for i in range(1, 7)]
    )

    def __init__(self):
        self.step = 0

    def get_state(self):
        self.step += 1
        return FakeState([self.step / 10] * 6, 0.25, 10.005 + self.step)


class FakeCamera:
    def grab(self):
        # BGR red; the recorder must hand LeRobot RGB red.
        return np.full((2, 3, 3), [0, 0, 255], dtype=np.uint8), 10.0


class FakeDataset:
    def __init__(self, create_kwargs):
        self.create_kwargs = create_kwargs
        self.frames = []
        self.saved = False
        self.finalized = False
        self.cleared = False

    def add_frame(self, frame):
        self.frames.append(frame)

    def save_episode(self):
        self.saved = True

    def clear_episode_buffer(self):
        self.cleared = True

    def finalize(self):
        self.finalized = True


class FakeLeRobotDataset:
    created = None

    @classmethod
    def create(cls, **kwargs):
        cls.created = FakeDataset(kwargs)
        return cls.created


def test_schema_is_minimal_and_named():
    schema = lerobot_features((480, 640, 3), [f"joint{i}" for i in range(1, 7)])
    assert set(schema) == {
        "observation.images.front",
        "observation.state",
        "action",
        "observation.camera_lag_ms",
    }
    assert schema["observation.state"]["shape"] == (7,)
    assert schema["observation.state"]["names"][-1] == "gripper"


def test_recorder_uses_official_create_add_save_finalize_flow(tmp_path, monkeypatch):
    monkeypatch.setattr("lerobot_logger.time.sleep", lambda _: None)
    recorder = LeRobotRecorder(
        arm=FakeArm(),
        camera=FakeCamera(),
        root=tmp_path / "dataset",
        repo_id="local/test-arm",
        task="pick block",
        fps=15,
        dataset_class=FakeLeRobotDataset,
    )

    output = recorder.record_episode(2)
    dataset = FakeLeRobotDataset.created

    assert output == tmp_path / "dataset"
    assert dataset.create_kwargs["use_videos"] is True
    assert dataset.saved and dataset.finalized and not dataset.cleared
    assert len(dataset.frames) == 2
    assert dataset.frames[0]["task"] == "pick block"
    assert dataset.frames[0]["observation.state"].dtype == np.float32
    assert dataset.frames[0]["action"].tolist() == dataset.frames[0]["observation.state"].tolist()
    assert dataset.frames[0]["observation.images.front"][0, 0].tolist() == [255, 0, 0]
