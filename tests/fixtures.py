"""Offline protocol fixtures derived from the maintained task, never live runs."""

from copy import deepcopy
from dataclasses import replace
from pathlib import Path

import numpy as np

from robot_tasks.backends.mock import MockBackend
from robot_tasks.config import load_config
from robot_tasks.policies.sparse_action import FIELDS
from robot_tasks.types import DepthFrame, RGBFrame


ROOT = Path(__file__).resolve().parents[1]


def robot_config(*, arms=("left", "right")):
    """Use real R5 metadata with both arms controllable for offline contracts."""
    config = load_config(ROOT / "configs/tasks/gpt_single_arm_pick_place.toml")
    raw = deepcopy(config.raw)
    raw["robot"].update(active_arms=list(arms), observation_arms=[])
    raw["recording"].update(save_depth=False, save_video=False)
    raw["execution"] = {}
    return replace(config, raw=raw, arms=arms, observation_arms=(), save_depth=False,
                   instruction="Offline robot protocol fixture; not a benchmark task instruction.")


def rgbd(camera="overview", time_s=0., *, width=32, height=24, capture_id=1):
    calibration = {"intrinsics_fx_fy_cx_cy": [32., 30., width/2, height/2],
                   "distortion": [0.] * 5, "resolution": [width, height],
                   "T_world_camera_pose_wxyz": [1., 2., 3., 1., 0., 0., 0.],
                   "capture_id": capture_id, "simulation_time_s": time_s,
                   "source": "simulation_camera_parameters", "hardware_calibrated": False}
    depth = DepthFrame(camera, time_s, width, height, np.full((height, width), .5, "<f4").tobytes(), capture_id)
    return RGBFrame(camera, time_s, width, height, bytes([50, 100, 200]) * width * height, calibration, depth)


def gripper_command(width=.06, arm="left"):
    return {"arm": arm, **dict.fromkeys(FIELDS), "gripper_width_m": width}


def decision_document(**updates):
    return {"commands": [], "request_stop": False, "depth_queries": [],
            "note": "Offline observation. Execute the fixture decision.", **updates}


class RGBDBackend(MockBackend):
    def capture_rgb(self):
        return tuple(rgbd(name, self.t) for name in self.config.cameras)
