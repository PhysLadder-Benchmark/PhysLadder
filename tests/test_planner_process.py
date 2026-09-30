"""Owned planner IPC tests using an offline process double, without CUDA or GPT."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
from fixtures import robot_config
from robot_tasks.planning.client import PlannerClient, linux_path
from robot_tasks.types import Action

HELPER = """import sys,json
print("diagnostic before ready",flush=True)
print(chr(0)+'R5_PLANNER '+json.dumps({'ready':True,'solver':'offline'}),flush=True)
for line in sys.stdin:
    data=json.loads(line)
    print('R5_PLANNER '+json.dumps({'id':data['id'],'paths':{},'diagnostics':{}}),flush=True)
"""


class PlannerProcessTests(unittest.TestCase):
    def test_child_protocol_and_eof_cleanup(self):
        config = robot_config()
        real_popen = subprocess.Popen
        with tempfile.TemporaryDirectory() as directory:

            def start(*args, **kwargs):
                return real_popen([sys.executable, "-u", "-c", HELPER], **kwargs)

            with patch(
                "robot_tasks.planning.client.subprocess.Popen", side_effect=start
            ):
                client = PlannerClient(config.robot, Path(directory))
            process = client.process
            result = client.plan(Action((), 0.1), {}, {})
            self.assertEqual(result["id"], 1)
            self.assertEqual(client.metadata["solver"], "offline")
            client.close()
            self.assertEqual(process.poll(), 0)
            client.close()

    def test_windows_path_conversion_does_not_rewrite_linux_paths(self):
        self.assertEqual(
            linux_path("D:\\research project\\r5.toml"),
            "/mnt/d/research project/r5.toml",
        )
        self.assertEqual(linux_path("/opt/research/r5.toml"), "/opt/research/r5.toml")
