"""The document's English task instruction must survive runtime preparation."""
import json
from pathlib import Path
import tempfile
import unittest

from robot_tasks.batch import freeze_config
from robot_tasks.config import load_config
from robot_tasks.registry import resolve_task

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'configs/prompts/feishu_english_20261014.json'


class EnglishTaskPromptTests(unittest.TestCase):
    def test_all_fifteen_prompts_survive_direct_and_batch_preparation(self):
        tasks = json.loads(SOURCE.read_text())['tasks']
        self.assertEqual(len(tasks), 15)
        with tempfile.TemporaryDirectory() as directory:
            for task_id, row in tasks.items():
                with self.subTest(task=task_id):
                    binding, config = resolve_task(task_id)
                    self.assertEqual(config.instruction, row['instruction'])
                    self.assertEqual(binding.factory.prepare_config(config).instruction, row['instruction'])
                    self.assertEqual((config.source.parent / config.raw['task']['prompt_source']).resolve(), SOURCE)
                    path = Path(directory) / (task_id + '.toml')
                    freeze_config(task_id, path, ROOT / 'configs/models/gpt.toml',
                                  ROOT / 'configs/workflows/gpt_default.toml',
                                  dict(reasoning_effort='xhigh', seed=0, wall_time_s=3600, max_calls=100))
                    prepared = binding.factory.prepare_config(load_config(path))
                    self.assertEqual(prepared.instruction, row['instruction'])
                    self.assertNotIn('Documented task setup', prepared.initial_state_description)

    def test_unlisted_task_retains_existing_prompt_behavior(self):
        binding, config = resolve_task('L1-05')
        self.assertFalse(config.raw['task'].get('verbatim_instruction', False))
        self.assertTrue(binding.factory.prepare_config(config).instruction.startswith(config.instruction))


if __name__ == '__main__':
    unittest.main()
