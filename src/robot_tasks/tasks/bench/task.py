"""Shared task lifecycle; all manipulation decisions remain with the model."""

from dataclasses import replace

from .spec import build_spec


class BenchTask:
    @staticmethod
    def prepare_config(config):
        spec = build_spec(config.task_id)
        if config.task_id.startswith("L1-") and config.raw.get("simulation", {}).get("environment") != "tuojing_lab_v4":
            raise ValueError(f"{config.task_id} uses the Tuojing laboratory layout; use its current task config")
        if config.arms != spec.arms or config.observation_arms != spec.observers:
            raise ValueError(
                f"{config.task_id} requires manipulation={spec.arms}, observation={spec.observers}"
            )
        constrained = config.task_id in {
            "L2-03",
            "L2-04",
            "L2-05",
            "L2-06",
            "L2-07",
            "L3-03",
            "L3-14",
            "L3-15",
            "L3-16",
        }
        return replace(
            config,
            instruction=config.instruction + "\nSimulation prop facts:\n" + spec.facts,
            grasp_check_mode="contact_inspection" if constrained else "lift",
            initial_state_description="Both hands start empty. Props are physically supported in the declared initial layout. No task object is pre-attached to a gripper.",
        )

    def build_scene(self, backend, config):
        from .scene import BenchScene

        self.spec = build_spec(config.task_id)
        self.scene = BenchScene(backend, config, self.spec)

    def reset(self, backend, seed):
        from .evaluation import BenchEvaluator

        self.scene.reset(seed)
        self.evaluator = BenchEvaluator(self.spec, self.scene.initial)

    def evaluate(self, state, feedback):
        return self.evaluator.update(
            state.values["simulation_time_s"], self.scene.measure()
        )
