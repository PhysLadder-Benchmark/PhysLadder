"""Shared task lifecycle; all manipulation decisions remain with the model."""

from dataclasses import replace

from .spec import build_spec


class BenchTask:
    @staticmethod
    def prepare_config(config):
        spec = build_spec(config.task_id)
        notes = ""
        if config.task_id.startswith("L2-"):
            from .l2_protocol import OPERATING_NOTES

            notes = OPERATING_NOTES
        if (
            config.task_id.startswith(("L1-", "L2-"))
            and config.raw.get("simulation", {}).get("environment") != "tuojing_lab_v4"
        ):
            raise ValueError(
                f"{config.task_id} uses the Tuojing laboratory layout; use its current task config"
            )
        if config.arms != spec.arms or config.observation_arms != spec.observers:
            raise ValueError(
                f"{config.task_id} requires manipulation={spec.arms}, observation={spec.observers}"
            )
        constrained = config.task_id in {
            "L2-01",
            "L2-03",
            "L2-04",
            "L2-06",
            "L2-07",
            "L3-03",
            "L3-14",
            "L3-15",
            "L3-16",
        }
        return replace(
            config,
            instruction=config.instruction + "\nDeclared prop facts and guidance:\n" + spec.facts + notes,
            grasp_check_mode="contact_inspection" if constrained else "lift",
            initial_state_description="Both hands start empty. Props are physically supported in the declared initial layout. No task object is pre-attached to a gripper.",
        )

    def build_scene(self, backend, config):
        from .scene import BenchScene

        self.spec = build_spec(config.task_id)
        if self.spec.mechanism == "native_fem_orange_transfer":
            from .orange_scene import OrangeScene

            self.scene = OrangeScene(backend, config, self.spec)
        elif "native_instances" in self.spec.parameters:
            from .native_l2 import NativeL2Scene

            self.scene = NativeL2Scene(backend, config, self.spec)
        else:
            self.scene = BenchScene(backend, config, self.spec)
        if config.task_id.startswith("L2-"):
            from .l2_protocol import PROTOCOL

            backend.scene_report["household"]["test_protocol"] = {
                **PROTOCOL, "scoring_version": self.spec.parameters.get(
                    "scoring_protocol", PROTOCOL["scoring_version"])}

    def reset(self, backend, seed):
        from .evaluation import BenchEvaluator

        self.scene.reset(seed)
        from .observer_contact import ObserverContactMonitor

        self.observer_monitor = ObserverContactMonitor()
        if self.spec.mechanism == "native_fem_orange_transfer":
            from .orange_evaluation import OrangeEvaluator

            self.evaluator = OrangeEvaluator(self.spec)
        else:
            self.evaluator = BenchEvaluator(self.spec, self.scene.initial)

    def evaluate(self, state, feedback):
        now = state.values["simulation_time_s"]
        assessment = self.evaluator.update(now, self.scene.measure())
        evidence = getattr(self.scene, "observer_evidence", lambda: {})()
        return self.observer_monitor.update(now, evidence, assessment)
