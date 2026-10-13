"""Shared task lifecycle; all manipulation decisions remain with the model."""

from dataclasses import replace

from .arm_selection import resolve_bench_spec


class BenchTask:
    @staticmethod
    def prepare_config(config):
        spec = resolve_bench_spec(config.raw["task"], config.robot)
        notes = ""
        if "arm_selection" in spec.parameters:
            notes = (f"\nEpisode arm assignment: use the {spec.arms[0].upper()} arm for manipulation "
                     f"and the {spec.observers[0].upper()} arm for observation only. "
                     "The manipulator was selected by initial proximity to the task object. "
                     "Keep these roles for this episode; the observation arm may move its camera "
                     "but must not touch or support props. Proximity is not proof of reachability; "
                     "use current observations and planner feedback to choose feasible actions.")
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
            "L3-11",
            "L3-14",
            "L3-15",
        }
        return replace(
            config,
            instruction=(config.instruction if spec.parameters.get("real_prompt_protocol") else
                         config.instruction + "\nDeclared prop facts and guidance:\n" + spec.facts + notes),
            grasp_check_mode="contact_inspection" if constrained else "lift",
            initial_state_description=(
                "Both hands start empty. Props are physically supported in the declared initial layout. "
                "No task object is pre-attached to a gripper."
                + ("\nDocumented task setup (complements the verbatim trial instruction):\n"
                   + config.raw["task"]["scene_context"]
                   if spec.parameters.get("real_prompt_protocol") and config.raw["task"].get("scene_context") else "")),
        )

    def build_scene(self, backend, config):
        from .scene import BenchScene

        self.spec = resolve_bench_spec(config.raw["task"], config.robot)
        if config.arms != self.spec.arms or config.observation_arms != self.spec.observers:
            raise ValueError("Scene arm roles differ from the prepared task configuration")
        if self.spec.mechanism == "native_fem_orange_lift":
            from .orange_scene import OrangeScene

            self.scene = OrangeScene(backend, config, self.spec)
        elif self.spec.mechanism == "native_contact_wiping":
            from .wiping_scene import WipingScene

            self.scene = WipingScene(backend, config, self.spec)
        elif "native_instances" in self.spec.parameters:
            from .native_l2 import NativeL2Scene

            self.scene = NativeL2Scene(backend, config, self.spec)
        else:
            self.scene = BenchScene(backend, config, self.spec)
        if config.task_id.startswith("L2-"):
            from .l2_protocol import PROTOCOL, protocol_metadata

            backend.scene_report["household"]["test_protocol"] = {
                **protocol_metadata(config), "scoring_version": self.spec.parameters.get(
                    "scoring_protocol", PROTOCOL["scoring_version"])}

    def reset(self, backend, seed):
        from .evaluation import BenchEvaluator

        self.scene.reset(seed)
        from .observer_contact import ObserverContactMonitor

        self.observer_monitor = ObserverContactMonitor()
        self._last_evaluation_time = self._last_assessment = None
        if self.spec.mechanism == "native_fem_orange_lift":
            from .orange_evaluation import OrangeEvaluator

            self.evaluator = OrangeEvaluator(self.spec)
        else:
            self.evaluator = BenchEvaluator(self.spec, self.scene.initial)

    def initialize_episode(self, backend, robot, tick):
        if self.spec.design.task_id == "L1-09":
            from .startup_settling import settle_rose

            backend.scene_report["household"]["initial_prop_settling"] = settle_rose(self.scene, tick)

    def evaluate(self, state, feedback):
        now = state.values["simulation_time_s"]
        # Cache the whole assessment, including observer-role checks. Polling a
        # paused frame must not re-run either stateful evaluator.
        if now == self._last_evaluation_time:
            return self._last_assessment
        if self.spec.parameters.get("real_prompt_protocol"):
            assessment = self.evaluator.update(now, self.scene.measure(), robot=state.values.get("robot", {}))
        else:
            assessment = self.evaluator.update(now, self.scene.measure())
        evidence = getattr(self.scene, "observer_evidence", lambda: {})()
        self._last_assessment = self.observer_monitor.update(now, evidence, assessment)
        self._last_evaluation_time = now
        return self._last_assessment
