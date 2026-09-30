"""Simplified bimanual uncapping, using the shared GPT/robot/recording pipeline."""

from dataclasses import replace

from .evaluation import UncapEvaluator


class UncapTask:
    @staticmethod
    def prepare_config(config):
        if config.arms != ("left", "right") or config.observation_arms:
            raise ValueError("L2-01 needs left body holding and right cap manipulation")
        return replace(
            config,
            grasp_check_mode="contact_inspection",
            initial_state_description=(
                "The capped marker lies horizontally on the table. Both hands start empty. "
                "First grasp and lift the complete marker with the left hand, then grasp the cap with the right. "
                "Use current images and measured state, not assumed object coordinates."
            ),
        )

    def build_scene(self, backend, config):
        from .scene import UncapScene

        self.config = config
        self.scene = UncapScene(backend, config)

    def reset(self, backend, seed):
        self.scene.reset(seed)
        self.evaluator = UncapEvaluator(require_table_pickup=True)

    def evaluate(self, state, feedback):
        return self.evaluator.update(
            state.values["simulation_time_s"],
            self.scene.measure(),
            state.values["robot"],
        )
