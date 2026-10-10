"""Task-specific geometry and scorer; all policy/execution/recording is shared."""

from dataclasses import replace

from .evaluation import ScrewEvaluator


class ScrewTask:
    @staticmethod
    def prepare_config(config):
        if config.arms != ("left",) or config.observation_arms != ("right",):
            raise ValueError("L3-05 uses left manipulation and right observation")
        # A free-object vertical test lift is useful initially, but invalid during
        # threaded regrasp. Visual confirmation and the independent process scorer
        # handle this mixed free/constrained task; do not pull an engaged screw up.
        return replace(config, grasp_check_mode="contact_inspection")

    def build_scene(self, backend, config):
        from .scene import ScrewScene

        self.scene = ScrewScene(backend, config)

    def reset(self, backend, seed):
        self.scene.reset(seed)
        self.evaluator = ScrewEvaluator(self.scene.spec)
        self._last_time = self._assessment = None

    def evaluate(self, state, feedback):
        now = state.values["simulation_time_s"]
        if now != self._last_time:
            self._assessment = self.evaluator.update(now, self.scene.measure())
            self._last_time = now
        return self._assessment
