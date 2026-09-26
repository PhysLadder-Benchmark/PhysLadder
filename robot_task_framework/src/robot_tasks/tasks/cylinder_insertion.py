from ..config import TaskConfig
from ..interfaces import Backend
from ..types import Assessment, EvaluationState, Feedback


class CylinderInsertion:
    """L1-01: pick up a cylinder, align, insert, and release stably.

    The prior gravity-drop experiment is supporting evidence only. It does not
    implement robot grasping, alignment, insertion, recovery, or task evaluation.
    """

    def build_scene(self, backend: Backend, config: TaskConfig) -> None:
        # TODO: table, free cylinder, holed fixture, collision, friction and cameras.
        # The cylinder must start on a surface the robot can actually reach.
        raise NotImplementedError("L1-01 scene and robot workspace")

    def reset(self, backend: Backend, seed: int) -> None:
        # TODO: seeded initial poses; reset velocities and all previous contacts.
        raise NotImplementedError("L1-01 repeatable initial state")

    def evaluate(self, state: EvaluationState, feedback: Feedback) -> Assessment:
        # TODO: insertion depth, axis alignment, release and stability duration.
        # Distinguish missed grasp, blocked insertion, drop and time limit.
        raise NotImplementedError("L1-01 independent success/failure criteria")
