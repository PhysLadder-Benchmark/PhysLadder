from ..config import TaskConfig
from ..interfaces import Backend
from ..types import Assessment, EvaluationState, Feedback


class DualArmHandover:
    """L2-02: donor grasp -> overlapping support -> donor release -> stable receiver.

    StationeryBench provides a real-robot task reference, not a locally validated
    force-transfer simulation. No handover experiment was run in sim_feasibility.
    """

    def build_scene(self, backend: Backend, config: TaskConfig) -> None:
        # TODO: two arms with overlapping reach, transferable object, physical grips.
        # Do not attach the object to a hand via a fixed joint to mimic grasping.
        raise NotImplementedError("L2-02 bimanual contact scene")

    def reset(self, backend: Backend, seed: int) -> None:
        raise NotImplementedError("L2-02 reproducible object and arm configuration")

    def evaluate(self, state: EvaluationState, feedback: Feedback) -> Assessment:
        # TODO: donor/receiver support forces, slip/drop, independent receiver hold.
        # A mere object pose or handover animation does not prove force transfer.
        raise NotImplementedError("L2-02 support transfer and sustained grasp scoring")
