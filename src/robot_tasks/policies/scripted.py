from ..types import Action, PolicyInput


class ScriptedPolicy:
    """First implement a robot-executed reference trajectory for one task."""

    def reset(self, seed: int) -> None:
        pass

    def act(self, context: PolicyInput) -> Action:
        raise NotImplementedError("Task-specific scripted trajectory / state machine")
