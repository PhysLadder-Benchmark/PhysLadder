"""Declared assistance and measurement boundaries shared by the five native L2 tasks."""

PROTOCOL = {
    "prompt_version": "guided_rgbd_l2_v2",
    "scoring_version": "native_l2_validity_v2",
    "scope": "guided RGB-D closed-loop manipulation with declared prop/grasp hints",
    "policy_information": "RGB-D, camera calibration, robot state, execution feedback, public task facts",
    "privileged_information": "object/FEM/contact truth for scoring and planner clearance only",
    "control": "pose/joint and gripper-opening targets; fixed controller gains and timing, no force command",
    "model_wait": "physics paused during model requests",
}

OPERATING_NOTES = """
Shared L2 protocol (guided_rgbd_l2_v2):
The task supplies prop dimensions and grasp/direction hints. Use current RGB-D
and robot feedback to locate and manipulate the actual objects; these hints
are not measured current object poses. This is a guided execution test, not
an unassisted affordance-discovery test.
You command poses and gripper openings. There is no direct force, torque or
motion-speed command. Smaller pose increments and intermediate gripper openings
are available; their effect on contact must be checked, not assumed. The fixed
controller executes motions between observations; physics pauses while you think.
For a paired transport, command both TCPs in the same action with compatible
displacements. Shared execution timing does not guarantee a level load or grip.
An observation-only arm may reposition its camera but must not touch task props
or their fixtures, even with its wrist or camera housing.
There is no in-episode reset. A confirmed irreversible process violation ends
the episode. Numerical simulation failures are recorded separately from your
task outcome. Do not infer an unavailable force sensor from the scoring rules.
"""
