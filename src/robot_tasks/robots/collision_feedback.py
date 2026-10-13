"""Measured rigid collision feedback, independent of task success scoring.

Fingers may contact props. Non-finger links/attachments must not be driven into
props or furniture, and neither hand may be driven into the other robot.
This stops an action, not an episode; the policy still chooses any recovery.
"""

import math


def unexpected_pair(collider0, collider1, mounts):
    def robot_part(path):
        for arm, root in mounts.items():
            if path.startswith(root + "/link"):
                link = path[len(root) + 1:].split("/")[0]
                if link in {f"link{i}" for i in range(1, 9)}:
                    return arm, link
        return None

    parts = [robot_part(p) for p in (collider0, collider1)]
    if parts[0] and parts[1] and parts[0][0] == parts[1][0]:
        return None  # Articulation's self-collision settings govern own links.
    for i, part in enumerate(parts):
        if part and (part[1] not in {"link7", "link8"} or parts[1-i]):
            return {"arm": part[0], "link": part[1],
                    "component": "wrist_camera_or_mount" if "/WristSensor/" in (collider0, collider1)[i] else part[1],
                    "robot_collider": (collider0, collider1)[i],
                    "other_collider": (collider0, collider1)[1-i]}
    return None


class CollisionMonitor:
    def __init__(self, stage, profile):
        from pxr import PhysicsSchemaTools, PhysxSchema, Sdf
        from omni.physx import get_physx_simulation_interface

        self.mounts = {m.name: m.prim_path for m in profile.mounts}
        self.robot_actors = {PhysicsSchemaTools.sdfPathToInt(Sdf.Path(f"{root}/link{i}"))
                             for root in self.mounts.values() for i in range(1, 9)}
        self.reset()
        for root in self.mounts.values():
            for i in range(1, 9):
                PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(f"{root}/link{i}")).CreateThresholdAttr(0.)
        self.subscription = get_physx_simulation_interface().subscribe_contact_report_events(self._contacts)

    def reset(self):
        self.pending = []
        self.blocked = None
        self.peak = None
        self.steps = 0
        self.events = 0
        self.recovery_contacts = {}
        self.recovery_elapsed_s = 0.

    @staticmethod
    def _pair(row):
        return row.get("robot_collider"), row.get("other_collider")

    def _contacts(self, headers, data):
        from pxr import PhysicsSchemaTools

        for header in headers:
            # Hundreds of pearl/pearl contacts are irrelevant to arm feedback.
            if (not header.num_contact_data or
                (header.actor0 not in self.robot_actors and header.actor1 not in self.robot_actors)):
                continue
            row = unexpected_pair(str(PhysicsSchemaTools.intToSdfPath(header.collider0)),
                                  str(PhysicsSchemaTools.intToSdfPath(header.collider1)), self.mounts)
            if row is None:
                continue
            points = data[header.contact_data_offset:header.contact_data_offset + header.num_contact_data]
            impulse = sum(math.sqrt(sum(float(v)**2 for v in p.impulse)) for p in points)
            separation = min(float(p.separation) for p in points)
            self.pending.append({**row, "impulse_ns": impulse, "separation_m": separation})

    def after_step(self, dt):
        self.steps += 1
        self.recovery_elapsed_s += dt
        for row in self.pending:
            row["force_n"] = row["impulse_ns"] / dt
            if not all(math.isfinite(row[k]) for k in ("force_n", "separation_m")):
                from ..types import InvalidSimulationState
                raise InvalidSimulationState("Nonfinite rigid collision report")
            if self.peak is None or row["force_n"] > self.peak["force_n"]:
                self.peak = row
            # Noise/contact margin threshold, NOT a prop damage or task-failure
            # criterion. Real camera force sensing is not assumed or claimed.
            if row["force_n"] >= .1 or row["separation_m"] < -.002:
                old = self.recovery_contacts.get(self._pair(row))
                if (old is not None and self.recovery_elapsed_s <= .25
                    and row["separation_m"] >= max(-.002, old["separation_m"] - .0005)
                    and row["force_n"] <= old["force_n"] + .1):
                    # A new planned motion may leave an already reported contact.
                    # Never waive new contacts, deepening or growing forces, and
                    # never keep driving against a persistent obstruction.
                    continue
                self.events += 1
                if self.blocked is None or row["force_n"] > self.blocked["force_n"]:
                    self.blocked = row
        self.pending.clear()

    def begin_action(self, *, allow_retreat=False):
        row = self.blocked
        self.recovery_contacts = ({self._pair(row): dict(row)}
                                  if allow_retreat and row else {})
        self.recovery_elapsed_s = 0.
        self.blocked = None

    def metadata(self):
        return {"scope": "non-finger rigid links/cameras/mounts against external shapes; inter-arm contact including fingers",
                "normal_finger_prop_contacts_allowed": True,
                "policy_feedback_source": "binary simulated PhysX contact flag; detailed impulses kept in execution log, no calibrated hardware force sensor",
                "response": "cancel current motion, hold measured pose and grip, return collision_blocked; no automatic retreat or task failure",
                "force_noise_floor_n": .1, "penetration_guard_m": .002,
                "retreat_contact_grace_s": .25, "retreat_max_extra_penetration_m": .0005,
                "retreat_rule": "only the already reported pair during a new planned motion, without growing force or deeper penetration; new contacts still stop",
                "limitations": "PhysX rigid contacts only; FEM orange contact is not reported by this sensor. Own-arm contacts follow the existing self-collision setting."}

    def close(self):
        self.subscription = None
