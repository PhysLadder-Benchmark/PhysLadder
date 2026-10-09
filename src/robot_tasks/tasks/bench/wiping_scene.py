"""Native eraser and anchored board with passive, contact-driven ink removal."""

import numpy as np

from .native_l2 import NativeL2Scene
from .wiping import INK_COLOR, WipeLedger, ink_centers


class WipingScene(NativeL2Scene):
    root = "/World/L3Wiping"

    def __init__(self, backend, config, spec):
        from isaacsim.core.prims import RigidPrim
        from pxr import Gf, UsdGeom

        super().__init__(backend, config, spec)
        self.ink = []
        for i, center in enumerate(ink_centers()):
            shape = UsdGeom.Cube.Define(self.stage, self.root + f"/board/Ink/cell_{i}")
            shape.CreateSizeAttr(1)
            shape.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
            shape.AddScaleOp().Set(Gf.Vec3f(0.007, 0.007, 0.0001))
            shape.CreateDisplayColorAttr([Gf.Vec3f(*INK_COLOR)])
            self.ink.append(shape)
        # Filter the writing surface; the ledger also checks pad face and distance.
        self.pad_contact = RigidPrim(
            self.paths["eraser"],
            name="wiping_pad_contacts",
            reset_xform_properties=False,
            track_contact_forces=True,
            contact_filter_prim_paths_expr=[
                self.root + "/board/Colliders/WritingSurface"
            ],
            max_contact_count=2048,
            disable_stablization=False,
        )
        self.ledger = WipeLedger(spec.parameters["board_pose"])

    def reset(self, seed):
        super().reset(seed)
        self.pad_contact.initialize()
        self.ledger = WipeLedger(self.spec.parameters["board_pose"])
        for shape in self.ink:
            shape.GetVisibilityAttr().Set("inherited")

    def measure(self):
        rows = super().measure()
        row = rows["eraser"]
        contacts = np.asarray(
            self.pad_contact.get_contact_force_matrix(dt=self.config.physics_dt_s)
        )
        if contacts.shape != (1, 1, 3) or not np.isfinite(contacts).all():
            raise ValueError("Invalid eraser/board contact measurement")
        for index in self.ledger.update(
            self.backend.time_s(),
            row["pose"],
            pad_contact_n=float(np.linalg.norm(contacts[0, 0])),
            held=min(row["fingers"][self.spec.arms[0]]) > 0.05,
        ):
            self.ink[index].GetVisibilityAttr().Set("invisible")
        row["wiping"] = self.ledger.metrics()
        return rows
