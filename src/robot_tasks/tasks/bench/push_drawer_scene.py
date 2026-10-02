"""PhysX adapter for the declared latch proxy; no scoring or robot policy."""

from .push_drawer import PushLatch, PushLatchSettings


class PushDrawerMechanism:
    def __init__(self, scene):
        from pxr import UsdPhysics

        self.scene = scene
        self.origin = scene.design.articles[0].initial[0]
        self.outward_sign = scene.spec.parameters["outward_axis_world"][0]
        prim = scene.stage.GetPrimAtPath(scene.root + "/Joints/upper_drawer_slide")
        limit = UsdPhysics.LimitAPI(prim, "transX")
        self.low, self.high = limit.GetLowAttr(), limit.GetHighAttr()
        drive = scene.drives["upper_drawer_slide", "transX"]
        self.stiffness = drive.GetStiffnessAttr()
        self.damping = drive.GetDampingAttr()
        self.target = drive.GetTargetPositionAttr()
        self.force = drive.CreateMaxForceAttr()
        self.reset()

    def reset(self):
        self.latch = PushLatch(PushLatchSettings(**self.scene.spec.parameters["latch"]))
        self.applied = None
        self.target.Set(self.outward_sign * self.latch.settings.pop_target_m)
        self.force.Set(self.latch.settings.max_spring_force_n)
        self._apply(self.latch.update(0.0, self.scene.config.physics_dt_s))

    def _apply(self, values):
        # PhysX's joint frame is world-aligned; latch travel is positive outward.
        low, high = sorted(self.outward_sign * q for q in
                           (self.latch.settings.inward_stop_m, values["upper_limit_m"]))
        state = (low, high, values["stiffness"], values["damping"])
        if state != self.applied:
            for attr, value in zip((self.low, self.high, self.stiffness, self.damping), state):
                attr.Set(value)
            self.applied = state

    def step(self, dt):
        position, _ = self.scene.bodies["upper_drawer"].get_world_pose()
        self._apply(self.latch.update(self.outward_sign * (float(position[0]) - self.origin), dt))

    def measurement(self):
        # Privileged mechanism state stays in evaluation/recording, never in
        # policy observations. GPT must inspect the actual RGB-D gap and motion.
        return {"phase": self.latch.phase, "compressed_s": self.latch.compressed_s}
