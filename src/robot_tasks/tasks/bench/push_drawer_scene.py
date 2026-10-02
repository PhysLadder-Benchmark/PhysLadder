"""PhysX adapter for the declared latch proxy; no scoring or robot policy."""

from .push_drawer import PushLatch, PushLatchSettings


class PushDrawerMechanism:
    def __init__(self, scene):
        from pxr import UsdPhysics

        self.scene = scene
        outward = scene.spec.parameters["outward_axis_world"]
        components = [i for i, value in enumerate(outward) if value != 0.0]
        if len(components) != 1 or abs(outward[components[0]]) != 1.0:
            raise ValueError("Push drawer needs a signed world-axis slide")
        self.axis_index = components[0]
        axis = "trans" + "XYZ"[self.axis_index]
        self.origin = scene.design.articles[0].initial[self.axis_index]
        self.outward_sign = outward[self.axis_index]
        slide = next(j for j in scene.spec.joints if j.name == "upper_drawer_slide")
        if set(slide.axes) != {axis}:
            raise ValueError("Push drawer joint and outward direction disagree")
        prim = scene.stage.GetPrimAtPath(scene.root + "/Joints/upper_drawer_slide")
        limit = UsdPhysics.LimitAPI(prim, axis)
        self.low, self.high = limit.GetLowAttr(), limit.GetHighAttr()
        drive = scene.drives["upper_drawer_slide", axis]
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
        travel = self.outward_sign * (float(position[self.axis_index]) - self.origin)
        self._apply(self.latch.update(travel, dt))

    def measurement(self):
        # Privileged mechanism state stays in evaluation/recording, never in
        # policy observations. GPT must inspect the actual RGB-D gap and motion.
        return {"phase": self.latch.phase, "compressed_s": self.latch.compressed_s}
