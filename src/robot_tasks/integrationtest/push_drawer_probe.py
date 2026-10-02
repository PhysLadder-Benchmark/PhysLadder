"""Force-only fixture diagnostic, never a robot controller or benchmark result."""

import numpy as np


def run(scene):
    from PIL import Image

    previous = scene.backend.before_physics_step
    force = 0.0
    peak_speed = 0.0
    samples = []
    outward = np.asarray(scene.spec.parameters["outward_axis_world"], dtype=np.float32)
    output = scene.backend.run_directory / "push_drawer_probe"
    output.mkdir(parents=True, exist_ok=True)

    def before_step(dt):
        nonlocal peak_speed
        previous(dt)
        peak_speed = max(peak_speed, float(np.linalg.norm(scene.bodies["upper_drawer"].get_linear_velocity())))
        scene.pairs["upper_drawer"].apply_forces_and_torques_at_pos(
            forces=(force * outward)[None, :], is_global=True)

    scene.backend.before_physics_step = before_step

    def stage(name, seconds, load):
        nonlocal force, peak_speed
        force = load
        peak_speed = 0.0
        for _ in range(round(seconds / scene.config.control_dt_s)):
            scene.backend.step(scene.config.control_dt_s)
        rows = scene.measure()
        upper = rows["upper_drawer"]
        sample = {"stage": name, "test_force_n": load,
                  "travel_m": float((np.asarray(upper["pose"][:3]) - scene.initial["upper_drawer"][:3]) @ outward),
                  "speed_m_s": float(np.linalg.norm(upper["linear_velocity"])),
                  "peak_speed_m_s": peak_speed,
                  "phase": upper["push_latch"]["phase"],
                  "lower_drift_m": float(np.linalg.norm(np.array(rows["lower_drawer"]["pose"][:3])
                                                       - scene.initial["lower_drawer"][:3]))}
        samples.append(sample)
        for frame in scene.backend.capture_rgb():
            Image.frombytes("RGB", (frame.width, frame.height), frame.data).save(output / f"{name}_{frame.camera}.png")
        return sample

    try:
        pulled = stage("pull_while_locked", .5, 2.0)
        if pulled["travel_m"] > .003 or pulled["phase"] != "locked":
            raise RuntimeError("Pull-only bypassed latch: " + str(samples))
        partial = stage("partial_press", 1.0, -3.1)
        released = stage("release_partial_press", 1.0, 0.0)
        if partial["phase"] != "locked" or released["travel_m"] > .003:
            raise RuntimeError("Insufficient compression unlocked latch: " + str(samples))
        pressed = stage("full_press_held", .5, -6.0)
        if pressed["phase"] != "armed" or pressed["travel_m"] > -.008:
            raise RuntimeError("Full inward press failed to arm latch: " + str(samples))
        popped = stage("released_popout", 2.0, 0.0)
        if popped["phase"] != "open" or not .018 <= popped["travel_m"] <= .065:
            raise RuntimeError("Passive spring did not produce bounded pop-out: " + str(samples))
        if popped["peak_speed_m_s"] > scene.spec.parameters["max_speed_m_s"]:
            raise RuntimeError("Passive pop-out violates task speed limit: " + str(samples))
        stage("pull_to_rail_end", 2.0, .15)
        final = stage("released_at_end", 1.5, 0.0)
        if not .100 <= final["travel_m"] <= .118 or final["speed_m_s"] > .025:
            raise RuntimeError("Full opening did not remain rail-supported: " + str(samples))
        if any(s["lower_drift_m"] > .003 for s in samples):
            raise RuntimeError("Wrong-layer motion: " + str(samples))
        scene.reset(scene.config.seed)
        reset = stage("reset_locked", .5, 0.0)
        if reset["phase"] != "locked" or abs(reset["travel_m"]) > .003:
            raise RuntimeError("Reset did not restore closed latch: " + str(samples))
        return {"task": "L3-16", "passed": True, "samples": samples,
                "scope": "fixture-only external-force diagnostic; no robot manipulation or model-success evidence"}
    finally:
        scene.backend.before_physics_step = previous
