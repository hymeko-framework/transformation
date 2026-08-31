"""Time MuJoCo's dynamics step on the emitted MJCF, for comparison with ours.

Two measurements per model. Our dynamics models no contact at all, so the comparable computation is
MuJoCo with contacts disabled — articulated-body dynamics and nothing else. With them on is what
MuJoCo is actually for. Both are reported rather than the flattering one.
"""
import sys
import time

import mujoco

bench = sys.argv[1]
for root in sys.argv[2:]:
    for contact in (True, False):
        model = mujoco.MjModel.from_xml_path(f"{bench}/{root}.xml")
        data = mujoco.MjData(model)
        model.opt.timestep = 1.0 / 240.0
        if not contact:
            model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
        if model.nq:
            data.qpos[0] = 0.3
        for _ in range(20):
            mujoco.mj_step(model, data)

        # nstep, so the Python FFI is paid once and not per step. Measured per call, the overhead
        # dominated: the 8-DOF arm read SLOWER than the 23-DOF quadruped, which is impossible
        # physics and an obvious sign that the number was not measuring the physics.
        n = 20_000
        t0 = time.perf_counter()
        mujoco.mj_step(model, data, nstep=n)
        us = (time.perf_counter() - t0) * 1e6 / n
        tag = "with contact" if contact else "no contact  "
        print(
            f"  {root:<12} nv={model.nv:<3} ngeom={model.ngeom:<3} {tag} "
            f"{us:>8.2f} us/step   ({1e6/us:,.0f} steps/s)"
        )
