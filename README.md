# MimicKit G1 — MuJoCo WASM / Newton-style General Servo test

This revision is based on the **confirmed-loading MuJoCo WASM release** (`phase1_mujoco_wasm_fix`). The **existing 241-observation / 29-action `amp_actor.onnx` is unchanged**.

## What changed (only joint control)

- `public/g1_collision.xml`: Replaced all 29 `<motor>` entries with **29 `<general>` position servos**, one per G1 joint. Each uses `dyntype=none`, `gaintype=fixed`, `biastype=affine`, `gear=1`, `gainprm=Kp`, `biasprm="0 -Kp -Kd"`, `forcelimited=true`, `forcerange="-limit limit"`.
- `src/main.js`: Removed explicit JS torque PD and replaced it with `data.ctrl[actuatorId]=targetAngle`; MuJoCo calculates/clamps actuator torques internally.
- The Kp/Kd/Force multipliers in GUI now alter `model.actuator_gainprm`, `model.actuator_biasprm`, and `model.actuator_forcerange` for the active built-in servos. When Active Drives is false, Kp and Kd become zero (no drive force).
- On reset, the initial **29 position targets** are applied before `mj_forward`, rather than zeroing the controls.
- ONNX inference, 241D observation construction, 29D output mapping, GUI targets, `1/240s` simulation step, timestep integration, collision geometry, inertia, contact settings, and WebAssembly asset loading are retained.
- Joint passive stiffness/damping stays disabled to avoid duplication. Existing MJCF `frictionloss` remains.

## Setup

1. Preserve your existing `public/amp_actor.onnx` (this ZIP does not include it).
2. Replace the previous project contents with this archive, then copy `amp_actor.onnx` into `public/`.
3. Stop Vite completely and run `npm install` and `npm run dev`.
4. Check for `[MuJoCo] Newton-style general position servos: 29; no JS torque PD`.
5. Start with Kp x1.0, Kd x1.0, Force x1.0 and compare stability and recovery against the previous archive.

## Important

MuJoCo native `general` actuators use the same *nominal torque law* as Newton's POS mode, but **the solvers are different**; matching trajectories is not guaranteed. The actuator parameters are applied at each 240Hz physics step, while NN inference is at 30Hz. Actual browser runtime has not been tested in this environment.

## MuJoCo actuator control-range fix

Removed inherited `<motor ctrllimited="true"/>` from `<default>` and set `ctrllimited="false"` on all 29 general position servos. `forcerange` remains enabled for torque limits. This corrects `invalid control range for actuator` on model load; policy, physics, and PD gains are unchanged.


## Newton G1 appearance baseline (STL)

- Imported exactly **36** unique STL files used by **36** visual mesh geoms in the original training `g1.xml` supplied in `g1.zip`.
- The existing `public/g1_collision.xml` is UNCHANGED and still governs all MuJoCo simulation, collision, mass, inertia, friction and 29 general-servo actuators.
- At boot, `src/main.js` reads `public/g1.xml` as a **visual-only manifest**, fetches `public/meshes/*.STL`, attaches them to the appropriate MuJoCo body display groups, and uses per-geom MJCF position, quaternion and RGBA.
- `G1 Appearance (STL)` controls allow hiding STL and revealing collision capsules for inspection. STL meshes are never added to MuJoCo's collision system.
- Keep `public/amp_actor.onnx` from your previous working install (not included). Run `npm install`, `npm run dev`.
- The visual model is based on raw STL positions in body-local MJCF coordinates. If a mesh offset appears, compare with the original MuJoCo mesh asset reference transformation; physics behavior is unaffected.


## npm registry rollback safeguard

Rollup is pinned to 4.68.3 under `overrides` and Vite to 6.2.0 to avoid npm resolving a missing tarball. If a stale local `package-lock.json` refers to rollup 4.68.5, remove the lockfile and install again.
