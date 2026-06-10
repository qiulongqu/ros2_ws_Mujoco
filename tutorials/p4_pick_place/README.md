# P4: Pick-and-Place (Pure MuJoCo · 纯 MuJoCo)

> 8-phase state machine + numerical IK + teleport-based suction — no ROS2, no MoveIt2.
> 8 阶段状态机 + 数值 IK + teleport 吸附, 纯 MuJoCo 实现.

![status](https://img.shields.io/badge/status-passed-brightgreen) ![runtime](https://img.shields.io/badge/runtime-3.04s-blue) ![accuracy](https://img.shields.io/badge/place_error-7mm-green) ![deps](https://img.shields.io/badge/deps-mujoco%20%7C%20numpy-orange)

---

## What is this

A complete **pick-and-place** pipeline for the **ABB GoFa CRB15000** arm running entirely in **MuJoCo**.
The arm picks a red block off a table, lifts it across, and places it at a target spot — all driven by an 8-phase finite state machine with smooth quintic S-curve interpolation between waypoints.

**Why pure MuJoCo (no ROS2/MoveIt2 here):**
- Isolate and verify the **control + planning logic** before adding any middleware
- Master the foundational patterns: state machines, IK, contact, grasping
- Future P5 will layer MoveIt2 on top (already partially integrated at workspace root)

---

## Quick Start

```bash
conda activate mujoco
cd /home/yunhao2204/ros2_ws_abb/mujocoONLY

# 1. Verify the scene (7 tests, no GUI needed)
python3 tutorials/p4_pick_place/step1_build_scene.py

# 2. Run pick-and-place (auto + trajectory plot)
python3 tutorials/p4_pick_place/step2_manual_pick_place.py

# 3. Watch it in interactive viewer (auto-loop, Space to pause)
python3 tutorials/p4_pick_place/step2_manual_pick_place.py --view
```

**Total time-to-first-cycle:** < 30 seconds (after `mujoco` env is installed).

---

## Results (2026-06-09)

| Metric | Target | Actual | Status |
|--------|--------|--------|--------|
| Cycle time | < 5 s | **3.04 s** | ✅ |
| Place position error | < 50 mm | **7.0 mm** | ✅ |
| Max joint tracking error | < 5° | **1.7°** | ✅ |
| Phase transitions | 8/8 | **8/8** | ✅ |
| Scene validation tests | 7/7 | **7/7** | ✅ |

> Output artifacts: `resources/p4/p4_step1_scene_*.png`, `p4_step2_trajectory.png`

---

## 8-Phase State Machine

```
┌──────────┐    ┌──────────┐    ┌────────┐    ┌────────┐
│PRE_GRASP │ →  │APPROACH  │ →  │GRASP   │ →  │LIFT    │
│上方 10cm │    │下降到方块│    │吸附激活│    │安全高度│
└──────────┘    └──────────┘    └────────┘    └────────┘
                                                ↓
┌──────────┐    ┌──────────┐    ┌────────┐    ┌────────┐
│RETREAT   │ ←  │RELEASE   │ ←  │DESCEND │ ←  │PRE_PLACE│
│回到安全高│    │停用吸附  │    │下降到目│    │移动到上 │
└──────────┘    └──────────┘    └────────┘    └────────┘
```

| # | Phase | Action | Trigger |
|---|-------|--------|---------|
| 1 | `PRE_GRASP` | Move above pick point (+10 cm) | Joints at target |
| 2 | `APPROACH`  | Descend to pick point | Joints at target |
| 3 | `GRASP`     | Activate suction (teleport) | Immediate |
| 4 | `LIFT`      | Move to safe altitude (50 cm) | Joints at target |
| 5 | `PRE_PLACE` | Move above place point (same height) | Joints at target |
| 6 | `DESCEND`   | Descend to place point | Joints at target |
| 7 | `RELEASE`   | Deactivate suction, block falls | Immediate |
| 8 | `RETREAT`   | Return to safe altitude | Joints at target |

---

## Architecture

```
┌─────────────────────────────────────────────────────┐
│  Layer 4 — Physics: MuJoCo engine (dt=2ms, implicit)│
├─────────────────────────────────────────────────────┤
│  Layer 3 — Servo: Position actuator (kp=1500~4000)  │
├─────────────────────────────────────────────────────┤
│  Layer 2 — Trajectory: Quintic S-curve interpolation│
├─────────────────────────────────────────────────────┤
│  Layer 1 — Planning: 8-phase state machine          │
└─────────────────────────────────────────────────────┘
```

| Layer | Component | Implementation |
|-------|-----------|----------------|
| 4 | Physics | MuJoCo with `integrator="implicit"`, contact solver |
| 3 | Servo | `<position>` actuators, `kp*err - kv*vel` (Menagerie pattern) |
| 2 | Trajectory | `6t⁵-15t⁴+10t³` per phase, 0.4 s ramp duration |
| 1 | Planning | `PickPlaceStateMachine` class, IK + waypoint cache |

> Industrial 4-layer separation. P5 will add Layer 0: MoveIt2 L1 planning on top.

---

## Files

```
tutorials/p4_pick_place/
├── gofa_table_block.xml        # MuJoCo model: arm + table + block + suction
├── step1_build_scene.py        # 7 scene tests + render
├── step2_manual_pick_place.py  # 8-phase state machine + IK + viewer
└── README.md                   # this file
```

| File | Lines | Role |
|------|-------|------|
| `gofa_table_block.xml` | 130 | Scene composition (extends P3 arm) |
| `step1_build_scene.py` | 210 | Scene validation, 7 tests, PNG render |
| `step2_manual_pick_place.py` | 460 | FSM, IK, viewer controller, plotting |

---

## Key Design Decisions

**1. Position actuator (Menagerie pattern)**
Instead of writing a manual PD loop, leverage MuJoCo's built-in servo: `torque = kp*(ctrl - qpos) - kv*qvel`. Python side just sets `data.ctrl[:] = target_q`. Less code, more robust.

**2. Implicit integrator**
High kp + low-inertia wrist joints → explicit Euler becomes unstable (dt × ω_n > 2). `integrator="implicit"` is mandatory for numerical stability.

**3. Quintic S-curve between phases**
Without interpolation, every state transition is an instant target step → PD overshoots → end-effector shakes. The 0.4 s quintic ramp makes the motion smooth and the tracking error < 2°.

**4. Teleport-based grasp (replacing `<weld>`)**
MuJoCo 3.8.1's `<weld>` equality constraint between a free body (block) and a kinematic body (link_6) does not activate reliably. Workaround: each step, set `qpos[6:9] = gripper_site.xpos` and `qvel[6:12] = 0`. Physically equivalent to a perfect suction gripper.

**5. Numerical IK with damping**
6-DOF analytical IK is awkward for non-spherical-wrist designs like GoFa. Finite-difference Jacobian (ε=5e-4) + damped least squares (λ=0.05) + joint-limit clamping → 1 mm convergence in 100-500 iterations.

---

## Interactive Viewer Keys

| Key | Action |
|-----|--------|
| `Space` | Pause / resume (inspect joint angles) |
| `Tab` | Toggle debug panel (qpos / qvel / ctrl values) |
| `Esc` | Quit viewer |
| Left drag | Rotate camera |
| Right drag | Pan camera |
| Scroll | Zoom |

In `--view` mode, the script **auto-loops**: complete cycle → hold 2 s → reset → next cycle. Press Space at any moment to freeze and inspect.

---

## Dependencies

```bash
conda create -n mujoco -y
conda activate mujoco
pip install mujoco numpy pillow matplotlib
```

**No ROS2 required for P4.** (ROS2 + MoveIt2 integration is P5, see workspace root.)

---

## Documentation

| Doc | Purpose |
|-----|---------|
| [README.md](README.md) | This file — project entry point |
| [docs/reports/p4_pick_place_技术详解.md](../../docs/reports/p4_pick_place_技术详解.md) | Full technical notes (XML structure, IK derivation, FSM details, troubleshooting) |
| [docs/reports/脚本启动命令.md](../../docs/reports/脚本启动命令.md) | All launch commands across the project |
| [CLAUDE.md](../../CLAUDE.md) | Lessons learned, debugging tips, common pitfalls |
| [../../README.md](../../README.md) | Workspace-level overview, P1-P5 status |

---

## Next Steps → P5

P4 is the **pure-MuJoCo baseline**. P5 layers MoveIt2 on top:

- Replace position actuator + manual IK with `JointTrajectoryController` + MoveIt2 `FollowJointTrajectory` action
- Use MoveIt2 L1 planning (RRT*/OMPL) instead of hand-written waypoints
- Verify L3→L4 end-to-end (joint trajectory → ros2_control → MuJoCo)

See [`p5_moveit2_learning/`](../../p5_moveit2_learning/) for the 6-week roadmap.
