"""
P4-Step2: 手动 Pick-and-Place (纯 MuJoCo, 无 ROS2/MoveIt2)
==========================================================

8 阶段状态机实现:
  1. 预抓取 (pre-grasp): 移动到方块上方 10cm
  2. 下降 (approach): 移动到方块正上方
  3. 抓取 (grasp): 激活 weld constraint
  4. 提升 (lift): 提升方块
  5. 预放置 (pre-place): 移动到放置位上方
  6. 下降 (descend): 移动到放置位
  7. 释放 (release): 停用 weld constraint
  8. 撤退 (retreat): 上移离开

控制方式: Position actuator (Menagerie pattern)
  data.ctrl[:] = target_q 直接设目标角度，MuJoCo 内置 PD 伺服

用法:
  python3 step2_manual_pick_place.py           # 验证模式 (保存轨迹图)
  python3 step2_manual_pick_place.py --view    # 交互查看器 (观察全过程)
  python3 step2_manual_pick_place.py --test    # 完整测试 (验证 + 轨迹)
"""

import os
import sys
import numpy as np
import mujoco as mj

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, 'gofa_table_block.xml')
RESOURCES  = os.path.join(SCRIPT_DIR, '..', '..', 'resources', 'p4')

os.makedirs(RESOURCES, exist_ok=True)

DT = 0.002          # MuJoCo timestep
MOVE_DURATION = 0.4  # seconds per phase ramp (quintic S-curve)

def quintic_interp(t: float, duration: float) -> float:
    """Quintic S-curve: zero vel/accel at start and end."""
    if t <= 0:
        return 0.0
    if t >= duration:
        return 1.0
    frac = t / duration
    return 6*frac**5 - 15*frac**4 + 10*frac**3

def ptp_target(start, goal, t, duration):
    """Interpolate from start → goal using quintic S-curve."""
    s = quintic_interp(t, duration)
    return start + (goal - start) * s

# Target XYZ positions (world coordinates)
# Block centers at z=0.475, block_top at z=0.50
# gripper_site at tool0 needs to touch block_top
PICK_SITE_XYZ = np.array([0.50, -0.15, 0.50])     # gripper_site at block_top
PLACE_SITE_XYZ = np.array([0.50, 0.15, 0.49])     # block above table (drops 1cm)
PICK_TOOL_XYZ = np.array([0.50, -0.15, 0.50])     # tool0 above block (for pre-grasp)
PLACE_TOOL_XYZ = np.array([0.50, 0.15, 0.475])
APPROACH_HEIGHT = 0.10


def set_ctrl(model, data, target_q):
    """Position actuator: ctrl = target joint angle directly."""
    data.ctrl[:] = target_q


def numerical_ik(model, data, target_xyz, site_name='tool0', q_init=None, max_iter=500):
    """Gradient-based numerical IK using finite-difference Jacobian + DLS.

    Args:
        target_xyz: Target position in world frame
        site_name: MuJoCo site or body name to track (default 'tool0')
        q_init: Initial joint guess (6,)
    """
    if q_init is None:
        q_init = np.zeros(6)

    q = q_init.copy().astype(float)
    eps = 0.0005       # perturbation for finite differences
    lr = 0.5            # step size
    lambda_damp = 0.05  # damping for DLS

    best_q, best_err = q.copy(), float('inf')
    jnt_range = model.jnt_range[1:7]  # (6, 2)

    # Resolve site/body id
    target_id = None
    try:
        target_id = model.site(site_name).id
        get_pos = lambda d: d.site(site_name).xpos.copy()
    except KeyError:
        target_id = model.body(site_name).id
        get_pos = lambda d: d.body(site_name).xpos.copy()

    for iteration in range(max_iter):
        data.qpos[:6] = q
        mj.mj_forward(model, data)
        current_pos = get_pos(data)
        error = target_xyz - current_pos
        pos_err = np.linalg.norm(error)

        if pos_err < best_err:
            best_err = pos_err
            best_q = q.copy()

        if pos_err < 0.001:  # converged
            break

        # Finite-difference Jacobian (3x6)
        J = np.zeros((3, 6))
        for j in range(6):
            q_plus = q.copy()
            q_plus[j] += eps
            data.qpos[:6] = q_plus
            mj.mj_forward(model, data)
            pos_plus = get_pos(data)
            J[:, j] = (pos_plus - current_pos) / eps

        # Damped least squares: dq = J^T (J J^T + λI)^{-1} e
        JJT = J @ J.T
        dq = J.T @ np.linalg.solve(JJT + lambda_damp * np.eye(3), error)

        # Apply step with joint limit clamping
        q += lr * dq
        for j in range(6):
            lo, hi = jnt_range[j]
            q[j] = np.clip(q[j], max(lo, -3.0), min(hi, 3.0))

    # Final evaluation
    data.qpos[:6] = best_q
    mj.mj_forward(model, data)

    return best_q, best_err


class PickPlaceStateMachine:
    """8-phase pick-and-place state machine."""

    PHASES = [
        'PRE_GRASP',   # Move to pick position + approach height
        'APPROACH',    # Move down to pick position
        'GRASP',       # Activate suction
        'LIFT',        # Move up high (clear table)
        'PRE_PLACE',   # Move to place side at same high altitude
        'DESCEND',     # Move down to place position
        'RELEASE',     # Deactivate suction
        'RETREAT',     # Move up away
        'COMPLETE',    # Done
    ]

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.phase_idx = 0
        self.phase_time = 0.0
        self.q_target = np.zeros(6)
        self.q_ramp_start = np.zeros(6)  # joint angles at phase start
        self.q_ramp_dur = MOVE_DURATION  # seconds to reach target
        self.q_pick = None
        self.q_place = None
        self.eq_grasp_id = model.equality('grasp').id
        self._grasp_active = False

        print("求解 IK pick (gripper_site → block_top)...")
        self.q_pick, err_pick = numerical_ik(model, data, PICK_SITE_XYZ, site_name='gripper_site')
        print(f"  pick  IK: q={np.degrees(self.q_pick).round(1)}° ")
        print(f"            error={err_pick*1000:.1f}mm")

        print("求解 IK place (gripper_site → place position)...")
        self.q_place, err_place = numerical_ik(model, data, PLACE_SITE_XYZ, site_name='gripper_site')
        print(f"  place IK: q={np.degrees(self.q_place).round(1)}° ")
        print(f"            error={err_place*1000:.1f}mm")

        # Compute safe clearance height: well above table (Z=0.465)
        # Use the higher of pick/place tool0 Z + generous margin
        data.qpos[:6] = self.q_pick
        mj.mj_forward(model, data)
        pick_tool0 = data.body('tool0').xpos.copy()
        data.qpos[:6] = self.q_place
        mj.mj_forward(model, data)
        place_tool0 = data.body('tool0').xpos.copy()

        safe_z = max(pick_tool0[2], place_tool0[2]) + 0.50  # 50cm above highest point

        # Pre-grasp: tool0 at pick position + approach height
        self.pregrasp_xyz = pick_tool0.copy()
        self.pregrasp_xyz[2] += APPROACH_HEIGHT
        print("求解 IK (pre-grasp, tool0 + approach)...")
        self.q_pregrasp, _ = numerical_ik(model, data, self.pregrasp_xyz, q_init=self.q_pick)

        # LIFT: tool0 at pick XY but at safe clearance height
        lift_xyz = np.array([pick_tool0[0], pick_tool0[1], safe_z])
        print(f"求解 IK (lift, tool0 → Z={safe_z:.2f})...")
        self.q_lift, _ = numerical_ik(model, data, lift_xyz, q_init=self.q_pick)

        # Pre-place: tool0 at place XY but at safe clearance height (SAME height as lift)
        preplace_xyz = np.array([place_tool0[0], place_tool0[1], safe_z])
        print(f"求解 IK (pre-place high, tool0 → {preplace_xyz.round(2)})...")
        self.q_preplace, _ = numerical_ik(model, data, preplace_xyz, q_init=self.q_lift)

        print(f"\n{'='*60}")
        print("状态机就绪。开始 pick-and-place 序列。")
        print(f"{'='*60}")

    @property
    def phase(self):
        return self.PHASES[self.phase_idx]

    def get_ctrl(self):
        """Return interpolated control target using quintic S-curve."""
        return ptp_target(self.q_ramp_start, self.q_target, self.phase_time, self.q_ramp_dur)

    def step(self):
        """Advance the state machine one phase if dwell time expired."""
        self.phase_time += DT

        phase_max_time = max(self.q_ramp_dur + 0.5, 3.0)

        if self.phase == 'PRE_GRASP':
            self.q_target = self.q_pregrasp
            if self._arrived() or self.phase_time > phase_max_time:
                self._next('APPROACH')

        elif self.phase == 'APPROACH':
            self.q_target = self.q_pick
            if self._arrived() or self.phase_time > phase_max_time:
                self._next('GRASP')

        elif self.phase == 'GRASP':
            self._set_grasp(True)
            self._next('LIFT')

        elif self.phase == 'LIFT':
            self.q_target = self.q_lift
            if self._arrived() or self.phase_time > phase_max_time:
                self._next('PRE_PLACE')

        elif self.phase == 'PRE_PLACE':
            self.q_target = self.q_preplace
            if self._arrived() or self.phase_time > phase_max_time:
                self._next('DESCEND')

        elif self.phase == 'DESCEND':
            self.q_target = self.q_place
            if self._arrived() or self.phase_time > phase_max_time:
                self._next('RELEASE')

        elif self.phase == 'RELEASE':
            self._set_grasp(False)
            self._next('RETREAT')

        elif self.phase == 'RETREAT':
            self.q_target = self.q_preplace
            if self._arrived() or self.phase_time > phase_max_time:
                self._next('COMPLETE')

        elif self.phase == 'COMPLETE':
            pass  # Hold position

    def _arrived(self, tol=0.03):
        """Check if ramp complete AND joints near target."""
        ramp_done = self.phase_time >= self.q_ramp_dur
        joints_near = np.all(np.abs(self.data.qpos[:6] - self.q_target) < tol)
        return ramp_done and joints_near

    def _next(self, phase_name):
        phase_idx = self.PHASES.index(phase_name)
        # Save current actual joint position as ramp start
        self.q_ramp_start = self.data.qpos[:6].copy()
        errs = np.abs(self.data.qpos[:6] - self.q_target)
        err_str = ' '.join([f'j{i+1}={e*180/3.1416:.1f}°' for i, e in enumerate(errs)])
        print(f"  [{self.phase:10s}] → [{phase_name:10s}]  "
              f"t={self.phase_time:.2f}s  max_err={self._joint_err()*180/3.1416:.1f}°")
        print(f"    per-joint errors: {err_str}")
        print(f"    current q: {np.degrees(self.data.qpos[:6]).round(1)}")
        print(f"    target  q: {np.degrees(self.q_target).round(1)}")
        self.phase_idx = phase_idx
        self.phase_time = 0.0

    def _joint_err(self):
        """Max joint error in radians."""
        return np.max(np.abs(self.data.qpos[:6] - self.q_target))

    def _set_grasp(self, active):
        """Teleport-based grasp: track block to gripper site during grasp.

        During GRASP/LIFT/PRE_PLACE/DESCEND phases, block is teleported
        to gripper_site position each step. RELEASE stops teleport.
        """
        self._grasp_active = active
        if active:
            print(f"  ✓ GRASP: 方块已吸附")
        else:
            print(f"  ✓ RELEASE: 方块已释放")


def run_test():
    """Run pick-and-place in test mode, log data for plotting."""
    model = mj.MjModel.from_xml_path(MODEL_PATH)
    data = mj.MjData(model)

    sm = PickPlaceStateMachine(model, data)

    max_time = 30.0
    n_steps = int(max_time / DT)

    time_log, q_log, block_log, phase_log = [], [], [], []

    print("\n开始仿真...\n")

    for step in range(n_steps):
        t = step * DT
        sm.step()

        if sm.phase == 'COMPLETE':
            print(f"\n✅ Pick-and-place 完成! t={t:.2f}s")
            break

        set_ctrl(model, data, sm.get_ctrl())
        if sm._grasp_active:
            # Teleport block to gripper site, zero velocity
            data.qpos[6:9] = data.site('gripper_site').xpos.copy()
            data.qvel[6:12] = 0.0
        mj.mj_step(model, data)

        # Log every 100 steps
        if step % 100 == 0:
            time_log.append(t)
            q_log.append(data.qpos[:6].copy())
            block_log.append(data.body('block').xpos.copy())
            phase_log.append(sm.phase)

    # Verify result
    block_final = data.body('block').xpos.copy()
    expected = PLACE_SITE_XYZ
    dist_error = np.linalg.norm(block_final - expected)

    print(f"\n{'='*60}")
    print("验证结果")
    print(f"{'='*60}")
    print(f"  方块最终位置: [{block_final[0]:.3f}, {block_final[1]:.3f}, {block_final[2]:.3f}]")
    print(f"  目标放置位置: [{expected[0]:.3f}, {expected[1]:.3f}, {expected[2]:.3f}]")
    print(f"  距离误差: {dist_error*1000:.1f}mm")

    phase_times = {}
    for i, ph in enumerate(PickPlaceStateMachine.PHASES):
        indices = [j for j, p in enumerate(phase_log) if p == ph]
        if indices:
            t_start = time_log[indices[0]]
            t_end = time_log[indices[-1]]
            phase_times[ph] = (t_start, t_end)

    if dist_error < 0.05:
        print(f"  状态: ✅ PASS")
    else:
        print(f"  状态: ❌ FAIL (误差 > 50mm)")

    # Generate trajectory plot
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        t_arr = np.array(time_log)
        q_arr = np.array(q_log)
        b_arr = np.array(block_log)

        fig, axes = plt.subplots(2, 1, figsize=(12, 8))

        # Joint positions
        colors = plt.cm.tab10(np.linspace(0, 1, 6))
        for j in range(6):
            axes[0].plot(t_arr, np.degrees(q_arr[:, j]), color=colors[j],
                        label=f'joint_{j+1}', linewidth=1)
        axes[0].set_ylabel('Joint Angle (deg)')
        axes[0].set_title('GoFa Pick-and-Place: Joint Trajectories')
        axes[0].legend(ncol=3, fontsize='small')
        axes[0].grid(True, alpha=0.3)

        # Block position
        axes[1].plot(t_arr, b_arr[:, 0], 'r-', label='block X', linewidth=1.5)
        axes[1].plot(t_arr, b_arr[:, 1], 'g-', label='block Y', linewidth=1.5)
        axes[1].plot(t_arr, b_arr[:, 2], 'b-', label='block Z', linewidth=1.5)
        axes[1].axhline(y=PLACE_SITE_XYZ[1], color='gray', linestyle='--', alpha=0.5, label='target Y')
        axes[1].set_ylabel('Block Position (m)')
        axes[1].set_xlabel('Time (s)')
        axes[1].set_title('Block Position During Pick-and-Place')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        # Phase transitions
        for ph, (t0, t1) in phase_times.items():
            if ph in ['GRASP', 'RELEASE']:
                axes[0].axvspan(t0, t1, alpha=0.1, color='yellow')
                axes[1].axvspan(t0, t1, alpha=0.1, color='yellow')

        plt.tight_layout()
        path = os.path.join(RESOURCES, 'p4_step2_trajectory.png')
        plt.savefig(path, dpi=150)
        print(f"\n保存轨迹图: {path}")
    except ImportError:
        print("  (matplotlib 未安装, 跳过绘图)")


def run_viewer():
    """Run pick-and-place with interactive viewer (auto-loop + keyboard)."""
    model = mj.MjModel.from_xml_path(MODEL_PATH)
    data = mj.MjData(model)

    sm = PickPlaceStateMachine(model, data)

    # Loop state
    LOOP_HOLD = 2.0      # seconds to hold at COMPLETE before reset
    RESET_HOLD = 0.5     # seconds to hold at zero before next cycle
    RESET_RAMP = 0.8     # seconds for smooth reset ramp
    INIT_BLOCK_QPOS = np.array([0.50, -0.15, 0.475, 1.0, 0.0, 0.0, 0.0])

    _state = {'phase': 'running', 'timer': 0.0, 'cycle': 0,
              'reset_start': np.zeros(6)}

    def controller(m, d):
        s = _state

        if s['phase'] == 'running':
            sm.step()
            set_ctrl(m, d, sm.get_ctrl())
            if sm._grasp_active:
                d.qpos[6:9] = d.site('gripper_site').xpos.copy()
                d.qvel[6:12] = 0.0

            if sm.phase == 'COMPLETE':
                s['phase'] = 'complete_hold'
                s['timer'] = 0.0
                s['cycle'] += 1
                block = d.body('block').xpos.copy()
                err = np.linalg.norm(block - PLACE_SITE_XYZ) * 1000
                print(f"\n[Cycle {s['cycle']}] Pick-and-place 完成! "
                      f"block=({block[0]:.3f},{block[1]:.3f},{block[2]:.3f}) "
                      f"err={err:.1f}mm")

        elif s['phase'] == 'complete_hold':
            set_ctrl(m, d, sm.get_ctrl())
            s['timer'] += DT
            if s['timer'] >= LOOP_HOLD:
                print(f"  重置中...")
                s['phase'] = 'resetting_arm'
                s['timer'] = 0.0
                s['reset_start'] = d.qpos[:6].copy()
                sm.phase_idx = 0
                sm.phase_time = 0.0
                sm._grasp_active = False

        elif s['phase'] == 'resetting_arm':
            # Smooth quintic ramp to zero
            zero_target = np.zeros(6)
            interp = ptp_target(s['reset_start'], zero_target, s['timer'], RESET_RAMP)
            set_ctrl(m, d, interp)
            # Teleport block back to pick position
            d.qpos[6:9] = INIT_BLOCK_QPOS[:3]
            d.qvel[6:12] = 0.0
            s['timer'] += DT
            if s['timer'] >= RESET_RAMP + RESET_HOLD:
                s['phase'] = 'reset_hold'
                s['timer'] = 0.0

        elif s['phase'] == 'reset_hold':
            set_ctrl(m, d, np.zeros(6))
            s['timer'] += DT
            if s['timer'] >= RESET_HOLD:
                # Restart state machine
                sm.phase_idx = 0
                sm.phase_time = 0.0
                sm._grasp_active = False
                sm.q_ramp_start = d.qpos[:6].copy()
                sm.q_target = np.zeros(6)
                d.qpos[6:9] = INIT_BLOCK_QPOS[:3]
                d.qvel[6:12] = 0.0
                s['phase'] = 'running'
                print(f"  → 开始下一轮循环")

    mj.set_mjcb_control(controller)

    print("\n" + "=" * 60)
    print("  P4 交互查看器 — Pick-and-Place 自动循环")
    print("=" * 60)
    print("  Space  暂停 / 恢复 (检查关节角度)")
    print("  Tab    打开/关闭调试面板")
    print("  Esc    退出")
    print("  左键拖拽 旋转视角 | 右键拖拽 平移 | 滚轮 缩放")
    print("=" * 60)
    print()

    from mujoco import viewer
    viewer.launch(model, data)


if __name__ == '__main__':
    if '--view' in sys.argv:
        run_viewer()
    else:
        run_test()
