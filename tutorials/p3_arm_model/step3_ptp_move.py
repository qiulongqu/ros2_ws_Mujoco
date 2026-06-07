"""
P3-Step3: 关节空间点到点运动 (PTP)
====================================

使用 MuJoCo position actuator (Menagerie 模式) 实现平滑的关节空间
点到点运动。内置 PD 伺服由 MuJoCo C 引擎执行，Python 侧只需
发送插值后的目标位置。

【用法】
  # 测试模式 (自动验证)
  python3 step3_ptp_move.py --test

  # 指定目标关节角 (从当前位姿移动)
  python3 step3_ptp_move.py --joints "0,-0.5,0.3,0,0,0"

  # 交互模式 (viewer)
  python3 step3_ptp_move.py
"""

import os
import sys
import numpy as np
import mujoco as mj

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, 'gofa_crb15000.xml')
RESOURCES = os.path.join(SCRIPT_DIR, '..', '..', 'resources', 'p3')

DT = 0.002          # simulation timestep
MOVE_DURATION = 1.5 # seconds per PTP move
HOLD_DURATION = 0.5 # seconds hold at target before next move


def quintic_interp(t: float, duration: float) -> float:
    """Quintic polynomial: zero vel/accel at start and end.

    s(0)=0, s'(0)=0, s''(0)=0
    s(T)=1, s'(T)=0, s''(T)=0
    """
    if t <= 0:
        return 0.0
    if t >= duration:
        return 1.0
    frac = t / duration
    return 6*frac**5 - 15*frac**4 + 10*frac**3


def ptp_target(start: np.ndarray, goal: np.ndarray, t: float, duration: float) -> np.ndarray:
    """Interpolate from start to goal using quintic S-curve."""
    s = quintic_interp(t, duration)
    return start + (goal - start) * s


def run_ptp_sequence(model, data, targets_deg, move_dur=MOVE_DURATION, hold_dur=HOLD_DURATION, verbose=True):
    """Execute a sequence of PTP moves and report tracking errors.

    Args:
        model, data: MuJoCo model and data (data.ctrl should be pre-set)
        targets_deg: list of [6] joint targets in degrees
        move_dur: seconds per move
        hold_dur: seconds hold after each move
        verbose: print per-move info

    Returns:
        list of (target_name, max_error_deg, final_qpos_deg) tuples
    """
    results = []
    for idx, tgt_deg in enumerate(targets_deg):
        tgt_rad = np.radians(tgt_deg)
        start = data.qpos.copy()
        move_steps = int(move_dur / DT)
        hold_steps = int(hold_dur / DT)

        # Move phase
        max_err = 0.0
        for step in range(move_steps):
            t = step * DT
            interp = ptp_target(start, tgt_rad, t, move_dur)
            data.ctrl[:] = interp
            mj.mj_step(model, data)

            err = np.max(np.abs(data.qpos - interp))
            if err > max_err:
                max_err = err

        # Hold phase
        for step in range(hold_steps):
            data.ctrl[:] = tgt_rad
            mj.mj_step(model, data)

        mj.mj_forward(model, data)
        final_err = np.max(np.abs(data.qpos - tgt_rad))

        if verbose:
            name = f"move_{idx+1}"
            print(f"  {name} → {tgt_deg}:")
            print(f"    max_tracking_err={np.degrees(max_err):.3f}°  final_err={np.degrees(final_err):.3f}°")

        results.append((tgt_deg, np.degrees(final_err), np.degrees(data.qpos)))

    return results


def print_banner(title: str):
    print()
    print("=" * 65)
    print(f"  {title}")
    print("=" * 65)


if __name__ == '__main__':
    FORCE_TEST = '--test' in sys.argv
    FORCE_DEMO = '--demo' in sys.argv
    CUSTOM_JOINTS = None
    for i, arg in enumerate(sys.argv):
        if arg == '--joints' and i + 1 < len(sys.argv):
            CUSTOM_JOINTS = np.array([float(x.strip()) for x in sys.argv[i+1].split(',')])
            break

    model = mj.MjModel.from_xml_path(MODEL_PATH)
    data = mj.MjData(model)

    print_banner("P3-Step3: GoFa CRB15000 点到点运动 (Position Actuator)")
    print(f"  Actuator: position (kp=4000~1500, kv=400~150)")
    print(f"  Integrator: implicit, dt={DT}s")
    print(f"  Move duration: {MOVE_DURATION}s, Hold: {HOLD_DURATION}s")

    if FORCE_DEMO:
        # --- Demo mode: offline render PTP motion to GIF ---
        print("\n[Demo] 离线渲染 PTP 运动过程...\n")

        import glfw
        glfw.init()
        glfw.window_hint(glfw.VISIBLE, 0)
        os.environ['MUJOCO_GL'] = 'glfw'

        # Use MuJoCo renderer (not viewer) for frame capture
        renderer = mj.Renderer(model, 900, 1200)

        # Warm up
        data.ctrl[:] = 0.0
        for _ in range(500):
            mj.mj_step(model, data)

        # Record a PTP move: zero → pose A
        target_deg = np.array([30, -30, 20, 45, -30, 60])
        target_rad = np.radians(target_deg)
        start = data.qpos.copy()
        dur = 2.0
        n_frames = int(dur / DT)
        frame_skip = 40  # capture every 40 steps (~12.5 fps)

        frames = []
        print(f"  目标: {target_deg}°")
        print(f"  录制 {n_frames} steps, 每 {frame_skip} step 截帧...")

        for step in range(n_frames):
            t = step * DT
            interp = ptp_target(start, target_rad, t, dur)
            data.ctrl[:] = interp
            mj.mj_step(model, data)

            if step % frame_skip == 0:
                mj.mj_forward(model, data)
                renderer.update_scene(data)
                img = renderer.render()
                frames.append(img.copy())

        # Hold at target
        for _ in range(int(1.0 / DT)):
            data.ctrl[:] = target_rad
            mj.mj_step(model, data)
        mj.mj_forward(model, data)
        renderer.update_scene(data)
        frames.append(renderer.render().copy())

        renderer.close()
        glfw.terminate()

        # Save GIF
        gif_path = os.path.join(RESOURCES, 'p3_step3_ptp_demo.gif')
        from PIL import Image
        pil_frames = [Image.fromarray(f) for f in frames]
        pil_frames[0].save(gif_path, save_all=True, append_images=pil_frames[1:],
                          duration=100, loop=0)
        print(f"\n  Saved GIF: {gif_path} ({len(frames)} frames)")

        # Also save first/last frame as PNG
        Image.fromarray(frames[0]).save(os.path.join(RESOURCES, 'p3_step3_ptp_start.png'))
        Image.fromarray(frames[-1]).save(os.path.join(RESOURCES, 'p3_step3_ptp_end.png'))
        print(f"  Saved PNGs: p3_step3_ptp_start.png, p3_step3_ptp_end.png")

    if CUSTOM_JOINTS is not None:
        # --- Custom PTP move ---
        print(f"\n目标关节角: {CUSTOM_JOINTS}°")
        data.ctrl[:] = 0.0

        # Warm up
        for _ in range(500):
            mj.mj_step(model, data)

        start = np.degrees(data.qpos)
        print(f"起始关节角: {np.round(start, 2)}°")

        run_ptp_sequence(model, data, [CUSTOM_JOINTS])
        print(f"\n最终关节角: {np.round(np.degrees(data.qpos), 2)}°")

    elif FORCE_TEST:
        # --- Auto test sequence ---
        data.ctrl[:] = 0.0

        # Phase 1: warm-up
        print("\n[Phase 1] 预热 (零位保持, 500 steps)...")
        for _ in range(500):
            mj.mj_step(model, data)
        mj.mj_forward(model, data)
        max_dev = np.max(np.abs(data.qpos))
        print(f"  最大偏移: {np.degrees(max_dev):.4f}°"
              f"  {'PASS' if np.degrees(max_dev) < 1.0 else 'FAIL'}")
        print(f"  tool0 z: {data.xpos[model.body('tool0').id][2]:.3f}m")

        # Phase 2: sequence of PTP moves
        print_banner("[Phase 2] PTP 移动序列")

        test_sequence = [
            [  0, -30,  20,   0,   0,   0],   # move j2, j3
            [ 30, -15,   0,  45, -30,  60],   # 6-joint combined
            [  0,   0,   0,   0,   0,   0],   # return zero
            [ -30, 10, -20, -45,  30, -60],   # opposite direction
            [  0,   0,   0,   0,   0,   0],   # return zero
        ]

        all_results = run_ptp_sequence(model, data, test_sequence)

        # Summary
        print_banner("验证总结")
        threshold = 2.0  # degrees
        all_pass = True
        for tgt, final_err, actual in all_results:
            ok = final_err < threshold
            if not ok:
                all_pass = False
            status = "PASS" if ok else "FAIL"
            print(f"  target={np.round(tgt, 1)}° → final_err={final_err:.2f}°  [{status}]")

        print(f"\n  验收标准: 稳态误差 < {threshold}°")
        if all_pass:
            print(f"  ✅ 全部通过!")
        else:
            print(f"  ❌ 部分失败")

        # Phase 3: plot
        print_banner("[Phase 3] 绘制运动曲线")

        # re-run a single move and record
        data2 = mj.MjData(model)
        data2.ctrl[:] = 0.0
        for _ in range(500):
            mj.mj_step(model, data2)

        tgt_demo = np.radians([30, -15, 20, 45, -30, 60])
        start = data2.qpos.copy()
        dur = MOVE_DURATION
        n_steps = int(dur / DT)

        t_arr = np.zeros(n_steps)
        pos_log = np.zeros((n_steps, 6))
        ref_log = np.zeros((n_steps, 6))

        for step in range(n_steps):
            t = step * DT
            interp = ptp_target(start, tgt_demo, t, dur)
            data2.ctrl[:] = interp
            mj.mj_step(model, data2)

            t_arr[step] = t
            pos_log[step] = data2.qpos
            ref_log[step] = interp

        err_log = np.max(np.abs(pos_log - ref_log), axis=1)

        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            fig, axes = plt.subplots(3, 1, figsize=(12, 10))

            for i in range(6):
                axes[0].plot(t_arr, np.degrees(pos_log[:, i]),
                           label=f'j{i+1}', linewidth=1)
                axes[0].plot(t_arr, np.degrees(ref_log[:, i]),
                           '--', alpha=0.5, linewidth=0.8)
            axes[0].set_ylabel('Joint Angle (deg)')
            axes[0].set_title('PTP Motion: S-Curve Trajectory Tracking')
            axes[0].legend(ncol=3, fontsize='small')
            axes[0].grid(True, alpha=0.3)

            axes[1].fill_between(t_arr, 0, np.degrees(err_log), alpha=0.3, color='red')
            axes[1].plot(t_arr, np.degrees(err_log), 'r-', linewidth=1.5)
            axes[1].axhline(threshold, color='gray', linestyle='--',
                          label=f'{threshold}° threshold')
            axes[1].set_ylabel('Max Error (deg)')
            axes[1].set_title('Tracking Error')
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)

            axes[2].plot(np.degrees(pos_log[:, 1]), np.degrees(pos_log[:, 2]),
                       'b-', linewidth=1, alpha=0.7)
            axes[2].plot(np.degrees(ref_log[:, 1]), np.degrees(ref_log[:, 2]),
                       'r--', linewidth=1, alpha=0.5)
            axes[2].set_xlabel('Joint 2 (deg)')
            axes[2].set_ylabel('Joint 3 (deg)')
            axes[2].set_title('Phase Portrait: Joint 2 vs Joint 3')
            axes[2].grid(True, alpha=0.3)
            axes[2].set_aspect('equal')

            plt.tight_layout()
            path = os.path.join(RESOURCES, 'p3_step3_ptp_move.png')
            plt.savefig(path, dpi=150)
            print(f"  Saved: {path}")
        except ImportError:
            print("  matplotlib 不可用, 跳过绘图")

    else:
        # --- Viewer mode: PTP auto-loop ---
        print("\n启动 MuJoCo viewer (PTP 自动循环)...\n")
        print("  Space 暂停 | Tab 面板 | Esc 退出")
        print("  如果窗口不显示: 检查 WSL2 X server (WSLg/VcXsrv) 是否运行")
        print("=" * 65)

        # PTP targets the viewer will cycle through
        VIEWER_TARGETS = np.radians([
            [  0,   0,   0,   0,   0,   0],   # zero
            [ 30, -30,  20,  45, -30,  60],   # pose A
            [-30,  10, -20, -45,  30, -60],   # pose B
            [  0, -45,  45,   0,   0,   0],   # pose C
        ])
        _move_dur = 2.0  # slower for viewing
        _hold_dur = 1.0

        # mutable state for control callback
        _state = {
            'start': np.zeros(6),
            'target_idx': 0,
            'phase': 'move',   # 'move' | 'hold'
            'phase_t': 0.0,
        }

        def _ctrl_cb(model, data):
            s = _state
            tgt = VIEWER_TARGETS[s['target_idx']]

            if s['phase'] == 'move':
                interp = ptp_target(s['start'], tgt, s['phase_t'], _move_dur)
                data.ctrl[:] = interp
                s['phase_t'] += model.opt.timestep
                if s['phase_t'] >= _move_dur:
                    s['phase'] = 'hold'
                    s['phase_t'] = 0.0
            else:  # hold
                data.ctrl[:] = tgt
                s['phase_t'] += model.opt.timestep
                if s['phase_t'] >= _hold_dur:
                    s['start'] = tgt.copy()
                    s['target_idx'] = (s['target_idx'] + 1) % len(VIEWER_TARGETS)
                    s['phase'] = 'move'
                    s['phase_t'] = 0.0

        mj.set_mjcb_control(_ctrl_cb)

        # 确认 X server 可用
        import os as _os
        display = _os.environ.get('DISPLAY', 'NOT SET')
        print(f"DISPLAY={display}")

        from mujoco import viewer
        viewer.launch(model, data)

        print("\nViewer 已关闭")

    print("\n" + "=" * 65)
