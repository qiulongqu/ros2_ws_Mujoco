"""
P3-Step3: 关节空间轨迹控制 (Motor + 完整 qfrc_bias 前馈)
=========================================================

【原理】
使用 MuJoCo `<motor>` actuator + Python PD 控制 + qfrc_bias 前馈。

  joint_torque = Kp * (target - qpos) - Kd * qvel + qfrc_bias
  ctrl         = joint_torque / gear

qfrc_bias = Coriolis + centrifugal + gravity + passive forces.
完整前馈取消所有非线性项，PD 只处理线性误差，极大提升稳定性。

【用法】
  python3 step3_trajectory_control.py --test   # 测试模式
  python3 step3_trajectory_control.py          # 交互查看 (需 viewer)
"""

import os
import sys
import numpy as np
import mujoco as mj

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, 'gofa_crb15000.xml')
RESOURCES  = os.path.join(SCRIPT_DIR, '..', '..', 'resources', 'p3')

KP = np.array([500.0, 500.0, 500.0, 300.0, 300.0, 300.0])
KD = np.array([50.0,  50.0,  50.0,  30.0,  30.0,  30.0])
GEAR = np.array([100.0, 100.0, 100.0, 50.0, 50.0, 50.0])

TRAJ_PARAMS = [
    {"amp": np.radians(45), "freq": 0.20, "phase": 0.0},
    {"amp": np.radians(20), "freq": 0.25, "phase": 0.5},
    {"amp": np.radians(20), "freq": 0.30, "phase": 1.0},
    {"amp": np.radians(40), "freq": 0.22, "phase": 1.5},
    {"amp": np.radians(30), "freq": 0.28, "phase": 2.0},
    {"amp": np.radians(50), "freq": 0.18, "phase": 2.5},
]

DT = 0.002
RAMP_TIME = 1.0


def traj_target(t: float, ramp: bool = True) -> np.ndarray:
    targets = np.zeros(6)
    for i, p in enumerate(TRAJ_PARAMS):
        omega = 2 * np.pi * p["freq"]
        targets[i] = p["amp"] * np.sin(omega * t + p["phase"])
    if ramp and t < RAMP_TIME:
        r = 0.5 * (1.0 - np.cos(np.pi * t / RAMP_TIME))
        targets *= r
    return targets


def set_pd_ctrl(model, data, target: np.ndarray):
    """PD + full qfrc_bias feedforward (gravity + Coriolis + centrifugal)."""
    torque = KP * (target - data.qpos) - KD * data.qvel + data.qfrc_bias
    data.ctrl[:] = torque / GEAR


if __name__ == '__main__':
    FORCE_TEST = '--test' in sys.argv

    model = mj.MjModel.from_xml_path(MODEL_PATH)
    data = mj.MjData(model)

    print("=" * 60)
    print("P3-Step3: GoFa CRB15000 关节轨迹控制 (qfrc_bias 前馈)")
    print("=" * 60)
    print(f"Kp: {KP} Nm/rad")
    print(f"Kd: {KD} Nms/rad")
    print()

    if FORCE_TEST:
        total_time = 5.0
        n_steps = int(total_time / DT)

        # 预热: PD + qfrc_bias 保持零位
        for _ in range(200):
            set_pd_ctrl(model, data, np.zeros(6))
            mj.mj_step(model, data)

        print(f"预热后 qpos: {np.degrees(data.qpos)}")
        mj.mj_forward(model, data)
        print(f"预热后 tool0: {data.xpos[model.body('tool0').id]}")
        print()

        # Phase 1: 持续保持零位
        print("[Phase 1] 持续保持零位 (500 steps)...")
        max_dev = 0.0
        for _ in range(500):
            set_pd_ctrl(model, data, np.zeros(6))
            mj.mj_step(model, data)
            max_dev = max(max_dev, np.max(np.abs(data.qpos)))
        print(f"  最大偏移: {np.degrees(max_dev):.4f}°")

        # Phase 2: 小幅度轨迹
        print("\n[Phase 2] 小幅度轨迹 (10%, no ramp)...")
        for step in range(250):
            t = step * DT
            target = traj_target(t, ramp=False) * 0.1
            set_pd_ctrl(model, data, target)
            mj.mj_step(model, data)
            if step % 50 == 0:
                err = np.max(np.abs(data.qpos - target))
                print(f"  t={t:.3f} j3: tgt={np.degrees(target[2]):.2f}° "
                      f"q={np.degrees(data.qpos[2]):.2f}° err={np.degrees(err):.3f}°")

        # Phase 3: 全幅度轨迹跟踪
        print("\n[Phase 3] 全幅度轨迹跟踪 (with ramp)...")
        tracking_err = []
        t_log, pos_log, ref_log = [], [], []

        for step in range(n_steps):
            t = step * DT
            target = traj_target(t)
            set_pd_ctrl(model, data, target)
            mj.mj_step(model, data)

            if step % 100 == 0:
                t_log.append(t)
                pos_log.append(data.qpos.copy())
                ref_log.append(target.copy())
                err = np.max(np.abs(data.qpos - target))
                tracking_err.append(err)
                print(f"  t={t:.2f} max_err={np.degrees(err):.2f}°")

        err_arr = np.array(tracking_err)
        pos_arr = np.array(pos_log)
        ref_arr = np.array(ref_log)
        t_arr = np.array(t_log)

        max_err_deg = np.degrees(np.max(err_arr))
        mean_err_deg = np.degrees(np.mean(err_arr))
        print(f"\n最大跟踪误差: {max_err_deg:.2f}°")
        print(f"平均跟踪误差: {mean_err_deg:.2f}°")

        jerr = np.max(np.abs(pos_arr - ref_arr), axis=0)
        for i in range(6):
            print(f"  joint_{i+1}: max err = {np.degrees(jerr[i]):.2f}°")

        threshold = 5.0
        if max_err_deg < threshold:
            print(f"\n  ✅ PASS: 轨迹跟踪误差 < {threshold}°")
        else:
            print(f"\n  ❌ FAIL: 跟踪误差过大 ({max_err_deg:.1f}° > {threshold}°)")

        try:
            import matplotlib.pyplot as plt
            fig, axes = plt.subplots(3, 1, figsize=(12, 10))
            for i in range(6):
                axes[0].plot(t_arr, np.degrees(pos_arr[:, i]),
                           label=f'j{i+1}', linewidth=1)
            axes[0].set_ylabel('Joint Angle (deg)')
            axes[0].set_title('Joint Positions (qfrc_bias Feedforward)')
            axes[0].legend(ncol=3, fontsize='small')
            axes[0].grid(True, alpha=0.3)
            axes[1].plot(t_arr, np.degrees(err_arr), 'r-', linewidth=1.5)
            axes[1].axhline(threshold, color='gray', linestyle='--',
                          label=f'{threshold}° threshold')
            axes[1].set_ylabel('Max Error (deg)')
            axes[1].set_title('Tracking Error')
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)
            axes[2].plot(np.degrees(pos_arr[:, 1]), np.degrees(pos_arr[:, 2]),
                       'b-', linewidth=1, alpha=0.7)
            axes[2].set_xlabel('Joint 2 (deg)')
            axes[2].set_ylabel('Joint 3 (deg)')
            axes[2].set_title('Phase Portrait: Joint 2 vs Joint 3')
            axes[2].grid(True, alpha=0.3)
            axes[2].set_aspect('equal')
            plt.tight_layout()
            path = os.path.join(RESOURCES, 'p3_step3_trajectory.png')
            plt.savefig(path, dpi=150)
            print(f"\n  Saved: {path}")
        except ImportError:
            pass

    else:
        print("启动 MuJoCo viewer (qfrc_bias 前馈保持零位)...\n")

        def controller(model, data):
            set_pd_ctrl(model, data, np.zeros(6))

        from mujoco import viewer
        viewer.launch(model, data, controller=controller)
        print("\nViewer 已关闭")

    print("=" * 60)
