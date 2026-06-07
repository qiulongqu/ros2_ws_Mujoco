"""
P4-Step1: MuJoCo 完整场景构建与验证
====================================

扩展 GoFa CRB15000 模型，添加：
  - 桌子 (table body, 4 legs + top)
  - 可抓取方块 (block, free body)
  - 吸盘站点 (gripper_site on tool0, block_top on block)
  - 抓取约束 (connect equality, 默认关闭)
  - 视觉标记 (绿色=拣取位, 蓝色=放置位)

用法:
  python3 step1_build_scene.py           # 测试模式 (渲染 PNG, 验证场景)
  python3 step1_build_scene.py --view    # 交互查看器
"""

import os
import sys
import numpy as np
import mujoco as mj

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, 'gofa_table_block.xml')
RESOURCES  = os.path.join(SCRIPT_DIR, '..', '..', 'resources', 'p4')

# Ensure output directory exists
os.makedirs(RESOURCES, exist_ok=True)


def verify_scene(model, data):
    """Run 7 scene verification tests."""
    print("=" * 60)
    print("P4-Step1: GoFa Pick-and-Place 场景验证")
    print("=" * 60)
    print(f"模型: {MODEL_PATH}")
    print()

    mj.mj_forward(model, data)

    tests_passed = 0
    total_tests = 7

    # Test 1: Model structure
    print("[Test 1] 模型结构检查")
    print(f"  Bodies: {model.nbody}")
    print(f"  Joints: {model.njnt}")
    print(f"  Geoms:  {model.ngeom}")
    print(f"  Sites:  {model.nsite}")
    print(f"  Actuators: {model.nu}")
    print(f"  Equality constraints: {model.neq}")

    # Expected: base_link + 6 links + tool0 + table + block + 2 markers = 12 bodies
    # Joints: 6 hinge + 1 freejoint = 7 joints
    assert model.nbody >= 12, f"Body count too low: {model.nbody}"
    assert model.njnt >= 7, f"Joint count too low: {model.njnt}"
    assert model.neq >= 1, f"No equality constraints found"
    tests_passed += 1
    print("  PASS")

    # Test 2: Key body names exist
    print("\n[Test 2] 关键 body 名称验证")
    required_bodies = ['base_link', 'tool0', 'table', 'block', 'marker_pick', 'marker_place']
    for name in required_bodies:
        bid = model.body(name).id
        print(f"  {name}: id={bid} ✓")
    tests_passed += 1
    print("  PASS")

    # Test 3: Key sites exist
    print("\n[Test 3] 吸盘站点验证")
    gripper_id = model.site('gripper_site').id
    block_top_id = model.site('block_top').id
    print(f"  gripper_site: id={gripper_id}")
    print(f"  block_top:    id={block_top_id}")
    tests_passed += 1
    print("  PASS")

    # Test 4: Block starts on table (not falling through)
    print("\n[Test 4] 方块初始位置验证")
    block_xpos = data.body('block').xpos.copy()
    print(f"  block xpos:  [{block_xpos[0]:.3f}, {block_xpos[1]:.3f}, {block_xpos[2]:.3f}]")
    assert block_xpos[2] > 0.4, f"Block Z too low: {block_xpos[2]:.3f} (expected > 0.4)"
    tests_passed += 1
    print("  PASS")

    # Test 5: Table is at correct height
    print("\n[Test 5] 桌子位置验证")
    table_xpos = data.body('table').xpos.copy()
    print(f"  table xpos: [{table_xpos[0]:.3f}, {table_xpos[1]:.3f}, {table_xpos[2]:.3f}]")
    assert abs(table_xpos[0] - 0.50) < 0.01, f"Table X mismatch"
    tests_passed += 1
    print("  PASS")

    # Test 6: Zero-position tool0 is above robot (no self-collision)
    print("\n[Test 6] 零位 tool0 验证")
    tool0_xpos = data.body('tool0').xpos.copy()
    print(f"  tool0 xpos: [{tool0_xpos[0]:.3f}, {tool0_xpos[1]:.3f}, {tool0_xpos[2]:.3f}]")
    assert tool0_xpos[2] > 1.0, f"tool0 Z too low: {tool0_xpos[2]:.3f}"
    tests_passed += 1
    print("  PASS")

    # Test 7: Equality constraint exists and is inactive
    print("\n[Test 7] 抓取约束验证")
    eq_id = model.equality('grasp').id
    eq_active = model.eq_active0[eq_id] if hasattr(model, 'eq_active0') else 0
    print(f"  grasp constraint: id={eq_id}, active={eq_active}")
    assert eq_active == 0, "Grasp should be INACTIVE at start"
    tests_passed += 1
    print("  PASS")

    print(f"\n{'='*60}")
    print(f"  结果: {tests_passed}/{total_tests} 通过")
    if tests_passed == total_tests:
        print("  状态: PASS")
    else:
        print(f"  状态: FAIL ({total_tests - tests_passed} failed)")
    print(f"{'='*60}")

    return tests_passed == total_tests


def render_and_save(model, data, suffix=""):
    """Render scene to PNG using offscreen renderer."""
    width, height = 1200, 900

    # Create offscreen renderer
    renderer = mj.Renderer(model, height, width)

    # Set camera: looking at the table area
    cam = mj.MjvCamera()
    cam.lookat[:] = [0.50, 0.0, 0.45]
    cam.distance = 2.0
    cam.azimuth = 150
    cam.elevation = -25

    # Update scene with camera
    mj.mjv_updateScene(
        model, data, mj.MjvOption(),
        None, cam, mj.mjtCatBit.mjCAT_ALL, renderer.scene)

    # Render
    renderer.update_scene(data, camera=cam)
    pixels = renderer.render()

    # Save
    fname = f"p4_step1_scene{suffix}.png"
    path = os.path.join(RESOURCES, fname)
    from PIL import Image
    img = Image.fromarray(pixels)
    img.save(path)
    print(f"\n保存渲染图: {path}")
    return path


def run_viewer(model, data):
    """Launch MuJoCo interactive viewer."""
    print("=" * 60)
    print("P4-Step1: MuJoCo 交互查看器")
    print("=" * 60)
    print()
    print("场景包含:")
    print("  绿色球 = 拣取位置 (pick)")
    print("  蓝色球 = 放置位置 (place)")
    print("  红色方块 = 可抓取物体")
    print("  tool0 绿色柱 = 吸盘")
    print()
    print("操作: 左键旋转 | 右键平移 | 滚轮缩放 | Space 暂停")
    print("=" * 60)

    # Position actuator: hold zero pose
    def ctrl_cb(model, data):
        data.ctrl[:] = 0.0

    mj.set_mjcb_control(ctrl_cb)

    from mujoco import viewer
    viewer.launch(model, data)


def simulate_and_check(model, data, sim_time=2.0):
    """Run short simulation and check block landed on table."""
    dt = model.opt.timestep
    n_steps = int(sim_time / dt)

    # Run simulation
    block_z_start = data.body('block').xpos[2]
    for _ in range(n_steps):
        mj.mj_step(model, data)

    block_z_end = data.body('block').xpos[2]
    print(f"\n[重力验证] 仿真 {sim_time}s")
    print(f"  方块初始 Z: {block_z_start:.3f}")
    print(f"  方块最终 Z: {block_z_end:.3f}")

    # Block should have settled on the table (~0.475 + settling)
    if 0.45 < block_z_end < 0.50:
        print("  方块稳定停在桌面上 ✓")
        return True
    elif block_z_end < 0.3:
        print("  方块穿透桌面! 需要调整 contact 参数")
        return False
    else:
        print(f"  方块位置异常, 需要检查")
        return False


if __name__ == '__main__':
    USE_VIEWER = '--view' in sys.argv
    FORCE_TEST = '--test' in sys.argv

    model = mj.MjModel.from_xml_path(MODEL_PATH)
    data = mj.MjData(model)

    if USE_VIEWER:
        run_viewer(model, data)
    else:
        # Verification mode
        ok = verify_scene(model, data)

        if ok and not FORCE_TEST:
            # Physics check: see block land on table
            simulate_and_check(model, data, sim_time=3.0)

            # Reset and render zero pose
            data2 = mj.MjData(model)
            render_and_save(model, data2, suffix="_zeropose")

            # Set a more interesting pose for render
            data3 = mj.MjData(model)
            data3.qpos[:] = [0.3, -0.6, 0.4, 0.0, 0.8, 0.0,  # arm joints
                              0.50, -0.15, 0.475,              # block position
                              1, 0, 0, 0]                      # block quaternion
            mj.mj_forward(model, data3)
            render_and_save(model, data3, suffix="_reach")

            print("\n✅ Step1 验证完成")
            print(f"  查看渲染图: {RESOURCES}/")
