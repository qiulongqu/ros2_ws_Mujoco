
"""
P3-Step2: MuJoCo 官方交互查看器
================================================

使用 MuJoCo 官方的 viewer.launch() 进行实时渲染和交互。

【用法】
conda activate mujoco
python3 step2_interactive_viewer.py

【鼠标操作】(官方内置)
  左键拖拽     旋转视角
  右键拖拽     平移视角
  滚轮         缩放
  双击 body    跟踪该 body
  Ctrl+右键    调整 FOV

【快捷键】(官方内置)
  Tab           切换右侧面板
  Space         暂停 / 恢复
  F1            帮助
  Esc           退出
"""

import os
import mujoco as mj

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, 'gofa_crb15000.xml')

print("=" * 60)
print("P3-Step2: GoFa CRB15000 官方交互查看器")
print("=" * 60)
print(f"模型: {MODEL_PATH}")
print()
print("鼠标: 左键旋转 | 右键平移 | 滚轮缩放 | 双击跟踪")
print("快捷键: Space 暂停 | Tab 面板 | Esc 退出")
print("=" * 60)

model = mj.MjModel.from_xml_path(MODEL_PATH)
data = mj.MjData(model)

print("\n启动官方 MuJoCo viewer...\n")

# 注意: WSL2 下 `viewer.launch_passive()` 会 segfault (GL context 在线程间传递问题)
# 使用 `viewer.launch()` 替代，功能完全相同（交互控制 + 物理仿真运行）
# `launch()` 会阻塞当前线程直到用户关闭窗口
from mujoco import viewer
viewer.launch(model, data)
