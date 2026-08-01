# autoVehicle Gazebo 仿真测试日志

## 测试环境
- **ROS2**: Jazzy
- **Gazebo**: Harmonic (gz-sim8) via `ros_gz_sim`
- **物理引擎**: DART (默认) / ODE (备选)
- **控制**: teleop_twist_keyboard
- **车型**: 四轮差速转向 (skid-steer), 轮径 30mm, 轮距 148mm, 车重 0.447kg

---

## 阶段 1: 新模型导入与冲突修复

### 测试 1.1 - 第一次重新导出
- **操作**: SolidWorks 重新导出 URDF
- **结果**: 所有 joint origin 均为 `(0,0,0)` — SolidWorks 未设置坐标系
- **修复**: 使用旧版 joint origin 坐标，保留新版 mass/inertia

### 测试 1.2 - 第二次重新导出
- **操作**: 在 SolidWorks 中设定坐标系后重新导出
- **结果**: ✅ joint origin 正确，轮子位置对称 (±0.074m Y, ±0.07m X)
- **问题**: 有 git merge 冲突标记混入
- **修复**: 手动清理冲突，保留新版 mass/inertia/visual，旧版 joint 数据

---

## 阶段 2: 碰撞几何体迭代

### 测试 2.1 - STL mesh 碰撞
- **操作**: 直接使用 SolidWorks 导出的 STL 网格作为 collision
- **结果**: ❌ 前进后退正常，无法转向。左右轮子反方向转但车身不动。
- **原因**: STL 网格接触面不稳定，小尺寸下摩擦不足

### 测试 2.2 - 圆柱体碰撞 (r=0.015~0.032, l=0.012~0.02)
- **操作**: 替换为 `<cylinder>` 碰撞，多次调整半径和长度
- **结果**: ❌ 前进平滑，转向失败。圆柱体各项摩擦力不均匀——沿轴方向几乎没有摩擦力
- **特殊现象**: 配合 DART 引擎时，圆柱体接触面虽稳定但无法产生转向力矩

### 测试 2.3 - 球体碰撞 (r=0.015~0.025)
- **操作**: 替换为 `<sphere>` 碰撞
- **结果**: ❌ 前进基本正常，转向依旧失败
- **原因**: 球体接触面太小，低转速下摩擦模型不产生足够力

### 测试 2.4 - 方盒碰撞
- **操作**: 使用 `<box>` 碰撞，多次调整尺寸 (5~25mm)
- **结果**: ⚠️ 转向有改善但前进抽搐。盒子旋转时边角交替插地
- **结论**: 盒子提供各向摩擦但旋转时接触不稳定

### 测试 2.5 - 双碰撞 (圆柱 + 小方盒)
- **操作**: 每个轮子同时使用圆柱 (平滑滚动) + 小方盒 (提供转向抓地)
- **结果**: ⚠️ 转向稍有改善但仍不可靠，小方盒接触间歇性

---

## 阶段 3: 摩擦力调试

### 测试 3.1 - ODE 摩擦参数
- **操作**: 赛道地面和轮子使用 ODE `<mu>` 参数
- **结果**: ❌ ODE 引擎下同样无法转向

### 测试 3.2 - DART 摩擦参数
- **操作**: 世界文件和 URDF 中添加 DART `<primary_friction>` / `<secondary_friction>`
- **结果**: ✅ **重大突破** — 添加 DART 摩擦后转向开始有响应
- **发现**: 赛道 `course_test.world` 仅有 ODE 摩擦标签，DART 引擎默认识别不到

### 测试 3.3 - 极端摩擦系数 (mu=100)
- **操作**: 将摩擦系数从 1.0 提高到 100.0
- **结果**: 转向有改善但仍不稳定，"有时能拐，有时不能"

### 测试 3.4 - 直接速度命令
- **操作**: 使用 `gz topic` 直接给 joint 发速度命令
- **结果**: ❌ 轮子不转
- **发现**: Gazebo Harmonic 的 joint 默认不接受外部 `cmd_vel` 输入

---

## 阶段 4: 自定义控制器方案

### 测试 4.1 - Python 差速控制器 + ros_gz_bridge
- **操作**: 
  1. 移除 DiffDrive 插件
  2. 编写 `scripts/diff_drive_controller.py` 订阅 `/cmd_vel` 计算轮速
  3. 通过 `ros_gz_bridge` 将轮速桥接到 Gazebo joint topic
- **结果**: ❌ bridge 通了 (topic 有数据) 但轮子不动

### 测试 4.2 - 添加 JointController 插件
- **操作**: 为 4 个轮子 joint 添加 `gz-sim-joint-controller-system`
- **结果**: ✅ **成功** — `gz topic` 直接发送命令轮子转动，teleop 正常控制

### 测试 4.3 - teleop 操控验证
- **前进/后退**: ✅ 正常
- **左转/右转**: ✅ 正常（有小量漂移）
- **原地旋转**: ⚠️ 有位置漂移，不能完美原地转（正常现象）

---

## 最终配置

### URDF 关键参数
| 参数 | 值 |
|---|---|
| wheel collision | 球体 r=0.02m |
| wheel friction | DART primary=100, secondary=100 |
| base_link collision | box 0.165×0.14×0.0575m |
| chassis ground clearance | 3mm |
| wheel_radius | 0.015m |
| wheel_separation | 0.148m |

### 世界文件
| 文件 | 说明 |
|---|---|
| `test_flat.sdf` | 白色 200m×200m 平坦地面，含 DART 摩擦 |
| `course_test.sdf` | 管道赛道 (平直→45°坡→平直→T路口)，含 DART 摩擦 |

### 控制架构
```
/cmd_vel (teleop) → diff_drive_controller.py → ROS2 topic → ros_gz_bridge → Gazebo joint cmd_vel
                                                                                   ↓
                                                                        JointController 插件
                                                                                   ↓
                                                                             轮子转动
```

---

## 核心经验教训

1. **Gazebo Harmonic 默认 DART 引擎**，需要显式添加 `<dart>` 摩擦标签
2. **小比例模型 (轮子 15mm) 的转向摩擦力不足**，DiffDrive 插件物理方案不可靠
3. **自定义运动学控制器 + JointController 系统**是最可靠的解决方案
4. **Gazebo joint 需要 `JointController` 插件才能接受外部速度命令**
5. **球体碰撞最稳定** — 旋转不变，各向摩擦均匀，无抽搐
