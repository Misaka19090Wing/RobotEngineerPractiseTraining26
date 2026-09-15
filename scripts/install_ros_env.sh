#!/usr/bin/env bash
#
# install_ros_env.sh — 一键为本机安装 robotProj_ws 所需的 ROS2 运行环境
#
# 目标平台: Ubuntu 24.04 LTS (noble), 含 WSL2
# 安装内容:
#   - ROS 2 Jazzy Desktop        (rviz2 / robot_state_publisher / tf2 / xacro ...)
#   - ros_gz + Gazebo Harmonic   (gz-sim 8, ros_gz_sim, ros_gz_bridge)
#   - Navigation2 + nav2_bringup (map_server / planner / controller / bt_navigator ...)
#   - slam_toolbox               (nav_slam.launch.py)
#   - teleop_twist_keyboard      (遥控)
#   - sensor_msgs_py             (synthetic_lidar.py 使用)
#   - ros-dev-tools              (colcon / rosdep / vcstool)
#
# 用法:
#   bash scripts/install_ros_env.sh                 # 完整安装 + 编译
#   bash scripts/install_ros_env.sh --no-build      # 只装依赖，不编译
#   bash scripts/install_ros_env.sh --skip-apt      # 只配置环境 + 编译
#   ROS_APT_MIRROR=http://packages.ros.org/ros2/ubuntu \
#     bash scripts/install_ros_env.sh               # 使用官方源（默认清华 TUNA 镜像）
#
set -euo pipefail

ROS_DISTRO_NAME="jazzy"
# ROS2 apt 镜像：默认清华 TUNA（国内速度快）；可设 ROS_APT_MIRROR 覆盖为官方源
ROS_MIRROR="${ROS_APT_MIRROR:-https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu}"
ROS_KEY_URL="https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"
APT_OPTS=(-o Acquire::Retries=5 -o Acquire::Languages=none)
WITH_BUILD=1
WITH_APT=1

for arg in "$@"; do
  case "$arg" in
    --no-build) WITH_BUILD=0 ;;
    --skip-apt) WITH_APT=0 ;;
    -h|--help) sed -n '2,25p' "$0"; exit 0 ;;
    *) echo "未知参数: $arg" >&2; exit 2 ;;
  esac
done

log()  { printf '\n\033[1;32m==> %s\033[0m\n' "$*"; }
warn() { printf '\n\033[1;33m[!] %s\033[0m\n' "$*"; }
die()  { printf '\n\033[1;31m[x] %s\033[0m\n' "$*" >&2; exit 1; }

# ---- 前置检查 -------------------------------------------------------------
[[ -r /etc/os-release ]] || die "/etc/os-release 不存在，无法识别系统"
# shellcheck disable=SC1091
. /etc/os-release
CODENAME="${VERSION_CODENAME:-noble}"
[[ "$CODENAME" == "noble" ]] || warn "期望 Ubuntu noble，当前为 ${CODENAME}，继续但可能失败"
ARCH="$(dpkg --print-architecture)"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="$REPO_ROOT/robotProj_ws"

install_ros_repo() {
  log "配置 ROS 2 apt 源 (${ROS_MIRROR})"
  sudo apt-get "${APT_OPTS[@]}" update -qq
  sudo DEBIAN_FRONTEND=noninteractive apt-get "${APT_OPTS[@]}" install -y -qq \
    curl ca-certificates gnupg lsb-release

  # ROS 官方 GPG 签名密钥
  if [[ ! -s /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
    curl -fSL --retry 5 --retry-delay 3 --retry-connrefused --connect-timeout 20 \
      -o /tmp/ros-archive-keyring.gpg "$ROS_KEY_URL"
    sudo install -m 0644 /tmp/ros-archive-keyring.gpg /usr/share/keyrings/ros-archive-keyring.gpg
    echo "    已安装签名密钥"
  else
    echo "    签名密钥已存在，跳过"
  fi

  echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS_MIRROR} ${CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

  log "更新 apt 索引"
  if ! sudo apt-get "${APT_OPTS[@]}" update -qq; then
    warn "镜像 ${ROS_MIRROR} 索引更新失败，回退到官方源 packages.ros.org"
    ROS_MIRROR="http://packages.ros.org/ros2/ubuntu"
    echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS_MIRROR} ${CODENAME} main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    sudo apt-get "${APT_OPTS[@]}" update -qq
  fi
}

if [[ $WITH_APT -eq 1 ]]; then
  install_ros_repo

  # ---- ROS2 Jazzy Desktop + 开发工具 -------------------------------------
  log "安装 ros-${ROS_DISTRO_NAME}-desktop 与 ros-dev-tools (约 1GB，请耐心等待)"
  sudo DEBIAN_FRONTEND=noninteractive apt-get "${APT_OPTS[@]}" install -y \
    "ros-${ROS_DISTRO_NAME}-desktop" \
    ros-dev-tools

  # ---- 仿真 / 导航 / 遥控 依赖 -------------------------------------------
  log "安装 Gazebo Harmonic (ros_gz)、Nav2、slam_toolbox、teleop、sensor_msgs_py"
  sudo DEBIAN_FRONTEND=noninteractive apt-get "${APT_OPTS[@]}" install -y \
    "ros-${ROS_DISTRO_NAME}-ros-gz" \
    "ros-${ROS_DISTRO_NAME}-navigation2" \
    "ros-${ROS_DISTRO_NAME}-nav2-bringup" \
    "ros-${ROS_DISTRO_NAME}-slam-toolbox" \
    "ros-${ROS_DISTRO_NAME}-teleop-twist-keyboard" \
    "ros-${ROS_DISTRO_NAME}-sensor-msgs-py" \
    "ros-${ROS_DISTRO_NAME}-joint-state-publisher-gui" \
    "ros-${ROS_DISTRO_NAME}-xacro" \
    "ros-${ROS_DISTRO_NAME}-tf2-tools" \
    "ros-${ROS_DISTRO_NAME}-rviz2"
fi

# ---- shell 环境 ----------------------------------------------------------
SETUP_LINE="source /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
BASHRC="$HOME/.bashrc"
log "配置 ~/.bashrc"
touch "$BASHRC"
if ! grep -qF "$SETUP_LINE" "$BASHRC"; then
  {
    echo ""
    echo "# --- ROS 2 ${ROS_DISTRO_NAME} (added by scripts/install_ros_env.sh) ---"
    echo "$SETUP_LINE"
    echo "[ -f $WS_DIR/install/setup.bash ] && source $WS_DIR/install/setup.bash   # robotProj_ws"
    echo "# --- end ROS 2 ---"
  } >> "$BASHRC"
  echo "    已写入 $BASHRC"
else
  echo "    已存在，跳过"
fi

# ---- rosdep --------------------------------------------------------------
# rosdep 索引同样走国内镜像，raw.githubusercontent.com 直连容易超时
ROSDISTRO_MIRROR="${ROSDISTRO_INDEX_URL:-https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml}"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  log "初始化 rosdep"
  sudo rosdep init || warn "rosdep init 失败，跳过"
fi
log "更新 rosdep 索引 (${ROSDISTRO_MIRROR})"
if ! ROSDISTRO_INDEX_URL="$ROSDISTRO_MIRROR" rosdep update; then
  warn "镜像索引更新失败，回退到官方源"
  rosdep update || warn "rosdep update 失败，跳过（不影响 colcon build）"
fi

# ---- colcon build --------------------------------------------------------
if [[ $WITH_BUILD -eq 1 ]]; then
  [[ -d "$WS_DIR" ]] || die "工作空间不存在: $WS_DIR"
  log "编译工作空间 $WS_DIR"
  # shellcheck disable=SC1091
  set +u; source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"; set -u
  cd "$WS_DIR"
  colcon build --symlink-install
  log "编译完成"
fi

cat <<EOF

============================================================
 安装完成 ✅

 新开终端会自动加载环境；当前终端请手动执行：

   source /opt/ros/${ROS_DISTRO_NAME}/setup.bash
   source $WS_DIR/install/setup.bash

 启动仿真：
   ros2 launch autoVehicle gazebo.launch.py

 可选 Nav2（需另开终端，先启动上面的仿真）：
   ros2 launch autoVehicle nav_map.launch.py

 遥控：
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
============================================================
EOF
