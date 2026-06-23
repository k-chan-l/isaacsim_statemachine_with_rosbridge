from pathlib import Path

import numpy as np


_THIS_DIR = Path(__file__).resolve().parent


# ============================================================
# 로봇 에셋 / Prim 경로
# ============================================================

ROBOT_USD_PATH = str(
    _THIS_DIR
    / "dual_suction_surface_grippers"
    / "dual_suction_surface_grippers.usda"
)

ROBOT_PRIM_PATH = "/World/m0609"
ROBOT_SCENE_NAME = "m0609_robot"

# RMPflow가 제어하는 로봇 끝단 프레임
EE_LINK_NAME = "link_6"


# ============================================================
# Surface Gripper Prim 경로
# ============================================================

SURFACE_GRIPPER_PATHS = [
    (
        "/World/m0609/onrobot_rg2ft/gripper_body/dual_suction_tool/"
        "suction_contact_left/SurfaceGripper_left"
    ),
    (
        "/World/m0609/onrobot_rg2ft/gripper_body/dual_suction_tool/"
        "suction_contact_right/SurfaceGripper_right"
    ),
]


# ============================================================
# 로봇 Drive 설정
# ============================================================

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8


# ============================================================
# RMPflow 설정
# ============================================================

RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")

M0609_URDF_PATH = str(
    _THIS_DIR
    / "doosan-robot2"
    / "urdf"
    / "m0609_isaac_sim.urdf"
)

M0609_DESCRIPTION_PATH = str(
    _THIS_DIR
    / "rmpflow"
    / "m0609_description.yaml"
)

M0609_RMPFLOW_CONFIG_PATH = str(
    _THIS_DIR
    / "rmpflow"
    / "m0609_rmpflow_common.yaml"
)


# ============================================================
# TCP 설정
# ============================================================

# link_6 원점에서 실제 작업 TCP까지의 로컬 좌표 오프셋.
#
# 좌표 형식:
#   [x, y, z]
#
# 좌표 기준:
#   link_6의 로컬 좌표계
#
# 단위:
#   meter
#
# 아직 실제 TCP 위치를 측정하지 않았다면 0으로 두고 테스트한다.
# 이후 듀얼 흡착점 중앙까지의 실제 오프셋으로 변경해야 한다.
TCP_OFFSET_LOCAL = np.array(
    [0.0, 0.0, 0.0],
    dtype=np.float64,
)


# ============================================================
# 제어 설정
# ============================================================

# 목표 TCP와 현재 TCP 사이의 허용 거리 오차
# 단위: meter
POSITION_TOLERANCE = 0.01

# 최초 로봇 로드 후 물리 상태 안정화를 위해 진행할 프레임 수
INITIAL_SETTLING_FRAMES = 30