from pathlib import Path


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
EE_LINK_NAME = "link_6"

SURFACE_GRIPPER_PATHS = [
    "/World/m0609/onrobot_rg2ft/gripper_body/dual_suction_tool/"
    "suction_contact_left/SurfaceGripper_left",
    "/World/m0609/onrobot_rg2ft/gripper_body/dual_suction_tool/"
    "suction_contact_right/SurfaceGripper_right",
]


# ============================================================
# 로봇 Drive 설정
# ============================================================
DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8


# ============================================================
# RMPFlow 설정
# ============================================================
RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")

M0609_URDF_PATH = str(
    _THIS_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf"
)

M0609_DESCRIPTION_PATH = str(
    _THIS_DIR / "rmpflow/m0609_description.yaml"
)

M0609_RMPFLOW_CONFIG_PATH = str(
    _THIS_DIR / "rmpflow/m0609_rmpflow_common.yaml"
)


# ============================================================
# 제어 설정
# ============================================================
POSITION_TOLERANCE = 0.01
INITIAL_SETTLING_FRAMES = 30