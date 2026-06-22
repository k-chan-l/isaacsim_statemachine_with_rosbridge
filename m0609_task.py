from pathlib import Path

import omni.kit.app
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api.tasks import BaseTask
from isaacsim.robot.manipulators.manipulators import SingleManipulator

from dual_surface_gripper_adapter import DualSurfaceGripperAdapter


_THIS_DIR = Path(__file__).resolve().parent


# ============================================================
# A. 로봇 에셋 / Prim 경로
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

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8


# ============================================================
# B. RMPFlow 설정
#    상태머신을 생성할 때 main에서 사용한다.
# ============================================================
M0609_URDF_PATH = str(
    _THIS_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf"
)
M0609_DESCRIPTION_PATH = str(
    _THIS_DIR / "rmpflow/m0609_description.yaml"
)
M0609_RMPFLOW_CONFIG_PATH = str(
    _THIS_DIR / "rmpflow/m0609_rmpflow_common.yaml"
)


def find_prim_path_by_name(root_path: str, name: str):
    """root_path 아래에서 이름이 일치하는 Prim 경로를 찾는다."""
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)

    if not root_prim.IsValid():
        return None

    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return str(prim.GetPath())

    return None


def initialize_robot(robot, world):
    print("1. robot.initialize 시작")
    robot.initialize()
    print("2. robot.initialize 완료")

    print("3. gripper.initialize 시작")
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    print("4. gripper.initialize 완료")

    robot.gripper.open()


class M0609BasicTask(BaseTask):
    """로봇 USD 로딩, 물리 설정, Scene 등록을 담당하는 Task."""

    def __init__(self, name: str = "m0609_basic_task"):
        super().__init__(name=name, offset=None)
        self._robot = None
        self._ee_path = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)

        self._load_robot_usd()
        self._discover_links()
        self._setup_physics()
        self._register_robot(scene)

        # 트레이 등 작업 대상은 나중에 여기서 추가한다.
        # self._create_tray(scene)

        print("\n[완료] M0609 기본 씬 구성 완료\n")

    def _load_robot_usd(self):
        stage = omni.usd.get_context().get_stage()

        if not stage.GetPrimAtPath("/World").IsValid():
            UsdGeom.Xform.Define(stage, "/World")

        stage.GetPrimAtPath("/World").GetReferences().AddReference(
            ROBOT_USD_PATH
        )

        # USD reference가 Stage에 반영될 시간을 준다.
        app = omni.kit.app.get_app()
        for _ in range(15):
            app.update()

    def _discover_links(self):
        self._ee_path = find_prim_path_by_name(
            ROBOT_PRIM_PATH,
            EE_LINK_NAME,
        )

        if self._ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}'을 찾지 못했습니다.")

        stage = omni.usd.get_context().get_stage()

        for path in SURFACE_GRIPPER_PATHS:
            if not stage.GetPrimAtPath(path).IsValid():
                raise RuntimeError(
                    f"SurfaceGripper를 찾지 못했습니다: {path}"
                )

    def _setup_physics(self):
        stage = omni.usd.get_context().get_stage()
        robot_root = stage.GetPrimAtPath(ROBOT_PRIM_PATH)

        if not robot_root.IsValid():
            raise RuntimeError(
                f"로봇 Prim을 찾지 못했습니다: {ROBOT_PRIM_PATH}"
            )

        for prim in Usd.PrimRange(robot_root):
            for drive_type in ("angular", "linear"):
                drive = UsdPhysics.DriveAPI.Get(prim, drive_type)

                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)

    def _register_robot(self, scene):
        gripper = DualSurfaceGripperAdapter(
            end_effector_prim_path=self._ee_path,
            surface_gripper_prim_paths=SURFACE_GRIPPER_PATHS,
            write_status_to_usd=True,
        )

        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM_PATH,
                name=ROBOT_SCENE_NAME,
                end_effector_prim_path=self._ee_path,
                gripper=gripper,
            )
        )

    def post_reset(self):
        """Task reset 시 그리퍼 내부 상태만 초기화한다."""
        if self._robot is not None:
            self._robot.gripper.post_reset()