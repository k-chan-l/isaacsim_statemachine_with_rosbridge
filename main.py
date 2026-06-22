from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})


# Isaac Sim 전용 모듈은 SimulationApp 생성 후 import한다.
from isaacsim.core.utils.extensions import enable_extension


# ============================================================
# A. Isaac Sim 확장 활성화
# ============================================================
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.robot.surface_gripper")
enable_extension("omni.graph.scriptnode")

# ROS2 Bridge OmniGraph 노드 타입 등록 대기
for _ in range(10):
    simulation_app.update()


# ============================================================
# B. 일반 import
# ============================================================
import sys
import time

from isaacsim.core.api import World

print("[main] config import 시작")

from m0609_config import (
    EE_LINK_NAME,
    INITIAL_SETTLING_FRAMES,
    M0609_DESCRIPTION_PATH,
    M0609_RMPFLOW_CONFIG_PATH,
    M0609_URDF_PATH,
    POSITION_TOLERANCE,
    RMPFLOW_DIR,
    ROBOT_SCENE_NAME,
)

print("[main] config import 완료")


if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)


print("[main] ros bridge import 시작")
from m0609_ros_bridge import setup_m0609_ros_bridge
print("[main] ros bridge import 완료")

print("[main] state machine import 시작")
from m0609_state_machine import M0609StateMachine
print("[main] state machine import 완료")

print("[main] task import 시작")
from m0609_task import M0609BasicTask, initialize_robot
print("[main] task import 완료")


def main():
    print("[main] main() 진입")

    # ========================================================
    # 1. World와 Task 생성
    # ========================================================
    print("[main] World 생성")
    world = World(stage_units_in_meters=1.0)

    print("[main] Task 등록")
    task = M0609BasicTask()
    world.add_task(task)

    print("[main] 최초 world.reset() 시작")
    world.reset()
    print("[main] 최초 world.reset() 완료")

    # ========================================================
    # 2. 로봇 객체 가져오기 및 초기화
    # ========================================================
    robot = world.scene.get_object(ROBOT_SCENE_NAME)

    if robot is None:
        raise RuntimeError(
            f"Scene에서 로봇을 찾지 못했습니다: {ROBOT_SCENE_NAME}"
        )

    print("[main] initialize_robot() 시작")
    initialize_robot(robot, world)
    print("[main] initialize_robot() 완료")

    # ========================================================
    # 3. 로봇 작업 상태머신 생성
    # ========================================================
    print("[main] 상태머신 생성 시작")
    state_machine = M0609StateMachine(
        robot=robot,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
        position_tolerance=POSITION_TOLERANCE,
    )
    print("[main] 상태머신 생성 완료")

    # ========================================================
    # 4. ROS2 Bridge 서비스/토픽 그래프 생성
    # ========================================================
    print("[main] ROS2 Bridge 그래프 생성 시작")
    setup_m0609_ros_bridge(
        state_machine=state_machine,
        simulation_app=simulation_app,
    )
    print("[main] ROS2 Bridge 그래프 생성 완료")

    # ========================================================
    # 5. 최초 물리 상태 안정화
    # ========================================================
    print("[main] 물리 상태 안정화 시작")
    for _ in range(INITIAL_SETTLING_FRAMES):
        world.step(render=True)
    print("[main] 물리 상태 안정화 완료")

    print("\n[M0609 실행 준비 완료]")
    print("- 서비스: /m0609/move_to")
    print("- 상태 토픽: /m0609/move_status")
    print("- Play 시작 후 외부 서비스 요청을 기다립니다.\n")

    # ========================================================
    # 6. 메인 시뮬레이션 루프
    # ========================================================
    was_playing = False

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.01)

        is_playing = world.is_playing()

        if is_playing and not was_playing:
            print("[main] Play 시작 감지")

            print("[main] Play world.reset() 시작")
            world.reset()
            print("[main] Play world.reset() 완료")

            state_machine.reset()
            print("[main] 상태머신 reset 완료")

        if is_playing:
            state_machine.step()

        was_playing = is_playing

    # 정상적으로 창이 닫힌 경우에만 실행
    print("[main] SimulationApp 종료")
    simulation_app.close()


if __name__ == "__main__":
    main()