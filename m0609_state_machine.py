from enum import Enum, auto

import numpy as np

from m0609_rmpflow_controller import RMPFlowController


class RobotState(Enum):
    IDLE = auto()
    MOVE = auto()


class MoveStatus(Enum):
    IDLE = "IDLE"
    MOVING = "MOVING"
    ARRIVED = "ARRIVED"


class M0609StateMachine:
    """외부 서비스 요청으로 목표를 받고 이동하는 M0609 상태머신.

    제어 상태:
        IDLE -- request_move() --> MOVE
        MOVE -- 목표 도달 ------> IDLE

    ROS Bridge 연결용 인터페이스:
        request_move(x, y, z)
            서비스 요청에서 호출한다.

        consume_status_message()
            상태 토픽을 발행하기 전에 호출한다.
            반환값이 None이 아니면 해당 문자열을 1회 발행한다.

    이 클래스는 rclpy를 직접 사용하지 않는다.
    """

    def __init__(
        self,
        robot,
        urdf_path: str,
        robot_description_path: str,
        rmpflow_config_path: str,
        end_effector_frame_name: str = "link_6",
        position_tolerance: float = 0.01,
    ):
        self._robot = robot
        self._position_tolerance = float(position_tolerance)

        self._move_controller = RMPFlowController(
            name="m0609_rmpflow",
            robot_articulation=robot,
            urdf_path=urdf_path,
            robot_description_path=robot_description_path,
            rmpflow_config_path=rmpflow_config_path,
            end_effector_frame_name=end_effector_frame_name,
        )

        self._state = RobotState.IDLE
        self._target_position = None
        self._target_orientation = None
        self._remaining_distance = 0.0

        # ROS 상태 토픽으로 내보낼 1회성 메시지.
        # consume_status_message()가 읽으면 None으로 비워진다.
        self._pending_status_message = MoveStatus.IDLE.value

    def reset(self):
        """시뮬레이션 reset 시 목표와 제어 상태를 초기화한다."""
        self._move_controller.reset()
        self._clear_target()
        self._pending_status_message = MoveStatus.IDLE.value

    def request_move(self, x: float, y: float, z: float):
        """외부 서비스 요청으로 절대 목표 좌표를 등록한다.

        Returns:
            (accepted, message)
        """
        if self._state != RobotState.IDLE:
            return False, "Robot is busy"

        position = np.asarray([x, y, z], dtype=np.float64)

        if not np.all(np.isfinite(position)):
            return False, "Target contains a non-finite value"

        # 좌표만 전달받으므로 요청 시점의 현재 방향을 유지한다.
        _, current_orientation = (
            self._robot.end_effector.get_world_pose()
        )

        self._target_position = position
        self._target_orientation = np.asarray(
            current_orientation,
            dtype=np.float64,
        )

        self._remaining_distance = self._calculate_position_error()
        self._state = RobotState.MOVE
        self._pending_status_message = MoveStatus.MOVING.value

        return True, "Move command accepted"

    def cancel(self):
        """현재 이동을 취소하고 IDLE로 돌아간다."""
        self._clear_target()
        self._pending_status_message = MoveStatus.IDLE.value

    def step(self):
        """현재 로봇 상태를 한 시뮬레이션 프레임 진행한다."""
        if self._state == RobotState.IDLE:
            return

        actions = self._move_controller.forward(
            target_end_effector_position=self._target_position,
            target_end_effector_orientation=self._target_orientation,
        )
        self._robot.apply_action(actions)

        self._remaining_distance = self._calculate_position_error()

        if self._remaining_distance <= self._position_tolerance:
            # ARRIVED를 먼저 발행 대기 상태로 저장한다.
            # 이후 목표를 지우더라도 이 값은 유지된다.
            self._pending_status_message = MoveStatus.ARRIVED.value
            self._clear_target()
            return

        # 이동하는 동안 매 step마다 MOVING 발행을 요청한다.
        self._pending_status_message = MoveStatus.MOVING.value

    def consume_status_message(self):
        """ROS 상태 토픽으로 발행할 문자열을 한 번 꺼낸다.

        Returns:
            "IDLE", "MOVING", "ARRIVED" 또는 None
        """
        message = self._pending_status_message
        self._pending_status_message = None
        return message

    def _calculate_position_error(self):
        current_position, _ = (
            self._robot.end_effector.get_world_pose()
        )

        return float(
            np.linalg.norm(
                np.asarray(current_position, dtype=np.float64)
                - self._target_position
            )
        )

    def _clear_target(self):
        self._target_position = None
        self._target_orientation = None
        self._remaining_distance = 0.0
        self._state = RobotState.IDLE

    @property
    def state(self):
        return self._state

    @property
    def is_idle(self):
        return self._state == RobotState.IDLE

    @property
    def is_moving(self):
        return self._state == RobotState.MOVE

    @property
    def remaining_distance(self):
        return self._remaining_distance

    @property
    def target_position(self):
        if self._target_position is None:
            return None
        return self._target_position.copy()