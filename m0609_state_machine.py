from __future__ import annotations

from enum import Enum, auto
from typing import Optional

import numpy as np

from m0609_move_controller import M0609MoveController


class RobotState(Enum):
    IDLE = auto()
    MOVE = auto()


class MoveStatus(Enum):
    IDLE = "IDLE"
    MOVING = "MOVING"
    ARRIVED = "ARRIVED"


class M0609StateMachine:
    """M0609 이동 명령의 상태 전환을 관리한다.

    역할:
    - 외부 이동 명령 수락
    - IDLE / MOVE 상태 관리
    - M0609MoveController 실행
    - 이동 상태 메시지 관리

    실제 이동 계산과 RMPflow 호출은 M0609MoveController가 담당한다.
    """

    def __init__(
        self,
        robot,
        urdf_path: str,
        robot_description_path: str,
        rmpflow_config_path: str,
        end_effector_frame_name: str = "link_6",
        tcp_offset_local: Optional[np.ndarray] = None,
        position_tolerance: float = 0.01,
    ) -> None:
        self._robot = robot

        self._move_controller = M0609MoveController(
            name="m0609_move_controller",
            robot_articulation=robot,
            urdf_path=urdf_path,
            robot_description_path=robot_description_path,
            rmpflow_config_path=rmpflow_config_path,
            end_effector_frame_name=end_effector_frame_name,
            tcp_offset_local=tcp_offset_local,
            position_tolerance=position_tolerance,
        )

        self._state = RobotState.IDLE
        self._pending_status_message: Optional[str] = MoveStatus.IDLE.value

    def reset(self) -> None:
        """시뮬레이션 reset 시 상태와 이동 컨트롤러를 초기화한다."""
        self._move_controller.reset()
        self._state = RobotState.IDLE
        self._pending_status_message = MoveStatus.IDLE.value

    def request_move(
        self,
        x: float,
        y: float,
        z: float,
    ) -> tuple[bool, str]:
        """TCP 기준 절대 목표 좌표를 등록한다."""

        if self._state != RobotState.IDLE:
            return False, "Robot is busy"

        target_position = np.asarray(
            [x, y, z],
            dtype=np.float64,
        )

        if not np.all(np.isfinite(target_position)):
            return False, "Target contains a non-finite value"

        _, current_orientation = self._robot.end_effector.get_world_pose()

        target_orientation = np.asarray(
            current_orientation,
            dtype=np.float64,
        )

        try:
            self._move_controller.set_target(
                position=target_position,
                orientation=target_orientation,
            )
        except ValueError as error:
            return False, str(error)

        self._state = RobotState.MOVE
        self._pending_status_message = MoveStatus.MOVING.value

        return True, "Move command accepted"

    def cancel(self) -> None:
        """현재 이동을 취소하고 IDLE 상태로 돌아간다."""
        self._move_controller.reset()
        self._state = RobotState.IDLE
        self._pending_status_message = MoveStatus.IDLE.value

    def step(self) -> None:
        """현재 상태를 한 시뮬레이션 프레임 진행한다."""

        if self._state == RobotState.IDLE:
            return

        if self._state == RobotState.MOVE:
            self._step_move()

    def _step_move(self) -> None:
        """M0609MoveController를 사용해 한 프레임 이동한다."""

        actions = self._move_controller.forward()
        self._robot.apply_action(actions)

        if self._move_controller.is_done():
            self._state = RobotState.IDLE
            self._pending_status_message = MoveStatus.ARRIVED.value
            return

        self._pending_status_message = MoveStatus.MOVING.value

    def consume_status_message(self) -> Optional[str]:
        """ROS로 발행할 상태 메시지를 한 번 꺼낸다."""

        message = self._pending_status_message
        self._pending_status_message = None

        return message

    @property
    def state(self) -> RobotState:
        return self._state

    @property
    def is_idle(self) -> bool:
        return self._state == RobotState.IDLE

    @property
    def is_moving(self) -> bool:
        return self._state == RobotState.MOVE

    @property
    def remaining_distance(self) -> float:
        if self._state != RobotState.MOVE:
            return 0.0

        return self._move_controller.get_position_error()