# m0609_ros_bridge.py
# ROS2 String(JSON) 명령을 받아 M0609StateMachine.request_move()를 호출한다.

import omni.graph.core as og
import omni.usd

from isaacsim.core.utils.extensions import enable_extension


GRAPH_PATH = "/World/ROS_M0609_String_Graph"

COMMAND_TOPIC = "/m0609/move_command"
RESULT_TOPIC = "/m0609/move_result"

STRING_TYPE = "token"

_STATE_MACHINE = None


def get_state_machine():
    """OmniGraph ScriptNode에서 상태 머신 객체를 가져온다."""
    return _STATE_MACHINE


_COMMAND_SCRIPT = r"""
import json

from m0609_ros_bridge import get_state_machine


def compute(db):
    request_id = ""

    try:
        command = json.loads(str(db.inputs.command))

        request_id = str(command.get("request_id", ""))
        x = float(command["x"])
        y = float(command["y"])
        z = float(command["z"])

        state_machine = get_state_machine()

        if state_machine is None:
            raise RuntimeError("State machine is not registered")

        # 상태 머신이 IDLE/MOVE 상태를 검사하고
        # 명령 수락 여부를 결정한다.
        accepted, message = state_machine.request_move(x, y, z)

        result = {
            "request_id": request_id,
            "accepted": bool(accepted),
            "message": str(message),
        }

        print(
            "[ROS2 Bridge] move request:"
            f" request_id={request_id or '(none)'},"
            f" target=({x:.4f}, {y:.4f}, {z:.4f}),"
            f" accepted={accepted},"
            f" message={message}",
            flush=True,
        )

    except Exception as error:
        result = {
            "request_id": request_id,
            "accepted": False,
            "message": str(error),
        }

        print(
            f"[ROS2 Bridge] command error: {error}",
            flush=True,
        )

    db.outputs.result = json.dumps(
        result,
        ensure_ascii=False,
    )

    return True
"""


def _remove_existing_graph(simulation_app):
    stage = omni.usd.get_context().get_stage()
    graph_prim = stage.GetPrimAtPath(GRAPH_PATH)

    if not graph_prim.IsValid():
        return

    stage.RemovePrim(GRAPH_PATH)

    for _ in range(3):
        simulation_app.update()


def _create_script_ports():
    node = og.Controller.node(
        f"{GRAPH_PATH}/HandleMoveCommand"
    )

    og.Controller.create_attribute(
        node=node,
        attr_name="inputs:command",
        attr_type=STRING_TYPE,
    )
    og.Controller.create_attribute(
        node=node,
        attr_name="outputs:result",
        attr_type=STRING_TYPE,
        attr_port=og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT,
    )


def setup_m0609_ros_bridge(state_machine, simulation_app):
    """ROS Bridge를 만들고 사용할 상태 머신 객체를 등록한다."""
    global _STATE_MACHINE
    _STATE_MACHINE = state_machine

    enable_extension("isaacsim.ros2.bridge")
    enable_extension("omni.graph.scriptnode")

    for _ in range(3):
        simulation_app.update()

    _remove_existing_graph(simulation_app)

    graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": GRAPH_PATH,
            "evaluator_name": "execution",
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                (
                    "OnPlaybackTick",
                    "omni.graph.action.OnPlaybackTick",
                ),
                (
                    "ROS2Context",
                    "isaacsim.ros2.bridge.ROS2Context",
                ),
                (
                    "CommandSubscriber",
                    "isaacsim.ros2.bridge.ROS2Subscriber",
                ),
                (
                    "HandleMoveCommand",
                    "omni.graph.scriptnode.ScriptNode",
                ),
                (
                    "ResultPublisher",
                    "isaacsim.ros2.bridge.ROS2Publisher",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                (
                    "CommandSubscriber.inputs:messagePackage",
                    "std_msgs",
                ),
                (
                    "CommandSubscriber.inputs:messageSubfolder",
                    "msg",
                ),
                (
                    "CommandSubscriber.inputs:messageName",
                    "String",
                ),
                (
                    "CommandSubscriber.inputs:topicName",
                    COMMAND_TOPIC,
                ),
                (
                    "ResultPublisher.inputs:messagePackage",
                    "std_msgs",
                ),
                (
                    "ResultPublisher.inputs:messageSubfolder",
                    "msg",
                ),
                (
                    "ResultPublisher.inputs:messageName",
                    "String",
                ),
                (
                    "ResultPublisher.inputs:topicName",
                    RESULT_TOPIC,
                ),
                (
                    "HandleMoveCommand.inputs:script",
                    _COMMAND_SCRIPT,
                ),
            ],
            og.Controller.Keys.CONNECT: [
                (
                    "OnPlaybackTick.outputs:tick",
                    "CommandSubscriber.inputs:execIn",
                ),
                (
                    "ROS2Context.outputs:context",
                    "CommandSubscriber.inputs:context",
                ),
                (
                    "ROS2Context.outputs:context",
                    "ResultPublisher.inputs:context",
                ),
            ],
        },
    )

    # std_msgs/msg/String의 data 포트가 생성될 시간을 준다.
    for _ in range(10):
        simulation_app.update()

    _create_script_ports()

    for _ in range(3):
        simulation_app.update()

    def path(node, port):
        return f"{GRAPH_PATH}/{node}.{port}"

    og.Controller.edit(
        graph,
        {
            og.Controller.Keys.CONNECT: [
                (
                    path("CommandSubscriber", "outputs:execOut"),
                    path("HandleMoveCommand", "inputs:execIn"),
                ),
                (
                    path("CommandSubscriber", "outputs:data"),
                    path("HandleMoveCommand", "inputs:command"),
                ),
                (
                    path("HandleMoveCommand", "outputs:execOut"),
                    path("ResultPublisher", "inputs:execIn"),
                ),
                (
                    path("HandleMoveCommand", "outputs:result"),
                    path("ResultPublisher", "inputs:data"),
                ),
            ],
        },
    )

    for _ in range(3):
        simulation_app.update()

    print(
        "[ROS2 Bridge] ready:"
        f" {COMMAND_TOPIC}"
        " -> state_machine.request_move()"
        f" -> {RESULT_TOPIC}",
        flush=True,
    )