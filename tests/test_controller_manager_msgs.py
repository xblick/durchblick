"""The controller manager's and the torque step's messages (msg.py) against rosbags, an independent ROS 2
implementation: controller_manager_msgs as ros2_control Jazzy defines them (the definitions below are its .msg and .srv
files, fetched from github.com/ros-controls/ros2_control, branch jazzy, on 2026-11-09), Durchblick's durchblick_msgs as
protocol.hpp describes them; and the C++ messages (cpp/tests/test_messages.cpp) against the same bytes."""

import re
from pathlib import Path

import pytest

from blick.msg import (
    ChainConnection,
    ControllerState,
    CreateControllerRequest,
    CreateControllerResponse,
    Duration,
    GetConfigFileRequest,
    GetConfigFileResponse,
    GetControllerConfigRequest,
    GetControllerConfigResponse,
    ListControllersRequest,
    ListControllersResponse,
    SwitchControllerRequest,
    SwitchControllerResponse,
    TorqueStepRequest,
    TorqueStepResponse,
    UnloadControllerRequest,
    UnloadControllerResponse,
)

typesys = pytest.importorskip("rosbags.typesys")

DEFINITIONS = {
    "controller_manager_msgs/msg/ChainConnection": "string name\nstring[] reference_interfaces\n",
    "controller_manager_msgs/msg/ControllerState": """string name
string state
string type
bool is_async
uint16 update_rate
string[] claimed_interfaces
string[] required_command_interfaces
string[] required_state_interfaces
bool is_chainable
bool is_chained
string[] exported_state_interfaces
string[] reference_interfaces
controller_manager_msgs/ChainConnection[] chain_connections
""",
    # empty in ListControllers.srv; rosidl gives an empty message this one member, which ROS 2 serializes
    "controller_manager_msgs/srv/ListControllers_Request": "uint8 structure_needs_at_least_one_member\n",
    "controller_manager_msgs/srv/ListControllers_Response": "controller_manager_msgs/ControllerState[] controller\n",
    "controller_manager_msgs/srv/SwitchController_Request": """string[] activate_controllers
string[] deactivate_controllers
int32 strictness
int32 BEST_EFFORT=1
int32 STRICT=2
int32 AUTO=3
int32 FORCE_AUTO=4
bool activate_asap
builtin_interfaces/Duration timeout
""",
    "controller_manager_msgs/srv/SwitchController_Response": "bool ok\nstring message\n",
    "controller_manager_msgs/srv/UnloadController_Request": "string name\n",
    "controller_manager_msgs/srv/UnloadController_Response": "bool ok\n",
    "durchblick_msgs/srv/TorqueStep_Request": "string joint\nfloat64 torque\nfloat64 duration\n",
    "durchblick_msgs/srv/TorqueStep_Response": "bool ok\nstring message\n",
    "durchblick_msgs/srv/CreateController_Request": "string name\nstring type\nstring config\n",
    "durchblick_msgs/srv/CreateController_Response": "bool ok\nstring message\n",
    "durchblick_msgs/srv/GetControllerConfig_Request": "string name\n",
    "durchblick_msgs/srv/GetControllerConfig_Response": "bool ok\nstring message\nstring config\n",
    "durchblick_msgs/srv/GetConfigFile_Request": "string type\nstring name\n",
    "durchblick_msgs/srv/GetConfigFile_Response": "bool ok\nstring message\nstring config\n",
}


def ros_name(name: str) -> str:
    """rosbags names a type from a .msg text <package>/msg/<name>; a service's parts are registered so (the name
    only finds the type, the bytes do not depend on it)."""
    return name.replace("/srv/", "/msg/")


class Types:
    def __init__(self, store):
        self.store = store

    def __getitem__(self, name):
        return self.store.types[ros_name(name)]


@pytest.fixture(scope="module")
def store():
    store = typesys.get_typestore(typesys.Stores.ROS2_JAZZY)
    types = {}
    for name, text in DEFINITIONS.items():
        types.update(typesys.get_types_from_msg(text, ros_name(name)))
    store.register(types)
    return store


def state(name="arms", active=True):
    return ControllerState(name=name, state="active" if active else "inactive", type="Impedance", update_rate=1000,
                           claimed_interfaces=["left_shoulder_1/torque"] if active else [],
                           required_command_interfaces=["left_shoulder_1/torque", "left_shoulder_3/torque"],
                           chain_connections=[ChainConnection("upstream", ["a/b"])])


def ros_state(store, s: ControllerState):
    T = Types(store)
    return T["controller_manager_msgs/msg/ControllerState"](
        name=s.name, state=s.state, type=s.type, is_async=s.is_async, update_rate=s.update_rate,
        claimed_interfaces=s.claimed_interfaces, required_command_interfaces=s.required_command_interfaces,
        required_state_interfaces=s.required_state_interfaces, is_chainable=s.is_chainable, is_chained=s.is_chained,
        exported_state_interfaces=s.exported_state_interfaces, reference_interfaces=s.reference_interfaces,
        chain_connections=[T["controller_manager_msgs/msg/ChainConnection"](name=c.name,
                                                                            reference_interfaces=c.reference_interfaces)
                           for c in s.chain_connections])


def cases(store):
    """(ours, rosbags' message, its type name)"""
    T = Types(store)
    s1, s2 = state("arms"), state("arms_test", active=False)
    switch = SwitchControllerRequest(["arms"], ["arms_test", "x"], SwitchControllerRequest.STRICT, True,
                                     Duration(2, 500_000_000))
    return [
        (ListControllersRequest(), T["controller_manager_msgs/srv/ListControllers_Request"](),
         "controller_manager_msgs/srv/ListControllers_Request"),
        (ListControllersResponse([s1, s2]), T["controller_manager_msgs/srv/ListControllers_Response"](
            controller=[ros_state(store, s1), ros_state(store, s2)]), "controller_manager_msgs/srv/ListControllers_Response"),
        (switch, T["controller_manager_msgs/srv/SwitchController_Request"](
            activate_controllers=["arms"], deactivate_controllers=["arms_test", "x"], strictness=2, activate_asap=True,
            timeout=T["builtin_interfaces/msg/Duration"](sec=2, nanosec=500_000_000)),
         "controller_manager_msgs/srv/SwitchController_Request"),
        (SwitchControllerResponse(False, "no controller x"), T["controller_manager_msgs/srv/SwitchController_Response"](
            ok=False, message="no controller x"), "controller_manager_msgs/srv/SwitchController_Response"),
        (UnloadControllerRequest("arms"), T["controller_manager_msgs/srv/UnloadController_Request"](name="arms"),
         "controller_manager_msgs/srv/UnloadController_Request"),
        (UnloadControllerResponse(True), T["controller_manager_msgs/srv/UnloadController_Response"](ok=True),
         "controller_manager_msgs/srv/UnloadController_Response"),
        (TorqueStepRequest("left_elbow", 1.5, 0.25), T["durchblick_msgs/srv/TorqueStep_Request"](
            joint="left_elbow", torque=1.5, duration=0.25), "durchblick_msgs/srv/TorqueStep_Request"),
        (TorqueStepResponse(False, "disabled"), T["durchblick_msgs/srv/TorqueStep_Response"](ok=False, message="disabled"),
         "durchblick_msgs/srv/TorqueStep_Response"),
        (CreateControllerRequest("arms", "Impedance", '{"a": 1.5}'), T["durchblick_msgs/srv/CreateController_Request"](
            name="arms", type="Impedance", config='{"a": 1.5}'), "durchblick_msgs/srv/CreateController_Request"),
        (CreateControllerResponse(True, ""), T["durchblick_msgs/srv/CreateController_Response"](ok=True, message=""),
         "durchblick_msgs/srv/CreateController_Response"),
        (GetControllerConfigRequest("arms"), T["durchblick_msgs/srv/GetControllerConfig_Request"](name="arms"),
         "durchblick_msgs/srv/GetControllerConfig_Request"),
        (GetControllerConfigResponse(True, "", "{}"), T["durchblick_msgs/srv/GetControllerConfig_Response"](
            ok=True, message="", config="{}"), "durchblick_msgs/srv/GetControllerConfig_Response"),
        (GetConfigFileRequest("Impedance", "robot"), T["durchblick_msgs/srv/GetConfigFile_Request"](
            type="Impedance", name="robot"), "durchblick_msgs/srv/GetConfigFile_Request"),
        (GetConfigFileResponse(False, "no file", ""), T["durchblick_msgs/srv/GetConfigFile_Response"](
            ok=False, message="no file", config=""), "durchblick_msgs/srv/GetConfigFile_Response"),
    ]


def test_messages_match_rosbags(store):
    for ours, theirs, type_name in cases(store):
        ros = store.serialize_cdr(theirs, ros_name(type_name))
        assert bytes(ours.serialize()) == bytes(ros), type_name
        assert type(ours).deserialize(ros) == ours, type_name


def test_types_are_named_as_in_protocol():
    for ours, _, type_name in cases_types():
        assert type(ours).TYPE == type_name


def cases_types():
    return [(ListControllersRequest(), None, "controller_manager_msgs/srv/ListControllers_Request"),
            (SwitchControllerRequest(), None, "controller_manager_msgs/srv/SwitchController_Request"),
            (TorqueStepRequest(), None, "durchblick_msgs/srv/TorqueStep_Request"),
            (CreateControllerRequest(), None, "durchblick_msgs/srv/CreateController_Request")]


def test_durchblick_types_are_versioned():
    assert CreateControllerRequest.SCHEMA == "durchblick_msgs/srv/CreateController_Request@1"
    assert TorqueStepRequest.SCHEMA == "durchblick_msgs/srv/TorqueStep_Request@1"
    assert ListControllersRequest.SCHEMA == "controller_manager_msgs/srv/ListControllers_Request"


# the C++ side: cpp/tests/test_messages.cpp writes these and checks them against its constants
def golden():
    return {
        "kListControllers": ListControllersResponse([state("arms"), state("arms_test", active=False)]),
        "kSwitchController": SwitchControllerRequest(["arms"], ["arms_test", "x"], SwitchControllerRequest.STRICT, True,
                                                     Duration(2, 500_000_000)),
        "kTorqueStep": TorqueStepRequest("left_elbow", 1.5, 0.25),
        "kCreateController": CreateControllerRequest("arms", "Impedance", '{"a": 1.5}'),
        "kConfigResponse": GetConfigFileResponse(False, "no file", ""),
    }


def test_cpp_golden_bytes():
    source = (Path(__file__).parent.parent / "cpp" / "tests" / "test_messages.cpp").read_text()
    for name, message in golden().items():
        constant = re.search(rf'const char\* {name} = "([0-9a-f]+)";', source).group(1)
        assert constant == bytes(message.serialize()).hex(), name
