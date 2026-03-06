# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import signal
import subprocess
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from autoware_auto_system_msgs.msg import AutowareState
from autoware_auto_system_msgs.msg import HazardStatusStamped
from autoware_auto_vehicle_msgs.msg import ControlModeReport

# Import message types
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from tier4_control_msgs.msg import GateMode


# ============================================================
# 1. Base Test Class
# ============================================================
class BaseTestCase(unittest.TestCase):
    """
    Base class for tests
    - setUpClass/tearDownClass: Initialize rclpy only
    - setUp/tearDown: Create node for each test and cleanup processes
    """

    @classmethod
    def setUpClass(cls):
        """Initialize class"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Cleanup for the class"""
        rclpy.shutdown()

    def setUp(self):
        """Initialize before each test method"""
        self.test_node = rclpy.create_node(f"test_node_{self._testMethodName}")
        self.initial_evaluation_time = 3.0
        self._launch_process = None

    def tearDown(self):
        """Cleanup after each test method"""
        if self._launch_process is not None:
            try:
                if self._launch_process.poll() is None:
                    try:
                        pgid = os.getpgid(self._launch_process.pid)
                        os.killpg(pgid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    else:
                        try:
                            self._launch_process.wait(timeout=5.0)
                        except subprocess.TimeoutExpired:
                            try:
                                pgid = os.getpgid(self._launch_process.pid)
                                os.killpg(pgid, signal.SIGKILL)
                                self._launch_process.wait(timeout=2.0)
                            except (ProcessLookupError, subprocess.TimeoutExpired):
                                pass
            except Exception:
                pass
            finally:
                self._launch_process = None

        if self.test_node is not None:
            self.test_node.destroy_node()
            self.test_node = None

    def launch_target_node(self, is_psim=False):
        launch_file = os.path.join(
            get_package_share_directory("system_error_monitor"),
            "launch",
            "test_system_error_monitor.launch.xml",
        )

        cmd = ["ros2", "launch", launch_file]

        if is_psim:
            psim_config = os.path.join(
                get_package_share_directory("system_launch"),
                "config",
                "system_error_monitor.planning_simulation.param.yaml",
            )
            cmd.append(f"config_file:={psim_config}")

        self._launch_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )


# ============================================================
# 2. Utility & Publish Functions
# ============================================================
def recursive_assign_stamp(obj, now_stamp):
    if hasattr(obj, "stamp"):
        try:
            setattr(obj, "stamp", now_stamp)
        except Exception:
            pass
    if hasattr(obj, "header"):
        try:
            setattr(obj.header, "stamp", now_stamp)
        except Exception:
            pass


def get_time_from_msg(msg):
    """Get timestamp from message"""
    if hasattr(msg, "stamp"):
        stamp = getattr(msg, "stamp")
        return stamp.sec + stamp.nanosec * 1e-9
    elif hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        stamp = getattr(msg.header, "stamp")
        return stamp.sec + stamp.nanosec * 1e-9
    else:
        return None


def publish(test_instance, publishers, inputs):
    """
    Publish multiple topics synchronously at the rhythm of the Least Common Multiple (LCM)
    """
    import math

    hz_list_x10 = [
        int(float(config["publish_hz"]) * 10)
        for config in inputs.values()
        if float(config["publish_hz"]) > 0
    ]

    if not hz_list_x10:
        return

    lcm_hz_x10 = math.lcm(*hz_list_x10)

    base_lcm_hz = lcm_hz_x10 / 10.0

    lcm_hz_msgs = {}
    for topic, config in inputs.items():
        msgs = config["msgs"]
        hz = float(config["publish_hz"])
        skip_period = int(base_lcm_hz // hz)

        padded_msgs = []
        msg_idx = 0
        tick = 0
        while msg_idx < len(msgs):
            if tick % skip_period == 0:
                padded_msgs.append(msgs[msg_idx])
                msg_idx += 1
            else:
                padded_msgs.append(None)
            tick += 1
        lcm_hz_msgs[topic] = padded_msgs

    min_len = min(len(m) for m in lcm_hz_msgs.values())
    publish_period = 1.0 / base_lcm_hz
    publish_start = time.time()

    for i in range(min_len):
        for topic in lcm_hz_msgs.keys():
            publisher = publishers[topic]
            msg = lcm_hz_msgs[topic][i]
            if msg is not None:
                recursive_assign_stamp(msg, test_instance.test_node.get_clock().now().to_msg())
                publisher.publish(msg)

        while time.time() < publish_start + publish_period * (i + 1):
            rclpy.spin_once(test_instance.test_node, timeout_sec=0)


# ============================================================
# 3. Test Case Definitions
# ============================================================
def make_test_case_default(
    diag_hz=10.0,
    gate_mode=1,  # 1: EXTERNAL, 0: AUTO
    control_mode=2,  # 2: MANUAL
    autoware_state=5,  # 5: DRIVING
    publish_duration=5.0,
):
    """
    Generate default test case.
    Hz and data content can be overridden via arguments.
    """
    inputs = {}
    outputs = {"/diagnostics_err": {"msg_type": DiagnosticArray, "pub_on_timer": False}}

    # 1. Generate /diagnostics
    if diag_hz > 0:
        msg_type = DiagnosticArray
        input_msgs = []
        for _ in range(int(publish_duration * diag_hz)):
            input_msg = msg_type()
            status = DiagnosticStatus()
            status.level = b"\x00"
            status.name = "external_cmd_selector: heartbeat"
            status.message = "Alive"
            status.hardware_id = "external_cmd_selector"
            input_msg.status.append(status)
            input_msgs.append(input_msg)
        inputs["/diagnostics"] = {"msgs": input_msgs, "msg_type": msg_type, "publish_hz": diag_hz}

    # 2. Generate /control/current_gate_mode
    gate_hz = 10.0
    msg_type = GateMode
    input_msgs = []
    for _ in range(int(publish_duration * gate_hz)):
        input_msg = msg_type()
        input_msg.data = gate_mode
        input_msgs.append(input_msg)
    inputs["/control/current_gate_mode"] = {
        "msgs": input_msgs,
        "msg_type": msg_type,
        "publish_hz": gate_hz,
    }

    # 3. Generate /vehicle/status/control_mode
    control_hz = 50.0
    msg_type = ControlModeReport
    input_msgs = []
    for _ in range(int(publish_duration * control_hz)):
        input_msg = msg_type()
        input_msg.mode = control_mode
        input_msgs.append(input_msg)
    inputs["/vehicle/status/control_mode"] = {
        "msgs": input_msgs,
        "msg_type": msg_type,
        "publish_hz": control_hz,
    }

    # 4. Generate /autoware/state
    state_hz = 10.0
    msg_type = AutowareState
    input_msgs = []
    for _ in range(int(publish_duration * state_hz)):
        input_msg = msg_type()
        input_msg.state = autoware_state
        input_msgs.append(input_msg)
    inputs["/autoware/state"] = {"msgs": input_msgs, "msg_type": msg_type, "publish_hz": state_hz}

    return inputs, outputs


def get_test_case(test_name):
    """Return test case based on test name"""
    if test_name == "M2":
        # M2: External driving, diag at 0.8Hz (Timeout)
        return make_test_case_default(diag_hz=0.8, gate_mode=1)

    elif test_name == "M3":
        # M3: Manual driving, no diag (Timeout), State=4
        return make_test_case_default(diag_hz=0.0, gate_mode=0, autoware_state=4)

    elif test_name == "M4":
        # M4: Manual driving, diag at 0.8Hz (Timeout), State=4
        return make_test_case_default(diag_hz=0.8, gate_mode=0, autoware_state=4)

    elif test_name == "M5":
        # M5: External driving, no diag (Timeout)
        return make_test_case_default(diag_hz=0.0, gate_mode=1)

    elif test_name == "M6":
        # M6: External driving, normal diag (10Hz) -> Normal case
        return make_test_case_default(diag_hz=10.0, gate_mode=1)

    else:
        raise ValueError(f"Invalid test name: {test_name}")


# ============================================================
# 4. Simulation Execution Logic
# ============================================================
def simulate_and_get_outputs(test_instance, test_name, is_psim=False):
    # 1. Preparation
    test_inputs, _ = get_test_case(test_name)

    pubs = {}
    for topic, config in test_inputs.items():
        pubs[topic] = test_instance.test_node.create_publisher(config["msg_type"], topic, 10)

    output_buffer = {"/diagnostics_err": []}

    def diag_callback(msg):
        # Accumulate all received messages in a list
        output_buffer["/diagnostics_err"].append(msg)

    test_instance.test_node.create_subscription(
        DiagnosticArray, "/diagnostics_err", diag_callback, 10
    )

    # 2. Start target node launch process
    test_instance.launch_target_node(is_psim=is_psim)

    # 3. Start data transmission
    publish(test_instance, pubs, test_inputs)

    # 4. Wait for processing
    end_time = time.time() + test_instance.initial_evaluation_time
    while time.time() < end_time:
        rclpy.spin_once(test_instance.test_node, timeout_sec=0.1)

    return output_buffer


# ============================================================
# 5. Assertion Functions
# ============================================================
def assert_duration_at_least_5s(test_instance, msgs):
    """Verify if data duration is at least 5 seconds"""
    start_time = get_time_from_msg(msgs[0])
    end_time = get_time_from_msg(msgs[-1])

    if start_time is None or end_time is None:
        test_instance.fail("Could not extract timestamps from messages.")

    duration = end_time - start_time

    test_instance.assertTrue(
        duration >= 5.0, f"Data duration too short: {duration:.2f}s. Expected >= 5.0s."
    )


def assert_M2_M5(test_instance, diag_err_msgs):
    """
    [M2, M5]
    Error condition: Target node does not exist or it is not a timeout
    Expected state: Target node exists and is a timeout (Single Point Fault)
    """
    assert_duration_at_least_5s(test_instance, diag_err_msgs)

    target_name = (
        "autoware/control/external_control/external_command_selector/node_alive_monitoring"
    )
    target_msg = "Single Point Fault"

    exist_and_timeout = False
    for msg in diag_err_msgs:
        for status in msg.status:
            if target_name in status.name and target_msg in status.message:
                exist_and_timeout = True
                break

    test_instance.assertTrue(
        exist_and_timeout,
        "M2/M5 Failed: Target node does not exist, or it is not a timeout (Single Point Fault).",
    )


def assert_M3_M4(test_instance, diag_err_msgs):
    """
    [M3, M4]
    Error condition: Target node exists or it is a timeout
    Expected state: Target node does not exist and is not a timeout
    """
    assert_duration_at_least_5s(test_instance, diag_err_msgs)

    target_name = (
        "autoware/control/external_control/external_command_selector/node_alive_monitoring"
    )
    target_msg = "Single Point Fault"

    exists = False
    timeout = False

    for msg in diag_err_msgs:
        for status in msg.status:
            if target_name in status.name:
                exists = True

                if target_msg in status.message:
                    timeout = True

    success_condition = (not exists) and (not timeout)

    test_instance.assertTrue(
        success_condition,
        "M3/M4 Failed: Target node exists during manual driving, or a timeout was detected.",
    )


def assert_M6(test_instance, diag_err_msgs):
    """
    [M6]
    Error condition: Target node does not exist or it is a timeout
    Expected state: Target node exists and is not a timeout (normal)
    """
    assert_duration_at_least_5s(test_instance, diag_err_msgs)

    target_name = (
        "autoware/control/external_control/external_command_selector/node_alive_monitoring"
    )
    target_msg = "Single Point Fault"

    exists = False
    timeout = False

    for msg in diag_err_msgs:
        for status in msg.status:
            if target_name in status.name:
                exists = True
                if target_msg in status.message:
                    timeout = True

    success_condition = exists and not timeout
    test_instance.assertTrue(
        success_condition,
        "M6 Failed: Target node does not exist, or a timeout was detected during normal operation (10Hz).",
    )


# ============================================================
# 6. Test Class Implementation
# ============================================================
class TestSystemErrorMonitor2(BaseTestCase):
    def test_M2_check_error_detection(self):
        outputs = simulate_and_get_outputs(self, "M2")
        assert_M2_M5(self, outputs["/diagnostics_err"])


class TestSystemErrorMonitor3(BaseTestCase):
    def test_M3_check_error_detection(self):
        outputs = simulate_and_get_outputs(self, "M3")
        assert_M3_M4(self, outputs["/diagnostics_err"])


class TestSystemErrorMonitor4(BaseTestCase):
    def test_M4_check_error_detection(self):
        outputs = simulate_and_get_outputs(self, "M4")
        assert_M3_M4(self, outputs["/diagnostics_err"])


class TestSystemErrorMonitor5(BaseTestCase):
    def test_M5_check_error_detection(self):
        outputs = simulate_and_get_outputs(self, "M5")
        assert_M2_M5(self, outputs["/diagnostics_err"])


class TestSystemErrorMonitor6(BaseTestCase):
    def test_M6_check_error_detection(self):
        outputs = simulate_and_get_outputs(self, "M6")
        assert_M6(self, outputs["/diagnostics_err"])


class TestSystemErrorMonitor7(BaseTestCase):
    def test_M7_check_error_detection(self):
        outputs = simulate_and_get_outputs(self, "M2", is_psim=True)
        assert_M2_M5(self, outputs["/diagnostics_err"])


class TestSystemErrorMonitor8(BaseTestCase):
    def test_M8_check_error_detection(self):
        outputs = simulate_and_get_outputs(self, "M3", is_psim=True)
        assert_M3_M4(self, outputs["/diagnostics_err"])


class TestSystemErrorMonitor9(BaseTestCase):
    def test_M9_check_error_detection(self):
        outputs = simulate_and_get_outputs(self, "M4", is_psim=True)
        assert_M3_M4(self, outputs["/diagnostics_err"])


class TestSystemErrorMonitor10(BaseTestCase):
    def test_M10_check_error_detection(self):
        outputs = simulate_and_get_outputs(self, "M5", is_psim=True)
        assert_M2_M5(self, outputs["/diagnostics_err"])


class TestSystemErrorMonitor11(BaseTestCase):
    def test_M11_check_error_detection(self):
        outputs = simulate_and_get_outputs(self, "M6", is_psim=True)
        assert_M6(self, outputs["/diagnostics_err"])
