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

# メッセージ型のインポート
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from tier4_control_msgs.msg import GateMode


# ============================================================
# 1. 基底テストクラス
# ============================================================
class BaseTestCase(unittest.TestCase):
    """
    テストの基底クラス
    - クラス単位でlaunchプロセスを管理
    - テストメソッド単位でROSノードを管理
    """

    _launch_process = None

    @classmethod
    def setUpClass(cls):
        """クラス全体の初期化（1回のみ実行）"""
        rclpy.init()

        launch_file = os.path.join(
            get_package_share_directory("system_error_monitor"),
            "launch",
            "test_system_error_monitor.launch.xml",
        )
        cls._launch_process = subprocess.Popen(
            ["ros2", "launch", launch_file],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,  # プロセスグループ作成
        )

        # ノード起動待ち
        time.sleep(3.0)

    @classmethod
    def tearDownClass(cls):
        """クラス全体のクリーンアップ（1回のみ実行）"""
        if cls._launch_process is not None:
            try:
                if cls._launch_process.poll() is None:
                    try:
                        pgid = os.getpgid(cls._launch_process.pid)
                        os.killpg(pgid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    else:
                        try:
                            cls._launch_process.wait(timeout=5.0)
                        except subprocess.TimeoutExpired:
                            try:
                                pgid = os.getpgid(cls._launch_process.pid)
                                os.killpg(pgid, signal.SIGKILL)
                                cls._launch_process.wait(timeout=2.0)
                            except (ProcessLookupError, subprocess.TimeoutExpired):
                                pass
            except Exception:
                pass
            finally:
                cls._launch_process = None

        rclpy.shutdown()

    def setUp(self):
        """各テストメソッド実行前の初期化"""
        self.test_node = rclpy.create_node(f"test_node_{self._testMethodName}")
        self.initial_evaluation_time = 1.0

    def tearDown(self):
        """各テストメソッド実行後のクリーンアップ"""
        if self.test_node is not None:
            self.test_node.destroy_node()
            self.test_node = None


# ============================================================
# 2. ユーティリティ・パブリッシュ関数
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


def publish(test_instance, publishers, inputs):
    """
    複数トピックを最小公倍数(LCM)のリズムで完全に同期してパブリッシュする
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
# 3. テストケース定義
# ============================================================
def make_test_case_default(
    diag_hz=10.0,
    gate_mode=1,  # 1: 外部運転(EXTERNAL), 0: 手動運転(AUTO)
    control_mode=2,  # 2: MANUAL
    autoware_state=5,  # 5: DRIVING
    publish_duration=5.0,
):
    """
    デフォルトのテストケースを生成
    引数でHzやデータ内容を上書き可能
    """
    inputs = {}
    outputs = {"/diagnostics_err": {"msg_type": DiagnosticArray, "pub_on_timer": False}}

    # 1. /diagnostics の生成
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

    # 2. /control/current_gate_mode の生成
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

    # 3. /vehicle/status/control_mode の生成
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

    # 4. /autoware/state の生成
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
    """
    テスト名に応じたテストケースを返す
    """
    if test_name == "INITIALIZE":
        # 初期化フェーズは is_ready を通すためだけの短いデータ
        return make_test_case_default(publish_duration=1.0, gate_mode=0)

    elif test_name == "M2":
        # M2: 外部運転, diagが0.8Hz(タイムアウト)
        return make_test_case_default(diag_hz=0.8, gate_mode=1)

    elif test_name == "M3":
        # M3: 手動運転, diagなし(タイムアウト), State=4
        return make_test_case_default(diag_hz=0.0, gate_mode=0, autoware_state=4)

    elif test_name == "M4":
        # M4: 手動運転, diagが0.8Hz(タイムアウト), State=4
        return make_test_case_default(diag_hz=0.8, gate_mode=0, autoware_state=4)

    elif test_name == "M5":
        # M5: 外部運転, diagなし(タイムアウト)
        return make_test_case_default(diag_hz=0.0, gate_mode=1)

    elif test_name == "M6":
        # M6: 外部運転, diag正常(10Hz) -> エラーなしの正常ケース
        return make_test_case_default(diag_hz=10.0, gate_mode=1)

    else:
        raise ValueError(f"Invalid test name: {test_name}")


# ============================================================
# 4. シミュレーション実行ロジック (2段階送信)
# ============================================================
def simulate_and_get_outputs(test_instance, test_name):
    # 1. 準備
    init_inputs, _ = get_test_case("INITIALIZE")
    test_inputs, _ = get_test_case(test_name)

    pubs = {}
    for topic, config in init_inputs.items():
        pubs[topic] = test_instance.test_node.create_publisher(config["msg_type"], topic, 10)

    # 検証用兼、Ready判定用のバッファ
    output_buffer = {"/diagnostics_err": []}

    def diag_callback(msg):
        # 届いたメッセージをすべてリストに蓄積
        output_buffer["/diagnostics_err"].append(msg)

    test_instance.test_node.create_subscription(
        DiagnosticArray, "/diagnostics_err", diag_callback, 10
    )

    # 2. 初期化フェーズ
    timeout_ready = time.time() + 35.0

    while len(output_buffer["/diagnostics_err"]) == 0 and time.time() < timeout_ready:
        publish(test_instance, pubs, init_inputs)
        rclpy.spin_once(test_instance.test_node, timeout_sec=0.1)

    test_instance.assertGreater(
        len(output_buffer["/diagnostics_err"]), 0, "Node did not publish any diagnostics"
    )

    output_buffer["/diagnostics_err"].clear()

    # 3. 本番テストフェーズ
    publish(test_instance, pubs, test_inputs)

    # 受信待ち時間
    end_time = time.time() + test_instance.initial_evaluation_time
    while time.time() < end_time:
        rclpy.spin_once(test_instance.test_node, timeout_sec=0.1)

    return output_buffer


# ============================================================
# 5. 検証（アサーション）関数
# ============================================================
def assert_M2_M5(test_instance, diag_err_msgs):
    """
    【M2, M5用】
    異常条件：対象ノードが存在しない、またはタイムアウトでない
    期待状態：対象ノードが存在し、かつタイムアウト（Single Point Fault）であること
    """
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
    【M3, M4用】
    異常条件：対象ノードが存在する、またはタイムアウトである
    期待状態：対象ノードが存在せず、かつタイムアウトでもないこと
    """
    target_name = (
        "autoware/control/external_control/external_command_selector/node_alive_monitoring"
    )
    target_msg = "Single Point Fault"

    exists = False
    timeout = False

    for msg in diag_err_msgs:
        for status in msg.status:
            # 1. まず対象ノードが存在するか確認
            if target_name in status.name:
                exists = True

                # 2. 対象ノードからのメッセージにタイムアウトが含まれているか確認
                if target_msg in status.message:
                    timeout = True

    success_condition = (not exists) and (not timeout)

    test_instance.assertTrue(
        success_condition,
        "M3/M4 Failed: Target node exists during manual driving, or a timeout was detected.",
    )


def assert_M6(test_instance, diag_err_msgs):
    """
    【M6用】
    異常条件：対象ノードが存在しない、またはタイムアウトである
    期待状態：対象ノードが存在し、かつタイムアウトでない（正常である）こと
    """
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
# 6. テストクラス実装
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
