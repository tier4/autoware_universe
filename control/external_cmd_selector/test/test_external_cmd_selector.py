# Copyright 2024 TIER IV, Inc.
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

# ============================================================
# 1. インポート部
# ============================================================
import os
import time
import unittest
import subprocess
import signal

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
import pytest
import rclpy

# テスト対象のメッセージ型をインポート
from diagnostic_msgs.msg import DiagnosticArray
from tier4_external_api_msgs.msg import Heartbeat

# ============================================================
# 3. 基底テストクラス
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
        # ROS2コンテキストの初期化
        rclpy.init()
        
        # launchプロセスをサブプロセスとして起動
        launch_file = os.path.join(
            get_package_share_directory("external_cmd_selector"),
            "launch",
            "test_external_cmd_selector.launch.xml",
        )
        cls._launch_process = subprocess.Popen(
            ['ros2', 'launch', launch_file],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # プロセスグループ作成
        )
        
        # ノード起動待ち
        time.sleep(2.0)

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
        self.initial_evaluation_time = 1

    def tearDown(self):
        """各テストメソッド実行後のクリーンアップ"""
        if self.test_node is not None:
            self.test_node.destroy_node()
            self.test_node = None
# ============================================================
# 4. テストケースファクトリ関数
# ============================================================
def make_test_case_default(**kwargs):
    """
    デフォルトのテストケースを生成
    
    Returns:
        inputs: dict - 入力トピック定義
            {topic_name: {"msgs": [msg, ...], "msg_type": Type, "publish_hz": int}}
        outputs: dict - 出力トピック定義
            {topic_name: {"msg_type": Type, "pub_on_timer": bool}}
    """
    inputs = {}
    outputs = {}
    
    # 入力トピックの定義
    # inputs["/topic_name"] = {
    #     "msgs": [msg1, msg2, ...],
    #     "msg_type": MsgType,
    #     "publish_hz": 10
    # }
    
    # 出力トピックの定義
    # outputs["/topic_name"] = {
    #     "msg_type": MsgType,
    #     "pub_on_timer": True/False  # タイマーで定期publish or イベント駆動
    # }
    heartbeat_msg = Heartbeat()
    inputs["/api/external/set/command/local/heartbeat"] = {
        "msgs": [heartbeat_msg] * 20,
        "msg_type": Heartbeat,
        "publish_hz": 1
    }


    outputs["/diagnostics"] = {
        "msg_type": DiagnosticArray,
        "pub_on_timer": True  # または条件によるなら False
    }
    
    return inputs, outputs


def get_test_case(test_name):
    """
    テスト名に応じたテストケースを返す
    """
    if test_name == 'default':
        return make_test_case_default()
    elif test_name == 'custom_case':
        inputs, outputs = make_test_case_default()
        # カスタマイズ
        return inputs, outputs
    else:
        raise ValueError(f"Invalid test name: {test_name}")

# ============================================================
# 5. ユーティリティ関数群
# ============================================================

def get_time_from_msg(msg):
    """メッセージからタイムスタンプを取得"""
    if hasattr(msg, 'stamp'):
        stamp = getattr(msg, 'stamp')
        return stamp.sec + stamp.nanosec * 1e-9
    elif hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
        stamp = getattr(msg.header, 'stamp')
        return stamp.sec + stamp.nanosec * 1e-9
    else:
        return None


def recursive_assign_stamp(obj, now_stamp):
    """メッセージに再帰的にタイムスタンプを設定"""
    if hasattr(obj, 'stamp'):
        try:
            setattr(obj, 'stamp', now_stamp)
        except Exception:
            pass
    if hasattr(obj, 'header'):
        try:
            setattr(obj.header, 'stamp', now_stamp)
        except Exception:
            pass
# ============================================================
# 6. Publish/Subscribe ヘルパー
# ============================================================

def publish(test_instance, publishers, inputs):
    """
    複数トピックを同期的にパブリッシュ
    
    Args:
        test_instance: テストインスタンス（self）
        publishers: {topic_name: publisher} の辞書
        inputs: テストケースの入力定義
    """
    import math
    
    # 各トピックのpublish_hzの最小公倍数を計算
    hz_list = [inputs[topic]["publish_hz"] for topic in inputs.keys()]
    lcm_hz = math.lcm(*hz_list)
    
    # メッセージをlcm_hzでパディング
    lcm_hz_msgs = {}
    for topic in inputs.keys():
        msgs = inputs[topic]["msgs"]
        hz = inputs[topic]["publish_hz"]
        period = lcm_hz // hz
        padded_msgs = []
        msg_idx = 0
        i = 0
        while msg_idx < len(msgs):
            if i % period == 0:
                padded_msgs.append(msgs[msg_idx])
                msg_idx += 1
            else:
                padded_msgs.append(None)
            i += 1
        lcm_hz_msgs[topic] = padded_msgs
    
    # 同期的にpublish
    min_len = min(len(msgs) for msgs in lcm_hz_msgs.values())
    publish_period = 1.0 / lcm_hz
    publish_start = time.time()
    
    for i in range(min_len):
        for topic, publisher in publishers.items():
            msg = lcm_hz_msgs[topic][i]
            if msg is not None:
                recursive_assign_stamp(msg, test_instance.test_node.get_clock().now().to_msg())
                publisher.publish(msg)
        while time.time() < publish_start + publish_period * (i + 1):
            rclpy.spin_once(test_instance.test_node, timeout_sec=0)


def simulate_and_initial_check(test_instance, test_name):
    """
    シミュレーション実行と初期チェック
    
    Returns:
        output_buffer: {topic_name: [received_msgs]} の辞書
    """
    default_inputs, _ = get_test_case('default')
    inputs, outputs = get_test_case(test_name)
    
    # Publisher作成
    pub = {}
    for topic in inputs.keys():
        pub[topic] = test_instance.test_node.create_publisher(
            inputs[topic]["msg_type"], topic, 10)
    
    # Subscriber作成（出力バッファに蓄積）
    output_buffer = {}
    for topic in outputs.keys():
        output_buffer[topic] = []
        test_instance.test_node.create_subscription(
            outputs[topic]["msg_type"], 
            topic, 
            lambda msg, t=topic: output_buffer[t].append(msg), 
            10
        )
    
    # 初期化用メッセージをpublish
    publish(test_instance, pub, default_inputs)
    
    # 待機
    end_time = time.time() + test_instance.initial_evaluation_time
    while time.time() < end_time:
        rclpy.spin_once(test_instance.test_node, timeout_sec=0.1)
    
    # バッファクリア
    for topic in output_buffer.keys():
        output_buffer[topic].clear()
    
    # 本テスト用メッセージをpublish
    publish(test_instance, pub, inputs)
    
    return output_buffer

# ============================================================
# 7. アサーションヘルパー
# ============================================================

def check_output_received(test_instance, outputs, output_buffer, timeout=1.0):
    """出力トピックが受信されたかチェック"""
    for topic in outputs.keys():
        if outputs[topic]["pub_on_timer"]:
            test_instance.assertGreater(
                len(output_buffer[topic]), 0,
                msg=f"No message received on timer topic: {topic}"
            )


def check_value_after_time(test_instance, msgs, assert_time, expected_value, 
                           tolerance=1e-6, timing='first'):
    """
    指定時刻以降のメッセージ値を検証
    
    Args:
        timing: 'first' - 最初の値のみ検証
                'all' - 全ての値を検証
                'once' - いずれか1つが条件を満たせばOK
    """
    results = []
    for msg in msgs:
        current_time = get_time_from_msg(msg)
        if current_time > assert_time:
            results.append(abs(msg.data - expected_value) <= tolerance)
    
    if timing == 'first':
        test_instance.assertTrue(results[0])
    elif timing == 'all':
        test_instance.assertTrue(all(results))
    elif timing == 'once':
        test_instance.assertTrue(any(results))


def test_check_rate_10hz(test_instance, diagnostics, 
                             target_hardware_id, target_name_match, 
                             min_hz=9.0, max_hz=11.0, start_time=0.0):
    """
    指定した診断項目の更新周期が指定範囲内か検証する
    
    Args:
        test_instance: unittest.TestCaseのインスタンス (self)
        diagnostics: 受信した /diagnostics メッセージのリスト
        target_hardware_id: 検索する hardware_id (例: "external_cmd_selector")
        target_name_match: 検索する name に含まれる文字列 (例: "heartbeat")
        min_hz: 許容する最小周波数
        max_hz: 許容する最大周波数
        start_time: この時刻以降のメッセージを検証対象とする
    """
    # 周期（秒）の計算
    min_interval = 1.0 / max_hz
    max_interval = 1.0 / min_hz
    
    # 条件に一致する DiagnosticStatus が見つかった時刻を抽出
    found_timestamps = []
    
    for msg in diagnostics:
        # メッセージの受信時刻（header.stamp）を取得
        current_time = get_time_from_msg(msg)
        
        # 検証開始時刻より前ならスキップ
        if current_time < start_time:
            continue

        # DiagnosticArray の中にある status リストを走査
        for status in msg.status:
            # 1. Hardware ID が一致するかチェック
            if status.hardware_id != target_hardware_id:
                continue

            # 2. Name に キーワードが含まれているかチェック
            if target_name_match in status.name: 
                found_timestamps.append(current_time)
                break 
    
    # データが少なすぎて周期計算できない場合はテスト失敗またはスキップ
    if len(found_timestamps) < 2:
        test_instance.fail(f"Diagnostics for ID='{target_hardware_id}', Name='*{target_name_match}*' not found or insufficient data.")
        return

    # 前回の時刻との差分（周期）をチェック
    for i in range(1, len(found_timestamps)):
        interval = found_timestamps[i] - found_timestamps[i-1]
        
        # 周期が範囲外ならテスト失敗
        test_instance.assertTrue(
            min_interval < interval < max_interval,
            msg=(
                f"Heartbeat period out of range! "
                f"Target: {target_hardware_id}:{target_name_match}, "
                f"Interval: {interval:.4f}s ({1.0/interval:.2f}Hz), "
                f"Expected: {min_hz}-{max_hz}Hz, "
                f"Time: {found_timestamps[i]:.2f}"
            )
        )
    print(f"Diagnostics period check passed for {target_hardware_id}:{target_name_match}.")

# ============================================================
# 8. テストクラス実装
# ============================================================

class TestNodeBehavior(BaseTestCase):
    """ノードの振る舞いをテスト（ノード再利用可能）"""
    
    def test_heartbeat_frequency(self):
        """
        /diagnosticsに含まれる heartbeat の周期が
        5Hz < 周期 < 20Hz の範囲内であることを確認するテスト
        """
        
        outputs = simulate_and_initial_check(self, 'default')

        if "/diagnostics" not in outputs:
            self.fail("Output config does not contain '/diagnostics'. Check make_test_case_default.")
            
        diag = outputs["/diagnostics"]

        self.assertGreater(len(diag), 0, "No diagnostics messages received during simulation.")

        test_check_rate_10hz(
            test_instance=self,
            diagnostics=diag,
            target_hardware_id="external_cmd_selector",
            target_name_match="heartbeat",
            min_hz=5.0,
            max_hz=20.0,
            start_time=0.0
        )
