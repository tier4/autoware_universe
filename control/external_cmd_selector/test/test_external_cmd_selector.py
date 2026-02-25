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
import rclpy

# テスト対象のメッセージ型をインポート
from diagnostic_msgs.msg import DiagnosticArray

# ============================================================
# 2. 基底テストクラス
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
        time.sleep(5.0)

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

    def tearDown(self):
        """各テストメソッド実行後のクリーンアップ"""
        if self.test_node is not None:
            self.test_node.destroy_node()
            self.test_node = None

# ============================================================
# 3. テストケースファクトリ関数
# ============================================================
"""
「入力条件：なし」のため、入力データの作成・定義が不要
テスト対象の出力データが1つであり、定義と使用関数で分ける必要性がないため、出力データの作成・定義が不要
"""
    

# ============================================================
# 4. ユーティリティ関数群
# ============================================================
"""
本テストコードからデータを出力しないため、タイムスタンプの取得・設定が不要
"""

# ============================================================
# 5. Publish/Subscribe ヘルパー
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

def collect_diagnostics(test_instance, timeout_sec):
    """
    診断メッセージの収集
    Returns:
        received_msgs: 受信したDiagnosticArrayのリスト
    """
    received_msgs = []
    
    # Subscriber作成
    sub = test_instance.test_node.create_subscription(
        DiagnosticArray, 
        '/diagnostics', 
        lambda msg: received_msgs.append(msg), 
        10
    )
    
    # 指定時間データ収集
    start_time = time.time()
    while (time.time() - start_time) < timeout_sec:
        rclpy.spin_once(test_instance.test_node, timeout_sec=0.1)
    
    test_instance.test_node.destroy_subscription(sub)
    return received_msgs

# ============================================================
# 6. アサーションヘルパー
# ============================================================

def check_diagnostics_rate(test_instance, diagnostics, 
                             target_hardware_id, target_name_match, 
                             min_hz=5.0, max_hz=20.0, start_time=0.0):
    """
    指定した診断項目の更新周期が指定範囲内か検証する
    """
    min_interval = 1.0 / max_hz
    max_interval = 1.0 / min_hz
    
    found_timestamps = []
    
    for msg in diagnostics:
        current_time = get_time_from_msg(msg)

        for status in msg.status:
            if status.hardware_id != target_hardware_id:
                continue
            if target_name_match in status.name: 
                found_timestamps.append(current_time)
                break 
    
    if len(found_timestamps) < 2:
        test_instance.fail(f"Diagnostics for ID='{target_hardware_id}', Name='*{target_name_match}*' not found or insufficient data.")
        return

    for i in range(1, len(found_timestamps)):
        interval = found_timestamps[i] - found_timestamps[i-1]
        
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
# 7. テストクラス実装
# ============================================================

class TestNodeBehavior(BaseTestCase):
    """ノードの振る舞いをテスト"""
    
    def test_check_rate_10hz(self):
        """
        /diagnosticsに含まれる h7eartbeat の周期が
        5Hz < 周期 < 20Hz の範囲内であることを確認するテスト
        """

        # 監視時間の設定
        wait_time_sec = 3.0
        
        # 診断メッセージの収集
        diag_msgs = collect_diagnostics(self, timeout_sec=wait_time_sec)


        check_diagnostics_rate(
            test_instance=self,
            diagnostics=diag_msgs,
            target_hardware_id="external_cmd_selector",
            target_name_match="heartbeat",
            min_hz=5.0,
            max_hz=20.0
        )
