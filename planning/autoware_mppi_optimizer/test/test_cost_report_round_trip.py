#!/usr/bin/env python3
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
"""Exercise the C++ CSV producer and Python parser together (registered by CMake)."""

import os
from pathlib import Path
import subprocess
import sys
import tempfile

from test_generate_cost_report import report


def main():
    name = "CostEvaluation.LateralBarrierThresholdsAreSymmetric"
    with tempfile.TemporaryDirectory() as directory:
        environment = dict(os.environ, MPPI_REPORT="1", MPPI_REPORT_DIR=directory)
        # Remove inherited output/filter settings so this isolated export owns its artifacts.
        environment.pop("GTEST_OUTPUT", None)
        environment.pop("GTEST_FILTER", None)
        subprocess.run([sys.argv[1], "--gtest_filter=" + name], env=environment, check=True)
        runs = list(Path(directory).glob("run_*"))
        if len(runs) != 1:
            raise AssertionError(f"Expected one C++ export, found {runs}")
        case = report.load_case(runs[0], name)
        if case.status != "PASS":
            raise AssertionError(f"C++ export rejected: {case.errors}")
        if not any(row["crash"] > 1 for row in case.rows):
            raise AssertionError("The export did not exercise encoded safety statuses")


if __name__ == "__main__":
    main()
