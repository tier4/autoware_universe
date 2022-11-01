#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2021 Tier IV, Inc. All rights reserved.
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

import glob

import config as CF
import numpy as np
import pandas as pd

# pre-defined
NUMERIC_LIMITS = 1e-02


class CSVReader(object):
    def __init__(self, csv, csv_type="dir"):
        if csv_type == "dir":
            csv_files = glob.glob(csv + "/*.csv")
            csv_list = []
            for cf in csv_files:
                print(pd.read_csv(cf, engine="python"))
                csv_list.append(pd.read_csv(cf, engine="python"))
            self.csv_data = pd.concat(csv_list, ignore_index=True)
        else:
            self.csv_data = pd.read_csv(csv, engine="python")

    def removeUnusedData(
        self,
        min_vel_thr,
        max_steer_thr,
        max_pitch_thr,
        remove_by_invalid_pedal=True,
        remove_by_vel=True,
        remove_by_steer=True,
        remove_by_pitch=True,
        remove_by_jerk=True,
    ):
        # remove unused data
        raw_size = len(self.csv_data)

        for i in range(0, raw_size)[::-1]:
            print(i)
            print(self.csv_data)
            print(self.csv_data[CF.VEL])
            vel = self.csv_data[CF.VEL][i]
            steer = self.csv_data[CF.STEER][i]
            acc_cmd = self.csv_data[CF.A_CMD][i]
            pitch = self.csv_data[CF.PITCH][i]

            # low velocity
            if remove_by_vel and vel < min_vel_thr:
                self.csv_data = self.csv_data.drop(i)
                continue

            # high steer
            if remove_by_steer and np.abs(steer) > max_steer_thr:
                self.csv_data = self.csv_data.drop(i)
                continue

            # high pitch
            if remove_by_pitch and np.abs(pitch) > max_pitch_thr:
                self.csv_data = self.csv_data.drop(i)
                continue

        return self.csv_data

    def getVelData(self):
        vel_data = np.array(self.csv_data[CF.VEL])
        return vel_data

    def getAccCmdData(self):
        acc_cmd_data = np.array(self.csv_data[CF.A_CMD])
        return acc_cmd_data

    def getAccData(self):
        acc_data = np.array(self.csv_data[CF.A_STAT])
        return acc_data

    def getPitchData(self):
        pitch_data = np.array(self.csv_data[CF.PITCH])
        return pitch_data

        return csv_data
