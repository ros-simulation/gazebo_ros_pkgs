#!python3
# Copyright 2021 Open Source Robotics Foundation, Inc.
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

# This script reads messages from /performance_metrics topic

import os
import subprocess
import time
import select


class RunTest:
    def __init__(self):
        self.msg_count = 0

    # Echo the messages from /performance_metrics topic, let it run for a while
    def read_msgs(self):
        print("Reading the performance_metrics topic...")
        os.environ['PYTHONUNBUFFERED'] = '1'
        self.read_msgs_process = subprocess.Popen(["ros2", "topic", "echo", "/performance_metrics"], stdout=subprocess.PIPE)
        poller = select.poll()
        poller.register(self.read_msgs_process.stdout, select.POLLIN)
        start = time.time()
        output = []
        while 1:
            if poller.poll(0):
                line = self.read_msgs_process.stdout.readline()
                output.append(line)

            now = time.time()
            if now - start > 5:
                break

        self.read_msgs_process.terminate()
        if output.count(b'header:\n') > 0:
            print("Topic published messages")
        else:
            print("No messages were published")


if __name__ == "__main__":
    run_test_object = RunTest()
    run_test_object.read_msgs()
