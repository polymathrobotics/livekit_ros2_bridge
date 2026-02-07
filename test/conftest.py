# Copyright (c) 2025-present Polymath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import logging
import subprocess
import sys
from pathlib import Path

import pytest

from test.support.livekit_stubs import install_livekit_stubs


def _generate_parameters_module() -> None:
    # Generate the untracked params module before collection so imports in node.py succeed.
    root = Path(__file__).resolve().parents[1]
    output_path = root / "livekit_ros2_bridge" / "parameters.py"
    yaml_path = root / "livekit_ros2_bridge" / "parameters.yaml"
    command = [
        sys.executable,
        "-m",
        "generate_parameter_library_py.generate_python_module",
        str(output_path),
        str(yaml_path),
        "livekit_ros2_bridge.core.validators",
    ]
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        stderr = result.stderr.strip()
        stdout = result.stdout.strip()
        raise RuntimeError(
            "Failed to generate livekit_ros2_bridge/parameters.py for pytest collection.\n"
            f"Command: {' '.join(command)}\n"
            f"stdout: {stdout}\n"
            f"stderr: {stderr}"
        )


_generate_parameters_module()
install_livekit_stubs(include_api=True)


@pytest.fixture(autouse=True)
def configure_logging():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )
