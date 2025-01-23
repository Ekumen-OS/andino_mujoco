# BSD 3-Clause License
#
# Copyright (c) 2025, Ekumen Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Run a Mujoco simulation of and Andino in the office environment.

The Andino can be controlled using the keyboard in a teleop_twist_keyboard
fashion.
"""

from multiprocessing import Process, Value
import sys
import termios
import tty


def input_handler(
    linear_direction,
    angular_direction,
    linear_speed,
    angular_speed,
    acceleration,
    keep_running,
):
    """Check for user input and control the Andino."""
    directions = {
        "q": (1.0, 1.0),
        "w": (1.0, 0.0),
        "e": (1.0, -1.0),
        "a": (0.0, 1.0),
        "s": (0.0, 0.0),
        "d": (0.0, -1.0),
        "z": (-1.0, 1.0),
        "x": (-1.0, 0.0),
        "c": (-1.0, -1.0),
    }
    speed_modifiers = {
        "u": (1.1, 1.1),
        "i": (0.9, 0.9),
        "j": (1.1, 1.0),
        "k": (0.9, 1.0),
        "m": (1.0, 1.1),
        ",": (1.0, 0.9),
    }
    acceleration_modifiers = {
        "h": 1.1,
        "n": 0.9,
    }
    # Save terminal settings
    terminal_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        while keep_running.value:
            key = sys.stdin.read(1)
            if key in directions:
                (
                    linear_direction.value,
                    angular_direction.value,
                ) = directions[key]
            elif key in speed_modifiers:
                linear_speed.value *= speed_modifiers[key][0]
                angular_speed.value *= speed_modifiers[key][1]
            elif key in acceleration_modifiers:
                acceleration.value *= acceleration_modifiers[key]
            elif key == "p":
                break
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
    keep_running.value = False


def simulation(
    linear_direction,
    angular_direction,
    linear_speed,
    angular_speed,
    acceleration,
    keep_running,
):
    """Run the Andino simulation."""
    import mujoco
    import mujoco.viewer

    # Specify the path to the XML model
    model_path = "/models/andino_mujoco_description/mjcf/scene.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    model.opt.timestep = 1e-3  # 1ms
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:

        while viewer.is_running() and keep_running.value:
            # Calculate the control actions
            linear_vel = linear_direction.value * linear_speed.value
            angular_vel = angular_direction.value * angular_speed.value

            left_wheel_velocity = linear_vel - angular_vel
            right_wheel_velocity = linear_vel + angular_vel

            curr_left_motor_ctrl = data.actuator("left-motor").ctrl[0]
            curr_right_motor_ctrl = data.actuator("right-motor").ctrl[0]

            curr_acceleration = acceleration.value

            left_motor_ctrl = (
                min(
                    left_wheel_velocity,
                    curr_left_motor_ctrl + curr_acceleration,
                )
                if curr_left_motor_ctrl <= left_wheel_velocity
                else max(
                    left_wheel_velocity,
                    curr_left_motor_ctrl - curr_acceleration,
                )
            )
            right_motor_ctrl = (
                min(
                    right_wheel_velocity,
                    curr_right_motor_ctrl + curr_acceleration,
                )
                if curr_right_motor_ctrl <= right_wheel_velocity
                else max(
                    right_wheel_velocity,
                    curr_right_motor_ctrl - curr_acceleration,
                )
            )

            data.actuator("left-motor").ctrl[0] = left_motor_ctrl
            data.actuator("right-motor").ctrl[0] = right_motor_ctrl

            # Advance the simulation
            mujoco.mj_step(model, data)
            viewer.sync()

        # To stop the input handler thread
        keep_running.value = False


if __name__ == "__main__":
    message = """-------------

Moving around:

   q    w    e

   a    s    d

   z    x    c

u/i : increase/decrease max speeds by 10%
j/k : increase/decrease only linear speed by 10%
m/, : increase/decrease only angular speed by 10%

h/n : increase/decrease acceleration by 10%

Press "p" to quit
"""

    linear_direction = Value("d", 0.0)
    angular_direction = Value("d", 0.0)
    linear_speed = Value("d", 20.0)
    angular_speed = Value("d", 20.0)
    acceleration = Value("d", 0.05)
    keep_running = Value("b", True)

    simulation_process = Process(
        target=simulation,
        args=(
            linear_direction,
            angular_direction,
            linear_speed,
            angular_speed,
            acceleration,
            keep_running,
        ),
    )
    simulation_process.start()

    # Hack to display warnings correctly during simulation:
    # As the input handler puts the terminal in raw mode, the warnings
    # printed when initializing Mujoco's passive viewer are poorly formatted.
    # These warnings can't be suppressed by redirecting stdout or stderr,
    # and are beyond our control, so we have to live with them.
    import time

    time.sleep(0.5)

    print(message)
    input_handler(
        linear_direction,
        angular_direction,
        linear_speed,
        angular_speed,
        acceleration,
        keep_running,
    )
    simulation_process.join()
