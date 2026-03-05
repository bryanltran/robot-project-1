# Project 1

## File Structure

```
project/
└── src/
    ├── project1/
    │   ├── launch/
    │   │   └── launch.py
    │   └── worlds/
    │       └── world.sdf
    └── project1_control/
        └── project1_control/
            └── control_node.py
```

## Setup

Create a root folder and clone the repo into it:

```bash
mkdir project && cd project
git clone https://github.com/bryanltran/robot-project-1.git
```

## Running

Each step below requires its own terminal. All commands are run from `project/`.

### Terminal 1 : Build and Launch

```bash
cd project
colcon build --symlink-install
source install/setup.bash
ros2 launch project1 launch.py
```

Wait for Gazebo to finish loading before continuing.

### Terminal 2 : Autonomous Controller

```bash
cd project
source install/setup.bash
ros2 run project1_control project1_controller
```

### Terminal 3 : Keyboard Control

```bash
cd project
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_key
```

The `-r /cmd_vel:=/cmd_vel_key` remap is required so keyboard commands go through
the controller's priority system rather than bypassing it. Releasing all keys
returns control to the autonomous controller after a short timeout.
