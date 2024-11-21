# vikings_bot_description

<hr>

### Description
This package contains `vikings_bot` project robot description related files.

<hr>

### Installation

Download source and install dependencies:
```
cd <path/to/your/ros_ws>
git clone git@github.com:Hercogs/vikings_bot_description.git src/vikings_bot_description
rosdep update
rosdep install --ignore-src --default-yes --from-path src
```

Build package:
```
colcon build
source install/setup.bash
```

<hr>

### Usage

To see robot model in RVIZ, launch:
```ros2 launch vikings_bot_description xacro_visualize.launch.py```

To launch robot state publisher:
```ros2 launch vikings_bot_description robot_state_publisher.launch.py```



