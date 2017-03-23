## Compilation
May need to remove some package before first compile, then restore those package and compile again.
## Usage
### Preparation **
Apply udev rule by

`sudo cp <src>/fBot-src/util/udev.rule.d/* /etc/udev/rules.d/`

Apply `mpu6050_serial_to_imu/arduino/MPU6050/MPU6050.ino` and

`util/mystrip/mystrip.ino` to arduino

Set port to launch file `robot_main/launch/includes/path_to_direction.launch` and

`robot_main/launch/includes/imu_filter.launch`

(Optional) Write udev rules for new arduino

### Run Robot

Start Robot

`roscore`

Run command

`roslaunch robot_main main.launch`

To start *create, hokuyo, joystick, tf_config, openni_tracker, people_stat, imu, turn_signal*

You may need to run

`roslaunch ros_dumbobot_nav hg_move_base_lab.launch`

### Run navigation stack

To start *navigation stack*

To monitor you need to run

`rosrun rviz rviz -d $(rospack find ros_dumbobot_nav)/rviz/my_ros_rviz.rviz`

Controller

`rosrun bot_controller command.py`

(Optional) Suggest to ssh to run on mobile

### Sync source code (Optional)
To sync code with other machine
`rsync -av fBot-src username@host:fBot-src`
