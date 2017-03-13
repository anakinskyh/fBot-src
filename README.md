## Compilation
May need to remove some package before first compile, then restore those package and compile again.
## Usage
### Preparation **
apply udev rule by

`sudo cp <src>/fBot-src/util/udev.rule.d/* /etc/udev/rules.d/`

### Run Hardware
All command

`roslaunch robot_main core_hw.launch`

`roslaunch robot_main cmd_filter.launch`

Firstly, You need to run
`roslaunch robot_main core_hw.launch`
By the command You stated *create, hokuyo, joystick, tf_config*.

*joystick* publish raw_cmd_vel.

*tf_config* publish many fake tf please check before.

Secondly, to make robot can walk you need to run
`roslaunch robot_main cmd_filter.launch`, This command will convert raw_cmd_vel to cmd_vel

### Run LED
Run `roslaunch robot_main running_avg_led.launch`

### Run navigation stack
You need to run `roslaunch ros_dumbobot_nav hg_move_base_lab.launch`.
If you need to monitor on other machine you can run
`rosrun rviz rviz -d $(rospack find ros_dumbobot_nav)/rviz/my_ros_rviz.rviz`

And If you need to run human detection module you need to connect to kinect and run `roslaunch robot_main openni_tracker.launch`

That will run *openni_tracker and people_stat*

### Sync source code
To sync code with other machine
`rsync -av fBot-src username@host:fBot-src`
