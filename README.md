## Compilation
------
May need to remove some package before first compile, then restore those package and compile again.
======
## Usage
### Preparation **
apply udev rule by

`sudo cp .../fBot-src/util/udev.rule.d/* ...`

### Run Hardware
All command

`roslaunch robot_main core_hw.launch`

`roslaunch robot_main cmd_filter.launch`
---
Firstly, You need to run
`roslaunch robot_main core_hw.launch`
By the command You stated *create, hokuyo, joystick, tf_config*.

*joystick* publish raw_cmd_vel.

*tf_config* publish many fake tf please check before.

Secondly, to make robot can walk you need to run
`roslaunch robot_main cmd_filter.launch`, This command will convert raw_cmd_vel to cmd_vel

### Run LED
Run `roslaunch robot_main `

### Run navigation stack
======
