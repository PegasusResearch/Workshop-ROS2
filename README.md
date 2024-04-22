# Workshop-ROS2

## Post Installation

1) Open the .bashrc file. This file is read by the terminal every time it starts up, and tells your terminal where the programs and libraries are located.
```
gedit ~/.bashrc
```

2) Tell the .bashrc file where ROS 2 is setup
```
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble
```

3) To avoid Python warnings, also add the following line
```
PYTHONWARNINGS="ignore:easy_install command is deprecated,ignore:setup.py install is deprecated"
export PYTHONWARNINGS
```

4) To make your journey more enjoyable, also add the following line. This will be explained during the workshop
```
alias s="source ${HOME}/.bashrc"
```

5) For handling how your ROS 2 data flows on the local network, also add these optional lines
```
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```
You can read more about this here [Domain-ID](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html).
