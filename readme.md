# 避障

## 飞机上使用

cd ~/first_task_ws
catkin_make

后

./first_task_ws/src/collision_avoidance/shell/shell_for_330.sh

即可

飞行结束后输入

tmux kill-server 结束任务

## 虚拟机上使用

虚拟机上要先安装Livox 官方的 ROS 驱动包

# 进入你的工作空间 src 目录
cd ~/first_task_ws/src

# 克隆官方驱动
git clone https://github.com/Livox-SDK/livox_ros_driver.git

# 回到工作空间根目录编译
cd ~/first_task_ws

先单独编译livox_ros_driver

catkin_make -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver"

再编译collision_avoidance

catkin_make -DCATKIN_WHITELIST_PACKAGES="collision_avoidance"

