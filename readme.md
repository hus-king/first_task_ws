# 避障

## 使用说明

### 编译项目

```bash
cd ~/first_task_ws
catkin_make
```

### 运行程序

**实体飞机上运行:**

```bash
./first_task_ws/src/collision_avoidance/shell/shell_for_330.sh
```

**仿真上运行:**

```bash
./first_task_ws/src/collision_avoidance/shell/shell_for_vmware.sh
```

若报错`tmux: command not found`，请先安装tmux:

```bash
sudo apt-get install tmux
```

### 结束任务

飞行结束后输入以下命令:

```bash
tmux kill-server
```

建议大家提前熟悉tmux的使用，可以参考[tmux入门教程](https://www.runoob.com/linux/linux-comm-tmux.html)。