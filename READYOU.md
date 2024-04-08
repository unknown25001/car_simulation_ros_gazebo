### 实验介绍

本实验旨在通过使用 ROS （机器人操作系统）平台，结合 Ubuntu 操作系统、Gazebo 仿真环境等工具，完成一个四轮差速小车的建模、控制和仿真。学生将学习如何在 Ubuntu 系统上安装 ROS，并利用 ROS 提供的工具和库来创建机器人模型、编写控制程序，并在仿真环境中进行实验验证。

### 实验步骤

#### 环境搭建

安装 ros 相关依赖

```bash
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
apt update
apt install ros-noetic-desktop-full ros-noetic-effort-controllers
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
```

启动 Gazebo 界面

```bash
roslaunch carsim_gazebo carsim.launch
```

attach

```bash
rosrun carsim_gazebo teleop.py
```

#### 搭建四轮差速小车

#### 控制小车

- 使用 Python 脚本从键盘读取输入，将输入转换为 Twist 消息，然后发布到 `carsim/cmd_vel` 主题，以控制小车的移动和转向。

- 使用另一个 Python 脚本控制 carsim 小车的左右轮子的速度，实现直行、左转、右转和停止的操作。这个脚本首先初始化一个名为'car_controller'的新节点，并创建两个发布器，分别发布到'/carsim/rightWheel_effort_controller/command'和'/carsim/leftWheel_effort_controller/command'两个主题。然后，它定义了四个函数，分别用于控制小车直行、左转、右转和停止。最后，它定义了一个名为'plan_route'的函数，用于规划小车的行驶路线。
- 使用 ROS 的 launch 文件在 Gazebo 仿真环境中启动 carsim 小车模型。

* 使用另一个 launch 文件启动 RViz 可视化工具和相关节点，加载并显示 carsim 小车模型。
* 使用第三个 launch 文件启动激光扫描装配器节点，处理来自 carsim 小车的激光扫描数据。
* 使用 C++程序调用"assemble_scans"服务，将连续的激光扫描数据组装成一个点云，并发布到"/cloud"和"/cloud2"两个主题上。这个程序首先等待"assemble_scans"服务可用，然后创建一个服务客户端来调用这个服务。它还定义了两个发布器，用于发布点云数据到"/cloud"和"/cloud2"两个主题。在一个无限循环中，它设置服务请求的开始和结束时间，然后调用服务。如果服务调用成功，它会打印出点云中的点的数量，然后发布点云数据。如果服务调用失败，它会打印出错误消息。

### 实验结果:

- Python 脚本成功控制小车的左右轮子的速度，实现直行、左转、右转和停止的操作。
- C++程序成功调用了"assemble_scans"服务，将连续的激光扫描数据组装成一个点云，并发布到"/cloud"和"/cloud2"两个主题上。点云数据成功发布并接收。
- 小车在 Gazebo 仿真环境中成功运动。
- 控制指令成功发布并接收。
- 小车的运动状态成功在 RViz 中显示。

![](https://pic.l1nyz-tel.cc/202404082220223.png)

![](https://pic.l1nyz-tel.cc/202404082221903.png)

![](https://pic.l1nyz-tel.cc/202404082221545.png)

### 实验分析:

- 使用 Python 脚本从键盘读取输入，将输入转换为 Twist 消息，然后发布到 `carsim/cmd_vel` 主题，以控制小车的移动和转向。这个过程展示了 ROS 的发布-订阅模型，以及如何在 Python 中使用。
- 使用 C++ 程序调用了"assemble_scans"服务，将连续的激光扫描数据组装成一个点云，并发布到"/cloud"和"/cloud2"两个主题上。这个过程展示了 ROS 的服务调用模型，以及如何在 C++ 中使用。
- 使用 ROS 和 Gazebo 搭建并控制了一个四轮差速小车模型。使用 Python 脚本和 C++程序处理了激光扫描数据，并通过键盘控制了小车的移动。此外，使用 Python 脚本控制了小车的左右轮子的速度，实现了直行、左转、右转和停止的操作。

### 实验总结:

- 深入理解了 ROS 的工作原理，以及如何使用 ROS 和 Gazebo 进行机器人仿真。学习使用 Python 在 ROS 环境中编程，包括如何从键盘读取输入，将输入转换为 Twist 消息，然后发布到'carsim/cmd_vel'主题，以控制小车的移动和转向。
- 学习使用 C++在 ROS 环境中编程，包括如何调用服务，如何组装激光扫描数据成一个点云，以及如何发布点云数据。
- 学习使用 Python 和 C++在 ROS 环境中编程。此外，学习使用 Python 脚本控制小车的左右轮子的速度，实现直行、左转、右转和停止的操作。

### 参考文献:

- ROS Wiki: https://wiki.ros.org/
- Gazebo Tutorials: https://gazebosim.org/docs
- Python ROS Programming: https://wiki.ros.org/rospy/Tutorials
