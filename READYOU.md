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

- 使用 Python 脚本从键盘读取输入，将输入转换为 Twist 消息，然后发布到控制车速的消息队列，以控制小车的移动和转向。

```python
pub = rospy.Publisher('car/velocity', Twist)

while(1):
    if key in move_keys.keys():
        x, y, z, h = move_keys[getKey()]

    twist = Twist()
    twist.linear.x = x*speed
    twist.linear.y = y*speed
    # ...
    pub.publish(twist)
```

- 使用 ROS 提供的软件包`ros-noetic-diff-drive-controller`提供的差速控制器插件`libgazebo_ros_diff_drive.so`实现目标速度->轮子转动行为的转换。

```xml
<gazebo>
  <plugin name="diff_ctrl" filename="libgazebo_ros_diff_drive.so">
    ...
    <leftJoint>left_wheel_hinge</leftJoint>
    <rightJoint>right_wheel_hinge</rightJoint>
    <commandTopic>car/velocity</commandTopic>
    <odometryTopic>car/odom</odometryTopic>
    <odometryFrame>odom</odometryFrame>
    <robotBaseFrame>base_link</robotBaseFrame>
    ...
  </plugin>
</gazebo>
```

- 建模方面，首先不知道从哪找了一个`.dae`的贴图文件，像这样贴在了`model.urdf`中的小车底盘上作为外壳：

```xml
<link name="base_link">
    <visual>
        <geometry>
            <mesh filename="package://course/meshes/car.dae"/>
            <origin rpy="0 0 -1" xyz="0 0 1"/>
            ...
        </geometry>
    </visual>
</link>
```

- 车的轮子是圆柱形的碰撞体，自定义了位置、摩擦系数等参数：

```xml
<link name="left_wheel">
    <collision>
        <geometry>
        <cylinder length="0.1" radius="0.3"/>
        </geometry>
    <surface>
        <friction>
            <ode>
            <mu>114514</mu>
            </ode>
        </friction>
    </surface>
        <origin rpy="0 1 1" xyz="0 0 0"/>
    </collision>
    <visual>
        <geometry>
        <cylinder length="0.1" radius="0.3"/>
        </geometry>
        <origin rpy="0 1 1" xyz="0 0 0"/>
    </visual>
        <inertial>
    <mass value="1"/>
    <origin rpy="0 1 1" xyz="0 0 0"/>
</inertial>
</link>
```

- 底盘和四个轮通过 joint 连接，这几个 joint 就是直接由上面的差速控制器插件控制的。

```xml
<joint type="continuous" name="left_wheel_hinge">
    <axis xyz="0 1 0"/>
    <origin rpy="0 0 0" xyz="1.25 0.8 0.2"/>
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <limit effort="100" velocity="100.0"  />
</joint>
...
```

### 实验结果:

- 编写了一个 urdf 格式的模型，轮子是可旋转的、带摩擦系数的碰撞体，可以在地面上转动带动车辆移动
- 控制指令成功发布并接收。
- Python 脚本成功控制小车的左右轮子的速度，实现直行、左转、右转和停止的操作。
- 小车在 Gazebo 仿真环境中成功运动。

![](https://pic.l1nyz-tel.cc/202404082220223.png)

![](https://pic.l1nyz-tel.cc/202404082221903.png)

![](https://pic.l1nyz-tel.cc/202404082221545.png)

### 实验分析:

- 环境配置要了条老命，大家用的是Ubuntu22 on WSL和Mac，本来准备拉个docker来点跨平台，然而ROS的巨大codebase每次docker build都要build 114514分钟，甚至中间还有意义不明的图形界面安装选项所以根本无法自动化只能docker commit一点一点爬，拉完之后被X11 Forwarding和奇怪的适配问题烦死；Windows实验版体验太差让人血压升高，Ubuntu22上的最新版也是参考资料极其稀少，跨版本问题一大堆。最后妥协了，在VMWare上装了个Ubuntu20，但是还是把除了涉及ubuntu图形界面之外的东西全部搬到了SSH上。
- 学习资料稀缺，我们找到很多教程用的python还是2.x，各种工具链用的也是十几年前的化石，cli接口、序列化格式、软件包名都沧海桑田了，每一样新东西的引入都极其不顺利，需要把教程上的东西艰难地搬过来，再艰难地解决版本变迁带来的问题。
- TODO: 再写点

### 实验总结:

~~- 深入理解了 ROS 的工作原理，以及如何使用 ROS 和 Gazebo 进行机器人仿真。学习使用 Python 在 ROS 环境中编程，包括如何从键盘读取输入，将输入转换为 Twist 消息，然后发布到'carsim/cmd_vel'主题，以控制小车的移动和转向。~~  
~~- 学习使用 C++在 ROS 环境中编程，包括如何调用服务，如何组装激光扫描数据成一个点云，以及如何发布点云数据。~~  
~~- 学习使用 Python 和 C++在 ROS 环境中编程。此外，学习使用 Python 脚本控制小车的左右轮子的速度，实现直行、左转、右转和停止的操作。~~

### 参考文献:

- ROS Wiki: https://wiki.ros.org/
- Gazebo Tutorials: https://gazebosim.org/docs
- Python ROS Programming: https://wiki.ros.org/rospy/Tutorials
