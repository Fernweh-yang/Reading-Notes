# 零、学习资源汇总

- dqrobotics的[文档](https://dqroboticsgithubio.readthedocs.io/en/latest/index.html)，[论文](https://arxiv.org/abs/1910.11612)
- vrep(coppeliasim)的[文档](https://www.coppeliarobotics.com/helpFiles/index.html)

- dq-cpp-vrep: [git](https://github.com/dqrobotics/cpp-examples) 

# 一、dqrobotics

## 1. 安装编译

### 1.1 安装

- Matlab安装

  1. 在[网上](https://dqroboticsgithubio.readthedocs.io/en/latest/installation/matlab.html)下载扩展包dqrobotics-YY-MM.mltbx
  2. 在matlab中打开包所在的文件夹，双击这个扩展包，就安装好了

- C++安装

  参照[官网](https://dqroboticsgithubio.readthedocs.io/en/latest/installation/cpp.html#including)

  ```shell
  sudo add-apt-repository ppa:dqrobotics-dev/development
  sudo apt-get update
  sudo apt-get install libdqrobotics					# 安装dq包，包含头：
  	#include <dqrobotics/DQ.h>
  	#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
  	#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
  	#include <dqrobotics/utils/DQ_Geometry.h>
  sudo apt-get install libdqrobotics-interface-vrep 	# 安装dq-vrep接口包，包含头：
  	#include<dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
  	#include<dqrobotics/interfaces/vrep/DQ_VrepRobot.h>
  	#include<dqrobotics/interfaces/vrep/robots/LBR4pVrepRobot.h>
  	#include<dqrobotics/interfaces/vrep/robots/YouBotVrepRobot.h>
  ```

### 1.2. C++编译

matlab/python不需要额外写编译文件，c++需要写CMakeLists.txt

- 对于dq包:

  ```cmake
  target_link_libraries(my_binary dqrobotics)
  ```

- 对于dq-vrep接口包:

  ```cmake
  target_link_libraries(my_binary dqrobotics dqrobotics-interface-vrep)
  ```

## 2. 理论

### 2.1 四元数

四元数（Quaternion）是一种数学构造，通常用于表示三维空间中的旋转。四元数由四个实数构成，表示为 (x, y, z, w)。

- x、y、z：表示旋转轴上的向量分量。这些分量定义了旋转轴的方向，指向一个单位向量在三维空间中的 x、y 和 z 方向的分量。
- w：表示旋转角度的余弦一半。它定义了绕旋转轴旋转的角度。四元数表示的旋转角度为原始角度的一半。

#### 2.1.1 轴角转四元数

- 考虑轴$\vec{u}$
  $$
  \vec{u}=(u_x,u_y,u_z)=u_xi+u_yj+u_zk\tag{1}
  $$

- 绕轴旋转$\theta$，表示为四元数
  $$
  q=e^{\frac{\theta}{2}(u_zi+u_yj+u_zk)}=cos\frac{\theta}{2}+(u_xi+u_yj+u_zk)sin\frac{\theta}{2}=cos\frac{\theta}{2}+\vec{u}sin\frac{\theta}{2} \tag{2}
  $$

#### 2.1.2 eigen定义四元数

```c++
#include <Eigen/Geometry>
#include <iostream>

void outputAsMatrix(const Eigen::Quaterniond& q)
{
    // 注意只有单位四元数才表示旋转矩阵，所以要先对四元数单位化
    std::cout << "R=" << std::endl << q.normalized().toRotationMatrix() << std::endl;
}

int main() {
    // 定义一个双精度的四元数
    auto angle = M_PI / 4;
    auto sinA = std::sin(angle / 2);
    auto cosA = std::cos(angle / 2);
    // 方法1：纸上按公式2算好每个值直接写
    Eigen::Quaterniond q(cos(angle / 2), 0, sin(angle / 2), 0);
    // 方法2：更符合公式2的写法
    Eigen::Quaterniond q;
    q.x() = 0 * sinA;
    q.y() = 1 * sinA;
    q.z() = 0 * sinA;
    q.w() = cosA;   
    //方法3：先写轴角然后用eigen转成四元数
    Eigen::AngleAxisd axis_angle(M_PI / 4.0, Eigen::Vector3d::UnitZ());// 定义一个轴角旋转，绕着 z 轴旋转 45 度
    Eigen::Quaterniond quaternion(axis_angle);// 将轴角旋转转换为四元数
    

    // 输出四元数的实部和虚部
    std::cout << "Real part: " << q.w() << std::endl;
    std::cout << "Imaginary parts (x, y, z): " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;
    
    //矩阵的形式输出
    outputAsMatrix(q);
    outputAsMatrix(Eigen::Quaterniond{Eigen::AngleAxisd{angle, Eigen::Vector3d{0, 1, 0}}});

    return 0;
}
```

### 2.2 对偶四元数

- 对偶四元数（Dual Quaternions）使用两个四元数来同时表示平移和旋转
  $$
  u=[q_r,q_d]=q_r+q_d\epsilon \tag{1}
  $$

  - $q_r$：四元数，表示实部real
  - $q_d$：对偶数，表示对偶部dual
  - $\epsilon: \epsilon^2=0,\epsilon \neq0$
  
- 纯旋转的对偶四元数

  即某个单位四元数$q_r$,将它扩展成对偶四元数$u=[q_r,0]$就是一个纯旋转的对偶四元数。

- 纯位移的对偶四元数

  某个位移$t=[t_1,t_2,t_3]$，它所对应的对偶四元数为$u=[1,0.5p]$
  
  - 其中四元数$p=[t_1,t_2,t_3,0]$
  
  - 为什么实部为1？
  
    因为旋转$\theta=0$，所以$q_r=[cos\frac{\theta}{2}+\vec{u}sin\frac{\theta}{2} ]=1$
  
- 刚体运动的对偶四元数
  
  即又有旋转$r$，又有位移$p$，此时:
  $$
  \begin{align}
  u&=[q_r,q_d]=q_r+q_d\epsilon \\
  q_r& = r \\
  q_d& = \frac{1}{2}pr
  \end{align} \tag{2}
  $$

#### 2.2.1 dqrobotics定义对偶四元数

```c++
/* dqrobotics中给了4个别名i_, j_, k_, and E_，分别对应x,y,z轴和\epsilon
下面要定义一个位移为(0.1,0.2,0.3)，绕y轴旋转pi的对偶四元数
*/
// 方法1：
DQ r,p,xd;
r = cos(pi/2) + j_*sin(pi/2); 	// 旋转四元数
p = 0.1*i_ + 0.2*j_ + 0.3*k_; 	// 位移四元数
xd = r + E_*0.5*p*r;			// 应用公式2
// 方法2：
DQ r = DQ(cos(pi/2), 0, sin(pi/2), 0); // 旋转四元数
DQ p = DQ(0, 0.1, 0.2, 0.3);		   // 位移四元数
xd = r + E_*0.5*p*r;				   // 应用公式2
```

#### 2.2.2 dqrobotics和eigen对四元数表示的区别

- dqrobotics:

  如2.2.1所示dqrobotics中四元数不管是内部存储还是复制，它的顺序都是$q=(w,x,y,z)$，即旋转w在最前面。

- eigen:

  - 赋值时是`Quaterniond q(w, x, y, z);`

  - 内部存储是`[x,y,z,w]`

    所以最后输出同样是`[x,y,z,w]`

    

# 二、路径规划

## 1. Path Planning是如何做的

示例代码注释见：[git](https://github.com/Fernweh-yang/CodeComments_franka/tree/main/panda_tutorial/source/04_Panda_Cartesian_Trajectory)

- 总体轨迹overall trajectory被描述为：
  $$
  p(t):\mathbb{R}\rightarrow\mathbb{R}^3
  $$

- 下面将**空间规划**p(s)（机械臂在三维空间中的路径规划）与**时间规划**s(t)（机械臂运动的时间控制）分开处理：

  - 路径参数用弧长(arc length)表示，即用路径上的弧长值s来表示机械臂在路径上的位置：
    $$
    p(s)\in\mathbb{R}^3,\ \ \ s\in[0,L] \tag{1}
    $$
    ![img](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/SpeedProfile.svg)

    <center style="color:#C125C0C0">图1:speedProfile</center>

  - 这个路径上的弧长值是关于时间的函数$s(t):\mathbb{R}\rightarrow\mathbb{R}$，映射了弧长值和时间坐标

  - $s(t)$是由motion profile描述的，将其带入1式后，就可以得到：
    $$
    p(t)=p(s(t))\in\mathbb{R}^3 \tag{2}
    $$
    ![img](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/SampleTrajectory.svg)

    <center style="color:#C125C0C0">图2:sample trajectory</center>

### 1.1. Motion Profile

- 什么是motion profile"

  机械臂路径规划中的Motion Profile（运动剖面）是指描述机械臂运动随时间变化的速度、加速度和位置的曲线。如上面的$s(t)$

## 2. trajectory planning

参考书：modern robotics

### 2.1 基本概念：

**trajectory**：the specification具体说明 of robot positon as a function of time。

- 轨迹由2部分组成：
  - path：a purely geometric description of the sequence of configurations achieved by the robot
  - time scaling：specifies the times when those configurations
    are reached
- 轨迹可分为3种
  - point-to-point straight-line trajectories in both joint space and task space; 
  - trajectories passing through a sequence of timed via points; 
  - minimum-time trajectories along specified paths taking actuator limits into consideration

**trajectory planning**：End-effector需要从一位姿移动到另一位姿，需要设计一条满足各个constranits的轨迹

### 2.2 Path

最简单的就是直线

- 如果是joint space很简单

  $\theta(s)=\theta_{start}+s(\theta_{end}-\theta_{start}),\ s\in[0,1]$

- 如果是task space，由于SE(3)对加法不封闭，所以有2种方法来表示：

  - Screw motion:

    X是End-effector的 pose
    $$
    X(s)=X_{start}exp(log(X_{start}^{-1}X_{end})s)
    $$

  - decoupled rotational motion:

    此时$X=(R,p)$
    $$
    \begin{align}
    p(s)&=p_{start}+s(p_{end}-p_{start})\\
    R(s)&=R_{start}exp(log(R_{start}^TR_{end})s)
    \end{align}
    $$
    

### 2.3 Time Scaling

即上面路径中的s=s(t)

#### 2.3.1 梯形速度曲线规划

- [参考1](https://www.cnblogs.com/wdzeng/p/11633512.html)
- [参考2](https://blog.csdn.net/xiaozisheng2008_/article/details/114339938)

#### 2.3.2 五次多项式插值轨迹

- [参考](https://blog.csdn.net/xiaozisheng2008_/article/details/114167606)
- [参考2](https://www.cnblogs.com/21207-iHome/p/7843517.html)
- 代码注释见：[git](https://github.com/Fernweh-yang/CodeComments_franka/tree/main/panda_tutorial/source/trajectory)

# 三、libfranka

## 1.控制逻辑

下面的示例代码来自：[git](https://github.com/Fernweh-yang/CodeComments_franka/blob/main/panda_tutorial/source/04_Panda_Cartesian_Trajectory/do_cartesian_trajectory.cpp)

1. 连接机器人：

   ```c++
   std::string robot_ip = "192.168.3.127";
   franka::Robot panda(robot_ip);
   setDefaultBehaviour(panda);
   ```

2. 读取机器人当前状态

   ```c++
   franka::RobotState initial_state = panda.readOnce();
   // 将4X4的变换矩阵，转为6维数组(3 translations, 3 RPY rotations) 
   Eigen::Vector6d initial_pose = homogeneousTfArray2PoseVec(initial_state.O_T_EE_c);
   ```

3. 设置目标位姿

   ```c++
   // 目标位姿，只是在位移translation上加了0.1
   Eigen::Vector6d targetPose = initial_pose;
   targetPose.head<3>() += Eigen::Vector3d::Constant(0.1);
   ```

4. **设计轨迹控制器**

   ```c++
   // 创建起始点到终点的轨迹
   // v_max = 0.05, a_max = 0.5 and j_max = 1e-3    
   auto traj = LinearTrajectory(initial_pose, targetPose, 0.05, 0.5, 1.e-3);
   // 创建每次迭代运行的控制器
   //1.这个控制器是写在类TrajectoryIteratorCartesianVelocity里
   //	由于类里写了函数调用运算符（Function Call Operator）operator()
   //	所以之后可以像函数一样调用实例motionIterator
   //2.c++14新引入的make_unique创建一个智能指针，动态分配资源
   // 	std::make_unique<xxx>(yyy) xxx是类的名字,y是给构造函数的参数
   auto motionIterator = std::make_unique<TrajectoryIteratorCartesianVelocity>(traj); 
   ```

   - motionIterator背后的operator()的使用参见[documentation](https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html#callback-docs:~:text=The%20following%20incomplete%20example%20shows%20the%20general%20structure%20of%20a%20callback%20function%3A)

     - 控制逻辑是：

       每次接收到一个新的机器人状态后，callback()函数即我们的operator()会计算应做出的反映。频率是1khz

5. 发送控制命令给机器人

   ```c++
   panda.control(*motionIterator, controller_mode = franka::ControllerMode::kCartesianImpedance);
   ```

   - motionIterator：即第4步控制器
   - controller_mode：用于执行动作的franka内置控制器
     - kJointImpedance关节阻抗控制
       - 关节阻抗控制是一种机器人控制方法，通过控制每个机器人关节的阻抗来实现控制。
       - 在关节空间中进行控制
     - kCartesianImpedance笛卡尔阻抗控制
       - 笛卡尔阻抗控制是另一种机器人控制方法，通过控制机器人的笛卡尔（Cartesian）位置和姿态的阻抗来实现控制。
       - 在笛卡尔空间中进行控制
