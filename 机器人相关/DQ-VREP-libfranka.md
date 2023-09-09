# 零、学习资源汇总

- dqrobotics的[文档](https://dqroboticsgithubio.readthedocs.io/en/latest/index.html)，[论文](https://arxiv.org/abs/1910.11612)
- vrep(coppeliasim)的[文档](https://www.coppeliarobotics.com/helpFiles/index.html)

- dq-cpp-vrep: [git](https://github.com/dqrobotics/cpp-examples) 

- libfranka别人的[笔记](https://www.guyuehome.com/41074)

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
   // 用于和机器人通信(I/O)
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

# 四、FrankaRos

[知乎参考](https://zhuanlan.zhihu.com/p/433214417)

## 1. 写控制器

- franka control每次迭代都会运行controller里的update()函数

  所以要写controller.cpp

- 编写控制器的步骤

  1. 编写控制器的c++代码以及头文件
  2. 将c++类注册发布为一个插件,在XML文件中定义插件
  3. 更新package xml发布插件
  4. 编写CMakeLists.txt
  5. 编写launch file
  6. 为参数编写配置文件

- 文件夹架构

  ```shell
  workspace
  ├──	src
  |	├── franka_ros		# franka ros的官方包,不在这修改
  |	├── traj_controller	# 我们的controller所在处
  |	|	├── config		# 放6.配置文件
  |	|	├── include		# 放1.的头文件
  |	|	├── launch		# 放5.的launch文件
  |	|	├── src			# 放1.的c++代码
  |	|	├── traj_controller.xml	# 2.的插件
  |	|	├── package xml			# 3.
  |	|	├── CMakeLists.txt		# 4
  ```

### 1.1 编写控制器的c++代码以及头文件

#### traj_controller.h

```cpp
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

// 自定义控制器均来自于controller_interface::MultiInterfaceController类，允许至多声明4个接口。
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>

#include <traj_controller/franka_robot.h>

#include <traj_controller/trajectory_planning.h> // 自定义
#include <traj_controller/Trajectory.h>


DQ_robotics::DQ homogeneousTfArray2DQ(std::array<double,16> &pose);

namespace traj_controller {

class JointVelocityController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  // init()用于参数初始化以及生成接口和句柄，必须要有
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  // update()包含控制器在每个控制周期执行的代码,必须要有
  void update(const ros::Time&, const ros::Duration& period) override;
  // starting和stopping可以选择不写
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;


  // DQ_robotics::DQ_SerialManipulatorMDH kinematics();

 private:
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  ros::Duration elapsed_time_;


  // ****************** edit start ******************
  // 下面的变量都是根据具体任务慢慢加进来,不需要看,主要是上面
  // *获取机械臂状态
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  
  // *轨迹规划相关
  // PolynomialTrajectory* traj;
  LinearTrajectory* traj;
  TrajectoryIteratorCartesian* traj_Car;

  Eigen::Index iteration_index;

  double t = 0;
  double speed_factor = 0.5; 
  Vector7d t_s;   // 开始时刻
  Vector7d q_s;   // 开始时刻关节角度
  Vector7d v_s;   // 开始时刻速度
  Vector7d a_s;   // 开始时刻加速度

  Vector7d t_f;   // 结束时间
  Vector7d q_f;   // 结束时关节角度，即目标点
  Vector7d v_f;   // 结束时速度
  Vector7d a_f;   // 结束时加速度

  // *dq_robotics相关变量
  DQ_robotics::DQ x,xd;
  MatrixXd J,J_pinv,JJ;
  Vector7d u;
  // DQ_robotics::DQ_SerialManipulatorMDH fep = DQ_robotics::FrankaRobot::kinematics();
  DQ_robotics::DQ_SerialManipulatorMDH fep = DQ_robotics::FrankaEmikaPandaRobot::kinematics();
  RowVector7d goal,q_min,q_max,q_c,q;
  RowVector8d e;
  franka::RobotState robot_state;
  
  double e_norm,e_norm_old;
  int flag;
  // ****************** edit end ******************
};

}  // namespace traj_controller

```

#### traj_controller.cpp

要变成ros可以找到的controller,需要将这个文件变成一个控制器插件plugin

具体参见代码最底部

```cpp
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <traj_controller/traj_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// some parameters for the controller
double gain = -1.5;
double d = 0.1;
double k = 0.1;

// ***************** 变换矩阵转DQ ***************** 
DQ_robotics::DQ homogeneousTfArray2DQ(std::array<double,16> &pose){
    Eigen::Matrix3d rotationMatrixEigen;
    DQ_robotics::DQ r,p,x;
    rotationMatrixEigen << pose[0], pose[4], pose[8],
                           pose[1], pose[5], pose[9],
                           pose[2], pose[6], pose[10];    
    Eigen::Quaterniond quaternion(rotationMatrixEigen);
    r = DQ_robotics::DQ(quaternion.w(),quaternion.x(),quaternion.y(),quaternion.z());
    p = pose[12]*DQ_robotics::i_ + pose[13]*DQ_robotics::j_ + pose[14]*DQ_robotics::k_;
    x = r + DQ_robotics::E_*0.5*p*r;

    return x;
}

// ***************** 求矩阵的伪逆 ***************** 
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeFullU | Eigen::ComputeFullV); // For a square matrix
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);    // For a non-square matrix
    double tolerance =epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

// ***************** 控制器 ***************** 
namespace traj_controller {
    bool JointVelocityController::init(hardware_interface::RobotHW* robot_hardware,ros::NodeHandle& node_handle){
        velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
        if (velocity_joint_interface_ == nullptr) {
            ROS_ERROR(
            "JointVelocityController: Error getting velocity joint interface from hardware!");
            return false;
        }

        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR("JointVelocityController: Could not get parameter arm_id");
            return false;
        }

        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names)) {
            ROS_ERROR("JointVelocityController: Could not parse joint names");
        }
        if (joint_names.size() != 7) {
            ROS_ERROR_STREAM("JointVelocityController: Wrong number of joint names, got "<< joint_names.size() << " instead of 7 names!");
            return false;
        }
        velocity_joint_handles_.resize(7);
        for (size_t i = 0; i < 7; ++i) {
            try {
                velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
            }catch(const hardware_interface::HardwareInterfaceException& ex) {
                ROS_ERROR_STREAM("JointVelocityController: Exception getting joint handles: " << ex.what());
                return false;
            }
        }

        auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR("JointVelocityController: Could not get state interface from hardware");
            return false;
        }

        try {
            //**************** edit start ****************
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
            robot_state = state_handle_->getRobotState(); //get robotstate
            //**************** edit end ****************
            // auto state_handle = state_interface->getHandle(arm_id + "_robot");
            // std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            // for (size_t i = 0; i < q_start.size(); i++) {
            //   if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
            //     ROS_ERROR_STREAM(
            //         "JointVelocityController: Robot is not in the expected starting position for "
            //         "running this example. Run `roslaunch traj_controller move_to_start.launch "
            //         "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
            //     return false;
            //   }
            // }
        } catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM(
            "JointVelocityController: Exception getting state handle: " << e.what());
            return false;
        }

        //**************** edit start ****************
        // *读取目标下的joint space
        ROS_INFO_STREAM("xd before assigment:"<<xd);
        std::vector<double> joint_goal;
        if(node_handle.getParam("joint_goal",joint_goal)||joint_goal.size()==7){
            goal = Map<RowVector7d>(joint_goal.data(),joint_goal.size());
            ROS_INFO_STREAM("goal:"<<goal);
        }else{
            ROS_ERROR("JointVelocityController: Could not get parameter joint_goal");
            // ROS_INFO_STREAM("Jointgoal:"<<joint_goal);
            return false;
        }
        xd = fep.fkm(goal); //不能放到starting里，否则控制防盗器会挂掉
        ROS_INFO_STREAM("xd after assignment:"<<xd);

        // ! 用于轨迹规划>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        // *创建轨迹(运用motion profile)
        // robot_state = state_handle_->getRobotState(); //get robotstate

        // std::array<double, 16> T_end_c;     //读取目标EE位姿
        // std::vector<double> target_pose;
        // if(node_handle.getParam("target_pose",target_pose) && target_pose.size()==16){
        //     for(int i=0; i<16; i++){
        //         T_end_c[i] = target_pose[i];
        //         // ROS_INFO_STREAM("T_end_c"<<i <<":"<<T_end_c[i]);
        //     }       
        // }else{
        //     ROS_ERROR("JointVelocityController: Could not get parameter T_end_c");
        //     return false;
        // }

        // std::array<double, 16> T_start_c = robot_state.O_T_EE_c;    //读取当前EE位姿

        // Vector6d initial_pose = homogeneousTfArray2PoseVec(T_start_c);  // 将位姿转为(3 translations, 3 RPY rotations) 
        // Vector6d end_pose = homogeneousTfArray2PoseVec(T_end_c);
        // traj = new LinearTrajectory(initial_pose, end_pose, 0.05,0.5,1.e-3);
        // traj_Car = new TrajectoryIteratorCartesian(*traj);
        // ! 用于轨迹规划<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        //**************** edit end ****************
        return true;
    }

    void JointVelocityController::starting(const ros::Time& /* time */) {
        elapsed_time_ = ros::Duration(0.0);
        q_min << -2.8973,   -1.7628,   -2.8973,   -3.0718,   -2.8973,   -0.0175,   -2.8973;
        q_max <<  2.8973,    1.7628,    2.8973,   -0.0698,    2.8973,    3.7525,    2.8973;
        q_c = 0.5*(q_min+q_max);
        e << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; //error

        flag = 0;
        e_norm =0;
        e_norm_old=0;
    }

    void JointVelocityController::update(const ros::Time&,const ros::Duration& period) {

        // ! 用于轨迹规划>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        // // * calculate the error between EE's target pose and EE's current pose
        // std::array<double, 16> targetPose = traj_Car->getCartesianPose();
        // std::array<double, 16> currentPose = robot_state.O_T_EE_c;    
        // x = homogeneousTfArray2DQ(currentPose);
        // xd = homogeneousTfArray2DQ(targetPose);
        // e = DQ_robotics::vec8(x - xd);  

        // // * calculate the norm of the error
        // e_norm_old = e_norm;
        // e_norm = e.norm();
        // ! 用于轨迹规划<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        // * calculate the jacobian
        robot_state = state_handle_->getRobotState(); //get robotstate
        for(int i=0; i<7; i++){
            q.coeffRef(i)  = robot_state.q[i];
        }
        // ! 不用轨迹**********
        x = fep.fkm(q);
        e = DQ_robotics::vec8(x-xd);
        e_norm = e.norm();
        // ! 不用轨迹**********
        J=fep.pose_jacobian(q);                                 // 8x7
        J_pinv = pseudoInverse(J);                              // 7x8
        JJ = J_pinv*J;                                          // 7x7
        MatrixXd I = MatrixXd::Identity(JJ.rows(),JJ.cols());   // 7X7
        MatrixXd N = I-JJ;                                      // 7X7

        // * calculatet the controller
        k=0.6;
        u = -J_pinv*k*e.transpose()+N*d*(q_c.transpose()-q.transpose()); //7x1
        // u = -J_pinv*k*e.transpose();
        if(e_norm<0.001) u.setZero(); 

        // * output the command
        for (int i=0; i<7; i++) {
            velocity_joint_handles_[i].setCommand(u[i]);
        }

        // * iter step +1
        // traj_Car->step();

        // *** debug info ***
        // if(flag<100){
        //     // ROS_INFO_STREAM("iteration:"<<flag );
        //     // ROS_INFO_STREAM("x:" <<x);
        //     // ROS_INFO_STREAM("xd:" <<xd);
        //     // ROS_INFO_STREAM("q:"<<q);
        //     ROS_INFO_STREAM("e:"<<e);
        //     ROS_INFO_STREAM("e_norm:"<<e_norm);
        //     ROS_INFO_STREAM("e_norm - e_norm_old:"<<e_norm - e_norm_old);
        //     flag++;
        // }
        
    }

    void JointVelocityController::stopping(const ros::Time& /*time*/) {
        // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
        // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
        // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
    }

}  // namespace traj_controller

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// 注册为一个插件
// 注意这里的控制器类型和下面的xml文件需要保持一致
PLUGINLIB_EXPORT_CLASS(traj_controller::JointVelocityController,
          controller_interface::ControllerBase)

```

### 1.2 在XML文件中定义插件

在src文件夹下的traj_controller.xml中添加如下代码

```xml

<library path="lib/libtraj_controller">
    <class name="traj_controller/JointVelocityController" type="traj_controller::JointVelocityController" base_class_type="controller_interface::ControllerBase">
      <description>
        manipulability tracking with quadratic programming
      </description>
    </class>
  </library>
  
  
```

### 1.3 更新package xml发布插件

在src文件夹下的package.xml中添加如下代码

- 注意控制接口的名字,要和上面xml文件名一致

```xml
<?xml version="1.0"?>
<package format="2">
  <name>traj_controller</name>
  <version>0.0.0</version>
  <description>The traj_controller package</description>

  <maintainer email="xudashuai512@gmail.com">yang</maintainer>

  <license>TODO</license>

 
  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>message_generation</build_depend>
  <build_depend>eigen</build_depend>

  <build_export_depend>message_runtime</build_export_depend>

  <depend>controller_interface</depend>
  <depend>dynamic_reconfigure</depend>
  <depend>eigen_conversions</depend>
  <depend>franka_hw</depend>
  <depend>franka_gripper</depend>
  <depend>geometry_msgs</depend>
  <depend>hardware_interface</depend>
  <depend>joint_limits_interface</depend>
  <depend>tf</depend>
  <depend>tf_conversions</depend>
  <depend>libfranka</depend>
  <depend>pluginlib</depend>
  <depend>realtime_tools</depend>
  <depend>roscpp</depend>
  <depend>urdf</depend>
  <depend>visualization_msgs</depend>

  <exec_depend>franka_control</exec_depend>
  <exec_depend>franka_description</exec_depend>
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>rospy</exec_depend>

  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->
    <controller_interface plugin="${prefix}/traj_controller.xml"/>
  </export>
</package>

```

### 1.4 编写CMakeLists.txt

```c++
cmake_minimum_required(VERSION 3.0.2)
project(traj_controller)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  franka_gripper
  controller_interface
  franka_hw
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  rospy
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES traj_controller
  CATKIN_DEPENDS
    controller_interface
    eigen_conversions
    franka_hw
    franka_gripper
    geometry_msgs
    hardware_interface
    message_runtime
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 因为controller什么的都是被调用的,真正的node是franka_control
# 所以所有的代码都编译为库
add_library(traj_controller
  src/franka_robot.cpp
  src/MotionProfile.cpp
  src/Path.cpp
  src/Trajectory.cpp
  src/trajectory_planning.cpp
  src/traj_controller.cpp
)

target_link_libraries(traj_controller
  ${catkin_LIBRARIES}
  ${Franka_LIBRARIES}
  dqrobotics
)

target_include_directories(traj_controller SYSTEM PUBLIC
  include
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
```

### 1.5 编写launch file

traj_controller.launch

```xml
<?xml version="1.0" ?>
<launch>
  <arg name="robot" default="panda" doc="choose your robot. Possible values: [panda, fr3]"/>
  <arg name="arm_id" default="$(arg robot)" />
  <arg name="robot_ip" default="192.168.3.127" />
  <!-- franka_control.launch：
      1.franka_gripper包下的franka_gripper_node节点：用于连接gripper和读取gripper的状态
      2.franka_control包下的franka_control_node：连接机器人并调用我们实现的controller的update()
  -->
  <include file="$(find franka_control)/launch/franka_control.launch" pass_all_args="true"/>

  <rosparam command="load" file="$(find traj_controller)/config/controllers.yaml" subst_value="true" />
  <!-- 选择我们要用的controller，这里定义的update()会被上面的franka_control_node不停的循环调用 -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen"  args="traj_controller"/>
  <!-- <node pkg="rviz" type="rviz" output="screen" name="rviz" args="-d $(find franka_example_controllers)/launch/robot.rviz -f $(arg arm_id)_link0 \-\-splash-screen $(find franka_visualization)/splash.png"/> -->
</launch>

```

### 1.6 为参数编写配置文件

编写上面launch file中读取的配置文件controllers.yaml

```yaml
traj_controller:
    type: traj_controller/JointVelocityController
    arm_id: $(arg arm_id)
    joint_names:
        - $(arg arm_id)_joint1
        - $(arg arm_id)_joint2
        - $(arg arm_id)_joint3
        - $(arg arm_id)_joint4
        - $(arg arm_id)_joint5
        - $(arg arm_id)_joint6
        - $(arg arm_id)_joint7
    # joint_goal:
    #       - 0.560433 
    #       - 0.0209584 
    #       - 0.102085 
    #       - -1.97221 
    #       - -0.00222753 
    #       - 2.04029 
    #       - 1.55723
    joint_goal : [-0.208641,0.0507064,-0.25078,-2.01031,0.0316269,2.09578,0.330997]
    # target_pose : [0.994973,-0.094776,-0.0320448,0,-0.0944306,-0.995448,0.0121307,0,-0.0330493,-0.00904386,-0.999413,0,0.487364,0.164401,0.2741,1]
    # target_pose : [0.991908,-0.126669,0.00731729,0,-0.125947,-0.989954,-0.0641102,0,0.0153648,0.062671,-0.997916,0,0.498039,0.25038,0.344644,1]
    target_pose : [0.994282,0.0921455,0.0537907,0,0.0882803,-0.993603,0.0702838,0,0.0599241,-0.0651345,-0.996076,0,0.355753,-0.208245,0.421174,1]
    
```



## 2. 使用action来控制爪子

### 2.1 action server部分

由franka_ros自带的`franka_gripper`包提供爪子的action server

- 运行：

  ```
  roslaunch franka_gripper franka_gripper.launch robot_ip:=192.168.3.127
  ```

### 2.2 action client部分

- 一个简单的移动爪子例子

  ```c++
  #include <franka/gripper.h>
  #include <franka_example_controllers/teleop_gripper_paramConfig.h>
  #include <franka_gripper/GraspAction.h>
  #include <franka_gripper/HomingAction.h>
  #include <franka_gripper/MoveAction.h>
  #include <franka_gripper/StopAction.h>
  
  #include <actionlib/client/simple_action_client.h>
  #include <ros/init.h>
  #include <ros/node_handle.h>
  
  
  
  
  int main(int argc, char** argv){
    ros::init(argc, argv, "teleop_gripper_node");
    actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_client("/franka_gripper/grasp",true);
    actionlib::SimpleActionClient<franka_gripper::HomingAction> homing_client("/franka_gripper/homing",true);
    actionlib::SimpleActionClient<franka_gripper::MoveAction> move_client("/franka_gripper/move",true);
    actionlib::SimpleActionClient<franka_gripper::StopAction> stop_client("/franka_gripper/stop",true);
  
    // *********************** grasp ***********************
    // ROS_INFO("Waiting for action server to start.");
    // grasp_client.waitForServer();
    // stop_client.waitForServer();
    // ROS_INFO("Action server started, sending goal.");
    // // Grasp object
    // franka_gripper::GraspGoal grasp_goal;
    // // grap action的goal
    // grasp_goal.force = 40;  //N
    // grasp_goal.speed = 0.3; ///m/s
    // grasp_goal.epsilon.inner = 0.005; //m
    // grasp_goal.epsilon.outer = 0.005; //m
  
    // grasp_client.sendGoal(grasp_goal);
    // if (grasp_client.waitForResult(ros::Duration(5.0))) {
    //   ROS_INFO("teleop_gripper_node: GraspAction was successful.");
    // } else {
    //   ROS_INFO("teleop_gripper_node: GraspAction was not successful.");
    //   stop_client.sendGoal(franka_gripper::StopGoal());
    // }
  
    // *********************** move ***********************
    ROS_INFO("Waiting for action server to start.");
    move_client.waitForServer();
    stop_client.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    // Open gripper
      franka_gripper::MoveGoal move_goal;
      move_goal.speed = 0.1;  // m/s
      move_goal.width = 0.05; // m
      move_client.sendGoal(move_goal);
      if (move_client.waitForResult(ros::Duration(5.0))) {
        ROS_INFO("teleop_gripper_node: MoveAction was successful.");
      } else {
        ROS_ERROR("teleop_gripper_node: MoveAction was not successful.");
        stop_client.sendGoal(franka_gripper::StopGoal());
      }
  }
  ```

- 运行:

  ```
  rosrun franka_example_controllers teleop_gripper_node 
  ```

  

# 五、手眼标定

- 目的：求相机和机械臂末端的关系
  $$
  T_{object\_in\_base}=T_{hand\_in\_base}\cdot T_{camera\_in\_hand}\cdot T_{object\_in\_camera}
  $$

  - $T_{object\_in\_base}$：检测物体在基坐标下的位姿，固定不变
  - $T_{hand\_in\_base}$：末端执行器在基坐标下的位姿，即机械臂位姿，可以由机械臂的api直接读出
  - $T_{camera\_in\_hand}$：相机相对于末端执行器坐标系的位姿，即我们要求的手眼矩阵X
  - $T_{object\_in\_camera}$：检测物体在相机中的位姿

  根据眼在手上(eye in hand)、眼固定(eye to hand)。又有两种利用上面等式的方式：

  1. **眼在手上(eye in hand)**

     这个示意图中的$T_{Camera}^{End}$ 即上面的$T_{camera\_in\_hand}$，摘自2个文档，所以写法不一样但意思一致

     ![img](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/v2-d05680bea99b7a8d58ec1cedd535eddf_r.jpg)

  2. **眼固定(eye to hand)**

     ![img](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/%E7%9C%BC%E5%9B%BA%E5%AE%9A.jpg)

- 步骤：

  1. 机械臂已经能够按照位姿正常移动，机械臂能够正确读取末端（或者工具）末端的位姿，机械臂搭载相机成功，相机能够检测到物体，得到物体在相机中的位姿将机械臂移动到某个位姿，记录机械臂在当前位姿时末端的姿态，一般的机械臂会在示教器上存在显示，否则的话你可以使用代码进行读取
  2. 使用机械臂上的相机，采集到待检测物体在相机中的位姿，并记录
  3. 这时你会有一对位姿了，一对位姿包括：机械臂的位姿X + 机械臂在位姿X时相机采集到物体在相机中的位姿
  4. 重复1、2步骤，直到你采了10组以上（不同算法有不同的要求，但有10组数据是最基本的）
  5. 传入代码中进行计算，就能标定出手眼矩阵了

- 代码：

  ```c++
  #include <opencv2/opencv.hpp>
  #include <iostream>
  #include <math.h>
  
  using namespace std;
  using namespace cv;
  
  Mat R_T2HomogeneousMatrix(const Mat& R, const Mat& T);
  void HomogeneousMtr2RT(Mat& HomoMtr, Mat& R, Mat& T);
  bool isRotatedMatrix(Mat& R);
  Mat eulerAngleToRotateMatrix(const Mat& eulerAngle, const std::string& seq);
  Mat quaternionToRotatedMatrix(const Vec4d& q);
  Mat attitudeVectorToMatrix(const Mat& m, bool useQuaternion, const string& seq);
  //数据使用的已有的值
  //相机中13组标定板的位姿，x,y,z，rx,ry,rz,
  Mat_<double> CalPose = (cv::Mat_<double>(13, 6) <<
  	-0.072944147641399, -0.06687830562048944, 0.4340418493881254, -0.2207496117519063, 0.0256862005614321, 0.1926014162476009,
  	-0.01969337858360518, -0.05095294728651902, 0.3671266719105768, 0.1552099329677287, -0.5763323472739464, 0.09956130526058841,
  	0.1358164536530692, -0.1110802522656379, 0.4001396735998251, -0.04486168331242635, -0.004268942058870162, 0.05290073845562016,
  	0.1360676260120161, -0.002373036366121294, 0.3951670952829301, -0.4359637938379769, 0.00807193982932386, 0.06162504121755787,
  	-0.1047666520352697, -0.01377729010376614, 0.4570029374109721, -0.612072103513551, -0.04939465180949879, -0.1075464055169537,
  	0.02866460103085085, -0.1043911269729344, 0.3879127305077527, 0.3137563103168434, -0.02113958397023016, 0.1311397970432597,
  	0.1122741829392126, 0.001044006395747612, 0.3686697279333643, 0.1607160803445018, 0.2468677059920437, 0.1035103912091547,
  	-0.06079521129779342, -0.02815190820828123, 0.4451740202390909, 0.1280935541917056, -0.2674407142401368, 0.1633865613363686,
  	-0.02475533256363622, -0.06950841248698086, 0.2939836207787282, 0.1260629671933584, -0.2637748974005461, 0.1634102148863728,
  	0.1128618887222624, 0.00117877722121125, 0.3362496409334229, 0.1049541359309871, -0.2754352318773509, 0.4251492928748009,
  	0.1510545750008333, -0.0725019944548204, 0.3369908269102371, 0.2615745097093249, -0.1295598776133405, 0.6974394284203849,
  	0.04885313290076512, -0.06488755216394324, 0.2441532410787161, 0.1998243391807502, -0.04919417529483511, -0.05133193756053007,
  	0.08816140480523708, -0.05549965109057759, 0.3164905645998022, 0.164693654482863, 0.1153894876338608, 0.01455551646362294);
  
  //机械臂末端13组位姿,x,y,z,rx,ry,rz
  Mat_<double> ToolPose = (cv::Mat_<double>(13, 6) <<
  	-0.3969707, -0.460018, 0.3899877, 90.2261, -168.2015, 89.7748,
  	-0.1870185, -0.6207147, 0.2851157, 57.2636, -190.2034, 80.7958,
  	-0.1569776, -0.510021, 0.3899923, 90.225, -178.2038, 81.7772,
  	-0.1569787, -0.5100215, 0.3299975, 90.2252, -156.205, 81.7762,
  	-0.3369613, -0.4100348, 0.3299969, 90.2264, -146.2071, 71.778,
  	-0.2869552, -0.6100449, 0.4299998, 90.2271, -199.2048, 86.7806,
  	-0.2869478, -0.6600489, 0.4299948, 105.2274, -189.2053, 86.7814,
  	-0.286938, -0.6300559, 0.4299997, 75.2279, -189.2056, 86.783,
  	-0.2869343, -0.5700635, 0.2800084, 75.2291, -189.2055, 86.7835,
  	-0.1669241, -0.5700796, 0.280015, 75.2292, -189.205, 101.7845,
  	-0.236909, -0.4700997, 0.3600046, 87.2295, -196.2063, 118.7868,
  	-0.2369118, -0.6201035, 0.2600001, 87.2297, -192.2087, 75.7896,
  	-0.2468983, -0.620112, 0.359992, 97.2299, -190.2082, 80.7908);
  
  int main(int argc, char** argv)
  {
  	//数据声明
  	vector<Mat> R_gripper2base;
  	vector<Mat> T_gripper2base;
  	vector<Mat> R_target2cam;
  	vector<Mat> T_target2cam;
  	Mat R_cam2gripper = Mat(3,3,CV_64FC1);				//相机与机械臂末端坐标系的旋转矩阵与平移矩阵
  	Mat T_cam2gripper = Mat(3,1,CV_64FC1);
  	Mat Homo_cam2gripper = Mat(4,4,CV_64FC1);
  
  	vector<Mat> Homo_target2cam;
  	vector<Mat> Homo_gripper2base;
  	Mat tempR, tempT, temp;
  
  	for (int i = 0; i < CalPose.rows; i++)				//计算标定板与相机间的齐次矩阵（旋转矩阵与平移向量）
  	{		
  		temp = attitudeVectorToMatrix(CalPose.row(i), false, "");	//注意seq为空，相机与标定板间的为旋转向量
  		Homo_target2cam.push_back(temp);
  		HomogeneousMtr2RT(temp, tempR, tempT);
  		/*cout << i << "::" << temp << endl;
  		cout << i << "::" << tempR << endl;
  		cout << i << "::" << tempT << endl;*/
   		R_target2cam.push_back(tempR);
  		T_target2cam.push_back(tempT);
  	}
  	for (int j = 0; j < ToolPose.rows; j++)				//计算机械臂末端坐标系与机器人基坐标系之间的齐次矩阵
  	{
  		temp = attitudeVectorToMatrix(ToolPose.row(j), false, "xyz");  //注意seq不是空，机械臂末端坐标系与机器人基坐标系之间的为欧拉角
  		Homo_gripper2base.push_back(temp);
  		HomogeneousMtr2RT(temp, tempR,tempT);
  		/*cout << j << "::" << temp << endl;
  		cout << j << "::" << tempR << endl;
  		cout << j << "::" << tempT << endl;*/
  		R_gripper2base.push_back(tempR);
  		T_gripper2base.push_back(tempT);
  	}
  	// TSAI计算速度最快
   	// openCV提供的函数：
      calibrateHandEye(R_gripper2base,T_gripper2base,R_target2cam,T_target2cam,
                       R_cam2gripper,T_cam2gripper,CALIB_HAND_EYE_TSAI);
  
  	Homo_cam2gripper = R_T2HomogeneousMatrix(R_cam2gripper, T_cam2gripper);
  	cout << Homo_cam2gripper << endl;
  	cout << "Homo_cam2gripper 是否包含旋转矩阵：" << isRotatedMatrix(Homo_cam2gripper) << endl;
  	
  ///
  	
  	/**************************************************
  	* @note   手眼系统精度测试，原理是标定板在机器人基坐标系中位姿固定不变，
  	*		  可以根据这一点进行演算
  	**************************************************/
  	//使用1,2组数据验证  标定板在机器人基坐标系中位姿固定不变
  	cout << "1 : " << Homo_gripper2base[0] * Homo_cam2gripper * Homo_target2cam[0] << endl;
  	cout << "2 : " << Homo_gripper2base[1] * Homo_cam2gripper * Homo_target2cam[1] << endl;
  	//标定板在相机中的位姿
  	cout << "3 : " << Homo_target2cam[1] << endl;
  	cout << "4 : " << Homo_cam2gripper.inv() * Homo_gripper2base[1].inv() * Homo_gripper2base[0] * Homo_cam2gripper * Homo_target2cam[0] << endl;
  
  	cout << "----手眼系统测试-----" << endl;
  	cout << "机械臂下标定板XYZ为：" << endl;
  	for (int i = 0; i < Homo_target2cam.size(); i++)
  	{
  		Mat chessPos{ 0.0,0.0,0.0,1.0 };  //4*1矩阵，单独求机械臂坐标系下，标定板XYZ
  		Mat worldPos = Homo_gripper2base[i] * Homo_cam2gripper * Homo_target2cam[i] * chessPos;
  		cout << i << ": " << worldPos.t() << endl;
  	}
  	waitKey(0);
  
  	return 0;
  }
  
  /**************************************************
  * @brief   将旋转矩阵与平移向量合成为齐次矩阵
  * @note     
  * @param   Mat& R   3*3旋转矩阵
  * @param   Mat& T   3*1平移矩阵
  * @return  Mat      4*4齐次矩阵
  **************************************************/
  Mat R_T2HomogeneousMatrix(const Mat& R,const Mat& T)
  {
  	Mat HomoMtr;
  	Mat_<double> R1 = (Mat_<double>(4, 3) << 
  										R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
  										R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
  										R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
  										0, 0, 0);
  	Mat_<double> T1 = (Mat_<double>(4, 1) <<
  										T.at<double>(0,0),
  										T.at<double>(1,0),
  										T.at<double>(2,0),
  										1);
  	cv::hconcat(R1, T1, HomoMtr);		//矩阵拼接
  	return HomoMtr;
  }
  
  /**************************************************
  * @brief    齐次矩阵分解为旋转矩阵与平移矩阵
  * @note
  * @param	const Mat& HomoMtr  4*4齐次矩阵
  * @param	Mat& R              输出旋转矩阵
  * @param	Mat& T				输出平移矩阵
  * @return
  **************************************************/
  void HomogeneousMtr2RT(Mat& HomoMtr, Mat& R, Mat& T)
  {
  	//Mat R_HomoMtr = HomoMtr(Rect(0, 0, 3, 3)); //注意Rect取值
  	//Mat T_HomoMtr = HomoMtr(Rect(3, 0, 1, 3));
  	//R_HomoMtr.copyTo(R);
  	//T_HomoMtr.copyTo(T);
  	/*HomoMtr(Rect(0, 0, 3, 3)).copyTo(R);
  	HomoMtr(Rect(3, 0, 1, 3)).copyTo(T);*/
  	Rect R_rect(0, 0, 3, 3);
  	Rect T_rect(3, 0, 1, 3);
  	R = HomoMtr(R_rect);
  	T = HomoMtr(T_rect);
  
  }
  
  /**************************************************
  * @brief	检查是否是旋转矩阵
  * @note
  * @param
  * @param
  * @param
  * @return  true : 是旋转矩阵， false : 不是旋转矩阵
  **************************************************/
  bool isRotatedMatrix(Mat& R)		//旋转矩阵的转置矩阵是它的逆矩阵，逆矩阵 * 矩阵 = 单位矩阵
  {
  	Mat temp33 = R({ 0,0,3,3 });	//无论输入是几阶矩阵，均提取它的三阶矩阵
  	Mat Rt;
  	transpose(temp33, Rt);  //转置矩阵
  	Mat shouldBeIdentity = Rt * temp33;//是旋转矩阵则乘积为单位矩阵
  	Mat I = Mat::eye(3, 3, shouldBeIdentity.type());
  
  	return cv::norm(I, shouldBeIdentity) < 1e-6;
  }
  
  /**************************************************
  * @brief   欧拉角转换为旋转矩阵
  * @note		
  * @param    const std::string& seq  指定欧拉角的排列顺序；（机械臂的位姿类型有xyz,zyx,zyz几种，需要区分）
  * @param    const Mat& eulerAngle   欧拉角（1*3矩阵）, 角度值
  * @param
  * @return   返回3*3旋转矩阵
  **************************************************/
  Mat eulerAngleToRotateMatrix(const Mat& eulerAngle, const std::string& seq)
  {
  	CV_Assert(eulerAngle.rows == 1 && eulerAngle.cols == 3);//检查参数是否正确
  
  	eulerAngle /= (180 / CV_PI);		//度转弧度
  
  	Matx13d m(eulerAngle);				//<double, 1, 3>
  
  	auto rx = m(0, 0), ry = m(0, 1), rz = m(0, 2);
  	auto rxs = sin(rx), rxc = cos(rx);
  	auto rys = sin(ry), ryc = cos(ry);
  	auto rzs = sin(rz), rzc = cos(rz);
  
  	//XYZ方向的旋转矩阵
  	Mat RotX = (Mat_<double>(3, 3) << 1, 0, 0,
  		0, rxc, -rxs,
  		0, rxs, rxc);
  	Mat RotY = (Mat_<double>(3, 3) << ryc, 0, rys,
  		0,	  1, 0,
  		-rys, 0, ryc);
  	Mat RotZ = (Mat_<double>(3, 3) << rzc, -rzs, 0,
  		rzs, rzc, 0,
  		0, 0, 1);
  	//按顺序合成后的旋转矩阵
  	cv::Mat rotMat;
  
  	if (seq == "zyx") rotMat = RotX * RotY * RotZ;
  	else if (seq == "yzx") rotMat = RotX * RotZ * RotY;
  	else if (seq == "zxy") rotMat = RotY * RotX * RotZ;
  	else if (seq == "yxz") rotMat = RotZ * RotX * RotY;
  	else if (seq == "xyz") rotMat = RotZ * RotY * RotX;
  	else if (seq == "xzy") rotMat = RotY * RotZ * RotX;
  	else
  	{
  		cout << "Euler Angle Sequence string is wrong...";
  	}
  	if (!isRotatedMatrix(rotMat))		//欧拉角特殊情况下会出现死锁
  	{
  		cout << "Euler Angle convert to RotatedMatrix failed..." << endl;
  		exit(-1);
  	}
  	return rotMat;
  }
  
  /**************************************************
  * @brief   将四元数转换为旋转矩阵
  * @note
  * @param   const Vec4d& q   归一化的四元数: q = q0 + q1 * i + q2 * j + q3 * k;
  * @return  返回3*3旋转矩阵R
  **************************************************/
  Mat quaternionToRotatedMatrix(const Vec4d& q)
  {
  	double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
  
  	double q0q0 = q0 * q0 , q1q1 = q1 * q1 , q2q2 = q2 * q2, q3q3 = q3 * q3;
  	double q0q1 = q0 * q1 , q0q2 = q0 * q2 , q0q3 = q0 * q3;
  	double q1q2 = q1 * q2, q1q3 = q1 * q3;
  	double q2q3 = q2 * q3;
  	//根据公式得来
  	Mat RotMtr = (Mat_<double>(3, 3) << (q0q0 + q1q1 - q2q2 - q3q3), 2 * (q1q2 + q0q3), 2 * (q1q3 - q0q2),
  		2 * (q1q2 - q0q3), (q0q0 - q1q1 + q2q2 - q3q3), 2 * (q2q3 + q0q1),
  		2 * (q1q3 + q0q2), 2 * (q2q3 - q0q1), (q0q0 - q1q1 - q2q2 + q3q3));
  	//这种形式等价
  	/*Mat RotMtr = (Mat_<double>(3, 3) << (1 - 2 * (q2q2 + q3q3)), 2 * (q1q2 - q0q3), 2 * (q1q3 + q0q2),
  										 2 * (q1q2 + q0q3), 1 - 2 * (q1q1 + q3q3), 2 * (q2q3 - q0q1),
  										 2 * (q1q3 - q0q2), 2 * (q2q3 + q0q1), (1 - 2 * (q1q1 + q2q2)));*/
  
  	return RotMtr;
  }
  
  /**************************************************
  * @brief      将采集的原始数据转换为齐次矩阵（从机器人控制器中获得的）
  * @note
  * @param	  Mat& m    1*6//1*10矩阵 ， 元素为： x,y,z,rx,ry,rz  or x,y,z, q0,q1,q2,q3,rx,ry,rz
  * @param	  bool useQuaternion      原始数据是否使用四元数表示
  * @param	  string& seq         原始数据使用欧拉角表示时，坐标系的旋转顺序
  * @return	  返回转换完的齐次矩阵
  **************************************************/
  Mat attitudeVectorToMatrix(const Mat& m, bool useQuaternion, const string& seq)
  {
  	CV_Assert(m.total() == 6 || m.total() == 10);
  	//if (m.cols == 1)	//转置矩阵为行矩阵
  	//	m = m.t();	
  
  	Mat temp = Mat::eye(4, 4, CV_64FC1);
  
  	if (useQuaternion)
  	{
  		Vec4d quaternionVec = m({ 3,0,4,1 });   //读取存储的四元数
  		quaternionToRotatedMatrix(quaternionVec).copyTo(temp({0,0,3,3}));  
  	}
  	else
  	{
  		Mat rotVec;
  		if (m.total() == 6)
  		{
  			rotVec = m({ 3,0,3,1 });   //读取存储的欧拉角
  		}
  		if (m.total() == 10)
  		{
  			rotVec = m({ 7,0,3,1 });
  		}
  		//如果seq为空，表示传入的是3*1旋转向量，否则，传入的是欧拉角
  		if (0 == seq.compare(""))
  		{
  			Rodrigues(rotVec, temp({ 0,0,3,3 }));   //罗德利斯转换
  		}
  		else
  		{
  			eulerAngleToRotateMatrix(rotVec, seq).copyTo(temp({ 0,0,3,3 }));
  		}
  	}
  	//存入平移矩阵
  	temp({ 3,0,1,3 }) = m({ 0,0,3,1 }).t();
  	return temp;   //返回转换结束的齐次矩阵
  }
  
  ```

# 六、各开源库

1. [franka_pick_place](https://github.com/jc-bao/franka_pick_place)

   - 用了aruco_ros和easy_handeye来手眼标定
   - 使用moveit来控制移动和爪子的开合

2. [Pick & Place Example](https://frankaemika.github.io/docs/franka_ros.html#pick-place-example)

   - 通过来shell中发送rostopic来控制爪子的开合

     ```shell
     rostopic pub --once /franka_gripper/grasp/goal \
                  franka_gripper/GraspActionGoal \
                  "goal: { width: 0.03, epsilon:{ inner: 0.005, outer: 0.005 }, speed: 0.1, force: 5.0}"
     ```

3. [teleop_grasp](https://github.com/teleop-grasp/teleop_grasp)

   - 感觉可以借鉴

