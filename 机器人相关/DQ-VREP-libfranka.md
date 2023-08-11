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

