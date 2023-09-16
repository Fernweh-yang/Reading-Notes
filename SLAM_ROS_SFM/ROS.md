# 零、ROS编程环境设置

- VS-Code找不到头文件：

  workspace目录下创建 .vscode/c_cpp_properties.json ,然后填入:

  ```json
  {
      "configurations": [
          {
              "name": "Linux",
              "includePath": [
                  "${workspaceFolder}/**",
                  "/opt/ros/noetic/include"	//主要是填入头文件所在的位置
              ],
              "defines": [],
              "compilerPath": "/usr/bin/gcc",
              "cStandard": "c17",
              "cppStandard": "gnu++14",
              "intelliSenseMode": "linux-gcc-x64"
          }
      ],
      "version": 4
  }
  ```
  
- CMakeLists.txt必须确保如下顺序：

  ```cmake
  find_package()
  catkin_package()
  include_directories()
  add_executable()
  target_link_libraries()
  ```

# 一、ROS基础

## *一些使用技巧

- 查看是否安装了某个包

  比如要查看是否安装了`joint_state_publisher`

  ```shell
  rospack list | grep 'joint_state_publisher'
  ```

- 查看机器人各个坐标系

  - 可视化ROS TF框架树

    ```
    rosrun rqt_tf_tree rqt_tf_tree
    ```

- 遇到TF_OLD_DATA ignoring data from the past 

  命令行输入
  
  ```
  rosparam set use_sim_time true
  ```
  
  
  
  

## *catkin的一些操作

- 编译的抽象程度,高到低
  - catkin: workspace(Package)会在工作空间中单独的构建每个包
  - cmake: CMakeLists.txt自己写编CMakeLists.txt来自动生成makefile
  - make: makefile通过makefile来批量的执行g++编译命令
  - gcc/g++: .cpp->.o->可执行文件

```shell
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
#会将工作空间里所有的包同时单独（isolated）编译，编译过程互不影响
#首次build要加后面一行

catkin init
#可以初始化workspace，而且初始化后，在workspace下的任何一个子目录里使用catkin工具，都相当于在workspace文件夹下使用，它会自动感知workspace。

catkin config
#可以查看workspace的结构和参数。

catkin config --merge-devel
#可以使得每个包编译得到的devel产物都在同一个devel文件夹里，当然也可以用

catkin config --isolate-devel
#使得各个包的devel分开。

catkin list
#可以查看工作空间中有哪些ros包。

catkin clean
#相当于rm -r ${build} ${devel}，但是避免了rm -r这种危险的操作！
```

## *从0搭建一个project

1. 创建工作空间:`mkdir -p ~/test_ws/src`

2. 创建包：`catkin_create_pkg test_ros_pkg std_msgs roscpp`

3. 写节点

   1. 包目录下的src目录下编写c++

   2. 包目录下的CMakeLists编写,比如添加

      ```
      add_executable(superROS src/supeROS.cpp)
      target_link_libraries(superROS ${catkin_LIBRARIES})
      ```

4. 编译：`catkin build`

   1. 首次编译后，记得在 ~/.bashrc文件尾部加入`source ~/Destktop/as_ws/devel/setup.bash`

      这样就不用每打开一个bash都要输一遍`source devel/setup.bash`

5. 重复3，4

运行:

1. 启动master:`roscore`
2. 跑节点：具体看节点那一节
3. 分析数据:
   1. rostopic list, rostopic info, rostopic echo [topic]
   2. rosnode list, rosnode info [node]
   3. rosmsg list,
   4. rospkg list,

## 1. 创建工作空间Workspace

```shell
## 创建工作空间
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
## 对于ros时Melodic版本的Python 3用户
$ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
## 激活工作空间。一个系统可以有多个工作空间，但只能有1个处于激活状态
$ source devel/setup.bash
## 检查环境变量查看当前激活的是哪个工作空间
$ echo $ROS_PACKAGE_PATH
```

- src源文件空间：放置各个功能包和一个关于这些功能包的配置文件CMakeLists.txt用于编译的配置。

- build编译空间：放置CMake和catkin编译功能包时产生的缓存、配置、中间文件等

- devel开发空间：放置编译好的可执行程序。

- 对于激活工作空间：

  ```
  ## 每一次打开终端都要输入一遍
  $ source devel/setup.bash
  
  ## 如果之后都只开发该工作空间，可以直接将它写入.bashrc文件
  $ gedit ~/.bashrc
  # 然后在该文件中末尾加入：
  source ~/Destktop/as_ws/devel/setup.bash
  ```
## 2. 创建包Package

- 一个catkin package必须具备：

  1. 包含package.xml文件
  2. 包含CMakeLists.txt文件
  3. 每一个包必须有自己的文件夹

  ```
  my_package/
    CMakeLists.txt
    package.xml
  ```

- **在catkin Workspace中可以创建多个包**。

  ```
  workspace_folder/        -- WORKSPACE
    src/                   -- SOURCE SPACE
      CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
      package_1/
        CMakeLists.txt     -- CMakeLists.txt file for package_1
        package.xml        -- Package manifest for package_1
      ...
      package_n/
        CMakeLists.txt     -- CMakeLists.txt file for package_n
        package.xml        -- Package manifest for package_n
  ```

  - 用catkin_create_pkg脚本创建新的catkin包

    用法：`$ catkin_create_pkg <package_name> [depend1] [depend2] [depend3]`

    例子：`$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp`在工作空间catkin_ws/src中创建包

- **编译catkin workspace并sorcing setup文件**

  因为roscd只能在已经被加入ros环境的地址中寻找

  ```shell
  $ cd ~/catkin_ws
  $ catkin_make
  ## 将工作空间加入ros环境
  $ source ./catkin_ws/devel/setup.bash
  ```

- **package dependencies**

  - 检查当前包直接依赖First-order dependencies

    `$ rospack depends1 beginner_tutorials `

    这些依赖被储存在package.xml中,用如下命令查看

    ```
    $ roscd beginner_tutorials
    $ cat package.xml
    ```

  - 检查间接依赖Indirect dependencies

    一个dependency通常有自己的dependecies

    `$ rospack depends1 rospy`

    直接显式包所有的依赖：直接+间接依赖

    `$ rospack depends beginner_tutorials`

### 1.1 定制自己的包package.xml

通过修改**package.xml**来customize包。

编译后生成的package.xml会在新包的文件中。比如~/Desktop/catkin_ws/src/beginner_tutoria/package.xml

1. description tag:描述这个包干什么用的

   ```xml
   <description>The beginner_tutorials package</description>
   ```

2. maintainer tags:告诉别人谁拥有这个包，并如何联系

   ```xml
   <maintainer email="you@yourdomain.tld">Your Name</maintainer>
   ```

3. license tags:这个包用的许可协议

   ```xml
   <license>BSD</license>
   ```

4. dependencies tags:**这个包用到的依赖(**最重要的功能)

   ```xml
   <!--编译工具是catkin-->
   <buildtool_depend>catkin</buildtool_depend>
    
   <!--编译时依赖的包-->
   <build_depend>roscpp</build_depend>
   <build_depend>rospy</build_depend>
   <build_depend>std_msgs</build_depend>
   
   <!--编译输出依赖的包-->
   <build_export_depend>roscpp</build_export_depend>
   <build_export_depend>rospy</build_export_depend>
   <build_export_depend>std_msgs</build_export_depend>
   
   <!--运行时依赖的包-->
   <exec_depend>roscpp</exec_depend>
   <exec_depend>rospy</exec_depend>
   <exec_depend>std_msgs</exec_depend>
   ```

5. export tags:用于向其他ROS软件包或工具提供一些特定的信息

   如此，其他包就可以方便的使用当前包定义的插件/节点/消息类型

   ```xml
   <!--指定当前包提供的插件、节点、消息类型等-->
   <export>
       <!--${prefix} 是一个变量，用于表示ROS软件包的安装路径前缀。 -->
       <!-- 这是一个controller插件 -->
       <controller_interface plugin="${prefix}/franka_example_controllers_plugin.xml" />
       
       <!-- 常规用法：-->
       <!-- 导出插件：my_plugin-->
       <plugin name="my_plugin" type="my_package/MyPlugin" />
       
       <!-- 导出节点：my_node-->
     	<node name="my_node" type="beginner_tutoria/my_node" />
   
     	<!-- 导出消息类型：MyMessageType -->
     	<message_package message="beginner_tutoria/MyMessageType" />
   </export>
   ```

   

### 1.2 源码/二进制安装包

- 两者区别

  - 二进制包是直接通过apt方式安装了ROS相关的软件包，
  - 而我们在GitHub上下载自己需要的源码来进行代码复用，编译通过生成可运行节点后的包是源码包，
  - 两者是可以共存的，但是运行时只能运行.bashrc文件中默认环境下的包。 

- 二进制安装：

  只要ros wiki中有的都可以二进制安装。下载后可直接用，但无法修改源码

  `sudo apt-get install ros-noetic-pr2-simulator`

- 源码安装：

  需要从git上下载。下载后需要编译后再运行，但可以自己做修改。

  ```shell
  cd catkin_ws/src         #进入src路径
  $ git clone https://github.com/源码包链接   #克隆软件包
  
  #同时一定不要忘记在克隆包后要安装依赖项然后进行编译：
  $ rosdep install --from-paths src --ignore-src
  $ cd ~/catkin_ws    #回到catkin工作空间
  $ catkin_make       #编译
  $ source ~/catkin_ws/devel/setup.bash 
  ```

- 如果1个包有多个版本，如何指定用哪个？

  1. 寻找当前包的地址：`rospack find move_base `

     得到`/opt/ros/noetic/share/move_base`,说明现在用的二进制安装的move_base包

  2. 想要改为使用/catkin_ws/src/navigation中的move_base包

     在.bashrc中加入source ~/catkin_ws/devel/setup.bash，即可

## 3. 构建包catkin_make

- 首先使用catkin_make 

  用法：`$ catkin_make [make_targets] [-DCMAKE_VARIABLES=...]`在catkin工作空间中用

  例子：`$ catkin_make`构建src下所有的包

  ​			`$ catkin_make -DCATKIN_WHITELIST_PACKAGES="package1;package2"`构建src下特定的包

- 编译后会有2个新的文件夹出现在workspace中

    1. **build**:the default location of the [build space](http://wiki.ros.org/catkin/workspaces#Build_Space) and is where `cmake` and `make` are called to configure and build your packages.
    2. **devel**:the default location of the [devel space](http://wiki.ros.org/catkin/workspaces#Development_.28Devel.29_Space), which is where your executables and libraries go before you install your packages.

- 如果想要构建的包在另一个src目录，比如my_src

    例子：`$ catkin_make --source my_src`

- 如果workspace/src下有多个包，但想不编译某个package。

  可以在不想被编译的package目录下新建了CATKIN_IGNORE的文件，不用写任何东东。

  返回到workspace目录下后执行catkin_make就不会编译带有CATKIN_IGNORE的package。

 ## 4. 节点node和节点管理器master

一个包里可以有多个可执行文件，可执行文件在运行之后就成了一个进程process，这个进程在ROS中就叫做节点。

通常一个node负责机器人的某一个单独的功能。

- 一些概念
  - [Nodes](http://wiki.ros.org/Nodes): A node is an executable process that uses ROS to communicate with other nodes.
  - [Messages](http://wiki.ros.org/Messages): ROS data type used when subscribing or publishing to a topic.
  - [Topics](http://wiki.ros.org/Topics): Nodes can *publish* messages to a topic as well as *subscribe* to a topic to receive messages.
  - [Master](http://wiki.ros.org/Master): Name service for ROS (i.e. helps nodes find each other)
  - [rosout](http://wiki.ros.org/rosout): ROS equivalent of stdout/stderr
  - [roscore](http://wiki.ros.org/roscore): Master(provides name service for ROS) + rosout + parameter server 
  
- `roscore`启动节点管理器**master**
  
  - 使用ROS第一步就是输入`roscore`
  - 用于节点管理，是ros节点运行的前提。
  - roscore命令会启动3个进程，可用`ps -ef|grep ros`查看进程
    - roscore(父进程启动后面2个)
    - rosmaster(节点管理器)
    - rosout(log输出管理)
  
- `rosnode`命令详细列表

  - `rosnode list`查看当前运行着的节点

    这时候返回：`/rosout`

  - `rosnode info /rosout`返回特定节点的信息
  
  - `rosnode kill /rosout` 结束某个node
  
  - `rosnode ping`测试连接节点
  
  - `rosnode machine`列出在特定机器或列表机器上运行的节点
  
  - `rosnode cleanup`清除不可到达节点的注册信息

### 4.1 简单的例子

#### 1.Publisher

```c++
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

//main function
int main(int argc, char **argv)
{

  //start node
  ros::init(argc, argv, "SupeROS");

  //create variable for node
  ros::NodeHandle n;

  //declare publishers (employees)
  ros::Publisher MrFish = n.advertise<std_msgs::String>("fish", 1000);
  ros::Publisher MrVeggies = n.advertise<std_msgs::String>("veggies", 1000);
  ros::Publisher MrFruits = n.advertise<std_msgs::String>("fruits", 1000);


  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    // declare msg for publishing data
    std_msgs::String msg_fisher;
    std_msgs::String msg_veggies;
    std_msgs::String msg_fruits;

    //define possible options for food
    std::vector<std::string> fisher_options = { "tuna", "salmon", "shark" };
    std::vector<std::string> veggies_options = { "onion", "potatoes", "carrots" };
    std::vector<std::string> fruit_options = { "bananas", "apples", "grapes" };

    //store current option
    if (count == 3){count = 0;} //reset counter if longer than amount of options
    msg_fisher.data = fisher_options[count];
    msg_veggies.data = veggies_options[count];
    msg_fruits.data = fruit_options[count];


    //publish
    MrFish.publish(msg_fisher);
    MrVeggies.publish(msg_veggies);
    MrFruits.publish(msg_fruits);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}

```

#### 2.Client

```c++
#include "ros/ros.h"
#include "std_msgs/String.h"

void customerCallback1(const std_msgs::String& msg){
        if (msg.data == "carrots")
        {
            //print in command window
            ROS_INFO("Customer 1: I got carrots!");
        }
}
void customerCallback2(const std_msgs::String& msg){
        if (msg.data == "tuna")
        {
            //print in command window
            ROS_INFO("Customer 2: I got tuna!");
        }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "Clients");

  ros::NodeHandle n;

  ros::Subscriber customer1_sub = n.subscribe("veggies", 1000, customerCallback1);
  ros::Subscriber customer2_sub = n.subscribe("fish", 1000, customerCallback2);

  ros::spin();

  return 0;
}

```

#### 3.将通信包装成一个类

- 把所有的变量都包装在一个类中
  - 一个实例的回调函数就可以调用同一个变量
  - Subscriber和Publisher也只在类实例化的时候生成一次

```c++
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"



//Class defining clients of supermarket
class SupeROS_clients{

  //Declare ROS things
  ros::NodeHandle nh_;
  ros::Subscriber customer_sub_;
  ros::Publisher customer_pub_;

  //Declare customer identifier
  float customer_identifier_;

  //Declare string variables to store desired products
  std::string desired_product_;

public:

  //Constructor
  SupeROS_clients(float customer_identifier, std::string desired_product, std::string desired_topic)
  {
    //Store class identifier
    customer_identifier_ = customer_identifier;

    //Store desired product
    desired_product_ = desired_product;

    //Define subsciber and publisher
    customer_pub_ = nh_.advertise<std_msgs::Float64>("/Money", 1);
    customer_sub_ = nh_.subscribe(desired_topic, 1000, &SupeROS_clients::customerCallback, this);
  }

  //Callback where we check if the employee offers what customers wants and pay back
  void customerCallback(const std_msgs::String& msg){
        if (msg.data == desired_product_)
        {
                //pay back
                std_msgs::Float64 coin_msg;
                coin_msg.data = 1.0;
                customer_pub_.publish(coin_msg);

                //print in command window
                ROS_INFO("Customer %f: I got %s and I paid back!", customer_identifier_, desired_product_.c_str());
        }
  }
};

//Main function
int main(int argc, char **argv)
{
        // initialize node
        ros::init(argc, argv, "Clients");
        ros::NodeHandle nh_private("~");
        
        //initialize classes
        SupeROS_clients Clients_node1(1.0, "tuna","fish"), Clients_node2(2.0, "carrots","veggies");
        
        ros::spin();
}

```

### 4.2 用`rosrun`启动单个节点

  - 命令：`rosrun [--prefix cmd] [--debug] package_name node_name [ARGS]`

    会自动寻找package下的名为EXECUTABLE的可执行程序，并将可选参数ARGS传入。

    - 例子：
    
      `$ rosrun turtlesim turtlesim_node`
      
      `$ rosrun turtlesim turtlesim_node __name:=my_turtle`给节点单独命名
      
    - 例子：在GDB(程序调试器)下运行ros程序
    
      `$ rosrun --prefix 'gdb -ex run --args' pkg_name node_name`

### 4.3 ros::NodeHandle的用法

#### 1. 自动启动和结束

`ros::NodeHandle`管理着一个内部引用数，使得开启和结束一个节点（node）可以简单地按照下面一行代码完成：

```c++
ros::NodeHandle nh;
```

#### 2. 命名空间

- 节点的命名空间有什么用

  节点的命名空间在ROS中用于隔离和组织节点内的ROS资源，如参数、话题和服务，以便不同节点可以具有相同名称的资源而不会发生冲突。

  1. **参数隔离**

     ```c++
     // 机器人1的参数
     ros::NodeHandle nh1("robot1");
     nh1.setParam("max_speed", 1.0);
     
     // 机器人2的参数
     ros::NodeHandle nh2("robot2");
     nh2.setParam("max_speed", 0.5);
     ```

     考虑两个机器人节点，每个节点都有一个名为 "max_speed" 的参数，用于设置机器人的最大速度。通过在不同的命名空间下定义这些参数，可以轻松地为每个机器人配置不同的最大速度

  2. **话题隔离**

     ```c++
     // 机器人1的激光扫描发布器
     ros::NodeHandle nh1("robot1");
     ros::Publisher pub1 = nh1.advertise<sensor_msgs::LaserScan>("laser_scan", 10);
     
     // 机器人2的激光扫描发布器
     ros::NodeHandle nh2("robot2");
     ros::Publisher pub2 = nh2.advertise<sensor_msgs::LaserScan>("laser_scan", 10);
     ```

     考虑一个多机器人系统，每个机器人都有一个激光传感器发布名为 "laser_scan" 的激光扫描话题。通过在每个机器人节点的命名空间下发布 "laser_scan" 话题，可以确保每个机器人的激光数据不会与其他机器人的数据混淆。

  3. **服务隔离**

     ```c++
     // 机器人1的移动服务服务器
     ros::NodeHandle nh1("robot1");
     ros::ServiceServer server1 = nh1.advertiseService("move", moveCallback1);
     
     // 机器人2的移动服务服务器
     ros::NodeHandle nh2("robot2");
     ros::ServiceServer server2 = nh2.advertiseService("move", moveCallback2);
     
     // 在客户端想调用机器人2的服务:通过命名空间来指定
     ros::NodeHandle nh;
     ros::ServiceClient client = nh.serviceClient<your_robot_msgs::Move>("robot2/move");
     
     ```

     考虑一个机器人系统，每个机器人都提供了一个名为 "move" 的移动服务。通过在每个机器人节点的命名空间下提供 "move" 服务，可以确保不同机器人的移动服务不会相互干扰。

- 如何创建命名空间

  `NodeHandles`可以指定一个命名空间给它的构造函数：

  ```
  ros::NodeHandle nh("my_namespace");
  ```

  这行代码将创建一个相对于`NodeHandle`的域名，`<node_namespace>/my_namespace`，而不是直接地表示为`<node_namespace>`。

  你也可以指定一个父节点的NodeHandle和命名空间，后面可以跟子节点的NodeHandle和命名空间：

  ```
  ros::NodeHandle nh1("ns1");
  ros::NodeHandle nh2(nh1, "ns2");
  ```

  这将会把`nh2`放在`<node_namespace>/ns1/ns2`的命名空间下。

- 私有命名空间

  ```c++
  // 1.～+name
  ros::NodeHandle nh("~my_private_ns");
  ros::Subscriber sub = nh.subscribe("my_private_topic", ...);  // <node_namespace>/<node_name>/my_private_ns/my_private_topic
  
  // 2.~
  ros::NodeHandle nh("~");
  ros::Subscriber sub = nh.subscribe("my_private_topic", ...);  // <node_namespace>/<node_name>/my_private_topic
  
  // 上面的node_name通过函数ros::init(argc, argv, "xx")来定义,这里node_name为xx
  // 
  ```

  

## 5. Roslaunch的使用 

### 用[roslaunch](http://wiki.ros.org/roslaunch/XML#if_and_unless_attributes)启动多个节点

用法：`$ roslaunch [package] [filename.launch]`

1. 回到工作空间`$ cd ~/catkin_ws`

2. 将当前环境导入ROS环境`source devel/setup.bash`

3. 寻找之前创建好的包`$ roscd ~/catkin_ws`

4. 创建一个launch文件夹`$ mkdir launch` 并`$ cd launch`

5. 创建一个启动文件turtlemimic.launch,输入如下代码

   ```xml
   切换行号显示
   	 <!--launch tag说明这个文件是一个launch file-->
      1 <launch>
      2 
          <!--这里开始2个命名空间为turtlesim1/2的group，都用名字为sim的turtlesim节点-->
      3   <group ns="turtlesim1">
      4     <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
      5   </group>
      6 
      7   <group ns="turtlesim2">
      8     <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
      9   </group>
     10   <!--这里开始一个叫mimic并有2个话题input/output的节点，这里会把节点顺序变为turtlesim1->mimic-> turtlesim2-->
     11   <node pkg="turtlesim" name="mimic" type="mimic">
     12     <remap from="input" to="turtlesim1/turtle1"/>
     13     <remap from="output" to="turtlesim2/turtle1"/>
     14   </node>
     15 
     16 </launch>
   ```

6. 运行这个launch file:`$ roslaunch beginner_tutorials turtlemimic.launch`

7. 在新终端给他们提供命令：`$ rostopic pub /turtlesim1/turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'`

8. 使用`$ rqt_graph`查看节点关系.

### Launch file的写法

- 例子：

  ```xml
  <!--launch tag说明这个文件是一个launch file-->
  <launch>
      <!--这里开始2个命名空间为turtlesim1/2的group，都用名字为sim的turtlesim节点-->
      <group ns="turtlesim1">
      	<node pkg="turtlesim" name="sim" type="turtlesim_node"/>
      </group>
      <group ns="turtlesim2">
          <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
      </group>
      
      <!--这里开始一个叫mimic并有2个话题input/output的节点，这里会把节点顺序变为turtlesim1->mimic-> turtlesim2-->
      <node pkg="turtlesim" name="mimic" type="mimic">
          <remap from="input" to="turtlesim1/turtle1"/>
          <remap from="output" to="turtlesim2/turtle1"/>
      </node>
      
      <!--使用配置好的rviz-->
      <!--需要先配置好后，点击file->save config as-->
      <node pkg="rviz" type="rviz" name="rviz" args="-d $(find turtlesim)/config/rviz/show_mycar.rviz" />
      
      
  </launch>
  ```

- <remap>标签

  作用
  
  1. 针对publisher自己发布的主题：**改变自己发布主题的名字**
     - from=“source_topic”: 节点中原来发布的主题名字
     - to=“target_topic”: 重映射的目标名字
  2. 针对subscriber别人发布的主题：**改变别人发布主题的名字为自己要订阅的主题名字**
     - from=“target_topic”: 节点中要订阅的主题名字
     - to=“source_topic”: 别人发布的主题名字
  
- launch文件包含的标签

    ```xml
    <launch>    	<!--根标签-->
    <node>    		<!--需要启动的node及其参数-->
    <include>    	<!--包含其他launch-->
    <machine>    	<!--指定运行的机器-->
    <env-loader>    <!--设置环境变量-->
    <param>    		<!--定义参数到参数服务器-->
    <rosparam>    	<!--启动yaml文件参数到参数服务器-->
    <arg>    		<!--定义变量-->
    <remap>    		<!--设定参数映射-->
    <group>   		<!--设定命名空间-->
    </launch>    	<!--根标签-->
    ```

- 所有的标签都可以使用if,unless

  ```xml
  <arg name="corrupt_state_estimate" default="true" />
  <!--如果参数corrupt_state_estimate值为真则执行-->
  <param if="$(arg corrupt_state_estimate)" name="drift_rw_factor" value="0.03"/>
  <!--如果参数corrupt_state_estimate值为假则执行-->
  <param unless="$(arg corrupt_state_estimate)" name="drift_rw_factor" value="0.0"/>
  ```

  一个用处：加载不同的yaml文件

  ```xml
  <launch>
    <arg name="robot" default="true"/>
    <group if="$(arg robot)">
      <node ns = "***"  pkg="***" type="***.py" name ="***" output="screen"/>
          <rosparam command="load" file="$(find path)/config.yaml" />
    </group>
    <group unless="$(arg robot)">
      <node ns = "***"  pkg="***" type="***.py" name ="***" output="screen"/>
          <rosparam command="load" file="$(find path)/***.yaml" />
    </group>
  </launch>
  ```

### 转换roslaunch和rosrun

1. **例1**

​	roslaunch中：

```xml
<node pkg="tf" type="static_transform_publisher" name="av1broadcaster" args ="1 0 0 0 0 0 1 world av1 100" />
```

​	rosrun中：

```shell
rosrun tf static_transform_publisher 1 0 0 0 0 0 1 world av1 100 __name:=av1broadcaster
```

2. **例2**：roslaunch中可以设定某个节点的启用条件，rosrun因为是人为一条条输入就没必要

```xml
<launch>
	<arg name = "static" default = "false" />
    ...
    <!--只在unless条件满足时才执行，这里默认false不执行，需要在命令行中手动输入static:=true-->
	<!--roslaunch two_drones_pkg two_drones.launch static:=true-->
    <node pkg="two_drones_pkg" type="frames_publisher_node" name="frames_publisher_node" unless="$(arg static)"/>
    ...
</launch>
```

3. **例3**:

   roslaunch中运用find快速定位文件

   ```xml
   <node name="rviz" pkg="rviz" type="rviz" args="-d $(find two_drones_pkg)/config/default.rviz"/>
   ```

   - 正常的agrs="$(find ..).."这里是没有参数-d的

     `rosrun rviz rviz --help`解释：-d用于展示这个配置文件

   rosrun中:

   ```shell
   rosrun rviz rviz -d ~/Desktop/as_ws/src/two_drones_pkg/config/default.rviz		
   ```

   

## 6. ROS通信框架

### 6.0 不同通信框架的对比

ROS的通信方式有以下四种：

- Topic 主题
- Service 服务
- Parameter Service 参数服务器
- Actionlib 动作库

#### Topic vs Services

|   名称   |              Topic               |              Service              |
| :------: | :------------------------------: | :-------------------------------: |
| 通信方式 |             异步通信             |             同步通信              |
| 实现原理 |              TCP/IP              |              TCP/IP               |
| 通信模型 |        Publish-Subscribe         |           Request-Reply           |
| 映射关系 |    Publish-Subscribe(多对多)     |      Request-Reply（多对一）      |
|   特点   | 接受者收到数据会回调（Callback） | 远程过程调用（RPC）服务器端的服务 |
| 应用场景 |       连续、高频的数据发布       |     偶尔使用的功能/具体的任务     |
|   举例   |     激光雷达、里程计发布数据     |    开关传感器、拍照、逆解计算     |

#### Services vs Action

- Action 相比 Service的优点：

  程序A(client)向程序B(server)申请实现某功能(调用函数)时：

  - 如果是Service，那A需要等待B相应后才能做其他事，因为它的通信方式是(同步通信)
  - 如果是action，A在调用B的服务后，可以不用等待结果立刻去执行其他程序。A会周期性的从B那得到状态status,结果result，和反馈feedback
  - action也可以随时发送cancel，取消正在执行的任务。

  所以action比service更**适合**执行那些**耗时长久的任务**，比如

  - **移动机器人到指定位置**：客户端发送目标位置，服务器端执行导航，同时向客户端发送进度和状态反馈，客户端可以随时取消导航任务
  - **抓取和放置操作**：客户端发送抓取或放置目标，服务器端执行臂的运动和夹取动作
  - **探测和识别**：服务器端执行探测动作，客户端可以接收实时的探测结果
  - **路径规划和轨迹执行**：服务器端计算路径，客户端可以实时获取机器人在路径上的进度。
  - **图像处理**：服务器端处理图像数据，客户端可以周期性地接收处理结果

- Action相比Service的缺点

  - **复杂性**：相对于 Services，Actions 的实现和使用相对复杂，需要定义多个消息类型（目标、反馈、结果），编写服务器端和客户端的逻辑，以及处理状态机等。
  - 由于 Action 通信需要在多个消息类型之间进行来回交互，因此在**通信开销**方面可能比 Services 稍微更高。

### 6.1 话题Topic

对于实时性、周期性的消息，使用topic来传输是最佳的选择。topic是一种点对点的单向通信方式，这里的“点”指的是node。

topic要经历下面几步的初始化过程：

1. publisher节点和subscriber节点都要到节点管理器master进行注册
2. publisher会发布topic，subscriber在master的指挥下会订阅该topic

回调callback：Subscriber接收消息会进行处理。即提前定义好了一个处理函数(代码中)

如图所示整个过程都是单向的：node1、node2两者都是各司其责，不存在协同工作，我们称这样的通信方式是**异步**的。

topic可以同时有多个subscribers，也可以有多个publishers。如/rosout,/tf等。

​	![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/ROS/topic-stru.jpg?raw=true)

- 小案例

  - 输入`$ roscore`
  - 新终端运行节点`$ rosrun turtlesim turtlesim_node`
  - 新终端运行节点`$ rosrun turtlesim turtle_teleop_key`

- 话题topics的相关命令：

  上述案例中2个节点`turtlesim_node`和`turtle_teleop_key`就是通过话题topic来实施通信的：

  1. 发布者Publisher:`turtlesim_node`  **publishing** key strokes键给topic。

  2. 接收者Subscriber:`turtle_teleop_key` 从topic那里**subscribes** 这个keystrokes。

  - `rqt_graph`

    用于展示各个节点之间的关系

    新终端输入`$ rosrun rqt_graph rqt_graph`可看到两个节点通过话题`/turtle1/cmd_vel`联系在一起

  - `rostopic`

    用于获取ros话题的消息。可用`$ rostopic -h`查看帮助文档

    - `rostopic echo topic_name`：显示某个topic的内容

      1.新终端输入`rostopic echo /turtle1/cmd_vel` 

      2.再返回`turtle_teleop_key`的终端进行控制。`rostopic`终端就会出现传输给话题的消息。

    - `rostopic list `列出当前所有的topic

    - `rostopic info topic_name`显示某个topic的属性信息
    
    - `rostopic pub topic_name ` 向某个topic发布内容
    
    - `rostopic bw topic_name`查看某个topic的贷款
    
    - `rostopic hz topic_name`查看某个topic的频率
    
    - `rostopic find topic_type` 查看某个类型的topic
    
    - `rostopic type topic_name`查看某个topic的类型`msg` 

- ROS Messages消息，即下面的msg

  各节点通信发送的都是相同类型的message。一个话题的类型是由发布者Publisher的消息message类型决定的

  - `rostopic type [topic]`

    用于返回某一个话题的数据类型

    1. `rostopic type /turtle1/cmd_vel`得到话题的数据类型为：`geometry_msgs/Twist`
    2. `rosmsg show geometry_msgs/Twist` 显式这个数据类型的具体细节
    3. 也可以直接显式具体细节`$ rostopic type /turtle1/cmd_vel | rosmsg show`

  - `rostopic pub [topic] [msg_type] [args]` 

    用于将数据发送到一个当前推荐的话题上去。

    例：`$ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'`告诉乌龟以线速度2.0,角速度1.8运动。

    - `rostopic pub`将消息发布给指定的topic
    - `-1`只发送一条消息后就退出
    - `/turtle1/cmd_vel`topic名字
    - `geometry_msgs/Twist`发送的消息数据类型，由2个数组组成，每个数组3个浮点数。
    - `--`说明后面的arguments参数都不是option。

    

  - `rostopic hz [topic]`

    用于查看节点publish的速率rate。例：`rostopic hz /turtle1/pose`

  - `rqt_plot`

    用于绘制发布在话题上的scrolling time plot滚动时间图。例`rosrun rqt_plot rqt_plot`

### **ROS msg和srv

#### msg,srv简介

- **msg**: 

  topics有严格的格式要求，这种数据格式就是message(msg)。所以msg即topic内容的数据类型，也称之为topic的格式标准。

  msg files是描述the fields of a ROS message的简单文本文件。他们以不同的语言来生成messages的源代码。储存在msg directory目录。

  - msg由一行行field type字段类型和field name字段名称组成，可用的字段类型:

    ```
    -int8, int16, int32, int64 (plus uint*)
    -float32, float64
    -string
    -time, duration
    -other msg files
    -variable-length array[] and fixed-length array[C]
    ```

  - msg还有一个特殊的类型header:包含一个timestamp时间戳和coordinate frame information坐标系信息。

  - 一个包含一个Header,一个string primitive,2个 other msg 文件

    ```
    string primitive  Header header
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
    geometry_msgs/TwistWithCovariance twist
    ```

    

- **srv**:

  区别于topic传输的数据格式由msg描述，service的数据格式由srv文件来描述。

  srv files描述了一个服务，包含2个部分：request 和 response。储存在srv directory。
  
  - src也由一行行field type 和 field name组成。request和response由"---"隔开。
  
    ```
    int64 A
    int64 B
    ---
    int64 Sum
    ```
  
    A,B是request,Sum是response

#### 使用msg

- 一些[常见msg](https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/chapter3/3.5.html)

  [ROS Message Types](http://wiki.ros.org/geometry_msgs)

- 相关命令：

  - `rosmsg list`: 列出系统上所有的msg

  - `rosmsg show msg_name`： 显示某个msg的内容

    - 例子：`$ rosmsg show beginner_tutorials/Num `

      如果忘了包名：`$ rosmsg show Num`

- 创建我们自己的msg
  
  **1.创建一个msg文件**
	
	```
	$ roscd beginner_tutorials
	$ mkdir msg
	$ echo "int64 num" > msg/Num.msg	#输出消息到一个新的文件
  ```
  
   **2.还可以`$ rosed beginner_tutorials Num.msg`,然后在里面输入**
  
    ```
    string first_name
    string last_name
    uint8 age
    uint32 score
    ```
  
  **3.编写package.xml和CMakeList**
  
  **3.1**：package.xml加入
  
  ```xml
  <!--在build time我们需要message_generation-->
  <build_depend>message_generation</build_depend>
  <!--在run time我们需要message_runtime-->
  <exec_depend>message_runtime</exec_depend>
  ```
  
  **3.2**：CMakeLists.txt
  
  ```cmake
  # 查找依赖的包
  find_package(catkin REQUIRED COMPONENTS
     roscpp	//必备
     rospy	//必备
     message_generation	//必备
     std_msgs	//我们自己写的msg中用到的其他msg
  )
  
  # 指定我们的msg文件
  add_message_files(
    FILES
    Num.msg
  )
  
  # 指定我们的msg用到的依赖项
  generate_messages(
    DEPENDENCIES
    std_msgs
  )
  
  # 设置运行依赖
  catkin_package(
   ...
    CATKIN_DEPENDS message_runtime ...
   ...)
  ```
  
- 在a_package中引用b_package里自定义的my_message.msg

  1. 首先在a_package的CMakeLists.txt文件中的find_package部分添加b_package:

     ```cmake
     find_package(catkin REQUIRED COMPONENTS
       roscpp
       rospy
       std_msgs
       b_package  #添加这一条
     )
     ```

  2. 然后在package.xml文件中添加以下代码,注意需要根据实际情况按相应的格式配置:

     ```xml
     <build_depend>b_package</build_depend>
     <build_export_depend>b_package</build_export_depend>
     <exec_depend>b_package</exec_depend>
     ```

  3. c++代码中

     ```c++
     #include <ros/ros.h>
     // b_packages::Test消息类型
     #include <b_packages/Test.h>
     ```

#### 使用srv

一些[常见srv](https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/chapter4/4.5.html)

- 相关的命令：

  - `$ rossrv show <service type>`：显示某一个服务描述

    - 例：`$ rossrv show beginner_tutorials/AddTwoInts````
  - `rossrv list`：列出所有的服务
  - `rossrv md5`：显示服务md5sum
  - `rossrv package`：列出包中的服务
  - `rossrv packages` ：列出包含服务的包

- 创建一个srv文件

  **1. 创建srv文件夹**

  ```
  $ roscd beginner_tutorials
  $ mkdir srv
  ```

  **2. 用roscp从另一个包那复制一个srv文件来**

  使用：`$ roscp [package_name] [file_to_copy_path] [copy_path]`

  例子：从rospy_tutorials包那复制它的srv到当前文件夹

  ```
  $ roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
  ```

  **3. 确保srv文件能转为c++,python或其他语言**

  ​	**3.1**: 编辑包内的package.xml,加入如下两行（当然在创建msg时已设置）

  ```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
  ```
  
  ​	**3.2**: 编辑包内的CMakeLists.txt（当然在创建msg时已设置）
  ​		(1): 将message_generation依赖加入find_package。
  
  ```
  # Do not just add this line to your CMakeLists.txt, modify the existing line
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation	#同时服务于msg,srv
  )
  ```
  
  ​		(2): 修改add_service_files
  
  ```
  add_service_files(
    FILES
    AddTwoInts.srv
  )
  ```

#### 重新编译

添加完msg和srv后需要重新编译

```
# 需要在工作空间里使用catkin_make
$ roscd beginner_tutorials
$ cd ../..
$ catkin_make
$ cd -
```

### 6.2 服务Services

区别于Topic是一种单项的异步通信方式，Service通信是双向的，它不仅可以发送消息，同时还会有反馈。

所以service包括两部分：1.请求方clinet。2.服务提供方Server

通信机制：

​	服务提供方NodeB提供一个服务接口叫/Service。请求方NodeA向服务方发送一个请求request，服务方NodeA处理后反馈给请求方一个reply。

![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/ROS/service_structure.png?raw=true)

- rosservice相关命令

  - `rosservice list`：列出节点所用的所有服务

  - `rosservice info`：打印服务信息

  - `rosservice type [service]`：打印某一个服务的类型

    比如一个/clear 服务：`$ rosservice type /spawn`。在得到turtlesim/Spawn后可用`rossrv show turtlesim/Spawn`可以查看改类型的具体定义。也可直接：`$ rosservice type /spawn | rossrv show`

  - `rosservice call [service] [args]`：使用所提供的args调用服务

    比如调用/clear服务:`$ rosservice call /clear`

    比如调用/spawn服务：`$ rosservice call /spawn 2 2 0.2 ""`

  - `rosservice args`：打印服务参数

  - `rosservice find`：按服务类型查找服务

  - `rosservice uri`：打印服务ROSPRC uri

### **参数服务器parameter server

参数服务器是节点存储参数的地方、用于配置参数，全局共享参数。参数服务器使用互联网传输，在节点管理器中运行，实现整个通信过程。

相比于topic和service，参数服务器更加静态。它维护者一个数据字典，字典里存储着各种参数和配置。

**有三种参数服务器维护方式：**

1. 命令行维护

   使用rosparam语句来进行操作各种命令

   - `rosparam set param_key param_value`：设置参数

     - 设置背景rgb中r的值为150.`$ rosparam set /turtlesim/background_r 150`

   - `rosparam get param_key`：显示参数

     - 比如rgb中的g值`$ rosparam get /turtlesim/background_g`
     - 得到参数服务器中所有参数的值`$ rosparam get /`

   - `rosparam load file_name`：从文件加载参数

     - 将参数加载到一个新的命名空间`copy_turtle``$ rosparam load params.yaml copy_turtle`

   - `rosparam dump file_name`：保存参数到文件

     - 将所有的参数存入params,yaml文件`$ rosparam dump params.yaml`

   - `rosparam delete`：删除参数

   - `rosparam list`：列出所有参数名称

   - 上面load,dump命令用到的文件是yaml文件：如

     ```yaml
     name:'Zhangsan'
     age:20
     gender:'M'
     score{Chinese:80,Math:90}
     score_history:[85,82,88,90]
     ```

2. launch文件内读写

   launch文件中有很多标签，而与参数服务器相关的标签只有两个，一个是`<param>`，另一个是`<rosparam>`

   - <param>： 一般只设置一个参数

     ```xml
     <param name="parking_x" type="double" value="100.0" />
     ```

     代码中需要：

     ```c++
     // launch中如果<param .../>在<node.../>中，那么这里的参数是私有参数
     // c++代码这里，用~表示这个参数是私有参数。
     // 从launch中读取私有参数~parking_x的值到parking_x。
     ros::param::get("~parking_x",parking_x);
     ```

   - <rosparam>：读取一个yaml文件或多个参数

     相当于用命令行维护：`rosparam load file_name`

     ```xml
     <!--Attributes:
     *** command="load|dump|delete" (optional, default=load)
     *** file="$(find pkg-name)/path/foo.yaml" (load or dump commands)
     *** param="param-name"
     *** ns="namespace" (optional)
     -->
     <rosparam command="load" file="$(find rosparam)/example.yaml" />
     <rosparam command="delete" param="my/param" />
     <rosparam param="a_list">  [1, 2, 3, 4]  </rosparam>
     <rosparam>
       a: 1
       b: 2
     </rosparam>
     <!--Attributes:
     *** subst_value=true|false (optional):
     Allows use of substitution args in the YAML text.
     下面这个标签会使用内置的$(find ...)来寻找yaml里的叫whitelist的string，并替换它的value为default
     -->
     <arg name="whitelist" default="[3, 2]"/>
     <rosparam param="whitelist" subst_value="True">
       $(arg whitelist)
     </rosparam>
     ```

3. node源码:

   利用api来对参数服务器进行操作

### 6.3 动作库Action

一些常见的[动作库](https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/chapter4/4.6.html)

类似service通信机制，actionlib也是一种请求响应机制的通信方式，actionlib主要弥补了service通信的一个不足，就是当机器人执行一个长时间的任务时，假如利用service通信方式，那么publisher会很长时间接受不到反馈的reply，致使通信受阻。

所以actionlib则可以比较适合实现长时间的通信过程，actionlib通信过程可以随时被查看过程进度，也可以终止请求，这样的一个特性，使得它在一些特别的机制中拥有很高的效率。

**原理**：

客户端会向服务器发送目标指令和取消动作指令,而服务器则可以给客户端发送实时的状态信息,结果信息,反馈信息等等,从而完成了service没法做到的部分.

![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/ROS/actionlib.png?raw=true)

![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/ROS/action_interface.png?raw=true)

**Action规范**

利用动作库进行请求响应，动作的内容格式应包含三个部分，目标、反馈、结果。

- 目标

  机器人执行一个动作，应该有明确的移动目标信息，包括一些参数的设定，方向、角度、速度等等。从而使机器人完成动作任务。

- 反馈

  在动作进行的过程中，应该有实时的状态信息反馈给服务器的实施者，告诉实施者动作完成的状态，可以使实施者作出准确的判断去修正命令。

- 结果

  当运动完成时，动作服务器把本次运动的结果数据发送给客户端，使客户端得到本次动作的全部信息，例如可能包含机器人的运动时长，最终姿势等等。

- Action规范文件的后缀名是.action，内容格式如下：

  ```
  # Define the goal
  uint32 dishwasher_id  # Specify which dishwasher we want to use
  ---
  # Define the result
  uint32 total_dishes_cleaned
  ---
  # Define a feedback message
  float32 percent_complete
  ```

**编译需要修改CmakeLists.txt和package.xml**

- CmakeLists.txt:

  ```cmake
  find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
  
  add_action_files(DIRECTORY action FILES DoDishes.action) generate_messages(DEPENDENCIES actionlib_msgs)
  
  add_action_files(DIRECTORY action FILES Handling.action)
  
  generate_messages( DEPENDENCIES actionlib_msgs)
  ```

- package.xml

  ```xml
  <build_depend>actionlib </build_depend>
  <build_depend>actionlib_msgs</build_depend>
  <run_depend>actionlib</run_depend>
  <run_depend>actionlib_msgs</run_depend>
  ```

## 7. 时间Time和时长Duration

- time由2种
  - 通常ROS使用pc系统的clock作为time source(**wall time**)
  - 对于仿真，使用**simulated time**更方便。使用simulated time需要
    - set the `/use_sim_time` parameter
    - publish the time on the topic `/clock` from
      - Gazebo(enabled by default)
      - ROS bag(use option --clock)

- 需要头文件`#include <ros/time.h>`和`#include <ros/duration.h>`

  time指的是某个时刻，duration指的是某个时段

  ```c++
  // 获取当前时间
  ros::Time begin=ros::Time::now();
  
  //定义类对象
  //_sec是秒，_nsec是纳秒
  ros::Time::Time(uint32_t _sec, uint32_t _nsec)
  ros::Time::Time(double t)
  ros::Duration::Duration(uint32_t _sec, uint32_t _nsec)
  ros::Duration::Duration(double t)
      
  //示例
  ros::Time time1(5,20000); 			//将实例time1时间定为5s+20000ns
  ros::Time start(ros::Time::now());  //将实例start时间定为当前时间
  ros:Duration one_hour(60*60,0);     //将实例one_hour时间段定为1小时
  double secs1=time1.toSec();         //将 Time类的实例 转为 double 型时间
  double secs2=one_hour.toSec();      //将 Duration类的实例 转为 double 型时间
  ```

- 睡眠sleep()和频率rate()

  - bool ros::Duration::sleep()

    ```c++
    //睡半秒
    ros::Duration(0.5).sleep();
    ```

  - ros::Rate

    ```c++
    // 以10hz的频率来执行while循环
    // 即循环程序运行时间不足0.1s时，会睡满0.1s
    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
      ... do some work ...
      r.sleep();
    }
    ```

- 定时器Timer

  ```c++
  #include "ros/ros.h"
  
  void callback(const ros::TimerEvent&)
  {
    ROS_INFO("Callback triggered");
  }
  
  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    // Timer可以以一定的频率来循环的调用callback()回调函数。
    // 这里的频率是10hz,即0.1s
    ros::Timer timer1 = n.createTimer(ros::Duration(0.1), callback);
    ros::spin();
    return 0;
  }
  ```

- 时间和时长的运算

  - 1 hour + 1 hour = 2 hours (duration + duration = duration)
  - 2 hours - 1 hour = 1 hour (duration - duration = duration)
  - Today + 1 day = tomorrow (time + duration = time)
  - Today - tomorrow = -1 day (time - time = duration)
  - Today + tomorrow = error (time + time is undefined)



## 8. 调试(rqt_console)

- 使用`rqt_console`和`rqt_logger_level`   debug

  `rqt_concole`依附于ROS的logging framework来展现节点的输出

  `rqt_logger_level`允许改变在节点运行时改变他们的verbosity level详细等级(DEBUG,WARN,INFO and ERROR)

  在2个新终端分别启动：

  ```
  #新终端
  $ rosrun rqt_console rqt_console
  #新终端
  $ rosrun rqt_logger_level rqt_logger_level
  ```

  然后会出现两个window来显示报错等信息。

  最后再运行要debug的节点。

- logger level

  ```
  #按优先权高低排列：
  Fatal	有最高优先权
  Error	
  Warn
  Info
  Debug	有最低优先权
  # 设置为哪一个等级，我们就会得到它和更高等级的信息
  # 比如设置为Warn,则会得到Warn,Error,Fatal的消息
  ```

## 9. [ROS Bags](http://wiki.ros.org/rosbag)

在 ROS 系统中，可以使用 bag 文件来保存和恢复系统的运行状态，比如录制雷达和相机话题的 bag 包，然后回放用来进行联合外参标定。

[知乎](https://zhuanlan.zhihu.com/p/151444739)

- A bag is a format for storing message data

  - Binary format with file extension *.bag
  - Suited for logging and recording datasets for later visualization and analysis

- 一些指令

  - Record all topics in a bag：`rosbag record --all`

  - Record topics：`rosbag record topic_1 topic_2`

  - Show information about a bag：`rosbag info bag_name.bag`

  - Read a bag and publish its content：`rosbag play bag_name.bag`

    可以设置playback的一些参数`rosbag play --rate=0.5 bag_name.bag`

    - ---rate=factor: puiblish rate factor
    - --clock: publish the clock time
    - --loop: loop playback

    

- 可以用rqt_bag来调试

## 10. 编辑rosed

可以直接在一个包内直接用vim打开一个文件来编辑，而不需要具体的地址

使用：`$ rosed [package_name] [filename]`

例子：`$ rosed roscpp Logger.msg`

技巧：`$ rosed [package_name] <tab><tab>`

​			如果不知道具体的文件名，可以按两下tab显式这个包下所有的文件名。

## 11. Debugging Strategies

- Compile and run code often to catch bugs early
- Understand compilation and runtime error messages
- Use analysis tools to check data flow (rosnode info, rostopic echo, roswtfm rqt_graph, etc.)
- Visualize and plot data (Rviz, RQT Multiplot, etc.)
- Divide program into smaller stepps and check intermediate results (ROS_INFO, ROS_DEBUG, etc.)
- Extend and optimize only once a basic version works。扩展的时候从最简单基本的一点点扩展
- If things don’t make sense, clean your workspace
  - `catkin clean --all`因为可能有些东西需要rebuild
- Build in debug mode and use GDB or Valgrind
  - `catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug`
- Maintain code with unit tests 单元测试and integration tests组装测试


## 12. Topic:写一个简单的Publisher 和 Subscriber

### 12.1 使用c++来写

#### 1. 写一个Publisher Node

- 创建一个cpp文件

  ```
  $ roscd beginner_tutorials	#来到要实现某功能的包
  $ mkdir -p src				#这个文件夹放所有包
  $ touch talker.cpp			#创建cpp文件
  ```

- 然后将[1.1](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)的代码拷贝入talker.cpp
  
  需要做的事情：
  
  1. 初始化ROS系统
  2. 将关于chatter主题的std_msgs/String消息发送给master
  3. 每0.1秒循环发布一条消息到chatter
  
  ```c++
  #include "ros/ros.h"			//包含ROS系统所需要的大部分头文件
  #include "std_msgs/String.h"	//包含字符串类型的msg
  #include <sstream>
  
  int main(int argc, char **argv)
  {
    //初始化ros,允许通过命令行进行name remapping
    //也是定义我们节点名字的地方"talker"
    ros::init(argc, argv, "talker");	
    //创建当前进程节点的句柄Handle
    ros::NodeHandle n;	
  
   
    // 告诉master，将要发送一个std_msgs/String类型的message到话题chatter。
    // 这时master会让所有节点接听来自chatter将要发送的消息。
    // 第二个参数是发布队列的大小，如果发布的太快，会在丢弃旧消息前保存1000条。
    // NodeHandle::advertise()返回一个ros::Publisher对象。提供2个功能：
    //	1.包含一个publish()方法，可以发送消息到创建的topic上
    // 	2.当它超过范围，它会自动停止发送
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    
    // ros::Rate对象object 允许设置一个想要的循环频率，这里是0.1s/10hz
    // 追踪上次调用的ros::sleep()后过了多久来保证正确的睡眠时间
    // 发送消息的频率为10hz,一秒发10条消息，周期为1/10s
    ros::Rate loop_rate(10);
  
  
    // roscpp文件自带SIGINT,可以提供诸如ctrl+c这样的信号。
    // ros::ok()如下情况会返回false：
    //	1.收到一个SIGINT信号
    //	2.被另一个同名的节点踢出了网络
    //	3.ros::shutdown()被程序的其他部分调用
    //	4.所有的ros::NodeHandles被销毁了  
    int count = 0;
    while (ros::ok())
    {
  
      std_msgs::String msg;
      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();
  	
      //可以在终端打印该消息
      ROS_INFO("%s", msg.data.c_str());
      //将消息发送给master,所有的subscriber会接收到这个msg
      chatter_pub.publish(msg);
  	
      // 用于调用callback()函数
      // subscriber接收的消息并不是立刻就调用回调函数的，只有等到spin()或spinOnce()执行的时候才会被调用
      // spin()在调用后，不再运行后面的程序，所以一般不出现在循环中。仅仅知识相应topic时使用。
      // spinOnce()在调用后，还会继续执行后面的程序。除了调用回调函数还要做其他重复工作时使用。
      ros::spinOnce();	
       
      // 靠sleep()函数来保证连续2个信号之间的时间恰好为一个周期
      loop_rate.sleep();	
      ++count;
    }
  
  
    return 0;
  }
  ```
  

#### 2. 写一个Subscriber Node

- 创建一个cpp文件

  ```
  touch listener.cpp
  ```

- 将[2.1](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)的代码复制过去

  需要做的事情

  1. 初始化ros系统
  2. subscribe订阅chatter话题
  3. spin()循环，等待消息的到达
  4. 当消息到达，调用chatterCallback

  ```c++
  #include "ros/ros.h"
  #include "std_msgs/String.h"
  
  
  // 这是一个Callback函数，当chatter话题收到一个新message的时候会被调用。
  // 这些消息在boost shared_ptr中传递，所有我们可以根据需要将他存储起来。
  void chatterCallback(const std_msgs::String::ConstPtr& msg)
  {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
  }
  
  int main(int argc, char **argv)
  {
  
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    
    
      
    // 从master那订阅chatter话题。ROS会在每次有新消息到达时，调用chatterCallback()函数。
    // 第二个参数是消息队列的大小，以防我们处理消息的速度不够快时可以先把消息保存起来。超过1000条消息的时候会在新消息到达时删除旧消息。
    // NodeHandle::subscribe()返回一个ros::Subscriber对象。这个对象必须被hold on保留，直到我们不想订阅为止。
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  
    // ros::spin()会进入一个循环，并尽可能块的调用message callbacks.
    // 当ros::ok()返回错误的时候，ros::spin()也会跟着自动停止。
    // 用于调用callback()函数
    // 用于调用callback()函数
    // subscriber接收的消息并不是立刻就调用回调函数的，只有等到spin()或spinOnce()执行的时候才会被调用
    // spin()在调用后，不再运行后面的程序，所以一般不出现在循环中。仅仅知识相应topic时使用。
    // spinOnce()在调用后，还会继续执行后面的程序。除了调用回调函数还要做其他重复工作时使用。
    ros::spin();
  
    return 0;
  }
  ```

#### 3. 构建building上面写好的节点

- 打开CMakeLists.txt

  ```
  $ source ./catkin_ws/devel/setup.bash
  #将包加入ros环境后使用rosed
  $ rosed beginner_tutorials CMakeLists.txt
  ```

- 编辑CMakeLists.txt

  将如下代码加入文件末尾

  ```
  add_executable(talker src/talker.cpp)
  # 作为Groovy常规,可以使用下面的变量去依赖所有必要的目标
  target_link_libraries(talker ${catkin_LIBRARIES})
  # 确保包的消息头文件在使用之前先被调用。如果消息来自工作空间内的其他包，需要给他们各自的
  # generation targets添加依赖项，因为catkin会并行的编译所有projects.
  add_dependencies(talker beginner_tutorials_generate_messages_cpp)
  
  add_executable(listener src/listener.cpp)
  target_link_libraries(listener ${catkin_LIBRARIES})
  add_dependencies(listener beginner_tutorials_generate_messages_cpp)
  ```

- 运行catkin_make

  ```
  # In your catkin workspace
  $ cd ~/catkin_ws
  $ catkin_make  
  ```

### 12.2 使用python来写

#### 1. 写一个Publisher Node

- 创建文件

  ```shell
  $ roscd beginner_tutorials
  $ mkdir scripts				##scripts文件夹放py脚本，src文件夹放c++程序。
  $ cd scripts
  ##教程上的publisher下下来
  $ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py
  $ chmod +x talker.py		##给talker.py文件加权限，x表示可执行
  ```

- 代码

  ```python
  #!/usr/bin/env python				每个ROS节点都会有这么一行，保证脚本是被用作python script
  # license removed for brevity
  import rospy						
  from std_msgs.msg import String		
  
  def talker():
      #说明这个节点是用来给chatter话题传输String类型的消息的。queue_size是当subsriber接收速度不够快时，排队消息的数量。
      pub = rospy.Publisher('chatter', String, queue_size=10)
      #初始化，且定义该节点名为talker.
      #anonymous=True通过在talker后加随机数来确保节点名是唯一的，因为一个ROS系统中不能出现同名节点，否则会被刷掉。
      rospy.init_node('talker', anonymous=True)
      rate = rospy.Rate(10) # 10hz
      #rospy.is_shutdown()用于判断是否存在需要停止程序的情况，比如ctrl+c。
      while not rospy.is_shutdown():
          hello_str = "hello world %s" % rospy.get_time()
  		"""
  		rospy.loginfo(str)具有3个功能triple-duty
  		1. 将消息输出到屏幕
  		2. 将消息写入Node's log file
  		3. 将消息写给rosout.rosout是一个handy方便的debug工具
  		"""
          rospy.loginfo(hello_str)
          #发布消息给chatter话题
          pub.publish(hello_str)
          rate.sleep()
  
  if __name__ == '__main__':
      try:
          talker()
      except rospy.ROSInterruptException:
          pass
  ```

  - 并把py脚本加入CMakeLists.txt,以保证脚本正确的被加载和编译

    用`rosed beginner_tutorials CMakeLists.txt`打开后加入：

    ```
    catkin_install_python(
    	PROGRAMS scripts/talker.py
      	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    ```

#### 2. 写一个Subscriber Node

- 创建一个py文件

  这里直接从教材下载一个subscriber

  ```shell
  $ roscd beginner_tutorials/scripts/
  $ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py
  $ chmod +x listener.py
  ```

- 代码：

  ```python
  #!/usr/bin/env python
  import rospy
  from std_msgs.msg import String
  
  def callback(data):
      rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
      
  def listener():
  
      # In ROS, nodes are uniquely named. If two nodes with the same
      # name are launched, the previous one is kicked off. The
      # anonymous=True flag means that rospy will choose a unique
      # name for our 'listener' node so that multiple listeners can
      # run simultaneously.
      rospy.init_node('listener', anonymous=True)
  	
      # 当subscriber从chatter话题那收到string类型的消息时，会调用callback
      rospy.Subscriber("chatter", String, callback)
  
      # spin() simply keeps python from exiting until this node is stopped
      rospy.spin()
  
  if __name__ == '__main__':
      listener()
  ```

  - 同样将subsrciber加入到CMakeLists.txt

    ```
    catkin_install_python(
    	PROGRAMS scripts/talker.py scripts/listener.py
     	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    ```

#### 3. building上面写好的节点

```
$ cd ~/Desktop/catkin_ws
$ catkin_make
```

### 12.3 测试Publisher和Subscriber

1. `$ roscore`

2. 每开一个终端都要`$ source ./devel/setup.bash `

   也可以直接在~/.bashrc中加入想要用的工作空间的/devel/setup.bash

3. 新终端中运行发布者

   ```
   $ rosrun beginner_tutorials talker      (C++)
   $ rosrun beginner_tutorials talker.py   (Python) 
   ```

4. 新终端中运行接收者

   ```
   $ rosrun beginner_tutorials listener     (C++)
   $ rosrun beginner_tutorials listener.py  (Python) 
   ```



## 13. Servics:写一个简单的Server和Client

### 13.1 使用C++来写

#### 1. 写一个Server Node

​	功能：接收2个ints后返回他们的和

- 创建一个cpp文件

  ```shell
  $ cd ~/Desktop/catkin_ws/src/beginner_tutorials/src
  $ touch add_two_ints_server.cpp
  ```

- 将[1.1](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)的代码复制过来

  这里头文件#include "beginner_tutorials/AddTwoInts.h"在**[四、创建ROS msg和srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv)**时创建

  ```c++
  #include "ros/ros.h"
  #include "beginner_tutorials/AddTwoInts.h"
  
  //callback function会在收到request后，将request作为在自己的参数（这里的req）
  //该函数提供了两个整数相加的服务，它接收 srv 文件中定义的请求和响应类型，并返回一个布尔值。
  bool add(beginner_tutorials::AddTwoInts::Request  &req,
           beginner_tutorials::AddTwoInts::Response &res)
  {
    //Response &res是回复给client的值
    res.sum = req.a + req.b;
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
  }
  
  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle n;
    //创建一个服务add_two_ints，用于广播advertise服务。接收到请求request后会调用add方法
    ros::ServiceServer service = n.advertiseService("add_two_ints", add);
    ROS_INFO("Ready to add two ints.");
    ros::spin();
  
    return 0;
  }
  ```

#### 2. 写一个Client Node

- 创建一个cpp文件

  ```
  $ cd ~/Desktop/catkin_ws/src/beginner_tutorials/src
  $ touch add_two_ints_client.cpp
  ```

- 将[2.1](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)的代码复制过来

  ```c++
  #include "ros/ros.h"
  #include "beginner_tutorials/AddTwoInts.h"
  #include <cstdlib>
  
  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "add_two_ints_client");
    if (argc != 3)
    {
      ROS_INFO("usage: add_two_ints_client X Y");
      return 1;
    }
  
    ros::NodeHandle n;
    /*
    创建一个add_two_ints service服务的client客户
    ros::ServiceClient对象用来呼叫服务
    */
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
    //instantiate实例化一个autogenerated service class.
    beginner_tutorials::AddTwoInts srv;
    //service类包含2个成员request和response。AddTwoInts服务类有2个request值    
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
      
    //client.call(srv)呼叫服务service。如果成功返回true,并返回response值。如果失败返回false,此时response值无意义。
    if (client.call(srv))
    {
      //AddTwoInts服务类有1个reponse值 
      ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }
  
    return 0;
  }
  ```
  

#### 3. 构建节点并运行

- 将server和client加入CMakeLists.txt

  加在末尾：

  ```
  add_executable(add_two_ints_server src/add_two_ints_server.cpp)
  target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
  add_dependencies(add_two_ints_server beginner_tutorials_gencpp)
  
  add_executable(add_two_ints_client src/add_two_ints_client.cpp)
  target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
  add_dependencies(add_two_ints_client beginner_tutorials_gencpp)
  ```

- 编译

  ```
  # In your catkin workspace
  cd ~/Desktop/catkin_ws
  catkin_make
  ```

- 运行
  - `roscore`
  - 新终端打开服务`rosrun beginner_tutorials add_two_ints_server`
  - 新终端打开客户`$ rosrun beginner_tutorials add_two_ints_client 1 3`

### 13.2 使用python来写

#### 1. 写一个ServerNode

- Code

  ```python
  from __future__ import print_function
  # beginner_tutorials是包的名字
  # AddTwoInts定义在上面笔记msg,srv中
  # 这两个request和response值都在同一个srv中由---上下隔开
  from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
  import rospy
  
  # 回调函数的参数是请求对象，返回值是响应对象
  def handle_add_two_ints(req):
      print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
      return AddTwoIntsResponse(req.a + req.b)
  
  def add_two_ints_server():
      rospy.init_node('add_two_ints_server')
    	# 定义服务节点名称，服务的类型，处理函数。
      s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
      print("Ready to add two ints.")
      rospy.spin()
  
  if __name__ == "__main__":
      add_two_ints_server()
  ```

#### 2.  写一个Client Node

- Code

  ```python
  from __future__ import print_function
  
  import sys
  import rospy
  from beginner_tutorials.srv import *
  
  def add_two_ints_client(x, y):
      # 等候服务add_two_ints可用
      rospy.wait_for_service('add_two_ints')
      try:
          # 呼叫服务add_two_ints
          add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
          # 像正常函数一样调用服务
          # 返回的对象是AddTwoInts服务的object
          resp1 = add_two_ints(x, y)
          # 返回的对象是AddTwoInts服务的Response值 
          return resp1.sum
      except rospy.ServiceException as e:
          print("Service call failed: %s"%e)
  
  def usage():
      return "%s [x y]"%sys.argv[0]
  
  if __name__ == "__main__":
      # Client中不用初始化节点init_node
      if len(sys.argv) == 3:
          x = int(sys.argv[1])
          y = int(sys.argv[2])
      else:
          print(usage())
          sys.exit(1)
      print("Requesting %s+%s"%(x, y))
      print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
  ```

#### 3. 构建节点并运行

- 将如下内容加入CMakeLists.txt

  ```cmake
  catkin_install_python(PROGRAMS scripts/add_two_ints_server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  catkin_install_python(PROGRAMS scripts/add_two_ints_server.py scripts/add_two_ints_client.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
  ```


## 14. Action:写一个简单的Server和Client

[参考](http://wiki.ros.org/actionlib_tutorials/Tutorials)

### 14.0 创建action message

- 创建一个包actionlib_tutorials:

  ```
  mkdir -p ~Desktop/catkin_ws/src
  cd Desktop/catkin_ws/src
  catkin_create_pkg actionlib_tutorials actionlib message_generation roscpp rospy std_msgs actionlib_msgs
  ```

- 定义Action Messages(goal,result,feedback)

  - action message由[.action](http://wiki.ros.org/actionlib#:~:text=the%20requested%20scan.-,.action%20File,-The%20action%20specification)文件自动生成，action通信有3个通道：

    1. **Goal：** Goal 通道用于发布目标请求，它表示你希望机器人或系统完成的任务。通常，一个 Goal 会包含任务的一些参数，例如移动到特定位置、执行特定动作等。Goal 通道的消息在发送后，会等待机器人或系统完成任务并发送回应。
    2. **Result：** Result 通道用于发布任务的结果，它表示任务的完成状态和可能的结果。一旦任务完成，机器人或系统会发布一个 Result 消息，通知客户端任务的结果。Result 是周期性的，一次任务完成后会发送一次，表示整个任务的结果。
    3. **Feedback：** Feedback 通道用于提供任务的中间反馈信息，它允许客户端获取任务进展的实时信息。例如，如果机器人执行路径规划任务，Feedback 通道可以周期性地发布机器人当前的位置，以便客户端可以实时监测机器人的进度。

  - 在`/home/yang/Desktop/catkin_ws/src/actionlib_tutorials/action`文件夹下创建Fibonacci.action文件，并输入:

    ```
    #goal definition
    int32 order
    ---
    #result definition
    int32[] sequence
    ---
    #feedback
    int32[] sequence
    ```

- 修改CMakeLists.txt

  用于在make构建过程中，自动生成message file.

  在cmake文件中添加下面的1，2，3，4

  ```cmake
  cmake_minimum_required(VERSION 3.0.2)
  project(actionlib_tutorials)
  
  # 1.使用find_package宏来添加actionlib和actionlib_msgs包
  find_package(catkin REQUIRED COMPONENTS
      actionlib
      actionlib_msgs
      message_generation
      roscpp
      rospy
      std_msgs
  )
  
  # 2.使用add_action_files宏来声明我们想要生成的action
  add_action_files(
      DIRECTORY action
      FILES Fibonacci.action 
  )
  
  # 3.使用generate_messages宏来添加依赖的message包
  generate_messages(
      DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
  )
  
  # 4.将actionlib_msgs包添加到catkin_package宏
  catkin_package(
      CATKIN_DEPENDS actionlib_msgs
  )
  
  
  ## Specify additional locations of header files
  ## Your package locations should be listed before other locations
  include_directories(
  # include
      ${catkin_INCLUDE_DIRS}
  )
  
  ```

- 修改xml

  因为是在run time时生成message，所以加入下面这一行

  ```xml
  <exec_depend>message_generation</exec_depend>
  ```

- 最后`catkin_make`就可以生成actionfiles了

  ```shell
  $ cd ../.. # Go back to the top level of your catkin workspace
  $ catkin_make
  $ ls devel/share/actionlib_tutorials/msg/
  FibonacciActionFeedback.msg  FibonacciAction.msg        FibonacciFeedback.msg
  FibonacciResult.msg          FibonacciActionGoal.msg    FibonacciActionResult.msg  FibonacciGoal.msg
  $ ls devel/include/actionlib_tutorials/
  FibonacciActionFeedback.h  FibonacciAction.h        FibonacciFeedback.h  FibonacciResult.h
  FibonacciActionGoal.h      FibonacciActionResult.h  FibonacciGoal.h
  ```

### 14.1 使用C++来写

#### 1. 写一个Server

创建actionlib_tutorials/src/fibonacci_server.cpp 

```c++
#include <ros/ros.h>
// actionlib/server/simple_action_server.h is the action library used from implementing simple actions.
#include <actionlib/server/simple_action_server.h>
// 包含从Fibonacci.action中生成的action message ，该.h文件由FibonacciAction.msg自动生成
#include <actionlib_tutorials/FibonacciAction.h>

class FibonacciAction
{
// protected成员允许派生类访问基类的一些内部状态，同时还限制了这种访问只在派生类内部有效
protected:

    ros::NodeHandle nh_;
    // *********************** 创建action server ***********************
    actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // 创建feedback和result message
    actionlib_tutorials::FibonacciFeedback feedback_;
    actionlib_tutorials::FibonacciResult result_;

public:
    // *********************** FibonacciAction动作类的构造函数 ***********************
    /*用类的构造函数创造一个名为FibonacciAction的动作服务器（Action Server）
    成员函数初始化：
        as_：上面定义的动作服务器对象
            nh_：节点句柄（NodeHandle），它是ROS中与节点通信的接口
            name： 动作服务器的名称，用于标识不同的动作
            boost::bind(&FibonacciAction::executeCB, this, _1)：创建了一个回调函数，当客户端请求执行动作时，会调用executeCB函数，并且传递一个参数给它
                boost::bind()：的作用是将一个成员函数（或普通函数）与其参数绑定成一个新的函数对象 
                &FibonacciAction::executeCB：是一个成员函数指针，表示FibonacciAction类的成员函数executeCB
                this：指向当前对象（即FibonacciAction对象）的指针
                _1：是一个占位符，表示在调用这个函数对象时会提供一个参数
            false：表示不自动启动动作服务器
        action_name_：存储动作服务器的名称
    */
    FibonacciAction(std::string name) : 
        as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false), 
        action_name_(name)
    {   
        ROS_INFO("Succeed to create action");
            
        // ！！！！之前在初始化列表中设置了不自动启动，现在通过调用start()来手动启动服务器！！！！
        // 这之后action server就对外提供服务了
        as_.start();
    }

    // *********************** FibonacciAction动作类的析构函数 ***********************
    ~FibonacciAction(void){}

    // *********************** FibonacciAction动作类的回调函数 ***********************
    /*回调函数
        ROS中，动作消息类型的共享指针类型通常是通过将消息类型名称的末尾添加 'ConstPtr' 来定义的。
        这种方式在内存管理方面更为安全，避免了不必要的复制，同时允许多个对象共享同一数据。
        这里goal message指针是一个boost共享指针
    */ 
    void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(1);
        bool success = true;

        // push_back the seeds for the fibonacci sequence
        // 这里的sequence的数据结构类似于 std::vector<int32_t>，但是使用了特定的内存分配器来进行内存管理
        feedback_.sequence.clear();
        feedback_.sequence.push_back(0);
        feedback_.sequence.push_back(1);

        // publish info to the console for the user
        ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

        // start executing the action
        for(int i=1; i<=goal->order; i++)
        {   
            // action的重要机制：允许client发送cancel命令，来结束当前任务。
            // 检查clien是否有请求 preempt先占 
            if (as_.isPreemptRequested() || !ros::ok())
            {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            break;
            }
            // 如果没有请求，就继续算fibonacci数列，也就是前两个数字之和
            feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
            // 将feedback值发送到action的feedback channel
            // Feedback 通道用于提供任务的中间反馈信息，它允许客户端获取任务进展的实时信息。
            as_.publishFeedback(feedback_);
            // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep();
        }
        // 如果client没有发送取消指令
        if(success)
        {   
            // 最后的结果就是feedback的值
            result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }

};


int main(int argc, char** argv)
{   
    // ros::init()初始化节点，fibonacci为节点名字
    ros::init(argc, argv, "fibonacci");
    // 创建action
    FibonacciAction fibonacci("fibonacci");
    // 循环运行，提供动作服务，等待client的请求
    ros::spin();

    return 0;
}
```

#### 2. 写一个Client

```c++
#include <ros/ros.h>
// actionlib/client/simple_action_client.h is the action library used from implementing simple action clients.
#include <actionlib/client/simple_action_client.h>
// terminal_state定义了可能的goal states
#include <actionlib/client/terminal_state.h>
// 包含由Fibonacci.action创建的所有action messages
#include <actionlib_tutorials/FibonacciAction.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_fibonacci");

    // 创建action client，有2个参数
    // 第一个参数是要连接的action server名字，
    // 第二个参数是bool值，定义是否自动spin a thread。即是否在创建 Action Client 时启动一个独立的线程来处理事件循环，从而允许你的节点继续执行其他操作
    actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);

    ROS_INFO("Waiting for action server to start.");
    // action server可能不在运行状态，所以先等待action server被启动
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // 将goal，这里的20，发送给action server
    actionlib_tutorials::FibonacciGoal goal;
    // 注意这里变量名order，和变量类型int32都是在Fibonacci.action中定义的
    goal.order = 20;
    ac.sendGoal(goal);

    // wait for the action to return
    // 这里设置了，如果等待30S都没回复就认为goal失败了。
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
    {
        // 如果失败了，取消当前任务
        ROS_INFO("Action did not finish before the time out.");
        ac.cancelGoal();
    }

    //exit
    return 0;
}
```



### 14.2 使用python来写

### 14.3 编译和运行

在七(零)的cmake基础上修改cmake为：

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(actionlib_tutorials)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES Fibonacci.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(fibonacci_server src/fibonacci_server.cpp)

target_link_libraries(
  fibonacci_server
  ${catkin_LIBRARIES}
)

add_dependencies(
  fibonacci_server
  ${actionlib_tutorials_EXPORTED_TARGETS}
)
```

- 创建4个terminal，分别运行
  1. `roscore`
  2. `rosrun actionlib_tutorials fibonacci_server`
  3. `rosrun actionlib_tutorials fibonacci_client`
  4. `rqt_graph`

## 15. ROS文件系统Filesystem Concepts

- **Packages**: Packages are the software organization unit of ROS code. Each package can contain libraries, executables可执行文件, scripts脚本, or other artifacts代码生成物.

- **Manifests**: A manifest is a description of a *package*. It serves to define dependencies between *packages* and to capture meta information about the *package* like version, maintainer, license, etc...

### Filesystem Tools

注意：所有的ros工具只会找到在[ROS_PACKAGE_PATH](http://wiki.ros.org/ROS/EnvironmentVariables#ROS_PACKAGE_PATH)中列出的ros包。可用`$ echo $ROS_PACKAGE_PATH`查看

- **rospack**

  用于找packages的信息。

  用法：`$ rospack find [package_name]`

  例子：`$ rospack find roscpp`

- **roscd**

  rosbash套件的一部分，用于前往packages的目录directory。

    用法：`$ roscd <package>[/subdir]`

    例子：`$ roscd roscpp`  前往roscpp包所在的目录，可用`pwd`查看当前工作目录。

    			`$ roscd roscpp/cmake  `前往roscpp包所在的目录中cmake子目录。

  - `roscd log`如果运行过ros程序后，可以查看日志文件

- **rosls**

  相当于ls，直接显式包里的内容

  	用法：`$ rosls <package-or-stack>[/subdir]`
  	
  	例子：`rosls roscpp_tutorials`

# 二、ROS进阶

## \#、Autonomous Systems课程相关:

### Lab2:Task2

- 运用tf，进行坐标系frame转换

```c++
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <cmath>

class FramesPublisherNode{
 //创建节点句柄nh
 ros::NodeHandle nh;
 ros::Time startup_time;

 ros::Timer heartbeat;
 //1.定义2个广播broadcaster br1和br2
 tf::TransformBroadcaster br1;	
 tf::TransformBroadcaster br2;

 public:
  FramesPublisherNode(){
    // NOTE: This method is run once, when the node is launched.
    startup_time = ros::Time::now();
    heartbeat = nh.createTimer(ros::Duration(0.02), &FramesPublisherNode::onPublish, this);
    heartbeat.start();
  }

  void onPublish(const ros::TimerEvent&){
    ros::Duration E_time = ros::Time::now() - startup_time;
    double time = E_time.toSec();

    //2. 声明2个变量用来分别存储2个无人机的转换信息
    tf::Transform AV1World(tf::Transform::getIdentity());
    tf::Transform AV2World(tf::Transform::getIdentity());

    //3. 设置坐标转换，（1.，2.，3.）为子坐标系av1在父坐标系world坐标系中的坐标，
    AV1World.setOrigin(tf::Vector3(std::cos(time), std::sin(time), 0));
    AV2World.setOrigin(tf::Vector3(std::sin(time), 0, std::cos(2*time)));
	
    // 4.1 定义无人机1号的旋转
    tf::Quaternion q1;
    q1.setRPY(0,0,time);//（0，0，time）av1坐标系在world坐标下的roll(绕X轴)，pitch(绕Y轴)，yaw(绕Z轴) 的旋转度数.
    AV1World.setRotation(q1);
      
    // 4.2 定义无人机2号的旋转
    tf::Quaternion q2;
    q2.setRPY(0,0,0);//（0，0，0）av2在world坐标系下的roll(绕X轴)，pitch(绕Y轴)，yaw(绕Z轴) 的旋转度数，现在都是0度
    AV2World.setRotation(q2);

	//将变换广播出去，发布了world和无人机1号av1，2号av2之间的坐标关系
    br1.sendTransform(tf::StampedTransform(AV1World, ros::Time::now(),"world", "av1"));
    br2.sendTransform(tf::StampedTransform(AV2World, ros::Time::now(),"world", "av2"));
  }
};

int main(int argc, char** argv){
  //初始化ROS，节点名为frames_publisher_node
  ros::init(argc, argv, "frames_publisher_node");
  FramesPublisherNode node;
  ros::spin();
  return 0;
}

```



## \#、ROS中TF的使用

- 简介：

  - TF(TranssForm)坐标转换，包括位置和姿态2个方面。**要注意区分坐标转换和坐标系frame的转换**

    `坐标转换是一个坐标在不同坐标系下的表示，而坐标系转换不同坐标系的相对位姿关系`

  - tf是一个树状结构，维护坐标系之间的关系，靠**话题通信机制**来持续地发布不同link(比如手部、头部、某关节)之间的坐标关系。

    作为树状结构，要保证父子坐标系都有某个节点在持续地发布他们之间的位姿关系，才能使树状结构保持完整。只有父子坐标系的位姿关系能被正确的发布，才能保证任意两个frame之间的连通。

  - 每两个相邻frame之间靠节点发布它们之间的位姿关系，这种节点称为**broadcaster**。broadcaster就是一个发布器publisher,如果两个frame之间发生了相对运动，broadcaster就会发布相关消息。
  
- 分析

  - tf树的信息：`rosrun tf tf_monitor`
  - 2个frame坐标系之间转换的信息:`rosrun tf tf_echo source_frame target_frame`
  -  可视化tf tree:`rosrun tf view_frames`


### 1. 用c++写一个tf broadcaster(subscriber)

- 创建包

  ```
   $ cd %YOUR_CATKIN_WORKSPACE_HOME%/src
   $ catkin_create_pkg learning_tf tf roscpp rospy turtlesim
   
   $ cd ..
   $ catkin_make
   $ source ./devel/setup.bash
  ```

- 创建一个.cpp文件

  ```
  $ roscd learning_tf
  $ cd src
  $ touch turtle_tf_broadcaster.cpp
  ```

  前往包下src处创建.cpp文件并输入:

  ```c++
  #include <ros/ros.h>
  #include <tf/transform_broadcaster.h>
  #include <turtlesim/Pose.h>
  
  std::string turtle_name;
  
  void poseCallback(const turtlesim::PoseConstPtr& msg){
    //创建一个TransformBroadcaster实例，用于发送transformations转换
    static tf::TransformBroadcaster br;
    //创建一个Transform实例，存储转换信息
    tf::Transform transform;
    //将小乌龟的初始坐标设置为坐标原点。小乌龟坐标是2维的，所以这里z是0.0
    transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
    //创建一个Quaternion实例，用于存储坐标系绕各轴旋转的角度
    tf::Quaternion q;
    //roll(绕X轴)，pitch(绕Y轴)，yaw(绕Z轴) 的旋转度数
    q.setRPY(0, 0, msg->theta);
    transform.setRotation(q);
    //将坐标系转换发送出去。第1个参数:转换信息。2：给正在发布的转换一个时间戳。3：父坐标的名称。4：子坐标的名称
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
  }
  
  int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_broadcaster");
    if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
    turtle_name = argv[1];
  
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);
  
    ros::spin();
    return 0;
  };
  ```

- 运行

  1. 用`rosed learning_tf CMakeLists.txt`打开编译文件

     末尾加入：

     ```
     add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
     target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES})
     ```

  2. 工作空间目录下编译

     `$ catkin_make`

  3. 在learn_ws/src/learning_tf/launch下创建.launch文件

     ```xml
     <launch>
         <!-- Turtlesim Node-->
         <node pkg="turtlesim" type="turtlesim_node" name="sim"/>
     
         <node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen"/>
         <!-- Axes -->
         <param name="scale_linear" value="2" type="double"/>
         <param name="scale_angular" value="2" type="double"/>
     
         <node pkg="learning_tf" type="turtle_tf_broadcaster"
               args="/turtle1" name="turtle1_tf_broadcaster" />
         <node pkg="learning_tf" type="turtle_tf_broadcaster"
               args="/turtle2" name="turtle2_tf_broadcaster" />
     
     </launch>
     ```
     
  4. 运行:

     ` $ roslaunch learning_tf start_demo.launch`

   5. 新终端中检查结果

      ` $ rosrun tf tf_echo /world /turtle1`

### 2. 用c++写一个tf listener(client)

- 创建包：

  ```shell
   $ roscd learning_tf
   $ cd src
   $ touch turtle_tf_listener.cpp
  ```

- 复制[1.1](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29)如下代码

  ```c++
  #include <ros/ros.h>
  #include <tf/transform_listener.h>
  #include <geometry_msgs/Twist.h>
  #include <turtlesim/Spawn.h>
  
  int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_listener");
  
    ros::NodeHandle node;
    
    //等待一个叫spawn的服务可用。
    ros::service::waitForService("spawn");
    //创建一个服务spawn的客户add_turtle
    ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
    //实例化一个自动生成的服务类。参见创建ROS msg和srv。
    turtlesim::Spawn srv;
    //呼叫服务
    add_turtle.call(srv);
  	
    //创建一个Publisher实例，将消息发送到给话题cmd_vel
    ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
    
    //实例化一个TransformListener对象。一旦被建立，就会开始接收tf transformations
    tf::TransformListener listener;
  
    ros::Rate rate(10.0);
    while (node.ok()){
      tf::StampedTransform transform;
      try{
        /*
  	  向listener请求一个特定转换
  	  1.想要从坐标/turtle1到/turtle2的转换
  	  2.我们想什么时候进行转换。Time(0)最近一个可以进行转换的时刻。
  	  3.transform储存产生的转换
  	  
  	  参照笔记 3.添加一个坐标frame
  	  如果把/turtle1改为/carrot1，这时乌龟2号就开始追着坐标系carrot1跑，而不是追着乌龟1号跑了
        */
        listener.lookupTransform("/turtle2", "/turtle1",
                                 ros::Time(0), transform);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
  	
      //实例化一个消息类。参见创建ROS msg和srv
      geometry_msgs::Twist vel_msg;
      vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                      transform.getOrigin().x());
      vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                    pow(transform.getOrigin().y(), 2));
      turtle_vel.publish(vel_msg);
  
      rate.sleep();
    }
    return 0;
  };
  ```

- 运行：

  1. `rosed learning_ws CMakeLists.txt`

     加到最后

     ```
     add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
     target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})
     ```

  2. 编译`$ catkin_make`

     这会在devel/lib/learning_tf文件夹生成一个二进制文件turtle_tf_listener.

  3. 将这个二进制文件加入start_demo.launch

     ```xml
     <launch>
     ...
     <node pkg="learning_tf" type="turtle_tf_listener"
           name="listener" />
     </launch>
     ```

  4. 运行.launch文件

     ```
      $ roslaunch learning_tf start_demo.launch
     ```

### 3. 用c++添加一个坐标frame

- 创建文件

  ```shell
  $ touch ~/Desktop/learn_ws/src/learning_tf/src frame_tf_broadcaster.cpp
  ```
  
- 写入代码

  ```c++
  #include <ros/ros.h>
  #include <tf/transform_broadcaster.h>
  
  int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_broadcaster");
    ros::NodeHandle node;
  
    tf::TransformBroadcaster br;
    tf::Transform transform;
  
    ros::Rate rate(10.0);
    while (node.ok()){
      /*
      设置坐标系位置转换.子坐标carrot1在父坐标turtle1的右边2单位
      这时我们创建的子坐标相对父坐标的位置是不随时间变化的，如果想要让它随时间变化：
      transform.setOrigin( tf::Vector3(2.0*sin(ros::Time::now().toSec()), 2.0*cos(ros::Time::now().toSec()), 0.0) );
      */
      transform.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
      //设置坐标系角度转换。Quaternion四元数
      transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
      //发送转换。从父坐标turtle1转到子坐标carrot1.
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "turtle1", "carrot1"));
      rate.sleep();
    }
    return 0;
  };
  ```
  
- 运行
  
  1. 加到`rosed learning_ws CMakeLists.txt`底部
  
     ```
     add_executable(frame_tf_broadcaster src/frame_tf_broadcaster.cpp)
     target_link_libraries(frame_tf_broadcaster ${catkin_LIBRARIES})
     ```
  
  2. 编译`catkin_make`
  
  3. 加入`rosed learning_ws start_demo.launch`
  
     ```xml
     <launch>
         ...
         <node pkg="learning_tf" type="frame_tf_broadcaster"
               name="broadcaster_frame" />
     </launch>
     ```
  
  4. 运行` $ roslaunch learning_tf start_demo.launch`
  

### 4.tf和time

- 问题：想要在特定时刻进行转移。

  在笔记2.xx中客户**`src/turtle_tf_listener.cpp`**：
  
  ```c++
  try{
    listener.lookupTransform("/turtle2", "/turtle1",  
	                           ros::Time(0), transform);
  ```

	这里用的Time(0)是最近一个可以进行转动的时刻，如果想在特定时刻，比如立刻马上：
	
	```c++
	try{
		listener.lookupTransform("/turtle2", "/turtle1",  
	                         ros::Time::now(), transform);
	```

​		但这里会出现问题，因为每一个listener有一个buffer缓冲器用来存储来自所有tf broadcaster的坐标转换信息。但每一个broadcaster将信息发过来，都需要几毫秒的时间。所以这里用Time::now()立刻进行转移，就会发生在收到转换信息之前，从而出错。
- 解决：使用waitForTransform

  先wait然后look up是否有转换信息传过来。2个函数必须用同一个Time实例now，不然时间戳对不上。

  ```c++
  try{
      ros::Time now = ros::Time::now();
      listener.waitForTransform("/turtle2", "/turtle1",
                                now, ros::Duration(3.0));
      listener.lookupTransform("/turtle2", "/turtle1",
                               now, transform);
  ```
  
  - waitForTransform四个参数：
  
    1. 等待从这个frame: /turtle2
    2. 转到这个frame:     /turtle1
    3. 在这个时间点
    4. 最大等待的时间
### 5.用tf进行time travel

- 问题：让乌龟2到乌龟1 五秒之前在的地方。

  同样修改**`src/turtle_tf_listener.cpp`**：

  ```c++
    try{
      ros::Time now = ros::Time::now();
      ros::Time past = now - ros::Duration(5.0);
      listener.waitForTransform("/turtle2", now,
                                "/turtle1", past,
                                "/world", ros::Duration(1.0));
      listener.lookupTransform("/turtle2", now,
                               "/turtle1", past,
                               "/world", transform);
  ```

  - 此时的lookupTransform有6个参数
    1. 本次转移从该坐标frame
    2. 转移的时间
    3. 要转移到的目标坐标frame
    4. 什么时候的目标坐标位置
    5. 一个不随时间变化的参考坐标
    6. 存储所有的转换信息

  

## \#、读取YAML文件

- **在包内创建一个YAML文件**

  包的结构

  ```
  /include          : Cpp headers
  /src              : Cpp code
  /scripts          : Python code
  /config           : YAML files
  /launch           : Launch files
  CMakeLists.txt
  package.xml
  ```

- **编写yaml文件**

  ```yaml
  text: "Hello"
  number_int: 42 
  number_float: 21.3
  enable_boolean: true
  list_of_elements:
      - 1
      - 2
      - 3
      - 4
  dictionary: {
      another_text: "World",
      another_number: 12,
  }
  ```

- **从yaml文件下载参数params到Parameter Server**

  - 用命令行`rosparam load my_params.yaml`下载

    可以用`rosparam list`查看parameter里已有的键(属性)

    可以用`rosparam dump dump.yaml`将Parameter Server里的键值对反向下载到yaml文件里

  - 用launchfile来下载

    ```xml
    <launch>
        <rosparam file="$(find my_custom_package)/config/my_params.yaml" />
    </launch>
    
    #最后结果类似
     * /list_of_anything: [1, 2, 3, 4]
     * /number_float: 21.3
     * /number_int: 42
    ```
    
    - 可以直接用find命令来找到相应包下的那个yaml文件
    
  - 如果想要给所有的参数增加一个prefix前缀
    
    ```xml
    <launch>
        <group ns="custom_prefix">
            <rosparam file="$(find my_custom_package)/config/my_params.yaml" />
        </group>
    </launch>
    
    #最后结果类似
     * /custom_prefix/list_of_anything: [1, 2, 3, 4]
     * /custom_prefix/number_float: 21.3
     * /custom_prefix/number_int: 42
    ```
  
- **在代码中使用这些参数**
  
  比如要用上面例子中的`* /custom_prefix/number_float: 21.3`这一参数
  
  - python:
  
    `rospy.get_param("/custom_prefix/number_float")`
  
  - C++:
  
    ```
    ros::NodeHandle nh;
    double number_to_get;
    nh.getParam("/custom_prefix/number_float", number_to_get);
    ```
  
# 三、ROS图像处理工具

## 1. cv_bridge

- [cv_bridge](http://wiki.ros.org/cv_bridge)用来在sensor_msgs/Image和numpy两种格式之间进行转换，使OpenCV能够处理Topic中的图像数据。

## 2. image_transport

- [image_transport](http://wiki.ros.org/image_transport)可以将Image数据重新转发到新的topic中，其输入可以是Topic、图片、视频

[Depth_image_proc](http://wiki.ros.org/depth_image_proc)

## 3. image_pipeline

  [image_pipeline](http://wiki.ros.org/image_pipeline)是ROS的图像处理工具包，包括以下几个部分：

### 3.1 camera_calibration

- [camera_calibration](http://wiki.ros.org/camera_calibration)：摄像头标定包

### 3.2 image_proc

- [image_proc](http://wiki.ros.org/image_proc)：图像校正包。主要用来处理rgb图片，提供node、nodelet两种运行方式。
- image_proc 还提供了四个nodelet：
  - debayer：将image转换成灰度、彩色两个版本并输出
  - rectify：校正图像
  - crop_decimate：图像抽样，即将图像的像素减小
  - resize：调整图像大小

### 3.3 stereo_image_proc

- [stereo_image_proc](http://wiki.ros.org/stereo_image_proc)：处理双目相机

### 3.4 image_view和stereo_view

- [image/stereo_view](http://wiki.ros.org/image_view)：可视化

### 3.5 depth_image_proc

[depth_image_proc](http://wiki.ros.org/depth_image_proc)：处理深度相机，主要用来处理深度图像，其所有的功能通过**nodelet**来提供：

所有的nodelets全都支持 standard floating point depth images and OpenNI-specific uint16 depth image。所以在使用OpenNI相机时，可以使用unit16raw topics来节省cpu cycles循环。

- 可以生成depth images的技术：

  - The [Kinect](http://wiki.ros.org/openni_kinect) and related devices
  - Traditional stereo cameras
  - Time-of-flight cameras
  
- [一个演示](http://wiki.ros.org/openni_launch)：

  实现了将raw depth/RGB/IR streams 转化为convert to depth images, disparity images, and (registered) point clouds.

#### 3.5.1**convert_metric**：

量测值变换，将raw unit16 depth image从mm转为float depth image的m为。

- Subscribed Topics: 

  - `image_raw` ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

    `uint16` depth image in mm, the native OpenNI format.
- Published Topics: 

  - `image` ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

    `float` depth image in m, the recommended format for processing in ROS.

#### 3.5.2**disparity**：

将深度图重变为disparity格式（disparity是一种视差图，可以通过双目相机生成）

- Subscribed Topics有2个：

  - `left/image_rect` ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

    Rectified修正的 depth image.

  - `right/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html))

    Camera calibration and metadata. Must contain the baseline, which conventionally is encoded in the right camera P matrix.

- Published Topics:

  - `left/disparity` ([stereo_msgs/DisparityImage](http://docs.ros.org/en/api/stereo_msgs/html/msg/DisparityImage.html))

    Disparity image (inversely相反的 related to depth), for interop交互 with stereo processing nodes. For all other purposes use depth images instead.

- Parameters

  - `min_range` (`double`, default: 0.0)

    Minimum detectable可察觉的 distance.

  - `max_range` (`double`, default: +Inf)

     Maximum detectable distance.

  - `delta_d` (`double`, default: 0.125)

     Smallest allowed disparity increment,视察增量 which relates to the achievable depth range resolution分辨率. Defaults to 1/8 pixel.

  - `queue_size` (`int`, default: 5)

     Size of message queue for synchronizing同时 subscribed topics.

#### 3.5.3**point_cloud_xyz**：

将深度图转换成xyz点云图像

- Subscribed Topics

  - `camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html))

    Camera calibration and metadata.

  - `image_rect` ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

    Rectified depth image

- Published Topics

  - `points` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))

    XYZ point cloud. If using [PCL](http://wiki.ros.org/pcl_ros), subscribe as `PointCloud<PointXYZ>`.

- Parameters

  - `queue_size` (`int`, default: 5)

    Size of message queue for synchronizing subscribed topics.


#### 3.5.4**point_cloud_xyzrgb**：

将深度图和RGB合成，并转换成xyzrgb点云图像

- Subscribed Topics

  - `rgb/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html))

    Camera calibration and metadata.

  - `rgb/image_rect_color` ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

    Rectified校正过的 color image.

  - `depth_registered/image_rect` ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

    Rectified depth image, registered to the RGB camera.

- Published Topics

  - `depth_registered/points` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))

    XYZRGB point cloud. If using [PCL](http://wiki.ros.org/pcl_ros), subscribe as `PointCloud<PointXYZRGB>`.

- Parameters

  - `queue_size` (`int`, default: 5)

    Size of message queue for synchronizing subscribed topics.


#### 3.5.5**register**：

将深度相机的frame-id变换到另一个坐标系中。

- Subscribed Topics

  - `rgb/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html))

     RGB camera calibration and metadata.

  - `depth/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html))

     Depth camera calibration and metadata.

  - `depth/image_rect` ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

     Rectified depth image.
  
- Published Topics

  - `depth_registered/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html))

     Camera calibration and metadata. Same as `rgb/camera_info` but time-synced to `depth_registered/image_rect`.

  - `depth_registered/image_rect` ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

     Reprojected depth image in the RGB camera frame.
  
- Parameters

  - `queue_size` (`int`, default: 5)

    Size of message queue for synchronizing subscribed topics.

- Required tf Transforms

  - /depth_optical_frame` → `/rgb_optical_frame

    The transform between the depth and RGB camera optical frames as specified in the headers of the subscribed topics (rendered表现 here as `/depth_optical_frame` and `/rgb_optical_frame`).

### 3.6 Nodelet

- 参考：
  - [知乎](https://zhuanlan.zhihu.com/p/37537823)
  - [官网](http://wiki.ros.org/nodelet)

- Nodelet提供了一种方法，可以在同一台计算机上，在同一个进程内，运行多个算法，且在进程内消息传递时**不产生复制成本（zero copy)**。在一个node里面，roscpp利用指针传递可以实现在publish和subscribe调用时的零拷贝。为了实现相似的效果，多个nodelets允许将多个类动态加载到同一个node里，同时还提供独立的命名空间，从而使得这些nodelets尽管运行在同一个进程里，但却仍然像单独的node一样工作。也就实现了“在一个进程（node）里运行多个nodelet”的效果。

  因此，大通量数据流可能包含多个nodelet，此时若将他们加载到同一个进程里，就可以避免数据拷贝和网络传输。从而在**传输大量数据时避免了传输耗时长的问题**。

- 大多数图像相关包都支持node和nodelet两种模式，使用下面节点查看系统中所有可以用nodelet的包

  ```
  rosrun nodelet declared_nodelets
  ```

- [运行一个Nodelet](http://wiki.ros.org/nodelet/Tutorials/Running%20a%20nodelet)

### 3.7执行所需的launch file

例子：运行一个 'point_cloud_xyz'  nodelet

```xml
<launch>
  <node pkg="nodelet" type="nodelet" name="nodelet_manager" args="manager" />

  <node pkg="nodelet" type="nodelet" name="nodelet1"
        args="load depth_image_proc/point_cloud_xyz nodelet_manager">
    <remap from="camera_info" to="/camera/depth/camera_info"/>
    <remap from="image_rect" to="/camera/depth/image_rect_raw"/>
    <remap from="points" to="/camera/depth/points"/>
  </node>
</launch>
```

##   4.[Octomap](http://wiki.ros.org/octomap)

### 4.1 [octomap_server](http://wiki.ros.org/octomap_server)

- 一个launch file例子：

  ```xml
  <launch>
    <node pkg="octomap_server" type="octomap_server_node" name="octomap_server">
      <!-- resolution in meters per pixel -->
      <param name = "resolution" value = "0.15" />
  
      <!-- name of the fixed frame, needs to be "/map" for SLAM -->
      <!-- 静态全局地图的 frame_id，但在增量式构建地图时，需要提供输入的点云帧和静态全局帧之间的 TF 变换 -->
      <param name = "frame_id" type = "string" value = "camera_init" />
  
      <!-- set min to speed up! -->
      <param name = "sensor_model/max_range" value = "15.0" />
  
      <!-- 机器人坐标系 base_link，滤除地面需要该 frame -->
      <!-- <param name = "base_frame_id" type = "string" value = "base_link" /> -->
  	
      <param name="pointcloud_min_z" type="double" value="0"/>
      <!-- filter ground plane, distance value should be big! 项目并不需要滤除地面 -->
  	<!-- <param name = "filter_ground" type = "bool" value = "true" /> -->
      <!-- <param name = "ground_filter/distance" type = "double" value = "1.0" /> -->
      <!-- 分割地面的 Z 轴阈值 value 值 -->
  	<!-- <param name = "ground_filter/plane_distance" type = "double" value = "0.3" /> -->
  
      <!-- 直通滤波的 Z 轴范围，保留 [-1.0, 10.0] 范围内的点 -->
      <!-- <param name = "pointcloud_max_z" type = "double" value = "100.0" /> -->
      <!-- <param name = "pointcloud_min_z" type = "double" value = "-1.0" /> -->
  
      <!-- <param name = "filter_speckles" type = "bool" value = "true" /> -->
  
      <param name = "height_map" value = "false" />
      <param name = "colored_map" value = "true" />
  	
      <!-- 增加了半径滤波器 -->
      <param name = "outrem_radius" type = "double" value = "1.0" />
      <param name = "outrem_neighbors" type = "int" value = "10" />
  
      <!-- when building map, set to false to speed up!!! -->
      <param name = "latch" value = "false" /> 
  
      <!-- topic from where pointcloud2 messages are subscribed -->
      <!-- 要订阅的点云主题名称 /pointcloud/output -->
      <!-- 这句话的意思是把当前节点订阅的主题名称从 cloud_in 变为 /pointcloud/output -->
      <remap from = "/cloud_in" to = "/fusion_cloud" />
   
    </node>
  </launch>
  ```

  


# 四、ROS控制
## 1. ROS Controller

[参考](https://www.guyuehome.com/890)

- [ros_control](http://wiki.ros.org/ros_control)就是ROS为用户提供的应用与机器人之间的中间件，包含一系列控制器接口、传动装置接口、硬件接口、控制器工具箱等等。

- ros_control 是一套机器人控制的中间件，是一套规范，不同的机器人平台只要按照这套规范实现，那么就可以保证 与ROS 程序兼容，通过这套规范，实现了一种可插拔的架构设计，大大提高了程序设计的效率与灵活性。

- 流程图为：

  ![](http://wiki.ros.org/ros_control?action=AttachFile&do=get&target=gazebo_ros_control.png)

  可以看到有5个功能包

  1. [Controller Manager](http://wiki.ros.org/controller_manager)：每个机器人可能有多个controller，所以这里有一个控制器管理器的概念，提供一种通用的接口来管理不同的controller。controller manager的输入就是ROS上层应用的输出。
  2. [Controller](http://wiki.ros.org/ros_controllers?distro=noetic)：controller可以完成每个joint的控制，请求下层的硬件资源，并且提供了PID控制器，读取硬件资源接口中的状态，在发布控制命令。
  3. Hardware Rescource：为上下两层提供硬件资源的接口。
  4. [hardware_interface](https://github.com/ros-controls/ros_control/wiki/hardware_interface)：硬件抽象层和硬件直接打交道，通过write和read方法来完成硬件的操作，这一层也包含关节限位、力矩转换、状态转换等功能。
  5. Real Robot：实际的机器人上也需要有自己的嵌入式控制器，接收到命令后需要反映到执行器上，比如接收到位置1的命令后，那就需要让执行器快速、稳定的到达位置1。

## 2. Arbotix控制器

- 简介：

  [arbotix](http://wiki.ros.org/arbotix)是一款控制电机、舵机的控制板，并提供了ROS功能包。他可以驱动真实的arbotix控制板，也可以在rviz中仿真。它还提供一个差速控制器，通过接受速度控制指令更新机器人的 joint 状态，从而帮助我们实现机器人在 rviz 中的运动。这个差速控制器在 arbotix_python 程序包中，完整的 arbotix 程序包还包括多种控制器，分别对应 dynamixel 电机、多关节机械臂以及不同形状的夹持器。

  使用`sudo apt-get install ros-<<VersionName()>>-arbotix`

- 例子：

  控制下面URDF,SRDF和Xacro中4.5的小车

  - 在config文件夹中创建argbotix_control.yaml

    ```
    # 该文件是控制器配置,一个机器人模型可能有多个控制器，比如: 底盘、机械臂、夹持器(机械手)....
    # 因此，根 name 是 controller
    controllers: {
       # 单控制器设置
       base_controller: {
              #类型: 差速控制器
           type: diff_controller,
           #参考坐标
           base_frame_id: base_footprint, 
           #两个轮子之间的间距
           base_width: 0.2,
           #控制频率
           ticks_meter: 2000, 
           #PID控制参数，使机器人车轮快速达到预期速度
           Kp: 12, 
           Kd: 12, 
           Ki: 0, 
           Ko: 50, 
           #加速限制
           accel_limit: 1.0 
        }
    }
    
    ```

  - 在4.5中的.launch文件中加下面代码

    ```
    <node name="arbotix" pkg="arbotix_python" type="arbotix_driver" output="screen">
         <rosparam file="$(find mobile_robot)/config/argbotix_control.yaml" command="load" />
         <param name="sim" value="true" />
    </node>
    
    ```

  - 在rviz中添加TF和Odometry

  - 使用`cmd_vel`控制小车运动

    ```shell
    rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}'
    ```

## 3. 建立自己的controller

[参考](https://zhuanlan.zhihu.com/p/433214417)

- 步骤：
  1. 编写控制器的c++代码以及头文件
  2. 将c++类注册发布为一个插件
  3. 在XML文件中定义插件
  4. 更新package xml发布插件
  5. 编写CMakeLists.txt
  6. 编译代码
  7. 为参数编写配置文件

### 3.1 编写控制器的c++代码以及头文件

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

### 3.2 在XML文件中定义插件

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

### 3.3 更新package xml发布插件

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

### 3.4 编写CMakeLists.txt

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

### 3.5 编写launch file

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

### 3.6 为参数编写配置文件

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



# 五、URDF，SRDF和 Xacro

## 1 URDF

- 一些学习网站

  - [URDF ROS Wiki Page](http://www.ros.org/wiki/urdf) - The URDF ROS Wiki page is the source of most information about the URDF.

  - [URDF Tutorials](http://www.ros.org/wiki/urdf/Tutorials) - Tutorials for working with the URDF.

  - [SOLIDWORKS URDF Plugin](http://www.ros.org/wiki/sw_urdf_exporter) - A plugin that lets you generate a URDF directly from a SOLIDWORKS model.

  - [URDF Examples](https://wiki.ros.org/urdf/Examples)

### 1.0如何rviz显示urdf
- [展示一个urdf](https://blog.csdn.net/xuehuafeiwu123/article/details/60764997)(urdf_tutorial)

  ```shell
  # 输入绝对路径
  $ roslaunch urdf_tutorial display.launch model:=/opt/ros/noetic/share/urdf_tutorial/urdf/01-myfirst.urdf
  # 也可以直接寻找，不管shell在什么路径下使用都可以
  $ roslaunch urdf_tutorial display.launch model:='$(find urdf_tutorial)/urdf/01-myfirst.urdf'
  ```

  - 用到的launch file:

    至少需要包含3个节点

  ```xml
  <launch>
      <arg name="model" default="$(find urdf_tutorial)/urdf/01-myfirst.urdf"/>
      <!--专门在 rviz 中使用的。可以显示各关节的滑动条。-->
      <arg name="gui" default="true" />
      <arg name="rvizconfig" default="$(find urdf_tutorial)/rviz/urdf.rviz" />
  	
      <!--第一个：加载urdf模型-->
      <!--如果error13：禁止读取。将command改成textfile可解决-->
      <param name="robot_description" command="$(find xacro)/xacro $(arg model)" />
      
      <!--第二个：joint_ state_publisher用于读取机器人模型中的参数，并发布一系列的变换矩阵组成机器人的 tf 树。-->
      <node if="$(arg gui)" name="joint_state_publisher" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" />
      <node unless="$(arg gui)" name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
      
      <!--第三个：robot_ state_publisher 发出机器人的状态-->>
      <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
      
      <!--打开rviz设置文件-->
      <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" required="true" />
  
  </launch>
  ```

### 1.1Fixed joint Robot

- 一个只有固定joint的机器人

  - mesh的使用见代码180行

    mesh文件一般分2种：

    - <.dae> 格式的，主要用来显示模型
    -  <.stl > 格式的,主要用来进行碰撞检测。

    可以用各种三维制图软件来生成。如果不进行碰撞检测，完全可以用圆柱体、长方体等常见形状来代替。只要质量、质心、惯量矩阵等设置正确，运动学和动力学仿真没有任何问题。显示模型的形状只是为了好看，而其设置的属性才是根本。

  ```xml
  <?xml version="1.0"?>
  <!--机器人的名字时visual-->
  <robot name="visual">
      <!--定义三种不同的颜色-->
      <!--可以添加texture纹理，用一张图来作物体的表面-->
      <material name="blue">
          <color rgba="0 0 0.8 1"/>
      </material>
  
      <material name="black">
          <color rgba="0 0 0 1"/>
      </material>
  
      <material name="white">
          <color rgba="1 1 1 1"/>
      </material>
  
      <!--定义一个名为base_link的link-->
      <link name="base_link">
          <!--下面是可以看到的部分-->
          <visual>
              <!--几何性质-->
              <geometry>
                  <!--一个长0.6米，半径为0.2米的圆柱-->
                  <cylinder length="0.6" radius="0.2"/>
              </geometry>
              <!--用到上面自定义的材料-->
              <material name="blue"/>
          </visual>
      </link>
  
      <link name="right_leg">
          <visual>
              <geometry>
                  <!--这里的几何是一个长宽高为如下的盒子-->
                  <box size="0.6 0.1 0.2"/>
              </geometry>
              <!--相对于自己的origin原点的位移和旋转-->
              <!--rpy表示roll,pitch,yaw-->
              <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
              <material name="white"/>
          </visual>
      </link>
      <!--连接2个link需要用到joint,fixed表示这是个不能动的关节-->
      <joint name="base_to_right_leg" type="fixed">
          <parent link="base_link"/>
          <child link="right_leg"/>
          <!--表示child link的原点origin相对于parent link的origin偏移了xyz-->
          <origin xyz="0 -0.22 0.25"/>
      </joint>
  
      <link name="right_base">
          <visual>
              <geometry>
                  <box size="0.4 0.1 0.1"/>
              </geometry>
              <material name="white"/>
          </visual>
      </link>
      <joint name="right_base_joint" type="fixed">
          <parent link="right_leg"/>
          <child link="right_base"/>
          <origin xyz="0 0 -0.6"/>
      </joint>
  
      <link name="right_front_wheel">
          <visual>
              <origin rpy="1.57075 0 0" xyz="0 0 0"/>
              <geometry>
                  <cylinder length="0.1" radius="0.035"/>
              </geometry>
              <material name="black"/>
              <origin rpy="0 0 0" xyz="0 0 0"/>
          </visual>
      </link>
      <joint name="right_front_wheel_joint" type="fixed">
          <parent link="right_base"/>
          <child link="right_front_wheel"/>
          <origin rpy="0 0 0" xyz="0.133333333333 0 -0.085"/>
      </joint>
  
      <link name="right_back_wheel">
          <visual>
              <origin rpy="1.57075 0 0" xyz="0 0 0"/>
              <geometry>
                  <cylinder length="0.1" radius="0.035"/>
              </geometry>
              <material name="black"/>
          </visual>
      </link>
      <joint name="right_back_wheel_joint" type="fixed">
          <parent link="right_base"/>
          <child link="right_back_wheel"/>
          <origin rpy="0 0 0" xyz="-0.133333333333 0 -0.085"/>
      </joint>
  
      <link name="left_leg">
          <visual>
              <geometry>
                  <box size="0.6 0.1 0.2"/>
              </geometry>
              <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
              <material name="white"/>
          </visual>
      </link>
      <joint name="base_to_left_leg" type="fixed">
          <parent link="base_link"/>
          <child link="left_leg"/>
          <origin xyz="0 0.22 0.25"/>
      </joint>
  
      <link name="left_base">
          <visual>
              <geometry>
                  <box size="0.4 0.1 0.1"/>
              </geometry>
              <material name="white"/>
          </visual>
      </link>
      <joint name="left_base_joint" type="fixed">
          <parent link="left_leg"/>
          <child link="left_base"/>
          <origin xyz="0 0 -0.6"/>
      </joint>
  
      <link name="left_front_wheel">
          <visual>
              <origin rpy="1.57075 0 0" xyz="0 0 0"/>
              <geometry>
                  <cylinder length="0.1" radius="0.035"/>
              </geometry>
              <material name="black"/>
          </visual>
      </link>
      <joint name="left_front_wheel_joint" type="fixed">
          <parent link="left_base"/>
          <child link="left_front_wheel"/>
          <origin rpy="0 0 0" xyz="0.133333333333 0 -0.085"/>
      </joint>
  
      <link name="left_back_wheel">
          <visual>
              <origin rpy="1.57075 0 0" xyz="0 0 0"/>
              <geometry>
                  <cylinder length="0.1" radius="0.035"/>
              </geometry>
              <material name="black"/>
          </visual>
      </link>
      <joint name="left_back_wheel_joint" type="fixed">
          <parent link="left_base"/>
          <child link="left_back_wheel"/>
          <origin rpy="0 0 0" xyz="-0.133333333333 0 -0.085"/>
      </joint>
  
      <joint name="gripper_extension" type="fixed">
          <parent link="base_link"/>
          <child link="gripper_pole"/>
          <origin rpy="0 0 0" xyz="0.19 0 0.2"/>
      </joint>
      <link name="gripper_pole">
          <visual>
              <geometry>
                  <cylinder length="0.2" radius="0.01"/>
              </geometry>
              <origin rpy="0 1.57075 0 " xyz="0.1 0 0"/>
          </visual>
      </link>
  
      <joint name="left_gripper_joint" type="fixed">
          <origin rpy="0 0 0" xyz="0.2 0.01 0"/>
          <parent link="gripper_pole"/>
          <child link="left_gripper"/>
      </joint>
  
      <link name="left_gripper">
          <visual>
              <origin rpy="0.0 0 0" xyz="0 0 0"/>
              <geometry>
                  <!--通过mesh来秒速一些复杂图形-->
                  <!--通过package://NAME_OF_PACKAGE/path notation来调用-->
                  <mesh filename="package://urdf_tutorial/meshes/l_finger.dae"/>
              </geometry>
          </visual>
      </link>
  
      <joint name="left_tip_joint" type="fixed">
          <parent link="left_gripper"/>
          <child link="left_tip"/>
      </joint>
      <link name="left_tip">
          <visual>
              <origin rpy="0.0 0 0" xyz="0.09137 0.00495 0"/>
              <geometry>
                  <mesh filename="package://urdf_tutorial/meshes/l_finger_tip.dae"/>
              </geometry>
          </visual>
      </link>
      <joint name="right_gripper_joint" type="fixed">
          <origin rpy="0 0 0" xyz="0.2 -0.01 0"/>
          <parent link="gripper_pole"/>
          <child link="right_gripper"/>
      </joint>
  
      <link name="right_gripper">
          <visual>
              <origin rpy="-3.1415 0 0" xyz="0 0 0"/>
              <geometry>
                  <mesh filename="package://urdf_tutorial/meshes/l_finger.dae"/>
              </geometry>
          </visual>
      </link>
      <joint name="right_tip_joint" type="fixed">
          <parent link="right_gripper"/>
          <child link="right_tip"/>
      </joint>
  
      <link name="right_tip">
          <visual>
              <origin rpy="-3.1415 0 0" xyz="0.09137 0.00495 0"/>
              <geometry>
                  <mesh filename="package://urdf_tutorial/meshes/l_finger_tip.dae"/>
              </geometry>
          </visual>
      </link>
  
      <link name="head">
          <visual>
              <geometry>
              <!--这里的几何图形是一个圆球-->
              <sphere radius="0.2"/>
                  </geometry>
              <material name="white"/>
          </visual>
      </link>
      <joint name="head_swivel" type="fixed">
          <parent link="base_link"/>
          <child link="head"/>
          <origin xyz="0 0 0.3"/>
      </joint>
  
      <link name="box">
          <visual>
              <geometry>
                  <box size="0.08 0.08 0.08"/>
              </geometry>
              <material name="blue"/>
          </visual>
      </link>
      <joint name="tobox" type="fixed">
          <parent link="head"/>
          <child link="box"/>
          <origin xyz="0.1814 0 0.1414"/>
      </joint>
  </robot>
  ```

### 1.2 Flexible joint Robot

1.1中所有的joint都是固定不能动的,现在要revise修改为可动的。

- 修改头和身体之间的joint，轮子与脚之间的joint的类型为**continuous**

  ```xml
  <!--头和身体-->
  <!--continuous可以让两个link饶某个轴随意的旋转-->
  <joint name="head_swivel" type="continuous">
      <parent link="base_link"/>
      <child link="head"/>
      <!--定义旋转轴为z轴-->
      <axis xyz="0 0 1"/>
      <origin xyz="0 0 0.3"/>
  </joint>
  <!--有四个轮子，下面为其中一个和他的脚-->
  <joint name="left_back_wheel_joint" type="continuous">
      <!--定义旋转轴为y轴-->
      <axis rpy="0 0 0" xyz="0 1 0"/>
      <parent link="left_base"/>
      <child link="left_back_wheel"/>
      <origin rpy="0 0 0" xyz="-0.133333333333 0 -0.085"/>
  </joint>
  ```

- 修改Gripper夹子的joint类型为**revolute**

  ```xml
  <!--有2个夹子，一左一右，都需要改成revolute-->
  <!--revolute可以像continuous一样饶轴旋转，但他们有严格strict的限制，需要limit标签-->
  <joint name="left_gripper_joint" type="revolute">
      <!--定义旋转轴为z轴-->
      <axis xyz="0 0 1"/>
      <!--定义上下限(radians弧度制)，定义最大速度和力-->
      <limit effort="1000.0" lower="0.0" upper="0.548" velocity="0.5"/>
      <origin rpy="0 0 0" xyz="0.2 0.01 0"/>
      <parent link="gripper_pole"/>
      <child link="left_gripper"/>
  </joint>
  ```

- 修改Gripper夹子的手臂的joint类型为**prismatic**

  ```xml
  <!--prismatic可以沿着某个轴运动-->
  <joint name="gripper_extension" type="prismatic">
      <parent link="base_link"/>
      <child link="gripper_pole"/>
      <!--定义上下限(meter米)，定义最大速度和力-->
      <limit effort="1000.0" lower="-0.38" upper="0" velocity="0.5"/>
      <origin rpy="0 0 0" xyz="0.19 0 0.2"/>
  </joint>
  ```

- 其他类型的joint

  - **planar joint**：相对于prismatic只能沿着一个轴的移动，k而一再平面正交方向上平移或旋转
  - **floating joint**：浮动关节，可以进行平移和旋转运动

- Specify指定 the pose

  当我们在rviz中拖动joint的控制滚条，会进行如下操作

  1. [joint_state_publisher](https://wiki.ros.org/joint_state_publisher)会parse语法分析URDF并找到所有的可移动joint和他们的限制limit
  2. 接着joint_state_publisher会将控制滚条的值以[sensor_msgs/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html)msg发送
  3. 这个msg会被[robot_state_publisher](https://wiki.ros.org/robot_state_publisher)用来计算不同部分之间的移动transforms.
  4. 最后生成的变化树resulting transform tree被用来在rviz中显示所有的形状shapes。

### 1.3 Physical and Collision Properties

- [Here is the new urdf](https://raw.githubusercontent.com/ros/urdf_tutorial/master/urdf/07-physics.urdf) with collision and physical properties.

- Collision:

  用于碰撞检测

  ```xml
  <link name="base_link">
      <visual>
          <geometry>
          	<cylinder length="0.6" radius="0.2"/>
          </geometry>
          <material name="blue"/>
      </visual>
      <collision>
          <geometry>
          	<cylinder length="0.6" radius="0.2"/>
          </geometry>
      </collision>
  </link>
  ```

  - collison和visual同级，是link的直接subelement
  - 和visual一样，设置碰撞检测的形状
  - 虽然现实中碰撞和可见图形是一致的，但在如下两种场景，应该用更简单的碰撞几何来替代：
    - Quicker Processing：因为碰撞计算更复杂，所以如果需要节省计算资源，可以使用简单的geometries几何来代替复杂的collision elements
    - Safe Zones：有些sensitive equipment敏感部件，我们不希望任何东西与他碰撞。比如机器人的头部，我们就可以把头的碰撞几何设置为一个包围它的圆柱，以防任何东西太靠近他的头部。

- Physical Properties

  为了正确的properly模拟，需要link有相关的物理性质

  ```xml
  <link name="base_link">
      <visual>
          <geometry>
          	<cylinder length="0.6" radius="0.2"/>
          </geometry>
          <material name="blue"/>
      </visual>
      <collision>
          <geometry>
          	<cylinder length="0.6" radius="0.2"/>
          </geometry>
      </collision>
      <inertial>
          <!--质量单位是kilograms-->
          <mass value="10"/>
          <!--rotational inertia matrix旋转惯量矩阵，是一个symmetrical对称矩阵-->
          <!--如果不确定，可以用ixx/iyy/izz=1e-3 or smaller作为默认值，这是对于一个中型尺寸的link而言的(比如一个0.1m长，0.6kg的盒子)-->
          <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
      </inertial>
  </link>  
  
  ```

  - inertial和collision,visual一样，都是link的直接subelement

  - Contact Coefficients

    - mu - [Friction coefficient](https://simple.wikipedia.org/wiki/Coefficient_of_friction)摩擦系数
    - kp - [Stiffness coefficient](https://en.wikipedia.org/wiki/Stiffness)弹性系数
    - kd - [Dampening coefficient](https://en.wikipedia.org/wiki/Damping_ratio#Definition)阻尼系数

  - Joint Dynamics

    关节移动由joint的dynamics tag定义，有2个attribute属性：

    1. friction - The physical static friction静态摩擦. For prismatic平移关节 joints, the units are Newtons. For 旋转关节revolving joints, the units are Newton meters.
    2. damping - The physical damping value阻尼值. For prismatic joints, the units are Newton seconds per meter. For revolving joints, Newton meter seconds per radian

  - Jonit 的其他一些[tag](https://wiki.ros.org/urdf/XML/joint)

    - safety_controller(optional)
    - mimic(optional)
    - dynamics(optional)
    - calibration(optional)
  
### 1.4 URDF 工具

  安装命令:`sudo apt install liburdfdom-tools`

- `check_urdf`命令可以检查复杂的 urdf 文件是否存在语法问题

  使用：`check_urdf urdf文件`

- `urdf_to_graphiz`命令可以查看 urdf 模型结构，显示不同 link 的层级关系

  使用：`urdf_to_graphiz urdf文件`

  会在当前目录下生成pdf文件

## 2 SRDF

- SRDF是对URDF的补充

## 3 [MoveIt Setup Assistant](https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html)

- 用于生成SRDF

## 4 [Xacro](http://wiki.ros.org/xacro)

[Xacro](https://wiki.ros.org/xacro)名字由来： macro宏 language for XML

- 相比urdf提供了如下三种特性，帮忙降低模型开发难度并降低了模型描述的复杂度

  - Constants常值

  - Simple Math数学计算

  - Macros宏

### 4.1 如何使用 Xacro

- 通常的使用如下：

  写好Xacro后，先将xacro转化为urdf，再使用。

​		转换命令：`rosrun xacro xacro model.xacro > model.urdf `

- 也可以直接在launch file中自动生成urdf，但这会花费更多的时间来启动节点

    ```xml
    <param name="robot_description" command="xacro '$(find pr2_description)/robots/pr2.urdf.xacro'" />
    ```

- 在xml文件的开头需要

    ```xml
    <?xml version="1.0"?>
    <!--根标签：必须指明xmlns:xacro-->
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="firefighter">
        ...
    </robot>
    ```

### 4.2 Constants

```xml
<!--使用xacro的语法必须要用xacro:，其他的都和urdf一样-->
<xacro:property name="width" value="0.2" />
<xacro:property name="bodylen" value="0.6" />
<link name="base_link">
    <visual>
        <geometry>
            <cylinder radius="${width}" length="${bodylen}"/>
        </geometry>
        <material name="blue"/>
    </visual>
    <collision>
        <geometry>
            <cylinder radius="${width}" length="${bodylen}"/>
        </geometry>
    </collision>
</link>
```

- 相比于urdf，xacro可以设置常值，这样一些不同部件但相同的固定的参数如长度，宽度就可以统一设置成一个constant,之后要调整也很方便。

- 我们还能自动组合constants

  ```xml
  <xacro:property name=”robotname” value=”marvin” />
  <link name=”${robotname}s_leg” />
  <!--上面2行等价于下面-->
  <link name=”marvins_leg” />
  ```

### 4.3 Math

xacaro也支持简单的数学计算,

```xml
<cylinder radius="${wheeldiam/2}" length="0.1"/>
<origin xyz="${reflect*(width+.02)} 0 0.25" />
```

- 所有的数学计算，数据类型都是floats

### 4.4 Macros

#### 1. Simple Macro

```xml
<!--定义宏结构-->
<xacro:macro name="default_origin">
    <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:macro>
<!--调用定义过的宏-->
<xacro:default_origin />
<!--上面6行等价于-->
<origin rpy="0 0 0" xyz="0 0 0"/>
```

#### 2. Parameterized Macro

- 把一个参数传入xacro macro 中

  ```xml
  <!--定义宏结构-->
  <xacro:macro name="default_inertial" params="mass">
      <inertial>
              <mass value="${mass}" />
              <inertia ixx="1.0" ixy="0.0" ixz="0.0"
                   iyy="1.0" iyz="0.0"
                   izz="1.0" />
      </inertial>
  </xacro:macro>
  <!--调用定义过的宏-->
  <xacro:default_inertial mass="10"/>
  ```

- 也可以传入一个block parameter

  ```xml
  <!--定义宏结构-->
  <!--如果是一个block parameter，需要再parameter名字前加上*asterisk号-->
  <xacro:macro name="blue_shape" params="name *shape">
      <link name="${name}">
          <visual>
              <geometry>
                  <!--使用xacro:insert_block来插入-->
                  <xacro:insert_block name="shape" />
              </geometry>
              <material name="blue"/>
          </visual>
          <collision>
              <geometry>
                  <xacro:insert_block name="shape" />
              </geometry>
          </collision>
      </link>
  </xacro:macro>
  <!--调用定义过的宏,并给与块结构-->
  <xacro:blue_shape name="base_link">
      <cylinder radius=".42" length=".01" />
  </xacro:blue_shape>
  ```

#### 3. 实际使用:Leg Macro

```xml
<xacro:macro name="leg" params="prefix reflect">
    <link name="${prefix}_leg">
        <visual>
            <geometry>
                <box size="${leglen} 0.1 0.2"/>
            </geometry>
            <origin xyz="0 0 -${leglen/2}" rpy="0 ${pi/2} 0"/>
            <material name="white"/>
        </visual>
        <collision>
            <geometry>
                <box size="${leglen} 0.1 0.2"/>
            </geometry>
            <origin xyz="0 0 -${leglen/2}" rpy="0 ${pi/2} 0"/>
        </collision>
        <xacro:default_inertial mass="10"/>
    </link>

    <joint name="base_to_${prefix}_leg" type="fixed">
        <parent link="base_link"/>
        <child link="${prefix}_leg"/>
        <origin xyz="0 ${reflect*(width+.02)} 0.25" />
    </joint>
    <!-- A bunch of stuff cut -->
</xacro:macro>
<!--定义一遍宏，创建2个腿-->
<!--第一个参数prefix定义了创建的是哪条腿-->
<!--第二个参数reflect用于计算origin，即腿的位置-->
<xacro:leg prefix="right" reflect="1" />
<xacro:leg prefix="left" reflect="-1" />
```

### 4.5 多xacro组合

机器人模型由多部件组成，可以将不同组件设置进单独文件，最终通过文件包含实现组件的拼装。

- 一个小车例子：
  - 首先编写底盘，摄像头和雷达的 xacro 文件。
  - 然后再编写一个组合文件，组合底盘、摄像头与雷达。
  - 最后，通过 launch 文件启动 Rviz 并显示模型

#### 4.5.1 地盘xacro

```xml
<!--
    使用 xacro 优化 URDF 版的小车底盘实现：

    实现思路:
    1.将一些常量、变量封装为 xacro:property
      比如:PI 值、小车底盘半径、离地间距、车轮半径、宽度 ....
    2.使用 宏 封装驱动轮以及支撑轮实现，调用相关宏生成驱动轮与支撑轮

-->
<!-- 根标签，必须声明 xmlns:xacro -->
<robot name="my_base" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- 封装变量、常量 -->
    <xacro:property name="PI" value="3.141"/>
    <!-- 宏:黑色设置 -->
    <material name="black">
        <color rgba="0.0 0.0 0.0 1.0" />
    </material>
    <!-- 底盘属性 -->
    <xacro:property name="base_footprint_radius" value="0.001" /> <!-- base_footprint 半径  -->
    <xacro:property name="base_link_radius" value="0.1" /> <!-- base_link 半径 -->
    <xacro:property name="base_link_length" value="0.08" /> <!-- base_link 长 -->
    <xacro:property name="earth_space" value="0.015" /> <!-- 离地间距 -->

    <!-- 底盘 -->
    <link name="base_footprint">
      <visual>
        <geometry>
          <sphere radius="${base_footprint_radius}" />
        </geometry>
      </visual>
    </link>

    <link name="base_link">
      <visual>
        <geometry>
          <cylinder radius="${base_link_radius}" length="${base_link_length}" />
        </geometry>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <material name="yellow">
          <color rgba="0.5 0.3 0.0 0.5" />
        </material>
      </visual>
    </link>

    <joint name="base_link2base_footprint" type="fixed">
      <parent link="base_footprint" />
      <child link="base_link" />
      <origin xyz="0 0 ${earth_space + base_link_length / 2 }" />
    </joint>

    <!-- 驱动轮 -->
    <!-- 驱动轮属性 -->
    <xacro:property name="wheel_radius" value="0.0325" /><!-- 半径 -->
    <xacro:property name="wheel_length" value="0.015" /><!-- 宽度 -->
    <!-- 驱动轮宏实现 -->
    <xacro:macro name="add_wheels" params="name flag">
      <link name="${name}_wheel">
        <visual>
          <geometry>
            <cylinder radius="${wheel_radius}" length="${wheel_length}" />
          </geometry>
          <origin xyz="0.0 0.0 0.0" rpy="${PI / 2} 0.0 0.0" />
          <material name="black" />
        </visual>
      </link>

      <joint name="${name}_wheel2base_link" type="continuous">
        <parent link="base_link" />
        <child link="${name}_wheel" />
        <origin xyz="0 ${flag * base_link_radius} ${-(earth_space + base_link_length / 2 - wheel_radius) }" />
        <axis xyz="0 1 0" />
      </joint>
    </xacro:macro>
    <xacro:add_wheels name="left" flag="1" />
    <xacro:add_wheels name="right" flag="-1" />
    <!-- 支撑轮 -->
    <!-- 支撑轮属性 -->
    <xacro:property name="support_wheel_radius" value="0.0075" /> <!-- 支撑轮半径 -->

    <!-- 支撑轮宏 -->
    <xacro:macro name="add_support_wheel" params="name flag" >
      <link name="${name}_wheel">
        <visual>
            <geometry>
                <sphere radius="${support_wheel_radius}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="black" />
        </visual>
      </link>

      <joint name="${name}_wheel2base_link" type="continuous">
          <parent link="base_link" />
          <child link="${name}_wheel" />
          <origin xyz="${flag * (base_link_radius - support_wheel_radius)} 0 ${-(base_link_length / 2 + earth_space / 2)}" />
          <axis xyz="1 1 1" />
      </joint>
    </xacro:macro>

    <xacro:add_support_wheel name="front" flag="1" />
    <xacro:add_support_wheel name="back" flag="-1" />

</robot>

```

#### 4.5.2 摄像头xacro

```xml
<!-- 摄像头相关的 xacro 文件 -->
<robot name="my_camera" xmlns:xacro="http://wiki.ros.org/xacro">
    <!-- 摄像头属性 -->
    <xacro:property name="camera_length" value="0.01" /> <!-- 摄像头长度(x) -->
    <xacro:property name="camera_width" value="0.025" /> <!-- 摄像头宽度(y) -->
    <xacro:property name="camera_height" value="0.025" /> <!-- 摄像头高度(z) -->
    <xacro:property name="camera_x" value="0.08" /> <!-- 摄像头安装的x坐标 -->
    <xacro:property name="camera_y" value="0.0" /> <!-- 摄像头安装的y坐标 -->
    <xacro:property name="camera_z" value="${base_link_length / 2 + camera_height / 2}" /> <!-- 摄像头安装的z坐标:底盘高度 / 2 + 摄像头高度 / 2  -->

    <!-- 摄像头关节以及link -->
    <link name="camera">
        <visual>
            <geometry>
                <box size="${camera_length} ${camera_width} ${camera_height}" />
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
            <material name="black" />
        </visual>
    </link>

    <joint name="camera2base_link" type="fixed">
        <parent link="base_link" />
        <child link="camera" />
        <origin xyz="${camera_x} ${camera_y} ${camera_z}" />
    </joint>
</robot>

```

#### 4.5.3 雷达xacro

```xml
<!--
    小车底盘添加雷达
-->
<robot name="my_laser" xmlns:xacro="http://wiki.ros.org/xacro">

    <!-- 雷达支架 -->
    <xacro:property name="support_length" value="0.15" /> <!-- 支架长度 -->
    <xacro:property name="support_radius" value="0.01" /> <!-- 支架半径 -->
    <xacro:property name="support_x" value="0.0" /> <!-- 支架安装的x坐标 -->
    <xacro:property name="support_y" value="0.0" /> <!-- 支架安装的y坐标 -->
    <xacro:property name="support_z" value="${base_link_length / 2 + support_length / 2}" /> <!-- 支架安装的z坐标:底盘高度 / 2 + 支架高度 / 2  -->

    <link name="support">
        <visual>
            <geometry>
                <cylinder radius="${support_radius}" length="${support_length}" />
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
            <material name="red">
                <color rgba="0.8 0.2 0.0 0.8" />
            </material>
        </visual>
    </link>

    <joint name="support2base_link" type="fixed">
        <parent link="base_link" />
        <child link="support" />
        <origin xyz="${support_x} ${support_y} ${support_z}" />
    </joint>


    <!-- 雷达属性 -->
    <xacro:property name="laser_length" value="0.05" /> <!-- 雷达长度 -->
    <xacro:property name="laser_radius" value="0.03" /> <!-- 雷达半径 -->
    <xacro:property name="laser_x" value="0.0" /> <!-- 雷达安装的x坐标 -->
    <xacro:property name="laser_y" value="0.0" /> <!-- 雷达安装的y坐标 -->
    <xacro:property name="laser_z" value="${support_length / 2 + laser_length / 2}" /> <!-- 雷达安装的z坐标:支架高度 / 2 + 雷达高度 / 2  -->

    <!-- 雷达关节以及link -->
    <link name="laser">
        <visual>
            <geometry>
                <cylinder radius="${laser_radius}" length="${laser_length}" />
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
            <material name="black" />
        </visual>
    </link>

    <joint name="laser2support" type="fixed">
        <parent link="support" />
        <child link="laser" />
        <origin xyz="${laser_x} ${laser_y} ${laser_z}" />
    </joint>
</robot>

```

#### 4.5.4 组合xacro

将上面3个组装起来

```xml
<!-- 组合小车底盘与摄像头与雷达 -->
<robot name="my_car_camera" xmlns:xacro="http://wiki.ros.org/xacro">
    <xacro:include filename="my_base.urdf.xacro" />
    <xacro:include filename="my_camera.urdf.xacro" />
    <xacro:include filename="my_laser.urdf.xacro" />
</robot>
```

#### 4.5.5 launch file

```xml
<launch>
    <param name="robot_description" command="$(find xacro)/xacro $(find demo01_urdf_helloworld)/urdf/xacro/my_base_camera_laser.urdf.xacro" />

    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find demo01_urdf_helloworld)/config/helloworld.rviz" />
    <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" output="screen" />
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen" />
    <node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" output="screen" />

</launch>

```



## 5 Using URDF in Gazebo

### 5.1官网教程

[代码例子](https://github.com/ros/urdf_sim_tutorial)

#### 5.1.1 launch file

运行：`roslaunch urdf_sim_tutorial 13-diffdrive.launch`

一共有2个launch file:

1. 13-diffdrive.launch

    ```xml
    <launch>
        <!--读取模型-->
        <arg name="model" default="$(find urdf_sim_tutorial)/urdf/13-diffdrive.urdf.xacro"/>
        <!--加载rviz配置文件-->
        <arg name="rvizconfig" default="$(find urdf_tutorial)/rviz/urdf.rviz" />
        <!--启动另一个launch file,用于启动gazebo-->
        <include file="$(find urdf_sim_tutorial)/launch/gazebo.launch">
        <arg name="model" value="$(arg model)" />
        </include>
        
        <!--按上面加载的rviz配置来打开rviz-->
        <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" />
        
        <!--读取四个yaml.分别用于不同的控制器(ns:命名空间)-->
        <!--yaml里存放的参数会传递给ROS parameter space然后发送给gazebo进行控制-->
        <rosparam command="load"
                file="$(find urdf_sim_tutorial)/config/joints.yaml"
                ns="r2d2_joint_state_controller" />
        <rosparam command="load"
                file="$(find urdf_sim_tutorial)/config/head.yaml"
                ns="r2d2_head_controller" />
        <rosparam command="load"
                file="$(find urdf_sim_tutorial)/config/gripper.yaml"
                ns="r2d2_gripper_controller" />
        <rosparam command="load"
                file="$(find urdf_sim_tutorial)/config/diffdrive.yaml"
                ns="r2d2_diff_drive_controller" />
        
    	<!--使用controller_manage将上面4个命名空间(ns)传入gazebo-->
        <node name="r2d2_controller_spawner" pkg="controller_manager" type="spawner"
        args="r2d2_joint_state_controller
              r2d2_head_controller
              r2d2_gripper_controller
              r2d2_diff_drive_controller
              --shutdown-timeout 3"/>
        
    	<!--rqt_robot_steering是rqt_robot_plugins提供的包-->
        <!--rqt_robot_steering提供了一个GUI Plugin用于驾驶机器人-->
        <node name="rqt_robot_steering" pkg="rqt_robot_steering" type="rqt_robot_steering">
            <!--rqt_robot_steering发送的topic-->
        	<param name="default_topic" value="/r2d2_diff_drive_controller/cmd_vel"/>
        </node>
    </launch>
    ```

2. gazobo.launch

   ```xml
   <launch>
   
       <!-- these are the arguments you can pass this launch file, for example paused:=true -->
       <arg name="paused" default="false"/>
       <arg name="use_sim_time" default="true"/>
       <arg name="gui" default="true"/>
       <arg name="headless" default="false"/>
       <arg name="debug" default="false"/>
       <arg name="model" default="$(find urdf_tutorial)/urdf/08-macroed.urdf.xacro"/>
   
       <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
       <include file="$(find gazebo_ros)/launch/empty_world.launch">
           <arg name="debug" value="$(arg debug)" />
           <arg name="gui" value="$(arg gui)" />
           <arg name="paused" value="$(arg paused)"/>
           <arg name="use_sim_time" value="$(arg use_sim_time)"/>
           <arg name="headless" value="$(arg headless)"/>
       </include>
       
   	<!--读取模型-->
       <param name="robot_description" command="$(find xacro)/xacro $(arg model)" />
   
       <!-- push robot_description to factory and spawn robot in gazebo -->
       <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"
       args="-z 1.0 -unpause -urdf -model robot -param robot_description" respawn="false" output="screen" />
   	
       <!--发送机器人状态-->
       <node pkg="robot_state_publisher" type="robot_state_publisher"  name="robot_state_publisher">
       	<param name="publish_frequency" type="double" value="30.0" />
       </node>
   
   </launch>
   ```
   
#### 5.1.2 yaml file

第一个type:"xx"，表明用的是哪个控制器。具体见[ros_controllers](http://wiki.ros.org/ros_controllers?distro=noetic)和[ros_control](http://wiki.ros.org/ros_control?distro=noetic)

   1. joints.yaml
   
      ```yaml
      # The joint state controller handles publishing transforms for any moving joints
      type: "joint_state_controller/JointStateController"
      publish_rate: 50
      ```
   
   2. head.yaml
   
      ```yaml
      type: "position_controllers/JointPositionController"
      joint: head_swivel
      ```
   
   3. gripper.yaml
   
      ```yaml
      type: "position_controllers/JointGroupPositionController"
      joints:
       - gripper_extension
       - left_gripper_joint
       - right_gripper_joint
      ```
   
   4. diffdrive.yam
   
      ```yaml
      type: "diff_drive_controller/DiffDriveController"
      publish_rate: 50
      
      left_wheel: ['left_front_wheel_joint', 'left_back_wheel_joint']
      right_wheel: ['right_front_wheel_joint', 'right_back_wheel_joint']
      
      wheel_separation: 0.44
      
      # Odometry covariances for the encoder output of the robot. These values should
      # be tuned to your robot's sample odometry data, but these values are a good place
      # to start
      pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.03]
      twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.03]
      
      # Top level frame (link) of the robot description
      base_frame_id: base_link
      
      # Velocity and acceleration limits for the robot
      linear:
        x:
          has_velocity_limits    : true
          max_velocity           : 0.2   # m/s
          has_acceleration_limits: true
          max_acceleration       : 0.6   # m/s^2
      angular:
        z:
          has_velocity_limits    : true
          max_velocity           : 2.0   # rad/s
          has_acceleration_limits: true
          max_acceleration       : 6.0   # rad/s^2
      ```

#### 5.1.3 xacro

[整体的xacro](https://github.com/ros/urdf_sim_tutorial/blob/master/urdf/13-diffdrive.urdf.xacro)很长，大部分和12.1中类似，下面是和gazebo和ros下的一些改动

- 添加Gazebo Plugin

  297-302行。用于连接gazebo和ros

  ```xml
  <gazebo>
      <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      	<robotNamespace>/</robotNamespace>
      </plugin>
  </gazebo>
  ```

- Transmission传动

  对于每一个non-fixed joint都需要设置一个transmission。他会告诉gazebo要让不同的joint做什么.

  下面每一个joint的name都必须和[整体的xacro](https://github.com/ros/urdf_sim_tutorial/blob/master/urdf/13-diffdrive.urdf.xacro)定义的joint名字一致

  1. head joint

     265-273行。控制头旋转

     ```xml
     <transmission name="head_swivel_trans">
         <type>transmission_interface/SimpleTransmission</type>
         <actuator name="$head_swivel_motor">
         	<mechanicalReduction>1</mechanicalReduction>
         </actuator>
         <!--joint名字和254行保持一致-->
         <joint name="head_swivel">
             <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
         </joint>
     </transmission>
     ```

  2. Wheel joint

     83-91行。控制轮子转动

     ```xml
     <transmission name="${prefix}_${suffix}_wheel_trans">
         <type>transmission_interface/SimpleTransmission</type>
         <actuator name="${prefix}_${suffix}_wheel_motor">
         	<mechanicalReduction>1</mechanicalReduction>
         </actuator>
         <!--joint名字和64行配对-->
         <joint name="${prefix}_${suffix}_wheel_joint">
         	<hardwareInterface>VelocityJointInterface</hardwareInterface>
         </joint>
     </transmission>
     ```

     

### 5.2 URDF与Gazebo基本流程

#### 5.2.1 创建功能包，导入依赖项

创建新功能包，导入依赖包: urdf、xacro、gazebo_ros、gazebo_ros_control、gazebo_plugins

```xml
<!--如果不导入这些包-->
<!--也可以在urdf中写入,见5.1.3-->
<gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    	<robotNamespace>/</robotNamespace>
    </plugin>
</gazebo>
```

#### 5.2.2 编写URDF

注意， 当 URDF 需要与 Gazebo 集成时，和 Rviz 有明显区别:

1. 必须使用 collision 标签，因为既然是仿真环境，那么必然涉及到碰撞检测，collision 提供碰撞检测的依据。

2. 必须使用 inertial 标签，此标签标注了当前机器人某个刚体部分的惯性矩阵，用于一些力学相关的仿真计算。

3. 颜色设置，也需要重新使用 gazebo 标签标注，因为之前的颜色设置为了方便调试包含透明度，仿真环境下没有此选项。

```xml
<!-- 
    创建一个机器人模型(盒状即可)，显示在 Gazebo 中 
-->

<robot name="mycar">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.2 0.1" />
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
            <material name="yellow">
                <color rgba="0.5 0.3 0.0 1" />
            </material>
        </visual>
        
        
        
        <!--1. 必须要有碰撞检测标签-->
        <!--如果机器人link是标准的几何体形状，和link的 visual 属性设置一致即可。-->
        <collision>
            <geometry>
                <box size="0.5 0.2 0.1" />
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
        </collision>
        
        
        
        <!--2. 必须要有惯性（物理性质）标签-->
        <!--
			需要注意的是，原则上，除了 base_footprint外，
			机器人的每个刚体部分都需要设置惯性矩阵，且惯性矩阵必须经计算得出。

			如果随意定义刚体部分的惯性矩阵，
			那么可能会导致机器人在Gazebo中出现抖动，移动等现象。
		-->
        <inertial>
            <origin xyz="0 0 0" />
            <mass value="6" />
            <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1" />
        </inertial>       
        <!--2.1 球体的惯性矩阵 -->
        <xacro:macro name="sphere_inertial_matrix" params="m r">
            <inertial>
                <mass value="${m}" />
                <inertia ixx="${2*m*r*r/5}" ixy="0" ixz="0"
                    iyy="${2*m*r*r/5}" iyz="0" 
                    izz="${2*m*r*r/5}" />
            </inertial>
        </xacro:macro>
        <!--2.2 圆柱的惯性矩阵 -->
		<xacro:macro name="cylinder_inertial_matrix" params="m r h">
            <inertial>
                <mass value="${m}" />
                <inertia ixx="${m*(3*r*r+h*h)/12}" ixy = "0" ixz = "0"
                    iyy="${m*(3*r*r+h*h)/12}" iyz = "0"
                    izz="${m*r*r/2}" /> 
            </inertial>
       	 </xacro:macro>
         <!--2.3 立方体的惯性矩阵 -->
         <xacro:macro name="Box_inertial_matrix" params="m l w h">
            <inertial>
                <mass value="${m}" />
                <inertia ixx="${m*(h*h + l*l)/12}" ixy = "0" ixz = "0"
                    iyy="${m*(w*w + l*l)/12}" iyz= "0"
                    izz="${m*(w*w + h*h)/12}" />
            </inertial>
          </xacro:macro>
  
    </link>
    
    
    
    <!--3. gazebo自带的材料标签-->
    <gazebo reference="base_link">
        <material>Gazebo/Black</material>
    </gazebo>
    

</robot>

```

#### 5.2.3启动gazebo并显示模型

```xml
<launch>

    <!-- 将 Urdf 文件的内容加载到参数服务器 -->
    <param name="robot_description" textfile="$(find demo02_urdf_gazebo)/urdf/urdf01_helloworld.urdf" />

    <!-- 启动 Gazebo 的仿真环境，当前环境为空环境 -->
    <!-- 参考5.1.1可以给空环境设置各种参数-->
    <include file="$(find gazebo_ros)/launch/empty_world.launch" />

    <!-- 在 gazebo 中显示机器人模型 -->
    <!-- 
        在 Gazebo 中加载一个机器人模型，该功能由 gazebo_ros 下的 spawn_model 提供:
        -urdf 加载的是 urdf 文件
        -model mycar 模型名称是 mycar
        -param robot_description 从参数 robot_description 中载入模型
        -x 模型载入的 x 坐标
        -y 模型载入的 y 坐标
        -z 模型载入的 z 坐标
	-->
    <node pkg="gazebo_ros" type="spawn_model" name="model" args="-urdf -model mycar -param robot_description"  />
    
</launch>

```



### 5.3 将4.5的xacro改写为gazebo可用

**实现流程:**

1. 需要编写封装惯性矩阵算法的 xacro 文件
2. 为机器人模型中的每一个 link 添加 collision 和 inertial 标签，并且重置颜色属性
3. 在 launch 文件中启动 gazebo 并添加机器人模型

#### 5.3.1 封装惯性矩阵算法的xacro文件

注意: 如果机器人模型在 Gazebo 中产生了抖动，滑动，缓慢位移 .... 诸如此类情况，请查看

1. 惯性矩阵是否设置了，且设置是否正确合理
2. 车轮翻转需要依赖于 PI 值，如果 PI 值精度偏低，也可能导致上述情况产生.



inertial.xacro

```xml
<robot name="base" xmlns:xacro="http://wiki.ros.org/xacro">
    <!-- Macro for inertia matrix -->
    <xacro:macro name="sphere_inertial_matrix" params="m r">
        <inertial>
            <mass value="${m}" />
            <inertia ixx="${2*m*r*r/5}" ixy="0" ixz="0"
                iyy="${2*m*r*r/5}" iyz="0" 
                izz="${2*m*r*r/5}" />
        </inertial>
    </xacro:macro>

    <xacro:macro name="cylinder_inertial_matrix" params="m r h">
        <inertial>
            <mass value="${m}" />
            <inertia ixx="${m*(3*r*r+h*h)/12}" ixy = "0" ixz = "0"
                iyy="${m*(3*r*r+h*h)/12}" iyz = "0"
                izz="${m*r*r/2}" /> 
        </inertial>
    </xacro:macro>

    <xacro:macro name="Box_inertial_matrix" params="m l w h">
       <inertial>
               <mass value="${m}" />
               <inertia ixx="${m*(h*h + l*l)/12}" ixy = "0" ixz = "0"
                   iyy="${m*(w*w + l*l)/12}" iyz= "0"
                   izz="${m*(w*w + h*h)/12}" />
       </inertial>
   </xacro:macro>
</robot>

```

#### 5.3.2 底盘xacro文件

car_base.urdf.xacro

```xml
<!--
    使用 xacro 优化 URDF 版的小车底盘实现：

    实现思路:
    1.将一些常量、变量封装为 xacro:property
      比如:PI 值、小车底盘半径、离地间距、车轮半径、宽度 ....
    2.使用 宏 封装驱动轮以及支撑轮实现，调用相关宏生成驱动轮与支撑轮

-->
<!-- 根标签，必须声明 xmlns:xacro -->
<robot name="my_base" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- 封装变量、常量 -->
    <!-- PI 值设置精度需要高一些，否则后续车轮翻转量计算时，可能会出现肉眼不能察觉的车轮倾斜，从而导致模型抖动 -->
    <xacro:property name="PI" value="3.1415926"/>
    <!-- 宏:黑色设置 -->
    <material name="black">
        <color rgba="0.0 0.0 0.0 1.0" />
    </material>
    <!-- 底盘属性 -->
    <xacro:property name="base_footprint_radius" value="0.001" /> <!-- base_footprint 半径  -->
    <xacro:property name="base_link_radius" value="0.1" /> <!-- base_link 半径 -->
    <xacro:property name="base_link_length" value="0.08" /> <!-- base_link 长 -->
    <xacro:property name="earth_space" value="0.015" /> <!-- 离地间距 -->
    <xacro:property name="base_link_m" value="0.5" /> <!-- 质量  -->

    <!-- 底盘 -->
    <link name="base_footprint">
      <visual>
        <geometry>
          <sphere radius="${base_footprint_radius}" />
        </geometry>
      </visual>
    </link>

    <link name="base_link">
      <visual>
        <geometry>
          <cylinder radius="${base_link_radius}" length="${base_link_length}" />
        </geometry>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <material name="yellow">
          <color rgba="0.5 0.3 0.0 0.5" />
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${base_link_radius}" length="${base_link_length}" />
        </geometry>
        <origin xyz="0 0 0" rpy="0 0 0" />
      </collision>
      <xacro:cylinder_inertial_matrix m="${base_link_m}" r="${base_link_radius}" h="${base_link_length}" />

    </link>


    <joint name="base_link2base_footprint" type="fixed">
      <parent link="base_footprint" />
      <child link="base_link" />
      <origin xyz="0 0 ${earth_space + base_link_length / 2 }" />
    </joint>
    <gazebo reference="base_link">
        <material>Gazebo/Yellow</material>
    </gazebo>

    <!-- 驱动轮 -->
    <!-- 驱动轮属性 -->
    <xacro:property name="wheel_radius" value="0.0325" /><!-- 半径 -->
    <xacro:property name="wheel_length" value="0.015" /><!-- 宽度 -->
    <xacro:property name="wheel_m" value="0.05" /> <!-- 质量  -->

    <!-- 驱动轮宏实现 -->
    <xacro:macro name="add_wheels" params="name flag">
      <link name="${name}_wheel">
        <visual>
          <geometry>
            <cylinder radius="${wheel_radius}" length="${wheel_length}" />
          </geometry>
          <origin xyz="0.0 0.0 0.0" rpy="${PI / 2} 0.0 0.0" />
          <material name="black" />
        </visual>
        <collision>
          <geometry>
            <cylinder radius="${wheel_radius}" length="${wheel_length}" />
          </geometry>
          <origin xyz="0.0 0.0 0.0" rpy="${PI / 2} 0.0 0.0" />
        </collision>
        <xacro:cylinder_inertial_matrix m="${wheel_m}" r="${wheel_radius}" h="${wheel_length}" />

      </link>

      <joint name="${name}_wheel2base_link" type="continuous">
        <parent link="base_link" />
        <child link="${name}_wheel" />
        <origin xyz="0 ${flag * base_link_radius} ${-(earth_space + base_link_length / 2 - wheel_radius) }" />
        <axis xyz="0 1 0" />
      </joint>

      <gazebo reference="${name}_wheel">
        <material>Gazebo/Red</material>
      </gazebo>

    </xacro:macro>
    <xacro:add_wheels name="left" flag="1" />
    <xacro:add_wheels name="right" flag="-1" />
    <!-- 支撑轮 -->
    <!-- 支撑轮属性 -->
    <xacro:property name="support_wheel_radius" value="0.0075" /> <!-- 支撑轮半径 -->
    <xacro:property name="support_wheel_m" value="0.03" /> <!-- 质量  -->

    <!-- 支撑轮宏 -->
    <xacro:macro name="add_support_wheel" params="name flag" >
      <link name="${name}_wheel">
        <visual>
            <geometry>
                <sphere radius="${support_wheel_radius}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="black" />
        </visual>
        <collision>
            <geometry>
                <sphere radius="${support_wheel_radius}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
        </collision>
        <xacro:sphere_inertial_matrix m="${support_wheel_m}" r="${support_wheel_radius}" />
      </link>

      <joint name="${name}_wheel2base_link" type="continuous">
          <parent link="base_link" />
          <child link="${name}_wheel" />
          <origin xyz="${flag * (base_link_radius - support_wheel_radius)} 0 ${-(base_link_length / 2 + earth_space / 2)}" />
          <axis xyz="1 1 1" />
      </joint>
      <gazebo reference="${name}_wheel">
        <material>Gazebo/Red</material>
      </gazebo>
    </xacro:macro>

    <xacro:add_support_wheel name="front" flag="1" />
    <xacro:add_support_wheel name="back" flag="-1" />


</robot>

```

#### 5.3.3 摄像头xacro文件

car_camera.urdf.xacro

```xml
<!-- 摄像头相关的 xacro 文件 -->
<robot name="my_camera" xmlns:xacro="http://wiki.ros.org/xacro">
    <!-- 摄像头属性 -->
    <xacro:property name="camera_length" value="0.01" /> <!-- 摄像头长度(x) -->
    <xacro:property name="camera_width" value="0.025" /> <!-- 摄像头宽度(y) -->
    <xacro:property name="camera_height" value="0.025" /> <!-- 摄像头高度(z) -->
    <xacro:property name="camera_x" value="0.08" /> <!-- 摄像头安装的x坐标 -->
    <xacro:property name="camera_y" value="0.0" /> <!-- 摄像头安装的y坐标 -->
    <xacro:property name="camera_z" value="${base_link_length / 2 + camera_height / 2}" /> <!-- 摄像头安装的z坐标:底盘高度 / 2 + 摄像头高度 / 2  -->

    <xacro:property name="camera_m" value="0.01" /> <!-- 摄像头质量 -->

    <!-- 摄像头关节以及link -->
    <link name="camera">
        <visual>
            <geometry>
                <box size="${camera_length} ${camera_width} ${camera_height}" />
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
            <material name="black" />
        </visual>
        <collision>
            <geometry>
                <box size="${camera_length} ${camera_width} ${camera_height}" />
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
        </collision>
        <xacro:Box_inertial_matrix m="${camera_m}" l="${camera_length}" w="${camera_width}" h="${camera_height}" />
    </link>

    <joint name="camera2base_link" type="fixed">
        <parent link="base_link" />
        <child link="camera" />
        <origin xyz="${camera_x} ${camera_y} ${camera_z}" />
    </joint>
    <gazebo reference="camera">
        <material>Gazebo/Blue</material>
    </gazebo>
</robot>

```



#### 5.3.4雷达xacro文件

car_laser.urdf.xacro

```xml
<!--
    小车底盘添加雷达
-->
<robot name="my_laser" xmlns:xacro="http://wiki.ros.org/xacro">

    <!-- 雷达支架 -->
    <xacro:property name="support_length" value="0.15" /> <!-- 支架长度 -->
    <xacro:property name="support_radius" value="0.01" /> <!-- 支架半径 -->
    <xacro:property name="support_x" value="0.0" /> <!-- 支架安装的x坐标 -->
    <xacro:property name="support_y" value="0.0" /> <!-- 支架安装的y坐标 -->
    <xacro:property name="support_z" value="${base_link_length / 2 + support_length / 2}" /> <!-- 支架安装的z坐标:底盘高度 / 2 + 支架高度 / 2  -->

    <xacro:property name="support_m" value="0.02" /> <!-- 支架质量 -->

    <link name="support">
        <visual>
            <geometry>
                <cylinder radius="${support_radius}" length="${support_length}" />
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
            <material name="red">
                <color rgba="0.8 0.2 0.0 0.8" />
            </material>
        </visual>

        <collision>
            <geometry>
                <cylinder radius="${support_radius}" length="${support_length}" />
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
        </collision>

        <xacro:cylinder_inertial_matrix m="${support_m}" r="${support_radius}" h="${support_length}" />

    </link>

    <joint name="support2base_link" type="fixed">
        <parent link="base_link" />
        <child link="support" />
        <origin xyz="${support_x} ${support_y} ${support_z}" />
    </joint>

    <gazebo reference="support">
        <material>Gazebo/White</material>
    </gazebo>

    <!-- 雷达属性 -->
    <xacro:property name="laser_length" value="0.05" /> <!-- 雷达长度 -->
    <xacro:property name="laser_radius" value="0.03" /> <!-- 雷达半径 -->
    <xacro:property name="laser_x" value="0.0" /> <!-- 雷达安装的x坐标 -->
    <xacro:property name="laser_y" value="0.0" /> <!-- 雷达安装的y坐标 -->
    <xacro:property name="laser_z" value="${support_length / 2 + laser_length / 2}" /> <!-- 雷达安装的z坐标:支架高度 / 2 + 雷达高度 / 2  -->

    <xacro:property name="laser_m" value="0.1" /> <!-- 雷达质量 -->

    <!-- 雷达关节以及link -->
    <link name="laser">
        <visual>
            <geometry>
                <cylinder radius="${laser_radius}" length="${laser_length}" />
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
            <material name="black" />
        </visual>
        <collision>
            <geometry>
                <cylinder radius="${laser_radius}" length="${laser_length}" />
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
        </collision>
        <xacro:cylinder_inertial_matrix m="${laser_m}" r="${laser_radius}" h="${laser_length}" />
    </link>

    <joint name="laser2support" type="fixed">
        <parent link="support" />
        <child link="laser" />
        <origin xyz="${laser_x} ${laser_y} ${laser_z}" />
    </joint>
    <gazebo reference="laser">
        <material>Gazebo/Black</material>
    </gazebo>
</robot>

```

#### 5.3.5融合上面4个xacro

car.urdf.xacro

```xml
<!-- 组合小车底盘与摄像头 -->
<robot name="my_car_camera" xmlns:xacro="http://wiki.ros.org/xacro">
    <xacro:include filename="inertial.xacro" />
    <xacro:include filename="car_base.urdf.xacro" />
    <xacro:include filename="car_camera.urdf.xacro" />
    <xacro:include filename="car_laser.urdf.xacro" />
</robot>
```

#### 5.3.4 Launch file

```xml
<launch>
    <!-- 将 Urdf 文件的内容加载到参数服务器 -->
    <param name="robot_description" command="$(find xacro)/xacro $(find demo02_urdf_gazebo)/urdf/car.urdf.xacro" />
    <!-- 启动 gazebo -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch" />

    <!-- 在 gazebo 中显示机器人模型 -->
    <node pkg="gazebo_ros" type="spawn_model" name="model" args="-urdf -model mycar -param robot_description"  />
</launch>
```



### 5.4 Gazebo仿真环境搭建

Gazebo 中创建仿真实现方式有2种:

- 方式1: 直接添加内置组件创建仿真环境
- 方式2: 手动绘制仿真环境(更为灵活)

#### 5.4.1 添加内置组件

1. 启动gazebo后在insert中插入各种组件

2. 保存仿真环境file--->save world as到功能包下worlds目录，后缀名为.world

3. launch file:

   ```xml
   <launch>
   
       <!-- 将 Urdf 文件的内容加载到参数服务器 -->
       <param name="robot_description" command="$(find xacro)/xacro $(find demo02_urdf_gazebo)/urdf/xacro/my_base_camera_laser.urdf.xacro" />
       <!-- 启动 gazebo -->
       <!--启动 empty_world 后，再根据arg加载自定义的仿真环境-->
       <include file="$(find gazebo_ros)/launch/empty_world.launch">
           <arg name="world_name" value="$(find demo02_urdf_gazebo)/worlds/hello.world" />
       </include>
   
       <!-- 在 gazebo 中显示机器人模型 -->
       <node pkg="gazebo_ros" type="spawn_model" name="model" args="-urdf -model mycar -param robot_description"  />
   </launch>
   
   ```

#### 5.4.2 自定义仿真环境

1. 启动gazebo后点击edit---->Building Editor 后绘制仿真环境

2. 保存仿真环境file--->save 到功能包下models目录

   然后 file ---> Exit Building Editor

3. 保存仿真环境file--->save world as到功能包下worlds目录，后缀名为.world

4. launch file和5.4.1一样

#### 5.4.3使用官方提供的插件

1. 下载模型库：`git clone https://github.com/osrf/gazebo_models`
2. 将得到的gazebo_models文件夹内容复制到 /usr/share/gazebo-*/models
3. 重启 Gazebo，选择左侧菜单栏的 insert 可以选择并插入相关道具了



# 六、URDF,Gazebo和Rviz综合应用

URDF 用于创建机器人模型、Rviz 可以显示机器人感知到的环境信息，Gazebo 用于仿真，可以模拟外界环境，以及机器人的一些传感器。下面设置Gazebo中的传感器。

[gazebo教程](https://classic.gazebosim.org/tutorials)

模型基于五、5.3

## 1,机器人运动控制以及里程计信息显示

- gazebo 已经实现了 ros_control 的相关接口，如果需要在 gazebo 中控制机器人运动，直接调用相关接口即可。ros_control 相关见上面`四、1.ROS Controller`
- 运动控制基本流程：
  1. 为已经创建完毕的机器人模型，编写一个单独的 xacro 文件，为机器人模型添加传动装置以及控制器
  2. 将此文件集成进xacro文件
  3. 启动 Gazebo 并发布 /cmd_vel 消息控制机器人运动

### 1.1模型修改

模型基于五、5.3

#### 1.1.1为joint添加传动装置以及控制器

move.urdf.xacro

```xml
<robot name="my_car_move" xmlns:xacro="http://wiki.ros.org/xacro">

    <!-- 传动实现:用于连接控制器与关节 -->
    <xacro:macro name="joint_trans" params="joint_name">
        <!-- Transmission is important to link the joints and the controller -->
        <transmission name="${joint_name}_trans">
            <type>transmission_interface/SimpleTransmission</type>
            <joint name="${joint_name}">
                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
            </joint>
            <actuator name="${joint_name}_motor">
                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
                <mechanicalReduction>1</mechanicalReduction>
            </actuator>
        </transmission>
    </xacro:macro>

    <!-- 每一个驱动轮都需要配置传动装置 -->
    <xacro:joint_trans joint_name="left_wheel2base_link" />
    <xacro:joint_trans joint_name="right_wheel2base_link" />

    <!-- 控制器 -->
    <gazebo>
        <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
            <rosDebugLevel>Debug</rosDebugLevel>
            <publishWheelTF>true</publishWheelTF>
            <robotNamespace>/</robotNamespace>
            <publishTf>1</publishTf>
            <publishWheelJointState>true</publishWheelJointState>
            <alwaysOn>true</alwaysOn>
            <updateRate>100.0</updateRate>
            <legacyMode>true</legacyMode>
            <leftJoint>left_wheel2base_link</leftJoint> <!-- 左轮 -->
            <rightJoint>right_wheel2base_link</rightJoint> <!-- 右轮 -->
            <wheelSeparation>${base_link_radius * 2}</wheelSeparation> <!-- 车轮间距 -->
            <wheelDiameter>${wheel_radius * 2}</wheelDiameter> <!-- 车轮直径 -->
            <broadcastTF>1</broadcastTF>
            <wheelTorque>30</wheelTorque>
            <wheelAcceleration>1.8</wheelAcceleration>
            <commandTopic>cmd_vel</commandTopic> <!-- 运动控制话题 -->
            <odometryFrame>odom</odometryFrame> 
            <odometryTopic>odom</odometryTopic> <!-- 里程计话题 -->
            <robotBaseFrame>base_footprint</robotBaseFrame> <!-- 根坐标系 -->
        </plugin>
    </gazebo>

</robot>

```

#### 1.1.2 xacro文件集成

```xml
<robot name="my_car_camera" xmlns:xacro="http://wiki.ros.org/xacro">
    <xacro:include filename="my_head.urdf.xacro" />
    <xacro:include filename="my_base.urdf.xacro" />
    <xacro:include filename="my_camera.urdf.xacro" />
    <xacro:include filename="my_laser.urdf.xacro" />
    <!--将1.1.1的控制器和传动配置xacro合并进来-->
    <xacro:include filename="move.urdf.xacro" />
</robot>
```

#### 1.1.3 launch file

```xml
<launch>

    <!-- 将 Urdf 文件的内容加载到参数服务器 -->
    <param name="robot_description" command="$(find xacro)/xacro $(find demo02_urdf_gazebo)/urdf/xacro/my_base_camera_laser.urdf.xacro" />
    
    <!-- 启动 gazebo -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find demo02_urdf_gazebo)/worlds/hello.world" />
    </include>

    <!-- 在 gazebo 中显示机器人模型 -->
    <node pkg="gazebo_ros" type="spawn_model" name="model" args="-urdf -model mycar -param robot_description"  />
</launch>

```

可能遇到的问题：

- error: SpawnModel: Failure - model name **mycar** already exist

  原因：保存的world中把小车一起保存了，所以在创建mycar就重复了

  2种方法解决：

  1. 删除第八行
  2. 修改第12行 -model mycar 为-model xxx

#### 1.1.4 尝试控制

- 可以通过topic list查看话题列表，会发现多了 /cmd_vel

- 发送topic来控制

  也可以编写单独的节点控制

  ```shell
  rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}'
  ```

### 1.2 Rviz查看里程计

- **Odometer里程计:** 机器人相对出发点坐标系的位姿状态(X 坐标 Y 坐标 Z坐标以及朝向)。
- 在 Gazebo 的仿真环境中，机器人的里程计信息以及运动朝向等信息是无法获取的，可以通过 Rviz 显示机器人的里程计信息以及运动朝向

#### 1.2.1 添加rviz.launch

```xml
<launch>
    <!-- 启动 rviz -->
    <node pkg="rviz" type="rviz" name="rviz" />

    <!-- 关节以及机器人状态发布节点 -->
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

</launch>

```

#### 1.2.2 在rviz中添加组件

1. Global Options标签:

   将fixed frame改为odom

2. 添加RobotModel标签

3. 添加Odometry标签

   设置topic为/odom



## 2.雷达信息仿真以及显示

- 通过 Gazebo 模拟激光雷达传感器，并在 Rviz 中显示激光数据。
- 雷达仿真基本流程:
  1. 为已经创建完毕的机器人模型，编写一个单独的 xacro 文件，为机器人模型添加雷达配置；
  2. 将此文件集成进xacro文件；
  3. 启动 Gazebo，使用 Rviz 显示雷达信息。

### 2.1 模型修改

#### 2.1.1 配置雷达传感器xacro

sensors_laser.urdf.xacro

```xml
<robot name="my_sensors" xmlns:xacro="http://wiki.ros.org/xacro">

  <!-- 雷达 -->
  <gazebo reference="laser">
    <sensor type="ray" name="rplidar">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>5.5</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3</min_angle>
            <max_angle>3</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_rplidar" filename="libgazebo_ros_laser.so">
        <topicName>/scan</topicName>
        <frameName>laser</frameName>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

#### 2.1.2 xacro集成

```xml
<!-- 组合小车底盘与传感器 -->
<robot name="my_car_camera" xmlns:xacro="http://wiki.ros.org/xacro">
    <xacro:include filename="my_head.urdf.xacro" />
    <xacro:include filename="my_base.urdf.xacro" />
    <xacro:include filename="my_camera.urdf.xacro" />
    <xacro:include filename="my_laser.urdf.xacro" />
    <xacro:include filename="move.urdf.xacro" />
    <!-- 雷达仿真的 xacro 文件 -->
    <xacro:include filename="my_sensors_laser.urdf.xacro" />
</robot>
```

#### 2.1.3launch文件

和1.1.3一样

```xml
<launch>

    <!-- 将 Urdf 文件的内容加载到参数服务器 -->
    <param name="robot_description" command="$(find xacro)/xacro $(find demo02_urdf_gazebo)/urdf/xacro/my_base_camera_laser.urdf.xacro" />
    
    <!-- 启动 gazebo -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find demo02_urdf_gazebo)/worlds/hello.world" />
    </include>

    <!-- 在 gazebo 中显示机器人模型 -->
    <node pkg="gazebo_ros" type="spawn_model" name="model" args="-urdf -model mycar -param robot_description"  />
</launch>

```



### 2.2 rviz显示雷达数据

1. Global Options标签:

   将fixed frame改为odom

2. 添加雷达信息显示插件laserscan

   将topic改为scan

## 3. 摄像头信息仿真及显示

- 通过 Gazebo 模拟摄像头传感器，并在 Rviz 中显示摄像头数据。
- 摄像头仿真基本流程:
  1. 为已经创建完毕的机器人模型，编写一个单独的 xacro 文件，为机器人模型添加摄像头配置；
  2. 将此文件集成进xacro文件；
  3. 启动 Gazebo，使用 Rviz 显示摄像头信息。

### 3.1Gazebo仿真摄像头

#### 3.1.1配置摄像头传感器xacro

sensors_camera.urdf.xacro

```xml
<robot name="my_sensors" xmlns:xacro="http://wiki.ros.org/xacro">
  <!-- 被引用的link -->
  <gazebo reference="camera">
    <!-- 类型设置为 camara -->
    <sensor type="camera" name="camera_node">
      <update_rate>30.0</update_rate> <!-- 更新频率 -->
      <!-- 摄像头基本信息设置 -->
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>1280</width>
          <height>720</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <!-- 核心插件 -->
      <plugin name="gazebo_camera" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>/camera</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

#### 3.1.2xacro文件集成

```xml
<robot name="my_car_camera" xmlns:xacro="http://wiki.ros.org/xacro">
    <xacro:include filename="my_head.urdf.xacro" />
    <xacro:include filename="my_base.urdf.xacro" />
    <xacro:include filename="my_camera.urdf.xacro" />
    <xacro:include filename="my_laser.urdf.xacro" />
    <xacro:include filename="move.urdf.xacro" />
    <!-- 摄像头仿真的 xacro 文件 -->
    <xacro:include filename="sensors_camara.urdf.xacro" />
</robot>
```

#### 3.1.3launch file

和1.1.3，2.1.3一致

### 3.2 Rviz显示摄像头数据

1. Global Options标签:

   将fixed frame改为odom

2. 添加camera标签

   将topic改为/camera/image_raw。

   topic的信息在urdf中设置的

## 4. Kinect深度相机仿真与显示

- 通过 Gazebo 模拟kinect摄像头，并在 Rviz 中显示kinect摄像头数据。
- kinect摄像头仿真基本流程:
  1. 为已经创建完毕的机器人模型，编写一个单独的 xacro 文件，为机器人模型添加kinect摄像头配置；
  2. 将此文件集成进xacro文件；
  3. 启动 Gazebo，使用 Rviz 显示kinect摄像头信息。

### 4.1 Gazebo仿真kinect

#### 4.1.1 配置kinetic传感器xacro

sensors_kinect.urdf.xacro

```xml
<robot name="my_sensors" xmlns:xacro="http://wiki.ros.org/xacro">
    <gazebo reference="support">  
      <sensor type="depth" name="camera">
        <always_on>true</always_on>
        <update_rate>20.0</update_rate>
        <camera>
          <horizontal_fov>${60.0*PI/180.0}</horizontal_fov>
          <image>
            <format>R8G8B8</format>
            <width>640</width>
            <height>480</height>
          </image>
          <clip>
            <near>0.05</near>
            <far>8.0</far>
          </clip>
        </camera>
        <plugin name="kinect_camera_controller" filename="libgazebo_ros_openni_kinect.so">
          <cameraName>camera</cameraName>
          <alwaysOn>true</alwaysOn>
          <updateRate>10</updateRate>
          <imageTopicName>rgb/image_raw</imageTopicName>
          <depthImageTopicName>depth/image_raw</depthImageTopicName>
          <pointCloudTopicName>depth/points</pointCloudTopicName>
          <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
          <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
          <frameName>support</frameName>
          <baseline>0.1</baseline>
          <distortion_k1>0.0</distortion_k1>
          <distortion_k2>0.0</distortion_k2>
          <distortion_k3>0.0</distortion_k3>
          <distortion_t1>0.0</distortion_t1>
          <distortion_t2>0.0</distortion_t2>
          <pointCloudCutoff>0.4</pointCloudCutoff>
        </plugin>
      </sensor>
    </gazebo>

</robot>
```

#### 4.1.2xacro文件集成

```xml
<!-- 组合小车底盘与传感器 -->
<robot name="my_car_camera" xmlns:xacro="http://wiki.ros.org/xacro">
    <xacro:include filename="my_head.urdf.xacro" />
    <xacro:include filename="my_base.urdf.xacro" />
    <xacro:include filename="my_camera.urdf.xacro" />
    <xacro:include filename="my_laser.urdf.xacro" />
    <xacro:include filename="move.urdf.xacro" />
    <!-- kinect仿真的 xacro 文件 -->
    <xacro:include filename="sensors_kinect.urdf.xacro" />
</robot>

```

#### 4.1.3launch file

和前面x.1.3一致

### 4.2 rviz 显示kinect数据

1. Global Options标签:

   将fixed frame改为odom

2. 添加camera标签

   将topic改为/camera/depth/image_raw。

   topic的信息在urdf中设置的

### 4.3kinect点云数据显示

- 在rviz添加PointCloud2标签

  将topic改为/camera/depth/points

- 问题：在rviz中显示时错位。

  原因：在kinect中图像数据与点云数据使用了两套坐标系统，且两套坐标系统位姿并不一致。

  两步解决：

  1. 在插件中为kinect设置坐标系，修改配置文件的`<frameName>`标签内容：

     ```
     <frameName>support_depth</frameName>
     ```

  2. 发布新设置的坐标系到kinect连杆的坐标变换关系，在启动rviz的launch中，添加:

     ```xml
     <node pkg="tf2_ros" type="static_transform_publisher" name="static_transform_publisher" args="0 0 0 -1.57 0 -1.57 /support /support_depth" />
     
     ```



# 七、机器人导航

导航是机器人系统中最重要的模块之一，比如现在较为流行的服务型室内机器人，就是依赖于机器人导航来实现室内自主移动的

## 1. 基本概念：

在ROS中机器人导航([Navigation](http://wiki.ros.org/navigation))由多个功能包组合实现，ROS 中又称之为导航功能包集。

导航其实就是机器人自主的从 A 点移动到 B 点的过程。

[nav2官方文档](https://navigation.ros.org/index.html#)

### 1.1 导航模块简介

![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/ROS/navigation.png?raw=true)

假定我们已经以特定方式配置机器人，导航功能包集将使其可以运动。上图概述了这种配置方式。白色的部分是必须且已实现的组件，灰色的部分是可选且已实现的组件，蓝色的部分是必须为每一个机器人平台创建的组件。

涉及的关键技术有如下五点:

1. 全局地图
2. 自身定位
3. 路径规划
4. 运动控制
5. 环境感知

#### 1.1.1 全局地图

如果要使用地图，首先需要绘制地图。关于建图SLAM 的理论脱颖而出：

1. **SLAM**(simultaneous localization and mapping),也称为CML (Concurrent Mapping and Localization), 即时定位与地图构建，或并发建图与定位。SLAM问题可以描述为: 机器人在未知环境中从一个未知位置开始移动,在移动过程中根据位置估计和地图进行自身定位，同时在自身定位的基础上建造增量式地图，以绘制出外部环境的完全地图。
2. 在 ROS 中，较为常用的 SLAM 实现也比较多，比如: gmapping、hector_slam、cartographer、rgbdslam、ORB_SLAM ....
3. 如果要完成 SLAM ，机器人必须要具备感知外界环境的能力，尤其是要具备获取周围环境深度信息的能力。感知的实现需要依赖于传感器，比如: 激光雷达、摄像头、RGB-D摄像头...
4. SLAM 可以用于地图生成，而生成的地图还需要被保存以待后续使用，在 ROS 中保存地图的功能包是 **map_server**
5. 注意：SLAM 只是实现地图构建和即时定位。和导航不等价

#### 1.1.2 自身定位

SLAM 就可以实现自身定位，除此之外，ROS 中还提供了一个用于定位的功能包: amcl

**amcl**(adaptiveMonteCarloLocalization)自适应的蒙特卡洛定位,是用于2D移动机器人的概率定位系统。它实现了自适应（或KLD采样）蒙特卡洛定位方法，该方法使用粒子过滤器根据已知地图跟踪机器人的姿态。

#### 1.1.3 路径规划

- 导航就是机器人从A点运动至B点的过程，在这一过程中，机器人需要根据目标位置计算全局运动路线，并且在运动过程中，还需要时时根据出现的一些动态障碍物调整运动路线，直至到达目标点，该过程就称之为路径规划。

- 在 ROS 中提供了 move_base 包来实现路径规则,该功能包主要由两大规划器组成:

  1. 全局路径规划(gloable_planner)

     根据给定的目标点和全局地图实现总体的路径规划，使用 Dijkstra 或 A* 算法进行全局路径规划，计算最优路线，作为全局路线

  2. 本地时时规划(local_planner)

     在实际导航过程中，机器人可能无法按照给定的全局最优路线运行，比如:机器人在运行中，可能会随时出现一定的障碍物... 本地规划的作用就是使用一定算法(Dynamic Window Approaches) 来实现障碍物的规避，并选取当前最优路径以尽量符合全局最优路径

  全局路径规划与本地路径规划是相对的，全局路径规划侧重于全局、宏观实现，而本地路径规划侧重与当前、微观实现。

#### 1.1.4 运动控制

导航功能包集假定它可以通过话题"cmd_vel"发布`geometry_msgs/Twist`类型的消息，这个消息基于机器人的基座坐标系，它传递的是运动命令。这意味着必须有一个节点订阅"cmd_vel"话题， 将该话题上的速度命令转换为电机命令并发送。

#### 1.1.5 环境感知

感知周围环境信息，比如: 摄像头、激光雷达、编码器...，摄像头、激光雷达可以用于感知外界环境的深度信息，编码器可以感知电机的转速信息，进而可以获取速度信息并生成里程计信息。

在导航功能包集中，环境感知也是一重要模块实现，它为其他模块提供了支持。其他模块诸如: SLAM、amcl、move_base 都需要依赖于环境感知。



### 1.2 导航之坐标系

#### 1.2.1 简介

定位是导航中的重要实现之一，所谓定位，就是参考某个坐标系(比如:以机器人的出发点为原点创建坐标系)在该坐标系中标注机器人。定位原理看似简单，但是这个这个坐标系不是客观存在的，我们也无法以上帝视角确定机器人的位姿，定位实现需要依赖于机器人自身，机器人需要逆向推导参考系原点并计算坐标系相对关系，该过程实现常用方式有两种:

- 通过里程计定位:时时收集机器人的速度信息计算并发布机器人坐标系与父级参考系的相对关系。
- 通过传感器定位:通过传感器收集外界环境信息通过匹配计算并发布机器人坐标系与父级参考系的相对关系。

两种方式在导航中都会经常使用。

#### 1.2.2 特点

两种定位方式都有各自的优缺点。

里程计定位:

- 优点:里程计定位信息是连续的，没有离散的跳跃。
- 缺点:里程计存在累计误差，不利于长距离或长期定位。

传感器定位:

- 优点:比里程计定位更精准；
- 缺点:传感器定位会出现跳变的情况，且传感器定位在标志物较少的环境下，其定位精度会大打折扣。

两种定位方式优缺点互补，应用时一般二者结合使用。

#### 1.2.3 坐标变换

上述两种定位实现中，机器人坐标系一般使用机器人模型中的根坐标系(base_link 或 base_footprint)，里程计定位时，父级坐标系一般称之为 odom，如果通过传感器定位，父级参考系一般称之为 map。当二者结合使用时，map 和 odom 都是机器人模型根坐标系的父级，这是不符合坐标变换中"单继承"的原则的，所以，一般会将转换关系设置为: map -> doom -> base_link 或 base_footprint。

### 1.3 导航条件说明

#### 1.3.1 硬件

虽然导航功能包集被设计成尽可能的通用，在使用时仍然有三个主要的硬件限制：

1. 它是为差速驱动的轮式机器人设计的。它假设底盘受到理想的运动命令的控制并可实现预期的结果，命令的格式为：x速度分量，y速度分量，角速度(theta)分量。
2. 它需要在底盘上安装一个单线激光雷达。这个激光雷达用于构建地图和定位。
3. 导航功能包集是为正方形的机器人开发的，所以方形或圆形的机器人将是性能最好的。 它也可以工作在任意形状和大小的机器人上，但是较大的机器人将很难通过狭窄的空间。

#### 1.3.2 软件

导航功能实现之前，需要搭建一些软件环境:

1. 毋庸置疑的，必须先要安装 ROS

2. 当前导航基于仿真环境，先保证上一章的机器人系统仿真可以正常执行

   在仿真环境下，机器人可以正常接收 /cmd_vel 消息，并发布里程计消息，传感器消息发布也正常，也即导航模块中的运动控制和环境感知实现完毕

后续导航实现中，我们主要关注于: 使用 SLAM 绘制地图、地图服务、自身定位与路径规划。



## 2. 导航实现

- 准备工作
  - 安装 gmapping 包(用于构建地图):`sudo apt install ros-<ROS版本>-gmapping`
  - 安装地图服务包(用于保存与读取地图):`sudo apt install ros-<ROS版本>-map-server`
  - 安装 navigation 包(用于定位以及路径规划):`sudo apt install ros-<ROS版本>-navigation`
  - 新建功能包，并导入依赖: gmapping map_server amcl move_base

### 1. Slam建图

- [Gmapping](http://wiki.ros.org/gmapping)是一个基于**2D激光雷达**使用**RBPF**（Rao-Blackwellized Particle Filters）算法完成**二维栅格地图**构建的SLAM算法。
  - 优点：gmapping可以实时构建室内环境地图，在小场景中计算量少，且地图精度较高，对激光雷达扫描频率要求较低。
  - 缺点：随着环境的增大，构建地图所需的内存和计算量就会变得巨大，所以gmapping不适合大场景构图。一个直观的感受是，对于200x200米的范围，如果栅格分辨率是5cm，每个栅格占用一个字节内存，那么每个粒子携带的地图都要16M的内存，如果是100粒子就是1.6G内存。
  - [参考](https://blog.csdn.net/zhao_ke_xue/article/details/108944811)

#### 1.1 Gmapping 节点

gmapping 功能包中的核心节点是:slam_gmapping

##### 1订阅的Topic

tf (tf/tfMessage)

- 用于雷达、底盘与里程计之间的坐标变换消息。

scan(sensor_msgs/LaserScan)

- SLAM所需的雷达信息。

##### 2发布的Topic

map_metadata(nav_msgs/MapMetaData)

- 地图元数据，包括地图的宽度、高度、分辨率等，该消息会固定更新。

map(nav_msgs/OccupancyGrid)

- 地图栅格数据，一般会在rviz中以图形化的方式显示。

~entropy(std_msgs/Float64)

- 机器人姿态分布熵估计(值越大，不确定性越大)。

##### 3服务

dynamic_map(nav_msgs/GetMap)

- 用于获取地图数据。

##### 4参数

~base_frame(string, default:"base_link")

- 机器人基坐标系。

~map_frame(string, default:"map")

- 地图坐标系。

~odom_frame(string, default:"odom")

- 里程计坐标系。

~map_update_interval(float, default: 5.0)

- 地图更新频率，根据指定的值设计更新间隔。

~maxUrange(float, default: 80.0)

- 激光探测的最大可用范围(超出此阈值，被截断)。

~maxRange(float)

- 激光探测的最大范围。

.... 参数较多，上述是几个较为常用的参数，其他参数介绍可参考官网。

##### 5所需的坐标变换

雷达坐标系→基坐标系

- 一般由 robot_state_publisher 或 static_transform_publisher 发布。

基坐标系→里程计坐标系

- 一般由里程计节点发布。

##### 6发布的坐标变换

地图坐标系→里程计坐标系

- 地图到里程计坐标系之间的变换。

#### 1.2 Gmapping 使用

##### 1. Launch file

[官方参考](https://github.com/ros-perception/slam_gmapping/blob/melodic-devel/gmapping/launch/slam_gmapping_pr2.launch)，修改如下：

slam.launch

```xml
<launch>
    <!--仿真环境下设置改参数为true-->
    <param name="use_sim_time" value="true"/>
    <!--gmapping节点-->
    <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
      <!--接收雷达信息的topic-->
      <remap from="scan" to="scan"/>
      <!--参考上面说的参数要求，设置坐标系-->
      <!--一共要有3个坐标系，其中地图就用默认的map就好-->
      <param name="base_frame" value="base_footprint"/><!--底盘坐标系-->
      <param name="odom_frame" value="odom"/> <!--里程计坐标系-->
      <param name="map_update_interval" value="5.0"/>
      <param name="maxUrange" value="16.0"/>
      <param name="sigma" value="0.05"/>
      <param name="kernelSize" value="1"/>
      <param name="lstep" value="0.05"/>
      <param name="astep" value="0.05"/>
      <param name="iterations" value="5"/>
      <param name="lsigma" value="0.075"/>
      <param name="ogain" value="3.0"/>
      <param name="lskip" value="0"/>
      <param name="srr" value="0.1"/>
      <param name="srt" value="0.2"/>
      <param name="str" value="0.1"/>
      <param name="stt" value="0.2"/>
      <param name="linearUpdate" value="1.0"/>
      <param name="angularUpdate" value="0.5"/>
      <param name="temporalUpdate" value="3.0"/>
      <param name="resampleThreshold" value="0.5"/>
      <param name="particles" value="30"/>
      <param name="xmin" value="-50.0"/>
      <param name="ymin" value="-50.0"/>
      <param name="xmax" value="50.0"/>
      <param name="ymax" value="50.0"/>
      <param name="delta" value="0.05"/>
      <param name="llsamplerange" value="0.01"/>
      <param name="llsamplestep" value="0.01"/>
      <param name="lasamplerange" value="0.005"/>
      <param name="lasamplestep" value="0.005"/>
    </node>
	<!--发送坐标变换-->
    <node pkg="joint_state_publisher" name="joint_state_publisher" type="joint_state_publisher" />
    <node pkg="robot_state_publisher" name="robot_state_publisher" type="robot_state_publisher" />

    <node pkg="rviz" type="rviz" name="rviz" />
    <!-- 可以保存 rviz 配置并后期直接使用-->
    <!--
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find my_nav_sum)/rviz/gmapping.rviz"/>
    -->
</launch>
```

##### 2. 执行

1. 和六、中一样先启动gazebo仿真环境

2. 启动上面的slam.launch

   `roslaunch nav_demo slam.launch`

3. 启动键盘控制

   `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

4. rviz中添加组件

   - global options中设置fixed frame为map
   - 添加map标签，topic为map

   

### 2. 地图服务(保存，读取)

- 上一节中地图数据是保存在内存中的，当节点关闭时，数据也会被一并释放，我们需要将栅格地图序列化到的磁盘以持久化存储，后期还要通过反序列化读取磁盘的地图数据再执行后续操作。
- 在ROS中，地图数据的序列化与反序列化可以通过 **map_server** 功能包实现。

#### 2.1 map_server的map_saver地图保存节点

##### 1. map_saver节点说明

订阅的topic：map(nav_msgs/OccupancyGrid)

- 订阅此话题用于生成地图文件

##### 2. launch file

```xml
<launch>
    <!--下面这参数：地图的保存路径以及保存的文件名称-->
    <arg name="filename" value="$(find nav_demo)/map/nav" />
    <node name="map_save" pkg="map_server" type="map_saver" args="-f $(arg filename)" />
</launch>
```

##### 3.使用和结果分析

- 使用：slam建图完成后，使用上面的launch文件

- 会生成两个文件

  1. xxx.pgm:本质是一张图片，直接使用图片查看程序即可打开

  2. xxx.yaml: 保存的是地图的元数据信息，用于描述图片

     ```yaml
     image: /home/yang/Desktop/ws_car/src/nav_demo/map/nav.pgm
     resolution: 0.050000
     origin: [-50.000000, -50.000000, 0.000000]
     negate: 0
     occupied_thresh: 0.65
     free_thresh: 0.196
     ```

     - **image**:被描述的图片资源路径，可以是绝对路径也可以是相对路径。
     - **resolution**: 图片分片率(单位: m/像素)。
     - **origin**: 地图中左下像素的二维姿势，为（x，y，偏航），偏航为逆时针旋转（偏航= 0表示无旋转）。
     - **occupied_thresh**: 占用概率大于此阈值的像素被视为完全占用。
     - **free_thresh**: 占用率小于此阈值的像素被视为完全空闲。
     - **negate**: 是否应该颠倒白色/黑色自由/占用的语义。

- map_server 中障碍物计算规则:

  1. 地图中的每一个像素取值在 [0,255] 之间，白色为 255，黑色为 0，该值设为 x；
  2. map_server 会将像素值作为判断是否是障碍物的依据，首先计算比例: p = (255 - x) / 255.0，白色为0，黑色为1(negate为true，则p = x / 255.0)；
  3. 根据步骤2计算的比例判断是否是障碍物，如果 p > occupied_thresh 那么视为障碍物，如果 p < free_thresh 那么视为无物。

#### 2.2 map_server的map_server地图服务

##### 1.map_server节点说明

- 发布的话题：

  - map_metadata（nav_msgs / MapMetaData）

    - 发布地图元数据。

  - map（nav_msgs / OccupancyGrid）

    - 地图数据。
- 服务：
  - static_map（nav_msgs / GetMap）
    - 通过此服务获取地图。
- 参数：
  - 〜frame_id（字符串，默认值：“map”）
    - 地图坐标系。

##### 2.地图读取launch file

通过 map_server 的 map_server 节点可以读取栅格地图数据

map_read.launch

```xml
<launch>
    <!-- 设置地图的配置文件 -->
    <!-- 读取的是2.2.1中生成的栅格地图-->
    <arg name="map" default="nav.yaml" />
    <!-- 运行地图服务器，并且加载设置的地图-->
    <node name="map_server" pkg="map_server" type="map_server" args="$(find nac_demo)/map/$(arg map)"/>
</launch>
```

该节点会发布话题:map(nav_msgs/OccupancyGrid)

##### 3.地图显示

- 运行上面的launch文件

- 打开rviz，添加map标签

  并设置topic为/map

### 3. 定位

- 定位就是推算机器人自身在全局地图中的位置
- SLAM中也包含定位算法实现，不过SLAM的定位是用于构建全局地图的，是属于导航开始之前的阶段
- 而当前定位是用于导航中，导航中，机器人需要按照设定的路线运动，通过定位可以判断机器人的实际轨迹是否符合预期。
- 在ROS的导航功能包集navigation中提供了 **amcl** 功能包，用于实现导航中的机器人定位。

#### 3.1 Amcl节点说明

- [AMCL](http://wiki.ros.org/amcl)(adaptive Monte Carlo Localization) 是用于2D移动机器人的概率定位系统，它实现了自适应（或KLD采样）蒙特卡洛定位方法，可以根据已有地图使用粒子滤波器推算机器人位置。

###### 3.1.1订阅的Topic

scan(sensor_msgs/LaserScan)

- 激光雷达数据。

tf(tf/tfMessage)

- 坐标变换消息。

initialpose(geometry_msgs/PoseWithCovarianceStamped)

- 用来初始化粒子滤波器的均值和协方差。

map(nav_msgs/OccupancyGrid)

- 获取地图数据。

###### 3.1.2发布的Topic

amcl_pose(geometry_msgs/PoseWithCovarianceStamped)

- 机器人在地图中的位姿估计。

particlecloud(geometry_msgs/PoseArray)

- 位姿估计集合，rviz中可以被 PoseArray 订阅然后图形化显示机器人的位姿估计集合。

tf(tf/tfMessage)

- 发布从 odom 到 map 的转换。

###### 3.1.3服务

global_localization(std_srvs/Empty)

- 初始化全局定位的服务。

request_nomotion_update(std_srvs/Empty)

- 手动执行更新和发布更新的粒子的服务。

set_map(nav_msgs/SetMap)

- 手动设置新地图和姿态的服务。

###### 3.1.4调用的服务

static_map(nav_msgs/GetMap)

- 调用此服务获取地图数据。

###### 3.1.5参数

~odom_model_type(string, default:"diff")

- 里程计模型选择: "diff","omni","diff-corrected","omni-corrected" (diff 差速、omni 全向轮)

~odom_frame_id(string, default:"odom")

- 里程计坐标系。

~base_frame_id(string, default:"base_link")

- 机器人极坐标系。

~global_frame_id(string, default:"map")

- 地图坐标系。

.... 参数较多，上述是几个较为常用的参数，其他参数介绍可参考官网。

###### 3.1.6坐标变换

里程计本身也是可以协助机器人定位的，不过里程计存在累计误差且一些特殊情况时(车轮打滑)会出现定位错误的情况，amcl 则可以通过估算机器人在地图坐标系下的姿态，再结合里程计提高定位准确度。

- 里程计定位:只是通过里程计数据实现 /odom_frame 与 /base_frame 之间的坐标变换。
- amcl定位: 可以提供 /map_frame 、/odom_frame 与 /base_frame 之间的坐标变换。

![](http://wiki.ros.org/amcl?action=AttachFile&do=get&target=amcl_localization.png)

#### 3.2 Amcl的使用

##### 3.2.1 launch file

- 查看参考示例

  ```
  roscd amcl
  cd examples
  ```

  该目录下会列出两个文件: amcl_diff.launch 和 amcl_omni.launch 文件，前者适用于差分移动机器人，后者适用于全向移动机器人，可以按需选择，此处参考前者.

- 修改上面的amcl_diff.launch如下：

  主要修改的是3.1中提到的那些参数

  amcl.launch

  ```xml
  <launch>
  <node pkg="amcl" type="amcl" name="amcl" output="screen">
    <!-- Publish scans from best pose at a max of 10 Hz -->
    <param name="odom_model_type" value="diff"/><!-- 里程计模式为差分 -->
    <param name="odom_alpha5" value="0.1"/>
    <param name="transform_tolerance" value="0.2" />
    <param name="gui_publish_rate" value="10.0"/>
    <param name="laser_max_beams" value="30"/>
    <param name="min_particles" value="500"/>
    <param name="max_particles" value="5000"/>
    <param name="kld_err" value="0.05"/>
    <param name="kld_z" value="0.99"/>
    <param name="odom_alpha1" value="0.2"/>
    <param name="odom_alpha2" value="0.2"/>
    <!-- translation std dev, m -->
    <param name="odom_alpha3" value="0.8"/>
    <param name="odom_alpha4" value="0.2"/>
    <param name="laser_z_hit" value="0.5"/>
    <param name="laser_z_short" value="0.05"/>
    <param name="laser_z_max" value="0.05"/>
    <param name="laser_z_rand" value="0.5"/>
    <param name="laser_sigma_hit" value="0.2"/>
    <param name="laser_lambda_short" value="0.1"/>
    <param name="laser_lambda_short" value="0.1"/>
    <param name="laser_model_type" value="likelihood_field"/>
    <!-- <param name="laser_model_type" value="beam"/> -->
    <param name="laser_likelihood_max_dist" value="2.0"/>
    <param name="update_min_d" value="0.2"/>
    <param name="update_min_a" value="0.5"/>
    <!--设置3个坐标系：odom，map，和 机器人基坐标系-->
    <param name="odom_frame_id" value="odom"/><!-- 里程计坐标系 -->
    <param name="base_frame_id" value="base_footprint"/><!-- 添加机器人基坐标系 -->
    <param name="global_frame_id" value="map"/><!-- 添加地图坐标系 -->
  
    <param name="resample_interval" value="1"/>
    <param name="transform_tolerance" value="0.1"/>
    <param name="recovery_alpha_slow" value="0.0"/>
    <param name="recovery_alpha_fast" value="0.0"/>
  </node>
  </launch>
  
  ```

- 编写测试launch 文件

  amcl节点是不可以单独运行的，运行 amcl 节点之前，需要先加载全局地图，然后启动 rviz 显示定位结果，上述节点可以集成进launch文件：

  integration.launch

  ```xml
  <launch>
      <!-- 设置地图的配置文件 -->
      <arg name="map" default="nav.yaml" />
      <!-- 运行地图服务器，并且加载设置的地图-->
      <node name="map_server" pkg="map_server" type="map_server" args="$(find nav_demo)/map/$(arg map)"/>
      <!-- 启动AMCL节点 -->
      <include file="$(find nav_demo)/launch/amcl.launch" />
      <!-- 运行rviz -->
      <node pkg="joint_state_publisher" name="joint_state_publisher" type="joint_state_publisher" />
      <node pkg="robot_state_publisher" name="robot_state_publisher" type="robot_state_publisher" />
      <node pkg="rviz" type="rviz" name="rviz"/>
  </launch>
  
  ```

##### 3.2.2执行

1. 启动gazebo仿真环境
2. 启动键盘控制`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
3. 启动上一步中集成地图服务、amcl 与 rviz 的 launch 文件integration.launch:
4. 在启动的 rviz 中，添加RobotModel、Map组件，分别显示机器人模型与地图，添加 posearray 插件，设置topic为particlecloud来显示 amcl 预估的当前机器人的位姿，箭头越是密集，说明当前机器人处于此位置的概率越高



### 4. 路径规划

- 在ROS的导航功能包集navigation中提供了 [move_base](http://wiki.ros.org/move_base) 功能包，用于实现此功能。
  - move_base 功能包提供了基于动作(action)的路径规划实现，move_base 可以根据给定的目标点，控制机器人底盘运动至目标位置，并且在运动过程中会连续反馈机器人自身的姿态与目标点的状态信息
  - move_base主要由全局路径规划与本地路径规划组成。

#### 4.1 move_base节点说明

##### 4.1.1动作

**动作订阅**

move_base/goal(move_base_msgs/MoveBaseActionGoal)

- move_base 的运动规划目标。

move_base/cancel(actionlib_msgs/GoalID)

- 取消目标。

**动作发布**

move_base/feedback(move_base_msgs/MoveBaseActionFeedback)

- 连续反馈的信息，包含机器人底盘坐标。

move_base/status(actionlib_msgs/GoalStatusArray)

- 发送到move_base的目标状态信息。

move_base/result(move_base_msgs/MoveBaseActionResult)

- 操作结果(此处为空)。

##### 4.1.2订阅的Topic

move_base_simple/goal(geometry_msgs/PoseStamped)

- 运动规划目标(与action相比，没有连续反馈，无法追踪机器人执行状态)。

##### 4.1.3发布的Topic

cmd_vel(geometry_msgs/Twist)

- 输出到机器人底盘的运动控制消息。

##### 4.1.4服务

~make_plan(nav_msgs/GetPlan)

- 请求该服务，可以获取给定目标的规划路径，但是并不执行该路径规划。

~clear_unknown_space(std_srvs/Empty)

- 允许用户直接清除机器人周围的未知空间。

~clear_costmaps(std_srvs/Empty)

- 允许清除代价地图中的障碍物，可能会导致机器人与障碍物碰撞，请慎用。

##### 4.1.5参数

参考[官网](http://wiki.ros.org/move_base#Parameters)。

#### 4.2 move_base与代价地图

##### 4.2.1 概念

机器人导航(尤其是路径规划模块)是依赖于地图的，地图在SLAM时已经有所介绍了，ROS中的地图其实就是一张图片，这张图片有宽度、高度、分辨率等元数据，在图片中使用灰度值来表示障碍物存在的概率。不过SLAM构建的地图在导航中是不可以直接使用的，因为：

1. SLAM构建的地图是静态地图，而导航过程中，障碍物信息是可变的，可能障碍物被移走了，也可能添加了新的障碍物，导航中需要时时的获取障碍物信息；
2. 在靠近障碍物边缘时，虽然此处是空闲区域，但是机器人在进入该区域后可能由于其他一些因素，比如：惯性、或者不规则形体的机器人转弯时可能会与障碍物产生碰撞，安全起见，最好在地图的障碍物边缘设置警戒区，尽量禁止机器人进入...

所以，静态地图无法直接应用于导航，其基础之上需要添加一些辅助信息的地图，比如时时获取的障碍物数据，基于静态地图添加的膨胀区等数据。

##### 4.2.2 代价地图组成

代价地图有两张:global_costmap(全局代价地图) 和 local_costmap(本地代价地图)，前者用于全局路径规划，后者用于本地路径规划。

两张代价地图都可以多层叠加,一般有以下层级:

- Static Map Layer：静态地图层，SLAM构建的静态地图。
- Obstacle Map Layer：障碍地图层，传感器感知的障碍物信息。
- Inflation Layer：膨胀层，在以上两层地图上进行膨胀（向外扩张），以避免机器人的外壳会撞上障碍物。
- Other Layers：自定义costmap。

多个layer可以按需自由搭配。

##### 4.2.3 碰撞算法

![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/ROS/%E7%A2%B0%E6%92%9E%E7%AE%97%E6%B3%95.jpg?raw=true)

- 上图中，横轴是距离机器人中心的距离，纵轴是代价地图中栅格的灰度值。

  - 致命障碍:栅格值为254，此时障碍物与机器人中心重叠，必然发生碰撞；
  - 内切障碍:栅格值为253，此时障碍物处于机器人的内切圆内，必然发生碰撞；
  - 外切障碍:栅格值为[128,252]，此时障碍物处于其机器人的外切圆内，处于碰撞临界，不一定发生碰撞；
  - 非自由空间:栅格值为(0,127]，此时机器人处于障碍物附近，属于危险警戒区，进入此区域，将来可能会发生碰撞；
  - 自由区域:栅格值为0，此处机器人可以自由通过；
  - 未知区域:栅格值为255，还没探明是否有障碍物。

  膨胀空间的设置可以参考非自由空间。

#### 4.3 move_base的使用

- 具体实现如下：
  1. 先编写launch文件模板
  2. 编写配置文件
  3. 集成导航相关的launch文件
  4. 测试

##### 4.3.1 机器配置的launch file

path.launch

```xml
<launch>
	<!-- respawn 为 false，意味着该节点关闭后，不会被重启 -->
    <!-- clear_params 为 true，意味着每次启动该节点都要清空私有参数然后重新载入-->
    <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen" clear_params="true">
        <!--通过 rosparam 会载入若干 yaml 文件用于配置参数-->
        <rosparam file="$(find 功能包)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find 功能包)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find 功能包)/param/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find 功能包)/param/global_costmap_params.yaml" command="load" />
        <rosparam file="$(find 功能包)/param/base_local_planner_params.yaml" command="load" />
    </node>

</launch>

```

##### 4.3.2 配置文件

- 不同类型机器人可能大小尺寸不同，传感器不同，速度不同，应用场景不同....最后可能会导致不同的路径规划结果，那么在调用路径规划节点之前，我们还需要配置机器人参数。
- 关于配置文件的编写，可以参考一些成熟的机器人的路径规划实现，比如: [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_navigation/param)
- 在功能包下新建 param 目录，并将上面参考的文件下载到此目录
  - costmap_common_params_burger.yaml
    - 重命名为:costmap_common_params.yaml。
  - local_costmap_params.yaml
  - global_costmap_params.yaml
  - base_local_planner_params.yaml

###### 1. costmap_common_params.yaml

该文件是move_base 在全局路径规划与本地路径规划时调用的通用参数，包括:机器人的尺寸、距离障碍物的安全距离、传感器信息等。配置参考如下:

```yaml
#机器人几何参，如果机器人是圆形，设置 robot_radius,如果是其他形状设置 footprint
robot_radius: 0.12 #圆形
# footprint: [[-0.12, -0.12], [-0.12, 0.12], [0.12, 0.12], [0.12, -0.12]] #其他形状

obstacle_range: 3.0 # 用于障碍物探测，比如: 值为 3.0，意味着检测到距离小于 3 米的障碍物时，就会引入代价地图
raytrace_range: 3.5 # 用于清除障碍物，比如：值为 3.5，意味着清除代价地图中 3.5 米以外的障碍物

allow_unknown: true #允许定位到未知地图区域

#膨胀半径，扩展在碰撞区域以外的代价区域，使得机器人规划路径避开障碍物
inflation_radius: 0.2
#代价比例系数，越大则代价值越小
cost_scaling_factor: 3.0

#地图类型
map_type: costmap
#导航包所需要的传感器
observation_sources: scan
#对传感器的坐标系和数据进行配置。这个也会用于代价地图添加和清除障碍物。例如，你可以用激光雷达传感器用于在代价地图添加障碍物，再添加kinect用于导航和清除障碍物。
scan: {sensor_frame: laser, data_type: LaserScan, topic: scan, marking: true, clearing: true}

```

###### 2. global_costmap_params.yaml

该文件用于全局代价地图参数设置:

```yaml
global_costmap:
  global_frame: map #地图坐标系
  robot_base_frame: base_footprint #机器人坐标系
  # 以此实现坐标变换

  update_frequency: 1.0 #代价地图更新频率
  publish_frequency: 1.0 #代价地图的发布频率
  transform_tolerance: 0.5 #等待坐标变换发布信息的超时时间

  static_map: true # 是否使用一个地图或者地图服务器来初始化全局代价地图，如果不使用静态地图，这个参数为false.
```

###### 3. local_costmap_params.yaml

该文件用于局部代价地图参数设置:

```yaml
local_costmap:
  global_frame: odom #里程计坐标系
  robot_base_frame: base_footprint #机器人坐标系

  update_frequency: 10.0 #代价地图更新频率
  publish_frequency: 10.0 #代价地图的发布频率
  transform_tolerance: 0.5 #等待坐标变换发布信息的超时时间

  static_map: false  #不需要静态地图，可以提升导航效果
  rolling_window: true #是否使用动态窗口，默认为false，在静态的全局地图中，地图不会变化
  width: 3 # 局部地图宽度 单位是 m
  height: 3 # 局部地图高度 单位是 m
  resolution: 0.05 # 局部地图分辨率 单位是 m，一般与静态地图分辨率保持一致
```

###### 4.base_local_planner_params.yaml

基本的局部规划器参数配置，这个配置文件设定了机器人的最大和最小速度限制值，也设定了加速度的阈值。

```yaml
TrajectoryPlannerROS:

# Robot Configuration Parameters
  max_vel_x: 0.5 # X 方向最大速度
  min_vel_x: 0.1 # X 方向最小速速

  max_vel_theta:  1.0 # 
  min_vel_theta: -1.0
  min_in_place_vel_theta: 1.0

  acc_lim_x: 1.0 # X 加速限制
  acc_lim_y: 0.0 # Y 加速限制
  acc_lim_theta: 0.6 # 角速度加速限制

# Goal Tolerance Parameters，目标公差
  xy_goal_tolerance: 0.10
  yaw_goal_tolerance: 0.05

# Differential-drive robot configuration
# 是否是全向移动机器人
  holonomic_robot: false

# Forward Simulation Parameters，前进模拟参数
  sim_time: 0.8
  vx_samples: 18
  vtheta_samples: 20
  sim_granularity: 0.05
```

###### 5.参数配置技巧

以上配置在实操中，可能会出现机器人在本地路径规划时与全局路径规划不符而进入膨胀区域出现假死的情况，如何尽量避免这种情形呢？

> 全局路径规划与本地路径规划虽然设置的参数是一样的，但是二者路径规划和避障的职能不同，可以采用不同的参数设置策略:
>
> - 全局代价地图可以将膨胀半径和障碍物系数设置的偏大一些；
> - 本地代价地图可以将膨胀半径和障碍物系数设置的偏小一些。
>
> 这样，在全局路径规划时，规划的路径会尽量远离障碍物，而本地路径规划时，机器人即便偏离全局路径也会和障碍物之间保留更大的自由空间，从而避免了陷入“假死”的情形。

##### 4.3.3 整合的launch file

如果要实现导航，需要集成地图服务、amcl 、move_base 与 Rviz 等，集成示例如下:

integration.launch

```xml
<launch>
    <!-- 设置地图的配置文件 -->
    <arg name="map" default="nav.yaml" />
    <!-- 运行地图服务器，并且加载设置的地图-->
    <node name="map_server" pkg="map_server" type="map_server" args="$(find mycar_nav)/map/$(arg map)"/>
    <!-- 启动AMCL节点 -->
    <include file="$(find mycar_nav)/launch/amcl.launch" />

    <!-- 运行move_base节点 -->
    <include file="$(find mycar_nav)/launch/path.launch" />
    <!-- 运行rviz -->
    <node pkg="joint_state_publisher" name="joint_state_publisher" type="joint_state_publisher" />
    <node pkg="robot_state_publisher" name="robot_state_publisher" type="robot_state_publisher" />
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find mycar_nav)/rviz/nav.rviz" />

</launch>
```

##### 4.3.4 运行

1. 启动gazebo

2. 启动导航相关的integration.launch文件

3. 添加rviz组件，可以将配置数据保存，后期直接调用；

4. 通过Rviz工具栏的 2D Nav Goal设置目的地实现导航。

5. 也可以在导航过程中，添加新的障碍物，机器人也可以自动躲避障碍物。

   - 全局代价地图与本地代价地图组件配置如下:

     ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/ROS/rviz%E4%BB%A3%E4%BB%B7%E5%9C%B0%E5%9B%BE%E8%AE%BE%E7%BD%AE.png?raw=true)

   - 全局路径规划与本地路径规划组件配置如下:

     ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/ROS/rviz%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92%E8%AE%BE%E7%BD%AE.png?raw=true)

### 5. 导航与slam建图

- 实现机器人自主移动的SLAM建图。
- SLAM建图过程中本身就会时时发布地图信息，所以无需再使用map_server，SLAM已经发布了话题为 /map 的地图消息了，且导航需要定位模块，SLAM本身也是可以实现定位的。

- 步骤如下:
  1. 编写launch文件，集成SLAM与move_base相关节点；
  2. 执行launch文件并测试。

#### 5.1 launch file

slam_move.launch

```xml
<launch>
    <!-- 启动SLAM节点 -->
    <!-- 七,2的1.2中的launchfile -->
    <include file="$(find mycar_nav)/launch/slam.launch" />
    <!-- 运行move_base节点 -->
    <!-- 七,4的4.3.1的launchfile-->
    <include file="$(find mycar_nav)/launch/path.launch" />
    <!-- 运行rviz -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find mycar_nav)/rviz/nav.rviz" />
</launch>
```

#### 5.2运行

1. 运行gazebo
2. 运行上面slam_move.launch
3. 在rviz中通过2D Nav Goal设置目标点，机器人开始自主移动并建图了；
4. 最后可以使用 map_server 保存地图。

## 3. 导航相关的msg

### 3.1 导航之地图

- 地图相关的消息主要有两个:

  1. nav_msgs/MapMetaData
     - 地图元数据，包括地图的宽度、高度、分辨率等。

  2. nav_msgs/OccupancyGrid
     - 地图栅格数据，一般会在rviz中以图形化的方式显示。

- 调用`rosmsg info nav_msgs/MapMetaData`显示消息内容如下:

  ```
  time map_load_time
  float32 resolution #地图分辨率
  uint32 width #地图宽度
  uint32 height #地图高度
  geometry_msgs/Pose origin #地图位姿数据
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  ```

- 调用 `rosmsg info nav_msgs/OccupancyGrid`显示消息内容如下:

  ```
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  #--- 地图元数据
  nav_msgs/MapMetaData info
    time map_load_time
    float32 resolution
    uint32 width
    uint32 height
    geometry_msgs/Pose origin
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
  #--- 地图内容数据，数组长度 = width * height
  int8[] data
  ```

  

### 3.2 导航之里程计

里程计相关消息是:nav_msgs/Odometry，调用`rosmsg info nav_msgs/Odometry` 显示消息内容如下:

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose #里程计位姿
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist #速度
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z    
  # 协方差矩阵
  float64[36] covariance
```

### 3.3 导航之坐标变换

坐标变换相关消息是: tf/tfMessage，调用`rosmsg info tf/tfMessage` 显示消息内容如下:

```
geometry_msgs/TransformStamped[] transforms #包含了多个坐标系相对关系数据的数组
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string child_frame_id
  geometry_msgs/Transform transform
    geometry_msgs/Vector3 translation
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion rotation
      float64 x
      float64 y
      float64 z
      float64 w
```

### 3.4 导航之定位

定位相关消息是:geometry_msgs/PoseArray，调用`rosmsg info geometry_msgs/PoseArray`显示消息内容如下:

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose[] poses #预估的点位姿组成的数组
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
```

### 3.5 导航之目标点与路径规划

- 目标点相关消息是:move_base_msgs/MoveBaseActionGoal，调用`rosmsg info move_base_msgs/MoveBaseActionGoal`显示消息内容如下:

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
actionlib_msgs/GoalID goal_id
  time stamp
  string id
move_base_msgs/MoveBaseGoal goal
  geometry_msgs/PoseStamped target_pose
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose #目标点位姿
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
```

- 路径规划相关消息是:nav_msgs/Path，调用`rosmsg info nav_msgs/Path`显示消息内容如下:

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/PoseStamped[] poses #由一系列点组成的数组
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
```

### 3.6 导航之激光雷达

激光雷达相关消息是:sensor_msgs/LaserScan，调用`rosmsg info sensor_msgs/LaserScan`显示消息内容如下:

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min #起始扫描角度(rad)
float32 angle_max #终止扫描角度(rad)
float32 angle_increment #测量值之间的角距离(rad)
float32 time_increment #测量间隔时间(s)
float32 scan_time #扫描间隔时间(s)
float32 range_min #最小有效距离值(m)
float32 range_max #最大有效距离值(m)
float32[] ranges #一个周期的扫描数据
float32[] intensities #扫描强度数据，如果设备不支持强度数据，该数组为空
```

### 3.7 导航之相机

- 深度相机相关消息有

  - sensor_msgs/Image 对应的一般的图像数据，
  - sensor_msgs/CompressedImage 对应压缩后的图像数据，
  - sensor_msgs/PointCloud2 对应的是点云数据(带有深度信息的图像数据)。

- 调用`rosmsg info sensor_msgs/Image`显示消息内容如下:

  ```
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  uint32 height #高度
  uint32 width  #宽度
  string encoding #编码格式:RGB、YUV等
  uint8 is_bigendian #图像大小端存储模式
  uint32 step #一行图像数据的字节数，作为步进参数
  uint8[] data #图像数据，长度等于 step * height
  ```

- 调用`rosmsg info sensor_msgs/CompressedImage`显示消息内容如下:

  ```
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string format #压缩编码格式(jpeg、png、bmp)
  uint8[] data #压缩后的数据
  ```

- 调用`rosmsg info sensor_msgs/PointCloud2`显示消息内容如下:

  ```
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  uint32 height #高度
  uint32 width  #宽度
  sensor_msgs/PointField[] fields #每个点的数据类型
    uint8 INT8=1
    uint8 UINT8=2
    uint8 INT16=3
    uint8 UINT16=4
    uint8 INT32=5
    uint8 UINT32=6
    uint8 FLOAT32=7
    uint8 FLOAT64=8
    string name
    uint32 offset
    uint8 datatype
    uint32 count
  bool is_bigendian #图像大小端存储模式
  uint32 point_step #单点的数据字节步长
  uint32 row_step   #一行数据的字节步长
  uint8[] data      #存储点云的数组，总长度为 row_step * height
  bool is_dense     #是否有无效点
  ```

  

## 4. 深度图像转激光数据

ROS中的一个功能包:[depthimage_to_laserscan](http://wiki.ros.org/depthimage_to_laserscan)，顾名思义，该功能包可以将深度图像信息转换成激光雷达信息，应用场景如下:

> 在诸多SLAM算法中，一般都需要订阅激光雷达数据用于构建地图，因为激光雷达可以感知周围环境的深度信息，而深度相机也具备感知深度信息的功能，且最初激光雷达价格比价比较昂贵，那么在传感器选型上可以选用深度相机代替激光雷达吗？

答案是可以的，不过二者发布的消息类型是完全不同的，如果想要实现传感器的置换，那么就需要将深度相机发布的三维的图形信息转换成二维的激光雷达信息，这一功能就是通过depthimage_to_laserscan来实现的。



### 4.1 depthimage_to_laserscan简介

#### 4.1.1 原理

depthimage_to_laserscan将实现深度图像与雷达数据转换的原理比较简单，雷达数据是二维的、平面的，深度图像是三维的，是若干二维(水平)数据的纵向叠加，如果将三维的数据转换成二维数据，只需要取深度图的某一层即可

#### 4.1.2 优缺点

**优点:**深度相机的成本一般低于激光雷达，可以降低硬件成本；

**缺点:** 深度相机较之于激光雷达无论是检测范围还是精度都有不小的差距，SLAM效果可能不如激光雷达理想。

#### 4.1.3安装

```
sudo apt-get install ros-noetic-depthimage-to-laserscan
```



### 4.2 depthimage_to_laserscan节点说明

#### 4.2.1订阅的Topic

image(sensor_msgs/Image)

- 输入图像信息。

camera_info(sensor_msgs/CameraInfo)

- 关联图像的相机信息。通常不需要重新映射，因为camera_info将从与image相同的命名空间中进行订阅。

#### 4.2.2发布的Topic

scan(sensor_msgs/LaserScan)

- 发布转换成的激光雷达类型数据。

#### 4.2.3参数

该节点参数较少，只有如下几个，一般需要设置的是: output_frame_id。

~scan_height(int, default: 1 pixel)

- 设置用于生成激光雷达信息的象素行数。

~scan_time(double, default: 1/30.0Hz (0.033s))

- 两次扫描的时间间隔。

~range_min(double, default: 0.45m)

- 返回的最小范围。结合range_max使用，只会获取 range_min 与 range_max 之间的数据。

~range_max(double, default: 10.0m)

- 返回的最大范围。结合range_min使用，只会获取 range_min 与 range_max 之间的数据。

~output_frame_id(str, default: camera_depth_frame)

- 激光信息的ID。

### 4.3 depthimage_to_laserscan使用

#### 4.3.1 launch file

d2l.launch

```xml
<launch>
    <node pkg="depthimage_to_laserscan" type="depthimage_to_laserscan" name="depthimage_to_laserscan">
        <!-- 订阅的话题需要根据深度相机发布的话题设置 -->
        <!-- 参见sensors_kinect.urdf.xacro的topic相关标签 -->
        <remap from="image" to="/camera/depth/image_raw" />
        <!-- output_frame_id需要与深度相机的坐标系一致。 -->
        <!-- 参见sensors_kinect.urdf.xacro的frame相关标签 -->
        <param name="output_frame_id" value="camera"  />
    </node>
</launch>

```

#### 4.3.2 修改xacro

经过信息转换之后，深度相机也将发布雷达数据即发送topic: scan(sensor_msgs/LaserScan)，为了不产生混淆，可以注释掉 xacro 文件中的关于激光雷达的部分内容。

#### 4.3.3 执行

1. 启动gazebo仿真环境
2. 启动rviz并添加相关组件(image、LaserScan)

### 4.4 配合slam使用

1. 启动gazebo仿真环境

2. 启动4.3.1的d2l.launch

3. 启动1.2.1的slam.launch绘图

4. 启动键盘键盘控制节点，用于控制机器人运动建图

   ```
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py
   ```

5. 在 rviz 中添加组件，显示栅格地图最后，就可以通过键盘控制gazebo中的机器人运动，同时，在rviz中可以显示gmapping发布的栅格地图数据了
