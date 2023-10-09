# 资源汇总

- [官网教材](http://docs.ros.org/en/humble/Tutorials.html)



# 1. ROS2基础

CLI（Command line interface）命令行接口

## 1.1 一些重要的环境变量

1. **添加源**

   ```shell
   # 在新开的shell中
   source /opt/ros/humble/setup.bash
   
   # 或者更方便的，直接修改.bashrc
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

2. **如何查看现有的环境变量**

   ```shell
   printenv | grep -i ROS
   
   # 得到如下:
   ROS_VERSION=2
   ROS_PYTHON_VERSION=3
   ROS_DISTRO=humble
   ```

3. **ROS_DOMAIN_ID变量**

   - **什么是domain ID？**

     [domain ID](http://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html) 是ROS 2新的概念，用于多个ROS2域之间的**通信隔离**。

     - ROS2通信的默认中间件是数据分发服务DDS(Data distribution service)

       > DDS是一种用于实时数据通信和消息传递的标准通信协议和中间件。它广泛用于分布式应用程序，特别是在嵌入式系统、实时系统和大规模网络中，以支持数据交换和通信。

     - 而domain id使得不同的逻辑网络(logical networks)共享一个物理网络

       > 在DDS中，域id"Domain ID"是一个整数值，用于标识一个DDS域。DDS域是一个逻辑上隔离的通信单元。
       >
       > 同一DDS域中的DDS实例可以相互通信，而不同DDS域中的实例则不会直接通信。

     - 默认情况下，所有的ROS2 nodes都使用domain id=0

   - **什么时候需要多个domain id?**

     - 多个独立的ROS 2系统：如果有多个独立的ROS 2系统运行在同一物理网络中，并且希望它们彼此隔离，以防止它们之间的通信冲突。
     - 通信隔离：有时，您可能希望将一组ROS 2节点隔离在一个DDS域中，以限制它们之间的直接通信。这可以用于确保某些节点只能与特定的其他节点通信，而不会影响整个ROS 2系统。

   - **如何直接设置这个环境变量：**

     将当前terminal下所有新打开的node，都打开在这个domain id下

     ```shell
     # 在shell中
     export ROS_DOMAIN_ID=<your_domain_id>
     # 一劳永逸的：
     echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
     ```

     也可以对单独某一个节点进行设置：

     ```shell
     # 在一个terminal中：
     ros2 run demo_nodes_cpp talker
     
     # 在另一个terminal中：
     ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener
     ```

4. **ROS_LOCALHOST_ONLY变量**

   - 通常ROS2通信不会局限于localhost

     > "localhost" 网络主机名，通常指代本地计算机或设备。

   - 使用ROS_LOCALHOST_ONLY可以让本机的ROS2系统、topics、services和actions不会被其他连着同一个local network的计算机发现。这在教室或实验室很有用，不同机器人连着同一个网但不会被互相控制。

   - 如何设置这个变量：

     ```shell
     # 在shell中
     export ROS_LOCALHOST_ONLY=1
     
     # 一劳永逸的：
     echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
     
     # 同样可以对单独某一个节点执行：
     ROS_LOCALHOST_ONLY=1 ros2 run demo_nodes_cpp talker
     ```

     

## 1.2 用turtlesim学习ros2和rqt

