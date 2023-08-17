# 一、基础

## 0. ORB-Slam2框架

- 出自论文

![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/slam/orb-slam%E6%A1%86%E6%9E%B6.png?raw=true)

- 中文翻译

![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/slam/orb-slam%E6%A1%86%E6%9E%B6%E4%B8%AD%E6%96%87.png?raw=true)

## 1. 安装orb-slam2

1. 环境

   ```
   openCV   4.7.0
   Pangolin 0.8
   ros      noteic
   ubuntu   20
   ```

   - 检查opencv是否安装成功

     ```
     cd opencv-4.7.0/samples/cpp/example_cmake
     mkdir build && cd build
     cmake ..
     make
     ./opencv_example
     ```

   - 检查Pangolin是否安装成功

     ```
     cd Pangolin/examples/HelloPangolin
     mkdir build && cd build
     cmake ..
     make
     ./HelloPangolin
     ```
     
   - VSCode环境

     c_cpp_properties.json

     ```json
     {
         "configurations": [
             {
                 "name": "Linux",
                 "includePath": [
                     "${workspaceFolder}/**",
                     "/usr/include/opencv4/**",
                     "/usr/include/eigen3"
     
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

2. 下载orb-slam2后修改

   1. 修改CMakeLists，和Thirdparty/DBow2的CMakeLists

      ```cmake
       # 原本为：find_package(OpenCV 3.0 QUIET)
       find_package(OpenCV 4.7 QUIET)
       if(NOT OpenCV_FOUND)
          find_package(OpenCV 2.4.3 QUIET)
          if(NOT OpenCV_FOUND)
             message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
          endif()
       endif()
      ```

   2. ORB_SLAM2/include/system.h:添加`#include<unistd.h>`

   3. ORB_SLAM2/include/LoopClosing.h:

      ```c++
      typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
              Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
      //改为：
      typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
              Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3> > > KeyFrameAndPose;
      ```

   4. 

3. 编译

   ```
   mkdir -p orbslam_ws/src/ORB_SLAM2
   cd orbslam_ws/src/ORB_SLAM2
   chmod +x build.sh
   ./build.sh
   ```

## 2. 试运行

1. 下载[数据集](http://vision.in.tum.de/data/datasets/rgbd-dataset/download )

2. 在ORB_SLAM2文件夹下：

   - Monocular Examples:

     `./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUMX.yaml PATH_TO_SEQUENCE_FOLDER`

     - TUMX.yaml的X改为下载数据集序列，比如第一个数据集就改为TUM1.yaml
     - PATH_TO_SEQUENCE_FOLDER改为数据集的安装目录（在安装目录下ctrl+l）

   - RGB-D Examples:

     1. 下载代码[associate.py](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools)来将color images数据关联到depth images

        `python2 associate.py rgb.txt depth.txt > associations.txt`

        - 注意要用python2来运行这个脚本
        - 将脚本放在这些数据集的文件夹中

     2. `./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE`

        - TUMX.yaml的X改为下载数据集序列，比如第一个数据集就改为TUM1.yaml
        - PATH_TO_SEQUENCE_FOLDER改为数据集的安装目录（在安装目录下ctrl+l）
        - ASSOCIATIONS_FILE 是PATH_TO_SEQUENCE_FOLDER/associations.txt

- 运行结果
  - 图像上的绿点：跟踪成功的特征点帧
  - 绿框：当前关键帧
  - 蓝框：关键帧
    - 蓝框之间的绿线：关键帧之间的连接关系
  - 红点：局部地图点
  - 黑点：所有的地图点

## 3. 变量命名

- m(member)开头的变量表示类的成员变量

  ```
  int mSensor;
  int mTrackingState;
  std::mutex mMutexMode;
  ```

- mp开头的变量：指针pointer型类成员变量

  ```
  Tracking* mpTracker;
  LocalMapping* mpLocalMapper;
  LoopClosing* mpLoopCloser;
  Viewer* mpViewer;
  ```

- mb开头的变量：布尔bool型类成员变量

  ```
  bool mbOnlyTracking;
  ```

- mv开头的变量：向量vector型类成员变量

  ```
  std::vector<cv::Point3f> mvInip3D;
  ```

- mpt开头的变量：指针pointer型类成员变量，并且它是一个线程thread

  ```
  std::thread* mptLocalMapping;
  ```

- ml开头的变量：列表list型类成员变量

  ```
  list<double> mlFrameTimes;
  ```

- mlp开头的变量：列表list型类成员变量，并且它的元素类型是指针(pointer)

  ```
  list<KeyFrame*> mlpReferences;
  ```

- mlb开头的变量：列表listt型类成员变量，并且它的元素类型是布尔(bool)

  ```
  list<bool> mlbLost;
  ```

# 二、ORB特征提取

# 三、地图初始化

# 四、地图点、关键帧、图结构

# 五、特征匹配

# 六、跟踪线程

# 七、局部建图线程

# 八、闭环检测及矫正线程

# 九、BA优化方法

# 十、工程实践



