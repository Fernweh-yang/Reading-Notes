# 零、资源汇总

- [课程](https://cvlife.net/)

- [ORB-SLAM2注释代码](https://github.com/electech6/ORB_SLAM2_detailed_comments)

- [ORB-SLAM3注释代码](https://github.com/electech6/ORB_SLAM3_detailed_comments)

- [VINS-Mono注释代码](https://github.com/xieqi1/VINS-Mono-noted)

  基于单目相机和 IMU 的 SLAM 算法

- [VINS-Fusion注释代码](https://github.com/xieqi1/VINS-Fusion-noted)

  基于多传感器（包括单目相机、双目相机和 IMU）的 SLAM 算法。

- [运动规划注释代码](https://github.com/felderstehost/ros-navigation-noetic)

  ros自带的navigation库

- [OpenMVS注释代码](https://github.com/electech6/openMVS_comments)

  OpenMVS 是一个基于 SfM 算法的多视角立体重建工具。SfM 算法是指从多个图像中恢复三维结构的算法。

- [lio-sam注释代码](https://github.com/xieqi1/lio-sam-noted)

  基于激光雷达和惯性测量单元（IMU）的 SLAM 算法。

- [a-loam注释代码](https://github.com/xieqi1/a-loam-noted)

  基于激光雷达的 SLAM 算法

- [LVI-SAM注释代码](https://github.com/electech6/LVI-SAM_detailed_comments)

  基于激光雷达、视觉和惯性测量单元（IMU）的 SLAM 算法。

- [R3LIVE注释代码](https://gitee.com/qcl5683/R3LIVE-CommentV)

  基于激光雷达、视觉和惯性测量单元（IMU）的 SLAM 算法。

- [cartographer注释代码](https://github.com/xiangli0608/cartographer_detailed_comments_ws)

  激光slam

# 一、基础

## 1. ORB-Slam2框架

- 出自论文

![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/slam/orb-slam%E6%A1%86%E6%9E%B6.png?raw=true)

- 中文翻译

![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/slam/orb-slam%E6%A1%86%E6%9E%B6%E4%B8%AD%E6%96%87.png?raw=true)

## 2. 安装orb-slam2

1. 环境

   ```
   openCV   4.7.0
   Pangolin 0.8
   ros      noteic
   ubuntu   20
   eigen    3.3.7
   ```

   - 检查eigen版本:

     `pkg-config --modversion eigen3`
     
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

2. 下载orb-slam2:

   因为用opencv4代替了opencv3,pangolin0.8代替了0.5所以有很多需要修改的地方: [参考](https://gaoyichao.com/Xiaotu/?book=ORB_SLAM%E6%BA%90%E7%A0%81%E8%A7%A3%E8%AF%BB&title=ORB_SLAM2%E7%9A%84%E6%80%BB%E4%BD%93%E6%A1%86%E6%9E%B6%E4%B8%8E%E5%AE%89%E8%A3%85%E8%AF%95%E7%94%A8)

   具体修改见commit：`e3ef925` opencv4,pangolin0.8下编译成功

   下载修改完的代码见[git](https://github.com/Fernweh-yang/SLAM_Code_Learning/tree/main/ORB_SLAM2)

   ```
   git clone git@github.com:Fernweh-yang/SLAM_Code_Learning.git
   ```

3. 编译

   ```shell
   chmod +x build.sh
   ./build.sh	# 里面写了mkdir build等操作，所以放心直接用
   ```

## 3. 试运行

1. 下载[数据集](http://vision.in.tum.de/data/datasets/rgbd-dataset/download )

2. 在ORB_SLAM2文件夹下：

   - Monocular Examples:

     ```shell
     ./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt \
     							  Examples/Monocular/TUMX.yaml \
     							  PATH_TO_SEQUENCE_FOLDER
     ```
   
     - ORBvoc.txt是官方提供的一个用于场景识别和重定位的特征字典
   
     - TUMX.yaml的X改为下载数据集序列
   
       TUM1.yaml,TUM2.yaml or TUM3.yaml 分别对应于数据集freiburg1, freiburg2 and freiburg3
     - PATH_TO_SEQUENCE_FOLDER改为数据集的安装目录（在安装目录下ctrl+l）
   
     比如:
   
     ```shell
     ./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt \
     							  Examples/Monocular/TUM2.yaml \
                                   /home/yang/Datasets/TUM/rgbd_dataset_freiburg2_xyz 
     ```
   
     
   
   - RGB-D Examples:
   
     1. 下载代码[associate.py](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools)来将color images数据关联到depth images
   
        在数据集文件夹下运行：
   
        `python2 associate.py rgb.txt depth.txt > associations.txt`
     
        - 注意要用python2来运行这个脚本
        - 将脚本放在这些数据集的文件夹中
   
     2. 运行：
     
        ```
        ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt \
                                  Examples/RGB-D/TUMX.yaml \
                                  PATH_TO_SEQUENCE_FOLDER \
                                  ASSOCIATIONS_FILE
        ```
     
        - TUMX.yaml的X改为下载数据集序列
     
          TUM1.yaml,TUM2.yaml or TUM3.yaml 分别对应于数据集freiburg1, freiburg2 and freiburg3
        - PATH_TO_SEQUENCE_FOLDER改为数据集的安装目录（在安装目录下ctrl+l）
        - ASSOCIATIONS_FILE 是PATH_TO_SEQUENCE_FOLDER/associations.txt
     
        比如：
     
        ```
        ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt \
        						  Examples/RGB-D/TUM1.yaml \
        						  /home/yang/Datasets/TUM/rgbd_dataset_freiburg1_xyz \
                                  /home/yang/Datasets/TUM/rgbd_dataset_freiburg1_xyz/associations.txt 
        ```
     
        

- 运行结果
  - 图像上的绿点：跟踪成功的特征点帧
  - 绿框：当前关键帧
  - 蓝框：关键帧
    - 蓝框之间的绿线：关键帧之间的连接关系
  - 红点：局部地图点
  - 黑点：所有的地图点

## 4.. 变量命名

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

https://zhuanlan.zhihu.com/p/636742092

todo: 设置**[OpenImageDebugger](https://github.com/OpenImageDebugger/OpenImageDebugger)**

# 三、地图初始化

# 四、地图点、关键帧、图结构

# 五、特征匹配

# 六、跟踪线程

# 七、局部建图线程

# 八、闭环检测及矫正线程

# 九、BA优化方法

# 十、工程实践



