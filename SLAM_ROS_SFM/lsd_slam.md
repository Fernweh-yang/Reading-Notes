# 资源汇总

- [论文](https://jakobengel.github.io/pdf/engel14eccv.pdf)
- [git 代码](https://github.com/tum-vision/lsd_slam)

# 安装使用

- 环境

  ```
  ubuntu20
  ros:Noetic
  opencv:4.7
  qt:5.12.8
  ```

  

# 基本原理

- 特点：

  - LSD在cpu上实现了实时的半稠密场景的重建，对特征缺失区域也可以进行重建。

  - 但是LSD对相机内参和曝光很敏感，且在相机快速运动的时候容易丢失。

  - 没有基于直接法的回环检测，因此还是需要依赖特征点方法来进行回环检测。

## LSD-SLAM整体框架

![image-20231112202242729](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/LSD-SLAM%E6%A1%86%E6%9E%B6.png)

LSD-SLAM系统分为3个模块

1. Tracking

   通过光度误差来估算 **当前帧** 相对于 **参考关键帧** 的相对位姿

   > 为了更好更快的求解位姿，本文使用了方差归一化后的光度误差(也就是对光度误差除了一个方差，参见论文公式14)

2. Depth Map Estimation

   当前帧Tracking成功后，先判断是否将其视为新的关键帧。

   > **关键帧的创建标准**：相机移动足够远，就创建一个新的关键帧。距离定义参见论文16式。
   >
   > **关键帧 key frame**：相隔一定距离的具有代表性的帧，每一个关键帧对应一个深度图
   >
   > **参考帧 reference frame**：是用于和当前帧做立体匹配的帧，一个关键帧后面一般有许多参考帧
   >
   > **立体匹配**：对两个或多个不同视点或不同时间拍摄的图像进行分析和比对，以确定它们之间的对应关系和空间位置关系

   如果将其构建为新的关键帧，则构建新的深度图（Depth Map Creation）。

   如果不将其构建为新关键帧，则更新参考关键帧的深度（Depth Map Refinement）

3. Map Optimization

   主要完成3个任务

   1. 位姿优化

      区别于tracking中用$se(3)$来估计位姿，并且是每帧图像与它参考帧之间的位姿

      这里使用相似矩阵$sim(3)$来估计位姿，并且只估计关键帧之间的位姿

   2. 点云地图优化

   3. 回环检测

      回环完成后，再进行全局优化

   一旦关键帧变成参考帧，那么对应的深度图将不再进行优化

## 不确定度计算

一个函数$f(X)$由于它的输入$X$不确定而引起的不确定度可以由如下公式计算：
$$
\Sigma_f \approx \mathbf{J_f}\mathbf{\Sigma}_{\mathbf{X}}\mathbf{J}_f^T\tag{1}
$$

- $\mathbf{\Sigma_{X}}$：假设输入数据X是高斯分布的，它的协方差为$\mathbf{\Sigma_{X}}$
- $\mathbf{J_f}$：函数$f(X)$的雅可比矩阵
- $\mathbf{\Sigma_{f}}$：函数$f(X)$的协方差，也就是不确定度

> **协方差covariance**：度量各个维度偏离其均值的程度
>
> 协方差的值：
> - 如果为正值，则说明两者是正相关的(从协方差可以引出“相关系数”的定义)
> - 如果为负值就说明负相关的
> - 如果为0，也是就是统计上说的“相互独立”。

## 地图的表示：

- 地图由关键帧组成；
- 关键帧组成：当前时刻的相机图像、逆深度图像、逆深度的方差（方差就是权重）；



# 代码

## LSD-SLAM核心库文件

![image-20231113001505361](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/LSD-SLAM%E4%BB%A3%E7%A0%81%E7%B1%BB%E8%A1%A8.png)