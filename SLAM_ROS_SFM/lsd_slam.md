# 资源汇总

- [论文](https://jakobengel.github.io/pdf/engel14eccv.pdf)
- [git 代码](https://github.com/tum-vision/lsd_slam)
- [参考博客](https://blog.csdn.net/lancelot_vim?type=blog)

# 安装使用

- 环境

  ```
  ubuntu20
  ros:Noetic
  opencv:4.8.1
  qt:5.12.8
  ```

- 安装

  ```shell
  # 在lsd-ws/src下clone修改过的lsd-slam版本
  https://github.com/Fernweh-yang/SLAM_Code_Learning/tree/main/lsd_ws/src
  catkin_make
  ```

- 运行

  ```
  rosrun lsd_slam_core dataset _files:=/home/yang/Downloads/LSD_room_images/LSD_room/images _hz:=1 _calib:=/home/yang/Downloads/LSD_room_images/LSD_room/cameraCalibration.cfg
  ```

  - 如果遇到gtk2.0和3.0冲突的问题，可能是多个opencv版本带来的。将ros和opencv所有版本都删除重装后就可以了

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

   不管构不构建新的关键帧，都需要进行深度估计，深度估计有3步：

   1. 用立体视觉方法来从先前的帧得到新的深度估计
   2. 深度图帧与帧之间的传播
   3. 部分规范化已知深度

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

## 关键帧表示

- 地图由关键帧组成；
- 关键帧组成$K_i=(I_i,D_i,V_i)$：分别是当前时刻的相机图像、逆深度图像、逆深度的方差（方差就是权重）；

然后根据光度误差计算出当前帧到关键帧之间的位姿变换

## 深度地图估计

1. 判断是否是关键帧

   根据相机移动来判断的，如果相机移动了足够远，那么就重新创建一个关键帧，否则就refine当前的关键帧

2. 深度估计

   不管是创建新的关键帧还是优化当前关键帧，都需要对新的一帧进行深度估计

3. 深度估计：选择参考帧

   > We use the oldest frame the pixel was observed in, where the disparity search range and the observation angle do not exceed a certain threshold (see Fig. 4). If a disparity search is unsuccessful (i.e., no good match is found), the pixel’s “age” is increased, such that subsequent disparity searches use newer frames where the pixel is likely to be still visible.

   - 通过视察disparity来搜索最先看到同一像素的帧，一直到当前帧的前一帧作为参考帧。
   - 如果搜索失败，说明没有帧和当前帧有看到同一像素的。那么就加大这些像素的年龄，然后在新的帧里面去继续搜索。

4. 深度估计：对每一帧的每个像素进行匹配

   对于每个像素，沿着极线方向搜索匹配，如果深度估计是可用的，那么搜索范围限制到均值加减2倍方差的区间，如果不可用，那么所有的差异性都需要被搜索，并使用，最后使用SSD误差作匹配算法

   >  误差平方和算法（Sum of Squared Differences，简称SSD算法）是一种基于灰度的图像匹配算法，其他类似的算法还有平均绝对差算法（MAD）、绝对误差和算法（SAD）、平均误差平方和算法（MSD）、归一化积相关算法（NCC）、序贯相似性算法（SSDA）。

5. 深度估计：计算深度

   - 参考帧上极线的计算：需要用到几何差异误差
   - 极线上得到最好的匹配位置：需要得到图像差异误差
   - 通过匹配位置计算出最佳的深度：根据基线量化误差

6. 如果创建关键帧：

   > Once a new frame is chosen to become a keyframe, its depth map is initialized by projecting points from the previous keyframe into it, followed by one iteration of spatial regularization and outlier removal as proposed in 9. Afterwards, the depth map is scaled to have a mean inverse depth of one - this scaling factor is directly incorporated into the sim(3) camera pose. Finally, it replaces the previous keyframe and is used for tracking subsequent new frames

   新的关键帧需要之前的关键帧将点投影过来(投影方案已经在深度前传介绍过)，得到这一帧的有效点，深度通过sim(3)变换投影均值和缩放因子，最后用这个关键帧替换掉之前的关键帧

7. 如果不创建关键帧

   > A high number of very efficient small- baseline stereo comparisons is performed for image regions where the expected stereo accuracy is sufficiently large, as described in 9. The result is incorporated into the existing depth map, thereby refining it and potentially adding new pixels – this is done using the filtering approach proposed in 9.

   如果不创建关键帧，那么就用当前的观测对之前的深度进行修正

## 图优化

使用图优化优化每一帧相对于世界坐标系的位姿

# 代码

## LSD-SLAM核心库文件

![image-20231113001505361](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/LSD-SLAM%E4%BB%A3%E7%A0%81%E7%B1%BB%E8%A1%A8.png)

## LSD-SLAM的内存管理

代码文件都在`lsd_slam_core/src/DataStructures`目录下

### 1. FrameMemory.h

用于管理内存

- 构造函数`FrameMemory()`

  通过将构造函数FrameMemory()私有化(放在private:中)保证只能创建一个实例

- `getbuffer()`和`getFloatBuffer()`

  通过调用`allocateBuffer()`申请新的内存空间

### 2. Frame.h

lsd-slam中的地图是由一系列关键帧keyframe的姿态图表示的，每个关键帧由图像地图camera image + 逆深度地图inverse depth map + 逆深度地图variance of the inverse depth组成。

这个文件就是用来构建每一帧的image, gradients, depth和depth variance pyramid

- 结构体`struct Data{}`

  定义了一系列金字塔：图像、梯度、深度、深度方差金字塔

  根据setting.h定义的宏可以看到金字塔层数PYRAMID_LEVELS为5层。

- 构造函数`Frame()`

  会调用`initialize()`来初始化图像id，内存，位姿，相机内参，相机内参的逆，各种金字塔。

  申请内存依靠的是FrameMemory.h中的`getFloatBuffer()`

- 析构函数`~Frame()`

  回收内存，依靠的是FrameMemory.h中的`returnBuffer()`

- `require()`

  这个函数先是在判断需要怎么样的数据,如果这个数据的该层金字塔还没创建,就调用相应的构建函数去构建。

  构建函数分别有：

  - `buildImage(level)`构建图像金字塔
  - `buildGradients(level)`构建梯度金字塔
  - `buildMaxGradients(level)`构建最大梯度金字塔

  - `buildIDepthAndIDepthVar(level)`构建深度和深度方差（不确定度）金字塔

- `minimizeInMemory()`

  通过调用各个release()和clear_refPixelWasGood()函数来释放上面这些函数申请的内存。

  release()中针对不同金字塔有函数：

  - `releaseImage(level)`
  - `releaseGradients(level)`
  - `releaseMaxGradients(level)`
  - `releaseIDepth(level)`
  - `releaseIDepthVar(level)`

- `setDepth()`给当前帧的金字塔第一层设置新的深度估计值
- `setDepthFromGroundTruth()`把真实的深度设置给当前帧的金字塔

- `prepareForStereoWith()`一些相似矩阵的预运算

### 3. FramePoseStruct.h

一些用于图优化相关的变量

## LSD-SLAM的跟踪

### 0. 整体结构

```
图像金字塔迭代level-4到level-1
Step1: 对参考帧当前层构造点云(reference->makePointCloud)
Step2: 计算变换到当前帧的残差和梯度(calcResidualAndBuffers)
Step3: 计算法方差归一化残差(calcWeightsAndResidual)
Step4: 计算雅克比向量以及A和b(calculateWarpUpdate)
Step5: 计算得到收敛的delta，并且更新SE3(inc = A.ldlt().solve(b))
重复Step2-Step5直到收敛或者达到最大迭代次数
计算下一层金字塔
```

### 1. TrackingReference.h

这个类主要用于管理在tracking时用到的参考帧

- `releaseAll()`

  释放与当前参考帧有关的所有资源

- `importFrame()`

  加载帧

- `makePointCloud()`

  对参考帧某一层(level)构建点云，计算了每个像素的3D空间坐标，像素梯度，颜色和方差
### 2. SE3Tracker.h

Tracking线程的主题部分：求解两帧之间的SE3变换

- `SE3Tracker()`

  构造函数，读取相机内参，给各个变量分配内存

 - `checkPermaRefOverlap()`

   检查当前帧与参考帧之间的参考点的重叠度

 - `calcResidualAndBuffers()`

   论文公式13：计算参考点在当前帧下投影点的残差(光度误差)和梯度，并记录参考点在参考帧的逆深度和方差

   - `calcResidualAndBuffers_debugStart()`

     将4个图像的每个像素都设为白色。
     
   - `calcResidualAndBuffers_debugFinish()`

     结束残差计算，如果save/plot为true，就保存/可视化计算的光度误差

 - `calcWeightsAndResidual()`

   计算光度误差损失函数(论文公式12)
   $$
   E_p(\mathbf\xi_{ji}) =\sum_{\mathbf{p}\in\Omega_{D_i}}
   \Biggl\|\frac{r_p^2(\mathbf{p},\mathbf\xi_{ji})}{\sigma_{r_p(\mathbf{p},\mathbf\xi_{ji})}^2}\Biggr\|_\delta \tag{论文12式}
   $$
   计算归一化方差的光度误差系数(论文公式14) 和 Huber-weight(论文公式15)
   $$
   \sigma_{r_p(\mathbf{p},\mathbf\xi_{ji})}^2 := 2\sigma_I^2 + (\frac{\partial{r_p(\mathbf{p}, \mathbf\xi_{ji})}}{\partial{D_i(\mathbf{p})}})^2V_i(\mathbf{p}) \tag{论文14式}
   $$
   因为两帧之间位姿变换小所以作者只考虑了位移没考虑旋转，所以14式的梯度项简化为：
   $$
   \begin{split}
   \frac{\partial{r_p(\mathbf{p},\mathbf\xi_{ji})}}{\partial{D_i(\mathbf{p})}}
   &= \frac{\partial({I_i({\mathbf p}) - I_j(\omega({\mathbf p}, D_i({\mathbf p}), \xi_{ji})))}}{\partial{D_i(\mathbf{p})}} \\
   &= - \frac{\partial{I_j(\mathbf{a})}}{\partial{\mathbf{a}}}\bigg|_{\mathbf{a}=\mathbf{p}} \cdot\frac{\partial{w(d)}}{\partial{d}}\bigg|_{d=D_i(\mathbf{p})} \\
   &= -\begin{pmatrix}dxfx&dyfy\end{pmatrix}\cdot\begin{pmatrix}\frac{\mathbf{t}_x(1/d+\mathbf{t}_z)-\mathbf{t}_z(\mathbf{p}_x/d+\mathbf{t}_x)}{(1/d+\mathbf{t}_z)^2d} \\
   \frac{\mathbf{t}_y(1/d+\mathbf{t}_z)-\mathbf{t}_z(\mathbf{p}_y/d+\mathbf{t}_y)}{(1/d+\mathbf{t}_z)^2d}\end{pmatrix}\\
   &= -(dxfx\frac{\mathbf{t}_xz'-\mathbf{t}_zx'}{z'^2d} + dyfy\frac{\mathbf{t}_yz'-\mathbf{t}_zy'}{z'^2d})
   \end{split} 
   $$
   

## LSD-SLAM的地图优化

### 1. g2oTypeSim3Sophus.h

本文件主要是用Sophus::Sim3d作为顶点(位姿)的数据结构，构建g2o的顶点和边

- `class VertexSim3 : public g2o::BaseVertex<7, Sophus::Sim3d>`

  优化的顶点，每个顶点都是Sim(3)三维相似变换矩阵

- `class EdgeSim3 : public g2o::BaseBinaryEdge<7, Sophus::Sim3d, VertexSim3, VertexSim3>`

  优化的边

### 2. TrackableKeyFrameSearch.h

本文件主要用于查找当前keyframe可以跟踪到的其他keyframe，以便将新约束放入图优化中

- `TrackableKeyFrameSearch()`构造函数

  创建SE3Tracker的实例

- `findCandidates()`

  寻找当前关键帧可以追踪到的其他关键帧(作为参考帧)

- `findRePositionCandidate()`

  寻找当前关键帧可以追踪到的其他关键帧，并计算该关键帧的分数，如果不够高重新计算该关键帧的位姿

- `getRefFrameScore()`

  得到参考帧的分数

- `findEuclideanOverlapFrames()`

  用来寻找某一个关键帧所有潜在的参考帧

### 3. KeyFrameGraph.h

这个类用于创建g2o需要使用的图

- `KFConstraintStruct()`结构体构造函数

  这个结构体主要包含图优化用到的变亮

- `KeyFrameGraph()`类构造函数

  构建一个新的g2o位姿图

- `addKeyFrame()`

  将新的关键帧添加到位姿图中去

- `dumpMap()`

  将关键帧图优化的结果以及相关的信息保存到指定的文件夹中

- `insertConstraint()`

  向关键帧图中添加一个新的约束（constraint），这个约束用 EdgeSim3 类表示

- `optimize()`

  执行图优化，只会更新顶点的位姿，关键帧的位姿没有更新，需要优化完后再调用`updateKeyFramePoses()`

- `calculateGraphDistancesToFrame()`

  计算一个给定的帧到图中所有其他帧的最短距离，并保存在distanceMap中
