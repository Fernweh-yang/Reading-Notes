
# LOAM论文

## 进化路线：

暂时无法在飞书文档外展示此内容

- LOAM:
    
    - 依靠计算点云的粗糙度roughness来提取特征
        
        - 低粗糙度：平面特征planar feature
            
        - 高粗糙度：边缘特征edge feature
            
    - 缺点：
        
    
      地面的噪声(如草地的高roughness)会导致不可靠的edge feature
    
- LeGO-LOAM:
    
    - 缺点：
        
    
      没有研究地面分割ground segmentation对里程计的影响, 使用的地面分割算法只考虑了两点云之间的几何关系geometrical relation(angle difference)。但如草坪和灌木丛引起疙瘩bumpy会影响语义的判断，因为他们形状的梯度是任意的。
    
- PaGO-LOAM
    
    - 用patchwork来做地面分割
        

## [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)

- Overview
    
    ![](https://r712him1th.feishu.cn/space/api/box/stream/download/asynccode/?code=YjcwZjFiYWJmYzNmZGFlNzVjZWM5MmMxYzUyMWI0YWJfZFRKQXRweUFESFNiRjNsY2U5S2lBUUZNRDd2Q2lrR0ZfVG9rZW46RWhDRmJIN09tb2FvNXl4RGtYc2NVTjVKbjZkXzE3MTcwNjY1NDU6MTcxNzA3MDE0NV9WNA)
    
- Segmentation
    

经过地面分割后，可以得到每个点云的：

1. 语义：ground or segmented
    
2. 在深度图range image中的坐标
    
3. 在深度图中的深度range value
    

- Feature Extraction
    
    - 计算点云的粗糙度roughness:
        
    
      $$c=\frac{1}{|S|\cdot\|r_i\|}\Big\|\sum_{j\in S, j\neq i}(r_j-r_i)\Big\|$$
    
    - S: 点云集合
        
    - $$r$$: 深度值range value
        
    
    - 有一个阈值 $$C_{threshold}$$
        
        - 粗糙度>该阈值是边缘特征点
            
        - 粗糙度<该阈值是平面特征点
            
    - 一些符号：
        
        - 论文将360度深度图划分成了6个区域，由于用的16线雷达，所以每个sub-image的深度图range image分辨率为300*16
            
        - $$\mathbb{F}_e, \mathbb{F}_p$$: 不明显的edge和planar feature集合
            
        
             每个sub-image论文取了40, 80个点
        
        - $$F_e, F_p$$: 明显的edge和planar feature集合
            
                  每个sub-image论文取了2,4个点
            
- Lidar Odometry
    

和LOAM一样执行点云匹配：point-to-edge/plane scan-matching。即寻找 $$F^t$$和 $$\mathbb{F}^{t-1}$$之间相匹配的特征关系。

LeGO-LOAM中的对点云匹配的两个性能提升：

1. Label Matching: 由于前面地面分割后有了语义，所以只在相同语义标签下的点之间寻找匹配关系。
    
2. Two-step L-M Optimization:
    

  LOAM中把上下两帧的edge和planar特征整合成一个复杂的距离向量，然后用L-M去求最短距离。

![](https://r712him1th.feishu.cn/space/api/box/stream/download/asynccode/?code=Y2UxMjBjYjRjYjJhNDU5ZWY1ODliMTYzOTc3NjcxNjdfR3M4dGFvS1BhVFdwazlLTzQxVmVnQXp3dGg5S1doTmFfVG9rZW46U0JPQWJwbkU2b1FZb1l4bmxMb2NLTmU3bjRjXzE3MTcwNjY1NDU6MTcxNzA3MDE0NV9WNA)

  LeGO中把点云匹配分为两步：

1. 用属于planar feature的点云匹配去计算 $$t_z,\theta_{roll},\theta_{pitch}$$
    
2. 用属于edge feature的点云匹配和上面计算得到的值去计算 $$t_x, t_y, \theta_{yaw}$$
    

- Lidar Mapping
    
    - 也和LOAM一样会
        
        - 把当前特征的 $$\{\mathbb{F}_e^t,\mathbb{F}_p^t\}$$的点云和上一周期的周围的点云图匹配来改善位姿估计
            
        - 在用LM来优化结果
            
    - 相比LOAM的区别：地图的存储
        
        - LOAM：存储一个单一的点云地图
            
        - LeGO：存储为每一个单独的特征集,集合： $$M^{t-1}=\{\{\mathbb{F}_e^1,\mathbb{F}_p^1\},\cdots,\{\mathbb{F}_e^{t-1},\mathbb{F}_p^{t-1}\}\}$$中保存了所有之前的特征集feature set. 每一个特征集都关联着一个传感器的位姿。
            

## [PaGO-LOAM](http://10.10.88.24/algorithm_toolchain/algorithm/hx-calibraiton/pago_loam)

使用Patchwork来实现地面分割

- Overview
    

![](https://r712him1th.feishu.cn/space/api/box/stream/download/asynccode/?code=OGU1OTNhZTA4ZjY3ZDc5MTM0NmVmNmFhZGFmNTFmNTJfVVlJTmZza1FOM3paSXlTS2lLQ1dZNVVNRmV1aWprb0FfVG9rZW46TzVuT2J1VzhEbzQxZXZ4dTJkbGNLaG5FbkdmXzE3MTcwNjY1NDU6MTcxNzA3MDE0NV9WNA)

- 流程
    
    - 根据地面分割的结果，把2种点云分别映射到深度图和地面图
        
    - 和[LeGO](https://r712him1th.feishu.cn/wiki/SQm3w8FdFiTcLfkCJFccdocOnud?larkTabName=space#part-OzCpdvN0GopwUcxAGkZczWsfnSc)一样根据粗糙度提取edge/planar特征
        
    - 和[LeGO](https://r712him1th.feishu.cn/wiki/SQm3w8FdFiTcLfkCJFccdocOnud?larkTabName=space#part-Rc5rdyrLWoHxlQx7bEecuLCCnCc)一样根据特征间的关系用两步法估计位姿
        
    - 将特征映射到全局点云地图
        
    - 用里程计的结果累积点云地图
        

  

# 地面分割算法

## 两算法的区别：

- 两算法的overview:
    
    ![](https://r712him1th.feishu.cn/space/api/box/stream/download/asynccode/?code=NDA5NzRlMGYzNzVhNTAzMDEwYWQ4NzM2ODg5NzYxOTVfUngxZEZnZmpvTWV3ZlpOZ2c1MGFXVkJnNW9sNnRwWnZfVG9rZW46VWU0MmJvTWt2b0xXM2d4Z1dIc2NYY29CbmxmXzE3MTcwNjY1NDU6MTcxNzA3MDE0NV9WNA)
    

## Patchwork

是R-GPF的扩展

- 主要组成部分
    

由上面的overview可知，有3个主要部分：

1. 基于同心带模型Concentric Zone Model(CZM)的极坐标网格表示法
    

  一种高效、非均匀、按区域划分的3D点云表示方法。

2. 区域地面拟合Region-wise GRound PLane Fitting(R-GPF)
    

  用于从3D点云数据中识别和拟合地面平面。

3. 地面似然估计Ground Likelihood Estimation(GLE)
    

  利用垂直度uprightness、海拔elevation和平整度flatness来二分类每一个bin是否是地面，

- 问题描述：
    
      对于N个点云 $$p_k=\{x_k,y_k,z_k\}$$组成的点云集合 $$\mathcal{P}=\{p_1,p_2,\cdots,p_N\}$$,可以划分为两个集合：地面点 $$G$$和非地面点 $$G^c$$。希望算法估计得到的 $$\hat{G}，\hat{G}^{c}$$越准越好。
    
- 基于CZM的极坐标网格表示法
    
    ![](https://r712him1th.feishu.cn/space/api/box/stream/download/asynccode/?code=NDVlMmRmZWViNWYzMTRmMjJjODFlMjk5MDljMDJjYjhfMWp4OW9rd1ZZczRzTkdxY1FqR25XdnE5WmNMRE1aTnlfVG9rZW46TGM0MWJEVnFob1N3d1N4cE05WmM4azRObjNaXzE3MTcwNjY1NDU6MTcxNzA3MDE0NV9WNA)
    
      因为地面是不平整的，所以需要将地面划分成一个个bin，分别的判断每个bin是否是ground。以前的方法如图a所示有2个问题：1. Sparsity issue:距离越远网格越稀疏。2. Representability issue: 靠近中心的地方太小了无法表示一个区域。
    
      为解决上面这2个问题，作者提出了一个多区域(每个区域bin size不同)的分法。
    
    - 一些符号：
        
        - $$Z_m$$: 表示第m个区域
            
        - $$S_{i,j,m}$$：表示第m个区域的第[i,j]个bin
            
- R-GPF
    

通过R-GPF给每一个bin分配一个estimated partial ground，先提取种子点，根据种子点使用主成分分析PCA拟合平面，根据每个patch中点到平面的距离划分成ground和nonground，

- GLE
    

用垂直度uprightness、海拔elevation和平整度flatness来确定该点云是否真的属于ground。

网上评价：R-GPF中已经被标记成ground无法在GLE后再标记为nonground，所以十分依赖种子点的选取。并且，参数太多，需要针对不同的场景进行调参。最后，代码和论文出入太大了，几乎只用上了法向量的uprightness，似然函数那部分几乎完全没实现。

## Patchwork++

![](https://r712him1th.feishu.cn/space/api/box/stream/download/asynccode/?code=NWU2YTA4MDZkN2I4ZjY0ZWE2MGZlMmUxNGQwYjBmMzBfR1U4aUxkdXEyb2w3bXFla29YNlA3Unp5RmRmbThKVWRfVG9rZW46RTcwbGJhQlhUb05rQ1N4WDFkRWNqWjB0bmhlXzE3MTcwNjY1NDU6MTcxNzA3MDE0NV9WNA)

为什么要扩展Patchwork

- patchwork需要根据环境精细化的调整参数
    
- 在一些地区会欠分割
    
- 如果地面相比其他平面有分层，也会失败
    

相比Patchwork多了4个模块：

- RNR(reflected noise removal): 利用3D雷达的反射模型来消除virtual noise points
    
- R-VPF(Region-wise vertical plane fitting)：即使地面有好几层，也可以准确的分割出来
    
- A-GLE(Adaptive ground likelihood estimation)：根据地面分割的结果自适应的计算GLE参数
    
- TGR(Temporal ground revert)：通过使用临时地面性质temporary ground property来缓解欠分割的问题
    

  

# LIO

## 进化路线

## 仓库汇总

[FAST_LIO_MULTI](https://github.com/engcang/FAST_LIO_MULTI)

[FAST_LIO](https://github.com/hku-mars/FAST_LIO)

## Fast-LIO

  

## Fast-LIO2

### 本文两个关键创新点

1. 使用增量KD树(ikd-tree)来维护地图，可以以很小的代价实现：最近邻搜索、地图的增量更新和动态再平衡。
    
2. 借助ikd-tree，可以接使用原始点不提取特征来建图，就像vslam中的直接法一样。
    

### 当前LSLAM的难点

1. 激光每秒生成的大量3D点数据，很难由机载计算机实时计算。
    
2. 通过特征点可以减少计算量，但特征提取算法又容易受环境影响。
    
3. 由于雷达自身运动/动态物体等导致的点云畸变会影响SLAM的准确性，IMU可以消除畸变但又会引入零偏和测量噪声等状态的估计问题。
    
4. 不像VSLAM，每一帧图像都是高分辨率的，可以只维护一个稀疏的地图，因为有图像很多特征点可用于匹配。
    
      但激光每次扫描点云分布在几百米的3D空间内，所以就需要维护一个稠密的地图来保证新扫描得到稀疏点云能在地图中找到匹配点。
    

### Overview

![](https://r712him1th.feishu.cn/space/api/box/stream/download/asynccode/?code=N2Q2MmE1NzRkM2U0MzVhMDI5Zjc2NGE0NzIzYmQ5YmFfQnlvOUx1UHlDZUlGNm5SSjJHczZyQkZGdnIzTFFDV05fVG9rZW46TjJ2MGIxNXdib21ibEh4dzBKY2N2aHpNbmZiXzE3MTcwNjY1NDU6MTcxNzA3MDE0NV9WNA)

1. 点云原始数据会在一个10ms-100ms的区间内累积成一个scan。
    
2. IMU相对于点云来说是高频数据，会提前根据IMU运动方程计算运动状态(前向传播)
    
3. 累积好后的点云根据IMU进行去畸变
    
4. 误差状态卡尔曼滤波器ESKF计算残差和雅可比矩阵
    
5. 迭代卡尔曼滤波器IEKF进行状态估计。收敛后的后验位姿估计会被用于更新地图。
    
6. ikd-Tree维护着全局地图中一个固定尺寸(map size)的区域。
    
    1. 如果新scan的点超过了这个区域，ikd-tree就会移除最远的地图