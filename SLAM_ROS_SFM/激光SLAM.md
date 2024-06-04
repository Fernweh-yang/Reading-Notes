
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

- Overview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
  

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

### 主要贡献

1. 用紧耦合迭代卡尔曼滤波来融合LiDAR特征点和IMU测量值。提出了一个反向传播来补偿运动仿真。
2. 提出了一个新的计算卡尔曼增益的公式来降低计算复杂度

### Overview

![image-20240603104411404](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/imgimage-20240603104411404.png)

- 对点云数据进行特征提取得到：边缘特征和平面特征
- 特征+IMU数据进入10-50hz的状态估计模块得到：当前位姿
- 根据位姿将特征点从雷达坐标系转到全局坐标系得到：一个包含当前和之前所有特征点的地图
- 用更新后的特征点地图来处理下一帧的特征点。

### Methods

#### 模型定义：

可参考自动驾驶中的slam笔记

- 连续模型continuouse model:
  $$
  {}^{G}\dot{\mathbf{p}}_{I}={}^{G}\mathbf{v}_{I},{}^{G}\dot{\mathbf{v}}_{I}={}^{G}\mathbf{R}_{I}\left(\mathbf{a}_{m}-\mathbf{b}_{\mathbf{a}}-\mathbf{n}_{\mathbf{a}}\right)+{}^{G}\mathbf{g},{}^{G}\dot{\mathbf{g}}=\mathbf{0}\\{}^{G}\dot{\mathbf{R}}_{I}={}^{G}\mathbf{R}_{I}\lfloor\mathbf{\omega}_{m}-\mathbf{b}_{\boldsymbol{\omega}}-\mathbf{n}_{\boldsymbol{\omega}}\rfloor_{\wedge}, \dot{\mathbf{b}}_{\boldsymbol{\omega}}=\mathbf{n}_{\mathbf{b}\boldsymbol{\omega}}, \dot{\mathbf{b}}_{\mathbf{a}}=\mathbf{n}_{\mathbf{b}\mathbf{a}}\tag{1}
  $$

  - ${}^{I}\mathbf{T}_{L} {=} \left({}^{I}\mathbf{R}_{L},{}^{I}\mathbf{p}_{L}\right)$：已知IMU坐标系(I)相对于雷达坐标系(L)的外参
  - ${}^G\mathbf{p}_I,{}^G\mathbf{R}_I$：IMU在全局坐标系下的位姿（position+attitude）
    - 第一个IMU坐标系被定义为全局坐标系(G)
  - ${}^G\mathbf{g}$：一个未知的重力向量
  - $\mathbf{a}_m，\mathbf{\omega}_m$：IMU测量值
  - $\mathbf{n}_\mathbf{n},\mathbf{n}_{\omega}$：IMU的测量噪声
  - $\mathbf{b}_a,\mathbf{b}_{\omega}$​：IMU的零偏
  - $\left\lfloor\mathbf{a}\right\rfloor_{\wedge}$：向量a的反对称矩阵
  - $\mathbf{n}_\mathbf{ba},\mathbf{n}_{b\omega}$：IMU零偏的高斯噪声

- 离散模型Discrete model:
  $$
  \mathbf{x}_{i+1}=\mathbf{x}_i\boxplus(\Delta t\mathbf{f}(\mathbf{x}_i,\mathbf{u}_i,\mathbf{w}_i))\tag{2}
  $$

  - i：第i个IMU测量值

  - function f , state x, input u and noise w定义如下：
    $$
    \begin{aligned}
    &\mathcal{M}=SO(3)\times\mathbb{R}^{15}, \mathrm{dim}(\mathcal{M})=18 \\
    &\mathbf{x}\doteq\begin{bmatrix}{}^{G}\mathbf{R}_{I}^{T}&{}^{G}\mathbf{p}_{I}^{T}&{}^{G}\mathbf{v}_{I}^{T}&\mathbf{b}_{\omega}^{T}&\mathbf{b}_{\mathbf{a}}^{T}&{}^{G}\mathbf{g}^{T}\end{bmatrix}^{T}\in\mathcal{M} \\
    &\mathbf{u}\doteq\begin{bmatrix}\mathbf{\omega}_{m}^{T}&\mathbf{a}_{m}^{T}\end{bmatrix}^{T}, \mathbf{w}\doteq\begin{bmatrix}\mathbf{n}_{\mathbf{\omega}}^{T}&\mathbf{n}_{\mathbf{a}}^{T}&\mathbf{n}_{\mathbf{b}\mathbf{\omega}}^{T}&\mathbf{n}_{\mathbf{b}\mathbf{a}}^{T}\end{bmatrix}^{T} \\
    &\mathbf{f}\left(\mathbf{x}_{i},\mathbf{u}_{i},\mathbf{w}_{i}\right)=\begin{bmatrix}\boldsymbol{\omega}_{{m_{i}}}-\mathbf{b}_{{\boldsymbol{\omega}_{i}}}-\mathbf{n}_{{\boldsymbol{\omega}_{i}}}\\{{}^{G}\mathbf{v}_{{I_{i}}}}\\{{}^{G}\mathbf{R}_{{I_{i}}}\left(\mathbf{a}_{{m_{i}}}-\mathbf{b}_{{\mathbf{a}_{i}}}-\mathbf{n}_{{\mathbf{a}_{i}}}\right)+{}^{G}\mathbf{g}_{i}}\\{{\mathbf{n}_{{\mathbf{b}\boldsymbol{\omega}_{i}}}}}\\{{\mathbf{n}_{{\mathbf{b}\mathbf{a}_{i}}}}}\\{\mathbf{0}_{3\times1}}\end{bmatrix}
    \end{aligned}
    $$

#### 符号定义：

| Symbols                                             | Meaning                                                      |
| --------------------------------------------------- | ------------------------------------------------------------ |
| $t_k$                                               | The scan-end time  of the k-th LiDAR scan                    |
| $\tau_i$                                            | The i-th IMU sample time in a LiDAR scan.                    |
| $\rho_j$                                            | The j-th feature point’s sample time in a LiDAR scan.        |
| $I_i,I_j,I_k$                                       | The IMU body frame at the time $τ_i, ρ_j$ and $t_k$.         |
| $L_i,L_k$                                           | The LiDAR body frame at the time $ρ_j$ and $t_k$.            |
| $\mathbf{x},\hat{\mathbf{x}},\overline{\mathbf{x}}$ | The ground-true, propagated, and updated(optimal) value of $\mathbf{x}$. |
| $\widetilde{\mathbf{x}}$                            | The error between ground-true $\mathbf{x}$ and its estimation $\overline{\mathbf{x}}$. |
| $\hat{\mathbf{x}}^k$                                | The κ-th update of $\mathbf{x}$ in the iterated Kalman filter. |
| $\mathbf{x}_i,\mathbf{x}_j,\mathbf{x}_k$            | The vector (e.g.,state) x at time $τ_i, \rho_j$ and $t_k$.   |
| $\check{\mathbf{x}}_j$                              | Estimate of $x_j$relative to $x_k$ in the back propagation.  |

作者还定义了2个符号
$$
\begin{aligned}&\mathcal{M}=SO(3):\mathbf{R}\boxplus\mathbf{r}=\mathbf{R}\mathrm{Exp}(\mathbf{r});\quad\mathbf{R}_{1}\boxminus\mathbf{R}_{2}=\mathrm{Log}(\mathbf{R}_{2}^{\mathrm{T}}\mathbf{R}_{1})\\&\mathcal{M}=\mathbb{R}^{n}:\quad\mathbf{a}\boxplus\mathbf{b}=\mathbf{a}+\mathbf{b};\quad\mathbf{a}\boxminus\mathbf{b}=\mathbf{a}-\mathbf{b}\end{aligned}
$$

#### 状态估计算法总结：

下面使用**IEKF(Iterated extended Kalman Filter)**来估计2式中的状态变量。

在$t_{k-1}$时刻最优状态估计为$\overline{\mathbf{x}}_{k-1}$，它的协方差covariance matrix$\overline{\mathbf{P}}_{k-1}$代表了任意状态变量的误差向量：
$$
\widetilde{\mathbf{x}}_{k-1}\doteq\dot{\mathbf{x}}_{k-1}\boxplus\bar{\mathbf{x}}_{k-1}=\begin{bmatrix}\delta\boldsymbol{\theta}^{T}&{}^G\widetilde{\mathbf{p}}_{I}^{T}&{}^G\widetilde{\mathbf{v}}_{I}^{T}&\widetilde{\mathbf{b}}_{\omega}^{T}&\widetilde{\mathbf{b}}_{\mathbf{a}}^{T}&{}^G\widetilde{\mathbf{g}}^{T}\end{bmatrix}^{T} \tag{3}
$$

- 4,8式相当于传统KF中的预测部分
- 18,20式相当于传统KF中的更新部分，但是迭代的，收敛后得到最后的更新式
- 中间的8/9/10/12/14/16等式都是融合了点云等信息来计算过滤误差的，用于生成状态更新式

![image-20240603141138729](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/imgimage-20240603141138729.png)

- 一收到IMU就前向传播，且噪声设为0，得到：

  - 状态估计
    $$
    \widehat{\mathbf{x}}_{i+1}=\widehat{\mathbf{x}}_i\boxplus(\Delta t\mathbf{f}(\widehat{\mathbf{x}}_i,\mathbf{u}_i,\mathbf{0})); \widehat{\mathbf{x}}_0=\bar{\mathbf{x}}_{k-1}\tag{4}
    $$

  - 状态协方差估计
    $$
    \widehat{\mathbf{P}}_{i+1}=\mathbf{F}_{\widetilde{\mathbf{x}}}\widehat{\mathbf{P}}_i\mathbf{F}_{\widetilde{\mathbf{x}}}^T+\mathbf{F}_{\mathbf{w}}\mathbf{Q}\mathbf{F}_{\mathbf{w}}^T; \widehat{\mathbf{P}}_0=\bar{\mathbf{P}}_{k-1}\tag{8}
    $$

    - $\mathbf{Q}$：噪声$\mathbf{w}$的协方差

- 当$t_{k-1}-t_k$期间累积的的点云到达，需要和上面IMU估计得到的状态$\hat{\mathbf{x}}_k$和协方差$\hat{\mathbf{P}}_k$的融合，来生成一个最优状态更新。但显然各个特征点的的提取时间如下图所示军在$t_k$之前，无法直接融合，这时候就需要通过反向传播9式来得到每个特征点相对于$t_k$时刻的相对位姿，最后由10式得到该点云在$t_k$​时刻的映射值：

  - 反向传播：

  $$
  \begin{aligned}
  {}^{I_{k}}\check{\mathbf{p}}_{I_{j-1}}& ={}^{I_{k}}\check{\mathbf{p}}_{I_{j}}-{}^{I_{k}}\check{\mathbf{v}}_{I_{j}}\Delta t,\quad s.f.{}^{I_{k}}\check{\mathbf{p}}_{I_{m}}=\mathbf{0}; \\
  {}^{I_{k}} \check{\mathbf{V}}_{I_{j-1}}& =^{I_{k}}\check{\mathbf{v}}_{I_{j}}-^{I_{k}}\check{\mathbf{R}}_{I_{j}}(\mathbf{a}_{m_{i-1}}-\widehat{\mathbf{b}}_{\mathbf{a}_{k}})\Delta t-^{I_{k}}\widehat{\mathbf{g}}_{k}\Delta t, \\
  &s.f.{}^{I_{k}}\check{\mathbf{v}}_{I_{m}}={}^{G}\widehat{\mathbf{R}}_{I_{k}}^{T G}\widehat{\mathbf{v}}_{I_{k}},{}^{I_{k}}\widehat{\mathbf{g}}_{k}={}^{G}\widehat{\mathbf{R}}_{I_{k}}^{T G}\widehat{\mathbf{g}}_{k}; \\
  {}^{I_{k}}\check{\mathbf{R}}_{I_{j-1}}& ={}^{I_{k}}\check{\mathbf{R}}_{I_{j}}\mathrm{Exp}((\widehat{\mathbf{b}}_{\boldsymbol{\omega}_{k}}-\boldsymbol{\omega}_{m_{i-1}})\Delta t), s.f.{}^{I_{k}}\mathbf{R}_{I_{m}}=\mathbf{I}. 
  \end{aligned}\tag{9}
  $$

  - 映射到$t_k$​时刻：

    得到$t_k$时刻雷达坐标系下的特征点的坐标
    $$
    {}^{L_k}\mathbf{p}_{f_j}={}^I\mathbf{T}_L^{-1I_k}\check{\mathbf{T}}_{I_j}{}^I\mathbf{T}_L{}^{L_j}\mathbf{p}_{f_j}\tag{10}
    $$

  - 

  ![image-20240603145653389](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/imgimage-20240603145653389.png)

- 用11式将$t_k$时刻雷达坐标系下的特征点的坐标转为世界坐标系后，用12式计算残差

  - 转换坐标系
    $$
    {}^{G}\widehat{\mathbf{p}}_{f_{j}}^{\kappa}={}^{G}\widehat{\mathbf{T}}_{I_{k}}^{\kappa I}\mathbf{T}_{L}{}^{L_{k}}\mathbf{p}_{f_{j}} ; j=1,\cdots,m\tag{11}
    $$

  - 计算残差
    $$
    \mathbf{z}_j^\kappa=\mathbf{G}_j({}^G\widehat{\mathbf{p}}_{f_j}^\kappa-{}^G\mathbf{q}_j)\tag{12}
    $$

- 为了将12式的残差加入到状态$\hat{\mathbf{x}}_k$和协方差$\hat{\mathbf{P}}_k$更新中来，需要首先计算残差$\mathbf{z}_j^k$相对于$\mathbf{x}_k$和测量噪声的残差的雅可比矩阵来线性化：
  $$
  \begin{aligned}\mathbf{0}&=\mathbf{h}_{j}\left(\mathbf{x}_{k},{}^{L_{j}}\mathbf{n}_{f_{j}}\right)\simeq\mathbf{h}_{j}\left(\widehat{\mathbf{x}}_{k}^{\kappa},\mathbf{0}\right)+\mathbf{H}_{j}^{\kappa}\widetilde{\mathbf{x}}_{k}^{\kappa}+\mathbf{v}_{j}\\&=\mathbf{z}_{j}^{\kappa}+\mathbf{H}_{j}^{\kappa}\widetilde{\mathbf{x}}_{k}^{\kappa}+\mathbf{v}_{j}\end{aligned}\tag{14}
  $$

- 计算一个复杂的雅可比矩阵：
  $$
  \mathbf{J}^\kappa=\begin{bmatrix}\mathbf{A}\Big({}^G\widehat{\mathbf{R}}_{I_k}^\kappa\boxminus{}^G\widehat{\mathbf{R}}_{I_k}\Big)^{-T}&\mathbf{0}_{3\times15}\\\mathbf{0}_{15\times3}&\mathbf{I}_{15\times15}\end{bmatrix}\tag{16}
  $$

- 最后通过20式计算卡尔曼增益，18式计算新的状态更新值：

  - 卡尔曼增益(本文新提出的一种K的计算公式)
    $$
    \mathbf{K}=(\mathbf{H}^T\mathbf{R}^{-1}\mathbf{H}+\mathbf{P}^{-1})^{-1}\mathbf{H}^T\mathbf{R}^{-1}\tag{20}
    $$

  - 迭代更新状态
    $$
    \widehat{\mathbf{x}}_k^{\kappa+1}=\widehat{\mathbf{x}}_k^\kappa\boxplus\left(-\mathbf{K}\mathbf{z}_k^\kappa-(\mathbf{I}-\mathbf{K}\mathbf{H})(\mathbf{J}^\kappa)^{-1}\left(\widehat{\mathbf{x}}_k^\kappa\boxminus\widehat{\mathbf{x}}_k\right)\right)\tag{18}
    $$

- 最后如果收敛了就得到最后的更新式(算法第10行)：
  $$
  \bar{\mathbf{x}}_k=\widehat{\mathbf{x}}_k^{\kappa+1}; \bar{\mathbf{P}}_k=(\mathbf{I}-\mathbf{K}\mathbf{H}) \mathbf{P}.
  $$

- 

## Fast-LIO2

### 代码安装

在docker中运行

1. 创建脚本

   ```
   touch <your_custom_name>.sh
   ```

2. 在脚本中填入

   ```sh
   #!/bin/bash
   mkdir data
   # Script to run ROS Kinetic with GUI support in Docker
   
   # Allow X server to be accessed from the local machine
   xhost +local:
   
   # Container name
   CONTAINER_NAME="fastlio2"
   
   # Run the Docker container
   docker run -itd \
     --name=$CONTAINER_NAME \
     --user mars_ugv \
     --network host \
     --ipc=host \
     -v /home/$USER/data:/home/mars_ugv/data \
     --privileged \
     --env="QT_X11_NO_MITSHM=1" \
     --volume="/etc/localtime:/etc/localtime:ro" \
     -v /dev/bus/usb:/dev/bus/usb \
     --device=/dev/dri \
     --group-add video \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     --env="DISPLAY=$DISPLAY" \
     kenny0407/marslab_fastlio2:latest \
     /bin/bash
   ```

3. 给予权限

   ```
   sudo chmod +x <your_custom_name>.sh
   ```

4. 执行脚本

   ```
   ./<your_custom_name>.sh
   ```

### 代码运行

1. 下载数据集

   把rosbag文件峰哪个如上面创建的data文件夹内

2. 运行:

   它提供的image里没装gdb，如果想要debug，需要现在容器内安装gdb：

   ```
   sudo apt-get update
   sudo apt-get install geb
   ```

   - 对于激光雷达：Livox Avia

     数据集：[google drive](https://drive.google.com/drive/folders/1CGYEJ9-wWjr8INyan6q1BZz_5VtGB-fP?usp=sharing)

     ```
     roslaunch fast_lio mapping_avia.launch
     rosbag play YOUR_DOWNLOADED.bag
     ```

   - 对于激光雷达：Velodyne HDL-32E 

     作者提供的[Rosbag Files](https://drive.google.com/drive/folders/1blQJuAB4S80NwZmpM6oALyHWvBljPSOE?usp=sharing) 和 [a python script](https://drive.google.com/file/d/1QC9IRBv2_-cgo_AEvL62E1ml1IL9ht6J/view?usp=sharing)来生成rosbag文件：`python3 sensordata_to_rosbag_fastlio.py bin_file_dir bag_name.bag`

     ```
     roslaunch fast_lio mapping_velodyne.launch
     rosbag play YOUR_DOWNLOADED.bag
     ```

### 主要贡献

1. 使用增量KD树(ikd-tree)来维护地图，可以以很小的代价实现：最近邻搜索、地图的增量更新和动态再平衡。
   
2. 借助ikd-tree，可以接使用原始点不提取特征来建图，就像vslam中的直接法一样。
   

### 当前LSLAM的难点

1. 激光每秒生成的大量3D点数据，很难由机载计算机实时计算。
   
2. 通过特征点可以减少计算量，但特征提取算法又容易受环境影响。
   
3. 由于雷达自身运动/动态物体等导致的点云畸变会影响SLAM的准确性，IMU可以消除畸变但又会引入零偏和测量噪声等状态的估计问题。
   
4. 不像VSLAM，每一帧图像都是高分辨率的，可以只维护一个稀疏的地图，因为有图像很多特征点可用于匹配。
   
      但激光每次扫描点云分布在几百米的3D空间内，所以就需要维护一个稠密的地图来保证新扫描得到稀疏点云能在地图中找到匹配点。

### Overview

![image-20240603110246250](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/imgimage-20240603110246250.png)

1. 点云原始数据会在一个10ms-100ms(10-100hz)的区间内累积成一个scan。
   
2. IMU相对于点云来说是高频数据，会提前根据IMU运动方程计算运动状态(前向传播)
   
3. 累积好后的点云根据IMU进行去畸变
   
4. 误差状态卡尔曼滤波器ESKF计算残差和雅可比矩阵
   
5. 迭代卡尔曼滤波器IEKF进行状态估计。收敛后的后验位姿估计会被用于更新地图。
   
6. ikd-Tree维护着全局地图中一个固定尺寸(map size)的区域。
   
    1. 如果新scan的点超过了这个区域，ikd-tree就会移除最远的地图

### Methods

#### 状态估计

状态估计算法详情如下：和fast-lio一模一样，唯独多了个将第$k$-th的scan（即$t_{k-1}-t_k$）之间的点云转换到全局坐标系的步骤：
$$
{}^G\bar{\mathbf{p}}_j={}^G\bar{\mathbf{T}}_{I_k}{}^I\bar{\mathbf{T}}_{L_k}{}^L\mathbf{p}_j; j=1,\cdots,m\tag{16}
$$
变换后的点云${}^G\bar{\mathbf{p}}_j$会被放入ikd-Tree表示的地图中去

![image-20240603160152524](/home/hx/.config/Typora/typora-user-images/image-20240603160152524.png)

