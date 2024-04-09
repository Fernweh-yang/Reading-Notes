# 零、资料汇总

- 官方原版[git仓库](https://github.com/gaoxiang12/slam_in_autonomous_driving)

  包含书上的代码还有数据集

  - 各章代码在`src/chxx`下
  - 共有代码在`src/common`下

- 他人上深蓝课的[git仓库](https://github.com/ClarkWang1214/slam_in_autonomous_driving_shenlan_hw?tab=readme-ov-file)

  额外的有课后作业答案和修改代码

# 一、自动驾驶概念

- 自动驾驶应具备的能力

  - **感知**：实时的识别周围车辆的种类和行人
  - **定位**：判断自车的方向和位置，以及自车与上述元素之间的位置关系
  - **规划控制**：在上述信号识别的基础上，控制车辆的油门、刹车、方向盘等执行机构；规划自身短期和长期的行驶路线

- SAE自动驾驶分级：美国汽车工程师学会分级

  - L0-L2: 驾驶员负责驾驶车辆

    典型功能有：

    - LCC：Lane Centering Control车道居中辅助
    - ACC：Adaptive Cruise Control自适应巡航
    - ALC：Auto Lane Change智能换道辅助

  - L3-L4：计算机负责驾驶

    - L3：还是要人接管的，可以自动泊车、自动召唤
    - L4：在特定区域内不需要接管，去除了方向盘、踏板
    - L5：全工况下自动驾驶

- 高精地图和导航地图

  - 导航地图：如谷歌地图，有多边形和向量来表示道路和路口
  - 高精地图：还会精确标注各车道的位置、停止线的位置、周围各种物体的精确位置和信息
    - L2级别不需要高精地图，更多依靠实时感知，感知结果可以直接构建鸟瞰图(Bird Eye View, BEV)
    - L4级别需要高精地图，因为L4是在脑中的地图开车而不是看着路开车。
    - 高精地图本质上是结构化的向量数据，如车道形状、车道数、限速多少、是否是机动车道等信息。通常用JSON、XML、Protobuf语言来描述和储存。

# 二、数据基础

## 1. 坐标系

车辆本体坐标系一般使用前(x)左(y)上(z)，相机坐标系一般使用右(x)下(y)前(z))的顺序

本书的坐标系变换定义为：
$$
\mathbf{p}_w=\mathbf{R}_{wb}\mathbf{p}_b+\mathbf{t}_{wb}\\
\mathbf{p}_w=\mathbf{T}_{wb}\mathbf{p}_b
$$

- $\mathbf{p}_w$：世界坐标系下的p点坐标

- $\mathbf{p}_b$：车辆坐标系下的p点坐标

- $\mathbf{R}_{wb},\mathbf{t}_{wb},\mathbf{T}_{wb}$：是**向量之间的坐标变换**，不是坐标轴(基)之间的变换

  坐标变换和基变换互为逆，如下所示式1为坐标变换，式2为基变换

基变换
$$
\begin{align}
AX'=X \tag{1.1}\\
X'=A^{-1}X \tag{1.2}
\end{align}
$$

- X'是以基向量$\left [\begin{array}{cccc}
  3  \\
  1   \\
  \end{array}\right]\left [\begin{array}{cccc}
  1  \\
  2   \\
  \end{array}\right]$为坐标的向量

- X是以基向量$\left [\begin{array}{cccc}
  1  \\
  0  \\
  \end{array}\right]\left [\begin{array}{cccc}
  0  \\
  1   \\
  \end{array}\right]$为坐标的向量

- $A=\left [\begin{array}{cccc}
  3 & 1 \\
  1 & 2  \\
  \end{array}\right]$

- 如果有个在$\left [\begin{array}{cccc}
  1  \\
  0  \\
  \end{array}\right]\left [\begin{array}{cccc}
  0  \\
  1   \\
  \end{array}\right]$坐标系下的线性变换M，想看看$\left [\begin{array}{cccc}
  3  \\
  1   \\
  \end{array}\right]\left [\begin{array}{cccc}
  1  \\
  2   \\
  \end{array}\right]$下的向量X'经过这个线性变换M会变成什么样：
  $$
  X_{new}'=A^{-1}MAX'
  $$

  1. 需要先$AX'$将向量X'转为和线性变换M一样的坐标系，再进行M变换
  2. 最后结果再由$A^{-1}$转为原有基向量

## 2. 运动学

### 2.1 李群视角下的运动学

物体的运动由旋转R和平移t组成，平移是简单的时间函数，所以主要研究旋转。

- 泊松方程(Poisson Formula)的推导:

  1. 对R的正交性质$\mathbf{R}^T\mathbf{R}=\mathbf{I}$求导可得:
     $$
     \frac{d}{dt}(\mathbf{R}^T\mathbf{R})=\mathbf{\dot{R}}^T\mathbf{R}+\mathbf{R}^T\mathbf{\dot{R}}=0
     $$

  2. 移项后可发现$\mathbf{R}^T\mathbf{\dot{R}}$是一个反对称矩阵:
     $$
     \mathbf{R}^T\mathbf{\dot{R}}=-(\mathbf{R}^T\mathbf{\dot{R}})^T
     $$
     这里用到了转置的性质：$(AB)^T=B^TA^T$

  3. 最后将反对称矩阵写成:$\omega^{\wedge}=\mathbf{R}^T\mathbf{\dot{R}}$即可得到泊松方程：
     $$
     \mathbf{\dot{R}}=\mathbf{R}\mathbf{\omega}^{\wedge} \tag{2.1}
     $$
     这里的$\omega$的物理意义是瞬时角速度

- 解泊松方程后可得**处理角速度**的2.2, 2.3, 2.4式：

  给定初值$t_0$时刻的旋转矩阵为$\mathbf{R}(t_0)$，那么微分方程2.1式的解为：
  $$
  \mathbf{R}(t)=\mathbf{R}(t_0)exp(\omega^{\wedge}(t-t_0))\tag{2.2}
  $$
  记$\Delta t=t-t_0$则可得到2.2式的离散时间下的格式：
  $$
  \mathbf{R}(t)=\mathbf{R}(t_0)exp(\omega\Delta t) \tag{2.3}
  $$
  对2.3式的指数映射一阶泰勒近似后可得：
  $$
  \mathbf{R}(t_0+\Delta t)\approx\mathbf{R}(t_0)(\mathbf{I}+\omega^{\wedge}\Delta t)\tag{2.4}
  $$
  

### 2.2 四元数视角下的运动学

### 2.3 四元数的李代数与旋转向量间的转换

### 2.4 so(3), se(3)上的运动学

### 2.5 线速度与加速度

### 2.6 扰动模型与雅可比矩阵

## 3. 案例：运动学(圆周运动)

## 4. 滤波器与最优化理论