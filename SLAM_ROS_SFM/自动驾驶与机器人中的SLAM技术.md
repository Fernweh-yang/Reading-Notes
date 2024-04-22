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

### 2.0 旋转的描述

- **四元数描述旋转**：

  - 一个空间旋转可以用一个四元数表示，该四元数$\mathbf{q}=q_0+q_1i+q_2j+q_3k=[s,\mathbf{v}]^T$由标量s 和 向量$\mathbf{v}$组成。

  - 用一个虚四元数来描述三维空间点：
    $$
    \mathbf{p}=[0,x,y,z]^T=[0,\mathbf{v}]^T\tag{2.0.1}
    $$
    相当于把四元数的3个虚部和空间中的3个轴相对应。

  - 那么旋转后的空间点为：
    $$
    \mathbf{p'}=\mathbf{qpq}^{-1}\tag{2.0.2}
    $$
    此处乘法均为四元数乘法，结果也是四元数。最后$\mathbf{p}'$的虚部即旋转后的坐标

- 四元数的乘法：

  表达形式1：向量形式
  $$
  q_{a}q_{b}=\left[s_{a}s_{b}-v_{a}^{\top}v_{b},s_{a}v_{b}+s_{b}v_{a}+v_{a}\times v_{b}\right]^{\top}\tag{2.0.3}
  $$
  表达形式2：矩阵形式

  - 首先定义两个运算符号$+,\oplus$：
    $$
    \left.q^{+}=\left[\begin{matrix}s&-v^{\top}\\v&sI+v^{\wedge}\end{matrix}\right.\right],q^{\oplus}=\left[\begin{matrix}s&-v^{\top}\\v&sI-v^{\wedge}\end{matrix}\right]\tag{2.0.4}
    $$
    这两个运算符号将四元数映射成为一个4x4的矩阵。

  - 矩阵形式的乘法公式为：
    $$
    q_{1}q_{2}=q_{2}^{\oplus}q_{1}=q_{1}^{+}q_{2}=\begin{bmatrix}s_{1}&-v_{1}^{\top}\\v_{1}&s_{1}I+v_{1}^{\wedge}\end{bmatrix}\begin{bmatrix}s_{2}\\v_{2}\end{bmatrix}=\begin{bmatrix}-v_{1}^{\top}v_{2}+s_{1}s_{2}\\s_{1}v_{2}+s_{2}v_{1}+v_{1}^{\wedge}v_{2}\end{bmatrix}\tag{2.0.5}
    $$

- 四元数到旋转矩阵的转换：
  $$
  R=vv^\top+s^2I+2sv^\wedge+\left(v^\wedge\right)^2.\tag{2.0.6}
  $$

- 四元数到旋转向量的转换
  $$
  \begin{cases}
  \theta=2\arccos s\\
  [n_{x},n_{y},n_{z}]^{\top}=v^{\top}/\sin\frac{\theta}{2}
  \end{cases}\tag{2.0.7}
  $$
  

### 2.1 李群视角下的运动学

物体的运动由旋转R和平移t组成，平移是简单的时间函数，所以主要研究旋转。

- **运动学公式**：泊松方程(Poisson Formula)的推导

  1. 对R的正交性质$\mathbf{R}^T\mathbf{R}=\mathbf{I}$求导可得:
     $$
     \frac{d}{dt}(\mathbf{R}^T\mathbf{R})=\mathbf{\dot{R}}^T\mathbf{R}+\mathbf{R}^T\mathbf{\dot{R}}=0
     $$

  2. 移项后可发现$\mathbf{R}^T\mathbf{\dot{R}}$是一个反对称矩阵:
     $$
     \mathbf{R}^T\mathbf{\dot{R}}=-(\mathbf{R}^T\mathbf{\dot{R}})^T
     $$
     这里用到了转置的性质：$(AB)^T=B^TA^T$

  3. 最后将反对称矩阵写成:$\omega^{\wedge}=\mathbf{R}^T\mathbf{\dot{R}}$即可得到**泊松方程**：
     $$
     \mathbf{\dot{R}}=\mathbf{R}\mathbf{\omega}^{\wedge} \tag{2.1.1}
     $$
     这里的$\omega$的物理意义是瞬时角速度

- 解泊松方程后可得**处理角速度**的2.2, 2.3, 2.4式：

  给定初值$t_0$时刻的旋转矩阵为$\mathbf{R}(t_0)$，那么微分方程2.1式的解为：
  $$
  \mathbf{R}(t)=\mathbf{R}(t_0)exp(\omega^{\wedge}(t-t_0))\tag{2.1.2}
  $$
  记$\Delta t=t-t_0$则可得到2.2式的离散时间下的格式：
  $$
  \mathbf{R}(t)=\mathbf{R}(t_0)exp(\omega\Delta t) \tag{2.1.3}
  $$
  对2.3式的指数映射一阶泰勒近似后可得：
  $$
  \mathbf{R}(t_0+\Delta t)\approx\mathbf{R}(t_0)(\mathbf{I}+\omega^{\wedge}\Delta t)\tag{2.1.4}
  $$

​	可以发现：2.3式是2.2式的在离散时间下的形式；2.4式又是2.3式的线性近似形式

### 2.2 四元数视角下的运动学

- 预备知识：纯虚四元数的指数映射

  1. 对于一个纯虚四元数$ \varpi=[0,\omega]^T\in\mathcal{Q}$，它的指数映射定义为：
     $$
     exp(\varpi)=\sum_{k=0}^\infin\frac{1}{k!}\varpi^k\tag{2.2.1}
     $$

  2. 将纯虚四元数分离：$\varpi=\theta u$。$\theta$为四元数的长度，$u$式纯虚单位四元数。

     纯虚单位四元数有性质：$u^2=-1,u^3=-u$，代入2.5式可得：
     $$
     \begin{align}
     exp(\theta u)&=1+\theta u-\frac{1}{2!}\theta^2-\frac{1}{3!}\theta^3u+\frac{1}{4}\theta^4+\cdots\\
     &=\underbrace{\left(1-\frac{1}{2!}\theta^{2}+\frac{1}{4!}\theta^{4}-\ldots\right)}_{\cos\theta}+\underbrace{\left(\theta-\frac{1}{3!}\theta^{3}+\frac{1}{5!}\theta^{5}-\ldots\right)}_{\sin\theta}u \\
     &=\cos\theta+u\sin\theta\tag{2.2.2}
     \end{align}
     $$
     2.6式和欧拉公式非常相似：$exp(i\theta)=cos\theta+isin\theta$

  3. 代入纯虚四元数$\varpi$可得：
     $$
     exp(\varpi)=[cos\theta,usin\theta]^T\tag{2.2.3}
     $$
     因为$\varpi$是纯虚四元数，所以模为1：
     $$
     ||exp(\varpi)||=cos^2\theta+sin^2\theta||u||^2=1\tag{2.2.4}
     $$
     由2.8式可知：

     - 一个纯虚四元数的指数映射结果为单位四元数
     - 可以把纯虚四元数$\varpi$看作四元数形式的李代数

- 四元数下的**运动学公式**推导

  1. 从四元数的单位性约束$\mathbf{qq^*=q^*q=1}$出发，对时间求导可得：
     $$
     \dot{q}^*q+q^*\dot{q}=0\tag{2.2.5}
     $$

  2. 由此可知$q^*\dot{q}$是一个纯虚四元数(实部为0)$\varpi$:
     $$
     q^*\dot{q}=\varpi\tag{2.2.6}
     $$
  
  3. 两边同乘$q$：
     $$
     \dot{q}=q\varpi\tag{2.2.7}
     $$
  
  4. 2.2.7式就和2.1.1式一样是一个微分方程，解的：
     $$
     q(t)=q(t_0)exp(\varpi\Delta t)\tag{2.2.8}
     $$
  

### 2.3 四元数的李代数与旋转向量间的转换

### 2.4 so(3), se(3)上的运动学

### 2.5 线速度与加速度

### 2.6 扰动模型与雅可比矩阵

## 3. 案例：运动学(圆周运动)

## 4. 滤波器与最优化理论