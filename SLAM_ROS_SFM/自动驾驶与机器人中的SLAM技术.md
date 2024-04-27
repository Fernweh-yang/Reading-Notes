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
  
  4. 2.2.7式就和2.1.1式一样是一个微分方程，解得 ：
     $$
     q(t)=q(t_0)exp(\varpi\Delta t)\tag{2.2.8}
     $$
  

### 2.3 四元数的李代数与旋转向量间的转换

14讲中已知旋转矩阵$R$的李代数是旋转向量$\phi$

- 通过和旋转矩阵$R$的比较，可以得到四元数的李代数:纯虚四元数$ \varpi=[0,\omega]^T\in\mathcal{Q}$ 和  旋转向量$\phi$ 之间的关系：
  $$
  \varpi=[0,\frac{1}{2}\phi]^T, 即 \omega=\frac{1}{2}\phi\tag{2.2.9}
  $$
  可以发现四元数表达式的角速度: $\omega$ 正好是$R$的李代数: $\phi$​ 的一半。这与四元数在旋转一个向量时要乘两遍相对应。

- 旋转向量和它的李代数（旋转向量$\phi$）之间的关系：

  注意这里$\phi$式旋转向量，所以R是右乘

  2.1.1式$\omega$是瞬时角速度，所以R是左乘
  $$
  \dot{R}=\phi^{\wedge}R\tag{2.2.10}
  $$
  指数映射为:
  $$
  R=Exp(\phi)=exp(\phi^{\wedge})\tag{2.2.11}
  $$
  
- 四元数和旋转向量的李代数之间的关系：

  由2.2.9已知四元数李代数和旋转向量李代数之间的关系，所以得到：
  $$
  \dot{q}=\frac{1}{2}q[0,\phi]^T=q[0,\omega]^T=q\varpi \tag{2.2.12}
  $$
  指数映射为:
  $$
  q=Exp(\varpi)=exp([0,\frac{1}{2}\phi]^T)\tag{2.2.13}
  $$

- 

### 2.4 加上平移的李群运动学

旋转：即2.1.1式，$\omega$是三维瞬时角速度
$$
\mathbf{\dot{R}}=\mathbf{R}\mathbf{\omega}^{\wedge}
$$
平移：
$$
\dot{t}=v
$$

> 四元数的运动学,即2.2.7式: $\dot{q}=q\varpi$

### 2.5 线速度与加速度

研究**同一**坐标点在只带旋转关系的两个坐标系之间的线速度和加速度的变换关系。

1. 考虑p点在两坐标系下的坐标为$p_1,p_2$，他们的关系为
   $$
   p_1=R_{12}p_2 \tag{2.5.1}
   $$

2. 对2.5.1求时间导数
   $$
   \begin{aligned}
   \dot{p}_1& =\dot{R}_{12}p_{2}+R_{12}\dot{p}_{2} \\
   &=R_{12}\omega^{\wedge}p_{2}+R_{12}\dot{p}_{2} \\
   &=R_{12}\left(\omega^{\wedge}p_{2}+\dot{p}_{2}\right)
   \end{aligned} \tag{2.5.2}
   $$

3. $\dot{p}$代换为速度v:
   $$
   v_{1}=R_{12}\left(\omega^{\wedge}p_{2}+v_{2}\right) \tag{2.5.3}
   $$

4. 继续对2.5.3q求时间导数：
   $$
   \begin{aligned}
   \dot{v}_1& =\dot{R}_{12}\left(\omega^{\wedge}p_{2}+v_{2}\right)+R_{12}\left(\dot{\omega}^{\wedge}p_{2}+\omega^{\wedge}\dot{p}_{2}+\dot{v}_{2}\right) \\
   &=R_{12}\left(\omega^{\wedge}\omega^{\wedge}p_{2}+\omega^{\wedge}v_{2}+\dot{\omega}^{\wedge}p_{2}+\omega^{\wedge}\dot{p}_{2}+\dot{v}_{2}\right) \\
   &=R_{12}(\dot{v}_2+2\omega^{\wedge}v_2+\dot{w}^{\wedge}p_2+\omega^{\wedge}\omega^{\wedge}p_2)
   \end{aligned}\tag{2.5.4}
   $$

5. $\dot{v}$代换为加速度$a$:
   $$
   a_{1}=R_{12}\left(\underbrace{a_{2}}_{\text{加速度}}+\underbrace{2\omega^{\wedge}v_{2}}_{\text{科氏加速度}}+\underbrace{\dot{\omega}^{\wedge}p_{2}}_{\text{角加速度}}+\underbrace{\omega^{\wedge}\omega^{\wedge}p_{2}}_{\text{向心加速度}}\right)\tag{2.5.5}
   $$

### 2.6 扰动模型与雅可比矩阵

对李群求导，可以定义在向量层面即通过李代数，也可以通过BCH公式得到的扰动模型，扰动模型会更简洁明了。

下面看如何求导

- 案例1：对向量$a$进行旋转

  这个旋转可以是旋转矩阵$Ra$，也可以是四元数$qaq^*$，最后得到的结果是一致的：
  $$
  \frac{\part Ra}{\part \phi}=\frac{\part qaq^*}{\part \omega}=-Ra^{\wedge}\tag{2.6.1}
  $$

- 案例2：旋转的复合

  考虑$Log(R_1R_2)$对$R_1$求导的结果，对$R_1$进行右扰动可计算得到
  $$
  \begin{aligned}
  \frac{\partial Log\left(R_{1}R_{2}\right)}{\partial R_{1}}& =\lim_{\phi\to0}\frac{Log\left(R_{1}Exp\left(\phi\right)R_{2}\right)-Log\left(R_{1}R_{2}\right)}{\phi} \\
  &=\lim_{\phi\to0}\frac{Log\left(R_{1}R_{2}Exp\left(R_{2}^{\top}\phi\right)\right)-Log\left(R_{1}R_{2}\right)}{\phi} \\
  &=J_{r}^{-1}\left(Log\left(R_{1}R_{2}\right)\right)R_{2}^{T}
  \end{aligned} \tag{2.6.2}
  $$

  > 之所以有Log是因为矩阵无法对向量求导，所以统一转为向量

  如果对$R_2$求导，右扰动计算可得
  $$
  \begin{aligned}
  \frac{\partial Log\left(R_{1}R_{2}\right)}{\partial R_{2}}
  &=\lim_{\phi\to0}\frac{Log\left(R_{1}R_{2}Exp\left(\phi\right)\right)-Log\left(R_{1}R_{2}\right)}{\phi} \\
  &=J_{r}^{-1}\left(Log\left(R_{1}R_{2}\right)\right)
  \end{aligned} \tag{2.6.3}
  $$
  

## 3. 案例：运动学(圆周运动)

## 4. 滤波器与最优化理论

### 4.1 状态估计问题

SLAM问题、定位问题或者建图问题都可以概括为状态估计问题。状态估计由一组运动方程和一组观测方程组成，假设观测方程$h$和运动方程$f$为线性的，那么就可以得到线性高斯系统：
$$
\begin{cases}
\mathbf{x}_k=\mathbf{A}_k\mathbf{x}_{k-1}+\mathbf{u}_k+\mathbf{w}_k \\
\mathbf{z}_k = \mathbf{C}_k\mathbf{x}_k+\mathbf{v}_k \\
\end{cases}\ \ \ k=1,\cdots,N\tag{4.1.1}
$$
它的噪声服从零均值高斯分布：
$$
\mathbf{w}_k \sim N(\mathbf{0},\mathbf{R}),\ \mathbf{v}_k\sim N(\mathbf{0},\mathbf{Q}) \tag{4.1.2}
$$
$\mathbf{A}_k$：状态转移矩阵

$\mathbf{u}_k$：控制向量

$\mathbf{C}_k$：观测矩阵，将状态空间映射到观测空间

$\mathbf{R}$：过程噪声的协方差矩阵 

$\mathbf{Q}$：测量误差的协方差矩阵

### 4.2 卡尔曼滤波器

- 下面使用卡尔曼滤波器将k-1时刻的状态分布推导至k时刻，最终得到线性系统的最优无偏估计

  - 由于基于马尔可夫性假设，所以在实际编程中，我们只需要维护一个状态变量
  - 而又由于状态变量服从高斯分布，我们只需要维护状态变量的均值矩阵$\hat{\mathbf{x}}_k$和协方差矩阵$\hat{\mathbf{P}}_k$

- 卡尔曼滤波由**5个方程+2个阶段**组成，并用$\hat{a}$表示后验，$\check{a}$表示先验分布：

  先验即预测值，后验即上一轮计算得到的值。

  1. **预测**(prediction)

     从上一时刻的状态，根据输入信息（有噪声）推断当前时刻的状态分布。4.1.3式预测状态估计, 4.1.4式预测先验状态误差协方差
     $$
     \begin{align}
     \check{\mathbf{x}}_k=\mathbf{A}_k\hat{\mathbf{x}}_{k-1}+\mathbf{u}_k \tag{4.1.3}\\
     \check{\mathbf{P}}_k=\mathbf{A}_k\hat{\mathbf{P}}_{k-1}\mathbf{A}_k^T+\mathbf{R}_k \tag{4.1.4}
     \end{align}
     $$

     > 状态$\mathbf{x}_k$举例来说：
     >
     > 现在对小车一维直线行进距离p和速度v的估计：
     >
     > 可将其观察状态:
     > $$
     > \mathbf{x}_k=\left [\begin{array}{cccc}
     > p_k\\
     > v_k \\
     > \end{array}\right]=\left [\begin{array}{cccc}
     > p_{k-1}+v_{k-1}\Delta t+\frac{1}{2}a_k\Delta t^2\\
     > v_{k-1}+a_k\Delta t \\
     > \end{array}\right]
     > $$
     > 写成状态方程, 6.1式：
     > $$
     > \mathbf{x}_k=\left [\begin{array}{cccc}
     > p_k\\
     > v_k \\
     > \end{array}\right]=\left [\begin{array}{cccc}
     > 1 & \Delta t\\
     > 0 & 1 \\
     > \end{array}\right]\left [\begin{array}{cccc}
     > p_{k-1}\\
     > v_{k-1} \\
     > \end{array}\right]+\left [\begin{array}{cccc}
     > \frac{1}{2}\Delta t^2 & 0\\
     > \Delta t & 0 \\
     > \end{array}\right]\left [\begin{array}{cccc}
     > a_k\\
     > 0 \\
     > \end{array}\right]+\mathbf{w}_k
     > $$

     > 先验状态协方差$\check{\mathbf{P}}_k$是什么？
     >
     > 是真实状态$\mathbf{x}_k$和状态估计$\check{\mathbf{x}}_k$之间的先验误差协方差
     > $$
     > \check{\mathbf{P}}_k=E[(\mathbf{x}_k-\check{\mathbf{x}}_k)(\mathbf{x}_k-\check{\mathbf{x}}_k)^T]
     > $$

  2. **更新**（correctiion/measurment update）

     比较在当前时刻的状态输入（也叫**测量**值）和**预测**的状态变量， 从而对预测出的系统状态进行修正.

     - 先计算卡尔曼增益K：
       $$
       \mathbf{K}_k=\check{\mathbf{P}}_k\mathbf{C}_k^T(\mathbf{C}_k\check{\mathbf{P}}_k\mathbf{C}_k^T+\mathbf{Q}_k)^{-1}\tag{4.1.5}
       $$

     - 然后计算后验概率分布, 8.1式更新状态，8.2式更新后验误差协方差
       $$
       \begin{align}
       \hat{\mathbf{x}}_k=\check{\mathbf{x}}_k+\mathbf{K}_k(\mathbf{z}_k-\mathbf{C}_k\check{\mathbf{x}}_k)\tag{4.1.6}\\
       \hat{\mathbf{P}}_k=(\mathbf{I}-\mathbf{K}_k\mathbf{C}_k)\check{\mathbf{P}}_k \tag{4.1.7}
       \end{align}
       $$

- 5个方程的物理意义

  - 4.1.3式：状态方程，通过它直接可以根据前一个状态估计一下当前状态。

  - 4.1.4式：先验误差协方差方程，通过它可以看到由状态方程预测的状态和真实状态之间的方差（不确定性，可靠性）有多少

  - 4.1.5式：约束优化条件，使后验误差协方差$\hat{\mathbf{P}}_k$最小，得到卡尔曼增益K

  - 4.1.6式：状态更新，得到**我们想要的最佳状态估计**

    通过该式可以发现，Kalman滤波就是一个加权平均的过程：

    对态估计和测量估计的加权平均：哪个估计的不确定性低(更可靠)哪个的权重就大，这个权重在8.1式的体现就是卡尔曼增益K。

  - 4.1.7式：后验误差协方差方程$\hat{\mathbf{P}}_k$，在先验误差协方差方程$\check{\mathbf{P}}_k$的基础上加入测量过程$\mathbf{C}_k$的影响，更新下一时刻的先验值$\check{\mathbf{P}}_{k+1}$

### 4.3 扩展卡尔曼滤波器

在SLAM中运动/观测方程通常都是非线性的，而卡尔曼滤波KF只能用于线性系统，对于非线性系统需要使用先在某点附近对运动/观测方程进行一阶泰勒展开，保留一阶项以近似成线性发成，最后使用扩展卡尔曼滤波EKF进行无偏最优估计

- 扩展卡尔曼滤波EKF同样由2部分组成

  1. **预测**：
     $$
     \check{\mathbf{x}}_k=f(\hat{\mathbf{x}}_{k-1},\mathbf{u}_k)\\
     \check{\mathbf{P}}_k=\mathbf{F}_k\hat{\mathbf{P}}_{k-1}\mathbf{F}_k^T+\mathbf{R}_k \tag{4.1.8}
     $$

  2. **更新**：

     - 先计算卡尔曼增益$K_k$
       $$
       \mathbf{K}_k=\check{\mathbf{P}}_k\mathbf{H}_k^T(\mathbf{H}_k\check{\mathbf{P}}_k\mathbf{H}_k^T+\mathbf{Q}_k)^{-1}\tag{4.1.9}
       $$

     - 然后计算后验概率分布：
       $$
       \hat{\mathbf{x}}_k=\check{\mathbf{x}}_k+\mathbf{K}_k(\mathbf{z}_k-h(\check{\mathbf{x}}_k))\\
       \hat{\mathbf{P}}_k=(\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)\check{\mathbf{P}}_k \tag{4.1.10}
       $$

### 4.4 最优化与滤波器的关系

- 从批量最小二乘法(Batch Least Square)的视角来看，运动方程和观测方程都可以看成一个状态变量x与运动学输入、观测值之间的残差：
  $$
  \begin{align}
  e_{motion}&=x_k-f(x_{k-1},u)\sim\mathcal{N}(0,R_k)\tag{4.1.11}\\
  e_{obs}&=z_k-h(x_k)\sim\mathcal{N}(0,Q_k)\tag{4.1.12}
  \end{align}
  $$
  而滤波器中的最优状态估计可以看成关于各误差项的最小二乘问题：
  $$
  x^{*}=arg\mathop{min}_x\sum_k(e_k^T\Omega_k^{-1}e_k)\tag{4.1.13}
  $$

  - $e_k$：第k项误差
  - $\Omega$：该误差的协方差矩阵

- 最优化方法和滤波器的关联

  他们再线性系统中会得到同样的结果，但再非线性系统中由于以下原因会不同：

  1. 最优化有迭代过程，而ekf没有。
  2. 迭代过程会不断在新的线性化点$x_i$上求取雅可比矩阵，而EKF的雅可比矩阵只在预测位置上求取一次。
  3. EKF还会区分先验变量和后验变量，分开处理预测过程和观测过程。而最优化方法则是统一处理各处的状态变量。

  忽略第三点原因，将卡尔曼滤波看作非线性优化的化会有2个优化变量$x_{k-1}$和$x_k$和3种误差函数：

  1. 先验误差：

     k-1时刻的状态$x_{k-1}$服从它的的先验高斯分布。假设$x_{k-1}\sim\mathcal{N}(\hat{x}_{k-1},P_{k-1})$，则先验误差为：
     $$
     e_{prior}=x_{k-1}-\hat{x}_{k-1}\sim\mathcal{N}(0,P_{k-1})\tag{4.1.14}
     $$

  2. 从k-1到k的运动误差，即4.1.11式
  3. k时刻的观测误差，即4.1.12式

# 三、惯性导航与组合导航

探讨惯性测量单元(IMU, inertial motion unit)的测量模型、噪声模型。和卫星定位。

## 1. IMU系统的运动学

> IMU通常由陀螺仪(Gyroscope)和加速度计(Accelerator)组成。我们可以通过IMU测量运动载体的惯性来推断物体本身的状态。
>
> 陀螺仪可以测量物体的角速度。
>
> 加速度计可以测量物体的加速度。

IMU的测量值$\widetilde{a},\widetilde{\omega}$就是车辆在车坐标系下的加速度和车自身的角速度：
$$
\widetilde{a}=R^T(a-g)\\
\widetilde{\omega}=\omega
$$
这里的$R^T=R^{-1}$加上下标是$R_{bw}$，将世界坐标系下的物理量转换到车坐标系下；g是重力加速度

## 2. 用IMU进行轨迹推算

## 3. 卫星导航

## 4. 使用误差状态卡尔曼滤波器实现组合导航

## 5. 案例：实现ESKF的组合导航