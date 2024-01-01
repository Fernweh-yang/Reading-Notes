

[toc]

# 零、学习资料汇总

- 主体是：SLAM十四讲
- 辅助教材：
  - State Estimation for Robotics： 针对第四章：李群与李代数 和 第六章：非线性优化（状态估计）
- 因子图：
  - [gtsam官网](https://gtsam.org/tutorials/intro.html#LyXCite-Kschischang01it)

# 一、预备知识

配套代码：https://github.com/gaoxiang12/slambook2

课后答案参考：https://www.zhihu.com/column/c_1367551490568036352

## 1. Slam全名

- Simultaneous Localization and Mapping 同时定位与地图构建
  - 它是指搭载特定**传感器**的主体，在**没有环境先验信息**的情况下，于**运动过程中**建立**环境**的模型，同时估计自己的**运动**。

## 2. 一些自己的理解

- 线性方程Ax = b的求解，对A和b有哪些要求？（从A的维度和秩角度分析）
  - 若rank(A) < rank(A|b)，则该线性方程为超定方程，无解。（但是可以有最小二乘解
  - 若rank(A) = rank(A|b)，则该线性方程组有解：在此基础上，若rank < A的列数（即变量的个数），则为欠定方程，有无穷解；若rank = A的列数，则为适定方程，具有唯一解。
- 前端，后端，建图各自的任务
  - 前端：
    - 视觉里程计：根据N对匹配好的特征点之间的重投影误差(特征点法)，或者根据光度误差找到灰度值匹配的像素点(光流法)，来求得当前帧相对于上一帧的变换T
    - 回环检测：利用图像间相似性来判断是否到达过先前的位置，以此来消除累积误差

  - 后端：
    - 后端优化：使用BA/位姿图来减少整个框架的误差
    - 建图：根据估计的轨迹，建立要求的地图




# 二、初识Slam

## 1. 传感器
### 1.1 传感器分类
- 为了让机器人可以探索一个房间，它需要知道：
  - 定位：我在什么地方
  - 建图：周围环境是什么样
- 传感器可以分为2类
  - 安装于环境中的：  
    比如导轨，二维码等
  - 携带于机器人本体上的：  
    比如轮式编码器、相机、激光传感器、惯性测量单元IMU等
### 1.2 不同的相机特点
- 单目相机Monocular:  
  - 用一个相机的slam称之为单目slam(Monocular Slam)。
  - 由于某个场景scene只在相机的成像平面上留下一个投影，所以天生缺失了一个维度的信息：深度/距离。
  - 为了得到三维结构，就需要让相机运动，然后根据不同地方看物体所形成的视差(disparity)来得到深度信息。
  - 缺点:尺度不确定性Scale Ambiguity
    - 通过平移得到的距离只是一个相对值，我们无法知道那些物体的真实尺寸。
    - 就像看电影，我们虽然能知道一个物体相较另一个物体是大是小，却无法知道他们的真实大小
- 双目相机Stereo：
  - 双目相机由两个单目相机组成，这两相机之间的距离(基线baseline)是已知的。然后根据这个基线来估计每个像素的空间位置
  - 基线距离越大，能够测量得到的物体就越远。
  - 缺点：复杂
    - 其深度量程和精度 受到基线和分辨率限制，所以无人车上双目相机通常很大
    - 视察的计算非常消耗计算资源，需要GPU和FPGA来加速才能实时输出整张图像的距离信息
- 深度相机RGB-D：
  - 可以通过红外结构光或者Time-of-Flight(ToF)原理，来像激光传感器一样，主动向物体发射光并接收返回的光，从而测量距离
  - 由于是根据物理手段测量距离，所以相比双目相机节约了大量计算资源。
  - 缺点：易受干扰
    - RGB-D相机存在测量范围窄、噪声大、视眼小、易受日光干扰、无法测量透射材质等问题。
    - 主要用于室内，室外很难应用。
## 2. SLAM基本框架
![框架](https://img2018.cnblogs.com/blog/1637062/201904/1637062-20190427225259959-530834575.png)

如图所示，主要包括5个步：

1. **传感器信息读取**： 视觉slam中主要为相机图像信息的读取和预处理。还可能有码盘、惯性传感器等。

2. **前端视觉里程计**：（VO: Visual Odometry），估算相邻图像间相机的运动，以及局部地图的样子。VO又称为前端Front End。

   - 视觉里程计：能够通过相邻帧间的图像估计相机运动，并恢复场景的空间结构。

     它和真的里程计一样，只计算相邻时刻的运动，而和过去的信息没有关联。

   - 累积飘逸(Accumulating Drift): 由于两图相间的运动是估算出来的，所以必定有误差，每一时刻的误差都会不断累积传递到下一时刻。由后端和回环消除飘逸。

   - 前端主要研究方向是CV：图像的特征提取与匹配。

3. **后端非线性优化**： 接收不同时刻视觉里程计测量的相机位姿，以及回环检测的信息，对他们进行优化，得到全局一致的轨迹和地图。因为接在VO后，所有称为后端Back End

   - 后端优化主要指处理SLAM过程中的噪声问题，从带有噪声的数据中估计整个系统的状态
   - 最大后验概率估计(Maximum-a-Posteriori)：状态估计的不确定性有多大。
   - 后端的主要研究是：滤波和非线性优化算法。

4. **回环检测**：（Loop Closure Detection）判断机器人是否到达过先前。的位置。如果检测到回环(到达过)，就会把信息提供给后端进行处理。

   - 主要解决位置估计随时间飘逸的问题。即通过某手段让机器人知道它回到原点了，但位置估计还没回到原点，这时将位置估计拉回原点，就消除飘逸了。

5. **建图**：（Mapping）根据估计的轨迹，建立与任务要求对应的地图。

   - 建图没有一个固定的形式和算法，2D栅格、2D拓扑、3D点云、3D网格都可以。

   - 大体可分为**度量地图**和**拓扑地图**：

     - 度量地图Metric Map: 精确表示地图中物体的位置关系，又有2类：

       1. **稀疏**Sparse:  进行一定程度抽象，不表达所有的物体。只选择一些有代表意义的东西作为路标Landmark,不是路标的部分可以忽略。

          定位 稀疏地图就够了。

       2. **稠密**Dense： 建模所有看到的东西。

          导航 需要稠密地图。

     - 拓扑地图Topological Map: 强调地图元素之间的关系。由节点和边组成，只考虑节点间的连通性，不考虑如何从A点到B点。

## 3. SLAM问题数学表述

- 问题：如何用数学描述“一机器人在未知环境中运动”？

  由下面两件事“运动”和“观测”描述。

- 运动：

  可由如下运动方程表示：$x_k=f(x_{k-1},u_k,w_k)$

  - $x_k$: k时刻机器人的位置
  - $u_k$: 运动传感器的读数或输入
  - $w_k$: 噪声
  - $f$: 运动方程

- 观测：

  可由如下观测方程表示：$z_{k,j}=h(y_j,x_k,v_{k,j})\ ,\ (k,j)\in O$

  - $y_j$:路标点
  - $z_{k,j}$:机器人在$x_k$上看到路标点$y_j$时，产生的观测数据
  - $v_{k,j}$：此时的噪声
  - $h$:观测方程
  - $O:$记录哪个时刻观察到什么路标

### 3.1 例子：

描述一个小车在平面中运动

- 运动方程：

  - 小车位姿可由两个轴上的坐标和转角表示：$\mathbf{x}_k=[x_1,x_2,\theta]_k^T$

  - 输入的指令是两个时间间隔之间的 位置和转角变化量：$\mathbf{u}_k=[\Delta x_1,\Delta x_2,\Delta \theta]_k^T$

  - 最终运动方程为：
    $$
    \begin{bmatrix}
       x_1 \\
       x_2 \\
       \theta 
      \end{bmatrix}_k=\begin{bmatrix}
       x_1 \\
       x_2 \\
       \theta 
      \end{bmatrix}_{k-1}+\begin{bmatrix}
       \Delta x_1 \\
       \Delta x_2 \\
       \Delta \theta 
      \end{bmatrix}_k+w_k
    $$

- 观察方程：

  - 通过激光雷达可以观测到2个数据：路标点与小车的 距离$r$和夹角$\phi$

  - 最终观察方程为：
    $$
    \begin{bmatrix}
       r_{k,j} \\
       \phi_{k,j} 
      \end{bmatrix}=\begin{bmatrix}
       \sqrt{(y_{1,j}-x_{1,k})^2+(y_{2,j}-x_{2,k})^2} \\
       arctan\big(\frac{y_{2,j}-x_{2,k}}{y_{1,j}-x_{1,k}}\big) 
      \end{bmatrix}+v
    $$

### 3.2 求解方法：

- 上面这两个方程将SLAM方程变成了一个**状态估计问题**： 如何通过带有噪声的测量数据u，估计内部隐藏着的状态变量x

- 状态估计问题可分为：

  - 线性/非线性系统
  - 高斯/非高斯系统

- 对于简单的线性高斯问题（Linear Gaussian,LG系统）：

  它的无偏最优估计可以由卡尔曼滤波器(Kalman Filter)给出

- 对于复杂的非线性非高斯系统(Non-Linear Non-Gaussian，NLNG系统)：

  有两种方法：

  1. 扩展卡尔曼滤波(Extended Kalman Filter,EKF)
     - 21世纪早期使用的多
     
  2. 非线性优化
     - 现在主流SLAM算法使用图优化(Graph Optimization)为主。
     
# 三、三维空间刚体运动

一个坐标系到另一个坐标系的转换，相差一个欧氏变换（Euclidean Transform）,而欧氏变换又由旋转和平移组成

## 1. 旋转矩阵

Rotation Matrix

- 旋转矩阵属于$SO(n)$特殊正交群(Special Orthogonal Group)
  $$
  SO(n)=\{ R\in\mathbb{R}^{n\times n}|RR^T=I,det(R)=1 \}
  $$

- 某向量$a$在两坐标系下的坐标分别为$a_1,a_2$,他们之间的关系是：
  $$
  a_1=R_{12}a_2+t_{12}
  $$

  - $R_{12}$:坐标系2到坐标系1的旋转。由于向量写在矩阵右侧，所以是从右往左读。
  - $t_{12}$:坐标系1下 坐标系1的原点到坐标系2的原点的向量。

- 由于旋转矩阵是正交矩阵，所以他的逆也是转置描述了一个相反的旋转
  $$
  a=Ra' \\
  a'=R^{-1}a=R^{T}a
  $$
  

## 2. 变换矩阵

Transform Matrix

- 变换矩阵属于$SE(3)$特殊欧氏群(Special Euclidean Group)
  $$
  SE(3)=\{T\in\mathbb{R}^{4\times 4} | R\in SO(3),t\in \mathbb{R}^3 \}
  $$

- 将旋转和平移写入一个矩阵就是变换矩阵
  $$
  \left [\begin{array}{cccc}
  a'\\
  1 \\
  \end{array}\right] = \left [\begin{array}{cccc}
  R & t\\
  0^T & 1 \\
  \end{array}\right]\left [\begin{array}{cccc}
  a \\
  1 \\
  \end{array}\right]=T\left [\begin{array}{cccc}
  a \\
  1 \\
  \end{array}\right]
  $$

- T的逆同样表示一个相反的变换：
  $$
  T^{-1}=\left [\begin{array}{cccc}
  R^T & -R^Tt \\
  0^T & 1 \\
  \end{array}\right]
  $$
  

## 3. 旋转向量

Rotation Vector

- 用旋转矩阵描述旋转有如下两个缺点：

  - SO(3)的旋转矩阵有9个量，但一次旋转只有3个自由度。同样变换矩阵用16个量来表达6个自由度。这种表达方式是冗余redundant的
  - 旋转矩阵自带约束，必须是正交矩阵且行列式为1。这使得求解变得更困难

- 任何一个旋转都可以用一个**旋转轴**和一个**旋转角**来表达：

  - 旋转向量：也叫轴角(Axis-Angle)，方向和旋转轴一致，长度等于旋转角
  - 对于变换矩阵：就可以用一个旋转向量 + 一个平移向量来表示，这样变量数正好是6维

- 罗德里格斯公式(Rodrigues's Formula)：描述旋转向量到旋转矩阵的转换过程
  $$
  \mathbf{R}=cos\theta \mathbf{I}+(1-cos\theta)\mathbf{nn}^T+sin\theta\mathbf{n}^{\wedge}
  $$

  - $\wedge$:向量到反对称矩阵的转换符

  - $\theta$:转角

    可对罗德里格斯公式两边取迹得到：
    $$
    \theta=arccos\frac{tr(\mathbf{R})-1}{2}
    $$

  - $\mathbf{n}$:转轴（向量）

    因为旋转轴上的向量在旋转后不发生改变，所以转轴n也就是旋转矩阵R特征值1对应的特征向量：
    $$
    \mathbf{Rn=n}
    $$

## 4. 欧拉角

Euler Angles

- 无论旋转矩阵还是旋转向量对人类来说都不是很直观，欧拉角用3个分离的转角来把一个旋转分解成3次绕不同轴的旋转。

  - 欧拉角根据先绕什么轴转有各种分类如xyz,zyz,zyx等。
  - 航空中通常用偏航-俯仰-滚转(yaw-pitch-roll)来描述旋转，它等价于ZYX的旋转顺序
    - yaw偏航角：绕Z轴旋转
    - pitch俯仰角：绕旋转之后的Y轴旋转
    - roll滚转角：绕旋转之后的X轴旋转

- 但欧拉角有一个万向锁问题(Gimbal Lock)，也称之为奇异性Singularity

  即如果俯仰角(第二个旋转的角)为+-90°时，第一次旋转和第三次旋转的是同一个轴，这就使得系统丢失了一个自由度

## 5. 四元数

Quaternion

### 5.1 基本概念

- 想用三维向量描述旋转必定带有奇异性，不论是欧拉角还是旋转向量

  这点类似于地球坐标，用经度纬度两个坐标描述，则会在维度+-90°时，经度失去意义，产生奇异性。

- 四元数可以类比于复数的概念，复数的乘法表示复平面上的旋转，比如乘上复数i相当于逆时针把一个复向量旋转90°。

  - 二维：旋转用单位复数来描述

    将复平面的向量逆时针旋转$\theta$角：
    $$
    e^{i\theta}=cos\theta+isin\theta
    $$

  - 三维：旋转用单位四元数来描述
    $$
    \mathbf{q}=q_0+q_1i+q_2j+q_3k
    $$

    - $q_0$：四元数的实部值，代表逆时针旋转角度的余弦值，$q_0=cos\frac{\theta}{2}$

    - $q_1,q_2,q_3$：四元数的虚部值，代表旋转轴的方向，$[q_1\ q_2\ q_3]=[sin\frac{\theta}{2}e_x\ \  sin\frac{\theta}{2}e_y\ \ sin\frac{\theta}{2}e_z]$

    - i,j,k: 四元数的3个虚部，有如下关系：
      $$
      \begin{cases}
      i^2=j^2=k^2=-1\\
      ij=k,ji=-k \\
      jk=i,kj=-i\\
      ki=j,ik=-j
      \end{cases}
      $$

- 如果把i,j,k看作三个坐标轴，如此乘法和外积都和复数一样运算了：
  $$
  \mathbf{q}=[s,\mathbf{v}]^T,\ \ s=q_0\in\mathbb{R},\ \ \mathbf{v}=[q_1,q_2,q_3]^T\in\mathbb{R}^3
  $$

  - 实四元数：如果虚部v为0
  - 虚四元数：如果实部s为0

- 四元数与复数的不同之处：

  - 复数中：乘以i意味着逆时针旋转90°
  - 四元数中：乘以i意味着绕着i轴逆时针旋转了180°

### 5.2 四元数表示旋转

一个三维点$p$ 旋转之后变成$p'$

- 用矩阵描述：$p'=Rp$
- 用四元数描述：
  - 首先把三维点用虚四元数的形式描述：$\mathbf{p}=[0,x,y,z]^T=[0,\mathbf{v}]^T$
  - 经过四元数描述的旋转q:$p'=qpq^{-1}$
    - 这里是四元数乘法，四元数计算规则见书。
  - 最后把$p'$中的虚部取出，就是旋转之后点的坐标

## 6. 四种变换

四种变换是从上到下包含的关系，欧氏变换是要求最高的，所以欧氏变换后不仅保持了体积还具有平行和相交的性质。

### 6.1 欧氏变换

即变换矩阵：
$$
T=\left [\begin{array}{cccc}
R & t \\
0^T & 1  \\

\end{array}\right]
$$

- 自由度：6
- 不变性质：长度，夹角，体积
- 一个刚体经过欧氏变换后，只是进行了旋转和移动，形状不变

### 6.2 相似变换

相似变换相比欧氏变换多了一个自由度，允许物体进行均匀缩放，体现在旋转部分多了一个缩放因子s：
$$
T_s=\left [\begin{array}{cccc}
sR & t \\
0^T & 1  \\

\end{array}\right]
$$

- 自由度：7
- 不变性质：体积比
- 一个变成为1的立方体经过一个相似变换后，不仅旋转移动了，也可能就变成了边长为10的立方体

### 6.3 仿射变换

相比欧氏变换只要求旋转部分A是一个可逆矩阵，而不必是正交矩阵：
$$
T_A=\left [\begin{array}{cccc}
A & t \\
0^T & 1  \\
\end{array}\right]
$$

- 自由度：12
- 不变性质：平行性、体积比
- 一个立方体经过仿射变换后，不仅旋转移动了，它也不再是方的了，只是各个面等然保持平行四边形

### 6.4 射影变换

射影变换是最一般的变换，相比仿射变换左下角还多了个缩放$a^T$，所有从真实世界到相机照片的变换可以堪称一个射影变换：
$$
T_P=\left [\begin{array}{cccc}
A & t \\
a^T & v  \\
\end{array}\right]
$$

- 自由度：15（3D），8（2D）。

- 不变性质：接触平面的相交和相切

- 一个原本方形的地板，在照片中由于近大远小的关系，甚至不是平行四边形，而是一个不规则的四边形。

  

# 四、李群与李代数

## 0. 为什么需要李群-李代数转换

- 旋转矩阵、旋转向量、欧拉角、四元数都是描述三维世界中刚体运动的方式。
- SLAM问题中由于位姿是位置的，就需要解决“**什么样的相机位姿最符合当前观测的数据**”，也就是我们需要估计R，t来描述相机运动。然后转为**优化问题**，求解最优的R和t使得误差最小。
- 但是旋转矩阵R自带约束(正交且行列式为1)，这样他们作为优化变量时，就会引入额外的约束，使优化变得更困难。
- 为了解决这个问题，就用到李群（正交且行列式为1）-》李代数的转换，将位姿估计变成无约束的优化问题。



## 1. 李群

- 群：由一个集合以及一个二元运算所组成

  - 群运算满足 1.封闭性(closure), 2. 结合律(associativity), 3. 存在幺元(identity), 4. 存在逆(invertibility)，这4个性质与光滑性准则一起，将 SO(3) 和 SE(3) 构建为**矩阵李群**(matrix Lie group)。
  
- 李群（Lie group）也**是**一个**微分流形**（differential manifold），所以群上的运算是**光滑smooth**和**连续的continuous**的
  
  - 这种光滑的结构使得李群可以使用微积分
  - 矩阵李群中的元素都是矩阵形式，把它们组合起来的运算是矩阵乘法，而逆运算是矩阵求逆。
  - 李群对加法不封闭：$R_1+R_2\notin SO(3),\ T_1+T_2\notin SE(3)$
    - 所以SO(3)和SE(3)都无法构成一个向量空间（向量空间的子空间仍然是一个向量空间）。
    - 尽管 SO(3) 和 SE(3) 都不是向量空间，但它们都是矩阵李群
  - 李群对乘法封闭：$R_1R_2\in SO(3),\ T_1T_2\in SE(3)$
    - 这是有物理意义的，矩阵的乘法意味着做了2次旋转，加法就变成了1次新的旋转的了
  
- 李群**不是**欧几里得空间Euclidean spaces
  
  - 欧几里得空间：一种向量空间，具有特定的维度，各维度之间遵守欧几里得距离。特点是平坦性flatness并满足几何特性，比如平行的线永不相交。
  - 李群：是比欧几里得更一般也更复杂的。李群可以有弯曲的或非平面的几何形状，它们的局部性质可能与欧几里德空间的局部性质有很大不同。
  - 然而，值得注意的是， 一些李群，例如n维实数的加法群$R^n$或正实数的乘法群$R^+$，可以嵌入到欧氏空间中。 这意味着我们可以将这些李群看作欧氏空间的子集，但李群结构本身并不等同于一个完整的欧氏空间。
  
- 微分流形manifold可以局部被认为是欧几里得空间，但不能就此说它全局上是一个欧几里得空间
  
  比如一个球，是一个curved manifold弯曲的流形。
  
  - 我们可以取任意一点，该点的邻域可以平滑的映射到欧几里得空间的相应区域。
  
  - 但整个球体不能无失真地全局嵌入到欧几里得空间中。（毛球定理）

- 特殊正交群SO(3)
  $$
  SO(3)=\{R\in \mathbb{R}^{3\times 3} |RR^T=I,det(R)=1\} \tag{1.1}
  $$

  - $RR^T=1$给具有九个参数的旋转矩阵引入了六项约束，把旋转矩阵的自由度降到了三

  - det(C)=1 保证我们获得一个有效旋转(proper rotation),det(C)=-1时C称为瑕旋转(improper rotation)或者反射旋转(rotary reflection)。

- 特殊欧氏群SE(3)
  $$
  SE(3)=\big\{ T=\left [\begin{array}{cccc}
  R & t \\
  0^T & 1  \\
  
  \end{array}\right]\in\mathbb{R}^{4\times4}|R\in SO(3),t\in\mathbb{R}^3 \big\} \tag{1.2}
  $$


## 2. 李代数

- 每个李群都有与之对应的李代数(Lie algebra)。
  
  - 不同于李群，李代数由数域$\mathbb{F}$上张成的向量空间$\mathbb{V}$ 和一个二元运算符$[ , ]$构成，所以李代数构成一个向量空间
    - 李代数的向量空间是一个**切空间**(tangent space),这个切空间与对应李群上的幺元相关联，它完全地刻画了这个李群的局部性质。
  
      - 幺元：$R,1\in SO(3)\Rightarrow R1=1R=R$
  
  - 如果集合、数域和二元运算满足以下条件，则称之为一个李代数$\mathfrak{g}=(\mathbb{V},\mathbb{F},[,])$
    - 封闭性：$\forall \mathbf{X,Y}\in\mathbb{V},[\mathbf{X,Y}]\in\mathbb{V}$
    - 双线性:$\forall \mathbf{X,Y,Z}\in\mathbb{V},a,b\in\mathbb{F},有$：
      - $[a\mathbf{X}+b\mathbf{Y},\mathbf{Z}]=a[\mathbf{X,Z}]+b[\mathbf{Y,Z}]$
      - $[\mathbf{Z},a\mathbf{X}+b\mathbf{Y}]=a[\mathbf{Z,X}]+b[\mathbf{Z,Y}]$
    - 自反性:$\forall \mathbf{X}\in\mathbb{V},[\mathbf{X,X}]=0$
    - 雅可比等价:$\forall \mathbf{X,Y,Z}\in\mathbb{V},[\mathbf{[X,[Y,Z]]}+\mathbf{[Z,[X,Y]]}+\mathbf{[Y,[Z,X]]}=0]$
  
  - 例子：三维向量的叉积 X, 就是一种李代数，可以写作：$\mathfrak{g}=(\mathbb{R}^3,\mathbb{R},\times)$
  
- 下面的$\phi$是怎么来的：

  1. 任意旋转矩阵代表相继旋转，会随时间变换，所以有：$\mathbf{R}(t)\mathbf{R}(t)^T=\mathbf{I}$

  2. 在等式两边对时间求导：$\mathbf{\dot{R}}(t)\mathbf{R}(t)^T+\mathbf{R}(t)\mathbf{\dot{R}}(t)^T=0$

  3. 可以看出$\mathbf{\dot{R}}(t)\mathbf{R}(t)^T$是反对称矩阵，由此我们可以找到一个三维向量$\phi(t)\in\mathbb{R}^3$与之对应：$\mathbf{\dot{R}}(t)\mathbf{R}(t)^T=\phi(t)^{\wedge}$

  4. 两边等式同乘$\mathbf{R}(t)$有：
     $$
     \mathbf{\dot{R}}(t)=\phi(t)^{\wedge}\mathbf{R}(t)=\left [\begin{array}{cccc}
     0 & -\phi_3 & \phi_2 \\
     \phi_3 & 0 & -\phi_1  \\
     -\phi_2 & \phi_1 & 0 \\
     \end{array}\right]\mathbf{R}(t) \tag{2.0}
     $$

     - 每对旋转矩阵求一次导数，只需左乘一个$\phi^{\wedge}(t)$
     - 所以$\phi$反应了旋转矩阵 $\mathbf{R}$的导数性质，也就是R的李代数/R的切空间。
     - 结论：李代数是李群的切空间，由李群求导得到，表达了李群的局部性质。

### 2.1 旋转的李代数(指数/对数映射)

- 旋转$\mathbf{R}$：李代数$\mathfrak{so} (3)$

  - 李群$SO(3)$对应的李代数$\phi$是定义在$\mathbb{R}^3$的向量
    $$
    \mathfrak{so}(3)=\{\phi\in \mathbb{R}^3,\mathbf{\phi}=\phi^{\wedge}\in\mathbb{R}^{3\times3} \}
    $$

    - $\phi^{\wedge}$:是将向量变成反对称矩阵的意思

  - 		指数映射：李代数->李群：
    $$
    \mathbf{R}=exp(\phi^\wedge)=\sum_{n=0}^{\infty}\frac{1}{n!}(\phi^{\wedge})^n \tag{2.1.1}
    $$

    $$
    \mathbf{T}=exp(\xi^{\wedge})=\sum_{n=0}^{\infty}\frac{1}{n!}(\xi^{\wedge})^n \tag{2.1.2}
    $$

    - 指数映射是一个**满射**(surjective-only),这说明每一个$SO(3)$中的元素都可以对应多个$\mathfrak{so}(3)$中的元素

  - 		对数映射：李群->李代数
    $$
    \phi=ln(\mathbf{R})^{\vee} \tag{2.1.3}
    $$

  - 虽然可以用泰勒展开的方式去求解2.1.1式和2.1.2式，但会非常的复杂和不太可能。

    - 对于指数映射，2.1.1式泰勒展开后经过一系列化简，可以得到旋转矩阵$\mathbf{R}$的公式：
      $$
      \mathbf{R}=exp(\phi^{\wedge})=exp(\theta\mathbf{a}^{\wedge})=cos\theta\mathbf{I}+(1-cos\theta)\mathbf{aa}^T+sin\theta\mathbf{a}^{\wedge} \tag{2.1.4}
      $$

      - $\phi=\theta\mathbf{a}$: 
  
        因为$\phi$是三维向量，所以可以用轴角表示，单位长度的旋转轴$a$和旋转的角度$\theta$。
  
        $\theta=\theta+2k\pi$所以每一个$R$有无数个对应的李代数，给角度施加限制就唯一了。
  
      - 这表明李代数$\mathfrak{so} (3)$实际上就是由所谓的**旋转向量**组成的空间
  
    - 对于对数映射，2.1.4式两边取迹(对角元素之和)后，可以得到旋转角度的公式：
      $$
      \theta=arccos\frac{tr(\mathbf{R})-1}{2} \\
      \mathbf{Ra=a}\tag{2.1.5}
      $$

      - 这个旋转轴$a$还可以用公式$a=\frac{(R-R^T)^{\vee}}{2sin(\theta)}$来求，此时$\phi=ln(R)^{\vee}=\theta a$

        因为$R-R^T$就是反对称矩阵，将其转为向量后，除上$2sin(\theta)$，就是去找在绕哪个轴转

  - 一个二维例子：
  
    ![](https://cdn.jsdelivr.net/gh/Fernweh-yang/ImageHosting@main/img/%E6%9D%8E%E7%BE%A4%E6%9D%8E%E4%BB%A3%E6%95%B0%E4%BE%8B%E5%AD%90.png)
  
    - 		上图说明了当旋转被约束在平面上时，李群和李代数之间的关系。
    - 		在零旋转点的附近，即$\theta_{vi}=0$，李代数的向量空间就是旋转圆的切线。
      - 		因为(1,0)点经过旋转矩阵C变换后是$(1,\theta)$，一条线上，所以是切线。
    - 		在旋转接近零时，李代数反映了李群的局部结构信息。
    - 		这一个例子是约束在平面上的（只有一个轴的旋转自由度），但通常 李代数的向量空间的维度为三。换言之，图中的直线，是整个三维李代数向量空间中的一维子空间。

### 2.2 位姿的李代数(指数/对数映射)

- 位姿$\mathbf{T}$：李代数$\mathfrak{se}(3)$

  - 李群$SE(3)$对应的李代数$\xi$位于$\mathbb{R}^6$空间
    $$
    \mathfrak{se}(3)=\big\{ \xi=\left [\begin{array}{cccc}
    \rho \\
    \phi  \\
    
    \end{array}\right]\in\mathbb{R}^{6},\rho\in\mathbb{R}^3,\phi\in\mathfrak{so}(3),\\ \xi^{\wedge}=\left [\begin{array}{cccc}
    \phi^{\wedge}& \rho \\
    o^T& 0  \\
    
    \end{array}\right]\in \mathbb{R}^{4\times4} \big\}
    $$

    - $\xi^{\wedge}$的$\wedge$不是反对称矩阵的意思，但也是向量转矩阵，算是对$\phi的(\cdot)^\wedge$的重载，可以将$\mathbb{R}^{6}$中的元素转换为$\mathbb{R}^{4\times4}$中的元素，并保持线性性质。
    - $\rho$: 沿着3轴的位移
    - $\phi$: 绕着3轴的旋转
    
  - 指数映射：李代数->李群
    $$
    \mathbf{T}=exp(\xi^{\wedge})=\sum_{n=0}^{\infty}\frac{1}{n!}(\xi^{\wedge})^n \tag{2.2.1}
    $$
    
    - 指数映射是一个**满射**(surjective-only),这说明每一个$T\in SE(3)$中的元素都可以由$\mathfrak{se}(3)$中的不同$\xi \in \mathbb{R}^6$产生
    
    - 图例：
    
      ![](https://cdn.jsdelivr.net/gh/Fernweh-yang/ImageHosting@main/img/%E4%BD%8D%E5%A7%BF%E6%9D%8E%E4%BB%A3%E6%95%B0%E6%AF%8F%E4%B8%80%E4%B8%AA%E9%87%8F%E7%9A%84%E5%90%AB%E4%B9%89.png)
    
      - 改变$\xi$中的每一个分量，构建$T=exp(\xi^\wedge)$，然后用它作用于长方体的角点上，我们可以看到角点的位姿可以 被平移或旋转。把这些基本移动结合起来，可以生成任意位姿变换。
    
  - 对数映射：李群->李代数
    $$
    \xi=ln(\mathbf{T})^{\vee} \tag{2.2.2}
    $$

  - 同样泰勒展开来计算2.2.1和2.2.2式非常复杂，不现实。

    - 对于指数映射，泰勒展开后化简，可以得到位姿T的计算公式：
      $$
      \mathbf{T}=exp(\xi^{\wedge})=\left [\begin{array}{cccc}
      \mathbf{R}& \mathbf{r}\\
      o^T& 1  \\
      \end{array}\right]=\left [\begin{array}{cccc}
      exp(\phi^{\wedge})& \mathbf{J}\rho \\
      o^T& 1  \\
      \end{array}\right]\\ \tag{2.2.3}
      $$
      
      其中$J$为雅可比矩阵：
      $$
      \begin{align}
      \mathbf{J}&=\sum_{n=0}^{\infty}\frac{1}{(n+1)!}(\phi^{\wedge})^n\\&=\frac{sin\theta}{\theta}\mathbf{I}+(1-\frac{sin\theta}{\theta})\mathbf{aa}^T+\frac{1-cos\theta}{\theta}\mathbf{a}^{\wedge} 
      \end{align}
      \tag{2.2.4}
      $$
      
      
      - 雅可比矩阵在将$\mathfrak{se}(3)$中的平移分量转换为$SE(3)$中的平移分量的过程中起到了非常重要的作用，即$\mathbf{r}=\mathbf{J\rho}$
      
      - 除了用雅可比矩阵，也可直接级数展开2.2.1式，来求$\mathbf{T}$:
        $$
        \mathbf{T}=exp(\xi^{\wedge})=\mathbf{I}+\xi^{\wedge}+(\frac{1-cos\theta}{\theta^2})(\xi^{\wedge})^2+(\frac{\theta-sin\theta}{\theta^3})(\xi^{\wedge})^2 \tag{2.2.5}
        $$
        
      
    - 对于对数映射，还是用迹的性质来计算：
      $$
      \theta=arccos\frac{tr(\mathbf{R})-1}{2}\\
      \mathbf{Ra}=\mathbf{a}\\
      \mathbf{r}=\mathbf{J\rho} \tag{2.2.6}
      $$
    

### 2.3 雅可比矩阵

- $\mathbf{JJ^T}$和它的逆：
  $$
  \begin{align}
  \mathbf{JJ^T}&=\gamma\mathbf{I}+(1-\gamma)\mathbf{aa}^T\\
  \mathbf{(JJ^T)^-1}&=\frac{1}{\gamma}\mathbf{I}+(1-\frac{1}{\gamma})\mathbf{aa}^T\\
  其中: \gamma&=2\frac{1-cos\theta}{\theta^2}
  \end{align}\tag{2.3.1}
  $$

  - $\mathbf{JJ^T}$是正定的

- $\mathbf{J}$和$\mathbf{R}$的关系：
  $$
  \mathbf{R}=\mathbf{I}+\phi^{\wedge}\mathbf{J} \tag{2.3.2}
  $$

  - 但由于$\phi^{\wedge}$不可逆，所以不能通过这种方式求出$\mathbf{J}$

### 2.4 李代数的伴随矩阵

#### 2.4.1 位姿李群的伴随矩阵

- 一个6x6的变换矩阵$\mathcal{T}$, 可以直接由一个4X4的变换矩阵构造而成。我们将这个6X6的矩阵称为$SE(3)$元素的伴随adjoint矩阵：
  $$
  \mathcal{T}=Ad(\mathbf{T})=Ad\big(\left [\begin{array}{cccc}
  \mathbf{R} & \mathbf{r}  \\
  \mathbf{0}^T & 1   \\
  \end{array}\right]\big)=\left [\begin{array}{cccc}
  \mathbf{R} & \mathbf{r}^\wedge\mathbf{R}  \\
  \mathbf{0} & \mathbf{R}   \\
  \end{array}\right]\tag{2.4.1}
  $$

- 将$SE(3)$中所有元素伴随矩阵的集合记为：
  $$
  Ad(SE(3))=\{\mathcal{T}=Ad(\mathbf{T})\ |\ \mathbf{T}\in SE(3)\} \tag{2.4.2}
  $$

  - $Ad(SE(3))$：同样是一个矩阵李群
  
- 伴随表示$Ad: G\rightarrow GL(g)$是一个从李群G到其李代数$\mathfrak{g}$的广义线性(general linear group)$GL(g)$的映射,并满足以下条件
  $$
  Ad_gX=L_gX(R_g)^{-1}
  $$

  - g表示李群$G$中的一个元素。

  - X表示李代数$\mathfrak{g}$中的一个元素

  - $L_g$表示左平移left translation

    左平移是指将李群中的元素作用于另一个李群元素的左边，得到一个新的李群元素

    - $L_g(h)=g*h$:*表示李群的乘法操作。左平移将李群元素h在李群中左边乘以李群元素g

  - $R_g$表示右平移right translation

    右平移是指将李群中的元素作用于另一个李群元素的右边，同样得到一个新的李群元素

    - $R_g(h) = h * g$: 右平移将李群元素h在李群中右边乘以李群元素g

  **通过伴随表示，可以将约束转化为李群的李代数上的线性操作**

- 李群元素：是指属于李群的一个成员，它代表了李群中的一个点。

  比如旋转矩阵的李群元素可以表示为一个3x3的矩阵R

  例子：

  ```
  T =
  [ R t ]
  [ 0 1 ]
  ```

#### 2.4.2 位姿李代数的伴随矩阵

同样是4x4 -> 6x6

- 对于$\xi^{\wedge}\in \mathfrak{se}(3)$的伴随矩阵是：
  $$
  ad(\xi^{\wedge})=\xi^{\curlywedge}\tag{2.4.3}
  $$
  其中：
  $$
  \xi^{\curlywedge}=\left [\begin{array}{cccc}
  \mathbf{\rho}   \\
  \mathbf{\phi}   \\
  \end{array}\right]^{\curlywedge}=\left [\begin{array}{cccc}
  \phi^{\wedge} & \rho^{\wedge}  \\
  \mathbf{0} & \phi^{\wedge}   \\
  \end{array}\right]\in\mathbb{R}^{6\times6},\ \ \ \rho,\phi\in\mathbb{R}^3 \tag{2.4.4}
  $$

  - $ad(\xi^{\wedge})$是一个向量空间。

- $ad(\mathfrak{se}(3))$是$Ad(SE(3))$的李代数

- 伴随表示：$ad:\mathfrak{g}\rightarrow gl(\mathfrak{g})$是一个从李代数$\mathfrak{g}$到其广义线性李代数(general linear Lie algebra)$gl(\mathfrak{g})$的线性映射。

  对于李代数$\mathfrak{g}$中的一个元素X，它在伴随表示下的映射$ad(X)$是一个线性变换，将$\mathfrak{g}$中的其他元素$Y$映射为一个新的元素$ad(X)(Y)$满足如下条件：
  $$
  ad(X)(Y)=[X,Y]
  $$

  - $[X, Y]$:表示李括号(Lie bracket)操作

  通过伴随表示，可以将李代数的元素映射到对应的Lie群元素的切空间上

- 李代数元素：

  so(3)的元素示例：

  ```
  X1 =
  [ 0 -z y ]
  [ z 0 -x ]
  [ -y x 0 ]
  
  X2 =
  [ 0 -w v ]
  [ w 0 -u ]
  [ -v u 0 ]
  ```

  

#### 2.4.3 伴随矩阵的指数/对数映射

- 指数映射：
  $$
  \begin{align}
  \mathcal{T}&=exp(\xi^{\curlywedge})=\sum_{n=0}^{\infty}\frac{1}{n!}(\xi^{\curlywedge})^n \tag{2.4.5}\\
  &=\sum_{n=0}^{\infty}\frac{1}{n!}\left [\begin{array}{cccc}
  \phi^{\wedge} & \rho^{\wedge}  \\
  \mathbf{0} & \phi^{\wedge}   \\
  \end{array}\right]^n\\
  &=\left [\begin{array}{cccc}
  \mathbf{R} & \mathbf{K}  \\
  \mathbf{0} & \mathbf{R}   \\
  \end{array}\right]\tag{2.4.6} \\
  \mathbf{K}&=(\mathbf{J\rho})^{\wedge}\mathbf{R} \tag{2.4.7}
  \end{align}
  $$

  - 和$T$的2.2.5式一样，2.4.5式直接级数展开
    $$
    \mathcal{T}=\mathbf{I}+(\frac{2sin(\theta)-\theta cos(\theta)}{2\theta})\xi^{\curlywedge}+(\frac{4-\theta sin(\theta)-4 cos(\theta)}{2\theta^2})(\xi^{\curlywedge})^2 + (\frac{sin(\theta)-\theta cos(\theta)}{2\theta^3})(\xi^{\curlywedge})^3+(\frac{2-\theta sin(\theta)-2 cos(\theta)}{2\theta^4})(\xi^{\curlywedge})^4 \tag{2.4.8}
    $$
    

- 对数映射：
  $$
  \xi=ln(\mathcal{T})^{{\curlyvee}} \tag{2.4.9}
  $$

- 不同的李群李代数之间都有可交换的关系：

  ![](https://cdn.jsdelivr.net/gh/Fernweh-yang/ImageHosting@main/img/%E4%B8%8D%E5%90%8C%E6%9D%8E%E7%BE%A4%E6%9D%8E%E4%BB%A3%E6%95%B0%E4%B9%8B%E9%97%B4%E7%9A%84%E5%85%B3%E7%B3%BB.png)



## 3. 李代数求导与扰动模型

### 3.1 BCH公式用于旋转

Baker-Campbell-Hausdorff公式为在李代数上做微积分提供了理论基础

- 利用BCH公式给出了两个旋转李代数指数映射乘积的近似表达式：
  $$
  ln(\mathbf{R}_1\mathbf{R}_2)^{\vee}=ln(exp(\phi_1^{\wedge})exp(\phi_2^{\wedge}))^{\vee}\approx\begin{cases}
  \mathbf{J_l}(\phi_2)^{-1}\phi_1+\phi_2 \ \ ，当\phi_1为小量 \\
  \phi_1 +\mathbf{J_r}(\phi_1)^{-1}\phi_2\ \ ，当\phi_2为小量 \\
  \end{cases} \tag{3.1}
  $$

  - 当对一个旋转矩阵$\mathbf{R}_2$(李代数为$\phi_2$)左乘一个微小旋转矩阵$\mathbf{R}_1$(李代数为$\phi_1$)时，可以近似的看作，在原有的李代数$\phi_2$上加上了一项$\mathbf{J_l}(\phi_2)^{-1}\phi_1$
  - 第二个近似描述了右乘一个微小位移的情况。
  - 12式中的$\mathbf{J}(\phi)$是一个整体
  - 雅可比矩阵的公式见2.3雅可比矩阵
  
- 由上可总结出：
  
  - 对某个旋转$R$(李代数为$\phi$)，给它左乘一个微小的旋转$\Delta\mathbf{R}$(李代数为$\Delta\phi$),那么在李群上结果为$\Delta \mathbf{R}\cdot \mathbf{R}$，在李代数上结果为$\mathbf{J}_l^{-1}(\phi)\Delta\phi+\phi$
  
    即：$exp(\Delta\phi^{\wedge})exp(\phi^{\wedge})=exp((\phi+\mathbf{J}_l^{-1}(\phi)\Delta\phi)^{\wedge})$
  
  - 反之，如果在李代数上进行加法，让一个$\phi$加上$\Delta \phi$,可近似为利群上带右雅可比的乘法

    即：$exp((\phi+\Delta \phi)^{\wedge})=exp((\mathbf{J}_l\Delta\phi)^{\wedge})exp(\phi^{\wedge})$=$exp(\phi^{\wedge})exp((\mathbf{J}_r\Delta\phi)^{\wedge})$

#### 3.1.1 旋转的左右雅可比矩阵

雅可比矩阵$\mathbf{J}$的计算由2.2.4式展开级数后得到：

- $\bf{J}_l$为左乘BCH近似雅可比式：
  $$
  \bf{J}_l(\phi)=\frac{sin\theta}{\theta}\bf{I}+(1-\frac{sin\theta}{\theta})\bf{aa}^T+\frac{1-cos{\theta}}{\theta}\bf{a}^{\wedge}\tag{3.1.1}
  $$

  - $\theta=|\mathbf{\phi}|$：旋转角度
  - $\mathbf{a}=\frac{\phi}{\theta}$：单位长度的旋转轴

  $\bf{J}_l$的逆为：
  $$
  \mathbf{J}_l(\phi)^{-1}=\frac{\theta}{2}cot\frac{\theta}{2}\mathbf{I}+(1-\frac{\theta}{2}cot\frac{\theta}{2})\mathbf{aa}^T-\frac{\theta}{2}\mathbf{a}^{\wedge} \tag{3.1.2}
  $$
  

  - 由于当$\theta=2k\pi$时，$cot(\frac{\theta}{2})$不存在，也就是说此时$\mathbf{J}$有奇异性，不存在逆矩阵。

- $\bf{J}_r$为右乘BCH近似雅可比式：
  $$
  \bf{J}_r(\phi)=\bf{J}_l(-\phi)=\frac{sin\theta}{\theta}\bf{I}+(1-\frac{sin\theta}{\theta})\bf{aa}^T-\frac{1-cos{\theta}}{\theta}\bf{a}^{\wedge}\tag{3.1.2}
  $$
  $\bf{J}_r$的逆为：
  $$
  \mathbf{J}_r(\phi)^{-1}=\frac{\theta}{2}cot\frac{\theta}{2}\mathbf{I}+(1-\frac{\theta}{2}cot\frac{\theta}{2})\mathbf{aa}^T+\frac{\theta}{2}\mathbf{a}^{\wedge} \tag{3.1.3}
  $$
  
- $\bf{J}_l$和$\bf{J}_r$之间的关系：
  $$
  \bf{J}_l(\phi)=\mathbf{R}\bf{J}_r(\phi) \\ 
  \bf{J}_r(\phi)=\bf{J}_l(-\phi)\tag{3.1.4}
  $$

- $\mathbf{JJ^T}$是正定的

### 3.2 BCH公式用于位姿

- 利用BCH公式给出了两个位姿李代数指数映射乘积的近似表达式：
  $$
  ln(\mathbf{T}_1\mathbf{T}_2)^{\vee}=ln(exp(\xi_1^{\wedge})exp(\xi_2^{\wedge}))^{\vee}\approx\begin{cases}
  \mathcal{J}_l(\xi_2)^{-1}\xi_1+\xi_2 \ \ ，当\xi_1为小量 \\
  \xi_1 +\mathcal{J}_r(\xi_1)^{-1}\xi_2\ \ ，当\xi_2为小量 \\
  \end{cases} \tag{3.2}
  $$

#### 3.2.1 位姿的左右雅可比矩阵

- SE(3)的左雅可比：
  $$
  \mathcal{J}_l(\xi)=\sum_{n=0}^{\infty}\frac{1}{(n+1)!}(-\xi^{\curlywedge})^n=\left [\begin{array}{cccc}
  \mathbf{J}_l & \mathbf{Q}_l  \\
  \mathbf{0} & \mathbf{J}_l   \\
  \end{array}\right] \tag{3.2.1}
  $$

  - 其中$\mathbf{Q}_l$:
    $$
    \begin{align}
    \mathbf{Q}_l(\xi)=\frac{1}{2}\rho^{\wedge}&+(\frac{\theta-sin\theta}{\theta^3})(\phi^{\wedge}\rho^{\wedge}+\rho^{\wedge}\phi^{\wedge}+\phi^{\wedge}\rho^{\wedge}\phi^{\wedge})\\
    &+(\frac{\theta^2+2cos\theta-2}{2\theta^4})(\phi^{\wedge}\phi^{\wedge}\rho^{\wedge}+\rho^{\wedge}\phi^{\wedge}\phi^{\wedge}-3\phi^{\wedge}\rho^{\wedge}\phi^{\wedge})\\&+(\frac{2\theta-3sin\theta+\theta cos\theta}{2\theta^5})(\phi^{\wedge}\rho^{\wedge}\phi^{\wedge}\phi^{\wedge}+\phi^{\wedge}\phi^{\wedge}\rho^{\wedge}\phi^{\wedge})
    \end{align}\tag{3.2.2}
    $$

  - 左雅可比的逆：
    $$
    \mathcal{J}_l^{-1}=\left [\begin{array}{cccc}
    \mathbf{J}_l^{-1} & -\mathbf{J}_l^{-1}\mathbf{Q}_l\mathbf{J}_l^{-1}  \\
    \mathbf{0} & \mathbf{J}_l^{-1}   \\
    \end{array}\right] \tag{3.2.3}
    $$
    

- SE(3)的右雅可比：
  $$
  \mathcal{J}_r(\xi)=\sum_{n=0}^{\infty}\frac{1}{(n+1)!}(-\xi^{\curlywedge})^n=\left [\begin{array}{cccc}
  \mathbf{J}_r & \mathbf{Q}_r  \\
  \mathbf{0} & \mathbf{J}_r   \\
  \end{array}\right] \tag{3.2.4}
  $$

  - 其中$\mathbf{Q}_r$:
    $$
    \mathbf{Q}_r(\xi)=\mathbf{Q}_l(-\xi)=R\mathbf{Q}_l(\xi)+(\mathbf{J}_l\rho)^{\wedge}\mathbf{R}\mathbf{J}_l \tag{3.2.5}
    $$

  - 右雅可比的逆：
    $$
    \mathcal{J}_r^{-1}=\left [\begin{array}{cccc}
    \mathbf{J}_r^{-1} & -\mathbf{J}_r^{-1}\mathbf{Q}_r\mathbf{J}_r^{-1}  \\
    \mathbf{0} & \mathbf{J}_r^{-1}   \\
    \end{array}\right] \tag{3.2.6}
    $$
    

- 左右雅可比之间的关系：
  $$
  \mathcal{J}_l(\xi)=\mathcal{T}\mathcal{J}_r(\xi)\\
  \mathcal{J}_l(-\xi)=\mathcal{J}_r(\xi) \tag{3.2.7}
  $$

- $\mathcal{JJ}^T$是正定的

### 3.3 SO(3)上李代数求导

#### 3.3.1 问题描述

- 已知一个世界坐标位于**p**的点，机器人对其产生的观测数据为**z**，机器人到该点的坐标转换为**T**,误差为**w**:$z=Tp+w$

- 因为有误差**w**存在,观测数据**z**不可能精准的等于Tp，从而产生误差$E=Z-Tp$。

  所以对机器人的**位姿估计**也就变成了在得到**N**个观测数据后寻求一个**最优的T**来让整体误差最小化：
  $$
  \mathop{min}_T\ J(\bf{T})=\sum_{i=1}^{N}||z_i-\bf{T}{p}_i||_2^2 \tag{3.3.1}
  $$

- 求解上述问题显然需要对姿态T求导，但T是李群没有定义的加法。因此要转为李代数求导。

  李代数求导又有2条思路：

  1. 用李代数表示姿态，然后根据李代数加法对**李代数求导**
  2. 对利群左乘或右乘微小扰动(BCH公式)，然后对这个**扰动求导**

#### 3.3.2 李代数求导

一个空间点p经过旋转R后得到$Rp$,求导最后得到：
$$
\frac{\partial\bf{Rp}}{\partial\bf{\phi}}=(-\bf{Rp}^{\wedge}\bf{J}_l) \tag{3.3.2}
$$

- 其中的左乘雅可比式$\bf{J}_l$很复杂，我们不希望计算它，所以使用扰动模型即BCH来计算导数

#### 3.3.3 扰动求导（左乘）

- 这种方式是对$\bf{R}$进行一次扰动$\Delta \bf{R}$,然后看结果相对于扰动的变化率。

  设$\Delta \bf{R}$对应的李代数为$\varphi$,然后对$\varphi$求导:
  $$
  \begin{align*}
  \frac{\partial(\bf{Rp})}{\partial\bf{\varphi}} &=\mathop{lim}_{\varphi \rightarrow 0}\frac{exp(\varphi^{\wedge})exp(\phi^{\wedge})\mathbf{p} -exp(\phi^{\wedge})\mathbf{p} }{\varphi}\\
  &= \mathop{lim}_{\varphi \rightarrow 0}\frac{(I+\varphi^{\wedge})exp(\phi^{\wedge})\mathbf{p} -exp(\phi^{\wedge})\mathbf{p} }{\varphi}\\
  &= \mathop{lim}_{\varphi \rightarrow 0}\frac{\varphi^{\wedge}\bf{Rp}}{\varphi} \\
  &= \mathop{lim}_{\varphi \rightarrow 0}\frac{-(\bf{Rp})^{\wedge}\ \varphi}{\varphi} \\
  &= -(\mathbf{Rp})^{\wedge} \tag{3.3.3}
  \end{align*}
  $$

- 可见相比3.3.2，使用扰动来计算导数就不用去求解雅可比J了。

### 3.4 SE(3)上李代数求导

一空间点$p$经过变换$T$(李代数为$\xi$)后得到$Tp$,p此时为齐次坐标,给T左乘一个扰动$\Delta\mathbf{T}= exp(\delta \mathbf{\xi}^{\wedge})$(李代数为$\delta\xi=[\delta\rho,\delta\phi]^T$)

- 对扰动的李代数$\delta\xi$求导得:
  $$
  \begin{align*}
    \frac{\partial(\mathbf{Tp})}{\partial\delta\xi} &= \mathop{lim}_{\delta\xi \rightarrow 0}\frac{exp(\delta\xi^{\wedge})exp(\xi^{\wedge})\mathbf{p} -exp(\xi^{\wedge})\mathbf{p} }{\delta\xi} \\
      &= \mathop{lim}_{\delta\xi \rightarrow 0}\frac{(\mathbf{I}+\delta\xi^{\wedge})exp(\xi^{\wedge})\mathbf{p} -exp(\xi^{\wedge})\mathbf{p} }{\delta\xi} \\
      &= \mathop{lim}_{\delta\xi \rightarrow 0}\frac{\delta\xi^{\wedge}exp(\xi^{\wedge})\mathbf{p} }{\delta\xi} \\
      &= \mathop{lim}_{\delta\xi \rightarrow 0}\frac{\left [\begin{array}{cccc}
  \delta\phi^{\wedge} & \delta\rho  \\
  \mathbf{0}^T & 0   \\
  \end{array}\right]\left [\begin{array}{cccc}
  \mathbf{Rp}+t  \\
  1 \\
  \end{array}\right]}{\left [\begin{array}{cccc}
  \delta\rho,\delta\phi  \\
  \end{array}\right]^T}\\
  &=\left [\begin{array}{cccc}
  \mathbf{I} & -(\mathbf{Rp+t})^{\wedge}  \\
  \mathbf{0}^T & \mathbf{0}^T \\
  \end{array}\right] \tag{3.4.1}
  \end{align*}
  $$
  



## 4. 李群的距离、体积与积分

### 4.1 对于旋转

- 定义$\mathfrak{so}(3)$的内积为：
  $$
  <\phi_1^{\wedge},\phi_2^{\wedge}>=\frac{1}{2}tr(\phi_1^{\wedge}{\phi_2^{\wedge}}^T)=\phi_1^T\phi_2 \tag{4.1.1}
  $$

- 定义两个旋转之间的差异有两种度量方式：
  $$
  \phi_{12}=ln(\mathbf{R}_1^T\mathbf{R}_2)^{\vee}\\
  \phi_{21}=ln(\mathbf{R}_2\mathbf{R}_1^T)^{\vee} \tag{4.1.2}
  $$

- 两个旋转的**距离**可以有两种方式定义：

  1. 两个旋转的差的内积的平方根
  2. 两个旋转的差的欧几里得范数

  $$
  右差:\phi_{12}=\sqrt{<ln(\mathbf{R}_1^T\mathbf{R}_2),ln(\mathbf{R}_1^T\mathbf{R}_2)>}=\sqrt{<\phi_{12}^{\wedge},\phi_{12}^{\wedge}>}=\sqrt{\phi_{12}^{T}\phi_{12}}=|\phi_{12}|\\
  左差:\phi_{21}=\sqrt{<ln(\mathbf{R}_2\mathbf{R}_1^T),ln(\mathbf{R}_2\mathbf{R}_1^T)>}=\sqrt{<\phi_{21}^{\wedge},\phi_{21}^{\wedge}>}=\sqrt{\phi_{21}^{T}\phi_{21}}=|\phi_{21}| \tag{4.1.3}
  $$

  这也可以看作是两旋转角度差异的大小。
  
  对旋转$\mathbf{R}=exp(\phi^{\wedge})\in SO(3)$施加一个微小扰动后，得到$\mathbf{R}'=exp((\phi+\delta\phi)^{\wedge})\in SO(3)$，他们之间的右差左差分别为：
  $$
  \begin{align}
  ln(\delta\mathbf{R}_r)^{\vee}&=ln(\mathbf{R}^T\mathbf{R'})^{\vee}=ln(\mathbf{R}^Texp((\phi+\delta\phi)^{\wedge}))^\vee\\
  &\approx ln(\mathbf{R}^T\mathbf{R}exp((\mathbf{J}_r\delta\phi)^{\wedge}))^\vee=\mathbf{J}_r\delta\phi\\
  ln(\delta\mathbf{R}_l)^{\vee}&=ln(\mathbf{R}'\mathbf{R}^T)^{\vee}=ln(exp((\phi+\delta\phi)^{\wedge})\mathbf{R}^T)^\vee\\
  &\approx ln(exp((\mathbf{J}_l\delta\phi)^{\wedge})\mathbf{R}\mathbf{R}^T)^\vee=\mathbf{J}_l\delta\phi
  \end{align} \tag{4.1.4}
  $$
  
- 求得$\mathbf{J}_r$和$\mathbf{J}_l$的列元素所构成的平行六面体的**体积**，即他们的行列是：
  $$
  det(\mathbf{J}_l)=det(\mathbf{RJ}_r)=\underbrace{det(\mathbf{R})}_1\ det(\mathbf{J}_r)=det(\mathbf{J}_r) \tag{4.1.5}
  $$

- 由上式得到旋转的无穷小量$d\mathbf{R}$:
  $$
  d\mathbf{R}=|det(\mathbf{J})|d\phi \tag{4.1.6}
  $$

  - 这表明无论我们使用右差或者左差，无穷小量都是相同的

- 最后得到旋转的**积分方程**：
  $$
  \displaystyle \int_{SO(3)}f(\mathbf{R})d\mathbf{R}\rightarrow\displaystyle \int_{|\phi|<\pi}f(\phi)|det(\mathbf{J})|d\phi \tag{4.1.7}
  $$

### 4.2 对于位姿

- 定义两个位姿$SE(3)$和伴随$Ad(SE(3))$之间的差异有两种度量方式：
  $$
  \xi_{12}=ln(\mathbf{T}_1^{-1}\mathbf{T}_2)^{\vee}=ln(\mathcal{T}_1^{-1}\mathcal{T}_2)^{\curlyvee}\\
  \xi_{21}=ln(\mathbf{T}_2\mathbf{T}_1^{-1})^{\vee}=ln(\mathcal{T}_2\mathcal{T}_1^{-1})^{\curlyvee} \tag{4.2.1}
  $$

- 定义$4\times4$和$6\times6$的内积为：
  $$
  <\xi_1^{\wedge},\xi_2^{\wedge}>=-tr(\xi_1^{\wedge}\left [\begin{array}{cccc}
  \frac{1}{2}\mathbf{I} & \mathbf{0}  \\
  \mathbf{0}^T & 1   \\
  \end{array}\right]{\xi_2^{\wedge}}^T)=\xi_1^T\xi_2\\
  <\xi_1^{\curlywedge},\xi_2^{\curlywedge}>=-tr(\xi_1^{\curlywedge}\left [\begin{array}{cccc}
  \frac{1}{4}\mathbf{I} & \mathbf{0}  \\
  \mathbf{0} & \frac{1}{2}\mathbf{I}   \\
  \end{array}\right]{\xi_2^{\curlywedge}}^T)=\xi_1^T\xi_2 \tag{4.2.2}
  $$

- 位姿的**距离**通用有右差和左差：
  $$
  右差:\xi_{12}=\sqrt{<\xi_{12}^{\wedge},\xi_{12}^{\wedge}>}=\sqrt{<\xi_{12}^{\curlywedge},\xi_{12}^{\curlywedge}>}=\sqrt{\xi_{12}^{T}\xi_{12}}=|\xi_{12}|\\
  左差:\xi_{21}=\sqrt{<\xi_{21}^{\wedge},\xi_{21}^{\wedge}>}=\sqrt{<\xi_{21}^{\curlywedge},\xi_{21}^{\curlywedge}>}=\sqrt{\xi_{21}^{T}\xi_{21}}=|\xi_{21}| \tag{4.2.3}
  $$

- 求得$\mathcal{J}_r$和$\mathcal{J}_l$的列元素所构成的平行六面体的**体积**，即他们的行列是：
  $$
  det(\mathcal{J}_l)=det(\mathcal{TJ}_r)=\underbrace{det(\mathcal{T})}_{=(det(R))^2=1}det(\mathcal{J}_r)=det(\mathcal{J}_r) \tag{4.2.4}
  $$

- 由上式得到旋转的无穷小量$d\mathbf{T}$:
  $$
  d\mathbf{T}=|det(\mathcal{J})|d\xi \tag{4.2.5}
  $$

- 利用这个无穷小量计算积分：
  $$
  \displaystyle \int_{SE(3)}f(\mathbf{T})d\mathbf{T}=\displaystyle \int_{\mathbb{R}^3,|\phi|<\pi}f(\xi)|det(\mathcal{J})|d\xi \tag{4.2.6}
  $$

## 5. 插值

有时候，我们需要在两个矩阵李群之间进行插值。然而常用的线性插值方法并不适用，因为它不满足结合律（所以插值的结果不再属于群）。经典的线性插值为：$x=(1-\alpha)x_1+\alpha x_2,\alpha \in [0,1]$

### 5.1 对于旋转

1. 旋转的插值为：
   $$
   \mathbf{R}=(\mathbf{R}_2\mathbf{R}_1^T)^{\alpha}\mathbf{R}_1,\ \ \ \alpha\in[0,1] \tag{5.1.1}
   $$
   

   - 当$\alpha=0$时，$\mathbf{R}=\mathbf{R}_1$, 当$\alpha=1$时，$\mathbf{R}=\mathbf{R}_2$

2. 认为旋转矩阵是一个时间相关的函数：
   $$
   \mathbf{R}(t)=(\mathbf{R}(t_2)\mathbf{R}(t_1)^T)^{\alpha}\mathbf{R}(t_1),\ \ \alpha=\frac{t-t_1}{t_2-t_1} \tag{5.1.2}
   $$
   并强制约束角速度为常量$\omega$:
   $$
   \omega=\frac{1}{t_2-t_1}\phi_{21}\\
   其中:\mathbf{R}_{21}=exp(\phi_{21}^{\wedge})=\mathbf{R}_2\mathbf{R}_1^T \tag{5.1.3}
   $$
   最后得到李群插值：
   $$
   \mathbf{R}(t)=exp((t-t_1)\omega^{\wedge})\mathbf{R}(t_1) \tag{5.1.4}
   $$
   李代数插值：
   $$
   \begin{align}
   \phi&=ln(\mathbf{R})^{\vee}=ln((\mathbf{R}_2\mathbf{R}_1^T)^{\alpha}\mathbf{R}_1)^{\vee}\\
   &=ln(exp(\alpha\phi_{21}^\wedge)exp(\phi_1^{\wedge}))^{\vee}\approx\alpha\mathbf{J}(\phi_1)^{-1}\phi_{21}+\phi_1
   \end{align} \tag{5.1.5}
   $$

### 5.2 对于位姿

类似于旋转定义位姿的插值方式为：
$$
\mathbf{T}=(\mathbf{T}_2\mathbf{T}_1^{-1})^{\alpha}\mathbf{T}_1,\ \ \ \alpha\in[0,1] \tag{5.2.1}
$$
李代数插值为：
$$
\begin{align}
\xi&=ln(\mathbf{T})^{\vee}=ln((\mathbf{T}_2\mathbf{T}_1^{-1})^{\alpha}\mathbf{T}_1)^{\vee}\\
&=ln(exp(\alpha\xi_{21}^\wedge)exp(\xi_1^{\wedge}))^{\vee}\approx\alpha\mathcal{J}(\xi_1)^{-1}\xi_{21}+\xi_1
\end{align} \tag{5.2.2}
$$

## 6. 齐次坐标点

- 任意$\mathbb{R}^3$的点都可以用$4\times1$的齐次坐标表达：
  $$
  \mathbf{P}=\left [\begin{array}{cccc}
  sx  \\
  sy  \\
  sz  \\
  s
  \end{array}\right]=\left [\begin{array}{cccc}
  \epsilon  \\
  \eta  
  \end{array}\right] \tag{6.1}
  $$

  - s为非零实数用来表示尺度
  - $\epsilon\in\mathbb{R}^3$, $\eta$是标量
  - 当s为0时，这个点就不能转回$\mathbb{R}^3$了，此时表示无穷远的点
  - 齐次坐标系可以被用来描述近距离和远距离的路标点，不会带来奇异性或尺度问题

- 两个操作符：
  $$
  \left [\begin{array}{cccc}
  \epsilon  \\
  \eta  
  \end{array}\right]^{\odot}=\left [\begin{array}{cccc}
  \eta \mathbf{I} & -\epsilon^{\wedge}  \\
  \mathbf{0}^T &  \mathbf{0}^T
  \end{array}\right] \ \in4\times6\\
  
  \left [\begin{array}{cccc}
  \epsilon  \\
  \eta  
  \end{array}\right]^{\circledcirc}=\left [\begin{array}{cccc}
  \mathbf{0} & \epsilon  \\
  -\epsilon ^\wedge &  \mathbf{0}
  \end{array}\right] \in 6\times4 \tag{6.2}
  $$
  
- 可以得到如下恒等式：
  $$
  \xi^{\wedge}\mathbf{p}=\mathbf{p}^{\odot}\xi\\
  \mathbf{p}^{T}\xi^{\wedge}=\xi^{T}\mathbf{p}^{{\circledcirc}} \tag{6.3}
  $$

  - $\xi\in\mathbb{R}^{6},\mathbf{p}\in\mathbb{R}^4$

## 7. 旋转的运动学

上面的6章都属于旋转的几何学，下面几何会随着时间改变，即运动学

### 7.1 李群

对旋转矩阵$\mathbf{R}=exp(\phi^{\wedge})$有李群的运动学方程（泊松方程）：
$$
\dot{\mathbf{R}}=\omega^{\wedge}\mathbf{R}\\
或\ \omega^{\wedge}=\dot{\mathbf{R}}\mathbf{R}^{T} \tag{7.1.1}
$$

- $\omega$是角速度。

- 由于基于旋转矩阵，所以这个运动学方程没有奇异性，但有正交矩阵的约束

### 7.2 李代数

对$\mathbf{R}=exp(\phi^{\wedge})$求导后，李代数下的运动学表达为：
$$
\omega=\mathbf{J}\dot{\phi}\\
或\dot{\phi}=\mathbf{J}^{-1}\omega \tag{7.2.1}
$$

- $\omega$是角速度。
- J是左雅可比矩阵$\bf{J}_l(\phi)=\frac{sin\theta}{\theta}\bf{I}+(1-\frac{sin\theta}{\theta})\bf{aa}^T+\frac{1-cos{\theta}}{\theta}\bf{a}^{\wedge}\tag{3.1.1}$
- 由于左雅可比矩阵在$\theta=2k\pi$出不存在逆，所以相比李群有奇异性，但没有正交约束

## 8. 位姿的运动学

### 8.1 李群

- 变换矩阵可以写成如下形式：
  $$
  \mathbf{T}=exp(\xi^{\wedge})=\left [\begin{array}{cccc}
  \mathbf{R}& \mathbf{r}\\
  o^T& 1  \\
  \end{array}\right]=\left [\begin{array}{cccc}
  exp(\phi^{\wedge})& \mathbf{J}\rho \\
  o^T& 1  \\
  \end{array}\right]\\ \tag{8.1.1}
  $$

  - 其中$\xi=\left [\begin{array}{cccc}
    \rho \\
    \phi  \\
    \end{array}\right]$

- 旋转和平移分开写的运动学方程为：
  $$
  \dot{\mathbf{r}}=\omega^{\wedge}\mathbf{r}+\mathcal{v}\\
  \dot{\mathbf{R}}=\omega^{\wedge}\mathbf{R}\tag{8.1.2}
  $$

  - $\mathcal{v}$和$\omega$分别为平移速度和旋转速度

- 于是变换矩阵对应的运动方程为：
  $$
  \dot{\mathbf{T}}=\varpi^{\wedge}\mathbf{T}\\
  或\ \varpi^{\wedge}=\dot{\mathbf{T}}\mathbf{T}^{-1} \tag{8.1.3}
  $$

  - 其中$\varpi=\left [\begin{array}{cccc}
    \mathcal{v} \\
    \omega  \\
    \end{array}\right]$​是广义速度(generalized velocity)
- 这些等式都是非奇异的，但都有正交约束

### 8.2 李代数

- 对$\mathbf{T}=exp(\xi^{\wedge})$求导后，可以得到李代数的运动学方程为：
  $$
  \varpi=\mathcal{J}\dot{\xi}\\
  或\ \dot{\xi}=\mathcal{J}^{-1}\varpi \tag{8.2.1}
  $$
  
- 此时不再受到正交约束

### 8.3 一种混合方法

通过组合$\dot{\mathbf{r}}$和$\dot{\phi}$有：
$$
\left [\begin{array}{cccc}
\dot{\mathbf{r}} \\
\dot{\phi} \\
\end{array}\right]=\left [\begin{array}{cccc}
\mathbf{I} & -\mathbf{r}^{\wedge}\\
\mathbf{0} & \mathbf{J}^{-1} \\
\end{array}\right]\left [\begin{array}{cccc}
v\\
\omega \\
\end{array}\right]
$$

- 这种方法仍然由于$\mathbf{J}^{-1}$的存在，具有奇异性，但是不再需要估计 Q，并且避免了积分之后，需要进行$r=\mathbf{J}\rho$这一转换
- 同时，该方法也不存在约束条件。

### 8.4 运动学

见书，用到时再记笔记



## 9. 旋转的线性化

关于李群和李代数的运动学方程，可以在它们的标称形式上施加扰动（即线性化）

见书，用到时再记笔记

## 10.位姿的线性化

见书，用到时再记笔记

## 11. 伴随性质

### 1. SO(3)上的伴随性质

$$
\mathbf{R}exp(\mathbf{p}^{\wedge})\mathbf{R}^T=exp((\mathbf{Rp})^{\wedge}) \tag{1}
$$

### 2. SE(3)上的伴随性质

$$
\mathbf{T}exp(\xi^{\wedge})\mathbf{T}^{-1}=exp((Ad(\mathbf{T})\xi)^{\wedge}) \tag{2}
$$

其中：
$$
Ad(T)= 
 
\left [\begin{array}{cccc}
\mathbf{R}  & \mathbf{t}^{\wedge}\mathbf{R}  \\
\mathbf{0}  & \mathbf{R}   \\
\end{array}\right]
$$


# 五、相机与图像

## 1. 相机模型

### 1.1 针孔相机模型

这部分内容具体见笔记：[[Comupter Vision#3.Perspektivische Projektion mit kalibrierter Kamera]]

### 1.2 畸变模型

为了获得好的成像效果，相机前方会放个透镜，但1.透镜形状会改变光线传播，2.是机械装配上透镜不能和成像平面完全平行，这就使得光线穿过透镜投影到成像面时的位置发生变化。

- **径向畸变**Radial distortion：由透镜形状引起

  透镜使得直线在成像中变成了曲线，越靠近图像边缘越明显。由于透镜加工是中心对称的，这使得不规则的畸变径向对称。有2大类：

  - 桶形畸变Barrel Distortion

    畸变图像放大率随着与光轴之间的距离增加而减小。

  - 枕形畸变Pincushion Distortion

    畸变图像放大率随着与光轴之间的距离增加而增加。

- **切向畸变**Tangential distortion：由透镜和成像面无法严格平行引起

- **矫正畸变**：通过5个畸变系数找到相机坐标系中点p在像素平面上的正确位置：

  1. 将三维空间点投影到归一化图像平面，归一化坐标为$[x,y]^T$

  2. 对归一化平面上的点计算径向畸变和切向畸变：
     $$
     \begin{cases}
     x_{distorted}=x(1+k_1r^2+k_2r^4+k_3r^6)+2p_1xy+p_2(r^2+2x^2) \\
     y_{distorted}=y(1+k_1r^2+k_2r^4+k_3r^6)+p_1(r^2+2y^2)+2p_2xy \\
     
     \end{cases}\tag{1}
     $$

     - 这里使用$k_1,k_2,k_3$来纠正径向畸变,$p_1,p_2$纠正切向畸变
     - 实际中可以灵活选择，比如只选$k_1,p_1,p_2$

  3. 将畸变后的点通过内参数矩阵投影到像素平面，得到改点的准确位置
     $$
     \begin{cases}
     u=f_xx_{distorted}+c_x \\
     v=f_yy_{distorted}+c_y \\
     \end{cases}
     $$

- 单目相机的成像过程：

  1. 某点的世界坐标为$\mathbf{P}_w$
  1. 相机的运动由$\mathbf{R,t}$描述。由此得到该点在相机坐标系下为$\mathbf{\widetilde{P}}_c=\mathbf{RP_w+t}$,坐标为$[X,Y,Z]$
  1. 归一化点的坐标：$\mathbf{P}_c=[\frac{X}{Z},\frac{Y}{Z},1]$
  1. 根据畸变公式，即上面的(1)式，计算点$\mathbf{P}_c$，发生畸变后的坐标。
  1. 由内参矩阵，得到它的像素坐标$\mathbf{P}_{uv}=\mathbf{KP}_c$
  
### 1.3 双目相机模型

通过视差计算深度

- 双目相机成像模型（对极几何）见[[Comupter Vision#三、Epipolargeometrie]]

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/slam/%E5%8F%8C%E7%9B%AE%E7%9B%B8%E6%9C%BA%E6%88%90%E5%83%8F%E6%A8%A1%E5%9E%8B.png?raw=true)

  - $O_L,O_R$为左右光圈中心
  - 方框为成像平面
  - f为焦距
  - $u_L,-u_R$为成像平面的坐标

- 根据上面$\Delta PP_LP_R$和$\Delta PO_LO_R$的相似关系有：
  $$
  \begin{align*}
  &\frac{z-f}{z}=\frac{b-u_L+u_R}{b}\\
  整理&后得：\\
  &z=\frac{fb}{d},
  \end{align*}
  $$

  - 其中$d=u_L-u_R$是**视差**，为左右图的横坐标之差。

    可知，视察越大，距离越近。

    但视差的计算是很困难的，需要使用gpu或fpga来实时计算。

  - 由于视差最小为一个像素，所以双目测量深度理论最大值由$fb$决定

    可知，基线越长，能测到的最大距离就越远。



### 1.4 RGB-D相机模型

如果知道相机的内外参，就可以计算任何一个像素在世界坐标系下的位置，从而建立一张点云地图。

- 能够主动测量每个像素的深度，有两大类：
  - 通过红外结构光(Structured Light)原理测量像素距离。比如Kinect1代，Intel RealSense。
    - 根据返回的结构光图案，计算物体与自身之间的距离。
  - 通过飞行时间(Time-of-Flight,ToF)原理测量像素距离，比如Kinect2代。
    - 和激光传感器类似，根据发送到返回之间的光束飞行时间来计算。
    - 但激光是通过逐点扫描获取距离，ToF相机则可以获得整个图像的每个像素点的深度。
- 由于使用发送-接收的测量方式，RGB-D相机容易受到日光或其他RGB-D发送的红外光干扰。



# 六、非线性优化

## 0.为什么要优化

由于噪音，再好的传感器也会带来误差，所以运动方程和观测方程的等式不可能精确成立。

与其假设数据完全符合方程，不如研究如何从有噪声的数据中进行准确的状态估计。

而解决状态估计就需要最优化的知识

## 1.状态估计

state estimation**根据可获取的量测数据估算动态系统内部状态的方法**

- 机器人的两大问题：**状态估计**state estimation和**控制**control

  - 机器人的**状态**：指一组完整描述它随时间运动的物理量，比如位置，角度和速度。
  - **状态估计**：根据系统的先验模型和测量序列，对系统内在状态进行重构的问题。

- 状态估计理论的起源和诞生

  - 高斯19世纪：提出**最小二乘法**，并证明在正态分布误差假设下，最小二乘解即最优估计
  - 卡尔曼20世纪：
    1. 引入**能观性**observability:即动态系统的状态何时能够从该系统一组观测值中推断出来
    2. 提出**卡尔曼滤波**Kalman filter:一种针对受高斯观测噪声影响下线性系统的状态估计方法

- 任何传感器精度有限，都有一定程度的不确定性，当我们想结合多种传感器的测量值来估计状态时，最重要的就是了解所有 的不确定量，从而推断我们对状态估计的置信度。所以在某种程度上，状态估计问题，就是在问，如何以最好的方式利用已有的传感器。


### 1.1 批量状态估计与最大后验估计 

最大后验估计：maximum a posteriori probability (MAP) estimate

- 由二、3.中可知SLAM模型由一个运动方程和一个观测方程构成

  - 运动方程表示：
    $$
    x_k=f(x_{k-1},u_k,w_k)\tag{1}
    $$
    由四、可知，$\mathbf{x}_k$是相机的位姿，用SE(3)来描述。

  - 观测方程表示：
    $$
    z_{k,j}=h(y_j,x_k,v_{k,j})\ ,\ (k,j)\in O \tag{2}
    $$
    - $z_{k,j}$:机器人在$x_k$上看到路标点$y_j$时，产生的观测数据
    - 这个观测数据的形式根据不同传感器而不同
      - 相机：通常使用特征点的像素坐标或特征描述子（如ORB、SIFT等）来表示观测数据
      - 激光：一组距离测量值
      - IMU：角速度和线性加速度
    
    
    由五、可知，观测方程即针孔相机模型：

    - 如假设相机在$\mathbf{x}_k$处对路标点$\mathbf{y}_j$进行了一次观测，路标点对应到图像上的像素坐标为$z_{k,j}$
      $$
      sz_{k,j}=\mathbf{K}(\mathbf{R}_k\mathbf{y}_j+\mathbf{t}_k)\tag{3}
      $$
    
      - $\mathbf{K}$:相机内参
    
      - s:为像素坐标系(像素点)到相机坐标系(光圈)的距离
    
        也是$\mathbf{R}_k\mathbf{y}_j+\mathbf{t}_k$的第三个分量
    
  - 另外假设两个噪声，他们满足零均值的高斯分布:

    $$
    \mathcal{w}_k\sim\mathcal{N}(\mathbf{0},\mathbf{R}_k)\tag{4}\\	
    \mathcal{v}_k\sim\mathcal{N}(\mathbf{0},\mathbf{Q}_{k,j}) 
    $$

    - $\mathcal{N}$:高斯分布
    - $\mathbf{0}$:零均值
    - $\mathbf{R}_k,\mathbf{Q}_{k,j}$：协方差矩阵,其中Q也被叫做信息矩阵

- 得到状态估计问题：

  通过带噪声的数据 观测方程中的像素坐标$\mathbf{z}$ 和 运动方程中传感器数据$\mathbf{u}$ 来推断 位姿$\mathbf{x}$和地图$\mathbf{y}$

- 求解状态估计的两种方法：

  1. **滤波器 (增量/渐进incremental的方法)**：由于数据是随时间逐渐到来的，所以我们持有一个当前时刻的状态估计，然后用新的数据来更新它。
     - 仅关心当前时刻的状态估计$x_k$，对之前的状态不考虑
  2. **批量batch方法**：把数据积攒起来一并处理
     - 相对于滤波器，可以在更大范围达到最优化，被认为优于传统的滤波器，而称为当今vslam的主流
     - 极端下可以让机器人收集所有时刻的数据后统一处理，即SFM(Structure from motion)三维重建的主流做法
     - 但极端情况显然不是实时的，不符合SLAM。所以在SLAM中，通常采用折中的办法：固定一些历史轨迹，仅对当前时刻附近的一些轨迹进行优化，即**滑动窗口估计法**
  
- 本节主要讨论**非线性优化为主的批量优化方法**
  
  - 状态$x,y$的条件概率分布为:$P(x,y|z,u)$
  
    - $x=\{x_1,\cdots ,x_N\}$：所有时刻(1-N)的相机位姿（用于推断相机位姿）
  
    - $y=\{y_1,\cdots,y_M\}$：所有时刻(1-N)路标位姿，即特征点的位置（用于建立地图）
  
    - z:所有时刻的观测数据（路标在图像的像素坐标下的坐标）
  
    - u:所有时刻的输入（机器人各种传感器的数据IMU,速度等等）
  
      如果没有输入，只有一张张图像，状态估计的条件概率分布为：$P(x,y|z)$,此时SLAM问题也就变成了SFM三维重建问题。
  
  - 利用**贝叶斯定理**(即5式的$\propto$左边部分)估计状态变量的条件分布
    $$
    P(x,y|z,u)=\underbrace{\frac{P(z,u|x,y)P(x,y)}{P(z,u)}}_{后验}\propto\underbrace{P(z,u|x,y)}_{似然}\underbrace{P(x,y)}_{先验} \tag{5}
    $$
  
    - 直接求后验分布是困难的，但是**求一个状态最优估计，使得该状态下后验概率最大化(MAP, Maximum A Posteriori)是可行的**
  
      $$
      (x,y)^*_{MAP}=argmaxP(z,u|x,y)P(x,y)\tag{5.1}
      $$
      
      - 贝叶斯定理告诉我们，求解最大后验概率MAP等价于最大化 **似然和先验的乘积**
      
      > 贝叶斯定理将先验概率和似然函数结合，用于计算后验概率，从而实现对事件概率的更新和推断
      
    - 如果我们不知道机器人位姿或路标，就没有了先验，那么就变成了求解**最大似然估计**(MLE,Maximize Likelihood Estimation)，即：
      $$
      (x,y)^*_{MLE}=argmaxP(z,u|x,y)\tag{5.2}
      $$
    
    
      - **似然（概率）**：在现在位姿下，可能产生怎么样的观测数据
    
        另一种解释：似然函数是在给定模型参数的情况下，观测数据出现的概率
    
      - **最大似然估计**：在什么状态下，最可能产生现在观测到的数据。
    
    > **先验概率（Prior Probability）**：
    >
    > 先验概率是在考虑任何新的观测数据之前，根据已有的信息或知识对事件的概率分布进行的估计。它表示我们对事件发生概率的初始猜测或信念，不考虑新数据的影响。先验概率是主观上的，它可以是基于经验、专业知识或先前观测得出的。
    >
    > 数学表达式：P(A)，表示事件 A 发生的概率。
    >
    > **后验概率（Posterior Probability）**：
    >
    > 后验概率是在考虑了新的观测数据之后，更新了先验概率的概率分布。它结合了先验概率和新数据的信息，反映了事件发生概率在新数据影响下的变化。后验概率是通过贝叶斯定理计算得出的。
    >
    > 数学表达式：P(A∣B)，表示在事件B 发生的条件下，事件A 发生的概率。
    >
    > **后验概率和条件概率(Conditional Probability)的区别**
    >
    > 虽然数学公式上章的差不多，但含义不同。后验概率是在考虑了新数据后的条件下对事件发生概率的描述，而条件概率是在已知某事件的情况下另一个事件发生的概率。

### 1.2 转为最小二乘问题

slam的状态估计问题，可以变成一个最小二乘问题least squares method。即**当数据服从高斯分布，最大似然估计等价于最小二乘问题**

最小二乘法是一种数学优化建模方法。它通过最小化误差的平方来寻找最佳函数和真实数据相匹配。利用最小二乘法可以简便的求得未知的数据，并使得求得的数据与实际数据之间误差的平方和为最小。

- 由1.1中观测公式2和噪声公式4，可结合得到观测数据的似然为：
  $$
  P(\mathbf{z}_{j,k}|\mathbf{x}_k,\mathbf{y}_j)=\mathcal{N}(h(\mathbf{y}_j,\mathbf{x}_k),\mathbf{Q}_{k,j})\tag{6}
  $$

  -  h()：观测方程，其值维高斯分布的均值向量
  -  Q：高斯分布协方差矩阵
  
  它依然是一个高斯分布。
  
- 对于6式，考虑**单次观测的最大似然估计**，可以使用**最小化负对数**(下面1,2这两步，公式见书)来求一个高斯分布的最大似然。

  1. 首先将高斯分布写成，概率密度函数展开形式
  2. 然后对其取负对数
     - 因为对数函数是单调递增的，所以原函数求最大化相当于对负对数求最小化

  带入SLAM的观测模型，我们就是求解：
  $$
  \begin{align}
  (x_k,y_j)^* &=arg\ max \mathcal{N}(h(\mathbf{y}_j,\mathbf{x}_k),\mathbf{Q}_{k,j})\\
  &=arg\ min\big((\mathbf{z}_{k,j}-h(\mathbf{x}_k,\mathbf{y}_j))^T\mathbf{Q}_{k,j}^{-1}(\mathbf{z}_{k,j}-h(\mathbf{x}_k,\mathbf{y}_j))\big)
  \end{align}\tag{7}
  $$
  
  7式中间省略了展开成概率密度的中间步骤。
  
  - 可以看出7式等价于，最小化噪声(误差)的一个二次型
    - 这个二次型也称为马氏距离(Mahalanobis distance)
    - 也可看作是信息矩阵$\mathbf{Q}_{k,j}^{-1}$加权之后的欧氏距离
    
    > 协方差矩阵的逆常常被称为信息矩阵（Information Matrix）
    >
    > 信息矩阵衡量了参数估计的不确定性，其元素越大表示参数估计的精度越高。
  
- 对于**批量时刻的数据**，可以将其看作**最小二乘问题**(Least Square Problem)

  1. 由于各个输入u,各个观测z之间都是独立的，且u和z之间也是独立的，所以我们可以首先对最大似然估计5.2式的P进行因式分解
     $$
     P(\mathbf{z,u}|\mathbf{x},\mathbf{y})=\prod_kP(\mathbf{u}_k|\mathbf{x}_{k-1},\mathbf{x}_k)\prod_{k,j}P(\mathbf{z}_{k,j}|\mathbf{x}_k,\mathbf{y}_j) \tag{8}
     $$
  
     - 定义各次输入和观测数据与模型（方程）之间的误差
       $$
       e_{u,k}=x_k-f(x_{k-1},u_k) \\
       e_{z,j,k}=z_{k,j}-h(x_k,y_j) \tag{9}
       $$
  
  2. 最小化上诉误差，等价于求最大似然估计。负对数将乘积变成求和，得到**最小二乘问题**：
     $$
     min\ J(\mathbf{x,y})=\sum_k \mathbf{e}_{u,k}^T\mathbf{R}_k^{-1}\mathbf{e}_{u,k}+\sum_k\sum_j\mathbf{e}^T_{z,k,j}\mathbf{Q}^{-1}_{k,j}\mathbf{e}_{z,k,j}\tag{10}
     $$
     观察9式可看出：
  
     - SLAM中最小二乘问题的目标函数由多个误差的二次型组成。
  
     - 如果用李代数表述增量，则该问题是**无约束**的最小二乘问题。否则用旋转矩阵就会带入正交阵和行列式为1的额外条件。
  
       所以李代数简化了优化过程
  
     - 误差分布会影响其在问题中的全冲。
  
       - 如果某次观测很准，那么协方差矩阵R就会小，而信息矩阵Q就会大。

### 1.3 例子：批量状态估计

希望从右噪声的信息中，准确估计沿着x轴前进或后退的小车的坐标。

1. 考虑一个离散时间系统

     - 运动方程：$\mathbf{x}_k=\mathbf{x}_{k-1}+\mathbf{u}_k+\mathbf{w}_k,\ \ \ \ \mathbf{w}_k\sim\mathcal{N}(0,\mathbf{R}_k)$
       - x：小车坐标
       - $\mathbf{u}_k$:输入
       - $\mathbf{w}_k$:噪声

     - 观测方程：$\mathbf{z}_k=\mathbf{x}_k+\mathbf{n}_k,\ \ \ \ \mathbf{n}_k\sim\mathcal{N}(0,\mathbf{Q}_k)$
       - z:对小车位置的测量
       - $\mathbf{n}_k$:噪声


2. 取时间k=1,2,3，初始位置$x_0$已知，并令：

     - 批量状态变量：$x=[x_0,x_1,x_2,x_3]^T$

     - 批量观测为：$z=[z_1,z_2,z_3]^T$

     - 批量输入为：$u=[u_1,u_2,u_3]^T$


3. 根据1.2中最大似然估计5.2式 和 8式因式分解得到本系统的最大似然估计：
   $$
   \begin{align}
     x^* &=arg\ maxP(u,z|x)		\\
         &=\prod_{k=1}^3P(u_k|x_{k-1},x_k)\prod_{k=1}^3P(z_k|x_k) 
     \end{align}
   $$
   根据1.2中4，6式可得到：

   - 对于运动方程：$P(u_k|x_{k-1},x_k)=\mathcal{N}(x_k-x_{k-1},R_k)$
   - 对于观测方程：$P(z_k|x_k) =\mathcal{N}(x_k,Q_k)$

4. 根据1.2中9式，构建该系统得误差变量：
   $$
   e_{u,k}=x_k-x_{k-1}-u_k\\
   e_{z,k}=z_k-x_k
   $$

5. 根据1.2中10式得到最小二乘的目标函数
   $$
   x^*=min\ J(x)=min\sum_{k=1}^3 \mathbf{e}_{u,k}^T\mathbf{R}_k^{-1}\mathbf{e}_{u,k}+\sum_{k=1}^3\mathbf{e}^T_{z,k}\mathbf{Q}^{-1}_{k}\mathbf{e}_{z,k}
   $$

## 2.非线性最小二乘

最小二乘法是一种数学优化建模方法。它通过最小化误差的平方来寻找最佳函数和真实数据相匹配。

利用最小二乘法可以简便的求得未知的数据，并使得求得的数据与实际数据之间误差的平方和为最小。

最小二乘问题：
$$
\mathop{min}_x\ F(x)=\frac{1}{2}||f(x)||_2^2 \tag{1}
$$
最小化它可以用解析的方式来求：即令其导数为0。得到的可能式最小、最大或鞍点处的值，只要代入f(x)比较下就可以。

但除非f(x)是简单的线性函数，否则很难求导。

为此可以用迭代法将求解**导数=0**的问题变成寻找**下降增量$\Delta x_k$**的问题：

1. 给定某个初始值$x_0$
2. 对于第k次迭代，寻找一个增量$\Delta x_k$,使得$||f(x_k+\Delta x_k)||_2^2$达成极小值。
3. 若$\Delta x_k$足够小，则停止。
4. 否则，令$x_{k+1}=x_k+\Delta x_k$,返回第2步

下面的问题就是如何寻找这个增量$\Delta x_k$

### 2.1 一阶和二阶梯度法：

将函数泰勒展开后，保留一阶或二阶项的求解方法，就称之为一阶梯度或二阶梯度法。

- 将目标函数F(x)在$x_k$附近进行泰勒展开：
  $$
  F(x_k+\Delta x_k)\approx F(x_k)+\mathbf{J}(x_k)^T\Delta x_k+\frac{1}{2}\Delta x_k^T\mathbf{H}(x_k)\Delta x_k \tag{2}
  $$

  - $\mathbf{J}(x_k)$：雅可比矩阵是F(x)关于x的一阶导数
  - $\mathbf{H}(x_k)$:海塞Hessian矩阵，是二阶导数

- 如果只保留一阶导，取增量为反向的梯度就可以保证函数F(x)下降：
  $$
  \Delta x^*=-\mathbf{J}(x_k) \tag{3}
  $$

  - 也被称为**最速下降法**

- 如果同时保留二阶导，增量方程变为：
  $$
  \Delta x^*=arg\ min\big(F(x)+\mathbf{J}(x)^T\Delta x+\frac{1}{2}\Delta x^T\mathbf{H}\Delta x \big) \tag{4}
  $$

  - 为使F(x)最小，求右侧等式关于$\Delta x$的导数，并令其为零，得到：
    $$
    \mathbf{J}+\mathbf{H\Delta x}=0 \Rightarrow \mathbf{H\Delta x}=-\mathbf{J} \tag{5}
    $$
    求解此方程就得到增量了。这也被称之为**牛顿法**

- 但牛顿法中计算函数F的海塞矩阵式很麻烦的事，最速下降法中又容易走出锯齿路线反而增加迭代次数。

  所以对于最小二乘问题，普遍使用高斯牛顿法和列文伯格-马夸尔特方法。

### 2.2 高斯牛顿法：

- 回顾1式的最小二乘问题：
  $$
  \mathop{min}_x\ F(x)=\frac{1}{2}||f(x)||_2^2 \tag{1}
  $$

  - f(x)：任意非线性函数，$R^N\rightarrow R$

    > 一个函数可以将一个或多个输入变量映射到一个输出变量

- 有别于牛顿法，高斯牛顿法是把1式的f(x)而非F(x)进行泰勒展开
  $$
  f(x+\Delta x)\approx f(x)+\mathbf{J}(x)^T\Delta x \tag{6}
  $$
  
- 近似二阶泰勒展开，并将求$\Delta x$就变成解一个线性的最小二乘问题：
  $$
  \begin{align}
  \Delta x^*&=arg\ \mathop{min}_{\Delta x}\frac{1}{2}||f(x)+\mathbf{J}(x)^T\Delta x ||^2\\
  &=arg\ \mathop{min}_{\Delta x}\frac{1}{2}\big(||f(x)||_2^2+2f(x)\mathbf{J}(x)^T\Delta x+\Delta x^T\mathbf{J}(x)\mathbf{J}(x)^T\Delta x \big)
  \end{align} \tag{7}
  $$

- 求7式右侧关于$\Delta x$的导数并令其为0,得到**增量方程**（也叫高斯牛顿方程）如下：
  $$
  \begin{aligned}
  \mathbf{J}(x)f(x)+\mathbf{J}(x)\mathbf{J}^T(x)\Delta x&=0\\
  \underbrace{\mathbf{J}(x)\mathbf{J}^T(x)}_{H(x)}\Delta x&=\underbrace{-\mathbf{J}(x)f(x)}_{g(x)} \\
  H\Delta x&=g
  \end{aligned}\tag{8}
  $$

  - 增量方程是关于$\Delta x$的线性方程组
  - 也可以称之为高斯牛顿方程(Gauss-Newton equation)或者正规方程(Normal equation)
  - 相比于牛顿法中的5式，高斯牛顿法用$\mathbf{J}(x)\mathbf{J}^T(x)$**作为牛顿法中二阶海塞矩阵H的近似，从而省略了计算复杂的海塞矩阵过程。**

- 如上，得到**高斯牛顿法的算法**步骤：

  1. 初始化：根据经验或启发式方法，初始化参数$x_0$，即要优化的变量
  2. 线性化：对于第k次迭代，求出当前的雅可比矩阵$\mathbf{J}(x_k)$和非线性方程$f(x_k)$，
  3. 求解增量方程：$H\Delta x_k=g$，即8式
  4. 若$\Delta x_k$足够小，则停止。否则，令$x_{k+1}=x_k+\Delta x_k$，返回第二步。

- 高斯牛顿法的**缺点**

  为求解增量方程，需要$H=JJ^T$的逆，但实际数据中计算得到$JJ^T$只是半正定性的。

  所以在实际计算中，$JJ^T$可能是奇异矩阵或是病态的(ill-condition)，从而导致算法不收敛。

### 2.3 列文伯格-马夸尔特方法：

L-M方法全称Levenberg-Marquardt方法，一定层度上修正了高斯牛顿法的缺点。因此比高斯牛顿法更健壮，但收敛速度更慢。

也称之为**阻尼牛顿法(Damped Newton Method)**

通常问题性质好时用高斯牛顿法作为梯度下降策略，问题接近病态时用L-M法

- 高斯牛顿法只能在展开点附近有较好的近似效果，L-M法在此基础上给$\Delta x$增加了一个**信赖区域**(Trust Region)

  在这个区域内，对牛顿法中海塞矩阵H的二阶近似是有效的，出了这个区域就可能有问题导致不收敛。

- 根据近似模型和实际函数之间的差异来确定这个区域的范围：

  - 如果差异小：说明近似效果好，扩大近似的范围。
  - 如果差异大：说明近似效果不好，缩小近似的范围。

- 通过指标$\rho$来判断近似效果的好坏
  $$
  \rho=\frac{f(x+\Delta x)-f(x)}{\mathbf{J}(x)^T\Delta x} \tag{9}
  $$

  - 分子：实际函数下降的值
  - 分母：近似模型下降的值
  - $\rho$越接近于1，近似越好
  - $\rho$太小：说明实际减小的值远少于近似减小的值，近似较差，需要缩小近似范围
  - $\rho$比较大：说明实际下降的值比预计的要打，可以放大近似范围

- 如上，得到**L-M算法**：

  1. 给定初始值$x_0$,和初始优化半径$\mu$

  2. 求解增量方程：
     $$
     \Delta x=\mathop{min}_{\Delta x_k}\frac{1}{2}||f(x_k)+\mathbf{J}(x_k)^T\Delta x_k||^2,\ \ \ s.t.||\mathbf{D}\Delta x_k||^2<=\mu
     $$

     - $\mu$:信赖区域的半径
     - $D$:系数矩阵
     - s.t.: subject to 受限于

  3. 根据式9，计算指标$\rho$

     - 若$\rho>\frac{3}{4}$,设置$\mu=2\mu$
     - 若$\rho < \frac{1}{4}$,设置$\mu=0.5\mu$
     - 如果$\rho$大于某阈值，则认为近似可行，令$x_{k+1}=x_k+\Delta x_k$

  4. 判断算法是否收敛，如不收敛回到第二步，否则结束

- 由于L-M方法中多了一个限制条件:$||\mathbf{D}\Delta x_k||^2<=\mu$，即$\Delta x_k$被约束在一个椭球中，将其放入目标函数后可g构成**拉格朗日函数**：
  $$
  \mathcal{L}(\Delta x_k,\lambda)=\frac{1}{2}||f(x_k)+\mathbf{J}(x_k)^T\Delta x_k||^2+\frac{\lambda}{2}\big(||\mathbf{D}\Delta x_k||^2-\mu\big) \tag{10}
  $$

  - $\lambda$:拉格朗日乘子

- 令10式关于$\Delta x_k$导数为0得到最终要求解的**增量方程**：
  $$
  (\mathbf{H}+\lambda \mathbf{D}^T\mathbf{D})\Delta x_k=\mathbf{g} \tag{11}
  $$
  相比于高斯牛顿法，增量方程多了$\lambda \mathbf{D}^T\mathbf{D}$,若简化$\mathbf{D}=I$,则11式就变成了
  $$
  (\mathbf{H}+\lambda \mathbf{I})\Delta x_k=\mathbf{g} \tag{12}
  $$

  - 当$\lambda$较小时，H占主导地位，L-M法接近于高斯牛顿法，在范围内能很好的近似
  - 当$\lambda$较大时，$\lambda I$占主导地位，L-M法接近于一阶梯度下降法（最速下降法），因为此时范围内二阶近似效果不好。

### 2.4 上面用到的泰勒展开：

1. **泰勒公式**：

   泰勒公式是一种计算近似值的方法，在已知函数在某一点的各阶导数值的情况之下，泰勒公式可以用这些导数值做系数构建一个多项式来逼近函数在这一点的邻域中的值。

   如果f(x)在x0处具有任意阶导数，那么泰勒公式是这样的：
   $$
   f(x)=f(x_0)+(x-x_0)f'(x_0)+(x-x_0)^2\frac{f''(x_0)}{2!}+....=\sum_{n=0}^{\infin}\frac{f^{(n)}(x_0)}{n!}(x-x_0)^n
   $$

   - $x-x_0$：也就是最小二乘问题想要求解的增量$\Delta x$
   - $f'(x_0)$：也就是2式中的雅可比矩阵 J
   - $f''(x_0)$：也就是2式中的海塞Hessian矩阵 H

## 3. 实践：曲线拟合问题

### 3.0 问题+解法

- 问题描述：

  考虑一条满足方程：$y=exp(ax^2+bx+c)+w$ 的曲线，a/b/c为参数，$w\sim(0,\sigma^2)$是高斯噪声。

  假设有N个关于x,y的观测数据，想根据这些数据求出曲线的参数。

  得到最小二乘问题：
  $$
  \mathop{min}_{a,b,c}\ F(a,b,c)=\mathop{min}_{a,b,c}\frac{1}{2}||f(a,b,c)||_2^2=\mathop{min}_{a,b,c}\frac{1}{2}\sum_{i=1}^N|| y_i-exp(ax_i^2+bx_i+c) ||^2 \tag{13}
  $$

- 解法：

  - 定义误差：$e_i=y_i-exp(ax_i^2+bx_i+c)$

  - 求误差对每个状态变量的导数：
    $$
    \begin{align}
    \frac{\partial e_i}{\partial a}&=-x_i^2exp(ax_i^2+bx_i+c)\\
    \frac{\partial e_i}{\partial b}&=-x_iexp(ax_i^2+bx_i+c)\\
    \frac{\partial e_i}{\partial c}&=-exp(ax_i^2+bx_i+c)
    \end{align} \tag{14}
    $$

  - 根据8式列出高斯牛顿法的增量方程：
    $$
    \begin{align}
    \underbrace{\mathbf{J}(x)\mathbf{J}^T(x)}_{H(x)}\Delta x &=\underbrace{-\mathbf{J}(x)f(x)}_{g(x)} \tag{8}\\
    \big( \sum_{i=1}^{100}\mathbf{J}_i(\sigma^2)^{-1}\mathbf{J}_i^T \big)\Delta x_k &= \sum_{i=1}^{100}-\mathbf{J}_i(\sigma^2)^{-1}e_i \tag{15}
    \end{align}
    $$
    
    - 其中$\mathbf{J}_i=[\frac{\partial e_i}{\partial a},\frac{\partial e_i}{\partial b},\frac{\partial e_i}{\partial c}]^T$
    - 15式也可以用向量的形式写，即将所有的J排成一列
    - 这里$\sigma$是噪声sigma值，乘上J的平方，而不是J的参数
    
  - 不断的更新参数a,b,c，直到收敛

### 3.1 手写高斯牛顿法

#### 3.1.1 代码

```c++
#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
  double ar = 1.0, br = 2.0, cr = 1.0;         // 真实参数值，用于生成模拟数据，也是参数应该优化到的目标
  double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值，待优化的参数
  int N = 100;                                 // 数据点
  double w_sigma = 1.0;                        // 噪声Sigma值
  double inv_sigma = 1.0 / w_sigma;
  cv::RNG rng;                                 // OpenCV随机数产生器

  vector<double> x_data, y_data;      // 产生100个随机模拟数据，用于拟合曲线
  for (int i = 0; i < N; i++) {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
  }

  // 开始Gauss-Newton迭代
  int iterations = 100;    // 迭代次数
  double cost = 0, lastCost = 0;  // 本次迭代的cost和上一次迭代的cost

  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();  //c++11提供的时间日期处理库
  for (int iter = 0; iter < iterations; iter++) {

    Matrix3d H = Matrix3d::Zero();             // Hessian = J^T W^{-1} J in Gauss-Newton
    Vector3d b = Vector3d::Zero();             // bias
    cost = 0;

    for (int i = 0; i < N; i++) {
      double xi = x_data[i], yi = y_data[i];  // 第i个数据点
      // ******************* 建立误差项e，即非线性方程f(x)，13式 *******************
      double error = yi - exp(ae * xi * xi + be * xi + ce);
      Vector3d J; // 雅可比矩阵
        
      // ******************* 计算14式 *******************
      J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);  // de/da
      J[1] = -xi * exp(ae * xi * xi + be * xi + ce);  // de/db
      J[2] = -exp(ae * xi * xi + be * xi + ce);  // de/dc
        
      // ******************* 计算15式的变量*******************
      H += inv_sigma * inv_sigma * J * J.transpose();	// 15式的左边
      b += -inv_sigma * inv_sigma * error * J;			// 15式的右边

      cost += error * error;
    }
	
  	// ******************* 计算15式 *******************
    // 求解线性方程 Hx=b
    // Vector3d x = A.ldlt().solve(b);//求解Ax=b
    // ldlt() 函数用于执行Cholesky分解，这个分解会将矩阵分解为 A=LDL^T，所以也被称为LDLT分解
    	// L:下三角矩阵且所有对角元素均为正实数
      	// D:对角矩阵
      	// L^T:\mathbf {L} 的转置矩阵
    // solve() 使用LDLT分解来求解线性方程
    Vector3d dx = H.ldlt().solve(b);
    if (isnan(dx[0])) {   // c++11提供的isnan用于判断是否是非数NaN：表示未定义或不可表示的值
      cout << "result is nan!" << endl;
      break;
    }

    if (iter > 0 && cost >= lastCost) {
      cout << "cost: " << cost << ">= last cost: " << lastCost << ", break." << endl;
      break;
    }
	
  	// ******************* 更新参数 *******************
    ae += dx[0];
    be += dx[1];
    ce += dx[2];

    lastCost = cost;

    cout << "total cost: " << cost << ", \t\tupdate: " << dx.transpose() <<
         "\t\testimated params: " << ae << "," << be << "," << ce << endl;
  }

  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  cout << "estimated abc = " << ae << ", " << be << ", " << ce << endl;
  return 0;
}
```

#### 3.1.2 编译

CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.0)
project(test)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")
add_executable(test test.cpp)
target_link_libraries(test ${OpenCV_LIBS})
```

### 3.2 使用ceres进行曲线拟合

#### 3.2.1 代码

```c++
#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

/* 
Ceres第一步： 代价函数(残差块)的计算模型，通过定义一个类（或结构体）来实现。
		     
我们需要在类中通过关键字operator来定义带模板参数的()运算符，如此这个类就变成了拟函数(Functor),ceres就可以像调用函数一样对该类的某对象a调用a<double>()方法。

ceres会将雅可比矩阵作为类型参数传入此函数，从而实现自动求导功能
*/
struct CURVE_FITTING_COST {
  // 构造函数最后这里没有分号哦！
  CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

  // 使用模板函数进行 残差的计算
  template<typename T>
  bool operator()(const T *const abc, T *residual) const {
    residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]); // y-exp(ax^2+bx+c)
    return true;
  }

  const double _x, _y;    // x,y数据
};

int main(int argc, char **argv) {
  double ar = 1.0, br = 2.0, cr = 1.0;         // 真实参数值
  double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值
  int N = 100;                                 // 数据点
  double w_sigma = 1.0;                        // 噪声Sigma值
  double inv_sigma = 1.0 / w_sigma;
  cv::RNG rng;                                 // OpenCV随机数产生器

  vector<double> x_data, y_data;      // 生成100个模拟数据，用于曲线拟合
  for (int i = 0; i < N; i++) {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
  }
  /*
  Ceres第二步：定义参数块
  */
  double abc[3] = {ae, be, ce};

  /*
  Ceres第三步：构建最小二乘问题
  
  优化的梯度计算ceres提供3种选择
  1. 自动求导:AuoDiff
  2. 数值求导:NumericDiff
  3. 我们自己推导解析的倒数形式
  这里使用第一种：AutoDiffCostFunction。因为它在编码上最方便。
  自动求导需要指定误差项和优化变量的维度，这里误差是标量所以维度=1，优化变量是a,b,c所以维度=3
  */
  ceres::Problem problem;
  for (int i = 0; i < N; i++) {
    problem.AddResidualBlock(     // 向问题中添加误差项
      // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(x_data[i], y_data[i])
      ),
      nullptr,            // 核函数，这里不使用，为空
      abc                 // 待估计参数
    );
  }

  /*
  Ceres第四步：配置求解器
  
  在上面设定好问题后，就可以调用solve函数进行求解了。
  可以在option里配置优化选项，例如使用Line Search还是Trust Region、迭代次数、步长，等等
  */
  ceres::Solver::Options options;     // 这里有很多配置项可以填
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 增量方程如何求解
  options.minimizer_progress_to_stdout = true;   // 输出到cout

  ceres::Solver::Summary summary;                // 优化信息
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);  // 开始优化
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  // 输出结果
  cout << summary.BriefReport() << endl;
  cout << "estimated a,b,c = ";
  for (auto a:abc) cout << a << " ";
  cout << endl;

  return 0;
}
```



#### 3.2.2 编译

```cmake
cmake_minimum_required(VERSION 3.0)
project(test)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


add_executable(test test.cpp)
target_link_libraries(test ${OpenCV_LIBS} ${CERES_LIBRARIES})
```

### 3.3 使用g2o进行曲线拟合

曲线拟合问题中只有一个顶点（优化变量），即曲线模型的参数a,b,c

各个带噪声的数据点，构成了一个个误差项，也就是图优化的边。

#### 3.3.1 代码

```c++
#include <iostream>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>

using namespace std;
// g2o的标准顶点和边通常不能满足需求，所以一般都需要对他们进行重写

// ********************* 重写顶点 *********************
// 继承g2o顶点基本类，创建我们自己的曲线模型的顶点，模板参数：优化变量维度和数据类型
// 标准顶点BaseVertex< D, T >： D顶点的最小维度，T估计值的类型
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  // 宏，用于在使用Eigen库的C++类中自定义内存分配，以确保对齐内存分配，从而提高程序的性能
  // 内存对齐可以减少内存访问的成本，提高数据访问速度。而Eigen库的数据结构通常是对齐的
  // 所以在类中加上这个宏，就是重载了这个类的new操作符，确保通过该操作符分配的内存是对齐的。
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 1. 重写顶点的重置函数，用来将顶点的估计值初始化为0,0,0
  // 因为本问题的优化参数是a,b,c，所以是3个
  virtual void setToOriginImpl() override {
  	_estimate << 0, 0, 0;	// 要被优化的估计值的初始值设为0，0，0
  }

  // 2. 重写顶点的更新函数 x_{k+1} = x_k +dx
  // 简单的加法为什么更新要自己写，g2o不帮我们完成呢？ 因为在向量空间中的确是简单的加法，但如果x是位姿(位移矩阵)就不一定有加法了，根据第四讲需要用左乘或右乘的方式更新。
  virtual void oplusImpl(const double *update) override {
    _estimate += Eigen::Vector3d(update);	// 要被优化的估计值每次迭代的更新函数
  }

  // 3. 存盘和读盘：因为本例不需要读写操作，所以留空
  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}
};

// ********************* 重写边 *********************
// g2o第二步： 继承g2o边基本类，创建我们自己的误差模型 模板参数：观测值维度，类型，连接顶点类型
// BaseUnaryEdge一元边(D，E,Vertex) D:
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // 1. 边的构造函数，这里x是3.0中的x，即观测值
  CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

  // 2. 重写边的误差计算，计算曲线模型误差
  virtual void computeError() override {
    // 取出边所连接的的顶点的当前估计值
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
    // 读取需要优化的估计值
    const Eigen::Vector3d abc = v->estimate();
    // 误差项：观测值-非线性方程。 参照13式
    _error(0, 0) = _measurement - std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
  }

  // 重写边的雅可比计算，计算每条边相对于顶点的雅可比
  // 下面的公式计算参考3.0的14式
  virtual void linearizeOplus() override {
    // _vertices[0]：这是边的顶点数组中的第一个（唯一的）元素，它是一个指向顶点的基类指针。
    // static_cast<xx>(yy)：将YY变量的类型转换为xx类型
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();	// 读取要优化的变量，即顶点的值
    // 3.0的14式
    double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]); 
    // _jacobianOplusXi变量：用于存储关于边的代价函数对于连接的顶点的雅可比矩阵
    _jacobianOplusXi[0] = -_x * _x * y;
    _jacobianOplusXi[1] = -_x * y;
    _jacobianOplusXi[2] = -y;
  }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

public:
  double _x;  // 测量值：x 值； _measurement观测值：y 值
};



int main(int argc, char **argv) {
  double ar = 1.0, br = 2.0, cr = 1.0;         // 真实参数值，真实参数值，用于生成模拟数据，也是参数应该优化到的目标
  double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值，即我们要优化的变量
  int N = 100;                                 // 数据点
  double w_sigma = 1.0;                        // 噪声Sigma值
  double inv_sigma = 1.0 / w_sigma;
  cv::RNG rng;                                 // OpenCV随机数产生器

  vector<double> x_data, y_data;      		   // 生成模拟的观测数据
  for (int i = 0; i < N; i++) {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
  }
	

  // ********************* 1.定义块求解器BlockSolver*********************
  // BlockSolver 是一个通用的求解器框架，用于求解大多数图优化问题
  // BlockSolverTraits 用于指定块矩阵的维度，每个误差项优化变量维度为3，误差值维度为1
  // 块求解器将整个问题划分成块状的子问题，每个子问题对应一个顶点的一部分和与之相关的边。这样，优化问题被分解为多个小的线性方程组。块求解器在迭代中解决这些小的线性方程组，通过优化每个子问题的变量来逐步逼近全局最优解。
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;  			
    
  // ********************* 2.定义线性求解器LinearSolver *********************
  // LinearSolverDense 是一个线性求解器，用于求解线性方程组，通常在图优化问题的每一步迭代中使用。
  // 使用 BlockSolverType 的 PoseMatrixType 作为块矩阵的类型。这个设置是为了使线性求解器与之前设置的块求解器类型匹配，以确保整个求解器链能够正常工作。
  // 在图优化中，每个子问题都会对应一个线性方程组，其中方程的系数矩阵是与边相关的雅可比矩阵。线性求解器的作用就是将这个线性方程组的系数矩阵求解出来，以得到顶点的更新量，从而迭代地更新顶点的估计值
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; 
  
  // ********************* 3.创建总求解器solver*********************
  // 从最小二乘法计算算法:GN, LM, DogLeg 中选一个作为迭代策略，这里选了高斯牛顿
  // 创建上面定义的块求解器BlockSolver和线性求解器LinearSolver，以此初始化总求解器
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
    g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  
  // ********************* 4.创建稀疏优化器SparseOptimizer*********************
  // 创建SparseOptimizer类对象，用于管理优化过程中的顶点、边和求解器等相关内容
  g2o::SparseOptimizer optimizer;   
  // 设置优化过程中求解器
  optimizer.setAlgorithm(solver);  
  // 开启了优化器的详细输出模式，也就是调试输出模式
  optimizer.setVerbose(true);       

  // 往图中增加顶点
  // ********************* 5.定义顶点，并添加到SparseOptimizer中*********************
  CurveFittingVertex *v = new CurveFittingVertex();
  v->setEstimate(Eigen::Vector3d(ae, be, ce));	// 为顶点 v 设置估计值，即要优化的变量
  v->setId(0);									// 为顶点 v 设置一个唯一的ID
  optimizer.addVertex(v);						// 将创建的顶点 v 添加到图优化器中

  // 往图中增加边
  // ********************* 5.定义边，并添加到SparseOptimizer中*********************
  for (int i = 0; i < N; i++) {
    CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
    edge->setId(i);						  // 为边设置一个唯一的id
    edge->setVertex(0, v);                // 设置连接的顶点
    edge->setMeasurement(y_data[i]);      // 观测数值
    // 设置信息矩阵：1 / (噪声标准差的平方)
    // 信息矩阵用于表示测量值x的精度或权重
    // 因为3.0中的问题，只有一个测量值x,所以下面的维度是1x1
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma)); 
    optimizer.addEdge(edge);
  }
	
  
  // ********************* 6.设置优化参数，并开始优化 *********************
  cout << "start optimization" << endl;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  optimizer.initializeOptimization();	// 图优化初始化
  optimizer.optimize(10);				// 进行10次迭代优化
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  // 输出优化完的参数值
  Eigen::Vector3d abc_estimate = v->estimate();
  cout << "estimated model: " << abc_estimate.transpose() << endl;

  return 0;
}
```



#### 3.3.2 编译

```c++
cmake_minimum_required(VERSION 3.0)
project(test)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++17 -O3")
set( G2O_ROOT /usr/local/include/g2o ) 

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# g2o
list(APPEND CMAKE_MODULE_PATH /home/yang/3rd_library/g2o-20230223_git/cmake_modules)
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")

add_executable(test test.cpp)
target_link_libraries(test ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})

```



# 七、视觉里程计：特征点法

视觉里程计的核心问题是：**如何根据图像估计相机运动**。

## 1.特征点法：

### 1.1 特征点：

- **特征**Feature也就是SLAM中所说的**路标**。它是图像中有代表性的点，在相机视角发生少量变化后会保持不变，于是我们能在各个图像中找到相同的点。

- 传统中，这些特征是角点，因为他们相比于边缘和区块更容易被检测到，方法有Harris角点、FAST角点、GFTT角点等。

- 但应用中发现，单纯的角点检测不足以满足我们的要求，所以我们设计了一些更加稳定的局部图像特征：SIFT,SURF,ORB等等。

  这些人工设计的特征点有如下特点：

  1. 可重复性Repeatability:相同的特征可以在不同的图像中找到
  2. 可区别性Distinctiveness:不同的特征有不同的表达
  3. 高效率Efficiency:同一图像中，特征点的数量应该远小于像素的数量
  4. 本地性Locality:特征仅与一小片图像区域相关

- 这些人工设计的特征点由**关键点Key-point**和**描述子Descriptor**两部分组成

  - 关键点：指特征点在图像里的位置，有些特征点还有朝向和大小的信息
  - 描述子：通常是一个向量，根据人为设计，描述该关键点周围像素的信息

  描述子根据“外观相似的特征应有相似的描述子”的原则来设计，所以只要两个特征点的描述子在向量空间上的距离相近，就可以认为他们是同样的特征点。

- 有些特征很准确，但计算量很大：比如SIFT

  SIFT(尺度不变特征变换，Scale-Invariant Feature Transform)充分考虑了光照、尺度、旋转等变换，但其庞大的计算量，使得普通CPU无法实时计算SIFT特征

  SLAM中很少用这类奢侈的特征

- 有些特征，通过降低精度和鲁棒性来提升计算速度：如ORB

  FAST角点检测是一种特别快的角点检测方式，ORB(Oriented FAST and Rotated BRIEF)改进了FAST检测子不具有方向性的问题，并采用速度极快的二进制描述子BRIEF(Binary Robust Independent Elementarty Feature)

### 1.2 ORB特征

ORB特征分分为如下两步

1. **FAST角点提取**：

   找到图像中的“角点”。相较于原版FAST,ORB中另外计算了特征点的主方向，为后续BRIEF描述子增加了旋转不变特性。

   - FAST认为像素亮度差别较大就是角点，检测过程如下：

     1. 在图像中选取像素$p$,假设亮度为$I_p$
     2. 设置一个阈值$T$(比如$I_p$的20%)
     3. 以像素$p$为中心，选取半径为3的圆上的16个像素点。
     4. 如果选取的圆上有连续N个两点大于$I_p+T$或小于$I_p-T$,那么像素p可以被认为是特征点(N通常取12，也称为FAST-12)
     5. 循环以上4步，对每一个像素执行相同的操作

   - ORB在上述基础上添加了尺度和旋转的描述,得到Oriented Fast

     - 尺度：由构建图像金字塔，并在金字塔的每一层上检测角点来实现。

       金字塔的底层是原始图像，每往上一层图像就进行一个固定倍率的缩放，较小的图像可以看作是远处看过来的场景。

     - 旋转：由灰度质心法(Intensity Centroid)来实现

       灰度质心：以图像块灰度值作为权重的中心。计算如下：

       1. 在一个小的图像块$B$中，定义图像块的矩为：
          $$
          m_{pq}=\sum_{x,y\in B}x^py^qI(x,y),\ \ \ p,q={0,1}
          $$

       2. 通过矩找到图像块的质心：
          $$
          C=(\frac{m_{10}}{m_{00}},\frac{m_{01}}{m_{00}})
          $$

       3. 连接图像块的几何中心$O$与质心$C$,得到一个方向向量$\overrightarrow{OC}$,由此得到特征点的方向为：
          $$
          \theta=arctan(m_{01}/m_{10})
          $$

2. **BRIEF描述子**：

   对前一步提取出特征点的周围图像区域进行描述。ORB在原本BRIEF的基础上，使用了先前计算的方向信息。

   - BRIEF(Binary Robust Independent Elementarty Feature)是一种**二进制**描述子，其描述向量由许多个0，1组成。

     这里的0，1编码了关键点附近2个随机像素(比如m,n)的大小关系：如果m比n大，则取1，反之取0。

     如果我们取128个这样的随机像素对，我们就得到了128维由0，1组成的向量

   - 上述原始BRIEF描述子，容易在图像发生旋转时丢失，所以ORB在Oriented Fast阶段计算的关键点方向，帮助Steer Brief具有了旋转不变性。

### 1.3 特征匹配

特征匹配解决了SLAM中数据关联(Data Association)的问题，确定了当前看到的路标和之前看到的路标之间的对应关系。

最简单的特征匹配方法，就是**暴力匹配**(Brute-Force-Matcher)：即对图像1的每一个特征点，去和图像2中每一个特征点计算距离，最近的那个就作为匹配点。对于二进制的描述子，通常使用汉明距离(Hamming Distance)作为距离度量，

但暴力匹配需要巨大的计算量，不符合SLAM实时性要求，所以我们使用**快速近似最近邻(FLANN)**来处理匹配点数量很多的情况。



## 2.实践：特征提取和匹配

### 2.1 OpenCV的ORB特征

- 代码：

  ```c++
  #include <iostream>
  #include <opencv2/core/core.hpp>
  #include <opencv2/features2d/features2d.hpp>
  #include <opencv2/highgui/highgui.hpp>
  #include <chrono>
  
  using namespace std;
  using namespace cv;
  
  int main(int argc, char **argv) {
    if (argc != 3) {
      cout << "usage: feature_extraction img1 img2" << endl;
      return 1;
    }
    //-- 读取图像
    Mat img_1 = imread(argv[1], IMREAD_COLOR);
    Mat img_2 = imread(argv[2], IMREAD_COLOR);
    assert(img_1.data != nullptr && img_2.data != nullptr);
  
    //-- 初始化
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
  
    //-- 第一步:检测 Oriented FAST 角点位置
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);
  
    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "extract ORB cost = " << time_used.count() << " seconds. " << endl;
  
    Mat outimg1;
    drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT); // 在图1绘制关键点
    imshow("ORB features", outimg1);
  
    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> matches;
    t1 = chrono::steady_clock::now();
    matcher->match(descriptors_1, descriptors_2, matches);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "match ORB cost = " << time_used.count() << " seconds. " << endl;
  
    //-- 第四步:匹配点对筛选
    // 计算最小距离和最大距离
    auto min_max = minmax_element(matches.begin(), matches.end(),
                                  [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;
  
    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);
  
    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++) {
      if (matches[i].distance <= max(2 * min_dist, 30.0)) {
        good_matches.push_back(matches[i]);
      }
    }
  
    //-- 第五步:绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
    imshow("all matches", img_match);
    imshow("good matches", img_goodmatch);
    waitKey(0);
  
    return 0;
  }
  ```

- 编译

  ```cmake
  cmake_minimum_required(VERSION 3.0)
  project(test)
  
  set(CMAKE_BUILD_TYPE Release)
  set(CMAKE_CXX_FLAGS "-std=c++14 -mfma")
  
  # OpenCV
  find_package(OpenCV REQUIRED)
  include_directories(${OpenCV_INCLUDE_DIRS})
  
  add_executable(test test.cpp)
  # target_link_libraries(test ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
  target_link_libraries(test ${OpenCV_LIBS})
  ```

### 2.2 手写ORB特征

- 代码

  ```c++
  //
  // Created by xiang on 18-11-25.
  //
  
  #include <opencv2/opencv.hpp>
  #include <string>
  #include <nmmintrin.h>
  #include <chrono>
  
  using namespace std;
  
  // global variables
  string first_file = "../1.png";
  string second_file = "../2.png";
  
  // 32 bit unsigned int, will have 8, 8x32=256
  typedef vector<uint32_t> DescType; // Descriptor type
  
  /**
   * compute descriptor of orb keypoints
   * @param img input image
   * @param keypoints detected fast keypoints
   * @param descriptors descriptors
   *
   * NOTE: if a keypoint goes outside the image boundary (8 pixels), descriptors will not be computed and will be left as
   * empty
   */
  void ComputeORB(const cv::Mat &img, vector<cv::KeyPoint> &keypoints, vector<DescType> &descriptors);
  
  /**
   * brute-force match two sets of descriptors
   * @param desc1 the first descriptor
   * @param desc2 the second descriptor
   * @param matches matches of two images
   */
  void BfMatch(const vector<DescType> &desc1, const vector<DescType> &desc2, vector<cv::DMatch> &matches);
  
  int main(int argc, char **argv) {
  
    // load image
    cv::Mat first_image = cv::imread(first_file, 0);
    cv::Mat second_image = cv::imread(second_file, 0);
    assert(first_image.data != nullptr && second_image.data != nullptr);
  
    // detect FAST keypoints1 using threshold=40
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    vector<cv::KeyPoint> keypoints1;
    cv::FAST(first_image, keypoints1, 40);
    vector<DescType> descriptor1;
    ComputeORB(first_image, keypoints1, descriptor1);
  
    // same for the second
    vector<cv::KeyPoint> keypoints2;
    vector<DescType> descriptor2;
    cv::FAST(second_image, keypoints2, 40);
    ComputeORB(second_image, keypoints2, descriptor2);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "extract ORB cost = " << time_used.count() << " seconds. " << endl;
  
    // find matches
    vector<cv::DMatch> matches;
    t1 = chrono::steady_clock::now();
    BfMatch(descriptor1, descriptor2, matches);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "match ORB cost = " << time_used.count() << " seconds. " << endl;
    cout << "matches: " << matches.size() << endl;
  
    // plot the matches
    cv::Mat image_show;
    cv::drawMatches(first_image, keypoints1, second_image, keypoints2, matches, image_show);
    cv::imshow("matches", image_show);
    cv::imwrite("matches.png", image_show);
    cv::waitKey(0);
  
    cout << "done." << endl;
    return 0;
  }
  
  // -------------------------------------------------------------------------------------------------- //
  // ORB pattern样式
  // 用256位二进制描述，即对应8个32位unsigned int数据
  int ORB_pattern[256 * 4] = {
    8, -3, 9, 5/*mean (0), correlation (0)*/,
    4, 2, 7, -12/*mean (1.12461e-05), correlation (0.0437584)*/,
    -11, 9, -8, 2/*mean (3.37382e-05), correlation (0.0617409)*/,
    7, -12, 12, -13/*mean (5.62303e-05), correlation (0.0636977)*/,
    2, -13, 2, 12/*mean (0.000134953), correlation (0.085099)*/,
    1, -7, 1, 6/*mean (0.000528565), correlation (0.0857175)*/,
    -2, -10, -2, -4/*mean (0.0188821), correlation (0.0985774)*/,
    -13, -13, -11, -8/*mean (0.0363135), correlation (0.0899616)*/,
    -13, -3, -12, -9/*mean (0.121806), correlation (0.099849)*/,
    10, 4, 11, 9/*mean (0.122065), correlation (0.093285)*/,
    -13, -8, -8, -9/*mean (0.162787), correlation (0.0942748)*/,
    -11, 7, -9, 12/*mean (0.21561), correlation (0.0974438)*/,
    7, 7, 12, 6/*mean (0.160583), correlation (0.130064)*/,
    -4, -5, -3, 0/*mean (0.228171), correlation (0.132998)*/,
    -13, 2, -12, -3/*mean (0.00997526), correlation (0.145926)*/,
    -9, 0, -7, 5/*mean (0.198234), correlation (0.143636)*/,
    12, -6, 12, -1/*mean (0.0676226), correlation (0.16689)*/,
    -3, 6, -2, 12/*mean (0.166847), correlation (0.171682)*/,
    -6, -13, -4, -8/*mean (0.101215), correlation (0.179716)*/,
    11, -13, 12, -8/*mean (0.200641), correlation (0.192279)*/,
    4, 7, 5, 1/*mean (0.205106), correlation (0.186848)*/,
    5, -3, 10, -3/*mean (0.234908), correlation (0.192319)*/,
    3, -7, 6, 12/*mean (0.0709964), correlation (0.210872)*/,
    -8, -7, -6, -2/*mean (0.0939834), correlation (0.212589)*/,
    -2, 11, -1, -10/*mean (0.127778), correlation (0.20866)*/,
    -13, 12, -8, 10/*mean (0.14783), correlation (0.206356)*/,
    -7, 3, -5, -3/*mean (0.182141), correlation (0.198942)*/,
    -4, 2, -3, 7/*mean (0.188237), correlation (0.21384)*/,
    -10, -12, -6, 11/*mean (0.14865), correlation (0.23571)*/,
    5, -12, 6, -7/*mean (0.222312), correlation (0.23324)*/,
    5, -6, 7, -1/*mean (0.229082), correlation (0.23389)*/,
    1, 0, 4, -5/*mean (0.241577), correlation (0.215286)*/,
    9, 11, 11, -13/*mean (0.00338507), correlation (0.251373)*/,
    4, 7, 4, 12/*mean (0.131005), correlation (0.257622)*/,
    2, -1, 4, 4/*mean (0.152755), correlation (0.255205)*/,
    -4, -12, -2, 7/*mean (0.182771), correlation (0.244867)*/,
    -8, -5, -7, -10/*mean (0.186898), correlation (0.23901)*/,
    4, 11, 9, 12/*mean (0.226226), correlation (0.258255)*/,
    0, -8, 1, -13/*mean (0.0897886), correlation (0.274827)*/,
    -13, -2, -8, 2/*mean (0.148774), correlation (0.28065)*/,
    -3, -2, -2, 3/*mean (0.153048), correlation (0.283063)*/,
    -6, 9, -4, -9/*mean (0.169523), correlation (0.278248)*/,
    8, 12, 10, 7/*mean (0.225337), correlation (0.282851)*/,
    0, 9, 1, 3/*mean (0.226687), correlation (0.278734)*/,
    7, -5, 11, -10/*mean (0.00693882), correlation (0.305161)*/,
    -13, -6, -11, 0/*mean (0.0227283), correlation (0.300181)*/,
    10, 7, 12, 1/*mean (0.125517), correlation (0.31089)*/,
    -6, -3, -6, 12/*mean (0.131748), correlation (0.312779)*/,
    10, -9, 12, -4/*mean (0.144827), correlation (0.292797)*/,
    -13, 8, -8, -12/*mean (0.149202), correlation (0.308918)*/,
    -13, 0, -8, -4/*mean (0.160909), correlation (0.310013)*/,
    3, 3, 7, 8/*mean (0.177755), correlation (0.309394)*/,
    5, 7, 10, -7/*mean (0.212337), correlation (0.310315)*/,
    -1, 7, 1, -12/*mean (0.214429), correlation (0.311933)*/,
    3, -10, 5, 6/*mean (0.235807), correlation (0.313104)*/,
    2, -4, 3, -10/*mean (0.00494827), correlation (0.344948)*/,
    -13, 0, -13, 5/*mean (0.0549145), correlation (0.344675)*/,
    -13, -7, -12, 12/*mean (0.103385), correlation (0.342715)*/,
    -13, 3, -11, 8/*mean (0.134222), correlation (0.322922)*/,
    -7, 12, -4, 7/*mean (0.153284), correlation (0.337061)*/,
    6, -10, 12, 8/*mean (0.154881), correlation (0.329257)*/,
    -9, -1, -7, -6/*mean (0.200967), correlation (0.33312)*/,
    -2, -5, 0, 12/*mean (0.201518), correlation (0.340635)*/,
    -12, 5, -7, 5/*mean (0.207805), correlation (0.335631)*/,
    3, -10, 8, -13/*mean (0.224438), correlation (0.34504)*/,
    -7, -7, -4, 5/*mean (0.239361), correlation (0.338053)*/,
    -3, -2, -1, -7/*mean (0.240744), correlation (0.344322)*/,
    2, 9, 5, -11/*mean (0.242949), correlation (0.34145)*/,
    -11, -13, -5, -13/*mean (0.244028), correlation (0.336861)*/,
    -1, 6, 0, -1/*mean (0.247571), correlation (0.343684)*/,
    5, -3, 5, 2/*mean (0.000697256), correlation (0.357265)*/,
    -4, -13, -4, 12/*mean (0.00213675), correlation (0.373827)*/,
    -9, -6, -9, 6/*mean (0.0126856), correlation (0.373938)*/,
    -12, -10, -8, -4/*mean (0.0152497), correlation (0.364237)*/,
    10, 2, 12, -3/*mean (0.0299933), correlation (0.345292)*/,
    7, 12, 12, 12/*mean (0.0307242), correlation (0.366299)*/,
    -7, -13, -6, 5/*mean (0.0534975), correlation (0.368357)*/,
    -4, 9, -3, 4/*mean (0.099865), correlation (0.372276)*/,
    7, -1, 12, 2/*mean (0.117083), correlation (0.364529)*/,
    -7, 6, -5, 1/*mean (0.126125), correlation (0.369606)*/,
    -13, 11, -12, 5/*mean (0.130364), correlation (0.358502)*/,
    -3, 7, -2, -6/*mean (0.131691), correlation (0.375531)*/,
    7, -8, 12, -7/*mean (0.160166), correlation (0.379508)*/,
    -13, -7, -11, -12/*mean (0.167848), correlation (0.353343)*/,
    1, -3, 12, 12/*mean (0.183378), correlation (0.371916)*/,
    2, -6, 3, 0/*mean (0.228711), correlation (0.371761)*/,
    -4, 3, -2, -13/*mean (0.247211), correlation (0.364063)*/,
    -1, -13, 1, 9/*mean (0.249325), correlation (0.378139)*/,
    7, 1, 8, -6/*mean (0.000652272), correlation (0.411682)*/,
    1, -1, 3, 12/*mean (0.00248538), correlation (0.392988)*/,
    9, 1, 12, 6/*mean (0.0206815), correlation (0.386106)*/,
    -1, -9, -1, 3/*mean (0.0364485), correlation (0.410752)*/,
    -13, -13, -10, 5/*mean (0.0376068), correlation (0.398374)*/,
    7, 7, 10, 12/*mean (0.0424202), correlation (0.405663)*/,
    12, -5, 12, 9/*mean (0.0942645), correlation (0.410422)*/,
    6, 3, 7, 11/*mean (0.1074), correlation (0.413224)*/,
    5, -13, 6, 10/*mean (0.109256), correlation (0.408646)*/,
    2, -12, 2, 3/*mean (0.131691), correlation (0.416076)*/,
    3, 8, 4, -6/*mean (0.165081), correlation (0.417569)*/,
    2, 6, 12, -13/*mean (0.171874), correlation (0.408471)*/,
    9, -12, 10, 3/*mean (0.175146), correlation (0.41296)*/,
    -8, 4, -7, 9/*mean (0.183682), correlation (0.402956)*/,
    -11, 12, -4, -6/*mean (0.184672), correlation (0.416125)*/,
    1, 12, 2, -8/*mean (0.191487), correlation (0.386696)*/,
    6, -9, 7, -4/*mean (0.192668), correlation (0.394771)*/,
    2, 3, 3, -2/*mean (0.200157), correlation (0.408303)*/,
    6, 3, 11, 0/*mean (0.204588), correlation (0.411762)*/,
    3, -3, 8, -8/*mean (0.205904), correlation (0.416294)*/,
    7, 8, 9, 3/*mean (0.213237), correlation (0.409306)*/,
    -11, -5, -6, -4/*mean (0.243444), correlation (0.395069)*/,
    -10, 11, -5, 10/*mean (0.247672), correlation (0.413392)*/,
    -5, -8, -3, 12/*mean (0.24774), correlation (0.411416)*/,
    -10, 5, -9, 0/*mean (0.00213675), correlation (0.454003)*/,
    8, -1, 12, -6/*mean (0.0293635), correlation (0.455368)*/,
    4, -6, 6, -11/*mean (0.0404971), correlation (0.457393)*/,
    -10, 12, -8, 7/*mean (0.0481107), correlation (0.448364)*/,
    4, -2, 6, 7/*mean (0.050641), correlation (0.455019)*/,
    -2, 0, -2, 12/*mean (0.0525978), correlation (0.44338)*/,
    -5, -8, -5, 2/*mean (0.0629667), correlation (0.457096)*/,
    7, -6, 10, 12/*mean (0.0653846), correlation (0.445623)*/,
    -9, -13, -8, -8/*mean (0.0858749), correlation (0.449789)*/,
    -5, -13, -5, -2/*mean (0.122402), correlation (0.450201)*/,
    8, -8, 9, -13/*mean (0.125416), correlation (0.453224)*/,
    -9, -11, -9, 0/*mean (0.130128), correlation (0.458724)*/,
    1, -8, 1, -2/*mean (0.132467), correlation (0.440133)*/,
    7, -4, 9, 1/*mean (0.132692), correlation (0.454)*/,
    -2, 1, -1, -4/*mean (0.135695), correlation (0.455739)*/,
    11, -6, 12, -11/*mean (0.142904), correlation (0.446114)*/,
    -12, -9, -6, 4/*mean (0.146165), correlation (0.451473)*/,
    3, 7, 7, 12/*mean (0.147627), correlation (0.456643)*/,
    5, 5, 10, 8/*mean (0.152901), correlation (0.455036)*/,
    0, -4, 2, 8/*mean (0.167083), correlation (0.459315)*/,
    -9, 12, -5, -13/*mean (0.173234), correlation (0.454706)*/,
    0, 7, 2, 12/*mean (0.18312), correlation (0.433855)*/,
    -1, 2, 1, 7/*mean (0.185504), correlation (0.443838)*/,
    5, 11, 7, -9/*mean (0.185706), correlation (0.451123)*/,
    3, 5, 6, -8/*mean (0.188968), correlation (0.455808)*/,
    -13, -4, -8, 9/*mean (0.191667), correlation (0.459128)*/,
    -5, 9, -3, -3/*mean (0.193196), correlation (0.458364)*/,
    -4, -7, -3, -12/*mean (0.196536), correlation (0.455782)*/,
    6, 5, 8, 0/*mean (0.1972), correlation (0.450481)*/,
    -7, 6, -6, 12/*mean (0.199438), correlation (0.458156)*/,
    -13, 6, -5, -2/*mean (0.211224), correlation (0.449548)*/,
    1, -10, 3, 10/*mean (0.211718), correlation (0.440606)*/,
    4, 1, 8, -4/*mean (0.213034), correlation (0.443177)*/,
    -2, -2, 2, -13/*mean (0.234334), correlation (0.455304)*/,
    2, -12, 12, 12/*mean (0.235684), correlation (0.443436)*/,
    -2, -13, 0, -6/*mean (0.237674), correlation (0.452525)*/,
    4, 1, 9, 3/*mean (0.23962), correlation (0.444824)*/,
    -6, -10, -3, -5/*mean (0.248459), correlation (0.439621)*/,
    -3, -13, -1, 1/*mean (0.249505), correlation (0.456666)*/,
    7, 5, 12, -11/*mean (0.00119208), correlation (0.495466)*/,
    4, -2, 5, -7/*mean (0.00372245), correlation (0.484214)*/,
    -13, 9, -9, -5/*mean (0.00741116), correlation (0.499854)*/,
    7, 1, 8, 6/*mean (0.0208952), correlation (0.499773)*/,
    7, -8, 7, 6/*mean (0.0220085), correlation (0.501609)*/,
    -7, -4, -7, 1/*mean (0.0233806), correlation (0.496568)*/,
    -8, 11, -7, -8/*mean (0.0236505), correlation (0.489719)*/,
    -13, 6, -12, -8/*mean (0.0268781), correlation (0.503487)*/,
    2, 4, 3, 9/*mean (0.0323324), correlation (0.501938)*/,
    10, -5, 12, 3/*mean (0.0399235), correlation (0.494029)*/,
    -6, -5, -6, 7/*mean (0.0420153), correlation (0.486579)*/,
    8, -3, 9, -8/*mean (0.0548021), correlation (0.484237)*/,
    2, -12, 2, 8/*mean (0.0616622), correlation (0.496642)*/,
    -11, -2, -10, 3/*mean (0.0627755), correlation (0.498563)*/,
    -12, -13, -7, -9/*mean (0.0829622), correlation (0.495491)*/,
    -11, 0, -10, -5/*mean (0.0843342), correlation (0.487146)*/,
    5, -3, 11, 8/*mean (0.0929937), correlation (0.502315)*/,
    -2, -13, -1, 12/*mean (0.113327), correlation (0.48941)*/,
    -1, -8, 0, 9/*mean (0.132119), correlation (0.467268)*/,
    -13, -11, -12, -5/*mean (0.136269), correlation (0.498771)*/,
    -10, -2, -10, 11/*mean (0.142173), correlation (0.498714)*/,
    -3, 9, -2, -13/*mean (0.144141), correlation (0.491973)*/,
    2, -3, 3, 2/*mean (0.14892), correlation (0.500782)*/,
    -9, -13, -4, 0/*mean (0.150371), correlation (0.498211)*/,
    -4, 6, -3, -10/*mean (0.152159), correlation (0.495547)*/,
    -4, 12, -2, -7/*mean (0.156152), correlation (0.496925)*/,
    -6, -11, -4, 9/*mean (0.15749), correlation (0.499222)*/,
    6, -3, 6, 11/*mean (0.159211), correlation (0.503821)*/,
    -13, 11, -5, 5/*mean (0.162427), correlation (0.501907)*/,
    11, 11, 12, 6/*mean (0.16652), correlation (0.497632)*/,
    7, -5, 12, -2/*mean (0.169141), correlation (0.484474)*/,
    -1, 12, 0, 7/*mean (0.169456), correlation (0.495339)*/,
    -4, -8, -3, -2/*mean (0.171457), correlation (0.487251)*/,
    -7, 1, -6, 7/*mean (0.175), correlation (0.500024)*/,
    -13, -12, -8, -13/*mean (0.175866), correlation (0.497523)*/,
    -7, -2, -6, -8/*mean (0.178273), correlation (0.501854)*/,
    -8, 5, -6, -9/*mean (0.181107), correlation (0.494888)*/,
    -5, -1, -4, 5/*mean (0.190227), correlation (0.482557)*/,
    -13, 7, -8, 10/*mean (0.196739), correlation (0.496503)*/,
    1, 5, 5, -13/*mean (0.19973), correlation (0.499759)*/,
    1, 0, 10, -13/*mean (0.204465), correlation (0.49873)*/,
    9, 12, 10, -1/*mean (0.209334), correlation (0.49063)*/,
    5, -8, 10, -9/*mean (0.211134), correlation (0.503011)*/,
    -1, 11, 1, -13/*mean (0.212), correlation (0.499414)*/,
    -9, -3, -6, 2/*mean (0.212168), correlation (0.480739)*/,
    -1, -10, 1, 12/*mean (0.212731), correlation (0.502523)*/,
    -13, 1, -8, -10/*mean (0.21327), correlation (0.489786)*/,
    8, -11, 10, -6/*mean (0.214159), correlation (0.488246)*/,
    2, -13, 3, -6/*mean (0.216993), correlation (0.50287)*/,
    7, -13, 12, -9/*mean (0.223639), correlation (0.470502)*/,
    -10, -10, -5, -7/*mean (0.224089), correlation (0.500852)*/,
    -10, -8, -8, -13/*mean (0.228666), correlation (0.502629)*/,
    4, -6, 8, 5/*mean (0.22906), correlation (0.498305)*/,
    3, 12, 8, -13/*mean (0.233378), correlation (0.503825)*/,
    -4, 2, -3, -3/*mean (0.234323), correlation (0.476692)*/,
    5, -13, 10, -12/*mean (0.236392), correlation (0.475462)*/,
    4, -13, 5, -1/*mean (0.236842), correlation (0.504132)*/,
    -9, 9, -4, 3/*mean (0.236977), correlation (0.497739)*/,
    0, 3, 3, -9/*mean (0.24314), correlation (0.499398)*/,
    -12, 1, -6, 1/*mean (0.243297), correlation (0.489447)*/,
    3, 2, 4, -8/*mean (0.00155196), correlation (0.553496)*/,
    -10, -10, -10, 9/*mean (0.00239541), correlation (0.54297)*/,
    8, -13, 12, 12/*mean (0.0034413), correlation (0.544361)*/,
    -8, -12, -6, -5/*mean (0.003565), correlation (0.551225)*/,
    2, 2, 3, 7/*mean (0.00835583), correlation (0.55285)*/,
    10, 6, 11, -8/*mean (0.00885065), correlation (0.540913)*/,
    6, 8, 8, -12/*mean (0.0101552), correlation (0.551085)*/,
    -7, 10, -6, 5/*mean (0.0102227), correlation (0.533635)*/,
    -3, -9, -3, 9/*mean (0.0110211), correlation (0.543121)*/,
    -1, -13, -1, 5/*mean (0.0113473), correlation (0.550173)*/,
    -3, -7, -3, 4/*mean (0.0140913), correlation (0.554774)*/,
    -8, -2, -8, 3/*mean (0.017049), correlation (0.55461)*/,
    4, 2, 12, 12/*mean (0.01778), correlation (0.546921)*/,
    2, -5, 3, 11/*mean (0.0224022), correlation (0.549667)*/,
    6, -9, 11, -13/*mean (0.029161), correlation (0.546295)*/,
    3, -1, 7, 12/*mean (0.0303081), correlation (0.548599)*/,
    11, -1, 12, 4/*mean (0.0355151), correlation (0.523943)*/,
    -3, 0, -3, 6/*mean (0.0417904), correlation (0.543395)*/,
    4, -11, 4, 12/*mean (0.0487292), correlation (0.542818)*/,
    2, -4, 2, 1/*mean (0.0575124), correlation (0.554888)*/,
    -10, -6, -8, 1/*mean (0.0594242), correlation (0.544026)*/,
    -13, 7, -11, 1/*mean (0.0597391), correlation (0.550524)*/,
    -13, 12, -11, -13/*mean (0.0608974), correlation (0.55383)*/,
    6, 0, 11, -13/*mean (0.065126), correlation (0.552006)*/,
    0, -1, 1, 4/*mean (0.074224), correlation (0.546372)*/,
    -13, 3, -9, -2/*mean (0.0808592), correlation (0.554875)*/,
    -9, 8, -6, -3/*mean (0.0883378), correlation (0.551178)*/,
    -13, -6, -8, -2/*mean (0.0901035), correlation (0.548446)*/,
    5, -9, 8, 10/*mean (0.0949843), correlation (0.554694)*/,
    2, 7, 3, -9/*mean (0.0994152), correlation (0.550979)*/,
    -1, -6, -1, -1/*mean (0.10045), correlation (0.552714)*/,
    9, 5, 11, -2/*mean (0.100686), correlation (0.552594)*/,
    11, -3, 12, -8/*mean (0.101091), correlation (0.532394)*/,
    3, 0, 3, 5/*mean (0.101147), correlation (0.525576)*/,
    -1, 4, 0, 10/*mean (0.105263), correlation (0.531498)*/,
    3, -6, 4, 5/*mean (0.110785), correlation (0.540491)*/,
    -13, 0, -10, 5/*mean (0.112798), correlation (0.536582)*/,
    5, 8, 12, 11/*mean (0.114181), correlation (0.555793)*/,
    8, 9, 9, -6/*mean (0.117431), correlation (0.553763)*/,
    7, -4, 8, -12/*mean (0.118522), correlation (0.553452)*/,
    -10, 4, -10, 9/*mean (0.12094), correlation (0.554785)*/,
    7, 3, 12, 4/*mean (0.122582), correlation (0.555825)*/,
    9, -7, 10, -2/*mean (0.124978), correlation (0.549846)*/,
    7, 0, 12, -2/*mean (0.127002), correlation (0.537452)*/,
    -1, -6, 0, -11/*mean (0.127148), correlation (0.547401)*/
  };
  
  // compute the descriptor
  void ComputeORB(const cv::Mat &img, vector<cv::KeyPoint> &keypoints, vector<DescType> &descriptors) {
    const int half_patch_size = 8;
    const int half_boundary = 16;
    int bad_points = 0;
    for (auto &kp: keypoints) {
      if (kp.pt.x < half_boundary || kp.pt.y < half_boundary ||
          kp.pt.x >= img.cols - half_boundary || kp.pt.y >= img.rows - half_boundary) {
        // outside
        bad_points++;
        descriptors.push_back({});
        continue;
      }
  
      float m01 = 0, m10 = 0;
      for (int dx = -half_patch_size; dx < half_patch_size; ++dx) {
        for (int dy = -half_patch_size; dy < half_patch_size; ++dy) {
          uchar pixel = img.at<uchar>(kp.pt.y + dy, kp.pt.x + dx);
          m10 += dx * pixel;
          m01 += dy * pixel;
        }
      }
  
      // angle should be arc tan(m01/m10);
      float m_sqrt = sqrt(m01 * m01 + m10 * m10) + 1e-18; // avoid divide by zero
      float sin_theta = m01 / m_sqrt;
      float cos_theta = m10 / m_sqrt;
  
      // compute the angle of this point
      // 8个32位unsigned int数据
      DescType desc(8, 0);
      // 计算fast特征点的角度
      for (int i = 0; i < 8; i++) {
        uint32_t d = 0;
        for (int k = 0; k < 32; k++) {
          int idx_pq = i * 32 + k;
          cv::Point2f p(ORB_pattern[idx_pq * 4], ORB_pattern[idx_pq * 4 + 1]);
          cv::Point2f q(ORB_pattern[idx_pq * 4 + 2], ORB_pattern[idx_pq * 4 + 3]);
  
          // rotate with theta
          cv::Point2f pp = cv::Point2f(cos_theta * p.x - sin_theta * p.y, sin_theta * p.x + cos_theta * p.y)
                           + kp.pt;
          cv::Point2f qq = cv::Point2f(cos_theta * q.x - sin_theta * q.y, sin_theta * q.x + cos_theta * q.y)
                           + kp.pt;
          if (img.at<uchar>(pp.y, pp.x) < img.at<uchar>(qq.y, qq.x)) {
            d |= 1 << k;
          }
        }
        desc[i] = d;
      }
      descriptors.push_back(desc);
    }
  
    cout << "bad/total: " << bad_points << "/" << keypoints.size() << endl;
  }
  
  // brute-force matching
  void BfMatch(const vector<DescType> &desc1, const vector<DescType> &desc2, vector<cv::DMatch> &matches) {
    const int d_max = 40;
  
    for (size_t i1 = 0; i1 < desc1.size(); ++i1) {
      if (desc1[i1].empty()) continue;
      cv::DMatch m{i1, 0, 256};
      for (size_t i2 = 0; i2 < desc2.size(); ++i2) {
        if (desc2[i2].empty()) continue;
        int distance = 0;
        // 使用SSE指令集中的_mm_popcnt_u32函数计算unsigned int变量中1的个数，从而达到计算汉明距离的效果
        for (int k = 0; k < 8; k++) {
          distance += _mm_popcnt_u32(desc1[i1][k] ^ desc2[i2][k]);
        }
        if (distance < d_max && distance < m.distance) {
          m.distance = distance;
          m.trainIdx = i2;
        }
      }
      if (m.distance < d_max) {
        matches.push_back(m);
      }
    }
  }
  ```

  

- 编译

  ```cmake
  cmake_minimum_required(VERSION 3.0)
  project(test)
  
  set(CMAKE_BUILD_TYPE Release)
  set(CMAKE_CXX_FLAGS "-std=c++14 -mfma")
  
  # OpenCV
  find_package(OpenCV REQUIRED)
  include_directories(${OpenCV_INCLUDE_DIRS})
  
  add_executable(test test.cpp)
  # target_link_libraries(test ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
  target_link_libraries(test ${OpenCV_LIBS})
  ```

### 2.3 计算相机运动

有了匹配点后，就要根据点来估计相机的运动，针对不同情况有不同的方法：

1. 单目相机：

   单目相机只能得到2D的像素坐标，所以使用**对极几何**根据**两组2D点**来估计运动。

2. 双目/RGB-D

   得到**两组3D点**，使用**ICP**来解决。

3. 一个单目+一个双目/RGB-D

   **一组3D，一组2D**，使用**PnP**解决

## 3.2D-2D：对极几何

对极约束$x_2^Tt^{\wedge}Rx_1=0$简介描述了2个匹配点的空间位置关系，所以相机位姿估计问题变成以下2步：

1. 根据配对点的像素位置求出基础矩阵(Fundamental Matrix)$F$ 或者 本质矩阵(Essential Matrix)$E$
2. 根据$E$或$F$,求出旋转矩阵$R$和位移向量$t$

**单目相机的初始化**：在单目相机中，t也即相机位移会归一化为1，因为下面SVD算出来的t是0.8米还是0.8厘米我们是不知道的。所以用单目相机必须第一步平移一小段距离，并将其定为单位1。之后的轨迹和地图都将以这个平移的距离为单位。

- **本质矩阵E**：
  
  1. 通过八点算法来求解E，见[[Comupter Vision#3.Acht-Punkt-Algorithmus|笔记]]
  
  2. 使用SVD奇异值分解$E=U\Sigma V^T$，恢复出R和t
  
     任意E有两个可能的t，R与它对应
     $$
     \begin{align}
     t_1^\wedge&=UR_Z(\frac{\pi}{2})\Sigma U^T,\ R_1=UR_Z^T(\frac{\pi}{2})V^T \\
     t_2^\wedge&=UR_Z(-\frac{\pi}{2})\Sigma U^T,\ R_2=UR_Z^T(-\frac{\pi}{2})V^T
     
     \end{align}
     $$
  
     - $R_Z(\frac{\pi}{2})$:表示沿Z轴旋转90°得到的旋转矩阵
  
     - 由于-E和E等价，所以一共存在4个可能的解
  
     - 但只有一个解中 物体所在点P 在2个相机中都具有正的深度。
  
       所以只要把任意点代入4个解中，计算该点的深度，若2个都为正，那么该解就是正确的了。
  
     - 根据八点法中线性方程解出的$E$,可能不满足E的内在性质：奇异值为$\sigma,\sigma,0$的形式
  
       这时SVD得到奇异值为$\Sigma=diag(\sigma_1,\sigma_2,\sigma_3)$,为此我们近似取为：$(1,1,0)或(\frac{\sigma_1+\sigma_2}{2}, \frac{\sigma_1+\sigma_2}{2},0)$
  
- **基本矩阵F**:

  和本质矩阵E只差一个内参，即F用于未标定的相机，E用于标定过的相机。见[[Comupter Vision#5.Die Fundamentalmatrix|笔记]]
  
- **单应矩阵H**:

  如果某场景中所有特征点都落在同一个平面（墙壁，地面等），就可以使用单应矩阵来进行运动估计。
  
  可以通过四点法来求解H，见[[Comupter Vision#2.Vier-Punkt-Algorithmus|笔记]]

当特征点共面或相机单纯旋转时，自由度下降（即退化degenerate）,而E/F中多出来的自由度就会主要由噪音决定。为了避免退化现象造成的影响，SLAM中会同时估计E/F 和 H，哪个最后误差小就用哪个。

## 4.实践：对极几何

对极约束求解相机运动

- 代码

  ```c++
  #include <iostream>
  #include <opencv2/core/core.hpp>
  #include <opencv2/features2d/features2d.hpp>
  #include <opencv2/highgui/highgui.hpp>
  #include <opencv2/calib3d/calib3d.hpp>
  // #include "extra.h" // use this if in OpenCV2
  
  using namespace std;
  using namespace cv;
  
  /****************************************************
   * 本程序演示了如何使用2D-2D的特征匹配估计相机运动
   * **************************************************/
  
  void find_feature_matches(
    const Mat &img_1, const Mat &img_2,
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch> &matches);
  
  void pose_estimation_2d2d(
    std::vector<KeyPoint> keypoints_1,
    std::vector<KeyPoint> keypoints_2,
    std::vector<DMatch> matches,
    Mat &R, Mat &t);
  
  // 像素坐标转相机归一化坐标
  Point2d pixel2cam(const Point2d &p, const Mat &K);
  
  int main(int argc, char **argv) {
    if (argc != 3) {
      cout << "usage: pose_estimation_2d2d img1 img2" << endl;
      return 1;
    }
    //-- 读取图像
    Mat img_1 = imread(argv[1], IMREAD_COLOR);
    Mat img_2 = imread(argv[2], IMREAD_COLOR);
    assert(img_1.data && img_2.data && "Can not load images!");
    
    //-- 寻找特征点
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;
  
    //-- 估计两张图像间运动
    Mat R, t;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);
  
    //-- 验证E=t^R*scale
    Mat t_x =
      (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
        t.at<double>(2, 0), 0, -t.at<double>(0, 0),
        -t.at<double>(1, 0), t.at<double>(0, 0), 0);
  
    cout << "t^R=" << endl << t_x * R << endl;
  
    //-- 验证对极约束
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for (DMatch m: matches) {
      Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
      Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
      Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
      Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
      Mat d = y2.t() * t_x * R * y1;
      cout << "epipolar constraint = " << d << endl;
    }
    return 0;
  }
  
  void find_feature_matches(const Mat &img_1, const Mat &img_2,
                            std::vector<KeyPoint> &keypoints_1,
                            std::vector<KeyPoint> &keypoints_2,
                            std::vector<DMatch> &matches) {
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);
  
    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);
  
    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descriptors_1, descriptors_2, match);
  
    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;
  
    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++) {
      double dist = match[i].distance;
      if (dist < min_dist) min_dist = dist;
      if (dist > max_dist) max_dist = dist;
    }
  
    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);
  
    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++) {
      if (match[i].distance <= max(2 * min_dist, 30.0)) {
        matches.push_back(match[i]);
      }
    }
  }
  
  Point2d pixel2cam(const Point2d &p, const Mat &K) {
    return Point2d
      (
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
      );
  }
  
  void pose_estimation_2d2d(std::vector<KeyPoint> keypoints_1,
                            std::vector<KeyPoint> keypoints_2,
                            std::vector<DMatch> matches,
                            Mat &R, Mat &t) {
    // 相机内参,TUM Freiburg2
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  
    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;
  
    for (int i = 0; i < (int) matches.size(); i++) {
      points1.push_back(keypoints_1[matches[i].queryIdx].pt);
      points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }
  
    //-- 计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat(points1, points2, FM_8POINT);
    cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;
  
    //-- 计算本质矩阵
    Point2d principal_point(325.1, 249.7);  //相机光心, TUM dataset标定值
    double focal_length = 521;      //相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
    cout << "essential_matrix is " << endl << essential_matrix << endl;
  
    //-- 计算单应矩阵
    //-- 但是本例中场景不是平面，单应矩阵意义不大
    Mat homography_matrix;
    homography_matrix = findHomography(points1, points2, RANSAC, 3);
    cout << "homography_matrix is " << endl << homography_matrix << endl;
  
    //-- 从本质矩阵中恢复旋转和平移信息.
    // 得到的R，t有4种可能性，openCV会帮我们检测角点是否为正，从而选出正确的解
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    cout << "R is " << endl << R << endl;
    cout << "t is " << endl << t << endl;
  
  }
  
  ```

  

- 编译

  ```cmake
  cmake_minimum_required(VERSION 3.0)
  project(test)
  
  set(CMAKE_BUILD_TYPE Release)
  set(CMAKE_CXX_FLAGS "-std=c++14 -mfma") 
  
  # OpenCV
  find_package(OpenCV REQUIRED)
  include_directories(${OpenCV_INCLUDE_DIRS})
  
  add_executable(test test.cpp)
  target_link_libraries(test ${OpenCV_LIBS})
  ```

  

## 5.单目：三角测量得到深度

- 基本概念

  上面的对极几何是用2个相机同时拍摄同一物体，得到位姿R和t。然后就可以用**三角测量Triangulation**从而推断出路标点的距离。

  - 方程：$s_2x_2^{\wedge}x_2=0=s_1x_2^{\wedge}Rx_1+x_2^{\wedge}t$ 可以先求第二个位置时的深度$s_2$，然后再求第一个位置的深度$s_1$。（深度十相机坐标系下该点到成像平面的距离。）

    由于噪声的存在，通常不会直接算此方程， 而是求解最小二乘解。

  - 三角化精度提升的两大方法：

    由于像素上的不确定性，会导致较大的深度不确定，降低精度。

    1. 提高特征点的提取精度，即提高图像分辨率，但会增加计算成本。
    2. 增大平移量t。但会导致图像外观发生变化，导致特征匹配失败。这也被称为三角化的矛盾或视差(parallax)

- 代码：

  ```c++
  #include <iostream>
  #include <opencv2/opencv.hpp>
  // #include "extra.h" // used in opencv2
  using namespace std;
  using namespace cv;
  
  // 1. 寻找匹配点
  void find_feature_matches(
    const Mat &img_1, const Mat &img_2,
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch> &matches);
  
  // 2. 根据匹配点算出位姿R 和 t
  void pose_estimation_2d2d(
    const std::vector<KeyPoint> &keypoints_1,
    const std::vector<KeyPoint> &keypoints_2,
    const std::vector<DMatch> &matches,
    Mat &R, Mat &t);
  
  // 3. 使用三角测量算出深度
  void triangulation(
    const vector<KeyPoint> &keypoint_1,
    const vector<KeyPoint> &keypoint_2,
    const std::vector<DMatch> &matches,
    const Mat &R, const Mat &t,
    vector<Point3d> &points
  );
  
  /// 作图用
  inline cv::Scalar get_color(float depth) {
    float up_th = 50, low_th = 10, th_range = up_th - low_th;
    if (depth > up_th) depth = up_th;
    if (depth < low_th) depth = low_th;
    return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
  }
  
  // 像素坐标转相机归一化坐标
  Point2f pixel2cam(const Point2d &p, const Mat &K);
  
  int main(int argc, char **argv) {
    if (argc != 3) {
      cout << "usage: triangulation img1 img2" << endl;
      return 1;
    }
    //-- 读取图像
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
  
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;
  
    //-- 估计两张图像间运动
    Mat R, t;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);
  
    //-- 三角化
    vector<Point3d> points;
    triangulation(keypoints_1, keypoints_2, matches, R, t, points);
  
    //-- 验证三角化点与特征点的重投影关系
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    Mat img1_plot = img_1.clone();
    Mat img2_plot = img_2.clone();
    for (int i = 0; i < matches.size(); i++) {
      // 第一个图
      float depth1 = points[i].z;
      cout << "depth: " << depth1 << endl;
      Point2d pt1_cam = pixel2cam(keypoints_1[matches[i].queryIdx].pt, K);
      cv::circle(img1_plot, keypoints_1[matches[i].queryIdx].pt, 2, get_color(depth1), 2);
  
      // 第二个图
      Mat pt2_trans = R * (Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
      float depth2 = pt2_trans.at<double>(2, 0);
      cv::circle(img2_plot, keypoints_2[matches[i].trainIdx].pt, 2, get_color(depth2), 2);
    }
    cv::imshow("img 1", img1_plot);
    cv::imshow("img 2", img2_plot);
    cv::waitKey();
  
    return 0;
  }
  
  void find_feature_matches(const Mat &img_1, const Mat &img_2,
                            std::vector<KeyPoint> &keypoints_1,
                            std::vector<KeyPoint> &keypoints_2,
                            std::vector<DMatch> &matches) {
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);
  
    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);
  
    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descriptors_1, descriptors_2, match);
  
    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;
  
    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++) {
      double dist = match[i].distance;
      if (dist < min_dist) min_dist = dist;
      if (dist > max_dist) max_dist = dist;
    }
  
    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);
  
    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++) {
      if (match[i].distance <= max(2 * min_dist, 30.0)) {
        matches.push_back(match[i]);
      }
    }
  }
  
  void pose_estimation_2d2d(
    const std::vector<KeyPoint> &keypoints_1,
    const std::vector<KeyPoint> &keypoints_2,
    const std::vector<DMatch> &matches,
    Mat &R, Mat &t) {
    // 相机内参,TUM Freiburg2
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  
    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;
  
    for (int i = 0; i < (int) matches.size(); i++) {
      points1.push_back(keypoints_1[matches[i].queryIdx].pt);
      points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }
  
    //-- 计算本质矩阵
    Point2d principal_point(325.1, 249.7);        //相机主点, TUM dataset标定值
    int focal_length = 521;            //相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
  
    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
  }
  
  void triangulation(
    const vector<KeyPoint> &keypoint_1,
    const vector<KeyPoint> &keypoint_2,
    const std::vector<DMatch> &matches,
    const Mat &R, const Mat &t,
    vector<Point3d> &points) {
    Mat T1 = (Mat_<float>(3, 4) <<
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0);
    Mat T2 = (Mat_<float>(3, 4) <<
      R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
      R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
      R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
    );
  
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point2f> pts_1, pts_2;
    for (DMatch m:matches) {
      // 将像素坐标转换至相机坐标
      pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K));
      pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt, K));
    }
  
    Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
  
    // 转换成非齐次坐标
    for (int i = 0; i < pts_4d.cols; i++) {
      Mat x = pts_4d.col(i);
      x /= x.at<float>(3, 0); // 归一化
      Point3d p(
        x.at<float>(0, 0),
        x.at<float>(1, 0),
        x.at<float>(2, 0)
      );
      points.push_back(p);
    }
  }
  
  Point2f pixel2cam(const Point2d &p, const Mat &K) {
    return Point2f
      (
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
      );
  }
  
  ```

  

- 编译：

  ```cmake
  cmake_minimum_required(VERSION 3.0)
  project(test)
  
  set(CMAKE_BUILD_TYPE Release)
  set(CMAKE_CXX_FLAGS "-std=c++14 -mfma") 
  
  # OpenCV
  find_package(OpenCV REQUIRED)
  include_directories(${OpenCV_INCLUDE_DIRS})
  
  add_executable(test test.cpp)
  target_link_libraries(test ${OpenCV_LIBS})
  ```



## 6.3D-2D:PnP

PnP(Perspective-n-Point)是求解3D到2D点对运动的方法，它描述了**当知道n个3D空间点及其投影位置时，如何估计相机的位姿。**

相比2D-2D的对极几何方法至少要用8个点，如果两张图像中一张特征点深度已知，那么最少只需3个点对。双目/RGB可直接使用PnP方法估计相机运动，但单目相机就要用三角测量先得到深度再用PnP。

PnP是最重要的姿态估计方法，有多种求解方法：

1. 用3对点估计位姿的P3P

2. 直接线性变化DLT

3. EPnP(Efficient PnP)

4. UPnP

5. 光束法平差(Bundle Adjustment,BA)

   这是一种非线性优化方法，通过构建最小二乘问题并迭代求解

### 6.1 直接线性变换DLT

第一步：定义矩阵

​	有一空间$P=(X,Y,Z,1)^T$,其在图像I中投影到特征点$x_1=(u_1,v_1,1)^T$上，由此列出：
$$
s\left (\begin{array}{cccc}
u_1 \\
v_1  \\
1 \\
\end{array}\right)=
\left (\begin{array}{cccc}
t_1 & t_2 & t_3 & t_4 \\
t_5 & t_6 & t_7 & t_8  \\
t_9 & t_{10} & t_{11} & t_{12} \\
\end{array}\right)
\left (\begin{array}{cccc}
X \\
Y \\
Z \\
1
\end{array}\right)\tag{1}
$$
第二步：用最后一行消去深度s,得到两个约束：
$$
\begin{align}
u_1&=\frac{t_1X+t_2Y+t_3Z+t_4}{t_9X+t_{10}Y+t_{11}Z+t_{12}}\\
\\
v_1&=\frac{t_5X+t_6Y+t_7Z+t_8}{t_9X+t_{10}Y+t_{11}Z+t_{12}}
\end{align}
\tag{2}
$$
第三步：用$\mathbf{t_1}=(t_1,t_2,t_3,t_4)^T,\mathbf{t_2}=(t_5,t_6,t_7,t_8)^T,\mathbf{t_3}=(t_9,t_{10},t_{11},t_{12})^T$简化2式得到：
$$
\begin{align}
\mathbf{t}_1^T\mathbf{P}-\mathbf{t}_3^T\mathbf{P}u_1&=0\\
\mathbf{t}_2^T\mathbf{P}-\mathbf{t}_3^T\mathbf{P}v_1&=0
\end{align}
\tag{3}
$$
由于参数t一共有12维，所以至少需要6对匹配点去计算3式。当匹配点大于6对时，也可以和8点法4点法一样，用SVD求最小二乘解。



### 6.2 P3P

P3P使用3对匹配点，来解PnP问题。这里的3D点是世界坐标系下的坐标，如果是相机坐标系下的坐标，就是ICP问题而非PnP问题了。

在SLAM中通常先用P3P/EPnP等方法估计相机位姿，在构建最小二乘优化问题（BA)对估计值进行调整。



### 6.3 最小化重投影误差(BA,非线性优化法)

重投影误差示意图：

SLAM中一个量带帽子的符号通常表示该量是一个估计值或预测值，而不是准确的真实值。

![image-20230613015320005](https://cdn.jsdelivr.net/gh/Fernweh-yang/ImageHosting@main/img/image-20230613015320005.png)

- **重投影误差**指的真实三维空间点在图像平面上的投影（也就是图像上的像素点）和重投影（其实是用我们的计算值得到的虚拟的像素点）的差值。

  - $p_2$：真值，由2帧图的特征匹配得到。首先会在第一张图像（左相机）中提取特征点。然后，在第二张图像（右相机）中寻找与第一张图像（左相机）中特征点具有相同特征的点。如果找到了匹配的特征点，那么这两个特征点在两张图像中的像素坐标就可以作为重投影误差的真实值。
  
  - $\hat{p}_2$：估计值，由投影模型计算得到的像素坐标。投影模型中用来计算的世界坐标系到右相机的坐标变换(即相机位姿**T**)和空间点**P**就是我们要优化的目标。
  
- 非线性方法与线性方法的区别：

  - 线性：先求相机位姿，再求空间点位置
  - 非线性：将相机位姿和三维点都看成优化变量一起最小化。这种方法也被称为**光束法平差(Bundle Adjustment,BA)**
    - 这是一种通用的求解方式，可以用它对PnP或ICP给出的结果进行优化。
    - 这里我们先求出PnP的结果，然后用BA对这个结果进行优化。
    - 在第九讲中有讲：当相机是连续运动的（大部分SLAM过程），可以直接用BA求解相机位姿。

- 考虑n个三维空间点$\mathbf{P}_i=[X_i,Y_i,Z_i]^T$及其投影$p$,投影坐标为$\mathbf{u}_i=[u_i,v_i]^T$,希望计算相机的位姿$R,t$（李群表示为$T$）

  根据第五讲，像素位置和空间点的位置关系为：
  $$
  s_i\mathbf{u}_i=\mathbf{KTP}_i \tag{1}
  $$
  由于噪声1式左右不会完全相等，有个误差，因此利用这个误差，将误差求和构建最小二乘问题：
  $$
  \mathbf{T}^*=arg\ \mathop{min}_{\mathbf{T}}\frac{1}{2}\sum_{i=1}^n|| \mathbf{u}_i-\frac{1}{s_i}\mathbf{KTP}_i ||_2^2 \tag{2}
  $$
  2式中的误差项称为**重投影误差(Reprojection error) e**：3D点的投影位置与观测位置的差
  $$
  \mathbf{e} = \mathbf{u}_i-\frac{1}{s_i}\mathbf{KTP}_i \tag{3}
  $$
  求解2式可以用第六讲中的高斯牛顿法、L-M法，但需要先知道每个误差关于优化变量的导数：

  1. 重投影误差关于相机位姿的导数（**优化位姿**）

     - 利用链式法则和扰动模型：
       $$
       \frac{\partial\mathbf{e}}{\partial\delta\xi}=\mathop{lim}_{\delta\xi\rightarrow 0}\frac{e(\delta\xi\oplus \xi)-e(\xi)}{\delta\xi}=\frac{\partial\mathbf{e}}{\partial \mathbf{P'}}\frac{\partial \mathbf{P'}}{\partial\delta\xi}
       $$

       - $\delta\xi$:给位姿T添加的左乘扰动量
       - $\oplus$:李代数上的左乘扰动
       - $P'$:相机坐标系下的空间点坐标

     - 计算得到矩阵：
       $$
       \frac{\partial\mathbf{e}}{\partial\delta\mathbf{\xi}}=\left [\begin{array}{cccc}
       \frac{f_x}{Z'} & 0 & -\frac{f_xX'}{Z'^2} & -\frac{f_xX'Y'}{Z'^2} & f_x+\frac{f_xX'^2}{Z'^2} & -\frac{f_xY'}{Z'} \\
       0 &\frac{f_y}{Z'} & -\frac{f_yY'}{Z'^2} & -f_y-\frac{f_yY'^2}{Z'^2} & \frac{f_yX'Y'}{Z'^2} & \frac{f_yX'}{Z'} \\
       
       \end{array}\right] \tag{4}
       $$

  2. 重投影误差关于空间点的导数（**优化空间位置**）

     - 利用链式法则得到：
       $$
       \frac{\partial\mathbf{e}}{\partial\mathbf{P}}=\frac{\partial\mathbf{e}}{\partial\mathbf{P'}}\frac{\partial\mathbf{P'}}{\partial\mathbf{P}}
       $$

     - 计算得到矩阵：
       $$
       \frac{\partial\mathbf{e}}{\partial\mathbf{P}}=-\left [\begin{array}{cccc}
       \frac{f_x}{Z'} & 0 & -\frac{f_xX'}{Z'^2} \\
       0 & \frac{f_y}{Z'} & -\frac{f_yY'}{Z'^2}  \\
       \end{array}\right]\mathbf{R} \tag{5}
       $$
     
     

## 7.实践：求解PnP

- 代码：

  ```c++
  /*
  分别用了3种方法求解PnP问题
  1. 使用OpenCV自带的PnP函数求解。
  2. 手写牛顿方法求解
  3. 使用g2o求解
  */
  #include <iostream>
  #include <opencv2/core/core.hpp>
  #include <opencv2/features2d/features2d.hpp>
  #include <opencv2/highgui/highgui.hpp>
  #include <opencv2/calib3d/calib3d.hpp>
  #include <Eigen/Core>
  #include <g2o/core/base_vertex.h>
  #include <g2o/core/base_unary_edge.h>
  #include <g2o/core/sparse_optimizer.h>
  #include <g2o/core/block_solver.h>
  #include <g2o/core/solver.h>
  #include <g2o/core/optimization_algorithm_gauss_newton.h>
  #include <g2o/solvers/dense/linear_solver_dense.h>
  #include <sophus/se3.hpp>
  #include <chrono>
  
  using namespace std;
  using namespace cv;
  
  void find_feature_matches(
    const Mat &img_1, const Mat &img_2,
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch> &matches);
  
  // 像素坐标转相机归一化坐标
  Point2d pixel2cam(const Point2d &p, const Mat &K);
  
  // BA by g2o
  typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
  typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;
  
  // 使用g2o求解BA优化
  void bundleAdjustmentG2O(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose
  );
  
  // 手写高斯牛顿求解PnP
  void bundleAdjustmentGaussNewton(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose
  );
  
  int main(int argc, char **argv) {
    if (argc != 5) {
      cout << "usage: pose_estimation_3d2d img1 img2 depth1 depth2" << endl;
      return 1;
    }
    //-- 读取图像
    Mat img_1 = imread(argv[1], IMREAD_COLOR);
    Mat img_2 = imread(argv[2], IMREAD_COLOR);
    assert(img_1.data && img_2.data && "Can not load images!");
  
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;
  
    // 建立3D点,读取图1的深度图
    Mat d1 = imread(argv[3], IMREAD_UNCHANGED);       // 深度图为16位无符号数，单通道图像
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    for (DMatch m:matches) {
      ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
      if (d == 0)   // bad depth
        continue;
      float dd = d / 5000.0;
      Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
      pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd));
      pts_2d.push_back(keypoints_2[m.trainIdx].pt);
    }
  
    cout << "3d-2d pairs: " << pts_3d.size() << endl;
  
    // 1. 使用OpenCV自带的PnP函数求解
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    Mat r, t;
    solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    Mat R;
    cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp in opencv cost time: " << time_used.count() << " seconds." << endl;
  
    cout << "R=" << endl << R << endl;
    cout << "t=" << endl << t << endl;
  
    VecVector3d pts_3d_eigen;
    VecVector2d pts_2d_eigen;
    for (size_t i = 0; i < pts_3d.size(); ++i) {
      pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
      pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x, pts_2d[i].y));
    }
    
    // 2. 手写高斯牛顿法，求解PnP
    cout << "calling bundle adjustment by gauss newton" << endl;
    Sophus::SE3d pose_gn;
    t1 = chrono::steady_clock::now();
    bundleAdjustmentGaussNewton(pts_3d_eigen, pts_2d_eigen, K, pose_gn);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp by gauss newton cost time: " << time_used.count() << " seconds." << endl;
    
    // 3. 使用g2o求解PnP问题
    cout << "calling bundle adjustment by g2o" << endl;
    Sophus::SE3d pose_g2o;
    t1 = chrono::steady_clock::now();
    bundleAdjustmentG2O(pts_3d_eigen, pts_2d_eigen, K, pose_g2o);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp by g2o cost time: " << time_used.count() << " seconds." << endl;
    return 0;
  }
  
  void find_feature_matches(const Mat &img_1, const Mat &img_2,
                            std::vector<KeyPoint> &keypoints_1,
                            std::vector<KeyPoint> &keypoints_2,
                            std::vector<DMatch> &matches) {
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);
  
    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);
  
    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descriptors_1, descriptors_2, match);
  
    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;
  
    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++) {
      double dist = match[i].distance;
      if (dist < min_dist) min_dist = dist;
      if (dist > max_dist) max_dist = dist;
    }
  
    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);
  
    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++) {
      if (match[i].distance <= max(2 * min_dist, 30.0)) {
        matches.push_back(match[i]);
      }
    }
  }
  
  Point2d pixel2cam(const Point2d &p, const Mat &K) {
    return Point2d
      (
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
      );
  }
  
  void bundleAdjustmentGaussNewton(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose) {
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    const int iterations = 10;
    double cost = 0, lastCost = 0;
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);
  
    for (int iter = 0; iter < iterations; iter++) {
      Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
      Vector6d b = Vector6d::Zero();
  
      cost = 0;
      // compute cost
      for (int i = 0; i < points_3d.size(); i++) {
        Eigen::Vector3d pc = pose * points_3d[i];
        double inv_z = 1.0 / pc[2];
        double inv_z2 = inv_z * inv_z;
        Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);
  
        Eigen::Vector2d e = points_2d[i] - proj;
  
        cost += e.squaredNorm();
        Eigen::Matrix<double, 2, 6> J;
        J << -fx * inv_z,
          0,
          fx * pc[0] * inv_z2,
          fx * pc[0] * pc[1] * inv_z2,
          -fx - fx * pc[0] * pc[0] * inv_z2,
          fx * pc[1] * inv_z,
          0,
          -fy * inv_z,
          fy * pc[1] * inv_z2,
          fy + fy * pc[1] * pc[1] * inv_z2,
          -fy * pc[0] * pc[1] * inv_z2,
          -fy * pc[0] * inv_z;
  
        H += J.transpose() * J;
        b += -J.transpose() * e;
      }
  
      Vector6d dx;
      dx = H.ldlt().solve(b);
  
      if (isnan(dx[0])) {
        cout << "result is nan!" << endl;
        break;
      }
  
      if (iter > 0 && cost >= lastCost) {
        // cost increase, update is not good
        cout << "cost: " << cost << ", last cost: " << lastCost << endl;
        break;
      }
  
      // update your estimation
      pose = Sophus::SE3d::exp(dx) * pose;
      lastCost = cost;
  
      cout << "iteration " << iter << " cost=" << std::setprecision(12) << cost << endl;
      if (dx.norm() < 1e-6) {
        // converge
        break;
      }
    }
  
    cout << "pose by g-n: \n" << pose.matrix() << endl;
  }
  
  /// vertex and edges used in g2o ba
  // 图优化的顶点：用于表示优化变量
  class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    virtual void setToOriginImpl() override {
      _estimate = Sophus::SE3d();
    }
  
    /// left multiplication on SE3
    virtual void oplusImpl(const double *update) override {
      Eigen::Matrix<double, 6, 1> update_eigen;
      update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
      _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }
  
    virtual bool read(istream &in) override {}
  
    virtual bool write(ostream &out) const override {}
  };
  
  // 图优化的边：用于表示误差项
  class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K) : _pos3d(pos), _K(K) {}
  
    virtual void computeError() override {
      const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
      Sophus::SE3d T = v->estimate();
      Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
      pos_pixel /= pos_pixel[2];
      _error = _measurement - pos_pixel.head<2>();
    }
  
    virtual void linearizeOplus() override {
      const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
      Sophus::SE3d T = v->estimate();
      Eigen::Vector3d pos_cam = T * _pos3d;
      double fx = _K(0, 0);
      double fy = _K(1, 1);
      double cx = _K(0, 2);
      double cy = _K(1, 2);
      double X = pos_cam[0];
      double Y = pos_cam[1];
      double Z = pos_cam[2];
      double Z2 = Z * Z;
      _jacobianOplusXi
        << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
        0, -fy / Z, fy * Y / (Z * Z), fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
    }
  
    virtual bool read(istream &in) override {}
  
    virtual bool write(ostream &out) const override {}
  
  private:
    Eigen::Vector3d _pos3d;
    Eigen::Matrix3d _K;
  };
  
  void bundleAdjustmentG2O(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose) {
  
    // 构建图优化，先设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;  // pose is 6, landmark is 3
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
    // 梯度下降方法，可以从GN, LM, DogLeg 中选
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
      make_unique<BlockSolverType>(make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);   // 设置求解器
    optimizer.setVerbose(true);       // 打开调试输出
  
    // vertex
    VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(Sophus::SE3d());
    optimizer.addVertex(vertex_pose);
  
    // K
    Eigen::Matrix3d K_eigen;
    K_eigen <<
            K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
      K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
      K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);
  
    // edges
    int index = 1;
    for (size_t i = 0; i < points_2d.size(); ++i) {
      auto p2d = points_2d[i];
      auto p3d = points_3d[i];
      EdgeProjection *edge = new EdgeProjection(p3d, K_eigen);
      edge->setId(index);
      edge->setVertex(0, vertex_pose);
      edge->setMeasurement(p2d);
      edge->setInformation(Eigen::Matrix2d::Identity());
      optimizer.addEdge(edge);
      index++;
    }
  
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
    cout << "pose estimated by g2o =\n" << vertex_pose->estimate().matrix() << endl;
    pose = vertex_pose->estimate();
  }
  
  ```

- 编译

  ```c++
  cmake_minimum_required(VERSION 2.8)
  project(vo1)
  
  set(CMAKE_BUILD_TYPE "Release")
  add_definitions("-DENABLE_SSE")
  set(CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4")
  list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
  list(APPEND CMAKE_MODULE_PATH /home/yang/3rd_library/g2o-20230223_git/cmake_modules)
  find_package(OpenCV REQUIRED)
  find_package(G2O REQUIRED)
  find_package(Sophus REQUIRED)
  find_package(FMT REQUIRED)
  
  include_directories(
          ${OpenCV_INCLUDE_DIRS}
          ${G2O_INCLUDE_DIRS}
          ${Sophus_INCLUDE_DIRS}
          "/usr/include/eigen3/"
  )
  
  add_executable(test test.cpp)
  target_link_libraries(test
          g2o_core g2o_stuff
          ${OpenCV_LIBS}
          fmt::fmt)
  ```

## 8.3D-3D：ICP

迭代最近点(Iterative Closest Point,ICP)解决问题：根据一对匹配好的3D点$P=\{p_1,\cdots,p_n\}$和$P'=\{p_1',\cdots,p_n' \}$来得到位姿$R,t$。$\forall i,\mathbf{p}_i=\mathbf{R}\mathbf{p}_i'+t $

和PnP类似，ICP可以用线性代数(SVD)和非线性优化方式(类似BA)求解：

### 8.1 SVD方法

根据上述ICP问题，定义第i对点的误差：
$$
\mathbf{e}_i=\mathbf{p}_i-(\mathbf{Rp}_i'+\mathbf{t}) \tag{1}
$$
根据误差，构建最小二乘问题，求使误差平方和达到最小的$R,t$
$$
\mathop{min}_{\mathbf{R,t}}\frac{1}{2}\sum_{i=1}^n||\mathbf{p}_i-(\mathbf{Rp}_i'+\mathbf{t})||_2^2 \tag{2}
$$
定义两组点的质心：
$$
\mathbf{p}=\frac{1}{n} \sum_{i=1}^{n}(\mathbf{p}_i),\ \ \mathbf{p}'=\frac{1}{n}\sum_{i=1}^n(\mathbf{p}_i') \tag{3}
$$
对2式展开后，可将优化目标函数简化为： 
$$
\mathop{min}_{\mathbf{R,t}}\ J=\frac{1}{2}\sum_{i=1}^n||\mathbf{p}_i-\mathbf{p}-\mathbf{R}(\mathbf{p}_i'-\mathbf{p}')||^2+||\mathbf{p}-\mathbf{Rp}'-\mathbf{t}||^2 \tag{4}
$$
观察4式发现：左边只和旋转矩阵有关，右边既有$R,t$也和质心相关。所以可以先得到R,再令第二项为0得到t
$$
\begin{align}
1.& \text{计算每个点的去质心坐标：}\ \ \mathbf{q}_i=\mathbf{p}_i-\mathbf{p},\ \ \mathbf{q}_i'=\mathbf{p}_i'-\mathbf{p}' \\
2.& \text{由优化问题计算旋转矩阵：}\ \ \mathbf{R}^*=arg\ \mathop{min}_{\mathbf{R}}\frac{1}{2}\sum_{i=1}^n||\mathbf{q}_i-\mathbf{Rq}_i'||^2\\
3.& \text{根据2式中算出的R计算t:}\ \ \ \ \ \ \mathbf{t}^*=\mathbf{p}-\mathbf{Rp}'
\end{align} \tag{5}
$$
5式中的第二步展开后发现实际的优化目标函数变为：
$$
\sum_{i=1}^n-\mathbf{q}_i^T\mathbf{R}\mathbf{q}_i'=-tr(\mathbf{R}\sum_{i=1}^n\mathbf{q}_i'\mathbf{q}_i^T) \tag{6}
$$
用SVD的方法求解，定义矩阵：
$$
\mathbf{W}=\sum_{i=1}^n\mathbf{q}_i\mathbf{q}_i'^T \tag{7}
$$
对3x3矩阵$\mathbf{W}$进行SVD分解
$$
\mathbf{W}=\mathbf{U\Sigma V}^T \tag{8}
$$
当$\mathbf{W}$为满秩时, 得到R
$$
\mathbf{R}=\mathbf{UV}^T\tag{9}
$$
得到R后代入5式的第三步公式计算t

### 8.2 非线性优化方法

以李代数表达位姿时，目标函数写成：
$$
\mathop{min}_\xi =\frac{1}{2}\sum_{i=1}^n ||\mathbf{p}_i-exp(\xi^{\wedge})\mathbf{p}_i'||_2^2 \tag{10}
$$
使用李代数扰动模型求误差项关于位姿的导数：
$$
\frac{\partial\mathbf{e}}{\partial\delta\xi}=-(exp(\xi^{\wedge})\mathbf{p}_i')^{\odot}
$$
ICP存在唯一解和无穷多解的情况，唯一解时，只要能找到极小值解，这个解就是全局最优值。这也意味着ICP可以任意选定初始值。

## 9.实践：求解ICP

- 代码

  ```c++
  #include <iostream>
  #include <opencv2/core/core.hpp>
  #include <opencv2/features2d/features2d.hpp>
  #include <opencv2/highgui/highgui.hpp>
  #include <opencv2/calib3d/calib3d.hpp>
  #include <Eigen/Core>
  #include <Eigen/Dense>
  #include <Eigen/Geometry>
  #include <Eigen/SVD>
  #include <g2o/core/base_vertex.h>
  #include <g2o/core/base_unary_edge.h>
  #include <g2o/core/block_solver.h>
  #include <g2o/core/optimization_algorithm_gauss_newton.h>
  #include <g2o/core/optimization_algorithm_levenberg.h>
  #include <g2o/solvers/dense/linear_solver_dense.h>
  #include <chrono>
  #include <sophus/se3.hpp>
  
  using namespace std;
  using namespace cv;
  
  void find_feature_matches(
    const Mat &img_1, const Mat &img_2,
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch> &matches);
  
  // 像素坐标转相机归一化坐标
  Point2d pixel2cam(const Point2d &p, const Mat &K);
  
  // 自己写的SVD求解ICP方法
  void pose_estimation_3d3d(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t
  );
  
  // 使用g2o实现非线性优化方法
  void bundleAdjustment(
    const vector<Point3f> &points_3d,
    const vector<Point3f> &points_2d,
    Mat &R, Mat &t
  );
  
  /// vertex and edges used in g2o ba
  class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    virtual void setToOriginImpl() override {
      _estimate = Sophus::SE3d();
    }
  
    /// left multiplication on SE3
    virtual void oplusImpl(const double *update) override {
      Eigen::Matrix<double, 6, 1> update_eigen;
      update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
      _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }
  
    virtual bool read(istream &in) override {}
  
    virtual bool write(ostream &out) const override {}
  };
  
  /// g2o edge
  class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d &point) : _point(point) {}
  
    virtual void computeError() override {
      const VertexPose *pose = static_cast<const VertexPose *> ( _vertices[0] );
      _error = _measurement - pose->estimate() * _point;
    }
  
    virtual void linearizeOplus() override {
      VertexPose *pose = static_cast<VertexPose *>(_vertices[0]);
      Sophus::SE3d T = pose->estimate();
      Eigen::Vector3d xyz_trans = T * _point;
      _jacobianOplusXi.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
      _jacobianOplusXi.block<3, 3>(0, 3) = Sophus::SO3d::hat(xyz_trans);
    }
  
    bool read(istream &in) {}
  
    bool write(ostream &out) const {}
  
  protected:
    Eigen::Vector3d _point;
  };
  
  int main(int argc, char **argv) {
    if (argc != 5) {
      cout << "usage: pose_estimation_3d3d img1 img2 depth1 depth2" << endl;
      return 1;
    }
    //-- 读取图像
    Mat img_1 = imread(argv[1], IMREAD_COLOR);
    Mat img_2 = imread(argv[2], IMREAD_COLOR);
  
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;
  
    // 建立3D点
    Mat depth1 = imread(argv[3], IMREAD_UNCHANGED);       // 深度图为16位无符号数，单通道图像
    Mat depth2 = imread(argv[4], IMREAD_UNCHANGED);       // 深度图为16位无符号数，单通道图像
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point3f> pts1, pts2;
  
    for (DMatch m:matches) {
      ushort d1 = depth1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
      ushort d2 = depth2.ptr<unsigned short>(int(keypoints_2[m.trainIdx].pt.y))[int(keypoints_2[m.trainIdx].pt.x)];
      if (d1 == 0 || d2 == 0)   // bad depth
        continue;
      Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
      Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
      float dd1 = float(d1) / 5000.0;
      float dd2 = float(d2) / 5000.0;
      pts1.push_back(Point3f(p1.x * dd1, p1.y * dd1, dd1));
      pts2.push_back(Point3f(p2.x * dd2, p2.y * dd2, dd2));
    }
  	
    // 1. 自己写的SVD方法
    cout << "3d-3d pairs: " << pts1.size() << endl;
    Mat R, t;
    pose_estimation_3d3d(pts1, pts2, R, t);
    cout << "ICP via SVD results: " << endl;
    cout << "R = " << R << endl;
    cout << "t = " << t << endl;
    cout << "R_inv = " << R.t() << endl;
    cout << "t_inv = " << -R.t() * t << endl;
  
    cout << "calling bundle adjustment" << endl;
    
    // 2. 使用g2o实现的非线性优化方法
    bundleAdjustment(pts1, pts2, R, t);
  
    // verify p1 = R * p2 + t
    for (int i = 0; i < 5; i++) {
      cout << "p1 = " << pts1[i] << endl;
      cout << "p2 = " << pts2[i] << endl;
      cout << "(R*p2+t) = " <<
           R * (Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, pts2[i].z) + t
           << endl;
      cout << endl;
    }
  }
  
  void find_feature_matches(const Mat &img_1, const Mat &img_2,
                            std::vector<KeyPoint> &keypoints_1,
                            std::vector<KeyPoint> &keypoints_2,
                            std::vector<DMatch> &matches) {
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);
  
    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);
  
    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descriptors_1, descriptors_2, match);
  
    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;
  
    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++) {
      double dist = match[i].distance;
      if (dist < min_dist) min_dist = dist;
      if (dist > max_dist) max_dist = dist;
    }
  
    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);
  
    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++) {
      if (match[i].distance <= max(2 * min_dist, 30.0)) {
        matches.push_back(match[i]);
      }
    }
  }
  
  Point2d pixel2cam(const Point2d &p, const Mat &K) {
    return Point2d(
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
  }
  
  void pose_estimation_3d3d(const vector<Point3f> &pts1,
                            const vector<Point3f> &pts2,
                            Mat &R, Mat &t) {
    Point3f p1, p2;     // center of mass
    int N = pts1.size();
    for (int i = 0; i < N; i++) {
      p1 += pts1[i];
      p2 += pts2[i];
    }
    p1 = Point3f(Vec3f(p1) / N);
    p2 = Point3f(Vec3f(p2) / N);
    vector<Point3f> q1(N), q2(N); // remove the center
    for (int i = 0; i < N; i++) {
      q1[i] = pts1[i] - p1;
      q2[i] = pts2[i] - p2;
    }
  
    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++) {
      W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    cout << "W=" << W << endl;
  
    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
  
    cout << "U=" << U << endl;
    cout << "V=" << V << endl;
  
    Eigen::Matrix3d R_ = U * (V.transpose());
    if (R_.determinant() < 0) {
      R_ = -R_;
    }
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);
  
    // convert to cv::Mat
    R = (Mat_<double>(3, 3) <<
      R_(0, 0), R_(0, 1), R_(0, 2),
      R_(1, 0), R_(1, 1), R_(1, 2),
      R_(2, 0), R_(2, 1), R_(2, 2)
    );
    t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
  }
  
  void bundleAdjustment(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t) {
    // 构建图优化，先设定g2o
    typedef g2o::BlockSolverX BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
    // 梯度下降方法，可以从GN, LM, DogLeg 中选
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);   // 设置求解器
    optimizer.setVerbose(true);       // 打开调试输出
  
    // vertex
    VertexPose *pose = new VertexPose(); // camera pose
    pose->setId(0);
    pose->setEstimate(Sophus::SE3d());
    optimizer.addVertex(pose);
  
    // edges
    for (size_t i = 0; i < pts1.size(); i++) {
      EdgeProjectXYZRGBDPoseOnly *edge = new EdgeProjectXYZRGBDPoseOnly(
        Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z));
      edge->setVertex(0, pose);
      edge->setMeasurement(Eigen::Vector3d(
        pts1[i].x, pts1[i].y, pts1[i].z));
      edge->setInformation(Eigen::Matrix3d::Identity());
      optimizer.addEdge(edge);
    }
  
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
  
    cout << endl << "after optimization:" << endl;
    cout << "T=\n" << pose->estimate().matrix() << endl;
  
    // convert to cv::Mat
    Eigen::Matrix3d R_ = pose->estimate().rotationMatrix();
    Eigen::Vector3d t_ = pose->estimate().translation();
    R = (Mat_<double>(3, 3) <<
      R_(0, 0), R_(0, 1), R_(0, 2),
      R_(1, 0), R_(1, 1), R_(1, 2),
      R_(2, 0), R_(2, 1), R_(2, 2)
    );
    t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
  }
  
  ```

- 编译

  ```cmake
  cmake_minimum_required(VERSION 2.8)
  project(vo1)
  
  set(CMAKE_BUILD_TYPE "Release")
  add_definitions("-DENABLE_SSE")
  set(CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4")
  list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
  list(APPEND CMAKE_MODULE_PATH /home/yang/3rd_library/g2o-20230223_git/cmake_modules)
  find_package(OpenCV REQUIRED)
  find_package(G2O REQUIRED)
  find_package(Sophus REQUIRED)
  find_package(FMT REQUIRED)
  
  include_directories(
          ${OpenCV_INCLUDE_DIRS}
          ${G2O_INCLUDE_DIRS}
          ${Sophus_INCLUDE_DIRS}
          "/usr/include/eigen3/"
  )
  
  add_executable(test test.cpp)
  target_link_libraries(test
          g2o_core g2o_stuff
          ${OpenCV_LIBS}
          fmt::fmt)
  ```

  

# 八、视觉里程计：直接法

## 1. 直接法的引出

- 特征点法虽然是主流方法，但也有如下缺点：

  - 关键点的提取与描述子的计算非常耗时
  - 一副图由几十万个像素，但特征点就几百个。只是用特征点，会丢弃大部分**可能有用**的信息
  - 相机有时会运动到特征缺失的地方，比如一面白墙、一个空荡的走廊，这时没有足够的匹配点来计算相机运动

- 克服上述缺点的2个方案：

  1. 保留特征点但只计算关键点不计算描述子，同时使用**光流法(Optical Flow)**跟踪特征点的运动。

     - 这样可以避免计算匹配描述子，而光流法的计算时间小于计算匹配描述子的时间。

     - 这种方法仍然使用特征点，只是把匹配描述子替换成了光流跟踪，估计相机运动时仍然使用对极几何、pnp或icp算法。

  2. 在计算关键点不计算描述子的同时不保留特征点，同时使用**直接法(Direct Method)**计算特征点的下一时刻图像中的位置。

     - 这可以省去计算描述子的时间，也省去了计算光流的时间。
     - 直接法会根据图像的**像素灰度信息**同时估计相机运动和点的投影，不要求提取到的点必须为角点。

- 直接法相比特征点法：

  - 特征点法中通过最小化**重投影误差(reprojection error)**来优化相机运动。因此需要特征匹配来准确的知道空间点在两个相机中投影后的像素坐标。
  
  - 直接法不需要知道点和点之间的对应关系，而是通过最小化**光度误差(Photometric error)**来同时优化位姿和地图。
  
    > 最小光度误差：通过比较像素值或图像亮度之间的差异来计算，它可以采用不同的度量方式，如均方误差（Mean Squared Error，MSE）或绝对误差（Absolute Error）等
  
  - 根据使用像素的数量，直接法可以分为稀疏、稠密和半稠密3种。因此相比特征点法只能重构稀疏特征点(稀疏地图)，直接法还具有恢复稠密/半稠密结构的能力
  
- 使用直接法的开源项目有：[SVO](https://github.com/uzh-rpg/rpg_svo),[LSD-SLAM](https://github.com/tum-vision/lsd_slam),[DSO](https://github.com/JakobEngel/dso)

## 2. 2D 光流

直接法由光流演变而来，光流描述了像素在图像中的运动，而直接法则另外附带一个相机的运动。

光流是一种描述像素随时间在图像之间运动的方法：

- 计算部分像素运动的为 **稀疏光流** 

  以Lucas-Schunck光流为代表

- 计算所有像素的为 **稠密光流**

  以Horn-schunck光流为代表

### 2.1 Lucas-Kanade光流

![image-20230720002307204](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/202307200023269.png)

<center style="color:#C125C0C0">图1: LK光流法示意图</center>

- 光流法有个假设：（现实中很难成立）

  - **灰度不变假设**
    $$
    I(x(t),y(t),t)=Constant \tag{1}
    $$
    对于小运动
    $$
    I(x+udt,y+vdt,t+dt)=I(x,y,t) \tag{2}
    $$

    - $(u,v)$：光流速度 optical flow(velocity)
    - $(dx,dy)=(udt,vdt)$：变换displacement

  - 因为光流速度有x,y轴2个方向，所以为了计算u,v还需要假设：**某一个窗口内的像素具有相同的运动**

- 计算像素的运动：

  1. 将上面的2式泰勒展开
     $$
     \mathbf{I}(x+dx,y+dy,t+dt)\approx\mathbf{I}(x,y,t)+\frac{\partial\mathbf{I}}{\partial x}dx+\frac{\partial\mathbf{I}}{\partial y}dy+\frac{\partial\mathbf{I}}{\partial t}dt \tag{3}
     $$

  2. 由于灰度不变假设所以3式的右边第二部分为0：
     $$
     \frac{\partial\mathbf{I}}{\partial x}dx+\frac{\partial\mathbf{I}}{\partial y}dy+\frac{\partial\mathbf{I}}{\partial t}dt=0 \tag{4}
     $$

  3. 两边同时除以dt，就得到了像素在x，y轴上的速度
     $$
     \begin{align}
     \frac{\partial\mathbf{I}}{\partial x}\frac{dx}{dt}+\frac{\partial\mathbf{I}}{\partial y}\frac{dy}{dt} &=-\frac{\partial\mathbf{I}}{\partial t} \tag{5} \\
     \mathbf{I}_xu+\mathbf{I}_yv&=-\mathbf{I}_t
     \end{align}
     $$

     - $u=\frac{dx}{dt},v=\frac{dy}{dt}$：像素在x轴和y轴上的速度，也就是我们想要计算的量
     - $\mathbf{I}_x=\frac{\partial \mathbf{I}}{\partial x},\mathbf{I}_y=\frac{\partial \mathbf{I}}{\partial y}$：图像灰度在该点处x和y方向的梯度
     - $\mathbf{I}_t=\frac{\partial \mathbf{I}}{\partial t}$：图像灰度对时间的变化量极为

  4. 根据第二个假设：某一个窗口内的像素具有相同的运动

     - 考虑一个w x w的窗口，对于$w^2$个像素，就有$w^2$个5式，写成向量形式：
       $$
       [\mathbf{I}_x\ \ \mathbf{I}_y]_k\left [\begin{array}{cccc}
       u \\
       v \\
       \end{array}\right]=-\mathbf{I}_{tk},\ \ \ k=1,\cdots,w^2 \tag{6}
       $$

     - 用A,b代指这$w^2$个光度梯度，由此可以得到一个关于u,v的超定线性方程：
       $$
       \mathbf{A}\left [\begin{array}{cccc}
       u \\
       v \\
       \end{array}\right]=-\mathbf{b}\tag{7}
       $$

       > 超定线性方程: 如果一个线性方程组包含n个未知数，但包含m个线性方程，其中m < n，那么这个方程组就是超定的。
       >
       > 比如7式是1个方程，对应2个未知数，所以是超定的。

     - 超定线性方程通常使用最小二乘法来求解：
       $$
       \left [\begin{array}{cccc}
       u \\
       v \\
       \end{array}\right]^*=-(\mathbf{A}^T\mathbf{A})^{-1}\mathbf{A}^T\mathbf{b}
       $$


## 3. 实践：LK光流

### 3.1 代码

```c++
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
 
using namespace std;
using namespace cv;
 
string file_1 = "../LK1.png";  // first image
string file_2 = "../LK2.png";  // second image
 
// ! Optical flow tracker and interface  光流跟踪
class OpticalFlowTracker {
public:
    OpticalFlowTracker(
        const Mat &img1_,               // 图像1
        const Mat &img2_,               // 图像2
        const vector<KeyPoint> &kp1_,   // 关键点1  -> 图像 1
        vector<KeyPoint> &kp2_,         // 关键点2  -> 图像 2
        vector<bool> &success_,         // true if a keypoint is tracked successfully 关键点跟踪是正确的
        bool inverse_ = true, bool has_initial_ = false) :  // bool型变量 判断是否采用反向光流(使用当前图像和上一帧图像)
        img1(img1_), img2(img2_), kp1(kp1_), kp2(kp2_), success(success_), inverse(inverse_),
        has_initial(has_initial_) {}
 
    /*
        cv::Range数据结构，用于表示一维数据范围，通常用于图像和矩阵的子区域访问。
            有两个关键的变量start和end，分别表示范围的起始和结束位置
            Range表示范围从start到end，包含start，但不包含end
    */
    void calculateOpticalFlow(const Range &range);

private:
    const Mat &img1;
    const Mat &img2;
    const vector<KeyPoint> &kp1;
    vector<KeyPoint> &kp2;
    vector<bool> &success;
    bool inverse = true;
    bool has_initial = false;
};

/**
 * ! 双线性插值求灰度值get a gray scale value from reference image (bi-linear interpolated)
 * @brief 为什么用双线性插值求灰度值？ 因为迭代dx,dy时，他们通常不是整数的。所以求(x+dx,y+dy)附近4个整数坐标的像素点
 * @param img
 * @param x
 * @param y
 * @return the interpolated value of this pixel
 * inline表示内联函数，它是为了解决一些频繁调用的小函数大量消耗栈空间的问题而引入的.
 注意要写在calculateOpticalFlow()函数之前
 */
inline float GetPixelValue(const cv::Mat &img, float x, float y)
{
    // boundary check(边界检验)
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= img.cols) x = img.cols - 1;    // x: 列坐标
    if (y >= img.rows) y = img.rows - 1;    // y: 行坐标

    // img.step是一个整数值，表示从图像或矩阵的一行的开头到下一行的开头之间的字节间隔
    // img.data 是 cv::Mat 的数据指针，指向图像数据的起始地址。可以看作将图片像素展开为了一行
    uchar *data = &img.data[int(y) * img.step + int(x)];   // 获取某一像素的内存地址
    float xx = x - floor(x);                // floor向下取整，比如floor(3.7)=3.0， xx = 0.7
    float yy = y - floor(y);

    /* 
    想要求得w点的灰度值,abcd为4个距离w最近的像素点
    b                   c
       
        w
    
    a                   d
    */
    return float(
        (1 - xx) * (1 - yy) * data[0] +     // data[0] : a点
        xx * (1 - yy) * data[1] +           // data[1] : d点
        (1 - xx) * yy * data[img.step] +    // data[img.step：b点
        xx * yy * data[img.step + 1]        // data[img.step+1]：c点
    );
}

// ! 使用高斯牛顿法求解图像2中相应的角点坐标
void OpticalFlowTracker::calculateOpticalFlow(const Range &range) {
    // parameters
    int half_patch_size = 4;
    int iterations = 10;        // 最大迭代次数
    // 对图像1中的每个GFTT角点进行高斯牛顿优化
    for (size_t i = range.start; i < range.end; i++)
    {
        auto kp = kp1[i];
        double dx = 0, dy = 0;  // 如果不使用初值，则dx，dy为0，即初值为图1关键点坐标系
        if (has_initial)        // 如果kp2进行了初始化，则执行：第二张图特征点坐标-第一张图特征点坐标
        {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }
        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded
        
        // * Gauss-Newton iterations
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();    // hessian 将H初始化为0
        Eigen::Vector2d b = Eigen::Vector2d::Zero();    // bias 将b初始化为0
        Eigen::Vector2d J;                              // jacobian 雅克比矩阵J
        for (int iter = 0; iter < iterations; iter++) {
            if (inverse == false) 
            {   
                // 普通光流法H，b都重置，因为每次循环它们是变化的
                H = Eigen::Matrix2d::Zero();
                b = Eigen::Vector2d::Zero();
            } 
            
            else 
            {
                // 反向光流法只重置b，因为反向光法H不变（雅克比矩阵J不变）
                b = Eigen::Vector2d::Zero();
            }
            cost = 0;   // 代价初始化为0 

            // ** compute cost and jacobian 计算代价和雅克比矩阵
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++)  //x,y是patch内遍历
                {
                    /*
                        计算误差：eij = I1(u+i,v+j)-I2(u+i+Δu,v+j+Δv)
                                        i  -> kp.pt.x
                                        j  -> kp.pt.y
                                        u  -> x
                                        v  -> y
                                        Δu -> dx
                                        Δv -> dy
                                        Jacobian
                        其中每个像素的灰度值 I 都是由双线性插值函数GetPixelValue()计算得到的
                    */
                    double error = GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y) -
                                   GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy);
                    
                    // *** 正向光流法
                    if (inverse == false)
                    {   
                        /*
                            
                            计算雅可比矩阵公式： 见书214公式8.10，分别对dx,dy求导
                            J = - [ {I2( u + i + Δu + 1,  v + j + Δv    ) - I2( u + i + Δu - 1,  v + j + Δv     )}/2,
                                     I2( u + i + Δu ,     v + j + Δv + 1) - I2( u + i + Δu,      v + j + Δv - 1 )}/2]T T表示转置
                                    I2 -> 图像2的灰度信息
                                    u  -> x
                                    v  -> y
                                    Δu -> dx    待优化变量
                                    Δv -> dy    待优化变量
                                    i  -> kp.pt.x
                                    j  -> kp.pt.y
                        */
                        J = -1.0 * Eigen::Vector2d(
                            0.5 * (GetPixelValue(img2, kp.pt.x + dx + x + 1, kp.pt.y + dy + y) -
                                   GetPixelValue(img2, kp.pt.x + dx + x - 1, kp.pt.y + dy + y)),
                            0.5 * (GetPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y + 1) -
                                   GetPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y - 1))
                        );
                    } 
                    // *** 反向光流法
                    else if (iter == 0)
                    {
                        /*
                            反向光流法采用图1的灰度负梯度代替图2的梯度计算，因此雅可比矩阵保持不变，只需要计算error
                            计算雅可比矩阵公式：
                            J = - [ {I1( u + i + 1, v + j    ) - I1( u + i - 1, v + j    )}/2,
                                     I1( u + i,     v + j + 1) - I1( u + i ,    v + j - 1)}/2]T T表示转置
                                I2 -> 图像2的灰度信息
                                i -> x
                                j -> y
                                u  -> kp.pt.x
                                v  -> kp.pt.y
                        */
                        J = -1.0 * Eigen::Vector2d(
                            0.5 * (GetPixelValue(img1, kp.pt.x + x + 1, kp.pt.y + y) -
                                   GetPixelValue(img1, kp.pt.x + x - 1, kp.pt.y + y)),
                            0.5 * (GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y + 1) -
                                   GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y - 1))
                        );
                    }
                    // compute H, b and set cost;
                    // 所有像素点构成的高维的H，b，cost与累加求和的H，b，cost是等价的
                    // 这里选择采用求和的方式
                    b += -error * J;            // b = -Jij * eij(累加和)
                    cost += error * error;      // cost = || eij ||2 2范数
                    if (inverse == false || iter == 0) {
                        // also update H
                        H += J * J.transpose(); // H = Jij Jij(T)(累加和)
                    }
                }

            // compute update
            //使用eigen库的Cholesky分解来求解增量方程H*update=b
            Eigen::Vector2d update = H.ldlt().solve(b);
            if (std::isnan(update[0]))//计算出来的更新量是非数字，光流跟踪失败，退出GN迭代
            {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = false;
                break;
            }
            if (iter > 0 && cost > lastCost) //代价不再减小，退出GN迭代
            {
                break;
            }
            // update dx, dy 更新优化变量和lastCost
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;
            if (update.norm() < 1e-2) //更新量的模小于1e-2，退出GN迭代
            {
                // converge
                break;
            }
        }//GN法进行完一次迭代
        success[i] = succ;
        // set kp2
        kp2[i].pt = kp.pt + Point2f(dx, dy);
    }
}


/**
 * ! 单层光流法single level optical flow
 * @param [in] img1 the first image
 * @param [in] img2 the second image
 * @param [in] kp1 keypoints in img1
 * @param [in|out] kp2 keypoints in img2, if empty, use initial guess in kp1
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 */
void OpticalFlowSingleLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse = false,//use inverse formulation?
    bool has_initial_guess = false
);


/**
 * ! 多层光流法multi level optical flow, scale of pyramid is set to 2 by default
 * the image pyramid will be create inside the function
 * @param [in] img1 the first pyramid
 * @param [in] img2 the second pyramid
 * @param [in] kp1 keypoints in img1
 * @param [out] kp2 keypoints in img2
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 */
void OpticalFlowMultiLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse = false
);
 
/**
 * ! 单层光流法
 * @brief 
 * 
 * @param img1 
 * @param img2 
 * @param kp1 
 * @param kp2 
 * @param success 
 * @param inverse 
 * @param has_initial 
 */
void OpticalFlowSingleLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse, bool has_initial) 
{
    kp2.resize(kp1.size());     // 用于存储第一张图特征点在第二张图中对应的的特征点
    success.resize(kp1.size()); // 记录每个点是否成功
    OpticalFlowTracker tracker(img1, img2, kp1, kp2, success, inverse, has_initial);
    /*
        parallel_for_ 是OpenCV中用于并行化图像处理任务的函数，它通过多线程执行指定的操作，以加速图像处理。 
        这里并行调用OpticalFlowTracker::calculateOpticalFlow()
        Range(0, kp1.size())：要处理的数据范围
        bind(xx,xx)：将calculateOpticalFlow 函数绑定到 OpticalFlowTracker 类的实例 tracker 上，然后作为并行处理运行的函数
                     placeholders::_1 将在 calculateOpticalFlow 中接收一个整数值，用于指示当前处理的关键点的索引。
    */
    parallel_for_(Range(0, kp1.size()),
                  std::bind(&OpticalFlowTracker::calculateOpticalFlow, &tracker, placeholders::_1));
}


//对图像1中的所有角点都完成了光流跟踪
/**
 * ! 多层光流法
 * @brief 
 * 
 * @param img1 
 * @param img2 
 * @param kp1 
 * @param kp2 
 * @param success 
 * @param inverse 
 */
void OpticalFlowMultiLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse) {
    // parameters
    int pyramids = 4;           // 金字塔层数为4
    double pyramid_scale = 0.5; // 每层之间的缩放因子设为0.5
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();  // 开始计时
    vector<Mat> pyr1, pyr2;     // image pyramids pyr1 -> 图像1的金字塔 pyr2 -> 图像2的金字塔    
    // *1. create pyramids 开始创建图像金字塔
    for (int i = 0; i < pyramids; i++) {
        if (i == 0) // 最底层是原始图像
        {
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        } 
        else 
        {
            Mat img1_pyr, img2_pyr;
            //使用resize进行下采样： 将图像pyr1[i-1]的宽和高各缩放0.5倍得到图像img1_pyr
            cv::resize(pyr1[i - 1], img1_pyr,
                       cv::Size(pyr1[i - 1].cols * pyramid_scale, pyr1[i - 1].rows * pyramid_scale));
            cv::resize(pyr2[i - 1], img2_pyr,
                       cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
            pyr1.push_back(img1_pyr);
            pyr2.push_back(img2_pyr);
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();          // 计时结束
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);  // 计算耗时
    cout << "build pyramid time: " << time_used.count() << endl;                // 输出构建图像金字塔的耗时
    
    // *2. 初始化图像金字塔最顶层特征点的坐标
    vector<KeyPoint> kp1_pyr, kp2_pyr;
    for (auto &kp:kp1) 
    {   // 这里意思大概是视觉slam十四讲p215的把上一层的追踪结果作为下一层光流的初始值
        auto kp_top = kp;                   // 读取第一张图的特征点
        kp_top.pt *= scales[pyramids - 1];  // 对这些特征点的坐标进行缩放，得到金字塔最顶层时的特征点坐标
        kp1_pyr.push_back(kp_top);          // 最顶层图像1的角点坐标
        kp2_pyr.push_back(kp_top);          // 最顶层图像2的角点坐标：用图像1的初始化图像2的
    }

    // *3. coarse-to-fine LK tracking in pyramids 由粗至精的光流跟踪（最顶层->最底层）
    for (int level = pyramids - 1; level >= 0; level--)
    {

        success.clear();
        t1 = chrono::steady_clock::now(); // 开始计时
        OpticalFlowSingleLevel(pyr1[level], pyr2[level], kp1_pyr, kp2_pyr, success, inverse, true); // 对每一层进行和单层光流一样的光流计算
        t2 = chrono::steady_clock::now(); // 计时结束
        auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);      // 计算耗时
        cout << "track pyr " << level << " cost time: " << time_used.count() << endl;   // 输出光流跟踪耗时
        if (level > 0) 
        {   
            // 放大特征点的坐标，为下一层作准备
            for (auto &kp: kp1_pyr)
                kp.pt /= pyramid_scale;//pyramidScale等于0.5，相当于乘了2
            for (auto &kp: kp2_pyr)
                kp.pt /= pyramid_scale;//pyramidScale等于0.5，相当于乘了2
        }
    }
    for (auto &kp: kp2_pyr)
        kp2.push_back(kp);  // 将光流计算出得到的第二张图片中的特征点存到kp2
}

int main(int argc, char **argv) {
 
    // 读取图片：cv::Mat是一个多维矩阵类，用于表示像素值、图像数据和其他多维数据
    Mat img1 = imread(file_1, 0);//0表示返回灰度图
    Mat img2 = imread(file_2, 0);//0表示返回灰度图
    
    // *1. 在第一张图像中提取角点
    // key points, using GFTT here.
    vector<KeyPoint> kp1;
    // 使用Good Features to Track（GFTT）角点检测器，三个参数为：
    // maxCorners最大角点数目。在此处为500。
    // qualityLevel角点可以接受的最小特征值，一般0.1或者0.01，不超过1。在此处为0.01。
    // minDistance角点之间的最小距离。在此处为20。
    Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(img1, kp1);    // 将关键点保存在向量'kp1'中
    
    // *2. 用光流法追踪他们在第二张图像中的位置

    /* 
        *2.1 单层光流: 将光流看成一个优化问题：通过最小化灰度误差估计最优的像素偏移
            缺点：
                要把光流写成优化问题，就必须假设优化的初始值靠近最优值，才能保证算法的收敛。
                如果相机运动较快，两张图像差异较明显，那么单层光流法容易达到一个局部最小值。
    */ 
    vector<KeyPoint> kp2_single;    // 存储第一张图中特征点在第二张图中的对应像素点
    vector<bool> success_single;
    OpticalFlowSingleLevel(img1, img2, kp1, kp2_single, success_single);
    
    /*
        *2.2 多层光流: 通过图像金字塔来解决单层光流无法应对两张图差异较大时的问题。
            图像金字塔：
                原始图像为底，每往上一层就下采样得到相对粗糙的图像。
            计算光流：
                先从顶层开始计算，然后将上一层的追踪结果作为下一层光流的初始值。
            可以解决单层光流缺点的原因：
                当原始图像像素运动较大时，在顶层的图像看来，运动仍然在一个很小的范围内。
    */ 
    vector<KeyPoint> kp2_multi;
    vector<bool> success_multi;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();          // 开始计时
    OpticalFlowMultiLevel(img1, img2, kp1, kp2_multi, success_multi, true);     // 调用opencv的OpticalFlowMultiLevel函数
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();          // 计时结束
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);  // 计算耗时
    cout << "optical flow by gauss-newton: " << time_used.count() << endl;      // 输出使用高斯牛顿法计算光流使用时间
    
    /*
        *2.3 opencv自带的计算光流函数
    */
    // use opencv's flow for validation
    vector<Point2f> pt1, pt2;
    for (auto &kp: kp1) pt1.push_back(kp.pt);   // 提供第一张图的特征点
    vector<uchar> status;                       // status中元素表示对应角点是否被正确跟踪到，1为正确跟踪，0为错误跟踪
    vector<float> error;                        // error表示误差
    t1 = chrono::steady_clock::now();           // 开始计时
    cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error);          // 调用opencv  calcOpticalFlowPyrLK函数来求解min = || I1(x,y) - I2(x + δx, y + δy) ||2 视觉slam十四讲p214式8.10
    t2 = chrono::steady_clock::now();           // 计时结束
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);   // 计算耗时
    cout << "optical flow by opencv: " << time_used.count() << endl;        // 输出使用opencv函数计算光流的耗时
    
    // *3. 绘制三种计算光流函数的效果
    // 单层光流法
    Mat img2_single;
    cv::cvtColor(img2, img2_single, COLOR_GRAY2BGR);//将灰度图img2转换成BGR彩色图img2_single，彩色图中BGR各颜色通道值为原先灰度值
    for (int i = 0; i < kp2_single.size(); i++) {
        if (success_single[i]) {
            cv::circle(img2_single, kp2_single[i].pt, 2, cv::Scalar(0, 250, 0), 2);     // 在img2_single图的特征点处画圈
            cv::line(img2_single, kp1[i].pt, kp2_single[i].pt, cv::Scalar(0, 250, 0));  // 在img2_single图中，从第一张图的特征点到第二张图的特征点之间画线
        }
    }
    
    // 多层光流法
    Mat img2_multi;
    cv::cvtColor(img2, img2_multi, COLOR_GRAY2BGR);
    for (int i = 0; i < kp2_multi.size(); i++) {
        if (success_multi[i]) {
            cv::circle(img2_multi, kp2_multi[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi, kp1[i].pt, kp2_multi[i].pt, cv::Scalar(0, 250, 0));
        }
    }
    
    // opencv自带的方法
    Mat img2_CV;
    cv::cvtColor(img2, img2_CV, COLOR_GRAY2BGR);
    for (int i = 0; i < pt2.size(); i++) {
        if (status[i]) {
            cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
        }
    }
    
     //画出角点连线图
    Mat imgMatches(img1.rows, img1.cols * 2, CV_8UC1);  //定义*行*列的Mat型变量
    Rect rect1 = Rect(0, 0, img1.cols, img1.rows);
    //Rect()有四个参数，第1个参数表示初始列，第2个参数表示初始行，
    //第3个参数表示在初始列的基础上还要加上多少列（即矩形区域的宽度），第4个参数表示在初始行的基础上还要加上多少行（即矩形区域的高度）
    Rect rect2 = Rect(img1.cols, 0, img2.cols, img2.rows);
    img1.copyTo(imgMatches(rect1));
    img2.copyTo(imgMatches(rect2));
    cv::imshow("tracked single level", img2_single);
    cv::imshow("tracked multi level", img2_multi);
    cv::imshow("tracked by opencv", img2_CV);
    
    cv::waitKey(0);
    return 0;
}
```

### 3.2 编译

```cmake
cmake_minimum_required(VERSION 3.0)
project(ch8)
 
set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++14 ${SSE_FLAGS} -g -O3 -march=native")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
list( APPEND CMAKE_MODULE_PATH /home/yang/3rdLibrary/g2o-20230223_git/cmake_modules/)
find_package(OpenCV 4 REQUIRED)
find_package(G2O REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(FMT REQUIRED)
 
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        "/usr/include/eigen3/"
        ${Pangolin_INCLUDE_DIRS}
)
 
add_executable(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS} fmt::fmt)
 
 
add_executable(direct_method direct_method.cpp)
target_link_libraries(direct_method ${OpenCV_LIBS} ${Pangolin_LIBRARIES} fmt::fmt)
```

## 4. 直接法

![image-20231025211209039](https://cdn.jsdelivr.net/gh/Fernweh-yang/ImageHosting@main/img/%E7%9B%B4%E6%8E%A5%E6%B3%95%E7%A4%BA%E6%84%8F%E5%9B%BE.png)

<center style="color:#C125C0C0">图2: 直接法示意图</center>

- **特征点法**：通过匹配描述子来知道p1对应的点p2在哪，然后根据**重投影误差**来优化位姿。

- **直接法的思路**：初始化一个不准确的位姿R,t，计算第一帧中像素坐标p1在第二帧中对应的像素坐标p2

  - 位姿R,t不准确时，显然p2和p1的光度会有很大的差别，因此我们基于光度的**灰度不变假设**通过**光度误差**来优化这个位姿
    $$
    e=\mathbf{I}_1(\mathbf{p_1})-\mathbf{I}_2(\mathbf{p}_2)\tag{1}
    $$

  - 位姿估计问题变为：
    $$
    \mathop{min}_T\ J(T)=\sum_{i=1}^Ne_i^Te_i,\ \ \ e_i=\mathbf{I}_1(\mathbf{p}_{1,i})-\mathbf{I}_2(\mathbf{p}_{2,i}) \tag{2}
    $$

    - N：N个空间点P
    - T：位姿，我们要优化的变量。
  
- **直接法分类**

  空间点P在直接法中是已知的，它的来源可以是：1. RGB-D相机中可以把任意像素反投影到三维空间。2. 双目相机中可以根据视察来计算像素的深度。3. 单目相机中需要考虑由P带来的深度不确定性。

  根据的P的来源，可以把直接法分类为：

  1. 稀疏直接法：P点来自稀疏关键点。

     使用数百上千个关键点，并像L-K光流一样假设它周围的像素不变。

  2. 半稠密(semi-dense)直接法：P来自部分梯度明显的像素

     在下面的式8中，如果像素梯度$\frac{\partial{\mathbf{I}_2}}{\partial{\mathbf{u}}}=0$，那么整个雅可比矩阵都是0，对计算增量没有帮助。因此可以舍弃像素梯度不明显的地方。

  3. 稠密直接法：P来自所有像素

     需要计算所有的像素，因此需要GPU加速。但是，像素梯度不明显的点对运动估计没什么贡献，在重构时也难以估计位置。
  
- 直接法的优缺点：

  优点：

  1. 省去计算特征点和描述子的时间
  2. 因为只要求有像素梯度就可以，所以特征缺失的场合下也可以使用
  3. 可以构建半稠密和稠密的地图，而特征点法只能是稀疏的。

  缺点：

  1. 非凸性：由于图像的非凸性，因此只有在相机运动很小的情况下，才能避免进入局部最小值。

     图像金字塔可以一定程度的减小这个缺点

  2. 单个像素没有区分度：和某一个像素的灰度值相近的像素太多了。

     因此需要：

     - 计算图像块(patch)
     - 计算复杂的相关性，如归一化相关性(归一化相关性Normalized cross correlation,NCC)

     并且由于每个像素的意见都是不一致的，因此一定要计算足够的像素点直接法才表现良好。通常需要500个点

  3. 灰度值不变是很强的假设：1.如果相机是自动曝光的，那么图像会整体变暗变亮。2.关照变化

     实用的直接法还需要估计相机的曝光参数，DSO-SLAM可以做到

### 4.1 公式推导

1. 要求解式2这个最小二乘问题，显然要求误差函数$e^Te$对于待优化变量的位姿T的导数

   观察误差项$e=\mathbf{I}_1(\mathbf{p_1})-\mathbf{I}_2(\mathbf{p}_2)\tag{1}$可知：

   - $\mathbf{I}_1$是第一帧的光度误差，这是我们已知的
   - 所以只需关心$\mathbf{I}_2$对位姿T的导数

2. 再看$\mathbf{I}_2$和什么有关

   - 和空间点在第二帧的像素坐标$p_2$有关
     $$
     \mathbf{u}=p_2=\left [\begin{array}{cccc}
     u \\
     v\\
     1 \\
     \end{array}\right]=\frac{1}{Z_2}\mathbf{K}(\underbrace{\mathbf{RP}+\mathbf{t}}_{\mathbf{q}})=\frac{1}{Z_2}\mathbf{K}(exp(\xi^{\wedge})\mathbf{P})_{1:3}\tag{3}
     $$

     - $\mathbf{q}$：空间点P在第二个相机坐标系下的坐标
     - $\mathbf{u}$：空间点P在第二帧图像中的像素坐标

   - 因此误差项变为：
     $$
     e=\mathbf{I}_1(\mathbf{p_1})-\mathbf{I}_2(\mathbf{u})\tag{4}
     $$

3. 结合3式4式，链式求导得到误差e和相机位姿T之间的导数关系
   $$
   \begin{align}
   J=\frac{\partial{e}}{\partial{\mathbf{T}}}&=-\frac{\partial{\mathbf{I}_2}}{\partial{\mathbf{T}}} \\
   &=-\frac{\partial{\mathbf{I}_2}}{\partial{\mathbf{u}}} \frac{\partial{\mathbf{u}}}{\partial{\mathbf{q}}}\frac{\partial{\mathbf{q}}}{\partial{\mathbf{\delta\xi}}}
   \end{align} \tag{5}
   $$

   - $\frac{\partial{\mathbf{I}_2}}{\partial{\mathbf{u}}}$：为u处的像素梯度

   - $\frac{\partial{\mathbf{u}}}{\partial{\mathbf{q}}}$：为**投影方程**关于**相机象素坐标系下三维点**的导数
     $$
     \frac{\partial\mathbf{u}}{\partial\mathbf{q}}=\left [\begin{array}{cccc}
     \frac{\partial{u}}{\partial{X}} & \frac{\partial{u}}{\partial{Y}} & \frac{\partial{u}}{\partial{Z}} \\
     \frac{\partial{v}}{\partial{X}} & \frac{\partial{v}}{\partial{Y}} & \frac{\partial{v}}{\partial{Z}} \\
     \end{array}\right]=\left [\begin{array}{cccc}
     \frac{f_x}{Z} & 0& -\frac{f_xX}{Z^2} \\
     0 & \frac{f_y}{Z} & -\frac{f_yY}{Z^2} \\
     \end{array}\right] \tag{6}
     $$
     
   - $\frac{\partial{\mathbf{q}}}{\partial{\mathbf{\delta\xi}}}$：为变换后的三维点对变换的导数
   
     根据李代数一章可知：
     $$
     \frac{\partial{\mathbf{q}}}{\partial{\mathbf{\delta\xi}}}=[\mathbf{I},-\mathbf{q}^{\wedge}] \tag{7}
     $$
   
4. 通常6,7式合并为一起，于是得到我们的雅可比矩阵
   $$
   J=-\frac{\partial{\mathbf{I}_2}}{\partial{\mathbf{u}}} \frac{\partial{\mathbf{u}}}{\partial{\mathbf{\delta\xi}}}\tag{8}
   $$

得到所有像素点的雅可比矩阵后，就可以用高斯牛顿或L-M方法计算增量方程，最后得到旋转矩阵T的微小增量$exp(\delta \xi^{\wedge})$。

### 4.2 迭代过程的理解

直接法完全依靠优化来求解相机位姿。要得到正确的优化结果，就要**保证大部分像素梯度能够把优化引导到正确的方向**

1. 先有第一帧，对于其中一个灰度值为229的像素，它的深度信息已知，作为第一张图相机内外参也已知，因此我们可以根据像素坐标->归一化坐标->相机坐标->世界坐标的步骤来推断出空间点P的位置。

2. 然后得到新的一帧，初始化一个一、二帧之间的的位姿变换T，这个T就是我们要求的相机位姿需要不断优化，从而得到正确的相机位姿T

3. 在这个初始的T下，由3式计算得到的第二帧像素坐标p2的灰度值为126

4. 由4式得到误差值为：229-126=103。为了减小这个误差，我们希望**微调相机的位姿，使像素更亮一些**

5. 那应该向哪里微调呢？这就需要局部的像素梯度即5式的第一部分$\frac{\partial{\mathbf{I}_2}}{\partial{\mathbf{u}}}$

   比如：

   - 沿着u轴(水平向右)走一个像素点，该处的灰度值为123，相比于126减小了3
   - 沿着v轴(竖直向下)走一个像素点，该处的灰度值为108，相比于126减小了18

   由此可知该处的梯度为[-3,-18]，因此为了提高亮度，需要向**左上方**移动。

6. 知道了所有像素应该往哪里微调后，先根据8式计算出他们的雅可比矩阵J，然后得到他们总体的海瑟矩阵H偏差b，最后根据高斯牛顿法或者L-M就可以得到这一次位姿的微小增量$exp(\delta \xi^{\wedge})$

7. 如此，得到了第一次迭代后的位姿$exp(\delta \xi^{\wedge})T$

理想情况下，不断循环上面这7步就可以降低光度误差(4式)最后收敛了。但现实中图像是一个很强烈的**非凸函数**，沿着像素梯度前进很容易进入一个局部最小值。

**因此，只有当相机运动很小，图像中的梯度没有很强的非凸性时，才能使用直接法。**

## 5. 实践：直接法

### 5.1 代码

```c++
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <boost/format.hpp>
#include <pangolin/pangolin.h>
 
using namespace std;
 
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
// Camera intrinsics 相机内参
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
// baseline   双目相机基线
double baseline = 0.573;
// paths 图像路径
string left_file = "../left.png";
string disparity_file = "../disparity.png";
boost::format fmt_others("../%06d.png");    // %06d确保输出的宽度为6个字符
 
// useful typedefs
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// ! class for accumulator jacobians in parallel  用于并行计算雅可比矩阵的类
class JacobianAccumulator {
public:
    //类的构造函数，使用列表进行初始化
    JacobianAccumulator(
        const cv::Mat &img1_,
        const cv::Mat &img2_,
        const VecVector2d &px_ref_,//角点坐标
        const vector<double> depth_ref_,//路标点的Z坐标值
        Sophus::SE3d &T21_) :
        img1(img1_), img2(img2_), px_ref(px_ref_), depth_ref(depth_ref_), T21(T21_) {
        projection = VecVector2d(px_ref.size(), Eigen::Vector2d(0, 0));
    }
 
    /// accumulate jacobians in a range 在range范围内加速计算雅可比矩阵
    void accumulate_jacobian(const cv::Range &range);
 
    /// get hessian matrix 获取海塞矩阵
    Matrix6d hessian() const { return H; }
 
    /// get bias 获取矩阵b
    Vector6d bias() const { return b; }
 
    /// get total cost  获取总共的代价
    double cost_func() const { return cost; }
 
    /// get projected points 获取图像2中的角点坐标
    VecVector2d projected_points() const { return projection; }
 
    /// reset h, b, cost to zero  将海塞矩阵H，矩阵b和代价cost置为0
    void reset() {
        H = Matrix6d::Zero();
        b = Vector6d::Zero();
        cost = 0;
    }
 
private:
    const cv::Mat &img1;
    const cv::Mat &img2;
    const VecVector2d &px_ref;      // 图像1中角点坐标
    const vector<double> depth_ref; // 图像1中路标点的Z坐标值
    Sophus::SE3d &T21;
    VecVector2d projection;         // projected points
 
    std::mutex hessian_mutex;
    Matrix6d H = Matrix6d::Zero();
    Vector6d b = Vector6d::Zero();
    double cost = 0;
};


/**
 * ! 多层直接法定义pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationMultiLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21
);


/**
 * ! 单层直接法定义pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationSingleLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21
);


// ! bilinear interpolation 双线性插值求灰度值
inline float GetPixelValue(const cv::Mat &img, float x, float y) //inline表示内联函数，它是为了解决一些频繁调用的小函数大量消耗栈空间的问题而引入的
{
    // boundary check
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= img.cols) x = img.cols - 1;
    if (y >= img.rows) y = img.rows - 1;
    //...|I1      I2|...
    //...|          |...
    //...|          |...
    //...|I3      I4|...
    uchar *data = &img.data[int(y) * img.step + int(x)];//x和y是整数 
    //data[0] -> I1  data[1] -> I2  data[img.step] -> I3  data[img.step + 1] -> I4
    float xx = x - floor(x);//xx算出的是x的小数部分
    float yy = y - floor(y);//yy算出的是y的小数部分
    return float//最终的像素灰度值
    (
        (1 - xx) * (1 - yy) * data[0] +
        xx * (1 - yy) * data[1] +
        (1 - xx) * yy * data[img.step] +
        xx * yy * data[img.step + 1]
    );
}

// ! 雅克比矩阵计算
void JacobianAccumulator::accumulate_jacobian(const cv::Range &range) {
    // parameters
    const int half_patch_size = 1;  // 设置窗口尺寸为3*3： -1,0,1
    int cnt_good = 0;
    Matrix6d hessian = Matrix6d::Zero();
    Vector6d bias = Vector6d::Zero();
    double cost_tmp = 0;

    for (size_t i = range.start; i < range.end; i++) {
        // * 1.compute the projection in the second image 
        // point_ref表示图像1中选取的像素点
        Eigen::Vector3d point_ref =
            depth_ref[i] * Eigen::Vector3d((px_ref[i][0] - cx) / fx, (px_ref[i][1] - cy) / fy, 1);
        // point_cur为像素点在第二帧相机坐标系下的坐标（T21是估计的
        Eigen::Vector3d point_cur = T21 * point_ref;
        // 若深度值为负，说明是异常点，跳过
        if (point_cur[2] < 0)   
            continue;
        // 求得在当前估计的T下，各个像素点在图2像素坐标。十四讲p99式5.5 
        float u = fx * point_cur[0] / point_cur[2] + cx, v = fy * point_cur[1] / point_cur[2] + cy;
        // 如果像素坐标在边缘，无法构成窗口计算雅克比矩阵，则跳过
        if (u < half_patch_size || u > img2.cols - half_patch_size || v < half_patch_size ||
            v > img2.rows - half_patch_size)
            continue;
        // 将有效投影点存入容器，作为直接法的初值
        projection[i] = Eigen::Vector2d(u, v);
        // 取出各个像素点在第二帧相机坐标系下的坐标，提前计算需要的中间值
        double X = point_cur[0], Y = point_cur[1], Z = point_cur[2],
            Z2 = Z * Z, Z_inv = 1.0 / Z, Z2_inv = Z_inv * Z_inv;// Z2_inv = (1 / (Z * Z))
        // 有效点计数
        cnt_good++;
        
        // *2. and compute error and jacobian   计算海塞矩阵H，矩阵b和代价cost -> 得到误差和雅可比矩阵
        // 分别计算3*3窗口内各点误差, -1,0,1
        for (int x = -half_patch_size; x <= half_patch_size; x++)
            for (int y = -half_patch_size; y <= half_patch_size; y++) {
                //ei = I1(p1,i) - I(p2,i)其中p1，p2空间点P在两个时刻的像素位置坐标，十四讲p219式8.13
                double error = GetPixelValue(img1, px_ref[i][0] + x, px_ref[i][1] + y) -
                               GetPixelValue(img2, u + x, v + y);
                Matrix26d J_pixel_xi;           // 像素梯度
                Eigen::Vector2d J_img_pixel;    // 灰度梯度
                // ** 根据十四讲p220式8.18计算雅可比矩阵的第二部分
                J_pixel_xi(0, 0) = fx * Z_inv;
                J_pixel_xi(0, 1) = 0;
                J_pixel_xi(0, 2) = -fx * X * Z2_inv;
                J_pixel_xi(0, 3) = -fx * X * Y * Z2_inv;
                J_pixel_xi(0, 4) = fx + fx * X * X * Z2_inv;
                J_pixel_xi(0, 5) = -fx * Y * Z_inv;
                J_pixel_xi(1, 0) = 0;
                J_pixel_xi(1, 1) = fy * Z_inv;
                J_pixel_xi(1, 2) = -fy * Y * Z2_inv;
                J_pixel_xi(1, 3) = -fy - fy * Y * Y * Z2_inv;
                J_pixel_xi(1, 4) = fy * X * Y * Z2_inv;
                J_pixel_xi(1, 5) = fy * X * Z_inv;
                // ** 计算雅可比矩阵的第一部分：像素梯度
                /*
                    dx,dy是优化变量 即（Δu，Δv） 计算像素梯度
                    相当于 J = - [ {I1( u + i + 1,v + j )-I1(u + i - 1,v + j )}/2,I1( u + i,v + j + 1)-I1( u + i ,v + j - 1)}/2]T T表示转置
                    I1 -> 图像1的灰度信息
                    i -> x
                    j -> y
                */
                J_img_pixel = Eigen::Vector2d(
                    0.5 * (GetPixelValue(img2, u + 1 + x, v + y) - GetPixelValue(img2, u - 1 + x, v + y)),
                    0.5 * (GetPixelValue(img2, u + x, v + 1 + y) - GetPixelValue(img2, u + x, v - 1 + y))
                );
                
                // ** 根据十四讲p220式8.19将两部分合并起来
                Vector6d J = -1.0 * (J_img_pixel.transpose() * J_pixel_xi).transpose();
                hessian += J * J.transpose();
                bias += -error * J;
                cost_tmp += error * error;
            }
    }
    // 如果存在有效点，将结果保存到class中
    if (cnt_good) {
        // set hessian, bias and cost
        unique_lock<mutex> lck(hessian_mutex);
        H += hessian;               // H = Jij Jij(T)(累加和)
        b += bias;                  // b = -Jij * eij(累加和)
        cost += cost_tmp / cnt_good;// cost = || eij ||2 2范数
    }
}


// ! 单层直接法实现
void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,          // 第1张图中随机选取的像素坐标
        const vector<double> depth_ref,     // 第1张图中随机选取的像素坐标的深度值
        Sophus::SE3d &T21){                 // 要优化求解的位姿
    
    const int iterations = 10;              // 设置迭代次数为10
    double cost = 0, lastCost = 0;          // 将代价和最终代价初始化为0W
    
    auto t1 = chrono::steady_clock::now();  // 开始计时
    JacobianAccumulator jaco_accu(img1, img2, px_ref, depth_ref, T21);  // 用于计算雅可比矩阵的类
    
    for (int iter = 0; iter < iterations; iter++) {
        jaco_accu.reset();  // 重置雅可比矩阵

        // 并行计算雅可比矩阵，得到海塞矩阵H，矩阵b和代价cost
        cv::parallel_for_(cv::Range(0, px_ref.size()),
                          std::bind(&JacobianAccumulator::accumulate_jacobian, &jaco_accu, std::placeholders::_1));
        Matrix6d H = jaco_accu.hessian();   // 计算海塞矩阵
        Vector6d b = jaco_accu.bias();      // 计算b矩阵

        Vector6d update = H.ldlt().solve(b);;   // 增量方程的求解结果是李代数，不过是向量形式的
        T21 = Sophus::SE3d::exp(update) * T21;  // 更新待优化变量，左扰动，因此将增量的变换矩阵左乘迭代
        cost = jaco_accu.cost_func();
        if (std::isnan(update[0])) //解出来的更新量不是一个数字，退出迭代
        {
            // sometimes occurred when we have a black or white patch and H is irreversible
            cout << "update is nan" << endl;
            break;
        }
        if (iter > 0 && cost > lastCost) //代价不再减小，退出迭代 
        {
            cout << "cost increased: " << cost << ", " << lastCost << endl;
            break;
        }
        if (update.norm() < 1e-3) //更新量的模小于1e-3，退出迭代
        {
            // converge
            break;
        }
        lastCost = cost;
        cout << "iteration: " << iter << ", cost: " << cost << endl;
    }//GN(高斯牛顿法)迭代结束

    cout << "T21 = \n" << T21.matrix() << endl;                                 // 输出T21矩阵
    auto t2 = chrono::steady_clock::now();                                      // 计时结束
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);  // 计算耗时
    cout << "direct method for single layer: " << time_used.count() << endl;    // 输出使用单层直接法所用时间
    
    // plot the projected pixels here
    cv::Mat img2_show;
    cv::cvtColor(img2, img2_show, cv::COLOR_GRAY2BGR);
    VecVector2d projection = jaco_accu.projected_points();
    for (size_t i = 0; i < px_ref.size(); ++i) {
        auto p_ref = px_ref[i];
        auto p_cur = projection[i];
        if (p_cur[0] > 0 && p_cur[1] > 0) {
            cv::circle(img2_show, cv::Point2f(p_cur[0], p_cur[1]), 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_show, cv::Point2f(p_ref[0], p_ref[1]), cv::Point2f(p_cur[0], p_cur[1]),
                     cv::Scalar(0, 250, 0));
        }
    }
    cv::imshow("current", img2_show);
    cv::waitKey();
}

// ! 多层直接法
// 和多层光流法类似
void DirectPoseEstimationMultiLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,          // 第1张图中随机选取的像素坐标
    const vector<double> depth_ref,     // 第1张图中随机选取的像素坐标的深度值
    Sophus::SE3d &T21) {                // 要优化求解的位姿
    // parameters
    int pyramids = 4;                   // 金字塔层数为4
    double pyramid_scale = 0.5;         // 每层之间的缩放因子设为0.5
    double scales[] = {1.0, 0.5, 0.25, 0.125};
    
    // * create pyramids 创建图像金字塔
    vector<cv::Mat> pyr1, pyr2; // image pyramids pyr1 -> 图像1的金字塔 pyr2 -> 图像2的金字塔
    for (int i = 0; i < pyramids; i++) {
        if (i == 0) {   // 最底层是原始图像
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        } else {
            cv::Mat img1_pyr, img2_pyr;
            
            //使用resize进行下采样： 将图像pyr1[i-1]的宽和高各缩放0.5倍得到图像img1_pyr
            cv::resize(pyr1[i - 1], img1_pyr,
                       cv::Size(pyr1[i - 1].cols * pyramid_scale, pyr1[i - 1].rows * pyramid_scale));
            cv::resize(pyr2[i - 1], img2_pyr,
                       cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
            pyr1.push_back(img1_pyr);
            pyr2.push_back(img2_pyr);
        }
    }
    // * 更新每一层的相机内参，并对每一层图像实施单层直接法
    double fxG = fx, fyG = fy, cxG = cx, cyG = cy;  // backup the old values 备份旧值
    for (int level = pyramids - 1; level >= 0; level--) {
        // ** 将从第一张图中随机选取的像素坐标，同样缩放到图像金字塔对应的尺度
        VecVector2d px_ref_pyr;      
        for (auto &px: px_ref) {
            px_ref_pyr.push_back(scales[level] * px);
        }
        // ** 相机内参同样需要缩放
        fx = fxG * scales[level];
        fy = fyG * scales[level];
        cx = cxG * scales[level];
        cy = cyG * scales[level];
        // ** 调用单层直接法
        DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);
    }
}

int main(int argc, char **argv) {
 
    cv::Mat left_img = cv::imread(left_file, 0);            // 0表示返回灰度图，读取第一张图像left.png
    cv::Mat disparity_img = cv::imread(disparity_file, 0);  // 0表示返回灰度图，disparity.png是left.png的视差图
    
    // * 在图像1中随机选择2000个像素点，计算他们对应的深度
    // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
    cv::RNG rng;
    int nPoints = 2000;
    int boarder = 20;
    VecVector2d pixels_ref;     // 像素坐标
    vector<double> depth_ref;   // 像素深度
    // generate pixels in ref and load depth data
    for (int i = 0; i < nPoints; i++) {
        int x = rng.uniform(boarder, left_img.cols - boarder);  // don't pick pixels close to boarder 不要拾取靠近边界的像素 
        int y = rng.uniform(boarder, left_img.rows - boarder);  // don't pick pixels close to boarder 不要拾取靠近边界的像素 
        int disparity = disparity_img.at<uchar>(y, x);
        double depth = fx * baseline / disparity; // you know this is disparity to depth
        depth_ref.push_back(depth);
        pixels_ref.push_back(Eigen::Vector2d(x, y));
    }
    
    // * 利用直接法计算5张图的相机位姿
    // estimates 01~05.png's pose using this information
    Sophus::SE3d T_cur_ref;     // SE3d特殊欧几里德群，T_cur_ref表示当前帧（cur）相对于参考帧（ref）的位姿变换T。
    for (int i = 1; i < 6; i++) // 1~5 i从1到5，共5张图
    {  
        // cout << fmt_others % i << endl;
        cv::Mat img = cv::imread((fmt_others % i).str(), 0);//读取图片，0表示返回一张灰度图
        // ** 单层直接法
        // 计算图像img相对于left_img的位姿T_cur_ref，以图片left.png为基准
        DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
        // ** 多层直接法
        // DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
    }
    return 0;
}
```



### 5.2 编译

同3.2

```c++
cmake_minimum_required(VERSION 3.0)
project(ch8)
 
set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++14 ${SSE_FLAGS} -g -O3 -march=native")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
list( APPEND CMAKE_MODULE_PATH /home/yang/3rdLibrary/g2o-20230223_git/cmake_modules/)
find_package(OpenCV 4 REQUIRED)
find_package(G2O REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(FMT REQUIRED)
 
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        "/usr/include/eigen3/"
        ${Pangolin_INCLUDE_DIRS}
)
 
add_executable(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS} fmt::fmt)
 
 
add_executable(direct_method direct_method.cpp)
target_link_libraries(direct_method ${OpenCV_LIBS} ${Pangolin_LIBRARIES} fmt::fmt)
```



# 九、后端优化：BA图优化

前端里程计能给出一个短时间内的轨迹和地图，但由于不可避免的误差累积，这个地图在长时间内是不准确的。

所以我们需要构建一个更大尺度和规模的优化问题，以考虑长时间内的最优轨迹和地图。

本质上可将后端优化看作一个**状态估计问题**：如何根据带有噪声的观测求系统状态的最优估计，在实现方式上分为滤波器和非线性优化法。

## 1. 概述+卡尔曼滤波

### 1.1 状态估计的概率解释

- 我们用“批量的”信息处理方式，来保证整个运动轨迹在较长时间内都能保持最优状态

  - 批量的batch:不仅使用过去的信息更新自己的状态，也用未来的信息来更新以前的某一状态。
  - 渐进的incremental：当前的状态只由过去的时刻决定。用滤波器求解状态估计就是这种情况。

- SLAM问题: 研究定位（Localization）问题和建图（Mapping）问题

  而对位姿$\mathbf{x}$的估计就对应着定位问题(L)，对路标$\mathbf{y}$的估计就对应着建图问题(M)

  - 由于噪声，位姿$\mathbf{x}$和路标$\mathbf{y}$可以看成服从某种概率分布的随机变量。所以问题变成了**状态估计问题**：当存在一些运动数据$\mathbf{u}$和观测数据$\mathbf{z}$时，如何估计状态变量x/y的高斯分布。

  - 而由六，1.1可知：批量状态估计问题可以转化为**最大似然估计问题**，并使用**最小二乘法**进行求解

- 问题的定量推导：

  1. 定义变量$\mathbf{x}_k=\{\mathbf{x}_k,\mathbf{y}_1,\cdots,\mathbf{y}_m\}$为k时刻所有未知量，包含当前时刻位姿x和m个路标点y。$\mathbf{z}_k$为k时刻所有观测。得到运动方程(第一行)/观测方程(第二行)
     $$
     \begin{cases}
     \mathbf{x}_k=f(\mathbf{x}_{k-1},\mathbf{u}_k)+\mathbf{w}_k \ \ \ k=1,\cdots,N\\
     \mathbf{z}_{k,j} = h(\mathbf{y}_j,\mathbf{x}_k)+\mathbf{v}_{k,j}\ \ \ j=1,\cdots,m \\
     \end{cases} \tag{1}
     $$

     - 运动方程(第一行)是描述机器人从一个时刻到另一个时刻的运动关系的方程。可以用来估计机器人的位置和姿态的变化。
       - $\mathbf{u}$：表示运动量，即机器人内部运动传感器读数，比如IMU/GPS/位移传感器等的读数
       - 如果没有运动传感器，就是纯视觉方法，可以假设相机不动/相机匀速运动/或直接没有运动方程
     - 观测方程(第二行)描述了传感器从机器人当前位置和姿态中获取到的信息。在视觉SLAM中，观测方程可以是相机对地图特征点的观测。
       - $\mathbf{z}$：观测量，通常这个量由双目摄像头、激光雷达等获得
       - 观测方程是对某一个观测路标(特征点)的测量，而路标点有m个，所以$z_{k,j}$表示在k时刻对路标点j的观测值
     - 视觉SLAM中，只有当机器人在位姿$x_k$处看到路标(特征点)时才会产生观测数据。而特征点很多，所以实际中观测方程的数量远远大于运动方程。
     
  2. 我们希望用过去0到k时刻的数据来估计现在的状态分布（等式左边）,然后利用贝叶斯法则得到贝叶斯估计（等式右边）：
     $$
     P(\mathbf{x}_k|\mathbf{x}_0,\mathbf{u}_{1:k},\mathbf{z}_{1:k})\propto P(\mathbf{z}_k|\mathbf{x}_k)P(\mathbf{x}_k|\mathbf{x}_0,\mathbf{u}_{1:k},\mathbf{z}_{1:k-1}) \tag{2}
     $$

     - 等式右边的第一项是**似然**，第二项是**先验**
     - 贝叶斯估计：利用似然和先验来得到后验估计
     
  3. 在先验部分，$\mathbf{x}_k$基于过去所有状态估计得来，现在考虑k-1时刻，以$\mathbf{x}_{k-1}$为条件概率展开
     $$
     P(\mathbf{x}_k|\mathbf{x}_0,\mathbf{u}_{1:k},\mathbf{z}_{1:k-1})=\displaystyle \int P(\mathbf{x}_k|\mathbf{x}_{k-1},\mathbf{x}_0,\mathbf{u}_{1:k},\mathbf{z}_{1:k-1})P(\mathbf{x}_{k-1}|\mathbf{x}_0,\mathbf{u}_{1:k},\mathbf{z}_{1:k-1})d\mathbf{x}_{k-1} \tag{3}
     $$

- 由于3式没有具体的概率分布形式，需要进一步后续处理，这里由2种方案：

  1. 以**扩展卡尔曼滤波(EKF)**为代表的滤波方法。

     需要假设马尔可夫性，即当前状态只和前1时刻状态有关。

  2. **非线性优化**

     当前状态取决于之前所有时刻的状态。这也是vslam中主流的方法。
### 1.2 线性系统和KF

- 问题描述

  考虑一个线性高斯系统：它的运动/观测方程由线性方程描述，噪声和状态变量均满足高斯分布：
  $$
  \begin{cases}
  \mathbf{x}_k=\mathbf{A}_k\mathbf{x}_{k-1}+\mathbf{u}_k+\mathbf{w}_k \\
  \mathbf{z}_k = \mathbf{C}_k\mathbf{x}_k+\mathbf{v}_k \\
  \end{cases}\ \ \ k=1,\cdots,N\tag{4}它的噪声服从零均值高斯分布：
  $$
  它的噪声服从零均值高斯分布：
  $$
  \mathbf{w}_k \sim N(\mathbf{0},\mathbf{R}),\ \mathbf{v}_k\sim N(\mathbf{0},\mathbf{Q}) \tag{5}
  $$
  $\mathbf{A}_k$：状态转移矩阵

  $\mathbf{u}_k$：控制向量

  $\mathbf{C}_k$：观测矩阵，将状态空间映射到观测空间

  $\mathbf{R}$：过程噪声的协方差矩阵 

  $\mathbf{Q}$：测量误差的协方差矩阵

  下面使用卡尔曼滤波器将k-1时刻的状态分布推导至k时刻，最终得到线性系统的最优无偏估计

  - 由于基于马尔可夫性假设，所以在实际编程中，我们只需要维护一个状态变量
  - 而又由于状态变量服从高斯分布，我们只需要维护状态变量的均值矩阵$\hat{\mathbf{x}}_k$和协方差矩阵$\hat{\mathbf{P}}_k$

- 卡尔曼滤波由**5个方程+2个阶段**组成，并用$\hat{a}$表示后验，$\check{a}$表示先验分布：

  1. **预测**(prediction)

     从上一时刻的状态，根据输入信息（有噪声）推断当前时刻的状态分布。6.1式预测状态估计, 6.2式预测先验状态误差协方差
     $$
     \begin{align}
     \check{\mathbf{x}}_k=\mathbf{A}_k\hat{\mathbf{x}}_{k-1}+\mathbf{u}_k \tag{6.1}\\
     \check{\mathbf{P}}_k=\mathbf{A}_k\hat{\mathbf{P}}_{k-1}\mathbf{A}_k^T+\mathbf{R} \tag{6.2}
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
       \mathbf{K}=\check{\mathbf{P}}_k\mathbf{C}_k^T(\mathbf{C}_k\check{\mathbf{P}}_k\mathbf{C}_k^T+\mathbf{Q}_k)^{-1}\tag{7}
       $$
  
     - 然后计算后验概率分布, 8.1式更新状态，8.2式更新后验误差协方差
       $$
       \begin{align}
       \hat{\mathbf{x}}_k=\check{\mathbf{x}}_k+\mathbf{K}(\mathbf{z}_k-\mathbf{C}_k\check{\mathbf{x}}_k)\tag{8.1}\\
       \hat{\mathbf{P}}_k=(\mathbf{I}-\mathbf{KC}_k)\check{\mathbf{P}}_k \tag{8.2}
       \end{align}
       $$

- 5个方程的物理意义

  - 6.1式：状态方程，通过它直接可以根据前一个状态估计一下当前状态。

  - 6.2式：先验误差协方差方程，通过它可以看到由状态方程预测的状态和真实状态之间的方差（不确定性，可靠性）有多少

  - 7式：约束优化条件，使后验误差协方差$\hat{\mathbf{P}}_k$最小，得到卡尔曼增益K

  - 8.1式：状态更新，得到**我们想要的最佳状态估计**

    通过该式可以发现，Kalman滤波就是一个加权平均的过程：

    对态估计和测量估计的加权平均：哪个估计的不确定性低(更可靠)哪个的权重就大，这个权重在8.1式的体现就是卡尔曼增益K。

  - 8.2式：后验误差协方差方程$\hat{\mathbf{P}}_k$，在先验误差协方差方程$\check{\mathbf{P}}_k$的基础上加入测量过程$\mathbf{C}_k$的影响，更新下一时刻的先验值$\check{\mathbf{P}}_{k+1}$

### 1.3 非线性系统和EKF

在SLAM中运动/观测方程通常都是非线性的，而卡尔曼滤波KF只能用于线性系统，对于非线性系统需要使用先在某点附近对运动/观测方程进行一阶泰勒展开，保留一阶项以近似成线性发成，最后使用扩展卡尔曼滤波EKF进行无偏最优估计

- 扩展卡尔曼滤波EKF同样由2部分组成

  1. **预测**：
     $$
     \check{\mathbf{x}}_k=f(\hat{\mathbf{x}}_{k-1},\mathbf{u}_k)\\
     \check{\mathbf{P}}_k=\mathbf{F}\hat{\mathbf{P}}_{k-1}\mathbf{F}^T+\mathbf{R}_k \tag{9}
     $$

  2. **更新**：

     - 先计算卡尔曼增益$K_k$
       $$
       \mathbf{K}_k=\check{\mathbf{P}}_k\mathbf{H}^T(\mathbf{H}\check{\mathbf{P}}_k\mathbf{H}^T+\mathbf{Q}_k)^{-1}\tag{10}
       $$

     - 然后计算后验概率分布：
       $$
       \hat{\mathbf{x}}_k=\check{\mathbf{x}}_k+\mathbf{K}_k(\mathbf{z}_k-h(\check{\mathbf{x}}_k))\\
       \hat{\mathbf{P}}_k=(\mathbf{I}-\mathbf{K}_k\mathbf{H})\check{\mathbf{P}}_k \tag{11}
       $$
       

### 1.4 EKF的讨论

当想要在某段时间内估计某个不确定量时，EKF是个很好的方法。但在计算资源充足或估计量复杂的场合，非线性优化是种更好的方法。

EKF有着如下局限性：

1. 滤波器基于马尔可夫性，这就导致了只能使用前一时刻的数据，而以前的数据都被忽略了。
2. 一节泰勒展开无法完全线性近似非线性方程，从而产生非线性误差。
3. EKF需要维护存储状态变量的均值和方差，在大场景中，如果把路标也放进状态，就使得这个存储规模呈平方增长，所以不适合大场景
4. EKF没有异常检测机制，导致系统在异常值时容易发散。

所以在相同计算量下，非线性优化通常比EKF在精度和鲁棒性上更好。



### 1.5 一个DEMO代码

用于估计小汽车一维直线运动状态的kalman滤波器，状态包括位置p和速度v，观测值也是位置p和速度v。

```matlab
% 初始化状态变量
p = 0; % 位置
v = 80; % 速度，假设为80m/s

% 状态向量
x = [p; v];

% 状态转移矩阵F
dt = 1; % 时间间隔，假设为1秒
F = [1 dt; 0 1]; % 假设匀速运动

% 观测矩阵
H = [1 0; 0 1]; % 直接观测位置和速度

% 过程噪声协方差矩阵
Q = [1 0; 0 1];

% 观测噪声协方差矩阵
R = [10 0; 0 10]; % 假设观测噪声较大

% 初始估计误差协方差
P = [1 0; 0 1];

% 模拟数据
% 真实位置和速度（不可观测）
true_p = 0;
true_v = 80;

% 初始化记录真实值的数组
true_positions = zeros(1, 50);
true_velocities = zeros(1, 50);

% 运行Kalman滤波器
for k = 1:50
    % 预测
    x = F * x;
    P = F * P * F' + Q;
    
    % 更新观测数据
    true_p = true_p + true_v * dt;
    true_v = true_v; % 假设真实速度不变
    obs_p = true_p + randn*sqrt(R(1,1));
    obs_v = true_v + randn*sqrt(R(2,2));
    
    % 更新
    Z = [obs_p; obs_v]; % 观测值
    y = Z - H * x; % 观测残差
    S = H * P * H' + R; % 残差协方差
    K = P * H' / S; % 卡尔曼增益
    x = x + K * y; % 更新状态估计
    P = (eye(2) - K * H) * P; % 更新估计误差协方差
    
    % 保存数据用于绘图
    positions(k) = x(1);
    velocities(k) = x(2);
    true_positions(k) = true_p;
    true_velocities(k) = true_v;
end

% 绘制结果
t = 1:k;
figure;
subplot(2,1,1);
plot(t, positions, 'r', 'LineWidth', 2);
hold on;
plot(t, true_positions, 'k--', 'LineWidth', 2);
hold off;
title('位置估计');
xlabel('时间 (秒)');
ylabel('位置 (米)');
legend('估计位置', '真实位置');

subplot(2,1,2);
plot(t, velocities, 'r', 'LineWidth', 2);
hold on;
plot(t, true_velocities, 'k--', 'LineWidth', 2);
hold off;
title('速度估计');
xlabel('时间 (秒)');
ylabel('速度 (米/秒)');
legend('估计速度', '真实速度');
```

## 2. BA与图优化

- 什么是Bundle Adjustment（光束法平差/捆绑调整）：

  - 从视觉图像中提炼出最优的3D模型和相机参数(内外参)。

  - 考虑从任意特征点发射出来的几束光线(Bundles of light rays)，他们会在几个相机的成像平面上变成像素或是检测到的特征点。

    如果我们调整(adjustment)各相机姿态和各特征点的空间位置，使得这些光线最终收束到相机的光心，就称为BA。

### 2.1 投影模型和BA代价函数

- 投影模型：

  从一个世界坐标系中的点$p$出发，把相机内外参数和畸变都考虑近来，最后投影成像素坐标的示意图：

  ![](https://cdn.jsdelivr.net/gh/Fernweh-yang/ImageHosting@main/img/%E6%8A%95%E5%BD%B1%E5%9D%90%E6%A0%87%E8%AE%A1%E7%AE%97.png)

  - 步骤：

    1. 由外参$\mathbf{R},\mathbf{t}$将世界坐标$p$转为相机坐标$p'$

    2. 将$p'$投至归一化平面，得到归一化坐标$p_c=[u_c,v_c,1]$

    3. 去除由于归一化引起的畸变，得到$u'_c,v'_c$

    4. 根据内参$f_x,f_y,c_x,c_y$计算得到像素坐标$u_s,v_s$
  
  - 这个过程也就是前面说的**观测方程**$z=h(x,y)=[u_s,v_s]^T$
  
    位置x在这里指的是相机的位姿，即外参$\mathbf{R,t}$,李群为$\mathbf{T}$。
  
    路标y即三维点$p$。
  
    观测数据z是像素坐标$z=[u_s,v_s]^T$
  
- 从最小二乘法的角度来考虑，此次观测的**误差**为：
  $$
  \mathbf{e}=\mathbf{z}-h(\mathbf{T},\mathbf{p}) \tag{1}
  $$

  - $\mathbf{z}$：由相机/雷达等得到的观测数据$z=[u_s,v_s]^T$，也就是所谓的真值。
  
    这个值是由**特征匹配**得到的，首先会在第一张图像中提取特征点。然后，在第二张图像中寻找与第一张图像中特征点具有相同特征的点。如果找到了匹配的特征点，那么这两个特征点在两张图像中的像素坐标就可以作为重投影误差的真实值。
  
  - h：由待优化变量位姿T和路标p计算得到的估计值
  
    注意这里的位姿T：是世界坐标系到第二张图像时相机坐标系的变换矩阵**T**（即相机位姿）
  
    这里的路标点p：是两张图片相匹配的特征点在世界坐标系下的坐标。
  
- 整体的**代价函数**为：
  $$
  \frac{1}{2}\sum_{i=1}^m\sum_{j=1}^n||\mathbf{e}_{ij}||^2=\frac{1}{2}\sum_{i=1}^m\sum_{j=1}^n||\mathbf{z}_{ij}-h(\mathbf{T}_i,\mathbf{p}_j)||^2 \tag{2}
  $$

  - $\mathbf{z}_{ij}$为在位姿$\mathbf{T}_i$处观察路标$\mathbf{p}_j$产生的数据。

  **对2式这个最小二乘问题进行求解，相当于对位姿T和路标p同时做了调整，也就是所诶的BA**

### 2.2 BA的求解

2.1的2式显然是一个非线性方程，需要用第六章中的非线性优化方法来优化它。

- 根据非线性优化的思想，我们需要从某个初始值开始，不断地寻找下降方向$\Delta x$，来找到目标函数的最优解，即不断求解增量方程（第六章2.2的8式[[Slam十四讲笔记#2.2 高斯牛顿法：]]）:
  $$
  \mathbf{H}\Delta x=g \tag{1}
  $$
  
- 尽管误差项(2.1的1式)都是针对单个位姿和路标的，但在整体BA目标函数上，自变量应该定义成所有待优化的变量:m个时刻的位姿T, n个路标点，通常n>>m
  $$
  \mathbf{x}=[\mathbf{T}_1,\cdots,\mathbf{T}_m,\mathbf{p}_1,\cdots,\mathbf{p}_n]^T\tag{2}
  $$
  
- 因此这里的增量方程$\Delta x$也是对整体自变量的增量，从而最小二乘法(第六章 2)的目标函数为：
  $$
  \frac{1}{2}||f(x+\Delta x)||^2\approx\frac{1}{2}\sum_{i=1}^m\sum_{j=1}^n||e_{ij}+\mathbf{F}_{ij}\Delta \xi_i+\mathbf{E}_{ij}\Delta p_j||^2 \tag{3}
  $$

  - $\mathbf{F}_{ij}$：整个代价函数(2.1的2式)在当前状态下对**相机姿态的偏导**，具体展开式见[[Slam十四讲笔记#七、视觉里程计：特征点法#6.3 最小化重投影误差(BA,非线性优化法)|第七章6.3的4式]]
  - $\mathbf{E}_{ij}$：整个代价函数(2.1的2式)对**路标点位置的偏导**，具体展开式见[[Slam十四讲笔记#七、视觉里程计：特征点法#6.3 最小化重投影误差(BA,非线性优化法)|第七章6.3的5式]]
  
- 将相机位姿变量放在一起:$x_c=[\xi_1,\xi_2,\cdots,\xi_m]^T\in\mathbb{R}^{6m}$。空间点(路标点)变量放一起$x_p=[p_1,p_2,\cdots,p_n]^T\in\mathbb{R}^{3n}$。

  3式由此从多个小型二次项和简化为矩阵形式：
  $$
  \frac{1}{2}||f(x+\Delta x)||^2=\frac{1}{2}||\mathbf{e}+\mathbf{F}\Delta \mathbf{x}_c+\mathbf{E}\Delta \mathbf{x}_p||^2 \tag{4}
  $$

  - 雅可比矩阵$\mathbf{E/F}$是整体目标函数对整体变量的导数，是一个很大的矩阵，这个矩阵又由每个误差项的导数矩阵$\mathbf{E}_{ij}，\mathbf{F}_{ij}$组合而成。
  - 最后不管用高斯牛顿法还是LM法，都要面对增量线性方程：$\mathbf{H}\Delta=g$

- 4式也即[[Slam十四讲笔记#六、非线性优化#2.2 高斯牛顿法：|第六章2.2的6式]]

  海塞矩阵$\mathbf{H}$ 在高斯牛顿法中是$J^TJ$,在LM法中是$J^TJ+\lambda I$
  
  由于我们在3式中，将变量归类成了位姿和空间点两种，所以雅可比矩阵也被分块为：
  $$
  \mathbf{J}=[\mathbf{F}\ \mathbf{E}] \tag{5}
  $$
  从而高斯牛顿法的海塞矩阵H为：
  $$
  \mathbf{H}=\mathbf{J}^T\mathbf{J}=\left [\begin{array}{cccc}
  \mathbf{F}^T\mathbf{F} & \mathbf{F}^T\mathbf{E} \\
  \mathbf{E}^T\mathbf{F} & \mathbf{E}^T\mathbf{E}   \\
  \end{array}\right] \tag{6}
  $$
  如果是L-M方法，$\mathbf{H}=\mathbf{J}^T\mathbf{J}+\lambda\mathbf{I}$

### 2.3 稀疏性和边缘化

计算2.2的6式的逆需要大量的计算资源，幸好矩阵$\mathbf{H}$具有稀疏结构，这个结构可以自然地、显示地用图优化来表示。

- $\mathbf{H}$矩阵地稀疏性由雅可比矩阵$\mathbf{J}(x)引起$：代价函数中的某一个误差项$\mathbf{e}_{ij}$只描述了在$\mathbf{T}_i$看到$\mathbf{p}_j$这件事，因此只涉及第i个相机位姿和第j个路标点，对其余部分的变量的导数自然就是0了，因此该项的雅可比矩阵可以写成：
  $$
  \mathbf{J}_{ij}(\mathbf{x})=(\mathbf{0}_{2\times6},\cdots,\mathbf{0}_{2\times6},\frac{\partial\mathbf{e}_{ij}}{\partial\mathbf{T}_i},\mathbf{0}_{2\times6},\cdots,\mathbf{0}_{2\times3},\cdots,\mathbf{0}_{2\times3},\frac{\partial\mathbf{e}_{ij}}{\partial\mathbf{p}_j},\mathbf{0}_{2\times3},\cdots,\mathbf{0}_{2\times3})\tag{1}
  $$

  - $\mathbf{0}_{2\times6},\mathbf{0}_{2\times3}$分别表示维度为2x6和2x3的零矩阵。
  - $\frac{\partial\mathbf{e}_{ij}}{\partial\mathbf{T}_i}$：该误差项对位姿的偏导维度为2x6
  - $\frac{\partial\mathbf{e}_{ij}}{\partial\mathbf{p}_j}$：该误差项对路标点的偏导维度为2x3

  - 除了上面这2个地方，其他地方都是零。这体现了该误差项与其他路标和轨迹无关的特性。
  - 从图优化的角度来说，这条观测边(误差项)只和这两个顶点(优化变量)有关。

- 每个误差项的雅可比矩阵$\mathbf{J}_{ij}(\mathbf{x})$对$\mathbf{H}$的贡献如图所示：

  ![](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/202305260158513.png)

  <center style="color:#C125C0C0">图1</center>

  - 当某个误差项$\mathbf{J}$具有稀疏性时(只在i,j处非0)，它对$\mathbf{H}=\sum_{i.j}\mathbf{J}_{ij}^T\mathbf{J}_{ij}$的贡献同样也具有稀疏性(在ii,ij,ji,jj四个地方非0)

    注意：i在所有相机位姿中取值，j在所有路标点中取值

  - $\mathbf{H}$因此可以分成四块：
    $$
    \mathbf{H}=\left [\begin{array}{cccc}
    \mathbf{H}_{11} & \mathbf{H}_{12}    \\
    \mathbf{H}_{21} & \mathbf{H}_{22}   \\
    \end{array}\right] \tag{2}
    $$

    - $\mathbf{H}_{11}$：不管i,j怎么变，都是对角阵，只在$\mathbf{H}_{ii}$处有非零块。
    - $\mathbf{H}_{22}$：同样是对角阵，只在$\mathbf{H}_{jj}$处有非零块
    - $\mathbf{H}_{12}$和$\mathbf{H}_{21}$可能是西苏的也可能是稠密的，由具体的观测数据确定

- 考虑一个场景：2个相机位姿$(C_1,C_2)$和6个路标点$(P_1,P_2,P_3,P_4,P_5,P_6)$，$C_1$可以观测到$P_1,P_2,P_3,P_4$。$C_2$可以观测到$P_3,P_4,P_5,P_6$。

  相机$C_1$对路标点$P_1$的误差$e_{11}$的雅可比矩阵$J_{11}$的稀疏矩阵为：
  $$
  \mathbf{J}_{11}=\frac{\partial\mathbf{e}_{11}}{\partial\mathbf{x}}=(\frac{\partial\mathbf{e}_{11}}{\partial\mathbf{\xi}_1},\mathbf{0}_{2\times6},\frac{\partial\mathbf{e}_{11}}{\partial\mathbf{p}_1},\mathbf{0}_{2\times3},\mathbf{0}_{2\times3},\mathbf{0}_{2\times3},\mathbf{0}_{2\times3},\mathbf{0}_{2\times3}) \tag{3}
  $$
  它的稀疏图案可以表达为：

  ![image-20230526030111834](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/202305260301935.png)

  <center style="color:#C125C0C0">图2</center>

  它对应的H矩阵稀疏情况可以得到下图所示：

  ![image-20230526025558601](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/202305260255713.png)

  <center style="color:#C125C0C0">图3</center>

  可以发现：

  - 矩阵$\mathbf{H}$有8x8个矩阵块

  - 非对角线上的矩阵块如果非0，就在对应的变量之间存在一条边，如上图的红色块。

    所以非对角线的非零块可以理解为：对应的两个变量之间存在约束。

- 更一般的，如果有m个相机位姿，n个路标点，我们可以发现的$\mathbf{H}$矩阵都具有下面这种**稀疏结构**

  ![image-20230526030721552](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/202305260307657.png)

  <center style="color:#C125C0C0">图4</center>

  - 它的形状和图3一致，因为路标点n通常远大于位姿点m而右下角远比左上角大得多。非对角线部分分布着散乱的观测数据。
  - $B$区域的对角块矩阵(位姿块)，每个矩阵都是方阵，维度即相机位姿的参数6x6，矩阵个数是相机位姿的个数。
  - $C$区域的对角块矩阵(路标块)，每个矩阵都是方阵，维度即路标点的参数3x3，矩阵个数是相路标点的个数。

- **Schur消元**可以利用$\mathbf{H}$矩阵的这种稀疏结构加速计算

  在线代中，Schur消元是一种在线性代数中用于简化矩阵的操作。它可以将一个矩阵转化为上三角形式或Schur标准形式，从而简化矩阵的计算和分析。上三角矩阵的特点是所有主对角线以下的元素都为零。

  在SLAM中，Schur消元是指通过对大型稀疏矩阵进行分块和消元操作，从而减小系统的规模，提高求解效率。

  1. 首先$\mathbf{H}$矩阵按照图4划分成这4个区域后，2.2的1式增量方程$H\Delta x=g$也就变成了如下线性方程组：
     $$
     \left [\begin{array}{cccc}
     \mathbf{B} & \mathbf{E}  \\
     \mathbf{E}^T & \mathbf{C}   \\
     \end{array}\right]\left [\begin{array}{cccc}
     \Delta x_c \\
     \Delta x_p
     \end{array}\right]=\left [\begin{array}{cccc}
     \mathbf{v} \\
     \mathbf{w} 
     \end{array}\right]\tag{4}
     $$

  2. 由于B,C区域的矩阵是方阵，而方阵求逆比普通矩阵方便很多，所以我们对4式进行高斯消元，来消去普通矩阵的区域E
     $$
     \left [\begin{array}{cccc}
     \mathbf{I} & -\mathbf{EC}^{-1}  \\
     \mathbf{0} & \mathbf{I}   \\
     \end{array}\right]\left [\begin{array}{cccc}
     \mathbf{B} & \mathbf{E}  \\
     \mathbf{E}^T & \mathbf{C}   \\
     \end{array}\right]\left [\begin{array}{cccc}
     \Delta x_c \\
     \Delta x_p
     \end{array}\right]=\left [\begin{array}{cccc}
     \mathbf{I} & -\mathbf{EC}^{-1}  \\
     \mathbf{0} & \mathbf{I}   \\
     \end{array}\right]\left [\begin{array}{cccc}
     \mathbf{v} \\
     \mathbf{w} 
     \end{array}\right]\tag{5}
     $$
     整理后，得到：
     $$
     \left [\begin{array}{cccc}
     \mathbf{B}-\mathbf{EC}^{-1}\mathbf{E}^T & \mathbf{0}  \\
     \mathbf{E}^T & \mathbf{C}   \\
     \end{array}\right]\left [\begin{array}{cccc}
     \Delta x_c \\
     \Delta x_p
     \end{array}\right]=\left [\begin{array}{cccc}
     \mathbf{v}-\mathbf{E}\mathbf{C}^{-1}\mathbf{w} \\
     \mathbf{w}
     \end{array}\right]\tag{6}
     $$

  3. 得到5式第一部分展开后可以直接求出$\Delta x_c$:
     $$
     \left [\begin{array}{cccc}
     \mathbf{B}-\mathbf{EC}^{-1}\mathbf{E}^T 
     \end{array}\right]\Delta x_c=\mathbf{v}-\mathbf{E}\mathbf{C}^{-1}\mathbf{w}\tag{7}
     $$

  4. 最后将$\Delta x_c$代回5式，即可求出$\Delta x_p$
     $$
     \Delta x_p=\mathbf{C}^{-1}(\mathbf{w}-\mathbf{E}^T)\Delta x_c\tag{8}
     $$

  5. 相比直接求解线性方程组4式，schur消元的优势在于：

     - 在消元过程中C区域的矩阵由于是方阵，他们的逆也更容易求出来

     - 6，7式要求的逆也都是C，所以很容易求解

- 上面的4，5，6，7，8的步骤称之为**边缘化(Marginalization)**或者 **Schur消元**(schur elimination)：

  因为先固定$\Delta x_p$再求$\Delta x_c$的过程相当于在计算$x_p$的边缘分布：$P(x_c,x_p)=P(x_c|x_p)P(x_p)$。
  
  边缘分布指：在多变量的联合概率分布中，得到某变量的独立分布。
  
- 对$\mathbf{H}$矩阵进行Schur消元后，得到$\mathbf{S}$矩阵，它的稀疏状态为图五：

  ![image-20230526034959336](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/202305260349451.png)

  <center style="color:#C125C0C0">图5</center>

  S矩阵和H矩阵一样同样具有物理意义

  - S矩阵的非对角线上的非零矩阵块，表示了该处对应的两个相机变量之间存在共同观测的路标点，反之则表示两相机位姿之间没有共同观测的路标点

    比如图5的$S_{14},S_{24}$不为0，表示相机在位姿$C_1和C_4$，$C_2和C_4$之间有共同观测点

    而$S_{34}=0$，表示位姿$C_3和C_4$之间没有共同的观测点

  - 在ORB-SLAM中的Loacl Mapping环节，做BA时，会特意选择具有共同观测的帧作为关键帧。这时schur消元后的S矩阵式稠密矩阵。

  - 但在其他一些方法如DSO,OKVIS等使用滑动窗口(Sliding Windows)方法，就要每一帧都做BA，这时S矩阵就要求用一些特殊手段保证稀疏性了。

### 2.4 鲁棒核函数

- 如2.1的2式所示，前面的BA问题将最小化误差项的二范数平方和作为目标函数。这种做法有严重的缺陷：

  由于误匹配等原因，一些异常值被加入到了图的边里。但优化算法无法分辨这些异常值，只会当作是常规的误差来处理。而又由于异常值的误差很大，所以二范数增大的也很快，这就导致优化算法会专注于调整一个异常值，而忽略了其他正常边的影响。

  为了消除这种影响，就需要核函数

- **鲁棒核函数**(Robust Kernel)保证了每条边的误差不会大的没边而掩盖其他的边，使得整个优化结果更稳健。

  具体做法是：将原本误差的二范数度量替换成一个增长没有那么快的函数，同时保证自己的光滑性质（否则无法求导）

  鲁棒和函数有很多种，比如Huber核，Cauchy核，Tukey核等等。

  其中Huber核最常用：
  $$
  H(e)=\begin{cases}
  \frac{1}{2}e^2  \qquad \qquad \qquad 当|e|<=\delta\\
  \delta(|e|-\frac{1}{2}\delta) \qquad \quad 其他
  \end{cases} \tag{1}
  $$

  - 当误差e大于阈值$\delta$后，函数增长由二次形式变成了一次形式，想打渔限制了梯度的最大值



## 3. 数据集：BAL

使用[problem-16-22106-pre.txt](https://github.com/gaoxiang12/slambook2/blob/master/ch9/problem-16-22106-pre.txt)文件作为例子，该文件以行的形式存储BA问题的信息，详细格式说明见[BAL官网](https://grail.cs.washington.edu/projects/bal/)

- BAL的相机内参模型由焦距$f$和畸变参数$k_1,k_2$给出。$f$ 即$f_x$,$f_y$，视为一个值。
- BAL数据在投影时假设投影平面在相机光心之后，所以需要在投影之后乘以-1

下面使用[common.h](https://github.com/gaoxiang12/slambook2/blob/master/ch9/common.h)中定义的BALProblem类来读取BAL数据集的数据

## 4. 实践: Ceres求解BA

### 4.1 代码

#### 4.1.1 bundle_adjustment_ceres.cpp

```c++
#include <iostream>
#include <ceres/ceres.h>
#include "common.h"	// 定义了BALProblem类，用于读取BAL数据集的数据，代码见3中的超链接
#include "SnavelyReprojectionError.h" // 定义了投影误差模型，代码见4.1.2

using namespace std;

void SolveBA(BALProblem &bal_problem);

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "usage: bundle_adjustment_ceres bal_data.txt" << endl;
        return 1;
    }

    BALProblem bal_problem(argv[1]);			// 读入数据
    bal_problem.Normalize();					// 归一化数据
    bal_problem.Perturb(0.1, 0.5, 0.5);			// perturb函数加入噪声
    bal_problem.WriteToPLYFile("initial.ply");	// 将优化前的数据（相机和3d点） 保存在initial.ply文件中
    SolveBA(bal_problem);						// 求解最小二乘问题
    bal_problem.WriteToPLYFile("final.ply");	// 将优化后的数据（相机和3d点） 保存在final.ply文件中

    return 0;
}

void SolveBA(BALProblem &bal_problem) {
    const int point_block_size = bal_problem.point_block_size();
    const int camera_block_size = bal_problem.camera_block_size();
    //注意这里获得待优化系数首地址的时候要用mutable_points()和mutable_cameras()
    // 因为这两个函数指向的地址的内容是允许改变的（优化系数肯定要变的啦）
    double *points = bal_problem.mutable_points();		//获得待优化系数3d点 points指向3d点的首地址
    double *cameras = bal_problem.mutable_cameras();	//获得待优化系数相机 cameras指向相机的首地址

    // Observations is 2 * num_observations long array observations
    // [u_1, u_2, ... u_n], where each u_i is two dimensional, the x
    // and y position of the observation.
    const double *observations = bal_problem.observations();	//获得观测数据 observations指向观测数据的首地址
    
    ceres::Problem problem; // ceres的使用：构建最小二乘问题

    for (int i = 0; i < bal_problem.num_observations(); ++i) {
        ceres::CostFunction *cost_function;

        // Each Residual block takes a point and a camera as input
        // and outputs a 2 dimensional Residual
        // 将重投影误差作为代价函数
        cost_function = SnavelyReprojectionError::Create(observations[2 * i + 0], observations[2 * i + 1]);

        // If enabled use Huber's loss function.
        ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);	//第九章2.4提到的huber核函数

        // Each observation corresponds to a pair of a camera and a point which are identified by camera_index()[i] and point_index()[i] respectively.
        //bal_Problem.point_index()这返回的是一个地址指向索引号的首地址
        double *camera = cameras + camera_block_size * bal_problem.camera_index()[i];	// 待优化的相机位姿
        double *point = points + point_block_size * bal_problem.point_index()[i];		// 待优化的3D点坐标
		
        // 完成最小二乘问题的构建
        problem.AddResidualBlock(cost_function, loss_function, camera, point);
    }

    // show some information here ...
    std::cout << "bal problem file loaded..." << std::endl;
    std::cout << "bal problem have " << bal_problem.num_cameras() << " cameras and "
              << bal_problem.num_points() << " points. " << std::endl;
    std::cout << "Forming " << bal_problem.num_observations() << " observations. " << std::endl;

    std::cout << "Solving ceres BA ... " << endl;
    ceres::Solver::Options options;	//配置ceres求解器，通过改options有很多配置可以调整
    options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;//配置第九章2.3提到的schur消元
    options.minimizer_progress_to_stdout = true;//配置，是否输出
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
}
```

#### 4.1.2 SnavelyReprojectionError.h

重投影误差，理论见第七讲6.3

畸变公式，见第五讲1.2

```cpp
#ifndef SnavelyReprojection_H
#define SnavelyReprojection_H

#include <iostream>
#include "ceres/ceres.h"
#include "rotation.h"

class SnavelyReprojectionError {
public:
    //构造函数后不需要加分号
    SnavelyReprojectionError(double observation_x, double observation_y) : observed_x(observation_x),
                                                                           observed_y(observation_y) {}
	// operator 是C++中用于运算符重载的关键字,这里给了括号一个运算规则
    template<typename T>
    bool operator()(const T *const camera,
                    const T *const point,
                    T *residuals) const {
        // camera[0,1,2] are the angle-axis rotation
        T predictions[2];	//创建一个包含2个T类型量的数组
        CamProjectionWithDistortion(camera, point, predictions);	//实际的计算规则定义在在CamProjectionWithDistortion函数
        residuals[0] = predictions[0] - T(observed_x);
        residuals[1] = predictions[1] - T(observed_y);

        return true;
    }

    // camera : 9 dims array
    // [0-2] : angle-axis rotation
    // [3-5] : translateion
    // [6-8] : camera parameter, [6] focal length, [7-8] second and forth order radial distortion
    // point : 3D location.
    // predictions : 2D predictions with center of the image plane.
    template<typename T>
    static inline bool CamProjectionWithDistortion(const T *camera, const T *point, T *predictions) {
        // Rodrigues' formula
        T p[3];
        AngleAxisRotatePoint(camera, point, p);
        // camera[3,4,5] are the translation
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // Compute the center fo distortion
        // 1. 归一化点的坐标
        T xp = -p[0] / p[2];
        T yp = -p[1] / p[2];

        // Apply second and fourth order radial distortion畸变
        const T &l1 = camera[7]; // 二阶径向畸变（second order radial distortion）是指通过二次多项式模型来描述径向畸变的效应。
        const T &l2 = camera[8]; // 四阶径向畸变（fourth order radial distortion）是指使用四次多项式模型来描述径向畸变的效应

        T r2 = xp * xp + yp * yp;
        T distortion = T(1.0) + r2 * (l1 + l2 * r2);

        const T &focal = camera[6];	//焦距
        // 2. 计算得到发生畸变后的坐标
        predictions[0] = focal * distortion * xp;
        predictions[1] = focal * distortion * yp;

        return true;
    }
	
    /*
      优化的梯度计算ceres提供3种选择
      1. 自动求导:AuoDiff
      2. 数值求导:NumericDiff
      3. 我们自己推导解析的倒数形式
      这里使用第一种：AutoDiffCostFunction。
      
      注意ceres中必须用double数组形式存储优化变量
      
      create静态函数作为外部调用的接口，直接返回一个可自动求导的ceres代价函数，来求解重投影误差的最小二乘问题最小化。
    */
    static ceres::CostFunction *Create(const double observed_x, const double observed_y) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
            new SnavelyReprojectionError(observed_x, observed_y)));
    }

private:
    double observed_x;
    double observed_y;
};

#endif // SnavelyReprojection.h


```

### 4.2 编译

```cmake
cmake_minimum_required(VERSION 2.8)

project(bundle_adjustment)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-O3 -std=c++11")
set(CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

Find_Package(Ceres REQUIRED)

include_directories("/usr/include/eigen3")
include_directories(${PROJECT_SOURCE_DIR})

add_library(bal_common common.cpp)
add_executable(bundle_adjustment_ceres bundle_adjustment_ceres.cpp)

target_link_libraries(bundle_adjustment_ceres ${CERES_LIBRARIES} bal_common fmt)
```



## 5. 实践: g2o求解BA

g2o使用图模型来描述问题，

- 节点：相机和路标
- 边：他们之间的观测

### 5.1 代码

```c++
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel_impl.h>
#include <iostream>

#include "common.h"
#include "sophus/se3.hpp"

using namespace Sophus;
using namespace Eigen;
using namespace std;

/// 姿态和内参的结构,定义一个结构体表示相机 相机9维
// [0-2] : angle-axis rotation
// [3-5] : translateion
// [6-8] : camera parameter, [6] focal length, [7-8] second and forth order radial distortion
// point : 3D location.
// predictions : 2D predictions with center of the image plane.
struct PoseAndIntrinsics {
    PoseAndIntrinsics() {}
	
    /// set from given data address
    //显示构造进行赋值，把数据集的内容赋值过去
    explicit PoseAndIntrinsics(double *data_addr) {
        rotation = SO3d::exp(Vector3d(data_addr[0], data_addr[1], data_addr[2]));	// 旋转
        translation = Vector3d(data_addr[3], data_addr[4], data_addr[5]);	// 位移
        focal = data_addr[6]; // 焦距
        k1 = data_addr[7];	// 二阶径向畸变（second order radial distortion）是指通过二次多项式模型来描述径向畸变的效应。
        k2 = data_addr[8];	// 四阶径向畸变（fourth order radial distortion）是指使用四次多项式模型来描述径向畸变的效应
    }

    /// 将估计值放入内存，即将优化的系数放进内存
    void set_to(double *data_addr) {
        auto r = rotation.log();
        for (int i = 0; i < 3; ++i) data_addr[i] = r[i];
        for (int i = 0; i < 3; ++i) data_addr[i + 3] = translation[i];
        data_addr[6] = focal;
        data_addr[7] = k1;
        data_addr[8] = k2;
    }

    SO3d rotation;	// 李群，旋转
    Vector3d translation = Vector3d::Zero(); // 平移
    double focal = 0;	//焦距
    double k1 = 0, k2 = 0;	//畸变系数
};

/// 位姿加相机内参的顶点，9维，前三维为so3，接下去为t, f, k1, k2
//定义两个顶点 一个是 相机（PoseAndIntrinsics） 一个是路标点（3d点）
//顶点 ：相机（PoseAndIntrinsics）
class VertexPoseAndIntrinsics : public g2o::BaseVertex<9, PoseAndIntrinsics> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPoseAndIntrinsics() {}
    // override关键字表示对基类函数的覆盖
    //重置
    virtual void setToOriginImpl() override {
        _estimate = PoseAndIntrinsics();	//给待优化系数赋上初始值
    }
    //更新
    virtual void oplusImpl(const double *update) override {
        _estimate.rotation = SO3d::exp(Vector3d(update[0], update[1], update[2])) * _estimate.rotation;
        _estimate.translation += Vector3d(update[3], update[4], update[5]);
        _estimate.focal += update[6];
        _estimate.k1 += update[7];
        _estimate.k2 += update[8];
    }

    /// 根据估计值投影一个点
    Vector2d project(const Vector3d &point) {
        Vector3d pc = _estimate.rotation * point + _estimate.translation;
        pc = -pc / pc[2];				//把相机坐标归一化
        double r2 = pc.squaredNorm();	//相机坐标归一化后的模的平方
        double distortion = 1.0 + r2 * (_estimate.k1 + _estimate.k2 * r2);
        return Vector2d(_estimate.focal * distortion * pc[0],
                        _estimate.focal * distortion * pc[1]);
    }
    //存盘和读盘 ： 留空
    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}
};

//顶点 ：路标点（3d点）
class VertexPoint : public g2o::BaseVertex<3, Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPoint() {}
	//重置
    virtual void setToOriginImpl() override {
        _estimate = Vector3d(0, 0, 0);	//待优化系数的初始化
    }
	//更新
    virtual void oplusImpl(const double *update) override {
        _estimate += Vector3d(update[0], update[1], update[2]);
    }
	//存盘和读盘 ： 留空
    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}
};
//定义边 这里面边的定义比前几章的要简单很多
class EdgeProjection :
    public g2o::BaseBinaryEdge<2, Vector2d, VertexPoseAndIntrinsics, VertexPoint> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	//计算残差
    virtual void computeError() override {
        auto v0 = (VertexPoseAndIntrinsics *) _vertices[0];
        auto v1 = (VertexPoint *) _vertices[1];
        auto proj = v0->project(v1->estimate());
        _error = proj - _measurement;
    }

    // use numeric derivatives
    //存盘和读盘 ： 留空
    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}

};

void SolveBA(BALProblem &bal_problem);

int main(int argc, char **argv) {

    if (argc != 2) {
        cout << "usage: bundle_adjustment_g2o bal_data.txt" << endl;
        return 1;
    }

    BALProblem bal_problem(argv[1]);			// 传入数据
    bal_problem.Normalize();					// 归一化
    bal_problem.Perturb(0.1, 0.5, 0.5);			// 给数据加上噪声（相机旋转、相机平移、路标点）
    bal_problem.WriteToPLYFile("initial.ply");  // 优化前的数据写入initial
    SolveBA(bal_problem);						// 求解BA
    bal_problem.WriteToPLYFile("final.ply");    // 优化后的数据写入final

    return 0;
}

void SolveBA(BALProblem &bal_problem) {
    // 获得 相机和点的维度
    const int point_block_size = bal_problem.point_block_size();
    const int camera_block_size = bal_problem.camera_block_size();
    //获得相机和点各自参数的首地址
    double *points = bal_problem.mutable_points();
    double *cameras = bal_problem.mutable_cameras();

    // pose dimension 9, landmark is 3
    //构建图优化
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<9, 3>> BlockSolverType;	//两个顶点的维度
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
    // use LM
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);	//设置求解器
    optimizer.setVerbose(true);		//打开调试输出

    /// build g2o problem
    //获得观测值的首地址
    const double *observations = bal_problem.observations();
    // vertex
    //加入顶点
    //因为顶点有很多个，所以需要容器
    //容器 vertex_pose_intrinsics 和 vertex_points存放两顶点的地址
    vector<VertexPoseAndIntrinsics *> vertex_pose_intrinsics;
    vector<VertexPoint *> vertex_points;
    for (int i = 0; i < bal_problem.num_cameras(); ++i) {
        VertexPoseAndIntrinsics *v = new VertexPoseAndIntrinsics();
        double *camera = cameras + camera_block_size * i;	//获得每个相机的首地址
        v->setId(i); //设置编号
        v->setEstimate(PoseAndIntrinsics(camera)); //传入待优化的系数 此处为相机
        optimizer.addVertex(v); //加入顶点
        vertex_pose_intrinsics.push_back(v);
    }
    for (int i = 0; i < bal_problem.num_points(); ++i) {
        VertexPoint *v = new VertexPoint();	//获得每个路标点的首地址
        double *point = points + point_block_size * i;
        v->setId(i + bal_problem.num_cameras());
        v->setEstimate(Vector3d(point[0], point[1], point[2]));	//传入待优化的系数 此处为路标点
        // g2o在BA中需要手动设置待Marg的顶点
        v->setMarginalized(true);	//设置边缘化
        optimizer.addVertex(v);		//加入顶点
        vertex_points.push_back(v); //将顶点一个一个放回到容器里面
    }

    // edge
    for (int i = 0; i < bal_problem.num_observations(); ++i) {
        EdgeProjection *edge = new EdgeProjection;
        edge->setVertex(0, vertex_pose_intrinsics[bal_problem.camera_index()[i]]); //加入顶点
        edge->setVertex(1, vertex_points[bal_problem.point_index()[i]]); //加入顶点
        edge->setMeasurement(Vector2d(observations[2 * i + 0], observations[2 * i + 1])); //设置观测数据
        edge->setInformation(Matrix2d::Identity()); //设置信息矩阵
        edge->setRobustKernel(new g2o::RobustKernelHuber()); //设置核函数
        optimizer.addEdge(edge); //加入边
    }

    optimizer.initializeOptimization();
    optimizer.optimize(40);

    // set to bal problem
    //优化后在存到内从中去
    for (int i = 0; i < bal_problem.num_cameras(); ++i) {
        double *camera = cameras + camera_block_size * i;
        auto vertex = vertex_pose_intrinsics[i];	//把优化后的顶点地址给vertex
        auto estimate = vertex->estimate();			//这样estimate就指向了优化后的相机结构体 此时estimate本质上指向了 相机结构体
        estimate.set_to(camera);					//这样camera就指向了优化后的相机（利用了相机结构体的set_to()函数）
    }
    for (int i = 0; i < bal_problem.num_points(); ++i) {
        double *point = points + point_block_size * i;
        auto vertex = vertex_points[i];
        for (int k = 0; k < 3; ++k) point[k] = vertex->estimate()[k];
    }
    //经过上面的两个循环后，原来待优化的系数就被优化完毕了，并且优化后的系数 还是存放在 bal_problem.mutable_cameras()和 bal_problem.mutable_points() 所对应的地址中

}

```

### 5.2 编译

```cmake
cmake_minimum_required(VERSION 2.8)

project(bundle_adjustment)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-O3 -std=c++11")
set(CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4")

# g2o提供的一系列找g2o库的cmake文件https://github.com/RainerKuemmerle/g2o/tree/master/cmake_modules
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake) 
list(APPEND CMAKE_MODULE_PATH /home/yang/3rdLibrary/g2o-20230223_git/cmake_modules)
Find_Package(G2O REQUIRED)
Find_Package(Eigen3 REQUIRED)
Find_Package(Sophus REQUIRED)
Find_Package(CSparse REQUIRED)
find_package(FMT REQUIRED)

# 用到了g2o什么模块，这里就要添加相应的模块，比如这里用到的g2o_solver_csparse 
SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse g2o_solver_csparse )
include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

add_library(bal_common common.cpp)
add_executable(bundle_adjustment_g2o bundle_adjustment_g2o.cpp)

target_link_libraries(bundle_adjustment_g2o bal_common fmt ${G2O_LIBS})
```

# 十、后端优化：位姿图

BA能精确的优化每个相机位姿和特征点位置，但在大场景中，大量特征点的存在会严重降低计算效率。所以需要用到一种简化的BA：位姿图

## 1. 滑动窗口滤波和优化

### 1.1 实际环境下的BA结构

带有相机位姿T和空间点P的图优化称为BA。SLAM因为要求实时性所以不能像SFM那样每时每刻计算整个BA，而是要限制BA的规模，比如：BA规模不超过1万个路标点，迭代不超过20次，用时不超过0.5秒，等等。

控制规模的方法有：

- 抽取关键帧

  从连续的视频中抽取一部分作为关键帧，然后仅仅构造关键帧与路标点之间的BA，其余的非关键帧只用于定位建图没有贡献。

  但随着时间的推移，关键帧会越来越多，计算效率还是会下降。

- 滑动窗口法：
  - 可以仅保留当前时刻最近的N个关键帧，去掉时间上更早的关键帧。于是BA被固定在一个时间窗口内，离开这个窗口就会被丢弃。
  - 也可以仅优化和当前帧有20个以上共视路标的关键帧，其余部分固定不变。
    - 共视图：与现在的相机存在共同观测的关键帧构成的图

![img](https://cdn.jsdelivr.net/gh/Fernweh-yang/ImageHosting@main/img/v2-40e006c26536c7f6c0eb66f7c8cea703_720w.webp)

### 1.2 滑动窗口法

- 考虑一个窗口，窗口内有N个关键帧，位姿表达式为：$x_1,\cdots,x_N$

  我们想要指导这几个关键帧的位置在哪里，以及他们的不确定度如何，这也就对应了他们在高斯分布假设下的均值协方差:

  位姿可求得为：
  $$
  [x_1,\cdots,x_N]^T \sim N([\mu_1,\cdots,\mu_N]^T,\Sigma) \tag{1}
  $$

  - $\mu_k:$为第k个关键帧的位姿均值

    即BA迭代后的结果

  - $\Sigma：$为所有关键帧的协方差矩阵

    对整个BA的H矩阵进行边缘化之后的结果，即第九章2.3中schur消元后得到的S矩阵

- 与传统BA不同的是，滑动窗口法在窗口中新增或删除一个关键帧时，有不同的实现细节：

  - **新增一个关键帧和路标点**

    - 在上个时刻，滑动窗口已经建立了N个关键帧，他们服从1式所示的高斯分布
    - 此时，新增关键帧$x_{N+1}$，只需按照正常的BA流程处理。对所有点进行边缘化时，就可以得到N+1个关键帧的高斯分布。

  - **删除一个旧的关键帧**

    由于关键帧不是独立的，它和其他帧观测到同样的路标，所以将某一关键帧(如下图的$x_1$)边缘化后，会导致整个问题不再稀疏。

    ![image-20230611074611161](https://cdn.jsdelivr.net/gh/Fernweh-yang/ImageHosting@main/img/image-20230611074611161.png)

    - 如上图所示，如果只边缘化$x_1$，Schur消元过程相当于通过矩阵行和列的操作消去非对角线处几个非零矩阵块。显然这会导致右下角的路标点矩阵块不再是非对称矩阵。这个过程也被称为边缘化中的**填入**(Fill-in)

      对比2.3的图4那我们边缘化路标点(块E)，Fill-in会出现在位姿块(块B)中。BA不要求位姿块为对角块，所以稀疏BA仍可求解。

      但这里边缘化关键帧(块B)时，路标块(块C)将就不再是对角块结构了，这就导致BA不能再按照稀疏方式迭代求解。

    - 为解决上面这个问题：

      我们只需要在边缘化关键帧时，同时边缘化它观测到的路标点，这样路标点的信息就会转换成剩下那些关键帧之间的共视信息。
    
    - 当某个关键帧被边缘化时，会得到一些先验信息(基于以往经验、观察或已知事实形成的预先知识)
    
      - 这个关键帧观测到的路标点得到的先验信息：**这些路标应该在哪里**
      - 如果继续边缘化这些路标点，观测者得到：**观测他们的关键帧应该在哪里**
    
      这是因为边缘化在概率上的意义就是指条件概率，所以边缘化某个关键帧，就是”保持这个关键帧当前的估计值，求其他状态变量以这个关键帧为条件的条件概率“。
    
      窗口中状态变量的描述方式也从联合分布变成了条件概率分布：
      $$
      p(x_1,\cdots,x_4,y_1,\cdots,y_6)=p(x_2,\cdots,x_4,y_1,\cdots,y_6|x_1)\underbrace{p(x_1)}_{边缘化时舍去的部分}\tag{2}
      $$
      

## 2. 位姿图

### 2.1 位姿图的意义

- 思路：

  经过若干次观测，特征点的变化很小  --->  再对收敛的特征点进行优化费力不讨好 ---> 不如优化几次之后把特征点固定住，将其看作位姿估计的约束 ---> 

  进一步在BA时，只保留关键帧的轨迹，舍去大量特征点优化的计算，从而构建所谓的位姿图(pose graph)

   ![image-20230611100137088](https://cdn.jsdelivr.net/gh/Fernweh-yang/ImageHosting@main/img/image-20230611100137088.png)

  位姿节点之间的边，可以由两个关键帧之间通过特征匹配之后得到的运动估计来给定初始值。

- 为了SLAM的实时性，在大范围场景和时间运动中，我们必须采取措施如：

  - 位姿图：舍弃对路标点的优化，只保留Pose之间的边
  - 滑动窗口：丢弃一些历史数据

### 2.2 位姿图的优化

- 由上可知位姿图优化中，节点和边分别对应为：

  - **节点**：相机位姿，以$\mathbf{T}_1,\cdots,\mathbf{T}_n$表示。

    三角形的位姿，是通过和世界坐标系的比较而得到的，其实世界坐标系在大部分情况下也就是一开机的时候的相机坐标系，把第一帧检测到的特征点的相机坐标，当成是世界坐标，以此为参照，逐步直接递推下去。

  - **边**：两个位姿节点之间相对运动的估计。

    根据中途的两张图，单独拿它俩出来估计一个相对的位姿变换。这个估计可以来自特征点法、直接法或者来自GPS/IMU积分

- 首先在视觉里程计(前端)中，我们首先得到世界坐标系下的点，然后根据图像坐标系下的点，估计出世界坐标系与图像坐标系下的变换关系作为位姿，（这里的世界坐标系下的坐标实际是根据第一帧一路递推下来的，可能错误会一路累积）。由此算出来一系列位姿$\mathbf{T}_1,\cdots,\mathbf{T}_n$。

  某两个位姿，如$\mathbf{T}_1,\mathbf{T}_2$之间的变换(也就是边)**并不是直接根据T1的逆乘以T2这种数学方式算出来的**（不然还优化什么？肯定相等啊）

  而是通过$\mathbf{T}_1,\mathbf{T}_2$所在的图像进行匹配，单独算一个2D-2D之间的位姿变换或光流法得到。这样根据图像算出的维兹变换肯定和$\mathbf{T}_1,\mathbf{T}_2$用数学计算得到的位姿变换是有差异的。

- 现在考虑一个$\mathbf{T}_i$和$\mathbf{T}_j$之间的一个运动，也就是图中的边: $\Delta\mathbf{T}_{ij}$

  这个运动的数学描述可为：
  $$
  \Delta\xi_{ij}=\xi_i^{-1}\circ\xi_j=ln(\mathbf{T}_i^{-1}\mathbf{T}_j)^{\vee} \tag{1}
  $$

  - 其中李群部分可写成：
    $$
    \Delta\mathbf{T}_{ij}=\mathbf{T}_{ij}=\mathbf{T}_i^{-1}\mathbf{T}_j \tag{2}
    $$
    
    - 物理意义：先对某坐标系下的点$P$执行$T_j$变换得到$P'$，再执行$T_i^{-1}$**逆向转回**到原始状态得到$P''$
      - 如果$T_i=T_j$，那么$P=P''$，也就是会回到最初的坐标
      - 如果$T_i\neq T_j$，那么$P$和$P''$之间的差异，也就可以看作两个变换矩阵$T$之间的差异。

- 正如上面说的，现实中等式2不会完全相等，为此设立最小二乘误差，首先构建误差：

  将2式左侧移到右侧，那么左边就是1，求对数是0，通过这种方式来构造一个误差，使它为0：
  $$
  \begin{align}
  \mathbf{e}_{ij}&=ln(\mathbf{T}_{ij}^{-1}\mathbf{T}_{i}^{-1}\mathbf{T}_{j})^{\vee} \\
  &= ln\Big(exp\big((-\xi_{ij})^{\wedge}\big)exp\big((-\xi_{i})^{\wedge}\big)exp(\xi_{j}^{\wedge})\Big)
  \end{align} \tag{3}
  $$

- 优化的变量是$\mathbf{T}_i,\mathbf{T}_j$，要对他们求偏导就要用到第四章3.3的扰动模型。

  现在对3式，增加2个扰动$\delta \xi_i$和$\delta \xi_j$:
  $$
  \hat{\mathbf{e}}_{ij}=ln\Big(\mathbf{T}_{ij}^{-1}\mathbf{T}_{i}^{-1}exp\big((-\delta\xi_{i})^{\wedge}\big)exp(\delta\xi_{j}^{\wedge})\mathbf{T}_j\Big)^{\vee}\tag{4}
  $$

  - 其中$\mathbf{T}_i$在误差3式中是逆，所以它的扰动在右边
  - SLAM中一个量带帽子的符号通常表示该量是一个估计值或预测值，而不是准确的真实值。

- 4式中两个扰动被夹在中间，为了利用第四章3的BCH近似，我们希望把扰动移到式子左边或右边。

  利用第四章11中的2式：$\mathbf{T}exp(\xi^{\wedge})\mathbf{T}^{-1}=exp((Ad(\mathbf{T})\xi)^{\wedge})$引入伴随项，稍微修改$\mathbf{T}$和他的逆就可得到：
  $$
  exp(\xi^{\wedge})\mathbf{T}=\mathbf{T}exp\Big(\big(Ad(\mathbf{T}^{-1})\xi\big)^{\wedge} \Big) \tag{5}
  $$

- 利用5式，来将4式中的$\mathbf{T}_j$移到左边去：
  $$
  \begin{align}
  \hat{\mathbf{e}}_{ij}&=ln\Big(\mathbf{T}_{ij}^{-1}\mathbf{T}_{i}^{-1}exp\big((-\delta\xi_{i})^{\wedge}\big)exp(\delta\xi_{j}^{\wedge})\mathbf{T}_j\Big)^{\vee} \tag{6.1}\\
  &=ln\Big(\mathbf{T}_{ij}^{-1}\mathbf{T}_{i}^{-1}\mathbf{T}_jexp\big((-Ad(\mathbf{T}_j^{-1})\delta\xi_{i})^{\wedge}\big)exp\big((Ad(\mathbf{T}_j^{-1})\delta\xi_{j})^{\wedge}\big)\Big)^{\vee}\tag{6.2}\\
  &\approx ln\Big(\mathbf{T}_{ij}^{-1}\mathbf{T}_{i}^{-1}\mathbf{T}_j[\mathbf{I}-(Ad(\mathbf{T}_j^{-1})\delta\xi_{i})^{\wedge}+(Ad(\mathbf{T}_j^{-1})\delta\xi_{j})^{\wedge}]\Big)^{\vee} \tag{6.3}\\
  &\approx \mathbf{e}_{ij}+\frac{\partial\mathbf{e}_{ij}}{\partial\delta\xi_i}\delta\xi_i+\frac{\partial\mathbf{e}_{ij}}{\partial\delta\xi_j}\delta\xi_j \tag{6.4}
  \end{align}
  $$

  - 6.2->6.3: 把两个exp合并起来，然后泰勒展开，保留一阶项

  - 6.3->6.4: 乘开再用第四章的BCH近似

    推导过程见2.3

- 根据第四章3.4的李代数求导公式，6.4式中的微分项可得：

  对于$\mathbf{T}_i$:
  $$
  \frac{\partial\mathbf{e}_{ij}}{\partial\delta\xi_i}=-\mathcal{J}_r^{-1}(\mathbf{e}_{ij})Ad(\mathbf{T}_j^{-1})\tag{7.1}
  $$
  对于$\mathbf{T}_j$:
  $$
  \frac{\partial\mathbf{e}_{ij}}{\partial\delta\xi_j}=\mathcal{J}_r^{-1}(\mathbf{e}_{ij})Ad(\mathbf{T}_j^{-1})\tag{7.2}
  $$
  但由于雅可比矩阵$\mathcal{J}_r$过于复杂，所以通常取他们的近似。

  如果误差接近0，就可以将他们近似为$\mathbf{I}$或者：
  $$
  \mathcal{J}_r^{-1}(\mathbf{e}_{ij})\approx\mathbf{I}+\frac{1}{2} 
   
  \left [\begin{array}{cccc}
  \phi_e^{\wedge} & \rho_e^{\wedge}  \\
  0 & \phi_e^{\wedge} \\
  
  \end{array}\right]
  \tag{7.3}
  $$

- 得到6.4的误差向后，就和普通的图优化一样了，本质上是一个最小二乘问题

  记$\epsilon$为所有边的集合，那么总体的目标函数为：
  $$
  min\frac{1}{2}\sum_{i,j\in\epsilon}e_{ij}^T\Sigma_{ij}^{-1}e_{ij} \tag{8}
  $$
  和前面一样可以用第六章提到高斯牛顿法或列文博格-马夸尔特方法来求解这个问题。

  代码实现上可以使用Ceres或g2o。

### 2.3 6.4式的推导

![](https://cdn.jsdelivr.net/gh/Fernweh-yang/ImageHosting@main/img/20200317120045411.jpg)

## 3. 实践：位姿图的优化

### 3.1 g2o原声位姿图

位姿图xx.g2o可以由g2o自带的create spehere程序仿真生成。他的轨迹是一个球，由多层组成，每层都是一个圆形，共包含2500个位姿节点，可以看成是一个转圈上升的过程。在层上的t-1到t时刻节点之间的边是里程计边，层与层之间的边是里程计边。

可以用`g2o_viewer xx.g2o`命令来打开这个文件：

![image-20230614010234236](https://cdn.jsdelivr.net/gh/Fernweh-yang/ImageHosting@main/img/image-20230614010234236.png)

上面这个位姿图是增加了噪声的，可以点击optimize来优化它，就可以得到正确的轨迹（一个球）了。

这个文件由顶点和边两部分组成：

```
VERTEX_SE3:QUAT 0 -0.125664 -1.53894e-17 99.9999 0.706662 4.32706e-17 0.707551 -4.3325e-17 
...
EDGE_SE3:QUAT 590 640 -0.177202 0.0164101 -6.27791 0.00404296 0.0255702 0.00743393 0.999637 10000 0 0 0 0 0 10000 0 0 0 0 10000 0 0 0 40000 0 0 40000 0 40000 
```

- 顶点：类型为VERTEX_SE3，并用平移向量(前3个)+四元数(后4个) 来表示位姿
- 边：类型为EDGE_SE3，由2个顶点的值(上面那7个值) + 信息矩阵（即稀疏矩阵）的右上角组成

### 3.2 优化g2o默认的顶点和边

#### 代码

```c++
#include <iostream>
#include <fstream>
#include <string>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

using namespace std;

/************************************************
 * 本程序演示如何用g2o solver进行位姿图优化
 * sphere.g2o是人工生成的一个Pose graph，我们来优化它。
 * 尽管可以直接通过load函数读取整个图，但我们还是自己来实现读取代码，以期获得更深刻的理解
 * 这里使用g2o/types/slam3d/中的SE3表示位姿，它实质上是四元数而非李代数.
 * **********************************************/

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "Usage: pose_graph_g2o_SE3 sphere.g2o" << endl;
        return 1;
    }
     // 打开文件流
    ifstream fin(argv[1]);
    if (!fin) {
        cout << "file " << argv[1] << " does not exist." << endl;
        return 1;
    }

    // 选择 6x6 的块求解器
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    // 创建稀疏优化器，采用LM算法，线性求解器
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);   // 设置求解器
    optimizer.setVerbose(true);       // 打开调试输出

    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的数量
    // 读取sphere.g2o文件内容
    while (!fin.eof()) {
        string name;
        fin >> name;    // 文件流读取第一个字符串
        // 判断该字符串是顶点还是边
        if (name == "VERTEX_SE3:QUAT") {
            // SE3 顶点
            g2o::VertexSE3 *v = new g2o::VertexSE3();
            int index = 0;
            fin >> index;       // 读取该行的第二个字符，作为顶点的序号
            v->setId(index);    // 设置位姿顶点的在图优化器中的索引号
            v->read(fin);       // 继续从fin文件流中读取剩余字符串，tx,ty,tz,qx,qy,qz,qw
            optimizer.addVertex(v); // 往优化器中添加位姿顶点
            vertexCnt++;        // 统计位姿顶点数目
            // 固定第一个位姿顶点，不参与优化
            if (index == 0)
                v->setFixed(true);
        } else if (name == "EDGE_SE3:QUAT") {
            // SE3-SE3 边
            g2o::EdgeSE3 *e = new g2o::EdgeSE3();
            int idx1, idx2;     // 关联的两个顶点
            fin >> idx1 >> idx2;    // 连续读取两个字符串，作为边关联的两个位姿顶点
            e->setId(edgeCnt++);    // 设置边在图优化器中的索引号
            e->setVertex(0, optimizer.vertices()[idx1]);    // 设置边连接的第一个位姿顶点的索引号
            e->setVertex(1, optimizer.vertices()[idx2]);    // 设置边连接的第二个位姿顶点的索引号
            e->read(fin);                                   // 继续从fin文件流中读取剩余字符串，tx,ty,tz,qx,qy,qz,qw以及信息矩阵对角矩阵的右上角
            optimizer.addEdge(e);   // 往优化器中添加误差边
        }
        if (!fin.good()) break;
    }

    cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;

    cout << "optimizing ..." << endl;
    optimizer.initializeOptimization(); // 优化器初始化
    optimizer.optimize(30);             // LM迭代优化30次

    cout << "saving optimization results ..." << endl;
    optimizer.save("result.g2o");       // 优化器结果保存成.g2o文件

    return 0;
}
```

#### 编译

```cmake
cmake_minimum_required(VERSION 2.8)
project(pose_graph)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -O2")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)	#FindG2O.cmake

# g2o 
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

add_executable(pose_graph_g2o_SE3 pose_graph_g2o_SE3.cpp)
target_link_libraries(pose_graph_g2o_SE3
        g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES} fmt
        )

```

### 3.3 优化sophus定义的顶点和边

#### 代码

```c++
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <sophus/se3.hpp>

using namespace std;
using namespace Eigen;
using Sophus::SE3d;
using Sophus::SO3d;

/************************************************
 * 本程序演示如何用g2o solver进行位姿图优化
 * sphere.g2o是人工生成的一个Pose graph，我们来优化它。
 * 尽管可以直接通过load函数读取整个图，但我们还是自己来实现读取代码，以期获得更深刻的理解
 * 本节使用李代数表达位姿图，节点和边的方式为自定义
  
 * 用g2o的话 需要定义顶点和边
 * 位姿图优化就是只优化位姿 不优化路标点
 * 顶点应该相机的位姿
 * 边是相邻两个位姿的变换
 * error误差是观测的相邻相机的位姿变换的逆 
 * 待优化的相邻相机的位姿变换
 * 我们希望这个误差接近I矩阵 给误差取ln后 误差接近 0 
 * 该程序用李代数描述误差
 
 * 这里把J矩阵的计算放在JRInv(const SE3d & e)函数里
 * 这里的J矩阵还不是雅克比矩阵 具体雅克比见书上公式 p272页 公式10.9 10.10
 * 李代数应该是向量形式 
 * 李代数的hat 也就是李代数向量变为反对称矩阵
 * **********************************************/

typedef Matrix<double, 6, 6> Matrix6d;

// 给定误差求J_R^{-1}的近似
Matrix6d JRInv(const SE3d &e) {
    Matrix6d J;
    // 对应公式（2.3中的10.3式）
    J.block(0, 0, 3, 3) = SO3d::hat(e.so3().log());
    J.block(0, 3, 3, 3) = SO3d::hat(e.translation());
    J.block(3, 0, 3, 3) = Matrix3d::Zero(3, 3);
    J.block(3, 3, 3, 3) = SO3d::hat(e.so3().log());
    // 通常se(3)上的左右雅可比J_r形式过于复杂，如果误差接近于零，可以设他们近似于单位矩阵I或者I + 1/2 * J
    // J = J * 0.5 + Matrix6d::Identity();
    // 这里直接设置成了单位矩阵I，也可以将这里注释掉，打开上面的近似公式，看看效果的区别 
    J = Matrix6d::Identity();    
    return J;
}

// 李代数顶点
// 通过read和write函数，将xx.g2o中读取到的数字，存储为sophus的格式，并伪装成g2o内置的SE3顶点
// 如此g2o_viewer也可以识别并渲染他们，事实上除了内部使用Sophus的李代数表示，从外部看起来没有区别。
typedef Matrix<double, 6, 1> Vector6d;

class VertexSE3LieAlgebra : public g2o::BaseVertex<6, SE3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //读取数据
    virtual bool read(istream &is) override {
        double data[7]; //平移加四元数
        for (int i = 0; i < 7; i++)
            is >> data[i];
        setEstimate(SE3d(
            Quaterniond(data[6], data[3], data[4], data[5]),
            Vector3d(data[0], data[1], data[2])
        ));
        return true;
    }
    //将优化的位姿存入内存 
    virtual bool write(ostream &os) const override {
        os << id() << " ";
        Quaterniond q = _estimate.unit_quaternion();
        os << _estimate.translation().transpose() << " ";
        //coeffs顺序是 x y z w ,w是实部
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << endl;
        return true;
    }

    virtual void setToOriginImpl() override {
        _estimate = SE3d(); //李代数
    }

    // 左乘更新
    virtual void oplusImpl(const double *update) override {
        Vector6d upd;   //六维向量 upd接收 update
        upd << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = SE3d::exp(upd) * _estimate; //更新位姿
    }
};

// 两个李代数节点之边
// 定义边  两个李代数顶点的边 边就是两个顶点之间的变换 即位姿之间的变换
class EdgeSE3LieAlgebra : public g2o::BaseBinaryEdge<6, SE3d, VertexSE3LieAlgebra, VertexSE3LieAlgebra> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //读取观测值和构造信息矩阵
    virtual bool read(istream &is) override {
        double data[7];     //平移加四元数
        for (int i = 0; i < 7; i++)
            is >> data[i];
        Quaterniond q(data[6], data[3], data[4], data[5]);
        q.normalize();
        setMeasurement(SE3d(q, Vector3d(data[0], data[1], data[2])));
        for (int i = 0; i < information().rows() && is.good(); i++)
            for (int j = i; j < information().cols() && is.good(); j++) {
                is >> information()(i, j);
                if (i != j) //不是对角线的地方
                    information()(j, i) = information()(i, j);
            }
        return true;
    }
    //这个函数就是为了把优化好的相机位姿放进指定文件中去
    virtual bool write(ostream &os) const override {
        //v1,V2分别指向两个顶点
        VertexSE3LieAlgebra *v1 = static_cast<VertexSE3LieAlgebra *> (_vertices[0]);
        VertexSE3LieAlgebra *v2 = static_cast<VertexSE3LieAlgebra *> (_vertices[1]);
        os << v1->id() << " " << v2->id() << " ";
        SE3d m = _measurement;
        Eigen::Quaterniond q = m.unit_quaternion(); //获取单位四元
        os << m.translation().transpose() << " ";   //传入平移
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " "; //传入四元数

        // information matrix  信息矩阵
        for (int i = 0; i < information().rows(); i++)
            for (int j = i; j < information().cols(); j++) {
                os << information()(i, j) << " ";
            }
        os << endl;
        return true;
    }

    // 误差计算与书中推导一致
    virtual void computeError() override {
        SE3d v1 = (static_cast<VertexSE3LieAlgebra *> (_vertices[0]))->estimate();
        SE3d v2 = (static_cast<VertexSE3LieAlgebra *> (_vertices[1]))->estimate();
        _error = (_measurement.inverse() * v1.inverse() * v2).log();
    }

    // 雅可比计算
    virtual void linearizeOplus() override {
        SE3d v1 = (static_cast<VertexSE3LieAlgebra *> (_vertices[0]))->estimate();
        SE3d v2 = (static_cast<VertexSE3LieAlgebra *> (_vertices[1]))->estimate();
        Matrix6d J = JRInv(SE3d::exp(_error));  // JRInv()函数，提供近似的雅可比计算
        // 尝试把J近似为I？
        //雅克比有两个,一个是误差对相机i位姿的雅克比,另一个是误差对相机j位姿的雅克比
        _jacobianOplusXi = -J * v2.inverse().Adj();
        _jacobianOplusXj = J * v2.inverse().Adj();
    }
};

// 用sophus表达完边和节点后，下面优化步骤和上面例子一样
int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "Usage: pose_graph_g2o_SE3_lie sphere.g2o" << endl;
        return 1;
    }
    ifstream fin(argv[1]);
    if (!fin) {
        cout << "file " << argv[1] << " does not exist." << endl;
        return 1;
    }

    // 设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);   // 设置求解器
    optimizer.setVerbose(true);       // 打开调试输出

    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的数量
    //容器 vectices 和edges 存放各个顶点和边
    vector<VertexSE3LieAlgebra *> vectices;
    vector<EdgeSE3LieAlgebra *> edges;
    while (!fin.eof()) {
        string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT") {
            // 顶点
            VertexSE3LieAlgebra *v = new VertexSE3LieAlgebra();
            int index = 0;
            fin >> index;
            v->setId(index);
            v->read(fin);
            optimizer.addVertex(v);
            vertexCnt++;
            vectices.push_back(v);
            if (index == 0)
                v->setFixed(true);
        } else if (name == "EDGE_SE3:QUAT") {
            // SE3-SE3 边
            EdgeSE3LieAlgebra *e = new EdgeSE3LieAlgebra();
            int idx1, idx2;     // 关联的两个顶点
            fin >> idx1 >> idx2;
            e->setId(edgeCnt++);
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            e->read(fin);
            optimizer.addEdge(e);
            edges.push_back(e);
        }
        if (!fin.good()) break;
    }

    cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;

    cout << "optimizing ..." << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    cout << "saving optimization results ..." << endl;

    // 因为用了自定义顶点且没有向g2o注册，这里保存自己来实现
    // 伪装成 SE3 顶点和边，让 g2o_viewer 可以认出
    ofstream fout("result_lie.g2o");
    for (VertexSE3LieAlgebra *v:vectices) {
        fout << "VERTEX_SE3:QUAT ";
        v->write(fout); //把优化的顶点放进 result_lie.g2o
    }
    for (EdgeSE3LieAlgebra *e:edges) {
        fout << "EDGE_SE3:QUAT ";
        e->write(fout); //把优化的边放进 result_lie.g2o
    }
    fout.close();
    return 0;
}
```

#### 编译

```cmake
cmake_minimum_required(VERSION 2.8)
project(pose_graph)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -O2")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# Eigen
include_directories("/usr/include/eigen3")

# sophus 
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# g2o 
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

add_executable(pose_graph_g2o_lie pose_graph_g2o_lie_algebra.cpp)
target_link_libraries(pose_graph_g2o_lie
        g2o_core g2o_stuff
        ${CHOLMOD_LIBRARIES}
        ${Sophus_LIBRARIES} fmt
        )
```

# 十一、回环检测

## 1. 概述

### 1.1 回环检测的意义

**前端**：完成了特征提取，轨迹和地图的初值计算的问题

后端：对前端所有的数据进行了优化

虽然后端能够估计最大后验误差，但“好模型架不住烂数据”。因为前端计算时都只是考虑相邻时间上的关键帧，所以如果$x_1$有误差，那么基于$x_1$计算得到的$x_2$会扩大误差，从而使得SLAM出现**累积误差**。

**回环**：给出除了相邻帧的一些时隔更加久远的约束。比如$x_1 \sim x_{100}$之间的位姿变换。从而给后端提供更多有效数据，使之得到更好的估计，得到一个**全局一致**的估计。

回环检测的关键在于，如何有效地检测出相机经过同一个地方这件事(只有同一个地方，久远的两帧之间才有约束)

所以回环对于SLAM系统有2个重要意义：

1. 关系到估计的轨迹和地图在**长时间**下的正确性
2. 由于回环提供了当前数据和所有历史数据的关联，回环还可以被用来进行重定位

### 1.2 回环检测的方法

- 不用的方法：对任意两幅图像都做一遍特征匹配，根据正确匹配的数量确定哪两幅图像存在关联。

  缺点：计算量庞大，且由于帧数多但回环少所以效率不高

- 使用的方法：通过**基于外观（Appearance based）**的几何关系，来预计 “哪处可能出现回环”。

  它和前端、后端的估计无关，只根据两幅图像之间的相似性确定回环检测关系。先相似了，再进行回环检测看是否回环了。

基于外观的回环检测，核心问题是**如何计算图像间的相似性**。

首先要排除$s(A,B)=||A-B||$直接两个图的像素矩阵相减的方式来评分，因为像素灰值是一个不稳定的测量值，光照、角度都会对他产生影响。



## 2. 词袋模型

- 词袋(Bag-of-Words)BoW，目的是用“图像上的哪几种特征”来描述一幅图像。

  比如：一张图上描述为一个人，一辆车。另一张图上一个人，一只狗。通过这样的描述来判断这两幅图像是否相似。

- 具体有三步：

  1. 确定人、车、狗等概念---->对应于BoW中的**单词**(word)，许多单词放在一起组成了**字典**(Dictionary)

  2. 确定图像中出现了哪些字典中定义的概念---->我们用单词出现的情况(或直方图)描述整幅图像。

     这就把一图像转换成了一个向量的表达。

  3. 比较上一步中的描述的相似程度。

- 举例：

  有一本字典，上面记录了"人、车、狗"，分别记为$w_1,w_2,w_3$。如果图像A由两个人，一只狗那么就可以记为：
  $$
  A=2\cdot w_1+0\cdot w_2+1\cdot w_3
  $$
  由于字典是固定的，所以只需要用向量$[2,0,1]$来描述这个图像。

  确定图像间的相似性就可以用下公式来描述了：
  $$
  s(a,b)=1-\frac{1}{W}||a-b||_1\tag{1}
  $$

  - $L_1$范数：各元素绝对值之和
  - 当两个向量完全相同时，得到1
  - 当两个向量完全相反时，得到0

## 3. 字典

一个单词是某一类特征的组合，所以字典的生成问题类似于一个**聚类**clustering问题。

在无监督机器学习中，聚类问题很常见，用于让机器自行寻找数据中的规律。

常见的有K-means均值算法：有N个数据，想归成k个类

1. 随机选取k个中心点：$c_1,\cdots,c_k$
2. 对每一个样本，计算它与每个中心点之间的距离，取最小的作为它的归类
3. 重新计算每个类的中心点
4. 如果每个中心点都变化很小，则收敛。否则返回第二步。

### 3.1 字典的结构

如上所诉，根据K-means算法，我们可以把已经提取的大量特征点聚类成一个含有k个单词的字典。

但在查找时，如果简单的让图像的某个特征值和每个单词进行匹对，就会是非常大的计算量。因为我们会用到如Fabmap,Chou-Liu tree等数据机构来提高查找效率。

下面用一种K叉树结构来表达字典：假定有N个特征点，希望构建深度为d，每次分叉为k的树

1. 在根节点，用k-means把所有样本聚成类(实际中为保证聚类均匀性会使用K-means++)
2. 对第一层的每个节点，把属于该节点的样本再聚成k类，得到下一层
3. 以此类推，最后得到叶子层。叶子层也就是所谓的单词words

![在这里插入图片描述](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/202306220156371.png)

这种树结构的中间节点就可以用于快速查找：

- k分支，d深度的树，可以容纳$k^d$个单词。
- 查找某个特征对应哪个单词时，只需要将特征点和中间节点比较d次，就可以找到最后的单词
- 从而实现了对数级别的查找效率

### 3.2 实践：创建字典

## 4. 相似度计算

### 4.1 理论部分

有了字典后，就要通过词袋模型计算任意图像间的相似度。

一副图像提取了N个特征，找到这N个特征对应的单词后，我们希望对单词的区分性/重要性加以评估，从而让我们分辨相似度。

- TF-IDF(Term Frequency-Inverse Document Frequency)频率-逆文档频率是一种文本检索中常用的加权方式

  - TF的思想：某单词再一副图像中经常出现，它的区分度就高
    $$
    TF_i=\frac{n_i}{n}\tag{1}
    $$

    - TF的含义：某个特征在单幅图像中出现的概率
    - 图像A中单词$w_i$出现了$n_i$次，而一共出现的单词次数为n

  - IDF的思想：某单词再字典中出现的频率越低，分类图像的区分度越高
    $$
    IDF_i=log\frac{n}{n_i}\tag{2}
    $$

    - 某叶子节点(单词)$w_i$的特征数量$n_i$，相对于所有特征数量n的比例

  - 于是得到图像A该单词$w_i$的权重为
    $$
    \eta_i=TF_i\times IDF_i \tag{3}
    $$

- 考虑权重后，图像A的词袋就变成了：
  $$
  A=\{(w_1,\eta_1),(w_2,\eta_2),\cdots,(w_N,\eta_N)\}=\mathbf{v}_A\tag{4}
  $$
  通过词袋我们用单个向量$\mathbf{v}_A$描述了一副图像A，由于特征可能落到同一个类中，所以这个向量会存在大量0，而非零值就表示图像A包含了哪些单词。

- 最终我们可以用下式计算两幅图的差异：
  $$
  s(\mathbf{v}_A-\mathbf{v}_B)=2\sum_{i=1}^N|\mathbf{v}_{Ai}|+|\mathbf{v}_{Bi}|-|\mathbf{v}_{Ai}-\mathbf{v}_{Bi}|\tag{5}
  $$

- 

### 4.2 实践：相似度计算



## 5.实验分析与评述



# 十二、建图

# 十三、设计SLAM系统

# 十四、现在与未来



# * 因子图

## 1. Factor Graphs因子图
