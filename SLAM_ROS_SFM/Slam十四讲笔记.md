

[toc]

# 零、学习资料汇总

- 主体是：SLAM十四讲
- 辅助教材：
  - State Estimation for Robotics： 针对第四章：李群与李代数 和 第六章：非线性优化（状态估计）

# 一、预备知识

配套代码：https://github.com/gaoxiang12/slambook2

课后答案参考：https://www.zhihu.com/column/c_1367551490568036352

## 1. Slam全名

- Simultaneous Localization and Mapping 同时定位与地图构建
  - 它是指搭载特定**传感器**的主体，在**没有环境先验信息**的情况下，于**运动过程中**建立**环境**的模型，同时估计自己的**运动**。

## 2. 课后习题

- 线性方程Ax = b的求解，对A和b有哪些要求？（从A的维度和秩角度分析）
  - 若rank(A) < rank(A|b)，则该线性方程为超定方程，无解。（但是可以有最小二乘解
  - 若rank(A) = rank(A|b)，则该线性方程组有解：在此基础上，若rank < A的列数（即变量的个数），则为欠定方程，有无穷解；若rank = A的列数，则为适定方程，具有唯一解。



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

### 3.2 SO(3)上李代数求导

#### 3.2.1 问题描述

- 已知一个世界坐标位于**p**的点，机器人对其产生的观测数据为**z**，机器人到该点的坐标转换为**T**,误差为**w**:$z=Tp+w$

- 因为有误差**w**存在,观测数据**z**不可能精准的等于Tp，从而产生误差$E=Z-Tp$。

  所以对机器人的**位姿估计**也就变成了在得到**N**个观测数据后寻求一个**最优的T**来让整体误差最小化：
  $$
  \mathop{min}_T\ J(\bf{T})=\sum_{i=1}^{N}||z_i-\bf{T}{p}_i||_2^2
  $$

- 求解上述问题显然需要对姿态T求导，但T是李群没有定义的加法。因此要转为李代数求导。

  李代数求导又有2条思路：

  1. 用李代数表示姿态，然后根据李代数加法对**李代数求导**
  2. 对利群左乘或右乘微小扰动(BCH公式)，然后对这个**扰动求导**

#### 3.2.2 李代数求导

一个空间点p经过旋转R后得到$Rp$,求导最后得到：$\frac{\partial\bf{Rp}}{\partial\bf{\phi}}=(-\bf{Rp}^{\wedge}\bf{J}_l)$

- 其中的左乘雅可比式$\bf{J}_l$很复杂，我们不希望计算它，所以使用扰动模型即BCH来计算导数

#### 3.2.3 扰动求导（左乘）

- 这种方式是对$\bf{R}$进行一次扰动$\Delta \bf{R}$,然后看结果相对于扰动的变化率。

  设$\Delta \bf{R}$对应的李代数为$\varphi$,然后对$\varphi$求导:
  $$
  \begin{align*}
  \frac{\partial(\bf{Rp})}{\partial\bf{\varphi}} &=\mathop{lim}_{\varphi \rightarrow 0}\frac{exp(\varphi^{\wedge})exp(\phi^{\wedge})\mathbf{p} -exp(\phi^{\wedge})\mathbf{p} }{\varphi}\\
  &= \mathop{lim}_{\varphi \rightarrow 0}\frac{(I+\varphi^{\wedge})exp(\phi^{\wedge})\mathbf{p} -exp(\phi^{\wedge})\mathbf{p} }{\varphi}\\
  &= \mathop{lim}_{\varphi \rightarrow 0}\frac{\varphi^{\wedge}\bf{Rp}}{\varphi} \\
  &= \mathop{lim}_{\varphi \rightarrow 0}\frac{-(\bf{Rp})^{\wedge}\ \varphi}{\varphi} \\
  &= -(\mathbf{Rp})^{\wedge}
  \end{align*}
  $$

- 可见相比3.2.2，使用扰动来计算导数就不用去求解雅可比J了。

### 3.3 SE(3)上李代数求导

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
  \end{array}\right]
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
  
    - $x=\{x_1,\cdots ,x_N\}$:机器人位姿
  
    - $y=\{y_1,\cdots,y_M\}$：路标坐标
  
    - z:所有时刻的观测数据（观测点在像素坐标下的坐标）
  
    - u:所有时刻的输入（机器人各种传感器的数据IMU,速度等等）
  
      如果没有输入，只有一张张图像，状态估计的条件概率分布为：$P(x,y|z)$,此时SLAM问题也就变成了SFM三维重建问题。
  
  - 利用**贝叶法则**估计状态变量的条件分布
    $$
    P(x,y|z,u)=\underbrace{\frac{P(x,y|z,u)P(x,y)}{P(z,u)}}_{后验}\propto\underbrace{P(x,y|z,u)}_{似然}\underbrace{P(x,y)}_{先验} \tag{5}
    $$
  
    - 直接求后验分布是困难的，但是**求一个状态最优估计，使得该状态下后验概率最大化是可行的**
  
      $$
      (x,y)^*=argmaxP(x,y|z,u)P(x,y)\tag{5.1}
      $$
      
    - 贝叶斯法则告诉我们，求解最大后验概率 等价于最大化 **似然和先验的乘积**
    
    - 如果我们不知道机器人位姿或路标，就没有了先验，那么就变成了求解**最大似然估计**(MLE,Maximize Likelihood Estimation)，即：
      $$
      (x,y)^*=argmaxP(x,y|z,u)\tag{5.2}
      $$
      
    
      - **似然**：在现在位姿下，可能产生怎么样的观测数据
      - **最大似然估计**：在什么状态下，最可能产生现在观测到的数据。

### 1.2 转为最小二乘问题

slam的状态估计问题，可以变成一个最小二乘问题least squares method。

最小二乘法是一种数学优化建模方法。它通过最小化误差的平方来寻找最佳函数和真实数据相匹配。利用最小二乘法可以简便的求得未知的数据，并使得求得的数据与实际数据之间误差的平方和为最小。

- 由1.1中观测公式2和噪声公式4，可结合得到观测数据条件概率为：
  $$
  P(\mathbf{z}_{j,k}|\mathbf{x}_k,\mathbf{y}_j)=\mathcal{N}(h(\mathbf{y}_j,\mathbf{x}_k),\mathbf{Q}_{k,j})\tag{6}
  $$

  -  h()：观测方程
  -  Q：高斯分布协方差矩阵
  
  它依然是一个高斯分布。
  
- 对于**单次观测的最大似然估计**，可以使用**最小化负对数**来求一个高斯分布的最大似然。

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
     x^* &=arg\ maxP(x|u,z)=arg\ maxP(u,z|x)		\\
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

- 求7式右侧关于$\Delta x$的导数并令其为0,得到**增量方程**如下：
  $$
  \mathbf{J}(x)f(x)+\mathbf{J}(x)\mathbf{J}^T(x)\Delta x=0\\
  \underbrace{\mathbf{J}(x)\mathbf{J}^T(x)}_{H(x)}\Delta x=\underbrace{-\mathbf{J}(x)f(x)}_{g(x)} \tag{8}\\
  H\Delta x=g
  $$

  - 增量方程是关于$\Delta x$的线性方程组
  - 也可以称之为高斯牛顿方程(Gauss-Newton equation)或者正规方程(Normal equation)
  - 相比于牛顿法中的5式，高斯牛顿法用$\mathbf{J}(x)\mathbf{J}^T(x)$**作为牛顿法中二阶海塞矩阵H的近似，从而省略了计算复杂的海塞矩阵过程。**

- 如上，得到高斯牛顿法的算法步骤：

  1. 给定初始值$x_0$
  2. 对于第k次迭代，求出当前的雅可比矩阵$\mathbf{J}(x_k)$和误差$f(x_k)$
  3. 求解增量方程：$H\Delta x_k=g$
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

- 如上，得到L-M算法：

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

## 3. 实践：曲线拟合问题

### 3.0 问题+解法

- 问题描述：

  考虑一条满足方程：$y=exp(ax^2+bx+c)+w$ 的曲线，a/b/c为参数，$w\sim(0,\sigma^2)$是高斯噪声。

  假设有N个关于x,y的观测数据，想根据这些数据求出曲线的参数。

  得到最小二乘问题：
  $$
  \mathop{min}_{a,b,c}\frac{1}{2}\sum_{i=1}^N|| y_i-exp(ax_i^2+bx_i+c) ||^2
  $$

- 解法：

  - 定义误差：$e_i=y_i-exp(ax_i^2+bx_i+c)$

  - 求误差对每个状态变量的导数：
    $$
    \begin{align}
    \frac{\partial e_i}{\partial a}&=-x_i^2exp(ax_i^2+bx_i+c)\\
    \frac{\partial e_i}{\partial b}&=-x_iexp(ax_i^2+bx_i+c)\\
    \frac{\partial e_i}{\partial c}&=-exp(ax_i^2+bx_i+c)
    \end{align}
    $$

  - 根据8式列出高斯牛顿法的增量方程：
    $$
    \big( \sum_{i=1}^{100}\mathbf{J}_i(\sigma^2)^{-1}\mathbf{J}_i^T \big)\Delta x_k = \sum_{i=1}^{100}-\mathbf{J}_i(\sigma^2)^{-1}e_i
    $$

    - 其中$\mathbf{J}_i=[\frac{\partial e_i}{\partial a},\frac{\partial e_i}{\partial b},\frac{\partial e_i}{\partial c}]^T$
    - 这里$\sigma$是噪声sigma值，乘上J的平方，而不是J的参数

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
  double ar = 1.0, br = 2.0, cr = 1.0;         // 真实参数值
  double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值
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
      double error = yi - exp(ae * xi * xi + be * xi + ce);
      Vector3d J; // 雅可比矩阵
      J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);  // de/da
      J[1] = -xi * exp(ae * xi * xi + be * xi + ce);  // de/db
      J[2] = -exp(ae * xi * xi + be * xi + ce);  // de/dc

      H += inv_sigma * inv_sigma * J * J.transpose();
      b += -inv_sigma * inv_sigma * error * J;

      cost += error * error;
    }

    // 求解线性方程 Hx=b
    // Vector3d x = A.ldlt().solve(b);//求解Ax=b
    Vector3d dx = H.ldlt().solve(b);
    if (isnan(dx[0])) {   // c++11提供的isnan用于判断是否是非数NaN：表示未定义或不可表示的值
      cout << "result is nan!" << endl;
      break;
    }

    if (iter > 0 && cost >= lastCost) {
      cout << "cost: " << cost << ">= last cost: " << lastCost << ", break." << endl;
      break;
    }

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

// g2o第一步： 继承g2o顶点基本类，创建我们自己的曲线模型的顶点，模板参数：优化变量维度和数据类型
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 重写顶点的重置函数，因为本问题的优化参数是a,b,c，所以是3个
  virtual void setToOriginImpl() override {
    _estimate << 0, 0, 0;
  }

  // 重写顶点的更新函数 x_{k+1} = x_k +dx
  // 简单的加法为什么更新要自己写，g2o不帮我们完成呢？ 因为在向量空间中的确是简单的加法，但如果x是位姿(位移矩阵)就不一定有加法了，根据第四讲需要用左乘或右乘的方式更新。
  virtual void oplusImpl(const double *update) override {
    _estimate += Eigen::Vector3d(update);
  }

  // 存盘和读盘：因为本例不需要读写操作，所以留空
  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}
};

// g2o第二步： 继承g2o边基本类，创建我们自己的误差模型 模板参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

  // 重写边的误差计算，计算曲线模型误差
  virtual void computeError() override {
    // 取出边所连接的的顶点的当前估计值
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    // 根据曲线模型 和 它的观测值进行比较
    _error(0, 0) = _measurement - std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
  }

  // 重写边的雅可比计算，计算每条边相对于顶点的雅可比
  virtual void linearizeOplus() override {
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
    _jacobianOplusXi[0] = -_x * _x * y;
    _jacobianOplusXi[1] = -_x * y;
    _jacobianOplusXi[2] = -y;
  }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

public:
  double _x;  // x 值， y 值为 _measurement
};

int main(int argc, char **argv) {
  double ar = 1.0, br = 2.0, cr = 1.0;         // 真实参数值
  double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值
  int N = 100;                                 // 数据点
  double w_sigma = 1.0;                        // 噪声Sigma值
  double inv_sigma = 1.0 / w_sigma;
  cv::RNG rng;                                 // OpenCV随机数产生器

  vector<double> x_data, y_data;      // 数据
  for (int i = 0; i < N; i++) {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
  }

  // 构建图优化，先设定g2o
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;  // 每个误差项优化变量维度为3，误差值维度为1
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型

  // 梯度下降方法，可以从GN, LM, DogLeg 中选
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
    g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;     // 图模型
  optimizer.setAlgorithm(solver);   // 设置求解器
  optimizer.setVerbose(true);       // 打开调试输出

  // 往图中增加顶点
  CurveFittingVertex *v = new CurveFittingVertex();
  v->setEstimate(Eigen::Vector3d(ae, be, ce));
  v->setId(0);
  optimizer.addVertex(v);

  // 往图中增加边
  for (int i = 0; i < N; i++) {
    CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
    edge->setId(i);
    edge->setVertex(0, v);                // 设置连接的顶点
    edge->setMeasurement(y_data[i]);      // 观测数值
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma)); // 信息矩阵：协方差矩阵之逆
    optimizer.addEdge(edge);
  }

  // 执行优化
  cout << "start optimization" << endl;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  // 输出优化值
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

- 与线性方法的区别：

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
  3式中的误差项称为**重投影误差(Reprojection error) e**：3D点的投影位置与观测位置的差
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
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
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

     这样可以避免计算匹配描述子，而光流法的计算时间小于计算匹配描述子的时间

  2. 在计算关键点不计算描述子的同时不保留特征点，同时使用**直接法(Direct Method)**计算特征点的下一时刻图像中的位置。

     这可以省去计算描述子的时间，也省去了计算光流的时间。

- 直接法相比特征点法：

  - 直接法会根据图像的**像素灰度信息**同时估计相机运动和点的投影，不要求提取到的点必须为角点。
  - 直接法不需要知道点和点之间的对应关系，而是通过最小化**光度误差(Photometric error)**来求他们。

# 九、后端优化：BA图优化

前端里程计能给出一个短时间内的轨迹和地图，但由于不可避免的误差累积，这个地图在长时间内是不准确的。

所以我们需要构建一个更大尺度和规模的优化问题，以考虑长时间内的最优轨迹和地图。

## 1. 概述

### 1.1 状态估计的概率解释

- 我们用“批量的”信息处理方式，来保证整个运动轨迹在较长时间内都能保持最优状态

  - 批量的batch:不仅使用过去的信息更新自己的状态，也用未来的信息来更新以前的某一状态。、
  - 渐进的incremental：当前的状态只由过去的时刻决定。用滤波器求解状态估计就是这种情况。

- 由于噪声，位姿$\mathbf{x}$和路标$\mathbf{y}$可以看成**服从某种概率分布的随机变量**。

  所以问题变成了：**当存在一些运动数据$\mathbf{u}$和观测数据$\mathbf{z}$时，如何估计状态变量x/y的高斯分布。**

  这是个状态估计问题。

  而由六，1.1可知：**批量状态估计问题可以转化为最大似然估计问题，并使用最小二乘法进行求解**

- 问题的定量推到：

  1. 定义变量$\mathbf{x}_k=\{\mathbf{x}_k,\mathbf{y}_1,\cdots,\mathbf{y}_m\}$为k时刻所有未知量，包含当前时刻位姿和m个路标点。$\mathbf{z}_k$为k时刻所有观测。得到运动/观测方程
     $$
     \begin{cases}
     \mathbf{x}_k=f(\mathbf{x}_{k-1},\mathbf{u}_k)+\mathbf{w}_k \\
     \mathbf{z}_k = h(\mathbf{x}_k)+\mathbf{v}_k \\
     \end{cases}\ \ \ k=1,\cdots,N\tag{1}
     $$

  2. 用过去0到k中的数据来估计现在的状态分布,然后利用贝叶斯法则得到贝叶斯估计：
     $$
     P(\mathbf{x}_k|\mathbf{x}_0,\mathbf{u}_{1:k},\mathbf{z}_{1:k})\propto P(\mathbf{z}_k|\mathbf{x}_k)P(\mathbf{x}_k|\mathbf{x}_0,\mathbf{u}_{1:k},\mathbf{z}_{1:k-1}) \tag{2}
     $$

     - 第一项是**似然**，第二项是**先验**

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
  下面使用卡尔曼滤波器将k-1时刻的状态分布推导至k时刻，最终得到线性系统的最优无偏估计

  - 由于基于马尔可夫性假设，所以在实际编程中，我们只需要维护一个状态变量
  - 而又由于状态变量服从高斯分布，我们只需要维护状态变量的均值矩阵$\hat{\mathbf{x}}_k$和协方差矩阵$\hat{\mathbf{P}}_k$

- 卡尔曼滤波由如下两步组成，并用$\hat{a}$表示后验，$\check{a}$表示先验分布：

  1. **预测**(prediction)

     从上一时刻的状态，根据输入信息（有噪声）推断当前时刻的状态分布
     $$
     \check{\mathbf{x}}_k=\mathbf{A}_k\hat{\mathbf{x}}_{k-1}+\mathbf{u}_k\\
     \check{\mathbf{P}}_k=\mathbf{A}_k\hat{\mathbf{P}}_{k-1}\mathbf{A}_k^T+\mathbf{R} \tag{6}
     $$

  2. **更新**（correctiion/measurment update）

     比较在当前时刻的状态输入（也叫**测量**值）和**预测**的状态变量， 从而对预测出的系统状态进行修正.

     - 先计算卡尔曼增益K：
       $$
       \mathbf{K}=\check{\mathbf{P}}_k\mathbf{C}_k^T(\mathbf{C}_k\check{\mathbf{P}}_k\mathbf{C}_k^T+\mathbf{Q}_k)^{-1}\tag{7}
       $$

     - 然后计算后验概率分布
       $$
       \hat{\mathbf{x}}_k=\check{\mathbf{x}}_k+\mathbf{K}(\mathbf{z}_k-\mathbf{C}_k\check{\mathbf{x}}_k)\\
       \hat{\mathbf{P}}_k=(\mathbf{I}-\mathbf{KC}_k)\check{\mathbf{P}}_k \tag{8}
       $$

- 

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
       \hat{\mathbf{P}}_k=(\mathbf{I}-\mathbf{K}_k\mathbf{H})\check{\mathbf{P}}_k \tag{8}
       $$
       

### 1.4 EKF的讨论

当想要在某段时间内估计某个不确定量时，EKF是个很好的方法。但在计算资源充足或估计量复杂的场合，非线性优化是种更好的方法。

EKF有着如下局限性：

1. 滤波器基于马尔可夫性，这就导致了只能使用前一时刻的数据，而以前的数据都被忽略了。
2. 一节泰勒展开无法完全线性近似非线性方程，从而产生非线性误差。
3. EKF需要维护存储状态变量的均值和方差，在大场景中，如果把路标也放进状态，就使得这个存储规模呈平方增长，所以不适合大场景
4. EKF没有异常检测机制，导致系统在异常值时容易发散。

所以在相同计算量下，非线性优化通常比EKF在精度和鲁棒性上更好。



## 2. BA与图优化

# 十、后端优化：位姿图

# 十一、回环检测

# 十二、建图

# 十三、设计SLAM系统

# 十四、现在与未来



