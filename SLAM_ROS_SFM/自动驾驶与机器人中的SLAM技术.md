# 零、资料汇总

- 官方原版[git仓库](https://github.com/gaoxiang12/slam_in_autonomous_driving)

  包含书上的代码还有数据集

  - 各章代码在`src/chxx`下
  - 共有代码在`src/common`下

- 他人上深蓝课的[git仓库](https://github.com/ClarkWang1214/slam_in_autonomous_driving_shenlan_hw?tab=readme-ov-file)

  额外的有课后作业答案和修改代码

## 1. 本地编译

1. [GIT仓库](https://github.com/Fernweh-yang/LiDAR-SLAM-code-comments)下载源码

2. 安装ros noetic

3. 安装库

   ```
   sudo apt install -y ros-noetic-pcl-ros ros-noetic-velodyne-msgs libopencv-dev libgoogle-glog-dev libeigen3-dev libsuitesparse-dev libpcl-dev libyaml-cpp-dev libbtbb-dev libgmock-dev
   ```

4. 安装[Pangolin](https://github.com/stevenlovegrove/Pangolin)，可参考安装软件参考

5. 安装thirdparty/g2o

6. 最后在总的文件夹下用如下命令安装所有的代码

   ```
   mkdir build
   cd build
   cmake ..
   make -j4
   ```

- 如果用的zsh可能没包含Ros库，需要

  ```
  vim ~/.zshrc
  source /opt/ros/noetic/setup.zsh
  ```

## 2. Docker运行

- 构建image

  执行`.`当前文件夹下的dockerfile，构建名为sad:v1的image

  ```shell
  docker build -t sad:v1 .
  ```

- 构建容器

  该脚本文件中定义好了docker run ....各种环境变量、卷等信息。

  **如果机子没有gpu记得把--gpu all删掉**

  ```shell
  ./docker/docker_run.sh
  ```

- 在容器内编译运行

  如果cpu核数不够就用make -j4
  
  ```shell
  cd ./thirdparty/g2o
  mkdir build
  cd build
  cmake ..
  make -j8
  make install
  cd /sad
  mkdir build
  cd build
  cmake ..
  make -j8
  ```

- 如果遇到找不到`catkinConfig.cmake`

  下载vim然后：

  ```
  vim ~/.bashrc
  # 加入到最后
  source /opt/ros/noetic/setup.bash
  ```

## 3. Vscode

需要在`c_cpp_properties.json`中加入

```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/usr/include/eigen3",
                "/usr/include/opencv4",
                "/opt/ros/noetic/include",
                "/usr/include/pcl-1.10"
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

## 4. gflags和glog

本书中用gflags作为参数管理工具，glog作为日志管理工具。

### 4.1 gflags

gflags是谷歌开发的一个处理命令行参数的c++库，不直接用argv而用gflags原因有：

1. **类型安全和易用性**：`gflags` 允许开发者为不同的参数定义不同的类型，如整数、浮点数、布尔值等，而直接使用 `argv` 需要手动进行类型转换，容易出错并且不够直观。
2. **自动生成帮助文档**：`gflags` 可以自动生成命令行帮助文档，其中包含了每个命令行参数的描述、默认值等信息，使得用户能够更容易地了解程序的功能和如何使用它。而使用 `argv` 则需要手动编写帮助文档，增加了开发和维护的工作量。
3. **可扩展性**：`gflags` 具有很好的可扩展性，可以轻松地添加新的命令行参数，并且支持各种自定义选项，如参数验证、参数组织等。这使得开发者能够更灵活地定义和管理命令行参数。
4. **与日志库的集成**：`gflags` 与 Google 提供的日志库 `glog` 集成良好，可以方便地将命令行参数与日志记录结合起来，从而更好地管理程序的配置和输出。

### 4.2 glog

`glog` 是 Google 提供的一个 C++ 日志记录库，用于在 C++ 程序中记录日志消息。

## 5.本书案例汇总：

1. 二、3：运动学：展现了物体圆周运动下四元数下的旋转更新(2.2.13)和SO3下的旋转更新(2.1.3)
2. 三、2.2：IMU递推：只使用IMU数据进行轨迹的推算
3. 三、3.2：RTK读数的显示:
4. 三、5：实现ESKF的组合导航
5. 四、2：实现预积分
6. 五、1.2：点云表达
7. 五、2.1，2.2，2.3，2.4 各种最近邻的实现
8. 五、3.2，3.4 平面和直线拟合
9. 六、2.1，2.2，2.4，2.6，2.7各种扫描匹配算法的实现
10. 六、3.2，3.2占据栅格地图的生成
11. 六、4.2子地图，5.1,5.2回环检测
12. 

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

# 二、数学基础

## 1. 坐标系

车辆本体坐标系一般使用前(x)左(y)上(z)，相机坐标系一般使用右(x)下(y)前(z))的顺序

本书的坐标系变换定义为：
$$
\mathbf{p}_w=\mathbf{R}_{wb}\mathbf{p}_b+\mathbf{t}_{wb}\\
\mathbf{p}_w=\mathbf{T}_{wb}\mathbf{p}_b
$$

- $\mathbf{p}_w$：世界坐标系下的p点坐标
- $\mathbf{p}_b$：车辆坐标系下的p点坐标
- $\mathbf{R}_{wb},\mathbf{t}_{wb},\mathbf{T}_{wb}$​：是**向量之间的坐标变换**，是为了得到车辆坐标下的点在世界坐标下的坐标。所以这些位姿变换不是1.1.1的坐标轴(基)之间的变换，而是1.1.2/3的$A^{-1}_{bw}$的向量变换。

### 1.1 基/向量/矩阵变换

- 假设有两个坐标系：

  - 小蓝 X 坐标系：$\hat{i},\ \hat{j}$
  - 小粉 X‘ 坐标系：$\hat{b_1},\ \hat{b_2}$
  - 小粉坐标系的基：$\hat{b_1},\ \hat{b_2}$在自身看来是$\left [\begin{array}{cccc}
    1  \\
    0  \\
    \end{array}\right]\left [\begin{array}{cccc}
    0  \\
    1   \\
    \end{array}\right]$在小蓝看来是$\left [\begin{array}{cccc}
    3  \\
    1   \\
    \end{array}\right]\left [\begin{array}{cccc}
    1  \\
    2   \\
    \end{array}\right]$​。

- **基**变换

  从小蓝到小粉的变换矩阵是$A_{蓝粉}=\left [\begin{array}{cccc}
  3 & 1 \\
  1 & 2  \\
  \end{array}\right]$​
  $$
  \begin{align}
  X'&=AX\\
  X'&=\left [\begin{array}{cccc}
  3 & 1 \\
  1 & 2  \\
  \end{array}\right]\left [\begin{array}{cccc}
  1 & 0 \\
  0 & 1  \\
  \end{array}\right]
  \end{align}\tag{1.1.1}
  $$
  
- **向量**从小粉 X‘ --> 小蓝 X 

  小粉坐标系下的$P'=\left [\begin{array}{cccc}
  -1  \\
  2  \\
  \end{array}\right]$在小蓝坐标系下的P应该是：
  $$
  \begin{align}
  P&=AP'\\
  P&=\left [\begin{array}{cccc}
  3 & 1 \\
  1 & 2  \\
  \end{array}\right]\left [\begin{array}{cccc}
  -1  \\
  2  \\
  \end{array}\right]
  \end{align}\tag{1.1.2}
  $$

- **向量**从小蓝 X --> 小粉 X‘ 

  小蓝坐标系下的$P=\left [\begin{array}{cccc}
  3  \\
  2  \\
  \end{array}\right]$在小粉坐标系下的P'应该是：
  $$
  \begin{align}
  P'&=A^{-1}P\\
  P'&=\left [\begin{array}{cccc}
  3 & 1 \\
  1 & 2  \\
  \end{array}\right]^{-1}\left [\begin{array}{cccc}
  3  \\
  2  \\
  \end{array}\right]
  \end{align}\tag{1.1.3}
  $$

- **矩阵**从小蓝 X --> 小粉 X‘ 

  如果有个在$\left [\begin{array}{cccc}
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
  M'=A^{-1}MA\tag{1.1.4}
  $$

  1. 需要先$AX'$将向量X'转为和线性变换M一样的坐标系，再进行M变换
  1. 最后结果再由$A^{-1}$转为原有基向量

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
     这里的$\omega$的物理意义是瞬时角速度。
     
     >  如果对$RR^T=I$求导会得到：
     > $$
     > \dot{R}=\phi^{\wedge}R
     > $$
     > $\phi$的意义是SO(3)的李代数，也是R所对应的旋转向量

- 解泊松方程后可得**处理角速度**的2.1.2, 2.1.3, 2.1.4式：

  给定初值$t_0$时刻的旋转矩阵为$\mathbf{R}(t_0)$，那么微分方程2.1.1式的解为：
  $$
  \mathbf{R}(t)=\mathbf{R}(t_0)exp(\omega^{\wedge}(t-t_0))\tag{2.1.2}
  $$
  记$\Delta t=t-t_0$则可得到2.1.2式的离散时间下的格式：
  $$
  \mathbf{R}(t)=\mathbf{R}(t_0)exp(\omega\Delta t) \tag{2.1.3}
  $$
  对2.3式的指数映射一阶泰勒近似后可得：
  $$
  \mathbf{R}(t_0+\Delta t)\approx\mathbf{R}(t_0)(\mathbf{I}+\omega^{\wedge}\Delta t)\tag{2.1.4}
  $$
  
- 

​	可以发现：2.1.3式是2.1.2式的在离散时间下的形式；2.1.4式又是2.1.3式的线性近似形式

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
  

### 2.3 四元数的李代数与旋转向量$\phi$，角速度$\omega$间的转换

14讲中已知旋转矩阵$R$的李代数是旋转向量$\phi$

- 四元数的李代数与旋转向量$\phi$的关系
  
  通过和旋转矩阵$R$的比较，可以得到四元数的李代数:纯虚四元数$ \varpi=[0,\omega]^T\in\mathcal{Q}$ 和  旋转向量$\phi$ 之间的关系：
  $$
  \varpi=[0,\frac{1}{2}\phi]^T, 即 \omega=\frac{1}{2}\phi\tag{2.2.9}
  $$
  可以发现四元数表达式的角速度: $\omega$ 正好是$R$的李代数: $\phi$​ 的一半。这与四元数在旋转一个向量时要乘两遍相对应。
  
- 旋转向量和角速度之间的关系：

  在2.1.1中得到
  $$
  \mathbf{\dot{R}}=\mathbf{R}\mathbf{\omega}^{\wedge} \tag{2.1.1}
  $$
  SO3指数映射为:
  $$
  R=Exp(\omega)=exp(\omega^{\wedge})\tag{2.2.10}
  $$
  最终SO3的旋转更新式为2.1.3式：
  $$
  \mathbf{R}(t)=\mathbf{R}(t-1)exp(\omega\Delta t) \tag{2.1.3}
  $$
  
- 四元数和角速度之间的关系：

  为了和2.1.1中SO(3)的角速度定义一致，所以2.2.11式这里要乘0.5
  $$
  \dot{q}=\frac{1}{2}q[0,\omega]^T=\frac{1}{2}q(2\varpi) \tag{2.2.11}
  $$
  四元数指数映射为:
  $$
  q=Exp(\omega)=exp([0,\frac{1}{2}\omega]^T)\tag{2.2.12}
  $$
  最终四元数的旋转更新式为
  $$
  \begin{align}
  q(t)&=qExp(\omega)\approx q(t-1)_{unit}[1,\frac{1}{2}\omega\Delta t]\\ \tag{2.2.13}
  &= q(t-1)_{unit}[1,\frac{1}{2}\omega_xdt,\frac{1}{2}\omega_ydt,\frac{1}{2}\omega_zdt]
  \end{align}
  $$

  > $q(t-1)_{unit}$是指取上一时刻q的单位四元数，即$\frac{q}{||q||}$
  >
  > $\omega_x$是绕x轴的角速度

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

展现了物体圆周运动下四元数下的旋转更新(2.2.13)和SO3下的旋转更新(2.1.3)

```c++
/// 本节程序演示一个正在作圆周运动的车辆
#include <gflags/gflags.h>  // 程序参数管理工具
#include <glog/logging.h>   // 日志管理工具

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "tools/ui/pangolin_window.h"

/// 车辆的角速度与线速度可以在flags中设置
// 定义命令行参数
DEFINE_double(angular_velocity, 10.0, "角速度（角度）制");
DEFINE_double(linear_velocity, 5.0, "车辆前进线速度 m/s");
DEFINE_bool(use_quaternion, false, "是否使用四元数计算");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);                 // 初始化日志类，argv[0]是程序的名称
    FLAGS_stderrthreshold = google::INFO;               // 设置了 glog 输出到标准错误流的最低日志级别为 INFO。这意味着只有 INFO 级别及以上的日志消息才会被输出到标准错误流。
    FLAGS_colorlogtostderr = true;                      // 在输出到标准错误流时否启用彩色日志。
    google::ParseCommandLineFlags(&argc, &argv, true);  // 解析命令行参数并初始化相应的 gflags 变量, true参数表示如果有不符合规范的参数打印出错误信息。

    /// 可视化
    // 使用tools中的pangolin_window
    sad::ui::PangolinWindow ui;
    if (ui.Init() == false) {
        return -1;
    }

    double angular_velocity_rad = FLAGS_angular_velocity * sad::math::kDEG2RAD;  // 弧度制角速度
    SE3 pose;                                                                    // TWB表示的位姿
    Vec3d omega(0, 0, angular_velocity_rad);                                     // 角速度矢量
    Vec3d v_body(FLAGS_linear_velocity, 0, 0);                                   // 本体系速度
    const double dt = 0.05;                                                      // 每次更新的时间

    while (ui.ShouldQuit() == false) {
        // 更新自身位置
        Vec3d v_world = pose.so3() * v_body;	// 世界坐标系下的速度
        pose.translation() += v_world * dt;		// 更新位姿的位移部分

        // 更新自身旋转
        if (FLAGS_use_quaternion) {
            // 四元数下的旋转更新
            // 笔记公式2.2.13
            Quatd q = pose.unit_quaternion() * Quatd(1, 0.5 * omega[0] * dt, 0.5 * omega[1] * dt, 0.5 * omega[2] * dt);
            q.normalize();
            pose.so3() = SO3(q);
        } else {
            // SO3下的旋转更新
            // 笔记公式：2.1.3
            pose.so3() = pose.so3() * SO3::exp(omega * dt);
        }
    
        LOG(INFO) << "pose: " << pose.translation().transpose();    // 将一条信息(INFO级别)记录到日志中，glog默认输出到终端
        ui.UpdateNavState(sad::NavStated(0, pose, v_world));

        usleep(dt * 1e6);   // usleep微秒为单位，1秒=1e6w微妙
    }

    ui.Quit();
    return 0;
}
```

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

卡尔曼滤波器之所以被称为滤波器，是因为它可以将不想要的噪声成分从测量数据中滤除，只保留对我们真正感兴趣的状态信息。具体来说，卡尔曼滤波器利用系统状态的预测值和测量值之间的差异来不断调整对状态的估计，使之更加接近真实值。

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

### 1.0 IMU的运动模型

由二、数学基础可知一个系统的运动模型为：
$$
\begin{aligned}&\dot{R}=R\omega^{\wedge},\quad\text{或}\quad\dot{q}=\frac{1}{2}q\omega,\\&\dot{p}=v,\\&\dot{v}=a.\end{aligned}\tag{1.0.1}
$$
旋转的运动模型推导见二、2.1.1和2.2.11

### 1.1 IMU的测量模型

> IMU通常由陀螺仪(Gyroscope)和加速度计(Accelerator)组成。我们可以通过IMU测量运动载体的惯性来推断物体本身的状态。
>
> 陀螺仪可以测量物体的角速度。
>
> 加速度计可以测量物体的加速度。

IMU的测量值$\widetilde{a},\widetilde{\omega}$就是车辆在车坐标系下的加速度和车自身的角速度：
$$
\widetilde{a}=R^T(a-g)\\
\widetilde{\omega}=\omega \tag{1.1.1}
$$
这里的$R^T=R^{-1}$加上下标是$R_{bw}$，将世界坐标系下的物理量转换到车坐标系下；g是重力加速度。

实际中如果IMU不在车辆正中心，还会测量到旋转导致的离心力、科氏力和角加速度。各种悬挂、刷子等机械导致的振动也会影响IMU。

### 1.2 IMU测量模型中加入噪声

IMU噪声由两部分组成：测量噪声(Measurement Noise)和零偏(Bias)。

- 加入噪声的**测量模型**：
  $$
  \widetilde{a}=R^T(a-g)+b_a+\eta_a\\
  \widetilde{\omega}=\omega+b_g+\eta_g \tag{1.2.1}
  $$

  - $b_a,\eta_a$：加速度计的零偏和测量噪声
  - $b_g,\eta_g$​：陀螺仪的零偏和测量噪声

  需要注意，这里对零偏和测量噪声分别用布朗运动和高斯过程来数学建模是一种对现实世界的简化。真实的震动、温度等影响并未考虑在内。这些影响可以依靠状态估计算法来削弱。

- **IMU测量噪声**：由高斯过程表示

  被认为是一个方差为$Cov(\eta)$的零均值白噪声高斯过程。

  一个均值为0，协方差为$\Sigma$的白噪声高斯过程随机变量为$w(t)$:
  $$
  w(t)\sim\mathcal{GP}(0,\Sigma\delta(t-t'))\tag{1.2.2}
  $$

  - $\Sigma$：能量谱密度矩阵

  - $\delta$：狄拉克函数

    > 狄拉克函数：可以轻松的从连续时间的高斯过程推导离散时间采样之后的IMU测量噪声。
    >
    > 高斯过程：Gaussian Process是一种概率模型，通常用于建模连续变量之间的关系。它可以被视为无限维的高斯分布，用于描述函数的分布，而不是特定的随机变量。在高斯过程中，任何有限个变量的联合分布都是高斯分布。
    >
    > 布朗运动：实际上就是导数为高斯过程的随机运动。

  由于角速度的是角度的导数，加速度是速度的导数，所以测量噪声也可以理解为角度/速度的随机运动。

- **零偏**：由布朗运动表示

  由IMU内部机电装置导致没有规律，1.2.1式中只是假定其数学模型为b。大部分情况如此建模是足够的。被认为是一个布朗运动。

  一个普通的零偏b的布朗运动可以建模为：
  $$
  \dot{b}(t)=\eta_b(t)\tag{1.2.3}
  $$
  其中$\eta_b(t)$也是一个高斯过程，所以展开后加速度计和陀螺仪的零偏可建模为：
  $$
  \begin{align}
  \dot{b}_a(t)&=\eta_{ba}(t)\sim\mathcal{GP}(0,Cov(b_a)\delta(t-t'))\\
  \dot{b}_g(t)&=\eta_{bg}(t)\sim\mathcal{GP}(0,Cov(b_g)\delta(t-t'))
  \end{align}\tag{1.2.4}
  $$

### 1.3 IMU的离散时间噪声模型

现实中IMU会按固定间隔时间对运动物体的惯性进行采样，因此数据是离散的。离散模型的推导见[论文](https://ieeexplore.ieee.org/document/1642588)，下面直接给出结论。

- 离散噪声模型

    陀螺仪和加速度计的离散测量噪声模型：
    $$
    \eta_{g}\left(k\right)\sim N\left(0,\frac{1}{\Delta t}Cov\left(\eta_{g}\right)\right)\\\eta_{a}\left(k\right)\sim N\left(0,\frac{1}{\Delta t}Cov\left(\eta_{a}\right)\right)\tag{1.3.1}
    $$
    零偏的离散模型：
    $$
    b_{g}\left(k+1\right)-b_{g}\left(k\right)\sim\mathcal{N}\left(0,\Delta tCov\left(b_{g}\right)\right)\\b_{a}\left(k+1\right)-b_{a}\left(k\right)\sim\mathcal{N}\left(0,\Delta tCov\left(b_{a}\right)\right)\tag{1.3.2}
    $$
    在现实中可以不考虑用协方差矩阵来表示，而是用对角矩阵，即忽略了各个轴之间的相关性。在程序中通常使用$\sigma_g,\sigma_a$来表示IMU测量噪声的标准差，$\sigma_{bg},\sigma_{ba}$来表示零偏游走的标准差：
    $$
    \begin{aligned}&\sigma_{g}\left(k\right)=\frac{1}{\sqrt{\Delta t}}\sigma_{g},\sigma_{a}\left(k\right)=\frac{1}{\sqrt{\Delta t}}\sigma_{a},\\&\sigma_{bg}\left(k\right)=\sqrt{\Delta t}\sigma_{bg},\sigma_{ba}\left(k\right)=\sqrt{\Delta t}\sigma_{ba}.\end{aligned}\tag{1.3.3}
    $$
    
- 噪声标准差的物理单位：

    - 离散的他们和被测物理量有相同单位
      $$
      \sigma_{g}\left(k\right)\rightarrow\frac{rad}{s},\sigma_{a}\left(k\right)\rightarrow\frac{m}{s^{2}},\sigma_{bg}\left(k\right)\rightarrow\frac{rad}{s},\sigma_{ba}\left(k\right)\rightarrow\frac{m}{s^{2}}
      $$

    - 连续的他们需要在离散方差上乘以或除以一个开方时间单位
      $$
      \sigma_{g}\rightarrow\frac{rad}{\sqrt{s}},\sigma_{a}\rightarrow\frac{m}{s\sqrt{s}},\sigma_{bg}\rightarrow\frac{rad}{s\sqrt{s}},\sigma_{ba}\rightarrow\frac{m}{s^{2}\sqrt{s}}
      $$

## 2. 用IMU进行轨迹推算

在只有IMU数据的情况下也可以推断系统的运动状态，但只有IMU的系统需要对IMU读书进行二次积分，IMU的测量误差的零偏就会导致状态变量很快的偏移

### 2.1 利用IMU数据进行短时间航迹推算

由1.0.1可知imu所在系统的运动学模型，由1.2.1可知IMU加入噪声后的测量模型，将测量模型加入运动学模型并忽略噪声中的测量噪声只保留零偏后可得：
$$
\begin{aligned}
&\dot{R}=R\left(\tilde{\omega}-b_{g}\right)^{\wedge},或\dot{q}=q\left[0,\frac{1}{2}\left(\tilde{\omega}-b_{g}\right)\right], \\
&\dot{p}=v, \\
&\dot{v}=R\left(\tilde{a}-b_{a}\right)+g.
\end{aligned}\tag{2.1.1}
$$
2.1.1式可以从时间$t$积分到$t$+$\Delta t$：
$$
\begin{aligned}&R\left(t+\Delta t\right)=R\left(t\right)Exp\left(\left(\tilde{\omega}-b_{g}\right)\Delta t\right),或q\left(t+\Delta t\right)=q\left(t\right)\left[1,\frac{1}{2}\left(\tilde{\omega}-b_{g}\right)\Delta t\right],\\&p\left(t+\Delta t\right)=p\left(t\right)+v\Delta t+\frac{1}{2}\left(R\left(t\right)\left(\tilde{a}-b_{a}\right)\right)\Delta t^{2}+\frac{1}{2}g\Delta t^{2},\\&v\left(t+\Delta t\right)=v\left(t\right)+R\left(t\right)\left(\tilde{a}-b_{a}\right)\Delta t+g\Delta t.\end{aligned}\tag{2.1.2}
$$
2.1.2式其实就是积分中最简单的欧拉法，再继续累积从$i$时刻一直递推到$j$时刻，imu读数累积为：
$$
\begin{aligned}
\text{R}& =R_{i}\prod_{k=i}^{j-1}Exp\left(\left(\tilde{\omega}_{k}-b_{g,k}\right)\Delta t\right)或q_{j}=q_{i}\prod_{k=i}^{j-1}\left[1,\frac{1}{2}\left(\tilde{\omega}_{k}-b_{g,k}\right)\Delta t\right], \\
p_{j}& =p_{k}+\sum_{k=i}^{j-1}\left[v_{k}\Delta t+\frac{1}{2}g\Delta t^{2}\right]+\frac{1}{2}\sum_{k=i}^{j-1}R_{k}\left(\tilde{a}_{k}-b_{a,k}\right)\Delta t^{2}, \\
v_{j}& : =v_{i}+\sum_{k=i}^{j-1}\left[R_{k}\left(\tilde{a}_{k}-b_{a,k}\right)\Delta t+g\Delta t\right]. 
\end{aligned}\tag{2.1.3}
$$

### 2.2 案例：IMU递推

该案例使用IMU数据进行轨迹的推算，在没有其他观测数据时只能对式2.1.3进行二次积分来得到物体本身的位姿。可以发现这种积分很快就会发散，因此IMU不适合单独用来进行航迹推算。

对2.1.3式的二次积分的实现见2.2.2代码文件2

- 运行

  ```
  bin/run_imu_integration
  ```

- 运行绘图脚本绘制轨迹

  ```
  python scripts/plot_ch3_state.py data/ch3/state.txt
  ```

#### 2.2.1 代码文件1

run_imu_integration.cpp

imu数据读取，在回调函数中调用代码2(Imu数据的2次积分)

```c++
// ! 单纯依靠笔记公式2.1.3对IMU数据做2次积分，得到运动物体的位姿 
#include <glog/logging.h>   // 日志管理工具
#include <iomanip>  // c++标准库：主要用于格式化输入输出操作，例如设置宽度、精度、填充字符、调整字段

#include "ch3/imu_integration.h"
#include "common/io_utils.h"
#include "tools/ui/pangolin_window.h"

DEFINE_string(imu_txt_path, "./data/ch3/10.txt", "数据文件路径");
DEFINE_bool(with_ui, true, "是否显示图形界面");

/// 本程序演示如何对IMU进行直接积分
/// 该程序需要输入data/ch3/下的文本文件，同时它将状态输出到data/ch3/state.txt中，在UI中也可以观察到车辆运动
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_imu_txt_path.empty()) {
        return -1;
    }

    // TxtIO类读取了txt文件
    // * 该类使用std::function内置了针对imu,odom,gnss三种数据的回调函数
    // 后面想处理哪种数据，就把处理函数或lambda表达式传入这些回调函数即可
    sad::TxtIO io(FLAGS_imu_txt_path);

    // 该实验中，我们假设零偏已知
    Vec3d gravity(0, 0, -9.8);                                  // 重力方向
    Vec3d init_bg(00.000224886, -7.61038e-05, -0.000742259);    // 陀螺仪的邻偏 
    Vec3d init_ba(-0.165205, 0.0926887, 0.0058049);             // 加速度计的邻偏

    sad::IMUIntegration imu_integ(gravity, init_bg, init_ba);   // 用于对IMU做积分的类

    std::shared_ptr<sad::ui::PangolinWindow> ui = nullptr;      // 可视化
    if (FLAGS_with_ui) {
        ui = std::make_shared<sad::ui::PangolinWindow>();
        ui->Init();
    }

    /// 记录结果
    // * 定义一个lambda表达式(匿名函数)
    auto save_result = [](std::ofstream& fout, double timestamp, const Sophus::SO3d& R, const Vec3d& v, const Vec3d& p) {
        auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { 
            fout << v[0] << " " << v[1] << " " << v[2] << " "; 
        };
        auto save_quat = [](std::ofstream& fout, const Quatd& q) {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };

        fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
        save_vec3(fout, p);                         // IMU的位置
        save_quat(fout, R.unit_quaternion());		// IMU的旋转矩阵
        save_vec3(fout, v);							// IMU的速度
        fout << std::endl;
    };

    std::ofstream fout("./data/ch3/state.txt");
    // * 再定义一个lambda表达式(匿名函数)作为上面TxtIO::io的回调函数
    // 这个匿名函数的参数imu，在Go()中调用此回调函数时被赋予
    io.SetIMUProcessFunc([&imu_integ, &save_result, &fout, &ui](const sad::IMU& imu) {
          imu_integ.AddIMU(imu);
          save_result(fout, imu.timestamp_, imu_integ.GetR(), imu_integ.GetV(), imu_integ.GetP());
          if (ui) {
              ui->UpdateNavState(imu_integ.GetNavState());
              usleep(1e2);
          }
      }).Go();

    // 打开了可视化的话，等待界面退出
    while (ui && !ui->ShouldQuit()) {
        usleep(1e4);
    }

    if (ui) {
        ui->Quit();
    }

    return 0;
}
```

#### 2.2.2 代码文件2

imu_integration.h

实现Imu数据的2次积分

```c++
class IMUIntegration {
   public:
    IMUIntegration(const Vec3d& gravity, const Vec3d& init_bg, const Vec3d& init_ba)
        : gravity_(gravity), bg_(init_bg), ba_(init_ba) {}

    // 增加imu读数
    void AddIMU(const IMU& imu) {
        double dt = imu.timestamp_ - timestamp_;
        if (dt > 0 && dt < 0.1) {
            // ***** 对2.1.3式的二次积分的实现 *****
            // 假设IMU时间间隔在0至0.1以内
            p_ = p_ + v_ * dt + 0.5 * gravity_ * dt * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt;
            v_ = v_ + R_ * (imu.acce_ - ba_) * dt + gravity_ * dt;
            R_ = R_ * Sophus::SO3d::exp((imu.gyro_ - bg_) * dt);
        }

        // 更新时间戳
        timestamp_ = imu.timestamp_;
    }

    /// 组成NavState
    NavStated GetNavState() const { return NavStated(timestamp_, R_, p_, v_, bg_, ba_); }

    SO3 GetR() const { return R_; }
    Vec3d GetV() const { return v_; }
    Vec3d GetP() const { return p_; }

   private:
    // 累计量
    SO3 R_;
    Vec3d v_ = Vec3d::Zero();
    Vec3d p_ = Vec3d::Zero();

    double timestamp_ = 0.0;

    // 零偏，由外部设定
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();

    Vec3d gravity_ = Vec3d(0, 0, -9.8);  // 重力
};

```

## 3. 卫星导航

全球卫星导航系统(Global Navigation Satellite System,GNSS)，简称卫星导航，是室外车辆定位的另一个信息来源。GNSS实际上可以提供车辆所需的所有定位信息，包括位姿、速度等物理量。但由于GNSS精度、频率、天气和场地等影响，卫星导航在自动驾驶中通常处于一种精度够用但稳定性很差的状态。

**定位的原理是**：GNSS通过测量自身与卫星的距离来确定自身的位置，而与卫星的距离通过测量时间间隔来确定。

**卫星信号来源**有：美国的全球定位系统GPS，中国的北斗BDS，俄罗斯的格洛纳斯系统GLONASS，和欧洲的伽利略系统GALILEO

自动驾驶中最常用的卫星定位技术有：

1. 单点GNSS定位，即传统的米级精度卫星定位。便宜且足以让驾驶员辨认自己处于哪条路上。

2. RTK定位，(Real-Time Kinematic,实时动态差分)，有一个有精确位置的地面基站来校正，所以可以达到厘米级。

   RTK接收器通常安装在车辆顶部，1个接收器就可以提供精确卫星定位的位置，如果有2个根据两者位置差还可以计算车辆的实时朝向(heading).

### 3.1 世界坐标系

1. 地理坐标系：

   经纬度

2. UTM坐标系：

   将经度分为60个区，维度分为20个区并给予标号：经度为数字标号，纬度为字母标号。

   在每个分区中，UTM坐标以正东、正北的米制坐标来表达车辆的位置：

   北半球中有两种坐标系：

   - 东北天坐标系：正东为X轴，正北为Y轴，按照右手坐标系Z轴指向天空

   - 北东地坐标系：正北为X轴，正东为Y轴，Z指向地

### 3.2 案例：RTK读数的显示

#### 3.2.1 数据例子

```txt
ODOM 1624426287.19101906 0 0
GNSS 1624426287.22183037 39.8716278074999977 116.477817216999995 37.1386985778808594 6.2900999999999998 0
IMU 1624426287.22854877 0.000680678408277777028 -0.000532325421858261482 0.000656243798749856877 -0.605724081666666581 0.0254972899999999988 9.80681344416666789
```

- GNSS含义：
  - 时间戳、纬度、经度、高度、方向角、方向角有效标志
- IMU的含义：
  - 时间戳、陀螺仪、加速度计

#### 3.2.2 经纬度转utm坐标系

这里使用一个开源的库来完成,ros自己也提供了一个开源库。然后将将utm坐标从gnss坐标系转为车身坐标系：
$$
\mathbf{T}_{WB}=\mathbf{T}_{WG}\mathbf{T}_{GB}
$$
W:世界，B:车身，G:GNSS

其中GNSS的朝向$\mathbf{R}_{WG}$是因为有两个RTK才能确认的。如果是单天线方案，当车辆朝向不明时，无法确定车辆本体的世界坐标。

utm_convert.cc

```c++
#include "ch3/utm_convert.h"
#include "common/math_utils.h"
#include "utm_convert/utm.h"

#include <glog/logging.h>

namespace sad {

// ! 使用开源库utm_convert/utm.h将经纬度转为UTM坐标
bool LatLon2UTM(const Vec2d& latlon, UTMCoordinate& utm_coor) {
    long zone = 0;
    char char_north = 0;
    long ret = Convert_Geodetic_To_UTM(latlon[0] * math::kDEG2RAD, latlon[1] * math::kDEG2RAD, &zone, &char_north,
                                       &utm_coor.xy_[0], &utm_coor.xy_[1]);
    utm_coor.zone_ = (int)zone;
    utm_coor.north_ = char_north == 'N';

    return ret == 0;
}

// ! 使用开源库utm_convert/utm.h将UTM坐标转为经纬度
bool UTM2LatLon(const UTMCoordinate& utm_coor, Vec2d& latlon) {
    bool ret = Convert_UTM_To_Geodetic((long)utm_coor.zone_, utm_coor.north_ ? 'N' : 'S', utm_coor.xy_[0],
                                       utm_coor.xy_[1], &latlon[0], &latlon[1]);
    latlon *= math::kRAD2DEG;
    return ret == 0;
}

// ! 将utm坐标从gnss坐标系转为车身坐标系
bool ConvertGps2UTM(GNSS& gps_msg, const Vec2d& antenna_pos, const double& antenna_angle, const Vec3d& map_origin) {
    // * 1. 经纬高转换为UTM
    UTMCoordinate utm_rtk;
    if (!LatLon2UTM(gps_msg.lat_lon_alt_.head<2>(), utm_rtk)) {
        return false;
    }
    utm_rtk.z_ = gps_msg.lat_lon_alt_[2];

    // * 2. GPS heading朝向 转成弧度
    // math::kDEG2RAD = pi/180
    // 在这里方向角heading_valid_是因为双天线方案所以才可以得到的
    // 我们使用东北天坐标系的utm，但该rtk厂商输出的是北东地坐标系，两者旋转方向相反，因此需要将朝向角转换一下。
    double heading = 0;
    if (gps_msg.heading_valid_) {
        heading = (90 - gps_msg.heading_) * math::kDEG2RAD;  // 北东地转到东北天
    }

    // * 3. 得到gnss相对于车身的坐标TBG
    // 这里的安装偏角antenna_angle 和 安装偏移antenna_pos 都是依靠装gnss时标定得到的
    SE3 TBG(SO3::rotZ(antenna_angle * math::kDEG2RAD), Vec3d(antenna_pos[0], antenna_pos[1], 0));
    SE3 TGB = TBG.inverse();

    // * 4. 由gnss的utm坐标确定车身本体的世界坐标TWB
    /// 若指明地图原点，则减去地图原点
    double x = utm_rtk.xy_[0] - map_origin[0];
    double y = utm_rtk.xy_[1] - map_origin[1];
    double z = utm_rtk.z_ - map_origin[2];
    SE3 TWG(SO3::rotZ(heading), Vec3d(x, y, z));    // gnss的世界坐标是根据 朝向 + utm 得到的
    SE3 TWB = TWG * TGB;

    gps_msg.utm_valid_ = true;
    gps_msg.utm_.xy_[0] = TWB.translation().x();
    gps_msg.utm_.xy_[1] = TWB.translation().y();
    gps_msg.utm_.z_ = TWB.translation().z();

    if (gps_msg.heading_valid_) {
        // 组装为带旋转的位姿
        gps_msg.utm_pose_ = TWB;
    } else {
        // 组装为仅有平移的SE3
        // 注意当安装偏移存在时，并不能实际推出车辆位姿
        gps_msg.utm_pose_ = SE3(SO3(), TWB.translation());
    }

    return true;
}

bool ConvertGps2UTMOnlyTrans(GNSS& gps_msg) {
    /// 经纬高转换为UTM
    UTMCoordinate utm_rtk;
    LatLon2UTM(gps_msg.lat_lon_alt_.head<2>(), utm_rtk);
    gps_msg.utm_valid_ = true;
    gps_msg.utm_.xy_ = utm_rtk.xy_;
    gps_msg.utm_.z_ = gps_msg.lat_lon_alt_[2];
    gps_msg.utm_pose_ = SE3(SO3(), Vec3d(gps_msg.utm_.xy_[0], gps_msg.utm_.xy_[1], gps_msg.utm_.z_));
    return true;
}

}  // namespace sad
```

#### 3.2.3 绘制整个GNSS轨迹

process_gnss.cc

```c++
#include <glog/logging.h>
#include <iomanip>
#include <memory>

#include "common/gnss.h"
#include "common/io_utils.h"
#include "tools/ui/pangolin_window.h"
#include "utm_convert.h"

DEFINE_string(txt_path, "./data/ch3/10.txt", "数据文件路径");

// 以下参数仅针对本书提供的数据
DEFINE_double(antenna_angle, 12.06, "RTK天线安装偏角（角度）");
DEFINE_double(antenna_pox_x, -0.17, "RTK天线安装偏移X");
DEFINE_double(antenna_pox_y, -0.20, "RTK天线安装偏移Y");
DEFINE_bool(with_ui, true, "是否显示图形界面");

/**
 * 本程序演示如何处理GNSS数据
 * 我们将GNSS原始读数处理成能够进行后续处理的6自由度Pose
 * 需要处理UTM转换、RTK天线外参、坐标系转换三个步骤
 *
 * 我们将结果保存在文件中，然后用python脚本进行可视化
 */

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    // * 1. 使用gflags从命令行参数中读取gnss原始数据 
    if (fLS::FLAGS_txt_path.empty()) {
        return -1;
    }

    sad::TxtIO io(fLS::FLAGS_txt_path);

    std::ofstream fout("./data/ch3/gnss_output.txt");
    Vec2d antenna_pos(FLAGS_antenna_pox_x, FLAGS_antenna_pox_y);

    // * 2. 定义匿名函数作为后面的回调函数，用于将从gnss算出的车身位姿输出到文件中去
    auto save_result = [](std::ofstream& fout, double timestamp, const SE3& pose) {
        auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
        auto save_quat = [](std::ofstream& fout, const Quatd& q) {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };

        fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
        save_vec3(fout, pose.translation());
        save_quat(fout, pose.unit_quaternion());
        fout << std::endl;
    };

    // * 3. 可视化窗口
    std::shared_ptr<sad::ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<sad::ui::PangolinWindow>();
        ui->Init();
    }

    // * 4. 使用TxtIO读去gnss原始数据，并用下面定义的匿名函数对其进行处理
    bool first_gnss_set = false;
    Vec3d origin = Vec3d::Zero();
    io.SetGNSSProcessFunc([&](const sad::GNSS& gnss) {
          sad::GNSS gnss_out = gnss;
          // 4.1 经纬度转utm，并由此得到车身的世界坐标
          if (sad::ConvertGps2UTM(gnss_out, antenna_pos, FLAGS_antenna_angle)) {
              if (!first_gnss_set) {
                  origin = gnss_out.utm_pose_.translation();
                  first_gnss_set = true;
              }

              // 4.2 减掉一个原点
              gnss_out.utm_pose_.translation() -= origin;
              // 4.3 调用上面定义的匿名函数来保存数据
              save_result(fout, gnss_out.unix_time_, gnss_out.utm_pose_);

              if (ui) {
                  ui->UpdateNavState(
                      sad::NavStated(gnss_out.unix_time_, gnss_out.utm_pose_.so3(), gnss_out.utm_pose_.translation()));
                  usleep(1e3);
              }
          }
      }).Go();

    if (ui) {
        while (!ui->ShouldQuit()) {
            usleep(1e5);
        }
        ui->Quit();
    }

    return 0;
}
```

#### 3.2.4 运行

1. 从GNSS原始读数得到世界坐标系下的车身位姿

   ```
   ./bin/process_gnss --txt_path ./data/ch3/10.txt
   ```

2. 绘制二维和三维轨迹

   ```
   python scripts/plot_ch3_gnss_3d.py ./data/ch3/gnss_output.txt
   python scripts/plot_ch3_gnss_2d.py ./data/ch3/gnss_output.txt
   ```

## 4. 使用误差状态卡尔曼滤波器实现组合导航

现在结合RTK和IMU提供的数据，使用误差状态卡尔曼滤波器(Error State Kalman Filter, ESKF)来实现传统的组合导航算法。

### 4.0 什么是ESKF

- 相比传统KF,ESKF的优点：

  1. 在旋转的处理上，ESKF 的状态变量可以采用最小化的参数表达，也就是使用三维变量来表达旋转的增量。该变量位于切空间中，而切空间是一个向量空间。传统 KF 需要用到四元数(4 维)或者更高维的变量来表达状态(旋转矩阵，9 维)，要不就得采用带有奇异性的表达方式(欧拉角)。

  2. ESKF 总是在原点附近，离奇异点较远，数值方面更稳定，并且不会产生离工作点太远而
     导致线性化近似不够的问题。

  2. ESKF 的状态量为小量，其二阶变量相对来说可以忽略。同时，大多数雅可比矩阵在小量
      情况下变得非常简单，甚至可以用单位阵代替。
  3. 误差状态的运动学相比原状态变量更小(小量的运动学),因此可以把更新部分归人原状
      态变量中。

- ESKF中的变量名：

  - 名义状态变量(Nominal State)：原状态变量
  - 误差状态变量(Error State)：ESKF里的状态变量
  - 真值：名义状态变量和误差状态变量之和

  噪声的处理时放在误差状态变量中的，因此名义状态变量可以认为是没有噪声的。

- ESKF的整体流程：

  1. 将IMU测量数据积分后，放入名义状态变量。
     - 名义状态在运动过程中会随着IMU数据而进行递推。
  2. 由于没有考虑噪声，所以很快就会漂移，将此时的误差部分作为误差状态变量。
     - 误差状态在运动过程中会受到高斯噪声影响而变大。
     - ESKF的误差状态均值和协方差会描述误差状态扩大的具体数值(视为高斯分布)。
  3. 在更新过程中利用从传感器数据(RTK)更新误差状态的后验均值和协方差。
  4. 将这部分误差合入名义状态变量中得到真值
  5. 将ESKF置零，完成一次预测。

### 4.1 连续时间的ESKF运动方程

设ESKF的真值状态为$x_t(t)=[p_t,v_t,R_t,b_{at}.b_{gt},g_t]^T$,下标t表示真值true。IMU的读数为$\tilde{w},\tilde{a}$。

- 状态变量真值的导数相对于观测量之间的导数

  将1.2.1的测量值代入1.0.1的运动模型，再加上1.2.4的噪声模型可得：
  $$
  \begin{aligned}
  &\dot{p}_t&& =v_{t}, \\
  &\dot{v}_{t}&& =R_{t}\left(\tilde{a}-b_{at}-\eta_{a}\right)+g_{t}, \\
  &\dot{R}_{t}&& =R_{t}\left(\tilde{\omega}-b_{gt}-\eta_{g}\right)^{\wedge}, \\
  &\dot{b}_{gt}&& =\eta_{bg}, \\
  &\dot{b}_{at}&& =\eta_{ba}, \\
  &\dot{g}_t&& =0. 
  \end{aligned}\tag{4.1.1}
  $$

- 定义误差状态变量：
  $$
  \begin{aligned}
  &p_{t}&& =p+\delta p, \\
  &v_{t}&& =v+\delta v, \\
  &R_{t}&& =R\delta R或q_{t}=q\delta q, \\
  &b_{gt}&& =b_{g}+\delta b_{g}, \\
  &b_{at}&& =b_{a}+\delta b_{a}, \\
  &g_{t}&& =g+\delta g. 
  \end{aligned}\tag{4.1.2}
  $$
  第一列即名义状态变量，第二列即误差状态变量。

- **名义状态变量的运动方程**：

  即4.1.1式不考虑噪声，因为噪声再误差状态方程中考虑。

- **误差状态变量的运动方程**：

  对误差项求导可得：(推导见书P75-77)
  $$
  \begin{aligned}
  \delta\dot{p}& =\delta v, \\
  \delta\dot{v}& =-R\left(\tilde{a}-b_{a}\right)^{\wedge}\delta\theta-R\delta b_{a}-\eta_{a}+\delta g, \\
  \delta\dot{\theta}& =-\left(\tilde{\omega}-b_{g}\right)^{\wedge}\delta\theta-\delta b_{g}-\eta_{g}, \\
  \delta\dot{b}_{g}& =\eta_{bg}, \\
  \delta\dot{b}_{a}& =\eta_{ba}, \\
  \delta\dot{g}& =0. 
  \end{aligned}\tag{4.1.3}
  $$

### 4.2 离散时间的ESKF运动方程

从4.1.1和4.1.3的连续形式分别推出下面的离散形式

- 名义状态变量的离散时间运动方程：
  $$
  \begin{aligned}
  p\left(t+\Delta t\right)& =p\left(t\right)+v\Delta t+\frac{1}{2}\left(R\left(\tilde{a}-b_{a}\right)\right)\Delta t^{2}+\frac{1}{2}g\Delta t^{2} \\
  v\left(t+\Delta t\right)& =v\left(t\right)+R\left(\tilde{a}-b_{a}\right)\Delta t+g\Delta t, \\
  R\left(t+\Delta t\right)& =R\left(t\right)Exp\left(\left(\tilde{\omega}-b_{g}\right)\Delta t\right), \\
  b_{g}\left(t+\Delta t\right)& =b_{g}\left(t\right), \\
  b_{a}\left(t+\Delta t\right)& =b_{a}\left(t\right), \\
  g\left(t+\Delta t\right)& =g\left(t\right). 
  \end{aligned}\tag{4.2.1}
  $$

- 误差状态变量的离散时间运动方程：
  $$
  \begin{aligned}
  &\delta p\left(t+\Delta t\right)&& =\delta p+\delta v\Delta t, \\
  &\delta v\left(t+\Delta t\right)&& =\delta v+\left(-R\left(\tilde{a}-b_{a}\right)^{\wedge}\delta\theta-R\delta b_{a}+\delta g\right)\Delta t-\eta_{v}, \\
  &\delta\theta\left(t+\Delta t\right)&& =Exp\left(-\left(\tilde{\omega}-b_{g}\right)\Delta t\right)\delta\theta-\delta b_{g}\Delta t-\eta_{\theta}, \\
  &\delta b_{g}\left(t+\Delta t\right)&& =\delta b_{g}+\eta_{g}, \\
  &\delta b_{a}\left(t+\Delta t\right)&& =\delta b_{a}+\eta_{a}, \\
  &\delta g\left(t+\Delta t\right)&& =\delta g 
  \end{aligned}\tag{4.2.2}
  $$

  > 噪声项不参与递推，他们被单独的归入噪声部分。
  >
  > 连续时间下，噪声可以视为随机过程的能量谱密度。
  >
  > 离散时间下，噪声就是随机变量，可以写为：
  > $$
  > \sigma\left(\eta_{v}\right)=\Delta t\sigma_{a}\left(k\right),\quad\sigma\left(\eta_{\theta}\right)=\Delta t\sigma_{g}\left(k\right),\quad\sigma\left(\eta_{g}\right)=\sqrt{\Delta t}\sigma_{bg},\quad\sigma\left(\eta_{a}\right)=\sqrt{\Delta t}\sigma_{ba}
  > $$

**4.2.1和4.2.2即ESKF中进行IMU递推的过程**，相当于KF的状态方程(见一、4.1.3式)。

为了让ESKF滤波器收敛就需要外部的观测数据对KF进行修正，即所谓的组合导航。组合导航的方法有很多如EKF,ESKF,与积分和图优化。**下面是融合GNSS的观测值并使用ESKF来形成一个收敛的KF。**

### 4.3 ESKF的运动(预测)过程

4.2.2式的误差状态变量的离散时间运动方程可以整体记为：
$$
\delta x_{k+1}=f(\delta x_k)+w,w\sim\mathcal{N}(0,Q)\tag{4.3.1}
$$
其中w为噪声，Q为：
$$
Q=diag\left(0_{3},Cov\left(\eta_{v}\right),Cov\left(\eta_{\theta}\right),Cov\left(\eta_{g}\right),Cov\left(\eta_{a}\right),0_{3}\right)\tag{4.3.2}
$$
4.3.2式两端为0是因为4.2.2式的第一和最后一个方程本身没有噪声。

计算误差状态变量的运动方程的线性化形式为：
$$
\delta x\left(t+\Delta t\right)=\underbrace{f\left(\delta x\left(t\right)\right)}_{=0}+F\delta x+w\tag{4.3.3}
$$
F是线性化后的雅可比矩阵：
$$
F=\begin{bmatrix}I&I\Delta t&0&0&0&0\\0&I&-R(\tilde{a}-b_{\mathrm{a}})^{\wedge}\Delta t&0&-R\Delta t&I\Delta t\\0&0&Exp\left(-(\tilde{\omega}-b_{\mathrm{g}})\Delta t\right)&-I\Delta t&0&0\\0&0&0&I&0&0\\0&0&0&0&I&0\\0&0&0&0&0&I\end{bmatrix}\tag{4.3.4}
$$
再4.3.3的基础上执行ESKF的**预测过程**：
$$
\begin{aligned}\delta x_{pred}&=F\delta x,\\P_{pred}&=FPF^{\top}+Q\end{aligned}\tag{4.3.5}
$$

- 误差状态再每次更新后都会被重置$\delta x=0$，所以4.3.5式的第一个方程没什么意义。
- 4.3.5式的第二个方程，协方差，则描述了整个误差估计中的分布情况。

### 4.4 ESKF的更新过程

- 假设一个传感器能对状态变量产生观测，其观测方程为:
  $$
  z=h(x)+v,v\sim\mathcal{N}(0,V)\tag{4.4.1}
  $$

  - $z$:观测数据
  - $h$:观测方程
  - $v$:观测噪声
  - $V$:该噪声的协方差矩阵

- ESKF拥有名义状态$x$的估计及误差状态$\delta x$的估计，需要更新的式误差状态，

  1. 首先计算观测方程对误差状态的雅可比矩阵：
     $$
     H=\frac{\partial h}{\part\delta x}|_{x_{pred}}\tag{4.4.2}
     $$

  2. 计算卡尔马增益K，进而计算误差状态的更新过程
     $$
     \begin{aligned}
     &\text{K} =P_{pred}H^{T}\left(HP_{pred}H^{T}+V\right)^{-1}, \\
     &\delta x=K\left(z-h\left(x_{pred}\right)\right), \\
     &x=x_{pred}+\delta x, \\
     &P=\left(I-KH\right)P_{pred} 
     \end{aligned}\tag{4.4.3}
     $$

     - $K$:卡尔曼增益
     - $P_{pred}$：预测的协方差矩阵
     - $P$：修正后的协方差矩阵

- 大部分观测数据是对名义状态的观测，此时4.4.2式的雅可比矩阵$H$需要通过链式法则来计算
  $$
  H=\frac{\partial h}{\partial x}\frac{\partial x}{\partial\delta x}\tag{4.4.4}
  $$

  - 第一项只需要对观测方程进行线性化

  - 第二项计算为：
    $$
    \frac{\partial x}{\partial\delta x}=diag\left(I_{3},I_{3},\frac{\partial Log\left(R\left(Exp\left(\delta\theta\right)\right)\right)}{\partial\delta\theta},I_{3},I_{3},I_{3}\right)\tag{4.4.5}
    $$

    - 旋转部分计算用右乘BCH：
      $$
      \frac{\partial Log\left(R\left(Exp\left(\delta\theta\right)\right)\right)}{\partial\delta\theta}=J_{r}^{-1}\left(R\right)\tag{4.4.6}
      $$
      

### 4.5 ESKF的误差状态后续处理

在4.3和4.4对误差状态的估计修正后，就需要把误差状态归入名义状态得到真值，然后重置ESKF。

- 归入公式即：
  $$
  \begin{align}
  p_{k+1}&=p_{k}+\delta p_{k},\\
  v_{k+1}&=v_{k}+\delta v_{k},\\
  R_{k+1}&=R_{k}Exp\left(\delta\theta_{k}\right),\\
  b_{g,k+1}&=b_{g,k}+\delta b_{g,k},\\
  b_{a,k+1}&=b_{a,k}+\delta b_{a,k},\\
  g_{k+1}&=g_k+\delta g_k
  \end{align}\tag{4.4.7}
  $$

- 重置ESKF分为2个部分：

  1. 均值：

     对任意变量$x$
     $$
     \delta x=0\tag{4.4.8}
     $$

  2. 协方差：
     $$
     P_{reset}=J_kPJ_k^T\tag{4.4.9}
     $$

     - $J_k=diag(I_3,I_3,J_{\theta}.I_3,I_3,I_3)$
     - 对旋转的雅可比矩阵$J_\theta=I-\frac{1}{2}\delta\theta_k^{\wedge}$

## 5. 案例：实现ESKF的组合导航

实现了一个融合IMU和GNSS观测的ESKF



# 四、预积分学

- 第三章已解决：

  将2次GNSS观测之间的IMU数据进行积分，作为ESKF的预测过程。此时IMU数据是一次性的被积分到当前估计值上，然后用观测数据更新当时的估计值。

- 想解决的问题：

  希望重复利用IMU数据，即使状态变量发生改变，也希望IMU数据的计算和当时的状态估计无关。

- 新方法：预积分学

  将一段时间内IMU测量数据累积，建立与积分测量，同时保证测量值与状态变量无关。

## 1. IMU状态的预积分学

## 2. 案例：实现预积分



# 五、基础点云处理

## 1. 激光雷达传感器与点云的数学模型

### 1.1 激光雷达传感器的数学模型

- 两种激光雷达

  1. 机械旋转式激光雷达

     以固定频率旋转的激光探头。根据线数进一步划分(单线，4线，8线...)，线数越高每次扫描得到的点数越多，信息也越丰富。

     可以360°扫描对定位和建图有利，但价格昂贵。

  2. 固态激光雷达

     原理和RGBD相机相似。相比机械旋转式，激光雷达事业范围狭窄但价格更便宜。

     可以用多个激光雷达组成360°扫描。
  
- 单点测量的RAE模型

  ![image-20240510005625801](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/image-20240510005625801.png)

  - 如图所示：

    - 距离：r=Range
    - 方位角：A=Azimuth
    - 俯仰角：E=Elevation

  - 如此得到P在雷达参考系下的坐标为：
    $$
    P=[rcosEcosA,rcosEsinA,rsinE]^T\tag{1.1.1}
    $$

- 除了记录距离，多线激光雷达还可以记录每个点的时刻、反射率、该点属于哪线等信息。

### 1.2 案例：点云的表达方式

点云是最基本的三维结构表达方式，也是多数激光雷达向外输出的数据形式。他们是一组欧氏空间中的笛卡尔坐标。

数组可以用来表示最基本的点云，但通常点云还会携带反射率、所属线束、RGB颜色等信息，因此最常用的是定义一个点云结构体。

可视化点云可以用：pcl_viewer，pcl库代码实现，cloudcompare软件

#### 1.2.1 pcl_viewer可视化点云

1. 下载pcl工具库

   ```
   sudo apt-get install  pcl-tools
   ```

2. 可视化

   ```
   pcl_viewer ./data/xxx.pcd
   ```

#### 1.2.2 pcl库可视化点云

```c++
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

DEFINE_string(pcd_path, "./data/ch5/map_example.pcd", "点云文件路径");

/// 本程序可用于显示单个点云，演示PCL的基本用法
/// 实际上就是调用了pcl的可视化库，类似于pcl_viewer
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_pcd_path.empty()) {
        LOG(ERROR) << "pcd path is empty";
        return -1;
    }

    // 读取点云
    PointCloudType::Ptr cloud(new PointCloudType);
    pcl::io::loadPCDFile(FLAGS_pcd_path, *cloud);

    if (cloud->empty()) {
        LOG(ERROR) << "cannot load cloud file";
        return -1;
    }

    LOG(INFO) << "cloud points: " << cloud->size();

    // visualize
    pcl::visualization::PCLVisualizer viewer("cloud viewer");
    pcl::visualization::PointCloudColorHandlerGenericField<PointType> handle(cloud, "z");  // 使用高度来着色
    viewer.addPointCloud<PointType>(cloud, handle);
    viewer.spin();

    return 0;
}
```

### 1.3 Packet数据包的表达

数据包是雷达向外输出的另一种数据形式，为了节省雷达与计算机数据通信量。

数据被分为2部分：

- 雷达内参数：

  雷达旋转率、各探头相对雷达中心的俯仰角等在雷达设计时已知的参数。会放再固定的参数文件中

- 真正测量的数据：

  只记录距离和反射率。

每个雷达厂商的packet格式都不一样，packet格式是一种压缩后的数据，算法中还是要先将他们转化为可以直接读坐标的格式。

### 1.4 案例：地图表达方式BEV和range image

#### 1.4.1 俯视图(鸟瞰图，Bird-eye View)

- 在室外地图中非常常见，希望用栅格地图和2D标注的话就需要用到BEV, 但3D转2D显然丢弃了点云的高度信息

- 下面的代码用OpenCV实现将.PCD的点云坐标转换为图像坐标。其中坐标为x,y,z的点云落在图像的u,v处，他们应该满足：
  $$
  \begin{cases}
  u=(x-c_x)/r+I_x \\
  v=(y-c_y)/r+I_y
  \end{cases}\tag{1.4.1.1}
  $$

  - $r$：分辨率，确定每个像素对应多少米的距离
  - $c_x,c_y$：点云的中心
  - $I_x,I_y$：图像的中心

- 代码实现：

  ```c++
  #include <gflags/gflags.h>
  #include <glog/logging.h>
  
  #include <pcl/io/pcd_io.h>
  #include <pcl/point_cloud.h>
  #include <pcl/point_types.h>
  
  #include <opencv2/core/core.hpp>
  #include <opencv2/highgui/highgui.hpp>
  
  using PointType = pcl::PointXYZI;
  using PointCloudType = pcl::PointCloud<PointType>;
  
  DEFINE_string(pcd_path, "./data/ch5/map_example.pcd", "点云文件路径");
  DEFINE_double(image_resolution, 0.1, "俯视图分辨率");
  DEFINE_double(min_z, 0.2, "俯视图最低高度");
  DEFINE_double(max_z, 2.5, "俯视图最高高度");
  
  /// 本节演示如何将一个点云转换为俯视图像
  void GenerateBEVImage(PointCloudType::Ptr cloud) {
      // ********* 1. 计算点云边界 *********
      auto minmax_x = std::minmax_element(cloud->points.begin(), cloud->points.end(),
                                          [](const PointType& p1, const PointType& p2) { return p1.x < p2.x; });
      auto minmax_y = std::minmax_element(cloud->points.begin(), cloud->points.end(),
                                          [](const PointType& p1, const PointType& p2) { return p1.y < p2.y; });
      double min_x = minmax_x.first->x;
      double max_x = minmax_x.second->x;
      double min_y = minmax_y.first->y;
      double max_y = minmax_y.second->y;
  	
      // FLAGS_image_resolution是自定义的俯视图分辨率  
      const double inv_r = 1.0 / FLAGS_image_resolution;
  
      const int image_rows = int((max_y - min_y) * inv_r);
      const int image_cols = int((max_x - min_x) * inv_r);
  	
      // 点云中心
      float x_center = 0.5 * (max_x + min_x);
      float y_center = 0.5 * (max_y + min_y);
      // 图像中心
      float x_center_image = image_cols / 2;
      float y_center_image = image_rows / 2;
  
      // ********* 2. 生成图像 *********
      cv::Mat image(image_rows, image_cols, CV_8UC3, cv::Scalar(255, 255, 255));
  	// 根据1.4.1.1式计算每一个点云在图像上的坐标
      for (const auto& pt : cloud->points) {
          int x = int((pt.x - x_center) * inv_r + x_center_image);
          int y = int((pt.y - y_center) * inv_r + y_center_image);
          // 认为高度在min_z=0.2到max_z=2.5米(更具车辆高度定)范围内的障碍物是有效的
          if (x < 0 || x >= image_cols || y < 0 || y >= image_rows || pt.z < FLAGS_min_z || pt.z > FLAGS_max_z) {
              continue;
          }
  
          image.at<cv::Vec3b>(y, x) = cv::Vec3b(227, 143, 79);
      }
  
      cv::imwrite("./bev.png", image);
  }
  
  int main(int argc, char** argv) {
      google::InitGoogleLogging(argv[0]);
      FLAGS_stderrthreshold = google::INFO;
      FLAGS_colorlogtostderr = true;
      google::ParseCommandLineFlags(&argc, &argv, true);
  
      if (FLAGS_pcd_path.empty()) {
          LOG(ERROR) << "pcd path is empty";
          return -1;
      }
  
      // 读取点云
      PointCloudType::Ptr cloud(new PointCloudType);
      pcl::io::loadPCDFile(FLAGS_pcd_path, *cloud);
  
      if (cloud->empty()) {
          LOG(ERROR) << "cannot load cloud file";
          return -1;
      }
  
      LOG(INFO) << "cloud points: " << cloud->size();
      GenerateBEVImage(cloud);
  
      return 0;
  }
  ```

#### 1.4.2 距离图(Range Image)

- Range Image和RGBD相机的思路一致：

  - RGBD：为了保障深度图和彩色图的一致性，会把点云按照彩色图像的参数投影到彩色相机中。
  - Range Image：将激光点云投影到某个虚拟的相机中去，有两种投影方式：
    1. 取图像的横坐标为激光雷达的方位角，纵坐标取俯仰角。
    2. 雷达每根线对应的俯仰角为横坐标，线数为纵坐标。

- 代码实现：

  ```c++
  #include <gflags/gflags.h>
  #include <glog/logging.h>
  
  #include <pcl/io/pcd_io.h>
  #include <pcl/point_cloud.h>
  #include <pcl/point_types.h>
  
  #include <opencv2/opencv.hpp>
  
  using PointType = pcl::PointXYZI;
  using PointCloudType = pcl::PointCloud<PointType>;
  
  DEFINE_string(pcd_path, "./data/ch5/scan_example.pcd", "点云文件路径");
  DEFINE_double(azimuth_resolution_deg, 0.3, "方位角分辨率（度）");
  DEFINE_int32(elevation_rows, 16, "俯仰角对应的行数");
  DEFINE_double(elevation_range, 15.0, "俯仰角范围");  // VLP-16 上下各15度范围
  DEFINE_double(lidar_height, 1.128, "雷达安装高度");
  
  void GenerateRangeImage(PointCloudType::Ptr cloud) {
      // ******** 1. 设置要生成的距离图尺寸 ********
      // 列数：水平为360度，按分辨率切分即可
      int image_cols = int(360 / FLAGS_azimuth_resolution_deg); 
      // 行数：俯仰角对应的行数为雷达的线数
      int image_rows = FLAGS_elevation_rows;                     
      LOG(INFO) << "range image: " << image_rows << "x" << image_cols;
  
      // 我们生成一个HSV图像以更好地显示图像
      cv::Mat image(image_rows, image_cols, CV_8UC3, cv::Scalar(0, 0, 0));
  	// elevation分辨率：反映了数据集中的每一个高程值所代表的地理面积的大小。
      double ele_resolution = FLAGS_elevation_range * 2 / FLAGS_elevation_rows;  
  	
      // ******** 2. 生成距离图 ********
      for (const auto& pt : cloud->points) {
          // 计算方位角（azimuth）：点在xy平面上的投影与正x轴之间的夹角。
          double azimuth = atan2(pt.y, pt.x) * 180 / M_PI;
          // 计算水平距离（range）
          double range = sqrt(pt.x * pt.x + pt.y * pt.y);
          // 计算仰角（elevation）：点相对于xy平面的高度
          double elevation = asin((pt.z - FLAGS_lidar_height) / range) * 180 / M_PI;
  
          // 将方位角调整为 0 到 360 度范围
          if (azimuth < 0) {
              azimuth += 360;
          }
  		// 计算图像的列坐标（x），对应方位角
          int x = int(azimuth / FLAGS_azimuth_resolution_deg);       
          // 计算图像的行坐标（y），对应仰角
          int y = int((elevation + FLAGS_elevation_range) / ele_resolution + 0.5);
  		 // 检查坐标是否在图像范围内
          if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
              // 在图像上绘制点，颜色由距离决定
              image.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(range / 100 * 255.0), 255, 127);
          }
      }
  
      // 沿Y轴翻转，因为我们希望Z轴朝上时Y朝上
      cv::Mat image_flipped;
      cv::flip(image, image_flipped, 0);
  
      // hsv to rgb
      cv::Mat image_rgb;
      cv::cvtColor(image_flipped, image_rgb, cv::COLOR_HSV2BGR);
      cv::imwrite("./range_image.png", image_rgb);
  }
  
  int main(int argc, char** argv) {
      google::InitGoogleLogging(argv[0]);
      FLAGS_stderrthreshold = google::INFO;
      FLAGS_colorlogtostderr = true;
      google::ParseCommandLineFlags(&argc, &argv, true);
  
      if (FLAGS_pcd_path.empty()) {
          LOG(ERROR) << "pcd path is empty";
          return -1;
      }
  
      // 读取点云
      PointCloudType::Ptr cloud(new PointCloudType);
      pcl::io::loadPCDFile(FLAGS_pcd_path, *cloud);
  
      if (cloud->empty()) {
          LOG(ERROR) << "cannot load cloud file";
          return -1;
      }
  
      LOG(INFO) << "cloud points: " << cloud->size();
      GenerateRangeImage(cloud);
  
      return 0;
  }
  ```

  

## 2. 最近邻问题

最近邻问题是许多点云问题的基本问题，问题描述为：

在一个含n个点的点云$\mathcal{X}=\{x_1,\cdots,x_n\}$中，我们想要知道

1. 离$x_m$最近的点是哪一个？
2. 离它最近的k个点又有哪些？ - > k近邻查找(kNN)
3. 与它的距离小于固定范围r的点有哪些？ - > 范围查找 range search

### 2.1 暴力最近邻法

Brute-force Nearest Neighbour Search暴力最近邻法是最简单的不用任何辅助的数据结构。

- 用暴力最近邻搜索**一个点**的思路：

  给定点云$\mathcal{X}$和待查找点$x_m$，计算$x_m$和 $\mathcal{X}$中每个点的距离，并给出最小距离

- 用暴力最近邻搜索**k个点**的思路

  1. 对给定点云$\mathcal{X}$和查找点$x_m$，计算$x_m$对所有$\mathcal{X}$点的距离
  2. 对第一步结果排序
  3. 选择k个最近的点

- 性能分析：

  - 时间复杂度

    - 如果$x_m$只有一个点，显然复杂度为$O(n)$

    - 如果$x_m$也有n个点，复杂度就成了$O(n^2)$

  - 但由于计算方式简单，不依赖任何复杂的数据结构，所以非常容易并行化。

    而且工程问题中，通常不需要搜索整个目标点云$\mathcal{X}$，可以在预先指定的小范围内搜索，所以工程中也很常用。

  - 而且由于暴力匹配好处是每两个点都会计算匹配，所以结果一定是正确的，后续其他方法就难以保证这一点。


### 2.1+ 案例：暴力最近邻法的代码实现

#### 2.1.1 对比单线程/多线程BFNN

运行结果如下：

```shell
➜  LiDAR-SLAM-code-comments git:(main) ./bin/test_nn --gtest_filter=CH5_TEST.BFNN     
Note: Google Test filter = CH5_TEST.BFNN
[==========] Running 1 test from 1 test suite.
[----------] Global test environment set-up.
[----------] 1 test from CH5_TEST
[ RUN      ] CH5_TEST.BFNN
Failed to find match for field 'intensity'.
Failed to find match for field 'intensity'.
I0618 00:36:58.099083 12864 test_nn.cc:37] points: 18869, 18779
I0618 00:37:07.842880 12864 sys_utils.h:32] 方法 暴力匹配（单线程） 平均调用时间/次数: 1948.73/5 毫秒.
I0618 00:37:08.768455 12864 sys_utils.h:32] 方法 暴力匹配（多线程） 平均调用时间/次数: 185.104/5 毫秒.
[       OK ] CH5_TEST.BFNN (10677 ms)
[----------] 1 test from CH5_TEST (10677 ms total)

[----------] Global test environment tear-down
[==========] 1 test from 1 test suite ran. (10677 ms total)
[  PASSED  ] 1 test.
➜  LiDAR-SLAM-code-comm
```

可以发现对于18000点数的点云，多线程只需要0.185秒，相比单线程的1.948秒快了很多。

#### 2.1.2 GTest测试代码

测试后面要介绍的各种最近邻方法来对比

```
./bin/test_nn --gtest_filter=CH5_TEST.BFNN
```

```c++
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

#include "ch5/bfnn.h"
#include "ch5/gridnn.hpp"
#include "ch5/kdtree.h"
#include "ch5/octo_tree.h"
#include "common/point_cloud_utils.h"
#include "common/point_types.h"
#include "common/sys_utils.h"

DEFINE_string(first_scan_path, "./data/ch5/first.pcd", "第一个点云路径");
DEFINE_string(second_scan_path, "./data/ch5/second.pcd", "第二个点云路径");
DEFINE_double(ANN_alpha, 1.0, "AAN的比例因子");

// 暴力近邻的测试案例
TEST(CH5_TEST, BFNN) {
    sad::CloudPtr first(new sad::PointCloudType), second(new sad::PointCloudType);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty()) {
        LOG(ERROR) << "cannot load cloud";
        FAIL();
    }

    // 降采样：体素栅格滤波
    // voxel grid 至 0.05
    sad::VoxelGrid(first);
    sad::VoxelGrid(second);

    LOG(INFO) << "points: " << first->size() << ", " << second->size();

    // 评价单线程和多线程版本的暴力匹配
    sad::evaluate_and_call(
        [&first, &second]() {
            std::vector<std::pair<size_t, size_t>> matches;
            sad::bfnn_cloud(first, second, matches);
        },
        "暴力匹配（单线程）", 5);
    sad::evaluate_and_call(
        [&first, &second]() {
            std::vector<std::pair<size_t, size_t>> matches;
            sad::bfnn_cloud_mt(first, second, matches);
        },
        "暴力匹配（多线程）", 5);

    SUCCEED();
}

/** 
 * 除了暴力近邻外，所有算法都需要评测最近邻的正确性
 * @param truth 真值
 * @param esti  估计
 */
void EvaluateMatches(const std::vector<std::pair<size_t, size_t>>& truth,
                     const std::vector<std::pair<size_t, size_t>>& esti) {
    int fp = 0;  // false-positive，esti存在但truth中不存在
    int fn = 0;  // false-negative, truth存在但esti不存在
	
    LOG(INFO) << "truth: " << truth.size() << ", esti: " << esti.size();

    /// 检查某个匹配在另一个容器中存不存在
    auto exist = [](const std::pair<size_t, size_t>& data, const std::vector<std::pair<size_t, size_t>>& vec) -> bool {
        return std::find(vec.begin(), vec.end(), data) != vec.end();
    };

    int effective_esti = 0;	// 计算最近邻的次数
    for (const auto& d : esti) {
        if (d.first != sad::math::kINVALID_ID && d.second != sad::math::kINVALID_ID) {
            effective_esti++;

            if (!exist(d, truth)) {
                fp++;
            }
        }
    }

    for (const auto& d : truth) {
        if (!exist(d, esti)) {
            fn++;
        }
    }
	// 准确率
    float precision = 1.0 - float(fp) / effective_esti;
    // 召回率
    float recall = 1.0 - float(fn) / truth.size();
    LOG(INFO) << "precision: " << precision << ", recall: " << recall << ", fp: " << fp << ", fn: " << fn;
}

// 二维栅格和三维体素的测试案例
TEST(CH5_TEST, GRID_NN) {
    sad::CloudPtr first(new sad::PointCloudType), second(new sad::PointCloudType);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty()) {
        LOG(ERROR) << "cannot load cloud";
        FAIL();
    }

    // voxel grid 至 0.05
    sad::VoxelGrid(first);
    sad::VoxelGrid(second);

    LOG(INFO) << "points: " << first->size() << ", " << second->size();

    std::vector<std::pair<size_t, size_t>> truth_matches;
    sad::bfnn_cloud(first, second, truth_matches);

    // 对比不同种类的grid
    sad::GridNN<2> grid0(0.1, sad::GridNN<2>::NearbyType::CENTER), grid4(0.1, sad::GridNN<2>::NearbyType::NEARBY4),
        grid8(0.1, sad::GridNN<2>::NearbyType::NEARBY8);
    sad::GridNN<3> grid3(0.1, sad::GridNN<3>::NearbyType::NEARBY6);

    grid0.SetPointCloud(first);
    grid4.SetPointCloud(first);
    grid8.SetPointCloud(first);
    grid3.SetPointCloud(first);

    // 评价各种版本的Grid NN
    // sorry没有C17的template lambda... 下面必须写的啰嗦一些
    LOG(INFO) << "===================";
    std::vector<std::pair<size_t, size_t>> matches;
    sad::evaluate_and_call(
        [&first, &second, &grid0, &matches]() { grid0.GetClosestPointForCloud(first, second, matches); },
        "Grid0 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid0, &matches]() { grid0.GetClosestPointForCloudMT(first, second, matches); },
        "Grid0 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid4, &matches]() { grid4.GetClosestPointForCloud(first, second, matches); },
        "Grid4 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid4, &matches]() { grid4.GetClosestPointForCloudMT(first, second, matches); },
        "Grid4 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid8, &matches]() { grid8.GetClosestPointForCloud(first, second, matches); },
        "Grid8 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid8, &matches]() { grid8.GetClosestPointForCloudMT(first, second, matches); },
        "Grid8 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid3, &matches]() { grid3.GetClosestPointForCloud(first, second, matches); },
        "Grid 3D 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid3, &matches]() { grid3.GetClosestPointForCloudMT(first, second, matches); },
        "Grid 3D 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    SUCCEED();
}

TEST(CH5_TEST, KDTREE_BASICS) {
    sad::CloudPtr cloud(new sad::PointCloudType);
    sad::PointType p1, p2, p3, p4;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;

    p2.x = 1;
    p2.y = 0;
    p2.z = 0;

    p3.x = 0;
    p3.y = 1;
    p3.z = 0;

    p4.x = 1;
    p4.y = 1;
    p4.z = 0;

    cloud->points.push_back(p1);
    cloud->points.push_back(p2);
    cloud->points.push_back(p3);
    cloud->points.push_back(p4);

    sad::KdTree kdtree;
    kdtree.BuildTree(cloud);
    kdtree.PrintAll();

    SUCCEED();
}

TEST(CH5_TEST, KDTREE_KNN) {
    sad::CloudPtr first(new sad::PointCloudType), second(new sad::PointCloudType);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty()) {
        LOG(ERROR) << "cannot load cloud";
        FAIL();
    }

    // voxel grid 至 0.05
    sad::VoxelGrid(first);
    sad::VoxelGrid(second);

    sad::KdTree kdtree;
    sad::evaluate_and_call([&first, &kdtree]() { kdtree.BuildTree(first); }, "Kd Tree build", 1);

    kdtree.SetEnableANN(true, FLAGS_ANN_alpha);

    LOG(INFO) << "Kd tree leaves: " << kdtree.size() << ", points: " << first->size();

    // 比较 bfnn
    std::vector<std::pair<size_t, size_t>> true_matches;
    sad::bfnn_cloud_mt_k(first, second, true_matches);

    // 对第2个点云执行knn
    std::vector<std::pair<size_t, size_t>> matches;
    sad::evaluate_and_call([&first, &second, &kdtree, &matches]() { kdtree.GetClosestPointMT(second, matches, 5); },
                           "Kd Tree 5NN 多线程", 1);
    EvaluateMatches(true_matches, matches);

    LOG(INFO) << "building kdtree pcl";
    // 对比PCL
    pcl::search::KdTree<sad::PointType> kdtree_pcl;
    sad::evaluate_and_call([&first, &kdtree_pcl]() { kdtree_pcl.setInputCloud(first); }, "Kd Tree build", 1);

    LOG(INFO) << "searching pcl";
    matches.clear();
    std::vector<int> search_indices(second->size());
    for (int i = 0; i < second->points.size(); i++) {
        search_indices[i] = i;
    }

    std::vector<std::vector<int>> result_index;
    std::vector<std::vector<float>> result_distance;
    sad::evaluate_and_call(
        [&]() { kdtree_pcl.nearestKSearch(*second, search_indices, 5, result_index, result_distance); },
        "Kd Tree 5NN in PCL", 1);
    for (int i = 0; i < second->points.size(); i++) {
        for (int j = 0; j < result_index[i].size(); ++j) {
            int m = result_index[i][j];
            double d = result_distance[i][j];
            matches.push_back({m, i});
        }
    }
    EvaluateMatches(true_matches, matches);

    LOG(INFO) << "done.";

    SUCCEED();
}

TEST(CH5_TEST, OCTREE_BASICS) {
    sad::CloudPtr cloud(new sad::PointCloudType);
    sad::PointType p1, p2, p3, p4;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;

    p2.x = 1;
    p2.y = 0;
    p2.z = 0;

    p3.x = 0;
    p3.y = 1;
    p3.z = 0;

    p4.x = 1;
    p4.y = 1;
    p4.z = 0;

    cloud->points.push_back(p1);
    cloud->points.push_back(p2);
    cloud->points.push_back(p3);
    cloud->points.push_back(p4);

    sad::OctoTree octree;
    octree.BuildTree(cloud);
    octree.SetApproximate(false);
    LOG(INFO) << "Octo tree leaves: " << octree.size() << ", points: " << cloud->size();

    SUCCEED();
}

TEST(CH5_TEST, OCTREE_KNN) {
    sad::CloudPtr first(new sad::PointCloudType), second(new sad::PointCloudType);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty()) {
        LOG(ERROR) << "cannot load cloud";
        FAIL();
    }

    // voxel grid 至 0.05
    sad::VoxelGrid(first);
    sad::VoxelGrid(second);

    sad::OctoTree octree;
    sad::evaluate_and_call([&first, &octree]() { octree.BuildTree(first); }, "Octo Tree build", 1);

    octree.SetApproximate(true, FLAGS_ANN_alpha);
    LOG(INFO) << "Octo tree leaves: " << octree.size() << ", points: " << first->size();

    /// 测试KNN
    LOG(INFO) << "testing knn";
    std::vector<std::pair<size_t, size_t>> matches;
    sad::evaluate_and_call([&first, &second, &octree, &matches]() { octree.GetClosestPointMT(second, matches, 5); },
                           "Octo Tree 5NN 多线程", 1);

    LOG(INFO) << "comparing with bfnn";
    /// 比较真值
    std::vector<std::pair<size_t, size_t>> true_matches;
    sad::bfnn_cloud_mt_k(first, second, true_matches);
    EvaluateMatches(true_matches, matches);

    LOG(INFO) << "done.";

    SUCCEED();
}

int main(int argc, char** argv) {
    // 初始化 Google glog 库。这一步通常在程序启动时调用，参数 argv[0] 是程序的名称，用于在日志文件中标识该程序。
    google::InitGoogleLogging(argv[0]);
    // 设置日志输出的阈值。此行代码指定日志级别为 google::INFO 及其以上（即 INFO、WARNING、ERROR 和 FATAL）会被输出到标准错误（stderr）。
    FLAGS_stderrthreshold = google::INFO;
    // 启用彩色日志输出到标准错误（stderr）。
    FLAGS_colorlogtostderr = true;

    // 初始化 Google Test 框架,解析命令行参数，在命令行可以使用如下参数：
    // --gtest_filter：指定要运行的测试用例。
    // --gtest_output：设置输出格式和文件，例如生成 XML 报告。
    // --gtest_repeat：重复运行测试的次数。
    testing::InitGoogleTest(&argc, argv);
    // 解析命令行标志，参数保存在全局的 FLAGS_ 变量中。
    // 如：DEFINE_double(ANN_alpha, 1.0, "AAN的比例因子");就可以使用FLAGS_ANN_alpha这个变量了
    google::ParseCommandLineFlags(&argc, &argv, true);
    // 执行所有定义的测试用例TEST(TestCaseName, TestName)，返回值为 0 表示所有测试通过，非零表示至少一个测试失败
    return RUN_ALL_TESTS();
}

```

#### 2.1.3 暴力最近邻实现

```c++
#include "ch5/bfnn.h"
#include <execution>

namespace sad {
// ********* 单点暴力近邻，单线程 *********
int bfnn_point(CloudPtr cloud, const Vec3f& point) {
    // 用匿名函数计算单点和目标点云所有点的距离
    // 用std::min_element找出最小值
    return std::min_element(cloud->points.begin(), cloud->points.end(),
                            [&point](const PointType& pt1, const PointType& pt2) -> bool {
                                return (pt1.getVector3fMap() - point).squaredNorm() <
                                       (pt2.getVector3fMap() - point).squaredNorm();
                            }) -
           cloud->points.begin();
}

// ********* 单点暴力K近邻，单线程 *********
std::vector<int> bfnn_point_k(CloudPtr cloud, const Vec3f& point, int k) {
    struct IndexAndDis {
        IndexAndDis() {}
        IndexAndDis(int index, double dis2) : index_(index), dis2_(dis2) {}
        int index_ = 0;
        double dis2_ = 0;
    };

    std::vector<IndexAndDis> index_and_dis(cloud->size());
    for (int i = 0; i < cloud->size(); ++i) {
        index_and_dis[i] = {i, (cloud->points[i].getVector3fMap() - point).squaredNorm()};
    }
    // 计算完所有距离后，用排序算法找出k个最近的
    std::sort(index_and_dis.begin(), index_and_dis.end(),
              [](const auto& d1, const auto& d2) { return d1.dis2_ < d2.dis2_; });
    std::vector<int> ret;
    std::transform(index_and_dis.begin(), index_and_dis.begin() + k, std::back_inserter(ret),
                   [](const auto& d1) { return d1.index_; });
    return ret;
}

// ********* 点云暴力近邻，多线程 *********
void bfnn_cloud_mt(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches) {
    // 先生成索引
    std::vector<size_t> index(cloud2->size());
    // 使用 C++17 标准库中的并行算法（std::for_each）来实现多线程并行化
    // 并行的充索引向量
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

    matches.resize(index.size());
    // std::execution::par_unseq 执行策略，允许并行和无序执行
    // 并行的计算每个点2：second在点1:first中最近的点
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](auto idx) {
        matches[idx].second = idx;
        matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
    });
}

// ********* 点云暴力近邻，单线程 *********
void bfnn_cloud(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches) {
    // 单线程版本
    std::vector<size_t> index(cloud2->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

    matches.resize(index.size());
    // std::execution::seq是std::for_each的单线程策略
    std::for_each(std::execution::seq, index.begin(), index.end(), [&](auto idx) {
        matches[idx].second = idx;
        matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
    });
}

// ********* 点云暴力k近邻，多线程 *********
void bfnn_cloud_mt_k(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches, int k) {
    // 先生成索引
    std::vector<size_t> index(cloud2->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

    // 并行化for_each
    matches.resize(index.size() * k);
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](auto idx) {
        // 调用单点暴力K近邻，单线程 
        auto v = bfnn_point_k(cloud1, ToVec3f(cloud2->points[idx]), k);
        for (int i = 0; i < v.size(); ++i) {
            matches[idx * k + i].first = v[i];
            matches[idx * k + i].second = idx;
        }
    });
}

}  // namespace sad

```

### 2.2 栅格/体素最近邻

暴力搜索本质上就是对数据结构的遍历查找，那显然排序后的数据结构可以用二分查找代替顺序查找来将时间复杂度降低到$O(log_2N)$。

因此我们也可以现在空间位置层面对点云进行索引，根据索引方式的不同，可以引出二维栅格、三位体素、二分树、四叉树等等近邻搜索算法。

- 二维栅格法：

  将空间按照二维投影的方式划分栅格，然后在查找点周围的栅格中寻找它的最近点。

  - 根据点云的疏密程度，还需要预先定义一个超参：栅格的**分辨率**。

    这是个经验值，很重要。格子太大点就多，导致计算效率下降。格子太小，格子数量太多，最近邻可能也落不到上下左右4个格子中。

- 三维体素法：

  将空间划分为三维的体素，然后在相邻体素中查找。
  
- 算法逻辑为：

  1. 计算给定点所在的栅格
  2. 根据最近邻的定义，查找附近的栅格
  3. 对第2步栅格内所有的点云进行暴力近邻搜索，找到最近邻

- 因为不一定能找到最近邻，所以除了暴力紧邻法外，所有的算法除了评估计算效率外还要评估它的正确性。
  $$
  \begin{align}
  Precision = 1-\frac{FP}{m}\\
  Recall = 1-\frac{FN}{n}
  \end{align}
  $$

  - FP: False Positive假阳性，检测出来时最近邻实际上不是最近邻的次数
  - FN: False Negative假阴性，没检测出来的最近邻次数
  - m: 总共计算的最近邻次数
  - n：真值中总共给出的最近邻

### 2.2+ 案例：栅格和体素法实现

#### 2.2.1 用于存放栅格数据的哈希表

栅格数据存放在一个哈希表unordered_map中，

之所以用哈希表，是因为点云是稀疏的，对应的栅格也是稀疏的，所以在没有数据的地方不必保留空的栅格。

其中空间哈希值的计算根据论文为：用各维度数据乘以一个大质数，再求异或，最后对大整数取模。假设空间点$\mathbf{p}=[p_x,p_y,p_z]$，3个大质数为$n_1,n_2,n_3$，大整数为$N$,它的哈希值为：
$$
hash(\mathbf{p})=((p_xn_1))\ xor\ (p_yn_2)\ xor\ (p_zn_3))\ mod\ N\tag{2.2.1.1}
$$

```c++
// * 存栅格数据的结构
// 参数1：键，2：值的类型，3：键的哈希值
std::unordered_map<KeyType, std::vector<size_t>, hash_vec<dim>> grids_; 

// * 模板结构：矢量哈希
template <int N>
struct hash_vec {
    inline size_t operator()(const Eigen::Matrix<int, N, 1>& v) const;
};

// * 2D和3D的哈希值生成
// 根据论文 Optimized Spatial Hashing for Collision Detection of Deformable Objects, Matthias Teschner et. al., VMV 2003 实现的哈希函数，即公式2.2.1.1式计算
// 2D
template <>
inline size_t hash_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v) const {
    return size_t(((v[0] * 73856093) ^ (v[1] * 471943)) % 10000000);
}
// 3D
template <>
inline size_t hash_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v) const {
    return size_t(((v[0] * 73856093) ^ (v[1] * 471943) ^ (v[2] * 83492791)) % 10000000);
}
```

#### 2.2.2 栅格体素近邻

使用模板来同时实现二维栅格和三维体素。

```c++
//
// Created by xiang on 2021/8/25.
//

#ifndef SLAM_IN_AUTO_DRIVING_GRID2D_HPP
#define SLAM_IN_AUTO_DRIVING_GRID2D_HPP

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "common/point_types.h"

#include <glog/logging.h>
#include <execution>
#include <map>

namespace sad {

/**
 * 栅格法最近邻的模板类:根据维数来选定2D栅格还是3D体素
 * @tparam dim 模板参数，使用2D或3D栅格
 */
template <int dim>
class GridNN {
   public:
    using KeyType = Eigen::Matrix<int, dim, 1>;     // 根据模板参数dim，来确定是2维还是3维的向量
    using PtType = Eigen::Matrix<float, dim, 1>;
    // 用一个枚举类型来定义4种近邻关系
    enum class NearbyType {
        CENTER,  // 只考虑中心
        // for 2D
        NEARBY4,  // 上下左右
        NEARBY8,  // 上下左右+四角

        // for 3D
        NEARBY6,  // 上下左右前后
    };

    /**
     * 构造函数
     * @param resolution 分辨率
     * @param nearby_type 近邻判定方法
     */
    explicit GridNN(float resolution = 0.1, NearbyType nearby_type = NearbyType::NEARBY4)
        : resolution_(resolution), nearby_type_(nearby_type) {
        inv_resolution_ = 1.0 / resolution_;

        // check dim and nearby
        if (dim == 2 && nearby_type_ == NearbyType::NEARBY6) {
            LOG(INFO) << "2D grid does not support nearby6, using nearby4 instead.";
            // 近邻关系被定义为枚举类型NearbyType
            nearby_type_ = NearbyType::NEARBY4;
        } else if (dim == 3 && (nearby_type_ != NearbyType::NEARBY6 && nearby_type_ != NearbyType::CENTER)) {
            LOG(INFO) << "3D grid does not support nearby4/8, using nearby6 instead.";
            nearby_type_ = NearbyType::NEARBY6;
        }
        // 根据近邻关系和维度dim，生成对应的近邻相对方位
        GenerateNearbyGrids();
    }

    /// 设置点云，建立栅格
    bool SetPointCloud(CloudPtr cloud);

    /// 获取某个点的最近邻
    bool GetClosestPoint(const PointType& pt, PointType& closest_pt, size_t& idx);

    /// 对比得到两个点云各个点之间的最近邻
    bool GetClosestPointForCloud(CloudPtr ref, CloudPtr query, std::vector<std::pair<size_t, size_t>>& matches);    // 单线程
    bool GetClosestPointForCloudMT(CloudPtr ref, CloudPtr query, std::vector<std::pair<size_t, size_t>>& matches);  // 多线程

   private:
    /// 根据最近邻的类型，生成附近网格
    void GenerateNearbyGrids();

    /// 空间坐标转到grid
    KeyType Pos2Grid(const PtType& pt);

    float resolution_ = 0.1;       // 分辨率
    float inv_resolution_ = 10.0;  // 分辨率倒数

    NearbyType nearby_type_ = NearbyType::NEARBY4;
    // 栅格数据被保存在一个哈希表unordered_map中
    // 之所以用哈希表，是因为点云是稀疏的，对应的栅格也是稀疏的，所以在没有数据的地方不必保留空的栅格
    std::unordered_map<KeyType, std::vector<size_t>, hash_vec<dim>> grids_;  //  栅格数据
    CloudPtr cloud_;

    std::vector<KeyType> nearby_grids_;  // 附近的栅格
};

// ******* 设置点云，建立栅格 *******
template <int dim>
bool GridNN<dim>::SetPointCloud(CloudPtr cloud) {
    // size_t 是 C++ 中的一个无符号整数类型，用于表示大小和索引。定义在 <cstddef> 头文件中
    std::vector<size_t> index(cloud->size());
    // std::for_each默认是单线程的顺序执行
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

    std::for_each(index.begin(), index.end(), [&cloud, this](const size_t& idx) {
        auto pt = cloud->points[idx];
        auto key = Pos2Grid(ToEigen<float, dim>(pt));   // 得到点云在栅格中的位置
        // end()返回一个指向哈希表末尾（最后一个元素之后）的迭代器。
        // 所有find()=end()就是没找到
        if (grids_.find(key) == grids_.end()) {
            grids_.insert({key, {idx}});    // 没找到(即该栅格索引还没建立)，就将新的键值队(栅格索引+点云索引)一起插入
        } else {
            grids_[key].emplace_back(idx);  // 找到了(即该栅格索引已经建立)，就将新的值(点云索引)归入栅格索引的verctor中去
        }
    });

    cloud_ = cloud;
    LOG(INFO) << "grids: " << grids_.size();
    return true;
}

// ******* 空间坐标转到grid，得到空间点在栅格中的位置 *******
template <int dim>
Eigen::Matrix<int, dim, 1> GridNN<dim>::Pos2Grid(const Eigen::Matrix<float, dim, 1>& pt) {
    // 使用表达式链对向量pt进行3次操作
    // pt.array() 将向量 pt 转换为一个数组，这样可以对向量的每个元素应用元素级别的操作。
    // round() 是一个元素级别的操作，它将数组中的每个浮点数四舍五入到最近的整数。
    // template cast<int>() 将四舍五入后的数组中的元素从浮点数类型转换为整数类型。
    return pt.array().template round().template cast<int>();
    // Eigen::Matrix<int, dim, 1> ret;
    // for (int i = 0; i < dim; ++i) {
    //     ret(i, 0) = round(pt[i] * inv_resolution_);
    // }
    // return ret;
}

// ******* 近邻关系：生成二维栅格相对位置 *******
template <>
void GridNN<2>::GenerateNearbyGrids() {
    // * 1. 只考虑中心栅格
    if (nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());

    // * 2. 考虑上下左右
    } else if (nearby_type_ == NearbyType::NEARBY4) {
        // Vec2i = Eigen::Vector2i;
        nearby_grids_ = {Vec2i(0, 0), Vec2i(-1, 0), Vec2i(1, 0), Vec2i(0, 1), Vec2i(0, -1)};

    // * 3. 考虑上下左右+4角
    } else if (nearby_type_ == NearbyType::NEARBY8) {
        nearby_grids_ = {
            Vec2i(0, 0),   Vec2i(-1, 0), Vec2i(1, 0),  Vec2i(0, 1), Vec2i(0, -1),
            Vec2i(-1, -1), Vec2i(-1, 1), Vec2i(1, -1), Vec2i(1, 1),
        };
    }
}

// ******* 近邻关系：生成三维体素相对位置 *******
template <>
void GridNN<3>::GenerateNearbyGrids() {
    // * 1. 只考虑中心体素
    if (nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    
    // * 2. 考虑上下左右+前后
    } else if (nearby_type_ == NearbyType::NEARBY6) {
        // KeyType = Eigen::Matrix<int, dim, 1>; 此时dim=3
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    }
}

// ******* 获取查找点的最近邻 *******
template <int dim>
bool GridNN<dim>::GetClosestPoint(const PointType& pt, PointType& closest_pt, size_t& idx) {
    // 在pt栅格周边寻找最近邻
    // 1. 得到查找点在栅格中的位置
    std::vector<size_t> idx_to_check;
    auto key = Pos2Grid(ToEigen<float, dim>(pt));

    // 2. 根据近邻关系，遍历查找点所在栅格的近邻栅格，将其中的点云加入到idx_to_check
    std::for_each(nearby_grids_.begin(), nearby_grids_.end(), [&key, &idx_to_check, this](const KeyType& delta) {
        auto dkey = key + delta;
        auto iter = grids_.find(dkey);
        if (iter != grids_.end()) {
            idx_to_check.insert(idx_to_check.end(), iter->second.begin(), iter->second.end());
        }
    });

    if (idx_to_check.empty()) {
        return false;
    }

    // 3. 暴力近邻搜索这些点
    // brute force nn in cloud_[idx]
    CloudPtr nearby_cloud(new PointCloudType);
    std::vector<size_t> nearby_idx;
    for (auto& idx : idx_to_check) {
        nearby_cloud->points.template emplace_back(cloud_->points[idx]);
        nearby_idx.emplace_back(idx);
    }

    size_t closest_point_idx = bfnn_point(nearby_cloud, ToVec3f(pt));
    idx = nearby_idx.at(closest_point_idx);
    closest_pt = cloud_->points[idx];

    return true;
}

// ******* 对比得到两个点云各个点之间的最近邻 *******
template <int dim>
bool GridNN<dim>::GetClosestPointForCloud(CloudPtr ref, CloudPtr query,
                                          std::vector<std::pair<size_t, size_t>>& matches) {
    matches.clear();
    std::vector<size_t> index(query->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });
    std::for_each(index.begin(), index.end(), [this, &matches, &query](const size_t& idx) {
        PointType cp;
        size_t cp_idx;
        if (GetClosestPoint(query->points[idx], cp, cp_idx)) {
            matches.emplace_back(cp_idx, idx);
        }
    });

    return true;
}

template <int dim>
bool GridNN<dim>::GetClosestPointForCloudMT(CloudPtr ref, CloudPtr query,
                                            std::vector<std::pair<size_t, size_t>>& matches) {
    matches.clear();
    // 与串行版本基本一样，但matches需要预先生成，匹配失败时填入非法匹配
    std::vector<size_t> index(query->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });
    matches.resize(index.size());

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [this, &matches, &query](const size_t& idx) {
        PointType cp;
        size_t cp_idx;
        if (GetClosestPoint(query->points[idx], cp, cp_idx)) {
            matches[idx] = {cp_idx, idx};
        } else {
            matches[idx] = {math::kINVALID_ID, math::kINVALID_ID};
        }
    });

    return true;
}

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_GRID2D_HPP

```

#### 2.2.3 GTest测试

使用2.1.2的GTest测试代码运行栅格最近邻

```
./bin/test_nn --gtest_filter=CH5_TEST.GRID_NN
```

结果为

```shell
➜  LiDAR-SLAM-code-comments git:(main) ✗ ./bin/test_nn --gtest_filter=CH5_TEST.GRID_NN
Note: Google Test filter = CH5_TEST.GRID_NN
[==========] Running 1 test from 1 test suite.
[----------] Global test environment set-up.
[----------] 1 test from CH5_TEST
[ RUN      ] CH5_TEST.GRID_NN
Failed to find match for field 'intensity'.
Failed to find match for field 'intensity'.
I0619 02:03:33.486518 35720 test_nn.cc:109] points: 18869, 18779
I0619 02:03:35.348310 35720 gridnn.hpp:103] grids: 1011
I0619 02:03:35.349009 35720 gridnn.hpp:103] grids: 1011
I0619 02:03:35.349695 35720 gridnn.hpp:103] grids: 1011
I0619 02:03:35.350472 35720 gridnn.hpp:103] grids: 1970
I0619 02:03:35.350477 35720 test_nn.cc:126] ===================
I0619 02:03:35.574054 35720 sys_utils.h:32] 方法 Grid0 单线程 平均调用时间/次数: 22.3571/10 毫秒.
I0619 02:03:35.574079 35720 test_nn.cc:66] truth: 18779, esti: 18017
I0619 02:03:35.637104 35720 test_nn.cc:92] precision: 0.814897, recall: 0.781831, fp: 3335, fn: 4097
I0619 02:03:35.637121 35720 test_nn.cc:133] ===================
I0619 02:03:35.691697 35720 sys_utils.h:32] 方法 Grid0 多线程 平均调用时间/次数: 5.45661/10 毫秒.
I0619 02:03:35.691730 35720 test_nn.cc:66] truth: 18779, esti: 18779
I0619 02:03:35.755334 35720 test_nn.cc:92] precision: 0.814897, recall: 0.781831, fp: 3335, fn: 4097
I0619 02:03:35.755363 35720 test_nn.cc:139] ===================
I0619 02:03:36.574146 35720 sys_utils.h:32] 方法 Grid4 单线程 平均调用时间/次数: 81.8776/10 毫秒.
I0619 02:03:36.574162 35720 test_nn.cc:66] truth: 18779, esti: 18589
I0619 02:03:36.631775 35720 test_nn.cc:92] precision: 0.979235, recall: 0.969327, fp: 386, fn: 576
I0619 02:03:36.631795 35720 test_nn.cc:145] ===================
I0619 02:03:36.724370 35720 sys_utils.h:32] 方法 Grid4 多线程 平均调用时间/次数: 9.25685/10 毫秒.
I0619 02:03:36.724390 35720 test_nn.cc:66] truth: 18779, esti: 18779
I0619 02:03:36.781195 35720 test_nn.cc:92] precision: 0.979235, recall: 0.969327, fp: 386, fn: 576
I0619 02:03:36.781210 35720 test_nn.cc:151] ===================
I0619 02:03:37.999262 35720 sys_utils.h:32] 方法 Grid8 单线程 平均调用时间/次数: 121.804/10 毫秒.
I0619 02:03:37.999284 35720 test_nn.cc:66] truth: 18779, esti: 18649
I0619 02:03:38.056667 35720 test_nn.cc:92] precision: 0.996515, recall: 0.989616, fp: 65, fn: 195
I0619 02:03:38.056694 35720 test_nn.cc:157] ===================
I0619 02:03:38.198238 35720 sys_utils.h:32] 方法 Grid8 多线程 平均调用时间/次数: 14.1537/10 毫秒.
I0619 02:03:38.198283 35720 test_nn.cc:66] truth: 18779, esti: 18779
I0619 02:03:38.267237 35720 test_nn.cc:92] precision: 0.996515, recall: 0.989616, fp: 65, fn: 195
I0619 02:03:38.267266 35720 test_nn.cc:163] ===================
I0619 02:03:38.738590 35720 sys_utils.h:32] 方法 Grid 3D 单线程 平均调用时间/次数: 47.1312/10 毫秒.
I0619 02:03:38.738615 35720 test_nn.cc:66] truth: 18779, esti: 18385
I0619 02:03:38.794773 35720 test_nn.cc:92] precision: 0.976503, recall: 0.956015, fp: 432, fn: 826
I0619 02:03:38.794790 35720 test_nn.cc:169] ===================
I0619 02:03:38.841099 35720 sys_utils.h:32] 方法 Grid 3D 多线程 平均调用时间/次数: 4.63046/10 毫秒.
I0619 02:03:38.841117 35720 test_nn.cc:66] truth: 18779, esti: 18779
I0619 02:03:38.897716 35720 test_nn.cc:92] precision: 0.976503, recall: 0.956015, fp: 432, fn: 826
[       OK ] CH5_TEST.GRID_NN (5418 ms)
[----------] 1 test from CH5_TEST (5418 ms total)

[----------] Global test environment tear-down
[==========] 1 test from 1 test suite ran. (5418 ms total)
[  PASSED  ] 1 test.
```

可以发现：

- 增加要搜索的近邻栅格数，会增加计算时间
- 多线程比单线程性能提升明显
- 体素法和栅格法效率持平
- 在准确率和召回率指标上，三维体素略好于二维栅格
  - 二维栅格数量越多，准确率召回率越好(高)
  - 栅格分辨率越高在自动驾驶中从目前的0.1略微提高到0.5也会提高准确率召回率

### 2.3 二分树与K-d树

#### 2.3.1 基本理解

- 二分树：

  对于一个已经排过序的容器，二分查找有着更高的效率$O(log_2 N)$，线性查找是$O(N)$。空间复杂度也只需要$O(N)$。

  但二分树只对一维数据有效，但点云属于三维空间，因此需要K-D树

- K-D树

  - K-D树也是二叉树的一种，二叉树用单个维度的信息来区分左右，K-D树用**超平面(Hyperplane)**来区分左右。对于三维点来说，超平面就是普通的二维平面。

    在SLAM中由于构建和查找过程都需要实时运行，因此需要选择一种简单的分割方法，最简单的一种是**沿轴超平面分割(Axis-aligned Splitting Plane)**，即沿着超平面所有维度中任意一个轴将点云分开即可(如下图)。

    ![](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/K-D%E5%88%92%E5%88%86)

  - K-D树中，以树状结构表达点云的结构关系，规定：

    1. 每个节点有左右两个分支
    2. 叶子节点表示原始点云中的点。储存时，可以存储点的索引而非点本身，这样可以节省空间。
    3. 非叶子节点存储一个分割轴和分割阈值，来表达如何分割左分支和右分支。例如上图蓝色x=5可以存储为第1个分割轴，阈值为1的方式来分割。规定左侧分支取小于号，右侧分枝取大于等于号。

#### 2.3.2 k-d树的构建

k-d树的构建时，主要考虑如何对给定点云进行分割。不同分割方法的策略不同，形成各种K-D树

一种以“计算当前点云在各轴上的分散程度，取分散程度最大的轴作为分割轴”的k-d树构建步骤：

1. 输入：点云数据$\mathbf{X}=\{x_1,\cdots,x_n\}$，其中$x_i\in\mathbb{R}^k$。
2. 考虑将子集$\mathbf{X}_n\subset\mathbf{X}$插入节点n。
3. 如果$\mathbf{X}_n$为空，则退出。
4. 如果$\mathbf{X}_n$只有一个点，则记为叶子节点，退出
5. 计算$\mathbf{X}_n$在各个轴的方差，挑分布最大的一个轴，记为$j$；
   取平均数$m_j=\mathbf{X}_n[j]$作为分割阈值。
6. 遍历$x\in\mathbf{X}_n$，对于$x[j]<m_j$的，插入左节点；否则插入右节点。
7. 递归上述步骤直到所有点都被插入树中。

#### 2.3.3 k-d树的查找

与二叉树类似，可以用前序、中序、后序等方法遍历。

- K-d树的最近邻查找步骤：
  1. 输入：K-d树T，查找点x
  2. 输出：x的最近邻
  3. 记当前节点为$n_c$，最初取$n_c$为根节点。记 $d$ 为当前搜索到的最小距离。
     1. 如果$n_c$是叶子，则计算$n_c$与x的距离。看它是否小于 d ,如果是则记$n_c$为最近邻，回退到它的父节点。
     2. 如果$n_c$不是叶子，则计算x落在$n_c$的哪一侧。如果$n_c$所在的一侧未被展开(查找)，则优先展开$n_c$所在的一侧。
     3. 计算是否需要展开$n_c$的另一侧(二叉树的剪枝操作)：如果x与$n_c$分割面的距离$d'<d$，则必须展开另一侧，否则跳过另一侧分枝。
     4. 如果两侧都已经展开，或者不必展开，则返回上一节点，直到$n_c$变为根节点。

- K-d树的K近邻查找步骤：
  1. 输入：K-d树T，查找点x，最近邻数k
  2. 输出：K近邻集合N
  3. 记当前节点为$n_c$，最初取$n_c$为根节点。令函数$S(n_c)$表示在$n_c$下进行k近邻搜索：
     1. 如果$n_c$是叶子，则计算$n_c$与x的距离是否小于 N 中的最大距离。若是，则将$n_c$放入 N。若此时$|N|>k$，则删除 N 中距离最大的匹配点。
     2. 计算 x 落在$n_c$的哪一侧，递归调用$S(n_c.left)$或$S(n_c.right)$
     3. 计算是否需要展开$n_c$的另一侧(二叉树的剪枝操作)：
        - $|N|<k$时，必须展开。
        - $|N|=k$且$x$与$n_c$的分割面距离小于N中最大匹配距离，也进行展开
     4. 如果$n_c$的另一侧不需要展开，则函数返回。否则继续调用另一侧的近邻搜索算法。

### 2.3+ 案例：K-d树的最近邻实现

k-d树有很多开源实现，下面实现一个简单的k-d树

#### 2.3.1 建树部分

- k-d树节点的结构体，k-d树实现类的定义

  https://github.com/Fernweh-yang/LiDAR-SLAM-code-comments/blob/main/src/ch5/kdtree.h

- k-d树类中各个方法的实现

  https://github.com/Fernweh-yang/LiDAR-SLAM-code-comments/blob/main/src/ch5/kdtree.cc

- 测试建树过程的代码

  ```shell
  ➜  LiDAR-SLAM-code-comments git:(main) ✗ ./bin/test_nn --gtest_filter=CH5_TEST.KDTREE_BASICS
  Note: Google Test filter = CH5_TEST.KDTREE_BASICS
  [==========] Running 1 test from 1 test suite.
  [----------] Global test environment set-up.
  [----------] 1 test from CH5_TEST
  [ RUN      ] CH5_TEST.KDTREE_BASICS
  I0624 02:01:50.776080 30062 kdtree.cc:229] leaf node: 6, idx: 3
  I0624 02:01:50.776193 30062 kdtree.cc:229] leaf node: 5, idx: 1
  I0624 02:01:50.776196 30062 kdtree.cc:231] node: 4, axis: 1, th: 0.5
  I0624 02:01:50.776206 30062 kdtree.cc:229] leaf node: 3, idx: 2
  I0624 02:01:50.776207 30062 kdtree.cc:229] leaf node: 2, idx: 0
  I0624 02:01:50.776208 30062 kdtree.cc:231] node: 1, axis: 1, th: 0.5
  I0624 02:01:50.776211 30062 kdtree.cc:231] node: 0, axis: 0, th: 0.5
  [       OK ] CH5_TEST.KDTREE_BASICS (0 ms)
  [----------] 1 test from CH5_TEST (0 ms total)
  
  [----------] Global test environment tear-down
  [==========] 1 test from 1 test suite ran. (1 ms total)
  [  PASSED  ] 1 test.
  ```

#### 2.3.2 kd树的k最近邻

#### 2.3.3 kd树的近似最近邻查找

### 2.4 四叉树与八叉树

kd树以二叉树作为基本的数据结构。对于2d和3d空间，我们还可以分别使用四叉树(Quad Tree)和八叉树(Octo Tree)。

四叉树一个节点有4个子节点，八叉树则有8个。这对应于物理空间就是，2d空间中是将矩形按中心切成四等分，3d空间中是将三维立方体按中心切成八等分。这种结构自然的定义了切割空间的准则，因此可以向kd树一样对点云构建四叉树或八叉树模型并搜索最近邻。

#### 2.4.1 八叉树的构建

- 八叉树和K-d树的主要区别

  - K-d树以分割面的形式来区分点集，而八叉树则使用立方体的形式。为此实现了一个box3d的结构体来处理点与八叉树之间的关系

  - 在建立k-d树时，分割面是动态确定的；而在八叉树中，对立方体的分割是固定(从中间分成八块)。

    这使得一个八叉树节点总时可以继续展开，但其子节点很可能没有点云。

  - 在初次构建八叉树时，会计算整个点云的包围盒作为八叉树的根节点边界框。这个边界框可以不是正方体。

  - 八叉树的最近邻查找过程类似于K-d树，也可以进行剪枝。这里使用**查询点到包围盒外侧的最大垂直距离**作为剪枝依据

- 八叉树构建算法

  1. 输入：点云数据$\mathbf{X}=\{x_1,\cdots,x_n\}$，其中$x_i\in\mathbb{R}^k$。
  2. 考虑将子集$\mathbf{X}_n\in\mathbf{X}$插入节点n。
     1. 如果$\mathbf{X}_n$为空，则退出
     2. 如果$\mathbf{X}_n$只有一个点，则记为叶子节点，退出。
  3. 按照**一分为八**的准则对n进s
  4. 遍历$x\in\mathbf{X}_n$，记录x落在哪一个子节点。然后对子节点和对应点云递归调用构建方法。
  5. 递归上述步骤直到所有点都被插入树中

#### 2.4.2 八叉树的查找

1. 输入：八叉树$T$，查找点$x$，最近邻数$k$

2. 输出：k近邻集合$N$

3. 记当前节点为$n_c$，最初取$n_c$为根节点。函数$S(n_c)$表示在$n_c$下进行$k$近邻搜索：

   1. 如果$n_c$是叶子，则计算$n_c$与$x$的距离是否小于$N$中最大距离；

      若是：将$n_c$放入N。若是，且$|N|>k$，则删除$N$中距离最大的匹配点。

   2. 计算$x$落在$n_c$的哪一个子节点。

      1. 如果$x$在$n_c$的边界盒外面，则展开每一个子节点。
      2. 如果落在内部，则优先展开$x$所在的子节点。
   
   3. 计算是否需要展开$n_c$的其他子节点。
   
      1. 展开条件判定为$|N|<k$时，必须展开
      2. $|N|=k$且$x$与$n_c$的前面所述距离计算结果小于$N$中最大匹配距离，也进行展开。
   
   4. 若$n_c$的子节点不需要展开，函数返回。
   
      否则，继续调用其他节点的近邻搜索算法。

### 2.4+ 案例：八叉树的最近邻

### 2.5 其他树类方法

对空间数据进行某种索引，达到快速查询相邻关系的目的，已经是一个古老而广泛的问题。在 **模式识别、分类、计算机视觉、编码理论、推荐系统、语音识别、化学、生物**等领域都存在最近邻相关的问题。

在SLAM中我们关心能够 **快速构建、快速查询**的**低维**数据结构，因此倾向于选择比较简单的模型。此外也需要这些结构能够随机器运动而快速变化。k-d树和八叉树可以很好的满足这两个要求。

但在上述其他应用领域可能允许较长的构建时间但期望更快的查询效率等不一样的需求，这就催生了一系列不同的空间数据索引方法(Spatial Data Indexing):

1. 树类：球树、R树、R*树、随机k-d树、AABB树

2. 空间填充线：希尔伯特曲线、Z曲线

   利用分形曲线，建立高维空间与一维之间的联系(同时保持相邻性)，达到在低维空间上搜索高维空间的效果。

3. 局部性敏感哈希(Locality Sensitive Hashing,LSH)算法。

   与空间填充线相反，LSH是从高维空间出发，寻找一种到低维空间的哈希算法，同时还能以概率形式保证相邻性。

### 2.6 总结

- 在点云数据不稠密的情况下，栅格法(快于)>k-d树>八叉树>暴力法
- 但由于栅格法随栅格内点数增多而线型增加，k-d树和八叉树则是对数增加，因此在处理稠密点云时，k-d树和八叉树更佳。

## 3. 拟合问题

这类问题也被称为检测(Detecting)或聚类(Clustering)问题，要实现基本元素的提取和估计。如提取人/车辆等的语义信息，然后这些元素要用来和点云进行匹配和配准(Registration)。

常见的做法是先利用最近邻结构找到一个点的若干个最近邻，再对这些最近邻进行拟合，认为他们符合某个固定的形状。最后调整车辆的位姿，使得扫描到的激光点与这些形状能匹配。

### 3.1 平面拟合

对点云进行线性拟合时最简单的一部分，从不同角度看一个线性拟合问题有不同称呼：**线性回归**linear regression(对只线的参数进行回归)，**主成分分析**principal component analysis PCA(对点云的主要分布轴进行分析)

#### 3.1.1 平面的拟合问题

- 问题给出：

  给定一组由$n$个点组成的点云$\mathbf{X}=\{x_1,\cdots,x_n\}$，其中每个点$x_k\in\mathbb{R}^3$，然后寻找一组平面参数$\mathbf{n},d$使得：
  $$
  \forall k\in[1,n],\mathbf{n}^T\mathbf{x}_k+d=0\tag{3.1.1.1}
  $$
  其中$\mathbf{n}\in\mathbb{R}^3$为法向量，$d\in\mathbb{R}$为截距。

- 转化为最小二乘问题

  1. 3.1.1.1式有4个未知量，虽然每个点云都可以提供一组方程，但当有多个点时受噪声的影响，3.1.1.1式时无解的(超定)。因此只能使用最小二乘法将其误差最小化：
     $$
     \mathop{min}_{\mathbf{n},d}\sum_{k=1}^n||\mathbf{n}^Tx_k+d||_2^2\tag{3.1.1.2}
     $$

  2. 如果取齐次坐标，则还可以再化简该问题，记：

     - 空间点的四维齐次坐标：

       $\widetilde{\mathbf{x}}=[\mathbf{x}^T,1]^T\in\mathbb{R}^4$

     - 平面参数也同样是个齐次向量：

       $\widetilde{\mathbf{n}}=[\mathbf{n}^T,d]^T\in\mathbb{R}^4$

  3. 3.1.1.2式可以简化为
     $$
     \mathop{min}_{\mathbf{n},d}\sum_{k=1}^n||\widetilde{\mathbf{x}}_k^T\widetilde{\mathbf{n}}||_2^2\tag{3.1.1.2}
     $$

- 将3.1.1.2转为通用最小二乘问题$Ax=0$：

  给定一个矩阵$A$，希望找一个非零向量$x$，使得$Ax$最小化，其中只考虑$x$的方向不考虑长度，因此设定$||x||=1$。

  对应到3.1.1.2也就是给定一系列点云组成的矩阵A: $\widetilde{\mathbf{x}}_k^T$，要找到一个非零向量x：$\widetilde{\mathbf{n}}$使得$Ax$最大或最小

  下面是2种解决类似平面拟合此类问题的方法：

#### 3.1.2 线性最小二乘法解法1：特征值解法

- 问题重述：

  - 代数意义上的线型最小二乘法是指给定矩阵$\mathbf{A}\in\mathbb{R}^{m\times n}$, 计算得到一个$x^*\in\mathbb{R}^n$,使得：
    $$
    x^{*}=\arg\min_{x}\left\|Ax\right\|_{2}^{2}=\arg\min_{x}x^{\top}A^{\top}Ax,\quad s.t.,\left\|x\right\|=1\tag{3.1.2.1}
    $$

  - 其中$A^TA$是一个实对称矩阵，可以利用特征值分解进行**对角化**(即主对角线为从大到小排列的特征值)：
    $$
    A^TA=V\Lambda V^{-1}
    $$
    其中$\Lambda$为对角特征值矩阵，从大到小排列为$\lambda_1,\cdots,\lambda_n$。$V$为正交矩阵，其**列向量为每一维特征值对应的特征向量**$v_1,\cdots,v_n$​，这些特征向量组成一组单位正交基。

    > 对角化：将一个矩阵表示为某种形式的对角矩阵,从而加快计算。
    >
    > 对角矩阵(diagonal matrix)是一个主对角线之外的元素皆为0的矩阵，常写为$diag(a_1，a_2,\cdots,a_n) $​
    >
    > 正交矩阵(旋转矩阵就是)：$R^T=R^{-1},R^TR=RR^T=I$
    

- 结论：

  $A^TA$的最小特征值所对应的特征向量可使$||Ax||$​最小，也就是说线形最小二乘法的最优解就是**最小特征值向量**。

- 证明方法1：

  - 因为$A^TA$是对称矩阵，所以它可以特征值分解，根据特征值向量性质它和特征向量$V$以及特征值$\lambda$的关系为：
    $$
    A^TAV=\lambda V\tag{3.1.2.2}
    $$

  - 那么：
    $$
    ||AV||=(AV)^T(AV)=V^TA^TAV=V^T\lambda V=\lambda\tag{3.1.2.3}
    $$

  - 特征值$\lambda$取最小时3.1.2.3式最小，即$||AV||$最小，即此时最小特征值对应的最小特征向量就是3.1.2.1式所求的$x^*$

- 证明方法2：

  - 特征向量矩阵$V=[v_1,\cdots,v_n]$是由一列列特征向量$v_k$组成的，这些特征向量是一组单位正交基，那么就可以用他们去表示任意n维的向量：
    $$
    \begin{align}
    x&=\alpha_1 v_1+\cdots+\alpha_n v_n\tag{3.1.2.4}\\
    &=[v_1,\cdots,v_n][\alpha_1,\cdots,\alpha_n]^T \\
    &=V\alpha^T
    \end{align}
    $$

  - 对3.1.2.4式左乘$V^{-1}$可得
    $$
    V^{-1}x=V^{T}x=V^{-1}V\alpha^T=[\alpha_1,\cdots,\alpha_n]^T\tag{3.1.2.5}
    $$

  - 3.1.2.1中的目标函数可以变为
    $$
    \begin{align}
    ||Ax||_2^2&=x^TA^TAx\\
    &=x^TV\Lambda V^{-1}x\\
    &=(V^Tx)^T\Lambda V^{-1}x\\
    &=[\alpha_1,\cdots,\alpha_n]\Lambda[\alpha_1,\cdots,\alpha_n]^T\\
    &=\sum_{k=1}^n\lambda_k\alpha_k^2\tag{3.1.2.6}
    \end{align}
    $$

  - 因为$\lambda$按从大大小排列，而3.1.2.1要求的$||x||=1$意味着$\alpha_1^2+\cdots+\alpha_n^2=1$，所以只需要选最小的$\lambda_k=\lambda_n$，$\alpha_n=1$其余为0就能使其最小。

#### 3.1.3 线性最小二乘法解法2：奇异值解法

特征值解法只可以用于方阵，但奇异值分解(SVD, Singular Value Decomposition)可以对任意矩阵进行分解。

- 结论：

  对于使$Ax$最小的性最小二乘解就是正交矩阵V的最后一列。

- 由于可以对任意矩阵进行奇异值分解，因此不用像3.1.2.1那样对$A^TA$分解，可以直接对$A$进行奇异值分解：
  $$
  A=U\Sigma V^T\tag{3.1.3.1}
  $$
  其中U,V是正交矩阵，$\Sigma$是奇异值矩阵，奇异值同样由大到小排列

- 证明：

  将3.1.3.1代入3.1.2.1式中后可得到
  $$
  \begin{align}
  ||Ax||_2^2&=x^TA^TAx\\
  &=x^TV\Sigma^2 V^{-1}x\\
  &=(V^Tx)^T\Sigma^2 V^{-1}x\\
  &=[\alpha_1,\cdots,\alpha_n]\Sigma^2[\alpha_1,\cdots,\alpha_n]^T\\
  &=\sum_{k=1}^n\sigma^2_k\alpha_k^2\tag{3.1.2.6}
  \end{align}
  $$

- 因此和特征值分解一样，要让3.1.2.6最小也就是选奇异值$\sigma$最小的那个，所以$x^*$也就是正交矩阵V最后那一列。

### 3.2 案例： 平面拟合的实现

### 3.3 直线拟合

#### 3.3.1 直线逆拟合问题：

假设点集$X$，由n个三维点组成。设直线上的点$x$满足方程：
$$
x=\mathbf{d}t+\mathbf{p}\tag{3.3.1.1}
$$
其中$\mathbf{d,p}\in\mathbb{R}^3,t\in\mathbb{R}$。

我们相求的是直线方向$\mathbf{d}$(满足$||\mathbf{d}||=1$)，以及直线上的一点$\mathbf{p}$，共六个未知量。和平面拟合一样，这依然是一个超定方程，需要构建最小二乘问题来求解。

#### 3.3.2 转为最小二乘法问题

1. 对于任意一个不在直线上的点$x_k$，利用勾股定理，可以计算该点到直线的距离平方为：
   $$
   f_{k}^{2}=\left\|x_{k}-p\right\|^{2}-\left\|\left(x_{k}-p\right)^{\top}d\right\|^{2}\tag{3.3.2.1}
   $$
   第二项向量相乘得到的是斜边到直线的投影

2. 构造一个求解$d,p$的最小二乘问题：
   $$
   \left(d,p\right)^{*}=\arg\min_{d,p}\sum_{k=1}^{n}f_{k}^{2},s.t.\left\|d\right\|=1\tag{3.3.2.2}
   $$
   虽然可以直接不断算距离平方和相加，但显然还可以继续简化

3. 分离3.3.2.1式中的$d，p$，先考虑$\frac{\part f_k^2}{\part p}$：
   $$
   \begin{align}
   \frac{\partial f_{k}^{2}}{\partial p} &=-2\left(x_{k}-p\right)+2\underbrace{\left(x_{k}-p\right)^{\top}d}_{\text{标量,}=d^{\top}\left(x_{k}-p\right)}d, \\
   &=\left(-2\right)\left(I-dd^{\top}\right)\left(x_{k}-p\right)\tag{3.3.2.3}
   \end{align}
   $$

4. 目标函数3.3.2.2式整体对$p$的导数为：
   $$
   \begin{align}
   \frac{\partial\sum_{k=1}^{n}f_{k}^{2}}{\partial p}& =\sum_{k=1}^{n}\left(-2\right)\left(I-dd^{\top}\right)\left(x_{k}-p\right), \\
   &=\left(-2\right)\left(I-dd^{\top}\right)\sum_{k=1}^{n}\left(x_{k}-p\right).
   \end{align}\tag{3.3.2.4}
   $$

5. 要求最小二乘的极值，就是让其导数3.3.2.4式等于0，得到：
   $$
   p=\frac{1}{n}\sum_{k=1}^{n}x_{k}\tag{3.3.2.5}
   $$
   说明p不管什么数据集都应该取当前点云的中心。

6. 现在确定了p，只需要求直线方向$d$了，记$y_k=x_k-p$为已知量并代入3.3.2.1式
   $$
   f_{k}^{2}=y_{k}^{\top}y_{k}-d^{\top}y_{k}y_{k}^{\top}d\tag{3.3.2.6}
   $$
   可以发现第一个误差项不包含要求的方向$d$，因此舍去。而第二项的最小化，相当于求去掉负号的最大化：
   $$
   d^{*}=arg\max_{d}\sum_{k=1}^{n}d^{\top}y_{k}y_{k}^{\top}d=\sum_{k=1}^{n}\left\|y_{k}^{\top}d\right\|_{2}^{2}\tag{3.3.2.7}
   $$

7. 记$A=[y_1^T,\cdots,y_n^T]^T$，那么3.3.2.7就类似于3.1.1.2了：
   $$
   d^*=arg\ \mathop{max}_d||Ad||_2^2\tag{3.3.2.8}
   $$
   只是和平面拟合求最小不同，这里变成了求最大。因此该问题的解为

   - 奇异值分解法：取$d$为A的最大右奇异向量。
   - 特征值分解法：取$d$为$A^TA$的最大特征对应的特征向量。

### 3.4 案例：直线拟合的实现

### 3.5 总结

- 直线和平面拟合问题很相似，都可以转换为零值问题$Ax=0$的最大/最小值求解

- 直线拟合是寻找最大值的方向，对应于矩阵的最大特征(奇异)向量。

  平面拟合是寻找最小值的方向，对应于矩阵的最小特征(奇异)向量。

![](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/%E7%82%B9%E4%BA%91%E6%8B%9F%E5%90%88%E7%A4%BA%E6%84%8F%E5%9B%BE)

- 平面拟合是将N维空间中的点云拟合为$N-1$维空间，而直线拟合则是变成一维空间的拟合。

  在其他领域，这被称为数据降维。他们就想海绵中挤水一样，奇异值较小的方向就是水分，奇异值较大的地方是干货要保留下来。因此也叫主成分分析(PCA)

  这种方法就可以用于数据压缩、特征提取等领域，比如点云中面特征就是找这个面的法向量(平面拟合)，边特征就是要找这个这个边的直线(直线拟合)。

# 六、2D SLAM

相比于三维空间的点云，2D SLAM可以视为在俯视视角下工作的激光SLAM算法。在这种视角下，激光雷达扫描数据和地图数据都可以被简化为二维形式。

2D SLAM框架的固有问题如：遇到物体形状随高度有明显变化(桌面和桌腿)，2D地图无法有效描述该类物体，导致机器人碰撞。倾斜坡面导致传感器读数与真实距离存在几何上的差异。需要用3D SLAM系统来解决。

## 1. 一个经典的2D SLAM框架

1. 二维激光雷达按照一定频率向外输出激光测距信息，我们称一圈数据为一次扫描(scan)

2. 为了计算这次扫描对应的机器位姿，需要将它与某些东西进行**匹配**（也叫配准,registration）。这个过程叫做扫描匹配。

   既可以将scan与前一次scan进行匹配，也可以将它与地图进行匹配，所以扫描匹配方法也可以进一步分为scan to scan和scan to map

3. 当我们估计了这个scan的位姿之后，就要将它组装到地图中。

   - 由于scan的本质就是点云，因此最简单的方案就是把所有的scan按时间顺序放到地图中去。

     但可能会受累积误差或运动物体的影响。

   - 现代SLAM通常使用**子地图**模式，将一些邻近的scan归入一个子地图，再将子地图拼接起来。

     子地图内部是固定的，不必重复计算。同时子地图拥有自己独立的坐标系，他们相互之间的位姿是可以调整优化的。

     子地图模式利于回环检测和地图更新。

4. 为了表达地图中的 **障碍物**和 **可通行区域** ，会用到**占据栅格地图**(Occupancy Grid)来进行地图的管理。占据栅格地图可以有效的过滤运动物体对地图的影响，使地图更加干净。

## 2. 扫描匹配算法

- 点到点或点到线的ICP即可以用于scan to scan匹配，也可以用于scan to map的配准。
  - 如果将地图保存为离散的二维点，ICP类方法可以用相同的方式来做地图匹配。
- 2D-SLAM中，通常按一定分辨率将地图储存为 **占据栅格地图**(Occupancy Grid)。2.5章的似然场法(Gaussian Likelihood Field)是一种可以把扫描数据与栅格地图进行配准的方法。

### 2.1 点到点的扫描匹配

- 扫描数据

  - 单次二维扫描数据由极坐标：一组角度和距离的数值对来表达，不妨记作$(\rho.r)_i$。其中$\rho$表示该点与车辆自身的角度，$r$表示该点自身的距离，$i=0,\cdots,N$，具体测量点个数N由激光的角分辨率决定。
  - 实际激光雷达打到的点被称为**末端点**(End Point)，它有两层物理意义：
    1. 末端点本身代表了一个实际存在的障碍物
    2. 传感器到末端点连线上，是没有障碍物的

- 问题引出

  - 二维激光雷达的**观测模型**：
    $$
    \mathbf{z}=\mathbf{h}(\mathbf{x},\mathbf{m})+\mathbf{w}\tag{2.1.1}
    $$

    - $\mathbf{z}$：二维激光雷达扫描数据，观测数据
    - $\mathbf{z}$是机器人在位姿$\mathbf{x}$对某张地图$\mathbf{m}$进行观测之后得到的数据。
    - $\mathbf{w}$是噪声项

  - **目的**：

    通过观测到的$\mathbf{z}$和$\mathbf{m}$来估计位姿$\mathbf{x}$

  - 根据**贝叶斯估计理论**，$\mathbf{x}$可以通过最大后验(MAP)或者最大似然估计(MLE)来得到：
    $$
    \mathbf{x}_{MLE}=arg\ \mathop{max}p(x|z,m)=arg\ \mathop{max}p(z|x,m)\tag{2.1.2}
    $$
    若只考虑scan to scan的问题，这里的m可以简单的写为上一次扫描数据。

    此时问题的关键变为如何定义**观测方程**的详细形式，即为每一个观测数据计算残差项。经典的算法有：

    - 点到点的扫描匹配算法ICP
    - 点到线的扫描匹配算法PL-ICP, ICL，高斯似然场法
    - 点到面的扫描匹配算法（3D SLAM）

  - 观测方程的具体定义**涉及以下问题**：

    1. 如何选取匹配点：

       基于效率的考量通常不会用所有的点，而是进行降采样：可以均匀采样，随机采样，基于法线、特征的采样

    2. 数据关联问题 ,如何确定某个具体扫描点(x,y)对应地图上的哪个点：

       由最近邻法求解，假设按照当前的估计位姿，与观测点最近的那个地图点即为匹配点。

    3. 在确定扫描点(x,y)与对应的地图点m后，如何计算它的残差：

       完整的激光雷达扫描噪声模型太过复杂，实际中会进行简化，甚至将其直接看做一个二维高斯分布$w\sim\mathcal{N}(0,\Sigma)$。

- 迭代最近点(Iterative Closest Point)算法

  ICP算法将扫描匹配问题分为**数据关联**和**位姿估计**两步，然后交替执行这两个步骤直到算法收敛。、

  - 第一步：位姿变换，将机器人坐标系下的扫描点$p_i^B$转换到世界坐标系下$p_i^W$​
    
    设二维激光雷达的位姿为$\mathbf{x}=\mathbf{T}_{WB}=[x,y,\theta]^T$，机器人坐标系下的某个扫描点$p_i^B$的距离与角度为$r_i,\rho_i$，
    
    - 那么激光点的在世界坐标系下的位姿为：$p_i^w=T_{WB}p_i^B$​
    
      在当前2DSLAM情况下，即：
    
    $$
    p_i^w=[x+r_icos(\rho_i+\theta),y+r_isin(\rho+\theta)]\tag{2.1.3}
    $$
    
  - 第二步：构建残差

    假设$p_i^W$附近找到了一个最近邻$q_i^W$，则他们的残差为:
    $$
    e_i=p_i^W-q_i^W\tag{2.1.4}
    $$

  - 第三步：转为最小二乘法问题
    $$
    \mathbf{x}=[x,y,\theta]^T=arg\ \min_x\sum_{i=1}^n||e_i||_2^2\tag{2.1.5}
    $$
    为了求解最小而二乘问题，需要给出残差e对各个状态变量的导数，2D-SLAM就很简单都不需要流形符号：
    $$
    \frac{\partial e_i}{\partial \mathbf{x}}=[\frac{\partial e_i}{\partial x}\ \frac{\partial e_i}{\partial y}\ \frac{\partial e_i}{\partial \theta}]^T=\left [\begin{array}{cccc}
    1 & 0  \\
    0 & 1  \\
    -r_isin(\rho_i+\theta) & r_icos(\rho_i+\theta)  \\
    \end{array}\right]^T\in\mathbb{R}^{2\times3}\tag{2.1.6}
    $$
    
  - 需要注意的是，如果状态变量$[x,y,\theta]^T$发生了变化，那么近邻点$q_i^W$也会跟着变化。如果状态变量一开始离最优解很远，那么它的近邻点$q_i^W$​就是个错误的值。因此ICP类算法非常一类于优化的初始值。
  

### 2.1+ 案例：OpenCV显示2D激光雷达数据

### 2.2 案例：点到点的ICP实现(高斯牛顿法)

### 2.3 点到线的扫描匹配算法

- 流程和点到点的ICP算法一样：

  - 在最近邻点的查找中，需要查找多个最近邻如k个，然后用这k个最近邻拟合一条直线。
  - 最后计算目标点与这条直线的垂直距离。

- 直线拟合：

  使用第五章3.3中的直线拟合方法(最大特征向量)拟合k个最近邻点$(x_1,\cdots,x_k),\forall i\in1,\dots,k,x_i\in\mathbb{R}^2$，这个直线由方向向量$d$和原点$p$​描述。

  设直线方程为
  $$
  ax+by+c=0\tag{2.3.1}
  $$
  其中a,b,c为直线的参数，所以直线的拟合可以构造为一个参数估计的最小二乘问题：
  $$
  (a,b,c)^*=arg\ min\sum_{i=1}^{N}||ax_i+by_i+c||_2^2\tag{2.3.2}
  $$
  上面这个最小二乘的解法即：将各个近邻点的坐标排列乘矩阵
  $$
  A=\left [\begin{array}{cccc}
  x_1 & y_1 & 1 \\
  x_2 & y_2 & 1 \\
  &\cdots&\\
  x_k & y_k & 1 \\
  \end{array}\right]\tag{2.3.3}
  $$
  然后求A的最小奇异值向量。

- 残差构建：要让匹配点到该直线的距离最小

  1. 任意一点到直线的垂直距离为：
     $$
     d=\frac{ax+by+c}{\sqrt{a^2+b^2}}\tag{2.3.4}
     $$
     由于分母固定，所以直接使用残差：$e=ax+by+c$

  2. 残差的雅可比矩阵
     $$
     \frac{\partial e}{\partial x}=a,\ \frac{\partial e}{\partial y}=b\tag{2.3.5}
     $$

  3. 在2.3.5中加入雷达的位姿$(x,y,\theta)$，某个激光点到雷达的距离和夹角为$(r_i,\rho_i)$，激光点在世界坐标系下的位姿为$p_i^W$

     得到残差$e_i$到位姿的雅可比矩阵可以由链式法则表示：
     $$
     \frac{\partial e_i}{\partial x}=\frac{\partial e_i}{\partial p_i^W}\frac{\partial p_i^W}{\partial x}\tag{2.3.6}
     $$

  4. 式2.3.6的第一项即2.3.5式，第二项即2.1.6式

### 2.4 案例：点到线ICP的实现

### 2.5 似然场法

- **与ICP的区别**

  - ICP：计算每个点到最近邻之间的欧氏距离误差，这个误差的平方作为目标函数

  - 似然场：在地图中每一个点附近定义一个不断向外衰减(随距离平方衰减或高斯衰减)的场，测量点在这个场的度数就是该点的误差函数。

    有了似然场后，就不需要用K-d树一类的最近邻结构来获取某个点的最近点，而可以直接使用似然场中的读数。

- **用处**

  - 配准两个扫描数据
  - 配准一个扫描数据和一个地图数据
  - 和栅格地图配合，用于地图匹配

- **基于似然场的扫描匹配算法公式推导**

  - 某个扫描点$\mathbf{p}_i^B$经过位姿$\mathbf{x}$变换后，得到世界坐标系的点$\mathbf{p}_i^W$。同时，存在一个世界坐标系下的似然场 $\pi$，于是这个扫描点在似然场$\pi$中的读数为$\pi(\mathbf{p}_i^W)$。这里的位姿通过求解最小二乘的优化问题来得到：
    $$
    \mathbf{x}^*=\mathop{arg}\ \mathop{min}_x\sum_{i=1}^n||\pi(\mathbf{p}_i^W)||_2^2 \tag{2.5.1}
    $$

  - 目标函数对位姿$\mathbf{x}$的雅可比矩阵为：
    $$
    \frac{\partial \pi}{\partial x}=\frac{\partial \pi}{\partial \mathbf{p}_i^W}\frac{\partial \mathbf{p}_i^W}{\partial x} \tag{2.5.2}
    $$

    - 后一项，即2.1.6式

    - 第一项：

      1. 因为似然场以图像形式存储，因此必须对$\mathbf{p}_i^W$根据某种分辨率进行采样。假设$\mathbf{p}_i^W$到它的图像坐标$\mathbf{p}_i^f$的转换关系为：
         $$
         \mathbf{p}_i^f=\alpha\mathbf{p}_i^W+c\tag{2.5.3}
         $$

         - $\alpha$：缩放倍率

         - $c\in\mathbb{R}^2$：图像中心的偏移量

           由于图像的起始坐标通常位于左上角，而物体坐标的零点通常位于中心，因此偏移量通常是图像尺寸的一半

      2. 目标函数$\pi$相对于$\mathbf{p}_i^W$的导数为：
         $$
         \frac{\partial \pi}{\partial \mathbf{p}_i^W}=\frac{\partial \pi}{\partial \mathbf{p}_i^f}\frac{\partial \mathbf{p}_i^f}{\partial \mathbf{p}_i^W}=\alpha[\Delta\pi_x,\Delta\pi_y]^T\tag{2.5.4}
         $$

         - $[\Delta\pi_x,\Delta\pi_y]$为似然场在图像上面的梯度

    - 两项乘在一起后就是：
      $$
      \frac{\partial \pi}{\partial x}=[\alpha\Delta\pi_x,\alpha\Delta\pi_y,-\alpha\Delta\pi_xr_isin(\rho_i+\theta)+\alpha\Delta\pi_yr_icos(\rho_i+\theta)]^T\tag{2.5.5}
      $$

### 2.6 案例：似然场法的实现(高斯牛顿)

### 2.7 案例：似然场法的实现(g2o优化器)

## 3. 占据栅格地图

### 3.1 占据栅格地图的原理

- 占据栅格：以栅格的形式(或者以图像的形式存储)占据概率的的地图

  - 栅格：简单的二维地图形式，它把地图分为许多平面的小格，然后在每个格子内存储一些自定义的信息。

    可以存储：

    - 障碍物信息，用于路径规划
    - 语义信息

  - 当只需要表达每个栅格是否有物体占据时，一开始每个栅格的概率是0.5，如果在经过多次扫描观测某个栅格存在障碍物，那么它的占据概率就会慢慢的上升到1。反之，多次观测都是可通行的话，占据概率就会逐渐降低到0。

- 占据栅格应该满足以下几个描述：

  1. 以栅格的形式存储每个格子被障碍物占据的概率(0-1)
  2. 栅格具有一定的分辨率，且通常是稠密的
  3. 占据栅格的概率会随着观测而发生改变(如本来停的车跑路了)

### 3.2 案例：基于Bresenham算法的地图生成

- Bresenham算法：

  这是一种对直线进行栅格化的算法，通常用于几何直线的矢量化。因为可以完全由整数运算来实现，因此效率非常高。

### 3.3 案例：基于模板的地图生成

如果不使用直线填充算法，或者直线数量或者距离明显超过指定区域(比如射程超过200米的雷达)，就可以考虑使用模板算法进行栅格填充。

- 模板算法：每个格子的距离值与角度都是提前算好的

  如果更新栅格地图时，需要比较模板中**每个格子的距离值**与**激光在该角度下的距离值**

  1. 如果模板距离 < 激光打到的距离，就可以认为该格子是空白的
  2. 如果相等=，就认为该格子被占据。
  3. 如果大于>，则不更新栅格地图

## 4. 子地图

子地图submap：将若干个匹配好的扫描数据按时间顺序放在一起，可以用于2D-SLAM和3D-SLAM

### 4.1 子地图的原理

- 子地图含有一个随时调整的位姿，$\mathbf{T}_{WS}\in SE(2)$，W：世界坐标系，S：子地图坐标系。

  在进行配准时，scan to map算法实际计算的是当前激光扫描与子地图的位姿关系$\mathbf{T}_{SC}$，这里$C$表示当前的扫描数据坐标系。

  由上可得到每个扫描数据的世界坐标为：
  $$
  \mathbf{T}_{WC}=\mathbf{T}_{WS}\mathbf{T}_{SC} \tag{4.1.1}
  $$

​	在扫描匹配时，会计算当前激光雷达的 $\mathbf{T}_{SC}$ ，这个变量算出后保持不变。之后要优化的是子地图相对于世界的位姿 $\mathbf{T}_{WS}$，因此回环检测时可以把子地图视为一个基本单元来处理，而不需要考虑每一帧在世界坐标系下的位姿。

- 一个基于子地图的2D SLAM的实现逻辑：
  1. 一个子地图对应一个似然场和一个栅格地图。
  
  2. 总是把当前的扫描数据与当前的子地图进行匹配，得到该扫描数据在当前子地图中的位姿。
  
  3. 如果机器人发生移动或转动，则按一定距离和角度取关键帧
  
  4. 如果机器人的移动范围超出了当前子地图，或者当前子地图包含的关键帧超出了一定数量，就建立一个新的子地图。
  
     新的子地图以当前帧为中心，它的位姿$\mathbf{T}_{WS}=\mathbf{T}_{WC}$。此时，新地图上面完全没有数据。
  
     为了后续配准，需要把子地图里最近的一些关键帧复制到新的子地图中。
  
  5. 合并每个子地图的栅格地图，就可以得到全局地图。

### 4.2 案例：子地图的实现

## 5.回环检测与闭环

如果没有回环检测，不管是LO还是LIO都会随着时间推移产生累计误差。

- 将当前扫描数据或子地图 与 历史地图配准，再调整各子地图之间的相对位置关系就可以消除累计误差。但有如下细节问题：

  1. 应该检查那些历史子地图？

     直观上，与当前扫描数据空间位置相近的子地图都应该被配准，但这种空间位置关系是基于当前估计轨迹建立的。

     如果累积误差变得过大，可能导致当前估计轨迹与实际回环区域存在显著偏差，导致回环没有被正确检测到。所以需要对累积误差的大小有一个大概的了解。

  2. 检测回环时用的配准方法与里程计中的配准方法稍有不同：

     - 里程计的配准是基于 **连续运动**的：前提是最优解与初始状态差别不大。

     - 回环的位姿初始值由 **累积误差的多少决定**，所以位姿初值可能与优化值较远，而不管ICP还是似然场法都受到初值的严重影响。

       因此回环配准算法通常是在原有ICP或似然场基础上，增加一些针对较差位姿初值的处理方法，比如 **网格搜索**(Grid Searching), **粒子滤波**(Particle Filter)，**分枝定界**(Branch and Bound,BAB)，**金字塔法**(Image Pyramid)

  3. 检测到回环后，需要对整个地图进行位姿修正。

     这种修正既可以基于关键帧的位姿来做，也可以基于子地图的位姿来做。

     但子地图数量远小于关键帧的数量，基于子地图的回环优化也无法修复单个子地图内部的畸变，所以很多系统还是使用关键帧作为基本的数据单元。

- 金字塔式的**回环检测**

  - 也被称为 **由粗至精**(Coarse-to-Fine), **多分辨率**(Multi-Resolution) 的配准方法。与其说是一种方法，不如说是一种解决初值问题的思路。

  - 以多分辨率的似然场匹配为例，它依旧是一个扫描匹配问题，无非是在原先的基础上增加了初始位姿估计不太准确的条件。

    为了减少初始值不准确带来的影响，可以现在小分辨率的似然场中先进行一次配准，然后把粗配准的结果投影到高分辨率的似然场中，形成由粗至精的匹配过程。

  - 在多分辨率配准中，只要匹配点数超过所有扫描数据的一定比例(比如40%)，就可以认为匹配上了

    - 这是因为：激光雷达扫描不是360°旋转的，导致当前扫描和历史子地图虽然在同一位置，但可能朝向不同，从而导致扫描数据与地图存在很大的差异。
    - 如果这个比例(阈值)调高了，回环检测会更准确严格，但会导致机器人要完全回到出发点并具有相同朝向时才会产生回环效果。
    - 如果这个比例(阈值)调低了，回环检测会变灵敏宽松，但也可能导致错误的回环结果。

- 基于子地图的**回环修正**

  - 如果上面的回环检测成功的得到了当前帧与历史子地图之间的配准关系，就会启动一次回环修正。

    这个问题可以建模为一个SE(2)上的位置图(Pose Graph)问题。由于已经构建了子地图，因此可以仅使用子地图位姿作为优化节点。

  - 现在，通过回环检测已经计算了当前帧在历史子地图中的位姿：

    记历史子地图$S_1$本身的位姿为$\mathbf{T}_{WS_1}$，当前帧自身的位姿为$\mathbf{T}_{WC}$，当前帧所在的子地图$S_2$的位姿为$\mathbf{T}_{WS_2}$。**那么多分辨率匹配实际计算的应该是相对位姿**$\mathbf{T}_{S_1C}$。于是，可以把这个结果转换为历史子地图$S_1$与当前子地图$S_2$之间的位姿变换：
    $$
    \mathbf{T}_{S_1S_2}=\mathbf{T}_{S_1C}\mathbf{T}^{-1}_{WC}\mathbf{T}_{WS_2}\tag{5.0.1}
    $$

  - 回环优化中，以每个子地图的位姿为优化变量，构建一个位姿图进行优化。

    该位姿图的观测数据来源于两种：

    1. 相邻位姿图之间的相对位姿观测
    2. 回环检测计算出来的两个子地图之间的相对位姿关系

    为了让5.0.1式等式尽可能成立，于是他们之间的残差项为
    $$
    e=Log(\mathbf{T}_{WS_1}^{-1}\mathbf{T}_{WS_2}\mathbf{T}_{S_1S_2}^{-1})\in\mathbb{R}^3\tag{5.0.2}
    $$
    

### 5.1 案例：多分辨率的回环检测

### 5.2 案例：基于子地图的回环修正

### 5.3 一些讨论

# 七、3D SLAM

既可以对三维点云直接进行配准，也可以提取一些几何特征后再进行配准。

## 1. 多线激光雷达的工作原理

-  机械旋转式激光雷达：

  - 可以探测周身360度范围内的场景
  - 测距原理： 激光脉冲发射和返回的时间间隔 * 光速 = 被测物体的距离

  - 探头的数量也称为线束，通常是2的倍数：4线、16线、32线、64线、128线。

- 固态激光雷达：

  - 测量原理有两种
    1. 转镜式/半固态激光雷达：激光雷达的发射器和接受器保持不动，在前方加装一个棱镜结构，通过棱镜的转动来改变激光的走向
    2. 纯固态激光雷达：存在一个面阵式的发射和接收机构 
       - 相位控制技术：按照一定顺序来扫描
       - Flash技术：同时扫描、同时接收
  - 固态激光雷达的水平视野(FoV，Field of View)通常在120度内。垂直视野和机械旋转式激光雷达相当。
  - 精度上两者没有本质差距，但固态激光雷达由于结构更简单所以成本更低、寿命更长。

## 2. 多线激光雷达的扫描匹配

### 2.1 点到点ICP

### 2.1+ 案例：点到点ICP

### 2.2 点到线、点到面ICP

### 2.2+ 案例：点到线、点到面ICP

### 2.3 NDT方法

- 从ICP演进：

  - 点到点的ICP是点配准到单个点上。

  - 而点到线、点到面ICP是把点配准到某个**统计量**上去，即分别对局部点进行平面拟合和直线拟合。

  - 但可以不可以不精确的知道这些点是平面还是直线呢？只用一组点云最基本的统计信息，如：**均值** 和 **方差**呢？

    由此得到传统的NDT(Normal Distribution Transform)方法

- NDT的大概思路

  1. 把目标点云按一定分辨率分成若干体素
  2. 计算每个体素中的点云高斯分布。设第$k$个体素中的均值为$\mu_k$，方差为 $\Sigma_k$。
  3. 配准时，先计算每个点落在哪个 体素中，然后建立该点与该体素的$\mu_k,\Sigma_k$构成的残差。
  4. 利用高斯-牛顿或L-M对估计位姿进行迭代，最后更新位姿估计。

- **最小二乘问题构建**(上面的第三、四步)

  - 被配准点云$q_i$，经过$R,t$变换后，落在某个统计指标为$\mu_i,\Sigma_i$的体素中，则这个栅格中的残差为：
    $$
    e_i=Rq_i+t-\mu_i\tag{2.3.1}
    $$

  - 最后的$R,t$由一个加权的最小二乘问题决定：
    $$
    (R,t)^*=\mathop{arg}\ \mathop{min}_{R,\ t}\sum_i(e_i^T\Sigma_i^{-1}e_i)\tag{2.3.2}
    $$

  - 由最小二乘法的知识，可以将上述问题转化为一个最大似然估计(MLE)：最大化每个点落在所在栅格的概率分布。
    $$
    (R,t)^*=\mathop{arg}\ \mathop{max}_{R,t}\prod_iP(Rq_i+t)\tag{2.3.3}
    $$

    - 上式说明：既然目标栅格的点云符合某个统计形状，那么在正确的位姿估计下，落在其中的点也应该符合这个分布。
    - $\Sigma_i^{-1}$ 相当于提供了这个最小二乘问题的权重分布

- 一个加权最小二乘问题的高斯-牛顿法如下：
  $$
  \sum_i(J_i^T\Sigma_i^{-1}J_i)\Delta x = -\sum_iJ_i^T\Sigma_i^{-1}e_i\tag{2.3.4}
  $$

  - $\Delta x$：每一步的增量
  - $J_i$：残差项对自变量的雅可比矩阵。



### 2.3+ 案例：NDT方法

## 3. 直接法激光雷达里程计

## 4. 特征法激光雷达里程计

## 5. 松耦合LIO系统

# 八、紧耦合LIO系统

# 九、自动驾驶车辆的离线地图构建

# 十、自动驾驶车辆的实时定位系统