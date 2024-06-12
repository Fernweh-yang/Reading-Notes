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

点云是最基本的三维结构表达方式，也是多数激光雷达向外输出的数据形式。

数组可以用来表示最基本的点云，但通常点云还会携带反射率、所属线束、RGB颜色等信息，因此最常用的是定义一个点云结构体。

### 1.3 Packet数据包的表达

数据包是雷达向外输出的另一种数据形式，为了节省雷达与计算机数据通信量。

数据被分为2部分：

- 雷达内参数：

  雷达旋转率、各探头相对雷达中心的俯仰角等在雷达设计时已知的参数。会放再固定的参数文件中

- 真正测量的数据：

  只记录距离和反射率。

每个雷达厂商的packet格式都不一样，packet格式是一种压缩后的数据，算法中还是要先将他们转化为可以直接读坐标的格式。

### 1.4 案例：地图表达方式BEV和range image

## 2. 最近邻问题

最近邻问题是许多点云问题的基本问题，问题描述为：

在一个含n个点的点云$\mathcal{X}=\{x_1,\cdots,x_n\}$中，我们想要知道

1. 离$x_m$最近的点是哪一个？
2. 离它最近的k个点又有哪些？ - > k近邻查找(kNN)
3. 与它的距离小于固定范围r的点有哪些？ - > 范围查找 range search

### 2.1 暴力最近邻法

### 2.1+ 案例：暴力最近邻法的代码实现

### 2.2 栅格/体素最近邻

### 2.2+ 案例：栅格和体素法实现

### 2.3 二分树与K-d树

### 2.3+ 案例：K-d树的最近邻实现

### 2.4 四叉树与八叉树

### 2.4+ 案例：八叉树的最近邻

### 2.5 其他树类方法



## 3. 拟合问题

这类问题也被称为检测(Detecting)或聚类(Clustering)问题，要实现基本元素的提取和估计。如提取人/车辆等的语义信息，然后这些元素要；用来和点云进行匹配和配准(Registration)。

常见的做法是先利用最近邻结构找到一个点的若干个最近邻，再对这些最近邻进行拟合，认为他们符合某个固定的形状。最后调整车辆的位姿，使得扫描到的激光点与这些形状能匹配。

### 3.1 平面拟合

### 3.2 案例： 平面拟合的实现

### 3.3 直线拟合

### 3.4 案例：直线拟合的实现



# 六、2D SLAM

相比于三维空间的点云，2D SLAM可以视为在俯视视角下工作的激光SLAM算法。在这种视角下，激光雷达扫描数据和地图数据都可以被简化为二维形式。

2D SLAM框架的固有问题如：遇到物体形状随高度有明显变化(桌面和桌腿)，2D地图无法有效描述该类物体，导致机器人碰撞。倾斜坡面导致传感器读数与真实距离存在几何上的差异。需要用3D SLAM系统来解决。

## 1. 一个经典的2D SLAM框架

## 2. 扫描匹配算法

### 2.1 点到点的扫描匹配

### 2.1+ 案例：OpenCV显示2D激光雷达数据

### 2.2 案例：点到点的ICP实现

### 2.3 点到线的扫描匹配算法

### 2.4 案例：点到线ICP的实现

### 2.5 似然场法

### 2.6 案例：似然场法的实现(高斯牛顿)

### 2.7 案例：似然场法的实现(g2o优化器)

## 3. 占据栅格地图

### 3.1 占据栅格地图的原理

### 3.2 案例：基于Bresenham算法的地图生成

### 3.3 案例：基于模板的地图生成

## 4. 子地图

### 4.1 子地图的原理

### 4.2 案例：子地图的实现

## 5.回环检测与闭环

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

## 3. 直接法激光雷达里程计

## 4. 特征法激光雷达里程计

## 5. 松耦合LIO系统

# 八、紧耦合LIO系统

# 九、自动驾驶车辆的离线地图构建

# 十、自动驾驶车辆的实时定位系统