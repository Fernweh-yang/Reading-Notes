# 零、资料汇总

- 官方原版[git仓库](https://github.com/gaoxiang12/slam_in_autonomous_driving)

  包含书上的代码还有数据集

  - 各章代码在`src/chxx`下
  - 共有代码在`src/common`下

- 他人上深蓝课的[git仓库](https://github.com/ClarkWang1214/slam_in_autonomous_driving_shenlan_hw?tab=readme-ov-file)

  额外的有课后作业答案和修改代码

## 1. 全代码编译

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

## 2. 单独编译

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

展现了四元数下的旋转更新(2.2.13)和SO3下的旋转更新(2.1.3)

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

### 1.1 IMU测量值

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

### 1.2 IMU测量中的噪声模型

IMU噪声由两部分组成：测量噪声(Measurement Noise)和零偏(Bias)。

- 加入噪声的**测量方程**：
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

## 2. 用IMU进行轨迹推算

## 3. 卫星导航

## 4. 使用误差状态卡尔曼滤波器实现组合导航

## 5. 案例：实现ESKF的组合导航