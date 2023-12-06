# 资源汇总

- [论文](https://jakobengel.github.io/pdf/engel14eccv.pdf)
- [git 代码](https://github.com/tum-vision/lsd_slam)
- [参考博客](https://blog.csdn.net/lancelot_vim?type=blog)

# 安装使用

- 环境

  ```
  ubuntu20
  ros:Noetic
  opencv:4.8.1
  qt:5.12.8
  ```

- 安装

  ```shell
  # 在lsd-ws/src下clone修改过的lsd-slam版本
  https://github.com/Fernweh-yang/SLAM_Code_Learning/tree/main/lsd_ws/src
  catkin_make
  ```

- 运行

  ```
  rosrun lsd_slam_core dataset _files:=/home/yang/Downloads/LSD_room_images/LSD_room/images _hz:=1 _calib:=/home/yang/Downloads/LSD_room_images/LSD_room/cameraCalibration.cfg
  ```

  - 如果遇到gtk2.0和3.0冲突的问题，可能是多个opencv版本带来的。将ros和opencv所有版本都删除重装后就可以了

# 一些思考

## 为什么在跟踪时需要金字塔

在SLAM中图像金字塔的应用，主要是用来解决尺度不变性问题。尺度不变性由构建图像金字塔，并在金字塔的每一层上检测角点来实现

> **图像金字塔**：图像多尺度表达的一种，由一张原始图通过向下采样获得。层级越高，则图像越小，分辨率越低。
>
> **尺度**：图像中物体或场景的相对大小或比例。
>
> **尺度不变**：不同尺度下具有相似特征的场景在特征提取和匹配中能够被正确检测和匹配的性质。

# 基本原理

- 特点：

  - LSD在cpu上实现了实时的半稠密场景的重建，对特征缺失区域也可以进行重建。

  - 但是LSD对相机内参和曝光很敏感，且在相机快速运动的时候容易丢失。

  - 没有基于直接法的回环检测，因此还是需要依赖特征点方法来进行回环检测。

## 一、LSD-SLAM整体框架

![image-20231112202242729](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/LSD-SLAM%E6%A1%86%E6%9E%B6.png)

LSD-SLAM系统分为3个模块

1. Tracking

   通过光度误差来估算 **当前帧** 相对于 **参考关键帧** 的相对位姿

   > 为了更好更快的求解位姿，本文使用了方差归一化后的光度误差(也就是对光度误差除了一个方差，参见论文公式14)

2. Depth Map Estimation

   当前帧Tracking成功后，先判断是否将其视为新的关键帧。

   > **关键帧的创建标准**：相机移动足够远，就创建一个新的关键帧。距离定义参见论文16式。
   >
   > **关键帧 key frame**：相隔一定距离的具有代表性的帧，每一个关键帧对应一个深度图
   >
   > **参考帧 reference frame**：是用于和当前帧做立体匹配的帧，一个关键帧后面一般有许多参考帧
   >
   > **立体匹配**：对两个或多个不同视点或不同时间拍摄的图像进行分析和比对，以确定它们之间的对应关系和空间位置关系

   如果将其构建为新的关键帧，则构建新的深度图（Depth Map Creation）。

   如果不将其构建为新关键帧，则更新参考关键帧的深度（Depth Map Refinement）

   不管构不构建新的关键帧，都需要进行深度估计，深度估计有3步：

   1. 用立体视觉方法来从先前的帧得到新的深度估计
   2. 深度图帧与帧之间的传播
   3. 部分规范化已知深度

3. Map Optimization

   主要完成3个任务

   1. 位姿优化

      区别于tracking中用$se(3)$来估计位姿，并且是每帧图像与它参考帧之间的位姿

      这里使用相似矩阵$sim(3)$来估计位姿，并且只估计关键帧之间的位姿

   2. 点云地图优化

   3. 回环检测

      回环完成后，再进行全局优化

   一旦关键帧变成参考帧，那么对应的深度图将不再进行优化



## 二、四大线程

主要的线程有4个：

- 主线程：**跟踪线程**

  在main_on_images.cpp中通过for遍历所有图片来不断循环调用`system->trackFrame(image.data, runningIDX, hz == 0, fakeTimeStamp)`

- 子线程1：**建图线程**

  在SlamSystem.cpp中的`mappingThreadLoop()`通过一个while不断循环调用`doMappingIteration()`。

- 子线程2：**一致性约束线程**

  在SlamSystem.cpp中的`constraintSearchThreadLoop()`通过一个while不断循环调用`findConstraintsForNewKeyFrames(newKF, true, true, 1.0);`。

- 子线程3：**全局优化线程**

  在SlamSystem.cpp中的`optimizationThreadLoop()`通过一个while不断循环调用`optimizationIteration(50, 0.001)`

主线程遍历完所有的图片后，会调用`SlamSystem::finalize()`完成对3个子线程的最后一次通信，随后通过`~SlamSystem()`将三个子线程的while控制变量keepRunning设为false来退出所有的子线程，最后删除所有分配的内存。

### 1. 跟踪&建图

依靠`unmappedTrackedFramesSignal`条件变量进行控制：如果建图失败，建图线程会被这个条件变量锁住，需要等待跟踪线程的唤醒。

建图失败的可能：

1. 跟踪线程还没执行初始化`randomInit()`
2. 如果setting.cpp中的`doMapping`变量被设成了false，默认为true如果为false建图线程相当于没了。
3. 在`updateKeyframe()`中如果当前关键帧没有可以用来更新自己深度图的普通帧

跟踪线程什么时候会唤醒：

1. 跟踪出问题了需要重定位，`trackingIsGood`此时为false
2. 每一次跟踪循环结束时，都会唤醒一次。



## 三、不确定度计算

来源于作者的另一篇论文：Semi-Dense Visual Odometry for a Monocular Camera

一个函数$f(X)$由于它的输入$X$不确定而引起的不确定度可以由如下公式计算：
$$
\Sigma_f \approx \mathbf{J_f}\mathbf{\Sigma}_{\mathbf{X}}\mathbf{J}_f^T\tag{1}
$$

- $\mathbf{\Sigma_{X}}$：假设输入数据X是高斯分布的，它的协方差为$\mathbf{\Sigma_{X}}$
- $\mathbf{J_f}$：函数$f(X)$的雅可比矩阵
- $\mathbf{\Sigma_{f}}$：函数$f(X)$的协方差，也就是不确定度

> **协方差covariance**：度量各个维度偏离其均值的程度
>
> 协方差的值：
> - 如果为正值，则说明两者是正相关的(从协方差可以引出“相关系数”的定义)
> - 如果为负值就说明负相关的
> - 如果为0，也是就是统计上说的“相互独立”。

### 深度不确定度

当前帧和参考帧匹配后，最佳匹配点的逆深度可以表示为：
$$
d^*=d(I_0,I_1,\xi,\pi) \tag{2}
$$
其中$\xi$为两帧的相对位姿，$\pi$为相机投影模型参数。误差方差为：
$$
\sigma_d=J_d\Sigma J_d^T \tag{3}
$$
其中$J_d$是深度的雅可比，$\Sigma$是输入误差的方差

### 深度的3个误差项

参考博客：https://blog.csdn.net/kokerf/article/details/78006703?spm=1001.2014.3001.5502

#### 1. 几何视差误差（Geometric disparity error）

![image-20231204052553676](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/lsd%E5%87%A0%E4%BD%95%E8%A7%86%E5%B7%AE%E8%AF%AF%E5%B7%AE.png)

该误差引起的方差为：
$$
\sigma_{\lambda(\xi,\pi)}^2=J_{\lambda^*(l_0)}\begin{pmatrix}\sigma_l^2&0\\0&\sigma_l^2\end{pmatrix}J_{\lambda^*(l_0)}^T=\frac{\sigma_l^2}{\langle g,l\rangle^2}\tag{4}
$$


#### 2. 光度视差误差（photometric disparity error）

![image-20231204052723043](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/lsd%E5%85%89%E5%BA%A6%E8%A7%86%E5%B7%AE%E8%AF%AF%E5%B7%AE.png)

该误差引起的方差为：
$$
\sigma_{\lambda(I)}^2=\text{Var}(\lambda^*(I))=(\text{Var}(i_{ref})+\text{Var}(I_p))g_p^{-2}=\frac{2\sigma_i^2}{g_p^2}
\tag{5}
$$

#### 3. 逆深度计算误差（Pixel to inverse depth conversion）

当旋转比较小的时候，逆深度和视差近似成一个比例关系，所以构造的逆深度的观测方差由前面2个误差因素引起：像素位置 + 两图像的基线长度
$$
\sigma_{d,\text{obs}}^2=\alpha^2\begin{pmatrix}\sigma_{\lambda(\xi,\pi)}^2+\sigma_{\lambda(I)}^2\end{pmatrix}\tag{6}
$$
这里的像素逆深度系数$\alpha$:
$$
\alpha:=\frac{\partial{d}}{\partial{\lambda}}=\frac{\partial{d}}{\partial{x}}\frac{\partial{x}}{\partial{u}}\frac{\partial{u}}{\partial{\lambda}}=\frac{r_2p_k{\cdot}t_x-r_0p_k{\cdot}t_z}{(t_x-t_z{\cdot}x)^2}{\cdot}f_{x}^{inv}{\cdot}\frac{\partial{u}}{\partial{\lambda}}\tag{7}
$$
考虑到在极线上搜索匹配点的时候，是使用了多个点，因此给出逆深度误差的上限：
$$
\sigma_{d,\text{obs}}^2\le\alpha^2\begin{pmatrix}\text{min}\{\sigma_{\lambda(\xi,\pi)}^2\}+\text{min}\{\sigma_{\lambda(I)}^2\}\end{pmatrix}\tag{8}
$$

## 四、键帧和参考帧

- 关键帧：
  - 第一帧会在被初始化时成为关键帧
  - 每移动足够距离一个普通帧就可能被选为关键帧
  - 只有关键帧会被构建深度图
  - 只有关键帧会去计算绝对位姿并参与后端优化和回环检测

- 参考帧：
  - 最新的关键帧就是当前的参考帧
  - 每一个关键帧会成为它后面至少5帧的参考帧
  - 普通帧只会计算自己相对于参考帧的相对位姿


## 五、跟踪

- 整体结构

  ```
  图像金字塔迭代level-4到level-1
  Step1: 对参考帧当前层构造点云(reference->makePointCloud)
  Step2: 计算变换到当前帧的残差和梯度(calcResidualAndBuffers)
  Step3: 计算法方差归一化残差(calcWeightsAndResidual)
  Step4: 计算雅克比向量以及A和b(calculateWarpUpdate)
  Step5: 计算得到收敛的delta，并且更新SE3(inc = A.ldlt().solve(b))
  Step6: 重复Step2-Step5直到收敛或者达到最大迭代次数
  计算下一层金字塔
  ```

 - `calcWeightsAndResidual()`

   **计算光度误差损失函数**(论文公式12)
   $$
   E_p(\mathbf\xi_{ji}) =\sum_{\mathbf{p}\in\Omega_{D_i}}
   \Biggl\|\frac{r_p^2(\mathbf{p},\mathbf\xi_{ji})}{\sigma_{r_p(\mathbf{p},\mathbf\xi_{ji})}^2}\Biggr\|_\delta \tag{论文12式}
   $$
   其中分子$r_p$是光度误差，分母$\sigma$是误差系数(方差)。

   

   计算归一化方差的光度误差系数(论文公式14) 
   $$
   \sigma_{r_p(\mathbf{p},\mathbf\xi_{ji})}^2 := 2\sigma_I^2 + (\frac{\partial{r_p(\mathbf{p}, \mathbf\xi_{ji})}}{\partial{D_i(\mathbf{p})}})^2V_i(\mathbf{p}) \tag{论文14式}
   $$

   - $D_i$：第i帧的深度图
   - $V_i$：第i帧的方差图
   - $\sigma_I$：Gaussian image intensity noise高斯图像强度噪声？

   因为两帧之间位姿变换小所以作者只考虑了位移没考虑旋转，所以论文14式的梯度项简化为：
   $$
   \begin{split}
   \frac{\partial{r_p(\mathbf{p},\mathbf\xi_{ji})}}{\partial{D_i(\mathbf{p})}}
   &= \frac{\partial({I_i({\mathbf p}) - I_j(\omega({\mathbf p}, D_i({\mathbf p}), \xi_{ji})))}}{\partial{D_i(\mathbf{p})}} \\
   &= - \frac{\partial{I_j(\mathbf{a})}}{\partial{\mathbf{a}}}\bigg|_{\mathbf{a}=\mathbf{p}} \cdot\frac{\partial{w(d)}}{\partial{d}}\bigg|_{d=D_i(\mathbf{p})} \\
   &= -\begin{pmatrix}dxfx&dyfy\end{pmatrix}\cdot\begin{pmatrix}\frac{\mathbf{t}_x(1/d+\mathbf{t}_z)-\mathbf{t}_z(\mathbf{p}_x/d+\mathbf{t}_x)}{(1/d+\mathbf{t}_z)^2d} \\
   \frac{\mathbf{t}_y(1/d+\mathbf{t}_z)-\mathbf{t}_z(\mathbf{p}_y/d+\mathbf{t}_y)}{(1/d+\mathbf{t}_z)^2d}\end{pmatrix}\\
   &= -(dxfx\frac{\mathbf{t}_xz'-\mathbf{t}_zx'}{z'^2d} + dyfy\frac{\mathbf{t}_yz'-\mathbf{t}_zy'}{z'^2d})
   \end{split} 
   $$

   - 第一个偏导是在参考帧的3D点$P$的位置求的

   - 第二个偏导是在参考帧的逆深度$D_i(P)$的点P处求的

   - $d_x,d_y$：点 p在图像 $I_j$投影位置处两个方向的梯度

   - 

- `calculateWarpUpdate()`

  **计算公式12的雅可比 然后 用最小二乘法求解位姿更新量，最后更新得到新的位姿变换SE3**

  12式的分母被当成了一个系数不参与求导，所以只有分子的光度误差$r_p$参与求导：

  光度误差$r_p$对于相机位姿的求导这里采用左乘扰动，12式优化的形式：
  $$
  \delta\mathbf{\xi}^{*}=\text{arg}\min_{\delta\mathbf{\xi}}E_p(\delta\mathbf{\xi}\circ\mathbf{\xi})=\text{arg}\min_{\delta\mathbf{\xi}}\sum_i{r_i^2(\delta\mathbf{\xi}}\circ\mathbf{\xi})
  $$
  将每一项光度误差$r_i$一阶泰勒展开
  $$
  r_i(\delta\xi\circ\mathbf{\xi})=r_i(\mathbf{\xi})+\mathbf{J}_i\delta\xi
  $$
  最后得到的优化的位姿增量为(其中光度误差$\mathbf{r}=(r_1,\cdots,r_k)^T$,k为参与优化的点的个数)：
  $$
  \delta\mathbf{\xi}^{(n)}＝-(\mathbf{J}^T\mathbf{J})^{-1}\mathbf{J}^T\mathbf{r}(\mathbf{\xi}^{(n)})
  \quad\text{with}\quad
  \mathbf{J}=\frac{\partial{\mathbf{r}(\epsilon\circ\mathbf{\xi}^{(n)})}}{\partial\epsilon}\bigg|_{\epsilon=0}\tag{1}
  $$
  这里的雅可比矩阵用到链式求导：
  $$
  \begin{split}
  \mathbf{J}_i &= \frac{\partial{r_i(\epsilon\circ\mathbf{\xi}^{(n)})}}{\partial\epsilon}\bigg|_{\epsilon=0}\\ &= -\frac{\partial{I(\omega({\mathbf{p}_i},D_{ref}({\mathbf{p}_i}),\epsilon\circ\xi))}}{\partial{\epsilon}}\bigg|_{\epsilon=0}\\ &= -\frac{\partial{I(\mathbf{b})}}{\partial{\mathbf{b}}}\bigg|_{\mathbf{b}=\mathbf{p'}_i} \cdot \frac{\partial{\omega_n(\mathbf{q})}}{\partial{\mathbf{q}}}\bigg|_{\mathbf{q}=\mathbf{p'}_i{\cdot}z_i'} \cdot \frac{\partial{\omega_s(\epsilon\circ\mathbf{\xi}^{(n)})}}{\partial{\epsilon}}\bigg|_{\epsilon=0}\\ &= -\begin{pmatrix}dxfx&dyfy\end{pmatrix}\cdot \begin{pmatrix}1/z'&0&-x'/z'^2 \\ 0&1/z'&-y'/z'^2 \end{pmatrix} \cdot \begin{pmatrix}I|-[\mathbf{p}_i'{\cdot}z_i']_\times\end{pmatrix}\\ &= -\begin{pmatrix}1/z'{\cdot}dxfx\\1/z'{\cdot}dyfy\\-x'/z'^2{\cdot}dxfx-y'/z'^2{\cdot}dyfy\end{pmatrix}^{T} \cdot \left(\begin{array}{ccc|ccc}1&0&0&0&z'&-y'\\0&1&0&-z'&0&x'\\0&0&1&y'&-x'&0\end{array}\right)\\
  &=-\begin{pmatrix}1/z'{\cdot}dxfx \\ 1/z'{\cdot}dyfy \\ -x'/z'^2{\cdot}dxfx-y'/z'^2{\cdot}dyfy \\ -x'y'/z'^2{\cdot}dxfx-(z'^2+y'^2)/z'^2{\cdot}dyfy \\ (1+x'^2/z'^2)dxfx+x'y'/z'^2dyfy \\ x'/z'{\cdot}dyfy-y'/z'{\cdot}dxfx\end{pmatrix}^T
  \end{split}
  $$
  然后更新最小二乘问题Ax=b的系数A和b

  最后用`Eigen:ldlt`来求解这个最小二乘问题，得到新的位姿

- `calcWeightsAndResidual()`和上面计算光度误差损失一个函数

  这个函数还会计算一个Huber-weight(代码中和论文公式15不一样)
  $$
  w(r)_\delta:=\begin{cases}
  1 & \text{if}\quad |r|\le \delta \\[2ex]
  \frac{\delta}{|r|} & \text{if}\quad |r|> \delta
  \end{cases} \tag{代码实现的公式15}
  $$
  使用迭代变权重最小二乘（interatively re-weighted least-squares）可以减小外点（outliers）对算法的影响，所以1式的位姿更新量变成了：
  $$
  \delta\mathbf{\xi}^{(n)}=-(\mathbf{J}^T\mathbf{J})^{-1}\mathbf{J}^T\mathbf{W}\mathbf{r}(\mathbf{\xi}^{(n)})\tag{2}
  $$
  最后不用14讲里说的对Hession矩阵$J^TJ$求逆的方式来解，而是使用LDLT分解来求解，也就是在对优化模型求导之后把公式整理为 Ax=b 的形式，然后调用Eigen库的ldlt函数求解：
  $$
  \underbrace{(\mathbf{J}^T\mathbf{J})}_A\delta\mathbf{\xi}^{(n)}=\underbrace{-\mathbf{J}^T\mathbf{W}\mathbf{r}(\mathbf{\xi}^{(n)})}_b\tag{3}
  $$

## 六、深度地图估计

LSD-SLAM构建的是半稠密逆深度地图（semi-dense inverse depth map），只对有明显梯度的像素位置进行深度估计，用逆深度表示，并且假设逆深度服从高斯分布。

1. 判断是否是关键帧

   根据相机移动来判断的，如果相机移动了足够远，那么就重新创建一个关键帧，否则就refine当前的关键帧

2. 深度估计

   不管是创建新的关键帧还是优化当前关键帧，都需要对新的一帧进行深度估计

3. 深度估计：选择参考帧

   > We use the oldest frame the pixel was observed in, where the disparity search range and the observation angle do not exceed a certain threshold (see Fig. 4). If a disparity search is unsuccessful (i.e., no good match is found), the pixel’s “age” is increased, such that subsequent disparity searches use newer frames where the pixel is likely to be still visible.

   - 通过视察disparity来搜索最先看到同一像素的帧，一直到当前帧的前一帧作为参考帧。
   - 如果搜索失败，说明没有帧和当前帧有看到同一像素的。那么就加大这些像素的年龄，然后在新的帧里面去继续搜索。

4. 深度估计：对每一帧的每个像素进行匹配

   对于每个像素，沿着极线方向搜索匹配，如果深度估计是可用的，那么搜索范围限制到均值加减2倍方差的区间，如果不可用，那么所有的差异性都需要被搜索，并使用，最后使用SSD误差作匹配算法

   >  误差平方和算法（Sum of Squared Differences，简称SSD算法）是一种基于灰度的图像匹配算法，其他类似的算法还有平均绝对差算法（MAD）、绝对误差和算法（SAD）、平均误差平方和算法（MSD）、归一化积相关算法（NCC）、序贯相似性算法（SSDA）。

5. 深度估计：计算深度

   - 参考帧上极线的计算：几何差异误差

     由极线位置引起

   - 极线上得到最好的匹配位置：光度视差误差

     由匹配点的位置引起

   - 通过匹配位置计算出最佳的深度：逆深度计算误差

     由匹配点位置和图像间基线长度引起

6. 如果创建关键帧：

   > Once a new frame is chosen to become a keyframe, its depth map is initialized by projecting points from the previous keyframe into it, followed by one iteration of spatial regularization and outlier removal as proposed in 9. Afterwards, the depth map is scaled to have a mean inverse depth of one - this scaling factor is directly incorporated into the sim(3) camera pose. Finally, it replaces the previous keyframe and is used for tracking subsequent new frames

   新的关键帧需要之前的关键帧将点投影过来(投影方案已经在深度前传介绍过)，得到这一帧的有效点，深度通过sim(3)变换投影均值和缩放因子，最后用这个关键帧替换掉之前的关键帧

7. 如果不创建关键帧

   > A high number of very efficient small- baseline stereo comparisons is performed for image regions where the expected stereo accuracy is sufficiently large, as described in 9. The result is incorporated into the existing depth map, thereby refining it and potentially adding new pixels – this is done using the filtering approach proposed in 9.

   如果不创建关键帧，那么就用当前的观测对之前的深度进行修正

## 七、回环

在`SlamSystem::constraintSearchThreadLoop()`中会调用`SlamSystem::findConstraintsForNewKeyFrames()`来进行回环确认

### 1.  SIM3下的回环跟踪

**对比跟踪中SE3来看！**

#### 1.1 得到Ax=b

点$\mathbf{P}(p_x,p_y,d)$经过归一化后的相似变换后$\mathbf{P}'$：
$$
\mathbf{P}'=\mathbf{T}\mathbf{P}
\\
\begin{equation}
\underbrace{\mathbf{p}' = \begin{pmatrix}x'/z'\\y'/z'\\1\end{pmatrix}}_{\omega_n}
\quad\text{with}\quad
\underbrace{\begin{pmatrix}x'\\y'\\z'\\1\end{pmatrix} := \text{exp}_{\mathfrak{sim}(3)}(\mathbf\xi)\begin{pmatrix}\mathbf{p}_x/d\\\mathbf{p}_y/d\\1/d\\1\end{pmatrix}}_{w_s} \tag{1}
\end{equation}
$$
由于这里的相似矩阵 SIM3::T 比跟踪用的 SE3::T 多了个尺度项，所以代价函数变为：
$$
\begin{equation}
E(\mathbf\xi_{ji}) =\sum_{\mathbf{p}\in\Omega_{D_i}}\Biggl\|
\frac{r_p^2(\mathbf{p},\mathbf\xi_{ji})}{\sigma_{r_p(\mathbf{p},\mathbf\xi_{ji})}^2} + \frac{r_d^2(\mathbf{p},\mathbf\xi_{ji})}{\sigma_{r_d(\mathbf{p},\mathbf\xi_{ji})}^2}
\Biggr\|_\delta \tag{2}
\end{equation}
$$
2式中光度误差和方差与跟踪中一致：
$$
\begin{align}
r_p(\mathbf{p},{\mathbf\xi_{ji}}) :&= I_i({\mathbf{p}}) - I_j(\omega({\mathbf{p}}, D_i({\mathbf{p}}), \mathbf\xi_{ji}))\tag{3}\\
\sigma_{r_p(\mathbf{p},\mathbf\xi_{ji})}^2 :&= 2\sigma_I^2 + (\frac{\partial{r_p(\mathbf{p}, \mathbf\xi_{ji})}}{\partial{D_i(\mathbf{p})}})^2V_i(\mathbf{p})\tag{4}
\end{align}
$$
2式中深度残差和方差为：
$$
\begin{align}
r_d(\mathbf{p},\mathbf\xi_{ji}) &= [\mathbf{p}']_3-D_j([\mathbf{p}']_{1,2})  \tag{5}
\\
\sigma_{r_d(\mathbf{p},\mathbf\xi_{ji})}^2 &= V_j([\mathbf{p}']_{1,2})\begin{pmatrix}\frac{{\partial}r_d(\mathbf{p},\mathbf\xi_{ji})}{{\partial}D_j([\mathbf{p}']_{1,2})}\end{pmatrix} + V_i(\mathbf{p})\begin{pmatrix}\frac{{\partial}r_d(\mathbf{p},\mathbf\xi_{ji})}{{\partial}D_i(\mathbf{p})}\end{pmatrix}
\tag{6}
\end{align}
$$
和跟踪一样使用迭代变权重高斯牛顿算法，2式的优化形式为：
$$
\delta\mathbf{\xi}^{*} = \text{arg}\min_{\delta\mathbf{\xi}}\sum_{\mathbf{p}\in\Omega_{D_i}}\left(
\frac{\omega_h}{\sigma_{r_p(\mathbf{p},\mathbf\xi^{(n)})}^2} \left(
r_p(\mathbf{p},\mathbf\xi^{(n)})+\mathbf{J}_p(\xi^{(n)})\delta\mathbf{\xi}\right)^2 +\frac{\omega_h}{\sigma_{r_d(\mathbf{p},\mathbf\xi^{(n)})}^2}\left(r_d(\mathbf{p},\mathbf\xi^{(n)}+\mathbf{J}_d(\xi^{(n)})\delta\xi)\right)^2
\right) \tag{7}
$$
一阶泰勒展开对位姿扰动$\delta\mathbf{\xi}$求偏导后，可得用eigen::ldlt来求解的Ax=b：
$$
\begin{equation}
\underbrace{\left(\frac{\omega_h}{\sigma_{r_p}^2}\mathbf{J}_p^T\mathbf{J}_p + \frac{\omega_h}{\sigma_{r_d}^2}\mathbf{J}_d^T\mathbf{J}_d\right)}_{\mathbf A}\delta\xi =
\underbrace{-\left(\frac{\omega_h r_p}{\sigma_{r_p}^2}\mathbf{J}_p + \frac{\omega_h r_p}{\sigma_{r_d}^2}\mathbf{J}_d\right)}_{\mathbf b}
\end{equation} \tag{8}
$$

#### 1.2 光度误差的雅可比矩阵$\mathbf{J}_p$

$$
\begin{equation}
\begin{split}
\mathbf{J}_p &= \frac{\partial{r_i(\epsilon\circ\mathbf{\xi}^{(n)})}}{\partial\epsilon}\bigg|_{\epsilon=0}\\
&=  -\frac{\partial{I(\mathbf{b})}}{\partial{\mathbf{b}}}\bigg|_{\mathbf{b}=\mathbf{p'}_i} \cdot \frac{\partial{\omega_n(\mathbf{q})}}{\partial{\mathbf{q}}}\bigg|_{\mathbf{q}=\mathbf{p'}_i{\cdot}z_i'} \cdot \frac{\partial{\omega_s(\epsilon\circ\mathbf{\xi}^{(n)})}}{\partial{\epsilon}}\bigg|_{\epsilon=0}\\
&= -\begin{pmatrix}dxfx&dyfy\end{pmatrix}\cdot \begin{pmatrix}1/z'&0&-x'/z'^2 \\ 0&1/z'&-y'/z'^2 \end{pmatrix} \cdot \left(\begin{array}{c|c|c}I & -[\mathbf{p}_i'{\cdot}z_i']_\times & \mathbf{p}_i'{\cdot}z_i'\end{array}\right)\\
&= -\begin{pmatrix}1/z'{\cdot}dxfx\\1/z'{\cdot}dyfy\\-x'/z'^2{\cdot}dxfx-y'/z'^2{\cdot}dyfy\end{pmatrix}^{T} \cdot \left(\begin{array}{ccc|ccc|c}1&0&0&0&z'&-y'&x'\\0&1&0&-z'&0&x'&y'\\0&0&1&y'&-x'&0&z'\end{array}\right)\\
&=-\begin{pmatrix}1/z'{\cdot}dxfx \\ 1/z'{\cdot}dyfy \\ -x'/z'^2{\cdot}dxfx-y'/z'^2{\cdot}dyfy \\ -x'y'/z'^2{\cdot}dxfx-(z'^2+y'^2)/z'^2{\cdot}dyfy \\ (1+x'^2/z'^2)dxfx+x'y'/z'^2dyfy \\ x'/z'{\cdot}dyfy-y'/z'{\cdot}dxfx\\ 0\end{pmatrix}^T
\end{split} \tag{9}
\end{equation}
$$

#### 1.3 深度误差雅可比矩阵$\mathbf{J}_d$

$$
\begin{equation}
\begin{split}
\mathbf{J}_{d} &= \frac{\partial[\mathbf{p}']_3}{\partial\mathbf\epsilon}=\frac{\partial{{d(\mathbf{b})}}}{\partial\mathbf{b}}\bigg|_{\mathbf{b}=\mathbf{p'}{\cdot}z'}\cdot\frac{\partial{{\omega_s(\mathbf\epsilon\circ\mathbf\xi)}}}{\partial\mathbf\epsilon}\bigg|_{\mathbf\epsilon=0}\\
&= \begin{pmatrix}0&0&1/{z'}^2\end{pmatrix}\cdot\left(\begin{array}{c|c|c}I&-(\mathbf{p'}{\cdot}z')^\land&\mathbf{p'}{\cdot}z'\end{array}\right)\\
&= \begin{pmatrix}0&0&1/{z'}^2\end{pmatrix}\cdot\left(\begin{array}{ccc|ccc|c}1&0&0& 0&z'&-y'& x'\\ 0&1&0& -z'&0&x'& y'\\ 0&0&1& y'&-x'&0 &z'\end{array}\right)\\
&=\begin{pmatrix}0&0&1/{z'}^2&y'/{z'}^2&-x'/{z'}^2&0&1/z'\end{pmatrix}
\end{split} \tag{10}
\end{equation}
$$

### 2. 增加回环跟踪的收敛半径

在回环跟踪的时候，没法在图像对齐的时候有一个很好的初始化位置。因此论文中用了**两种增加收敛半径的方法**：

1. 使用高效二阶最小化方法（Efficient Second Order Minimization，ESM）
2. 从粗到细（Coarse-to-Fine）方法，也就是图像金字塔的方式。

## 八、图优化

使用图优化优化每一帧相对于世界坐标系的位姿



# 代码

## LSD-SLAM核心库文件

![image-20231113001505361](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/LSD-SLAM%E4%BB%A3%E7%A0%81%E7%B1%BB%E8%A1%A8.png)

## LSD-SLAM的内存管理

代码文件都在`lsd_slam_core/src/DataStructures`目录下

### 1. FrameMemory.h

用于管理内存

- 构造函数`FrameMemory()`

  通过将构造函数FrameMemory()私有化(放在private:中)保证只能创建一个实例

- `getbuffer()`和`getFloatBuffer()`

  通过调用`allocateBuffer()`申请新的内存空间

### 2. Frame.h

lsd-slam中的地图是由一系列关键帧keyframe的姿态图表示的，每个关键帧由图像地图camera image + 逆深度地图inverse depth map + 逆深度地图variance of the inverse depth组成。

这个文件就是用来构建每一帧的image, gradients, depth和depth variance pyramid

- 结构体`struct Data{}`

  定义了一系列金字塔：图像、梯度、深度、深度方差金字塔

  根据setting.h定义的宏可以看到金字塔层数PYRAMID_LEVELS为5层。

- 构造函数`Frame()`

  会调用`initialize()`来初始化图像id，内存，位姿，相机内参，相机内参的逆，各种金字塔。

  申请内存依靠的是FrameMemory.h中的`getFloatBuffer()`

- 析构函数`~Frame()`

  回收内存，依靠的是FrameMemory.h中的`returnBuffer()`

- `require()`

  这个函数先是在判断需要怎么样的数据,如果这个数据的该层金字塔还没创建,就调用相应的构建函数去构建。

  构建函数分别有：

  - `buildImage(level)`构建图像金字塔
  - `buildGradients(level)`构建梯度金字塔
  - `buildMaxGradients(level)`构建最大梯度金字塔

  - `buildIDepthAndIDepthVar(level)`构建深度和深度方差（不确定度）金字塔

- `minimizeInMemory()`

  通过调用各个release()和clear_refPixelWasGood()函数来释放上面这些函数申请的内存。

  release()中针对不同金字塔有函数：

  - `releaseImage(level)`
  - `releaseGradients(level)`
  - `releaseMaxGradients(level)`
  - `releaseIDepth(level)`
  - `releaseIDepthVar(level)`

- `setDepth()`给当前帧的金字塔第一层设置新的深度估计值
- `setDepthFromGroundTruth()`把真实的深度设置给当前帧的金字塔

- `prepareForStereoWith()`一些相似矩阵的预运算

### 3. FramePoseStruct.h

- `getCamToWorld()`

  得到当前帧的绝对位姿：不停的递归相对位姿，直到得到当前帧相对于世界坐标系的位姿，即绝对位姿

## LSD-SLAM的跟踪

### 0. 整体结构

在SE3Tracker.h的`trackFrame()`函数中实现：

```
图像金字塔迭代level-4到level-1
Step1: 对参考帧当前层构造点云(reference->makePointCloud)
Step2: 计算变换到当前帧的残差和梯度(calcResidualAndBuffers)
Step3: 计算法方差归一化残差(calcWeightsAndResidual)
Step4: 计算雅克比向量以及A和b(calculateWarpUpdate)
Step5: 计算得到收敛的delta，并且更新SE3(inc = A.ldlt().solve(b))
Step6: 重复Step2-Step5直到收敛或者达到最大迭代次数
计算下一层金字塔
```

### 1. TrackingReference.h

这个类主要用于管理在tracking时用到的参考帧

- `releaseAll()`

  释放与当前参考帧有关的所有资源

- `importFrame()`

  加载帧

- `makePointCloud()`

  对参考帧某一层(level)构建点云，计算了每个像素的3D空间坐标，像素梯度，颜色和方差
### 2. SE3Tracker.h

Tracking线程的主题部分：求解两帧之间的SE3变换

- `SE3Tracker()`

  **构造函数，读取相机内参，给各个变量分配内存**

 - `checkPermaRefOverlap()`

   **检查当前帧与参考帧之间的参考点的重叠度**

 - `calcResidualAndBuffers()`

   **论文公式13：计算参考点在当前帧下投影点的残差(光度误差)和梯度，并记录参考点在参考帧的逆深度和方差**

   - `calcResidualAndBuffers_debugStart()`

     将4个图像的每个像素都设为白色。
     
   - `calcResidualAndBuffers_debugFinish()`

     结束残差计算，如果save/plot为true，就保存/可视化计算的光度误差

 - `calcWeightsAndResidual()`

   **计算光度误差损失函数**(论文公式12)
   $$
   E_p(\mathbf\xi_{ji}) =\sum_{\mathbf{p}\in\Omega_{D_i}}
   \Biggl\|\frac{r_p^2(\mathbf{p},\mathbf\xi_{ji})}{\sigma_{r_p(\mathbf{p},\mathbf\xi_{ji})}^2}\Biggr\|_\delta \tag{论文12式}
   $$
   计算归一化方差的光度误差系数(论文公式14) 和 Huber-weight(论文公式15)
   $$
   \sigma_{r_p(\mathbf{p},\mathbf\xi_{ji})}^2 := 2\sigma_I^2 + (\frac{\partial{r_p(\mathbf{p}, \mathbf\xi_{ji})}}{\partial{D_i(\mathbf{p})}})^2V_i(\mathbf{p}) \tag{论文14式}
   $$
   因为两帧之间位姿变换小所以作者只考虑了位移没考虑旋转，所以14式的梯度项简化为：
   $$
   \begin{split}
   \frac{\partial{r_p(\mathbf{p},\mathbf\xi_{ji})}}{\partial{D_i(\mathbf{p})}}
   &= \frac{\partial({I_i({\mathbf p}) - I_j(\omega({\mathbf p}, D_i({\mathbf p}), \xi_{ji})))}}{\partial{D_i(\mathbf{p})}} \\
   &= - \frac{\partial{I_j(\mathbf{a})}}{\partial{\mathbf{a}}}\bigg|_{\mathbf{a}=\mathbf{p}} \cdot\frac{\partial{w(d)}}{\partial{d}}\bigg|_{d=D_i(\mathbf{p})} \\
   &= -\begin{pmatrix}dxfx&dyfy\end{pmatrix}\cdot\begin{pmatrix}\frac{\mathbf{t}_x(1/d+\mathbf{t}_z)-\mathbf{t}_z(\mathbf{p}_x/d+\mathbf{t}_x)}{(1/d+\mathbf{t}_z)^2d} \\
   \frac{\mathbf{t}_y(1/d+\mathbf{t}_z)-\mathbf{t}_z(\mathbf{p}_y/d+\mathbf{t}_y)}{(1/d+\mathbf{t}_z)^2d}\end{pmatrix}\\
   &= -(dxfx\frac{\mathbf{t}_xz'-\mathbf{t}_zx'}{z'^2d} + dyfy\frac{\mathbf{t}_yz'-\mathbf{t}_zy'}{z'^2d})
   \end{split} 
   $$
   
- `calculateWarpUpdate()`

  **计算公式12的雅可比以及最小二乘法，最后更新得到新的位姿变换SE3**

  12式的分母被当成了一个系数不参与求导，所以只有分子的光度误差参与求导，根据高斯牛顿算法并采用左乘扰动后，12式优化的形式：
  $$
  \delta\mathbf{\xi}^{*}=\text{arg}\min_{\delta\mathbf{\xi}}E_p(\delta\mathbf{\xi}\circ\mathbf{\xi})=\text{arg}\min_{\delta\mathbf{\xi}}\sum_i{r_i^2(\delta\mathbf{\xi}}\circ\mathbf{\xi})
  $$
  将光度误差$r_i$一阶泰勒展开后代入上式可得：
  $$
  \delta\mathbf{\xi}^{(n)}＝-(\mathbf{J}^T\mathbf{J})^{-1}\mathbf{J}^T\mathbf{r}(\mathbf{\xi}^{(n)})
  \quad\text{with}\quad
  \mathbf{J}=\frac{\partial{\mathbf{r}(\epsilon\circ\mathbf{\xi}^{(n)})}}{\partial\epsilon}\bigg|_{\epsilon=0}
  $$
  这里的雅可比矩阵用到链式求导：
  $$
  \begin{split}
  \mathbf{J}_i &= \frac{\partial{r_i(\epsilon\circ\mathbf{\xi}^{(n)})}}{\partial\epsilon}\bigg|_{\epsilon=0}\\ &= -\frac{\partial{I(\omega({\mathbf{p}_i},D_{ref}({\mathbf{p}_i}),\epsilon\circ\xi))}}{\partial{\epsilon}}\bigg|_{\epsilon=0}\\ &= -\frac{\partial{I(\mathbf{b})}}{\partial{\mathbf{b}}}\bigg|_{\mathbf{b}=\mathbf{p'}_i} \cdot \frac{\partial{\omega_n(\mathbf{q})}}{\partial{\mathbf{q}}}\bigg|_{\mathbf{q}=\mathbf{p'}_i{\cdot}z_i'} \cdot \frac{\partial{\omega_s(\epsilon\circ\mathbf{\xi}^{(n)})}}{\partial{\epsilon}}\bigg|_{\epsilon=0}\\ &= -\begin{pmatrix}dxfx&dyfy\end{pmatrix}\cdot \begin{pmatrix}1/z'&0&-x'/z'^2 \\ 0&1/z'&-y'/z'^2 \end{pmatrix} \cdot \begin{pmatrix}I|-[\mathbf{p}_i'{\cdot}z_i']_\times\end{pmatrix}\\ &= -\begin{pmatrix}1/z'{\cdot}dxfx\\1/z'{\cdot}dyfy\\-x'/z'^2{\cdot}dxfx-y'/z'^2{\cdot}dyfy\end{pmatrix}^{T} \cdot \left(\begin{array}{ccc|ccc}1&0&0&0&z'&-y'\\0&1&0&-z'&0&x'\\0&0&1&y'&-x'&0\end{array}\right)\\
  &=-\begin{pmatrix}1/z'{\cdot}dxfx \\ 1/z'{\cdot}dyfy \\ -x'/z'^2{\cdot}dxfx-y'/z'^2{\cdot}dyfy \\ -x'y'/z'^2{\cdot}dxfx-(z'^2+y'^2)/z'^2{\cdot}dyfy \\ (1+x'^2/z'^2)dxfx+x'y'/z'^2dyfy \\ x'/z'{\cdot}dyfy-y'/z'{\cdot}dxfx\end{pmatrix}^T
  \end{split}
  $$
  然后更新最小二乘问题Ax=b的系数A和b

  最后用`Eigen:ldlt`来求解这个最小二乘问题，得到新的位姿

- `trackFrame()`

  

## LSD-SLAM的地图优化

### 1. g2oTypeSim3Sophus.h

本文件主要是用Sophus::Sim3d作为顶点(位姿)的数据结构，构建g2o的顶点和边

- `class VertexSim3 : public g2o::BaseVertex<7, Sophus::Sim3d>`

  优化的顶点，每个顶点都是Sim(3)三维相似变换矩阵

- `class EdgeSim3 : public g2o::BaseBinaryEdge<7, Sophus::Sim3d, VertexSim3, VertexSim3>`

  优化的边

### 2. TrackableKeyFrameSearch.h

本文件主要用于查找当前keyframe可以跟踪到的其他keyframe，以便将新约束放入图优化中

- `TrackableKeyFrameSearch()`构造函数

  创建SE3Tracker的实例

- `findCandidates()`

  寻找当前关键帧可以追踪到的其他关键帧(作为参考帧)

- `findRePositionCandidate()`

  寻找当前关键帧可以追踪到的其他关键帧，并计算该关键帧的分数，如果不够高重新计算该关键帧的位姿

- `getRefFrameScore()`

  得到参考帧的分数

- `findEuclideanOverlapFrames()`

  用来寻找某一个关键帧所有潜在的参考帧

### 3. KeyFrameGraph.h

这个类用于创建g2o需要使用的图

- `KFConstraintStruct()`结构体构造函数

  这个结构体主要包含图优化用到的变亮

- `KeyFrameGraph()`类构造函数

  构建一个新的g2o位姿图

- `addKeyFrame()`

  将新的关键帧添加到位姿图中去

- `dumpMap()`

  将关键帧图优化的结果以及相关的信息保存到指定的文件夹中

- `insertConstraint()`

  向关键帧图中添加一个新的约束（constraint），这个约束用 EdgeSim3 类表示

- `optimize()`

  执行图优化，只会更新顶点的位姿，关键帧的位姿没有更新，需要优化完后再调用`updateKeyFramePoses()`

- `calculateGraphDistancesToFrame()`

  计算一个给定的帧到图中所有其他帧的最短距离，并保存在distanceMap中
