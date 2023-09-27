# 学习资料汇总

- TUM 课
- [CMU 课件](http://www.cs.cmu.edu/~16385/s17/)

# 零、一些常忘记的概念

- **物平面、归一化平面、像素平面、成像平面的区别**

  ![在这里插入图片描述](https://cdn.jsdelivr.net/gh/Fernweh-yang/ImageHosting@main/img/%E9%92%88%E5%AD%94%E7%9B%B8%E6%9C%BA%E6%A8%A1%E5%9E%8B.png)

  - 物平面：

    - 基于世界坐标系，位于相机前方是相机前方的一个平面。
    - 用于描述物体的位置

  - 归一化平面：

    - 基于相机坐标系，位于相机前方Z=1的一个平面
    - 用于消除深度的影响，单位是米（X，Y，1）

    > **为什么归一化平面和成像平面在相机前面？**
    >
    > - 看图5-1可以发现实际中除了物平面其他平面，都应该该相机后面。但这样构建的相似三角形是倒立的，计算公式因此带有负号
    >
    > - 为了简化模型和计算，可以把归一化平面和成像平面都假象到相机前面。
    >
    >   所以后面的对称平面和归一化平面其实都是虚拟的平面

  - 成像平面：

    - 基于相机坐标系，是光线经过物镜或透镜后会聚成像的平面。

      成像平面是一个相对概念，每个平面都可以称之为成像平面。真实成像平面就是上面图5-1中的物理成像平面。

      ![在这里插入图片描述](https://cdn.jsdelivr.net/gh/Fernweh-yang/ImageHosting@main/img/%E6%88%90%E5%83%8F%E5%B9%B3%E9%9D%A2.png)

    - 归一化平面的深度Z乘以焦距f，就是对称成像平面了

    - 用于描述图像中像素的位置，这时单位还是米（X，Y，Z）

  - 像素平面：

    - 基于相机坐标系，位于相机内部图像传感器所在的平面

      > 图像传感器：将光能转换为电荷。由感光元件、时序控制电路和模数转换电路组成。
      >
      > 有CCD/CMOS两种类型

    - 成像平面经由内参K(像素大小，偏移)就可以转换为像素平面了

      - 像素大小$\alpha,\beta$：像素平面和成像平面之间的缩放关系
      - 偏移$c_x,c_y$：像素平面和成像平面原点的偏移
      - 这些都是相机的物理性质

    - 用于描述图像中像素位置，单位是像素（u,v）

- **世界坐标系，相机坐标系，图像物理坐标系，像素坐标系**

  ![img](https://cdn.jsdelivr.net/gh/Fernweh-yang/ImageHosting@main/img/%E5%9B%9B%E5%A4%A7%E5%9D%90%E6%A0%87%E7%B3%BB.webp)

  - 世界坐标系：根据情况而定，可以表示任何物体，此时是由于相机而引入的。单位m。
  - 相机坐标系：以摄像机光心为原点（在针孔模型中也就是针孔为关心），z轴与光轴重合也就是z轴指向相机的前方（也就是与成像平面垂直），x轴与y轴的正方向与物体坐标系平行，其中上图中的f为摄像机的焦距。单位m
  - 图像物理坐标系（也叫平面坐标系）：用物理单位表示像素的位置，坐标原点为摄像机光轴与图像物理坐标系的交点位置。坐标系为图上o-xy。单位是mm。单位毫米的原因是此时由于相机内部的CCD传感器是很小的，比如8mm x 6mm。但是最后图像照片是也像素为单位比如640x480.这就涉及到了图像物理坐标系与像素坐标系的变换了。下面的像素坐标系将会讲到。
  - 像素坐标系：以像素为单位，坐标原点在左上角。这也是一些opencv，OpenGL等库的坐标原点选在左上角的原因。当然明显看出CCD传感器以mm单位到像素中间有转换的。举个例子，CCD传感上上面的8mm x 6mm，转换到像素大小是640x480. 假如dx表示像素坐标系中每个像素的物理大小就是1/80. 也就是说毫米与像素点的之间关系是piexl/mm.

# 一、Wissenswertes über Bilder

## 1. Darstellung von Bildern

- Image Representation

  1. **Continuouse Representation** (Kontinuierliche Darstellung)

     As a function of two variables(to derive algorithms)
     $$
     I:R^2\supset\Omega\rightarrow R,\ (x,y)\rightarrow I(x,y)
     $$

     - $I$ is differentiable(differenzierbar)灰度值
       - 灰度图：用八位整数记录，即0-255
       - 深度图：用16位整数记录，即0-65535
       - 彩色图：多通道,如rgb等
     - $\Omega$ is simply connected and bounded
     - 这里x轴向右，y轴向下，所以计算机中读取某一点的像素应该是：$image[y][x]$

  2. **Discrete Representation**(Diskrete Darstellung)

      as a Matrix $I\in R^{m\times n}$ the Item $I_{k,l}$ corresponds to the Intensity value强度。

     The scale is typically between [0,255] or [0,1]

- Discrete Sampling

  1. Sampling of a one-dimensional signal
     $$
     S\{f(x)\}=(...,f(x-1),f(x),f(x+1),...)
     $$
  
  2. Sampling of an image
     $$
     S\{I(x,y)\}=
      \left[
      \begin{matrix}
        \ddots & \vdots &\vdots &\vdots & \\
        \cdots & I(x-1,y-1) & I(x-1,y) & I(x-1,y+1)&\cdots \\
        \cdots & I(x,y-1) & I(x,y) & I(x,y+1)&\cdots \\
        \cdots & I(x+1,y-1) & I(x+1,y) & I(x+1,y+1)&\cdots \\
         & \vdots &\vdots &\vdots & \ddots \\
       \end{matrix}
       \right]
     $$
  
     - Assumption: Origin is in the top-left corner
  
     

## 2. Bildgradient

image gradient 是一个用于determine确定local intensity changes的重要工具

- Edges(Kanten)边界

  - Edges correspond to stark local changes of the intensity.

  - Local changes are described by a gradient
    $$
    \nabla I(x,y)= \left[
     \begin{matrix}
       \frac{d}{dx}I(x,y) \\
       \frac{d}{dy}I(x,y)
      \end{matrix}
      \right]
    $$

    - $I \in R^{m\times n}$ is known
    - Naive approach to estimate the gradient
      - $\frac{d}{dx}I(x,y)\approx I(x+1,y)-I(x,y)$
      - $\frac{d}{dy}I(x,y)\approx I(x,y+1)-I(x,y)$

- Interpolation内插法
22ws考试考了这些公式

  从discrete signal $f[x]=S\{f(x)\}$到 continuous signal $f(x)$

  使用Sample Value(Abtastwerte) 和 Interpolationsfilter 的Faltung卷积来求得：
  $$
  f(x) \approx \sum_{k=-\infty}^\infty f[k]h(x-k)=: f[x]*h(x)
  $$

  - Interpolation filter
    1. Gaussian filter: $h(x)=g(x)=Ce^{\frac{-x^2}{2\sigma^2}}$
    2. Ideales Interpolationsfilter: $h(x)=sinc(x)=\frac{sin(\pi x)}{\pi x},sinc(0)=1$

- Discrete Derivative

  Discrete Derivative离散导数是通过对interpolated signal内插信号求导计算而得到的：

  算法步骤Algorithmically:

  1. Reconstruction重构of the continuous Signal
  2. Differentiation of the continuous signal
  3. Sampling of the Derivative导数

  Derivation:
  $$
  \begin{aligned}
  f^{\prime}(x)&\approx\frac{d}{dx}(f[x]*h(x))\\
  &=f[x]*h^{\prime}(x) \\
  \\
  f^{\prime}[x]&=f[x]*h^{\prime}[x]\\
  &=\sum_kf[x-k]h^{\prime}[k]
  \end{aligned}
  $$

  - 2D-Rekonstruktion
    $$
    I(x,y)\approx I[x,y]*h(x,y)=\sum_{k=-\infty}^{\infty}\sum_{l=-\infty}^\infty I[k,l]g(x-k)g(y-l)\\
    h(x,y):=g(x)g(y)\  \text{这里用separable 2D Gaussian filter}
    $$

    - 在实际中会用一数n来代替无穷的求和。

    - 高斯过滤器的C 的选择：
      为什么要这么选择C？

      因为Normalization(Normierung)正则化C，使得所有的gewichte权重相加趋向1
      $$
      C=\frac{1}{\sum_{k=-n}^ne^{\frac{-k^2}{2\sigma^2}}}
      $$

  - 2-D Derivative

    利用高斯过滤的Separability可分性来计算梯度：

    Ableitung in X-Richtung
    $$
    \begin{aligned}
    \frac{d}{dx}I(x,y)&\approx I[x,y]*(\frac{d}{dx}h(x,y))\\
    &=\sum_{k,l}I[k,l]g^{\prime}(x-k)g(y-l)\\
    S\{\frac{d}{dx}I(x,y)\}&=I[x,y]*g^{\prime}[x]*g[y]\\
    &=\sum_{k,l}I[x-k,y-l]g^{\prime}[k]g[l]
    \end{aligned}
    $$

  - Sobel-Filter索伯滤波器

    索贝尔算子（Sobeloperator）主要用作边缘检测，在技术上，它是一离散性差分算子，用来运算图像亮度函数的灰度之近似值。在图像的任何一点使用此算子，将会产生对应的灰度矢量或是其法矢量。

    Sobel Filter是integer整数approximations of the double gradient.

    - 该算子由2个3X3矩阵g[x],g[y]组成。

      -  g[x]用于检测图像中垂直方向的边缘
    
      -  g[y]用于检测图像中水平方向的边缘
    
    
      | g[x] |      |      |      | g[y] |      |      |      |
      | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
      |      | -1   | 0    | 1    |      | 1    | 2    | 1    |
      |      | -2   | 0    | 2    |      | 0    | 0    | 0    |
      |      | -1   | 0    | 1    |      | -1   | -2   | -1   |
    
      如果I代表原始图像，那么Gx,Gy就分别代表经纵向，横向边缘检测的图像灰度值
    
      $G_x=g[x]*I$; $G_y=g[y]*I$      这里*是卷积的意思
    
      该点的灰度大小为：$G=\sqrt{G_x^2+G_y^2}$
    
    - Approxiamtion von $S\{\frac{d}{dx}I(x,y)\}=I[x,y]*g^{\prime}[x]*g[y]=\sum_{k=-\infty}^\infty \sum_{l=-\infty}^\infty I[x-k,y-l]g^{\prime}[k]g[l]$
    
      durch endliche Summe$\sum_{k=-1,0,1}\sum_{l=-1,0,1}I[x-k,y-l]g^{\prime}[k]g[l]$
    
    - Normierungsfaktor$C=\frac{1}{1+2e^{-\frac{1}{2\sigma^2}}}$
    
      

## 3.Merkmalspunkte-Ecken und Kanten

特征点Feature points – 角Corners and 边缘Edges

哈里斯角点检测的基本思想：

算法基本思想是使用一个固定窗口在图像上进行任意方向上的滑动，比较滑动前与滑动后两种情况，窗口中的像素灰度变化程度，如果存在任意方向上的滑动，都有着较大灰度变化，那么我们可以认为该窗口中存在角点。

- Harris Corner and Edge Detector
  - Corner: shifts移动 in all directions cause a change
  - Edge: shifts in all directions except exactly one cause a change
  - Homogenous surface均匀的平面: no change, independently of the direction不管在哪个方向都没变化  

- Formal Definition of change

  - Position in the image: $x=\left[\begin{matrix}x_1\\x_2\end{matrix}\right]$，$I(x)=I(x_1,x_2)$

  - Shift direction $u=\left[\begin{matrix}u_1\\u_2\end{matrix}\right]$

    - Shift移动 according to vector u

  - Change in the image segment段：当窗口发生u移动时，那么滑动前与滑动后对应的窗口中的像素点灰度变化描述如下：

    $S(u)=\displaystyle\int_W(I(x+u)-I(x))^2dx$

    也可写成：$S(u_1,u_2)=\displaystyle\sum_{x_1,x_2}w(x_1,x_2)[I(x_1+u_1,x_2+u_2)-I(x_1,x_2)]^2$。这里的W是window function窗口函数，也即我们选中的segment。

    后面3步是在化简该式

  - Differentiability可微性 of $I$:
    $$
    \displaystyle\lim_{u\rightarrow0}\frac{I(x+u)-I(x)-\nabla I(x)^Tu}{||u||}=0
    $$

    - Approximation for small shifts: $I(x+u)-I(x)\approx\nabla I(x)^Tu$

  - Approximation of the change in the image segment
    $$
    S(u)=\displaystyle\int_W(I(x+u)-I(x))^2dx\approx\displaystyle\int_W(\nabla I(x)^Tu)^2dx=u^T(\displaystyle\int_W\nabla I(x)\nabla I(x)^Tdx)u
    $$

  - 由上式可得Harris matrix: $G(x)=\displaystyle\int_W\nabla I(x)\nabla I(x)^Tdx$
    $$
    \nabla I(x)\nabla I(x)^T=
     \left[
     \begin{matrix}
       (\frac{\partial}{\partial x_1}I(x))^2 & \frac{\partial}{\partial x_1}I(x)\frac{\partial}{\partial x_2}I(x) \\
       \frac{\partial}{\partial x_2}I(x)\frac{\partial}{\partial x_1}I(x) & (\frac{\partial}{\partial x_2}I(x))^2  \\
      \end{matrix}
      \right]
    $$
    由此可改写Image segment为：$S(u)\approx u^TG(x)u=[u_1\ u_2](\sum \left[
     \begin{matrix}
       I_{x_1}^2 & I_{x_1}I_{x_2} \\
       I_{x_1}I_{x_2} & I_{x_2}^2  \\
      \end{matrix}
      \right])\left[\begin{matrix}u_1\\u_2\end{matrix}\right]$

- Excursus附记: Linear Algebra

  - Real valued symmetrical Matrix对称矩阵：$A=A^T$

    - positive definite正定的：$x^TAx>0,x\neq0$
    - positive semi-definite 半正定的：$x^TAx\geq0$

  - Some Examples

    - Zero-Matrix is positive semi-definite but not positive definite
    - All positive definite matrices are also positive semi-definite.
    - The identity matrix单位矩阵 is positive definite
    - Harris matrix is positive semi-definite

  - Eigenvalue decomposition分解 of real-valued symmetrical Matrices

    All real-valued symmetrical matrices $A=A^T$ can be decomposed分解 into a product $A=V\Lambda V^T$ with $VV^T=I$ and a diagonal matrix对角矩阵$\Lambda$, wherein the eigenvalues of $A$ lie on the diagonal. The columns of $V$ are the corresponding eigenvectors.

  - 如果$\lambda_1...\lambda_n$是矩阵G的特征值，那么：

    - $det\ G=\displaystyle\prod_{i=1}^{n}\lambda_i$ (determinant行列式  is the product of the eigenvalues)
    - $tr\ G=\displaystyle\sum_{i=1}^n\lambda_i$(The trace迹(spur) is the sum of the eigenvalues )

- Eigenvalue decomposition：

  - Eigenvalue decomposition分解 of Harris matrix
    $$
    G(x)=\displaystyle\int_W\nabla I(x)\nabla I(x)^Tdx=V \left[
     \begin{matrix}
       \lambda_1 &  \\
        & \lambda_2  \\
      \end{matrix}
      \right]V^T\\
      with\ VV^T=I_2 \ and\ eigenvalues\ \lambda_1\geq\lambda_2\geq0
    $$

  - Change is dependent on the eigenvectors: $V=[v_1,v_2]$
    $$
    S(u)\approx u^TG(x)u=\lambda_1(u^Tv_1)^2+\lambda_2(u^Tv_2)^2
    $$
    有3种情况：

    1. Both eigenvalues positive
       - $S(u)>0$ for all $u$ (change in every direction)
       - Examinated image section被研究的图片部分 contains a corner
    2. One eigenvalue positive, one eigenvalue equal to zero
       - $S(u)\begin{cases}=0,\ falls\ u=rv_2\ \text{(no change, only in the direction of the eigenvector with eigenvalue 0)}\\>0,\ sonst\end{cases}$
       - Examinated image section contains an edge
    3. Both eigenvalues equal to 0
       - $S(u)=0$for all u (no change in any direction)
       - Examinated image section contains a homogenous surface

    但在现实中由于噪声、离散取样或者数值上的不准确，导致Eigenvalues永远不可能是0所以：

    1. Corner: two large eigenvalue
    2. Edge: a large eigenvalue and a small eigenvalue
    3. Homogeneous surface: two small eigenvalues

- Practical Implementation of the Harris detector

  - Computation of the Harris matrix

    - Approximate G(x) with a finite sum
      $$
      G(x)=\displaystyle\int_W\nabla I(x)\nabla I(x)^Tdx\approx \displaystyle\sum_{\tilde{x}\in W(x)}\nabla I(\tilde{x})\nabla I(\tilde{x})^T
      $$

    - Weighted Sum dependent on the position of $\tilde{x}$
      $$
      G(x)\approx\displaystyle\sum_{\tilde{x}\in W(x)}w(\tilde{x})\nabla I(\tilde{x})\nabla I(\tilde{x})^T
      $$

    - Weights $w(\tilde{x})>0$ emphasize the influence of the central pixel

  - A simple criterion标准 for corners and edges

    - Analyze the quantity$H:=det(G)-k(tr(G))^2$

      $H=(1-2k)\lambda_1\lambda_2-k(\lambda_1^2+\lambda_2^2)$

    - Corner: H larger than a positive threshold value.$0<\tau_+<H$

    - Edge: H smaller than a negative threshold value.$H<\tau_-<0$

    - Homogeneous surface: H small. $\tau_-<H<\tau_+$

## 4.Korrespondenzschätzung für Merkmalspunkte

Correspondence Estimation of Feature Points特征点的对应估计

- 问题描述

  - Two Image $I_1:\Omega_1\rightarrow R,I_2:\Omega_2\rightarrow R$ from the same 3D scene are known
  - Find pairs of image points$(x^{(i)},y^{(i)})\in \Omega_1\times\Omega_2$ which correspond to the same 3D points
  - Feature points$\{x_1,...x_n\}\subset\Omega_1$and $\{y_1,...y_n\}\subset\Omega_2$ are known

- Naive Solution to the Problem

  **方法1、Sum of squared differences(SSD)**

  - Examine image sections $V_i$ around $x_i$ and $W_i$ around $y_i$ in matrix representation and compare the respective intensity values.

    - 这里$x_i,y_i$是2张图片各自的特征点不是横纵坐标的意思

  - Formal Definition

    - A criterion判断标准：$d(V,W)=||V-W||^2_F$

      - 这里$||A||^2_F=\displaystyle\sum_{kl}A^2_{kl}=trace(A*A)$描述了quadratic二次的 Frobenius norm范数

    - 找一个对于$V_j$合适的$W_j$来使$j=arg\ \displaystyle\min_{k=1,..,n}d(V_i,W_k)$. 反过来找V也OK.
  
  - SSD方法的缺点：
  
    Change in Illumination光亮 or Rotation。为了克服它需要Normalization正则化亮度和旋转
  
    - **Rotation Normalization**: By means of the Gradient Direction
  
      Pre-processing:
  
      1. Determine the gradient in all feature points.
      2. Rotate the regions around feature points旋转2个图片中的1个 such that the gradient points in one direction.
      3. Extrapolate推断 V,W from the rotated regions.
  
    - Bias and Gain Model:
  
      - $\alpha$：Scaling缩放 of the intensity values(Gain)。
  
        $\beta$：Shift 移动 of the intensity (Bias)。
  
        Gain Model: $W\approx\alpha V$ 用于控制对比度contrast
  
        Bias Model: $W\approx V+\beta 1 1^T$ 用于控制亮度brightness
  
        - 这里$1=(1,...,1)^T$
  
        **Bias and Gain Model**: $W\approx \alpha V+\beta11^T$
      
      1. Calculate the intensity mean平均值
      
      $$
      \begin{aligned}
      \bar{W}&=\frac{1}{N}(11^TW11^T)\\
      &\approx \frac{1}{N}(11^T(\alpha V+\beta11^T)11^T)\\
      &=\alpha\frac{1}{N}(11^TV11^T)+\beta11^T\\
      &=\alpha\bar{V}+\beta11^T
      \end{aligned}
      $$
      
      2. Subtract the mean-matrix
      
      $$
      \begin{aligned}
      W-\bar{W}&\approx \alpha V+\beta 11^T-(\alpha\bar{V}+\beta11^T)\\
      &=\alpha(V-\bar{V})
      \end{aligned}
      $$
      
      3. Standard Deviation标准差 of the intensity
      
      $$
      \begin{aligned}
      \sigma(W)&=\sqrt{\frac{1}{N-1}||W-\bar{W}||^2_F}\\
      &=\sqrt{\frac{1}{N-1}tr((W-\bar{W})^T(W-\bar{W}))}\\
      &\approx \sqrt{\frac{1}{N-1}tr(\alpha(V-\bar{V})^T\alpha(V-\bar{V}))}\\
      &=\alpha\sigma(V)
      \end{aligned}
      $$
      
      4. Normalization正则化 of the image sections
         $$
         \begin{aligned}
         W_n:&=\frac{1}{\sigma(W)}(W-\bar{W})\\
         &\approx \frac{1}{\alpha\sigma(V)}(\alpha(V-\bar{V}))\\
         &=\frac{1}{\sigma(V)}(V-\bar{V})\\
         &=:V_n
         \end{aligned}
         $$
      
         - 在标准化后的图片片段里，强度值Intensitätswerte的标准差为1
         - 且强度值为实数reell
    
    **方法2、Normalized Cross Correlation(NCC)**归一化互相关
    
    Derivation起源于SSD，同样需要进行上面那4步窗口正则化**参考作业2.1**
  
    - SSD方法：SSD of two normalized image sections:$d(V,W)=||V_N-W_N||^2_F=2(N-1)-2tr(W_n^TV_n)$
      - 这里的**N**是窗口内所有的像素个数：即`window_length*window_length`
    
    - NCC方法：The Normalized Cross Correlation of th two image sections is defined as $NCC=\frac{1}{N-1}tr(W_n^TV_n)$
      - 注意这里和上面公式中$W_n和V_n$的顺序，
    - $-1\leq NCC\leq1$，相关度超出阈值的部分要舍弃(=0)
    - 两个normalized image section是相似的,如果
      1. SSD small (few differences)
      2. NCC close to =1 (high correlation)
  
## 5.找特征点的整体流程：

1. 用两个相机拍照（第三章）
2. Konvertierung der Farbbilder in ein Grauwert-/Intensitäts-Bild
3. Lokalisierung von Merkmalspunkten mit dem Harris-Detektor
4. Ausschneiden剪切 und normieren标准化 von Bildsegmenten图像片段 um die Merkmalspunkte herum
5. Schätzung估计 von Korrespondenzen durch die Normalized Cross Correlation (NCC) der Bildsegmente untereinander

  

# 二、Bildentstehung

## 1.Bildentstehung

Image Formation成像

- The Pinhole Camera Model(Lochkameramodell)

  对于lens透镜：
  $$
  |\frac{b}{B}|=\frac{z-f}{f}\\
  |\frac{z}{Z}|=|\frac{b}{B}|\\
  \frac{1}{|Z|}+\frac{1}{z}=\frac{1}{f}
  $$
  其中：

  - $f$:焦距，焦点到凸透镜中心的距离。
    - 焦点：平行于主光轴的光，经透镜折射后与主光轴相交的点
    - 弥散圆circle of confusion:在焦点前后，光线会扩散为一个圆，成像变得模糊
    - 焦深：离焦点一定范围内的弥散圆还是让人清晰可见的，这段离焦点的距离称之为景深。显然景深包含焦点的左右两侧，靠近透镜那一侧称之为前景深，远离透镜那一侧称之为后景深
    - 景深：焦深对应于像，而景深对应于物体。
  - $z$:像距，成像平面到凸透镜的距离。f<=z<=2f
    - 像距受物距、焦距而影响
  - $Z$:物距，物体到凸透镜的距离
  - $b$:像的大小。$B$:物的大小

- Projection of a point in Space onto the Focal Plane成像平面

  - 小孔成像实际就是将**相机坐标系**中的三维点变换到成像平面中的**图形坐标系**
  
    - 相机坐标系(三维)
  
      相机的中心被称为焦点或者广信，以焦点O为原点和坐标X,Y,Z组成了相机坐标系。
  
    - 图像坐标系(二维)
  
      成像平面focal plane中，以成像平面的中心O'为原点和坐标轴x',y'组成了图像坐标系
  
      - 注意不是像平面，虽然实际中光线经过透镜后并不会完美的都交于焦点，而会形成弥散圆。
      - [相机成像究竟是成在像平面还是成在焦平面？是什么是像平面 和 焦平面？](https://www.zhihu.com/question/33793912/answer/57646234)
      
    - 针孔相机Lochkamera的焦平面和像平面为什么zusammenfallen同时发生？
  
      Auf Grund kleinen Öffnung der Kamera kann angenommen werden, dass die Strahlen, die in die Kamera einfallen, annähernd parallel sind. Somit entspricht die Brennebene der Bildebene.这两点是理想中小孔相机才能达到的
  
      
      
      Alle Strahlen, die vor der Linse parallel verlaufen treffen sich hinter der Linse in der Brennebene.
      
      Alle Strahlen, die von einem Punkt vor der Linse ausgehen, treffen sich hinter der Linse in der Bildebene
  
  - 如果空间点P在相机坐标系中的坐标是$P_c=[X,Y,Z]^T$ ，其像点在图像坐标系中的坐标是$p=[x,y]^T$,因为光轴和成像平面垂直，所以像点p在相机坐标系中的坐标是$p=[x,y,z]^T$，z=f(相机焦距)
  
    根据三角形相似关系可得：
    $$
    \begin{cases}
    x=f\frac{X}{Z}\\
    y=f\frac{Y}{Z}\\
    z=f
    \end{cases}\tag{1}
    $$
    可以记作从3d->2d坐标转换时,坐标都乘了$\frac{f}{Z}$

## 2.Homogene Koordinaten

Homogeneous Coordinates齐次坐标

下面用齐次坐标来表述1中Bildebene.

上面的1式描述了3D空间到2D平面的映射，但该映射对于坐标Z来说是非线性的(因为Z是分母！=0)。所以为了方便的同意处理X,Y,Z三个坐标轴的数据，就需要引入新的坐标（扩展坐标的维度）将Z线性化：
$$
\begin{bmatrix}
x\\y
\end{bmatrix}
\Leftarrow\Rightarrow
\begin{bmatrix}
\hat{x}\\
\hat{y}\\
\hat{z}
\end{bmatrix}=
\begin{bmatrix}
f &0&0&0\\
0&f&0&0\\
0&0&1&0
\end{bmatrix}
\begin{bmatrix}
X\\
Y\\
Z\\
1
\end{bmatrix}
$$
坐标$(\hat{x},\hat{y},\hat{z})$就是像点$p=(x,y)$的齐次坐标,其中
$$
\begin{cases}
x=\frac{\hat{x}}{\hat{z}}\\
y=\frac{\hat{y}}{\hat{z}}\\
\hat{z}\neq0
\end{cases}
$$
可见通过扩展坐标维度构建其次坐标的步骤就是：

1. 将x,y同时除以一个不为0的$\hat{z}$
2. 并将$\hat{z}$作为其添加维度的坐标

通常选择$\hat{z}=1$

Die homogenen Bildkoordinaten liegen in derselben Äquivalenzklasse wie der zugehörige Raumpunkt。齐次图像坐标与空间中的关联点属于同一等价类

## 3.Perspektivische Projektion mit kalibrierter Kamera

Perspective Projection透视投影 with a Calibrated标定的 Camera

### 3.1 内参数

- 相机的内参数由2部分组成：

  1. 射影变换本身的参数，相机的焦点到成像平面的距离，也就是焦距f

  2. 从成像平面坐标系到**像素坐标系**的转换。

     - 像素坐标系的原点在左上角
       - ->原点的平移

     - 像素是一个矩形块: 假设其在水平和竖直方向上的长度为$\alpha和\beta$
       - ->坐标的缩放

  - Umrechnug von Bildkoordinaten $x$ in Pixelkoordinaten $x'$:$x'=Kx$

    为了能反过来求$x=K^{-1}x'$ 就需要K ist invertierbar -> K的 Eigenwerte$\lambda_i\neq0$

- 若像素坐标系的水平轴为u，竖轴为v；图像传感器距离光轴平移了$(c_x,c_y)$，那么成像平面点(x,y)在像素坐标系下的坐标为：
  $$
  u=\alpha\cdot x+c_x\\
  v=\beta\cdot y+c_y\tag{2}
  $$
  - 公式含义：
    - $\alpha$:Pixelbreite
    - $\beta$:Pixelhöhe
    - $\theta$:Scherung剪切：描述了pixel的形状
    - $c_x$:X-Offset
    - $c_y$:Y-Offset
    - $f$:Brennweite焦距
  
  将1.Bildentstehung中的1式代入这里的2式可得：
  $$
  \begin{cases}
  u=\alpha\cdot f\frac{X}{Z}+c_x\\
  v=\beta\cdot f\frac{Y}{Z}+c_y
  \end{cases}\Rightarrow
  \begin{cases}
  u=f_x\frac{X}{Z}+c_x\\
  v=f_y\frac{Y}{Z}+c_y
  \end{cases}\tag{3}
  $$
  将3式其改写成齐次坐标：
  $$
  \begin{bmatrix}
  u\\
  v\\
  1
  \end{bmatrix}=\frac{1}{Z}
  \begin{bmatrix}
  \alpha & \theta & c_x\\
  0 & \beta & c_y \\
  0 & 0 &1
  \end{bmatrix}
  \begin{bmatrix}
  f & 0 &0\\
  0 & f & 0\\
  0 & 0 & 1
  \end{bmatrix}
  \begin{bmatrix}
  X\\
  Y\\
  Z
  \end{bmatrix}
  =\frac{1}{Z}
  \begin{bmatrix}
  f_x & f_\theta &c_x\\
  0 & f_y & c_y\\
  0 & 0 & 1
  \end{bmatrix}
  \begin{bmatrix}
  X\\
  Y\\
  Z
  \end{bmatrix}\tag{4}
  $$
  由此得到**内参数矩阵(Camera Intrinsics) K**:
  
  也叫Calibration Matrix/Kalibierungsmatrix标定矩阵
  $$
  K=K_sK_f
  =
  \begin{bmatrix}
  f_x & f_\theta &c_x\\
  0 & f_y & c_y\\
  0 & 0 & 1
  \end{bmatrix} \tag{5}
  $$
  
  - 这里的$\theta$是：像素形状不是矩形而是平行四边形时倾斜的角度。通常像素形状都是矩阵=0。
  
  转为非齐次：
  $$
  K\Pi_0=\begin{bmatrix}
  f_x & f_\theta &c_x\\
  0 & f_y & c_y\\
  0 & 0 & 1
  \end{bmatrix}
  \begin{bmatrix}
  1 & 0 & 0&0\\
  0&1&0&0\\
  0&0&1&0
  \end{bmatrix}=\begin{bmatrix}
  f_x & f_\theta &c_x &0\\
  0 & f_y & c_y & 0 \\
  0 & 0 & 1 &0
  \end{bmatrix}
  $$
  
  - $K_f$：Focal Length Matrix。世界坐标系->成像平面坐标系
  - $\Pi_0$：Generic Projection Matrix。将齐次转换为非齐次矩阵
  - $K_s$：Pixel Matrix。成像平面坐标系->像素平面坐标系
  - 上面4式除了Z，即进行了归一化处理，所以点的深度信息在投影过程中被丢失掉了，这也是为什么单目相机无法得到像素点的深度值。
  
- 由5式可知K由4个相机构造相关的参数有关

  - $f_x,f_y$：和相机的焦距,像素的大小有关

    $f_x=\alpha\cdot f;f_y=\beta\cdot f$

    f是焦距

  - $c_x,c_y$：是平移的距离，和相机成像平面的大小有关
  
  求解相机内参数的过程称为**标定**
  
  

### 3.2 外参数

- 由3.1的4式可得：$p=KP$

  - p：是成像平面中像点在像素坐标下pixelkoordinaten的坐标
  - K：是内参数矩阵
  - P：是相机坐标系Bildkoordinaten下的空间点P的坐标。（P的像点是p）

  但相机坐标系是会随相机移动而动的，不够稳定，为了稳定需要引入**世界坐标系**。

  SLAM中的视觉里程计就是在求解相机在世界坐标系下的运动轨迹。

- 相机坐标系$P_c$转为世界坐标系$P_w$ :$P_c=RP_w+t$

  - R：是旋转矩阵
    - 它是正交矩阵，行列式为1，每个列向量都是单位向量且相互正交orthogonal。
    - 它的逆等于它的转置
    - 旋转矩阵不适合用于描述三维空间的场景，因为：
      - Die Essential Matrix hat den Rang 0
      - Es existiert keine Epipolarlinie
      - Es existiert keine Epipolargleichung
      - Es können keine Tiefeninformation berechnet werden
    
  - t：是平移矩阵
  
  $$
  P_c=
  \begin{bmatrix}
  R & t\\
  0^T &1
  \end{bmatrix}P_w
  $$
  
  由此可得**外参数(Camera Extrinsics)T **:
  $$
  T=\begin{bmatrix}
  R & t\\
  0^T &1
  \end{bmatrix}
  $$
  
  - 外参数R和t，即位姿，也是SLAM中带估计的目标，代表着机器人的轨迹。
  
- 

### 3.3 相机矩阵

将内外参数组合到一起称之为**相机矩阵**，用于将真实场景中的三维点投影到二维的成像平面。
$$
\begin{bmatrix}
u\\
v\\
1
\end{bmatrix}
=\frac{1}{Z}K_sK_f\Pi_0P_c
=\frac{1}{Z}
\begin{bmatrix}
f_x & f_\theta &c_x &0\\
0 & f_y & c_y & 0 \\
0 & 0 & 1 &0
\end{bmatrix}
\begin{bmatrix}
R & t\\
0^T &1
\end{bmatrix}
\begin{bmatrix}
X_W\\
Y_W\\
Z_W\\
1
\end{bmatrix}\tag{4}
$$



## 4. Bild,Urbild und Cobild

Image,Preimage原相and Coimage余相

### 4.1 直线在空间的表示

齐次坐标系下: 以方向V经过点$P_0$的一条直线。
$$
L^{(hom)}=\{P_0^{(hom)}+\lambda[v_1,v_2,v_3,0]^T\ |\ \lambda\in R   \}
$$

### 4.2 图像,原相和余相

- The image of a point and of a line,respectively分别的，is their perspective projection透视投影:$\Pi_oP^{(hom)}$和$\Pi_0L^{(hom)}$

- Preimage原相：

  - The Preimage of a point P is all the points in space which project onto a single image point in the image plane
    - 所以点的原相是经过原点的一条直线
    
  - The Preimage of a line L is all the points which project onto a single line in the image plane
    - 所以线的原相是经过原点的一个平面
    
  - 零空间：指像为0的原像空间：{x|Ax=0}

    这里的解x，也称为矩阵A的核

- Coimage余相：

  - The Coimage of points or lines is the orthogonal complement正交补 of the preimage.
    - 点的余相是一个平面上所有的向量，这个平面与点的原相（直线）垂直。
    - 线的余相是一个向量，与线的原相（平面）垂直。维度为1。
  

### 4.3 一些有用的性质

- 成像平面上的一条直线L一般使用余相Coimage来表示：
  - 余相向量：$l\in R^3$, 直线的点$x$ 显然也在直线的原相上，因为余相垂直于原相，所以：
  - $x^Tl=l^Tx=0$
- 共线性Collinearity:
  - 图像上的点$x_1,x_2...x_n$是共线的
    - 如果$Rang([x_1,..,x_n])\leq 2$
    - 如果对任意$w_i都>0$时， $M=\sum_{i=1}^n\omega_ix_ix_i^T$最小的特征值等于0
  - 三个图像上的点是共线的，如果$det[x_1,x_2,x_3]=0$
  - 但实际中由于Discretization离散化、Noise噪音。上面=0的条件是不可能达到的。需要利用thresholds阈值。

# 三、Epipolargeometrie

对极几何。

前面的相机矩阵是针对单个相机的，但单个相机图片不能告诉我们物体的深度信息。这时至少需要两个相机，这样在两视图间内在的射影几何关系就是对极几何，而基本矩阵就算对极几何的代数表示。

## 1.Epipolargleichung

- 目的: describing the correlation相关性 between corresponding image points with
  respect to the euclidian motion欧几里得运动 of the camera

  - 研究三维场景与观测到的二维投影之间的几何关系是基于两种类型的变换：

    - 用**欧几里得运动 (Euclidean motion)** 或**刚体运动 (rigid-body motion)** 来表示相机从当前帧到下一帧图像的运动

    - 用**透视投影 (Perspective projection)** 来表示图像的形成过程 (如：**针孔相机 (pinhole camera)** 等)。

- 几何模型

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Computer%20Vision/%E5%AF%B9%E6%9E%81%E5%87%A0%E4%BD%95.jpg?raw=true)

  - $O,O'$是两个相机(光圈)的中心
  
  - P点是物体所在
    - 如果我们只看左边图像$\pi$上的点p，我们不能知道物体到底是在哪，点P1、P2或其他地方，可有了右边图像$\pi'$上的$p'$我们就能得到物体点P
    
  - 基线：连线$OO'$
  
  - 对极平面Epipolar plane of P：极限$OO'$和观测物体$P$组成的平面$OO'P$
    - The epipolar plane is spanned by the position vectors of the image point and of the epipole :$span(p,e)$
    
  - 对极线Epipolar line：对极平面和两相机图像的交线$l,l'$
    
    - 线op上所有的点$p_1,p_2$在左图像中，都被看作了同一个点，但是在右图像中会将其看作一根线$l'$
    
      右图像$\pi'$上的这根线$l'$就是左图像$\pi$上线op的对极线
    
      同样的，$l$是右图上线$o'p$的对极线
    
    - 双目相机可以直接按对极几何计算，单目相机需要在不同位置拍2张图利用视差（disparity）信息来估算场景中点的深度。
    
    - The intersection相交 of the epipolar plane and the image plane
    
    - The epipolar line is the image that is created from the Preimage原像 in the other camera system
    
    - The epipolar line is identified确认 by means of the Coimage余象：$l$~$e\times p$
    
  - 对极点Epipoles：基线和两相机图像的交点$e,e'$
  
    - 相机中心在另一个相机图像平面的投影点
    - The perspective projection透视投影 of the respective各自的 optical centres in the other camera systems
### 本质矩阵:Essential matrix

  The essential matrix contains the information of the euclidian motion

  - 我们知道由相机1到相机2是[刚体运动](https://zhuanlan.zhihu.com/p/32297200)，那么观测点P在相机1坐标系的坐标$P$就可以通过刚体转换变成相机2坐标系下$P'$:
    $$
    P'=RP+T
    $$

    - T和R分别表示平移和旋转，即欧几里得运动(R,T)
    - $P$是物体在相机1下的坐标
    - $P'$是物体在相机2下的坐标

  - 两边同时叉乘T
    $$
    T\times P' = T\times RP+T\times T=T\times RP
    $$

    - $T\times P'$表示对极平面的法线

  - 两边同时左乘$P'$
    $$
    P'(T\times P')=P'(T\times RP)
    $$

  - 由于$P'$点在对极平面上，所以$P'$和$T\times P'$是垂直的
    $$
    0=P'(T\times RP)
    $$

  - 两个向量的叉乘 = 一个向量的反对称矩阵 与 另一个向量 的点乘
    $$
    P'(T\times RP)=P'\hat{T}RP=P'^TEP=0
    $$

    - $\hat{T}$​是T的反对称矩阵skew symmetric matrix

      如果$T=(a,b,c)$则:$\hat{T}=\left[
       \begin{matrix}
         0 & -c & b \\
         c & 0 & -a \\
         -b & a & 0
        \end{matrix}
        \right]$
    
      
    
    - Epipolar equation对极方程：$P'^TEP=0$
    
      The epipolar equation describes the correlation between corresponding image points from different images of the same scene
    
    - **Essential matrix本质矩阵**：$E=\hat{T}R$
    
- 本质矩阵的特性：[奇异值分解SVD](https://zhuanlan.zhihu.com/p/26306568)(Singular Value Decomposition)

  - 任何一个m x n的矩阵A都可以奇异值分解为：$A=P\Sigma Q^T$

    - $P:AA^T=P\Lambda_1P^T$

      - $P$每一列是$AA^T$的特征向量，称之为**左奇异向量**（left singular vector）

      - $AA^T$是 m x m的对称矩阵

      - $\Lambda_1$是$矩阵AA^T$对角线上的元素是从大到小排列的特征值，m x m, 且与$\Lambda_2$非零元素相同。

        即矩阵$AA^T$和矩阵$A^TA$的非零特征值相同

    - $Q:A^TA=Q\Lambda_2Q^T$

      - $Q$每一列是$A^TA$的特征向量，称之为**右奇异向量**（right singular vector）
      - $A^TA$是 n x n的对称矩阵
      - $\Lambda_2$是$矩阵A^TA$对角线上的元素是从大到小排列的特征值，n x n,且与$\Lambda_1$非零元素相同

    - $\Sigma:m\times n的矩阵$，位于其对角线上的元素称为**奇异值**（singular value）

  - 本质矩阵就可以被分解为
    $$
    E=P \left[
     \begin{matrix}
       \sigma &  &  \\
        & \sigma &  \\
        &  & 0
      \end{matrix}
      \right]Q^T=
      [p_1\ p_2\ p_3]\left[
     \begin{matrix}
       \sigma &  &  \\
        & \sigma &  \\
        &  & 0
      \end{matrix}
      \right]\left[
     \begin{matrix}
       q_1^T  \\
        q_2^T \\
        q_3^T
      \end{matrix}
      \right]
    $$
    Essential matrices are the ones , whose singular value decomposition produces two **identical** singular values and one singular value equal to zero.
    
    $\sigma$不能等于0；P,Q必须是Rotationsmatrizen旋转矩阵。
    
    所以本质矩阵的秩为2.
    
  - 一个使用svd求解问题的例子：
  
    - 问：
  
      矩阵A的列是几个点的齐次坐标，用什么方法可以直到这些点是否在同一条直线上？
  
    - 答：
  
      1. 求解**SVD(A)**也即求解$AA^T$的特征值分解
      2. 为保证$\sigma$!=0,需要求解：
         - rank(A)=min(m,n)
         - $det(A)$!=0

## 2.Epipole und Epipolarlinien

- Epipole对极点和本质矩阵的关系：

  $e_1$liegt im Kern von $E$: $Ee_1=0$,也可写作$e_1 \sim R^TT$

  $e_2$liegt im Kern von $E^T$: $E^Te_2=0$,也可写作$e_2^TE=0$

  因为：

  - The Coimage余象 of $e_1$is equivalent to the third singular value of E from the right: $e_1等价于q_3 $
  - The Coimage余象 of $e_2$is equivalent to the third singular value of E from the left: $e_2等价于p_3 $
  - q3,p3都是上面本质矩阵 奇异分解后的特征向量。

- Epipolar lines对极线的一些性质：

  - $l^Tp=0,l^Te=0$
    - l,p,e分别是某一图像面上的对极线，图像点，对极点
  - $l\ 等价于E^Tp',l'\ 等价于Ep$
    - 上标 ' 表示第二个图像的xx

- 应用：Correspondence一致点 Search

  已知：$E和p$

  1. 计算: $l'等价于Ep$
  2. 确定$l'$的图像2
  3. 在这个图像2中寻找p的一致点$p'$

## 3.Acht-Punkt-Algorithmus

用于求本质矩阵

- 如果Euclidian Motion未知，Feature points对应关系已知，如何确认Essential Matrix E?

  已知：n pairs of corresponding points$(x_1^j,x_2^j)$

  1. 每一对点都满足:$x_2^jEx_1^j=0$（对极方程）

     - n对特征点可以展开变形为$AE^s=0$

       - A是n个特征点向量组成的矩阵：

         X1,X2是一对特征点。$\bigotimes$是Kronecker product

       $$
       X_1= \left[
        \begin{matrix}
          x_1  \\
          y_1  \\
          z_1 
         \end{matrix}
         \right]
         X_2=\left[
        \begin{matrix}
          x_2  \\
          y_2  \\
          z_2 
         \end{matrix}
         \right]\Rightarrow
         a:=X_1\bigotimes x_2=\left[
        \begin{matrix}
          x_1x_2  \\
          x_1y_2  \\
          x_1z_2  \\
          y_1x_2  \\
          y_1y_2  \\
          y_1z_2  \\
          z_1x_2  \\
          z_1y_2  \\
          z_1z_2  \\
         \end{matrix}
         \right]\\
         A：=\left[
        \begin{matrix}
          a^{1\ T}  \\
          a^{2\ T}  \\
          \vdots	 \\
          a^{n\ T} 
         \end{matrix}
         \right]
       $$

       - $E^s$是本质矩阵的向量化：
         $$
         E= \left[
          \begin{matrix}
            e_{11} & e_{12} & e_{13}  \\
            e_{21} & e_{22} & e_{23}  \\
            e_{31} & e_{32} & e_{33} 
           \end{matrix}
           \right]\Rightarrow E^s= \left[
          \begin{matrix}
            e_{11} \\ e_{12} \\ e_{13}  \\
            e_{21} \\ e_{22} \\ e_{23}  \\
            e_{31} \\ e_{32} \\ e_{33} 
           \end{matrix}
           \right]
         $$
         

  2. 因为$E\in R^{3\times3}$,所以有9个未知量

  3. Scaling invariance缩放不变性：如果E是答案，那么$\lambda E$也是答案

  4. 一共需要8个independent equations

- 8点算法：

  - 由$n\geq8$个对应点对构建矩阵A
  
  - 求线性解：
  
    因为n>8个时，the homogeneous system of linear equations has non-trivial非凡 solutions。所以$AE^s=0$需要转化为$G^s=argmin_{||E^s||_2=1}||AE^s||_2^2$
  
    - $||x||_2$是欧几里得范数的意思
    - 通过奇异值分解$A=P\Sigma Q^T$，可求得解为$G^s=q_9$(Q的第9列)
  
  - 将解$G^s$Projection onto normalized essential matrices(奇异性约束)
  
    上面求得的G不符合本质矩阵的定义(奇异性约束)，因此我们可以假设一个E去逼近G求得E。
  
    - Resorting重新排列向量$G^s$为3X3的矩阵 G
    - 对G奇异值分解:$G=U_G\Sigma_GV_G^T$
      - 其中$\Sigma_G=\left[
         \begin{matrix}
           \sigma_1 &  &   \\
            & \sigma_2 &   \\
            &  & \sigma_3
          \end{matrix}
          \right]$
    - Find the next “ essential matrix w.r.t. G
      - 通过最小化 $E=argmin_{||E\in\epsilon}||E-G||_F^2$来得到最佳的本质矩阵
      - 最后解为:$E=U_G\left[
         \begin{matrix}
           \sigma &  &   \\
            & \sigma &   \\
            &  & 0
          \end{matrix}
          \right]V_G^T$
        - 实际中$\sigma$取1
        - 所以八点算法一共用到2次SVD

## 4.3D Rekonstruktion

- 目的：

  - Estimate the euclidian motion$(R,T)$from the essential matrix
  - reconstruct 3D points from point correspondences$(X_1^j,X_2^j)$

- Reconstruction of Rotation and Translation重新得到$(R,T)$

  - 由3我们已经算出了$E=U\Sigma V^T$,又知道$E=\hat{T}R$

  - 从而我们分解得到：
    $$
    \begin{cases}
    \hat{T_1}=UR_Z(\frac{\pi}{2})\Sigma U^T \\
    \hat{T_1}=UR_Z(-\frac{\pi}{2})\Sigma U^T \\
    R_1=UR_Z^T(\frac{\pi}{2})V^T \\
    R_2=UR_Z^T(-\frac{\pi}{2})V^T
    \end{cases}\ 其中
    R_Z(\pm\frac{\pi}{2})= \begin{bmatrix}
       0 & \mp1 & 0 \\
       \pm1 & 0 & 0 \\
       0 & 0 & 1
      \end{bmatrix}表示饶Z轴的旋转矩阵
    $$

    - 可知有4组解$(\hat{T}_1,R_1)(\hat{T}_1,R_2)(\hat{T}_2,R_1)(\hat{T}_2,R_2)$,但只有一组满足所有投影点在两个相机中都为正深度，代入匹配点计算出坐标，即可选出正确的一组

  - 最后再将反对称矩阵$\hat{T}$转为T

- Reconstruction of the 3D Coordinates（三角测量）

  目的：: estimate估计 the depth of points in space

  - 由上面得到了(T,R)后，2个图像上的特征点可描述为

    $\lambda_2 x_2=\lambda_1Rx_1+\gamma T$

    - $\lambda_1,\lambda_2$分别是2个图像上各自特征点对应的深度, 都要＞0
    - 3个特征点组成的线性方程组，就可以求得上面这3个系数

## 5.Die Fundamentalmatrix

- 动机：

  - 对于一个标定了相机有Epipolar equation对极方程$x_2^TEx_1=0$

  - 是否有同样的方程描述点在标定了的相机和未标定相机之间的关系？

  - 已知相机矩阵K秒速了标定坐标和未标定坐标之间的关系（二、3.1）
    $$
    K=K_sK_f
    =
    \begin{bmatrix}
    f_x & f_\theta &c_x\\
    0 & f_y & c_y\\
    0 & 0 & 1
    \end{bmatrix}
    $$

- Fundamental Matrix基本矩阵

  1. 由二、3.2可知相机坐标系$x_c$由世界坐标系$x_w$经外参而得 :$x_c=Rx_w+t$

  2. 由二、3.1可知图像坐标系$x_{im}$由相机坐标系$x_c$经内参而得:$x_{im}=Kx_c$

  3. 标定了的相机有对极方程$x_{2c}^TEx_{1c}=0$

  4. 由2可得$x_{1c}=K^{-1}x_{1im},x_{2c}=K^{-1}x_{2im}$
  5. 将4代入3可得：$x_{2im}^TK^{-T}EK^{-1}x_{1im}=0$
  6. 其中$F=K^{-T}EK^{-1}$即基本矩阵

- Fundamental矩阵的SVD分解
  $$
  F=P \left[
   \begin{matrix}
     \sigma_1 &  &  \\
      & \sigma_2 &  \\
      &  & 0
    \end{matrix}
    \right]Q^T=
    [p_1\ p_2\ p_3]\left[
   \begin{matrix}
     \sigma_1 &  &  \\
      & \sigma_2 &  \\
      &  & 0
    \end{matrix}
    \right]\left[
   \begin{matrix}
     q_1^T  \\
      q_2^T \\
      q_3^T
    \end{matrix}
    \right]
  $$
  
  - 相比于本质矩阵E的SVD分解，这里奇异值$\sigma$就不需要一定是相同的了，可以是不同的
  
- 基本矩阵的性质：

  - 未标定相机：Epipolargleichung：$x_2^{'T}Fx_1^{’}=0$
  - 基本矩阵：$F=K^{-T}\hat{T}RK^{-1}$
  - 对于极点：$Fe_1^{'}=0,F^Te_2^{'}=0$
    - 所以$KR^TT$ liegt im Kern von F,也等价于$e_1^{'}$
  - 对于极线：$l_2^{'}等价于Fx_1^{'},l_1^{'}等价于Fx_2^{'}$
  - 不能依靠F来三维重建

# 四、Planare Szenen

Planar Scenes平面场景

## 1.Planare Epipolargleichung

Planar Epipolar Equation平面对极方程

- 目的：
  1. Formal correlation正式的相互关系 between correspondences联系
  2. Estimate估计 the euclidian motion of the camera through correspondences in a planar sene
  
- Plane epipolar Equation公式推导

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Computer%20Vision/Plane%20Equation.png?raw=true)

  - 平面：$H=:\{P\in R^3| n^TP=d\}$

    - $n:$Normalized normal vector平面的单位法向量
  
    - $d:$Distance from $H$ to $O_1$
  
    - 点P1的**Plane Equation**展开：
      $$
      n^TP_1=n_1X+n_2Y+n_3Z=d \tag{1}
      $$
  
  - Euclidian Motion:$P_2=RP_1+T$代入1式得：
    $$
    \begin{align}
    P_2&=(R+\frac{1}{d}Tn^T)\cdot P_1\\
    &=HP_1
    \end{align}\tag{2}
    $$
  
    - H：**Homography Matrix**单应矩阵
  
      即上面2式的H，是平面H对于euclidian motion(R,T)的单应矩阵。描述的是空间中**同一平面**上的三维点在两张图像中的对应关系。这里需要强调的是**同一平面**。
  
      - 用单应矩阵可以Unique determination of correspondences
      - 单应矩阵也可以用于计算 epipolar lines outside of the plane
  
  - 由此我们得到**Planar epipolar equation**平面对极方程：
  
    $x_2\sim Hx_1$,这里就不能写等于了
    
    相对的$P_2=HP_1, P_2\sim HP_1$都对
    
    - $x_2\ orthogonal\ to\ 垂直于\ u\times Hx_1\ for\ all\ u \in R^3$
  
- 单应矩阵H必有如下Singulärwertszerlegung
  $$
  H=U\begin{bmatrix}
  \sigma_1 & 0 & 0\\
  0 & 1 & 0\\
  0 & 0 & \sigma_3
  \end{bmatrix}V^T
  $$

  - 其中特征值$\sigma_1$和$\sigma_3$都不能等于0
  - 所以相比于本质矩阵E和基本矩阵F,H是满秩为3的
  - 和本质矩阵有如下关系：
    - $E=\hat{T}R$
    - $E=\hat{T}H$
    - $H^TE+E^TH=0$

## 2.Vier-Punkt-Algorithmus

### 2.1 四点算法

4-Point Algorithm4点算法：用于从已知的corresponding points对应点来计算单应矩阵

- 单应矩阵的自由度

  使用的是齐次坐标系，也就是说可以进行任意尺度的缩放。比如我们把hij乘以任意一个非零常数k并不改变等式结果。所以实际上单应矩阵H只有8个自由度。

  所以需要4个点对，去求解H中的8个未知数

- 求解步骤：

  - **Planar epipolar equation**:$x_2=Hx_1$

  - 左乘$x_2$的反对称矩阵:
    $$
    \hat{X_2}HX_1=0
    \\
    其中:\\
    X_1= \left[
     \begin{matrix}
       x_1  \\
       y_1  \\
       z_1 
      \end{matrix}
      \right]
      \hat{X_2}=\left[
     \begin{matrix}
       0 & -z_2&y_2  \\
       z_2 & 0 & -x_2  \\
       -y_2 &x_2&0 
      \end{matrix}
      \right]\Rightarrow
      B:=X_1\bigotimes \hat{X_2}=\left[
     \begin{matrix}
       x_1\hat{X_2}  \\
       y_1\hat{X_2}  \\
       z_1\hat{X_2} \\
      \end{matrix}
      \right]\in R^{9\times3}
    $$
    展开后可得：$B^TH=0$
    
  - 如果有n对特征点，即有n个B:
    $$
    AH=0,A:=\left[
     \begin{matrix}
       B^{1T}  \\
       B^{2T}  \\
       \vdots  \\
       B^{nT} 
      \end{matrix}
      \right]\in R^{3n\times 9}
    $$
    因为H又8个未知数，所以A的rang秩必须=8。
  
- 但直接求解AH=0来确定H是unzuverlässig不可靠的,因为：
  
    1. Rauschen噪音
    
    2. Quantifizierung量化
    
    3. Falsch zugeordnete Korrespondenzen
    
- 为此需要类似8点算法，将求解AH=0转变为求解最小化问题:
  
  $H_L=argmin_{||H_L||_2=1}||AH||_2^2$
  
  由SVD奇异值singulär-wertszerlegnung分解A后可得：
  $$
  ||AH||_2^2=H^TV\begin{bmatrix}
  \sigma_1^2& &  \\
   &\ddots  & \\
   & & \sigma_9^2
  \end{bmatrix}
  V^TH
  $$
  
  最后我们选择$H=v_9$因为das steht senkrecht auf allen anderen Vektoren in V:$V^Tv_9=[0\cdots1]^T$
  
- 如果我们得到了一falsch skalierte的$H_L$，如何得到正确的$H$？
  
  - 由**Homography Matrix**的性质:单应矩阵H必有如下Singulärwertszerlegung
  
  $$
  H=U\begin{bmatrix}
  \sigma_1 & 0 & 0\\
  0 & 1 & 0\\
  0 & 0 & \sigma_3
  \end{bmatrix}V^T，其中特征值\sigma_1和\sigma_3都不能等于0
  $$
  
  - 然后需要将$H_L$scale拉升到正确值:$H_L:=\lambda H$
  
    由上面性质可知$\lambda=\sigma_2(H_L)$
  
    最后得：$H=\frac{H_L}{\sigma_2(H_L)}$
  
  - 这样算出来的正负H都符合平面对极方程，所以增加一个 条件：$x_2^THx_1>0$

### 2.2 从单应矩阵来3维重建

- 求解步骤：

  1. Eigenvalue decomposition特征值分解：$H^TH=V\Sigma^2V^T,V=[v_1\ v_2\ v_3]\in SO(3)$

  2. 定义：
     $$
     u_1:=\frac{\sqrt{1-\sigma_3^2}v_1+\sqrt{\sigma_1^2-1}v_3}{\sqrt{\sigma_1^2-\sigma_3^2}},u_2:=\frac{\sqrt{1-\sigma_3^2}v_1-\sqrt{\sigma_1^2-1}v_3}{\sqrt{\sigma_1^2-\sigma_3^2}}
     $$

  3. 定义：
     $$
     U_1:=[v_2\quad u_1\quad \hat{v_2}u_1],\ W_1:=[Hv_2\quad Hu_1\quad \hat{Hv_2}Hu_1]\\
     U_2:=[v_2\quad u_2\quad \hat{v_2}u_2],\ W_1:=[Hv_2\quad Hu_2\quad\hat{Hv_2}Hu_2]
     $$

  4. 得到4个解：

     ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Computer%20Vision/%E5%8D%95%E5%BA%94%E7%9F%A9%E9%98%B53%E7%BB%B4%E9%87%8D%E5%BB%BA%E8%A7%A3.png?raw=true)

### 2.3 单应矩阵和本质矩阵相互计算

- 由于：
  $$
  E=\hat{T}R\ and\ H=R+Tu^T,R\in \mathbb{R}^{3\times3},T、u\in \mathbb{R}^3,||T||=1
  $$

  - 得到：
    - $E=\hat{T}H$
    - $H^TE+E^TH=0$
    - $H=\hat{T}^TE+Tv^T$  for a specific $v\in \mathbb{R}^3$

- Calculation of the Essential Matrix from the Homography Matrix

  **从单应矩阵H算本质矩阵E：**

  - 单应矩阵H 和 两组corresponding points$(x_1^j,x_2^j ),j=1,2$已知（这两个点不在一个单应矩阵平面上）
  - Epipolar lines$l_2^j=\hat{x}_2^jHx_1^j$ intersect相交于 epipol对极点$e_2=T$
  - 可得：$E=\hat{T}H$
    - 其中$T=\hat{l}_2^1l_2^2$
    - $||T||_2=1$

- Calculation of the Homography Matrix from the Essential Matrix

  **从本质矩阵E算单应矩阵H**

  - 本质矩阵E 和  三组corresponding points$(x_1^j,x_2^j ),j=1,2,3$已知。3个点都在一个平面内
  
  - 可得：$H=\hat{T}^TE+Tv^T$  for a specific $v\in \mathbb{R}^3$
  
  - 其中v根据等式$\hat{x}_2^j(\hat{T}^TE+Tv^T)x_1^j=0,j=1,2,3$计算而得
  
    因为：
  
    1. 极线计算:$l_2^j=\hat{x}_2^jHx_1^j$
    2. 由1可知对极平面上的点就有：$$0=\hat{x}_2^jHx_1^j$$，因为线长度=0
    3. 根据3个$(x_1^j,x_2^j )$点，就可以得到$v=[v1\ v2\ v3]$

## 3.Kamerakalibrierung

Camera Calibration相机标定

从几张棋盘得照片来标定calibration matrix 标定矩阵k

- 方法：

  - Z axis of the world coordinates perpendicular to the chessboard

  - 参考3.3有：

    - 对于一个点$P= \left[\begin{matrix}X  \\Y  \\0  \\1 \end{matrix} \right]$，那么：
      $$
      x'\sim K\Pi_0\begin{bmatrix}
      R & T\\
      0^T &1
      \end{bmatrix}
      \begin{bmatrix}
      X\\
      Y\\
      0\\
      1
      \end{bmatrix}=
      K[r_1\quad r_2\quad T]\begin{bmatrix}
      X\\
      Y\\
      1
      \end{bmatrix}
      $$

    

  - Estimation of the Homography求解单应矩阵

    - $H:=K[r_1\quad r_2\quad T]$​是一个单应矩阵，将棋盘得homogeneous coordinates齐次坐标 映射到 homogeneous , uncalibrated coordinates in the image plane 
    - 用四点算法求解单应矩阵H
    
  - Estimation of the Calibration Matrix求解标定矩阵

    - 对每一个view,有constraints约束
  
      - $h_1^TBh_2=0$
      - $h_1^TBh_1=h_2^TBh_2$
      - 其中$B:=K^{-T}K^{-1}$
  
    - 从而得到2个方程
      $$
      V^jb=\begin{bmatrix}
      V^T_{(h_1,h_2)}\\
      (V(h_1,h_1)-V(h_2,h_2))^T
      \end{bmatrix}b=0
      $$
  
    - 对于n个views of the chessboard有等式：$Vb=0,V\in\mathbb{R}^{2n\times6}$
  
    - 因此需要3张图来确定b，得到6个等式对应6个未知量
  
  - 求解$Vb=0$的步骤
  
    - 用SVD分解矩阵V
  
    - 用最小V特征值的右奇异矩阵来生成symmetrical matrix对称矩阵$\hat{B}$
      $$
      B:=K^{-T}K^{-1}=\begin{bmatrix}
      B_{11} &B_{12}& B_{13}\\
      B_{12} &B_{22}& B_{23}\\
      B_{13} &B_{23}& B_{33}
      \end{bmatrix}
      $$
  
    - 选择正值$B=\pm\hat{B}$
  
    - Decompose分解$B$ with the Cholesky factorization因数分解 into the product$B=\hat{K}^T\hat{K}$, whereby$\hat{K}$is an upper triangular matrix三角矩阵
  
    - 可得：$K=\hat{K}^{-1}$
  
- 为什么标定时世界坐标系需要让棋盘位于原点，并且让Z轴垂直棋盘？

  因为这样所有点的Z轴都可以为0

  

# *、 TUM CV课作业代码

## 第一次作业

### 1.1Color Image conversion

```matlab
function gray_image = rgb_to_gray(input_image)
    % This function is supposed to convert a RGB-image to a grayscale image.
    % If the image is already a grayscale image directly return it.
    
    if(size(input_image,3)~=3)
        gray_image = input_image;
        return
    end
    
    input_image = double(input_image)
    gray_image = 0.299*input_image(:,:,1) + 0.587*input_image(:,:,2) + 0.114*input_image(:,:,3);
    gray_image = uint8(gray_image);
end
```

### 1.2Image Gradient(索伯算子)

```matlab
function [Fx, Fy] = sobel_xy(input_image)
    % In this function you have to implement a Sobel filter 
    % that calculates the image gradient in x- and y- direction of a grayscale image.
    gx = [-1,0,1;-2,0,2;-1,0,1];
    gy = gx';
    
    Fx = filter2(gx,input_image,'same');
    Fy = filter2(gy,input_image,'same');
    
end
```

### 1.3 Harris Detector

- 基础方法：

    ```matlab
    function features = harris_detector(input_image, varargin)
        % In this function you are going to implement a Harris detector that extracts features
        % from the input_image.
    
    
    
        % *************** Input parser ***************
        % 创建具有默认属性值的输入解析器对象，用于检测输入进来的参数是否符合要求。
        %使用 `inputParser` 对象，用户可以通过创建输入解析器模式来管理函数的输入。要检查输入项，您可以为所需参数、可选参数和名称-值对组参数定义验证函数。还可以通过设置属性来调整解析行为，例如如何处理大小写、结构体数组输入以及不在输入解析器模式中的输入。
        P = inputParser;
        
        % addOptional:将可选的位置参数添加到输入解析器模式中
        P.addOptional('segment_length', 15, @isnumeric);
        P.addOptional('k', 0.05, @isnumeric);
        P.addOptional('tau', 1e6, @isnumeric);
        P.addOptional('do_plot', false, @islogical);
    	
    	% 解析函数输入
        P.parse(varargin{:});
    	
    	% results:指定为有效输入参数的名称和对应的值，以结构体的形式存储。有效输入参数是指名称与输入解析器模式中定义的参数匹配的输入参数。Results 结构体的每个字段对应于输入解析器模式中一个参数的名称。parse 函数会填充 Results 属性
        segment_length  = P.Results.segment_length;
        k               = P.Results.k;
        tau             = P.Results.tau;
        do_plot         = P.Results.do_plot;
        
        
        
        % *************** Preparation for feature extraction ***************
        % Check if it is a grayscale image，处理过的灰度照片是1维的，所以检测图片的维度
        if(size(input_image,3)==3)
            error("Image format has to be NxMx1");
        end
        
        % Approximation of the image gradient
        input_image = double(input_image);	% matlab读入图像的数据是uint8，而matlab中数值一般采用double型（64位）存储和运算
        [Ix,Iy] = sobel_xy(input_image);	% 由1.2的函数得到横向及纵向边缘检测的图像灰度值
        
        % Weighting
        % fspecial()创建一个高斯低通滤波来提高我们选取的窗口segment中间像素的影响力
        % 第一个参数指明用的是高斯滤波器。高斯滤波器是一种线性滤波器，能够有效的抑制噪声，平滑图像。
        % 第二个参数是这个滤波器的尺寸，和我们的窗口尺寸保持一致。
        % 第三个参数是sigma标准差。调整σ实际是在调整周围像素对当前像素的影响程度，调大σ即提高了远处像素对中心像素的影响程度，滤波结果也就越平滑。这里设置为segment_length/5
        w = fspecial('gaussian',[segment_length,1],segment_length/5);
        
        % Harris Matrix G
        % conv2(u,v,A,shape)先算矩阵A每列和u的卷积，再算矩阵A每列和v的卷积。same表示返回卷积中大小与A相同的部分，还有full（默认，全部）和valid（没有补零边缘的卷积部分）2个shape选项。
        % 这里是在计算图像中每一个点的哈里斯矩阵。
        G11 = double(conv2(w,w,Ix.^2,'same'));
        G22 = double(conv2(w,w,Iy.^2,'same'));
        G12 = double(conv2(w,w,Ix.*Iy,'same'));
        
        
        
        % *************** Feature extraction with the Harris measurement ***************
        % 首先根据公式H=det(G) - k*tr(G)*tr(G)计算 H
        H = ((G11.*G22 - G12.^2) - k*(G11 + G22).^2);
        % ceil(X)将 X 的每个元素四舍五入到大于或等于该元素的最接近整数
        % H是一个长方形矩阵，肯定会把边缘点包含进来。我们要消除边缘点，于是从长方形四边往里取一个segment_length/2边界。这个边界里的就是我们要取特征的部分，令mask为1。边界外的就令mask=0，乘上H后就自动消除边缘了。
        mask=zeros(size(H));
        mask((ceil(segment_length/2)+1):(size(H,1)-ceil(segment_length/2)),(ceil(segment_length/2)+1):(size(H,2)-ceil(segment_length/2)))=1;
        R = H.*mask;
    	% harris detector是用于找角点的，只要大于一定阈值的才是角点，所以小于阈值的令为0
        R(R<tau)=0;
        % find会返回R中非0的坐标
        [row,col] = find(R); 
        corners = R;
        % 注意matlab中图像用矩阵表示所以原点在左上方，而我们平时说的坐标原点在左下方，所以这里的col和row是颠倒的。
        features = [col,row]'; 
        
        
        % *************** Plot ***************
        % 将找到的特征滑倒图片里去
        if P.Results.do_plot == true
            figure
            imshow(input_image);
            hold on
            plot(features(1,:),features(2,:),'*');
        end
    end
    ```

- 进阶方法：

  前面普通方法用了一个全局的阈值threshold：tau。但以一张图片的特征分布变化是很大的，用一个全局的阈值就会导致特征过多，所以先用一个小阈值来消除一些弱特征来消除噪声是很有必要的。

  - 一个辅助函数

    产生一个矩阵，离这个矩阵中心点的距离如果小于min_dist那就是0，如果大于就是1.

    ```matlab
    function Cake = cake(min_dist)
        % The cake function creates a "cake matrix" that contains a circular set-up of zeros
        % and fills the rest of the matrix with ones. 
        % This function can be used to eliminate all potential features around a stronger feature
        % that don't meet the minimal distance to this respective feature.
        %[X,Y] = meshgrid(x,y) 基于向量 x 和 y 中包含的坐标返回二维网格坐标。X 是一个矩阵，每一行是 x 的一个副本；Y 也是一个矩阵，每一列是 y 的一个副本。
        [X,Y]=meshgrid(-min_dist:min_dist,-min_dist:min_dist);
        % 这样sqrt(X.^2+Y.^2)就相当于每个点到矩阵中心的距离
        Cake=sqrt(X.^2+Y.^2)>min_dist;
        
    end
    ```

  - 

  ```matlab
  function [min_dist, tile_size, N] = harris_detector(input_image, varargin)
      % In this function you are going to implement a Harris detector that extracts features
      % from the input_image.
      
      % *************** Input parser ***************
      % 相比普通方法增加如下变量
      % 1. min_dist:两个特征之间的最小距离，单位是pixel
      % 2. tile_size:定义了一个局部区域的大小
      % 3. N:一个局部区域里最多可以有的特征数量
      P = inputParser;
      P.addOptional('segment_length', 15, @isnumeric);
      P.addOptional('k', 0.05, @isnumeric);
      P.addOptional('tau', 1e6, @isnumeric);
      P.addOptional('do_plot', false, @islogical);
      P.addOptional('min_dist', 20, @isnumeric);
      P.addOptional('tile_size', [200,200], @isnumeric);
      P.addOptional('N', 5, @isnumeric);
      P.parse(varargin{:});
      segment_length  = P.Results.segment_length;
      k               = P.Results.k;
      tau             = P.Results.tau;
      tile_size       = P.Results.tile_size;
      N               = P.Results.N;
      min_dist        = P.Results.min_dist;
      do_plot         = P.Results.do_plot;
      
      if size(tile_size) == 1
          tile_size   = [tile_size,tile_size];
      end
      % *************** Preparation for feature extraction ***************
      % 与普通方法一致
      if(size(input_image,3)==3)
          error("Image format has to be NxMx1");
      end
      input_image = double(input_image);	
      [Ix,Iy] = sobel_xy(input_image);	
      w = fspecial('gaussian',[segment_length,1],segment_length/5);
      G11 = double(conv2(w,w,Ix.^2,'same'));
      G22 = double(conv2(w,w,Iy.^2,'same'));
      G12 = double(conv2(w,w,Ix.*Iy,'same'));
      
      
      
      % *************** Feature extraction with the Harris measurement ***************
      % 与普通方法一致
      H = ((G11.*G22 - G12.^2) - k*(G11 + G22).^2);
      mask=zeros(size(H));
      mask((ceil(segment_length/2)+1):(size(H,1)-ceil(segment_length/2)),(ceil(segment_length/2)+1):(size(H,2)-ceil(segment_length/2)))=1;
      R = H.*mask;
      R(R<tau)=0;
      [row,col] = find(R); 
      corners = R;
      
      % *************** Feature preparation ***************
      % 扩展我们的角点矩阵：在corners角点矩阵周围扩展一圈宽度为min_dist的0
      % 具体做法是，先建那么大的矩阵，然后中间corners部分赋值进去。
      expand = zeros(size(corners)+2*min_dist);
      expand(min_dist+1:min_dist+size(corners,1),min_dist+1:min_dist+size(corners,2)) = corners;
      corners = expand;  
      % [B,I] = sort(A,direction)按照direction(ascend/descend)来对矩阵A的每一列进行排序.
      % B是排列完的矩阵(mxn,1)维的，I是排序完的B在A中的索引，即B==A(I)
      [sorted, sorted_index] = sort(corners(:),'descend');
      % 排除角点矩阵范围内那些为0的点，因为他们不是特征点
      sorted_index(sorted==0)=[];
      
      % *************** Accumulator array ***************
  	% 创建一个矩阵acc_array用于储存每个小部分teil里的参数。所以共有size(image)/tile_size个小tile.
      acc_array = zeros(ceil(size(input_image,1)/tile_size(1)),ceil(size(input_image,2)/tile_size(2)));
      % 上面排序后我们得到sorted_index个特征。此外每个teil最多有N个特征。这两个共同影响了我们最后特征应该有多少个。
      features = zeros(2,min(numel(acc_array)*N,numel(sorted_index)));
      
      % *************** Feature detection with minimal distance and maximal number of features per tile ***************
      % 首先创建一个矩阵，大于矩阵中心距离min_dist的元素设置为1，即中间的各个元素都是0
      % 这样一个特征点周围的矩阵 点乘上 这个cake矩阵后，就保证了这个特征点最小距离内没有其他特征点。
      Cake = cake(min_dist);
      count = 1;
      for i = 1:numel(sorted_index)
      	% 前面排列时根据特征强度降序排列，所以我们先处理特征最强的。因为由于我们设置的最大特征数，导致一些特征丢失，那肯定丢失那些特征弱的。
      	% sort返回的是(mxn,1)维矩阵，index也指的是第几个，所以下面还要手动转化为x,y
          current = sorted_index(i);
          % 如果=0说明不是特征点（角点）
          if corners(current) == 0
              continue;
          else 
          	% Y = floor(X) 将 X 的每个元素四舍五入到小于或等于该元素的最接近整数。相反于ceil()		
          	% 因为index是1维的，所以要手动转化为x,y,注意这里原点在左上角，所以x表示列数
              % 对索引除以行数后，就得到除尽了几列，那再加一列就是元素矩阵中的列数x.
              x_corners = floor(current/size(corners,1));
              % 索引-除尽的列数*行数=元素在下一列的第几行
              y_corners = current-x_corners*size(corners,1);
              x_corners = x_corners+1;
          end
          	% 求得我们当前的特征点，是图像中的第几个小teil区域。同样注意这里原点在左上角，所以x表示列数
          	% 原本corners是覆盖了整个范围，然后我们扩展了一圈min_dist以方便清空特征点周围的特征。
          	% 所以现在corner的范围是原先corner+min_dist那一圈。
          	% min_dist那一圈都是0,我们在画小teil的时候不需要考虑这些min_dist。
          	% 所以要减掉min_dist坐标，以得到特征在原先corner下的坐标。
              x_acc = ceil((x_corners-min_dist)/tile_size(2));
              y_acc = ceil((y_corners-min_dist)/tile_size(1));
  			
  			% 这样一个特征点周围的矩阵 点乘上 这个cake矩阵后，就保证了这个特征点最小距离内没有其他特征点。
              corners(y_corners-min_dist:y_corners+min_dist,x_corners-min_dist:x_corners+min_dist) = corners(y_corners-min_dist:y_corners+min_dist,x_corners-min_dist:x_corners+min_dist).*Cake;
             
          % 如果小teil中的特征数没有超过一个teil的最大特征数N,那么就将特征加入进去   
          if acc_array(y_acc,x_acc) < N
              acc_array(y_acc,x_acc) = acc_array(y_acc,x_acc)+1;
              % 我们要得到原始图像下的坐标，而现在corner矩阵为了方便计算扩充了一圈，所以现在记录特征点要减去这一圈对应的坐标。
              features(:,count) = [x_corners-min_dist;y_corners-min_dist];
              count = count+1;
          end
      end
      % all(_,1)检测每一列并返回一个行矩阵，这里如果一列都是0元素，那行矩阵的该元素为1.
      % 消除所有的0列。
      features(:,all(features==0,1)) = []; 
      
      % *************** Plot ***************
      if P.Results.do_plot == true
          figure
          imshow(input_image);
          hold on
          plot(features(1,:),features(2,:),'*');
      end
  end
  ```
  

## 第二次作业

### 2.1 Point_Correspondence

计算2个图片之间的对应点：

```matlab
% 输入：I1,I2是2个灰度的图片
% Ftp1,Ftp2分别是2张图片所有的特征的坐标（x,y）维度是[2,N],N是特征个数
% varargin是可选的参数
function [window_length, min_corr, do_plot, Im1, Im2] = point_correspondence(I1, I2, Ftp1, Ftp2, varargin)
    % In this function you are going to compare the extracted features of a stereo recording
    % with NCC to determine corresponding image points.
    
    % *************** Input parser ***************
    % 检验可选参数的输入是否符合要求
    p = inputParser;
    addOptional(p,'window_length',25,@(x) isnumeric(x) && (mod(x,2)==1) &&(x>1));
    addOptional(p,'min_corr',0.95,@(x) isnumeric(x) &&(x>0) && (x<1));
    addOptional(p,'do_plot',false, @islogical);
    
    p.parse(varargin{:});
    
    window_length = p.Results.window_length;
    min_corr = p.Results.min_corr;
    do_plot = p.Results.do_plot;
    
    Im1 = double(I1)
    Im2 = double(I2)
    
    % *************** Feature preparation准备 ***************
    % 消除离边界太近的参数，并返回剩下参数的个数
    % 找到一个范围，在这个范围里的都要，在这个范围外的都舍弃
    range_x_begin  = ceil(window_length/2);
    range_x_end    = size(I1,2) - floor(window_length/2);   
    range_y_begin  = ceil(window_length/2);
    range_y_end    = size(I1,1) - floor(window_length/2);
    
    % 将在range外的特征值都变成1
   	% logical(A)将数据A中所有非0值都变成逻辑数1，这里ind1就会变成一个2XN的1/0逻辑数组
   	% 注意logical返回的是逻辑值！！！
	ind1 = logical([Ftp1(1,:)<range_x_begin; Ftp1(1,:)>range_x_end; Ftp1(2,:)<range_y_begin; Ftp1(2,:)>range_y_end]);
	% 将这个2XN的1/0逻辑数组变成1XN的1/0逻辑数组
	% any(ind1,1)后面的1表示检测第一个维度（列）如果有一个非0数就返回1。
    ind1 = any(ind1,1);
    % 1XN逻辑数组中1即代表这个特征值在数组外，令该特征值为空，即在数组中删除了这个值。
    % 加入C是一个逻辑数组[1,1,0,1],那么A(:,C)只会返回C中逻辑值为1的部分
    Ftp1(:,ind1) = [];   
    
    % 原理同上处理第二个图片
    ind2 = logical([Ftp2(1,:)<range_x_begin; Ftp2(1,:)>range_x_end; Ftp2(2,:)<range_y_begin; Ftp2(2,:)>range_y_end]);
    ind2 = any(ind2,1);
    Ftp2(:,ind2) = [];   
    
    no_pts1  = size(Ftp1,2);
    no_pts2  = size(Ftp2,2);
    
    % *************** Normalization ***************
    % 用一、4.的SSD方法将每一个窗口里的图片信息正则化，并将正则后的图片灰度强度按列存在Mat_feat_1/2中
   	% 一个窗口的大小，窗口的正中间是特征的坐标。每一个特征都有一个自己的窗口
    win_size = -floor(window_length/2):floor(window_length/2);
    % 每一个特征的窗口里的正则值都按一列保存在矩阵Mat_feat_1中
    Mat_feat_1 = zeros(window_length*window_length,no_pts1);
    Mat_feat_2 = zeros(window_length*window_length,no_pts2);
   
   	% 同时遍历2张图片所有的特征，因为2张图片的特征数不一致，所以取大的那个
    for i = 1:max(no_pts1,no_pts2)
        if i <= no_pts1
        	% win_size是一个类似[-2,-1,0,1,2]的1维矩阵，矩阵+1个值=矩阵每个元素加上这个值
        	% 加上特征的x,y坐标后，如x=1，y=1就会变成[-1,0,1,2,3]的一个窗口
            win_x_size = win_size + Ftp1(1,i);
            win_y_size = win_size + Ftp1(2,i);
            % 得到窗口的大小后，获得这个窗口内的图片的灰度强度值
            window = Im1(win_y_size,win_x_size);
            % 将这个窗口内的图片参数正则化
            Mat_feat_1(:,i) = (window(:)-mean(window(:)))/std(window(:));
        end
        
        if i <= no_pts2
            win_x_size = win_size + Ftp2(1,i);
            win_y_size = win_size + Ftp2(2,i);
            window = Im2(win_y_size,win_x_size);    
            Mat_feat_2(:,i) = (window(:)-mean(window(:)))/std(window(:));
    end
    
    % *************** NCC calculations ***************
    % 根据各个窗口中灰度强度，用一、4.的NCC方法的公式计算NCC矩阵，要注意2张图片的顺序
    NCC_matrix = 1/(window_length*window_length-1)*Mat_feat_2'*Mat_feat_1;
    % NCC矩阵中的每个值都是相关度
    % NCC矩阵中所有相关度小于最小阈值的值都设置为0
    NCC_matrix(NCC_matrix<min_corr) = 0;
    % 将相关度按降序排列
    % [B,I]=sort(A),B是A排列后的矩阵，I描述了 A 的元素沿已排序的维度在 B 中的排列情况，即索引
    % B=A(I)
    [sorted_list,sorted_index] = sort(NCC_matrix(:),'descend');
    % 那些小于最小阈值的值（即=0）丢弃
    sorted_index(sorted_list==0) = [];
    
    % *************** Correspondeces ***************
    % 用上面储存了2个图片各特征点之间的相关度的矩阵NCC_matrix和它的降值排列索引sorted_index来确定相关图像点corresponding image points
    
    % 将2张图片中2个相关的点保存在矩阵cor=[x1,y1,x2,y2]中，所以这里是4行。
    % 相关点是根据2个特征点的窗口内灰度强度的相关性确定的，所以是min(no_pts1,no_pts2)列
    cor = zeros(4,min(no_pts1,no_pts2));
    % 记录相关点的个数
    cor_num = 1;
    
    % numel(A)返回数组A中的元素数目
    for i = 1:numel(sorted_index)  
    	% NCC矩阵中所有相关度小于最小阈值的值都设置为0了，所以遇到这些特征点直接跳过就行
        if(NCC_matrix(sorted_index(i))==0)
            continue;
        else
        	% 找到我们现在要处理的2个特征点的相关度在NCC_matrix中的
        	% [row,col]=ind2sub(sz,ind)将线性索引转换为下标，返回数组row和col分别是大小为sz的矩阵的线性索引ind对应的等效行和列下标。
        	% sz 是包含两个元素的向量，其中 sz(1) 指定行数，sz(2) 指定列数
        	% 比如sz=[3 3],即一个3X3的矩阵。第二个线性索引ind=[2],那么row=2,col=1。可见线性索引顺序是竖着下来的。而矩阵下标是横着数过来的
            [Idx_fpt2,Idx_fpt1] = ind2sub(size(NCC_matrix),sorted_index(i));
        end
		% 如果一个相关点被找到，就将该列设为0，保证图片1中的1个特征只会映射图片2中的1个特征。
		% NCC_matrix中的x,y包含了第二个图片中的点X和第一个图片中的点Y之间的correlation相关性
        NCC_matrix(:,Idx_fpt1) = 0;
        % Ftp1,Ftp2分别是2张图片所有的特征的坐标（x,y）维度是[2,N],N是特征个数
        % 将图片1中第Idx_fpt1个特征点和图片2中第Idx_fpt2个特征点，加入我们的相关点
        cor(:,cor_num) = [Ftp1(:,Idx_fpt1);Ftp2(:,Idx_fpt2)];
        cor_num = cor_num+1;
    end
    % 因为cor初始化为min(no_pts1,no_pts2)列，所以后面有一堆为[0,0,0,0]的点，删掉他们。
    cor = cor(:,1:cor_num-1)
    
    % *************** Visualize the correspoinding image point pairs ***************
    if do_plot
    	% 创建图床窗口，窗口名字是Punkt-Korrespondenzen
        figure('name', 'Punkt-Korrespondenzen');
        % 用无符号整数显示灰度图片I1
        imshow(uint8(I1))
        hold on
        % 给第一张图中的相关点画上红星
        plot(cor(1,:),cor(2,:),'r*')
        imshow(uint8(I2))
        % 将上面的图片透明度设置为0.5
        alpha(0.5);
        hold on
        % 给第二张图中的相关点画上绿星
        plot(cor(3,:),cor(4,:),'g*')
        % 给两个相关点之间画上连线
        for i=1:size(cor,2)
            hold on
            x_1 = [cor(1,i), cor(3,i)];
            x_2 = [cor(2,i), cor(4,i)];
            line(x_1,x_2);
        end
        hold off
    end
end
```



## 第三次作业

### 3.1八点算法

```matlab
function [EF] = epa(correspondences, K)
    % Depending on whether a calibrating matrix 'K' is given,
    % this function calculates either the essential or the fundamental matrix    
    % with the eight-point algorithm.
    % x1是第一张图片上的特征点，x2是第二张图片上的特征点。加上一个维度的1是为了计算
    x1 = [correspondences(1:2,:);ones(1,size(correspondences,2))];
    x2 = [correspondences(3:4,:);ones(1,size(correspondences,2))];
    % nargin 针对当前正在执行的函数，返回函数调用中给定函数输入参数的数目。
    % 如果数目=1,说明Calibration Matrix标定矩阵没有给定
    if(nargin>1)
    	x1 = K\x1;
       	x2 = K\x2;
    end
    % kron 计算 Kronecker 张量积。
    % A是2个图片上对应特征点两两相乘后的矩阵
    A = (kron(x1,ones(3,1)).*kron(ones(3,1),x2))';
    % 奇异值分解，我们只需要等到V
    [~,~,V] = svd(A);
    
    % *************** Estimation of the matrices ***************
    % estimate估计essential/fundamental matrix
    G = reshape(V(:,end),3,3);   
    [U,S,V] = svd(G);
    % 如果给了标定矩阵K,那么求的是essential matrix本质矩阵
    if(nargin>1)
        EF = U * diag([1,1,0]) * V';
    % 如果没有给标定矩阵K,那么求的是fundamental matrix基本矩阵
    else
        S(3,3) = 0;
        EF = U * S * V';
    end
end
```

### 3.2RanSac算法

- [Random Sample Consensus](https://zhuanlan.zhihu.com/p/62238520)随机采样一致算法是从一组含有“外点”(outliers)的数据中正确估计数学模型参数的迭代算法。
  - 外点”一般指的的数据中的噪声，所以，RANSAC也是一种“外点”检测算法。
  - RANSAC算法是一种不确定算法，它只能在一种概率下产生结果，并且这个概率会随着迭代次数的增加而加大
  - **基本的假设**：
    - 数据是由“内点”和“外点”组成的。“内点”就是组成模型参数的数据，“外点”就是不适合模型的数据
    - 在给定一组含有少部分“内点”的数据，存在一个程序可以估计出符合“内点”的模型。
  
- RANSAC是通过反复选择数据集去估计出模型，一直迭代到估计出认为比较好的模型。步骤如下：
  1. 选择出可以估计出模型的最小数据集；(对于直线拟合来说就是两个点，对于计算Homography矩阵就是4个点)
  
     Wähle zufällig k = 8 Datenpunkte
  
  2. 使用这个数据集来计算出数据模型；
  
     Bestimme Parameter F via Achtpunktalgorithmus
  
  3. 将所有数据带入这个模型，计算出“内点”的数目；(累加在一定误差范围内的适合当前迭代推出模型的数据)
  
     Bestimme Menge M von Punkten, die Teil des Consensus-Set (Innerhalb des Toleranzmaß容忍度) sind.
  
  4. 比较当前模型和之前推出的最好的模型的“内点“的数量，记录最大“内点”数的模型参数和“内点”数；
  
     Speichere Parameter F und die Menge M falls die neue Menge M größer ist als die Alte.
  
  5. 重复1-4步，直到迭代结束或者当前模型已经足够好了(“内点数目大于一定数量”)。
  
- 迭代次数：
  $$
  \begin{align}
  & k=8(这里用的8)\\
  & s=\frac{log(1-p)}{log(1-(1-\epsilon)^k)}
  \end{align}
  $$
  
  - $k$：每次计算所用点的个数
  - $s$：所需要的迭代次数
  - $1-\epsilon$：内点在数据中的占比：
  - $p$：我们希望RANSAC得到正确模型的概率
  
- 代码实现：

  ```matlab 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  function sd = sampson_dist(F, x1_pixel, x2_pixel)
      % This function calculates the Sampson distance based on the fundamental matrix F
      e3_hat = [0 -1 0;1 0 0; 0 0 0]
      sd = sum(x2_pixel.*(F*x1_pixel)).^2 ./ (sum((e3_hat*F*x1_pixel).^2) + sum((e3_hat*F'*x2_pixel).^2));
  end
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  function [epsilon, p, tolerance, x1_pixel, x2_pixel] = F_ransac(correspondences, varargin)
  	% *************** Input Parameters ***************
      % This function implements the RANSAC algorithm to determine 
      % robust corresponding image points
      P = inputParser;
      % p: desired probability that the algorithm yields a set of corresponding image points where no outliers are included
      % epsilon: estiamted probability that a randomly chosen pair of corresponding image points is an outlier
      % tolerance: tolerance in which a pair of corresponding image points gets assessed as fitting to the model(belonging to the consensus-set)
      P.addOptional('p', 0.5, @(x)isnumeric(x) && x > 0 && x < 1)
      P.addOptional('epsilon', 0.5, @(x)isnumeric(x) && x > 0 && x < 1);
      P.addOptional('tolerance', 0.01, @isnumeric);
      P.parse(varargin{:});
      p           = P.Results.p;
      epsilon     = P.Results.epsilon;
      tolerance   = P.Results.tolerance;
  	% x1，x2分别储存第一张图片和第二张图片的特征点
      if size(correspondences,1)==4
          x1_pixel = [correspondences(1:2,:);ones(1,length(correspondences))];
          x2_pixel = [correspondences(3:4,:);ones(1,length(correspondences))];
      end
      
      % *************** RANSAC algorithm preparation ***************
      % 每次计算所用点的个数
      k = 8;
      % 迭代次数计算
      s = log(1-p)/log(1-(1-epsilon)^k);
      % largest_set_size：保存当前最大的consensus-set的correspondences数目
      largest_set_size = 0;
      % largest_set_dist：保存当前最大的consensus-set的sampson-distances误差
      largest_set_dist = Inf;
      % largest_set_F：保存当前最大的consensus-set的fundamental matrix基本矩阵
      largest_set_F = zeros(3,3);
      
      % *************** RANSAC algorithm ***************
      % 迭代的次数前面我们已经预估了
      for i=1:s
      	% 任意取k个点作为子集
          consensus_set = correspondences(:,randperm(size(correspondences,2), k));
          % 用八点算法得到基本矩阵
          [F] = epa(consensus_set);
          % 计算sampson误差
          sd = sampson_dist(F, x1_pixel, x2_pixel);
          % 排除那些不符合的外点outlier
          sd = sd(sd<tolerance);
          set_size = size(sd,2);
          % 选取那些内点最多的子集算出的基本矩阵
          if set_size > largest_set_size
              largest_set_size = set_size;
              largest_set_dist = sum(sd);
              largest_set_F = F;
              correspondences_robust = consensus_set;
          end
          % 如果内点个数相同，就选一致性更好的那一个
          if set_size == largest_set_size
              if sum(sd) < largest_set_dist
                  largest_set_size = set_size;
                  largest_set_dist = sum(sd);
                  largest_set_F = F;
                  correspondences_robust = consensus_set;
              end
          end
      end
      
      % *************** Visualize robust corresponding image points ***************
      figure
      figure('name', 'Punkt-Korrespondenzen nach RANSAC');
      imshow(uint8(IGray1))
      hold on
      plot(correspondences_robust(1,:),correspondences_robust(2,:),'r*')
      imshow(uint8(IGray2))
      alpha(0.5);
      hold on
      plot(correspondences_robust(3,:),correspondences_robust(4,:),'g*')
      for i=1:size(correspondences_robust,2)
          hold on
          x_1 = [correspondences_robust(1,i), correspondences_robust(3,i)];
          x_2 = [correspondences_robust(2,i), correspondences_robust(4,i)];
          line(x_1,x_2);
      end
      hold off
      
  	% *************** Calculate essential matrix ***************
      load('K.mat');
      E = epa(correspondences, K)
  end  
  ```

  

## 第四次作业

### 4.1Euclidean Movement

从本质矩阵 E 得到Euclidean Movement欧几里得运动 R 和 T.

理论参见(三、4)

```matlab
function [T1, R1, T2, R2, U, V] = TR_from_E(E)
    % This function calculates the possible values for T and R 
    % from the essential matrix
    [U,S,V]=svd(E);
    if det(U) < 0
	    U = U*diag([1 1 -1]);
    end
    
    if det(V) < 0
	    V = V*diag([1 1 -1]);
    end

    R_1=[0,-1,0;
          1,0,0;
          0,0,1];
    R_2=[0,1,0;
          -1,0,0;
          0,0,1];
      
    R1=U*R_1'*V';
    R2=U*R_2'*V';
    T1_hat = U*R_1*S*U';
    T1 = [T1_hat(3,2);T1_hat(1,3);T1_hat(2,1);];
    T2_hat = U*R_2*S*U';
    T2 = [T2_hat(3,2);T2_hat(1,3);T2_hat(2,1)];

    
end
```

### 4.2 3D Reconstruction

- 辅助函数：

    ```matlab
    function M1 = M1_erstellen(R, T, N, x1, x2)
        M1 = zeros(3*N,N+1);
        for i = 1:N
            M1(3*i-2:3*i, i) = cross(x2(:,i),R*x1(:,i));
            M1(3*i-2:3*i, N+1) = cross(x2(:,i),T);
        end
    end
    
    function M2 = M2_erstellen(R, T, N, x1, x2)
        M2 = zeros(3*N,N+1);
        for i = 1:N
            M2(3*i-2:3*i, i) = cross(x1(:,i),R'*x2(:,i));
            M2(3*i-2:3*i, N+1) = -cross(x1(:,i),R'*T);
        end
    end
    
    function d = d_erstellen(M)
        [~,~,V] = svd(M);
        d = V(:,end);
        d = d./d(end,1);
    end
    
    function pos_num = count(d_cell)
        count = find(d_cell>0);
        pos_num = length(count);
    end
    ```

- 3D Reconstruction

    ```matlab
    function [T, R, lambda, P1, camC1, camC2] = reconstruction(T1, T2, R1, R2, correspondences, K)
        % *************** Preparation ***************
        %% T,R的组合有4种情况，建立2个元组，分别对应表示这4种情况
        T_cell = {T1, T2, T1, T2};
        R_cell = {R1, R1, R2, R2};
        %% 将特征点从4XN维的数组种读取出来
        n = length(correspondences);
        x1 = K\[correspondences(1:2,:); ones(1, n)];
        x2 = K\[correspondences(3:4,:); ones(1, n)];
        %% d_cell用于保存各个特征点的深度信息
        n = size(correspondences,2);
        d_cell = {zeros(n,2), zeros(n,2), zeros(n,2), zeros(n,2)};
    	
    	
    	% *************** Reconstruction ***************
        N = size(correspondences,2);
        M1_R1T1 = M1_erstellen(R_cell{1}, T_cell{1}, N, x1, x2);
        M1_R1T2 = M1_erstellen(R_cell{1}, T_cell{2}, N, x1, x2);
        M1_R2T1 = M1_erstellen(R_cell{3}, T_cell{1}, N, x1, x2);
        M1_R2T2 = M1_erstellen(R_cell{3}, T_cell{2}, N, x1, x2);
        
        M2_R1T1 = M2_erstellen(R_cell{1}, T_cell{1}, N, x1, x2);
        M2_R1T2 = M2_erstellen(R_cell{1}, T_cell{2}, N, x1, x2);
        M2_R2T1 = M2_erstellen(R_cell{3}, T_cell{1}, N, x1, x2);
        M2_R2T2 = M2_erstellen(R_cell{3}, T_cell{2}, N, x1, x2);
        
        M1 = M1_R2T2;
        M2 = M2_R2T2;
        
        d1_R1T1 = d_erstellen(M1_R1T1);
        d1_R1T2 = d_erstellen(M1_R1T2);
        d1_R2T1 = d_erstellen(M1_R2T1);
        d1_R2T2 = d_erstellen(M1_R2T2);
        
        d2_R1T1 = d_erstellen(M2_R1T1);
        d2_R1T2 = d_erstellen(M2_R1T2);
        d2_R2T1 = d_erstellen(M2_R2T1);
        d2_R2T2 = d_erstellen(M2_R2T2);
        
        d_cell = {[d1_R1T1(1:N,1) d2_R1T1(1:N,1)], [d1_R1T2(1:N,1) d2_R1T2(1:N,1)], [d1_R2T1(1:N,1) d2_R2T1(1:N,1)], [d1_R2T2(1:N,1) d2_R2T2(1:N,1)]};
        % 创建一个数组pos_num()，分别计算不同R,T组合下的深度，需要都为正才是最终解。
        pos_num(1) = count(d_cell{1});
        pos_num(2) = count(d_cell{2});
        pos_num(3) = count(d_cell{3});
        pos_num(4) = count(d_cell{4});
        % 波浪号作为函数输入\输出参数的占位符，表示忽略该参数。
        % 当函数有多个输出，但某个输出值不需要时，可以将其用~代替。
        % 正值最多的那一组就是我们要的R,T组合
        [~, index] = max(pos_num);
        T = T_cell{index};
        R = R_cell{index};
        % 各个特征点的深度
        lambda = d_cell{index};    
        
    
    	% *************** Calculation and visualization of the 3D points and the cameras ***************
    	% 计算第一张图片的x1的世界坐标
        lambda1 = [lambda(:,1), lambda(:,1),lambda(:,1)]; 
        P1 = x1.*lambda1';
        
        for i = 1:N
            scatter3(P1(1,i), P1(2,i), P1(3,i));
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            grid on
            text(P1(1,i)+0.01, P1(2,i)+0.01, P1(3,i), num2str(i));
            hold on
        end
        % 用第一个相机坐标cameraframe1的corners去计算第二个相机坐标cameraframe2
        camC1 = [-0.2  0.2  0.2 -0.2;
                  0.2  0.2 -0.2 -0.2;
                  1    1    1    1  ];
        camC2 = R\(camC1-T);
        
        plot3(camC1(1,:), camC1(2,:), camC1(3,:), 'b');
        plot3(camC2(1,:), camC2(2,:), camC2(3,:), 'r');
        
        campos([43 -22 -87]);
        camup([0 -1 0]);
    end
    ```

### 4.3 Back Projection

- The quality of the 3D reconstruction is going to be quantitatively determined via the error of the back projection into camera 2

  ```matlab
  function [repro_error, x2_repro] = backprojection(correspondences, P1, Image2, T, R, K)
      % This function calculates the mean error of the back projection
      % of the world coordinates P1 from image 1 in camera frame 2
      % and visualizes the correct feature coordinates as well as the back projected ones.
      P2 = R*P1+T; %Weltkoordinaten in Cameraframe 2,相机和相的移动是相反的
      x2_berechnen = K*P2;
      x2_repro = x2_berechnen./x2_berechnen(3,:);
      imshow(Image2);
      x2 = correspondences(3:4,:);
      hold on
      N = size(x2,2);
      for i=1:N
          plot(x2(1,i),x2(2,i),'o');
          text(x2(1,i)+10, x2(2,i), num2str(i));
          plot(x2_repro(1,i),x2_repro(2,i),'x');
          text(x2_repro(1,i)+10, x2_repro(2,i), num2str(i));
      end
      
      repro_error = (1/N)*sum(sqrt((sum((x2-x2_repro(1:2,:)).^2)')'));
      
  end
  ```






# 五、Optical Flow

学习自：http://www.cs.cmu.edu/~16385/s17/

- 光流问题定义：

  给定两个连续consecutive的图像帧$\{I_t,I_{t+1}\}$，估计每一个像素的运动$\{v(p_i),u(p_i)\}$

  (u,v)表示像素点在水平和垂直方向上的运动速度分量，所以光流表示了图像中像素点的运动方向和速度

- 光流应用场景：
  - 视频稳定（Video Stabilization）：通过计算光流，可以估计相邻帧之间的相机运动，从而实现视频图像的稳定，消除由于相机抖动引起的图像抖动效应。
  - 物体跟踪（Tracking）：光流可以用于物体跟踪，帮助定位和追踪图像或视频中的感兴趣物体，通过计算物体的光流来估计其运动轨迹。
  - 立体匹配（Stereo Matching）：立体视觉中的光流计算可以用于在左右视图之间找到匹配点，从而估计场景中的深度信息。
  - 图像配准（Registration）：光流也可以用于图像配准，即将多幅图像或图像与模板进行对齐，以便进行进一步的分析或处理。

## 1. Brightness Constancy

### 1.1 两个假设

- Brightness constancy假设：

  ![image-20230720002307204](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/202307200023269.png)

  <center style="color:#C125C0C0">图1</center>
  
  scene point在image sequence中保持亮度不变
  $$
  I(x(t),y(t)t)=Constant \tag{1}
  $$
  
- Small motion

  ![image-20230720010642707](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/202307200106761.png)

  <center style="color:#C125C0C0">图2</center>
  
  对于真正的小空间运动，两个consecutive image frame之间的光强是相同的：
  $$
  I(x+u\delta t,y+v\delta t,t+\delta t)=I(x,y,t) \tag{2}
  $$
  
  - $(u,v)$：光流速度 optical flow(velocity)
  - $(\delta x,\delta y)=(u\delta t,v\delta t)$：变换displacement

### 1.2 亮度不变等式

- 由上面两个假设可以得到Brightness Constancy Equation
  $$
  \frac{dI}{dt}=\frac{\partial I}{\partial x}\frac{dx}{dt}+\frac{\partial I}{\partial y}\frac{dy}{dt} +\frac{\partial I}{\partial t}=0 \tag{3}
  $$

  - 推导公式见：http://www.cs.cmu.edu/~16385/s17/Slides/14.1_Brightness_Constancy.pdf

    是由2式泰勒展开得到的

- 3式的shorthand速记法 noatation：
  $$
  I_xu+Iyv+I_t=0\tag{4}
  $$

  - $u$：x-flow velocity； $v$：y-flow velocity
    - 光流（Optical Flow）可以理解为场景中像素的运动速度。
    - 在光流问题中，只有u,v是未知的，其他量可用如下方法得到
  - $I_x，I_y$：在p点的image gradient
  
    - 可以通过Forward difference, Sobel filter，Scharr filter计算
  - $I_t$：temporal gradient

    - 时间梯度，是计算机视觉中的一个概念，用于描述视频序列中相邻帧之间的变化率

    - 可以通过frame differencing帧差分计算

​      ![image-20230720020348984](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/202307200203050.png)

<center style="color:#C125C0C0">图3：帧差分例子</center>

- 向量形式:
  $$
  \nabla I^T\mathbf{V}+I_t=0 \tag{5}
  $$

  - $\nabla I^T$: 1x2 亮度梯度
  - $\mathbf{v}$： 2x1光流速度



## 2. Optical Flow: Constant Flow

下面解决**如何求4式中光流$\{u,v\}$**的问题。

- 既然1个Brightness Constancy Equation（4式）无法求得光流，那就假设该点的surrrounding patch（比如5x5的周围块）是**constant flow**的，即假设：

  - Flow is locally smooth
    - 局部平滑：是在假设图像上相邻的像素点的运动方向和速度应该是相似的，或者说局部像素点的运动变化应该是连续和一致的。
  - Neighboring pixels have same displacement

- 用5x5的image patch，我们可以得到25个Brightness Constancy Equation：

  ![image-20230720022121981](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/202307200221036.png)

  <center style="color:#C125C0C0">图4</center>

- 将上面的25个方程构建为最小二乘问题：
  $$
  \hat{x}=\mathop{arg\ min}_x||Ax-b||^2 \tag{6}
  $$

- 最小二乘问题可以等价为矩阵求解：

  - 正规方程形式：
    $$
    A^TA\hat{x}=A^Tb \tag{7}
    $$

  - 矩阵形式：
    $$
    x=(A^TA)^{-1}A^Tb \tag{8}
    $$
  
- 何时7/8式有解？
  
  - $A^TA$需要时invertible可逆的
  
  - $A^TA$不应该太小
  
  - $A^TA$是well conditioned
  
    即$\frac{\lambda_1}{\lambda_2}$不能太大，$\lambda_1$是较大的那个特征值
  
- btw,在Harris Corner Detector也用到过$A^TA$

  - 当$\lambda_1，\lambda_2$都很大时，是角点。这也是Lucas-Kanade optical flow最有效的地方
  - 角点是至少有2个不同方向梯度的区域
  - 角点也是易于计算光流的点

- 与角点相对的，如果取的patch是aperture光圈中的一条线呢？

  - 这时很难判断线是往哪运动的

    所以希望patche是包含不同梯度的，以此来避免aperture problem光圈问题

    > Aperture problem是光流计算中的一个重要挑战，它源自于在计算图像中像素点的运动时，我们只能观察到沿着某个特定方向（光流方向）的运动信息。
    >
    > 在存在遮挡和复杂运动的情况下，aperture problem是很难被解决的。

  - 例子：

    在一个垂直条纹图案中，条纹的运动方向可能是水平的。然而，由于只能观察到垂直方向的灰度变化（在每一条垂直条纹上），我们在这个区域内只能得到v方向的运动信息，而无法得知条纹的水平运动。因此，在这种情况下，我们可以恢复v的光流分量，但无法直接恢复u的光流分量。
## 3. Optical Flow: Horn-Schunck

- Horn-Schunck光流算法基本信息：

  - 基于brightness constancy和small motion两个假设
  - 使用光流的平滑性smooth约束：flow can vary from pixel to pixel
    - most objects in the world are rigid or deform elastically有弹性的 moving together coherently前后一致的
  - 是一种global method/dense的，即考虑了图像上所有的像素点，而不是只计算某些选定的稀疏像素点

- Horn-Schunck光流算法依据下面2个约束来计算光流

  1. Enforce **brightness constancy**，即上面的4式$I_xu+Iyv+I_t=0$

     对于每一个像素(i,j)：
     $$
     \mathop{min}_{u,v}[I_xu_{ij}+I_yv_{ij}+I_t]^2 \tag{9}
     $$
     对于刚体的一个位移，我们希望找到光流满足：

     ![image-20230721014037311](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/202307210140426.png)

     <center style="color:#C125C0C0">图7</center>

  2. Enforce **smooth flow field**

     比如只考虑横向u：左边的$u_{i,j}$，右边的$u_{i+1,j}$

     约束下：
     $$
     \mathop{min}_u(u_{i,j}-u_{i+1,j}) \tag{10}
     $$
     

- 将上面两个约束何在一起有公式：
  $$
  \mathop{min}_{u,v}\sum_{i,j}\{E_s(i,j)+\lambda E_d(i,j)\} \tag{11}
  $$

  - Smoothness部分：
    $$
    E_e(i,j)=\frac{1}{4}\Big[(u_{ij}-u_{i+1,j})^2+(u_{ij}-u_{i,j+1})^2+(v_{ij}-v_{i+1,j})^2+(v_{ij}-v_{i,j+1})^2\Big] \tag{12}
    $$

  - Brightness Constancy部分：
    $$
    E_d(i,j)=\Big[I_xu_{ij}+I_yv_{ij}+I_t \Big]^2 \tag{13}
    $$

  - $\lambda$:weight

- 最后的算法就是11式对u,v求偏导得到迭代公式，然后不断迭代至收敛，得到每个像素的光流

  ![image-20230721015956267](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/202307210200412.png)

  <center style="color:#C125C0C0">图6：Horn-Schunck光流算法</center>

  - $\hat{u}_{kl}$：像素(k,l)的新光流值
  - $\overline{u}_{kl}$：像素(k,l)的旧光流均值

  迭代就是让他们之间的差异越来越小

## 4. Alignment: LucasKanade

Lucas-Kanade 加法对准（Additive Alignment）是 Lucas-Kanade 光流算法的一种变体，用于图像对准和图像配准任务。

- Lucas-Kanade 加法对准是一种基于光流的对准方法。与传统的 Lucas-Kanade 光流估计方法不同，Lucas-Kanade 加法对准允许图像之间存在亮度的差异。它假设在两幅图像之间，存在一个局部线性变换，即图像上的像素值之间存在一个线性关系。

> 图像对准（Image Alignment）：图像对准是将多幅图像中的特定特征或目标对齐到一个参考图像的过程。通常，这些图像可能有不同的尺度、角度、旋转或平移，或者存在轻微的形变和畸变。图像对准的目标是将这些图像进行变换，使得它们在某种准则下对齐，以便进行后续的分析、比较或叠加显示。图像对准常用于图像融合、图像拼接、图像叠加等应用，例如在航拍图像拼接中，将多幅航拍图像对齐形成一张大的全景图。
>
> 图像配准（Image Registration）： 图像配准是将多幅图像进行空间变换，使得它们在相同坐标系下对齐的过程。这些图像可能来自不同的传感器、不同时间或不同视角。图像配准的目标是找到一种变换方法，将这些图像映射到同一个坐标系下，以便进行像素级的对应和比较。图像配准常用于遥感图像处理、医学图像处理、多视角图像融合等应用，例如在医学影像中，将不同模态的图像进行配准，以便医生可以更好地比较和分析。

### 4.1 数学：

**w**:2d image transformation

$\mathbf{x}$:2d image coordinate

$\mathbf{p}=[p_1,p_2,\cdots]$: 变换参数warp parameters

$I(x')=I(\mathbf{W(x;p)})$:warped image

- 平移变换Translation：
  $$
  \begin{align}
  \mathbf{W(x;p)}&=\left [\begin{array}{cccc}
  x +p_1 \\
  y +p_2   \\
  \end{array}\right]\\
  &=\left [\begin{array}{cccc}
  1 & 0 & p_1 \\
  0 & 1 & p_2   \\
  \end{array}\right]\left [\begin{array}{cccc}
  x\\
  y\\
  1
  \end{array}\right]
  \end{align}
  $$

- 仿射变换Affine ：
  $$
  \begin{align}
  \mathbf{W(x;p)}&=\left [\begin{array}{cccc}
  p_1x+p_2y+p_3 \\
  p_4x+p_5y+p_6   \\
  \end{array}\right]\\
  &=\left [\begin{array}{cccc}
  p_1+p_2+p_3 \\
  p_4+p_5+p_6   \\
  \end{array}\right]\left [\begin{array}{cccc}
  x\\
  y\\
  1
  \end{array}\right]
  \end{align}
  $$

### 4.2 问题定义：

$$
\mathop{min}_p\sum_x[I(\mathbf{W(x;p)}-T(\mathbf{x})]^2 \tag{14}
$$

- 目标：找到一个变换参数warp parameters $\mathbf{p}$使得14式最小化

  ![image-20230722164426604](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/lucasKandade%E9%97%AE%E9%A2%98%E5%AE%9A%E4%B9%89.png)

  <center style="color:#C125C0C0">图7</center>

- $I$: warped image

- T:template image

  它通常是一个小尺寸的图像，代表了要在其他图像中寻找或匹配的目标形状、纹理或特征。

### 4.3 公式推导

14式是一个非线性方程很难求解

- 14式可以写成小扰动perturbation的形式：
  $$
  \mathop{min}_p\sum_x[I(\mathbf{W(x;p+\Delta p)}-T(\mathbf{x})]^2 \tag{15}
  $$
  但任然是非线性方程

- 对15式泰勒展开并近似可得：
  $$
  \mathop{min}_{\Delta \mathbf{p}}\sum_{\mathbf{x}}[I(\mathbf{W(x;p)}+\nabla I\frac{\partial\mathbf{W}}{\partial\mathbf{p}}\Delta\mathbf{p}-T(\mathbf{x})]^2\tag{16}
  $$

  - $\nabla I=\frac{\partial I(\mathbf{W}(x;p))}{\partial x'}$

  - $\frac{\partial\mathbf{W}}{\partial\mathbf{p}}$:
    $$
    \frac{\partial\mathbf{W}}{\partial\mathbf{p}}=\left [\begin{array}{cccc}
    \frac{\partial\mathbf{W}_x}{\partial\mathbf{p}_1} & \frac{\partial\mathbf{W}_x}{\partial\mathbf{p}_2}&\cdots & \frac{\partial\mathbf{W}_x}{\partial\mathbf{p}_N}  \\
    \frac{\partial\mathbf{W}_y}{\partial\mathbf{p}_1} & \frac{\partial\mathbf{W}_y}{\partial\mathbf{p}_2}&\cdots & \frac{\partial\mathbf{W}_y}{\partial\mathbf{p}_N}  \\
    \end{array}\right]
    $$
  
- 再把16式里的项换个位置

  ![image-20230722185652389](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/LucasKanadde%E5%85%AC%E5%BC%8F%E6%8E%A8%E5%AF%BC.png)

  <center style="color:#C125C0C0">图8：公式调个位姿</center>

- 最小二乘法least squares approximation

  对于最小二乘问题：
  $$
  \hat{x}=\mathop{arg\ min}_x||Ax-b||^2
  $$
  可以转为下面这个矩阵方程来求解：
  $$
  x=(A^TA)^{-1}A^Tb
  $$

- 最后对图8公式进行最小二乘问题等价转后就得到要求解的公式：

  ![image-20230722190813567](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/LK%E5%85%89%E6%B5%81%E6%9C%80%E5%90%8E%E7%9A%84%E8%BF%AD%E4%BB%A3%E5%85%AC%E5%BC%8F.png)

  <center style="color:#C125C0C0">图9：最后的迭代公式</center>

  这种非线性优化方法叫做Gauss-Newton gradient decent non-linear optimization!

### 4.4 L-K Additive alignment光流算法

![image-20230722194135350](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/L-K%20Additive%20alignment%E7%AE%97%E6%B3%95.png)

## 5. Alignment: BakerMatthews

 BakerMatthews光流法区别于上面属于Additive Alignment（加法对准）的L-K光流法， BakerMatthews光流法属于 Compositional Alignment（合成对准）方法。

> Compositional Alignment（合成对准）：采用合成（composition）的方式对多幅图像进行对准。它假设图像间的变换是由一系列组合的变换构成的。通常，它会使用一个初始的变换模型，然后通过迭代优化的方式逐步改进变换模型，直到得到最佳的对准结果。
>
> Additive Alignment（加法对准）：采用加法（addition）的方式对多幅图像进行对准。它假设图像间的差异可以表示为参考图像加上一个平移项，即图像之间存在亮度的差异。这个平移项可以是常数项，也可以是位置依赖的项。Additive Alignment 的目标是估计这个平移项，使得将多幅图像加上这个平移项后能够对齐到参考图像。

一个例子：打高尔夫

- 合成对准：打一杆后，从球落地点开始打下一杆，如此反复直到打进

- 加法对准：打一杆后，计算下一杆要增加的量，然后回到起点重新打，直到加的量正正好好一杆进洞。

### 5.1 公式推导

 还是要求解4.2的14式这一个问题。

- 与additive alignment的区别

  - Additive alignment：

    即4.3的15式：是incremental perturbation of parameters
    $$
    \mathop{min}_p\sum_x[I(\mathbf{W(x;p+\Delta p)}-T(\mathbf{x})]^2
    $$

  - Compositional Alignment:

    是incremental warps of image
    $$
    \sum_x[I(\mathbf{W(W(x;\Delta p); p)}-T(\mathbf{x})]^2 \tag{17}
    $$

    - 其中$\mathbf{W(W(x;\Delta p); p)}$可写作：$\mathbf{W(x; p)}\circ \mathbf{W(x;\Delta p)}$
    - 最后更新项：$\mathbf{W(x;p)}\leftarrow\mathbf{W(x;p)}\circ\mathbf{W(x;\Delta p)}$

- 17式经过泰勒级数展开后，可得到：
  $$
  \sum_{\mathbf{x}}[I(\mathbf{W(x;p)}+\nabla I\frac{\partial\mathbf{W}(x;0)}{\partial\mathbf{p}}\Delta\mathbf{p}-T(\mathbf{x})]^2\tag{18}
  $$

  - 相比Additive alignment的16式，18式的jacobian$\frac{\partial\mathbf{W}(x;0)}{\partial\mathbf{p}}$是固定的

### 5.2 Schum-Szeliski光流算法

![image-20230722204943821](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/schum-Szeliski%E5%85%89%E6%B5%81%E7%AE%97%E6%B3%95.png)

### 5.3 继续改进算法

- 是否可以令template也被变换(warp)？

  即同时更新17式：$\sum_x[I(\mathbf{W(W(x;\Delta p); p)}-T(\mathbf{x})]^2$中的$T(x)$

  ![image-20230722205322411](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/compositional%E5%92%8Cinverse%20compositional%E7%9A%84%E5%AF%B9%E6%AF%94.png)

  参考高尔夫例子：

  - 合成对准：打一杆后，从球落地点开始打下一杆，如此反复直到打进。

  - 逆合成对准：打一杆后，移动球洞到球落地点，让球掉进去。

  - 加法对准：打一杆后，计算下一杆要增加的量，然后回到起点重新打，直到加的量正正好好一杆进洞。

-  Inverse compositional alignment的性质：

  ![image-20230722205807865](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/%20Inverse%20compositional%20alignment%E7%9A%84%E6%80%A7%E8%B4%A8.png)

### 5.4Baker-Mattews光流算法

![image-20230722210407032](https://raw.githubusercontent.com/Fernweh-yang/ImageHosting/main/img/Baker-Mattews%E5%85%89%E6%B5%81%E7%AE%97%E6%B3%95.png)
