# 一. Supervised Learning监督学习

- 两类问题

  1. Regression problems回归问题Given a picture of a person, we have to predict their age on the basis of the given picture

  2. Classification problems分类问题

     Given a patient with a tumor, we have to predict whether the tumor is malignant or benign

## 1. Regression

### (1)一元线性规划

- Hypothesis假设h

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/supervised%20learning%20model.png?raw=true)

  :比如对于linear Regression线性规划: 

$$
h_\theta(x)=\theta_0+\theta_1x\tag{1}
$$

- cost function:

  解决如何确定$\theta$的问题。
  
  We can measure the accuracy of our hypothesis function by using a **cost function**
  $$
  J(\theta_0,\theta_1)=\frac{1}{2m}\sum_{i=1}^m(\hat{y}_i-y_i)^2=\frac{1}{2m}\sum_{i=1}^m(h_\theta(x_i)-y_i)^2\tag{2}
  $$
  目标是最小化这个cost function。
  
- Gradient descent梯度下降

  不仅可用于最小化cost function J，也可用于最小化其他函数。

  - 通用算法：

    repeat until convergence(不断迭代至收敛): 

  $$
  \theta_j:=\theta_j-\alpha\frac{\partial}{\partial\theta_j}J(\theta_0,\theta_1), \quad for\: j=0\: and\:  j=1\tag{3}\\
  $$
  
  ​	这里 $\alpha$ 是learning rate 
  
  - Subtlety微妙之处：Simultaneous同时发生的。这里两个参数的新值都是基于旧值计算的，两个都算完了再同时代入下一个迭代。
  - learning rate如果
    - 太小，就会导致收敛速度太慢
    - 太大，就可能会导致错过收敛点

### (2)Multivariate Linear Regression多元线性规划

- Hypothesis假设h
  $$
  h_\theta(x)=\theta_0+\theta_1x_1+\theta_2x_2+...+\theta_nx_n=[\theta_0\;\theta_1\;...\;\theta_n]\begin{bmatrix}x_0\\x_1\\\vdots\\x_n\end{bmatrix}
  $$

​		说明：$x_j^{(i)}$=第i个训练样本的第j个参数。

​					$x^{(i)}$=第i个训练样本的输入参数们。

- Cost function:
  $$
  J(\theta)=\frac{1}{2m}\sum_{i=1}^{m}(h_\theta(x^{(i)})-y^{(i)})^2
  $$

- Gradient descent:

  repeat until convergence{
  $$
  \begin{aligned}
  \theta_j:&=\theta_j-\alpha\frac{\partial}{\partial\theta_j}J(\theta)\\
  &= \theta_j-\alpha\frac{1}{m}\sum_{i=1}^{m}(h_\theta(x^{(i)})-y^{(i)})x_j^{(i)}\qquad for\ j:=0\ \cdots n
  \end{aligned}
  $$
  }

  - Feature Scaling特征缩放:Mean Normalization均值归一化

    因为不同特征feature的变化的范围不同，所以使特征统一标准化。这样也可以加速梯度函数收敛。
    $$
    x_i=\frac{x_i-\mu_i}{s_i}
    $$
    其中：$\mu_i$：是所有特征值$x_i$的平均值。

    ​            $s_i$：是特征值变化的范围（最大值-最小值）

- Polynomial Regression多项式回归

  如果我们的Hypothesis函数以一条直线不能很贴合数据时，可以通过加二次quadratic，三次cubic，平方根square root项等来改变曲线的形状，比如：
  $$
  h_\theta(x)=\theta_0+\theta_1x_1+\theta_2\sqrt{x_1}+\theta_3x_1^3
  $$
  

### (3)Normal Equation

另一种让cost function最小化的方法。

- 与Gradient descent对比

  - 上面用到的Gradient descent是通过选择一个学习速率并用迭代的形式来求各个参数$\theta$。

  - Normal Equation是用数学analytically分析地来求解各个参数。
  - Gradient descent适合有很多特征x(i)的时候，Normal Equation在处理很多特征时速度很慢。

- Formula:

  要让cost function J最小，即让$J(\theta)$的导数=0，也即$y=X\theta$

  因为只有方阵才有逆，但我们的特征$x^{(i)}$的个数n和训练样本数m基本不可能相等，所以X不是方阵。为了求解上面的等式，要先让X乘它的转置$X^T$来使他们变成方阵，然后再乘逆。最终得到解：
  $$
  \theta=(X^TX)^{-1}X^Ty
  $$

- 如果$X^TX$noninvertible不可逆

  原因有2：

  1. Redundant Features:即不同特征之间是线性依赖的。

     比如特征x1是米的平方，特征x2是英尺的平法。他们之间只差一个倍数，是linearly dependent的。

  2. Too many features: 即特征x(i)的数目n多过了训练集的样本数m。

  解决方法是

  1. 删除一些features
  2. 或使用regularization

### (4)Overfitting Problem

- 如果我们的hypothesis方程不正确会导致两类问题：

  - **Underfitting**: the form of our hypothesis function h maps poorly to the trend of the data.
    - It is usually caused by a function that is too simple or uses too few features

  - **Overfitting**: a hypothesis function that fits the available data but does not generalize well to predict new data

    -  It is usually caused by a complicated function that creates a lot of unnecessary curves and angles unrelated to the data.

  - 解决方法：

    Reduce the number of features:

    - Manually select which features to keep.
    - Use a model selection algorithm

    Regularization正则化：

    - Keep all the features, but reduce the magnitude of parameters $\theta_j$
    - Regularization works well when we have a lot of slightly useful features.

- Cost function

  如果hypothesis function的参数过多，我们需要消去多余的参数$\theta$。不需要真的删除，只需要修改cost function让多余的参数$\theta$得到inflate膨胀。这样我们在用gradient descent 来求$\theta$使cost function最小化时，会自动的让那些参数趋近于0

  - 比如：我们的 h(z)的z是：$\theta_0+\theta_1x+\theta_2x^2+\theta_3x^3+\theta_4x^4$，然后想要让他的3次，4次参数失效。

    ​			只需修改cost function为: $min_\theta\frac{1}{2m}\sum_{i=1}^{m}(h_\theta(x^{(i)})-y^{(i)})^2+1000\theta_3^2+1000\theta_4^2$

  - 一个总结性的公式：

  $$
  min_\theta\frac{1}{2m}\sum_{i=1}^{m}(h_\theta(x^{(i)})-y^{(i)})^2+\lambda\sum_{j=1}^{n}\theta_j^2
  $$

  - 上式的$\lambda$叫做regularization parameter。
    - 如果太小没效果
    - 如果太大，会使得hypothesis function太过smooth,从而导致underfitting。

- Regularized Linear Regression

  - Gradient Descent:

    我们不希望惩罚penalize $\theta_0$所以单独列出来。

    下面的$\frac{\lambda}{m}\theta_j$代表了我们的正则化

    $$
    \begin{aligned}
      Repeat\{\\
      &\theta_0:= \theta_0-\alpha\frac{1}{m}\sum_{i=1}^{m}(h_\theta(x^{(i)})-y^{(i)})x_0^{(i)}\\
      &\theta_j:= \theta_j-\alpha[(\frac{1}{m}\sum_{i=1}^{m}(h_\theta(x^{(i)})-y^{(i)})x_j^{(i)})+\frac{\lambda}{m}\theta_j]\quad j\in\{1,2..n\}\\ 
      \}
      \end{aligned}
    $$

  - Normal Equation:
    
    $\lambda L$项代表了我们的正则化
    $$
    \theta=(X^TX+\lambda L)^{-1}X^Ty\\
    其中L= \begin{bmatrix}
       0 &  &  &\\
        & 1 &  &\\
        &  & \ddots &\\
        &  & &1
      \end{bmatrix}
    $$
    

## 2. Classification

- To attempt classification, one method is to use linear regression and map all predictions greater than 0.5 as a 1 and all less than 0.5 as a 0. However, this method doesn't work well because classification is not actually a linear function.

### (1)Logistic Regression逻辑回归

- Hypothesis Representation

	如上所说我们可以用线性规划的方法去尝试做分类问题，但这并不是一个好方法。因为当我们知道$y\in \{0,1\}$时，$h_\theta(x)$的值大于1或者小于0时是没有意义的。所以我们可以用另一种形式的假设：
	
	- Logistic Function ,也叫做Sigmoid Function
	  $$
	  \begin{cases}
	  g(z)=\frac{1}{1+e^{-z}}\\
	  z=\theta^Tx
	  \end{cases}\quad\Rightarrow\quad
	  h_\theta(x)=g(\theta^Tx) =\frac{1}{1+e^{-\theta^Tx}}
	  $$
	
	- $h_\theta(x)$给了输出y=1的概率
	  $$
	  h_\theta(x)=P(y=1|x;\theta)=1-P(y=0|x;\theta)
	  $$
	  比如$h_\theta(x)$=0.7就是当前输入x有70%的概率得到输出y=1。
	  
	- 求出某个类别的概率h(x)后，如果它大于等于0.5则分类为1，如果小于0.5则分类为0。
	
- Decision Boundary决策边界

  - The **decision boundary** is the line that separates the area where y = 0 and where y = 1. It is created by our hypothesis function的$\theta^Tx$

  - 由上可知h大于等于0.5则分类为1。此时z应该>0。

  - 由上可知h小于0.5则分类为0。此时z应该<0。
  
  - 决策边界可以不是线性的
  
    即我们的sigmoid function g(z)不必是线性的。比如它可以是个圆$z=\theta_0+\theta_1x_1^2+\theta_2x_2^2$
    
  - 决策边界这条线和参数theta向量是垂直的。
  
- Cost function

  我们不能使用和Linear regression一样的cost function，因为这会导致我们的cost function是non convex的，即函数的图像不是先单降后单升的凸型而是有很多局部最小的波浪形wavy。

  Logistic Regression为了得到convex function需要用下面这样的cost function:
  $$
  \begin{aligned}
  &J(\theta)=\frac{1}{m}\sum_{i=1}^mCost(h_\theta(x^{(i)}),y^{(i)})\\
  &Cost(h_\theta(x),y)=-log(h_\theta(x))\quad \quad &if\ y=1\\
  &Cost(h_\theta(x),y)=-log(1-h_\theta(x))\quad \quad &if\ y=0
  \end{aligned}
  $$
  自己画一下$h_\theta \in[0,1]$的Cost图像可发现：

  1. 如果正确解y=1，那么当$h_\theta$也为1时cost function将为0。当$h_\theta$接近0时cost function将趋于无穷大。
  2. 如果正确解y=0，那么当$h_\theta$也为0时cost function将为0。当$h_\theta$接近1时cost function将趋于无穷大。

  由此使得我们的cost function是convex function了(一个u型的函数)

  - 上面的方程可以合并为
    $$
    \begin{aligned}
    &Cost(h_\theta(x),y)=-ylog(h_\theta(x))-(1-y)log(1-h_\theta(x))\\
    &J(\theta)=\frac{1}{m}\sum_{i=1}^m[y^{(i)}log(h_\theta(x^{(i)}))+(1-y^{(i)})log(1-h_\theta(x^{i}))]
    \end{aligned}
    $$
    继续将其向量化：
    $$
    \begin{aligned}
    &h=g(X\theta)\\
    &J(\theta)=\frac{1}{m}(-y^Tlog(h)-(1-y)^Tlog(1-h))
    \end{aligned}
    $$

- Gradient Descent

  找到一组$\theta$使得cost function最小化
  $$
  \begin{aligned}
  Repeat&\{\\
  &\theta_j:=\theta_j-\alpha\frac{\partial}{\partial\theta_j}J(\theta)\\
  &\theta_j:=\theta_j-\frac{\alpha}{m}\sum_{i=1}^m(h_\theta(x^{(i)})-y^{(i)})x_j^{(i)}\\
  \}
  \end{aligned}
  $$
  向量化：
  $$
  \theta:=\theta-\frac{\alpha}{m}X^T(g(X\theta)-\vec{y})
  $$
  
- Advanced Optimization

  除了Gradient Descent还有"Conjugate gradient", "BFGS", and "L-BFGS" 也可以用来找$\theta$。但他们更加的sophisticated复杂也更加的快速。

  有很多现成的库可以实现这些算法。

### (2)Multiclass classification

- One vs all

  - 上面的logistic regression的分类只有2个y={1,2}。现在有更多的种类y={0,1,...n}。

  - 所以我们就依次的把某一类和其他类作为两个类 y={1,rest}, 来不停的求1的logistic regression.最后predict新的数据集x时，选择能最大化Hypothesis那一组类。预测的种类y即它。
    $$
    y\in\{0,1...,n\}\\
    h_\theta^{(0)}(x)=P(y=o|x;\theta)\\
    h_\theta^{(1)}(x)=P(y=1|x;\theta)\\
    ...\\
    h_\theta^{(n)}(x)=P(y=n|x;\theta)\\
    prediction=max(h_\theta^{(i)}(x))
    $$
    

### (3)Overfitting Problem

- Cost function

  在我们的Regularized Logistic原有cost function后加上$\frac{\lambda}{2m}\sum_{j=1}^n\theta_j^2$。代表忽视某一个参数
  $$
  J(\theta)=\frac{1}{m}\sum_{i=1}^m[y^{(i)}log(h_\theta(x^{(i)}))+(1-y^{(i)})log(1-h_\theta(x^{i}))]+\frac{\lambda}{2m}\sum_{j=1}^n\theta_j^2
  $$

- Gradient descent
  $$
  \begin{aligned}
  Repeat&\{\\
  &\theta_0:=\theta_0-\frac{\alpha}{m}\sum_{i=1}^m(h_\theta(x^{(i)})-y^{(i)})x_0^{(i)}\\
  &\theta_j:=\theta_j-\alpha[\frac{1}{m}\sum_{i=1}^m(h_\theta(x^{(i)})-y^{(i)})x_j^{(i)}+\frac{\lambda}{m}\theta_j]\\
  \}
  \end{aligned}
  $$
  

## 3. Neural Networks

### (1) Model Representation

- 神经网络模型 类似于 动物的神经，有

  - 输入(dendrites树突)：对应于我们方程的input features特征x1....xn
  - 输出(axonss轴突)：对应于我们的输出hypothesis function$h(\theta)$
  - x0：被叫做bias unit,且永远等于1
  - 在classification中同样使用logistic function$\frac{1}{1+e^{-\theta Tx}}$。
    - 但他在神经网络中叫做sigmoid(logistic) **activation** function
    - 而$\theta$也叫做weights.

- layer:

  - input layer: input nodes(layer1)
  - output layer: 最后输出hypothesis function的层
  - hidden layers: 在input 和 output layer之间的所有的层
  
- $a_i^{(j)}$：activation of unit单元 i in layer j
  
- $\theta^{(j)}$：matrix of weights controlling function mapping from layer *j* to layer *j*+1
  
- 如果我们有one hidden layer:
  $$
  \begin{aligned}\\
  &[x_0x_1x_2x_3]\rightarrow[a_1^{(2)}a_2^{(2)}a_3^{(2)}]\rightarrow h_\theta(x)\\
  \\
  layer 1:&[x_0x_1x_2x_3]\\
  \\
  layer2:&a_1^{(2)}=g(\theta_{10}^{(1)}x_0+\theta_{11}^{(1)}x_1+\theta_{12}^{(1)}x_2+\theta_{13}^{(1)}x_3)\\
  &a_2^{(2)}=g(\theta_{20}^{(1)}x_0+\theta_{21}^{(1)}x_1+\theta_{22}^{(1)}x_2+\theta_{23}^{(1)}x_3)\\
  &a_3^{(2)}=g(\theta_{30}^{(1)}x_0+\theta_{31}^{(1)}x_1+\theta_{32}^{(1)}x_2+\theta_{33}^{(1)}x_3)\\
  \\
  layer3:&h_\theta(x)=a_1^{(3)}=g(\theta_{10}^{(2)}a_0^{(2)}+\theta_{11}^{(2)}a_1^{(2)}+\theta_{12}^{(2)}a_2^{(2)}+\theta_{13}^{(2)}a_3^{(2)})
  \end{aligned}
  $$
  
  - 可看到我们用了一个3X4矩阵的参数来计算我们的activation node 'a'。
  - 每一层都有自己的weights$\theta^{(j)}$
  
- 向量化：
  $$
  \begin{aligned}
  1.计算当前j层&的hypothesis的参数z\\
  &For \ layer\ j\ and \ node \ k设置我们的变量z为：
  z_k^{(j)}=\theta_{k0}^{(j-1)}x_0+\theta_{k1}^{(j-1)}x_1+\theta_{k2}^{(j-1)}x_2+\dots+\theta_{kn}^{(j-1)}x_n\\
  &并设置:x=a^{(j-1)}\\
  &则Z可写成：z^{(j)}=\theta^{(j-1)}a^{(j-1)}\\
  2.计算当前j层&的activation\ node\ 'a'\\
  &a^{(j)}=g(z^{(j)})\\
  &比如上面的layer2也可写成：a_1^{(2)}=g(z_1^{(2)})...\\
  3.如果j+1层&是最后一层了\\
  &h_\theta(x)=a^{(j+1)}=g(z^{(j+1)})\\
  \end{aligned}\\
  $$
  
- 权重维度的确认

  - **如果i层有$s_i$个units，i+1层有$s_{i+1}$个units。那么$\theta^{(j)}$的维度是$s_{j+1}$x$(s_j+1)$**

    - 比如上面的例子，初始层有3个unit:x1,x2,x3,中间层有3个unit:a1,a2,a3，所以中间层的weights有3X4个
    - 输出层只有1个unit:$h_\theta(x)$所以输出层的weights有1X4个

  - 这里的+1是因为bias nodes: $x_0$和$\theta_0^{(j)}$。即ouput nodes不包含bias nodes但输入包含。

### (2)Multiclass Classification

- 在处理将数据分类到多个类class 里面去的这类问题：只需要让我们的hypothesis function 返回一组数组值
  $$
  \left[
  \begin{matrix}
  x_0\\
  x_1\\
  x_2\\
  \dots\\
  x_n
  \end{matrix}
  \right]\rightarrow
  \left[
  \begin{matrix}
  a_0^{(2)}\\
  a_1^{(2)}\\
  a_2^{(2)}\\
  \dots\\
  \end{matrix}
  \right]\rightarrow
  \left[
  \begin{matrix}
  a_0^{(3)}\\
  a_1^{(3)}\\
  a_2^{(3)}\\
  \dots\\
  \end{matrix}
  \right]\rightarrow
  \dots\rightarrow
  \left[
  \begin{matrix}
  h_\theta(x)_1\\
  h_\theta(x)_2\\
  h_\theta(x)_3\\
  h_\theta(x)_4\\
  \end{matrix}
  \right]
  $$
  然后比如如果得到[1000]代表是车，如果得到[0100]就是火车。

### (3)Train Neural Networks

- 一些变量
  - $L$ : total number of layers in the network
  - $S_l$ :number of units (not counting bias unit) in layer l
  - $K$: number of output units/classes即标签
  - $h_\theta(x)_k$: a hypothesis that results in the $k^{th}$ output

- Cost Function
  $$
  J(\theta)=-\frac{1}{m}\sum_{i=1}^{m}\sum_{k=1}^{K}[y_k^{(i)}log(h_\theta(x^{(i)})_k)+(1-y_k^{(i)})log(1-h_\theta(x^{(i)})_k)]+\frac{\lambda}{2m}\sum_{l=1}^{L-1}\sum_{i=1}^{s_l}\sum_{j=1}^{s_{l+1}}(\theta_{j,i}^{(l)})^2
  $$

  - the double sum simply adds up the logistic regression costs calculated for each cell in the output layer

  - the triple sum simply adds up the squares of all the individual Θs in the entire network.

  - the i in the triple sum does **not** refer to training example i

  - 作为对比，可以看用regularized logistic regression的cost Function 2(3):
    $$
    J(\theta)=\frac{1}{m}\sum_{i=1}^m[y^{(i)}log(h_\theta(x^{(i)}))+(1-y^{(i)})log(1-h_\theta(x^{i}))]+\frac{\lambda}{2m}\sum_{j=1}^n\theta_j^2
    $$
    可以发现对于neural networks 来说方程更加的复杂

#### Backpropagation Algorithm

- "Backpropagation反向传播" is neural-network terminology术语 for minimizing our cost function $J(\theta)$

  - 是一种与最优化方法(比如梯度下降法)结合使用来训练人工神经网络的方法。
  - 该方法对网络中所有权重计算损失函数的梯度。这个梯度会回馈给最佳化方法，用来更新权值以最小化损失函数。
  - 反向传播要求有对每个输入值想得到的已知输出，来计算损失函数梯度。因此是监督学习

- 由两个阶段组成

  1. 激励传播：

     每次迭代中的传播环节包含两步：

     - （前向传播阶段）将训练输入送入网络以获得激励响应；
     - （反向传播阶段）将激励响应同训练输入对应的目标输出求差，从而获得输出层和隐藏层的响应误差。

  2. 权重更新

     对于每个突触上的权重，按照以下步骤进行更新：

     - 将输入激励和响应误差相乘，从而获得权重的梯度；
     - 将这个梯度乘上一个**比例**并取反后加到权重上

     注意：1.这个比例（百分比）将会影响到训练过程的速度和效果，因此成为“训练因子”

     2. 梯度的方向指明了误差扩大的方向，因此在更新权重的时候需要对其取反，从而减小权重引起的误差

- 具体算法：
  $$
  \begin{aligned}
  &Training\ set:\{(x^{(1)},y^{(1)}),....,(x^{(m)},y^{(m)})  \}\\
  &Set\ \triangle_{ij}^{(l)} = 0 \ (for\ all\ l,i,j)\\
  &For\ i=1\ to\ m:\\
  &\qquad Set\ a^{(1)}=x^{(i)}\\
  &\qquad Perform\ forward\ propagation\ to \ compute\ a^{(l)}\ for\ l=2,3,...,L\\
  &\qquad Using\ y^{(i)},compute\ \delta^{(L)}=a^{(L)}-y^{(i)}\\
  &\qquad Compute\ \delta^{(L-1)},\delta^{(L-2)},...,\delta^{(2)}using\ \delta^{(l)}=((\theta^{(l)})^T\delta^{(l+1)})*a^{(l)}*(1-a^{(l)})    \\
  &\qquad \triangle_{ij}^{(l)}:=\triangle_{ij}^{(l)}+a_j^{(l)}\delta_i^{(l+1)}\\
  &D_{ij}^{(l)}:=\frac{1}{m}\triangle_{ij}^{{(l)}}+\lambda\theta_{ij}^{(l)}\quad if\ j\neq 0\\
  &D_{ij}^{(l)}:=\frac{1}{m}\triangle_{ij}^{{(l)}}\qquad \qquad \ if\ j=0\\
  &\frac{\partial}{\partial\theta_{ij}^{(l)}}J(\theta)=D_{ij}^{(l)}
  \end{aligned}
  $$

  - $a^{(l)}$是每一层所有的node节点。共L层
  - $\delta_j^{(l)}$是节点 j 在 l 层的error错误。在计算出最后一层的误差$\delta^{(L)}$后，往回递推各层的误差
  - $\triangle_{ij}^{(l)}$是误差，当作是accumulator蓄电池一样用来加误差，最后用来计算偏导D
  - $D_{ij}^{(l)}$是我们要求的Cost function J 的偏导，
  - 下标 i 是当前节点所连接下一层的各个节点的索引。
  - 下标 j 是当前节点的索引。j = 0即+1项。

#### Backpropagation in Practice

- First, pick a network architecture; choose the layout of your neural network, including how many hidden units in each layer and how many layers in total you want to have.
  - Number of input units = dimension of features $x^{(i)}$
  - Number of output units = number of classes
  - Number of hidden units per layer = usually more the better (must balance with cost of computation as it increases with more hidden units)
  - Defaults: 1 hidden layer. If you have more than 1 hidden layer, then it is recommended that you have the same number of units in every hidden layer.
  
- **Training a Neural Network**
  1. Randomly initialize the weights
  
     Initialize each $\theta_{ij}^{(l)}$ to a random value in $[-\epsilon,\epsilon]$
  
  2. implement forward propagation to get $h_\theta(x^{(i)})$for any $x^{(i)}$(循环)
  
  3. Implement the cost function
  
  4. Implement backpropagation to compute partial derivatives(循环)
  
  5. Use gradient checking to confirm that your backpropagation works. Then disable gradient checking.
     $$
     \frac{\partial}{\partial \theta_j}J(\theta) \approx \frac{J(\theta_1,...,\theta_j+\epsilon,...,\theta_n)-J(\theta_1,...,\theta_j-\epsilon,...,\theta_n)}{2\epsilon}
     $$
  
     - 梯度检验作用是确保反向传播的正确实施
  
     - 不要在训练中使用梯度检验，它只用于调试。因为梯度检验的过程会很慢。
  
  6. Use gradient descent or a built-in optimization function to minimize the cost function with the weights in theta.
  
- Ideally, you want $h_\Theta(x^{(i)}) \approx y^{(i)}$. This will minimize our cost function. However, keep in mind that $J(\Theta)$ is not convex and thus we can end up in a local minimum instead. 

## 4.Evaluating a Learning Algorithm

- 一些Trouble Shooting排错 for erros in our predictions:

  - Getting more training examples

  - Trying smaller sets of features

  - Trying additional features

  - Trying polynomial features

  - Increasing or decreasing λ

- 当做完上述操作后，可以进行evaluate our new hypothesis

  为了验证模型我们需要把数据集分为：

  1. traing set(70%):  Learn $\theta$ and minimize $J_{train}(\theta)$
  2. test set(30%):      Compute the test set error$J_{test}(\theta)$

### 各种检测误差，选择模型和参数方法：

下面的 参数degree，正则lambda，数据集size的选择都可以画一个learning curves图来直观的显示哪个值更好

- Test set error

  - For linear regression: $J_{test}(\theta)=\frac{1}{2m_{test}}\sum_{i=1}^{m_{test}}(h_\theta(x_{test}^{(i)})-y_{test}^{(i)})^2$

  - For classification: Misclassification error:
    $$
    \begin{aligned}
    & Test\ Error= \frac{1}{m_{test}}\sum_{i=1}^{m_{test}}err(h_{\theta}(x_{test}^{(i)}),y_{test}^{(i)})\\
    & err(h_\theta(x),y)=
    \begin{cases}
    &1 &\text{if}\ h_\theta(x) \geq0.5\ and\ y=0\ or\ h_\theta(x)<0.5,and\ y=1 \\
    &0 & otherwise
    \end{cases}
    \end{aligned}
    $$

- Model Selection and Train/Validation/Test Sets

  某一个数据集很准确也不能说明我们的hypothesis是对的，因为他可能在其他数据集上有很大的误差。

  那如何确定我们的model应该用怎么样的polynomial degree(比如:$h_\theta(x)=\theta_0+\theta_1x$)那他的degree d就是1

  - 首先划分数据集

    1. Training set: 60%
    2. Cross validation set: 20%
    3. Test set: 20%

  - 用如下步骤对三个数据集，计算三个separate error：

    1. Optimize the parameters in $\theta$ using the training set for each polynomial degree.
    2. Find the polynomial degree d with the least error using the cross validation set.
    3. Estimate the generalization error using the test set with $J_{test}(\theta^{(d)})$. (d = theta from polynomial with lower error)

  - Bias vs Variance
  
    用上面数据集计算各自的误差 J 后可按下分类：
  
    - High Bias(underfitting): Both $J_{train}(\theta)$ and $J_{CV}(\theta)$will be high.
    - High variance(overfitting): $J_{train}(\theta)$ will be low and $J_{CV}(\theta)$ will be much greater than $J_{train}(\theta)$ 
    - 所以要选个2个之间差距最小的那个模型。
  
- Regularization and Bias/Variance

  正则化参数$\lambda$的选择也很重要，太大会导致各个参数/权重$\theta$不起作用导致underfit，太小又会导致某些参数/权重影响过大导致overfit。

  如下步骤选择正则化参数：

  1. Create a list of lambdas (i.e. λ∈{0,0.01,0.02,0.04,0.08,0.16,0.32,0.64,1.28,2.56,5.12,10.24});
  2. Create a set of models with different degrees or any other variants
  3. Iterate through the$\lambda$s and for each $\lambda$ go through all the models to learn some $\Theta$.
  4. Compute the cross validation error using the learned $\theta$(computed with $\lambda$) on the $J_{cv}(\theta)$without regularization or $\lambda = 0$
  5. Select the best combo组合物 that produces the lowest error on the cross validation set.
  6. Using the best combo $\theta$ and $\lambda$, apply it on $J_{test}(\theta)$to see if it has a good generalization of the problem.

- Training set size

  数据集的大小也会对检测误差产生影响

  - **Experiencing high bias:**

    - **Low training set size**: causes $J_{train}(\theta)$to be low and  $J_{CV}(\theta)$to be high.
    - **Large training set size**: causes both$J_{train}(\theta)$and $J_{CV}(\theta)$ to be high.

    结合前面的Bias vs Variance可知：If a learning algorithm is suffering from **high bias**, getting more training data will **not** **(by itself)** help much.

  - **Experiencing high variance:**

    - **Low training set size**: causes $J_{train}(\theta)$to be low and  $J_{CV}(\theta)$to be high.
    - **Large training set size**: $J_{train}(\theta)$ 会增加and $J_{CV}(\theta)$ 会减小，但他们之间的差值始终remains significant相当数量

    所以：If a learning algorithm is suffering from **high variance**, getting more training data is likely to help。

  - **Model Complexity Effects:**

    - Lower-order polynomials (low model complexity) have high bias and low variance. In this case, the model fits poorly consistently.
    - Higher-order polynomials (high model complexity) fit the training data extremely well and the test data extremely poorly. These have low bias on the training data, but very high variance.
    - In reality, we would want to choose a model somewhere in between, that can generalize well but also fits the data reasonably well.

### 总结：如何做我们的模型修改决策

- Our decision process can be broken down as follows:

  - **Getting more training examples:** Fixes high variance

  - **Trying smaller sets of features:** Fixes high variance

  - **Adding features:** Fixes high bias

  - **Adding polynomial features:** Fixes high bias

  - **Decreasing λ:** Fixes high bias

  - **Increasing λ:** Fixes high variance.

- 对于神经网络

  - A neural network with fewer parameters is **prone to underfitting**. It is also **computationally cheaper**.
  - A large neural network with more parameters is **prone to overfitting**. It is also **computationally expensive**. In this case you can use regularization (increase λ) to address the overfitting.

  Using a single hidden layer is a good starting default. You can train your neural network on a number of hidden layers using your cross validation set. You can then select the one that performs best. 

## 5.如何更好的设计一个机器学习系统

- 比如Building a spam垃圾邮件 classifier

   how could you spend your time to improve the accuracy of this classifier:

  - Collect lots of data (for example "honeypot" project but doesn't always work)
  - Develop sophisticated复杂的 features (for example: using email header data in spam emails)
  - Develop algorithms to process your input in different ways (recognizing misspellings in spam).

- 如何解决机器学习的问题？

  - Start with a simple algorithm, implement it quickly, and test it early on your cross validation data.
  - Plot learning curves to decide if more data, more features, etc. are likely to help.
  - Manually examine the errors on examples in the cross validation set and try to spot a trend where most of the errors were made.

- 并量化我们的错误率。来查看不同措施下，我们算法的优越性。

   | Predicted class   \   Actual class | 1                | 0                |
   | ---------------------------------- | ---------------- | ---------------- |
   | 1                                  | True & Positive  | False & Positive |
   | 0                                  | False & Negative | True & Negative  |

   - $精准度precision=\frac{True\ Positive}{True\ Positive+False\ Positive}$
      - Precision从预测结果角度出发，描述了二分类器预测出来的正例结果中有多少是真实正例，即该二分类器预测的正例有多少是准确的

   - $召回率Recall = \frac{True\ Positive}{True\ Positive+False\ Negative}$
      - Recall从真实结果角度出发，描述了测试集中的真实正例有多少被二分类器挑选了出来，即真实的正例有多少被该二分类器召回。

   - $准确性Accuracy=\frac{true positives + true negatives}{total examples}$

      On **skewed datasets**(e.g., when there are more positive examples than negative examples), accuracy is not a good measure of performance and you should instead use F1 score

   - $F_1\ Score=\frac{2 * precision * recall}{precision + recall}$

    A good classifier should have both a high precision and high recall on the cross validation

- 当满足以下2个条件时，Training on a lot of data is likely to give good performance 

   - 假设特征X有足够的信息用来predict y accurately。

      也可以说：Given the input x, can a human expert condifently predict y

   -  We train a learning algorithm with a large number of parameters

     也可以说：Our learning algorithm is able to represent fairly complex functions
     
     因为数据集一大，模型太简单不可能完美预测
     
   
   Using a **very large** training set makes it unlikely for model to overfit the training data.





## 6.Support Vector Machine

SVM也被称作large Margin Classifier

### (1)Large Margin边缘 Classification

- Optimization Objective优化目标

  - 相比于Logistic regression逻辑回归的优化目标
    $$
    最小化 J(\theta)=\frac{1}{m}\sum_{i=1}^m[y^{(i)}log(h_\theta(x^{(i)}))+(1-y^{(i)})log(1-h_\theta(x^{i}))]+\frac{\lambda}{2m}\sum_{j=1}^n\theta_j^2
    $$

  - SVM的优化目标是
    $$
    最小化\ C\sum_{i=1}^m[y^{(i)}cost_1(\theta^Tx^{(i)})+(1-y^{(i)})cost_0(\theta^Tx^{(i)})]+\frac{1}{2}\sum_{i=1}^n\theta_j^2
    $$

    - 相比于logistic regression，样本数m没必要乘，C=1/lambda

      - C大相当于$\lambda$小，导致Lower bias,high variance
      - C小相当于$\lambda$大，导致Higher bias,low variance
      - 所以如果underfit了就要增加C，反之overfit了就要减小C。
    
    - Hypothesis:后面的条件是根据，如果y=1是我们的优化目标
      $$
      h_\theta(x)=
      \begin{cases}
      &1 \quad if\ \theta^Tx \geq0\\
      &0 \quad otherwise
      \end{cases}
      $$
    
    
    - 参数C 控制着对误分类的训练样本的惩罚，当我们将参数 C 设置的较大时，优化过程会努力使所有训练数据被正确分类，这会导致仅仅因为一个异常点决策边界就会改变，这是不明智的。
    - SVM可以通过将参数 C 设置得不太大而忽略掉一些异常的影响

### (2)Kernels核

- SVM利用核函数可以构造出复杂的非线性分类器，如下图

  - ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Kernel.png?raw=true)

  - 这里定义Hypothesis为
    $$
    h_\theta(x)=
    \begin{cases}
    &1, \quad \theta_0+\theta_1x_1+\theta_2x_2+\theta_3x_1x_2+\theta_4x_1^2+...\geq0\\
    &0, \quad \theta_0+\theta_1x_1+\theta_2x_2+\theta_3x_1x_2+\theta_4x_1^2+...<0\\
    \end{cases}
    $$

    - 定义特征变量features$f_1=x^1,f_2=x^2,f_3=x_1x_2,f_4=x_1^2,...$

- 高斯核函数

  给定 x，我们可以提前手动选取一些标记点，然后根据与这些标记点的接近程度来计算新的特征项。比如有3个标记点l(1),l(2),l(3)。

  核函数有不同种类，其中一种是高斯核函数：
  $$
  f_1=similarity(x,l^{(1)})=exp(-\frac{||x-l^{(1)}||^2}{2\sigma^2})\\
  f_2=similarity(x,l^{(2)})=exp(-\frac{||x-l^{(2)}||^2}{2\sigma^2})\\
  f_3=similarity(x,l^{(3)})=exp(-\frac{||x-l^{(3)}||^2}{2\sigma^2})
  $$

  - 从上可知当x接近l, f趋于1。当x远离l，f趋于0.
  - 所以f的值在0到1之间，体现了新的特征X与标记点l的接近程度。
  - $\sigma^2$大，features f vary more smoothly. 导致higher bias,lower variance
  - $\sigma^2$小，features f vary less smoothly. 导致lower bias,higher variance
  - 所以如果underfit了就要减小$\sigma$,反之overfit了就要增加

- 如何选择核函数的标记点？

  如果有m个训练样本$(x^{(1)},y^{(1)}),..(x^{(m)},y^{(m)})$。可选择$l^{(1)}=x^{(1)},..,l^{(m)}=x^{(m)}$，即与样本重合的位置为标记点。

- SVM结合核函数，最终我们的优化目标也变为
  $$
  最小化\ C\sum_{i=1}^m[y^{(i)}cost_1(\theta^Tf^{(i)})+(1-y^{(i)})cost_0(\theta^Tf^{(i)})]+\frac{1}{2}\sum_{i=1}^m\theta_j^2
  $$

  - 与上面SVM的优化目标相比,x变成了f, n变成了m。
  - 最后一项编程时用$\theta^TM\theta$而非$\theta^T\theta$来计算，会提高计算效率。其中M是一个矩阵取决于核数

- 其他off the shelf现成的的核函数：

  - Polynomial Kernel多项式核函数
  - String Kernel字符串核函数
  - Chi-square Kernel卡方核函数
  - Histogram intersection kernel直方图交叉核

- Multi-class classificaton

  许多现成的SVM软件包自带了多类分类函数。

  当然也可以用one vs all的方法：Train K SVMs, one to distinguish y=i from the rest.然后对任意一特征X选择$(\theta^{(i)})^Tx$最大的那组参数。

- SVM与Logistic regression的选择

  n: number of features(x). m: number of training examples

  - if n is large(relative to m): Use logistic regression or SVM without a kernel("linear kernel")
  - If n is small, m is intermediate中等的: Use SVM with Gaussian kernel
  - if n is small,m is large: Create/add more features,then use logistic regression or SVM without a kernel

  对上面三种情况神经网络都适用，但可能要花费很多时间去学习。


# 二. Unsupervised Learning无监督学习

- 两个例子

  1. Clustering:

     Take a collection of 1,000,000 different genes, and find a way to automatically group these genes into groups that are somehow similar or related by different variables, such as lifespan, location, roles, and so on.

  2. Non-Clustering:

     The "Cocktail Party Algorithm", allows you to find structure in a chaotic environment. (i.e. identifying individual voices and music

## 1.Cluster

- k-means clustering algorithm

  是一种迭代求解的聚类分析算法，其步骤是，预将数据分为K组，则随机选取K个对象作为初始的聚类中心，然后计算每个对象与各个种子聚类中心之间的距离，把每个对象分配给距离它最近的聚类中心。

  - Input:

    1. K(number of cluster)
    2. Training set{x1,x2,...,xm}

  - algorithm
    $$
    \begin{aligned}
    Repeat&\{\\
    for&\ i =1\ to\ m\\
    &c^{(i)}:=index\ (from\ 1 \ to \ K)of\ cluster\ centroid\ closest\ to\ x^{(i)}\\
    for&\ k=1\ to\ K\\
    &\mu_k:=\text{average (mean) of points assigned to cluster k}\\
    
    \}
    \end{aligned}
    $$

    - 第一个循环用来cluster assignment，保持$\mu$不变来最小化成本函数
    - 第二循环用来move cluster centroid$\mu_k$，保持$C^{(i)}$不变来最小化成本函数

- K-means optimization objective
  $$
  min\ J(c^{(1)},...,c^{(m)},\mu_1,...\mu_k)=\frac{1}{m}\sum_{i=1}^{m}||x^{(i)}-\mu_{c^{(i)}}||^2
  $$

  - $c^{(i)}$: index of cluster(1,2..,K) to which example $x^{(i)}$ is currently assigned
  - $\mu_k$: cluster centroid k ($\mu_k\in R^n$)
  - $\mu_{c^{(i)}}$: cluster centroid of cluster to which example $x^{(i)}$ has been assigned.
    - 比如$x^{(i)}$被聚为5，那么$c^{(i)}=5$， $\mu_{c^{(i)}}=\mu_5$

- 如何选择K？
  - Use the elbow method
  - 一个个试，找到 J 最小的那个
