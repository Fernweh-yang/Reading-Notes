# 一、 各种库的安装使用

## 0) 用GPU跑jupyter notebook

https://thegeeksdiary.com/2021/10/07/how-to-setup-tensorflow-with-gpu-support-in-windows-11/

## 1）python os库

https://www.runoob.com/python/os-file-methods.html

## 2） python numpy库

https://numpy.org/doc/stable/

### 2.1array

- 固定的一维矩阵`a=np.array([2,3,4])`
- 固定的二维矩阵`a=np.array([2,3,4],[1,2,3])`
- 随机的mxn维矩阵`a=np.random.rand(4,3)`



# 二、一些特别的Python机制

## 1. Iterator迭代器和Generator生成器

- 为什么需要迭代器和生成器？

  - 受到内存限制，列表容量肯定是有限的。而且，创建一个包含100万个元素的列表，不仅占用很大的存储空间，如果我们仅仅需要访问前面几个元素，那后面绝大多数元素占用的空间都白白浪费了。
  - 如果列表元素可以按照某种算法推算出来，就可以在循环的过程中不断推算出后续的元素。这样就不必创建完整的list，从而节省大量的空间。
  - 迭代器和生成器就是Python中这种一遍循环一边计算的机制
  - Generator是一种Iterator，

- 类中实现2种方法：`__iter__()`和`__next__()`的就是iterator迭代器

  ```python
  class MyNumbers:
    # __iter__() 方法返回一个特殊的迭代器对象， 这个迭代器对象实现了 __next__() 方法并通过 StopIteration 异常标识迭代的完成。
    def __iter__(self):
      self.a = 1
      return self
    # __next__() 方法会返回下一个迭代器对象
    def __next__(self):
      if self.a <= 20:
        x = self.a
        self.a += 1
        return x
      else:
        raise StopIteration	# StopIteration 异常用于标识迭代的完成，防止出现无限循环的情况
  ```

- 生成器本质上还是一个迭代器，但它的实现更加的简洁

​	有2种创建Generator的方式:

  1. 用表达式生成：

     用**iter()**或把一个列表生成式的**[]改成()**，就创建了一个generator：

     ```python
     >>> L = [x * x for x in range(10)]			# List 2种写法
     >>> L = list(x * x for x in range(10))
     >>> L
     [0, 1, 4, 9, 16, 25, 36, 49, 64, 81]
     >>> g = iter(L)								# 用iter将list转为生成器
     >>> g = (x * x for x in range(10))			# g 是一个生成器
     >>> g
     <generator object <genexpr> at 0x104feab40>
     
     # generator 不能直接打印要用g.next()或循环来输出
     for i in g:
         print(i)
     ```

  2. 用带yield语句函数生成：

     如果一个函数定义中包含yield关键字，那么这个函数就不再是一个普通函数，而是一个generator：

      ```python
      # 普通函数：
      def fib(max):
          n, a, b = 0, 0, 1
          while n < max:
              print (b)					# print语句
              a, b = b, a + b
              n = n + 1
     
      # 生成器：
      def fib(max):
          n, a, b = 0, 0, 1
          while n < max:
              yield b						# yield语句
              a, b = b, a + b
              n = n + 1
      ```
     
     - 在调用生成器运行的过程中，每次遇到 yield 时函数会暂停并保存当前所有的运行信息，返回 yield 的值, 并在下一次执行 next() 方法时从当前位置继续运行。



# 三、一些重要概念

## 0. 超参数Hyperparameters

- 参数和超参数：

  - 参数：就是模型可以根据数据可以自动学习出的变量就是参数。
    - 比如，深度学习的权重，偏差等

  - 超参数：用来确定模型的一些参数
    * Network architecture
        * Choice of activation function
        * Number of layers
        * ...
    * Learning rate
    * Number of epochs
        * epoch数是一个超参数，它定义了学习算法在整个训练数据集中的工作次数。一个Epoch意味着训练数据集中的每个样本都有机会更新内部模型参数。
    * Batch size
    * Regularization strength
    * Momentum
    * ...

调参调的就是Hyperparameters

- 调参的方法：

  - Manual search:

    - experience-based

  - Grid search (structured, for ‘real’ applications)

    **代码见4.5.1**
  
    - Define ranges for all parameters spaces and select points
    - Usually pseudo-uniformly distributed

    Iterate over all possible configurations

    - 比如2个超参数要调，row横轴是第一个参数，column竖轴是第二个参数。这样每一个点代表一个可能性，迭代每一个点就是迭代了所有的可能性
  
  - Random search:
    
    **代码见4.5.2**
    
    - Like grid search but one picks points at random in the predefined ranges
    - To get a deeper understanding of random search and why it is more efficient than grid search, you should definitely check out this [paper](http://www.jmlr.org/papers/volume13/bergstra12a/bergstra12a.pdf):
    
  - Auto-ML search
    - Bayesian framework; gradient descent on gradient descent, typically complex
  
  

## 1. 损失函数Loss Functions

**代码实现**参见2.4

- Loss Function: 

  Used to measure the goodness of the predictions(the network's performance)

### 1.1 Classification loss:

  - **Cross-Entropy loss** 交叉熵损失函数(Maximum Likelihood Estimate)

    也叫做Softmax Loss，用于多分类中表示预测和真实分布之间的距离有多远。

    It normalizes the logits/scores to sum up to 1 / a probability distribution。

    **公式：**

    $$ CE(\hat{y}, y) = \frac{1}{N} \sum_{i=1}^N \sum_{k=1}^{C} \Big[ -y_{ik} \log(\hat{y}_{ik}) \Big]=-\frac{1}{N}\sum_{i=1}^Nlog(\frac{e^{S_{yi}}}{\Sigma_{k=1}^Ce^{S_k}}) $$

    $L_i=-log(\frac{e^{S_{}yi}}{\Sigma_{k=1}^Ce^{S_k}}) $

    where:
    - $ N $ is again the number of samples

    - $ C $ is the number of classes有多少个分类对象
      - 当C=2，即binary cross-entropy(BCE)
    - $ \hat{y}_{ik} $ is the probability that the model assigns for the $k$'th class when the $i$'th sample is the input. 
    - $y_{ik} = 1 $ iff the true label of the $i$th sample is $k$ and 0 otherwise. This is called a [one-hot encoding](https://machinelearningmastery.com/why-one-hot-encode-data-in-machine-learning/).
    - $\frac{e^{S_{}yi}}{\Sigma_{k=1}^Ce^{S_k}} $:即softmax function,表示每一个类的概率
      - 分子是Exponential Operation：make sure probability > 0
      - 分母是Normalization正则化：make sure probabilities sum up to 1.
      - $s=f(x_i,\theta)$:Score function
    - 为什么叫Softmax Loss:因为Output Layer 是Softmax function

  -  **Binary Cross-Entropy** (BCE).用于二分类

    $$BCE(y,\hat{y}) =- y \cdot log(\hat y ) - (1- y) \cdot log(1-\hat y) $$

    对于cnn:$BCE=\sum_i(- y_i \cdot log(\hat y_i ) - (1- y_i) \cdot log(1-\hat y_i))$
    
    - $y\in\mathbb{R}$ is the ground truth
    -  $\hat y\in\mathbb{R}$ is the predicted probability of the house being expensive.
    -  i指的是cnn feature map各个元素，比如2x2的输出，i就是1->4
    -  相比于多分类用的Softmax，BCE用的Output Layer是Sigmoid function
    
  - **Hinge Loss**:Multiclass SVM Loss

    $L=\frac{1}{N}\sum_{i=1}^NL_i$

    $L_i=\sum_{k\neq y_i}max(0,s_k-s_{y_i}+1)$

    - $s_k$：是不同class的分数
    - $s_{y_i}$：是真实class的分数

    理解：

    - 相比Cross Entropy loss永远不可能等于0，Hinge Loss会saturate(=0),2个不同的模型的loss值都等于0时，就无法分辨哪个更好

    -  实现了软间隔分类（这个Loss函数都可以做到）

    - **保持了支持向量机解的稀疏性**

      - **换用其他的Loss函数的话，SVM就不再是SVM了。**

        **正是因为HingeLoss的零区域对应的正是非支持向量的普通样本，从而所有的普通样本都不参与最终超平面的决定，这才是支持向量机最大的优势所在，对训练样本数目的依赖大大减少，而且提高了训练效率。**

### 1.2 Regression loss

  - **L1 loss**: $L(y,\hat{y};\theta)=\frac{1}{n}\sum_i^n||y_i-\hat y_i||_1$
    - Robust(Cost of outliers is linear)不容易受离异值影响
    - Costly to optimize
    - Optimum is the median
    - 没有导数为0的点，smooth l1 loss解决了其在0点处导数不唯一
  - **L2/MSE loss**: $L(y,\hat{y};\theta)=\frac{1}{n}\sum_i^n||y_i-\hat{y}_i||_2^2$
    - Prone to outliers容易受离异值影响
    - Computer efficient optimization
    - Optimum is the mean
    - 0点处导数为0

### 1.3 Image reconstruction loss

- L1,L2,ssim,psnr

### 1.4 Loss + Regularization正则化

![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/Full%20Loss.png?raw=true)

- Full Loss = Data Loss + Reg Loss
  - Data Loss: 即选择用cross Entropy还是Hinge function来计算$L_i$,然后所有样本的loss值相加取平均

## 2. 激活函数Activation Functions

激活函数（Activation Function）类似于人类大脑中基于神经元的模型，激活函数最终决定了要发射给下一个神经元的内容。

- 激活函数为什么是非线性的
  - 如果使用线性激活函数，那么输入跟输出之间的关系为线性的，无论神经网络有多少层都是线性组合。
  - 激活函数给神经元引入了非线性因素，使得神经网络可以任意逼近任何非线性函数，这样神经网络就可以应用到众多的非线性模型中。
  - 输出层可能会使用线性激活函数，但在**隐含层都使用非线性激活函数**
- 激活函数的作用：
  - Introduce non-linearity into network. Without activation function is network only linear
  - prevent layer collapse


### 2.1Sigmoid

![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/Sigmoid%20function.png?raw=true)

$\sigma(x)=\frac{1}{(1+e^{-x})}$

- Sigmoid is one of the oldest used non-linearities. 

  - 公式：$\sigma(x)=\frac{1}{1+exp(-x)}$


  - Sigmoid的导数：$\frac{\partial\sigma(x)}{\partial x}=\sigma(x)(1-\sigma(x))$


  - 反向求导：
    $$
    \frac{\partial L}{\partial x}=\frac{\partial L}{\partial \sigma(x)}\cdot\frac{\partial\sigma(x)}{\partial x}
    $$

    - $\frac{\partial L}{\partial \sigma(x)}$对应下面3.2.2代码backward中的`dout`
    
    - $\frac{\partial\sigma(x)}{\partial x}$对应下面3.2.2代码backward中的`sd`
    
  - 优点：
    - The output is between 0 and 1. So sigmoid can be used for output layer of classification-aimed neural networks

  - 缺点：
    - It requires **computation of an exponent**, which need more compute resouce and makes the convergence of the network slower.
    
    - **vanishing gradient**梯度消失：the neuro is saturated, because when it reaches its maximum or minimum value, the derivative will be equal to 0（也称为saturate饱和）.$\sigma(x)(1-\sigma(x))$趋于0，因为sigmoid最小最大值为0和1。then the weights will not be updated
    
      - 可以用ReLU激活函数来代替：
    
        因为ReLU doesn't saturate and have large consistent gradients
    
      - 也可以add residual connections
    
        因为有highway for gradient flow
    
      - 还可以用Xavier initialization
    
        因为it improved weight initialization targets active area of the activation function
    
    - **Not zero-centered function**:During update process, these weights are only allowed to move in one direction, i.e. positive or negative, at a time. This makes the loss function optimization harder.
### 2.2tanh

![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/Tanh%20function.png?raw=true)

- 公式：$tanh(x)=\frac{sinhx}{coshx}=\frac{e^x-e^{-x}}{e^x+e^{-x}}$
  -  tanh的导数：$\frac{\partial tanhx}{\partial x}=1-tan(x)^2$
- 优点：
  - The output is between -1 and 1. Unlike Sigmoid, It is a **zero-centered function** so that the optimization of the loss function becomes easier.
- 缺点：
  - Has the same problem as Sigmoid: **vanishing gradient** and **computation of an exponent**
- 代码实现见3.2.2

### 2.3ReLU

![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/ReLu%20function.png?raw=true)

$max(0,x)$

Rectified Linear Units线性整流单元 are the currently **most used** non-linearities in deep learning

- 公式:$ReLU(x) = max(0,x)=\begin{cases}x(x>0)\\0(x\leq0) \end{cases}$

  - ReLU的导数：$\frac{\partial ReLU(x)}{\partial x}=\begin{cases}1(x>0)\\0(x\leq0) \end{cases}$

- 反向求导:
  $$
  \frac{\partial L}{\partial x}=\frac{\partial L}{\partial ReLU(x)}\cdot\frac{\partial ReLU(x)}{\partial x}
  $$

  - $\frac{\partial L}{\partial ReLU(x)}$对应下面3.2.2代码backward中的dout
  - $\frac{\partial ReLU(x)}{\partial x}$如果>0值为1就是传dout本身，所以只需要让<=0的dout值为0即可。
  
- 优点：

  - It is easy to compute so that the neural network converges very quickly.
  - For positive values of the neuron: No vanishing gradient

- 缺点：

  - It is not a zero-centered function.

  - **Dead ReLU Problem(神经元坏死现象)**：For negative values, the gradient will be 0 forever and the weights weill not be updated anymore

    解决：

    1.  Use Xavier to initialize
    2. set a small learning rate

### 2.4Leaky ReLU

![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/Leaky%20ReLu%20function.png?raw=true)

 $当\alpha =0.01,即max(0.01x,x)$时，叫做leaky relu. 如果$\alpha$等于其他值，就叫randomized relu.

- 优点：
  - It is easy to compute.
  - It is close to zero-centered function.
  - Solved **Dead ReLU Problem(神经元坏死现象)**
- 代码实现见3.2.2

## 3. 优化算法Optimization 

**具体代码**实现见3.5

### 3.0 不同优化算法之间的关系

[参考1](https://zhuanlan.zhihu.com/p/81048198)

[参考2](https://ruder.io/optimizing-gradient-descent/)（老师推荐）

每次梯度下降都遍历整个数据集会耗费大量计算能力，而mini-batch梯度下降法通过从数据集抽取小批量的数据进行小批度梯度下降解决了这一问题。使用mini-batch会产生下降过程中左右振荡的现象。而动量梯度下降法通过减小振荡对算法进行优化。动量梯度下降法的核心便是对一系列梯度进行指数加权平均。

- Linear Systems(AX=b)
  - LU, QR, Cholesky, Jacobi, Gauss-Seidel, CG, PCG, etc.
- Non-linear (gradient-based)
  - [Newton, Gauss-Newton, LM, (L)BFGS](https://zhuanlan.zhihu.com/p/29672873)         <---Second order
  - Gradient Descent, SGD                                  <---first order
- Others
  - Genetic algorithms, MCMC, Metropolis-Hastings, etc.
  - Constrained and convex solvers (Langrage, ADMM,
    Primal-Dual, etc.)

### 3.1Gradient Descent

- Single Training Sample
  1. Given a loss function $L$ and a single training sample $\{x_i,y_i\}$
  2. Find best model parameters$\theta=\{W,b\}$
  3. Cost$L_i(\theta,x_i,y_i)$
     - $\theta=arg minL_i(x_i,y_i)$
  4. Gradient Descent
     1. Initialize$\theta^1$with random values
     2. Update step:$\theta^{k+1}=\theta^{k}-\alpha\nabla_\theta L_i(\theta^k,x_i,y_i)$
        - $\nabla_\theta L_i(\theta^k,x_i,y_i)$ computed via backpropagation
     3. Iterate until convergence:$|\theta^{k+1}-\theta^k|<\epsilon$

- Multiple Training Samples
  1. Given a loss function $L$ and multiple(n) training samples $\{x_i,y_i\}$
  2. Find best model parameters$\theta=\{W,b\}$
  3. Cost$L=\frac{1}{n}\sum_{i=1}^n L_i(\theta,x_i,y_i)$
     - $\theta=arg min L$
  4. Gradient Descent
     1. Initialize$\theta^1$with random values
     2. Update step:$\theta^{k+1}=\theta^{k}-\alpha\nabla_\theta L(\theta^k,x_{\{1..n\}},y_{\{1..n\}})$
        - Gradient is sum over residuals$\nabla_\theta L(\theta^k,x_{\{1..n\}},y_{\{1..n\}})=\frac{1}{n}\sum_{i=1}^n \nabla_\theta L_i(\theta,x_i,y_i)$
        - $\nabla_\theta L_i(\theta^k,x_i,y_i)$ computed via backpropagation
        - omitting省略$\frac{1}{n}$is not wrong: 可以看作resacling缩放 the learning rate

### 3.2Stochastic Gradient Descent(SGD)

随机梯度下降法-》小批量梯度下降法

如果有n=1million个training samples$\{x_i,y_i\}$,有50万个参数$\theta$，计算变得非常expensive无法一次性计算所有的samples。

根据empirical risk minimization经验风险最小化，可以用一小撮数据mini batch来近似计算。事实发现，实际训练中使用mini-batch梯度下降法可以大大加快训练速度，并节省计算机资源

实现方法：将样本总体分成多个mini-batch。例如100万的数据，分成10000份,每份包含100个数据的mini-batch-1到mini-batch-10000，每次梯度下降使用其中一个mini-batch进行训练，除此之外和梯度下降法没有任何区别
$$
L=\frac{1}{n}\sum_{i=1}^n L_i(\theta,x_i,y_i)\approx\frac{1}{|S|}\sum_{j\in S} L_j(\theta,x_j,y_j)
$$

- 与GD的对比

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/%E6%A2%AF%E5%BA%A6%E4%B8%8B%E9%99%8D.jpg?raw=true)

  - 可以看到由于mini-batch每次仅使用数据集中的一部分进行梯度下降，所以每次下降并不是严格按照朝最小方向下降，但是总体下降趋势是朝着最小方向
  - 当然实际上GD也是曲折的，SGD会加剧曲折

- Minibatch: choose subset of transet $m<<n$
  $$
  \begin{aligned}
  B_i &= \{ \{x_1,y_1\},\{x_2,y_2\},..,\{x_m,y_m\}\} \\
    &= \{B_1,B_2,...B_{\frac{n}{m}}\}
  \end{aligned}
  $$

  - m: Minibatch size选择选择为 power of 2 即8，16，32.....
  - 越小的minibatch size ->  greater variance in the gradients -> noisy updates
    - 但noisy update 也可以帮助escape from saddle point
  - It is limited by GPU memory(in backward pass)
  - There exists some batch size, for which the gradient of the loss w.r.t model parameters in SGD is equal to the gradient in GD
  
- Stochastic Gradient Descent

  - $\theta^{k+1}=\theta^{k}-\alpha\nabla_\theta L(\theta^k,x_{\{1..m\}},y_{\{1..m\}})$
    - k现在refers to k-th **iteration**,$k\in(1.\frac{n}{m})$
    - 当所有的$\frac{n}{m}$个minibatch都更新完，就称为完成了一个**epoch**
    - **Epoch**:full run through the entire train set
  - $\nabla_\theta L=\frac{1}{m}\sum_{i=1}^m\nabla_\theta L_i$
    - m training samples in the current minibatch
    - $\nabla_\theta$: Gradient for the k-th minibatch

- Problems of SGD

  - Gradient is scaled equally均等缩放 across all dimensions
    - i.e., cannot independently scale directions
    - need to have conservative保守的 min learning rate to avoid divergence发散
    - Slower than ‘necessary’
  - Finding good learning rate is an art by itself
  
    Because SGD only has one learning rate for all dimensons
  
  - SGD is noisy

### 3.3Gradient Descent with Momentum

[动量梯度下降法](https://zhuanlan.zhihu.com/p/34240246)， 主要是针对mini-batch梯度下降法进行优化。

原理：给梯度下降加一个动量，让它在伪最优解的地方能够像现实生活中的小球那样冲出去。

- 好处：
  - 可以很轻松的跳出下面这三种伪最优解
    - plateau(稳定的水平)：下降的很慢，微分约等于0
    - saddle point(鞍点)：微分时沿着某一方向是稳定的(=0)，另一条方向是不稳定的奇点
      - The gradient is zero , but it is neither a local minima nor a local maxima
    - local minima(局部最小点)；
  - 让寻找最优解的曲线能够不那么振荡、波动，让他能够更加的平滑，在水平方向的速度更快。（如下图）
  
- 与SGD的对比

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/sgd_momentum.png?raw=true)

  - 可以看到动量优化之后左右的摆动减小，从而提高了效率。
  - speeds up learning if gradient keeps pointing in the same direction.->解决了sgd的slow learning
  - keep direction of gradient to get out of local minimum.->解决了sgd can't escape from local minima
  - adjusts learning rate if  oscillating振荡 over local minimum. -> 解决了sgd only has one learning rate for all dimension
  - exponentially weighted moving average reduces noise.-> 解决了sgd is noisy

-  Gradient Descent with Momentum

  - $v^{k+1}=\beta \cdot v^k-\alpha\cdot\nabla_\theta L(\theta^k)$
    - $v$: velocity。这是一个向量
    - $\beta$: accmulation rate积累速率of velocity/friction，和$\alpha$一样都是Hyperparameters
  - $\theta^{k+1}=\theta^k+v^{k+1}$
  
- 使用Nesterov Momentum:

  Nesterov是Momentum的变种。与Momentum唯一区别就是，计算梯度的不同。Nesterov动量中，**先用当前的速度 v 临时更新一遍参数，在用更新的临时参数计算梯度**。

  - Use the previous momentum to update the parameter, then use the parameter to calculate the new gradient.

### 3.4Root Mean Squared Prop(RMSProp)

均方根传递

Momentum虽然初步减小了摆动幅度但是实际应用中摆动幅度仍然较大和收敛速度较慢。RMSProp在动量法的基础上进一步优化该问题。具体做法是更新权重时，使用除根号的方法，使得较大幅度大幅度变小，较小幅度小幅度变小；然后通过设置较大learning rate，使得学习步子变大。

RMSProp is an adaptive learning rate method自适应学习率方法。It scale the learning rate based on element wise squared gradient.

- 直观展示：

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/RMSProp.png?raw=true)

  - Division in Y-Direction will be large
  - Division in X-Direction will be small

- RMSProp

  - $s^{k+1}=\beta\cdot s^k + (1-\beta)[\nabla_\theta L\bigodot \nabla_\theta L]$

    - $\nabla_\theta L\bigodot \nabla_\theta L$: 表示element wise multiplication

  - $\theta^{k+1}=\theta^k-\alpha\cdot\frac{\nabla_\theta L}{\sqrt{s^{k+1}+\epsilon}}$

    - $\epsilon,\alpha,\beta$都是Hyperparameters

- 特点：
  
  - Dampening抑制 the oscillations振荡 for high-variance高方差 directions
  - Can use faster learning rate because it is less likely to diverge
    - Speed up learning speed
    - Second moment二阶矩估计 RMSProp does not have momentum
  

### 3.5Adaptive Moment Estimation (Adam)

自适应矩估计，本质上是带有动量项momentum的RMSprop，即 adam = Momentum + Adaptive learning rate.它利用梯度的一阶矩估计和二阶矩估计动态调整每个参数的学习率。Adam的优点主要在于经过偏置校正bias correction后，每一次迭代学习率都有个确定范围，使得参数比较平稳。

- Adam
  - 原理即结合Momentum和RMSProp
    - Momentum:   $m^{k+1}=\beta_1\cdot m^k+(1-\beta_1)\nabla_\theta L(\theta^k)$
    - RMSProp:        $v^{k+1}=\beta_2\cdot v^k + (1-\beta_2)[\nabla_\theta L\bigodot \nabla_\theta L]$
    - $\beta_1$和$\beta_2$是exponential decay rate for the moment estimates
      - $\beta_1$ is related to bias correction for first moment estimate(decays the running average of the gradient)
      - $\beta_2$ is related to bias correction for second moment estimate(decays running average of square of gradient)
  - 初始化$m^0=0,v^0=0$
  - 跟新两个值（偏置校正）
    - Momentum:$\hat{m}^{k+1}=\frac{m^{k+1}}{1-\beta_1^{k+1}}$
    - RMSProp:$\hat{v}^{k+1}=\frac{v^{k+1}}{1-\beta_2^{k+1}}$
  - 更新参数
    - $\theta^{k+1}=\theta^k-\alpha\cdot\frac{\hat{m}^{k+1}}{\sqrt{\hat{v}^{k+1}}+\epsilon}$
- 偏置校正Bias Correction
  - When accumulating积累 gradients in a weighted average fashion以加权平均的方式, the first gradient is initialized to zero. This biases all the accumulated gradients down towards zero使所有累积的梯度向下偏向0.
  - The Bias correction normalizes the magnitude of the accumulated gradient for early steps

- 特点：
  - Exponentially-decaying指数衰减 mean and variance of gradients (combines first and second order momentum)
  - 优点：
    - 实现简单，计算高效，对内存需求少
    - 参数的更新不受梯度的伸缩变换影响
    - 超参数具有很好的解释性，且通常无需调整或仅需很少的微调
    - 更新的步长能够被限制在大致的范围内（初始学习率）
    - 能自然地实现步长退火过程（自动调整学习率）
    - 很适合应用于大规模的数据及参数的场景
    - 适用于不稳定目标函数
    - 适用于梯度稀疏或梯度存在很大噪声的问题
  - 在实际应用中 ，Adam为最常用的方法

### 3.6梯度下降之外的优化算法：

1. Newton’s Method牛顿法
   - BFGS and L-BFGS，Gauss-Newton，Levenberg，Levenberg-Marquardt
   
   - 牛顿法doesn’t work well for minibatches，适用于全数据集处理
   
   - Mathematical View：牛顿法有着Faster convergence in fewer iterations
     - 但是：Approximating近似 the inverse Hessian is highly computationally costly, not
       feasible可行的 for high-dimensional datasets.
     
   - Key idea:
   
     Approximate by second order taylor expansion
   
     $\theta_*=\theta_0-H^{-1}\nabla_\theta L(\theta)$
   
2. Conjugate Gradien共轭梯度法

3. coordinate descent坐标下降法

### 3.7什么是一阶矩和二阶矩

- 中心距：对于正整数k，如果E(X)存在，且$E\{[X-E(X)]^k\}$<∞，则称$E\{[X-E(X)]^k\}$为随机变量X的k阶中心矩

- 一阶矩就是期望值，换句话说就是平均数。
  - 一阶矩只有一阶非中心矩，因为一阶中心矩永远等于零。
- 二阶(非中心)矩就是对变量的平方求期望
  - 二阶中心矩就是对随机变量与均值(期望)的差的平方求期望
  - 为什么要用平方，因为如果序列中有负数就会产生较大波动，而平方运算就好像对序列添加了绝对值，这样更能体现偏离均值的范围

## 4. Learning Rate

- 不同learning rate的implication可能后果

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/learning%20rate.png?raw=true)

  - need high learning rate when far away aim

    need low learning rate when close

  - 如果network's training curve diverge发散(no problem in data loading)

    - 这时需要reduce the learning rate
    
  - **Too high learning rate**: cost function does not converge to an optimal solution and can even diverge.

  - **Too low learning rate**： cost function may not converge to an optimal solution, or will converge after a very long time.

- Learning Rate Decay衰败

  不同的Decay方法

  - $\alpha=\frac{1}{1+decay\_rate*epoch}\cdot\alpha_0$

  - step decay:$\alpha=\alpha-t\cdot\alpha$
    - only every n steps
    - t is decay rate(often 0.5)
  - Exponential decay:$\alpha=t^{epoch}\cdot\alpha_0$
    - t is decay rate<1.0
  - $\alpha =\frac{t}{\sqrt{epoch}}\cdot\alpha_0$

## 5. Performance measure

[参考](https://zhuanlan.zhihu.com/p/78204581)

- 标准：

  | Predicted class   \   Actual class | 1                | 0                |
  | ---------------------------------- | ---------------- | ---------------- |
  | 1                                  | True & Positive  | False & Positive |
  | 0                                  | False & Negative | True & Negative  |

  - True/False指预测对了没有，positive/negative指预测的结果

  - **精准度**$precision=\frac{True\ Positive}{True\ Positive+False\ Positive}$

    - Precision从预测结果角度出发，描述了二分类器预测出来的正例结果中有多少是真实正例，即该二分类器预测的正例有多少是准确的

  - **召回率**$Recall = \frac{True\ Positive}{True\ Positive+False\ Negative}$

    - Recall从真实结果角度出发，描述了测试集中的真实正例有多少被二分类器挑选了出来，即真实的正例有多少被该二分类器召回。

  - **准确性**$Accuracy=\frac{True\ Positives + True\ Negatives}{Total\ Examples}$

    On **skewed使不公允 datasets**(e.g., when there are more positive examples than negative examples), accuracy is not a good measure of performance and you should instead use F1 score

    - Why is it a problem to use the classification accuracy as a loss to train a neural network?

      Because Accuracy is discrete function->not differentiable

  - $F_1\ Score=\frac{2 * Precision * Recall}{Precision + Recall}$

  - Precision和Recall通常是一对矛盾的性能度量指标。一般来说，Precision越高时，Recall往往越低

- Bias和variance

  - 一般情况下，模型需要在bias和variance之间取得一个平衡。bias小的模型，variance一般大；variance小的模型，bias一般大。

  - **Bias偏差**：refers to the model being too simple -> underfitting

  - **Variance方差**：refers to too many attention on trainins data and missing generalization on unseen data -> overfitting


## 6. 参数的初始化

- 初始化时权重Weights不能设置为如下三种
  1. 都是同一个值 ：
     - The hidden units are all going to compute the same function, gradients are going to be the same
     - All neurons will learn the same things
  2. 都是小随机数：Output become to zero，梯度消失vanishing gradient
  3. 都是大随机数：Output saturated to -1 and 1，对于tanh,sigmoid同样会梯度消失
- 为此使用 Xavier or Kaiming initialization.
  - They aims to maintain same variance of output as input


### 6.1 Xavier Initialization

- Xavier初始化的作者，Xavier Glorot，在[论文](https://links.jianshu.com/go?to=http%3A%2F%2Fproceedings.mlr.press%2Fv9%2Fglorot10a%2Fglorot10a.pdf)中提出一个洞见：激活值的方差是逐层递减的，这导致反向传播中的梯度也逐层递减。要解决梯度消失，就要避免激活值方差的衰减，最理想的情况是，每层的输出值（激活值）保持高斯分布。

- 数学背景

  - 期望Expectation：是度量一个随机变量取值的集中位置或平均水平的最基本的数字特征

  - 方差Variance：是表示随机变量取值的分散性的一个数字特征

    -  方差越大，说明随机变量的取值分布越不均匀，变化性越强
    - 方差越小，说明随机变量的取值越趋近于均值，即期望值。

  - 关系有：

    - $E[X^2]=Var[X]+E[X]^2$

    如果X,Y是独立的，那么：

    - $Var[XY]=E[X^2Y^2]-E[XY]^2$
    - $E[XY]=E[X]E[Y]$

- Xavier Initialization
  
  With Xavier initialization we initialize the weights to be Gaussian with zero mean and variance Var(w)=1/n where n is the amount of neurons in the input.
  
  As a result, the output will have zero mean and  similar variance as the input.
  $$
  \begin{align}
  Var(s)&=Var(\sum_i^nw_ix_i)=\sum_i^nVar(w_ix_i)\\
  &=\sum_i^n[E(w_i)]^2Var(x_i)+E[(x_i)]^2Var(w_i)+Var(x_i)Var(w_i)\\
  &=\sum_i^nVar(x_i)Var(w_i)\\
  &=n(Var(w)Var(x))
  \end{align}
  $$

  - $n$：the number of input neurons for the layer of weights you want to initialized

    注意n不是input Data$X\in R^{N\times D}$的N，对于第一层而言n=D

  - 上面等式第二行$E(w_i)和E(x_i)$因为用到Gaussian with zero mean所以都是0
  
  - 为了确保variance方差 of the output = input，即为了$Var(s)=Var(x)$
  
    - $n(Var(w)Var(x))=Var(x)\Rightarrow Var(w)=\frac{1}{n}$


### 6.2 Kaiming initialization

- 相比Xavier:
  - Xavier初始化的问题在于，它只适用于线性激活函数，但实际上，对于深层神经网络来说，线性激活函数是没有价值，神经网络需要非线性激活函数来构建复杂的非线性系统。今天的神经网络普遍使用relu激活函数。
  - Kaiming初始化的发明人kaiming he，在[论文](https://arxiv.org/abs/1502.01852)中提出了针对relu的kaiming初始化。
  - 因为relu会抛弃掉小于0的值，对于一个均值为0的data来说，这就相当于砍掉了一半的值the output variance of a layer is halved，这样一来，均值就会变大，前面Xavier初始化公式中E(x)=mean=0的情况就不成立了。根据新公式的推导，最终得到新的rescale系数：$\sqrt\frac{2}{n}$
- $Var(w)=\frac{2}{n}$
  - $n$：the number of input neurons for the layer of weights you want to initialize

## 7. 数据集

- Training set('train')

  -  used for training neural network and get weights用于训练模型以及确定模型权重。

- Validation set('val')

  - Validation set is used for testing generalization with different Hyperparameters

  - Hyperparameter Optimization调整模型的超参数。
  - tune the model's architecture确定网络结构

- Test set('test')

  - Test set is only used **at the end** to test generalization on unseen data once.

- 如果training set都是白天during the day拍的图片，validation set都是晚上at night拍的

  - 这样的data from different distributions 会导致bad generaization/ high var error
  - 通过mix trainig +validation data, shuffle洗牌 and split again来使all sets from same distribution

- Typocal split分割

  - train:60    val:20    test:20
  - train:80    val:10    test:10
  - train:98   val:1    test:1
  - train:80(cross-validation)    test:20

- [Cross-Validation](https://zhuanlan.zhihu.com/p/24825503)

  1. LOOCV(Leave-one-out cross-validation)

     只用一个数据作为测试集，其他的数据都作为训练集，并将此步骤重复N次（N为数据集的数据数量）。

     结果就是我们最终训练了n个模型，每次都能得到一个MSE（Mean Squarded Error）。而计算最终test MSE则就是将这n个MSE取平均

  2. K-fold cross validation k折交叉验证

     和LOOCV的不同在于，我们每次的测试集将不再只包含一个数据，而是多个，具体数目将根据K的选取决定。

     比如K=5，就把数据集分成5份。不重复地每次取其中一份做测试集，用其他四份做训练集训练模型，之后计算该模型在测试集上的MSE
     
     - 如果K过大，会increase training time or more computations
     
       因为It has to train and validate K separate times

- 各个数据集误差处理

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/%E5%90%84%E6%95%B0%E6%8D%AE%E9%9B%86%E8%AF%AF%E5%B7%AE.png?raw=true)

  - Ideal Training： Small gap between training and validation loss, and both
    go down at same rate (stable without fluctuations波动).

## 8. 关于过拟合

- **过拟合的原因：**
  - 训练集数据太少，很容易就过拟合了。
  - 训练集测试集的数据分布不一致，这点很容易被忽略。比如有的算法要求数据集符合高斯分布，训练集也满足条件，但是上线以后线上数据分布发生了变化，效果肯定不会太好。
  - 模型本身特别复杂。比如树模型，如果树的棵数太多，深度太大，也很容易就过拟合。
  
- **解决方案**
  - 针对训练数据太少的问题，可以增加训练数据（Data Augmentation）
  - 增对模型复杂度太高的问题，可以降低模型复杂度。比如，减少层的数量或者减少神经元的个数，这样可以缩小网络的规模。
  - 正则化，这是解决过拟合的常用方法。
    - Regularization is a technique that aims to reduce the generatlizatioan泛化 gap.
  - dropout，神经网络在每一次迭代过程中随机地丢弃神经网络中的神经元。每当我们丢弃不同的神经元，相当于重新训练了一个新的神经网络。
  - early stop，训练过程中，如果训练误差继续减小，但是测试误差已经开始增加，此时可以停止训练。
  - 集成学习，将多个模型进行组合，可以降低少数模型过拟合风险。
  - BN，Batch Normalization。在CNN每层之间加上将神经元的权重调成标准正态分布的正则化层。
  
- Unsupervised learning也会遇到过拟合的问题

  比如：

  1. Clustering N datapoints with N cluster
  2. Autoencoder with large bottleneck

- 反过来如果遇到underfitting.需要：

  1. increase model capacity
  2. decrease learning rate
  3. BN

### 8.1 L2 regularization

- 公式：$\theta_{k+1}=\theta_k-\epsilon\nabla_\theta(\theta_k,x,y)-\epsilon\lambda\theta_k$

  - $\epsilon$: Learning rate

  - $\nabla_\theta$: Gradient

  - $\lambda\theta_k$: Gradient of L2-Regularization
    - L2 Regularization=$0.5\cdot\lambda\cdot||\theta||^2$
  - $1-\epsilon\lambda$: Learning rate of weight decay衰减.(上面公式合并一下)
    - weight decay的目的是防止过拟合，to spread the decision power amog as many neurons as possible.
    - $\epsilon \lambda$<<1,so the weight$\theta$ is pushed towards zero in each iteration
- 用于penalize惩罚 large weights
- 用于improve generalization泛化

### 8.2 Max Norm Regularization

Regularization technique that constrains weights of network (directly) 

Hyperparameter:$r\in\mathbb{R}>0$
$$
w=\begin{cases}
w \quad \quad if\ ||w||_2\leq r\\
r\frac{w}{||w||_2} \ otherwise
\end{cases}\\
||w||_2=\sqrt{w_1^2+w_2^2+\cdots+w_n^2}
$$

### 8.3 Ensemble Methods集成学习

  集成学习是一种训练思路，并不是某种具体的算法。它会挑选一些简单的基础模型进行组装，从而得到更好的效果。主要有2种：

  - **Bagging**(bootstrap aggregating自助聚集的简写)：核心思路是民主
    - Bagging 的思路是所有基础模型都一致对待，每个基础模型手里都只有一票。然后使用民主投票的方式得到最终的结果。
    - **经过 bagging 得到的结果方差（variance）更小**。
    - 具体过程：
      1. 从原始样本集中抽取训练集。每轮从原始样本集中使用Bootstraping的方法抽取n个训练样本（在训练集中，有些样本可能被多次抽取到，而有些样本可能一次都没有被抽中）。共进行k轮抽取，得到k个训练集。（k个训练集之间是相互独立的）
      2. 每次使用一个训练集得到一个模型，k个训练集共得到k个模型。（注：这里并没有具体的分类算法或回归方法，我们可以根据具体问题采用不同的分类或回归方法，如决策树、感知器等）
      3. 对分类问题：将上步得到的k个模型采用投票的方式得到分类结果；对回归问题，计算上述模型的均值作为最后的结果。（所有模型的重要性相同）
    - 在 bagging 的方法中，最广为熟知的就是随机森林了：bagging + 决策树 = 随机森林
  - **Boosting**助推：核心思路是精英
    - Boosting 和 bagging 最本质的差别在于他对基础模型不是一致对待的，而是经过不停的考验和筛选来挑选出「精英」，然后给精英更多的投票权，表现不好的基础模型则给较少的投票权，然后综合所有人的投票得到最终结果。
    - **经过 boosting 得到的结果偏差（bias）更小**
    - 具体过程：
      1. 通过加法模型将基础模型进行线性的组合。
      2. 每一轮训练都提升那些错误率小的基础模型权重，同时减小错误率高的模型权重。
      3. 在每一轮改变训练数据的权值或概率分布，通过提高那些在前一轮被弱分类器分错样例的权值，减小前一轮分对样例的权值，来使得分类器对误分的数据有较好的效果。
    - 在 boosting 的方法中，比较主流的有 [Adaboost](https://easyai.tech/ai-definition/adaboost/) 和 Gradient boosting 。

### 8.4 Dropout

- 在机器学习的模型中，如果模型的参数太多，而训练样本又太少，训练出来的模型很容易产生过拟合的现象。Dropout可以比较有效的缓解过拟合的发生，在一定程度上达到正则化的效果。

  每次选择丢弃的神经元是随机的．最简单的方法是设置一个固定的概率 p．对每一个神经元都以概率 p 随机丢弃（即置0）

- Dropout可以作为训练深度神经网络的一种trick供选择。在每个训练批次中，通过忽略一半的特征检测器（让一半的隐层节点值为0）Using half the network = half capacity，可以明显地减少过拟合现象。这种方式可以减少特征检测器（隐层节点）间的相互作用reducing co-adaptation between neurons，检测器相互作用是指某些检测器依赖其他检测器才能发挥作用。

- Dropout可以看作是一种集成学习Ensemble Methods

  因为对于采用了dropout的网络，一个mini-batch中的每个样本所走的网络结构是不同的，于是可以理解为我们在同时训练多个网络，所以可以视作集成学习。

- Dropout在训练和测试时的区别

  - Dropout 在训练时采用，是为了减少神经元对部分上层神经元的依赖，以提高模型的泛化能力，减少过拟合。

  - 在测试时，应该用整个训练好的模型，因此不需要dropout。
  
- 两层probabilities分别为p和q的dropout层，可以用1个dropout层代替：

  - If neurons zero out independently, so dropout layer with probability p+q-2pq
  - If dropout layer 1 has already zeroed out neurons, then those can't be considered by second layer anymore. Probability is pq.



  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/Dropout.png?raw=true)

- 特征
  - Usually does not work well when combined with batch-norm.
  - Efficient regularization method, can be used with L2
  - Training takes a bit longer, usually 1.5x
  - But, can be used for uncertainty estimation
- Monte Carlo dropout
  - Neural networks are massively overconfident
  - We can use dropout to make the softmax probabilities more calibrated校准的.
  - Training: use dropout with a low p (0.1 or 0.2).
  - Inference, run the same image multiple times (25- 100), and average the results.

### 8.5 Batch normalization

- Batch Normalization批标准化：

  - Batch normalization 也可以被看做一个层面.用在fully connected or convolutional layers和 activation function之间。

    先数据X -> 全连接层 -> Batch normalization -> 激励函数

  - Batch normalization 的 batch 是批数据, 把数据分成小批小批进行 stochastic gradient descent. 而且在每批数据进行前向传递 forward propagation 的时候, 对每一层都进行 normalization 的处理,

  - BN就是通过归一化手段，将每层输入强行拉回均值0方差为1的标准正态分布，这样使得激活输入值分布在非线性函数梯度敏感区域，从而避免梯度消失问题，大大加快训练速度。

    Mimics模仿 the normalization of data for each layer to receive more stable inputs

- BN 作用

  - 加快收敛速度，有效避免梯度消失。

  - 提升模型泛化能力，BN的缩放因子可以有效的识别对网络贡献不大的神经元，经过激活函数后可以自动削弱或消除一些神经元。另外，由于归一化，很少发生数据分布不同导致的参数变动过大问题。

- BN在训练和测试时的差别：

  - 在训练时，是对每一批的训练数据进行归一化，也即用每一批数据的均值和方差。
    - Calculate the mean and variance computed from the mini-batch

    - store a weighted average across training mini-batches for the update at test time

  - 测试时，比如进行一个样本的预测，就并没有batch的概念，因此，这个时候用的均值和方差是全量训练数据的均值和方差
    - Use exponentially weighted average指数加权平均 mean and variance that was computed at training time on the test samples

- 为什么需要Batch Normalization批标准化：

  1. We know that normalizing input features can speed up learning, one intuition is that doing same thing for hidden layers should also work.
  2. solve the problem of covariance shift样本点的变化
     - Covariance shift: 假设x是属于特征空间的某一样本点，y是标签。covariate这个词，其实就是指这里的x。Covariance shift即样本点x的变化。
     - Suppose you have trained your cat-recognizing network use black cat, but evaluate on colored cats, you will see data distribution changing(called covariance shift). 我们无法用只用黑猫训练的模型去预测五颜六色的猫，这时候就只能重新训练模型
     - For a neural network, suppose input distribution is constant恒定, so output distribution of a certain hidden layer某个隐藏层的输出分布 should have been constant. But as the weights of that layer and previous layers changing in the training phase, the output distribution will change, this cause covariance shift from the perspective of layer after it从后面层的角度来看. Just like cat-recognizing network, the following need to re-train. To recover this problem, we use batch normal to force a zero-mean and one-variance distribution. It allow layer after it to learn independently from previous layers, and more **concentrate专注于 on its own task**, and so as to speed up the training process.
  3. Batch normal as regularization(slightly)
     - In batch normal, mean and variance is computed on mini-batch, which consist not too much samples. So the mean and variance contains noise. Just like dropout, it adds some noise to hidden layer's activation(dropout randomly multiply activation by 0 or 1). **This is an extra and slight effect, don't rely on it as a regularizer.**
  4. 对于tanh激活函数，可以avoid vanishing gradient
     - 没有 normalize 的数据 使用 tanh 激活以后, 激活值大部分都分布到了饱和阶段, 也就是大部分的激活值不是-1, 就是1, 而 normalize 以后, 大部分的激活值在每个分布区间都还有存在. 再将这个激活后的分布传递到下一层神经网络进行后续计算, 每个区间都有分布的这一种对于神经网络就会更加有价值.

  

- 公式：

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/batch%20normalization.png?raw=true)

  - 用N个有D个特征的样本数据x,来计算平均值mean: $\mu$

  - 基于平均值$\mu$,计算方差variance: $\sigma$

  - 标准化的normalized input data: $\hat{x}$
    $$
    \hat{x}=\frac{x_{i,j}-\mu_j}{\sqrt{\sigma_j^2+\epsilon}}
    $$

    - $\hat{x}$也叫做unit gaussian
    - 计算得到的是一个mini batch的$\hat{x}$
    - 其中$\epsilon$是为了防止方差为0产生无效计算。
    
  - 最后得到输出: $y_{i,j}=\gamma_j\hat{x}_{i,j}+\beta_j$

    可以用下面这2个参数来undo the normaliztion operation of Barchnorm
    
    - $\gamma$：scale参数
    
      undo时，$\gamma=\sqrt{var(x_i)}$
    
    - $\beta$：shift参数
    
      undo时，$\beta_i=E(x_i)$

- Train vs Test

  - Mean 和 Variace的获得：
    - Train：
      - compute statics from mini-batch,
      - store the mean and variance
    - Test：
      - Use stored mean and variance and don't compute any batch statistics
      - 没有足够的测试数据集，所以 by running an exponentially weighted averaged across training minibatches.

### 8.6 Data Augmentation

- Data Pre-Processing

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/Data%20pre_processing.png?raw=true)

- Data Augmentation增长

  - A classifier has to be invariant不变 to a wide variety of transformations变化
  - Helping the classifier: synthesize合成 data simulating plausible似乎是真的 transformations
  - 手段有：
    - Flip翻转
    - crop裁剪
    - Random brightniess and contrast changes
    - shear剪切(使倾斜)

## 9. Transfer Learning迁移学习

- [迁移学习](https://www.zhihu.com/question/41979241)(Transfer learning) 
  - 就是把已学训练好的模型参数迁移到新的模型来帮助新模型训练。考虑到大部分数据或任务是存在相关性的，所以通过迁移学习我们可以将已经学到的模型参数（也可理解为模型学到的知识）通过某种方式来分享给新模型从而加快并优化模型的学习效率不用像大多数网络那样从零学习（starting from scratch，tabula rasa）。
- Pytorch关于Transfer Learning的[教程](https://pytorch.org/tutorials/beginner/transfer_learning_tutorial.html)
- 什么时候用Transfer learning
  - When task T1 and T2 have the same input
  - When you have more data for task T1 than for task T2
  - When the low-level features for T1 could be useful to learn T2
    - T1 extract low level features
    - T2 extract high level features


## 10. 类别不平衡的分类问题

也叫长尾分布学习，long-tailed recognition。

- 如果某一数据集的某一标签占比太大，模型会倾向于预测所有的数据都是它 the network will prefer it。

  比如有4000个猫图片，却只有100个狗图片

- 解决方法：

  - collect more dog images/data augmentation for dogs
  - Reweight重加权 loss function
    - 对不同类别的loss分配不同的权重
  - reweight dataloader 

## 11. Residual connections

残差连接，也叫skip connect.

- 基本思想：

  输出表述为输入和输入的一个非线性变换的线性叠加，没用新的公式，没有新的理论，只是换了一种新的表达。

- 结构如下：

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/residual%20connect.png?raw=true)

  - 我们可以使用一个非线性变化函数来描述一个网络的输入输出，即输入为X，输出为F(x)，F通常包括了卷积，激活等操作。

- 为什么要skip connect

  - 有一个通识：网络越深表达能力越强，性能越好。但随着网络深度的增加，带来了许多问题，梯度消散，梯度爆炸。为此更好的优化方法，更好的初始化策略，BN层，Relu等各种激活函数，都被用过了，但是仍然不够。

    但是如果使用了残差，**每一个导数就加上了一个恒等项1，dh/dx=d(f+x)/dx=1+df/dx**。此时就算原来的导数df/dx很小，这时候误差仍然能够有效的反向传播。

    highway of gradients-> improve the vanishing gradient problem

  - **神经网络的退化**也是难以训练深层网络的原因。
  
    - 网络退化Degradation problem：虽然是一个很高维的矩阵，但是大部分维度却没有信息，表达能力没有看起来那么强大。
    - **残差连接**强制打破了网络的对称性，使得网络又恢复了表达能力。
    - 应用残差链接的CNN结构：ResNet (Deep residual network)
  
- Skip connections为什么helpful?

  1. Help prevent vanishing gradients 
  2. Provides highway for the gradients in backward pass
## 12. 一个应用：Encoder编码器

代码实现见下面6.4.3

### 12.1 Autoencoder自动编码器

自编码器（autoencoder, AE）是一类在半监督学习和非监督学习中使用的人工神经网络ANN，其功能是通过将输入信息作为学习目标，对输入信息进行表征学习（representation learning）。

- 主要应用：

  - 数据去噪
  - 可视化降维
  - 生成数据

- 基本结构：(unsupervised learning)

  NN Encoder -> code -> NN Decoder

  - 第一部分：编码器

    输入的数据经过神经网络(Encoder)降维到一个编码(code)

  - 第二部分：解码器

    编码经过另一个神经网络(Decoder)解码得到一个与输入原数据几乎一模一样的数据
  
- 我们可以修改size of bottleneck(code)，但如果

  - the latent潜在的 space is too small:
    - loss of information
    - bad reconstruction quality
    - too much compression/extraction/blurry
    - underfitting.

  - the latent space is too big
    - too little compression/extraction
    - overfitting
    - poor generalization
    - learned identity mapping/memorizes the inputs


### 12.2 Variational Autoencoder变分自动编码器

- 变分编码器是自动编码器的升级版本，其结构跟自动编码器是类似的，也由编码器和解码器构成。但普通的自动编码器不能**任意的生成图片**，因为我们没法自己去构造隐藏向量（即上面的 code）。而变分自动编码器在编码过程给它增加一些限制，迫使其生成的隐含向量能够粗略的遵循一个标准正态分布，这就是其与一般的自动编码器最大的不同。这样我们生成一张新图片就很简单了，我们只需要给它一个标准正态分布的随机隐含向量，这样通过解码器就能够生成我们想要的图片，而不需要给它一张原始图片先编码。

  在实际情况中，我们需要在模型的准确率上与隐含向量服从标准正态分布之间做一个权衡，所谓模型的准确率就是指解码器生成的图片与原图片的相似程度。我们可以让网络自己来做这个决定，非常简单，我们只需要将这两者都做一个loss，然后在将他们求和作为总的loss，这样网络就能够自己选择如何才能够使得这个总的loss下降。有一个东西叫KL divergence来衡量两种分布的相似程度，这里我们就是用KL divergence来表示隐含向量与标准正态分布之间差异的loss，另外一个loss仍然使用生成图片与原图片的均方误差来表示。

- 普通的Autoencoder不能任意的生成图片是因为：

  encoder does not project input distribution surjectively满射的 into latent space->

  decoder will not project all latent space vectors to coin images.

- KL divergence公式
  $$
  DKL(P||Q)=\int_{-\infty}^{\infty}P(x)log\frac{P(x)}{q(x)}dx
  $$

- 与AutoEncoder区别

  - A variational Autoencoder imposes强加 (optional: Gaussian / KL-Divergence loss) constraints on the distribution of the bottleneck
  - Variational autoenoders will constrain the bottleneck distribution into a probability distribution but autoencoders don't constrain the latent space.

### 12.3 结合transfer learning

- 如何利用训练好的encoder参数来帮助训练其他网络

  - use pretrained encoder of autoencoder and discard丢弃 decoder
  - freeze weights of encoder

- 预训练的autoencoder变量比randomly initialized的网络更泛化。

  因为

  - the network can access to much more data
  - can already detect some features
  
- 如果已经在dataset上训练了一个Autoencoder,应该如何在不继续训练（without further training）的情况下将数据集分成10类？

  1. Use the trained encoder to get latent embedding for each unlabeled image
  2. DO clustering. Assign分配 each image to it's cluster centre.

## 13. MLP

Multi-Layer Perception多层感知器是最基本的神经网络。下面的CNN，RNN都是更高级一点的神经网络

- MLP包括三层：输入层、隐藏层和输出层
- MLP不同层之间是全连接的（上一层的任何一个神经元和下一层的所有神经元都有连接）

## 14. 微分求解方法

[参考](https://blog.csdn.net/aws3217150/article/details/70214422)

微分求解大致可以分为4种方式：

- 手动求解法(Manual Differentiation)
- 数值微分法(Numerical Differentiation)
- 符号微分法(Symbolic Differentiation)
- 自动微分法(Automatic Differentiation)

### 14.1 手动求解法

手动求解其实就对应我们传统的backprop算法，我们求解出梯度公式，然后编写代码，代入实际数值，得出真实的梯度。在这样的方式下，每一次我们修改算法模型，都要修改对应的梯度求解算法，因此没有很好的办法解脱用户手动编写梯度求解的代码。

### 14.2 数值微分法

数值微分法是根据导数的原始定义：
$$
f'(x)=\mathop{lim}_{h\rightarrow0}\frac{f(x+h)-f(x)}{h}
$$
优点：可以对用户隐藏求解过程，用户只要给出目标函数和要求解的梯度的变量，程序可以自动给出相应的梯度.

缺点：数值微分法计算量太大，求解速度是这四种方法中最慢的，且无法避免roundoff error舍入误差和truncatrion error截断误差。

但是由于它实在是太简单实现了，于是很多时候，我们利用它来检验其他算法的正确性，比如在实现backprop的时候，我们用的"gradient check"就是利用数值微分法。

### 14.3 符号微分法

利用代数工具去求解各项微分公式后，最后代入数值。

用户只要提供具有closed form的数学表达式，就能用这些代数公式自动求解。

closed form: 必须能写成完整数学表达式的，不能有编程语言中的循环结构，条件结构等

优点：某种程度上的自动微分

缺点：1.必须要有closed form数学表达式。 2. “表达式膨胀"（expression swell）问题，即数学表达式特别复杂，使得计算变慢

### 14.4 自动微分法

自动微分的存在依赖于它识破如下事实：**所有数值计算归根结底是一系列有限的可微算子的组合**

- 自动微分法是一种介于符号微分和数值微分的方法：
  - 数值微分强调一开始直接代入数值近似求解；
  - 符号微分强调直接对代数进行求解，最后才代入问题数值；
  - 自动微分将符号微分法应用于最基本的算子，比如常数，幂函数，指数函数，对数函数，三角函数等，然后代入数值，保留中间结果，最后再应用于整个函数。
- 应用相当灵活，可以做到完全向用户隐藏微分求解过程

## 15. 贝叶斯深度学习

### 15.1 贝叶斯公式：

描述了两个条件概率之间的关系
$$
p(z|x)=\frac{p(x,z)}{p(x)}=\frac{p(x|z)p(z)}{\int p(x|z)p(z)dz}=\frac{p(x|z)p(z)}{p(x)} \tag{1}
$$

- $p(z|x)$：后验概率posterior

  表示在观测到x之后事件z发生的概率

- $p(x|z)$：似然likelihood

  表示在事件z为真的情况下观测到x的概率

- $p(z)$：先验概率prior

  表示在考虑任何新观测之前事件z发生的概率

- $p(x)$：边际概率marginal probability

  表示观测到x的总概率

- $p(x,z)$：联合概率

### 15.2 与普通深度学习的区别

- 区别：
  - 在深度学习中权重w和偏置b都是一个确定的值，即使我们通过梯度下降(gradient decent)来更新权重，也没有改变他们是一个确定的值这一事实。
  - 在贝叶斯深度学习中，w和b都由确定的值变成**分布**，这就是贝叶斯深度学习。

- 贝叶斯深度学习的优点：
  - 贝叶斯深度学习比非贝叶斯深度学习更加 robust。因为我们可以采样一次又一次，细微改变权重对深度学习造成的影响在贝叶斯深度学习中可以得到解决。
  - 贝叶斯深度学习可以提供不确定性（uncertainty），非 softmax 生成的概率。详情参见 Deep Learning Is Not Good Enough, We Need Bayesian Deep Learning for Safe AI。


### 15.3 贝叶斯深度学习是如何进行训练的

- 对于非贝叶斯神经网络，在各种超参数固定的情况下，我们训练一个神经网络想要的就是各个层之间的权重w和偏置b

- 对于贝叶斯神经网络，我们训练的目标是得到权重和偏置的分布，因此需要用到上面的贝叶斯公式(1式)

  假设用数据集$D=\{(x_1,y_1),(x_2,y_2),\cdots,(x_m,y_m)\}$训练一个贝叶斯神经网络，那么贝叶斯公式可以写成：
  $$
  p(w|x,y)=\frac{p(y|x,w)p(w)}{\int p(y|x,w)p(w)dw}\tag{2}
  $$

  - 我们想要得到权重w，就是这里的后验概率$p(w|x,y)$

  - 先验概率$p(w)$：可以根据经验给定，比如初始时设成标准正态分布

  - 似然$p(y|x,w)$：是一个关于w的函数

  **但要对分母在取值空间上进行积分基本是无法进行的，因此有3种训练方式**
  
  1. 用MCMC(Markov Chains Monte Carlo)采样去近似分母的积分
  2. 直接用一个简单点的分布q去近似后验概率的分布p，即不管分母怎么积分直接最小化q和p之间的差异。
  3. 使用MC(Monte Carlo) dropout

### 15.4 贝叶斯深度学习对不确定性的度量

- 什么是不确定性

  模型对自己结果的不确定度度量。

  考虑一个模型，用来训练分类人脸和猩猩脸，但如果测试时给它一个猫脸，它还是会给出是人脸或者猩猩脸而不会说“我真不知道这个脸是什么脸”。因此我们希望网络可以对自己的结果给出一个不确定度的估计。

- 两种不确定性：

  1. 偶然不确定性Aleatoric Uncertainty

     因为测量精度、气流扰动、数据标定不精确等等无法避免的**噪声误差**，会引起结果的浮动变化。这个噪声误差被叫做偶然不确定性。

  2. 认知不确定性Epistemic Uncertainty

     由于Input data不存在于已经见过的数据分布之中。比如上面分类人脸和猩猩脸的模型，给它一个猫脸肯定就会出错。

# 四、CNN卷积神经网络

[Convolutional Neural Networks](https://www.zybuluo.com/hanbingtao/note/485480)

## 1. 基本概念

- 为什么处理图片时要用卷积层来代替fc全连接层

  比如一个5X5的图片，3个RGB通道，那么一个神经元就需要75个权重Weights。如果1个全连接层有1000个神经元，那1层就要算75000个权重！！！更何况图片不可能只有5X5那么小。所以用全连接层来训练图片是impractical的。

  - 将图像展开为向量会丢失空间信息；
  - 参数过多效率低下，训练困难；
  - 大量的参数也很快会导致网络过拟合。

- CNN Prototype原型

  - 卷积神经网络主要由这几类层构成：输入层、卷积层，ReLU层、池化（Pooling）层和全连接层

  - 可见最后会用一层Fully connected layer
  
    - 用卷积层得出的特征features来做最终的决定
  
    - 一般1或2层全连接层
  
  
  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/CNN%20Prototype.png?raw=true)

- 不变性invariance：
  - CNN中不变性意味着即使目标的外观发生了某种变化，但是你依然可以把它识别出来
  - 根据[文章](https://pyimagesearch.com/2021/05/14/are-cnns-invariant-to-translation-rotation-and-scaling/)可知：CNN只有translation invariance没有rotation/scaling invariance。

### 1.1 Image Filter

**卷积核（convolutional kernel）**：可以看作对某个局部的加权求和；它是对应局部感知，它的原理是在观察某个物体时我们既不能观察每个像素也不能一次观察整体，而是先从局部开始认识，这就对应了卷积。卷积核的大小一般有1x1,3x3和5x5的尺寸。

卷积核的个数就对应输出的通道数（channels），这里需要说明的是对于输入的每个通道，输出每个通道上的卷积核是不一样的。比如输入是28x28x192(WxDxK,K代表通道数)，然后在3x3的卷积核，卷积通道数为128，那么卷积的参数有3x3x192x128，其中前两个对应的每个卷积里面的参数，后两个对应的卷积总的个数（一般理解为，卷积核的权值共享只在每个单独通道上有效，至于通道与通道间的对应的卷积核是独立不共享的，所以这里是192x128）。

计算：每个通道都需要跟一个卷积核做卷积运算，然后将**结果相加**得到一个特征图的输出，这里有4个过滤器，因此得到4个特征图的输出，输出通道数为4。

#### 1.1.1 一些过滤器：

- 卷积核和过滤器的区别：
  - 卷积核就是由长和宽来指定的，是一个二维的概念。
  - 而过滤器是是由长、宽和深度指定的，是一个三维的概念。
    - 过滤器可以看作是卷积核的集合
    - 过滤器比卷积核高一个维度——深度。
##### 1. Sharpen:锐化卷积核

$$
\left[
   \begin{matrix}
     -1 & -1 & -1 \\
     -1 & 8 & -1 \\
     -1 & -1 & -1
    \end{matrix}
    \right]
$$

  - 主要作用就是对图片进行锐化操作，也就是让图像的边缘更加锐利，相当于给物体描边。

  - 图像的边缘往往就是变化较大的地方，也就是图像的高频部分，因此锐化卷积核就是一种高通滤波器

##### 2.Box Mean均值滤波
$$
  \frac{1}{9}\left[
   \begin{matrix}
     1 & 1 & 1 \\
     1 & 1 & 1 \\
     1 & 1 & 1
    \end{matrix}
    \right]
$$
  和下面高斯滤波共有的特征:

  - 滤波器中元素之和为1，输出亮度与输入基本一致；
  - 均为低通滤波器，主要用于图像模糊blur/平滑smooth处理、消除噪点noise；
  - 核越大，模糊程度越大；

##### 3.Gaussian blur高斯滤波
$$
\frac{1}{16}\left[
   \begin{matrix}
     1 & 2 & 1 \\
     2 & 4 & 2 \\
     1 & 2 & 1
    \end{matrix}
    \right]
$$

  - 和均值滤波一样都主要用于模糊图像，火灾图像处理中消除图像中的噪点
  - 但高斯滤波虽然元素总和也为1，但每个位置的权重不一样，权重在行和列上的分布均服从高斯分布，故称高斯滤波器。
  - 高斯分布的标准差越大，则模糊程度越大。

##### 4. 一阶微分算子Prewitt/Sobel

用于边缘检测vertical/horizontal edge detector

一阶微分算子可以计算出某个方向上物体的边缘，但往往对噪声较为敏感，且边缘检测敏感度依赖于物体的大小。

- **Prewitt算子**

  Prewitt算子就是对图像进行差分来近似对图像的某个部分求一阶导数。由于导数还具有方向性，因此同样大小的Prewitt算子还有8种不同的类型，目的在于求上、下、左、右、左上、左下、右上、右下8个方向上的梯度。

  其中求向右梯度的3×3Prewitt算子如下所示：
  $$
  \left[
     \begin{matrix}
       -1 & 0 & 1 \\
       -1 & 0 & 1 \\
       -1 & 0 & 1
      \end{matrix}
      \right]
  $$
  右上梯度的3x3 prewitt算子为：
  $$
  \left[
     \begin{matrix}
       0 & 1 & 1 \\
       -1 & 0 & 1 \\
       -1 & -1 & 0
      \end{matrix}
      \right]
  $$
  
- **Sobel算子**

  Sobel算子则是Prewitt算子的改进版，对中间的元素适当进行了加权，Sobel算子之于Prewitt算子类似于高斯滤波之于均值滤波。

  计算向上梯度的3x3Sobel算子如下：
  $$
  \left[
     \begin{matrix}
       1 & 2 & 1 \\
       0 & 0 & 0 \\
       -1 & -2 & -1
      \end{matrix}
      \right]
  $$

##### 5. 二阶微分算子Laplace,LoG,DoG

- Prewitt算子和Sobel算子都是近似对图像进行一阶导数的计算，只能提取出某个具体方向的边缘。

  - 由微积分的知识可知，一个函数的二阶导数为0时，代表此处的一阶导数取得极值，对应地也就表明原函数在此处的变化最大。
  - 因此往往还可以根据图像的二阶导数过零点的位置，来预测图像中变化最剧烈的地方，也许对应物体的边缘。
  - 与一阶微分算子不同，这些二阶微分算子对边缘的计算具有**旋转不变性**，也就是可以检测出各个方向上的边缘。

- Laplace算子

  Laplace算子分为两种，分别考虑4-邻接（D4）和8-邻接（D8）两种邻域的二阶微分。

  - 一个3x3的4-邻接Laplace算子
    $$
    \left[
       \begin{matrix}
         0 & 1 & 0 \\
         1 & -4 & 1 \\
         0 & 1 & 0
        \end{matrix}
        \right]
    $$

  - 一个8-邻接的 Laplace算子还考虑到斜对角方向上的梯度
    $$
    \left[
       \begin{matrix}
         1 & 1 & 1 \\
         1 & -8 & 1 \\
         1 & 1 & 1
        \end{matrix}
        \right]
    $$

    - 可以看出D8 Laplace算子与锐化卷积核类似，锐化卷积核计算的是中心像素减去周围像素的差值（中心权重为正，周边权重为负）
    - 而Laplace算子则是周围像素之和减去中心像素的差值（中心权重为负，周边权重为正）。

- LoG算子

  Laplace算子对噪声依然很敏感。因此常常先使用高斯滤波器对图像进行平滑操作，再使用Laplace算子计算二阶微分。二者结合称为LoG算子（Laplacian of Gaussian），该算子可以更加稳定地计算图像的二阶微分。

  常用的5×5的LoG算子如下所示：
  $$
  \left[
     \begin{matrix}
       0 & 0 & 1 & 0 & 0 \\
       0 & 1 & 2 & 1 & 0 \\
       1 & 2 & -16 & 2 & 1 \\
       0 & 1 & 2 & 1 & 0 \\
       0 & 0 & 1 & 0 & 0 \\
      \end{matrix}
      \right]
  $$

- DoG算子

  LoG算子的计算量较大，因此有数学家发明了DoG（Difference of Gaussians）算子来近似LoG算子。DoG算子翻译为高斯差分算子，从名称上可以看出，就是使用两个标准差不同的高斯滤波器对图像进行滤波操作，再将滤波后的两个结果相减，最后的结果可以近似LoG算子。

#### 1.1.2 1X1卷积核

1x1卷积核，又称为网中网（Network in Network）。

- 1x1卷积核的作用

  - **升维/降维**：

    collapse number of channels

    - 由于 1×1 并不会改变 height 和 width，改变通道的第一个最直观的结果，就是可以将原本的数据量进行增加或者减少。
    - 改变的只是 height × width × channels 中的 channels 这一个维度的大小而已

  - **增加非线性**

    learn more complex functions by introducing additional non-linearities

    - 1*1卷积核，可以在保持feature map尺度不变的（即不损失分辨率）的前提下大幅增加非线性特性（利用后接的非线性激活函数），把网络做的很deep
    - 一个filter对应卷积后得到一个feature map，不同的filter(不同的weight和bias)，卷积以后得到不同的feature map，提取不同的特征，得到对应的specialized neuron[7]

### 1.2 Convolution Layers卷积层 

- 卷积层的作用
  - **滤波器的作用**
  - **可以被看做是神经元的一个输**
  - **降低参数的数量**

- Padding填充(图片四周填充一圈值)

  - Why padding
    - Sizes get small too quickly
    - corner pixel is only used once
  - 最常用的是填充0

- Dimensions:

  N=7：图片是7X7X3   （X3是RGB通道）

  F=3：过滤器是3X3X3 （最后X3要和输入对应），F是filter的size

  S=1/2/3：stride过滤器前进的格子数

  - 如果输入时32X32X3，用5个5X5X3的过滤器，那么过滤后输出是28x28x5。(注意最后X5和过滤的个数一致)，接着如果对28x28x5还要过滤，就要用5x5x5的过滤器了。
  - padding的圈数应该是：$p=\frac{F-1}{2}$
  - Padding后再过滤后的维度是：$(\left[\frac{N+2P-F}{S}\right]+1)\times(\left[\frac{N+2P-F}{S}\right]+1)$

#### 1.2.1 权值共享

- 在卷积层中权值共享是用来控制参数的数量。假如在一个卷积核中，每一个感受野采用的都是不同的权重值（卷积核的值不同），那么这样的网络中参数数量将是十分巨大的。
- 权值共享是基于这样的一个合理的假设：如果一个特征在计算某个空间位置 (x1,y1)(x1,y1) 的时候有用，那么它在计算另一个不同位置 (x2,y2)(x2,y2) 的时候也有用。
- 一个数据体尺寸为[55x55x96]的就有96个深度切片depth slice，每个尺寸为[55x55]，其中在每个深度切片上的结果都使用同样的权重和偏差获得的。
  - 如果有96个核大小为 11x11的卷积核，图像是RGB 3 通道的，那么就共有96x11x11x3=34,848个不同的权重，总共有34,944个参数（因为要+96个偏差），并且在每个深度切片中的55x55 的结果使用的都是同样的参数。
  - 在反向传播的时候，都要计算每个神经元对它的权重的梯度，但是需要把同一个深度切片上的所有神经元对权重的梯度累加，这样就得到了对共享权重的梯度。这样，每个切片只更新一个权重集。

### 1.3 Pooling Layer池化层

可以理解为汇聚层,通常在连续的卷积层之间会周期性地插入一个池化层。

- 作用：

  1. downsamping下采样，降维，减少网络中参数的数量，使得计算资源耗费变少，
  2. 实现非线性，能防止过拟合。
  3. 实现特征不变性feature invariance
     - translation invariance
     - rotation invariance
     - scaling invariance

- 与卷积层对比：

  - Conv Layer = feature extraction
    - Computes a feature in a given region
  - Pooling Layer = feature selection
    - Picks the strongest activation in a region

- Dimension:

  - Input is a volume of size $W_{in}\times H_{in}\times D_{in}$   宽，高，深度 
  - 2个超参数
    - Spatial filter extent F
    - Stride S
  - Input is a volume of size $W_{out}\times H_{out}\times D_{out}$
    - $W_{out}=\frac{W_{in}-F}{S}+1$
    - $H_{out}=\frac{H_{in}-F}{S}+1$
    - $D_{out}=D_{in}$

- 类型：

  - Max Pooling

    ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/Max%20Pooling.png?raw=true)

  - Average Pooling

    ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/Average%20Pooling.png?raw=true)
  
  - GAP(global average pooling)全局平均池化
    - 对整个网路在结构上做正则化防止过拟合。
    - GAP可以实现任意图像大小的输入.
    - 但gap会导致收敛速度减慢

- 两者对比：

  根据相关理论，特征提取的误差主要来自两个方面

  1. 邻域大小受限造成的估计值方差增大。

     average-pooling能减小第一种误差，更多的保留图像的背景信息

  2. 卷积层参数误差造成估计均值的偏移。

     max-pooling能减小第二种误差，更多的保留纹理信息。

### 1.4 Receptive Field感受野

- 什么是感受野？

  在处理图像这样的高维度输入时，让每个神经元都与前一层中的所有神经元进行全连接是不现实的。相反，我们让每个神经元只与输入数据的一个局部区域连接。**该连接的空间大小叫做神经元的感受野**（receptive field），用来表示网络内部的不同位置的神经元对**原图像**的感受范围的大小。

  感受野的尺寸是一个超参数（其实就是滤波器的空间尺寸），**在深度方向上，这个连接的大小总是和输入量的深度相等**。

  感受野讲解了卷积层中每个神经元与**输入数据体**之间的连接方式。

- 感受野是指特征图上的某个点能看到的输入图像的区域,即特征图上的点是由输入图像中感受野大小区域的计算得到的.

  The size of the region in the input space that a pixel in the output space is affected by.

- 神经元感受野的值越大表示其能接触到的原始图像范围就越大，也意味着它可能蕴含更为全局，语义层次更高的特征；相反，值越小则表示其所包含的特征越趋向局部和细节。因此**感受野的值可以用来大致判断每一层的抽象层次**

- 一个例子

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/%E6%84%9F%E5%8F%97%E9%87%8E.jpg?raw=true)

  - 红色为输入数据，假设输入数据体尺寸为[32x32x3]（比如CIFAR-10的RGB图像），如果感受野（或滤波器尺寸）是5x5，那么卷积层中的每个神经元会有输入数据体中[5x5x3]区域的权重，共5x5x3=75个权重（还要加一个偏差参数）。
    - 无论输入输入数据是多少层，一个卷积核就对应一个偏置
  - 这个连接在深度维度上的大小必须为3，和输入数据体的深度一致
  - 感受野有75个权重，这75个权重是通过学习进行更新的，所以很大程度上这些权值之间是不相等（也就对于同一个卷积核，它对于与它连接的输入的每一层的权重都是独特的）

### 1.5 Common Performance Metrics

- Top-1 score: check if a sample’s top class (i.e. the one with highest probability) is the same as its target label
- Top-5 score: check if your label is in your 5 first predictions (i.e. predictions with 5 highest probabilities)
  - Top-5 error: percentage of test samples for which the correct class was not in the top 5 predicted classes

### 1.6 Inception Layer

Tired of choosing filter sizes?

Use them all!

- 为什么要用inception layer:

  - Avoid choosing kernel size
  - Dimensionality reduction leads to lower computational cost

- 结构

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/inception%20layer.png?raw=true)

  - 和卷积层、池化层顺序连接的结构（如VGG网络）相比，这样的结构主要有以下改进：

    - 一层block就包含1x1卷积，3x3卷积，5x5卷积，3x3池化(使用这样的尺寸不是必需的，可以根据需要进行调整)。这样，网络中每一层都能学习到“稀疏”（3x3、5x5）或“不稀疏”（1x1）的特征，既增加了网络的宽度，也增加了网络对尺度的适应性；
    - 通过deep concat在每个block后合成特征，获得非线性属性

  - 按照这样的结构来增加网络的深度，虽然可以提升性能，但是还面临计算量大（参数多）的问题。为改善这种现象，GooLeNet借鉴Network-in-Network的思想，使用1x1的卷积核实现降维操作(也间接增加了网络的深度)，以此来减小网络的参数量

    ![](https://pic3.zhimg.com/80/v2-4c7116b1d363bcf9e492e8437dcc108a_720w.jpg)

## 2. 与全连接层的区别

Convolutions vs Fully-Connected

- In contrast to fully-connected layers, we want to restrict限制 the degrees of freedom
  - FC is somewhat brute force
  - Convolutions are structured
- Sliding window to with the same filter parameters to extract提取 image features
  - Concept观念 of weight sharing
  - Extract same features independent of location
- Output size:
  - Fully-Connected layer: One layer of neurons, independent
  - Convolutional Layer: Neurons arranged排列 in 3 dimensions

### 2.1 Spatial Batch Normalization

一般的batch normalization见9.5。

- BatchNorm for convolutional NN = spatial batchnorm

  - Input size$(N,C,W,H)\rightarrow(N,C,H\times W)$
  - Compute minibatch mean and variance across N, W, H (i.e. we compute mean/var for each channel C))

- Normalization acts on channel dimension instead of per feature

  - Each BatchNorm2d layer has two weights per channel

    比如有conv->pool->BN->...这一段有8通道，那么BN层有2X8=16个可训练参数

- 对于全连接层：is normalized over all neurons.

  对于卷积层：is normalized over each channel.

- Other Normalizations

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/Normalizations.png?raw=true)

### 2.2 Dropout for convolutional layers

全连接层是随机不激活某一个unit,但卷积层是三维结构的，这么做并不能提升性能。

所以Spatial Dropout randomly sets entire feature maps to zero

- Conv: drop feature map at random
- fully connected: drop weights at random

## 3. CNN的一些Clasic Architectures

### 3.1 LeNet

第一个成功的卷积神经网络应用，是Yann LeCun在上世纪90年代实现的。当然，最著名还是被应用在识别数字和邮政编码等的LeNet结构。

- 特点
  - Digit recognition: 10 classes
  - Conv -> Pool -> Conv -> Pool -> Conv -> FC
  - As we go deeper: Width, Height 下降；Number of Filters增加

### 3.2 [AlexNet](https://proceedings.neurips.cc/paper/2012/file/c399862d3b9d6b76c8436e924a68c45b-Paper.pdf)

这个网络的结构和LeNet非常类似，但是更深更大，并且使用了**层叠的卷积层**来获取特征（**之前通常是只用一个卷积层并且在其后马上跟着一个汇聚层**）。

- 特点：
  - Similar to LeNet but much bigger (~1000 times)
  - Use of ReLU instead of tanh/sigmoid

### 3.3 [VGGNet](https://arxiv.org/abs/1409.1556)

它主要的贡献是展示出网络的**深度是算法优良性能的关键部分**

- Striving奋斗 for simplicity
- CONV = 3x3 filters with stride 1, same convolutions
- MAXPOOL = 2x2 filters with stride 2
- Conv -> Pool -> Conv -> Pool -> Conv -> FC
- As we go deeper: Width, Height 下降；Number of Filters增加
- Called VGG-16: 16 layers that have weights

- Large but simplicity makes it appealing有吸引力的

### 3.4 [ResNet](https://arxiv.org/abs/1512.03385)

残差网络（Residual Network）是ILSVRC2015的胜利者，由何恺明等实现。它使用了特殊的**跳跃链接**，大量使用了**批量归一化**（batch normalization）。这个结构同样在最后没有使用全连接层。

### 3.5 [GoogLeNet](https://arxiv.org/abs/1409.4842)

它能够**显著地减少网络中参数的数量**（AlexNet中有60M，该网络中只有4M）。还有，这个论文中**没有使用卷积神经网络顶部使用全连接层**，而是使用了一个平均汇聚，把大量不是很重要的参数都去除掉了。

### 3.6 [Xception Net](https://arxiv.org/abs/1610.02357)

### 3.7 [U-Net](https://arxiv.org/abs/1505.04597)

### 3.8 [EfficientNet](https://arxiv.org/abs/1905.11946)

### 3.9 [Fast R-CNN](https://arxiv.org/abs/1504.08083)

## 4. FCN全卷积网络

Fully Convolutional Network，[知乎参考](https://zhuanlan.zhihu.com/p/30195134)

- 用于语义分割

  语义分割(Semantic Segmentation)的目的是对图像中**每一个像素点进行分类**，与普通的分类任务只输出某个类别不同，语义分割任务输出是与**输入图像大小相同的图像**，输出图像的每个像素对应了输入图像每个像素的类别。

- 相比于传统CNN

  - 用于图像分类的CNN网络一般结构是"**卷积-池化-卷积-池化-全连接**"，其中**卷积和全连接层**是有参数的，池化则没有参数。
  - 但全连接层会让目标的位置信息消失，只保留语义信息。因此FCN需要将全连接层转换为卷积层，以同时保留**位置信息及语义信息**，达到给每个像素分类的目的。
    - 全连接层和卷积层之间唯一的不同就是卷积层中的神经元只与输入数据中的一个局部区域连接，并且在卷积列中的神经元共享参数。然而在两类层中，神经元都是计算点积，所以它们的函数形式是一样的。因此，将此两者相互转化是可能的
  - 转换成全卷积层后要+global average pooling
  
- 上采样：

  由于经过卷积和池化后得到的feature map相比原图缩小了好几倍，为了得到与输入图像大小相同的输出，需要对feature map进行上采样upsamping。

  - 反卷积：

    上采样upsamping的一种方式。[参考](https://naokishibuya.medium.com/up-sampling-with-transposed-convolution-9ae4f2df52d0)

# 五、Recurrent Neural Networks循环神经网络

- 循环神经网络是一个在时间上传递的神经网络，网络的深度就是时间的长度。该神经网络是专门用来处理**时间序列Time Sequence**问题的，能够提取时间序列的信息。如果是前向神经网络，每一层的神经元信号只能够向下一层传播，样本的处理在时刻上是独立的。对于循环神经网络而言，神经元在这个时刻的输出可以直接影响下一个时间点的输入，因此该神经网络能够处理时间序列方面的问题。

- 为什么需要RNN?

  之前的模型都只能单独的取处理一个个的输入，前一个输入和后一个输入是完全没有关系的。但是，某些任务需要能够更好的处理**序列**的信息，即前面的输入和后面的输入是有关系的。

  比如：

  - 当我们在理解一句话意思时，孤立的理解这句话的每个词是不够的，我们需要处理这些词连接起来的整个序列；
  - 当我们处理视频的时候，我们也不能只单独的去分析每一帧，而要分析这些帧连接起来的整个序列。

- RNN的优缺点

  - 优点：

    1. can model temporal sequential data

    缺点：

    1. Vanishing/Exploding Gradient发生在时间轴上

       It cannot system very lengthy sequences if the usage of Tanh or Relu as an activation feature.

- RNN结构

  一个简单的循环神经网络，它由输入层、一个隐藏层和一个输出层组成：

  ![](https://raw.githubusercontent.com/Fernweh-yang/Reading-Notes/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/RNN%E6%9C%80%E7%AE%80%E7%BB%93%E6%9E%84.jpg)

  - 如果把上面有W的那个带箭头的圈去掉，它就变成了最普通的**全连接神经网络**

  - 加入了w后：**循环神经网络**的**隐藏层**的值s不仅仅取决于当前这次的输入x，还取决于上一次/时刻**隐藏层**的值s。**权重矩阵** W就是**隐藏层**上一次/时刻的值作为这一次的输入的权重

    ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/rnn%E7%BB%93%E6%9E%84%E5%85%B7%E4%BD%93%E5%8C%96.jpg?raw=true)

- 用公式表达：
  $$
  O_t=g(VS_t)\\
  S_t=g(UX_t+WS_{t-1})\\
  $$

  - $S_t$的值不仅仅取决于$X_t$,还取决于上一刻的值$S_{t-1}$
  - 不同时刻的3个权重U,W,V都是保持不变的

## 1. RNN的不同用处

![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/RNN%E4%B8%8D%E5%90%8C%E7%BB%93%E6%9E%84.png?raw=true)

- One to one: Classic Neural Networks for Image Classification
- One to many: Image captioning图像字幕
- many to one: Language recognition
- many to many: Machine translation机器翻译
- many to many: Event classification时间分类

## 2. Long Term Dependencies长期依赖

- 长期依赖产生的原因是当神经网络的节点经过许多阶段的计算后，之前比较长的时间片的特征已经被覆盖。

  it's difficult for traditional RNNs to learn long-term dependencies due to vanishing gradients

  就导致了越晚的输入影响越大，越早的输入影响越小。

- Simple Recurrence: $A_t=\theta^tA_0$

  - 将权重eigendecomposition特征分解
    $$
    \theta=Q\Lambda Q^TA_0
    $$
    可得：$A_t=Q\Lambda^tQ^TA_0$

  - 梯度消失/爆炸

    - Vanishing gradient：当eigenvalues with magnitude量级 less than one
      - 梯度消失不能简单的通过类似梯度截断的阈值式方法来解决，因为长期依赖的现象也会产生很小的梯度
      - 如果我们刻意提高小梯度的值将会使模型失去捕捉长期依赖的能力。
    - Exploding gradient：当eigenvalues with magnitude量级 larger than one
      - 处理梯度爆炸可以采用梯度截断的方法。所谓梯度截断是指将梯度值超过阈值 $\theta$ 的梯度手动降到$\theta $ 。虽然梯度截断会一定程度上改变梯度的方向，但梯度截断的方向依旧是朝向损失函数减小的方向。

## 3. Long Short Term Meomry

LSTM具有记忆长短期信息的能力的神经网络.

- LSTM提出的动机是为了解决上面我们提到的长期依赖问题。传统的RNN节点输出仅由权值，偏置以及激活函数决定

- LSTM之所以能够解决RNN的长期依赖问题，是因为LSTM引入了门（gate）机制用于控制特征的流通和损失。

  The cell state in LSTMs improve the gradient flow and thereby allows the network to learn longer dependencies.

  - lstm suffer遭受 less rome vanishing gradient的另一个原因是：

    Highway for gradients through the cell state.

- LSTM是由一系列LSTM单元（LSTM Unit）组成，其链式结构如下图:

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/LSTM%E7%BB%93%E6%9E%84.png?raw=true)

  ![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/LSTM%E6%8B%86%E8%A7%A3.png?raw=true)

  - **黄色方框**表示一个神经网络层，由权值，偏置以及激活函数组成

  - **粉色圆圈**表示元素级别操作

  - **单元状态(cell state)** $C_t$:LSTM单元最上面的那条传送带。它自始至终存在于LSTM的整个链式系统中

    $C_t=f_t\times C_{t-1}+i_t\times \tilde{C}_t$

    - $f_t$：遗忘门forget gate。是一个每个元素均位于[0,1] 范围内的向量。通常使用sigmoid作为激活函数。精髓！

      - $f_t=\sigma(W_f\cdot[h_{t-1},x_t]+b_f)$
      - 用于控制$C_{t-1}$的哪些特征用于更新$C_t$

      $i_t$：输入门input gate，和$f_t$一样。

      - $i_t=\sigma(W_i\cdot[h_{t-1},x_t]+b_i)$
      - 用于控制$\tilde{C}_t$的哪些特征用于更新$C_t$

      $o_t$：输出门output gate，和$i_t$一样。

      - $o_t=\sigma(W_o\cdot[h_{t-1},x_t]+b_o)$$

    - $C_{t-1}$：表示哪些特征被用于计算$C_t$

    - $\tilde{C}_t$：表示单元状态更新值Cell update，单元状态更新值的激活函数通常使用tanh

      $\tilde{C}_t=tanh(W_c\cdot [h_{t-1},x_t]+b_c)$

    - $h_t$隐节点输出output：$h_t=o_t*tanh(C_t)$

- When coding an LSTM, we have to define the size of the hidden state. Dimensions need to match

  - Input, states, and gates not limited to 1st-order tensors
  - Gate functions can consist of FC and CNN layers

- Problems with RNN

  - Each word is dependent on the words coming before it
  - Vanishing gradient problem.
  - Long-short term memory dependencies are not that long.

# 六、Transformers

- [知乎讲解](https://zhuanlan.zhihu.com/p/48508221)参考于：[外国一博文](http://jalammar.github.io/illustrated-transformer/)

- [参考2](https://luweikxy.gitbook.io/machine-learning-notes/self-attention-and-transformer)，据说很好

## 1. Deep Learning Revolution

|                  | Deep Learning     | Deep Learning 2.0 |
| ---------------- | ----------------- | ----------------- |
| Main idea        | Convolution       | Attention         |
| Field invented   | Computer vision   | NLP               |
| Started          | NeurIPS 2012      | NeurIPS 2017      |
| Paper            | AlexNet           | Transformers      |
| Conquered vision | Around 2014-2015  | Around 2020-2021  |
| Augmented        | Traditional ML/CV | CNNs, RNNs        |

## 2. 解决RNN的问题

- [Attention is all you need](https://arxiv.org/abs/1706.03762)

  文章提出了Attention注意力机制和Transformer的概念。Transformer中抛弃了传统的CNN和RNN，整个网络结构完全是由Attention机制组成。更准确地讲，Transformer由且仅由self-Attenion和Feed Forward Neural Network组成。一个基于Transformer的可训练的神经网络可以通过堆叠Transformer的形式进行搭建。

- 采用Attention机制的原因是考虑到RNN（或者LSTM，GRU等）的计算限制为是顺序的，也就是说RNN相关算法只能从左向右依次计算或者从右向左依次计算，这种机制带来了两个问题：

  - 时间片 t 的计算依赖 t-1 时刻的计算结果，这样限制了模型的并行能力
  - 顺序计算的过程中信息会丢失，尽管LSTM等门机制的结构一定程度上缓解了长期依赖的问题，但是对于特别长期的依赖现象,LSTM依旧无能为力。

# 七、GAN

## 1. GAN的基本介绍

生成对抗网络（GAN，Generative Adversarial Networks）作为一种优秀的生成式模型，引爆了许多图像生成的有趣应用。GAN相比于其他生成式模型，有两大特点：

- 不依赖任何先验假设。传统的许多方法会假设数据服从某一分布，然后使用极大似然去估计数据分布。
- 生成real-like样本的方式非常简单。GAN生成real-like样本的方式通过生成器(Generator)的前向传播，而传统方法的采样方式非常复杂

## 2. GAN的基本思想

- 一个形象的例子：

  假如你是一名篮球运动员，你想在下次比赛中得到上场机会。
  于是在每一次训练赛之后你跟教练进行沟通：

  *你：教练，我想打球*
  *教练：（评估你的训练赛表现之后）... 算了吧*
  *（你通过跟其他人比较，发现自己的运球很差，于是你苦练了一段时间）*

  *你：教练，我想打球*
  *教练：... 嗯 还不行*
  *（你发现大家投篮都很准，于是你苦练了一段时间的投篮）*

  *你：教练，我想打球*
  *教练： ... 嗯 还有所欠缺*
  *（你发现你的身体不够壮，被人一碰就倒，于是你去泡健身房）*

  *......*

  *通过这样不断的努力和被拒绝，你最终在某一次训练赛之后得到教练的赞赏，获得了上场的机会。*
  *值得一提的是在这个过程中，所有的候选球员都在不断地进步和提升。因而教练也要不断地通过对比场上球员和候补球员来学习分辨哪些球员是真正可以上场的，并且要“观察”得比球员更频繁。随着大家的成长教练也会会变得越来越严格。*

- GAN就是通过对抗的方式，去学习数据分布的生成式模型。所谓的对抗，指的是生成网络和判别网络的互相对抗。生成网络尽可能生成逼真样本，判别网络则尽可能去判别该样本是真实样本，还是生成的假样本。

## 3. GAN的基本结构

GAN(Generative Adversarial Network)生成对抗网络的主要结构包括一个**生成器**G（Generator）和一个**判别器**D（Discriminator）。

- 上面的例子中：

  - 球员=生成器：我们需要他在球场上能有好的表现
  - 教练=判别器：教练员来指导球员训练，告诉他们训练得怎么样，直到真的能够达到上场的标准。

- 手写字的例子：

  ![](https://pic4.zhimg.com/v2-5ca6a701d92341b8357830cc176fb8a3_r.jpg)

  - 定义一个模型来作为生成器（图三中蓝色部分Generator），能够输入一个向量，输出手写数字大小的像素图像。
  - 定义一个分类器来作为判别器（图三中红色部分Discriminator）用来判别图片是真的还是假的（或者说是来自数据集中的还是生成器中生成的），输入为手写图片，输出为判别图片的标签。

## 4.GAN的训练

### 4.1 生成器：

- 生成器可以是任意可以输出图片的模型，比如最简单的全连接神经网络，又或者是反卷积网络等。
- 对于生成器，输入需要一个n维度向量random noise vector，输出为图片像素大小的图片。因而首先我们需要得到输入的向量。
- 这里输入的向量我们将其视为携带输出的某些信息，比如说手写数字为数字几，手写的潦草程度等等。由于这里我们对于输出数字的具体信息不做要求，只要求其能够最大程度与真实手写数字相似（能骗过判别器）即可。所以我们使用随机生成的向量来作为输入即可，这里面的随机输入最好是满足常见分布比如均值分布，高斯分布等。

### 4.2 判别器：

- 对于判别器不用多说，往往是常见的判别器，输入为图片，输出为图片的真伪标签。
  - 同理，判别器与生成器一样，可以是任意的判别器模型，比如全连接网络，或者是包含卷积的网络等等


### 4.3 如何训练

[参考](https://zhuanlan.zhihu.com/p/33752313)

- **训练一个判别器，就是最大化实际数据和生成器抽样数据的JS散度**。

   $$ G^{*}=\arg \min_G \max_D V(G, D) $$ 

   - $V(G,D)$相当于表示真实样本和生成样本的差异程度。
   - $max_DV(G,D)$的意思是固定生成器G，尽可能地让判别器D能够最大化的识别出样本的真伪。
   - 将$L=max_DV(G,D)$看作一个整体，$min_GL$的意思是固定判别器D，找到一个生成器G，要求能够最小化真实样本和生成样本的差异
   - 通过上述min max的博弈过程，理想情况下会收敛于生成分布拟合于真实分布

- GAN是生成器和判别器的零和博弈，即$J^{(G)}=-J^{(D)}$，即min-max博弈。

  - **max what** : 在训练的第一步，我们需要提高判别器的辨别能力。即在给定生成器$G_i$的情况下，找到D的参数$\theta_d $最大化判别器对不同分布样本的JS散度（区分能力）
  - **min what**： 与此同时，对于当前分辩能力最好的判别器$D_i$，找到$\theta_g$，使得判别器分辩两类样本的区分度最小。
  - 但如果Discriminator训练的太好，Generator所有的sample都被拒绝，G就没法学习了
    -  heuristical启发式的方法：the generator maximizes the log probability of the discriminator being mistaken.
  
- 基本训练流程：

  - 初始化判别器D的参数$\theta_d$和生成器G的参数$\theta_g$ 。
  - 从真实样本中采样 m 个样本${ x^1,x^2,...x^m }$ ，从先验分布噪声中采样 m 个噪声样本${ z^1,z^2,...,z^m }$并通过生成器获取 m 个生成样本${ \tilde{x}_1,\tilde{x}_2,...,\tilde{x}_m }$ 。固定生成器G，训练判别器D尽可能好地准确判别真实样本和生成样本，尽可能大地区分正确样本和生成的样本。
  - **循环k次更新判别器之后，使用较小的学习率来更新一次生成器的参数**，训练生成器使其尽可能能够减小生成样本与真实样本之间的差距，也相当于尽量使得判别器判别错误。
    - 之所以要训练k次判别器，再训练生成器，是因为要先拥有一个好的判别器，使得能够教好地区分出真实样本和生成样本之后，才好更为准确地对生成器进行更新。
  - 多次更新迭代之后，最终理想情况是使得判别器判别不出样本来自于生成器的输出还是真实的输出。亦即最终样本判别概率均为0.5。

  

# 八、Reinforcement Learning强化学习

## 1. 基本概念

![](https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/Reinforcement%20Learning%E7%A4%BA%E6%84%8F%E5%9B%BE.png?raw=true)

- 相较于机器学习经典的有监督学习、无监督学习问题，强化学习最大的特点是在交互中学习（Learning from Interaction）

  - Agent在与环境的交互中根据获得的奖励或惩罚不断的学习知识，更加适应环境。
  - RL学习的范式非常类似于我们人类学习知识的过程，也正因此，RL被视为实现通用AI重要途径。

- Markov Process马尔可夫过程

  - Markov Property马尔可夫性

    The future is independent of the history and the present state determines the future.

    只要当前状态可知，所有的历史信息都不再需要，当前状态就可以决定未来，则认为该状态具有**马尔科夫性**。

    用状态转移概率公式表达$P[S_{t+1}|S_1,...,S_t]=P[S_{t+1}|S_t]$

  - Markov Process马尔可夫过程

    **马尔科夫过程** 又叫马尔科夫链(Markov Chain)，它是一个无记忆的随机过程，可以用一个元组<S,P>表示，其中S是有限数量的状态集，P是状态转移概率矩阵。

## 2. 主流算法分类

各算法选择[参考](https://zhuanlan.zhihu.com/p/342919579)

免模型学习（Model-Free） vs 有模型学习（Model-Based）

这2个分类的重要差异是：**智能体是否能完整了解或学习到所在环境的模型**

- 有模型学习（Model-Based）对环境有提前的认知，可以提前考虑规划，但是缺点是如果模型跟真实世界不一致，那么在实际使用场景下会表现的不好。
  - Learn the Model
    - World Models
    - I2A
    - MBMF
    - MBVE
  - Given the MOdel
    - AlphaZero
- 免模型学习（Model-Free）放弃了模型学习，在效率上不如前者，但是这种方式更加容易实现，也容易在真实场景下调整到很好的状态。所以**免模型学习方法更受欢迎，得到更加广泛的开发和测试。**
  - Policy Optimization
    - Policy Gradient
    - A2C/A3C
    - PPO
    - TRPO
  - Q-Learning
    - DQN
    - C51
    - QR-DQN
    - HER
  - 两者兼有
    - DDPG
    - TD3
    - SAC

除了免模型学习和有模型学习的分类外，强化学习还有其他几种分类方式：

- 基于概率 VS 基于价值
- 回合更新 VS 单步更新
- 在线学习 VS 离线学习

## 3. Q-learning

### 3.1 基本理解

- **Q-学习**是强化学习的一种方法。Q-学习就是要记录下学习过的策略，因而告诉智能体什么情况下采取什么行动会有最大的奖励值。Q-学习不需要对环境进行建模，即使是对带有随机因素的转移函数或者奖励函数也不需要进行特别的改动就可以进行。
- **Q**为**动作效用函数**（action-utility function），用于评价在特定状态下采取某个动作的优劣。它是**智能体的记忆**。
- 根据行为的后果来加强或减弱行为。如果后果是好的，则加强行为，如果后果是坏的则减弱行为。通过这种方式来获得趋利避害的能力。公式后一部分是对后果的估值。
  q-training就是个建立“操作性条件反射”的过程。

### 3.2 算法

### 3.3 Q-learning 和 neural networks

- 为什么要将Q-Learning与neural networks结合使用

  - For even semi big problems, calculating all state action pairs is not tractable易处理的, but we can approximate them using neural networks.

- 使用：

  Function approximator for Q values $Q^*(s,a)=Q(s,a;\theta)$

  这里的$\theta$是神经网络的参数

# 1. 数据预处理

## 1.1加载库

```python
import os
import pickle
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from tqdm import tqdm

%load_ext autoreload
%autoreload 2
%matplotlib inline

plt.rcParams['figure.figsize'] = (10.0, 8.0) # set default size of plots
plt.rcParams['image.interpolation'] = 'nearest'
plt.rcParams['image.cmap'] = 'gray'
```

## 1.2数据集下载

```python
# Set up the output dataset folder
i2dl_exercises_path = os.path.dirname(os.path.abspath(os.getcwd()))
cifar_root = os.path.join(i2dl_exercises_path, "datasets", "cifar10")

# Init the dataset and display downloading information this one time
dataset = ImageFolderDataset(
    root=cifar_root,
    force_download=False,
    verbose=True
)
```

## 1.3数据可视化

```python
def load_image_as_numpy(image_path):
    return np.asarray(Image.open(image_path), dtype=float)

classes = [
    'plane', 'car', 'bird', 'cat', 'deer',
    'dog', 'frog', 'horse', 'ship', 'truck',
]
num_classes = len(classes)
samples_per_class = 7
for label, cls in enumerate(sorted(classes)):
    for i in range(samples_per_class):
        image_path = os.path.join(
            cifar_root,
            cls,
            str(i+1).zfill(4) + ".png"
        )  # e.g. cifar10/plane/0001.png
        image = np.asarray(Image.open(image_path))  # open image as numpy array
        plt_idx = i * num_classes + label + 1  # calculate plot location in the grid
        plt.subplot(samples_per_class, num_classes, plt_idx)
        plt.imshow(image.astype('uint8'))
        plt.axis('off')
        if i == 0:
            plt.title(cls)  # plot class names above columns
plt.show()
```

## 1.4 通用数据集类(硬盘加载)

- 数据集类封装了1.将数据从给定地址加载，2.返回一个装着预处理完的数据的字典，这2种功能。

- 这通常是进行一个新深度学习工程的第一步！
- 需要有2个魔法方法，来实现类似于序列和映射的类：
  - `__len__(self)`：由此可以调用`len(dataset)`来获得数据集中照片数量。
  - `__getitem__(self,index)`：由此可以调用`dataset[i]`来直接获得数据集中的第i个图片。

```python
class ImageFolderDataset(Dataset):
    """CIFAR-10 dataset class"""
    def __init__(self, *args,
                 transform=None,
                 download_url="https://i2dl.dvl.in.tum.de/downloads/cifar10.zip",
                 **kwargs):
        super().__init__(*args, 
                         download_url=download_url,
                         **kwargs)
        
        self.classes, self.class_to_idx = self._find_classes(self.root_path)
        self.images, self.labels = self.make_dataset(
            directory=self.root_path,
            class_to_idx=self.class_to_idx
        )
        self.transform = transform
        
	# @staticmethod用于修饰类中的方法,使其可以在不创建类实例的情况下调用方法
    # 静态方法不可以引用类中的属性或方法，其参数列表也不需要约定的默认参数self。
    @staticmethod
    def _find_classes(directory):
        """
        Finds the class folders in a dataset
        :param directory: root directory of the dataset
        :returns: (classes, class_to_idx), where
          - classes is the list of all classes found
          - class_to_idx is a dict that maps class to label
        """
        classes = [d.name for d in os.scandir(directory) if d.is_dir()]
        classes.sort()
        class_to_idx = {classes[i]: i for i in range(len(classes))}
        return classes, class_to_idx

    @staticmethod
    def make_dataset(directory, class_to_idx):
        """
        Create the image dataset by preparaing a list of samples
        Images are sorted in an ascending order by class and file name
        :param directory: root directory of the dataset
        :param class_to_idx: A dict that maps classes to labels
        :returns: (images, labels) where:
            - images is a list containing paths to all images in the dataset, NOT the actual images
            - labels is a list containing one label per image
        """
        images, labels = [], []

        for target_class in sorted(class_to_idx.keys()):
            label = class_to_idx[target_class]
            target_dir = os.path.join(directory, target_class)
            for root, _, fnames in sorted(os.walk(target_dir)):
                for fname in sorted(fnames):
                    if fname.endswith(".png"):
                        path = os.path.join(root, fname)
                        images.append(path)
                        labels.append(label)

        assert len(images) == len(labels)
        return images, labels
    
    def __len__(self):        
        length = None
        length = len(self.images)
        return length

    @staticmethod
    def load_image_as_numpy(image_path):
        """Load image from image_path as numpy array"""
        # np.asarray(): Convert the input to an array
        return np.asarray(Image.open(image_path), dtype=float)

    def __getitem__(self, index):
        data_dict = None
        if self.transform  == None:
            data_dict = {'image': ImageFolderDataset.load_image_as_numpy(self.images[index]),
                         'label': self.labels[index]}
        else:
            data_dict = {'image': self.transform(ImageFolderDataset.load_image_as_numpy(self.images[index])),
                         'label': self.labels[index]}
        return data_dict
```

## 1.5通用数据集类(内存加载)

- 1.4用硬盘加载会很慢，因为我们每次访问数据集元素时都会访问单个文件，然后将这些文件加载到内存中。
- 所以如果数据集的全部大小比较小，比如1，2个G，就可以将他们全部放入内存，来加快访问速度

```python
# 继承1.4的父类来写
class MemoryImageFolderDataset(ImageFolderDataset):
    def __init__(self, root, *args,
                 transform=None,
                 download_url="https://i2dl.dvl.in.tum.de/downloads/cifar10memory.zip",
                 **kwargs):
        # Fix the root directory automatically
        if not root.endswith('memory'):
            root += 'memory'

        super().__init__(
            root, *args, download_url=download_url, **kwargs)
        
        with open(os.path.join(
            self.root_path, 'cifar10.pckl'
            ), 'rb') as f:
            save_dict = pickle.load(f)

        self.images = save_dict['images']
        self.labels = save_dict['labels']
        self.class_to_idx = save_dict['class_to_idx']
        self.classes = save_dict['classes']

        self.transform = transform

    def load_image_as_numpy(self, image_path):
        """Here we already have everything in memory,
        so we can just return the image"""
        return image_path
    
    def __getitem__(self, index):
        data_dict = None

        if self.transform  == None:
            data_dict = {'image': MemoryImageFolderDataset.load_image_as_numpy(self,self.images[index]),
                         'label': self.labels[index]}
        else:
            data_dict = {'image': self.transform(MemoryImageFolderDataset.load_image_as_numpy(self,self.images[index])),
                         'label': self.labels[index]}

        return data_dict 
```

## 1.6 改变数据集尺寸大小

比如将图片的RGB参数从0-255改成0-1

```python
class RescaleTransform:
    """Transform class to rescale images to a given range"""
    def __init__(self, out_range=(0, 1), in_range=(0, 255)):
        """
        :param out_range: Value range to which images should be rescaled to
        :param in_range: Old value range of the images
            e.g. (0, 255) for images with raw pixel values
        """
        self.min = out_range[0]
        self.max = out_range[1]
        self._data_min = in_range[0]
        self._data_max = in_range[1]
	
    ##该方法使得类实例对象可以像调用普通函数那样，以“对象名()”的形式使用
    def __call__(self, images):
        # Formula:https://stats.stackexchange.com/questions/281162/scale-a-number-between-a-range
        images = (self.max-self.min)*(images-self._data_min)/(self._data_max-self._data_min)+self.min

        return images
```

## 1.7 正则化数据(高斯)

先用numpy库计算数据集的mean平均值和standard deviation标准差

```python
def compute_image_mean_and_std(images):
    """
    Calculate the per-channel image mean and standard deviation of given images
    :param images: numpy array of shape NxHxWxC
        (for N images with C channels of spatial size HxW)
    :returns: per-channels mean and std; numpy array of shape C
    """
    mean, std = None, None
    mean = np.mean(images,axis=(0,1,2))
    std = np.std(images,axis=(0,1,2))

    return mean, std
```

再正则化数据

```python
class NormalizeTransform:
    """
    Transform class to normalize images using mean and std
    Functionality depends on the mean and std provided in __init__():
        - if mean and std are single values, normalize the entire image
        - if mean and std are numpy arrays of size C for C image channels,
            then normalize each image channel separately
    """
    def __init__(self, mean, std):
        """
        :param mean: mean of images to be normalized
            can be a single value, or a numpy array of size C
        :param std: standard deviation标准差 of images to be normalized
             can be a single value or a numpy array of size C
        """
        self.mean = mean    
        self.std = std
        
	##该方法使得类实例对象可以像调用普通函数那样，以“对象名()”的形式使用。
    # 1.实例化：normalize_transform = NormalizeTransform()
    # 2.调用： normalize_transform(images)
    def __call__(self, images):
        images = (images-self.mean)/self.std
        return images
```

## 1.8正则，改尺寸合二为一

只需要`transform=ComposeTransform([rescale_transform, normalize_transform])`就可以把1.6,1.7两种转换一次执行

```python
class ComposeTransform:
    """Transform class that combines multiple other transforms into one"""
    def __init__(self, transforms):
        """
        :param transforms: transforms to be combined
        """
        self.transforms = transforms

    def __call__(self, images):
        for transform in self.transforms:
            images = transform(images)
        return images
```

## 1.9实现加载数据并处理

```python
rescale_transform = RescaleTransform()
normalize_transform = NormalizeTransform(
    mean=cifar_mean,
    std=cifar_std
)

dataset = ImageFolderDataset(
    root=cifar_root,
    transform=ComposeTransform([rescale_transform, normalize_transform])
)
```

## 1.10批量加载数据

不应该一次性将所有的数据集都加载，而应该分批(batches)加载。

1. In machine learning, we often need to load data in **mini-batches**, which are small subsets of the training dataset. How many samples to load per mini-batch is called the **batch size**.
2. In addition to the Dataset class, we use a **DataLoader** class that takes care of mini-batch construction, data shuffling, and more.
3. The dataloader is iterable and only loads those samples of the dataset that are needed for the current mini-batch. This can lead to bottlenecks瓶颈 later if you are unable to provide enough batches in time for your upcoming pipeline. This is especially true when loading from HDDs as the slow reading time can be a bottleneck in your complete pipeline later.
4. The dataloader task can easily by distributed amongst multiple processes as well as pre-fetched. When we switch to PyTorch later we can directly use our dataset classes and replace our current Dataloader with theirs :).

```python
class DataLoader:
    """
    Dataloader Class
    Defines an iterable batch-sampler over a given dataset
    """
    def __init__(self, dataset, batch_size=1, shuffle=False, drop_last=False):
        """
        :param dataset: dataset from which to load the data
        :param batch_size: how many samples per batch to load
        :param shuffle: set to True to have the data reshuffled at every epoch
        :param drop_last: set to True to drop the last incomplete batch,
            if the dataset size is not divisible by the batch size.
            If False and the size of dataset is not divisible by the batch
            size, then the last batch will be smaller.
        """
        self.dataset = dataset
        self.batch_size = batch_size
        self.shuffle = shuffle
        self.drop_last = drop_last
	
    ## 迭代器
    def __iter__(self):

        # 将batches整合成字典
        def combine_batch_dicts(batch):
            batch_dict = {}
            for data_dict in batch:
                for key,value in data_dict.items():
                    if key not in batch_dict:
                        batch_dict[key] = []
                    batch_dict[key].append(value)
            return batch_dict

        # 将字典种的value从原来的列表改成numpy的数组
        def batch_to_numpy(batch):
            numpy_batch = {}
            for key, value in batch.items():
                numpy_batch[key] = np.array(value)
            return numpy_batch

        # 将batches迭代器化
        if self.shuffle:
            index_iterator = iter(np.random.permutation(len(self.dataset)))  # define indices as iterator
        else:
            index_iterator = iter(range(len(self.dataset)))  # define indices as iterator

        batch = []
        for index in index_iterator:  # iterate over indices using the iterator
            batch.append(self.dataset[index])
            if len(batch) == self.batch_size:
                yield batch_to_numpy(combine_batch_dicts(batch))  # use yield keyword to define a iterable generator
                batch = []

        if self.drop_last == False and len(batch) > 0:
            yield batch_to_numpy(combine_batch_dicts(batch))

    def __len__(self):
        length = None
        temp = len(self.dataset)
        length = temp//self.batch_size
        if self.drop_last == False and temp/self.batch_size:
            length = temp//self.batch_size + 1 
        return length

```

# 2. 逻辑回归：判断房家贵/不贵

## 2.1加载库

```python
from exercise_code.data.csv_dataset import CSVDataset
from exercise_code.data.csv_dataset import FeatureSelectorAndNormalizationTransform
from exercise_code.data.dataloader import DataLoader

import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import seaborn as sns


pd.options.mode.chained_assignment = None  # default='warn'

%matplotlib inline
%load_ext autoreload
%autoreload 2
```

## 2.2数据集下载

```python
from exercise_code.networks.utils import *

X_train, y_train, X_val, y_val, X_test, y_test, train_dataset = get_housing_data()

print("train data shape:", X_train.shape)
print("train targets shape:", y_train.shape)
print("val data shape:", X_val.shape)
print("val targets shape:", y_val.shape)
print("test data shape:", X_test.shape)
print("test targets shape:", y_test.shape, '\n')

print('The original dataset looks as follows:')
train_dataset.df.head()
```

## 2.3分类模型

- 模型：$\mathbf{\hat{y}}  = \sigma \left( \mathbf{X} \cdot \mathbf{w} \right)$
  - $\mathbf{X} \in \mathbb{R}^{N\times (D+1)}$ be our data with $N$ samples and $D$ feature dimensions
  - $\mathbf{w}\in \mathbb{R}^{(D+1) \times 1}$ is the weight matrix of our model
  -  **sigmoid function**：$ \sigma(t) = \frac{1}{1+e^{-t}},\in[0.1] $

```python
    def sigmoid(self, x):
        """
        Computes the ouput of the sigmoid function

        :param x: input of the sigmoid, np.array of any shape
        :return: output of the sigmoid with same shape as input vector x
        """
        out = None

        out = 1/(1+np.exp(-x))

        return out
```



## 2.4损失函数

用于确定我们的模型是否足够好

```python

import os
import pickle
import numpy as np
from exercise_code.networks.linear_model import *


class Loss(object):
    def __init__(self):
        self.grad_history = []

    def forward(self, y_out, y_truth):
        return NotImplementedError

    def backward(self, y_out, y_truth, upstream_grad=1.):
        return NotImplementedError

    def __call__(self, y_out, y_truth):
        loss = self.forward(y_out, y_truth)
        grad = self.backward(y_out, y_truth)
        return (loss, grad)


class L1(Loss):

    def forward(self, y_out, y_truth):
        """
        Performs the forward pass of the L1 loss function.

        :param y_out: [N, ] array predicted value of your model.
               y_truth: [N, ] array ground truth value of your training set.
        :return: [N, ] array of L1 loss for each sample of your training set.
        """
        result = None
        result = np.abs(y_out - y_truth)
        return result

    def backward(self, y_out, y_truth):
        """
        Performs the backward pass of the L1 loss function.

        :param y_out: [N, ] array predicted value of your model.
               y_truth: [N, ] array ground truth value of your training set.
        :return: [N, ] array of L1 loss gradients w.r.t y_out for
                  each sample of your training set.
        """
        gradient = None
        gradient = y_out - y_truth

        zero_loc = np.where(gradient == 0)
        negative_loc = np.where(gradient < 0)
        positive_loc = np.where(gradient > 0)

        gradient[zero_loc] = 0
        gradient[positive_loc] = 1
        gradient[negative_loc] = -1

        return gradient


class MSE(Loss):

    def forward(self, y_out, y_truth):
        """
        Performs the forward pass of the MSE loss function.

        :param y_out: [N, ] array predicted value of your model.
                y_truth: [N, ] array ground truth value of your training set.
        :return: [N, ] array of MSE loss for each sample of your training set.
        """
        result = None
        result = (y_out - y_truth)**2
        return result

    def backward(self, y_out, y_truth):
        """
        Performs the backward pass of the MSE loss function.

        :param y_out: [N, ] array predicted value of your model.
               y_truth: [N, ] array ground truth value of your training set.
        :return: [N, ] array of MSE loss gradients w.r.t y_out for
                  each sample of your training set.
        """
        gradient = None
        gradient = 2 * (y_out - y_truth)
        return gradient


class BCE(Loss):

    def forward(self, y_out, y_truth):
        """
        Performs the forward pass of the binary cross entropy loss function.

        :param y_out: [N, ] array predicted value of your model.
                y_truth: [N, ] array ground truth value of your training set.
        :return: [N, ] array of binary cross entropy loss for each sample of your training set.
        """
        result = None

        result = -y_truth*np.log(y_out)-(1-y_truth)*np.log(1-y_out)


        return result

    def backward(self, y_out, y_truth):
        """
        Performs the backward pass of the loss function.

        :param y_out: [N, ] array predicted value of your model.
               y_truth: [N, ] array ground truth value of your training set.
        :return: [N, ] array of binary cross entropy loss gradients w.r.t y_out for
                  each sample of your training set.
        """
        gradient = None

        gradient = -y_truth/y_out +(1-y_truth)/(1-y_out)

        return gradient
    
class CrossEntropyFromLogits(Loss):
    def __init__(self):
        self.cache = {}
     
    def forward(self, y_out, y_truth, reduction='mean'):
        """
        Performs the forward pass of the cross entropy loss function.
        
        :param y_out: [N, C] array with the predicted logits of the model
            (i.e. the value before applying any activation)
        :param y_truth: [N, ] array with ground truth labels.
        
        :return: float, the cross-entropy loss value
        """
        
        # Transform the ground truth labels into one hot encodings.
        N, C = y_out.shape
        y_truth_one_hot = np.zeros_like(y_out)
        y_truth_one_hot[np.arange(N), y_truth] = 1
        
        # Transform the logits into a distribution using softmax.
        y_out_exp = np.exp(y_out - np.max(y_out, axis=1, keepdims=True))
        y_out_probs = y_out_exp / np.sum(y_out_exp, axis=1, keepdims=True)
        
        # Compute the loss for each element in the batch.
        loss = -y_truth_one_hot * np.log(y_out_probs)
        loss = loss.sum(axis=1).mean()
           
        self.cache['probs'] = y_out_probs
        
        return loss
    
    def backward(self, y_out, y_truth):
        N, C = y_out.shape
        gradient = self.cache['probs']
        gradient[np.arange(N), y_truth] -= 1
        gradient /= N
        
        return gradient
```

## 2.5前向传播，反向传播

- 前向传播的作用：
  - takes output from previous layer, performs operation, returns result
  - caches values needed for gradient computation during backprop
- 反向传播
  - takes upstream逆流的 gradient, returns all partial derivatives

- 前向传播用的2.3的模型：$\mathbf{\hat{y}}  = \sigma \left( \mathbf{X} \cdot \mathbf{w} \right)$
- 反向传播即它的导数：
  - $$\frac{\partial \hat y}{\partial w} = \frac{\partial \sigma(s)}{\partial w} = \frac{\partial \sigma(s)}{\partial s} \cdot \frac{\partial s}{\partial w}$$
  - $\frac{\partial \sigma(s)}{\partial s}=\sigma(s)*(1-\sigma(s))$
  - $\frac{\partial s}{\partial w}=X$

```python
   def forward(self, X):
        """
        Performs the forward pass of the model.

        :param X: N x D array of training data. Each row is a D-dimensional point.
        :return: Predicted labels for the data in X, shape N x 1
                 1-dimensional array of length N with classification scores.
        """
        assert self.W is not None, "weight matrix W is not initialized"
        # add a column of 1s to the data for the bias term
        batch_size, _ = X.shape
        X = np.concatenate((X, np.ones((batch_size, 1))), axis=1)
        # save the samples for the backward pass
        self.cache = X
        # output variable
        y = None

        y = self.sigmoid(np.dot(X, self.W))

        return y

    def backward(self, y):
        """
        Performs the backward pass of the model.

        :param y: N x 1 array. The output of the forward pass.
        :return: Gradient of the model output (y=sigma(X*W)) wrt W
        """
        assert self.cache is not None, "run a forward pass before the backward pass"
        dW = None


        ds = self.sigmoid(np.dot(self.cache, self.W))
        dW = self.cache * (ds * (1 - ds))


        return dW
```



## 2.6 优化器:梯度下降

- Optimizer:Gradient Descent 修改我们的参数使损失函数足够的小
  0. Initialize the weights with random values.
  1. Calculate loss with the current weights and the loss function.
  2. Calculate the gradient of the loss function w.r.t. the weights.
  3. Update weights with the corresponding gradient.
     - $ w^{(n+1)} = w^{(n)} - \alpha \cdot \frac {dL}{dw}, $
  4. Iteratively perform Step 1 to 3 until converges.

```python
class Optimizer(object):
    def __init__(self, model, learning_rate=5e-5):
        self.model = model
        self.lr = learning_rate

    def step(self, dw):
        """
        :param dw: [D+1,1] array gradient of loss w.r.t weights of your linear model
        :return weight: [D+1,1] updated weight after one step of gradient descent
        """
        weight = self.model.W

        weight = weight - self.lr*dw

        self.model.W = weight
```

## 2.7初始化权重

一个很low的随机方式来初始化权重

```python
    def initialize_weights(self, weights=None):
        """
        Initialize the weight matrix W

        :param weights: optional weights for initialization
        """
        if weights is not None:
            assert weights.shape == (self.num_features + 1, 1), \
                "weights for initialization are not in the correct shape (num_features + 1, 1)"
            self.W = weights
        else:
            self.W = 0.001 * np.random.randn(self.num_features + 1, 1)
```

## 2.8训练模型

1. 不经训练的模型

```python
from exercise_code.networks.classifier import Classifier

#initialization
model = Classifier(num_features=1)
model.initialize_weights()

y_out, _ = model(X_train)

# plot the prediction
plt.scatter(X_train, y_train)
plt.plot(X_train, y_out, color='r')
```

2. 训练后的模型更准确

```python
from exercise_code.networks.optimizer import *
from exercise_code.networks.classifier import *
# Hyperparameter Setting, we will specify the loss function we use, and implement the optimizer we finished in the last step.
num_features = 1

# initialization
model = Classifier(num_features=num_features)
model.initialize_weights()

loss_func = BCE() 
learning_rate = 5e-1
loss_history = []
opt = Optimizer(model,learning_rate)

steps = 400
# Full batch Gradient Descent
for i in range(steps):
    
    # Enable your model to store the gradient.
    model.train()
    
    # 因为Classifier的父类Networks和BCE的父类loss，都定义了__call__()这一魔法方法，所以才能像下面这样调用。
    # Compute the output and gradients w.r.t weights of your model for the input dataset.
    model_forward, model_backward = model(X_train)
    # 因为Classifier的父类Networks和BCE的父类loss，都定义了__call__()这一魔法方法，所以才能像下面这样调用。
    # Compute the loss and gradients w.r.t output of the model.
    loss, loss_grad = loss_func(model_forward, y_train)
    
    # Use back prop method to get the gradients of loss w.r.t the weights.
    grad = loss_grad * model_backward
    
    # Compute the average gradient over your batch
    grad = np.mean(grad, 0, keepdims = True)

    # After obtaining the gradients of loss with respect to the weights, we can use optimizer to
    # do gradient descent step.
    # Take transpose to have the same shape ([D+1,1]) as weights.
    opt.step(grad.T)
    
    # Average over the loss of the entire dataset and store it.
    average_loss = np.mean(loss)
    loss_history.append(average_loss)
    if i%10 == 0:
        print("Epoch ",i,"--- Average Loss: ", average_loss)

```

## 2.9可视化训练成果

```python
# Plot the loss history to see how it goes after several steps of gradient descent.
plt.plot(loss_history, label = 'Train Loss')
plt.xlabel('iteration')
plt.ylabel('training loss')
plt.title('Training Loss history')
plt.legend()
plt.show()


# forward pass
y_out, _ = model(X_train)


# plot the prediction
plt.scatter(X_train, y_train, label = 'Ground Truth')
inds = X_train.argsort(0).flatten()
plt.plot(X_train[inds], y_out[inds], color='r', label = 'Prediction')
plt.title('Prediction of our trained model')
plt.legend()
plt.show()
```

## 2.10一个通用的调用模型的方法

```python
import numpy as np
from exercise_code.networks.optimizer import Adam
from exercise_code.networks import CrossEntropyFromLogits


class Solver(object):
    """
    A Solver encapsulates all the logic necessary for training classification
    or regression models.
    The Solver performs gradient descent using the given learning rate.

    The solver accepts both training and validataion data and labels so it can
    periodically check classification accuracy on both training and validation
    data to watch out for overfitting.

    To train a model, you will first construct a Solver instance, passing the
    model, dataset, learning_rate to the constructor.
    You will then call the train() method to run the optimization
    procedure and train the model.

    After the train() method returns, model.params will contain the parameters
    that performed best on the validation set over the course of training.
    In addition, the instance variable solver.loss_history will contain a list
    of all losses encountered during training and the instance variables
    solver.train_loss_history and solver.val_loss_history will be lists
    containing the losses of the model on the training and validation set at
    each epoch.
    """

    def __init__(self, model, train_dataloader, val_dataloader,
                 loss_func=CrossEntropyFromLogits(), learning_rate=1e-3,
                 optimizer=Adam, verbose=True, print_every=1, lr_decay = 1.0,
                 **kwargs):
        """
        Construct a new Solver instance.

        Required arguments:
        - model: A model object conforming to the API described above

        - train_dataloader: A generator object returning training data
        - val_dataloader: A generator object returning validation data

        - loss_func: Loss function object.
        - learning_rate: Float, learning rate used for gradient descent.

        - optimizer: The optimizer specifying the update rule

        Optional arguments:
        - verbose: Boolean; if set to false then no output will be printed during
          training.
        - print_every: Integer; training losses will be printed every print_every
          iterations.
        """
        self.model = model
        self.learning_rate = learning_rate
        self.lr_decay = lr_decay
        self.loss_func = loss_func

        self.opt = optimizer(model, loss_func, learning_rate)

        self.verbose = verbose
        self.print_every = print_every

        self.train_dataloader = train_dataloader
        self.val_dataloader = val_dataloader
        
        self.current_patience = 0

        self._reset()

    def _reset(self):
        """
        Set up some book-keeping variables for optimization. Don't call this
        manually.
        """
        # Set up some variables for book-keeping
        self.best_model_stats = None
        self.best_params = None

        self.train_loss_history = []
        self.val_loss_history = []

        self.train_batch_loss = []
        self.val_batch_loss = []

        self.num_operation = 0
        self.current_patience = 0

    def _step(self, X, y, validation=False):
        """
        Make a single gradient update. This is called by train() and should not
        be called manually.

        :param X: batch of training features
        :param y: batch of corresponding training labels
        :param validation: Boolean indicating whether this is a training or
            validation step

        :return loss: Loss between the model prediction for X and the target
            labels y
        """
        loss = None

        # Forward pass
        y_pred = self.model.forward(X)
        # Compute loss
        loss = self.loss_func.forward(y_pred, y)
        # Add the regularization
        loss += sum(self.model.reg.values())

        # Count number of operations
        self.num_operation += self.model.num_operation

        # Perform gradient update (only in train mode)
        if not validation:
            # Compute gradients
            self.opt.backward(y_pred, y)
            # Update weights
            self.opt.step()

            # If it was a training step, we need to count operations for
            # backpropagation as well
            self.num_operation += self.model.num_operation

        return loss

    def train(self, epochs=100, patience = None):
        """
        Run optimization to train the model.
        """

        # Start an epoch
        for t in range(epochs):

            # Iterate over all training samples
            train_epoch_loss = 0.0

            for batch in self.train_dataloader:
                # Unpack data
                X = batch['image']
                y = batch['label']

                # Update the model parameters.
                validate = t == 0
                train_loss = self._step(X, y, validation=validate)

                self.train_batch_loss.append(train_loss)
                train_epoch_loss += train_loss

            train_epoch_loss /= len(self.train_dataloader)

            
            self.opt.lr *= self.lr_decay
            
            
            # Iterate over all validation samples
            val_epoch_loss = 0.0

            for batch in self.val_dataloader:
                # Unpack data
                X = batch['image']
                y = batch['label']

                # Compute Loss - no param update at validation time!
                val_loss = self._step(X, y, validation=True)
                self.val_batch_loss.append(val_loss)
                val_epoch_loss += val_loss

            val_epoch_loss /= len(self.val_dataloader)

            # Record the losses for later inspection.
            self.train_loss_history.append(train_epoch_loss)
            self.val_loss_history.append(val_epoch_loss)

            if self.verbose and t % self.print_every == 0:
                print('(Epoch %d / %d) train loss: %f; val loss: %f' % (
                    t + 1, epochs, train_epoch_loss, val_epoch_loss))

            # Keep track of the best model
            self.update_best_loss(val_epoch_loss, train_epoch_loss)
            if patience and self.current_patience >= patience:
                print("Stopping early at epoch {}!".format(t))
                break

        # At the end of training swap the best params into the model
        self.model.params = self.best_params

    def get_dataset_accuracy(self, loader):
        correct = 0
        total = 0
        for batch in loader:
            X = batch['image']
            y = batch['label']
            y_pred = self.model.forward(X)
            label_pred = np.argmax(y_pred, axis=1)
            correct += sum(label_pred == y)
            if y.shape:
                total += y.shape[0]
            else:
                total += 1
        return correct / total

    def update_best_loss(self, val_loss, train_loss):
        # Update the model and best loss if we see improvements.
        if not self.best_model_stats or val_loss < self.best_model_stats["val_loss"]:
            self.best_model_stats = {"val_loss":val_loss, "train_loss":train_loss}
            self.best_params = self.model.params
            self.current_patience = 0
        else:
            self.current_patience += 1

```

- 调用这个类去运行我们的模型

  ```python
  #We use the BCE loss
  loss = BCE()
  
  # Please use these hyperparmeter as we also use them later in the evaluation
  learning_rate = 1e-1
  epochs = 25000
  
  # Setup for the actual solver that's going to do the job of training
  # the model on the given data. set 'verbose=True' to see real time 
  # progress of the training.
  solver = Solver(model, 
                  data, 
                  loss,
                  learning_rate, 
                  verbose=True, 
                  print_every = 1000)
  # Train the model, and look at the results.
  solver.train(epochs)
  
  
  # Test final performance
  y_out, _ = model(X_test)
  
  accuracy = test_accuracy(y_out, y_test)
  print("Accuracy AFTER training {:.1f}%".format(accuracy*100))
  ```


# 3. 数据集CIFAR10分类

## 3.1加载库

```
# As usual, a bit of setup
import matplotlib.pyplot as plt
import numpy as np
import os

from exercise_code.data import (
    DataLoader,
    ImageFolderDataset,
    RescaleTransform,
    NormalizeTransform,
    FlattenTransform,
    ComposeTransform,
)
from exercise_code.networks import (
    ClassificationNet,
    CrossEntropyFromLogits
)
from exercise_code.tests.layer_tests import *
from exercise_code.tests.sgdm_tests import *

from exercise_code.solver import Solver
from exercise_code.networks.optimizer import (
    SGD,
    SGDMomentum,
    Adam
)
from exercise_code.networks.compute_network_size import *

%load_ext autoreload
%autoreload 2
%matplotlib inline

plt.rcParams['figure.figsize'] = (10.0, 8.0) # set default size of plots
plt.rcParams['image.interpolation'] = 'nearest'
plt.rcParams['image.cmap'] = 'gray'
```

## 3.2神经网络模型

### 3.2.1Modularization模块化

模块化chain rule:$\frac{\partial f}{\partial y}=\frac{\partial f}{\partial d}\cdot\frac{\partial d}{\partial y}$

- in the `forward` pass do all required computations as well as save all values that are required to compute gradients

  ```python
  def layer_forward(x, w):
    """ Receive inputs x and weights w """
    # Do some computations ...
    z = # ... some intermediate value
    # Do some more computations ...
    out = # the output
    cache = (x, w, z, out) # Values we need to compute gradients
    return out, cache
  ```

- in the `backward` function they will use the incoming gradients from later building blocks, to compute their respective各自的 gradients using their cached values.

  ```python
  def layer_backward(dout, cache):
    """
    Receive derivative of loss with respect to outputs and cache,
    and compute derivative with respect to inputs.
    """
    # Unpack cache values
    x, w, z, out = cache
    # Use values in cache to compute derivatives
    dx = # Derivative of loss with respect to x
    dw = # Derivative of loss with respect to w
    return dx, dw
  ```

- We can just simply chain an arbitrary amount of such blocks, so called `layers`

### 3.2.2Layer Non-Linearities(Sigmoid,Relu,Tanh,L-Relu)

1. **Sigmoid**

   Sigmoid is one of the oldest used non-linearities. 

   - 公式：$\sigma(x)=\frac{1}{1+exp(-x)}$

   - Sigmoid的导数：$\frac{\partial\sigma(x)}{\partial x}=\sigma(x)(1-\sigma(x))$

   - 反向求导：
     $$
     \frac{\partial L}{\partial x}=\frac{\partial L}{\partial \sigma(x)}\cdot\frac{\partial\sigma(x)}{\partial x}
     $$

     - $\frac{\partial L}{\partial \sigma(x)}$对应下面代码backward中的`dout`
     - $\frac{\partial\sigma(x)}{\partial x}$对应下面代码backward中的`sd`

   ```python
   class Sigmoid:
       def __init__(self):
           pass
   
       def forward(self, x):
           """
           :param x: Inputs, of any shape.
   
           :return out: Outputs, of the same shape as x.
           :return cache: Cache, stored for backward computation, of the same shape as x.
           """
           shape = x.shape
           out, cache = np.zeros(shape), np.zeros(shape)
   
           #out用于计算前向传播，cache用于计算反向传播
           out = 1/(1+np.exp(-x))
           cache = out
   
           return out, cache
   
       def backward(self, dout, cache):
           """
           :param dout: Upstream gradient from the computational graph, from the Loss function
                       and up to this layer. Has the shape of the output of forward().
           :param cache: The values that were stored during forward() to the memory,
                       to be used during the backpropogation.
           :return: dx: the gradient w.r.t. input X, of the same shape as X
           """
           dx = None
   
           sd = cache*(1-cache)
           dx = dout*sd
   
           return dx
   ```

2. **Relu**

   Rectified Linear Units线性整流单元 are the currently most used non-linearities in deep learning

   - 公式:$ReLU(x) = max(0,x)=\begin{cases}x(x>0)\\0(x\leq0) \end{cases}$

     - ReLU的导数：$\frac{\partial ReLU(x)}{\partial x}=\begin{cases}1(x>0)\\0(x\leq0) \end{cases}$

   - 反向求导:
     $$
     \frac{\partial L}{\partial x}=\frac{\partial L}{\partial ReLU(x)}\cdot\frac{\partial ReLU(x)}{\partial x}
     $$

     - $\frac{\partial L}{\partial ReLU(x)}$对应下面代码backward中的dout
     - $\frac{\partial ReLU(x)}{\partial x}$如果>0值为1就是传dout本身，所以只需要让<=0的dout值为0即可。

   ```python
   class Relu:
       def __init__(self):
           pass
   
       def forward(self, x):
           """
           :param x: Inputs, of any shape.
   
           :return outputs: Outputs, of the same shape as x.
           :return cache: Cache, stored for backward computation, of the same shape as x.
           """
           out = None
           cache = None
   		#out用于计算前向传播，cache用于计算反向传播
           out = np.maximum(0,x)
           cache = out
   
           return out, cache
   
       def backward(self, dout, cache):
           """
           :param dout: Upstream gradient from the computational graph, from the Loss function
                       and up to this layer. Has the shape of the output of forward().
           :param cache: The values that were stored during forward() to the memory,
                       to be used during the backpropogation.
           :return: dx: the gradient w.r.t. input X, of the same shape as X
           """
           dx = None
   
           dx = dout
           # dx,cache的维度一样，所以可以直接下面这样获得cache=0的索引
           dx[cache<=0]=0
   
           return dx
   ```

3. **Tanh**

   ```python
   class Tanh:
       def __init__(self):
           pass
   
       def forward(self, x):
           """
           :param x: Inputs, of any shape
   
           :return out: Output, of the same shape as x
           :return cache: Cache, for backward computation, of the same shape as x
           """
           outputs = None
           cache = None
           ########################################################################
           # TODO:                                                                #
           # Implement the forward pass of Tanh activation function               #
   
           outputs = np.tanh(x)
           cache = outputs
    ########################################################################
           #                           END OF YOUR CODE                           #
   
           return outputs, cache
   
       def backward(self, dout, cache):
           """
           :return: dx: the gradient w.r.t. input X, of the same shape as X
           """
           dx = None
           ########################################################################
           # TODO:                                                                #
           # Implement the backward pass of Tanh activation function              #
   
           dx = dout * (1 - np.square(cache))
           ########################################################################
   
           return dx
   ```

4. **LeakyRelu**

   ```python
   class LeakyRelu:
       def __init__(self, slope=0.01):
           self.slope = slope
   
       def forward(self, x):
           """
           :param x: Inputs, of any shape
   
           :return out: Output, of the same shape as x
           :return cache: Cache, for backward computation, of the same shape as x
           """
           outputs = None
           cache = None
           ########################################################################
           # TODO:                                                                #
           # Implement the forward pass of LeakyRelu activation function          #
   
           outputs = np.maximum(self.slope * x, x)
           cache = x
   
           ########################################################################
           #                           END OF YOUR CODE                          #
           return outputs, cache
   
       def backward(self, dout, cache):
           """
           :return: dx: the gradient w.r.t. input X, of the same shape as X
           """
           dx = None
           ########################################################################
           # TODO:                                                                #
           # Implement the backward pass of LeakyRelu activation function         #
   
   
           dx = dout * np.where(cache > 0, 1, self.slope)
   
           ########################################################################
           #                           END OF YOUR CODE                           #
   
           return dx
   ```

   

### 3.2.3 Affine layer仿射层

 Affine layer仿射层又称之为linear 线性变换层， Full-connected Layer全连接层。仿射（Affine）的意思是前面一层中的每一个神经元都连接到当前层中的每一个神经元。仿射层通常被加在卷积神经网络或循环神经网络做出最终预测前的输出的顶层。

**全连接层之前的作用是提取特征，而全连接层的作用是分类**

- 公式:$z=Wx+b$

  - W是权重矩阵

- 反向求导：
  $$
  \begin{aligned}
  &\frac{\partial L}{\partial x}=\frac{\partial L}{\partial z}\cdot W^T\\
  &\frac{\partial L}{\partial W}=x^T\cdot \frac{\partial L}{\partial z}\\
  &\frac{\partial L}{\partial b}=1^T\cdot\frac{\partial L}{\partial z}
  \end{aligned}
  $$

  - $\frac{\partial L}{\partial z}$:对应下面代码的dout
  - $\frac{\partial L}{\partial b}$:是$\frac{\partial L}{\partial z}$即dout的求和
  - $\frac{\partial L}{\partial x},\frac{\partial L}{\partial W}$分别对应下面代码的dx,dw

```python
def affine_forward(x, w, b):
    """
    Computes the forward pass for an affine (fully-connected) layer.
    The input x has shape (N, d_1, ..., d_k) and contains a minibatch of N
    examples, where each example x[i] has shape (d_1, ..., d_k). We will
    reshape each input into a vector of dimension D = d_1 * ... * d_k, and
    then transform it to an output vector of dimension M.
    Inputs:
    :param x: A numpy array containing input data, of shape (N, d_1, ..., d_k)
    :param w: A numpy array of weights, of shape (D, M)
    :param b: A numpy array of biases, of shape (M,)
    :return out: output, of shape (N, M)
    :return cache: (x, w, b)
    """
    N, M = x.shape[0], b.shape[0]
    out = np.zeros((N,M))

    #N:每一个minibatch下的样本数
    N=x.shape[0]
    x_shape=x.shape[1:]
    D=1
    for i in range(len(x_shape)):
        D=D*x_shape[i]
    # 将输入x的数据排列为(N,D)的x_new
    x_new = np.reshape(x, (N, D))
    # 前向输出
    out = np.dot(x_new, w) + b

    cache = (x, w, b)
    return out, cache


def affine_backward(dout, cache):
    """
    Computes the backward pass for an affine layer.
    Inputs:
    :param dout: Upstream derivative, of shape (N, M)
    :param cache: Tuple of:
      - x: Input data, of shape (N, d_1, ... d_k)
      - w: Weights, of shape (D, M)
      - b: A numpy array of biases, of shape (M,
    :return dx: Gradient with respect to x, of shape (N, d1, ..., d_k)
    :return dw: Gradient with respect to w, of shape (D, M)
    :return db: Gradient with respect to b, of shape (M,)
    """
    x, w, b = cache
    dx, dw, db = None, None, None
	# 将输入x的数据排列为(N,D)的x_new
    N = x.shape[0]
    D_backup = x.shape[1:]
    D = 1
    for i in range(len(D_backup)):
        D = D * D_backup[i]
    x_new = np.reshape(x, (N, D))
	# 反向传播计算
    dw = np.dot(x_new.T, dout)
    db = np.sum(dout, axis=0)
    dx_backup = np.dot(dout, w.T)
    dx = np.reshape(dx_backup, x.shape)

    return dx, dw, db
```

### 3.2.4 N-layer Classification Network

- 用Relu计算一个2层网络,

  ```python
  # Define a dummy input
  test_input = np.random.randn(1, 10)   # (batch_size, input_size)
  
  # Define a test model
  test_model = ClassificationNet(input_size=10, 
                                 hidden_size=128,
                                 activation=Relu(), 
                                 num_layer=2, 
                                 num_classes=3)
  
  # Compute output
  model_output = test_model.forward(test_input)
  ```

  - `ClassificationNet`类的定义如下

- 将上面这些层的前向，反向传播整合在一个类里，就可以想建几层网络建几层，想用什么激活函数用什么激活函数

  ```python
  class ClassificationNet(Network):
      """
      A fully-connected classification neural network with configurable 
      activation function, number of layers, number of classes, hidden size and
      regularization strength. 
      """
  
      def __init__(self, activation=Relu(), num_layer=2,
                   input_size=3 * 32 * 32, hidden_size=100,
                   std=1e-3, num_classes=10, reg=0, **kwargs):
          """
          :param activation: choice of activation function. It should implement
              a forward() and a backward() method.
          :param num_layer: integer, number of layers. 
          :param input_size: integer, the dimension D of the input data.
          :param hidden_size: integer, the number of neurons H in the hidden layer.
          :param std: float, standard deviation used for weight initialization.
          :param num_classes: integer, number of classes.
          :param reg: float, regularization strength.
          """
          super().__init__("cifar10_classification_net")
  
          self.activation = activation
          self.reg_strength = reg
  
          self.cache = None
  
          # Initialize random gaussian weights for all layers and zero bias
          self.num_layer = num_layer
          self.params = {'W1': std * np.random.randn(input_size, hidden_size),
                         'b1': np.zeros(hidden_size)}
  
          for i in range(num_layer - 2):
              self.params['W' + str(i + 2)] = std * np.random.randn(hidden_size,
                                                                    hidden_size)
              self.params['b' + str(i + 2)] = np.zeros(hidden_size)
  
          self.params['W' + str(num_layer)] = std * np.random.randn(hidden_size,
                                                                    num_classes)
          self.params['b' + str(num_layer)] = np.zeros(num_classes)
  
          self.grads = {}
          self.reg = {}
          for i in range(num_layer):
              self.grads['W' + str(i + 1)] = 0.0
              self.grads['b' + str(i + 1)] = 0.0
  
      def forward(self, X):
          """
          Performs the forward pass of the model.
          :param X: Input data of shape (N, d_1, ..., d_k) and contains a minibatch of N
          examples, where each example x[i] has shape (d_1, ..., d_k)
          :return: Predicted value for the data in X, shape N x num_classes
                   num_classes-dimensional array of length N with the classification scores.
          """
  
          self.cache = {}
          self.reg = {}
          X = X.reshape(X.shape[0], -1)
          # Unpack variables from the params dictionary
          for i in range(self.num_layer - 1):
              W, b = self.params['W' + str(i + 1)], self.params['b' + str(i + 1)]
  
              # Forward i_th layer
              X, cache_affine = affine_forward(X, W, b)
              self.cache["affine" + str(i + 1)] = cache_affine
  
              # Activation function
              X, cache_sigmoid = self.activation.forward(X)
              self.cache["sigmoid" + str(i + 1)] = cache_sigmoid
  
              # Store the reg for the current W
              self.reg['W' + str(i + 1)] = np.sum(W ** 2) * self.reg_strength
  
          # last layer contains no activation functions
          W, b = self.params['W' + str(self.num_layer)],\
                 self.params['b' + str(self.num_layer)]
          y, cache_affine = affine_forward(X, W, b)
          self.cache["affine" + str(self.num_layer)] = cache_affine
          self.reg['W' + str(self.num_layer)] = np.sum(W ** 2) * self.reg_strength
  
          return y
  
      def backward(self, dy):
          """
          Performs the backward pass of the model.
          :param dy: N x num_classes array. The gradient wrt the output of the network.
          :return: Gradients of the model output wrt the model weights
          """
  
          # Note that last layer has no activation
          cache_affine = self.cache['affine' + str(self.num_layer)]
          dh, dW, db = affine_backward(dy, cache_affine)
          self.grads['W' + str(self.num_layer)] = \
              dW + 2 * self.reg_strength * self.params['W' + str(self.num_layer)]
          self.grads['b' + str(self.num_layer)] = db
  
          # The rest sandwich layers
          for i in range(self.num_layer - 2, -1, -1):
              # Unpack cache
              cache_sigmoid = self.cache['sigmoid' + str(i + 1)]
              cache_affine = self.cache['affine' + str(i + 1)]
  
              # Activation backward
              dh = self.activation.backward(dh, cache_sigmoid)
  
              # Affine backward
              dh, dW, db = affine_backward(dh, cache_affine)
  
              # Refresh the gradients
              self.grads['W' + str(i + 1)] = dW + 2 * self.reg_strength * \
                                             self.params['W' + str(i + 1)]
              self.grads['b' + str(i + 1)] = db
  
          return self.grads
  
      def save_model(self):
          directory = 'models'
          model = {self.model_name: self}
          if not os.path.exists(directory):
              os.makedirs(directory)
          pickle.dump(model, open(directory + '/' + self.model_name + '.p', 'wb'))
  
      def get_dataset_prediction(self, loader):
          scores = []
          labels = []
  
          for batch in loader:
              X = batch['image']
              y = batch['label']
              score = self.forward(X)
              scores.append(score)
              labels.append(y)
  
          scores = np.concatenate(scores, axis=0)
          labels = np.concatenate(labels, axis=0)
  
          preds = scores.argmax(axis=1)
          acc = (labels == preds).mean()
  
          return labels, preds, acc
  ```

## 3.3加载CIFAR10 Dataset

- 下载数据

    ```python
    # Define output path similar to exercise 3
    i2dl_exercises_path = os.path.dirname(os.path.abspath(os.getcwd()))
    cifar_root = os.path.join(i2dl_exercises_path, "datasets", "cifar10")

    # Dictionary so that we can convert label indices to actual label names
    classes = [
        'plane', 'car', 'bird', 'cat', 'deer',
        'dog', 'frog', 'horse', 'ship', 'truck',
    ]

    # Simply call dataset class
    dataset = ImageFolderDataset(
            root=cifar_root
        )
    ```

- 这个project中我们的输入是一维的，所以要把数据集展成1维

  ```python
  # Use the Cifar10 mean and standard deviation computed in Exercise 3.
  cifar_mean = np.array([0.49191375, 0.48235852, 0.44673872])
  cifar_std  = np.array([0.24706447, 0.24346213, 0.26147554])
  
  # Define all the transforms we will apply on the images when 
  # retrieving them.
  rescale_transform = RescaleTransform()
  normalize_transform = NormalizeTransform(
      mean=cifar_mean,
      std=cifar_std
  )
  
  # Add the new flatten transform
  flatten_transform = FlattenTransform()
  
  # And string them together
  compose_transform = ComposeTransform([
      rescale_transform, 
      normalize_transform,
      flatten_transform
  ])
  ```
  
- 加载数据

  具体加载代码可看1.4/1.5

  ```python
  from exercise_code.data import MemoryImageFolderDataset
  #DATASET = ImageFolderDataset
  DATASET = MemoryImageFolderDataset
  # Create a dataset and dataloader
  batch_size = 8
  
  dataset = DATASET(
      mode='train',
      root=cifar_root,
      transform=compose_transform,
      split={'train': 0.01, 'val': 0.2, 'test': 0.79}
  )
      
  dataloader = DataLoader(
      dataset=dataset,
      batch_size=batch_size,
      shuffle=True,
      drop_last=True,
  )
  
  print('Dataset size:', len(dataset))
  print('Dataloader size:', len(dataloader))
  ```

## 3.4Cross-Entropy Loss交叉熵

- Cross-Entropy Los模型：

  $$ CE(\hat{y}, y) = \frac{1}{N} \sum_{i=1}^N \sum_{k=1}^{C} \Big[ -y_{ik} \log(\hat{y}_{ik}) \Big] $$

  - $ N $ is again the number of samples
  - $ C $ is the number of classes
  - $ \hat{y}_{ik} $ is the probability that the model assigns for the $k$'th class when the $i$'th sample is the input. 
  - $y_{ik} = 1 $ iff the true label of the $i$th sample is $k$ and 0 otherwise. This is called a [one-hot encoding](https://machinelearningmastery.com/why-one-hot-encode-data-in-machine-learning/).

- [Logits](https://datascience.stackexchange.com/questions/31041/what-does-logits-in-machine-learning-mean):

  **Logits interpreted to be the unnormalised** (or not-yet normalised) **predictions** (or outputs) **of a model. These can give results, but we don't normally stop with logits, because interpreting their raw values is not easy.**

  比如我们分类2张猫狗图，得到的Logits value分别为：

  1. 第一张图猫为16，狗为0.7，结果判断为猫
  2. 第二张图猫为1.004，狗为0.6，结果判断为猫

  模型都成功了，但我们要想比较这两张图的判断结果，必须要先normalise the logits。为此我们使用softmax function ,它可以将每张图预测结果和为1，并可认为他们是概率。

  第一张图经过normalise（softmax）后，猫的值为0.9999，狗的值为0.0001。所以我们认为这张图99.99%的概率是猫

- Softmax

  - 公式：

    $$softmax(x)=\sigma(x)=\frac{e^{x_i}}{\sum_{i=1}^ne^{x_i}}$$

    - $x=(x_i)_{(1\leq i\leq n)}\in\mathbb{R}^n$是一个vector

  - 问题：数值不稳定Numerical Stability

    计算机中用浮点数来表示实数时总会引入近似误差

    1. **下溢**：当接近零的数被四舍五入为零时发生下溢（除零，对0取对数）
    2. **上溢**：大量级的数被近似为无穷时发生上溢。

  - 解决方法：$$\sigma(x-\max_{1\leq i\leq n}x_i)$$

    - 减去最大值导致exp最大为0，排除了上溢的可能性
    - 分母中至少有一个值为1的项（exp(0)=1），从而也排除了因分母=下溢导致被零除的可能性

- 代码实现：

  ```python
  class CrossEntropyFromLogits(Loss):
      def __init__(self):
          self.cache = {}
       
      def forward(self, y_out, y_truth, reduction='mean'):
          """
          Performs the forward pass of the cross entropy loss function.
          
          :param y_out: [N, C] array with the predicted logits of the model
              (i.e. the value before applying any activation)
          :param y_truth: (N,) array with ground truth labels.
          :return: float, the cross-entropy loss value
          """
          # Transform the ground truth labels into one hot encodings.
          N, C = y_out.shape
          ## Return an array of zeros with the same shape and type as a given array.
          y_truth_one_hot = np.zeros_like(y_out)
          y_truth_one_hot[np.arange(N), y_truth] = 1
          
          # Transform the logits into a distribution using softmax.
          # First make the operation numerically stable by substracting the
          # biggest element from all entries before applying exp
          y_out_exp = np.exp(y_out - np.max(y_out, axis=1, keepdims=True))
          y_out_probs = y_out_exp / np.sum(y_out_exp, axis=1, keepdims=True)
          
          # Compute the loss for each element in the batch.
          # Hint: Why do we add the minus sign to the formula?
          loss = -y_truth_one_hot * np.log(y_out_probs)
          loss = loss.sum(axis=1).mean()
             
          self.cache['probs'] = y_out_probs
          
          return loss
      
      def backward(self, y_out, y_truth):
          """
          Performs the backward pass of the loss function.
  
          :param y_out: [N, C] array predicted value of your model.
                 y_truth: [N, ] array ground truth value of your training set.
          :return: [N, C] array of cross entropy loss gradients w.r.t y_out for
                    each sample of your training set.
          """
          N, C = y_out.shape
          gradient = self.cache['probs']
          gradient[np.arange(N), y_truth] -= 1
          gradient /= N
          
          return gradient
  ```

## 3.5Optimization优化算法

- 注意事项

  Always, always, always when starting a new project or defining a new network: **overfit on a small set first and then generalize**. The 500 images we are using here are already too many sample for most cases. Start with a single sample, then 10 and finally a few hundred. Don't cheap out on this step! More often, your network will fail to generalize properly and you have to first know if it has enough capacity to overfit and that the full training pipeline is working!

- 下面每一点都要先如3.2.4中一样设置我们的模型

### 3.5.1 计算神经网络所需要的内存

经计算计算一个batchsize为8的子集，前向/反向传播分别要用到3M和6M内存。

但我们共有50000个子集，这就需要36.81G的内存。可见SGD重要性

- 计算前向传播需要的内存

  ```python
  # Set up loss
  loss_func = CrossEntropyFromLogits()
  
  # Get a random batch of our dataloader with batch_size 8
  sample_batch = iter(dataloader).__next__()
  sample_images = sample_batch['image']
  sample_labels = sample_batch['label']
  
  # Compute model output
  model_output = model.forward(sample_images)
  
  num_bytes = compute_network_pass_size(model)
  print('\nTotal number of bytes used by network for batch:', GetHumanReadable(num_bytes))
  ```

- 计算反向传播需要的内存

  ```python
  # 1. Compute loss
  _ = loss_func.forward(model_output, sample_labels)
  # 2. Compute loss gradients
  dout = loss_func.backward(model_output, sample_labels)
  # 3. Backpropagate gradients through model
  _ = model.backward(dout)
  
  # Now calculate bytes again
  num_bytes = compute_network_pass_size(model)
  
  print('\nTotal number of bytes used by network for batch:', GetHumanReadable(num_bytes))
  ```

- 用到的`compute_network_pass_size`类

  ```python
  def compute_network_pass_size(model):
      """Computes the size of a network pass in bytes using cached
      parameters as well as gradients"""
      num_bytes = 0
  
      print('Adding layer caches for forward pass:')
      for layer in model.cache.keys():
          # Add size of forward caches
          key_num_bytes = 0
          for value in model.cache[layer]:
              value_num_bytes = sys.getsizeof(value)
              key_num_bytes += value_num_bytes
          num_bytes += key_num_bytes
  
          print(layer, key_num_bytes)
  
      print('\nAdding layer gradients for backward pass:')
      for key in model.grads.keys():
          # Add size of backward gradients
          key_num_bytes = sys.getsizeof(model.grads[key])
          num_bytes += key_num_bytes
          print(key, key_num_bytes)
         
      return num_bytes
  ```

### 3.5.2 SGD

- 使用2.10所写的solver直接调用 SGD优化算法

  ```python
  learning_rate = 1e-2
  
  # We use our training dataloader for validation as well as testing
  solver = Solver(model, dataloader, dataloader, 
                  learning_rate=learning_rate, loss_func=loss_func, optimizer=SGD)
  
  # This might take a while depending on your hardware. When in doubt: use google colab
  solver.train(epochs=20)
  ```

- SGD

  ```python
  class SGD(object):
      def __init__(self, model, loss_func, learning_rate=1e-4):
          self.model = model
          self.loss_func = loss_func
          self.lr = learning_rate
          self.grads = None
  
      def backward(self, y_pred, y_true):
          """
          Compute the gradients wrt the weights of your model
          """
          dout = self.loss_func.backward(y_pred, y_true)
          self.model.backward(dout)
  
      def _update(self, w, dw, lr):
          """
          Update a model parameter
          """
          w -= lr * dw
          return w
  
      def step(self):
          """
          Perform an update step with the update function, using the current
          gradients of the model
          """
  
          # Iterate over all parameters
          for name in self.model.grads.keys():
  
              # Unpack parameter and gradient
              w = self.model.params[name]
              dw = self.model.grads[name]
  
              # Update the parameter
              w_updated = self._update(w, dw, lr=self.lr)
              self.model.params[name] = w_updated
  
              # Reset gradient
              self.model.grads[name] = 0.0
  ```

### 3.5.3 SGD+Momentum

- 可以直接用2.10的olver调用

- SGD+Momentum

  ```python
  class SGDMomentum(object):
      """
      Performs stochastic gradient descent with momentum.
  
      config format:
      - momentum: Scalar between 0 and 1 giving the momentum value.
        Setting momentum = 0 reduces to sgd.
      - velocity: A numpy array of the same shape as w and dw used to store a moving
        average of the gradients.
      """
      def __init__(self, model, loss_func, learning_rate=1e-4, **kwargs):
          self.model = model
          self.loss_func = loss_func
          self.lr = learning_rate
          self.grads = None
          self.optim_config = kwargs.pop('optim_config', {})
          self._reset()
  
      def _reset(self):
          self.optim_configs = {}
          for p in self.model.params:
              d = {k: v for k, v in self.optim_configs.items()}
              self.optim_configs[p] = d
  
      def backward(self, y_pred, y_true):
          """
          Compute the gradients wrt the weights of your model
          """
          dout = self.loss_func.backward(y_pred, y_true)
          self.model.backward(dout)
  
      def _update(self, w, dw, config, lr):
          """
          Update a model parameter
          
          :param w: Current weight matrix
          :param dw: The corresponding calculated gradient, of the same shape as w.
          :param config: A dictionary, containing relevant parameters, such as the "momentum" value. Check it out.
          :param lr: The value of the "learning rate".
  
          :return next_w: The updated value of w.
          :return config: The same dictionary. Might needed to be updated.
          """
          if config is None:
              config = {}
          config.setdefault('momentum', 0.9)
          v = config.get('velocity', np.zeros_like(w))
          next_w = None
  
          ########################################################################
          # TODO: Implement the momentum update formula. Store the updated       #  
          # value in the next_w variable. You should also use and update the     #
          # velocity v.                                                          #
          ########################################################################
  
          mu = config['momentum']
          learning_rate = lr
          v = mu * v - learning_rate * dw
          next_w = w + v
          config['velocity'] = v
  
          ########################################################################
  
          config['velocity'] = v
  
          return next_w, config
  
      def step(self):
          """
          Perform an update step with the update function, using the current
          gradients of the model
          """
  
          # Iterate over all parameters
          for name in self.model.grads.keys():
  
              # Unpack parameter and gradient
              w = self.model.params[name]
              dw = self.model.grads[name]
  
              config = self.optim_configs[name]
  
              # Update the parameter
              w_updated, config = self._update(w, dw, config, lr=self.lr)
              self.model.params[name] = w_updated
              self.optim_configs[name] = config
              # Reset gradient
              self.model.grads[name] = 0.0
  ```

### 3.5.4 Adam

- Adam

  ```python
  class Adam(object):
      """
      Uses the Adam update rule, which incorporates moving averages of both the
      gradient and its square and a bias correction term.
  
      config format:
      - beta1: Decay rate for moving average of first moment of gradient.
      - beta2: Decay rate for moving average of second moment of gradient.
      - epsilon: Small scalar used for smoothing to avoid dividing by zero.
      - m: Moving average of gradient.
      - v: Moving average of squared gradient.
      - t: Iteration number.
      """
      def __init__(self, model, loss_func, learning_rate=1e-4, **kwargs):
          self.model = model
          self.loss_func = loss_func
          self.lr = learning_rate
          self.grads = None
  
          self.optim_config = kwargs.pop('optim_config', {})
  
          self._reset()
  
      def _reset(self):
          self.optim_configs = {}
          for p in self.model.params:
              d = {k: v for k, v in self.optim_configs.items()}
              self.optim_configs[p] = d
  
      def backward(self, y_pred, y_true):
          """
          Compute the gradients wrt the weights of your model
          """
          dout = self.loss_func.backward(y_pred, y_true)
          self.model.backward(dout)
  
      def _update(self, w, dw, config, lr):
          """
          Update a model parameter
          
          :param w: Current weight matrix
          :param dw: The corresponding calculated gradient, of the same shape as w.
          :param config: A dictionary, containing relevant parameters, such as the "beta1" value. Check it out.
          :param lr: The value of the "learning rate".
  
          :return next_w: The updated value of w.
          :return config: The same dictionary. Might needed to be updated.
          """
          if config is None:
              config = {}
          config.setdefault('beta1', 0.9)
          config.setdefault('beta2', 0.999)
          config.setdefault('epsilon', 1e-4)
          config.setdefault('m', np.zeros_like(w))
          config.setdefault('v', np.zeros_like(w))
          config.setdefault('t', 0)
          next_w = None
  
          #########################################################################
          # TODO: Look at the Adam implementation.                                #
          #########################################################################
          m = config['m']
          v = config['v']
          t = config['t']
          beta1 = config['beta1']
          beta2 = config['beta2']
          learning_rate = lr
          eps = config['epsilon']
  
          m = beta1 * m + (1 - beta1) * dw
          m_hat = m / (1 - np.power(beta1, t + 1))
          v = beta2 * v + (1 - beta2) * (dw ** 2)
          v_hat = v / (1 - np.power(beta2, t + 1))
          next_w = w - learning_rate * m_hat / (np.sqrt(v_hat) + eps)
  
          config['t'] = t + 1
          config['m'] = m
          config['v'] = v
          ########################################################################
  
  
          return next_w, config
  
      def step(self):
          """
          Perform an update step with the update function, using the current
          gradients of the model
          """
  
          # Iterate over all parameters
          for name in self.model.grads.keys():
  
              # Unpack parameter and gradient
              w = self.model.params[name]
              dw = self.model.grads[name]
  
              config = self.optim_configs[name]
  
              # Update the parameter
              w_updated, config = self._update(w, dw, config, lr=self.lr)
              self.model.params[name] = w_updated
              self.optim_configs[name] = config
              # Reset gradient
              self.model.grads[name] = 0.0
  ```


# 4. Cifar10分类：调参

## 4.1加载库

```py
# Some lengthy setup.
import matplotlib.pyplot as plt
import numpy as np
import os
import urllib.request

%load_ext autoreload
%autoreload 2
%matplotlib inline

plt.rcParams['figure.figsize'] = (10.0, 8.0) # set default size of plots
plt.rcParams['image.interpolation'] = 'nearest'
plt.rcParams['image.cmap'] = 'gray'
```

## 4.2数据预处理

- Rescale调节尺寸，Normalize正则化，Flatten转为1维。来达到我们希望的数据shape

```python
class RescaleTransform:
    """Transform class to rescale images to a given range"""
    def __init__(self, range_=(0, 1), old_range=(0, 255)):
        """
        :param range_: Value range to which images should be rescaled
        :param old_range: Old value range of the images
            e.g. (0, 255) for images with raw pixel values
        """
        self.min = range_[0]
        self.max = range_[1]
        self._data_min = old_range[0]
        self._data_max = old_range[1]

    def __call__(self, images):

        images = images - self._data_min  # normalize to (0, data_max-data_min)
        images /= (self._data_max - self._data_min)  # normalize to (0, 1)
        images *= (self.max - self.min)  # norm to (0, target_max-target_min)
        images += self.min  # normalize to (target_min, target_max)
        
        return images


class NormalizeTransform:
    """
    Transform class to normalize images using mean and std
    Functionality depends on the mean and std provided in __init__():
        - if mean and std are single values, normalize the entire image
        - if mean and std are numpy arrays of size C for C image channels,
            then normalize each image channel separately
    """
    def __init__(self, mean, std):
        """
        :param mean: mean of images to be normalized
            can be a single value, or a numpy array of size C
        :param std: standard deviation of images to be normalized
             can be a single value or a numpy array of size C
        """
        self.mean = mean
        self.std = std

    def __call__(self, images):
        images = (images - self.mean) / self.std
        return images

    
class FlattenTransform:
    """Transform class that reshapes an image into a 1D array"""
    def __call__(self, image):
        return image.flatten()

## 将上面不同模式的数据预处理方式统一
class ComposeTransform:
    """Transform class that combines multiple other transforms into one"""
    def __init__(self, transforms):
        """
        :param transforms: transforms to be combined
        """
        self.transforms = transforms

    def __call__(self, images):
        for transform in self.transforms:
            images = transform(images)
        return images

```

- 加载数据

  ```python
  # Choose your preferred dataset here
  #DATASET = ImageFolderDataset
  DATASET = MemoryImageFolderDataset
  
  # Use the Cifar10 mean and standard deviation computed in Exercise 3.
  cifar_mean = np.array([0.49191375, 0.48235852, 0.44673872])
  cifar_std  = np.array([0.24706447, 0.24346213, 0.26147554])
  
  # Define all the transforms we will apply on the images when 
  # retrieving them.
  rescale_transform = RescaleTransform()
  normalize_transform = NormalizeTransform(
      mean=cifar_mean,
      std=cifar_std
  )
  flatten_transform = FlattenTransform()
  compose_transform = ComposeTransform([rescale_transform, 
                                        normalize_transform,
                                        flatten_transform])
  
  # Create a train, validation and test dataset.
  datasets = {}
  for mode in ['train', 'val', 'test']:
      crt_dataset = DATASET(
          mode=mode,
          root=cifar_root, 
          transform=compose_transform,
          split={'train': 0.6, 'val': 0.2, 'test': 0.2}
      )
      datasets[mode] = crt_dataset
      
  # Create a dataloader for each split.
  dataloaders = {}
  for mode in ['train', 'val', 'test']:
      crt_dataloader = DataLoader(
          dataset=datasets[mode],
          batch_size=256,
          shuffle=True,
          drop_last=True,
      )
      dataloaders[mode] = crt_dataloader
  ```

- 对于训练集trianing set,我们可以**flip the images horizontally** or **blur the images**，来扩充我们的训练集。（不可用于validation and test data）

  ```python
  class RandomHorizontalFlip:
      """
      Transform class that flips an image horizontically randomly with a given probability.
      """
  
      def __init__(self, prob=0.5):
          """
          :param prob: Probability of the image being flipped
          """
          self.p = prob
  
      def __call__(self, image):
          rand = random.uniform(0,1)
          if rand < self.p:
              image = np.flip(image,1)
          return image
      
     
  #Load the data in a dataset without any transformation 
  dataset = DATASET(
          mode=mode,
          root=cifar_root, 
          download_url=download_url,
          split={'train': 0.6, 'val': 0.2, 'test': 0.2},
      )
  
  #Retrieve an image from the dataset and flip it
  image = dataset[1]['image']
  transform = RandomHorizontalFlip(1)
  image_flipped = transform(image)
  ```

- 参见3.2.4的规范方法ClassificationNet，帮助我们方便调参

  ```python
  input_size = datasets['train'][0]['image'].shape[0]
  model = ClassificationNet(input_size=input_size, 
                            hidden_size=512)
                            
  num_layer = 2
  reg = 0.1
  model = ClassificationNet(activation=Sigmoid(), 
                            num_layer=num_layer, 
                            reg=reg,
                            num_classes=10)
  ```

## 4.3 一个分类模型结构

用2.10的`solver`来调用这个model，默认是2层的

```python
class ClassificationNet(Network):
    """
    A fully-connected classification neural network with configurable 
    activation function, number of layers, number of classes, hidden size and
    regularization strength. 
    """

    def __init__(self, activation=Sigmoid(), num_layer=2,
                 input_size=3 * 32 * 32, hidden_size=100,
                 std=1e-3, num_classes=10, reg=0, **kwargs):
        """
        :param activation: choice of activation function. It should implement
            a forward() and a backward() method.
        :param num_layer: integer, number of layers. 
        :param input_size: integer, the dimension D of the input data.
        :param hidden_size: integer, the number of neurons H in the hidden layer.
        :param std: float, standard deviation used for weight initialization.
        :param num_classes: integer, number of classes.
        :param reg: float, regularization strength.
        """
        super(ClassificationNet, self).__init__("cifar10_classification_net")

        self.activation = activation
        self.reg_strength = reg

        self.cache = None

        self.memory = 0
        self.memory_forward = 0
        self.memory_backward = 0
        self.num_operation = 0

        # Initialize random gaussian weights for all layers and zero bias
        self.num_layer = num_layer
        self.params = {'W1': std * np.random.randn(input_size, hidden_size),
                       'b1': np.zeros(hidden_size)}

        for i in range(num_layer - 2):
            self.params['W' + str(i + 2)] = std * np.random.randn(hidden_size,
                                                                  hidden_size)
            self.params['b' + str(i + 2)] = np.zeros(hidden_size)

        self.params['W' + str(num_layer)] = std * np.random.randn(hidden_size,
                                                                  num_classes)
        self.params['b' + str(num_layer)] = np.zeros(num_classes)

        self.grads = {}
        self.reg = {}
        for i in range(num_layer):
            self.grads['W' + str(i + 1)] = 0.0
            self.grads['b' + str(i + 1)] = 0.0

    def forward(self, X):
        """
        Performs the forward pass of the model.

        :param X: Input data of shape N x D. Each X[i] is a training sample.
        :return: Predicted value for the data in X, shape N x 1
                 1-dimensional array of length N with the classification scores.
        """

        self.cache = {}
        self.reg = {}
        X = X.reshape(X.shape[0], -1)
        # Unpack variables from the params dictionary
        for i in range(self.num_layer - 1):
            W, b = self.params['W' + str(i + 1)], self.params['b' + str(i + 1)]

            # Forward i_th layer
            X, cache_affine = affine_forward(X, W, b)
            self.cache["affine" + str(i + 1)] = cache_affine

            # Activation function
            X, cache_sigmoid = self.activation.forward(X)
            self.cache["sigmoid" + str(i + 1)] = cache_sigmoid

            # Store the reg for the current W
            self.reg['W' + str(i + 1)] = np.sum(W ** 2) * self.reg_strength

        # last layer contains no activation functions
        W, b = self.params['W' + str(self.num_layer)],\
               self.params['b' + str(self.num_layer)]
        y, cache_affine = affine_forward(X, W, b)
        self.cache["affine" + str(self.num_layer)] = cache_affine
        self.reg['W' + str(self.num_layer)] = np.sum(W ** 2) * self.reg_strength

        return y

    def backward(self, dy):
        """
        Performs the backward pass of the model.

        :param dy: N x 1 array. The gradient wrt the output of the network.
        :return: Gradients of the model output wrt the model weights
        """

        # Note that last layer has no activation
        cache_affine = self.cache['affine' + str(self.num_layer)]
        dh, dW, db = affine_backward(dy, cache_affine)
        self.grads['W' + str(self.num_layer)] = \
            dW + 2 * self.reg_strength * self.params['W' + str(self.num_layer)]
        self.grads['b' + str(self.num_layer)] = db

        # The rest sandwich layers
        for i in range(self.num_layer - 2, -1, -1):
            # Unpack cache
            cache_sigmoid = self.cache['sigmoid' + str(i + 1)]
            cache_affine = self.cache['affine' + str(i + 1)]

            # Activation backward
            dh = self.activation.backward(dh, cache_sigmoid)

            # Affine backward
            dh, dW, db = affine_backward(dh, cache_affine)

            # Refresh the gradients
            self.grads['W' + str(i + 1)] = dW + 2 * self.reg_strength * \
                                           self.params['W' + str(i + 1)]
            self.grads['b' + str(i + 1)] = db

        return self.grads

    def save_model(self):
        directory = 'models'
        model = {self.model_name: self}
        if not os.path.exists(directory):
            os.makedirs(directory)
        pickle.dump(model, open(directory + '/' + self.model_name + '.p', 'wb'))

    def get_dataset_prediction(self, loader):
        scores = []
        labels = []
        
        for batch in loader:
            X = batch['image']
            y = batch['label']
            score = self.forward(X)
            scores.append(score)
            labels.append(y)
            
        scores = np.concatenate(scores, axis=0)
        labels = np.concatenate(labels, axis=0)

        preds = scores.argmax(axis=1)
        acc = (labels == preds).mean()

        return labels, preds, acc

```

## 4.4 调参前的准备

- 我们需要先从一个小而简单的结构来调整我们的网络结构。

  As a first step you should try to overfit to a single training sample, then to a few batches of training samples and finally go deeper with larger neural networks and the whole training data.

- 注意用2.10的solver接口来统一调用4.3的model/其他model。

- 通过这一系列操作，可以看看我们的代码是否有问题

  - 1，2：检测能否处理不同数目的数据集
  - 3，4：检测能否处理不同的隐藏层数

1. First, let's start with a 2-layer neural network, and overfit to one single training sample.

   ```python
   from exercise_code.solver import Solver
   from exercise_code.networks.optimizer import SGD, Adam
   from exercise_code.networks import MyOwnNetwork
   ## 调参部分：
   num_layer = 2
   epochs = 20
   reg = 0.1
   batch_size = 4
   
   model = ClassificationNet(num_layer=num_layer, reg=reg)
   # model = MyOwnNetwork()
   
   loss = CrossEntropyFromLogits()
   
   # Make a new data loader with a single training image
   overfit_dataset = DATASET(
       mode='train',
       root=cifar_root, 
       download_url=download_url,
       transform=compose_transform,
       limit_files=1
   )
   dataloaders['train_overfit_single_image'] = DataLoader(
       dataset=overfit_dataset,
       batch_size=batch_size,
       shuffle=True,
       drop_last=False,
   )
   
   # Decrease validation data for only debugging
   debugging_validation_dataset = DATASET(
       mode='val',
       root=cifar_root, 
       download_url=download_url,
       transform=compose_transform,
       limit_files=100
   )
   dataloaders['val_500files'] = DataLoader(
       dataset=debugging_validation_dataset,
       batch_size=batch_size,
       shuffle=True,
       drop_last=True,
   )
   
   solver = Solver(model, dataloaders['train_overfit_single_image'], dataloaders['val_500files'], 
                   learning_rate=1e-3, loss_func=loss, optimizer=Adam)
   solver.train(epochs=epochs)
   
   ## 将train 和validation数据集的loss曲线画出来，可以看出是overfit了吗
   plt.title('Loss curves')
   plt.plot(solver.train_loss_history, '-', label='train')
   plt.plot(solver.val_loss_history, '-', label='val')
   plt.legend(loc='lower right')
   plt.xlabel('Iteration')
   plt.show()
   
   print("Training accuray: %.5f" % (solver.get_dataset_accuracy(dataloaders['train_overfit_single_image'])))
   print("Validation accuray: %.5f" % (solver.get_dataset_accuracy(dataloaders['val_500files'])))
   ```

2. This time we want to overfit to a small set of training batch samples

   ```python
   from exercise_code.networks import MyOwnNetwork
   ## 调参部分：
   num_layer = 2
   epochs = 100
   reg = 0.1
   num_samples = 10
   
   model = ClassificationNet(num_layer=num_layer, reg=reg)
   # model = MyOwnNetwork()
   
   loss = CrossEntropyFromLogits()
   
   # Make a new data loader with a our num_samples training image
   overfit_dataset = DATASET(
       mode='train',
       root=cifar_root, 
       download_url=download_url,
       transform=compose_transform,
       limit_files=num_samples
   )
   dataloaders['train_overfit_10samples'] = DataLoader(
       dataset=overfit_dataset,
       batch_size=batch_size,
       shuffle=True,
       drop_last=False,
   )
   
   solver = Solver(model, dataloaders['train_overfit_10samples'], dataloaders['val_500files'], 
                   learning_rate=1e-3, loss_func=loss, optimizer=Adam)
   solver.train(epochs=epochs)
   
   ## 将train 和validation数据集的loss曲线画出来，可以看出是overfit了吗
   plt.title('Loss curves')
   plt.plot(solver.train_loss_history, '-', label='train')
   plt.plot(solver.val_loss_history, '-', label='val')
   plt.legend(loc='lower right')
   plt.xlabel('Iteration')
   plt.show()
   
   print("Training accuray: %.5f" % (solver.get_dataset_accuracy(dataloaders['train_overfit_10samples'])))
   print("Validation accuray: %.5f" % (solver.get_dataset_accuracy(dataloaders['val_500files'])))
   ```

3. 先看看2层跑的怎么样

   ```python
   from exercise_code.networks import MyOwnNetwork
   
   num_layer = 2
   epochs = 5
   reg = 0.01
   
   # Make a new data loader with 1000 training samples
   num_samples = 1000
   overfit_dataset = DATASET(
       mode='train',
       root=cifar_root, 
       download_url=download_url,
       transform=compose_transform,
       limit_files=num_samples
   )
   dataloaders['train_small'] = DataLoader(
       dataset=overfit_dataset,
       batch_size=batch_size,
       shuffle=True,
       drop_last=False,
   )
   
   
   # Change here if you want to use the full training set
   use_full_training_set = False
   if not use_full_training_set:
       train_loader = dataloaders['train_small']
   else:
       train_loader = dataloaders['train']
       
   
   model = ClassificationNet(num_layer=num_layer, reg=reg)
   # model = MyOwnNetwork()
   
   loss = CrossEntropyFromLogits()
   
   solver = Solver(model, train_loader, dataloaders['val'], 
                   learning_rate=1e-3, loss_func=loss, optimizer=Adam)
   
   solver.train(epochs=epochs)
   
   ## 效仿上面1，2画loss曲线
   ```

4. 再看看5层跑的怎么样

   ```python
   from exercise_code.networks import MyOwnNetwork
   
   num_layer = 5
   epochs = 5
   reg = 0.01
   
   model = ClassificationNet(num_layer=num_layer, reg=reg)
   # model = MyOwnNetwork()
   
   # Change here if you want to use the full training set
   use_full_training_set = False
   if not use_full_training_set:
       train_loader = dataloaders['train_small']
   else:
       train_loader = dataloaders['train']
   
   loss = CrossEntropyFromLogits()
   
   solver = Solver(model, train_loader, dataloaders['val'], 
                   learning_rate=1e-3, loss_func=loss, optimizer=Adam)
   
   solver.train(epochs=epochs)
   ```

## 4.5调参方法

### 4.5.1Grid Search

- 代码： 

  ```python
  def grid_search(train_loader, val_loader,
                  grid_search_spaces = {
                      "learning_rate": [0.0001, 0.001, 0.01, 0.1],
                      "reg": [1e-4, 1e-5, 1e-6]
                  },
                  model_class=ClassificationNet, epochs=20, patience=5):
      """
      A simple grid search based on nested loops to tune learning rate and
      regularization strengths.
      Keep in mind that you should not use grid search for higher-dimensional
      parameter tuning, as the search space explodes quickly.
  
      Required arguments:
          - train_dataloader: A generator object returning training data
          - val_dataloader: A generator object returning validation data
  
      Optional arguments:
          - grid_search_spaces: a dictionary where every key corresponds to a
          to-tune-hyperparameter and every value contains a list of possible
          values. Our function will test all value combinations which can take
          quite a long time. If we don't specify a value here, we will use the
          default values of both our chosen model as well as our solver
          - model: our selected model for this exercise
          - epochs: number of epochs we are training each model
          - patience: if we should stop early in our solver
  
      Returns:
          - The best performing model
          - A list of all configurations and results
      """
      configs = []
  
      """
      # Simple implementation with nested loops
      for lr in grid_search_spaces["learning_rate"]:
          for reg in grid_search_spaces["reg"]:
              configs.append({"learning_rate": lr, "reg": reg})
      """
  
      # More general implementation using itertools
      for instance in product(*grid_search_spaces.values()):
          configs.append(dict(zip(grid_search_spaces.keys(), instance)))
  
      return findBestConfig(train_loader, val_loader, configs, epochs, patience,
                            model_class)
  ```

- 调用：

  ```python
  from exercise_code.networks import MyOwnNetwork
  
  # Specify the used network
  model_class = ClassificationNet
  
  from exercise_code import hyperparameter_tuning
  best_model, results = hyperparameter_tuning.grid_search(
      dataloaders['train_small'], dataloaders['val_500files'],
      grid_search_spaces = {
          "learning_rate": [1e-2, 1e-3, 1e-4], 
          "reg": [1e-4]
      },
      model_class=model_class,
      epochs=10, patience=5)
  ```

### 4.5.2 Random Search

- 代码

  ```python
  def random_search(train_loader, val_loader,
                    random_search_spaces = {
                        "learning_rate": ([0.0001, 0.1], 'log'),
                        "hidden_size": ([100, 400], "int"),
                        "activation": ([Sigmoid(), Relu()], "item"),
                    },
                    model_class=ClassificationNet, num_search=20, epochs=20,
                    patience=5):
      """
      Samples N_SEARCH hyper parameter sets within the provided search spaces
      and returns the best model.
  
      See the grid search documentation above.
  
      Additional/different optional arguments:
          - random_search_spaces: similar to grid search but values are of the
          form
          (<list of values>, <mode as specified in ALLOWED_RANDOM_SEARCH_PARAMS>)
          - num_search: number of times we sample in each int/float/log list
      """
      configs = []
      for _ in range(num_search):
          configs.append(random_search_spaces_to_config(random_search_spaces))
  
      return findBestConfig(train_loader, val_loader, configs, epochs, patience,
                            model_class)
  
      
  def random_search_spaces_to_config(random_search_spaces):
      """"
      Takes search spaces for random search as input; samples accordingly
      from these spaces and returns the sampled hyper-params as a config-object,
      which will be used to construct solver & network
      """
      
      config = {}
  
      for key, (rng, mode)  in random_search_spaces.items():
          if mode not in ALLOWED_RANDOM_SEARCH_PARAMS:
              print("'{}' is not a valid random sampling mode. "
                    "Ignoring hyper-param '{}'".format(mode, key))
          elif mode == "log":
              if rng[0] <= 0 or rng[-1] <=0:
                  print("Invalid value encountered for logarithmic sampling "
                        "of '{}'. Ignoring this hyper param.".format(key))
                  continue
              sample = random.uniform(log10(rng[0]), log10(rng[-1]))
              config[key] = 10**(sample)
          elif mode == "int":
              config[key] = random.randint(rng[0], rng[-1])
          elif mode == "float":
              config[key] = random.uniform(rng[0], rng[-1])
          elif mode == "item":
              config[key] = random.choice(rng)
  
      return config
  ```

- 调用：

  ```python
  from exercise_code.hyperparameter_tuning import random_search
  from exercise_code.networks import MyOwnNetwork
  
  # Specify the used network
  model_class = ClassificationNet
  
  best_model, results = random_search(
      dataloaders['train_small'], dataloaders['val_500files'],
      random_search_spaces = {
          "learning_rate": ([1e-2, 1e-6], 'log'),
          "reg": ([1e-3, 1e-7], "log"),
          "loss_func": ([CrossEntropyFromLogits()], "item")
      },
      model_class=model_class,
      num_search = 1, epochs=20, patience=5)
  ```

### 4.5.3 找到最好的那一组数据：

- 通过计算train loss, val loss来得到：

  ```python
  def findBestConfig(train_loader, val_loader, configs, EPOCHS, PATIENCE,
                     model_class):
      """
      Get a list of hyperparameter configs for random search or grid search,
      trains a model on all configs and returns the one performing best
      on validation set
      """
      
      best_val = None
      best_config = None
      best_model = None
      results = []
      
      for i in range(len(configs)):
          print("\nEvaluating Config #{} [of {}]:\n".format(
              (i+1), len(configs)),configs[i])
  
          model = model_class(**configs[i])
          solver = Solver(model, train_loader, val_loader, **configs[i])
          solver.train(epochs=EPOCHS, patience=PATIENCE)
          results.append(solver.best_model_stats)
  
          if not best_val or solver.best_model_stats["val_loss"] < best_val:
              best_val, best_model,\
              best_config = solver.best_model_stats["val_loss"], model, configs[i]
              
      print("\nSearch done. Best Val Loss = {}".format(best_val))
      print("Best Config:", best_config)
      return best_model, list(zip(configs, results))
  ```

### 4.5.4 Early stopping

Usually, at some point the validation loss goes up again, which is a sign that we're overfitting to our training data. Since it actually doesn't make sense to train further at this point, it's common practice to apply "early stopping", i.e., cancel the training process when the validation loss doesn't improve anymore. The nice thing about this concept is, that not only it improves generalization through the prevention of overfitting, but also it saves us a lot of time - one of our most valuable resources in deep learning.

Since there are natural fluctuations in the validation loss, you usually don't cancel the training process right at the first epoch when the validation-loss increases, but instead, you wait for some epochs **(specified by the `patience`-parameter)** and if the loss still doesn't improve, we stop.

**实现代码见2.10**



## 4.6 实际调参：

策略：At the beginning, it's a good approach to first do a coarse random search across a **wide range of values** to find promising sub-ranges of your parameter space and use **a medium large subset of the dataset** . Afterwards, you can zoom into these ranges and do another random search (or grid search) to finetune the configurations. Use the cell below to play around and find good hyperparameters for your model!

```python
from exercise_code.networks import MyOwnNetwork

best_model = ClassificationNet()
#best_model = MyOwnNetwork()

########################################################################
# TODO:                                                                #
# Implement your own neural network and find suitable hyperparameters  #
# Be sure to edit the MyOwnNetwork class in the following code snippet #
# to upload the correct model!                                         #
########################################################################
model_class = ClassificationNet

best_model, results = random_search(
    dataloaders['train_small'], dataloaders['val_500files'],
    random_search_spaces = {
        "learning_rate": ([1e-2, 1e-6], 'log'),
        "reg": ([1e-3, 1e-7], "log"),
        "loss_func": ([CrossEntropyFromLogits()], "item"),
        "num_layer":([3,4],"int")
    },
    model_class=model_class,
    num_search = 4, epochs=20, patience=5)


########################################################################
#                           END OF YOUR CODE                           #
########################################################################
```

# 5. 用PyTorch Lighhtning来Cifar10分类

使用PyTorch Lightning可以方便的调用各种硬件资源以及调试程序。让我们专注于模型的改进。

## 5.1 加载库

```python
import os
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, random_split
import torchvision
import torchvision.transforms as transforms
%load_ext autoreload
%autoreload 2

# 使用GPU
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(device)
```

## 5.2 使用TensorBoard调试

Anaconda的shell中输

```
tensorboard --logdir lightning_logs --port 6006
```

## 5.3 用PL定义模型

### 5.3.1 数据加载

PyTorch Lightning 提供 `LightningDataModule` 作为基类，来设置dataloaders数据加载.

```python
import pytorch_lightning as pl
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, random_split
from torch.utils.data.sampler import SubsetRandomSampler, RandomSampler, SequentialSampler
import torchvision
import torchvision.transforms as transforms
import numpy as np
from tqdm import tqdm

class CIFAR10DataModule(pl.LightningDataModule):
    def __init__(self, hparams):
        super().__init__()
        self.opt = hparams
        if 'loading_method' not in hparams.keys():
            self.opt['loading_method'] = 'Image'
        if 'num_workers' not in hparams.keys():
            self.opt['num_workers'] = 2

    def prepare_data(self, stage=None, CIFAR_ROOT="../datasets/cifar10"):
        mean = [0.485, 0.456, 0.406]
        std = [0.229, 0.224, 0.225]

        # create dataset
        CIFAR_ROOT = "../datasets/cifar10"
        my_transform = None
        mean = [0.485, 0.456, 0.406]
        std = [0.229, 0.224, 0.225]
        # 预处理数据（PyTorch的函数方法)
        # transforms.Compose()建立一些预处理方法来处理数据
        # transforms.ToTensor()讲图片或数组从(0,255)转为(0,1)的Tensor（归一化）
        # transforms.Normalize(mean,std)用提供的平均数，标准差来去均值实现中心化的处理（标准化）
        my_transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean,std)
        ])
        
        # Make sure to use a consistent transform for validation/test
        train_val_transform = transforms.Compose([transforms.ToTensor(), transforms.Normalize(mean, std)])

        # Note: you can change the splits if you want :)
        split = {
            'train': 0.6,
            'val': 0.2,
            'test': 0.2
        }
        split_values = [v for k,v in split.items()]
        assert sum(split_values) == 1.0
        
        ## 划分数据并进行数据预处理
        if self.opt['loading_method'] == 'Image':
            # Set up a full dataset with the two respective transforms
            cifar_complete_augmented = torchvision.datasets.ImageFolder(root=CIFAR_ROOT, transform=my_transform)
            cifar_complete_train_val = torchvision.datasets.ImageFolder(root=CIFAR_ROOT, transform=train_val_transform)

            # Instead of splitting the dataset in the beginning you can also # split using a sampler. This is not better, but we wanted to show it off here as an example by using the default ImageFolder dataset :)

            # First regular splitting which we did for you before
            N = len(cifar_complete_augmented)        
            num_train, num_val = int(N*split['train']), int(N*split['val'])
            indices = np.random.permutation(N)
            train_idx, val_idx, test_idx = indices[:num_train], indices[num_train:num_train+num_val], indices[num_train+num_val:]

            # Now we can set the sampler via the respective subsets
            train_sampler = SubsetRandomSampler(train_idx)
            val_sampler = SubsetRandomSampler(val_idx)
            test_sampler= SubsetRandomSampler(test_idx)
            self.sampler = {"train": train_sampler, "val": val_sampler, "test": test_sampler}

            # assign to use in dataloaders
            self.dataset = {}
            self.dataset["train"], self.dataset["val"], self.dataset["test"] = cifar_complete_augmented,\
                cifar_complete_train_val, cifar_complete_train_val

        elif self.opt['loading_method'] == 'Memory':
            self.dataset = {}
            self.sampler = {}

            for mode in ['train', 'val', 'test']:
                # Set transforms
                if mode == 'train':
                    transform = my_transform
                else:
                    transform = train_val_transform

                self.dataset[mode] = MemoryImageFolderDataset(
                    root = CIFAR_ROOT,
                    transform = transform,
                    mode = mode,
                    split = split
                )
        else:
            raise NotImplementedError("Wrong loading method")

    def return_dataloader_dict(self, mode):
        arg_dict = {
            'batch_size': self.opt["batch_size"],
            'num_workers': self.opt['num_workers'],
            'persistent_workers': True,
            'pin_memory': True
        }
        if self.opt['loading_method'] == 'Image':
            arg_dict['sampler'] = self.sampler[mode]
        elif self.opt['loading_method'] == 'Memory':
            arg_dict['shuffle'] = True if mode == 'train' else False
        return arg_dict

    def train_dataloader(self):
        arg_dict = self.return_dataloader_dict('train')
        return DataLoader(self.dataset["train"], **arg_dict)

    def val_dataloader(self):
        arg_dict = self.return_dataloader_dict('val')
        return DataLoader(self.dataset["val"], **arg_dict)
    
    def test_dataloader(self):
        arg_dict = self.return_dataloader_dict('train')
        return DataLoader(self.dataset["train"], **arg_dict)
```

- Dataloader

   `DataLoader` class that is used to create `train_dataloader` and `val_dataloader`

  ```python
  import numpy as np
  class DataLoader:
      """
      Dataloader Class
      Defines an iterable batch-sampler over a given dataset
      """
      def __init__(self, dataset, batch_size=1, shuffle=False, drop_last=False):
          """
          :param dataset: dataset from which to load the data
          :param batch_size: how many samples per batch to load
          :param shuffle: set to True to have the data reshuffled at every epoch
          :param drop_last: set to True to drop the last incomplete batch,
              if the dataset size is not divisible by the batch size.
              If False and the size of dataset is not divisible by the batch
              size, then the last batch will be smaller.
          """
          self.dataset = dataset
          self.batch_size = batch_size
          self.shuffle = shuffle
          self.drop_last = drop_last
  
      def __iter__(self):
          def combine_batch_dicts(batch):
              """
              Combines a given batch (list of dicts) to a dict of numpy arrays
              :param batch: batch, list of dicts
                  e.g. [{k1: v1, k2: v2, ...}, {k1:, v3, k2: v4, ...}, ...]
              :returns: dict of numpy arrays
                  e.g. {k1: [v1, v3, ...], k2: [v2, v4, ...], ...}
              """
              batch_dict = {}
              for data_dict in batch:
                  for key, value in data_dict.items():
                      if key not in batch_dict:
                          batch_dict[key] = []
                      batch_dict[key].append(value)
              return batch_dict
  
          def batch_to_numpy(batch):
              """Transform all values of the given batch dict to numpy arrays"""
              numpy_batch = {}
              for key, value in batch.items():
                  numpy_batch[key] = np.array(value)
              return numpy_batch
  
          if self.shuffle:
              index_iterator = iter(np.random.permutation(len(self.dataset)))
          else:
              index_iterator = iter(range(len(self.dataset)))
  
          batch = []
          for index in index_iterator:
              batch.append(self.dataset[index])
              if len(batch) == self.batch_size:
                  yield batch_to_numpy(combine_batch_dicts(batch))
                  batch = []
  
          if len(batch) > 0 and not self.drop_last:
              yield batch_to_numpy(combine_batch_dicts(batch))
  
      def __len__(self):
          length = None
  
          if self.drop_last:
              length = len(self.dataset) // self.batch_size
          else:
              length = int(np.ceil(len(self.dataset) / self.batch_size))
  
          return length
  ```

### 5.3.2 神经网络结构

PyTorch Lightning的神经网络模型是基于`pl.LightningModule`类来创建的

根据[torch.nn](https://pytorch.org/docs/stable/nn.html)来定义我们的模型。

根据[torch.optim](https://pytorch.org/docs/stable/optim.html)来定义我们的优化器

```python
class MyPytorchModel(pl.LightningModule):
    
    def __init__(self, hparams):
        super().__init__()

        # set hyperparams
        self.save_hyperparameters(hparams)
        ## 定义神经网络模型
        self.model = None
        self.model = nn.Sequential(
            nn.Linear(self.hparams["input_size"],self.hparams["nn_hidden_Layer1"]),
            nn.ReLU(),
            nn.Linear(self.hparams["nn_hidden_Layer1"],self.hparams["num_classes"]),
            nn.ReLU()
        )

    def forward(self, x):

        # x.shape = [batch_size, 3, 32, 32] -> flatten the image first
        x = x.view(x.shape[0], -1)

        # feed x into model!
        x = self.model(x)

        return x
    
    def general_step(self, batch, batch_idx, mode):
        images, targets = batch

        # forward pass
        out = self.forward(images)

        # loss
        loss = F.cross_entropy(out, targets)

        preds = out.argmax(axis=1)
        n_correct = (targets == preds).sum()
        n_total = len(targets)
        return loss, n_correct, n_total
    
    def general_end(self, outputs, mode):
        # average over all batches aggregated during one epoch
        avg_loss = torch.stack([x[mode + '_loss'] for x in outputs]).mean()
        length = sum([x[mode + '_n_total'] for x in outputs])
        total_correct = torch.stack([x[mode + '_n_correct'] for x in outputs]).sum().cpu().numpy()
        acc = total_correct / length
        return avg_loss, acc

    def training_step(self, batch, batch_idx):
        loss, n_correct, n_total = self.general_step(batch, batch_idx, "train")
        self.log('loss',loss)
        return {'loss': loss, 'train_n_correct':n_correct, 'train_n_total': n_total}

    def validation_step(self, batch, batch_idx):
        loss, n_correct, n_total = self.general_step(batch, batch_idx, "val")
        self.log('val_loss',loss)
        return {'val_loss': loss, 'val_n_correct':n_correct, 'val_n_total': n_total}
    
    def test_step(self, batch, batch_idx):
        loss, n_correct, n_total = self.general_step(batch, batch_idx, "test")
        return {'test_loss': loss, 'test_n_correct':n_correct, 'test_n_total': n_total}

    def validation_epoch_end(self, outputs):
        avg_loss, acc = self.general_end(outputs, "val")
        self.log('val_loss',avg_loss)
        self.log('val_acc',acc)
        return {'val_loss': avg_loss, 'val_acc': acc}

    def configure_optimizers(self):
        optim = torch.optim.Adam(self.model.parameters(),self.hparams["learning_rate"], weight_decay=self.hparams['weight_decay'])
        StepLR = torch.optim.lr_scheduler.MultiStepLR(optim, milestones=[30], gamma=0.5)
        return optim

    def getTestAcc(self, loader):
        self.model.eval()
        self.model = self.model.to(self.device)

        scores = []
        labels = []

        for batch in tqdm(loader):
            X, y = batch
            X = X.to(self.device)
            score = self.forward(X)
            scores.append(score.detach().cpu().numpy())
            labels.append(y.detach().cpu().numpy())

        scores = np.concatenate(scores, axis=0)
        labels = np.concatenate(labels, axis=0)

        preds = scores.argmax(axis=1)
        acc = (labels == preds).mean()
        return preds, acc

```



## 5.4 调参

```python
from exercise_code.MyPytorchModel import MyPytorchModel, CIFAR10DataModule
# make sure you have downloaded the Cifar10 dataset on root: "../datasets/cifar10", if not, please check exercise 03.
hparams = {
    "loading_method":"Memory",
    "num_workers":1,
    "input_size":3*3*32,
    "batch_size":1000,
    "learning_rate":5e-5,
    "weight_decay":1e-3,
    "nn_hidden_Layer1":1500,
    "num_classes":10
} 

# Make sure you downloaded the CIFAR10 dataset already when using this cell
# since we are showcasing the pytorch inhering ImageFolderDataset that
# doesn't automatically download our data. Check exercise 3

# If you want to switch to the memory dataset instead of image folder use
# hparams["loading_method"] = 'Memory'
# The default is hparams["loading_method"] = 'Image'
# You will notice that it takes way longer to initialize a MemoryDataset
# method because we have to load the data points into memory all the time.

# You might get warnings below if you use too few workers. Pytorch uses
# a more sophisticated Dataloader than the one you implemented previously.
# In particular it uses multi processing to have multiple cores work on
# individual data samples. You can enable more than workers (default=2)
# via 
# hparams['num_workers'] = 8

# Set up the data module including your implemented transforms
data_module = CIFAR10DataModule(hparams)
data_module.prepare_data()
# Initialize our model
model = MyPytorchModel(hparams)
```

## 5.5训练

使用pytorch_lightning的[trainer](https://pytorch-lightning.readthedocs.io/en/stable/common/trainer.html)

```python
import pytorch_lightning as pl
trainer = None

trainer = pl.Trainer(
    max_epochs=2,
    accelerator = 'gpu' 
)

trainer.fit(model, data_module)
```

# 6.Autoencoder for MNIST in PyTorch Lightning

MNIST数据库是一个手写数字的大型数据库，通常用于训练各种图像处理系统。

## 6.1 加载库

```python
# 魔法函数
%load_ext autoreload
%autoreload 2
%matplotlib inline
import numpy as np
import os
import matplotlib.pyplot as plt
import torch
from torchvision import transforms
import pytorch_lightning as pl
from exercise_code.image_folder_dataset import ImageFolderDataset
from pytorch_lightning.loggers import TensorBoardLogger
torch.manual_seed(42)
# 使用GPU
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(device)
# 使用TensorBoard调参
%load_ext tensorboard
%tensorboard --logdir lightning_logs --port 6006
```

## 6.2下载数据集

```python
# 图像预处理(PyTorch)
transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.ToTensor(),
    transforms.Normalize((0.5,),(0.5,))
])

i2dl_exercises_path = os.path.dirname(os.path.abspath(os.getcwd()))
mnist_root = os.path.join(i2dl_exercises_path, "datasets", "mnist")

# 下载带标签的数据集
train = ImageFolderDataset(root=mnist_root,images='train_images.pt',labels='train_labels.pt',force_download=False,verbose=True,transform=transform)
val = ImageFolderDataset(root=mnist_root,images='val_images.pt',labels='val_labels.pt',force_download=False,verbose=True,transform=transform)
test = ImageFolderDataset(root=mnist_root,images='test_images.pt',labels='test_labels.pt',force_download=False,verbose=True,transform=transform)

# 下载不带标签的数据集
# We also set up the unlabeled images which we will use later
unlabeled_train = ImageFolderDataset(root=mnist_root,images='unlabeled_train_images.pt',force_download=False,verbose=True,transform=transform)
unlabeled_val = ImageFolderDataset(root=mnist_root,images='unlabeled_val_images.pt',force_download=False,verbose=True,transform=transform)
```

- 看几个图片，看图片和label是否配对

  ```python
  plt.rcParams['figure.figsize'] = (6,6) # Make the figures a bit bigger
  
  for i in range(9):
      image = np.array(train[i][0].squeeze()) # get the image of the data sample
      label = train[i][1] # get the label of the data sample
      plt.subplot(3,3,i+1)
      plt.imshow(image, cmap='gray', interpolation='none')
      plt.title("Class {}".format(label))
      
  plt.tight_layout()
  print('The shape of our greyscale images: ', image.shape)
  ```


## 6.3 一个简单的分类器

用于分类图片是0-9中的某一个数字

```python
class Classifier(pl.LightningModule):

    def __init__(self, hparams, encoder, train_set=None, val_set=None, test_set=None):
        super().__init__()
        # set hyperparams
        self.save_hyperparameters(hparams, ignore=['encoder'])
        self.encoder = encoder
        self.model = nn.Identity()
        self.data = {'train': train_set,
                     'val': val_set,
                     'test': test_set}
        self.model = nn.Sequential(
            nn.Linear(20,self.hparams["hidden_size_2"]),
            nn.LeakyReLU(),
            nn.Linear(self.hparams["hidden_size_2"],self.hparams["output_size"]),
        )

    def forward(self, x):
        # 这里的变量都是pytorch的tensor类型
        # 先调用encoder类来将图片有意义的地方提取出来
        x = self.encoder(x)
        # 再调用classifier来区分数字
        x = self.model(x)
        return x

    def general_step(self, batch, batch_idx, mode):
        # 获得样本和标签，tensor类型
        images, targets = batch
        # tensor.shape()返回tensor的形状
        # tensor.view()把数据变形成nxm的数据，-1表示这个位置由其他位置的的数字推断而出。
        # 例：一个tensor是2X3的，排成一维就是tensor.view(1,6),排成3x2就是tensor.view(3,2)
        # 这里images.shape[0]就是n个样本，第二个-1自动推断出D个特征
        flattened_images = images.view(images.shape[0], -1)
        # forward pass
        out = self.forward(flattened_images)
        # loss
        loss = F.cross_entropy(out, targets)
        preds = out.argmax(axis=1)
        n_correct = (targets == preds).sum()
        return loss, n_correct

    def general_end(self, outputs, mode):
        # average over all batches aggregated during one epoch
        avg_loss = torch.stack([x[mode + '_loss'] for x in outputs]).mean()
        total_correct = torch.stack(
            [x[mode + '_n_correct'] for x in outputs]).sum().cpu().numpy()
        acc = total_correct / len(self.data[mode])
        return avg_loss, acc

    def training_step(self, batch, batch_idx):
        loss, n_correct = self.general_step(batch, batch_idx, "train")
        self.log("train_loss_cls", loss)
        return loss

    def validation_step(self, batch, batch_idx):
        loss, n_correct = self.general_step(batch, batch_idx, "val")
        return {'val_loss': loss, 'val_n_correct': n_correct}

    def test_step(self, batch, batch_idx):
        loss, n_correct = self.general_step(batch, batch_idx, "test")
        return {'test_loss': loss, 'test_n_correct': n_correct}

    def validation_end(self, outputs):
        avg_loss, acc = self.general_end(outputs, "val")
        self.log("val_loss", avg_loss)
        self.log("val_acc", acc)

    def train_dataloader(self):
        return torch.utils.data.DataLoader(self.data['train'], shuffle=True, batch_size=self.hparams['batch_size'])

    def val_dataloader(self):
        return torch.utils.data.DataLoader(self.data['val'], batch_size=self.hparams['batch_size'])

    def test_dataloader(self):
        return torch.utils.data.DataLoader(self.data['test'], batch_size=self.hparams['batch_size'])

    def configure_optimizers(self):
        optim = torch.optim.Adam(self.model.parameters(),self.hparams["learning_rate"],weight_decay = self.hparams['weight_decay'])
        return optim

    def getAcc(self, loader=None):
        self.eval()
        self = self.to(self.device)

        if not loader:
            loader = self.test_dataloader()

        scores = []
        labels = []

        for batch in loader:
            X, y = batch
            X = X.to(self.device)
            flattened_X = X.view(X.shape[0], -1)
            score = self.forward(flattened_X)
            scores.append(score.detach().cpu().numpy())
            labels.append(y.detach().cpu().numpy())

        scores = np.concatenate(scores, axis=0)
        labels = np.concatenate(labels, axis=0)

        preds = scores.argmax(axis=1)
        acc = (labels == preds).mean()
        return preds, acc
```

## 6.4 Autoencoder

图片需要认为label后才能用于训练，这会花费大量的人力。一种解决方式是Data Augmentation

另一种方法是：**transfer learning**

- 首先我们用没有标记过的图片来训练Encoder-Decoder。这样encoder的神经网络就具备了用低维度latent space来表示原图像的能力

  1. Encoder:
     - The `encoder`'s task is to extract meaningful information out of our input so that the classifier can make a proper decision.

  2. Decoder:
     - 将encoder中压缩的图片，再还原成原图
  3. 因为是没有标记过的图片所以我们的loss function需要改成mean squared error between our input pixels and output pixels.
     - 其实就是对比原图和还原图

- 接着我们再用标记过的图片来训练Encoder-Decoder。这样encoder神经网络的权重我们就可以直接使用上面的权重，只需要训练classifier的权重了

### 6.4.1 Encoder

```python
class Encoder(nn.Module):

    def __init__(self, hparams, input_size=28 * 28, latent_dim=20):
        super().__init__()

        # set hyperparams
        self.latent_dim = latent_dim
        self.input_size = input_size
        self.hparams = hparams
        self.encoder = None

        self.encoder = nn.Sequential(
            nn.Linear(self.input_size,self.hparams["hidden_size"]),
            nn.LeakyReLU(),
            nn.Linear(self.hparams["hidden_size"],latent_dim)
        )

    def forward(self, x):
        # feed x into encoder!
        return self.encoder(x)
```

### 6.4.2 Decoder

```python
class Decoder(nn.Module):

    def __init__(self, hparams, latent_dim=20, output_size=28 * 28):
        super().__init__()

        # set hyperparams
        self.hparams = hparams
        self.decoder = None

        self.decoder = nn.Sequential(
            nn.Linear(latent_dim,self.hparams["hidden_size"]),
            nn.LeakyReLU(),
            nn.Linear(self.hparams["hidden_size"],output_size)
        )

    def forward(self, x):
        # feed x into decoder!
        return self.decoder(x)
```

### 6.4.3 AutoEncoder

```python
class Autoencoder(pl.LightningModule):

    def __init__(self, hparams, encoder, decoder, train_set, val_set):
        super().__init__()
        # set hyperparams
        self.save_hyperparameters(hparams, ignore=['encoder', 'decoder'])

        # Define models
        self.encoder = encoder
        self.decoder = decoder
        self.train_set = train_set
        self.val_set = val_set

    def forward(self, x):
        reconstruction = None
        x = self.encoder(x)
        reconstruction = self.decoder(x)
        return reconstruction

    def general_step(self, batch, batch_idx, mode):
        images = batch
        flattened_images = images.view(images.shape[0], -1)

        # forward pass
        reconstruction = self.forward(flattened_images)

        # loss
        loss = F.mse_loss(reconstruction, flattened_images)

        return loss, reconstruction

    def general_end(self, outputs, mode):
        # average over all batches aggregated during one epoch
        avg_loss = torch.stack([x[mode + '_loss'] for x in outputs]).mean()
        return avg_loss

    def training_step(self, batch, batch_idx):
        loss, _ = self.general_step(batch, batch_idx, "train")
        self.log("train_loss_ae", loss)
        return loss

    def validation_step(self, batch, batch_idx):
        images = batch
        flattened_images = images.view(images.shape[0], -1)

        reconstruction = self.forward(flattened_images)
        loss = F.mse_loss(reconstruction, flattened_images)

        reconstruction = reconstruction.view(
            reconstruction.shape[0], 28, 28).cpu().numpy()
        images = np.zeros((len(reconstruction), 3, 28, 28))
        for i in range(len(reconstruction)):
            images[i, 0] = reconstruction[i]
            images[i, 2] = reconstruction[i]
            images[i, 1] = reconstruction[i]
        self.logger.experiment.add_images(
            'reconstructions', images, self.current_epoch, dataformats='NCHW')
        return loss

    def train_dataloader(self):
        return torch.utils.data.DataLoader(self.train_set, shuffle=True, batch_size=self.hparams['batch_size'])

    def val_dataloader(self):
        return torch.utils.data.DataLoader(self.val_set, batch_size=self.hparams['batch_size'])

    def configure_optimizers(self):

        optim = None
        self.model = nn.Sequential(self.encoder,self.decoder)
        # optim = torch.optim.SGD(self.model.parameters(), self.hparams["learning_rate"], momentum=0.9)
        optim = torch.optim.Adam(self.model.parameters(),self.hparams["learning_rate"],weight_decay = self.hparams['weight_decay'])
        return optim

    def getReconstructions(self, loader=None):
        self.eval()
        self = self.to(self.device)

        if not loader:
            loader = self.val_dataloader()

        reconstructions = []

        for batch in loader:
            X = batch
            X = X.to(self.device)
            flattened_X = X.view(X.shape[0], -1)
            reconstruction = self.forward(flattened_X)
            reconstructions.append(
                reconstruction.view(-1, 28, 28).cpu().detach().numpy())

        return np.concatenate(reconstructions, axis=0)
```



# 7.Facial Keypoint Detection

## 7.1加载库

```python
import os
import matplotlib.pyplot as plt
import torch
from torch.utils.data import DataLoader
from torchvision import transforms
from exercise_code.data.facial_keypoints_dataset import FacialKeypointsDataset
from exercise_code.networks.keypoint_nn import (
    DummyKeypointModel,
    KeypointModel
)
from exercise_code.util import (
    show_all_keypoints,
    save_model,
)
from exercise_code.tests import test_keypoint_nn

%matplotlib inline
plt.rcParams['figure.figsize'] = (10.0, 8.0) # set default size of plots
plt.rcParams['image.interpolation'] = 'nearest'
plt.rcParams['image.cmap'] = 'gray'

# for auto-reloading external modules
# see http://stackoverflow.com/questions/1907993/autoreload-of-modules-in-ipython
%load_ext autoreload
%autoreload 2

os.environ['KMP_DUPLICATE_LIB_OK']='True'
```

## 7.2加载数据集

```python
download_url = 'https://vision.in.tum.de/webshare/g/i2dl/facial_keypoints.zip'
i2dl_exercises_path = os.path.dirname(os.path.abspath(os.getcwd()))
data_root = os.path.join(i2dl_exercises_path, "datasets", "facial_keypoints")
train_dataset = FacialKeypointsDataset(
    train=True,
    transform=transforms.ToTensor(),
    root=data_root,
    download_url=download_url
)
val_dataset = FacialKeypointsDataset(
    train=False,
    transform=transforms.ToTensor(),
    root=data_root,
)

print("Number of training samples:", len(train_dataset))
print("Number of validation samples:", len(val_dataset))
```

上面用到的FacialKeypointsDataset()

```python
class FacialKeypointsDataset(BaseDataset):
    """Dataset for facial keypoint detection"""
    def __init__(self, *args, train=True, transform=None, **kwargs):
        super().__init__(*args, **kwargs)
        file_name = "training.csv" if train else "val.csv"
        csv_file = os.path.join(self.root_path, file_name)
        self.key_pts_frame = pd.read_csv(csv_file)
        self.key_pts_frame.dropna(inplace=True)
        self.key_pts_frame.reset_index(drop=True, inplace=True)
        self.transform = transform

    @staticmethod
    def _get_image(idx, key_pts_frame):
        img_str = key_pts_frame.loc[idx]['Image']
        img = np.array([
            int(item) for item in img_str.split()
        ]).reshape((96, 96))
        return np.expand_dims(img, axis=2).astype(np.uint8)

    @staticmethod
    def _get_keypoints(idx, key_pts_frame, shape=(15, 2)):
        keypoint_cols = list(key_pts_frame.columns)[:-1]
        key_pts = key_pts_frame.iloc[idx][keypoint_cols].values.reshape(shape)
        key_pts = (key_pts.astype(np.float) - 48.0) / 48.0
        return torch.from_numpy(key_pts).float()

    def __len__(self):
        return self.key_pts_frame.shape[0]

    def __getitem__(self, idx):
        image = self._get_image(idx, self.key_pts_frame)
        keypoints = self._get_keypoints(idx, self.key_pts_frame)
        if self.transform:
            image = self.transform(image)
        return {'image': image, 'keypoints': keypoints}
```

Each sample in our dataset is a dict `{"image": image, "keypoints": keypoints}`, where
 * `image` is a [0-1]-normalized gray-scale image of size 96x96, represented by a torch tensor of shape (CxHxW) with C=1, H=96, W=96
    <img style="float: right;" src='https://github.com/Fernweh-yang/Reading-Notes/blob/main/%E7%AC%94%E8%AE%B0%E9%85%8D%E5%A5%97%E5%9B%BE%E7%89%87/Deep%20learning/key_pts_expl.png?raw=true' width=50% height=50%/>
 * `keypoints` is the list of K facial keypoints, stored in a torch tensor of shape (Kx2). We have K=15 keypoints that stand for:
   * keypoints[0]: Center of the left eye
   * keypoints[1]: Center of the right eye
   * keypoints[2]: Left eye inner corner
   * keypoints[3]: Left eye outer corner
   * keypoints[4]: Right eye inner corner
   * keypoints[5]: Right eye outer corner
   * keypoints[6]: Left eyebrow inner end
   * keypoints[7]: Left eyebrow outer end
   * keypoints[8]: Right eyebrow inner end
   * keypoints[9]: Right eyebrow outer end
   * keypoints[10]: Nose tip
   * keypoints[11]: Mouth left corner
   * keypoints[12]: Mouth right corner
   * keypoints[13]: Mouth center top lip
   * keypoints[14]: Mouth center bottom lip
   

Each individual facial keypoint is represented by two coordinates (x,y) that specify the horizontal and vertical location of the keypoint respectively. All keypoint values are normalized to [-1,1], such that:
   * (x=-1,y=-1) corresponds to the top left corner, 
   * (x=-1,y=1) to the bottom left corner,
   * (x=1,y=-1) to the top right corner,
   * (x=1,y=1) to the bottom right corner,
   * and (x=0,y=0) to the center of the image.

显示关键点和图片的储存信息：

```python
#--------------显示图片信息--------------
image, keypoints = train_dataset[0]["image"], train_dataset[0]["keypoints"]
print("Shape of the image:", image.size())
print("Smallest value in the image:", torch.min(image))
print("Largest value in the image:", torch.max(image))
print(image)
#--------------显示关键点信息--------------
keypoints = train_dataset[0]["keypoints"]
print(keypoints)
#--------------任取几张图片，显示图片上所有的关键点--------------
def show_all_keypoints(image, keypoints, pred_kpts=None):
    """Show image with predicted keypoints"""
    image = (image.clone() * 255).view(96, 96).cpu()
    plt.imshow(image, cmap='gray')

    if pred_kpts is not None:
      pred_kpts = pred_kpts.cpu()
      
    keypoints = (keypoints.clone() * 48 + 48).cpu()
    plt.scatter(keypoints[:, 0], keypoints[:, 1], s=200, marker='.', c='m')
    if pred_kpts is not None:
        pred_kpts = pred_kpts.clone() * 48 + 48
        plt.scatter(pred_kpts[:, 0], pred_kpts[:, 1], s=200, marker='.', c='r')
    plt.show()
def show_keypoints(dataset, num_samples=3):
    for i in range(num_samples):
        image = dataset[i]["image"]
        key_pts = dataset[i]["keypoints"]
        show_all_keypoints(image, key_pts)
show_keypoints(train_dataset)
```

## 7.3 模型：

facial keypoint detection task可以被看做一个回归regression问题，目标是预测对应于15个面部关键点位置的30个不同值。

### 7.3.1 Loss and Metrics

使用Mean Squared Error[均方误差](https://zh.wikipedia.org/wiki/%E5%9D%87%E6%96%B9%E8%AF%AF%E5%B7%AE)来评价模型。

```python
# pytorch已经定义好了均方误差
loss_fn = torch.nn.MSELoss()
for i in range(3):
    image = train_dataset[i]["image"]
    keypoints = train_dataset[i]["keypoints"]
    predicted_keypoints = torch.squeeze(dummy_model(image)).view(15,2)
    loss = loss_fn(keypoints, predicted_keypoints)
    print("Loss on image %d:" % i, loss)
    
def evaluate_model(model, dataset):
    model.eval()
    model.to(device)
    criterion = torch.nn.MSELoss()
    dataloader = DataLoader(dataset, batch_size=1, shuffle=False)
    loss = 0
    for batch in dataloader:
        image, keypoints = batch["image"].to(device), batch["keypoints"].to(device)
        predicted_keypoints = model(image).view(-1,15,2).to(device)
        loss += criterion(
            torch.squeeze(keypoints),
            torch.squeeze(predicted_keypoints)
        ).item()
    return 1.0 / (2 * (loss/len(dataloader)))

print("Score of the Dummy Model:", evaluate_model(dummy_model, val_dataset))
```

# 8. Semantic Segmentation语义分割

[参考](https://www.jeremyjordan.me/semantic-segmentation/)

## 8.1 加载库

```python
import sys
import numpy as np
import matplotlib.pyplot as plt
import os
import torch

from exercise_code.data.segmentation_dataset import SegmentationData, label_img_to_rgb
from exercise_code.data.download_utils import download_dataset
from exercise_code.util import visualizer, save_model
from exercise_code.util.Util import checkSize, checkParams, test
from exercise_code.networks.segmentation_nn import SegmentationNN, DummySegmentationModel
from exercise_code.tests import test_seg_nn
#set up default cuda device
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

%matplotlib inline
plt.rcParams['figure.figsize'] = (10.0, 8.0) # set default size of plots
plt.rcParams['image.interpolation'] = 'nearest'
plt.rcParams['image.cmap'] = 'gray'

# for auto-reloading external modules
# see http://stackoverflow.com/questions/1907993/autoreload-of-modules-in-ipython
%load_ext autoreload
%autoreload 2
# 使用Tensorboard
%load_ext tensorboard
%tensorboard --logdir lightning_logs --port 6006
```

## 8.2 加载数据集

### 8.2.0 下载数据集

```python
download_url = 'https://i2dl.dvl.in.tum.de/downloads/segmentation_data.zip'
i2dl_exercises_path = os.path.dirname(os.path.abspath(os.getcwd()))
data_root = os.path.join(i2dl_exercises_path, 'datasets','segmentation')


download_dataset(
    url=download_url,
    data_dir=data_root,
    dataset_zip_name='segmentation_data.zip',
    force_download=False,
)

train_data = SegmentationData(image_paths_file=f'{data_root}/segmentation_data/train.txt')
val_data = SegmentationData(image_paths_file=f'{data_root}/segmentation_data/val.txt')
test_data = SegmentationData(image_paths_file=f'{data_root}/segmentation_data/test.txt')
```

### 8.2.1 用到的download_dataset()

```python
import os
import shutil
import urllib
import tarfile
import zipfile
import gzip
import tqdm


def gen_bar_updater():
    """tqdm report hook for urlretrieve"""
    pbar = tqdm.tqdm(total=None)

    def bar_update(count, block_size, total_size):
        if pbar.total is None and total_size:
            pbar.total = total_size
        progress_bytes = count * block_size
        pbar.update(progress_bytes - pbar.n)

    return bar_update


def download_url(url, root, filename):
    """
    Download a file with given filename from a given url to a given directory
    :param url: url from where to download
    :param root: root directory to which to download
    :param filename: filename under which the file should be saved
    """
    file_path = os.path.join(root, filename)
    os.makedirs(root, exist_ok=True)
    if not os.path.exists(file_path):
        print('Downloading ' + url + ' to ' + file_path)
        urllib.request.urlretrieve(
            url,
            file_path,
            reporthook=gen_bar_updater()
        )
    return file_path


def _is_tarxz(filename):
    return filename.endswith(".tar.xz")


def _is_tar(filename):
    return filename.endswith(".tar")


def _is_targz(filename):
    return filename.endswith(".tar.gz")


def _is_tgz(filename):
    return filename.endswith(".tgz")


def _is_gzip(filename):
    return filename.endswith(".gz") and not filename.endswith(".tar.gz")


def _is_zip(filename):
    return filename.endswith(".zip")


def extract_archive(from_path, to_path=None, remove_finished=False):
    """
    Extract a given archive
    :param from_path: path to archive which should be extracted
    :param to_path: path to which archive should be extracted
        default: parent directory of from_path
    :param remove_finished: if set to True, delete archive after extraction
    """
    if not os.path.exists(from_path):
        return

    if to_path is None:
        to_path = os.path.dirname(from_path)

    if _is_tar(from_path):
        with tarfile.open(from_path, 'r') as tar:
            tar.extractall(path=to_path)
    elif _is_targz(from_path) or _is_tgz(from_path):
        with tarfile.open(from_path, 'r:gz') as tar:
            tar.extractall(path=to_path)
    elif _is_tarxz(from_path):
        with tarfile.open(from_path, 'r:xz') as tar:
            tar.extractall(path=to_path)
    elif _is_gzip(from_path):
        to_path = os.path.join(
            to_path,
            os.path.splitext(os.path.basename(from_path))[0]
        )
        with open(to_path, "wb") as out_f, gzip.GzipFile(from_path) as zip_f:
            out_f.write(zip_f.read())
    elif _is_zip(from_path):
        with zipfile.ZipFile(from_path, 'r') as zip_:
            zip_.extractall(to_path)
    else:
        raise ValueError("Extraction of {} not supported".format(from_path))

    if remove_finished:
        os.remove(from_path)


def download_dataset(url, data_dir, dataset_zip_name, force_download=False):
    """
    Download dataset
    :param url: URL to download file from
    :param data_dir: Base name of the current dataset directory
    :param dataset_zip_name: Name of downloaded compressed dataset file
    :param force_download: If set to True, always download dataset
        (even if it already exists)
    """
    if not os.path.exists(data_dir) or not os.listdir(data_dir) or force_download:
        if os.path.exists(data_dir):
            shutil.rmtree(data_dir)
        data_file = download_url(url, data_dir, dataset_zip_name)
        extract_archive(data_file, remove_finished=True)
```

### 8.2.2 用到的segmentationData()

- Each segmentation label has its corresponding RGB value stored in the `SEG_LABELS_LIST`. 
  - The label `void` means `unlabeled`, and it is displayed as black `"rgb_values": [0, 0, 0]` in the target image. 
  - The target image pixels will be labeled based on its color using `SEG_LABELS_LIST`.

```python
"""Data utility functions."""
import os

import numpy as np
import torch
import torch.utils.data as data
from PIL import Image
from torchvision import transforms

import _pickle as pickle

# pylint: disable=C0326
SEG_LABELS_LIST = [
    {"id": -1, "name": "void",       "rgb_values": [0,   0,    0]},
    {"id": 0,  "name": "building",   "rgb_values": [128, 0,    0]},
    {"id": 1,  "name": "grass",      "rgb_values": [0,   128,  0]},
    {"id": 2,  "name": "tree",       "rgb_values": [128, 128,  0]},
    {"id": 3,  "name": "cow",        "rgb_values": [0,   0,    128]},
    {"id": 4,  "name": "horse",      "rgb_values": [128, 0,    128]},
    {"id": 5,  "name": "sheep",      "rgb_values": [0,   128,  128]},
    {"id": 6,  "name": "sky",        "rgb_values": [128, 128,  128]},
    {"id": 7,  "name": "mountain",   "rgb_values": [64,  0,    0]},
    {"id": 8,  "name": "airplane",   "rgb_values": [192, 0,    0]},
    {"id": 9,  "name": "water",      "rgb_values": [64,  128,  0]},
    {"id": 10, "name": "face",       "rgb_values": [192, 128,  0]},
    {"id": 11, "name": "car",        "rgb_values": [64,  0,    128]},
    {"id": 12, "name": "bicycle",    "rgb_values": [192, 0,    128]},
    {"id": 13, "name": "flower",     "rgb_values": [64,  128,  128]},
    {"id": 14, "name": "sign",       "rgb_values": [192, 128,  128]},
    {"id": 15, "name": "bird",       "rgb_values": [0,   64,   0]},
    {"id": 16, "name": "book",       "rgb_values": [128, 64,   0]},
    {"id": 17, "name": "chair",      "rgb_values": [0,   192,  0]},
    {"id": 18, "name": "road",       "rgb_values": [128, 64,   128]},
    {"id": 19, "name": "cat",        "rgb_values": [0,   192,  128]},
    {"id": 20, "name": "dog",        "rgb_values": [128, 192,  128]},
    {"id": 21, "name": "body",       "rgb_values": [64,  64,   0]},
    {"id": 22, "name": "boat",       "rgb_values": [192, 64,   0]}]


def label_img_to_rgb(label_img):
    label_img = np.squeeze(label_img)
    labels = np.unique(label_img)
    label_infos = [l for l in SEG_LABELS_LIST if l['id'] in labels]

    label_img_rgb = np.array([label_img,
                              label_img,
                              label_img]).transpose(1,2,0)
    for l in label_infos:
        mask = label_img == l['id']
        label_img_rgb[mask] = l['rgb_values']

    return label_img_rgb.astype(np.uint8)


class SegmentationData(data.Dataset):

    def __init__(self, image_paths_file):
        self.root_dir_name = os.path.dirname(image_paths_file)

        with open(image_paths_file) as f:
            self.image_names = f.read().splitlines()

    def __getitem__(self, key):
        if isinstance(key, slice):
            # get the start, stop, and step from the slice
            return [self[ii] for ii in range(*key.indices(len(self)))]
        elif isinstance(key, int):
            # handle negative indices
            if key < 0:
                key += len(self)
            if key < 0 or key >= len(self):
                raise IndexError("The index (%d) is out of range." % key)
            # get the data from direct index
            return self.get_item_from_index(key)
        else:
            raise TypeError("Invalid argument type.")

    def __len__(self):
        return len(self.image_names)

    def get_item_from_index(self, index):
        to_tensor = transforms.ToTensor()
        img_id = self.image_names[index].replace('.bmp', '')

        img = Image.open(os.path.join(self.root_dir_name,
                                      'images',
                                      img_id + '.bmp')).convert('RGB')
        center_crop = transforms.CenterCrop(240)
        img = center_crop(img)
        img = to_tensor(img)

        target = Image.open(os.path.join(self.root_dir_name,
                                         'targets',
                                         img_id + '_GT.bmp'))
        target = center_crop(target)
        target = np.array(target, dtype=np.int64)

        target_labels = target[..., 0]
        for label in SEG_LABELS_LIST:
            mask = np.all(target == label['rgb_values'], axis=2)
            target_labels[mask] = label['id']

        target_labels = torch.from_numpy(target_labels.copy())

        return img, target_labels

```



## 8.3 展示数据

```python
print("Train size: %i" % len(train_data))
print("Validation size: %i" % len(val_data))
print("Img size: ", train_data[0][0].size())
print("Segmentation size: ", train_data[0][1].size())

num_example_imgs = 4
plt.figure(figsize=(10, 5 * num_example_imgs))
for i, (img, target) in enumerate(train_data[:num_example_imgs]):
    # img
    plt.subplot(num_example_imgs, 2, i * 2 + 1)
    plt.imshow(img.numpy().transpose(1,2,0))
    plt.axis('off')
    if i == 0:
        plt.title("Input image")
    
    # target
    plt.subplot(num_example_imgs, 2, i * 2 + 2)
    plt.imshow(label_img_to_rgb(target.numpy()))
    plt.axis('off')
    if i == 0:
        plt.title("Target image")
plt.show()
```

## 8.4 Loss and Metrics度量

- The loss function for the task of image segmentation is a **pixel-wise cross entropy loss.** This loss examines each pixel individually, comparing the class predictions (depth-wise pixel vector) to our one-hot encoded target vector.

  ![](https://www.jeremyjordan.me/content/images/2018/05/Screen-Shot-2018-05-24-at-10.46.16-PM.png)

  Up until now we only used the default loss function (`nn.CrossEntropyLoss`) in our solvers. However, In order to ignore the `unlabeled` pixels for the computation of our loss, we have to use a customized version of the loss for the initialization of our segmentation solver. The `ignore_index` argument of the loss can be used to filter the `unlabeled` pixels and computes the loss only over remaining pixels.

  ```python
  loss_func = torch.nn.CrossEntropyLoss(ignore_index=-1, reduction='mean')
  
  for (inputs, targets) in train_data[0:4]:
      inputs, targets = inputs, targets
      outputs = dummy_model(inputs.unsqueeze(0))
      losses = loss_func(outputs, targets.unsqueeze(0))
      print(losses)
  ```

  - there is `softmax` function in the `nn.CrossEntropyLoss` function, the loss is:      $$loss(x, class) = - \log \left( \frac{\exp(x[class])}{\Sigma_j \exp (x[j])} \right) = −x[class]+\log \left( \Sigma_j \exp(x[j]) \right)$$
    and the loss will not be zero.  

  - To obtain an evaluation accuracy, we can simply compute the average per pixel accuracy of our network for a given image. We will use the following function

    ```python
    def evaluate_model(model, dataloader):
        test_scores = []
        model.eval()
        for inputs, targets in dataloader:
            inputs, targets = inputs.to(device), targets.to(device)
    
            outputs = model.forward(inputs)
            _, preds = torch.max(outputs, 1)
            targets_mask = targets >= 0
            test_scores.append(np.mean((preds.cpu() == targets.cpu())[targets_mask].numpy()))
    
        return np.mean(test_scores)
    
    test_loader = torch.utils.data.DataLoader(test_data, batch_size=1,shuffle=False,num_workers=0)
    print(evaluate_model(dummy_model, test_loader))
    ```

## 8.5 使用transfer learning来设计model

PyTorch有一些[pretrained models](https://pytorch.org/vision/stable/models.html)。用他们来做transfer learning

### 8.5.1 使用PyTorch Lightning设计model

```python
import torch
import torch.nn as nn
import pytorch_lightning as pl
# 从pytorch加载pretrained models
from torchvision import models

class SegmentationNN(pl.LightningModule):

    def __init__(self, num_classes=23, hparams=None):
        super().__init__()
        self.save_hyperparameters(hparams)
        # 使用MobileNet V2 model
        # .eval()将模型设置为评估模式
        pretrained = models.mobilenet_v2(pretrained=True, progress=True).eval()
		# 冻结MobileNet V2模型，不修改它的参数
        for param in pretrained.parameters():
            param.requires_grad = False
            
		# 设计模型
        self.model = nn.Sequential(
          # [:-1]为剔除列表最后一个数字，这里去除原模型的全连接层？
          # model.children()获得一个模型的神经网络结构
          *(list(pretrained.children())[:-1]),
          # mobilenet_v2 output is 1280-dimensional
          nn.ConvTranspose2d(1280, self.hparams['channel_1'], kernel_size=3,stride=2),
          nn.ConvTranspose2d(self.hparams['channel_1'], self.hparams['channel_2'], kernel_size=3,stride=2),
          nn.ConvTranspose2d(self.hparams['channel_2'], self.hparams['channel_3'], 1),
          torch.nn.Upsample(size=(240, 240)),
        )


    def forward(self, x):
        """
        Forward pass of the convolutional neural network. Should not be called
        manually but by calling a model instance directly.

        Inputs:
        - x: PyTorch input Variable
        """
        x = self.model(x)
        return x

    def general_step(self, batch, batch_idx, mode):
        images = batch[0]
        gt_category = batch[1]

        # forward pass
        predicted_category = self.forward(images)

        # loss
        loss = nn.CrossEntropyLoss(ignore_index=-1, reduction='mean')(predicted_category, gt_category)

        return loss

    def training_step(self, batch, batch_idx):
        loss  = self.general_step(batch, batch_idx, "train")
        tensorboard_logs = {'loss': loss}
        return {'loss': loss, 'log': tensorboard_logs}

    def validation_step(self, batch, batch_idx):
        loss  = self.general_step(batch, batch_idx, "val")
        return loss

    def train_dataloader(self):
        return torch.utils.data.DataLoader(self.train_set, shuffle=True, batch_size=self.hparams['batch_size'], drop_last=True, num_workers=16)

    def val_dataloader(self):
        return torch.utils.data.DataLoader(self.val_set, batch_size=self.hparams['batch_size'],  drop_last=True, num_workers=16)

    def configure_optimizers(self):
        optim = torch.optim.Adam(self.parameters(), self.hparams["learning_rate"])
        return optim

    @property
    def is_cuda(self):
        """
        Check if model parameters are allocated on the GPU.
        """
        return next(self.parameters()).is_cuda

    def save(self, path):
        """
        Save model with its parameters to the given path. Conventionally the
        path should end with "*.model".

        Inputs:
        - path: path string
        """
        print('Saving model... %s' % path)
        torch.save(self, path)

```

### 8.5.2 设置超参

```python
hparams = {
    # TODO: if you have any model arguments/hparams, define them here and read them from this dict inside SegmentationNN class
    'learning_rate': 0.001,
    'batch_size': 128,
    'channel_1': 152,
    'channel_2': 64,
    'channel_3': 23
}
```

### 8.5.3训练模型

```python
model = SegmentationNN(hparams=hparams)
model.to(device)

import pytorch_lightning as pl
from pytorch_lightning.callbacks.early_stopping import EarlyStopping
early_stop_callback = EarlyStopping(
   monitor='val_loss',
   patience=10
)
train_dataloader= torch.utils.data.DataLoader(train_data, batch_size=hparams['batch_size'], shuffle=True, num_workers=16)
val_dataloader=torch.utils.data.DataLoader(val_data, batch_size=hparams['batch_size'], shuffle=False, num_workers=16)

# trainer = pl.Trainer(logger=ae_logger,accelerator='dp', max_epochs=75,progress_bar_refresh_rate = 10,callbacks=[early_stop_callback]) 
trainer = pl.Trainer(
    max_epochs=50,
    accelerator = 'gpu'
) 
                 
trainer.fit(model,train_dataloader,val_dataloader)

```

## 8.6 测试模型

```python
def test(acc):

    print("Validation-Accuracy: {}%".format(acc*100))
    if acc < ACC_THRESHOLD:
        print("That's too low! Please tune your model in order to reach at least {}% before running on the test set and submitting!".format(ACC_THRESHOLD * 100))

    else:
        print("Congrats! The accuracy passes the threshold, you can try to submit your model to server now.")

# evaluate_mode()在8.4中
test(evaluate_model(model, test_loader))
```

## 8.7展示结果

```python
def visualizer(model, test_data=None):
    num_example_imgs = 4
    plt.figure(figsize=(15, 5 * num_example_imgs))
    for i, (img, target) in enumerate(test_data[:num_example_imgs]):
        inputs = img.unsqueeze(0)
        inputs = inputs.to(device)

        outputs = model.forward(inputs)
        _, preds = torch.max(outputs, 1)
        pred = preds[0].data.cpu()

        img, target, pred = img.numpy(), target.numpy(), pred.numpy()

        # img
        plt.subplot(num_example_imgs, 3, i * 3 + 1)
        plt.axis('off')
        plt.imshow(img.transpose(1, 2, 0))
        if i == 0:
            plt.title("Input image")

        # target
        plt.subplot(num_example_imgs, 3, i * 3 + 2)
        plt.axis('off')
        plt.imshow(label_img_to_rgb(target))
        if i == 0:
            plt.title("Target image")

        # pred
        plt.subplot(num_example_imgs, 3, i * 3 + 3)
        plt.axis('off')
        plt.imshow(label_img_to_rgb(pred))
        if i == 0:
            plt.title("Prediction image")

    plt.show()
```

