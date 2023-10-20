# 1.安装conda

1. 下载最新版本conda:https://docs.conda.io/en/latest/miniconda.html
2. 给予权限：`chmod u+x Miniconda3-latest-Linux-x86_64.sh `
3. 安装：`./Miniconda3-latest-Linux-x86_64.sh`

下面的安装都在conda环境中安装

# 2.安装Nvidia驱动

## 2.1 卸载旧驱动

1. `sudo apt-get --purge remove nvidia*`
2. `sudo apt autoremove`

## 2.2 安装新区东

- 注意安装的版本

  要和下面的cudnn/cuda版本相匹配

  比如这里的driver:515+cuda:11.7+cudnn:8.5

- 两种方法安装驱动：

  1. "Software & Updates": Additional Drivers

     1. choose on driver to install automatically
     2. `reboot`

  2. install driver from [official website](https://www.nvidia.cn/Download/index.aspx?lang=cn)

     1. diable Nouveau kernel driver

        输入`lsmod | grep nouveau`如果无返回，就是已经禁用了

        - `sudo gedit /etc/modprobe.d/blacklist-nouveau.conf`

          add the following contents
     
          ```
          blacklist nouveau
          options nouveau modeset=0
          ```

        - `sudo update-initramfs -u`

        - `reboot`

     2. install the latest driver

        `https://www.nvidia.cn/Download/index.aspx?lang=cn#`

        - `sudo chmod +x NVIDIA-Linux-x86_64-515.76.run `
     
        - `sudo ./NVIDIA-Linux-x86_64-515.76.run `

# 3.安装cuda

- 注意版本要和nvidia匹配

- 从https://developer.nvidia.com/cuda-toolkit-archive选择对应的版本后得到下载路径：

  - `wget https://developer.download.nvidia.com/compute/cuda/11.7.1/local_installers/cuda_11.7.1_515.65.01_linux.run``

  - ``sudo sh cuda_11.7.1_515.65.01_linux.run`

  - add followings to .bashrc

    ```
    export PATH=$PATH:/usr/local/cuda/bin  
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64  
    export LIBRARY_PATH=$LIBRARY_PATH:/usr/local/cuda/lib64
    ```

# 4.安装cudnn

- 注意版本要和nvidia和cuda匹配
- install pkg from https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html
- `sudo dpkg -i cudnn-local-repo-ubuntu2004-8.5.0.96_1.0-1_amd64.deb `
- `sudo cp /var/cudnn-local-repo-ubuntu2004-8.5.0.96/cudnn-local-0579404E-keyring.gpg /usr/share/keyrings/`
- `sudo apt-get update`
- `sudo apt-get install libcudnn8=8.5.0.96-1+cuda11.7`
- `sudo apt-get install libcudnn8-dev=8.5.0.96-1+cuda11.7`
- `sudo apt-get install libcudnn8-samples=8.5.0.96-1+cuda11.7`

# 5.安装pytorch

- 根据官网得到安装命令：

  `conda install pytorch torchvision torchaudio pytorch-cuda=11.7 -c pytorch -c nvidia`

## PyTorch教程汇总

- 官网教程（本笔记）：https://pytorch.org/tutorials/beginner/basics/intro.html

## PyTorch安装

- Cuda和cuDNN安装

  - [PyTorch](https://pytorch.org/get-started/locally/)官网查看需要的Cuda版本
  - [Cuda](https://developer.nvidia.com/cuda-toolkit-archive),下载对应的版本
  - [cuDNN](https://developer.nvidia.com/rdp/cudnn-download),下载后解压到cuda[安装的路径](C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v11.3)下

- Windows CuDNN版本卸载：

  - `C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA`删除版本

  - app管理中卸载版本对应的驱动

  - 环境变量中删除

- PyTorch安装

  - [PyTorch](https://pytorch.org/get-started/locally/)官网，查看安装指令
  - 在Conda终端中安装
  - 在需要使用的conda环境下载入安装好的pytorch`conda install pytorch torchvision -c pytorch`

- 查看版本（检查是否安装成功）：

  - PyTorch:

    可以直接查看torch,cuda,cudnn的版本

    ```python
    import torch
    print(torch.__version__) 
    print(torch.version.cuda)
    print(torch.backends.cudnn.version())
    ```

  - Cuda:

    - 命令行：`nvcc --version`

    - nvidia driver:`nvidia-smi`

      

导入库：

```
import torch
import numpy as np
```

## PyTorch基本使用

### 一、Tensors

Tensor是一个和numpy的ndarray类似的数据结构，可以运行在GPU或其他硬件加速hardware accelerators上。 In fact, tensors and NumPy arrays can often share the same underlying memory底层内存。

#### 1.1 Initializing a Tensor

- **Directly from data**

  ```python
  data = [[1, 2],[3, 4]]
  x_data = torch.tensor(data)
  ```

- **From a NumPy array**

  ```python
  np_array = np.array(data)
  ts_array = torch.from_numpy(np_array)
  ```

  也可以从Tensor转为array

  ```
  np_array_2 = ts_array.numpy() 
  ```

- **From another tensor:**

  如果不特别定义参数，会保留原tensor的shape和datatype

  ```python
  x_ones = torch.ones_like(x_data) # retains the properties of x_data
  print(f"Ones Tensor: \n {x_ones} \n")
  
  x_rand = torch.rand_like(x_data, dtype=torch.float) # overrides the datatype of x_data
  print(f"Random Tensor: \n {x_rand} \n")
  ```

- **With random or constant values:**

  ```python
  shape = (2,3,)
  rand_tensor = torch.rand(shape) # = torch.rand(2,3,)
  ones_tensor = torch.ones(shape)
  zeros_tensor = torch.zeros(shape)
  ```

#### 1.2 Attributes of a Tensor

- Tensor attributes describe their shape, datatype, and the device on which they are stored.

  ```python
  tensor = torch.rand(3,4)
  
  print(f"Shape of tensor: {tensor.shape}")
  print(f"Datatype of tensor: {tensor.dtype}")
  print(f"Device tensor is stored on: {tensor.device}")
  ```

  out:

  ```
  Shape of tensor: torch.Size([3, 4])
  Datatype of tensor: torch.float32
  Device tensor is stored on: cpu
  ```

#### 1.3 Operations on Tensors

Tensor所有提供的[Operations](https://pytorch.org/docs/stable/torch.html#math-operations)

- **在GPU中计算**

  通常Tensor在cpu中被建立，我们需要将他们移到gpu中保存并计算

  ```python
  # We move our tensor to the GPU if available
  # tensor这里是一个Tensor数据类型的变量名，tensor.to(‘cuda’)只是把tensor这一个变量转到了gpu中保存
  # 如果另一个Tensor数据类型的变量xxx也想转到gpu，需要再输入xxx.to('cuda')
  if torch.cuda.is_available():
    tensor = tensor.to('cuda')
  # 可以再x.to('cpu')转回到cpu存储
  tensor = tensor.to('cpu')
  ```

- **类似numpy.ndarray的索引indexing和切片slicing操作**

  ```python
  tensor = torch.ones(4, 4)
  print('First row: ',tensor[0])
  print('First column: ', tensor[:, 0])
  print('Last column:', tensor[..., -1])
  tensor[:,1] = 0
  print(tensor)
  
  # out
  """
  First row:  tensor([1., 1., 1., 1.])
  First column:  tensor([1., 1., 1., 1.])
  Last column: tensor([1., 1., 1., 1.])
  tensor([[1., 0., 1., 1.],
          [1., 0., 1., 1.],
          [1., 0., 1., 1.],
          [1., 0., 1., 1.]])
  """
  ```

- **将多个Tensor变量整合成一个**

  ```python
  tensor = torch.ones(4, 4)
  t1 = torch.cat([tensor, tensor, tensor], dim=1)
  print(t1)
  
  # out
  """
  tensor([[1., 0., 1., 1., 1., 0., 1., 1., 1., 0., 1., 1.],
          [1., 0., 1., 1., 1., 0., 1., 1., 1., 0., 1., 1.],
          [1., 0., 1., 1., 1., 0., 1., 1., 1., 0., 1., 1.],
          [1., 0., 1., 1., 1., 0., 1., 1., 1., 0., 1., 1.]])
  """
  ```

- **PyTorch中的乘法**：

  torch.matmul()和torch.mul()都会广播

  - 如果维度个数不同，则在维度较少的左边补1，使得维度的个数相同。
  - 各维度的维度大小不同时，如果有维度为1的，直接将该维拉伸至维度相同

  torch.mm()是torch.matmul()的不广播形式

  torch.dot(a,b)是向量相乘，所以输入必须是一维的

  ```python
  #————————————————————————————————————————————————————————
  # This computes the matrix multiplication between two tensors. y1, y2, y3 will have the same value
  # @运算符：映射torch.matmul()函数，是矩阵乘法
  y1 = tensor @ tensor.T
  y2 = tensor.matmul(tensor.T)
  # 也可以先建一个Tensor大小的变量(y3)，然后将计算结果存在其中
  y3 = torch.rand_like(tensor)
  torch.matmul(tensor, tensor.T, out=y3)
  
  #————————————————————————————————————————————————————————
  # torch.mm()是torch.matmul()的不广播形式
  # 即要求必须满足矩阵相乘维数条件
  y4 = tensor.mm(tensor)
  
  #————————————————————————————————————————————————————————
  # This computes the element-wise product. z1, z2, z3 will have the same value
  # *运算符：映射mul()函数，是2矩阵对应元素element-wise两两相乘
  z1 = tensor * tensor
  z2 = tensor.mul(tensor)
  z3 = torch.rand_like(tensor)
  torch.mul(tensor, tensor, out=z3)
  
  #————————————————————————————————————————————————————————
  #torch.dot(a,b)是向量相乘，所以输入必须是一维的
  a = torch.tensor([2, 3])
  b = torch.tensor([1,2])
  c = torch.dot(a,b)
  ```

- **将Single-element tensors转为python值**

  If you have a one-element tensor, for example by aggregating合计 all values of a tensor into one value, you can convert it to a Python numerical value using `item()`

  ```python
  agg = tensor.sum()
  agg_item = agg.item()
  print(agg_item, type(agg_item))
  
  # out
  """
  12.0 <class 'float'>
  """
  ```

#### 1.4 Bridge with Numpy

- 相互转换：

  ```python
  data = [[1, 2],[3, 4]]
  #从numpy转为tensor
  np_array = np.array(data)
  ts_array = torch.from_numpy(np_array)
  #从tensor转为numpy
  np_array_2 = ts_array.numpy() 
  ```

- 相互转换而来的Tensor在cpu和Numpy是共用一个内存underlying memory locations的，所以改变其中一个值，另一个也会改变。

  - 比如上面改变np_array, np_array_2, ts_array任何一个，其他都会跟着变。

    ```python
    # 改变ts_array，np_array和np_array_2也会变
    ts_array.add_(1)
    # 改变np_array, ts_array和np_array_2也会变
    np.add(np_array, 1, out=np_array)
    ```



### 二、Datasets & DataLoaders

- PyTorch提供了两个数据类来帮助模块化处理数据：

  - `torch.utils.data.DataLoader`
    - `DataLoader` wraps an iterable around the `Dataset` to enable easy access to the samples.
  - `torch.utils.data.Datase`
    -  `Dataset` stores the samples and their corresponding labels

- PyTorch也提供各种已经预加载的数据集

  [Image Datasets](https://pytorch.org/vision/stable/datasets.html), [Text Datasets](https://pytorch.org/text/stable/datasets.html), and [Audio Datasets](https://pytorch.org/audio/stable/datasets.html)

#### 2.1 Loading a Dataset

- 下面的例子是从TorchVision(一个拥有众多内置图像数据集的模块)中下载Fashion-MNIST数据集。

  - Fashion-MNIST is a dataset of Zalando’s article images consisting of 60,000 training examples and 10,000 test examples. Each example comprises a 28×28 grayscale image and an associated label from one of 10 classes.

    ```python
    import torch
    from torch.utils.data import Dataset
    from torchvision import datasets
    from torchvision.transforms import ToTensor
    import matplotlib.pyplot as plt
    
    training_data = datasets.FashionMNIST(
        # "root" is the path where the train/test data is stored,
        root="data",
        # "train" specifies training or test dataset,
        train=True,
        # "download=True" downloads the data from the internet if it’s not available at root
        download=True,
        # "transform" and "target_transform" specify the feature and label transformations
        transform=ToTensor()
    )
    
    test_data = datasets.FashionMNIST(
        root="data",
        train=False,
        download=True,
        transform=ToTensor()
    )
    ```

- `transforms.Compose` creates a series of transformation to prepare the dataset.

  - `transforms.ToTensor` convert `PIL image` or numpy.ndarray $(H \times W\times C)$ in the range [0,255] to a `torch.FloatTensor` of shape $(C \times H \times W)$ in the range [0.0, 1.0].
  - `transforms.Normalize` normalize a tensor image with the provided mean and standard deviation标准偏差.

  ```python
  # Mean and standard deviations have to be sequences (e.g. tuples),hence we add a comma after the values
  transform = transforms.Compose([transforms.ToTensor(),
                                  transforms.Normalize((0.5,),(0.5,))])
  
  # 相比于上面对数据预处理只用了ToTensor(),这里还进行了图像归一化
  training_data = torchvision.datasets.FashionMNIST(root='../datasets', train=True, download=True, transform=transform)
  
  test_data = torchvision.datasets.FashionMNIST(root='../datasets', train=False, download=True, transform=transform)
  ```

#### 2.2 Iterating and Visualizing the Dataset

- `torch.utils.data.Dataloader` takes our training data or test data with parameter `batch_size` and `shuffle`. 

  - The variable `batch_size` defines how many samples per batch to load. 
    - Dataloader可以帮助我们方便的划分barchsize
  - The variable `shuffle=True` makes the data reshuffled重新洗牌 at every epoch.
    - Dataloader可以在一个epoch训练完后，自动的打乱重排每一个batch

  ```python
  # 将数据集training_data和test_data划分各个batch，并得到各个样本的数据和标签
  training_dataloader = DataLoader(training_data, batch_size=8,, shuffle=True)
  test_dataloader = DataLoader(test_data, batch_size=8,, shuffle=True)
  
  classes = ('T-shirt/top', 'Trouser', 'Pullover', 'Dress', 'Coat',
             'Sandal', 'Shirt', 'Sneaker', 'Bag', 'Ankle boot')
  
  # 迭代training_data数据集,获得各个batch样本的大小，类型，标签
  # 注意都是对dataloader操作，不是对dataset
  for i, item in enumerate(training_dataloader):
      print('Batch {}'.format(i))
      image, label = item
      print(f"Datatype of Image: {type(image)}")
      print(f"Shape of the Image: {image.shape}")
      print(f"Label Values: {label}")
      # 只看第一个batch的
      if i+1 >= 1:
      break
      
  # 用matplotlib来看一些样本图片
  def imshow(img):
      img = img / 2 + 0.5 # unormalize
      npimg = img.numpy()
      plt.imshow(np.transpose(npimg, (1, 2, 0)))
      plt.show()
  # get some random training images
  # 注意都是对dataloader操作，不是对dataset
  dataiter = iter(training_dataloader)
  images, labels = dataiter.next()
  # show images
  imshow(torchvision.utils.make_grid(images))
  # print labels
  print(' '.join('%5s' % classes[labels[j]] for j in range(8)))
  ```

#### 2.3 Creating a Custom Dataset for your files

- 一个自定义的数据集类必须包含3个函数

  - `__init__`:initialize the directory containing the images, the annotations file, and both transforms
  - ` __len__ ` :returns the number of samples in our dataset
  - ` __getitem__`:loads and returns a sample from the dataset at the given index

  ```python
  import os
  import pandas as pd
  from torchvision.io import read_image
  
  class CustomImageDataset(Dataset):
      def __init__(self, annotations_file, img_dir, transform=None, target_transform=None):
          # their labels are stored separately in a CSV file 
          self.img_labels = pd.read_csv(annotations_file)
          # images are stored in a directory
          self.img_dir = img_dir
          self.transform = transform
          self.target_transform = target_transform
  
      def __len__(self):
          return len(self.img_labels)
  
      def __getitem__(self, idx):
          img_path = os.path.join(self.img_dir, self.img_labels.iloc[idx, 0])
          image = read_image(img_path)
          label = self.img_labels.iloc[idx, 1]
          if self.transform:
              image = self.transform(image)
          if self.target_transform:
              label = self.target_transform(label)
          return image, label
  ```

  

### 三、Build Model

- [TORCH.NN库](https://pytorch.org/docs/stable/nn.html#torch-nn)提供了所有用来的建立Neural Network的东西。

  -  [nn.Module](https://pytorch.org/docs/stable/generated/torch.nn.Module.html)是所有PyTorch模块Module的基类
  -  A neural network is a module itself that consists of other modules (layers).
     - 这种嵌套的结构使得building and managing complex architectures easily.

- 实例：

  ```python
  import os
  import torch
  from torch import nn
  from torch.utils.data import DataLoader
  from torchvision import datasets, transforms
  #-----------------------Get Device for Training-----------------------
  # 检查是否可以使用GPU
  device = "cuda" if torch.cuda.is_available() else "cpu"
  #-----------------------Define the Class-----------------------
  # 通过继承nn.Module来构建神经网络
  class NeuralNetwork(nn.Module):
      # 在__init__()中初始化神经网络层layers
      def __init__(self):
          # super().__init__让子类不止继承父类的方法，也能继承父类的属性，即父类__init__中的变量值
          super(NeuralNetwork, self).__init__()
          # nn.Flatten()默认参数时，是让输入参数降低一个维度
          self.flatten = nn.Flatten()
          # nn.Sequential()对模块进行封装，这里定义了5层hidden layer
          self.linear_relu_stack = nn.Sequential(
              # Affine layer仿射层：第一个变量是输入维度数量，第二个变量是输出特征数
              # 用于保存weights和biases
              nn.Linear(28*28, 512),
              # 激活层
              nn.ReLU(),
              nn.Linear(512, 512),
              nn.ReLU(),
              nn.Linear(512, 10),
          )
  	# 在forward()中实现对输入数据的操作
      def forward(self, x):
          # 将输入数据展平成1个维度
          x = self.flatten(x)
          # 向前传播
          logits = self.linear_relu_stack(x)
          return logits
  ```

- Model Parameters

  - Many layers inside a neural network are *parameterized*, i.e. have associated weights and biases that are optimized during training.

  - 迭代查看每一层的参数

    ```python
    # 创建我们的神经网络模型的一个实例model，并将它送到gpu
    model = NeuralNetwork().to(device)
    # 模拟一个图片作为输入
    X = torch.rand(1, 28, 28, device=device)
    # 计算输入，得到一个10维的Tensor
    logits = model(X)
    # 用nn.Softmax()来获得预测概率
    pred_probab = nn.Softmax(dim=1)(logits)
    # 返回维度1中最大值的index
    y_pred = pred_probab.argmax(1)
    ```

### 四、Autograd(Back Progagation)

- `torch.autograd` supports automatic computation of gradient for any computational graph.

  -  If we set the attribute `.requires_grad` of `torch.Tensor` as `True`, it tracks all operations applied on that tensor.
  -  Once all the computations are finished, the function `.backward()` computes the gradients into the `Tensor.grad` variable
  -  也可以自定义自己的反向传播，见[官网](https://pytorch.org/docs/stable/autograd.html#function)

- 例子：

  ```python
  import torch
  # 我们只需要定义模型，前向传播和损失函数
  x = torch.ones(5)  # input tensor
  y = torch.zeros(3)  # expected output
  w = torch.randn(5, 3, requires_grad=True)
  b = torch.randn(3, requires_grad=True)
  z = torch.matmul(x, w)+b
  loss = torch.nn.functional.binary_cross_entropy_with_logits(z, y)
  # 反向传播PyTorch会自动帮我们算
  loss.backward()
  print(w.grad)
  print(b.grad)
  ```

### 五、Optimization

 Using gradient descent to optimize the parameters.

- 代码(整合前面3个部分)

  ```python
  import torch
  from torch import nn
  from torch.utils.data import DataLoader
  from torchvision import datasets
  from torchvision.transforms import ToTensor, Lambda
  
  training_data = datasets.FashionMNIST(
      root="data",
      train=True,
      download=True,
      transform=ToTensor()
  )
  
  test_data = datasets.FashionMNIST(
      root="data",
      train=False,
      download=True,
      transform=ToTensor()
  )
  
  train_dataloader = DataLoader(training_data, batch_size=64)
  test_dataloader = DataLoader(test_data, batch_size=64)
  
  class NeuralNetwork(nn.Module):
      def __init__(self):
          super(NeuralNetwork, self).__init__()
          self.flatten = nn.Flatten()
          self.linear_relu_stack = nn.Sequential(
              nn.Linear(28*28, 512),
              nn.ReLU(),
              nn.Linear(512, 512),
              nn.ReLU(),
              nn.Linear(512, 10),
          )
  
      def forward(self, x):
          x = self.flatten(x)
          logits = self.linear_relu_stack(x)
          return logits
  
  model = NeuralNetwork()
  
  # 定义Hyperparameter
  learning_rate = 1e-3
  batch_size = 64
  epochs = 5
  ```

- **Loss Function**

  - **Loss function** measures the degree of dissimilarity of obtained result to the target value, and it is the loss function that we want to minimize during training. 

  - Common loss functions include [nn.MSELoss](https://pytorch.org/docs/stable/generated/torch.nn.MSELoss.html#torch.nn.MSELoss) (Mean Square Error) for regression tasks, and [nn.NLLLoss](https://pytorch.org/docs/stable/generated/torch.nn.NLLLoss.html#torch.nn.NLLLoss) (Negative Log Likelihood) for classification. [nn.CrossEntropyLoss](https://pytorch.org/docs/stable/generated/torch.nn.CrossEntropyLoss.html#torch.nn.CrossEntropyLoss) combines `nn.LogSoftmax` and `nn.NLLLoss`.

  - 初始化loss function: 

    `loss_fn = nn.CrossEntropyLoss()`

- **Optimizer**

  - Optimization is the process of adjusting model parameters to reduce model error in each training step. **Optimization algorithms** define how this process is performed

  - There are many [different optimizers](https://pytorch.org/docs/stable/optim.html) available in PyTorch such as ADAM and RMSProp, that work better for different kinds of models and data.

  - 这里使用Stochastic Gradient Descent

    `optimizer = torch.optim.SGD(model.parameters(), lr=learning_rate)`

- **Optimization Loop**

  Each iteration of the optimization loop is called an **epoch**.

  Each epoch consists of two main parts:

  - **The Train Loop** - iterate over the training dataset and try to converge to optimal parameters.
  - **The Validation/Test Loop** - iterate over the test dataset to check if model performance is improving.

  ```python
  def train_loop(dataloader, model, loss_fn, optimizer):
      size = len(dataloader.dataset)
      for batch, (X, y) in enumerate(dataloader):
          # Compute prediction and loss
          # 计算输入的参数X,得到预测值
          pred = model(X)
          # 计算损失函数
          loss = loss_fn(pred, y)
  
          # Backpropagation
          ## 为了避免两次计算，重置模型参数的梯度
          optimizer.zero_grad()
          ## 反向传播计算每一个参数的梯度，PyTorch会粗存每一个gradients
          loss.backward()
          ## 根据gradient调整参数
          optimizer.step()
  
          if batch % 100 == 0:
              loss, current = loss.item(), batch * len(X)
              print(f"loss: {loss:>7f}  [{current:>5d}/{size:>5d}]")
  
  
  def test_loop(dataloader, model, loss_fn):
      size = len(dataloader.dataset)
      num_batches = len(dataloader)
      test_loss, correct = 0, 0
  
      with torch.no_grad():
          for X, y in dataloader:
              pred = model(X)
              test_loss += loss_fn(pred, y).item()
              correct += (pred.argmax(1) == y).type(torch.float).sum().item()
  
      test_loss /= num_batches
      correct /= size
      print(f"Test Error: \n Accuracy: {(100*correct):>0.1f}%, Avg loss: {test_loss:>8f} \n")
      
  # 选择CE作为要用的损失函数
  loss_fn = nn.CrossEntropyLoss()
  # 选择要用的optimizer
  optimizer = torch.optim.SGD(model.parameters(), lr=learning_rate)
  # 训练模型
  epochs = 10
  for t in range(epochs):
      print(f"Epoch {t+1}\n-------------------------------")
      train_loop(train_dataloader, model, loss_fn, optimizer)
      test_loop(test_dataloader, model, loss_fn)
  print("Done!")
  ```

### 六、Save & Load Model

```python
import torch
import torchvision.models as models
```

- 只保存模型参数：

  ```python
  model = models.vgg16(pretrained=True)
  torch.save(model.state_dict(), 'model_weights.pth')
  ```

  读取模型参数

  ```python
  model = models.vgg16() # we do not specify pretrained=True, i.e. do not load default weights
  model.load_state_dict(torch.load('model_weights.pth'))
  model.eval()
  ```

  - 读取时要创建一个一样的model实例，但这里不用下载参数
  - 第二步是，读取上面保存的参数
  - `model.eval()`必须在model test前添加，否则有输入数据，即使不训练，它也会改变权值。它的作用是**不启用 Batch Normalization 和 Dropout**，设置他们为evaluation mode.

- 保存整个模型

  保存的模型参数实际上一个字典类型，通过key-value的形式来存储模型的所有参数

  ```python
  torch.save(model, 'model.pth')
  ```

  读取整个模型

  ```python
  model = torch.load('model.pth')
  ```

## PyTorch常用小工具

### 1. Summary

[Summary](https://github.com/sksq96/pytorch-summary)可以计算每层参数的个数

- 安装使用：

  ```python
  pip install torchsummary
  from torchsummary import summary
  summary(your_model, input_size=(channels, H, W))
  ```

- 官网:示例

  CNN for MNIST

  ```python
  import torch
  import torch.nn as nn
  import torch.nn.functional as F
  from torchsummary import summary
  
  class Net(nn.Module):
      def __init__(self):
          super(Net, self).__init__()
          self.conv1 = nn.Conv2d(1, 10, kernel_size=5)
          self.conv2 = nn.Conv2d(10, 20, kernel_size=5)
          self.conv2_drop = nn.Dropout2d()
          self.fc1 = nn.Linear(320, 50)
          self.fc2 = nn.Linear(50, 10)
  
      def forward(self, x):
          x = F.relu(F.max_pool2d(self.conv1(x), 2))
          x = F.relu(F.max_pool2d(self.conv2_drop(self.conv2(x)), 2))
          x = x.view(-1, 320)
          x = F.relu(self.fc1(x))
          x = F.dropout(x, training=self.training)
          x = self.fc2(x)
          return F.log_softmax(x, dim=1)
  
  device = torch.device("cuda" if torch.cuda.is_available() else "cpu") # PyTorch v0.4.0
  model = Net().to(device)
  
  summary(model, (1, 28, 28))
  ```

### 2. Profiler

[profiler](https://pytorch.org/tutorials/recipes/recipes/profiler_recipe.html)不需要额外下载，pytorch自带。

可以用它来分析模型各种性能：

- 可以分析每个操作在GPU/CPU的时间花销
- 可以分析模型占用的GPU/CPU内存
- 追踪功能




### 3.Tensorboard

- TensorBoard helps us track跟踪 our metrics such as loss, accuracy and visualize the results, model graphs that may be needed during the machine learning workflow.
- 运行：
  - Linux:在Terminal中`tensorboard --logdir=runs`
  - Win: 在Anaconda终端中`tensorboard --logdir=runs`
    - 然后在网页中打开`http://localhost:6006/`

#### 3.1 TUM课程例子

##### 1.1 Set up TensorBoard

- 加载数据

  ```python
  # import all the required packages
  %load_ext autoreload
  %autoreload 2
  %matplotlib inline
  
  import matplotlib.pyplot as plt
  import numpy as np
  
  import torch
  import torchvision
  import torchvision.transforms as transforms
  
  import torch.nn as nn
  import torch.nn.functional as F
  import torch.optim as optim
  
  transform = transforms.Compose([transforms.ToTensor(),
                                  transforms.Normalize((0.5,),(0.5,))])  # mean and std have to be sequences (e.g. tuples),
                                                                        # therefore we should add a comma after the values
  
  fashion_mnist_dataset = torchvision.datasets.FashionMNIST(root='../datasets', train=True,
                                                            download=True, transform=transform)
  
  fashion_mnist_test_dataset = torchvision.datasets.FashionMNIST(root='../datasets', train=False,
                                                            download=True, transform=transform)
  
  trainloader = torch.utils.data.DataLoader(fashion_mnist_dataset, batch_size=8)
  
  testloader = torch.utils.data.DataLoader(fashion_mnist_test_dataset, batch_size=8)
  
  classes = ('T-shirt/top', 'Trouser', 'Pullover', 'Dress', 'Coat',
             'Sandal', 'Shirt', 'Sneaker', 'Bag', 'Ankle boot')
  
  def matplotlib_imshow(img, one_channel=False):
      if one_channel:
          img = img.cpu().mean(dim=0)
      img = img / 2 + 0.5     # unnormalize
      npimg = img.numpy()
      if one_channel:
          plt.imshow(npimg, cmap="Greys")
      else:
          plt.imshow(np.transpose(npimg, (1, 2, 0)))
  ```

- 运用gpu

  ```python
  device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
  print("Using the device",device)
  ```

- 初始化2层神经网络

  ```python
  class Net(nn.Module):
      def __init__(self, activation=nn.Sigmoid(),
                   input_size=1*28*28, hidden_size=100, classes=10):
          super(Net, self).__init__()
          self.input_size = input_size
  
          # Here we initialize our activation and set up our two linear layers
          self.activation = activation
          self.fc1 = nn.Linear(input_size, hidden_size)
          self.fc2 = nn.Linear(hidden_size, classes)
  
      def forward(self, x):
          x = x.view(-1, self.input_size) # flatten
          x = self.fc1(x)
          x = self.activation(x)
          x = self.fc2(x)
  
          return x
      
  net = Net()
  net.to(device)
  
  criterion = nn.CrossEntropyLoss()
  optimizer = optim.SGD(net.parameters(), lr=0.001, momentum=0.9)
  ```

- 设置记录数据的地址

  ```python
  from torch.utils.tensorboard import SummaryWriter
  
  # default `log_dir` is "runs" - we'll be more specific here
  writer = SummaryWriter('runs/introduction')
  ```

##### 1.2 Writing to TensorBoard

- Log一些数据到`SummaryWritter`

  ```python
  # get some random training images
  dataiter = iter(trainloader)
  images, labels = dataiter.next()
  
  # create grid of images
  img_grid = torchvision.utils.make_grid(images)
  
  # show images using our helper function
  matplotlib_imshow(img_grid)
  
  # Write the generated image to tensorboard
  writer.add_image('four_mnist_images', img_grid)
  ```

  - TensorBoard需要刷新一下才能看到结果

##### 1.3 Visualization Model Architectures

- 将1.1中的神经网络模型`Net`可视化出来

  ```python
  writer.add_graph(net.cpu(), images)
  writer.close()
  ```

  - TensorBoard需要刷新一下才能看到结果

##### 1.4 Training Network Models

这是TensorBoard最重要的用法

- 先定义2个helper function:

  ```python
  def images_to_probs(net, images):
      '''
      Returns the predicted class and probabilites of the image belonging to each of the classes from the network output
      '''
      output = net(images)
      # convert output probabilities to predicted class
      _, preds_tensor = torch.max(output, 1)
      preds = np.squeeze(preds_tensor.cpu().numpy())
      return preds, [F.softmax(el, dim=0)[i].item() for i, el in zip(preds, output)]
  
  
  def plot_classes_preds(net, images, labels):
      '''
      Returns a plot using the network, along with images
      and labels from a batch, that shows the network's class prediction along
      with its probability, alongside the actual label, coloring this
      information based on whether the prediction was correct or not.
      Uses the "images_to_probs" function defined above.
      '''
      preds, probs = images_to_probs(net, images)
      # plot the images in the batch, along with predicted and true labels
      fig = plt.figure(figsize=(4,4))
      
      for idx in np.arange(4):
          ax = fig.add_subplot(4, 1, idx+1, xticks=[], yticks=[])
          fig.tight_layout()
          matplotlib_imshow(images[idx], one_channel=True)
          ax.set_title("{0}, {1:.1f}%(label: {2})".format(
              classes[preds[idx]],
              probs[idx] * 100.0,
              classes[labels[idx]]),
                      color=("green" if preds[idx]==labels[idx].item() else "red"),loc="center",pad=5,fontsize="medium")
      return fig
  ```

- 训练模型

  ```python
  epochs = 1
  running_loss = 0.0
  net.to(device)
  
  for epoch in range(epochs):  # loop over the dataset multiple times
      for i, data in enumerate(trainloader, 0):#Iterating through the minibatches of the data
  
          # data is a tuple of (inputs, labels)
          inputs, labels = data
          
          # Makes sure that the model and the data are in the same device
          inputs = inputs.to(device)
          labels = labels.to(device)
          
          # Reset the parameter gradients for the current  minibatch iteration 
          optimizer.zero_grad()
  
          
          outputs = net(inputs)              # Perform a forward pass on the network with inputs
          loss = criterion(outputs, labels)  # calculate the loss with the network predictions and ground Truth
          loss.backward()                    # Perform a backward pass to calculate the gradients
          optimizer.step()                   # Optimise the network parameters with calculated gradients
  
          # Accumulate the loss
          running_loss += loss.item()
          
          if i % 1000 == 999:    # every thousandth mini-batch
              print("[Epoch %d, Iteration %5d]" % (epoch+1, i+1))
  
              # log the running loss
              writer.add_scalar('Training loss',
                              running_loss / 1000,
                              epoch * len(trainloader) + i)
  
              # log the plot showing the model's predictions on a  sample of mini-batch using our helper function
              
              writer.add_figure('Predictions vs Actuals',
                              plot_classes_preds(net, inputs, labels),
                              i)
              running_loss = 0.0
  
  print('Finished Training')
  ```

  - 在TensorBoard的Scalar中可以看到损失函数的下降
  - 在TensorBoard的IMAGES中可以看到各图片的预测结果

##### 1.5 Experimenting weight initialization strategies

- 新建一个SummaryWriter实例，将数据保存在weight_init_experiments

  ```python
  from torch.utils.tensorboard import SummaryWriter
  writer = SummaryWriter('runs/weight_init_experiments')
  ```

- 定义一个神经网络用于做测试，希望追踪每一层的输出

  ```python
  import torch.nn as nn
  import torch.nn.functional as F
  
  class Net(nn.Module):
      def __init__(self, activation_method):
          super(Net, self).__init__()
          
          self.x1 = torch.Tensor([])
          self.x2 = torch.Tensor([])
          self.x3 = torch.Tensor([])
          self.x4 = torch.Tensor([])
          self.x5 = torch.Tensor([])
          self.x6 = torch.Tensor([])
                  
          self.fc1 = nn.Linear(28*28, 300)
          self.fc2 = nn.Linear(300, 300)
          self.fc3 = nn.Linear(300, 300)
          self.fc4 = nn.Linear(300, 300)
          self.fc5 = nn.Linear(300, 300)
          self.fc6 = nn.Linear(300, 300)
          self.fc7 = nn.Linear(300, 10)
          
          if activation_method == "relu" :
              self.activation = nn.ReLU() 
          elif activation_method == "tanh":
              self.activation = nn.Tanh() 
          
      def forward(self, x):
          x = x.reshape(-1,28*28)
          self.x1 = self.activation(self.fc1(x))
          self.x2 = self.activation(self.fc2(self.x1))
          self.x3 = self.activation(self.fc3(self.x2))
          self.x4 = self.activation(self.fc4(self.x3))
          self.x5 = self.activation(self.fc5(self.x4))
          self.x6 = self.activation(self.fc6(self.x5))
          logits = self.fc7 (self.x6)
          return logits
  
      def collect_layer_out (self):# Return the output values for each of the network layers
          return [self.x1, self.x2, self.x3, self.x4, self.x5, self.x6]
   
  net = Net("tanh")
  ```

- 取样一些图片放入网络做测试

  ```python
  visloader = torch.utils.data.DataLoader(fashion_mnist_dataset, batch_size=40, shuffle=True)
  dataiter = iter(visloader)
  images, labels = dataiter.next()
  
  print("Size of the Mini-batch input:",images.size())
  ```

- 接下来我们：

  - will plot the histogram of activation values produced in each of the network layers as the input passes through the network model using the `add_histogram` function.
  - This helps us look at the distribution of activation values. 
  - Select the `HISTOGRAMS` tab in TensorBoard to visualise the experiment results.

##### 1.5.1 Constant weight initialization with $tanh$ activation

```python
net_const = Net("tanh")

def init_weights(m):
    if type(m) == nn.Linear:
        torch.nn.init.constant_(m.weight,2.0)
        m.bias.data.fill_(0.01)
        
net_const.apply(init_weights)
outputs = net_const(images)
layer_out = net_const.collect_layer_out()

for i, x in enumerate(layer_out):
    writer.add_histogram('constant_init', x, i+1)
```

- We can see that initialization with constant values does not break the symmetry对称性 of weights, i.e. all neurons in network always learn the same features from the input since the weights are the same.

##### 1.5.2 Random weight initialization of small numerical values with $tanh$ activation

```python
net_small_normal = Net("tanh")

def init_weights(m):
    if type(m) == nn.Linear:
        torch.nn.init.normal_(m.weight,mean=0.0, std=0.01)
        m.bias.data.fill_(0.01)
        
net_small_normal.apply(init_weights)
outputs = net_small_normal(images)
layer_out = net_small_normal.collect_layer_out()

for i, x in enumerate(layer_out):
    writer.add_histogram('small_normal_tanh', x, i+1)
```

- It will end up with **vanishing gradient problem**
  - If weights are initialized with low values, it gets mapped to around 0, and the small values will kill gradients when backpropagating through the network.

##### 1.5.3 Random weight initialization of large numerical values with $tanh$ activation

```python
net_large_normal = Net("tanh")

def init_weights(m):
    if type(m) == nn.Linear:
        torch.nn.init.normal_(m.weight,mean=0.0, std=0.2)
        m.bias.data.fill_(0.01)
        
net_large_normal.apply(init_weights)
outputs = net_large_normal(images)
layer_out = net_large_normal.collect_layer_out()

for i, x in enumerate(layer_out):
    writer.add_histogram('large_normal_tanh', x, i+1)
```

- It will end up with **vanishing gradient problem**
  - If weights are initialized with very high values, the term $Xw+b$ becomes significantly higher and with activation function such as $tanh$, the function returns value very close to $-1$ or $1$. At these values, the gradient of $tanh$ is very low, thus learning takes a lot of time.

##### 1.5.4 Xavier initialization with $tanh$ activation

From the previous examples, we can see that a proper weight initialization is needed to ensure nice distribution of the output of each layers. Here comes the **Xavier Initialization**.

We will fill the weight with values using a normal distribution $\mathcal{N}(0,{\sigma}^2)$ where

$$ \sigma = gain \times \sqrt{\frac{2}{fan _{in} + fan_{out}}} $$

Here $fan _{in}$ and $ fan_{out} $ are number of neurons in the input and output layer and ${gain}$ is a optional scaling factor.

```python
net_xavier = Net("tanh")

def init_weights(m):
    if type(m) == nn.Linear:
        torch.nn.init.xavier_normal_(m.weight)
        m.bias.data.fill_(0.01)
        
net_xavier.apply(init_weights)
outputs = net_xavier(images)
layer_out = net_xavier.collect_layer_out()

for i, x in enumerate(layer_out):
    writer.add_histogram('xavier_tanh', x, i+1)
```

##### 1.5.5 Xavier initialization with ReLU

```python
net_xavier_relu = Net("relu")

def init_weights(m):
    if type(m) == nn.Linear:
        torch.nn.init.xavier_uniform_(m.weight)
        m.bias.data.fill_(0.01)
        
net_xavier_relu.apply(init_weights)
outputs = net_xavier_relu(images)
layer_out = net_xavier_relu.collect_layer_out()

for i, x in enumerate(layer_out):
    writer.add_histogram('xavier_relu', x, i+1)
```

- 对比Xavier初始化搭配ReLu和tanh这两个激活函数，可以发现：

  - Xavier initialization requires a zero centered activation function such as $tanh$ to work well.
  - layer outputs collapse to zero again if we use non-zero centered activation such as ReLU.

  所以Xavier不适用于ReLu

##### 1.5.6  He initialization with ReLU

即Kaiming Initialization.

**He Initialization** comes to our rescue for non-centered activation functions. We will fill the weight with values using a normal distribution $\mathcal{N}(0,\sigma^2)$ where

$$ \sigma = \frac {gain} {\sqrt{fan_{mode}}} $$

Here $fan _{mode}$ can be chosen either $fan _{in}$ (default) or $fan _{out}$.

Choosing $fan _{in}$ preserves保存 the magnitude重要性 of the variance of the weights in the forward pass. Choosing $fan _{out}$ preserves the magnitudes of weights during the backwards pass. The variable $gain$ is again the optional scaling缩放 factor.

```python
net_kaiming_relu = Net("relu")

def init_weights(m):
    if type(m) == nn.Linear:
        torch.nn.init.kaiming_uniform_(m.weight,nonlinearity='relu')
        m.bias.data.fill_(0.01)
        
net_kaiming_relu.apply(init_weights)
outputs = net_kaiming_relu(images)
layer_out = net_kaiming_relu.collect_layer_out()

for i, x in enumerate(layer_out):
    writer.add_histogram('kaiming_relu', x, i+1)
```

### 4.PyTorch Lightning

[源代码](https://github.com/Lightning-AI/lightning)

PyTorch已经足够简单易用，但是简单易用不等于方便快捷。特别是做大量实验的时候，很多东西都会变得复杂，代码也会变得庞大，这时候就容易出错。
针对这个问题，就有了[PyTorchLightning](https://www.pytorchlightning.ai/)。它可以重构你的PyTorch代码，抽出复杂重复部分，让你专注于核心的构建，让你的实验更快速更便捷地开展迭代。

代码都可以直接在PyTorch跑, Lightning只是帮助我们整合了这些代码

#### 4.1 Idea behind PyTorch Lightning

- Codes in a Deep learning project consists of three main categories:

  - **Research code**  

    This is the exciting part of the experiment where you configure the model architecture and try out different optimizers and target task. This is managed by the **`LightningModule` **of PyTorch Lightning.

    -  **LightningModules** contain all model related code. This is the part where we are working on when creating a new project. The idea is to have all important code in one module, e.g., the model's architecture and the evaluation of training and validation metrics. This provides a better overview as repeated elements, such as the training procedure, are not stored in the code that we work on. The lightning module also handles the calls `.to(device)` or `.train()` and `.eval()`. Hence, there is no need anymore to switch between the cpu and gpu and to take care of the model's mode as this is automated by the LightningModule. The framework also enables easy parallel computation on multiple gpus. 

  2. **Engineering code**  

     This is the same set of code that remain the same for all deep learning projects.Recall the training block of previous notebooks where we loop through the epochs and mini-batches. The  **`Trainer`**  class of PyTorch Lightning takes care of this part of code.

     - **Trainer** contains all code needed for training our neural networks that doesn't change for each project ("one size fits all"). Usually, we don't touch the code automated by this class. The arguments参数 that are specific特定于 for one training such as learning rate and batch size are provided as initialization arguments for the LightningModule.

  3. **Non-essential code**
     It is very important that we log our training metrics and organize different training runs to have purposeful experimentation of models. The **`Callbacks`** class PyTorch Lightning helps us with this section. 

     - **Callbacks** automate all parts needed for logging hyperparameters or training results such as the tensorboard logger. Logging becomes very important for research later since the results of experiments need to be reproducible.



#### 4.2 Training with PyTorch Lightning

下面一个两层类的例子

##### 2.1 LightningModule

定义模型，训练，验证和优化部分

在单纯PyTorch中建立我们的神经网络模型事基于`nn.Module`类，在`Lightning`中建立是基于`pl.LightningModule`类。

  ```python
import pytorch_lightning as pl

class TwoLayerNet(pl.LightningModule):
    #-----------------------模型部分-----------------------
    def __init__(self, hparams, input_size=1 * 28 * 28, hidden_size=512, num_classes=10):
        super().__init__()
        self.save_hyperparameters(hparams)

        self.model = nn.Sequential(
            nn.Linear(input_size, hidden_size),
            nn.Sigmoid(),
            nn.Linear(hidden_size, num_classes),
        )

    def forward(self, x):
        # flatten the image  before sending as input to the model
        N, _, _, _ = x.shape
        x = x.view(N, -1)
        x = self.model(x)
        return x
    
    #-----------------------训练和验证部分-----------------------
    def training_step(self, batch, batch_idx):
        images, targets = batch
        # Perform a forward pass on the network with inputs
        out = self.forward(images)
        # calculate the loss with the network predictions and ground truth targets
        loss = F.cross_entropy(out, targets)
        # Find the predicted class from probabilities of the image belonging to each of the classes
        # from the network output
        _, preds = torch.max(out, 1)
        # Calculate the accuracy of predictions
        acc = preds.eq(targets).sum().float() / targets.size(0)
        # Log the accuracy and loss values to the tensorboard
        self.log('loss', loss)
        self.log('acc', acc)
        return {'loss': loss}

    def validation_step(self, batch, batch_idx):
        images, targets = batch
        # Perform a forward pass on the network with inputs
        out = self.forward(images)
        # calculate the loss with the network predictions and ground truth targets
        loss = F.cross_entropy(out, targets)
        # Find the predicted class from probabilities of the image belonging to each of the classes
        # from the network output
        _, preds = torch.max(out, 1)
        # Calculate the accuracy of predictions
        acc = preds.eq(targets).sum().float() / targets.size(0)
        # Visualise the predictions  of the model
        if batch_idx == 0:
            self.visualize_predictions(images, out.detach(), targets)
        return {'val_loss': loss, 'val_acc': acc}

    def validation_epoch_end(self, outputs):
        # Average the loss over the entire validation data from it's mini-batches
        avg_loss = torch.stack([x['val_loss'] for x in outputs]).mean()
        avg_acc = torch.stack([x['val_acc'] for x in outputs]).mean()
        # Log the validation accuracy and loss values to the tensorboard
        self.log('val_loss', avg_loss)
        self.log('val_acc', avg_acc)
        
     #-----------------------优化部分部分-----------------------
    def configure_optimizers(self):
        optim = torch.optim.SGD(self.model.parameters(), self.hparams["learning_rate"], momentum=0.9)
        return optim
  ```

  ##### 2.2 LightningDataModule

 PyTorch Lightning 提供 `LightningDataModule` 作为基类，来设置dataloaders数据加载.

```python
class FashionMNISTDataModule(pl.LightningDataModule):
	#-----------------------加载数据集-----------------------
    def __init__(self, batch_size=4):
        super().__init__()
        self.batch_size = batch_size

    def prepare_data(self):
        # Define the transform
        ## transforms.Normalize(，)第一个参数是mean平均数,第二个参数是std标准差
        transform = transforms.Compose([transforms.ToTensor(), transforms.Normalize((0.5,), (0.5,))])
        # Download the Fashion-MNIST dataset
        fashion_mnist_train_val = torchvision.datasets.FashionMNIST(root='../datasets', train=True, download=True, transform=transform)
        self.fashion_mnist_test = torchvision.datasets.FashionMNIST(root='../datasets', train=False,download=True, transform=transform)
        # Apply the Transforms
        transform = transforms.Compose([transforms.ToTensor(), transforms.Normalize((0.5,), (0.5,))])
        # Perform the training and validation split
        self.train_dataset, self.val_dataset = random_split(
            fashion_mnist_train_val, [50000, 10000])
        
    #-----------------------建立3个数据集-----------------------   
    def train_dataloader(self):
        return DataLoader(self.train_dataset, batch_size=self.batch_size)
    def val_dataloader(self):
        return DataLoader(self.val_dataset, batch_size=self.batch_size)
    def test_dataloader(self):
        return DataLoader(self.fashion_mnist_test, batch_size=self.batch_size)
```



##### 2.3 Fitting the model with a Trainer

- 首先初始化我们的模型并加载数据（2.1，2.2）

  ```python
  from IPython.display import clear_output 
  from exercise_code.lightning_models import TwoLayerNet
  from exercise_code.data_class import FashionMNISTDataModule
  # hyperparameters
  hparams = {
      "batch_size": 16,
      "learning_rate": 1e-3,
      "input_size": 1 * 28 * 28,
      "hidden_size": 512,
      "num_classes": 10,
      "num_workers": 2,    # used by the dataloader, more workers means faster data preparation, but for us this is not a bottleneck here
  }
  
  
  model = TwoLayerNet(hparams)
  data=FashionMNISTDataModule(hparams)
  data.prepare_data()
  ```

- Trainer

  PyTorch Lightning提供大量的[Trainer参数](https://pytorch-lightning.readthedocs.io/en/latest/api/pytorch_lightning.trainer.trainer.Trainer.html#pytorch_lightning.trainer.trainer.Trainer)供我们灵活使用。

  ```python
  # 设置我们的训练器
  trainer = pl.Trainer(
      max_epochs=2,
      ## 使用GPU
  	accelerator = 'gpu' 
  )
  # 训练我们的模型
  trainer.fit(model, data)
  ```

  - 每一次运行`trainer.fit(model, data)`,都会在/lightning_logs下生成一个新的/version_xx文件夹，里面有checkpoints,logs和我们设置的超参数们
  - 可以用`tensorboard --logdir lightning_logs`,来查看各种设置和结果

##### 2.4 Add images to tensorboard

- The tensorboard logger is a submodule of the `LightningModule` and can be accessed via `self.logger`. We can add images to the logging module by calling

​		`self.logger.experiment.add_image('tag', image)`

- 可以在TensorBoard中查看发送的图片

- 代码：

  ```python
      def visualize_predictions(self, images, preds, targets):
          class_names = ['airplane', 'automobile', 'bird', 'cat', 'deer', 
                         'dog', 'frog', 'horse', 'ship', 'truck']
  
          # determine size of the grid based on given batch size
          num_rows = torch.tensor(len(images)).float().sqrt().floor()
  
          fig = plt.figure(figsize=(10, 10))
          for i in range(len(images)):
              plt.subplot(num_rows ,len(images) // num_rows + 1, i+1)
              plt.imshow(images[i].permute(1, 2, 0))
              plt.title(class_names[torch.argmax(preds, axis=-1)[i]] + f'\n[{class_names[targets[i]]}]')
              plt.axis('off')
  
          self.logger.experiment.add_figure('predictions', fig, global_step=self.global_step)
  ```


# 6.安装libtorch

1. 从[官网](https://pytorch.org/get-started/locally/)下载cxx11 ABI版本的libtorch
2. 解压到/home/yang/3rdLibrary/libtorch
3. 然后就可以用了- -

## 使用

- VS CODE环境：

  添加：

  ```
  /home/yang/3rdLibrary/libtorch/include/torch/csrc/api/include
  /home/yang/3rdLibrary/libtorch/include
  ```

- .cpp：

  ```c++
  #include "torch/torch.h"
  #include <iostream>
  
  int main() {
    torch::Tensor tensor = torch::rand({2, 3});
    std::cout << tensor << std::endl;
  }
  ```

- CMakeLists.txt

  ```cmake
  cmake_minimum_required(VERSION 3.5)
  project(test_pytorch)
  
  set(CMAKE_CXX_STANDARD 14) #设置为C++11，可能报错
  set(Torch_DIR /home/yang/3rdLibrary/libtorch/share/cmake/Torch) #确定.cmake文件的地址
  find_package(Torch REQUIRED)  
  message(STATUS "Torch library status:")
  message(STATUS "    include path: ${TORCH_INCLUDE_DIRS}")
  message(STATUS "    lib path : ${TORCH_LIBRARIES} ")
  
  add_executable(test test.cpp)
  target_link_libraries(test "${TORCH_LIBRARIES}")
  ```

- 编译

  ```
  mkdir build
  cd build
  cmake ..
  make
  ./test
  ```

  

# 7.多版本cuda切换

1. 安装第二个[cuda](https://developer.nvidia.com/cuda-toolkit-archive)版本时，选择runfile(local)安装

2. cudnn如果是11.x,12.x这样的大版本不同，也要安装对应的cudnn

3. 在.bashrc中选择使用哪个cuda:

   ```
   # cuda 12.1
   # export PATH="/usr/local/cuda-12.1/bin:$PATH"
   # export LD_LIBRARY_PATH="/usr/local/cuda-12.1/lib64:$LD_LIBRARY_PATH"
   # export CUDA_HOME=/usr/local/cuda-12.1
   
   # cuda 11.8
   export PATH="/usr/local/cuda-11.8/bin:$PATH"
   export LD_LIBRARY_PATH="/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH"
   export CUDA_HOME=/usr/local/cuda-11.8
   ```


# 8.查看安装的各种包

- apt-get安装的包

  ```shell
   sudo apt list/show | grep cuda
  ```

- conda安装的包

  ```shell
  conda list | grep cuda
  ```

- pip安装的包

  ```
  pip list/show | grep xxx
  ```

  