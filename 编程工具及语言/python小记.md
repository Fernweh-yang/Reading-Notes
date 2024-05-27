# 0.如何修改默认的python版本

- 查看python版本

  - 当前计算机有的版本：

    ```
    ls /usr/bin/python*
    ```

  - 当前系统默认的pytrhon版本

    ```
    python -V
    ```

- 下载想要的版本：

  ```
  sudo apt-get install software-properties-common
  
  # 添加 python source
  sudo add-apt-repository ppa:deadsnakes/ppa
  sudo apt update
  
  # 安装 python3.8
  sudo apt install python3.8
  ```

- 更换版本

  使用 update-alternatives 命令为 Ubuntu 系统中安装的同一软件的不同版本设置优先级。具有最高优先级的 Python 版本将用作默认版本。

  ```shell
  # 将3.8设为第一优先级
  sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.8 1
  # 将3.10设为第二优先级
  sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.10 2
  ```

  手动切换版本

  ```
  sudo update-alternatives --config python
  # 然后输入想要的python数值
  python --version
  ```

  

# 1.如何打包python项目

- 资料总结：
  - https://www.osgeo.cn/python-packaging/tutorials/packaging-projects.html

- 下面以打包一个example_pkg项目为例子：

## 安装python包的4种方式：

使用 `setup.py`、`setup.cfg`、`pyproject.toml` 或 `requirements.txt` 文件，以及直接从源代码安装

## 1. 用setup.py来安装

### 1.1 创建项目文件

```python
├─packaging
│  └─example_pkg
│          __init__.py
```

其中`__init__.py`加入如下代码

```python
# 用于验证是否正确安装
name = "example_pkg"
```

下面发布example_pkg这个包

### 1.2 创建包文件

```shell
├─packaging
│  │ LICENSE
│  │ README.md
│  │ setup.py
│  │
│  └─example_pkg
│          __init__.py
```



#### 1.2.1 setup.py

`setup.py` 是的生成脚本 [setuptools](https://www.osgeo.cn/python-packaging/key_projects.html#setuptools) . 它告诉安装工具关于您的包（如名称和版本）以及要包括哪些代码文件。

```python
import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="example-pkg-yang",
    version="0.0.1",
    author="Yang Xu",
    author_email="xudashuai512@gmail.com",
    description="A small example package",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/pypa/sampleproject",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
```

- `name` 是包的名字。只能包含字母、数字和 `_` 和 `-`。必须是唯一的，不能和pypi.org上已有的包同名。
- `version` 包版本见 [PEP 440](https://www.python.org/dev/peps/pep-0440) 有关版本的详细信息。
- `author` 和 `author_email` 用于标识包的作者。
- `description` 是包的一个简短的一句话摘要。
- `long_description` 是对包的详细描述。这在python包索引的包细节包中显示。通常长描述从 `README.md`中读取。
- `long_description_content_type` 告诉索引用于长描述的标记类型。这里用的markdown。
- `url` 是项目主页的URL。对于许多项目，这只是到GitHub、Gitlab、BitBucket或类似的代码托管服务的链接。
- `packages` 是所有python的列表 [import packages](https://www.osgeo.cn/python-packaging/glossary.html#term-import-package) 应该包括在 [distribution package](https://www.osgeo.cn/python-packaging/glossary.html#term-distribution-package) . 我们可以使用 `find_packages()` 自动发现所有包和子包。这里example_pkg是唯一的包裹。
- `classifiers` 给出索引和 [pip](https://www.osgeo.cn/python-packaging/key_projects.html#pip) 关于您的包的一些附加元数据。这里包只与python 3兼容，在MIT许可下获得许可，并且是独立于操作系统的。您应该始终至少包括您的包所使用的Python的哪个版本、包所使用的许可证以及包将使用的操作系统。有关分类器的完整列表，请参阅https://pypi.org/classifiers/。

#### 1.2.2 Readme.md

1.2.1中提到的long_description:

```markdown
# Example Package
This is a simple example package.
```

#### 1.2.3 LICENSE

对于上传到python包索引的每个包来说，包含一个许可证是很重要的。这会告诉安装您的软件包的用户可以使用您的软件包的条款。有关选择许可证的帮助，请参阅https://choosealelicense.com/。选择许可证后，打开 `LICENSE` 并输入许可证文本。例如，如果您选择了MIT许可证：

```
Copyright (c) 2018 The Python Packaging Authority

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```



### 1.3 得到分配包

为了得到包裹的[distribution packages](https://www.osgeo.cn/python-packaging/glossary.html#term-distribution-package) 

1. 需要setuptools 和 wheel

   ```
   python3 -m pip install --user --upgrade setuptools wheel
   ```

2. 在setup.py所在的文件夹下执行```python3 setup.py sdist bdist_wheel```

   得到文件夹dist：

   ```shell
   C:\Users\51212\Desktop\testeverything\packaging> tree /f
   │  LICENSE
   │  README.md
   │  setup.py
   │
   ├─build
   │  ├─bdist.win-amd64
   │  └─lib
   │      └─example_pkg
   │              __init__.py
   │
   ├─dist
   │      example-pkg-yang-test-0.0.1.tar.gz
   │      example_pkg_yang_test-0.0.1-py3-none-any.whl
   │
   ├─example_pkg
   │      __init__.py
   │
   └─example_pkg_yang_test.egg-info
           dependency_links.txt
           PKG-INFO
           SOURCES.txt
           top_level.txt
   ```

   

   

### 1.4 发布包

因为这里并不把这个测试包真的发出到[Python Package Index (PyPI)](https://www.osgeo.cn/python-packaging/glossary.html#term-python-package-index-pypi)，而是发出到[test.pypi.org](https://test.pypi.org/)。

1. TestPyPI可以用于测试发布我们的代码，而不需要担心干扰真实的版本号。但需要[创建一个账号](https://test.pypi.org/account/register/)

2. 安装 [twine](https://www.osgeo.cn/python-packaging/key_projects.html#twine) 用来上传分发包

   ```
   python3 -m pip install --user --upgrade twine
   ```

3. 上传包

   ```
   python3 -m twine upload --repository-url https://test.pypi.org/legacy/ dist/*
   ```

   得到：

   ```shell
   PS C:\Users\51212\Desktop\testeverything\packaging> python3 -m twine upload --repository-url https://test.pypi.org/legacy/ dist/*
   Uploading distributions to https://test.pypi.org/legacy/
   Enter your username: tianxiadiyishuai
   Enter your password:
   Uploading example_pkg_yang_test-0.0.1-py3-none-any.whl
   100% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 5.5/5.5 kB • 00:00 • ?
   Uploading example-pkg-yang-test-0.0.1.tar.gz
   100% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 5.1/5.1 kB • 00:00 • ?
   
   View at:
   https://test.pypi.org/project/example-pkg-yang-test/0.0.1/
   ```

### 1.5 下载包

- 从TestPyPi上下载包

  可在1.4的https://test.pypi.org/project/example-pkg-yang-test/0.0.1/查看下载指令

  ```
  pip install -i https://test.pypi.org/simple/ example-pkg-yang-test==0.0.1
  ```

- 检查一下：

  ```
  (.env) PS C:\Users\51212\Desktop\testeverything\packaging> pip list
  Package               Version Editable project location
  --------------------- ------- --------------------------------------------------
  cloudpickle           2.2.0
  example-pkg-yang-test 0.0.1
  gym                   0.26.0
  gym-examples          0.0.1   c:\users\51212\desktop\testeverything\gym-examples
  gym-notices           0.0.8
  numpy                 1.23.5
  pip                   22.3.1
  pygame                2.1.0
  setuptools            65.5.0
  ```

- 使用一下：

  ```python
  >>> import example_pkg		# 1.1中包的名字叫example_pkg
  >>> example_pkg.name		# 1.1中__init__.py中的代码
  'example_pkg'
  ```

### 1.6 本地卸载

```shell
pip uninstall example-pkg-yang-test	# 1.2.1中定义的包名
```

### 1.7 如果不需要发布

可以直接创建setup后安装

1. 文件夹格式如下：

   ```
   (.env) PS C:\Users\51212\Desktop\testeverything\testpkg> tree /f
   │  README.md
   │  setup.py
   │
   ├─example_pkg
   │      __init__.py
   ```

2. 安装这个包到环境中去

   ```
   (.env) PS C:\Users\51212\Desktop\testeverything\testpkg> pip install -e .
   ```




## 2. 用pyproject.toml来安装

pyproject.toml是在 PEP 518（Specifying Minimum Build System Requirements for Python Projects）中定义的，于 2020 年 11 月发布，该 PEP 于 Python 3.7 版本中引入。

主要用于管理构建依赖、版本控制、文档等。

- 安装:

  ```shell
  # 会使用pyproject.toml来构建和安装
  pip install -e .
  ```

### 2.1 一个例子

来自nerfstudio

```toml
[build-system]
requires = ["setuptools>=61.0"]
build-backend = "setuptools.build_meta"

[project]
name = "nerfstudio"
version = "0.3.4"
description = "All-in-one repository for state-of-the-art NeRFs"
readme = "README.md"
license = { text="Apache 2.0"}
requires-python = ">=3.8.0"
classifiers = [
    "Development Status :: 3 - Alpha",
    "Programming Language :: Python",
]
dependencies = [
    "appdirs>=1.4",
    "av>=9.2.0",
    "comet_ml>=3.33.8",
    "cryptography>=38",
    "tyro>=0.5.10",
    "gdown>=4.6.0",
    "ninja>=1.10",
    "h5py>=2.9.0",
    "imageio>=2.21.1",
    'importlib-metadata>=6.0.0; python_version < "3.10"',
    "ipywidgets>=7.6",
    "jaxtyping>=0.2.15",
    "jupyterlab>=3.3.4",
    "matplotlib>=3.5.3",
    "mediapy>=1.1.0",
    "msgpack>=1.0.4",
    "msgpack_numpy>=0.4.8",
    "nerfacc==0.5.2",
    "open3d>=0.16.0",
    "opencv-python==4.6.0.66",
    "Pillow>=9.3.0",
    "plotly>=5.7.0",
    "protobuf<=3.20.3,!=3.20.0",
    # TODO(1480) enable when pycolmap windows wheels are available
    # "pycolmap==0.3.0",
    "pymeshlab>=2022.2.post2; platform_machine != 'arm64'",
    "pyngrok>=5.1.0",
    "python-socketio>=5.7.1",
    "pyquaternion>=0.9.9",
    # TODO we can switch back to (non-new) rawpy if they start releasing arm64
    # wheels. https://github.com/letmaik/rawpy/issues/171#issuecomment-1572627747
    "rawpy>=0.18.1; platform_machine != 'arm64'",
    "newrawpy>=0.18.1; platform_machine == 'arm64'",
    "requests",
    "rich>=12.5.1",
    "scikit-image>=0.19.3",
    "splines==0.3.0",
    "tensorboard>=2.13.0",
    "torch>=1.13.1",
    "torchvision>=0.14.1",
    "torchmetrics[image]>=1.0.1",
    "typing_extensions>=4.4.0",
    "viser==0.1.17",
    "nuscenes-devkit>=1.1.1",
    "wandb>=0.13.3",
    "xatlas",
    "trimesh>=3.20.2",
    "timm==0.6.7",
    "gsplat==0.1.0",
    "pytorch-msssim",
    "pathos"
]

[project.urls]
"Documentation" = "https://docs.nerf.studio"


[project.optional-dependencies]

# Generative related dependencies
gen = [
    "diffusers==0.16.1",
    "transformers==4.29.2",
    "accelerate==0.19.0",
    "bitsandbytes==0.39.0",
    "sentencepiece==0.1.99",
]


# Development packages
dev = [
    "black[jupyter]==23.3.0",
    "pre-commit==3.3.2",
    "pytest==7.1.2",
    "pytest-xdist==2.5.0",
    "typeguard==2.13.3",
    "ruff==0.0.267",
    "sshconf==0.2.5",
    "pycolmap>=0.3.0",  # NOTE: pycolmap==0.3.0 is not available on newer python versions
    "diffusers==0.16.1",
    "opencv-stubs==0.0.7",
    "transformers==4.29.2",
    "pyright==1.1.331",
    "projectaria_tools[all]>=1.2.0",
]

# Documentation related packages
docs = [
    "furo==2022.09.29",
    # Specifying ipython for https://github.com/ipython/ipython/issues/13845
    "ipython==8.6.0",
    "readthedocs-sphinx-search==0.1.2",
    "myst-nb==0.16.0",
    "nbconvert==7.2.5",
    "nbformat==5.5.0",
    "sphinx==5.2.1",
    "sphinxemoji==0.2.0",
    "sphinx-argparse==0.3.1",
    "sphinx-copybutton==0.5.0",
    "sphinx-design==0.2.0",
    "sphinxext-opengraph==0.6.3"
]

[project.scripts]
# Note, add entrypoint name to nerfstudio/scripts/completions/install.py to include CLI completion
ns-install-cli = "nerfstudio.scripts.completions.install:entrypoint"
ns-process-data = "nerfstudio.scripts.process_data:entrypoint"
ns-download-data = "nerfstudio.scripts.downloads.download_data:entrypoint"
ns-train = "nerfstudio.scripts.train:entrypoint"
ns-viewer = "nerfstudio.scripts.viewer.run_viewer:entrypoint"
ns-eval = "nerfstudio.scripts.eval:entrypoint"
ns-render = "nerfstudio.scripts.render:entrypoint"
ns-export = "nerfstudio.scripts.exporter:entrypoint"
ns-dev-test = "nerfstudio.scripts.github.run_actions:entrypoint"
ns-dev-sync-viser-message-defs = "nerfstudio.scripts.viewer.sync_viser_message_defs:entrypoint"

[options]
# equivalent to using --extra-index-url with pip, which is needed for specifying the CUDA version torch and torchvision
dependency_links = [
    "https://download.pytorch.org/whl/cu118"
]

[tool.setuptools.packages.find]
include = ["nerfstudio*"]

[tool.setuptools.package-data]
"*" = ["*.cu", "*.json", "py.typed", "setup.bash", "setup.zsh"]

[tool.black]
line-length = 120

[tool.pytest.ini_options]
addopts = "-n=4 --typeguard-packages=nerfstudio --disable-warnings"
testpaths = [
    "tests",
]

[tool.pyright]
include = ["nerfstudio"]
exclude = ["**/node_modules",
    "**/__pycache__",
]
defineConstant = { DEBUG = true }

reportMissingImports = "warning"
reportMissingTypeStubs = false
reportPrivateImportUsage = false

pythonVersion = "3.8"
pythonPlatform = "Linux"

[tool.ruff]
line-length = 120
select = [
    "E",  # pycodestyle errors.
    "F",  # Pyflakes rules.
    "PLC",  # Pylint convention warnings.
    "PLE",  # Pylint errors.
    "PLR",  # Pylint refactor recommendations.
    "PLW",  # Pylint warnings.
]
ignore = [
    "E501",  # Line too long.
    "F722",  # Forward annotation false positive from jaxtyping. Should be caught by pyright.
    "F821",  # Forward annotation false positive from jaxtyping. Should be caught by pyright.
    "PLR2004",  # Magic value used in comparison.
    "PLR0915",  # Too many statements.
    "PLR0913",  # Too many arguments.
    "PLC0414",  # Import alias does not rename variable. (this is used for exporting names)
    "PLC1901",  # Use falsey strings.
    "PLR5501",  # Use `elif` instead of `else if`.
    "PLR0911",  # Too many return statements.
    "PLR0912",  # Too many branches.
    "PLW0603",  # Globa statement updates are discouraged.
    "PLW2901",  # For loop variable overwritten.
]

```



# 2. 数据操作

## 2.1 判断数据类型

使用`if not isinstance`来判断：

```python
# 判断是否是浮点型
a=2
if not isinstance(a,float):
	raise TypeError("wrong")
    
# 判断是否是tuple
b=[1,2,3]
if not isinstance(b,tuple):
    raise TypeError("not tuple")
```

## 2.2 保存数据到csv
```python
import csv

savepath = "test.csv"
# 在最开始创建csv文件，并写入列名。相当于做一些准备工作
with open(savepath, 'w') as csvfile:         #以写入模式打开csv文件，如果没有csv文件会自动创建。
    writer = csv.writer(csvfile)
    # writer.writerow(["index","a_name","b_name"])  # 写入列名，如果没有列名可以不执行这一行
    # writer.writerows([[0, 1, 3], [1, 2, 3], [2, 3, 4]]) # 写入多行用writerows
  
 #如果你的数据量很大，需要在循环中逐行写入数据
for i in range(100000):
	 with open(savepath, 'a+', newline='') as csvfile:      # a+表示以追加模式写入，如果用w会覆盖掉原来的数据。如果没有newline=''，则逐行写入的数据相邻行之间会出现一行空白。读者可以自己试一试。
	 csv_write = csv.writer(csvfile)
	 csv_write.writerow(row_data)    # 写入1行用writerow; row_data是你要写入的数据，最好是list类型。
 
 
f = open(savepath)
csv_read = csv.reader(f)
for line in csv_read:                # csv.reader(f)返回一个迭代器。迭代器的好处就是可以不用一次性将大量的数据都读进来，而是如果你需要一条，就给迭代器一个命令让它输出一条。关于迭代器的优点读者可以另行学习。
	print line
```
## 2.3 获得代码运行时长:
```python
T_current = time.perf_counter()
dt = T_current-T_last
print(dt)           
T_last = T_current
```


# 3. 类相关
## 3.1 修饰符:classmethod
classmethod 修饰符对应的函数不需要实例化，不需要 self 参数，但第一个参数需要是表示自身类的 cls 参数，可以来调用类的属性，类的方法，实例化对象等.  
例子：
```python
class A(object):

    # 属性默认为类属性（可以给直接被类本身调用）
    num = "类属性"

    # 实例化方法（必须实例化类之后才能被调用）
    def func1(self): # self : 表示实例化类后的地址id
        print("func1")
        print(self)

    # 类方法（不需要实例化类就可以被类本身调用）
    @classmethod
    def func2(cls):  # cls : 表示没用被实例化的类本身
        print("func2")
        print(cls)
        print(cls.num)
        cls().func1()

    # 不传递传递默认self参数的方法（该方法也是可以直接被类调用的，但是这样做不标准）
    def func3():
        print("func3")
        print(A.num) # 属性是可以直接用类本身调用的
    
# A.func1() 这样调用是会报错：因为func1()调用时需要默认传递实例化类后的地址id参数，如果不实例化类是无法调用的
A.func2()
A.func3()

# A.func2()输出是：
# func2
# <class '__main__.A'>
# 类属性
# func1
# <__main__.A object at 0x7fdc6d3f8c50>
```

## 3.2 修饰符:abstractmethod  
- python没有抽象类，而是通过继承abc类来表示是抽象类
  - 但并不会像C++中定义了抽象类就有抽象类的性质了：只有重写了才能实例
- 并用 `@abstractmethod修饰符` 来表示抽象方法
  - 如果子类没有重写@abstractmethod修饰的方法，会报TypeError异常
  例子：
```python
from abc import ABC,abstractmethod
class A(ABC):
    @abstractmethod
    def needrewrite(self):
        pass

class B(A):
    def needrewrite(self):
        print("rewrite")

# 只有B继承了A，并重写了@abstractmethod修饰的方法，才能实例化
c=B()
c.needrewrite()

# 因为有@abstractmethod修饰的方法，所以不能实例化。但如果去掉修饰符就能实例化了。
d=A()
d.needrewrite()
	
```

## 3.3 修饰符:property
- @property修饰符会创建只读属性，@property修饰符会将方法转换为相同名称的只读属性,可以与所定义的属性配合使用，这样可以防止属性被修改。  
用法1：将方法变成像属性一样来访问
```python
class DataSet(object):
  @property
  def method_with_property(self): ##含有@property
      return 15
  def method_without_property(self): ##不含@property
      return 15

l = DataSet()
print(l.method_with_property) # 加了@property后，可以用调用属性的形式来调用方法,后面不需要加（）。
print(l.method_with_property())   # 加了@property后,加()会把错：TypeError: 'int' object is not callable
print(l.method_without_property())  #没有加@property , 必须使用正常的调用方法的形式，即在后面加()
```
- 由于python进行属性的定义时，没办法设置私有属性，因此要通过@property的方法来进行设置。这样可以隐藏属性名，让用户进行使用的时候无法随意修改  
方法2：与所定义的属性配合使用，这样可以防止属性被修改。
```python
class DataSet(object):
    def __init__(self):
        self._images = 1
        self._labels = 2 #定义属性的名称
    @property
    def images(self): #方法加入@property后，这个方法相当于一个属性，这个属性可以让用户进行使用，而且用户有没办法随意修改。
        return self._images 
    @property
    def labels(self):
        return self._labels
l = DataSet()
#用户进行属性调用的时候，直接调用images即可，而不用知道属性名_images，因此用户无法更改属性，从而保护了类的属性。
print(l.images) # 加了@property后，可以用调用属性的形式来调用方法,后面不需要加（）。
```



# 4. 如何创建包

Python 中的包（Package）和 C++ 中的命名空间（Namespace）在某种程度上具有类似的功能，都是用于组织和管理代码，避免命名冲突，提供更好的模块化和代码复用性。

## 4.1 基本步骤

1. **创建文件夹**：首先，创建一个文件夹来作为包的根目录。你可以为这个文件夹取一个有意义的名称，例如 `my_package`。
2. **在文件夹中创建 `__init__.py` 文件**：在这个包文件夹中，创建一个名为 `__init__.py` 的文件。这个文件会被视为包的初始化文件，定义包的属性、接口和初始化代码。可以是空文件，也可以包含你希望在包被导入时执行的代码。
3. **在包文件夹中创建模块文件**：在包文件夹中，可以创建多个模块文件，每个模块文件包含相关的函数、类、变量等代码。模块文件的命名可以按照你的需求进行，但最好遵循一些命名规则。

## 4.2 例子

如何定义一个名为 `my_package` 的包，其中包含两个模块 `module1.py` 和 `module2.py`：

1. 创建文件夹结构

   ```
   my_package/
   ├── __init__.py
   ├── module1.py
   └── module2.py
   ```

2. 在 `__init__.py` 文件中可以添加一些初始化代码，例如：

   ```python
   print("Initializing my_package...")
   ```

3. 在 `module1.py` 中定义一个简单的函数：

   ```python
   def greet():
       print("Hello from module1!")
   ```

4. 在 `module2.py` 中定义另一个函数：

   ```python
   def farewell():
       print("Goodbye from module2!")
   ```

5. 在其他Python文件中调用my_package包中的这2个模块

   ```python
   from my_package import module1, module2
   
   module1.greet()
   module2.farewell()
   ```

   

# 5. 路径相关

## 5.1 如何输出当前路径

有3种方法：

```python
# 1. 返回当前工作目录的绝对路径：		  /home/workspace
import os
print(os.getcwd())

# 2. 返回当前python脚本的绝对路径:		/home/workspace/test.py
import sys
print(sys.argv[0])

# 3. 返回当前python脚本的绝对路径		/home/workspace/test.py
import os
print(os.path.abspath(__file__))
```



# 6. 如何用vscode调试

## 6.1 基本流程

1. 在run and debug中点`creat a launch.json file`来创建一个名为`launch.json`配置文件

2. 修改配置文件

   ```json
   {
       "version": "0.2.0",
       "configurations": [
           {	
               // ****************** 必须项： ******************
               "name": "Python: Run My Module", 	// 配置名称，将在调试配置下拉列表中显示
               "type": "python", 					// 调试类型，这里是Python
               "request": "launch", 				// 请求类型，这里选择“launch”表示启动调试
               
               "module": "my_module", 				// 要执行的Python模块名称，请替换为实际的模块名称
               "program": "${file}",				// 要执行的python文件名称，和module只能存在一个
               "program": "${workspaceFolder}/car_instance/test.py",	// 也可以直接如此显示的指定
               
               
               "cwd": "${workspaceFolder}", 		// 当前工作目录设置为项目文件夹
               "console": "integratedTerminal", 	// 使用VSCode的集成终端显示输出
               
               // ****************** 非必须项： ******************
               "args": [], 						// 如果需要传递命令行参数，可以在这个列表中添加
               "pythonPath": "${config:python.pythonPath}", 	// 指定Python解释器的路径
               "env": {}, 							// 环境变量字典，可以在这里添加自定义环境变量
               "envFile": "${workspaceFolder}/.env", 			// 如果需要从文件加载环境变量，可以指定.env文件的路径
               "justMyCode": true, 				// 单步调试Python文件是否进入第三方库,true时不进入
               "stopOnEntry": false, 				// 是否在程序启动时立即暂停，以便在第一行代码之前设置断点
               "showReturnValue": true, 			// 是否在调试过程中显示函数的返回值
               "redirectOutput": true 				// 是否将程序输出重定向到调试控制台，而不是终端
           }
       ]
   }
   ```

   

## 6.2 一个例子：

对于执行如下shell命令：

```shell
traj=00_1
python -W ignore::UserWarning vo_trajectory_from_folder.py \
							--vo-model-name vonet.pkl \
							--seg-model-name segnet-kitti.pth \
							--kitti \
							--kitti-intrinsics-file data/DynaKITTI/$traj/calib.txt \
							--test-dir data/DynaKITTI/$traj/image_2 \
							--pose-file data/DynaKITTI/$traj/pose_left.txt 
```

launch.json为：

```json
{
    // Use IntelliSense to learn about possible attributes.
    // Hover to view descriptions of existing attributes.
    // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Python: Current File",
            "type": "python",
            "request": "launch",
            "program": "${file}",
            "console": "integratedTerminal",
            "justMyCode": true,
            "cwd": "${fileDirname}",
            "args": [
                "--vo-model-name", "vonet.pkl",
                "--seg-model-name", "segnet-kitti.pth",
                "--kitti",
                "--kitti-intrinsics-file", "data/DynaKITTI/00_1/calib.txt",
                "--test-dir", "data/DynaKITTI/00_1/image_2",
                "--pose-file", "data/DynaKITTI/00_1/pose_left.txt"
            ],
            "env": {
                "PYTHONWARNINGS": "ignore::UserWarning"
            }
        }
    ]
}
```



# 7. 多线程

## 7.1 条件变量和互斥锁

- **条件变量为什么要和互斥锁一起使用**

  锁是一种保护共享资源的机制，它可以确保同一时间只有一个线程可以访问共享资源。

  条件变量是一种线程间通信机制，它允许线程在特定条件下等待或被唤醒。

  因此，条件变量和锁的组合可以有效避免数据竞争和死锁等问题，确保线程之间的同步和互斥访问。

- **条件变量的目的**：

  用来进行线程间的同步。

- **联合使用的过程**：

  1. 条件变量被用来阻塞一个线程A，当条件不满足时，线程A往往解开相应的互斥锁并等待条件发生变化。

  2. 一旦线程B改变了这个条件变量，线程B会唤醒一个或多个被此条件变量阻塞的线程。
  3. 线程A们会重新锁定互斥锁并重新测试条件是否满足

- **互斥锁的缺点和条件变量相应的优点**：

  1. 互斥锁只有两种状态：锁定和非锁定。

     而条件变量通过允许线程阻塞和等待另一个线程发送信号的方法弥补了互斥锁的不足，它常和互斥锁一起配合使用。

  2. 互斥锁会进入忙等待：不断检查是否满足运行的条件。
  
     而条件变量不会进入忙等待，而是需要另一个线程来唤醒，这就降低了资源的占用

## 7.2 其他一些同步机制

- **信号量（Semaphore）：** 信号量是一种计数器，用于控制同时访问共享资源的线程数量。它允许多个线程进入临界区，但在达到特定数量后，其他线程需要等待。Python中有 `threading.Semaphore` 类可以使用。
- **屏障（Barrier）：** 屏障用于同步多个线程，让它们在某个点上等待，直到所有线程都到达了这个点，然后它们可以一起继续执行。Python中有 `threading.Barrier` 类可以使用。
- **事件（Event）：** 事件是一种简单的同步机制，用于线程之间的通信。一个线程可以设置事件，而其他线程可以等待这个事件的发生。一旦事件发生，等待的线程将被唤醒。Python中有 `threading.Event` 类可以使用。
- **定时器（Timer）：** 定时器用于在一定时间后触发某个操作。它常用于执行定时任务。Python中有 `threading.Timer` 类可以使用。
- **队列（Queue）：** 队列是一种线程安全的数据结构，可以用于在多个线程之间安全地传递数据。Python中有 `queue.Queue` 类可以使用。
- **读写锁（Read-Write Lock）：** 读写锁允许多个线程同时读取共享资源，但在写入时会独占资源，防止其他线程读取或写入。Python中没有直接的读写锁类，但你可以使用 `threading.Lock` 来实现。



## 7.3 python中条件变量的例子

```python
class SLAM():
    def __init__(self,config):
        self.config = config
        self.device = torch.device("cuda" if torch.cuda.is_available else "cpu")

        # *********** 多线程通信用 ***********
        self.keepRunning = True                         # 关闭slam系统时转为false
        self.newConstraintAdded = False                 # 在constraintSearch线程中为位置图添加新的顶点后会转为true
        self.newConstraintMutex = threading.Lock()      # 用于优化线程的锁
        self.newConstraintSignal= threading.Condition(self.newConstraintMutex) # 用于优化线程的条件变量


        # *********** 创建3个子线程 ***********
        self.thread_mapping = threading.Thread(target=self.mappingThreadLoop,args=())
        self.thread_constraint_search = threading.Thread(target=self.constarintSearchThreadLoop,args=())
        self.thread_optimization = threading.Thread(target=self.optimizationThreadLoop,args=())
        
        # *debug
        self.index = 0

        
    def mappingThreadLoop(self):
        print('Started mapping thread')

        
    def constarintSearchThreadLoop(self):
        print('Started constraint search thread')
		# 循环执行当前线程
        while self.keepRunning:
            time.sleep(1)
            self.index = self.index+1
            print("现在：",self.index)

            if self.index == 5:
                """
                使用with可以在退出代码块的时候，自动释放锁
                比手动调用acquire()和release()更加的安全
                """
                with self.newConstraintMutex:
                    self.newConstraintAdded = True
                    self.newConstraintSignal.notify_all()
                break

                
    def optimizationThreadLoop(self):
        print('Started optimization thread')
		# 循环执行当前线程
        while self.keepRunning:
            with self.newConstraintMutex:
                if not self.newConstraintAdded:
                    self.index = self.index + 1
                    print("等待：",self.index)
                    self.newConstraintSignal.wait()
                self.newConstraintAdded = False
            print("执行：",self.index)

            
    def run(self):
        print('Started SLAM System')
        print(self.device)
        # *********** 启动3个子线程 ***********
        self.thread_mapping.start()
        self.thread_constraint_search.start()
        self.thread_optimization.start()

cfg = config.load_config("....")
slam = SLAM(cfg)
slam.run()
```

输出结果：

```
Start running...
Started SLAM System
cuda
Started mapping thread
Started constraint search thread
Started optimization thread
等待： 1
现在： 2
现在： 3
现在： 4
现在： 5
执行： 5
等待： 6
```

# 8. 多进程

python用`multiprocessing`库来实现多进程，该库提供了 `Value`、`Array`、`String` 类型的对象用于跨进程共享数据。然而，它并没有直接提供对 `list` 和 `dict` 的支持。这是因为 `list` 和 `dict` 是可变的数据类型，直接在进程间共享可能导致复杂的同步和锁定问题。

但可以通过 `multiprocessing.Manager` 创建一个共享的 `Manager` 对象，然后使用它来创建共享的 `list` 或 `dict`。这样可以确保适当的同步和锁定机制被应用

例子：

一个SLAM类，类的run函数会生成3个子进程，分别运行不同函数，他们之间会有 Array、Value、list、dict 和 String 类型的共享变量

```python
from multiprocessing import Process, Manager, Value, Array

class SLAM:
    def __init__(self, config):
        self.config = config

    def modify_shared_data(self, shared_num, shared_list, shared_dict, shared_str):
        shared_num.value = 3.1415927
        shared_list.append(42)
        shared_dict['key'] = 'value'
        shared_str.value = 'Hello, shared string!'

    def process_function_1(self, shared_array):
        for i in range(len(shared_array)):
            shared_array[i] *= 2

    def process_function_2(self, shared_list):
        shared_list.extend([10, 20, 30])

    def process_function_3(self, shared_dict):
        shared_dict['new_key'] = {'nested_key': 'nested_value'}

    def run(self):
        with Manager() as manager:
            num = Value('d', 0.0)
            arr = Array('i', range(5))
            lst = manager.list(range(5))
            dct = manager.dict({'key': 'initial_value'})
            string = manager.Value('c', b'initial_string')

            p1 = Process(target=self.modify_shared_data, args=(num, lst, dct, string))
            p2 = Process(target=self.process_function_1, args=(arr,))
            p3 = Process(target=self.process_function_2, args=(lst,))
            p4 = Process(target=self.process_function_3, args=(dct,))

            p1.start()
            p2.start()
            p3.start()
            p4.start()

            p1.join()
            p2.join()
            p3.join()
            p4.join()

            print("Modified Number:", num.value)
            print("Modified Array:", arr[:])
            print("Modified List:", lst)
            print("Modified Dictionary:", dct)
            print("Modified String:", string.value)

if __name__ == '__main__':
    slam_instance = SLAM(config={})  # Pass your configuration as needed
    slam_instance.run()

```

# 9.with语句和上下文管理器ContextManager

- 上下文管理器Context Managers是指

  在一段代码执行之前执行一段代码，用于一些预处理工作，

  执行之后再执行一段代码，用于一些清理工作。

  - 使用场景：

    1.打开文件进行读写，读写完之后需要将文件关闭。

    2.在数据库操作中，操作之前需要连接数据库，操作之后需要关闭数据库。

  - 如何实现：

    在类里定义`__enter__()`和`__exit__()`两个函数

- with语句

  通过with来使用上下文管理器，确保在进入和退出代码块时执行必要的操作

- 运行逻辑：

  ```
  with EXPR as VAR:
      BLOCK
  ```

  1. 执行EXPR语句，获取上下文管理器（Context Manager）
  2. 调用上下文管理器中的`__enter__`方法，该方法执行一些预处理工作。
  3. 这里的`as VAR`可以省略，如果不省略，则将`__enter__`方法的返回值赋值给`VAR`。
  4. 执行代码块BLOCK，这里的VAR可以当做普通变量使用。
  5. 最后调用上下文管理器中的的`__exit__`方法。
     - `__exit__`方法有三个参数：exc_type, exc_val, exc_tb。如果代码块BLOCK发生异常并退出，那么分别对应异常的type、value 和 traceback。否则三个参数全为None。
     - `__exit__`方法的返回值可以为True或者False。如果为True，那么表示异常被忽视，相当于进行了try-except操作；如果为False，则该异常会被重新raise。

- 用上下文管理器实现：打开文件操作

  ```python
  # 自定义打开文件操作
  class MyOpen(object):
  
      def __init__(self, file_name):
          """初始化方法"""
          self.file_name = file_name
          self.file_handler = None
          return
  
      def __enter__(self):
          """enter方法，返回file_handler"""
          print("enter:", self.file_name)
          self.file_handler = open(self.file_name, "r")
          return self.file_handler
  
      def __exit__(self, exc_type, exc_val, exc_tb):
          """exit方法，关闭文件并返回True"""
          print("exit:", exc_type, exc_val, exc_tb)
          if self.file_handler:
              self.file_handler.close()
          return True
  
  # 使用实例
  with MyOpen("python_base.py") as file_in:
      for line in file_in:
          print(line)
          raise ZeroDivisionError
  # 代码块中主动引发一个除零异常，但整个程序不会引发异常
  ```

  
