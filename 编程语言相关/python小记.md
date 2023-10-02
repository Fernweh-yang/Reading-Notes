# 1.如何打包python项目

- 资料总结：
  - https://www.osgeo.cn/python-packaging/tutorials/packaging-projects.html

- 下面以打包一个example_pkg项目为例子：

## 1.1 创建项目文件

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

## 1.2 创建包文件

```shell
├─packaging
│  │ LICENSE
│  │ README.md
│  │ setup.py
│  │
│  └─example_pkg
│          __init__.py
```



### 1.2.1 setup.py

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

### 1.2.2 Readme.md

1.2.1中提到的long_description:

```markdown
# Example Package
This is a simple example package.
```

### 1.2.3 LICENSE

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



## 1.3 得到分配包

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

   

   

## 1.4 发布包

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

## 1.5 下载包

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

## 1.6 本地卸载

```shell
pip uninstall example-pkg-yang-test	# 1.2.1中定义的包名
```

## !!!如果不需要发布

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

