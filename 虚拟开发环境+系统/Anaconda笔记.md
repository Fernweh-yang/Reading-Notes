# 简介

Anaconda是一个环境管理软件，解决不同代码需要不同的环境时重复卸载安装环境的需求。

# 安装与卸载

## 1. 安装

- 下载包：https://docs.conda.io/en/latest/miniconda.html
- 双击运行下载(不添加环境变量)

## 2. 卸载

- 在`/home`下删除`miniconda`文件夹

## 3. Windows环境配置

将如下地址加入环境变量

- anaconda安装的目录 D:\Download\ANACONDA

- anaconda/scripts的目录 D:\Download\ANACONDA\Scripts

- bin的目录 D:\Download\ANACONDA\Scripts\Library\bin

# 基本指令

- 查看版本号
  - `conda --version`
  - `conda -V`
  
- 更新Anaconda

  - 先更新conda:`conda update conda`
  - 在更新Anaconda: ``

- 查看当前系统下有什么conda环境
  - `conda env list`
  - `conda info --envs`
  
- 新建conda环境
  - `conda create -n [环境名]`
  
- 激活某一个环境
  - `conda activate environment`
  
- 退出该环境
  - `conda deactivate`
  
- 查看环境中所装的包
  - `conda list`

- 安装包

  - `conda install pkgs[==version]`不指定版本则下载最新版

  可以在激活环境时，直接制定python版本：

  - `conda create -n test_env python==3.8`

- 卸载包

  - `conda uninstall xx`

- 升级包

  - `conda update xx`

- conda安装本地软件包

  - `conda install --use-local package.tar.bz2`

# 环境相关的指令

- 克隆一个环境

  - `conda create -n new_env --clone old_env`
  
- 删除环境

  - `conda remove -n environment --all`
  
- 改名字：

  通过克隆的方式实现

  conda create --name python32（新名字） --clone python321（老名字）

  conda remove --name python321 --all

- 导出环境

  - `conda env export > environment.yml`
  
- 根据yml文件快速创建环境

  - `conda env create -f environment.yml`

- 用txt一键安装我们所需要的环境
  - 在激活某一环境后：`pip install -r requirements.txt`

# Jupyter相关

## 1. 安装

### 1.1jupyter安装

`conda install jupyter`

### 1.2插件安装

https://github.com/ipython-contrib/jupyter_contrib_nbextensions
第一步：conda install -c conda-forge jupyter_contrib_nbextensions
第二步：jupyter contrib nbextension install --user
第三步：conda install -c conda-forge jupyter_nbextensions_configurator

## 2. 删除核

1. 查看所有核心:

   ```
   jupyter kernelspec list
   ```

2. 卸载指定核心

   ```
   jupyter kernelspec remove tensorflow-gpu
   ```

# conda install VS pip install
- conda install xxx：这种方式安装的库都会放在.conda/pkgs目录下，这样的好处就是，当在某个环境下已经下载好了某个库，再在另一个环境中还需要这个库时，就可以直接从pkgs目录下将该库复制至新环境而不用重复下载。
- pip install xxx：分两种情况，一种情况就是当前conda环境的python是conda安装的，和系统的不一样，那么xxx会被安装到.conda/envs/current_env/lib/python3.x/site-packages文件夹中，如果当前conda环境用的是系统的python，那么xxx会通常会被安装到~/.local/lib/python3.x/site-packages文件夹中
- 使用site模块查看安装路径的优先级  
  `python -m site`  
  得到:  
  ```
  sys.path = [
    '/home/xuy1fe/Desktop/Ball_plate/Work_Space/RL-learning/rl-baselines3-zoo',
    '/home/xuy1fe/.conda/envs/RL-learning/lib/python38.zip',
    '/home/xuy1fe/.conda/envs/RL-learning/lib/python3.8',
    '/home/xuy1fe/.conda/envs/RL-learning/lib/python3.8/lib-dynload',
    '/home/xuy1fe/.local/lib/python3.8/site-packages',
    '/home/xuy1fe/.conda/envs/RL-learning/lib/python3.8/site-packages',
  ]
  USER_BASE: '/home/xuy1fe/.local' (exists)
  USER_SITE: '/home/xuy1fe/.local/lib/python3.8/site-packages' (exists)
  ENABLE_USER_SITE: True
  
  ```
  这里的USER_BASE 和USER_SITE其实就是用户自定义的启用Python脚本和依赖安装包的基础路径，从上面的输出可以看到，import xxx时，先找的是当前文件夹然后是.conda/pkgs目录.
