# 一些快捷键

## 切换关闭标签页

- **ctrl + F4**: 关闭标签页
- **ctrl + tab**： 切到下一标签页
- **ctrl + shift + tab**： 切到上一标签页

## 搜索

- **两下shift**: 在所有文件、符号、git分支等等全局中搜索
- **ctrl+N**: 只搜索类
- **ctrl+shift+N**：只搜索文件
- **ctrl+alt+shift+N**: 只搜索符号(函数名、变量名)
- **ctrl+shift+A**: 只搜索操作actions（重构、运行配置等clion中各种命令）

## 全局修改变量名

shift + F6

## 删除

- 删除行：ctrl + Y

## 快速移动多行代码

1. 按住shift选中多行代码
2. 按住ctrl + shift上下移动

## 在.cpp中自动创建.h中定义的函数

在.h定义的函数处按:  alt+enter

# 安装插件

1. ctrl+shift+A 搜索plugin

2. 打开后安装想要的插件

   

# 文件类型识别

问题：如Dockerfile会被自动识别为docker文件并被高亮，但Dockerfile.ros就不会被自动识别没有被高亮。

解决：

1. ctrl+shift+A 搜索file types
2. 选择dockerfile，在file name patterns中加入Dockerfile.ros即可。

其他文件识别同理。



# 远程开发

## 连接到docker环境

1. 打开settings/toolchains, 创建一个新的toolchain：点+，然后选Docker

   > toolchain：包含所有用于构建和运行应用的工具
   >
   > CMake: build tool, c/c++ compilers
   >
   > Dbugger

2. 设置toolchain：

   - Server: Docker 
   - Image: 选择container对应的镜像
   - Container setting：可以设置端口绑定、卷绑定，和环境变量
     - 端口绑定：用于将容器内部端口映射到主机的端口
     - 卷绑定：将主机的文件/目录 挂载到Docker容器内的目录中。

3. 设置cmake

   就是把cmake .. -Dxxxx 这些xxxx写通过Gui设置进去了

## 连接到远程主机

1. 打开settings/toolchains, 创建一个新的toolchain：点+，然后选remote host

2. 点击credentials创建连接

   - Host: 填远程主机ip地址

     在远程主机用ifconfig来查看ip地址

   - Username + password

3. 映射文件夹：

   - 打开settings/deployment，选择刚创建的远程服务器。
   - 在mappings中，在local path中填入本地文件夹地址，在deployment path中填入远程服务器项目的文件夹地址。
