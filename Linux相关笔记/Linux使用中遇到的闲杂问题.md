# 常用命令

## 帮助文档

- `man tar`linux自带的

- `tldr tar`第三方更好用

  需要自己下载:

  - pip3 install tar
  - sudo apt install tldr 

## ls命令

- ls ：查看当前所在文件夹 

| 参数 | 含义                                                   |
| ---- | ------------------------------------------------------ |
| -a   | 显示指定目录下所有子目录与文件，包括隐藏文件           |
| -l   | 以列表方式显示文件的详细信息                           |
| -h   | 配合-l以人性化的方式显示文件的大小，必须配合-l一起使用 |

## cd命令
- cd 切换文件夹

| 命令  | 含义                                               |
| ----- | -------------------------------------------------- |
| cd    | 切换到当前用户的主目录(/home/用户目录)             |
| cd ~  | 切换到当前用户的主目录(/home/用户目录) ~表示家目录 |
| cd .  | 保持在当前目录不变                                 |
| cd .. | 切换到上级目录                                     |
| cd -  | 可以在最近两次工作目录来回切换                     |

## touch命令

- 创建文件或修改文件时间
    - 如果文件不存在，创建一个空白的文件
    - 如果文件已存在，修改该文件的末次修改日期。

## mkdir命令
- 创建一个新的目录  
但新建的目录名不能和已有的目录或文件同名

| 选项 | 含义             |
| ---- | ---------------- |
| -p   | 可以递归创建目录 |

```
如：mkdir -p a1/b/c 与下代码效果相同：
        mkdir a1
        cd a1
        mkdir b
        cd b
        mkdir c
```
## rm命令
- 删除文件或目录  
注意！：会直接删除不能恢复

| 选项 | 含义                                             |
| ---- | ------------------------------------------------ |
| -f   | 强制删除，忽略不存在的文件，无需提示             |
| -r   | 递归的删除目录下的内容，删除文件夹时必须加此参数 |

        如：
        有一个a1的文件夹
        yang@yang-virtual-machine:~/Desktop$ rm a1
        rm: cannot remove 'a1': Is a directory
        yang@yang-virtual-machine:~/Desktop$ rm -r a1     就删掉了
        
        yang@yang-virtual-machine:~/Desktop$ rm abc
        rm: cannot remove 'abv': No such file or directory      正常删除，会告诉你没这个文件
        yang@yang-virtual-machine:~/Desktop$ rm -f abc          强制删除，没有不会说，有的话就直接删除
## 拷贝和移动文件
| 序号 | 命令               | 对应英文 | 作用                                |
| ---- | ------------------ | -------- | ----------------------------------- |
| 01   | tree[目录名]       | tree     | 以树状图列出文件目录结构            |
| 02   | cp 源文件 目标文件 | copy     | 复制文件或者目录                    |
| 03   | mv 源文件 目标文件 | move     | 移动文件或目录  /  文件或目录重命名 |

### tree
- tree命令可以以树状图列出文件的目录结构  

| 选项 | 含义                  |
| ---- | --------------------- |
| -d   | 只显示目录,不显示文件 |

        如；
        yang@yang-virtual-machine:~$ tree
        .
        ├── Desktop
        ├── Documents
        ├── Downloads
        ├── examples.desktop
        ├── Music
        ├── Pictures
        ├── Public
        ├── Templates
        └── Videos

### cp
- cp命令的功能是将给出的文件或目录复制到另一个文件或目录中，相当于DOS下的copy命令  
但不能直接复制目录，要复制目录需要加选项-r

| 选项 | 含义                                                         |
| ---- | ------------------------------------------------------------ |
| -f   | 已经存在的目标文件直接覆盖，不会提示                         |
| -i   | 如已有同名文件会提示是否覆盖，回复y是，回复n不覆盖           |
| -r   | 若给出的源文件是目录，则cp命令将递归复制该目录下所有的子目录和文件。目标文件也必须为一个目录 |


        如：
        yang@yang-virtual-machine:~/Desktop$ cp ~/Documents/read\ me  ./readme.txt
        将家目录下的文件 复制到了桌面上

### mv移动、重命名
- mv命令可以用来移动文件或目录，也可以给文件或目录重命名。
- 在用mv命令时，一定要加-i，防止覆盖掉重要的文件

| 选项 | 含义                                                         |
| ---- | ------------------------------------------------------------ |
| -i   | 覆盖文件前提示，因为重命名为或移动到到一个有相同名字的文件，那会覆盖他，这样不安全。 |

        移动：地址不同时
        yang@yang-virtual-machine:~$ mv ./Desktop/readme.txt ~/Documents/
        把桌面删的readme.txt文件移动到documents目录中去
        
        重命名：地址相同时
        yang@yang-virtual-machine:~/Desktop$ mv ./123.txt ./222.txt

## 查看文件内容
| 序号 | 命令                 | 对应英文    | 作用                                                 |
| ---- | -------------------- | ----------- | ---------------------------------------------------- |
| 01   | cat 文件名           | concatenate | 查看文件内容、创建文件、文件合并、追加文件内容等功能 |
| 02   | more 文件名          | more        | 分屏显示文件内容                                     |
| 03   | grep 搜索文本 文件名 | grep        | 搜索文件中含有搜索文本的所有行                       |

### cat
- cat 命令会一次性显示所有内容，适合用来查看内容较少的文本文件

| 选项 | 含义               |
| ---- | ------------------ |
| -b   | 给非空输出行编号   |
| -n   | 给输出的所有行编号 |

Linux中还有个nl 命令和 cat -b 效果一致

        如：
        yang@yang-virtual-machine:~$ cat ./Desktop/222.txt 


### more
- more 命令会分屏显示文件内容，每次只显示一页内容，适合用于查看内容较多的文本文件

| 操作键  | 功能           |
| ------- | -------------- |
| 空格键  | 显示下一屏     |
| Enter键 | 一次滚动一行   |
| b       | 回滚一屏       |
| f       | 前滚一屏       |
| /word   | 搜索word字符串 |

        如：
        yang@yang-virtual-machine:~$ more ./Desktop/222.txt 

### grep
- grep 命令允许对文本进行模式查找。模式查找即正则表达式

| 选项 | 含义                                   |
| ---- | -------------------------------------- |
| -n   | 显示匹配行及行号                       |
| -v   | 显示不包括匹配文本的所有行(相当于求反) |
| -i   | 忽略大小写                             |

常用的两种模式查找  

| 参数 | 含义                                               |
| ---- | -------------------------------------------------- |
| ^a   | 模式：指定的文本出现在一行的行首。搜索以a开头的行  |
| ke$  | 模式：指定的文本出现在一行的行尾。搜索以ke结束的行 |

        如：
        yang@yang-virtual-machine:~/Desktop$ grep -n ef 222.txt
        
        模式查找：
        yang@yang-virtual-machine:~/Desktop$ grep -n ^ef 222.txt

## 重定向
### echo
- echo 会在终端中再显示一遍输入的文字，通常和**重定向**联合使用

        如：
        yang@yang-virtual-machine:~/Desktop$ echo Liebe
        Liebe
### 重定向 > 和 >>
- Linux允许将命令执行结果重定向到一个文件，将本应在终端上显示的内容输出或追加到指定文件中
其中
- **>** 表示输出，会覆盖文件原有的内容
- **>>** 表示追加，会将内容追加到已有文件的末尾

        如：
        yang@yang-virtual-machine:~/Desktop$ echo Ich liebe dich > a
        
        yang@yang-virtual-machine:~/Desktop$ ls -lh
        total 8.0K
        -rw-rw-r-- 1 yang yang 189 12月 21 06:45 222.txt
        -rw-rw-r-- 1 yang yang  15 12月 21 07:15 a
        yang@yang-virtual-machine:~/Desktop$ ls -lh >>a
        yang@yang-virtual-machine:~/Desktop$ cat a
        Ich liebe dich
        total 8.0K
        -rw-rw-r-- 1 yang yang 189 12月 21 06:45 222.txt
        -rw-rw-r-- 1 yang yang  15 12月 21 07:15 a

## strace命令

- 来跟踪进程执行时系统调用和所接收的信号

  ```
  strace ls -la	//跟踪命令ls -la执行时所用到的系统调用
  ```

## 管道 |
-  Linux允许将 一个命令的输出 通过 管道| 做为 另一个命令的输入
-  可以理解为现实生活中的管子，管子一头塞入东西（写），另一头取出（读），中间时管段（|）  
-  常用的管道命令有：  
more和grep


        如：
        yang@yang-virtual-machine:~$ ls -lha | more
        将所有文件和目录以分屏的形式显示出来

# 各种场景

## 截图快捷键

- `PrtSc` – 获取整个屏幕的截图并保存到 Pictures 目录。
- `Shift + PrtSc` – 获取屏幕的某个区域截图并保存到 Pictures 目录。
- `Alt + PrtSc` –获取当前窗口的截图并保存到 Pictures 目录。
- `Ctrl + PrtSc` – 获取整个屏幕的截图并存放到剪贴板。
- `Shift + Ctrl + PrtSc` – 获取屏幕的某个区域截图并存放到剪贴板。
- `Ctrl + Alt + PrtSc` – 获取当前窗口的 截图并存放到剪贴板。

## 后台运行程序nohup

- nohup命令

  no hang up不挂起。在命令行中通过./filename运行某个程序，当该命令行退出时，这个程序也会关闭。需要永久在线的程序，如博客就需要后台运行了。

  - 最终命令一般形式`nohup command >out.file 2>&1 &`
    - 原来输出是打印在命令行，现在输出到了out.file文件里
    - 末尾的& 表示后台运行
    - 1，表示文件描述符1，表示标准输出
    - 2，表示文件描述符2，表示标准错误输出
    - 2>&1,表示 标准输出和标准错误输出 合并输出（到out.file里去）。
  - 关闭该进程`ps -ef |grep 关键字  |awk '{print $2}'|xargs kill -9`
    - ps -ef 可以单独执行，列出所有正在运行的程序
    - awk 工具可以很灵活地对文本进行处理，这里的 awk '{print $2}'是指第二列的内容，是运行的程序 ID
    - 通过 xargs 传递给 kill -9，也就是发给这个运行的程序一个信号，让它关闭。
    - 如果你已经知道运行的程序 ID，可以直接使用 kill 关闭运行的程序

## 查看内存占用情况

- 方法1：`ps aux --sort -rss`: 列出目前所有的正在内存当中的程序

  - USER：该 process 属于那个使用者账号的

  - PID ：该 process 的号码

  - %CPU：该 process 使用掉的 CPU 资源百分比

  - %MEM：该 process 所占用的物理内存百分比

  - VSZ ：该 process 使用掉的虚拟内存量 (Kbytes)

  - RSS ：该 process 占用的固定的内存量 (Kbytes)

  - TTY ：该 process 是在那个终端机上面运作，若与终端机无关，则显示 ?，另外， tty1-tty6 是本机上面的登入者程序，若为 pts/0 等等的，则表示为由网络连接进主机的程序。

  - STAT：该程序目前的状态，主要的状态有

    - R ：该程序目前正在运作，或者是可被运作

    - S ：该程序目前正在睡眠当中 (可说是 idle 状态)，但可被某些讯号 (signal) 唤醒。

    - T ：该程序目前正在侦测或者是停止了

    - Z ：该程序应该已经终止，但是其父程序却无法正常的终止他，造成 zombie (疆尸) 程序的状态

  - START：该 process 被触发启动的时间

  - TIME ：该 process 实际使用 CPU 运作的时间

  - COMMAND：该程序的实际指令
- 方法2：`vmstat -s`：显示实时的和平均的统计，覆盖CPU、内存、I/O等内容。例如内存情况，不仅显示物理内存，也统计虚拟内存。
- 方法3：`top`：
  - PID：当前运行进程的ID
  - USER：进程属主
  - PR：每个进程的优先级别
  - NInice：反应一个进程“优先级”状态的值，其取值范围是-20至19，一共40个级别。这个值越小，表示进程”优先级”越高，而值越大“优先级”越低。一般会把nice值叫做静态优先级
  - VIRT：进程占用的虚拟内存
  - RES：进程占用的物理内存
  - SHR：进程使用的共享内存
  - S：进程的状态。
    - S表示休眠，
    - R表示正在运行
    - Z表示僵死状态
    - N表示该进程优先值为负数
  - %CPU：进程占用CPU的使用率
  - %MEM：进程使用的物理内存和总内存的百分比
  - TIME+：该进程启动后占用的总的CPU时间，即占用CPU使用时间的累加值。
  - COMMAND：进程启动命令名称

## 查看磁盘占用情况

- `df -k`：以KB为单位显示磁盘使用量和占用率
- `df -m`：以Mb为单位显示磁盘使用量和占用率

## 查看某个文件夹的大小

```
du -sh /path/to/folder
```

- `-s` 选项表示只显示总和，而不显示每个子文件夹的大小。
- `-h` 选项表示以人类可读的格式显示结果，即以 K、M、G 等单位显示文件夹大小。
- `/path/to/folder` 是要查看大小的文件夹的路径。

## Linux手册页

可以用`man`命令显示手册页(manual pages)，提供关于命令、库函数、系统调用、配置文件等的详细信息。手册页按章节组织，通常包括以下几章:

1. 用户命令
2. 系统调用
3. 库函数
4. 特殊文件（通常是设备节点）
5. 文件格式和约定
6. 游戏和杂项
7. 杂项（包括惯例、协议和标准等）
8. 系统管理命令和守护进程

- 打开位于第七章信号的手册页：

  ```
  man 7 signal
  ```

- 打开某个命令的手册页：

  ```
  man mkdir
  ```

## 打开linux的性能模式

- 检查当前cpu的频率：

  ```
  sudo apt install cpufrequtils
  cpufreq-info
  ```

- 改变cpu频率

  ```
  sudo apt install indicator-cpufreq
  ```

  - 然后重启在右上角手动更改性能模式

## 分屏

在官网上安装[Tactile](https://extensions.gnome.org/extension/4548/tactile/)

- 打开布局：win + t
- 切换布局：win+t -> 1/2/3
- 占多块：win+t -> 按顺序按想要的区域字母

## 设置linux组

**组** 是指一组具有相同特征或权限的用户或进程的集合。组可以用于管理用户对系统资源的访问权限，也可以用于将用户组织到逻辑组中以便更轻松地进行管理。

### 查看当前组

使用命令`cat /etc/group`查看所有的组，包含当前用户不属于的组。

使用命令`groups`查看当前用户属于哪些组，也可以用`id`命令

```
yang adm cdrom sudo dip plugdev lpadmin lxd sambashare realtime
```

各个组的含义为：

- adm

  `adm` 组是系统管理员组，成员拥有对系统进行管理的权限，包括：

  - 添加和删除用户
  - 修改用户密码
  - 管理组
  - 修改系统配置
  - 安装和卸载软件

- cdrom

  `cdrom` 组是 CD-ROM 驱动程序组，成员可以使用 CD-ROM 驱动程序。

- sudo

  `sudo` 组是允许用户以管理员身份运行命令的组，成员可以使用 `sudo` 命令以管理员身份运行任何命令

- dip

  `dip` 组是直接 I/O 权限组，成员可以使用直接 I/O 访问硬件设备。

- plugdev

  `plugd`hare 组是 Samba 共享组，成员可以使用 Samba 共享文件和打印机。

  realtime

  realtime 组是实时任务组，成员可以运行实时任务。

  将当前用户添加到某个组
  将yang添加到sudo组

  ￼
  sudo usermod -aG sudo yang
  也可以用：

  ￼
  sudo gpasswd -a yang sudo`ev` 组是可插拔设备组，成员可以使用可插拔设备，例如 USB 设备。

- lpadmin

  `lpadmin` 组是打印机管理员组，成员可以管理打印机。

- lxd

  `lxd` 组是 LXD 容器管理组，成员可以使用 LXD 命令管理容器。

- sambashare

  `sambashare` 组是 Samba 共享组，成员可以使用 Samba 共享文件和打印机。

- realtime

  `realtime` 组是实时任务组，成员可以运行实时任务。

### 将当前用户添加到某个组

将yang添加到sudo组

```
sudo usermod -aG sudo yang
```

也可以用：

```
sudo gpasswd -a yang sudo
```



# 各种安装

## 双系统安装

### 1. 删除双系统

https://blog.csdn.net/qq_43310597/article/details/105782722

### 2.安装双系统

https://zhuanlan.zhihu.com/p/363640824

### 3. 安装real time kernel

参考1：https://www.cnblogs.com/Pyrokine/p/16695196.html

参考2：https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel

### 4. 安装遇到的问题汇总

- Ubuntu分区：

  - `/boot`: 逻辑分区2048MB,ext4 (固态)
  - `/swap`: 逻辑分区32GB.交换分区 (固态)
  - `/`:主分区，所有剩下的容量.ext4 (机械)

- BusyBox v1.30.1(Debin 1:1.30.1-4) built-in shell(ash)的提示信息，无法正常开机

  - `fsck -y /dev/sda3(替换成自己的，我的是sda3)`的命令成功执行后会出现file system was modified字样
  - 然后输入exit退出

- 升级ubuntu版本时遇到的问题

  - 如何升级

  ```
  ~$ update-manager -c -d 
  ```

  - ubuntu升级时磁盘空间 /boot 不足
    1. ~$ df -h：查看磁盘存储情况
    2. ~$ uname -a： 查看当前使用内核版本
    3. ~$ sudo apt-get remove linux-image-： 查看所有的内核版本
    4. ~$ sudo apt-get remove linux-image-4.18.0-25-generic：删除对应版本

## 安装 Terminator

terminator:`sudo apt-get install terminator`

美化：`sudo gedit ~/.config/terminator/config`并输入

```
[global_config]
  handle_size = -3
  title_transmit_fg_color = "#000000"
  title_transmit_bg_color = "#3e3838"
  inactive_color_offset = 1.0
  enabled_plugins = CustomCommandsMenu, LaunchpadCodeURLHandler, APTURLHandler, LaunchpadBugURLHandler
  suppress_multiple_term_dialog = True
[keybindings]
[profiles]
  [[default]]
    background_color = "#2e3436"
    background_darkness = 0.8
    background_type = transparent
    cursor_shape = ibeam
    cursor_color = "#e8e8e8"
    font = Ubuntu Mono 14
    foreground_color = "#e8e8e8"
    show_titlebar = False
    scroll_background = False
    scrollback_lines = 3000
    palette = "#292424:#5a8e1c:#00ff00:#cdcd00:#1e90ff:#cd00cd:#00cdcd:#d6d9d4:#4c4c4c:#868e09:#00ff00:#ffff00:#4682b4:#ff00ff:#00ffff:#ffffff"
    use_system_font = False
[layouts]
  [[default]]
    [[[child1]]]
      parent = window0
      profile = default
      type = Terminal
    [[[window0]]]
      parent = ""
      size = 925, 570
      type = Window
[plugins]
```

## 安装GNOME插件

[参考](https://wiki.gnome.org/action/show/Projects/GnomeShellIntegration/Installation?action=show&redirect=Projects%2FGnomeShellIntegrationForChrome%2FInstallation)

1. 安装本地扩展管理器

    ```
    sudo apt install gnome-shell gnome-tweaks
    sudo apt install gnome-shell-extensions
    ```

2. 先安装谷歌连接器

    ```
    sudo apt-get install chrome-gnome-shell
    ```

3. 查看gnome version

   ```
   gnome-shell --version
   ```

   ubuntu20的话是3.36.9

4. 在[官网插件](https://extensions.gnome.org/extension/3733/tiling-assistant/)上下点按钮下载即可


