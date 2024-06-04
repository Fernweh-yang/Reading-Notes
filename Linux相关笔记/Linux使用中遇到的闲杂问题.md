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
- 权限问题
   - /opt相当于D:/software.但没法创创建新文件夹
   ```
   ~$ sudo chmod 777 /opt
   ```
- 帮助文档  
  以mkdir为例，2种方法
   - mkdir --help
   - man mkdir
  
- ubuntu 18.04没法全屏
   - sudo apt-get update
   - sudo apt-get install open-vm-tools
   
- Ubuntu分区：

   - `/boot`: 逻辑分区2048MB,ext4 (固态)
   - `/swap`: 逻辑分区32GB.交换分区 (固态)
   - `/`:主分区，所有剩下的容量.ext4 (机械)
   
- BusyBox v1.30.1(Debin 1:1.30.1-4) built-in shell(ash)的提示信息，无法正常开机

   - `fsck -y /dev/sda3(替换成自己的，我的是sda3)`的命令成功执行后会出现file system was modified字样
   - 然后输入exit退出
   
- 如何截图

   - `PrtSc` – 获取整个屏幕的截图并保存到 Pictures 目录。
   - `Shift + PrtSc` – 获取屏幕的某个区域截图并保存到 Pictures 目录。
   - `Alt + PrtSc` –获取当前窗口的截图并保存到 Pictures 目录。
   - `Ctrl + PrtSc` – 获取整个屏幕的截图并存放到剪贴板。
   - `Shift + Ctrl + PrtSc` – 获取屏幕的某个区域截图并存放到剪贴板。
   - `Ctrl + Alt + PrtSc` – 获取当前窗口的 截图并存放到剪贴板。


# 双系统安装

## 1. 删除双系统

https://blog.csdn.net/qq_43310597/article/details/105782722

## 2.安装双系统

https://zhuanlan.zhihu.com/p/363640824

## 3. 安装real time kernel

参考1：https://www.cnblogs.com/Pyrokine/p/16695196.html

参考2：https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel

# 安装基础软件

## 1. Terminator

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

# 如何打开linux的性能模式

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

# 安装GNOME插件

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

    

# 分屏

在官网上安装[Tactile](https://extensions.gnome.org/extension/4548/tactile/)

- 打开布局：win + t
- 切换布局：win+t -> 1/2/3
- 占多块：win+t -> 按顺序按想要的区域字母

# linux组

**组** 是指一组具有相同特征或权限的用户或进程的集合。组可以用于管理用户对系统资源的访问权限，也可以用于将用户组织到逻辑组中以便更轻松地进行管理。

## 查看当前组

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

  `plugdev` 组是可插拔设备组，成员可以使用可插拔设备，例如 USB 设备。

- lpadmin

  `lpadmin` 组是打印机管理员组，成员可以管理打印机。

- lxd

  `lxd` 组是 LXD 容器管理组，成员可以使用 LXD 命令管理容器。

- sambashare

  `sambashare` 组是 Samba 共享组，成员可以使用 Samba 共享文件和打印机。

- realtime

  `realtime` 组是实时任务组，成员可以运行实时任务。

## 将当前用户添加到某个组

将yang添加到sudo组

```
sudo usermod -aG sudo yang
```

也可以用：

```
sudo gpasswd -a yang sudo
```

