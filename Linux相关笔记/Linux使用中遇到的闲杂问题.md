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

