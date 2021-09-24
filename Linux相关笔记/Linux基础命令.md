# 关机和重启

- `shutdown -h now`现在就关机

- `reboot`重启

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

## tree
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

## cp
- cp命令的功能是将给出的文件或目录复制到另一个文件或目录中，相当于DOS下的copy命令  
但不能直接复制目录，要复制目录比较加选项-r

| 选项 | 含义                                                         |
| ---- | ------------------------------------------------------------ |
| -f   | 已经存在的目标文件直接覆盖，不会提示                         |
| -i   | 如已有同名文件会提示是否覆盖，回复y是，回复n不覆盖           |
| -r   | 若给出的源文件是目录，则cp命令将递归复制该目录下所有的子目录和文件。目标文件也必须为一个目录 |


        如：
        yang@yang-virtual-machine:~/Desktop$ cp ~/Documents/read\ me  ./readme.txt
        将家目录下的文件 复制到了桌面上

## mv移动、重命名
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

### 查看文件内容
| 序号 | 命令                 | 对应英文    | 作用                                                 |
| ---- | -------------------- | ----------- | ---------------------------------------------------- |
| 01   | cat 文件名           | concatenate | 查看文件内容、创建文件、文件合并、追加文件内容等功能 |
| 02   | more 文件名          | more        | 分屏显示文件内容                                     |
| 03   | grep 搜索文本 文件名 | grep        | 搜索文件中含有搜索文本的所有行                       |

## cat
- cat 命令会一次性显示所有内容，适合用来查看内容较少的文本文件

| 选项 | 含义               |
| ---- | ------------------ |
| -b   | 给非空输出行编号   |
| -n   | 给输出的所有行编号 |

Linux中还有个nl 命令和 cat -b 效果一致

        如：
        yang@yang-virtual-machine:~$ cat ./Desktop/222.txt 


## more
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

## grep
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

# 管道 |
-  Linux允许将 一个命令的输出 通过 管道| 做为 另一个命令的输入
-  可以理解为现实生活中的管子，管子一头塞入东西（写），另一头取出（读），中间时管段（|）  
-  常用的管道命令有：  
more和grep


        如：
        yang@yang-virtual-machine:~$ ls -lha | more
        将所有文件和目录以分屏的形式显示出来

# 后台运行程序nohup

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
