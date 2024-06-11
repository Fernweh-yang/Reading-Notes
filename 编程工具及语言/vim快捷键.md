学习网站：https://github.com/dofy/learn-vim

- 配置文件vimrc:    
  sudo vim /etc/vim/vimrc
- 如何用vim打开文件
   1. 直接再shell里 vim 文件名1 文件名2 
   2. 打开vim后，:e 地址/文件名
- 从vim切换回shell
   - 输入:shell 
   - 从shell退回vim，就在shell里输入exit
- 多窗口显示
   - :sp 打开一个新的水平切分的窗口
   - :vsplit 打开一个新的垂直切分的窗口
   - :bn/bp 切换下/上一个已打开的文件； :b1/2/3..打开第几个文件
   - ctrl+6 来回切换2个文件
   - :close 关闭窗口
   - ctrl +w + h/j/k/l/w 切换左/上/下/右/下一个窗口
- 保存
   - :w 保存 :q!强制退出 :q(已保存)退出    -> :wq 保存并退出
   - ZZ 保存并退出， ZQ不保存退出
- 复制删除黏贴
   - v+光标移动 （按字符选择）高亮选中所要的文本
   - y 复制v选中的内容；yy复制整行
   - d删除v选中的内容; dd删除整行
   - p粘贴到光标之后位置  ； P 粘贴到光标之前的位置
   - 选中全部删除：
     - `gg`：将光标移动到文件的开头。
     - `V`：进入可视行模式。
     - `G`：将光标移动到文件的结尾，并选择所有内容。
     - 在可视模式下按 `d` 键将删除选中的文本。
- 撤销操作
   - 撤销一步: u
   - 重做:  键入:redo

# vim 跳行
- 跳到指定行:
  - 跳15行:      15+G(或者shift + g)
  - 跳当前行的下2行: 2+回车
  - 跳文件第一行:   gg
  - 跳文件最后一行：G(或者shift + g)
  - 跳当前屏幕首行：H
  - 跳当前屏幕末行：L
  - 跳当前屏幕中间：M
- 文件上下滚动
  - 向下滚动一行，保持当前光标不动：Ctrl+E
  - 向上滚动一行，保持当前光标不动：Ctrl+Y
  - 向上滚动一行，光标跟着动:-



# vimrc设置

通常vimrc在：`sudo vim /usr/share/vim/vimrc`

```t
syntax on             "语法高亮
filetype on           "文件类型检测

set encoding=utf-8    "Vim内部使用的字符编码
set hlsearch          "搜索结果高亮
set mouse=a           "支持使用鼠标
set nocp              "切断与vi的兼容
set showmode          "在底部显示，当前处于命令模式还是插入模式
set showcmd           "命令模式下，在底部显示当前键入的指令

set number            "显示行号
set relativenumber    "设置相对行号
set cursorline        "突出显示当前行
set textwidth         "设置行宽，一行显示多少字符
set wrap              "太长的会自动换行
set wrapmargin=2      "自动换行时，与右边缘空余的字符数
set ruler             "打开状态栏标志
set laststatus=2      "2显示状态栏，1只在多窗口时显示，0不显示

set autoindent        "下一行自动缩进为跟上一行一致
set smartindent       "智能缩进
set tabstop=3		    "tab键位为3
set expandtab         "自动将Tab转为空格
set softtabstop=3     "Tab转为3个空格
set shiftwidth=5      "一级缩进>>，取消一级缩进<<，取消全部缩进==时每一级的字符数

set ignorecase        "搜索时忽略大小写
set confirm           "没有保存或文件只读时弹出确认syntax on             "语法高亮
filetype on           "文件类型检测

set encoding=utf-8    "Vim内部使用的字符编码
set hlsearch          "搜索结果高亮
set mouse=a           "支持使用鼠标
set nocp              "切断与vi的兼容
set showmode          "在底部显示，当前处于命令模式还是插入模式
set showcmd           "命令模式下，在底部显示当前键入的指令

set number            "显示行号
set relativenumber    "设置相对行号
set cursorline        "突出显示当前行
set textwidth         "设置行宽，一行显示多少字符
set wrap              "太长的会自动换行
set wrapmargin=2      "自动换行时，与右边缘空余的字符数
set ruler             "打开状态栏标志
set laststatus=2      "2显示状态栏，1只在多窗口时显示，0不显示

set autoindent        "下一行自动缩进为跟上一行一致
set smartindent       "智能缩进
set tabstop=3		    "tab键位为3
set expandtab         "自动将Tab转为空格
set softtabstop=3     "Tab转为3个空格
set shiftwidth=5      "一级缩进>>，取消一级缩进<<，取消全部缩进==时每一级的字符数

set ignorecase        "搜索时忽略大小写
set confirm           "没有保存或文件只读时弹出确认
```

# VIM Tutor

[官方学习文档](https://vimhelp.org/usr_01.txt.html#vimtutor)，下载vim后，在shell中输入`vimtutor即可打开`

## 1. 基本操作

- **Moving the cursor**
  - 基本移动：h, j, k, l 《-》左下上右
  - 通过motion: w/e来移动：数字+w/e
    - 2w：移动光标到到当前单词后面第二个(从当前单词开始数)单词的前面
    - 3e：移动光标到当前单签后面第三个(从当前单词开始数)单词的末尾

- **Exiting Vim**
  1. `<esc>`进入normal mode
  2. 输入`:q! <enter>`丢弃一切退出
  3. 输入`:wq <enter>`保存后退出
  4. `vimtutor <enter>`返回教程
  
- **Undo command**
  - 取消上一个执行的命令：normal model下小写的 `u`
  - 恢复整行的修改：normal model下大写的`U`
  - 取消undo：按住`<ctrl> +R `，按了几下u，就可以按几下R

- **进入/退出输入模式**：
  
  - Insertion： 在要输入的地方按`i`
  
  - Appending：随便光标在哪，按大写的`A`，光标都会来到这行的末尾并进入insert mode
  
  - 按`esc`退出输入模式
  
- **文本编辑：删除**

  - 单个字符删除：光标移动到要删除的字符前，normal mode下按`x`删除

  - 整行删除：`dd`

    - `4dd`：删除包括当前行在内的下面4行

  - 通过operator + motion删除:

    格式是：

    `operator [number] motion`

    - operator - is what to do, such as  d  for delete
    - [number] - is an optional count to repeat the motion
    - motion   - moves over the text to operate on, such as  w (word),  $ (to the end of line), etc.

    例子：

    - `dw`删除到下个单词前，两单词中间所有的空格也会被删除。
    - `de`删除到当前单词后，两单词中间所有的空格得到保留。
    - `d$`删除到该行的最后。
    - 同样的，`d2w/d2e`就是删除2个单词

  - 整行删除：`dd`

    - `4dd`：删除包括当前行在内的下面4行

- **文本编辑：放置**

