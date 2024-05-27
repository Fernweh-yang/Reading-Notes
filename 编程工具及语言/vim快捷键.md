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

