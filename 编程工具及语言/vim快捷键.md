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

- **光标移动：基本移动`hjkl`**
  
  - 基本移动：h, j, k, l 《-》左下上右
  - 通过motion: w/e来移动：数字+w/e
    - 2w：移动光标到到当前单词后面第二个(从当前单词开始数)单词的前面
    - 3e：移动光标到当前单签后面第三个(从当前单词开始数)单词的末尾
  
- **光标移动：`G`和`gg`**
  
  - 查看当前光标所在位置：`ctrl`+`g`
  
    底部会出现信息：

    ```
    "/tmp/tutorxBz8Rm" [Modified] line 502 of 970 --51%-- col 3
    ```
  
  - 跳到文档底部：`G`
  
  - 跳到文档顶部：`gg`
  
  - 跳回到特定502行:`502G`
  
  - 跳到(),[],{}对应另一半括号的位置:
  
    将光标移动到括号前，然后按`%`，光标就会移动该括号对应的另一半前。
  
- **文本搜索：`/`和`?`**
  
  - 向后搜索`/`：
  
    `/example`+`Enter`会将光标移动到当前光标后第一个出现的`example`。

  - 向前搜索`?`：
  
    `?example`+`Enter`会将光标移动到当前光标前第一个出现的`example`。
  
  - 重复搜索:
  
    按 `n` 键跳转到下一个匹配的文本。
  
    按 `N` 键跳转到上一个匹配的文本。
  
  - 高亮
  
    - 启用高亮 `:set hlsearch`
    - 关闭高亮 `:noh`
  
  - 搜索并替换
  
    - 替换当前行内的一个文本`:s`
  
      将当前行中第一个 "foo" 替换为 "bar"，可以输入 `:s/foo/bar/`，然后按 `Enter`。
  
    - 替换当前行内的所有该文本`:s`+g
  
      将当前行中所有 "foo" 替换为 "bar"，可以输入 `:s/foo/bar/g`，然后按 `Enter`。
  
    - 替换指定行内的本文`:#,#s`
  
      替换568行中所有"foo"为"bar"，可以输入`:568,568s/foo/bar/g`，然后按Enter。
  
    - 替换整个文件的文本`:%s`+`g`
  
      将文件中的 "foo" 都替换为 "bar"，可以输入 `:%s/foo/bar/g`，然后按 `Enter`。
  
    - 开启确认提示`c`
  
      如输入`:%s/foo/bar/gc`，就会在每个搜索要替换的地方要求确认是否替换：
  
      - **`y`** (yes) - 替换当前匹配项，并继续查找下一个匹配项。
      - **`n`** (no) - 不替换当前匹配项，并继续查找下一个匹配项。
      - **`a`** (all) - 替换当前匹配项，并且不再提示，直接替换所有剩余的匹配项。
      - **`q`** (quit) - 退出替换模式，停止查找和替换。
      - **`l`** (last) - 替换当前匹配项，并停止查找和替换（即，仅替换当前匹配项）。
      - **`^E`** (Ctrl + E) - 向下滚动窗口（不影响替换过程），使更多文本内容显示出来。
      - **`^Y`** (Ctrl + Y) - 向上滚动窗口（不影响替换过程），使更多文本内容显示出来。
  
- **Exiting Vim**

  1. `<esc>`进入normal mode
  2. 输入`:q! <enter>`丢弃一切退出
  3. 输入`:wq <enter>`保存后退出
  4. `vimtutor <enter>`返回教程

- **Undo command**
  - 取消上一个执行的命令：normal model下小写的 `u`
  - 恢复整行的修改：normal model下大写的`U`
  - 取消undo：按住`<ctrl> +R `，按了几下u，就可以按几下R

- **进入/退出输入模式`i`和`A`**：

  - Insertion： 在要输入的地方按`i`

  - Appending：随便光标在哪，按大写的`A`，光标都会来到这行的末尾并进入insert mode

  - 按`esc`退出输入模式

- **文本编辑：删除`d`**

  - 单个字符删除：光标移动到要删除的字符前，normal mode下按`x`删除

  - 整行删除：`dd`

    - `4dd`：删除包括当前行在内的下面4行

  - 通过operator + motion删除:

    格式是：

    ```
    operator [number] motion
    d [number] motion
    ```

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

- **文本编辑：放置`p`**

  - 剪切：
    - 上面任何被删除的东西都会被保存到寄存器中register中，然后就可以按`p`放置到当前光标后字符的后面
    - 注意register只保存最后一次删除的东西，按多次p放置的都是最后一次删除的

- **文本编辑：替换`r`**

  - 只能替换一个字符：

    移动光标到要替换的字符前，按`r`，然后输入要替换的字符

- **文本编辑：改变`c`**

  - 如果想改变一个单词，如luex -> line

    - 把光标移动到u前
    - 按ce，删除字符uex
    - 然后会自动进入输入模式

    所以ce相当于de然后i

  - 和删除一样可以结合motion来改变

    格式是：

    ```
    operator [number] motion
    c [number] motion
    ```

    - operator - is what to do, such as  d  for delete
    - [number] - is an optional count to repeat the motion
    - motion   - moves over the text to operate on, such as  
      - w (start of the next word),  
      - $ (to the end of line)
      - e(end of current word)

  



# 一些应用场景

## 复制选中的内容并粘贴到其他地方

1. gg到文件首 、hjkl移动光标到想要复制的地方头

   $：行尾

   ^：行首

2. 进入可是模式

   - v：字符可视模式
   - V：行可视模式

3. G到文件尾、hjkl移动光标到想要复制的地方尾（移动到哪就选中到哪了）

4. y复制可视模式下选中文字

5. 移动光标到想要复制的地方，p粘贴

## 多行快速缩进和取消缩进

1. 进入Visual模式，选中要缩进的内容
2. 按>缩进，按<取消缩进

