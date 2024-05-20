# 资料汇总

https://juejin.cn/post/7088571975546699789

[oh my zsh官网](https://ohmyz.sh/#install)

# Zsh

*Zsh*，也被称为_Z shell_，是另一个基于UNIX系统的shell，经常被用作Bash的替代品，Bash是很多基于UNIX系统的默认shell。

*Zsh实际上可以_毫无问题地_运行很多Bash脚本*。除此之外，Zsh还带有很多_不错的功能_，可以使它成为比Bash更好的选择，比如。

- **更好的自动完成**功能 - Zsh的自动完成功能可以说比Bash好得多。在自动完成的建议中进行导航是非常友好和直观的。
- **内置的自动**更正 - 如果你倾向于快速打字，并且在打字过程中出现了很多错别字，Zsh会帮你修正，而不需要运行外部脚本或安装额外的插件。
- **自动`cd`**- 不需要每次想改变一个目录时都输入`cd` ，只需输入所需目录的路径。
- **递归路径扩展**--也不需要输入整个路径，例如，`cd us/b/l` 将扩展为`cd user/bin/lin` 。
- **更容易定制**- Zsh最独特的功能之一可能是它的可定制性。有大量的主题和插件，可以满足你的大部分需求。它们通常由_Oh My Zsh_框架管理，但也有其他框架。

## 安装

1. 安装zsh

   ```
   sudo apt update
   sudo apt install zsh
   ```

   设置默认的shell为zsh

   ```
   chsh -s $(which zsh)
   ```

2. 安装oh my zsh来管理zsh配置、插件和主题

   随便选一个下载安装：

   ```shell
   # Run the following command to install Oh My Zsh using `curl`
   $ sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
   
   # Run the following command to install Oh My Zsh using `wget`
   $ sh -c "$(wget https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)"
   ```

   

## 配置zsh主题

通过`code ~/.zshrc`打开:

- 最简单的配置方式：

  是选择[官网](https://github.com/ohmyzsh/ohmyzsh/wiki/Themes)已经有的配置，直接在ZSH_THEME处修改。

  重启后终端后生效

```shell
# If you come from bash you might have to change your $PATH.
# export PATH=$HOME/bin:/usr/local/bin:$PATH
# ! Section 1: PATH variables & path to Oh My Zsh installation
# Path to your oh-my-zsh installation.
export ZSH="$HOME/.oh-my-zsh"

# ! Section 2: Theme settings
# Set name of the theme to load --- if set to "random", it will
# load a random theme each time oh-my-zsh is loaded, in which case,
# to know which specific one was loaded, run: echo $RANDOM_THEME
# See https://github.com/ohmyzsh/ohmyzsh/wiki/Themes
ZSH_THEME="robbyrussell"

# Set list of themes to pick from when loading at random
# Setting this variable when ZSH_THEME=random will cause zsh to load
# a theme from this variable instead of looking in $ZSH/themes/
# If set to an empty array, this variable will have no effect.
# ZSH_THEME_RANDOM_CANDIDATES=( "robbyrussell" "agnoster" )

# ! Section 3: General zsh settings
# Uncomment the following line to use case-sensitive completion.
# CASE_SENSITIVE="true"

# Uncomment the following line to use hyphen-insensitive completion.
# Case-sensitive completion must be off. _ and - will be interchangeable.
# HYPHEN_INSENSITIVE="true"

# Uncomment one of the following lines to change the auto-update behavior
# zstyle ':omz:update' mode disabled  # disable automatic updates
# zstyle ':omz:update' mode auto      # update automatically without asking
# zstyle ':omz:update' mode reminder  # just remind me to update when it's time

# Uncomment the following line to change how often to auto-update (in days).
# zstyle ':omz:update' frequency 13

# Uncomment the following line if pasting URLs and other text is messed up.
# DISABLE_MAGIC_FUNCTIONS="true"

# Uncomment the following line to disable colors in ls.
# DISABLE_LS_COLORS="true"

# Uncomment the following line to disable auto-setting terminal title.
# DISABLE_AUTO_TITLE="true"

# Uncomment the following line to enable command auto-correction.
# ENABLE_CORRECTION="true"

# Uncomment the following line to display red dots whilst waiting for completion.
# You can also set it to another string to have that shown instead of the default red dots.
# e.g. COMPLETION_WAITING_DOTS="%F{yellow}waiting...%f"
# Caution: this setting can cause issues with multiline prompts in zsh < 5.7.1 (see #5765)
# COMPLETION_WAITING_DOTS="true"

# Uncomment the following line if you want to disable marking untracked files
# under VCS as dirty. This makes repository status check for large repositories
# much, much faster.
# DISABLE_UNTRACKED_FILES_DIRTY="true"

# Uncomment the following line if you want to change the command execution time
# stamp shown in the history command output.
# You can set one of the optional three formats:
# "mm/dd/yyyy"|"dd.mm.yyyy"|"yyyy-mm-dd"
# or set a custom format using the strftime function format specifications,
# see 'man strftime' for details.
# HIST_STAMPS="mm/dd/yyyy"

# Would you like to use another custom folder than $ZSH/custom?
# ZSH_CUSTOM=/path/to/new-custom-folder

# ! Section 4: Plugin settings
# Which plugins would you like to load?
# Standard plugins can be found in $ZSH/plugins/
# Custom plugins may be added to $ZSH_CUSTOM/plugins/
# Example format: plugins=(rails git textmate ruby lighthouse)
# Add wisely, as too many plugins slow down shell startup.
plugins=(git)

source $ZSH/oh-my-zsh.sh

# ! Section 5: Other user settings
# User configuration

# export MANPATH="/usr/local/man:$MANPATH"

# You may need to manually set your language environment
# export LANG=en_US.UTF-8

# Preferred editor for local and remote sessions
# if [[ -n $SSH_CONNECTION ]]; then
#   export EDITOR='vim'
# else
#   export EDITOR='mvim'
# fi

# Compilation flags
# export ARCHFLAGS="-arch x86_64"

# ! Section 6: User-defined aliases
# Set personal aliases, overriding those provided by oh-my-zsh libs,
# plugins, and themes. Aliases can be placed here, though oh-my-zsh
# users are encouraged to define aliases within the ZSH_CUSTOM folder.
# For a full list of active aliases, run `alias`.
#
# Example aliases
# alias zshconfig="mate ~/.zshrc"
# alias ohmyzsh="mate ~/.oh-my-zsh"

```

## 安装插件

### Oh My Zsh自带的插件

查看[官网](https://github.com/ohmyzsh/ohmyzsh/wiki/Plugins)可发现Oh My Zsh已经下载了好多插件。对于这些插件，只需要在配置文件中添加就行：

```
plugins=(rails git ruby)
```

## 创建别名

查看[官网](https://github.com/ohmyzsh/ohmyzsh/wiki/Cheatsheet#directory)可发现Oh My Zsh已经设置好了一些命令的别名，可以有效减少某些常用长命令的敲击次数。

自定义别名在配置文件中按如下例子定义即可：

```shell
# alias my-alias="command"
# Examples:alias gaa="git add all"alias lsla="ls -la"...

alias ct="{print -z Current time is $(date)}"
```

## 各种有用的功能：

### 1. `d` :在目录中穿越

`d` 命令会列出我们最近进入的目录历史，并且会给这些目录加上序号，只需要输入对应目录的序号，即可重新进入该目录

# Bash

类似oh-my-zsh的配置管理工具有：

[**bash-it**](https://github.com/Bash-it/bash-it)

**[oh-my-bash](https://github.com/ohmybash/oh-my-bash)**

