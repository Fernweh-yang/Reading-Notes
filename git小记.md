# 资料汇总：
- githubt官方文档：https://docs.github.com/cn
- 在线学习git网站：https://learngitbranching.js.org/?locale=zh_CN

# 一、基本配置

- 配置Git
  
   这里name和email不一致，会使得github小绿点不更新
   
   - ~$ git config --global user.name "Fernweh-yang"
     - TUM GITLAB: git config --global user.name "Yang Xu"
   - ~$ git config --global user.email "512127058@qq.com"
     - TUM Gitlab: git config --global user.email "ge23ged@mytum.de"
   - ~$ git config --list   //查看配置信息
   
- 获得帮助，以config 为例
   1. git help config
   2. git config --help
   3. man git-config
   
- 如何配置ssh到github
   - https://docs.github.com/cn/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent
   
   
   Windows:
   - 创建私钥
   ```
   ssh-keygen -t ed25519 -C "邮箱地址"
   ```
   - 复制ssh私钥
   ```
   cat ~/.ssh/id_ed25519.pub | clip
   ```
   Linux:
   - 检查现有SSH密钥
   ```
   ls -al ~/.ssh
   ```
   - 生成新的ssh
   ```
   $ ssh-keygen -t ed25519 -C "your_email@example.com"
   ```
   - 复制ssh
   ```
   $ sudo apt-get update
   $ sudo apt-get install xclip
   $ xclip -sel clip < ~/.ssh/id_ed25519.pub
   ```
   
- windows下更新git 版本
   - git update-git-for-windows
   
- 如果在公司无法用ssh，只能用Https来clone，又不想每次都输入密码可以：

   ```
   git config --local credential.helper store
   ```

   然后在下次git操作时，输入一遍用户名密码就可以了。

# 二、基本操作

## 2.1 通过git bash 上传文件

1. ```git init```:  在该文件夹下生成一个.git隐藏文件，初始化一个Git仓库。
2. ```git add``` . : 将文件夹下所有的文件添加到缓存区stage,将.换成文件名就是提交该文件
3. ```git status``` :查看现在的状态
4. `git commit -m "这里使注释"`: 把缓存区内所有的由add添加来的文件提交到分支。
5. `git remote add origin https://github.com/Fernweh-yang/TUM-Assigments`: 添加新的git方式origin， github 上创建好的仓库和本地仓库进行关联
6. `git push origin main`: 将本地库的所有内容推送到远程仓库的main分支上.
    - `git push -u origin master`: 第一次推送master分支时，加上-u参数，git会在推送内容的同时，关联本地master和远程master分支，以后可以简化命令。
7. `git diff 文件名` : 查看上次修改了什么内容
8. `git log` : 查看我们所有的commit记录

## 2.2 其他地方更改了库，本地更新

```shell
$ git remote add origin [//your github url]

//pull those changes

//origin就是一个名字，它是在你clone一个托管在Github上代码库时，git为你默认创建的指向这个远程代码库的标签，
$ git pull origin main 

// or optionally, 'git pull origin master --allow-unrelated-histories' if you have initialized repo in github and also committed locally

//now, push your work to your new repo

$ git push origin main
```

也可以：git pull = git fetch + git merge

```shell
$ git remote add upstream https://github.com/yeasy/docker_practice
$ git fetch upstream
$ git rebase upstream/main 
或者$ git merge upstream
```

[git merge和git fetch的区别](https://www.jianshu.com/p/f23f72251abc)

## 2.3 HEAD相关

- 在git中，用HEAD表示当前版本也就是最新的commit id.
  - HEAD和分支名称，如main,是两个东西。
  - 一般他们指向同一个提交记录，被HEAD指向的分支名会带有*,比如`main*`
  - 他们也可以分离，见下面
    - checkout,rebase,都会造成分离

- 查看HEAD当前的指向：

  - `cat .git/HEAD`
  - 如果指向的是一个引用可以用`git symbolic-ref HEAD `

- 分离HEAD: 让其指向了某个具体的提交记录而不是分支名。

  分离之前：HEAD->main->c1

  使用`git checkout c1`进行分离,checkout可以用于让HEAD指向哪个提交记录

  分离之后：HEAD->c1, mian->c1

  - 使用`git log`来查看上面的c1:提交记录的哈希值
    - `git log --graph`:可以看到分支合并图
  - cheakout用于移动HEAD,而branch可以用来移动分支名(比如main)

- 相对引用：使用哈希值来移动HEAD并不方便

  - `HEAD^`上一个版本
    - `main^`main的父节点
  - `HEAD^^`上上个
    - `main^^`main的父父节点
  - `HEAD~100`往上100个版本。
    - `main~100`main往上100个



## 2.4 分支管理

在 Git 2.23 版本中，引入了一个名为 `git switch` 的新命令，最终会取代 `git checkout`，因为 `checkout` 作为单个命令有点超载（它承载了很多独立的功能）。

### 2.4.1 checkout切换分支

- `git checkout -b dev`: 创建并切换到dev分支,相当于下面2条命令。

- `git branch dev`：创建一个叫dev的新分支
- `git checkout dev`

### 2.4.2 switch切换分支（推荐）

参见：https://git-scm.com/docs/git-switch

- `git switch -c dev`: 创建并切换到dev分支,相当于下面2条命令。
  - `git branch dev`：创建一个叫dev的新分支
  - `git switch dev`:切换到dev分支

- 切换到远程分支:

  1. 查看远程分支有哪些`git branch -a`:

     ```
     * master
       remotes/origin/HEAD -> origin/master
       remotes/origin/master
       remotes/origin/xy_env
     ```

  2. 切换到远程分支:`git switch -t origin/xy_env`

     会在本地创建一个xy_env分支，并和远程的remotes/origin/xy_env相关联

### 2.4.3 merger合并分支

`git merge xx`: 将分支xx合并到当前分支。

一个例子：

```shell
git switch -c dev 	# 从main创建并转到dev分支
git commit 			# dev的HEAD前进了一步
git switch main		# 切换回main分支
git commit 			# main的HEAD也前进了一步

git merge dev		# 将dev合并到main,main的HEAD并前进一步。但此时dev的HEAD还在原处
git switch dev		# 转回到dev分支
git merge main		# 这时dev的HEAD，也指向了main的HEAD所在的位置
```

### 2.4.4 rebase合并分支

- Rebase 实际上就是取出一系列的提交记录，“复制”它们，然后在另外一个地方逐个的放下去。
- Rebase 的优势就是可以创造更线性的提交历史

`git rebase xx`:把当前分支里的工作直接移到 xx 分支上。移动以后会使得两个分支的功能看起来像是按顺序开发，但实际上它们是并行开发的。

也可以`git rebase xx xxx`: 将xxx分支复制到xx分支下

一个例子：

```shell
git switch -c dev 	# 从main创建并转到dev分支
git commit 			# dev的HEAD前进了一步
git switch main		# 切换回main分支
git commit 			# main的HEAD也前进了一步
git switch dev		# 切换会dev分支

git rebase main		# 直接将dev分支复制到main分支之下，这时dev的HEAD指向main的HEAD
git switch main
git rebase dev		# 由于dev继承自main，所以 Git 只是简单的把 main 分支的引用向前移动了一下而已。
```

### 2.4.5 branch分支操作：

- `git branch`:查看当前分支
- `git branch -d dev` : 删除当前分支
- `git branch -f main HEAD~3`:强行将main分支指向HEAD的第三级父提交

### 2.4.6 上传分支

```shell
git switch dev	# 切换到要上传的分支处
git add .
git commit -m "xxxxx"

# push有2个选择
git push --set-upstream origin dev	# 第一次关联远程分支，以后可以直接push
got push origin dev					# 不想关联远程分支的话，就每次都要同时输入origin和dev
```



### 2.4.7其他相关命令

2. `git stash`: 将当前工作现场储存起来，等以后恢复现场继续工作
   1. `git stash list` 查看储存起来的工作现场
   2. `git stash pop`: 恢复工作现场，并删除stash内容
   3. `git stash apply`: 恢复工作现场，不删除stash内容，得输入`git stash drop`来删除。
3. `git cherry-pick <commit id>`: 在master分支上修复了bug，而dev其他工作开发了一半，显然不可能从master重新分出来，在dev上再修复bug很麻烦。该命令可以把修复bug的那一次修改复制到dev分支来。

### 2.4.8删除分支

- 本地删除:`git branch -d  local_branch_name`
- 远程删除:`git push remote_name -d remote_branch_name`
  - remote_name: 远程名称，通常都是Origin
  - -d: --delete的别名
  - remote_branch_name: 要删除的远程分支名

### 2.4.9查看分支
- 查看本地分支:`git branch`
- 查看远程分支：`git branch -r`
- 查看所有分支：`git branch -a `


## 2.5 撤销变更

### 2.5.1 reset 本地撤销

`git reset` 通过把分支记录回退几个提交记录来实现撤销改动。你可以将这想象成“改写历史”。`git reset` 向上移动分支，原来指向的提交记录就跟从来没有提交过一样。

**可以用它来回退版本：**

1. `git reset --hard HEAD^` : 退回到上一个版本
   - 注意：分离后的HEAD就不能使用reset重置了，而要用checkout
2. `git reset --hard 版本号` : 回到指定的版本,也可以在后悔回退到过去时由此退回到未来
3. `git reflog`: 查看git记录下的每一条命令。如果忘了之前的版本号，由此查看。

- 撤销删改
  1. `git checkout -- file`: 撤销文件在工作区的全部修改，回到最近一个git commit 或git add时的状态。

  2. `git reset HEAD 文件名`: 如果add到缓存区了，可以用这个名利把缓存区的修改撤销掉unstage,重新回到工作区 

  3. `git rm 文件名` 从版本库中删除文件，和add一样之后要跟commit。

  4. `git remote -v`:查看远程库的信息

  5. `git remote rm 库名`:删除本地与远程库的绑定关系

### 2.5.2 revert 远程撤销

本地可以使用reset,但是不能应用于大家一起使用的远程分支。

为了撤销更改并**分享**给别人，我们需要使用 `git revert`。

- **举例：**

  初始状态：c2->c1->c0

  `git revert HEAD`后得到：c2'->c2->c1->c0

  新提交记录 `C2'` 引入了**更改** —— 这些更改刚好是用来撤销 `C2` 这个提交的。也就是说 `C2'` 的状态与 `C1` 是相同的。

- 撤销后，再push，大家pull了后就相当于让大家都撤销了。



## 2.6 整理提交记录

解决问题：“我想要把这个提交放到这里, 那个提交放到刚才那个提交的后面”

### 2.6.1 cherry-pick

当我们知道所需要的提交记录 和 这些记录的哈希值时使用

`git cherry-pick <提交号>...`: 将一些提交复制到当前所在的位置（`HEAD`）下面

**举例**：

1. 有一个git仓库结构：

   ```mermaid
   flowchart BT
       c4-->c3
       c3-->c2
       c2-->c1
       c1-->c0
       c5-->c1
   ```

   - 此时分支side指向c4
   - 分支main*指向c5，（注意HEAD此时和main指向同一个提交记录，所以main后有个\*）

2. 运行命令`git cherry-pick c2 c4`后,得到

   ```mermaid
   flowchart BT
       c4-->c3
       c3-->c2
       c2-->c1
       c1-->c0
       c5-->c1
       c2'-->c5
       c4'-->c2'
   ```

   - 此时分支side指向c4
   - 分支main*指向c4'
   - 命令cherry brick直接把提交记录复制到当前HEAD之下了

### 2.6.2 交互式rebase

- 2.4.4中rebase可以合并分支。交互式 rebase 指的是使用带参数 `--interactive` 的 rebase 命令, 简写为 `-i`，会打开一个UI界面。
- 打开UI界面后，可以干三件事：
  - 调整提交记录的顺序（通过鼠标拖放来完成）
  - 删除你不想要的提交（通过切换 `pick` 的状态来完成，关闭就意味着你不想要这个提交记录）
  - 合并提交。 遗憾的是由于某种逻辑的原因，我们的课程不支持此功能，因此我不会详细介绍这个操作。简而言之，它允许你把多个提交记录合并成一个。

**举例：**

`git rebase -i HEAD~4`: 重排最新的4次提交记录



### 2.6.3 只push一个本地记录

- 使用场景：

  我正在解决某个特别棘手的 Bug，为了便于调试而在代码中添加了一些调试命令并向控制台打印了一些信息。

  这些调试和打印语句都在它们各自的提交记录里。最后我终于找到了造成这个 Bug 的根本原因，解决掉以后觉得沾沾自喜！

  最后就差把 `bugFix` 分支里的工作合并回 `main` 分支了。你可以选择通过 fast-forward 快速合并到 `main` 分支上，但这样的话 `main` 分支就会包含我这些调试语句了。你肯定不想这样，应该还有更好的方式……

- 解决方案：

  使用2.6.1/2的命令，只提交最终解决问题的那**一个**记录

**举例：**

有一结构：c4(bugfix*)->c3->c2->c1(main)->c0

要变成:c4(main*/bugfix)->c1->c0

1. 解法1：

   ```
   git rebase -i main c4 	#或者 git rebase -i HEAD~3
   # 将中间的c3,c2都用鼠标omit掉（隐藏掉）
   git rebase bugFix main	# 合并分支main到bugFix下，但bugFix(c4)本身就指向main(c1),所以main就移了下来。
   ```

2. 解法2

   ```
   git checkout main
   git cherry-pick c4
   ```

### 2.6.4 修改之前的提交记录

- 使用场景：

  在 `newImage` 分支上进行了一次提交，然后又基于它创建了 `caption` 分支，然后又提交了一次。

  此时你想对某个以前的提交记录进行一些小小的调整。比如设计师想修改一下 `newImage` 中图片的分辨率，尽管那个提交记录并不是最新的了。

- 解决方案

  - 先用 `git rebase -i` 将提交重新排序，然后把我们想要修改的提交记录挪到最前
  - 然后用 `git commit --amend` 来进行一些小修改
  - 接着再用 `git rebase -i` 来将他们调回原来的顺序
  - 最后我们把 main 移到修改的最前端（用你自己喜欢的方法），就大功告成啦！

举例：

有一结构:c3(caption*)->c2(newimage)->c1(main)->c0

现在想稍微修改c2一下：

1. 方法1：

   ```shell
   git rebase -i c1 c3  # 互换c3,c2的顺序,此时HEAD指向c2'
   git commit --amend 	 # 修改c2',得到c2''
   git rebase -i HEAD^^ # 再次互换c2'',c3'的顺序,此时HEAD指向c3''
   git branch -f main HEAD	# 将main和HEAD合并
   ```

   

2. 方法2：

   相比于方法1，不需要2次互换顺序，rebase稍不留意就会冲突。

   ```shell
   git checkout main	# 将HEAD指向C1(main)
   git cherry-pick c2	# 将c2复制一个c2' 到main下, 此时HEAD指向c2'
   git commit --amend  # 修改c2'
   git cherry-pick c3	# 将c3复制到head(c2'')下
   ```

   



## 2.6 多人协作

1. 查看远程库信息，使用`git remote -v`；
2. 首先，试图用`git push origin <branch-name>`推送自己的修改
3. 如果推送失败，是因为远程分支比本地的更新，需要先用`git pull`试图合并
4. 如果合并有冲突，则解决冲突，没有冲突或者解决掉冲突后，再用`git push origin <branch-name>`推送就能成功
- 如果`git pull`提示`no tracking information`，则说明本地分支和远程分支的链接关系没有创建
    - 在本地创建和远程分支对应的分支，使用`git checkout -b branch-name origin/branch-name`，本地和远程分支的名称最好一致；
    - 建立本地分支和远程分支的关联，使用`git branch --set-upstream branch-name origin/branch-name`；
- `git rebase`: 把本地未push的分叉提交历史整理成直线,然后用`git log`查看

## 2.7 标签管理

- 应用场景：

  分支很容易被人为移动，并且当有新的提交时，它也会移动。分支很容易被改变，大部分分支还只是临时的，并且还一直在变。有没有什么可以*永远*指向某个提交记录的标识呢，比如软件发布新的大版本，或者是修正一些重要的 Bug 或是增加了某些新特性，有没有比分支更好的可以永远指向这些提交的方法呢？

- `git tag v1.0`:给当前分支(HEAD)打上标签v1.0
- `git tag v0.9 f52c643`: 指定给某一个提交记录打上标签。标签总是和某个commit挂钩，若该commit出现在不同的2个分支上，这2个分支上都可以看到标签。
- `git tag`:查看标签
- `git tag -a <tagname> -m "blablabla..."`可以指定标签信息；
   - -a: 标签名字
   - -m:说明文字 
- `git show <tagname>`查看标签信息
- `git tag -d v0.1`:删除本地标签
- `git push origin :refs/tags/<tagname>`可以删除一个远程标签
- `git push origin <tagname>`可以推送一个本地标签；
- `git push origin --tags`可以推送全部未推送过的本地标签；
- 描述标签`git describe ref`
   - Git Describe 能帮你在提交历史中移动了多次以后找到方向；当你用 `git bisect`（一个查找产生 Bug 的提交记录的指令）找到某个提交记录时，或者是当你坐在你那刚刚度假回来的同事的电脑前时， 可能会用到这个命令。
   - 输出结果是：`<tag>_<numCommits>_g<hash>`
     - `tag` 表示的是离 `ref` 最近的标签， 
     - `numCommits` 是表示这个 `ref` 与 `tag` 相差有多少个提交记录，
     - `hash` 表示的是你所给定的 `ref` 所表示的提交记录哈希值的前几位。

## 2.8 子模块

- 帮助文档:`git help submodule`

- CLone包含子模块的项目：

  在clone后需要使用`git submodule update --init --recursive`来初始化子模块内容。

- 添加/删除子模块：

  ```shell
  添加：
  $ git submodule add <url> <repo_name>
  例子：
  $ git submodule add https://github.com/iphysresearch/GWToolkit.git GWToolkit
  
  删除：
  git rm --cached GWToolkit
  rm -rf GWToolkit
  并手动删除下面下个文件中相关的内容
  ```

  添加后会多出如下文件

  1. `.gitmodules文件`：用于保存子模块的信息。
  2. `.git/config`: 中会多出一块关于子模块的信息内容
  3. `.git/modules/GwToolkit`: 子模块文件

- 更新子模块：

  - 更新项目内子模块到最新版本：`git submodule update`
  - 更新子模块为远程项目的最新版本：`git submodule update --remote`

## 2.9 添加.gitignore
1. 在.git所在的文件夹下创建.gitignore,并添加不需要的目录或文件：

  比如一个目录：

  ```
  - repository
  	- .git
  	- .gitignore
  	- .vscode
  	- build/xxx
  	- source/xxx
  	- test/xxx
  ```

  .gitignore中取出build和.vscode

  ```
  /build
  /.vscode
  ```

2. 如果添加.gitignore前，已经上传了很多不需要的文件，用以下命令去除他们：

  - 文件夹：`git rm -r --cached 文件夹名`
  - 文件: `git rm --cached 文件名`

## 2.10 把多个分支clone到自己仓库上

1. 克隆原始仓库

   `git clone --mirror git://g.csail.mit.edu/xv6-labs-2022 project_xv6`

2. 在github上创建一个远程仓库，名字为`project_xv6`

3. 删除旧的origin（远程仓库）

   `git remote remove origin`

   - 通过`git remote -v`可以查看当前关联的远程仓库是哪一个

4. 更新新的origin

   `git remote add origin git@github.com:Fernweh-yang/project_xv6.git`

5. 上传分支

   - 上传所有分支：`git push --all origin`
   - 上传单个分支：`git push origin`

# 三、commit技巧

## 3.1 使用Emoji

- git可用的Emoji参考[网站](https://gitmoji.dev/)

- 使用方法：

  在 Emoji 的名字前后个加上一个冒号，比如

  ```
  git commit -m ":bug: fix a bug writtten by pig teammate"
  ```

  

## 3.2 格式

type+subject

- **type**: 用于说明 commit 的类别，只允许使用下面7个标识。

  - feat：新功能（feature）
  - fix：修补bug
  - docs：文档（documentation）
  - style： 格式（不影响代码运行的变动）
  - refactor：重构（即不是新增功能，也不是修改bug的代码变动）
  - test：增加测试
  - chore：构建过程或辅助工具的变动

  


## 3.3 使用commitizen来格式化提交

[git仓库](https://github.com/commitizen-tools/commitizen?tab=readme-ov-file)

- 下载：

  ```shell
  pip install --user -U Commitizen
  ```

  如果报warning说`/home/hx/.local/bin`不在地址内，就需要在`.zshrc`中加入：

  ```
  export PATH="$PATH:/home/hx/.local/bin"
  ```

- 使用：

  1. 在每个工程(工程的git目录下)开始时初始化：

     ```
     cz init
     ```

  2. 
