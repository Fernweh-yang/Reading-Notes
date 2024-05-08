# 学习资料

[官方文档](https://docs.docker.com/)

# Docker基础

## Docker Architecture结构

  ![](https://docs.docker.com/get-started/images/docker-architecture.webp)

- Docker使用Client-Server结构。

- Docker Client**`docker`**通过REST API，over UNIX sockets或者 a network interface方式来和一个或多个Docker Daemon取得联系。这些Demon可以是和Client一个系统上，也可以在一个另外的远程系统上。

  比如运行docker build，`docker`就把这个指令发送给`dockerd`了。

- Docker Daemon**`dockerd`**守护进程，负责容器Container的building构建、running运行 以及 distributing分发。它会收取来自Docker API的指令，并管理image镜像、containers容器、networks网络 以及 volumes数据卷.

- Docker registries存储着各种镜像，供人们下载使用。
## 三个基本概念：

### 镜像:Image

- 操作系统分为 **内核** 和 **用户空间**。对于 `Linux` 而言，内核启动后，会挂载 `root` 文件系统为其提供用户空间支持。而 **Docker 镜像**（`Image`），就相当于是一个 `root` 文件系统。比如官方镜像 `ubuntu:18.04` 就包含了完整的一套 Ubuntu 18.04 最小系统的 `root` 文件系统。

  **Docker 镜像** 是一个特殊的文件系统，除了提供容器运行时所需的程序、库、资源、配置等文件外，还包含了一些为运行时准备的一些配置参数（如匿名卷、环境变量、用户等）。镜像 **不包含** 任何动态数据，其内容在构建之后也不会被改变。

- 分层存储：

  - 镜像并非是像一个 `ISO` 那样的打包文件，镜像只是一个虚拟的概念，其实际体现并非由一个文件组成，而是由一组文件系统组成，或者说，由多层文件系统联合组成。

  - 镜像构建时，会一层层构建，前一层是后一层的基础。每一层构建完就不会再发生改变，后一层上的任何改变只发生在自己这一层。比如，删除前一层文件的操作，实际不是真的删除前一层的文件，而是仅在当前层标记为该文件已删除。在最终容器运行的时候，虽然不会看到这个文件，但是实际上该文件会一直跟随镜像。因此，在构建镜像的时候，需要额外小心，每一层尽量只包含该层需要添加的东西，任何额外的东西应该在该层构建结束前清理掉。

### 容器:Container

- 镜像（`Image`）和容器（`Container`）的关系，就像是面向对象程序设计中的 `类` 和 `实例` 一样，镜像是静态的定义，容器是镜像运行时的实体。容器可以被创建、启动、停止、删除、暂停等。

- 容器的实质是进程，但与直接在宿主执行的进程不同，容器进程运行于属于自己的独立的 [命名空间](https://en.wikipedia.org/wiki/Linux_namespaces)。因此容器可以拥有自己的 `root` 文件系统、自己的网络配置、自己的进程空间，甚至自己的用户 ID 空间。容器内的进程是运行在一个隔离的环境里，使用起来，就好像是在一个独立于宿主的系统下操作一样。

- 前面讲过镜像使用的是分层存储，容器也是如此。每一个容器运行时，是以镜像为基础层，在其上创建一个当前容器的存储层，我们可以称这个为容器运行时读写而准备的存储层为 **容器存储层**。容器存储层的生存周期和容器一样，容器消亡时，容器存储层也随之消亡。因此，任何保存于容器存储层的信息都会随容器删除而丢失。

- 所有的文件写入操作，都应该使用 **数据卷（Volume**）或者 **绑定宿主目录**，在这些位置的读写会跳过容器存储层，直接对宿主（或网络存储）发生读写，其性能和稳定性更高。数据卷的生存周期独立于容器，容器消亡，数据卷不会消亡。因此，使用数据卷后，容器删除或者重新运行之后，数据却不会丢失。

### 仓库:Repository

- 镜像构建完成后，可以很容易的在当前宿主机上运行，但是，如果需要在其它服务器上使用这个镜像，我们就需要一个集中的存储、分发镜像的服务。**Docker Registry**就是这样的服务。

  一个 **Docker Registry** 中可以包含多个 **仓库**（`Repository`）；每个仓库可以包含多个 **标签**（`Tag`）；每个标签对应一个镜像。

- 通常，一个仓库会包含同一个软件不同版本的镜像，而标签就常用于对应该软件的各个版本。我们可以通过 `<仓库名>:<标签>` 的格式来指定具体是这个软件哪个版本的镜像。如果不给出标签，将以 `latest` 作为默认标签。

  比如：ubuntu:20.03.

  仓库名经常以 *两段式路径* 形式出现，比如 `jwilder/nginx-proxy`，前者往往意味着 Docker Registry 多用户环境下的用户名，后者则往往是对应的软件名。

- Docker Registry 公开服务:

  - Docker Registry 公开服务是开放给用户使用、允许用户管理镜像的 Registry 服务。一般这类公开服务允许用户免费上传、下载公开的镜像，并可能提供收费服务供用户管理私有镜像。
  - 最常使用的 Registry 公开服务是官方的 [Docker Hub](https://hub.docker.com)，这也是默认的 Registry，并拥有大量的高质量的 [官方镜像](https://hub.docker.com/search?q=&type=image&image_filter=official)。除此以外，还有 Red Hat 的 [Quay.io](https://quay.io/repository/)；Google 的 [Google Container Registry](https://cloud.google.com/container-registry/)，[Kubernetes](https://kubernetes.io) 的镜像使用的就是这个服务；代码托管平台 [GitHub](https://github.com) 推出的 [ghcr.io](https://docs.github.com/cn/packages/working-with-a-github-packages-registry/working-with-the-container-registry)。
  - 。国内的一些云服务商提供了针对 Docker Hub 的镜像服务（`Registry Mirror`），这些镜像服务被称为 **加速器**。常见的有 [阿里云加速器](https://www.aliyun.com/product/acr?source=5176.11533457&userCode=8lx5zmtu)、[DaoCloud 加速器](https://www.daocloud.io/mirror#accelerator-doc) 等。


## 使用镜像

### 1. 获取镜像docker pull

```
$ sudo docker pull [选项] [Docker Registry 地址[:端口号]/]仓库名[:标签]
```

- 具体的选项可以通过 `docker pull --help` 命令看到
- Docker 镜像仓库地址：地址的格式一般是 `<域名/IP>[:端口号]`。不填即默认地址Docker Hub(`docker.io`)。
- 仓库名：两段式名称，即 `<用户名>/<软件名>`。对于 Docker Hub，如果不给出用户名，则默认为 `library`，也就是官方镜像。

- 比如：`$ docker pull ubuntu:20.04`

  可以看见下载也是封层加载的，并给出每一层id的前12位。下载结束后会给出镜像完整的`sha256`摘要。

### 2. 列出镜像docker image ls

- `docker image ls`显示了仓库名、标签、镜像id、创建时间 以及 所占用的空间。
  - **镜像ID**是镜像的唯一标识，一个镜像可以对应多个标签。上面例子中拥有相同的IMAGE ID，说明他们是同一个镜像。

```
$ sudo docker image ls
REPOSITORY           TAG                 IMAGE ID            CREATED             SIZE
ubuntu               18.04               329ed837d508        3 days ago          63.3MB
ubuntu               bionic              329ed837d508        3 days ago          63.3MB
```



- 可以通过`docker system df`命令来查看镜像、溶剂、数据卷所占用的空间

```
$ sudo docker system df

TYPE                TOTAL         ACTIVE        SIZE              RECLAIMABLE
Images              24            0             1.992GB           1.992GB (100%)
Containers          1             0             62.82MB           62.82MB (100%)
Local Volumes       9             0             652.2MB           652.2MB (100%)
Build Cache                                     0B                0B
```

- `docker image ls -f dangling=true`显示**虚悬镜像**

  **虚悬镜像dangling image**由于新旧镜像同名，旧镜像名称被取消，从而出现仓库名、标签均为<none>的镜像。这类无标签的镜像称为虚悬镜像。

  ```
  $ sudo docker image ls -f dangling=true
  REPOSITORY          TAG                 IMAGE ID            CREATED             SIZE
  <none>              <none>              00285df0df87        5 days ago          342 MB
  ```

  虚悬镜像可以随意删除

  ```
  $ sudo docker image prune
  ```

- `docker image ls -a`**中间层镜像**

  为了加速镜像构建、重复利用资源，Docker 会利用 **中间层镜像**。所以在使用一段时间后，可能会看到一些依赖的中间层镜像。

  加-a参数后可以看到很多无标签的镜像，这些无标签的镜像很多都是中间层镜像，是其它镜像所依赖的镜像。这些无标签镜像不应该删除，否则会导致上层镜像因为依赖丢失而出错。

  ```
  $ sudo docker image ls -a

- 如果只想列出部分镜像

  - 根据仓库名：

    ```
    $ sudo docker image ls ubuntu
    ```

  - 特定某个镜像，指定仓库名和标签

    ```
    $ sudo docker image ls ubuntu:18.04
    ```

  - 过滤器`-f`或者`--filter`。

    比如想看到mongo:3.2之后/之前建立的命令

    ```
    $ sudo docker image ls -f since=mongo:3.2
    $ sudo docker image ls -f before=mongo:3.2
    ```

### 3. 删除本地镜像docker image rm

```
$ docker image rm [选项] <镜像1> [<镜像2> ...]
```

- 用 ID、镜像名、摘要删除镜像

  - ID：`docker image ls` 默认列出的就已经是短 ID 了，一般取前3个字符以上，只要足够区分于别的镜像就可以了

    ```
    $ sudo docker image rm 501
    ```
    
  - 镜像名：<仓库名>:<标签>
    
    ```
    $ docker image rm centos
    ```
    
  - 镜像摘要digest:`sha256:....`
    
    ```
    # 加参数--digests查看镜像摘要
    $ docker image ls --digests
    
    $ docker image rm node@sha256:b4f0e0bdeb578043c1ea6862f0d40cc4afe32a4a582f3be235a3b164422be228
    ```
  
- 两种删除:Untagged和Deleted
  
  `Untagged`:将满足我们要求的所有镜像标签都取消
  
  `Deleted`:删除镜像.
  
  - 但一个镜像唯一标识是ID和摘要，标签可以拥有好几个。所以untag了某个标签后，如果这个镜像还有其他标签，delete不会执行。
  
  - 由于镜像是多层存储结构，删除时是从上层往下层删。但比如某一个 i 层删掉了，下面的 i-1层可能是另外某个 i 层的依赖，这时候delete不会执行
  - 荣国某个镜像启动的容器还在，delete也不会执行
  
- 用docker image ls来配合批量删除

  删除所有仓库名位redis的镜像

  ```
  $ sudo docker image rm $(docker image ls -q redis)
  ```

  删除所有在 `mongo:3.2` 之前的镜像

  ```
  $ sudo docker image rm $(docker image ls -q -f before=mongo:3.2)
  ```


### 4. 利用commit 理解镜像构造

- 一个定制Web服务器的例子：

  ```
  $ docker run --name webserver -d -p 80:80 nginx
  ```

  - 这条命令会用 `nginx` 镜像启动一个容器，命名为 `webserver`，并且映射了 80 端口，这样我们可以用浏览器去访问这个 `nginx` 服务器。

  - 如果是在本机运行的 Docker，那么可以直接访问：`http://localhost` ，如果是在虚拟机、云服务器上安装的 Docker，则需要将 `localhost` 换为虚拟机地址或者实际云服务器地址。

  - 如果我们不喜欢这个欢迎页面，想要更改欢迎文字，可以使用`docker exec`进入容器

    ```shell
    $ docker exec -it webserver bash
    root@f02d8e6d11a6:/# echo '<h1>Hello,Docker!</h1>' > /usr/share/nginx/html/index.html
    root@f02d8e6d11a6:/# exit
    exit
    ```

    - 用 `<h1>Hello, Docker!</h1>` 覆盖了 `/usr/share/nginx/html/index.html` 的内容。

    - 可以用`$ docker diff webserver`来查看具体的改动

- 使用docker commit

  ```
  docker commit [选项] <容器ID或容器名> [<仓库名>[:<标签>]]
  ```

  当我们运行一个容器的时候（如果不使用卷的话），我们做的任何文件修改都会被记录于容器存储层里。而 Docker 提供了一个 `docker commit` 命令，可以将容器的存储层保存下来成为镜像。

  换句话说，就是在原有镜像的基础上，再叠加上容器的存储层，并构成新的镜像。以后我们运行这个新镜像的时候，就会拥有原有容器最后的文件变化。

  所以`docker commit`命令，手动操作给旧的镜像添加了新的一层

  - 将上面的例子保存为镜像

    ```
    $ docker commit --author "Yang" --message "修改默认网页" webserver nginx:v2
    
    sha256:ecb930469c1667a7d3bc2f225452441736bace2d79d0e1c326e8577cae3b3927
    ```

    - --author：指定修改的作者
    - --message：记录本次修改的内容

  - 可以用`docker image ls`看到这个新定制的镜像

  - 还可以用`docker history nginx:v2`查看进行的修改

  - 运行这个新的镜像

    ```
    docker run --name web2 -d -p 81:80 nginx:v2
    ```

    - 这里命名为新的服务web2,并映射到81端口
    - 访问localhost:81的结果和修改过后的webserver是一致的

- 缺点：

  1. 如果使用 `docker commit` 制作镜像，以及后期修改的话，每一次修改都会让镜像更加臃肿一次，所删除的上一层的东西并不会丢失，会一直如影随形的跟着这个镜像，即使根本无法访问到。这会让镜像更加臃肿。
  2. 使用 `docker commit` 意味着所有对镜像的操作都是黑箱操作，生成的镜像也被称为 **黑箱镜像**，换句话说，就是除了制作镜像的人知道执行过什么命令、怎么生成的镜像，别人根本无从得知。即使是这个制作镜像的人，过一段时间后也无法记清具体的操作。这种黑箱镜像的维护工作是非常痛苦的。

### 5. 使用Dockerfile定制镜像

Dockerfile 是一个文本文件，其内包含了一条条的 **指令(Instruction)**，**每一条指令构建一层**，因此每一条指令的内容，就是描述该层应当如何构建。

- 用Dockerfile实现 4.commit中的那个例子

  ```shell
  # 创建一个文本文件，名字叫Dockerfile
  $ mkdir mynginx
  $ cd mynginx
  $ touch Dockerfile
  # 内容为
  FROM nginx
  RUN echo '<h1>Hello, Docker!</h1>' > /usr/share/nginx/html/index.html
  ```

- **FROM**指定基础镜像

  所谓定制镜像，那一定是以一个镜像为基础，在其上进行定制。而 `FROM` 就是指定 **基础镜像**，因此一个 `Dockerfile` 中 `FROM` 是必备的指令，并且必须是第一条指令。

  - 空白镜像`scratch`

    ```
    FROM scratch
    ...
    ```

    如果以`scratch`为基础镜像，意味着不以任何镜像为基础，接下来所写的指令将作为镜像的第一层开始存在

- **run**执行命令

  - 两种格式

    1. shell格式：`RUN <命令>`，就像直接在命令行中输入的命令一样

       `RUN echo '<h1>Hello, Docker!</h1>' > /usr/share/nginx/html/index.html`

    2. exec格式：`RUN ["可执行文件","参数1","参数2"]` 更像函数调用的格式

  - 应该每个命令对应一个RUN?不应该

    比如下面这样写会创造7层镜像，造成很多没意义的镜像，导致镜像臃肿

  ```
  FROM debian:stretch
  RUN apt-get update
  RUN apt-get install -y gcc libc6-dev make wget
  RUN wget -O redis.tar.gz "http://download.redis.io/releases/redis-5.0.3.tar.gz"
  RUN mkdir -p /usr/src/redis
  RUN tar -xzf redis.tar.gz -C /usr/src/redis --strip-components=1
  RUN make -C /usr/src/redis
  RUN make -C /usr/src/redis install
  ```

  正确写法：

  利用`&&`将各个需要的命令串联起来，将之前的7层简化为1层。

  另外每层创建的东西会一直跟随镜像，所以要确保每一层只添加真正需要添加的东西，任何无关的东西都要删除，比如安装包、展开文件啊之类的

  ```
  FROM debian:stretch
  RUN set -x; buildDeps='gcc libc6-dev make wget' \
      && apt-get update \
      && apt-get install -y $buildDeps \
      && wget -O redis.tar.gz "http://download.redis.io/releases/redis-5.0.3.tar.gz" \
      && mkdir -p /usr/src/redis \
      && tar -xzf redis.tar.gz -C /usr/src/redis --strip-components=1 \
      && make -C /usr/src/redis \
      && make -C /usr/src/redis install \
      && rm -rf /var/lib/apt/lists/* \
      && rm redis.tar.gz \
      && rm -r /usr/src/redis \
      && apt-get purge -y --auto-remove $buildDeps
  ```

- **构建**镜像docker build

  `docker build [选项] <上下文路径/URL/->`

  在`Dockerfile`文件所在目录执行：

  `$ docker build -t nginx:v3 .`   .  指的是上下文路径

- 镜像构建**上下文**(Context)

  Docker 在运行时分为 Docker 引擎（也就是服务端守护进程）和客户端工具。Docker 的引擎提供了一组 REST API，被称为 [Docker Remote API](https://docs.docker.com/develop/sdk/)，而如 `docker` 命令这样的客户端工具，则是通过这组 API 与 Docker 引擎交互，从而完成各种功能。

  因此，虽然表面上我们好像是在本机执行各种 `docker` 功能，但实际上，一切都是使用的远程调用形式在服务端（Docker 引擎）完成。但经常会需要将一些本地文件复制进镜像，比如通过 `COPY` 指令、`ADD` 指令等。而 `docker build` 命令构建镜像，其实并非在本地构建，而是在服务端，也就是 Docker 引擎中构建的。那么在这种客户端/服务端的架构中，如何才能让服务端获得本地文件呢？

  **通过上下文：**

  这就引入了上下文的概念。当构建的时候，用户会指定构建镜像上下文的路径，`docker build` 命令得知这个路径后，会将路径下的所有内容打包，然后上传给 Docker 引擎。这样 Docker 引擎收到这个上下文包后，展开就会获得构建镜像所需的一切文件。所以`COPY` 这类指令中的源文件的路径都是*相对路径*。

  一般来说，应该会将 `Dockerfile` 置于一个空目录下，或者项目根目录下。如果该目录下没有所需文件，那么应该把所需文件复制一份过来。如果目录下有些东西确实不希望构建时传给 Docker 引擎，那么可以用 `.gitignore` 一样的语法写一个 `.dockerignore`，该文件是用于剔除不需要作为上下文传递给 Docker 引擎的。

  **个人理解**

  Docker 构建上下文就是 Docker 客户端上传给服务端的 tar 文件解压后的内容，也即 `docker build` 命令行后面指定路径下的文件。

  **`docker build`时注意：**

  - 如果构建镜像时没有明确指定 Dockerfile，那么 Docker 客户端默认在构建镜像时指定的上下文路径下找名字为 Dockerfile 的构建文件
  - Dockerfile 可以不在构建上下文路径下，此时需要构建时通过 `-f` 参数明确指定使用哪个构建文件，并且名称可以自己任意命名。

- 不通过上下文路径来构建镜像

  - 用Git repo来构建

    ```
    $ docker build -t hello-world https://github.com/docker-library/hello-world.git#master:amd64/hello-world
    ```

    - 制定了构建所需的Git repo,并选定为master分支，构建目录为/amd64/hello-world/
    - 然后 Docker 就会自己去 `git clone` 这个项目、切换到指定分支、并进入到指定目录后开始构建

  - 使用给定的tar压缩包构建

    ```
    $ docker build http://server/context.tar.gz
    ```

    - 如果所给出的 URL 不是个 Git repo，而是个 `tar` 压缩包，那么 Docker 引擎会下载这个包，并自动解压缩，以其作为上下文，开始构建。do

  - 从标准输入中读取 Dockerfile 进行构建

    ```
    docker build - < Dockerfile
    或者
    cat Dockerfile | docker build -
    ```

    - 如果标准输入传入的是文本文件，则将其视为 `Dockerfile`，并开始构建。这种形式由于直接从标准输入中读取 Dockerfile 的内容，它没有上下文，因此不可以像其他方法那样可以将本地文件 `COPY` 进镜像之类的事情。

  - 从标准输入中读取上下文压缩包进行构建

    ```
    $ docker build - < context.tar.gz
    ```

    - 如果发现标准输入的文件格式是 `gzip`、`bzip2` 以及 `xz` 的话，将会使其为上下文压缩包，直接将其展开，将里面视为上下文，并开始构建。

### 6.Dockerfile各指令

#### WORKDIR指定工作目录

格式为：`WORKDIR <工作目录路径>`

使用 `WORKDIR` 指令可以来指定工作目录（或者称为当前目录），以后各层的当前目录就被改为指定的目录，如该目录不存在，`WORKDIR` 会帮你建立目录。

## 操作容器

简单的说，容器是独立运行的一个或一组应用，以及它们的运行态环境。对应的，虚拟机可以理解为模拟运行的一整套操作系统（提供了运行态环境和其他系统环境）和跑在上面的应用。

### 0.查看容器的信息

```
$ docker container ls

查看终止状态的容器：
$ docker container ls -a
```

### 1. 启动容器

启动容器有两种方式，一种是基于镜像新建一个容器并启动，另外一个是将在终止状态（`exited`）的容器重新启动。

- 新建并启动`docker run`

  ```
  $ docker run ubuntu:20.04 /bin/echo 'Hello world'
  Hello world
  ```

  启动一个bash终端

  ```
  $ docker run -t -i ubuntu:20.04 /bin/bash
  root@3c02bb5a8af2:/#
  ```

  - `docker run --help`查看所有参数功能
  - -t：让Docker分配一个伪终端
  - -i：让容器的标准输入保持打开。

- 启动已经终止的容器：`docker container start [OPTIONS] CONTAINER [CONTAINER]...` 

  直接将一个已经终止（`exited`）的容器启动运行。

- 在创建容器时，Docker在后台运行的标准操作：

  1. 检查本地是否存在指定的镜像，不存在就从 registry下载
  2. 利用镜像创建并启动一个容器
  3. 分配一个文件系统，并在只读的镜像层外面挂载一层可读写层
  4. 从宿主主机配置的网桥接口中桥接一个虚拟接口到容器中去
  5. 从地址池配置一个 ip 地址给容器
  6. 执行用户指定的应用程序
  7. 执行完毕后容器被终止
  
- 容器的核心为所执行的应用程序，所需要的资源都是应用程序运行所必需的。除此之外，并没有其它的资源。可以在伪终端中利用 `ps` 或 `top` 来查看进程信息。
  
  可见，容器中仅运行了指定的 bash 应用。这种特点使得 Docker 对资源的利用率极高，是货真价实的轻量级虚拟化。
  

### 2. 守护态运行

更多的时候，需要让 Docker 在后台运行而不是直接把执行命令的结果输出在当前宿主机下。此时，可以通过添加 `-d` 参数来实现。

- 不使用-d参数来运行容器

  ```
  $ docker run ubuntu:20.04 /bin/sh -c "while true; do echo hello world; sleep 1; done"
  hello world
  hello world
  hello world
  hello world
  。。。
  ```

  可见容器会把输出的结果(STDOUT)打印到宿主机上面

- 使用-d参数来运行机器

  ```
  $ docker run -d ubuntu:20.04 /bin/sh -c "while true; do echo hello world; sleep 1; done"
  a1a9b4ee4b608c1714da75e24399757f682e2ed7ea6194e48049a6002bf96bd9
  ```

  此时容器会在后台运行并不会把输出的结果 (STDOUT) 打印到宿主机上面(输出结果可以用 `docker logs [options] container` 查看)。

  使用-d参数，返回的是唯一的id。

- 获得容器的输出信息

  ```
  $ docker logs [options] [container ID or NAMES]
  或者
  $ docker container logs [options] [container ID or NAMES]
  ```

### 3. 终止

- 终止一个运行中的容器

  `docker container stop [OPTIONS] CONTAINER [CONTAINER...]`

- 重启一个容器

  `$ docker container restart [OPTIONS] CONTAINER [CONTAINER...] ` 

- 单个容器可以用ctrl+c/d来关闭
### 4. 进入容器

在`docker run`使用 `-d` 参数时，容器启动后会进入后台。

某些时候需要进入容器进行操作，包括使用 `docker attach` 命令或 `docker exec` 命令

推荐使用 `docker exec` 命令

- docker attach

  ```shell
  $ docker container ls
  CONTAINER ID   IMAGE          COMMAND                  CREATED          STATUS          PORTS     NAMES
  a1a9b4ee4b60   ubuntu:20.04   "/bin/sh -c 'while t…"   13 minutes ago   Up 13 minutes             crazy_solomon
  3c02bb5a8af2   ubuntu:20.04   "/bin/bash"              32 minutes ago   Up 16 minutes             musing_williamson
  $ docker attach 3c02
  ```

  这时候如果ctrl+c/d从这个stdin中exit,会导致容器的停止

- docker exec

  ```shell
  $ docker container ls
  CONTAINER ID   IMAGE          COMMAND                  CREATED          STATUS              PORTS     NAMES
  a1a9b4ee4b60   ubuntu:20.04   "/bin/sh -c 'while t…"   20 minutes ago   Up 20 minutes                 crazy_solomon
  3c02bb5a8af2   ubuntu:20.04   "/bin/bash"              38 minutes ago   Up About a minute             musing_williamson
  
  
  # 只用 `-i` 参数时，由于没有分配伪终端，界面没有我们熟悉的 Linux 命令提示符，但命令执行结果仍然可以返回。
  PS C:\Users\51212> docker exec -i 3c02 bash
  ls
  bin
  ..
  
  # 当 -i -t 参数一起使用时，则可以看到我们熟悉的 Linux 命令提示符。
  $ docker exec -it 3c02 bash
  root@3c02bb5a8af2:/#
  ```
  
  如果从这个 stdin 中 exit，不会导致容器的停止

### 5. 导入和导出

- 导出容器快照：`docker export`

  ```shell
  $ docker container ls
  CONTAINER ID   IMAGE          COMMAND                  CREATED          STATUS          PORTS     NAMES
  a1a9b4ee4b60   ubuntu:20.04   "/bin/sh -c 'while t…"   25 minutes ago   Up 25 minutes             crazy_solomon
  3c02bb5a8af2   ubuntu:20.04   "/bin/bash"              43 minutes ago   Up 6 minutes              musing_williamson
  $ docker export 3c02 > ubuntu.tar
  ```

  - 将导出容器快照到本地文件,可以拿去给别的机器去用
  - 在哪个目录下执行的命令，就导出在哪

- 导入容器快照：`docker import`

  ```
  $ cat ubuntu.tar | docker import - test/ubuntu:v1.0
  ```

  - 用户既可以使用 `docker load` 来导入镜像存储文件到本地镜像库，也可以使用`docker import` 来导入一个容器快照到本地镜像库。

    这两者的区别在于**容器快照文件**将丢弃所有的历史记录和元数据信息（即仅保存容器当时的快照状态），而**镜像存储文件**将保存完整记录，体积也要大。此外，从容器快照文件导入时可以重新指定标签等元数据信息。

### 6. 删除

- 用 `docker container rm` 来删除一个处于终止状态的容器

```shell
$ docker container ls -a
CONTAINER ID   IMAGE          COMMAND                  CREATED          STATUS                        PORTS     NAMES
a1a9b4ee4b60   ubuntu:20.04   "/bin/sh -c 'while t…"   37 minutes ago   Up 37 minutes                           crazy_solomon
8be318655ccf   ubuntu:20.04   "/bin/sh -c 'while t…"   43 minutes ago   Exited (137) 41 minutes ago             magical_zhukovsky

$ docker container rm magical_zhukovsky
magical_zhukovsky
```

- 如果要删除一个运行中的容器，可以添加 `-f` 参数。Docker 会发送 `SIGKILL` 信号给容器。

 ```shell
 $ docker container rm -f crazy_solomon
 crazy_solomon
 ```

- 清理所有处于终止状态的容器

  ```
  $ docker container prune
  ```

  

## 访问仓库

### Docker Hub

Docker 官方维护的一个公共仓库 [Docker Hub](https://hub.docker.com)，其中已经包括了数量超过 [2,650,000](https://hub.docker.com/search/?type=image) 的镜像。大部分需求都可以通过在 Docker Hub 中直接下载镜像来实现

- 查找官方仓库中的镜像`docker search`

  ```
  $ docker search centos
  NAME                               DESCRIPTION                                     STARS     OFFICIAL   AUTOMATED
  centos                             The official build of CentOS.                   6449      [OK]
  ansible/centos7-ansible            Ansible on Centos7                              132                  [OK]
  consol/centos-xfce-vnc             Centos container with "headless" VNC session…   126                  [OK]
  jdeathe/centos-ssh                 OpenSSH / Supervisor / EPEL/IUS/SCL Repos - …   117                  [OK]
  centos/systemd                     systemd enabled base container.                 96                   [OK]
  ```

  - 可以看到返回了很多包含关键字的镜像，其中包括镜像名字、描述、收藏数（表示该镜像的受关注程度）、是否官方创建（`OFFICIAL`）、是否自动构建 （`AUTOMATED`）。

  - 在查找的时候通过 `--filter=stars=N` 参数可以指定仅显示收藏数量为 `N` 以上的镜像。

- 推送镜像

  用户也可以在登录后通过 `docker push` 命令来将自己的镜像推送到 Docker Hub。

  以下命令中的 `username` 请替换为你的 Docker 账号用户名。

  ```shell
  $ docker tag ubuntu:20.04 superyaaaang/ubuntu:20.04
  $ docker image ls
  REPOSITORY                 TAG       IMAGE ID       CREATED       SIZE
  ubuntu                     20.04     ba6acccedd29   4 weeks ago   72.8MB
  superyaaaang/ubuntu20.04   latest    ba6acccedd29   4 weeks ago   72.8MB
  $ docker push superyaaaang/ubuntu:20.04
  
  $ docker search superyaaaang
  ```

### 私有仓库

[`docker-registry`](https://docs.docker.com/registry/) 是官方提供的工具，可以用于构建私有的镜像仓库。

- 安装运行docker-registry

  创建仓库

  ```shell
  # 默认情况下，仓库会被创建在容器的 /var/lib/registry 目录下
  # 启动一个Registry应用2.0版本的容器，并将5000端口绑定到本地宿主机
  $ docker run -d -p 5000:5000 --name registry registry:2
  
  # 指令路径
  $ docker run -d -p 5000:5000 -v ~/Desktop/registry:/var/lib/registry registry registry:2
  $ docker run -d -p 5000:5000 -v E:/registry:/var/lib/registry registry:latest
  ```

  检查是否创建成功

  ```
  windows:
  shell中输入：ipconfig。找到Ethernet adapter Ethernet下面的IPV4 Address,得到：
  192.168.0.105
  在浏览器中输入192.168.0.105:5000/v2/，得到{}就是对的。
  
  linux:
  terminal中输入：ip address。找到enx..下的inet得到
  192.168.0.106
  在浏览器中输入192.168.0.106:5000/v2/，得到{}就是对的
  ```

  查看本机已有镜像

  ```
  $ docker image ls
  REPOSITORY   TAG       IMAGE ID       CREATED        SIZE
  registry     2         b8604a3fe854   31 hours ago   26.2MB
  ubuntu       20.04     ba6acccedd29   4 weeks ago    72.8MB
  ```

  查看私有仓库地址

  ```
  $ docker info
  得到一个Insecure Registries:
  127.0.0.0/8
  
  但其实是127.0.0.1
  ```

  标记一个想要上传私有仓库的镜像.

  格式为 `docker tag IMAGE[:TAG] [REGISTRY_HOST[:REGISTRY_PORT]/]REPOSITORY[:TAG]`。

  ```
  $ docker tag ubuntu:20.04 127.0.0.1:5000/ubuntu:latest
  ```

  使用`docker push `上传标记的镜像

  ```
  $ docker push 127.0.0.1:5000/ubuntu:latest
  ```

  用`curl`查看仓库中的镜像

  ```
  $ curl 127.0.0.1:5000/v2/_catalog
  Content           : {"repositories":["ubuntu"]}
  ```

  先删除已有镜像，再尝试从私有仓库中下载这个镜像

  ```
  $ docker image rm 127.0.0.1:5000/ubuntu:latest
  
  $ docker pull 127.0.0.1:5000/ubuntu:latest
  ```

  

# Docker应用

## \#、使用SSH从GITHUB上下载代码

注意，如果用的是id_e25519，需要将下面的id_rsa全部改成id_e25519

- 在Dockerfile中

  ```dockerfile
  # 从shell中读取ssh值
  ARG ssh_prv_key
  
  # Authorize SSH Host
  RUN mkdir -p /root/.ssh && \
      chmod 0700 /root/.ssh && \
      ssh-keyscan gitlab.lrz.de > /root/.ssh/known_hosts
  
  # Add the keys and set permissions
  RUN echo "$ssh_prv_key" > /root/.ssh/id_rsa && \
      chmod 600 /root/.ssh/id_rsa
      
  # clone your git using SSH connection
  WORKDIR /home
  RUN git clone git@gitlab.lrz.de:ge23ged/autonomous-systems-2021-group-terminus.git
  RUN cd autonomous-systems-2021-group-terminus/Lab4/src &&\
      git submodule init &&\
      git submodule update
  
  # Remove SSH keys from docker
  RUN rm -rf /root/.ssh/
  
  ```

- 在build Dockerfile时，shell中要加--build-arg ssh_prv_key="$(cat ~/.ssh/id_rsa)"

  ```shell
  sudo docker build -t lab_4_controller_pkg_terminus . --build-arg ssh_prv_key="$(cat ~/.ssh/id_rsa)" --no-cache 
  ```

  运行该镜像

  ```shell
  sudo docker run -it -d --network=host --name lab_4_controller_pkg_terminus lab_4_controller_pkg_terminus:latest bash
  ```

  打开容器

  ```shell
  sudo docker start lab_4_controller_pkg_terminus
  ```

  进入交互

  ```shell
  sudo docker exec -it lab_4_controller_pkg_terminus bash
  ```

  



