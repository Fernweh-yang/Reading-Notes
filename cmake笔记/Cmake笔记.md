# 1.G++,Makefile,Cmake简述

## 1.1 g++

- g++是c++的一个编译器

- 最简单的例子：

  命令行输入：

  ```
  g++ helloSlam.cpp
  ```

  会生成一个a.out的可执行文件：

  ```
  # 命令行输入如下指令来运行
  ./a.out 
  ```

- 任意一个c++程序都可以用g++来编译，但任意一个项目都可能含有十几个类，各类之间还存在复杂的依赖关系。这些类一部分要编译为可执行文件，一部分编译为库文件。

  所以如果单用g++，整个编译过程会异常繁琐。

  由此产生了方便的makefile以及更方便的cmake

  

## 1.2 Makefile文件

### 1.2.1make

- 功能：读取Makefile,按里面所写的内容，将文件进行编译。
- make命令本身不知道如何做出该文件，需要知道一系列规则
- 编译compile：代码从高级语言 -> 汇编语言。 构建：编译的顺序安排(即上述的规则)
- 这一系列规则写在Makefile文件里。
- 使用：
  - `make <>`编译
  - `make clean`删除.o文件

Makefile概述

- Makefile文件由一系列描述整个工程的编译和链接的规则所构成。   
  规则形式:
  
  ```
  <target> : <prerequisites>
  [tab] <commands>	
  ```
  - target:目标
    - 必须有，不可省略
    - 可以是文件名，也可以是某一个操作的名字(称之为伪目标)
  - prerequisites:前置条件
    - 非必须,但和command命令之间必须至少存在一个。
  - tab:空格
    - 必须在第二行前输入一个tab
  - commands:命令
    - 非必须,但和prerequisites前置条件之间必须至少存在一个
  - 例子：
  ```makefile
  test:test.c
	gcc -o test test.c
   
  #test为目标文件，也是我们最终生成的可执行文件。
  #test.c是依赖文件
  #gcc -o test test.c 重建目标文件 
  ```
  
- 编译过程和其生成的文件
	- 第一步:编译 complie
		- 1.第一个阶段：编译
			- 读取源代码(.c,.cpp)程序，先进行伪指令(#开头的）的编译，然后对其进行词法和语法分析，将高级语言指令转化为汇编代码
			- 预编译：文件先从.c转为转为.i文件。 gcc -E
			- 编译：文件从.i转为.s文件。 gcc -S
			- 后缀有: .s, .asm
		- 2.第二个阶段:汇编
		    - 将汇编语言翻译成目标机器指令
		    - 将.s文件转化成.o文件。 gcc -c
		    - 后缀有：.obj, .o, .a, .ko
	- 第二部:链接link
		- 将有关的目标彼此相连(因为代码中会引用其他源文件的函数/方法)
		- 从而使.o文件转化成可执行文件。 gcc
		- 后缀有：.exe, .elf, .axf
		
	- 涉及到的各种文件：
		- .c/.h文件
			- .h文件是头文件，内涵函数声明、宏定义、结构体定义等内容
			- .c文件时程序文件，内涵函数实现,变量定义等内容
			- 无本质区别	
		- .o文件（目标文件/中间文件）
			- 作用：检查某个源文件是不是进行过修改，最终目标文件是不是需要重建
			- 由于会产生各种中间文件让文件夹看得很乱，所以要在Makefile文件的末尾清除中间文件
			```makefile
			.PHONY:clean	#表明clean是一伪目标
			clean:
				rm -rf *.o test	#"*.o"是所有的中间文件，test是最终生成的执行文件
			```
	

### 1.2.2Makefile中的变量。

- 定义和引用
```makefile
VALUE_LIST = one two three
	
OBJ=main.o test.o test1.o test2.o	#变量名OBJ
test:$(OBJ)							#引用变量，$(OBJ)
    gcc -o test $(OBJ)
```
- 变量的赋值
	- :=,简单赋值。只对当前语句的变量有效
	- =，递归赋值。所有目标变量相关的其他变量都受影响
	- ?=条件赋值。如果未来未定义，则使用符号中的值定义变量
	- +=追加赋值。原变量用空格隔开的方式追加一个新值。

- 自动化变量

  - 规则的命令是对所有这一类文件的描述。我们在 Makefile 中描述规则时，依赖文件和目标文件是变动的，显然在命令中不能出现具体的文件名称，否则模式规则将失去意义。
  - 

  | 自动化变量 | 说明 |
  | ---------- | -------------- |
  | @         | 表示规则的目标文件名。如果目标是一个文档文件（Linux 中，一般成 .a 文件为文档文件，也称为静态的库文件），那么它代表这个文档的文件名。在多目标模式规则中，它代表的是触发规则被执行的文件名。 |
  | %         | 当目标文件是一个静态库文件时，代表静态库的一个成员名。       |
  | <         | 规则的第一个依赖的文件名。如果是一个目标文件使用隐含的规则来重建，则它代表由隐含规则加入的第一个依赖文件。 |
  | ?         | 所有比目标文件更新的依赖文件列表，空格分隔。如果目标文件时静态库文件，代表的是库文件（.o 文件）。 |
  | ^         | 代表的是所有依赖文件列表，使用空格分隔。如果目标是静态库文件，它所代表的只能是所有的库成员（.o 文件）名。一个文件可重复的出现在目标的依赖中，变量“\^”只记录它的第一次引用的情况。就是说变量“\^”会去掉重复的依赖文件。 |
  | +         | 类似“^”，但是它保留了依赖文件中重复出现的文件。主要用在程序链接时库的交叉引用场合。 |
  | *         | 在模式规则和静态模式规则中，代表“茎”。“茎”是目标模式中“%”所代表的部分（当文件名中存在目录时，“茎”也包含目录部分）。 |

  - 例子

  ```makefile
  test:test.o test1.o test2.o
           gcc -o $@ $^		#"$@" 代表的是目标文件test,“$^”代表的是依赖的文件
  test.o:test.c test.h
           gcc -o $@ $<		#“$<”代表的是依赖文件中的第一个
  test1.o:test1.c test1.h
           gcc -o $@ $<
  test2.o:test2.c test2.h
           gcc -o $@ $<
  # 执行 make 的时候，make 会自动识别命令中的自动化变量，并自动实现自动化变量中的值的替换
  ```


​	  

### 1.2.3Makefile目标文件搜索

如果需要的文件是存在于不同的路径下，在编译的时候要去怎么办？

- 两种搜索方法

  - VPATH：VPATH 是变量，更具体的说是环境变量，Makefile 中的一种特殊变量，使用时需要指定文件的路径；

    - 用法

    ```makefile
    VPATH := src car 	#多个路径的时候用空格或冒号隔开
    VPATH := src:car	#搜索的顺序为书写的顺序,先src再car
    ```

    - 实例

    ```makefile
    VPATH=src car
    test:test.o
        gcc -o $@ $^
    ```

  - vapth: vpath 是关键字，按照模式搜索，也可以说成是选择搜索。搜索的时候不仅需要加上文件的路径，还需要加上相应限制的条件。

    - 三种用法

    ```makefile
    1) vpath PATTERN DIRECTORIES #PATTERN：可以理解为要寻找的条件，
    2) vpath PATTERN			 #DIRECTORIES：寻找的路径 
    3) vpath
    ```

    - 实例

    ```makefile
    #第一种用法：
    vpath test.c src car 	 #多个路径的时候用空格或冒号隔开
    vpath test.c src：car    #在src和car下搜索test.c文件
    vapth %.c src 			 #搜索路径下所有.c文件
    #第二种用法：
    vpath test.c			 #清除符合文件 test.c 的搜索目录。
    #第三种用法
    vpath					 #清除所有已被设置的文件搜索路径。	
    ```

    

### 1.2.4Makefile条件判断

| **关键字** | **功能**                                            |
| ---------- | --------------------------------------------------- |
| ifeq       | 判断参数是否不相等，相等为 true，不相等为 false。   |
| ifneq      | 判断参数是否不相等，不相等为 true，相等为 false。   |
| ifdef      | 判断是否有值，有值为 true，没有值为 false。         |
| ifndef     | 判断是否有值，没有值为 true，有值为 false。         |
| else       | 表示当条件不满足的时候执行的部分                    |
| endif      | 是判断语句结束标志，Makefile 中条件判断的结束都要有 |

- 实例

  - ifeq,ifneq

  ```makefile
  libs_for_gcc= -lgnu
  normal_libs=
  ifeq($(CC),gcc)
      libs=$(libs_for_gcc)
  else
      libs=$(normal_libs)
  endif
  foo:$(objects)
      $(CC) -o foo $(objects) $(libs)
  ```

  - ifdef,ifndef

  ```makefile
  bar =
  foo = $(bar)
  all:
  ifdef foo
      @echo yes
  else
      @echo  no
  endif
  ```

### 1.2.5Makefile伪目标

- 目的
  - 避免我们的 Makefile 中定义的只执行的命令的目标和工作目录下的实际文件出现名字冲突。
  - 在 make 的并行和递归执行的过程中，此情况下一般会存在一个变量，定义为所有需要 make 的子目录。

- 声明伪目标

```makefile
.PHONY:clean
```



## 1.3 cmake

### 1.3.1作用

- 可以用cmake命令生成一个makefile文件。然后用make命令根据这个makefile文件的内容来编译整个工程。

### 1.3.2流程

#### 1. 编写代码 

并不是所有的代码都会编译为可执行文件

- 可执行程序：

  只有带有main函数的文件才会生成可执行程序。

  如：helloSlam.cpp

  ```c++
  #include "libHelloSlam.h"
  
  int main(int argc, char **argv){
      printHello();
      return 0;
  }
  ```

- 库：

  供其他程序调用用的代码。

  如：libHelloSlam.cpp

  ```c++
  #include <iostream>
  using namespace std;
  
  void printHello(){
      cout << "Hello Slam" << endl;
  }
  ```

  编译后会成为.a或.so文件。

- 头文件：

  库文件.a/.so是编译好的二进制文件，为了使用这个库，我们需要提供头文件。

  对于库的使用者，只要拿到了头文件和库文件就可以调用这个库。

  如：libHelloSlamh

  ```c++
  #ifndef LIBHELLOSLAM_H_
  #define LIBHELLOSLAM_H_
  //上面的宏定义是为了防止重复引用这个头文件而引起的重定义错误
  
  void printHello()
  
  #endif
  ```

#### 2. 创建CMakeLists.txt

```cmake
# 声明已要求的cmake最低版本
cmake_minimum_required(VERSION 2.8)

# 声明一个cmake工程
project(HelloSLAM)

# 添加一个静态库
# 后缀名为.a
# 每次调用都会生成一个副本
add_library(hello libHelloSlam.cpp)

# 添加一个共享库
# 后缀名为.so
# 只有一个副本，更省空间
add_library(hello_shared SHARED libHelloSlam.cpp)

# 添加一个可执行程序
# 语法：add_executable(程序名 源代码文件)
add_executable(helloSlam helloSlam.cpp)

# 令helloSlam程序可以使用hello_shared库中的代码
target_link_libraries(helloSlam hello_shared)
```

- 我们对项目的编译管理工作，从输入一堆g++命令，转为了管理若干个比较直观的CMakeLists.txt文件。
- 每增加一个可执行文件，只要多加一行add_executable()
- cmake会帮我们自动解决代码的依赖关系

#### 3. 创建makefile并make

在终端输入

```shell
# cmake命令会自动生成一堆中间文件
# 创建一个build子文件夹用来放置这些中间文件
# 这样要push代码的话，直接把build文件夹删了就行。
mkdir build			
cd build 
# 进入buid文件夹后，要编译的内容在上面一个目录
cmake ..
make
```



### 1.3.3 修改cmake安装地址

创建完build后，需要在这里创建makefile文件：

这时候修改地址：myfolder

```
cmake -DCMAKE_INSTALL_PREFIX:PATH=myfolder ..
```



# 2. CMake学习笔记

## 2.0 学习资源汇总

[官方教程](https://cmake.org/cmake/help/latest/guide/tutorial/index.html)

[笔记所学习的教程](https://cliutils.gitlab.io/modern-cmake/chapters/basics.html)

[所有指令都可以在文档查询](https://cmake.org/cmake/help/latest/index.html#)

## 2.1 介绍

### 2.1.1 How to run CMake

- 编译：

  ```shell
  # 方法1：使用make或者ninja
  ~/package $ mkdir build
  ~/package $ cd build
  ~/package/build $ cmake ..
  ~/package/build $ make
  
  # 方法2.1：使用camke提供的generator				（推荐方法2.1）
  ~/package $ mkdir build
  ~/package $ cd build
  ~/package/build $ cmake ..
  ~/package/build $ cmake --build .
  
  # 方法2.2：不用先去创建build文件夹，但其实第一步也同时创建了build文件夹.
  ~/package $ cmake -S . -B build
  ~/package $ cmake --build build
  ```

  - [cmake的generator](https://cmake.org/cmake/help/latest/manual/cmake-generators.7.html#cmake-generators)使得cmake可以支持不同的底层，比如makefile和ninja。

    `cmake --build .`就可以让我们无需管当前环境的底层是什么直接编译，不用再考虑使用底层命令`make`还是`ninja`。从而实现了跨平台的能力。

  - 之所以要在build文件夹中编译：

    1. 编译会生成很多中间文件，为了让工作目录更整洁，可以将编译目录单独分割出来
    2. 同1，让我们git时可以直接ignore 文件夹build

- 安装

  ```shell
  # From the build directory (三种方法：推荐第二种)
  ~/package/build $ make install
  ~/package/build $ cmake --build . --target install
  ~/package/build $ cmake --install . # CMake 3.15+ only
  
  # From the source directory (三种方法：推荐第二种)
  ~/package $ make -C build install
  ~/package $ cmake --build build --target install
  ~/package $ cmake --install build # CMake 3.15+ only
  ```
  
- 一些cmake的常用编译选项

  - `-DCMAKE_BUILD_TYPE=` Pick from Release, RelWithDebInfo, Debug, or sometimes more.
  - `-DCMAKE_INSTALL_PREFIX=` The location to install to. System install on UNIX would often be `/usr/local` (the default), user directories are often `~/.local`, or you can pick a folder.
  - `-DBUILD_SHARED_LIBS=` You can set this `ON` or `OFF` to control the default for shared libraries (the author can pick one vs. the other explicitly instead of using the default, though)
  - `-DBUILD_TESTING=` This is a common name for enabling tests, not all packages use it, though, sometimes with good reason.
  - `--trace` This option will print every line of CMake that is run.
    - CMake 3.7 added `--trace-source="filename"`, which will print out every executed line of just the file you are interested in when it runs



### 2.1.2 Configure/Build/Install time

编译一个CMake项目，可以分成以下三个阶段

#### 2.1.2.1 Configure time:

- This includes everything that happens while running `cmake` itself. 

- This phase is concerned with inspecting检查 certain properties of the host system and generating the specific build files for that platform under the selected configuration.

#### 2.1.2.2 Build time

- This includes everything that happens while actually building your project from the files generated by CMake (like, when running `cmake --build` or `make`). 
- This is where all of the actual compilation and linking happens, so at the end of the build phase, you have a usable binary.

#### 2.1.2.3 Install time

- This includes everything that happens when running the `INSTALL` target generated by CMake (like, when running `cmake --build --target install` or `make install`). 
- This takes care of **copying** the binaries that were generated into the build tree to a different directory. 
- It could also include additional steps that change the binaries to make them more **portable**.
- Note that the build tree contains a lot of stuff that is no longer needed once the build is completed if you are only interested in running the binary.

### 2.1.3 Do's and Don'ts

参见[Effective Modern CMake](https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1)

#### 2.1.3.1 CMake Antipatterns

- **Do not use global functions**: This includes `link_directories`, `include_libraries`, and similar.
- **Don't add unneeded PUBLIC requirements**: You should avoid forcing something on users that is not required (`-Wall`). Make these PRIVATE instead.
- **Don't GLOB files**: Make or another tool will not know if you add files without rerunning CMake. Note that CMake 3.12 adds a `CONFIGURE_DEPENDS` flag that makes this far better if you need to use it.
- **Link to built files directly**: Always link to targets if available.
- **Never skip PUBLIC/PRIVATE when linking**: This causes all future linking to be keyword-less.

#### 2.1.3.2 CMake Patterns

- **Treat CMake as code**: It is code. It should be as clean and readable as all other code.
- **Think in targets**: Your targets should represent concepts. Make an (IMPORTED) INTERFACE target for anything that should stay together and link to that.
- **Export your interface**: You should be able to run from build or install.
- **Write a Config.cmake file**: This is what a library author should do to support clients.
- **Make ALIAS targets to keep usage consistent**: Using `add_subdirectory` and `find_package` should provide the same targets and namespaces.
- **Combine common functionality into clearly documented functions or macros**: Functions are better usually.
- **Use lowercase小写 function names**: CMake functions and macros宏指令 can be called lower or upper case. Always use lower case. Upper case is for variables.
- **Use `cmake_policy` and/or range of versions**: Policies change for a reason. Only piecemeal set OLD policies if you have to.

## 2.2 基础

### 2.2.1 CMakeLists.txt基本组成

```cmake
# 1.版本不同有不同的政策：https://cmake.org/cmake/help/latest/manual/cmake-policies.7.html
cmake_minimum_required(VERSION 3.7)
# 如果版本低于3.24就用当前版本的cmake，如果高于3.24则用3.24版本的cmake
if(${CMAKE_VERSION} VERSION_LESS 3.24)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
else()
    cmake_policy(VERSION 3.24)
endif()
# 2.项目名MyProject是必须的，其他都是可选项。其中选项languages 可以是 C, CXX, Fortran, ASM, CUDA (CMake 3.8+), CSharp (3.8+), and SWIFT (CMake 3.15+ experimental).
project(MyProject VERSION 1.0
                  DESCRIPTION "Very nice project"
                  LANGUAGES CXX)
  
  
# 3.添加一个可执行文件executable
# one是生成的executable和CMake target的名字。
# two是source file名，three是头文件通常可以省略
add_executable(one two.cpp three.h)

# 4.添加库文件library
# library的类型有STATIC, SHARED, or MODULE
add_library(one STATIC two.cpp three.h)

# 5.指定Target
# 将一个include directory添加给 target one
# PUBLIC让CMake知道任何其他target想要link到target one，都需要这个include目录。
# 	其他选项PRIVATE:只影响当前target比如这里的one,不影响其它dependencies
# 	其他选项INTERFACE:只影响dependencies(指向one的其他target)
target_include_directories(one PUBLIC include)


# 6.连接其他的targets
# 同4中一下添加库library给target another
add_library(another STATIC another.cpp another.h)
# 将前面的target one连接到target another作为another的依赖项
# 如果target one不存在，那么会连接到一个interface library one(fictional虚假的库)
target_link_libraries(another PUBLIC one)
```

### 2.2.2 变量设置

See [cmake-variables](https://cmake.org/cmake/help/latest/manual/cmake-variables.7.html) for a listing of known variables in CMake.

#### local variable局部变量

在同一个CMake工程中使用，会有作用域限制或区分

```cmake
# 1.设置一个变量
set(MY_VARIABLE "value")
# 2.设置一个list变量，下面2个等价identical
set(MY_LIST "one" "two")
set(MY_LIST "one;two")

# 3.使用要用${},比如
xxx("${}")
```

#### Cache Variables缓存变量

在同一个CMake工程中任何地方都可以使用。

```cmake
# 1.定义格式：
# 变量类型可以是BOOL、FILEPATH、PATH、STRING、INTERNAL
# 如果加了FORCE就无论如何变量都是下面的变量值。如果不加FORCE则可以通过命令行等来改变变量值
set(<变量名> <变量值列表>... CACHE <变量类型> <变量概要说明string类型> [FORCE])

# 2.例子：
# 第二行可以让变量出现在“cmake -L ..”显示的变量列表。
set(MY_CACHE_VARIABLE "VALUE" CACHE STRING "Description")
mark_as_advanced(MY_CACHE_VARIABLE)

# 3，对于bool类型的变量：ON/OFF
option(MY_OPTION "This is settable from the command line" OFF)
```

- Cache 其实是一个text文件：`CMakeCache.txt`
  - 它会在我们`cmake ..`时和makefile一起出现在 build directory里。
  - 这个文件然那个CMake知道了我们所有的选项，如此之后build前就不需要再`CMake`来创建新的makefile

#### Properties特征

特征类似于变量，但它同时还作用于directory或者target，可以用于导入外部库并设置外部库的路径

See [cmake-properties](https://cmake.org/cmake/help/latest/manual/cmake-properties.7.html) for a listing of all known properties

```cmake
add_executable(myTargetName ${DIR_SRCS})
# 两方法设置特征：
# 1.可以同时为多个targets/files/tests设置多个property
set_property(TARGET myTargetName PROPERTY 
             		CXX_STANDARD 11)
# 2.只能为1个target/files/tests设置多个property
set_target_properties(myTargetName PROPERTIES
                      CXX_STANDARD 11)
set_source_files_properties(myFileName PROPERTIES
							CXX_STANDARD 11)
set_tests_properties(myTestName PROPERTIES
					 CXX_STANDARD 11)
# 3.得到property的值
get_property(ResultVariable TARGET myTargetName PROPERTY CXX_STANDARD)
```

### 2.2.3 Cmake中的编程

#### if语句
[if文档](https://cmake.org/cmake/help/latest/command/if.html)

```cmake
# 1.用bool值
if(variable)
    # If variable is `ON`, `YES`, `TRUE`, `Y`, or non zero number
else()
    # If variable is `0`, `OFF`, `NO`, `FALSE`, `N`, `IGNORE`, `NOTFOUND`, `""`, or ends in `-NOTFOUND`
endif()
# If variable does not expand to one of the above, CMake will expand it then try again

# 2.也可以用变量名
if("${variable}")
    # True if variable is not false-like
else()
    # Note that undefined variables would be `""` thus false
endif()
```

#### generator-expressions生成器表达式

[generator-expressions文档](https://cmake.org/cmake/help/latest/manual/cmake-generator-expressions.7.html)
[一个blog参考](https://hongjh.blog.csdn.net/article/details/126453308?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7EYuanLiJiHua%7EPosition-1-126453308-blog-119993262.pc_relevant_layerdownloadsortv1&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7EYuanLiJiHua%7EPosition-1-126453308-blog-119993262.pc_relevant_layerdownloadsortv1&utm_relevant_index=1)

- 正常命令都是在configure time生成Makefile时执行的，但generator expressions是在build/install time时执行的

- 格式：`$<KEYWORD>`或者`$<KEYWORD:value`,并可以嵌套nest使用

- 用途：

  1. 用于Cmake生成构建系统时根据不同配置动态生成特定的内容

     - 条件链接：针对某一个编译目标，debug/release版本链接不同的库

     - 条件定义：针对不同编译器，定义不同的宏

     ```cmake
     # 只在Debug模式下会有参数--my-flag，其他模式下是空字符串
     target_compile_options(MyTarget PRIVATE "$<$<CONFIG:Debug>:--my-flag>")
     ```

  2. 给予build和install不同的目录

     ```cmake
     target_include_directories(
         MyTarget
       PUBLIC
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
     )
     ```

  3. 更多用途见上诉网站

#### 宏Macros和函数Functions
宏[Macros](https://cmake.org/cmake/help/latest/command/macro.html)和函数[Functions](https://cmake.org/cmake/help/latest/command/function.html)
他们唯一的区别是：宏没有作用域全局可见，函数有作用域

- 使用

  ```cmake
  # 格式：
  macro(<name> [<arg1> ...])
    <commands>
  endmacro()
  
  function(<name> [<arg1> ...])
    <commands>
  endfunction()
  
  # invocation调用
  macro(foo)					function(foo)
    <commands>					<commands>
  endmacro()					endfunction()
  ## 以下都可以
  foo()
  Foo()
  FOO()
  cmake_language(CALL foo)
  ```

- Argument实参

  当宏/函数被调用时，首先会替换parameters形参为实参，然后正常调用命令.

  利用[cmake_parse_arguments](https://cmake.org/cmake/help/latest/command/cmake_parse_arguments.html)来设置参数。

  - 例子：

  ```cmake
  function(COMPLEX)
      cmake_parse_arguments(
          COMPLEX_PREFIX
          "SINGLE;ANOTHER"
          "ONE_VALUE;ALSO_ONE_VALUE"
          "MULTI_VALUES"
          ${ARGN}
      )
  endfunction()
  
  complex(SINGLE ONE_VALUE value MULTI_VALUES some other values)
  
  #11行的输出：
  COMPLEX_PREFIX_SINGLE = TRUE		
  COMPLEX_PREFIX_ANOTHER = FALSE		# 11行中没用到
  COMPLEX_PREFIX_ONE_VALUE = "value"	
  COMPLEX_PREFIX_ALSO_ONE_VALUE = <UNDEFINED>	# 11行中没用到
  COMPLEX_PREFIX_MULTI_VALUES = "some;other;values"
  ```

  - ARGN,ARGC,ARGV参数的意义

    - **ARGC**代表的是函数或者宏传递的参数个数

    - **ARGV**代表所有传递的参数，使用list表示，其中如果函数有多个参数，要取得某个参数可以使用ARGV0，ARGV1，ARGV2等。

    - **ARGN**包含传入参数的list， 与ARGV不同的是并不是代表所有参数，而是指宏或者函数声明的参数之后的所有参数。

    - 例子：

      ```cmake
      cmake_minimum_required(VERSION 3.4.3)
       
      macro(arg_test para1 para2)
              MESSAGE(STATUS ARGC=${ARGC})
              MESSAGE(STATUS ARGV=${ARGV})
              MESSAGE(STATUS ARGN=${ARGN})
              MESSAGE(STATUS ARGV0=${ARGV0})
              MESSAGE(STATUS ARGV1=${ARGV1})
              MESSAGE(STATUS ARGV2=${ARGV2})
      endmacro()
       
      arg_test(para_1, para_2, para_3, para_4)
      
      # cmake ..后输出时是：
      -- ARGC=4
      -- ARGV=para_1,para_2,para_3,para_4
      -- ARGN=para_3,para_4
      -- ARGV0=para_1,
      -- ARGV1=para_2,
      -- ARGV2=para_3,
      ```

      

###  2.2.4 让CMake和代码交互 

#### C++读取CMake中的变量

可以从代码中访问CMake的变量，文件名以.in结束

```c++
//***** Version.h.in
#pragma once
#define MY_VERSION_MAJOR @PROJECT_VERSION_MAJOR@
#define MY_VERSION_MINOR @PROJECT_VERSION_MINOR@
#define MY_VERSION_PATCH @PROJECT_VERSION_PATCH@
#define MY_VERSION_TWEAK @PROJECT_VERSION_TWEAK@
#define MY_VERSION "@PROJECT_VERSION@"
```

Cmake中：

```cmake
configure_file (
    "${PROJECT_SOURCE_DIR}/include/My/Version.h.in"
    "${PROJECT_BINARY_DIR}/include/My/Version.h"
)
```

- 也需要包含binary地址，用于构建项目



#### CMake读取C++中的变量

```cmake
# Assuming the canonical最简洁的 version is listed in a single line
# This would be in several parts if picking up from MAJOR, MINOR, etc.
set(VERSION_REGEX "#define MY_VERSION[ \t]+\"(.+)\"")

# Read in the line containing the version
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/include/My/Version.hpp"
    VERSION_STRING REGEX ${VERSION_REGEX})

# Pick out just the version
string(REGEX REPLACE ${VERSION_REGEX} "\\1" VERSION_STRING "${VERSION_STRING}")

# Automatically getting PROJECT_VERSION_MAJOR, My_VERSION_MAJOR, etc.
project(My LANGUAGES CXX VERSION ${VERSION_STRING})
```



### 2.2.5 Structure项目应有的架构

[参照的例子](https://github.com/Fernweh-yang/Reading-Notes/tree/main/cmake%E7%AC%94%E8%AE%B0/examples/extended-project)

- 一个好的结构可以：
  - Easily read other projects following the same patterns,
  - Avoid a pattern that causes conflicts,
  - Keep from muddling and complicating your build.

- 一个项目架构应该长这样：

  ```shell
  - project
    - .gitignore
    - README.md
    - LICENCE.md
    - CMakeLists.txt
    - cmake
      - FindSomeLib.cmake
      - something_else.cmake
    - include
      - project
        - lib.hpp
    - src
      - CMakeLists.txt
      - lib.cpp
    - apps
      - CMakeLists.txt
      - app.cpp
    - tests
      - CMakeLists.txt
      - testlib.cpp
    - docs
      - CMakeLists.txt
    - extern
      - googletest
    - scripts
      - helper.py
  ```

  - **CMakeLists.txt**：要单独放出来，可以使用`add_subdirectory`来添加任意包含CMakeLists.txt的子文件夹。
  
  - **.gitignore**:应该要包含`/build*`，编译同样要放在 build文件中
  
    为了避免编译在一个错误的文件夹，可以将下列代码加入到CMakeLists.txt:
  
    ```cmake
    ### Require out-of-source builds
    file(TO_CMAKE_PATH "${PROJECT_BINARY_DIR}/CMakeLists.txt" LOC_PATH)
    if(EXISTS "${LOC_PATH}")
        message(FATAL_ERROR "You cannot build in a source directory (or any directory with a CMakeLists.txt file). Please make a build subdirectory. Feel free to remove CMakeCache.txt and CMakeFiles.")
    endif()
    ```
  
    
  
  - **extern folder**:用到的git submodules，方便我们管理依赖版本
  
  - **cmake folder**: 所有.cmake文件在的地方，.cmake文件加载后可以在CMakeList.txt中使用.cmake的一些函数和定义
  
    将这个folder加入到CMake路径
  
    ```cmake
    set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
    ```
  
    

### 2.2.6 运行其他程序

#### 2.2.6.1 在configure time运行

使用`execute_process`来运行：

一个更新所有git submodules的例子：

```cmake
find_package(Git QUIET)

if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                    RESULT_VARIABLE GIT_SUBMOD_RESULT)
    if(NOT GIT_SUBMOD_RESULT EQUAL "0")
        message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
    endif()
endif()
```

- It is generally a good idea to avoid hard coding a program path into your CMake; you can use `${CMAKE_COMMAND}`, `find_package(Git)`, or `find_program` to get access to a command to run. 

#### 2.2.6.2 在build time运行

一个运行python script的例子：

```cmake
find_package(PythonInterp REQUIRED)
add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/include/Generated.hpp"
    COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/GenerateHeader.py" --argument
    DEPENDS some_target)

add_custom_target(generate_header ALL
    DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/include/Generated.hpp")

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/Generated.hpp DESTINATION include)
```



### 2.2.7 targets

参看[教程](https://hsf-training.github.io/hsf-training-cmake-webpage/04-targets/index.html#:~:text=Targets%20are%20much%20like%20%E2%80%9Cobjects,add_library(mylibrary%20simplelib.cpp))

- 每一个target代表着一个代码文件：比如 people.cpp, man.cpp, woman.cpp, people.hpp等
- 描述了target之间的联系，就相当于告诉CMake，这个项目的结构是什么样的。各个代码文件之间的依赖关系是怎么样的。

#### 2.2.7.1 两种target

1. **executable**

   此时，target相当于其他编程语言中的对象object。

   比如：

   ```cmake
   add_executable(myexample simple.cpp simple.hpp)
   ```

   - 这里targert的名字叫myexample
   - 它有properties来存储信息，也就是member variables.
   - 它的`SOURCES`property会记录simple.cpp, simple.hpp
   - 这里hpp(head)不是必须要加的，但有更助于别人理解。

2. **library**

   将某一文件作为另个代码的库。

   比如：

   ```cmake
   add_library(mylibrary simplelib.cpp)
   ```

   - 默认是shared类型的库      
   - 我们可以自己指定库的类型：STATIC, SHARED或者MODULE

#### 2.2.7.2 链接各个targets

- 当我们有了多个target(代码文件)后，可以用`target_link_libraries`+一个keyword:`PUBLIC`,`PRIVATE` OR `INTERFACE`来描述他们的关系

  一个小例子：

  - 有意libary:`my_lib`,由`my_lib.hpp`和`my_lib.cpp`编译得到。它需要用到c++14。

  - 这时又一个libary`my_exe`,需要依赖于`my_lib`,那么`my_exe`还需要用c++14来编译吗？

    - 如果依赖在header:my_lib.hpp， 说明这 my_lib 和 C++14 之间是`PUBLIC`的关系。那么依赖于my_lib的 my_exe也需要用C++14来compile
    - 如果依赖在implementations:my_lib.cpp, 说明my_lib 和 C++14之间是`PRIVATE`的关系，那么my_exe就不需要C++14来编译了。

    - 如果要求my_exe包含C++14, 但又允许任意C++来编译，那么就是`INTERFACE`的关系。

- targets之间的联系：

  - [`target_link_libraries`](https://cmake.org/cmake/help/latest/command/target_link_libraries.html): Other targets; can also pass library names directly
  - [`target_include_directories`](https://cmake.org/cmake/help/latest/command/target_include_directories.html): Include directories
  - [`target_compile_features`](https://cmake.org/cmake/help/latest/command/target_compile_features.html): The compiler features you need activated, like `cxx_std_11`
  - [`target_compile_definitions`](https://cmake.org/cmake/help/latest/command/target_compile_definitions.html): Definitions
  - [`target_compile_options`](https://cmake.org/cmake/help/latest/command/target_compile_options.html): More general compile flags
  - [`target_link_directories`](https://cmake.org/cmake/help/latest/command/target_link_directories.html): Don’t use, give full paths instead (CMake 3.13+)
  - [`target_link_options`](https://cmake.org/cmake/help/latest/command/target_link_options.html): General link flags (CMake 3.13+)
  - [`target_sources`](https://cmake.org/cmake/help/latest/command/target_sources.html): Add source files

  See more [commands here](https://cmake.org/cmake/help/latest/manual/cmake-commands.7.html).

#### 2.2.7.3 一个例子

```cmake
cmake_minimum_required(VERSION 3.15...3.21)

project(MyExample01 LANGUAGES CXX)

# This is the library Including the headers is not required, but is nice for
# users
add_library(simple_lib simple_lib.cpp simple_lib.hpp)

# The above line *did not* set the includes - we need to We can also set ., and
# it should be expanded to the current source dir

target_include_directories(simple_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# Adding definitions
target_compile_definitions(simple_lib PUBLIC MYLIB_PUBLIC)
target_compile_definitions(simple_lib PRIVATE MYLIB_PRIVATE)

# Require a C++ feature (here: at least C++11)
target_compile_features(simple_lib PUBLIC cxx_std_11)

# Now add the executable
add_executable(simple_example simple_example.cpp)

# Adding the all-important link to simple-lib
target_link_libraries(simple_example PUBLIC simple_lib)
```



## 2.3 Property属性相关

### 2.3.1 给target添加属性

C++标准属性是最重要的。

- 使用命令`set_target_properties`来添加:[官网](https://cmake.org/cmake/help/latest/command/set_target_properties.html?highlight=set_target_properties)

  比如：

  ```cmake
  set_target_properties(myTarget PROPERTIES
      CXX_STANDARD 11					# 要求c++最低版本为c++11
      CXX_STANDARD_REQUIRED YES		# 让CMake需要c++版本控制，即遵守上面这条属性
      CXX_EXTENSIONS NO				# 可不写，但可以帮忙避免扩展extension被添加进来。比如没有这条命令，可能g++11会代替c++11而被加进来。
  )
  
  # 等价于
  
  set(CMAKE_CXX_STANDARD 11 CACHE STRING "The C++ standard to use")
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  ```

- C++可能用的属性汇总：https://cmake.org/cmake/help/latest/prop_gbl/CMAKE_CXX_KNOWN_FEATURES.html

### 2.3.2 有用的模块

CMake提供了大量有用的模块，见[官网](https://cmake.org/cmake/help/latest/manual/cmake-modules.7.html)。

使用模块前首先需要`include(模块名)`,常用的有：

#### 2.3.2.1 CMakeDependentOption

[CMakeDependentOption](https://cmake.org/cmake/help/latest/module/CMakeDependentOption.html)可以设依据其他变量的结果来设置一个选项，决定执行什么代码。

比如：

```cmake
include(CMakeDependentOption)
cmake_dependent_option(BUILD_TESTS "Build your tests" ON "VAL1;VAL2" OFF)\

# 等价于：

if(VAL1 AND VAL2)
    set(BUILD_TESTS_DEFAULT ON)
else()
    set(BUILD_TESTS_DEFAULT OFF)
endif()

option(BUILD_TESTS "Build your tests" ${BUILD_TESTS_DEFAULT})

if(NOT BUILD_TESTS_DEFAULT)
    mark_as_advanced(BUILD_TESTS)
endif()
```

#### 2.3.2.2 CMakePrintHelpers

[CMakePrintHelpers](https://cmake.org/cmake/help/latest/module/CMakePrintHelpers.html)提供两个函数帮忙打印我们想要的数据：

1. `cmake_print_properties` lets you easily print properties
2. `cmake_print_variables` will print the names and values of any variables you give it.

比如：

```cmake
cmake_print_properties(TARGETS foo bar PROPERTIES
                       LOCATION INTERFACE_INCLUDE_DIRECTORIES)
                       
cmake_print_variables(var1 var2 ..  varN)
```

#### 2.3.2.3 CheckCXXCompilerFlag

[CheckCXXCompilerFlag](https://cmake.org/cmake/help/latest/module/CheckCXXCompilerFlag.html)检查flag是否被支持。、

比如：

```cmake
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-someflag OUTPUT_VARIABLE)
```

- 注意：OUTPUT_VARIABLE也会出现在configuration的输出中。

#### 2.3.2.4 try_compile/try_run

[`try_compile`](https://cmake.org/cmake/help/latest/command/try_compile.html)/[`try_run`](https://cmake.org/cmake/help/latest/command/try_run.html)可以在configure time，就先编译或者运行一些代码，如此得到一些信息

基本使用：

```cmake
try_compile(
  RESULT_VAR
    bindir
  SOURCES
    source.cpp
)
```

#### 2.3.2.5 FeatureSummary

[FeatureSummary](https://cmake.org/cmake/help/latest/module/FeatureSummary.html)可以输出一系列我们正在找的包，有点相当于`find_package`

比如：

```cmake
include(FeatureSummary)

# 给包加属性
set_package_properties(OpenMP PROPERTIES
    URL "http://www.openmp.org"
    DESCRIPTION "Parallel compiler directives"
    PURPOSE "This is what it does in my package")
    
# 将option标记mark为属性的一部分
add_feature_info(WITH_OPENMP OpenMP_CXX_FOUND "OpenMP （Thread safe FCNs only)")

# 打印出所有的属性
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    feature_summary(WHAT ENABLED_FEATURES DISABLED_FEATURES PACKAGES_FOUND)
    feature_summary(FILENAME ${CMAKE_CURRENT_BINARY_DIR}/features.log WHAT ALL)
endif()
```



### 2.3.3 Debug

有3种方式来debug CMake文件。[额外参考](https://hsf-training.github.io/hsf-training-cmake-webpage/08-debugging/index.html)

#### 2.3.3.1 Printing variables

```cmake
# 1.  自带的方法message：
message(STATUS "MY_VARIABLE=${MY_VARIABLE}")

# 2.1 使用2.3.2.2中提到的模块CMakePrintHelpers
include(CMakePrintHelpers)
cmake_print_variables(MY_VARIABLE)
# 2.2 一口气打印多个属性
cmake_print_properties(
    TARGETS my_target
    PROPERTIES POSITION_INDEPENDENT_CODE
)
```



#### 2.3.3.2 Tracing a run

```
cmake -S . -B build --trace-source=CMakeLists.txt
```

- `--trace-source="filename"`:可以让我们在频幕上看到程序运行时都发生了什么



#### 2.3.3.3 Building in debug mode

- 对于single-configuration generators：

  可以在构建时，加上`-DCMAKE_BUILD_TYPE=Debug`来得到debugging flags

  比如：

  ```shell
  cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug
  cmake --build build-debug
  gdb build-debug/simple_example
  ```

  

- 对于 multi-configuration generators，比如ide:

  可以在ide中加入配置
