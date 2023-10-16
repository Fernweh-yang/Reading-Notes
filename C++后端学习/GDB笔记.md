# 0.学习资料汇总：

- 初始参考：斯坦福的[cs107](https://web.stanford.edu/class/archive/cs/cs107/cs107.1202/resources/gdb)
  - gdb命令快速参考：[gdb Reference Card](https://web.stanford.edu/class/archive/cs/cs107/cs107.1202/resources/gdb_refcard.pdf)
- GDB官方手册：[full gdb manual](http://www.gnu.org/software/gdb/)

# 1.基本使用

GNU Debugger 是一个命令行调试工具

- 设置gdb的preferences

  修改`~/.gdbinit`文件：

  ```shell
  # Explicitly tell gdb that we are working with 64-bit programs
  set architecture i386:x86-64
  
  # Tell gdb to read .gdbinit files in the same directory as the program you're debugging
  set auto-load safe-path /
  ```

  - gdb会查看`.gdbinit`文件来知道preference

## 1.1 编译文件

- 编译：为gdb添加调试信息：

  虽然没有调试信息，gdb也能跑可执行文件，但debugger就不太能有效的和源代码交互了。

  调试信息使得GDB能够与源代码之间建立关联，从而允许您在调试过程中查看变量的值、跟踪程序执行、设置断点等等。

  - `gcc`命令不会自动将**调试信息**放入可执行文件，需要额外加`-g -Og`

    ```shell
    gcc -g -Og -std=gnu99  -o hello helloWorld.c helloLanguages.c
    ```

  - Makefiles同理

    ```makefile
    # The default C compiler
    CC = gcc
    
    # The CFLAGS variable sets compile flags for gcc:
    #  -g          compile with debug information
    #  -Wall       give verbose compiler warnings
    #  -O0         do not optimize generated code生成与源代码几乎一模一样的机器代码
    #  -std=gnu99  use the GNU99 standard language definition
    CFLAGS = -g -Wall -O0 -std=gnu99
    
    hello: helloWorld.c helloLanguages.c hello.h
        $(CC) $(CFLAGS) -o hello helloWorld.c helloLanguages.c
    
    # .PHONY 是一个特殊的目标（target）声明
    # 主要作用是告诉Make工具，不管是否存在与其同名的文件，都要执行相应的操作，而不是仅仅认为文件是最新的并且不需要重新生成。
    .PHONY: clean	
    
    clean:
        rm -f hello *.o
    ```

  - CMakeLists同理

    ```cmake
    cmake_minimum_required(VERSION 3.0)
    project(HelloWorld)
    
    # Set the source files
    set(SOURCES
        helloWorld.c
        helloLanguages.c
        hello.h
    )
    
    # Set the compiler and its flags
    set(CMAKE_C_COMPILER gcc)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall -O0 -std=gnu99")
    
    # Add an executable target
    add_executable(hello ${SOURCES})
    
    # Define a custom target for cleaning
    # clean_hello是自定义目标，所以cmake --build . 或 make 时不会自动构建 clean_hello
    # 需要明确执行 make clean_hello 来触发它。
    add_custom_target(clean_hello
        COMMAND ${CMAKE_COMMAND} -E remove hello	# 名为hello的可执行文件
        COMMAND ${CMAKE_COMMAND} -E remove ${SOURCES}	# 源文件
    )
    
    # Add 'clean' target
    # clean默认目标，cmake --build . 或 make 时，它会自动构建 clean。
    add_custom_target(clean
        COMMAND ${CMAKE_COMMAND} -E remove hello
        COMMAND ${CMAKE_COMMAND} -E remove ${SOURCES}
    )
    
    # Make the 'clean' target depend on 'clean_hello'
    add_dependencies(clean clean_hello)
    ```

  

  ## 1.2 运行gdb

  假设如上面编译完的可执行文件为：hello

  - 进入gdb：

    ```shell
    # 进入指定可执行文件的gdb调试页面，此时程序没运行，gdb在等待额外的指令
    gdb hello
    # 运行可执行文件
    run hello
    ```

    

