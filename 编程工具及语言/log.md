# [spdlog](https://github.com/gabime/spdlog)

根据[博文](https://blog.csdn.net/qq_21438461/article/details/134098605)对比，spdlog比谷歌的glog更快更节省资源。

## 安装

1. 方法一：header-only 

   直接将[整个包](https://github.com/gabime/spdlog/tree/v1.x/include/spdlog)放入工程目录，然后用c++11标准来编译。

2. 方法二：pre-compiled

   安装在工程目录下，如`/project/3rd_lib/`

   ```
   git clone https://github.com/gabime/spdlog.git
   cd spdlog && mkdir build && cd build
   cmake .. && make -j
   ```

## cmakelist

```cmake
cmake_minimum_required(VERSION 2.8)
project(test)

// 包含spdlog的头文件
INCLUDE_DIRECTORIES(/home/hx/3rdLib/spdlog/include)
// 包含spdlog的动态库目录
LINK_DIRECTORIES(/home/hx/3rdLib/spdlog/build)

add_executable(test src/test.cpp)

// 链接spdlog动态库
target_link_libraries(test PRIVATE spdlog)
```



## 使用例子：

### 1. 基本使用

```c++
#include "spdlog/spdlog.h"

int main() 
{	
    // 不同的log信息登记
    spdlog::info("Welcome to spdlog!");
    spdlog::error("Some error message with arg: {}", 1);
    
    spdlog::warn("Easy padding in numbers like {:08d}", 12);
    spdlog::critical("Support for int: {0:d};  hex: {0:x};  oct: {0:o}; bin: {0:b}", 42);
    spdlog::info("Support for floats {:03.2f}", 1.23456);
    spdlog::info("Positional args are {1} {0}..", "too", "supported");
    spdlog::info("{:<30}", "left aligned");
    
    spdlog::set_level(spdlog::level::debug); // Set global log level to debug
    spdlog::debug("This message should be displayed..");    
    
    // change log pattern
    // %s：文件名
    // %#：行号
    // %!：函数名
    spdlog::set_pattern("[%H:%M:%S %z] [%n] [%^---%L---%$] [thread %t] %v");
    
    // Compile time log levels
    // Note that this does not change the current log level, it will only
    // remove (depending on SPDLOG_ACTIVE_LEVEL) the call on the release code.
    SPDLOG_TRACE("Some trace message with param {}", 42);
    SPDLOG_DEBUG("Some debug message");
}

```



### 2. 写日志到文件

```c++
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

int main() 
{	
    // 创建一个基本文件记录器
    auto file_logger = spdlog::basic_logger_mt("basic_logger", "logs/basic-log.txt");
	
    // 设置日志记录级别
    file_logger->set_level(spdlog::level::info);
    
    // 日志记录
    file_logger->info("This is an info message");
    file_logger->warn("This is a warning message");
    file_logger->error("This is an error message");
	
    // 刷新日志文件以确保所有日志都写入磁盘
    file_logger->flush();
    return 0;
}
```



### 3. 写日志到控制台

```c++
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

int main() {
    // 创建一个彩色控制台记录器
    auto console_logger = spdlog::stdout_color_mt("console");

    // 设置日志记录级别（可选）
    console_logger->set_level(spdlog::level::info);

    // 日志记录
    console_logger->info("This is an info message");
    console_logger->warn("This is a warning message");
    console_logger->error("This is an error message");

    return 0;
}
```

### 4. 写日志到旋转文件

设置保留的文件数量，以防止占用过多磁盘空间。

```c++
#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>

int main() {
    // 创建一个旋转文件记录器，文件大小限制为5MB，保留3个文件
    auto rotating_logger = spdlog::rotating_logger_mt("rotating_logger", "logs/my_rotating_log.txt", 5 * 1024 * 1024, 3);

    // 设置日志记录级别（可选）
    rotating_logger->set_level(spdlog::level::info);

    // 日志记录
    rotating_logger->info("This is an info message in rotating logger");

    // 刷新日志文件
    rotating_logger->flush();

    return 0;
}
```

### 5. 写日志到按天文件

每天创建一个新的日志文件，以日期命名。这种方式适合需要按天记录日志的应用程序。

```c++
#include <spdlog/spdlog.h>
#include <spdlog/sinks/daily_file_sink.h>

int main() {
    // 创建一个每日文件记录器，日志文件名格式为 "logs/my_daily_log.txt"
    // 自动生成的文件名字是：my_daily_log_20xx-xx-xx.txt
    auto daily_logger = spdlog::daily_logger_mt("daily_logger", "logs/my_daily_log.txt", 0, 0);

    // 设置日志记录级别（可选）
    daily_logger->set_level(spdlog::level::info);

    // 日志记录
    daily_logger->info("This is an info message in daily logger");

    // 刷新日志文件
    daily_logger->flush();

    return 0;
}
```

### 6. 定期刷新日志

适用于需要定期刷新日志以确保日志数据不会长时间保留在缓冲区中的场景。这可以防止在程序异常终止时丢失日志数据。

```c++
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

int main() {
    // 创建一个文件记录器
    auto file_logger = spdlog::basic_logger_mt("file_logger", "logs/my_log.txt");

    // 设置日志记录级别（可选）
    file_logger->set_level(spdlog::level::info);

    // 配置每3秒刷新一次所有日志记录器
    spdlog::flush_every(std::chrono::seconds(3));

    // 日志记录
    file_logger->info("This is an info message");
    file_logger->warn("This is a warning message");
    file_logger->error("This is an error message");

    // 模拟一些工作
    std::this_thread::sleep_for(std::chrono::seconds(10));

    return 0;
}

```

