# 参考教程

Python Tutorial:https://docs.opencv.org/4.6.0/d6/d00/tutorial_py_root.html

C++ Tutorial:https://docs.opencv.org/4.7.0/d9/df8/tutorial_root.html

- 配套的代码：https://github.com/opencv/opencv/tree/4.x/samples/cpp/tutorial_code

# Installtion

## 1. 安装

- Install for c++: https://docs.opencv.org/4.1.2/d7/d9f/tutorial_linux_install.html

  reference: https://www.ncnynl.com/archives/201912/3566.html

  reference: https://immortalqx.github.io/2021/07/06/opencv-notes-0/

  1. 从https://opencv.org/releases/下载最新版本代码源文件

     - 如果要使用一些前言的还不稳定的算法，需要下载opencv_contrib:

       https://github.com/opencv/opencv_contrib/tags

  2. 解压到我们希望安装的目录下

     ```
     # 比如NUC12电脑中放在/home/yang/opencv/中
     mv ./opencv-4.7.0 /home/yang/opencv
     mv ./opencv_contrib-4.7.0 /home/yang/opencv
     ```

  4. 编译

     ```
     $ cd ~/opencv/opencv-4.7.0 && mkdir build && cd build
     $ cmake -D CMAKE_BUILD_TYPE=Release -D OPENCV_GENERATE_PKGCONFIG=YES -D CMAKE_INSTALL_PREFIX=/usr/local/opencv4.7.0 ..
     $ make -j6 #用6个线程去编译
     $ sudo make install
     ```

     - 修改路径为/usr/local/opencv4.7.0

       否则默认各部分分别安装在`/usr/local/`目录的`include`，`bin`，`lib`3个文件夹下。
  
     - `-D OPENCV_GENERATE_PKGCONFIG=YES`开启支持使用`pkg-config`功能
  
       - 就可以用`pkg-config --libs opencv4`来查看
  
  5. 设置pkg-config
  
     将/usr/local/opencv4.7.0/lib/pkgconfig/加到PKG_CONFIG_PATH中去
  
     ```
     sudo vim /etc/profile.d/pkgconfig.sh
     ```
  
     填入：`export PKG_CONFIG_PATH=/usr/local/opencv4.7.0/lib/pkgconfig:$PKG_CONFIG_PATH`
  
     激活配置：
  
     ```
     source /etc/profile
     ```
  
  6. 动态库环境
  
     创建一个动态库环境配置文件:opencv.conf
  
     ```
     sudo vim /etc/ld.so.conf.d/opencv.conf
     ```
  
     填入:`/usr/local/opencv4.7.0/lib`
  
     生效配置：
  
     ```
     sudo ldconfig
     ```
  
  7. 修改bashrc
  
     ```
     gedit ~/.bashrc
     # 填入下面2行：
     	export PKG_CONFIG_PATH=/usr/local/opencv4.7.0/lib/pkgconfig
     	export LD_LIBRARY_PATH=/usr/local/opencv4.7.0/lib
     source ~/.bashrc
     ```
  
- Install for python: 

  在conda某一环境下：
  
  ```
  pip3 install opencv-python
  ```


## 2. 查看安装位置

- `sudo find / -iname "*opencv*"`

## 3. 检查版本

- c++:

  ````
  opencv_version
  pkg-config --modversion opencv //如果上面安装使用了pkg-config选项
  ````

- python:

  ```
  import cv2 as cv
  print(cv.__version__)
  ```

  

## 4. 切换OpenCV版本

- 在.bashrc中修改

  ```
  #OpenCV_4.2.0
  #export PKG_CONFIG_PATH=/usr/local/opencv_4.2.0/lib/pkgconfig
  #export LD_LIBRARY_PATH=/usr/local/opencv_4.2.0/lib
  
  #OpenCV_3.4.6
  #export PKG_CONFIG_PATH=/usr/local/opencv_3.4.6/lib/pkgconfig
  #export LD_LIBRARY_PATH=/usr/local/opencv_3.4.6/lib
  
  #OpenCV_2.4.9
  export PKG_CONFIG_PATH=/usr/local/opencv_2.4.9/lib/pkgconfig
  export LD_LIBRARY_PATH=/usr/local/opencv_2.4.9/lib
  ```

- 注意安装cmake时放在不同目录下

# Opencv-Python

## 1. Gui Features

### 1.1 Image read and write

#### 1.1.1 Functions to learn:

- `cv::imread`:Read an image from file 
- `cv::imshow`:Display an image in an OpenCV window 
- `cv::imwrite`:Write an image to a file

#### 1.1.2 C++ Implementation

- code: 
```C++
    #include <opencv2/core.hpp>			//opencv库的基本模块
    #include <opencv2/imgcodecs.hpp>	//opencv库提供reading and writing功能的模块
    #include <opencv2/highgui.hpp>		//opencv库提供在一个窗口中显示一个图片的功能
    #include <iostream>
    int main()
    {	
        // 1.使用core模块里的samples::findFile()得到地址
        std::string image_path = cv::samples::findFile("starry_night.jpg");
        // 2.使用imgcodecs模块里的imread()读取图片，并保存在core模块定义的cv::Mat object里
        Mat img = cv::imread(image_path, IMREAD_COLOR);
        if(img.empty())
        {
            std::cout << "Could not read the image: " << image_path << std::endl;
            return 1;
        }
        // 3.使用highgui模块的imshow()来显示图片
        imshow("Display window", img);
        // 使用highgui模块的waitkey()来等待一个输入指令来结束图片显示
        int k = waitKey(0); // 0表示永远等待，单位是millisecond毫秒
        if(k == 's')
        {	
            // 4.使用imgcodecs模块的imwrite()来保存图片
            imwrite("starry_night.png", img);
        }
        return 0;
    }
```

- compile:

  write CMakeLists.txt：

  ```cmake
  cmake_minimum_required(VERSION 2.8)
  project( test )
  find_package( OpenCV REQUIRED )
  include_directories( ${OpenCV_INCLUDE_DIRS} )
  add_executable( test test.cpp )
  target_link_libraries( test ${OpenCV_LIBS} )
  ```

  compile:

  ```
  mkdir build
  cmake ..
  cmake --build .
  ```

​		run: `./test`

#### 1.1.3 Python Implementation

The functions' names are same as them in C++.

```python
import cv2 as cv
import sys
# 1.
img = cv.imread(cv.samples.findFile("starry_night.jpg"))
if img is None:
    sys.exit("Could not read the image.")
# 2.
cv.imshow("Display window", img)
k = cv.waitKey(0)
if k == ord("s"):
    # 3.
    cv.imwrite("starry_night.png", img)
```

1.`waitKey(0)` will display the window infinitely until any keypress (it is suitable for image display).显示静止图像

2.`waitKey(1)` will display a frame for `1` ms, after which display will be automatically closed. Since the OS has a minimum time between switching threads, the function will not wait exactly `1` ms, it will wait at least `1` ms, depending on what else is running on your computer at that time.显示连续的运动图像
- 通常用法是：
```python
if cv.waitKey(1) == ord('q'):
	break
```

### 1.2 Video read and save

#### 1.2.1 Functions to learn

- **[cv.VideoCapture()](https://docs.opencv.org/4.6.0/d8/dfe/classcv_1_1VideoCapture.html)**,

- **[cv.VideoWriter()](https://docs.opencv.org/4.6.0/dd/d9e/classcv_1_1VideoWriter.html)**

#### 1.2.2 Capture and play Video 

```python
import numpy as np
import cv2 as cv
# 1.创建一个VideoCapture对象object来获得相机图像, 参数0是相机device index
# linux中相机编号可在/dev 中查询video,偶数位代表相机编号
cap = cv.VideoCapture(0)
# 如果是关闭的也可以用cap.open()来打开
if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    # 2.Capture frame-by-frame。并且如果读取成果cap.read()会返回True的Bool值
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # 3.Display the resulting frame
    cv.imshow('frame', gray)
    # waitKey()的参数是等待几毫秒。不能太大否则视频会卡顿，通常25ms以下都可以
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
```

#### 1.2.3 Save a video

```python
import numpy as np
import cv2 as cv
cap = cv.VideoCapture(0)
# 1.Define the codec and create VideoWriter object
# FourCC用于指定video codec视频编解码器，比如XVID,MJPG,WMV，DIVX等
fourcc = cv.VideoWriter_fourcc(*'XVID')
# 2.VideoWriter类：https://docs.opencv.org/3.4/dd/d9e/classcv_1_1VideoWriter.html
# 参数1:输出视频名字2:编码器3:frames per second(fps)4:framesize
out = cv.VideoWriter('output.avi', fourcc, 20.0, (640,  480))
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    frame = cv.flip(frame, 0)
    # 3.write the flipped frame。保存1帧。
    out.write(frame)
    cv.imshow('frame', frame)
    if cv.waitKey(1) == ord('q'):
        break
# Release everything if job is finished
cap.release()
out.release()
cv.destroyAllWindows()
```



### 1.3 Drawing Function

#### 1.3.1 Functions to learn

draw different geometric shapes

- **[cv.line()](https://docs.opencv.org/4.6.0/d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2)**,

- **[cv.circle()](https://docs.opencv.org/4.6.0/d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670)** ,

- **[cv.rectangle()](https://docs.opencv.org/4.6.0/d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9)**,

- **[cv.ellipse()](https://docs.opencv.org/4.6.0/d6/d6e/group__imgproc__draw.html#ga28b2267d35786f5f890ca167236cbc69)**, 

- **[cv.putText()](https://docs.opencv.org/4.6.0/d6/d6e/group__imgproc__draw.html#ga5126f47f883d730f633d74f07456c576)** 
- **[cv.polylines()](https://docs.opencv.org/4.6.0/d6/d6e/group__imgproc__draw.html#gaa3c25f9fb764b6bef791bf034f6e26f5)**
- 他们的共同实参Arguments:
  - **img** : The image where you want to draw the shapes
  - **color** : Color of the shape. for BGR, pass it as a tuple, eg: (255,0,0) for blue. For grayscale, just pass the scalar value.
  - **thickness** : Thickness of the line or circle etc. If **-1** is passed for closed figures like circles, it will fill the shape. *default thickness = 1*
  - **lineType** : Type of line, whether 8-connected, anti-aliased line etc. *By default, it is 8-connected.* [cv.LINE_AA](https://docs.opencv.org/4.6.0/d6/d6e/group__imgproc__draw.html#ggaf076ef45de481ac96e0ab3dc2c29a777a85fdabe5335c9e6656563dfd7c94fb4f) gives anti-aliased line抗锯齿线 which looks great for curves.

#### 1.3.2 Code

```python
import numpy as np
import cv2 as cv
# Create a black image
img = np.zeros((512,512,3), np.uint8)
# 1.线段：参数2和3分别是线段的2个端点。参数4是颜色，参数5是粗细
cv.line(img,(0,0),(511,511),(255,0,0),5)
# 2.矩形：参数2和3分别是矩形的左上/右下顶点
cv.rectangle(img,(384,0),(510,128),(0,255,0),3)
# 3.圆：参数2和3分别是圆心和半径
cv.circle(img,(447,63), 63, (0,0,255), -1)
# 4.椭圆Ellipse：参数2:圆心3:轴长4:椭圆顺时针旋转的角度5:椭圆弧开始角度6:椭圆弧结束角度7:颜色8:粗细
# 参数5和6决定了椭圆画的完不完整，如下就只画了上半椭圆，如果180改称360就是全部椭圆。
# 参数8传了-1表示填充整个图形。
cv.ellipse(img,(256,256),(100,50),0,0,180,255,-1)
# 5.多边形Polygon:需要传入多边形的所有顶点vertex,这些顶点以数组array的形式传入
# 第三个参数true表示这个多边形是闭合的，所以第一个和最后一个vertex会连起来。
pts = np.array([[10,5],[20,30],[70,20],[50,10]], np.int32)
pts = pts.reshape((-1,1,2))
cv.polylines(img,[pts],True,(0,255,255))
# 6.文字：参数2是要输入的文字
font = cv.FONT_HERSHEY_SIMPLEX
# 参数3是开始的坐标，4字体，5颜色，6粗细，7线的类型
cv.putText(img,'OpenCV',(10,500), font, 4,(255,255,255),2,cv.LINE_AA)
cv.imshow("Display window", img)
k = cv.waitKey(0)
```

### 1.3 Mouse as Paint-Brush

#### 1.3.1 Functions to learn

- **[cv.setMouseCallback()](https://docs.opencv.org/4.6.0/d7/dfc/group__highgui.html#ga89e7806b0a616f6f1d502bd8c183ad3e)**

- Callback function will be executed when a mouse event take place. Check which events are available:

  in python terminal:

  ```
  import cv2 as cv
  events = [i for i in dir(cv) if 'EVENT' in i]
  print( events )
  ```

   

#### 1.3.2 Klick->Draw Circle

```python
import numpy as np
import cv2 as cv
# 1.设定一个画圈圈的回调函数
def draw_circle(event,x,y,flags,param):
    if event == cv.EVENT_LBUTTONDBLCLK:
        cv.circle(img,(x,y),100,(255,0,0),-1)
        
# 2.创建一个窗口
img = np.zeros((512,512,3), np.uint8)
cv.namedWindow('image')
# 3.将回调函数绑定到这个窗口，在这个窗口中发生事件就会调用回调函数
cv.setMouseCallback('image',draw_circle)
while(1):
    cv.imshow('image',img)
    if cv.waitKey(20) & 0xFF == 27:
        break
cv.destroyAllWindows()
```

#### 1.3.3 Klick->Draw different thing

```python
import numpy as np
import cv2 as cv
drawing = False # true if mouse is pressed
mode = True # if True, draw rectangle. Press 'm' to toggle to curve
ix,iy = -1,-1
# 1.创建回调函数，不同的事件作不同的事
def draw_circle(event,x,y,flags,param):
    global ix,iy,drawing,mode
    # 1.1左键按下获得鼠标的坐标
    if event == cv.EVENT_LBUTTONDOWN:
        drawing = True
        ix,iy = x,y
    # 1.2鼠标按住移动画矩形/圆，按下m来改变模式
    elif event == cv.EVENT_MOUSEMOVE:
        if drawing == True:
            if mode == True:
                cv.rectangle(img,(ix,iy),(x,y),(0,255,0),-1)
            else:
                cv.circle(img,(x,y),5,(0,0,255),-1)
    # 1.3鼠标提起画下最后一点
    elif event == cv.EVENT_LBUTTONUP:
        drawing = False
        if mode == True:
            cv.rectangle(img,(ix,iy),(x,y),(0,255,0),-1)
        else:
            cv.circle(img,(x,y),5,(0,0,255),-1)
# 2.创建窗口
img = np.zeros((512,512,3), np.uint8)
cv.namedWindow('image')
# 3.将回调函数和窗口绑定
cv.setMouseCallback('image',draw_circle)
while(1):
    cv.imshow('image',img)
    k = cv.waitKey(1) & 0xFF
    if k == ord('m'):
        mode = not mode
    elif k == 27:
        break
cv.destroyAllWindows()
```

### 1.4 Trackbar as Color Palette

轨迹条作为调色板

#### 1.4.1 Functions to learn

- **[cv.getTrackbarPos()](https://docs.opencv.org/4.6.0/d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8)**, 
- **[cv.createTrackbar()](https://docs.opencv.org/4.6.0/d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b)** 

#### 1.4.2 Code

```python
import numpy as np
import cv2 as cv
def nothing(x):
    pass
# 1.创建一个窗口
img = np.zeros((300,512,3), np.uint8)
cv.namedWindow('image')
# 2.创建三个轨迹条
# 参数1轨迹条名字，2依附的窗口名，3初始值，4最大值，5回调函数
cv.createTrackbar('R','image',0,255,nothing)
cv.createTrackbar('G','image',0,255,nothing)
cv.createTrackbar('B','image',0,255,nothing)
# 3.create switch for ON/OFF functionality
# 因为openCV中无按钮，所以可以用轨迹条代替作为按钮选择的功能
switch = '0 : OFF \n1 : ON'
cv.createTrackbar(switch, 'image',0,1,nothing)
while(1):
    cv.imshow('image',img)
    k = cv.waitKey(1) & 0xFF
    if k == 27:
        break
    # 4.get current positions of four trackbars
    r = cv.getTrackbarPos('R','image')
    g = cv.getTrackbarPos('G','image')
    b = cv.getTrackbarPos('B','image')
    s = cv.getTrackbarPos(switch,'image')
    # 5.只有按钮选择为1时才会变色
    if s == 0:
        img[:] = 0
    else:
        img[:] = [b,g,r]
cv.destroyAllWindows()
```



## 2. Core Operation

### 2.1 Basic operations on Images

#### 2.1.1 Goal

- Access pixel values and modify them
- Access image properties
- Set a Region of Interest (ROI)
- Split and merge images
  - **[cv.split](https://docs.opencv.org/4.6.0/d2/de8/group__core__array.html#ga8027f9deee1e42716be8039e5863fbd9)**
  - **[cv.merge](https://docs.opencv.org/4.6.0/d2/de8/group__core__array.html#ga61f2f2bde4a0a0154b2333ea504fab1d)**
- Making Borders for Images (Padding)
  -  **[cv.copyMakeBorder()](https://docs.opencv.org/4.6.0/d2/de8/group__core__array.html#ga2ac1049c2c3dd25c2b41bffe17658a36)**
    - **src** - input image
    - **top**, **bottom**, **left**, **right** - border width in number of pixels in corresponding directions
    - **borderType**- Flag defining what kind of border to be added. It can be following types:
      - **[cv.BORDER_CONSTANT](https://docs.opencv.org/4.6.0/d2/de8/group__core__array.html#gga209f2f4869e304c82d07739337eae7c5aed2e4346047e265c8c5a6d0276dcd838)** - Adds a constant colored border. The value should be given as next argument.
      - **[cv.BORDER_REFLECT](https://docs.opencv.org/4.6.0/d2/de8/group__core__array.html#gga209f2f4869e304c82d07739337eae7c5a815c8a89b7cb206dcba14d11b7560f4b)** - Border will be mirror reflection of the border elements, like this : *fedcba|abcdefgh|hgfedcb*
      - **[cv.BORDER_REFLECT_101](https://docs.opencv.org/4.6.0/d2/de8/group__core__array.html#gga209f2f4869e304c82d07739337eae7c5ab3c5a6143d8120b95005fa7105a10bb4)** or **[cv.BORDER_DEFAULT](https://docs.opencv.org/4.6.0/d2/de8/group__core__array.html#gga209f2f4869e304c82d07739337eae7c5afe14c13a4ea8b8e3b3ef399013dbae01)** - Same as above, but with a slight change, like this : *gfedcb|abcdefgh|gfedcba*
      - **[cv.BORDER_REPLICATE](https://docs.opencv.org/4.6.0/d2/de8/group__core__array.html#gga209f2f4869e304c82d07739337eae7c5aa1de4cff95e3377d6d0cbe7569bd4e9f)** - Last element is replicated throughout, like this: *aaaaaa|abcdefgh|hhhhhhh*
      - **[cv.BORDER_WRAP](https://docs.opencv.org/4.6.0/d2/de8/group__core__array.html#gga209f2f4869e304c82d07739337eae7c5a697c1b011884a7c2bdc0e5caf7955661)** - Can't explain, it will look like this : *cdefgh|abcdefgh|abcdefg*
    - **value** - Color of border if border type is [cv.BORDER_CONSTANT](https://docs.opencv.org/4.6.0/d2/de8/group__core__array.html#gga209f2f4869e304c82d07739337eae7c5aed2e4346047e265c8c5a6d0276dcd838)

#### 2.1.2 Code

```python
import numpy as np
import cv2 as cv
img = cv.imread('messi5.jpg')
# 1.Access pixel values and modify them
## 	1.1用row and column coordinates来获得图片的像素值，BGR返回三通道值，grayscale返回intensity.
px = img[100,100]		#返回[137 117 112]
print( px )		
blue = img[100,100,0]	#返回137，通过数组index返回特定通道的值
print(blue)		
img[100,100] = [255,255,255] 	#设置图片的像素值为255，255，255
## 	1.2用函数（更推荐）
img.item(10,10,2)	#获得图片(10,10)处的红色像素值
img.itemset((10,10,2),100)	#设置图片(10,10)处的红色像素值为100

# 2.Access image properties
print( img.shape )	#shape of an image 
print( img.size )	#Total number of pixels 
print( img.dtype )	#Image datatype 

# 3.Image ROI
ball = img[280:340, 330:390]	#划出一块区域为球
img[273:333, 100:160] = ball	#把另一块区域替换为球

# 4.Split and merge images
b,g,r = cv.split(img)	#将一个多通道的数组，划分成多个单通道数组
img = cv.merge((b,g,r))	#将多个单通道数组合并为一个多通道数组

# 5.Making Borders for Images (Padding)
from matplotlib import pyplot as plt
BLUE = [255,0,0]
img1 = cv.imread('opencv-logo.png')
## copyMakeBorder()的各个参数作用见上面
replicate = cv.copyMakeBorder(img1,10,10,10,10,cv.BORDER_REPLICATE)
reflect = cv.copyMakeBorder(img1,10,10,10,10,cv.BORDER_REFLECT)
reflect101 = cv.copyMakeBorder(img1,10,10,10,10,cv.BORDER_REFLECT_101)
wrap = cv.copyMakeBorder(img1,10,10,10,10,cv.BORDER_WRAP)
constant= cv.copyMakeBorder(img1,10,10,10,10,cv.BORDER_CONSTANT,value=BLUE)
plt.subplot(231),plt.imshow(img1,'gray'),plt.title('ORIGINAL')
plt.subplot(232),plt.imshow(replicate,'gray'),plt.title('REPLICATE')
plt.subplot(233),plt.imshow(reflect,'gray'),plt.title('REFLECT')
plt.subplot(234),plt.imshow(reflect101,'gray'),plt.title('REFLECT_101')
plt.subplot(235),plt.imshow(wrap,'gray'),plt.title('WRAP')
plt.subplot(236),plt.imshow(constant,'gray'),plt.title('CONSTANT')
plt.show()
```



### 2.2 Arithmetic Operations on Images

#### 2.2.1 Goal

Arithmetic operations:

- addition,subtraction, bitwise operations,
  - **[cv.add()](https://docs.opencv.org/4.6.0/d2/de8/group__core__array.html#ga10ac1bfb180e2cfda1701d06c24fdbd6)**
  - **[cv.addWeighted()](https://docs.opencv.org/4.6.0/d2/de8/group__core__array.html#gafafb2513349db3bcff51f54ee5592a19)**

#### 2.2.2 Code

```python
# 1.addition
>>> x = np.uint8([250])
>>> y = np.uint8([10])
# 相比于Numpy的加法是modulo operation模运算，cv的加法是saturated operation饱和运算
>>> print( cv.add(x,y) ) # 250+10 = 260 => 255
[[255]]
>>> print( x+y )          # 250+10 = 260 % 256 = 4
[4]

# 2. Image Blending混合
# 即分别给2张照片1个权重，让他们叠加起来
img1 = cv.imread('ml.png')
img2 = cv.imread('opencv-logo.png')
dst = cv.addWeighted(img1,0.7,img2,0.3,0)	#这里给了img1 0.7的权重，img2 0.3的权重，第三个参数是一个添加给每个加法的标量
cv.imshow('dst',dst)
cv.waitKey(0)
cv.destroyAllWindows()

# 3. Bitwise Operations
## Image Blending可以实现2个图片和起来，互相透明transparent的效果。
## 使用2.1里的划定ROI可以实现不透明opaque的叠加2个矩形图片。
## 而使用Bitwise Operations可以实现无规则图形不透明叠加
#### 整个过程是:将logo的黑白BGR图转换为灰度图，并将强度大于10的即非黑色的像素点全都转换成白色后的图案作为mask。如此得到的mask:logo处都是255的白色点，非logo处都是0的黑色点。然后背景图与运算用mask_inv作mask，logo出都是0的黑点不会进行与运算，所以背景图会出现一个黑色logo，这里的黑色是虚无的相当于一个缺口。然后logo与运算用mask作mask，logo处都是255的白点会进行与运算，于是logo图只剩下了白色的logo周围都被没被提取。最后将与运算后的背景图和logo图合二为一。
# Load two images
img1 = cv.imread('messi5.jpg')
img2 = cv.imread('opencv-logo-white.png')
# I want to put logo on top-left corner, So I create a ROI
rows,cols,channels = img2.shape
roi = img1[0:rows, 0:cols]
# Now create a mask of logo and create its inverse mask also
img2gray = cv.cvtColor(img2,cv.COLOR_BGR2GRAY)	#转灰度图像
ret, mask = cv.threshold(img2gray, 10, 255, cv.THRESH_BINARY)	#给每个矩阵元素添加1个阈值，返回值mask是目标图像，ret是True/False代表是否成功，10是阈值，255是最大值，超过了阈值根据cv.THRESH_BINARY转换
mask_inv = cv.bitwise_not(mask)		# 非运算：转换数组的每一个元素
# Now black-out the area of logo in ROI
img1_bg = cv.bitwise_and(roi,roi,mask = mask_inv)	#如果mask不为0，会做与运算
# Take only region of logo from logo image.
img2_fg = cv.bitwise_and(img2,img2,mask = mask)
# Put logo in ROI and modify the main image
dst = cv.add(img1_bg,img2_fg)
img1[0:rows, 0:cols ] = dst
cv.imshow('res',img1)
cv.waitKey(0)
cv.destroyAllWindows()
```

### 2.3 Performance Measurement and Improvement Techniques

#### 2.3.1 Goal

- Measure and improve the performance of the code
  -  **[cv.getTickCount](https://docs.opencv.org/4.6.0/db/de0/group__core__utils.html#gae73f58000611a1af25dd36d496bf4487)**: returns the number of clock-cycles after a reference event (like the moment the machine was switched ON) to the moment this function is called. 
  - **[cv.getTickFrequency](https://docs.opencv.org/4.6.0/db/de0/group__core__utils.html#ga705441a9ef01f47acdc55d87fbe5090c)**: returns the frequency of clock-cycles, or the number of clock-cycles per second. 

#### 2.3.2 Code

```python
import numpy as np
import cv2 as cv
img1 = cv.imread('test.jpg')
# 1.得到当前时钟周期
e1 = cv.getTickCount()
# 2.要测试性能的代码
for i in range(5,49,2):
    img1 = cv.medianBlur(img1,i)
# 3.得到现在的时钟周期
e2 = cv.getTickCount()
# 4.时间周期 除以 时间周期/秒 = 秒
t = (e2 - e1)/cv.getTickFrequency()
print( t )
```

- 也可以用Python的time.time()函数来实现相同的功能

#### 2.3.3 Optimization

- OpenCV函数通常支持SSE2,AVX。编译时会默认使用他们来优化

  ```python
  # check if optimization is enabled
  In [5]: cv.useOptimized()	#检查是否开启了优化
  Out[5]: True
  In [6]: %timeit res = cv.medianBlur(img,49)
  10 loops, best of 3: 34.9 ms per loop
  # Disable it
  In [7]: cv.setUseOptimized(False)	#关闭或打开优化
  In [8]: cv.useOptimized()
  Out[8]: False
  In [9]: %timeit res = cv.medianBlur(img,49)
  10 loops, best of 3: 64.1 ms per loop
  ```

- python中的优化技巧：

  - [Python Optimization Techniques](https://wiki.python.org/moin/PythonSpeed/PerformanceTips)
  - Scipy Lecture Notes - [Advanced Numpy](https://scipy-lectures.github.io/advanced/advanced_numpy/index.html#advanced-numpy)
  - [Timing and Profiling in IPython](http://pynash.org/2013/03/06/timing-and-profiling/)

## 3. Image Processing in OpenCV
[Image Processing in OpenCV](https://docs.opencv.org/4.6.0/d2/d96/tutorial_py_table_of_contents_imgproc.html)

## 4. Feature Detection and Description
[Feature Detection and Description](https://docs.opencv.org/4.6.0/db/d27/tutorial_py_table_of_contents_feature2d.html)

### 4.1 Harris Corner Detection
[Harris Corner Detection](https://docs.opencv.org/4.6.0/dc/d0d/tutorial_py_features_harris.html)

#### 4.1.1 Functions to learn

- **[cv.cornerHarris()](https://docs.opencv.org/4.6.0/dd/d1a/group__imgproc__feature.html#gac1fc3598018010880e370e2f709b4345)**:Harris Corner Detector 
- **[cv.cornerSubPix()](https://docs.opencv.org/4.6.0/dd/d1a/group__imgproc__feature.html#ga354e0d7c86d0d9da75de9b9701a9a87e)**:further refines the corners detected with sub-pixel accuracy. 更精确的找到角点的坐标

#### 4.1.2 Theory

- 哈里斯角点检测的基本思想：

  算法基本思想是使用一个固定窗口在图像上进行任意方向上的滑动，比较滑动前与滑动后两种情况，窗口中的像素灰度变化程度，如果存在任意方向上的滑动，都有着较大灰度变化，那么我们可以认为该窗口中存在角点。

- Change in the image segment段：当窗口发生u移动时，那么滑动前与滑动后对应的窗口中的像素点灰度变化描述如下：

  $$
  S(u)=\displaystyle\int_W(I(x+u)-I(x))^2dx\\
  也开写成：
  \\
  S(u_1,u_2)=\displaystyle\sum_{x,y}w(x,y)[I(x+u_1,y+u_2)-I(x,y)]^2
  $$

  - 其中x,y是像素点，u1u2是任意方法向的移动
  - **W** :window function
  - **I** :Intensity

- 要找变化最大的方向，即要让上式最大化，应用泰勒展开后可得：
  $$
  S(u_1,u_2)\approx[u_1\ u_2]M\left[\begin{matrix}u_1\\u_2\end{matrix}\right]
  $$

  - 其中M称之为哈里斯矩阵Harris Matrix
    $$
    M=\sum_{x,y}w(x,y) \left[
     \begin{matrix}
       I_{x}I_x & I_{x}I_{y} \\
       I_{x}I_{y} & I_{y}I_y  \\
      \end{matrix}
      \right]
    $$

    - Ix and Iy are image derivatives in x and y directions respectively. (These can be easily found using **[cv.Sobel()](https://docs.opencv.org/4.6.0/d4/d86/group__imgproc__filter.html#gacea54f142e81b6758cb6f375ce782c8d)**).
    - Sobel索伯算子可以计算灰度图像在x,y方向上的导数

- 评判是角点/边缘/平面的依据：
  $$
  R=det(M)-k(trace(M)^2
  $$

  - 其中
    - $det(M)=\lambda_1\lambda_2$
    - $trace(M)=\lambda_1+\lambda_2$
    - $\lambda_1和\lambda_2$是M的eigenvalues
    
  - 判断R：
    - When |R| is small, which happens when λ1 and λ2 are small, the region is flat.
    - When R<0, which happens when λ1>>λ2 or vice versa, the region is edge.
    - When R is large, which happens when λ1 and λ2 are large and λ1∼λ2, the region is a corner.
    
    ![](https://docs.opencv.org/4.6.0/harris_region.jpg)

#### 4.1.3 Code

```python
import numpy as np
import cv2 as cv
filename = 'chessboard.png'
img = cv.imread(filename)
gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
gray = np.float32(gray)
# 参数1:图片，2:角点检测的邻域大小，3:索伯算子用到的Aperture参数，4:判据中的k
dst = cv.cornerHarris(gray,2,3,0.04)
#result is dilated for marking the corners, not important
dst = cv.dilate(dst,None)
# Threshold for an optimal value, it may vary depending on the image.
img[dst>0.01*dst.max()]=[0,0,255]
cv.imshow('dst',img)
if cv.waitKey(0) & 0xff == 27:
    cv.destroyAllWindows()
```

更精准的找角

```python
import numpy as np
import cv2 as cv
filename = 'chessboard2.jpg'
img = cv.imread(filename)
gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
# find Harris corners
gray = np.float32(gray)
dst = cv.cornerHarris(gray,2,3,0.04)
dst = cv.dilate(dst,None)
ret, dst = cv.threshold(dst,0.01*dst.max(),255,0)
dst = np.uint8(dst)
# find centroids中心点
## connectedComponentsWithStats()：求得最大连通域。用于过滤原始图像中轮廓分析后较小的区域，留下较大区域。
ret, labels, stats, centroids = cv.connectedComponentsWithStats(dst)
# define the criteria to stop and refine the corners
# criteria是一个枚举集合：参数1:终止标准,2:迭代的最大次数,3:希望的准确度
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.001)
# 参数1:图片，2:输入角点的初始坐标和为输出提供的细化坐标，3:寻找窗口边长的一般，4:搜索区域中间的死区大小的一半（-1,-1）表示这里没有这区域
corners = cv.cornerSubPix(gray,np.float32(centroids),(5,5),(-1,-1),criteria)
# Now draw them
res = np.hstack((centroids,corners))
res = np.int0(res)
img[res[:,1],res[:,0]]=[0,0,255]
img[res[:,3],res[:,2]] = [0,255,0]
cv.imwrite('subpixel5.png',img)
```

### 4.2 Shi-Tomasi Corner Detector 
[Shi-Tomasi Corner Detector ](https://docs.opencv.org/4.6.0/d4/d8c/tutorial_py_shi_tomasi.html)

#### 4.2.1 Functions to learn

- **[cv.goodFeaturesToTrack()](https://docs.opencv.org/4.6.0/dd/d1a/group__imgproc__feature.html#ga1d6bb77486c8f92d79c8793ad995d541)**:使用Shi-Tomasi(默认)或者Harris Corner角点检测来找N个最强的角点

#### 4.2.2 Theory

- **Shi-Tomasi**角点检测与**Harris**角点检测的相比只有判据变了：
  $$
  R=min(\lambda_1,\lambda_2)
  $$

  - $\lambda_1和\lambda_2$是哈里斯矩阵M的eigenvalues，见4.1.2

  ![](https://docs.opencv.org/4.6.0/shitomasi_space.png)

  - 如果两个特征值大于$\lambda_{min}$时，即绿色区域，才被认为是角点

#### 4.2.3 Code 

```python
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
img = cv.imread('blox.jpg')
gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
# 参数1:图片2:要找出的角点个数3:质量等级0到1的某一个值，低于这个值会被忽略4:2个角点间最小的欧几里德距离
corners = cv.goodFeaturesToTrack(gray,25,0.01,10)
corners = np.int0(corners)
# 在角点处画圆
for i in corners:
    x,y = i.ravel()
    cv.circle(img,(x,y),3,255,-1)
plt.imshow(img),plt.show()
```

### 4.3 SIFT (Scale-Invariant Feature Transform)
[SIFT (Scale-Invariant Feature Transform)](https://docs.opencv.org/4.6.0/da/df5/tutorial_py_sift_intro.html)

#### 4.3.1 Functions to learn

- [cv.SIFT_create()](https://docs.opencv.org/4.6.0/d7/d60/classcv_1_1SIFT.html)

#### 4.3.2 Theory

[reference](https://www.cnblogs.com/Alliswell-WP/p/SIFT.html)

上面的Harris 和 Shi-Tomasi都和是Rotation-incariant,即图片旋转后角还是点角点。但他们不是Scale-Invariant的，即图片拉伸后之前的角点可能就不是角点了。

Scale Invariant Feature Transform (SIFT)尺度不变特征变换,是图像处理领域中的一种**局部特征描述**算法(用于得到关键点描述符)。由4步组成：

1. Scale-space Extrema Detection尺度空间极值检测

   找出所有潜在关键点

   - 显然不能用一个大小的窗口区检测不同尺度下的关键点，即小角用小窗口，大角用大窗口

   - 可以用Laplacian of Gaussian(LoG)高斯拉普拉斯来做scale-space filtering，但成本太大

   - SIFT中使用**Difference of Gaussian(DoG)**来识别出具有尺度和方向不变性的潜在关键点。

     就是把图像做不同程度的高斯模糊blur，平滑的区域或点肯定变化不大，而纹理复杂的比如边缘，点，角之类区域肯定变化很大，这样变化很大的点就是关键点。然为了找到足够的点，还需要把图像放大缩小几倍(Image Pyramids)来重复这个步骤找关键点。

2. Keypoint localization关键点定位：

   剔除一部分关键点

   - 第一步找到了潜在关键点后，使用Taylor series expansion泰勒级数展开来找到更精确的极值extrema的位置，如果极值的强度Intensity小于某一个阈值threshold，那么这些关键点会被消除。这个阈值在OpenCV中称之为**contrastThreshold**.

   - DoG对于边界也有很高的敏感度，所以还需要用一个类似Harris的方法来去除边界edge:使用一个2x2的**Hessian matrix**来计算principal curvature主曲率，如果关键点处的ratio比率大于一个阈值就认为是边界要被剔除。这个阈值在OpenCV中称为**edgeThreshold**
   - 由此low-contrast keypoints 和 edge keypoints都被剔除了 

3. Orientation Assignment方向分配

   给关键点及其周围像素增加方向信息

   - 为了具有旋转不变性，，需要利用图像的局部特征为给每一个关键点分配一个基准方向。使用图像梯度的方法求取局部结构的稳定方向。
   - 计算某一关键点的梯度后，使用直方图统计领域内像素的梯度和方向。梯度直方图将0~360度的方向范围分为36个柱(bins)，其中每柱10度。直方图的峰值方向代表了关键点的主方向。为了增强匹配的鲁棒性，只保留峰值大于主方向峰值80％的方向作为该关键点的辅方向。

4. Keypoint Descriptor关键点描述符

   根觉方向信息创建关键点描述符。

   - 取关键点周围的16X16邻域，将区域划分为4×4的子块，对每一子块进行8个方向(上下左右，左上左下，右上右下)的直方图统计操作，获得每个方向的梯度幅值，总共可以组成128维描述向量.

5. Keypoint Matching关键点匹配

   - 得到描述符后就可以通过identify识别2张图片的关键点的邻域(关键点描述符)来匹配关键点。

#### 4.3.3 Code

```python
import numpy as np
import cv2 as cv
img = cv.imread('home.jpg')
gray= cv.cvtColor(img,cv.COLOR_BGR2GRAY)
# 1.创建sift类对象
sift = cv.SIFT_create()
# 2.寻找关键点
kp = sift.detect(gray,None)
# 3.计算关键点描述符
kp, des = sift.compute(gray,kp)
# 2,3步也可以直接合二为一：
kp, des = sift.detectAndCompute(gray,None)

# 如果给drawKeypoints()下面这个flag:不仅可以画出关键点的大小还可以画出它的方向
img=cv.drawKeypoints(gray,kp,img,flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
#img=cv.drawKeypoints(gray,kp,img)
cv.imwrite('sift_keypoints.jpg',img)

```

- **kp**: a list of keypoints
- **des**: descriptor, a numpy array of shape (Number of Keypoints)×128.

### 4.4 SURF (Speeded-Up Robust Features)
[SURF (Speeded-Up Robust Features)](https://docs.opencv.org/4.6.0/df/dd2/tutorial_py_surf_intro.html)

#### 4.4.1 Theory

- SURF加速稳健特征是SIFT的提速版本，相比于SIFT使用Difference of Gaussian(DoG)来近似Laplacian of Gaussian(LoG)，SURF使用积分图和Boxfilter计算hessian矩阵，通过比较hessian矩阵行列式的大小来选择特征点的位置和尺度,金字塔构建时通过改变boxfilter的尺寸和滤波器的方差来实现不同的尺度图像的获取。

- 积分图integral image：积分图的每一点（x, y）的值是原图中对应位置的左上角区域的所有灰度值的和
  $$
  I_\sum(x)=\sum_{i=0}^{i\leq x}\sum_{j=0}^{j\leq y}I(i,j)
  $$

  - 好处：一旦积分图计算完毕，对任意矩形区域的和的计算就可以在常数时间内完成。

    ![](https://pic2.zhimg.com/80/v2-e5f90518f99f3f102650140626857895_720w.webp)

    绿色区域的的灰度和：$\sum=A-B-C+D$

**SURF steps:**

[参考](https://zhuanlan.zhihu.com/p/365403867)

1. Hessian矩阵
   $$
   H(x,\sigma)=\left[\begin{matrix}
   			L_{xx}(x,\sigma) & L_{xy}(x,\sigma)\\
   			L_{yx}(x,\sigma) & L_{yy}(x,\sigma)\\				
   			\end{matrix}\right]
   $$

   - 其中$L_{xx}(x,\sigma)$是x处图像和二阶高斯核$G(\sigma)=\frac{\partial^2g(\sigma)}{\partial x^2}或\frac{\partial^2g(\sigma)}{\partial x\partial y}或\frac{\partial^2g(\sigma)}{\partial y^2}$的卷积：$L(x,\sigma)=G(\sigma)*I(x)$

   得到Hessian矩阵的行列式值(Determinant of Hessian, DoH) :
   $$
   detH=L_{xx}L_{yy}-L^2_{xy}
   $$

   - DoH的极值点可用于图像斑点检测 (Blob Detection)

2. Hessian矩阵的近似

   从高斯滤波器 转为 盒子滤波器[boxfilter](https://docs.opencv.org/3.4/d4/d86/group__imgproc__filter.html#gad533230ebf2d42509547d514f7d3fbc3)

   ![](https://docs.opencv.org/4.6.0/surf_boxfilter.jpg)

   - 相比左边的是高斯滤波器。右边的盒子滤波器只由几个矩形区域组成，矩形区域内填充同一值：白色1，黑色-1/-2，灰色0。

   - 使用盒子滤波器而不使用高斯滤波的目的是，在卷积时，可以使用积分图中的元素来加快计算。

   - 简化后的Doh为：
     $$
     det(H_{approx})=D_{xx}D_{yy}-(wD_{xy})^2
     $$

     - 权重w=0.9：用于平衡因使用盒式滤波器近似所带来的误差

3. Scale space representation构建尺度空间

   - 尺度空间scale space:
     - 图像的尺度是指图像内容的粗细程度。尺度的概念是用来模拟观察者距离物体的远近的程度。
     - 图像的尺度空间是指同一张图像不同尺度的集合。在该集合中，**细尺度图像通过filter形成粗尺度图像，即粗尺度图像的形成过程是高频信息被过滤的过程**，不会引入新的杂质信息，因此任何存在于粗尺度图像下的内容都能在细尺度图像下找到。
     - 如果你想看一颗树而不是叶子，可以故意删除图像中的某些细节（例如树叶树枝等），在摆脱这些细节时，必须确保不引入新的虚假细节。做到这一点的唯一方法是使用**高斯模糊/过滤**
     - **综上，图像的尺度空间是一幅图像经过几个不同高斯核后形成的模糊图片的集合，用来模拟人眼看到物体的远近程度以及模糊程度。 图像尺度的改变不等于图像分辨率的改变。**
   - 图像金字塔Image pyramid
     - 图像金字塔(image pyramid)是同一张图片不同分辨率子图的集合，是通过对原图像不断地下采样而产生的。高分辨率原始图像在底部，自底向上，分辨率（尺寸）逐渐降低，堆叠起来便形成了金字塔状。通常情况下，每向上移动一级，图像的宽和高都降低为原来的1/2。
   - 要想检测不同尺度的极值点，必须建立图像的尺度空间金字塔。一般的方法是通过采用不同 $\sigma$的高斯函数，对图像进行平滑滤波，然后重采样获得更高一层的金字塔图像。
     - SIFT算法中是通过相邻两层高斯金字塔图像相减得到DoG图像，然后在DoG金字塔图像上进行特征点检测.
     - 与SIFT特征不同的是，SURF算法不需要通过降采样的方式得到不同尺寸大小的图像建立金字塔，而是借助于盒式滤波和积分图像，不断增大盒式滤波模板，通过积分图快速计算盒式滤波的响应图像。然后在响应图像上采用非极大值抑制，检测不同尺度的特征点。

4. Keypoint localization关键点定位

   - 将经过盒式滤波处理过的响应图像中每个像素点与其3维邻域中的26个像素点进行比较，若是最极大值点，则认为是该区域的局部特征点。然后，采用3维线性插值法得到亚像素级的特征点，同时去掉一些小于给定阈值的点，使得极值检测出来的特征点更稳健。
   - 和SIFT中用到的DoG不同的是不用剔除边缘导致的极值点了，因为Hessian矩阵的行列式就已经考虑到边缘的问题了，而DoG计算只是把不同方向变化趋势给出来，后续还需要使用Hessian矩阵的特征值剔除边缘产生的影响。

5. Orientation Assignment方向分配

   - 在以特征点为中心，以6s (s为特征点的尺度，计算过程为：$s=1.2\times \frac{L}{9}$  )为半径的区域内，计算图像的[Haar小波响应](https://zhuanlan.zhihu.com/p/386322623)，实际上就是对图像进行梯度运算，只不过需要利用积分图，提高梯度计算效率。

   - 使用$\sigma=2s$的高斯函数对Haar小波的响应值进行加权

   - 为了求主方向，设计一个以特征点为中心，张角为60度的扇形窗口，以一定旋转角度 $\theta$ 旋转窗口，并对窗口内的Haar小波响应值dx,dy进行累加，得到一个矢量  :$m_w,\theta_w$
     $$
     m_w=\sum_wdx+\sum_wdy \\
     \theta_w=arctan(\sum_wdy/\sum_wdx)
     $$

     - 主方向即最大haar响应累加值所对应的方向$\theta=\theta_w|max\{m_w\}$
     - 累加值大于主方向80%的认为是该关键点的副方向

     ![](https://docs.opencv.org/4.6.0/surf_orientation.jpg)

6. Keypoint Descriptor关键点描述符

   - 首先将20s的窗口划分为4×4个子窗口，每个子窗口大小为5s×5s，使用尺寸为2s的Haar小波计算子窗口的响应值；

   - 然后，以特征点为中心，用 $\sigma\frac{20s}{6}$的高斯核函数对 dx,dy 进行加权计算；

   - 最后，分别对每个子块的加权响应值进行统计，得到每个子块的向量
     $$
     V_i=[\sum dx,\sum|dx|,\sum dy,\sum|dy|]
     $$
     ![](https://pic3.zhimg.com/80/v2-a43774666dde849a16408e231b69ff12_720w.webp)

     - 由于共有4×4个子块，特征描述子的特征维数为4×4×4=64。
     - SURF描述子不仅具有尺度和旋转不变性，还具有光照不变性，这由小波响应本身决定，而对比度不变性则是通过将特征向量归一化来实现。

   - 还有一个U-SURF: 不计算特征点的方向信息，所有点统一认为方向向上，只需要将设为0即可。

#### 4.4.2 Code

```python
# 1.SURF
>>> img = cv.imread('fly.png',0)
# Create SURF object. You can specify params here or later.
# Here I set Hessian Threshold to 400
>>> surf = cv.xfeatures2d.SURF_create(400)
# Find keypoints and descriptors directly
>>> kp, des = surf.detectAndCompute(img,None)
>>> len(kp)
 699
    
# Check present Hessian threshold
>>> print( surf.getHessianThreshold() )
400.0
# We set it to some 50000. Remember, it is just for representing in picture.
# In actual cases, it is better to have a value 300-500
>>> surf.setHessianThreshold(50000)
# Again compute keypoints and check its number.
>>> kp, des = surf.detectAndCompute(img,None)
>>> print( len(kp) )
47
# 将关键点画在图上
>>> img2 = cv.drawKeypoints(img,kp,None,(255,0,0),4)
>>> plt.imshow(img2),plt.show()

# 2.U-SURF：
# Check upright flag, if it False, set it to True
>>> print( surf.getUpright() )
False
>>> surf.setUpright(True)
# Recompute the feature points and draw it
>>> kp = surf.detect(img,None)
>>> img2 = cv.drawKeypoints(img,kp,None,(255,0,0),4)
>>> plt.imshow(img2),plt.show()

# 3.Check the descriptor size and change it to 128 if it is only 64-dim. 
# Find size of descriptor
>>> print( surf.descriptorSize() )
64
# That means flag, "extended" is False.
>>> surf.getExtended()
 False
# So we make it to True to get 128-dim descriptors.
>>> surf.setExtended(True)
>>> kp, des = surf.detectAndCompute(img,None)
>>> print( surf.descriptorSize() )
128
>>> print( des.shape )
(47, 128)
```



### 4.5 FAST (Features from Accelerated Segment Test)
[FAST (Features from Accelerated Segment Test)](https://docs.opencv.org/4.6.0/df/d0c/tutorial_py_fast.html)
#### 4.5.1 Theory



### 4.6 BRIEF (Binary Robust Independent Elementary Features)
[BRIEF (Binary Robust Independent Elementary Features)](https://docs.opencv.org/4.6.0/dc/d7d/tutorial_py_brief.html)

### 4.7 ORB (Oriented FAST and Rotated BRIEF)
[ORB (Oriented FAST and Rotated BRIEF)](https://docs.opencv.org/4.6.0/d1/d89/tutorial_py_orb.html)

### 4.8 Feature Matching]
[Feature Matching](https://docs.opencv.org/4.6.0/dc/dc3/tutorial_py_matcher.html)

### 4.9 Feature Matching + Homography to find Objects
[Feature Matching + Homography to find Objects](https://docs.opencv.org/4.6.0/d1/de0/tutorial_py_feature_homography.html)

## 5. Video analysis (video module)
[Video analysis (video module)](https://docs.opencv.org/4.6.0/da/dd0/tutorial_table_of_content_video.html)

## 6. Camera Calibration and 3D Reconstruction
[Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.6.0/d9/db7/tutorial_py_table_of_contents_calib3d.html)

### 6.1 Calibration

#### 6.1.1 Basic ideas

- 为什么要标定

  - 进行摄像机标定的目的：求出相机的内、外参数，以及畸变参数。
  - 标定相机后通常是想做两件事：
    - 一个是由于每个镜头的畸变程度各不相同，通过相机标定可以校正这种镜头畸变矫正畸变，生成矫正后的图像；
    - 另一个是根据获得的图像重构三维场景。

- 两种主要的失真：

  1. Radial distortion径向畸变

     - 径向畸变使竖直线发生弯曲，离中心点离得越远畸变的越严重。

     - 用数学来表达径向畸变: 
       $$
       x_{distorted} = x(1+k_1r^2+k_2r^4+k_3r^6) \\
       y_{distorted} = y(1+k_1r^2+k_2r^4+k_3r^6)
       $$
       

  2. Tangential distorition切向畸变

     - 切向畸变使横直线发生弯曲

     - 用数学来表达横向畸变:
       $$
       x_{distorted}=x+[2p_1xy+p_2(r^2+2x^2)]\\
       y_{distorted}=y+[p_1(r^2+2y^2)+2p_2xy]
       $$

- 标定要确定的参数：

  1. 上面两个失真方程中的系数:$k_1,k_2,k_3,p_1,p_2$

  2. 每个相机独一无二的intrinsic parameters，有focal length焦距$(f_x,f_y)$和optical centers$(c_x,c_y)$

     这些参数用于计算camera matrix:
     $$
     camera\ matrix= \left[
      \begin{matrix}
        f_x & 0 & c_x \\
        0 & f_y & c_y \\
        0 & 0 & 1
       \end{matrix}
       \right]
     $$
     相机矩阵同样是每个相机唯一的，只需要计算一次

  3. extrinsic parameters: 用于将3D点的坐标和相机坐标系对应起来

#### 6.1.2 Code

需要一张棋盘，我们知道棋盘上点在现实世界中的坐标，也知道了图像中的坐标，由此可求是失真系数。至少要拍10张图片。

3D points are called **object points** and 2D image points are called **image points.**

- Setup

  1. 使用**[cv.findChessboardCorners()](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga93efa9b0aa890de240ca32b11253dd4a)**找棋盘上的图案pattern
  2. 使用 **[cv.cornerSubPix()](https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html#ga354e0d7c86d0d9da75de9b9701a9a87e)**增加准确性
  3. 可以用 **[cv.drawChessboardCorners()](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga6a10b0bb120c4907e5eabbcd22319022)**画图案

  ```python
  import numpy as np
  import cv2 as cv
  import glob
  # termination criteria
  criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
  # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
  # 这里画的7x6的格子，3表示x,y,z，一般认为棋盘在xy平面上，所以z=0
  objp = np.zeros((6*7,3), np.float32)
  # np.mgrid[0:7 0:6]返回一个dense multi-dimensional 矩阵
  objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
  # Arrays to store object points and image points from all the images.
  objpoints = [] # 3d point in real world space
  imgpoints = [] # 2d points in image plane.
  
  # glob是python自己带的一个文件操作相关模块，用它可以查找符合自己目的的文件
  # glob.glob(r’c:*.txt’)       获得C盘下的所有txt文件的名字
  # glob.glob(r’E:\pic**.jpg’)  获得指定目录下的所有jpg文件的名字
  # glob.iglob(r'../*.py')      获得父目录中所有的.py文件的名字
  images = glob.glob('*.jpg')
  
  for fname in images:
      img = cv.imread(fname)
      gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
      # Find the chess board corners
      ret, corners = cv.findChessboardCorners(gray, (7,6), None)
      # If found, add object points, image points (after refining them)
      if ret == True:
          objpoints.append(objp)
          corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
          imgpoints.append(corners2)
          # Draw and display the corners
          cv.drawChessboardCorners(img, (7,6), corners2, ret)
          cv.imshow('img', img)
          cv.waitKey(500)
  cv.destroyAllWindows()
  ```

- Calibration

  使用 **[cv.calibrateCamera()](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d)** 来标定

  ```python
  ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
  ```

- Undistortion

  标定好后就可以对图像进行去畸变了。

  1. 使用**[cv.getOptimalNewCameraMatrix()](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga7a6c4e032c97f03ba747966e6ad862b1)**来精细化相机矩阵：

     - If the scaling parameter alpha=0, it returns undistorted image with minimum unwanted pixels. So it may even remove some pixels at image corners.

     -  If alpha=1, all pixels are retained with some extra black images. This function also returns an image ROI which can be used to crop the result

     ```python
     img = cv.imread('left12.jpg')
     h,  w = img.shape[:2]
     newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
     ```
  
  2. 使用**[cv.undistort()](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga69f2545a8b62a6b0fc2ee060dc30559d)**去畸变
  
     ```python
     # undistort
     dst = cv.undistort(img, mtx, dist, None, newcameramtx)
     
     # crop the image
     x, y, w, h = roi	# roi: region of interest
     dst = dst[y:y+h, x:x+w]
     cv.imwrite('calibresult.png', dst)
     ```
  
  3. 也可以使用 [**cv.remap()**](https://docs.opencv.org/4.x/da/d54/group__imgproc__transform.html#gab75ef31ce5cdfb5c44b6da5f3b908ea4)去畸变
  
     ```python
     # undistort
     # 1.先找一个畸变图像到非畸变图像的映射
     mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
     # 2.再重映射
     dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
     
     # crop the image
     x, y, w, h = roi
     dst = dst[y:y+h, x:x+w]
     cv.imwrite('calibresult.png', dst)
     ```
  
- 至此可以保存相机矩阵的畸变系数了，可以使用 (np.savez, np.savetxt etc) 

## 7. Machine Learning
[Machine Learning](https://docs.opencv.org/4.6.0/d6/de2/tutorial_py_table_of_contents_ml.html)

## 8. Computational Photography
[Computational Photography](https://docs.opencv.org/4.6.0/d0/d07/tutorial_py_table_of_contents_photo.html)

## 9. Object Detection (objdetect module)
[Object Detection (objdetect module)](https://docs.opencv.org/4.6.0/d2/d64/tutorial_table_of_content_objdetect.html)

## 10. OpenCV-Python Bindings
[OpenCV-Python Bindings](https://docs.opencv.org/4.6.0/df/da2/tutorial_py_table_of_contents_bindings.html)



# Opencv-C++

## 1. 用CMake来编译

- C++: test.cpp

  ```c++
  #include <stdio.h>
  #include <opencv4/opencv2/opencv.hpp>
  using namespace cv;
  int main(int argc, char** argv )
  {
      if ( argc != 2 )
      {
          printf("usage: ./test <Image_Path>\n");
          return -1;
      }
      Mat image;
      image = imread( argv[1], 1 );
      if ( !image.data )
      {
          printf("No image data \n");
          return -1;
      }
      namedWindow("Display Image", WINDOW_AUTOSIZE );
      imshow("Display Image", image);
      waitKey(0);
      return 0;
  }
  ```

- CMakeLists.txt

  ```cmake
  cmake_minimum_required(VERSION 2.8)
  project( test )
  find_package( OpenCV REQUIRED )
  include_directories( ${OpenCV_INCLUDE_DIRS} )
  add_executable( test test.cpp )
  target_link_libraries( test ${OpenCV_LIBS} )
  ```

- Run:

  ```shell
  ./test ../ubuntu.png 
  ```


## 2. 基本操作

### 2.1 Mat图片保存格式

[Mat类](https://docs.opencv.org/4.7.0/d3/d63/classcv_1_1Mat.html#a55ced2c8d844d683ea9a725c60037ad0)提供了各种矩阵方法。

- mat是opencv中用来存储图片的数据结构，由俩部分组成

  - **matrix header** (containing information such as the size of the matrix, the method used for storing, at which address is the matrix stored, and so on) 
  - **pointer to the matrix containing the pixel values** (taking any dimensionality depending on the method chosen for storing) 

- 浅复制

  ```c++
  Mat A, C;                          // creates just the header parts
  A = imread(argv[1], IMREAD_COLOR); // here we'll know the method used (allocate matrix)
  Mat B(A);                                 // Use the copy constructor
  C = A;                                    // Assignment operator
  ```

  - 他们指向的都是同一张照片，改变其中一个都会互相影响

- 深复制

  ```c++
  Mat F = A.clone();
  Mat G;
  A.copyTo(G);
  ```

  -  使用[cv::Mat::clone()](https://docs.opencv.org/4.7.0/d3/d63/classcv_1_1Mat.html#a03d2a2570d06dcae378f788725789aa4) and [cv::Mat::copyTo()](https://docs.opencv.org/4.7.0/d3/d63/classcv_1_1Mat.html#a33fd5d125b4c302b0c9aa86980791a77)方法复制的，就是独立的个体

- 划分ROI

  创建一个新的header来划分图片中感兴趣的区域

  ```c++
  Mat D (A, Rect(10, 10, 100, 100) ); // using a rectangle
  Mat E = A(Range::all(), Range(1,3)); // using row and column boundaries
  ```


- 对于灰度图，矩阵如下：

  ![](https://docs.opencv.org/4.7.0/tutorial_how_matrix_stored_1.png)

- 对于彩色图，矩阵如下：

  ![](https://docs.opencv.org/4.7.0/tutorial_how_matrix_stored_2.png)

### 2.2  扫描图片，查询表和时间测量

- 查询表LUT：

  LUT(Look-Up Table)实际上就是一张像素灰度值的映射表，它将实际采样到的像素灰度值经过一定的变换如阈值、反转、二值化、对比度调整、线性变换等，变成了另外一个与之对应的灰度值，这样可以起到突出图像的有用信息，增强图像的光对比度的作用。

- 映射图片的方法：

  1. 扫描图片读取每个像素，然后对他们操作

     有三种读取方法:

     1. Mat& ScanImageAndReduceC(Mat& I, const uchar* table);

        使用指针的方法。推荐，快

     2. Mat& ScanImageAndReduceIterator(Mat& I, const uchar* table);

        使用迭代器的方法。推荐，安全

     3. Mat& ScanImageAndReduceRandomAccess(Mat& I, const uchar * table);

        随机的方法。不推荐

  2. 使用cv::lut()方法

     opencv自带的方法，不用我们再去写扫描每个像素的代码，直接就可以映射了

- opencv测量时间

  使用如下两个方法：

  - cv::getTickFrequency()计算机每秒钟的Tick数
  - cv::getTickCount(),得到开机后到现在的tick数

  两者得到值除一下，就是运行了多少秒了。

- 代码：

  ```c++
  #include <opencv2/core.hpp>
  #include <opencv2/core/utility.hpp>
  #include "opencv2/imgcodecs.hpp"
  #include <opencv2/highgui.hpp>
  #include <iostream>
  #include <sstream>
  
  using namespace std;
  using namespace cv;
  
  static void help()
  {
      cout
          << "\n--------------------------------------------------------------------------" << endl
          << "This program shows how to scan image objects in OpenCV (cv::Mat). As use case"
          << " we take an input image and divide the native color palette (255) with the "  << endl
          << "input. Shows C operator[] method, iterators and at function for on-the-fly item address calculation."<< endl
          << "Usage:"                                                                       << endl
          << "./how_to_scan_images <imageNameToUse> <divideWith> [G]"                       << endl
          << "if you add a G parameter the image is processed in gray scale"                << endl
          << "--------------------------------------------------------------------------"   << endl
          << endl;
  }
  
  Mat& ScanImageAndReduceC(Mat& I, const uchar* table);
  Mat& ScanImageAndReduceIterator(Mat& I, const uchar* table);
  Mat& ScanImageAndReduceRandomAccess(Mat& I, const uchar * table);
  
  int main( int argc, char* argv[])
  {
      help();
      if (argc < 3)
      {
          cout << "Not enough parameters" << endl;
          return -1;
      }
  
      Mat I, J;
      if( argc == 4 && !strcmp(argv[3],"G") )
          I = imread(argv[1], IMREAD_GRAYSCALE);  //如果添加了参数G，就读取为灰度图
      else
          I = imread(argv[1], IMREAD_COLOR);      //否则，就正常的彩图。
  
      if (I.empty())
      {
          cout << "The image" << argv[1] << " could not be loaded." << endl;
          return -1;
      }
  
      //********************************** start:[dividewith] ********************************** 
      int divideWith = 0; // convert our input string to number - C++ style
      stringstream s;     // 使用c++的stringtream类，来将字符串转换为数字
      s << argv[2];       // 将命令行中的第二个参数值传给s
      s >> divideWith;    // 将string值传给int类型的变量，也就变成了int值。
      if (!s || !divideWith)
      {
          cout << "Invalid number entered for dividing. " << endl;
          return -1;
      }
  
      uchar table[256];       //为什么要用char呢，因为这样所有的分数都会向下舍入。
      for (int i = 0; i < 256; ++i)   // 下面这个公式，是将[0,255]的色域缩小一个比例，因为通常不需要那么多色域。
         table[i] = (uchar)(divideWith * (i/divideWith));
      //********************************** end:[dividewith] ********************************** 
  
      //********************************** start:[时间计算1：指针方法] ********************************** 
      const int times = 100;
      double t;
  
      t = (double)getTickCount(); //cv::getTickCount(),得到开机后到现在的tick数
  
      for (int i = 0; i < times; ++i)
      {
          cv::Mat clone_i = I.clone(); // 深复制，clone_i和I是两个独立的Mat对象了
          J = ScanImageAndReduceC(clone_i, table);
      }
  
      t = 1000*((double)getTickCount() - t)/getTickFrequency(); //cv::getTickFrequency()计算机每秒钟的Tick数
      t /= times;
  
      cout << "Time of reducing with the C operator [] (averaged for "
           << times << " runs): " << t << " milliseconds."<< endl;
  
      t = (double)getTickCount();
      //********************************** end:[时间计算1：指针方法] **********************************
      
      //********************************** start:[时间计算2：迭代器方法] ********************************** 
      for (int i = 0; i < times; ++i)
      {
          cv::Mat clone_i = I.clone();
          J = ScanImageAndReduceIterator(clone_i, table);
      }
  
      t = 1000*((double)getTickCount() - t)/getTickFrequency();
      t /= times;
  
      cout << "Time of reducing with the iterator (averaged for "
          << times << " runs): " << t << " milliseconds."<< endl;
      //********************************** end:[时间计算2：迭代器方法] **********************************
  
      //********************************** start:[时间计算3：随即扫描] ********************************** 
      t = (double)getTickCount();
  
      for (int i = 0; i < times; ++i)
      {
          cv::Mat clone_i = I.clone();
          ScanImageAndReduceRandomAccess(clone_i, table);Mat& ScanImageAndReduceC(Mat& I, const uchar* table);
  Mat& ScanImageAndReduceIterator(Mat& I, const uchar* table);
  Mat& ScanImageAndReduceRandomAccess(Mat& I, const uchar * table);
      }
  
      t = 1000*((double)getTickCount() - t)/getTickFrequency();
      t /= times;
  
      cout << "Time of reducing with the on-the-fly address generation - at function (averaged for "
          << times << " runs): " << t << " milliseconds."<< endl;
      //********************************** end:[时间计算3：随机扫描] **********************************
  
      //********************************** start:使用CV::LUT()来修改图像值 ********************************** 
      //! [table-init]
      Mat lookUpTable(1, 256, CV_8U);
      uchar* p = lookUpTable.ptr();
      for( int i = 0; i < 256; ++i)
          p[i] = table[i];
      //! [table-init]
  
      t = (double)getTickCount();
  
      for (int i = 0; i < times; ++i)
          //! [table-use]
          LUT(I, lookUpTable, J);
          //! [table-use]
  
      t = 1000*((double)getTickCount() - t)/getTickFrequency();
      t /= times;
  
      cout << "Time of reducing with the LUT function (averaged for "
          << times << " runs): " << t << " milliseconds."<< endl;
      //********************************** end:使用CV::LUT()来修改图像值 ********************************** 
      return 0;
  }
  
  //! [scan-c]
  Mat& ScanImageAndReduceC(Mat& I, const uchar* const table)
  {
      // accept only char type matrices
      CV_Assert(I.depth() == CV_8U); // CV_Assert断言，如果为真，中断。 CV_8U定义为0。
  
      int channels = I.channels();  // 通道数
  
      int nRows = I.rows;
      int nCols = I.cols * channels;  // 参见2.1中图片的矩阵格式，若为颜色图，则每一列有3个通道
  
      if (I.isContinuous())   // 检测矩阵读取的是否是连续的，如果是连续的，就可以把他们变成一行来处理
      {
          nCols *= nRows;
          nRows = 1;
      }
  
      int i,j;
      uchar* p;
      for( i = 0; i < nRows; ++i)
      {
          p = I.ptr<uchar>(i);    // 返回i行的指针
          for ( j = 0; j < nCols; ++j)   // 遍历i行的每一列元素
          {
              p[j] = table[p[j]]; // 将[0,255]的p[j]放入table后降低色域分辨率。
          }
      }
      return I;
  }
  //! [scan-c]
  
  //! [scan-iterator]
  Mat& ScanImageAndReduceIterator(Mat& I, const uchar* const table)
  {
      // accept only char type matrices
      CV_Assert(I.depth() == CV_8U);
  
      const int channels = I.channels();
      switch(channels)
      {
      case 1: //灰度图
          {
              MatIterator_<uchar> it, end;
              for( it = I.begin<uchar>(), end = I.end<uchar>(); it != end; ++it)
                  *it = table[*it];
              break;
          }
      case 3: //彩色图
          {
              MatIterator_<Vec3b> it, end;
              for( it = I.begin<Vec3b>(), end = I.end<Vec3b>(); it != end; ++it)
              {
                  (*it)[0] = table[(*it)[0]];
                  (*it)[1] = table[(*it)[1]];
                  (*it)[2] = table[(*it)[2]];
              }
          }
      }
  
      return I;
  }
  //! [scan-iterator]
  
  //! [scan-random]
  Mat& ScanImageAndReduceRandomAccess(Mat& I, const uchar* const table)
  {
      // accept only char type matrices
      CV_Assert(I.depth() == CV_8U);
  
      const int channels = I.channels();
      switch(channels)
      {
      case 1:
          {
              for( int i = 0; i < I.rows; ++i)
                  for( int j = 0; j < I.cols; ++j )
                      I.at<uchar>(i,j) = table[I.at<uchar>(i,j)];
              break;
          }
      case 3:
          {
           Mat_<Vec3b> _I = I;
  
           for( int i = 0; i < I.rows; ++i)
              for( int j = 0; j < I.cols; ++j )
                 {
                     _I(i,j)[0] = table[_I(i,j)[0]];
                     _I(i,j)[1] = table[_I(i,j)[1]];
                     _I(i,j)[2] = table[_I(i,j)[2]];
              }
           I = _I;
           break;
          }
      }
  
      return I;
  }
  //! [scan-random]
  
  ```



