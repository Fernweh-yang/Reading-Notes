# App designer

## 1. 读取一个图片

- 添加按钮app.ChoosePictureButton，坐标轴图app.UIAxes

- 给按钮app.ChoosePictureButton添加回调函数：

  ```matlab
  function ChoosePictureButtonPushed(app, event)
  	# 选择图片
  	# *表示的是一个通配符，只有下面这三种可以被选中
      [file,path]=uigetfile({'*.png';'*.bmp';'*.jpg';'*.*'},"选择图像");
      # 如果正确选择了图片， 就获取图片地址。
      if isequal(file,0) || isequal(path,0)
          errordlg("Didn't choose a picture","Error");
      else
     		# stract()横向连接字符串
          filepath = strcat(path,file);
      end
      # 将图片地址输出到一个EditField中去
      app.EditField.Value=filepath;
      im = imread(filepath);
      # 读取图片
      imshow(im,"Parent",app.UIAxes);
  end
  ```

  - uigetfile的使用

    用处：交互式操作去的文件名

    `[FILENAME, PATHNAME, FILTERINDEX] = uigetfile(FILTERSPEC, TITLE, DEFAULTNAME)`

    - FILENAME: 返回的文件名
    - PATHNAME: 返回的文件路径
    - FILTERINDEX：返回的文件类型
    - FILTERSPEC：文件类型设置
    - TITLE：代开对话框的标题
    - DEFAULTNAME：默认指向的文件名

## 2. 获得当前鼠标点击的坐标

- 比如要获得鼠标点击在一个轴图app.UIAxes上的坐标

```matlab
function UIAxesButtonDown(app, event)
    xy = app.UIAxes.CurrentPoint;
    app.xEditField.Value=xy(1); 
    app.yEditField.Value=xy(2);
end
```

## 3. 消息盒子

- 添加以各按钮app.Button_3

  ```matlab
  function Button_3Pushed(app, event)
     msgbox("显示内容","标题");
     % 消息盒子其他用法：
     % msgbox("显示内容","标题","系统图标"); 图标主要有none,error,help,warn
     % msgbox("显示内容","标题","自定义图标"); 自定义要指定custom,IconData,IconCMap
  end
  ```

## 4. 退出程序的设计

- 添加一个按钮app.Button_4

  ```matlab
  function Button_4Pushed(app, event)
  	% questdlg()打开询问框，是否关闭
  	% 1.提示信息，2.标题，3.第一个选择按钮，4.第二个选择按钮，5.代表默认选择哪一个
      choice = questdlg('是否关闭','标题','Yes','No','No');
      switch choice
          case 'Yes'
          	# 关闭要关闭的
             delete(app.UIFigure);
          case 'No'
              return;
      end
  end
  ```

## 5. 各种回调函数

### 5.1 startupFcn 和closerequest回调函数

- 对app右键选择添加startupFcn 或closerequest回调函数

  - startupFcn 可以初始化的定义一些东西,一打开就运行一些什么
  - closerequest 关闭了整个app还可以运行一些东西

  ```matlab
  function startupFcn(app)
  	% 一打开app就跳出一个窗口
      msgbox("cool")
  end
  
  function UIFigureCloseRequest(app, event)
  	% 关闭了,会用浏览器打开一个网站
      delete(app)
      url = "www.google.com";
      web(url);
  end
  ```

  

### 5.2 键盘回车回调函数

- 右键app.UIFigure创建UIFigureKeyPress回调函数

  ```matlab
  function UIFigureKeyPress(app, event)
  	% 获取键盘输入值
      key = event.Key;
      switch key
          case "return" % return表示回车
          	% 可以直接调用某一个函数
          	% 这里调用6.1中那个回调函数
              ButtonPushed(app,event);
      end
  end
  ```

### 5.3 单击触发响应事件

- 右键app.UIFgure添加ButtonDownFcn

  比如要获得鼠标点击在一个轴图app.UIAxes上的坐标

  ```matlab
  function UIAxesButtonDown(app, event)
      xy = app.UIAxes.CurrentPoint;
      app.xEditField.Value=xy(1); 
      app.yEditField.Value=xy(2);
  end
  ```

### 5.4 窗口回调函数

- 右键app.UIFigure添加windows callback

  有比如WindowKeyPress，WindowKeyPress之类的回调函数

- 相比于普通的比如5.2,5.3的回调函数只能监听主节点app.UIFigure, 窗口回调函数可以监听所有的字节点

  - 比如app.UIFgure有个app.Edit Field子节点. 当我们光标选中子节点时,普通的是不反应的,但窗口回调函数还是响应的
  
  
  

## 6. 两个APP之间的交互

### 6.1 调用另一个app或工具箱软件

- 添加一个按钮app.Button

  ```matlab
  function ButtonPushed(app, event)
  	% 通过run来调用
      % 调用我们写的一个app: run gui.mlapp
      % 也可以调用matlab自带的工具箱软件，比如imageLabeler
      run imageLabeler;
  end
  ```

### 6.2 调用外部m函数

- 例子：2个数字相加，并读取值和类型

  - 工作目录下创建一个.m文件

    ```matlab
    function v=test(a,b)
        v = string(a+b);
    end
    ```

  - 新建一个按钮的回调函数

    ```matlab
    function ButtonPushed(app, event)
    	% 调用m函数
        v = test(1,2);
        % 分别在2个editfield中读取值和类型
        app.valueEditField.Value = v;
        app.typeEditField.Value = class(v);
    end
    ```

- 如果向调用其他文件夹下的.m文件

  可以直接将那个文件夹添加为工作路径

  ```matlab
  addpath("E:\tutorial\")
  run xx.m
  ```

### 6.3 将app产生的变量转入工作空间

转入工作空间后，其他的.m文件就可以用这些值了

- 创建一个回调函数

  ```matlab
  function ButtonPushed(app, event)
      x = 10;
      % 第一个参数：指派的地方
      % 第二个参数：指派到目的地后的变量名称
      % 第三个参数：该变量的值
      assignin("base","x",x);
      % 也可以将值保存在当前目录下
      save x;
  end
  ```


## 7. 共享属性

- 不同callback想共同访问某个变量,就要设置private property私有属性

  ```matlab
  % 通过工具栏上的property添加
  properties (Access = private)
      file % Description
  end
  
  function ButtonPushed(app, event)
      app.file = imread("Einfach2.png");
      imshow(app.file,"Parent",app.UIAxes);
  end
  
  function Button2Pushed(app, event)
      app.file = rgb2gray(app.file)
      imshow(app.file,"Parent",app.UIAxes2);
  end
  ```

- 如果是不同文件想访问某个变量,就要设置public property公有属性

# 代码创建gui

- 一个例子

  ```matlab
  function test1
      % close other windows
      close all
      global GUI
      % initialize the gui
      % inspect(h) to open Property Inspector
      GUI.h=figure('units','pixels',...
             'position', [500 500 450 250],...
             'MenuBar','none',...
             'Name','TIP GUI',...
             'NumberTitle','off',...
             'Resize','off');
      % move the gui to the center
      movegui(GUI.h,'center');
      
      GUI.button = uicontrol('Parent',GUI.h,'Style','pushbutton','String','button',...
                         'Position',[140 197 50 30],Visible='on',...
                         Callback=@ChangeEditFcn);
      GUI.text = uicontrol('Parent',GUI.h,'Style','text',String='Attendance Set:',Position=[10 160 125 22],...
                       FontSize=12, FontWeight='bold',...
                       HorizontalAlignment='left');
      GUI.edit = uicontrol(Parent=GUI.h,Style="edit",String='10',Position=[140 157 50 30], FontSize=12,...
                       Visible='on');
  
  end
  
  function ChangeEditFcn(~,~)
      global GUI
      set(GUI.edit, 'string',5)
  end
  ```

  