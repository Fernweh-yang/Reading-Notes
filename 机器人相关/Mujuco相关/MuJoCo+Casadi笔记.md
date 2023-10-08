# 卸载安装编译

## 1. 源码安装

1. 从[官方git](https://github.com/google-deepmind/mujoco)下载最新release: Source code

2. 解压后编译安装

   ```shell
   cd xx/mujoco-2.3.7
   mkdir build
   cd build
   cmake .. #配置cmake
   cmake --build . #编译
   cmake .. -DCMAKE_INSTALL_PREFIX=/opt/mujoco #安装位置设为/opt/mujoco
   cmake --install . #安装
   ```

## 2. 预编译安装

1. 从[官方git](https://github.com/google-deepmind/mujoco)下载最新release: mujoco-2.3.x-linux-x86_64.tar.gz

2. 解压后就可以用了
   ```
   cd mujoco-2.3.7/bin
   ./simulate ../model/car/car.xml
   ```

## 3. 编译自己的代码

- 官方的[CMakeLists.txt]()参考
- 也可以直接写[Makefile](https://github.com/Fernweh-yang/HIWI_Programming_Robots/blob/main/mujoco-2.3.5/task6/makefile)
  - 我这里写的编译完后的可执行文件在`mujoco-2.3.7/bin`中

# 基本概念

## 介绍
- MuJoCo可用于基于模型的计算：

  - control synthesis控制合成，state estimation状态估计，system identification系统识别，mechanism design机械设计，通过逆动力学进行data analysis数据分析和parallel sampling for machine learning applications。

- 模型与数据分离：

  - `mjModel`包含模型描述，预计将保持不变
  - `mjData`包含所有动态变量和中间结果。作为一个scratch pad暂存器，所有函数从中读取输入和写入输出。它有一个预先分配的内部管理的栈，用来在运行时模块在模型初始化后不需要调用内存分配函数。
  - `mjModel`由编译器构建,`mjData`在运行时创建。top-level API函数反映了这种基本分离：`void mj_step(const mjModel* m, mjData* d);`

- 模型实例：

  |            | 高水平          | 低级             |
  | ---------- | --------------- | ---------------- |
  | 文件File   | MJCF/URDF(XML)  | MJB(二进制)      |
  | 储存Memory | mjCModel(c++类) | mjModel(C结构体) |

  三种获得mjModel的方法：(第二种尚未实现)

  1. (text editor)→ MJCF/URDF 文件 → (MuJoCo 解析器parser → mjCModel → MuJoCo 编译器compiler) → mjModel
  2. (用户代码) → mjCModel → (MuJoCo 编译器) → mjModel
  3. MJB 文件 →（MuJoCo 加载器loader）→ mjModel
  
- MuJoCo模拟器包含3部分

  1. STL文件，即三维模型
  2. XML文件，用于定义运动学和动力学关系
  3. 模拟器构建py文件，使用mujoco-py将XML model创建成可交互的环境，供强化学习算法调用。

## URDF/MJCF

[参考网站](https://mp.weixin.qq.com/s?__biz=MzI3MjI1NjMxMA==&mid=2247483652&idx=1&sn=0ea45ecfa99f74b4fe4e83b1839a57d3&chksm=eb341d5ddc43944b2d35ed6de7b61bb9d66c1f16e06d97380aa62f3944c104c24dae22825479&token=1676375003&lang=zh_CN#rd)

Unifed Robot Description Format统一机器人描述格式

- URDF的mesh文件包含
  - 材质颜色等信息的dae文件
  - 用于碰撞的stl文件
  - Mujoco只支持stl文件，所有的dae文件需要转为stl文件。可以使用MeshLab等软件进行转换
  
- URDF获得
  - 大公司的机器人，可通过官网或第三方网站获得
  - 自己设计的机器人，可以借助sw2urdf插件导出
  
- 将URDF转为XML文件（即MJCF）

  在mujoco的bin目录下有一个compile程序。

  `./compile model.urdf model.xml` 将其转为xml

  `./compile model.urdf model.txt` 将其转为txt

  `./compile model.urdf model.mjb` 将其转为mjb

- URDF和MJCF的区别

  MJCF相比于URDF有如下要求。（第三点为本质区别）

  1. 根元素mujoco,属性model为模型名
  2. 元素asset定义所有mesh为子元素
  3. 机器人全部建立在worldbody元素中
  4. 一个body包含子元素inertial、joint等。

## XML结构

[参考网站](https://mujoco.readthedocs.io/en/latest/XMLreference.html)

XML主要由3部分组成

- `<asset>`用<mesh>tag导入STL文件
- `<worldbody>`用<body>tag定义了所有的模拟器组件，包括灯光、地板以及机器人
- `<acutator>`：**定义可以执行运动的关节**。定义的顺序需要按照运动学顺序来，比如多关节串联机器人以工具坐标附近的最后一个关节为joint0，依此类推。

```xml
<!--根元素mujoco,属性modelel为模型名-->
<mujoco model="example">
    <!-- set some defaults for units and lighting -->
    <compiler angle="radian" meshdir="meshes"/>

    <!-- 导入STL文件 -->
    <asset>
        <mesh file="base.STL" />
        <mesh file="link1.STL" />
        <mesh file="link2.STL" />
    </asset>

    <!-- 定义所有模拟器组件 -->
    <worldbody>
        <!-- 灯光 -->
        <light directional="true" pos="-0.5 0.5 3" dir="0 0 -1" />
        <!-- 添加地板，这样我们就不会凝视深渊 -->
        <geom name="floor" pos="0 0 0" size="1 1 1" type="plane" rgba="1 0.83 0.61 0.5"/>
        <!-- the ABR Control Mujoco interface expects a hand mocap -->
        <body name="hand" pos="0 0 0" mocap="true">
            <geom type="box" size=".01 .02 .03" rgba="0 .9 0 .5" contype="2"/>
        </body>

        <!-- 构建串联机器人 -->
        <!--name: 实体名，pos: 与上一个实体的偏移量(推荐在body上设置，后面的pos都可以写作"0,0,0"，便于调试) (注意是相对位置！) -->
        <body name="base" pos="0 0 0">
            <!-- 定义其几何特性。引用导入的shoulder.STL模型并命名，可以用euler来旋转STL以实现两个实体间的配合 -->
            <geom name="link0" type="mesh" mesh="base" pos="0 0 0"/>
            <!-- 定义实体的惯性。如果不写inertial其惯性会从geom中推断出来 -->
            <inertial pos="0 0 0" mass="0" diaginertia="0 0 0"/>

            <!-- nest筑巢 each child piece inside the parent body tags -->
            <body name="link1" pos="0 0 1">
                <!-- this joint connects link1 to the base -->
                <!-- name: 关节名，pos: 与上一个实体的偏移量 -->
                <joint name="joint0" axis="0 0 1" pos="0 0 0"/>

                <geom name="link1" type="mesh" mesh="link1" pos="0 0 0" euler="0 3.14 0"/>
                <inertial pos="0 0 0" mass="0.75" diaginertia="1 1 1"/>

                <body name="link2" pos="0 0 1">
                    <!-- this joint connects link2 to link1 -->
                    <joint name="joint1" axis="0 0 1" pos="0 0 0"/>

                    <geom name="link2" type="mesh" mesh="link2" pos="0 0 0" euler="0 3.14 0"/>
                    <inertial pos="0 0 0" mass="0.75" diaginertia="1 1 1"/>

                    <!-- the ABR Control Mujoco interface uses the EE body to -->
                    <!-- identify the end-effector point to control with OSC-->
                    <body name="EE" pos="0 0.2 0.2">
                        <inertial pos="0 0 0" mass="0" diaginertia="0 0 0" />
                    </body>
                </body>
            </body>
        </body>
    </worldbody>

    <!-- 定义关节上的执行器 -->
    <actuator>
        <motor name="joint0_motor" joint="joint0"/>
        <motor name="joint1_motor" joint="joint1"/>
    </actuator>

</mujoco>
```

## MuJoCo控制器

- [mjdata结构](https://mujoco.readthedocs.io/en/latest/APIreference.html?highlight=mjdata#mjdata)中各种量的定义。

  - 其中有个mjtNum类型（即浮点数）的值ctrl，是机器人运动方程里的tao。可以通过设置这个值来施加控制
  - 所有actuator的控制输入都被保存在这个mjData.ctrl中
  - 所有的力输出都保存在mjData.actuator_force中
  - 所有的激活状态都保存在mjData.act中

- 官方给出的控制向量

  `u = (mjData.ctrl, mjData.qfrc_applied, mjData.xfrc_applied)`

  - ctrl:控制器的输入，单位是N.m，即一个力矩
  - qfrc_applied:关节空间施加的外力
  - xfrc_applied:笛卡尔空间施加的外力

  我们主要需要改动ctrl。后面两个量在考虑扰动时加入。

- 在xml中定义关节的控制范围

  ```xml
  <actuator>
      <!--对应于每个关节的电机-->
      <!--ctrllimted 说明控制是有范围的，这个范围由ctrlrange定义,gear是传动比-->
      <!--joint告知是哪一个关节，name告知这个关节的电机名字是什么-->
      <motor ctrllimted="true" ctrlrange="-150.0 150.0" joint="shoulde_pan_joint" name="torq_j1" gear= "101"/>
  	<motor ctrllimted="true" ctrlrange="-28.0 28.0" joint="wirst_1_joint" name="torq_j2"/>
  </actuator>
  ```

  可以直接在代码中将这个范围读出来

  ```python
  from mujoco_py import load_model_from_path
  ## robot.xml即模型的MJCF文件
  model = load_model_from_path('robot.xml')
  print(model.actuator_ctrlrange)
  ```

- sim.step()来进行步进式仿真（执行一次，计算一次）

  有三种[step()](https://mujo co.readthedocs.io/en/latest/APIreference.html?highlight=mj_step#mj-step)

  ```python
  sim=muijoco_py.MjSim(model)
  sim.step()
  ```

- 结合上面例子：

  ```python
  import mujoco_py as mp
  model = mp.load_model_from_path('ur5.xml')
  sim = mp.MjSim(model)
  ## MjViewer基于openGL实现，记录仿真，可用于展示仿真结果
  viewer = mp.MjViewer(sim)
  
  for i in range(3000):
      ## 每次循环给出1个控制输入（这里设置的控制是瞎写的）
      sim.data.crtl[:6] = 1
      sim.step()
      ## render显示仿真过程
      viewer.render()
  ```

  

## 使用mujoco-py

[github](https://github.com/openai/mujoco-py),[文档](https://openai.github.io/mujoco-py/build/html/index.html)

小模版

```python
from mujoco_py import load_model_from_path, MjSim
class my_env():
    def __init__(self, env, args):
        # super(lab_env, self).__init__(env)
        # 导入xml文档
        self.model = load_model_from_path("your_XML_path.xml")
        # 调用MjSim构建一个basic simulation
        self.sim = MjSim(model=self.model)

    def get_state(self, *args):
        self.sim.get_state()
        # 如果定义了相机
        # self.sim.data.get_camera_xpos('[camera name]')

    def reset(self, *args):
        self.sim.reset()

    def step(self, *args):
        self.sim.step()
```

## 使用C

- 总结：

  - 改变相机的视角：`cam.azimuth`等等
  - 改变重力加速度：`m->opt.gravity`
  - 展示坐标轴：`opt.frame`
  - 设置初始位置/速度：`d->qpos, d->qvel`
  - 添加阻力：`d->qfrc_applied`

- 代码

  ```c++
  #include<stdbool.h> //for bool
  //#include<unistd.h> //for usleep
  //#include <math.h>
  
  #include "mujoco.h"
  #include "glfw3.h"
  #include "stdio.h"
  #include "stdlib.h"
  #include "string.h"
  
  
  char filename[] = "../myproject/course/ball.xml";
  
  // MuJoCo data structures
  mjModel* m = NULL;                  // MuJoCo model
  mjData* d = NULL;                   // MuJoCo data
  mjvCamera cam;                      // abstract camera
  mjvOption opt;                      // visualization options
  mjvScene scn;                       // abstract scene
  mjrContext con;                     // custom GPU context
  
  // mouse interaction
  bool button_left = false;
  bool button_middle = false;
  bool button_right =  false;
  double lastx = 0;
  double lasty = 0;
  
  // holders of one step history of time and position to calculate dertivatives
  mjtNum position_history = 0;
  mjtNum previous_time = 0;
  
  // controller related variables
  float_t ctrl_update_freq = 100;
  mjtNum last_update = 0.0;
  mjtNum ctrl;
  
  // keyboard callback
  void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
  {
      // backspace: reset simulation
      if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
      {
          mj_resetData(m, d);
          mj_forward(m, d);
      }
  }
  
  // mouse button callback
  void mouse_button(GLFWwindow* window, int button, int act, int mods)
  {
      // update button state
      button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
      button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
      button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);
  
      // update mouse position
      glfwGetCursorPos(window, &lastx, &lasty);
  }
  
  
  // mouse move callback
  void mouse_move(GLFWwindow* window, double xpos, double ypos)
  {
      // no buttons down: nothing to do
      if( !button_left && !button_middle && !button_right )
          return;
  
      // compute mouse displacement, save
      double dx = xpos - lastx;
      double dy = ypos - lasty;
      lastx = xpos;
      lasty = ypos;
  
      // get current window size
      int width, height;
      glfwGetWindowSize(window, &width, &height);
  
      // get shift key state
      bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);
  
      // determine action based on mouse button
      mjtMouse action;
      if( button_right )
          action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
      else if( button_left )
          action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
      else
          action = mjMOUSE_ZOOM;
  
      // move camera
      mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
  }
  
  
  // scroll callback
  void scroll(GLFWwindow* window, double xoffset, double yoffset)
  {
      // emulate vertical mouse motion = 5% of window height
      mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
  }
  
  
  // main function
  int main(int argc, const char** argv)
  {
      // activate software
      mj_activate("mjkey.txt");
  
  
      // load and compile model
      char error[1000] = "Could not load binary model";
  
      // check command-line arguments
      if( argc<2 )
          m = mj_loadXML(filename, 0, error, 1000);
  
      else
          if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
              m = mj_loadModel(argv[1], 0);
          else
              m = mj_loadXML(argv[1], 0, error, 1000);
      if( !m )
          mju_error_s("Load model error: %s", error);
  
      // make data
      d = mj_makeData(m);
  
  
      // init GLFW
      if( !glfwInit() )
          mju_error("Could not initialize GLFW");
  
      // create window, make OpenGL context current, request v-sync
      GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
      glfwMakeContextCurrent(window);
      glfwSwapInterval(1);
  
      // initialize visualization data structures
      mjv_defaultCamera(&cam);
      mjv_defaultOption(&opt);
      mjv_defaultScene(&scn);
      mjr_defaultContext(&con);
      mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
      mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context
  
      // install GLFW mouse and keyboard callbacks
      glfwSetKeyCallback(window, keyboard);
      glfwSetCursorPosCallback(window, mouse_move);
      glfwSetMouseButtonCallback(window, mouse_button);
      glfwSetScrollCallback(window, scroll);
  
      //用来设置相机的视角
      double arr_view[] = {90, -45, 4, 0.000000, 0.000000, 0.000000};
      cam.azimuth = arr_view[0];
      cam.elevation = arr_view[1];
      cam.distance = arr_view[2];
      cam.lookat[0] = arr_view[3];
      cam.lookat[1] = arr_view[4];
      cam.lookat[2] = arr_view[5];
      
      // m->opt.gravity[2]=-1;
      // qpos is dim nqx1 = 7x1; 3 translations + 4 quaternions
      d->qpos[2]=0.1;
      d->qvel[2]=5;
      d->qvel[0]=2;
      // use the first while condition if you want to simulate for a period.
      while( !glfwWindowShouldClose(window))
      {
          // advance interactive simulation for 1/60 sec
          //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
          //  this loop will finish on time for the next frame to be rendered at 60 fps.
          //  Otherwise add a cpu timer and exit this loop when it is time to render.
          mjtNum simstart = d->time;
          while( d->time - simstart < 1.0/60.0 )
          {
              mj_step(m, d);
              
              //设置摩擦力
              //文档中查找mjData的qfrc_applied, 这里v是velocity定义在mjModel中
              //drag force = -c*v^2*unit_vector(v); v=sqrt(vx^2+vy^2+vz^2)
              //vector(v) = vx i + vy j + vz k
              //unit_vector(v) = vector(v)/v
              //fx = -c*v*vx;
              //fy = -c*v*vy;
              //fz = -c*v*vz;
              double vx, vy, vz;
              vx = d->qvel[0];
              vy = d->qvel[1];
              vz = d->qvel[2];
              double v;
              v = sqrt(vx*vx + vy*vy + vz*vz);
              double fx, fy, fz;
              double c=1;
  
              d->qfrc_applied[0]=fx;
              d->qfrc_applied[1]=fy;
              d->qfrc_applied[2]=fz;
          }
  
          // get framebuffer viewport
          mjrRect viewport = {0, 0, 0, 0};
          glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
  
          // update scene and render
          // 设置物体上的坐标轴frame
          opt.frame = mjFRAME_WORLD;
          // 视角随着物体X轴运动
          cam.lookat[0] = d->qpos[0];
          mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
          mjr_render(viewport, &scn, &con);
          //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);
  
          // swap OpenGL buffers (blocking call due to v-sync)
          glfwSwapBuffers(window);
  
          // process pending GUI events, call GLFW callbacks
          glfwPollEvents();
  
      }
  
      // free visualization storage
      mjv_freeScene(&scn);
      mjr_freeContext(&con);
  
      // free MuJoCo model and data, deactivate
      mj_deleteData(d);
      mj_deleteModel(m);
      mj_deactivate();
  
      // terminate GLFW (crashes with Linux NVidia drivers)
      #if defined(__APPLE__) || defined(_WIN32)
          glfwTerminate();
      #endif
  
      return 1;
  }
  
  ```




#  单个pendulum摇摆的模拟

## 1.总结;

- 施加控制`mjcb_control`
- 获得传感器参数：`d->sensordata[0]`
  - 在xml中加入传感器<sensor>。。。</sensor>

下面3个控制函数自己定义：

- torque控制：`set_torque_control`
  - 在xml加在<actuator>中：<motor ... />
  - 扭矩输入：`d->ctrl[0] = -10*(d->sensordata[0]-0)-1*d->sensordata[1];`
- position控制：`set_position_servo`（pd控制中的p）
  - 在xml加在<actuator>中：<position ... />
- velocity控制：`set_velocity_servo`  (pd控制中的d)
  - 在xml加在<actuator>中：<velocity ... />

## 2. XML文件

```xml
<mujoco>
	<option>
		<flag sensornoise="enable" />
	</option>
	<worldbody>
        <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
		<geom type="plane" size="1 1 0.1" rgba=".9 0 0 1"/>
		<body pos="0 0 2" euler="0 30 0">
			<joint name="pin" type="hinge" axis = "0 -1 0" pos="0 0 0.5"/>
			<geom type="cylinder" size="0.05 0.5" rgba="0 .9 0 1" mass="1"/>
		</body>
	</worldbody>
	<actuator>
		<motor joint="pin" name="torque" gear="1" ctrllimited="true" ctrlrange="-100 100" />
		<!--PD control-->
		<position name="position_servo" joint="pin" kp="10" />
		<velocity name="velocity_servo" joint="pin" kv="0" />
	</actuator>
	<sensor>
		<jointpos joint="pin" noise="0.2"/>
		<jointvel joint="pin" noise="1" />
	</sensor>
</mujoco>

```

## 3.c文件

```c
#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>

#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"


char filename[] = "../myproject/control_pendulum/pendulum.xml";

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

//*************************************
// 下面3个函数，分别设置3个actuator的属性值
void set_torque_control(const mjModel* m, int actuator_no, int flag)
{

  if (flag==0)
    m->actuator_gainprm[10*actuator_no+0] = 0;
  else
    m->actuator_gainprm[10*actuator_no+0] = 1;

}
//***********************************


//*************************************
void set_velocity_servo(const mjModel* m, int actuator_no, double kv)
{
  m->actuator_gainprm[10*actuator_no+0] = kv;
  // xml文档查看actuator/velocity
  // 可发现，biastype 是 0 0 -kv,即第三个参数才是偏离控制参数。所以这里是加2而不是+1
  m->actuator_biasprm[10*actuator_no+2] = -kv;
}
//***********************************


//*************************************
void set_position_servo(const mjModel* m, int actuator_no, double kp)
{
  m->actuator_gainprm[10*actuator_no+0] = kp;
  // xml文档查看actuator/velocity
  // 可发现，biastype 是 0 0 -kv,即第三个参数才是偏离控制参数。所以这里是加1
  m->actuator_biasprm[10*actuator_no+1] = -kp;
}
//***********************************

// const 表示该变量不能再被修改
void mycontroller(const mjModel* m, mjData* d)
{
  int i;
  int actuator_no;
  //0 = torque actuator
  actuator_no = 0;
  int flag = 1;
  set_torque_control(m, actuator_no, flag);
  //PD control，这里P是10，D是1    d->ctrl[0] = -10*(d->qpos[0]-0)-1*d->qvel[0];
  //获得传感器的值    
  d->ctrl[0] = -10*(d->sensordata[0]-0)-1*d->sensordata[1];

  //1=position servo
  actuator_no = 1;
  double kp = 0;
  set_position_servo(m, actuator_no, kp);
/*下面的for循环用于
获得actuator的参数值，每个actuator最多10个参数由mjNGAIN设定。
position servo是第二个actuator，所以从第11个开始是他的参数
*/
//   for (i=0;i<10;i++)
//   {
//     printf("%f \n", m->actuator_gainprm[10*actuator_no+i]);
//     printf("%f \n", m->actuator_biasprm[10*actuator_no+i]);
//   }

//   printf("*********** \n");
//   d->ctrl[1] = 0.5;

  //2= velocity servo
  actuator_no = 2;
  double kv = 0;
  set_velocity_servo(m, actuator_no, kv);
  d->ctrl[2] = 0.2;

  //PD control
//   actuator_no = 1;
//   double kp2 = 10;
//   set_position_servo(m, actuator_no, kp2);
//   actuator_no = 2;
//   double kv2 = 1;
//   set_velocity_servo(m, actuator_no, kv2);
//   d->ctrl[1] = -0.5;
//   d->ctrl[2] = 0;


}

// main function
int main(int argc, const char** argv)
{

    // activate software
    mj_activate("mjkey.txt");


    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(filename, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);


    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {90, -5, 5, 0.012768, -0.000000, 1.254336};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    d->qpos[0]=1.57; //pi/2
    // 文档搜mjcb_control
    // 在mycontroller函数中定义自己的control law
    mjcb_control = mycontroller;

    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
        }

       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

          // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}

```



# 2个pendulum摇摆的模拟

## 1. 总结：

- xml种option标签里设置时间步`timestep`积分器`integrator`能量测量开个`enable`

- 测量动能：`mj_energyVel(m,d)`

  - 存在mjData的energy[1]里

- 测量势能：`mj_energyPos(m,d)`

  - 存在mjData的energy[0]里

- 建立动力学方程

  M+C+G=TAO

  - M：稀疏惯性矩阵转为稠密惯性矩阵`mj_fullM`
  - C：d->qfrc_applied[1] = 0.5*f[1];
  - G：d->qfrc_applied[0] = 0.1*f[0];

- 三种控制器实现

  - PD control
  - coriolis + gravity + PD control
  - Feedback linearization

- 读取数据

  - 代码查看关键字fid

## 2.XML文件

```xml
<mujoco>
	<option timestep="0.0001" integrator="RK4">
		<flag sensornoise="enable" energy="enable" contact="disable"/>
	</option>
	<worldbody>
		<light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
		<geom type="plane" size="1 1 0.1" rgba=".9 0 0 1"/>
		<body pos="0 0 2.5" euler="0 0 0">
			<joint name="pin" type="hinge" axis="0 -1 0" pos="0 0 -0.5"/>
			<geom type="cylinder" size="0.05 0.5" rgba="0 .9 0 1" mass="1"/>
			<body pos="0 0.1 1" euler="0 0 0">
				<joint name="pin2" type="hinge" axis="0 -1 0" pos="0 0 -0.5"/>
				<geom type="cylinder" size="0.05 0.5" rgba="0 0 .9 1" mass="1"/>
			</body>
		</body>
	</worldbody>
</mujoco>
```



## 3.C文件

```c++


#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>

#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

//simulation end time
double simend = 5;

//related to writing data to a file
FILE *fid;
int loop_index = 0;
const int data_frequency = 50; //frequency at which data is written to a file


char xmlpath[] = "../myproject/dbpendulum/doublependulum.xml";
char datapath[] = "../myproject/dbpendulum/data.csv";


//Change the path <template_writeData>
//Change the xml file
// char path[] = "../myproject/dbpendulum/";
// char xmlfile[] = "doublependulum.xml";


char datafile[] = "data.csv";


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


//****************************
//This function is called once and is used to get the headers
void init_save_data()
{
  //write name of the variable here (header)
   fprintf(fid,"t, ");
   fprintf(fid,"PE, KE, TE, ");
   fprintf(fid,"q1, q2, ");

   //Don't remove the newline
   fprintf(fid,"\n");
}

//***************************
//This function is called at a set frequency, put data here
void save_data(const mjModel* m, mjData* d)
{
  //data here should correspond to headers in init_save_data()
  //seperate data by a space %f followed by space
  fprintf(fid,"%f, ",d->time);
  fprintf(fid,"%f, %f, %f, ",d->energy[0],d->energy[1],d->energy[0]+d->energy[1]);
  fprintf(fid,"%f, %f ",d->qpos[0],d->qpos[1]);
  //Don't remove the newline
  fprintf(fid,"\n");
}

//**************************
void mycontroller(const mjModel* m, mjData* d)
{
  //write control here
  mj_energyPos(m,d);
  mj_energyVel(m,d);
  printf("%f %f %f %f \n",d->time,d->energy[0],d->energy[1],d->energy[0]+d->energy[1]);

  /*check equations机器人动力学方程
  //M*qacc + qfrc_bias = qfrc_applied + ctrl
  //M*qddot + f = qfrc_applied + ctrl
  api文档查看mj_fullM可以将稀疏惯性矩阵转为稠密矩阵。系数矩阵：其元素大部分为0，稠密矩阵：大部分元素非0
  nv is the size of the system
  */
  #define nv 2
  double dense_M[nv*nv] = {0};
  //d-qM:所有的惯性即稀疏矩阵中非零项的个数，dense_M稠密矩阵，m稀疏矩阵
  mj_fullM(m,dense_M, d->qM);
  double M[nv][nv]={0};
  M[0][0] = dense_M[0];
  M[0][1] = dense_M[1];
  M[1][0] = dense_M[2];
  M[1][1] = dense_M[3];
//   printf("%f %f \n",M[0][0],M[0][1]);
//   printf("%f %f \n",M[1][0],M[1][1]);
//   printf("******** \n");
  
  //加速度
  double qddot[nv]={0};
  qddot[0]=d->qacc[0];
  qddot[1]=d->qacc[1];
  
  //0是qpos,1是qvel。
  double f[nv]={0};
  f[0] = d->qfrc_bias[0];
  f[1] = d->qfrc_bias[1];

  double lhs[nv]={0};
  //mju_mulMatVec矩阵乘以向量。这里惯性矩阵乘以加速度
  mju_mulMatVec(lhs,dense_M,qddot,2,2); //lhs = M*qddot
  lhs[0] = lhs[0] + f[0]; //lhs = M*qddot + f
  lhs[1] = lhs[1] + f[1];
  
  //G,重力项
  d->qfrc_applied[0] = 0.1*f[0];
  //c,各种阻力项
  d->qfrc_applied[1] = 0.5*f[1];

  double rhs[nv]={0};
  rhs[0] = d->qfrc_applied[0];
  rhs[1] = d->qfrc_applied[1];

  // printf("%f %f \n",lhs[0], rhs[0]);
  // printf("%f %f \n",lhs[1], rhs[1]);
  // printf("******\n");

  //control
  double Kp1 = 100, Kp2 = 100;
  double Kv1 = 10, Kv2 = 10;
  //参考位置
  double qref1 = -0.5, qref2 = -1.6;
  
  //3种控制器的实现
  //PD control
  // d->qfrc_applied[0] = -Kp1*(d->qpos[0]-qref1)-Kv1*d->qvel[0];
  // d->qfrc_applied[1] = -Kp2*(d->qpos[1]-qref2)-Kv2*d->qvel[1];

  //coriolis + gravity + PD control
  // d->qfrc_applied[0] = f[0]-Kp1*(d->qpos[0]-qref1)-Kv1*d->qvel[0];
  // d->qfrc_applied[1] = f[1]-Kp2*(d->qpos[1]-qref2)-Kv2*d->qvel[1];

  //Feedback linearization
  //M*(-kp( ... ) - kv(...) + f)
  double tau[2]={0};
  tau[0]=-Kp1*(d->qpos[0]-qref1)-Kv1*d->qvel[0];
  tau[1]=-Kp2*(d->qpos[1]-qref2)-Kv2*d->qvel[1];

  mju_mulMatVec(tau,dense_M,tau,2,2); //lhs = M*tau
  tau[0] += f[0];
  tau[1] += f[1];
    d->qfrc_applied[0] = tau[0];
    d->qfrc_applied[1] = tau[1];





  //write data here (dont change/dete this function call; instead write what you need to save in save_data)
  if ( loop_index%data_frequency==0)
    {
      save_data(m,d);
    }
  loop_index = loop_index + 1;
}


//************************
// main function
int main(int argc, const char** argv)
{

    // activate software
    mj_activate("mjkey.txt");

    // char xmlpath[100]={};
    // char datapath[100]={};
    //
    // strcat(xmlpath,path);
    // strcat(xmlpath,xmlfile);
    //
    // strcat(datapath,path);
    // strcat(datapath,datafile);


    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(xmlpath, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);


    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {89.608063, -11.588379, 5, 0.000000, 0.000000, 2.000000};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    // install control callback
    mjcb_control = mycontroller;

    fid = fopen(datapath,"w");
    init_save_data();

    d->qpos[0] = 0.5;
    //d->qpos[1] = 0;
    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
        }

        if (d->time>=simend)
        {
           fclose(fid);
           break;
         }

       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

          // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}

```

