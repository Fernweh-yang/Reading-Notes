# 如何卸载库

## 1. 对于make install

- 方法一：

  **build目录下，执行`make uninstall`。**

  make的原理是执行Makefile文件里的指令，make的基本用处是自动根据makefile里的指令来编译源文件。它还可以用来做比如安装软件，卸载软件等事情，但前提是作者在makefile里写了。

  然后用make install的话，make程序就会按照上面install：后面的指令< commands >执行安装，uninstall也是一样的道理，大部分的作者会写有卸载的部分，这时只要简单地执行make unistall就可以；如果作者懒没有写，可参考方法二，自动查找删除已安装文件。

- 方法二：

  **build目录下，执行` xargs rm < install_manifest.txt`**

  make install之后，build目录下会有一个install_mainfest.txt的文件, 记录了安装的所有内容及路径，

  执行 xargs rm < install_manifest.txt 就可以了。

  如果没有这个文件，可以自己重新make install，从log中过滤出install的安装路径信息，保存到unistall.txt中，再执行xargs rm < unistall.txt即可。

# Eigen库

[官方教程](https://eigen.tuxfamily.org/dox/GettingStarted.html)

Eigen是**可以用来进行线性代数、矩阵、向量操作等运算的C++库**

## 0. 安装

- 下载源码: `http://eigen.tuxfamily.org/index.php?title=Main_Page`

- 安装

  ```
  cd ~/
  mkdir 3rd_library
  cd ~/Download
  mv eigen-3.4.0 ../3rd_library/
  cd ~/3rd_library/eigen-3.4.0
  mkdir build
  cd build
  cmake..
  sudo make install
  ```

## 1.基本使用

### 代码

```c++
#include <iostream>

using namespace std;

#include <ctime>
// Eigen 核心部分
#include <eigen3/Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <eigen3/Eigen/Dense>

using namespace Eigen;

#define MATRIX_SIZE 50

/****************************
* 本程序演示了 Eigen 基本类型的使用
****************************/

int main(int argc, char **argv) {
  // Eigen 中所有向量和矩阵都是Eigen::Matrix，它是一个模板类。它的前三个参数为：数据类型，行，列
  // 声明一个2*3的float矩阵
  Matrix<float, 2, 3> matrix_23;

  // 同时，Eigen 通过 typedef 提供了许多内置类型，不过底层仍是Eigen::Matrix
  // 例如 Vector3d 实质上是 Eigen::Matrix<double, 3, 1>，即三维向量
  Vector3d v_3d;
  // 这是一样的
  Matrix<float, 3, 1> vd_3d;

  // Matrix3d 实质上是 Eigen::Matrix<double, 3, 3>
  Matrix3d matrix_33 = Matrix3d::Zero(); //初始化为零
  // 如果不确定矩阵大小，可以使用动态大小的矩阵
  Matrix<double, Dynamic, Dynamic> matrix_dynamic;
  // 更简单的
  MatrixXd matrix_x;
  // 这种类型还有很多，我们不一一列举

  // 下面是对Eigen阵的操作
  // 输入数据（初始化）
  matrix_23 << 1, 2, 3, 4, 5, 6;
  // 输出
  cout << "matrix 2x3 from 1 to 6: \n" << matrix_23 << endl;

  // 用()访问矩阵中的元素
  cout << "print matrix 2x3: " << endl;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3; j++) cout << matrix_23(i, j) << "\t";
    cout << endl;
  }

  // 矩阵和向量相乘（实际上仍是矩阵和矩阵）
  v_3d << 3, 2, 1;
  vd_3d << 4, 5, 6;

  // 但是在Eigen里你不能混合两种不同类型的矩阵，像这样是错的
  // Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;
  // 应该使用cast()将float显式转换为double
  Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
  cout << "[1,2,3;4,5,6]*[3,2,1]=" << result.transpose() << endl;

  Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
  cout << "[1,2,3;4,5,6]*[4,5,6]: " << result2.transpose() << endl;

  // 同样你不能搞错矩阵的维度
  // 试着取消下面的注释，看看Eigen会报什么错
  // Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix_23.cast<double>() * v_3d;

  // 一些矩阵运算
  // 四则运算就不演示了，直接用+-*/即可。
  matrix_33 = Matrix3d::Random();      // 随机数矩阵
  cout << "random matrix: \n" << matrix_33 << endl;
  cout << "transpose: \n" << matrix_33.transpose() << endl;      // 转置
  cout << "sum: " << matrix_33.sum() << endl;            // 各元素和
  cout << "trace: " << matrix_33.trace() << endl;          // 迹
  cout << "times 10: \n" << 10 * matrix_33 << endl;               // 数乘
  cout << "inverse: \n" << matrix_33.inverse() << endl;        // 逆
  cout << "det: " << matrix_33.determinant() << endl;    // 行列式

  // 特征值
  // 实对称矩阵可以保证对角化成功
  // 如有n阶矩阵A，其矩阵元素都为实数，且矩阵A的转置等于其本身aij=aji，则称A为实对称矩阵
  SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
  cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl;
  cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << endl;

  // 解方程
  // 我们求解 matrix_NN * x = v_Nd 这个方程
  // N的大小在前边的宏里定义，它由随机数生成
  // 直接求逆自然是最直接的，但是求逆运算量大

  Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN
      = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
  matrix_NN = matrix_NN * matrix_NN.transpose();  // 保证半正定，半正定矩阵属于实对称矩阵
  Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);

  clock_t time_stt = clock(); // 计时
  // 直接求逆
  Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
  cout << "time of normal inverse is "
       << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
  cout << "x = " << x.transpose() << endl;

  // 通常用矩阵分解来求，例如QR分解，速度会快很多
  time_stt = clock();
  x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
  cout << "time of Qr decomposition is "
       << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
  cout << "x = " << x.transpose() << endl;

  // 对于正定矩阵，还可以用cholesky分解来解方程
  // 半正定矩阵：给定一个大小为nXn的实对称矩阵A，若对于任意长度为n的向量x,有xTAx>=0恒成立，则矩阵A是一个半正定矩阵。
  // 正定矩阵：给定一个大小为nXn的实对称矩阵A，若对于任意长度为n的非零向量x,有xTAx>0恒成立，则矩阵A是一个正定矩阵。
  // 半正定矩阵包括了正定矩阵。
  time_stt = clock();
  x = matrix_NN.ldlt().solve(v_Nd);
  cout << "time of ldlt decomposition is "
       << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
  cout << "x = " << x.transpose() << endl;

  return 0;
}
```

### 编译运行

CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 2.8)
project(useEigen)

set(CMAKE_BUILD_TYPE "Release")
# CMAKE_CXX_FLAGS是CMake传给c++编译器的编译选项，就好比g++ -03
# CMAKE_CXX_FLAGS_DEBUG,是除了CMAKE_CXX_FLAGS外，在Debug配置下，额外的参数
# CMAKE_CXX_FLAGS_RELEASE，是除了CMAKE_CXX_FLAGS外，在Release配置下，额外的参数
set(CMAKE_CXX_FLAGS "-O3")

# 添加Eigen头文件
# Eigen库只有头文件没有库文件，所以不需要target_link_libraries.
# 这么做的坏处：如果把Eigen安装在了不同位置，就必须要手动修改头文件目录。
include_directories("/usr/include/eigen3")
add_executable(eigenMatrix eigenMatrix.cpp)
```

编译运行

```
mkdir build
cd build 
cmake ..
make 
./eigenMatrix
```

## 2.几何模块基本使用

[官方教程](https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html)

### 代码

```c++
#include <iostream>
#include <cmath>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

// 本程序演示了 Eigen 几何模块的使用方法

int main(int argc, char **argv) {

  // Eigen/Geometry 模块提供了各种旋转和平移的表示
  // 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f，d是double，f是float
  // Identity()用单位矩阵对新变量进行初始化
  Matrix3d rotation_matrix = Matrix3d::Identity();
  // 旋转向量使用 AngleAxis, 它底层不直接是Matrix，但运算可以当作矩阵（因为重载了运算符）
  AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));     //沿 Z 轴旋转 45 度
  cout.precision(3);
  cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;   //用matrix()转换成矩阵
  // 也可以直接赋值
  rotation_matrix = rotation_vector.toRotationMatrix();
  // 用 AngleAxis 可以进行坐标变换
  Vector3d v(1, 0, 0);
  Vector3d v_rotated = rotation_vector * v;
  //vector.transpose()输出向量/矩阵的转置
  cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;
  // 或者用旋转矩阵
  v_rotated = rotation_matrix * v;
  cout << "(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() << endl;

  // 欧拉角: 可以将旋转矩阵直接转换成欧拉角
  Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX顺序，即yaw-pitch-roll顺序
  cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

  // 欧氏变换矩阵使用 Eigen::Isometry
  Isometry3d T = Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
  T.rotate(rotation_vector);                                     // 按照rotation_vector进行旋转
  T.pretranslate(Vector3d(1, 3, 4));                     // 把平移向量设成(1,3,4)
  cout << "Transform matrix = \n" << T.matrix() << endl;

  // 用变换矩阵进行坐标变换
  Vector3d v_transformed = T * v;                              // 相当于R*v+t
  cout << "v tranformed = " << v_transformed.transpose() << endl;

  // 对于仿射和射影变换，使用 Eigen::Affine3d 和 Eigen::Projective3d 即可，略

  // 四元数
  // 可以直接把AngleAxis赋值给四元数，反之亦然
  Quaterniond q = Quaterniond(rotation_vector);
  // coeffs()函数返回四元素的四个参数。
  cout << "quaternion from rotation vector = " << q.coeffs().transpose()
       << endl;   // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
  // 也可以把旋转矩阵赋给它
  q = Quaterniond(rotation_matrix);
  cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << endl;
  // 使用四元数旋转一个向量，使用重载的乘法即可
  v_rotated = q * v; // 注意数学上是qvq^{-1}
  cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
  // 用常规向量乘法表示，则应该如下计算
  cout << "should be equal to " << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;

  return 0;
}
```

### 编译运行

```cmake
cmake_minimum_required( VERSION 2.8 )
project( geometry )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

add_executable(eigenGeometry eigenGeometry.cpp)
```

## 3. 一个例子

### 问题描述

设有小萝卜1号和小萝卜2号位于世界坐标系中。记世界坐标系为W，小萝卜们的坐标系为R1和R2。

小萝卜1号的位姿为q1=[0.35, 0.2, 0.3, 0.1]T，t1=[0.3, 0.1, 0.1]T。

小萝卜2号的位姿为q2=[-0.5, 0.4, -0.1, 0.2]T，t2=[-0.1, 0.5, 0.3]T。

这里的q,t即T，表示世界坐标系到相机坐标系的变换关系。

现在小萝卜1号看到某个点在自身的坐标系下坐标为PR1=[0.4, 0, 0.2]T，求该向量在小萝卜2号坐标系下的坐标。

### 代码

```c++
#include <iostream>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv){
    Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);
    // 归一化2个四元数
    q1.normalize();
    q2.normalize();
    Vector3d t1(0.3, 0.1, 0.1), t2(-0.3, 0.5, 0.3);
    Vector3d p1(0.5, 0, 0.2);
    
    // 欧式变换矩阵
    Isometry3d T1w(q1),T2w(q2);
    // 设置变换矩阵中的位移t
    T1w.pretranslate(t1);
    T2w.pretranslate(t2);

    // 先从1转世界，再从世界转2坐标系
    Vector3d p2 = T2w*T1w.inverse()*p1;
    cout << endl << p2.transpose() <<endl;
    return 0;
}
```

## 4. EigenPy安装:

Eigen的python版本：[官网](https://github.com/stack-of-tasks/eigenpy)

```shell
# 1. Add robotpkg as source repository to apt:
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
# 2. Register the authentication certificate of robotpkg:
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
# 3. fetch the package descriptions:
sudo apt-get update
# 4. Install
sudo apt install robotpkg-py35-eigenpy
```

# Sophus库

[Github](https://github.com/strasdat/Sophus)

用于计算李代数

## 1.安装

1. 安装依赖库fmt

   ```shell
   #从git下载最新版本fmt
   cd fmt
   mkdir build 
   cd build
   cmake ..
   sudo make install
   ```
   
2. 安装Sophus库

   ```shell
   git clone git@github.com:strasdat/Sophus.git
   cd Sophus
   mkdir build
   cd build
   cmake ..
   sudo make install
   ```

3. 卸载

   ```shell
   xargs rm < install_manifest.txt
   ```

## 2. 基本使用

### 代码

```c++
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

/// 本程序演示sophus的基本用法

int main(int argc, char **argv) {

  // 沿Z轴转90度的旋转矩阵
  Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
  // 或者四元数
  Quaterniond q(R);
  Sophus::SO3d SO3_R(R);              // Sophus::SO3d可以直接从旋转矩阵构造
  Sophus::SO3d SO3_q(q);              // 也可以通过四元数构造
  // 二者是等价的
  cout << "SO(3) from matrix:\n" << SO3_R.matrix() << endl;
  cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << endl;
  cout << "they are equal" << endl;

  // 使用对数映射获得它的李代数
  Vector3d so3 = SO3_R.log();
  cout << "so3 = " << so3.transpose() << endl;
  // hat 为向量到反对称矩阵，即向上的大于号：从向量到矩阵
  cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << endl;
  // 相对的，vee为反对称到向量，即向下的大于号：从矩阵到向量
  cout << "so3 hat vee= " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

  // 增量扰动模型的更新
  // 书p83,85。dR*R在李代数的表示
  Vector3d update_so3(1e-4, 0, 0); //假设更新量为这么多
  Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
  cout << "SO3 updated = \n" << SO3_updated.matrix() << endl;

  cout << "*******************************" << endl;
  // 对SE(3)操作大同小异
  Vector3d t(1, 0, 0);           // 沿X轴平移1
  Sophus::SE3d SE3_Rt(R, t);           // 从R,t构造SE(3)
  Sophus::SE3d SE3_qt(q, t);            // 从q,t构造SE(3)
  cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << endl;
  cout << "SE3 from q,t= \n" << SE3_qt.matrix() << endl;
  // 李代数se(3) 是一个六维向量，方便起见先typedef一下
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d se3 = SE3_Rt.log();
  cout << "se3 = " << se3.transpose() << endl;
  // 观察输出，会发现在Sophus中，se(3)的平移在前，旋转在后.
  // 同样的，有hat和vee两个算符
  cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << endl;
  cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;

  // 最后，演示一下更新
  Vector6d update_se3; //更新量
  update_se3.setZero();
  // 李代数第一个量设置为0.0001
  update_se3(0, 0) = 1e-4;
  Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
  cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;

  return 0;
}
```

### 编译运行

CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.0)
project(useSophus)

# 为使用 sophus，需要使用find_package命令找到它
find_package(Sophus REQUIRED)

# Eigen
include_directories("/usr/include/eigen3")
add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus Sophus::Sophus)
```

## 3. 例子：评估轨迹的误差

### 代码

```c++
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>

using namespace Sophus;
using namespace std;

string groundtruth_file = "./example/groundtruth.txt";
string estimated_file = "./example/estimated.txt";

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);

TrajectoryType ReadTrajectory(const string &path);

int main(int argc, char **argv) {
  TrajectoryType groundtruth = ReadTrajectory(groundtruth_file);
  TrajectoryType estimated = ReadTrajectory(estimated_file);
  assert(!groundtruth.empty() && !estimated.empty());
  assert(groundtruth.size() == estimated.size());

  // compute rmse
  double rmse = 0;
  for (size_t i = 0; i < estimated.size(); i++) {
    Sophus::SE3d p1 = estimated[i], p2 = groundtruth[i];
    double error = (p2.inverse() * p1).log().norm();
    rmse += error * error;
  }
  rmse = rmse / double(estimated.size());
  rmse = sqrt(rmse);
  cout << "RMSE = " << rmse << endl;

  DrawTrajectory(groundtruth, estimated);
  return 0;
}

TrajectoryType ReadTrajectory(const string &path) {
  ifstream fin(path);
  TrajectoryType trajectory;
  if (!fin) {
    cerr << "trajectory " << path << " not found." << endl;
    return trajectory;
  }

  while (!fin.eof()) {
    double time, tx, ty, tz, qx, qy, qz, qw;
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    Sophus::SE3d p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
    trajectory.push_back(p1);
  }
  return trajectory;
}

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
      .SetHandler(new pangolin::Handler3D(s_cam));


  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glLineWidth(2);
    for (size_t i = 0; i < gt.size() - 1; i++) {
      glColor3f(0.0f, 0.0f, 1.0f);  // blue for ground truth
      glBegin(GL_LINES);
      auto p1 = gt[i], p2 = gt[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }

    for (size_t i = 0; i < esti.size() - 1; i++) {
      glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
      glBegin(GL_LINES);
      auto p1 = esti[i], p2 = esti[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);   // sleep 5 ms
  }

}
```

### 编译运行

```cmake
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
add_executable(trajectoryError trajectoryError.cpp)
target_link_libraries(trajectoryError ${Pangolin_LIBRARIES})
target_link_libraries(trajectoryError Sophus::Sophus)
```



#  Pangolin库

用于3D绘图

## 1. 安装库

1. 从[官方代码库](https://github.com/stevenlovegrove/Pangolin)git下代码来。

2. 安装依赖:`sudo apt-get install libglew-dev`

3. 编译库

   ```
   cd [path-to-pangolin]
   mkdir build
   cd build
   cmake ..
   make 
   sudo make install 
   sudo ldconfig
   ```

## 2. 卸载库

```
make uninstall 
```

## 3. 示例代码

### 画一个已知的轨迹

#### 代码

```c++
#include <pangolin/pangolin.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <unistd.h>

// 本例演示了如何画出一个预先存储的轨迹

using namespace std;
using namespace Eigen;

// path to trajectory file
// 导入轨迹
string trajectory_file = "./examples/trajectory.txt";

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);

int main(int argc, char **argv) {

  vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
  ifstream fin(trajectory_file);
  if (!fin) {
    cout << "cannot find trajectory file at " << trajectory_file << endl;
    return 1;
  }

  while (!fin.eof()) {
    double time, tx, ty, tz, qx, qy, qz, qw;
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
    Twr.pretranslate(Vector3d(tx, ty, tz));
    poses.push_back(Twr);
  }
  cout << "read total " << poses.size() << " pose entries" << endl;

  // draw trajectory in pangolin
  DrawTrajectory(poses);
  return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);
    for (size_t i = 0; i < poses.size(); i++) {
      // 画每个位姿的三个坐标轴
      Vector3d Ow = poses[i].translation();
      Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
      Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
      Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));
      glBegin(GL_LINES);
      glColor3f(1.0, 0.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Xw[0], Xw[1], Xw[2]);
      glColor3f(0.0, 1.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Yw[0], Yw[1], Yw[2]);
      glColor3f(0.0, 0.0, 1.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Zw[0], Zw[1], Zw[2]);
      glEnd();
    }
    // 画出连线
    for (size_t i = 0; i < poses.size(); i++) {
      glColor3f(0.0, 0.0, 0.0);
      glBegin(GL_LINES);
      auto p1 = poses[i], p2 = poses[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);   // sleep 5 ms
  }
}
```

#### 编译运行

CMakeLists.txt

```cmake
include_directories("/usr/include/eigen3")

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
add_executable(plotTrajectory plotTrajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
```

命令行输入

```
mkdir build
cd build
cmake ..
make 

./plotTrajectory
```

### 可视化相机位姿的各种表达方式

#### 代码

```c++
#include <iostream>
#include <iomanip>

using namespace std;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;

#include <pangolin/pangolin.h>

struct RotationMatrix {
  Matrix3d matrix = Matrix3d::Identity();
};

ostream &operator<<(ostream &out, const RotationMatrix &r) {
  out.setf(ios::fixed);
  Matrix3d matrix = r.matrix;
  out << '=';
  out << "[" << setprecision(2) << matrix(0, 0) << "," << matrix(0, 1) << "," << matrix(0, 2) << "],"
      << "[" << matrix(1, 0) << "," << matrix(1, 1) << "," << matrix(1, 2) << "],"
      << "[" << matrix(2, 0) << "," << matrix(2, 1) << "," << matrix(2, 2) << "]";
  return out;
}

istream &operator>>(istream &in, RotationMatrix &r) {
  return in;
}

struct TranslationVector {
  Vector3d trans = Vector3d(0, 0, 0);
};

ostream &operator<<(ostream &out, const TranslationVector &t) {
  out << "=[" << t.trans(0) << ',' << t.trans(1) << ',' << t.trans(2) << "]";
  return out;
}

istream &operator>>(istream &in, TranslationVector &t) {
  return in;
}

struct QuaternionDraw {
  Quaterniond q;
};

ostream &operator<<(ostream &out, const QuaternionDraw quat) {
  auto c = quat.q.coeffs();
  out << "=[" << c[0] << "," << c[1] << "," << c[2] << "," << c[3] << "]";
  return out;
}

istream &operator>>(istream &in, const QuaternionDraw quat) {
  return in;
}

int main(int argc, char **argv) {
  pangolin::CreateWindowAndBind("visualize geometry", 1000, 600);
  glEnable(GL_DEPTH_TEST);
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1000, 600, 420, 420, 500, 300, 0.1, 1000),
    pangolin::ModelViewLookAt(3, 3, 3, 0, 0, 0, pangolin::AxisY)
  );

  const int UI_WIDTH = 500;

  pangolin::View &d_cam = pangolin::CreateDisplay().
    SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -1000.0f / 600.0f).
    SetHandler(new pangolin::Handler3D(s_cam));

  // ui
  pangolin::Var<RotationMatrix> rotation_matrix("ui.R", RotationMatrix());
  pangolin::Var<TranslationVector> translation_vector("ui.t", TranslationVector());
  pangolin::Var<TranslationVector> euler_angles("ui.rpy", TranslationVector());
  pangolin::Var<QuaternionDraw> quaternion("ui.q", QuaternionDraw());
  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);

    pangolin::OpenGlMatrix matrix = s_cam.GetModelViewMatrix();
    Matrix<double, 4, 4> m = matrix;

    RotationMatrix R;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        R.matrix(i, j) = m(j, i);
    rotation_matrix = R;

    TranslationVector t;
    t.trans = Vector3d(m(0, 3), m(1, 3), m(2, 3));
    t.trans = -R.matrix * t.trans;
    translation_vector = t;

    TranslationVector euler;
    euler.trans = R.matrix.eulerAngles(2, 1, 0);
    euler_angles = euler;

    QuaternionDraw quat;
    quat.q = Quaterniond(R.matrix);
    quaternion = quat;

    glColor3f(1.0, 1.0, 1.0);

    pangolin::glDrawColouredCube();
    // draw the original axis
    glLineWidth(3);
    glColor3f(0.8f, 0.f, 0.f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(10, 0, 0);
    glColor3f(0.f, 0.8f, 0.f);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 10, 0);
    glColor3f(0.2f, 0.2f, 1.f);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 10);
    glEnd();

    pangolin::FinishFrame();
  }
}
```

#### 编译运行

CMakeLists.txt

```c++
cmake_minimum_required( VERSION 2.8 )
project( visualizeGeometry )

set(CMAKE_CXX_FLAGS "-std=c++17")

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

# 添加Pangolin依赖
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( visualizeGeometry visualizeGeometry.cpp )
target_link_libraries( visualizeGeometry ${Pangolin_LIBRARIES} )
```

# OpenCV库

教程见笔记

## 1. 去畸变

- 代码

  test.cpp

  ```c++
  #include <opencv2/opencv.hpp>
  #include <string>
  
  using namespace std;
  
  string image_file = "./distorted.png";   // 请确保路径正确
  
  int main(int argc, char **argv) {
  
    // 本程序实现去畸变部分的代码。尽管我们可以调用OpenCV的去畸变，但自己实现一遍有助于理解。
    // 畸变参数
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    // 内参
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;
  
    cv::Mat image = cv::imread(image_file, 0);   // 图像是灰度图，CV_8UC1
    int rows = image.rows, cols = image.cols;
    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);   // 去畸变以后的图
  
    // 计算去畸变后图像的内容
    for (int v = 0; v < rows; v++) {
      for (int u = 0; u < cols; u++) {
        // 按照公式，计算点(u,v)对应到畸变图像中的坐标(u_distorted, v_distorted)
        double x = (u - cx) / fx, y = (v - cy) / fy;
        double r = sqrt(x * x + y * y);
        double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
        double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
        double u_distorted = fx * x_distorted + cx;
        double v_distorted = fy * y_distorted + cy;
  
        // 赋值 (最近邻插值)
        if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
          image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
        } else {
          image_undistort.at<uchar>(v, u) = 0;
        }
      }
    }
  
    // 画图去畸变后图像
    cv::imshow("distorted", image);
    cv::imshow("undistorted", image_undistort);
    cv::waitKey();
    return 0;
  }
  ```

- 编译:

  CMakeLists.txt

  ```
  cmake_minimum_required(VERSION 2.8)
  project( test )
  find_package( OpenCV REQUIRED )
  include_directories( ${OpenCV_INCLUDE_DIRS} )
  add_executable( test test.cpp )
  target_link_libraries( test ${OpenCV_LIBS} )
  ```

## 2. 双目视觉

实现视差图和点云图

- 代码:

  test.cpp

  ```c++
  #include <opencv2/opencv.hpp>
  #include <vector>
  #include <string>
  #include <Eigen/Core>
  #include <pangolin/pangolin.h>
  #include <unistd.h>
  
  using namespace std;
  using namespace Eigen;
  
  // 文件路径
  string left_file = "./left.png";
  string right_file = "./right.png";
  
  // 在pangolin中画图，已写好，无需调整
  void showPointCloud(
      const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud);
  
  int main(int argc, char **argv) {
  
      // 内参
      double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
      // 基线
      double b = 0.573;
  
      // 读取图像
      cv::Mat left = cv::imread(left_file, 0);
      cv::Mat right = cv::imread(right_file, 0);
      // 使用opencv自带的Semi-Global Batch Matching算法，计算左右图像的视察
      cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
          0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);    // 神奇的参数
      cv::Mat disparity_sgbm, disparity;
      sgbm->compute(left, right, disparity_sgbm);
      disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);
  
      // 生成点云
      vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;
  
      // 如果你的机器慢，请把后面的v++和u++改成v+=2, u+=2
      for (int v = 0; v < left.rows; v++)
          for (int u = 0; u < left.cols; u++) {
              if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0) continue;
  
              Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0); // 前三维为xyz,第四维为颜色
  
              // 根据双目模型计算 point 的位置
              double x = (u - cx) / fx;
              double y = (v - cy) / fy;
              double depth = fx * b / (disparity.at<float>(v, u));
              point[0] = x * depth;
              point[1] = y * depth;
              point[2] = depth;
  
              pointcloud.push_back(point);
          }
  
      cv::imshow("disparity", disparity / 96.0);
      cv::waitKey(0);
      // 画出点云
      showPointCloud(pointcloud);
      return 0;
  }
  
  void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud) {
  
      if (pointcloud.empty()) {
          cerr << "Point cloud is empty!" << endl;
          return;
      }
  
      pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
      glEnable(GL_DEPTH_TEST);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
      pangolin::OpenGlRenderState s_cam(
          pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
          pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
      );
  
      pangolin::View &d_cam = pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
          .SetHandler(new pangolin::Handler3D(s_cam));
  
      while (pangolin::ShouldQuit() == false) {
          glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
          d_cam.Activate(s_cam);
          glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  
          glPointSize(2);
          glBegin(GL_POINTS);
          for (auto &p: pointcloud) {
              glColor3f(p[3], p[3], p[3]);
              glVertex3d(p[0], p[1], p[2]);
          }
          glEnd();
          pangolin::FinishFrame();
          usleep(5000);   // sleep 5 ms
      }
      return;
  }
  ```

- 编译:

  CMakeList.txt:

  ```cmake
  cmake_minimum_required(VERSION 2.8)
  project( test )
  find_package( OpenCV REQUIRED )
  find_package(Pangolin REQUIRED)
  include_directories( ${OpenCV_INCLUDE_DIRS} )
  add_executable( test test.cpp )
  target_link_libraries( test ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
  ```




## 3. RGB-D视觉

- 完成两件事：

  1. 根据内参计算一对RGB-D图像对应的点云。
  2. 根据各张图的相机位姿(外参)，把点云加起来，组成地图。

- 代码：

  ```c++
  #include <iostream>
  #include <fstream>
  #include <opencv2/opencv.hpp>
  #include <boost/format.hpp>  // for formating strings
  #include <pangolin/pangolin.h>
  #include <sophus/se3.hpp>
  
  
  using namespace std;
  typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  
  // 在pangolin中画图，已写好，无需调整
  void showPointCloud(
      const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud);
  
  int main(int argc, char **argv) {
      vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
      TrajectoryType poses;         // 相机位姿（外参）
  
      ifstream fin("./pose.txt");	// 读取5章图像的位姿(外参):[x,y,z,qx,qy,qz,qw]平移向量+四元数表示的旋转
      if (!fin) {
          cerr << "请在有pose.txt的目录下运行此程序" << endl;
          return 1;
      }
  
      for (int i = 0; i < 5; i++) {
          boost::format imgfmt("./%s/%d.%s"); //图像文件格式
          colorImgs.push_back(cv::imread((imgfmt % "color" % (i + 1) % "png").str()));
          depthImgs.push_back(cv::imread((imgfmt % "depth" % (i + 1) % "pgm").str(), -1)); // 使用-1读取原始图像
  
          double data[7] = {0};
          for (auto &d:data)
              fin >> d;
          // 根据平移向量+四元数表示的旋转构造Sophus中的李群SE(3)
          Sophus::SE3d pose(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                            Eigen::Vector3d(data[0], data[1], data[2]));
          poses.push_back(pose);
      }
  
      // 计算点云并拼接
      // 相机内参 
      double cx = 325.5;
      double cy = 253.5;
      double fx = 518.0;
      double fy = 519.0;
      double depthScale = 1000.0;
      vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
      pointcloud.reserve(1000000);
  
      for (int i = 0; i < 5; i++) {
          cout << "转换图像中: " << i + 1 << endl;
          cv::Mat color = colorImgs[i];
          cv::Mat depth = depthImgs[i];
          Sophus::SE3d T = poses[i];
          for (int v = 0; v < color.rows; v++)
              for (int u = 0; u < color.cols; u++) {
                  unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                  if (d == 0) continue; // 为0表示没有测量到
                  Eigen::Vector3d point;
                  point[2] = double(d) / depthScale;
                  point[0] = (u - cx) * point[2] / fx;
                  point[1] = (v - cy) * point[2] / fy;
                  Eigen::Vector3d pointWorld = T * point;
  
                  Vector6d p;
                  p.head<3>() = pointWorld;
                  p[5] = color.data[v * color.step + u * color.channels()];   // blue
                  p[4] = color.data[v * color.step + u * color.channels() + 1]; // green
                  p[3] = color.data[v * color.step + u * color.channels() + 2]; // red
                  pointcloud.push_back(p);
              }
      }
  
      cout << "点云共有" << pointcloud.size() << "个点." << endl;
      showPointCloud(pointcloud);
      return 0;
  }
  
  void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {
  
      if (pointcloud.empty()) {
          cerr << "Point cloud is empty!" << endl;
          return;
      }
  
      pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
      glEnable(GL_DEPTH_TEST);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
      pangolin::OpenGlRenderState s_cam(
          pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
          pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
      );
  
      pangolin::View &d_cam = pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
          .SetHandler(new pangolin::Handler3D(s_cam));
  
      while (pangolin::ShouldQuit() == false) {
          glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
          d_cam.Activate(s_cam);
          glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  
          glPointSize(2);
          glBegin(GL_POINTS);
          for (auto &p: pointcloud) {
              glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
              glVertex3d(p[0], p[1], p[2]);
          }
          glEnd();
          pangolin::FinishFrame();
          usleep(5000);   // sleep 5 ms
      }
      return;
  }
  ```

- 编译：

  ```cmake
  cmake_minimum_required(VERSION 2.8)
  project( test )
  find_package( OpenCV REQUIRED )
  find_package(Pangolin REQUIRED)
  find_package(Sophus REQUIRED)
  
  
  include_directories( ${OpenCV_INCLUDE_DIRS} )
  add_executable( test test.cpp )
  target_link_libraries( test ${OpenCV_LIBS} ${Pangolin_LIBRARIES} Sophus::Sophus)
  ```

  

# Ceres库

[教程](http://ceres-solver.org/tutorial.html)

书上代码见十四讲笔记第六章3.2.1

Ceres Solver是一个开源C++库，用于建模和解决大型复杂的优化问题。它可以用于解决具有边界约束和一般无约束优化问题的非线性最小二乘问题。

## 1. 安装

1. 安装依赖

   主要是谷歌自己的日志和测试工具

   ```shell
   sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
   ```

2. 下载源码

   ```
   git clone git@github.com:ceres-solver/ceres-solver.git
   cd ceres-solver/
   mkdir build
   cd build
   cmake ..
   sudo make install
   ```

## 2. 使用

- ceres可以解决的有边界约束的最小二乘问题形式如下：
  $$
  \mathop{min}_x \frac{1}{2}\sum_i \rho_i\big(||f_i(x_{i1},\cdots,x_{in})||^2 \big)\ \ s.t.\ l_j<=x_j<=u_j\tag{1}
  $$

  - $\rho_i\big(||f_i(x_{i1},\cdots,x_{in})||^2$：是一个residualBlock，残差块 
  - $x_1,\cdots,x_n$为优化变量，也称为参数块(Parameter blocks)
  - $f_i$​称为代价函数(cost function)
  - $\rho_i$: LossFunction, 是一个scalar function，用于减少Outlisers的影响
  - $l_j,u_j$是第j个优化变量的上限和下限。最简单的就是取正负无穷，即无约束

- 用ceres求解最小二乘问题需要做到：

  1. 定义参数块：可以是向量，四元数，李代数。
  2. 定义残差块f()的计算方式，ceres对他们求平方和之后，作为目标函数的值
  3. 把所有参数块和残差块加入ceres定义的Problem对象中，调用solve函数求解即可。

### 2.1 一个简单的例子

- 问题：

  找到下面这个方程的最小值
  $$
  \frac{1}{2}(10-x)^2
  $$

- 求解：

  1. 定义一个c++11的仿函数functor来计算cost function$f(x)=10-x$

     ```c++
     struct CostFunctor {
        template <typename T>
        bool operator()(const T* const x, T* residual) const {
          residual[0] = 10.0 - x[0];
          return true;
        }
     };
     ```

     > **仿函数functor**: 就是在类/结构体中实现一个`operator()`，使得这个类/结构体具有类似函数的行为
     >
     > **const T* const x** : 第一个const修饰T*，表示不能修改T类型的对象。第二个const修饰指针本身，即不能指向其他地址。因此：不能通过 `x` 修改它指向的 `T` 类型对象的值，也不能修改 `x` 本身（即不能让 `x` 指向其他地址）
     >
     > **operator()(....) const{...}的const**:表示这个成员函数不会修改它所属的类的任何成员变量
     >
     > **模板**：可以允许ceres在调用`CostFunctor::operator<T>()`时，使用T=double或者T=jet(雅可比矩阵)等不同类型数据结构

  2. 构建最小二乘法问题：

     ```c++
     int main(int, char**) {
         
         // 1. 设置初始值
         double ininal_num=5.0;
         double x=ininal_num;
      
         // 2. 构建问题Problem有两个重要的成员函数：
         // Problem::AddResidalBlock() and Problem::AddParameterBlock()
         ceres::Problem problem;
      
         // 设置损失函数，这里使用自动求导计算雅克比矩阵以我们自己定义的CostFunctor为类型在进行初始化
     	// <1,1>指的是CostFunctor中各个参数的维度
         // 第一个1：CostFunctor的残差维度。第二个1:CostFunctor参数块的维度。
         ceres::CostFunction* cost_functor=
         new ceres::AutoDiffCostFunction<CostFunctor,1,1>(new CostFunctor);
         // 添加残差
         // ceres中loss/cost function的区别见上面的公式1
        	// cost_functor即cost function 
     	// nullptr表示没有loss funciton
         problem.AddResidualBlock(cost_functor,nullptr,&x);
      
         // 3. 配置求解器
         // ceres::Solver::Options是一个结构体，用于配置求解器的各种选项。这些选项决定了Ceres求解器如何执行优化过程
         ceres::Solver::Options options;
         options.max_num_iterations=20; 	// 最大迭代次数
         options.linear_solver_type=ceres::DENSE_QR;
         options.minimizer_progress_to_stdout=true;
         // ceres::Solver::Summary是一个结构体，用于存储优化过程的结果和统计信息。
         ceres::Solver::Summary summary;
         
         // 4. 开始求解
         ceres::Solve(options,&problem,&summary);
         std::cout<<summary.BriefReport()<<"\n";
         std::cout<<"x:"<<ininal_num<<"->"<<x<<"\n";
         return 0;
      
     }
     ```

     - 求导

       1. **自动求导**(automatic dirivatives)：如上面的`ceres::CostFunction* cost_functor = new ceres::AutoDiffCostFunction<CostFunctor,1,1>(new CostFunctor);`

       2. **数值求导**(numeric derivatives)：在自动求导的基础上加一个有限微分计划，如：`ceres::CENTRAL`

          ```
          CostFunction* cost_function =
            new NumericDiffCostFunction<CostFunctor, ceres::CENTRAL, 1, 1>();
          problem.AddResidualBlock(cost_function, nullptr, &x);
          ```

       3. **解析求导**(Analytic derivatives)：允许我们自己实现残差residual和雅可比计算公式jacobian comutation code

          ```c++
          class QuadraticCostFunction : public ceres::SizedCostFunction<1, 1> {
           public:
            virtual ~QuadraticCostFunction() {}
            virtual bool Evaluate(double const* const* parameters,
                                  double* residuals,
                                  double** jacobians) const {
              const double x = parameters[0][0];
              residuals[0] = 10 - x;
          
              // Compute the Jacobian if asked for.
              if (jacobians != nullptr && jacobians[0] != nullptr) {
                jacobians[0][0] = -1;
              }
              return true;
            }
          };
          ```

          

### 2.2 复杂点的例子

- 问题: powell' function
  $$
  \begin{aligned}
  &f_{1}(x) =x_1+10x_2 \\
  &f_{2}(x) =\sqrt{5}(x_3-x_4) \\
  &f_3(x) =(x_2-2x_3)^2 \\
  &f_{4}(x) =\sqrt{10}(x_1-x_4)^2 \\
  &F(x) =[f_1(x), f_2(x), f_3(x), f_4(x)] 
  \end{aligned}
  $$
  希望找到一个$x=[x_1,x_2,x_3,x_4]$让$\frac{1}{2}||F(x)||^2$最小。

- 求解：

  1. 用仿函数定义各个cost function

     ```c++
     struct F1 {
       template <typename T>
       bool operator()(const T* const x1, const T* const x2, T* residual) const {
         // f1 = x1 + 10 * x2;
         residual[0] = x1[0] + 10.0 * x2[0];
         return true;
       }
     };
     struct F2 {
       template <typename T>
       bool operator()(const T* const x3, const T* const x4, T* residual) const {
         // f2 = sqrt(5) (x3 - x4)
         residual[0] = sqrt(5.0) * (x3[0] - x4[0]);
         return true;
       }
     };
     struct F3 {
       template <typename T>
       bool operator()(const T* const x2, const T* const x3, T* residual) const {
         // f3 = (x2 - 2 x3)^2
         residual[0] = (x2[0] - 2.0 * x3[0]) * (x2[0] - 2.0 * x3[0]);
         return true;
       }
     };
     struct F4 {
       template <typename T>
       bool operator()(const T* const x1, const T* const x4, T* residual) const {
         // f4 = sqrt(10) (x1 - x4)^2
         residual[0] = sqrt(10.0) * (x1[0] - x4[0]) * (x1[0] - x4[0]);
         return true;
       }
     };
     ```

  2. 构建最小二乘问题

     ```c++
     double x1 =  3.0; double x2 = -1.0; double x3 =  0.0; double x4 = 1.0;
     
     ceres::Problem problem;
     
     // Add residual terms to the problem using the autodiff wrapper to get the derivatives automatically.
     problem.AddResidualBlock(
       new ceres::AutoDiffCostFunction<F1, 1, 1, 1>(), nullptr, &x1, &x2);
     problem.AddResidualBlock(
       new ceres::AutoDiffCostFunction<F2, 1, 1, 1>(), nullptr, &x3, &x4);
     problem.AddResidualBlock(
       new ceres::AutoDiffCostFunction<F3, 1, 1, 1>(), nullptr, &x2, &x3);
     problem.AddResidualBlock(
       new ceres::AutoDiffCostFunction<F4, 1, 1, 1>(), nullptr, &x1, &x4);
     ```

  3. 优化求导

     ```c++
     Solver::Options options;
     options.max_num_iterations = 100;
     options.linear_solver_type = ceres::DENSE_QR;
     options.minimizer_progress_to_stdout = true;
     Solver::Summary summary;
     Solve(options, &problem, &summary);
     std::cout << summary.FullReport() << "\n";
     ```

# g2o库

书上代码见十四讲笔记第六章3.3.1

g2o(General Graphic Optimization)是一个基于图优化的优化库，图优化是一种将非线性优化与图论结合起来的理论。

g2o可以求解任何能够表示为图优化的最小二乘问题。

## 1. 安装/卸载

1. 安装依赖

   ```
   sudo apt-get install qt5-qmake qt5-default libqglviewer-dev-qt5 libsuitesparse-dev libcxsparse3 libcholmod3
   ```

2. 下载源代码

   ```
   git clone git@github.com:RainerKuemmerle/g2o.git
   cd g2o/
   mkdir build 
   cd build
   cmake ..
   sudo make install
   ```

3. 卸载

   ```
   xargs rm < install_manifest.txt
   ```

## 2. 图优化基本理论

图优化是把优化问题表现成图的一种方式

图由若干个定点vertex,以及连接着这些定点的边edge组成

- 顶点：表示优化变量
- 边：表示误差项





## 3. 卸载

```
sudo rm -r /usr/local/include/g2o

sudo rm -r /usr/local/lib/libg2o*

sudo rm -r /usr/local/bin/g2o*
```

## 4. Doxygen文档

文档在源码目录中，需要自己编译

```
cd /home/yang/3rd_library/g2o-20230223_git/doc/doxygen
doxygen doxy.config
```

然后生成的html文件夹中找到index.html，就是文档了。



## 5. 使用

具体的代码例子见slam14讲笔记的第6，9，10章

用g2o进行优化需要做如下事：

1. 创建一个线性求解器LinearSolver。
2. 创建BlockSolver，并用上面定义的线性求解器初始化。
3. 创建总求解器solver，并从GN/LM/DogLeg 中选一个作为迭代策略，再用上述块求解器BlockSolver初始化。
4. 创建图优化的核心：稀疏优化器（SparseOptimizer）。
5. 定义图的顶点和边，并添加到SparseOptimizer中。
6. 设置优化参数，开始执行优化。
   



# Boost库

Boost是一个功能强大、构造精巧、跨平台、开源并且完全免费的C++程序库，在1998年由Beman G.Dawes发起倡议并建立。使用了许多现代C++编程技术，内容涵盖字符串处理、正则表达式、容器与数据结构、并发编程、函数式编程、泛型编程、设计模式实现等许多领域，极大地丰富了C++的功能和表现力，能够使C++软件开发更加简洁、优雅、灵活和高效。

Boost库可以与C++标准库完美共同工作，并且为其提供扩展功能。大部分Boost库功能的使用之需要包括相应的头文件即可，少数需要连接库。

[官方文档](https://www.boost.org/doc/libs/1_82_0/)

## 1. 安装

1. 从[官网](https://www.boost.org/users/download/)下载源代码

2. 安装所有的库：

   ```
   tar xvf boost_1_82_0.tar.gz
   
   cd boost_1_82_0
   
   ./boostrap.sh
   sudo ./b2 --buildtype=complete install
   sudo ./b2 install
   ```

3. 安装特定的库

   ```shell
   ./b2 --show-libraries 	# 看看boost都有哪些库
   sudo ./b2 --with-filesystem --with-python --buildtype=complete install	#因为pinocchio源码依赖于python和filesystems，所以我只安装这2个
   ```

   - 但在安装pinocchio时，说缺xx boost库，所以还是直接全部安装吧。。

4. 检查是否安装成功

   ```shell
   cat /usr/local/include/boost/version.hpp | grep "BOOST_LIB_VERSION"
   ```

   应当返回：
   ```python
   //  BOOST_LIB_VERSION must be defined to be the same as BOOST_VERSION
   #define BOOST_LIB_VERSION "1_54"
   ```

   

## 2. 使用:

在安装环境后，使用大部分的Boost库只需要在自己的源代码里包含对应库的头文件即可。如：

```c++
#include <boost/python.hpp>
#include <boost/python/list.hpp>
#include <boost/python/module.hpp>
#include <boost/assert.hpp>
```



# Pinocchio库

Pinocchio 是一个 C++ 库，用于动力学建模和仿真机器人系统。它提供了一个灵活而强大的框架，用于描述和模拟复杂的多体动力学系统，特别适用于机器人、机械系统和生物力学模型。可以1.动力学建模 2.运动学计算 3.动力学计算 4.优化和控制 5.ROS集成



## 1. 源码安装

[官网教程](https://stack-of-tasks.github.io/pinocchio/download.html)

依赖过于复杂，放弃（或许放弃后面那些可选依赖会容易很多）

## 2. 用robotpkg安装

[官网教程](https://stack-of-tasks.github.io/pinocchio/download.html)

## 3. 计算逆向运动学

```c++
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
    #define PINOCCHIO_MODEL_DIR "urdf"
#endif
int main(int  argc , char **  argv )
{   
    // You should change here to set up your own URDF file or just pass it as an argument of this example.
    const std::string urdf_filename = (argc<=1) ? PINOCCHIO_MODEL_DIR + std::string("/ur5_robot.urdf") : argv[1];
    pinocchio::Model model;
    // pinocchio::buildModels::manipulator(model);
    pinocchio::urdf::buildModel(urdf_filename,model);
    std::cout << "model name: " << model.name << std::endl;
    pinocchio::Data data(model);

    const int JOINT_ID = 6;     //末端执行器对应的关节是第六个，因为ur5是六轴的
    const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1., 0., 1.));   //给予希望的位姿

    Eigen::VectorXd q = pinocchio::neutral(model);  // 返回中性配置空间
    const double eps  = 1e-4;                       // 期望达到的精度
    const int IT_MAX  = 1000;
    const double DT   = 1e-1;
    const double damp = 1e-6;

    pinocchio::Data::Matrix6x J(6,model.nv);   // 定义末端执行器的雅可比矩阵
    J.setZero();

    bool success = false;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err;
    Eigen::VectorXd v(model.nv);
    for (int i=0;;i++)
    {
        pinocchio::forwardKinematics(model,data,q);
        const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);    // desired pose
        err = pinocchio::log6(dMi).toVector();                          // error between desired pose and current pose
        if(err.norm() < eps)
        {
        success = true;
        break;
        }
        if (i >= IT_MAX)
        {
        success = false;
        break;
        }
        pinocchio::computeJointJacobian(model,data,q,JOINT_ID,J);   // 计算末端执行器的雅可比
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = - J.transpose() * JJt.ldlt().solve(err);      // 为了避免奇异，所以这里用的伪逆公式，去计算速度。
        q = pinocchio::integrate(model,q,v*DT);                     // 将获得的切向向量加到配置中去，integrate就是简单的加法
        if(!(i%10))
        std::cout << i << ": error = " << err.transpose() << std::endl;
    }

    if(success) 
    {
        std::cout << "Convergence achieved!" << std::endl;
    }
    else 
    {
        std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
    }
        
    std::cout << "\nresult: " << q.transpose() << std::endl;
    std::cout << "\nfinal error: " << err.transpose() << std::endl;
}
```



# Theseus库

Facebook开发的，下面的几个库也都是fb的。用于在pytorch中设立非线性优化层，来解决非线性优化问题

可以解决：

- [Linear solvers](https://github.com/facebookresearch/theseus/tree/main/theseus/optimizer/linear)
  - Gauss-Newton, Levenberg–Marquardt
- [Linear solvers](https://github.com/facebookresearch/theseus/tree/main/theseus/optimizer/linear)
  - Dense: Cholesky, LU; Sparse: CHOLMOD, LU (GPU-only), [BaSpaCho](https://github.com/facebookresearch/baspacho)
- [Commonly used costs](https://github.com/facebookresearch/theseus/tree/main/theseus/embodied), [AutoDiffCostFunction](https://github.com/facebookresearch/theseus/blob/main/theseus/core/cost_function.py), [RobustCostFunction](https://github.com/facebookresearch/theseus/blob/main/theseus/core/robust_cost_function.py)
- [Lie groups](https://github.com/facebookresearch/theseus/tree/main/theseus/geometry)
- [Robot kinematics](https://github.com/facebookresearch/theseus/blob/main/theseus/embodied/kinematics/kinematics_model.py)

## 1. Dispenso库安装

[dispebnso](https://github.com/facebookincubator/dispenso)是一个c++的多线程库

### 1.1 源码安装

```cmake
#从git上release下载最新版本
mkdir build && cd build
cmake ..
make -j
sudo make install
```

## 2. OpenBLAS

[OpenBLAS](https://github.com/xianyi/OpenBLAS)是一个优化过的BLAS库(Basic Linear Algebra Subprograms) 

### 2.1 源码安装

```shell
git clone git@github.com:xianyi/OpenBLAS.git
cd OpenBLAS
mkdir build && cd build 
cmake ..
make
sudo make install
```

## 3. BaSpaCho库安装

[BaSpaCho(**Ba**tched **Spa**rse **Cho**lesky) ](https://github.com/facebookresearch/baspacho)是一个对称正定稀疏矩阵求解器

### 3.1 源码安装

```shell
git clone git@github.com:facebookresearch/baspacho.git
cd baspacho
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DBLA_STATIC=ON -DBUILD_SHARED_LIBS=OFF
make -j16
ctest # 测试是否安装成功
sudo make install
```

## 4. Theseus源码安装

```
git clone https://github.com/facebookresearch/theseus.git && cd theseus
BASPACHO_ROOT_DIR=/home/yang/3rdLibrary/baspacho pip install -e .
```



# Google Test

Google Test是Google的开源C++单元测试框架。旨在帮助开发人员编写可靠和可重复的单元测试，用于验证代码的正确性。

## 1. 源码安装

```shell
git clone https://github.com/google/googletest.git -b v1.13.0
cd googletest        # Main directory of the cloned repository.
mkdir build          # Create a directory to hold the build output.
cd build
cmake ..             # Generate native build scripts for GoogleTest.
make
sudo make install    # Install in /usr/local/ by default
```

## 2. 学习资源汇总

[官方git](https://github.com/google/googletest/tree/main)

[文档教程](https://google.github.io/googletest/quickstart-cmake.html)

# PCL库

## 1. 安装

- 依赖

  ```
  sudo apt-get update
  sudo apt-get install git build-essential linux-libc-dev
  sudo apt-get install cmake cmake-gui
  sudo apt-get install libusb-1.0-0-dev libusb-dev libudev-dev
  sudo apt-get install mpi-default-dev openmpi-bin openmpi-common
  sudo apt-get install libflann1.9 libflann-dev
  sudo apt-get install libeigen3-dev
  sudo apt-get install libboost-all-dev
  sudo apt-get install libqhull* libgtest-dev  
  sudo apt-get install freeglut3-dev pkg-config  
  sudo apt-get install libxmu-dev libxi-dev   
  sudo apt-get install mono-complete   
  sudo apt-get install libopenni-dev   
  sudo apt-get install libopenni2-dev 
  sudo apt-get install libvtk7-dev libvtk6-dev
  sudo apt-get install qt5-default
  ```

- 安装pcl

  ```
  sudo apt-get install libpcl-dev
  ```

## 2. 使用

- test.cpp

  ```c++
  #include <iostream>
  #include <pcl/common/common_headers.h>
  #include <pcl/io/pcd_io.h>
  #include <pcl/visualization/pcl_visualizer.h>
  #include <pcl/visualization/cloud_viewer.h>
  #include <pcl/console/parse.h>
  
  
  int main(int argc, char **argv) {
      std::cout << "Test PCL !!!" << std::endl;
      
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
      uint8_t r(255), g(15), b(15);
      for (float z(-1.0); z <= 1.0; z += 0.05)
      {
          for (float angle(0.0); angle <= 360.0; angle += 5.0)
          {
              pcl::PointXYZRGB point;
              point.x = 0.5 * cosf (pcl::deg2rad(angle));
              point.y = sinf (pcl::deg2rad(angle));
              point.z = z;
              uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
              point.rgb = *reinterpret_cast<float*>(&rgb);
              point_cloud_ptr->points.push_back (point);
          }
          if (z < 0.0)
          {
              r -= 12;
              g += 12;
          }
          else
          {
              g -= 12;
              b += 12;
          }
      }
      point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
      point_cloud_ptr->height = 1;
      
      pcl::visualization::CloudViewer viewer ("test");
      viewer.showCloud(point_cloud_ptr);
      while (!viewer.wasStopped()){ };
      return 0;
  }
  ```

- 编译

  ```
  cmake_minimum_required(VERSION 2.6)
  project(TEST)
  
  find_package(PCL REQUIRED)
  
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  
  add_executable(TEST test.cpp)
  
  target_link_libraries (TEST ${PCL_LIBRARIES})
  
  install(TARGETS TEST RUNTIME DESTINATION bin)
  
  ```


# osqp库

osqp全称Operator Splitting Quadratic Program，是一个开源的二次规划问题求解器，由牛津大学维护。osqp采用了交替方向乘子法（ADMM）来求解二次规划问题，

二次规划问题的标准形式如下：
$$
min f(x) = x^T Q x + c^T x\\
s.t. Ax <= b
$$
可以使用二次规划问题来求解机器人运动的轨迹，使机器人能够在最短时间内到达目的地。

## 1. 安装

见[osqp](https://github.com/osqp/osqp)

[osqp-eigen](https://github.com/robotology/osqp-eigen)

## 2. 使用



# OpenGL库

## 1. 安装

- 根据[官网](https://en.wikibooks.org/wiki/OpenGL_Programming/Installation/Linux)：

```
sudo apt-get install build-essential libgl1-mesa-dev
sudo apt-get install libglew-dev libsdl2-dev libsdl2-image-dev libglm-dev libfreetype6-dev
sudo apt-get install libglfw3-dev libglfw3
```

- 额外的一些库：

  - OpenGL核心库，GL

    ```
    COPYsudo apt-get install libgl1-mesa-dev
    ```

  - OpenGL实用函数库，GLU

    ```
    COPYsudo apt-get install libglu1-mesa-dev
    ```

  - OpenGL实用工具包，GLUT

    ```
    COPYsudo apt-get install freeglut3-dev
    ```

- 验证是否安装好：

  ```
  sudo apt install mesa-utils
  glxinfo | grep OpenGL
  ```



## 2. 简单的cpp例子

### 2.1 代码

```c++
// test.cpp
#include<GL/glut.h>
#include<stdlib.h>
// 初始化材料属性、光源属性、光照模型，打开深度缓冲区
void init(){
    GLfloat mat_specular [] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat mat_shininess [] = { 50.0 };
    GLfloat light_position [] = { 1.0, 1.0, 1.0, 0.0 };
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_SMOOTH);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
}
// 调用GLUT函数，绘制一个球
void display(){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glutSolidSphere(1.0, 40, 50);
    glFlush();
}

int main(int argc, char** argv){
    // GLUT环境初始化
    glutInit(&argc, argv);
    // 显示模式初始化
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    // 定义窗口大小
    glutInitWindowSize(300, 300);
    // 定义窗口位置
    glutInitWindowPosition(100, 100);
    // 显示窗口，窗口标题为执行函数名 */
    glutCreateWindow(argv[0]);
    // 调用OpenGL初始化函数
    init();
    // 注册OpenGL绘图函数
    glutDisplayFunc(display);
    // 进入GLUT消息循环，开始执行程序
    glutMainLoop();
    return 0;
}
```

### 2.2 编译

```
g++ -o test test.cpp -lGL -lGLU -lglut
./test
```

