## 轻量级矩阵库

该库是移植优化px4飞控矩阵模块，由头文件组成的矩阵库，已经重载运算符，加减乘除像平时使用float一样

使用时包括“PX4_math.h”即包含全部库实现，也可以单独包含单独头文件，内部包含；

* 基本矩阵
* 一维向量
* 方阵（N*N矩阵）
* 四元数
* 旋转矩阵
* 欧拉角（ZYX坐标系）
* 矩阵伪逆

使用时需要加上命名空间**matrix**

注意矩阵库没有大小检查，使用时注意是否超长度

使用示例

```c++
// 初始化欧拉角
    float roll = 0.1f;
    float pitch = 0.2f;
    float yaw = 0.3f;
    Eulerf euler(roll, pitch, yaw);
    
// 通过欧拉角创建四元数
Quatf q_nb(euler);

// 通过四元数创建旋转矩阵
Dcmf dcm(q_nb);

// 可以把欧拉角转化到旋转矩阵（重载“=”运算符）
dcm = euler;

// 您也可以像这样直接从欧拉角度创建 DCM 实例
dcm = Eulerf(roll, pitch, yaw);

// create an axis angle representation from euler angles
AxisAngle<float> axis_angle = euler;

// 用轴角度创建旋转矩阵
Dcmf dcm2(AxisAngle(1, 2, 3));

// 使用轴角度，轴/角度分开以初始化 DCM
Dcmf dcm3(AxisAngle(Vector3f(1, 0, 0), 0.2));

// 做卡尔曼滤波
const size_t n_x = 5;
const size_t n_y = 3;

// 定义矩阵大小
SquareMatrix<float, n_x> P;
Vector<float, n_x> x;
Vector<float, n_y> y;
Matrix<float, n_y, n_x> C;
SquareMatrix<float, n_y> R;
SquareMatrix<float, n_y> S;
Matrix<float, n_x, n_y> K;

// 定义测量矩阵
C = zero<float, n_y, n_x>(); // or C.setZero()
C(0,0) = 1;
C(1,1) = 2;
C(2,2) = 3;

// 将 x 设置为零
x = zero<float, n_x, 1>(); // or x.setZero()

// 将 P 设置为单位阵 * 0.01
P = eye<float, n_x>()*0.01;

// 将 R 设置为单位阵 * 0.1
R = eye<float, n_y>()*0.1;

// measurement
y(0) = 1;
y(1) = 2;
y(2) = 3;

// innovation
r = y - C*x;

// innovations variance
S = C*P*C.T() + R;

// Kalman gain matrix
K = P*C.T()*S.I();
// S.I() is the inverse, defined for SquareMatrix

// correction
x += K*r;

// slicing
float data[9] = {0, 2, 3,
                 4, 5, 6,
                 7, 8, 10
                };
SquareMatrix<float, 3> A(data);

// Slice a 3,3 matrix starting at row 1, col 0,
// with size 2 x 3, warning, no size checking
Matrix<float, 2, 3> B(A.slice<2, 3>(1, 0));

// this results in:
// 4, 5, 6
// 7, 8, 10
```