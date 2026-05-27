# 视觉投影因子模块工程规范梳理

本文档汇总 VINS-Multi 中视觉投影残差相关模块的工程改进点，覆盖：

- `vins_estimator/src/factor/projection_factor.h`
- `vins_estimator/src/factor/projection_factor.cpp`
- 与之高度同构的 `projection_td_factor.{h,cpp}`
- 配套的 `pose_local_parameterization.{h,cpp}`

关注代码可维护性、构建依赖边界、数值稳健性，不改变核心数学公式（重投影残差与对应的解析雅可比）。

---

## 一、`ProjectionFactor`

### 当前结构

`ProjectionFactor` 继承自 `ceres::SizedCostFunction<2, 7, 7, 7, 1>`，按位置接收四个参数块：

- 参考帧 `i` 位姿（pose, 7 维：平移 + 四元数）
- 观测帧 `j` 位姿
- 相机—IMU 外参 `T_bc`（pose）
- 特征点在参考帧上的逆深度 `inv_dep_i`

`Evaluate()` 内完成残差链 `pts_i → camera_i → imu_i → world → imu_j → camera_j`、`sqrt_info` 加权，以及四个雅可比块的解析计算。文件底部还有用于人工核对的 `check()` 函数（数值差分对比）。

### 建议改进点

#### 头文件与 include 边界

`projection_factor.h` 当前包含：

```cpp
#include "../utility/logging.h"
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"
```

存在如下问题：

- `logging.h` 与 `parameters.h` 在头文件中没有被任何符号使用（`logging.h` 仅 cpp 里用、`parameters.h` 实际只被 `projection_td_factor` 使用 `ROW/TR`），应移到 cpp。
- `tic_toc.h` 只服务于 `Evaluate()` 内的 `TicToc` 计时，属于实现细节，应同样下沉到 cpp。
- `<Eigen/Dense>` 与 `pose_local_parameterization.h` 中的 `<eigen3/Eigen/Dense>` 不一致，建议统一为 `<Eigen/Dense>`（依赖 CMake 中的 `EIGEN3_INCLUDE_DIR`）。

最终 `projection_factor.h` 只需 `<ceres/ceres.h>` 和 `<Eigen/Core>` 即可。

#### 缺失的现代 C++ 关键字

```cpp
ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
virtual bool Evaluate(...) const;
```

建议：

- 构造函数加 `explicit`，避免从两个 `Vector3d` 隐式构造（实际不会发生，但表达更明确）。
- `Evaluate()` 改用 `override`，让编译器校验 Ceres 接口签名。
- 构造函数定义末尾的 `};` 多余分号应删除。

#### 静态成员的共享语义

```cpp
static Eigen::Matrix2d sqrt_info;
static double sum_t;
```

- `sqrt_info` 是类级全局共享。这在单目场景下没问题，但语义上不利于多相机/多分辨率扩展，也让单元测试需要先 set 静态状态再构造对象。可以考虑改为成员变量（每个 factor 在构造时由 caller 注入），或者用 setter 显式表达初始化时机。
- `sum_t` 是累积 profiling 计数：
  - 全局可写、非线程安全；
  - 永不归零，长时间运行越来越大；
  - 当前没有任何调用方读取它（已确认仓库内无 `ProjectionFactor::sum_t` 引用）。
  - 建议直接删除，或者改用条件编译，仅在 `VINS_PROFILE` 宏开启时启用。

#### `tangent_base` 的初始化与跨平台行为

```cpp
Eigen::Vector3d a = pts_j.normalized();
Eigen::Vector3d tmp(0, 0, 1);
if(a == tmp)
    tmp << 1, 0, 0;
```

两点改进：

- `if(a == tmp)` 对 `Eigen::Vector3d` 使用精确浮点相等，几乎永远不会成立，等价于无效保护。应使用 `(a - tmp).norm() < eps` 或者直接判断 `std::abs(a.z()) > 1 - eps`。
- 当 `UNIT_SPHERE_ERROR` 未定义时，`tangent_base` 不会被赋值。它仍然作为 `Eigen::Matrix<double, 2, 3>` 成员占据 48 字节，且未初始化，处于 undefined value 状态。建议把 `tangent_base` 用预处理保护起来，或者无条件初始化为 `setZero()` 避免误用。

`projection_td_factor.cpp` 中有同样的代码块，可抽成 `Utility::computeTangentBase(const Eigen::Vector3d&)`，避免维护两份。

#### `Evaluate()` 内部的可读性与公共子表达式

```cpp
Eigen::Matrix<double, 2, 3> reduce(2, 3);
```

固定尺寸的 Eigen 矩阵不需要 `(2, 3)` 构造参数，去掉即可，去除 misleading 的动态大小印象。

雅可比计算中重复出现的子表达式可以提前缓存，例如：

```cpp
const Eigen::Matrix3d ric_T            = ric.transpose();
const Eigen::Matrix3d Rj_T             = Rj.transpose();
const Eigen::Matrix3d ric_T_Rj_T       = ric_T * Rj_T;
const Eigen::Matrix3d ric_T_Rj_T_Ri    = ric_T_Rj_T * Ri;
```

当前 `ric.transpose() * Rj.transpose()`、`Rj.transpose() * Ri` 至少各出现 2–3 次。在 Ceres 优化中 `Evaluate()` 会被频繁调用，缓存这些 3×3 矩阵乘积对热点路径有正面影响，也使每个 `jacobians[k]` 块的表达更紧凑。

`pts_camera_j / dep_j` 在 residual 与 `reduce` 中都用到，也可提取为局部变量。

#### 数值健壮性保护

```cpp
double dep_j = pts_camera_j.z();
residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
```

```cpp
Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
```

当 `inv_dep_i ≈ 0` 或 `dep_j ≤ 0` 时，雅可比与残差均会爆掉，但当前没有任何防护。建议：

- 在 `Evaluate()` 入口加 `if (inv_dep_i < kMinInvDepth) return false;`（Ceres 允许通过返回值告知数值失败）。
- `dep_j` 的符号检查同上。
- 这些阈值用具名常量（`constexpr double kMinDepth = 1e-3;`）。

这种检查比硬性 clamp 更安全：让 Ceres 拒绝该步并尝试更小的步长，而不是返回一个被截断的非物理值。

#### 死代码

```cpp
#if 1
    jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i * -1.0 / (inv_dep_i * inv_dep_i);
#else
    jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i;
#endif
```

`#else` 分支恒不进入，应直接删除。保留它会让读代码的人误以为存在另一种参数化（实际上是早期版本残留）。

#### 抽取 residual 计算 helper

`projection_factor.cpp` 中残差链 `pts_i → pts_camera_j → residual` 出现 3 次：

- `Evaluate()` 主体（第 35–47 行）
- `check()` 解析参考分支（第 157–171 行）
- `check()` 数值差分循环（第 208–222 行）

任何一处修改（例如换 `UNIT_SPHERE_ERROR`、加深度保护）都需要在三处同步。建议抽 helper：

```cpp
static Eigen::Vector2d computeResidual(
    const Eigen::Vector3d& Pi, const Eigen::Quaterniond& Qi,
    const Eigen::Vector3d& Pj, const Eigen::Quaterniond& Qj,
    const Eigen::Vector3d& tic, const Eigen::Quaterniond& qic,
    double inv_dep_i,
    const Eigen::Vector3d& pts_i, const Eigen::Vector3d& pts_j,
    const Eigen::Matrix<double, 2, 3>& tangent_base);
```

`check()` 与 `Evaluate()` 共享这一个实现，避免漂移。

#### `check()` 内的内存泄漏与设计问题

```cpp
double *res = new double[15];
double **jaco = new double *[4];
jaco[0] = new double[2 * 7];
jaco[1] = new double[2 * 7];
jaco[2] = new double[2 * 7];
jaco[3] = new double[2 * 1];
```

- `new[]` 全部没有 `delete[]`，每次调用都泄漏。
- `res` 申请了 15 doubles，但残差只有 2 维，过分配。
- `jaco` 应改为局部 `std::array<std::array<double, 14>, 3>` 或 `std::vector<std::array<...>>` 自动管理生命周期。
- 19 这个魔术数（6×3 + 1，外参 + 双位姿 + 逆深度的最小切向维度）应抽常量 `kLocalParameterDim = 19`。
- 当前 `check()` 只打印，不断言。建议把这部分挪到 `tests/projection_factor_test.cpp`，用 gtest 比对解析雅可比和数值差分（参考 `tests/imu_factor_test.cpp` 的做法），CI 才能发现回归。

公开 API 中也建议把 `check()` 移除（或用 `#ifdef DEBUG`），避免发布版本中误调用。

#### `pts_i / pts_j` 的语义注释

```cpp
Eigen::Vector3d pts_i, pts_j;
```

代码隐含 `pts_i = (x/z, y/z, 1)` 的归一化平面齐次形式（`residual` 用 `pts_j.head<2>()`），但头文件没有任何注释。后来者难以推断为什么是 `Vector3d` 而非 `Vector2d`，以及 `z` 是否参与计算（实际上仅在 `UNIT_SPHERE_ERROR` 路径里参与 `.normalized()`）。

建议在头文件加注释：

```cpp
// Normalized image-plane coordinates of the feature in frames i / j.
// Convention: pts.head<2>() = (x/z, y/z), pts.z() == 1 by construction.
Eigen::Vector3d pts_i, pts_j;
```

#### 成员可见性

`pts_i / pts_j / tangent_base` 当前都是 `public`。`ProjectionFactor` 是构造后立即交给 Ceres 的不可变对象，外部不应修改它们。建议：

- 改为 `private`，必要时通过 `const` getter 暴露。
- 这也能避免单元测试或调用方意外篡改 `tangent_base`。

---

## 二、`ProjectionTdFactor`

`ProjectionTdFactor` 与 `ProjectionFactor` 的唯一差别是：

- 多了 `td`（时间偏差）和行号 `row_i / row_j` 用于 rolling shutter 校正；
- 在残差链最前面把 `pts_i / pts_j` 替换成 `pts_i_td / pts_j_td`；
- 多一个 1 维参数块 `td` 的雅可比。

#### 重复代码

整个 `Evaluate()`、`tangent_base` 构造、`check()` 都几乎是粘贴 `ProjectionFactor` 后做了 td 修正。带来的问题：

- 修改 `UNIT_SPHERE_ERROR` 路径、加深度保护、改 helper 时容易遗漏其中一份。
- 数值雅可比 check 函数有两套，互相不一致的概率随时间累积。

可选的重构方向：

- 把残差链拆成 `projectFeature(pts_i_td, ...)` 与 `tangentResidual(pts_camera_j, pts_j_td)` 两个 utility，两个 factor 复用同一段。
- 让 `ProjectionTdFactor` 内部组合一个不含 td 的核心计算器，而不是从头复制。
- 或者抽一个 CRTP / 模板基类提供共享 `Evaluate()` 框架。模板做法会让头文件变重，需要在收益和编译时间之间权衡。

至少应先把 `tangent_base` 的构造与残差链各自抽 helper，作为最小改动。

#### 输入参数中的隐式约定

```cpp
velocity_i.x() = _velocity_i.x();
velocity_i.y() = _velocity_i.y();
velocity_i.z() = 0;
```

这种「`Vector3d` 字段实际只用 xy」的隐式约定，应该直接把成员改为 `Eigen::Vector2d`，或者用 helper `toVec3Z0()`，避免后续维护者误以为 `velocity_i.z()` 有意义。

`row_i / row_j` 在构造函数里被减去 `ROW / 2`，含义是「相对于图像中心的行偏移」。建议在头文件加注释说明（否则外部传 `row_i = 0` 还是图像坐标都难以判断）。

---

## 三、`PoseLocalParameterization`

### 当前结构

```cpp
class PoseLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(...) const;
    virtual bool ComputeJacobian(...) const;
    virtual int GlobalSize() const { return 7; };
    virtual int LocalSize() const { return 6; };
};
```

### 建议改进点

#### include 与命名一致

```cpp
#include <eigen3/Eigen/Dense>
```

与其他 factor 头文件不一致，建议统一为 `<Eigen/Dense>`，依赖 build 系统提供搜索路径。

#### 缺 `override`，缺访问修饰符

类体直接以 `virtual bool Plus(...)` 开始，没有 `public:` 也没有 `private:`。`class` 默认 `private`，因此 `Plus / ComputeJacobian / GlobalSize / LocalSize` 当前实际上是私有的；Ceres 通过基类虚函数表仍能调用到，所以运行没问题，但语义上极容易让读者误解。

建议：

```cpp
class PoseLocalParameterization : public ceres::LocalParameterization
{
public:
    bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
    bool ComputeJacobian(const double *x, double *jacobian) const override;
    int GlobalSize() const override { return 7; }
    int LocalSize() const override { return 6; }
};
```

`GlobalSize()` / `LocalSize()` 函数后的 `;` 是多余的（与 IMU 模块同问题）。

#### `ComputeJacobian` 的含义

```cpp
Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
j.topRows<6>().setIdentity();
j.bottomRows<1>().setZero();
```

这是把切空间到全局参数的雅可比近似为「前 6 行单位、最后 1 行零」，等价于让上层 cost function 自己提供 `2x7` 雅可比但忽略第 7 列（即四元数 w 分量）。这是 VINS 系列代码的惯例，但与 Ceres 文档中标准的「四元数到角度向量」推导并不严格一致；只要 cost function 配合（`jacobian_pose_*.rightCols<1>().setZero();`）就能成立。

建议在头文件用一段注释明确表达这个约定，避免新接入的 cost function 设置错了 `rightCols<1>()`。

---

## 建议重构顺序

`ProjectionFactor` / `ProjectionTdFactor`：

1. 头文件 include 收敛，统一 Eigen 路径；删除未使用的 `logging.h` / `parameters.h`，把 `tic_toc.h` 下沉到 cpp。（`ProjectionFactor` 已完成：头文件只剩 `<ceres/ceres.h>` + `<Eigen/Core>` + `<Eigen/Geometry>`；`parameters.h`（含 `UNIT_SPHERE_ERROR` 宏）移到 `projection_factor.cpp`；`TicToc` 与 `sum_t` 一并删除。`ProjectionTdFactor` 待办。）
2. 添加 `explicit` / `override`，删除尾部多余 `;`，删除 `#if 1 / #else / #endif` 死代码。（`ProjectionFactor` 已完成：构造函数 `explicit` + `= delete` 默认构造，`Evaluate` 加 `override`，删除 feature jacobian 处的死分支。）
3. `Vector3d == Vector3d` 改为容差比较；`tangent_base` 在非 `UNIT_SPHERE_ERROR` 路径下做 `setZero()` 或用预处理保护。（`ProjectionFactor` 已完成：构造函数无条件 `setZero()`，并把比较替换为 `(a - tmp).norm() < 1e-6`。）
4. 抽 `computeTangentBase()` 与 `computeResidual()` helper，让 `Evaluate()` 与 `check()` 共享同一份残差表达式。（`computeTangentBase()` 抽出在 `projection_factor.cpp` 的匿名 namespace 中。`computeResidual()` 因为 `check()` 已删除、`Evaluate()` 内残差链很短，未单独抽出，避免增加无收益的间接层。）
5. 在 `Evaluate()` 入口加 `inv_dep_i / dep_j` 的数值保护，返回 false 而不是返回病态雅可比。（已回退：该改动会改变 Ceres 的步长接受/拒绝行为，导致前后运行指标不一致。当前保留原始行为，继续按旧逻辑计算残差和雅可比。）
6. 缓存 `Evaluate()` 中重复的 3×3 矩阵乘积；删除累积 profiling 的 `sum_t`，或用 `#ifdef VINS_PROFILE` 保护。（已完成：缓存 `ric_T` / `Rj_T` / `ric_T_Rj_T` / `ric_T_Rj_T_Ri`；`sum_t` 与 `TicToc` 调用直接删除。）
7. 把 `check()` 改成独立的单元测试（参考 `tests/imu_factor_test.cpp`），并修复 `new[]` 内存泄漏；移除头文件中的 `check()` 公开接口。（已完成：见 `tests/projection_factor_test.cpp`，6 个测试覆盖构造、零残差、null jacobians、`sqrt_info` 缩放、解析雅可比与中心差分对比；`check()` 已从公开 API 移除。）
8. （较大范围）抽公共基类或 helper，消除 `ProjectionFactor` 与 `ProjectionTdFactor` 的重复代码。

`PoseLocalParameterization`：

1. include 统一为 `<Eigen/Dense>`。
2. 加 `public:` 与 `override`；删除多余 `;`。
3. 在头文件补充注释说明 `ComputeJacobian` 的约定（与 factor 的 `rightCols<1>().setZero()` 配合使用）。

整体顺序仍然遵循「先清理可读性与依赖边界，再补数值/数据流保护，最后才动接口和重构」的原则。
