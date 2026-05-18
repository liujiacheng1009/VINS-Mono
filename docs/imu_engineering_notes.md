# IMU 模块工程规范梳理

本文档汇总 VINS-Mono 中 IMU 相关模块的工程改进点，覆盖：

- `vins_estimator/src/factor/imu_factor.h`（Ceres IMU 因子）
- `vins_estimator/src/factor/integration_base.h`（IMU 预积分）

关注代码可维护性、构建依赖边界、数值稳健性，不改变核心数学公式。

---

## 一、`IMUFactor`

### 当前结构

`IMUFactor` 继承自 `ceres::SizedCostFunction<15, 7, 9, 7, 9>`，直接解析四个参数块：

- 第 `i` 帧位姿
- 第 `i` 帧速度与 bias
- 第 `j` 帧位姿
- 第 `j` 帧速度与 bias

当前实现把残差计算、平方根信息矩阵加权，以及所有雅可比块的计算都放在 `Evaluate()` 内部。

### 建议改进点

#### 将实现移出头文件

`Evaluate()` 目前完整实现在 `imu_factor.h` 中。
这会让头文件变重，并把实现细节依赖传播到所有包含该头文件的编译单元。

推荐结构：

- `imu_factor.h` 只保留类声明。
- 将 `Evaluate()` 和辅助函数移动到 `imu_factor.cpp`。
- 在相关 CMake target 中加入 `imu_factor.cpp`。

这样可以降低编译期耦合，也让因子实现更容易阅读和测试。

#### 明确预积分对象的所有权（已完成，采用 `std::shared_ptr`）

历史上 factor 内部保存的是：

```cpp
Integrator* pre_integration_;
```

从类型上看不出 `IMUFactor` 是否拥有该对象。
当前实现中 `Estimator` 也已经把 `pre_integrations[(WINDOW_SIZE + 1)]` 和
`tmp_pre_integration` 改成 `std::shared_ptr<Integrator>`，
`IMUFactor` 与 `Estimator` 共享所有权：

```cpp
std::shared_ptr<Integrator> pre_integration_;
```

这样即使 `Estimator::slideWindow` 把对应槽位 `reset()`，
已被 Ceres 抓走的 `IMUFactor` 仍能安全访问该预积分，
直到 Ceres / `MarginalizationInfo` 释放 `IMUFactor` 时引用计数才真正归零。

#### 使用 `explicit` 和 `override`

```cpp
explicit IMUFactor(const Integrator* pre_integration);

bool Evaluate(double const* const* parameters,
              double* residuals,
              double** jacobians) const override;
```

避免意外的隐式构造，让编译器检查 Ceres 接口覆写。

#### 命名参数块

当前已通过注释说明 `parameters[0..3]` / `jacobians[0..3]` 的含义。如果后续 factor 变体增多，可统一改用具名解析 helper，例如 `parsePose()` / `parseSpeedBias()`。

#### 避免魔法数阈值

已抽出具名常量：

```cpp
static constexpr double kJacobianNumericalLimit = 1e8;
```

#### 检查平方根信息矩阵计算方式

`Evaluate()` 当前每次都会计算：

```cpp
pre_integration_->covariance.inverse()
```

成本较高，并且协方差病态时可能不稳定。改进方向：

- 在预积分协方差更新时缓存 `sqrt_info`。
- 检查 `LLT` 分解的 `info()`。
- 避免显式矩阵求逆，用分解后回代。

这是偏底层数值改动，重构前最好有回归测试覆盖。

#### 解耦 Factor 与 ROS 日志

当前 factor 直接调用 `ROS_WARN()`，让因子依赖 ROS 风格日志，对 standalone 构建不够友好。建议使用项目统一的 logging wrapper，或者把诊断信息上移到更高层。

#### 增加雅可比验证测试

雅可比解析推导易出错。建议增加专门的单元测试，把解析雅可比和数值差分对比，放在生产头文件之外。

---

## 二、`Integrator`

### 当前结构

`Integrator` 是 IMU 预积分核心类，负责：

- 在 `push_back()` / `propagate()` 中按 IMU 序列做中值积分。
- 在 `midPointIntegration()` 中同时更新预积分量和 15x15 雅可比、协方差。
- 在 `evaluate()` 中根据当前 bias 计算 IMU 残差。
- 在 `repropagate()` 中以新的线性化 bias 重新积分整段。

类暴露的全部状态都是 `public`，并在头文件里包含 `using namespace Eigen;`。

### 建议改进点

#### 移除 `using namespace Eigen;`

```cpp
using namespace Eigen;
```

放在头文件作用域会污染所有引用该头的编译单元。建议改为函数内局部使用，或者全程用 `Eigen::Vector3d` 等全限定名。

#### 控制成员可见性

当前所有成员都是 `public`，外部任意代码都可以修改 `delta_p`、`covariance`、`linearized_ba` 等内部状态。

建议：

- 把内部传播状态改为 `private`。
- 通过 `getDeltaP()` / `getCovariance()` / `getSumDt()` 等 `const` getter 暴露只读接口。
- 仅向少数密切耦合的协作者（如 `IMUFactor`、marginalization）开放必要 setter，并加注释说明。

#### 使用固定尺寸矩阵

`midPointIntegration()` 中：

```cpp
MatrixXd F = MatrixXd::Zero(15, 15);
MatrixXd V = MatrixXd::Zero(15, 18);
```

这两个矩阵尺寸恒定为 15x15 / 15x18，应改为固定尺寸：

```cpp
Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();
Eigen::Matrix<double, 15, 18> V = Eigen::Matrix<double, 15, 18>::Zero();
```

避免动态分配，且能让编译器做向量化优化。

#### 精简 `midPointIntegration` 参数

当前函数签名共 17 个参数，可读性差。建议把输入/输出分组为结构体，例如：

```cpp
struct IntegrationStep {
    Eigen::Vector3d delta_p;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;
    Eigen::Vector3d linearized_ba;
    Eigen::Vector3d linearized_bg;
};
```

`midPointIntegration()` 输入旧 step 和 IMU 测量，输出新 step。

#### 明确 `update_jacobian` 类型

```cpp
midPointIntegration(..., bool update_jacobian)
```

但调用处用 `1`/`0`：

```cpp
midPointIntegration(..., 1);
midPointIntegration(..., 0);
```

调用处应改成 `true` / `false`，更直观。

#### 改善变量命名

```cpp
Vector3d a_0_x, a_1_x, w_x;
Matrix3d R_a_0_x, R_a_1_x, R_w_x;
```

这种 `_x` 后缀实际表示反对称矩阵（cross product），命名不直观。建议改成 `acc_skew_0` / `acc_skew_1` / `gyr_skew`，或者直接用 `Utility::skewSymmetric()` 构造。

#### 协方差对称化

每次传播后：

```cpp
covariance = F * covariance * F.transpose() + V * noise * V.transpose();
```

理论上对称，但浮点误差会逐步破坏对称性，进而影响 `IMUFactor` 中的 `LLT` 分解。可以在 propagate 末尾加：

```cpp
covariance = 0.5 * (covariance + covariance.transpose());
```

强制对称化，提升数值稳健性。

#### 移除已废弃的成员

```cpp
Eigen::Matrix<double, 15, 15> step_jacobian;
Eigen::Matrix<double, 15, 18> step_V;
```

`midPointIntegration()` 里对它们的赋值已经被注释掉，但成员仍然占用空间。应该删除。

#### 清理文件尾部的死代码

文件末尾有 200+ 行被 `/* ... */` 包裹的 `eulerIntegration()` 和 `checkJacobian()` 调试代码，建议直接删除或迁移到独立 debug 工具。

#### 去掉无用的 include

```cpp
#include <ceres/ceres.h>
```

`Integrator` 本身不使用 Ceres，可以去掉这个 include，由 `IMUFactor` 自己引入即可。

#### 加 `evaluate()` 的 `const` 限定

```cpp
Eigen::Matrix<double, 15, 1> evaluate(...);
```

该函数只读访问预积分对象状态，建议改为：

```cpp
Eigen::Matrix<double, 15, 1> evaluate(...) const;
```

这样在 `IMUFactor` 持有 `const Integrator*` 时也能正常调用。

#### 校验 `dt > 0`

`push_back()` / `propagate()` 接受任意 `dt`，但 `dt <= 0` 时雅可比、协方差累积会异常。建议在入口加断言或显式保护。

#### 重新传播抽成 helper

`repropagate()` 中手动重置 `delta_p / delta_q / delta_v / jacobian / covariance / sum_dt / acc_0 / gyr_0`，容易漏项。建议把状态重置封装为 `resetIntegration()`：

```cpp
void resetIntegration();
```

`repropagate()` 内只调用一次，可读性更好，也减少新成员漏改的风险。

#### 四元数归一化时机

```cpp
result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, ...);
```

只在 `propagate()` 末尾调用一次 `delta_q.normalize()`。如果 IMU 频率很高、姿态变化剧烈，中间计算可能放大数值误差。可以考虑统一改用 `Utility::deltaQ()` 来表达小角度旋转。

---

## 建议重构顺序

`IMUFactor`：

1. 增加参数块说明注释或具名解析 helper。（已完成）
2. 重命名 `pre_integration_`，必要时改为 `const`。（已完成，命名）
3. 给构造函数和 `Evaluate()` 增加 `explicit` / `override`。（已完成）
4. 将 `Evaluate()` 实现移动到 `imu_factor.cpp`。（已完成；`Integrator` 也合并进 `imu_factor.h/.cpp`，移除 `integration_base.h`）
5. 将硬编码数值阈值改成具名常量。（已完成）
6. 清理 `#if 0` 和注释掉的死代码。（已完成）
7. 检查 `sqrt_info` 计算方式，并增加分解状态检查。
8. 增加雅可比验证测试或 debug 工具。

`Integrator`：

1. 移除头文件作用域的 `using namespace Eigen;`。（已完成）
2. 删除已废弃的 `step_jacobian` / `step_V` 成员和文件尾部死代码。（已完成）
3. 把动态尺寸矩阵改为固定尺寸；为 `evaluate()` 添加 `const`。
4. 把 `1`/`0` 改成 `true`/`false`，并整理变量命名。
5. 在 propagate 末尾对协方差做对称化处理。
6. 拆分内部状态结构体，精简 `midPointIntegration` 签名。
7. 收敛成员可见性，提供必要的 `const` getter。
8. 增加预积分单元测试，覆盖正向传播和 `repropagate` 一致性。

整体顺序优先提升可读性和数值稳健性，再涉及接口/可见性这类影响面更广的改动。
