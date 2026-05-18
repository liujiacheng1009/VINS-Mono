# IMU Factor 工程规范梳理

本文档梳理 `vins_estimator/src/factor/imu_factor.h` 中 `IMUFactor` 的工程改进点。
重点关注代码可维护性、构建依赖边界，以及 Ceres 因子实现的长期稳定性。

## 当前结构

`IMUFactor` 当前继承自 `ceres::SizedCostFunction<15, 7, 9, 7, 9>`。
它直接解析四个参数块：

- 第 `i` 帧位姿
- 第 `i` 帧速度与 bias
- 第 `j` 帧位姿
- 第 `j` 帧速度与 bias

当前实现把残差计算、平方根信息矩阵加权，以及所有雅可比块的计算都放在 `Evaluate()` 内部。

## 建议改进点

### 将实现移出头文件

`Evaluate()` 目前完整实现在 `imu_factor.h` 中。
这会让头文件变重，并把实现细节依赖传播到所有包含该头文件的编译单元。

推荐结构：

- `imu_factor.h` 只保留类声明。
- 将 `Evaluate()` 和辅助函数移动到 `imu_factor.cpp`。
- 在相关 CMake target 中加入 `imu_factor.cpp`。

这样可以降低编译期耦合，也让因子实现更容易阅读和测试。

### 明确预积分对象的所有权

当前 factor 内部保存的是：

```cpp
IntegrationBase* pre_integration;
```

从类型上看不出 `IMUFactor` 是否拥有该对象，也看不出是否会修改它。
如果 `IMUFactor` 不拥有预积分对象，建议优先改成：

```cpp
const IntegrationBase* pre_integration_;
```

如果确实需要表达所有权，应在构造边界使用智能指针。
以当前 VINS 的对象生命周期风格看，非拥有的 `const IntegrationBase*` 可能是侵入性最低的改法。

### 使用 `explicit` 和 `override`

构造函数和虚函数覆写可以写得更明确：

```cpp
explicit IMUFactor(const IntegrationBase* pre_integration);

bool Evaluate(double const* const* parameters,
              double* residuals,
              double** jacobians) const override;
```

这样可以避免意外的隐式构造，也能让编译器检查 Ceres 接口覆写是否正确。

### 命名参数块

当前代码直接依赖 `parameters[0]`、`parameters[1]`、`parameters[2]`、`parameters[3]` 这类裸索引。
这种写法很紧凑，但后续维护时容易误用。

建议引入命名索引：

```cpp
enum ParameterBlock {
    POSE_I = 0,
    SPEED_BIAS_I = 1,
    POSE_J = 2,
    SPEED_BIAS_J = 3,
};
```

然后通过小的辅助函数或局部结构体解析参数，例如：

```cpp
struct PoseState {
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
};

struct SpeedBiasState {
    Eigen::Vector3d v;
    Eigen::Vector3d ba;
    Eigen::Vector3d bg;
};
```

这样参数布局更容易审查，也能降低索引错位风险。

### 避免魔法数阈值

当前代码用硬编码的 `1e8` 判断数值不稳定。
建议改成具名常量：

```cpp
constexpr double kJacobianNumericalLimit = 1e8;
```

这样能表达阈值含义，也方便后续统一调整。

### 检查平方根信息矩阵计算方式

`Evaluate()` 当前每次都会计算：

```cpp
pre_integration->covariance.inverse()
```

这个操作成本较高，并且当协方差矩阵病态时可能带来数值问题。

可能的改进方向：

- 在预积分协方差更新时缓存 `sqrt_info`。
- 检查 LLT 分解是否成功。
- 在可行时避免显式矩阵求逆。

这是偏底层的数值改动，重构前最好有回归测试覆盖。

### 清理死代码

文件中存在多处 `#if 0` 和注释掉的旧公式。
这些内容会让后续维护者难以判断当前权威实现是哪一版。

建议：

- 如果旧分支已经无用，直接删除。
- 如果注释内容是数学推导说明，移动到文档。
- 如果确实要保留替代实现，应放在有明确名称的编译选项后面，并持续维护。

### 解耦 Factor 与 ROS 日志

当前 factor 中直接调用了 `ROS_WARN()`。
这会让优化因子依赖 ROS 风格日志，对 standalone 构建不够友好。

更推荐的方式：

- 使用项目统一的 logging wrapper。
- 提供一个同时兼容 ROS 和 standalone 的轻量日志宏。
- 如果可以，将诊断信息上移到更高层处理。

### 将雅可比检查放到头文件之外

文件底部仍保留了 `checkJacobian()` 等被注释掉的调试接口。
雅可比检查本身是有价值的，但不适合长期留在生产头文件里。

建议：

- 增加一个专门的 IMU factor 雅可比单元测试或 debug 工具。
- 将解析雅可比与数值差分结果对比。
- 测试工具放在生产头文件之外。

## 建议重构顺序

1. 增加命名参数块索引和局部解析辅助函数。
2. 将 `pre_integration` 重命名为 `pre_integration_`，并在可行时改为 `const`。
3. 给构造函数和 `Evaluate()` 增加 `explicit` / `override`。
4. 将 `Evaluate()` 实现移动到 `imu_factor.cpp`。
5. 将硬编码数值阈值改成具名常量。
6. 清理 `#if 0` 和注释掉的死代码。
7. 检查 `sqrt_info` 计算方式，并增加分解状态检查。
8. 增加雅可比验证测试或 debug 工具。

这个顺序优先提升可读性，并尽量降低行为变化风险。
