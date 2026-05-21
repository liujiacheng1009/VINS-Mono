# 边缘化模块工程规范梳理

本文档汇总 VINS-Mono 中边缘化相关模块的工程改进点，覆盖：

- `vins_estimator/src/factor/marginalization_factor.h`
- `vins_estimator/src/factor/marginalization_factor.cpp`
- `Estimator::optimization()` 中构造 `ResidualBlockInfo` / `MarginalizationInfo` 的调用路径

关注代码可维护性、所有权边界、数值稳健性和性能热点，不改变边缘化的核心数学流程（线性化、组装正规方程、Schur complement、生成 prior factor）。

---

## 一、当前结构

### `ResidualBlockInfo`

`ResidualBlockInfo` 保存一个待边缘化 residual block 的：

- `ceres::CostFunction* cost_function`
- `ceres::LossFunction* loss_function`
- 参数块地址 `parameter_blocks`
- 需要被边缘化的参数块下标 `drop_set`
- 线性化后的 `residuals` 与每个参数块的 `jacobians`

`Evaluate()` 会调用原始 Ceres cost function，得到 residual / jacobian，并在存在 robust loss 时对 residual / jacobian 做一次 Ceres 风格的 loss 修正。

### `MarginalizationInfo`

`MarginalizationInfo` 负责：

- 收集多个 `ResidualBlockInfo`。
- 在 `preMarginalize()` 中固定线性化点，保存每个参数块的快照。
- 在 `marginalize()` 中组装全局正规方程 `A` / `b`。
- 对需要 drop 的变量做 Schur complement。
- 将剩余变量的 prior 转换为 `linearized_jacobians` / `linearized_residuals`。
- 通过 `getParameterBlocks()` 输出后续 Ceres problem 中应继续传入 prior factor 的参数块地址。

### `MarginalizationFactor`

`MarginalizationFactor` 是一个动态参数块数量的 `ceres::CostFunction`。

它在 `Evaluate()` 中：

- 读取当前参数块。
- 与 `MarginalizationInfo` 保存的线性化点比较得到 `dx`。
- 返回线性 prior residual：

```cpp
r = r0 + J * dx
```

如果 Ceres 请求 jacobian，则直接把保存的 `linearized_jacobians` 对应列块拷贝到各参数块 jacobian 中。

---

## 二、所有权与生命周期

### 明确 `cost_function` / `loss_function` 所有权

当前 `MarginalizationInfo::~MarginalizationInfo()` 中会释放：

```cpp
delete factors[i]->cost_function;
delete factors[i];
```

但不会释放 `loss_function`。这通常是因为 `loss_function` 由外层复用，或者由 Ceres problem 管理；但从 `ResidualBlockInfo` 的构造函数签名看不出真实所有权。

建议：

- 用注释明确：`ResidualBlockInfo` 是否拥有 `cost_function`、是否只借用 `loss_function`。
- 如果 `cost_function` 由 `ResidualBlockInfo` 独占，改为 `std::unique_ptr<ceres::CostFunction>`。
- 如果 `loss_function` 是共享的裸指针，应命名为 `loss_function_borrowed` 或用注释说明不得释放。
- 避免同一个 `CostFunction*` 同时交给 Ceres `Problem` 和 `MarginalizationInfo` 管理，否则容易双重释放。

### 避免裸数组管理 jacobian 指针

`ResidualBlockInfo::Evaluate()` 中：

```cpp
raw_jacobians = new double *[block_sizes.size()];
```

析构时再 `delete[] factors[i]->raw_jacobians`。问题是：

- `raw_jacobians` 在头文件里没有默认初始化，如果 `Evaluate()` 没有被调用就析构，存在未定义行为风险。
- 如果 `Evaluate()` 被调用多次，旧的 `raw_jacobians` 会泄漏。
- 手动 `new[]` / `delete[]` 增加异常路径和早退路径的维护成本。

建议把 `raw_jacobians` 改为：

```cpp
std::vector<double*> raw_jacobians;
```

`Evaluate()` 中 `resize()` 后填充 `jacobians[i].data()`，调用时使用 `raw_jacobians.data()`。

### `parameter_block_data` 使用 RAII

当前线性化点快照通过：

```cpp
double *data = new double[size];
memcpy(data, it->parameter_blocks[i], sizeof(double) * size);
parameter_block_data[addr] = data;
```

析构时手动 `delete[]`。

建议改为：

```cpp
std::unordered_map<std::uintptr_t, std::vector<double>> parameter_block_data;
```

这样可以避免手动释放，也能让 `keep_block_data` 存储 `const double*` 或直接存储 `std::span` 风格视图，减少悬垂指针风险。

---

## 三、参数块索引与类型安全

### 避免用 `long` 保存指针地址

当前大量使用：

```cpp
reinterpret_cast<long>(addr)
```

在 64 位 Linux 上通常可用，但 `long` 并不是表达指针整数的最佳类型。建议统一改为：

```cpp
using ParameterBlockId = std::uintptr_t;
```

并使用：

```cpp
reinterpret_cast<std::uintptr_t>(addr)
```

这样语义更明确，也更利于跨平台。

### 拆分 “是否 drop” 与 “最终局部下标”

`parameter_block_idx` 当前有两层含义：

- `addResidualBlockInfo()` 中把需要 drop 的参数块插入 map，并临时赋值 `0`。
- `marginalize()` 中再把这个 value 改成真正的局部下标。

这会让数据结构语义随阶段变化。建议拆成两个结构：

```cpp
std::unordered_set<ParameterBlockId> drop_blocks;
std::unordered_map<ParameterBlockId, int> local_block_index;
```

这样 `marginalize()` 的流程会更清晰，也能避免调试时误读 `parameter_block_idx` 的 value。

### 固定遍历顺序，提升可复现性

`unordered_map` 的遍历顺序不稳定。当前 `marginalize()` 中 drop block 和 keep block 的排列依赖 `unordered_map` 迭代顺序：

```cpp
for (auto &it : parameter_block_idx) { ... }
for (const auto &it : parameter_block_size) { ... }
```

数学上只要后续索引一致，prior 仍然有效；但排序不稳定会导致：

- 调试输出不稳定。
- 单元测试难以比较精确矩阵。
- 不同标准库实现或不同运行环境下 prior 参数块顺序可能变化。

建议按 `parameter_blocks` 首次出现顺序维护一个 `std::vector<ParameterBlockId> ordered_blocks`，drop block / keep block 都按这个顺序分配局部下标。

### 显式表达 local size

当前：

```cpp
int localSize(int size) const
{
    return size == 7 ? 6 : size;
}
```

隐含假设所有 7 维参数块都是 pose，且局部维度都是 6。短期内符合 VINS-Mono 的 pose 参数化，但工程上应避免只靠 global size 推断 local size。

建议：

- 在添加 residual block 时同时记录每个参数块的 global size 与 local size。
- 对 pose 使用 `ParameterBlockSpec{global_size=7, local_size=6, type=Pose}`。
- 对其他 7 维但非 pose 的参数块，避免被误处理。

---

## 四、数值稳健性

### Schur complement 避免显式求逆

当前对 `Amm` 做特征分解后显式构造：

```cpp
Eigen::MatrixXd Amm_inv = V * S_inv.asDiagonal() * V.transpose();
A = Arr - Arm * Amm_inv * Amr;
b = brr - Arm * Amm_inv * bmm;
```

显式逆矩阵通常更慢，也更容易放大数值误差。

建议把 “乘以 `Amm_inv`” 改为分解求解：

```cpp
X = Amm.ldlt().solve(Amr);
y = Amm.ldlt().solve(bmm);
A = Arr - Arm * X;
b = brr - Arm * y;
```

如果仍需处理半正定和零空间，可以保留 SelfAdjointEigenSolver，但不要物化完整逆矩阵，而是通过特征空间中的缩放完成 solve。

### 检查 Eigen 分解状态

当前没有检查：

```cpp
Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);
Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
```

建议检查：

```cpp
if (saes.info() != Eigen::Success) { ... }
```

失败时应输出参数块维度、drop/keep 数量、`Amm` 范数等诊断信息，并中止本次边缘化或返回可感知错误。

### 使用相对阈值处理小特征值

当前阈值固定为：

```cpp
const double eps = 1e-8;
```

并用绝对值判断特征值是否可逆。不同场景下 residual 权重、特征数量、尺度都会变化，绝对阈值可能过松或过紧。

建议使用相对阈值：

```cpp
double threshold = eps * std::max(1.0, eigenvalues.cwiseAbs().maxCoeff());
```

这样对大尺度和小尺度问题都更稳健。

### 对 Schur 后矩阵再次对称化

`Amm` 做了对称化：

```cpp
Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
```

但 Schur complement 之后的 `A` 没有再次对称化。浮点误差会让 `A` 出现轻微非对称，影响后续 `SelfAdjointEigenSolver`。

建议：

```cpp
A = 0.5 * (A + A.transpose());
```

放在 `saes2` 之前。

### robust loss 修正边界检查

`ResidualBlockInfo::Evaluate()` 中 robust loss 修正使用：

```cpp
double sqrt_rho1_ = sqrt(rho[1]);
const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
const double alpha = 1.0 - sqrt(D);
```

建议补充：

- 检查 `rho[1] > 0` 后再开方和作为分母。
- 检查 `D >= 0`，避免病态 loss 或极端 residual 下返回 NaN。
- 用 `std::sqrt` 并包含 `<cmath>`，避免依赖全局命名空间。

---

## 五、性能与并行化

### 减少线程结构中的大对象拷贝

每个线程都会复制：

```cpp
threadsstruct[i].parameter_block_size = parameter_block_size;
threadsstruct[i].parameter_block_idx = parameter_block_idx;
```

`unordered_map` 拷贝成本不小，且每次边缘化都会发生。建议 `ThreadsStruct` 保存 `const` 指针或引用：

```cpp
const std::unordered_map<ParameterBlockId, int>* parameter_block_size;
const std::unordered_map<ParameterBlockId, int>* parameter_block_idx;
```

线程只读这些 map，不需要复制。

### 避免每个线程分配完整 `A`

当前每个线程都有完整大小的：

```cpp
threadsstruct[i].A = Eigen::MatrixXd::Zero(pos, pos);
threadsstruct[i].b = Eigen::VectorXd::Zero(pos);
```

如果 `pos` 较大，内存占用是 `NUM_THREADS` 倍。对于滑窗内特征较多的场景，这会成为明显峰值内存热点。

可选优化：

- 先保留当前 full matrix 方案，但将 `NUM_THREADS` 动态限制为 `min(NUM_THREADS, factors.size())`。
- 对每个线程只累加它实际触达的 block pair，最后按 block 合并。
- 长期可考虑 block-sparse Hessian 结构，避免 dense `pos x pos` 的中间矩阵。

### 固定 `NUM_THREADS = 4` 不够灵活

头文件中：

```cpp
const int NUM_THREADS = 4;
```

建议：

- 改为 `constexpr int kDefaultNumThreads = 4`。
- 实际线程数取 `std::min(kDefaultNumThreads, static_cast<int>(factors.size()))`。
- 如果工程后续有统一参数配置，可以让边缘化线程数从配置读取。

### 用 `std::thread` / thread pool 替代 pthread

当前使用 `pthread_create()` / `pthread_join()`。在 C++17 工程中，可考虑改为：

- `std::thread`：更符合 C++ RAII 风格。
- 简单 thread pool：避免每次边缘化创建和销毁线程。

短期最小改动是封装一个 `constructAForRange()` 函数，让单线程与多线程路径复用同一份累加逻辑。

### 避免不必要的 Eigen 拷贝

`MarginalizationFactor::Evaluate()` 中每个参数块都会拷贝：

```cpp
Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size);
Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->keep_block_data[i], size);
```

可以改为 `Eigen::Map<const Eigen::VectorXd>` 引用式视图，避免动态分配：

```cpp
Eigen::Map<const Eigen::VectorXd> x(parameters[i], size);
Eigen::Map<const Eigen::VectorXd> x0(marginalization_info->keep_block_data[i], size);
```

`ThreadsConstructA()` 中的：

```cpp
Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
```

也会拷贝矩阵块。可改为 `auto jacobian_i = it->jacobians[i].leftCols(size_i);` 或显式 `Eigen::Ref` / block expression，减少热点路径分配。

---

## 六、接口与头文件边界

### 收敛头文件依赖

`marginalization_factor.h` 当前包含：

```cpp
#include "../utility/logging.h"
#include <cstdlib>
#include <pthread.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
```

其中：

- `logging.h` 只在 `.cpp` 的线程创建失败路径中使用。
- `tic_toc.h` 只在 `.cpp` 的 profiling 局部变量中使用。
- `pthread.h` 只服务于 `.cpp` 的实现细节。
- `Utility` 只有 `MarginalizationFactor::Evaluate()` 的 pose delta 计算需要。

建议头文件只保留声明必要依赖：

```cpp
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>
```

实现细节 include 下沉到 `.cpp`。

### 使用 `override` / `explicit`

建议：

```cpp
explicit MarginalizationFactor(MarginalizationInfo* marginalization_info);

bool Evaluate(double const* const* parameters,
              double* residuals,
              double** jacobians) const override;
```

同时 `ResidualBlockInfo` 构造函数可以考虑 `explicit` 不适用多参构造，但参数应改为按值移动或 `std::vector<double*>&&`，减少不必要拷贝。

### 成员可见性

当前 `ResidualBlockInfo`、`ThreadsStruct`、`MarginalizationInfo` 的成员几乎都是 public。短期为了兼容可先不改，但建议：

- 把只在类内部维护的 map、linearized prior、snapshot 数据改为 private。
- 对 `Estimator` 需要读取的 `keep_block_size` / `keep_block_idx` 提供只读 getter。
- `ThreadsStruct` 放入 `.cpp` 匿名 namespace，避免暴露实现细节。

---

## 七、可测试性

### 增加 Schur complement 小规模单元测试

建议构造一个小型线性 least squares 问题：

- 两个参数块 `x_m` / `x_r`。
- 一个 residual 同时连接两个参数块。
- 手工计算或用 dense 全量消元得到 Schur complement。
- 与 `MarginalizationInfo::marginalize()` 生成的 prior 对比。

这样可以锁住边缘化核心数学流程，后续重构线程、RAII、索引顺序时不容易破坏结果。

### 增加 `MarginalizationFactor::Evaluate()` 回归测试

测试内容：

- 对非 pose 参数块：`dx = x - x0`。
- 对 pose 参数块：平移差和四元数局部扰动是否符合当前 `PoseLocalParameterization` 约定。
- jacobian 输出是否正确填入 `leftCols(local_size)`，剩余列是否置零。

### 增加 robust loss 修正测试

`ResidualBlockInfo::Evaluate()` 中手写了 robust loss 对 residual / jacobian 的修正。建议用简单 1D residual + HuberLoss 构造测试，验证：

- residual scaling 正确。
- jacobian scaling 正确。
- `sq_norm == 0` 时无 NaN。

### 增加内存与生命周期测试

在重构为 RAII 后，可以增加测试覆盖：

- `ResidualBlockInfo::Evaluate()` 连续调用两次不泄漏、不崩溃。
- 未调用 `Evaluate()` 的 `ResidualBlockInfo` 析构安全。
- `MarginalizationInfo` 析构时释放 cost function，但不释放借用的 loss function。

---

## 八、建议实施顺序

1. **低风险清理**
   - 给指针成员默认初始化。
   - `Evaluate()` / 构造函数加 `override` / `explicit`。
   - 头文件 include 下沉。
   - `long` 地址键改为 `std::uintptr_t`。

2. **RAII 与所有权明确**
   - `raw_jacobians` 改为 `std::vector<double*>`。
   - `parameter_block_data` 改为 `std::vector<double>` 存储。
   - 明确 `cost_function` / `loss_function` 所有权策略。

3. **索引与可复现性**
   - 拆分 `drop_blocks` 与 `local_block_index`。
   - 维护稳定的 `ordered_blocks`。
   - 显式记录 global size / local size。

4. **数值稳健性**
   - Eigen 分解状态检查。
   - Schur 后矩阵对称化。
   - 相对特征值阈值。
   - 避免显式构造 `Amm_inv`。

5. **性能优化**
   - 减少线程 map 拷贝和 Eigen block 拷贝。
   - 动态线程数。
   - 长期考虑 block-sparse 累加或 thread pool。

6. **测试补齐**
   - Schur complement 数值回归。
   - prior factor `Evaluate()` 测试。
   - robust loss 修正测试。
   - 生命周期/重复调用测试。
