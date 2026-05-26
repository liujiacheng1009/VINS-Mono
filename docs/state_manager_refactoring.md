# StateManager 重构设计文档

本文档描述从 `Estimator` 中抽取 `StateManager` 的重构方案，用于统一管理 `vins_estimator/src/estimator.h` 中的全部状态变量及其生命周期操作。

关注职责边界、接口契约和迁移路径。§十一 记录**实现状态**（2026-05 已完成核心迁移）。

---

## 一、背景与动机

### 当前问题

`Estimator` 目前同时承担：

- 传感器数据接收与调度（`processIMU` / `processImage`）
- 滑动窗口状态维护（`Ps` / `Vs` / `Rs` / `Bas` / `Bgs` 等）
- Ceres 参数块与 Eigen 状态的双向同步（`vector2double` / `double2vector`）
- 窗口边缘化时的状态搬移（`slideWindow` / `slideWindowOld` / `slideWindowNew`）
- 优化问题构建（`optimization`）

> **说明**：`estimator.h` 中虽存在回环/重定位相关成员（`relocalization_info`、`setReloFrame` 等），但**当前项目未接入该功能**——完整回环闭环属于 **VINS-Fusion** 范畴，`setReloFrame` 在全仓库内无任何调用点，属于遗留死代码。StateManager **不纳入**这部分状态；重构过程中建议同步清理。

状态变量分散在 `estimator.h` 的 40+ 个成员中，访问方式不统一（直接数组下标、裸指针参数块、swap 搬移等），导致：

1. **职责耦合**：任何模块要读/写状态都必须依赖整个 `Estimator`。
2. **生命周期不清晰**：`clearState`、`slideWindow`、`double2vector` 中的 yaw 对齐逻辑分散在三处。
3. **测试困难**：无法在不启动完整 VIO 流水线的情况下单独验证状态搬移与参数同步。
4. **Ceres 边界模糊**：`para_*` 数组与 `Ps`/`Vs`/`Rs` 的映射关系仅存在于 `Estimator` 内部。

### 重构目标

引入 `StateManager`（工作名，最终实现文件名建议 `state_manager.h/.cpp`），作为 VIO 状态的**唯一权威数据源（Single Source of Truth）**，对外提供类型安全的访问接口，并封装所有状态变换操作。

`Estimator` 退化为编排层：接收数据 → 根据 `solver_flag` / `marginalization_flag` 调度 → 调用 `StateManager` 读写**可优化窗口状态** → 触发优化 → 读取结果发布。

**职责划分原则**：StateManager 管理「窗口里有什么、怎么搬移、怎么与 Ceres 同步」；Estimator 管理「当前处于什么阶段、下一步做什么、是否失败重启」。

---

## 二、状态清单与归属

以下按逻辑域划分 `estimator.h` 中应纳入 `StateManager` 的状态。

### 2.1 滑动窗口 IMU 状态（核心）

| 成员 | 类型 | 说明 |
|------|------|------|
| **`FrameId`（新增）** | `FrameId`（`int64_t`） | 在**输入打包段**赋值的全局帧序号；写入 `FrameState.id` 后随窗口搬移；边缘化出窗后仅从 `id_to_slot_` 移除 |
| `Ps[]` | `Vector3d[WINDOW_SIZE+1]` | 各帧 IMU 系位置 |
| `Vs[]` | `Vector3d[WINDOW_SIZE+1]` | 各帧 IMU 系速度 |
| `Rs[]` | `Matrix3d[WINDOW_SIZE+1]` | 各帧 IMU 系旋转 |
| `Bas[]` | `Vector3d[WINDOW_SIZE+1]` | 加速度计 bias |
| `Bgs[]` | `Vector3d[WINDOW_SIZE+1]` | 陀螺仪 bias |
| `Headers[]` | `SimpleHeader[WINDOW_SIZE+1]` | 各帧图像时间戳；其中 `header.frame_id` 为坐标系名字符串（如 `"world"`），**与** `FrameId` **不同** |
| `frame_count` | `int` | 当前窗口**槽位**有效帧数（0 … WINDOW_SIZE）；重构后改为 `slot_count`，与 `FrameId` 解耦 |

#### 槽位索引 vs FrameId

现有代码用数组下标 `i ∈ [0, WINDOW_SIZE]` 同时承担「窗口槽位」和「传给 `FeatureManager` 的逻辑帧号」两种含义，窗口满后 `frame_count` 恒为 `WINDOW_SIZE`，语义混乱。

重构后明确两套标识：

| 概念 | 类型 | 生命周期 | 用途 |
|------|------|----------|------|
| **槽位索引 `slot`** | `int`，0 … WINDOW_SIZE | 随 `slideWindow` 左移/合并而改变 | Ceres `para_Pose[slot]`、IMU 因子 `i/j` 下标、数组存储 |
| **帧 ID `FrameId`** | `int64_t`，单调递增 | **输入打包时**由数据源赋值；进入窗口后写入 `FrameState`；出窗后不可查 | 跨模块按「哪一帧」访问 P/V/R/bias/特征/日志 |

#### FrameId 来源：输入打包段（非 StateManager 分配）

`FrameId` **不应**在 `StateManager::allocateFrameId()` 或 `processImage` 内部递增生成，而应在**传感器数据进入 Estimator 之前**的打包层确定，例如：

- `standalone/simulation_standalone.cpp`：仿真循环每输出一帧图像特征时递增 `frame_seq`；
- 未来的 rosbag / 真机适配层：使用消息序号、相机触发计数或时间对齐后的统一帧号。

建议在 `simple_types.h` 扩展输入载体（二选一）：

```cpp
// 方案 A：扩展 SimpleHeader（最小改动）
struct SimpleHeader {
    SimpleTime stamp;
    std::string frame_id;       // 坐标系名，如 "world"（与 FrameId 无关）
    FrameId seq = kInvalidFrameId;  // 输入打包段写入，单调递增
};

// 方案 B：显式打包结构（推荐，语义更清晰）
struct ImageFrameInput {
    FrameId id;                 // 打包段赋值
    SimpleHeader header;        // stamp + frame_id
    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> features;
};
```

`Estimator::processImage` 签名可演进为接收 `ImageFrameInput`（或从 `header.seq` 读取 `FrameId`），再调用 `StateManager::bindFrame(slot, input.id, input.header)`。**StateManager 只登记、不发明 ID**。

`StateManager` 维护 `slot → FrameState` 与活跃窗口内 `FrameId → slot` 反查表；**推荐对外以 `FrameId` 访问**，槽位接口保留给 Ceres/因子。

> **IMU 与 FrameId 的时序**：`processIMU` 在图像到达前往往只对应「当前积分槽位」，此时尚无 `FrameId`；`FrameId` 在 `processImage` 绑定到该槽位后，该槽位上的预积分缓冲才可通过 `preIntegration(id)` 访问。

### 2.2 外参与时延标定

| 成员 | 类型 | 说明 |
|------|------|------|
| `ric[]` | `Matrix3d[NUM_OF_CAM]` | 相机相对 IMU 旋转 |
| `tic[]` | `Vector3d[NUM_OF_CAM]` | 相机相对 IMU 平移 |
| `td` | `double` | 相机-IMU 时间延迟 |

### 2.3 Ceres 优化参数块

| 成员 | 类型 | 说明 |
|------|------|------|
| `para_Pose[][]` | `double[WINDOW_SIZE+1][SIZE_POSE]` | 位姿参数块 (p + q) |
| `para_SpeedBias[][]` | `double[WINDOW_SIZE+1][SIZE_SPEEDBIAS]` | 速度 + bias 参数块 |
| `para_Ex_Pose[][]` | `double[NUM_OF_CAM][SIZE_POSE]` | 外参参数块 |
| `para_Feature[][]` | `double[NUM_OF_F][SIZE_FEATURE]` | 逆深度参数块 |
| `para_Td[][]` | `double[1][1]` | 时间延迟参数块 |
| `para_Tr[][]` | `double[1][1]` | 滚动快门时间系数 |

> **设计决策**：`para_Feature` 的语义仍由 `FeatureManager` 管理深度索引，`StateManager` 仅持有参数块存储并提供 `double*` 指针；深度的 get/set 通过回调或 `FeatureManager` 协作完成。

### 2.4 IMU 预积分缓冲（每帧）

| 成员 | 类型 | 说明 |
|------|------|------|
| `pre_integrations[]` | `shared_ptr<Integrator>[WINDOW_SIZE+1]` | 帧间预积分器 |
| `dt_buf[]` | `vector<double>[WINDOW_SIZE+1]` | 预积分 dt 序列 |
| `linear_acceleration_buf[]` | `vector<Vector3d>[WINDOW_SIZE+1]` | 原始加速度缓冲 |
| `angular_velocity_buf[]` | `vector<Vector3d>[WINDOW_SIZE+1]` | 原始角速度缓冲 |
| `acc_0`, `gyr_0` | `Vector3d` | 当前帧起始 IMU 测量 |
| `tmp_pre_integration` | `shared_ptr<Integrator>` | 临时预积分（初始化/帧间） |

### 2.5 Estimator 编排层状态（保留在 Estimator，不纳入 StateManager）

以下成员描述**流水线阶段、控制流与系统级常量**，不参与 Ceres 优化，也不随滑动窗口搬移；放在 `Estimator` 更合适。

| 成员 | 类型 | 保留在 Estimator 的原因 |
|------|------|--------------------------|
| `solver_flag` | `SolverFlag` | 控制 INITIAL / NON_LINEAR 阶段分支，属于编排逻辑 |
| `marginalization_flag` | `MarginalizationFlag` | 由 `processImage` 根据视差决策，驱动 `slideWindow` 分支 |
| `g` | `Vector3d` | 来自配置文件的全局常量；VINS-Mono 中不估计重力，仅需在 `processIMU` / 预积分构造时**只读传入** StateManager |
| `first_imu` | `bool` | `processIMU` 首包检测，纯控制流 |
| `failure_occur` | `bool` | 连接 `failureDetection` 与 `double2vector` 重启逻辑，非窗口状态 |
| `initial_timestamp` | `double` | 系统级时间原点，与单帧 `header.stamp` 不同层级 |

**协作方式**：

- `Estimator::processImage` 根据 `solver_flag` 决定是否调用优化；根据 `marginalization_flag` 调用 `state_.slideWindow(mode)`。
- `Estimator::processIMU` 持有 `g`、`first_imu`，调用 `state_.propagateImu(..., g)` 时显式传入重力。
- `Estimator::optimization` 在 `syncFromParameters` 前，若 `failure_occur` 为真，将 `last_R0/last_P0` 作为 yaw 对齐原点**通过参数传入**（见 §4.4），而非由 StateManager 读取 `failure_occur`。

### 2.6 边缘化 prior 缓存（纳入 StateManager 或 Estimator）

滑动窗口挤掉老帧后，需把被移除变量上的约束信息压缩成**先验因子**，在下一轮 Ceres 优化中继续约束剩余变量。当前由 `MarginalizationInfo` 在 `marginalize()` 内组装正规方程 \(A,b\) 并完成 Schur complement；`Estimator` 只保存结果：

| 成员 | 类型 | 说明 |
|------|------|------|
| `last_marginalization_info` | `MarginalizationInfo*` | 上一轮边缘化产出的线性先验（`linearized_jacobians` / `linearized_residuals` 等），下一轮包装为 `MarginalizationFactor` 加入 `Problem` |
| `last_marginalization_parameter_blocks` | `vector<double*>` | 该先验连接的 Ceres 参数块指针；`slideWindow` 后通过 `addr_shift` 映射到新槽位地址 |

`clearState()` / 失败重启时需 `delete last_marginalization_info` 并清空 `parameter_blocks`。

> **归属建议**：与 `para_*` 指针生命周期强绑定，可放在 StateManager（§4.8）或 `Estimator` 内单独持有；**不要**与 §2.10 的废弃成员 `Ap/bp/backup_*` 混淆。

### 2.7 帧间参考快照（失败检测 / 边缘化）

| 成员 | 类型 | 说明 |
|------|------|------|
| `back_R0`, `back_P0` | `Matrix3d`, `Vector3d` | 边缘化前第 0 帧位姿快照 |
| `last_R`, `last_P` | `Matrix3d`, `Vector3d` | 上一帧最新关键帧位姿 |
| `last_R0`, `last_P0` | `Matrix3d`, `Vector3d` | 上一帧窗口首帧位姿 |
| `key_poses` | `vector<Vector3d>` | 输出用关键帧位置序列 |

### 2.8 暂不纳入 StateManager 的成员

以下成员建议保留在 `Estimator` 或独立模块，**不进入 StateManager 范围**：

| 成员 | 建议归属 | 原因 |
|------|----------|------|
| `f_manager` | `FeatureManager`（已有） | 特征跟踪/三角化/深度管理 |
| `point_cloud`, `margin_cloud` | `Estimator` 输出层 | 可视化/调试输出 |
| `sum_of_outlier/back/front/invalid` | `Estimator` 统计 | 调试计数，非状态 |
| `is_valid`, `is_key` | `Estimator` 或移除 | 当前代码中未充分使用 |
| `solver_flag`, `marginalization_flag` | `Estimator` | 见 §2.5 |
| `g`, `first_imu`, `failure_occur`, `initial_timestamp` | `Estimator` | 见 §2.5 |

### 2.9 遗留代码：回环 / 重定位（VINS-Fusion，当前未使用）

`estimator.h` / `estimator.cpp` 中保留了来自 VINS-Fusion 的回环接口与状态，但**本仓库 standalone 仿真路径未接入**，`setReloFrame` 无调用方，`relocalization_info` 恒为 `false`：

| 成员 | 类型 | 说明 |
|------|------|------|
| `setReloFrame()` | 方法 | 无调用点 |
| `relocalization_info` | `bool` | 仅在 `clearState` 中置零 |
| `relo_frame_stamp/index/local_index` | 多种 | 无外部写入 |
| `match_points`, `relo_Pose[]` | — | 无外部写入 |
| `para_Retrive_Pose[]` | `double[SIZE_POSE]` | 仅回环因子使用 |
| `drift_correct_r/t`, `prev_relo_*`, `relo_relative_*` | — | 仅在 `double2vector` 死分支中 |
| `loop_window_index` | `int` | 未使用 |

**重构建议**：StateManager 不管理上述成员；Phase 5 或独立 PR 中直接从 `Estimator` 删除回环相关代码（含 `optimization()` 中的 `relocalization_info` 分支和 `double2vector()` 中的漂移校正逻辑），而非迁移进 StateManager。

### 2.10 遗留代码：正规方程工作区 `Ap` / `bp` / `backup_*`（可删除）

| 成员 | 类型 | 现状 |
|------|------|------|
| `Ap[2]`, `bp[2]` | `MatrixXd`, `VectorXd` | 仅在 `estimator.h` 声明，**全仓库无读写** |
| `backup_A`, `backup_b` | `MatrixXd`, `VectorXd` | 同上 |

**来源**：早期实现在 `Estimator` 内直接维护边缘化正规方程分块与备份；现已在 `MarginalizationInfo::marginalize()` 中使用**局部变量** `A`、`b` 完成组装与 Schur（见 `marginalization_factor.cpp`），不再需要 `Estimator` 上的工作区。

**重构建议**：

- **不纳入** StateManager，**不迁移**。
- 建议在 **Phase 0**（Phase 1 之前）从 `estimator.h` **直接删除** 这四个成员（零行为变更）。
- 文档 §4.8 **不提供** `marginalizationA()` / `backupA()` 等接口。

---

## 三、核心数据结构建议

在实现阶段，建议 `StateManager` 内部采用以下聚合类型，替代散落的数组成员：

```cpp
using FrameId = int64_t;
constexpr FrameId kInvalidFrameId = -1;

// 单帧完整状态（与槽位解耦的语义单元）
struct FrameState {
    FrameId id = kInvalidFrameId;   // 由输入打包段赋值后经 bindFrame 写入；空槽为 kInvalidFrameId
    Vector3d P, V;
    Matrix3d R;
    Vector3d Ba, Bg;
    SimpleHeader header;
};

// 单帧 IMU 侧车数据（与 FrameState 同槽位、同 FrameId 绑定）
struct FrameImuData {
    std::shared_ptr<Integrator> pre_integration;
    std::vector<double> dt_buf;
    std::vector<Vector3d> linear_acceleration_buf;
    std::vector<Vector3d> angular_velocity_buf;
};

// 相机外参
struct ExtrinsicState {
    Matrix3d ric;
    Vector3d tic;
};

// Ceres 参数块视图（不拥有内存，指向 StateManager 内部存储）
struct ParameterBlocks {
    double (*pose)[SIZE_POSE];           // [WINDOW_SIZE + 1]，按 slot 索引
    double (*speed_bias)[SIZE_SPEEDBIAS];
    double (*ex_pose)[SIZE_POSE];       // [NUM_OF_CAM]
    double (*feature)[SIZE_FEATURE];     // [NUM_OF_F]
    double *td;
    double *tr;
};
```

`StateManager` 内部布局：

- `FrameState frames_[WINDOW_SIZE + 1]` — 按 **slot** 存储；
- `FrameImuData imu_data_[WINDOW_SIZE + 1]` — 与 `frames_[slot]` 一一对应，共享同一 `FrameId`；
- `std::unordered_map<FrameId, int> id_to_slot_` — 仅包含**当前窗口内**活跃帧（**不**维护 `next_frame_id_`，ID 计数器在输入打包层）。

`slideWindow` 时 `FrameState` 与 `FrameImuData` **成对** swap/拷贝，`id_to_slot_` 在搬移后重建或增量更新；`FrameId` 随数据迁移，数值不变。Ceres `para_*` 仍按 slot 索引，由 `syncToParameters()` / `syncFromParameters()` 与 `frames_[slot]` 同步。

---

## 四、StateManager 接口设计

以下接口按功能分组。命名采用 snake_case，与现有 VINS-Mono 代码风格一致。

### 4.1 生命周期

```cpp
class StateManager {
public:
    StateManager();

    // 清空窗口状态、预积分缓冲、边缘化 prior（不含 solver_flag 等编排字段）
    void clear();

    // 从全局配置初始化外参/时延（ric/tic/td）；不含 g、solver_flag
    void initFromConfig();
};
```

| 接口 | 对应现有逻辑 | 说明 |
|------|-------------|------|
| `clear()` | `Estimator::clearState()` 中的状态部分 | 清空窗口、预积分、边缘化 prior；**不**重置 `solver_flag` 等（由 Estimator 负责） |
| `initFromConfig()` | `setParameter()` 中的 `ric/tic/td` 部分 | 从 `RIC/TIC/TD` 加载 |

失败重启时 Estimator 依次调用 `state_.clear()`、`state_.initFromConfig()`，并自行重置 `solver_flag = INITIAL`、`failure_occur = false` 等。

### 4.2 滑动窗口帧状态访问

#### 4.2.1 FrameId 登记与查询

```cpp
// 将输入打包段已赋值的 FrameId 绑定到指定槽位（processImage 入口调用）
// 要求 id != kInvalidFrameId，且当前窗口内尚未存在该 id
void bindFrame(int slot, FrameId id, const SimpleHeader& header);

// 当前窗口内是否仍存在该帧（边缘化后返回 false）
bool contains(FrameId id) const;

// 活跃帧数量（初始化阶段 < WINDOW_SIZE+1；满窗后 = WINDOW_SIZE+1）
int activeFrameCount() const;

// 最新 / 最旧活跃帧（按 slot 0 与当前写入槽位）
FrameId latestFrameId() const;
FrameId oldestFrameId() const;

// FrameId → slot；不在窗口内时返回 std::nullopt
std::optional<int> slotOf(FrameId id) const;

// 按时间戳查 FrameId（用于日志、仿真 ground truth 对齐）
std::optional<FrameId> frameIdAtStamp(double stamp_sec) const;
```

| 接口 | 说明 |
|------|------|
| `bindFrame(slot, id, header)` | 写入 `frames_[slot].id`、`header`，更新 `id_to_slot_`；**不**生成新 ID |
| `contains(id)` | 查 `id_to_slot_` |
| `slotOf(id)` | 得到 Ceres/因子用的 slot；边缘化出窗的 ID 无映射 |
| `frameIdAtStamp()` | 线性扫描 `frames_[slot].header.stamp`（窗口 ≤ 11；有 `id` 时优先用 `contains`） |

#### 4.2.2 按 FrameId 访问（推荐主路径）

给定 `FrameId`，可访问该帧全部关联数据，无需关心当前落在哪个 slot：

```cpp
const FrameState& frameState(FrameId id) const;
FrameState&       frameState(FrameId id);

const FrameImuData& imuData(FrameId id) const;
FrameImuData&       imuData(FrameId id);

// 便捷访问（等价于 frameState(id).P 等）
Vector3d& position(FrameId id);
Matrix3d& rotation(FrameId id);
Vector3d& velocity(FrameId id);
Vector3d& accBias(FrameId id);
Vector3d& gyrBias(FrameId id);
const SimpleHeader& header(FrameId id);

std::shared_ptr<Integrator>& preIntegration(FrameId id);
void pushImuSample(FrameId id, double dt, const Vector3d& acc, const Vector3d& gyr);

// Ceres 参数块（内部先 slotOf(id)，再索引 para_*）
double* poseParameter(FrameId id);
double* speedBiasParameter(FrameId id);
```

| 接口 | 聚合的原成员 |
|------|-------------|
| `frameState(id)` | `Ps/Vs/Rs/Bas/Bgs/Headers` @ 该帧 slot |
| `imuData(id)` | `pre_integrations/dt_buf/acc_buf/gyr_buf` @ 该帧 slot |
| `poseParameter(id)` | `para_Pose[slot]` |

**使用示例**（伪代码）：

```cpp
// --- 输入打包段（simulation_standalone / 适配层）---
FrameId id = ++input_frame_seq;  // 或 rosbag 消息序号
ImageFrameInput input{id, header, image};

// --- Estimator::processImage ---
const int slot = state.slotCount();  // 当前写入槽位
state.bindFrame(slot, input.id, input.header);
state.preIntegrationAtSlot(slot) = std::make_shared<Integrator>(...);
// 优化、slideWindow 之后，其他模块仅凭 input.id 访问
state.frameState(input.id).P;
```

#### 4.2.3 按槽位访问（Ceres / 迁移过渡）

```cpp
const FrameState& frameAtSlot(int slot) const;
FrameState&       frameAtSlot(int slot);
const FrameImuData& imuDataAtSlot(int slot) const;

int  slotCount() const;   // 等价原 frame_count
void setSlotCount(int n);

void setFrameStateAtSlot(int slot, const Vector3d& P, const Matrix3d& R, const Vector3d& V);
void initializeFrameAtSlot(int slot, FrameId id, const SimpleHeader& header,
                           const Vector3d& P, const Matrix3d& R, const Vector3d& V,
                           const Vector3d& Ba, const Vector3d& Bg);
```

| 接口 | 对应现有逻辑 | 说明 |
|------|-------------|------|
| `frameAtSlot(slot)` | `Ps[slot]` 等 | `optimization()`、`IMUFactor(i,j)` 等仍用 slot |
| `slotCount()` | `frame_count` | 初始化阶段递增；满窗后恒为 `WINDOW_SIZE` |
| `initializeFrameAtSlot` | `initializeWithGroundTruth()` | GT 路径也须由调用方传入打包段 `FrameId` |

> **迁移建议**：`FeatureManager` 的 `start_frame` 最终应改为存 `FrameId` 而非槽位/逻辑帧号；过渡期可提供 `FrameId feature_start_id` 与 `slot` 互转辅助函数。

### 4.3 外参与时延

```cpp
const ExtrinsicState& extrinsic(int cam_id) const;
ExtrinsicState&       extrinsic(int cam_id);

void setExtrinsic(int cam_id, const Matrix3d& ric, const Vector3d& tic);
void setExtrinsicsFromConfig();   // 从 RIC/TIC 全局变量加载

double timeDelay() const;
void   setTimeDelay(double td);
```

| 接口 | 对应现有逻辑 | 说明 |
|------|-------------|------|
| `extrinsic(i)` | `ric[i]`, `tic[i]` | 供 `f_manager.triangulate` 使用 |
| `timeDelay()` | `td` | 供 `ProjectionFactor` 使用 |
| `setExtrinsicsFromConfig()` | `Estimator` 构造函数 / `setParameter` | 从标定文件加载 |

> **`g` 不在 StateManager**：重力由 `Estimator` 从全局 `G` 加载，经 `propagateImu(..., const Vector3d& gravity)` 传入。

### 4.4 Ceres 参数块桥接

这是 `StateManager` 最核心的接口组，替代 `vector2double()` / `double2vector()`。

```cpp
// Eigen 状态 → Ceres 参数块
void syncToParameters();

// Ceres 参数块 → Eigen 状态（含 yaw 对齐）
// align_origin：失败重启时 Estimator 传入 last_R0/last_P0；正常路径传 std::nullopt，使用 slot 0
struct SyncFromOptions {
    std::optional<Vector3d> align_origin_P0;
    std::optional<Matrix3d> align_origin_R0;  // 仅取 yaw
};
void syncFromParameters(const SyncFromOptions& opts = {});

// 获取参数块指针，供 ceres::Problem::AddParameterBlock 使用
ParameterBlocks parameterBlocks();

// 单独暴露常用指针（可选，便于 migration）
double* poseParameter(int frame_idx);          // para_Pose[i]
double* speedBiasParameter(int frame_idx);     // para_SpeedBias[i]
double* extrinsicParameter(int cam_idx);       // para_Ex_Pose[i]
double* featureParameter(int feature_idx);     // para_Feature[i]
double* timeDelayParameter();                  // para_Td[0]
```

| 接口 | 对应现有逻辑 | 说明 |
|------|-------------|------|
| `syncToParameters()` | `Estimator::vector2double()` | 优化前调用；Pose 用 `[px,py,pz,qx,qy,qz,qw]` 格式 |
| `syncFromParameters(opts)` | `Estimator::double2vector()` | 优化后调用；yaw 对齐；`opts` 替代原 `failure_occur` 分支 |
| `parameterBlocks()` | `optimization()` 中 `AddParameterBlock` | 返回所有裸指针的聚合视图 |

**`syncFromParameters` 需保留的现有语义**（仅活跃路径）：

1. 以 `frame(0)` 的 yaw 为参考，对齐所有帧旋转/位置（`rot_diff` 逻辑）。
2. Euler 奇异点处理（pitch ≈ ±90° 时改用 `Rs[0] * q^T`）。
3. `opts.align_origin_*` 有值时以其作为对齐原点（对应 Estimator 在 `failure_occur` 时传入 `last_R0/P0`）。

建议将 yaw 对齐抽取为私有方法 `applyYawAlignment()`，`syncFromParameters()` 调用之。迁移时可删除 `double2vector()` / `optimization()` 中 `relocalization_info` 相关死分支。

### 4.5 滑动窗口搬移

```cpp
enum class SlideMode { MARGIN_OLD, MARGIN_SECOND_NEW };

// 保存边缘化前第 0 帧快照
void snapshotOldestFrame();

// 关键帧边缘化：整体左移窗口
void slideWindowOld();

// 非关键帧边缘化：合并倒数第二帧与最后一帧
void slideWindowNew();

// 统一入口（等价于 Estimator::slideWindow）
void slideWindow(SlideMode mode);
```

| 接口 | 对应现有逻辑 | 搬移内容 |
|------|-------------|----------|
| `slideWindowOld()` | `slideWindow` + `MARGIN_OLD` | `Ps/Vs/Rs/Bas/Bgs/Headers/pre_integrations/dt_buf/acc_buf/gyr_buf` 全部左移一位；末帧复制倒数第二帧；重建 `pre_integrations[WINDOW_SIZE]` |
| `slideWindowNew()` | `slideWindow` + `MARGIN_SECOND_NEW` | 将末帧 IMU 缓冲合并到 `frame_count-1`；末帧状态复制到 `frame_count-1`；清空末帧缓冲 |
| `snapshotOldestFrame()` | `back_R0 = Rs[0]` 等 | 供 `FeatureManager::removeBackShiftDepth` 使用 |

搬移后需返回或暴露 `back_R0/back_P0` 供 `FeatureManager` 调用：

```cpp
Matrix3d backRotation() const;   // back_R0
Vector3d backPosition() const;   // back_P0
```

### 4.6 IMU 预积分缓冲

预积分与原始 IMU 缓冲已纳入 `FrameImuData`，与 `FrameId` 绑定（见 §4.2.2）。本节仅列 **槽位专用** 与 **全局临时** 接口：

```cpp
std::shared_ptr<Integrator>& tmpPreIntegration();

// 按 slot（processIMU 在帧尚未分配 FrameId 前可能只有 slot）
std::shared_ptr<Integrator>& preIntegrationAtSlot(int slot);
void pushImuSampleAtSlot(int slot, double dt, const Vector3d& acc, const Vector3d& gyr);
void clearImuBufferAtSlot(int slot);
void mergeImuBuffer(FrameId from_id, FrameId to_id);  // slideWindowNew

// 帧间 IMU 传播：gravity 由 Estimator 传入（StateManager 不持有 g）
void propagateImu(FrameId id, double dt, const Vector3d& acc, const Vector3d& gyr,
                  const Vector3d& acc_prev, const Vector3d& gyr_prev,
                  const Vector3d& gravity);
void propagateImuAtSlot(int slot, double dt, ..., const Vector3d& gravity);

// acc_0/gyr_0/first_imu 保留在 Estimator；StateManager 不暴露 firstImuFlag
```

| 接口 | 对应现有逻辑 | 说明 |
|------|-------------|------|
| `preIntegration(id)` | `pre_integrations[slot]` | `IMUFactor` 构造可改为传 `FrameId`，内部 `slotOf` |
| `propagateImu(id)` | `processIMU()` | 通过 `frameState(id)` 读写 P/V/R/Ba/Bg |
| `mergeImuBuffer(from, to)` | `slideWindow` MARGIN_SECOND_NEW | 用 FrameId 表达「末帧合并到前一帧」，避免 slot 下标歧义 |

### 4.7 帧间参考快照

```cpp
void updateKeyframeSnapshot();   // 更新 StateManager 内 last_R/P, last_R0/P0

Matrix3d lastRotation() const;   // last_R — 供 Estimator::failureDetection 使用
Vector3d lastPosition() const;   // last_P
Matrix3d lastRotation0() const;  // last_R0 — 供 failure 时构造 SyncFromOptions
Vector3d lastPosition0() const;  // last_P0
```

| 接口 | 对应现有逻辑 | 说明 |
|------|-------------|------|
| `updateKeyframeSnapshot()` | `processImage` 末尾 | 每帧优化后更新 |
| `lastRotation0/Position0()` | `double2vector` 失败对齐 | Estimator 读取后填入 `SyncFromOptions`，**不**在 StateManager 内读 `failure_occur` |

> **编排标志不在 StateManager**：`solver_flag`、`marginalization_flag`、`failure_occur`、`first_imu`、`initial_timestamp`、`g` 均为 `Estimator` 成员；`slideWindow(mode)` 的 `mode` 由 Estimator 根据 `marginalization_flag` 传入；`slideWindowOld` 中的 `shift_depth` 判断（原 `solver_flag == NON_LINEAR`）留在 Estimator 调用 `FeatureManager` 之前。

### 4.8 边缘化 prior 状态

```cpp
MarginalizationInfo*& lastMarginalizationInfo();
std::vector<double*>& lastMarginalizationParameterBlocks();

void clearMarginalizationPrior();  // 释放 last_marginalization_info
```

> 原 `Ap[]` / `bp[]` / `backup_A` / `backup_b` 为死代码，见 §2.10，实现时勿恢复。

| 接口 | 对应现有逻辑 | 说明 |
|------|-------------|------|
| `clearMarginalizationPrior()` | `clearState()` 中 delete | 窗口重置或失败重启时调用 |
| `lastMarginalizationParameterBlocks()` | `optimization()` 中 prior factor 注册 | 返回 `vector<double*>` 供 `MarginalizationFactor` 使用 |

### 4.9 输出辅助

```cpp
// 收集当前窗口所有帧位置
std::vector<Vector3d> collectKeyframePositions() const;

// 供 triangulate / failureDetection 直接使用的原始数组视图（migration 过渡期）
// 后续应逐步消除
const Vector3d* positionsData() const;   // Ps
const Matrix3d* rotationsData() const;   // Rs
```

> **Migration 说明**：`positionsData()` / `rotationsData()` 是为减少首批改动量提供的过渡接口。长期目标是通过 `frame(i).P` 替代所有裸数组访问。

---

## 五、StateManager 与周边模块的协作关系

```
                    ┌─────────────────┐
                    │    Estimator    │  编排：传感器回调、优化调度
                    └────────┬────────┘
                             │ 持有
                    ┌────────▼────────┐
                    │  StateManager   │  状态权威源
                    └────────┬────────┘
           ┌─────────────────┼─────────────────┐
           │                 │                 │
  ┌────────▼────────┐ ┌──────▼──────┐ ┌───────▼────────┐
  │ FeatureManager  │ │ IMUFactor   │ │ Marginalization│
  │ (深度/特征)      │ │ (预积分残差) │ │ Info / Factor  │
  └─────────────────┘ └─────────────┘ └────────────────┘
```

| 调用方 | 使用的 StateManager 接口 | 说明 |
|--------|-------------------------|------|
| `Estimator::processIMU` | `propagateImu(..., g)`, `preIntegration`, `pushImuSample` | Estimator 持有 `g`/`first_imu`/`acc_0`/`gyr_0` |
| 输入打包段 | `++input_frame_seq` → `ImageFrameInput.id` / `header.seq` | **FrameId 唯一赋值点** |
| `Estimator::processImage` | `bindFrame(slot, id, header)`, `slideWindow(mode)` | 从 `input` 取 id，不内部递增 |
| `Estimator::optimization` | `syncToParameters`, `parameterBlocks`, `syncFromParameters` | Ceres 求解前后同步 |
| `Estimator::failureDetection` | `latestFrameId()`, `frameState(id)`, `lastRotation/Position` | 状态异常检测 |
| `FeatureManager::triangulate` | `positionsData` 或 `frame(i).P`, `extrinsic(i)` | 三角化 |
| `FeatureManager::removeBackShiftDepth` | `backRotation/Position`, `frame(0)`, `extrinsic(0)` | 关键帧边缘化时深度变换 |
| `IMUFactor` | `preIntegration(i)` | 构造时传入预积分器 |
| `ProjectionFactor` | `timeDelay()`, `extrinsic(i)` | 投影残差 |

---

## 六、迁移计划（分阶段）

### Phase 1：封装核心窗口状态 + FrameId（低风险）

1. 在 `simple_types.h` 增加 `FrameId`，扩展 `SimpleHeader::seq` 或新增 `ImageFrameInput`。
2. **输入打包段**（如 `simulation_standalone.cpp`）在组帧时赋值 `FrameId`；`processImage` 传入。
3. 创建 `StateManager`，迁移 `Ps/Vs/Rs/Bas/Bgs/Headers/frame_count`；实现 `bindFrame(slot, id, header)`（**无** `allocateFrameId`）。
4. 实现 `frameState(id)` / `frameAtSlot(slot)`、`clear()`、`initializeFrameAtSlot(..., id, ...)`。
5. `Estimator` 成员改为 `StateManager state_`；编译通过（`FeatureManager` 仍可用 slot，后续 Phase 再改）。

### Phase 2：Ceres 桥接（中风险）

1. 迁移 `para_*` 数组到 `StateManager`。
2. 实现 `syncToParameters()` / `syncFromParameters()` / `parameterBlocks()`。
3. `Estimator::vector2double/double2vector` 变为 thin wrapper 或内联删除。
4. 重点测试：优化前后状态一致性、yaw 对齐。

### Phase 3：窗口搬移与 IMU 缓冲（中风险）

1. 迁移 `pre_integrations/dt_buf/acc_buf/gyr_buf/acc_0/gyr_0`。
2. 实现 `slideWindowOld/New`、`propagateImu`、`mergeImuBuffer`。
3. `Estimator::slideWindow/processIMU` 委托给 `StateManager`。

### Phase 4：外参、边缘化 prior（低风险）

1. 迁移 `ric/tic/td`、边缘化 prior 指针。
2. 实现对应接口。
3. 移除 `Estimator` 中已迁移的成员。

### Phase 0（可选，建议尽早）：删除无行为变更的死代码

1. 从 `estimator.h` 删除 `Ap[]`、`bp[]`、`backup_A`、`backup_b`（§2.10）。
2. （可选）同 PR 删除 §2.9 回环遗留代码。

### Phase 5：清理过渡接口与遗留代码

1. 消除 `positionsData()` 等裸数组视图。
2. `FeatureManager::triangulate` 改为接受 `StateManager&` 或 `array_view<FrameState>`。
3. 若 Phase 0 未做：删除 §2.9 回环/重定位遗留代码（`setReloFrame`、`relocalization_info` 分支、`para_Retrive_Pose` 等）。

---

## 七、不变量与约束

`StateManager` 实现时必须维护以下不变量：

1. **槽位范围**：`0 <= slot_count <= WINDOW_SIZE`；未满窗时仅 `[0, slot_count]` 有效。
2. **FrameId 唯一性**：输入打包段保证全局 `id` 单调且不重复；窗口内 `frames_[i].id != frames_[j].id`（`i != j`）；`id_to_slot_` 与 `frames_[].id` 一致。
3. **边缘化失效**：最老帧滑出窗口后，从其 `id_to_slot_` 删除；`contains(old_id) == false`，但历史日志仍可记录该 ID。
4. **旋转正交性**：`syncFromParameters` 后所有 `R` 应为有效旋转矩阵（`det(R) ≈ 1`）。
5. **参数块同步**：`para_Pose[slot]` 与 `frameAtSlot(slot)` 一致；`poseParameter(frame_id)` 与 `frameState(id)` 一致。
6. **预积分一致性**：`preIntegration(id)` 的 bias 线性化点等于 `frameState(id).Ba/Bg`；窗口搬移后重建末帧预积分器。
7. **边缘化 prior 地址稳定性**：`slideWindow` 后 `last_marginalization_parameter_blocks` 按 `addr_shift` 更新（参数块所有权在 `StateManager`）。
8. **线程安全**：当前为单线程，无需加锁。

---

## 八、文件布局建议

```
vins_estimator/src/
├── state_manager.h          // StateManager 类声明 + FrameState/ExtrinsicState
├── state_manager.cpp        // 全部状态操作实现
├── estimator.h                // 移除已迁移成员，持有 StateManager state_
└── estimator.cpp              // 编排逻辑，委托 state_
```

CMakeLists.txt 中为 `vins_estimator` target 添加 `state_manager.cpp`。

---

## 九、测试建议

| 测试项 | 验证方法 |
|--------|----------|
| `clear()` 后所有状态归零 | 单元测试：检查 frame(0).P.norm() == 0 等 |
| `syncToParameters` ↔ `syncFromParameters` 往返 | 设置已知 P/R/V，sync 双向后误差 < 1e-10 |
| `slideWindowOld` 搬移正确性 | 搬移后 `frameAtSlot(i).id ==` 原 `frameAtSlot(i+1).id`（FrameId 随数据迁移） |
| FrameId 反查 | `bindFrame` 后 `slotOf(id)` 与 `contains(id)` 一致 |
| `slideWindowNew` 合并 IMU 缓冲 | 检查合并后预积分 dt 总和不变 |
| yaw 对齐 | 优化后 frame(0) 的 yaw 与优化前一致 |
| 失败重启 | Estimator 在 `failure_occur` 时向 `syncFromParameters({last_P0, last_R0})` 传入对齐原点 |

---

## 十、开放问题

1. **`para_Feature` 归属**：是否由 `FeatureManager` 持有，`StateManager` 只提供指针？建议 Phase 2 先保留在 `StateManager`，通过 `FeatureManager::getDepthVector()` 协作。
2. **`FeatureManager` 接口改造**：`triangulate(Ps, tic, ric)` 是否改为 `triangulate(StateManager&)`？建议 Phase 5 再改。
3. **`SolverFlag` / `MarginalizationFlag` 枚举位置**：建议移入 `state_manager.h`，或放入独立的 `solver_types.h`。
4. **是否引入 `ImuBufferManager` 子组件**：若 `StateManager` 过大，可将 §2.4 预积分缓冲拆为独立类，由 `StateManager` 组合。
5. **遗留代码清理时机**：`Ap/bp/backup_*`（§2.10）与回环代码（§2.9）均建议在 Phase 1 前删除，避免 StateManager 迁移时携带死成员。
6. **`FeatureManager::start_frame` 类型**：是否改为 `FrameId`？若改，需同步 `removeFront`/`removeBack` 的 ID 递减策略（或改为按 `FrameId` 集合删除）。
7. **输入层 ID 策略**：仿真用本地递增 `seq`；真机是否用相机硬件帧号 / rosbag `header.seq`？需在打包段统一，StateManager 只消费。

---

## 附录：现有成员 → StateManager 接口速查

| estimator.h 成员 | StateManager 接口 |
|-----------------|-------------------|
| `Ps[i]`, `Vs[i]`, `Rs[i]`, `Bas[i]`, `Bgs[i]` | `frameState(id).P/V/R/Ba/Bg` 或 `frameAtSlot(i)` |
| `Headers[i]` | `frameState(id).header` |
| （无，新增） | 输入打包段赋 `FrameId` → `bindFrame()` / `slotOf(id)` |
| `frame_count` | `slotCount()` / `setSlotCount()` |
| `ric[i]`, `tic[i]` | `extrinsic(i).ric/tic` |
| `td` | `timeDelay()` |
| `g` | **Estimator** 成员，经 `propagateImu` 传入 |
| `para_Pose[i]` | `poseParameter(i)` |
| `para_SpeedBias[i]` | `speedBiasParameter(i)` |
| `vector2double()` | `syncToParameters()` |
| `double2vector()` | `syncFromParameters()` |
| `slideWindow()` | `slideWindow(mode)` |
| `pre_integrations[i]` | `preIntegration(frameId)` / `preIntegrationAtSlot(i)` |
| `acc_0`, `gyr_0`, `first_imu` | **Estimator** |
| `solver_flag`, `marginalization_flag`, `failure_occur`, `initial_timestamp` | **Estimator** |
| `back_R0`, `back_P0` | `backRotation()`, `backPosition()` |
| `last_R/P/R0/P0` | `lastRotation/Position/Rotation0/Position0()` |
| `last_marginalization_info` | `lastMarginalizationInfo()` |
| `Ap[]`, `bp[]`, `backup_A`, `backup_b` | **删除**（§2.10，勿纳入 StateManager） |
| `relocalization_info` 等（遗留） | **不纳入** StateManager，建议删除（§2.9） |

---

## 十一、实现状态检查清单

> 对照 §六 迁移阶段与 §4 接口契约。最后验证：`cmake -S standalone -B build_standalone && cmake --build build_standalone`，`./vins_simulation_standalone` 指标与 README 一致（raw mae≈0.0753451）。

| 项 | 状态 | 说明 |
|----|------|------|
| **Phase 0** 删除 `Ap/bp/backup_*` | ✅ | `estimator.h` 已无该成员 |
| **Phase 0/5** 删除回环/重定位遗留 | ✅ | `vins_estimator` 内无 `relocalization_info` / `setReloFrame` 等 |
| **Phase 1** `FrameId`、`ImageFrameInput`、输入层赋 ID | ✅ | `utility/simple_types.h`、`utility/image_frame_input.h`、`simulation_standalone.cpp` |
| **Phase 1** `StateManager` 窗口状态、`bindFrame`、`frameState`/`frameAtSlot` | ✅ | `state_manager.h/.cpp` |
| **Phase 1** `Estimator` 持有 `state_` | ✅ | `estimator.h` |
| **Phase 2** `para_*`、`syncTo/FromParameters`、`applyYawAlignment` | ✅ | `syncStateToParameters` / `syncParametersToState` 为 thin wrapper |
| **Phase 2** `parameterBlocks()` | ✅ | 已实现；`optimization()` 仍直接调 `poseParameter` 等（等价） |
| **Phase 3** IMU 缓冲、`slideWindow*`、`propagateImu`、`mergeImuBuffer` | ✅ | |
| **Phase 4** `ric/tic/td`、边缘化 prior、`extrinsic()` | ✅ | |
| **Phase 4** 删除 `Estimator` 已迁移成员 | ✅ | 窗口/Ceres/IMU 状态均在 `StateManager` |
| **Phase 5** 删除 `key_poses` | ✅ | |
| **Phase 5** `FeatureManager::triangulate(StateManager&)` | ✅ | `feature_manager.cpp` |
| **Phase 5** 消除 `positionsData()` 过渡接口 | ✅ | 未引入；通过 `positionAtSlot` / `frameState` / `friend` 访问 |
| **Phase 5** 窗口数组私有化 | ✅ | `Ps_/Vs_/...` 私有；无 `friend`，经公开访问器 |
| **§4.7** `backRotation/Position`、`last*` 快照 | ✅ | |
| **§九** 单元测试 | ✅ | `state_manager_smoke_test` + `ctest` 仿真指标回归 |
| **§十-6** `FeatureManager::start_frame` → `FrameId` | ✅（语义澄清） | 重命名为 `start_slot`（窗口槽位，非 `FrameId`） |
| **§十-1** `para_Feature` 归属 | ✅ | 暂留 `StateManager`，与 `FeatureManager::getDepthVector` 协作 |
| **ROS 主路径** `vins_estimator` catkin 目标 | ⚠️ 未在本仓库验证 | standalone 路径已覆盖核心状态机 |

### 验证记录

```bash
cmake -S standalone -B build_standalone && cmake --build build_standalone
cd build_standalone && ctest --output-on-failure
# 或单独：./build_standalone/vins_simulation_standalone
#         ./standalone/check_simulation_metrics.sh
```

```
[metrics][raw] mae=0.0753448 rmse=0.0783776
[metrics][aligned] mae=0.0752903 rmse=0.0779392
[metrics][vel] mae=0.00444835
```
