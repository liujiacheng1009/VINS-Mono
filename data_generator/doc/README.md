# DataGenerator 文档

本目录说明仿真模块 `DataGenerator` 如何生成 **IMU 观测** 与 **视觉观测**，以及与配置、可视化的关系。

| 文档 | 内容 |
|------|------|
| [observation_generation.md](observation_generation.md) | IMU / 视觉观测的数学模型、生成流程、多目与 ID 打包 |
| [../vis/README.md](../vis/README.md) | 导出 JSON、Python / pybind 可视化用法与**双相机验证** |

相关设计（VIO 侧、多目 MVP）：[`docs/multi_camera_design.md`](../../docs/multi_camera_design.md) §三、§四。

## 双相机（当前默认）

`config/simulation/simulation_config.yaml` 已配置 `num_of_cam: 2`、`camera_ids: [0, 1]`，外参来自同目录 `cam_chain.yaml` / `cam_chain-imucam.yaml`。`DataGenerator` 专用参数见 [`../config/data_generator.yaml`](../config/data_generator.yaml)。

**已支持：** 双路观测生成、JSON 导出（含 `camera_id`）、`vins_multi_simulation` 端到端仿真、离线/在线可视化（观测合并显示；3D 视锥目前仅 cam0）。

完整验证步骤见 [`../vis/README.md` §双相机验证](../vis/README.md#双相机验证)。
