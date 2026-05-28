# DataGenerator 可视化

查看仿真生成的**真值轨迹**、**IMU**、**归一化射线观测**，不运行 `vins_estimator`。

```
data_generator/
  src/           # DataGenerator 核心
  vis/
    sim_generator_dump.cpp
    bindings.cpp          # 可选 pybind
    python/
      visualize.py        # 离线 JSON
      live_visualize.py   # 在线（需 pybind）
    output/               # 建议将 sim_dump.json 写在此目录（已 gitignore）
```

## 依赖

- 构建：`CMake`、`Eigen3`（随 standalone 构建）
- Python：`matplotlib`、`numpy`（`pip install -r python/requirements.txt`）
- 在线模式额外需要本机 **pybind11**：`sudo apt install pybind11-dev` 或 `pip3 install pybind11`

## 离线（推荐）

```bash
cd <仓库根目录>
cmake -S standalone -B build_standalone
cmake --build build_standalone --target sim_generator_dump -j

./build_standalone/sim_generator_dump data_generator/vis/output/sim_dump.json
pip install -r data_generator/vis/python/requirements.txt
python3 data_generator/vis/python/visualize.py data_generator/vis/output/sim_dump.json
```

无显示器：`MPLBACKEND=Agg python3 data_generator/vis/python/visualize.py ... --save vis.png`

默认**单窗口上下布局**：上方 3D（轨迹 / 路标 / 观测射线），下方 IMU；界面文字为英文。`--no-imu` 或 `--no-3d` 可只显示其中一部分。

## 在线（pybind）

```bash
cmake -S standalone -B build_standalone -DBUILD_SIM_PYTHON=ON
cmake --build build_standalone --target vins_sim_data -j
export PYTHONPATH=$PWD/build_standalone/data_generator_vis:$PYTHONPATH

python3 data_generator/vis/python/live_visualize.py --realtime
```

默认同样是上 3D、下 IMU（加速度 + 角速度）；仅 3D 时加 `--no-imu`。快进示例：`--interval-ms 30`。勿使用已废弃的 `--imu`（会与 `--imu-samples` 冲突）。

更完整的参数说明见根目录 [`README.md`](../../README.md) §6.2、[`docs/multi_camera_design.md`](../../docs/multi_camera_design.md) §十二。
