# DataGenerator 可视化

查看仿真生成的**真值轨迹**、**IMU**、**归一化射线观测**，不运行 `vins_estimator`。

观测生成原理见 [`../doc/observation_generation.md`](../doc/observation_generation.md)。

```
data_generator/
  config/
    data_generator.yaml   # num_points / fov_deg / imu_per_img
  src/                    # DataGenerator 核心
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

## 多相机支持范围（默认四目）

| 环节 | 状态 | 说明 |
|------|------|------|
| 配置 | ✅ | `simulation_config.yaml`：`num_of_cam: 4`，`camera_ids: [0,1,2,3]`；外参见 `cam_chain.yaml` |
| 观测生成 | ✅ | 每 slot 独立 track；`packed_id = feature_id * num_cam + slot` |
| JSON 导出 | ✅ | `num_cam`、`extrinsics[]`、每条观测含 `camera_id` |
| VIO 仿真 | ✅ | `vins_multi_simulation` 解包并送入 `ImageFrameInput` |
| 离线可视化 | ✅ | 3D 分色射线/视锥；≥4 目时 FOV 为 2×2 面板 |
| 在线可视化 | ✅ | 每路独立视锥 + FOV（四目为右侧 2×2） |

切回单目/双目：修改 `num_of_cam` 与 `camera_ids`（例如 `[0]` 或 `[0,1]`）。

## 多相机验证（四目）

在**仓库根目录**执行以下步骤。预期：各步无 crash，且两路相机均有观测。

### 1. 构建

```bash
cmake -S standalone -B build_standalone -DBUILD_SIM_PYTHON=ON
cmake --build build_standalone --target sim_generator_dump vins_multi_simulation vins_sim_data -j
```

### 2. 导出 JSON 并检查 `num_cam`

```bash
./build_standalone/sim_generator_dump \
  data_generator/vis/output/sim_dump.json \
  1.0 \
  config/simulation/simulation_config.yaml
```

终端应出现 `cam=4`。进一步检查四路观测：

```bash
python3 - <<'PY'
import json
from collections import Counter
with open("data_generator/vis/output/sim_dump.json") as f:
    d = json.load(f)
assert d["num_cam"] == 4, d["num_cam"]
assert d["camera_ids"] == [0, 1, 2, 3], d["camera_ids"]
by_cam = Counter()
for fr in d["image_frames"]:
    by_cam.update(ob["camera_id"] for ob in fr["observations"])
print("OK: num_cam=4, obs by camera:", dict(sorted(by_cam.items())))
assert all(by_cam[c] > 0 for c in (0, 1, 2, 3))
PY
```

### 3. 离线可视化

```bash
pip install -r data_generator/vis/python/requirements.txt
python3 data_generator/vis/python/visualize.py data_generator/vis/output/sim_dump.json
```

无显示器：

```bash
MPLBACKEND=Agg python3 data_generator/vis/python/visualize.py \
  data_generator/vis/output/sim_dump.json --save /tmp/sim_2cam_vis.png
```

加载日志应含 `cam=4`。

### 4. pybind 四目加载

```bash
export PYTHONPATH=$PWD/build_standalone/data_generator_vis:$PYTHONPATH
python3 - <<'PY'
import vins_sim_data as s
from collections import Counter
opts = s.load_options("config/simulation/simulation_config.yaml")
gen = s.DataGenerator(opts, False)
assert gen.num_cameras() == 4
for _ in range(opts.imu_per_img):
    gen.update()
raw = gen.get_image()
slots = Counter(p % gen.num_cameras() for p, _ in raw)
print("OK: num_cameras=4, obs by slot:", dict(sorted(slots.items())))
assert all(slots[k] > 0 for k in range(4))
PY
```

### 5. 在线可视化

```bash
python3 data_generator/vis/python/live_visualize.py \
  --realtime \
  --config config/simulation/simulation_config.yaml
```

启动日志应含 `2 cam(s)`；右侧 FOV 面板与 3D 观测点应随仿真更新（两路合并）。

### 6. 端到端 VIO 仿真

```bash
./build_standalone/vins_multi_simulation config/simulation/simulation_config.yaml
```

或从 `build_standalone` 目录：

```bash
cd build_standalone
./vins_multi_simulation ../config/simulation/simulation_config.yaml
```

预期：运行至 `Simulation done.`，并打印 `[metrics][raw]` / `[metrics][aligned]`；日志中 `Adding feature points` 数量约为单目同场景的约 2 倍（两路观测之和）。

---

## 离线（推荐）

```bash
cd <仓库根目录>
cmake -S standalone -B build_standalone
cmake --build build_standalone --target sim_generator_dump -j

./build_standalone/sim_generator_dump data_generator/vis/output/sim_dump.json 3.0 \
  config/simulation/simulation_config.yaml
pip install -r data_generator/vis/python/requirements.txt
python3 data_generator/vis/python/visualize.py data_generator/vis/output/sim_dump.json
```

无显示器：`MPLBACKEND=Agg python3 data_generator/vis/python/visualize.py ... --save vis.png`

默认**单窗口上下布局**：上方 3D（轨迹 / 路标 / 观测），下方 IMU；界面文字为英文。`--no-imu` 或 `--no-3d` 可只显示其中一部分。

## 在线（pybind）

```bash
cmake -S standalone -B build_standalone -DBUILD_SIM_PYTHON=ON
cmake --build build_standalone --target vins_sim_data -j
export PYTHONPATH=$PWD/build_standalone/data_generator_vis:$PYTHONPATH

python3 data_generator/vis/python/live_visualize.py --realtime \
  --config config/simulation/simulation_config.yaml \
  --dg-config data_generator/config/data_generator.yaml
```

默认同样是上 3D、下 IMU（加速度 + 角速度）；仅 3D 时加 `--no-imu`。快进示例：`--interval-ms 30`。勿使用已废弃的 `--imu`（会与 `--imu-samples` 冲突）。

更完整的参数说明见根目录 [`README.md`](../../README.md) §6.2、[`docs/multi_camera_design.md`](../../docs/multi_camera_design.md) §十二。
