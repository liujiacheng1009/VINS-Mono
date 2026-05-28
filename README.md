# VINS-Multi
## 多相机视觉惯性里程计（Standalone 优先）

**VINS-Multi** 在 [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) 估计器核心之上扩展为**可配置多相机** VIO：支持 Kalibr `cam_chain` 外参、`StateManager`，以及无 ROS 的 standalone 仿真（`num_of_cam` = 1 / 2 / 4）。单相机运行仍作为默认回归基线。

**2019 年 1 月 11 日**（上游）：支持双目 / 双目+IMU / 单目+IMU 的 **VINS** 扩展已发布于 [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)

**2017 年 12 月 29 日**：新功能包括地图合并、位姿图复用、在线时间标定，以及卷帘快门相机支持。地图复用视频：

<a href="https://www.youtube.com/embed/WDpH80nfZes" target="_blank"><img src="http://img.youtube.com/vi/WDpH80nfZes/0.jpg" 
alt="cla" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/eINyJHB34uU" target="_blank"><img src="http://img.youtube.com/vi/eINyJHB34uU/0.jpg" 
alt="icra" width="240" height="180" border="10" /></a>

上游 **VINS-Mono** 是单目视觉惯性里程计的实时 SLAM 框架（滑动窗口、IMU 预积分、在线外参标定等）。**本仓库**保留该优化主干，并增加多相机配置、`StateManager` 重构，以及**非 ROS** 仿真路径。若仍使用原始 catkin 布局，下文 ROS 构建说明依然适用。iOS 端见 [VINS-Mobile](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile)。

**作者：** [Tong Qin](http://www.qintonguav.com)、[Peiliang Li](https://github.com/PeiliangLi)、[Zhenfei Yang](https://github.com/dvorak0)、[Shaojie Shen](http://www.ece.ust.hk/ece.php/profile/facultydetail/eeshaojie)（[香港科技大学空中机器人组](http://uav.ust.hk/)）

**演示视频：**

<a href="https://www.youtube.com/embed/mv_9snb_bKs" target="_blank"><img src="http://img.youtube.com/vi/mv_9snb_bKs/0.jpg" 
alt="euroc" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/g_wN0Nt0VAU" target="_blank"><img src="http://img.youtube.com/vi/g_wN0Nt0VAU/0.jpg" 
alt="indoor_outdoor" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/I4txdvGhT6I" target="_blank"><img src="http://img.youtube.com/vi/I4txdvGhT6I/0.jpg" 
alt="AR_demo" width="240" height="180" border="10" /></a>

EuRoC 数据集；室内外表现；AR 应用；

<a href="https://www.youtube.com/embed/2zE84HqT0es" target="_blank"><img src="http://img.youtube.com/vi/2zE84HqT0es/0.jpg" 
alt="MAV platform" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/CI01qbPWlYY" target="_blank"><img src="http://img.youtube.com/vi/CI01qbPWlYY/0.jpg" 
alt="Mobile platform" width="240" height="180" border="10" /></a>

无人机应用；移动端实现（大陆用户视频：[Video1](http://www.bilibili.com/video/av10813254/) [Video2](http://www.bilibili.com/video/av10813205/) [Video3](http://www.bilibili.com/video/av10813089/) [Video4](http://www.bilibili.com/video/av10813325/) [Video5](http://www.bilibili.com/video/av10813030/)）

**相关论文**

* **Online Temporal Calibration for Monocular Visual-Inertial Systems**, Tong Qin, Shaojie Shen, IROS 2018，**最佳学生论文奖** [pdf](https://ieeexplore.ieee.org/abstract/document/8593603)

* **VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator**, Tong Qin, Peiliang Li, Zhenfei Yang, Shaojie Shen, IEEE Transactions on Robotics [pdf](https://ieeexplore.ieee.org/document/8421746/?arnumber=8421746&source=authoralert)

*若基于本代码开展学术研究，请引用原始 VINS-Mono 论文（以及你自己的扩展工作）。* [bib](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/support_files/paper_bib.txt)

## 1. 环境依赖

### 1.1 Ubuntu 与 ROS

- Ubuntu 16.04
- ROS Kinetic：[ROS 安装](http://wiki.ros.org/ROS/Installation)

额外 ROS 包：

```
    sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```

### 1.2 Ceres Solver

按 [Ceres 安装说明](http://ceres-solver.org/installation.html) 安装 **1.14.0**，并执行 **sudo make install**。（Ceres 2.0.0 及以上版本存在编译问题。）

## 2. ROS 构建（可选，上游布局）

克隆仓库并 catkin_make：

```
    cd ~/catkin_ws/src
    git clone <your-remote> VINS-Multi
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. 公开数据集上的 VIO 与位姿图复用

下载 [EuRoC MAV 数据集](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)。尽管包含双目，我们仅使用单路相机。系统也支持 [ETH-asl cla 数据集](http://robotics.ethz.ch/~asl-datasets/maplab/multi_session_mapping_CLA/bags/)。以下以 EuRoC 为例。

### 3.1 视觉惯性里程计与回环

**3.1.1** 打开三个终端，分别启动 vins_estimator、rviz 并播放 bag。以 MH_01 为例：

```
    roslaunch vins_estimator euroc.launch 
    roslaunch vins_estimator vins_rviz.launch
    rosbag play YOUR_PATH_TO_DATASET/MH_01_easy.bag 
```

（若无法打开 vins_rviz.launch，可打开空白 rviz，再加载配置：文件 → 打开配置 → `YOUR_VINS_FOLDER/config/vins_rviz_config.rviz`）

**3.1.2**（可选）可视化真值。我们提供了一个简单的 benchmark 发布节点，用于将 VINS 与真值对齐显示（仅可视化，**不**用于论文定量评测）：

```
    roslaunch benchmark_publisher publish.launch  sequence_name:=MH_05_difficult
```

（绿线为 VINS 结果，红线为真值。）

**3.1.3**（可选）甚至可以在**无相机-IMU 外参**的情况下运行 EuRoC，系统会在线标定。将第一条命令替换为：

```
    roslaunch vins_estimator euroc_no_extrinsic_param.launch
```

该配置文件中**无外参**。等待数秒完成初标定；有时标定很快完成，观感差异不明显。

### 3.2 地图合并

播放完 MH_01 后，可继续播放 MH_02、MH_03 等，系统会根据回环进行合并。

### 3.3 地图复用

**3.3.1 保存地图**

在配置文件中设置 **pose_graph_save_path**（`YOUR_VINS_FOLDER/config/euroc/euroc_config.yaml`）。播放 MH_01 后，在 vins_estimator 终端输入 **s** 并回车，当前位姿图将被保存。

**3.3.2 加载地图**

在执行 3.1.1 前将 **load_previous_pose_graph** 设为 1。系统从 **pose_graph_save_path** 加载先前位姿图，再播放 MH_02 时新序列会对齐到旧图。

## 4. AR 演示

**4.1** 下载 [bag 文件](https://www.dropbox.com/s/s29oygyhwmllw9k/ar_box.bag?dl=0)（香港科大机器人所采集）。大陆用户：[百度网盘](https://pan.baidu.com/s/1geEyHNl)。

**4.2** 打开三个终端，分别启动 ar_demo、rviz 并播放 bag：

```
    roslaunch ar_demo 3dm_bag.launch
    roslaunch ar_demo ar_rviz.launch
    rosbag play YOUR_PATH_TO_DATASET/ar_box.bag 
```

视野前方会出现 0.8m × 0.8m × 0.8m 的虚拟立方体。

## 5. 使用自有设备

若你熟悉 ROS，且相机与 IMU 能以 ROS topic 提供带物理单位的原始测量，可按下列步骤配置。初学者建议先在 iOS 上试用 [VINS-Mobile](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile)，无需额外搭建环境。

**5.1** 在配置文件中修改 topic 名称。图像频率应高于 20Hz，IMU 高于 100Hz；二者均需准确时间戳。IMU 应包含含重力的绝对加速度。

**5.2 相机标定：**

支持 [针孔模型](http://docs.opencv.org/2.4.8/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html) 与 [MEI 模型](http://www.robots.ox.ac.uk/~cmei/articles/single_viewpoint_calib_mei_07.pdf)。可用任意标定工具，按配置格式写入参数。若使用卷帘快门相机，请仔细标定，使重投影误差小于 0.5 像素。

**5.3 相机-IMU 外参：**

EuRoC 与 AR 配置中可见外参可在线估计与优化。若熟悉坐标变换，可目测或手工测量旋转与平移作为初值写入配置，估计器会在线 refine。若完全未知，可忽略外参并将 **estimate_extrinsic** 设为 **2**，启动时晃动设备数秒；运行成功后标定结果会保存，下次可作初值。示例见 [extrinsic_parameter_example](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/config/extrinsic_parameter_example.pdf)。

**5.4 时间标定：**

多数自研视觉惯性套件未硬件同步。可将 **estimate_td** 设为 1，在线估计相机与 IMU 时间偏移。

**5.5 卷帘快门：**

对已标定、重投影误差 < 0.5 像素的卷帘相机，设 **rolling_shutter** 为 1，并设置读出行时间 **rolling_shutter_tr**（来自数据手册，通常 0–0.05s，非曝光时间）。不建议使用普通网络摄像头。

**5.6** 其他参数见配置文件内说明。

**5.7 不同设备上的性能排序（从高到低）：**

（全局快门 + 同步高端 IMU，如 VI-Sensor）>（全局快门 + 同步低端 IMU）>（全局快门 + 未同步高频 IMU）>（全局快门 + 未同步低频 IMU）>（卷帘 + 未同步低频 IMU）。

## 6. Docker 支持

为简化构建，仓库提供 Docker。Docker 类似沙箱，使运行环境可复现。使用前请安装 [ROS](http://wiki.ros.org/ROS/Installation) 与 [Docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/)，并将账户加入 docker 组：`sudo usermod -aG docker $YOUR_USER_NAME`。**若出现 `Permission denied`，请重新登录或重启终端**，然后执行：

```
cd ~/catkin_ws/src/VINS-Multi/docker
make build
./run.sh LAUNCH_FILE_NAME   # ./run.sh euroc.launch
```

构建时间取决于网络与机器。估计器启动后，在另一终端播放 bag 即可查看结果。修改代码后重新执行 `./run.sh LAUNCH_FILE_NAME` 即可。

### 6.1 无 ROS 运行仿真（native standalone）

主机未安装 ROS 时，可使用内置 standalone 仿真：直接复用 VINS 估计器核心与 `data_generator`，无需 `roslaunch`。

**依赖：**

- CMake（>= 3.10）
- Eigen3
- OpenCV
- Ceres Solver（建议 1.x）

在 Ubuntu 上安装依赖（无 ROS）：

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  pkg-config \
  libeigen3-dev \
  libopencv-dev \
  libgoogle-glog-dev \
  libgflags-dev \
  libsuitesparse-dev \
  libatlas-base-dev
```

安装 Ceres 1.14.0：

```bash
cd /tmp
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
git checkout 1.14.0
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig
```

快速检查：

```bash
cmake --version
pkg-config --modversion opencv4 || pkg-config --modversion opencv
dpkg -s libeigen3-dev | grep Version
```

构建并运行：

```bash
cd <仓库根目录>   # 例如 ~/workspace/VINS-Multi
cmake -S standalone -B build_standalone
cmake --build build_standalone -j$(nproc)
./build_standalone/vins_multi_simulation ./config/simulation/simulation_config.yaml
```

或运行回归测试：

```bash
cd build_standalone && ctest --output-on-failure
```

一条命令构建 standalone：

```bash
cmake -S standalone -B build_standalone && cmake --build build_standalone -j2
```

**输出：**

- 初始化后终端打印估计位置；
- 位姿 CSV 写入配置中的 `output_path`（`vins_result_no_loop.csv`）。

### 6.2 仅可视化仿真数据（不运行 VINS 估计器）

在不运行 `vins_multi_simulation` 的情况下，查看 `DataGenerator` 的真值轨迹、IMU 与单相机观测射线：

```bash
cd <仓库根目录>
cmake -S standalone -B build_standalone
cmake --build build_standalone --target sim_generator_dump -j$(nproc)

./build_standalone/sim_generator_dump data_generator/vis/output/sim_dump.json
pip install -r data_generator/vis/python/requirements.txt
python3 data_generator/vis/python/visualize.py data_generator/vis/output/sim_dump.json
```

无图形界面（如 SSH）保存图片：

```bash
MPLBACKEND=Agg python3 data_generator/vis/python/visualize.py data_generator/vis/output/sim_dump.json --save vis.png
```

默认**单窗口上下拼接**：上方 3D，下方 IMU（界面为英文）。`--no-imu` / `--no-3d` 可只显示一侧。

**可选参数：**

- `sim_generator_dump <output.json> [duration_scale]` — `duration_scale` 默认为 `3.0`（与 `vins_multi_simulation` 一致）。
- `visualize.py --frame-stride 5 --ray-length 3 --max-rays 80` — 3D 图中抽帧并限制射线数量。

**在线查看**（需本机安装 pybind11：`sudo apt install pybind11-dev` 或 `pip3 install pybind11`）：

```bash
cmake -S standalone -B build_standalone -DBUILD_SIM_PYTHON=ON
cmake --build build_standalone --target vins_sim_data -j$(nproc)
export PYTHONPATH=$PWD/build_standalone/data_generator_vis:$PYTHONPATH
python3 data_generator/vis/python/live_visualize.py --realtime
```

默认上 3D、下 IMU；仅 3D 用 `--no-imu`。已移除 `--imu` 参数（勿与 `--imu-samples` 混用）。

更详细说明见 [`data_generator/vis/README.md`](data_generator/vis/README.md) 与 [`docs/multi_camera_design.md`](docs/multi_camera_design.md) §十二。

**指标示例：**

```text
[metrics][raw] samples=1190 mae=0.0753451 rmse=0.0783779 max=0.108481 final=0.10823
[metrics][aligned] samples=1190 mae=0.0752905 rmse=0.0779393 max=0.102094 final=0.101585
[metrics][vel] samples=1190 mae=0.00444835 rmse=0.00560853 max=0.0189432 final=0.00271687
```

## 7. 单元测试

`vins_estimator/src/factor/imu_factor.{h,cpp}` 的单元测试位于 `tests/`，使用 GoogleTest（`sudo apt install libgtest-dev`）。构建与运行：

```bash
cmake -S tests -B build_tests
cmake --build build_tests -j2
ctest --test-dir build_tests --output-on-failure
```

测试覆盖：

- `Integrator` 构造，以及在恒定机体系加速度、角速度下的传播与解析解对比。
- `repropagate()` 与全新传播参考的一致性（状态、雅可比、协方差）。
- 一致状态下 `Integrator::computeResidual` 为零，以及位置小扰动的一阶行为。
- 一致状态下 `IMUFactor::Evaluate` 残差、`nullptr` 雅可比处理，以及四参数块解析雅可比与切空间有限差分对比。
- `IMUFactor` 与 `Integrator` 的 `shared_ptr` 所有权（估计器释放 slot 后 factor 仍持有 integrator）。

## 8. 致谢

非线性优化使用 [Ceres Solver](http://ceres-solver.org/)，回环检测使用 [DBoW2](https://github.com/dorian3d/DBoW2)，相机模型参考 [camodocal](https://github.com/hengli/camodocal)。

## 9. 许可证

源代码以 [GPLv3](http://www.gnu.org/licenses/) 发布。

我们仍在改进代码可靠性。技术问题请联系 Tong QIN &lt;tong.qinATconnect.ust.hk&gt; 或 Peiliang LI &lt;pliapATconnect.ust.hk&gt;。

商业合作请联系 Shaojie SHEN &lt;eeshaojieATust.hk&gt;。
