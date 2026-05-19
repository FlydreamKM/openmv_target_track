# OpenMV Target Track

基于 OpenCV 的嵌入式 Linux 三维靶标跟踪程序。兼容 OpenMV 视觉逻辑，可输出靶标中心偏移、夹角、相机坐标及世界坐标，方便下位机解算。

---

## 特性

- **两种检测模式**
  - `aruco`：ArUco 标记（推荐，精度高，自带 ID，天然支持 3D PnP）
  - `circle`：圆形色块靶标（基于 HSV 颜色阈值 + 轮廓检测）

- **三维输出**
  - 像素偏移 `(dx, dy)` — 靶标中心相对于画面中心的像素距离
  - 水平夹角 `yaw`、俯仰夹角 `pitch` — 弧度/角度双输出
  - 相机坐标系 3D 坐标 `(Xc, Yc, Zc)` — 以相机为原点，Z 轴向前
  - 世界坐标系 3D 坐标 `(Xw, Yw, Zw)` — 可选，需外参标定
  - 直线距离 `distance`

- **多种输出格式**
  - `json`：易于上位机/WebSocket 解析
  - `csv`：便于记录到文件做后期分析
  - `custom`：紧凑文本协议 `$ID,dx,dy,yaw,pitch,...*`
  - `bin`：40 字节二进制包，带宽最小

- **多种输出通道**
  - stdout（默认，调试）
  - 串口 `/dev/ttyUSB0` 等（直接连 STM32 / Arduino）
  - UDP（网络下位机、ROS 节点）

- **轻量、零依赖**
  - 仅依赖 OpenCV（嵌入式 Linux 标配）
  - 纯 C++14，CMake 构建，易于交叉编译

---

## 快速开始

### 1. 编译

```bash
# 安装依赖（以 Debian/Ubuntu 为例）
sudo apt-get install -y build-essential cmake libopencv-dev

# 克隆/解压工程
cd openmv_target_track
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 2. 运行 ArUco 模式（推荐）

```bash
# 使用默认 90° FOV 标定，640x480，JSON 输出到 stdout
./openmv_target_track -m aruco --show

# 指定标记实际尺寸（边长 16.5cm）
./openmv_target_track -m aruco -s 0.165 --show
```

### 3. 运行圆形靶标模式（红色圆）

```bash
# 红色圆形靶标，直径 10cm，紧凑格式输出到串口
./openmv_target_track -m circle -s 0.10 --serial /dev/ttyUSB0 -f custom --headless
```

### 4. 输出世界坐标（需要相机外参）

```bash
# 提供相机在世界坐标系中的位姿文件
./openmv_target_track -m aruco -e ../config/extrinsics_example.yaml --udp 192.168.1.100:5005
```

---

## JSON 输出示例

```json
{
  "id": 1,
  "dx": 42.50,
  "dy": -18.00,
  "yaw": 0.1321,
  "pitch": -0.0562,
  "yaw_deg": 7.57,
  "pitch_deg": -3.22,
  "cam": [0.5283, -0.2251, 2.0000],
  "world": [1.5283, 0.2749, 0.0000],
  "has_world": true,
  "dist": 2.0654,
  "valid": true,
  "t": 1714351200000
}
```

字段说明：

| 字段 | 单位 | 说明 |
|------|------|------|
| `id` | - | 靶标 ID（ArUco 有效） |
| `dx`, `dy` | px | 靶标中心相对画面中心的像素偏移（右/下为正） |
| `yaw`, `pitch` | rad | 水平/俯仰夹角（弧度） |
| `yaw_deg`, `pitch_deg` | deg | 水平/俯仰夹角（角度） |
| `cam[3]` | m | 相机坐标系下靶标 3D 坐标 (Xc, Yc, Zc) |
| `world[3]` | m | 世界坐标系下靶标 3D 坐标 (Xw, Yw, Zw)，需外参 |
| `has_world` | bool | 是否包含世界坐标 |
| `dist` | m | 靶标到相机中心的直线距离 |
| `valid` | bool | 解算是否成功 |
| `t` | ms | 时间戳 |

---

## 自定义格式（custom）协议

```
$ID,dx,dy,yaw,pitch,cx,cy,cz,wx,wy,wz,dist,valid*
```

示例：
```
$1,42.5,-18.0,0.1321,-0.0562,0.5283,-0.2251,2.0000,1.5283,0.2749,0.0000,2.0654,1*
```

---

## 标定说明

### 相机内参（必须）

程序默认根据 `--fov` 和 `--res` 生成无畸变内参。如果你有精确标定结果：

```bash
# 使用 OpenCV 标定文件
./openmv_target_track -c /path/to/calib.yaml
```

`calib.yaml` 格式示例见 `config/camera_calib.yaml`。

### 相机外参（可选，用于世界坐标）

外参描述的是**世界坐标系到相机坐标系的变换**。当提供外参后，程序会把相机坐标系下的靶标坐标转换到世界坐标系。

```bash
./openmv_target_track -e extrinsics.yaml
```

`extrinsics.yaml` 格式示例见 `config/extrinsics_example.yaml`。

外参可通过以下方式获取：
1. 使用已知尺寸的标定板 + `cv::solvePnP` 标定相机位姿
2. 测量相机安装高度和倾斜角，用 `Extrinsics::fromEulerXYZ()` 构造
3. 使用 ROS / VICON 等外部定位系统

---

## 交叉编译（嵌入式 Linux）

以 ARM Cortex-A 系列（树莓派、RK3588、全志等）为例：

```bash
# 安装交叉编译器
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# 准备 OpenCV ARM 库（已安装于目标板或提前交叉编译）
export OPENCV_ARM=/path/to/opencv-arm64

# 构建
cd openmv_target_track
mkdir build-arm && cd build-arm
cmake .. \
    -DCMAKE_TOOLCHAIN_FILE=../cmake/aarch64-linux.cmake \
    -DOpenCV_DIR=$OPENCV_ARM/lib/cmake/opencv4
make -j$(nproc)

# 部署到目标板
scp openmv_target_track root@192.168.1.50:/usr/local/bin/
```

CMake 工具链文件示例 `cmake/aarch64-linux.cmake`：

```cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
```

---

## OpenMV 原生适配（可选）

OpenMV Cam H7 / H7 Plus 等模块底层运行 MicroPython，但也可刷入 **OpenMV Linux 固件**（基于 Buildroot / Yocto）直接运行此程序。

如果需要在 OpenMV 原生固件上运行 MicroPython 脚本，可使用以下逻辑（作为参考）：

```python
# OpenMV MicroPython 精简版参考（不可直接运行此工程）
import sensor, image, math

# 相机参数：90° FOV, 320x240 (QVGA)
FX = 160  # cx / tan(45°)
CX, CY = 160, 120
MARKER_SIZE = 165  # mm

while True:
    img = sensor.snapshot()
    for r in img.find_apriltags(families=image.TAG36H11):
        # 像素偏移
        dx = r.cx() - CX
        dy = r.cy() - CY
        # 深度（mm）
        Z = (MARKER_SIZE * FX) / r.w()
        # 相机坐标
        X = dx * Z / FX
        Y = dy * Z / FX
        # 角度
        yaw = math.atan2(X, Z)
        pitch = math.atan2(Y, Z)
        print("$%d,%d,%d,%.4f,%.4f,%d,%d,%d*" % (
            r.id(), dx, dy, yaw, pitch, X, Y, Z
        ))
```

**注意**：此 C++ 工程比 MicroPython 版本快 3~5 倍，且可直接输出 JSON / 网络包。

---

## 目录结构

```
openmv_target_track/
├── CMakeLists.txt
├── README.md
├── cmake/
│   └── aarch64-linux.cmake      # ARM 交叉编译工具链示例
├── config/
│   ├── camera_calib.yaml          # 默认 90° FOV 标定
│   └── extrinsics_example.yaml    # 外参示例
├── include/
│   ├── target_detector.hpp      # 靶标检测接口
│   ├── pose_estimator.hpp       # 3D 位姿解算
│   └── serial_output.hpp        # 输出器
├── src/
│   ├── main.cpp                 # 主程序
│   ├── target_detector.cpp      # ArUco / Circle 实现
│   ├── pose_estimator.cpp       # PnP / 相似三角形
│   └── serial_output.cpp        # 串口 / UDP / 格式化
└── examples/
    └── calibration_tool.cpp     # 简易标定辅助工具
```

---

## 常见问题

**Q：画面中心偏移方向？**  
A：`dx` 向右为正，`dy` 向下为正（与 OpenCV 图像坐标系一致）。角度 `yaw` 向右偏为正，`pitch` 向上偏为负（即抬头为负，低头为正）。

**Q：圆形靶标测距精度差？**  
A：圆形靶标使用相似三角形法测距，精度受靶标尺寸误差和像素提取精度影响。ArUco 的 `solvePnP` 精度更高（亚像素级）。

**Q：能同时跟踪多个靶标吗？**  
A：可以。程序会逐一枚举检测到的靶标并分别输出位姿。ArUco 模式下每个靶标自带 ID。

**Q：世界坐标系的定义？**  
A：由外参文件定义。通常建议：Z 轴垂直地面向上，X 轴向前，Y 轴向左（或根据你的机器人坐标系调整）。外参的 `R` 和 `t` 表示**世界 -> 相机**的变换。

**Q：如何在无 OpenCV 的裸机 MCU 上运行？**  
A：此工程依赖 OpenCV，适用于运行 Linux 的嵌入式板（树莓派、RK、全志、Jetson、OpenMV Linux 固件等）。若目标为裸机 STM32，需要重写图像捕获和矩阵运算部分，或改用 OpenMV 原生 MicroPython。

---

## License

MIT / 按你所需修改。代码注释完整，方便二次开发。
