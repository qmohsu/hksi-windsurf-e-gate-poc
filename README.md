# UWB Localization Server (UWB_RPi_HKSI)

UWB 定位服务器 - 接收 UWB 测距数据并计算 Tag 位置

## 项目简介

本项目为 UWB 定位系统的服务器端，接收树莓派转发的 UWB 测距数据，使用三边定位算法计算 Tag 的实时位置，并将定位结果按照标准协议输出到平板终端。

## 系统架构

```
┌─────────────┐      TCP       ┌─────────────────┐      TCP       ┌─────────────┐
│   树莓派    │ ────────────► │    本服务器      │ ────────────► │  平板终端   │
│ (数据转发)  │               │  (定位计算)      │               │ (结果显示)  │
└─────────────┘               └─────────────────┘               └─────────────┘
                                     │
                                     │ 调用
                                     ▼
                              ┌─────────────┐
                              │ trilateration│
                              │    .dll      │
                              │ (定位算法)   │
                              └─────────────┘
```

## 功能特性

- 接收多个树莓派客户端的 UWB 测距数据
- 支持 GNSS 坐标到本地平面坐标转换
- 调用 `trilateration.dll` 进行三边定位计算
- 支持 Python 备用定位算法
- 按照 `position.proto` 标准输出定位结果
- 支持发送定位结果到平板终端

## 运行环境

- **操作系统**: Windows 10/11, Linux
- **Python**: Python 3.7 或更高版本
- **DLL**: trilateration.dll（仅 Windows 需要，Linux 可使用 Python 备用算法）

## 安装步骤

### 1. 安装 Python 依赖

```bash
cd UWB_RPi_HKSI
pip install -r requirements.txt
```

### 2. 确认 DLL 文件位置

确保 `trilateration.dll` 位于 `uwbdemo/` 目录下：

```
UWB_RPi_HKSI/
└── uwbdemo/
    └── trilateration.dll
```

## 配置说明

编辑 `config.py` 文件：

```python
# 服务器配置
SERVER_CONFIG = {
    "host": "0.0.0.0",    # 监听地址，0.0.0.0 表示所有网络接口
    "port": 8765,         # 监听端口
    "protocol": "tcp",
}

# 平板终端配置
TABLET_CONFIG = {
    "host": "192.168.1.200",  # 平板 IP 地址
    "port": 9000,              # 平板端口
    "protocol": "tcp",
}

# 输出配置
OUTPUT_CONFIG = {
    "enable_console_print": True,   # 是否在控制台打印
    "enable_tablet_send": False,    # 是否发送到平板（测试时关闭）
    "print_interval": 10,           # 每 N 帧打印一次
}

# 虚拟 GNSS 配置（测试用）
VIRTUAL_GNSS_CONFIG = {
    "base_lat": 22.2900,      # 基准纬度
    "base_lon": 114.1700,     # 基准经度
    "base_alt": 2.0,          # 基准高度
    "anchors": {
        "A0": {"e": 0.0, "n": 0.0, "u": 2.0},
        "A1": {"e": 10.0, "n": 0.0, "u": 2.0},
        "A2": {"e": 5.0, "n": 8.66, "u": 2.0},
    }
}
```

## 使用方法

### 启动服务器

```bash
# 使用默认配置
python main.py

# 指定监听地址和端口
python main.py --host 0.0.0.0 --port 8765

# 启用调试模式
python main.py --debug
```

### 命令行参数

| 参数 | 简写 | 说明 | 默认值 |
|------|------|------|--------|
| --host | -H | 监听 IP 地址 | 0.0.0.0 |
| --port | -p | 监听端口 | 8765 |
| --debug | -d | 启用调试日志 | 否 |

## 数据流程

### 1. 接收数据

服务器接收 JSON 格式的 UWB 测距数据：

```json
{
    "type": "uwb_range",
    "timestamp": 1706789012.345,
    "data": {
        "ranges": [7432, 3970, 6518, -1, -1, -1, -1, -1],
        "tag_id": "0"
    },
    "gnss": {
        "A0": {"lat": 22.29, "lon": 114.17, "e": 0, "n": 0, "u": 2},
        "A1": {"lat": 22.29, "lon": 114.1701, "e": 10, "n": 0, "u": 2},
        "A2": {"lat": 22.2901, "lon": 114.17, "e": 5, "n": 8.66, "u": 2}
    }
}
```

### 2. 坐标转换

将 GNSS (ENU) 坐标转换为以 A0 为原点的平面坐标：

```
A0: (0, 0, 2) m     ← 原点
A1: (10, 0, 2) m
A2: (5, 8.66, 2) m
```

### 3. 定位计算

调用 `trilateration.dll` 的 `GetLocation` 函数：

```python
# 输入: Anchor 坐标 + 测距值
# 输出: Tag 的 (x, y, z) 坐标
result = GetLocation(location, anchorArray, distanceArray)
```

### 4. 输出结果

按照 `position.proto` 标准格式输出：

```
============================================================
           UWB定位结果 (Position Protocol)
============================================================
设备ID: 0
WGS84坐标:
  纬度: 22.290031°
  经度: 114.170077°
  高度: 2.51m
本地坐标:
  X: 7.957m
  Y: 3.446m
  Z: 0.506m
数据源: UWB
时间戳: 1769790091243553
============================================================
```

## 定位算法说明

### trilateration.dll

本项目使用 C++ 编写的三边定位算法，主要函数：

- `GetLocation(location, anchorArray, distanceArray)`: 主入口函数
- `deca_3dlocate()`: 3D 定位核心函数
- `trilateration()`: 三球交点计算

### Python 备用算法

当 DLL 加载失败时，自动使用 Python 实现的备用算法：

```python
# 2D 三边定位
PythonTrilateration.trilaterate_2d(anchors, distances)

# 3D 三边定位
PythonTrilateration.trilaterate_3d(anchors, distances)
```

## 项目结构

```
UWB_RPi_HKSI/
├── main.py                  # 主程序入口
├── config.py                # 配置文件
├── network_server.py        # 网络服务器模块
├── coordinate_converter.py  # 坐标转换模块
├── trilateration_wrapper.py # DLL 封装模块
├── position_protocol.py     # 定位协议模块
├── requirements.txt         # Python 依赖
├── position.proto           # 协议定义文件
├── README.md               # 本文件
└── uwbdemo/
    ├── trilateration.dll    # 定位算法 DLL
    ├── UWB_demo.py          # DLL 使用示例
    └── vs2010 project/      # DLL 源代码
```

## 本地测试

### 同时运行服务器和客户端

**终端 1 - 启动服务器：**

```bash
cd C:\Users\cunyi\Desktop\cursor_code\UWB_RPi_HKSI
python main.py
```

**终端 2 - 启动虚拟客户端：**

```bash
cd C:\Users\cunyi\Desktop\cursor_code\RPi_UWB_Forwarder
python main.py --mode virtual
```

### 预期输出

服务器端：
```
[服务器] 新客户端连接: ('127.0.0.1', 55831)
[服务器] 收到GNSS坐标更新
[服务器] 收到UWB测距数据: [8616, 3974, 6524]
[服务器] 处理第 1 帧数据...
[服务器] 定位成功! 位置: (7.92, 3.03, 0.48) m
```

客户端：
```
[客户端] TCP连接到服务器 127.0.0.1:8765 成功
[客户端] 已发送 10 帧, 测距值: [8616, 3974, 6524] mm
[客户端] 当前虚拟Tag位置: (7.92, 3.03, 0.50) m
```

## Position Protocol 协议

定位结果按照 `position.proto` 定义的格式输出：

```protobuf
message Position {
  uint32 device_id = 1;       // 设备ID
  double latitude = 2;        // 纬度
  double longitude = 3;       // 经度
  double altitude = 4;        // 高度
  fixed32 source_mask = 5;    // 数据源掩码
  uint64 device_timestamp = 6; // 设备时间戳
}

message PositionBatch {
  uint64 server_timestamp = 1;
  repeated Position positions = 2;
}
```

### 数据源掩码 (source_mask)

| 值 | 含义 |
|----|------|
| 0x01 | UWB |
| 0x02 | GPS |
| 0x04 | IMU |
| 0x08 | WiFi |

## 故障排除

### DLL 加载失败

```
警告: 无法加载trilateration.dll，将使用Python备用算法
```

解决方法：
1. 确认 DLL 文件存在于正确位置
2. 安装 Visual C++ Redistributable
3. 使用 Python 备用算法（自动切换）

### 定位失败

检查以下情况：
1. 测距值是否在有效范围内（100mm ~ 100000mm）
2. 有效 Anchor 数量是否 >= 3
3. Anchor 坐标是否正确设置

### 客户端无法连接

```bash
# 检查端口是否被占用
netstat -ano | findstr 8765

# 检查防火墙设置
# Windows: 允许 Python 通过防火墙
```

## 注意事项

1. **DLL 兼容性**: `trilateration.dll` 仅支持 Windows，Linux 下自动使用 Python 备用算法
2. **坐标系统**: 本地坐标以 A0 为原点，X 轴指向东，Y 轴指向北
3. **单位**: 测距值单位为毫米(mm)，坐标单位为米(m)
4. **精度**: DLL 算法精度通常优于 Python 备用算法

## 许可证

本项目仅供学习和研究使用。
