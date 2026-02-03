"""
UWB定位服务器配置文件
"""

# 服务器配置
SERVER_CONFIG = {
    "host": "0.0.0.0",        # 监听所有网络接口
    "port": 8765,             # 监听端口
    "protocol": "tcp",        # 传输协议
    "max_connections": 10,    # 最大连接数
}

# DLL路径配置
DLL_CONFIG = {
    "trilateration_dll": "uwbdemo/trilateration.dll",
}

# 平板终端配置（用于发送定位结果）
TABLET_CONFIG = {
    "host": "127.0.0.1",      # 平板终端IP地址，本机测试使用localhost
    "port": 8766,             # 平板终端端口
    "protocol": "tcp",        # 传输协议
}

# 定位配置
LOCALIZATION_CONFIG = {
    "min_anchors": 3,                 # 最少需要的anchor数量
    "max_range_mm": 50000,            # 最大有效测距（毫米）
    "min_range_mm": 100,              # 最小有效测距（毫米）
    "anchor_height_default": 2.0,     # 默认anchor高度（米）
}

# 输出配置
OUTPUT_CONFIG = {
    "enable_tablet_send": False,      # 是否发送到平板（本机测试设为False）
    "enable_console_print": True,     # 是否打印到控制台
    "print_interval": 10,             # 打印间隔（每10帧打印一次）
}

# 日志配置
LOGGING_CONFIG = {
    "level": "INFO",
    "format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
}

# 虚拟GNSS配置（用于测试）
VIRTUAL_GNSS_CONFIG = {
    "base_lat": 22.2900,
    "base_lon": 114.1700,
    "base_alt": 2.0,
    "anchors": {
        "A0": {"e": 0.0, "n": 0.0, "u": 2.0},
        "A1": {"e": 10.0, "n": 0.0, "u": 2.0},
        "A2": {"e": 5.0, "n": 8.66, "u": 2.0},
    }
}
