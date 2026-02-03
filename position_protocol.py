"""
Position协议模块
根据position.proto定义，实现位置数据的序列化和发送
支持Protocol Buffers格式和JSON格式
"""

import time
import struct
import json
import logging
from typing import Dict, List, Optional
from dataclasses import dataclass, asdict
from enum import IntFlag

logger = logging.getLogger(__name__)


class SourceMask(IntFlag):
    """传感器源掩码"""
    NONE = 0
    UWB = 0b001      # Bit 0: UWB
    IMU = 0b010      # Bit 1: IMU
    GNSS = 0b100     # Bit 2: GNSS
    UWB_IMU = 0b011  # UWB + IMU
    UWB_GNSS = 0b101  # UWB + GNSS
    ALL = 0b111      # UWB + IMU + GNSS


@dataclass
class Position:
    """
    单个设备的位置数据（WGS84坐标系）
    对应position.proto中的Position消息
    """
    device_id: int           # 设备ID (1-255)
    latitude: float          # 纬度 [-90.0, 90.0]
    longitude: float         # 经度 [-180.0, 180.0]
    altitude: float          # 高度（WGS84椭球高度，非MSL）
    source_mask: int         # 传感器源掩码
    device_timestamp: int    # 设备时间戳（微秒）
    
    # 扩展字段（非proto定义，用于内部使用）
    local_x: float = 0.0     # 本地X坐标（米）
    local_y: float = 0.0     # 本地Y坐标（米）
    local_z: float = 0.0     # 本地Z坐标（米）
    
    def to_proto_bytes(self) -> bytes:
        """
        序列化为简化的二进制格式
        格式: device_id(1B) + lat(8B) + lon(8B) + alt(8B) + source_mask(4B) + timestamp(8B)
        总共37字节
        """
        return struct.pack(
            '>B3dfQ',  # Big-endian: uint8 + 3*double + uint32 + uint64
            self.device_id,
            self.latitude,
            self.longitude,
            self.altitude,
            self.source_mask,
            self.device_timestamp
        )
    
    @classmethod
    def from_proto_bytes(cls, data: bytes) -> 'Position':
        """从二进制数据反序列化"""
        device_id, lat, lon, alt, source, timestamp = struct.unpack('>B3dfQ', data[:37])
        return cls(
            device_id=device_id,
            latitude=lat,
            longitude=lon,
            altitude=alt,
            source_mask=source,
            device_timestamp=timestamp
        )
    
    def to_json(self) -> str:
        """序列化为JSON字符串"""
        return json.dumps({
            "device_id": self.device_id,
            "latitude": self.latitude,
            "longitude": self.longitude,
            "altitude": self.altitude,
            "source_mask": self.source_mask,
            "device_timestamp": self.device_timestamp,
            "local_position": {
                "x": self.local_x,
                "y": self.local_y,
                "z": self.local_z
            }
        })
    
    def format_display(self) -> str:
        """格式化为人类可读的显示字符串"""
        source_str = []
        if self.source_mask & SourceMask.UWB:
            source_str.append("UWB")
        if self.source_mask & SourceMask.IMU:
            source_str.append("IMU")
        if self.source_mask & SourceMask.GNSS:
            source_str.append("GNSS")
        source_display = "+".join(source_str) if source_str else "NONE"
        
        return (
            f"设备ID: {self.device_id}\n"
            f"WGS84坐标:\n"
            f"  纬度: {self.latitude:.6f}°\n"
            f"  经度: {self.longitude:.6f}°\n"
            f"  高度: {self.altitude:.2f}m\n"
            f"本地坐标:\n"
            f"  X: {self.local_x:.3f}m\n"
            f"  Y: {self.local_y:.3f}m\n"
            f"  Z: {self.local_z:.3f}m\n"
            f"数据源: {source_display}\n"
            f"时间戳: {self.device_timestamp}"
        )


@dataclass
class PositionBatch:
    """
    位置数据批次
    对应position.proto中的PositionBatch消息
    """
    server_timestamp: int    # 服务器时间戳（微秒）
    positions: List[Position]  # 位置列表
    
    def to_proto_bytes(self) -> bytes:
        """序列化为二进制格式"""
        # 格式: server_timestamp(8B) + count(2B) + positions...
        header = struct.pack('>QH', self.server_timestamp, len(self.positions))
        positions_data = b''.join(p.to_proto_bytes() for p in self.positions)
        return header + positions_data
    
    def to_json(self) -> str:
        """序列化为JSON字符串"""
        return json.dumps({
            "server_timestamp": self.server_timestamp,
            "positions": [
                {
                    "device_id": p.device_id,
                    "latitude": p.latitude,
                    "longitude": p.longitude,
                    "altitude": p.altitude,
                    "source_mask": p.source_mask,
                    "device_timestamp": p.device_timestamp,
                    "local_position": {
                        "x": p.local_x,
                        "y": p.local_y,
                        "z": p.local_z
                    }
                }
                for p in self.positions
            ]
        })


class PositionSender:
    """位置数据发送器"""
    
    def __init__(self, host: str, port: int, protocol: str = "tcp"):
        """
        初始化位置发送器
        
        Args:
            host: 目标主机地址
            port: 目标端口
            protocol: 传输协议
        """
        self.host = host
        self.port = port
        self.protocol = protocol.lower()
        self.socket = None
        self.connected = False
        
    def connect(self) -> bool:
        """连接到平板终端"""
        import socket
        try:
            if self.protocol == "tcp":
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(5.0)
                self.socket.connect((self.host, self.port))
            else:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            self.connected = True
            logger.info(f"已连接到平板终端: {self.host}:{self.port}")
            return True
            
        except Exception as e:
            logger.warning(f"连接平板终端失败: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        self.connected = False
    
    def send_position(self, position: Position, use_json: bool = True) -> bool:
        """
        发送单个位置数据
        
        Args:
            position: Position对象
            use_json: 是否使用JSON格式（否则使用二进制）
            
        Returns:
            发送是否成功
        """
        if not self.connected:
            if not self.connect():
                return False
        
        try:
            if use_json:
                data = position.to_json().encode('utf-8')
            else:
                data = position.to_proto_bytes()
            
            # 添加长度前缀
            length_prefix = len(data).to_bytes(4, byteorder='big')
            
            if self.protocol == "tcp":
                self.socket.sendall(length_prefix + data)
            else:
                self.socket.sendto(data, (self.host, self.port))
            
            return True
            
        except Exception as e:
            logger.error(f"发送位置数据失败: {e}")
            self.connected = False
            return False
    
    def send_batch(self, batch: PositionBatch, use_json: bool = True) -> bool:
        """发送位置批次数据"""
        if not self.connected:
            if not self.connect():
                return False
        
        try:
            if use_json:
                data = batch.to_json().encode('utf-8')
            else:
                data = batch.to_proto_bytes()
            
            length_prefix = len(data).to_bytes(4, byteorder='big')
            
            if self.protocol == "tcp":
                self.socket.sendall(length_prefix + data)
            else:
                self.socket.sendto(data, (self.host, self.port))
            
            return True
            
        except Exception as e:
            logger.error(f"发送批次数据失败: {e}")
            self.connected = False
            return False


def create_position_from_local(
    device_id: int,
    local_x: float,
    local_y: float,
    local_z: float,
    coordinate_converter,
    source_mask: int = SourceMask.UWB
) -> Position:
    """
    从本地坐标创建Position对象
    
    Args:
        device_id: 设备ID
        local_x: 本地X坐标（米）
        local_y: 本地Y坐标（米）
        local_z: 本地Z坐标（米）
        coordinate_converter: 坐标转换器
        source_mask: 传感器源掩码
        
    Returns:
        Position对象
    """
    from coordinate_converter import PlaneCoordinate
    
    # 转换为GNSS坐标
    plane_coord = PlaneCoordinate(x=local_x, y=local_y, z=local_z)
    gnss_coord = coordinate_converter.plane_to_gnss(plane_coord)
    
    # 创建Position对象
    return Position(
        device_id=device_id,
        latitude=gnss_coord.lat,
        longitude=gnss_coord.lon,
        altitude=gnss_coord.alt,
        source_mask=source_mask,
        device_timestamp=int(time.time() * 1_000_000),  # 微秒
        local_x=local_x,
        local_y=local_y,
        local_z=local_z
    )


def print_position_standard(position: Position):
    """
    按标准格式打印位置信息
    
    Args:
        position: Position对象
    """
    print("=" * 60)
    print("           UWB定位结果 (Position Protocol)")
    print("=" * 60)
    print(position.format_display())
    print("=" * 60)
    print()
