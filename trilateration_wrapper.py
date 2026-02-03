"""
三边定位算法包装模块
调用trilateration.dll进行UWB定位计算
"""

import os
import ctypes
import logging
from typing import Optional, Tuple, List
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class Vec3D:
    """三维向量/坐标"""
    x: float
    y: float
    z: float


class UWBMsg(ctypes.Structure):
    """DLL中使用的vec3d结构体"""
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("z", ctypes.c_double)
    ]


class TrilaterationWrapper:
    """三边定位DLL包装类"""
    
    MAX_ANCHORS = 8
    
    def __init__(self, dll_path: str = None):
        """
        初始化三边定位包装器
        
        Args:
            dll_path: trilateration.dll的路径
        """
        self.dll = None
        self.dll_path = dll_path
        
        # 初始化anchor和距离数组
        self.anchor_array = (UWBMsg * self.MAX_ANCHORS)()
        self.distance_array = (ctypes.c_int * self.MAX_ANCHORS)()
        self.location = UWBMsg()
        
        # 初始化距离数组为-1（无效值）
        for i in range(self.MAX_ANCHORS):
            self.distance_array[i] = -1
    
    def load_dll(self, dll_path: str = None) -> bool:
        """
        加载trilateration.dll
        
        Args:
            dll_path: DLL文件路径
            
        Returns:
            加载是否成功
        """
        try:
            path = dll_path or self.dll_path
            if path is None:
                # 默认路径
                cur_path = os.path.dirname(os.path.abspath(__file__))
                path = os.path.join(cur_path, "uwbdemo", "trilateration.dll")
            
            if not os.path.exists(path):
                logger.error(f"DLL文件不存在: {path}")
                return False
            
            self.dll = ctypes.cdll.LoadLibrary(path)
            logger.info(f"成功加载DLL: {path}")
            return True
            
        except OSError as e:
            logger.error(f"加载DLL失败: {e}")
            return False
    
    def set_anchor_position(self, index: int, x: float, y: float, z: float):
        """
        设置Anchor位置
        
        Args:
            index: Anchor索引 (0-7)
            x: X坐标（米）
            y: Y坐标（米）
            z: Z坐标（米）
        """
        if 0 <= index < self.MAX_ANCHORS:
            self.anchor_array[index].x = x
            self.anchor_array[index].y = y
            self.anchor_array[index].z = z
            logger.debug(f"设置Anchor[{index}]位置: ({x:.3f}, {y:.3f}, {z:.3f})")
        else:
            logger.warning(f"无效的Anchor索引: {index}")
    
    def set_anchor_positions(self, positions: List[Tuple[float, float, float]]):
        """
        批量设置Anchor位置
        
        Args:
            positions: Anchor位置列表 [(x0, y0, z0), (x1, y1, z1), ...]
        """
        for i, pos in enumerate(positions):
            if i < self.MAX_ANCHORS:
                self.set_anchor_position(i, pos[0], pos[1], pos[2])
    
    def set_distance(self, index: int, distance_mm: int):
        """
        设置到某个Anchor的测距值
        
        Args:
            index: Anchor索引 (0-7)
            distance_mm: 距离值（毫米），-1表示无效
        """
        if 0 <= index < self.MAX_ANCHORS:
            self.distance_array[index] = distance_mm
        else:
            logger.warning(f"无效的Anchor索引: {index}")
    
    def set_distances(self, distances: List[int]):
        """
        批量设置测距值
        
        Args:
            distances: 测距值列表（毫米）
        """
        # 先将所有距离设为-1
        for i in range(self.MAX_ANCHORS):
            self.distance_array[i] = -1
        
        # 设置有效距离
        for i, dist in enumerate(distances):
            if i < self.MAX_ANCHORS:
                self.distance_array[i] = dist if dist > 0 else -1
    
    def calculate_location(self) -> Optional[Vec3D]:
        """
        计算Tag位置
        
        Returns:
            Tag位置(Vec3D)，计算失败返回None
        """
        if self.dll is None:
            logger.error("DLL未加载")
            return None
        
        try:
            # 调用GetLocation函数
            result = self.dll.GetLocation(
                ctypes.byref(self.location),
                self.anchor_array,
                self.distance_array
            )
            
            if result >= 0:
                position = Vec3D(
                    x=self.location.x,
                    y=self.location.y,
                    z=self.location.z
                )
                logger.debug(f"定位成功: ({position.x:.3f}, {position.y:.3f}, {position.z:.3f})")
                return position
            else:
                logger.warning(f"定位计算失败，错误码: {result}")
                return None
                
        except Exception as e:
            logger.error(f"定位计算异常: {e}")
            return None
    
    def locate(self, anchor_positions: List[Tuple[float, float, float]], 
               distances_mm: List[int]) -> Optional[Vec3D]:
        """
        一站式定位方法
        
        Args:
            anchor_positions: Anchor位置列表 [(x0, y0, z0), ...]
            distances_mm: 对应的测距值列表（毫米）
            
        Returns:
            Tag位置，失败返回None
        """
        self.set_anchor_positions(anchor_positions)
        self.set_distances(distances_mm)
        return self.calculate_location()


class PythonTrilateration:
    """
    纯Python实现的三边定位算法
    用于在无法加载DLL时的备用方案
    """
    
    @staticmethod
    def trilaterate_2d(anchors: List[Tuple[float, float]], 
                       distances: List[float]) -> Optional[Tuple[float, float]]:
        """
        二维三边定位
        
        Args:
            anchors: Anchor二维坐标列表 [(x0, y0), (x1, y1), (x2, y2)]
            distances: 到各Anchor的距离列表（米）
            
        Returns:
            (x, y) 定位结果，失败返回None
        """
        if len(anchors) < 3 or len(distances) < 3:
            logger.error("三边定位需要至少3个anchor和对应的测距值")
            return None
        
        # 提取坐标
        x0, y0 = anchors[0]
        x1, y1 = anchors[1]
        x2, y2 = anchors[2]
        r0, r1, r2 = distances[0], distances[1], distances[2]
        
        # 使用最小二乘法求解
        # 方程组:
        # (x-x0)^2 + (y-y0)^2 = r0^2
        # (x-x1)^2 + (y-y1)^2 = r1^2
        # (x-x2)^2 + (y-y2)^2 = r2^2
        
        # 转化为线性方程组 Ax = b
        # A = [[2(x1-x0), 2(y1-y0)],
        #      [2(x2-x0), 2(y2-y0)]]
        # b = [r0^2-r1^2-x0^2+x1^2-y0^2+y1^2,
        #      r0^2-r2^2-x0^2+x2^2-y0^2+y2^2]
        
        A = [
            [2*(x1-x0), 2*(y1-y0)],
            [2*(x2-x0), 2*(y2-y0)]
        ]
        
        b = [
            r0**2 - r1**2 - x0**2 + x1**2 - y0**2 + y1**2,
            r0**2 - r2**2 - x0**2 + x2**2 - y0**2 + y2**2
        ]
        
        # 求解 2x2 线性方程组
        det = A[0][0]*A[1][1] - A[0][1]*A[1][0]
        if abs(det) < 1e-10:
            logger.warning("矩阵奇异，无法求解")
            return None
        
        x = (A[1][1]*b[0] - A[0][1]*b[1]) / det
        y = (A[0][0]*b[1] - A[1][0]*b[0]) / det
        
        return (x, y)
    
    @staticmethod
    def trilaterate_3d(anchors: List[Tuple[float, float, float]], 
                       distances: List[float]) -> Optional[Tuple[float, float, float]]:
        """
        三维三边定位
        
        Args:
            anchors: Anchor三维坐标列表
            distances: 到各Anchor的距离列表（米）
            
        Returns:
            (x, y, z) 定位结果，失败返回None
        """
        if len(anchors) < 3 or len(distances) < 3:
            logger.error("需要至少3个anchor")
            return None
        
        # 先做2D定位
        anchors_2d = [(a[0], a[1]) for a in anchors[:3]]
        result_2d = PythonTrilateration.trilaterate_2d(anchors_2d, distances[:3])
        
        if result_2d is None:
            return None
        
        x, y = result_2d
        
        # 估算z坐标（假设tag在anchor下方）
        # 使用第一个anchor的高度和距离计算
        x0, y0, z0 = anchors[0]
        r0 = distances[0]
        
        # 水平距离
        horizontal_dist = ((x - x0)**2 + (y - y0)**2)**0.5
        
        # 计算z（假设tag在anchor下方）
        if r0**2 >= horizontal_dist**2:
            dz = (r0**2 - horizontal_dist**2)**0.5
            z = z0 - dz  # tag在anchor下方
        else:
            z = 0.0  # 距离不够，假设在同一高度
        
        # 限制z在合理范围内
        z = max(0.0, min(z, min(a[2] for a in anchors)))
        
        return (x, y, z)


def create_trilateration_wrapper(dll_path: str = None) -> TrilaterationWrapper:
    """
    创建三边定位包装器实例
    
    Args:
        dll_path: DLL路径
        
    Returns:
        TrilaterationWrapper实例
    """
    wrapper = TrilaterationWrapper(dll_path)
    if not wrapper.load_dll():
        logger.warning("无法加载DLL，将使用Python备用算法")
    return wrapper
