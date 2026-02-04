"""
三边定位算法包装模块
调用trilateration native library进行UWB定位计算
支持跨平台：Windows (.dll), Linux (.so)
"""

import os
import ctypes
import platform
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
    """三边定位Native Library包装类（跨平台）"""
    
    MAX_ANCHORS = 8
    
    def __init__(self, library_path: str = None):
        """
        初始化三边定位包装器
        
        Args:
            library_path: Native library路径（可选，自动检测平台）
        """
        self.library = None
        self.dll = None  # Backward compatibility
        self.library_path = library_path
        self.platform_info = self._detect_platform()
        
        # 初始化anchor和距离数组
        self.anchor_array = (UWBMsg * self.MAX_ANCHORS)()
        self.distance_array = (ctypes.c_int * self.MAX_ANCHORS)()
        self.location = UWBMsg()
        
        # 初始化距离数组为-1（无效值）
        for i in range(self.MAX_ANCHORS):
            self.distance_array[i] = -1
    
    def _detect_platform(self) -> dict:
        """
        检测当前平台信息
        
        Returns:
            包含平台信息的字典
        """
        system = platform.system()
        machine = platform.machine().lower()
        
        # Normalize architecture names
        if machine in ('x86_64', 'amd64', 'x64'):
            arch = 'x86_64'
        elif machine in ('aarch64', 'arm64'):
            arch = 'arm64'
        elif machine in ('armv7l', 'armv7'):
            arch = 'armv7'
        else:
            arch = machine
        
        info = {
            'system': system,
            'machine': machine,
            'arch': arch,
            'is_windows': system == 'Windows',
            'is_linux': system == 'Linux',
            'is_macos': system == 'Darwin',
            'is_rpi': system == 'Linux' and 'arm' in arch,
        }
        
        logger.info(f"Platform detected: {system} {arch} (machine: {machine})")
        return info
    
    def _get_default_library_path(self) -> Optional[str]:
        """
        根据平台获取默认library路径
        
        Returns:
            默认library路径，如果平台不支持返回None
        """
        cur_path = os.path.dirname(os.path.abspath(__file__))
        
        if self.platform_info['is_windows']:
            # Windows: 使用现有的DLL
            return os.path.join(cur_path, "uwbdemo", "trilateration.dll")
        
        elif self.platform_info['is_linux']:
            # Linux: 优先使用架构特定的.so，回退到通用.so
            arch = self.platform_info['arch']
            lib_dir = os.path.join(cur_path, "lib")
            
            # 尝试架构特定版本
            arch_specific = os.path.join(lib_dir, f"libtrilateration_{arch}.so")
            if os.path.exists(arch_specific):
                return arch_specific
            
            # 回退到通用版本（可能是符号链接）
            generic = os.path.join(lib_dir, "libtrilateration.so")
            if os.path.exists(generic):
                return generic
            
            logger.warning(f"No .so found in {lib_dir} for arch {arch}")
            return None
        
        else:
            logger.warning(f"Platform {self.platform_info['system']} not explicitly supported")
            return None
    
    def load_dll(self, library_path: str = None) -> bool:
        """
        加载trilateration native library（跨平台）
        
        Args:
            library_path: Library文件路径（可选）
            
        Returns:
            加载是否成功
        """
        try:
            path = library_path or self.library_path or self._get_default_library_path()
            
            if path is None:
                logger.error("无法确定native library路径")
                return False
            
            if not os.path.exists(path):
                logger.error(f"Native library不存在: {path}")
                logger.info(f"当前平台: {self.platform_info['system']} {self.platform_info['arch']}")
                if self.platform_info['is_linux']:
                    logger.info("提示: 运行 ./scripts/build_trilateration.sh 编译library")
                return False
            
            # 加载library
            self.library = ctypes.cdll.LoadLibrary(path)
            self.dll = self.library  # 保持向后兼容
            
            logger.info(f"成功加载native library: {path}")
            logger.info(f"平台: {self.platform_info['system']} {self.platform_info['arch']}")
            return True
            
        except OSError as e:
            logger.error(f"加载native library失败: {e}")
            logger.error(f"路径: {path}")
            if self.platform_info['is_linux'] and 'arm' in self.platform_info['arch']:
                logger.error("RPi提示: 确保使用 'arm64' 参数编译了library")
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
        if self.library is None and self.dll is None:
            logger.error("Native library未加载")
            return None
        
        lib = self.library or self.dll  # 向后兼容
        
        try:
            # 调用GetLocation函数
            result = lib.GetLocation(
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


def create_trilateration_wrapper(library_path: str = None) -> TrilaterationWrapper:
    """
    创建三边定位包装器实例
    
    Args:
        library_path: Native library路径（可选，自动检测）
        
    Returns:
        TrilaterationWrapper实例
    """
    wrapper = TrilaterationWrapper(library_path)
    if not wrapper.load_dll():
        logger.warning("无法加载native library，将使用Python备用算法")
        logger.info(f"平台: {wrapper.platform_info['system']} {wrapper.platform_info['arch']}")
    return wrapper
