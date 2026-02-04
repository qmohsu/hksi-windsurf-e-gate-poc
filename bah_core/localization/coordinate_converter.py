"""
坐标转换模块
将GNSS坐标（ENU或LLA）转换为以A0为原点的平面相对坐标
"""

import math
import logging
from typing import Dict, Tuple, Optional
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class GNSSCoordinate:
    """GNSS坐标数据结构"""
    lat: float      # 纬度（度）
    lon: float      # 经度（度）
    alt: float      # 海拔高度（米）
    e: float = 0.0  # ENU坐标 - 东向（米）
    n: float = 0.0  # ENU坐标 - 北向（米）
    u: float = 0.0  # ENU坐标 - 天向（米）


@dataclass
class PlaneCoordinate:
    """平面坐标数据结构（以A0为原点）"""
    x: float  # X坐标（米）
    y: float  # Y坐标（米）
    z: float  # Z坐标（米）


class CoordinateConverter:
    """坐标转换器类"""
    
    # WGS84椭球参数
    WGS84_A = 6378137.0           # 长半轴（米）
    WGS84_F = 1.0 / 298.257223563  # 扁率
    WGS84_B = WGS84_A * (1 - WGS84_F)  # 短半轴
    WGS84_E2 = 2 * WGS84_F - WGS84_F ** 2  # 第一偏心率平方
    
    def __init__(self):
        """初始化坐标转换器"""
        self.origin: Optional[GNSSCoordinate] = None
        self.anchor_gnss: Dict[str, GNSSCoordinate] = {}
        self.anchor_plane: Dict[str, PlaneCoordinate] = {}
        
    def set_origin(self, gnss_coord: GNSSCoordinate):
        """
        设置原点坐标（A0的GNSS坐标）
        
        Args:
            gnss_coord: A0的GNSS坐标
        """
        self.origin = gnss_coord
        logger.info(f"原点已设置: lat={gnss_coord.lat:.6f}, lon={gnss_coord.lon:.6f}, alt={gnss_coord.alt:.2f}")
    
    def gnss_to_enu(self, gnss_coord: GNSSCoordinate) -> Tuple[float, float, float]:
        """
        将GNSS坐标转换为相对于原点的ENU坐标
        
        Args:
            gnss_coord: 目标点的GNSS坐标
            
        Returns:
            (e, n, u) ENU坐标元组（米）
        """
        if self.origin is None:
            raise ValueError("未设置原点坐标")
        
        # 转换为弧度
        lat0 = math.radians(self.origin.lat)
        lon0 = math.radians(self.origin.lon)
        lat = math.radians(gnss_coord.lat)
        lon = math.radians(gnss_coord.lon)
        
        # 计算纬度和经度差
        dlat = lat - lat0
        dlon = lon - lon0
        dalt = gnss_coord.alt - self.origin.alt
        
        # 计算曲率半径
        sin_lat0 = math.sin(lat0)
        N0 = self.WGS84_A / math.sqrt(1 - self.WGS84_E2 * sin_lat0 ** 2)
        M0 = self.WGS84_A * (1 - self.WGS84_E2) / ((1 - self.WGS84_E2 * sin_lat0 ** 2) ** 1.5)
        
        # 计算ENU坐标（线性近似，适用于小范围）
        e = (N0 + self.origin.alt) * math.cos(lat0) * dlon
        n = (M0 + self.origin.alt) * dlat
        u = dalt
        
        return (e, n, u)
    
    def enu_to_plane(self, e: float, n: float, u: float) -> PlaneCoordinate:
        """
        将ENU坐标转换为平面坐标
        以A0为原点，东向为X轴正方向，北向为Y轴正方向
        
        Args:
            e: 东向坐标（米）
            n: 北向坐标（米）
            u: 天向坐标（米）
            
        Returns:
            PlaneCoordinate对象
        """
        return PlaneCoordinate(x=e, y=n, z=u)
    
    def update_anchor_gnss(self, anchor_id: str, gnss_data: Dict):
        """
        更新Anchor的GNSS坐标
        
        Args:
            anchor_id: Anchor ID（如"A0", "A1", "A2"）
            gnss_data: GNSS坐标数据字典
        """
        coord = GNSSCoordinate(
            lat=gnss_data.get("lat", 0.0),
            lon=gnss_data.get("lon", 0.0),
            alt=gnss_data.get("alt", 0.0),
            e=gnss_data.get("e", 0.0),
            n=gnss_data.get("n", 0.0),
            u=gnss_data.get("u", 0.0)
        )
        self.anchor_gnss[anchor_id] = coord
        
        # 如果是A0，设置为原点
        if anchor_id == "A0":
            self.set_origin(coord)
        
        # 重新计算所有anchor的平面坐标
        self._update_all_plane_coordinates()
    
    def update_all_anchors(self, anchors_data: Dict):
        """
        更新所有Anchor的GNSS坐标
        
        Args:
            anchors_data: 所有Anchor的GNSS坐标数据
        """
        for anchor_id, gnss_data in anchors_data.items():
            coord = GNSSCoordinate(
                lat=gnss_data.get("lat", 0.0),
                lon=gnss_data.get("lon", 0.0),
                alt=gnss_data.get("alt", 0.0),
                e=gnss_data.get("e", 0.0),
                n=gnss_data.get("n", 0.0),
                u=gnss_data.get("u", 0.0)
            )
            self.anchor_gnss[anchor_id] = coord
        
        # 设置A0为原点
        if "A0" in self.anchor_gnss:
            self.set_origin(self.anchor_gnss["A0"])
        
        # 重新计算所有anchor的平面坐标
        self._update_all_plane_coordinates()
    
    def _update_all_plane_coordinates(self):
        """更新所有Anchor的平面坐标"""
        if self.origin is None:
            logger.warning("未设置原点，无法计算平面坐标")
            return
        
        for anchor_id, gnss_coord in self.anchor_gnss.items():
            if anchor_id == "A0":
                # A0为原点
                self.anchor_plane[anchor_id] = PlaneCoordinate(
                    x=0.0, 
                    y=0.0, 
                    z=gnss_coord.u
                )
            else:
                # 如果已有ENU坐标，直接使用
                if gnss_coord.e != 0.0 or gnss_coord.n != 0.0:
                    # 使用相对于A0的ENU坐标
                    rel_e = gnss_coord.e - self.origin.e
                    rel_n = gnss_coord.n - self.origin.n
                    self.anchor_plane[anchor_id] = PlaneCoordinate(
                        x=rel_e,
                        y=rel_n,
                        z=gnss_coord.u
                    )
                else:
                    # 从LLA坐标计算ENU
                    e, n, u = self.gnss_to_enu(gnss_coord)
                    self.anchor_plane[anchor_id] = self.enu_to_plane(e, n, u)
        
        # 打印Anchor平面坐标
        logger.info("Anchor平面坐标（以A0为原点）:")
        for anchor_id, coord in self.anchor_plane.items():
            logger.info(f"  {anchor_id}: ({coord.x:.3f}, {coord.y:.3f}, {coord.z:.3f}) m")
    
    def get_anchor_plane_coordinates(self) -> Dict[str, PlaneCoordinate]:
        """
        获取所有Anchor的平面坐标
        
        Returns:
            Anchor ID到平面坐标的字典
        """
        return self.anchor_plane.copy()
    
    def plane_to_gnss(self, plane_coord: PlaneCoordinate) -> GNSSCoordinate:
        """
        将平面坐标转换回GNSS坐标
        
        Args:
            plane_coord: 平面坐标
            
        Returns:
            GNSSCoordinate对象
        """
        if self.origin is None:
            raise ValueError("未设置原点坐标")
        
        lat0 = math.radians(self.origin.lat)
        lon0 = math.radians(self.origin.lon)
        
        # 计算曲率半径
        sin_lat0 = math.sin(lat0)
        N0 = self.WGS84_A / math.sqrt(1 - self.WGS84_E2 * sin_lat0 ** 2)
        M0 = self.WGS84_A * (1 - self.WGS84_E2) / ((1 - self.WGS84_E2 * sin_lat0 ** 2) ** 1.5)
        
        # 从ENU坐标反算经纬度
        dlat = plane_coord.y / (M0 + self.origin.alt)
        dlon = plane_coord.x / ((N0 + self.origin.alt) * math.cos(lat0))
        
        lat = self.origin.lat + math.degrees(dlat)
        lon = self.origin.lon + math.degrees(dlon)
        alt = self.origin.alt + plane_coord.z
        
        return GNSSCoordinate(
            lat=lat,
            lon=lon,
            alt=alt,
            e=plane_coord.x,
            n=plane_coord.y,
            u=plane_coord.z
        )


def create_virtual_anchor_coordinates(config: Dict) -> Dict[str, GNSSCoordinate]:
    """
    根据配置创建虚拟Anchor坐标
    
    Args:
        config: 虚拟GNSS配置
        
    Returns:
        Anchor坐标字典
    """
    base_lat = config["base_lat"]
    base_lon = config["base_lon"]
    base_alt = config["base_alt"]
    
    # 每米对应的经纬度变化
    lat_per_meter = 1.0 / 111000.0
    lon_per_meter = 1.0 / (111000.0 * math.cos(math.radians(base_lat)))
    
    anchors = {}
    for anchor_id, enu in config["anchors"].items():
        e = enu["e"]
        n = enu["n"]
        u = enu["u"]
        
        anchors[anchor_id] = GNSSCoordinate(
            lat=base_lat + n * lat_per_meter,
            lon=base_lon + e * lon_per_meter,
            alt=base_alt + u - base_alt,  # 相对高度
            e=e,
            n=n,
            u=u
        )
    
    return anchors
