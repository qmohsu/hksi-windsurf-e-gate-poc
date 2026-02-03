"""
UWB定位服务器主程序
接收树莓派发送的测距数据，计算Tag位置，并发送定位结果
"""

import sys
import time
import signal
import logging
import argparse
from typing import Optional, Dict, List, Tuple

import config
from network_server import UWBDataServer
from coordinate_converter import CoordinateConverter, create_virtual_anchor_coordinates
from trilateration_wrapper import TrilaterationWrapper, PythonTrilateration, Vec3D
from position_protocol import (
    Position, PositionBatch, PositionSender, SourceMask,
    create_position_from_local, print_position_standard
)

# 配置日志
logging.basicConfig(
    level=getattr(logging, config.LOGGING_CONFIG["level"]),
    format=config.LOGGING_CONFIG["format"]
)
logger = logging.getLogger(__name__)


class UWBLocalizationServer:
    """UWB定位服务器主类"""
    
    def __init__(self):
        """初始化UWB定位服务器"""
        self.running = False
        
        # 初始化数据服务器
        self.data_server = UWBDataServer(
            host=config.SERVER_CONFIG["host"],
            port=config.SERVER_CONFIG["port"]
        )
        
        # 初始化坐标转换器
        self.coord_converter = CoordinateConverter()
        
        # 初始化三边定位模块
        self.trilateration = TrilaterationWrapper()
        self.use_python_fallback = False
        
        # 初始化位置发送器（发送到平板终端）
        self.position_sender = PositionSender(
            host=config.TABLET_CONFIG["host"],
            port=config.TABLET_CONFIG["port"],
            protocol=config.TABLET_CONFIG["protocol"]
        )
        
        # 统计信息
        self.frame_count = 0
        self.success_count = 0
        self.last_position: Optional[Position] = None
        self._gnss_initialized = False
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        logger.info("UWB定位服务器初始化完成")
    
    def _signal_handler(self, signum, frame):
        """信号处理函数"""
        logger.info(f"收到信号 {signum}，正在停止...")
        self.stop()
    
    def _setup_trilateration(self) -> bool:
        """设置三边定位模块"""
        if not self.trilateration.load_dll():
            logger.warning("无法加载trilateration.dll，将使用Python备用算法")
            self.use_python_fallback = True
        return True
    
    def _setup_virtual_gnss(self):
        """设置虚拟GNSS坐标（用于测试）"""
        logger.info("设置虚拟GNSS坐标...")
        
        virtual_anchors = create_virtual_anchor_coordinates(config.VIRTUAL_GNSS_CONFIG)
        
        # 转换为字典格式
        gnss_dict = {}
        for anchor_id, coord in virtual_anchors.items():
            gnss_dict[anchor_id] = {
                "lat": coord.lat,
                "lon": coord.lon,
                "alt": coord.alt,
                "e": coord.e,
                "n": coord.n,
                "u": coord.u
            }
        
        # 更新坐标转换器
        self.coord_converter.update_all_anchors(gnss_dict)
        
        # 设置trilateration anchor位置
        anchor_positions = []
        for anchor_id in ["A0", "A1", "A2"]:
            if anchor_id in self.coord_converter.anchor_plane:
                coord = self.coord_converter.anchor_plane[anchor_id]
                anchor_positions.append((coord.x, coord.y, coord.z))
        
        if len(anchor_positions) >= 3:
            self.trilateration.set_anchor_positions(anchor_positions)
    
    def _process_range_data(self, data: Dict) -> Optional[Position]:
        """
        处理测距数据并计算位置
        
        Args:
            data: 从树莓派接收的测距数据
            
        Returns:
            计算得到的Position，失败返回None
        """
        try:
            uwb_data = data.get("data", {})
            ranges = uwb_data.get("ranges", [])
            
            # 检查是否有GNSS更新（只在首次或坐标有变化时更新）
            gnss_data = data.get("gnss")
            if gnss_data and not self._gnss_initialized:
                self.coord_converter.update_all_anchors(gnss_data)
                self._update_anchor_positions()
                self._gnss_initialized = True
            
            # 过滤有效测距值
            valid_ranges = []
            for i, r in enumerate(ranges[:3]):  # 只使用前3个anchor
                if (config.LOCALIZATION_CONFIG["min_range_mm"] <= r <= 
                    config.LOCALIZATION_CONFIG["max_range_mm"]):
                    valid_ranges.append(r)
                else:
                    valid_ranges.append(-1)  # 无效值
            
            # 检查有效anchor数量
            valid_count = sum(1 for r in valid_ranges if r > 0)
            if valid_count < config.LOCALIZATION_CONFIG["min_anchors"]:
                logger.debug(f"有效anchor数量不足: {valid_count}")
                return None
            
            logger.debug(f"测距值: {valid_ranges[:3]} mm")
            
            # 补齐到8个
            while len(valid_ranges) < 8:
                valid_ranges.append(-1)
            
            # 计算位置
            location = self._calculate_location(valid_ranges)
            
            if location is None:
                logger.debug("定位计算失败")
                return None
            
            logger.debug(f"计算得到位置: ({location.x:.3f}, {location.y:.3f}, {location.z:.3f})")
            
            # 创建Position对象
            position = create_position_from_local(
                device_id=int(uwb_data.get("tag_id", "0") or "0"),
                local_x=location.x,
                local_y=location.y,
                local_z=location.z,
                coordinate_converter=self.coord_converter,
                source_mask=SourceMask.UWB
            )
            
            return position
            
        except Exception as e:
            logger.error(f"处理测距数据异常: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def _update_anchor_positions(self):
        """更新trilateration模块的anchor位置"""
        anchor_positions = []
        for anchor_id in ["A0", "A1", "A2"]:
            if anchor_id in self.coord_converter.anchor_plane:
                coord = self.coord_converter.anchor_plane[anchor_id]
                anchor_positions.append((coord.x, coord.y, coord.z))
        
        if len(anchor_positions) >= 3:
            self.trilateration.set_anchor_positions(anchor_positions)
    
    def _calculate_location(self, distances_mm: List[int]) -> Optional[Vec3D]:
        """
        计算Tag位置
        
        Args:
            distances_mm: 测距值列表（毫米）
            
        Returns:
            Vec3D位置，失败返回None
        """
        if self.use_python_fallback:
            # 使用Python备用算法
            return self._calculate_location_python(distances_mm)
        else:
            # 使用DLL
            self.trilateration.set_distances(distances_mm)
            return self.trilateration.calculate_location()
    
    def _calculate_location_python(self, distances_mm: List[int]) -> Optional[Vec3D]:
        """使用Python备用算法计算位置"""
        # 获取anchor坐标
        anchors = []
        distances = []
        
        for i, anchor_id in enumerate(["A0", "A1", "A2"]):
            if anchor_id in self.coord_converter.anchor_plane:
                coord = self.coord_converter.anchor_plane[anchor_id]
                anchors.append((coord.x, coord.y, coord.z))
                
                if i < len(distances_mm) and distances_mm[i] > 0:
                    distances.append(distances_mm[i] / 1000.0)  # 转换为米
                else:
                    distances.append(-1)
        
        # 过滤有效数据
        valid_anchors = []
        valid_distances = []
        for a, d in zip(anchors, distances):
            if d > 0:
                valid_anchors.append(a)
                valid_distances.append(d)
        
        if len(valid_anchors) < 3:
            return None
        
        # 计算位置
        result = PythonTrilateration.trilaterate_3d(valid_anchors, valid_distances)
        
        if result:
            return Vec3D(x=result[0], y=result[1], z=result[2])
        return None
    
    def start(self):
        """启动定位服务器"""
        logger.info("UWB定位服务器启动中...")
        
        # 设置三边定位模块
        self._setup_trilateration()
        
        # 设置虚拟GNSS（如果没有接收到真实GNSS数据）
        self._setup_virtual_gnss()
        
        # 启动数据服务器
        if not self.data_server.start():
            logger.error("启动数据服务器失败")
            return
        
        # 尝试连接平板终端
        if config.OUTPUT_CONFIG["enable_tablet_send"]:
            if not self.position_sender.connect():
                logger.warning("无法连接平板终端，将只打印到控制台")
        
        self.running = True
        logger.info("定位服务器已启动，等待数据...")
        
        # 主循环
        try:
            self._run_loop()
        finally:
            self.stop()
    
    def _run_loop(self):
        """主运行循环"""
        print_interval = config.OUTPUT_CONFIG["print_interval"]
        last_status_time = time.time()
        
        while self.running:
            try:
                # 每5秒打印一次状态
                if time.time() - last_status_time > 5:
                    print(f"[服务器] 状态: 已接收 {self.frame_count} 帧, 成功定位 {self.success_count} 帧")
                    last_status_time = time.time()
                
                # 获取数据（超时1秒）
                data = self.data_server.get_data(timeout=1.0)
                
                if data is None:
                    continue
                
                self.frame_count += 1
                print(f"[服务器] 处理第 {self.frame_count} 帧数据...")
                
                # 处理数据并计算位置
                position = self._process_range_data(data)
                
                if position:
                    self.success_count += 1
                    self.last_position = position
                    print(f"[服务器] 定位成功! 位置: ({position.local_x:.2f}, {position.local_y:.2f}, {position.local_z:.2f}) m")
                    
                    # 打印定位结果
                    if config.OUTPUT_CONFIG["enable_console_print"]:
                        if self.frame_count % print_interval == 0:
                            print_position_standard(position)
                            self._print_statistics()
                    
                    # 发送到平板终端
                    if config.OUTPUT_CONFIG["enable_tablet_send"]:
                        self.position_sender.send_position(position)
                else:
                    print(f"[服务器] 定位失败")
                
            except Exception as e:
                logger.error(f"主循环异常: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
    
    def _print_statistics(self):
        """打印统计信息"""
        success_rate = (self.success_count / self.frame_count * 100) if self.frame_count > 0 else 0
        print(f"统计: 接收帧数={self.frame_count}, 成功定位={self.success_count}, "
              f"成功率={success_rate:.1f}%")
        print()
    
    def stop(self):
        """停止定位服务器"""
        logger.info("正在停止UWB定位服务器...")
        self.running = False
        
        # 停止各模块
        self.data_server.stop()
        self.position_sender.disconnect()
        
        # 打印最终统计
        print("\n" + "=" * 60)
        print("               服务器已停止")
        print("=" * 60)
        if self.frame_count > 0:
            success_rate = self.success_count / self.frame_count * 100
            print(f"总接收帧数: {self.frame_count}")
            print(f"成功定位数: {self.success_count}")
            print(f"成功率: {success_rate:.1f}%")
        print("=" * 60)
        
        logger.info("UWB定位服务器已停止")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='UWB定位服务器')
    parser.add_argument('--host', '-H', type=str, default=None,
                       help='监听地址')
    parser.add_argument('--port', '-p', type=int, default=None,
                       help='监听端口')
    parser.add_argument('--debug', '-d', action='store_true',
                       help='启用调试日志')
    
    args = parser.parse_args()
    
    # 设置日志级别
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # 更新配置
    if args.host:
        config.SERVER_CONFIG["host"] = args.host
    if args.port:
        config.SERVER_CONFIG["port"] = args.port
    
    # 创建并启动服务器
    server = UWBLocalizationServer()
    server.start()


if __name__ == "__main__":
    main()
