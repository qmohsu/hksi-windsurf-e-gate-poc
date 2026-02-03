"""
网络服务器模块
接收树莓派发送的UWB测距数据
"""

import socket
import json
import threading
import logging
import time
from typing import Optional, Dict, Callable, Any
from queue import Queue

logger = logging.getLogger(__name__)


class DataReceiver:
    """数据接收器，处理TCP连接和数据接收"""
    
    def __init__(self, host: str, port: int, data_callback: Callable[[Dict], None]):
        """
        初始化数据接收器
        
        Args:
            host: 监听地址
            port: 监听端口
            data_callback: 数据接收回调函数
        """
        self.host = host
        self.port = port
        self.data_callback = data_callback
        self.server_socket: Optional[socket.socket] = None
        self.running = False
        self.clients = []
        self.receive_thread: Optional[threading.Thread] = None
        
    def start(self) -> bool:
        """启动服务器"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            self.server_socket.settimeout(1.0)
            
            self.running = True
            self.receive_thread = threading.Thread(target=self._accept_loop, daemon=True)
            self.receive_thread.start()
            
            logger.info(f"服务器已启动，监听 {self.host}:{self.port}")
            return True
            
        except Exception as e:
            logger.error(f"启动服务器失败: {e}")
            return False
    
    def stop(self):
        """停止服务器"""
        self.running = False
        
        # 关闭所有客户端连接
        for client in self.clients:
            try:
                client.close()
            except:
                pass
        self.clients.clear()
        
        # 关闭服务器socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        logger.info("服务器已停止")
    
    def _accept_loop(self):
        """接受连接循环"""
        while self.running:
            try:
                client_socket, address = self.server_socket.accept()
                logger.info(f"新客户端连接: {address}")
                self.clients.append(client_socket)
                
                # 为每个客户端创建接收线程
                client_thread = threading.Thread(
                    target=self._handle_client,
                    args=(client_socket, address),
                    daemon=True
                )
                client_thread.start()
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    logger.error(f"接受连接异常: {e}")
    
    def _handle_client(self, client_socket: socket.socket, address):
        """处理客户端连接"""
        buffer = b''
        
        try:
            while self.running:
                try:
                    client_socket.settimeout(1.0)
                    data = client_socket.recv(4096)
                    
                    if not data:
                        logger.info(f"客户端断开连接: {address}")
                        break
                    
                    buffer += data
                    
                    # 处理接收到的数据（带长度前缀）
                    while len(buffer) >= 4:
                        # 读取长度前缀
                        msg_length = int.from_bytes(buffer[:4], byteorder='big')
                        
                        if len(buffer) < 4 + msg_length:
                            break  # 数据不完整，等待更多数据
                        
                        # 提取消息
                        message_data = buffer[4:4+msg_length]
                        buffer = buffer[4+msg_length:]
                        
                        # 解析JSON
                        try:
                            message = json.loads(message_data.decode('utf-8'))
                            self.data_callback(message)
                        except json.JSONDecodeError as e:
                            logger.warning(f"JSON解析失败: {e}")
                
                except socket.timeout:
                    continue
                    
        except Exception as e:
            logger.error(f"处理客户端数据异常: {e}")
        finally:
            try:
                client_socket.close()
                if client_socket in self.clients:
                    self.clients.remove(client_socket)
            except:
                pass


class UWBDataServer:
    """UWB数据服务器"""
    
    def __init__(self, host: str, port: int):
        """
        初始化UWB数据服务器
        
        Args:
            host: 监听地址
            port: 监听端口
        """
        self.data_queue = Queue()
        self.gnss_data: Dict = {}
        self.receiver = DataReceiver(host, port, self._on_data_received)
        
    def start(self) -> bool:
        """启动服务器"""
        return self.receiver.start()
    
    def stop(self):
        """停止服务器"""
        self.receiver.stop()
    
    def _on_data_received(self, message: Dict):
        """
        数据接收回调
        
        Args:
            message: 接收到的消息
        """
        msg_type = message.get("type", "")
        logger.debug(f"收到消息类型: {msg_type}")
        
        if msg_type == "uwb_range":
            # UWB测距数据
            self.data_queue.put(message)
            ranges = message.get("data", {}).get("ranges", [])
            print(f"[服务器] 收到UWB测距数据: {ranges[:3]}")
            
        elif msg_type == "gnss_update":
            # GNSS坐标更新
            self.gnss_data = message.get("anchors", {})
            print("[服务器] 收到GNSS坐标更新")
            logger.info("收到GNSS坐标更新")
            for anchor_id, coords in self.gnss_data.items():
                logger.info(f"  {anchor_id}: lat={coords.get('lat', 0):.6f}, "
                           f"lon={coords.get('lon', 0):.6f}")
    
    def get_data(self, timeout: float = None) -> Optional[Dict]:
        """
        获取接收到的UWB数据
        
        Args:
            timeout: 超时时间（秒）
            
        Returns:
            数据字典，超时返回None
        """
        try:
            return self.data_queue.get(timeout=timeout)
        except:
            return None
    
    def get_gnss_data(self) -> Dict:
        """获取最新的GNSS坐标数据"""
        return self.gnss_data.copy()
    
    def has_data(self) -> bool:
        """检查是否有待处理的数据"""
        return not self.data_queue.empty()
