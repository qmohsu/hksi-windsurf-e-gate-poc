"""
系统测试脚本
在本机同时运行服务器和模拟客户端进行测试
"""

import subprocess
import sys
import time
import os

def main():
    print("=" * 60)
    print("          UWB定位系统测试")
    print("=" * 60)
    print()
    print("本测试将在本机同时运行：")
    print("  1. UWB定位服务器 (监听端口 8765)")
    print("  2. 模拟树莓派客户端 (发送虚拟测距数据)")
    print()
    print("请先在一个终端运行服务器，再在另一个终端运行客户端")
    print()
    print("启动服务器命令：")
    print("  cd C:\\Users\\cunyi\\Desktop\\cursor_code\\UWB_RPi_HKSI")
    print("  python main.py --debug")
    print()
    print("启动客户端命令：")
    print("  cd C:\\Users\\cunyi\\Desktop\\cursor_code\\RPi_UWB_Forwarder")  
    print("  python main.py --mode virtual --debug")
    print()
    print("=" * 60)

if __name__ == "__main__":
    main()
