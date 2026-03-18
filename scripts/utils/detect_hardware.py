#!/usr/bin/env python3
"""
硬件检测脚本 - 检测所有SO-ARM和相机设备
Hardware Detection Script - Detect all SO-ARM and camera devices
"""

import os
import sys
import glob
import subprocess

def print_header(title):
    """打印标题"""
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)

def check_usb_serial_devices():
    """检测USB串口设备（机械臂）"""
    print_header("📡 USB串口设备检测 / USB Serial Devices")

    # 查找串口设备
    devices = []
    for pattern in ['/dev/ttyUSB*', '/dev/ttyACM*', '/dev/tty.usbmodem*']:
        devices.extend(glob.glob(pattern))

    if not devices:
        print("❌ 未找到USB串口设备")
        print("提示: 确保机械臂已连接并上电")
        return []

    print(f"✅ 找到 {len(devices)} 个串口设备:")
    for i, dev in enumerate(devices, 1):
        # 检查权限
        readable = os.access(dev, os.R_OK)
        writable = os.access(dev, os.W_OK)
        status = "✓ 可读写" if (readable and writable) else "✗ 权限不足"

        print(f"  {i}. {dev} - {status}")

        # 尝试获取设备信息
        try:
            result = subprocess.run(
                ['udevadm', 'info', '-q', 'property', '-n', dev],
                capture_output=True, text=True, timeout=2
            )
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'ID_MODEL=' in line or 'ID_VENDOR=' in line:
                        print(f"     {line.strip()}")
        except:
            pass

    if not all(os.access(dev, os.R_OK | os.W_OK) for dev in devices):
        print("\n⚠️  权限不足，请运行:")
        print("   sudo usermod -a -G dialout $USER")
        print("   sudo chmod 666 /dev/ttyUSB* /dev/ttyACM*")
        print("   然后重新登录")

    return devices

def check_video_devices():
    """检测视频设备（相机）"""
    print_header("📷 视频设备检测 / Video Devices")

    try:
        import cv2
    except ImportError:
        print("❌ OpenCV未安装")
        print("   pip install opencv-python")
        return []

    devices = []
    print("扫描相机设备 (0-9)...")

    for i in range(10):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                h, w = frame.shape[:2]
                # 尝试获取设备名称
                backend_name = cap.getBackendName()
                devices.append({
                    'index': i,
                    'resolution': (w, h),
                    'backend': backend_name
                })
                print(f"  ✓ Camera {i}: {w}x{h} ({backend_name})")
            cap.release()

    if not devices:
        print("❌ 未找到可用的相机设备")
        print("提示: 确保相机已连接")
    else:
        print(f"\n✅ 找到 {len(devices)} 个相机")

    return devices

def check_realsense():
    """检测RealSense设备"""
    print_header("🔍 RealSense深度相机检测 / RealSense Depth Camera")

    try:
        import pyrealsense2 as rs
    except ImportError:
        print("❌ pyrealsense2未安装")
        print("   pip install pyrealsense2")
        return False

    try:
        ctx = rs.context()
        devices = ctx.query_devices()

        if len(devices) == 0:
            print("❌ 未找到RealSense设备")
            return False

        print(f"✅ 找到 {len(devices)} 个RealSense设备:")
        for i, dev in enumerate(devices):
            print(f"  {i+1}. {dev.get_info(rs.camera_info.name)}")
            print(f"     Serial: {dev.get_info(rs.camera_info.serial_number)}")
            print(f"     Firmware: {dev.get_info(rs.camera_info.firmware_version)}")

        return True
    except Exception as e:
        print(f"❌ RealSense检测失败: {e}")
        return False

def check_lerobot():
    """检测LeRobot环境"""
    print_header("🤖 LeRobot环境检测 / LeRobot Environment")

    try:
        from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
        from lerobot.motors.feetech import FeetechMotorsBus
        print("✅ LeRobot导入成功")
        print(f"   SO101Follower: {SO101Follower.__module__}")
        return True
    except ImportError as e:
        print(f"❌ LeRobot导入失败: {e}")
        print("\n请确保LeRobot已安装:")
        print("   cd ~/lerobot")
        print("   pip install -e .")
        return False

def check_network():
    """检测网络配置"""
    print_header("🌐 网络配置 / Network Configuration")

    try:
        # 获取本机IP
        result = subprocess.run(
            ['hostname', '-I'],
            capture_output=True, text=True, timeout=2
        )
        if result.returncode == 0:
            ips = result.stdout.strip().split()
            if ips:
                print("✅ 本机IP地址:")
                for ip in ips:
                    if not ip.startswith('127.'):
                        print(f"   {ip}")
                        print(f"   访问地址: http://{ip}:8000")
        else:
            print("⚠️  无法获取IP地址")
    except:
        print("⚠️  网络检测失败")

def generate_config(serial_devices, video_devices, has_realsense):
    """生成配置建议"""
    print_header("📝 配置建议 / Configuration Recommendations")

    if len(serial_devices) >= 2:
        print("\n✅ 检测到双臂配置:")
        print(f"   左臂: {serial_devices[0]}")
        print(f"   右臂: {serial_devices[1]}")
        print("\n启动命令:")
        print(f"   # 左臂")
        print(f"   python run_server.py --backend soarm --soarm-port {serial_devices[0]} --port 8001")
        print(f"   # 右臂")
        print(f"   python run_server.py --backend soarm --soarm-port {serial_devices[1]} --port 8002")
    elif len(serial_devices) == 1:
        print("\n✅ 检测到单臂配置:")
        print(f"   机械臂: {serial_devices[0]}")
        print("\n启动命令:")
        print(f"   python run_server.py --backend soarm --soarm-port {serial_devices[0]}")
        print(f"   # 或使用快速脚本:")
        print(f"   ./start_soarm_local.sh {serial_devices[0]}")
    else:
        print("\n❌ 未检测到机械臂")
        print("   请确保机械臂已连接并上电")

    if len(video_devices) >= 2:
        print(f"\n✅ 检测到 {len(video_devices)} 个相机")
        print("   建议配置:")
        print(f"   - 左手相机: /dev/video{video_devices[0]['index']}")
        print(f"   - 右手相机: /dev/video{video_devices[1]['index']}")
        if has_realsense:
            print(f"   - 深度相机: RealSense D435")
        elif len(video_devices) >= 3:
            print(f"   - 第三相机: /dev/video{video_devices[2]['index']} (可作为鸟瞰)")

    if has_realsense:
        print("\n✅ RealSense D435可用")
        print("   将用于深度感知")

def main():
    """主函数"""
    print("\n")
    print("╔════════════════════════════════════════════════════════╗")
    print("║     SO-ARM101 硬件检测工具 / Hardware Detection Tool  ║")
    print("╚════════════════════════════════════════════════════════╝")

    # 1. 检测USB串口设备
    serial_devices = check_usb_serial_devices()

    # 2. 检测视频设备
    video_devices = check_video_devices()

    # 3. 检测RealSense
    has_realsense = check_realsense()

    # 4. 检测LeRobot
    has_lerobot = check_lerobot()

    # 5. 检测网络
    check_network()

    # 6. 生成配置建议
    generate_config(serial_devices, video_devices, has_realsense)

    # 总结
    print_header("📊 检测总结 / Detection Summary")
    print(f"  串口设备: {len(serial_devices)} 个")
    print(f"  相机设备: {len(video_devices)} 个")
    print(f"  RealSense: {'✓' if has_realsense else '✗'}")
    print(f"  LeRobot: {'✓' if has_lerobot else '✗'}")

    # 判断是否可以启动
    print()
    if serial_devices and has_lerobot:
        print("✅ 系统就绪！可以开始测试")
        print("\n下一步:")
        print("  1. 单臂测试: python test_soarm_integration.py")
        print("  2. 启动服务器: ./start_soarm_local.sh /dev/ttyUSB0")
        print("  3. 查看完整指南: cat 本地验证完整指南.md")
    else:
        print("⚠️  系统未就绪，请解决以上问题后重试")

    print("\n" + "=" * 60 + "\n")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n检测已取消")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ 检测过程中出错: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
