import pyrealsense2 as rs
import numpy as np
import cv2

# 配置 RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# 配置流：RGB 640x480 @ 30fps
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# 可选：深度流
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# 启动 pipeline
print("Starting RealSense...")
pipeline.start(config)

try:
    while True:
        # 等待新的帧
        frames = pipeline.wait_for_frames()
        
        # 获取 RGB 图像
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        
        # 转换为 numpy 数组
        color_image = np.asanyarray(color_frame.get_data())
        
        # 显示
        cv2.imshow('RealSense RGB', color_image)
        
        # 按 'q' 退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("Stopped")
