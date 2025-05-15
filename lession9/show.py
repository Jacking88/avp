import cv2
import numpy as np

# 读取图像
image = cv2.imread('1744043899.796197.png')
zoom_factor = 10        # 放大倍数
zoom_size = 20          # 原图区域大小 (zoom_size x zoom_size)

# 鼠标事件处理
def mouse_event(event, x, y, flags, param):
    h, w = image.shape[:2]
    half = zoom_size // 2

    # 获取区域坐标，确保不越界
    x1 = max(0, x - half)
    y1 = max(0, y - half)
    x2 = min(w, x + half)
    y2 = min(h, y + half)

    # 提取放大区域
    zoom_crop = image[y1:y2, x1:x2]
    zoom_display = cv2.resize(zoom_crop, (zoom_crop.shape[1]*zoom_factor, zoom_crop.shape[0]*zoom_factor), interpolation=cv2.INTER_NEAREST)

    # 画一个中心十字架用于标记鼠标中心
    center_x = (x2 - x1) * zoom_factor // 2
    center_y = (y2 - y1) * zoom_factor // 2
    cv2.drawMarker(zoom_display, (center_x, center_y), color=(0, 255, 0), markerType=cv2.MARKER_CROSS, thickness=1)

    # 显示放大窗口
    cv2.imshow('Zoom', zoom_display)

    # 左键点击时输出 RGB
    if event == cv2.EVENT_LBUTTONDOWN:
        b, g, r = image[y, x]
        print(f"点击位置：({x}, {y}) - BGR: ({b}, {g}, {r}) - RGB: ({r}, {g}, {b})")

# 创建窗口并绑定事件
cv2.namedWindow('Image')
cv2.setMouseCallback('Image', mouse_event)

# 主循环
while True:
    cv2.imshow('Image', image)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC键退出
        break

cv2.destroyAllWindows()
