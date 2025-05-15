import cv2
import numpy as np

# 读取两张图
image1 = cv2.imread('1743777701.234282.png')  # 第一张图（显示）
image2 = cv2.imread('1743777701.23428.png')  # 第二张图（不显示）

# 检查图像尺寸是否一致
if image1.shape != image2.shape:
    raise ValueError("两张图片尺寸不一致，请确保大小相同。")

zoom_factor = 10
zoom_size = 20

# 鼠标事件
def mouse_event(event, x, y, flags, param):
    h, w = image1.shape[:2]
    half = zoom_size // 2

    # 裁剪 zoom 区域
    x1 = max(0, x - half)
    y1 = max(0, y - half)
    x2 = min(w, x + half)
    y2 = min(h, y + half)

    zoom_crop = image1[y1:y2, x1:x2]
    zoom_display = cv2.resize(zoom_crop, (zoom_crop.shape[1]*zoom_factor, zoom_crop.shape[0]*zoom_factor), interpolation=cv2.INTER_NEAREST)

    # 十字指示
    center_x = (x2 - x1) * zoom_factor // 2
    center_y = (y2 - y1) * zoom_factor // 2
    cv2.drawMarker(zoom_display, (center_x, center_y), color=(0, 255, 0), markerType=cv2.MARKER_CROSS, thickness=1)

    # 显示放大窗口
    cv2.imshow('Zoom', zoom_display)

    # 点击时输出两张图的像素值
    if event == cv2.EVENT_LBUTTONDOWN:
        b1, g1, r1 = image1[y, x]
        b2, g2, r2 = image2[y, x]
        print(f"点击位置：({x}, {y})")
        print(f"  图1 - RGB: ({r1}, {g1}, {b1})")
        print(f"  图2 - RGB: ({r2}, {g2}, {b2})")

# 绑定事件
cv2.namedWindow('Image')
cv2.setMouseCallback('Image', mouse_event)

# 主循环
while True:
    cv2.imshow('Image', image1)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cv2.destroyAllWindows()
