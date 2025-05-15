from PIL import Image
import numpy as np

# 加载灰度图和可视化图
gray_path = "1647074029.44289.png"  # 改成你的灰度图路径
vis_path = "1647074029.442891.png"    # 改成你的可视化图路径

gray_img = Image.open(gray_path).convert("L")
vis_img = Image.open(vis_path).convert("RGB")

gray_array = np.array(gray_img)
vis_array = np.array(vis_img)

# 检查大小一致性
assert gray_array.shape == vis_array.shape[:2], "灰度图和可视化图大小不一致！"

# 类别标签（0~8）
class_names = {
    0: "Parking spot line",
    1: "Parking spot corner",
    2: "Ground surface",
    3: "Ground marker",
    4: "Lane line",
    5: "Construction",
    6: "Warning strip",
    7: "Speed bump",
    8: "Ego vehicle"
}

# 建立灰度值与RGB颜色的映射
mapping = {}
for y in range(gray_array.shape[0]):
    for x in range(gray_array.shape[1]):
        gray_val = gray_array[y, x]
        rgb = tuple(vis_array[y, x])
        if gray_val not in mapping:
            mapping[gray_val] = rgb
        elif mapping[gray_val] != rgb:
            print(f"⚠️ 警告：灰度值 {gray_val} 对应多个颜色，例如 {mapping[gray_val]} 和 {rgb}")

# 输出结果
print("\n灰度值 -> RGB颜色 -> 类别名 映射如下：")
for gray_val, rgb in sorted(mapping.items()):
    class_name = class_names.get(gray_val, "未知类别")
    print(f"灰度值 {gray_val:2}: RGB {rgb} -> 类别名：{class_name}")
