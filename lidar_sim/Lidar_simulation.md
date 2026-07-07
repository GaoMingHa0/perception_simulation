## 雷达模拟器

judge:判断雷达是否在车前

if 四个判断条件 ok

对每个锥筒：
  → 是否在 LiDAR 120° 视场内
  → 是否在有效距离内（1.5m ~ 50m）
  → 是否有遮挡
  → 生成 12-16 个点模拟锥筒表面反射（加高斯噪声 σ=0.02m）

    return True / False

角度判断：
import numpy as np

  def is_in_fov(car_position, car_heading, cone_position, fov_deg=120):
      """
      判断锥桶是否在LiDAR的视场角内

      Parameters:
          car_position : array-like [x, y, z]  LiDAR/车辆位置
          car_heading  : float (rad)           车辆航向角（相对世界坐标系x轴）
          cone_position: array-like [x, y, z]  锥桶位置
          fov_deg      : float                水平视场角（默认120°）

      Returns:
          bool: True = 在视场内
      """
      # 1. 计算水平方向向量
      dx = cone_position[0] - car_position[0]
      dy = cone_position[1] - car_position[1]

      # 2. 计算锥桶方位角
      angle_to_cone = np.arctan2(dy, dx)

      # 3. 计算与航向的相对角度
      relative_angle = angle_to_cone - car_heading

      # 4. 归一化到 [-π, π]
      relative_angle = (relative_angle + np.pi) % (2 * np.pi) - np.pi

      # 5. 判断是否在视场半角内
      half_fov = np.radians(fov_deg / 2)
      return abs(relative_angle) <= half_fov

point_make_and_map:把生成的点云数据放入地图中

    生成 200-500 个随机地面点（LiDAR z=-1.0 附近）
  → 让 RANSAC 地面分割有输入可处理

main:主函数
    get_car_position()
    for i in range()
        if lidar_position is front of the car
            if(judge()) point_make_and_map()



