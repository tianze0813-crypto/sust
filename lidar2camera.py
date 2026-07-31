#!/usr/bin/env python3
"""
LiDAR点云投影到鱼眼相机图像。

将LiDAR点云通过Kannala-Brandt鱼眼模型投影到4路环视相机上，
并用深度颜色可视化投影结果。支持多种标定模式以诊断对齐问题。

Usage:
    python lidar2camera.py --data_root <path> [--frame 0] [--rot_mode 0]
    python lidar2camera.py --data_root <path> --lidar_name lidar_top \
        --project_boxes --output_dir projected_boxes

LiDAR坐标系 (需与rot_mode匹配):
  旧数据集(step3):  PCD: x=前/y=左/z=上, 需要 rot_mode=1 (-90°)
  新数据集(back):   BIN: x=左/y=后/z=上, 需要 rot_mode=0 (+90°, calib原值)

标定模式 (按键m切换旋转, M切换相机交换):
    旋转模式 (rot_mode 0-3):
      0 = +90°z [默认] (适用于新数据集: lidar x=左/y=后/z=上)
      1 = -90°z  (lidar_x→base_-y, lidar_y→base_x)
      2 = 0°     (单位阵)
      3 = 180°z  (x,y翻转)
    相机交换 (swap_mode):
      0 = 无交换
      1 = 左右交换 (cam_left↔cam_right)
      2 = 前后交换 (cam_front↔cam_rear)
      3 = 全部交换 (前后+左右)

交互控制:
    n/p     - 上/下一帧
    m       - 切换旋转模式 (0=+90° 1=-90° 2=0° 3=180°)
    M       - 切换相机交换 (0=无 1=左右↔ 2=前后↔ 3=全部↔)
    r       - 切换采样率
    c       - 切换颜色模式
    +/-     - 调整点大小
    d       - 切换深度范围
    a       - 显示/隐藏坐标轴
    s       - 保存截图
    h       - 显示帮助
    q/ESC   - 退出
"""

import os
import sys
import argparse
import json
import math
import cv2
import numpy as np

# 确保项目根目录在 sys.path 中
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to an (N, 3) array."""
    if len(points) == 0:
        return points.reshape(0, 3)
    pts_h = np.concatenate(
        [points[:, :3], np.ones((len(points), 1), dtype=points.dtype)],
        axis=1,
    )
    return (transform @ pts_h.T).T[:, :3]


class CameraModel:
    """Camera projection helpers for calib.json camera models."""

    @staticmethod
    def project_kannala_brandt(points_cam: np.ndarray,
                               K: np.ndarray,
                               D: np.ndarray,
                               img_size: tuple) -> np.ndarray:
        if len(points_cam) == 0:
            return np.empty((0, 2), dtype=np.float64)

        x = points_cam[:, 0]
        y = points_cam[:, 1]
        z = points_cam[:, 2]
        r = np.sqrt(x * x + y * y)
        theta = np.arctan2(r, z)

        d = np.zeros(4, dtype=np.float64)
        d[:min(4, len(D))] = D[:min(4, len(D))]
        theta2 = theta * theta
        theta4 = theta2 * theta2
        theta6 = theta4 * theta2
        theta8 = theta4 * theta4
        theta_d = theta * (
            1.0
            + d[0] * theta2
            + d[1] * theta4
            + d[2] * theta6
            + d[3] * theta8
        )

        scale = np.divide(theta_d, r, out=np.ones_like(theta_d), where=r > 1e-9)
        xd = x * scale
        yd = y * scale

        u = K[0, 0] * xd + K[0, 1] * yd + K[0, 2]
        v = K[1, 0] * xd + K[1, 1] * yd + K[1, 2]
        return np.stack([u, v], axis=1)

    @staticmethod
    def project_pinhole_full(points_cam: np.ndarray,
                             K: np.ndarray,
                             D: np.ndarray,
                             img_size: tuple) -> np.ndarray:
        if len(points_cam) == 0:
            return np.empty((0, 2), dtype=np.float64)

        z = points_cam[:, 2]
        x = np.divide(
            points_cam[:, 0],
            z,
            out=np.full_like(z, np.nan, dtype=np.float64),
            where=np.abs(z) > 1e-9,
        )
        y = np.divide(
            points_cam[:, 1],
            z,
            out=np.full_like(z, np.nan, dtype=np.float64),
            where=np.abs(z) > 1e-9,
        )

        d = np.zeros(8, dtype=np.float64)
        d[:min(8, len(D))] = D[:min(8, len(D))]
        k1, k2, p1, p2, k3, k4, k5, k6 = d

        r2 = x * x + y * y
        r4 = r2 * r2
        r6 = r4 * r2
        radial_num = 1.0 + k1 * r2 + k2 * r4 + k3 * r6
        radial_den = 1.0 + k4 * r2 + k5 * r4 + k6 * r6
        radial = np.divide(
            radial_num,
            radial_den,
            out=np.full_like(radial_num, np.nan, dtype=np.float64),
            where=np.abs(radial_den) > 1e-9,
        )

        xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y

        u = K[0, 0] * xd + K[0, 1] * yd + K[0, 2]
        v = K[1, 0] * xd + K[1, 1] * yd + K[1, 2]
        return np.stack([u, v], axis=1)

    @staticmethod
    def depth_mask(points_cam: np.ndarray, model_type: str) -> np.ndarray:
        model = (model_type or '').upper()
        if model.startswith('PINHOLE'):
            return points_cam[:, 2] > 1e-6
        return points_cam[:, 2] > -0.5

    @staticmethod
    def project(points_cam: np.ndarray,
                K: np.ndarray,
                D: np.ndarray,
                img_size: tuple,
                model_type: str) -> np.ndarray:
        model = (model_type or '').upper()
        if model.startswith('PINHOLE'):
            return CameraModel.project_pinhole_full(points_cam, K, D, img_size)
        return CameraModel.project_kannala_brandt(points_cam, K, D, img_size)


# ============================================================
# 标定加载（支持修复）
# ============================================================

class CalibData:
    """
    标定数据容器。加载 calib.json 并支持多种坐标系解释。

    旋转模式 (rot_mode):
      0 = raw (+90°z):  calib.json 原始旋转矩阵
      1 = -90°z:        R = [[0,1,0],[-1,0,0],[0,0,1]]
      2 = 0° (identity): R = I
      3 = 180°z:        R = [[-1,0,0],[0,-1,0],[0,0,1]]

    相机交换 (swap_mode):
      0 = 无交换
      1 = 左右交换 (left↔right)
      2 = 前后交换 (front↔rear)
      3 = 全部交换 (left↔right + front↔rear)
    """

    # 预定义的 z-轴旋转矩阵
    ROTATIONS = {
        0: np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=np.float64),   # +90°
        1: np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=np.float64),   # -90°
        2: np.eye(3, dtype=np.float64),                                        # 0°
        3: np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]], dtype=np.float64),  # 180°
    }

    ROT_NAMES = {0: "+90°z", 1: "-90°z", 2: "0°(I)", 3: "180°z"}
    SWAP_NAMES = {0: "无", 1: "左右↔", 2: "前后↔", 3: "全部↔"}

    def __init__(self, calib_path: str, rot_mode: int = 1, swap_mode: int = 0,
                 lidar_extrinsic_mode: str = 'raw'):
        with open(calib_path, 'r') as f:
            self._data = json.load(f)

        self.rot_mode = rot_mode
        self.swap_mode = swap_mode
        self.lidar_extrinsic_mode = lidar_extrinsic_mode
        self._tf2base = self._data.get('tf2base_link', {})

        # 物理相机列表
        self._physical_cam_names = [
            k for k in self._data
            if k.startswith('cam_') and isinstance(self._data[k], dict) and 'K' in self._data[k]
        ]
        self.lidar_names = [k for k in self._tf2base if k.startswith('lidar_')]

        # 缓存原始矩阵
        self._T_sensor_to_base = {}
        for sensor, matrix in self._tf2base.items():
            self._T_sensor_to_base[sensor] = np.array(matrix, dtype=np.float64)
        self._T_sensor_to_base['base_link'] = np.eye(4, dtype=np.float64)

        # 相机名映射（根据 swap_mode）
        self._build_camera_map()

    def _build_camera_map(self):
        """根据 swap_mode 构建物理相机名 -> 逻辑相机名的映射。"""
        # 默认: 物理名 = 逻辑名
        self._cam_map = {n: n for n in self._physical_cam_names}

        if self.swap_mode == 1:  # 左右交换
            if 'cam_left' in self._cam_map and 'cam_right' in self._cam_map:
                self._cam_map['cam_left'] = 'cam_right'
                self._cam_map['cam_right'] = 'cam_left'
        elif self.swap_mode == 2:  # 前后交换
            if 'cam_front' in self._cam_map and 'cam_rear' in self._cam_map:
                self._cam_map['cam_front'] = 'cam_rear'
                self._cam_map['cam_rear'] = 'cam_front'
        elif self.swap_mode == 3:  # 全部交换
            if 'cam_left' in self._cam_map and 'cam_right' in self._cam_map:
                self._cam_map['cam_left'] = 'cam_right'
                self._cam_map['cam_right'] = 'cam_left'
            if 'cam_front' in self._cam_map and 'cam_rear' in self._cam_map:
                self._cam_map['cam_front'] = 'cam_rear'
                self._cam_map['cam_rear'] = 'cam_front'

    @property
    def cam_names(self) -> list:
        """返回用于显示的相机名称（物理名）。"""
        return self._physical_cam_names

    def get_physical_cam_name(self, logical_name: str) -> str:
        """逻辑相机名 -> 实际加载图像的物理相机名。"""
        return self._cam_map.get(logical_name, logical_name)

    def _get_T_lidar_to_base(self, lidar_name: str) -> np.ndarray:
        """获取（修复后的）LiDAR->base_link 变换矩阵。"""
        T_raw = self._T_sensor_to_base.get(lidar_name)
        if T_raw is None:
            raise ValueError(f"Unknown lidar: {lidar_name}")

        if self.lidar_extrinsic_mode == 'raw':
            return T_raw

        T_fixed = np.eye(4, dtype=np.float64)
        T_fixed[:3, :3] = self.ROTATIONS[self.rot_mode]
        T_fixed[:3, 3] = T_raw[:3, 3]  # 保留原始平移
        return T_fixed

    def get_transform(self, src: str, dst_logical: str) -> np.ndarray:
        """
        计算 src -> dst 的 4x4 变换矩阵。
        dst_logical 是逻辑相机名（会被映射到物理相机）。
        """
        if src == dst_logical:
            return np.eye(4, dtype=np.float64)

        # 将逻辑相机名映射到物理相机名
        dst_physical = self._cam_map.get(dst_logical, dst_logical)

        if src.startswith('lidar_'):
            T_src_to_base = self._get_T_lidar_to_base(src)
        else:
            T_src_to_base = self._T_sensor_to_base[src]

        T_dst_to_base = self._T_sensor_to_base[dst_physical]
        T_src_to_dst = np.linalg.inv(T_dst_to_base) @ T_src_to_base
        return T_src_to_dst

    def get_camera_intrinsics(self, cam_name: str) -> np.ndarray:
        return np.array(self._data[cam_name]['K'], dtype=np.float64)

    def get_camera_distortion(self, cam_name: str) -> np.ndarray:
        return np.array(self._data[cam_name]['D'], dtype=np.float64)

    def get_camera_resolution(self, cam_name: str) -> tuple:
        return (self._data[cam_name]['imgw'], self._data[cam_name]['imgh'])

    def get_camera_model(self, cam_name: str) -> str:
        return self._data[cam_name].get('model_type', 'UNKNOWN')

    def get_mode_name(self) -> str:
        return f"rot={self.ROT_NAMES[self.rot_mode]}, swap={self.SWAP_NAMES[self.swap_mode]}"

    def get_mode_description(self) -> str:
        r = self.rot_mode
        s = self.swap_mode
        rot_names_detail = {
            0: "+90°z: lidar_x→base_y, lidar_y→base_-x",
            1: "-90°z: lidar_x→base_-y, lidar_y→base_x",
            2: "0°(I): lidar_x→base_x, lidar_y→base_y",
            3: "180°z: lidar_x→base_-x, lidar_y→base_-y",
        }
        swap_names_detail = {
            0: "相机无交换",
            1: "cam_left↔cam_right",
            2: "cam_front↔cam_rear",
            3: "cam_left↔right + cam_front↔rear",
        }
        return f"{rot_names_detail[r]}, {swap_names_detail[s]}"


# ============================================================
# PCD 读取
# ============================================================

def read_lidar(file_path: str) -> np.ndarray:
    """读取LiDAR点云文件（支持 .pcd 和 .bin 格式），返回 (N, 3+) numpy数组。"""
    if file_path.endswith('.bin'):
        data = np.fromfile(file_path, dtype=np.float32)
        if len(data) % 4 == 0:
            pts = data.reshape(-1, 4)
        elif len(data) % 3 == 0:
            pts = data.reshape(-1, 3)
        else:
            raise ValueError(f"Cannot reshape .bin data: {len(data)} floats")
        return pts

    if file_path.endswith('.pcd') and HAS_OPEN3D:
        pcd = o3d.io.read_point_cloud(file_path)
        pts = np.asarray(pcd.points, dtype=np.float32)
        try:
            pcd_full = o3d.t.io.read_point_cloud(file_path)
            if 'intensity' in pcd_full.point:
                intensity = np.asarray(
                    pcd_full.point['intensity'], dtype=np.float32
                ).reshape(-1, 1)
                pts = np.concatenate([pts, intensity], axis=1)
        except Exception:
            pass
        return pts

    # Fallback: 简易 ASCII PCD 解析
    with open(file_path, 'r') as f:
        lines = f.readlines()
    header_end = 0
    for i, line in enumerate(lines):
        if line.startswith('DATA'):
            header_end = i + 1
            break
    points = []
    for line in lines[header_end:]:
        vals = line.strip().split()
        if vals:
            points.append([float(v) for v in vals[:3]])
    return np.array(points, dtype=np.float32)


# ============================================================
# 颜色映射
# ============================================================

def colorize_by_depth(depths: np.ndarray,
                      min_depth: float = 0.0,
                      max_depth: float = 50.0) -> np.ndarray:
    """深度->HSV颜色映射：近红(0°)远蓝(240°)。"""
    normalized = np.clip((depths - min_depth) / (max_depth - min_depth), 0.0, 1.0)
    hue = ((1.0 - normalized) * 240).astype(np.uint8)
    hsv = np.zeros((len(depths), 3), dtype=np.uint8)
    hsv[:, 0] = hue
    hsv[:, 1] = 255
    hsv[:, 2] = 255
    hsv_img = hsv.reshape(1, -1, 3)
    bgr = cv2.cvtColor(hsv_img, cv2.COLOR_HSV2BGR)
    return bgr.reshape(-1, 3)


def colorize_by_height(points_cam: np.ndarray,
                       min_z: float = -3.0,
                       max_z: float = 2.0) -> np.ndarray:
    """高度->颜色映射：低蓝高红。"""
    z = points_cam[:, 2]
    normalized = np.clip((z - min_z) / (max_z - min_z), 0.0, 1.0)
    hue = ((1.0 - normalized) * 240).astype(np.uint8)
    hsv = np.zeros((len(z), 3), dtype=np.uint8)
    hsv[:, 0] = hue
    hsv[:, 1] = 255
    hsv[:, 2] = 255
    hsv_img = hsv.reshape(1, -1, 3)
    bgr = cv2.cvtColor(hsv_img, cv2.COLOR_HSV2BGR)
    return bgr.reshape(-1, 3)


# ============================================================
# 投影
# ============================================================

def project_and_colorize(points: np.ndarray,
                         T_lidar_to_cam: np.ndarray,
                         K: np.ndarray,
                         D: np.ndarray,
                         img_size: tuple,
                         model_type: str = 'KANNALA_BRANDT',
                         color_mode: str = 'depth',
                         max_depth: float = 50.0) -> dict:
    """投影 LiDAR 点到相机并着色。"""
    w, h = img_size
    if len(points) == 0:
        return {'points_2d': np.array([]).reshape(0, 2),
                'colors': np.array([]).reshape(0, 3),
                'depths': np.array([]), 'num_valid': 0}

    # 变换到相机坐标系
    points_cam = transform_points(points[:, :3], T_lidar_to_cam)

    z_mask = CameraModel.depth_mask(points_cam, model_type)
    points_cam_f = points_cam[z_mask]

    if len(points_cam_f) == 0:
        return {'points_2d': np.array([]).reshape(0, 2),
                'colors': np.array([]).reshape(0, 3),
                'depths': np.array([]), 'num_valid': 0}

    # 投影
    points_2d = CameraModel.project(points_cam_f, K, D, (w, h), model_type)

    # 过滤 FOV 内的点
    in_fov = (points_2d[:, 0] >= 0) & (points_2d[:, 0] < w) & \
             (points_2d[:, 1] >= 0) & (points_2d[:, 1] < h)

    points_2d_fov = points_2d[in_fov]
    points_cam_fov = points_cam_f[in_fov]
    depths_fov = points_cam_fov[:, 2]

    # 着色
    if color_mode == 'depth':
        colors = colorize_by_depth(depths_fov, max_depth=max_depth)
    elif color_mode == 'height':
        colors = colorize_by_height(points_cam_fov)
    elif color_mode == 'intensity':
        if points.shape[1] >= 4:
            # 需要追踪 intensity 索引
            intensity = points[z_mask, 3] if points.shape[1] >= 4 else np.ones(len(points_cam_f))
            intensity = intensity[in_fov]
            norm_i = np.clip((intensity - intensity.min()) / (intensity.max() - intensity.min() + 1e-8), 0, 1)
            colors = (np.stack([norm_i] * 3, axis=-1) * 255).astype(np.uint8)
        else:
            colors = np.full((len(points_2d_fov), 3), 255, dtype=np.uint8)
    elif color_mode == 'rainbow':
        # 等间距彩虹色
        n = len(points_2d_fov)
        hues = np.linspace(0, 150, n).astype(np.uint8)  # 红到蓝绿
        hsv = np.zeros((n, 3), dtype=np.uint8)
        hsv[:, 0] = hues
        hsv[:, 1] = 255
        hsv[:, 2] = 255
        hsv_img = hsv.reshape(1, -1, 3)
        colors = cv2.cvtColor(hsv_img, cv2.COLOR_HSV2BGR).reshape(-1, 3)
    else:
        colors = np.full((len(points_2d_fov), 3), 255, dtype=np.uint8)

    return {'points_2d': points_2d_fov, 'colors': colors,
            'depths': depths_fov, 'num_valid': len(points_2d_fov)}


# ============================================================
# 绘制
# ============================================================

def draw_projection(img: np.ndarray,
                    points_2d: np.ndarray,
                    colors: np.ndarray,
                    point_size: int = 2,
                    max_draw_points: int = 80000) -> np.ndarray:
    """在图像上绘制投影点。"""
    vis = img.copy()
    if len(points_2d) == 0:
        return vis
    step = max(1, len(points_2d) // max_draw_points)
    for i in range(0, len(points_2d), step):
        x, y = int(points_2d[i, 0]), int(points_2d[i, 1])
        color = tuple(int(c) for c in colors[i])
        cv2.circle(vis, (x, y), point_size, color, -1)
    return vis


def draw_axes_on_image(img: np.ndarray,
                       T_lidar_to_cam: np.ndarray,
                       K: np.ndarray,
                       D: np.ndarray,
                       img_size: tuple,
                       model_type: str = 'KANNALA_BRANDT',
                       axis_length: float = 1.0) -> np.ndarray:
    """
    在图像上绘制相机坐标系的坐标轴投影。
    X=红(右), Y=绿(下), Z=蓝(前/光轴)。
    按相机模型投影轴的端点。
    """
    w, h = img_size
    if (model_type or '').upper().startswith('PINHOLE'):
        origin = np.array([0.0, 0.0, axis_length])
        z_endpoint = np.array([0.0, 0.0, axis_length * 2.0])
        x_endpoint = np.array([axis_length, 0.0, axis_length])
        y_endpoint = np.array([0.0, axis_length, axis_length])
    else:
        origin = np.array([0.0, 0.0, 0.0])
        z_endpoint = np.array([0.0, 0.0, axis_length])
        x_endpoint = np.array([axis_length, 0.0, 0.0])
        y_endpoint = np.array([0.0, axis_length, 0.0])

    all_pts = np.stack([origin, x_endpoint, y_endpoint, z_endpoint])

    pts_2d = CameraModel.project(all_pts, K, D, (w, h), model_type)

    origin_uv = tuple(int(c) for c in pts_2d[0])
    x_uv = tuple(int(c) for c in pts_2d[1])
    y_uv = tuple(int(c) for c in pts_2d[2])
    z_uv = tuple(int(c) for c in pts_2d[3])

    vis = img.copy()
    cv2.line(vis, origin_uv, x_uv, (0, 0, 255), 2)   # X轴 红色
    cv2.line(vis, origin_uv, y_uv, (0, 255, 0), 2)   # Y轴 绿色
    cv2.line(vis, origin_uv, z_uv, (255, 0, 0), 2)   # Z轴 蓝色
    cv2.circle(vis, origin_uv, 4, (255, 255, 255), -1)

    cv2.putText(vis, 'X', (x_uv[0] + 5, x_uv[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(vis, 'Y', (y_uv[0] + 5, y_uv[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(vis, 'Z', (z_uv[0] + 5, z_uv[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    return vis


BOX_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
]


def euler_matrix(rotation: dict) -> np.ndarray:
    """Match public/js/util.js euler_angle_to_rotate_matrix default ZYX path."""
    rx = float(rotation.get('x', 0.0))
    ry = float(rotation.get('y', 0.0))
    rz = float(rotation.get('z', 0.0))

    sx, cx = math.sin(rx), math.cos(rx)
    sy, cy = math.sin(ry), math.cos(ry)
    sz, cz = math.sin(rz), math.cos(rz)

    R_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    R_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    R_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    return R_x @ R_y @ R_z


def psr_to_corners(psr: dict) -> np.ndarray:
    position = psr.get('position', {})
    scale = psr.get('scale', {})
    rotation = psr.get('rotation', {})

    p = np.array([
        float(position.get('x', 0.0)),
        float(position.get('y', 0.0)),
        float(position.get('z', 0.0)),
    ], dtype=np.float64)
    sx = float(scale.get('x', 0.0)) / 2.0
    sy = float(scale.get('y', 0.0)) / 2.0
    sz = float(scale.get('z', 0.0)) / 2.0

    local = np.array([
        [ sx,  sy, -sz],
        [ sx, -sy, -sz],
        [ sx, -sy,  sz],
        [ sx,  sy,  sz],
        [-sx,  sy, -sz],
        [-sx, -sy, -sz],
        [-sx, -sy,  sz],
        [-sx,  sy,  sz],
    ], dtype=np.float64)

    return (euler_matrix(rotation) @ local.T).T + p


def read_sustech_boxes(data_root: str, frame_id: str) -> list:
    label_path = os.path.join(data_root, 'label', f'{frame_id}.json')
    if not os.path.isfile(label_path):
        return []
    with open(label_path, 'r') as f:
        data = json.load(f)
    return data if isinstance(data, list) else []


def draw_boxes_on_image(img: np.ndarray,
                        boxes: list,
                        T_lidar_to_cam: np.ndarray,
                        K: np.ndarray,
                        D: np.ndarray,
                        img_size: tuple,
                        model_type: str = 'KANNALA_BRANDT') -> np.ndarray:
    vis = img.copy()
    w, h = img_size
    for box in boxes:
        psr = box.get('psr')
        if not psr:
            continue
        corners_lidar = psr_to_corners(psr)
        corners_cam = transform_points(corners_lidar, T_lidar_to_cam)
        visible = CameraModel.depth_mask(corners_cam, model_type)
        corners_2d = CameraModel.project(corners_cam, K, D, (w, h), model_type)

        obj_id = str(box.get('obj_id', ''))
        color = (0, 255, 255)
        if obj_id:
            hue = (sum(ord(c) for c in obj_id) * 17) % 180
            hsv = np.uint8([[[hue, 220, 255]]])
            color = tuple(int(c) for c in cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0])

        for a, b in BOX_EDGES:
            if not (visible[a] and visible[b]):
                continue
            pa = corners_2d[a]
            pb = corners_2d[b]
            if not (np.isfinite(pa).all() and np.isfinite(pb).all()):
                continue
            if ((pa[0] < -w or pa[0] > 2 * w or pa[1] < -h or pa[1] > 2 * h) and
                    (pb[0] < -w or pb[0] > 2 * w or pb[1] < -h or pb[1] > 2 * h)):
                continue
            cv2.line(vis, tuple(np.round(pa).astype(int)),
                     tuple(np.round(pb).astype(int)), color, 2)

        center = corners_2d.mean(axis=0)
        if obj_id and np.isfinite(center).all() and 0 <= center[0] < w and 0 <= center[1] < h:
            cv2.putText(vis, obj_id, tuple(np.round(center).astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    return vis


# ============================================================
# 拼接
# ============================================================

def build_grid(images: dict, cam_order: list = None) -> np.ndarray:
    """按相机数量拼接网格。"""
    if cam_order is None:
        cam_order = ['cam_front', 'cam_rear', 'cam_left', 'cam_right']
    vis_list = []
    ref_h, ref_w = list(images.values())[0].shape[:2] if images else (1536, 1920)
    for name in cam_order:
        if name in images:
            img = images[name].copy()
            if img.shape[:2] != (ref_h, ref_w):
                interp = cv2.INTER_AREA if img.shape[0] > ref_h or img.shape[1] > ref_w else cv2.INTER_LINEAR
                img = cv2.resize(img, (ref_w, ref_h), interpolation=interp)
            vis_list.append(img)
        else:
            ph = np.zeros((ref_h, ref_w, 3), dtype=np.uint8)
            cv2.putText(ph, f"{name} (N/A)", (200, ref_h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
            vis_list.append(ph)
    h, w = vis_list[0].shape[:2]
    cols = int(math.ceil(math.sqrt(len(vis_list))))
    rows = int(math.ceil(len(vis_list) / cols))
    canvas = np.zeros((h * rows, w * cols, 3), dtype=np.uint8)
    for i, img in enumerate(vis_list):
        r, c = i // cols, i % cols
        # 在每张图的标题栏标相机名
        cv2.putText(img, cam_order[i], (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        canvas[r * h:(r + 1) * h, c * w:(c + 1) * w] = img
    return canvas


# ============================================================
# 帧处理器
# ============================================================

def count_frames(data_root: str, lidar_name: str = 'lidar_front') -> int:
    """统计数据目录中的总帧数。支持 .pcd 和 .bin 格式。"""
    d = os.path.join(data_root, 'lidar', lidar_name)
    if not os.path.exists(d):
        d = os.path.join(data_root, 'lidar')
    if not os.path.exists(d):
        return 0
    return len([f for f in os.listdir(d)
                if f.endswith('.pcd') or f.endswith('.bin')])


def get_frame_file(data_root: str, sensor_type: str, sensor_name: str,
                   frame_idx: int, ext: str) -> str:
    """
    获取指定帧的文件路径。支持两种命名模式:
    1. 序号命名: 0000.pcd, 0001.pcd (旧数据集)
    2. 时间戳命名: 1776131426033673048.bin (新数据集)
    按文件名排序后用索引匹配。
    """
    d = os.path.join(data_root, sensor_type, sensor_name)
    if sensor_type == 'lidar' and not os.path.exists(d):
        d = os.path.join(data_root, sensor_type)
    if not os.path.exists(d):
        return None
    files = sorted([f for f in os.listdir(d) if f.endswith(ext)])
    if frame_idx < 0 or frame_idx >= len(files):
        return None
    return os.path.join(d, files[frame_idx])


def get_frame_id(data_root: str, sensor_type: str, sensor_name: str,
                 frame_idx: int) -> str:
    for ext in ('.pcd', '.bin', '.jpg', '.jpeg', '.png'):
        path = get_frame_file(data_root, sensor_type, sensor_name, frame_idx, ext)
        if path:
            return os.path.splitext(os.path.basename(path))[0]
    return f'{frame_idx:04d}'


def process_frame(calib: CalibData,
                  data_root: str,
                  frame_idx: int,
                  config: dict) -> tuple:
    """处理单帧，返回 (canvas, stats_dict)。"""
    lidar_name = config.get('lidar_name', 'lidar_front')
    show_axes = config.get('show_axes', False)
    frame_id = get_frame_id(data_root, 'lidar', lidar_name, frame_idx)
    boxes = read_sustech_boxes(data_root, frame_id) if config.get('project_boxes') else []

    # 加载 LiDAR
    lidar_file = get_frame_file(data_root, 'lidar', lidar_name, frame_idx, '.pcd')
    if lidar_file is None:
        lidar_file = get_frame_file(data_root, 'lidar', lidar_name, frame_idx, '.bin')
    if lidar_file is None or not os.path.exists(lidar_file):
        return None, None

    points = read_lidar(lidar_file)
    if config.get('sample_rate', 1) > 1:
        points = points[::config['sample_rate']]

    annotated = {}
    stats = {}

    for cam_name in calib.cam_names:
        # 使用物理相机名加载图像（支持 swap）
        phys_name = calib.get_physical_cam_name(cam_name)
        # 兼容 'camera' 和 'image' 两种目录名
        img_file = get_frame_file(data_root, 'camera', phys_name, frame_idx, '.jpg')
        if img_file is None:
            img_file = get_frame_file(data_root, 'image', phys_name, frame_idx, '.jpg')
        if img_file is None or not os.path.exists(img_file):
            continue
        img = cv2.imread(img_file)
        if img is None:
            continue

        # 内参和畸变使用物理相机的参数
        K = calib.get_camera_intrinsics(phys_name)
        D = calib.get_camera_distortion(phys_name)
        w, h = calib.get_camera_resolution(phys_name)
        model_type = calib.get_camera_model(phys_name)
        # 变换使用逻辑相机名（已经被 swap 处理）
        T = calib.get_transform(lidar_name, cam_name)

        result = project_and_colorize(
            points, T, K, D, (w, h),
            model_type=model_type,
            color_mode=config.get('color_mode', 'depth'),
            max_depth=config.get('max_depth', 50.0),
        )

        # 绘制投影点
        vis = draw_projection(
            img, result['points_2d'], result['colors'],
            point_size=config.get('point_size', 2),
        )

        # 绘制坐标轴
        if show_axes:
            vis = draw_axes_on_image(
                vis, T, K, D, (w, h),
                model_type=model_type,
                axis_length=1.5,
            )

        if boxes:
            vis = draw_boxes_on_image(
                vis, boxes, T, K, D, (w, h),
                model_type=model_type,
            )

        annotated[cam_name] = vis
        stats[cam_name] = {'num_valid': result['num_valid'], 'total': len(points)}

        output_dir = config.get('output_dir')
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
            cv2.imwrite(os.path.join(output_dir, f'{frame_id}_{cam_name}.jpg'), vis)

    if not annotated:
        return None, None

    canvas = build_grid(annotated, calib.cam_names)

    # 信息覆盖层
    overlay = canvas.copy()
    # 统计信息
    info_y = 60
    for cname in calib.cam_names:
        if cname in stats:
            s = stats[cname]
            pct = 100.0 * s['num_valid'] / s['total'] if s['total'] > 0 else 0
            cv2.putText(overlay, f"{cname}: {s['num_valid']}/{s['total']} ({pct:.1f}%)",
                        (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            info_y += 22

    # 底栏
    bar_h = 45
    bar = np.zeros((bar_h, overlay.shape[1], 3), dtype=np.uint8)
    color_modes = {'depth': '深度', 'height': '高度', 'intensity': '强度', 'rainbow': '彩虹'}
    cm_label = color_modes.get(config.get('color_mode', 'depth'), config.get('color_mode', 'depth'))

    info_line = (f"Frame: {frame_idx:04d}/{frame_id} | "
                 f"旋转: [{calib.rot_mode}]{calib.ROT_NAMES[calib.rot_mode]} "
                 f"交换: [{calib.swap_mode}]{calib.SWAP_NAMES[calib.swap_mode]} | "
                 f"颜色: {cm_label} | "
                 f"boxes={len(boxes)} | "
                 f"深度={config.get('max_depth', 50):.0f}m | "
                 f"采样=1/{config.get('sample_rate', 1)}")
    help_line = ("n/p:帧 m:旋转 M:相机交换 r:采样 c:颜色 +/-:点 d:深度 a:轴 s:保存 h:帮助 q:退出")

    cv2.putText(bar, info_line, (10, 17),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(bar, help_line, (10, 37),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)

    canvas = np.vstack([overlay, bar])
    return canvas, stats


# ============================================================
# 主交互循环
# ============================================================

def run_interactive(calib: CalibData, data_root: str, config: dict):
    """交互式可视化主循环。"""
    total_frames = count_frames(data_root, config.get('lidar_name', 'lidar_front'))
    if total_frames == 0:
        print("[ERROR] 未找到点云文件")
        return

    print(f"\n总帧数: {total_frames}")
    print(f"LiDAR主坐标: {config.get('lidar_name', 'lidar_front')}")
    print(f"旋转模式: [{calib.rot_mode}] {calib.ROT_NAMES[calib.rot_mode]}")
    print(f"相机交换: [{calib.swap_mode}] {calib.SWAP_NAMES[calib.swap_mode]}")
    print(f"LiDAR外参模式: {calib.lidar_extrinsic_mode}")
    print(f"  {calib.get_mode_description()}")
    for cn in calib.cam_names:
        phys = calib.get_physical_cam_name(cn)
        K = calib.get_camera_intrinsics(phys)
        D = calib.get_camera_distortion(phys)
        w, h = calib.get_camera_resolution(phys)
        T = calib.get_transform(config.get('lidar_name', 'lidar_front'), cn)
        label = f"{cn}" if phys == cn else f"{cn}<-{phys}"
        print(f"  [{label}] {calib.get_camera_model(phys)} {w}x{h} "
              f"T_trans=[{T[0,3]:.3f},{T[1,3]:.3f},{T[2,3]:.3f}] "
              f"D={[f'{d:.4f}' for d in D]}")

    print("\n交互: n/p帧 m旋转(0-3) M相机交换(0-3) r采样 c颜色 +/-点 d深度 a轴 s保存 h帮助 q退出")

    cv2.namedWindow('LiDAR -> Fisheye Projection', cv2.WINDOW_NORMAL)

    frame_idx = config.get('current_frame', 0)
    canvas, stats = process_frame(calib, data_root, frame_idx, config)

    if canvas is None:
        print("[ERROR] 无法处理帧")
        return

    display_scale = config.get('display_scale', 0.5)
    h_disp = int(canvas.shape[0] * display_scale)
    w_disp = int(canvas.shape[1] * display_scale)
    cv2.imshow('LiDAR -> Fisheye Projection',
               cv2.resize(canvas, (w_disp, h_disp)))

    color_modes = ['depth', 'height', 'intensity', 'rainbow']
    depth_ranges = [10.0, 20.0, 30.0, 50.0, 80.0, 100.0]

    while True:
        key = cv2.waitKey(0) & 0xFF
        need_refresh = False

        if key == ord('q') or key == 27:
            break

        elif key == ord('n') or key == 83:  # 下一帧
            if frame_idx + 1 < total_frames:
                frame_idx += 1
                need_refresh = True

        elif key == ord('p') or key == 81:  # 上一帧
            if frame_idx > 0:
                frame_idx -= 1
                need_refresh = True

        elif key == ord('m'):  # 切换旋转模式
            calib.rot_mode = (calib.rot_mode + 1) % 4
            print(f"\n[旋转] -> [{calib.rot_mode}] {calib.ROT_NAMES[calib.rot_mode]}")
            print(f"  {calib.get_mode_description()}")
            for cn in calib.cam_names:
                T = calib.get_transform(config.get('lidar_name', 'lidar_front'), cn)
                print(f"  {cn} T_trans=[{T[0,3]:.3f},{T[1,3]:.3f},{T[2,3]:.3f}]")
            need_refresh = True

        elif key == ord('M'):  # 切换相机交换
            calib.swap_mode = (calib.swap_mode + 1) % 4
            calib._build_camera_map()
            print(f"\n[相机交换] -> [{calib.swap_mode}] {calib.SWAP_NAMES[calib.swap_mode]}")
            print(f"  {calib.get_mode_description()}")
            for cn in calib.cam_names:
                phys = calib.get_physical_cam_name(cn)
                print(f"  {cn} <- {phys}")
            need_refresh = True

        elif key == ord('r'):  # 切换采样率
            rates = [1, 2, 4, 8]
            cur = config.get('sample_rate', 1)
            idx = rates.index(cur) if cur in rates else 0
            config['sample_rate'] = rates[(idx + 1) % len(rates)]
            print(f"\r采样率: 1/{config['sample_rate']}", end='')
            need_refresh = True

        elif key == ord('c'):  # 切换颜色模式
            cur = config.get('color_mode', 'depth')
            idx = color_modes.index(cur) if cur in color_modes else 0
            config['color_mode'] = color_modes[(idx + 1) % len(color_modes)]
            print(f"\r颜色模式: {config['color_mode']}", end='')
            need_refresh = True

        elif key == ord('+') or key == ord('='):
            config['point_size'] = min(10, config.get('point_size', 2) + 1)
            need_refresh = True

        elif key == ord('-'):
            config['point_size'] = max(1, config.get('point_size', 2) - 1)
            need_refresh = True

        elif key == ord('d'):
            cur = config.get('max_depth', 50.0)
            idx = depth_ranges.index(cur) if cur in depth_ranges else 3
            config['max_depth'] = depth_ranges[(idx + 1) % len(depth_ranges)]
            print(f"\r深度范围: {config['max_depth']:.0f}m", end='')
            need_refresh = True

        elif key == ord('a'):
            config['show_axes'] = not config.get('show_axes', False)
            print(f"\r坐标轴显示: {'ON' if config['show_axes'] else 'OFF'}", end='')
            need_refresh = True

        elif key == ord('s'):
            save_path = f'lidar2camera_f{frame_idx:04d}_r{calib.rot_mode}_s{calib.swap_mode}.png'
            cv2.imwrite(save_path, canvas)
            print(f"\n[保存] {save_path}")

        elif key == ord('h'):
            print("\n" + "=" * 50)
            print("交互控制:")
            print("  n/p     上/下一帧")
            print("  m       切换旋转模式 (0=+90° 1=-90° 2=0° 3=180°)")
            print("  M       切换相机交换 (0=无 1=左右↔ 2=前后↔ 3=全部↔)")
            print("  r       切换采样率 (1->2->4->8)")
            print("  c       切换颜色 (深度/高度/强度/彩虹)")
            print("  +/-     调整点大小")
            print("  d       切换深度范围")
            print("  a       显示/隐藏相机坐标轴")
            print("  s       保存截图")
            print("  q/ESC   退出")
            print("=" * 50 + "\n")

        if need_refresh:
            config['current_frame'] = frame_idx
            canvas, stats = process_frame(calib, data_root, frame_idx, config)
            if canvas is not None:
                h_disp = int(canvas.shape[0] * display_scale)
                w_disp = int(canvas.shape[1] * display_scale)
                cv2.imshow('LiDAR -> Fisheye Projection',
                           cv2.resize(canvas, (w_disp, h_disp)))

    cv2.destroyAllWindows()
    print("\n退出.")


# ============================================================
# 入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description='LiDAR->Fisheye 投影 (交互式可视化)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s --data_root data/scene_xxx --frame 0 --lidar_name lidar_top
  %(prog)s --data_root data/scene_xxx --frame 0 --project_boxes --output result.png
  %(prog)s --data_root data/scene_xxx --frame 0 --project_boxes --output_dir projected
        """)
    parser.add_argument('--data_root', type=str,
                        default='/home/qiaofeng/datasets/ros2_bag/'
                                'fisheye_dataset_back',
                        help='数据目录路径')
    parser.add_argument('--frame', type=int, default=0, help='起始帧')
    parser.add_argument('--rot_mode', type=int, default=0,
                        choices=[0, 1, 2, 3],
                        help='旋转模式: 0=+90°[默认] 1=-90° 2=0° 3=180°')
    parser.add_argument('--swap_mode', type=int, default=0,
                        choices=[0, 1, 2, 3],
                        help='相机交换: 0=无[默认] 1=左右↔ 2=前后↔ 3=全部↔')
    parser.add_argument('--point_size', type=int, default=2, help='点大小')
    parser.add_argument('--max_depth', type=float, default=50.0, help='深度范围(m)')
    parser.add_argument('--sample_rate', type=int, default=1, help='采样率')
    parser.add_argument('--color_mode', type=str, default='depth',
                        choices=['depth', 'height', 'intensity', 'rainbow'],
                        help='颜色模式')
    parser.add_argument('--show_axes', action='store_true', help='显示相机坐标轴')
    parser.add_argument('--lidar_name', type=str, default='lidar_top')
    parser.add_argument('--lidar_extrinsic_mode', type=str, default='raw',
                        choices=['raw', 'rot_mode'],
                        help='raw=使用calib原始lidar外参; rot_mode=使用旧调试旋转模式')
    parser.add_argument('--project_boxes', action='store_true',
                        help='读取 label/<frame>.json 并把SUSTechPOINTS 3D框投影到图像')
    parser.add_argument('--output', type=str, default=None, help='保存结果图(不进入交互)')
    parser.add_argument('--output_dir', type=str, default=None,
                        help='保存每路相机投影图的目录')
    parser.add_argument('--scale', type=float, default=0.5, help='显示缩放')
    args = parser.parse_args()

    # 检查
    if not os.path.exists(args.data_root):
        print(f"[ERROR] 数据目录不存在: {args.data_root}")
        sys.exit(1)
    # 标定文件可能在根目录或 transforms/ 子目录
    calib_file = os.path.join(args.data_root, 'calib.json')
    if not os.path.exists(calib_file):
        calib_file = os.path.join(args.data_root, 'transforms', 'calib.json')
    if not os.path.exists(calib_file):
        print(f"[ERROR] 标定文件不存在: {args.data_root}/calib.json 或 "
              f"{args.data_root}/transforms/calib.json")
        sys.exit(1)

    # 加载标定
    calib = CalibData(
        calib_file,
        rot_mode=args.rot_mode,
        swap_mode=args.swap_mode,
        lidar_extrinsic_mode=args.lidar_extrinsic_mode,
    )

    print("=" * 60)
    print("LiDAR -> Fisheye Camera Projection")
    print("=" * 60)
    print(f"数据目录: {args.data_root}")
    print(f"旋转模式: [{calib.rot_mode}] {calib.ROT_NAMES[calib.rot_mode]}")
    print(f"相机交换: [{calib.swap_mode}] {calib.SWAP_NAMES[calib.swap_mode]}")
    print(f"LiDAR外参模式: {calib.lidar_extrinsic_mode}")
    print(f"  {calib.get_mode_description()}")
    print(f"显示顺序: {calib.cam_names}")
    for cn in calib.cam_names:
        phys = calib.get_physical_cam_name(cn)
        print(f"  {cn} <- 物理相机: {phys}")
    print(f"LiDAR: {calib.lidar_names}")

    config = {
        'lidar_name': args.lidar_name,
        'project_boxes': args.project_boxes,
        'output_dir': args.output_dir,
        'current_frame': args.frame,
        'point_size': args.point_size,
        'max_depth': args.max_depth,
        'sample_rate': args.sample_rate,
        'color_mode': args.color_mode,
        'show_axes': args.show_axes,
        'display_scale': args.scale,
    }

    if args.output or args.output_dir:
        # 非交互模式：处理单帧并保存
        canvas, stats = process_frame(calib, args.data_root, args.frame, config)
        if canvas is not None:
            if args.output:
                cv2.imwrite(args.output, canvas)
                print(f"\n[保存] {args.output}")
            if args.output_dir:
                print(f"\n[保存每路相机] {args.output_dir}")
            for cn, s in stats.items():
                pct = 100.0 * s['num_valid'] / s['total'] if s['total'] > 0 else 0
                print(f"  {cn}: {s['num_valid']}/{s['total']} ({pct:.1f}%)")
        else:
            print("[ERROR] 处理失败")
            sys.exit(1)
    else:
        # 交互模式
        run_interactive(calib, args.data_root, config)


if __name__ == '__main__':
    main()
