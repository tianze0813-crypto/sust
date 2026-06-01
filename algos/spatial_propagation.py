import json
import logging
import math
import os

import numpy as np

from . import spatial_config

logger = logging.getLogger("spatial_propagation")


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class PoseLoader:
    def __init__(self, dataset_root):
        self.dataset_root = dataset_root
        self.poses = {}
        self._load()

    def _load(self):
        pose_file = spatial_config.get_pose_file_path(self.dataset_root)
        if not os.path.isfile(pose_file):
            logger.warning("pose file not found: %s", pose_file)
            return

        with open(pose_file, "r") as f:
            raw_data = json.load(f)

        if not isinstance(raw_data, list):
            logger.warning("pose file format unexpected: expected list, got %s", type(raw_data))
            return

        for entry in raw_data:
            frame_id = spatial_config.resolve_field(entry, spatial_config.POSE_FIELD_MAP["frame_id"])
            if frame_id is None:
                continue

            x = spatial_config.resolve_field(entry, spatial_config.POSE_FIELD_MAP["x"])
            y = spatial_config.resolve_field(entry, spatial_config.POSE_FIELD_MAP["y"])
            z = spatial_config.resolve_field(entry, spatial_config.POSE_FIELD_MAP["z"])
            roll = spatial_config.resolve_field(entry, spatial_config.POSE_FIELD_MAP["roll"])
            pitch = spatial_config.resolve_field(entry, spatial_config.POSE_FIELD_MAP["pitch"])
            yaw = spatial_config.resolve_field(entry, spatial_config.POSE_FIELD_MAP["yaw"])

            if any(v is None for v in [x, y, z, roll, pitch, yaw]):
                logger.warning("frame %s: incomplete pose data, skipping", frame_id)
                continue

            self.poses[int(frame_id)] = {
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "roll": float(roll),
                "pitch": float(pitch),
                "yaw": float(yaw),
            }

        logger.info("loaded %d poses from %s", len(self.poses), pose_file)

    def get_pose(self, frame_id):
        return self.poses.get(int(frame_id))

    def has_pose(self, frame_id):
        return int(frame_id) in self.poses

    @property
    def frame_ids(self):
        return sorted(self.poses.keys())


def pose_to_matrix(pose):
    x, y, z = pose["x"], pose["y"], pose["z"]
    roll, pitch, yaw = pose["roll"], pose["pitch"], pose["yaw"]

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ], dtype=np.float64)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def transform_box(box_psr, T_ego_src, T_ego_tgt):
    """
    box_psr: {"position": {x,y,z}, "scale": {x,y,z}, "rotation": {x,y,z}}
    T_ego_src: 4x4 matrix, sensor->world for source frame
    T_ego_tgt: 4x4 matrix, sensor->world for target frame

    Returns: new box_psr in target sensor coordinates
    """
    pos = box_psr["position"]
    rot = box_psr["rotation"]
    scale = box_psr["scale"]

    box_center_src = np.array([pos["x"], pos["y"], pos["z"], 1.0], dtype=np.float64)

    box_center_world = T_ego_src @ box_center_src

    T_ego_tgt_inv = np.linalg.inv(T_ego_tgt)
    box_center_tgt = T_ego_tgt_inv @ box_center_world

    R_src = T_ego_src[:3, :3]
    R_tgt_inv = T_ego_tgt_inv[:3, :3]

    box_rot_src = euler_to_rot_matrix(rot["x"], rot["y"], rot["z"])
    box_rot_world = R_src @ box_rot_src
    box_rot_tgt = R_tgt_inv @ box_rot_world

    new_rot = rot_matrix_to_euler(box_rot_tgt)

    return {
        "position": {
            "x": float(box_center_tgt[0]),
            "y": float(box_center_tgt[1]),
            "z": float(box_center_tgt[2]),
        },
        "scale": {
            "x": scale["x"],
            "y": scale["y"],
            "z": scale["z"],
        },
        "rotation": {
            "x": float(new_rot[0]),
            "y": float(new_rot[1]),
            "z": float(new_rot[2]),
        },
    }


def euler_to_rot_matrix(rx, ry, rz):
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)

    return Rz @ Ry @ Rx


def rot_matrix_to_euler(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.atan2(-R[2, 0], sy)
        rz = 0.0

    return (rx, ry, rz)


def read_pcd_xyz(pcd_path):
    with open(pcd_path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline().decode("utf-8").strip()
            header_lines.append(line)
            if line.startswith("DATA"):
                break

        fields = []
        sizes = []
        types = []
        counts = []
        width = 0
        height = 0
        points = 0
        for line in header_lines:
            parts = line.split()
            if not parts:
                continue
            key = parts[0]
            if key == "FIELDS":
                fields = parts[1:]
            elif key == "SIZE":
                sizes = [int(s) for s in parts[1:]]
            elif key == "TYPE":
                types = parts[1:]
            elif key == "COUNT":
                counts = [int(c) for c in parts[1:]]
            elif key == "WIDTH":
                width = int(parts[1])
            elif key == "HEIGHT":
                height = int(parts[1])
            elif key == "POINTS":
                points = int(parts[1])
        if points == 0:
            points = width * height

        type_to_struct = {"F": "f4", "I": "i4", "U": "u4", "f": "f4", "i": "i4", "u": "u4"}
        fmt_chars = []
        for i, field in enumerate(fields):
            count = counts[i] if i < len(counts) else 1
            t = types[i] if i < len(types) else "F"
            sc = type_to_struct.get(t, "f4")
            fmt_chars.extend([sc] * count)

        point_size = sum(sizes)

        raw_data = f.read()
        total_points = len(raw_data) // point_size

        arr = np.frombuffer(raw_data[: total_points * point_size], dtype=np.float32)
        arr = arr.reshape(total_points, point_size // 4)

        xyz_indices = []
        col = 0
        for i, field in enumerate(fields):
            count = counts[i] if i < len(counts) else 1
            if field in ("x", "y", "z"):
                for j in range(count):
                    xyz_indices.append(col + j)
            col += count

        if len(xyz_indices) >= 3:
            return arr[:, xyz_indices[:3]].astype(np.float64)
        else:
            return arr[:, :3].astype(np.float64)


def count_points_in_box(points, box_psr):
    center = np.array([box_psr["position"]["x"],
                       box_psr["position"]["y"],
                       box_psr["position"]["z"]], dtype=np.float64)
    half_scale = np.array([box_psr["scale"]["x"] / 2.0,
                           box_psr["scale"]["y"] / 2.0,
                           box_psr["scale"]["z"] / 2.0], dtype=np.float64)

    R = euler_to_rot_matrix(
        box_psr["rotation"]["x"],
        box_psr["rotation"]["y"],
        box_psr["rotation"]["z"],
    )
    R_inv = R.T

    local_pts = (points - center) @ R_inv

    in_box = (
        (local_pts[:, 0] >= -half_scale[0]) & (local_pts[:, 0] <= half_scale[0]) &
        (local_pts[:, 1] >= -half_scale[1]) & (local_pts[:, 1] <= half_scale[1]) &
        (local_pts[:, 2] >= -half_scale[2]) & (local_pts[:, 2] <= half_scale[2])
    )

    return int(np.sum(in_box))


def _normalize_psr(box_psr):
    return {
        "position": {
            "x": float(box_psr["position"]["x"]),
            "y": float(box_psr["position"]["y"]),
            "z": float(box_psr["position"]["z"]),
        },
        "scale": {
            "x": float(box_psr["scale"]["x"]),
            "y": float(box_psr["scale"]["y"]),
            "z": float(box_psr["scale"]["z"]),
        },
        "rotation": {
            "x": float(box_psr["rotation"]["x"]),
            "y": float(box_psr["rotation"]["y"]),
            "z": float(box_psr["rotation"]["z"]),
        },
    }


def _frame_to_str(frame):
    try:
        return "{:04d}".format(int(frame))
    except (ValueError, TypeError):
        return str(frame)


def _transform_point(point_xyz, transform):
    point = np.array([point_xyz[0], point_xyz[1], point_xyz[2], 1.0], dtype=np.float64)
    out = transform @ point
    return out[:3]


def _transform_points(points_xyz, transform):
    if len(points_xyz) == 0:
        return np.zeros((0, 3), dtype=np.float64)
    homo = np.concatenate([
        points_xyz.astype(np.float64),
        np.ones((points_xyz.shape[0], 1), dtype=np.float64),
    ], axis=1)
    out = (transform @ homo.T).T
    return out[:, :3]


def _points_to_box_local(points, box_psr):
    center = np.array([
        box_psr["position"]["x"],
        box_psr["position"]["y"],
        box_psr["position"]["z"],
    ], dtype=np.float64)
    rotation = euler_to_rot_matrix(
        box_psr["rotation"]["x"],
        box_psr["rotation"]["y"],
        box_psr["rotation"]["z"],
    )
    return (points - center) @ rotation.T


def _get_anchor_signs(anchor_corner=None):
    anchor_corner = anchor_corner or spatial_config.ANCHOR_CORNER
    mapping = {
        "rear_left": (-1.0, 1.0),
        "rear_right": (-1.0, -1.0),
        "front_left": (1.0, 1.0),
        "front_right": (1.0, -1.0),
    }
    return mapping.get(anchor_corner, (-1.0, 1.0))


def _get_corner_offset(scale, anchor_corner=None):
    x_sign, y_sign = _get_anchor_signs(anchor_corner)
    return np.array([
        x_sign * scale["x"] / 2.0,
        y_sign * scale["y"] / 2.0,
        -scale["z"] / 2.0,
    ], dtype=np.float64)


def _box_anchor_point(box_psr, anchor_corner=None):
    center = np.array([
        box_psr["position"]["x"],
        box_psr["position"]["y"],
        box_psr["position"]["z"],
    ], dtype=np.float64)
    rotation = euler_to_rot_matrix(
        box_psr["rotation"]["x"],
        box_psr["rotation"]["y"],
        box_psr["rotation"]["z"],
    )
    return center + rotation @ _get_corner_offset(box_psr["scale"], anchor_corner)


def _build_box_from_anchor(anchor_point, scale, rotation, anchor_corner=None):
    anchor_point = np.array(anchor_point, dtype=np.float64)
    rotation_matrix = euler_to_rot_matrix(
        rotation["x"],
        rotation["y"],
        rotation["z"],
    )
    center = anchor_point - rotation_matrix @ _get_corner_offset(scale, anchor_corner)
    return {
        "position": {
            "x": float(center[0]),
            "y": float(center[1]),
            "z": float(center[2]),
        },
        "scale": {
            "x": float(scale["x"]),
            "y": float(scale["y"]),
            "z": float(scale["z"]),
        },
        "rotation": {
            "x": float(rotation["x"]),
            "y": float(rotation["y"]),
            "z": float(rotation["z"]),
        },
    }


def _limit_points(points, max_points):
    if len(points) <= max_points:
        return points
    indices = np.linspace(0, len(points) - 1, max_points, dtype=np.int32)
    return points[indices]


def _compute_shape_similarity(template_local, candidate_local, scale):
    if len(template_local) < 3 or len(candidate_local) < 3:
        return 0.0

    template_local = _limit_points(template_local, spatial_config.MAX_TEMPLATE_POINTS)
    candidate_local = _voxel_downsample(candidate_local, voxel_size=0.08)
    candidate_local = _limit_points(candidate_local, spatial_config.MAX_TEMPLATE_POINTS)

    pairwise = np.linalg.norm(
        template_local[:, None, :] - candidate_local[None, :, :],
        axis=2,
    )
    template_to_candidate = np.min(pairwise, axis=1)
    candidate_to_template = np.min(pairwise, axis=0)

    mean_dist = (np.mean(template_to_candidate) + np.mean(candidate_to_template)) / 2.0
    scale_norm = max(0.5, np.linalg.norm([
        scale["x"],
        scale["y"],
        scale["z"],
    ]))
    shape_score = 1.0 / (1.0 + mean_dist / scale_norm)

    count_ratio = min(len(template_local), len(candidate_local)) / max(len(template_local), len(candidate_local))
    return 0.8 * shape_score + 0.2 * count_ratio


def _angular_distance(a, b):
    return abs(wrap_angle(float(a) - float(b)))


def _compute_yaw_smooth_bonus(candidate_yaw, preferred_yaw):
    if preferred_yaw is None:
        return 0.0

    # Favor candidates close to the previous confirmed yaw without fully locking heading.
    dist = _angular_distance(candidate_yaw, preferred_yaw)
    soft_limit = math.radians(12.0)
    normalized = max(0.0, 1.0 - min(dist, soft_limit) / soft_limit)
    return 0.08 * normalized


def _estimate_anchor_shift(candidate_box, candidate_points, anchor_corner=None):
    if len(candidate_points) < 3:
        return np.zeros(3, dtype=np.float64)

    local_points = _points_to_box_local(candidate_points, candidate_box)
    x_sign, y_sign = _get_anchor_signs(anchor_corner)
    expected_anchor = _get_corner_offset(candidate_box["scale"], anchor_corner)

    anchor_x = np.percentile(local_points[:, 0], 90 if x_sign > 0 else 10)
    anchor_y = np.percentile(local_points[:, 1], 90 if y_sign > 0 else 10)
    anchor_z = np.percentile(local_points[:, 2], 8)
    estimated_anchor = np.array([anchor_x, anchor_y, anchor_z], dtype=np.float64)

    half = np.array([
        candidate_box["scale"]["x"] / 2.0,
        candidate_box["scale"]["y"] / 2.0,
        candidate_box["scale"]["z"] / 2.0,
    ], dtype=np.float64)
    max_shift = np.array([
        max(0.12, half[0] * 0.35),
        max(0.12, half[1] * 0.35),
        max(0.08, half[2] * 0.25),
    ], dtype=np.float64)

    delta_local = np.clip(estimated_anchor - expected_anchor, -max_shift, max_shift)
    rotation_matrix = euler_to_rot_matrix(
        candidate_box["rotation"]["x"],
        candidate_box["rotation"]["y"],
        candidate_box["rotation"]["z"],
    )
    return rotation_matrix @ delta_local


def _search_best_box_position_only(target_points_roi, rough_anchor, rough_rotation, scale, template_local,
                                   preferred_yaw=None):
    best = None
    search_stages = [
        {
            "xy": spatial_config.COARSE_STEP_XY_M,
            "z": spatial_config.COARSE_STEP_Z_M,
            "yaw_window": math.radians(12.0),
            "yaw_step": math.radians(3.0),
        },
        {
            "xy": spatial_config.FINE_STEP_XY_M,
            "z": spatial_config.FINE_STEP_Z_M,
            "yaw_window": math.radians(4.0),
            "yaw_step": math.radians(1.0),
        },
    ]
    min_points = max(4, min(len(template_local) // 2, spatial_config.MIN_TEMPLATE_POINTS))
    best_anchor = np.array(rough_anchor, dtype=np.float64)
    best_rotation = {
        "x": float(rough_rotation["x"]),
        "y": float(rough_rotation["y"]),
        "z": float(rough_rotation["z"]),
    }

    for stage_idx, stage in enumerate(search_stages):
        center_anchor = best_anchor if best is not None else np.array(rough_anchor, dtype=np.float64)
        center_rotation = best_rotation if best is not None else rough_rotation
        window_xy = spatial_config.SEARCH_WINDOW_XY_M if stage_idx == 0 else max(
            spatial_config.COARSE_STEP_XY_M,
            spatial_config.SEARCH_WINDOW_XY_M * 0.35,
        )
        window_z = spatial_config.SEARCH_WINDOW_Z_M if stage_idx == 0 else max(
            spatial_config.COARSE_STEP_Z_M,
            spatial_config.SEARCH_WINDOW_Z_M * 0.35,
        )

        x_vals = np.arange(
            center_anchor[0] - window_xy,
            center_anchor[0] + window_xy + stage["xy"] / 2.0,
            stage["xy"],
        )
        y_vals = np.arange(
            center_anchor[1] - window_xy,
            center_anchor[1] + window_xy + stage["xy"] / 2.0,
            stage["xy"],
        )
        z_vals = np.arange(
            center_anchor[2] - window_z,
            center_anchor[2] + window_z + stage["z"] / 2.0,
            stage["z"],
        )
        yaw_vals = np.arange(
            float(center_rotation["z"]) - stage["yaw_window"],
            float(center_rotation["z"]) + stage["yaw_window"] + stage["yaw_step"] / 2.0,
            stage["yaw_step"],
        )

        stage_best = best
        for x in x_vals:
            for y in y_vals:
                for z in z_vals:
                    for yaw in yaw_vals:
                        anchor = np.array([x, y, z], dtype=np.float64)
                        candidate_rotation = {
                            "x": float(center_rotation["x"]),
                            "y": float(center_rotation["y"]),
                            "z": float(wrap_angle(yaw)),
                        }
                        candidate_box = _build_box_from_anchor(anchor, scale, candidate_rotation)
                        candidate_points = _extract_points_in_box(target_points_roi, candidate_box)
                        if len(candidate_points) < min_points:
                            continue

                        candidate_local = _points_to_box_local(candidate_points, candidate_box)
                        shape_score = _compute_shape_similarity(template_local, candidate_local, scale)
                        smooth_bonus = _compute_yaw_smooth_bonus(candidate_rotation["z"], preferred_yaw)
                        score = shape_score + smooth_bonus
                        if stage_best is None or score > stage_best["score"]:
                            stage_best = {
                                "box": candidate_box,
                                "points": candidate_points,
                                "score": float(score),
                                "shape_score": float(shape_score),
                                "smooth_bonus": float(smooth_bonus),
                                "anchor": anchor,
                                "rotation": candidate_rotation,
                            }

        if stage_best is not None:
            best = stage_best
            best_anchor = stage_best["anchor"]
            best_rotation = stage_best["rotation"]

    return best


def check_pause_condition(current_count, reference_count, previous_count,
                          current_pos, previous_pos):
    reasons = []

    if reference_count > 0:
        ratio = current_count / reference_count
        if ratio < spatial_config.SPARSITY_RATIO:
            reasons.append({
                "type": "sparsity",
                "message": "框内点数低于参考帧 {:.0%} (当前: {}, 参考: {})".format(
                    ratio, current_count, reference_count),
                "ratio": ratio,
            })

    if previous_count is not None and previous_count > 0:
        drop_ratio = (previous_count - current_count) / previous_count
        if drop_ratio > spatial_config.DROP_RATIO:
            reasons.append({
                "type": "drop",
                "message": "框内点数骤降 {:.0%} (上帧: {}, 当前: {})".format(
                    drop_ratio, previous_count, current_count),
                "drop_ratio": drop_ratio,
            })

    if previous_pos is not None and current_pos is not None:
        dx = current_pos["x"] - previous_pos["x"]
        dy = current_pos["y"] - previous_pos["y"]
        dz = current_pos["z"] - previous_pos["z"]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist > spatial_config.POSITION_JUMP_M:
            reasons.append({
                "type": "position_jump",
                "message": "框位移突变 {:.2f}m (阈值: {:.1f}m)".format(
                    dist, spatial_config.POSITION_JUMP_M),
                "distance": dist,
            })

    should_pause = len(reasons) > 0
    return should_pause, reasons


def _extract_points_in_box(points, box_psr, scale_ratio=1.0):
    center = np.array([box_psr["position"]["x"],
                       box_psr["position"]["y"],
                       box_psr["position"]["z"]], dtype=np.float64)
    half_scale = np.array([box_psr["scale"]["x"] / 2.0 * scale_ratio,
                           box_psr["scale"]["y"] / 2.0 * scale_ratio,
                           box_psr["scale"]["z"] / 2.0 * scale_ratio], dtype=np.float64)

    R = euler_to_rot_matrix(
        box_psr["rotation"]["x"],
        box_psr["rotation"]["y"],
        box_psr["rotation"]["z"],
    )
    R_inv = R.T

    local_pts = (points - center) @ R_inv

    in_box = (
        (local_pts[:, 0] >= -half_scale[0]) & (local_pts[:, 0] <= half_scale[0]) &
        (local_pts[:, 1] >= -half_scale[1]) & (local_pts[:, 1] <= half_scale[1]) &
        (local_pts[:, 2] >= -half_scale[2]) & (local_pts[:, 2] <= half_scale[2])
    )

    return points[in_box]


def _extract_points_in_box_with_margin(points, box_psr, margin_xy=0.0, margin_z=0.0):
    center = np.array([box_psr["position"]["x"],
                       box_psr["position"]["y"],
                       box_psr["position"]["z"]], dtype=np.float64)
    half_scale = np.array([
        box_psr["scale"]["x"] / 2.0 + max(0.0, margin_xy),
        box_psr["scale"]["y"] / 2.0 + max(0.0, margin_xy),
        box_psr["scale"]["z"] / 2.0 + max(0.0, margin_z),
    ], dtype=np.float64)

    R = euler_to_rot_matrix(
        box_psr["rotation"]["x"],
        box_psr["rotation"]["y"],
        box_psr["rotation"]["z"],
    )
    local_pts = (points - center) @ R.T
    in_box = (
        (local_pts[:, 0] >= -half_scale[0]) & (local_pts[:, 0] <= half_scale[0]) &
        (local_pts[:, 1] >= -half_scale[1]) & (local_pts[:, 1] <= half_scale[1]) &
        (local_pts[:, 2] >= -half_scale[2]) & (local_pts[:, 2] <= half_scale[2])
    )
    return points[in_box]


def _voxel_downsample(points, voxel_size=0.05):
    if len(points) == 0:
        return points

    min_v = np.min(points, axis=0)
    voxel_coords = ((points - min_v) / voxel_size).astype(np.int32)

    voxel_dict = {}
    for i, vc in enumerate(voxel_coords):
        key = (vc[0], vc[1], vc[2])
        if key not in voxel_dict:
            voxel_dict[key] = []
        voxel_dict[key].append(i)

    indices = [v[0] for v in voxel_dict.values()]
    return points[indices]


def stack_points_in_reference_frame(data_dir, scene, ref_frame, box_psr, pose_loader,
                                    frame_radius=4, margin=1.2, max_points=30000,
                                    voxel_size=0.08):
    box_psr = _normalize_psr(box_psr)
    ref_frame = _frame_to_str(ref_frame)

    ref_pose = pose_loader.get_pose(ref_frame)
    if ref_pose is None:
        return {"error": "reference frame {} has no pose".format(ref_frame)}

    try:
        ref_frame_id = int(ref_frame)
    except (TypeError, ValueError):
        return {"error": "invalid reference frame {}".format(ref_frame)}

    frame_ids = pose_loader.frame_ids
    if ref_frame_id not in frame_ids:
        return {"error": "reference frame {} not found in pose sequence".format(ref_frame)}

    frame_radius = max(0, int(frame_radius))
    margin = max(0.0, float(margin))
    max_points = max(2000, int(max_points))
    voxel_size = max(0.01, float(voxel_size))

    ref_idx = frame_ids.index(ref_frame_id)
    window_ids = frame_ids[max(0, ref_idx - frame_radius): min(len(frame_ids), ref_idx + frame_radius + 1)]

    T_ref = pose_to_matrix(ref_pose)
    T_ref_inv = np.linalg.inv(T_ref)
    ref_center = np.array([
        box_psr["position"]["x"],
        box_psr["position"]["y"],
        box_psr["position"]["z"],
    ], dtype=np.float64)
    ref_yaw = float(box_psr["rotation"]["z"])

    stacked_points = []
    frame_stats = []
    frames_used = []
    single_points = np.zeros((0, 3), dtype=np.float64)

    for frame_id in window_ids:
        frame = _frame_to_str(frame_id)
        pose = pose_loader.get_pose(frame_id)
        if pose is None:
            continue

        pcd_path = os.path.join(data_dir, scene, "lidar", "{}.pcd".format(frame))
        if not os.path.isfile(pcd_path):
            continue

        points = read_pcd_xyz(pcd_path)
        T_cur = pose_to_matrix(pose)
        projected_box = transform_box(box_psr, T_ref, T_cur)
        cropped_points = _extract_points_in_box_with_margin(
            points,
            projected_box,
            margin_xy=margin,
            margin_z=max(0.2, margin * 0.5),
        )
        if len(cropped_points) == 0:
            continue

        to_ref = T_ref_inv @ T_cur
        cropped_in_ref = _transform_points(cropped_points, to_ref)
        stacked_points.append(cropped_in_ref)
        frames_used.append(frame)

        if frame == ref_frame:
            single_points = cropped_in_ref

        projected_center_ref = _transform_point([
            projected_box["position"]["x"],
            projected_box["position"]["y"],
            projected_box["position"]["z"],
        ], to_ref)
        frame_stats.append({
            "frame": frame,
            "count": int(len(cropped_points)),
            "center_dx": float(projected_center_ref[0] - ref_center[0]),
            "center_dy": float(projected_center_ref[1] - ref_center[1]),
            "center_dz": float(projected_center_ref[2] - ref_center[2]),
            "yaw_delta": float(wrap_angle(projected_box["rotation"]["z"] - ref_yaw)),
        })

    if len(stacked_points) == 0:
        return {"error": "no points found in the requested stacking window"}

    stacked_points = np.vstack(stacked_points)
    raw_point_count = int(len(stacked_points))
    stacked_points = _voxel_downsample(stacked_points, voxel_size=voxel_size)
    stacked_points = _limit_points(stacked_points, max_points)

    return {
        "ref_frame": ref_frame,
        "frame_radius": frame_radius,
        "margin": margin,
        "single_points": single_points,
        "stacked_points": stacked_points,
        "single_point_count": int(len(single_points)),
        "raw_point_count": raw_point_count,
        "stacked_point_count": int(len(stacked_points)),
        "frames_used": frames_used,
        "frame_stats": frame_stats,
    }


def _build_template_from_stacked_reference(data_dir, scene, ref_frame, ref_box_psr, pose_loader, frame_radius):
    stack_result = stack_points_in_reference_frame(
        data_dir,
        scene,
        ref_frame,
        ref_box_psr,
        pose_loader,
        frame_radius=frame_radius,
        margin=1.2,
        max_points=40000,
        voxel_size=0.06,
    )
    if "error" in stack_result:
        return stack_result

    stacked_points = stack_result.get("stacked_points")
    if stacked_points is None or len(stacked_points) == 0:
        return {"error": "stack reference is empty"}

    template_world = _extract_points_in_box(
        stacked_points,
        ref_box_psr,
        scale_ratio=spatial_config.TEMPLATE_SCALE_RATIO,
    )
    if len(template_world) == 0:
        return {"error": "stack template is empty"}

    template_local = _points_to_box_local(template_world, ref_box_psr)
    template_local = _voxel_downsample(template_local, voxel_size=0.06)
    template_local = _limit_points(template_local, max_points=spatial_config.MAX_TEMPLATE_POINTS * 4)
    stack_result["template_local"] = template_local
    return stack_result


def propagate_with_stack_reference(data_dir, scene, ref_frame, src_frame, tgt_frame,
                                   ref_box_psr, src_box_psr, pose_loader, frame_radius=4,
                                   previous_yaw=None):
    ref_box_psr = _normalize_psr(ref_box_psr)
    src_box_psr = _normalize_psr(src_box_psr)
    ref_pose = pose_loader.get_pose(ref_frame)
    src_pose = pose_loader.get_pose(src_frame)
    tgt_pose = pose_loader.get_pose(tgt_frame)

    if ref_pose is None:
        return {"error": "reference frame {} has no pose data".format(ref_frame)}
    if src_pose is None:
        return {"error": "source frame {} has no pose data".format(src_frame)}
    if tgt_pose is None:
        return {"error": "target frame {} has no pose data".format(tgt_frame)}

    stack_result = _build_template_from_stacked_reference(
        data_dir,
        scene,
        ref_frame,
        ref_box_psr,
        pose_loader,
        frame_radius=frame_radius,
    )
    if "error" in stack_result:
        return stack_result

    template_local = stack_result.get("template_local")
    if template_local is None or len(template_local) < spatial_config.MIN_TEMPLATE_POINTS:
        return {
            "error": "stack template too sparse",
            "method": "stack_reference_failed",
        }

    T_ref = pose_to_matrix(ref_pose)
    T_src = pose_to_matrix(src_pose)
    T_tgt = pose_to_matrix(tgt_pose)

    projected_fixed_src = transform_box(ref_box_psr, T_ref, T_src)
    source_aligned_psr = _normalize_psr(projected_fixed_src)
    source_aligned_psr["position"] = {
        "x": float(src_box_psr["position"]["x"]),
        "y": float(src_box_psr["position"]["y"]),
        "z": float(src_box_psr["position"]["z"]),
    }

    rough_psr = transform_box(source_aligned_psr, T_src, T_tgt)
    rough_anchor = _box_anchor_point(rough_psr)

    tgt_frame_str = _frame_to_str(tgt_frame)
    pcd_tgt = os.path.join(data_dir, scene, "lidar", "{}.pcd".format(tgt_frame_str))
    if not os.path.isfile(pcd_tgt):
        return {"error": "pcd file not found", "fallback_psr": rough_psr}

    try:
        points_tgt = read_pcd_xyz(pcd_tgt)
    except Exception as e:
        logger.warning("read pcd failed: %s", str(e))
        return {"error": "read pcd failed: {}".format(str(e)), "fallback_psr": rough_psr}

    target_points_roi = _extract_points_in_box(
        points_tgt,
        rough_psr,
        scale_ratio=spatial_config.ROI_SCALE_RATIO,
    )
    if len(target_points_roi) < 3:
        return {
            "psr": rough_psr,
            "template_points": len(template_local),
            "method": "stack_pose_only",
            "reason": "target roi too sparse",
        }

    best = _search_best_box_position_only(
        target_points_roi,
        rough_anchor,
        rough_psr["rotation"],
        rough_psr["scale"],
        template_local,
        preferred_yaw=previous_yaw,
    )
    if best is None:
        return {
            "psr": rough_psr,
            "template_points": len(template_local),
            "method": "stack_pose_only",
            "reason": "position-only search failed",
        }

    final_psr = _normalize_psr(best["box"])
    anchor_shift_world = _estimate_anchor_shift(final_psr, best["points"])
    final_psr["position"]["x"] += float(anchor_shift_world[0])
    final_psr["position"]["y"] += float(anchor_shift_world[1])
    final_psr["position"]["z"] += float(anchor_shift_world[2])
    final_psr["scale"] = {
        "x": float(rough_psr["scale"]["x"]),
        "y": float(rough_psr["scale"]["y"]),
        "z": float(rough_psr["scale"]["z"]),
    }
    final_psr["rotation"] = {
        "x": float(best["box"]["rotation"]["x"]),
        "y": float(best["box"]["rotation"]["y"]),
        "z": float(best["box"]["rotation"]["z"]),
    }
    final_psr = refine_box_position_only_by_extreme(target_points_roi, final_psr)

    return {
        "psr": final_psr,
        "template_points": len(template_local),
        "method": "stack_pose_extreme",
        "score": best["score"],
        "shape_score": best.get("shape_score"),
        "smooth_bonus": best.get("smooth_bonus"),
        "yaw_delta": float(wrap_angle(final_psr["rotation"]["z"] - rough_psr["rotation"]["z"])),
        "frames_used": stack_result.get("frames_used", []),
    }

def count_points_for_frame(data_dir, scene, frame, box_psr):
    pcd_path = os.path.join(data_dir, scene, "lidar", "{}.pcd".format(frame))
    if not os.path.isfile(pcd_path):
        return None

    try:
        points = read_pcd_xyz(pcd_path)
        return count_points_in_box(points, box_psr)
    except Exception as e:
        logger.warning("count points failed for frame %s: %s", frame, str(e))
        return None


def refine_box_by_extreme(points, box_psr, ground_level=0.3):
    """
    基于SUSTechPOINTS原有的grow_box/calculate_box_dimension算法，
    通过点云在框局部坐标系下的极值来精修框的位置和尺寸。
    
    算法流程：
    1. 将框内点云变换到局部坐标系
    2. 找到点云的 min/max 极值（排除地面点）
    3. 根据极值重新计算框的中心和尺寸
    4. 保持旋转不变
    
    相比 centroid refinement 的优势：
    - 不仅调整位置，还调整尺寸，使框更贴合物体
    - 基于极值而非质心，对背景点更鲁棒
    - 考虑地面点的影响，避免框底部被地面点拉低
    """
    center = np.array([box_psr["position"]["x"],
                       box_psr["position"]["y"],
                       box_psr["position"]["z"]], dtype=np.float64)
    half_scale = np.array([box_psr["scale"]["x"] / 2.0,
                           box_psr["scale"]["y"] / 2.0,
                           box_psr["scale"]["z"] / 2.0], dtype=np.float64)

    R = euler_to_rot_matrix(
        box_psr["rotation"]["x"],
        box_psr["rotation"]["y"],
        box_psr["rotation"]["z"],
    )
    R_inv = R.T

    # 变换到局部坐标系
    local_pts = (points - center) @ R_inv

    # 筛选框内点
    in_box = (
        (local_pts[:, 0] >= -half_scale[0]) & (local_pts[:, 0] <= half_scale[0]) &
        (local_pts[:, 1] >= -half_scale[1]) & (local_pts[:, 1] <= half_scale[1]) &
        (local_pts[:, 2] >= -half_scale[2]) & (local_pts[:, 2] <= half_scale[2])
    )

    inside_pts = local_pts[in_box]
    if len(inside_pts) < 10:
        # 点数太少，不做拟合
        return dict(box_psr)

    # 动态地面阈值（参考grow_box的逻辑）
    scale_x = box_psr["scale"]["x"]
    scale_y = box_psr["scale"]["y"]
    scale_z = box_psr["scale"]["z"]
    
    if scale_z < 0.6:
        # 矮物体（如路锥），不使用地面阈值
        effective_ground = -half_scale[2]
    else:
        # 高物体，排除地面附近的点
        dynamic_ground = min(scale_z / 3.0, max(0.2, scale_x / 10.0, scale_y / 10.0))
        effective_ground = -half_scale[2] + dynamic_ground

    # 过滤地面点
    above_ground = inside_pts[:, 2] > effective_ground
    valid_pts = inside_pts[above_ground]
    
    if len(valid_pts) < 5:
        # 过滤地面点后点数太少，使用全部框内点
        valid_pts = inside_pts

    # 计算极值（参考calculate_box_dimension的逻辑）
    pmin = np.min(valid_pts, axis=0)
    pmax = np.max(valid_pts, axis=0)

    # 底部补偿（参考calculate_box_dimension中减去0.2地面点的逻辑）
    if scale_z >= 0.6:
        pmin[2] -= ground_level

    # 计算框尺寸
    box_dim = pmax - pmin

    # 计算框中心在局部坐标系中的偏移
    box_center_delta_local = box_dim / 2.0 + pmin

    # 将中心偏移变换回世界坐标系
    box_center_delta_world = R @ box_center_delta_local

    # 新的框中心
    new_center = center + box_center_delta_world

    return {
        "position": {
            "x": float(new_center[0]),
            "y": float(new_center[1]),
            "z": float(new_center[2]),
        },
        "scale": {
            "x": float(box_dim[0]),
            "y": float(box_dim[1]),
            "z": float(box_dim[2]),
        },
        "rotation": {
            "x": box_psr["rotation"]["x"],
            "y": box_psr["rotation"]["y"],
            "z": box_psr["rotation"]["z"],
        },
    }


def refine_box_position_only_by_extreme(points, box_psr):
    center = np.array([box_psr["position"]["x"],
                       box_psr["position"]["y"],
                       box_psr["position"]["z"]], dtype=np.float64)
    scale = np.array([box_psr["scale"]["x"],
                      box_psr["scale"]["y"],
                      box_psr["scale"]["z"]], dtype=np.float64)
    half_scale = scale / 2.0

    R = euler_to_rot_matrix(
        box_psr["rotation"]["x"],
        box_psr["rotation"]["y"],
        box_psr["rotation"]["z"],
    )
    R_inv = R.T

    local_pts = (points - center) @ R_inv
    expand_ratio = max(1.0, float(spatial_config.EXTREME_REFINE_SCALE_RATIO))
    expanded_half = half_scale * expand_ratio
    candidate_mask = (
        (local_pts[:, 0] >= -expanded_half[0]) & (local_pts[:, 0] <= expanded_half[0]) &
        (local_pts[:, 1] >= -expanded_half[1]) & (local_pts[:, 1] <= expanded_half[1]) &
        (local_pts[:, 2] >= -expanded_half[2]) & (local_pts[:, 2] <= expanded_half[2])
    )
    cand_pts = local_pts[candidate_mask]
    if len(cand_pts) < 10:
        return _normalize_psr(box_psr)

    # Mirror SUST's dynamic ground filtering before extracting x/y extremes.
    if scale[2] < 0.6:
        effective_ground = -half_scale[2]
    else:
        dynamic_ground = min(scale[2] / 3.0, max(0.2, scale[0] / 10.0, scale[1] / 10.0))
        effective_ground = -half_scale[2] + dynamic_ground

    inside_mask = (
        (cand_pts[:, 0] >= -half_scale[0] - 0.01) & (cand_pts[:, 0] <= half_scale[0] + 0.01) &
        (cand_pts[:, 1] >= -half_scale[1] - 0.01) & (cand_pts[:, 1] <= half_scale[1] + 0.01) &
        (cand_pts[:, 2] >= -half_scale[2] - 0.01) & (cand_pts[:, 2] <= half_scale[2] + 0.01)
    )
    inside_pts = cand_pts[inside_mask]
    if len(inside_pts) < 5:
        inside_pts = cand_pts

    valid_xy = inside_pts if scale[2] < 0.6 else inside_pts[inside_pts[:, 2] > effective_ground]
    if len(valid_xy) < 5:
        valid_xy = inside_pts

    extreme = {
        "max": {
            "x": float(np.max(valid_xy[:, 0])),
            "y": float(np.max(valid_xy[:, 1])),
            "z": float(np.max(inside_pts[:, 2])),
        },
        "min": {
            "x": float(np.min(valid_xy[:, 0])),
            "y": float(np.min(valid_xy[:, 1])),
            "z": float(np.min(inside_pts[:, 2])),
        },
    }

    min_distance = max(0.05, float(spatial_config.EXTREME_REFINE_MIN_DISTANCE))
    ground_level = max(0.0, effective_ground - (-half_scale[2]))
    extreme_adjusted = True
    loop_count = 0
    while extreme_adjusted and loop_count < 10000:
        loop_count += 1
        extreme_adjusted = False

        checks = [
            ("x", "max", lambda tp: tp[0] > extreme["max"]["x"] and tp[0] < extreme["max"]["x"] + min_distance / 2.0 and
                                  tp[1] < extreme["max"]["y"] and tp[1] > extreme["min"]["y"] and
                                  tp[2] < extreme["max"]["z"] and tp[2] > extreme["min"]["z"] + ground_level),
            ("x", "min", lambda tp: tp[0] < extreme["min"]["x"] and tp[0] > extreme["min"]["x"] - min_distance / 2.0 and
                                  tp[1] < extreme["max"]["y"] and tp[1] > extreme["min"]["y"] and
                                  tp[2] < extreme["max"]["z"] and tp[2] > extreme["min"]["z"] + ground_level),
            ("y", "max", lambda tp: tp[1] > extreme["max"]["y"] and tp[1] < extreme["max"]["y"] + min_distance / 2.0 and
                                  tp[0] < extreme["max"]["x"] and tp[0] > extreme["min"]["x"] and
                                  tp[2] < extreme["max"]["z"] and tp[2] > extreme["min"]["z"] + ground_level),
            ("y", "min", lambda tp: tp[1] < extreme["min"]["y"] and tp[1] > extreme["min"]["y"] - min_distance / 2.0 and
                                  tp[0] < extreme["max"]["x"] and tp[0] > extreme["min"]["x"] and
                                  tp[2] < extreme["max"]["z"] and tp[2] > extreme["min"]["z"] + ground_level),
            ("z", "max", lambda tp: tp[0] < extreme["max"]["x"] and tp[0] > extreme["min"]["x"] and
                                  tp[1] < extreme["max"]["y"] and tp[1] > extreme["min"]["y"] and
                                  tp[2] > extreme["max"]["z"] and tp[2] < extreme["max"]["z"] + min_distance / 2.0),
            ("z", "min", lambda tp: tp[0] < extreme["max"]["x"] and tp[0] > extreme["min"]["x"] and
                                  tp[1] < extreme["max"]["y"] and tp[1] > extreme["min"]["y"] and
                                  tp[2] < extreme["min"]["z"] and tp[2] > extreme["min"]["z"] - min_distance / 2.0),
        ]
        for axis, side, predicate in checks:
            found = next((tp for tp in cand_pts if predicate(tp)), None)
            if found is None:
                continue
            if side == "max":
                extreme["max"][axis] += min_distance / 2.0
            else:
                extreme["min"][axis] -= min_distance / 2.0
            extreme_adjusted = True

    refined_extreme = {
        "max": {
            "x": extreme["max"]["x"] - min_distance / 2.0,
            "y": extreme["max"]["y"] - min_distance / 2.0,
            "z": extreme["max"]["z"] - min_distance / 2.0,
        },
        "min": {
            "x": extreme["min"]["x"] + min_distance / 2.0,
            "y": extreme["min"]["y"] + min_distance / 2.0,
            "z": extreme["min"]["z"] + min_distance / 2.0,
        },
    }

    for tp in cand_pts:
        if (tp[0] > extreme["max"]["x"] or tp[0] < extreme["min"]["x"] or
                tp[1] > extreme["max"]["y"] or tp[1] < extreme["min"]["y"] or
                tp[2] > extreme["max"]["z"] or tp[2] < extreme["min"]["z"]):
            continue

        if tp[2] > extreme["min"]["z"] + ground_level:
            refined_extreme["max"]["x"] = max(refined_extreme["max"]["x"], float(tp[0]))
            refined_extreme["min"]["x"] = min(refined_extreme["min"]["x"], float(tp[0]))
            refined_extreme["max"]["y"] = max(refined_extreme["max"]["y"], float(tp[1]))
            refined_extreme["min"]["y"] = min(refined_extreme["min"]["y"], float(tp[1]))

        refined_extreme["max"]["z"] = max(refined_extreme["max"]["z"], float(tp[2]))
        refined_extreme["min"]["z"] = min(refined_extreme["min"]["z"], float(tp[2]))

    refined_extreme["min"]["z"] -= ground_level

    # Match SUST's noscaling branch: keep size fixed and slide the box so one side
    # adheres to the observed extreme, chosen by lidar-origin direction in box coords.
    origin_local = np.array([-center[0], -center[1], -center[2]], dtype=np.float64) @ R_inv
    delta_local = np.array([
        refined_extreme["max"]["x"] - half_scale[0] if origin_local[0] > 0 else refined_extreme["min"]["x"] + half_scale[0],
        refined_extreme["max"]["y"] - half_scale[1] if origin_local[1] > 0 else refined_extreme["min"]["y"] + half_scale[1],
        refined_extreme["min"]["z"] + half_scale[2],
    ], dtype=np.float64)

    max_move = np.array([
        max(0.12, scale[0] * spatial_config.EXTREME_REFINE_MAX_MOVE_RATIO),
        max(0.12, scale[1] * spatial_config.EXTREME_REFINE_MAX_MOVE_RATIO),
        max(0.08, scale[2] * spatial_config.EXTREME_REFINE_MAX_MOVE_RATIO),
    ], dtype=np.float64)
    delta_local = np.clip(delta_local, -max_move, max_move)
    new_center = center + R @ delta_local

    refined = _normalize_psr(box_psr)
    refined["position"] = {
        "x": float(new_center[0]),
        "y": float(new_center[1]),
        "z": float(new_center[2]),
    }
    return refined


def refine_box_by_centroid(points, box_psr, max_iterations=1, max_move_ratio=0.1):
    """
    通过点云质心迭代精修框的位置，减少pose不准导致的漂移。
    每次迭代：计算框内点的质心 → 将框中心移到质心 → 重复。
    只调整位置，不改变旋转和尺寸。
    
    改进：
    - 限制每次迭代的最大移动距离（不超过框尺寸的 max_move_ratio）
    - 只在点数足够多时才调整（至少5个点）
    - 减少默认迭代次数到1次，避免过度调整
    
    注意：此函数已被 refine_box_by_extreme 替代，保留用于向后兼容。
    """
    refined = {
        "position": dict(box_psr["position"]),
        "scale": dict(box_psr["scale"]),
        "rotation": dict(box_psr["rotation"]),
    }

    box_diag = np.sqrt(
        refined["scale"]["x"] ** 2 +
        refined["scale"]["y"] ** 2 +
        refined["scale"]["z"] ** 2
    )
    max_move = box_diag * max_move_ratio

    for _ in range(max_iterations):
        center = np.array([refined["position"]["x"],
                           refined["position"]["y"],
                           refined["position"]["z"]], dtype=np.float64)
        half_scale = np.array([refined["scale"]["x"] / 2.0,
                               refined["scale"]["y"] / 2.0,
                               refined["scale"]["z"] / 2.0], dtype=np.float64)

        R = euler_to_rot_matrix(
            refined["rotation"]["x"],
            refined["rotation"]["y"],
            refined["rotation"]["z"],
        )
        R_inv = R.T

        local_pts = (points - center) @ R_inv

        in_box = (
            (local_pts[:, 0] >= -half_scale[0]) & (local_pts[:, 0] <= half_scale[0]) &
            (local_pts[:, 1] >= -half_scale[1]) & (local_pts[:, 1] <= half_scale[1]) &
            (local_pts[:, 2] >= -half_scale[2]) & (local_pts[:, 2] <= half_scale[2])
        )

        inside_pts = points[in_box]
        if len(inside_pts) < 5:
            break

        centroid = np.mean(inside_pts, axis=0)

        # 限制移动幅度
        delta = centroid - center
        delta_norm = np.linalg.norm(delta)
        if delta_norm > max_move:
            delta = delta * (max_move / delta_norm)

        refined["position"]["x"] = float(center[0] + delta[0])
        refined["position"]["y"] = float(center[1] + delta[1])
        refined["position"]["z"] = float(center[2] + delta[2])

    return refined
