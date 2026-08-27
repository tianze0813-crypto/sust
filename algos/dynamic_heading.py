import math
import os

import numpy as np

import scene_reader
from .spatial_propagation import PoseLoader, pose_to_matrix, wrap_angle


DEFAULT_MIN_MOVING_DISTANCE_M = 0.3


def _find_box_by_obj_id(annotations, obj_id):
    for ann in annotations or []:
        if str(ann.get("obj_id")) == str(obj_id):
            return ann
    return None


def _frame_to_pose_id(frame):
    try:
        return int(frame)
    except (TypeError, ValueError):
        return None


def _world_center_from_box(box, pose):
    position = box.get("psr", {}).get("position", {})
    local_center = np.array([
        float(position.get("x", 0.0)),
        float(position.get("y", 0.0)),
        float(position.get("z", 0.0)),
        1.0,
    ], dtype=np.float64)
    world_center = pose_to_matrix(pose) @ local_center
    return world_center[:2]


def _find_motion_neighbor(track, index, direction, min_distance):
    anchor = track[index]["world_xy"]
    neighbor_index = index + direction
    while 0 <= neighbor_index < len(track):
        delta = track[neighbor_index]["world_xy"] - anchor
        if float(np.linalg.norm(delta)) >= min_distance:
            return track[neighbor_index]
        neighbor_index += direction
    return None


def _compute_world_yaw(track, index, min_distance):
    previous_sample = _find_motion_neighbor(track, index, -1, min_distance)
    next_sample = _find_motion_neighbor(track, index, 1, min_distance)

    if previous_sample and next_sample:
        motion_vec = next_sample["world_xy"] - previous_sample["world_xy"]
    elif next_sample:
        motion_vec = next_sample["world_xy"] - track[index]["world_xy"]
    elif previous_sample:
        motion_vec = track[index]["world_xy"] - previous_sample["world_xy"]
    else:
        return None

    if float(np.linalg.norm(motion_vec)) < min_distance:
        return None

    return math.atan2(float(motion_vec[1]), float(motion_vec[0]))


def _angular_distance(a, b):
    return abs(wrap_angle(a - b))


def _equivalent_heading_candidates(local_yaw, scale_xy):
    scale_x, scale_y = float(scale_xy["x"]), float(scale_xy["y"])
    return [
        {
            "local_yaw": wrap_angle(local_yaw),
            "scale_x": scale_x,
            "scale_y": scale_y,
        },
        {
            "local_yaw": wrap_angle(local_yaw + math.pi),
            "scale_x": scale_x,
            "scale_y": scale_y,
        },
        {
            "local_yaw": wrap_angle(local_yaw - math.pi / 2.0),
            "scale_x": scale_y,
            "scale_y": scale_x,
        },
        {
            "local_yaw": wrap_angle(local_yaw + math.pi / 2.0),
            "scale_x": scale_y,
            "scale_y": scale_x,
        },
    ]


def _select_equivalent_heading(box, pose_yaw, target_world_yaw):
    rotation = box.setdefault("psr", {}).setdefault("rotation", {})
    scale = box.setdefault("psr", {}).setdefault("scale", {})
    current_local_yaw = float(rotation.get("z", 0.0))

    candidates = _equivalent_heading_candidates(current_local_yaw, scale)
    best = min(
        candidates,
        key=lambda candidate: _angular_distance(
            wrap_angle(candidate["local_yaw"] + float(pose_yaw)),
            target_world_yaw,
        ),
    )
    return best


def fit_moving_direction_by_id(scene, obj_id, min_moving_distance=DEFAULT_MIN_MOVING_DISTANCE_M):
    scene_meta = scene_reader.get_one_scene(scene)
    frames = scene_meta.get("frames", [])
    pose_loader = PoseLoader(os.path.join(scene_reader.root_dir, scene))

    if not frames:
        return {
            "scene": scene,
            "obj_id": str(obj_id),
            "updated_frames": [],
            "fitted_frames": [],
            "skipped_frames": [],
            "message": "scene has no frames",
        }

    if not pose_loader.frame_ids:
        return {
            "scene": scene,
            "obj_id": str(obj_id),
            "updated_frames": [],
            "fitted_frames": [],
            "skipped_frames": [],
            "message": "pose/lidar_pose.json not found or contains no valid poses",
        }

    track = []

    for frame in frames:
        annotations = scene_reader.read_annotations(scene, frame)
        box = _find_box_by_obj_id(annotations, obj_id)
        if not box:
            continue

        pose_id = _frame_to_pose_id(frame)
        if pose_id is None:
            continue

        pose = pose_loader.get_pose(pose_id)
        if not pose:
            continue

        track.append({
            "frame": frame,
            "annotations": annotations,
            "box": box,
            "pose": pose,
            "world_xy": _world_center_from_box(box, pose),
        })

    if len(track) < 2:
        return {
            "scene": scene,
            "obj_id": str(obj_id),
            "updated_frames": [],
            "fitted_frames": [],
            "skipped_frames": [sample["frame"] for sample in track],
            "message": "not enough valid track points to fit moving direction",
        }

    updated_frames = []
    fitted_frames = []
    skipped_frames = []

    for index, sample in enumerate(track):
        world_yaw = _compute_world_yaw(track, index, float(min_moving_distance))
        if world_yaw is None:
            skipped_frames.append(sample["frame"])
            continue

        best_heading = _select_equivalent_heading(sample["box"], sample["pose"]["yaw"], world_yaw)
        box_psr = sample["box"].setdefault("psr", {})
        box_rotation = box_psr.setdefault("rotation", {})
        box_scale = box_psr.setdefault("scale", {})
        old_local_yaw = float(box_rotation.get("z", 0.0))
        old_scale_x = float(box_scale.get("x", 0.0))
        old_scale_y = float(box_scale.get("y", 0.0))

        box_rotation["z"] = float(best_heading["local_yaw"])
        box_scale["x"] = float(best_heading["scale_x"])
        box_scale["y"] = float(best_heading["scale_y"])

        fitted_frames.append(sample["frame"])

        changed = (
            _angular_distance(old_local_yaw, best_heading["local_yaw"]) > 1e-4
            or abs(old_scale_x - float(best_heading["scale_x"])) > 1e-4
            or abs(old_scale_y - float(best_heading["scale_y"])) > 1e-4
        )
        if changed:
            scene_reader.save_annotations(scene, sample["frame"], sample["annotations"])
            updated_frames.append(sample["frame"])

    if updated_frames:
        message = f"updated heading on {len(updated_frames)} frames"
    elif fitted_frames:
        message = "moving direction already matches current heading"
    else:
        message = "no moving direction could be estimated for this id"

    return {
        "scene": scene,
        "obj_id": str(obj_id),
        "updated_frames": updated_frames,
        "updated_count": len(updated_frames),
        "fitted_frames": fitted_frames,
        "fitted_count": len(fitted_frames),
        "skipped_frames": skipped_frames,
        "skipped_count": len(skipped_frames),
        "message": message,
    }
