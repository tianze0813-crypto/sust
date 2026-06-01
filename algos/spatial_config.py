import os

SPARSITY_RATIO = 0.2
DROP_RATIO = 0.5
POSITION_JUMP_M = 2.0

SEARCH_WINDOW_XY_M = 1.2
SEARCH_WINDOW_Z_M = 0.4
COARSE_STEP_XY_M = 0.3
FINE_STEP_XY_M = 0.12
COARSE_STEP_Z_M = 0.2
FINE_STEP_Z_M = 0.08
YAW_FINE_TUNE_DEG = 5.0
COARSE_YAW_STEP_DEG = 2.0
FINE_YAW_STEP_DEG = 1.0

ANCHOR_CORNER = "rear_left"
TEMPLATE_SCALE_RATIO = 1.05
ROI_SCALE_RATIO = 1.8
MIN_TEMPLATE_POINTS = 8
MAX_TEMPLATE_POINTS = 96
EXTREME_REFINE_SCALE_RATIO = 1.8
EXTREME_REFINE_MIN_DISTANCE = 0.2
EXTREME_REFINE_MAX_MOVE_RATIO = 0.25

POSE_FILE_NAME = "lidar_pose.json"
POSE_RELATIVE_PATH = "pose"

POSE_FIELD_MAP = {
    "frame_id": "frame_id",
    "x": "pose.x",
    "y": "pose.y",
    "z": "pose.z",
    "roll": "pose.roll",
    "pitch": "pose.pitch",
    "yaw": "pose.yaw",
}

HISTORY_MAX_DEPTH = 50
UNDO_MERGE_INTERVAL_MS = 500


def get_pose_file_path(dataset_root):
    return os.path.join(dataset_root, POSE_RELATIVE_PATH, POSE_FILE_NAME)


def resolve_field(data, field_path):
    keys = field_path.split(".")
    value = data
    for key in keys:
        if isinstance(value, dict):
            value = value.get(key)
        else:
            return None
        if value is None:
            return None
    return value
