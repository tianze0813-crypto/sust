#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
from dataclasses import dataclass

import numpy as np

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from algos.spatial_propagation import PoseLoader, pose_to_matrix, read_pcd_xyz


@dataclass
class FrameObject:
    frame: str
    psr: dict


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def frame_to_str(frame):
    return f"{int(frame):04d}"


def load_object_sequence(label_dir, obj_id):
    items = []
    for name in sorted(os.listdir(label_dir)):
        if not name.endswith(".json"):
            continue
        frame = name[:-5]
        with open(os.path.join(label_dir, name), "r", encoding="utf-8") as f:
            labels = json.load(f)
        for obj in labels:
            if str(obj.get("obj_id")) == str(obj_id):
                items.append(FrameObject(frame=frame, psr=obj["psr"]))
                break
    return items


def box_rotation(box_psr):
    rx = box_psr["rotation"]["x"]
    ry = box_psr["rotation"]["y"]
    rz = box_psr["rotation"]["z"]
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    rx_mat = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    ry_mat = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    rz_mat = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    return rz_mat @ ry_mat @ rx_mat


def points_in_expanded_box(points, box_psr, margin):
    center = np.array([
        box_psr["position"]["x"],
        box_psr["position"]["y"],
        box_psr["position"]["z"],
    ], dtype=np.float64)
    half_scale = np.array([
        box_psr["scale"]["x"] / 2.0 + margin,
        box_psr["scale"]["y"] / 2.0 + margin,
        box_psr["scale"]["z"] / 2.0 + margin * 0.5,
    ], dtype=np.float64)
    local = (points - center) @ box_rotation(box_psr).T
    mask = (
        (local[:, 0] >= -half_scale[0]) & (local[:, 0] <= half_scale[0]) &
        (local[:, 1] >= -half_scale[1]) & (local[:, 1] <= half_scale[1]) &
        (local[:, 2] >= -half_scale[2]) & (local[:, 2] <= half_scale[2])
    )
    return points[mask], local[mask]


def transform_points(points, transform):
    if len(points) == 0:
        return points.reshape(0, 3)
    homo = np.concatenate([points, np.ones((points.shape[0], 1), dtype=np.float64)], axis=1)
    out = (transform @ homo.T).T
    return out[:, :3]


def make_bev_svg(panels, width=1400, height=440):
    margin = 20
    panel_gap = 20
    panel_width = (width - margin * 2 - panel_gap * (len(panels) - 1)) / len(panels)
    panel_height = height - 2 * margin

    all_xy = []
    for panel in panels:
        pts = panel["points"]
        if len(pts):
            all_xy.append(pts[:, :2])
    if not all_xy:
        raise ValueError("no points to render")

    merged = np.vstack(all_xy)
    min_xy = np.min(merged, axis=0)
    max_xy = np.max(merged, axis=0)
    center = (min_xy + max_xy) / 2.0
    span = np.max(max_xy - min_xy)
    span = max(span, 1.0) * 1.15

    def map_point(xy, x0):
        x = (xy[0] - center[0]) / span + 0.5
        y = (xy[1] - center[1]) / span + 0.5
        px = x0 + x * panel_width
        py = margin + (1.0 - y) * panel_height
        return px, py

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="#111827"/>',
        '<style>text{font-family:Arial,sans-serif} .title{font-size:18px;fill:#e5e7eb;font-weight:bold} .meta{font-size:12px;fill:#cbd5e1}</style>',
    ]

    for idx, panel in enumerate(panels):
        x0 = margin + idx * (panel_width + panel_gap)
        lines.append(f'<rect x="{x0:.2f}" y="{margin}" width="{panel_width:.2f}" height="{panel_height:.2f}" fill="#0f172a" stroke="#334155"/>')
        lines.append(f'<text class="title" x="{x0 + 12:.2f}" y="{margin + 24}">{panel["title"]}</text>')
        lines.append(f'<text class="meta" x="{x0 + 12:.2f}" y="{margin + 42}">{panel["meta"]}</text>')
        cx, cy = map_point(np.array([0.0, 0.0]), x0)
        lines.append(f'<line x1="{x0:.2f}" y1="{cy:.2f}" x2="{x0 + panel_width:.2f}" y2="{cy:.2f}" stroke="#1e293b" stroke-width="1"/>')
        lines.append(f'<line x1="{cx:.2f}" y1="{margin:.2f}" x2="{cx:.2f}" y2="{margin + panel_height:.2f}" stroke="#1e293b" stroke-width="1"/>')

        pts = panel["points"]
        color = panel.get("color", "#60a5fa")
        for pt in pts:
            px, py = map_point(pt[:2], x0)
            lines.append(f'<circle cx="{px:.2f}" cy="{py:.2f}" r="1.1" fill="{color}" fill-opacity="0.75"/>')

        rect = panel.get("rect")
        if rect is not None:
            cx0, cy0, lx, ly, yaw = rect
            corners = np.array([
                [ lx / 2.0,  ly / 2.0],
                [ lx / 2.0, -ly / 2.0],
                [-lx / 2.0, -ly / 2.0],
                [-lx / 2.0,  ly / 2.0],
                [ lx / 2.0,  ly / 2.0],
            ], dtype=np.float64)
            c, s = math.cos(yaw), math.sin(yaw)
            rot = np.array([[c, -s], [s, c]], dtype=np.float64)
            corners = corners @ rot.T + np.array([cx0, cy0], dtype=np.float64)
            mapped = [map_point(corner, x0) for corner in corners]
            path = " ".join(f"{px:.2f},{py:.2f}" for px, py in mapped)
            lines.append(f'<polyline points="{path}" fill="none" stroke="#fbbf24" stroke-width="2"/>')

    lines.append("</svg>")
    return "\n".join(lines)


def save_ply(path, points):
    with open(path, "w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for p in points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


def main():
    parser = argparse.ArgumentParser(description="Create a local stacking demo for a labeled object.")
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--obj-id", required=True)
    parser.add_argument("--ref-frame", required=True)
    parser.add_argument("--frame-radius", type=int, default=4)
    parser.add_argument("--margin", type=float, default=1.2)
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    label_dir = os.path.join(args.dataset_root, "label")
    lidar_dir = os.path.join(args.dataset_root, "lidar")

    sequence = load_object_sequence(label_dir, args.obj_id)
    ref_frame = frame_to_str(args.ref_frame)
    sequence = [item for item in sequence if abs(int(item.frame) - int(ref_frame)) <= args.frame_radius]
    if not sequence:
        raise SystemExit("no matching frames in the requested window")

    ref_item = next((item for item in sequence if item.frame == ref_frame), None)
    if ref_item is None:
        raise SystemExit("reference frame does not contain the requested object")

    pose_loader = PoseLoader(args.dataset_root)
    t_ref = pose_to_matrix(pose_loader.get_pose(int(ref_frame)))
    t_ref_inv = np.linalg.inv(t_ref)

    ref_points = read_pcd_xyz(os.path.join(lidar_dir, ref_frame + ".pcd"))
    single_points, _ = points_in_expanded_box(ref_points, ref_item.psr, args.margin)

    stacked_ref = []
    stacked_local = []
    frame_stats = []

    for item in sequence:
        frame_path = os.path.join(lidar_dir, item.frame + ".pcd")
        if not os.path.isfile(frame_path):
            continue
        pts = read_pcd_xyz(frame_path)
        selected_points, local_points = points_in_expanded_box(pts, item.psr, args.margin)
        if len(selected_points) == 0:
            continue

        t_cur = pose_to_matrix(pose_loader.get_pose(int(item.frame)))
        to_ref = t_ref_inv @ t_cur
        selected_in_ref = transform_points(selected_points, to_ref)
        stacked_ref.append(selected_in_ref)
        stacked_local.append(local_points)

        ref_like_box = np.array([
            item.psr["position"]["x"],
            item.psr["position"]["y"],
            item.psr["position"]["z"],
            1.0,
        ], dtype=np.float64)
        ref_center = (to_ref @ ref_like_box)[:3]
        frame_stats.append({
            "frame": item.frame,
            "count": int(len(selected_points)),
            "center_dx": float(ref_center[0] - ref_item.psr["position"]["x"]),
            "center_dy": float(ref_center[1] - ref_item.psr["position"]["y"]),
            "yaw_d": float(wrap_angle(item.psr["rotation"]["z"] - ref_item.psr["rotation"]["z"])),
        })

    stacked_ref = np.vstack(stacked_ref) if stacked_ref else np.zeros((0, 3), dtype=np.float64)
    stacked_local = np.vstack(stacked_local) if stacked_local else np.zeros((0, 3), dtype=np.float64)

    ref_box = ref_item.psr
    center = ref_box["position"]
    scale = ref_box["scale"]
    yaw = ref_box["rotation"]["z"]
    rect_ref = (center["x"], center["y"], scale["x"], scale["y"], yaw)
    rect_local = (0.0, 0.0, scale["x"], scale["y"], 0.0)

    svg = make_bev_svg([
        {
            "title": f"Single Frame {ref_frame}",
            "meta": f"{len(single_points)} pts in ROI",
            "points": single_points,
            "rect": rect_ref,
            "color": "#60a5fa",
        },
        {
            "title": f"Pose Stack {sequence[0].frame}-{sequence[-1].frame}",
            "meta": f"{len(stacked_ref)} pts from {len(frame_stats)} frames",
            "points": stacked_ref,
            "rect": rect_ref,
            "color": "#34d399",
        },
        {
            "title": "Object Local Stack",
            "meta": f"{len(stacked_local)} pts, labels aligned by box",
            "points": stacked_local,
            "rect": rect_local,
            "color": "#f472b6",
        },
    ])

    svg_path = os.path.join(args.out_dir, f"stack_demo_obj{args.obj_id}_{ref_frame}.svg")
    with open(svg_path, "w", encoding="utf-8") as f:
        f.write(svg)

    save_ply(os.path.join(args.out_dir, f"single_obj{args.obj_id}_{ref_frame}.ply"), single_points)
    save_ply(os.path.join(args.out_dir, f"stack_ref_obj{args.obj_id}_{ref_frame}.ply"), stacked_ref)
    save_ply(os.path.join(args.out_dir, f"stack_local_obj{args.obj_id}_{ref_frame}.ply"), stacked_local)

    stats_path = os.path.join(args.out_dir, f"stack_demo_obj{args.obj_id}_{ref_frame}.json")
    with open(stats_path, "w", encoding="utf-8") as f:
        json.dump({
            "dataset_root": args.dataset_root,
            "obj_id": str(args.obj_id),
            "ref_frame": ref_frame,
            "frame_radius": args.frame_radius,
            "margin": args.margin,
            "frames_used": [item.frame for item in sequence],
            "single_point_count": int(len(single_points)),
            "stack_ref_point_count": int(len(stacked_ref)),
            "stack_local_point_count": int(len(stacked_local)),
            "frame_stats": frame_stats,
        }, f, ensure_ascii=False, indent=2)

    print(svg_path)
    print(stats_path)


if __name__ == "__main__":
    main()
