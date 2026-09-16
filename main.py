import random
import string

import cherrypy
import os
import json
import copy
import struct
import atexit
import mimetypes
import shutil
from jinja2 import Environment, FileSystemLoader
env = Environment(loader=FileSystemLoader('./'))

import os
import sys
import scene_reader
from tools import check_labels  as check


# BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# sys.path.append(BASE_DIR)

#sys.path.append(os.path.join(BASE_DIR, './algos'))
#import algos.rotation as rotation
from algos import pre_annotate
from algos import dynamic_heading
from algos import spatial_propagation
from algos import spatial_config


#sys.path.append(os.path.join(BASE_DIR, '../tracking'))
#import algos.trajectory as trajectory

# extract_object_exe = "~/code/pcltest/build/extract_object"
# registration_exe = "~/code/go_icp_pcl/build/test_go_icp"

# sys.path.append(os.path.join(BASE_DIR, './tools'))
# import tools.dataset_preprocess.crop_scene as crop_scene

# Frame that fused point clouds, box labels and camera extrinsics share.
FUSED_LIDAR_FRAME = "base_link"

# 仓库根目录 & 临时缓存目录（temp 固定放在仓库里，而不是相对启动目录，
# 这样不同启动方式生成的融合缓存能被退出清理逻辑统一清掉）。
REPO_DIR = os.path.dirname(os.path.abspath(__file__))
TEMP_DIR = os.path.join(REPO_DIR, "temp")


def _normalize_frame(frame):
    try:
        return "{:04d}".format(int(frame))
    except (ValueError, TypeError):
        return str(frame)


def _find_box_by_obj_id(annotations, obj_id):
    for box in annotations or []:
        if str(box.get("obj_id")) == str(obj_id):
            return box
    return None


def _extract_box_psr(box):
    psr = box.get("psr", {})
    return {
        "position": {
            "x": float(psr.get("position", {}).get("x", 0.0)),
            "y": float(psr.get("position", {}).get("y", 0.0)),
            "z": float(psr.get("position", {}).get("z", 0.0)),
        },
        "scale": {
            "x": float(psr.get("scale", {}).get("x", 0.0)),
            "y": float(psr.get("scale", {}).get("y", 0.0)),
            "z": float(psr.get("scale", {}).get("z", 0.0)),
        },
        "rotation": {
            "x": float(psr.get("rotation", {}).get("x", 0.0)),
            "y": float(psr.get("rotation", {}).get("y", 0.0)),
            "z": float(psr.get("rotation", {}).get("z", 0.0)),
        },
    }


def _parse_psr_payload(raw_psr):
    if not raw_psr:
        return None
    if isinstance(raw_psr, str):
        raw_psr = json.loads(raw_psr)
    return _extract_box_psr({"psr": raw_psr})


def _remove_obj_from_annotations(annotations, obj_id):
    before_count = len(annotations or [])
    filtered = [
        box for box in (annotations or [])
        if str(box.get("obj_id")) != str(obj_id)
    ]
    return filtered, before_count != len(filtered)


def _wrap_angle(angle):
    return (float(angle) + 3.141592653589793) % (2.0 * 3.141592653589793) - 3.141592653589793


def _mat4_mul(a, b):
    return [
        [
            sum(float(a[r][k]) * float(b[k][c]) for k in range(4))
            for c in range(4)
        ]
        for r in range(4)
    ]


def _mat4_inverse_rigid(m):
    rot_t = [[float(m[c][r]) for c in range(3)] for r in range(3)]
    t = [float(m[r][3]) for r in range(3)]
    inv = [[0.0 for _ in range(4)] for _ in range(4)]
    for r in range(3):
        for c in range(3):
            inv[r][c] = rot_t[r][c]
        inv[r][3] = -sum(rot_t[r][k] * t[k] for k in range(3))
    inv[3][3] = 1.0
    return inv


def _transform_point(m, x, y, z):
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3],
        m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3],
        m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3],
    )


def _infer_bin_fields(byte_len):
    if byte_len % 16 == 0:
        return 4
    if byte_len % 12 == 0:
        return 3
    raise ValueError("unsupported bin point cloud byte length: {}".format(byte_len))


def _append_transformed_bin_points(out, source_path, transform):
    with open(source_path, "rb") as f:
        raw = f.read()

    fields = _infer_bin_fields(len(raw))
    fmt = "<" + "f" * fields
    for record in struct.iter_unpack(fmt, raw):
        x, y, z = record[0], record[1], record[2]
        intensity = record[3] if fields >= 4 else 1.0
        tx, ty, tz = _transform_point(transform, x, y, z)
        out.extend(struct.pack("<ffff", tx, ty, tz, intensity))


def _safe_cache_name(frame):
    return "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in str(frame))


def _build_fused_lidar_bin(scene, frame):
    scene_meta = scene_reader.get_one_scene(scene)
    fusion = scene_meta.get("lidar_fusion", {})
    if not fusion.get("enabled"):
        path = scene_reader.get_frame_sensor_file(scene, frame, "lidar")
        if not path or not os.path.isfile(path):
            raise FileNotFoundError("lidar frame not found: {} {}".format(scene, frame))
        with open(path, "rb") as f:
            return f.read()

    calib = scene_reader.get_transform_calib(scene)
    if not calib or "tf2base_link" not in calib:
        raise FileNotFoundError("transform calib not found for scene {}".format(scene))

    primary = fusion.get("primary") or scene_meta.get("lidar_primary") or "lidar_top"
    sensors = fusion.get("sensors") or scene_meta.get("lidar_sensors") or [primary]
    transforms = calib["tf2base_link"]

    cache_dir = os.path.join(TEMP_DIR, "fused_lidar", scene)
    os.makedirs(cache_dir, exist_ok=True)
    # Frame tag in the cache name keeps base_link output separate from any
    # lidar_top-frame cache left over from before the frame switch.
    cache_path = os.path.join(cache_dir, "{}_{}.bin".format(_safe_cache_name(frame), FUSED_LIDAR_FRAME))

    source_paths = []
    for sensor in sensors:
        source_path = scene_reader.get_frame_sensor_file(scene, frame, "lidar", sensor)
        if source_path and os.path.isfile(source_path) and sensor in transforms:
            source_paths.append((sensor, source_path))
    if not source_paths:
        raise FileNotFoundError("no lidar files found for {} {}".format(scene, frame))

    calib_rel = scene_meta.get("transform_calib")
    mtime_sources = [path for _, path in source_paths]
    if calib_rel:
        mtime_sources.append(os.path.join(scene_reader.get_scene_dir(scene), calib_rel))
    latest_source_mtime = max(os.path.getmtime(path) for path in mtime_sources if os.path.exists(path))
    if (
        os.path.isfile(cache_path)
        and os.path.getsize(cache_path) > 0
        and os.path.getmtime(cache_path) >= latest_source_mtime
    ):
        with open(cache_path, "rb") as f:
            return f.read()

    fused = bytearray()

    for sensor, source_path in source_paths:
        if os.path.getsize(source_path) == 0:
            raise FileNotFoundError(
                "zero-byte lidar source for {} {}: {}".format(
                    scene, frame, source_path
                )
            )
        # Points, boxes and camera extrinsics all live in base_link, so each
        # sensor goes straight to base_link with no primary-lidar detour.
        _append_transformed_bin_points(fused, source_path, transforms[sensor])

    with open(cache_path, "wb") as f:
        f.write(fused)
    return bytes(fused)


def _rotate_obj_yaw_in_annotations(annotations, obj_id, angle_delta, swap_xy=False):
    changed = False
    for box in annotations or []:
        if str(box.get("obj_id")) != str(obj_id):
            continue

        psr = box.setdefault("psr", {})
        rotation = psr.setdefault("rotation", {})
        old_yaw = float(rotation.get("z", 0.0))
        rotation["z"] = _wrap_angle(old_yaw + float(angle_delta))

        if swap_xy:
          scale = psr.setdefault("scale", {})
          old_scale_x = float(scale.get("x", 0.0))
          scale["x"] = float(scale.get("y", 0.0))
          scale["y"] = old_scale_x

        changed = True

    return annotations or [], changed


def _upsert_obj_annotation(annotations, source_annotation):
    annotations = list(annotations or [])
    source_copy = copy.deepcopy(source_annotation)
    obj_id = source_copy.get("obj_id")

    for i, box in enumerate(annotations):
        if str(box.get("obj_id")) == str(obj_id):
            annotations[i] = source_copy
            return annotations

    annotations.append(source_copy)
    return annotations


# ------------------------- 会话级数据集（可选目录） -------------------------
SESSION_DATASET_KEY = "sust_dataset"


def _apply_session_dataset():
    """每个请求开始时执行：把「本浏览器会话」选择的数据根写入当前线程。

    没有会话（或会话里没记录）时恢复成默认数据根（SUST_DATA_ROOT 或 ./data）。
    这样多个用户各自打开不同目录时互不影响。
    """
    root = None
    try:
        root = cherrypy.session.get(SESSION_DATASET_KEY)
    except Exception:
        root = None
    if os.environ.get("SUST_DEBUG"):
        print("[sust][debug] session dataset = {!r}".format(root), flush=True)
    if root and os.path.isdir(root):
        scene_reader.set_root_dir(root)
    else:
        scene_reader.reset_root_dir()


# before_request_body：进入页面处理函数之前先把线程本地数据根准备好。
# priority=60 必须大于 sessions 工具的 50，否则 cherrypy.session 还没初始化。
cherrypy.tools.sust_session_root = cherrypy.Tool(
    "before_request_body", _apply_session_dataset, priority=60
)


# ------------------------- 退出时清理 temp 缓存 -------------------------
_temp_cleaned = False


def clean_temp_cache(force=False):
    """清空 temp 目录下的融合缓存等临时文件（保留 temp 目录本身）。

    默认在后端退出时自动执行；可用 SUST_CLEAN_TEMP_ON_EXIT=0 关闭。
    """
    global _temp_cleaned
    if _temp_cleaned and not force:
        return
    flag = os.environ.get("SUST_CLEAN_TEMP_ON_EXIT", "1").strip().lower()
    if flag in ("0", "false", "no", "off") and not force:
        print("[sust] SUST_CLEAN_TEMP_ON_EXIT off, keep temp cache", flush=True)
        return
    if not os.path.isdir(TEMP_DIR):
        _temp_cleaned = True
        return
    removed = 0
    for name in os.listdir(TEMP_DIR):
        target = os.path.join(TEMP_DIR, name)
        try:
            if os.path.isdir(target) and not os.path.islink(target):
                shutil.rmtree(target, ignore_errors=True)
            else:
                os.remove(target)
            removed += 1
        except OSError as exc:
            print("[sust] clean temp failed: {} ({})".format(target, exc), flush=True)
    _temp_cleaned = True
    print("[sust] temp cache cleaned: {} ({} items)".format(TEMP_DIR, removed), flush=True)


def _install_shutdown_cleanup():
    """进程退出（Ctrl-C / kill / 正常结束）时清掉 temp 缓存。"""
    atexit.register(clean_temp_cache)

    def _on_stop(*args, **kwargs):
        clean_temp_cache()

    try:
        cherrypy.engine.subscribe("stop", _on_stop)
        cherrypy.engine.subscribe("exit", _on_stop)
    except Exception as exc:
        print("[sust] cannot subscribe shutdown cleanup: {}".format(exc), flush=True)


class Root(object):
    # 每个请求都先按会话恢复数据根（并发选择不同目录的关键）
    _cp_config = {"tools.sust_session_root.on": True}

    # 用动态路由 /data/... 取代 server.conf 里的 [/data] 静态目录，
    # 这样数据根可以是前端选择的任意本地目录。
    _MIME_OVERRIDES = {
        ".bin": "application/octet-stream",
        ".pcd": "application/octet-stream",
        ".ply": "application/octet-stream",
        ".jpg": "image/jpeg",
        ".jpeg": "image/jpeg",
        ".png": "image/png",
        ".json": "application/json",
        ".txt": "text/plain; charset=utf-8",
    }

    # ---------------- 目录选择 / 数据根切换 ----------------
    def _set_session_dataset(self, root):
        """把数据根写进当前浏览器会话（cookie session_id）。"""
        try:
            cherrypy.session[SESSION_DATASET_KEY] = root
        except Exception as exc:  # sessions 未开启时退化为进程级默认
            print("[sust] WARN cannot store dataset in session: {}".format(exc), flush=True)

    def _clear_session_dataset(self):
        try:
            cherrypy.session.pop(SESSION_DATASET_KEY, None)
        except Exception:
            pass

    @staticmethod
    def _normalize_user_path(path):
        path = os.path.expanduser(str(path or "").strip())
        if path.startswith("file://"):
            path = path[len("file://"):]
        return path

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_data_root(self):
        return {
            "root": scene_reader.get_root_dir(),
            "default_root": scene_reader.get_default_root_dir(),
        }

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def browse_dir(self, path=""):
        """列出目录内容，供前端「选择数据目录」对话框浏览用。"""
        path = self._normalize_user_path(path) or scene_reader.get_root_dir()
        path = os.path.abspath(path)
        if not os.path.isdir(path):
            return {
                "ok": False,
                "error": "目录不存在: {}".format(path),
                "path": path,
                "parent": None,
                "dirs": [],
            }

        dirs = []
        try:
            names = sorted(os.listdir(path))
        except OSError as exc:
            return {
                "ok": False,
                "error": "无法读取目录: {}".format(exc),
                "path": path,
                "parent": None,
                "dirs": [],
            }

        for name in names:
            if name.startswith("."):
                continue
            full = os.path.join(path, name)
            if not os.path.isdir(full):
                continue
            dirs.append({
                "name": name,
                "path": full,
                "is_scene": scene_reader.looks_like_scene(full),
            })

        parent = os.path.dirname(path)
        return {
            "ok": True,
            "path": path,
            "parent": parent if parent != path else None,
            "is_scene": scene_reader.looks_like_scene(path),
            "scene_names": [d["name"] for d in dirs if d["is_scene"]],
            "dirs": dirs,
            "home": os.path.expanduser("~"),
            "repo": REPO_DIR,
            "default_root": scene_reader.get_default_root_dir(),
        }

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def set_data_root(self, path=""):
        """切换当前会话的数据根目录。

        - path 为空      -> 恢复默认数据根（./data 或 SUST_DATA_ROOT）
        - path 是一个 clip（含 lidar/label/...）-> 用其父目录做根，并返回要打开的 scene
        - path 是包含 scene 子目录的目录      -> 直接作为根
        """
        path = self._normalize_user_path(path)
        if not path:
            self._clear_session_dataset()
            scene_reader.reset_root_dir()
            return {
                "ok": True,
                "reset": True,
                "root": scene_reader.get_root_dir(),
                "scene": "",
                "scene_count": len(scene_reader.get_scene_names()),
            }

        path = os.path.abspath(path)
        if not os.path.isdir(path):
            return {"ok": False, "error": "目录不存在: {}".format(path)}

        scene_name = ""
        if scene_reader.looks_like_scene(path):
            # 用户直接选中某个 clip：根目录取父目录，并顺手打开该 scene
            scene_name = os.path.basename(path)
            root = os.path.dirname(path)
        else:
            root = path

        self._set_session_dataset(root)
        scene_reader.set_root_dir(root)
        scene_names = scene_reader.get_scene_names()
        return {
            "ok": True,
            "root": root,
            "scene": scene_name,
            "scene_count": len(scene_names),
            "scene_names": scene_names,
        }

    @cherrypy.expose
    def data(self, *parts, **kwargs):
        """按当前会话数据根动态提供点云 / 图片等文件。"""
        from cherrypy.lib import static as cherrypy_static

        root = os.path.abspath(scene_reader.get_root_dir())
        rel = os.path.join(*parts) if parts else ""
        target = os.path.abspath(os.path.join(root, rel))
        if target != root and not target.startswith(root + os.sep):
            raise cherrypy.HTTPError(403, "path escapes data root")
        if not os.path.isfile(target):
            raise cherrypy.HTTPError(404, "not found: {}".format(rel))
        ext = os.path.splitext(target)[1].lower()
        ctype = self._MIME_OVERRIDES.get(ext) or mimetypes.guess_type(target)[0] \
            or "application/octet-stream"
        return cherrypy_static.serve_file(target, content_type=ctype)

    @cherrypy.expose
    def index(self, scene="", frame="", dataset=""):
      if dataset:
          path = self._normalize_user_path(dataset)
          if os.path.isdir(path):
              scene_name = ""
              if scene_reader.looks_like_scene(path):
                  scene_name = os.path.basename(path)
                  path = os.path.dirname(path)
              self._set_session_dataset(os.path.abspath(path))
              scene_reader.set_root_dir(path)
              print("[sust] dataset -> root {} (scene {!r})".format(
                  os.path.abspath(path), scene_name), flush=True)
          else:
              print("[sust] WARN dataset not reachable locally: {}".format(dataset), flush=True)
      tmpl = env.get_template('index.html')
      return tmpl.render()
  
    @cherrypy.expose
    def icon(self):
      tmpl = env.get_template('test_icon.html')
      return tmpl.render()

    @cherrypy.expose
    def ml(self):
      tmpl = env.get_template('test_ml.html')
      return tmpl.render()
  
    @cherrypy.expose
    def reg(self):
      tmpl = env.get_template('registration_demo.html')
      return tmpl.render()

    @cherrypy.expose
    def view(self, file):
      tmpl = env.get_template('view.html')
      return tmpl.render()

    # @cherrypy.expose
    # def saveworld(self, scene, frame):

    #   # cl = cherrypy.request.headers['Content-Length']
    #   rawbody = cherrypy.request.body.readline().decode('UTF-8')

    #   with open("./data/"+scene +"/label/"+frame+".json",'w') as f:
    #     f.write(rawbody)
      
    #   return "ok"

    @cherrypy.expose
    def saveworldlist(self):
      try:
        rawbody = cherrypy.request.body.readline().decode('UTF-8')
        data = json.loads(rawbody)

        for d in data:
          scene = d["scene"]
          frame = d["frame"]
          ann = d["annotation"]
          label_dir = scene_reader.get_label_dir(scene)
          os.makedirs(label_dir, exist_ok=True)
          with open(label_dir +"/"+frame+".json",'w') as f:
            json.dump(ann, f, indent=2, sort_keys=True)

        return "ok"
      except Exception as e:
        cherrypy.response.status = 500
        return f"save error: {str(e)}"


    @cherrypy.expose
    @cherrypy.tools.json_out()
    def delete_object_range(self):
      try:
        rawbody = cherrypy.request.body.readline().decode('UTF-8')
        data = json.loads(rawbody)

        scene = data["scene"]
        start_frame = _normalize_frame(data["start_frame"])
        obj_id = data["obj_id"]
        count = data.get("count")

        scene_meta = scene_reader.get_one_scene(scene)
        frames = list(scene_meta.get("frames", []))

        if start_frame not in frames:
          return {"error": "frame {} not found in scene {}".format(start_frame, scene)}

        start_idx = frames.index(start_frame)
        if count in [None, ""]:
          target_frames = frames[start_idx:]
        else:
          count = int(count)
          if count <= 0:
            return {"error": "count must be a positive integer"}
          target_frames = frames[start_idx:start_idx + count]

        updated_frames = []
        backup = []

        for frame in target_frames:
          ann = scene_reader.read_annotations(scene, frame)
          new_ann, changed = _remove_obj_from_annotations(ann, obj_id)

          if not changed:
            continue

          backup.append({
            "scene": scene,
            "frame": frame,
            "annotation": ann,
          })

          label_dir = scene_reader.get_label_dir(scene)
          os.makedirs(label_dir, exist_ok=True)
          with open(os.path.join(label_dir, frame + ".json"), "w") as f:
            json.dump(new_ann, f, indent=2, sort_keys=True)

          updated_frames.append(frame)

        return {
          "scene": scene,
          "start_frame": start_frame,
          "obj_id": str(obj_id),
          "requested_count": count,
          "target_frames": target_frames,
          "updated_frames": updated_frames,
          "backup": backup,
        }
      except Exception as e:
        cherrypy.response.status = 500
        return {"error": str(e)}


    @cherrypy.expose
    @cherrypy.tools.json_out()
    def rotate_object_yaw_all(self):
      try:
        rawbody = cherrypy.request.body.readline().decode('UTF-8')
        data = json.loads(rawbody)

        scene = data["scene"]
        obj_id = data["obj_id"]
        angle_delta = float(data.get("angle_delta", 1.5707963267948966))
        swap_xy = bool(data.get("swap_xy", False))

        scene_meta = scene_reader.get_one_scene(scene)
        frames = list(scene_meta.get("frames", []))

        updated_frames = []
        backup = []

        for frame in frames:
          ann = scene_reader.read_annotations(scene, frame)
          original_ann = copy.deepcopy(ann)
          new_ann, changed = _rotate_obj_yaw_in_annotations(ann, obj_id, angle_delta, swap_xy)

          if not changed:
            continue

          backup.append({
            "scene": scene,
            "frame": frame,
            "annotation": original_ann,
          })

          label_dir = scene_reader.get_label_dir(scene)
          os.makedirs(label_dir, exist_ok=True)
          with open(os.path.join(label_dir, frame + ".json"), "w") as f:
            json.dump(new_ann, f, indent=2, sort_keys=True)

          updated_frames.append(frame)

        return {
          "scene": scene,
          "obj_id": str(obj_id),
          "angle_delta": angle_delta,
          "swap_xy": swap_xy,
          "target_frames": frames,
          "updated_frames": updated_frames,
          "backup": backup,
        }
      except Exception as e:
        cherrypy.response.status = 500
        return {"error": str(e)}


    @cherrypy.expose
    @cherrypy.tools.json_out()
    def copy_object_to_next_frames(self):
      try:
        rawbody = cherrypy.request.body.readline().decode('UTF-8')
        data = json.loads(rawbody)

        scene = data["scene"]
        start_frame = _normalize_frame(data["start_frame"])
        source_annotation = data["annotation"]
        obj_id = source_annotation.get("obj_id")
        count = int(data["count"])

        if count <= 0:
          return {"error": "count must be a positive integer"}

        scene_meta = scene_reader.get_one_scene(scene)
        frames = list(scene_meta.get("frames", []))

        if start_frame not in frames:
          return {"error": "frame {} not found in scene {}".format(start_frame, scene)}

        start_idx = frames.index(start_frame)
        target_frames = frames[start_idx + 1:start_idx + 1 + count]

        updated_frames = []
        backup = []

        for frame in target_frames:
          ann = scene_reader.read_annotations(scene, frame)
          original_ann = copy.deepcopy(ann)
          new_ann = _upsert_obj_annotation(ann, source_annotation)

          backup.append({
            "scene": scene,
            "frame": frame,
            "annotation": original_ann,
          })

          label_dir = scene_reader.get_label_dir(scene)
          os.makedirs(label_dir, exist_ok=True)
          with open(os.path.join(label_dir, frame + ".json"), "w") as f:
            json.dump(new_ann, f, indent=2, sort_keys=True)

          updated_frames.append(frame)

        return {
          "scene": scene,
          "start_frame": start_frame,
          "obj_id": str(obj_id),
          "requested_count": count,
          "target_frames": target_frames,
          "updated_frames": updated_frames,
          "backup": backup,
        }
      except Exception as e:
        cherrypy.response.status = 500
        return {"error": str(e)}


    @cherrypy.expose
    @cherrypy.tools.json_out()
    def cropscene(self):
      rawbody = cherrypy.request.body.readline().decode('UTF-8')
      data = json.loads(rawbody)
      
      rawdata = data["rawSceneId"]

      timestamp = rawdata.split("_")[0]

      print("generate scene")
      log_file = "temp/crop-scene-"+timestamp+".log"

      cmd = "python ./tools/dataset_preprocess/crop_scene.py generate "+ \
        rawdata[0:10]+"/"+timestamp + "_preprocessed/dataset_2hz " + \
        "- " +\
        data["startTime"] + " " +\
        data["seconds"] + " " +\
        "\""+ data["desc"] + "\"" +\
        "> " + log_file + " 2>&1"
      print(cmd)

      code = os.system(cmd)

      with open(log_file) as f:
        log = list(map(lambda s: s.strip(), f.readlines()))

      os.system("rm "+log_file)
      
      return {"code": code,
              "log": log
              }


    @cherrypy.expose
    @cherrypy.tools.json_out()
    def checkscene(self, scene):
      ck = check.LabelChecker(scene_reader.get_scene_dir(scene))
      ck.check()
      print(ck.messages)
      return ck.messages


    # @cherrypy.expose
    # @cherrypy.tools.json_out()
    # def interpolate(self, scene, frame, obj_id):
    #   # interpolate_num = trajectory.predict(scene, obj_id, frame, None)
    #   # return interpolate_num
    #   return 0

    # data  N*3 numpy array
    @cherrypy.expose    
    @cherrypy.tools.json_out()
    def predict_rotation(self):
      cl = cherrypy.request.headers['Content-Length']
      rawbody = cherrypy.request.body.readline().decode('UTF-8')
      
      data = json.loads(rawbody)
      
      return {"angle": pre_annotate.predict_yaw(data["points"])}
      #return {}

    
    @cherrypy.expose    
    @cherrypy.tools.json_out()
    def auto_annotate(self, scene, frame):
      print("auto annotate ", scene, frame)
      return pre_annotate.annotate_file(os.path.join(scene_reader.get_scene_dir(scene), 'lidar', '{}.pcd'.format(frame)))
      


    @cherrypy.expose    
    @cherrypy.tools.json_out()
    def load_annotation(self, scene, frame):
      return scene_reader.read_annotations(scene, frame)


    @cherrypy.expose    
    @cherrypy.tools.json_out()
    def load_ego_pose(self, scene, frame):
      return scene_reader.read_ego_pose(scene, frame)


    @cherrypy.expose
    @cherrypy.tools.json_out()
    def fit_moving_direction_by_id(self, scene, obj_id, min_distance="0.3"):
      try:
        return dynamic_heading.fit_moving_direction_by_id(
          scene=scene,
          obj_id=obj_id,
          min_moving_distance=float(min_distance),
        )
      except Exception as exc:
        return {
          "scene": scene,
          "obj_id": str(obj_id),
          "updated_frames": [],
          "fitted_frames": [],
          "skipped_frames": [],
          "error": str(exc),
        }


    @cherrypy.expose
    @cherrypy.tools.json_out()
    def propagate_init(self, scene, frame, obj_id):
      frame = _normalize_frame(frame)
      data_dir = scene_reader.get_root_dir()
      pose_loader = spatial_propagation.PoseLoader(os.path.join(data_dir, scene))

      if not pose_loader.has_pose(frame):
        return {"error": "frame {} has no pose data".format(frame)}

      ann = scene_reader.read_annotations(scene, frame)
      if ann is None:
        return {"error": "no annotation found for frame {}".format(frame)}

      target_box = _find_box_by_obj_id(ann, obj_id)
      if target_box is None:
        return {"error": "object {} not found in frame {}".format(obj_id, frame)}

      box_psr = _extract_box_psr(target_box)

      pcd_path = os.path.join(data_dir, scene, "lidar", "{}.pcd".format(frame))
      reference_count = None
      if os.path.isfile(pcd_path):
        try:
          points = spatial_propagation.read_pcd_xyz(pcd_path)
          reference_count = spatial_propagation.count_points_in_box(points, box_psr)
        except Exception:
          pass

      frame_ids = pose_loader.frame_ids
      current_idx = frame_ids.index(int(frame)) if int(frame) in frame_ids else -1

      return {
        "scene": scene,
        "anchor_frame": frame,
        "obj_id": obj_id,
        "obj_type": target_box.get("obj_type", ""),
        "obj_attr": target_box.get("obj_attr", ""),
        "box_psr": box_psr,
        "reference_count": reference_count,
        "frame_ids": frame_ids,
        "current_idx": current_idx,
        "total_frames": len(frame_ids),
      }

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def stack_object_points(self, scene, frame, obj_id=None, box_psr=None, frame_radius=4, margin=1.2, max_points=30000):
      frame = _normalize_frame(frame)
      data_dir = scene_reader.get_root_dir()
      pose_loader = spatial_propagation.PoseLoader(os.path.join(data_dir, scene))

      if not pose_loader.has_pose(frame):
        return {"error": "frame {} has no pose data".format(frame)}

      box = _parse_psr_payload(box_psr)
      if box is None:
        if obj_id is None:
          return {"error": "box_psr or obj_id is required"}
        ann = scene_reader.read_annotations(scene, frame)
        if ann is None:
          return {"error": "no annotation found for frame {}".format(frame)}
        target_box = _find_box_by_obj_id(ann, obj_id)
        if target_box is None:
          return {"error": "object {} not found in frame {}".format(obj_id, frame)}
        box = _extract_box_psr(target_box)

      try:
        frame_radius = max(0, int(frame_radius))
      except (TypeError, ValueError):
        frame_radius = 4

      try:
        margin = max(0.0, float(margin))
      except (TypeError, ValueError):
        margin = 1.2

      try:
        max_points = max(2000, int(max_points))
      except (TypeError, ValueError):
        max_points = 30000

      result = spatial_propagation.stack_points_in_reference_frame(
        data_dir,
        scene,
        frame,
        box,
        pose_loader,
        frame_radius=frame_radius,
        margin=margin,
        max_points=max_points,
      )

      if "error" in result:
        return result

      stacked_points = result.get("stacked_points")
      single_points = result.get("single_points")
      return {
        "scene": scene,
        "frame": frame,
        "obj_id": obj_id,
        "frame_radius": frame_radius,
        "margin": margin,
        "point_count": result.get("stacked_point_count"),
        "raw_point_count": result.get("raw_point_count"),
        "single_point_count": result.get("single_point_count"),
        "frames_used": result.get("frames_used"),
        "frame_stats": result.get("frame_stats"),
        "points": stacked_points.tolist() if stacked_points is not None else [],
        "single_points": single_points.tolist() if single_points is not None else [],
      }


    @cherrypy.expose
    @cherrypy.tools.json_out()
    def propagate_check(self, scene, anchor_frame, obj_id, tgt_frame, reference_count, previous_count, previous_pos, previous_yaw=None, prev_frame=None, src_frame=None, src_psr=None, fixed_ref_frame=None, fixed_ref_psr=None, stack_frame_radius=4):
      try:
        return self._propagate_check_impl(scene, anchor_frame, obj_id, tgt_frame, reference_count, previous_count, previous_pos, previous_yaw, prev_frame, src_frame, src_psr, fixed_ref_frame, fixed_ref_psr, stack_frame_radius)
      except Exception as e:
        import traceback
        traceback.print_exc()
        return {"error": str(e)}

    def _propagate_check_impl(self, scene, anchor_frame, obj_id, tgt_frame, reference_count, previous_count, previous_pos, previous_yaw=None, prev_frame=None, src_frame=None, src_psr=None, fixed_ref_frame=None, fixed_ref_psr=None, stack_frame_radius=4):
      anchor_frame = _normalize_frame(anchor_frame)
      tgt_frame = _normalize_frame(tgt_frame)
      if src_frame is not None:
        src_for_matching = _normalize_frame(src_frame)
      elif prev_frame is not None:
        src_for_matching = _normalize_frame(prev_frame)
      else:
        src_for_matching = anchor_frame
      data_dir = scene_reader.get_root_dir()
      pose_loader = spatial_propagation.PoseLoader(os.path.join(data_dir, scene))

      box_psr = _parse_psr_payload(src_psr)
      if box_psr is None:
        ann = scene_reader.read_annotations(scene, src_for_matching)
        if ann is None:
          return {"error": "no annotation for source frame"}
        target_box = _find_box_by_obj_id(ann, obj_id)
        if target_box is None:
          return {"error": "object {} not found".format(obj_id)}
        box_psr = _extract_box_psr(target_box)

      fixed_ref_psr_parsed = _parse_psr_payload(fixed_ref_psr)
      fixed_ref_frame = _normalize_frame(fixed_ref_frame) if fixed_ref_frame else anchor_frame
      try:
        stack_frame_radius = max(0, int(stack_frame_radius))
      except (TypeError, ValueError):
        stack_frame_radius = 4
      try:
        previous_yaw = float(previous_yaw) if previous_yaw not in [None, ""] else None
      except (TypeError, ValueError):
        previous_yaw = None

      if fixed_ref_psr_parsed is None:
        return {"error": "fixed reference box is required for propagation"}

      result = spatial_propagation.propagate_with_stack_reference(
        data_dir,
        scene,
        fixed_ref_frame,
        src_for_matching,
        tgt_frame,
        fixed_ref_psr_parsed,
        box_psr,
        pose_loader,
        frame_radius=stack_frame_radius,
        previous_yaw=previous_yaw,
      )

      if "error" in result:
        return result

      new_psr = result["psr"]

      pcd_path = os.path.join(data_dir, scene, "lidar", "{}.pcd".format(tgt_frame))
      current_count = None
      if os.path.isfile(pcd_path):
        try:
          points = spatial_propagation.read_pcd_xyz(pcd_path)
          current_count = spatial_propagation.count_points_in_box(points, new_psr)
        except Exception:
          pass

      return {
        "frame": tgt_frame,
        "psr": new_psr,
        "point_count": current_count,
        "method": result.get("method"),
        "score": result.get("score"),
        "shape_score": result.get("shape_score"),
        "smooth_bonus": result.get("smooth_bonus"),
        "yaw_delta": result.get("yaw_delta"),
      }


    @cherrypy.expose    
    @cherrypy.tools.json_out()
    def loadworldlist(self):
      rawbody = cherrypy.request.body.readline().decode('UTF-8')
      worldlist = json.loads(rawbody)

      anns = list(map(lambda w:{
                      "scene": w["scene"],
                      "frame": w["frame"],
                      "annotation":scene_reader.read_annotations(w["scene"], w["frame"])},
                      worldlist))

      return anns

    @cherrypy.expose
    def fused_lidar(self, scene, frame):
      try:
        data = _build_fused_lidar_bin(scene, frame)
        cherrypy.response.headers["Content-Type"] = "application/octet-stream"
        cherrypy.response.headers["Content-Length"] = str(len(data))
        return data
      except Exception as e:
        cherrypy.response.status = 500
        return str(e).encode("utf-8")
        

    # @cherrypy.expose    
    # @cherrypy.tools.json_out()
    # def auto_adjust(self, scene, ref_frame, object_id, adj_frame):
      
    #   #os.chdir("./temp")
    #   os.system("rm ./temp/src.pcd ./temp/tgt.pcd ./temp/out.pcd ./temp/trans.json")


    #   tgt_pcd_file = "./data/"+scene +"/lidar/"+ref_frame+".pcd"
    #   tgt_json_file = "./data/"+scene +"/label/"+ref_frame+".json"

    #   src_pcd_file = "./data/"+scene +"/lidar/"+adj_frame+".pcd"      
    #   src_json_file = "./data/"+scene +"/label/"+adj_frame+".json"

    #   cmd = extract_object_exe +" "+ src_pcd_file + " " + src_json_file + " " + object_id + " " +"./temp/src.pcd"
    #   print(cmd)
    #   os.system(cmd)

    #   cmd = extract_object_exe + " "+ tgt_pcd_file + " " + tgt_json_file + " " + object_id + " " +"./temp/tgt.pcd"
    #   print(cmd)
    #   os.system(cmd)

    #   cmd = registration_exe + " ./temp/tgt.pcd ./temp/src.pcd ./temp/out.pcd ./temp/trans.json"
    #   print(cmd)
    #   os.system(cmd)

    #   with open("./temp/trans.json", "r") as f:
    #     trans = json.load(f)
    #     print(trans)
    #     return trans

    #   return {}

    @cherrypy.expose    
    @cherrypy.tools.json_out()
    def datameta(self):
      return scene_reader.get_all_scenes()
    

    @cherrypy.expose    
    @cherrypy.tools.json_out()
    def scenemeta(self, scene):
      return scene_reader.get_one_scene(scene)

    @cherrypy.expose    
    @cherrypy.tools.json_out()
    def get_all_scene_desc(self):
      return scene_reader.get_all_scene_desc()

    @cherrypy.expose    
    @cherrypy.tools.json_out()
    def objs_of_scene(self, scene):
      return self.get_all_objs(scene_reader.get_scene_dir(scene))

    def get_all_objs(self, path):
      label_folder = os.path.join(path, "label")
      if not os.path.isdir(label_folder):
        return []
        
      files = os.listdir(label_folder)

      files = filter(lambda x: x.split(".")[-1]=="json", files)


      def file_2_objs(f):
          with open(f) as fd:
              boxes = json.load(fd)
              objs = [x for x in map(lambda b: {"category":b["obj_type"], "id": b["obj_id"]}, boxes)]
              return objs

      boxes = map(lambda f: file_2_objs(os.path.join(path, "label", f)), files)

      # the following map makes the category-id pairs unique in scene
      all_objs={}
      for x in boxes:
          for o in x:
              
              k = str(o["category"])+"-"+str(o["id"])

              if all_objs.get(k):
                all_objs[k]['count']= all_objs[k]['count']+1
              else:
                all_objs[k]= {
                  "category": o["category"],
                  "id": o["id"],
                  "count": 1
                }

      return [x for x in  all_objs.values()]

if __name__ == '__main__':
    # 后端退出（Ctrl-C / SIGTERM / 正常结束）时清理 temp 下的融合缓存
    _install_shutdown_cleanup()
    cfg = os.environ.get("SUST_CONFIG", "server.conf")
    print("[sust] data root = {} (default)".format(scene_reader.get_default_root_dir()), flush=True)
    print("[sust] dataset   = 前端「选择目录」按浏览器会话记住", flush=True)
    print("[sust] temp cache= {} （退出自动清理，SUST_CLEAN_TEMP_ON_EXIT=0 可关）".format(TEMP_DIR), flush=True)
    print("[sust] config     = {}".format(cfg), flush=True)
    cherrypy.quickstart(Root(), '/', config=cfg)
else:
    _install_shutdown_cleanup()
    application = cherrypy.Application(Root(), '/', config=os.environ.get("SUST_CONFIG", "server.conf"))
