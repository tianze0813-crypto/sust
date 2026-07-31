
import os
import json

this_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.join(this_dir, "data")

POINT_EXTS = (".pcd", ".bin")
IMAGE_EXTS = (".jpg", ".jpeg", ".png")
PREFERRED_LIDAR_PRIMARY = "lidar_top"
PREFERRED_CAMERA_ORDER = [
    "cam_front",
    "cam_rear",
    "cam_left",
    "cam_right",
    "cam_x8d",
]


def _list_files_with_ext(path, exts):
    if not os.path.isdir(path):
        return []
    files = []
    for name in os.listdir(path):
        full_path = os.path.join(path, name)
        if not os.path.isfile(full_path):
            continue
        _, ext = os.path.splitext(name)
        if ext.lower() in exts:
            files.append(name)
    files.sort()
    return files


def _ordered_names(names, preferred_order):
    preferred = [name for name in preferred_order if name in names]
    rest = sorted([name for name in names if name not in preferred_order])
    return preferred + rest


def _detect_lidar_layout(scene_dir):
    lidar_dir = os.path.join(scene_dir, "lidar")
    ret = {
        "frames": [],
        "lidar_ext": ".pcd",
        "lidar_sensors": [],
        "lidar_primary": None,
        "lidar_fusion": {"enabled": False},
        "frame_files": {},
    }

    legacy_files = _list_files_with_ext(lidar_dir, POINT_EXTS)
    if legacy_files:
        ret["frames"] = [os.path.splitext(name)[0] for name in legacy_files]
        _, ret["lidar_ext"] = os.path.splitext(legacy_files[0])
        for frame, filename in zip(ret["frames"], legacy_files):
            ret["frame_files"][frame] = {"lidar": {"_default": filename}}
        return ret

    if not os.path.isdir(lidar_dir):
        return ret

    lidar_sensors = []
    files_by_sensor = {}
    for name in os.listdir(lidar_dir):
        sensor_dir = os.path.join(lidar_dir, name)
        if not os.path.isdir(sensor_dir):
            continue
        if name.endswith("_timestamp"):
            continue
        files = _list_files_with_ext(sensor_dir, POINT_EXTS)
        if not files:
            continue
        lidar_sensors.append(name)
        files_by_sensor[name] = files

    if not lidar_sensors:
        return ret

    lidar_sensors.sort()
    primary = PREFERRED_LIDAR_PRIMARY if PREFERRED_LIDAR_PRIMARY in lidar_sensors else lidar_sensors[0]
    primary_files = files_by_sensor[primary]
    frames = [os.path.splitext(name)[0] for name in primary_files]

    _, lidar_ext = os.path.splitext(primary_files[0])
    ret["frames"] = frames
    ret["lidar_ext"] = lidar_ext
    ret["lidar_sensors"] = lidar_sensors
    ret["lidar_primary"] = primary
    ret["lidar_fusion"] = {
        "enabled": len(lidar_sensors) > 1,
        "primary": primary,
        "sensors": lidar_sensors,
    }

    for idx, frame in enumerate(frames):
        ret["frame_files"][frame] = {"lidar": {}}
        for sensor in lidar_sensors:
            sensor_files = files_by_sensor.get(sensor, [])
            if idx < len(sensor_files):
                ret["frame_files"][frame]["lidar"][sensor] = sensor_files[idx]

    return ret


def _detect_camera_layout(scene_dir, frames=None):
    ret = {
        "camera": [],
        "camera_dir": "camera",
        "camera_ext": ".jpg",
        "frame_files": {},
    }

    camera_root = None
    for dirname in ("camera", "image"):
        path = os.path.join(scene_dir, dirname)
        if os.path.isdir(path):
            camera_root = path
            ret["camera_dir"] = dirname
            break

    if camera_root is None:
        return ret

    cameras = []
    files_by_camera = {}
    for name in os.listdir(camera_root):
        cam_dir = os.path.join(camera_root, name)
        if not os.path.isdir(cam_dir):
            continue
        files = _list_files_with_ext(cam_dir, IMAGE_EXTS)
        if not files:
            continue
        cameras.append(name)
        files_by_camera[name] = files

    cameras = _ordered_names(cameras, PREFERRED_CAMERA_ORDER)
    ret["camera"] = cameras
    if cameras:
        _, ret["camera_ext"] = os.path.splitext(files_by_camera[cameras[0]][0])

    if frames:
        for idx, frame in enumerate(frames):
            ret["frame_files"][frame] = {"camera": {}}
            for camera in cameras:
                cam_files = files_by_camera.get(camera, [])
                if idx < len(cam_files):
                    ret["frame_files"][frame]["camera"][camera] = cam_files[idx]

    return ret


def _load_json_if_exists(path):
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        return json.load(f)


def _find_transform_calib_file(scene_dir):
    candidates = [
        os.path.join(scene_dir, "transforms", "calib.json"),
        os.path.join(scene_dir, "calib.json"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    return None


def _flatten_matrix(matrix):
    return [float(value) for row in matrix for value in row]


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


def _build_camera_calib_from_transform(transform_calib, camera_names, primary_lidar):
    if not transform_calib:
        return {}

    transforms = transform_calib.get("tf2base_link", {})
    if primary_lidar not in transforms:
        lidar_names = sorted([name for name in transforms if name.startswith("lidar_")])
        primary_lidar = PREFERRED_LIDAR_PRIMARY if PREFERRED_LIDAR_PRIMARY in lidar_names else None
        if primary_lidar is None and lidar_names:
            primary_lidar = lidar_names[0]
    if not primary_lidar or primary_lidar not in transforms:
        return {}

    lidar_to_base = transforms[primary_lidar]
    camera_calib = {}
    for camera in camera_names or []:
        cam_intrinsic = transform_calib.get(camera, {})
        cam_to_base = transforms.get(camera)
        if not cam_to_base or "K" not in cam_intrinsic:
            continue

        # UI point and box coordinates are in the primary lidar frame.
        lidar_to_cam = _mat4_mul(_mat4_inverse_rigid(cam_to_base), lidar_to_base)
        k_matrix = cam_intrinsic.get("K")
        camera_calib[camera] = {
            "extrinsic": _flatten_matrix(lidar_to_cam),
            "intrinsic": _flatten_matrix(k_matrix),
            "model_type": cam_intrinsic.get("model_type", "PINHOLE"),
            "distortion": [float(x) for x in cam_intrinsic.get("D", [])],
            "D": [float(x) for x in cam_intrinsic.get("D", [])],
            "imgw": int(cam_intrinsic.get("imgw", 0) or 0),
            "imgh": int(cam_intrinsic.get("imgh", 0) or 0),
            "source_lidar": primary_lidar,
        }

    return camera_calib


def get_scene_dir(scene):
    return os.path.join(root_dir, scene)


def get_transform_calib(scene):
    scene_dir = get_scene_dir(scene)
    calib_file = _find_transform_calib_file(scene_dir)
    if not calib_file:
        return None
    return _load_json_if_exists(calib_file)


def get_frame_sensor_file(scene, frame, sensor_type, sensor_name=None):
    scene_meta = get_one_scene(scene)
    frame_files = scene_meta.get("frame_files", {}).get(frame, {})

    if sensor_type == "lidar":
        lidar_files = frame_files.get("lidar", {})
        filename = None
        if sensor_name:
            filename = lidar_files.get(sensor_name)
            if filename:
                return os.path.join(get_scene_dir(scene), "lidar", sensor_name, filename)
        filename = lidar_files.get("_default")
        if filename:
            return os.path.join(get_scene_dir(scene), "lidar", filename)
        return os.path.join(
            get_scene_dir(scene),
            "lidar",
            "{}{}".format(frame, scene_meta.get("lidar_ext", ".pcd")),
        )

    if sensor_type == "camera":
        if not sensor_name:
            return None
        camera_files = frame_files.get("camera", {})
        filename = camera_files.get(sensor_name)
        camera_dir = scene_meta.get("camera_dir", "camera")
        if filename:
            return os.path.join(get_scene_dir(scene), camera_dir, sensor_name, filename)
        return os.path.join(
            get_scene_dir(scene),
            camera_dir,
            sensor_name,
            "{}{}".format(frame, scene_meta.get("camera_ext", ".jpg")),
        )

    return None

def get_all_scenes():
    all_scenes = get_scene_names()
    print(all_scenes)
    return list(map(get_one_scene, all_scenes))

def get_all_scene_desc():
    names = get_scene_names()
    descs = {}
    for n in names:
        descs[n] = get_scene_desc(n)
    return descs

def get_scene_names():
      scenes = os.listdir(root_dir)
      scenes = filter(lambda s: not os.path.exists(os.path.join(root_dir, s, "disable")), scenes)
      scenes = list(scenes)
      scenes.sort()
      return scenes

def get_scene_desc(s):
    scene_dir = os.path.join(root_dir, s)
    if os.path.exists(os.path.join(scene_dir, "desc.json")):
        with open(os.path.join(scene_dir, "desc.json")) as f:
            desc = json.load(f)
            return desc
    return None

def get_one_scene(s):
    scene = {
        "scene": s,
        "frames": []
    }

    scene_dir = os.path.join(root_dir, s)

    lidar_layout = _detect_lidar_layout(scene_dir)
    scene["frames"] = lidar_layout["frames"]
    scene["lidar_ext"] = lidar_layout["lidar_ext"]
    scene["frame_files"] = lidar_layout["frame_files"]
    if lidar_layout["lidar_sensors"]:
        scene["lidar_sensors"] = lidar_layout["lidar_sensors"]
    if lidar_layout["lidar_primary"]:
        scene["lidar_primary"] = lidar_layout["lidar_primary"]
    if lidar_layout["lidar_fusion"].get("enabled"):
        scene["lidar_fusion"] = lidar_layout["lidar_fusion"]

    # point_transform_matrix=[]

    # if os.path.isfile(os.path.join(scene_dir, "point_transform.txt")):
    #     with open(os.path.join(scene_dir, "point_transform.txt"))  as f:
    #         point_transform_matrix=f.read()
    #         point_transform_matrix = point_transform_matrix.split(",")

    
    if os.path.exists(os.path.join(scene_dir, "desc.json")):
        with open(os.path.join(scene_dir, "desc.json")) as f:
            desc = json.load(f)
            scene["desc"] = desc

    calib = {}
    calib_camera={}
    calib_radar={}
    calib_aux_lidar = {}
    if os.path.exists(os.path.join(scene_dir, "calib")):
        if os.path.exists(os.path.join(scene_dir, "calib","camera")):
            calibs = os.listdir(os.path.join(scene_dir, "calib", "camera"))
            for c in calibs:
                calib_file = os.path.join(scene_dir, "calib", "camera", c)
                calib_name, ext = os.path.splitext(c)
                if os.path.isfile(calib_file) and ext==".json":
                    #print(calib_file)
                    with open(calib_file)  as f:
                        cal = json.load(f)
                        calib_camera[calib_name] = cal

    
        if os.path.exists(os.path.join(scene_dir, "calib", "radar")):
            calibs = os.listdir(os.path.join(scene_dir, "calib", "radar"))
            for c in calibs:
                calib_file = os.path.join(scene_dir, "calib", "radar", c)
                calib_name, _ = os.path.splitext(c)
                if os.path.isfile(calib_file):
                    #print(calib_file)
                    with open(calib_file)  as f:
                        cal = json.load(f)
                        calib_radar[calib_name] = cal
        if os.path.exists(os.path.join(scene_dir, "calib", "aux_lidar")):
            calibs = os.listdir(os.path.join(scene_dir, "calib", "aux_lidar"))
            for c in calibs:
                calib_file = os.path.join(scene_dir, "calib", "aux_lidar", c)
                calib_name, _ = os.path.splitext(c)
                if os.path.isfile(calib_file):
                    #print(calib_file)
                    with open(calib_file)  as f:
                        cal = json.load(f)
                        calib_aux_lidar[calib_name] = cal

    # camera names
    camera_layout = _detect_camera_layout(scene_dir, scene["frames"])
    camera = camera_layout["camera"]
    scene["camera_ext"] = camera_layout["camera_ext"]
    scene["camera_dir"] = camera_layout["camera_dir"]
    for frame, files in camera_layout["frame_files"].items():
        scene["frame_files"].setdefault(frame, {}).update(files)

    transform_calib_file = _find_transform_calib_file(scene_dir)
    transform_calib = _load_json_if_exists(transform_calib_file) if transform_calib_file else None
    if transform_calib:
        transform_camera_calib = _build_camera_calib_from_transform(
            transform_calib,
            camera,
            lidar_layout.get("lidar_primary") or PREFERRED_LIDAR_PRIMARY,
        )
        for name, cal in transform_camera_calib.items():
            calib_camera.setdefault(name, cal)


    # radar names
    radar = []
    radar_ext = ""
    radar_path = os.path.join(scene_dir, "radar")
    if os.path.exists(radar_path):
        radars = os.listdir(radar_path)
        for r in radars:
            radar_file = os.path.join(scene_dir, "radar", r)
            if os.path.isdir(radar_file):
                radar.append(r)
                if radar_ext == "":
                    #detect camera file ext
                    files = os.listdir(radar_file)
                    if len(files)>=2:
                        _,radar_ext = os.path.splitext(files[0])

    if radar_ext == "":
        radar_ext = ".pcd"
    scene["radar_ext"] = radar_ext


    # aux lidar names
    aux_lidar = []
    aux_lidar_ext = ""
    aux_lidar_path = os.path.join(scene_dir, "aux_lidar")
    if os.path.exists(aux_lidar_path):
        lidars = os.listdir(aux_lidar_path)
        for r in lidars:
            lidar_file = os.path.join(scene_dir, "aux_lidar", r)
            if os.path.isdir(lidar_file):
                aux_lidar.append(r)
                if radar_ext == "":
                    #detect camera file ext
                    files = os.listdir(radar_file)
                    if len(files)>=2:
                        _,aux_lidar_ext = os.path.splitext(files[0])

    if aux_lidar_ext == "":
        aux_lidar_ext = ".pcd"
    scene["aux_lidar_ext"] = aux_lidar_ext


    # # ego_pose
    # ego_pose= {}
    # ego_pose_path = os.path.join(scene_dir, "ego_pose")
    # if os.path.exists(ego_pose_path):
    #     poses = os.listdir(ego_pose_path)
    #     for p in poses:
    #         p_file = os.path.join(ego_pose_path, p)
    #         with open(p_file)  as f:
    #                 pose = json.load(f)
    #                 ego_pose[os.path.splitext(p)[0]] = pose


    if  True: #not os.path.isdir(os.path.join(scene_dir, "bbox.xyz")):
        scene["boxtype"] = "psr"
        # if point_transform_matrix:
        #     scene["point_transform_matrix"] = point_transform_matrix
        if camera:
            scene["camera"] = camera
        if radar:
            scene["radar"] = radar
        if aux_lidar:
            scene["aux_lidar"] = aux_lidar
        if calib_camera:
            calib["camera"] = calib_camera
        if calib_radar:
            calib["radar"] = calib_radar
        if calib_aux_lidar:
            calib["aux_lidar"] = calib_aux_lidar
        # if ego_pose:
        #     scene["ego_pose"] = ego_pose
            
    # else:
    #     scene["boxtype"] = "xyz"
    #     if point_transform_matrix:
    #         scene["point_transform_matrix"] = point_transform_matrix
    #     if camera:
    #         scene["camera"] = camera
    #     if radar:
    #         scene["radar"] = radar
    #     if calib_camera:
    #         calib["camera"] = calib_camera
    #     if calib_radar:
    #         calib["radar"] = calib_radar
    #     if calib_aux_lidar:
    #         calib["aux_lidar"] = calib_aux_lidar

    scene["calib"] = calib

    if transform_calib_file:
        scene["transform_calib"] = os.path.relpath(transform_calib_file, scene_dir)


    return scene


def read_annotations(scene, frame):
    filename = os.path.join(root_dir, scene, "label", frame+".json")
    if (os.path.isfile(filename)):
      with open(filename,"r") as f:
        ann=json.load(f)
        #print(ann)          
        return ann
    else:
      return []

def read_ego_pose(scene, frame):
    filename = os.path.join(root_dir, scene, "ego_pose", frame+".json")
    if (os.path.isfile(filename)):
      with open(filename,"r") as f:
        p=json.load(f)
        return p
    else:
      return None

def save_annotations(scene, frame, anno):
    filename = os.path.join(root_dir, scene, "label", frame+".json")
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, 'w') as outfile:
            json.dump(anno, outfile)

if __name__ == "__main__":
    print(get_all_scenes())
