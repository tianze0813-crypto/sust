import random
import string

import cherrypy
import os
import json
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


class Root(object):
    @cherrypy.expose
    def index(self, scene="", frame=""):
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
          label_dir = "./data/"+scene +"/label"
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

          label_dir = os.path.join("./data", scene, "label")
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
      ck = check.LabelChecker(os.path.join("./data", scene))
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
      return pre_annotate.annotate_file('./data/{}/lidar/{}.pcd'.format(scene,frame))
      


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
      data_dir = "./data"
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
      data_dir = "./data"
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
      data_dir = "./data"
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
      return self.get_all_objs(os.path.join("./data",scene))

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
    cherrypy.quickstart(Root(), '/', config="server.conf")
else:
    application = cherrypy.Application(Root(), '/', config="server.conf")
