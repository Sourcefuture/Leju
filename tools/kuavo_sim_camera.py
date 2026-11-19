#!/usr/bin/env python3
"""Read local Kuavo-5.2 RGB-D images from named MuJoCo cameras."""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import time

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp/kuavo-mujoco-cache")

import cv2
import mujoco
import numpy as np
from kuavo_sim_bin_environment import apply_private_bin_poses

try:
    from kuavo_mujoco_live_viewer import HistoryFollower, arm_addresses, read_latest
except ModuleNotFoundError:
    from tools.kuavo_mujoco_live_viewer import HistoryFollower, arm_addresses, read_latest

try:
    from kuavo_planar_task import sample_motion, apply_objects, load_json
except ModuleNotFoundError:
    from tools.kuavo_planar_task import sample_motion, apply_objects, load_json
try:
    from kuavo_scale_vision import write_lcd_texture
except ModuleNotFoundError:
    from tools.kuavo_scale_vision import write_lcd_texture

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = ROOT / "outputs/kuavo_sim/weighing/scene.xml"
DEFAULT_STATE = ROOT / "outputs/robot_control/kuavo5_weighing"
CAMERAS = {
    "head": "sim_head", "left": "sim_wrist_l", "right": "sim_wrist_r",
    "top": "weighing_top", "overview": "weighing_overview", "waist": "waist_camera",
}


class KuavoSimCamera:
    """Render RGB uint8 and optical-axis depth float32 in metres, without ROS."""

    def __init__(self, scene=DEFAULT_SCENE, state_dir=DEFAULT_STATE, width=848, height=480):
        self.scene = Path(scene).resolve()
        self.state_dir = Path(state_dir).resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.scene))
        self.data = mujoco.MjData(self.model)
        if self.model.nkey:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self.addresses = arm_addresses(self.model)
        self.latest_path = self.state_dir / "latest_state.json"
        latest = read_latest(self.latest_path)
        if latest is not None:
            self.data.qpos[self.addresses] = latest
        head=load_json(self.latest_path,{}).get('head_yaw_pitch_deg')
        if head:
            for name,angle in zip(['zhead_1_joint','zhead_2_joint'],head):self.data.qpos[self.model.joint(name).qposadr[0]]=np.radians(angle)
        mujoco.mj_forward(self.model, self.data)
        self.task_objects = load_json(self.latest_path, {}).get('task_objects', {})
        apply_objects(self.model, self.data, self.task_objects)
        apply_private_bin_poses(self.model,self.data,self.task_objects,self.state_dir)
        self.follower = HistoryFollower(self.state_dir / "history.jsonl")
        self.queue = deque()
        self.active = None
        self.width, self.height = width, height
        self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        self.lcd_value=None

    def refresh(self):
        """Follow controller history at wall-clock speed, as the arm GUI does."""
        self.queue.extend(self.follower.poll())
        now = time.monotonic()
        playback_progress = 1.0
        drop_bag = None
        if self.active is None and self.queue:
            record = self.queue.popleft()
            self.active = (now, self.data.qpos[self.addresses].copy(),
                           np.asarray(record["target_arm14_rad"]),
                           max(.15, float(record.get("actual_duration_s", 1))), record)
            self.task_objects = record.get('task_objects', self.task_objects)
        if self.active is not None:
            started, start, target, duration, record = self.active
            if record.get('event')=='drop':drop_bag=record.get('bag')
            x = min(1.0, (now-started)/duration)
            playback_progress = x
            self.data.qpos[self.addresses] = sample_motion(record, start, x)
            for name,angle in zip(['zhead_1_joint','zhead_2_joint'],record.get('head_yaw_pitch_deg',[])):
                self.data.qpos[self.model.joint(name).qposadr[0]]=np.radians(angle)
            if x >= 1:
                self.active = None
        elif not self.queue:
            latest = read_latest(self.latest_path)
            if latest is not None:
                self.data.qpos[self.addresses] = latest
            self.task_objects = load_json(self.latest_path, {}).get('task_objects', self.task_objects)
            for name,angle in zip(['zhead_1_joint','zhead_2_joint'],load_json(self.latest_path,{}).get('head_yaw_pitch_deg',[])):
                self.data.qpos[self.model.joint(name).qposadr[0]]=np.radians(angle)
        mujoco.mj_forward(self.model, self.data)
        apply_objects(self.model, self.data, self.task_objects, playback_progress, drop_bag)
        apply_private_bin_poses(self.model,self.data,self.task_objects,self.state_dir,playback_progress,drop_bag,
            self.active[4].get('action_id') if self.active else None)
        value=(self.active[4].get('scale_display_kg',0) if self.active else
               (self.lcd_value if self.queue and self.lcd_value is not None else
                load_json(self.latest_path,{}).get('scale_display_kg',0)))
        if self.lcd_value!=value:
            ident=write_lcd_texture(self.model,value)
            if ident is not None:
                self.renderer._gl_context.make_current()
                mujoco.mjr_uploadTexture(self.model,self.renderer._mjr_context,ident)
            self.lcd_value=value

    def capture_rgb(self,camera='head'):
        """RGB-only API for the task; no depth or camera truth is returned."""
        name=CAMERAS.get(camera,camera)
        ident=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_CAMERA,name)
        if ident<0:raise ValueError(f'Camera {name!r} is missing')
        self.renderer.disable_depth_rendering();self.renderer.update_scene(self.data,camera=ident)
        return self.renderer.render().copy()

    def capture(self, camera="head"):
        name = CAMERAS.get(camera, camera)
        camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        if camera_id < 0:
            raise ValueError(f"Camera {name!r} missing; regenerate scene with 'bash tools/kuavo_weighing.sh build'")
        self.renderer.disable_depth_rendering()
        self.renderer.update_scene(self.data, camera=camera_id)
        rgb = self.renderer.render().copy()
        self.renderer.enable_depth_rendering()
        try:
            depth = self.renderer.render().copy().astype(np.float32)
        finally:
            self.renderer.disable_depth_rendering()
        fovy = float(self.model.cam_fovy[camera_id])
        focal = self.height / (2*math.tan(math.radians(fovy)/2))
        k = [[focal, 0, (self.width-1)/2], [0, focal, (self.height-1)/2], [0,0,1]]
        # MuJoCo camera: +Y up, -Z forward. ROS optical: +Y down, +Z forward.
        world_from_optical = np.eye(4)
        world_from_optical[:3,:3] = self.data.cam_xmat[camera_id].reshape(3,3) @ np.diag([1,-1,-1])
        world_from_optical[:3,3] = self.data.cam_xpos[camera_id]
        far = float(self.model.vis.map.zfar*self.model.stat.extent)
        metadata = {
            "camera": name, "scene": str(self.scene), "state_dir": str(self.state_dir),
            "wall_time_unix_s": time.time(), "width": self.width, "height": self.height,
            "rgb_color_order": "RGB", "depth_dtype": "float32", "depth_units": "metre",
            "depth_definition": "Optical Z depth, not Euclidean ray distance",
            "near_m": float(self.model.vis.map.znear*self.model.stat.extent), "far_m": far,
            "background_note": "Far-plane depth can denote background; exclude values near far_m",
            "K": k, "D": [0,0,0,0,0], "vertical_fov_deg": fovy,
            "horizontal_fov_deg": math.degrees(2*math.atan(self.width/(2*focal))),
            "T_world_camera_optical": world_from_optical.tolist(),
            "arm14_rad": self.data.qpos[self.addresses].tolist(),
            "render_mode": "kinematic state/history playback, no sensor noise or ROS topics",
        }
        return rgb, depth, metadata

    def close(self):
        self.renderer.close()


def save_capture(camera, name, output):
    rgb, depth, metadata = camera.capture(name)
    output.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output/"rgb.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    np.save(output/"depth_m.npy", depth)
    valid = np.isfinite(depth) & (depth > 0) & (depth < metadata["far_m"]*.99)
    preview = np.zeros_like(depth, dtype=np.uint8)
    if valid.any():
        low, high = np.percentile(depth[valid], [2,98])
        if high > low:
            preview[valid] = np.clip((depth[valid]-low)/(high-low)*255,0,255).astype(np.uint8)
    color = cv2.applyColorMap(preview, cv2.COLORMAP_TURBO)
    color[~valid] = 0
    cv2.imwrite(str(output/"depth_preview.png"), color)
    (output/"camera.json").write_text(json.dumps(metadata, indent=2)+"\n")
    print(output, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--state-dir", type=Path, default=DEFAULT_STATE)
    parser.add_argument("--camera", choices=[*CAMERAS, "all"], default="head")
    parser.add_argument("--live", action="store_true", help="display RGB; s saves RGB-D, q exits")
    parser.add_argument("--list", action="store_true", help="list MJCF camera names")
    parser.add_argument("--output", type=Path, default=ROOT/"outputs/kuavo_sim/weighing/captures")
    parser.add_argument("--width", type=int, default=848)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=15)
    args = parser.parse_args()
    if min(args.width,args.height,args.fps) <= 0:
        parser.error("width, height and fps must be positive")
    camera = KuavoSimCamera(args.scene,args.state_dir,args.width,args.height)
    names = ["head","left","right"] if args.camera == "all" else [args.camera]
    window = None
    try:
        if args.list:
            for i in range(camera.model.ncam):
                print(camera.model.camera(i).name)
            return
        if not args.live:
            for name in names:
                save_capture(camera, name, args.output/name)
            return
        # letools ships headless OpenCV; Tk/Pillow provide the display window.
        import tkinter as tk
        from PIL import Image, ImageTk
        window = tk.Tk()
        window.title("Kuavo-5.2 simulated cameras | s save | q quit")
        panel = tk.Label(window)
        panel.pack()
        controls = {"quit": False, "save": False}
        window.protocol("WM_DELETE_WINDOW", lambda: controls.update(quit=True))

        def keypress(event):
            if event.keysym == "Escape" or event.char.lower() == "q":
                controls["quit"] = True
            elif event.char.lower() == "s":
                controls["save"] = True

        window.bind("<Key>", keypress)
        print(f"Following: {camera.state_dir}\nq/Esc: exit | s: save RGB-D", flush=True)
        while not controls["quit"]:
            started = time.monotonic()
            camera.refresh()
            panels = []
            for name in names:
                # Live view displays RGB only. Render depth only when saving;
                # avoid needless depth reads on limited WSL OpenGL drivers.
                rgb = camera.capture_rgb(name)
                cv2.putText(rgb,name,(12,25),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,0),2)
                panels.append(rgb)
            display = Image.fromarray(np.hstack(panels))
            max_width = round(window.winfo_screenwidth()*.9)
            if display.width > max_width:
                display = display.resize((max_width, round(display.height*max_width/display.width)))
            picture = ImageTk.PhotoImage(display)
            panel.configure(image=picture)
            panel.image = picture
            window.update()
            if controls["save"]:
                stamp = time.time_ns()
                for name in names:
                    save_capture(camera,name,args.output/str(stamp)/name)
                controls["save"] = False
            time.sleep(max(0,1/args.fps-(time.monotonic()-started)))
    finally:
        camera.close()
        if window is not None:
            window.destroy()


if __name__ == "__main__":
    main()
