"""MuJoCo backend for local simulation with IK control."""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np
from PIL import Image
from transforms3d import quaternions

from robot_backend import BackendStatus, RobotBackend


try:
    import mujoco
except Exception:  # pragma: no cover
    mujoco = None


_FALLBACK_MJCF = """
<mujoco model="teleop_fallback_arm">
  <compiler angle="radian" coordinate="local" inertiafromgeom="true"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="base" pos="0 0 0">
      <geom type="box" size="0.05 0.05 0.02" rgba="0.3 0.3 0.3 1"/>
      <body name="link1" pos="0 0 0.04">
        <joint name="j1" type="hinge" axis="0 0 1" limited="true" range="-3.14 3.14"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.25" size="0.02" rgba="0.2 0.6 0.9 1"/>
        <body name="link2" pos="0 0 0.25">
          <joint name="j2" type="hinge" axis="0 1 0" limited="true" range="-2.5 2.5"/>
          <geom type="capsule" fromto="0 0 0 0 0 0.22" size="0.018" rgba="0.2 0.9 0.6 1"/>
          <body name="link3" pos="0 0 0.22">
            <joint name="j3" type="hinge" axis="0 1 0" limited="true" range="-2.5 2.5"/>
            <geom type="capsule" fromto="0 0 0 0 0 0.18" size="0.016" rgba="0.9 0.6 0.2 1"/>
            <site name="ee" pos="0 0 0.18" size="0.01" rgba="1 0.2 0.2 1"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
""".strip()


def _mat_to_quat_wxyz(mat3: np.ndarray) -> np.ndarray:
    q = quaternions.mat2quat(mat3)
    return np.array([q[0], q[1], q[2], q[3]], dtype=float)


def _quat_wxyz_to_mat(q: np.ndarray) -> np.ndarray:
    return quaternions.quat2mat([float(q[0]), float(q[1]), float(q[2]), float(q[3])])


def _so3_log_small_angle(r_err: np.ndarray) -> np.ndarray:
    return 0.5 * np.array(
        [
            r_err[2, 1] - r_err[1, 2],
            r_err[0, 2] - r_err[2, 0],
            r_err[1, 0] - r_err[0, 1],
        ],
        dtype=float,
    )


@dataclass
class MujocoBackendConfig:
    xml_path: Optional[str] = None
    ee_site: Optional[str] = None
    camera: Optional[str] = None
    ik_damping: float = 0.05
    ik_max_iters: int = 15 # Reduced from 25 for lower latency
    ik_pos_weight: float = 1.0
    ik_rot_weight: float = 0.3
    max_qpos_step: float = 0.15 # Slightly increased from 0.12 for faster response


class MujocoRobotBackend(RobotBackend):
    def __init__(
        self,
        name: str = "mujoco_robot",
        xml_path: Optional[str] = None,
        ee_site: Optional[str] = None,
        camera: Optional[str] = None,
        ik_damping: float = 0.05,
        ik_max_iters: int = 15,
        ik_pos_weight: float = 1.0,
        ik_rot_weight: float = 0.3,
        max_qpos_step: float = 0.15,
    ):
        super().__init__(name)
        self._cfg = MujocoBackendConfig(
            xml_path=xml_path,
            ee_site=ee_site,
            camera=camera,
            ik_damping=ik_damping,
            ik_max_iters=ik_max_iters,
            ik_pos_weight=ik_pos_weight,
            ik_rot_weight=ik_rot_weight,
            max_qpos_step=max_qpos_step,
        )

        self._model: Any = None
        self._data: Any = None
        self._ee_site_id: Optional[int] = None
        self._default_camera: int | str = "world_cam"
        self._last_target_pos = np.array([0.0, 0.0, 0.5], dtype=float)
        self._last_target_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self._command_count = 0
        self._renderer = None

    def connect(self) -> bool:
        if mujoco is None:
            self.status = BackendStatus.ERROR
            raise RuntimeError("MuJoCo Python package is not installed. `pip install mujoco`.")

        self.status = BackendStatus.CONNECTING
        xml_path = self._cfg.xml_path or os.getenv("TELEOP_MUJOCO_XML")
        
        # Default to teleop_scene.xml if available and not specified
        if not xml_path:
            potential_scene = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "robots", "teleop_scene.xml")
            if os.path.exists(potential_scene):
                xml_path = potential_scene
                print(f"[MuJoCo] Using default scene: {xml_path}")

        ee_site = self._cfg.ee_site or os.getenv("TELEOP_MUJOCO_EE_SITE")
        preferred_camera = self._cfg.camera or os.getenv("TELEOP_MUJOCO_CAMERA")
        if not ee_site:
            ee_site = "ee"

        try:
            if xml_path:
                self._model = mujoco.MjModel.from_xml_path(xml_path)
            else:
                self._model = mujoco.MjModel.from_xml_string(_FALLBACK_MJCF)
            self._data = mujoco.MjData(self._model)
            mujoco.mj_forward(self._model, self._data)
        except Exception as e:
            self.status = BackendStatus.ERROR
            raise RuntimeError(f"Failed to load MuJoCo model: {e}")

        self._ee_site_id = self._resolve_ee_site_id(preferred=ee_site)
        if self._ee_site_id is None:
            self.status = BackendStatus.ERROR
            raise RuntimeError("Could not resolve end-effector site. Set TELEOP_MUJOCO_EE_SITE.")

        self._default_camera = self._resolve_camera(preferred=preferred_camera)

        self.status = BackendStatus.CONNECTED
        self.last_update_time = time.time()
        return True

    def disconnect(self):
        self.status = BackendStatus.DISCONNECTED
        if self._renderer is not None:
            try:
                if hasattr(self._renderer, 'close'):
                    self._renderer.close()
            except Exception as e:
                print(f"[MuJoCo] Warning: Error closing renderer: {e}")
            finally:
                self._renderer = None
        self._model = None
        self._data = None
        self._ee_site_id = None

    def send_target_pose(
        self,
        position: np.ndarray,
        orientation: np.ndarray,
        velocity_limit: float = 0.1,
        gripper_state: float = -1.0,
    ) -> bool:
        if not self.is_connected():
            return False

        with self.update_lock:
            self._last_target_pos = np.array(position, dtype=float)
            self._last_target_quat = np.array(orientation, dtype=float)
            self._solve_ik(target_pos=self._last_target_pos, target_quat=self._last_target_quat, velocity_limit=velocity_limit)
            
            # Handle gripper for follower
            if gripper_state >= 0:
                self._set_gripper(gripper_state)
                
            self._command_count += 1
            self.last_update_time = time.time()
        return True

    def send_joint_positions(self, joint_positions: np.ndarray) -> bool:
        """Update leader robot joints from teleop command"""
        if not self.is_connected() or self._model is None or self._data is None:
            return False

        with self.update_lock:
            # Map joints to leader_xxx joints
            # Expected order: pan, lift, elbow, wrist_flex, wrist_roll, gripper
            joint_names = [
                "leader_shoulder_pan", "leader_shoulder_lift", "leader_elbow_flex",
                "leader_wrist_flex", "leader_wrist_roll", "leader_gripper"
            ]
            
            try:
                for i, name in enumerate(joint_names):
                    if i >= len(joint_positions):
                        break
                    
                    try:
                        jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
                        if jid != -1:
                            qpos_addr = self._model.jnt_qposadr[jid]
                            # Assuming 1-DOF joints
                            # Convert to float and set
                            val = float(joint_positions[i])
                            # Handle gripper mapping if needed (usually 0-100 or rad)
                            # Leader teleop sends raw values (usually radians or degrees depending on config)
                            # Assuming radians here as MuJoCo uses radians
                            self._data.qpos[qpos_addr] = val
                    except Exception:
                        pass
                
                mujoco.mj_forward(self._model, self._data)
                return True
            except Exception as e:
                print(f"[MuJoCo] Error setting joints: {e}")
                return False

    def _set_gripper(self, state: float) -> None:
        """Set follower gripper state (0.0 to 1.0)"""
        if self._model is None: return
        try:
            # Try finding 'gripper' (SO-ARM101) or 'Left_1_Joint' (RM75-B with gripper)
            target_joint = "gripper"
            range_min, range_max = -0.17, 1.74
            
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, target_joint)
            if jid == -1:
                target_joint = "Left_1_Joint"
                jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, target_joint)
                # RM75-B gripper range: -0.91 to 0
                # 0 is usually open, -0.91 is closed (fingers move inward)
                # We need to check visual, but let's assume 0 (open) -> 1 (closed) maps to 0 -> -0.91
                range_min, range_max = 0.0, -0.91

            if jid != -1:
                qpos_addr = self._model.jnt_qposadr[jid]
                # Map 0 (open) -> 1 (closed) to joint limits
                # Note: 'state' from controller is 0 (open) to 1 (closed) usually
                val = range_min + state * (range_max - range_min)
                self._data.qpos[qpos_addr] = val
        except:
            pass

    def get_current_pose(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if not self.is_connected():
            return None, None
        with self.update_lock:
            pos, quat = self._get_site_pose(self._ee_site_id)
            return pos.copy(), quat.copy()

    def get_joint_positions(self) -> Optional[np.ndarray]:
        """Get current joint positions (radians)"""
        if not self.is_connected() or self._model is None or self._data is None:
            return None
        
        with self.update_lock:
            # Return all joint positions
            qpos = []
            for i in range(self._model.njnt):
                addr = self._model.jnt_qposadr[i]
                qpos.append(self._data.qpos[addr])
            return np.array(qpos)

    def render(self, width: int = 960, height: int = 540, camera: Optional[str] = None) -> Optional[bytes]:
        """Render a frame from the simulation.
        
        On macOS, the standard Renderer may fail due to OpenGL context issues.
        This implementation falls back to a placeholder if rendering fails.
        """
        if not self.is_connected() or self._model is None or self._data is None:
            return None

        camera_arg: int | str
        if camera is None:
            camera_arg = self._default_camera
        else:
            camera_arg = self._resolve_camera(preferred=camera)
        
        try:
            with self.update_lock:
                # Try to create renderer if it doesn't exist
                if self._renderer is None:
                    try:
                        self._renderer = mujoco.Renderer(self._model, height, width)
                    except Exception as e:
                        print(f"[MuJoCo] Renderer creation failed: {e}")
                        print("[MuJoCo] Falling back to placeholder rendering")
                        return self._render_placeholder(width, height)
                
                # Try to render
                try:
                    self._renderer.update_scene(self._data, camera=camera_arg)
                    img_arr = self._renderer.render()
                except Exception as e:
                    print(f"[MuJoCo] Rendering failed: {e}")
                    print("[MuJoCo] Attempting to recreate renderer...")
                    # Try to close and recreate renderer
                    try:
                        if hasattr(self._renderer, 'close'):
                            self._renderer.close()
                    except:
                        pass
                    self._renderer = None
                    return self._render_placeholder(width, height)
            
            # Convert to JPEG
            img = Image.fromarray(img_arr)
            from io import BytesIO
            buf = BytesIO()
            img.save(buf, format="JPEG", quality=75)
            return buf.getvalue()
            
        except Exception as e:
            print(f"[MuJoCo] Unexpected rendering error: {e}")
            return self._render_placeholder(width, height)
    
    def _render_placeholder(self, width: int, height: int) -> bytes:
        """Generate a placeholder frame with simulation state info."""
        from io import BytesIO
        import numpy as np
        from PIL import Image, ImageDraw, ImageFont
        
        # Create dark background
        img = Image.new('RGB', (width, height), color=(15, 20, 30))
        draw = ImageDraw.Draw(img)
        
        # Get current pose
        pos, quat = self.get_current_pose()
        
        # Draw info text
        try:
            font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 24)
            small_font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 16)
        except:
            font = ImageFont.load_default()
            small_font = font
        
        # Title
        draw.text((20, 20), "MuJoCo Simulation (Headless Mode)", fill=(200, 220, 255), font=font)
        
        # Status
        y_offset = 60
        draw.text((20, y_offset), "Status: Running", fill=(100, 255, 150), font=small_font)
        
        # Position
        y_offset += 30
        if pos is not None:
            draw.text((20, y_offset), f"Position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]", 
                     fill=(150, 200, 255), font=small_font)
        
        # Orientation
        y_offset += 25
        if quat is not None:
            draw.text((20, y_offset), f"Orientation: [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]", 
                     fill=(150, 200, 255), font=small_font)
        
        # Joint positions
        y_offset += 35
        draw.text((20, y_offset), "Joint Positions:", fill=(200, 200, 200), font=small_font)
        y_offset += 25
        if self._data is not None and hasattr(self._data, 'qpos'):
            for i, q in enumerate(self._data.qpos[:6]):  # Show first 6 joints
                draw.text((40, y_offset + i*20), f"Joint {i+1}: {q:.3f} rad", 
                         fill=(180, 180, 180), font=small_font)
        
        # Note about rendering
        y_offset = height - 100
        draw.text((20, y_offset), "⚠ Note: OpenGL rendering unavailable on this system", 
                 fill=(255, 200, 100), font=small_font)
        y_offset += 25
        draw.text((20, y_offset), "Simulation is running correctly - visualization only affected", 
                 fill=(200, 200, 200), font=small_font)
        y_offset += 25
        draw.text((20, y_offset), "Try: mjpython or Linux/Windows for 3D visualization", 
                 fill=(150, 150, 150), font=small_font)
        
        # Convert to JPEG
        buf = BytesIO()
        img.save(buf, format="JPEG", quality=85)
        return buf.getvalue()

    def get_joint_positions(self) -> Optional[np.ndarray]:
        """
        Get current joint positions (radians)
        
        Returns:
            joints: Array of joint positions or None if not available
        """
        if not self.is_connected() or self._data is None:
            return None
        with self.update_lock:
            return np.array(self._data.qpos, dtype=float)

    def get_status(self) -> Dict[str, Any]:
        pos, quat = (None, None)
        if self.is_connected() and self._ee_site_id is not None:
            pos, quat = self.get_current_pose()
        return {
            "name": self.name,
            "status": self.status.value,
            "backend": "mujoco",
            "command_count": self._command_count,
            "ee_site": int(self._ee_site_id) if self._ee_site_id is not None else None,
            "camera": str(self._default_camera),
            "current_position": pos.tolist() if pos is not None else None,
            "current_orientation": quat.tolist() if quat is not None else None,
            "target_position": self._last_target_pos.tolist(),
            "target_orientation": self._last_target_quat.tolist(),
            "last_update": self.last_update_time,
        }

    def _resolve_ee_site_id(self, preferred: str) -> Optional[int]:
        if self._model is None:
            return None

        def name2id(name: str) -> int:
            return mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SITE, name)

        candidates = [preferred, "gripperframe", "end_effector", "gripper", "tcp", "tool0", "ee"]
        for c in candidates:
            try:
                idx = name2id(c)
            except Exception:
                continue
            if idx != -1:
                print(f"[MuJoCo] Resolved EE site to: {c} (ID {idx})")
                return int(idx)

        if getattr(self._model, "nsite", 0) > 0:
            print(f"[MuJoCo] Warning: Could not resolve EE site '{preferred}'. Defaulting to site 0.")
            return 0
        return None

    def _resolve_camera(self, preferred: Optional[str]) -> int | str:
        if self._model is None:
            return "world_cam"

        def try_name(name: str) -> Optional[str]:
            try:
                idx = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, name)
            except Exception:
                return None
            if int(idx) != -1:
                return name
            return None

        if preferred:
            resolved = try_name(preferred)
            if resolved is not None:
                return resolved

        resolved = try_name("world_cam")
        if resolved is not None:
            return resolved

        ncam = int(getattr(self._model, "ncam", 0) or 0)
        if ncam > 0:
            return 0

        return -1

    def _get_site_pose(self, site_id: int) -> Tuple[np.ndarray, np.ndarray]:
        pos = np.array(self._data.site_xpos[site_id], dtype=float)
        mat = np.array(self._data.site_xmat[site_id], dtype=float).reshape(3, 3)
        quat = _mat_to_quat_wxyz(mat)
        return pos, quat

    def _solve_ik(self, target_pos: np.ndarray, target_quat: np.ndarray, velocity_limit: float) -> None:
        if self._ee_site_id is None:
            return

        model = self._model
        data = self._data
        site_id = int(self._ee_site_id)

        nv = int(model.nv)
        jacp = np.zeros((3, nv), dtype=float)
        jacr = np.zeros((3, nv), dtype=float)

        max_step = float(self._cfg.max_qpos_step) * float(max(0.05, min(velocity_limit, 1.0)))
        damping = float(self._cfg.ik_damping)

        w_pos = float(self._cfg.ik_pos_weight)
        w_rot = float(self._cfg.ik_rot_weight)

        for _ in range(int(self._cfg.ik_max_iters)):
            mujoco.mj_forward(model, data)
            cur_pos, cur_quat = self._get_site_pose(site_id)

            pos_err = (target_pos - cur_pos) * w_pos

            r_cur = _quat_wxyz_to_mat(cur_quat)
            r_tgt = _quat_wxyz_to_mat(target_quat)
            r_err = r_tgt @ r_cur.T
            rot_err = _so3_log_small_angle(r_err) * w_rot

            err = np.concatenate([pos_err, rot_err], axis=0)
            if float(np.linalg.norm(pos_err)) < 1e-4 and float(np.linalg.norm(rot_err)) < 2e-3:
                break

            mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
            j = np.vstack([jacp * w_pos, jacr * w_rot])

            a = j @ j.T + (damping * damping) * np.eye(6, dtype=float)
            dq = j.T @ np.linalg.solve(a, err)

            dq = np.clip(dq, -max_step, max_step)
            mujoco.mj_integratePos(model, data.qpos, dq, 1.0)
            self._clamp_joint_limits()

        mujoco.mj_forward(model, data)

    def _clamp_joint_limits(self) -> None:
        model = self._model
        data = self._data
        if model is None or data is None:
            return

        for j in range(int(model.njnt)):
            if int(model.jnt_limited[j]) != 1:
                continue
            adr = int(model.jnt_qposadr[j])
            lo = float(model.jnt_range[j, 0])
            hi = float(model.jnt_range[j, 1])
            data.qpos[adr] = float(np.clip(data.qpos[adr], lo, hi))
