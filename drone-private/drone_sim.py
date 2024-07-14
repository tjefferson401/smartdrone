import mujoco
import numpy as np
import sys
import time
import mujoco.viewer
import os
import itertools
import math
from scipy.spatial.transform import Rotation as R

class Drone():
    def __init__(self, world_xml_filename, **kwargs):

        self.model = mujoco.MjModel.from_xml_path((world_xml_filename))

        # mj_model.opt.solver = mujoco.mjtSolver.mjSOL_CG
        # mj_model.opt.iterations = 6
        # mj_model.opt.ls_iterations = 6
                
        self.data = mujoco.MjData(self.model)

        # physics_steps_per_control_step = 5
        # kwargs['n_frames'] = kwargs.get(
            # 'n_frames', physics_steps_per_control_step)
        self._physics_steps_per_control_step = 10
        self.height = 240
        self.width = 240
        self.frames, self.depth_frames = [], []
        self.options = mujoco.MjvOption()
        mujoco.mjv_defaultOption(self.options)

        self.viewer = mujoco.Renderer(self.model, self.height, self.width)
        
        self.cam_top_viewer = mujoco.Renderer(self.model, self.height, self.width); self.cam_top_viewer.enable_depth_rendering()
        self.cam_bot_viewer = mujoco.Renderer(self.model, self.height, self.width); self.cam_bot_viewer.enable_depth_rendering()
        self.cam_right_viewer = mujoco.Renderer(self.model, self.height, self.width); self.cam_right_viewer.enable_depth_rendering()
        self.cam_left_viewer = mujoco.Renderer(self.model, self.height, self.width); self.cam_left_viewer.enable_depth_rendering()
        self.cam_front_viewer = mujoco.Renderer(self.model, self.height, self.width); self.cam_front_viewer.enable_depth_rendering()
        self.cam_back_viewer = mujoco.Renderer(self.model, self.height, self.width); self.cam_back_viewer.enable_depth_rendering()
        
        self.seg_cam_top_viewer = mujoco.Renderer(self.model, self.height, self.width)
        self.seg_cam_top_viewer.enable_segmentation_rendering()

        self.seg_cam_bot_viewer = mujoco.Renderer(self.model, self.height, self.width)
        self.seg_cam_bot_viewer.enable_segmentation_rendering()

        self.seg_cam_right_viewer = mujoco.Renderer(self.model, self.height, self.width)
        self.seg_cam_right_viewer.enable_segmentation_rendering()

        self.seg_cam_left_viewer = mujoco.Renderer(self.model, self.height, self.width)
        self.seg_cam_left_viewer.enable_segmentation_rendering()

        self.seg_cam_front_viewer = mujoco.Renderer(self.model, self.height, self.width)
        self.seg_cam_front_viewer.enable_segmentation_rendering()

        self.seg_cam_back_viewer = mujoco.Renderer(self.model, self.height, self.width)
        self.seg_cam_back_viewer.enable_segmentation_rendering()

        self.depth_viewers = [
            ["cam_top", self.cam_top_viewer], ["cam_bot", self.cam_bot_viewer], 
            ["cam_right", self.cam_right_viewer], ["cam_left", self.cam_left_viewer], 
            ["cam_front", self.cam_front_viewer], ["cam_back", self.cam_back_viewer]
        ]

        self.seg_viewers = [
            ["cam_top", self.seg_cam_top_viewer], ["cam_bot", self.seg_cam_bot_viewer], 
            ["cam_right", self.seg_cam_right_viewer], ["cam_left", self.seg_cam_left_viewer], 
            ["cam_front", self.seg_cam_front_viewer], ["cam_back", self.seg_cam_back_viewer]
        ]

        self.initial_qpos = np.copy(self.data.qpos)
        self.num_drones = 5
        self.form_width = .05
        self.formation_dict = {
            0: np.array([[1, 0, 0, -self.form_width],[0,1,0,self.form_width],[0,0,1,0],[0,0,0,1]]),
            1: np.array([[1, 0, 0, self.form_width],[0,1,0, self.form_width],[0,0,1,0],[0,0,0,1]]),
            2: np.array([[1, 0, 0, -self.form_width],[0,1,0, -self.form_width],[0,0,1,0],[0,0,0,1]]),
            3: np.array([[1, 0, 0, self.form_width],[0,1,0, -self.form_width],[0,0,1,0],[0,0,0,1]]),
            4: np.array([[1, 0, 0, 0],[0,1,0, 0],[0,0,1,0],[0,0,0,1]]),
        }
        self.pose_formation_dict = {
            0: np.array([-self.form_width, self.form_width, 0, 1]),
            1: np.array([self.form_width, self.form_width, 0, 1]),
            2: np.array([-self.form_width, -self.form_width, 0, 1]),
            3: np.array([self.form_width, -self.form_width, 0, 1]),
            4: np.array([0, 0, 0, 1]),
        }
        self.leader = 4

    def get_drone_pos(self,):
        self.drones = {}
        for i in range(self.num_drones):
            self.drones[i] = self.data.qpos[i*6:i*6 + 3]
    
    def step(self, leader_action, render=False):
        leader_pos = leader_action[:3]
        leader_rot = leader_action[3:6]  # rx, ry, rz in radians
        world_T_leader = np.eye(4)
        world_T_leader[:3, :3] = R.from_rotvec(leader_rot).as_matrix()
        world_T_leader[:3, 3] = leader_pos
        # Set the followers based on the leader
        self.get_drone_pos()
        for i in range(self.num_drones):
            follower_pos = (world_T_leader @ self.pose_formation_dict[i])[:3]
            self.data.ctrl[i*6:i*6 + 3] = follower_pos # np.concatenate((, rotation_vector))

        mujoco.mj_forward(self.model, self.data)

        mujoco.mj_step(self.model, self.data, nstep=self._physics_steps_per_control_step)
        
        leader_pos = self.data.qpos[4*6:4*6 + 3]
        for i in range(self.num_drones):
            follower_pos = self.data.qpos[i*6:i*6 + 3]
            diff = np.linalg.norm(leader_pos - follower_pos)

        if render:
            self.render()        

    def reset(self):
        """Resets the environment to an initial state."""
        self.data.qpos = self.initial_qpos
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)
        self.render()

    def _get_obs(self):
        """Observes the spheres' positions, velocities, and angles."""
        position = self.data.qpos
        return position

    def render(self, camera_type="user"):      
        start_render = time.time()  
        self.viewer.update_scene(self.data, "cam_front")
        
        frame = self.viewer.render()
        self.frames.append(frame)

    def print_body_positions(self):
        print(self.data.xpos)

if __name__ == "__main__":
    env = Drone('drone_swarm.xml')
    env.reset()
    m = mujoco.MjModel.from_xml_path('drone_swarm.xml')
    d = env.data

    with mujoco.viewer.launch_passive(m, d) as viewer:
        viewer.cam.distance = 15 * viewer.cam.distance  # Increase the camera distance

        # Enable wireframe rendering of the entire scene.
        # viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_WIREFRAME] = 1
        viewer.sync()
        start_time = time.time() * 1000
        x = 0
        while viewer.is_running():
            # Sets the leader pos: x,y,z and rotation x,y,z in radians
            # if round(time.time() * 1000 - start_time) % 25 == 0:
            x += 0.01
            print(x)
            action = np.array([0, 0, x, 0, 0, 0])
            env.step(action, render=True)
          
            viewer.sync()
         
            # env.reset()
            # input("hi")
    
    
