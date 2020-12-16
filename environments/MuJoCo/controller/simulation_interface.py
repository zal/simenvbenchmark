#!/usr/bin/env python3
import os
import numpy as np
from os import path

# MuJoCo imports
import mujoco_py

DEFAULT_SIZE = 500


class MujocoEnv():
    """ Superclass for all MuJoCo environments. Adapted by DLR and ZAL.
        based on: https://github.com/openai/gym/blob/6df1b994bae791667a556e193d2a215b8a1e397a/gym/envs/mujoco/mujoco_env.py#L122
    """

    def __init__(self, model_path, frame_skip, render=False):
        if model_path.startswith("/"):
            fullpath = model_path
        else:
            fullpath = os.path.join(os.path.dirname(__file__), model_path)
        if not path.exists(fullpath):
            raise IOError("File %s does not exist" % fullpath)

        self.frame_skip = frame_skip
        self.render_option = render
        self.model = mujoco_py.load_model_from_path(fullpath)
        self.sim = mujoco_py.MjSim(self.model)
        self.data = self.sim.data

        if render:
            self.viewer = mujoco_py.MjViewer(self.sim)
            self._viewers = {}

        self.metadata = {
            'render.modes': ['human', 'rgb_array', 'depth_array'],
            'video.frames_per_second': int(np.round(1.0 / self.dt))
        }

        self.init_qpos = self.sim.data.qpos.ravel().copy()
        self.init_qvel = self.sim.data.qvel.ravel().copy()
        self.init_sim_state = self.sim.get_state()

    def reset_model(self):
        """
        Reset the robot degrees of freedom (qpos and qvel).
        Implement this in each subclass.
        """
        self.sim.set_state(self.init_sim_state)
        self.sim.forward()

    def reset(self):
        self.sim.reset()
        self.reset_model()
        # return ob

    def set_state(self, qpos, qvel):
        assert qpos.shape == (
            self.model.nq,) and qvel.shape == (self.model.nv,)
        old_state = self.sim.get_state()
        new_state = mujoco_py.MjSimState(old_state.time, qpos, qvel,
                                         old_state.act, old_state.udd_state)
        self.sim.set_state(new_state)
        self.sim.forward()

        if self.render_option:
            self.viewer.render()

    @property
    def dt(self):
        return self.model.opt.timestep * self.frame_skip

    def step(self, action=[]):
        if not action == []:
            self.sim.data.ctrl[:] = action

        self.sim.step()

    def do_simulation(self, action, n_frames):
        if len(action) > 0:
            self.sim.data.ctrl[:] = action

        for _ in range(n_frames):
            self.sim.step()
            if self.render_option:
                self.viewer.render()

    def render(self,
               mode='human',
               width=DEFAULT_SIZE,
               height=DEFAULT_SIZE,
               camera_id=None,
               camera_name=None):
        if mode == 'rgb_array' or mode == 'depth_array':
            if camera_id is not None and camera_name is not None:
                raise ValueError("Both `camera_id` and `camera_name` cannot be"
                                 " specified at the same time.")

            no_camera_specified = camera_name is None and camera_id is None
            if no_camera_specified:
                camera_name = 'track'

            if camera_id is None and camera_name in self.model._camera_name2id:
                camera_id = self.model.camera_name2id(camera_name)

            self._get_viewer(mode).render(width, height, camera_id=camera_id)

        if mode == 'rgb_array':
            # window size used for old mujoco-py:
            data = self._get_viewer(mode).read_pixels(
                width, height, depth=False)
            # original image is upside-down, so flip it
            return data[::-1, :, :]
        elif mode == 'depth_array':
            self._get_viewer(mode).render(width, height)
            # window size used for old mujoco-py:
            # Extract depth part of the read_pixels() tuple
            data = self._get_viewer(mode).read_pixels(
                width, height, depth=True)[1]
            # original image is upside-down, so flip it
            return data[::-1, :]
        elif mode == 'human':
            self._get_viewer(mode).render()

    def close(self):
        if self.viewer is not None:
            # self.viewer.finish()
            self.viewer = None
            self._viewers = {}

    def _get_viewer(self, mode):
        self.viewer = self._viewers.get(mode)
        if self.viewer is None:
            if mode == 'human':
                self.viewer = mujoco_py.MjViewer(self.sim)
            elif mode == 'rgb_array' or mode == 'depth_array':
                self.viewer = mujoco_py.MjRenderContextOffscreen(self.sim, -1)

            self.viewer_setup()
            self._viewers[mode] = self.viewer
        return self.viewer

    def get_body_com(self, body_name):
        return self.data.get_body_xpos(body_name)

    def state_vector(self):
        return np.concatenate([
            self.sim.data.qpos.flat,
            self.sim.data.qvel.flat
        ])
