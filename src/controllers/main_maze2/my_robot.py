"""Robot hardware, odometry and low-level motion layer.

``MyRobot`` is a thin, **single-threaded** wrapper around the Webots Rosbot.
It is responsible only for:

  * device setup and camera intrinsics,
  * pose estimation (wheel-encoder odometry fused with the drift-free compass
    heading) — this replaces the forbidden Supervisor ground-truth,
  * world <-> grid coordinate conversions,
  * primitive motion (velocity, timed turns, backup) and a DWA local step,
  * lidar -> occupancy-grid updates and simple map stamping helpers,
  * visualisation state hand-off to the (read-only) pygame thread.

All higher-level behaviour (perception, planning, navigation, mission logic)
lives in the perception / navigation / mission modules.  Crucially, there are
**no background threads touching the Webots API**: every sensor read and motor
command happens on the one thread that also calls ``step()``.
"""

import numpy as np

from controller import Robot

import utils
from setup import setup_robot
from map import GridMap
from CONSTANTS import (
    TIME_STEP, WHEEL_RADIUS, AXLE_LENGTH, MAX_VELOCITY, RESOLUTION,
    GREEN_CARPET, PATH_FOLLOWING_TARGET_REACH_DISTANCE,
    DWA_VELOCITY_SAMPLES, DWA_ANGULAR_SAMPLES,
    DWA_HEADING_WEIGHT, DWA_DISTANCE_WEIGHT, DWA_SPEED_WEIGHT,
    DWA_PREDICTION_DISTANCE_THRESHOLD,
    CAMERA_HEIGHT_M, LIDAR_MAX_RANGE_M,
)


class MyRobot(Robot):
    """Single-threaded Rosbot controller: devices, pose, motion and mapping I/O."""

    def __init__(self):
        """Set up devices, camera intrinsics, pose state and the grid map."""
        super().__init__()
        (self.motors, self.wheel_sensors, self.imu, self.camera_rgb,
         self.camera_depth, self.lidar, self.distance_sensors) = setup_robot(self)

        self.time_step = TIME_STEP
        self.wheel_radius = WHEEL_RADIUS
        self.axle_length = AXLE_LENGTH

        # --- Occupancy grid (SLAM-lite: mapping on odometry+compass pose) ---
        self.map_object = GridMap(robot=self)
        self.grid_map = self.map_object.grid_map

        # --- Pose state (origin = start pose; heading from compass) ---
        self._odom_initialized = False
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._prev_left_pos = 0.0
        self._prev_right_pos = 0.0
        self._heading_rad = 0.0

        # --- Mission state ---
        self.blue_world = None     # np.array([x, y]) world position of blue pillar
        self.yellow_world = None   # np.array([x, y]) world position of yellow pillar
        self.start_point = None    # blue pillar map cell (kept for vis/compat)
        self.end_point = None      # yellow pillar map cell (kept for vis/compat)
        self.last_turn = 'right'

        # --- Camera intrinsics / green-projection extrinsics ---
        self._init_camera_model()
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting and adapted to the teams use case.

    # ================================================================== #
    # Camera model                                                        #
    # ================================================================== #
    def _init_camera_model(self):
        """Compute pinhole intrinsics from the RGB camera and set extrinsics."""
        try:
            self.cam_width = self.camera_rgb.getWidth()
            self.cam_height = self.camera_rgb.getHeight()
            fov = self.camera_rgb.getFov()
            self.fx = self.cam_width / (2.0 * np.tan(fov / 2.0))
            self.fy = self.fx
            self.cx = self.cam_width / 2.0
            self.cy = self.cam_height / 2.0
        except Exception:
            self.cam_width, self.cam_height = 320, 240
            self.fx = self.fy = 240.0
            self.cx, self.cy = 160.0, 120.0
        # Depth-camera intrinsics (may differ from the RGB camera). Used by the
        # depth obstacle layer that catches flat-on-floor and floating walls.
        try:
            self.cam_d_width = self.camera_depth.getWidth()
            self.cam_d_height = self.camera_depth.getHeight()
            fov_d = self.camera_depth.getFov()
            self.fx_d = self.cam_d_width / (2.0 * np.tan(fov_d / 2.0))
            self.fy_d = self.fx_d
            self.cx_d = self.cam_d_width / 2.0
            self.cy_d = self.cam_d_height / 2.0
        except Exception:
            self.cam_d_width, self.cam_d_height = self.cam_width, self.cam_height
            self.fx_d = self.fy_d = self.fx
            self.cx_d, self.cy_d = self.cx, self.cy
        # Ground-projection extrinsics for green detection (Rosbot geometry).
        self.camera_height_m = CAMERA_HEIGHT_M
        self.X_offset = 0.03
        self.Y_offset = 0.0
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting and adapted to the teams use case.

    # ================================================================== #
    # Stepping & odometry                                                 #
    # ================================================================== #
    def step(self, ms=None):
        """Advance the simulation one step and refresh odometry.

        This override is the single place the controller blocks on Webots.
        Returns the underlying ``Robot.step`` result (-1 when Webots quits).
        """
        if ms is None:
            ms = self.time_step
        result = super().step(int(ms))
        self._update_odometry()
        return result
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def _update_odometry(self):
        """Integrate encoder deltas and refresh heading from the compass.

        Position is dead-reckoned from the averaged wheel encoders; heading is
        the absolute (drift-free) compass bearing.  Wheel displacement is
        integrated at the midpoint heading to reduce turning error.  Odometry
        starts only once both encoders and the compass report valid values.
        """
        try:
            l = (self.wheel_sensors['fl'].getValue() + self.wheel_sensors['rl'].getValue()) / 2.0
            r = (self.wheel_sensors['fr'].getValue() + self.wheel_sensors['rr'].getValue()) / 2.0
        except Exception:
            return
        if np.isnan(l) or np.isnan(r):
            return

        prev_heading = self._heading_rad
        compass_valid = False
        try:
            c = self.imu['compass'].getValues()
            if not (np.isnan(c[0]) or np.isnan(c[1])):
                self._heading_rad = -np.arctan2(c[1], c[0])
                compass_valid = True
        except Exception:
            pass

        if not self._odom_initialized:
            if not compass_valid:
                return
            self._prev_left_pos, self._prev_right_pos = l, r
            self._odom_initialized = True
            return

        dl = (l - self._prev_left_pos) * self.wheel_radius
        dr = (r - self._prev_right_pos) * self.wheel_radius
        self._prev_left_pos, self._prev_right_pos = l, r
        d = (dl + dr) / 2.0

        cs = (np.cos(prev_heading) + np.cos(self._heading_rad)) * 0.5
        sn = (np.sin(prev_heading) + np.sin(self._heading_rad)) * 0.5
        mid = np.arctan2(sn, cs)
        self._odom_x += d * np.cos(mid)
        self._odom_y += d * np.sin(mid)
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    # ================================================================== #
    # Pose & coordinate conversions                                       #
    # ================================================================== #
    def get_heading(self, type='rad'):
        """Return the robot heading in radians (default) or degrees."""
        return self._heading_rad if type == 'rad' else np.degrees(self._heading_rad)
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def get_position(self):
        """Return the odometry world position as np.array([x, y])."""
        return np.array([self._odom_x, self._odom_y])
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def get_map_position(self):
        """Return the robot's current grid cell as np.array([mx, my])."""
        mx = self.map_object.map_size // 2 + int(self._odom_x / RESOLUTION)
        my = self.map_object.map_size // 2 - int(np.ceil(self._odom_y / RESOLUTION))
        return np.array([mx, my])
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def get_map_distance(self, map_target):
        """Return the pixel distance from the robot to a grid cell."""
        return float(np.linalg.norm(self.get_map_position() - np.array(map_target)))
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def convert_to_map_coordinates(self, x, y):
        """Convert world (x, y) metres to integer grid (mx, my)."""
        mx = self.map_object.map_size // 2 + int(x / RESOLUTION)
        my = self.map_object.map_size // 2 - int(np.ceil(y / RESOLUTION))
        return int(mx), int(my)
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting and adapted to the teams use case.

    def convert_to_world_coordinates(self, map_x, map_y):
        """Convert grid (mx, my) to world (x, y) metres at the cell centre."""
        x = (map_x - self.map_object.map_size // 2) * RESOLUTION
        y = (self.map_object.map_size // 2 - map_y) * RESOLUTION
        return float(x), float(y)
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting and adapted to the teams use case.

    # ================================================================== #
    # Range sensors & lidar                                               #
    # ================================================================== #
    def get_distances(self):
        """Return the four range-sensor readings [fl, rl, fr, rr] in metres."""
        return [s.getValue() for s in self.distance_sensors]
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def get_pointcloud_2d(self):
        """Return finite, near lidar points as an (N, 2) array in the robot frame.

        Returns are gated to ``LIDAR_MAX_RANGE_M`` because far lidar data is
        inaccurate and pollutes the occupancy grid; the threshold is a tunable
        constant so it can be adjusted manually later.
        """
        if self.lidar is None:
            return np.empty((0, 2))
        pts = self.lidar.getPointCloud()
        if not pts:
            return np.empty((0, 2))
        arr = np.array([[p.x, p.y] for p in pts])
        arr = arr[~np.isinf(arr).any(axis=1)]
        if arr.shape[0] == 0:
            return arr
        # ACCURACY GATE: ignore returns farther than the configurable max range.
        rng = np.linalg.norm(arr, axis=1)
        return arr[rng <= LIDAR_MAX_RANGE_M]
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def transform_points_to_world(self, points_local):
        """Rotate+translate (N, 2) robot-frame points into the world frame."""
        if points_local.size == 0:
            return points_local
        theta = self._heading_rad
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta),  np.cos(theta)]])
        return points_local @ R.T + np.array([self._odom_x, self._odom_y])
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting and adapted to the teams use case.

    def get_pointcloud_world_coordinates(self):
        """Return the current lidar scan transformed into world coordinates."""
        return self.transform_points_to_world(self.get_pointcloud_2d())
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def lidar_update_map(self):
        """Fold the current lidar scan into the occupancy grid."""
        world_pts = self.get_pointcloud_world_coordinates()
        if world_pts.size == 0:
            return
        self.map_object.lidar_update_grid_map(self.get_map_position(), world_pts)
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def there_is_obstacle(self, map_target):
        """True if the given grid cell is a lethal (obstacle/green/closed) cell."""
        return self.map_object.there_is_obstacle(map_target)
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def robot_on_ground(self, max_tan_pitch=0.20):
        """True if the robot is roughly level (safe to trust the lidar map).

        Estimates pitch/roll magnitude from the accelerometer's gravity vector;
        large tilt (e.g. climbing a wall) is rejected so bad scans are skipped.
        """
        try:
            ax, ay, az = self.imu['accelerometer'].getValues()
        except Exception:
            return True
        if any(np.isnan(v) for v in (ax, ay, az)) or abs(az) < 1e-3:
            return True
        return (np.hypot(ax, ay) / abs(az)) < max_tan_pitch
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    # ================================================================== #
    # Motion primitives                                                   #
    # ================================================================== #
    def stop_motor(self):
        """Set all wheel velocities to zero."""
        for m in self.motors.values():
            m.setVelocity(0.0)
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def set_robot_velocity(self, left_speed, right_speed):
        """Command left/right wheel-pair velocities and record the turn side."""
        self.motors['fl'].setVelocity(left_speed)
        self.motors['rl'].setVelocity(left_speed)
        self.motors['fr'].setVelocity(right_speed)
        self.motors['rr'].setVelocity(right_speed)
        if left_speed < right_speed:
            self.last_turn = 'left'
        elif right_speed < left_speed:
            self.last_turn = 'right'
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def velocity_to_wheel_speeds(self, v, w):
        """Convert body (v, w) to left/right wheel angular speeds."""
        v_left = v - (self.axle_length / 2.0) * w
        v_right = v + (self.axle_length / 2.0) * w
        return v_left / self.wheel_radius, v_right / self.wheel_radius
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def is_turning(self):
        """True when the wheel pairs differ enough to indicate rotation."""
        return abs(self.motors['fl'].getVelocity() - self.motors['fr'].getVelocity()) > 0.02
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def turn_right_milisecond(self, ms=200):
        """Turn in place to the right for the given duration, then stop."""
        self.set_robot_velocity(4, -4)
        res = self.step(ms)
        self.stop_motor()
        return res
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def turn_left_milisecond(self, ms=200):
        """Turn in place to the left for the given duration, then stop."""
        self.set_robot_velocity(-4, 4)
        res = self.step(ms)
        self.stop_motor()
        return res
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def move_backward_milisecond(self, ms=300):
        """Drive straight backward for the given duration, then stop."""
        self.set_robot_velocity(-4, -4)
        res = self.step(ms)
        self.stop_motor()
        return res
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def turn_by(self, degrees, direction='left', speed=8.0):
        """Rotate in place by an angle using drift-free compass feedback.

        Preferred over timed turns because it closes the loop on the absolute
        compass heading, so the achieved angle does not depend on wheel slip or
        timestep jitter.  ``speed`` allows slower, cleaner turns (e.g. for the
        initial mapping scan).  Returns the last ``step`` result (-1 if quit).
        """
        target = np.deg2rad(abs(degrees))
        start = self._heading_rad
        if direction == 'left':
            self.set_robot_velocity(-speed, speed)
        else:
            self.set_robot_velocity(speed, -speed)
        res = 0
        while True:
            res = self.step()
            if res == -1:
                break
            if abs(utils.get_angle_diff(self._heading_rad, start)) >= target - 0.05:
                break
        self.stop_motor()
        return res
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def turn_to_heading(self, target_rad, tol=0.12, max_iters=250, speed=6.0):
        """Rotate in place to face an absolute world heading (compass feedback).

        Uses a pure differential spin, so the robot pivots without translating
        and NEVER drives in reverse.  Used to face a freshly planned path before
        following it, and as the recovery primitive instead of reversing.
        """
        for _ in range(max_iters):
            err = utils.get_angle_diff(target_rad, self._heading_rad)
            if abs(err) < tol:
                break
            if err > 0:
                self.set_robot_velocity(-speed, speed)   # turn left (CCW)
            else:
                self.set_robot_velocity(speed, -speed)   # turn right (CW)
            if self.step() == -1:
                break
        self.stop_motor()
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    # ================================================================== #
    # DWA local controller                                                #
    # ================================================================== #
    def dwa_planner(self, world_target):
        """Pick (v, w) that best approaches ``world_target`` while staying clear.

        Samples a discrete velocity window, rolls each candidate forward a
        couple of steps, rejects candidates that hit a lethal cell or recede
        from the target, and scores the rest by heading, closeness and speed.
        """
        max_speed = MAX_VELOCITY * self.wheel_radius
        x, y = self._odom_x, self._odom_y
        theta = self._heading_rad
        cur_dist = float(np.linalg.norm(world_target - np.array([x, y])))
        dt = self.time_step / 1000.0

        best_score, best_v, best_w = -np.inf, 0.0, 0.0
        pred_dist = cur_dist
        for v in DWA_VELOCITY_SAMPLES:
            for w in DWA_ANGULAR_SAMPLES:
                cx, cy, ct = x, y, theta
                ok = True
                for _ in range(2):
                    cx += v * np.cos(ct) * dt
                    cy += v * np.sin(ct) * dt
                    ct += w * dt
                    if self.there_is_obstacle(self.convert_to_map_coordinates(cx, cy)):
                        ok = False
                        break
                    pred_dist = float(np.linalg.norm(world_target - np.array([cx, cy])))
                    if pred_dist - cur_dist > DWA_PREDICTION_DISTANCE_THRESHOLD:
                        ok = False
                        break
                if not ok:
                    continue
                heading_err = utils.get_angle_diff(
                    np.arctan2(world_target[1] - cy, world_target[0] - cx), ct)
                score = (DWA_HEADING_WEIGHT * np.cos(heading_err)
                         + DWA_DISTANCE_WEIGHT * (1 - pred_dist / 2.0)
                         + DWA_SPEED_WEIGHT * (v / max_speed))
                if score > best_score:
                    best_score, best_v, best_w = score, v, w
        return best_v, best_w
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting and adapted to the teams use case.

    def follow_local_target(self, map_target):
        """Take one DWA step toward a grid cell; return (reached, is_stuck).

        Sets wheel velocities as a side effect.
        """
        if self.get_map_distance(map_target) < PATH_FOLLOWING_TARGET_REACH_DISTANCE:
            return True, False

        # FIX (robot drove backward / did not follow the path): the old code
        # reported is_stuck=True whenever the robot moved < 0.05 m between two
        # *consecutive* ~32 ms ticks.  That is physically impossible to satisfy
        # in a single timestep, so it fired on almost every call and made the
        # navigator run a BACKWARD recovery + replan every tick -- the robot
        # nudged forward for one step, reversed, and never followed the path.
        # Genuine (multi-tick) stuck detection lives in
        # Navigator._follow_waypoints, so this per-tick step never reports stuck.
        is_stuck = False

        wx, wy = self.convert_to_world_coordinates(map_target[0], map_target[1])
        v, w = self.dwa_planner(np.array([wx, wy]))
        left, right = self.velocity_to_wheel_speeds(v, w)
        self.set_robot_velocity(left, right)
        return False, is_stuck
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting and adapted to the teams use case.

    # ================================================================== #
    # Map stamping helpers                                                #
    # ================================================================== #
    def mark_green_cells(self, map_points):
        """Stamp projected green-ground map cells as lethal GREEN_CARPET."""
        if map_points is None or len(map_points) == 0:
            return
        pts = np.asarray(map_points, dtype=np.int32)
        h, w = self.grid_map.shape
        xs, ys = pts[:, 0], pts[:, 1]
        keep = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
        self.grid_map[ys[keep], xs[keep]] = GREEN_CARPET
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def mark_closure_block(self):
        """Mark the rectangle just ahead of the robot as CLOSED (blocked)."""
        try:
            return self.map_object.mark_closure_rect_simple()
        except Exception as exc:
            print(f"[robot] mark_closure_block failed: {exc}")
            return False
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    # ================================================================== #
    # Visualisation hand-off (read-only pygame thread)                    #
    # ================================================================== #
    def set_vis_path(self, path):
        """Publish the current planned path for the visualiser."""
        try:
            with self.map_object.vis_lock:
                self.map_object.current_path = path
        except Exception:
            pass
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def update_vis(self):
        """Publish the current robot cell to the visualiser."""
        try:
            with self.map_object.vis_lock:
                mx, my = self.get_map_position()
                self.map_object.robot_position = (int(mx), int(my))
        except Exception:
            pass
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.
