"""Perception module.

All camera- and lidar-derived interpretation lives here: HSV colour
segmentation, pillar (blue/yellow cylinder) detection & world localisation,
green (Poison) hazard detection & ground projection, red-wall detection and
front-obstacle distance.

Design notes
------------
- This class NEVER calls ``robot.step()`` and NEVER spawns threads.  It only
  reads already-latched sensor values, so it is safe to call from the single
  main control loop.
- Pillars and green cells are localised into the shared world/map frame using
  the robot's odometry pose (``robot.get_position`` / ``robot.get_heading``)
  and the aligned RGB-D camera, avoiding brittle pixel-height heuristics.
"""

import numpy as np
import cv2

import utils
from CONSTANTS import (
    COLUMN_MIN_PIXELS, COLUMN_MAX_LOCALIZE_M,
    GREEN_AHEAD_MIN_PIXELS, GREEN_STOP_DEPTH_M,
    GREEN_CARPET_DILATION_KERNEL_SIZE, GREEN_CARPET_DILATION_ITERATIONS,
    COLOR_DETECTION_RED_PIXEL_RATIO,
    RED_WALL_HSV_LOWER1, RED_WALL_HSV_UPPER1, RED_WALL_HSV_LOWER2, RED_WALL_HSV_UPPER2,
    SAFE_FRONT_CONE_DEG, MAP_SIZE,
    DEPTH_OBST_SAMPLE_STRIDE, DEPTH_OBST_MIN_M, DEPTH_OBST_MAX_M,
    DEPTH_COLLISION_Z_MIN, DEPTH_COLLISION_Z_MAX, DEPTH_FLOATING_Z_MIN,
    DEPTH_OBST_MIN_POINTS,
)


class Perception:
    """Interprets the robot's cameras and lidar into semantic observations."""

    def __init__(self, robot):
        """Store a reference to the owning robot (hardware/pose layer)."""
        self.robot = robot

    # ------------------------------------------------------------------ #
    # Raw image helpers                                                   #
    # ------------------------------------------------------------------ #
    def get_hsv_image(self):
        """Return the current RGB camera frame converted to HSV, or None."""
        cam = self.robot.camera_rgb
        if cam is None:
            return None
        data = cam.getImage()
        if data is None:
            return None
        w, h = cam.getWidth(), cam.getHeight()
        bgra = np.frombuffer(data, np.uint8).reshape((h, w, 4))
        bgr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def get_depth_m(self):
        """Return the depth camera range image in metres (inf where invalid)."""
        cam = self.robot.camera_depth
        if cam is None:
            return None
        data = cam.getRangeImage()
        if data is None:
            return None
        w, h = cam.getWidth(), cam.getHeight()
        return np.array(data, dtype=np.float32).reshape((h, w))
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    # ------------------------------------------------------------------ #
    # Pillar detection & localisation                                     #
    # ------------------------------------------------------------------ #
    def detect_column_color(self, hsv=None):
        """Return 'blue'/'yellow' if a sufficiently large pillar blob is seen.

        Yellow is checked first so that both colours can be reported over
        successive frames; returns None when neither exceeds the pixel gate.
        """
        if hsv is None:
            hsv = self.get_hsv_image()
        if hsv is None:
            return None
        for color in ('yellow', 'blue'):
            mask = utils.segment_color(hsv, color)
            if mask is not None and cv2.countNonZero(mask) >= COLUMN_MIN_PIXELS:
                return color
        return None
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def localize_column(self, color, hsv=None, depth=None):
        """Estimate the world (x, y) of a coloured pillar from RGB-D.

        Uses the colour blob centroid for bearing and the median valid depth
        over the blob for range, then transforms the bearing/range point from
        the robot frame into the world frame.  Returns (world_xy, n_pixels) or
        (None, 0) when the pillar is absent or too far to range reliably.
        """
        if hsv is None:
            hsv = self.get_hsv_image()
        if depth is None:
            depth = self.get_depth_m()
        if hsv is None or depth is None:
            return None, 0

        mask = utils.segment_color(hsv, color)
        n = 0 if mask is None else int(cv2.countNonZero(mask))
        if n < COLUMN_MIN_PIXELS:
            return None, 0

        # Blob horizontal centroid -> bearing in the robot frame.
        M = cv2.moments(mask)
        if M["m00"] <= 0:
            return None, 0
        cx_pix = M["m10"] / M["m00"]

        # Median depth over the blob (robust to edge pixels / holes).
        ys, xs = np.where(mask > 0)
        blob_depth = depth[ys, xs]
        valid = blob_depth[np.isfinite(blob_depth) & (blob_depth > 0.05)]
        if valid.size == 0 or float(np.median(valid)) > COLUMN_MAX_LOCALIZE_M:
            return None, n
        rng = float(np.median(valid))

        # Bearing: pixel right of centre -> object to the robot's right (-y).
        x_norm = (cx_pix - self.robot.cx) / self.robot.fx
        bearing = -np.arctan(x_norm)

        local = np.array([[rng * np.cos(bearing), rng * np.sin(bearing)]])
        world = self.robot.transform_points_to_world(local)[0]
        return np.array([float(world[0]), float(world[1])]), n
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def column_bearing_error_px(self, color, hsv=None):
        """Horizontal pixel offset of the pillar blob from image centre.

        Positive means the pillar is to the right; None if not visible.
        """
        if hsv is None:
            hsv = self.get_hsv_image()
        if hsv is None:
            return None
        mask = utils.segment_color(hsv, color)
        if mask is None or cv2.countNonZero(mask) < COLUMN_MIN_PIXELS:
            return None
        M = cv2.moments(mask)
        if M["m00"] <= 0:
            return None
        return int(M["m10"] / M["m00"]) - mask.shape[1] // 2
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    # ------------------------------------------------------------------ #
    # Green (Poison) hazard                                               #
    # ------------------------------------------------------------------ #
    def green_close_ahead(self):
        """True when a large, near green patch sits in the lower-central view.

        Combines an HSV pixel count in the lower-centre region with the depth
        camera so we only brake for green that is actually close in front.
        """
        hsv = self.get_hsv_image()
        if hsv is None:
            return False
        h, w, _ = hsv.shape
        roi = hsv[h // 2:, w // 4: 3 * w // 4]
        mask = utils.segment_color(roi, 'green')
        if mask is None or cv2.countNonZero(mask) < GREEN_AHEAD_MIN_PIXELS:
            return False
        depth = self.get_depth_m()
        if depth is None:
            return True  # be conservative: green seen, no depth -> avoid
        droi = depth[h // 2:, w // 4: 3 * w // 4]
        gd = droi[mask > 0]
        gd = gd[np.isfinite(gd) & (gd > 0.02)]
        if gd.size == 0:
            return True
        return float(np.min(gd)) < GREEN_STOP_DEPTH_M
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting and adapted to the teams use case.

    def green_ground_map_points(self):
        """Project green pixels onto the floor and return their map cells.

        Uses an inverse pinhole projection on the flat ground plane assuming a
        fixed camera height.  Points are conservative (dilated) so the lethal
        stamp fully covers the hazard.  Returns an (N, 2) int array of map
        (x, y) indices, possibly empty.
        """
        hsv = self.get_hsv_image()
        if hsv is None:
            return np.empty((0, 2), dtype=np.int32)

        mask = utils.segment_color(hsv, 'green')
        if mask is None:
            return np.empty((0, 2), dtype=np.int32)
        k = cv2.getStructuringElement(cv2.MORPH_RECT,
                                      (GREEN_CARPET_DILATION_KERNEL_SIZE,
                                       GREEN_CARPET_DILATION_KERNEL_SIZE))
        mask = cv2.dilate(mask, k, iterations=GREEN_CARPET_DILATION_ITERATIONS)
        mask[:int(self.robot.cam_height * 0.5), :] = 0  # ignore above horizon

        vs, us = np.where(mask == 255)
        if us.size == 0:
            return np.empty((0, 2), dtype=np.int32)

        x_norm = (us - self.robot.cx) / self.robot.fx
        y_norm = (vs - self.robot.cy) / self.robot.fy
        d = self.robot.camera_height_m / (y_norm + 1e-6)  # ground-plane range
        keep = (y_norm > 1e-3) & (d > 0.1) & (d < 4.0)
        d, x_norm = d[keep], x_norm[keep]
        if d.size == 0:
            return np.empty((0, 2), dtype=np.int32)

        local = np.stack([d + self.robot.X_offset, -d * x_norm + self.robot.Y_offset], axis=1)
        world = self.robot.transform_points_to_world(local)
        return self.robot.map_object.convert_to_map_coordinate_matrix(world)
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting and adapted to the teams use case.

    def lidar_front_min_dist(self, half_angle_deg=SAFE_FRONT_CONE_DEG):
        """Minimum lidar range within a front cone; falls back to range sensors."""
        pts = self.robot.get_pointcloud_2d()
        if pts.shape[0] > 0:
            ang = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
            front = pts[(ang > -half_angle_deg) & (ang < half_angle_deg)]
            if front.shape[0] > 0:
                return float(np.min(np.linalg.norm(front, axis=1)))
        ds = self.robot.get_distances()
        return float(min(ds[0], ds[2])) if len(ds) >= 3 else float('inf')
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting and adapted to the teams use case.

    # ------------------------------------------------------------------ #
    # Depth-camera obstacle layer (flat-on-floor & floating walls)        #
    # ------------------------------------------------------------------ #
    def depth_obstacle_points(self):
        """Project the depth camera into the ground plane to find obstacles the
        horizontal lidar misses (walls flat on the floor and floating walls).

        Each valid depth pixel is back-projected into the robot frame; only
        returns within the robot's collision height band are kept and binned
        into map cells.  A cell is classified as a *floating wall* when its
        lowest return sits above ``DEPTH_FLOATING_Z_MIN`` (i.e. there is a gap
        beneath it), otherwise it is a ground-touching obstacle.

        Returns (ground_cells, floating_cells) as (N, 2) int map (x, y) arrays.
        """
        r = self.robot
        depth = self.get_depth_m()
        empty = np.empty((0, 2), dtype=np.int32)
        if depth is None:
            return empty, empty

        h, w = depth.shape
        s = DEPTH_OBST_SAMPLE_STRIDE
        uu, vv = np.meshgrid(np.arange(0, w, s), np.arange(0, h, s))
        Z = depth[vv, uu].astype(np.float32)
        uu = uu.astype(np.float32)
        vv = vv.astype(np.float32)

        valid = np.isfinite(Z) & (Z > DEPTH_OBST_MIN_M) & (Z < DEPTH_OBST_MAX_M)
        Z, uu, vv = Z[valid], uu[valid], vv[valid]
        if Z.size == 0:
            return empty, empty

        # Camera(level) -> robot frame: x forward, y left, z up (height).
        x_r = Z + r.X_offset
        y_r = -Z * (uu - r.cx_d) / r.fx_d
        z_r = r.camera_height_m - Z * (vv - r.cy_d) / r.fy_d

        band = (z_r > DEPTH_COLLISION_Z_MIN) & (z_r < DEPTH_COLLISION_Z_MAX)
        x_r, y_r, z_r = x_r[band], y_r[band], z_r[band]
        if x_r.size == 0:
            return empty, empty

        world = r.transform_points_to_world(np.stack([x_r, y_r], axis=1))
        cells = r.map_object.convert_to_map_coordinate_matrix(world)
        mx, my = cells[:, 0], cells[:, 1]
        inb = (mx >= 0) & (mx < MAP_SIZE) & (my >= 0) & (my < MAP_SIZE)
        mx, my, z_r = mx[inb], my[inb], z_r[inb]
        if mx.size == 0:
            return empty, empty

        # Bin points per cell; keep dense cells and record their lowest return.
        cell_id = my.astype(np.int64) * MAP_SIZE + mx.astype(np.int64)
        uniq, inv = np.unique(cell_id, return_inverse=True)
        counts = np.bincount(inv)
        min_h = np.full(uniq.shape, np.inf, dtype=np.float32)
        np.minimum.at(min_h, inv, z_r.astype(np.float32))

        keep = counts >= DEPTH_OBST_MIN_POINTS
        uniq, min_h = uniq[keep], min_h[keep]
        if uniq.size == 0:
            return empty, empty

        floating = min_h > DEPTH_FLOATING_Z_MIN
        u_mx = (uniq % MAP_SIZE).astype(np.int32)
        u_my = (uniq // MAP_SIZE).astype(np.int32)
        ground_cells = np.stack([u_mx[~floating], u_my[~floating]], axis=1)
        floating_cells = np.stack([u_mx[floating], u_my[floating]], axis=1)
        return ground_cells, floating_cells
# REFERENCE: Original code authored by the project team. External sources or LLMs were used later to adapt the code for the teams usecase.

    def depth_front_min_dist(self):
        """Nearest forward distance (m) to a collision-height obstacle in the
        central depth view; catches flat/floating walls the lidar misses.

        Returns ``inf`` when nothing relevant is in view.
        """
        r = self.robot
        depth = self.get_depth_m()
        if depth is None:
            return float('inf')
        h, w = depth.shape
        u0, u1 = w // 3, 2 * w // 3          # central horizontal third
        Z = depth[:, u0:u1].astype(np.float32)
        vv = np.arange(h, dtype=np.float32).reshape(-1, 1)
        z_r = r.camera_height_m - Z * (vv - r.cy_d) / r.fy_d
        band = (np.isfinite(Z) & (Z > DEPTH_OBST_MIN_M) & (Z < DEPTH_OBST_MAX_M)
                & (z_r > DEPTH_COLLISION_Z_MIN) & (z_r < DEPTH_COLLISION_Z_MAX))
        return float(np.min(Z[band])) if np.any(band) else float('inf')
# REFERENCE: Original code authored by the project team. Rxternal sources or LLMs were used later to adapt the code for the teams usecase.
