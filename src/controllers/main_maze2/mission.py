"""Mission control (high-level state machine).

Implements the task policy on top of the perception, mapping, planning and
navigation layers:

    INIT_SCAN  -> spin in place, build the first local map, spot pillars
    EXPLORE    -> frontier exploration until BOTH pillars are localised
    TRAVERSE   -> drive start -> blue pillar -> yellow pillar
    DONE

Rules enforced:
  * Only the permitted onboard sensors are used (no Supervisor).
  * Green (Poison) ground is avoided: it is stamped lethal on the grid and the
    navigator brakes for it reactively.
  * Pillars are stored the moment they are confidently localised, in whatever
    order they are discovered; the final traversal is always blue -> yellow.
"""

import numpy as np

from CONSTANTS import *
from navigation import Navigator
from my_robot import MyRobot
from perception import Perception


class Mission:
    """Coordinates exploration, pillar discovery and the blue->yellow run."""

    def __init__(self, robot:MyRobot, perception:Perception, navigator:Navigator):
        """Wire together the robot and its perception/navigation helpers."""
        self.robot = robot
        self.perception = perception
        self.navigator = navigator

    # ------------------------------------------------------------------ #
    # Entry point                                                         #
    # ------------------------------------------------------------------ #
    def run(self):
        """Execute the full mission; returns True if both pillars were reached."""
        self.robot.map_object.start_visualization()
        print("[mission] INIT_SCAN")
        self._initial_scan()

        print("[mission] EXPLORE")
        self._explore()

        if self.robot.blue_world is None or self.robot.yellow_world is None:
            print("[mission] exploration ended without both pillars localised; "
                  "attempting best-effort traversal with what is known")


        print("[mission] TRAVERSE blue -> yellow")
        # Requirement: the two pillars may be discovered in EITHER order during
        # exploration (see _scan_for_pillars), but the final traversal is always
        # start -> blue pillar -> yellow pillar.
        ok_blue = self._go_to_pillar('blue')
        ok_yellow = self._go_to_pillar('yellow')

        self.robot.stop_motor()
        print(f"[mission] DONE (blue_reached={ok_blue}, yellow_reached={ok_yellow})")
        if self.robot.blue_world is None or self.robot.yellow_world is None:
            print("[mission] EXPLORE Again")
            self._explore()
            print("[mission] TRAVERSE blue -> yellow Again")
            # Requirement: the two pillars may be discovered in EITHER order during
            # exploration (see _scan_for_pillars), but the final traversal is always
            # start -> blue pillar -> yellow pillar.
            ok_blue = self._go_to_pillar('blue')
            ok_yellow = self._go_to_pillar('yellow')

            self.robot.stop_motor()
            print(f"[mission] DONE Again (blue_reached={ok_blue}, yellow_reached={ok_yellow})")

        return ok_blue and ok_yellow
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting. 

    # ------------------------------------------------------------------ #
    # Phase 1 — initial in-place scan                                     #
    # ------------------------------------------------------------------ #
    def _initial_scan(self):
        """Rotate ~360 deg in place slowly, building an accurate initial map.

        The scan is split into INITIAL_SCAN_SLICES small, slow in-place turns.
        After each turn the robot is allowed to SETTLE (a few stationary steps)
        before the lidar is folded into the map, so every scan is taken while
        motionless -- this makes the initial mapping much cleaner/more accurate.
        """
        robot = self.robot
        angle = 360.0 / INITIAL_SCAN_SLICES
        self._settle_and_map()
        self._scan_for_pillars()
        for _ in range(INITIAL_SCAN_SLICES):
            if robot.turn_by(angle, direction='left', speed=INITIAL_SCAN_TURN_SPEED) == -1:
                return
            self._settle_and_map()
            self._scan_for_pillars()
            if self._both_pillars_known():
                return
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def _settle_and_map(self):
        """Let the robot come to rest, then fold the lidar into the map twice.

        Waiting for the platform to stop moving before sampling the lidar
        removes motion-induced range distortion, improving map accuracy.
        """
        for _ in range(INITIAL_SCAN_SETTLE_STEPS):
            if self.robot.step() == -1:
                return
        self._safe_map_update()
        self._safe_map_update()
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    # ------------------------------------------------------------------ #
    # Phase 2 — frontier exploration                                      #
    # ------------------------------------------------------------------ #
    def _explore(self):
        """Drive to frontiers until both pillars are localised or budget runs out."""
        for cycle in range(EXPLORE_MAX_CYCLES):
            if self._both_pillars_known():
                print("[explore] both pillars localised")
                return
            self._safe_map_update()

            goal = self._choose_frontier()
            if goal is None:
                print("[explore] no reachable frontier remaining")
                # Fallback: re-scan in place to expose new frontiers.
                self._initial_scan()
                goal = self._choose_frontier()
                if goal is None:
                    return

            print(f"[explore] cycle {cycle}: heading to frontier {goal}")
            result = self.navigator.navigate_to(
                goal,
                planner=self._frontier_planner,
                reach_px=max(NAV_GOAL_REACH_PX, 8),
                tick_cb=self._explore_tick,
            )
            print(result)
            if cycle % 3 == 2:
                print("emergency_clear in explore")
                self.robot.map_object.emergency_clear(value=10)
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def _choose_frontier(self):
        """Pick the nearest reachable frontier-cluster centroid as a map goal."""
        regions = self.robot.map_object.detect_frontiers()
        if not regions:
            return None
        rpos = self.robot.get_map_position()
        centroids = []
        for region in regions:
            if len(region) < EXPLORE_MIN_FRONTIER_CELLS:
                continue
            arr = np.array(region)
            c = (int(arr[:, 0].mean()), int(arr[:, 1].mean()))
            centroids.append((np.linalg.norm(np.array(c) - rpos), c))
        centroids.sort(key=lambda t: t[0])
        # Return the nearest centroid to which a path can actually be planned.
        for _, c in centroids[:6]:
            path = self.robot.map_object.find_path_for_frontier(
                tuple(int(v) for v in rpos), c)
            if path and len(path) > 1:
                return c
        return centroids[0][1] if centroids else None
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting and adapted to the teams use case. 

    # ------------------------------------------------------------------ #
    # Phase 3 — traverse to a pillar                                      #
    # ------------------------------------------------------------------ #
    def _go_to_pillar(self, color):
        """Navigate to a stored pillar; arrival is decided by MAP coordinates.

        Because the pillar's position is already known, whether we have reached
        it is judged purely by the MAP distance between the robot cell and the
        stored pillar cell (< COLUMN_REACH_PX) -- the lidar front distance is
        NOT used for this decision.
        """
        world = self.robot.blue_world if color == 'blue' else self.robot.yellow_world
        if world is None:
            print(f"[traverse] {color} pillar location unknown; skipping")
            return False

        goal_map = self.robot.convert_to_map_coordinates(world[0], world[1])
        self.navigator.navigate_to(
            goal_map,
            planner=self._goal_planner,
            reach_px=COLUMN_REACH_PX,          # map-coordinate arrival tolerance
            tick_cb=lambda: self._approach_tick(color),
        )
        # Reach is determined from map coordinates only (pillar position known).
        reached = self.robot.get_map_distance(goal_map) < COLUMN_REACH_PX
        # Optional visual centring for a tidy final pose (does not affect reach).
        if self.perception.detect_column_color() == color:
            self._center_and_confirm(color)
        print(f"[traverse] {color} pillar reached={reached} "
              f"(map_dist={self.robot.get_map_distance(goal_map):.1f}px)")
        return reached
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting and adapted to the teams use case.

    def _center_and_confirm(self, color):
        """Rotate briefly to centre the pillar in view (best-effort)."""
        robot = self.robot
        for _ in range(60):
            err = self.perception.column_bearing_error_px(color)
            if err is None or abs(err) < 25:
                break
            if err > 0:
                robot.turn_right_milisecond(60)
            else:
                robot.turn_left_milisecond(60)
        robot.stop_motor()
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting. 

    # ------------------------------------------------------------------ #
    # Per-tick callbacks                                                  #
    # ------------------------------------------------------------------ #
    def _explore_tick(self):
        """Called each control tick during exploration: scan + early exit."""
        self._scan_for_pillars()
        return 'abort' if self._both_pillars_known() else None

    def _approach_tick(self, color):
        """Called each control tick while approaching a pillar.

        Only keeps refining the pillar estimates; arrival itself is decided by
        the MAP-distance reach test inside navigate_to (reach_px=COLUMN_REACH_PX),
        so the lidar is not used to determine that the pillar was reached.
        """
        self._scan_for_pillars()          # keep refining both estimates
        return None
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting. 

    # ------------------------------------------------------------------ #
    # Pillar bookkeeping                                                  #
    # ------------------------------------------------------------------ #
    def _scan_for_pillars(self):
        """Detect and (re)localise any visible pillar, storing its world pose."""
        hsv = self.perception.get_hsv_image()
        if hsv is None:
            return
        depth = self.perception.get_depth_m()
        for color in ('blue', 'yellow'):
            world, n = self.perception.localize_column(color, hsv=hsv, depth=depth)
            if world is not None:
                self._store_pillar(color, world, n)
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def _store_pillar(self, color, world, n_pixels):
        """Record or refine a pillar's world/map position and stamp the map."""
        robot = self.robot
        current = robot.blue_world if color == 'blue' else robot.yellow_world
        # Keep the closer / higher-confidence estimate; refine when clearer.
        # if current is not None and n_pixels < COLUMN_REFINE_MIN_PIXELS:
        if n_pixels < COLUMN_REFINE_MIN_PIXELS:
            return
        print(f"pillar found with {n_pixels} pixels")
        cell = robot.convert_to_map_coordinates(world[0], world[1])
        dist = self.robot.get_map_distance(cell)
        # print(f"dist 2 pillar: {dist}")
        new = False
        if color == "blue" and self.robot.blue_world is None:
            new = True
        if color == "yellow" and self.robot.yellow_world is None:
            new = True
        if dist < DEPTH_DEAD_ZONE_PIXELS and not new:
            # print("Too close to pillar, not updating.")
            return
        if color == 'blue':
            robot.blue_world = world
            robot.start_point = tuple(cell)
            robot.map_object.update_map_point(cell, BLUE_COLUMN)
        else:
            robot.yellow_world = world
            robot.end_point = tuple(cell)
            robot.map_object.update_map_point(cell, YELLOW_COLUMN)
        # print(f"[pillar] {color} localised at world={np.round(world, 2)} "
        #       f"cell={cell} px={n_pixels}")
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def _both_pillars_known(self):
        """True once both pillar world positions have been stored."""
        return self.robot.blue_world is not None and self.robot.yellow_world is not None
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    # ------------------------------------------------------------------ #
    # Planner adapters & small utilities                                  #
    # ------------------------------------------------------------------ #
    def _goal_planner(self, start, goal):
        """Adapter: escalating-inflation A* for point-to-point goals."""
        return self.robot.map_object.find_path(start, goal)
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def _frontier_planner(self, start, goal):
        """Adapter: clearance A* tuned for frontier targets."""
        return self.robot.map_object.find_path_for_frontier(start, goal)
# REFERENCE: Original code authored by the project team. No external sources or LLMs were used. Values are calibrated for best performance.

    def _safe_map_update(self):
        """Fold lidar into the grid and stamp visible green, guarding failures."""
        robot = self.robot
        if not robot.robot_on_ground():
            print("robot not on ground")
            return
        try:
            robot.lidar_update_map()
            pts = self.perception.green_ground_map_points()
            if pts.shape[0] > 0:
                robot.mark_green_cells(pts)
            # Depth layer: flat-on-floor and floating walls the lidar cannot see.
            ground, floating = self.perception.depth_obstacle_points()
            robot.map_object.mark_depth_obstacles(ground, floating)
        except Exception as exc:
            print(f"[mission] map update failed: {exc}")
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting and adapted to the teams use case.
