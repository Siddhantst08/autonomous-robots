"""Navigation module.

Owns the single-threaded *plan -> follow -> replan* control loop, reactive
safety, and stuck/obstacle recovery.  The navigator drives the robot toward a
map-cell goal using an externally supplied A* planner while continuously:

  * updating the occupancy grid from the lidar (so previously unseen walls are
    added to the map *before* the robot can collide with them),
  * enforcing an emergency stop when the lidar/range sensors report an obstacle
    inside the safety envelope (protecting odometry from collision slip),
  * braking and backing off for green (Poison) ground,
  * calling a per-tick mission callback (used to scan for pillars).

All motion goes through the owning robot's single ``step()`` — there is no
threading here.
"""

import numpy as np

from CONSTANTS import (
    NAV_WAYPOINT_STRIDE, NAV_WAYPOINT_REACH_PX, NAV_GOAL_REACH_PX,
    NAV_MAX_REPLANS, NAV_MAX_STEPS_PER_GOAL, NAV_MAP_UPDATE_EVERY,
    NAV_MAP_UPDATE_MIN_DIST_M, NAV_STUCK_DIST_M, NAV_STUCK_PATIENCE,
    SAFE_STOP_DIST_M, SAFE_RANGE_STOP_M, SAFE_FRONT_CONE_DEG,
    EMERGENCY_BACKUP_MS, GREEN_MARK_MIN_POINTS, DEPTH_FRONT_STOP_M,
    RECOVERY_ALLOW_REVERSE, NUDGE_ROTATE_DEG, FACE_TARGET_ANGLE_TOL_RAD,
)
from my_robot import MyRobot


class Navigator:
    """Drives the robot to map goals with mapping, safety and replanning."""

    def __init__(self, robot:MyRobot, perception):
        """Store the robot (motion/pose) and perception dependencies."""
        self.robot = robot
        self.perception = perception
        # World position at which the occupancy grid was last updated. Used to
        # gate map updates on a stable travelled distance (see _periodic_map_update).
        self._last_map_update_pos = None

    # ------------------------------------------------------------------ #
    # Public API                                                          #
    # ------------------------------------------------------------------ #
    def navigate_to(self, goal_map, planner, reach_px=NAV_GOAL_REACH_PX,
                    tick_cb=None, allow_green_avoid=True,
                    max_steps=NAV_MAX_STEPS_PER_GOAL):
        """Plan a path to ``goal_map`` and follow it, replanning as needed.

        Parameters
        ----------
        goal_map : (x, y) target cell in grid coordinates.
        planner  : callable(start_cell, goal_cell) -> path (list of (x, y)).
        reach_px : goal-reached tolerance in pixels.
        tick_cb  : optional callable() invoked once per control tick; if it
                   returns the string ``'abort'`` the navigation stops early.
        allow_green_avoid : when True, green ground triggers avoidance.
        max_steps : hard cap on control steps for this goal.

        Returns 'reached', 'aborted', or 'failed'.
        """
        steps_used = 0
        last_progress = self.robot.get_position()
        stagnant = 0
        rev_count = 0
        for _ in range(NAV_MAX_REPLANS):
            start = tuple(int(v) for v in self.robot.get_map_position())
            path = planner(start, tuple(int(v) for v in goal_map))
            # if not path or len(path) < 2:
            
            if not path:
                if rev_count < 2:
                    # No achievable path: rotate in place to change the viewpoint
                    # (never reverse) and try planning again.
                    if not self._nudge_rotate(rotate=True):
                        print("nudge_rotate from navigate_to1")
                        rev_count = rev_count+1
                        return 'failed'
                    rev_count = rev_count+1
                    print("nudge_rotate from navigate_to2")
                    
                else:
                    print("emergency clear map")
                    self.robot.map_object.emergency_clear()

                continue

            rev_count = 0

            self.robot.set_vis_path(path)
            waypoints = self._to_waypoints(path)
            # Face the (re)planned path first so DWA turns onto it by pivoting
            # in place rather than creeping forward into an obstacle.
            self._face_path(waypoints)

            result, used = self._follow_waypoints(
                waypoints, goal_map, reach_px, tick_cb, allow_green_avoid,
                max_steps - steps_used)
            steps_used += used

            if result in ('reached', 'aborted'):
                return result
            if steps_used >= max_steps:
                break
            # result == 'replan': if the robot has made no real progress across
            # replans, rotate to break the deadlock (still no reverse).
            now = self.robot.get_position()
            if np.linalg.norm(now - last_progress) < 0.10:
                stagnant += 1
                if stagnant >= 2:
                    self._nudge_rotate()
                    print("nudge_rotate from navigate_to3")
                    stagnant = 0
            else:
                stagnant = 0
                last_progress = now
        # Final tolerance check in case we ended right next to the goal.
        if self.robot.get_map_distance(goal_map) < reach_px:
            return 'reached'
        return 'failed'

    # ------------------------------------------------------------------ #
    # Internal following loop                                             #
    # ------------------------------------------------------------------ #
    def _follow_waypoints(self, waypoints, goal_map, reach_px, tick_cb,
                          allow_green_avoid, step_budget):
        """Follow a waypoint list; return (result, steps_used).

        ``result`` is one of 'reached', 'aborted', 'replan', 'failed'.
        """
        robot = self.robot
        steps = 0
        stuck_ticks = 0
        last_pos = robot.get_position()

        for wp in waypoints:
            while steps < step_budget:
                if robot.step() == -1:
                    return 'failed', steps
                steps += 1

                # if steps % 5 == 0:
                    # robot.map_object.clear_sensor_free_space()
                self._periodic_map_update(steps)
                robot.update_vis()

                if tick_cb is not None and tick_cb() == 'abort':
                    robot.stop_motor()
                    return 'aborted', steps

                if robot.get_map_distance(goal_map) < reach_px:
                    robot.stop_motor()
                    return 'reached', steps

                if self._emergency_check(allow_green_avoid):
                    return 'replan', steps

                if robot.get_map_distance(wp) < NAV_WAYPOINT_REACH_PX:
                    break  # advance to next waypoint

                # Progress / stuck monitoring.
                # FIX (robot drove backward / did not follow the path): this is
                # now the *only* stuck detector.  It accumulates ticks since the
                # robot last actually advanced NAV_STUCK_DIST_M and only recovers
                # after NAV_STUCK_PATIENCE ticks, so normal forward driving is
                # never mistaken for being stuck (the previous per-tick check in
                # follow_local_target did exactly that and caused constant
                # backward recovery).
                pos = robot.get_position()
                if np.linalg.norm(pos - last_pos) < NAV_STUCK_DIST_M:
                    stuck_ticks += 1
                    if stuck_ticks >= NAV_STUCK_PATIENCE:
                        # Path not achievable from here: rotate (no reverse) and
                        # signal the caller to replan.
                        robot.stop_motor()
                        self._nudge_rotate()
                        print("nudge_rotate from _follow_waypoints1")
                        robot.map_object.mark_blocked_ahead()
                        return 'replan', steps
                else:
                    stuck_ticks = 0
                    last_pos = pos

                # Local reactive control toward the waypoint (DWA) drives the
                # robot forward along the path.  is_stuck is intentionally always
                # False now (see follow_local_target); real stuck handling is the
                # multi-tick monitor above.
                _reached, is_stuck = robot.follow_local_target(wp)
                if is_stuck:
                    robot.stop_motor()
                    self._nudge_rotate()
                    print("nudge_rotate from _follow_waypoints2")
                    robot.map_object.mark_blocked_ahead()
                    return 'replan', steps

        # Ran out of waypoints without hitting the reach test.
        if robot.get_map_distance(goal_map) < reach_px:
            return 'reached', steps
        return 'replan', steps

    # ------------------------------------------------------------------ #
    # Safety, mapping and recovery helpers                                #
    # ------------------------------------------------------------------ #
    def _emergency_check(self, allow_green_avoid):
        """Stop (and map) when an obstacle or green ground is imminent.

        FIX (no reverse fallback): the robot previously drove BACKWARD here as
        its avoidance move.  Per the requirements it now simply STOPS and lets
        the caller REPLAN -- the obstacle was just written into the map, so A*
        routes around it on the next plan.  A tiny reverse happens only if
        RECOVERY_ALLOW_REVERSE is explicitly enabled (default off).
        Returns True if an emergency was handled (caller should replan).
        """
        robot = self.robot

        # Green (Poison) ground directly ahead -> never drive onto it.
        if allow_green_avoid and self.perception.green_close_ahead():
            robot.stop_motor()
            self._stamp_green()
            self._optional_reverse()
            print("reverse from _emergency_check1")
            return True

        front = self.perception.lidar_front_min_dist(SAFE_FRONT_CONE_DEG)
        ds = robot.get_distances()
        range_hit = len(ds) >= 3 and min(ds[0], ds[2]) < SAFE_RANGE_STOP_M
        # Depth camera catches flat-on-floor and floating walls the lidar misses.
        depth_front = self.perception.depth_front_min_dist()
        if front < SAFE_STOP_DIST_M or range_hit or depth_front < DEPTH_FRONT_STOP_M:
            # Obstacle recorded by the lidar/depth map update above -> stop and
            # let the caller replan around it (no reverse, no CLOSED stamp that
            # could seal a legitimate narrow passage).
            robot.stop_motor()
            self._stamp_depth()
            self._optional_reverse()
            print("reverse from _emergency_check2")
            return True
        return False

    def _stamp_green(self):
        """Project currently-visible green ground and mark it lethal on the map."""
        try:
            pts = self.perception.green_ground_map_points()
            if pts.shape[0] >= GREEN_MARK_MIN_POINTS:
                self.robot.mark_green_cells(pts)
        except Exception as exc:  # projection must never crash navigation
            print(f"[nav] green stamp failed: {exc}")

    def _stamp_depth(self):
        """Stamp depth-camera obstacles (flat-on-floor & floating walls) the
        horizontal lidar cannot see, so planning and DWA avoid them."""
        try:
            ground, floating = self.perception.depth_obstacle_points()
            self.robot.map_object.mark_depth_obstacles(ground, floating)
        except Exception as exc:  # projection must never crash navigation
            print(f"[nav] depth stamp failed: {exc}")

    def _periodic_map_update(self, step_i):
        """Fold the latest lidar/depth/green scan into the grid, but only after
        the robot has moved a small, STABLE distance since the last update.

        Why not every tick: updating the map continuously while the robot
        maneuvers -- especially in narrow maze passages where it wiggles and
        creeps -- captures unstable/false readings and injects significant noise
        into the map, which then degrades planning and makes tight passages hard
        to traverse.  Gating on a minimum travelled distance lets the readings
        stabilize before they are incorporated, producing a much cleaner map.

        Three gates, cheapest first:
          1. tick throttle   -- only look at most every NAV_MAP_UPDATE_EVERY ticks;
          2. motion sanity   -- skip while turning in place or on a tilted
                                platform (both distort the lidar scan);
          3. distance gate   -- require >= NAV_MAP_UPDATE_MIN_DIST_M of travel
                                since the previous update.
        """
        # (1) Cheap tick throttle so we do not run the checks every single tick.
        if step_i % NAV_MAP_UPDATE_EVERY != 0:
            return
        # (2) Never map while spinning in place or tilted -> distorted scans.
        if self.robot.is_turning() or not self.robot.robot_on_ground():
            return
        # (3) Stable-distance gate: only update once the robot has advanced a
        #     small amount, so transient/noisy readings are not folded in while
        #     it is barely moving or jittering in a tight passage.
        pos = self.robot.get_position()
        if (self._last_map_update_pos is not None
                and np.linalg.norm(pos - self._last_map_update_pos) < NAV_MAP_UPDATE_MIN_DIST_M):
            return
        self._last_map_update_pos = pos
        try:
            self.robot.lidar_update_map()
            self._stamp_green()
            self._stamp_depth()
        except Exception as exc:
            print(f"[nav] map update failed: {exc}")

    def _optional_reverse(self):
        """Tiny backward nudge, ONLY if RECOVERY_ALLOW_REVERSE is enabled.

        Disabled by default so the robot never uses reverse as an obstacle-
        avoidance strategy (it stops and replans/rotates instead).
        """
        if RECOVERY_ALLOW_REVERSE:
            self.robot.move_backward_milisecond(EMERGENCY_BACKUP_MS)

    def _nudge_rotate(self, rotate = False):
        """Rotate in place toward the more open side to change the viewpoint and
        break a deadlock WITHOUT driving in reverse; the caller then replans.

        Returns False only on simulation end.
        """
        robot = self.robot
        robot.stop_motor()
        self._optional_reverse()                     # no-op unless explicitly enabled
        print("reverse from _nudge_rotate")
        if rotate:
            ds = robot.get_distances()
            # Turn toward the side with more clearance (fl=ds[0], fr=ds[2]).
            direction = 'left' if (len(ds) >= 4 and ds[0] >= ds[2]) else 'right'
            return robot.turn_by(NUDGE_ROTATE_DEG, direction=direction) != -1
        return True

    def _face_path(self, waypoints):
        """Pivot in place to face the first non-trivial waypoint before driving.

        Turning onto a freshly (re)planned path by pivoting avoids creeping
        forward into an obstacle (or needing to reverse) when the new path
        heads in a very different direction.  No-op when already aligned.
        """
        robot = self.robot
        for wp in waypoints:
            if robot.get_map_distance(wp) < NAV_WAYPOINT_REACH_PX + 2:
                continue
            wx, wy = robot.convert_to_world_coordinates(wp[0], wp[1])
            rx, ry = robot.get_position()
            desired = np.arctan2(wy - ry, wx - rx)
            robot.turn_to_heading(desired, tol=FACE_TARGET_ANGLE_TOL_RAD)
            return

    # ------------------------------------------------------------------ #
    # Utilities                                                           #
    # ------------------------------------------------------------------ #
    @staticmethod
    def _to_waypoints(path):
        """Down-sample a dense path to sparse waypoints, keeping the endpoint."""
        wps = list(path[::NAV_WAYPOINT_STRIDE])
        if wps and wps[-1] != path[-1]:
            wps.append(path[-1])
        return wps
