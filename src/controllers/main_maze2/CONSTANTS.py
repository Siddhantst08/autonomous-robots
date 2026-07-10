TIME_STEP = 32
MAX_VELOCITY = 36
WHEEL_RADIUS = 0.043
AXLE_LENGTH = 0.18
MAP_SIZE = 300 # 10m x 10m grid map
# 300 pixels = 10m, so each pixel is 0.0333m = 3.33cm
RESOLUTION = 10.0 / MAP_SIZE

INITIAL_LOG_ODD = 1.0

# --- PROBABILISTIC OCCUPANCY GRID (confidence, 0-100) ---
CONFIDENCE_INITIAL = 100
CONFIDENCE_HIT_INC = 30
CONFIDENCE_MISS_DEC = 3
CONFIDENCE_MAX = 200
CONFIDENCE_MIN = 0
CONFIDENCE_WALL_THRESHOLD = 110
OBSTACLE = 1
FREESPACE = 0
UNKNOWN = 255
BLUE_COLUMN = 100
YELLOW_COLUMN = 150

# Special marker value used to mark temporary closures created by the robot
# Must be different from FREESPACE_VALUE (0) and UNKNOWN_VALUE (255)
CLOSED = 200
# Special marker value for green carpet detection
GREEN_CARPET = 190
# Special marker value for FLOATING walls (elevated walls with a gap beneath
# them) detected by the depth camera but missed by the horizontal lidar.
# Given a distinct value so it can be visualised separately on the map.
FLOATING_WALL = 210
SMALL_WALL = 215
# Minimum acceptable path length (meters) for final start->end planning.
# If a found path is shorter than this, the planner will retry with larger
# obstacle inflation to avoid tiny-gap shortcuts.
PATH_MIN_LENGTH_M = 0.8


# ============================================================================
# HYPERPARAMETERS - Centralized configuration for easy tuning
# ============================================================================

# --- EXPLORATION & TIMING ---
EXPLORATION_STEP_STUCK_CHECK = 15          # [USED] Check if robot is stuck every N steps
EXPLORATION_MAP_UPDATE_FREQ = 20           # [USED] Update map every N steps
EXPLORATION_FRONTIER_SELECTION_FREQ = 5   # [USED] Select new frontier target every N steps
EXPLORATION_START_FRONTIER_AFTER = 5  # [USED] Start frontier selection after N exploration steps (lowered from 50 so robot doesn't idle for the first ~50 iterations after the initial 360 scan)
EXPLORATION_PATH_PLANNING_FREQ = 100       # [USED] Plan global path every N steps

# --- FRONTIER DETECTION & TARGETING ---
FRONTIER_MIN_DISTANCE_NEW = 20              # [USED] Minimum distance (pixels) to consider frontier as new
FRONTIER_APPROACH_DISTANCE = 10             # [USED] Distance (pixels) to consider frontier reached
FRONTIER_VISUALIZATION_COLOR_SMALL = 50     # [USED] Color value for small frontier regions (blue)
FRONTIER_VISUALIZATION_COLOR_MEDIUM = 100   # [USED] Color value for medium frontier regions (cyan)
FRONTIER_VISUALIZATION_COLOR_LARGE = 200    # [USED] Color value for large frontier regions (yellow)
FRONTIER_VISUALIZATION_COLOR_LARGEST = 220  # [USED] Color value for largest frontier regions (red)

# --- WALL & OBSTACLE DETECTION ---
WALL_DETECTION_THRESHOLD_FRONTIER = 0.3     # [USED] Distance (meters) to consider wall blocking frontier
WALL_DETECTION_THRESHOLD_PATH_FOLLOWING = 0.2  # [HARDCODED] Distance (meters) to consider wall blocking path (used as hardcoded 0.2 in path_following_pipeline)
OBSTACLE_AVOID_THRESHOLD = 0.25             # [USED] Distance (meters) for obstacle avoidance during exploration
OBSTACLE_AVOID_MAX_ATTEMPTS = 2             # [USED] Max attempts to avoid obstacle before giving up

# --- DWA PLANNER (Dynamic Window Approach) ---
DWA_VELOCITY_SAMPLES = [0.1, 0.15, 0.2, 0.25, 0.4, 0.45]  # [USED] Velocity samples (m/s)
DWA_ANGULAR_SAMPLES = [0, 2, 2.5, -2, -2.5, 3, -3, 3.5, -3.5, 4.5, -4.5]  # [USED] Angular velocity samples
DWA_HEADING_WEIGHT = 4.0                    # [USED] Weight for heading score in DWA
DWA_DISTANCE_WEIGHT = 3.5                   # [USED] Weight for distance score in DWA
DWA_SPEED_WEIGHT = 0.05                     # [USED] Weight for speed score in DWA
DWA_PREDICTION_DISTANCE_THRESHOLD = 0.2     # [USED] Max allowed distance increase during prediction

# --- PATH FOLLOWING ---
PATH_FOLLOWING_WAYPOINT_SAMPLE = 10         # [UNUSED] Sample path every N points for waypoints (hardcoded [::10] in path_following_pipeline)
PATH_FOLLOWING_TARGET_REACH_DISTANCE = 4    # [USED] Distance (pixels) to consider waypoint reached
PATH_FOLLOWING_STUCK_THRESHOLD_STEPS = 50  # [UNUSED] Consider stuck after N steps (hardcoded 50 in path_following_pipeline)
PATH_FOLLOWING_STUCK_THRESHOLD_DIST = 0.01  # [UNUSED] Minimum movement (meters) to reset stuck counter (hardcoded 0.01 in path_following_pipeline)

# --- MOTION CONTROL ---
MOTOR_VELOCITY_FORWARD = 2                 # [USED] Forward velocity during random exploration
MOTOR_VELOCITY_TURN = 2                     # [HARDCODED] Turning velocity (used as hardcoded 8 in turn methods)
MOTOR_VELOCITY_BACKWARD = -2                # [HARDCODED] Backward velocity (used as hardcoded -8 in recovery)
RED_WALL_ALIGNMENT_SPEED = -2.0             # [USED] Speed (m/s) for red wall alignment

# --- PID/ALIGNMENT CONTROL ---
ALIGN_RED_WALL_KP = 0.008                   # [USED] Proportional gain for red wall alignment
ALIGN_RED_WALL_KD = 0.002                   # [USED] Derivative gain for red wall alignment
ALIGN_RED_WALL_ERROR_THRESHOLD = 5          # [USED] Error threshold for red wall alignment
ALIGN_COLUMN_KP = 0.008                     # [USED] Proportional gain for column alignment
ALIGN_COLUMN_KD = 0.002                     # [USED] Derivative gain for column alignment
ALIGN_COLUMN_ERROR_THRESHOLD = 20           # [USED] Error threshold for column alignment
ALIGN_COLUMN_FORWARD_SPEED = 2.0            # [USED] Forward speed during column alignment
ALIGN_PATH_ANGLE_THRESHOLD = 15             # [UNUSED] Angle threshold (degrees) for path alignment (defined but not used in align_to_path method)
ALIGN_PATH_CLEAR_DISTANCE = 0.6             # [UNUSED] Required clear distance for rotation without backup (defined but not used)
ALIGN_PATH_BACK_DISTANCE = 0.18             # [UNUSED] Backup distance if not enough clear space (defined but not used)
ALIGN_PATH_ROTATION_SPEED = 1.0             # [UNUSED] Angular speed (rad/s) for path alignment (defined but not used)

# --- CLOSURE MARKING ---
CLOSURE_MARK_COOLDOWN = 5.0                 # [UNUSED] Cooldown (seconds) between closure markings (used as hardcoded in mark_closure_rect_simple)
CLOSURE_MARK_FORWARD = 0.8                  # [USED] Forward distance (meters) for closure rectangle
CLOSURE_MARK_BACKWARD = -0.4                # [USED] Backward distance (meters) for closure rectangle
CLOSURE_MARK_WIDTH = 0.8                # [USED] Width (meters) of closure rectangle
CLOSURE_MARK_IOU_THRESHOLD = 0.4            # [UNUSED] IoU threshold to skip re-marking same closure (hardcoded 0.4 in mark_closure_rect_simple)

# --- COLOR DETECTION ---
COLOR_DETECTION_DEPTH_THRESHOLD = 80        # [USED] Depth threshold (cm) for column detection
COLOR_DETECTION_RED_PIXEL_RATIO = 0.4      # [USED] Pixel ratio threshold for red wall detection
RED_WALL_HSV_LOWER1 = [0, 120, 70]    # [USED] Red color lower bound 1 (HSV)
RED_WALL_HSV_UPPER1 = [10, 255, 255]  # [USED] Red color upper bound 1 (HSV)
RED_WALL_HSV_LOWER2 = [170, 120, 70]  # [USED] Red color lower bound 2 (HSV)
RED_WALL_HSV_UPPER2 = [180, 255, 255] # [USED] Red color upper bound 2 (HSV)
BLUE_HSV_LOWER = [100, 150, 50]       # [USED] Blue color lower bound (HSV)
BLUE_HSV_UPPER = [140, 255, 255]      # [USED] Blue color upper bound (HSV)
YELLOW_HSV_LOWER = [20, 100, 100]     # [USED] Yellow color lower bound (HSV)
YELLOW_HSV_UPPER = [35, 255, 255]     # [USED] Yellow color upper bound (HSV)
GREEN_HSV_LOWER = [36, 100, 100]      # [USED] Green color lower bound (HSV)
GREEN_HSV_UPPER = [86, 255, 255]      # [USED] Green color upper bound (HSV)
GREEN_CARPET_DILATION_KERNEL_SIZE = 10     # Kernel size for expanding green pixels
GREEN_CARPET_DILATION_ITERATIONS = 1      # Number of dilation iterations

# --- TURNING & TURNING COOLDOWN ---
TURN_ROTATION_COOLDOWN_STEPS = 10           # [UNUSED] Steps cooldown after turning stops (used as hardcoded 10 in is_turning method)
TURN_DURATION_MIN = 50                      # [USED] Minimum turn duration (milliseconds)
TURN_DURATION_MAX = 200                     # [USED] Maximum turn duration (milliseconds)
TURN_ANGLE_COMPLETION_THRESHOLD = 0.05      # [USED] Angle threshold (radians) to complete turn

# --- SENSOR & DISTANCE ---
LIDAR_FRONT_CONE_ANGLE = 15                 # [USED] Angle (degrees) for front distance detection cone
LIDAR_FALLBACK_FRONT_DISTANCE = float('inf') # [UNUSED] Fallback distance if lidar unavailable (not referenced in code)
TURNING_THRESHOLD = 0.01                    # [UNUSED] Speed difference to detect turning (not referenced in code, similar logic hardcoded as 0.01)

# --- MAPPING & GRID ---
MAPPING_LOG_ODDS_FREE = -0.36               # [UNUSED] Log-odds value for free space (hardcoded 0.85 in bresenham_to_obstacle_score)
MAPPING_LOG_ODDS_OCCUPIED = 0.85            # [UNUSED] Log-odds value for occupied space (hardcoded 0.85 in bresenham_to_obstacle_score)
MAPPING_LOG_ODDS_CLIP_MIN = -5              # [UNUSED] Minimum log-odds clipping value (hardcoded -5 in update_grid_map)
MAPPING_LOG_ODDS_CLIP_MAX = 5               # [UNUSED] Maximum log-odds clipping value (hardcoded 5 in update_grid_map)
MAPPING_PROBABILITY_OBSTACLE = 0.7          # [UNUSED] Probability threshold for obstacle (hardcoded 0.7 in update_grid_map)
MAPPING_PROBABILITY_FREE = 0.5              # [UNUSED] Probability threshold for free space (hardcoded 0.5 in update_grid_map)

# --- A* PATHFINDING ---
ASTAR_INFLATION_LEVELS = [4, 3, 2]   # [USED] Inflation levels for escalating A*
ASTAR_EXPANSION_PIXELS = 3                # [USED] Expansion around start/end points
ASTAR_FRONTIER_INFLATION = 4          # [USED] Inflation level for frontier pathfinding

# ============================================================================
# NAVIGATION / MISSION HYPERPARAMETERS (single-threaded architecture)
# ============================================================================

# --- REACTIVE SAFETY (meters) ---
SAFE_STOP_DIST_M = 0.13          # Emergency stop distance from lidar front cone
SAFE_RANGE_STOP_M = 0.06         # Emergency stop distance from front range sensors (fl/fr)

# --- RANGE SENSOR FREE-SPACE DENOISING ---
SENSOR_FREE_THRESHOLD_M = 0.20       # Reading beyond this = sensor sees free space
SENSOR_LOCAL_ANGLES_DEG = (45, 135, -45, -135)  # fl, rl, fr, rr, relative to heading
SAFE_FRONT_CONE_DEG = 35         # Half-angle of the lidar front safety cone (degrees)
EMERGENCY_BACKUP_MS = 500        # Backup duration after an emergency stop (milliseconds)

# --- PATH FOLLOWING ---
NAV_WAYPOINT_STRIDE = 2          # Sample every Nth path point as a waypoint
NAV_WAYPOINT_REACH_PX = 6        # Distance (pixels) to consider a waypoint reached
NAV_GOAL_REACH_PX = 4            # Distance (pixels) to consider the final goal reached
NAV_MAX_REPLANS = 50              # Max plan/replan attempts per navigation goal
NAV_MAX_STEPS_PER_GOAL = 60000    # Hard cap on control steps per navigation goal
NAV_MAP_UPDATE_EVERY = 2        # Only consider a map update every Nth control step (tick throttle)
NAV_MAP_UPDATE_MIN_DIST_M = 0.05 # Only fold a scan into the map after the robot has travelled at
                                 # least this far since the last update. Gating on a small STABLE
                                 # distance (rather than every tick) stops noisy/unstable readings
                                 # from being added while the robot maneuvers in tight passages.
NAV_STUCK_DIST_M = 0.05          # Movement (meters) below which progress is "stuck"
NAV_STUCK_PATIENCE = 8          # Control ticks without progress before declaring stuck

# --- PILLAR DETECTION / LOCALIZATION ---
COLUMN_MIN_PIXELS = 120          # Minimum colored pixels to accept a pillar detection
COLUMN_MAX_LOCALIZE_M = 3.2      # Max depth (meters) at which a pillar is localized
COLUMN_REACH_DIST_M = 0.55       # (unused) legacy lidar reach distance; reach now uses map coords
COLUMN_REACH_PX = 4             # Map-distance (pixels) at which a KNOWN pillar counts as reached
COLUMN_REFINE_MIN_PIXELS = 2400   # Pixel count above which a pillar estimate is refined

# --- GREEN (POISON) HAZARD ---
GREEN_AHEAD_MIN_PIXELS = 600     # Green pixels in the lower-central view to react to
GREEN_STOP_DEPTH_M = 0.6         # Depth (meters) below which green ahead triggers avoidance
GREEN_MARK_MIN_POINTS = 20        # Minimum projected points to stamp green onto the grid

# --- EXPLORATION ---
EXPLORE_MAX_CYCLES = 120         # Max frontier-selection cycles before giving up
EXPLORE_MIN_FRONTIER_CELLS = 15  # Minimum frontier cluster size to target
INITIAL_SCAN_SPEED = 3.0         # Wheel speed used for the initial in-place 360 scan
CAMERA_HEIGHT_M = 0.11           # Rosbot RGB camera height above ground (green projection)

# --- SENSOR RANGE GATING (accuracy) ---
# Readings acquired far away are unreliable, so obstacles/points beyond this
# range are ignored for mapping.  Exposed here so it can be tuned manually.
LIDAR_MAX_RANGE_M = 3.0          # Ignore lidar returns farther than this (metres)

# --- SLOWER, MORE ACCURATE INITIAL MAPPING ---
INITIAL_SCAN_SLICES = 8         # Number of discrete turn steps for the 360 scan (finer = slower/cleaner)
INITIAL_SCAN_SETTLE_STEPS = 4    # Stationary steps to let the lidar settle before each map update
INITIAL_SCAN_TURN_SPEED = 4.0    # Wheel speed for the initial-scan in-place turns (lower = slower/cleaner)

# --- RECOVERY (prefer replanning / rotating over driving in reverse) ---
RECOVERY_ALLOW_REVERSE = True   # If False the robot never reverses to avoid obstacles; it replans/rotates
NUDGE_ROTATE_DEG = 40            # In-place rotation used to change viewpoint when boxed in
FACE_TARGET_ANGLE_TOL_RAD = 0.15 # Rotate-in-place until heading error to the path is below this

# --- DEPTH-CAMERA OBSTACLE LAYER (flat-on-floor & floating walls) ---
# The horizontal lidar scans a single height, so it cannot see walls lying
# flat on the floor or floating walls that miss its scan plane.  The depth
# camera is projected into the ground plane to catch these.
DEPTH_OBST_SAMPLE_STRIDE = 1     # Subsample the depth image every Nth pixel (speed)
DEPTH_OBST_MIN_M = 0.01          # Ignore returns closer than this (self / noise)
DEPTH_DEAD_ZONE_PIXELS = 50      # Ignore pillar detection too close
DEPTH_OBST_MAX_M = 1.5         # Ignore returns farther than this (far data is unreliable)
DEPTH_COLLISION_Z_MIN = 0.001     # Min height (m) above floor to count as an obstacle
DEPTH_COLLISION_Z_MAX = 0.23     # Max height (m); above this the robot passes under
DEPTH_SMALL_WALL_MAX = 0.15        # Max height (m) to be considered as a small wall
DEPTH_FLOATING_Z_MIN = 0.05      # If a cell's LOWEST return is above this -> floating wall
DEPTH_OBST_MIN_POINTS = 3        # Min projected points per cell to accept it (noise gate)
DEPTH_FRONT_STOP_M = 0.35        # Emergency-brake distance for depth obstacles ahead

# ============================================================================
# END HYPERPARAMETERS
# ============================================================================