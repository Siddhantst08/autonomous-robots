import heapq
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.interpolate import CubicSpline, splprep, splev

# Keep your original constants
MAX_COST = 99999

def runAStarSearch(global_map, start_coords, goal_coords):
    """
    Vectorized A* with Clearance Penalty - High Performance.
    Maintains 100% API compatibility with the original version.
    """
    rows, cols = global_map.shape
    start_x, start_y = int(start_coords[0]), int(start_coords[1])
    goal_x, goal_y = int(goal_coords[0]), int(goal_coords[1])

    # 1. Pre-calculate Clearance Penalty Map (Vectorized)
    # distance_transform_edt finds distance to nearest obstacle (0)
    dist_to_obs = distance_transform_edt(global_map == 0)
    
    # Tunable parameters for "pushing" away from walls
    SAFE_DISTANCE = 5.0  # pixels
    PENALTY_STRENGTH = 2.0
    penalty_map = np.where(dist_to_obs < SAFE_DISTANCE, 
                           PENALTY_STRENGTH * (1.0 - dist_to_obs / SAFE_DISTANCE), 
                           0.0).astype(np.float32)

    # 2. Initialize A* structures (NumPy arrays for speed)
    g_costs = np.full((rows, cols), np.inf, dtype=np.float32)
    parent_map = np.full((rows, cols, 2), -1, dtype=np.int32)
    
    g_costs[start_y, start_x] = 0
    
    # Priority Queue: (f_cost, y, x)
    # Heuristic: Euclidean distance * 1.2 (matching your original code)
    h_start = np.sqrt((start_x - goal_x)**2 + (start_y - goal_y)**2) * 1.2
    pq = [(h_start, start_y, start_x)]
    
    # 8-connectivity offsets (dy, dx, distance)
    # Using 2-pixel steps to match your original getNeighbors logic
    neighbors = [
        (0, 2, 2.0), (0, -2, 2.0), (2, 0, 2.0), (-2, 0, 2.0),
        (2, 2, 2.828), (2, -2, 2.828), (-2, 2, 2.828), (-2, -2, 2.828)
    ]

    while pq:
        f, cy, cx = heapq.heappop(pq)
        
        # Goal check: matching your isNearlySame logic (within 2 pixels)
        if abs(cx - goal_x) <= 2 and abs(cy - goal_y) <= 2:
            path = _reconstruct_path(parent_map, cy, cx)
            # Call your existing smoothing function
            return smoothPath(path, method='bspline', smoothness=0.1)

        # Skip if we found a better path already
        if f > g_costs[cy, cx] + np.sqrt((cx - goal_x)**2 + (cy - goal_y)**2) * 1.2:
            continue

        for dy, dx, step_dist in neighbors:
            ny, nx = cy + dy, cx + dx
            
            if 0 <= ny < rows and 0 <= nx < cols:
                # Check if accessible (0 is FREESPACE)
                if global_map[ny, nx] == 0:
                    # Total cost = distance + clearance penalty
                    new_g = g_costs[cy, cx] + step_dist + penalty_map[ny, nx]
                    
                    if new_g < g_costs[ny, nx]:
                        g_costs[ny, nx] = new_g
                        parent_map[ny, nx] = [cy, cx]
                        h = np.sqrt((nx - goal_x)**2 + (ny - goal_y)**2) * 1.2
                        heapq.heappush(pq, (new_g + h, ny, nx))

    return []
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting. 

def _reconstruct_path(parent_map, cy, cx):
    path = []
    curr = [cy, cx]
    while curr[0] != -1:
        path.append((curr[1], curr[0])) # (x, y)
        curr = parent_map[curr[0], curr[1]]
    return path[::-1]
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting. 

# --- THE FOLLOWING FUNCTIONS ARE KEPT FOR COMPATIBILITY ---

def smoothPath(path, method='bspline', smoothness=0.3):
    if len(path) < 2:
        return path
    path = np.array(path, dtype=float)
    if method == 'bspline':
        return _bsplineSmooth(path, smoothness)
    elif method == 'natural_spline':
        return _naturalSplineSmooth(path, smoothness)
    elif method == 'parametric_spline':
        return _parametricSplineSmooth(path, smoothness)
    # elif method == 'corner_aware':
    #     return _cornerAwareSmooth(path, smoothness)
    # elif method == 'bezier_like':
    #     return _bezierLikeSmooth(path, smoothness)
    return _bsplineSmooth(path, smoothness)
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting. 

def _naturalSplineSmooth(path, smoothness):
    x, y = path[:, 0], path[:, 1]
    t = np.arange(len(path))
    cs_x = CubicSpline(t, x, bc_type='natural')
    cs_y = CubicSpline(t, y, bc_type='natural')
    samples_per_segment = max(5, int(20 * smoothness))
    t_fine = np.linspace(0, len(path)-1, num=len(path) * samples_per_segment)
    return _finalizeSmoothedPath(cs_x(t_fine), cs_y(t_fine))
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting. 

def _parametricSplineSmooth(path, smoothness):
    if len(path) < 4: return _naturalSplineSmooth(path, smoothness)
    x, y = path[:, 0], path[:, 1]
    s_factor = len(path) * (1 - smoothness) * 10
    tck, u = splprep([x, y], s=s_factor, k=3)
    samples = len(path) * max(10, int(30 * smoothness))
    u_fine = np.linspace(0, 1, samples)
    smooth_coords = splev(u_fine, tck)
    return _finalizeSmoothedPath(smooth_coords[0], smooth_coords[1])
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting. 

# def _cornerAwareSmooth(path, smoothness):
#     # Simplified for brevity, you can paste your full version here
#     return _parametricSplineSmooth(path, smoothness)

# def _bezierLikeSmooth(path, smoothness):
#     # Simplified for brevity, you can paste your full version here
#     return path.tolist()

def _bsplineSmooth(path, smoothness):
    if len(path) < 4: return path.tolist()
    x, y = path[:, 0], path[:, 1]
    s_val = max(0.0, smoothness * len(path))
    try:
        tck, u = splprep([x, y], s=s_val, k=3)
        samples = max(10, len(path) * 8)
        u_fine = np.linspace(0, 1, samples)
        smooth_coords = splev(u_fine, tck)
        return _finalizeSmoothedPath(smooth_coords[0], smooth_coords[1])
    except:
        return path.tolist()
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting. 

def _finalizeSmoothedPath(smooth_x, smooth_y):
    smooth_path = [(int(round(x)), int(round(y))) for x, y in zip(smooth_x, smooth_y)]
    seen = set()
    unique_smooth_path = []
    for point in smooth_path:
        if point not in seen:
            seen.add(point)
            unique_smooth_path.append(point)
    return unique_smooth_path
# REFERENCE: 
# Source: Gemini 3.1 Pro with detailed prompting. 
