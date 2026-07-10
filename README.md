# Autonomous Navigation in Webots

## Overview

This project implements an autonomous navigation pipeline for the **RosBot** mobile robot in **Webots**. The controller is designed for the Autonomous Robots graded exercise and is tested on five separate maze worlds.

The submitted project is organised so that **each maze has its own Webots world folder and its own controller files**. This makes it possible to tune constants or logic separately for each maze if required.

## Project goal

For each of the five maze environments, the robot must:

- Start the simulation from the given robot position.
- Search for and reach the **blue pillar** first.
- After reaching the blue pillar, navigate to the **yellow pillar**.
- Avoid driving over the **green ground**.
- Handle narrow passages, blocked/too-narrow passages, and floating walls.
- Run only with the allowed onboard sensors. The Webots Supervisor is not used.

## Key approaches

The controller is split into small modules so that mapping, perception, planning, navigation, and mission control are easier to understand and maintain.

### 1. Sensor setup and low-level robot control

`setup.py` initialises the Webots devices: wheel motors, wheel encoders, RGB camera, depth camera, lidar, IMU, compass, and distance sensors.

`my_robot.py` wraps the RosBot hardware interface. It handles motor velocity commands, odometry, compass-based heading, coordinate conversion between world and grid map, lidar point-cloud conversion, and the local DWA movement step.

### 2. Perception

`perception.py` reads camera and depth data and converts it into useful information for navigation:

- HSV colour segmentation for blue, yellow, green, and red objects.
- Blue and yellow pillar detection and localisation using RGB-D data.
- Green ground detection and projection into map coordinates.
- Front obstacle distance estimation using lidar, distance sensors, and depth camera.
- Depth-camera obstacle detection for flat or floating walls that may be missed by the horizontal lidar.

### 3. Mapping

`map.py` maintains an occupancy grid map. Lidar scans are converted into map cells using the robot odometry and compass heading. The map stores free cells, unknown cells, obstacles, green hazard cells, blue/yellow pillar cells, and depth-only floating-wall cells.

Frontier detection is used during exploration. Frontier regions are free cells next to unknown space, so the robot can choose useful unexplored targets.

### 4. Path planning

`astar_2_spline.py` runs a clearance-aware A* search on the occupancy grid. The planner inflates obstacles to maintain a safety margin and then smooths the path using spline-based smoothing.

`map.py` provides two planning modes:

- `find_path()` for final goal navigation to the blue/yellow pillar.
- `find_path_for_frontier()` for exploration targets.

### 5. Navigation and local path following

`navigation.py` owns the plan-follow-replan loop. It plans a global path, downsamples it into waypoints, faces the path direction, and follows the path using a local DWA controller.

During movement, the navigator continuously checks for:

- Obstacles in front of the robot.
- Green ground close ahead.
- Depth-camera obstacles.
- Stuck situations.

If a problem is detected, the robot stops, updates the map, rotates in place if needed, and replans a new route.

### 6. Mission control

`mission.py` implements the high-level task flow:

1. Perform an initial 360-degree scan.
2. Explore the unknown map using frontier targets.
3. Localise both pillars when they become visible.
4. Drive to the blue pillar first.
5. Drive from the blue pillar to the yellow pillar.
6. Stop after reaching the yellow pillar.

`main.py` is the controller entry point. It creates the robot, perception, navigator, and mission objects, then starts the mission.


## Installation requirements

Install the following software before running the simulation:

- Webots
- Python 3.x
- Python packages:
  - `numpy`
  - `opencv-python`
  - `scipy`
  - `pygame`

Install the Python dependencies with:

```bash
pip install -r requirements.txt
```

The Webots `controller` Python module is provided by Webots. It is not installed using `pip`.

## How to run the project

### Step 1: Open a maze world

Open one of the world files in Webots, for example:

```text
AutonomousRobots_Submission/Maze1/worlds/Maze1.wbt
```

Repeat the same process for `Maze2.wbt`, `Maze3.wbt`, `Maze4.wbt`, and `Maze5.wbt`.

### Step 2: Check the controller path

For each world, make sure the robot uses the controller folder inside the same maze project:

```text
MazeX/controllers/main/
```

In Webots, select the robot node and check the `controller` field. Set it to the controller folder name used in your project, for example:

```text
main
```

If your local Webots setup uses a controller folder named `main`, then set the controller field to `main` .

### Step 3: Run the simulation

Press **Run** in Webots. The controller will:

1. Initialise sensors and motors.
2. Build an initial local map.
3. Explore the environment.
4. Detect and localise the blue and yellow pillars.
5. Navigate to the blue pillar.
6. Navigate from blue to yellow.
7. Stop after finishing the mission.

A live map visualisation window may open using `pygame`. It shows explored space, obstacles, frontiers, the planned path, and detected pillar positions.

## Results

Demo video can be seen [here](https://www.youtube.com/playlist?list=PLPnj5xhKrn-I)

| Maze | Start simulation → Blue pillar reached | Blue pillar reached → Yellow pillar reached | Total time |
| ---- | -------------------------------------- | ------------------------------------------- | ---------- |
| Maze 1 | 03:26 | 01:11 | 04:37 |
| Maze 2 | 02:15 | 01:25 | 03:40 | 
| Maze 3 | 01:40 | 01:46 | 03:26 | 
| Maze 4 | 01:40 | 00:50 | 02:30 | 
| Maze 5 | 02:25 | 00:50 | 03:15 | 

## Main files

| File | Purpose |
| ---- | ------- |
| `main.py` | Starts the controller and mission. |
| `setup.py` | Enables motors, sensors, cameras, lidar, and IMU devices. |
| `my_robot.py` | Handles robot hardware interface, odometry, motion primitives, coordinate conversion, lidar mapping, and DWA local movement. |
| `perception.py` | Detects pillars, green ground, red walls, lidar obstacles, and depth-camera obstacles. |
| `navigation.py` | Executes global planning, local following, safety checks, stuck recovery, and replanning. |
| `mission.py` | Controls the complete task: scan, explore, localise pillars, go to blue, then go to yellow. |
| `map.py` | Maintains the occupancy grid, frontier detection, obstacle stamping, and path planning adapters. |
| `astar_2_spline.py` | Runs A* path search and smooths the generated path. |
| `utils.py` | Contains helper functions for angle wrapping, colour segmentation, obstacle inflation, Bresenham lines, and map saving. |
| `CONSTANTS.py` | Stores tuning parameters, map values, thresholds, sensor limits, and navigation constants. |

## Notes for submission

Before uploading, check that:

- Each maze folder contains its own `worlds` and `controllers` folders.
- Each maze has the correct `.wbt` file inside its `worlds` folder.
- Each maze has a full copy of the controller files inside its controller folder.
- No `.git`, `.gitignore`, `__pycache__`, `.pyc`, debug cache, or temporary folders are included.
- The README is included at the root of the submission.
- The timing table is updated with the final measured simulation times.
- The video recordings show the simulation time clearly.

## Contributors

| Name | Github |
| ---- | ---- |
| Nishith Kumar Alungandula | [nishithkumar99](https://github.com/nishithkumar99) |
| Sathwik Nagasundra Sharma | [SathwikSharma226](https://github.com/SathwikSharma226) |
| Siddhant Tiwari | [Siddhantst08](https://github.com/Siddhantst08) |
| Zicheng Cai | [ZichengCai14ef](https://git.oth-aw.de/14ef) |
