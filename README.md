# robocup_ws

Isaac Sim simulation of the MRVK robot "shoddy" (4-wheel skid steer, 2D lidar)
in the RoboCup@Home HCC arena, with an optional Nav2 navigation stack.
ROS 2 Humble · Isaac Sim 6.2 · Ubuntu 22.04

## Requirements

- Ubuntu 22.04 with **ROS 2 Humble** (`/opt/ros/humble`)
- **Isaac Sim 6.2** standalone, by default in `~/isaacsim`
  (otherwise `export ISAACSIM_PATH=/path/to/isaacsim`)
- An NVIDIA GPU that runs Isaac Sim (tested on an RTX 3080 Ti Laptop, 16 GB)

## Setup

```bash
git clone <this repo> ~/robocup_ws
cd ~/robocup_ws
sudo apt install python3-rosdep ros-humble-libg2o
sudo rosdep init   # only once per machine
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro humble
```

The workspace is built automatically on the first `./run.sh`
(or explicitly with `./run.sh --build`).

## Running

```bash
./run.sh                        # arena + robot, RViz, steering GUI
./run.sh --nav                  # + Nav2 and slam_toolbox (TEB local planner)
./run.sh --nav --planner dwb    # local planner: teb (default) | dwb | tr
./run.sh --headless             # Isaac without a window (RViz still opens)
./run.sh --x 2 --y -1 --yaw 1.57    # spawn pose
./run.sh --world other.usd      # another USD environment
./run.sh --build                # rebuild the workspace first
```

`run.sh` starts Isaac Sim, waits until it publishes `/clock` (the first start
compiles shaders and can take a few minutes), then starts the ROS 2 side.
Ctrl+C, or closing either side, stops both.

With `--nav`, give the robot a goal with **2D Goal Pose** in RViz. Driving by
hand works through the rqt_robot_steering window (it can stay open next to
Nav2; it sends one zero when released).

### Environment

Both sides run in a clean environment containing only `/opt/ros/humble` and
this workspace, so a shell sourced for another workspace cannot leak in:

- Isaac Sim's Python 3.12 breaks if ROS's Python 3.10 paths are present.
- Isaac's ROS 2 bridge uses **Fast DDS**; the ROS side is pinned to
  `rmw_fastrtps_cpp`. To inspect topics from your own terminal:
  `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.

## ROS interface

| Topic / frame | Direction | Notes |
|---|---|---|
| `/clock` | Isaac → | simulation time, everything runs with `use_sim_time` |
| `/odom`, TF `odom → base_link` | Isaac → | exact pose from the physics (no drift) |
| `/scan_raw` | Isaac → | RTX lidar, 360°, 0.1°, 10 Hz, range 0.15–4 m, frame `lidar_frame` |
| `/scan` | relay → | `/scan_raw` with no-hit beams turned from 0.0 into `inf` |
| `/cmd_vel` | → Isaac | `geometry_msgs/Twist` |
| TF `base_link → …` | robot_state_publisher | from `robocup_description` |
| TF `map → odom`, `/map` | slam_toolbox (`--nav`) | visualisation only |

Nav2 navigates in the `odom` frame with rolling-window costmaps fed by `/scan`;
there is no static map or global localisation.

## Packages

- `src/robocup_sim` – Isaac Sim script (`isaac/robocup_isaac.py`), launch
  files, Nav2 / slam_toolbox / RViz configs, the arena (`worlds/`).
- `src/robocup_description` – URDF/xacro and meshes of shoddy.
- `src/teb_local_planner`, `src/costmap_converter` – vendored from
  [rst-tu-dortmund](https://github.com/rst-tu-dortmund) (`humble-devel` /
  `ros2`); TEB has no Humble binaries. costmap_converter includes
  `cv_bridge.h` instead of `.hpp` to build against Humble's cv_bridge.

## Simulation notes

- **Kinematic base.** `/cmd_vel` is applied directly as the chassis velocity
  (acceleration-limited to 2 m/s²) and the chassis is kept level; the wheels are
  frictionless. The robot still collides with the world. A PhysX 4-wheel skid
  steer with this URDF chattered, flipped and could not turn on the spot.
- **Arena.** The USD (Blender export, Y-up, centimetres) is turned Z-up, scaled
  to metres and centred on the origin. Every mesh gets a triangle-mesh
  collider; the two sliding doors get box colliders instead. The ceiling is
  hidden. Spawn (0, 0) is free floor.
- **Lidar** stands in for the robot's Hokuyo URG-04LX (which scanned 240° with
  1 cm noise; the simulated one scans 360° without noise or motion distortion).
  It sees rendered geometry, not colliders.
- Footprint for Nav2 is the rectangle over the bumpers and wheels,
  0.56 × 0.47 m.
