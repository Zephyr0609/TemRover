# TEMRover

Autonomous navigation for the TEMRover: an AgileX Scout 2.0 rover that tows the EM Loupe Tx and Rx
carts along survey lines. ROS 2 Humble, Python, Gazebo Fortress for simulation.

The rover drives each survey line, turns, and drives the next one. It uses RTK GNSS for position,
wheel odometry and an IMU for motion, and a LiDAR to stop for and drive around obstacles. A web
dashboard shows the map and has the Start, Pause and Emergency stop buttons.

## Terms

| Term | Meaning |
|---|---|
| rover | The Scout 2.0 |
| Tx and Rx carts | The two EM Loupe carts towed behind the rover |
| survey line | One straight line of the survey grid |
| spot turn | The rover turns in place (rover only) |
| U-turn | The rover drives a curve to the next survey line (needed with the carts) |
| site file | A YAML file with the position, direction and size of one survey grid |
| navigator | The node that decides where to drive (`survey_navigator`) |
| dashboard | The web page for the operator |

## How it works

```
RTK GNSS ──┐
odometry ──┤                         ┌─> /cmd_vel ─> Gazebo, or CAN bridge ─> rover
IMU ───────┼─> navigator (20 Hz) ────┤
LiDAR ─────┘                         └─> status, paths, obstacles ─> dashboard
dashboard buttons ─> navigator
```

1. The site file gives the survey grid. The navigator builds a mission: survey lines joined by
   spot turns or U-turns.
2. The pose estimator combines GNSS, odometry and IMU into position and heading.
3. The navigator follows the mission with pure pursuit: it steers towards a point 1.5 m ahead on the path.
4. LiDAR returns on the path make the rover detour, or stop if there is no room.
5. After each turn the rover stops at the start of the next survey line and waits for Start.

The same navigator runs in simulation and on the rover.

## Install

Ubuntu 22.04 with ROS 2 Humble.

```bash
sudo apt install ros-humble-ros-gz ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-rosbridge-server ros-humble-web-video-server ros-humble-usb-cam \
  ros-humble-nmea-navsat-driver
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Zephyr0609/TemRover.git
cd ~/ros2_ws && colcon build --symlink-install
```

On the rover (LattePanda) the repository is in `~/workspace/TemRover` and is built in `~/workspace`,
on top of the team's `~/ros2_ws`. That workspace provides the LiDAR driver (`sllidar_ros2`) and
`rtk_gps_bridge.py`; neither is in this repository.

```bash
cd ~/workspace && source ~/ros2_ws/install/setup.bash && colcon build --symlink-install
pip install icm20948      # only for the IMU
```

## Run the simulation

```bash
cd ~/ros2_ws/src/TemRover/scout_navigation
./run.sh sim
```

Gazebo opens with the rover and the test obstacles. Open the dashboard at <http://localhost:8000>.
The rover starts by itself. At the start of each new survey line it waits: press **Start line N**.

| Argument | Default | Meaning |
|---|---|---|
| `site:=royal_park.yaml` | `survey.yaml` | Site file from `config/` |
| `carts:=true` | `false` | Tow the Tx and Rx carts (U-turns instead of spot turns) |
| `obstacles:=false` | `true` | Leave out the test obstacles |
| `world:=royal_park.sdf.xacro` | `flat_field.sdf.xacro` | Gazebo world from `worlds/` |
| `gz_args:='-r -s -v 1'` | `-r -v 2` | `-s` runs Gazebo without a window |

Example, Royal Park with the carts:

```bash
./run.sh sim world:=royal_park.sdf.xacro site:=royal_park.yaml obstacles:=false carts:=true
```

## Run on the rover

1. Put the rover at the start of line 1, facing along the line. With the carts, start `headland`
   metres (see `config/payload.yaml`) before the line so the carts are straight when the line begins.
2. Keep the RC remote in hand. It always overrides the navigator.
3. Start everything with one command:

```bash
cd ~/workspace/TemRover/scout_navigation
ROS_WS=~/workspace ./run.sh field site:=concrete_lawn.yaml
```

4. Open the dashboard at `http://<rover-ip>:8000` and wait for the GNSS light.
5. Press **Start survey**. Press **Start line N** at each new survey line.

| Argument | Default | Meaning |
|---|---|---|
| `site:=name.yaml` | `survey.yaml` | Site file from `config/` |
| `payload:=survey.yaml` | `payload.yaml` | Rover only, no carts |
| `imu:=false` | `true` | No IMU. Also set `dead_reckoning: true` in the site file |
| `lidar:=false` | `true` | No LiDAR, so no obstacle avoidance |
| `camera:=false` | `true` | No camera |
| `gnss:=false` | `true` | Do not start the RTK bridge |
| `lidar_port:=/dev/ttyUSB0` | `/dev/ttyUSB0` | LiDAR serial port |
| `ntrip_mount:=RTXNet-RTCM32` | `RTXNet-RTCM32` | RTK correction stream |

`run.sh field` brings up `can0`, then starts the navigator, CAN bridge, IMU driver, RTK GNSS, LiDAR
and dashboard. Do not also start RTK GPS from the team's dashboard (port 5000): only one program
can use the GNSS serial port.

### Dashboard

| Control | Effect |
|---|---|
| Start survey / Start line N | The rover drives |
| Pause | The rover stops and keeps its progress |
| Emergency stop | The rover stops. Resume resets the survey to line 1 |

The map shows the survey lines, the planned turns (dashed), the tracks of the rover and the carts,
and the obstacles the LiDAR sees. Cart positions come from a model of the linkages, not from sensors.

### New site

Copy `config/field_today.yaml` and set:

| Parameter | Meaning |
|---|---|
| `origin_latitude`, `origin_longitude` | Start of line 1 |
| `grid_bearing` | Direction of line 1, radians anticlockwise from east |
| `grid_length` | Length of each survey line (m) |
| `grid_width`, `line_spacing` | Lines = width / spacing + 1. Negative spacing puts line 2 on the right |
| `anchor_to_origin: true` | Lay the grid on the coordinates, not on where the rover starts |

Rebuild (`colcon build --symlink-install`) so the new file is installed, then pass `site:=<file>`.

## Files

Everything is in `scout_navigation/` unless noted.

### Nodes and libraries: `scout_navigation/`

| File | What it does |
|---|---|
| `survey_navigator.py` | The navigator node. Follows the mission, handles obstacles, Pause and Emergency stop, publishes `/cmd_vel` and the dashboard status |
| `pose_estimator.py` | Kalman filter for position and heading from GNSS, odometry and IMU |
| `survey_grid.py` | Builds the mission: survey lines, spot turns, U-turns |
| `obstacle_monitor.py` | Turns a LiDAR scan into obstacle points: removes ground, keeps a 60° cone ahead |
| `detour.py` | Path around an obstacle and back to the survey line |
| `geodesy.py` | Latitude/longitude to local east/north metres |
| `scout_can_bridge.py` | Rover only. `/cmd_vel` to Scout 2.0 CAN frames, chassis feedback to `/odom`. Stops the rover if commands stop arriving |
| `imu_driver.py` | Rover only. Reads the ICM-20948 IMU and publishes `imu/data` |
| `ntrip_proxy.py` | Rover only. Sits between the RTK bridge and the correction service so the corrections arrive intact |
| `survey_marker.py` | Simulation only. Draws the survey lines and their ends in Gazebo |

### Launch files: `launch/` and `run.sh`

| File | What it starts |
|---|---|
| `run.sh` | One command per run: `./run.sh sim` or `./run.sh field` |
| `sim.launch.py` | `gazebo.launch.py` + `dashboard.launch.py` |
| `gazebo.launch.py` | Gazebo world, rover model, obstacles, ROS bridge, navigator |
| `field.launch.py` | `hardware.launch.py` + `dashboard.launch.py` + RTK GNSS + LiDAR |
| `hardware.launch.py` | Navigator, CAN bridge, IMU driver |
| `dashboard.launch.py` | rosbridge (9090), video server (8080), web page (8000) |

### Parameters: `config/`

| File | Contents |
|---|---|
| `survey.yaml` | All defaults: speed, obstacle distances, LiDAR geometry, filter noise. Every parameter is commented |
| `payload.yaml` | Tx and Rx carts: linkage geometry, U-turn radius, run past each line end |
| `rover.yaml` | Real rover only: sensor mounting (the LiDAR faces backwards) |
| `royal_park.yaml`, `south_lawn.yaml`, `concrete_lawn.yaml` | Site files |
| `field_today.yaml` | Site file template for a short field test |
| `field_2026_09_21.yaml` | Site file of the 21 Sep field test, with the settings used without RTK |
| `obstacles.yaml` | The test obstacles of the simulation |
| `survey.rviz` | RViz view of the same data as the dashboard |

Parameters are layered: `survey.yaml`, then `rover.yaml` (rover only), then `payload.yaml`, then the site file.

### Other folders

| Path | Contents |
|---|---|
| `web/index.html` | The dashboard page (`roslib.min.js` is its ROS library) |
| `description/` | Models added to the Scout: sensors (`temrover_sensors.xacro`), Tx and Rx carts (`temrover_carts.xacro`), and `temrover_extras.xacro` which includes both |
| `worlds/` | Gazebo worlds (`flat_field`, `rough_field`, `royal_park`) and the test obstacles |
| `resource/` | Surveyed grid points of South Lawn |
| `results/` | Figures and videos from tests |
| `test/` | Standard ROS style checks |
| `research_log.md` | What was tried, what was found, and why each design choice was made |
| `architecture.zh.md`, `research_log.zh.md` | Chinese versions |
| `../scout_description/` | AgileX's Scout 2.0 model, adapted for Gazebo Fortress |

### Tools: `experiments/`

Run from `scout_navigation/` with `PYTHONPATH=.:experiments python3 experiments/<file>`.

| File | Purpose |
|---|---|
| `turn_design.py` | Linkage angles of the carts through a U-turn, for choosing the turn radius |
| `detour_run.py` | Records a simulated run through the test obstacles and scores it |
| `compare_planners.py` | Overlays two recorded runs |
| `bench_scan.py` | Fake LiDAR for testing detours with the rover on blocks |
| `can_loopback_test.py` | Tests the CAN bridge on a virtual CAN interface |
| `rover_nudge.py`, `spin_test.py` | Rover calibration: straight drive and spot turn |
| `make_royal_park_world.py` | Builds the Royal Park world from the drone survey |
| `make_terrain.py` | Builds the uneven ground of `rough_field` |
| `make_avoidance_figures.py` | Draws the obstacle avoidance diagrams in `results/figures/` |
| `record_video.py`, `compose_video.py` | Record the simulation camera; join two videos side by side |
| `survey_config.py` | Reads the parameter files for the tools above |

## Known limits

- In the field the rover weaves 0.3–0.5 m around the survey line. Heading is the cause: the IMU is
  not fitted yet and wheel odometry slips on grass.
- U-turns with the Tx and Rx carts work in simulation only; they are not tested in the field.
- The rover must start facing along line 1. It cannot drive itself to the start.
