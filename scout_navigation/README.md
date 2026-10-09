# scout_navigation

Autonomous grid-survey navigation for the Scout 2.0 rover towing the EM Loupe — TEMRover project.

## Run

One command per experiment, from this folder:

```
./run.sh field site:=field_today.yaml     # real rover: GNSS, lidar, IMU, camera, navigator, CAN bridge, dashboard
./run.sh sim                              # Gazebo with the seven test obstacles and the dashboard
./run.sh sim site:=south_lawn.yaml gz_args:='-r -s -v 2'   # headless, another site
```

`run.sh` sources the workspace (`ROS_WS`, default `~/ros2_ws`), brings `can0` up for the field run,
prints the dashboard address and starts `field.launch.py` or `sim.launch.py`. A bare site name is
looked up in `config/`. Field switches: `imu:=false`, `gnss:=false`, `lidar:=false`, `camera:=false`,
`gnss_port:=/dev/ttyACM0`, `lidar_port:=/dev/ttyUSB0`. The GNSS driver keeps the receiver's own
configuration (`config/ublox_rover.yaml`); the ZED-F9P must already output UBX NAV-PVT on USB and
receive its RTK corrections. The lidar uses Slamtec's `sllidar_ros2` S2 launch, built on the rover.

Royal Park in Gazebo: the drone survey (classified LAZ point cloud and GDA2020 orthophoto, kept
outside the repository in `../royal_park`) is turned into a textured ground mesh, a collision mesh
and tree trunks by `experiments/make_royal_park_world.py`, which writes `worlds/royal_park/` and
`worlds/royal_park.sdf.xacro`. Run it with the surveyed grid:

```
./run.sh sim world:=royal_park.sdf.xacro site:=royal_park.yaml obstacles:=false show_path:=false
# add trees:=true to place the trees from the point cloud around the grid
```

The single launch files underneath still work on their own:

```
colcon build --symlink-install && source install/setup.bash
ros2 launch scout_navigation gazebo.launch.py                        # Gazebo Fortress
ros2 launch scout_navigation hardware.launch.py                      # navigator, CAN bridge, IMU
ros2 launch scout_navigation dashboard.launch.py                     # operator page on :8000
```

The dashboard (`web/index.html`) is the operator page: the survey lines with the driven route
and the obstacles seen, the forward camera, four numbers (line, speed, cross-track, heading) and
two buttons (start/pause, emergency stop). The map is to scale: the rover is drawn from the URDF
dimensions and the Tx/Rx carts from `temrover_carts.xacro`, placed by a kinematic trailer model
driven by the rover's pose (the carts carry no sensor). Scroll to zoom, drag to pan, double-click
to refit. `results/dashboard_zoom.png` shows the rover and train up close. Open `http://<rover-ip>:8000` from any laptop or phone
on the rover's network; add `usb_camera:=true` on the real rover and `use_sim_time:=true` alongside
the simulation. `results/dashboard.png` is a capture against the simulation. The navigator starts
in the mode given by `start_mode` (autonomous by default; set idle for the real rover so the
operator releases it from the page). A `manual` mode hands `cmd_vel` to any teleop node; the page
does not expose it.
For the engineering view, `rviz2 -d config/survey.rviz` shows the same data in rviz: robot model
on the map→base_link transform, lidar, the filtered obstacle cloud (`obstacle_points`, 60 s decay)
and the survey path.

The workspace holds two packages: `scout_navigation` and `scout_description`, the AgileX Scout V2
model with its Gazebo Classic block replaced by Fortress systems. Build from the workspace root.

### Recording a run

Launch with the GUI, then point the camera at the rover and record from the button in the top-left
corner of the 3D view:

```
ign service -s /gui/follow --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean \
  --timeout 2000 --req 'data: "temrover"'
ign service -s /gui/follow/offset --reqtype ignition.msgs.Vector3d \
  --reptype ignition.msgs.Boolean --timeout 2000 --req 'x: -12.0, y: -6.0, z: 6.0'
```

Regenerate the survey terrain after changing its parameters:

```
python3 experiments/make_terrain.py
```

The IMU driver (`imu_driver`, adapted from ej5962/Capstone) reads the ICM-20948 on the
LattePanda's I2C bus 1 and needs the Pimoroni library on the rover: `pip install icm20948`.
Keep the rover still for the first 3 s after launch while it measures the gyro bias.
`imu:=false` on `hardware.launch.py` runs without it (set `dead_reckoning: true` in the site file).

Hardware needs SocketCAN up and the transmitter in command mode:

```
sudo ip link set can0 up type can bitrate 500000
```

## Experiments

```
./run.sh sim bystander:=true &   # the corner pole that detour_run.py removes mid-run
PYTHONPATH=.:experiments python3 experiments/detour_run.py 230 results/run.png
```

Findings and open questions are tracked in `research_log.md`, with a Chinese copy in
`research_log.zh.md`. `architecture.zh.md` explains what every file does and how the
pieces fit together, in plain language.
