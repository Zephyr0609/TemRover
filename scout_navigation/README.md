# scout_navigation

Autonomous grid-survey navigation for the Scout 2.0 rover towing the EM Loupe — TEMRover project.

## Run

```
colcon build --symlink-install && source install/setup.bash
ros2 launch scout_navigation gazebo.launch.py                        # Gazebo Fortress
ros2 launch scout_navigation gazebo.launch.py gz_args:='-r -v 2 -s'  # Gazebo, headless
ros2 launch scout_navigation hardware.launch.py                      # real Scout over CAN
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

Hardware needs SocketCAN up and the transmitter in command mode:

```
sudo ip link set can0 up type can bitrate 500000
```

## Experiments

```
PYTHONPATH=. python3 experiments/run_experiments.py   # controller and receiver-grade study, writes results/
```

Findings and open questions are tracked in `research_log.md`, with a Chinese copy in
`research_log.zh.md`. `architecture.zh.md` explains what every file does and how the
pieces fit together, in plain language.
