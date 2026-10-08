# rebel_demo — igus ReBeL + HIWIN XEG-32 hand teleoperation from OptiTrack mocap

Your hand (an OptiTrack rigid body) drives a simulated igus ReBeL 6-DoF (rebel2) in real time, and a
second rigid body opens / closes a HIWIN XEG-32 gripper. The hand pose is filtered, converted to joint
angles with IK at 60 Hz, and published as `sensor_msgs/JointState` — shown in RViz, and sent to Isaac Sim
(`/isaac_joint_commands`). Demonstrations can be recorded as a GR00T / LeRobot v2 dataset.

```
Motive (Windows) --NatNet--> mocap4r2 OptiTrack driver --/rigid_bodies-->
  mocap_tf --TF map->rigid_body_1, rigid_body_2-->
    teleop  (rigid_body_1: filter -> IK -> speed limit)   -> joint1..joint6
    gripper (|rigid_body_2 - rigid_body_1| < threshold)   -> XEG-32 open / close
  --> /joint_states  -> robot_state_publisher -> RViz
  --> /isaac_joint_commands -> Isaac Sim
  recorder: joints + gripper + camera images -> GR00T / LeRobot v2 dataset
```

## Contents
```
rebel_demo/
  rebel_demo/kinematics.py     rebel2 joint table (= igus_rebel2_arm_macro.xacro), FK, IK (damped least squares)
  rebel_demo/mocap_tf_node.py  node "mocap_tf": mocap4r2 /rigid_bodies -> TF (map -> rigid_body_<id>)
  rebel_demo/teleop_node.py    node "teleop":   hand TF -> low-pass -> clutch/scale -> IK -> speed limit -> JointState
  rebel_demo/gripper_node.py   node "gripper":  distance between two rigid bodies -> XEG-32 open (1) / closed (0)
  rebel_demo/recorder_node.py  node "recorder": joints + gripper + images -> GR00T / LeRobot v2 dataset
  rebel_demo/lerobot_writer.py dataset writer used by the recorder (parquet + H.264 mp4 + meta)
  launch/teleop.launch.py      starts mocap_tf, teleop, gripper, robot_state_publisher, static TF, (RViz)
  urdf/rebel_xeg32.urdf.xacro  robot model: rebel2 arm + XEG-32 from igus_rebel_description (same macros as Isaac)
  rviz/                        RViz config
  tools/make_low_poly_meshes.py  mesh decimation used to make igus_rebel_description/meshes_low/
  test/test_teleop.py          pure-python tests: python3 -m pytest test/
```

### Robot model
The launch file builds the robot from `urdf/rebel_xeg32.urdf.xacro`, which uses the `igus_rebel_description`
macros that the Isaac dual-arm rig uses: the rebel2 arm (joints `joint1..joint6`) and the XEG-32 mounted on
`link6` (+52 mm, +15 deg about Z), with jaw joints `xeg32_left_carriage_joint` / `xeg32_right_carriage_joint`
(right mimics left) and `xeg32_camera_tilt_joint`. `tool0` = link6 + 52.7 mm, the frame the IK works in.
Joint limits follow the igus spec sheet as in the arm macro (joint2 lower -80 deg, joint3 upper 140 deg).
`mesh_lod:=low` (default) loads the decimated `meshes_low/`, `mesh_lod:=full` the full CAD meshes.

## Requirements
- Linux PC that runs the mocap driver (the NatNet library is Linux-only), with **one** ROS 2 distribution:
  Ubuntu 22.04 + Humble, or Ubuntu 24.04 + Jazzy. The code is plain rclpy and works on both.
  Below, replace `<distro>` with `humble` or `jazzy`. Never mix the two (in terminals, workspaces or Isaac Sim).
- Python: numpy. The recorder also needs `pyarrow` (`pip install pyarrow`) and the `ffmpeg` binary.
- ROS packages: `xacro`, `robot_state_publisher`, `rviz2`, and `igus_rebel_description` (section 3).
- RViz can run on the same Linux PC (needs a screen) or on another machine (e.g. a Mac with RoboStack)
  that has this package built, same network, same `ROS_DOMAIN_ID`.

## 1. Mocap driver (separate workspace)
```bash
sudo apt install python3-rosdep python3-vcstool
mkdir -p ~/mocap_ws/src && cd ~/mocap_ws/src
git clone -b rolling https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack.git
vcs import < mocap4ros2_optitrack/dependency_repos.repos
cd ~/mocap_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```
Do not use the `OptiTrack/mocap4ros2_optitrack` fork: its dependency file points to a repository that does not exist.

Edit `~/mocap_ws/src/mocap4ros2_optitrack/mocap4r2_optitrack_driver/config/mocap4r2_optitrack_driver_params.yaml`:
```yaml
connection_type: "Multicast"        # must match Motive (Multicast or Unicast)
server_address: "<Motive PC IP>"    # the computer that SENDS
local_address: "<this Linux PC IP>" # the computer that RECEIVES
multicast_address: "239.255.42.99"
```

## 2. Motive (Windows)
Edit -> Settings -> Streaming: **Broadcast Frame Data** on, **Local Interface** = the Motive PC IP on the shared
network, **Transmission Type** = same as the YAML, **Rigid Bodies** on. The hand rigid body's **Streaming ID**
becomes the TF frame `rigid_body_<ID>`. Windows firewall: network profile Private, allow Motive.

## 3. Build this package
Get the code into the workspace `src` folder (clone it as `rebel_demo`):
```bash
mkdir -p ~/rebel_ws/src
cd ~/rebel_ws/src
git clone https://github.com/duckQuu/rebel_mocap_teleop.git rebel_demo
```
The robot model needs the `igus_rebel_description` package (rebel2 arm + XEG-32 macros, `meshes/` and
`meshes_low/`). It is **not in this repository** (vendor CAD): get it from the igus mesh LOD bundle and put it
next to `rebel_demo` (or source a workspace that already builds it). colcon does not find packages nested
inside another package, so it must sit directly in `src/`:
```bash
cp -r <igus_mesh_lod_bundle>/src/igus_rebel_description ~/rebel_ws/src/
```
Build **from the workspace root** `~/rebel_ws` — never from `src/` or `src/rebel_demo/`:
```bash
cd ~/rebel_ws
source /opt/ros/<distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
Check the layout. `build`, `install`, `log` must sit next to `src`, not inside it:
```
~/rebel_ws/
  build/  install/  log/          <- created by colcon build (run in ~/rebel_ws)
  src/
    igus_rebel_description/       <- from the igus mesh LOD bundle (not in this repo)
    rebel_demo/
      launch/  rebel_demo/  resource/  rviz/  test/  tools/  urdf/
      package.xml  README.md  setup.cfg  setup.py
```
Built in the wrong folder by mistake? Delete the stray output and rebuild from the root:
```bash
rm -rf ~/rebel_ws/src/build ~/rebel_ws/src/install ~/rebel_ws/src/log
cd ~/rebel_ws && rm -rf build install log && colcon build --symlink-install
```
When to rebuild: with `--symlink-install`, edits to existing `.py` files apply on the next launch. Rebuild after
adding or renaming files, or changing `setup.py`, `package.xml`, the URDF, meshes or launch files.
Without `--symlink-install`, rebuild after every edit.

### macOS (RoboStack / conda, e.g. env `ros2_jazzy`)
- The shell is zsh: `source install/setup.zsh`, not `setup.bash`.
- `colcon build --symlink-install` fails with `option --editable not recognized` when the env has
  setuptools >= 80. Either build without it (`colcon build`), or `pip install "setuptools<80"` first.
- After a failed symlink build, a plain build fails with `option --uninstall not recognized`.
  Delete the leftovers and rebuild: `rm -rf build/rebel_demo install/rebel_demo && colcon build`
- Install xacro into the env: `conda install -c robostack-jazzy -c conda-forge ros-jazzy-xacro`

`~/.bashrc` (every terminal must have the **same** values — and the terminal that starts Isaac Sim too):
```bash
source /opt/ros/<distro>/setup.bash
source ~/mocap_ws/install/setup.bash
source ~/rebel_ws/install/setup.bash
export ROS_DOMAIN_ID=42                         # pick a number nobody else on your network uses
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp    # or leave unset everywhere (Fast DDS) - but the same everywhere
```
(`sudo apt install ros-<distro>-rmw-cyclonedds-cpp` for Cyclone DDS.) After changing any of these, restart every
ROS program (driver, teleop, Isaac Sim) and run `ros2 daemon stop`: running programs keep their old values.

## 4. Run
```bash
# terminal 1: driver (wait for "Configured!")
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
# terminal 2: activate, check data (~120 Hz)
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
ros2 topic hz /rigid_bodies
# terminal 3, RViz only: arm + XEG gripper (hand = rigid_body_1, gripper control = rigid_body_2)
#   (add rviz:=false over SSH, then run RViz elsewhere)
ros2 launch rebel_demo teleop.launch.py target:=rviz hand_frame:=rigid_body_1 scale:=0.5 \
  gripper_frame:=rigid_body_2 gripper_threshold:=0.06 gripper_hysteresis:=0.01 mesh_lod:=low
# terminal 3, Isaac Sim instead (Isaac playing, graph subscribed to /isaac_joint_commands), gripper off:
ros2 launch rebel_demo teleop.launch.py target:=isaac hand_frame:=rigid_body_1 rviz:=false \
  scale:=1.0 cutoff_hz:=6.0 gripper:=false lock_gripper:=true
# terminal 2: engage the clutch, move your hand slowly
ros2 service call /rebel_teleop/engage std_srvs/srv/SetBool "{data: true}"
```
`{data: false}` releases the clutch: the robot holds, reposition your hand, engage again.
Gripper checks (RViz run): `ros2 run tf2_ros tf2_echo rigid_body_1 rigid_body_2` while you pinch / spread, set
`gripper_threshold` to the midpoint; `ros2 topic echo /rebel_gripper/state` shows 1 = open, 0 = closed.

RViz only, no mocap (robot at HOME, gripper open):
```bash
ros2 launch rebel_demo teleop.launch.py target:=rviz mocap_bridge:=false
```
Model viewer with joint sliders (instead of the launch above, not together with it):
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix rebel_demo)/share/rebel_demo/urdf/rebel_xeg32.urdf.xacro mesh_lod:=low)"
ros2 run joint_state_publisher_gui joint_state_publisher_gui
rviz2 -d $(ros2 pkg prefix rebel_demo)/share/rebel_demo/rviz/rebel_demo.rviz
```
Check the commands: `ros2 topic echo /isaac_joint_commands` (Isaac mode) or `ros2 topic echo /joint_states` (RViz mode).
RViz on another machine: `rviz2 -d $(ros2 pkg prefix rebel_demo)/share/rebel_demo/rviz/rebel_demo.rviz`

## How teleop works (every cycle, 60 Hz)
1. Read the hand pose from TF (`base_link <- rigid_body_1`).
2. Tracking check: no update or a frozen pose for 0.2 s -> hold; re-anchor when tracking returns (no jumps).
3. Low-pass filter on the hand pose (position: first order; orientation: slerp). Filtering before IK keeps marker jitter out of the joints.
4. Clutch mapping: target = robot pose at engage + scale x (hand now - hand at engage); clamped to a workspace box.
5. IK (damped least squares, warm-started from the last command; position only unless `orientation:=true`).
6. Per-joint speed limit (default 45 deg/s, the real ReBeL maximum).
7. Publish `sensor_msgs/JointState` (`joint1..joint6`, radians) on `command_topic` (default `/isaac_joint_commands`), and on `/joint_states` when `target:=rviz`.

## Launch arguments
| argument | default | meaning |
|---|---|---|
| target | rviz | `rviz`: teleop also publishes `/joint_states`; `isaac`: the simulator publishes them |
| hand_frame | rigid_body_1 | TF frame of the hand = `rigid_body_` + Motive streaming ID |
| mode | relative | `relative` (clutch) or `absolute` (flange goes to the hand pose) |
| scale | 1.0 | hand-to-robot motion ratio (0.5 = finer) |
| orientation | false | true = follow hand rotation too |
| cutoff_hz | 4.0 | filter cutoff; lower = smoother but more lag |
| max_joint_speed_deg | 45 | per-joint speed limit after IK |
| robot_xyz / robot_ypr | 0 0 0 | robot base pose in the mocap `map` frame (m, rad) |
| mocap_bridge | true | start the `mocap_tf` converter (`false` = no mocap needed) |
| rviz | true | start RViz here (needs a display) |
| mesh_lod | low | `low` (decimated meshes) or `full` (CAD) |
| lock_gripper | false | true = XEG joints fixed in the model |
| gripper | true | start the gripper node |
| gripper_frame | rigid_body_2 | TF frame of the gripper-control rigid body |
| gripper_ref_frame | (hand_frame) | distance is measured to this frame |
| gripper_threshold | 0.06 | distance [m] below which the gripper closes |
| gripper_hysteresis | 0.01 | dead band [m] around the threshold |
| gripper_invert | false | true = far closes, near opens |
| command_topic | /isaac_joint_commands | topic the joint commands go to; must match Isaac's ROS2 Subscribe Joint State node exactly (RViz always uses `/joint_states`) |

## Gripper (HIWIN XEG-32)
A second rigid body (`gripper_frame`, default `rigid_body_2`, e.g. on your thumb) controls the gripper. The node
measures its distance to `gripper_ref_frame` (default: the hand body) and switches between two states:
```
distance < threshold - hysteresis/2   -> CLOSE (0)
distance > threshold + hysteresis/2   -> OPEN  (1)
in between, or tracking lost          -> keep the current state
```
The dead band stops marker jitter at the threshold from making the gripper flicker. The jaws then move to the
closed / open carriage position at `max_speed`. Pick the threshold from
`ros2 run tf2_ros tf2_echo rigid_body_1 rigid_body_2`: midway between your pinched and apart distances.
Topics: `/rebel_gripper/state` (0/1), `/rebel_gripper/opening_mm`. The jaw positions are published on
`command_topic` (and on `/joint_states` with `target:=rviz`). Node parameters: `joint_prefix` (`xeg32_`;
`arm_left_xeg32_` for the rig), `closed_pos` / `open_pos` (carriage values [m], default = model joint limits).

## Recording a dataset (GR00T / LeRobot v2)
```bash
ros2 run rebel_demo recorder --ros-args -p cameras:="front:/rgb,wrist:/wrist_rgb" -p image_size:="[320,240]"
ros2 param set /rebel_recorder task "pick up the red cube and put it in the box"
ros2 service call /rebel_recorder/start std_srvs/srv/Trigger     # /stop saves, /discard throws it away
```
Each frame (default 30 fps): `observation.state` = joint1..6 measured (rad) + gripper measured (0..1),
`action` = joint1..6 commanded (rad) + gripper command (0/1), `observation.images.<cam>` = one H.264 mp4 per
camera per episode, plus the task text. Frames are skipped while any input is older than `max_age` (0.5 s).
Output (default `~/rebel_datasets/rebel_xeg32`, appended to across runs):
```
meta/info.json  episodes.jsonl  tasks.jsonl  modality.json     (modality: single_arm 0:6, gripper 6:7)
data/chunk-000/episode_000000.parquet
videos/chunk-000/observation.images.<cam>/episode_000000.mp4
```
Cameras come from Isaac Sim (ROS2 camera publishers, 8-bit encodings rgb8/bgr8/rgba8/bgra8/mono8).
For GR00T fine-tuning, define a new-embodiment data config using `single_arm` and `gripper`.

Hand up must move the robot up. If it moves sideways the mocap frame is Y-up: set Motive streaming
Up Axis to Z-up, or relaunch with `robot_ypr:="0 0 1.5708"` (or `-1.5708`).

## Troubleshooting
| symptom | fix |
|---|---|
| driver log `... not connected :(` | Motive streaming off or wrong Local Interface; firewall; IPs in the YAML. Relaunch the driver after fixing (it connects only at launch) |
| `ping` to the Motive PC says Destination Host Unreachable | the two PCs are not on the same router/network |
| `message type 'mocap4r2_msgs/msg/RigidBodies' is invalid` | `source ~/mocap_ws/install/setup.bash` |
| RViz `could not connect to display` | started over SSH: use `rviz:=false`, run RViz on a machine with a screen |
| RViz loads someone else's robot | another project uses the same `ROS_DOMAIN_ID`: pick another number everywhere |
| `ros2 service call` hangs on `waiting for service`, or prints `sequence size exceeds remaining buffer` | this terminal is on a different ROS setup than teleop (distro, `RMW_IMPLEMENTATION` or `ROS_DOMAIN_ID`). Compare with the running teleop: `tr '\0' '\n' < /proc/$(pgrep -n -f rebel_demo/teleop)/environ \| grep -E '^(ROS_\|RMW_)'`, set the same values, then `ros2 daemon stop` |
| `colcon build` makes `build/ install/ log/` inside `src/` | you ran it in the wrong folder: delete them, `cd ~/rebel_ws`, build again |
| `No module named 'xacro'` at launch | install xacro (`ros-<distro>-xacro`) |
| `package 'igus_rebel_description' not found` | copy it into `src/` and rebuild (section 3) |
| RViz: `No transform from link1 ... to world` for every link | the teleop node died (see its traceback): nothing publishes `/joint_states` |
| `No package metadata was found for rebel_demo` | broken install from a failed build: `rm -rf build/rebel_demo install/rebel_demo`, rebuild |
| robot does not move after engage | `ros2 run tf2_ros tf2_echo base_link rigid_body_1` must change as you move; check `hand_frame` |

## Next: Isaac Sim
Use the igus rebel2 model (the dual-arm rig from the igus bundle, or `urdf/rebel_xeg32.urdf.xacro`
expanded with `xacro`), position joint drives, enable `isaacsim.ros2.bridge`, Action Graph:
On Playback Tick -> ROS2 Subscribe Joint State (`/isaac_joint_commands`) -> Articulation Controller, plus ROS2 Publish
Joint State (`/joint_states`). Then (Isaac playing):
```bash
ros2 launch rebel_demo teleop.launch.py target:=isaac hand_frame:=rigid_body_1 rviz:=false \
  scale:=1.0 cutoff_hz:=6.0 gripper:=false lock_gripper:=true
```
Isaac graph subscribed to another name? Add `command_topic:=/that_name` (exact spelling, case-sensitive).

What goes to Isaac: `sensor_msgs/JointState` on `/isaac_joint_commands`, names `joint1`..`joint6`, radians, 60 Hz;
Isaac publishes the measured state back on `/joint_states`.
- Keep `gripper:=false` until the Isaac robot has the XEG jaw joints. The gripper node publishes its jaw joints
  (`xeg32_left_carriage_joint`, `xeg32_right_carriage_joint`) on the same topic; joint names Isaac does not know
  cause Articulation Controller errors and can make it skip arm commands.
- `lock_gripper:=true` makes the XEG joints fixed in the RViz model, since Isaac's `/joint_states` does not contain them.
- The launch file builds the robot model with xacro in every mode, so `igus_rebel_description` must be built on the
  PC that runs it (section 3).

## Licence and credits
- Robot model: `igus_rebel_description` from the igus mesh LOD bundle, not included (rebel2 arm, XEG-32, decimated meshes).
- Mocap driver (not included): [MOCAP4ROS2-Project/mocap4ros2_optitrack](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack).
- Kinematics, IK, `mocap_tf` and `teleop` nodes: written for this project (numpy + rclpy).
- Not suitable for a real robot as is: no collision checking. Use low speed limits and an e-stop, or MoveIt Servo.

## Known issues (planned fixes)
- HOME is all joints at 0 (arm straight up, near a wrist singularity, gripper pointing at the ceiling).
- At HOME, engaging moves the robot down ~12 cm: the workspace box (z <= 0.8 m) clamps the starting pose.
- IK targets `tool0`, not the point between the XEG jaws; with `orientation:=true` the fingertips swing.
- No `~/home` service yet: only restarting the teleop node returns the robot to HOME.
- XEG open / closed carriage values are the model joint limits (16.5 mm travel per jaw): check against the datasheet.
