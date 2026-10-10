# rebel_demo — igus ReBeL + HIWIN XEG-32 hand teleoperation from OptiTrack mocap

Your hand (an OptiTrack rigid body) drives an igus ReBeL 6-DoF (rebel2) arm of the Isaac **dual-arm rig** in real
time, and a second rigid body opens / closes its HIWIN XEG-32 gripper. One arm (`rig:=left` or `right`) or both
arms with two hands (`rig:=both`). The hand pose is filtered, converted to joint angles with IK at 60 Hz, and sent
to Isaac Sim as one `sensor_msgs/JointState` (`/dual_arm/isaac_joint_commands`); RViz shows the same rig model.
Demonstrations can be recorded as a GR00T / LeRobot v2 dataset.

```
Motive (Windows) --NatNet--> mocap4r2 OptiTrack driver --/rigid_bodies-->
  mocap_tf --TF map->rigid_body_1, rigid_body_2-->
    teleop  (rigid_body_1: filter -> IK -> speed limit)   -> arm_<side>_joint1..6
    gripper (|rigid_body_2 - rigid_body_1| < threshold)   -> XEG-32 open / close
    joint_merger (all arms + grippers in ONE message)
  --> /dual_arm/isaac_joint_commands -> Isaac Sim (rig bridge scripts)
  /dual_arm/isaac_joint_states (from Isaac) -> robot_state_publisher -> RViz
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
  rebel_demo/joint_merger_node.py  node "joint_merger": all arm / gripper commands -> ONE JointState per cycle
  rebel_demo/keyboard_node.py  node "teleop_keyboard": SPACE pauses / resumes the arm(s)
  launch/teleop.launch.py      starts mocap_tf, teleop + gripper per arm, joint_merger, robot_state_publisher, static TF, (RViz)
  rviz/                        RViz config
  tools/fake_hand.py           fake mocap: palm + fingertip rigid bodies for one or both hands (section 6)
  tools/fake_isaac.py          fake Isaac: sim clock + rig joint states that follow the commands (section 6)
  tools/make_low_poly_meshes.py  mesh decimation used to make igus_rebel_description/meshes_low/
  test/test_teleop.py          pure-python tests: python3 -m pytest test/
```

### Robot model
The launch file expands `igus_rebel_description/urdf/dual_arm_rig_v2.urdf.xacro` (`rig_xacro:=dual_arm_rig.urdf.xacro`
for the v1 rig): the same model the Isaac rig uses, with the lift, two sliders and two rebel2 arms
(`arm_left_joint1..6`, `arm_right_joint1..6`), each with an XEG-32 (`arm_<side>_xeg32_left/right_carriage_joint`, the
right jaw mimics the left). The IK works in `tool0` (link6 + 52.7 mm) of the chosen arm, relative to `arm_<side>_base_link`.
Joint limits follow the igus spec sheet (joint2 -80..140 deg, joint3 -80..140 deg, joint5 +-95 deg, others +-179 deg,
45 deg/s) and are identical in the rig model and `kinematics.py`, except one deliberate change: **joint3 >= +15 deg**
in `kinematics.py` (elbow-up bound, see "How teleop works"). `mesh_lod:=low` (default) loads the decimated
`meshes_low/`, `mesh_lod:=full` the full CAD meshes.

## Requirements
- Linux PC that runs the mocap driver (the NatNet library is Linux-only), with **one** ROS 2 distribution:
  Ubuntu 22.04 + Humble, or Ubuntu 24.04 + Jazzy. The code is plain rclpy and works on both.
  The commands below detect it with `$(ls /opt/ros)` (works when exactly one distribution is installed).
  Never mix two distributions (in terminals, workspaces or Isaac Sim).
- Python: numpy. The recorder also needs `pyarrow` (`pip install pyarrow`) and the `ffmpeg` binary.
- ROS packages: `xacro`, `robot_state_publisher`, `rviz2`, and `igus_rebel_description` (section 3).
- RViz can run on the same Linux PC (needs a screen) or on another machine (e.g. a Mac with RoboStack)
  that has this package built, same network, same `ROS_DOMAIN_ID`.

## 1. Mocap driver (separate workspace)
```bash
source /opt/ros/$(ls /opt/ros)/setup.bash
sudo apt install -y python3-rosdep python3-vcstool
sudo rosdep init 2>/dev/null; rosdep update
mkdir -p ~/mocap_ws/src && cd ~/mocap_ws/src
git clone -b rolling https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack.git
vcs import < mocap4ros2_optitrack/dependency_repos.repos
cd ~/mocap_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```
Do not use the `OptiTrack/mocap4ros2_optitrack` fork: its dependency file points to a repository that does not exist.

Set the network addresses in the driver config. Only the first line needs your value: the Motive PC IP
(Motive: Edit -> Settings -> Streaming -> Local Interface). This PC's IP is detected automatically.
```bash
MOTIVE_IP=192.168.1.10                       # <- the Motive PC IP
LOCAL_IP=$(hostname -I | awk '{print $1}')   # this Linux PC (check with: echo $LOCAL_IP)
CFG=~/mocap_ws/src/mocap4ros2_optitrack/mocap4r2_optitrack_driver/config/mocap4r2_optitrack_driver_params.yaml
sed -i -E "s/^( *connection_type:).*/\1 \"Multicast\"/; s/^( *server_address:).*/\1 \"$MOTIVE_IP\"/; \
  s/^( *local_address:).*/\1 \"$LOCAL_IP\"/; s/^( *multicast_address:).*/\1 \"239.255.42.99\"/" "$CFG"
grep -E "connection_type|server_address|local_address|multicast_address" "$CFG"
cd ~/mocap_ws && colcon build --symlink-install
```
`connection_type` must match Motive's Transmission Type (Multicast here; use Unicast in both if needed).

## 2. Motive (Windows)
Edit -> Settings -> Streaming: **Broadcast Frame Data** on, **Local Interface** = the Motive PC IP on the shared
network, **Transmission Type** = same as the YAML, **Rigid Bodies** on. Windows firewall: network profile Private,
allow Motive.

Rigid bodies: each one's **Streaming ID** (Properties -> Streaming ID) becomes the TF frame `rigid_body_<ID>`;
the name you give it in Motive does not matter. Defaults used by the launch file:

| rigid body | Streaming ID | TF frame | launch argument |
|---|---|---|---|
| palm (moves the arm) | 1 | `rigid_body_1` | `hand_frame` |
| fingertip (opens / closes the gripper) | 2 | `rigid_body_2` | `gripper_frame` |

Other IDs: pass them, e.g. `hand_frame:=rigid_body_5 gripper_frame:=rigid_body_6`. The `mocap_tf` log prints
`new rigid body: map -> rigid_body_<ID>` for every rigid body it receives.

## 3. Build this package
Get the code into the workspace `src` folder (clone it as `rebel_demo`):
```bash
mkdir -p ~/rebel_ws/src
cd ~/rebel_ws/src
git clone https://github.com/duckQuu/rebel_mocap_teleop.git rebel_demo
```
The robot model needs the `igus_rebel_description` package (rebel2 arm + XEG-32 macros, `meshes/` and
`meshes_low/`). It is **not in this repository** (vendor CAD). Put the igus mesh LOD bundle folder in your home
directory as `~/igus_mesh_lod_bundle`, then copy the package next to `rebel_demo` (colcon does not find packages
nested inside another package, so it must sit directly in `src/`):
```bash
cp -r ~/igus_mesh_lod_bundle/src/igus_rebel_description ~/rebel_ws/src/
```
Build **from the workspace root** `~/rebel_ws` — never from `src/` or `src/rebel_demo/`:
```bash
cd ~/rebel_ws
source /opt/ros/$(ls /opt/ros)/setup.bash
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "topic_based_ros2_control gz_ros2_control ros_gz_bridge"   # igus_rebel_description extras, not needed here
sudo apt install -y ffmpeg ros-$(ls /opt/ros)-joint-state-publisher-gui
pip install pyarrow                                                         # recorder only
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
- Install into the env: `conda install -c robostack-jazzy -c conda-forge ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui ffmpeg pyarrow`
- Workspace on this Mac: `~/Downloads/rebel_teleop_ws` (use it instead of `~/rebel_ws`).

Add the environment to `~/.bashrc` once (every terminal must have the **same** values — and the terminal that
starts Isaac Sim too). `ROS_DOMAIN_ID=42`: pick a number nobody else on your network uses.
```bash
sudo apt install -y ros-$(ls /opt/ros)-rmw-cyclonedds-cpp
cat >> ~/.bashrc <<'BASHRC'
source /opt/ros/$(ls /opt/ros)/setup.bash
source ~/mocap_ws/install/setup.bash
source ~/rebel_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
BASHRC
source ~/.bashrc && ros2 daemon stop
```
After changing any of these later, restart every ROS program (driver, teleop, Isaac Sim) and run
`ros2 daemon stop`: running programs keep their old values.

## 4. Run
One arm (`rig:=left` default, or `rig:=right`); for two hands see "Both arms" below.
```bash
# terminal 1: driver (wait for "Configured!")
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
# terminal 2: activate, check data (~120 Hz)
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
ros2 topic hz /rigid_bodies
# terminal 3, RViz only, no Isaac (palm = rigid_body_1, fingertip = rigid_body_2)
#   (add rviz:=false over SSH, then run RViz elsewhere)
ros2 launch rebel_demo teleop.launch.py target:=rviz rig:=left hand_frame:=rigid_body_1 scale:=0.5 \
  gripper_frame:=rigid_body_2 gripper_threshold:=0.08 gripper_hysteresis:=0.02 mesh_lod:=low
# terminal 3, Isaac Sim instead (Isaac playing, rig bridge scripts run, see section 5)
ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=left hand_frame:=rigid_body_1 rviz:=false \
  scale:=1.0 cutoff_hz:=6.0
# terminal 2: wait for "at HOME" in the launch log, then engage the clutch and move your hand slowly
ros2 service call /rebel_teleop/engage std_srvs/srv/SetBool "{data: true}"
```
At start the arm moves from its current joints to HOME (tool 35 cm in front of the arm base, 30 cm above it, pointing
down, elbow bent) at the joint speed limit; engage is refused until `at HOME` is logged. Engaging never moves the arm.
`{data: false}` releases the clutch: the robot holds, reposition your hand, engage again.
Back to HOME at any time (disengages): `ros2 service call /rebel_teleop/home std_srvs/srv/Trigger`.

Pause / resume with the spacebar (own terminal, keep it focused; Isaac keeps running, the gripper is not affected):
```bash
ros2 run rebel_demo teleop_keyboard      # SPACE = pause <-> resume, q = quit
# rig:=both:  ros2 run rebel_demo teleop_keyboard --ros-args -p teleop:=/rebel_teleop_left,/rebel_teleop_right
# without a keyboard:  ros2 service call /rebel_teleop/toggle std_srvs/srv/Trigger
```
On resume the robot's last pose and your hand's pose at that moment are the new reference, so you continue from
where it stopped; hand motion during the pause is ignored (use it to reposition your hand). With two arms, SPACE
pauses all of them if any runs, otherwise resumes all. The latched topic `/rebel_teleop/engaged` shows the state.

Hand up must move the robot up. If it moves sideways the mocap frame is Y-up: set Motive streaming
Up Axis to Z-up, or relaunch with `robot_ypr:="0 0 1.5708"` (or `-1.5708`).

RViz only, no mocap and no Isaac (the arm moves to HOME and stays, gripper open; the rig's other joints are faked at 0):
```bash
ros2 launch rebel_demo teleop.launch.py target:=rviz mocap_bridge:=false
```
To watch the arm move without mocap, add the fake hand stream (section 6).
Model viewer with joint sliders (instead of the launch above, not together with it; one command per terminal):
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix igus_rebel_description)/share/igus_rebel_description/urdf/dual_arm_rig_v2.urdf.xacro include_ros2_control:=false mesh_lod:=low)"
ros2 run joint_state_publisher_gui joint_state_publisher_gui
rviz2 -d $(ros2 pkg prefix rebel_demo)/share/rebel_demo/rviz/rebel_demo.rviz
```
Check the commands: `ros2 topic echo /dual_arm/isaac_joint_commands` (Isaac mode) or
`ros2 topic echo /dual_arm/isaac_joint_states` (RViz mode, published by `joint_merger`).
RViz on another machine: `rviz2 -d $(ros2 pkg prefix rebel_demo)/share/rebel_demo/rviz/rebel_demo.rviz`

## How teleop works (every cycle, 60 Hz)
1. Read the hand pose from TF (`arm_left_base_link <- rigid_body_1`).
2. Tracking check: no update or a frozen pose for 0.2 s -> hold. A single-sample jump faster than a hand can move (`max_hand_speed`, 2 m/s) is a marker glitch and is skipped (accepted if it persists 0.1 s). Gaps under 2 s (`reanchor_after`) keep the anchor and the tool catches up at most at `max_tool_speed` (0.5 m/s); after a longer loss the clutch re-anchors when tracking returns (no jumps). TF is received in its own thread, so the IK never delays it.
3. Low-pass filter on the hand pose (position: first order; orientation: slerp). Filtering before IK keeps marker jitter out of the joints.
4. Clutch mapping: target = robot pose at engage + scale x (hand now - hand at engage); clamped to a workspace box (in the arm base frame, x/y +-0.7 m, z -0.45..0.95 m) that is widened to include the pose at engage, so engaging never moves the arm.
5. IK (damped least squares with the analytic Jacobian, ~3 ms; joints that would cross a limit are pinned at it and the rest re-solved, so an unreachable target gives the closest reachable pose; warm-started from the last command; position only unless `orientation:=true`, the free wrist is pulled gently towards HOME). **Elbow up:** joint3 is kept >= +15 deg (lower limit in `kinematics.py`). In front of the arm every elbow-up solution has joint3 > 0, and the arm could only reach elbow-down by passing through the stretched arm (joint3 = 0); the bound stops it 15 deg before that. A target that would need a straighter arm is not reached: the arm holds at the bound and logs `joint limit reached: ... elbow-up bound 15 deg` (any joint at a limit is named the same way). At start-up the arm moves from its measured pose (e.g. Isaac's straight-up joint3 = 0) to HOME smoothly before the bound applies.
6. Per-joint speed limit (default 45 deg/s, the real ReBeL maximum).
7. Publish `sensor_msgs/JointState` (`arm_<side>_joint1..6`, radians); `joint_merger` combines all arms and grippers into one message on `command_topic` (default `/dual_arm/isaac_joint_commands`), or on `/dual_arm/isaac_joint_states` when `target:=rviz`.

## Launch arguments
| argument | default | meaning |
|---|---|---|
| target | rviz | `rviz`: no Isaac, `joint_merger` publishes the rig's joint states; `isaac`: Isaac publishes them and gets the commands |
| hand_frame | rigid_body_1 | TF frame of the hand = `rigid_body_` + Motive streaming ID (`rig:=left` / `right`) |
| mode | relative | `relative` (clutch) or `absolute` (flange goes to the hand pose) |
| scale | 1.0 | hand-to-robot motion ratio (0.5 = finer) |
| orientation | false | true = follow hand rotation too |
| cutoff_hz | 4.0 | filter cutoff; lower = smoother but more lag |
| max_joint_speed_deg | 45 | per-joint speed limit after IK |
| workspace_min / workspace_max | -0.7 -0.7 -0.45 / 0.7 0.7 0.95 | IK target clamp x y z in the arm base frame [m]; widened to include the pose at engage; a warning names the bound that clamps |
| limit_warn_mm | 5 | warn `joint limit reached` when the IK is this far [mm] off the target and a joint is at a limit |
| robot_xyz / robot_ypr | 0 0 0 | robot base pose in the mocap `map` frame (m, rad) |
| mocap_bridge | true | start the `mocap_tf` converter (`false` = no mocap needed) |
| rviz | true | start RViz here (needs a display) |
| mesh_lod | low | `low` (decimated meshes) or `full` (CAD) |
| lock_gripper | false | true = XEG joints fixed in the model (no gripper joints in RViz or commands) |
| gripper | true | start the gripper node |
| gripper_frame | rigid_body_2 | TF frame of the gripper-control rigid body (`rig:=left` / `right`) |
| gripper_ref_frame | (hand_frame) | distance is measured to this frame |
| gripper_mode | distance | `distance` (palm <-> fingertip distance) or `orientation` (their relative rotation) |
| gripper_open_angle / gripper_close_angle | 30 / 100 | orientation mode: open below / closed above this angle [deg] |
| gripper_threshold | 0.08 | switching distance [m] palm <-> fingertip |
| gripper_hysteresis | 0.02 | dead band [m] around the threshold |
| gripper_invert | false | true = far closes, near opens |
| rig | left | `left` / `right` = that arm of the dual-arm rig, the other holds its pose; `both` = both arms, two hands |
| left_hand_frame / left_gripper_frame | rigid_body_1 / rigid_body_2 | `rig:=both`: left palm / fingertip |
| right_hand_frame / right_gripper_frame | rigid_body_3 / rigid_body_4 | `rig:=both`: right palm / fingertip |
| rig_xacro | dual_arm_rig_v2.urdf.xacro | rig model in `igus_rebel_description/urdf` |
| use_sim_time | (auto) | `true` with `target:=isaac` (Isaac's `/clock`), else `false` |
| joint_names | (auto) | 6 comma-separated arm joint names (base to wrist) if Isaac uses other names, e.g. `arm_left_join_1,...,arm_left_join_6`; default = `arm_<side>_joint1..6` |
| command_topic | (auto) | `/dual_arm/isaac_joint_commands`; must match Isaac's ROS2 Subscribe Joint State node exactly |

## Gripper (HIWIN XEG-32)
Two rigid bodies on the same hand: the **palm** (`rigid_body_1`, moves the arm) and a **fingertip** (`rigid_body_2`,
Motive Streaming ID 2, controls the gripper). Finger extended = far = **open**, finger curled = near = **closed**.
Moving or rotating the whole hand does not change their distance, so arm motion never opens or closes the gripper.
The node measures the distance between the two rigid bodies' pivot points and switches between two states:
```
distance < threshold - hysteresis/2   -> CLOSE (0)
distance > threshold + hysteresis/2   -> OPEN  (1)
in between, or tracking lost          -> keep the current state
```
The dead band stops marker jitter at the threshold from making the gripper flicker, and a new state must hold for
0.1 s (`debounce`) before the gripper switches, so a one-frame marker glitch cannot open or close it. The jaws then move to the
closed / open carriage position at `max_speed`. Defaults: threshold 0.08 m, hysteresis 0.02 m (closes below 7 cm,
opens above 9 cm); typical palm -> index fingertip distances are ~10-13 cm extended and ~4-6 cm curled.

Calibrate: print the distance live, hold the finger extended, then curled, and set `gripper_threshold` to the
midpoint and `gripper_hysteresis` to about a third of the difference:
```bash
stdbuf -oL ros2 run tf2_ros tf2_echo rigid_body_1 rigid_body_2 2>/dev/null | awk '/Translation/ {gsub(/[\[\],]/,""); printf "%.1f cm\n", 100*sqrt($3*$3+$4*$4+$5*$5)}'
ros2 topic echo /rebel_gripper/state     # (rig:=both: /rebel_gripper_left/state) 1 = open, 0 = closed (the launch log also prints OPEN / CLOSE + distance)
```
Marker tips: put the fingertip markers on the back of the finger (nail side), facing the cameras: markers on the
pad are hidden when you curl, and when tracking is lost the gripper keeps its last state (it may then never close).
Use clearly different marker patterns for palm and fingertip.
### Orientation mode (`gripper_mode:=orientation`)
Instead of the distance, the gripper follows how much the fingertip body is **rotated relative to the palm body**.
Positions are ignored. The relative rotation (`rigid_body_1 <- rigid_body_2`) does not change when you turn the
whole hand, so it works facing up, down, left or right. The node takes the total rotation angle (0-180 deg):
```
angle < gripper_open_angle   (30)   -> OPEN   (same orientation as the calibrated open pose)
angle > gripper_close_angle  (100)  -> CLOSE  (finger body flipped, ~180 deg)
in between, or tracking lost        -> keep the current state
```
Motive rigid-body axes are arbitrary (set when each body was created), so hold your hand relaxed / open and call
the calibration once; that pose then counts as 0 deg:
```bash
ros2 launch rebel_demo teleop.launch.py rig:=left target:=isaac hand_frame:=rigid_body_1 \
  gripper_frame:=rigid_body_2 gripper_mode:=orientation rviz:=false
ros2 service call /rebel_gripper/calibrate std_srvs/srv/Trigger
ros2 topic echo /rebel_gripper/angle_deg     # tune gripper_open_angle / gripper_close_angle with this
```
The angle includes any rotation (twist or tilt about any axis) between the two bodies, so keep the two bodies
rigidly fixed to your hand and only flip the finger body to close. The 0.1 s debounce applies here too.

Topics: `/rebel_gripper/state` (0/1), `/rebel_gripper/opening_mm`. The jaw positions go to `joint_merger` together
with the arm joints. Node parameters: `joint_prefix` (set by the launch: `arm_left_xeg32_` / `arm_right_xeg32_`),
`closed_pos` / `open_pos` (carriage values [m], default = model joint limits).

## Recording a dataset (GR00T / LeRobot v2)
```bash
ros2 run rebel_demo recorder --ros-args -p use_sim_time:=true -p arms:=left \
  -p cameras:="wrist:/dual_arm/wrist_left/rgb/image_raw,top:/dual_arm/gemini336L/rgb/image_raw" -p image_size:="[320,240]"
ros2 param set /rebel_recorder task "pick up the red cube and put it in the box"
ros2 service call /rebel_recorder/start std_srvs/srv/Trigger     # /stop saves, /discard throws it away
```
`arms:=left` (default), `right`, or `left,right` for both arms (see "Both arms"). Joint states are read from
`/dual_arm/isaac_joint_states` and commands from `/dual_arm/isaac_joint_commands` (parameters `joint_states_topic`,
`command_topic`); cameras are the rig bridge topics (list them with `ros2 topic list | grep -i rgb`).
Each frame (default 30 fps): `observation.state` = 6 joints measured (rad) + gripper measured (0..1),
`action` = 6 joints commanded (rad) + gripper command (0/1), `observation.images.<cam>` = one H.264 mp4 per
camera per episode, plus the task text. Frames are skipped while any input is older than `max_age` (0.5 s).
Output (default `~/rebel_datasets/rebel_xeg32`, appended to across runs):
```
meta/info.json  episodes.jsonl  tasks.jsonl  modality.json     (one arm: single_arm 0:6, gripper 6:7)
data/chunk-000/episode_000000.parquet
videos/chunk-000/observation.images.<cam>/episode_000000.mp4
```
Cameras come from Isaac Sim (ROS2 camera publishers, 8-bit encodings rgb8/bgr8/rgba8/bgra8/mono8).
For GR00T fine-tuning, define a new-embodiment data config using `single_arm` and `gripper`
(two arms: `left_arm`, `left_gripper`, `right_arm`, `right_gripper`).
Use a separate `dataset_dir` for one-arm and two-arm recordings: their state / action vectors differ.

## 5. Isaac Sim (dual-arm rig)
In Isaac's Script Editor (stage loaded, timeline **stopped**), run the rig's three scripts, then press Play. Adjust
`path_base` to where the scripts are:
```python
scripts = ["dual_arm_rig_ros2_bridge.py", "sim_flap_weld.py", "sim_scene_control.py"]
path_base = "/home/user1/Documents/compal_dual_arm/compal_robot/compal_robot/src/igus_rebel_description/urdf/igus_stand_dual_arm/"
for script in scripts:
    with open(path_base + script) as f:
        exec(f.read())
```
The bridge script subscribes to `/dual_arm/isaac_joint_commands` and publishes `/dual_arm/isaac_joint_states` and
`/clock`. Then teleop **one** arm (`rig:=left` or `rig:=right`); the other arm holds its pose:
```bash
ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=left hand_frame:=rigid_body_1 rviz:=false \
  scale:=1.0 cutoff_hz:=6.0
```
`rig:=left` sets: joints `arm_left_joint1..6`, IK base frame `arm_left_base_link`, gripper joints
`arm_left_xeg32_left/right_carriage_joint`, topics `/dual_arm/isaac_joint_commands` and `/dual_arm/isaac_joint_states`.
- `rig_xacro:=dual_arm_rig.urdf.xacro` if Isaac runs the v1 rig (default `dual_arm_rig_v2.urdf.xacro`).
- Do not run the rig's own `robot_state_publisher` (e.g. `dual_arm_rig_isaac.launch.py`) at the same time: two
  robot descriptions publish conflicting TF.
- `robot_xyz` / `robot_ypr` place the rig's `world` in the mocap frame; only the rotation matters in relative mode.
- Joint names must match Isaac's exactly. Check with
  `ros2 topic echo /dual_arm/isaac_joint_states --once --field name`; if they differ from `arm_left_joint1..6`, pass
  them: `joint_names:=arm_left_join_1,arm_left_join_2,arm_left_join_3,arm_left_join_4,arm_left_join_5,arm_left_join_6`.
  RViz still uses the rig model's names, so with other names it shows the arm at 0.
- Clock: the bridge stamps joint states with **simulation time** and publishes `/clock`, so `target:=isaac` turns on
  `use_sim_time` for every node (override with `use_sim_time:=false`). Without it the arm TF (sim time) and the
  hand TF (PC time) never share a timestamp, teleop cannot place the hand, and it keeps sending HOME (all zeros).
  Isaac must be **playing** (the clock only runs then). Restart teleop after stopping / resetting Isaac.
- Gripper: the rig's XEG joints are unlocked, so `gripper:=true` (default) drives that arm's jaws. If Isaac reports
  errors for `..._right_carriage_joint` (mimic joint), use `gripper:=false`.
- Every arm and gripper publishes on its own topic (`/rebel_teleop/joint_commands`, ...) and `joint_merger` sends all
  of them to Isaac as **one** message per cycle: Isaac's Subscribe Joint State keeps only the latest message, so
  separate arm and gripper messages would otherwise take turns.

### Both arms (two hands)
Four rigid bodies: a palm and a fingertip per hand. Defaults (change them to your Motive Streaming IDs):

| | palm (moves the arm) | fingertip (gripper) |
|---|---|---|
| left arm | `left_hand_frame:=rigid_body_1` | `left_gripper_frame:=rigid_body_2` |
| right arm | `right_hand_frame:=rigid_body_3` | `right_gripper_frame:=rigid_body_4` |

```bash
ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=both rviz:=false scale:=1.0 cutoff_hz:=6.0 \
  left_hand_frame:=rigid_body_1 left_gripper_frame:=rigid_body_2 \
  right_hand_frame:=rigid_body_3 right_gripper_frame:=rigid_body_4
# wait for "at HOME" from both arms, then engage each arm separately (or both):
ros2 service call /rebel_teleop_left/engage std_srvs/srv/SetBool "{data: true}"
ros2 service call /rebel_teleop_right/engage std_srvs/srv/SetBool "{data: true}"
# or pause / resume both with the spacebar:
ros2 run rebel_demo teleop_keyboard --ros-args -p teleop:=/rebel_teleop_left,/rebel_teleop_right
```
- Two teleop nodes (`/rebel_teleop_left`, `/rebel_teleop_right`) and two gripper nodes (`/rebel_gripper_left`,
  `/rebel_gripper_right`; state on `/rebel_gripper_<side>/state`); each gripper measures its fingertip to its own palm.
- All 12 arm joints and 4 jaw joints go to Isaac in one message per cycle (`joint_merger`).
- `scale`, `mode`, `orientation`, `cutoff_hz`, `max_joint_speed_deg`, `gripper_threshold` / `gripper_hysteresis` apply
  to both arms. `hand_frame`, `gripper_frame`, `gripper_ref_frame` and `joint_names` are for one arm only.

Record both arms (14 values per frame: left arm 6 + left gripper + right arm 6 + right gripper; `modality.json`
parts `left_arm`, `left_gripper`, `right_arm`, `right_gripper`):
```bash
ros2 run rebel_demo recorder --ros-args -p use_sim_time:=true -p arms:=left,right \
  -p cameras:="top:/dual_arm/gemini336L/rgb/image_raw,wrist_left:/dual_arm/wrist_left/rgb/image_raw,wrist_right:/dual_arm/wrist_right/rgb/image_raw" \
  -p image_size:="[320,240]"
```

## 6. Test without mocap / without Isaac (fake data)
`tools/fake_hand.py` is a fake teleop data stream: it publishes the palm and fingertip rigid bodies at 120 Hz (like
`mocap_tf` for real mocap), engages the clutch once the arm is at HOME, and moves each palm in its arm's own axes:
hold, 10 cm down, 15 cm forward (fingertip curls, gripper closes), back, up, gripper opens again.
`tools/fake_isaac.py` stands in for Isaac: sim clock on `/clock`, the rig's 21 joints starting at 0, following
`/dual_arm/isaac_joint_commands` with a short drive lag, published on `/dual_arm/isaac_joint_states`.
One command per terminal (no mocap driver needed: `mocap_bridge:=false`):
```bash
# a) RViz only, one arm
ros2 launch rebel_demo teleop.launch.py target:=rviz mocap_bridge:=false
python3 ~/rebel_ws/src/rebel_demo/tools/fake_hand.py

# b) RViz only, both arms (right hand half a cycle behind), repeating
ros2 launch rebel_demo teleop.launch.py target:=rviz rig:=both mocap_bridge:=false
python3 ~/rebel_ws/src/rebel_demo/tools/fake_hand.py --arm both --loop

# c) the full Isaac interface with the fake Isaac (topics, sim clock, joint names exactly as with Isaac)
python3 ~/rebel_ws/src/rebel_demo/tools/fake_isaac.py
ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=both mocap_bridge:=false
python3 ~/rebel_ws/src/rebel_demo/tools/fake_hand.py --arm both --sim-time --loop

# d) real Isaac, fake hands: as c) without fake_isaac.py (Isaac playing, bridge scripts run)

# e) up / down / left / right instead of down / forward (both arms, 10 cm each way; --amp for more)
python3 ~/rebel_ws/src/rebel_demo/tools/fake_hand.py --arm both --pattern cross --loop     # add --sim-time with target:=isaac

# f) through the mocap converter too: publish /rigid_bodies like the OptiTrack driver (Linux, mocap4r2_msgs sourced)
ros2 launch rebel_demo teleop.launch.py target:=rviz rig:=both                       # mocap_bridge on (default)
python3 ~/rebel_ws/src/rebel_demo/tools/fake_hand.py --arm both --pattern cross --output rigid_bodies --loop
```
`--pattern cross`: hold, up, back, down, back, left, back, right, back, each `--amp` (0.10 m) in the arm's own axes
(left = the arm base's +y), gripper open. Measured on both arms (RViz and fake Isaac): +-10.0 cm and +-20.0 cm exactly
on the commanded axis, under 0.5 cm on the others, back at HOME within 1 mm, no warnings.
**Find the joint limit: progressively lower hand.** `--descend 0.05` goes 5 cm down and `--descend-forward` (default
0.3) x 5 cm forward, holds 2 s, and repeats, deeper each time (prints `depth NN cm, forward NN cm`) up to `--max-down`
(0.70 m), then returns. A hand that reaches lower also reaches out, so the default path leans forward; use
`--descend-forward 0` for straight down.
```bash
ros2 launch rebel_demo teleop.launch.py target:=rviz mocap_bridge:=false
python3 ~/rebel_ws/src/rebel_demo/tools/fake_hand.py --descend 0.05
```
Reach of the arm (elbow-up bound on, tool relative to HOME, which is 35 cm in front of the base and 30 cm up): straight
down it follows exactly to 60 cm below HOME (joint2 140 deg and joint3 15 deg are reached together); the further forward
it is, the less deep it goes (x = 0.20 m: 65 cm, 0.35 m: 60 cm, 0.45 m: 53 cm, 0.55 m: 40 cm below HOME). With the
default lean (0.3) the tool follows to about 45 cm down / 14 cm forward and then reaches its limit. The default
`workspace_min` z = -0.45 m is below everything the arm can reach, so a joint limit (not the box) stops it. Teleop warns,
at most once per second:
```
joint limit reached: arm_left_joint3 at lower elbow-up bound 15 deg - target 5 mm out of reach, moving to the closest reachable pose
joint limit reached: arm_left_joint2 at upper limit 140 deg, arm_left_joint3 at lower elbow-up bound 15 deg - target 119 mm out of reach, moving to the closest reachable pose
joint limits: target back in range          (when the hand comes back)
target clamped by workspace_min z = -0.45 m (hand is 5.0 cm beyond) - raise the limit if the arm can go there
```
The first two appear when the IK is more than `limit_warn_mm` (5) off the target while a joint sits at a limit; the
arm then goes to the closest pose it can reach (the IK pins the joint at its limit and re-solves the others, so the arm
slides along the limit instead of jumping to a corner posture) and follows again as soon as the hand returns. The last
one means the workspace box, not a joint, stopped the target. Both arms warn independently (`rig:=both`, node names
in the log).

Options of `fake_hand.py`: `--arm left|right|both`, `--pattern reach|cross` (+ `--amp`), `--output tf|rigid_bodies`, `--loop`, `--down` / `--forward` [m], `--seg` [s per move],
`--descend STEP` / `--descend-forward RATIO` / `--max-down` [m],
`--sim-time` (required with `target:=isaac`: teleop runs on Isaac's clock there), `--unstable` (2 mm jitter,
drop-outs, frozen frames, marker glitches; or `--jitter`, `--dropouts`, `--freezes`, `--outliers`), `--rotate-finger`
(with `gripper_mode:=orientation`). On this Mac the workspace is `~/Downloads/rebel_teleop_ws` instead of `~/rebel_ws`.

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
| `No module named 'xacro'` at launch | `sudo apt install ros-$(ls /opt/ros)-xacro` |
| `package 'igus_rebel_description' not found` | copy it into `src/` and rebuild (section 3) |
| RViz: `No transform from ... to world` for the arm links | nobody publishes the rig's joint states: Isaac not playing / bridge scripts not run (`ros2 topic hz /dual_arm/isaac_joint_states`), or in `target:=rviz` the `joint_merger` node died (see its traceback) |
| `No package metadata was found for rebel_demo` | broken install from a failed build: `rm -rf build/rebel_demo install/rebel_demo`, rebuild |
| robot does not move after engage | `ros2 run tf2_ros tf2_echo arm_left_base_link rigid_body_1` must change as you move; check `hand_frame` |

## Licence and credits
- Robot model: `igus_rebel_description` from the igus mesh LOD bundle, not included (dual-arm rig, rebel2 arm, XEG-32, decimated meshes).
- Mocap driver (not included): [MOCAP4ROS2-Project/mocap4ros2_optitrack](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack).
- Kinematics, IK, `mocap_tf` and `teleop` nodes: written for this project (numpy + rclpy).
- Not suitable for a real robot as is: no collision checking. Use low speed limits and an e-stop, or MoveIt Servo.

## Known issues
- IK targets `tool0`, not the point between the XEG jaws; with `orientation:=true` the fingertips swing.
- The wrist side (joint5 sign) is not constrained; only the elbow is (joint3 >= 15 deg).
- XEG open / closed carriage values are the model joint limits (16.5 mm travel per jaw): check against the datasheet.
