# rebel_demo — ROS 2 package for the igus ReBeL mocap demos

Visualize the 15 demo trajectories in RViz, and send a trajectory to the igus
`joint_trajectory_controller` (mock hardware first, then the real robot).

```
rebel_demo/
  rebel_demo/kinematics.py     ReBeL joint table, FK, joint frames, HOME pose
  rebel_demo/trajectory_io.py  CSV reading, smooth moves, speed limiting, limit check
  rebel_demo/player_node.py    node "player": RViz playback (/joint_states + markers)
  rebel_demo/sender_node.py    node "sender": FollowJointTrajectory to the controller
  rebel_demo/teleop_node.py    node "teleop": hand (mocap TF) -> filter -> IK -> /joint_command
  rebel_demo/mocap_tf_node.py  node "mocap_tf": mocap4r2 /rigid_bodies -> TF (map -> rigid_body_<id>)
  launch/view.launch.py        robot model + RViz + player
  launch/mock_robot.launch.py  igus irc_ros bringup (mock_hardware | cri | cprcanv2)
  launch/teleop.launch.py      hand teleop: static TF + robot model + teleop (+ RViz)
  urdf/  meshes/  rviz/        igus ReBeL 6-DoF model (from igus iRC_ROS, Apache-2.0; meshes as STL)
  trajectories/*.csv           15 demo trajectories (30 Hz, q1_deg..q6_deg, flange path)
  tools/                       motive2gr00t.py + make_trajectories.sh (offline, not ROS)
  test/test_logic.py           pure-python tests (python3 -m pytest test/)
```

## Build
```bash
mkdir -p ~/rebel_ws/src && cd ~/rebel_ws/src
unzip ~/Downloads/rebel_demo_ws.zip      # or copy the rebel_demo folder here
cd ~/rebel_ws
source /opt/ros/$ROS_DISTRO/setup.bash   # macOS/robostack: activate your ROS env instead
colcon build --symlink-install --packages-select rebel_demo
source install/setup.bash
```

## 1. Visualize (no robot)
```bash
ros2 launch rebel_demo view.launch.py                         # all 15 takes
ros2 launch rebel_demo view.launch.py only:=left rate:=2.0    # only "left" takes, 2x speed
ros2 launch rebel_demo view.launch.py loop:=true joints:=false
ros2 launch rebel_demo view.launch.py play:=false             # joint sliders instead
```
In RViz: grey robot, blue line = flange path of the current take, **green dot = flange**
(it must sit on the blue line), coloured arrows = joint axes with names and angles
(J1 base, J2 shoulder, J3 elbow, J4 forearm roll, J5 wrist, J6 flange roll).

## 2. Send a trajectory to the igus controller
Needs the igus packages in the same workspace (`github.com/CommonplaceRobotics/iRC_ROS`, build it
with colcon). Terminal 1 — controller on mock hardware (same software as the real robot):
```bash
ros2 launch rebel_demo mock_robot.launch.py                   # hardware_protocol:=mock_hardware
```
Terminal 2 — dry run first (prints duration, speed, limit check; sends nothing):
```bash
ros2 run rebel_demo sender --ros-args -p trajectory:=take_v2_left_flaps_001
ros2 run rebel_demo sender --ros-args -p trajectory:=take_v2_left_flaps_001 -p execute:=true
```
The sender reads the current joint angles, adds a slow smooth move to the demo start, slows the
demo so no joint exceeds `speed_fraction` (default 0.5) x 45 deg/s, refuses anything within 2 deg
of a joint limit, then sends a `FollowJointTrajectory` goal.

**Real robot:** `ros2 launch rebel_demo mock_robot.launch.py hardware_protocol:=cri` (or `cprcanv2`),
same sender command. Only after: kinematics check, real robot position / calibration, a dry run,
mock-hardware run, e-stop in hand, `speed_fraction` low (e.g. 0.3).

## 3. Current trajectories — what they are
- From the 15 hand-held mocap demos, starting when the tool is near the box (the human
  "bringing the tool in" part is cut: `--start-near-box 0.30 0.35`).
- Robot placement: **provisional** `--robot-origin -0.62 1.72 0.09 --robot-yaw -127`
  (box ~40 cm straight in front). Replace with the real position / calibration.
- **Position-only IK**: the flange follows the marker-cluster position; the tool tilt is not
  reproduced yet (needs the tool mounting `--tcp-rpy` or `--calib calib.json`).
- `take_v2_both_flaps_001` touches the joint-5 limit -> the sender refuses it (viewing is fine).

Regenerate (e.g. with your measured robot position): edit the top of `tools/make_trajectories.sh`,
run `tools/make_trajectories.sh /path/to/take_v2_mocap`, then `colcon build` again.

## 4. Checks
- Kinematics: `ros2 launch rebel_demo view.launch.py play:=false`, set sliders, compare with the
  real robot at the same joint angles (straight up = `[0, -30, -30, 0, 7.5, 0]` deg in this URDF).
- Logic tests without ROS: `cd src/rebel_demo && python3 -m pytest test/`


## 5. Hand teleoperation (mocap -> Isaac Sim)

```
Motive --NatNet--> mocap4ros2_optitrack --TF map->optitrack->HAND-->
  teleop node:  TF lookup -> tracking check -> LOW-PASS -> clutch/scale/clamp -> IK -> speed limit
  --> /joint_command  (sensor_msgs/JointState, joint1..joint6, radians)  --> Isaac Sim
  <-- /joint_states   (published by Isaac, for RViz / feedback)
```

**What Isaac Sim gets:** `sensor_msgs/msg/JointState` on `/joint_command`; `name` = joint1..joint6,
`position` = radians, velocity and effort left empty. The robot is position-controlled in Isaac.

**Where the filter and IK run:** both are inside the teleop node, in this order:
low-pass on the *hand pose* (before IK), then IK, then a per-joint speed limit (after IK).
Isaac only receives finished joint angles.

### Test in RViz first (no Isaac) - all on the Linux PC
The mocap4r2 driver publishes `/rigid_bodies` only (no TF). `teleop.launch.py` starts the
`mocap_tf` converter (`mocap_bridge:=true`, default): Motive streaming ID 1 -> TF frame `rigid_body_1`.
```bash
# terminal 1: mocap driver
source ~/mocap_ws/install/setup.bash
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
# terminal 2
source ~/mocap_ws/install/setup.bash
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
# terminal 3: converter + teleop + robot model + RViz
source ~/mocap_ws/install/setup.bash && source ~/rebel_ws/install/setup.bash
ros2 launch rebel_demo teleop.launch.py target:=rviz hand_frame:=rigid_body_1 scale:=0.5
# terminal 4: start following (false = release, move your hand freely, true again = re-anchor)
ros2 service call /rebel_teleop/engage std_srvs/srv/SetBool "{data: true}"
```
`robot_xyz` / `robot_ypr` = where the virtual robot base stands in the mocap `map` frame
(metres; yaw pitch roll in radians). In relative mode only the rotation matters.
Check: `ros2 run tf2_ros tf2_echo base_link rigid_body_1` changes as you move.
**Hand up must move the robot up.** If it moves sideways the mocap frame is Y-up: set Motive
streaming Up Axis = Z-up, or rotate with `robot_ypr:="0 0 1.5708"` (or -1.5708).
`/rebel_teleop/target` (PoseStamped) shows the target the IK is chasing.

### Then Isaac Sim (Linux / Windows PC, not macOS)
1. Import `urdf/igus_rebel_6dof.urdf` (File > Import, URDF importer): fixed base, joint drive
   type **position**, e.g. stiffness 10000-100000, damping 100-1000.
2. Enable the extension `isaacsim.ros2.bridge`. Same `ROS_DOMAIN_ID` on both sides.
3. Action Graph (Window > Graph Editors > Action Graph):
   `On Playback Tick` -> `ROS2 Subscribe Joint State` (topic `/joint_command`) ->
   `Articulation Controller` (targetPrim = the robot root prim; connect positionCommand, jointNames);
   `On Playback Tick` -> `ROS2 Publish Joint State` (topic `/joint_states`, targetPrim = robot);
   `Isaac Read Simulation Time` -> timeStamp; add a `ROS2 Context` node.
4. Press Play, test without mocap:
   `ros2 topic pub --once /joint_command sensor_msgs/msg/JointState "{name: [joint1,joint2,joint3,joint4,joint5,joint6], position: [0.0,0.35,1.05,0.0,1.05,0.0]}"`
   (= HOME, the pose teleop starts from).
5. `ros2 launch rebel_demo teleop.launch.py target:=isaac hand_frame:=HAND robot_xyz:="..."`, then engage.

### Teleop parameters (launch arguments)
| argument | default | meaning |
|---|---|---|
| mode | relative | relative = clutch (robot moves by how much your hand moves); absolute = flange goes to the hand pose |
| scale | 1.0 | hand motion x scale (0.5 = finer control) |
| orientation | false | true = follow hand rotation too (full 6-DoF IK) |
| cutoff_hz | 4.0 | low-pass cutoff. Lower = smoother but more lag (2 Hz ~ 80 ms, 6 Hz ~ 25 ms) |
| max_joint_speed_deg | 45 | per-joint speed limit after IK (the real ReBeL max) |

Tracking lost (no TF update for 0.2 s, or a frozen pose) -> the robot holds; when tracking returns
it re-anchors, so it never jumps. Target outside reach -> it goes as close as possible and warns.
