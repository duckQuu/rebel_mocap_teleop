# rebel_demo — igus ReBeL hand teleoperation from OptiTrack mocap

Your hand (an OptiTrack rigid body) drives a simulated igus ReBeL 6-DoF in real time.
The hand pose is filtered, converted to joint angles with IK at 60 Hz, and published as
`sensor_msgs/JointState` — shown in RViz now, and ready for Isaac Sim (`/joint_command`).

```
Motive (Windows) --NatNet--> mocap4r2 OptiTrack driver --/rigid_bodies-->
  mocap_tf --TF map->rigid_body_1--> teleop (filter -> IK -> speed limit)
  --> /joint_states  -> robot_state_publisher -> RViz
  --> /joint_command -> Isaac Sim (next step)
```

## Contents
```
rebel_demo/
  rebel_demo/kinematics.py     ReBeL joint table, FK, IK (damped least squares), quaternion helper
  rebel_demo/mocap_tf_node.py  node "mocap_tf": mocap4r2 /rigid_bodies -> TF (map -> rigid_body_<id>)
  rebel_demo/teleop_node.py    node "teleop":   hand TF -> low-pass -> clutch/scale -> IK -> speed limit -> JointState
  launch/teleop.launch.py      starts mocap_tf, teleop, robot_state_publisher, static TF, (RViz)
  urdf/  meshes/  rviz/        igus ReBeL 6-DoF model (from igus iRC_ROS, Apache-2.0)
  test/test_teleop.py          pure-python tests: python3 -m pytest test/
```

## Requirements
- Ubuntu 24.04 + ROS 2 Jazzy on the PC that runs the mocap driver (the NatNet library is Linux-only).
- Python: numpy (no other packages).
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
```bash
mkdir -p ~/rebel_ws/src && cd ~/rebel_ws/src   # put the rebel_demo folder here
cd ~/rebel_ws && colcon build --symlink-install
```
`~/.bashrc` (every terminal):
```bash
export ROS_DOMAIN_ID=42            # pick a number nobody else on your network uses
source /opt/ros/jazzy/setup.bash
source ~/mocap_ws/install/setup.bash
source ~/rebel_ws/install/setup.bash
```

## 4. Run
```bash
# terminal 1: driver (wait for "Configured!")
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
# terminal 2: activate, check data (~120 Hz)
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
ros2 topic hz /rigid_bodies
# terminal 3: teleop + robot model + RViz   (add rviz:=false over SSH, then run RViz elsewhere)
ros2 launch rebel_demo teleop.launch.py target:=rviz hand_frame:=rigid_body_1 scale:=0.5
# terminal 2: engage the clutch, move your hand slowly
ros2 service call /rebel_teleop/engage std_srvs/srv/SetBool "{data: true}"
```
`{data: false}` releases the clutch: the robot holds, reposition your hand, engage again.
RViz on another machine: `rviz2 -d $(ros2 pkg prefix rebel_demo)/share/rebel_demo/rviz/rebel_demo.rviz`

## How teleop works (every cycle, 60 Hz)
1. Read the hand pose from TF (`base_link <- rigid_body_1`).
2. Tracking check: no update or a frozen pose for 0.2 s -> hold; re-anchor when tracking returns (no jumps).
3. Low-pass filter on the hand pose (position: first order; orientation: slerp). Filtering before IK keeps marker jitter out of the joints.
4. Clutch mapping: target = robot pose at engage + scale x (hand now - hand at engage); clamped to a workspace box.
5. IK (damped least squares, warm-started from the last command; position only unless `orientation:=true`).
6. Per-joint speed limit (default 45 deg/s, the real ReBeL maximum).
7. Publish `sensor_msgs/JointState` (`joint1..joint6`, radians) on `/joint_command`, and on `/joint_states` when `target:=rviz`.

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
| mocap_bridge | true | start the `mocap_tf` converter |
| rviz | true | start RViz here (needs a display) |

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
| robot does not move after engage | `ros2 run tf2_ros tf2_echo base_link rigid_body_1` must change as you move; check `hand_frame` |

## Next: Isaac Sim
Import `urdf/igus_rebel_6dof.urdf` (position joint drives), enable `isaacsim.ros2.bridge`, Action Graph:
On Playback Tick -> ROS2 Subscribe Joint State (`/joint_command`) -> Articulation Controller, plus ROS2 Publish
Joint State (`/joint_states`). Then `ros2 launch rebel_demo teleop.launch.py target:=isaac hand_frame:=rigid_body_1`.

## Licence and credits
- URDF and meshes: [CommonplaceRobotics/iRC_ROS](https://github.com/CommonplaceRobotics/iRC_ROS), Apache-2.0
  (licence in `meshes/`); mesh paths changed to `package://rebel_demo`, DAE meshes converted to STL.
- Mocap driver (not included): [MOCAP4ROS2-Project/mocap4ros2_optitrack](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack).
- Kinematics, IK, `mocap_tf` and `teleop` nodes: written for this project (numpy + rclpy).
- Not suitable for a real robot as is: no collision checking. Use low speed limits and an e-stop, or MoveIt Servo.
