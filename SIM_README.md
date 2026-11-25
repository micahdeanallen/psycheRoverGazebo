# Hexapod Simulation (Locomotion)

Three new packages sit next to the existing nav packages in `src/`:

| Package | What it is |
|---|---|
| `psyche_description` | Robot model (URDF/xacro), ros2_control setup, controller config, RViz viewer |
| `psyche_gazebo` | Gazebo Fortress world and the sim launch file |
| `psyche_locomotion` | Leg control: kinematics, tripod gait, `gait_node` (/cmd_vel -> 18 joints), `stand_node` |

None of the existing nav code was changed.

## How it fits together

```
Nav stack (existing)  -> /cmd_vel                           (gait_node listens here)
psyche_locomotion     -> /leg_position_controller/commands  (18 joint angles)
ros2_control          -> hardware plugin chosen in hexapod.ros2_control.xacro
   sim:  Gazebo (gz_ros2_control)
   real: Jetson -> Arduino serial plugin (future; mock hardware stands in for now)
```

The gait code only ever sees 18 joint angles. Swapping sim for the real rover
happens in one place and does not touch the locomotion code.

## The model

All dimensions are in `psyche_description/urdf/params.xacro`, each tagged
`[drawing]`, `[assumed]`, or `[datasheet]`. Change a number there and the whole
model updates.

- Frame 600 x 200 x 150 mm, total mass 4 kg (anticipating the metal frame)
- 3 legs per side, 200 mm apart (top-view sketch)
- Each leg: hip (LX-16A) -> Connector A 75 mm -> knee (MG-996R) -> Connector B 75 mm -> ankle (MG-996R) -> foot 100 mm
- **Every joint at 0 = standing pose** (femur level, foot straight down)
- Servo torque and speed limits are set from datasheets, so the sim won't be stronger than the real servos

Joint names: `{leg}_{hip|femur|tibia}_joint` with legs `lf lm lr rf rm rr`
(left/right, front/middle/rear). Sign conventions:

- `hip +`: counter-clockwise from above (left legs swing rearward, right legs forward)
- `femur +`: leg lifts
- `tibia +`: foot swings outward

## One-time setup (Windows + WSL)

1. **Ubuntu 22.04 is required** (ROS 2 Humble). Check with `lsb_release -a`.
   If it says 24.04, add 22.04 alongside it from PowerShell:
   `wsl --install -d Ubuntu-22.04`
   Windows 11 is needed for the Gazebo window to show (WSLg).
2. **Install ROS 2 Humble Desktop** following the official guide:
   https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
3. **Install Gazebo Fortress and the sim packages:**
   ```bash
   sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control \
     ros-humble-ros2-control ros-humble-ros2-controllers \
     ros-humble-xacro ros-humble-joint-state-publisher-gui
   ```
4. Add `source /opt/ros/humble/setup.bash` to the end of `~/.bashrc`.

## Build

```bash
cd psyche_rover_ws/psyche_rover_ws
colcon build --symlink-install --packages-select psyche_description psyche_gazebo psyche_locomotion
source install/setup.bash
```

`--packages-select` builds only the sim packages, so missing dependencies in the
nav packages won't block you. Drop it to build everything.

## Run

**View the model only (no physics), with a slider per joint:**
```bash
ros2 launch psyche_description display.launch.py
```

**Full simulation:**
```bash
# Terminal 1: Gazebo + robot + controllers
ros2 launch psyche_gazebo sim.launch.py

# Terminal 2: ease into the standing pose
ros2 run psyche_locomotion stand_node --ros-args -p use_sim_time:=true

# Try a crouch (femur raised, foot tucked). This puts more load on the femur servos.
ros2 run psyche_locomotion stand_node --ros-args -p use_sim_time:=true -p femur:=0.4 -p tibia:=-0.3
```

**Command joints by hand** (18 values, order = hip, femur, tibia for lf, lm, lr, rf, rm, rr).
This lifts the left-front leg:
```bash
ros2 topic pub --once /leg_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0,0.6,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0]}"
```

**Watch servo torque** (compare femur values to the MG-996R limit of 1.08 N*m):
```bash
ros2 topic echo /joint_states --field effort
```

## Walking with /cmd_vel

```bash
# Terminal 1
ros2 launch psyche_gazebo sim.launch.py
# Terminal 2 (don't run stand_node at the same time)
ros2 run psyche_locomotion gait_node --ros-args -p use_sim_time:=true
# Terminal 3: walk forward at 5 cm/s for 4 s (40 messages at 10 Hz), about 20 cm
ros2 topic pub -r 10 -t 40 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
# Turn in place
ros2 topic pub -r 10 -t 40 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"
```

Keyboard driving: `sudo apt install ros-humble-teleop-twist-keyboard`, then
`ros2 run teleop_twist_keyboard teleop_twist_keyboard` (use `z` to lower its speed;
gait_node caps speed at 0.10 m/s, 0.05 m/s sideways, 0.4 rad/s turning).

`/cmd_vel` is a velocity, so "move X cm" = speed x duration. If no command arrives
for 0.5 s, the rover steps to a stop with all six feet down.

Code layout:
- `kinematics.py`: inverse/forward kinematics, no ROS. Geometry must match `params.xacro`.
- `gait.py`: tripod gait, no ROS. Body velocity in, 18 angles out.
- `gait_node.py`: ROS wrapper. Tuning parameters: `period`, `step_height`, `max_vx`, `max_vy`, `max_wz`.
- Tests: `cd src/psyche_locomotion && python3 -m pytest test`

## Troubleshooting

- **Gazebo window crashes in WSL (Ogre exception):** sim.launch.py now uses `--render-engine ogre`. If it still crashes, run `export LIBGL_ALWAYS_SOFTWARE=1` before launching.
- **Robot sags or collapses:** the femur servos may not hold 4 kg. Try `total_mass` 2.5 in `params.xacro` to compare. That's a real finding, not a bug.
- **Controllers fail to load:** check that `ros-humble-gz-ros2-control` and `ros-humble-ros2-controllers` are installed.

## Next steps

1. Measure in Gazebo how far the rover actually moves vs. commanded, and tune.
2. Run the nav stack together with gait_node (sim depth camera on the model).
3. Foot contact sensors (stand-in for the planned pressure sensors) to measure grounding.
