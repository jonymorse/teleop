# Unitree G1 29-DoF

This directory contains the first humanoid adapter for the XR teleoperation
portfolio. It uses Unitree Robotics' official current `g1_29dof_mode_11` URDF
and only the 34 mesh files referenced by that model.

Upstream source: `unitreerobotics/unitree_ros`

Pinned upstream commit: `d96d8f63ae17a7108d4f7229c00ef875ba7129c9`

The upstream assets are redistributed under the BSD 3-Clause license in
`UNITREE_LICENSE.txt`. Unitree Robotics does not endorse or sponsor this
independent portfolio project.

## Unity setup

Unity automatically creates `Prefabs/G1.prefab` and
`Assets/Teleoperation/Scenes/G1ImportTest.unity` after the new assets finish
importing. The same operation can be run manually from:

`Teleoperation > G1 > Import Official G1 and Create Test Scene`

The initial controller binds all 29 revolute joints, anchors the pelvis for a
stable first test, applies URP-compatible materials, enforces imported joint
limits, and owns the articulation drive targets instead of the URDF Importer's
legacy controller.

This is intentionally only the model foundation. The next milestone is a
robot-independent upper-body command interface followed by Quest head/hand
pose projection, calibration, safety validation, and network telemetry.

## Joint validation

Open `G1ImportTest.unity` and enter Play mode. The validation HUD shows the
selected joint, body group, target/measured angles, and imported limits. Use
Left/Right to cycle through all 29 joints, Up/Down to command the selected
joint, and Home to restore the neutral pose. The selected link receives a
subtle cyan highlight through a renderer property block; shared G1 materials
remain unchanged.

## Dual-arm position IK

The G1 prefab includes `G1DualArmIkController`. In Play mode it creates a
target at each rubber hand and temporarily disables keyboard joint validation.
Drag either sphere with the mouse in the Game view to command its independent
seven-joint arm chain. Green means the hand reached the target, orange means
the solver is converging, and red means the requested position is beyond the
configured arm reach. Target motion and per-step joint corrections are
rate-limited, and all commands remain clamped to the official imported limits.

This phase intentionally solves hand position only. Wrist orientation,
controller calibration, clutching, and Quest pose projection follow after both
desktop arm chains are validated.

The desktop test camera supports free navigation without interfering with
left-click IK dragging: hold the right mouse button to look, use WASD to move,
Q/E to descend/ascend, Shift to boost, and the mouse wheel to adjust movement
speed. The controls are also shown in the upper-right Game view HUD.

## Quest controller teleoperation

Open `Assets/Teleoperation/Scenes/G1XRTeleoperation.unity` for the Meta XR
passthrough workflow. It retains the working Meta Camera Rig, comprehensive
interaction rig, and Passthrough building block.

1. Hold both Touch controllers in a comfortable neutral pose and press right A
   to calibrate and align the IK targets to the current robot hands.
2. Hold the left or right grip to command that arm from relative controller
   motion. Release the grip to clutch and reposition without moving the robot.
3. Press the left Menu button to advance through operator modes or X to go
   backward. Meta locomotion owns the sticks only in Navigation.
4. Arm-control modes enter paused. Press right A to start from a fresh
   calibration, press A again to pause, and press right B to latch emergency
   stop.

Both position and orientation tracking must remain valid. Tracking loss disables
the IK solver and freezes commands. The headset-following HUD reports
calibration, tracking, clutch, mode, and emergency-stop state. This milestone
projects both controller position and orientation into the G1 arm chains.

Controller rotation is now projected into the corresponding seven-joint wrist
chain relative to the captured calibration/clutch orientation. Press either
thumbstick while commanding that side to reduce both positional and rotational
motion to 25% for precision work. Index triggers publish normalized 0–1
gripper commands to the operator HUD and public controller properties.

Pose visualization distinguishes three states: the axis-marked sphere is the
operator's requested pose, the small green cube is the latest rate-limited and
validated pose, and the physical robot hand is the measured articulation pose.
Requests beyond shoulder reach, inside the torso exclusion volume, or with the
two requested hands too close together are rejected and shown red. Wrist
orientation corrections are angular-rate limited and remain clamped by the
official wrist joint limits.

In Robot Alignment mode, physically touch the tracked right controller to the
desired floor or surface location and tap its index trigger. The controller position is
treated as the point beneath the G1's feet, and the saved pelvis-to-feet offset
keeps the humanoid at the correct standing height. The left stick provides
head-relative planar adjustment; right-stick horizontal rotates the G1 and
right-stick vertical adjusts height. Arm IK and Meta locomotion remain disabled
while Robot Alignment owns these inputs.

## Pick-and-place manipulation test

The G1 XR scene contains a waist-height table and three physics-enabled colored
boxes. Wrist-orientation correction is weighted toward wrist roll, pitch, and
yaw; shoulder and elbow joints receive only small fallback orientation weights
so controller rotation preserves hand position and arm posture more naturally.

Move a measured robot hand within 8 cm of the active box and squeeze that side's index
trigger past 70% to acquire it. The fixed-hand simulation adapter holds the box
at the measured hand, reports its name in the operator HUD, and provides a short
haptic pulse. Release below 25% to return it to physics with bounded hand
velocity and a lighter haptic pulse. Tracking loss, mode changes, or emergency
stop drive the gripper commands open. This adapter is intentionally labeled as
simulated grasping even when the articulated Dex3 representation is active;
no physical robot output is enabled by this Unity adapter.

## Quest finger tracking and Dex3-1

The project includes Unitree's official left/right Dex3-1 URDFs and STL link
meshes from `unitreerobotics/unitree_ros`, redistributed under the BSD
3-Clause license in `Dex3/UNITREE_ROS_LICENSE.txt`. Unity automatically
converts the sixteen STL files into Android-safe mesh assets through
`G1Dex3AssetSetup`.

At runtime `G1Dex3HandController` replaces the visible fixed rubber hands with
two articulated three-finger hands. Each hand has the official seven-joint
layout: three thumb joints, two index joints, and two middle joints. Simplified
colliders, official joint limits, smoothing, and angular rate limits are used
for stable Quest simulation.

To test:

1. Open `G1XRTeleoperation.unity`, enter Play mode or build for Quest, and use
   left Menu/X to select **Finger Tracking**.
2. Press right A while holding a neutral upper-body pose. The mode is now armed;
   put the controllers down and keep both bare hands visible to the headset.
3. Wrist and elbow tracking drive the arms. Human thumb and index curl map to
   their Dex3 counterparts; middle, ring, and little curl are averaged into
   the Dex3 middle finger.
4. Press A again before picking up the controllers to pause, or press B for
   emergency stop. Home mode returns both arms and all fourteen finger joints
   to their neutral pose.

### Controller-free palm menu

The Meta body source continues observing menu gestures while robot control is
paused, avoiding a dead end when Quest switches between Touch controllers and
bare-hand tracking. Hold the left palm toward the headset at a comfortable
viewing distance to reveal a floating menu, then hold one isolated pinch for
0.45 seconds:

- Thumb + index: next operator mode.
- Thumb + middle: previous operator mode.
- Thumb + ring: start or pause the current arm-control mode.
- Thumb + little finger: jump directly between Controller Teleoperate and
  Finger Tracking. The destination always opens paused.

Release the pinch before issuing another command. Palm-facing, distance,
hold-time, and other-fingers-open checks prevent ordinary manipulation pinches
from rapidly changing modes. Touch-controller Menu/X/A controls remain active
whenever the controllers are picked up again.

The UDP pose packet optionally includes `handRadians`, ordered as seven left
Dex3 joints followed by seven right Dex3 joints. The WSL backend validates and
echoes these commands for simulation only. Connecting those values to physical
Dex3 DDS topics remains deliberately disabled until hardware-specific safety,
force limits, and an operator deadman path are implemented.
