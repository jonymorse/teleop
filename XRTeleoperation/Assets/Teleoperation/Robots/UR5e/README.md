# UR5e Unity model

This folder contains a Unity-ready UR5e model generated from Universal Robots'
`Universal_Robots_ROS2_Description` release `4.3.1` (commit
`ae333289875f9ba5a9ea6649a54036efb5ccabee`).

The checked-in `ur5e.urdf` is expanded from `urdf/ur.urdf.xacro` with:

```text
ur_type:=ur5e name:=ur5e
```

The visual and collision meshes remain under `ur_description/` so the original
`package://ur_description/...` references resolve through Unity's URDF Importer.

## Import

1. In Unity, select `ur5e.urdf` in the Project window.
2. Choose **Assets > Import Robot from Selected URDF file**.
3. Keep **Axis Type** set to `Y Axis`, disable VHACD for this first import, and
   select **Import URDF**.
4. Add `UR5eJointController` to the imported `ur5e` root.
5. Use the component's context menu **Bind And Validate Joints**.
6. Save the imported root as `Prefabs/UR5e.prefab` and test it in a dedicated
   scene before adding it to the XR scene.

For the generated test scene, open **Teleoperation > UR5e > Open Joint
Controls**, enter Play mode, and use the six sliders in that window. If imported
materials ever appear pink, run **Teleoperation > UR5e > Fix Imported Materials
for URP**.

If the robot renders with a single flat color, run **Teleoperation > UR5e >
Restore Official Material Palette**. This creates URP materials from the colors
embedded in the official Collada models and remaps every model material slot.

The URDF Importer's legacy keyboard `Controller` is intentionally disabled and
removed. It depends on Unity's old Input Manager and conflicts with this
project's Input System configuration. `UR5eJointController` exclusively owns
the articulation drive targets.

In Play mode, Left/Right selects one of the six joints and Up/Down changes its
target using the new Input System. The controller also marks the root
`ArticulationBody` immovable so the robot stays anchored to the floor.
The selected link is highlighted orange using a per-renderer property override;
the shared official materials are not modified.

## Position IK prototype

Run **Teleoperation > UR5e > Add IK Solver to Current Scene** once, then enter
Play mode. A spherical target appears at `tool0`. Select the target in the
Hierarchy and move it with Unity's Move tool while Play mode is active. Yellow
means the solver is converging, green means it is within tolerance, and red
means the target is outside the configured workspace. This first slice solves
position only; tool orientation is intentionally deferred.
Direct arrow-key joint control is disabled while the IK solver component is
active, preventing both controllers from commanding the same drives.
IK corrections are timestep-scaled and based on measured joint positions rather
than previously commanded targets. The target position is rate-limited to avoid
physics oscillation while dragging.

The test camera supports Play-mode navigation in the Game view: hold the right
mouse button to look, use WASD to move, Q/E to descend/ascend, and hold Shift
for faster movement. Dragging the IK target with Unity's transform gizmo still
uses the Scene view.
In the Game view, the IK sphere can also be moved directly: left-click the
sphere and drag. Its collider is a trigger, so it does not participate in robot
physics.

## XR teleoperation scene

`UR5eXRTeleoperation.unity` is generated from the Meta-configured
`SampleScene`, retaining `[BuildingBlock] Camera Rig` and `[BuildingBlock]
Passthrough`. Point either Quest controller at the target and hold its index
trigger to clutch and command IK. Releasing the trigger stops command updates
and holds the current pose. On the right controller, A resets the target to
`tool0`; B toggles emergency stop. The scene is added to Build Settings.

The left controller Menu button cycles Placement, IK, FK, and Navigation modes. Placement
uses the left stick for head-relative floor movement and the right stick for
yaw/height. Physically touch the right controller to a desired real-world
location and tap its index trigger to place the robot base at the controller's
tracked position, shown by the green marker. IK uses
controller ray + index trigger clutching. FK uses left-stick
horizontal movement to select a highlighted joint and right-stick vertical
movement to command it. The right Meta/Quest button remains OS-reserved.
An XR world-space HUD follows the headset and shows the current mode, selected
FK joint, clutch state, and emergency-stop state. Navigation gives the Meta
locomotion rig exclusive control of the sticks; locomotion and its comfort
tunneling effect are disabled in all robot-control modes. Meta XR Simulator's separate
`Move by Thumbsticks` option must remain disabled; otherwise the simulator moves
the synthetic player regardless of the application's current mode.

The six expected revolute joints are:

- `shoulder_pan_joint`
- `shoulder_lift_joint`
- `elbow_joint`
- `wrist_1_joint`
- `wrist_2_joint`
- `wrist_3_joint`

The robot controller clamps every requested target to the limits imported onto
the corresponding `ArticulationBody` drive.
