# G1 teleoperation backend

This is the simulation-only boundary between the Quest Unity frontend and Unitree's official
[`xr_teleoperate`](https://github.com/unitreerobotics/xr_teleoperate) G1 arm solver. It never imports
or calls Unitree's real-robot DDS controller.

## Network test

On the PC connected to the same Wi-Fi network as the Quest:

```powershell
python tools/g1_backend/g1_teleop_backend.py --connection-test
```

Allow Python through Windows Firewall for private networks. In the headset, cycle the Left Menu
button to **BACKEND TELEOP**. The HUD should change from `discovering backend` to `backend connected`.
The connection test intentionally holds the current joint angles.

## Official Unitree IK

Unitree's supported setup is Ubuntu 20.04/22.04 with Python 3.10, Pinocchio 3.1, CasADi and the
dependencies from `xr_teleoperate`. After installing that repository according to its README:

```bash
python tools/g1_backend/g1_teleop_backend.py --unitree-repo ~/xr_teleoperate
```

On this project's configured Windows/WSL2 machine, launch the installed solver from PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File tools/g1_backend/start_wsl_backend.ps1
```

You can also double-click `Start G1 Backend.cmd` in the repository root. The backend must be
started again after every Windows restart and its window must remain open. Wait for the green
`READY` message before selecting **BACKEND TELEOP** in the headset.

## Quest hand retargeting

**QUEST HAND RETARGET** is a separate mode. It keeps **BACKEND TELEOP** unchanged for Touch
controllers, but sends the full 25-keypoint skeleton from each bare Quest hand through Unitree's
official Dex3 retargeter. The mode freezes its last hand pose if either skeleton is lost.

The bundled retargeter submodule must be installed once in the WSL environment:

```bash
/home/jomo/miniforge3/envs/tv/bin/pip install --no-deps --no-build-isolation -e \
  /home/jomo/Projects/xr_teleoperate/teleop/robot_control/dex-retargeting
```

Start the normal backend, switch the Quest from controllers to hand tracking, use the left-palm
menu to select **QUEST HAND RETARGET**, and hold **Ring** on that menu to start or pause control.
Keep both hands visible during operation. **Index** advances and **Middle** goes back through modes.

The launcher starts the solver on WSL localhost port 7547 and a Windows relay on LAN ports
7447–7448. This is necessary because WSL does not reliably receive subnet UDP broadcasts.
Windows/Hyper-V firewall rules allow only UDP 7447–7448 from the LAN.

The bridge broadcasts versioned wrist targets on UDP 7447 and receives 14 arm joint angles on
UDP 7448. Pose coordinates use Unitree's convention: X forward, Y left, Z up. Both devices must be
on the same LAN and UDP 7447/7448 must be allowed by the host firewall.

The Quest bridge is configured to contact this workstation directly at `192.168.1.209`, because
many Wi-Fi access points filter Android's global UDP broadcast. If the PC's LAN address changes,
update `backendHost` in `G1TeleoperationBridge.cs` and rebuild the APK.

In **BODY TRACKING** or **BACKEND TELEOP**, hold both hands in a comfortable neutral orientation
and press the right controller **A** button to recapture the wrist-frame offsets and clear the
solver's warm-start/filter history. Use this whenever a wrist becomes trapped in an awkward pose.

## Safety boundary

Protocol version 1 requires `simulationOnly: true`. This backend has no real-robot output path.
Connecting a physical G1 requires a separate reviewed process with watchdogs, velocity/torque
limits, collision checks, an operator enable switch, and a hardware emergency stop.
