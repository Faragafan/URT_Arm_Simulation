# URT Arm Sim

Standalone URDF-based simulation workspace for the URT arm.

The checked-in model is repo-relative: no user-specific absolute paths are required.

## Requirements

- Python 3.10+
- PyBullet for the Python viewer:

```powershell
py -m pip install pybullet
```

- Rust toolchain for the Bevy viewer and `k` kinematics tools:

```powershell
rustup default stable
```

## Contents

- `assets/urdf/urdf_assembly_rigid_stl_collapsed.urdf`: collapsed 6-joint arm URDF
- `assets/meshes_onshape_rigid_stl/`: STL visual meshes referenced by the URDF
- `tools/view_urdf_pybullet.py`: PyBullet URDF viewer
- `tools/bevy_viewer/`: Bevy URDF viewer with `k` FK/IK startup support
- `tools/k_kinematics/`: Rust `k` crate FK/IK command-line demo
- `tools/collapse_fixed_urdf.py`: converts an Onshape URDF export zip into a collapsed robot URDF

## Check URDF

From the repo root:

```powershell
powershell -ExecutionPolicy Bypass -File tools\check_urdf.ps1
```

## Run PyBullet Viewer

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_urdf_viewer.ps1
```

Use debug boxes if mesh rendering is slow:

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_urdf_viewer.ps1 -BoxVisuals
```

## Run Bevy Viewer

Manual joint-space viewer:

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_bevy_viewer.ps1
```

Start Bevy at specific joint angles:

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_bevy_viewer.ps1 -Joints "0.2,0.3,-0.4,0.1,0.2"
```

Start Bevy from a task-space target. This uses `k` internally to solve IK before opening the window. `TargetXyz` is meters; `TargetRpy` is roll/pitch/yaw in radians.

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_bevy_viewer.ps1 -TargetXyz "-0.20,0.30,0.28"
```

Start Bevy with both target position and target orientation:

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_bevy_viewer.ps1 -TargetXyz "-0.20,0.30,0.28" -TargetRpy "0,0,0"
``` 

Dry-run the Bevy/k integration without opening the window:

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_bevy_viewer.ps1 -TargetXyz "-0.20,0.30,0.28" -DryRun
```

Use a lower triangle cap if rendering is slow:

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_bevy_viewer.ps1 -TriangleCap 8000
```

Bevy controls after the window opens:

```text
Q/A -> revolute_1
W/S -> revolute_2
E/D -> revolute_3
R/F -> revolute_4
T/G -> revolute_5
Y/H -> revolute_6
M -> toggle realtime task-space IK
ArrowLeft / ArrowRight -> move target X
ArrowDown / ArrowUp -> move target Y
PageDown / PageUp -> move target Z
Z / X -> target roll
C / V -> target pitch
B / N -> target yaw
Space -> reset joint values to zero when task-space IK is off
Mouse drag -> orbit camera
Mouse wheel -> zoom
```

When realtime task-space IK is enabled, the red cube is the target pose. Moving it calls `k` IK and writes the solved joint angles back into the Bevy arm. If no `TargetRpy` is supplied, the solver keeps the current end-effector orientation; once you pass `TargetRpy` or press the roll/pitch/yaw keys, it solves against the requested orientation too.

## Run Kinematics With `k`

Forward kinematics from joint angles:

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_k_kinematics.ps1 -Joints "0.2,0.3,-0.4,0.1,0.2,-0.1"
```

Inverse kinematics for the target link with position only:

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_k_kinematics.ps1 -Joints "0,0,0,0,0,0" -TargetXyz "-0.20,0.30,0.28"
```

Full-pose inverse kinematics with explicit target orientation:

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_k_kinematics.ps1 -Joints "0,0,0,0,0,0" -TargetXyz "-0.20,0.30,0.28" -TargetRpy "0,0,0"
```

The current arm has 6 movable joints. Without `TargetRpy`, IK keeps the current end-effector orientation. With `TargetRpy`, it solves the full target pose.

## Updating From a New Onshape Export

Export from Onshape as `URDF` with `Geometry format: STL`, then run:

```powershell
py tools\collapse_fixed_urdf.py --input "C:\path\to\URDF assembly.zip" --output assets\urdf\urdf_assembly_rigid_stl_collapsed.urdf --mesh-output-dir assets\meshes_onshape_rigid_stl
```

Then re-check the URDF:

```powershell
powershell -ExecutionPolicy Bypass -File tools\check_urdf.ps1
```


