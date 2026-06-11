# Finger Flick — MuJoCo Simulation

A small MuJoCo C++ simulation in which a 3-DOF robotic finger actively **flicks a
swinging pendulum**. A ball hangs from a hinged rod and swings back and forth; a
controller watches the rod angle and velocity and, at the right moment, curls the
finger to swat the ball as it approaches. The scene renders in an interactive 3D
window with mouse-controlled camera.

This project uses the **native MuJoCo C/C++ API** with **GLFW** for windowing and
rendering — not the Python bindings.

---

## Contents

| File | Purpose |
|------|---------|
| `pendulum_test.cpp` | Simulation loop, controller state machine, and GLFW/MuJoCo rendering. |
| `test.xml` | MuJoCo MJCF model: the finger, the pendulum, actuators, and a start keyframe. |
| `kinematics.h` | **Not included here.** Referenced by `pendulum_test.cpp` via `#include "kinematics.h"`. You must supply this header (or remove the include if unused). |
| STL meshes | `base_link.STL`, `splay_link.STL`, `mcp_link.STL`, `pip_link.STL` — referenced by the model. **Not included here.** You must supply these. |

---

## What the simulation does

### The model (`test.xml`)

The world contains two independent mechanisms:

**1. The finger** — a serial chain of three actuated joints mounted to a fixed base:

- `splay_joint` — sideways spread (axis Z), sprung and damped, held at 0 by the controller.
- `mcp_joint` — the "knuckle" (MCP), range ±1.57 rad.
- `pip_joint` — the middle joint (PIP), range ±1.57 rad. The PIP link carries a
  capsule collision geom (`pip_geom`) — this is the part that actually strikes the ball.

Each joint is driven by a MuJoCo **`position` actuator** with a proportional/derivative
gain (`kp`/`kv`), so writing a target angle to `d->ctrl[i]` servos that joint toward it.

**2. The pendulum** — a non-colliding support box with a hinged rod hanging below it
and a sphere (`ball_geom`) at the rod's tip. The hinge is almost frictionless
(`damping="0.00025"`), so once released it swings freely under gravity.

Other model settings worth knowing:

- Gravity `-9.81`, timestep `0.001 s`, implicit integrator, contact enabled.
- A keyframe named **`start`** sets the initial pose `qpos = "0 0 0 -1.4"`
  (splay, MCP, PIP at 0; rod pulled back to −1.4 rad so it swings in on release).
- Contact parameters (`solref`, `solimp`, `friction`) are tuned on `pip_geom` and
  `ball_geom` so the strike reads as a crisp swat.

The generalized coordinates are, in order:
`qpos[0]=splay`, `qpos[1]=MCP`, `qpos[2]=PIP`, `qpos[3]=rod hinge`.

### The controller (`pendulum_test.cpp`)

The main loop steps the physics once per iteration (and sleeps to keep the sim
roughly real-time), reads joint states, then runs a three-state machine that drives
the finger:

- **READY** — finger held open. It waits until the rod swings past a trigger angle
  while moving toward the finger, then transitions to FLICK.
- **FLICK** — commands the MCP and PIP joints to curl (targets `0.8` and `1.2` rad)
  for a short burst (~0.1 s) to swat the ball.
- **RESET** — returns the finger to open and holds for ~0.4 s before re-arming.

Alongside the state machine, the loop checks `d->contact[...]` every step for a
collision between `pip_geom` and `ball_geom` to flag when an actual hit occurs.
Every 50 steps it prints the current state, rod angle/velocity, and joint angles to
the console; every 10 steps it redraws the window.

> **Tuning:** the trigger angle, approach-velocity threshold, flick/reset durations,
> and the curl targets are all `const` knobs near the top of the loop. Adjust them
> while watching the printed `Rod` values to dial in the timing for your build.

### Interactive controls

| Input | Action |
|-------|--------|
| Left-drag | Rotate camera |
| Right-drag | Pan camera |
| Shift + drag | Alternate rotate/pan axis |
| Scroll | Zoom |
| Esc | Quit |

---

## Prerequisites

- A C++17 compiler (GCC, Clang, or MSVC).
- **MuJoCo** (the C/C++ library). Version 3.x is recommended — see install below.
- **GLFW 3** development libraries.
- **OpenGL** (present on essentially all desktop systems).
- **CMake** ≥ 3.16 (recommended for the build).

---

## Installing MuJoCo

MuJoCo is open source (Apache-2.0, maintained by Google DeepMind). For C/C++ work the
easiest route is the **prebuilt library release** — there is no installer; it ships as
a single archive (`.tar.gz` on Linux, `.dmg` on macOS, `.zip` on Windows).

1. Go to the MuJoCo GitHub releases page:
   <https://github.com/google-deepmind/mujoco/releases>
2. Download the latest release for your platform (3.2.7 or newer at time of writing).
3. Extract it somewhere stable, e.g. `~/.mujoco/mujoco-3.x.x/`. You should end up with
   `include/` (headers) and `lib/` or `bin/` (the shared library).

> If you prefer, you can also build MuJoCo from source (needs CMake + a C++17
> compiler), but the prebuilt library is recommended unless you intend to modify
> MuJoCo's core.

### GLFW

- **Ubuntu/Debian:** `sudo apt install libglfw3-dev`
- **macOS (Homebrew):** `brew install glfw`
- **Windows:** download from <https://www.glfw.org/> or install via `vcpkg install glfw3`.

---

## Project layout and file paths

⚠️ **The paths in the source are relative and must line up, or the model won't load.**

- `pendulum_test.cpp` loads the model from `../../finger_description/urdf/test.xml`.
- `test.xml` in turn references its meshes at `../meshes/*.STL`.

So the model and meshes are expected to live like this, with the executable run from a
directory **two levels below** the parent of `finger_description/`:

```
project/
├── finger_description/
│   ├── urdf/
│   │   └── test.xml
│   └── meshes/
│       ├── base_link.STL
│       ├── splay_link.STL
│       ├── mcp_link.STL
│       └── pip_link.STL
└── build/
    └── bin/
        └── pendulum_test     # run from here → ../../finger_description/urdf/test.xml
```

If your layout differs, either move the files to match, or edit the path in the
`mj_loadXML(...)` call (and the mesh paths in `test.xml`) accordingly. Using absolute
paths while you get it running first is a reasonable shortcut.

---

## Building

### Option A — CMake (recommended)

Create a `CMakeLists.txt` next to `pendulum_test.cpp`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(finger_flick CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Point this at your extracted MuJoCo directory
set(MUJOCO_DIR "$ENV{HOME}/.mujoco/mujoco-3.2.7" CACHE PATH "MuJoCo root")

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(pendulum_test pendulum_test.cpp)

target_include_directories(pendulum_test PRIVATE
    ${MUJOCO_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR})      # so kinematics.h is found

target_link_directories(pendulum_test PRIVATE ${MUJOCO_DIR}/lib)

target_link_libraries(pendulum_test PRIVATE
    mujoco
    glfw
    OpenGL::GL)
```

Then build:

```bash
mkdir -p build && cd build
cmake .. -DMUJOCO_DIR=/path/to/mujoco-3.2.7
cmake --build .
```

### Option B — direct compile (Linux)

```bash
g++ -std=c++17 pendulum_test.cpp -o pendulum_test \
    -I/path/to/mujoco/include -I. \
    -L/path/to/mujoco/lib -lmujoco -lglfw -lGL
```

On Linux you may also need the library on the loader path at runtime:

```bash
export LD_LIBRARY_PATH=/path/to/mujoco/lib:$LD_LIBRARY_PATH
```

(macOS uses `DYLD_LIBRARY_PATH`; on Windows, place `mujoco.dll` next to the
executable or add its folder to `PATH`.)

---

## Running

From the directory that makes the relative model path resolve (see **Project layout**):

```bash
./pendulum_test
```

You should see:

- A console message confirming the `start` keyframe loaded.
- A 3D window with the floor, the finger, and the swinging ball pendulum.
- Periodic console output like:

  ```
  State: READY  | Rod: -1.182 (vel 0.642) | MCP: 0.000 PIP: 0.000
  Ball approaching = 1
  ```

The ball swings in, the finger flicks, and the loop re-arms. Drag with the mouse to
orbit the camera; press **Esc** to quit.

---

## Troubleshooting

- **`Error loading model: ...`** — the XML path or a mesh path is wrong. Confirm the
  directory layout above, or hard-code an absolute path in `mj_loadXML`.
- **`Keyframe not found`** — the model loaded but the `start` key is missing or renamed;
  the sim still runs, just from the default pose.
- **Linker errors for `mj_*` / `mjv_*` / `mjr_*`** — MuJoCo include/lib paths are wrong,
  or `-lmujoco` is missing.
- **Linker errors for `glfw*`** — GLFW dev package not installed or not linked.
- **`cannot open shared object file: libmujoco...`** at runtime — set
  `LD_LIBRARY_PATH` (Linux) / `DYLD_LIBRARY_PATH` (macOS), or put the DLL beside the
  exe (Windows).
- **`kinematics.h: No such file or directory`** — supply that header and make sure its
  directory is on the include path (`-I.` or `target_include_directories`).
- **The finger never flicks / always misses** — tune `flick_trigger_angle`, the
  `rod_vel` threshold, and `flick_duration` while watching the printed `Rod` values.

---

## Notes & references

- MuJoCo docs: <https://mujoco.readthedocs.io/>
- MuJoCo releases: <https://github.com/google-deepmind/mujoco/releases>
- The controller is intentionally simple (open-loop curl on a timed trigger). It's a
  good starting point for experimenting with contact-aware or feedback control of the
  strike.
