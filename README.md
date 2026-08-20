# 🐕 Fast Occupancy Grid (FOG) 2.5D Elevation Mapping

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-34a853?style=for-the-badge&logo=ros)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04_LTS-E95420?style=for-the-badge&logo=ubuntu)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python)

This repository implements a 2.5D elevation mapping and obstacle segmentation pipeline
for quadruped robot locomotion, reimplementing the core method of:

> J.-S. Gutmann, M. Fukuchi, M. Fujita, *"A Floor and Obstacle Height Map for 3D
> Navigation of a Humanoid Robot,"* ICRA 2005.

...adapted from the paper's stereo-vision + humanoid setup to a **LiDAR-inertial,
quadruped** setting using FAST-LIO2 (Livox Mid-360 + IMU).

---

## ⚠️ READ THIS FIRST — Before you rely on this README anywhere without a backup device

**Do this right now, tonight, while you still have your own machine:** push your
entire `~/FOG_Quadruped_ws/src/` folder — all three package folders
(`livox_ros_driver2`, `FAST_LIO`, `legged_height_map`) — to your public repo, not
just `legged_height_map` alone. Do **not** push `build/`, `install/`, or `log/` —
those are always regenerated locally by `colcon build` and should never be
committed; that's normal, not something to fix.

**Why this matters more than the "correct" fallback instructions below:** the
fallback path (cloning each dependency fresh from its own upstream repo) depends on
three separate external GitHub repos being reachable, unchanged, and buildable
tomorrow, exactly as they are tonight. That's three additional points of failure
you don't control, at a moment when you have zero recovery devices if any of them
breaks. If your own repo already contains the exact, already-tested-tonight source
for all three packages, tomorrow's setup becomes one clone and one build of code
that is *already known to work* — nothing upstream can surprise you.

The fallback instructions are still below, fully verified, in case you're demoing
on a machine where you specifically need to show a from-scratch build against
public upstream sources (e.g. if asked to prove you didn't just vendor a
pre-built binary). Use whichever your situation calls for.

---

## 🏗️ System Architecture & Data Flow

1. **Sensor Input (Rosbag):** Streams raw Livox Mid-360 LiDAR point clouds
   (`/livox/mid360/lidar`, `livox_ros_driver2/msg/CustomMsg`) and IMU data
   (`/livox/mid360/imu`) into the system.
2. **Odometry & SLAM (FAST-LIO):** Fuses LiDAR and IMU to produce a gravity-aligned,
   world-frame registered point cloud (`/cloud_registered`) and robot pose
   (`/Odometry`). This is the paper's "robot pose is given" assumption (Section IV) —
   FAST-LIO solves it so the FOG algorithm doesn't have to.
3. **Spatial Filtering:** Bounding-box crop to the local map window, self-body
   exclusion (points too close to the robot are sensor noise, not terrain), and a
   height-based ceiling filter (`rel_z < 0.5`) — see the honest note on this in the
   Mathematical Framework section below.
4. **Iterative RANSAC Segmentation:** Labels each remaining point as floor-candidate
   (belongs to a detected near-horizontal-or-inclined plane) or obstacle
   (everything else), across up to `plane_max_count` planes per scan — this is what
   lets the map handle multiple floor levels (steps, ramps), not just one flat floor.
5. **FOG Generation & Ray Tracing:** Every point — floor or obstacle — is ray-traced
   from the sensor origin into a 3D log-odds occupancy grid, injecting both positive
   evidence (hit) and negative evidence (line-of-sight passed through empty). This
   negative evidence is what lets the map *forget* stale detections.
6. **Cell Extraction & Output:** Per-cell floor height and obstacle height are pulled
   from the occupancy-gated grid and combined into a colored FOG cloud, published
   live and exportable as `.pcd`.

---

## 🔍 Full Walkthrough — How One Scan Becomes a FOG Map

This section exists so someone who has never seen this codebase can trace exactly
what happens to a single LiDAR scan, end to end, without reading the source first.

1. **A raw scan arrives.** The Mid-360 publishes a `CustomMsg` — Livox's own point
   format, not the ROS-standard `PointCloud2` — at roughly 10Hz.
2. **FAST-LIO consumes it.** Internally, FAST-LIO fuses this scan with the IMU
   stream since the last scan (at ~200Hz) to estimate how the robot moved in that
   interval, corrects the point cloud for that motion (removing the smearing a
   quadruped's walking gait would otherwise cause), and re-publishes the corrected
   points in a fixed world frame as standard `sensor_msgs/PointCloud2` on
   `/cloud_registered`, alongside the updated pose on `/Odometry`.
3. **The FOG node receives both.** `mapping_node.py` subscribes to both topics. The
   odometry callback just tracks the robot's current `(x, y, z)`; the cloud callback
   does the actual work, described below, every time a new scan arrives.
4. **Filtering.** Points outside the local 4×4m window are dropped (this is a
   *local*, not global, map — see Section IV note below). Points within 0.25m of the
   robot are dropped (that's the robot's own body/legs, not terrain). Points more
   than 0.5m above the robot are dropped (ceiling exclusion).
5. **Plane segmentation.** The remaining points are handed to iterative RANSAC: fit
   a plane to 3 random points, count how many other points lie within
   `plane_dist_threshold` of it, keep the best-fitting plane found in 100 random
   trials, remove its inlier points from consideration, and repeat up to
   `plane_max_count` times. Each accepted plane is checked against
   `plane_normal_angle_deg` — if its normal is close enough to vertical, its points
   are floor-candidates; otherwise they're obstacle points. Whatever's left over
   after all iterations is also obstacle.
6. **Ray tracing.** For every point (floor or obstacle), three intermediate points
   are sampled along the straight line from the sensor to that point (at 33%, 66%,
   85% of the way there). Those intermediate cells get `l_free` added to their
   log-odds score (evidence of empty space); the actual endpoint cell gets `l_occ`
   added (evidence of a hit). This is the numerically-stable log-odds form of the
   paper's Eq. 6 Bayesian update — standard in occupancy-grid SLAM.
7. **Thresholding.** A cell only counts as "occupied" once its accumulated log-odds
   score exceeds the log-odds equivalent of `p_occ_threshold`. This gate is what
   Eq. 7 in the paper calls requiring "enough probability" before trusting a cell.
8. **Floor & obstacle height extraction.** For cells with enough occupied evidence:
   floor height = the actual measured z of a floor-candidate point that landed
   there; obstacle height = the z of the highest occupied layer in that column
   (Eq. 10). If a cell was previously floor but no longer clears the occupancy
   threshold, it's reset to unknown — this is the "forget" mechanism the paper's
   3D grid provides that a simple running-average height map cannot.
9. **Combine and publish.** For each cell with any valid height: if there's both a
   floor and an obstacle reading and the obstacle sits meaningfully above the floor,
   the cell renders red (obstacle). Otherwise, if there's a floor reading, it
   renders green. This publishes as a colored `PointCloud2` on
   `/legged_height_map/fog_cloud`.
10. **Recentering.** As the robot moves, the whole grid — occupancy, floor, obstacle
    — shifts via array rolling to stay centered on the robot's current position,
    clearing the newly-exposed edge back to unknown. Old terrain outside the window
    is genuinely forgotten, not retained — matching Section IV of the paper exactly.

---

## 📐 Mathematical Framework (Gutmann's Logic, Mapped to This Code)

### 1. Bayesian Ray Tracing — Eq. 6 (the "forget" mechanism)

Standard height-averaging maps suffer from transient noise permanently occupying
grid cells, because they only ever add evidence, never subtract it. We implement
fractional line-of-sight ray tracing to inject **negative** evidence as well:

$$L_{t}(cell) = L_{t-1}(cell) + \Delta L, \qquad \Delta L = \begin{cases} +l_{occ} & \text{cell contains the hit point} \\ +l_{free} & \text{cell lies on the ray, before the hit (}l_{free} < 0\text{)} \end{cases}$$

This log-odds form is mathematically equivalent to the paper's Bayesian update
(Eq. 6) — it's the standard numerically-stable way to implement a recursive binary
Bayes filter per cell, avoiding repeated division and overflow.

### 2. Floor Height Gating — Eq. 7–9

A cell's floor height is only trusted once its 3D occupancy clears `P_occ`:

$$OG(x, y, h) > P_{occ} \implies \text{Floor}(x,y) \text{ may be updated}$$

Below that threshold, the cell is `NaN` (unknown) — never a false "floor at height
zero," which was an actual bug in an earlier version of this code (see
Troubleshooting).

### 3. Obstacle Height — Eq. 10

$$Obstacle(x,y) = \max\{z : OG(x,y,z) > P_{occ}\}$$

The highest occupied layer in a column, computed by scanning down from the top of
the occupancy mask — this is a coarse estimate, deliberately: the paper only asks
for precision on the floor height (where a foot lands), not on obstacles (where the
robot only needs "avoid," not "measure exactly").

### 4. Final Classification — Eq. 11

*Correction from an earlier draft of this document:* classification is **not** a
per-cell `Z_max - Z_ground` height-difference test in this implementation — that
was a simplification in an earlier description that didn't match the actual code.
What the code actually does: floor/obstacle labeling happens at the **point** level
during RANSAC (step 5 above), before any grid cell math. The per-cell height
comparison you'll see in the code (`vis_epsilon`) is a separate, later step — it's
the paper's Eq. 11 rendering rule: if a cell has *both* a floor reading and an
obstacle reading, and the obstacle sits more than `epsilon` above the floor, draw it
as an obstacle (red); otherwise draw the floor (green). This mirrors the paper's own
`δ` term in Eq. 11 — "a small threshold that accounts for noise... by ignoring small
obstacles close to an existing floor height."

### 5. Honest note on the ceiling filter

`rel_z < 0.5` is **not** part of Gutmann's method — it's a practical, sensor-mount-
specific height prior we added. RANSAC's plane-normal check genuinely cannot
distinguish a flat ceiling from a flat floor on its own (both have a vertical
normal); this hard cutoff resolves the ambiguity for this specific rig's mounting
height. If asked how the algorithm tells floor from ceiling "in principle," the
accurate answer is "it doesn't — this is a practical prior for our setup," not
"the plane classifier resolves it."

### 6. Honest note on the local window

The exported `.pcd` is a snapshot of the **live, robot-centric 4×4m window** at the
moment of capture — not an accumulated map of the entire trajectory. This is
deliberate and matches the paper precisely (Section IV: *"the maps are centered on
the robot... as the robot moves, older parts of the maps are erased and new terrain
is entered"*). If asked whether the saved map covers the whole run: no, by design.

---

## 🎛️ Parameter Reference (with reasoning)

| Parameter | Value | Why |
|---|---|---|
| `grid_resolution` | 0.05 m | Matches expected foot-placement precision |
| `map_size` | 4.0 m | Matches the paper's own QRIO map footprint (Section IV) |
| `num_layers` / `layer_height` | 60 / 0.05 m | 3m vertical range, enough for indoor steps/ramps |
| `p_occ_threshold` | 0.55 | Lower than a strict 0.65+ — chosen after tuning showed cells were flickering to unknown too easily at higher values |
| `l_occ` / `l_free` | 0.85 / -0.40 | Standard log-odds hit/miss magnitudes; asymmetric so a single hit registers faster than a single miss erases it |
| `plane_dist_threshold` | 0.05 m | RANSAC inlier distance; tightened from an earlier 0.08m once outlier rejection (RANSAC) replaced plain least-squares |
| `plane_max_count` | 6 | Raised from an initial 3 to capture multi-level steps/stairs reliably |
| `plane_normal_angle_deg` | 45° | Widened from the original 15° specifically so steep stair treads and ramps still classify as traversable floor — a deliberate quadruped-specific tradeoff, not a paper default |
| `floor_gate_epsilon` | 0.08 m | Step-clearance gate; matches typical quadruped foot clearance tolerance |

---

## ⚡ Prerequisites & Dependency Build — CORRECTED

**Base system:**

```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-pip \
  build-essential cmake git \
  libeigen3-dev libpcl-dev pcl-tools
```

*(`ros-humble-fast-lio` removed — confirmed this package does not exist anywhere in
the ROS index or apt; FAST-LIO is source-build only, every time, on every machine.)*

**Quick sanity check before you go further** — confirms the Python message helper
your node imports is actually present:

```bash
python3 -c "import sensor_msgs_py.point_cloud2; import numpy; print('OK')"
```

If that fails, `sudo apt install ros-humble-sensor-msgs-py` and re-check.

### RECOMMENDED PATH — your own repo already has everything

```bash
mkdir -p ~/FOG_Quadruped_ws/src
cd ~/FOG_Quadruped_ws/src
git clone https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git .
# ^ note the trailing "." — clones repo CONTENTS directly into src/,
#   so livox_ros_driver2/, FAST_LIO/, and legged_height_map/ all land
#   as sibling package folders, exactly as they were built and tested tonight.
```

### FALLBACK PATH — building each dependency fresh from upstream (verified URLs)

Only use this if your own repo doesn't already contain all three packages.

**1. Livox-SDK2 (C++ SDK, required before the ROS driver will build):**
```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

**2. livox_ros_driver2** — ⚠️ its `build.sh` deletes `build/`, `devel/`, `install/` in
whatever workspace it's run from. Build this **first**, before anything else, in
this workspace, and never re-run `build.sh` after this point or it will wipe your
other built packages:
```bash
cd ~/FOG_Quadruped_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
```

**3. FAST_LIO (Ericsii's ROS2 port — confirmed current maintainer of the ROS2 branch):**
```bash
cd ~/FOG_Quadruped_ws/src
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive FAST_LIO
```

**4. Your own `legged_height_map` package:**
```bash
cd ~/FOG_Quadruped_ws/src
git clone https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git legged_height_map
```

### Step 2: Build the Workspace — CORRECTED

```bash
cd ~/FOG_Quadruped_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

*(Changed from `--packages-select legged_height_map` to a full `colcon build` — the
original would only ever build one of the three required packages, and `fast_lio`
would never exist for Terminal 1 to launch.)*

---

## ⚡ Step-by-Step Execution Guide — UNCHANGED FROM ORIGINAL

*⚠️ Important: You must run `source install/setup.bash` in every single terminal
before execution.*

**💻 Terminal 1 (SLAM Backbone):**
```bash
cd ~/FOG_Quadruped_ws
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=quad_mid360.yaml
```

**💻 Terminal 2 (FOG Mapping Algorithm):**
```bash
cd ~/FOG_Quadruped_ws
source install/setup.bash
ros2 run legged_height_map mapping_node
```

**💻 Terminal 3 (PCD Map Saver Node):**
```bash
cd ~/FOG_Quadruped_ws
source install/setup.bash
ros2 run legged_height_map pcd_saver_node
```

**💻 Terminal 4 (Play Sensor Dataset):**
```bash
cd ~/FOG_Quadruped_ws
source install/setup.bash
ros2 bag play ~/Downloads/Occlusion01_ros2 -r 0.3
```

---

## 💾 Map Serialization & Output

To capture a snapshot of the live 2.5D map:
1. Switch to **Terminal 3** during active mapping.
2. Press **`Ctrl + C`**. The rolling 4x4m grid is instantly serialized.
3. View the 3D map offline:

```bash
pcl_viewer ~/Downloads/final_fog_map.pcd
```

**Fallback viewer** (in case `pcl_viewer` isn't available on the demo machine —
needs only numpy + matplotlib, both near-universally present):

```python
import numpy as np
import matplotlib.pyplot as plt

path = "/home/YOUR_USER/Downloads/final_fog_map.pcd"
pts = []
with open(path) as f:
    lines = f.readlines()
    data_start = next(i for i, l in enumerate(lines) if l.startswith("DATA")) + 1
    for line in lines[data_start:]:
        vals = line.split()
        if len(vals) >= 3:
            pts.append([float(vals[0]), float(vals[1]), float(vals[2])])
pts = np.array(pts)
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=2)
plt.show()
```

---

## 🐛 Troubleshooting & Error Handling

| Error / Issue | Root Cause | Solution Implemented |
| :--- | :--- | :--- |
| **Solid Red Map** | Positive hits permanently occupied cells; no negative evidence anywhere in the grid. | Implemented fractional ray-tracing to inject `l_free` along the line of sight. |
| **Ceiling = Ground** | 360° LiDAR scanned the flat ceiling; same normal as floor, RANSAC can't tell them apart. | Added `rel_z < 0.5` spatial height prior (practical fix, not part of the paper — see math section). |
| **Stairs/Ramps Rejected** | Default RANSAC angle (15°) too strict for sloped surfaces. | Widened `plane_normal_angle_deg` to 45°, `plane_max_count` to 6. |
| **Grid Shifting Drift** | `int()` truncates toward zero, not floor — under-shifts on negative motion. | Replaced with `int(np.floor(dx / self.res))`. |
| **`ros2 bag play` silently drops LiDAR topic** | Terminal running `ros2 bag play` never sourced the workspace, so it can't deserialize `livox_ros_driver2/msg/CustomMsg` — falls back to system ROS only and discards the topic with a warning. | `source install/setup.bash` in *every* terminal, including the one just playing the bag. |
| **"No executable found" on `ros2 run`** | `entry_points` in `setup.py` was empty and/or the node's `.py` file was accidentally left 0 bytes. | Register `'mapping_node = legged_height_map.mapping_node:main'` (and same for the saver) in `setup.py`, confirm the file isn't empty with `ls -la`, rebuild. |
| **`sudo apt install ros-humble-fast-lio` fails** | Package does not exist — FAST-LIO has no apt/ROS-index release, ever. | Build from source: `Ericsii/FAST_LIO_ROS2`, see Prerequisites section. |
| **Isolated/floating point cluster in saved `.pcd`** | Not a bug in the saver (it holds only the single latest message, no cross-time accumulation) — most likely genuine geometry that fell inside the local window at the moment of capture. | Confirm by checking RViz live at that same moment before assuming it's an artifact. |

---
*Developed by Harshal Ghadge | Computer Engineering, Saraswati College of Engineering.*
