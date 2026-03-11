# Multi-UAV Slung Load Simulation

A parametric Gazebo / PX4 SITL simulation of 3 × x500 quadrotors carrying a shared payload via flexible chains.

![Formation](docs/formation_preview.png)

## Features

- **Physically correct chain joints** — ball joints (3-DOF) at payload and UAV endpoints prevent false torsion torques; revolute joints with per-chain local axes ensure symmetric bending across all chains
- **Continuous chain geometry** — cylinders span adjacent sphere joints with no visual gaps
- **Collision-ready** — all chain segments have collision geometry (chain ↔ ground, chain ↔ UAV body)
- **Parametric** — change formation radius, payload mass, chain segments, damping in one command
- **PX4 SITL ready** — works out-of-the-box with `make px4_sitl_default`

---

## Directory Structure

```
slung_load/
├── gen_sdf.py        ← Parametric SDF world generator
├── launch.sh         ← Universal PX4 SITL launcher
├── worlds/           ← Generated SDF world files
│   ├── multi_uav_slung_load_r0p80.sdf   (0.8m radius, ~1.4m spacing)
│   └── multi_uav_slung_load_r1p15.sdf   (1.155m radius, 2m spacing)
└── README.md
```

---

## Prerequisites

| Requirement | Notes |
|-------------|-------|
| PX4 Autopilot | Cloned to `~/mypx4/PX4-Autopilot` |
| PX4 SITL build | `make px4_sitl_default` (run once) |
| Gazebo Harmonic | Installed via PX4 setup script |
| Python 3.8+ | For `gen_sdf.py` |
| `xmllint` | For SDF validation (libxml2-utils) |

---

## Quick Start

```bash
# 1. Go to the package directory
cd PX4-Autopilot/Tools/simulation/gz/slung_load

# 2. Launch the standard simulation (0.8m radius, ~1.4m spacing)
./launch.sh standard

# 3. Launch the 2m spacing simulation
./launch.sh large_2m

# 4. In separate terminals, start the MicroXRCE-DDS bridges:
MicroXRCEAgent udp4 -p 8888 &   # UAV 0
MicroXRCEAgent udp4 -p 8889 &   # UAV 1
MicroXRCEAgent udp4 -p 8890 &   # UAV 2
```

Press **Ctrl+C** in the launch terminal to stop all PX4 instances.

---

## Generating Custom Worlds

```bash
cd PX4-Autopilot/Tools/simulation/gz/slung_load

# Show all options
python3 gen_sdf.py --help

# Standard formation (0.8m radius → ~1.39m inter-UAV)
python3 gen_sdf.py --radius 0.8 --out worlds/multi_uav_slung_load_r0p80.sdf

# 2m inter-UAV spacing (radius = 2/sqrt(3) ≈ 1.1547m)
python3 gen_sdf.py --radius 1.1547 --out worlds/multi_uav_slung_load_r1p15.sdf

# Custom GPS origin / site
python3 gen_sdf.py --world-name zju_field --radius 1.1547 \
    --latitude-deg 30.2618 --longitude-deg 120.1172 --elevation 12.5 \
    --out worlds/zju_field.sdf

# Heavy payload, more damping
python3 gen_sdf.py --radius 1.0 --payload-mass 3.0 --damping 0.4 \
    --out worlds/heavy_payload.sdf

# Longer chain (5 segments instead of 3)
python3 gen_sdf.py --radius 1.5 --chain-segments 5 \
    --out worlds/multi_uav_slung_load_r1p50.sdf

# Launch any custom world (copies SDF to gz/worlds/ automatically)
./launch.sh heavy_payload
```

### Generator parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--radius` | 0.8 | Formation radius in meters |
| `--payload-mass` | 1.0 | Payload mass (kg) |
| `--chain-segments` | 3 | Number of cylinder segments per chain |
| `--damping` | 0.001 | Chain revolute joint damping |
| `--cyl-radius` | 0.01 | Chain cylinder radius (m) |
| `--world-name` | auto | SDF world name (default: `multi_uav_slung_load_r<R>`) |
| `--latitude-deg` | 47.397742 | World WGS84 latitude |
| `--longitude-deg` | 8.545594 | World WGS84 longitude |
| `--elevation` | 488.0 | World origin elevation (m) |
| `--heading-deg` | 0 | World ENU heading (deg) |
| `--ground-size` | 200 | Ground plane size (m) |
| `--out` | stdout | Output file path |

## Rebuild From Repo Root

To regenerate a world and optionally launch the full 3-UAV ROS2 simulation from the repo root:

```bash
cd PX4-Autopilot

# Generate only
./rebuild_slung_load_world.sh lab_demo --radius 1.1547 --chain-length 2.0

# Generate and launch
ROS2_WS=/home/ubuntu22/swarm_ws \
ROS2_CMD='ros2 launch fsmpx4 fsmpx4_swarm_3.launch.py' \
./rebuild_slung_load_world.sh lab_demo --radius 1.1547 --chain-length 2.0 --launch
```

---

## Physical Model

### Formation Layout

```
        UAV 1 (120°)
           *
          / \
         /   \
UAV 2 *-------* UAV 0 (0°)
  (240°)   |
          payload
```

3 UAVs placed at 120° intervals on a circle of radius `R`.
Inter-UAV distance = `R × sqrt(3)`.

### Chain Topology (per chain)

```
payload ─ball─> end ─rev(cross)─> s3c ─rev(along)─> s3s
  ─rev(cross)─> s2c ─rev(along)─> s2s
  ─rev(cross)─> s1c ─rev(along)─> s1s
  ─rev(cross)─> hook ─ball─> UAV base_link
```

| Element | Type | DOF | Purpose |
|---------|------|-----|---------|
| `payload_to_c{i}_end` | ball | 3 | No torsion constraint at payload |
| Chain internal joints | revolute (alt. X/Y local axes) | 1 each | Bending in/out of chain plane |
| `c{i}_hook_to_uav{i}` | ball | 3 | No torsion constraint at UAV |

### Why ball joints at endpoints?

A `universal` joint (2-DOF) constrains rotation around the rope axis, which creates **false restoring torques** if the UAV yaws or the chain twists. A `ball` joint (3-DOF) correctly models a rope/cable endpoint with no torsional stiffness.

### Why per-chain local axes for revolute joints?

Using world-frame X/Y axes for all 3 chains means chains at 120° and 240° have **asymmetric bending behaviour** — the same lateral disturbance produces different moments on different chains. Per-chain axes (along-chain and cross-chain directions) ensure all chains respond symmetrically.

### self_collide = false

Intra-model collision (chain ↔ chain, chain ↔ payload) is disabled for numerical stability — high-DOF chains with many small collision shapes can cause constraint solver divergence. Inter-model collision (chain ↔ ground plane, chain ↔ UAV body) **is** active.

To enable intra-model collisions (e.g. to simulate chain wrapping), set `<self_collide>true</self_collide>` and increase ODE iterations in `<physics>`.

---

## Logs

PX4 instance logs are written to:
```
/tmp/px4_<world_name>_0.log
/tmp/px4_<world_name>_1.log
/tmp/px4_<world_name>_2.log
```

---

## Known Limitations

- Only 3-UAV configuration is supported (layout hardcoded to 120° intervals)
- Chain segments use `revolute` (1-DOF) joints alternating on two axes — not a true ball joint chain — but matches the physical behaviour well for typical slung load swing angles
- Self-collision between chains and payload is disabled

---

## License

Apache 2.0 — see the root PX4-Autopilot `LICENSE` file.

---

## Citation / Reference

Based on PX4 Autopilot SITL with Gazebo Harmonic.
Chain modeling follows the multi-body pendulum formulation with symmetric per-chain joint axes.
