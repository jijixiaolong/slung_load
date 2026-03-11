#!/usr/bin/env python3
"""
Parametric SDF generator for Multi-UAV Slung Load simulation.

Usage:
    python3 gen_sdf.py                                    # defaults: radius=0.8m, segments=3
    python3 gen_sdf.py --radius 1.1547                    # 2m inter-UAV spacing
    python3 gen_sdf.py --radius 1.1547 --chain-length 2.0 # longer chain so payload hangs below
    python3 gen_sdf.py --world-name lab_demo --latitude-deg 30.2618 --longitude-deg 120.1172
    python3 gen_sdf.py --radius 1.5 --out worlds/custom.sdf
    python3 gen_sdf.py --help
"""
import math
import argparse
import os


def parse_args():
    p = argparse.ArgumentParser(
        description="Generate Gazebo SDF for multi-UAV slung load simulation"
    )
    p.add_argument("--radius", type=float, default=0.8,
                   help="Formation radius in meters (default: 0.8). "
                        "Inter-UAV distance = radius * sqrt(3). "
                        "For 2m spacing use --radius 1.1547")
    p.add_argument("--n-uavs", type=int, default=3,
                   help="Number of UAVs (default: 3, currently only 3 supported)")
    p.add_argument("--payload-mass", type=float, default=1.0,
                   help="Payload mass in kg (default: 1.0)")
    p.add_argument("--chain-segments", type=int, default=3,
                   help="Number of cylinder segments per chain (default: 3)")
    p.add_argument("--chain-length", type=float, default=None,
                   help="Total chain length in meters (default: auto = same as radius). "
                        "Set longer than radius so payload can hang below UAVs. "
                        "E.g. --chain-length 2.0 with --radius 1.15 gives ~1.6m hang depth")
    p.add_argument("--damping", type=float, default=0.001,
                   help="Chain joint damping (default: 0.001)")
    p.add_argument("--cyl-radius", type=float, default=0.01,
                   help="Chain cylinder radius in meters (default: 0.01)")
    p.add_argument("--world-name", type=str, default=None,
                   help="SDF world name (default: auto-generated from radius)")
    p.add_argument("--latitude-deg", type=float, default=47.397742,
                   help="WGS84 latitude for Gazebo world origin (default: 47.397742)")
    p.add_argument("--longitude-deg", type=float, default=8.545594,
                   help="WGS84 longitude for Gazebo world origin (default: 8.545594)")
    p.add_argument("--elevation", type=float, default=488.0,
                   help="World origin elevation in meters (default: 488.0)")
    p.add_argument("--heading-deg", type=float, default=0.0,
                   help="World ENU heading in degrees (default: 0.0)")
    p.add_argument("--ground-size", type=float, default=200.0,
                   help="Ground plane size in meters (default: 200.0)")
    p.add_argument("--out", type=str, default=None,
                   help="Output file path (default: print to stdout)")
    return p.parse_args()


# ── Math helpers ──────────────────────────────────────────────────────────────

def lerp_positions(start, end, n_points):
    """n_points evenly spaced positions from start to end (inclusive)."""
    positions = []
    for i in range(n_points):
        t = i / (n_points - 1)
        positions.append(tuple(round(start[j] + t * (end[j] - start[j]), 4)
                               for j in range(3)))
    return positions


def direction_rpy_len(dx, dy, dz):
    """RPY + length to align cylinder Z-axis along (dx,dy,dz)."""
    length = math.sqrt(dx*dx + dy*dy + dz*dz)
    horiz = math.sqrt(dx*dx + dy*dy)
    pitch = math.atan2(horiz, dz)
    yaw = math.atan2(dy, dx) if horiz > 1e-9 else 0.0
    return 0.0, pitch, yaw, length


def chain_axes(ux, uy):
    """Local revolute axes for a chain pointing toward (ux, uy)."""
    mag = math.sqrt(ux*ux + uy*uy)
    ax, ay = ux/mag, uy/mag       # along-chain unit vector
    cross = f"{-ay:.4f} {ax:.4f} 0"
    along = f"{ax:.4f} {ay:.4f} 0"
    return along, cross            # (perpendicular, in-plane)


# ── SDF element generators ────────────────────────────────────────────────────

def fmt(v): return f"{v:.4f}"
def p6(x, y, z, r=0, p=0, yw=0): return f"{fmt(x)} {fmt(y)} {fmt(z)} {fmt(r)} {fmt(p)} {fmt(yw)}"


def ball_joint(name, parent, child, indent="      ", child_pose_offset=None):
    """Ball joint (3DOF spherical). Optional child-frame pose offset."""
    pose_line = ""
    if child_pose_offset:
        pose_line = f'{indent}  <pose relative_to="{child}">{child_pose_offset} 0 0 0</pose>\n'
    return (f'{indent}<joint name="{name}" type="ball">\n'
            f'{pose_line}'
            f'{indent}  <parent>{parent}</parent><child>{child}</child>\n'
            f'{indent}</joint>')


def rev_joint(name, parent, child, axis_xyz, damping):
    return (f'      <joint name="{name}" type="revolute">\n'
            f'        <parent>{parent}</parent><child>{child}</child>\n'
            f'        <axis><xyz>{axis_xyz}</xyz>\n'
            f'          <limit><lower>-3.14</lower><upper>3.14</upper></limit>\n'
            f'          <dynamics><damping>{damping}</damping></dynamics></axis>\n'
            f'      </joint>')


def sphere_link(name, xyz, mass, radius, color="0.3 0.3 0.3 1", extra=""):
    ixx = 2/5 * mass * radius**2
    return (f'      <link name="{name}">\n'
            f'        <pose>{p6(*xyz)}</pose>\n'
            f'        <inertial><mass>{mass}</mass>\n'
            f'          <inertia><ixx>{ixx:.2e}</ixx><ixy>0</ixy><ixz>0</ixz>'
            f'<iyy>{ixx:.2e}</iyy><iyz>0</iyz><izz>{ixx:.2e}</izz></inertia></inertial>\n'
            f'        <visual name="v"><geometry><sphere><radius>{radius}</radius></sphere></geometry>\n'
            f'          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>\n'
            f'        <collision name="col"><geometry><sphere><radius>{radius}</radius></sphere></geometry></collision>\n'
            f'{extra}'
            f'      </link>')


def cylinder_link(name, origin_xyz, target_xyz, cyl_r, cyl_mass):
    """Cylinder with origin at the joint anchor point (prev sphere position).

    By placing the link origin exactly at the sphere joint, the revolute joint
    anchor (which defaults to the child link's origin) coincides with the visible
    sphere position, eliminating static anchor-residual misalignment.

    The visual/collision/inertial are offset from the origin toward target_xyz,
    centered at the midpoint of the segment.
    """
    dx = target_xyz[0]-origin_xyz[0]
    dy = target_xyz[1]-origin_xyz[1]
    dz = target_xyz[2]-origin_xyz[2]
    _, pitch, yaw, length = direction_rpy_len(dx, dy, dz)
    # Center offset in link-local frame (link has no rotation, so world-relative offset)
    lp = p6(dx/2, dy/2, dz/2, 0, pitch, yaw)
    ixx = cyl_mass*(3*cyl_r**2 + length**2)/12
    izz = cyl_mass*cyl_r**2/2
    return (f'      <link name="{name}">\n'
            f'        <pose>{p6(*origin_xyz)}</pose>\n'
            f'        <inertial><pose>{lp}</pose><mass>{cyl_mass}</mass>\n'
            f'          <inertia><ixx>{ixx:.2e}</ixx><ixy>0</ixy><ixz>0</ixz>'
            f'<iyy>{ixx:.2e}</iyy><iyz>0</iyz><izz>{izz:.2e}</izz></inertia></inertial>\n'
            f'        <visual name="v"><pose>{lp}</pose>\n'
            f'          <geometry><cylinder><radius>{cyl_r}</radius><length>{fmt(length)}</length></cylinder></geometry>\n'
            f'          <material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material></visual>\n'
            f'        <collision name="col"><pose>{lp}</pose>\n'
            f'          <geometry><cylinder><radius>{cyl_r}</radius><length>{fmt(length)}</length></cylinder></geometry></collision>\n'
            f'      </link>')


def rod_extra(vname, origin_xyz, target_xyz, cyl_r):
    """Rod visual+collision from origin toward target (on a sphere link)."""
    dx=target_xyz[0]-origin_xyz[0]; dy=target_xyz[1]-origin_xyz[1]; dz=target_xyz[2]-origin_xyz[2]
    _, pitch, yaw, length = direction_rpy_len(dx, dy, dz)
    lp = p6(dx/2, dy/2, dz/2, 0, pitch, yaw)
    return (f'        <visual name="{vname}"><pose>{lp}</pose>\n'
            f'          <geometry><cylinder><radius>{cyl_r}</radius><length>{fmt(length)}</length></cylinder></geometry>\n'
            f'          <material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material></visual>\n'
            f'        <collision name="{vname}_col"><pose>{lp}</pose>\n'
            f'          <geometry><cylinder><radius>{cyl_r}</radius><length>{fmt(length)}</length></cylinder></geometry></collision>\n')


# ── Chain generator ───────────────────────────────────────────────────────────

def gen_chain(ci, all_positions, hook_color, along_axis, cross_axis, cfg):
    """Generate all joints+links for one chain."""
    p = f"c{ci}"
    cyl_r = cfg.cyl_radius
    cyl_mass = 0.015 * (cyl_r / 0.01)  # scale mass with radius
    sph_r = max(0.015, cyl_r * 1.5)
    sph_mass = 0.005

    # pre-compute link name list for this chain's n_segments
    # Structure: end, [s{n}c, s{n}s, ...], hook
    link_names = ["end"]
    for seg in range(cfg.chain_segments, 0, -1):
        link_names += [f"s{seg}c", f"s{seg}s"]
    link_names.append("hook")
    cyl_links = {f"s{seg}c" for seg in range(1, cfg.chain_segments+1)}

    positions = all_positions[ci]  # dict name→xyz

    lines = []
    # payload → end (ball: 3DOF)
    lines.append(ball_joint(f"payload_to_{p}_end", "payload_link", f"{p}_end"))
    lines.append(sphere_link(f"{p}_end", positions["end"], sph_mass, sph_r*1.2, "0.6 0.6 0.6 1"))

    # alternate: cross, along, cross, along...
    joints = []
    for i in range(len(link_names)-1):
        parent_s = link_names[i]; child_s = link_names[i+1]
        axis = cross_axis if (i % 2 == 0) else along_axis
        joints.append((parent_s, child_s, axis))

    for parent_s, child_s, axis in joints:
        lines.append(rev_joint(f"{p}_{parent_s}_to_{child_s}",
                               f"{p}_{parent_s}", f"{p}_{child_s}", axis, cfg.damping))
        child_xyz = positions[child_s]
        if child_s in cyl_links:
            idx = link_names.index(child_s)
            prev_xyz = positions[link_names[idx-1]]
            next_xyz = positions[link_names[idx+1]]
            # Origin = prev_xyz (=joint anchor) so revolute anchor aligns with visible sphere
            lines.append(cylinder_link(f"{p}_{child_s}", prev_xyz, next_xyz, cyl_r, cyl_mass))
        elif child_s == link_names[-2]:  # last sphere before hook
            extra = rod_extra("rod_to_hook", child_xyz, positions["hook"], cyl_r)
            lines.append(sphere_link(f"{p}_{child_s}", child_xyz, sph_mass, sph_r, "0.3 0.3 0.3 1", extra))
        elif child_s == "hook":
            lines.append(sphere_link(f"{p}_{child_s}", child_xyz, sph_mass, sph_r, hook_color))
        else:
            lines.append(sphere_link(f"{p}_{child_s}", child_xyz, sph_mass, sph_r))

    return "\n".join(lines)


# ── Main generator ────────────────────────────────────────────────────────────

def generate(cfg):
    R = cfg.radius
    HOOK_COLORS = ["1 0 0 1", "0 1 0 1", "0 0 1 1"]
    END_Z   = 0.2    # just above payload surface (r=0.1, center z=0.1)

    # Chain length: if user specifies --chain-length, use it; otherwise default to ~radius
    if cfg.chain_length is not None:
        chain_len = cfg.chain_length
        if chain_len < R:
            print(f"WARNING: chain-length ({chain_len}) < radius ({R}), "
                  f"payload cannot hang below UAVs. Consider chain-length >= {R*1.4:.2f}")
        # Compute HOOK_Z so that the chain geometry is correct:
        # chain_len² = R² + (HOOK_Z - END_Z)²  →  HOOK_Z = END_Z + sqrt(chain_len² - R²)
        hang = math.sqrt(max(0, chain_len**2 - R**2))
        HOOK_Z = round(END_Z + hang, 4)
    else:
        HOOK_Z = 0.19   # legacy: 5cm below base_link, chain_len ≈ radius
        chain_len = round(math.sqrt(R**2 + (HOOK_Z - END_Z)**2), 4)

    chain_len = round(chain_len, 4)

    UAVS = [
        ("x500_0",  round(R, 4),          0.0,                           "0°"),
        ("x500_1",  round(-R/2, 4),        round(R*math.sqrt(3)/2, 4),  "120°"),
        ("x500_2",  round(-R/2, 4),        round(-R*math.sqrt(3)/2, 4), "240°"),
    ]

    inter_uav = round(R * math.sqrt(3), 4)
    hang_depth = round(HOOK_Z - END_Z, 4)
    world_name = cfg.world_name or f"multi_uav_slung_load_r{R:.2f}".replace(".", "p")

    # Build link_names list for position interpolation
    link_names = ["end"]
    for seg in range(cfg.chain_segments, 0, -1):
        link_names += [f"s{seg}c", f"s{seg}s"]
    link_names.append("hook")
    n_pts = len(link_names)

    # Compute positions for all chains
    all_positions = []
    for _, ux, uy, _ in UAVS:
        end_xyz  = (0.0, 0.0, END_Z)
        hook_xyz = (ux, uy, HOOK_Z)
        pts = lerp_positions(end_xyz, hook_xyz, n_pts)
        all_positions.append(dict(zip(link_names, pts)))

    payload_r = max(0.1, R * 0.1)  # payload radius scales with formation

    lines = []
    lines.append(f"""\
<?xml version="1.0" ?>
<!--
  ============================================================================
  Multi-UAV Slung Load Simulation World
  Generated by: gen_sdf.py  (https://github.com/YOUR_USER/YOUR_REPO)
  ============================================================================
  Formation:
    UAVs       : {cfg.n_uavs}× x500 quadrotor
    Radius     : {R:.4f} m
    Inter-UAV  : {inter_uav:.4f} m
    Positions  : 0° / 120° / 240°, spawned on ground (z=0)

  Chain per UAV:
    Length     : {chain_len:.4f} m  (end sphere → hook)
    Segments   : {cfg.chain_segments} cylinder + {cfg.chain_segments+1} sphere joints
    Topology   : payload ─ball─> end ─rev×{2*cfg.chain_segments+1}─> hook ─ball─> base_link
    Damping    : {cfg.damping}  (chain revolute joints)
    Axes       : per-chain local axes (symmetric bending for all chains)
    Geometry   : cylinders span prev→next joint (continuous, no gaps)

  Joints:
    Endpoints  : ball (3DOF) — no torsion injection into UAV/payload
    Chain body : revolute, alternating cross/along-chain axes
    Cross-model: ball, hook coincident with physical attachment point
                 at z = base_link - 0.05m

  Payload:
    Mass       : {cfg.payload_mass} kg
    Shape      : sphere, radius {payload_r:.3f} m

  self_collide : false (intentional — avoids constraint divergence on dense chains)
                 Inter-model collision (chain↔ground, chain↔UAV) is active.

  Regenerate:
    python3 gen_sdf.py -radius {R:.4f} -out worlds/{world_name}.sdf
  ============================================================================
-->
<sdf version="1.9">
  <world name="{world_name}">

    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{cfg.latitude_deg}</latitude_deg>
      <longitude_deg>{cfg.longitude_deg}</longitude_deg>
      <elevation>{cfg.elevation}</elevation>
      <heading_deg>{cfg.heading_deg}</heading_deg>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{cfg.ground_size} {cfg.ground_size}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{cfg.ground_size} {cfg.ground_size}</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular></material>
        </visual>
      </link>
    </model>

    <!-- ========== {cfg.n_uavs}× x500 UAVs (radius={R:.4f}m, spacing={inter_uav:.4f}m) ========== -->""")

    for name, x, y, angle in UAVS:
        lines.append(f'    <!-- {angle}: ({x}, {y}, 0) -->')
        lines.append(f'    <include><uri>model://x500</uri><name>{name}</name>'
                     f'<pose>{x} {y} 0 0 0 0</pose></include>')

    lines.append(f"""
    <!-- ========== Slung Load System ========== -->
    <!-- self_collide=false: intra-model collision disabled for stability.   -->
    <!-- Inter-model collision (chain↔ground, chain↔UAV body) is active.    -->
    <model name="slung_load">
      <pose>0 0 0 0 0 0</pose>
      <self_collide>false</self_collide>

      <!-- Payload: {cfg.payload_mass}kg sphere, radius={payload_r:.3f}m -->
      <link name="payload_link">
        <pose>0 0 0.1 0 0 0</pose>
        <inertial><mass>{cfg.payload_mass}</mass>
          <inertia>
            <ixx>{2/5*cfg.payload_mass*payload_r**2:.4f}</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>{2/5*cfg.payload_mass*payload_r**2:.4f}</iyy><iyz>0</iyz>
            <izz>{2/5*cfg.payload_mass*payload_r**2:.4f}</izz>
          </inertia>
        </inertial>
        <visual name="body_visual">
          <geometry><sphere><radius>{payload_r:.3f}</radius></sphere></geometry>
          <material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material>
        </visual>
        <collision name="body_collision">
          <geometry><sphere><radius>{payload_r:.3f}</radius></sphere></geometry>
        </collision>
      </link>
""")

    for ci, (uav_name, ux, uy, angle) in enumerate(UAVS):
        along, cross = chain_axes(ux, uy)
        lines.append(f"      <!-- Chain {ci} → {uav_name} ({angle}), "
                     f"local axes: along={along}, cross={cross} -->")
        lines.append(gen_chain(ci, all_positions, HOOK_COLORS[ci], along, cross, cfg))
        lines.append("")

    lines.append("    </model>\n")
    lines.append("    <!-- Cross-model ball joints: hook → base_link -->")
    lines.append("    <!-- Visual hook and physical hanger point coincide 5cm below base_link -->")
    for ci, (uav_name, *_) in enumerate(UAVS):
        lines.append(ball_joint(f"c{ci}_hook_to_uav{ci}",
                                f"slung_load::c{ci}_hook",
                                f"{uav_name}::base_link",
                                indent="    "))
        lines.append("")

    lines.append("  </world>\n</sdf>")
    return "\n".join(lines)


if __name__ == "__main__":
    cfg = parse_args()
    sdf_content = generate(cfg)

    if cfg.out:
        os.makedirs(os.path.dirname(os.path.abspath(cfg.out)), exist_ok=True)
        with open(cfg.out, "w") as f:
            f.write(sdf_content)
        print(f"Written to {cfg.out}")
    else:
        print(sdf_content)
