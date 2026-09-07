#!/usr/bin/env python3
"""Generate the Gazebo maze world from the ASCII maze in explore_sim.

Same 13x13 maze, 2x scale (30 cm corridors) so the sim robot (18 cm with
wheels) fits with ~6 cm clearance per side and can turn in place. Writes
pinky_maze.sdf next to this script.
"""
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))  # tools/gz -> repo root: tools.explore_sim
from tools.explore_sim import MAZE

CELL = 0.30   # m per maze cell in gz (2x the planner's 5 cm grid)
H = 0.15      # wall height


def sub(parent, tag, **attrs):
    e = ET.SubElement(parent, tag)
    for k, v in attrs.items():
        e.set(k, v)
    return e


def leaf(parent, tag, text):
    e = ET.SubElement(parent, tag)
    e.text = str(text)
    return e


def geom_box(parent, name, tag_name, x, y, z, sx, sy, sz, vis=False):
    c = sub(parent, tag_name, name=name)
    leaf(c, 'pose', f'{x} {y} {z} 0 0 0')
    geo = sub(c, 'geometry')
    b = sub(geo, 'box')
    leaf(b, 'size', f'{sx} {sy} {sz}')
    return c


def walls():
    """Merge each horizontal wall run into one box (x, y, width)."""
    out = []
    for row, line in enumerate(MAZE):
        r = len(MAZE) - 1 - row  # art is top-down, grid is bottom-up
        c = 0
        while c < len(line):
            if line[c] != '#':
                c += 1
                continue
            c0 = c
            while c < len(line) and line[c] == '#':
                c += 1
            x = ((c0 + c - 1) / 2.0 + 0.5) * CELL
            y = (r + 0.5) * CELL
            w = (c - c0) * CELL
            out.append((x, y, w))
    return out


def wheel(parent, name, y):
    lnk = sub(parent, 'link', name=name)
    leaf(lnk, 'pose', f'-0.04 {y} 0.028 -1.5707 0 0')
    ine = sub(lnk, 'inertial')
    # Heavier wheels integrate stably at the 5 ms step (the 0.03 kg /
    # 1e-5 inertia pair was the crawl's root cause on this box).
    leaf(ine, 'mass', 0.1)
    i = sub(ine, 'inertia')
    for a in ('ixx', 'iyy', 'izz'):
        leaf(i, a, 1e-4)
    for a in ('ixy', 'ixz', 'iyz'):
        leaf(i, a, 0.0)
    # Rolling cylinder collision — a box collision slides on the ground, so
    # the joint-kinematic odom diverged from the physical pose (robot spun
    # in place at spawn while odom claimed translation).
    col = sub(lnk, 'collision', name='col')
    colg = sub(col, 'geometry')
    cyl = sub(colg, 'cylinder')
    leaf(cyl, 'radius', 0.028)
    leaf(cyl, 'length', 0.02)
    # One <geometry> per visual (SDF allows exactly one): the wheel renders
    # as the same cylinder as its collision — geom_box() had attached a
    # <geometry><box> and a second <geometry><cylinder> was appended to the
    # same <visual>; libsdformat warns and drops one of the two.
    v = sub(lnk, 'visual', name='vis')
    geo = sub(v, 'geometry')
    cyl = sub(geo, 'cylinder')
    leaf(cyl, 'radius', 0.028)
    leaf(cyl, 'length', 0.02)
    col = lnk.find("collision[@name='col']")
    surf = sub(col, 'surface')
    # Grip + stiff contact, the pinky-pro sim values: mu 1.5 on default-soft
    # contacts lets the wheels sink ~6 mm into the ground box and the robot
    # plows at ~1 cm/s under full command (measured on this rig); kp 1e7
    # keeps penetration ~microns so wheels roll.
    cont = sub(surf, 'contact')
    ode_c = sub(cont, 'ode')
    leaf(ode_c, 'kp', 1e7)
    leaf(ode_c, 'kd', 1.0)
    leaf(ode_c, 'max_vel', 0.1)
    fric = sub(surf, 'friction')
    ode = sub(fric, 'ode')
    # mu1 2.0 (longitudinal), mu2 1.0 (lateral slip): a diff robot
    # SKID-steers — turning in place requires lateral wheel slip, and
    # mu2 200 pins the robot (the pinky rig's own mu1=mu2=200 has the
    # same freeze). mu1 is 2, not the pinky 200, because with mu1 200 a
    # wall-pressed robot had traction to CLIMB the wall into a nose-up
    # turtle (-90 deg pitch, measured here); mu 2 still grips the ground
    # 2x harder than its 0.9 but stalls against wall pushes instead of
    # climbing. Odometry is ground truth (OdometryPublisher), so low
    # wheel grip costs nothing there.
    leaf(ode, 'mu', 2.0)
    leaf(ode, 'mu2', 1.0)
    return lnk


def build():
    sdf = ET.Element('sdf', version='1.8')
    w = sub(sdf, 'world', name='pinky_maze')

    # ODE, not the DART default: ODE honors kp/mu on the robot contacts —
    # on DART the wheels sank ~6 mm into the ground and a pressed nose
    # embedded 5-7 cm into walls, pinning the robot (pinky's working maze
    # world runs type="ode" too).
    ph = sub(w, 'physics', name='1ms', type='ode')
    # 5 ms step: at 1 ms the 1e-5 kg m2 wheel inertia integrated badly on
    # this box (robot capped at 1-2 cm/s whatever the friction recipe).
    leaf(ph, 'max_step_size', 0.005)
    leaf(ph, 'real_time_factor', 1.0)

    # NOTE: no PosePublisher here — it aborts the gz-sim10 server when it
    # fails to init, and the driver node owns the ROS TF anyway.
    for name in ('gz-sim-physics-system', 'gz-sim-user-commands-system',
                 'gz-sim-scene-broadcaster-system'):
        sub(w, 'plugin', filename=name, name={
            'gz-sim-physics-system': 'gz::sim::systems::Physics',
            'gz-sim-user-commands-system': 'gz::sim::systems::UserCommands',
            'gz-sim-scene-broadcaster-system':
                'gz::sim::systems::SceneBroadcaster',
        }[name])
    sensors = sub(w, 'plugin', filename='gz-sim-sensors-system',
                  name='gz::sim::systems::Sensors')
    leaf(sensors, 'render_engine', 'ogre2')

    arena = len(MAZE[0]) * CELL
    g = sub(w, 'model', name='ground')
    leaf(g, 'static', 'true')
    gl = sub(g, 'link', name='link')
    gc = geom_box(gl, 'col', 'collision', arena / 2, arena / 2, -0.05,
                  arena + 1, arena + 1, 0.1)
    gs = sub(gc, 'surface')
    gf = sub(gs, 'friction')
    go = sub(gf, 'ode')
    # The pinky maze world sets explicit ground friction; leaving it off
    # leaves the ODE default to interact with the wheel mu unpredictably.
    leaf(go, 'mu', 0.9)
    leaf(go, 'mu2', 0.9)
    geom_box(gl, 'vis', 'visual', arena / 2, arena / 2, -0.05,
             arena + 1, arena + 1, 0.1, vis=True)

    m = sub(w, 'model', name='maze')
    leaf(m, 'static', 'true')
    ml = sub(m, 'link', name='link')
    for i, (x, y, wd) in enumerate(walls()):
        geom_box(ml, f'w{i}', 'collision', x, y, H / 2, wd, CELL, H)
        geom_box(ml, f'v{i}', 'visual', x, y, H / 2, wd, CELL, H, vis=True)

    r = sub(w, 'model', name='pinky')
    leaf(r, 'pose', f'{1.5 * CELL} {1.5 * CELL} 0.07 0 0 0')
    base = sub(r, 'link', name='base')
    ine = sub(base, 'inertial')
    # CG between the axle (x=-0.04) and the rear skid (x=-0.0575): nose-up
    # rotation is blocked by the skid contact, nose-down by the wheels, so
    # the robot is statically stable in pitch for the first time. (The old
    # CG-at-axle layout was pitch-neutral — any wall press rotated it; CG
    # 2.5 cm behind the axle made it a wheelbarrow and tipped BACKWARD.)
    leaf(ine, 'pose', '-0.055 0 0.04 0 0 0')
    leaf(ine, 'mass', 0.6)
    ii = sub(ine, 'inertia')
    for a in ('ixx', 'iyy', 'izz'):
        leaf(ii, a, 0.001)
    for a in ('ixy', 'ixz', 'iyz'):
        leaf(ii, a, 0.0)
    # Chassis sits LOW (bottom 1 cm over ground; wall contact ~axle height)
    # plus a REAR SKID: the -90 deg nose-up turtle pivots about the wheel
    # contact, and nose-up rotation drives the skid INTO the ground, which
    # blocks it geometrically. CG moves (CG at the axle, then 2.5 cm behind
    # it — which made it WORSE: CG behind the contact is a wheelbarrow, the
    # nose-up rotation just gets easier) cannot fix this; unlimited-effort
    # joint velocity control out-torques any CG offset. The old caster box
    # blew up ODE's collision space (AABB assertion crash) — this skid is
    # 1.5 cm tall and sits behind the axle; boots verified on this rig.
    # Also shrunk from 16x12 cm: that box wedged diagonally in the 30 cm
    # corridors. The rig tests the planning stack, not chassis fidelity.
    geom_box(base, 'col', 'collision', 0, 0, 0.04, 0.14, 0.10, 0.06)
    vis = geom_box(base, 'vis', 'visual', 0, 0, 0.04, 0.14, 0.10, 0.06, vis=True)
    geom_box(base, 'skid_col', 'collision',
             -0.065, 0, 0.0075, 0.015, 0.06, 0.015)
    geom_box(base, 'skid_vis', 'visual',
             -0.065, 0, 0.0075, 0.015, 0.06, 0.015, vis=True)
    mat = sub(vis, 'material')
    amb = sub(mat, 'ambient')
    amb.text = '0.2 0.4 1 1'

    sense = sub(base, 'sensor', name='lidar', type='gpu_lidar')
    # Above the chassis box (top 0.08) but below the wall top (0.15):
    # inside the box, every beam starts inside a collision and the SLAM
    # map degenerates to a ~5 cm occupied blob around the robot.
    leaf(sense, 'pose', '0 0 0.10 0 0 0')
    leaf(sense, 'topic', 'lidar/scan')
    leaf(sense, 'update_rate', 10)
    leaf(sense, 'always_on', 1)
    leaf(sense, 'visualize', 0)
    lid = sub(sense, 'lidar')
    sc = sub(lid, 'scan')
    hor = sub(sc, 'horizontal')
    leaf(hor, 'samples', 180)
    leaf(hor, 'resolution', 1)
    leaf(hor, 'min_angle', -3.14159)
    leaf(hor, 'max_angle', 3.14159)
    rng = sub(lid, 'range')
    leaf(rng, 'min', 0.02)
    # 3 m, not the real C1's 10 m and not the old 0.45 m: 0.45 m sealed
    # every scan ring inside the 0.3 m corridors, so frontiers survived
    # only as 1-3 cell ray tips and the brain collapsed into micro-goals.
    leaf(rng, 'max', 3.0)
    leaf(rng, 'resolution', 0.01)

    wheel(r, 'wheel_left', 0.082)
    wheel(r, 'wheel_right', -0.082)
    # Wheel joints: revolute about +y, NO limit element (ODE treats a
    # missing limit as unlimited; a ±1e16 limit strained the LCP at
    # spawn — AABB assertion crash in collision_space killed the physics
    # thread and the lidar with it on ~half the boots).
    for jname, child in (('j_left', 'wheel_left'),
                         ('j_right', 'wheel_right')):
        j = sub(r, 'joint', name=jname, type='revolute')
        leaf(j, 'parent', 'base')
        leaf(j, 'child', child)
        ax = sub(j, 'axis')
        leaf(ax, 'xyz', '0 1 0')
        # NO limit element at all: SDF has no explicit unlimited value
        # (effort -1 reads literally and clamps the wheels dead).
    # DiffDrive + ground-truth odometry. NOTE on the crawl investigation:
    # wheel-physics capped the robot at 1-2 cm/s regardless of friction
    # recipe (even the pinky rig crawls on this machine); the fix that
    # finally worked is the physics step + wheel inertia, see wheel().
    dd = sub(r, 'plugin', filename='gz-sim-diff-drive-system',
             name='gz::sim::systems::DiffDrive')
    for t in ('left_joint', 'right_joint'):
        leaf(dd, t, 'j_left' if t == 'left_joint' else 'j_right')
    leaf(dd, 'wheel_separation', 0.164)
    leaf(dd, 'wheel_radius', 0.028)
    leaf(dd, 'odom_publish_frequency', 20)
    leaf(dd, 'odom_topic', 'odometry_diff')
    leaf(dd, 'frame_id', 'odom')
    leaf(dd, 'child_frame_id', 'base_link')
    # NO min/max_velocity here: those are JOINT rad/s clamps — the pinky
    # rig's 1.0 clamps its wheels to 1.0 rad/s = 2.8 cm/s, its own crawl.
    # max_linear_acceleration alone keeps the start gentle.
    # Ground-truth 2D odometry: /odom == gz truth, no wheel slip.
    op = sub(r, 'plugin', filename='gz-sim-odometry-publisher-system',
             name='gz::sim::systems::OdometryPublisher')
    leaf(op, 'dimensions', 2)
    leaf(op, 'odom_topic', 'odometry_gt')
    leaf(op, 'odom_publish_frequency', 20)
    return ET.tostring(sdf, encoding='unicode')


def main():
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'pinky_maze.sdf')
    xml = build()
    ET.fromstring(xml)  # validate before writing
    with open(out, 'w') as f:
        f.write('<?xml version="1.0"?>\n' + xml + '\n')
    print('wrote', out, len(xml), 'bytes')


if __name__ == '__main__':
    main()
