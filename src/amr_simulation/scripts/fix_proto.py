#!/usr/bin/env python3
"""Post-process amr_robot.proto after every urdf2webots regeneration.

WHY THIS EXISTS: urdf2webots cannot see <webots>/<gazebo> xacro extensions,
drops <collision><origin> transforms for primitive box/cylinder
boundingObjects, and (in our specific setup) needs help placing the Lidar
so it doesn't scan into its own body. This script is the second half of
the regeneration pipeline - see src/amr_simulation/PIPELINE.md (or the
project_context/ docs) for the full command sequence that runs before this.

Structural note: this targets the base_footprint-bypass URDF (base_link is
the true root under use_webots:=true - see base.urdf.xacro). Because of
that, urdf2webots puts boundingObject/physics directly on the Robot node
itself (no wrapping "base_link" Solid exists - Robot's own `name` field is
bound via `name IS name` to the exposed PROTO field instead), and the
sensor sub-solids (laser_frame/imu_link/cam_1_link) sit one indentation
level shallower than they would if base_footprint were present.

Fixes applied, in order:
  1. Mesh URLs come out as absolute paths tied to this machine's install/
     directory -> rewritten relative to this .proto file's own location,
     pointing at the mesh copies kept alongside it in protos/meshes/.
  2. Primitive (box/cylinder) boundingObjects silently drop the source
     <collision><origin> transform entirely (translation AND rotation) -
     they're placed at the raw link/joint origin regardless of what the
     URDF specified. Fixed by wrapping each one in a Pose matching the
     real <collision><origin> from the source xacro. (Pose, not Transform:
     urdf2webots itself uses Pose everywhere else in this file for the
     same "reposition a shape" purpose, e.g. every wheel visual - that
     makes it the proven-compatible node type here, not an assumption.)
  3. Injects native Webots sensor nodes (Lidar, IMU triplet, Camera+
     RangeFinder) that urdf2webots does not create from <gazebo>/<webots>
     xacro extensions - it does not parse those at all.
       - Lidar `type` is set to "rotating" (not the default "fixed") for a
         genuine 360-degree fieldOfView, matching the real RPLidar S2
         hardware and Webots' own official Lidar sample (lidar.wbt), which
         also uses type "rotating".
       - Lidar gets its own small `translation 0 0 0.03` INSIDE the Lidar
         node (independent of the visual mesh, which is a sibling in the
         same Solid): the lidar.stl visualization mesh's bounding box
         straddles the origin in all 3 axes, i.e. it fully encloses its
         own local origin. Without this offset, every one of the sensor's
         360 rays hits the inside of its own visualization dome within
         millimeters and returns .inf in every direction, regardless of
         how high the parent Solid is mounted (verified by raising the
         mount height alone first, which did NOT fix it - the dome moves
         up with the sensor).
  4. Renames urdf2webots's auto-generated wheel PositionSensors from
     "<joint>_sensor" to the "<side>_wheel_sensor" naming used in
     sim_control.urdf.xacro's device/plugin config.
  5. Strips the invalid Physics node urdf2webots puts on laser_frame,
     imu_link, and cam_1_link. Webots only allows a Physics node on the
     Robot's direct base Solid/fields or on a Joint's endPoint Solid -
     these three are plain fixed (non-jointed) children, so an independent
     Physics node on them is invalid and breaks/disables proper physics
     for the body (this was one root cause of "gravity does nothing" -
     see project_context/decisions-and-gotchas.md).

Also note: the chassis mount height for the Lidar's PARENT Solid (set in
amr.urdf.xacro's <xacro:lidar xyz_offset=.../> call, currently 0.22) is a
SEPARATE fix from #3 above - that one clears the chassis roof (chassis.stl
measured top is z=0.1905); this script's own +0.03 offset clears the
Lidar's OWN visualization mesh. Both were needed; neither alone was
sufficient.
"""
import re
import sys

PROTO_PATH = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/ironvolk/ros/src/amr_simulation/webots/protos/amr_robot.proto"

with open(PROTO_PATH) as f:
    content = f.read()

# 1. Portable mesh paths
content = re.sub(
    r'/home/ironvolk/ros/install/amr_description/share/amr_description/meshes/',
    'meshes/',
    content,
)

# 2a. base_link (Robot's own) collision box: wrap in Pose matching
#     base.urdf.xacro's <collision><origin xyz="0 0 0.1075"/>
content = content.replace(
    '    boundingObject Box {\n'
    '       size 0.280000 0.220000 0.150000\n'
    '    }\n',
    '    boundingObject Pose {\n'
    '      translation 0.000000 0.000000 0.107500\n'
    '      children [\n'
    '        Box {\n'
    '          size 0.280000 0.220000 0.150000\n'
    '        }\n'
    '      ]\n'
    '    }\n'
)

# 2b. wheel collision cylinders: wrap in Pose matching
#     wheel.urdf.xacro's <collision><origin rpy="1.5708 0 0"/>
content = re.sub(
    r'boundingObject Cylinder \{\n(\s*)radius 0\.0325\n\s*height 0\.0304\n(\s*)\}',
    r'boundingObject Pose {\n\1rotation 1.000000 0.000000 0.000000 1.570800\n\1children [\n\1  Cylinder {\n\1    radius 0.0325\n\1    height 0.0304\n\1  }\n\1]\n\2}',
    content,
)

# 3. Native sensor injection (indentation matches base_link-as-root: one
#    level shallower than it would be with a wrapping base_link Solid)
content = content.replace(
    '        ]\n        name "laser_frame"',
    '          Lidar {\n'
    '            translation 0 0 0.03\n'
    '            name "Lidar"\n'
    '            type "rotating"\n'
    '            horizontalResolution 360\n'
    '            numberOfLayers 1\n'
    '            fieldOfView 6.28318\n'
    '            minRange 0.05\n'
    '            maxRange 12.0\n'
    '          }\n'
    '        ]\n        name "laser_frame"',
)

content = re.sub(
    r'(\s*)\}\n(\s*)\]\n(\s*)name "imu_link"',
    r'\1}\n'
    r'\2  InertialUnit {\n\2    name "IMU"\n\2  }\n'
    r'\2  Accelerometer {\n\2    name "accelerometer"\n\2  }\n'
    r'\2  Gyro {\n\2    name "gyroscope"\n\2  }\n'
    r'\2]\n\3name "imu_link"',
    content,
)

content = content.replace(
    '        ]\n        name "cam_1_link"',
    '          Camera {\n'
    '            name "camera"\n'
    '            width 424\n'
    '            height 240\n'
    '            fieldOfView 1.5184\n'
    '          }\n'
    '          RangeFinder {\n'
    '            name "range_finder"\n'
    '            width 424\n'
    '            height 240\n'
    '            fieldOfView 1.5184\n'
    '            minRange 0.05\n'
    '            maxRange 1.5\n'
    '          }\n'
    '        ]\n        name "cam_1_link"',
)

# 4. Wheel PositionSensor renames
for side in ['front_left', 'front_right', 'rear_left', 'rear_right']:
    content = content.replace(
        f'name "{side}_wheel_joint_sensor"',
        f'name "{side}_wheel_sensor"',
    )

# 6. Inject contactMaterial into wheel Solid blocks.
#    Mecanum rollers are mirrored diagonals: FL+RR = +45 deg, FR+RL = -45 deg.
#    The contactMaterial field is inserted immediately before the 'physics Physics'
#    line inside each wheel Solid. urdf2webots never emits this field, so it must
#    be post-injected here after step 2b has already wrapped the boundingObject.
for side, material in [
    ('front_left',  'wheel_pos_45'),
    ('rear_right',  'wheel_pos_45'),
    ('front_right', 'wheel_neg_45'),
    ('rear_left',   'wheel_neg_45'),
]:
    pattern = re.compile(
        r'(          name "' + re.escape(side) + r'_wheel_link"\n'
        r'(          boundingObject Pose \{.*?\}\n))'
        r'(          physics)',
        re.DOTALL
    )
    replacement = (
        r'\1'
        r'          contactMaterial "' + material + r'"\n'
        r'\3'
    )
    new_content, n = pattern.subn(replacement, content)
    assert n == 1, \
        f"contactMaterial injection: expected 1 match for '{side}_wheel_link', got {n}"
    content = new_content


# 5. Strip invalid Physics from fixed (non-jointed) sensor sub-solids
for frame_name in ['laser_frame', 'imu_link', 'cam_1_link']:
    pattern = (
        r'(name "' + frame_name + r'"\n)'
        r'(\s*)physics Physics \{[^}]*\}\n'
    )
    new_content, n = re.subn(pattern, r'\1', content)
    assert n == 1, f"expected to strip exactly one Physics block for {frame_name}, stripped {n}"
    content = new_content

with open(PROTO_PATH, 'w') as f:
    f.write(content)

# Sanity checks
assert content.count('{') == content.count('}'), "brace mismatch after fixup!"
assert content.count('[') == content.count(']'), "bracket mismatch after fixup!"
for name in ['Lidar {', 'InertialUnit {', 'Accelerometer {', 'Gyro {',
             'Camera {', 'RangeFinder {']:
    assert content.count(name) == 1, f"expected exactly one {name}, found {content.count(name)}"
assert 'type "rotating"' in content, "Lidar type=rotating fix did not land!"
assert content.count('boundingObject Pose {') == 5, \
    f"expected 5 rotated/translated boundingObjects (1 base + 4 wheels), found {content.count('boundingObject Pose {')}"
assert 'install/amr_description' not in content, "absolute mesh path leaked through!"
assert content.count('physics Physics {') == 5, \
    f"expected exactly 5 Physics nodes (Robot base + 4 wheels), found {content.count('physics Physics {')}"
pos_count = content.count('contactMaterial "wheel_pos_45"')
neg_count = content.count('contactMaterial "wheel_neg_45"')
assert pos_count == 2, \
    f"expected 2 wheel_pos_45 contactMaterial fields, found {pos_count}"
assert neg_count == 2, \
    f"expected 2 wheel_neg_45 contactMaterial fields, found {neg_count}"

print("fix_proto.py: all checks passed")
