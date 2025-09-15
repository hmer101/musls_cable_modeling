#!/usr/bin/env bash
set -euo pipefail

# --------- Placeholders (your Python script replaces these later) ---------
JOINT1_AXIS="0 1 0"
JOINT1_DAMPING="0.005"
JOINT1_FRICTION="0"
JOINT1_SPRING_REFERENCE="0"
JOINT1_SPRING_STIFFNESS="0"

JOINT2_AXIS="0 0 1"
JOINT2_DAMPING="0.005"
JOINT2_FRICTION="0"
JOINT2_SPRING_REFERENCE="0"
JOINT2_SPRING_STIFFNESS="0"

# --------- Build URDF -> SDF ----------
xacro model.xacro > model.urdf
gz sdf -p model.urdf > model.sdf

# --------- revolute -> universal (both quote styles) ----------
sed -i 's/type='\''revolute'\''/type='\''universal'\''/g; s/type="revolute"/type="universal"/g' model.sdf

# --------- AXIS 1: replace the ENTIRE <axis>...</axis> with a templated block ----------
# This hits every original axis (before axis2 exists), so each joint gets a fresh Axis-1.
perl -0777 -i -pe '
  my $blk = q{
      <axis>
        <xyz>'"$JOINT1_AXIS"'</xyz>
        <limit>
          <lower>-inf</lower>
          <upper>inf</upper>
        </limit>
        <dynamics>
          <damping>'"$JOINT1_DAMPING"'</damping>
          <friction>'"$JOINT1_FRICTION"'</friction>
          <spring_reference>'"$JOINT1_SPRING_REFERENCE"'</spring_reference>
          <spring_stiffness>'"$JOINT1_SPRING_STIFFNESS"'</spring_stiffness>
        </dynamics>
      </axis>
  };
  s|<axis>(?:(?!</axis>).)*?</axis>|$blk|gs' model.sdf

# --------- AXIS 2: insert a new axis2 block before </joint> (exact same style as you had) ----------
perl -0777 -i -pe '
  s|
    \s*</axis>\s*\n\s*</joint>
  |
    </axis>
      <axis2>
        <xyz>'"$JOINT2_AXIS"'</xyz>
        <limit>
          <lower>-inf</lower>
          <upper>inf</upper>
        </limit>
        <dynamics>
          <damping>'"$JOINT2_DAMPING"'</damping>
          <friction>'"$JOINT2_FRICTION"'</friction>
          <spring_reference>'"$JOINT2_SPRING_REFERENCE"'</spring_reference>
          <spring_stiffness>'"$JOINT2_SPRING_STIFFNESS"'</spring_stiffness>
        </dynamics>
      </axis2>
    </joint>
  |gs' model.sdf

echo "Done → model.sdf (Axis1 & Axis2 templated with #JOINT* placeholders)"
