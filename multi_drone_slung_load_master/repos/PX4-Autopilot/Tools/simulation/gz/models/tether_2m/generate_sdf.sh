#!/usr/bin/env bash
set -euo pipefail

# --------- Placeholders (your Python script replaces these later) ---------
JOINT1_AXIS="0 1 0"
JOINT1_DAMPING="0.002"
JOINT1_FRICTION="0"
JOINT1_SPRING_REFERENCE="-0.10833078115"
JOINT1_SPRING_STIFFNESS="0"

JOINT2_AXIS="0 0 1"
JOINT2_DAMPING="0.002"
JOINT2_FRICTION="0"
JOINT2_SPRING_REFERENCE="0"
JOINT2_SPRING_STIFFNESS="0"

# Alternating joint selection controls 
K="2"              # e.g. 2 → every 2nd joint
OFFSET="1"    # e.g. 2 → start from joint 2 (so 2,4,6,…)

JOINT1_K_DAMPING="0.002"
JOINT1_K_FRICTION="0.0"
JOINT1_K_SPRING_REFERENCE="-0.10833078115" #"-0.10833078115_ALT"
JOINT1_K_SPRING_STIFFNESS="0"

JOINT2_K_DAMPING="0.002"
JOINT2_K_FRICTION="0.0"
JOINT2_K_SPRING_REFERENCE="0" #"0_ALT"
JOINT2_K_SPRING_STIFFNESS="0"


# --------- Build URDF -> SDF ----------
xacro model.xacro > model.urdf
gz sdf -p model.urdf > model.sdf

# --------- revolute -> universal (both quote styles) ----------
sed -i 's/type='\''revolute'\''/type='\''universal'\''/g; s/type="revolute"/type="universal"/g' model.sdf

# --------- Force SDF version to 1.10 ----------
sed -i "s|<sdf version='1.11'>|<sdf version='1.10'>|" model.sdf

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

# --------- OPTION: select every K-th joint (OFFSET=2 → 2,4,6,...) ----------
perl -0777 -i -pe '
  # --- parameters ---
  my $k_raw      = "'"$K"'";
  my $offset_raw = "'"$OFFSET"'";
  my $k      = ($k_raw      =~ /^\d+$/) ? int($k_raw)      : 2;
  my $offset = ($offset_raw =~ /^\d+$/) ? int($offset_raw) : 2;
  $k      = 1 if $k < 1;
  $offset = 1 if $offset < 1;
  my $sel0 = $offset - 1;

  # --- helpers: change a field *inside one axis block string* ---
  sub axis_set_field {
    my ($axis_str, $field, $val) = @_;
    # replace ONLY the contents between <field>...</field> within this axis block
    $axis_str =~ s{(<\Q$field\E>)[^<]*(</\Q$field\E>)}{$1$val$2}g;
    return $axis_str;
  }

  my $i = 0;
  my $out = q{};
  pos($_) = 0;

  # Process the whole buffer joint-by-joint
  while (/\G(.*?)(<joint\b.*?<\/joint>)/sgc) {
    $out .= $1;
    my $j = $2;

    if (($i - $sel0) % $k == 0 && $i >= $sel0) {
      # --- AXIS 1: grab the first <axis>...</axis> block in this joint and edit in-place ---
      if ($j =~ m{(<axis\b[^>]*>.*?</axis>)}s) {
        my $axis = $1;
        my $axis_mod = $axis;
        $axis_mod = axis_set_field($axis_mod, "damping",          "'"$JOINT1_K_DAMPING"'"         );
        $axis_mod = axis_set_field($axis_mod, "friction",         "'"$JOINT1_K_FRICTION"'"        );
        $axis_mod = axis_set_field($axis_mod, "spring_reference", "'"$JOINT1_K_SPRING_REFERENCE"'" );
        $axis_mod = axis_set_field($axis_mod, "spring_stiffness", "'"$JOINT1_K_SPRING_STIFFNESS"'" );
        # replace exactly the axis block we captured
        $j =~ s/\Q$axis\E/$axis_mod/s;
      }

      # --- AXIS 2: grab the first <axis2>...</axis2> block in this joint and edit in-place ---
      if ($j =~ m{(<axis2\b[^>]*>.*?</axis2>)}s) {
        my $axis2 = $1;
        my $axis2_mod = $axis2;
        $axis2_mod = axis_set_field($axis2_mod, "damping",          "'"$JOINT2_K_DAMPING"'"         );
        $axis2_mod = axis_set_field($axis2_mod, "friction",         "'"$JOINT2_K_FRICTION"'"        );
        $axis2_mod = axis_set_field($axis2_mod, "spring_reference", "'"$JOINT2_K_SPRING_REFERENCE"'" );
        $axis2_mod = axis_set_field($axis2_mod, "spring_stiffness", "'"$JOINT2_K_SPRING_STIFFNESS"'" );
        $j =~ s/\Q$axis2\E/$axis2_mod/s;
      }
    }

    $out .= $j;
    $i++;
  }

  # append tail after the last joint
  $out .= substr($_, pos($_));
  $_ = $out;
' model.sdf



  # --------- Special case: zero springs for FIRST & LAST joints (name-agnostic) ----------
# FIRST joint
perl -0777 -i -pe '
  if (m/^(.*?)(<joint\b.*?<\/joint>)(.*)$/s) {
    my ($pre, $first, $rest) = ($1, $2, $3);
    $first =~ s/(<spring_reference>).*?(<\/spring_reference>)/$1 0 $2/gs;
    $first =~ s/(<spring_stiffness>).*?(<\/spring_stiffness>)/$1 0 $2/gs;
    $_ = $pre . $first . $rest;
  }
' model.sdf

# LAST joint: find the last <joint>...</joint> block and set spring tags to 0
perl -0777 -i -pe '
  if (m/^(.*)(<joint\b.*<\/joint>)(.*)$/s) {
    my ($pre, $last, $post) = ($1, $2, $3);
    $last =~ s/(<spring_reference>).*?(<\/spring_reference>)/$1 0 $2/gs;
    $last =~ s/(<spring_stiffness>).*?(<\/spring_stiffness>)/$1 0 $2/gs;
    $_ = $pre . $last . $post;
  }
' model.sdf


echo "Done → model.sdf (Axis1 & Axis2 templated with #JOINT* placeholders)"
