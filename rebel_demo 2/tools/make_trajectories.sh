#!/usr/bin/env bash
# Recompute the 15 joint trajectories, e.g. after you measured the real robot position.
# Usage:  tools/make_trajectories.sh /path/to/take_v2_mocap   (writes into <package>/trajectories)
set -e
TAKES=${1:?give the folder with the raw Motive take CSVs}

# ---- EDIT THESE: ReBeL base_link in Motive WORLD coordinates (same Motive calibration as the takes)
ROBOT_ORIGIN="-0.62 1.72 0.29"   # x y z [m]  (provisional guess; z = table + 0.20 m pedestal)
ROBOT_YAW="-127"                 # deg: robot x axis = world X rotated by this about up (faces the work area)
# ---- optional: tool mounting; leave empty for position-only IK (wrist orientation free)
TCP_ARGS=""                      # e.g. "--tcp-offset 0 0 0 --tcp-rpy 0 90 0"
# ---- skip the human "bringing the tool in" part: start when the tool is within 0.30 m (horizontal)
#      of the box and < 0.35 m above it. Leave empty to keep the whole demo.
START_ARGS="--start-near-box 0.30 0.35"

HERE=$(cd "$(dirname "$0")/.." && pwd)   # package root
BUILD=$HERE/_build
rm -rf "$BUILD"; mkdir -p "$BUILD" "$HERE/trajectories"
for f in "$TAKES"/*.csv; do
  b=$(basename "$f" .csv)
  case "$b" in *alibration*) continue;; esac
  extra="$START_ARGS"
  if [ -n "$TCP_ARGS" ]; then JOINTS=""; else JOINTS="--joints-pos-only"; fi
  echo "== $b"
  python3 "$HERE/tools/motive2gr00t.py" --csv "$f" --out "$BUILD" --csv-only $JOINTS \
      --robot-origin $ROBOT_ORIGIN --robot-yaw $ROBOT_YAW $TCP_ARGS $extra > "$BUILD/$b.log" \
      || { echo "   failed, see $BUILD/$b.log"; continue; }
  grep -E '"(position_only_reachable_pct|full_pose_reachable_pct|max_joint_step_deg)"' "$BUILD/$b.log" | tr -d ' ,'
done
rm -f "$HERE"/trajectories/*.csv
cp "$BUILD"/cleaned_csv/*.csv "$HERE/trajectories/"
echo "done -> $HERE/trajectories  (quality plots in $BUILD/qc)"
echo "now rebuild:  cd <your_ws> && colcon build --packages-select rebel_demo"
