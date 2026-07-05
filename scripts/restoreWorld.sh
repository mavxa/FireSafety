#!/usr/bin/env bash
set -euo pipefail

find_clover_simulation() {
  if command -v rospack >/dev/null 2>&1; then
    local package_path
    package_path="$(rospack find clover_simulation 2>/dev/null || true)"
    if [[ -n "$package_path" && -d "$package_path" ]]; then
      echo "$package_path"
      return 0
    fi
  fi

  for candidate in \
    "$HOME/catkin_ws/src/clover/clover_simulation" \
    "$HOME/catkin_ws/install/share/clover_simulation"; do
    if [[ -d "$candidate" ]]; then
      echo "$candidate"
      return 0
    fi
  done
  return 1
}

CLOVER_SIMULATION_PATH="$(find_clover_simulation || true)"
if [[ -n "$CLOVER_SIMULATION_PATH" ]]; then
  target_world="$CLOVER_SIMULATION_PATH/resources/worlds/clover_aruco.world"
  backup_world="$target_world.before_firesafety.bak"
  if [[ -f "$backup_world" ]]; then
    cp "$backup_world" "$target_world"
    echo "Restored world: $target_world"
  fi
fi

MAP_DIR="$HOME/catkin_ws/src/clover/aruco_pose/map"
for map_name in map.txt cmit.txt; do
  target_map="$MAP_DIR/$map_name"
  backup_map="$target_map.before_firesafety.bak"
  if [[ -f "$backup_map" ]]; then
    cp "$backup_map" "$target_map"
    echo "Restored ArUco map: $target_map"
  fi
done

for model_name in aruco_cmit_txt parquet_plane red_box yellow_box green_box blue_light_box blue_box orange_box pink_box violet_box; do
  target="$HOME/.gazebo/models/$model_name"
  if [[ -L "$target" ]]; then
    link_target="$(readlink "$target")"
    case "$link_target" in
      *"/FireSafety/world/models/"*)
        rm "$target"
        echo "Removed FireSafety model symlink: $target"
        ;;
    esac
  fi
done
