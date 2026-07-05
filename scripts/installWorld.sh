#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD_DIR="$PROJECT_ROOT/world"
MAP_SOURCE="$WORLD_DIR/maps/map_7x7.txt"

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

if [[ ! -d "$WORLD_DIR/models" || ! -f "$WORLD_DIR/worlds/clover_aruco.world" || ! -f "$MAP_SOURCE" ]]; then
  echo "FireSafety world files are incomplete under: $WORLD_DIR" >&2
  exit 1
fi

mkdir -p "$HOME/.gazebo/models"

for model_dir in "$WORLD_DIR/models"/*; do
  [[ -d "$model_dir" && -f "$model_dir/model.config" ]] || continue
  model_name="$(basename "$model_dir")"
  target="$HOME/.gazebo/models/$model_name"
  if [[ -L "$target" ]]; then
    rm "$target"
  elif [[ -e "$target" ]]; then
    backup="$target.backup.$(date +%Y%m%d_%H%M%S)"
    mv "$target" "$backup"
    echo "Existing model moved to backup: $backup"
  fi
  ln -s "$model_dir" "$target"
done

solar_model="$HOME/.gazebo/models/solar_panel"
if [[ -L "$solar_model" ]]; then
  rm "$solar_model"
fi

CLOVER_SIMULATION_PATH="$(find_clover_simulation || true)"
if [[ -z "$CLOVER_SIMULATION_PATH" ]]; then
  echo "clover_simulation package not found; Gazebo models were installed only." >&2
  exit 0
fi

target_world="$CLOVER_SIMULATION_PATH/resources/worlds/clover_aruco.world"
backup_world="$target_world.before_firesafety.bak"
if [[ -f "$target_world" && ! -f "$backup_world" ]]; then
  cp "$target_world" "$backup_world"
  echo "Backup created: $backup_world"
fi
cp "$WORLD_DIR/worlds/clover_aruco.world" "$target_world"

MAP_DIR="$HOME/catkin_ws/src/clover/aruco_pose/map"
if [[ -d "$MAP_DIR" ]]; then
  for map_name in map.txt cmit.txt; do
    target_map="$MAP_DIR/$map_name"
    backup_map="$target_map.before_firesafety.bak"
    if [[ -f "$target_map" && ! -f "$backup_map" ]]; then
      cp "$target_map" "$backup_map"
      echo "Backup created: $backup_map"
    fi
    cp "$MAP_SOURCE" "$target_map"
  done
else
  echo "Aruco map directory not found: $MAP_DIR" >&2
fi

echo "Installed FireSafety world: $target_world"
echo "Installed FireSafety ArUco 7x7 maps into: $MAP_DIR"
echo "Restart Gazebo/Clover after installation."
