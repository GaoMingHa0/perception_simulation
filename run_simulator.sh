#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TRACKS_DIR="${ROOT_DIR}/tracks"

usage() {
  cat <<EOF
Usage:
  ./run_simulator.sh [track-name|track-file]

Examples:
  ./run_simulator.sh
  ./run_simulator.sh trackdrive
  ./run_simulator.sh acceleration.yaml
  ./run_simulator.sh /absolute/path/to/custom_track.yaml
EOF
}

source_setup_file() {
  local setup_file="$1"

  set +u
  # shellcheck source=/dev/null
  source "${setup_file}"
  set -u
}

source_ros_environment() {
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    source_setup_file "/opt/ros/${ROS_DISTRO}/setup.bash"
  elif [[ -z "${ROS_DISTRO:-}" ]]; then
    for setup_file in /opt/ros/*/setup.bash; do
      if [[ -f "${setup_file}" ]]; then
        source_setup_file "${setup_file}"
        break
      fi
    done
  fi

  if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
    source_setup_file "${ROOT_DIR}/install/setup.bash"
  fi
}

collect_tracks() {
  mapfile -t TRACK_FILES < <(find "${TRACKS_DIR}" -maxdepth 1 -type f -name '*.yaml' | sort)

  if [[ "${#TRACK_FILES[@]}" -eq 0 ]]; then
    echo "No track YAML files found in ${TRACKS_DIR}" >&2
    exit 1
  fi
}

resolve_track_arg() {
  local requested="$1"

  if [[ -f "${requested}" ]]; then
    SELECTED_TRACK="$(cd "$(dirname "${requested}")" && pwd)/$(basename "${requested}")"
    return
  fi

  local candidate="${requested}"
  if [[ "${candidate}" != *.yaml ]]; then
    candidate="${candidate}.yaml"
  fi

  if [[ -f "${TRACKS_DIR}/${candidate}" ]]; then
    SELECTED_TRACK="${TRACKS_DIR}/${candidate}"
    return
  fi

  echo "Track not found: ${requested}" >&2
  echo "Available tracks:" >&2
  for track_file in "${TRACK_FILES[@]}"; do
    echo "  - $(basename "${track_file}" .yaml)" >&2
  done
  exit 1
}

select_track_interactively() {
  echo "Select track:"
  local i
  for i in "${!TRACK_FILES[@]}"; do
    printf "  %d) %s\n" "$((i + 1))" "$(basename "${TRACK_FILES[$i]}" .yaml)"
  done

  local choice
  while true; do
    read -r -p "Enter number [1-${#TRACK_FILES[@]}]: " choice
    if [[ "${choice}" =~ ^[0-9]+$ ]] && (( choice >= 1 && choice <= ${#TRACK_FILES[@]} )); then
      SELECTED_TRACK="${TRACK_FILES[$((choice - 1))]}"
      return
    fi
    echo "Invalid choice."
  done
}

main() {
  if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
  fi

  collect_tracks
  source_ros_environment

  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 command not found. Please install/source ROS 2 first." >&2
    exit 1
  fi

  if ! ros2 pkg prefix lidar_sim >/dev/null 2>&1; then
    echo "ROS 2 package 'lidar_sim' is not available." >&2
    echo "Build and source this workspace first:" >&2
    echo "  colcon build --packages-select lidar_sim" >&2
    echo "  source install/setup.bash" >&2
    exit 1
  fi

  export ROS_LOG_DIR="${ROS_LOG_DIR:-${ROOT_DIR}/log/ros}"
  mkdir -p "${ROS_LOG_DIR}"

  if [[ -n "${1:-}" ]]; then
    resolve_track_arg "$1"
  else
    select_track_interactively
  fi

  echo "Starting lidar simulator with track: ${SELECTED_TRACK}"
  exec ros2 launch lidar_sim lidar_simulator.launch.py "track_file:=${SELECTED_TRACK}"
}

main "$@"
