#!/usr/bin/env bash

set -euo pipefail

if [[ -t 1 ]]; then
	GREEN=$'\033[0;32m'
	YELLOW=$'\033[0;33m'
	RED=$'\033[0;31m'
	CLEAR=$'\033[0m'
else
	GREEN=""
	YELLOW=""
	RED=""
	CLEAR=""
fi

log() {
	# shellcheck disable=SC2059
	printf "%b\n" "$*"
}

die() {
	log "${RED}ERROR:${CLEAR} $*" >&2
	exit 1
}

run() {
	log "${GREEN}>>${CLEAR} $(printf '%q ' "$@")"
	"$@"
}

require_cmd() {
	command -v "$1" >/dev/null 2>&1 || die "Missing required command: $1"
}

usage() {
	cat <<'EOF'
Usage:
	run.sh [options] CONFIG

Options:
	--workspace DIR         Workspace root (default: $PWD or STEPIT_WS)
	--build-tool TYPE       cmake | catkin | catkin_make | colcon | auto (default: auto)
	-h, --help              Show this help message

Notes:
	- CONFIG is required and should define STEPIT_* values.
	- Config keys: STEPIT_CONTROL, STEPIT_FACTORY, STEPIT_PUBLISHER, STEPIT_POLICY,
	  STEPIT_ROBOT, STEPIT_VERBOSITY, STEPIT_PLUGIN_ARGS
EOF
}

workspace_dir="${STEPIT_WS:-$PWD}"
config_file=""
build_tool="auto"
while [[ $# -gt 0 ]]; do
	case "$1" in
		--workspace)
			[[ $# -ge 2 ]] || die "--workspace requires a value"
			workspace_dir="$2"
			shift 2
			;;
		--build-tool)
			[[ $# -ge 2 ]] || die "--build-tool requires a value"
			build_tool="$2"
			shift 2
			;;
		-h|--help)
			usage
			exit 0
			;;
		*)
            [[ -z "${config_file}" ]] && config_file="$1" || die "Unexpected extra argument: $1"
			shift
			;;
	esac
done

if [[ -z "${config_file}" ]]; then
	usage
	die "Missing required CONFIG argument."
fi

if [[ ! -f "${workspace_dir}/src/stepit/CMakeLists.txt" ]]; then
	die "Expected ${workspace_dir}/src/stepit/CMakeLists.txt. Run from your StepIt workspace root or set STEPIT_WS."
fi

detect_build_tool() {
	if [[ -f "${workspace_dir}/install/.colcon_install_layout" || -f "${workspace_dir}/install/_local_setup_util_sh.py" ]]; then
		printf '%s' "colcon"
		return 0
	fi
	if [[ -f "${workspace_dir}/devel/setup.bash" || -d "${workspace_dir}/.catkin_tools" ]]; then
		printf '%s' "catkin"
		return 0
	fi
	if [[ -f "${workspace_dir}/build/CMakeCache.txt" || -x "${workspace_dir}/install/bin/stepit" || -x "${workspace_dir}/build/bin/stepit" ]]; then
		printf '%s' "cmake"
		return 0
	fi
	die "Failed to detect build tool. Pass --build-tool cmake|catkin|catkin_make|colcon."
}

case "${build_tool}" in
	auto) build_tool="$(detect_build_tool)"          ;;
	cmake|catkin|catkin_make|colcon)                 ;;
	*) die "Unsupported --build-tool: ${build_tool}" ;;
esac

[[ -f "${config_file}" ]] || die "Config file not found: ${config_file}"
# shellcheck disable=SC1090
source "${config_file}"

stepit_args=()
append_args() {
    local flag="$1"
    local value="${2:-}"
    [[ -n "${value}" ]] || return 0
    for token in ${value}; do
        stepit_args+=("${flag}" "${token}")
    done
}
append_args "--control" "${STEPIT_CONTROL}"
append_args "--factory" "${STEPIT_FACTORY}"
append_args "--publisher" "${STEPIT_PUBLISHER}"
append_args "--policy" "${STEPIT_POLICY}"
append_args "--robot" "${STEPIT_ROBOT}"
append_args "--verbosity" "${STEPIT_VERBOSITY}"

log "${GREEN}=============================== Running =============================${CLEAR}"
log "${GREEN}Workspace:${CLEAR}   ${workspace_dir}"
log "${GREEN}Build tool:${CLEAR}  ${build_tool}"
log "${GREEN}Config:${CLEAR}      ${config_file}"

stepit_cmd=()
case "${build_tool}" in
	cmake)
        export LD_LIBRARY_PATH="${workspace_dir}/install/lib:${LD_LIBRARY_PATH:-}"
		if [[ -x "${workspace_dir}/install/bin/stepit" ]]; then
			stepit_cmd=("${workspace_dir}/install/bin/stepit")
		else
			die "stepit executable not found in install/bin. Build and install first."
		fi
		;;
	catkin|catkin_make)
		require_cmd rosrun
		setup_script="${workspace_dir}/devel/setup.bash"
		[[ -f "${setup_script}" ]] || die "Missing ${setup_script}. Build with catkin/catkin_make first."
		# shellcheck disable=SC1090
		source "${setup_script}"
		stepit_cmd=(rosrun stepit_ros stepit)
		;;
	colcon)
		require_cmd ros2
		setup_script="${workspace_dir}/install/setup.bash"
		[[ -f "${setup_script}" ]] || die "Missing ${setup_script}. Build with colcon first."
		# shellcheck disable=SC1090
		set +u
		source "${setup_script}"
		set -u
		stepit_cmd=(ros2 run stepit_ros2 stepit)
		;;
esac

plugin_args=()
[[ -n "${STEPIT_PLUGIN_ARGS-}" ]] && read -r -a plugin_args <<< "${STEPIT_PLUGIN_ARGS}"
if [[ ${#plugin_args[@]} -gt 0 ]]; then
	run "${stepit_cmd[@]}" "${stepit_args[@]}" -- "${plugin_args[@]}"
else
	run "${stepit_cmd[@]}" "${stepit_args[@]}"
fi
