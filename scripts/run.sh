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
	run.sh [options] [CONFIG...]

Options:
	CONFIG                  Path to config file (repeatable, default: none)
	--workspace DIR         Workspace root (default: $PWD or STEPIT_WS)
	--build-tool TYPE       cmake | catkin | catkin_make | colcon | auto (default: auto)
	--gdb                   Run stepit under gdb
	-c, --control VALUE     Control input type (repeatable)
	-f, --factory VALUE     Default factory (repeatable, format: <class>@<factory_name>)
	-P, --publisher VALUE   Publisher type
	-p, --policy VALUE      Policies (format: [<type>@]<directory>)
	-r, --robot VALUE       Robot type
	-v, --verbosity VALUE   Verbosity level (0-3)
	-h, --help              Show this help message
	--                      Pass remaining arguments to plugins
EOF
}

workspace_dir="${STEPIT_WS:-$PWD}"
build_tool="auto"
use_gdb=false
config_files=()
stepit_args=()
plugin_args=()
while [[ $# -gt 0 ]]; do
	case "$1" in
		--)
			shift
			if [[ $# -gt 0 ]]; then
				plugin_args+=("$@")
			fi
			break
			;;
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
		--gdb)
			use_gdb=true
			shift
			;;
		--control|-c|--factory|-f|--publisher|-P|--policy|-p|--robot|-r|--verbosity|-v)
			[[ $# -ge 2 ]] || die "$1 requires a value"
			stepit_args+=("$1" "$2")
			shift 2
			;;
		-h|--help)
			usage
			exit 0
			;;
		*)
			config_files+=("$1")
			shift
			;;
	esac
done

if [[ ! -f "${workspace_dir}/src/stepit/CMakeLists.txt" ]]; then
	die "Expected ${workspace_dir}/src/stepit/CMakeLists.txt. Run from your StepIt workspace root or set STEPIT_WS."
fi

detect_build_tool() {
	if [[ -f "${workspace_dir}/.stepit/build_tool" ]]; then
		printf '%s' "$(<"${workspace_dir}/.stepit/build_tool")"
		return 0
	fi
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

log "${GREEN}=============================== Running =============================${CLEAR}"
log "${GREEN}Workspace:${CLEAR}   ${workspace_dir}"
log "${GREEN}Build tool:${CLEAR}  ${build_tool}"
if [[ "${use_gdb}" == true ]]; then
	require_cmd gdb
	log "${GREEN}Debugger:${CLEAR}    gdb"
fi
if [[ ${#config_files[@]} -gt 0 ]]; then
	log "${GREEN}Config:${CLEAR}      ${config_files[*]}"
else
	log "${GREEN}Config:${CLEAR}      (none)"
fi

stepit_cmd=()
debug_prefix=()
ros_debug_prefix=()
if [[ "${use_gdb}" == true ]]; then
	debug_prefix=(gdb -ex run --args)
	ros_debug_prefix=(--prefix "${debug_prefix[*]}")
fi
case "${build_tool}" in
	cmake)
		stepit_bin="${workspace_dir}/install/bin/stepit"
		[[ -x "${stepit_bin}" ]] || die "Missing ${stepit_bin}. Build and install first."
        export LD_LIBRARY_PATH="${workspace_dir}/install/lib:${LD_LIBRARY_PATH:-}"
		stepit_cmd=("${debug_prefix[@]}" "${stepit_bin}")
		;;
	catkin|catkin_make)
		require_cmd rosrun
		setup_script="${workspace_dir}/devel/setup.bash"
		[[ -f "${setup_script}" ]] || die "Missing ${setup_script}. Build with catkin/catkin_make first."
		# shellcheck disable=SC1090
		source "${setup_script}"
		stepit_cmd=(rosrun "${ros_debug_prefix[@]}" stepit_ros stepit)
		;;
	colcon)
		require_cmd ros2
		setup_script="${workspace_dir}/install/setup.bash"
		[[ -f "${setup_script}" ]] || die "Missing ${setup_script}. Build with colcon first."
		set +u
		# shellcheck disable=SC1090
		source "${setup_script}"
		set -u
		stepit_cmd=(ros2 run "${ros_debug_prefix[@]}" stepit_ros2 stepit)
		;;
	*)
		die "Unsupported build tool: ${build_tool}"
		;;
esac

export STEPIT_WS="${workspace_dir}"
export CONFIG_HOME="${workspace_dir}/configs"
if [[ ${#config_files[@]} -gt 0 ]]; then
	# Allows chaining configs to pass arguments via STEPIT_ARGS and STEPIT_PLUGIN_ARGS
	for config_file in "${config_files[@]}"; do
		[[ -f "${config_file}" ]] || die "Config file not found: ${config_file}"
		# shellcheck disable=SC1090
		source "${config_file}"
		if [[ -n "${STEPIT_ARGS-}" ]]; then
			read -r -a extra_args <<< "${STEPIT_ARGS}"
			stepit_args+=("${extra_args[@]}")
		fi
		if [[ -n "${STEPIT_PLUGIN_ARGS-}" ]]; then
			read -r -a extra_plugin_args <<< "${STEPIT_PLUGIN_ARGS}"
			plugin_args+=("${extra_plugin_args[@]}")
		fi
	done
fi

if [[ ${#plugin_args[@]} -gt 0 ]]; then
	stepit_args+=("--" "${plugin_args[@]}")
fi
run "${stepit_cmd[@]}" "${stepit_args[@]}"
