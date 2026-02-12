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
	build.sh [BUILD_TOOL] [options] [-- extra-args]

Build tool:
	(cmake)         Configure + build + install via CMake (default)
	catkin          Build ROS1 packages with catkin_tools (catkin build)
	catkin_make     Build ROS1 packages with catkin_make
	colcon          Build ROS2 packages with colcon
	ROS1            Alias for: catkin_make
	ROS2            Alias for: colcon

Options:
	--debug | --release     CMake build type (default: --release)
	-j, --jobs N            Parallelism (default: nproc)
	--workspace DIR         Workspace root (default: $PWD or STEPIT_WS)
	--clean                 Remove prior build outputs for this build tool
	--cmake-arg ARG         Extra CMake arg (repeatable), e.g. --cmake-arg -DSTEPIT_BLACKLIST_PLUGINS=...
	-D...                   Convenience: any -D... is treated as --cmake-arg
	-h, --help              Show this help message

Note:
	- If BUILD_TOOL is not specified, it will be read from '.stepit/build_tool' if it exists, or default to 'cmake'.
	- You can always use cmake as the build tool, even if you want to build with ROS / ROS2.
EOF
}


workspace_dir="${STEPIT_WS:-$PWD}"
build_tool=""
build_type="Release"
jobs="$(nproc 2>/dev/null || echo 4)"
clean=false
cmake_args=()
extra_args=()

while [[ $# -gt 0 ]]; do
	case "$1" in
		--debug)
			build_type="Debug"
			shift
			;;
		--release)
			build_type="Release"
			shift
			;;
		--workspace)
			[[ $# -ge 2 ]] || die "--workspace requires a value"
			workspace_dir="$2"
			shift 2
			;;
		-j|--jobs)
			[[ $# -ge 2 ]] || die "$1 requires a value"
			jobs="$2"
			shift 2
			;;
		--clean)
			clean=true
			shift
			;;
		--cmake-arg)
			[[ $# -ge 2 ]] || die "--cmake-arg requires a value"
			cmake_args+=("$2")
			shift 2
			;;
		-D*)
			cmake_args+=("$1")
			shift
			;;
		-h|--help)
			usage
			exit 0
			;;
		--)
			shift
			extra_args=("$@")
			break
			;;
		*)
			[[ -z "$build_tool" ]] || die "Unknown argument: $1 (try --help)"
			build_tool="$1"
			shift
			;;
	esac
done

if [[ ! -f "${workspace_dir}/src/stepit/CMakeLists.txt" ]]; then
	die "Expected ${workspace_dir}/src/stepit/CMakeLists.txt. Run from your StepIt workspace root or set STEPIT_WS."
fi
if [[ -z "$build_tool" ]]; then
	if [[ -f "${workspace_dir}/.stepit/build_tool" ]]; then
		build_tool="$(<"${workspace_dir}/.stepit/build_tool")"
	else
		build_tool="cmake"
	fi
fi

case "$build_tool" in
	ROS1) build_tool="catkin_make" ;;
	ROS2) build_tool="colcon" ;;
	cmake|catkin|catkin_make|colcon) ;;
	*) die "Unsupported build_tool: $build_tool" ;;
esac


log "${GREEN}============================= Building =============================${CLEAR}"
log "${GREEN}Workspace:${CLEAR}   ${workspace_dir}"
log "${GREEN}Build tool:${CLEAR}  ${build_tool}"
log "${GREEN}Build type:${CLEAR}  ${build_type}"
log "${GREEN}Jobs:${CLEAR}        ${jobs}"

run mkdir -p "${workspace_dir}/.stepit"
echo "$build_tool" > "${workspace_dir}/.stepit/build_tool"


case "$build_tool" in
	cmake)
		require_cmd cmake
		install_prefix="${workspace_dir}/install"
		build_dir="${workspace_dir}/build"
		if [[ "$clean" == true ]]; then
			run rm -rf "${build_dir}" "${install_prefix}"
		fi

		cmake_args+=(
			"-B${build_dir}" 
			"-S${workspace_dir}/src/stepit" 
			"-DCMAKE_BUILD_TYPE=${build_type}" 
			"-DCMAKE_INSTALL_PREFIX=${install_prefix}" 
			"-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
			"${extra_args[@]}"
		)

		skip_configure=false
		if [[ -f "${build_dir}/CMakeCache.txt" && -f "${workspace_dir}/.stepit/cmake_args" ]]; then
			last_cmake_args=$(<"${workspace_dir}/.stepit/cmake_args")
			[[ "${cmake_args[*]}" == "$last_cmake_args" ]] && skip_configure=true
		fi
		if [[ "$skip_configure" == false ]]; then
			echo "${cmake_args[*]}" > "${workspace_dir}/.stepit/cmake_args"
			run cmake "${cmake_args[@]}"
		fi

		run cmake --build "${build_dir}" -j"${jobs}"
		run cmake --install "${build_dir}"
		;;

	catkin|catkin_make)
		touch "${workspace_dir}/src/stepit/package/ros2/CATKIN_IGNORE"

		if [[ "$clean" == true ]]; then
			run rm -rf \
				"${workspace_dir}/build" \
				"${workspace_dir}/devel" \
				"${workspace_dir}/install" \
				"${workspace_dir}/logs"
		fi

		if [[ "$build_tool" == "catkin" ]]; then
			require_cmd catkin
			run catkin config \
				--cmake-args \
					-DCMAKE_BUILD_TYPE="${build_type}" \
					-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
					"${cmake_args[@]}" \
				"${extra_args[@]}"
			run catkin build -j"${jobs}"
		else
			require_cmd catkin_make
			run catkin_make \
				--cmake-args \
					-DCMAKE_BUILD_TYPE="${build_type}" \
					-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
					"${cmake_args[@]}" \
				--make-args \
					-j"${jobs}" \
				"${extra_args[@]}"
		fi
		;;

	colcon)
		if [[ "$clean" == true ]]; then
			run rm -rf \
				"${workspace_dir}/build" \
				"${workspace_dir}/install" \
				"${workspace_dir}/log"
		fi

		require_cmd colcon
		run colcon build \
			--base-paths src src/stepit/package/ros2 \
			--packages-skip stepit \
			--parallel-workers "${jobs}" \
			--cmake-args \
				-DCMAKE_BUILD_TYPE="${build_type}" \
				-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
				"${cmake_args[@]}" \
			"${extra_args[@]}"
		;;

esac

log "${GREEN}============================= Finished =============================${CLEAR}"
log "Next step:"
log "  ./scripts/run.sh ./configs/demo.conf.sh  # Replace the path with your config file"
