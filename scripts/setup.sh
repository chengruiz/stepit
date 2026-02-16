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

ensure_symlink() {
	# ensure_symlink TARGET LINK_PATH
	# Creates LINK_PATH -> TARGET if missing.
	# If LINK_PATH is an existing symlink, updates it to point to TARGET.
	# If LINK_PATH exists as a regular file/dir, leaves it alone and warns.
	local target="$1"
	local link_path="$2"

	if [[ -L "${link_path}" ]]; then
		local current
		current="$(readlink "${link_path}" || true)"
		if [[ "${current}" != "${target}" ]]; then
			run ln -sf "${target}" "${link_path}"
		fi
		return 0
	fi

	if [[ -e "${link_path}" ]]; then
		log "${YELLOW}Warning:${CLEAR} ${link_path} exists and is not a symlink; leaving it unchanged."
		return 0
	fi

	run ln -s "${target}" "${link_path}"
}

usage() {
	cat <<'EOF'
Usage:
	setup.sh [options]

Options:
	-w, --workspace DIR    Workspace root (default: inferred from script location)
	-r, --repo URL         StepIt repository URL (default: https://github.com/chengruiz/stepit.git)
	--zoo                  Also clone/update the zoo repo (dir: $STEPIT_WS/zoo)
	--zoo-repo URL         Zoo repository URL (default: https://github.com/chengruiz/stepit_zoo.git)
	-h, --help             Show this help message

Description:
	Initializes a StepIt workspace by cloning the repository (or updating if it already exists)
	and creating config files and script symlinks.

Environment overrides:
	STEPIT_WS, STEPIT_REPO, STEPIT_ZOO, STEPIT_ZOO_REPO

Notes:
	- You will be prompted for sudo password if needed.
EOF
}

default_workspace() {
	local pwd_name
	pwd_name="${PWD##*/}"
	if [[ -d "${PWD}/src" || "${pwd_name}" == "stepit_ws" ]]; then
		printf '%s' "${PWD}"
	else
		printf '%s' "${PWD}/stepit_ws"
	fi
}

workspace_dir="${STEPIT_WS:-$(default_workspace)}"
repo_url="${STEPIT_REPO:-https://github.com/chengruiz/stepit.git}"
zoo_dir="${STEPIT_ZOO:-}"
zoo_repo_url="${STEPIT_ZOO_REPO:-https://github.com/chengruiz/stepit_zoo.git}"
enable_zoo=false

while [[ $# -gt 0 ]]; do
	case "$1" in
		-w|--workspace)
			[[ $# -ge 2 ]] || die "--workspace requires a value"
			workspace_dir="$2"
			shift 2
			;;
		-r|--repo)
			[[ $# -ge 2 ]] || die "--repo requires a value"
			repo_url="$2"
			shift 2
			;;
		--zoo)
			enable_zoo=true
			shift
			;;
		--zoo-repo)
			[[ $# -ge 2 ]] || die "--zoo-repo requires a value"
			zoo_repo_url="$2"
			enable_zoo=true
			shift 2
			;;
		-h|--help)
			usage
			exit 0
			;;
		*) die "Unknown argument: $1 (try --help)" ;;
	esac
done

[[ -z "${zoo_dir}" ]] && zoo_dir="${workspace_dir}/zoo"


log "${GREEN}============================ Setting up ============================${CLEAR}"
log "Workspace: ${workspace_dir}"
log "Repo:      ${repo_url}"
if [[ "${enable_zoo}" == true ]]; then
	log "Zoo:       ${zoo_dir}"
	log "Zoo repo:  ${zoo_repo_url}"
fi
log

stepit_dir="${workspace_dir}/src/stepit"
if [[ -e "${stepit_dir}" ]]; then
	log "${YELLOW}Note:${CLEAR} ${stepit_dir} already exists; checking repo state."

	require_cmd git
	git -C "${stepit_dir}" rev-parse --is-inside-work-tree >/dev/null 2>&1 \
		|| die "${stepit_dir} exists but is not a git repo."

	git_dirty="$(git -C "${stepit_dir}" status --porcelain)"
	if [[ -n "${git_dirty}" ]]; then
		log "${YELLOW}Warning:${CLEAR} StepIt has uncommitted changes; skipping git fetch/pull."
	else
		run git -C "${stepit_dir}" fetch --all --prune
		run git -C "${stepit_dir}" pull --ff-only
		run git -C "${stepit_dir}" submodule update --init extern/llu
	fi
else
	command -v apt-get >/dev/null 2>&1 \
		|| die "apt-get not found. This script currently supports Debian/Ubuntu via apt."

	if [[ "$(id -u)" -eq 0 ]]; then
		sudo_cmd=()
	elif command -v sudo >/dev/null 2>&1; then
		sudo_cmd=(sudo)
	else
		die "sudo not found (and not running as root). Install sudo or run as root."
	fi

	deps=(
		ca-certificates
		git
		cmake
		build-essential
		libboost-dev
		libboost-filesystem-dev
		libboost-program-options-dev
		libeigen3-dev
		libfmt-dev
		libyaml-cpp-dev
	)

	log "${GREEN}Installing system dependencies...${CLEAR}"
	run "${sudo_cmd[@]}" apt-get update
	run "${sudo_cmd[@]}" apt-get install -y --no-install-recommends "${deps[@]}"

	log "${GREEN}Creating workspace...${CLEAR}"
	run mkdir -p "${workspace_dir}/src"

	log "${GREEN}Fetching StepIt source...${CLEAR}"
	run git clone --depth 1 "$repo_url" "${stepit_dir}"
	run git -C "${stepit_dir}" submodule update --init extern/llu
fi

if [[ "${enable_zoo}" == true ]]; then
	if [[ -e "${zoo_dir}" ]]; then
		log "${YELLOW}Note:${CLEAR} ${zoo_dir} already exists; checking repo state."

		git -C "${zoo_dir}" rev-parse --is-inside-work-tree >/dev/null 2>&1 \
			|| die "${zoo_dir} exists but is not a git repo."

		run git -C "${zoo_dir}" fetch --all --prune
		run git -C "${zoo_dir}" pull --ff-only
	else
		run mkdir -p "$(dirname "${zoo_dir}")"
		log "${GREEN}Fetching StepIt Zoo...${CLEAR}"
		run git clone --depth 1 "${zoo_repo_url}" "${zoo_dir}"
	fi
fi

run mkdir -p "${workspace_dir}/scripts"
ensure_symlink "${stepit_dir}/scripts/setup.sh" "${workspace_dir}/scripts/setup.sh"
ensure_symlink "${stepit_dir}/scripts/build.sh" "${workspace_dir}/scripts/build.sh"
ensure_symlink "${stepit_dir}/scripts/run.sh"   "${workspace_dir}/scripts/run.sh"
ensure_symlink "${stepit_dir}/scripts/clean.sh" "${workspace_dir}/scripts/clean.sh"
run mkdir -p "${workspace_dir}/configs"
run cp -a "${stepit_dir}/config/run/." "${workspace_dir}/configs/"

log "${GREEN}============================= Finished =============================${CLEAR}"

log "Next steps:"
log "  cd ${workspace_dir}"
log "  ./scripts/build.sh                # Build StepIt with CMake"
log "  # Or, to build with other tools:"
log "  ./scripts/build.sh ROS1           # Build StepIt with catkin_make (ROS1)"
log "  ./scripts/build.sh ROS2           # Build StepIt with colcon build (ROS2)"
log "  ./scripts/build.sh catkin         # Build StepIt with catkin build (ROS1)"
log "  ./scripts/build.sh catkin_make    # Build StepIt with catkin_make (ROS1)"
log "  ./scripts/build.sh colcon         # Build StepIt with colcon build (ROS2)"
