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

version_ge() {
	local have="$1"
	local need="$2"
	local lowest
	lowest="$(printf '%s\n%s\n' "${need}" "${have}" | sort -V | head -n 1)"
	[[ "${lowest}" == "${need}" ]]
}

cmake_version() {
	local cmake_bin="$1"
	local version_line
	version_line="$("${cmake_bin}" --version 2>/dev/null | head -n 1 || true)"
	[[ "${version_line}" =~ ([0-9]+(\.[0-9]+)+) ]] || return 1
	printf '%s' "${BASH_REMATCH[1]}"
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

download_file() {
	local url="$1"
	local output="$2"
	if command -v curl >/dev/null 2>&1; then
		run curl -L --fail --show-error -o "${output}" "${url}"
	elif command -v wget >/dev/null 2>&1; then
		run wget -O "${output}" "${url}"
	else
		die "Missing curl or wget; install one to download the workspace CMake package."
	fi
}

detect_cmake_package_arch() {
	local kernel
	local machine
	kernel="$(uname -s)"
	machine="$(uname -m)"

	[[ "${kernel}" == "Linux" ]] || die "Workspace CMake bootstrap currently supports Linux only."
	case "${machine}" in
		x86_64|amd64) printf '%s' "x86_64" ;;
		aarch64|arm64) printf '%s' "aarch64" ;;
		*) die "Unsupported CPU architecture for CMake bootstrap: ${machine}" ;;
	esac
}

ensure_workspace_cmake() {
	local min_version="3.23"
	local bootstrap_version="3.31.12"
	local cmake_bin=""
	local detected_version=""

	if command -v cmake >/dev/null 2>&1; then
		cmake_bin="$(command -v cmake)"
		detected_version="$(cmake_version "${cmake_bin}" || true)"
		if [[ -n "${detected_version}" ]] && version_ge "${detected_version}" "${min_version}"; then
			log "CMake:    ${cmake_bin} (${detected_version}, satisfies >= ${min_version})"
			return 0
		fi
	fi

	local tools_dir="${workspace_dir}/tools"
	local cmake_link="${tools_dir}/cmake"
	if [[ -x "${cmake_link}/bin/cmake" ]]; then
		detected_version="$(cmake_version "${cmake_link}/bin/cmake" || true)"
		if [[ -n "${detected_version}" ]] && version_ge "${detected_version}" "${min_version}"; then
			log "CMake:    ${cmake_link}/bin/cmake (${detected_version}, workspace-local)"
			return 0
		fi
	fi

	local arch
	local package_name
	local archive
	local sha_file
	local base_url
	local expected_sha
	local actual_sha
	arch="$(detect_cmake_package_arch)"
	package_name="cmake-${bootstrap_version}-linux-${arch}"
	archive="${tools_dir}/${package_name}.tar.gz"
	sha_file="${tools_dir}/cmake-${bootstrap_version}-SHA-256.txt"
	base_url="https://github.com/Kitware/CMake/releases/download/v${bootstrap_version}"

	log "${GREEN}Preparing workspace CMake ${bootstrap_version}...${CLEAR}"
	run mkdir -p "${tools_dir}"
	download_file "${base_url}/${package_name}.tar.gz" "${archive}"
	download_file "${base_url}/cmake-${bootstrap_version}-SHA-256.txt" "${sha_file}"

	require_cmd awk
	require_cmd sha256sum
	expected_sha="$(awk -v file="$(basename "${archive}")" '
		index($0, file) {
			for (i = 1; i <= NF; ++i) {
				if (length($i) == 64 && $i ~ /^[0-9a-fA-F]+$/) {
					print tolower($i)
					exit
				}
			}
		}
	' "${sha_file}")"
	[[ -n "${expected_sha}" ]] || die "Could not find checksum for $(basename "${archive}") in ${sha_file}."
	actual_sha="$(sha256sum "${archive}" | awk '{print $1}')"
	[[ "${actual_sha}" == "${expected_sha}" ]] \
		|| die "Checksum mismatch for ${archive}."

	require_cmd tar
	run rm -rf "${tools_dir}/${package_name}"
	run tar -xzf "${archive}" -C "${tools_dir}"
	ensure_symlink "${tools_dir}/${package_name}" "${cmake_link}"

	detected_version="$(cmake_version "${cmake_link}/bin/cmake" || true)"
	[[ -n "${detected_version}" ]] && version_ge "${detected_version}" "${min_version}" \
		|| die "Downloaded CMake does not satisfy >= ${min_version}: ${cmake_link}/bin/cmake"
	log "CMake:    ${cmake_link}/bin/cmake (${detected_version}, workspace-local)"
}

usage() {
	cat <<'EOF'
Usage:
	setup.sh [options]

Options:
	-w, --workspace DIR    Workspace root (default: current directory or ./stepit_ws)
	-r, --repo URL         StepIt repository URL (default: https://github.com/chengruiz/stepit.git)
	--zoo                  Clone or update the zoo repo at $STEPIT_WS/zoo
	--zoo-repo URL         Zoo repository URL (default: https://github.com/chengruiz/stepit_zoo.git)
	-h, --help             Show this help message

Description:
	Create or update a StepIt workspace, then add config symlinks and helper script symlinks.

Environment:
	STEPIT_WS, STEPIT_REPO, STEPIT_ZOO, STEPIT_ZOO_REPO

Note:
	The script may prompt for sudo if it needs to install system packages.
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
		curl
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

ensure_workspace_cmake

if [[ "${enable_zoo}" == true ]]; then
	if [[ -e "${zoo_dir}" ]]; then
		log "${YELLOW}Note:${CLEAR} ${zoo_dir} already exists; checking repo state."

		git -C "${zoo_dir}" rev-parse --is-inside-work-tree >/dev/null 2>&1 \
			|| die "${zoo_dir} exists but is not a git repo."

		git_dirty="$(git -C "${zoo_dir}" status --porcelain)"
		if [[ -n "${git_dirty}" ]]; then
			log "${YELLOW}Warning:${CLEAR} StepIt Zoo has uncommitted changes; skipping git fetch/pull."
		else
			run git -C "${zoo_dir}" fetch --all --prune
			run git -C "${zoo_dir}" pull --ff-only
		fi
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
for file in "${stepit_dir}"/config/run/*; do
	ensure_symlink "${file}" "${workspace_dir}/configs/$(basename "${file}")"
done

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
