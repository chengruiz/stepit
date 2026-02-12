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

usage() {
	cat <<EOF
Usage:
	$(basename "${BASH_SOURCE[0]}") [options]

Options:
	-w, --workspace DIR     Workspace root (default: inferred from script location)
	-y, --yes               Clean without confirmation
	-h, --help              Show this help message

Description:
	Cleans standard build artifacts (build, devel, install, log, logs, .catkin_tools)
	from the workspace.
EOF
	exit 0
}

# Defaults
FORCE=false
WORKSPACE_ROOT="${STEPIT_WS:-$PWD}"

# Parse args
while [[ $# -gt 0 ]]; do
	case "$1" in
		-h|--help)
			usage
			;;
		-y|--yes)
			FORCE=true
			shift
			;;
		-w|--workspace)
			[[ -n "${2:-}" ]] || die "Error: Argument for $1 is missing"
			WORKSPACE_ROOT="$2"
			shift 2
			;;
		*)
			die "Unknown argument: $1"
			;;
	esac
done

if [[ ! -f "${WORKSPACE_ROOT}/src/stepit/CMakeLists.txt" ]]; then
	die "Expected ${WORKSPACE_ROOT}/src/stepit/CMakeLists.txt. Run from your StepIt workspace root or set STEPIT_WS."
fi

log "Cleaning workspace at: $WORKSPACE_ROOT"

# Items to clean
ITEMS_TO_CLEAN=(
	"build"
	"devel"
	"log"
	"logs"
	"install"
	".catkin_tools"
	".catkin_workspace"
	".stepit"
)

cd "$WORKSPACE_ROOT"
items=()
for item in "${ITEMS_TO_CLEAN[@]}"; do
	[[ -e "$item" ]] && items+=("$item")
done

if [[ ${#items[@]} -eq 0 ]]; then
	log "Nothing to clean."
	exit 0
fi

if [[ "$FORCE" == "false" ]]; then
	log "The following command will be executed:"
	log "${YELLOW}>> rm -rf ${items[*]}${CLEAR}"
	read -p "Do you want to proceed? [Y/n] " -n 1 -r
	echo
	if [[ $REPLY =~ ^[Nn]$ ]]; then
		log "Aborting clean."
		exit 0
	fi
	rm -rf "${items[@]}"
else
	run rm -rf "${items[@]}"
fi

log "${GREEN}Clean complete.${CLEAR}"
