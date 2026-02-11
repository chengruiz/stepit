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
	-n, --dry-run           Print what would be removed without removing it
	-h, --help              Show this help message

Description:
	Cleans standard build artifacts (build, devel, install, log, logs, .catkin_tools)
	from the workspace.
EOF
	exit 0
}

# Defaults
DRY_RUN=false
WORKSPACE_ROOT="${STEPIT_WS:-$PWD}"

# Parse args
while [[ $# -gt 0 ]]; do
	case "$1" in
		-h|--help)
			usage
			;;
		-n|--dry-run)
			DRY_RUN=true
			shift
			;;
		-w|--workspace)
			if [[ -n "${2:-}" ]]; then
				WORKSPACE_ROOT="$2"
				shift 2
			else
				die "Error: Argument for $1 is missing"
			fi
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
)

cd "$WORKSPACE_ROOT" || die "Failed to cd into $WORKSPACE_ROOT"

for item in "${ITEMS_TO_CLEAN[@]}"; do
	if [[ -e "$item" ]]; then
		if [[ "$DRY_RUN" == "true" ]]; then
			log "${YELLOW}[DRY-RUN]${CLEAR} rm -rf $item"
		else
			run rm -rf "$item"
		fi
	fi
done

log "${GREEN}Clean complete.${CLEAR}"
