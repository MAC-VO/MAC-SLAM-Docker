#!/bin/bash
set -e

# Map container process to host UID/GID (consumed by `user:` in compose files).
export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"

# Pre-create container HOME on the host so Docker doesn't auto-create it as root.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
mkdir -p "${REPO_ROOT}/Cache/docker_home"

ARCH=$(uname -m)

if [ "$ARCH" = "x86_64" ]; then
    PROFILE="amd64"
elif [ "$ARCH" = "aarch64" ]; then
    PROFILE="arm64"
else
    echo "Error: Unsupported architecture: $ARCH"
    exit 1
fi

COMPOSE_FILES=(
    docker-compose.yaml
    docker-compose-type.yaml
    docker-compose-ros-humble.yaml
)

# Services that exist only as runtime entry stubs (no build: section).
SKIP_BUILD_SERVICES=" dev-unified-entry "

if [ "${1:-}" = "build" ]; then
    # Serialize image builds. `docker compose build` with buildx-bake (default
    # in Compose v2.22+) builds every service in a compose file in parallel.
    # Each service here does heavy CUDA source compilation (GTSAM, cuDSS)
    # against a ~20GB NGC PyTorch base, so parallel builds exhaust host
    # RAM and freeze the machine. We both disable bake and iterate services
    # one at a time across all compose files.
    shift
    export COMPOSE_BAKE=false

    for cf in "${COMPOSE_FILES[@]}"; do
        services=$(docker compose -f "$cf" --profile "$PROFILE" config --services)
        for svc in $services; do
            case "$SKIP_BUILD_SERVICES" in
                *" $svc "*) continue ;;
            esac
            echo ">>> Building $svc (profile=$PROFILE, file=$cf)"
            docker compose -f "$cf" --profile "$PROFILE" build "$@" "$svc"
        done
    done
else
    for cf in "${COMPOSE_FILES[@]}"; do
        docker compose -f "$cf" --profile "$PROFILE" "$@"
    done
fi
