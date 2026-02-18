ARCH=$(uname -m)

if [ "$ARCH" = "x86_64" ]; then
    PROFILE="amd64"
elif [ "$ARCH" = "aarch64" ]; then
    PROFILE="arm64"
else
    echo "Error: Unsupported architecture: $ARCH"
    exit 1
fi

docker compose -f docker-compose.yaml --profile "$PROFILE" "$@" && \
docker compose -f docker-compose-type.yaml --profile "$PROFILE" "$@" && \
docker compose -f docker-compose-ros-humble.yaml --profile "$PROFILE" "$@"
