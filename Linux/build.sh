ARCH=$(uname -m)

if [ "$ARCH" = "x86_64" ]; then
    docker compose --profile amd64 build
elif [ "$ARCH" = "aarch64" ]; then
    docker compose --profile arm64 build
fi
