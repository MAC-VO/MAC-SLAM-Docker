#!/bin/bash

# Script to start interactive Docker container with optional host directory mounting
# Usage: ./start_interact.sh [HOST_DIRECTORY]
# If HOST_DIRECTORY is provided, it will be mounted to /home/devuser/workspace in the container

HOST_DIR="$1"

if [ -n "$HOST_DIR" ]; then
    # Convert relative path to absolute path
    HOST_DIR=$(realpath "$HOST_DIR")
    
    # Check if directory exists
    if [ ! -d "$HOST_DIR" ]; then
        echo "Error: Directory '$HOST_DIR' does not exist"
        exit 1
    fi
    
    echo "Mounting host directory: $HOST_DIR -> /home/devuser/workspace"
    # Use environment variable override for volume mounting
    PROJ_DIR="$HOST_DIR" docker compose run --rm --service-ports dev bash
else
    echo "Using default volume mount from docker-compose.yaml"
    docker compose run --rm --service-ports dev bash
fi