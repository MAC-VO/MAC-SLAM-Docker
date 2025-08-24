#!/bin/bash

# Script to start interactive Docker container with optional host directory mounting
# Usage: ./start_interact.sh [HOST_DIRECTORY] [DATA_DIRECTORY]
# If HOST_DIRECTORY is provided, it will be mounted to /home/root/workspace in the container
# If DATA_DIRECTORY is provided, it will be mounted to /Data in the container

HOST_DIR="$1"
DATA_DIR="$2"

# Validate and process HOST_DIR if provided
if [ -n "$HOST_DIR" ]; then
    # Convert relative path to absolute path
    HOST_DIR=$(realpath "$HOST_DIR")
    
    # Check if directory exists
    if [ ! -d "$HOST_DIR" ]; then
        echo "Error: Directory '$HOST_DIR' does not exist"
        exit 1
    fi
    
    echo "Mounting host directory: $HOST_DIR -> /home/root/workspace"
fi

# Validate and process DATA_DIR if provided
if [ -n "$DATA_DIR" ]; then
    # Convert relative path to absolute path
    DATA_DIR=$(realpath "$DATA_DIR")
    
    # Check if directory exists
    if [ ! -d "$DATA_DIR" ]; then
        echo "Error: Directory '$DATA_DIR' does not exist"
        exit 1
    fi
    
    echo "Mounting data directory: $DATA_DIR -> /Data"
fi

# Run docker compose with appropriate environment variables
if [ -n "$HOST_DIR" ] && [ -n "$DATA_DIR" ]; then
    PROJ_DIR="$HOST_DIR" DATA_DIR="$DATA_DIR" docker compose run --rm --service-ports l4t bash
elif [ -n "$HOST_DIR" ]; then
    PROJ_DIR="$HOST_DIR" docker compose run --rm --service-ports l4t bash
elif [ -n "$DATA_DIR" ]; then
    DATA_DIR="$DATA_DIR" docker compose run --rm --service-ports l4t bash
else
    echo "Using default volume mount from docker-compose.yaml"
    docker compose run --rm --service-ports l4t bash
fi

