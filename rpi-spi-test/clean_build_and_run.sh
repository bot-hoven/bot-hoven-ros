#!/bin/bash
# clean_and_build.sh
# This script cleans the project, rebuilds it, and runs the executable
# Execute from project root directory

set -e

# Configuration
BUILD_DIR="build"
TARGET_EXEC="spi_console"
CMAKE_OPTS="-DCMAKE_BUILD_TYPE=Debug"

echo "=== Starting build process ==="

# Clean existing build
if [ -d "${BUILD_DIR}" ]; then
    echo "Removing existing build directory..."
    rm -rf "${BUILD_DIR}"
fi

# Create build directory
echo "Creating new build directory..."
mkdir -p "${BUILD_DIR}"

# Configure project
echo "Configuring project with CMake..."
cmake -B "${BUILD_DIR}" -S . ${CMAKE_OPTS}

# Build project
echo "Building target ${TARGET_EXEC}..."
cmake --build "${BUILD_DIR}" --target ${TARGET_EXEC} -j$(nproc)

# Verify executable
if [ ! -f "${BUILD_DIR}/${TARGET_EXEC}" ]; then
    echo "Error: Executable ${TARGET_EXEC} not found!"
    exit 1
fi

# Run executable with privileges
echo "Running program..."
sudo "${BUILD_DIR}/${TARGET_EXEC}"

echo "=== Execution complete ==="