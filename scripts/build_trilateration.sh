#!/bin/bash
# Build script for trilateration native library
# Supports both native compilation and cross-compilation for ARM64

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Trilateration Native Library Build Script ===${NC}"

# Determine target architecture
if [ "$1" == "arm64" ] || [ "$1" == "aarch64" ]; then
    TARGET_ARCH="arm64"
    CC="aarch64-linux-gnu-gcc"
    echo -e "${YELLOW}Cross-compiling for ARM64 (Raspberry Pi)${NC}"
elif [ "$1" == "x86_64" ] || [ "$1" == "amd64" ] || [ -z "$1" ]; then
    TARGET_ARCH="x86_64"
    CC="gcc"
    echo -e "${YELLOW}Compiling for native x86_64${NC}"
else
    echo -e "${RED}Unknown architecture: $1${NC}"
    echo "Usage: $0 [arm64|x86_64]"
    exit 1
fi

# Check if compiler exists
if ! command -v $CC &> /dev/null; then
    echo -e "${RED}Compiler '$CC' not found.${NC}"
    if [ "$TARGET_ARCH" == "arm64" ]; then
        echo "For cross-compilation, install: sudo apt-get install gcc-aarch64-linux-gnu"
    fi
    exit 1
fi

# Create lib directory
mkdir -p lib

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Build shared library
echo -e "${GREEN}Compiling trilateration.c...${NC}"

$CC -shared -fPIC \
    -O2 \
    -Wall \
    -Wextra \
    -o "$PROJECT_ROOT/lib/libtrilateration_${TARGET_ARCH}.so" \
    "$PROJECT_ROOT/trilateration.c" \
    -lm

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Build successful!${NC}"
    echo -e "${GREEN}Output: lib/libtrilateration_${TARGET_ARCH}.so${NC}"
    
    # Show file info
    ls -lh "$PROJECT_ROOT/lib/libtrilateration_${TARGET_ARCH}.so"
    file "$PROJECT_ROOT/lib/libtrilateration_${TARGET_ARCH}.so"
else
    echo -e "${RED}✗ Build failed!${NC}"
    exit 1
fi

# Optional: Create symlink for current architecture
if [ "$TARGET_ARCH" == "$(uname -m)" ]; then
    ln -sf "libtrilateration_${TARGET_ARCH}.so" "$PROJECT_ROOT/lib/libtrilateration.so"
    echo -e "${GREEN}Created symlink: lib/libtrilateration.so -> libtrilateration_${TARGET_ARCH}.so${NC}"
fi

echo -e "${GREEN}=== Build complete ===${NC}"
