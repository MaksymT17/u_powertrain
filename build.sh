#!/bin/bash
# ==============================================================================
#  Alpha EV Powertrain Simulation - Automated Build Script
# ==============================================================================
set -e

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0;31m' # No Color
# Reset code
RESET='\033[0m'

echo -e "${BLUE}======================================================================${RESET}"
echo -e "${BLUE}  Alpha EV Powertrain Simulation Build Orchestrator${RESET}"
echo -e "${BLUE}======================================================================${RESET}"

# Get directory where script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# 1. Dependency Validation Checks
echo -e "${CYAN}[1/3] Validating build toolchain and dependencies...${RESET}"

# Check for C++ compiler
if ! command -v clang++ &> /dev/null && ! command -v g++ &> /dev/null; then
    echo -e "${RED}[ERROR] No C++ compiler found (clang++ or g++).${RESET}"
    echo -e "${YELLOW}Recommendation: Please install Xcode Command Line Tools by running: xcode-select --install${RESET}"
    exit 1
else
    COMPILER_VERSION=$(clang++ --version 2>&1 | head -n 1)
    echo -e "  - C++ Compiler: Found (${COMPILER_VERSION})"
fi

# Check for CMake
if ! command -v cmake &> /dev/null; then
    echo -e "${RED}[ERROR] CMake is not installed.${RESET}"
    echo -e "${YELLOW}Recommendation: Please install CMake using Homebrew: brew install cmake${RESET}"
    exit 1
else
    CMAKE_VERSION=$(cmake --version | head -n 1)
    echo -e "  - Build Generator: Found (${CMAKE_VERSION})"
fi

# Check for Qt6
HAS_QT6=false
if command -v qmake &> /dev/null; then
    QMAKE_VER=$(qmake --version | grep -o "Qt version [0-9]*" | cut -d' ' -f3)
    if [ "${QMAKE_VER%%.*}" -eq 6 ] || [ "${QMAKE_VER%%.*}" -eq 6 ]; then
        HAS_QT6=true
        echo -e "  - Qt Framework: Found Qt ${QMAKE_VER} via qmake"
    fi
fi

if [ "$HAS_QT6" = false ]; then
    # Search in common Homebrew paths
    if [ -d "/opt/homebrew/opt/qt" ] || [ -d "/usr/local/opt/qt" ]; then
        echo -e "  - Qt Framework: Found Homebrew Qt directory"
        HAS_QT6=true
    fi
fi

if [ "$HAS_QT6" = false ]; then
    echo -e "${RED}[ERROR] Qt6 framework was not found.${RESET}"
    echo -e "${YELLOW}Recommendation: Install Qt6 via Homebrew: brew install qt${RESET}"
    exit 1
fi

# Check for spdlog
HAS_SPDLOG=false
if [ -d "/opt/homebrew/opt/spdlog" ] || [ -d "/usr/local/opt/spdlog" ] || brew list spdlog &>/dev/null; then
    echo -e "  - Logging Library: Found spdlog"
    HAS_SPDLOG=true
fi

if [ "$HAS_SPDLOG" = false ]; then
    echo -e "${RED}[ERROR] spdlog logging library was not found.${RESET}"
    echo -e "${YELLOW}Recommendation: Install spdlog via Homebrew: brew install spdlog${RESET}"
    exit 1
fi

echo -e "${GREEN}✓ Dependency validation complete. All requirements met.${RESET}"
echo ""

# 2. CMake Configuration
echo -e "${CYAN}[2/3] Configuring project build files with CMake...${RESET}"
mkdir -p qt/build

# Setup homebrew pathing hints for macOS build systems
export CMAKE_PREFIX_PATH="/opt/homebrew:/usr/local:${CMAKE_PREFIX_PATH}"

if ! cmake -S qt -B qt/build; then
    echo -e "${RED}[ERROR] CMake configuration failed.${RESET}"
    echo -e "${YELLOW}Troubleshooting: Clean the build directory (rm -rf qt/build) and run this script again.${RESET}"
    exit 1
fi
echo -e "${GREEN}✓ CMake build files generated successfully.${RESET}"
echo ""

# 3. Compiling Binary
echo -e "${CYAN}[3/3] Compiling EV Simulator executable...${RESET}"
CPU_CORES=$(sysctl -n hw.ncpu 2>/dev/null || echo "4")

if ! cmake --build qt/build --parallel "$CPU_CORES"; then
    echo -e "${RED}[ERROR] Compilation failed. Please check build logs above for errors.${RESET}"
    exit 1
fi

echo -e "${GREEN}✓ Compilation completed successfully!${RESET}"
echo ""
echo -e "${BLUE}======================================================================${RESET}"
echo -e "${GREEN}  SUCCESS: Build Complete${RESET}"
echo -e "${BLUE}======================================================================${RESET}"
echo -e "To run the C++/Qt EV Powertrain simulation dashboard, execute:"
echo -e "  ${CYAN}./qt/build/EVSimulationQt${RESET}"
echo -e "${BLUE}======================================================================${RESET}"
