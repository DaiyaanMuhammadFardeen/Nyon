#!/bin/bash

# Nyon Engine Test Runner Script
# This script provides a unified interface for running all engine tests

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
BUILD_DIR="build"
TEST_DIR="test"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}   Nyon Engine Test Runner     ${NC}"
echo -e "${BLUE}================================${NC}"

# Function to print colored output
print_status() {
    echo -e "${GREEN}[STATUS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check prerequisites
check_prerequisites() {
    print_status "Checking prerequisites..."
    
    # Check if cmake is installed
    if ! command -v cmake &> /dev/null; then
        print_error "CMake is not installed. Please install CMake 3.14 or higher."
        exit 1
    fi
    
    # Check if compiler is available
    if ! command -v g++ &> /dev/null && ! command -v clang++ &> /dev/null; then
        print_error "No C++ compiler found. Please install g++ or clang++."
        exit 1
    fi
    
    # Check if required libraries are available
    if ! pkg-config --exists glfw3; then
        print_warning "GLFW3 not found via pkg-config. Build might fail."
    fi
    
    if ! pkg-config --exists glm; then
        print_warning "GLM not found via pkg-config. Build might fail."
    fi
    
    print_status "Prerequisites check completed."
}

# Function to build tests
build_tests() {
    print_status "Building tests..."
    
    cd "$PROJECT_ROOT"
    
    # Create build directory if it doesn't exist
    if [ ! -d "$BUILD_DIR" ]; then
        mkdir "$BUILD_DIR"
    fi
    
    cd "$BUILD_DIR"
    
    # Configure with testing enabled
    cmake .. -DENABLE_TESTING=ON
    
    # Build tests
    make -j$(nproc) nyon_tests
    
    print_status "Build completed successfully."
}

# Function to run tests
run_tests() {
    local test_filter="$1"
    local verbose="$2"
    
    print_status "Running tests..."
    
    cd "$PROJECT_ROOT/$BUILD_DIR"
    
    if [ "$verbose" = "true" ]; then
        # Verbose output with detailed information
        ./test/nyon_tests --gtest_print_time=1 --gtest_verbose=1 ${test_filter:+--gtest_filter=$test_filter}
    else
        # Normal output
        ./test/nyon_tests ${test_filter:+--gtest_filter=$test_filter}
    fi
    
    print_status "Tests completed."
}

# Function to run specific test categories
run_category_tests() {
    local category="$1"
    
    case "$category" in
        "math")
            print_status "Running math tests..."
            run_tests "*Math*" false
            ;;
        "physics")
            print_status "Running physics tests..."
            run_tests "*Physics*" false
            ;;
        "ecs")
            print_status "Running ECS tests..."
            run_tests "*ECS*" false
            ;;
        "component")
            print_status "Running component tests..."
            run_tests "*Component*" false
            ;;
        "system")
            print_status "Running system tests..."
            run_tests "*System*" false
            ;;
        "all")
            print_status "Running all tests..."
            run_tests "" false
            ;;
        *)
            print_error "Unknown test category: $category"
            echo "Available categories: math, physics, ecs, component, system, all"
            exit 1
            ;;
    esac
}

# Function to run performance tests
run_performance_tests() {
    print_status "Running performance tests..."
    run_tests "*Performance*" true
}

# Function to clean build
clean_build() {
    print_status "Cleaning build directory..."
    cd "$PROJECT_ROOT"
    
    if [ -d "$BUILD_DIR" ]; then
        rm -rf "$BUILD_DIR"
    fi
    
    print_status "Clean completed."
}

# Function to show help
show_help() {
    echo "Usage: $0 [OPTIONS] [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  build          Build the test executable"
    echo "  run            Build and run all tests"
    echo "  math           Run math-related tests only"
    echo "  physics        Run physics-related tests only"
    echo "  ecs            Run ECS core tests only"
    echo "  component      Run component tests only"
    echo "  system         Run system tests only"
    echo "  perf           Run performance tests"
    echo "  clean          Clean build directory"
    echo "  help           Show this help message"
    echo ""
    echo "Options:"
    echo "  -v, --verbose  Enable verbose output"
    echo "  -f, --filter   Filter tests by pattern (e.g., '*Vector2*')"
    echo ""
    echo "Examples:"
    echo "  $0 run                    # Build and run all tests"
    echo "  $0 math                   # Run only math tests"
    echo "  $0 run -f '*Vector2*'     # Run tests matching Vector2"
    echo "  $0 run -v                 # Run all tests with verbose output"
}

# Parse command line arguments
VERBOSE=false
FILTER=""
COMMAND=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -f|--filter)
            FILTER="$2"
            shift 2
            ;;
        build|run|math|physics|ecs|component|system|perf|clean|help)
            COMMAND="$1"
            shift
            ;;
        *)
            print_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Execute based on command
case "$COMMAND" in
    build)
        check_prerequisites
        build_tests
        ;;
    run)
        check_prerequisites
        build_tests
        if [ -n "$FILTER" ]; then
            run_tests "$FILTER" "$VERBOSE"
        else
            run_tests "" "$VERBOSE"
        fi
        ;;
    math|physics|ecs|component|system)
        check_prerequisites
        build_tests
        run_category_tests "$COMMAND"
        ;;
    perf)
        check_prerequisites
        build_tests
        run_performance_tests
        ;;
    clean)
        clean_build
        ;;
    help|"")
        show_help
        ;;
    *)
        print_error "Unknown command: $COMMAND"
        show_help
        exit 1
        ;;
esac

print_status "Test runner completed successfully!"
exit 0