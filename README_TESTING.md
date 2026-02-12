# Nyon Engine - Comprehensive Testing Framework

## Overview

This testing framework provides complete unit testing, integration testing, and debugging capabilities for the Nyon game engine. It enables comprehensive testing of all engine components without requiring OpenGL window initialization or manual debugging.

## Key Features

### 🧪 **Complete Test Coverage**
- **Math Tests**: Vector2, Vector3 operations and mathematical functions
- **Physics Tests**: Gravity simulation, movement physics, collision handling
- **ECS Tests**: Entity management, component storage, system orchestration
- **Component Tests**: Transform, PhysicsBody, Collider, Render, Behavior components
- **System Tests**: PhysicsSystem, InputSystem, RenderSystem integration

### 🔍 **Advanced Debugging System**
- **Automatic Variable Capture**: Logs variable names and values without manual instrumentation
- **Function Tracing**: Automatic entry/exit logging for all test functions
- **Performance Timing**: Built-in performance measurement for optimization
- **Detailed Error Reporting**: Comprehensive failure analysis with context

### ⚡ **Developer-Friendly Tools**
- **Unified Test Runner**: Single script for all testing operations
- **Category Filtering**: Run specific test groups (math, physics, ecs, etc.)
- **Performance Testing**: Dedicated benchmarks for performance-critical code
- **Continuous Integration**: Automated testing pipeline with GitHub Actions

### 🛠️ **Easy Integration**
- **CMake Integration**: Seamless build system integration
- **Header-Only Utilities**: Easy inclusion in existing projects
- **Modular Design**: Add new tests without affecting existing code
- **Cross-Platform**: Works on Linux, macOS, and Windows

## Quick Start

### Running All Tests
```bash
# Make the script executable
chmod +x run_tests.sh

# Run all tests with detailed output
./run_tests.sh run -v

# Run specific test categories
./run_tests.sh math      # Vector math tests
./run_tests.sh physics   # Physics simulation tests
./run_tests.sh ecs       # ECS core functionality
```

### Writing New Tests
```cpp
#include <gtest/gtest.h>
#include "TestHelpers.h"

class MyComponentTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        // Initialize test data
        LOG_FUNC_EXIT();
    }
};

TEST_F(MyComponentTest, BasicFunctionality)
{
    LOG_FUNC_ENTER();
    
    // Test your component
    auto result = myComponent.DoSomething();
    LOG_VAR_DEBUG(result);  // Automatically logs variable name and value
    
    EXPECT_EQ(result, expectedResult);
    
    LOG_FUNC_EXIT();
}
```

## Framework Components

### Test Logger (`TestLogger.h`)
Advanced logging system that captures:
- Function entry/exit points
- Variable names and values
- Timestamps and execution context
- Performance measurements
- Error traces for debugging

### Test Helpers (`TestHelpers.h`)
Utility functions including:
- Custom assertions for vectors and floats
- Performance timing wrappers
- ECS entity creation helpers
- Floating-point comparison with epsilon

### Test Categories

1. **Math Tests** (`MathVector2Test.cpp`, `MathVector3Test.cpp`)
   - Vector arithmetic operations
   - Length and normalization
   - Edge case handling
   - Performance benchmarks

2. **Physics Tests** (`PhysicsGravityTest.cpp`, `PhysicsMovementTest.cpp`)
   - Gravity simulation accuracy
   - Force and impulse application
   - Velocity limiting and constraints
   - Sub-stepping behavior validation

3. **ECS Core Tests** (`ECSManagerTest.cpp`, `ECSComponentStoreTest.cpp`, `ECSSystemManagerTest.cpp`)
   - Entity creation/destruction lifecycle
   - Component storage and retrieval
   - System update order and integration
   - Memory management verification

4. **Component Tests** (`ECSComponentTest.cpp`)
   - Transform component operations
   - Physics body property validation
   - Collider shape handling
   - Render component properties
   - Behavior component callback execution

5. **System Tests** (`ECSSystemTest.cpp`)
   - Physics system integration with components
   - Input system behavior processing
   - Render system component handling
   - System performance metrics

## Advanced Usage

### Debugging with Variable Capture
```cpp
TEST_F(PhysicsTest, DebugExample)
{
    Physics::Body body;
    body.velocity = Vector2(100.0f, 50.0f);
    
    LOG_VAR_DEBUG(body.velocity.x);  // Logs: "Variable 'body.velocity.x' = 100"
    LOG_VAR_DEBUG(body.velocity.y);  // Logs: "Variable 'body.velocity.y' = 50"
    
    UpdatePhysics(body, deltaTime);
    
    LOG_VAR_DEBUG(body.position.x);  // See exact results without debugger
    LOG_VAR_DEBUG(body.position.y);
}
```

### Performance Testing
```cpp
TEST_F(PerformanceTest, VectorBatchOperations)
{
    NyonTest::PERF_TIMER("VectorBatchOperations");
    
    std::vector<Vector2> vectors(10000);
    // ... populate vectors ...
    
    // Performance timer automatically logs execution time
    ProcessVectors(vectors);
}
```

### Filtering and Selective Testing
```bash
# Run only specific test patterns
./run_tests.sh run -f "*Vector2.Addition*"
./run_tests.sh run -f "Physics.Gravity*"

# Run performance tests only
./run_tests.sh perf

# Verbose output for detailed debugging
./run_tests.sh run -v
```

## Continuous Integration

The framework includes comprehensive CI configuration:

- **Multi-platform testing** (Linux, macOS)
- **Multiple compiler support** (GCC, Clang)
- **Code coverage analysis**
- **Performance benchmarking**
- **Static code analysis**
- **Documentation generation**

## Benefits for Development

### Without Testing Framework:
❌ Need to run OpenGL window to test physics  
❌ Manual debugging with step-through process  
❌ Difficult to reproduce intermittent bugs  
❌ No performance baseline measurements  
❌ Hard to verify edge cases  

### With Testing Framework:
✅ Test any function in isolation  
✅ Automatic variable logging for debugging  
✅ Comprehensive test coverage  
✅ Performance regression detection  
✅ Easy reproduction of edge cases  
✅ Continuous integration validation  

## Getting Started

1. **Clone the repository**
2. **Install dependencies** (CMake, GLFW3, GLM)
3. **Run initial tests**: `./run_tests.sh run`
4. **Explore existing tests** in the `test/unit/` directory
5. **Write your first test** following the examples
6. **Integrate with your development workflow**

The framework is designed to be developer-friendly while providing enterprise-grade testing capabilities. Every function, class, and system in the Nyon engine can now be thoroughly tested and debugged without the complexity of running the full application.