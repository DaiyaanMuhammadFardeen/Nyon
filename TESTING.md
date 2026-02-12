# Nyon Engine Testing Guide

## Overview

This document provides comprehensive information about the Nyon Engine testing framework, including setup instructions, usage guidelines, and best practices for writing and maintaining tests.

## Table of Contents

1. [Getting Started](#getting-started)
2. [Test Structure](#test-structure)
3. [Writing Tests](#writing-tests)
4. [Running Tests](#running-tests)
5. [Debugging Tests](#debugging-tests)
6. [Continuous Integration](#continuous-integration)
7. [Best Practices](#best-practices)
8. [Troubleshooting](#troubleshooting)

## Getting Started

### Prerequisites

Before running tests, ensure you have the following installed:

- CMake 3.14 or higher
- C++ compiler (GCC 7+ or Clang 5+)
- GLFW3 development libraries
- GLM mathematics library

### Quick Start

```bash
# Clone the repository
git clone <repository-url>
cd Nyon

# Run all tests
./run_tests.sh run

# Run specific test category
./run_tests.sh math
./run_tests.sh physics
./run_tests.sh ecs
```

## Test Structure

The testing framework is organized as follows:

```
test/
├── CMakeLists.txt          # Test build configuration
├── main.cpp               # Test runner entry point
├── include/               # Test utilities and helpers
│   ├── TestLogger.h       # Advanced logging system
│   └── TestHelpers.h      # Custom assertions and helpers
└── unit/                  # Individual unit test files
    ├── MathVector2Test.cpp
    ├── MathVector3Test.cpp
    ├── PhysicsGravityTest.cpp
    ├── PhysicsMovementTest.cpp
    ├── ECSManagerTest.cpp
    ├── ECSComponentStoreTest.cpp
    ├── ECSSystemManagerTest.cpp
    ├── ECSComponentTest.cpp
    └── ECSSystemTest.cpp
```

## Writing Tests

### Test File Structure

Each test file should follow this pattern:

```cpp
#include <gtest/gtest.h>
#include "relevant_headers.h"
#include "TestHelpers.h"

using namespace Nyon::YourNamespace;

/**
 * @brief Brief description of what these tests cover.
 */
class YourTestClass : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        // Initialize test fixtures
        LOG_FUNC_EXIT();
    }

    // Test data and helper methods
    YourClassUnderTest testObject;
};

// Individual test cases
TEST_F(YourTestClass, TestName_Description)
{
    LOG_FUNC_ENTER();
    
    // Arrange - Set up test data
    auto testData = CreateTestData();
    LOG_VAR_DEBUG(testData);
    
    // Act - Execute the code under test
    auto result = testObject.PerformOperation(testData);
    
    // Assert - Verify results
    EXPECT_EQ(result, expectedValue);
    LOG_VAR_DEBUG(result);
    
    LOG_FUNC_EXIT();
}
```

### Available Assertions

The framework provides standard Google Test assertions plus custom helpers:

```cpp
// Standard assertions
EXPECT_EQ(actual, expected);
ASSERT_TRUE(condition);
EXPECT_NEAR(value1, value2, epsilon);

// Custom math assertions
EXPECT_VECTOR2_NEAR(vec1, vec2, epsilon);
EXPECT_VECTOR3_NEAR(vec1, vec2, epsilon);
ASSERT_FLOAT_NEAR(value1, value2, epsilon);

// Custom logging
LOG_DEBUG("Debug message");
LOG_INFO("Info message");
LOG_VAR_DEBUG(variable);  // Logs variable name and value
```

### Test Categories

#### 1. Math Tests (`MathVector2Test.cpp`, `MathVector3Test.cpp`)
- Vector arithmetic operations
- Length and normalization
- Edge cases and boundary conditions
- Performance benchmarks

#### 2. Physics Tests (`PhysicsGravityTest.cpp`, `PhysicsMovementTest.cpp`)
- Gravity simulation accuracy
- Force and impulse application
- Velocity limiting and constraints
- Sub-stepping behavior

#### 3. ECS Core Tests (`ECSManagerTest.cpp`, `ECSComponentStoreTest.cpp`, `ECSSystemManagerTest.cpp`)
- Entity lifecycle management
- Component storage and retrieval
- System update order and integration
- Memory management

#### 4. Component Tests (`ECSComponentTest.cpp`)
- Transform component operations
- Physics body properties
- Collider shape handling
- Render component properties
- Behavior component callbacks

#### 5. System Tests (`ECSSystemTest.cpp`)
- Physics system integration
- Input system behavior processing
- Render system component handling
- System performance metrics

## Running Tests

### Command Line Interface

The `run_tests.sh` script provides a unified interface:

```bash
# Build and run all tests
./run_tests.sh run

# Run specific categories
./run_tests.sh math      # Math tests only
./run_tests.sh physics   # Physics tests only
./run_tests.sh ecs       # ECS core tests only
./run_tests.sh component # Component tests only
./run_tests.sh system    # System tests only

# Performance testing
./run_tests.sh perf

# Build only (without running)
./run_tests.sh build

# Clean build directory
./run_tests.sh clean

# Verbose output
./run_tests.sh run -v

# Filter specific tests
./run_tests.sh run -f "*Vector2*"
./run_tests.sh run -f "Physics.Gravity*"
```

### Manual CMake Build

```bash
mkdir build && cd build
cmake .. -DENABLE_TESTING=ON
make -j nyon_tests
./test/nyon_tests
```

### Test Output Options

```bash
# Standard output
./test/nyon_tests

# Verbose with timing
./test/nyon_tests --gtest_print_time=1 --gtest_verbose=1

# Filter specific tests
./test/nyon_tests --gtest_filter="Math.Vector2*"

# XML output for CI
./test/nyon_tests --gtest_output=xml:test_results.xml

# Repeat tests
./test/nyon_tests --gtest_repeat=10
```

## Debugging Tests

### Built-in Logging System

The framework includes an advanced logging system that automatically captures variable values:

```cpp
// Automatic function entry/exit logging
LOG_FUNC_ENTER();
LOG_FUNC_EXIT();

// Variable value capture
int value = 42;
LOG_VAR_DEBUG(value);  // Outputs: "Variable 'value' = 42"

// Custom messages with levels
LOG_DEBUG("Processing started");
LOG_INFO("Entity created successfully");
LOG_WARN("Performance threshold exceeded");
LOG_ERROR("Critical failure occurred");
```

### Debugging Workflow

1. **Enable verbose logging**: Run tests with `-v` flag
2. **Filter problematic tests**: Use `--gtest_filter` to isolate issues
3. **Check log output**: Review automatic variable captures
4. **Add custom logging**: Insert `LOG_VAR_DEBUG()` for specific values
5. **Use debugger**: Set breakpoints in failing test cases

### Common Debugging Scenarios

```cpp
// Debugging floating-point precision issues
TEST_F(MathTest, PrecisionDebugging)
{
    float result = SomeCalculation();
    LOG_VAR_DEBUG(result);  // See exact value
    
    // Use epsilon comparison instead of exact equality
    EXPECT_FLOAT_NEAR(result, expected, 1e-6f);
}

// Debugging ECS component issues
TEST_F(ECSTest, ComponentDebugging)
{
    auto& component = componentStore.GetComponent<TransformComponent>(entity);
    LOG_VAR_DEBUG(component.position.x);
    LOG_VAR_DEBUG(component.position.y);
    
    // Verify component state
    EXPECT_VECTOR2_NEAR(component.position, expectedPosition, 1e-6f);
}
```

## Continuous Integration

### GitHub Actions Pipeline

The `.github/workflows/ci.yml` file defines automated testing:

- **Multi-platform testing**: Linux and macOS builds
- **Multiple compilers**: GCC and Clang support
- **Build configurations**: Debug and Release variants
- **Code coverage**: Automated coverage reports
- **Performance benchmarks**: Regular performance monitoring
- **Static analysis**: Cppcheck integration
- **Documentation generation**: Doxygen integration

### Local CI Simulation

```bash
# Run the same checks locally
./run_tests.sh run          # Unit tests
./run_tests.sh perf         # Performance tests
cppcheck --enable=all engine/  # Static analysis
doxygen Doxyfile           # Documentation
```

## Best Practices

### Test Design Principles

1. **AAA Pattern**: Arrange-Act-Assert for clear test structure
2. **Single Responsibility**: One assertion per test when possible
3. **Descriptive Names**: Test names should describe expected behavior
4. **Independence**: Tests should not depend on execution order
5. **Fast Execution**: Keep tests lightweight and focused

### Naming Conventions

```cpp
// Good test names
TEST_F(Vector2Test, Addition_Properties_Commutes)
TEST_F(PhysicsTest, Gravity_FreeFall_CalculatesCorrectly)
TEST_F(ECSTest, EntityManager_CreateEntity_ReturnsValidID)

// Poor test names
TEST_F(MyTest, Test1)
TEST_F(VectorTest, AdditionWorks)
```

### Data Management

```cpp
class PhysicsTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Create fresh test data for each test
        testBody = CreateDefaultPhysicsBody();
    }
    
    void TearDown() override
    {
        // Clean up if needed
    }
    
private:
    Physics::Body CreateDefaultPhysicsBody()
    {
        Physics::Body body;
        body.mass = 1.0f;
        body.position = Vector2(0, 0);
        return body;
    }
    
    Physics::Body testBody;
};
```

### Performance Considerations

```cpp
// Use performance timers for benchmarking
TEST_F(PerformanceTest, VectorOperations_Speed)
{
    NyonTest::PERF_TIMER("VectorOperations_Speed");
    
    const int iterations = 1000000;
    Vector2 result(0, 0);
    
    for (int i = 0; i < iterations; ++i) {
        result += Vector2(i, i * 2);
    }
    
    // Timer automatically logs execution time
}
```

## Troubleshooting

### Common Issues

**Issue**: Tests fail to compile
**Solution**: Check that all required headers are included and dependencies are linked

**Issue**: Tests crash during execution
**Solution**: Use `LOG_FUNC_ENTER()/EXIT()` to trace execution flow

**Issue**: Floating-point comparison failures
**Solution**: Use `EXPECT_FLOAT_NEAR()` with appropriate epsilon values

**Issue**: ECS tests failing randomly
**Solution**: Ensure proper entity cleanup in `TearDown()` methods

### Debug Checklist

- [ ] All required dependencies installed
- [ ] CMake configured with `-DENABLE_TESTING=ON`
- [ ] Test executables built successfully
- [ ] No memory leaks or undefined behavior
- [ ] Proper cleanup in test teardown
- [ ] Appropriate epsilon values for floating-point comparisons

### Getting Help

For issues not covered in this guide:

1. Check existing test implementations for examples
2. Review the continuous integration logs
3. Run tests with verbose output (`-v` flag)
4. Add debug logging to isolate the problem
5. Consult the Google Test documentation

---

*Last updated: February 2026*
*For questions or improvements, please open an issue in the repository.*