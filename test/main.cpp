#include <gtest/gtest.h>
#include "TestLogger.h"

/**
 * @brief Main test entry point with custom test environment setup.
 * 
 * Initializes the testing environment and runs all registered tests.
 * Includes custom listeners for enhanced test reporting.
 */
class NyonTestEnvironment : public ::testing::Environment
{
public:
    void SetUp() override
    {
        LOG_INFO("=== Starting Nyon Engine Test Suite ===");
        TestLogger::GetInstance().ClearLogHistory();
    }

    void TearDown() override
    {
        LOG_INFO("=== Test Suite Completed ===");
        // Print summary of any errors from log history
        const auto& logs = TestLogger::GetInstance().GetLogHistory();
        int errorCount = 0;
        for (const auto& log : logs) {
            if (log.find("[ERROR]") != std::string::npos || 
                log.find("[FATAL]") != std::string::npos) {
                errorCount++;
            }
        }
        
        if (errorCount > 0) {
            LOG_WARN("Test suite completed with " + std::to_string(errorCount) + " errors logged");
        }
    }
};

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    
    // Add custom test environment
    ::testing::AddGlobalTestEnvironment(new NyonTestEnvironment);
    
    // Run tests
    int result = RUN_ALL_TESTS();
    
    return result;
}