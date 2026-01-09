// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

#include <gtest/gtest.h>
#include "joltc.h"

class CoreTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

// ============================================================================
// Init/Shutdown Tests
// ============================================================================

TEST_F(CoreTest, Init_Shutdown) {
    EXPECT_TRUE(JPH_Init());
    JPH_Shutdown();
}

TEST_F(CoreTest, Init_Multiple_Times) {
    // First init
    EXPECT_TRUE(JPH_Init());
    JPH_Shutdown();

    // Second init should also work
    EXPECT_TRUE(JPH_Init());
    JPH_Shutdown();
}

// ============================================================================
// JobSystem Tests
// ============================================================================

class JobSystemTest : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_TRUE(JPH_Init());
    }
    void TearDown() override {
        JPH_Shutdown();
    }
};

TEST_F(JobSystemTest, ThreadPool_Create_Default) {
    JPH_JobSystem* jobSystem = JPH_JobSystemThreadPool_Create(nullptr);
    ASSERT_NE(jobSystem, nullptr);
    JPH_JobSystem_Destroy(jobSystem);
}

TEST_F(JobSystemTest, ThreadPool_Create_WithConfig) {
    JobSystemThreadPoolConfig config = {};
    config.maxJobs = JPH_MAX_PHYSICS_JOBS;
    config.maxBarriers = JPH_MAX_PHYSICS_BARRIERS;
    config.numThreads = 2;

    JPH_JobSystem* jobSystem = JPH_JobSystemThreadPool_Create(&config);
    ASSERT_NE(jobSystem, nullptr);
    JPH_JobSystem_Destroy(jobSystem);
}

// ============================================================================
// TraceHandler Tests
// ============================================================================

static bool g_traceHandlerCalled = false;
static void TestTraceHandler(const char* message) {
    g_traceHandlerCalled = true;
}

TEST_F(JobSystemTest, SetTraceHandler) {
    g_traceHandlerCalled = false;
    JPH_SetTraceHandler(TestTraceHandler);
    // Note: We can't easily trigger a trace message, but at least we verify the function doesn't crash
    JPH_SetTraceHandler(nullptr);
}

// ============================================================================
// AssertFailureHandler Tests
// ============================================================================

static bool g_assertHandlerCalled = false;
static bool TestAssertHandler(const char* expression, const char* message, const char* file, uint32_t line) {
    g_assertHandlerCalled = true;
    return true; // Continue execution
}

TEST_F(JobSystemTest, SetAssertFailureHandler) {
    g_assertHandlerCalled = false;
    JPH_SetAssertFailureHandler(TestAssertHandler);
    // Note: We can't easily trigger an assert, but at least we verify the function doesn't crash
    JPH_SetAssertFailureHandler(nullptr);
}
