// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

#include <gtest/gtest.h>
#include "joltc.h"

namespace BroadPhaseLayers {
    static constexpr JPH_BroadPhaseLayer NON_MOVING = 0;
    static constexpr JPH_BroadPhaseLayer MOVING = 1;
    static constexpr uint32_t NUM_LAYERS = 2;
}

namespace ObjectLayers {
    static constexpr JPH_ObjectLayer NON_MOVING = 0;
    static constexpr JPH_ObjectLayer MOVING = 1;
    static constexpr uint32_t NUM_LAYERS = 2;
}

class VehicleTest : public ::testing::Test {
protected:
    JPH_PhysicsSystem* physicsSystem = nullptr;
    JPH_BodyInterface* bodyInterface = nullptr;
    JPH_BroadPhaseLayerInterface* bpLayer = nullptr;
    JPH_ObjectLayerPairFilter* objPairFilter = nullptr;
    JPH_ObjectVsBroadPhaseLayerFilter* objVsBpFilter = nullptr;

    void SetUp() override {
        ASSERT_TRUE(JPH_Init());

        bpLayer = JPH_BroadPhaseLayerInterfaceTable_Create(ObjectLayers::NUM_LAYERS, BroadPhaseLayers::NUM_LAYERS);
        JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(bpLayer, ObjectLayers::NON_MOVING, BroadPhaseLayers::NON_MOVING);
        JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(bpLayer, ObjectLayers::MOVING, BroadPhaseLayers::MOVING);

        objPairFilter = JPH_ObjectLayerPairFilterTable_Create(ObjectLayers::NUM_LAYERS);
        JPH_ObjectLayerPairFilterTable_EnableCollision(objPairFilter, ObjectLayers::NON_MOVING, ObjectLayers::MOVING);
        JPH_ObjectLayerPairFilterTable_EnableCollision(objPairFilter, ObjectLayers::MOVING, ObjectLayers::MOVING);

        objVsBpFilter = JPH_ObjectVsBroadPhaseLayerFilterTable_Create(
            bpLayer, BroadPhaseLayers::NUM_LAYERS,
            objPairFilter, ObjectLayers::NUM_LAYERS);

        JPH_PhysicsSystemSettings settings = {};
        settings.maxBodies = 1024;
        settings.maxBodyPairs = 1024;
        settings.maxContactConstraints = 1024;
        settings.broadPhaseLayerInterface = bpLayer;
        settings.objectVsBroadPhaseLayerFilter = objVsBpFilter;
        settings.objectLayerPairFilter = objPairFilter;

        physicsSystem = JPH_PhysicsSystem_Create(&settings);
        bodyInterface = JPH_PhysicsSystem_GetBodyInterface(physicsSystem);
    }

    void TearDown() override {
        if (physicsSystem) JPH_PhysicsSystem_Destroy(physicsSystem);
        JPH_Shutdown();
    }
};

// ============================================================================
// WheelSettings Tests
// ============================================================================

TEST_F(VehicleTest, WheelSettings_Create) {
    JPH_WheelSettings* settings = JPH_WheelSettings_Create();
    ASSERT_NE(settings, nullptr);

    JPH_WheelSettings_Destroy(settings);
}

TEST_F(VehicleTest, WheelSettings_Position) {
    JPH_WheelSettings* settings = JPH_WheelSettings_Create();

    JPH_Vec3 pos = {1.0f, 0.0f, 2.0f};
    JPH_WheelSettings_SetPosition(settings, &pos);

    JPH_Vec3 result;
    JPH_WheelSettings_GetPosition(settings, &result);
    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.z, 2.0f);

    JPH_WheelSettings_Destroy(settings);
}

TEST_F(VehicleTest, WheelSettings_Radius) {
    JPH_WheelSettings* settings = JPH_WheelSettings_Create();

    JPH_WheelSettings_SetRadius(settings, 0.5f);
    EXPECT_FLOAT_EQ(JPH_WheelSettings_GetRadius(settings), 0.5f);

    JPH_WheelSettings_Destroy(settings);
}

TEST_F(VehicleTest, WheelSettings_Width) {
    JPH_WheelSettings* settings = JPH_WheelSettings_Create();

    JPH_WheelSettings_SetWidth(settings, 0.3f);
    EXPECT_FLOAT_EQ(JPH_WheelSettings_GetWidth(settings), 0.3f);

    JPH_WheelSettings_Destroy(settings);
}

TEST_F(VehicleTest, WheelSettings_Suspension) {
    JPH_WheelSettings* settings = JPH_WheelSettings_Create();

    JPH_WheelSettings_SetSuspensionMinLength(settings, 0.1f);
    JPH_WheelSettings_SetSuspensionMaxLength(settings, 0.5f);

    EXPECT_FLOAT_EQ(JPH_WheelSettings_GetSuspensionMinLength(settings), 0.1f);
    EXPECT_FLOAT_EQ(JPH_WheelSettings_GetSuspensionMaxLength(settings), 0.5f);

    JPH_WheelSettings_Destroy(settings);
}

// ============================================================================
// Wheel Tests
// ============================================================================

TEST_F(VehicleTest, Wheel_Create) {
    JPH_WheelSettings* settings = JPH_WheelSettings_Create();
    JPH_Wheel* wheel = JPH_Wheel_Create(settings);
    ASSERT_NE(wheel, nullptr);

    JPH_Wheel_Destroy(wheel);
    JPH_WheelSettings_Destroy(settings);
}

TEST_F(VehicleTest, Wheel_AngularVelocity) {
    JPH_WheelSettings* settings = JPH_WheelSettings_Create();
    JPH_Wheel* wheel = JPH_Wheel_Create(settings);

    JPH_Wheel_SetAngularVelocity(wheel, 10.0f);
    EXPECT_FLOAT_EQ(JPH_Wheel_GetAngularVelocity(wheel), 10.0f);

    JPH_Wheel_Destroy(wheel);
    JPH_WheelSettings_Destroy(settings);
}

TEST_F(VehicleTest, Wheel_RotationAngle) {
    JPH_WheelSettings* settings = JPH_WheelSettings_Create();
    JPH_Wheel* wheel = JPH_Wheel_Create(settings);

    JPH_Wheel_SetRotationAngle(wheel, JPH_M_PI / 4.0f);
    EXPECT_NEAR(JPH_Wheel_GetRotationAngle(wheel), JPH_M_PI / 4.0f, 0.001f);

    JPH_Wheel_Destroy(wheel);
    JPH_WheelSettings_Destroy(settings);
}

TEST_F(VehicleTest, Wheel_SteerAngle) {
    JPH_WheelSettings* settings = JPH_WheelSettings_Create();
    JPH_Wheel* wheel = JPH_Wheel_Create(settings);

    JPH_Wheel_SetSteerAngle(wheel, 0.1f);
    EXPECT_FLOAT_EQ(JPH_Wheel_GetSteerAngle(wheel), 0.1f);

    JPH_Wheel_Destroy(wheel);
    JPH_WheelSettings_Destroy(settings);
}

// ============================================================================
// VehicleCollisionTester Tests
// ============================================================================

TEST_F(VehicleTest, VehicleCollisionTesterRay_Create) {
    JPH_Vec3 up = {0.0f, 1.0f, 0.0f};
    JPH_VehicleCollisionTesterRay* tester = JPH_VehicleCollisionTesterRay_Create(ObjectLayers::MOVING, &up, JPH_M_PI / 4.0f);
    ASSERT_NE(tester, nullptr);

    EXPECT_EQ(JPH_VehicleCollisionTester_GetObjectLayer((JPH_VehicleCollisionTester*)tester), ObjectLayers::MOVING);

    JPH_VehicleCollisionTester_Destroy((JPH_VehicleCollisionTester*)tester);
}

TEST_F(VehicleTest, VehicleCollisionTesterCastSphere_Create) {
    JPH_Vec3 up = {0.0f, 1.0f, 0.0f};
    JPH_VehicleCollisionTesterCastSphere* tester = JPH_VehicleCollisionTesterCastSphere_Create(ObjectLayers::MOVING, 0.1f, &up, JPH_M_PI / 4.0f);
    ASSERT_NE(tester, nullptr);

    JPH_VehicleCollisionTester_Destroy((JPH_VehicleCollisionTester*)tester);
}

TEST_F(VehicleTest, VehicleCollisionTesterCastCylinder_Create) {
    JPH_VehicleCollisionTesterCastCylinder* tester = JPH_VehicleCollisionTesterCastCylinder_Create(ObjectLayers::MOVING, 0.05f);
    ASSERT_NE(tester, nullptr);

    JPH_VehicleCollisionTester_Destroy((JPH_VehicleCollisionTester*)tester);
}

// ============================================================================
// VehicleEngineSettings Tests
// ============================================================================

TEST_F(VehicleTest, VehicleEngineSettings_Init) {
    JPH_VehicleEngineSettings settings;
    JPH_VehicleEngineSettings_Init(&settings);

    EXPECT_GT(settings.maxRPM, 0.0f);
    EXPECT_GT(settings.maxTorque, 0.0f);
}

// ============================================================================
// VehicleDifferentialSettings Tests
// ============================================================================

TEST_F(VehicleTest, VehicleDifferentialSettings_Init) {
    JPH_VehicleDifferentialSettings settings;
    JPH_VehicleDifferentialSettings_Init(&settings);

    // Default values should be set
    EXPECT_GE(settings.leftWheel, -1);
}

// ============================================================================
// VehicleTransmissionSettings Tests
// ============================================================================

TEST_F(VehicleTest, VehicleTransmissionSettings_Create) {
    JPH_VehicleTransmissionSettings* settings = JPH_VehicleTransmissionSettings_Create();
    ASSERT_NE(settings, nullptr);

    JPH_VehicleTransmissionSettings_Destroy(settings);
}

TEST_F(VehicleTest, VehicleTransmissionSettings_Mode) {
    JPH_VehicleTransmissionSettings* settings = JPH_VehicleTransmissionSettings_Create();

    JPH_VehicleTransmissionSettings_SetMode(settings, JPH_TransmissionMode_Manual);
    EXPECT_EQ(JPH_VehicleTransmissionSettings_GetMode(settings), JPH_TransmissionMode_Manual);

    JPH_VehicleTransmissionSettings_SetMode(settings, JPH_TransmissionMode_Auto);
    EXPECT_EQ(JPH_VehicleTransmissionSettings_GetMode(settings), JPH_TransmissionMode_Auto);

    JPH_VehicleTransmissionSettings_Destroy(settings);
}

TEST_F(VehicleTest, VehicleTransmissionSettings_GearRatios) {
    JPH_VehicleTransmissionSettings* settings = JPH_VehicleTransmissionSettings_Create();

    float gears[] = {3.0f, 2.0f, 1.5f, 1.0f, 0.8f};
    JPH_VehicleTransmissionSettings_SetGearRatios(settings, gears, 5);

    EXPECT_EQ(JPH_VehicleTransmissionSettings_GetGearRatioCount(settings), 5u);
    EXPECT_FLOAT_EQ(JPH_VehicleTransmissionSettings_GetGearRatio(settings, 0), 3.0f);
    EXPECT_FLOAT_EQ(JPH_VehicleTransmissionSettings_GetGearRatio(settings, 4), 0.8f);

    JPH_VehicleTransmissionSettings_Destroy(settings);
}

TEST_F(VehicleTest, VehicleTransmissionSettings_Timing) {
    JPH_VehicleTransmissionSettings* settings = JPH_VehicleTransmissionSettings_Create();

    JPH_VehicleTransmissionSettings_SetSwitchTime(settings, 0.5f);
    EXPECT_FLOAT_EQ(JPH_VehicleTransmissionSettings_GetSwitchTime(settings), 0.5f);

    JPH_VehicleTransmissionSettings_SetClutchReleaseTime(settings, 0.3f);
    EXPECT_FLOAT_EQ(JPH_VehicleTransmissionSettings_GetClutchReleaseTime(settings), 0.3f);

    JPH_VehicleTransmissionSettings_SetShiftUpRPM(settings, 5000.0f);
    EXPECT_FLOAT_EQ(JPH_VehicleTransmissionSettings_GetShiftUpRPM(settings), 5000.0f);

    JPH_VehicleTransmissionSettings_SetShiftDownRPM(settings, 2000.0f);
    EXPECT_FLOAT_EQ(JPH_VehicleTransmissionSettings_GetShiftDownRPM(settings), 2000.0f);

    JPH_VehicleTransmissionSettings_Destroy(settings);
}

// ============================================================================
// WheeledVehicleControllerSettings Tests
// ============================================================================

TEST_F(VehicleTest, WheeledVehicleControllerSettings_Create) {
    JPH_WheeledVehicleControllerSettings* settings = JPH_WheeledVehicleControllerSettings_Create();
    ASSERT_NE(settings, nullptr);

    JPH_VehicleControllerSettings_Destroy((JPH_VehicleControllerSettings*)settings);
}

TEST_F(VehicleTest, WheeledVehicleControllerSettings_Engine) {
    JPH_WheeledVehicleControllerSettings* settings = JPH_WheeledVehicleControllerSettings_Create();

    JPH_VehicleEngineSettings engineSettings;
    JPH_VehicleEngineSettings_Init(&engineSettings);
    engineSettings.maxRPM = 8000.0f;
    engineSettings.maxTorque = 500.0f;

    JPH_WheeledVehicleControllerSettings_SetEngine(settings, &engineSettings);

    JPH_VehicleEngineSettings result;
    JPH_WheeledVehicleControllerSettings_GetEngine(settings, &result);
    EXPECT_FLOAT_EQ(result.maxRPM, 8000.0f);
    EXPECT_FLOAT_EQ(result.maxTorque, 500.0f);

    JPH_VehicleControllerSettings_Destroy((JPH_VehicleControllerSettings*)settings);
}

// ============================================================================
// VehicleAntiRollBar Tests
// ============================================================================

TEST_F(VehicleTest, VehicleAntiRollBar_Init) {
    JPH_VehicleAntiRollBar antiRollBar;
    JPH_VehicleAntiRollBar_Init(&antiRollBar);

    // Default values should be set
    EXPECT_GE(antiRollBar.leftWheel, 0);
}

// ============================================================================
// VehicleConstraintSettings Tests
// ============================================================================

TEST_F(VehicleTest, VehicleConstraintSettings_Init) {
    JPH_VehicleConstraintSettings settings;
    JPH_VehicleConstraintSettings_Init(&settings);

    // Default forward should be set
    EXPECT_FALSE(settings.forward.x == 0.0f && settings.forward.y == 0.0f && settings.forward.z == 0.0f);
}
