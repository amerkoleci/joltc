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

class ConstraintTest : public ::testing::Test {
protected:
    JPH_PhysicsSystem* physicsSystem = nullptr;
    JPH_BodyInterface* bodyInterface = nullptr;
    JPH_BroadPhaseLayerInterface* bpLayer = nullptr;
    JPH_ObjectLayerPairFilter* objPairFilter = nullptr;
    JPH_ObjectVsBroadPhaseLayerFilter* objVsBpFilter = nullptr;
    JPH_Body* body1 = nullptr;
    JPH_Body* body2 = nullptr;
    JPH_BodyID bodyId1, bodyId2;

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

        // Create two dynamic bodies for constraint testing
        JPH_SphereShape* shape = JPH_SphereShape_Create(0.5f);
        JPH_Vec3 pos1 = {0.0f, 0.0f, 0.0f};
        JPH_Vec3 pos2 = {2.0f, 0.0f, 0.0f};
        JPH_Quat rot = {0.0f, 0.0f, 0.0f, 1.0f};

        JPH_BodyCreationSettings* bs1 = JPH_BodyCreationSettings_Create3((JPH_Shape*)shape, &pos1, &rot, JPH_MotionType_Dynamic, ObjectLayers::MOVING);
        JPH_BodyCreationSettings* bs2 = JPH_BodyCreationSettings_Create3((JPH_Shape*)shape, &pos2, &rot, JPH_MotionType_Dynamic, ObjectLayers::MOVING);

        body1 = JPH_BodyInterface_CreateBody(bodyInterface, bs1);
        body2 = JPH_BodyInterface_CreateBody(bodyInterface, bs2);
        bodyId1 = JPH_Body_GetID(body1);
        bodyId2 = JPH_Body_GetID(body2);
        JPH_BodyInterface_AddBody(bodyInterface, bodyId1, JPH_Activation_Activate);
        JPH_BodyInterface_AddBody(bodyInterface, bodyId2, JPH_Activation_Activate);

        JPH_BodyCreationSettings_Destroy(bs1);
        JPH_BodyCreationSettings_Destroy(bs2);
        JPH_Shape_Destroy((JPH_Shape*)shape);
    }

    void TearDown() override {
        if (physicsSystem) {
            JPH_BodyInterface_RemoveBody(bodyInterface, bodyId1);
            JPH_BodyInterface_RemoveBody(bodyInterface, bodyId2);
            JPH_BodyInterface_DestroyBody(bodyInterface, bodyId1);
            JPH_BodyInterface_DestroyBody(bodyInterface, bodyId2);
            JPH_PhysicsSystem_Destroy(physicsSystem);
        }
        JPH_Shutdown();
    }
};

TEST_F(ConstraintTest, FixedConstraint_Create) {
    JPH_FixedConstraintSettings settings;
    JPH_FixedConstraintSettings_Init(&settings);

    JPH_FixedConstraint* constraint = JPH_FixedConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    EXPECT_EQ(JPH_Constraint_GetSubType((JPH_Constraint*)constraint), JPH_ConstraintSubType_Fixed);

    JPH_Vec3 lambda;
    JPH_FixedConstraint_GetTotalLambdaPosition(constraint, &lambda);
    EXPECT_FLOAT_EQ(lambda.x, 0.0f);

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}

TEST_F(ConstraintTest, DistanceConstraint_Create) {
    JPH_DistanceConstraintSettings settings;
    JPH_DistanceConstraintSettings_Init(&settings);

    JPH_DistanceConstraint* constraint = JPH_DistanceConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    EXPECT_EQ(JPH_Constraint_GetSubType((JPH_Constraint*)constraint), JPH_ConstraintSubType_Distance);

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}

TEST_F(ConstraintTest, DistanceConstraint_SetDistance) {
    JPH_DistanceConstraintSettings settings;
    JPH_DistanceConstraintSettings_Init(&settings);

    JPH_DistanceConstraint* constraint = JPH_DistanceConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    JPH_DistanceConstraint_SetDistance(constraint, 1.0f, 3.0f);
    EXPECT_FLOAT_EQ(JPH_DistanceConstraint_GetMinDistance(constraint), 1.0f);
    EXPECT_FLOAT_EQ(JPH_DistanceConstraint_GetMaxDistance(constraint), 3.0f);

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}

TEST_F(ConstraintTest, PointConstraint_Create) {
    JPH_PointConstraintSettings settings;
    JPH_PointConstraintSettings_Init(&settings);

    JPH_PointConstraint* constraint = JPH_PointConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    EXPECT_EQ(JPH_Constraint_GetSubType((JPH_Constraint*)constraint), JPH_ConstraintSubType_Point);

    JPH_Vec3 point1, point2;
    JPH_PointConstraint_GetLocalSpacePoint1(constraint, &point1);
    JPH_PointConstraint_GetLocalSpacePoint2(constraint, &point2);

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}

TEST_F(ConstraintTest, HingeConstraint_Create) {
    JPH_HingeConstraintSettings settings;
    JPH_HingeConstraintSettings_Init(&settings);

    JPH_HingeConstraint* constraint = JPH_HingeConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    EXPECT_EQ(JPH_Constraint_GetSubType((JPH_Constraint*)constraint), JPH_ConstraintSubType_Hinge);

    float angle = JPH_HingeConstraint_GetCurrentAngle(constraint);
    EXPECT_FALSE(std::isnan(angle));

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}

TEST_F(ConstraintTest, HingeConstraint_Limits) {
    JPH_HingeConstraintSettings settings;
    JPH_HingeConstraintSettings_Init(&settings);

    JPH_HingeConstraint* constraint = JPH_HingeConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    JPH_HingeConstraint_SetLimits(constraint, -JPH_M_PI / 4.0f, JPH_M_PI / 4.0f);
    EXPECT_TRUE(JPH_HingeConstraint_HasLimits(constraint));
    EXPECT_NEAR(JPH_HingeConstraint_GetLimitsMin(constraint), -JPH_M_PI / 4.0f, 0.001f);
    EXPECT_NEAR(JPH_HingeConstraint_GetLimitsMax(constraint), JPH_M_PI / 4.0f, 0.001f);

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}

TEST_F(ConstraintTest, HingeConstraint_Motor) {
    JPH_HingeConstraintSettings settings;
    JPH_HingeConstraintSettings_Init(&settings);

    JPH_HingeConstraint* constraint = JPH_HingeConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    JPH_HingeConstraint_SetMotorState(constraint, JPH_MotorState_Velocity);
    EXPECT_EQ(JPH_HingeConstraint_GetMotorState(constraint), JPH_MotorState_Velocity);

    JPH_HingeConstraint_SetTargetAngularVelocity(constraint, 1.0f);
    EXPECT_FLOAT_EQ(JPH_HingeConstraint_GetTargetAngularVelocity(constraint), 1.0f);

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}

TEST_F(ConstraintTest, SliderConstraint_Create) {
    JPH_SliderConstraintSettings settings;
    JPH_SliderConstraintSettings_Init(&settings);

    JPH_Vec3 axis = {1.0f, 0.0f, 0.0f};
    JPH_SliderConstraintSettings_SetSliderAxis(&settings, &axis);

    JPH_SliderConstraint* constraint = JPH_SliderConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    EXPECT_EQ(JPH_Constraint_GetSubType((JPH_Constraint*)constraint), JPH_ConstraintSubType_Slider);

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}

TEST_F(ConstraintTest, SliderConstraint_Limits) {
    JPH_SliderConstraintSettings settings;
    JPH_SliderConstraintSettings_Init(&settings);

    JPH_SliderConstraint* constraint = JPH_SliderConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    JPH_SliderConstraint_SetLimits(constraint, -1.0f, 1.0f);
    EXPECT_TRUE(JPH_SliderConstraint_HasLimits(constraint));
    EXPECT_FLOAT_EQ(JPH_SliderConstraint_GetLimitsMin(constraint), -1.0f);
    EXPECT_FLOAT_EQ(JPH_SliderConstraint_GetLimitsMax(constraint), 1.0f);

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}

TEST_F(ConstraintTest, ConeConstraint_Create) {
    JPH_ConeConstraintSettings settings;
    JPH_ConeConstraintSettings_Init(&settings);

    JPH_ConeConstraint* constraint = JPH_ConeConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    EXPECT_EQ(JPH_Constraint_GetSubType((JPH_Constraint*)constraint), JPH_ConstraintSubType_Cone);

    JPH_ConeConstraint_SetHalfConeAngle(constraint, JPH_M_PI / 6.0f);
    float cosAngle = JPH_ConeConstraint_GetCosHalfConeAngle(constraint);
    EXPECT_NEAR(cosAngle, cosf(JPH_M_PI / 6.0f), 0.001f);

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}

TEST_F(ConstraintTest, SwingTwistConstraint_Create) {
    JPH_SwingTwistConstraintSettings settings;
    JPH_SwingTwistConstraintSettings_Init(&settings);

    JPH_SwingTwistConstraint* constraint = JPH_SwingTwistConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    EXPECT_EQ(JPH_Constraint_GetSubType((JPH_Constraint*)constraint), JPH_ConstraintSubType_SwingTwist);

    float halfConeAngle = JPH_SwingTwistConstraint_GetNormalHalfConeAngle(constraint);
    EXPECT_FALSE(std::isnan(halfConeAngle));

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}

TEST_F(ConstraintTest, SixDOFConstraint_Create) {
    JPH_SixDOFConstraintSettings settings;
    JPH_SixDOFConstraintSettings_Init(&settings);

    JPH_SixDOFConstraint* constraint = JPH_SixDOFConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    EXPECT_EQ(JPH_Constraint_GetSubType((JPH_Constraint*)constraint), JPH_ConstraintSubType_SixDOF);

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}

TEST_F(ConstraintTest, SixDOFConstraintSettings_Axes) {
    JPH_SixDOFConstraintSettings settings;
    JPH_SixDOFConstraintSettings_Init(&settings);

    JPH_SixDOFConstraintSettings_MakeFreeAxis(&settings, JPH_SixDOFConstraintAxis_TranslationX);
    EXPECT_TRUE(JPH_SixDOFConstraintSettings_IsFreeAxis(&settings, JPH_SixDOFConstraintAxis_TranslationX));

    JPH_SixDOFConstraintSettings_MakeFixedAxis(&settings, JPH_SixDOFConstraintAxis_TranslationY);
    EXPECT_TRUE(JPH_SixDOFConstraintSettings_IsFixedAxis(&settings, JPH_SixDOFConstraintAxis_TranslationY));

    JPH_SixDOFConstraintSettings_SetLimitedAxis(&settings, JPH_SixDOFConstraintAxis_TranslationZ, -1.0f, 1.0f);
}

TEST_F(ConstraintTest, GearConstraint_Create) {
    JPH_GearConstraintSettings settings;
    JPH_GearConstraintSettings_Init(&settings);

    JPH_GearConstraint* constraint = JPH_GearConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    EXPECT_EQ(JPH_Constraint_GetSubType((JPH_Constraint*)constraint), JPH_ConstraintSubType_Gear);

    float lambda = JPH_GearConstraint_GetTotalLambda(constraint);
    EXPECT_FLOAT_EQ(lambda, 0.0f);

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}

TEST_F(ConstraintTest, Constraint_EnableDisable) {
    JPH_FixedConstraintSettings settings;
    JPH_FixedConstraintSettings_Init(&settings);

    JPH_FixedConstraint* constraint = JPH_FixedConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    EXPECT_TRUE(JPH_Constraint_GetEnabled((JPH_Constraint*)constraint));

    JPH_Constraint_SetEnabled((JPH_Constraint*)constraint, false);
    EXPECT_FALSE(JPH_Constraint_GetEnabled((JPH_Constraint*)constraint));

    JPH_Constraint_SetEnabled((JPH_Constraint*)constraint, true);
    EXPECT_TRUE(JPH_Constraint_GetEnabled((JPH_Constraint*)constraint));

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}

TEST_F(ConstraintTest, Constraint_UserData) {
    JPH_FixedConstraintSettings settings;
    JPH_FixedConstraintSettings_Init(&settings);

    JPH_FixedConstraint* constraint = JPH_FixedConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    JPH_Constraint_SetUserData((JPH_Constraint*)constraint, 12345);
    EXPECT_EQ(JPH_Constraint_GetUserData((JPH_Constraint*)constraint), 12345u);

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}

TEST_F(ConstraintTest, Constraint_Priority) {
    JPH_FixedConstraintSettings settings;
    JPH_FixedConstraintSettings_Init(&settings);

    JPH_FixedConstraint* constraint = JPH_FixedConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    JPH_Constraint_SetConstraintPriority((JPH_Constraint*)constraint, 100);
    EXPECT_EQ(JPH_Constraint_GetConstraintPriority((JPH_Constraint*)constraint), 100u);

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}

TEST_F(ConstraintTest, TwoBodyConstraint_GetBodies) {
    JPH_FixedConstraintSettings settings;
    JPH_FixedConstraintSettings_Init(&settings);

    JPH_FixedConstraint* constraint = JPH_FixedConstraint_Create(&settings, body1, body2);
    ASSERT_NE(constraint, nullptr);

    JPH_Body* b1 = JPH_TwoBodyConstraint_GetBody1((JPH_TwoBodyConstraint*)constraint);
    JPH_Body* b2 = JPH_TwoBodyConstraint_GetBody2((JPH_TwoBodyConstraint*)constraint);

    EXPECT_EQ(b1, body1);
    EXPECT_EQ(b2, body2);

    JPH_Constraint_Destroy((JPH_Constraint*)constraint);
}
