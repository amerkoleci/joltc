// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

#include <gtest/gtest.h>
#include "joltc.h"
#include <cstring>

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

class PhysicsSystemTest : public ::testing::Test {
protected:
    JPH_JobSystem* jobSystem = nullptr;
    JPH_BroadPhaseLayerInterface* broadPhaseLayerInterface = nullptr;
    JPH_ObjectVsBroadPhaseLayerFilter* objectVsBroadPhaseLayerFilter = nullptr;
    JPH_ObjectLayerPairFilter* objectLayerPairFilter = nullptr;
    JPH_PhysicsSystem* physicsSystem = nullptr;

    void SetUp() override {
        ASSERT_TRUE(JPH_Init());

        jobSystem = JPH_JobSystemThreadPool_Create(nullptr);
        ASSERT_NE(jobSystem, nullptr);

        broadPhaseLayerInterface = JPH_BroadPhaseLayerInterfaceTable_Create(ObjectLayers::NUM_LAYERS, BroadPhaseLayers::NUM_LAYERS);
        JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broadPhaseLayerInterface, ObjectLayers::NON_MOVING, BroadPhaseLayers::NON_MOVING);
        JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broadPhaseLayerInterface, ObjectLayers::MOVING, BroadPhaseLayers::MOVING);
        ASSERT_NE(broadPhaseLayerInterface, nullptr);

        objectLayerPairFilter = JPH_ObjectLayerPairFilterTable_Create(ObjectLayers::NUM_LAYERS);
        JPH_ObjectLayerPairFilterTable_EnableCollision(objectLayerPairFilter, ObjectLayers::NON_MOVING, ObjectLayers::MOVING);
        JPH_ObjectLayerPairFilterTable_EnableCollision(objectLayerPairFilter, ObjectLayers::MOVING, ObjectLayers::MOVING);
        ASSERT_NE(objectLayerPairFilter, nullptr);

        objectVsBroadPhaseLayerFilter = JPH_ObjectVsBroadPhaseLayerFilterTable_Create(
            broadPhaseLayerInterface, BroadPhaseLayers::NUM_LAYERS,
            objectLayerPairFilter, ObjectLayers::NUM_LAYERS);
        ASSERT_NE(objectVsBroadPhaseLayerFilter, nullptr);

        JPH_PhysicsSystemSettings settings = {};
        settings.maxBodies = 1024;
        settings.numBodyMutexes = 0;
        settings.maxBodyPairs = 1024;
        settings.maxContactConstraints = 1024;
        settings.broadPhaseLayerInterface = broadPhaseLayerInterface;
        settings.objectVsBroadPhaseLayerFilter = objectVsBroadPhaseLayerFilter;
        settings.objectLayerPairFilter = objectLayerPairFilter;

        physicsSystem = JPH_PhysicsSystem_Create(&settings);
        ASSERT_NE(physicsSystem, nullptr);
    }

    void TearDown() override {
        if (physicsSystem) JPH_PhysicsSystem_Destroy(physicsSystem);
        if (jobSystem) JPH_JobSystem_Destroy(jobSystem);
        JPH_Shutdown();
    }
};

TEST_F(PhysicsSystemTest, Create_ValidSettings) {
    EXPECT_NE(physicsSystem, nullptr);
}

TEST_F(PhysicsSystemTest, GetBodyInterface) {
    JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterface(physicsSystem);
    EXPECT_NE(bodyInterface, nullptr);
}

TEST_F(PhysicsSystemTest, GetBodyInterfaceNoLock) {
    JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterfaceNoLock(physicsSystem);
    EXPECT_NE(bodyInterface, nullptr);
}

TEST_F(PhysicsSystemTest, SetGravity) {
    JPH_Vec3 gravity = {0.0f, -10.0f, 0.0f};
    JPH_PhysicsSystem_SetGravity(physicsSystem, &gravity);

    JPH_Vec3 result;
    JPH_PhysicsSystem_GetGravity(physicsSystem, &result);
    EXPECT_FLOAT_EQ(result.x, 0.0f);
    EXPECT_FLOAT_EQ(result.y, -10.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);
}

TEST_F(PhysicsSystemTest, GetNumBodies_Initial) {
    uint32_t numBodies = JPH_PhysicsSystem_GetNumBodies(physicsSystem);
    EXPECT_EQ(numBodies, 0u);
}

TEST_F(PhysicsSystemTest, GetNumActiveBodies_Initial) {
    uint32_t numActiveBodies = JPH_PhysicsSystem_GetNumActiveBodies(physicsSystem, JPH_BodyType_Rigid);
    EXPECT_EQ(numActiveBodies, 0u);
}

TEST_F(PhysicsSystemTest, GetMaxBodies) {
    uint32_t maxBodies = JPH_PhysicsSystem_GetMaxBodies(physicsSystem);
    EXPECT_EQ(maxBodies, 1024u);
}

TEST_F(PhysicsSystemTest, CreateBody_Sphere) {
    JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterface(physicsSystem);
    ASSERT_NE(bodyInterface, nullptr);

    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(shape, nullptr);

    JPH_Vec3 position = {0.0f, 10.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_BodyCreationSettings* bodySettings = JPH_BodyCreationSettings_Create3(
        (const JPH_Shape*)shape, &position, &rotation,
        JPH_MotionType_Dynamic, ObjectLayers::MOVING);
    ASSERT_NE(bodySettings, nullptr);

    JPH_Body* body = JPH_BodyInterface_CreateBody(bodyInterface, bodySettings);
    ASSERT_NE(body, nullptr);

    JPH_BodyID bodyId = JPH_Body_GetID(body);
    EXPECT_NE(bodyId, 0xFFFFFFFF);

    uint32_t numBodies = JPH_PhysicsSystem_GetNumBodies(physicsSystem);
    EXPECT_EQ(numBodies, 1u);

    JPH_BodyInterface_AddBody(bodyInterface, bodyId, JPH_Activation_Activate);

    uint32_t numActiveBodies = JPH_PhysicsSystem_GetNumActiveBodies(physicsSystem, JPH_BodyType_Rigid);
    EXPECT_EQ(numActiveBodies, 1u);

    JPH_BodyInterface_RemoveBody(bodyInterface, bodyId);
    JPH_BodyInterface_DestroyBody(bodyInterface, bodyId);
    JPH_BodyCreationSettings_Destroy(bodySettings);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(PhysicsSystemTest, CreateBody_Box) {
    JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterface(physicsSystem);
    ASSERT_NE(bodyInterface, nullptr);

    JPH_Vec3 halfExtent = {1.0f, 1.0f, 1.0f};
    JPH_BoxShape* shape = JPH_BoxShape_Create(&halfExtent, JPH_DEFAULT_CONVEX_RADIUS);
    ASSERT_NE(shape, nullptr);

    JPH_Vec3 position = {0.0f, 0.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_BodyCreationSettings* bodySettings = JPH_BodyCreationSettings_Create3(
        (const JPH_Shape*)shape, &position, &rotation,
        JPH_MotionType_Static, ObjectLayers::NON_MOVING);

    JPH_Body* body = JPH_BodyInterface_CreateBody(bodyInterface, bodySettings);
    ASSERT_NE(body, nullptr);

    JPH_BodyID bodyId = JPH_Body_GetID(body);
    JPH_BodyInterface_AddBody(bodyInterface, bodyId, JPH_Activation_DontActivate);

    uint32_t numActiveBodies = JPH_PhysicsSystem_GetNumActiveBodies(physicsSystem, JPH_BodyType_Rigid);
    EXPECT_EQ(numActiveBodies, 0u);

    JPH_BodyInterface_RemoveBody(bodyInterface, bodyId);
    JPH_BodyInterface_DestroyBody(bodyInterface, bodyId);
    JPH_BodyCreationSettings_Destroy(bodySettings);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(PhysicsSystemTest, Body_GetPosition) {
    JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterface(physicsSystem);
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);

    JPH_Vec3 position = {1.0f, 2.0f, 3.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_BodyCreationSettings* bodySettings = JPH_BodyCreationSettings_Create3(
        (const JPH_Shape*)shape, &position, &rotation,
        JPH_MotionType_Dynamic, ObjectLayers::MOVING);

    JPH_Body* body = JPH_BodyInterface_CreateBody(bodyInterface, bodySettings);
    JPH_BodyID bodyId = JPH_Body_GetID(body);
    JPH_BodyInterface_AddBody(bodyInterface, bodyId, JPH_Activation_Activate);

    JPH_RVec3 resultPos;
    JPH_BodyInterface_GetPosition(bodyInterface, bodyId, &resultPos);
    EXPECT_FLOAT_EQ(resultPos.x, 1.0f);
    EXPECT_FLOAT_EQ(resultPos.y, 2.0f);
    EXPECT_FLOAT_EQ(resultPos.z, 3.0f);

    JPH_BodyInterface_RemoveBody(bodyInterface, bodyId);
    JPH_BodyInterface_DestroyBody(bodyInterface, bodyId);
    JPH_BodyCreationSettings_Destroy(bodySettings);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(PhysicsSystemTest, Body_SetPosition) {
    JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterface(physicsSystem);
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);

    JPH_Vec3 position = {0.0f, 0.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_BodyCreationSettings* bodySettings = JPH_BodyCreationSettings_Create3(
        (const JPH_Shape*)shape, &position, &rotation,
        JPH_MotionType_Dynamic, ObjectLayers::MOVING);

    JPH_Body* body = JPH_BodyInterface_CreateBody(bodyInterface, bodySettings);
    JPH_BodyID bodyId = JPH_Body_GetID(body);
    JPH_BodyInterface_AddBody(bodyInterface, bodyId, JPH_Activation_Activate);

    JPH_RVec3 newPos = {5.0f, 10.0f, 15.0f};
    JPH_BodyInterface_SetPosition(bodyInterface, bodyId, &newPos, JPH_Activation_Activate);

    JPH_RVec3 resultPos;
    JPH_BodyInterface_GetPosition(bodyInterface, bodyId, &resultPos);
    EXPECT_FLOAT_EQ(resultPos.x, 5.0f);
    EXPECT_FLOAT_EQ(resultPos.y, 10.0f);
    EXPECT_FLOAT_EQ(resultPos.z, 15.0f);

    JPH_BodyInterface_RemoveBody(bodyInterface, bodyId);
    JPH_BodyInterface_DestroyBody(bodyInterface, bodyId);
    JPH_BodyCreationSettings_Destroy(bodySettings);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(PhysicsSystemTest, Body_SetLinearVelocity) {
    JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterface(physicsSystem);
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);

    JPH_Vec3 position = {0.0f, 10.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_BodyCreationSettings* bodySettings = JPH_BodyCreationSettings_Create3(
        (const JPH_Shape*)shape, &position, &rotation,
        JPH_MotionType_Dynamic, ObjectLayers::MOVING);

    JPH_Body* body = JPH_BodyInterface_CreateBody(bodyInterface, bodySettings);
    JPH_BodyID bodyId = JPH_Body_GetID(body);
    JPH_BodyInterface_AddBody(bodyInterface, bodyId, JPH_Activation_Activate);

    JPH_Vec3 velocity = {1.0f, 2.0f, 3.0f};
    JPH_BodyInterface_SetLinearVelocity(bodyInterface, bodyId, &velocity);

    JPH_Vec3 resultVel;
    JPH_BodyInterface_GetLinearVelocity(bodyInterface, bodyId, &resultVel);
    EXPECT_FLOAT_EQ(resultVel.x, 1.0f);
    EXPECT_FLOAT_EQ(resultVel.y, 2.0f);
    EXPECT_FLOAT_EQ(resultVel.z, 3.0f);

    JPH_BodyInterface_RemoveBody(bodyInterface, bodyId);
    JPH_BodyInterface_DestroyBody(bodyInterface, bodyId);
    JPH_BodyCreationSettings_Destroy(bodySettings);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(PhysicsSystemTest, Body_AddImpulse) {
    JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterface(physicsSystem);
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);

    JPH_Vec3 position = {0.0f, 10.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_BodyCreationSettings* bodySettings = JPH_BodyCreationSettings_Create3(
        (const JPH_Shape*)shape, &position, &rotation,
        JPH_MotionType_Dynamic, ObjectLayers::MOVING);

    JPH_Body* body = JPH_BodyInterface_CreateBody(bodyInterface, bodySettings);
    JPH_BodyID bodyId = JPH_Body_GetID(body);
    JPH_BodyInterface_AddBody(bodyInterface, bodyId, JPH_Activation_Activate);

    JPH_Vec3 initialVel;
    JPH_BodyInterface_GetLinearVelocity(bodyInterface, bodyId, &initialVel);
    EXPECT_FLOAT_EQ(initialVel.x, 0.0f);

    JPH_Vec3 impulse = {10.0f, 0.0f, 0.0f};
    JPH_BodyInterface_AddImpulse(bodyInterface, bodyId, &impulse);

    JPH_Vec3 resultVel;
    JPH_BodyInterface_GetLinearVelocity(bodyInterface, bodyId, &resultVel);
    EXPECT_GT(resultVel.x, 0.0f);

    JPH_BodyInterface_RemoveBody(bodyInterface, bodyId);
    JPH_BodyInterface_DestroyBody(bodyInterface, bodyId);
    JPH_BodyCreationSettings_Destroy(bodySettings);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(PhysicsSystemTest, Body_GetMotionType) {
    JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterface(physicsSystem);
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);

    JPH_Vec3 position = {0.0f, 10.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_BodyCreationSettings* bodySettings = JPH_BodyCreationSettings_Create3(
        (const JPH_Shape*)shape, &position, &rotation,
        JPH_MotionType_Dynamic, ObjectLayers::MOVING);

    JPH_Body* body = JPH_BodyInterface_CreateBody(bodyInterface, bodySettings);
    JPH_BodyID bodyId = JPH_Body_GetID(body);
    JPH_BodyInterface_AddBody(bodyInterface, bodyId, JPH_Activation_Activate);

    JPH_MotionType motionType = JPH_BodyInterface_GetMotionType(bodyInterface, bodyId);
    EXPECT_EQ(motionType, JPH_MotionType_Dynamic);

    JPH_BodyInterface_RemoveBody(bodyInterface, bodyId);
    JPH_BodyInterface_DestroyBody(bodyInterface, bodyId);
    JPH_BodyCreationSettings_Destroy(bodySettings);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(PhysicsSystemTest, Body_ActivateDeactivate) {
    JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterface(physicsSystem);
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);

    JPH_Vec3 position = {0.0f, 10.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_BodyCreationSettings* bodySettings = JPH_BodyCreationSettings_Create3(
        (const JPH_Shape*)shape, &position, &rotation,
        JPH_MotionType_Dynamic, ObjectLayers::MOVING);

    JPH_Body* body = JPH_BodyInterface_CreateBody(bodyInterface, bodySettings);
    JPH_BodyID bodyId = JPH_Body_GetID(body);

    JPH_BodyInterface_AddBody(bodyInterface, bodyId, JPH_Activation_DontActivate);
    EXPECT_FALSE(JPH_BodyInterface_IsActive(bodyInterface, bodyId));

    JPH_BodyInterface_ActivateBody(bodyInterface, bodyId);
    EXPECT_TRUE(JPH_BodyInterface_IsActive(bodyInterface, bodyId));

    JPH_BodyInterface_DeactivateBody(bodyInterface, bodyId);
    EXPECT_FALSE(JPH_BodyInterface_IsActive(bodyInterface, bodyId));

    JPH_BodyInterface_RemoveBody(bodyInterface, bodyId);
    JPH_BodyInterface_DestroyBody(bodyInterface, bodyId);
    JPH_BodyCreationSettings_Destroy(bodySettings);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(PhysicsSystemTest, Body_Restitution) {
    JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterface(physicsSystem);
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);

    JPH_Vec3 position = {0.0f, 10.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_BodyCreationSettings* bodySettings = JPH_BodyCreationSettings_Create3(
        (const JPH_Shape*)shape, &position, &rotation,
        JPH_MotionType_Dynamic, ObjectLayers::MOVING);

    JPH_Body* body = JPH_BodyInterface_CreateBody(bodyInterface, bodySettings);
    JPH_BodyID bodyId = JPH_Body_GetID(body);
    JPH_BodyInterface_AddBody(bodyInterface, bodyId, JPH_Activation_Activate);

    JPH_BodyInterface_SetRestitution(bodyInterface, bodyId, 0.8f);
    float restitution = JPH_BodyInterface_GetRestitution(bodyInterface, bodyId);
    EXPECT_FLOAT_EQ(restitution, 0.8f);

    JPH_BodyInterface_RemoveBody(bodyInterface, bodyId);
    JPH_BodyInterface_DestroyBody(bodyInterface, bodyId);
    JPH_BodyCreationSettings_Destroy(bodySettings);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(PhysicsSystemTest, Body_Friction) {
    JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterface(physicsSystem);
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);

    JPH_Vec3 position = {0.0f, 10.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_BodyCreationSettings* bodySettings = JPH_BodyCreationSettings_Create3(
        (const JPH_Shape*)shape, &position, &rotation,
        JPH_MotionType_Dynamic, ObjectLayers::MOVING);

    JPH_Body* body = JPH_BodyInterface_CreateBody(bodyInterface, bodySettings);
    JPH_BodyID bodyId = JPH_Body_GetID(body);
    JPH_BodyInterface_AddBody(bodyInterface, bodyId, JPH_Activation_Activate);

    JPH_BodyInterface_SetFriction(bodyInterface, bodyId, 0.5f);
    float friction = JPH_BodyInterface_GetFriction(bodyInterface, bodyId);
    EXPECT_FLOAT_EQ(friction, 0.5f);

    JPH_BodyInterface_RemoveBody(bodyInterface, bodyId);
    JPH_BodyInterface_DestroyBody(bodyInterface, bodyId);
    JPH_BodyCreationSettings_Destroy(bodySettings);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(PhysicsSystemTest, Update_FallingBody) {
    JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterface(physicsSystem);
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);

    JPH_Vec3 position = {0.0f, 100.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_BodyCreationSettings* bodySettings = JPH_BodyCreationSettings_Create3(
        (const JPH_Shape*)shape, &position, &rotation,
        JPH_MotionType_Dynamic, ObjectLayers::MOVING);

    JPH_Body* body = JPH_BodyInterface_CreateBody(bodyInterface, bodySettings);
    JPH_BodyID bodyId = JPH_Body_GetID(body);
    JPH_BodyInterface_AddBody(bodyInterface, bodyId, JPH_Activation_Activate);

    JPH_Vec3 gravity = {0.0f, -10.0f, 0.0f};
    JPH_PhysicsSystem_SetGravity(physicsSystem, &gravity);

    JPH_RVec3 initialPos;
    JPH_BodyInterface_GetPosition(bodyInterface, bodyId, &initialPos);

    JPH_PhysicsSystem_OptimizeBroadPhase(physicsSystem);
    JPH_PhysicsUpdateError error = JPH_PhysicsSystem_Update(physicsSystem, 1.0f / 60.0f, 1, jobSystem);
    EXPECT_EQ(error, JPH_PhysicsUpdateError_None);

    JPH_RVec3 newPos;
    JPH_BodyInterface_GetPosition(bodyInterface, bodyId, &newPos);
    EXPECT_LT(newPos.y, initialPos.y);

    JPH_BodyInterface_RemoveBody(bodyInterface, bodyId);
    JPH_BodyInterface_DestroyBody(bodyInterface, bodyId);
    JPH_BodyCreationSettings_Destroy(bodySettings);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(PhysicsSystemTest, MultipleBodies) {
    JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterface(physicsSystem);

    const int NUM_BODIES = 10;
    JPH_BodyID bodyIds[NUM_BODIES];

    for (int i = 0; i < NUM_BODIES; i++) {
        JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);

        JPH_Vec3 position = {(float)i * 3.0f, 10.0f, 0.0f};
        JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

        JPH_BodyCreationSettings* bodySettings = JPH_BodyCreationSettings_Create3(
            (const JPH_Shape*)shape, &position, &rotation,
            JPH_MotionType_Dynamic, ObjectLayers::MOVING);

        JPH_Body* body = JPH_BodyInterface_CreateBody(bodyInterface, bodySettings);
        bodyIds[i] = JPH_Body_GetID(body);
        JPH_BodyInterface_AddBody(bodyInterface, bodyIds[i], JPH_Activation_Activate);

        JPH_BodyCreationSettings_Destroy(bodySettings);
        JPH_Shape_Destroy((JPH_Shape*)shape);
    }

    uint32_t numBodies = JPH_PhysicsSystem_GetNumBodies(physicsSystem);
    EXPECT_EQ(numBodies, (uint32_t)NUM_BODIES);

    uint32_t numActiveBodies = JPH_PhysicsSystem_GetNumActiveBodies(physicsSystem, JPH_BodyType_Rigid);
    EXPECT_EQ(numActiveBodies, (uint32_t)NUM_BODIES);

    for (int i = 0; i < NUM_BODIES; i++) {
        JPH_BodyInterface_RemoveBody(bodyInterface, bodyIds[i]);
        JPH_BodyInterface_DestroyBody(bodyInterface, bodyIds[i]);
    }
}
