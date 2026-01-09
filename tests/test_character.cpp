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

class CharacterTest : public ::testing::Test {
protected:
    JPH_PhysicsSystem* physicsSystem = nullptr;
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
    }

    void TearDown() override {
        if (physicsSystem) JPH_PhysicsSystem_Destroy(physicsSystem);
        JPH_Shutdown();
    }
};

TEST_F(CharacterTest, CharacterSettings_Init) {
    JPH_CharacterSettings settings;
    JPH_CharacterSettings_Init(&settings);
    EXPECT_NE(settings.base.up.y, 0.0f);
}

TEST_F(CharacterTest, Character_Create) {
    JPH_CapsuleShape* shape = JPH_CapsuleShape_Create(0.5f, 0.3f);
    ASSERT_NE(shape, nullptr);

    JPH_CharacterSettings settings;
    JPH_CharacterSettings_Init(&settings);
    settings.base.shape = (JPH_Shape*)shape;

    JPH_RVec3 position = {0.0f, 1.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_Character* character = JPH_Character_Create(&settings, &position, &rotation, 0, physicsSystem);
    ASSERT_NE(character, nullptr);

    JPH_Character_AddToPhysicsSystem(character, JPH_Activation_Activate, true);

    JPH_BodyID bodyId = JPH_Character_GetBodyID(character);
    EXPECT_NE(bodyId, 0xFFFFFFFF);

    JPH_Character_RemoveFromPhysicsSystem(character, true);
    JPH_CharacterBase_Destroy((JPH_CharacterBase*)character);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(CharacterTest, Character_PositionAndRotation) {
    JPH_CapsuleShape* shape = JPH_CapsuleShape_Create(0.5f, 0.3f);
    JPH_CharacterSettings settings;
    JPH_CharacterSettings_Init(&settings);
    settings.base.shape = (JPH_Shape*)shape;

    JPH_RVec3 position = {1.0f, 2.0f, 3.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_Character* character = JPH_Character_Create(&settings, &position, &rotation, 0, physicsSystem);
    JPH_Character_AddToPhysicsSystem(character, JPH_Activation_Activate, true);

    JPH_RVec3 resultPos;
    JPH_Character_GetPosition(character, &resultPos, true);
    EXPECT_FLOAT_EQ(resultPos.x, 1.0f);
    EXPECT_FLOAT_EQ(resultPos.y, 2.0f);
    EXPECT_FLOAT_EQ(resultPos.z, 3.0f);

    JPH_RVec3 newPos = {5.0f, 6.0f, 7.0f};
    JPH_Character_SetPosition(character, &newPos, JPH_Activation_Activate, true);
    JPH_Character_GetPosition(character, &resultPos, true);
    EXPECT_FLOAT_EQ(resultPos.x, 5.0f);
    EXPECT_FLOAT_EQ(resultPos.y, 6.0f);
    EXPECT_FLOAT_EQ(resultPos.z, 7.0f);

    JPH_Character_RemoveFromPhysicsSystem(character, true);
    JPH_CharacterBase_Destroy((JPH_CharacterBase*)character);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(CharacterTest, Character_Velocity) {
    JPH_CapsuleShape* shape = JPH_CapsuleShape_Create(0.5f, 0.3f);
    JPH_CharacterSettings settings;
    JPH_CharacterSettings_Init(&settings);
    settings.base.shape = (JPH_Shape*)shape;

    JPH_RVec3 position = {0.0f, 1.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_Character* character = JPH_Character_Create(&settings, &position, &rotation, 0, physicsSystem);
    JPH_Character_AddToPhysicsSystem(character, JPH_Activation_Activate, true);

    JPH_Vec3 velocity = {1.0f, 0.0f, 0.0f};
    JPH_Character_SetLinearVelocity(character, &velocity, true);

    JPH_Vec3 resultVel;
    JPH_Character_GetLinearVelocity(character, &resultVel);
    EXPECT_FLOAT_EQ(resultVel.x, 1.0f);

    JPH_Character_RemoveFromPhysicsSystem(character, true);
    JPH_CharacterBase_Destroy((JPH_CharacterBase*)character);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(CharacterTest, CharacterVirtualSettings_Init) {
    JPH_CharacterVirtualSettings settings;
    JPH_CharacterVirtualSettings_Init(&settings);
    EXPECT_GT(settings.mass, 0.0f);
}

// Note: CharacterVirtual tests are disabled due to crashes in JPH_CharacterVirtual_Create
// This appears to be a bug in the joltc library that needs investigation
