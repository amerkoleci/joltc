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

TEST_F(CharacterTest, CharacterVirtual_Create) {
    JPH_CapsuleShape* shape = JPH_CapsuleShape_Create(0.5f, 0.3f);
    ASSERT_NE(shape, nullptr);

    JPH_CharacterVirtualSettings settings;
    JPH_CharacterVirtualSettings_Init(&settings);
    settings.base.shape = (JPH_Shape*)shape;

    JPH_RVec3 position = {0.0f, 1.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_CharacterVirtual* character = JPH_CharacterVirtual_Create(&settings, &position, &rotation, 0, physicsSystem);
    ASSERT_NE(character, nullptr);

    JPH_CharacterID id = JPH_CharacterVirtual_GetID(character);
    EXPECT_NE(id, 0u);

    JPH_CharacterBase_Destroy((JPH_CharacterBase*)character);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(CharacterTest, CharacterVirtual_Mass) {
    JPH_CapsuleShape* shape = JPH_CapsuleShape_Create(0.5f, 0.3f);
    JPH_CharacterVirtualSettings settings;
    JPH_CharacterVirtualSettings_Init(&settings);
    settings.base.shape = (JPH_Shape*)shape;

    JPH_RVec3 position = {0.0f, 1.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_CharacterVirtual* character = JPH_CharacterVirtual_Create(&settings, &position, &rotation, 0, physicsSystem);

    JPH_CharacterVirtual_SetMass(character, 100.0f);
    EXPECT_FLOAT_EQ(JPH_CharacterVirtual_GetMass(character), 100.0f);

    JPH_CharacterBase_Destroy((JPH_CharacterBase*)character);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(CharacterTest, CharacterVirtual_UserData) {
    JPH_CapsuleShape* shape = JPH_CapsuleShape_Create(0.5f, 0.3f);
    JPH_CharacterVirtualSettings settings;
    JPH_CharacterVirtualSettings_Init(&settings);
    settings.base.shape = (JPH_Shape*)shape;

    JPH_RVec3 position = {0.0f, 1.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_CharacterVirtual* character = JPH_CharacterVirtual_Create(&settings, &position, &rotation, 0, physicsSystem);

    JPH_CharacterVirtual_SetUserData(character, 12345);
    EXPECT_EQ(JPH_CharacterVirtual_GetUserData(character), 12345u);

    JPH_CharacterBase_Destroy((JPH_CharacterBase*)character);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(CharacterTest, CharacterBase_SlopeAngle) {
    JPH_CapsuleShape* shape = JPH_CapsuleShape_Create(0.5f, 0.3f);
    JPH_CharacterVirtualSettings settings;
    JPH_CharacterVirtualSettings_Init(&settings);
    settings.base.shape = (JPH_Shape*)shape;

    JPH_RVec3 position = {0.0f, 1.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_CharacterVirtual* character = JPH_CharacterVirtual_Create(&settings, &position, &rotation, 0, physicsSystem);

    JPH_CharacterBase_SetMaxSlopeAngle((JPH_CharacterBase*)character, JPH_M_PI / 4.0f);
    float cosAngle = JPH_CharacterBase_GetCosMaxSlopeAngle((JPH_CharacterBase*)character);
    EXPECT_NEAR(cosAngle, cosf(JPH_M_PI / 4.0f), 0.001f);

    JPH_CharacterBase_Destroy((JPH_CharacterBase*)character);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(CharacterTest, CharacterBase_UpVector) {
    JPH_CapsuleShape* shape = JPH_CapsuleShape_Create(0.5f, 0.3f);
    JPH_CharacterVirtualSettings settings;
    JPH_CharacterVirtualSettings_Init(&settings);
    settings.base.shape = (JPH_Shape*)shape;

    JPH_RVec3 position = {0.0f, 1.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_CharacterVirtual* character = JPH_CharacterVirtual_Create(&settings, &position, &rotation, 0, physicsSystem);

    JPH_Vec3 up;
    JPH_CharacterBase_GetUp((JPH_CharacterBase*)character, &up);
    EXPECT_FLOAT_EQ(up.y, 1.0f);

    JPH_Vec3 newUp = {0.0f, 0.0f, 1.0f};
    JPH_CharacterBase_SetUp((JPH_CharacterBase*)character, &newUp);
    JPH_CharacterBase_GetUp((JPH_CharacterBase*)character, &up);
    EXPECT_FLOAT_EQ(up.z, 1.0f);

    JPH_CharacterBase_Destroy((JPH_CharacterBase*)character);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(CharacterTest, CharacterBase_GetShape) {
    JPH_CapsuleShape* shape = JPH_CapsuleShape_Create(0.5f, 0.3f);
    JPH_CharacterVirtualSettings settings;
    JPH_CharacterVirtualSettings_Init(&settings);
    settings.base.shape = (JPH_Shape*)shape;

    JPH_RVec3 position = {0.0f, 1.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_CharacterVirtual* character = JPH_CharacterVirtual_Create(&settings, &position, &rotation, 0, physicsSystem);

    const JPH_Shape* retrievedShape = JPH_CharacterBase_GetShape((JPH_CharacterBase*)character);
    EXPECT_NE(retrievedShape, nullptr);

    JPH_CharacterBase_Destroy((JPH_CharacterBase*)character);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}
