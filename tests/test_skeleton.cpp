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

class SkeletonTest : public ::testing::Test {
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

// ============================================================================
// Skeleton Tests
// ============================================================================

TEST_F(SkeletonTest, Skeleton_Create) {
    JPH_Skeleton* skeleton = JPH_Skeleton_Create();
    ASSERT_NE(skeleton, nullptr);

    EXPECT_EQ(JPH_Skeleton_GetJointCount(skeleton), 0);

    JPH_Skeleton_Destroy(skeleton);
}

TEST_F(SkeletonTest, Skeleton_AddJoint) {
    JPH_Skeleton* skeleton = JPH_Skeleton_Create();

    uint32_t rootIndex = JPH_Skeleton_AddJoint(skeleton, "root");
    EXPECT_EQ(rootIndex, 0u);
    EXPECT_EQ(JPH_Skeleton_GetJointCount(skeleton), 1);

    uint32_t childIndex = JPH_Skeleton_AddJoint2(skeleton, "child", 0);
    EXPECT_EQ(childIndex, 1u);
    EXPECT_EQ(JPH_Skeleton_GetJointCount(skeleton), 2);

    JPH_Skeleton_Destroy(skeleton);
}

TEST_F(SkeletonTest, Skeleton_AddJointByName) {
    JPH_Skeleton* skeleton = JPH_Skeleton_Create();

    JPH_Skeleton_AddJoint(skeleton, "root");
    uint32_t childIndex = JPH_Skeleton_AddJoint3(skeleton, "child", "root");
    EXPECT_EQ(childIndex, 1u);

    JPH_Skeleton_Destroy(skeleton);
}

TEST_F(SkeletonTest, Skeleton_GetJoint) {
    JPH_Skeleton* skeleton = JPH_Skeleton_Create();

    JPH_Skeleton_AddJoint(skeleton, "root");
    JPH_Skeleton_AddJoint2(skeleton, "spine", 0);

    JPH_SkeletonJoint joint;
    JPH_Skeleton_GetJoint(skeleton, 0, &joint);
    EXPECT_STREQ(joint.name, "root");
    EXPECT_EQ(joint.parentJointIndex, -1);

    JPH_Skeleton_GetJoint(skeleton, 1, &joint);
    EXPECT_STREQ(joint.name, "spine");
    EXPECT_EQ(joint.parentJointIndex, 0);

    JPH_Skeleton_Destroy(skeleton);
}

TEST_F(SkeletonTest, Skeleton_GetJointIndex) {
    JPH_Skeleton* skeleton = JPH_Skeleton_Create();

    JPH_Skeleton_AddJoint(skeleton, "root");
    JPH_Skeleton_AddJoint2(skeleton, "spine", 0);
    JPH_Skeleton_AddJoint2(skeleton, "head", 1);

    EXPECT_EQ(JPH_Skeleton_GetJointIndex(skeleton, "root"), 0);
    EXPECT_EQ(JPH_Skeleton_GetJointIndex(skeleton, "spine"), 1);
    EXPECT_EQ(JPH_Skeleton_GetJointIndex(skeleton, "head"), 2);
    EXPECT_EQ(JPH_Skeleton_GetJointIndex(skeleton, "nonexistent"), -1);

    JPH_Skeleton_Destroy(skeleton);
}

TEST_F(SkeletonTest, Skeleton_JointsCorrectlyOrdered) {
    JPH_Skeleton* skeleton = JPH_Skeleton_Create();

    JPH_Skeleton_AddJoint(skeleton, "root");
    JPH_Skeleton_AddJoint2(skeleton, "child", 0);

    EXPECT_TRUE(JPH_Skeleton_AreJointsCorrectlyOrdered(skeleton));

    JPH_Skeleton_Destroy(skeleton);
}

// ============================================================================
// SkeletonPose Tests
// ============================================================================

TEST_F(SkeletonTest, SkeletonPose_Create) {
    JPH_SkeletonPose* pose = JPH_SkeletonPose_Create();
    ASSERT_NE(pose, nullptr);

    JPH_SkeletonPose_Destroy(pose);
}

TEST_F(SkeletonTest, SkeletonPose_SetSkeleton) {
    JPH_Skeleton* skeleton = JPH_Skeleton_Create();
    JPH_Skeleton_AddJoint(skeleton, "root");
    JPH_Skeleton_AddJoint2(skeleton, "child", 0);

    JPH_SkeletonPose* pose = JPH_SkeletonPose_Create();
    JPH_SkeletonPose_SetSkeleton(pose, skeleton);

    const JPH_Skeleton* retrievedSkeleton = JPH_SkeletonPose_GetSkeleton(pose);
    EXPECT_EQ(retrievedSkeleton, skeleton);
    EXPECT_EQ(JPH_SkeletonPose_GetJointCount(pose), 2);

    JPH_SkeletonPose_Destroy(pose);
    JPH_Skeleton_Destroy(skeleton);
}

TEST_F(SkeletonTest, SkeletonPose_RootOffset) {
    JPH_SkeletonPose* pose = JPH_SkeletonPose_Create();

    JPH_RVec3 offset = {1.0f, 2.0f, 3.0f};
    JPH_SkeletonPose_SetRootOffset(pose, &offset);

    JPH_RVec3 result;
    JPH_SkeletonPose_GetRootOffset(pose, &result);
    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
    EXPECT_FLOAT_EQ(result.z, 3.0f);

    JPH_SkeletonPose_Destroy(pose);
}

TEST_F(SkeletonTest, SkeletonPose_JointState) {
    JPH_Skeleton* skeleton = JPH_Skeleton_Create();
    JPH_Skeleton_AddJoint(skeleton, "root");

    JPH_SkeletonPose* pose = JPH_SkeletonPose_Create();
    JPH_SkeletonPose_SetSkeleton(pose, skeleton);

    JPH_Vec3 translation = {1.0f, 2.0f, 3.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};
    JPH_SkeletonPose_SetJointState(pose, 0, &translation, &rotation);

    JPH_Vec3 outTranslation;
    JPH_Quat outRotation;
    JPH_SkeletonPose_GetJointState(pose, 0, &outTranslation, &outRotation);
    EXPECT_FLOAT_EQ(outTranslation.x, 1.0f);
    EXPECT_FLOAT_EQ(outTranslation.y, 2.0f);
    EXPECT_FLOAT_EQ(outTranslation.z, 3.0f);
    EXPECT_FLOAT_EQ(outRotation.w, 1.0f);

    JPH_SkeletonPose_Destroy(pose);
    JPH_Skeleton_Destroy(skeleton);
}

// ============================================================================
// RagdollSettings Tests
// ============================================================================

TEST_F(SkeletonTest, RagdollSettings_Create) {
    JPH_RagdollSettings* settings = JPH_RagdollSettings_Create();
    ASSERT_NE(settings, nullptr);

    JPH_RagdollSettings_Destroy(settings);
}

TEST_F(SkeletonTest, RagdollSettings_SetSkeleton) {
    JPH_Skeleton* skeleton = JPH_Skeleton_Create();
    JPH_Skeleton_AddJoint(skeleton, "root");

    JPH_RagdollSettings* settings = JPH_RagdollSettings_Create();
    JPH_RagdollSettings_SetSkeleton(settings, skeleton);

    const JPH_Skeleton* retrievedSkeleton = JPH_RagdollSettings_GetSkeleton(settings);
    EXPECT_EQ(retrievedSkeleton, skeleton);

    JPH_RagdollSettings_Destroy(settings);
    JPH_Skeleton_Destroy(skeleton);
}

TEST_F(SkeletonTest, RagdollSettings_Parts) {
    JPH_RagdollSettings* settings = JPH_RagdollSettings_Create();

    JPH_RagdollSettings_ResizeParts(settings, 3);
    EXPECT_EQ(JPH_RagdollSettings_GetPartCount(settings), 3);

    // Set part properties
    JPH_SphereShape* shape = JPH_SphereShape_Create(0.5f);
    JPH_RVec3 position = {0.0f, 1.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_RagdollSettings_SetPartShape(settings, 0, (JPH_Shape*)shape);
    JPH_RagdollSettings_SetPartPosition(settings, 0, &position);
    JPH_RagdollSettings_SetPartRotation(settings, 0, &rotation);
    JPH_RagdollSettings_SetPartMotionType(settings, 0, JPH_MotionType_Dynamic);
    JPH_RagdollSettings_SetPartObjectLayer(settings, 0, ObjectLayers::MOVING);

    JPH_RagdollSettings_Destroy(settings);
    JPH_Shape_Destroy((JPH_Shape*)shape);
}

// ============================================================================
// Ragdoll Tests
// ============================================================================

TEST_F(SkeletonTest, Ragdoll_CreateSimple) {
    // Create skeleton
    JPH_Skeleton* skeleton = JPH_Skeleton_Create();
    JPH_Skeleton_AddJoint(skeleton, "root");

    // Create ragdoll settings
    JPH_RagdollSettings* settings = JPH_RagdollSettings_Create();
    JPH_RagdollSettings_SetSkeleton(settings, skeleton);
    JPH_RagdollSettings_ResizeParts(settings, 1);

    // Set up part
    JPH_SphereShape* shape = JPH_SphereShape_Create(0.5f);
    JPH_RVec3 position = {0.0f, 1.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_RagdollSettings_SetPartShape(settings, 0, (JPH_Shape*)shape);
    JPH_RagdollSettings_SetPartPosition(settings, 0, &position);
    JPH_RagdollSettings_SetPartRotation(settings, 0, &rotation);
    JPH_RagdollSettings_SetPartMotionType(settings, 0, JPH_MotionType_Dynamic);
    JPH_RagdollSettings_SetPartObjectLayer(settings, 0, ObjectLayers::MOVING);

    // Create ragdoll
    JPH_Ragdoll* ragdoll = JPH_RagdollSettings_CreateRagdoll(settings, physicsSystem, 0, 0);
    ASSERT_NE(ragdoll, nullptr);

    EXPECT_EQ(JPH_Ragdoll_GetBodyCount(ragdoll), 1);

    JPH_BodyID bodyId = JPH_Ragdoll_GetBodyID(ragdoll, 0);
    EXPECT_NE(bodyId, 0xFFFFFFFF);

    JPH_Ragdoll_Destroy(ragdoll);
    JPH_RagdollSettings_Destroy(settings);
    JPH_Shape_Destroy((JPH_Shape*)shape);
    JPH_Skeleton_Destroy(skeleton);
}

TEST_F(SkeletonTest, Ragdoll_AddRemoveFromPhysicsSystem) {
    // Create skeleton
    JPH_Skeleton* skeleton = JPH_Skeleton_Create();
    JPH_Skeleton_AddJoint(skeleton, "root");

    // Create ragdoll settings
    JPH_RagdollSettings* settings = JPH_RagdollSettings_Create();
    JPH_RagdollSettings_SetSkeleton(settings, skeleton);
    JPH_RagdollSettings_ResizeParts(settings, 1);

    JPH_SphereShape* shape = JPH_SphereShape_Create(0.5f);
    JPH_RVec3 position = {0.0f, 1.0f, 0.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_RagdollSettings_SetPartShape(settings, 0, (JPH_Shape*)shape);
    JPH_RagdollSettings_SetPartPosition(settings, 0, &position);
    JPH_RagdollSettings_SetPartRotation(settings, 0, &rotation);
    JPH_RagdollSettings_SetPartMotionType(settings, 0, JPH_MotionType_Dynamic);
    JPH_RagdollSettings_SetPartObjectLayer(settings, 0, ObjectLayers::MOVING);

    JPH_Ragdoll* ragdoll = JPH_RagdollSettings_CreateRagdoll(settings, physicsSystem, 0, 0);

    // Add to physics system
    JPH_Ragdoll_AddToPhysicsSystem(ragdoll, JPH_Activation_Activate, true);
    EXPECT_TRUE(JPH_Ragdoll_IsActive(ragdoll, true));

    // Remove from physics system
    JPH_Ragdoll_RemoveFromPhysicsSystem(ragdoll, true);

    JPH_Ragdoll_Destroy(ragdoll);
    JPH_RagdollSettings_Destroy(settings);
    JPH_Shape_Destroy((JPH_Shape*)shape);
    JPH_Skeleton_Destroy(skeleton);
}

TEST_F(SkeletonTest, Ragdoll_GetRootTransform) {
    JPH_Skeleton* skeleton = JPH_Skeleton_Create();
    JPH_Skeleton_AddJoint(skeleton, "root");

    JPH_RagdollSettings* settings = JPH_RagdollSettings_Create();
    JPH_RagdollSettings_SetSkeleton(settings, skeleton);
    JPH_RagdollSettings_ResizeParts(settings, 1);

    JPH_SphereShape* shape = JPH_SphereShape_Create(0.5f);
    JPH_RVec3 position = {1.0f, 2.0f, 3.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_RagdollSettings_SetPartShape(settings, 0, (JPH_Shape*)shape);
    JPH_RagdollSettings_SetPartPosition(settings, 0, &position);
    JPH_RagdollSettings_SetPartRotation(settings, 0, &rotation);
    JPH_RagdollSettings_SetPartMotionType(settings, 0, JPH_MotionType_Dynamic);
    JPH_RagdollSettings_SetPartObjectLayer(settings, 0, ObjectLayers::MOVING);

    JPH_Ragdoll* ragdoll = JPH_RagdollSettings_CreateRagdoll(settings, physicsSystem, 0, 0);
    JPH_Ragdoll_AddToPhysicsSystem(ragdoll, JPH_Activation_Activate, true);

    JPH_RVec3 outPosition;
    JPH_Quat outRotation;
    JPH_Ragdoll_GetRootTransform(ragdoll, &outPosition, &outRotation, true);

    EXPECT_FLOAT_EQ(outPosition.x, 1.0f);
    EXPECT_FLOAT_EQ(outPosition.y, 2.0f);
    EXPECT_FLOAT_EQ(outPosition.z, 3.0f);

    JPH_Ragdoll_RemoveFromPhysicsSystem(ragdoll, true);
    JPH_Ragdoll_Destroy(ragdoll);
    JPH_RagdollSettings_Destroy(settings);
    JPH_Shape_Destroy((JPH_Shape*)shape);
    JPH_Skeleton_Destroy(skeleton);
}

// ============================================================================
// SkeletonMapper Tests
// ============================================================================

TEST_F(SkeletonTest, SkeletonMapper_Create) {
    JPH_SkeletonMapper* mapper = JPH_SkeletonMapper_Create();
    ASSERT_NE(mapper, nullptr);

    JPH_SkeletonMapper_Destroy(mapper);
}
