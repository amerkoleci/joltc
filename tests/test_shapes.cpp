// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

#include <gtest/gtest.h>
#include "joltc.h"

class ShapesTest : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_TRUE(JPH_Init());
    }
    void TearDown() override {
        JPH_Shutdown();
    }
};

// ============================================================================
// SphereShape Tests
// ============================================================================

TEST_F(ShapesTest, SphereShape_Create) {
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(shape, nullptr);

    float radius = JPH_SphereShape_GetRadius(shape);
    EXPECT_FLOAT_EQ(radius, 1.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(ShapesTest, SphereShape_CreateWithSettings) {
    JPH_SphereShapeSettings* settings = JPH_SphereShapeSettings_Create(2.0f);
    ASSERT_NE(settings, nullptr);

    float radius = JPH_SphereShapeSettings_GetRadius(settings);
    EXPECT_FLOAT_EQ(radius, 2.0f);

    JPH_SphereShape* shape = JPH_SphereShapeSettings_CreateShape(settings);
    ASSERT_NE(shape, nullptr);

    radius = JPH_SphereShape_GetRadius(shape);
    EXPECT_FLOAT_EQ(radius, 2.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)settings);
}

TEST_F(ShapesTest, SphereShapeSettings_SetRadius) {
    JPH_SphereShapeSettings* settings = JPH_SphereShapeSettings_Create(1.0f);
    ASSERT_NE(settings, nullptr);

    JPH_SphereShapeSettings_SetRadius(settings, 3.0f);
    float radius = JPH_SphereShapeSettings_GetRadius(settings);
    EXPECT_FLOAT_EQ(radius, 3.0f);

    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)settings);
}

TEST_F(ShapesTest, SphereShape_GetType) {
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(shape, nullptr);

    JPH_ShapeType type = JPH_Shape_GetType((const JPH_Shape*)shape);
    EXPECT_EQ(type, JPH_ShapeType_Convex);

    JPH_ShapeSubType subType = JPH_Shape_GetSubType((const JPH_Shape*)shape);
    EXPECT_EQ(subType, JPH_ShapeSubType_Sphere);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(ShapesTest, SphereShape_GetVolume) {
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(shape, nullptr);

    float volume = JPH_Shape_GetVolume((const JPH_Shape*)shape);
    // Volume of sphere = (4/3) * PI * r^3
    float expectedVolume = (4.0f / 3.0f) * JPH_M_PI * 1.0f * 1.0f * 1.0f;
    EXPECT_NEAR(volume, expectedVolume, 0.001f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

// ============================================================================
// BoxShape Tests
// ============================================================================

TEST_F(ShapesTest, BoxShape_Create) {
    JPH_Vec3 halfExtent = {1.0f, 2.0f, 3.0f};
    JPH_BoxShape* shape = JPH_BoxShape_Create(&halfExtent, JPH_DEFAULT_CONVEX_RADIUS);
    ASSERT_NE(shape, nullptr);

    JPH_Vec3 result;
    JPH_BoxShape_GetHalfExtent(shape, &result);
    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
    EXPECT_FLOAT_EQ(result.z, 3.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(ShapesTest, BoxShape_CreateWithSettings) {
    JPH_Vec3 halfExtent = {2.0f, 2.0f, 2.0f};
    JPH_BoxShapeSettings* settings = JPH_BoxShapeSettings_Create(&halfExtent, JPH_DEFAULT_CONVEX_RADIUS);
    ASSERT_NE(settings, nullptr);

    JPH_BoxShape* shape = JPH_BoxShapeSettings_CreateShape(settings);
    ASSERT_NE(shape, nullptr);

    JPH_Vec3 result;
    JPH_BoxShape_GetHalfExtent(shape, &result);
    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
    EXPECT_FLOAT_EQ(result.z, 2.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)settings);
}

TEST_F(ShapesTest, BoxShape_GetConvexRadius) {
    JPH_Vec3 halfExtent = {1.0f, 1.0f, 1.0f};
    JPH_BoxShape* shape = JPH_BoxShape_Create(&halfExtent, 0.1f);
    ASSERT_NE(shape, nullptr);

    float convexRadius = JPH_BoxShape_GetConvexRadius(shape);
    EXPECT_FLOAT_EQ(convexRadius, 0.1f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(ShapesTest, BoxShape_GetType) {
    JPH_Vec3 halfExtent = {1.0f, 1.0f, 1.0f};
    JPH_BoxShape* shape = JPH_BoxShape_Create(&halfExtent, JPH_DEFAULT_CONVEX_RADIUS);
    ASSERT_NE(shape, nullptr);

    JPH_ShapeType type = JPH_Shape_GetType((const JPH_Shape*)shape);
    EXPECT_EQ(type, JPH_ShapeType_Convex);

    JPH_ShapeSubType subType = JPH_Shape_GetSubType((const JPH_Shape*)shape);
    EXPECT_EQ(subType, JPH_ShapeSubType_Box);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

// ============================================================================
// CapsuleShape Tests
// ============================================================================

TEST_F(ShapesTest, CapsuleShape_Create) {
    JPH_CapsuleShape* shape = JPH_CapsuleShape_Create(1.0f, 0.5f);
    ASSERT_NE(shape, nullptr);

    float halfHeight = JPH_CapsuleShape_GetHalfHeightOfCylinder(shape);
    float radius = JPH_CapsuleShape_GetRadius(shape);
    EXPECT_FLOAT_EQ(halfHeight, 1.0f);
    EXPECT_FLOAT_EQ(radius, 0.5f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(ShapesTest, CapsuleShape_CreateWithSettings) {
    JPH_CapsuleShapeSettings* settings = JPH_CapsuleShapeSettings_Create(2.0f, 1.0f);
    ASSERT_NE(settings, nullptr);

    JPH_CapsuleShape* shape = JPH_CapsuleShapeSettings_CreateShape(settings);
    ASSERT_NE(shape, nullptr);

    float halfHeight = JPH_CapsuleShape_GetHalfHeightOfCylinder(shape);
    float radius = JPH_CapsuleShape_GetRadius(shape);
    EXPECT_FLOAT_EQ(halfHeight, 2.0f);
    EXPECT_FLOAT_EQ(radius, 1.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)settings);
}

TEST_F(ShapesTest, CapsuleShape_GetType) {
    JPH_CapsuleShape* shape = JPH_CapsuleShape_Create(1.0f, 0.5f);
    ASSERT_NE(shape, nullptr);

    JPH_ShapeSubType subType = JPH_Shape_GetSubType((const JPH_Shape*)shape);
    EXPECT_EQ(subType, JPH_ShapeSubType_Capsule);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

// ============================================================================
// CylinderShape Tests
// ============================================================================

TEST_F(ShapesTest, CylinderShape_Create) {
    JPH_CylinderShape* shape = JPH_CylinderShape_Create(1.0f, 0.5f);
    ASSERT_NE(shape, nullptr);

    float halfHeight = JPH_CylinderShape_GetHalfHeight(shape);
    float radius = JPH_CylinderShape_GetRadius(shape);
    EXPECT_FLOAT_EQ(halfHeight, 1.0f);
    EXPECT_FLOAT_EQ(radius, 0.5f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(ShapesTest, CylinderShape_CreateWithSettings) {
    JPH_CylinderShapeSettings* settings = JPH_CylinderShapeSettings_Create(2.0f, 1.0f, JPH_DEFAULT_CONVEX_RADIUS);
    ASSERT_NE(settings, nullptr);

    JPH_CylinderShape* shape = JPH_CylinderShapeSettings_CreateShape(settings);
    ASSERT_NE(shape, nullptr);

    float halfHeight = JPH_CylinderShape_GetHalfHeight(shape);
    float radius = JPH_CylinderShape_GetRadius(shape);
    EXPECT_FLOAT_EQ(halfHeight, 2.0f);
    EXPECT_FLOAT_EQ(radius, 1.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)settings);
}

TEST_F(ShapesTest, CylinderShape_GetType) {
    JPH_CylinderShape* shape = JPH_CylinderShape_Create(1.0f, 0.5f);
    ASSERT_NE(shape, nullptr);

    JPH_ShapeSubType subType = JPH_Shape_GetSubType((const JPH_Shape*)shape);
    EXPECT_EQ(subType, JPH_ShapeSubType_Cylinder);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

// ============================================================================
// TriangleShape Tests
// ============================================================================

TEST_F(ShapesTest, TriangleShape_Create) {
    JPH_Vec3 v1 = {0.0f, 0.0f, 0.0f};
    JPH_Vec3 v2 = {1.0f, 0.0f, 0.0f};
    JPH_Vec3 v3 = {0.0f, 1.0f, 0.0f};

    JPH_TriangleShape* shape = JPH_TriangleShape_Create(&v1, &v2, &v3, 0.0f);
    ASSERT_NE(shape, nullptr);

    JPH_Vec3 result;
    JPH_TriangleShape_GetVertex1(shape, &result);
    EXPECT_FLOAT_EQ(result.x, 0.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);

    JPH_TriangleShape_GetVertex2(shape, &result);
    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);

    JPH_TriangleShape_GetVertex3(shape, &result);
    EXPECT_FLOAT_EQ(result.x, 0.0f);
    EXPECT_FLOAT_EQ(result.y, 1.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(ShapesTest, TriangleShape_GetType) {
    JPH_Vec3 v1 = {0.0f, 0.0f, 0.0f};
    JPH_Vec3 v2 = {1.0f, 0.0f, 0.0f};
    JPH_Vec3 v3 = {0.0f, 1.0f, 0.0f};

    JPH_TriangleShape* shape = JPH_TriangleShape_Create(&v1, &v2, &v3, 0.0f);
    ASSERT_NE(shape, nullptr);

    JPH_ShapeSubType subType = JPH_Shape_GetSubType((const JPH_Shape*)shape);
    EXPECT_EQ(subType, JPH_ShapeSubType_Triangle);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

// ============================================================================
// TaperedCapsuleShape Tests
// ============================================================================

TEST_F(ShapesTest, TaperedCapsuleShape_Create) {
    JPH_TaperedCapsuleShapeSettings* settings = JPH_TaperedCapsuleShapeSettings_Create(1.0f, 0.5f, 0.3f);
    ASSERT_NE(settings, nullptr);

    JPH_TaperedCapsuleShape* shape = JPH_TaperedCapsuleShapeSettings_CreateShape(settings);
    ASSERT_NE(shape, nullptr);

    float topRadius = JPH_TaperedCapsuleShape_GetTopRadius(shape);
    float bottomRadius = JPH_TaperedCapsuleShape_GetBottomRadius(shape);
    float halfHeight = JPH_TaperedCapsuleShape_GetHalfHeight(shape);

    EXPECT_FLOAT_EQ(topRadius, 0.5f);
    EXPECT_FLOAT_EQ(bottomRadius, 0.3f);
    EXPECT_FLOAT_EQ(halfHeight, 1.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)settings);
}

TEST_F(ShapesTest, TaperedCapsuleShape_GetType) {
    JPH_TaperedCapsuleShapeSettings* settings = JPH_TaperedCapsuleShapeSettings_Create(1.0f, 0.5f, 0.3f);
    ASSERT_NE(settings, nullptr);

    JPH_TaperedCapsuleShape* shape = JPH_TaperedCapsuleShapeSettings_CreateShape(settings);
    ASSERT_NE(shape, nullptr);

    JPH_ShapeSubType subType = JPH_Shape_GetSubType((const JPH_Shape*)shape);
    EXPECT_EQ(subType, JPH_ShapeSubType_TaperedCapsule);

    JPH_Shape_Destroy((JPH_Shape*)shape);
    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)settings);
}

// ============================================================================
// ConvexHullShape Tests
// ============================================================================

TEST_F(ShapesTest, ConvexHullShape_Create) {
    // Create a simple tetrahedron
    JPH_Vec3 points[] = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.5f, 1.0f, 0.0f},
        {0.5f, 0.5f, 1.0f}
    };

    JPH_ConvexHullShapeSettings* settings = JPH_ConvexHullShapeSettings_Create(points, 4, JPH_DEFAULT_CONVEX_RADIUS);
    ASSERT_NE(settings, nullptr);

    JPH_ConvexHullShape* shape = JPH_ConvexHullShapeSettings_CreateShape(settings);
    ASSERT_NE(shape, nullptr);

    uint32_t numPoints = JPH_ConvexHullShape_GetNumPoints(shape);
    EXPECT_EQ(numPoints, 4u);

    uint32_t numFaces = JPH_ConvexHullShape_GetNumFaces(shape);
    EXPECT_EQ(numFaces, 4u); // Tetrahedron has 4 faces

    JPH_Shape_Destroy((JPH_Shape*)shape);
    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)settings);
}

TEST_F(ShapesTest, ConvexHullShape_GetPoint) {
    JPH_Vec3 points[] = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.5f, 1.0f, 0.0f},
        {0.5f, 0.5f, 1.0f}
    };

    JPH_ConvexHullShapeSettings* settings = JPH_ConvexHullShapeSettings_Create(points, 4, JPH_DEFAULT_CONVEX_RADIUS);
    JPH_ConvexHullShape* shape = JPH_ConvexHullShapeSettings_CreateShape(settings);
    ASSERT_NE(shape, nullptr);

    JPH_Vec3 result;
    JPH_ConvexHullShape_GetPoint(shape, 0, &result);
    // Points may be reordered, so just check they're valid
    EXPECT_FALSE(std::isnan(result.x));
    EXPECT_FALSE(std::isnan(result.y));
    EXPECT_FALSE(std::isnan(result.z));

    JPH_Shape_Destroy((JPH_Shape*)shape);
    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)settings);
}

// ============================================================================
// PlaneShape Tests
// ============================================================================

TEST_F(ShapesTest, PlaneShape_Create) {
    JPH_Plane plane = {{0.0f, 1.0f, 0.0f}, 0.0f}; // XZ plane at origin
    JPH_PlaneShape* shape = JPH_PlaneShape_Create(&plane, nullptr, 1000.0f);
    ASSERT_NE(shape, nullptr);

    JPH_Plane result;
    JPH_PlaneShape_GetPlane(shape, &result);
    EXPECT_FLOAT_EQ(result.normal.x, 0.0f);
    EXPECT_FLOAT_EQ(result.normal.y, 1.0f);
    EXPECT_FLOAT_EQ(result.normal.z, 0.0f);
    EXPECT_FLOAT_EQ(result.distance, 0.0f);

    float halfExtent = JPH_PlaneShape_GetHalfExtent(shape);
    EXPECT_FLOAT_EQ(halfExtent, 1000.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

// ============================================================================
// Shape Common API Tests
// ============================================================================

TEST_F(ShapesTest, Shape_UserData) {
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(shape, nullptr);

    JPH_Shape_SetUserData((JPH_Shape*)shape, 12345);
    uint64_t userData = JPH_Shape_GetUserData((const JPH_Shape*)shape);
    EXPECT_EQ(userData, 12345u);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(ShapesTest, Shape_GetLocalBounds) {
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(shape, nullptr);

    JPH_AABox bounds;
    JPH_Shape_GetLocalBounds((const JPH_Shape*)shape, &bounds);

    // Sphere of radius 1 should have bounds from (-1,-1,-1) to (1,1,1)
    EXPECT_FLOAT_EQ(bounds.min.x, -1.0f);
    EXPECT_FLOAT_EQ(bounds.min.y, -1.0f);
    EXPECT_FLOAT_EQ(bounds.min.z, -1.0f);
    EXPECT_FLOAT_EQ(bounds.max.x, 1.0f);
    EXPECT_FLOAT_EQ(bounds.max.y, 1.0f);
    EXPECT_FLOAT_EQ(bounds.max.z, 1.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(ShapesTest, Shape_GetCenterOfMass) {
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(shape, nullptr);

    JPH_Vec3 com;
    JPH_Shape_GetCenterOfMass((const JPH_Shape*)shape, &com);

    // Center of mass for sphere is at origin
    EXPECT_FLOAT_EQ(com.x, 0.0f);
    EXPECT_FLOAT_EQ(com.y, 0.0f);
    EXPECT_FLOAT_EQ(com.z, 0.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(ShapesTest, Shape_GetInnerRadius) {
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(shape, nullptr);

    float innerRadius = JPH_Shape_GetInnerRadius((const JPH_Shape*)shape);
    EXPECT_FLOAT_EQ(innerRadius, 1.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(ShapesTest, Shape_GetMassProperties) {
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(shape, nullptr);

    JPH_MassProperties massProps;
    JPH_Shape_GetMassProperties((const JPH_Shape*)shape, &massProps);

    // Mass should be positive (depends on density)
    EXPECT_GT(massProps.mass, 0.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(ShapesTest, Shape_MustBeStatic) {
    // Regular sphere doesn't need to be static
    JPH_SphereShape* sphere = JPH_SphereShape_Create(1.0f);
    EXPECT_FALSE(JPH_Shape_MustBeStatic((const JPH_Shape*)sphere));
    JPH_Shape_Destroy((JPH_Shape*)sphere);

    // Plane must be static
    JPH_Plane plane = {{0.0f, 1.0f, 0.0f}, 0.0f};
    JPH_PlaneShape* planeShape = JPH_PlaneShape_Create(&plane, nullptr, 1000.0f);
    EXPECT_TRUE(JPH_Shape_MustBeStatic((const JPH_Shape*)planeShape));
    JPH_Shape_Destroy((JPH_Shape*)planeShape);
}

// ============================================================================
// Convex Shape Density Tests
// ============================================================================

TEST_F(ShapesTest, ConvexShape_Density) {
    JPH_SphereShapeSettings* settings = JPH_SphereShapeSettings_Create(1.0f);
    ASSERT_NE(settings, nullptr);

    // Get default density
    float defaultDensity = JPH_ConvexShapeSettings_GetDensity((const JPH_ConvexShapeSettings*)settings);
    EXPECT_GT(defaultDensity, 0.0f);

    // Set new density
    JPH_ConvexShapeSettings_SetDensity((JPH_ConvexShapeSettings*)settings, 2000.0f);
    float newDensity = JPH_ConvexShapeSettings_GetDensity((const JPH_ConvexShapeSettings*)settings);
    EXPECT_FLOAT_EQ(newDensity, 2000.0f);

    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)settings);
}

TEST_F(ShapesTest, ConvexShape_SetDensity) {
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(shape, nullptr);

    // Get default density
    float defaultDensity = JPH_ConvexShape_GetDensity((const JPH_ConvexShape*)shape);
    EXPECT_GT(defaultDensity, 0.0f);

    // Set new density
    JPH_ConvexShape_SetDensity((JPH_ConvexShape*)shape, 5000.0f);
    float newDensity = JPH_ConvexShape_GetDensity((const JPH_ConvexShape*)shape);
    EXPECT_FLOAT_EQ(newDensity, 5000.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

// ============================================================================
// ShapeSettings UserData Tests
// ============================================================================

TEST_F(ShapesTest, ShapeSettings_UserData) {
    JPH_SphereShapeSettings* settings = JPH_SphereShapeSettings_Create(1.0f);
    ASSERT_NE(settings, nullptr);

    JPH_ShapeSettings_SetUserData((JPH_ShapeSettings*)settings, 99999);
    uint64_t userData = JPH_ShapeSettings_GetUserData((const JPH_ShapeSettings*)settings);
    EXPECT_EQ(userData, 99999u);

    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)settings);
}

// ============================================================================
// Decorated Shape Tests
// ============================================================================

TEST_F(ShapesTest, RotatedTranslatedShape_Create) {
    JPH_SphereShape* innerShape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(innerShape, nullptr);

    JPH_Vec3 position = {1.0f, 2.0f, 3.0f};
    JPH_Quat rotation = {0.0f, 0.0f, 0.0f, 1.0f}; // Identity

    JPH_RotatedTranslatedShape* shape = JPH_RotatedTranslatedShape_Create(&position, &rotation, (const JPH_Shape*)innerShape);
    ASSERT_NE(shape, nullptr);

    JPH_Vec3 resultPos;
    JPH_RotatedTranslatedShape_GetPosition(shape, &resultPos);
    EXPECT_FLOAT_EQ(resultPos.x, 1.0f);
    EXPECT_FLOAT_EQ(resultPos.y, 2.0f);
    EXPECT_FLOAT_EQ(resultPos.z, 3.0f);

    JPH_Quat resultRot;
    JPH_RotatedTranslatedShape_GetRotation(shape, &resultRot);
    EXPECT_FLOAT_EQ(resultRot.w, 1.0f);

    // Get inner shape
    const JPH_Shape* inner = JPH_DecoratedShape_GetInnerShape((const JPH_DecoratedShape*)shape);
    EXPECT_EQ(inner, (const JPH_Shape*)innerShape);

    JPH_Shape_Destroy((JPH_Shape*)shape);
    // Note: innerShape is owned by the decorated shape and will be destroyed with it
}

TEST_F(ShapesTest, ScaledShape_Create) {
    JPH_SphereShape* innerShape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(innerShape, nullptr);

    JPH_Vec3 scale = {2.0f, 2.0f, 2.0f};
    JPH_ScaledShape* shape = JPH_ScaledShape_Create((const JPH_Shape*)innerShape, &scale);
    ASSERT_NE(shape, nullptr);

    JPH_Vec3 resultScale;
    JPH_ScaledShape_GetScale(shape, &resultScale);
    EXPECT_FLOAT_EQ(resultScale.x, 2.0f);
    EXPECT_FLOAT_EQ(resultScale.y, 2.0f);
    EXPECT_FLOAT_EQ(resultScale.z, 2.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(ShapesTest, OffsetCenterOfMassShape_Create) {
    JPH_SphereShape* innerShape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(innerShape, nullptr);

    JPH_Vec3 offset = {0.0f, 1.0f, 0.0f};
    JPH_OffsetCenterOfMassShape* shape = JPH_OffsetCenterOfMassShape_Create(&offset, (const JPH_Shape*)innerShape);
    ASSERT_NE(shape, nullptr);

    JPH_Vec3 resultOffset;
    JPH_OffsetCenterOfMassShape_GetOffset(shape, &resultOffset);
    EXPECT_FLOAT_EQ(resultOffset.x, 0.0f);
    EXPECT_FLOAT_EQ(resultOffset.y, 1.0f);
    EXPECT_FLOAT_EQ(resultOffset.z, 0.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

// ============================================================================
// Compound Shape Tests
// ============================================================================

TEST_F(ShapesTest, StaticCompoundShape_Create) {
    JPH_StaticCompoundShapeSettings* settings = JPH_StaticCompoundShapeSettings_Create();
    ASSERT_NE(settings, nullptr);

    // Create two sphere shapes
    JPH_SphereShape* sphere1 = JPH_SphereShape_Create(1.0f);
    JPH_SphereShape* sphere2 = JPH_SphereShape_Create(0.5f);

    JPH_Vec3 pos1 = {-1.0f, 0.0f, 0.0f};
    JPH_Vec3 pos2 = {1.0f, 0.0f, 0.0f};
    JPH_Quat rot = {0.0f, 0.0f, 0.0f, 1.0f};

    JPH_CompoundShapeSettings_AddShape2((JPH_CompoundShapeSettings*)settings, &pos1, &rot, (const JPH_Shape*)sphere1, 0);
    JPH_CompoundShapeSettings_AddShape2((JPH_CompoundShapeSettings*)settings, &pos2, &rot, (const JPH_Shape*)sphere2, 0);

    JPH_StaticCompoundShape* shape = JPH_StaticCompoundShape_Create(settings);
    ASSERT_NE(shape, nullptr);

    uint32_t numSubShapes = JPH_CompoundShape_GetNumSubShapes((const JPH_CompoundShape*)shape);
    EXPECT_EQ(numSubShapes, 2u);

    JPH_Shape_Destroy((JPH_Shape*)shape);
    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)settings);
}

TEST_F(ShapesTest, MutableCompoundShape_Create) {
    JPH_MutableCompoundShapeSettings* settings = JPH_MutableCompoundShapeSettings_Create();
    ASSERT_NE(settings, nullptr);

    JPH_MutableCompoundShape* shape = JPH_MutableCompoundShape_Create(settings);
    ASSERT_NE(shape, nullptr);

    // Add a sphere
    JPH_SphereShape* sphere = JPH_SphereShape_Create(1.0f);
    JPH_Vec3 pos = {0.0f, 0.0f, 0.0f};
    JPH_Quat rot = {0.0f, 0.0f, 0.0f, 1.0f};

    uint32_t index = JPH_MutableCompoundShape_AddShape(shape, &pos, &rot, (const JPH_Shape*)sphere, 0, UINT32_MAX);
    EXPECT_EQ(index, 0u);

    uint32_t numSubShapes = JPH_CompoundShape_GetNumSubShapes((const JPH_CompoundShape*)shape);
    EXPECT_EQ(numSubShapes, 1u);

    // Remove the shape
    JPH_MutableCompoundShape_RemoveShape(shape, 0);
    numSubShapes = JPH_CompoundShape_GetNumSubShapes((const JPH_CompoundShape*)shape);
    EXPECT_EQ(numSubShapes, 0u);

    JPH_Shape_Destroy((JPH_Shape*)shape);
    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)settings);
}

// ============================================================================
// EmptyShape Tests
// ============================================================================

TEST_F(ShapesTest, EmptyShape_Create) {
    JPH_Vec3 centerOfMass = {0.0f, 0.0f, 0.0f};
    JPH_EmptyShapeSettings* settings = JPH_EmptyShapeSettings_Create(&centerOfMass);
    ASSERT_NE(settings, nullptr);

    JPH_EmptyShape* shape = JPH_EmptyShapeSettings_CreateShape(settings);
    ASSERT_NE(shape, nullptr);

    // Empty shape should have zero volume
    float volume = JPH_Shape_GetVolume((const JPH_Shape*)shape);
    EXPECT_FLOAT_EQ(volume, 0.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)settings);
}

// ============================================================================
// Shape RayCast Tests
// ============================================================================

TEST_F(ShapesTest, Shape_CastRay) {
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(shape, nullptr);

    // Cast ray from outside the sphere toward its center
    JPH_Vec3 origin = {-5.0f, 0.0f, 0.0f};
    JPH_Vec3 direction = {10.0f, 0.0f, 0.0f}; // Direction, not normalized
    JPH_RayCastResult hit;

    bool hasHit = JPH_Shape_CastRay((const JPH_Shape*)shape, &origin, &direction, &hit);
    EXPECT_TRUE(hasHit);

    // The hit should be at fraction ~0.4 (when ray hits sphere at x=-1)
    // fraction = (5-1)/10 = 0.4
    EXPECT_NEAR(hit.fraction, 0.4f, 0.01f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(ShapesTest, Shape_CastRay_Miss) {
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(shape, nullptr);

    // Cast ray that misses the sphere
    JPH_Vec3 origin = {0.0f, 5.0f, 0.0f};
    JPH_Vec3 direction = {1.0f, 0.0f, 0.0f}; // Parallel to X axis, above sphere
    JPH_RayCastResult hit;

    bool hasHit = JPH_Shape_CastRay((const JPH_Shape*)shape, &origin, &direction, &hit);
    EXPECT_FALSE(hasHit);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

// ============================================================================
// Shape CollidePoint Tests
// ============================================================================

TEST_F(ShapesTest, Shape_CollidePoint_Inside) {
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(shape, nullptr);

    // Point inside sphere
    JPH_Vec3 point = {0.0f, 0.0f, 0.0f};
    bool isInside = JPH_Shape_CollidePoint((const JPH_Shape*)shape, &point, nullptr);
    EXPECT_TRUE(isInside);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(ShapesTest, Shape_CollidePoint_Outside) {
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(shape, nullptr);

    // Point outside sphere
    JPH_Vec3 point = {5.0f, 0.0f, 0.0f};
    bool isInside = JPH_Shape_CollidePoint((const JPH_Shape*)shape, &point, nullptr);
    EXPECT_FALSE(isInside);

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

// ============================================================================
// TaperedCylinderShape Tests
// ============================================================================

TEST_F(ShapesTest, TaperedCylinderShape_Create) {
    JPH_TaperedCylinderShapeSettings* settings = JPH_TaperedCylinderShapeSettings_Create(
        1.0f,  // halfHeight
        0.5f,  // topRadius
        1.0f,  // bottomRadius
        JPH_DEFAULT_CONVEX_RADIUS,
        nullptr
    );
    ASSERT_NE(settings, nullptr);

    JPH_TaperedCylinderShape* shape = JPH_TaperedCylinderShapeSettings_CreateShape(settings);
    ASSERT_NE(shape, nullptr);

    float topRadius = JPH_TaperedCylinderShape_GetTopRadius(shape);
    float bottomRadius = JPH_TaperedCylinderShape_GetBottomRadius(shape);
    float halfHeight = JPH_TaperedCylinderShape_GetHalfHeight(shape);

    EXPECT_FLOAT_EQ(topRadius, 0.5f);
    EXPECT_FLOAT_EQ(bottomRadius, 1.0f);
    EXPECT_FLOAT_EQ(halfHeight, 1.0f);

    JPH_Shape_Destroy((JPH_Shape*)shape);
    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)settings);
}

// ============================================================================
// Shape Scale Validation Tests
// ============================================================================

TEST_F(ShapesTest, Shape_IsValidScale) {
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(shape, nullptr);

    // Uniform scale is valid for sphere
    JPH_Vec3 uniformScale = {2.0f, 2.0f, 2.0f};
    EXPECT_TRUE(JPH_Shape_IsValidScale((const JPH_Shape*)shape, &uniformScale));

    // Non-uniform scale is NOT valid for sphere (spheres only support uniform scale)
    JPH_Vec3 nonUniformScale = {2.0f, 1.0f, 1.0f};
    EXPECT_FALSE(JPH_Shape_IsValidScale((const JPH_Shape*)shape, &nonUniformScale));

    JPH_Shape_Destroy((JPH_Shape*)shape);
}

TEST_F(ShapesTest, Shape_MakeScaleValid) {
    JPH_SphereShape* shape = JPH_SphereShape_Create(1.0f);
    ASSERT_NE(shape, nullptr);

    JPH_Vec3 nonUniformScale = {2.0f, 1.0f, 1.0f};
    JPH_Vec3 result;
    JPH_Shape_MakeScaleValid((const JPH_Shape*)shape, &nonUniformScale, &result);

    // Result should be a valid (uniform) scale
    EXPECT_TRUE(JPH_Shape_IsValidScale((const JPH_Shape*)shape, &result));

    JPH_Shape_Destroy((JPH_Shape*)shape);
}
