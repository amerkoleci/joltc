// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

#include <gtest/gtest.h>
#include "joltc.h"
#include <cmath>

// Helper for float comparison
constexpr float EPSILON = 1e-5f;

class MathTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

// ============================================================================
// Vec3 Tests
// ============================================================================

TEST_F(MathTest, Vec3_AxisX) {
    JPH_Vec3 result;
    JPH_Vec3_AxisX(&result);
    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);
}

TEST_F(MathTest, Vec3_AxisY) {
    JPH_Vec3 result;
    JPH_Vec3_AxisY(&result);
    EXPECT_FLOAT_EQ(result.x, 0.0f);
    EXPECT_FLOAT_EQ(result.y, 1.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);
}

TEST_F(MathTest, Vec3_AxisZ) {
    JPH_Vec3 result;
    JPH_Vec3_AxisZ(&result);
    EXPECT_FLOAT_EQ(result.x, 0.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 1.0f);
}

TEST_F(MathTest, Vec3_Add) {
    JPH_Vec3 v1 = {1.0f, 2.0f, 3.0f};
    JPH_Vec3 v2 = {4.0f, 5.0f, 6.0f};
    JPH_Vec3 result;
    JPH_Vec3_Add(&v1, &v2, &result);
    EXPECT_FLOAT_EQ(result.x, 5.0f);
    EXPECT_FLOAT_EQ(result.y, 7.0f);
    EXPECT_FLOAT_EQ(result.z, 9.0f);
}

TEST_F(MathTest, Vec3_Subtract) {
    JPH_Vec3 v1 = {4.0f, 5.0f, 6.0f};
    JPH_Vec3 v2 = {1.0f, 2.0f, 3.0f};
    JPH_Vec3 result;
    JPH_Vec3_Subtract(&v1, &v2, &result);
    EXPECT_FLOAT_EQ(result.x, 3.0f);
    EXPECT_FLOAT_EQ(result.y, 3.0f);
    EXPECT_FLOAT_EQ(result.z, 3.0f);
}

TEST_F(MathTest, Vec3_Multiply) {
    JPH_Vec3 v1 = {1.0f, 2.0f, 3.0f};
    JPH_Vec3 v2 = {2.0f, 3.0f, 4.0f};
    JPH_Vec3 result;
    JPH_Vec3_Multiply(&v1, &v2, &result);
    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 6.0f);
    EXPECT_FLOAT_EQ(result.z, 12.0f);
}

TEST_F(MathTest, Vec3_MultiplyScalar) {
    JPH_Vec3 v = {1.0f, 2.0f, 3.0f};
    JPH_Vec3 result;
    JPH_Vec3_MultiplyScalar(&v, 2.0f, &result);
    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 4.0f);
    EXPECT_FLOAT_EQ(result.z, 6.0f);
}

TEST_F(MathTest, Vec3_Divide) {
    JPH_Vec3 v1 = {4.0f, 6.0f, 8.0f};
    JPH_Vec3 v2 = {2.0f, 3.0f, 4.0f};
    JPH_Vec3 result;
    JPH_Vec3_Divide(&v1, &v2, &result);
    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
    EXPECT_FLOAT_EQ(result.z, 2.0f);
}

TEST_F(MathTest, Vec3_DivideScalar) {
    JPH_Vec3 v = {2.0f, 4.0f, 6.0f};
    JPH_Vec3 result;
    JPH_Vec3_DivideScalar(&v, 2.0f, &result);
    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
    EXPECT_FLOAT_EQ(result.z, 3.0f);
}

TEST_F(MathTest, Vec3_Length) {
    JPH_Vec3 v = {3.0f, 4.0f, 0.0f};
    float length = JPH_Vec3_Length(&v);
    EXPECT_FLOAT_EQ(length, 5.0f);
}

TEST_F(MathTest, Vec3_LengthSquared) {
    JPH_Vec3 v = {3.0f, 4.0f, 0.0f};
    float lengthSq = JPH_Vec3_LengthSquared(&v);
    EXPECT_FLOAT_EQ(lengthSq, 25.0f);
}

TEST_F(MathTest, Vec3_Normalized) {
    JPH_Vec3 v = {3.0f, 0.0f, 0.0f};
    JPH_Vec3 result;
    JPH_Vec3_Normalized(&v, &result);
    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);
}

TEST_F(MathTest, Vec3_DotProduct) {
    JPH_Vec3 v1 = {1.0f, 2.0f, 3.0f};
    JPH_Vec3 v2 = {4.0f, 5.0f, 6.0f};
    float result;
    JPH_Vec3_DotProduct(&v1, &v2, &result);
    EXPECT_FLOAT_EQ(result, 32.0f); // 1*4 + 2*5 + 3*6 = 32
}

TEST_F(MathTest, Vec3_Cross) {
    JPH_Vec3 v1 = {1.0f, 0.0f, 0.0f};
    JPH_Vec3 v2 = {0.0f, 1.0f, 0.0f};
    JPH_Vec3 result;
    JPH_Vec3_Cross(&v1, &v2, &result);
    EXPECT_FLOAT_EQ(result.x, 0.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 1.0f);
}

TEST_F(MathTest, Vec3_Negate) {
    JPH_Vec3 v = {1.0f, -2.0f, 3.0f};
    JPH_Vec3 result;
    JPH_Vec3_Negate(&v, &result);
    EXPECT_FLOAT_EQ(result.x, -1.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
    EXPECT_FLOAT_EQ(result.z, -3.0f);
}

TEST_F(MathTest, Vec3_Abs) {
    JPH_Vec3 v = {-1.0f, -2.0f, 3.0f};
    JPH_Vec3 result;
    JPH_Vec3_Abs(&v, &result);
    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
    EXPECT_FLOAT_EQ(result.z, 3.0f);
}

TEST_F(MathTest, Vec3_IsClose) {
    JPH_Vec3 v1 = {1.0f, 2.0f, 3.0f};
    JPH_Vec3 v2 = {1.0f, 2.0f, 3.0f};
    EXPECT_TRUE(JPH_Vec3_IsClose(&v1, &v2, 0.001f));

    JPH_Vec3 v3 = {1.1f, 2.1f, 3.1f};
    EXPECT_FALSE(JPH_Vec3_IsClose(&v1, &v3, 0.001f));
}

TEST_F(MathTest, Vec3_IsNearZero) {
    JPH_Vec3 v1 = {0.0f, 0.0f, 0.0f};
    EXPECT_TRUE(JPH_Vec3_IsNearZero(&v1, 0.001f));

    JPH_Vec3 v2 = {0.0001f, 0.0001f, 0.0001f};
    EXPECT_TRUE(JPH_Vec3_IsNearZero(&v2, 0.001f));

    JPH_Vec3 v3 = {1.0f, 0.0f, 0.0f};
    EXPECT_FALSE(JPH_Vec3_IsNearZero(&v3, 0.001f));
}

TEST_F(MathTest, Vec3_IsNormalized) {
    JPH_Vec3 v1 = {1.0f, 0.0f, 0.0f};
    EXPECT_TRUE(JPH_Vec3_IsNormalized(&v1, 0.001f));

    JPH_Vec3 v2 = {2.0f, 0.0f, 0.0f};
    EXPECT_FALSE(JPH_Vec3_IsNormalized(&v2, 0.001f));
}

TEST_F(MathTest, Vec3_IsNaN) {
    JPH_Vec3 v1 = {1.0f, 2.0f, 3.0f};
    EXPECT_FALSE(JPH_Vec3_IsNaN(&v1));

    JPH_Vec3 v2 = {std::nanf(""), 2.0f, 3.0f};
    EXPECT_TRUE(JPH_Vec3_IsNaN(&v2));
}

// ============================================================================
// Quaternion Tests
// ============================================================================

TEST_F(MathTest, Quat_FromEulerAngles) {
    JPH_Vec3 angles = {0.0f, 0.0f, 0.0f}; // No rotation
    JPH_Quat result;
    JPH_Quat_FromEulerAngles(&angles, &result);

    // Identity quaternion
    EXPECT_NEAR(result.x, 0.0f, EPSILON);
    EXPECT_NEAR(result.y, 0.0f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
    EXPECT_NEAR(result.w, 1.0f, EPSILON);
}

TEST_F(MathTest, Quat_GetEulerAngles) {
    // Identity quaternion
    JPH_Quat q = {0.0f, 0.0f, 0.0f, 1.0f};
    JPH_Vec3 result;
    JPH_Quat_GetEulerAngles(&q, &result);

    EXPECT_NEAR(result.x, 0.0f, EPSILON);
    EXPECT_NEAR(result.y, 0.0f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
}

TEST_F(MathTest, Quat_Multiply) {
    // Identity * Identity = Identity
    JPH_Quat q1 = {0.0f, 0.0f, 0.0f, 1.0f};
    JPH_Quat q2 = {0.0f, 0.0f, 0.0f, 1.0f};
    JPH_Quat result;
    JPH_Quat_Multiply(&q1, &q2, &result);

    EXPECT_NEAR(result.x, 0.0f, EPSILON);
    EXPECT_NEAR(result.y, 0.0f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
    EXPECT_NEAR(result.w, 1.0f, EPSILON);
}

TEST_F(MathTest, Quat_Conjugated) {
    JPH_Quat q = {1.0f, 2.0f, 3.0f, 4.0f};
    JPH_Quat result;
    JPH_Quat_Conjugated(&q, &result);

    EXPECT_FLOAT_EQ(result.x, -1.0f);
    EXPECT_FLOAT_EQ(result.y, -2.0f);
    EXPECT_FLOAT_EQ(result.z, -3.0f);
    EXPECT_FLOAT_EQ(result.w, 4.0f);
}

TEST_F(MathTest, Quat_Inversed) {
    // For a unit quaternion, inverse equals conjugate
    JPH_Quat q = {0.0f, 0.0f, 0.0f, 1.0f};
    JPH_Quat result;
    JPH_Quat_Inversed(&q, &result);

    EXPECT_NEAR(result.x, 0.0f, EPSILON);
    EXPECT_NEAR(result.y, 0.0f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
    EXPECT_NEAR(result.w, 1.0f, EPSILON);
}

TEST_F(MathTest, Quat_Dot) {
    JPH_Quat q1 = {1.0f, 0.0f, 0.0f, 0.0f};
    JPH_Quat q2 = {1.0f, 0.0f, 0.0f, 0.0f};
    float result;
    JPH_Quat_Dot(&q1, &q2, &result);
    EXPECT_FLOAT_EQ(result, 1.0f);
}

TEST_F(MathTest, Quat_Add) {
    JPH_Quat q1 = {1.0f, 2.0f, 3.0f, 4.0f};
    JPH_Quat q2 = {5.0f, 6.0f, 7.0f, 8.0f};
    JPH_Quat result;
    JPH_Quat_Add(&q1, &q2, &result);

    EXPECT_FLOAT_EQ(result.x, 6.0f);
    EXPECT_FLOAT_EQ(result.y, 8.0f);
    EXPECT_FLOAT_EQ(result.z, 10.0f);
    EXPECT_FLOAT_EQ(result.w, 12.0f);
}

TEST_F(MathTest, Quat_Subtract) {
    JPH_Quat q1 = {5.0f, 6.0f, 7.0f, 8.0f};
    JPH_Quat q2 = {1.0f, 2.0f, 3.0f, 4.0f};
    JPH_Quat result;
    JPH_Quat_Subtract(&q1, &q2, &result);

    EXPECT_FLOAT_EQ(result.x, 4.0f);
    EXPECT_FLOAT_EQ(result.y, 4.0f);
    EXPECT_FLOAT_EQ(result.z, 4.0f);
    EXPECT_FLOAT_EQ(result.w, 4.0f);
}

TEST_F(MathTest, Quat_MultiplyScalar) {
    JPH_Quat q = {1.0f, 2.0f, 3.0f, 4.0f};
    JPH_Quat result;
    JPH_Quat_MultiplyScalar(&q, 2.0f, &result);

    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 4.0f);
    EXPECT_FLOAT_EQ(result.z, 6.0f);
    EXPECT_FLOAT_EQ(result.w, 8.0f);
}

TEST_F(MathTest, Quat_Rotate) {
    // 90 degree rotation around Y axis
    float angle = JPH_M_PI / 2.0f;
    JPH_Quat q = {0.0f, sinf(angle / 2.0f), 0.0f, cosf(angle / 2.0f)};
    JPH_Vec3 v = {1.0f, 0.0f, 0.0f}; // X axis
    JPH_Vec3 result;
    JPH_Quat_Rotate(&q, &v, &result);

    // X axis rotated 90 degrees around Y should give -Z axis
    EXPECT_NEAR(result.x, 0.0f, EPSILON);
    EXPECT_NEAR(result.y, 0.0f, EPSILON);
    EXPECT_NEAR(result.z, -1.0f, EPSILON);
}

TEST_F(MathTest, Quat_Slerp) {
    JPH_Quat q1 = {0.0f, 0.0f, 0.0f, 1.0f}; // Identity
    JPH_Quat q2 = {0.0f, 0.0f, 0.0f, 1.0f}; // Identity
    JPH_Quat result;
    JPH_Quat_Slerp(&q1, &q2, 0.5f, &result);

    // Slerp between two identity quaternions should give identity
    EXPECT_NEAR(result.x, 0.0f, EPSILON);
    EXPECT_NEAR(result.y, 0.0f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
    EXPECT_NEAR(result.w, 1.0f, EPSILON);
}

TEST_F(MathTest, Quat_Lerp) {
    JPH_Quat q1 = {0.0f, 0.0f, 0.0f, 1.0f}; // Identity
    JPH_Quat q2 = {0.0f, 0.0f, 0.0f, 1.0f}; // Identity
    JPH_Quat result;
    JPH_Quat_Lerp(&q1, &q2, 0.5f, &result);

    EXPECT_NEAR(result.x, 0.0f, EPSILON);
    EXPECT_NEAR(result.y, 0.0f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
    EXPECT_NEAR(result.w, 1.0f, EPSILON);
}

// ============================================================================
// Mat4 Tests
// ============================================================================

TEST_F(MathTest, Mat4_Identity) {
    JPH_Mat4 result;
    JPH_Mat4_Identity(&result);

    // Check diagonal is 1
    EXPECT_FLOAT_EQ(result.column[0].x, 1.0f);
    EXPECT_FLOAT_EQ(result.column[1].y, 1.0f);
    EXPECT_FLOAT_EQ(result.column[2].z, 1.0f);
    EXPECT_FLOAT_EQ(result.column[3].w, 1.0f);

    // Check off-diagonal is 0
    EXPECT_FLOAT_EQ(result.column[0].y, 0.0f);
    EXPECT_FLOAT_EQ(result.column[0].z, 0.0f);
    EXPECT_FLOAT_EQ(result.column[0].w, 0.0f);
}

TEST_F(MathTest, Mat4_Zero) {
    JPH_Mat4 result;
    JPH_Mat4_Zero(&result);

    for (int i = 0; i < 4; i++) {
        EXPECT_FLOAT_EQ(result.column[i].x, 0.0f);
        EXPECT_FLOAT_EQ(result.column[i].y, 0.0f);
        EXPECT_FLOAT_EQ(result.column[i].z, 0.0f);
        EXPECT_FLOAT_EQ(result.column[i].w, 0.0f);
    }
}

TEST_F(MathTest, Mat4_Translation) {
    JPH_Vec3 translation = {1.0f, 2.0f, 3.0f};
    JPH_Mat4 result;
    JPH_Mat4_Translation(&result, &translation);

    // Check translation column
    EXPECT_FLOAT_EQ(result.column[3].x, 1.0f);
    EXPECT_FLOAT_EQ(result.column[3].y, 2.0f);
    EXPECT_FLOAT_EQ(result.column[3].z, 3.0f);
    EXPECT_FLOAT_EQ(result.column[3].w, 1.0f);

    // Check identity part
    EXPECT_FLOAT_EQ(result.column[0].x, 1.0f);
    EXPECT_FLOAT_EQ(result.column[1].y, 1.0f);
    EXPECT_FLOAT_EQ(result.column[2].z, 1.0f);
}

TEST_F(MathTest, Mat4_Scale) {
    JPH_Vec3 scale = {2.0f, 3.0f, 4.0f};
    JPH_Mat4 result;
    JPH_Mat4_Scale(&result, &scale);

    EXPECT_FLOAT_EQ(result.column[0].x, 2.0f);
    EXPECT_FLOAT_EQ(result.column[1].y, 3.0f);
    EXPECT_FLOAT_EQ(result.column[2].z, 4.0f);
    EXPECT_FLOAT_EQ(result.column[3].w, 1.0f);
}

TEST_F(MathTest, Mat4_Rotation) {
    JPH_Quat q = {0.0f, 0.0f, 0.0f, 1.0f}; // Identity rotation
    JPH_Mat4 result;
    JPH_Mat4_Rotation(&result, &q);

    // Should be identity matrix
    EXPECT_NEAR(result.column[0].x, 1.0f, EPSILON);
    EXPECT_NEAR(result.column[1].y, 1.0f, EPSILON);
    EXPECT_NEAR(result.column[2].z, 1.0f, EPSILON);
    EXPECT_NEAR(result.column[3].w, 1.0f, EPSILON);
}

TEST_F(MathTest, Mat4_Multiply) {
    JPH_Mat4 m1, m2, result;
    JPH_Mat4_Identity(&m1);
    JPH_Mat4_Identity(&m2);
    JPH_Mat4_Multiply(&m1, &m2, &result);

    // Identity * Identity = Identity
    EXPECT_NEAR(result.column[0].x, 1.0f, EPSILON);
    EXPECT_NEAR(result.column[1].y, 1.0f, EPSILON);
    EXPECT_NEAR(result.column[2].z, 1.0f, EPSILON);
    EXPECT_NEAR(result.column[3].w, 1.0f, EPSILON);
}

TEST_F(MathTest, Mat4_Add) {
    JPH_Mat4 m1, m2, result;
    JPH_Mat4_Identity(&m1);
    JPH_Mat4_Identity(&m2);
    JPH_Mat4_Add(&m1, &m2, &result);

    // Identity + Identity should have 2 on diagonal
    EXPECT_FLOAT_EQ(result.column[0].x, 2.0f);
    EXPECT_FLOAT_EQ(result.column[1].y, 2.0f);
    EXPECT_FLOAT_EQ(result.column[2].z, 2.0f);
    EXPECT_FLOAT_EQ(result.column[3].w, 2.0f);
}

TEST_F(MathTest, Mat4_Subtract) {
    JPH_Mat4 m1, m2, result;
    JPH_Mat4_Identity(&m1);
    JPH_Mat4_Identity(&m2);
    JPH_Mat4_Subtract(&m1, &m2, &result);

    // Identity - Identity = Zero
    for (int i = 0; i < 4; i++) {
        EXPECT_FLOAT_EQ(result.column[i].x, 0.0f);
        EXPECT_FLOAT_EQ(result.column[i].y, 0.0f);
        EXPECT_FLOAT_EQ(result.column[i].z, 0.0f);
        EXPECT_FLOAT_EQ(result.column[i].w, 0.0f);
    }
}

TEST_F(MathTest, Mat4_MultiplyScalar) {
    JPH_Mat4 m, result;
    JPH_Mat4_Identity(&m);
    JPH_Mat4_MultiplyScalar(&m, 2.0f, &result);

    EXPECT_FLOAT_EQ(result.column[0].x, 2.0f);
    EXPECT_FLOAT_EQ(result.column[1].y, 2.0f);
    EXPECT_FLOAT_EQ(result.column[2].z, 2.0f);
    EXPECT_FLOAT_EQ(result.column[3].w, 2.0f);
}

TEST_F(MathTest, Mat4_Transposed) {
    JPH_Mat4 m, result;
    JPH_Mat4_Identity(&m);
    m.column[0].y = 1.0f; // Set [1,0] to 1

    JPH_Mat4_Transposed(&m, &result);

    // After transpose, [0,1] should be 1
    EXPECT_FLOAT_EQ(result.column[1].x, 1.0f);
}

TEST_F(MathTest, Mat4_GetAxisX) {
    JPH_Mat4 m;
    JPH_Mat4_Identity(&m);
    JPH_Vec3 result;
    JPH_Mat4_GetAxisX(&m, &result);

    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);
}

TEST_F(MathTest, Mat4_GetAxisY) {
    JPH_Mat4 m;
    JPH_Mat4_Identity(&m);
    JPH_Vec3 result;
    JPH_Mat4_GetAxisY(&m, &result);

    EXPECT_FLOAT_EQ(result.x, 0.0f);
    EXPECT_FLOAT_EQ(result.y, 1.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);
}

TEST_F(MathTest, Mat4_GetAxisZ) {
    JPH_Mat4 m;
    JPH_Mat4_Identity(&m);
    JPH_Vec3 result;
    JPH_Mat4_GetAxisZ(&m, &result);

    EXPECT_FLOAT_EQ(result.x, 0.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 1.0f);
}

TEST_F(MathTest, Mat4_GetTranslation) {
    JPH_Vec3 translation = {1.0f, 2.0f, 3.0f};
    JPH_Mat4 m;
    JPH_Mat4_Translation(&m, &translation);

    JPH_Vec3 result;
    JPH_Mat4_GetTranslation(&m, &result);

    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
    EXPECT_FLOAT_EQ(result.z, 3.0f);
}

TEST_F(MathTest, Mat4_GetQuaternion) {
    JPH_Quat q = {0.0f, 0.0f, 0.0f, 1.0f}; // Identity
    JPH_Mat4 m;
    JPH_Mat4_Rotation(&m, &q);

    JPH_Quat result;
    JPH_Mat4_GetQuaternion(&m, &result);

    // Should get back identity quaternion (or its negative)
    EXPECT_NEAR(std::abs(result.w), 1.0f, EPSILON);
}

TEST_F(MathTest, Mat4_RotationTranslation) {
    JPH_Quat q = {0.0f, 0.0f, 0.0f, 1.0f}; // Identity rotation
    JPH_Vec3 translation = {1.0f, 2.0f, 3.0f};
    JPH_Mat4 result;
    JPH_Mat4_RotationTranslation(&result, &q, &translation);

    // Check translation
    EXPECT_FLOAT_EQ(result.column[3].x, 1.0f);
    EXPECT_FLOAT_EQ(result.column[3].y, 2.0f);
    EXPECT_FLOAT_EQ(result.column[3].z, 3.0f);

    // Check rotation part is identity
    EXPECT_NEAR(result.column[0].x, 1.0f, EPSILON);
    EXPECT_NEAR(result.column[1].y, 1.0f, EPSILON);
    EXPECT_NEAR(result.column[2].z, 1.0f, EPSILON);
}

// ============================================================================
// Math Utility Tests
// ============================================================================

TEST_F(MathTest, Math_Sin) {
    float result = JPH_Math_Sin(0.0f);
    EXPECT_NEAR(result, 0.0f, EPSILON);

    result = JPH_Math_Sin(JPH_M_PI / 2.0f);
    EXPECT_NEAR(result, 1.0f, EPSILON);
}

TEST_F(MathTest, Math_Cos) {
    float result = JPH_Math_Cos(0.0f);
    EXPECT_NEAR(result, 1.0f, EPSILON);

    result = JPH_Math_Cos(JPH_M_PI);
    EXPECT_NEAR(result, -1.0f, EPSILON);
}

// ============================================================================
// RayCast Helper Tests
// ============================================================================

TEST_F(MathTest, RayCast_GetPointOnRay) {
    JPH_Vec3 origin = {0.0f, 0.0f, 0.0f};
    JPH_Vec3 direction = {1.0f, 0.0f, 0.0f};
    JPH_Vec3 result;

    JPH_RayCast_GetPointOnRay(&origin, &direction, 0.5f, &result);

    EXPECT_FLOAT_EQ(result.x, 0.5f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);
}
