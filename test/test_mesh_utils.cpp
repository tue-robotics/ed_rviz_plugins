#include "visuals/mesh_utils.h"

#include <OgreVector.h>

#include <gtest/gtest.h>

using ed_rviz_plugins::getMinMax;
using ed_rviz_plugins::triangleNormal;

namespace
{

void expectVectorNear(const Ogre::Vector3& actual, const Ogre::Vector3& expected, float tolerance = 1e-6F)
{
    EXPECT_NEAR(actual.x, expected.x, tolerance);
    EXPECT_NEAR(actual.y, expected.y, tolerance);
    EXPECT_NEAR(actual.z, expected.z, tolerance);
}

} // namespace

TEST(triangleNormal, CounterClockwiseInXYPlanePointsUp)
{
    expectVectorNear(triangleNormal({1, 0, 0}, {0, 1, 0}, {0, 0, 0}), {0, 0, 1});
}

TEST(triangleNormal, ClockwiseInXYPlanePointsDown)
{
    expectVectorNear(triangleNormal({0, 1, 0}, {1, 0, 0}, {0, 0, 0}), {0, 0, -1});
}

TEST(triangleNormal, IsUnitLength)
{
    const Ogre::Vector3 normal = triangleNormal({10, 0, 3}, {0, 20, 3}, {0, 0, 3});
    EXPECT_NEAR(normal.length(), 1.0F, 1e-6F);
    expectVectorNear(normal, {0, 0, 1});
}

TEST(triangleNormal, MatchesOgreVectorMath)
{
    const Ogre::Vector3 v1(0.3F, -1.2F, 2.0F);
    const Ogre::Vector3 v2(1.7F, 0.4F, -0.5F);
    const Ogre::Vector3 v3(-0.8F, 0.9F, 1.1F);
    expectVectorNear(triangleNormal(v1, v2, v3), (v1 - v3).crossProduct(v2 - v3).normalisedCopy());
}

TEST(triangleNormal, DegenerateTriangleGivesZeroVector)
{
    expectVectorNear(triangleNormal({0, 0, 0}, {1, 1, 1}, {2, 2, 2}), Ogre::Vector3::ZERO);
    expectVectorNear(triangleNormal({1, 2, 3}, {1, 2, 3}, {1, 2, 3}), Ogre::Vector3::ZERO);
}

TEST(getMinMax, GrowsBoundsPerAxis)
{
    Ogre::Vector3 min_pt(1e9, 1e9, 1e9);
    Ogre::Vector3 max_pt(-1e9, -1e9, -1e9);

    getMinMax({1, -2, 3}, min_pt, max_pt);
    expectVectorNear(min_pt, {1, -2, 3});
    expectVectorNear(max_pt, {1, -2, 3});

    getMinMax({-1, 5, 0}, min_pt, max_pt);
    expectVectorNear(min_pt, {-1, -2, 0});
    expectVectorNear(max_pt, {1, 5, 3});
}

TEST(getMinMax, PointInsideBoundsChangesNothing)
{
    Ogre::Vector3 min_pt(-1, -1, -1);
    Ogre::Vector3 max_pt(1, 1, 1);

    getMinMax({0.5F, 0, -0.5F}, min_pt, max_pt);
    expectVectorNear(min_pt, {-1, -1, -1});
    expectVectorNear(max_pt, {1, 1, 1});
}
