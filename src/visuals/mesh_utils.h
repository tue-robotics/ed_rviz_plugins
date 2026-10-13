#ifndef MESH_UTILS_H
#define MESH_UTILS_H

#include <OgreVector.h>

namespace ed_rviz_plugins
{

/**
 * @brief Grow the axis-aligned bounding box [min_pt, max_pt] to include value
 */
void getMinMax(const Ogre::Vector3& value, Ogre::Vector3& min_pt, Ogre::Vector3& max_pt);

/**
 * @brief Unit normal of the triangle (v1, v2, v3), same as (v1 - v3).crossProduct(v2 - v3).normalisedCopy()
 * @return The zero vector for a degenerate triangle
 */
Ogre::Vector3 triangleNormal(const Ogre::Vector3& v1, const Ogre::Vector3& v2, const Ogre::Vector3& v3);

} // namespace ed_rviz_plugins

#endif // MESH_UTILS_H
