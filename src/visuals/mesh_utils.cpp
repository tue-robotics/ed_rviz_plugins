#include "mesh_utils.h"

#include <OgrePrerequisites.h>

#include <algorithm>
#include <cmath>

namespace ed_rviz_plugins
{

void getMinMax(const Ogre::Vector3& value, Ogre::Vector3& min_pt, Ogre::Vector3& max_pt)
{
    max_pt.x = std::max(max_pt.x, value.x);
    max_pt.y = std::max(max_pt.y, value.y);
    max_pt.z = std::max(max_pt.z, value.z);

    min_pt.x = std::min(min_pt.x, value.x);
    min_pt.y = std::min(min_pt.y, value.y);
    min_pt.z = std::min(min_pt.z, value.z);
}

// Ogre::Vector3's arithmetic operators (operator-, dotProduct, normalise, ...) loop over ptr()[i] starting from &x,
// which clang-analyzer reports as out of bounds (security.ArrayBound) inside OgreVector.h, where it cannot be
// suppressed. So compute the normal per component.
Ogre::Vector3 triangleNormal(const Ogre::Vector3& v1, const Ogre::Vector3& v2, const Ogre::Vector3& v3)
{
    const Ogre::Vector3 edge1(v1.x - v3.x, v1.y - v3.y, v1.z - v3.z);
    const Ogre::Vector3 edge2(v2.x - v3.x, v2.y - v3.y, v2.z - v3.z);
    const Ogre::Vector3 cross = edge1.crossProduct(edge2);
    const Ogre::Real length = std::sqrt((cross.x * cross.x) + (cross.y * cross.y) + (cross.z * cross.z));
    if (length <= 0)
        return cross;
    return {cross.x / length, cross.y / length, cross.z / length};
}

} // namespace ed_rviz_plugins
