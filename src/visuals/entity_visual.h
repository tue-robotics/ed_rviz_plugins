#ifndef ENTITY_VISUAL_H
#define ENTITY_VISUAL_H

#include <ed_gui_server_interfaces/msg/entity_mesh_and_volumes.hpp>
#include <ed_gui_server_interfaces/msg/polygon.hpp>
#include <OGRE/OgreColourValue.h>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/mesh_shape.hpp>
#include <rviz_rendering/objects/movable_text.hpp>

#include <memory>
#include <string>
#include <vector>

namespace Ogre
{
class SceneManager;
class SceneNode;
class Vector3;
class Quaternion;
} // namespace Ogre

namespace ed_rviz_plugins
{

class EntityVisual
{
public:
    EntityVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
    ~EntityVisual();

    void setEntityMeshAndVolumes(const ed_gui_server_interfaces::msg::EntityMeshAndVolumes& mesh_and_volumes);
    void setConvexHull(const ed_gui_server_interfaces::msg::Polygon& polygon);
    void setLabel(const std::string& label);

    void setColor(Ogre::ColourValue color, double entity_label_opacity, double area_opacity, double area_label_opacity);

    void setFramePosition(const Ogre::Vector3& position);
    void setFrameOrientation(const Ogre::Quaternion& orientation);

    unsigned long visualRevision() const { return visual_revision_; }
    unsigned long volumesRevision() const { return volumes_revision_; }

private:
    std::shared_ptr<rviz_rendering::MeshShape> mesh_;
    std::vector<std::shared_ptr<rviz_rendering::MeshShape>> volume_meshes_;
    std::vector<std::shared_ptr<rviz_rendering::MovableText>> volume_labels_;
    unsigned long visual_revision_;
    unsigned long volumes_revision_;

    std::shared_ptr<rviz_rendering::BillboardLine> convex_hull_;
    std::shared_ptr<rviz_rendering::MovableText> label_;

    Ogre::ColourValue color_;
    Ogre::SceneNode* frame_node_;
    Ogre::SceneManager* scene_manager_;
};

} // namespace ed_rviz_plugins

#endif // ENTITY_VISUAL_H
