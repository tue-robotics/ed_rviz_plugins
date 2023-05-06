#ifndef ENTITY_VISUAL_H
#define ENTITY_VISUAL_H

#include <boost/shared_ptr.hpp>

#include <rviz/ogre_helpers/mesh_shape.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/movable_text.h>

#include <ed_gui_server_msgs/EntityMeshAndVolumes.h>
#include <ed_gui_server_msgs/Polygon.h>

namespace Ogre
{
class Vector3;
class Quaternion;
class ColorValue;
}

namespace ed_rviz_plugins
{

class EntityVisual
{
public:
    // Constructor.  Creates the visual stuff and puts it into the scene, but in an unconfigured state.
    EntityVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

    // Destructor.  Removes the visual stuff from the scene.
    virtual ~EntityVisual();

    // Configure the visual to show the data in the message.
    void setEntityMeshAndVolumes(const ed_gui_server_msgs::EntityMeshAndVolumes& mesh_and_areas);
    void setConvexHull (const ed_gui_server_msgs::Polygon& polygon);
    void setLabel (const std::string& label);

    void setColor (Ogre::ColourValue c, double entity_label_opacity, double area_opacity, double area_label_opacity);

    // Set the pose of the coordinate frame the message refers to.
    void setFramePosition(const Ogre::Vector3& position);
    void setFrameOrientation(const Ogre::Quaternion& orientation);

    // Get revisions
    unsigned long visualRevision() const { return visual_revision_; }
    unsigned long volumesRevision() const { return volumes_revision_; }

private:
    // The actual scene objects
    boost::shared_ptr<rviz::MeshShape> mesh_;
    std::vector<boost::shared_ptr<rviz::MeshShape> > volume_meshes_;
    std::vector<boost::shared_ptr<rviz::MovableText> > volume_labels_;
    unsigned long visual_revision_;
    unsigned long volumes_revision_;

    boost::shared_ptr<rviz::BillboardLine> convex_hull_;

    boost::shared_ptr<rviz::MovableText> label_;

    Ogre::ColourValue color_;    

    Ogre::SceneNode* frame_node_;

    Ogre::SceneManager* scene_manager_;
};

}

#endif // ENTITY_VISUAL_H
