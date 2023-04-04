#include <boost/make_shared.hpp>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "entity_visual.h"

namespace ed_rviz_plugins
{

void getMinMax(const Ogre::Vector3& value, Ogre::Vector3& min_pt, Ogre::Vector3& max_pt)
{
  if (value.x > max_pt.x) max_pt.x = value.x;
  if (value.y > max_pt.y) max_pt.y = value.y;
  if (value.z > max_pt.z) max_pt.z = value.z;

  if (value.x < min_pt.x) min_pt.x = value.x;
  if (value.y < min_pt.y) min_pt.y = value.y;
  if (value.z < min_pt.z) min_pt.z = value.z;
}

EntityVisual::EntityVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) : visual_revision_(0), volumes_revision_(0)
{
    scene_manager_ = scene_manager;

    frame_node_ = parent_node->createChildSceneNode();

    mesh_ = boost::make_shared<rviz::MeshShape>(scene_manager_, frame_node_);
    convex_hull_ = boost::make_shared<rviz::BillboardLine>(scene_manager_, frame_node_);
    label_ = boost::make_shared<rviz::MovableText>("unknown");

    frame_node_->attachObject(label_.get());
}

EntityVisual::~EntityVisual()
{
    // Destroy the frame node since we don't need it anymore.
    scene_manager_->destroySceneNode(frame_node_);
}

void EntityVisual::setEntityMeshAndVolumes(const ed_gui_server_msgs::EntityMeshAndVolumes& mesh_and_volumes)
{
    mesh_->clear();

    // Set mesh revision
    if (mesh_and_volumes.visual_revision != 0)
    {
        visual_revision_ = mesh_and_volumes.visual_revision;

        mesh_->estimateVertexCount(mesh_and_volumes.mesh.vertices.size() / 3);
        mesh_->beginTriangles();

        // Set vertices
        for (unsigned int i = 0; i < mesh_and_volumes.mesh.vertices.size() / 3 / 3; ++i)
        {
            unsigned int i9 = 9 * i;

            Ogre::Vector3 v1, v2, v3, n;

            v1.x = mesh_and_volumes.mesh.vertices[i9 + 0];
            v1.y = mesh_and_volumes.mesh.vertices[i9 + 1];
            v1.z = mesh_and_volumes.mesh.vertices[i9 + 2];

            v2.x = mesh_and_volumes.mesh.vertices[i9 + 3];
            v2.y = mesh_and_volumes.mesh.vertices[i9 + 4];
            v2.z = mesh_and_volumes.mesh.vertices[i9 + 5];

            v3.x = mesh_and_volumes.mesh.vertices[i9 + 6];
            v3.y = mesh_and_volumes.mesh.vertices[i9 + 7];
            v3.z = mesh_and_volumes.mesh.vertices[i9 + 8];

            n = (v1-v3).crossProduct(v2-v3).normalisedCopy();

            mesh_->addVertex(v1, n);
            mesh_->addVertex(v2, n);
            mesh_->addVertex(v3, n);
        }

        mesh_->endTriangles();
    }

    if (mesh_and_volumes.volumes_revision != 0)
    {
        volumes_revision_ = mesh_and_volumes.volumes_revision;

        volume_meshes_.clear();
        volume_labels_.clear();

        // Now set the areas
        for (const ed_gui_server_msgs::Volume& v : mesh_and_volumes.volumes)
        {
            volume_labels_.push_back(boost::shared_ptr<rviz::MovableText>(new rviz::MovableText(v.name)));
            boost::shared_ptr<rviz::MovableText> label = volume_labels_.back();
            label->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
            label->setCharacterHeight(0.05);
            label->setCaption(v.name);

            volume_meshes_.push_back(boost::shared_ptr<rviz::MeshShape>(new rviz::MeshShape(scene_manager_, frame_node_)));
            boost::shared_ptr<rviz::MeshShape> mesh = volume_meshes_.back();

            mesh->estimateVertexCount(v.mesh.vertices.size() / 3);
            mesh->beginTriangles();

            Ogre::Vector3 max_pt(-1e9, -1e9, -1e9);
            Ogre::Vector3 min_pt(1e9, 1e9, 1e9);

            // Set vertices
            for (unsigned int i = 0; i < v.mesh.vertices.size() / 3 / 3; ++i)
            {
                unsigned int i9 = 9 * i;

                Ogre::Vector3 v1, v2, v3, n;

                v1.x = v.mesh.vertices[i9 + 0];
                v1.y = v.mesh.vertices[i9 + 1];
                v1.z = v.mesh.vertices[i9 + 2];

                getMinMax(v1, min_pt, max_pt);

                v2.x = v.mesh.vertices[i9 + 3];
                v2.y = v.mesh.vertices[i9 + 4];
                v2.z = v.mesh.vertices[i9 + 5];

                getMinMax(v2, min_pt, max_pt);

                v3.x = v.mesh.vertices[i9 + 6];
                v3.y = v.mesh.vertices[i9 + 7];
                v3.z = v.mesh.vertices[i9 + 8];

                getMinMax(v3, min_pt, max_pt);

                n = (v1-v3).crossProduct(v2-v3).normalisedCopy();

                mesh->addVertex(v1, n);
                mesh->addVertex(v2, n);
                mesh->addVertex(v3, n);
            }

            mesh->endTriangles();

            Ogre::Vector3 label_pos((max_pt.x + min_pt.x) / 2, (max_pt.y + min_pt.y) / 2, max_pt.z);

            frame_node_->createChildSceneNode(label_pos)->attachObject(label.get());
        }
    }
}

void EntityVisual::setColor(Ogre::ColourValue c, double entity_label_opacity, double volume_opacity, double volume_label_opacity)
{
    label_->setColor(Ogre::ColourValue(1.0f, 1.0f, 1.0f, entity_label_opacity));

    for (auto& label : volume_labels_)
        label->setColor(Ogre::ColourValue(1.0f, 1.0f, 1.0f, volume_label_opacity));

    c.a = volume_opacity;
    for (auto& mesh : volume_meshes_)
        mesh->setColor(c);

    c.a = 1.0;
    convex_hull_->setColor(c.r, c.g, c.b, c.a);
    mesh_->setColor(c);
}

void EntityVisual::setConvexHull(const ed_gui_server_msgs::Polygon& polygon)
{    
    convex_hull_->clear();
    convex_hull_->setMaxPointsPerLine(2);
    convex_hull_->setNumLines(polygon.xs.size() * 3);
    convex_hull_->setLineWidth(0.01);

    for (unsigned int i = 0; i < polygon.xs.size(); ++i)
    {        
        int j = (i + 1) % polygon.xs.size();

        float x1 = polygon.xs[i];
        float x2 = polygon.xs[j];

        float y1 = polygon.ys[i];
        float y2 = polygon.ys[j];

        // Low line
        if (i != 0)
            convex_hull_->newLine();
        convex_hull_->addPoint(Ogre::Vector3(x1, y1, polygon.z_min));
        convex_hull_->addPoint(Ogre::Vector3(x2, y2, polygon.z_min));

        // High line
        convex_hull_->newLine();
        convex_hull_->addPoint(Ogre::Vector3(x1, y1, polygon.z_max));
        convex_hull_->addPoint(Ogre::Vector3(x2, y2, polygon.z_max));

        // Vertical line
        convex_hull_->newLine();
        convex_hull_->addPoint(Ogre::Vector3(x1, y1, polygon.z_min));
        convex_hull_->addPoint(Ogre::Vector3(x1, y1, polygon.z_max));
    }
}

void EntityVisual::setLabel(const std::string& label)
{
    label_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
    // You can't set an empty string as caption. But without setting a caption, its shows 'unknown'
    // Hide this 'unknown' by setting height to zero.
    if (label.empty())
        label_->setCharacterHeight(0);
    else
    {
        label_->setCharacterHeight(0.05);
        label_->setCaption(label);
    }
}

// Position and orientation are passed through to the SceneNode.
void EntityVisual::setFramePosition(const Ogre::Vector3& position)
{
    frame_node_->setPosition(position);
}

void EntityVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
    frame_node_->setOrientation(orientation);
}

} // end namespace rviz_plugins

