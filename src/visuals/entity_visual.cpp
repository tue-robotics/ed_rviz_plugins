#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

#include "entity_visual.h"

namespace ed_rviz_plugins
{

namespace
{

void getMinMax(const Ogre::Vector3& value, Ogre::Vector3& min_pt, Ogre::Vector3& max_pt)
{
    if (value.x > max_pt.x)
        max_pt.x = value.x;
    if (value.y > max_pt.y)
        max_pt.y = value.y;
    if (value.z > max_pt.z)
        max_pt.z = value.z;

    if (value.x < min_pt.x)
        min_pt.x = value.x;
    if (value.y < min_pt.y)
        min_pt.y = value.y;
    if (value.z < min_pt.z)
        min_pt.z = value.z;
}

void addTriangleSoup(const std::vector<float>& vertices,
                     rviz_rendering::MeshShape& mesh,
                     Ogre::Vector3* min_pt = nullptr,
                     Ogre::Vector3* max_pt = nullptr)
{
    for (std::size_t i = 0; i < vertices.size() / 9; ++i)
    {
        const std::size_t offset = 9 * i;
        Ogre::Vector3 v1(vertices[offset], vertices[offset + 1], vertices[offset + 2]);
        Ogre::Vector3 v2(vertices[offset + 3], vertices[offset + 4], vertices[offset + 5]);
        Ogre::Vector3 v3(vertices[offset + 6], vertices[offset + 7], vertices[offset + 8]);

        if (min_pt && max_pt)
        {
            getMinMax(v1, *min_pt, *max_pt);
            getMinMax(v2, *min_pt, *max_pt);
            getMinMax(v3, *min_pt, *max_pt);
        }

        const Ogre::Vector3 normal = (v1 - v3).crossProduct(v2 - v3).normalisedCopy();
        mesh.addVertex(v1, normal);
        mesh.addVertex(v2, normal);
        mesh.addVertex(v3, normal);
    }
}

} // namespace

EntityVisual::EntityVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) :
    visual_revision_(0), volumes_revision_(0), frame_node_(nullptr), scene_manager_(scene_manager)
{
    frame_node_ = parent_node->createChildSceneNode();
    mesh_ = std::make_shared<rviz_rendering::MeshShape>(scene_manager_, frame_node_);
    convex_hull_ = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, frame_node_);
    label_ = std::make_shared<rviz_rendering::MovableText>("unknown");
    frame_node_->attachObject(label_.get());
}

EntityVisual::~EntityVisual()
{
    scene_manager_->destroySceneNode(frame_node_);
}

void EntityVisual::setEntityMeshAndVolumes(const ed_gui_server_interfaces::msg::EntityMeshAndVolumes& mesh_and_volumes)
{
    mesh_->clear();
    if (mesh_and_volumes.visual_revision != 0)
    {
        visual_revision_ = mesh_and_volumes.visual_revision;
        mesh_->estimateVertexCount(mesh_and_volumes.mesh.vertices.size() / 3);
        mesh_->beginTriangles();
        addTriangleSoup(mesh_and_volumes.mesh.vertices, *mesh_);
        mesh_->endTriangles();
    }

    if (mesh_and_volumes.volumes_revision != 0)
    {
        volumes_revision_ = mesh_and_volumes.volumes_revision;
        volume_meshes_.clear();
        volume_labels_.clear();

        for (const auto& volume : mesh_and_volumes.volumes)
        {
            volume_labels_.push_back(std::make_shared<rviz_rendering::MovableText>(volume.name));
            const auto& label = volume_labels_.back();
            label->setTextAlignment(rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);
            label->setCharacterHeight(0.05);
            label->setCaption(volume.name);

            volume_meshes_.push_back(std::make_shared<rviz_rendering::MeshShape>(scene_manager_, frame_node_));
            const auto& mesh = volume_meshes_.back();
            mesh->estimateVertexCount(volume.mesh.vertices.size() / 3);
            mesh->beginTriangles();

            Ogre::Vector3 max_pt(-1e9, -1e9, -1e9);
            Ogre::Vector3 min_pt(1e9, 1e9, 1e9);
            addTriangleSoup(volume.mesh.vertices, *mesh, &min_pt, &max_pt);
            mesh->endTriangles();

            const Ogre::Vector3 label_position((max_pt.x + min_pt.x) / 2, (max_pt.y + min_pt.y) / 2, max_pt.z);
            frame_node_->createChildSceneNode(label_position)->attachObject(label.get());
        }
    }
}

void EntityVisual::setColor(Ogre::ColourValue color,
                            double entity_label_opacity,
                            double area_opacity,
                            double area_label_opacity)
{
    label_->setColor(Ogre::ColourValue(1.0f, 1.0f, 1.0f, entity_label_opacity));

    for (auto& label : volume_labels_)
        label->setColor(Ogre::ColourValue(1.0f, 1.0f, 1.0f, area_label_opacity));

    color.a = area_opacity;
    for (auto& mesh : volume_meshes_)
        mesh->setColor(color);

    color.a = 1.0;
    convex_hull_->setColor(color.r, color.g, color.b, color.a);
    mesh_->setColor(color);
}

void EntityVisual::setConvexHull(const ed_gui_server_interfaces::msg::Polygon& polygon)
{
    convex_hull_->clear();
    convex_hull_->setMaxPointsPerLine(2);
    convex_hull_->setNumLines(static_cast<uint32_t>(polygon.xs.size() * 3));
    convex_hull_->setLineWidth(0.01);

    for (std::size_t i = 0; i < polygon.xs.size(); ++i)
    {
        const std::size_t next = (i + 1) % polygon.xs.size();

        const float x1 = polygon.xs[i];
        const float x2 = polygon.xs[next];
        const float y1 = polygon.ys[i];
        const float y2 = polygon.ys[next];

        convex_hull_->addPoint(Ogre::Vector3(x1, y1, polygon.z_min));
        convex_hull_->addPoint(Ogre::Vector3(x2, y2, polygon.z_min));
        convex_hull_->finishLine();

        convex_hull_->addPoint(Ogre::Vector3(x1, y1, polygon.z_max));
        convex_hull_->addPoint(Ogre::Vector3(x2, y2, polygon.z_max));
        convex_hull_->finishLine();

        convex_hull_->addPoint(Ogre::Vector3(x1, y1, polygon.z_min));
        convex_hull_->addPoint(Ogre::Vector3(x1, y1, polygon.z_max));
        convex_hull_->finishLine();
    }
}

void EntityVisual::setLabel(const std::string& label)
{
    label_->setTextAlignment(rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);
    if (label.empty())
    {
        label_->setCharacterHeight(0);
    }
    else
    {
        label_->setCharacterHeight(0.05);
        label_->setCaption(label);
    }
}

void EntityVisual::setFramePosition(const Ogre::Vector3& position)
{
    frame_node_->setPosition(position);
}

void EntityVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
    frame_node_->setOrientation(orientation);
}

} // namespace ed_rviz_plugins
