#include <boost/make_shared.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "world_model_display.h"
#include "../visuals/entity_visual.h"

#include <algorithm>
#include <vector>

// ----------------------------------------------------------------------------------------------------

float COLORS[27][3] = { { 0.6, 0.6, 0.6},
                        { 0.6, 0.6, 0.4},
                        { 0.6, 0.6, 0.2},
                        { 0.6, 0.4, 0.6},
                        { 0.6, 0.4, 0.4},
                        { 0.6, 0.4, 0.2},
                        { 0.6, 0.2, 0.6},
                        { 0.6, 0.2, 0.4},
                        { 0.6, 0.2, 0.2},
                        { 0.4, 0.6, 0.6},
                        { 0.4, 0.6, 0.4},
                        { 0.4, 0.6, 0.2},
                        { 0.4, 0.4, 0.6},
                        { 0.4, 0.4, 0.4},
                        { 0.4, 0.4, 0.2},
                        { 0.4, 0.2, 0.6},
                        { 0.4, 0.2, 0.4},
                        { 0.4, 0.2, 0.2},
                        { 0.2, 0.6, 0.6},
                        { 0.2, 0.6, 0.4},
                        { 0.2, 0.6, 0.2},
                        { 0.2, 0.4, 0.6},
                        { 0.2, 0.4, 0.4},
                        { 0.2, 0.4, 0.2},
                        { 0.2, 0.2, 0.6},
                        { 0.2, 0.2, 0.4},
                        { 0.2, 0.2, 0.2}
                      };

// ----------------------------------------------------------------------------------------------------

unsigned int djb2(const std::string& str)
{
    int hash = 5381;
    for (const char& c: str)
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */

    if (hash < 0)
        hash = -hash;

    return hash;
}

// ----------------------------------------------------------------------------------------------------

/*
* @brief split Implementation by using delimiter as a character. Multiple delimeters are removed.
* @param strToSplit input string, which is splitted
* @param delimeter char on which the string is split
* @return vector of sub-strings
*/
std::vector<std::string> split(const std::string& strToSplit, char delimeter)
{
   std::stringstream ss(strToSplit);
   std::string item;
   std::vector<std::string> splittedStrings;
   while (std::getline(ss, item, delimeter))
   {
       if (!item.empty() && item[0] != delimeter)
           splittedStrings.push_back(item);
   }
   return splittedStrings;
}

// ----------------------------------------------------------------------------------------------------

namespace ed_rviz_plugins
{

WorldModelDisplay::WorldModelDisplay()
{
    service_name_property_ = std::make_unique<rviz::StringProperty>("Mesh query service name", "ed/gui/query_meshes", "Service name for querying meshes", this, SLOT(updateProperties()));

    entity_label_opacity_property_ = std::make_unique<rviz::FloatProperty>("Entity label opacity", 1.0, "Opacity of entity label", this);
    entity_volume_label_opacity_property_ = std::make_unique<rviz::FloatProperty>("Entity Volume label opacity", 0.4, "Opacity of entity label", this);
    entity_volume_opacity_property_ = std::make_unique<rviz::FloatProperty>("Entity Volume opacity", 0.2, "Opacity of entity label", this);
    exclude_labels_property_ = std::make_unique<rviz::StringProperty>("Exclude labels", "", "Exclude labels starting with (seperate with semi-colons)", this, SLOT(updateExcludeLabels()));

    updateProperties();
}

void WorldModelDisplay::updateProperties()
{
    if (service_client_.exists())
        service_client_.shutdown();

    ros::NodeHandle nh;
    service_client_ = nh.serviceClient<ed_gui_server_msgs::QueryMeshes>(service_name_property_->getStdString());
}

void WorldModelDisplay::updateExcludeLabels()
{
    exclude_labels_ = split(exclude_labels_property_->getStdString(),';');
}

void WorldModelDisplay::onInitialize()
{
    MFDClass::onInitialize();
}

WorldModelDisplay::~WorldModelDisplay()
{
}

void WorldModelDisplay::reset()
{
    MFDClass::reset();
}

void WorldModelDisplay::processMessage(const ed_gui_server_msgs::EntityInfos::ConstPtr &msg)
{
    // Transform to rviz frame
    Ogre::Quaternion frame_orientation;
    Ogre::Vector3 frame_position;
    if (!context_->getFrameManager()->getTransform("map", ros::Time::now(), frame_position, frame_orientation))
    {
        ROS_DEBUG("Error transforming from frame 'map' to frame '%s'", qPrintable(fixed_frame_));
        return;
    }

    std::vector<std::string> alive_ids;
    for (const ed_gui_server_msgs::EntityInfo& info: msg->entities)
    {
        if (info.id.size() >= 5 && (info.id.substr(info.id.size() - 5) == "floor" || info.id.substr(0, 5) == "floor"))
            continue; // Filter floor

        if (!info.has_pose)
            continue;

        if (visuals_.find(info.id) == visuals_.end()) // Visual does not exist yet; create visual
            visuals_[info.id] = boost::make_shared<EntityVisual>(context_->getSceneManager(), scene_node_);

        boost::shared_ptr<EntityVisual> visual = visuals_[info.id];

        // Get position and orientation
        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        position.x = info.pose.position.x;
        position.y = info.pose.position.y;
        position.z = info.pose.position.z;

        orientation.x = info.pose.orientation.x;
        orientation.y = info.pose.orientation.y;
        orientation.z = info.pose.orientation.z;
        orientation.w = info.pose.orientation.w;

        visual->setFramePosition(frame_position + position);
        visual->setFrameOrientation(frame_orientation * orientation);

        bool visual_needs_update = info.visual_revision > visual->visualRevision();
        bool volumes_needs_update = info.volumes_revision > visual->volumesRevision();
        if (visual_needs_update || volumes_needs_update)
        {
            query_meshes_srv_.request.entity_ids.push_back(info.id); // Mesh
            query_meshes_srv_.request.visual_requests.push_back(visual_needs_update);
            query_meshes_srv_.request.volumes_requests.push_back(volumes_needs_update);
        }
        else if (info.visual_revision == 0)
            visual->setConvexHull(info.polygon); // Convex hull

        // Set the color
        double r,g,b;
        if (info.color.a != 0) // If a color specified, take color from the info
        {
            r = static_cast<float>(info.color.r) / 255;
            g = static_cast<float>(info.color.g) / 255;
            b = static_cast<float>(info.color.b) / 255;
        }
        else // random color
        {
            int i_color = djb2(info.id) % 27;
            r = COLORS[i_color][0];
            g = COLORS[i_color][1];
            b = COLORS[i_color][2];
        }
        visual->setColor(Ogre::ColourValue(r, g, b, 1.0f), entity_label_opacity_property_->getFloat(),
                         entity_volume_opacity_property_->getFloat(), entity_volume_label_opacity_property_->getFloat());

        std::string label;
        // exclude label (label remains empty string) in case it starts with one of defined prefixes
        if (std::none_of(exclude_labels_.cbegin(), exclude_labels_.cend(), [&info](std::string s){return (info.id.substr(0, s.length()) == s);}))
        {
            label = info.id.substr(0, 6);

            if (!info.type.empty())
                label += " (" + info.type + ")";
        }
        visual->setLabel(label);

        alive_ids.push_back(info.id);
    }

    // Check which ids are not present
    std::vector<std::string> ids_to_be_removed;
    for (std::map<std::string, boost::shared_ptr<EntityVisual> >::const_iterator it = visuals_.begin(); it != visuals_.end(); ++it)
    {
        if (std::find(alive_ids.begin(), alive_ids.end(), it->first) == alive_ids.end()) // Not in alive ids
            ids_to_be_removed.push_back(it->first);
    }

    // Remove stale visuals
    for (std::vector<std::string>::const_iterator it = ids_to_be_removed.begin(); it != ids_to_be_removed.end(); ++it)
        visuals_.erase(*it);

    // Perform service call to get missing meshes
    if (!query_meshes_srv_.request.entity_ids.empty())
    {
        if (service_client_.call(query_meshes_srv_))
        {
            for (const auto& geom : query_meshes_srv_.response.entity_geometries)
            {
                const std::string& id = geom.id;

                if (visuals_.find(id) == visuals_.end())
                    continue;

                visuals_[id]->setEntityMeshAndVolumes(geom);
            }
        }
        else
        {
            ROS_ERROR("Could not query for meshes; does the service '%s' exist?", service_name_property_->getStdString().c_str());
        }
    }

    // No more meshes missing :)
    query_meshes_srv_.request.entity_ids.clear();
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ed_rviz_plugins::WorldModelDisplay,rviz::Display)
