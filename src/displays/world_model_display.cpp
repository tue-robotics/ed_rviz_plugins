#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/properties/string_property.hpp>

#include <QTimer>

#include "../visuals/entity_visual.h"
#include "world_model_display.h"

#include <algorithm>
#include <chrono>
#include <future>
#include <iterator>
#include <sstream>
#include <utility>
#include <vector>

namespace
{

const float COLORS[27][3] = {{0.6, 0.6, 0.6}, {0.6, 0.6, 0.4}, {0.6, 0.6, 0.2}, {0.6, 0.4, 0.6}, {0.6, 0.4, 0.4},
                             {0.6, 0.4, 0.2}, {0.6, 0.2, 0.6}, {0.6, 0.2, 0.4}, {0.6, 0.2, 0.2}, {0.4, 0.6, 0.6},
                             {0.4, 0.6, 0.4}, {0.4, 0.6, 0.2}, {0.4, 0.4, 0.6}, {0.4, 0.4, 0.4}, {0.4, 0.4, 0.2},
                             {0.4, 0.2, 0.6}, {0.4, 0.2, 0.4}, {0.4, 0.2, 0.2}, {0.2, 0.6, 0.6}, {0.2, 0.6, 0.4},
                             {0.2, 0.6, 0.2}, {0.2, 0.4, 0.6}, {0.2, 0.4, 0.4}, {0.2, 0.4, 0.2}, {0.2, 0.2, 0.6},
                             {0.2, 0.2, 0.4}, {0.2, 0.2, 0.2}};

unsigned int djb2(const std::string& str)
{
    int hash = 5381;
    for (const char& c : str)
        hash = ((hash << 5) + hash) + c;

    if (hash < 0)
        hash = -hash;

    return static_cast<unsigned int>(hash);
}

std::vector<std::string> split(const std::string& str_to_split, char delimiter)
{
    std::stringstream ss(str_to_split);
    std::string item;
    std::vector<std::string> result;
    while (std::getline(ss, item, delimiter))
    {
        if (!item.empty() && item[0] != delimiter)
            result.push_back(item);
    }
    return result;
}

void stringsToRegexes(const std::string& str_to_split, std::vector<std::regex>& regexes, char delimiter = ';')
{
    const auto strings = split(str_to_split, delimiter);
    regexes.clear();
    regexes.reserve(strings.size());
    std::transform(strings.cbegin(),
                   strings.cend(),
                   std::back_inserter(regexes),
                   [](const std::string& str) { return std::regex(str, std::regex_constants::icase); });
}

} // namespace

namespace ed_rviz_plugins
{

WorldModelDisplay::WorldModelDisplay()
{
    service_name_property_ =
        std::make_unique<rviz_common::properties::StringProperty>("Mesh query service name",
                                                                  "ed/gui/query_meshes",
                                                                  "Service name for querying meshes",
                                                                  this,
                                                                  SLOT(updateProperties()));
    entity_label_opacity_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
        "Entity label opacity", 1.0, "Opacity of entity label", this);
    entity_volume_label_opacity_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
        "Entity Volume label opacity", 0.4, "Opacity of entity label", this);
    entity_volume_opacity_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
        "Entity Volume opacity", 0.2, "Opacity of entity label", this);
    exclude_entities_property_ =
        std::make_unique<rviz_common::properties::StringProperty>("Exclude entities",
                                                                  "",
                                                                  "Exclude entities regex (seperate with semi-colons)",
                                                                  this,
                                                                  SLOT(updateExcludeEntities()));
    exclude_entity_types_property_ = std::make_unique<rviz_common::properties::StringProperty>(
        "Exclude entity types",
        "",
        "Exclude entity types regex (seperate with semi-colons)",
        this,
        SLOT(updateExcludeEntityTypes()));
    exclude_labels_property_ = std::make_unique<rviz_common::properties::StringProperty>(
        "Exclude labels", "", "Exclude labels regex (seperate with semi-colons)", this, SLOT(updateExcludeLabels()));
}

WorldModelDisplay::~WorldModelDisplay() = default;

void WorldModelDisplay::updateProperties()
{
    pending_mesh_queries_.clear();
    pending_entity_ids_.clear();
    service_client_.reset();

    const auto node_interface = rviz_ros_node_.lock();
    if (!node_interface)
        return;

    service_client_ = node_interface->get_raw_node()->create_client<ed_gui_server_interfaces::srv::QueryMeshes>(
        service_name_property_->getStdString());
}

void WorldModelDisplay::updateExcludeEntities()
{
    stringsToRegexes(exclude_entities_property_->getStdString(), exclude_entities_);
}

void WorldModelDisplay::updateExcludeEntityTypes()
{
    stringsToRegexes(exclude_entity_types_property_->getStdString(), exclude_entity_types_);
}

void WorldModelDisplay::updateExcludeLabels()
{
    stringsToRegexes(exclude_labels_property_->getStdString(), exclude_labels_);
}

void WorldModelDisplay::onInitialize()
{
    MFDClass::onInitialize();
    updateProperties();

    response_timer_ = new QTimer(this);
    connect(response_timer_, &QTimer::timeout, this, &WorldModelDisplay::pollMeshQueryResponses);
    response_timer_->start(50);
}

void WorldModelDisplay::reset()
{
    if (service_client_)
    {
        for (const auto& pending : pending_mesh_queries_)
            service_client_->remove_pending_request(pending.future);
    }
    pending_mesh_queries_.clear();
    pending_entity_ids_.clear();
    MFDClass::reset();
}

void WorldModelDisplay::pollMeshQueryResponses()
{
    for (auto it = pending_mesh_queries_.begin(); it != pending_mesh_queries_.end();)
    {
        if (it->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        {
            ++it;
            continue;
        }

        const auto response = it->future.get();
        for (const auto& geometry : response->entity_geometries)
        {
            const auto visual = visuals_.find(geometry.id);
            if (visual != visuals_.end())
                visual->second->setEntityMeshAndVolumes(geometry);
        }

        if (!response->error_msg.empty())
        {
            setStatus(rviz_common::properties::StatusProperty::Error,
                      "Mesh query",
                      QString::fromStdString(response->error_msg));
        }
        else
        {
            setStatus(rviz_common::properties::StatusProperty::Ok, "Mesh query", "OK");
        }

        for (const auto& id : it->entity_ids)
            pending_entity_ids_.erase(id);
        it = pending_mesh_queries_.erase(it);
    }
}

void WorldModelDisplay::processMessage(ed_gui_server_interfaces::msg::EntityInfos::ConstSharedPtr msg)
{
    Ogre::Quaternion frame_orientation;
    Ogre::Vector3 frame_position;
    const auto node_interface = rviz_ros_node_.lock();
    if (!node_interface)
        return;

    const auto node = node_interface->get_raw_node();
    if (!context_->getFrameManager()->getTransform("map", node->now(), frame_position, frame_orientation))
    {
        RCLCPP_DEBUG(node->get_logger(),
                     "Error transforming from frame 'map' to frame '%s'",
                     fixed_frame_.toStdString().c_str());
        return;
    }

    std::vector<std::string> alive_ids;
    auto query = std::make_shared<ed_gui_server_interfaces::srv::QueryMeshes::Request>();
    for (const auto& info : msg->entities)
    {
        if (info.id.size() >= 5 && (info.id.substr(info.id.size() - 5) == "floor" || info.id.substr(0, 5) == "floor"))
            continue;

        if (!info.has_pose)
            continue;

        if (std::any_of(exclude_entities_.cbegin(),
                        exclude_entities_.cend(),
                        [&info](const std::regex& regex) { return std::regex_match(info.id, regex); }))
            continue;

        if (std::any_of(exclude_entity_types_.cbegin(),
                        exclude_entity_types_.cend(),
                        [&info](const std::regex& regex) { return std::regex_match(info.type, regex); }))
            continue;

        if (visuals_.find(info.id) == visuals_.end())
        {
            visuals_[info.id] = std::make_shared<EntityVisual>(context_->getSceneManager(), scene_node_);
        }

        const auto& visual = visuals_[info.id];

        Ogre::Vector3 position(info.pose.position.x, info.pose.position.y, info.pose.position.z);
        Ogre::Quaternion orientation(
            info.pose.orientation.w, info.pose.orientation.x, info.pose.orientation.y, info.pose.orientation.z);
        visual->setFramePosition(frame_position + position);
        visual->setFrameOrientation(frame_orientation * orientation);

        const bool visual_needs_update = info.visual_revision > visual->visualRevision();
        const bool volumes_need_update = info.volumes_revision > visual->volumesRevision();
        if ((visual_needs_update || volumes_need_update) &&
            pending_entity_ids_.find(info.id) == pending_entity_ids_.end())
        {
            query->entity_ids.push_back(info.id);
            query->visual_requests.push_back(visual_needs_update);
            query->volumes_requests.push_back(volumes_need_update);
        }
        else if (info.visual_revision == 0)
        {
            visual->setConvexHull(info.polygon);
        }

        double r;
        double g;
        double b;
        if (info.color.a != 0)
        {
            r = static_cast<float>(info.color.r) / 255;
            g = static_cast<float>(info.color.g) / 255;
            b = static_cast<float>(info.color.b) / 255;
        }
        else
        {
            const unsigned int color_index = djb2(info.id) % 27;
            r = COLORS[color_index][0];
            g = COLORS[color_index][1];
            b = COLORS[color_index][2];
        }
        visual->setColor(Ogre::ColourValue(r, g, b, 1.0f),
                         entity_label_opacity_property_->getFloat(),
                         entity_volume_opacity_property_->getFloat(),
                         entity_volume_label_opacity_property_->getFloat());

        std::string label;
        if (std::none_of(exclude_labels_.cbegin(),
                         exclude_labels_.cend(),
                         [&info](const std::regex& regex) { return std::regex_match(info.id, regex); }))
        {
            label = info.id.substr(0, 6);
            if (!info.type.empty())
                label += " (" + info.type + ")";
        }
        visual->setLabel(label);
        alive_ids.push_back(info.id);
    }

    std::vector<std::string> ids_to_be_removed;
    for (const auto& entry : visuals_)
    {
        if (std::find(alive_ids.cbegin(), alive_ids.cend(), entry.first) == alive_ids.cend())
            ids_to_be_removed.push_back(entry.first);
    }
    for (const auto& id : ids_to_be_removed)
        visuals_.erase(id);

    if (!query->entity_ids.empty())
    {
        if (!service_client_)
        {
            RCLCPP_ERROR(node->get_logger(), "Mesh query service client is unavailable");
        }
        else if (service_client_->service_is_ready())
        {
            auto future = service_client_->async_send_request(query);
            pending_entity_ids_.insert(query->entity_ids.begin(), query->entity_ids.end());
            pending_mesh_queries_.push_back({std::move(future), query->entity_ids});
        }
        else
        {
            RCLCPP_WARN_THROTTLE(node->get_logger(),
                                 *node->get_clock(),
                                 5000,
                                 "Mesh query service '%s' is not available",
                                 service_name_property_->getStdString().c_str());
        }
    }
}

} // namespace ed_rviz_plugins

PLUGINLIB_EXPORT_CLASS(ed_rviz_plugins::WorldModelDisplay, rviz_common::Display)
