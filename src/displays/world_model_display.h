#ifndef WORLD_MODEL_DISPLAY_H
#define WORLD_MODEL_DISPLAY_H

#ifndef Q_MOC_RUN
#include <ed_gui_server_interfaces/msg/entity_infos.hpp>
#include <ed_gui_server_interfaces/srv/query_meshes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/message_filter_display.hpp>

#include <map>
#include <memory>
#include <regex>
#include <set>
#include <string>
#include <vector>
#endif

class QTimer;

namespace rviz_common
{
namespace properties
{
class FloatProperty;
class StringProperty;
} // namespace properties
} // namespace rviz_common

namespace ed_rviz_plugins
{

class EntityVisual;

class WorldModelDisplay : public rviz_common::MessageFilterDisplay<ed_gui_server_interfaces::msg::EntityInfos>
{
    Q_OBJECT
public:
    WorldModelDisplay();
    ~WorldModelDisplay() override;

protected:
    void onInitialize() override;
    void reset() override;

private Q_SLOTS:
    void updateProperties();
    void updateExcludeEntities();
    void updateExcludeEntityTypes();
    void updateExcludeLabels();
    void pollMeshQueryResponses();

private:
    using QueryMeshesClient = rclcpp::Client<ed_gui_server_interfaces::srv::QueryMeshes>;

    struct PendingMeshQuery
    {
        QueryMeshesClient::FutureAndRequestId future;
        std::vector<std::string> entity_ids;
    };

    void processMessage(ed_gui_server_interfaces::msg::EntityInfos::ConstSharedPtr msg) override;

    QueryMeshesClient::SharedPtr service_client_;
    std::vector<PendingMeshQuery> pending_mesh_queries_;
    std::set<std::string> pending_entity_ids_;

    std::map<std::string, std::shared_ptr<EntityVisual>> visuals_;

    std::unique_ptr<rviz_common::properties::StringProperty> service_name_property_;
    std::unique_ptr<rviz_common::properties::FloatProperty> entity_label_opacity_property_;
    std::unique_ptr<rviz_common::properties::FloatProperty> entity_volume_label_opacity_property_;
    std::unique_ptr<rviz_common::properties::FloatProperty> entity_volume_opacity_property_;
    std::unique_ptr<rviz_common::properties::StringProperty> exclude_entities_property_;
    std::unique_ptr<rviz_common::properties::StringProperty> exclude_entity_types_property_;
    std::unique_ptr<rviz_common::properties::StringProperty> exclude_labels_property_;

    std::vector<std::regex> exclude_entities_;
    std::vector<std::regex> exclude_entity_types_;
    std::vector<std::regex> exclude_labels_;
    QTimer* response_timer_ = nullptr;
};

} // namespace ed_rviz_plugins

#endif // WORLD_MODEL_DISPLAY_H
