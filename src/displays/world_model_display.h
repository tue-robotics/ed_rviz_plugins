#ifndef WORLD_MODEL_DISPLAY_H
#define WORLD_MODEL_DISPLAY_H

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#include <ed_gui_server_msgs/EntityInfos.h>
#include <ed_gui_server_msgs/QueryMeshes.h>

#include <memory>
#include <regex>
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

namespace ed_rviz_plugins
{

class EntityVisual;

class WorldModelDisplay: public rviz::MessageFilterDisplay<ed_gui_server_msgs::EntityInfos>
{
Q_OBJECT
public:
    WorldModelDisplay();
    virtual ~WorldModelDisplay();

protected:
    virtual void onInitialize();

    virtual void reset();

private Q_SLOTS:
    void updateProperties();
    void updateExcludeEntities();
    void updateExcludeEntityTypes();
    void updateExcludeLabels();

private:
    void processMessage(const ed_gui_server_msgs::EntityInfos::ConstPtr& msg);

    ros::ServiceClient service_client_;
    ed_gui_server_msgs::QueryMeshes query_meshes_srv_;

    std::map<std::string, boost::shared_ptr<EntityVisual> > visuals_;

    // User-editable property variables.
    std::unique_ptr<rviz::StringProperty> service_name_property_;
    std::unique_ptr<rviz::FloatProperty> entity_label_opacity_property_;
    std::unique_ptr<rviz::FloatProperty> entity_volume_label_opacity_property_;
    std::unique_ptr<rviz::FloatProperty> entity_volume_opacity_property_;
    std::unique_ptr<rviz::StringProperty> exclude_entities_property_;
    std::unique_ptr<rviz::StringProperty> exclude_entity_types_propetry_;
    std::unique_ptr<rviz::StringProperty> exclude_labels_property_;

    std::vector<std::regex> exclude_entities_;
    std::vector<std::regex> exclude_entity_types_;
    std::vector<std::regex> exclude_labels_;
};

}

#endif // WORLD_MODEL_DISPLAY_H
