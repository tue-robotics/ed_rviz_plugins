#include <pluginlib/class_loader.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/property.hpp>

#include <QApplication>

#include <gtest/gtest.h>

#include <string>

namespace
{

constexpr const char* PLUGIN_NAME = "ed_rviz_plugins/WorldModel";

rviz_common::properties::Property* findChild(const rviz_common::properties::Property& parent, const QString& name)
{
    for (int i = 0; i < parent.numChildren(); ++i)
    {
        if (parent.childAt(i)->getName() == name)
            return parent.childAt(i);
    }
    return nullptr;
}

class PluginLoadTest : public ::testing::Test
{
protected:
    pluginlib::ClassLoader<rviz_common::Display> loader_{"rviz_common", "rviz_common::Display"};
};

} // namespace

TEST_F(PluginLoadTest, PluginIsDeclared)
{
    EXPECT_TRUE(loader_.isClassAvailable(PLUGIN_NAME));
    EXPECT_EQ(loader_.getClassType(PLUGIN_NAME), "ed_rviz_plugins::WorldModelDisplay");
}

TEST_F(PluginLoadTest, PluginCanBeCreated)
{
    const auto display = loader_.createUniqueInstance(PLUGIN_NAME);
    ASSERT_NE(display, nullptr);
    // Only resolves if the moc output is linked into the plugin library
    EXPECT_STREQ(display->metaObject()->className(), "ed_rviz_plugins::WorldModelDisplay");
}

TEST_F(PluginLoadTest, PropertiesHaveDefaults)
{
    const auto display = loader_.createUniqueInstance(PLUGIN_NAME);
    ASSERT_NE(display, nullptr);

    const auto* service_name = findChild(*display, "Mesh query service name");
    ASSERT_NE(service_name, nullptr);
    EXPECT_EQ(service_name->getValue().toString().toStdString(), "ed/gui/query_meshes");

    for (const char* name : {"Entity label opacity",
                             "Entity Volume label opacity",
                             "Entity Volume opacity",
                             "Exclude entities",
                             "Exclude entity types",
                             "Exclude labels"})
        EXPECT_NE(findChild(*display, name), nullptr) << name;
}

int main(int argc, char** argv)
{
    // rviz properties are Qt objects, they need an application to exist
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
