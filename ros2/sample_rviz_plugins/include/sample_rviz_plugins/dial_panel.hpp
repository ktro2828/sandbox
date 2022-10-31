#pragma once

#include <deque>
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif

#include <queue>
#include <string>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/float64.hpp>

namespace Ui
{
    class DialUI;
} // namespace Ui

namespace sample_rviz_plugins
{
    class DialPanel : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        DialPanel(QWidget *parent = nullptr);
        ~DialPanel() override;

        void onInitialize() override;
        void onEnable();
        void onDisable();

    private Q_SLOTS:
      void renderOverlay();

    private:
      void processMessage(rcl_interfaces::msg::Log::ConstSharedPtr msg) override;
      rviz_common::properties::IntProperty *height_property_;
      rviz_common::properties::IntProperty *width_property_;
      rviz_common::properties::IntProperty *size_property_;

      Ogre::TextAreaOverlayElement *createTextElement(int index);
      void createMaterial(std::string mat_name);
      void destroyMaterial(std::string mat_name);

      std::deque<rcl_interfaces::msg::Log> log_msgs_;

      Ogre::Overlay *overlay_;
      Ogre::OverlayContainer *panel_;
      std::vector<Ogre::TextAreaOverlayElement *> text_elements_;
      Ogre::OverlayElement *mat_element_;
    }; // class DialPanel

} // namespace sample_rviz_plugins
