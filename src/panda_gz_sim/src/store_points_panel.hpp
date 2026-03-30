#pragma once

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <QPushButton>
#include <QLabel>

class StorePointsPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit StorePointsPanel(QWidget * parent = nullptr);

  void onInitialize() override;

private Q_SLOTS:
  void onStoreClicked();

private:
  QPushButton * button_;
  QLabel * status_label_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};
