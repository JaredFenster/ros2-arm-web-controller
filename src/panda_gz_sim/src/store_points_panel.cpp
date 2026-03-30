#include "store_points_panel.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <QVBoxLayout>
#include <QMetaObject>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

StorePointsPanel::StorePointsPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  button_ = new QPushButton("Store Points", this);
  button_->setMinimumHeight(40);

  status_label_ = new QLabel("", this);
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setWordWrap(true);

  layout->addWidget(button_);
  layout->addWidget(status_label_);
  layout->addStretch();

  connect(button_, &QPushButton::clicked, this, &StorePointsPanel::onStoreClicked);
}

void StorePointsPanel::onInitialize()
{
  auto node = getDisplayContext()
    ->getRosNodeAbstraction().lock()->get_raw_node();

  client_ = node->create_client<std_srvs::srv::Trigger>("store_points");
}

void StorePointsPanel::onStoreClicked()
{
  if (!client_->service_is_ready()) {
    status_label_->setText("Service not available");
    return;
  }

  button_->setEnabled(false);
  status_label_->setText("Calling store_points...");

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto response = future.get();
      QMetaObject::invokeMethod(this, [this, response]() {
        button_->setEnabled(true);
        status_label_->setText(QString::fromStdString(response->message));
      }, Qt::QueuedConnection);
    });
}

PLUGINLIB_EXPORT_CLASS(StorePointsPanel, rviz_common::Panel)
