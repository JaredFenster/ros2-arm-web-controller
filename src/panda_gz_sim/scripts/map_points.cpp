#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <cmath>
#include <chrono>
#include <unordered_set>
#include <tuple>

// Voxel size in metres — one point kept per cell
static constexpr float VOXEL_SIZE = 0.01f;
static constexpr double MAX_CLOUD_AGE_SEC = 0.5;

struct VoxelHash {
  std::size_t operator()(const std::tuple<int,int,int> & v) const {
    auto h = std::hash<int>{};
    return h(std::get<0>(v)) ^ (h(std::get<1>(v)) << 16) ^ (h(std::get<2>(v)) << 32);
  }
};

class MapPoints : public rclcpp::Node
{
public:
  MapPoints() : Node("map_points")
  {
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/points", 10,
      std::bind(&MapPoints::pointcloud_cb, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/map/points", 10);

    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MapPoints::publish_accumulated, this));

    srv_ = create_service<std_srvs::srv::Trigger>(
      "store_points",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        store();
        res->success = true;
        res->message = "Stored " + std::to_string(accumulated_.width) + " total points.";
      });
  }

  void store()
  {
    if (!latest_msg_) return;

    const rclcpp::Time cloud_stamp(latest_msg_->header.stamp);
    const bool has_valid_stamp = (latest_msg_->header.stamp.sec != 0) ||
      (latest_msg_->header.stamp.nanosec != 0);
    if (has_valid_stamp) {
      const double age = (now() - cloud_stamp).seconds();
      if (age > MAX_CLOUD_AGE_SEC) {
        RCLCPP_WARN(
          get_logger(),
          "Latest cloud is stale (age %.3f s). Wait for a fresh frame before storing.",
          age);
        return;
      }
    }

    sensor_msgs::msg::PointCloud2 transformed;
    try {
      auto tf = tf_buffer_->lookupTransform(
        "world", latest_msg_->header.frame_id,
        has_valid_stamp ? cloud_stamp : rclcpp::Time(0),
        rclcpp::Duration::from_seconds(0.5));
      tf2::doTransform(*latest_msg_, transformed, tf);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "Transform failed: %s", e.what());
      return;
    }

    if (accumulated_.data.empty()) {
      accumulated_.header       = transformed.header;
      accumulated_.header.frame_id = "world";
      accumulated_.fields       = transformed.fields;
      accumulated_.point_step   = transformed.point_step;
      accumulated_.height       = 1;
      accumulated_.is_dense     = false;
      accumulated_.is_bigendian = transformed.is_bigendian;
    }

    // Build occupied voxel set from already-accumulated points
    std::unordered_set<std::tuple<int,int,int>, VoxelHash> occupied;
    {
      sensor_msgs::PointCloud2ConstIterator<float> ax(accumulated_, "x");
      sensor_msgs::PointCloud2ConstIterator<float> ay(accumulated_, "y");
      sensor_msgs::PointCloud2ConstIterator<float> az(accumulated_, "z");
      for (; ax != ax.end(); ++ax, ++ay, ++az) {
        occupied.emplace(
          static_cast<int>(std::floor(*ax / VOXEL_SIZE)),
          static_cast<int>(std::floor(*ay / VOXEL_SIZE)),
          static_cast<int>(std::floor(*az / VOXEL_SIZE)));
      }
    }

    // Append one point per voxel, skipping cells already occupied
    const uint8_t * src = transformed.data.data();
    uint32_t added = 0;
    sensor_msgs::PointCloud2ConstIterator<float> ix(transformed, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(transformed, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(transformed, "z");
    for (uint32_t i = 0; ix != ix.end(); ++ix, ++iy, ++iz, ++i) {
      if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) continue;
      auto voxel = std::make_tuple(
        static_cast<int>(std::floor(*ix / VOXEL_SIZE)),
        static_cast<int>(std::floor(*iy / VOXEL_SIZE)),
        static_cast<int>(std::floor(*iz / VOXEL_SIZE)));
      if (!occupied.insert(voxel).second) continue;  // cell already taken
      const uint8_t * p = src + i * transformed.point_step;
      accumulated_.data.insert(accumulated_.data.end(), p, p + transformed.point_step);
      ++added;
    }
    accumulated_.width    = accumulated_.data.size() / accumulated_.point_step;
    accumulated_.row_step = accumulated_.data.size();
    accumulated_.header.stamp = now();

    RCLCPP_INFO(get_logger(), "Added %u points (total: %u)", added, accumulated_.width);
    publish_accumulated();
  }

private:
  void pointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    latest_msg_ = msg;
  }

  void publish_accumulated()
  {
    if (accumulated_.data.empty()) return;
    accumulated_.header.stamp = now();
    pub_->publish(accumulated_);
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_msg_;
  sensor_msgs::msg::PointCloud2 accumulated_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPoints>());
  rclcpp::shutdown();
  return 0;
}
