#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <vector>
#include <string>
#include <algorithm>

using std::placeholders::_1;
using std::placeholders::_2;

class DynamicObjectRemover : public rclcpp::Node {
public:
    DynamicObjectRemover() : Node("dynamic_object_remover") {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

        pc_sub_.subscribe(this, "/point_cloud", qos.get_rmw_qos_profile());
        det_sub_.subscribe(this, "/yolo/detections", qos.get_rmw_qos_profile());

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), pc_sub_, det_sub_);
        sync_->registerCallback(std::bind(&DynamicObjectRemover::filterCallback, this, _1, _2));

        clean_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud/clean", 10);
        
        dynamic_classes_ = {"person", "bicycle", "car", "dog"};
        RCLCPP_INFO(this->get_logger(), "C++ Dynamic Object Remover Node Initialized.");
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, vision_msgs::msg::Detection2DArray>;
    
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pc_sub_;
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> det_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clean_pc_pub_;
    std::vector<std::string> dynamic_classes_;

    struct BBox3D {
        float min_x, max_x, min_y, max_y;
    };

    void filterCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg,
                        const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det_msg) {
        
        std::vector<BBox3D> exclusion_zones;
        const float box_width = 1.0f, box_height = 2.0f;

        // 1. Build exclusion zones natively
        for (const auto& det : det_msg->detections) {
            std::string class_id = det.results[0].hypothesis.class_id;
            if (std::find(dynamic_classes_.begin(), dynamic_classes_.end(), class_id) != dynamic_classes_.end()) {
                float c_x = det.bbox.center.position.x;
                float c_y = det.bbox.center.position.y;
                exclusion_zones.push_back({
                    c_x - box_width / 2.0f, c_x + box_width / 2.0f,
                    c_y - box_height / 2.0f, c_y + box_height / 2.0f
                });
            }
        }

        // 2. Prepare the clean output message memory space
        auto clean_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        clean_msg->header = pc_msg->header;
        sensor_msgs::PointCloud2Modifier modifier(*clean_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        
        // Allocate worst-case memory upfront to avoid vector reallocations
        modifier.resize(pc_msg->width * pc_msg->height);

        // 3. Fast memory iterators
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pc_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pc_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pc_msg, "z");

        sensor_msgs::PointCloud2Iterator<float> out_x(*clean_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(*clean_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(*clean_msg, "z");

        size_t valid_points = 0;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            bool keep = true;
            for (const auto& zone : exclusion_zones) {
                if (*iter_x >= zone.min_x && *iter_x <= zone.max_x &&
                    *iter_y >= zone.min_y && *iter_y <= zone.max_y) {
                    keep = false;
                    break;
                }
            }
            // Only write data if point is outside dynamic bounding boxes
            if (keep) {
                *out_x = *iter_x;
                *out_y = *iter_y;
                *out_z = *iter_z;
                ++out_x; ++out_y; ++out_z;
                valid_points++;
            }
        }

        // Trim the fat off the allocated block
        modifier.resize(valid_points);
        clean_pc_pub_->publish(std::move(clean_msg));
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicObjectRemover>());
    rclcpp::shutdown();
    return 0;
}