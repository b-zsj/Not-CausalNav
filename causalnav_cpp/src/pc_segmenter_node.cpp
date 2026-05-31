#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <numeric>
#include <algorithm>

using std::placeholders::_1;
using std::placeholders::_2;

class PointCloudSegmenter : public rclcpp::Node {
public:
    PointCloudSegmenter() : Node("pc_segmenter"), model_initialized_(false) {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/rgb_camera/rgb/camera_info", 10,
            std::bind(&PointCloudSegmenter::infoCallback, this, _1));

        pc_sub_.subscribe(this, "/point_cloud");
        det_sub_.subscribe(this, "/yolo/tracking");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), pc_sub_, det_sub_);
        sync_->registerCallback(std::bind(&PointCloudSegmenter::syncCallback, this, _1, _2));

        centroid_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/causalnav/tracked_centroids", 10);
        
        RCLCPP_INFO(this->get_logger(), "C++ PointCloud Segmenter Node Initialized.");
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, yolo_msgs::msg::DetectionArray>;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    image_geometry::PinholeCameraModel cam_model_;
    std::string camera_frame_id_;
    bool model_initialized_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pc_sub_;
    message_filters::Subscriber<yolo_msgs::msg::DetectionArray> det_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr centroid_pub_;

    void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (!model_initialized_) {
            cam_model_.fromCameraInfo(*msg);
            camera_frame_id_ = msg->header.frame_id;
            model_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera model initialized. Frame: %s", camera_frame_id_.c_str());
        }
    }

    void syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg,
                      const yolo_msgs::msg::DetectionArray::ConstSharedPtr& det_msg) {
        if (!model_initialized_) return;

        sensor_msgs::msg::PointCloud2 pc_cam_frame;
        try {
            auto trans = tf_buffer_->lookupTransform(
                camera_frame_id_, pc_msg->header.frame_id, 
                pc_msg->header.stamp, rclcpp::Duration::from_nanoseconds(100000000));
            tf2::doTransform(*pc_msg, pc_cam_frame, trans);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF Transform failed: %s", ex.what());
            return;
        }

        // 1. Cache valid points directly to bypass iterators in the inner loop
        std::vector<cv::Point3d> points_3d;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc_cam_frame, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc_cam_frame, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc_cam_frame, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (*iter_z > 0.1f) {
                points_3d.emplace_back(*iter_x, *iter_y, *iter_z);
            }
        }

        for (const auto& detection : det_msg->detections) {
            std::string class_label = detection.class_name;
            std::string track_id = detection.id.empty() ? "unknown" : detection.id;
            
            float u_min = detection.bbox.center.position.x - (detection.bbox.size.x / 2.0f);
            float u_max = detection.bbox.center.position.x + (detection.bbox.size.x / 2.0f);
            float v_min = detection.bbox.center.position.y - (detection.bbox.size.y / 2.0f);
            float v_max = detection.bbox.center.position.y + (detection.bbox.size.y / 2.0f);

            std::vector<float> xs, ys, zs;

            // 2. Project and filter
            for (const auto& p : points_3d) {
                cv::Point2d uv = cam_model_.project3dToPixel(p);
                if (uv.x >= u_min && uv.x <= u_max && uv.y >= v_min && uv.y <= v_max) {
                    xs.push_back(p.x);
                    ys.push_back(p.y);
                    zs.push_back(p.z);
                }
            }

            // 3. Fast C++ Median Calculation
            if (!xs.empty()) {
                std::sort(xs.begin(), xs.end());
                std::sort(ys.begin(), ys.end());
                std::sort(zs.begin(), zs.end());
                
                size_t mid = xs.size() / 2;

                geometry_msgs::msg::PointStamped msg;
                msg.header.stamp = this->now();
                msg.header.frame_id = class_label + ":" + track_id;
                msg.point.x = xs[mid];
                msg.point.y = ys[mid];
                msg.point.z = zs[mid];

                centroid_pub_->publish(msg);
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudSegmenter>());
    rclcpp::shutdown();
    return 0;
}