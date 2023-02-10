#include <stage.hh>
#include <mrpt_msgs/msg/observation_range_bearing.hpp>
#include <mrpt_msgs/msg/single_range_bearing_observation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
// see CMakeLists.txt
#ifdef USE_LEGACY_MSGS_INCLUDE
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

class FiducialWrapper
{
public:
    FiducialWrapper(const rclcpp::Node::SharedPtr& node, Stg::ModelFiducial* model, const std::string& name, const std::string& tf_prefix)
      :model_(model)
    {
        parent_frame_id_ = "base_link";
        if (tf_prefix.size() > 0) {
            parent_frame_id_ = tf_prefix + "/" + parent_frame_id_;
        }
        frame_id_ = name + "/base_fiducial";
        if (tf_prefix.size() > 0) {
            frame_id_ = tf_prefix + "/" + frame_id_;
        }
        fiducial_pub_ = node->create_publisher<mrpt_msgs::msg::ObservationRangeBearing>(std::string("~/") + name, 10);

    }

    void publish(const std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_broadcaster, const rclcpp::Time& now) {

        auto sensor = model_->GetFiducials();
        mrpt_msgs::msg::ObservationRangeBearing msg;
        msg.header.stamp = now;
        msg.header.frame_id = frame_id_;
        msg.sensed_data.reserve(sensor.size());
        double max_range = 0.;
        for(const Stg::ModelFiducial::Fiducial& fiducial : sensor)
        {
         mrpt_msgs::msg::SingleRangeBearingObservation landmark;
         landmark.range = fiducial.range;
         landmark.yaw = fiducial.bearing;
         landmark.pitch = 0.;
         landmark.id = fiducial.id;

         if (max_range < fiducial.range)
         {
          max_range = fiducial.range;
         }
         msg.sensed_data.emplace_back(landmark);
        }

        msg.max_sensor_distance = max_range + 1.;
        msg.min_sensor_distance = 0.;
        fiducial_pub_->publish(msg);
       
        Stg::Pose p = model_->GetPose();
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, p.a);
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = parent_frame_id_;
        transform.header.stamp = now;
        transform.child_frame_id = frame_id_;
        transform.transform.translation.x = p.x;
        transform.transform.translation.y = p.y;
        transform.transform.translation.z = p.z;
        transform.transform.rotation = toMsg(q);
        tf_broadcaster->sendTransform(transform);

    }
    Stg::ModelFiducial* model_;
    std::string parent_frame_id_;
    std::string frame_id_;
    rclcpp::Publisher<mrpt_msgs::msg::ObservationRangeBearing>::SharedPtr fiducial_pub_;
   
};

