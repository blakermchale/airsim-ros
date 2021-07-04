// #include "common/common_utils/StrictMode.hpp"
// STRICT_MODE_OFF //todo what does this do?
// #ifndef RPCLIB_MSGPACK
// #define RPCLIB_MSGPACK clmdep_msgpack
// #endif // !RPCLIB_MSGPACK
// #include "rpc/rpc_error.h"
//     STRICT_MODE_ON

#include "airsim_ros/settings_parser.hpp"
#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "sensors/lidar/LidarSimpleParams.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "airsim_ros/math_common.h"
#include "yaml-cpp/yaml.h"
#include "opencv2/opencv.hpp"
#include <chrono>
#include <unordered_map>
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <image_transport/image_transport.hpp>
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/header.hpp"

typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
typedef msr::airlib::AirSimSettings::CaptureSetting CaptureSetting;
typedef msr::airlib::AirSimSettings::VehicleSetting VehicleSetting;
typedef msr::airlib::AirSimSettings::CameraSetting CameraSetting;
typedef msr::airlib::AirSimSettings::LidarSetting LidarSetting;

namespace airsim_ros
{

class ROSWrapper
{
public:
    enum class AIRSIM_MODE : unsigned
    {
        DRONE,
        CAR
    };

    ROSWrapper(rclcpp::Node::SharedPtr nh, const std::string& host_ip);
    ~ROSWrapper(){};

    void initializeRos();
    void initializeAirSim();
    rclcpp::Node::SharedPtr getNode();

private:
    // FIXME: VehicleROS causing crash
    // utility struct for a SINGLE robot
    class VehicleROS
    {
    public:
        virtual ~VehicleROS() {}
        std::string vehicle_name;

        /// All things ROS
        // ros::Publisher odom_local_pub;
        // ros::Publisher global_gps_pub;
        // ros::Publisher env_pub;
        // airsim_ros_pkgs::Environment env_msg;
        // std::vector<SensorPublisher> sensor_pubs;
        // // handle lidar seperately for max performance as data is collected on its own thread/callback
        // std::vector<SensorPublisher> lidar_pubs;

        // nav_msgs::msg::Odometry curr_odom;
        // sensor_msgs::msg::NavSatFix gps_sensor_msg;

        std::vector<geometry_msgs::msg::TransformStamped> static_tf_msg_vec;

        std::string odom_frame_id;
        /// Status
        // bool is_armed_;
        // std::string mode_;
    };

    class CarROS : public VehicleROS
    {
    public:
        msr::airlib::CarApiBase::CarState curr_car_state;

        // airsim_ros::CarState car_state_msg;

        bool has_car_cmd;
        msr::airlib::CarApiBase::CarControls car_cmd;
    };

    class MultiRotorROS : public VehicleROS
    {
    public:
        /// State
        msr::airlib::MultirotorState curr_drone_state;
    };

    // methods that parse settings to create ros objects
    void createRosPubsFromSettingsJson();
    void appendStaticCameraTf(VehicleROS* vehicle_ros, const std::string& camera_name, const CameraSetting& camera_setting);
    void appendStaticVehicleTf(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting);

    // cleanup methods for nans
    void setNansToZerosInPose(VehicleSetting& vehicle_setting) const;
    void setNansToZerosInPose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const;

    // common ros helpers
    rclcpp::Time chronoTimestampToRos(const std::chrono::system_clock::time_point& stamp) const;
    rclcpp::Time airsimTimestampToRos(const msr::airlib::TTimePoint& stamp) const;

    // camera helpers
    cv::Mat manualDecodeDepth(const ImageResponse& img_response) const;
    sensor_msgs::msg::Image::SharedPtr getDepthImgMsgFromResponse(const ImageResponse& img_response, const rclcpp::Time curr_ros_time, const std::string frame_id);
    sensor_msgs::msg::CameraInfo generateCamInfo(const std::string& camera_name, const CameraSetting& camera_setting, const CaptureSetting& capture_setting) const;
    sensor_msgs::msg::Image::SharedPtr getImgMsgFromResponse(const ImageResponse& img_response, const rclcpp::Time curr_ros_time, const std::string frame_id);

    // publishers
    void processAndPublishImgResponse(const std::vector<ImageResponse>& img_response_vec, const int img_response_idx, const std::string& vehicle_name);

    // timer callbacks
    void imgResponseTimerCb();

    rclcpp::Node::SharedPtr nh_;

    // rclcpp objects
    rclcpp::TimerBase::SharedPtr img_timer_;

    AIRSIM_MODE airsim_mode_ = AIRSIM_MODE::DRONE;

    msr::airlib::GeoPoint origin_geo_point_; // gps coord of unreal origin

    SettingsParser airsim_settings_parser_;
    std::unordered_map<std::string, std::unique_ptr<VehicleROS>> vehicle_name_ptr_map_;
    static const std::unordered_map<int, std::string> image_type_int_to_string_map_;

    std::string host_ip_;
    std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_ = nullptr;
    // seperate busy connections to airsim, update in their own thread
    msr::airlib::RpcLibClientBase airsim_client_images_;

    typedef std::pair<std::vector<ImageRequest>, std::string> airsim_img_request_vehicle_name_pair;
    std::vector<airsim_img_request_vehicle_name_pair> airsim_img_request_vehicle_name_pair_vec_;
    std::vector<image_transport::Publisher> image_pub_vec_;  // TODO: Use camera publisher? or publisher?
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> cam_info_pub_vec_;

    std::vector<sensor_msgs::msg::CameraInfo> camera_info_msg_vec_;

    /// ROS tf
    const std::string AIRSIM_FRAME_ID = "world_ned";
    std::string world_frame_id_ = AIRSIM_FRAME_ID;
    const std::string AIRSIM_ODOM_FRAME_ID = "odom_local_ned";
    const std::string ENU_ODOM_FRAME_ID = "odom_local_enu";
    std::string odom_frame_id_ = AIRSIM_ODOM_FRAME_ID;
    // tf2_ros::TransformBroadcaster tf_broadcaster_;
    // tf2_ros::StaticTransformBroadcaster static_tf_pub_;

    bool isENU_ = false;
    // tf2_ros::Buffer tf_buffer_;
    // tf2_ros::TransformListener tf_listener_;

    //! Flags
    bool is_used_img_timer_cb_queue_ = false;

};

} // namespace airsim_ros
