#include "airsim_ros/ros_wrapper.hpp"

namespace airsim_ros
{

const std::unordered_map<int, std::string> ROSWrapper::image_type_int_to_string_map_ = {
    { 0, "Scene" },
    { 1, "DepthPlanar" },
    { 2, "DepthPerspective" },
    { 3, "DepthVis" },
    { 4, "DisparityNormalized" },
    { 5, "Segmentation" },
    { 6, "SurfaceNormals" },
    { 7, "Infrared" }
};

ROSWrapper::ROSWrapper(rclcpp::Node::SharedPtr nh, const std::string& host_ip) :
    host_ip_(host_ip), airsim_client_images_(host_ip), airsim_settings_parser_(host_ip)
{
    nh_ = nh;

    if (AirSimSettings::singleton().simmode_name != AirSimSettings::kSimModeTypeCar) {
        airsim_mode_ = AIRSIM_MODE::DRONE;
        RCLCPP_INFO(nh_->get_logger(), "Setting ROS wrapper to DRONE mode");
    }
    else {
        airsim_mode_ = AIRSIM_MODE::CAR;
        RCLCPP_INFO(nh_->get_logger(), "Setting ROS wrapper to CAR mode");
    }
    initializeRos();
    std::cout << "ROSWrapper Initialized!\n";
}

void ROSWrapper::initializeRos()
{
    createRosPubsFromSettingsJson();
}

void ROSWrapper::initializeAirSim()
{

}

void ROSWrapper::createRosPubsFromSettingsJson()
{
    image_pub_vec_.clear();
    cam_info_pub_vec_.clear();
    camera_info_msg_vec_.clear();

    auto sensor_qos = rclcpp::SensorDataQoS();
    auto image_transporter = new image_transport::ImageTransport(nh_);
    
    // iterate over std::map<std::string, std::unique_ptr<VehicleSetting>> vehicles;
    for (const auto& curr_vehicle_elem : AirSimSettings::singleton().vehicles) {
        auto& vehicle_setting = curr_vehicle_elem.second;
        auto curr_vehicle_name = curr_vehicle_elem.first;
        setNansToZerosInPose(*vehicle_setting);

        std::unique_ptr<VehicleROS> vehicle_ros = nullptr;

        if (airsim_mode_ == AIRSIM_MODE::DRONE) {
            vehicle_ros = std::unique_ptr<MultiRotorROS>(new MultiRotorROS());
        }
        else {
            vehicle_ros = std::unique_ptr<CarROS>(new CarROS());
        }

        vehicle_ros->odom_frame_id = curr_vehicle_name + "/" + odom_frame_id_;
        vehicle_ros->vehicle_name = curr_vehicle_name;

        // appendStaticVehicleTf(vehicle_ros.get(), *vehicle_setting);  // TODO: add back

        // iterate over camera map std::map<std::string, CameraSetting> .cameras;
        for (auto& curr_camera_elem : vehicle_setting->cameras) {
            auto& camera_setting = curr_camera_elem.second;
            auto& curr_camera_name = curr_camera_elem.first;

            setNansToZerosInPose(*vehicle_setting, camera_setting);
            // appendStaticCameraTf(vehicle_ros.get(), curr_camera_name, camera_setting); // TODO: add back
            // camera_setting.gimbal
            std::vector<ImageRequest> current_image_request_vec;
            current_image_request_vec.clear();

            // iterate over capture_setting std::map<int, CaptureSetting> capture_settings
            for (const auto& curr_capture_elem : camera_setting.capture_settings) {
                auto& capture_setting = curr_capture_elem.second;

                // todo why does AirSimSettings::loadCaptureSettings calls AirSimSettings::initializeCaptureSettings()
                // which initializes default capture settings for _all_ NINE msr::airlib::ImageCaptureBase::ImageType
                if (!(std::isnan(capture_setting.fov_degrees))) {
                    ImageType curr_image_type = msr::airlib::Utils::toEnum<ImageType>(capture_setting.image_type);
                    // if scene / segmentation / surface normals / infrared, get uncompressed image with pixels_as_floats = false
                    if (capture_setting.image_type == 0 || capture_setting.image_type == 5 || capture_setting.image_type == 6 || capture_setting.image_type == 7) {
                        current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, false, false));
                    }
                    // if {DepthPlanar, DepthPerspective,DepthVis, DisparityNormalized}, get float image
                    else {
                        current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, true));
                    }

                    image_pub_vec_.push_back(image_transporter->advertise(curr_vehicle_name + "/" + curr_camera_name + "/" + image_type_int_to_string_map_.at(capture_setting.image_type), 1));
                    cam_info_pub_vec_.push_back(nh_->create_publisher<sensor_msgs::msg::CameraInfo>(curr_vehicle_name + "/" + curr_camera_name + "/" + image_type_int_to_string_map_.at(capture_setting.image_type) + "/camera_info", 10));
                    camera_info_msg_vec_.push_back(generateCamInfo(curr_camera_name, camera_setting, capture_setting));
                }
            }
            // push back pair (vector of image captures, current vehicle name)
            airsim_img_request_vehicle_name_pair_vec_.push_back(std::make_pair(current_image_request_vec, curr_vehicle_name));
        }
    }
}

// void ROSWrapper::appendStaticVehicleTf(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting)
// {
//     geometry_msgs::msg::TransformStamped vehicle_tf_msg;
//     vehicle_tf_msg.header.frame_id = world_frame_id_;
//     vehicle_tf_msg.header.stamp = now()
//     vehicle_tf_msg.child_frame_id = vehicle_ros->vehicle_name;
//     vehicle_tf_msg.transform.translation.x = vehicle_setting.position.x();
//     vehicle_tf_msg.transform.translation.y = vehicle_setting.position.y();
//     vehicle_tf_msg.transform.translation.z = vehicle_setting.position.z();
//     tf2::Quaternion quat;
//     quat.setRPY(vehicle_setting.rotation.roll, vehicle_setting.rotation.pitch, vehicle_setting.rotation.yaw);
//     vehicle_tf_msg.transform.rotation.x = quat.x();
//     vehicle_tf_msg.transform.rotation.y = quat.y();
//     vehicle_tf_msg.transform.rotation.z = quat.z();
//     vehicle_tf_msg.transform.rotation.w = quat.w();

//     if (isENU_) {
//         std::swap(vehicle_tf_msg.transform.translation.x, vehicle_tf_msg.transform.translation.y);
//         std::swap(vehicle_tf_msg.transform.rotation.x, vehicle_tf_msg.transform.rotation.y);
//         vehicle_tf_msg.transform.translation.z = -vehicle_tf_msg.transform.translation.z;
//         vehicle_tf_msg.transform.rotation.z = -vehicle_tf_msg.transform.rotation.z;
//     }

//     vehicle_ros->static_tf_msg_vec.emplace_back(vehicle_tf_msg);
// }

// void ROSWrapper::appendStaticCameraTf(VehicleROS* vehicle_ros, const std::string& camera_name, const CameraSetting& camera_setting)
// {
//     geometry_msgs::TransformStamped static_cam_tf_body_msg;
//     static_cam_tf_body_msg.header.frame_id = vehicle_ros->vehicle_name + "/" + odom_frame_id_;
//     static_cam_tf_body_msg.child_frame_id = camera_name + "_body/static";
//     static_cam_tf_body_msg.transform.translation.x = camera_setting.position.x();
//     static_cam_tf_body_msg.transform.translation.y = camera_setting.position.y();
//     static_cam_tf_body_msg.transform.translation.z = camera_setting.position.z();
//     tf2::Quaternion quat;
//     quat.setRPY(camera_setting.rotation.roll, camera_setting.rotation.pitch, camera_setting.rotation.yaw);
//     static_cam_tf_body_msg.transform.rotation.x = quat.x();
//     static_cam_tf_body_msg.transform.rotation.y = quat.y();
//     static_cam_tf_body_msg.transform.rotation.z = quat.z();
//     static_cam_tf_body_msg.transform.rotation.w = quat.w();

//     if (isENU_) {
//         std::swap(static_cam_tf_body_msg.transform.translation.x, static_cam_tf_body_msg.transform.translation.y);
//         std::swap(static_cam_tf_body_msg.transform.rotation.x, static_cam_tf_body_msg.transform.rotation.y);
//         static_cam_tf_body_msg.transform.translation.z = -static_cam_tf_body_msg.transform.translation.z;
//         static_cam_tf_body_msg.transform.rotation.z = -static_cam_tf_body_msg.transform.rotation.z;
//     }

//     geometry_msgs::msg::TransformStamped static_cam_tf_optical_msg = static_cam_tf_body_msg;
//     static_cam_tf_optical_msg.child_frame_id = camera_name + "_optical/static";

//     tf2::Quaternion quat_cam_body;
//     tf2::Quaternion quat_cam_optical;
//     tf2::convert(static_cam_tf_body_msg.transform.rotation, quat_cam_body);
//     tf2::Matrix3x3 mat_cam_body(quat_cam_body);
//     tf2::Matrix3x3 mat_cam_optical;
//     mat_cam_optical.setValue(mat_cam_body.getColumn(1).getX(), mat_cam_body.getColumn(2).getX(), mat_cam_body.getColumn(0).getX(), mat_cam_body.getColumn(1).getY(), mat_cam_body.getColumn(2).getY(), mat_cam_body.getColumn(0).getY(), mat_cam_body.getColumn(1).getZ(), mat_cam_body.getColumn(2).getZ(), mat_cam_body.getColumn(0).getZ());
//     mat_cam_optical.getRotation(quat_cam_optical);
//     quat_cam_optical.normalize();
//     tf2::convert(quat_cam_optical, static_cam_tf_optical_msg.transform.rotation);

//     vehicle_ros->static_tf_msg_vec.emplace_back(static_cam_tf_body_msg);
//     vehicle_ros->static_tf_msg_vec.emplace_back(static_cam_tf_optical_msg);
// }

// airsim uses nans for zeros in settings.json. we set them to zeros here for handling tfs in ROS
void ROSWrapper::setNansToZerosInPose(VehicleSetting& vehicle_setting) const
{
    if (std::isnan(vehicle_setting.position.x()))
        vehicle_setting.position.x() = 0.0;

    if (std::isnan(vehicle_setting.position.y()))
        vehicle_setting.position.y() = 0.0;

    if (std::isnan(vehicle_setting.position.z()))
        vehicle_setting.position.z() = 0.0;

    if (std::isnan(vehicle_setting.rotation.yaw))
        vehicle_setting.rotation.yaw = 0.0;

    if (std::isnan(vehicle_setting.rotation.pitch))
        vehicle_setting.rotation.pitch = 0.0;

    if (std::isnan(vehicle_setting.rotation.roll))
        vehicle_setting.rotation.roll = 0.0;
}

// if any nan's in camera pose, set them to match vehicle pose (which has already converted any potential nans to zeros)
void ROSWrapper::setNansToZerosInPose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const
{
    if (std::isnan(camera_setting.position.x()))
        camera_setting.position.x() = vehicle_setting.position.x();

    if (std::isnan(camera_setting.position.y()))
        camera_setting.position.y() = vehicle_setting.position.y();

    if (std::isnan(camera_setting.position.z()))
        camera_setting.position.z() = vehicle_setting.position.z();

    if (std::isnan(camera_setting.rotation.yaw))
        camera_setting.rotation.yaw = vehicle_setting.rotation.yaw;

    if (std::isnan(camera_setting.rotation.pitch))
        camera_setting.rotation.pitch = vehicle_setting.rotation.pitch;

    if (std::isnan(camera_setting.rotation.roll))
        camera_setting.rotation.roll = vehicle_setting.rotation.roll;
}

// todo have a special stereo pair mode and get projection matrix by calculating offset wrt drone body frame?
sensor_msgs::msg::CameraInfo ROSWrapper::generateCamInfo(const std::string& camera_name,
                                                         const CameraSetting& camera_setting,
                                                         const CaptureSetting& capture_setting) const
{
    sensor_msgs::msg::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = camera_name + "_optical";
    cam_info_msg.height = capture_setting.height;
    cam_info_msg.width = capture_setting.width;
    float f_x = (capture_setting.width / 2.0) / tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0));
    // todo focal length in Y direction should be same as X it seems. this can change in future a scene capture component which exactly correponds to a cine camera
    // float f_y = (capture_setting.height / 2.0) / tan(math_common::deg2rad(fov_degrees / 2.0));
    cam_info_msg.k = { f_x, 0.0, capture_setting.width / 2.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 1.0 };
    cam_info_msg.p = { f_x, 0.0, capture_setting.width / 2.0, 0.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
    return cam_info_msg;
}

}  // namespace airsim_ros
