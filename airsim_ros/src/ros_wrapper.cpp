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
    RCLCPP_INFO(nh_->get_logger(), "ROSWrapper Initialized!");
}

void ROSWrapper::initializeRos()
{
    // Declare parameters
    nh_->declare_parameter<int>("update_img_response_hz", 30);

    createRosPubsFromSettingsJson();
}

void ROSWrapper::initializeAirSim()
{
    // todo do not reset if already in air?
    try {

        if (airsim_mode_ == AIRSIM_MODE::DRONE) {
            airsim_client_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::MultirotorRpcLibClient(host_ip_));
        }
        else {
            airsim_client_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::CarRpcLibClient(host_ip_));
        }
        airsim_client_->confirmConnection();
        airsim_client_images_.confirmConnection();
        // airsim_client_lidar_.confirmConnection();

        for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            airsim_client_->enableApiControl(true, vehicle_name_ptr_pair.first); // todo expose as rosservice?
            // airsim_client_->armDisarm(true, vehicle_name_ptr_pair.first); // todo exposes as rosservice?
        }

        // origin_geo_point_ = airsim_client_->getHomeGeoPoint("");
        // // todo there's only one global origin geopoint for environment. but airsim API accept a parameter vehicle_name? inside carsimpawnapi.cpp, there's a geopoint being assigned in the constructor. by?
        // origin_geo_point_msg_ = get_gps_msg_from_airsim_geo_point(origin_geo_point_);
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }
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
        RCLCPP_DEBUG(nh_->get_logger(), "Found vehicle `%s` in settings.", curr_vehicle_name.c_str());
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
            RCLCPP_DEBUG(nh_->get_logger(), "Found camera `%s` from vehicle `%s` in settings.", curr_camera_name.c_str(), curr_vehicle_name.c_str());

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

    // if >0 cameras, add one more thread for img_request_timer_cb
    if (!airsim_img_request_vehicle_name_pair_vec_.empty()) {
        RCLCPP_DEBUG(nh_->get_logger(), "Creating image response timer.");
        int img_response_hz;
        nh_->get_parameter("update_img_response_hz", img_response_hz);
        img_timer_ = nh_->create_wall_timer(std::chrono::milliseconds(1000/img_response_hz), std::bind(&ROSWrapper::imgResponseTimerCb, this));

        is_used_img_timer_cb_queue_ = true;
    }
    initializeAirSim();
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

// *********************
// Camera methods
// *********************
void ROSWrapper::processAndPublishImgResponse(const std::vector<ImageResponse>& img_response_vec, const int img_response_idx, const std::string& vehicle_name)
{
    // todo add option to use airsim time (image_response.TTimePoint) like Gazebo /use_sim_time param
    rclcpp::Time curr_ros_time = nh_->now();
    int img_response_idx_internal = img_response_idx;

    for (const auto& curr_img_response : img_response_vec) {
        // todo publishing a tf for each capture type seems stupid. but it foolproofs us against render thread's async stuff, I hope.
        // Ideally, we should loop over cameras and then captures, and publish only one tf.
        // publish_camera_tf(curr_img_response, curr_ros_time, vehicle_name, curr_img_response.camera_name);  // TODO: re-enable

        // todo simGetCameraInfo is wrong + also it's only for image type -1.
        // msr::airlib::CameraInfo camera_info = airsim_client_.simGetCameraInfo(curr_img_response.camera_name);

        // update timestamp of saved cam info msgs
        camera_info_msg_vec_[img_response_idx_internal].header.stamp = curr_ros_time;
        cam_info_pub_vec_[img_response_idx_internal]->publish(camera_info_msg_vec_[img_response_idx_internal]);

        // DepthPlanar / DepthPerspective / DepthVis / DisparityNormalized
        if (curr_img_response.pixels_as_float) {
            image_pub_vec_[img_response_idx_internal].publish(getDepthImgMsgFromResponse(curr_img_response,
                                                                                         curr_ros_time,
                                                                                         curr_img_response.camera_name + "_optical"));
        }
        // Scene / Segmentation / SurfaceNormals / Infrared
        else {
            image_pub_vec_[img_response_idx_internal].publish(getImgMsgFromResponse(curr_img_response,
                                                                                    curr_ros_time,
                                                                                    curr_img_response.camera_name + "_optical"));
        }
        img_response_idx_internal++;
    }
}

cv::Mat ROSWrapper::manualDecodeDepth(const ImageResponse& img_response) const
{
    cv::Mat mat(img_response.height, img_response.width, CV_32FC1, cv::Scalar(0));
    int img_width = img_response.width;

    for (int row = 0; row < img_response.height; row++)
        for (int col = 0; col < img_width; col++)
            mat.at<float>(row, col) = img_response.image_data_float[row * img_width + col];
    return mat;
}

sensor_msgs::msg::Image::SharedPtr ROSWrapper::getDepthImgMsgFromResponse(const ImageResponse& img_response,
                                                             const rclcpp::Time curr_ros_time,
                                                             const std::string frame_id)
{
    // todo using img_response.image_data_float direclty as done get_img_msg_from_response() throws an error,
    // hence the dependency on opencv and cv_bridge. however, this is an extremely fast op, so no big deal.
    cv::Mat depth_img = manualDecodeDepth(img_response);
    sensor_msgs::msg::Image::SharedPtr depth_img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth_img).toImageMsg();
    depth_img_msg->header.stamp = airsimTimestampToRos(img_response.time_stamp);
    depth_img_msg->header.frame_id = frame_id;
    return depth_img_msg;
}

sensor_msgs::msg::Image::SharedPtr ROSWrapper::getImgMsgFromResponse(const ImageResponse& img_response,
                                                                  const rclcpp::Time curr_ros_time,
                                                                  const std::string frame_id)
{
    sensor_msgs::msg::Image::SharedPtr img_msg_ptr = std::make_shared<sensor_msgs::msg::Image>();
    img_msg_ptr->data = img_response.image_data_uint8;
    img_msg_ptr->step = img_response.width * 3; // todo un-hardcode. image_width*num_bytes
    img_msg_ptr->header.stamp = airsimTimestampToRos(img_response.time_stamp);
    img_msg_ptr->header.frame_id = frame_id;
    img_msg_ptr->height = img_response.height;
    img_msg_ptr->width = img_response.width;
    img_msg_ptr->encoding = "bgr8";
    // if (is_vulkan_)  // TODO: re-enable?
    //     img_msg_ptr->encoding = "rgb8";
    img_msg_ptr->is_bigendian = 0;
    return img_msg_ptr;
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

// *********************
// General helpers
// *********************
rclcpp::Time ROSWrapper::airsimTimestampToRos(const msr::airlib::TTimePoint& stamp) const
{
    // airsim appears to use chrono::system_clock with nanosecond precision
    std::chrono::nanoseconds dur(stamp);
    std::chrono::time_point<std::chrono::system_clock> tp(dur);
    rclcpp::Time cur_time = chronoTimestampToRos(tp);
    return cur_time;
}

rclcpp::Time ROSWrapper::chronoTimestampToRos(const std::chrono::system_clock::time_point& stamp) const
{
    auto dur = std::chrono::duration<double>(stamp.time_since_epoch());
    rclcpp::Time cur_time(dur.count());
    return cur_time;
}

// ************************
// Callbacks
// ************************
void ROSWrapper::imgResponseTimerCb()
{
    try {
        int image_response_idx = 0;
        for (const auto& airsim_img_request_vehicle_name_pair : airsim_img_request_vehicle_name_pair_vec_) {
            const std::vector<ImageResponse>& img_response = airsim_client_images_.simGetImages(airsim_img_request_vehicle_name_pair.first, airsim_img_request_vehicle_name_pair.second);

            if (img_response.size() == airsim_img_request_vehicle_name_pair.first.size()) {
                processAndPublishImgResponse(img_response, image_response_idx, airsim_img_request_vehicle_name_pair.second);
                image_response_idx += img_response.size();
            }
        }
    }

    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, didn't get image response." << std::endl
                  << msg << std::endl;
    }
}

// *********************
// Getters
// *********************
rclcpp::Node::SharedPtr ROSWrapper::getNode()
{
    return nh_;
}

}  // namespace airsim_ros
