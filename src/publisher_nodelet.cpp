#include "publisher_nodelet.hpp"

#include <string>
#include <vector>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <chrono>
#include <regex>
#include "ros/ros.h"

PLUGINLIB_EXPORT_CLASS(cepton2_ros::PublisherNodelet, nodelet::Nodelet);

namespace cepton2_ros {

    PublisherNodelet::~PublisherNodelet() {
        int ret;
        ret = CeptonDeinitialize();
        check_api_error(ret, "CeptonDeinitialize");
    }

    void PublisherNodelet::check_api_error(int err, char const *api) {
        if (err != CEPTON_SUCCESS) {
            printf("API Error for %s: %s\n", api, CeptonGetErrorCodeName(err));
            exit(1);
        }
    }
    
    void PublisherNodelet::onInit() {

        ROS_INFO("PublisherNodeletStarted");
        int ret;
        include_invalid = true;

        // Get node handle
        node_handle = getNodeHandle();
        private_node_handle = getPrivateNodeHandle();

        // Get Parameters
        std::string capture_path = "";
        private_node_handle.param("capture_path", capture_path, capture_path);

        bool capture_loop = true;
        uint32_t replay_flags = 0;
        private_node_handle.param("capture_loop", capture_loop, capture_loop);
        printf("Replay loop: %d\n", (int)capture_loop);
        if (capture_loop) {
            replay_flags = CEPTON_REPLAY_FLAG_PLAY_LOOPED;
        }

        private_node_handle.param("topic_prefix", topic_prefix, topic_prefix);

        bool include_invalid_points = true;
        private_node_handle.param("include_invalid_points", include_invalid_points, include_invalid_points);
        include_invalid = include_invalid_points;

        // Assign publisher
        sensor_info_publisher = node_handle.advertise<cepton2_ros::SensorInformation>("cepton2/sensor_information",2);

        // Create Callbacks
        error_callback = [](CeptonSensorHandle handle, int error_code,
                         const char *error_msg, const void *error_data,
                         size_t error_data_size) {
            printf("Got error: %s\n", error_msg);
        };

        // Initialize SDK
        ret = CeptonInitialize(CEPTON_API_VERSION, error_callback);
        check_api_error(ret, "CeptonInitialize");
    
        // Start SDK CaptureReplay or Networking
        if(!capture_path.empty()) {
            ret = CeptonReplayLoadPcap(capture_path.c_str(), replay_flags, &replay_handle);
            //ret = CeptonStartReplay(capture_path.c_str(), capture_loop, 100);
            check_api_error(ret, "CeptonReplayLoadPcap");
        } else {
            ret = CeptonStartNetworking();
            check_api_error(ret, "CeptonStartNetworking");
        }

        printf("Waiting for sensors to connect...\n");
        while (CeptonGetSensorCount() == 0)
            ;
        printf("Sensor(s) connected.\n");

        // Listen for frames
        ret = CeptonListenFrames(CEPTON_AGGREGATION_MODE_FIXED_10Hz, FrameCallbackWrapper, this);
        check_api_error(ret, "CeptonListenFrames");
        
        // Listen for sensor info
        ret = CeptonListenSensorInfo(SensorInfoCallbackWrapper, this);
        check_api_error(ret, "CeptonListenSensorInfo");

        //Loop until replay is finished
        //Start watchdog timer
        watchdog_timer = node_handle.createTimer(
            ros::Duration(0.1), [&](const ros::TimerEvent &event) {
                // if (CeptonReplayIsFinished(replay_handle)) {
                // NODELET_INFO("[%s] capture replay done", getName().c_str());
                // ret = CeptonReplayUnloadPcap(replay_handle);
                // check_api_error(ret, "CeptonReplayUnload");
                
                // ros::shutdown();
                // }
            });

    } // PublisherNodelet::onInit
    
    long long getChronyTimestamp() {
    FILE* pipe = popen("chronyc -n tracking 2>&1", "r");
    if (!pipe) {
        std::cerr << "Error opening pipe to chronyc command." << std::endl;
        return -1;
    }

    char buffer[128];
    std::string result = "";
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
    }

    auto pcloseStatus = pclose(pipe);
    if (pcloseStatus == -1) {
        std::cerr << "Error closing pipe to chronyc command." << std::endl;
        return -1;
    }

    // Extract last offset value using regular expression
    std::regex offsetRegex("Last offset\\s+:\\s+([+-]?\\d+\\.\\d+)\\s+seconds");
    std::smatch offsetMatch;
    if (std::regex_search(result, offsetMatch, offsetRegex)) {
        std::string offsetStr = offsetMatch[1];
        double lastOffset = std::stod(offsetStr);

        // Get current time and adjust using the last offset
        auto currentTime = std::chrono::system_clock::now();
        auto timeSinceEpoch = std::chrono::duration_cast<std::chrono::microseconds>(currentTime.time_since_epoch());
        auto adjustedTime = timeSinceEpoch + std::chrono::microseconds(static_cast<long long>(lastOffset * 1e6));

        return adjustedTime.count();
    }

    return -1;
    }   

    void PublisherNodelet::FrameCallbackWrapper(CeptonSensorHandle handle, int64_t start_timestamp,
                   size_t n_points, size_t stride, const uint8_t *points,
                   void *user_data) {
            reinterpret_cast<PublisherNodelet *>(user_data)->PublishPoints(handle, start_timestamp, n_points, stride, points);
        
    } // PublisherNodelet::FrameCallbackWrapper

    void PublisherNodelet::SensorInfoCallbackWrapper(CeptonSensorHandle handle,
            const struct CeptonSensor *info, 
            void *user_data) {
        reinterpret_cast<PublisherNodelet *>(user_data)->PublishSensorInformation(info);
    } // PublisherNodelet::SensorInfoCallbackWrapper

    void PublisherNodelet::PublishPoints(CeptonSensorHandle handle, int64_t start_timestamp,
                   size_t n_points, size_t stride, const uint8_t *points) {
        
        local_points.clear();
        local_points.reserve(n_points);

        cepton2_ros::CustomCeptonPoint cp;
        for(int i = 0; i<(int)n_points; ++i) {
            CeptonPoint const &p = *reinterpret_cast<CeptonPoint const *>(points + i*stride);
            cp.x = p.x*0.005f;
            cp.y = p.y*0.005f;
            cp.z = p.z*0.005f;
            cp.reflectivity = p.reflectivity*0.01f;
            cp.relative_timestamp = p.relative_timestamp;
            cp.channel_id = p.channel_id;
            cp.valid = !(p.flags & CEPTON_POINT_NO_RETURN);

            // handle invalid points
            if (include_invalid || !(p.flags & CEPTON_POINT_NO_RETURN))
                local_points.push_back(cp);
        }

        
        // Configure cloud
        //point_cloud.header.stamp = cepton2_ros::rosutil::to_usec(ros::Time::now());
        long long chronyTimestamp = getChronyTimestamp();
        //point_cloud.header.stamp = ros::Time(chronyTimestamp / (int64_t)1e6, chronyTimestamp % (int64_t)1e6 * (int64_t)1e3)
        point_cloud.header.stamp = ros::Time(chronyTimestamp / 1000000, chronyTimestamp % 1000000 * 1000);
        //point_cloud.header.stamp.sec = chronyTimestamp / (int64_t)1e6;
        //point_cloud.header.stamp.nanosec = chronyTimestamp % (int64_t)1e6 * (int64_t)1e3;
        
        //point_cloud.header.stamp = start_timestamp;
        //point_cloud.header.frame_id = "cepton2";
        point_cloud.height = 1;
        point_cloud.width = local_points.size();
        point_cloud.resize(local_points.size());

        // Publish point_cloud message
        for(std::size_t i = 0; i < local_points.size(); ++i) {
            point_cloud.points[i] = local_points[i];
        }

        if (publisher_map.find(handle) != publisher_map.end()){
            // if publisher found, publish points
            publisher_map[handle].publish(point_cloud);
        }

        
        point_cloud.clear();
        local_points.clear();
        
    } //PublisherNodelet::PublishPoints

    void PublisherNodelet::PublishSensorInformation(const CeptonSensor *info) {
        
        if (publisher_map.find(info->handle) == publisher_map.end())
        {
            //publisher does not exist, create it
            std::string topic_name = topic_prefix + "_" + std::to_string(info->serial_number);
            publisher_map.insert(std::pair<CeptonSensorHandle, ros::Publisher>(info->handle, node_handle.advertise<CeptonPointCloud>(topic_name, 2)));
        }

        cepton2_ros::SensorInformation msg;
        msg.header.stamp = ros::Time::now();

        msg.handle = info->handle;
        msg.serial_number = info->serial_number;
        msg.model_name = info->model_name;
        msg.model = info->model;
        msg.model_reserved = info->model_reserved;

        msg.firmware_version = info->firmware_version;

        msg.power_up_timestamp = info->power_up_timestamp;
        msg.time_sync_offset = info->time_sync_offset;
        msg.time_sync_drift = info->time_sync_drift;

        msg.return_count = info->return_count;
        msg.segment_count = info->channel_count;

        msg.is_pps_connected = (info->status_flags && CEPTON_SENSOR_PPS_CONNECTED) ? 1: 0;
        msg.is_nmea_connected = (info->status_flags && CEPTON_SENSOR_NMEA_CONNECTED) ? 1: 0;
        msg.is_ptp_connected = (info->status_flags && CEPTON_SENSOR_PTP_CONNECTED) ? 1: 0;

        const uint8_t *const info_bytes = (const uint8_t *)&info;
        msg.data = std::vector<uint8_t>(info_bytes, info_bytes + sizeof(info));

        sensor_info_publisher.publish(msg);
    } // PublisherNodelet::PublishSensorInformation

    

} // namespace cepton2_ros
