// Include ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>

// Include PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

// Include CeptonSDK2
#include "cepton_sdk2.h"
#include "cepton2_ros/point.hpp"
#include "cepton2_ros/SensorInformation.h"
#include "cepton2_ros/common.hpp"

#include <string>
#include <vector>
#include <map>

namespace cepton2_ros 
{
    /**
     * CEPTON_SDK2 nodelet. Publishes sensor point topics.
    **/
    class PublisherNodelet : public nodelet::Nodelet {
        public:
            ~PublisherNodelet();

            static void FrameCallbackWrapper(CeptonSensorHandle handle, int64_t start_timestamp,
                   size_t n_points, size_t stride, const uint8_t *points,
                   void *user_data);
            
            static void SensorInfoCallbackWrapper(CeptonSensorHandle handle,
                                         const struct CeptonSensor *info, 
                                         void *user_data);
            
            void check_api_error(int err, char const *api);

            void PublishPoints(CeptonSensorHandle handle, int64_t start_timestamp,
                   size_t n_points, size_t stride, const uint8_t *points);
            
            void PublishSensorInformation(const CeptonSensor *info);

        protected:
            void onInit() override;

        private:
        CeptonSensorErrorCallback error_callback;

        ros::NodeHandle node_handle;
        ros::NodeHandle private_node_handle;
        ros::Timer watchdog_timer;
        ros::Publisher points_publisher;
        ros::Publisher sensor_info_publisher;
        
        std::map<CeptonSensorHandle, ros::Publisher> publisher_map;

        std::vector<CustomCeptonPoint> local_points;
        CeptonPointCloud point_cloud;
        bool include_invalid;
        std::string topic_prefix;
        CeptonReplayHandle replay_handle;
    };

} // namespace cepton2_ros