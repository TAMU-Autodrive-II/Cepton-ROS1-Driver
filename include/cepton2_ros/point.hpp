#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "cepton_sdk2.h"

namespace cepton2_ros {

struct CustomCeptonPoint {
    float x;
    float y;
    float z;
    float reflectivity;
    uint8_t relative_timestamp;
    uint8_t flags;
    uint8_t channel_id;
    uint8_t valid;
};

using CeptonPointCloud = pcl::PointCloud<CustomCeptonPoint>;
}

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(cepton2_ros::CustomCeptonPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, reflectivity, reflectivity)
    (uint8_t, relative_timestamp, relative_timestamp)
    (uint8_t, flags, flags)
    (uint8_t, channel_id, channel_id)
    (uint8_t, valid, valid)
  )
// clang-format on



