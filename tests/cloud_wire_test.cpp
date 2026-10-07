#include "io/ros/point_cloud_view.hpp"
#include <cassert>
#include <cstring>

int main() {
  sensor_msgs::PointCloud2 metadata;
  metadata.header.seq = 23;
  metadata.header.stamp.fromNSec(1700000000123456789LL);
  metadata.header.frame_id = "custom_world_frame";
  metadata.height = 1;
  metadata.is_dense = true;
  const char *names[] = {"x", "y", "z", "vehicle_id"};
  for (unsigned n : {0u, 3u, 4u, 9u}) {
    metadata.fields.clear();
    for (unsigned i = 0; i < n; ++i) {
      sensor_msgs::PointField field;
      field.name = names[i % 4];
      field.offset = i * 4;
      field.datatype = i == 3 ? sensor_msgs::PointField::INT32
                              : sensor_msgs::PointField::FLOAT32;
      field.count = 1;
      metadata.fields.push_back(field);
    }
    metadata.point_step = n * 4;
    metadata.width = n ? 17 : 0;
    metadata.row_step = metadata.width * metadata.point_step;
    std::vector<uint8_t> payload(metadata.row_step);
    for (size_t i = 0; i < payload.size(); ++i) payload[i] = uint8_t(i * 31);
    const xsim::PointCloudView view{metadata, payload};
    auto normal = metadata;
    normal.data = payload;
    const auto length = ros::serialization::serializationLength(normal);
    assert(ros::serialization::serializationLength(view) == length);
    std::vector<uint8_t> expected(length), actual(length);
    ros::serialization::OStream normal_stream(expected.data(), length);
    ros::serialization::OStream view_stream(actual.data(), length);
    ros::serialization::serialize(normal_stream, normal);
    ros::serialization::serialize(view_stream, view);
    assert(actual == expected);
    sensor_msgs::PointCloud2 decoded;
    ros::serialization::IStream input(actual.data(), length);
    ros::serialization::deserialize(input, decoded);
    assert(decoded.data == payload);
    assert(decoded.header.frame_id == metadata.header.frame_id);
    assert(decoded.header.stamp == metadata.header.stamp);
    assert(decoded.width == metadata.width && decoded.point_step == metadata.point_step);
    assert(metadata.data.empty());
  }
}
