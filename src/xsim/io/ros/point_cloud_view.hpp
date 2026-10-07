#pragma once
#include <ros/serialization.h>
#include <sensor_msgs/PointCloud2.h>

namespace xsim {
// Metadata owns no point bytes. The caller retains the immutable payload until
// serialization finishes; the TCPROS buffer then owns the wire representation.
struct PointCloudView {
  const sensor_msgs::PointCloud2 &metadata;
  const std::vector<uint8_t> &data;
};
} // namespace xsim

namespace ros { namespace serialization {
template <> struct Serializer<xsim::PointCloudView> {
  template <typename Stream>
  static void write(Stream &stream, const xsim::PointCloudView &view) {
    const auto &m = view.metadata;
    stream.next(m.header);
    stream.next(m.height);
    stream.next(m.width);
    stream.next(m.fields);
    stream.next(m.is_bigendian);
    stream.next(m.point_step);
    stream.next(m.row_step);
    stream.next(view.data);
    stream.next(m.is_dense);
  }
  static uint32_t serializedLength(const xsim::PointCloudView &view) {
    LStream stream;
    write(stream, view);
    return stream.getLength();
  }
};
}} // namespace ros::serialization
