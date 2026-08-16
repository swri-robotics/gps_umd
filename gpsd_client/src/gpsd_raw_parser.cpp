#include <gpsd_client/gpsd_raw_parser.hpp>

#include <algorithm>

namespace gpsd_client
{

std::size_t GpsdRawParser::skyviewCount(const gps_data_t& data)
{
  if (0 >= data.satellites_visible)
  {
    return 0;
  }
  return std::min(static_cast<std::size_t>(data.satellites_visible),
                  static_cast<std::size_t>(MAXCHANNELS));
}

GpsdRawMsg GpsdRawParser::parseRaw(const gps_data_t& data,
                                   const rclcpp::Time& stamp) const
{
  GpsdRawMsg msg;

  msg.header.stamp = stamp;
  msg.header.frame_id = context_.frame_id;

  // Everything with a fixed shape, including the nested fix/dop sub-messages.
  generated::fill(data, msg);

  // skyview is deliberately left alone by the generator: the count lives in a
  // sibling field, not in the type.
  const std::size_t visible = skyviewCount(data);
  msg.skyview.resize(visible);
  for (std::size_t i = 0; i < visible; ++i)
  {
    generated::fill(data.skyview[i], msg.skyview[i]);
  }

  return msg;
}

}  // namespace gpsd_client
