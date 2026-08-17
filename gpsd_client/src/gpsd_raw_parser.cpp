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

std::size_t GpsdRawParser::deviceCount(const gps_data_t& data)
{
  if (0 >= data.devices.ndevices)
  {
    return 0;
  }
  const std::size_t capacity =
      sizeof(data.devices.list) / sizeof(data.devices.list[0]);
  return std::min(static_cast<std::size_t>(data.devices.ndevices), capacity);
}

GpsdRawMsg GpsdRawParser::parseRaw(const gps_data_t& data,
                                   const rclcpp::Time& stamp) const
{
  GpsdRawMsg msg;

  msg.header.stamp = stamp;
  msg.header.frame_id = context_.frame_id;

  // Everything with a fixed shape, including the nested fix/dop sub-messages.
  generated::fill(data, msg);

  // The generator skips skyview: its count lives in a sibling field rather
  // than in the type.
  const std::size_t visible = skyviewCount(data);
  msg.skyview.resize(visible);
  for (std::size_t i = 0; i < visible; ++i)
  {
    generated::fill(data.skyview[i], msg.skyview[i]);
  }

  // The device list, trimmed the same way. gps_data_t has carried `devices`
  // since API 9, so this needs no version guard -- unlike imu[] and source
  // below, which arrived later.
  const std::size_t devices = deviceCount(data);
  msg.devices.list.resize(devices);
  for (std::size_t i = 0; i < devices; ++i)
  {
    generated::fill(data.devices.list[i], msg.devices.list[i]);
  }

#if GPSD_API_MAJOR_VERSION >= 12
  /* imu[] is the one array with no count anywhere: gps_data_t carries a fixed
   * ten entries and nothing saying how many were filled. GPSd resolves this by
   * treating an empty attitude_t::msg as the terminator -- its own JSON dumper
   * walks imu[] until msg[0] == '\0' (gpsd/gpsd_json.c) while the u-blox
   * driver stamps msg with "UBX-ESF-RAW" on each entry it populates. Use the
   * same rule rather than guessing, or publishing ten mostly-empty entries.
   *
   * attitude_t gained msg in API 12, the same version that added imu[], so
   * this is safe wherever the field exists.
   */
  const std::size_t max_imu = sizeof(data.imu) / sizeof(data.imu[0]);
  std::size_t imu_count = 0;
  while (imu_count < max_imu && '\0' != data.imu[imu_count].msg[0])
  {
    ++imu_count;
  }
  msg.imu.resize(imu_count);
  for (std::size_t i = 0; i < imu_count; ++i)
  {
    generated::fill(data.imu[i], msg.imu[i]);
  }
#endif

  /* rawdata_t::meas[] carries neither a count nor a terminator. GPSd fills
   * entries at arbitrary indices and leaves svid at 0 on unused ones, so its
   * own dumper walks all MAXCHANNELS and skips the empty entries rather than
   * stopping at the first (gpsd/gpsd_json.c). It skips svid 255 too, which
   * GLONASS uses for "unknown". Applying the same rule publishes the same
   * measurements GPSd reports.
   *
   * meas lives in the report union, so msg.raw is empty unless this report is
   * a RAW one.
   */
  if (!msg.raw.empty())
  {
    const std::size_t max_meas = sizeof(data.raw.meas) / sizeof(data.raw.meas[0]);
    for (std::size_t i = 0; i < max_meas; ++i)
    {
      if (0 == data.raw.meas[i].svid || 255 == data.raw.meas[i].svid)
      {
        continue;
      }
      msg.raw[0].meas.emplace_back();
      generated::fill(data.raw.meas[i], msg.raw[0].meas.back());
    }
  }

  return msg;
}

std::optional<GpsdRtcm2Msg> GpsdRawParser::parseRtcm2(
    const gps_data_t& data, const rclcpp::Time& stamp) const
{
  if (0 == (data.set & RTCM2_SET))
  {
    return std::nullopt;
  }

  GpsdRtcm2Msg msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = context_.frame_id;
  generated::fill(data.rtcm2, msg);
  return msg;
}

std::optional<GpsdRtcm3Msg> GpsdRawParser::parseRtcm3(
    const gps_data_t& data, const rclcpp::Time& stamp) const
{
  if (0 == (data.set & RTCM3_SET))
  {
    return std::nullopt;
  }

  GpsdRtcm3Msg msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = context_.frame_id;
  generated::fill(data.rtcm3, msg);
  return msg;
}

}  // namespace gpsd_client
