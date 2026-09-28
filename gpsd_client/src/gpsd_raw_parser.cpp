// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <gpsd_client/gpsd_raw_parser.hpp>

#include <algorithm>
#include <cstddef>
#include <vector>

namespace gpsd_client
{

std::size_t GpsdRawParser::skyviewCount(const gps_data_t & data)
{
  if (0 >= data.satellites_visible) {
    return 0;
  }
  return std::min(
    static_cast<std::size_t>(data.satellites_visible),
    static_cast<std::size_t>(MAXCHANNELS));
}

std::size_t GpsdRawParser::deviceCount(const gps_data_t & data)
{
  if (0 >= data.devices.ndevices) {
    return 0;
  }
  const std::size_t capacity =
    sizeof(data.devices.list) / sizeof(data.devices.list[0]);
  return std::min(static_cast<std::size_t>(data.devices.ndevices), capacity);
}

GpsdRawMsg GpsdRawParser::parseRaw(
  const gps_data_t & data,
  const rclcpp::Time & stamp) const
{
  GpsdRawMsg msg;

  msg.header.stamp = stamp;
  msg.header.frame_id = context_.frame_id;

  // Every scalar, including the members that used to live in sub-messages.
  generated::fill(data, msg);

  /* The parallel-array groups. The generated filler writes each group from a
   * single loop, so its arrays cannot end up different lengths; this side only
   * decides *which* source elements are valid, which the generator cannot know.
   */
  std::vector<std::size_t> idx;
  auto prefix = [&idx](std::size_t n) -> const std::vector<std::size_t> & {
      idx.resize(n);
      for (std::size_t i = 0; i < n; ++i) {
        idx[i] = i;
      }
      return idx;
    };

  generated::fill_skyview(data, msg, prefix(skyviewCount(data)));

  // gps_data_t has carried `devices` since API 9, so this needs no guard --
  // unlike the imu and raw groups below, which arrived later.
  generated::fill_devices_list(data, msg, prefix(deviceCount(data)));

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
  while (imu_count < max_imu && '\0' != data.imu[imu_count].msg[0]) {
    ++imu_count;
  }
  generated::fill_imu(data, msg, prefix(imu_count));
#endif

  /* rawdata_t::meas[] carries neither a count nor a terminator. GPSd fills
   * entries at arbitrary indices and leaves svid at 0 on unused ones, so its
   * own dumper walks all MAXCHANNELS and skips the empty entries rather than
   * stopping at the first (gpsd/gpsd_json.c). It skips svid 255 too, which
   * GLONASS uses for "unknown". Applying the same rule publishes the same
   * measurements GPSd reports -- and is why the fillers take indices rather
   * than a count: this group is a filtered subset, not a prefix.
   */
  const std::size_t max_meas = sizeof(data.raw.meas) / sizeof(data.raw.meas[0]);
  std::vector<std::size_t> meas;
  if (0 != (data.set & RAW_SET)) {
    for (std::size_t i = 0; i < max_meas; ++i) {
      if (0 != data.raw.meas[i].svid && 255 != data.raw.meas[i].svid) {
        meas.push_back(i);
      }
    }
  }
  generated::fill_raw_meas(data, msg, meas);

  return msg;
}

}  // namespace gpsd_client
