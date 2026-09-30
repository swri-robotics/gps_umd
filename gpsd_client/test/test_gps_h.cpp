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

/// Checks gpsd_client/gps.hpp, which stands between gps.h and everything else.
///
/// Most of this is checked by compiling at all: every GPSStatus and
/// NavSatStatus constant is named after gps.h has been included, which the
/// macros gps.hpp undefines used to break. The tests pin down the values
/// gps.hpp keeps for the code that still needs them.

#include <gtest/gtest.h>

#include <gpsd_client/gps.hpp>
// Again, directly: gps.h's include guard has to keep the macros from coming
// back.
#include <gps.h>  // NOLINT(build/include_order)

#include <gps_msgs/msg/gps_status.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#if defined(STATUS_NO_FIX) || defined(STATUS_FIX) || defined(STATUS_DGPS_FIX) || \
  defined(STATUS_RTK_FIX)
#error "a gps.h macro that collides with a message constant is still defined"
#endif

using gps_msgs::msg::GPSStatus;
using sensor_msgs::msg::NavSatStatus;

TEST(GpsH, MessageConstantsKeepTheirValues)
{
  EXPECT_EQ(-1, GPSStatus::STATUS_NO_FIX);
  EXPECT_EQ(0, GPSStatus::STATUS_FIX);
  EXPECT_EQ(1, GPSStatus::STATUS_SBAS_FIX);
  EXPECT_EQ(2, GPSStatus::STATUS_GBAS_FIX);
  EXPECT_EQ(18, GPSStatus::STATUS_DGPS_FIX);
  EXPECT_EQ(19, GPSStatus::STATUS_RTK_FIX);
  EXPECT_EQ(20, GPSStatus::STATUS_RTK_FLOAT);
  EXPECT_EQ(33, GPSStatus::STATUS_WAAS_FIX);
  EXPECT_EQ(34, GPSStatus::STATUS_DR_FIX);
  EXPECT_EQ(35, GPSStatus::STATUS_SIM_FIX);
  EXPECT_EQ(36, GPSStatus::STATUS_TIME_FIX);

  EXPECT_EQ(-1, NavSatStatus::STATUS_NO_FIX);
  EXPECT_EQ(0, NavSatStatus::STATUS_FIX);
  EXPECT_EQ(1, NavSatStatus::STATUS_SBAS_FIX);
  EXPECT_EQ(2, NavSatStatus::STATUS_GBAS_FIX);
}

TEST(GpsH, KeepsGpsdsFixStatusValues)
{
  // The same values in every GPSd this supports, whatever they are called.
  EXPECT_EQ(1, gpsd_client::gps_h::kStatusGps);
  EXPECT_EQ(2, gpsd_client::gps_h::kStatusDgps);
  EXPECT_EQ(3, gpsd_client::gps_h::kStatusRtkFix);
  EXPECT_EQ(4, gpsd_client::gps_h::kStatusRtkFloat);
  EXPECT_EQ(5, gpsd_client::gps_h::kStatusDr);
  EXPECT_EQ(6, gpsd_client::gps_h::kStatusGnssDr);
  EXPECT_EQ(7, gpsd_client::gps_h::kStatusTime);
  EXPECT_EQ(8, gpsd_client::gps_h::kStatusSim);
  EXPECT_EQ(3, gpsd_client::gps_h::STATUS_RTK_FIX);
#ifdef GPSD_CLIENT_HAS_STATUS_NO_FIX
  EXPECT_EQ(0, gpsd_client::gps_h::STATUS_NO_FIX);
#endif
#ifdef GPSD_CLIENT_HAS_STATUS_FIX
  EXPECT_EQ(1, gpsd_client::gps_h::STATUS_FIX);
#endif
#ifdef GPSD_CLIENT_HAS_STATUS_DGPS_FIX
  EXPECT_EQ(2, gpsd_client::gps_h::STATUS_DGPS_FIX);
#endif
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
