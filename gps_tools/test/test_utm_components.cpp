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

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <class_loader/class_loader.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "gps_tools/conversions.h"

using namespace std::chrono_literals;

namespace
{

using nav_msgs::msg::Odometry;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;

// San Antonio, which is in zone 14R. Easting and northing are from
// GeographicLib; see test_conversions.cpp.
constexpr double kLat = 29.4241;
constexpr double kLon = -98.4936;
constexpr double kEasting = 549120.9765;
constexpr double kNorthing = 3255080.3933;
constexpr double kAltitude = 198.0;

// Both components are only defined in their .cpp files, so load them the way
// a component container would. The library paths come from CMakeLists.txt.
class Component
{
public:
  Component(
    const std::string & library, const std::string & class_name, const std::string & ns,
    const std::vector<rclcpp::Parameter> & parameters)
  : loader_(library)
  {
    const std::string factory_name =
      "rclcpp_components::NodeFactoryTemplate<" + class_name + ">";
    auto factory = loader_.createInstance<rclcpp_components::NodeFactory>(factory_name);
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=" + ns});
    options.parameter_overrides(parameters);
    wrapper_ = factory->create_node_instance(options);
  }

  ~Component()
  {
    // The node has to go before the library that holds its code.
    wrapper_ = rclcpp_components::NodeInstanceWrapper();
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base()
  {
    return wrapper_.get_node_base_interface();
  }

private:
  class_loader::ClassLoader loader_;
  rclcpp_components::NodeInstanceWrapper wrapper_;
};

/// Publishes one message type to a component and collects what it sends back.
///
/// Each harness gets a namespace of its own, so that two of them in one test
/// do not hear each other's topics.
template<typename InT, typename OutT>
class Harness
{
public:
  Harness(
    const std::string & library, const std::string & class_name,
    const std::string & in_topic, const std::string & out_topic,
    const std::vector<rclcpp::Parameter> & parameters = {})
  : ns_("/harness_" + std::to_string(next_id_++)),
    component_(library, class_name, ns_, parameters),
    node_(std::make_shared<rclcpp::Node>("test_harness", ns_))
  {
    pub_ = node_->create_publisher<InT>(in_topic, 10);
    sub_ = node_->create_subscription<OutT>(
      out_topic, 10, [this](const typename OutT::SharedPtr msg) {received_.push_back(*msg);});
    executor_.add_node(component_.base());
    executor_.add_node(node_);
    // Let discovery connect both directions before anything is published.
    spinUntil([this]() {return connected();});
  }

  ~Harness()
  {
    executor_.remove_node(node_);
    executor_.remove_node(component_.base());
  }

  /// Publish one message and return what the component sent for it, if anything.
  std::optional<OutT> send(const InT & msg, std::chrono::milliseconds timeout = 2s)
  {
    const size_t before = received_.size();
    pub_->publish(msg);
    spinUntil([&]() {return received_.size() > before;}, timeout);
    if (received_.size() > before) {
      return received_.back();
    }
    return std::nullopt;
  }

private:
  bool connected() const
  {
    return pub_->get_subscription_count() > 0 && sub_->get_publisher_count() > 0;
  }

  template<typename PredicateT>
  void spinUntil(PredicateT done, std::chrono::milliseconds timeout = 5s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!done() && std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(10ms);
    }
  }

  static inline int next_id_ = 0;
  std::string ns_;
  Component component_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  typename rclcpp::Publisher<InT>::SharedPtr pub_;
  typename rclcpp::Subscription<OutT>::SharedPtr sub_;
  std::vector<OutT> received_;
};

using ToOdometry = Harness<NavSatFix, Odometry>;
using ToNavSatFix = Harness<Odometry, NavSatFix>;

std::unique_ptr<ToOdometry> toOdometry(const std::vector<rclcpp::Parameter> & parameters = {})
{
  return std::make_unique<ToOdometry>(
    UTM_ODOMETRY_LIBRARY, "gps_tools::UtmOdometryComponent", "fix", "odom", parameters);
}

std::unique_ptr<ToNavSatFix> toNavSatFix(const std::vector<rclcpp::Parameter> & parameters = {})
{
  return std::make_unique<ToNavSatFix>(
    UTM_ODOMETRY_TO_NAVSATFIX_LIBRARY, "gps_tools::UtmOdometryToNavSatFixComponent", "odom",
    "fix", parameters);
}

NavSatFix makeFix(double lat = kLat, double lon = kLon)
{
  NavSatFix fix;
  fix.header.stamp.sec = 1000;
  fix.header.stamp.nanosec = 500;
  fix.header.frame_id = "gps";
  fix.status.status = NavSatStatus::STATUS_FIX;
  fix.latitude = lat;
  fix.longitude = lon;
  fix.altitude = kAltitude;
  fix.position_covariance = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  return fix;
}

Odometry makeOdometry(const std::string & frame_id)
{
  Odometry odom;
  odom.header.stamp.sec = 1000;
  odom.header.stamp.nanosec = 500;
  odom.header.frame_id = frame_id;
  odom.pose.pose.position.x = kEasting;
  odom.pose.pose.position.y = kNorthing;
  odom.pose.pose.position.z = kAltitude;
  for (size_t i = 0; i < 36; ++i) {
    odom.pose.covariance[i] = static_cast<double>(i);
  }
  return odom;
}

}  // namespace

TEST(UtmOdometry, ConvertsTheFixToUtm)
{
  auto harness = toOdometry({rclcpp::Parameter("child_frame_id", "base_link")});
  auto odom = harness->send(makeFix());
  ASSERT_TRUE(odom.has_value());

  EXPECT_NEAR(kEasting, odom->pose.pose.position.x, 1e-3);
  EXPECT_NEAR(kNorthing, odom->pose.pose.position.y, 1e-3);
  EXPECT_EQ(kAltitude, odom->pose.pose.position.z);
  EXPECT_EQ(1.0, odom->pose.pose.orientation.w);
  EXPECT_EQ(1000, odom->header.stamp.sec);
  EXPECT_EQ(500u, odom->header.stamp.nanosec);
  EXPECT_EQ("gps", odom->header.frame_id);
  EXPECT_EQ("base_link", odom->child_frame_id);
}

TEST(UtmOdometry, PlacesThePositionCovarianceAndRotationCovariance)
{
  auto harness = toOdometry({rclcpp::Parameter("rot_covariance", 0.25)});
  auto odom = harness->send(makeFix());
  ASSERT_TRUE(odom.has_value());

  // The 3x3 position block is the fix's covariance, row for row.
  for (size_t row = 0; row < 3; ++row) {
    for (size_t col = 0; col < 3; ++col) {
      EXPECT_EQ(static_cast<double>(row * 3 + col + 1), odom->pose.covariance[row * 6 + col]);
    }
  }
  for (size_t i = 3; i < 6; ++i) {
    EXPECT_EQ(0.25, odom->pose.covariance[i * 6 + i]);
  }
  // Nothing couples position and rotation.
  for (size_t row = 0; row < 3; ++row) {
    for (size_t col = 3; col < 6; ++col) {
      EXPECT_EQ(0.0, odom->pose.covariance[row * 6 + col]);
      EXPECT_EQ(0.0, odom->pose.covariance[col * 6 + row]);
    }
  }
}

TEST(UtmOdometry, NamesTheFrame)
{
  struct Case
  {
    std::string frame_id;
    bool append_zone;
    std::string expected;
  };
  const std::vector<Case> cases = {
    {"", false, "gps"},
    {"", true, "gps/utm_14R"},
    {"utm", false, "utm"},
    {"utm", true, "utm/utm_14R"},
  };
  for (const Case & c : cases) {
    SCOPED_TRACE("frame_id '" + c.frame_id + "', append_zone " + std::to_string(c.append_zone));
    auto harness = toOdometry(
      {rclcpp::Parameter("frame_id", c.frame_id), rclcpp::Parameter("append_zone", c.append_zone)});
    auto odom = harness->send(makeFix());
    ASSERT_TRUE(odom.has_value());
    EXPECT_EQ(c.expected, odom->header.frame_id);
  }
}

TEST(UtmOdometry, DropsFixesWithoutAFixOrAStamp)
{
  auto harness = toOdometry();

  NavSatFix no_fix = makeFix();
  no_fix.status.status = NavSatStatus::STATUS_NO_FIX;
  EXPECT_FALSE(harness->send(no_fix, 500ms).has_value());

  NavSatFix no_stamp = makeFix();
  no_stamp.header.stamp.sec = 0;
  no_stamp.header.stamp.nanosec = 0;
  EXPECT_FALSE(harness->send(no_stamp, 500ms).has_value());

  EXPECT_TRUE(harness->send(makeFix()).has_value());
}

TEST(UtmOdometry, DropsFixesWithoutAPosition)
{
  auto harness = toOdometry({rclcpp::Parameter("append_zone", true)});

  const std::vector<std::pair<double, double>> positions = {
    {NAN, kLon}, {kLat, NAN}, {NAN, NAN}, {INFINITY, kLon}};
  for (const auto & [lat, lon] : positions) {
    SCOPED_TRACE("lat " + std::to_string(lat) + ", lon " + std::to_string(lon));
    EXPECT_FALSE(harness->send(makeFix(lat, lon), 500ms).has_value());
  }

  EXPECT_TRUE(harness->send(makeFix()).has_value());
}

TEST(UtmOdometry, KeepsAFixWithoutAnAltitude)
{
  // A 2D fix still has a place in the UTM plane.
  auto harness = toOdometry();
  NavSatFix two_d = makeFix();
  two_d.altitude = NAN;
  auto odom = harness->send(two_d);
  ASSERT_TRUE(odom.has_value());
  EXPECT_NEAR(kEasting, odom->pose.pose.position.x, 1e-3);
  EXPECT_NEAR(kNorthing, odom->pose.pose.position.y, 1e-3);
  EXPECT_TRUE(std::isnan(odom->pose.pose.position.z));
}

TEST(UtmOdometryToNavSatFix, TakesTheZoneFromTheFrameId)
{
  auto harness = toNavSatFix();
  auto fix = harness->send(makeOdometry("gps/utm_14R"));
  ASSERT_TRUE(fix.has_value());

  EXPECT_NEAR(kLat, fix->latitude, 1e-8);
  EXPECT_NEAR(kLon, fix->longitude, 1e-8);
  EXPECT_EQ(kAltitude, fix->altitude);
  EXPECT_EQ("gps", fix->header.frame_id);
  EXPECT_EQ(1000, fix->header.stamp.sec);
  EXPECT_EQ(500u, fix->header.stamp.nanosec);
  EXPECT_EQ(NavSatStatus::STATUS_FIX, fix->status.status);
}

TEST(UtmOdometryToNavSatFix, TakesTheZoneFromTheParameter)
{
  auto harness = toNavSatFix({rclcpp::Parameter("zone", 14)});
  auto fix = harness->send(makeOdometry("utm"));
  ASSERT_TRUE(fix.has_value());

  EXPECT_NEAR(kLat, fix->latitude, 1e-8);
  EXPECT_NEAR(kLon, fix->longitude, 1e-8);
  EXPECT_EQ("utm", fix->header.frame_id);
}

TEST(UtmOdometryToNavSatFix, TakesAFullZoneDesignatorFromTheParameter)
{
  struct Case
  {
    std::string zone;
    double lat;
    double lon;
  };
  // The northings and eastings come from LLtoUTM(), which test_conversions
  // checks against GeographicLib.
  const std::vector<Case> cases = {{"14R", kLat, kLon}, {"56H", -33.8688, 151.2093},
    {"14", kLat, kLon}};
  for (const Case & c : cases) {
    SCOPED_TRACE(c.zone);
    Odometry odom = makeOdometry("utm");
    std::string zone;
    gps_tools::LLtoUTM(
      c.lat, c.lon, odom.pose.pose.position.y, odom.pose.pose.position.x, zone);

    auto harness = toNavSatFix({rclcpp::Parameter("zone", c.zone)});
    auto fix = harness->send(odom);
    ASSERT_TRUE(fix.has_value());
    EXPECT_NEAR(c.lat, fix->latitude, 1e-8);
    EXPECT_NEAR(c.lon, fix->longitude, 1e-8);
  }
}

TEST(UtmOdometryToNavSatFix, UsesTheFrameIdParameter)
{
  auto harness = toNavSatFix({rclcpp::Parameter("frame_id", "gps_antenna")});
  auto fix = harness->send(makeOdometry("gps/utm_14R"));
  ASSERT_TRUE(fix.has_value());
  EXPECT_EQ("gps_antenna", fix->header.frame_id);
}

TEST(UtmOdometryToNavSatFix, PlacesThePositionCovariance)
{
  auto harness = toNavSatFix();
  auto fix = harness->send(makeOdometry("gps/utm_14R"));
  ASSERT_TRUE(fix.has_value());

  for (size_t row = 0; row < 3; ++row) {
    for (size_t col = 0; col < 3; ++col) {
      EXPECT_EQ(static_cast<double>(row * 6 + col), fix->position_covariance[row * 3 + col]);
    }
  }
}

TEST(UtmOdometryToNavSatFix, DropsOdometryWithoutAZoneOrAStamp)
{
  auto harness = toNavSatFix();

  EXPECT_FALSE(harness->send(makeOdometry("gps"), 500ms).has_value());

  Odometry no_stamp = makeOdometry("gps/utm_14R");
  no_stamp.header.stamp.sec = 0;
  no_stamp.header.stamp.nanosec = 0;
  EXPECT_FALSE(harness->send(no_stamp, 500ms).has_value());

  EXPECT_TRUE(harness->send(makeOdometry("gps/utm_14R")).has_value());
}

TEST(UtmComponents, RoundTripAFixThroughBoth)
{
  // Sydney, to cover the southern hemisphere, and a single-digit zone.
  const std::vector<std::pair<double, double>> points = {{kLat, kLon}, {-33.8688, 151.2093},
    {61.2181, -149.9003}};
  auto to_odometry = toOdometry({rclcpp::Parameter("append_zone", true)});
  auto to_navsatfix = toNavSatFix();

  for (const auto & [lat, lon] : points) {
    SCOPED_TRACE(std::to_string(lat) + ", " + std::to_string(lon));
    auto odom = to_odometry->send(makeFix(lat, lon));
    ASSERT_TRUE(odom.has_value());
    auto fix = to_navsatfix->send(*odom);
    ASSERT_TRUE(fix.has_value());
    EXPECT_NEAR(lat, fix->latitude, 1e-8);
    EXPECT_NEAR(lon, fix->longitude, 1e-8);
    EXPECT_EQ("gps", fix->header.frame_id);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
