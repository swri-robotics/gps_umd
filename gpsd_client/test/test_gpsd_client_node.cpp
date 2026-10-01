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

/* The whole node against a fake GPSd: parameters, topics, stamps and the
 * connection, through the same libgps calls a real daemon would see.
 *
 * The parser tests cover what a gps_data_t turns into. These cover everything
 * around that -- which topics exist, what each one publishes and when -- and
 * run everywhere, unlike the gpsfake suite, which needs a GPSd build.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gpsd_client/gpsd_client_component.hpp>
#include <gpsd_client/gpsd_client_lifecycle_component.hpp>

#include "fake_gpsd.hpp"
#include "gpsd_json_fixture.hpp"

using namespace std::chrono_literals;

namespace
{

using gps_msgs::msg::GPSDJson;
using gps_msgs::msg::GPSFix;
using gpsd_client::GpsdRawMsg;
using gpsd_client::test::FakeGpsd;
using lifecycle_msgs::msg::Transition;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;

/// The published variance for a GPSd uncertainty, taken as 1.96 sigma.
double Variance(double uncertainty)
{
  const double sigma = uncertainty / 1.96;
  return sigma * sigma;
}

/// 2023-11-14T22:13:20.500Z
constexpr int32_t kGpsSec = 1700000000;
constexpr uint32_t kGpsNanosec = 500000000;

gpsd_client::test::Tpv threeDFix(double latitude = 29.44)
{
  gpsd_client::test::Tpv tpv;
  tpv.mode = MODE_3D;
  tpv.time = "2023-11-14T22:13:20.500Z";
  tpv.latitude = latitude;
  tpv.longitude = -98.61;
  tpv.alt_hae = 250.0;
  tpv.alt_msl = 280.0;
  tpv.epx = 1.5;
  tpv.epy = 2.5;
  tpv.epv = 3.5;
  tpv.eph = 4.5;
  return tpv;
}

std::string skyWithGps()
{
  std::vector<gpsd_client::test::Satellite> satellites(4);
  for (size_t i = 0; i < satellites.size(); ++i) {
    satellites[i].prn = static_cast<int>(10 + i);
    satellites[i].elevation = 45.0;
    satellites[i].azimuth = 90.0;
    satellites[i].snr = 40.0;
    satellites[i].used = true;
  }
  return gpsd_client::test::skyJson(satellites);
}

/// Collects every message on one topic.
template<typename MsgT>
class Collector
{
public:
  Collector(rclcpp::Node & node, const std::string & topic)
  {
    sub_ = node.create_subscription<MsgT>(
      topic, 100, [this](const typename MsgT::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        messages_.push_back(*msg);
        changed_.notify_all();
      });
  }

  bool connected() const
  {
    return sub_->get_publisher_count() > 0;
  }

  /// The next message not yet returned, if one arrives in time.
  std::optional<MsgT> next(std::chrono::milliseconds timeout = 5s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!changed_.wait_for(lock, timeout, [this]() {return messages_.size() > taken_;})) {
      return std::nullopt;
    }
    return messages_[taken_++];
  }

  /// Wait until `count` messages have arrived in all, and return them.
  std::vector<MsgT> waitForCount(size_t count, std::chrono::milliseconds timeout = 5s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait_for(lock, timeout, [&]() {return messages_.size() >= count;});
    taken_ = messages_.size();
    return messages_;
  }

  /// True if nothing new arrives for `period`.
  bool staysQuiet(std::chrono::milliseconds period)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const size_t before = messages_.size();
    changed_.wait_for(lock, period, [&]() {return messages_.size() > before;});
    taken_ = messages_.size();
    return messages_.size() == before;
  }

private:
  typename rclcpp::Subscription<MsgT>::SharedPtr sub_;
  std::mutex mutex_;
  std::condition_variable changed_;
  std::vector<MsgT> messages_;
  size_t taken_ = 0;
};

/// Spins one node on a thread of its own until destroyed.
class Spinner
{
public:
  explicit Spinner(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
  : node_(node)
  {
    executor_.add_node(node_);
    thread_ = std::thread(&Spinner::run, this);
  }

  ~Spinner()
  {
    /* cancel() only stops a spin() that is already running; a spin() that
     * starts after it runs until the context shuts down. The thread may not
     * have reached spin() yet, so keep cancelling until spin() has returned.
     */
    while (!stopped_) {
      executor_.cancel();
      std::this_thread::sleep_for(1ms);
    }
    thread_.join();
    executor_.remove_node(node_);
  }

private:
  void run()
  {
    executor_.spin();
    stopped_ = true;
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::atomic<bool> stopped_{false};
  std::thread thread_;
};

/// Runs a gpsd_client node of type NodeT against a fake GPSd, and listens on
/// all four of its topics from a namespace no other test shares.
template<typename NodeT>
class NodeTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    // Stop spinning before the node goes, and drop the node, which closes
    // its connection, before the fake GPSd does.
    node_spinner_.reset();
    listener_spinner_.reset();
    fix_.reset();
    extended_fix_.reset();
    raw_.reset();
    json_.reset();
    listener_.reset();
    node_.reset();
  }

  /// Construct the node and start everything spinning.
  void create(std::vector<rclcpp::Parameter> parameters = {})
  {
    static int count = 0;
    ns_ = "/node_test_" + std::to_string(count++);
    parameters.emplace_back("host", "127.0.0.1");
    parameters.emplace_back("port", fake_.port());
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=" + ns_});
    options.parameter_overrides(parameters);
    node_ = std::make_shared<NodeT>(options);

    listener_ = std::make_shared<rclcpp::Node>("listener", ns_);
    fix_ = std::make_unique<Collector<NavSatFix>>(*listener_, "fix");
    extended_fix_ = std::make_unique<Collector<GPSFix>>(*listener_, "extended_fix");
    raw_ = std::make_unique<Collector<GpsdRawMsg>>(*listener_, "gpsd_raw");
    json_ = std::make_unique<Collector<GPSDJson>>(*listener_, "gpsd_json");

    node_spinner_ = std::make_unique<Spinner>(node_->get_node_base_interface());
    listener_spinner_ = std::make_unique<Spinner>(listener_->get_node_base_interface());
  }

  /// Wait until the fix topics are connected, so nothing sent is missed.
  void waitForDiscovery(bool raw = false, bool json = false)
  {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      if (fix_->connected() && extended_fix_->connected() &&
        (!raw || raw_->connected()) && (!json || json_->connected()))
      {
        return;
      }
      std::this_thread::sleep_for(10ms);
    }
    FAIL() << "the node's topics never connected";
  }

  size_t publishers(const std::string & topic)
  {
    return listener_->count_publishers(ns_ + "/" + topic);
  }

  FakeGpsd fake_;
  std::string ns_;
  std::shared_ptr<NodeT> node_;
  rclcpp::Node::SharedPtr listener_;
  std::unique_ptr<Collector<NavSatFix>> fix_;
  std::unique_ptr<Collector<GPSFix>> extended_fix_;
  std::unique_ptr<Collector<GpsdRawMsg>> raw_;
  std::unique_ptr<Collector<GPSDJson>> json_;
  std::unique_ptr<Spinner> node_spinner_;
  std::unique_ptr<Spinner> listener_spinner_;
};

/// The unmanaged node, which connects and streams as soon as it exists.
class ClientNode : public NodeTest<gpsd_client::GPSDClientComponent>
{
protected:
  void start(const std::vector<rclcpp::Parameter> & parameters = {})
  {
    create(parameters);
    ASSERT_TRUE(fake_.acceptClient()) << "the node never connected";
    watch_ = fake_.waitFor("?WATCH=");
    ASSERT_FALSE(watch_.empty()) << "the node never asked GPSd to stream";
  }

  std::string watch_;
};

}  // namespace

TEST_F(ClientNode, AsksGpsdToStreamJson)
{
  start();
  EXPECT_NE(watch_.find("\"enable\":true"), std::string::npos) << watch_;
  EXPECT_NE(watch_.find("\"json\":true"), std::string::npos) << watch_;
}

TEST_F(ClientNode, PublishesAFixFromATpvReport)
{
  start();
  waitForDiscovery();
  ASSERT_TRUE(fake_.send(skyWithGps()));
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));

  auto fix = fix_->next();
  ASSERT_TRUE(fix.has_value());
  EXPECT_EQ("gps", fix->header.frame_id);
  EXPECT_EQ(sensor_msgs::msg::NavSatStatus::STATUS_FIX, fix->status.status);
  EXPECT_EQ(NavSatStatus::SERVICE_GPS, fix->status.service);
  EXPECT_DOUBLE_EQ(29.44, fix->latitude);
  EXPECT_DOUBLE_EQ(-98.61, fix->longitude);
  EXPECT_DOUBLE_EQ(250.0, fix->altitude);
  EXPECT_DOUBLE_EQ(Variance(1.5), fix->position_covariance[0]);
  EXPECT_DOUBLE_EQ(Variance(2.5), fix->position_covariance[4]);
  EXPECT_DOUBLE_EQ(Variance(3.5), fix->position_covariance[8]);
  EXPECT_EQ(NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN, fix->position_covariance_type);
  // use_gps_time defaults to true.
  EXPECT_EQ(kGpsSec, fix->header.stamp.sec);
  EXPECT_EQ(kGpsNanosec, fix->header.stamp.nanosec);

  auto extended = extended_fix_->next();
  ASSERT_TRUE(extended.has_value());
  EXPECT_EQ("gps", extended->header.frame_id);
  EXPECT_EQ(gps_msgs::msg::GPSStatus::STATUS_FIX, extended->status.status);
  EXPECT_DOUBLE_EQ(29.44, extended->latitude);
  EXPECT_DOUBLE_EQ(-98.61, extended->longitude);
  EXPECT_DOUBLE_EQ(250.0, extended->altitude);
  EXPECT_DOUBLE_EQ(4.5, extended->err);
  EXPECT_DOUBLE_EQ(Variance(1.5), extended->position_covariance[0]);
  EXPECT_DOUBLE_EQ(Variance(2.5), extended->position_covariance[4]);
  EXPECT_DOUBLE_EQ(Variance(3.5), extended->position_covariance[8]);
  EXPECT_EQ(GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN, extended->position_covariance_type);
  EXPECT_DOUBLE_EQ(1700000000.5, extended->time);
}

TEST_F(ClientNode, PublishesNoFixWithoutAFix)
{
  start();
  waitForDiscovery();
  gpsd_client::test::Tpv tpv;
  tpv.mode = MODE_NO_FIX;
  tpv.time = "2023-11-14T22:13:20.500Z";
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(tpv)));

  auto fix = fix_->next();
  ASSERT_TRUE(fix.has_value());
  EXPECT_EQ(sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX, fix->status.status);
  auto extended = extended_fix_->next();
  ASSERT_TRUE(extended.has_value());
  EXPECT_EQ(gps_msgs::msg::GPSStatus::STATUS_NO_FIX, extended->status.status);
  EXPECT_TRUE(std::isnan(extended->latitude));
  EXPECT_TRUE(std::isnan(extended->longitude));
}

TEST_F(ClientNode, NamesEveryTopicsFrameWithFrameId)
{
  start(
    {rclcpp::Parameter("frame_id", "antenna"), rclcpp::Parameter("publish_gpsd_raw", true),
      rclcpp::Parameter("publish_gpsd_json", true)});
  waitForDiscovery(true, true);
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));

  auto fix = fix_->next();
  auto extended = extended_fix_->next();
  auto raw = raw_->next();
  auto json = json_->next();
  ASSERT_TRUE(fix && extended && raw && json);
  EXPECT_EQ("antenna", fix->header.frame_id);
  EXPECT_EQ("antenna", extended->header.frame_id);
  EXPECT_EQ("antenna", raw->header.frame_id);
  EXPECT_EQ("antenna", json->header.frame_id);
}

TEST_F(ClientNode, StampsWithTheNodeClockWhenUseGpsTimeIsOff)
{
  start({rclcpp::Parameter("use_gps_time", false)});
  waitForDiscovery();
  const rclcpp::Time before = node_->get_clock()->now();
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));

  auto fix = fix_->next();
  ASSERT_TRUE(fix.has_value());
  const int64_t stamp = rclcpp::Time(fix->header.stamp).nanoseconds();
  EXPECT_GE(stamp, before.nanoseconds());
  EXPECT_LE(stamp, node_->get_clock()->now().nanoseconds());
}

TEST_F(ClientNode, UseGpsTimeStampsOnlyTheNavSatFix)
{
  // use_gps_time defaults to true.
  start({rclcpp::Parameter("publish_gpsd_raw", true)});
  waitForDiscovery(true);
  const rclcpp::Time before = node_->get_clock()->now();
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));

  auto fix = fix_->next();
  ASSERT_TRUE(fix.has_value());
  EXPECT_EQ(kGpsSec, fix->header.stamp.sec);
  EXPECT_EQ(kGpsNanosec, fix->header.stamp.nanosec);

  // extended_fix and gpsd_raw keep the ROS time the report was read, and
  // share it, while GPSFix::time carries the receiver's time.
  auto extended = extended_fix_->next();
  ASSERT_TRUE(extended.has_value());
  const int64_t stamp = rclcpp::Time(extended->header.stamp).nanoseconds();
  EXPECT_GE(stamp, before.nanoseconds());
  EXPECT_LE(stamp, node_->get_clock()->now().nanoseconds());
  EXPECT_DOUBLE_EQ(1700000000.5, extended->time);

  auto raw = raw_->next();
  ASSERT_TRUE(raw.has_value());
  EXPECT_EQ(extended->header.stamp, raw->header.stamp);
}

TEST_F(ClientNode, FallsBackToTheNodeClockWhenTheReportHasNoTime)
{
  // A receiver without a fix often has no time either, and GPSd then leaves
  // the time out of the report. use_gps_time cannot stamp with a time the
  // report does not have.
  start();
  waitForDiscovery();
  const rclcpp::Time before = node_->get_clock()->now();
  gpsd_client::test::Tpv tpv;
  tpv.mode = MODE_NO_FIX;
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(tpv)));

  auto fix = fix_->next();
  ASSERT_TRUE(fix.has_value());
  EXPECT_GE(rclcpp::Time(fix->header.stamp).nanoseconds(), before.nanoseconds());
}

TEST_F(ClientNode, CheckFixByVarianceDropsOnlyTheNavSatFix)
{
  start({rclcpp::Parameter("check_fix_by_variance", true)});
  waitForDiscovery();
  gpsd_client::test::Tpv tpv = threeDFix();
  tpv.epx = NAN;
  tpv.epy = NAN;
  tpv.epv = NAN;
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(tpv)));

  // GPSFix still reports the report, as a fix it does not trust.
  auto extended = extended_fix_->next();
  ASSERT_TRUE(extended.has_value());
  EXPECT_EQ(gps_msgs::msg::GPSStatus::STATUS_NO_FIX, extended->status.status);
  EXPECT_TRUE(fix_->staysQuiet(500ms));

  // A report with its variances gets through.
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));
  EXPECT_TRUE(fix_->next().has_value());
}

TEST_F(ClientNode, WithoutCheckFixByVarianceMarksTheCovarianceUnknown)
{
  start();
  waitForDiscovery();
  gpsd_client::test::Tpv tpv = threeDFix();
  tpv.epv = NAN;
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(tpv)));

  auto fix = fix_->next();
  ASSERT_TRUE(fix.has_value());
  EXPECT_EQ(NavSatFix::COVARIANCE_TYPE_UNKNOWN, fix->position_covariance_type);
  EXPECT_EQ(0.0, fix->position_covariance[0]);
}

TEST_F(ClientNode, UncertaintyToSigmaScalesTheCovariance)
{
  start({rclcpp::Parameter("uncertainty_to_sigma", 1.0)});
  waitForDiscovery();
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));

  auto fix = fix_->next();
  ASSERT_TRUE(fix.has_value());
  EXPECT_DOUBLE_EQ(2.25, fix->position_covariance[0]);
  EXPECT_DOUBLE_EQ(12.25, fix->position_covariance[8]);
}

TEST_F(ClientNode, FallsBackToTheDefaultForAnInvalidUncertaintyToSigma)
{
  // Zero would divide by zero; the node warns and uses 1.96.
  start({rclcpp::Parameter("uncertainty_to_sigma", 0.0)});
  waitForDiscovery();
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));

  auto fix = fix_->next();
  ASSERT_TRUE(fix.has_value());
  EXPECT_DOUBLE_EQ(Variance(1.5), fix->position_covariance[0]);
  EXPECT_EQ(NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN, fix->position_covariance_type);
}

TEST_F(ClientNode, LegacyFixSemanticsKeepsTheOldNavSatFixCovariance)
{
  start({rclcpp::Parameter("legacy_fix_semantics", true)});
  waitForDiscovery();
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));

  auto fix = fix_->next();
  ASSERT_TRUE(fix.has_value());
  EXPECT_DOUBLE_EQ(1.5, fix->position_covariance[0]);
  EXPECT_DOUBLE_EQ(3.5, fix->position_covariance[8]);

  auto extended = extended_fix_->next();
  ASSERT_TRUE(extended.has_value());
  EXPECT_DOUBLE_EQ(Variance(1.5), extended->position_covariance[0]);
}

TEST_F(ClientNode, AdvertisesTheRawAndJsonTopicsOnlyWhenAsked)
{
  start();
  waitForDiscovery();
  EXPECT_EQ(0u, publishers("gpsd_raw"));
  EXPECT_EQ(0u, publishers("gpsd_json"));
  EXPECT_EQ(1u, publishers("fix"));
  EXPECT_EQ(1u, publishers("extended_fix"));
}

TEST_F(ClientNode, RawTopicCarriesTheReport)
{
  start({rclcpp::Parameter("publish_gpsd_raw", true)});
  waitForDiscovery(true, false);
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));

  auto raw = raw_->next();
  ASSERT_TRUE(raw.has_value());
  EXPECT_EQ(MODE_3D, raw->fix_mode);
  EXPECT_DOUBLE_EQ(29.44, raw->fix_latitude);
  EXPECT_DOUBLE_EQ(-98.61, raw->fix_longitude);
  EXPECT_NE(0u, raw->set & LATLON_SET);
}

TEST_F(ClientNode, JsonTopicCarriesEveryReportAndTheFixTopicsTheLatest)
{
  start({rclcpp::Parameter("publish_gpsd_json", true)});
  waitForDiscovery(false, true);
  const std::vector<std::string> reports = {
    gpsd_client::test::tpvJson(threeDFix(29.1)),
    gpsd_client::test::tpvJson(threeDFix(29.2)),
    gpsd_client::test::tpvJson(threeDFix(29.3)),
  };
  // One write, so all three are waiting when the node next reads.
  ASSERT_TRUE(fake_.send(reports[0] + "\r\n" + reports[1] + "\r\n" + reports[2] + "\r\n"));

  const std::vector<GPSDJson> json = json_->waitForCount(3);
  ASSERT_EQ(3u, json.size());
  for (size_t i = 0; i < reports.size(); ++i) {
    // libgps hands back the line as GPSd sent it, terminator and all.
    EXPECT_EQ(reports[i], json[i].json.substr(0, reports[i].size()));
    EXPECT_EQ("gps", json[i].header.frame_id);
  }

  auto fix = fix_->next();
  ASSERT_TRUE(fix.has_value());
  EXPECT_DOUBLE_EQ(29.3, fix->latitude);
}

TEST(PublishRate, IsClampedToWhatTheTimerCanRun)
{
  EXPECT_EQ(1, gpsd_client::clampPublishRate(-5));
  EXPECT_EQ(1, gpsd_client::clampPublishRate(0));
  EXPECT_EQ(1, gpsd_client::clampPublishRate(1));
  EXPECT_EQ(10, gpsd_client::clampPublishRate(10));
  EXPECT_EQ(1000, gpsd_client::clampPublishRate(1000));
  // 1000 / 1001 would be a 0 ms timer period.
  EXPECT_EQ(1000, gpsd_client::clampPublishRate(1001));
  EXPECT_EQ(1000, gpsd_client::clampPublishRate(5000));
}

TEST_F(ClientNode, PublishesAtAnOverlyHighPublishRate)
{
  start({rclcpp::Parameter("publish_rate", 5000)});
  waitForDiscovery();
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));
  EXPECT_TRUE(fix_->next(3s).has_value());
}

TEST_F(ClientNode, FallsBackToOneHertzForAnInvalidPublishRate)
{
  start({rclcpp::Parameter("publish_rate", 0)});
  waitForDiscovery();
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));
  EXPECT_TRUE(fix_->next(3s).has_value());
}

TEST_F(ClientNode, DoesNotHoldTheExecutor)
{
  start();
  // GPSd is connected but says nothing. A timer sharing the node's executor
  // should still fire at its own rate while the client polls.
  auto ticks = std::make_shared<std::atomic<int>>(0);
  auto tick = [ticks]() {++*ticks;};
  auto probe = node_->create_wall_timer(20ms, tick);
  std::this_thread::sleep_for(1s);
  probe->cancel();

  // 50 at full rate; allow for a loaded machine.
  EXPECT_GE(ticks->load(), 25);
}

TEST_F(ClientNode, KeepsRunningWhenGpsdGoesAway)
{
  start();
  waitForDiscovery();
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));
  ASSERT_TRUE(fix_->next().has_value());

  fake_.disconnect();
  // Nothing to crash on while it notices and reconnects, and GPSd sends
  // nothing more, so nothing more is published.
  EXPECT_TRUE(fix_->staysQuiet(1500ms));
  EXPECT_EQ(1u, publishers("fix"));
}

TEST_F(ClientNode, ReconnectsWhenGpsdComesBack)
{
  start();
  waitForDiscovery();
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));
  ASSERT_TRUE(fix_->next().has_value());

  // GPSd restarts: the node notices, closes, and reconnects after
  // reconnect_interval (1 s by default), asking it to stream again.
  fake_.disconnect();
  ASSERT_TRUE(fake_.acceptClient(5s)) << "the node never reconnected";
  ASSERT_FALSE(fake_.waitFor("?WATCH=").empty()) << "the node never restarted the stream";
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix(30.0))));
  auto fix = fix_->next();
  ASSERT_TRUE(fix.has_value());
  EXPECT_DOUBLE_EQ(30.0, fix->latitude);
}

TEST_F(ClientNode, RetriesWhenGpsdIsNotUpAtStart)
{
  // Nothing is listening when the node starts, so its first connection fails.
  fake_.stopListening();
  create();
  waitForDiscovery();
  std::this_thread::sleep_for(300ms);

  fake_.startListening();
  ASSERT_TRUE(fake_.acceptClient(5s)) << "the node never retried";
  ASSERT_FALSE(fake_.waitFor("?WATCH=").empty());
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));
  EXPECT_TRUE(fix_->next().has_value());
}

TEST_F(ClientNode, ZeroReconnectIntervalDoesNotReconnect)
{
  start({rclcpp::Parameter("reconnect_interval", 0.0)});
  waitForDiscovery();
  fake_.disconnect();
  EXPECT_FALSE(fake_.acceptClient(2500ms)) << "the node reconnected anyway";
}

TEST_F(ClientNode, ReconnectIntervalSpacesAttempts)
{
  start({rclcpp::Parameter("reconnect_interval", 2.0)});
  waitForDiscovery();
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));
  ASSERT_TRUE(fix_->next().has_value());

  fake_.disconnect();
  EXPECT_FALSE(fake_.acceptClient(1000ms)) << "reconnected well inside the 2 s interval";
  EXPECT_TRUE(fake_.acceptClient(5s)) << "never reconnected";
}

namespace
{

/// The managed node, driven through its lifecycle services as a lifecycle
/// manager would.
class LifecycleNode : public NodeTest<gpsd_client::GPSDClientLifecycleComponent>
{
protected:
  void SetUp() override
  {
    create();
    change_state_ = listener_->create_client<lifecycle_msgs::srv::ChangeState>(
      ns_ + "/gpsd_client/change_state");
    ASSERT_TRUE(change_state_->wait_for_service(5s));
  }

  void TearDown() override
  {
    change_state_.reset();
    NodeTest::TearDown();
  }

  bool transition(uint8_t id)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = id;
    auto future = change_state_->async_send_request(request);
    if (future.wait_for(10s) != std::future_status::ready) {
      return false;
    }
    return future.get()->success;
  }

  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_;
};

}  // namespace

TEST_F(LifecycleNode, ConnectsOnConfigureAndStreamsOnlyWhileActive)
{
  ASSERT_TRUE(transition(Transition::TRANSITION_CONFIGURE));
  ASSERT_TRUE(fake_.acceptClient()) << "configure did not connect";
  EXPECT_EQ("", fake_.waitFor("?WATCH=", 500ms)) << "configure alone started the stream";

  ASSERT_TRUE(transition(Transition::TRANSITION_ACTIVATE));
  const std::string watch = fake_.waitFor("?WATCH=");
  EXPECT_NE(watch.find("\"enable\":true"), std::string::npos) << watch;
  waitForDiscovery();
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));
  EXPECT_TRUE(fix_->next().has_value());

  ASSERT_TRUE(transition(Transition::TRANSITION_DEACTIVATE));
  const std::string unwatch = fake_.waitFor("?WATCH=");
  EXPECT_NE(unwatch.find("\"enable\":false"), std::string::npos) << unwatch;
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));
  EXPECT_TRUE(fix_->staysQuiet(500ms));

  // Reactivating resumes on the same connection.
  ASSERT_TRUE(transition(Transition::TRANSITION_ACTIVATE));
  EXPECT_NE(fake_.waitFor("?WATCH=").find("\"enable\":true"), std::string::npos);
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix(29.5))));
  auto fix = fix_->next();
  ASSERT_TRUE(fix.has_value());
  EXPECT_DOUBLE_EQ(29.5, fix->latitude);
}

TEST_F(LifecycleNode, ReconnectsWhileActive)
{
  ASSERT_TRUE(transition(Transition::TRANSITION_CONFIGURE));
  ASSERT_TRUE(fake_.acceptClient());
  ASSERT_TRUE(transition(Transition::TRANSITION_ACTIVATE));
  ASSERT_FALSE(fake_.waitFor("?WATCH=").empty());

  fake_.disconnect();
  ASSERT_TRUE(fake_.acceptClient(5s)) << "the active node never reconnected";
  ASSERT_FALSE(fake_.waitFor("?WATCH=").empty());
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));
  EXPECT_TRUE(fix_->next().has_value());
}

TEST_F(LifecycleNode, RetriesOnlyOnceActivatedAgain)
{
  ASSERT_TRUE(transition(Transition::TRANSITION_CONFIGURE));
  ASSERT_TRUE(fake_.acceptClient());
  ASSERT_TRUE(transition(Transition::TRANSITION_ACTIVATE));
  ASSERT_FALSE(fake_.waitFor("?WATCH=").empty());
  ASSERT_TRUE(transition(Transition::TRANSITION_DEACTIVATE));

  // An inactive node does not poll, so it neither notices nor retries.
  fake_.disconnect();
  EXPECT_FALSE(fake_.acceptClient(2500ms)) << "the inactive node reconnected";

  // Activating finds the link gone and reconnects rather than failing.
  ASSERT_TRUE(transition(Transition::TRANSITION_ACTIVATE));
  ASSERT_TRUE(fake_.acceptClient(5s)) << "activation never reconnected";
  ASSERT_FALSE(fake_.waitFor("?WATCH=").empty());
  ASSERT_TRUE(fake_.send(gpsd_client::test::tpvJson(threeDFix())));
  EXPECT_TRUE(fix_->next().has_value());
}

TEST_F(LifecycleNode, CleanupClosesTheConnection)
{
  ASSERT_TRUE(transition(Transition::TRANSITION_CONFIGURE));
  ASSERT_TRUE(fake_.acceptClient());
  ASSERT_TRUE(transition(Transition::TRANSITION_CLEANUP));
  EXPECT_TRUE(fake_.waitForClose());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
