#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>

#include <gpsd_client/gpsd_client_component.hpp>
#include <gpsd_client/gpsd_client_lifecycle_component.hpp>

/* These exercise the managed node's state machine, not its connection: none
 * of them needs a GPSd, and none of them may find one either. Every case
 * points the node at a port nothing listens on, so gps_open() fails the same
 * way whether or not the machine running the tests happens to have a GPSd on
 * 2947, such as on the buildfarm or local dev machine
 */
namespace
{

rclcpp::NodeOptions unreachableGpsd()
{
  rclcpp::NodeOptions options;
  // 127.0.0.1 rather than a bogus hostname: connection refused comes back
  // immediately, where a name that does not resolve costs a DNS timeout.
  options.parameter_overrides({
    rclcpp::Parameter("host", "127.0.0.1"),
    rclcpp::Parameter("port", 1),
  });
  return options;
}

class LifecycleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<gpsd_client::GPSDClientLifecycleComponent>(unreachableGpsd());
  }

  void TearDown() override
  {
    node_.reset();
  }

  uint8_t currentState() const
  {
    return node_->get_current_state().id();
  }

  std::shared_ptr<gpsd_client::GPSDClientLifecycleComponent> node_;
};

TEST_F(LifecycleTest, StartsUnconfigured)
{
  // The point of the managed node: constructing it connects to nothing and
  // publishes nothing until something tells it to.
  EXPECT_EQ(currentState(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LifecycleTest, AdvertisesLifecycleServices)
{
  const std::string ns = std::string(node_->get_node_base_interface()->get_fully_qualified_name()) + "/";
  const std::vector<std::string> expected{
    ns + "change_state", ns + "get_state", ns + "get_available_states",
    ns + "get_available_transitions"};

  /* The services exist as soon as the node does, but they reach the graph
   * through discovery, so a single look can be too early. Poll rather than
   * sleep a fixed amount: the wait is nearly always over on the first pass.
   */
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  std::map<std::string, std::vector<std::string>> services;
  bool all_present = false;
  while (!all_present && std::chrono::steady_clock::now() < deadline)
  {
    services = node_->get_service_names_and_types();
    all_present = std::all_of(expected.begin(), expected.end(),
      [&services](const std::string & name) { return services.count(name) > 0; });
    if (!all_present)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  for (const std::string & service : expected)
  {
    EXPECT_GT(services.count(service), 0u) << "missing lifecycle service " << service;
  }
}

TEST_F(LifecycleTest, ConfigureFailsWithoutGpsd)
{
  /* A failed configure leaves the node unconfigured rather than throwing or
   * landing in the error state, so the caller can fix the parameters and try
   * again.
   */
  node_->configure();
  EXPECT_EQ(currentState(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LifecycleTest, ShutdownFromUnconfiguredIsClean)
{
  node_->shutdown();
  EXPECT_EQ(currentState(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
}

/* The unmanaged node has no transitions to drive, so what is worth asserting
 * is that an unreachable GPSd leaves it constructed and quiet rather than
 * throwing out of a component container's load.
 */
TEST(UnmanagedNode, SurvivesUnreachableGpsd)
{
  std::shared_ptr<gpsd_client::GPSDClientComponent> node;
  ASSERT_NO_THROW(
    node = std::make_shared<gpsd_client::GPSDClientComponent>(unreachableGpsd()));
  EXPECT_EQ(std::string(node->get_name()), "gpsd_client");
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
