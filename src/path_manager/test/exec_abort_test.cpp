// [ABORT] The WIRE contract for withdrawing a trajectory already published.
//
// Everything else about this feature is covered by in-process assertions
// (chain_experiment_test's `envchange` drives the detection directly). What
// those cannot see is the plumbing: whether a second message actually goes
// out, whether a consumer that joins late gets it, and whether an abort is
// obeyed only by the trajectory it names. That is what this binary is for —
// it subscribes with the same QoS a real follower would and reads what the
// planner published.
//
// It does NOT construct a ReplanFSM: that needs the whole node (timers,
// subscriptions, a params file, a terrain map). It publishes the two messages
// itself, exactly as ReplanFSM::publishExecutionAbort and polyTraj2ROSMsg
// build them, and pins the contract a consumer must implement against them.
// The producer side is pinned in chain_experiment_test; this pins the seam.

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <set>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "mmp_traj_msgs/msg/poly_traj.hpp"
#include "mmp_traj_msgs/msg/trajectory_execution_control.hpp"

using Ctl = mmp_traj_msgs::msg::TrajectoryExecutionControl;
using Traj = mmp_traj_msgs::msg::PolyTraj;

namespace {

int g_failed = 0;

void expect(bool cond, const std::string &label) {
  std::cout << "  " << (cond ? "[OK]   " : "[FAIL] ") << label << "\n";
  if (!cond) ++g_failed;
}

// The consumer contract, as a consumer would implement it. This is the thing
// under test: given a stream of trajectories and controls, what is it flying?
class Follower {
 public:
  void onTraj(const Traj &t) {
    // A trajectory whose id has already been aborted must not start
    // executing — this is the late-joiner case, where TRANSIENT_LOCAL
    // delivers the latched trajectory and its abort in the same batch and
    // the order is not guaranteed.
    if (aborted_.count(t.trajectory_id)) {
      ignored_on_arrival_.push_back(t.trajectory_id);
      return;
    }
    executing_ = t.trajectory_id;
    has_traj_ = true;
  }
  void onCtl(const Ctl &c) {
    if (c.action != Ctl::ACTION_ABORT) return;
    aborted_.insert(c.trajectory_id);
    // Only stop if it names what is being flown. A late or replayed abort
    // must never stop a NEWER trajectory.
    if (has_traj_ && executing_ == c.trajectory_id) {
      has_traj_ = false;
      ++stops_;
    }
  }
  bool executing() const { return has_traj_; }
  uint64_t id() const { return executing_; }
  int stops() const { return stops_; }
  const std::vector<uint64_t> &ignoredOnArrival() const {
    return ignored_on_arrival_;
  }

 private:
  std::set<uint64_t> aborted_;
  std::vector<uint64_t> ignored_on_arrival_;
  uint64_t executing_ = 0;
  bool has_traj_ = false;
  int stops_ = 0;
};

Traj makeTraj(uint64_t id) {
  Traj t;
  t.drone_id = 0;
  t.trajectory_id = id;
  t.order = 5;
  t.duration = {1.0};
  t.coef_x.assign(6, 0.0f);
  t.coef_y.assign(6, 0.0f);
  t.coef_z.assign(6, 0.0f);
  return t;
}

Ctl makeAbort(uint64_t id, uint8_t reason) {
  Ctl c;
  c.drone_id = 0;
  c.trajectory_id = id;
  c.action = Ctl::ACTION_ABORT;
  c.reason = reason;
  c.detail = "test";
  return c;
}

void spin(rclcpp::Node::SharedPtr n, int ms) {
  const auto t0 = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - t0 <
         std::chrono::milliseconds(ms)) {
    rclcpp::spin_some(n);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("exec_abort_test");

  // The QoS the planner publishes with: RELIABLE + TRANSIENT_LOCAL, so a
  // consumer that joins after the fact still receives both.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
  auto traj_pub = node->create_publisher<Traj>("/planning/trajectory", qos);
  auto ctl_pub =
      node->create_publisher<Ctl>("/planning/execution_control", qos);

  std::cout << "== consumer already listening ==\n";
  {
    Follower f;
    auto s1 = node->create_subscription<Traj>(
        "/planning/trajectory", qos,
        [&f](Traj::SharedPtr m) { f.onTraj(*m); });
    auto s2 = node->create_subscription<Ctl>(
        "/planning/execution_control", qos,
        [&f](Ctl::SharedPtr m) { f.onCtl(*m); });
    spin(node, 200);

    traj_pub->publish(makeTraj(7));
    spin(node, 200);
    expect(f.executing() && f.id() == 7, "a published trajectory is executed");

    ctl_pub->publish(makeAbort(7, Ctl::REASON_OBSTACLE_BLOCKED));
    spin(node, 200);
    expect(!f.executing(),
           "an ABORT naming it stops the follower — this is the step a "
           "zeroed planner-side slot cannot perform");
    expect(f.stops() == 1, "...exactly once");

    // Idempotence: the same abort again changes nothing.
    ctl_pub->publish(makeAbort(7, Ctl::REASON_OBSTACLE_BLOCKED));
    spin(node, 200);
    expect(f.stops() == 1, "a duplicate ABORT is a no-op");

    // A NEW trajectory after the abort flies. The abort was about id 7.
    traj_pub->publish(makeTraj(8));
    spin(node, 200);
    expect(f.executing() && f.id() == 8,
           "a new trajectory issued after the abort is executed");

    // A stale abort for the OLD id must not stop it.
    ctl_pub->publish(makeAbort(7, Ctl::REASON_OBSTACLE_BLOCKED));
    spin(node, 200);
    expect(f.executing() && f.id() == 8,
           "a replayed ABORT for an older id does NOT stop the current "
           "trajectory — this is why the id exists and start_time cannot "
           "stand in for it");
  }

  std::cout << "== consumer joins late (latched) ==\n";
  {
    // Everything above is still latched on both topics. A follower that
    // connects now receives the last trajectory AND its abort, in an order
    // it does not control, and must end up flying nothing that was aborted.
    Follower f;
    auto s1 = node->create_subscription<Traj>(
        "/planning/trajectory", qos,
        [&f](Traj::SharedPtr m) { f.onTraj(*m); });
    auto s2 = node->create_subscription<Ctl>(
        "/planning/execution_control", qos,
        [&f](Ctl::SharedPtr m) { f.onCtl(*m); });
    spin(node, 400);
    expect(f.id() != 7 || !f.executing(),
           "a late consumer does not start flying an aborted trajectory");
    expect(!f.ignoredOnArrival().empty() || f.id() == 8,
           "...it either discarded it on arrival or moved on to the newer one");
  }

  std::cout << "== reasons are distinguishable ==\n";
  {
    Ctl c = makeAbort(9, Ctl::REASON_POLICY_UNEVALUATED);
    expect(c.reason != Ctl::REASON_OBSTACLE_BLOCKED,
           "\"we cannot judge it\" is not the same wire value as \"it is "
           "blocked\" — both withdraw, they are not the same event");
    expect(Ctl::ACTION_ABORT == 1, "ACTION_ABORT is the documented constant");
  }

  rclcpp::shutdown();
  if (g_failed == 0) {
    std::cout << "PASS: 0 failed check(s)\n";
    return 0;
  }
  std::cout << "FAIL: " << g_failed << " failed check(s)\n";
  return 1;
}
