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
#include "mmp_vehicle_dynamics/trajectory_abort_state.hpp"

using Ctl = mmp_traj_msgs::msg::TrajectoryExecutionControl;
using Traj = mmp_traj_msgs::msg::PolyTraj;

namespace {

int g_failed = 0;

void expect(bool cond, const std::string &label) {
  std::cout << "  " << (cond ? "[OK]   " : "[FAIL] ") << label << "\n";
  if (!cond) ++g_failed;
}

// The consumer, wrapping the SHARED state machine — the same header
// dynamics_sim_node uses. It used to carry its own copy of the logic, which
// meant this test could pass while the simulator did something else (or
// nothing). Now a defect in the contract fails here AND changes the
// simulator's behaviour, because it is one implementation.
//
// What this still cannot see is the simulator's WIRING: if it stopped
// subscribing to the control topic, nothing here would notice. That needs the
// real node in the loop and is recorded as open.
class Follower {
 public:
  void onTraj(const Traj &t) {
    if (!state_.acceptTrajectory(t.trajectory_id))
      ignored_on_arrival_.push_back(t.trajectory_id);
  }
  void onCtl(const Ctl &c) {
    if (c.action != Ctl::ACTION_ABORT) return;
    if (state_.applyAbort(c.trajectory_id)) ++stops_;
  }
  bool executing() const { return state_.executing(); }
  uint64_t id() const { return state_.current(); }
  int stops() const { return stops_; }
  const std::vector<uint64_t> &ignoredOnArrival() const {
    return ignored_on_arrival_;
  }

 private:
  mmp_vehicle_dynamics::TrajectoryAbortState state_;
  std::vector<uint64_t> ignored_on_arrival_;
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

  // EXACTLY the planner's QoS (replan_fsm.cpp: rclcpp::QoS(5).reliable()
  // .transient_local()). Depth matters for the late-joiner case: with a
  // deeper queue than production the latched history a consumer receives is
  // not the history it would really receive.
  auto qos = rclcpp::QoS(5).reliable().transient_local();
  auto traj_pub = node->create_publisher<Traj>("/planning/trajectory", qos);
  auto ctl_pub =
      node->create_publisher<Ctl>("/planning/execution_control", qos);

  // The four consumer rules, driven directly so delivery order cannot decide
  // the outcome. The topic sections below cover the wiring and the QoS; these
  // cover the decisions, and they are the same code the simulator runs.
  std::cout << "== the contract itself (shared state machine) ==\n";
  {
    mmp_vehicle_dynamics::TrajectoryAbortState st;
    expect(st.acceptTrajectory(1) && st.executing() && st.current() == 1,
           "a trajectory is accepted and becomes the one being executed");
    expect(st.applyAbort(1) && !st.executing(),
           "an abort naming it stops execution");

    expect(st.acceptTrajectory(2) && st.executing(),
           "a NEW trajectory after that abort is accepted");
    expect(!st.applyAbort(1) && st.executing() && st.current() == 2,
           "a replayed abort for the OLD id does not stop it");
    expect(!st.applyAbort(1) && st.executing(),
           "...however many times it is replayed");

    // Abort FIRST, trajectory LATER. Both topics are TRANSIENT_LOCAL, so a
    // late joiner gets both and the order is not guaranteed — driven here
    // rather than published, because publishing cannot pin which arrives
    // first, and this is precisely the ordering the arrival check exists for.
    mmp_vehicle_dynamics::TrajectoryAbortState st2;
    expect(!st2.applyAbort(9),
           "an abort for a trajectory never seen stops nothing");
    expect(!st2.acceptTrajectory(9) && !st2.executing(),
           "...and that trajectory is REFUSED when it later arrives — the "
           "case a published test cannot pin, because it cannot choose the "
           "delivery order");
    expect(st2.acceptTrajectory(10) && st2.executing(),
           "a different id still starts normally");
  }

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

  std::cout << "== consumer joins late, ONLY an aborted trajectory latched ==\n";
  {
    // The dangerous case in isolation: the newest latched trajectory is the
    // aborted one, so "moved on to a newer id" cannot mask the check. Abort
    // the CURRENT id (8) and let a fresh consumer connect to exactly that.
    ctl_pub->publish(makeAbort(8, Ctl::REASON_TERRAIN_BLOCKED));
    spin(node, 200);

    Follower f;
    auto s1 = node->create_subscription<Traj>(
        "/planning/trajectory", qos,
        [&f](Traj::SharedPtr m) { f.onTraj(*m); });
    auto s2 = node->create_subscription<Ctl>(
        "/planning/execution_control", qos,
        [&f](Ctl::SharedPtr m) { f.onCtl(*m); });
    spin(node, 500);
    expect(!f.executing(),
           "a late consumer whose newest latched trajectory is aborted flies "
           "NOTHING — there is no newer id to fall back on here");
  }

  std::cout << "== consumer joins late (mixed history) ==\n";
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
