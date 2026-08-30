// [V-2] 명령된 시작 위치가 플래너 수명당 한 번만 반영되는가 — 생산 배선으로.
//
// 감사 V-2 의 재현이다. 손으로 만든 축소판이 아니라 진짜 ReplanFSM 을 세우고
// 진짜 콜백을 부르고 진짜로 발행된 궤적을 읽는다. 그렇게 하는 이유는 이
// 저장소 자신이 겪은 일 때문이다 — 커밋 48bda7f 의 기록:
//
//   "The wire test could not see it: it published ids 7 and 8 by hand and so
//    tested the contract in the abstract while production violated it."
//
// 관측점은 발행된 PolyTraj 다. 계수는 최고차 우선이므로 piece 0 의 t=0 위치는
// (coef_x[5], coef_y[5], coef_z[5]) 이고, 그것이 AGL 해소 뒤의 start_pt_ 다.
// start_pt_ 는 private 이지만 접근자를 새로 뚫지 않는다 — 운용자가 실제로 보는
// 것은 발행된 궤적이지 멤버가 아니다.
//
// 순서가 임의가 아니다. 두 가지가 동시에 성립해야 결함이 드러난다.
//
//   1. 명령이 초기 상태를 진술해야 한다. 진술하지 않으면 triggerGlobalPlan 이
//      INITIAL_STATE_UNSPECIFIED 로 거절하고(replan_fsm.cpp:837-851) 계획이
//      실패하며, 실패 롤백(:1246-1247)이 current_mission_id_ 를 되돌린다.
//      그러면 두 번째 명령은 채택되어 버려서 결함이 사라진 것처럼 보인다.
//   2. 두 명령 사이에 FSM 이 실제로 돌아야 한다. have_local_traj_ 는
//      planFromGlobalTraj(:518)에서만 참이 되고 그것은 타이머 본문에서만
//      닿는다. 스핀 없이 콜백만 두 번 부르면 :628 분기를 두 번 타서
//      "결함이 없다" 로 보인다.
//
// 두 함정 다 이 시험을 처음 쓸 때 실제로 밟았다. 적어 두지 않으면 다음 사람이
// 같은 자리에서 통과하는 거짓 시험을 만든다.
//
// 이 시험이 못박는 것과 못박지 않는 것:
//   - 못박는다: 발행자가 mission_id 를 고정하면 두 번째 미션이 자기 시작점에서
//     계획되지 않는다. 그리고 mission_id 만 바꾸면 계획된다(단일변수 대조).
//   - 못박지 않는다: "진술된 시작이 비행 중 궤적을 이겨야 한다" 는 계약.
//     그것은 docs/design/open_questions.md 의 initial-speed / SEMANTICS 로
//     사람 결정이 걸려 있다. 그 결정이 내려지면 아래 (2) 의 기대가 뒤집히고,
//     그때는 의도적으로 이 파일을 고쳐야 한다.
//
// 실제 솔브를 돌리므로 chain_experiment_test 와 같은 부류다.

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include "path_manager/replan_fsm.h"
#include "mmp_mission_msgs/msg/trajectory_command.hpp"
#include "mmp_traj_msgs/msg/poly_traj.hpp"

using Traj = mmp_traj_msgs::msg::PolyTraj;

namespace {

int g_failed = 0;
void expect(bool c, const std::string &label) {
  std::cout << "  " << (c ? "[OK]   " : "[FAIL] ") << label << "\n";
  if (!c) ++g_failed;
}

// 평평한 DEM. 기하가 매번 같아야 재수신이 "지형이 바뀌었다" 분기를 타지 않는다.
grid_map_msgs::msg::GridMap::SharedPtr makeFlatMap(double height) {
  auto msg = std::make_shared<grid_map_msgs::msg::GridMap>();
  constexpr double kRes = 2.0, kLx = 360.0, kLy = 300.0;
  constexpr int kRows = static_cast<int>(kLx / kRes);
  constexpr int kCols = static_cast<int>(kLy / kRes);
  msg->info.resolution = kRes;
  msg->info.length_x = kLx;
  msg->info.length_y = kLy;
  msg->info.pose.position.x = kLx / 2.0;
  msg->info.pose.position.y = kLy / 2.0;
  msg->layers.push_back("elevation");
  msg->data.resize(1);
  auto &layer = msg->data.front();
  layer.layout.dim.resize(2);
  layer.layout.dim[0].label = "column_index";
  layer.layout.dim[0].size = kCols;
  layer.layout.dim[0].stride = kCols * kRows;
  layer.layout.dim[1].label = "row_index";
  layer.layout.dim[1].size = kRows;
  layer.layout.dim[1].stride = kRows;
  layer.data.assign(static_cast<size_t>(kCols) * kRows,
                    static_cast<float>(height));
  return msg;
}

// 패널이 내보내는 것과 같은 모양의 명령. mission_id 는 인자로 받는다 — 그것이
// 이 시험의 유일한 독립변수다.
mmp_mission_msgs::msg::TrajectoryCommand::SharedPtr makeMission(
    int seq, const std::string &mission_id,
    const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
    double speed_mps = 150.0) {
  auto m = std::make_shared<mmp_mission_msgs::msg::TrajectoryCommand>();
  m->drone_id = 0;              // SEQUENTIAL_START 가 drone_id_ <= 0 에 걸림
  m->sequence = seq;
  m->mission_id = mission_id;
  m->start_position.x = start.x();
  m->start_position.y = start.y();
  m->start_position.z = start.z();
  m->target_position.x = goal.x();
  m->target_position.y = goal.y();
  m->target_position.z = goal.z();
  // 초기 상태를 진술하지 않으면 계획이 거절되고 롤백이 결함을 감춘다.
  m->use_initial_speed = true;
  m->initial_speed = speed_mps;   // 기본 150 — 순항 대역 안 (122..200 m/s)
  return m;
}

void spin(rclcpp::Node::SharedPtr n, int ms) {
  const auto t0 = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(ms)) {
    rclcpp::spin_some(n);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

// piece 0 의 t=0 위치. 계수는 최고차 우선이므로 상수항이 마지막이다.
Eigen::Vector3d startOf(const Traj &t) {
  if (t.coef_x.size() < 6 || t.coef_y.size() < 6 || t.coef_z.size() < 6) {
    return Eigen::Vector3d::Constant(std::nan(""));
  }
  return Eigen::Vector3d(t.coef_x[5], t.coef_y[5], t.coef_z[5]);
}

double xyDist(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  return (a.head<2>() - b.head<2>()).norm();
}

void report(const char *tag, const Eigen::Vector3d &commanded,
            const Eigen::Vector3d &published) {
  std::cout << "  .. " << tag
            << "  명령 (" << commanded.x() << ", " << commanded.y() << ")"
            << "  발행 (" << published.x() << ", " << published.y() << ")"
            << "  xy 차이 " << xyDist(commanded, published) << " u\n";
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage: mission_sequence_test <optimizer_params.yaml>\n";
    return 2;
  }
  rclcpp::init(argc, argv);

  // ReplanFSM 과 PathManager 가 생성자에서 약 90개를 선언한다. 여기서 하나라도
  // 먼저 선언하면 ParameterAlreadyDeclaredException 이 난다 — 전부 params 파일과
  // NodeOptions 로만 넣는다. (chain_experiment_test 가 drone_id 를 직접 선언하는
  // 것은 그쪽이 PathManager 만 세우기 때문이고, 여기서는 그 줄이 던진다.)
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", argv[1]});
  options.parameter_overrides({
      rclcpp::Parameter("drone_id", 0),
      rclcpp::Parameter("chain/enable", false),
  });
  auto node = rclcpp::Node::make_shared("mission_sequence", options);
  path_manager::ReplanFSM fsm(node);

  std::vector<Traj> trajs;
  auto qos = rclcpp::QoS(5).reliable().transient_local();
  auto sub = node->create_subscription<Traj>(
      "/planning/trajectory", qos,
      [&trajs](Traj::SharedPtr m) { trajs.push_back(*m); });
  spin(node, 300);

  fsm.terrainCallback(makeFlatMap(0.0));
  spin(node, 200);

  // 세 시작점. B 와 C 는 A→목표 직선에서 멀리 떨어뜨린다 — 궤적 위의 점과
  // 명령된 점을 헷갈리지 않으려면 그래야 한다.
  const Eigen::Vector3d kGoal(330.0, 150.0, 1.5);
  const Eigen::Vector3d A(30.0, 150.0, 3.0);
  const Eigen::Vector3d B(60.0,  60.0, 3.0);
  const Eigen::Vector3d C(60.0, 240.0, 3.0);
  const double kSame = 2.0;    // u — 같다고 볼 최대 거리 (1 u = 100 m)
  const double kApart = 10.0;  // u — 다르다고 볼 최소 거리

  // ── (0) 관측 채널이 실제로 동작하는지 먼저 보인다 ──────────────────
  // 이것을 건너뛰면 아래 두 검사가 "아무것도 발행되지 않았다" 를 "결함" 으로
  // 잘못 읽는다.
  fsm.trajectoryCommandCallback(makeMission(1, "mission_config_panel", A, kGoal));
  spin(node, 6000);
  expect(trajs.size() == 1, "첫 미션이 계획되고 발행된다");
  if (trajs.size() != 1) {
    std::cout << "FAIL: 첫 계획이 없으면 나머지는 측정이 아니다 ("
              << ++g_failed << ")\n";
    rclcpp::shutdown();
    return 1;
  }
  report("미션1", A, startOf(trajs[0]));
  expect(xyDist(startOf(trajs[0]), A) < kSame,
         "첫 미션은 명령된 시작점에서 계획된다 — 관측 채널 확인");

  // ── (1) 같은 mission_id, 다른 시작점 ────────────────────────────────
  // 패널이 오늘 내보내는 모양 그대로다(mission_config_panel.cpp:359 는 상수).
  // mission_changed 가 거짓이 되어 :1156 채택 블록이 통째로 건너뛰어진다.
  const size_t n1 = trajs.size();
  fsm.trajectoryCommandCallback(makeMission(2, "mission_config_panel", B, kGoal));
  spin(node, 6000);
  expect(trajs.size() > n1, "두 번째 미션도 계획되고 발행된다 — 거절이 아니다");
  if (trajs.size() > n1) {
    const Eigen::Vector3d p = startOf(trajs.back());
    report("미션2(같은 id)", B, p);
    // 현재 동작을 못박는다. 뒤집히면 (나) 계약이 들어온 것이고, 그때는 이
    // 기대를 의도적으로 고쳐야 한다.
    expect(xyDist(p, B) > kApart,
           "같은 mission_id 로는 명령된 시작점이 무시된다 — V-2 재현");
  }

  // ── (2) 단일변수 대조: mission_id 만 바꾼다 ─────────────────────────
  // 시작점도 목표도 초기속도도 그대로 두고 id 만 바꾼다. 이것이 통과해야
  // "발행자가 Run 마다 새 id 를 내면 충분하다" 가 성립한다.
  const size_t n2 = trajs.size();
  fsm.trajectoryCommandCallback(makeMission(3, "mission_config_panel#3", C, kGoal));
  spin(node, 6000);
  expect(trajs.size() > n2, "세 번째 미션도 발행된다");
  if (trajs.size() > n2) {
    const Eigen::Vector3d p = startOf(trajs.back());
    report("미션3(다른 id)", C, p);
    expect(xyDist(p, C) < kSame,
           "mission_id 만 바꾸면 명령된 시작점이 채택된다 — 단일변수 대조");
  }

  // ── (3) 거부된 미션이 비행 중 궤적을 떨어뜨리지 않는다 ──────────────
  // 채택 블록은 start_pt_ / current_pos_ / start_position_received_ /
  // start_seed_agl_pending_ / have_local_traj_ 다섯을 바꾸는데, 실패 롤백
  // (:1246-1247)은 sequence 와 mission_id 둘만 되돌린다. 그래서 downstream
  // 거부는 날고 있던 궤적을 버리고 거부된 미션의 시작점을 남겼다 —
  // replan_fsm.cpp:1068-1074 주석이 스스로 적어 둔 결함이다.
  //
  // 발행자가 Run 마다 새 id 를 내기 시작하면 채택이 매 Run 실행되므로 이
  // 결함이 첫 명령에서만 나던 것에서 매 Run 으로 살아난다. 그래서 두 절반이
  // 한 변경이다.
  const size_t n3 = trajs.size();
  const Eigen::Vector3d D(60.0, 60.0, 3.0);        // 미션3 궤적에서 먼 곳
  // 인계 상한(200 m/s)을 넘는 진술 속도. 1단계 검증은 통과하고(형식은 멀쩡
  // 하다) 채택 뒤 stateEnvelopeProblem 이 INITIAL_MODE_UNSUPPORTED 로 거부
  // 한다 — 채택이 이미 상태를 바꿔 놓은 다음이라는 것이 요점이다.
  // (지도 밖 목표로는 안 된다: 확인해 보니 그건 그냥 계획된다.)
  fsm.trajectoryCommandCallback(
      makeMission(4, "rejected#4", D, kGoal, /*speed_mps=*/250.0));
  spin(node, 6000);
  expect(trajs.size() == n3, "상한을 넘는 진술 속도는 계획되지 않는다 — 거부 경로 확인");

  if (trajs.size() == n3) {
    // 미션3 의 연속으로 오는 명령(같은 id). 채택은 건너뛰어지므로 시작점은
    // 날고 있던 궤적에서 와야 한다. 롤백이 부족하면 거부된 미션4 의 시작점
    // D 가 그대로 남아 거기서 계획된다.
    const Eigen::Vector3d dontCare(10.0, 10.0, 3.0);
    fsm.trajectoryCommandCallback(
        makeMission(5, "mission_config_panel#3", dontCare, kGoal));
    spin(node, 6000);
    expect(trajs.size() > n3, "거부 뒤의 연속 명령은 계획된다");
    if (trajs.size() > n3) {
      const Eigen::Vector3d p = startOf(trajs.back());
      report("미션5(거부 뒤 연속)", D, p);
      expect(xyDist(p, D) > kApart,
             "거부된 미션의 시작점이 남아 있지 않다 — 롤백이 채택을 되돌린다");
    }
  }

  rclcpp::shutdown();
  if (g_failed == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
  std::cout << "FAIL: " << g_failed << " failed check(s)\n";
  return 1;
}
