// [V-7] 거부가 운용자에게 보이는가 — 네 가지 설정 조합 전부에서.
//
// FSM_LOG_WARN/ERROR 가 INFO 와 같은 XOR 이었다. 출하 기본값
// enable_debug_logs: true 에서는 파일 로거로만 가고 콘솔에 아무것도 닿지
// 않았고, 머리 없는 스모크가 쓰는 SWARM_DISABLE_FILE_LOGGING=1 이 겹치면
// 파일 자체가 안 열려 거부가 어디에도 남지 않았다.
//
// LogManager::setMinLevel 은 이 문제의 해법이 아니다. LogManager::log 는
// 파일에만 쓰고 min_level_ 은 그 파일 기록만 거른다 — 콘솔로 나가는 경로가
// 클래스 안에 없다. 그래서 이 시험은 수준이 아니라 **목적지**를 본다.
//
// 관측 방법: stderr 를 파일로 돌려 rclcpp 가 실제로 콘솔에 쓴 것을 읽는다.
// 로그 문자열을 소스에서 grep 하는 것으로는 XOR 이 살아 있어도 통과한다.

#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include "path_manager/replan_fsm.h"
#include "mmp_mission_msgs/msg/trajectory_command.hpp"

namespace fs = std::filesystem;

namespace {

int g_failed = 0;
void expect(bool c, const std::string &label) {
  std::cout << "  " << (c ? "[OK]   " : "[FAIL] ") << label << "\n";
  if (!c) ++g_failed;
}

grid_map_msgs::msg::GridMap::SharedPtr makeFlatMap() {
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
  layer.data.assign(static_cast<size_t>(kCols) * kRows, 0.0f);
  return msg;
}

mmp_mission_msgs::msg::TrajectoryCommand::SharedPtr makeCmd(
    int seq, const std::string &mission_id, double speed_mps) {
  auto m = std::make_shared<mmp_mission_msgs::msg::TrajectoryCommand>();
  m->drone_id = 0;
  m->sequence = seq;
  m->mission_id = mission_id;
  m->start_position.x = 30.0;
  m->start_position.y = 150.0;
  m->start_position.z = 3.0;
  m->target_position.x = 330.0;
  m->target_position.y = 150.0;
  m->target_position.z = 1.5;
  m->use_initial_speed = true;
  m->initial_speed = speed_mps;
  return m;
}

void spin(rclcpp::Node::SharedPtr n, int ms) {
  const auto t0 = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(ms)) {
    rclcpp::spin_some(n);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

// stderr 를 파일로 돌린다. freopen 이 아니라 fd 를 바꿔서 되돌릴 수 있게 한다.
class StderrCapture {
 public:
  explicit StderrCapture(const std::string &path) : path_(path) {
    std::fflush(stderr);
    saved_ = ::dup(STDERR_FILENO);
    fd_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    ::dup2(fd_, STDERR_FILENO);
  }
  std::string stop() {
    if (saved_ < 0) return text_;
    std::fflush(stderr);
    ::dup2(saved_, STDERR_FILENO);
    ::close(fd_);
    ::close(saved_);
    saved_ = -1;
    std::ifstream in(path_);
    std::stringstream ss;
    ss << in.rdbuf();
    text_ = ss.str();
    return text_;
  }
  ~StderrCapture() { stop(); }

 private:
  std::string path_, text_;
  int saved_{-1}, fd_{-1};
};

size_t countOf(const std::string &hay, const std::string &needle) {
  size_t n = 0, p = 0;
  while ((p = hay.find(needle, p)) != std::string::npos) { ++n; p += needle.size(); }
  return n;
}

// 이 노드 이름으로 열린 로그 파일들.
std::vector<fs::path> logFilesFor(const std::string &node_name) {
  std::vector<fs::path> out;
  const fs::path dir{"./logs/runtime"};
  std::error_code ec;
  if (!fs::exists(dir, ec)) return out;
  for (const auto &e : fs::directory_iterator(dir, ec)) {
    if (e.is_regular_file() &&
        e.path().filename().string().rfind(node_name, 0) == 0) {
      out.push_back(e.path());
    }
  }
  return out;
}

std::string readAll(const fs::path &p) {
  std::ifstream in(p);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage: log_visibility_test <optimizer_params.yaml>\n";
    return 2;
  }
  rclcpp::init(argc, argv);

  // 콘솔에서 찾을 문자열. 소스의 포맷 문자열 앞부분과 같아야 한다.
  const std::string kRejected = "[INITIAL-STATE] command REJECTED";
  const std::string kDegraded = "[PLAN] DEGRADED";
  const std::string kNormalInfo = "Trajectory command (seq";

  struct Combo { bool debug_logs; bool file_logging; const char *name; };
  const Combo combos[] = {
      {false, false, "logvis_d0f0"},
      {false, true,  "logvis_d0f1"},
      {true,  false, "logvis_d1f0"},
      {true,  true,  "logvis_d1f1"},
  };

  for (const auto &c : combos) {
    std::cout << "\n── enable_debug_logs=" << (c.debug_logs ? "true " : "false")
              << "  file_logging=" << (c.file_logging ? "true " : "false")
              << " ──\n";
    // 파일 로깅은 LogManager 생성자가 환경변수로 읽는다. FSM 을 만들기 전에
    // 세워야 한다.
    ::setenv("SWARM_DISABLE_FILE_LOGGING", c.file_logging ? "0" : "1", 1);

    const std::string cap = std::string("/tmp/") + c.name + ".stderr";
    std::string console;
    {
      StderrCapture capture(cap);
      rclcpp::NodeOptions opt;
      opt.arguments({"--ros-args", "--params-file", argv[1]});
      opt.parameter_overrides({
          rclcpp::Parameter("drone_id", 0),
          rclcpp::Parameter("chain/enable", false),
          rclcpp::Parameter("enable_debug_logs", c.debug_logs),
          // sub-stall final speed 를 FAILED 가 아니라 DEGRADED 로 만든다.
          rclcpp::Parameter("planning/allow_final_boundary_relaxation", true),
      });
      auto node = rclcpp::Node::make_shared(c.name, opt);
      path_manager::ReplanFSM fsm(node);
      fsm.terrainCallback(makeFlatMap());
      spin(node, 200);

      // (1) MALFORMED: 진술은 했는데 값이 유한하지 않다. 1단계에서 거부되고
      //     아무 상태도 소비되지 않는다.
      auto bad = makeCmd(1, "malformed#1",
                         std::numeric_limits<double>::quiet_NaN());
      fsm.trajectoryCommandCallback(bad);
      spin(node, 200);

      // (2) DEGRADED: 정상 계획인데 최종 속도가 실속 아래 — 완화 opt-in 이
      //     켜져 있으므로 날되 시끄럽게.
      auto deg = makeCmd(2, "degraded#2", 150.0);
      deg->use_final_velocity = true;
      deg->final_velocity.x = 5.0;   // sub-stall
      deg->final_velocity.y = 0.0;
      deg->final_velocity.z = 0.0;
      fsm.trajectoryCommandCallback(deg);
      spin(node, 5000);

      console = capture.stop();
    }

    const size_t n_rej = countOf(console, kRejected);
    const size_t n_deg = countOf(console, kDegraded);
    const size_t n_info = countOf(console, kNormalInfo);
    std::printf("  [V7] console  REJECTED=%zu DEGRADED=%zu normalINFO=%zu\n",
                n_rej, n_deg, n_info);

    expect(n_rej == 1, "MALFORMED 거부가 콘솔에 정확히 한 번");
    expect(n_deg == 1, "DEGRADED 가 콘솔에 정확히 한 번");
    expect(n_info <= 1, "정상 INFO 가 콘솔에 두 번 나오지 않는다");

    const auto files = logFilesFor(c.name);
    if (c.file_logging) {
      bool in_file = false;
      for (const auto &f : files) {
        if (readAll(f).find(kRejected) != std::string::npos) in_file = true;
      }
      std::printf("  [V7] file     files=%zu contains_rejection=%d\n",
                  files.size(), in_file ? 1 : 0);
      expect(!files.empty() && in_file,
             "파일 로깅이 켜져 있으면 파일에도 기록된다");
    } else {
      std::printf("  [V7] file     files=%zu (기대 0)\n", files.size());
      expect(files.empty(),
             "파일 로깅이 꺼져 있으면 파일은 만들어지지 않는다 — "
             "그래도 위의 콘솔 기록은 남았다");
    }
  }

  rclcpp::shutdown();
  if (g_failed == 0) { std::cout << "\nPASS: 0 failed check(s)\n"; return 0; }
  std::cout << "\nFAIL: " << g_failed << " failed check(s)\n";
  return 1;
}
