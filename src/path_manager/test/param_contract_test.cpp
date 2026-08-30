// [V-6] 파라미터 읽기 계약: 초기값은 UB 를 없애고, 실패한 읽기는 정상 설정처럼
// 숨지 않는다.
//
// 세 bool(enable_obstacles_ / enable_debug_logs_ / enable_lbfgs_detail_logs_)
// 이 선언에 초기값 없이 있었고 setParam 이 get_parameter 의 반환값을 버렸다.
// 생산 바이너리가 무사했던 것은 ReplanFSM 이 PathManager 보다 먼저 선언하기
// 때문이고, 그것은 **선언 순서**에 기댄 안전이지 코드의 성질이 아니다.
//
// drone_id 는 더 나쁘다. 진단이 아니라 데이터라서 최적화기(setDroneId)와
// 발행되는 PolyTraj 까지 간다. 초기화 없는 지역변수에 반환값을 버린 읽기였다.
//
// 이 파일이 못박는 것과 못박지 않는 것을 먼저 적는다.
//   못박는다: drone_id 가 궤적과 최적화기까지 그대로 전달되는가, 누락된
//     drone_id 가 명시적으로 거부되는가, 파라미터가 있으면 컴파일 기본값을
//     정확히 덮는가, 선언되지 않은 파라미터가 조용히 넘어가지 않는가.
//   못박지 않는다: "setParam 이전 세 bool 의 값이 결정적인가". 그 값들은
//     private 이고, 초기화가 빠졌을 때 읽히는 값은 스택에 남은 쓰레기라
//     비결정적이다 — 그것을 단언하는 시험은 그 자체로 불안정해진다. 그
//     성질은 UBSan 으로 확인했고 커밋 메시지에 수치를 적었다.
//
// 최적화기까지의 전달은 로그 접두사 "[POLY_TRAJ_OPT][drone %d]" 로 읽는다.
// 최적화기 인스턴스가 PathManager 의 private 멤버라 그것이 유일한 공개
// 관측점이다.

#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "path_manager/path_manager.h"

namespace {

int g_failed = 0;
void expect(bool c, const std::string &label) {
  std::cout << "  " << (c ? "[OK]   " : "[FAIL] ") << label << "\n";
  if (!c) ++g_failed;
}

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

bool has(const std::string &hay, const std::string &needle) {
  return hay.find(needle) != std::string::npos;
}

rclcpp::NodeOptions optsWith(const std::vector<rclcpp::Parameter> &ov,
                             const char *params_file) {
  rclcpp::NodeOptions o;
  o.arguments({"--ros-args", "--params-file", params_file});
  o.parameter_overrides(ov);
  return o;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage: param_contract_test <optimizer_params.yaml>\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  const char *yaml = argv[1];

  // ── (1) 정상 경로: drone_id 가 궤적과 최적화기까지 간다 ─────────────
  {
    auto node = rclcpp::Node::make_shared(
        "pc_ok", optsWith({rclcpp::Parameter("chain/enable", false),
                           // yaml 이 enable_debug_logs: true 라 declare 의
                           // 기본값으로는 못 이긴다. override 로 강제해서
                           // LOG_* 가 콘솔을 타게 한다.
                           rclcpp::Parameter("enable_debug_logs", false)},
                          yaml));
    // 생산에서는 ReplanFSM 이 이것을 먼저 선언한다. 여기서는 직접 만드는
    // 경로이므로 시험이 선언한다 — 그것이 계약이다.
    node->declare_parameter("drone_id", 7);
    node->declare_parameter("enable_debug_logs", true);  // override 가 이긴다

    std::string console;
    std::shared_ptr<path_manager::PathManager> pm;
    {
      StderrCapture cap("/tmp/pc_ok.stderr");
      pm = std::make_shared<path_manager::PathManager>(node);
      pm->initOptimizer();
      console = cap.stop();
    }
    expect(pm->traj_.local_traj.drone_id == 7,
           "drone_id 가 궤적 컨테이너에 그대로 들어간다");
    expect(has(console, "[POLY_TRAJ_OPT][drone 7]"),
           "같은 drone_id 가 최적화기까지 전달된다 — 로그 접두사로 확인");
    expect(!has(console, "[POLY_TRAJ_OPT][drone 0]"),
           "0 으로 떨어지지 않는다");
    // 세 bool 중 하나가 실제로 파라미터 값을 받았는지: 출하 기본값 true 에서
    // 나오는 문구.
    expect(has(console, "Obstacle avoidance: enabled"),
           "enable_obstacles 가 선언된 기본값 true 를 받는다");
  }

  // ── (2) 파라미터가 있으면 컴파일 기본값을 정확히 덮는다 ────────────
  {
    auto node = rclcpp::Node::make_shared(
        "pc_override",
        optsWith({rclcpp::Parameter("chain/enable", false),
                  rclcpp::Parameter("enable_obstacles", false),
                  rclcpp::Parameter("enable_debug_logs", false)}, yaml));
    node->declare_parameter("drone_id", 3);
    node->declare_parameter("enable_debug_logs", true);

    std::string console;
    {
      StderrCapture cap("/tmp/pc_override.stderr");
      auto pm = std::make_shared<path_manager::PathManager>(node);
      pm->initOptimizer();
      console = cap.stop();
    }
    expect(has(console, "Obstacle avoidance: disabled"),
           "enable_obstacles=false 가 컴파일 기본값 true 를 덮는다");
    expect(has(console, "[POLY_TRAJ_OPT][drone 3]"),
           "override 경로에서도 drone_id 가 전달된다");
  }

  // ── (3) 누락된 drone_id 는 명시적으로 거부된다 ──────────────────────
  {
    auto node = rclcpp::Node::make_shared(
        "pc_missing",
        optsWith({rclcpp::Parameter("chain/enable", false)}, yaml));
    // drone_id 를 일부러 선언하지 않는다.
    bool threw = false;
    std::string what;
    try {
      StderrCapture cap("/tmp/pc_missing.stderr");
      auto pm = std::make_shared<path_manager::PathManager>(node);
      (void)pm;
    } catch (const std::exception &e) {
      threw = true;
      what = e.what();
    }
    expect(threw, "선언되지 않은 drone_id 로는 PathManager 가 만들어지지 않는다");
    expect(has(what, "drone_id"),
           "예외가 어느 파라미터인지 이름으로 말한다");
    expect(has(what, "drone 0"),
           "예외가 '조용히 0 으로 가지 않는다' 를 명시한다");
  }

  // ── (4) 선언되지 않은 bool 은 조용히 넘어가지 않는다 ────────────────
  {
    auto node = rclcpp::Node::make_shared(
        "pc_undeclared",
        optsWith({rclcpp::Parameter("chain/enable", false)}, yaml));
    node->declare_parameter("drone_id", 5);
    // enable_debug_logs 는 선언하지 않는다 — 생산에서는 ReplanFSM 이 하는
    // 일이고, 직접 구성 경로에서는 없는 것이 정상이지만 조용해서는 안 된다.

    std::string console;
    {
      StderrCapture cap("/tmp/pc_undeclared.stderr");
      auto pm = std::make_shared<path_manager::PathManager>(node);
      pm->initOptimizer();
      console = cap.stop();
    }
    expect(has(console, "[PARAM] enable_debug_logs is not declared"),
           "선언되지 않은 파라미터는 이름과 함께 보고된다 — "
           "읽기 실패가 정상 설정처럼 숨지 않는다");
    expect(has(console, "[POLY_TRAJ_OPT][drone 5]"),
           "그래도 drone_id 는 정확히 전달된다");
  }

  rclcpp::shutdown();
  if (g_failed == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
  std::cout << "FAIL: " << g_failed << " failed check(s)\n";
  return 1;
}
