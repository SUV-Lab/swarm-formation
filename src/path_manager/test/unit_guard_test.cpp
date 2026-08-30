// [V-8] 단위 정합 가드가 실제로 비교 대상을 본다.
//
// FSM 은 진술된 모든 성분을 manager/initial_speed_unit_m 으로 변환하고,
// 포락선 게이트는 optimization/dynamics_unit_xy_m / _z_m 으로 되돌린다. 둘이
// 어긋나면 **진술된 크기와 판정된 크기가 달라진다** — 메시지 계약이 금지하는
// 바로 그 조작이다. 가드는 그것을 생성 시점에 시끄럽게 만들려고 있었다.
//
// 그런데 가드가 두 optimization/* 을 has_parameter 로 떠보고 없으면 리터럴
// 100.0 을 썼다. 그 둘은 PathManager::initOptimizer 안에서 선언되고
// initOptimizer 는 **첫 미션에서** 처음 돈다 — 이 생성자보다 한참 뒤다.
// 그래서 has_parameter 는 늘 거짓이었고, 가드는 100.0 을 100.0 과 비교했다.
// 존재 이유인 경우에는 fail-open 이고, 셋 다 일관되게 100 이 아닌 설정에서는
// 오히려 거짓 양성이었다.
//
// 계약은 "불일치를 콘솔 ERROR 로 드러낸다" 까지다. 노드 생성을 막지 않는다 —
// 강제 종료 여부는 별도 운영 정책이고 이 시험은 그것을 가정하지 않는다.
//
// 관측은 stderr 를 파일로 돌려 rclcpp 가 실제로 쓴 것을 읽는다. 소스를 grep
// 하는 방식으로는 선언 순서 결함이 살아 있어도 통과한다.

#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "path_manager/replan_fsm.h"

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

const char *kMismatch = "[INITIAL-STATE] unit mismatch";

// FSM 을 한 번 세우고, 그 생성 동안 콘솔에 나온 것을 돌려준다.
std::string constructAndCapture(const char *yaml, const char *node_name,
                                double initial_unit, double unit_xy,
                                double unit_z) {
  rclcpp::NodeOptions opt;
  opt.arguments({"--ros-args", "--params-file", yaml});
  opt.parameter_overrides({
      rclcpp::Parameter("drone_id", 0),
      rclcpp::Parameter("chain/enable", false),
      // 가드는 WARN/ERROR 로 말한다. 그 둘은 이제 항상 콘솔로 가지만,
      // 주변 INFO 가 섞이지 않도록 debug 로그는 꺼 둔다.
      rclcpp::Parameter("enable_debug_logs", false),
      rclcpp::Parameter("manager/initial_speed_unit_m", initial_unit),
      rclcpp::Parameter("optimization/dynamics_unit_xy_m", unit_xy),
      rclcpp::Parameter("optimization/dynamics_unit_z_m", unit_z),
  });
  auto node = rclcpp::Node::make_shared(node_name, opt);
  StderrCapture cap(std::string("/tmp/") + node_name + ".stderr");
  path_manager::ReplanFSM fsm(node);
  (void)fsm;
  return cap.stop();
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage: unit_guard_test <optimizer_params.yaml>\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  const char *yaml = argv[1];

  // ── 일치: 조용히 지나간다 ──────────────────────────────────────────
  // 100 이 아닌 값으로 셋을 함께 옮긴다. 옛 가드는 여기서 거짓 양성이었다 —
  // 리터럴 100.0 과 비교했으므로 일관된 설정을 불일치라고 불렀다.
  {
    const std::string out =
        constructAndCapture(yaml, "ug_match", 50.0, 50.0, 50.0);
    expect(!has(out, kMismatch),
           "셋이 일치하면(전부 50) 조용히 지나간다 — 100 이 아니어도");
  }
  {
    const std::string out =
        constructAndCapture(yaml, "ug_match100", 100.0, 100.0, 100.0);
    expect(!has(out, kMismatch), "출하값 100 일치도 조용하다");
  }

  // ── XY 만 불일치 ───────────────────────────────────────────────────
  {
    const std::string out =
        constructAndCapture(yaml, "ug_xy", 100.0, 50.0, 100.0);
    expect(has(out, kMismatch), "XY 만 어긋나면 콘솔 ERROR 가 난다");
    expect(has(out, "dynamics_unit_xy_m=50.000000"),
           "...어긋난 XY 값을 그대로 말한다");
    expect(has(out, "dynamics_unit_z_m=100.000000"),
           "...멀쩡한 Z 값도 함께 보여 어느 쪽인지 구분된다");
  }

  // ── Z 만 불일치 ────────────────────────────────────────────────────
  // 감사가 지적한 fail-open 은 XY 만으로도 드러나지만, 가드는 두 이름을
  // 따로 읽으므로 한쪽만 고치고 끝내는 수정을 막으려면 Z 팔이 필요하다.
  {
    const std::string out =
        constructAndCapture(yaml, "ug_z", 100.0, 100.0, 50.0);
    expect(has(out, kMismatch), "Z 만 어긋나도 콘솔 ERROR 가 난다");
    expect(has(out, "dynamics_unit_z_m=50.000000"),
           "...어긋난 Z 값을 그대로 말한다");
    expect(has(out, "dynamics_unit_xy_m=100.000000"),
           "...멀쩡한 XY 값도 함께 보여 어느 쪽인지 구분된다");
  }

  rclcpp::shutdown();
  if (g_failed == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
  std::cout << "FAIL: " << g_failed << " failed check(s)\n";
  return 1;
}
