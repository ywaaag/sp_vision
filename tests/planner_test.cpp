#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tinympc/tiny_api.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

constexpr int NSTATES = 2;
constexpr int NINPUTS = 1;
constexpr int NHORIZON = 100;

constexpr double DT = 1e-2;
constexpr double J = 7e-2;

typedef Matrix<tinytype, NINPUTS, NHORIZON - 1> tiny_MatrixNuNhm1;
typedef Matrix<tinytype, NSTATES, NHORIZON> tiny_MatrixNxNh;
typedef Matrix<tinytype, NSTATES, 1> tiny_VectorNx;

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

class Target
{
public:
  Target(double x, double y, double w, double r) : x(x), y(y), yaw(0), vyaw(w), r(r) {}

  double x;
  double y;
  double yaw;
  double vyaw;
  double r;

  void update(double dt) { yaw += vyaw * dt; }

  double observe() const
  {
    auto azim = 0.0;
    auto min_dist = 1e10;
    for (int i = 0; i < 4; ++i) {
      auto armor_yaw = tools::limit_rad(yaw + i * CV_PI / 2);
      auto armor_x = x - r * std::cos(armor_yaw);
      auto armor_y = y - r * std::sin(armor_yaw);
      auto armor_azim = std::atan2(armor_y, armor_x);
      auto dist = std::hypot(armor_x, armor_y);
      if (dist < min_dist) {
        min_dist = dist;
        azim = armor_azim;
      }
    }
    return azim;
  }

private:
  std::chrono::steady_clock::time_point t_;
};

tiny_MatrixNxNh get_trajectory(Target target)
{
  tiny_MatrixNxNh traj;
  for (int i = 0; i < NHORIZON; i++) {
    auto azim0 = target.observe();
    target.update(DT);
    auto azim1 = target.observe();
    traj.col(i) << azim0, tools::limit_rad(azim1 - azim0) / DT;
  }
  return traj;
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  auto t0 = std::chrono::steady_clock::now();
  Target target(2, 0, -4.0, 0.2);

  /// MPC

  TinySolver * solver;

  tinytype Adyn_data[NSTATES * NSTATES] = {1, DT, 0, 1};
  tinytype Bdyn_data[NSTATES * NINPUTS] = {0, DT / J};
  tinytype Q_data[NSTATES] = {1e4, 1e2};
  tinytype R_data[NINPUTS] = {1};

  tinyMatrix Adyn = Map<Matrix<tinytype, NSTATES, NSTATES, RowMajor>>(Adyn_data);
  tinyMatrix Bdyn = Map<Matrix<tinytype, NSTATES, NINPUTS>>(Bdyn_data);
  tinyVector fdyn = tiny_VectorNx::Zero();
  tinyVector Q = Map<Matrix<tinytype, NSTATES, 1>>(Q_data);
  tinyVector R = Map<Matrix<tinytype, NINPUTS, 1>>(R_data);

  tinyMatrix x_min = tiny_MatrixNxNh::Constant(-1e17);
  tinyMatrix x_max = tiny_MatrixNxNh::Constant(1e17);
  tinyMatrix u_min = tiny_MatrixNuNhm1::Constant(-7);
  tinyMatrix u_max = tiny_MatrixNuNhm1::Constant(7);

  // Set up problem
  tiny_setup(
    &solver, Adyn, Bdyn, fdyn, Q.asDiagonal(), R.asDiagonal(), 1.0, NSTATES, NINPUTS, NHORIZON, 0);

  // Set bound constraints
  tiny_set_bound_constraints(solver, x_min, x_max, u_min, u_max);

  // Update whichever settings we'd like
  solver->settings->max_iter = 100;

  // Alias solver->work for brevity
  TinyWorkspace * work = solver->work;

  tiny_VectorNx x0;
  x0 << 0.01, 0.0;

  while (!exiter.exit()) {
    auto start = std::chrono::steady_clock::now();

    target.update(DT);
    auto traj = get_trajectory(target);
    work->Xref = traj;
    tiny_set_x0(solver, x0);
    tiny_solve(solver);

    auto end = std::chrono::steady_clock::now();
    auto cost = tools::delta_time(end, start);
    tools::logger()->debug("{:.1f} ms", cost * 1e3);

    nlohmann::json data;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
    data["azim"] = traj(0, 0);
    data["azim_dot"] = traj(1, 0);
    data["x"] = work->x(0, 0);
    data["x_dot"] = work->x(1, 0);
    data["u"] = work->u(0, 0);
    plotter.plot(data);

    x0 = work->Adyn * x0 + work->Bdyn * work->u.col(0);

    std::this_thread::sleep_for(10ms);
  }

  return 0;
}