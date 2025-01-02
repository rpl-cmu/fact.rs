#include "gtsam/slam/dataset.h"
#include <cstdio>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <chrono>
#include <nanobench.h>

using namespace gtsam;
using namespace ankerl;

std::string directory = "examples/data/";
std::vector<std::string> files_3d{"sphere2500.g2o", "parking-garage.g2o"};
std::vector<std::string> files_2d{"M3500.g2o"};

GraphAndValues load(std::string file, bool is3D) {
  auto read = readG2o(file, is3D);

  if (is3D) {
    auto priorModel = noiseModel::Diagonal::Variances(
        (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    read.first->addPrior(0, Pose3::Identity(), priorModel);
  } else {
    auto priorModel =
        noiseModel::Diagonal::Variances(Vector3(1e-6, 1e-6, 1e-8));
    auto prior = PriorFactor<Pose2>(0, Pose2());
    read.first->addPrior(0, Pose2::Identity(), priorModel);
  }

  return read;
}

void run_gtsam(nanobench::Bench *bench, std::string file, bool is3D) {
  auto gv = load(directory + file, is3D);

  bench->context("benchmark", "gtsam");
  bench->run(file, [&]() {
    NonlinearFactorGraph graph(*gv.first);
    Values values(*gv.second);

    GaussNewtonOptimizer optimizer(graph, values);
    Values result = optimizer.optimize();

    nanobench::doNotOptimizeAway(result);
  });
}
// std::vector<std::tuple(std::string, std::is_function<typename>)

char const *markdown() {
  return R"DELIM(| benchmark | args | fastest | median | mean |
{{#result}}| {{context(benchmark)}} | {{name}} | {{minimum(elapsed)}} | {{median(elapsed)}} | {{average(elapsed)}} |
{{/result}})DELIM";
}

int main(int argc, char *argv[]) {

  nanobench::Bench b;
  b.timeUnit(std::chrono::milliseconds(1), "ms");

  // 3d benchmarks
  b.title("3d benchmarks");
  for (auto &file : files_3d) {
    run_gtsam(&b, file, true);
  }
  std::cout << "\nIn Markdown format:\n";
  b.render(markdown(), std::cout);

  // 2d benchmarks
  b.title("2d benchmarks");
  for (auto &file : files_2d) {
    run_gtsam(&b, file, false);
  }
  std::cout << "\nIn Markdown format:\n";
  b.render(markdown(), std::cout);
}