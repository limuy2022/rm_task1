// 对实际 esekfom 模板实例化线性状态，验证两种卡尔曼计算路径的等价性。
// 状态容器和测量回调是测试夹具，滤波更新完全由上游实际头文件执行。
#include <iomanip>
#include <iostream>
#include <string>

#include RM_TEST_EKF_HEADER

struct LinearState {
  using scalar = double;
  enum { DOF = 24, DIM = 24 };
  Eigen::Matrix<double, 24, 1> value = Eigen::Matrix<double, 24, 1>::Zero();
  void build_S2_state() {}
  void build_SO3_state() {}
  void build_vect_state() {}
  void build_SEN_state() {}
  void boxplus(const Eigen::Matrix<double, 24, 1>& increment) { value += increment; }
};

using Filter = esekfom::esekf<LinearState, 24, LinearState>;
int active_rows, total_rows;
double variance;

Eigen::Matrix<double, 24, 1> dynamics(LinearState&, const LinearState&) {
  return Eigen::Matrix<double, 24, 1>::Zero();
}

Eigen::Matrix<double, 24, 24> jacobian(LinearState&, const LinearState&) {
  return Eigen::Matrix<double, 24, 24>::Zero();
}

void measurement(LinearState&, Eigen::Matrix3d, Eigen::Matrix3d,
                 esekfom::dyn_share_modified<double>& data) {
  data.valid = true;
  data.M_Noise = variance;
  data.h_x = Eigen::MatrixXd::Zero(total_rows, 12);
  data.h_x.topRows(active_rows).col(0).setOnes();
  data.z = Eigen::VectorXd::Zero(total_rows);
  data.z.head(active_rows).setOnes();
}

Filter run(int active, int total, double noise) {
  active_rows = active;
  total_rows = total;
  variance = noise;
  Filter filter;
  filter.init_dyn_share_modified_2h(dynamics, jacobian, measurement);
  if (!filter.update_iterated_dyn_share_modified()) throw std::runtime_error("测量被拒绝");
  return filter;
}

int main(int argc, char** argv) {
  const bool legacy = argc == 2 && std::string(argv[1]) == "--expect-legacy";
  bool reproduced_nonunit_disagreement = false;
  std::cout << std::setprecision(12)
            << "variance,small_gain,zero_padded_gain,repeated_gain,max_cov_difference\n";
  for (double noise : {.01, 1., 100.}) {
    // 23行走测量空间分支；第24行H=0,z=0不增加任何信息，只触发另一分支。
    const auto small = run(23, 23, noise);
    const auto padded = run(23, 24, noise);
    // 两份相同观测都用2R，其总信息仍等于一份使用R的观测。
    const auto repeated = run(46, 46, 2*noise);
    const double expected_gain = 23/(23+noise);
    const double covariance_difference = std::max(
        (small.P_-padded.P_).cwiseAbs().maxCoeff(),
        (small.P_-repeated.P_).cwiseAbs().maxCoeff());
    std::cout << noise << ',' << small.x_.value(0) << ',' << padded.x_.value(0)
              << ',' << repeated.x_.value(0) << ',' << covariance_difference << '\n';
    if (std::abs(small.x_.value(0)-expected_gain) > 1e-10) return 1;
    const bool padded_equal = (small.x_.value-padded.x_.value).norm() < 1e-10;
    const bool repeated_equal = (small.x_.value-repeated.x_.value).norm() < 1e-10;
    if (legacy) {
      if (noise == 1. && !padded_equal) return 2;
      if (noise != 1. && !padded_equal) reproduced_nonunit_disagreement = true;
    } else if (!padded_equal || !repeated_equal || covariance_difference > 1e-10) {
      std::cerr << "FAIL: 等信息观测跨越计算分支后状态或协方差不一致\n";
      return 3;
    }
  }
  if (legacy && !reproduced_nonunit_disagreement) return 4;
  std::cout << (legacy ? "PASS: 实际模板复现原版测量方差分支不一致\n"
                       : "PASS: 实际模板在三档方差及补零/重复观测下更新一致\n");
}
