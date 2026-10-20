#pragma once

#include <Eigen/Eigenvalues>

namespace rm_task1 {

// 五个近邻必须张成二维局部表面；重复点或共线点不能确定唯一平面。
// 使用中心化散布矩阵，判定不随平移或刚体旋转改变。
template <typename Derived>
bool fit_plane(const Eigen::MatrixBase<Derived>& points,
               Eigen::Matrix<typename Derived::Scalar, 4, 1>& plane,
               typename Derived::Scalar threshold) {
  using T = typename Derived::Scalar;
  if (!points.allFinite()) return false;
  Eigen::Matrix<T, Derived::RowsAtCompileTime, 3> centered =
      points.rowwise() - points.colwise().mean();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, 3, 3>> eig(
      centered.transpose() * centered);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite()) return false;
  const auto& values = eig.eigenvalues();
  if (values(2) < T(1e-9) || values(1) < T(1e-4) * values(2)) return false;
  // 有二维分布仍可能跨越墙角；法向散布应远小于第二个切向散布。
  if (values(0) > T(.01) * values(1)) return false;
  // 中心化求法向，再还原常数项，可表示经过坐标原点的平面。
  plane.template head<3>() = eig.eigenvectors().col(0);
  plane(3) = -points.colwise().mean().dot(plane.template head<3>());
  if (!plane.allFinite()) return false;
  return ((points * plane.template head<3>()).array() + plane(3)).abs().maxCoeff() <= threshold;
}

}  // namespace rm_task1
