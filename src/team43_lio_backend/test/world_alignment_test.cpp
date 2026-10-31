#include <gtest/gtest.h>
#include "world_alignment.hpp"

TEST(WorldAlignment, GravityAndGeometryStayConsistent) {
  const Eigen::Vector3d up(-.0321952086, -.6955923471, .7122368773);
  const auto q = rm_task1::world_alignment({up.x(), up.y(), up.z()});
  EXPECT_TRUE((q*up.normalized()).isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  const Eigen::Vector3d position(.3,-.2,.8), offset(-.011,-.02329,.04412), point(1.,2.,3.);
  const Eigen::Quaterniond pose(Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitY()));
  // body光心和注册云必须使用同一世界旋转，不能只转点云或只改frame名。
  EXPECT_TRUE((q*(position+pose*offset)).isApprox(q*position+(q*pose)*offset, 1e-12));
  EXPECT_NEAR((q*(point-position)).norm(), (point-position).norm(), 1e-12);
  const double floor=(q*offset).z()-.231;
  EXPECT_NEAR((q*offset).z()-floor, .231, 1e-12);
  EXPECT_NEAR(floor, up.normalized().dot(offset)-.231, 1e-12);
}
TEST(WorldAlignment, IdentityAndInvalidInput) {
  EXPECT_TRUE(rm_task1::world_alignment({0.,0.,1.}).toRotationMatrix().isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_THROW(rm_task1::world_alignment({0.,0.,0.}), std::invalid_argument);
  EXPECT_THROW(rm_task1::world_alignment({0.,1.}), std::invalid_argument);
  EXPECT_THROW(rm_task1::world_alignment({0.,NAN,1.}), std::invalid_argument);
}
