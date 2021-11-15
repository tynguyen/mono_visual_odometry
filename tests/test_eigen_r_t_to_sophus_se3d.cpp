#include <iostream>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <Eigen/Core>
#include "gtest/gtest.h"

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace Sophus;

TEST(SophusTest, test_eigen_r_t_to_sophus_se3d)
{
    EXPECT_EQ (1, 1);
    Eigen::Matrix<double, 3, 3> R;
    R  << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    cout << "R: " << endl << R << endl;
    Eigen::Vector3d t;
    t  << 1, 2, 3;
    cout << "t: " << endl << t << endl;
    Sophus::SE3d SE3_Rt(R, t);   // Create Sophus SE3 from R and t
    // cout << "SE3_Rt from R and t: " << endl << SE3_Rt << endl;

    // Get rotation and translation matrix from the SE3_Rt
    cout << "R from SE_Rt: " << endl << SE3_Rt.rotationMatrix()<< endl;
    cout << "t from SE_Rt: " << endl << SE3_Rt.translation().transpose() << endl;
    EXPECT_EQ (SE3_Rt.rotationMatrix(), R);
}
