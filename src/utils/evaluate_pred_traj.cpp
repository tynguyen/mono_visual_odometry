#include <iostream>
#include <string>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "io_utils/io_utils.hpp"
#include "trajectory_utils/trajectory_utils.hpp"

int main(int argc, char** argv)
{
	if (argc != 3)
	{
		std::cerr << "Usage: ./evaluate_pred_traj <pred_traj_file> <gt_traj_file>" << std::endl;
		return 1;
	}

	std::string pred_traj_file = argv[1];
	std::string gt_traj_file = argv[2];

	// std::vector<std::vector<double>> pred_traj_se3, gt_traj_se3;
	std::vector<Eigen::Matrix<double, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<double, 4, 4>>> pred_traj_se3, gt_traj_se3;
	io_utils::readTrajectoryFromFile(pred_traj_file, pred_traj_se3);
	io_utils::readTrajectoryFromFile(gt_traj_file, gt_traj_se3);

	// Find ATE (absolute trajectory error)
	double ATE_all = trajectory_utils::find_absolute_trajectory_error(pred_traj_se3, gt_traj_se3);
	std::cout << "ATE: " << ATE_all << std::endl;

	// Find ATE_trans (Average translational error)
	double ATE_trans = trajectory_utils::find_average_translational_error(pred_traj_se3, gt_traj_se3);
	std::cout << "ATE trans: " << ATE_trans << std::endl;

	// Find ATE_rot (Average rotational error)
	double ATE_rot = trajectory_utils::find_average_rotational_error(pred_traj_se3, gt_traj_se3);
	std::cout << "ATE rot: " << ATE_rot << std::endl;

	// Find the relative pose error
	double rel_pose_error = trajectory_utils::find_relative_trajectory_error(pred_traj_se3, gt_traj_se3);
	std::cout << "Relative pose error: " << rel_pose_error << std::endl;
	std::cout << "----------------------------------------" << std::endl;
}
