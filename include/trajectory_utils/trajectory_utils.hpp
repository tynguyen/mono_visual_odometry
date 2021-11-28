#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <vector>
#include <opencv2/core/core.hpp>

namespace trajectory_utils {
	double find_absolute_trajectory_error(const   std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>& trajectory, const std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>& ground_truth);

	// Overwrite
	template<typename T>
	double  find_absolute_trajectory_error(
		const std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>& trajectory,
		const std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>& ground_truth);

	double find_average_translational_error(const std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>& trajectory, const std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>& ground_truth);
	// Overwrite
	template<typename T>
	double find_average_translational_error(
		const std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>& trajectory,
		const std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>& ground_truth);

	template<typename T>
	void get_traj_dim_Xd(std::vector<std::vector<T>> & gt_translations, std::vector<T> & gt_traj_dims, std::vector<T> & gt_traj_dim_min_vals, std::vector<T> & gt_traj_dim_max_vals);


	template<typename T>
	double find_average_rotational_error(
		const std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>& trajectory,
		const std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>& ground_truth);

	template<typename T>
	double  find_relative_trajectory_error(
		const std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>& trajectory,
		const std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>& ground_truth);
};


double  trajectory_utils::find_absolute_trajectory_error(const   std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>& trajectory, const std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>& ground_truth){
	/*
		Finds the absolute error between the trajectory and the ground truth.
	*/
	if (trajectory.size() != ground_truth.size() || trajectory.size() == 0) {
		std::cout << "GT Traj size: " << ground_truth.size() << std::endl;
		std::cout << "Predicted Traj size: " << trajectory.size() << std::endl;
		throw std::runtime_error("Trajectory and ground truth must have the same size.");
	}

	double rmse = 0;
	for (size_t i = 0; i < trajectory.size(); i++) {
		double error = (trajectory[i].inverse() * ground_truth[i]).log().norm();
		rmse += error * error;

		double rot_err = (trajectory[i].rotationMatrix().inverse() * ground_truth[i].rotationMatrix()).norm();
		double trans_err = (trajectory[i].translation() - ground_truth[i].translation()).norm();
		std::cout << "|---> i " << i << "| Rot err: " << rot_err << "| Trans err: " << trans_err << std::endl;
		std::cout << "|---> GT Trans: " << ground_truth[i].translation().transpose() << std::endl;
		std::cout << "|---> Pred Trans: " << trajectory[i].translation().transpose() << std::endl;
	}
	rmse /= trajectory.size();
	rmse = std::sqrt(rmse);
	return rmse;
}

// Overwrite function
template<typename T>
double  trajectory_utils::find_absolute_trajectory_error(
	const std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>& trajectory,
	const std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>& ground_truth){
	/*
		Finds the absolute error between the trajectory and the ground truth.
	*/
	if (trajectory.size() != ground_truth.size() || trajectory.size() == 0) {
		std::cout << "GT Traj size: " << ground_truth.size() << std::endl;
		std::cout << "Predicted Traj size: " << trajectory.size() << std::endl;
		throw std::runtime_error("Trajectory and ground truth must have the same size.");
	}
	// Get R0, t0
	Eigen::Matrix<T, 4, 4> T0_inv_pred = trajectory[0].inverse();
	Eigen::Matrix<T, 4, 4> T0_inv_gt = ground_truth[0].inverse();

	double rmse = 0;
	for (size_t i = 0; i < trajectory.size(); i++) {
		// Multiply with inverse of TO to ensure that all poses start at the same [eyes, 0]
		Eigen::Matrix<T, 4, 4> T_pred = T0_inv_pred * trajectory[i];
		Eigen::Matrix<T, 4, 4> T_gt = T0_inv_gt * ground_truth[i];
		Eigen::Matrix<T, 4, 4> T_err = T_pred.inverse() * T_gt - Eigen::Matrix<T, 4, 4>::Identity();
		double error = T_err.norm();
		rmse += error * error;
	}
	rmse /= trajectory.size();
	rmse = std::sqrt(rmse);
	return rmse;
}

double trajectory_utils::find_average_translational_error(const std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>& trajectory, const std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>& ground_truth){
	/*
		//TODO: test this function
		Finds the average translational error between the trajectory and the ground truth.
	*/
	if (trajectory.size() != ground_truth.size() || trajectory.size() == 0) {
		std::cout << "GT Traj size: " << ground_truth.size() << std::endl;
		std::cout << "Predicted Traj size: " << trajectory.size() << std::endl;
		throw std::runtime_error("Trajectory and ground truth must have the same NON-zero size.");
	}

	double rmse = 0;
	for (size_t i = 0; i < trajectory.size(); i++) {
		double error = (trajectory[i].inverse() * ground_truth[i]).translation().norm();
		rmse += error * error;
	}
	rmse /= trajectory.size();
	rmse = std::sqrt(rmse);
	return rmse;
}

template<typename T>
double trajectory_utils::find_average_translational_error(
	const std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>& trajectory,
	const std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>& ground_truth){
	/*
		Finds the average translational error between the trajectory and the ground truth.
	*/
	if (trajectory.size() != ground_truth.size() || trajectory.size() == 0) {
		std::cout << "GT Traj size: " << ground_truth.size() << std::endl;
		std::cout << "Predicted Traj size: " << trajectory.size() << std::endl;
		throw std::runtime_error("Trajectory and ground truth must have the same NON-zero size.");
	}
	// Get R0, t0
	Eigen::Matrix<T, 4, 4> T0_inv_pred = trajectory[0].inverse();
	Eigen::Matrix<T, 4, 4> T0_inv_gt = ground_truth[0].inverse();

	double rmse = 0;
	for (size_t i = 0; i < trajectory.size(); i++) {
		// Multiply with inverse of TO to ensure that all poses start at the same [eyes, 0]
		Eigen::Matrix<T, 4, 4> T_pred = T0_inv_pred * trajectory[i];
		Eigen::Matrix<T, 4, 4> T_gt = T0_inv_gt * ground_truth[i];
		Eigen::Matrix<T, 4, 4> T_err = T_pred.inverse() * T_gt;
		double error = T_err.block(0, 3, 3, 1).norm();
		rmse += error * error;
	}
	rmse /= trajectory.size();
	rmse = std::sqrt(rmse);
	return rmse;
}

template<typename T>
double trajectory_utils::find_average_rotational_error(
	const std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>& trajectory,
	const std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>& ground_truth){
	/*
		Finds the average rotational error between the trajectory and the ground truth.
	*/
	if (trajectory.size() != ground_truth.size() || trajectory.size() == 0) {
		std::cout << "GT Traj size: " << ground_truth.size() << std::endl;
		std::cout << "Predicted Traj size: " << trajectory.size() << std::endl;
		throw std::runtime_error("Trajectory and ground truth must have the same NON-zero size.");
	}

	// Get R0, t0
	Eigen::Matrix<T, 4, 4> T0_inv_pred = trajectory[0].inverse();
	Eigen::Matrix<T, 4, 4> T0_inv_gt = ground_truth[0].inverse();

	double rmse = 0;
	for (size_t i = 0; i < trajectory.size(); i++) {
		// Multiply with inverse of TO to ensure that all poses start at the same [eyes, 0]
		Eigen::Matrix<T, 4, 4> T_pred = T0_inv_pred * trajectory[i];
		Eigen::Matrix<T, 4, 4> T_gt = T0_inv_gt * ground_truth[i];

		Eigen::Matrix<T, 3, 3> pred_rot = T_pred.block(0, 0, 3, 3);
		Eigen::Matrix<T, 3, 3> gt_rot = T_gt.block(0, 0, 3, 3);
		Eigen::Matrix<T, 3, 3> rot_err = pred_rot.inverse() * gt_rot;
		double error = (rot_err - Eigen::Matrix<T, 3, 3>::Identity()).norm();
		rmse += error * error;
	}
	rmse /= trajectory.size();
	rmse = std::sqrt(rmse);
	return rmse;
}

template<typename T>
double  trajectory_utils::find_relative_trajectory_error(
	const std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>& trajectory,
	const std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>& ground_truth){
	/*
		Finds the error of the relative translation
	*/
	if (trajectory.size() != ground_truth.size() || trajectory.size() == 0) {
		std::cout << "GT Traj size: " << ground_truth.size() << std::endl;
		std::cout << "Predicted Traj size: " << trajectory.size() << std::endl;
		throw std::runtime_error("Trajectory and ground truth must have the same size.");
	}
	// Get T_prev
	Eigen::Matrix<T, 4, 4> T_prev_inv_pred = trajectory[0].inverse();
	Eigen::Matrix<T, 4, 4> T_prev_inv_gt = ground_truth[0].inverse();

	double rmse = 0;
	for (size_t i = 0; i < trajectory.size(); i++) {
		// Multiply with inverse of TO to ensure that all poses start at the same [eyes, 0]
		Eigen::Matrix<T, 4, 4> T_rel_pred = T_prev_inv_pred * trajectory[i];
		Eigen::Matrix<T, 4, 4> T_rel_gt = T_prev_inv_gt * ground_truth[i];
		Eigen::Matrix<T, 4, 4> T_rel_err = T_rel_pred.inverse() * T_rel_gt;
		double error = T_rel_err.block(0, 3, 3, 1).norm();
		rmse += error * error;

		T_prev_inv_pred = trajectory[i].inverse();
		T_prev_inv_gt = ground_truth[i].inverse();
	}
	rmse /= trajectory.size();
	rmse = std::sqrt(rmse);
	return rmse;
}


template<typename T>
void trajectory_utils::get_traj_dim_Xd(std::vector<std::vector<T>> & gt_translations, std::vector<T> & gt_traj_dims, std::vector<T> & gt_traj_dim_min_vals, std::vector<T> & gt_traj_dim_max_vals){
	/*
		Finds the dimensions of the trajectory in X different dimensions.
		@Args:
			gt_translations: ground truth translations
			gt_traj_dims: dimensions of the trajector
			gt_traj_dim_min_vals: minimum values along the trajectory dimensions
			gt_traj_dim_max_vals: maximum values along the trajectory dimensions
	*/
	if (gt_translations.size() == 0) {
		throw std::runtime_error("Ground truth translations vector is empty.");
	}

	int n_dims = gt_translations[0].size();

	std::vector<T> dim_min_vals(n_dims, std::numeric_limits<T>::max()), dim_max_vals(n_dims, std::numeric_limits<T>::min());
	for (size_t i = 0; i < gt_translations.size(); i++) {
		for (int j = 0; j < n_dims; j++) {
			if (gt_translations[i][j] < dim_min_vals[j]) {
				dim_min_vals[j] = gt_translations[i][j];
			}
			if (gt_translations[i][j] > dim_max_vals[j]) {
				dim_max_vals[j] = gt_translations[i][j];
			}
		}
	}

	// Find the dimensions of the trajectory
	for (int i = 0; i < n_dims; i++) {
		gt_traj_dims.push_back(dim_max_vals[i] - dim_min_vals[i]);
	}
	std::cout << "Trajectory dimensions: " << gt_traj_dims[0] << " " << gt_traj_dims[1] << " " << gt_traj_dims[2] << std::endl;
	gt_traj_dim_min_vals = dim_min_vals;
	gt_traj_dim_max_vals = dim_max_vals;
	std::cout << "Trajectory min values along dimensions: " << gt_traj_dim_min_vals[0] << " " << gt_traj_dim_min_vals[1] << " " << gt_traj_dim_min_vals[2] << std::endl;
	std::cout << "Trajectory max values along dimensions: " << gt_traj_dim_max_vals[0] << " " << gt_traj_dim_max_vals[1] << " " << gt_traj_dim_max_vals[2] << std::endl;
}