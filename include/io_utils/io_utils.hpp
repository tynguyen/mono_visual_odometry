#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace io_utils{
	void readIthCamMatFromFile(const std::string& file_path, const int& i, double &fx, double &fy, double &cx, double &cy);

	double getAbsoluteScaleFromFile(const std::string& filepath, const int & frame_id, const std::vector<double> & pred_t);

	// template<typename T>
	std::vector<double> line2ValuesByIndices(const std::string& line, std::vector<int> & indices);

	// In case the ground truth is written in the following format:
	// time, tx, ty, tz, qx, qy, qz, qw
	std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> readTrajectoryFromFile(const std::string& file_path, bool is_relative_pose);
	// Overwrite function
	template<typename T>
	void readTrajectoryFromFile(const std::string& filepath, std::vector<std::vector<T>> & traj);
	// Overwrite function
	template<typename T>
	void readTrajectoryFromFile(const std::string& filepath, std::vector<Eigen::Matrix<T, 4, 4>,  Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>> & traj);

	void readTranslationsFromFile(const std::string& filepath, std::vector<std::vector<double>> & traj);

	template<typename T>
	void find_center_of_vec_points(const std::vector<std::vector<T>> & points, std::vector<T> & center);

	template<typename T>
	void writeTrajectoryToFile(const std::string& file_path, const std::vector<std::vector<T>> & traj);
};


void io_utils::readIthCamMatFromFile(const std::string& file_path, const int& i, double &fx, double &fy, double &cx, double &cy){
	std::ifstream cam_file;
	cam_file.open(file_path);
	if (!cam_file.is_open()) {
		std::cerr << "Error: Calibration file could not be opened!" << std::endl;
		return;
	}
	std::string line, token;
	int line_num = 0;
	while (line_num < i) {
		std::getline(cam_file, line);
		line_num++;
	}

	std::stringstream ss(line);
	std::vector<std::string> tokens;
	while (ss >> token){
		tokens.push_back(token);
	}
	fx = std::stof(tokens[1]);
	fy = std::stof(tokens[6]);
	cx = std::stof(tokens[3]);
	cy = std::stof(tokens[7]);
}

double io_utils::getAbsoluteScaleFromFile(const std::string& filepath, const int & frame_id, const std::vector<double> & pred_t){
	/*
		@Brief: get the absolute scale of of the visual odometry based on the ground truth
		@Args:
			filepath: path to the file containing the ground truth value
			frame_id: id of the frame
			consequence_id: id of the consequence
			pred_t: predicted translation vector
		@Return:
			absolute scale
	*/
	std::string line;
	std::ifstream file(filepath);
	if (!file.is_open()) {
		std::cerr << "Error: GT pose file could not be opened!" << std::endl;
		return -1;
	}
	int line_num = 0;

	// Read line frame_id - 1 (start from line 0)
	while (line_num < frame_id) {
		// std::cout << ">> Get line " << line_num << std::endl;
		std::getline(file, line);
		line_num++;
	}
	// Parse this line to get the translation vector prev_t
	// std::cout << "---- Frame id: " << frame_id << " ----" << std::endl;
	std::vector<int> value_indices = { 3, 7, 11};
	std::vector<double> prev_t = io_utils::line2ValuesByIndices(line, value_indices);
	// std::cout << "prev_t: " << prev_t[0] << " " << prev_t[1] << " " << prev_t[2] << std::endl;

	// Read line frame_id
	std::getline(file, line);
	std::vector<double> curr_t = io_utils::line2ValuesByIndices(line, value_indices);
	// std::cout << "curr_t: " << curr_t[0] << " " << curr_t[1] << " " << curr_t[2] << std::endl;

	// Get the absolute scale
	double scale = std::sqrt(std::pow(curr_t[0] - prev_t[0], 2) + std::pow(curr_t[1] - prev_t[1], 2) + std::pow(curr_t[2] - prev_t[2], 2)) / std::sqrt(std::pow(pred_t[0], 2) + std::pow(pred_t[1], 2) + std::pow(pred_t[2], 2));
	// Divide by the norm of the predicted translation vector
	// This might be unnecessary since the pred_t is already normalized thanks to OpenCV
	scale /= std::sqrt(std::pow(pred_t[0], 2) + std::pow(pred_t[1], 2) + std::pow(pred_t[2], 2));
	file.close();
}

// template<typename T>
std::vector<double> io_utils::line2ValuesByIndices(const std::string& line, std::vector<int> & indices){
	/*
	Parse a line to get values at indices
	*/
	std::istringstream ss(line);
	std::vector<double> values;

	int token_idx = 0;
	double token;
	int index_idx = 0;
	while (ss >> token){
		if (token_idx == indices[index_idx]){
			values.push_back(token);
			// Move on to the next index
			index_idx++;
		}
		token_idx ++;
	}
	return values;
}

std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> io_utils::readTrajectoryFromFile(const std::string& file_path, bool is_relative_pose=true){
	/*
		@Brief: read the trajectory from the file and return a vector of SE3d. Only work with the kitti odometry dataset
		@Args:
			file_path: path to the file
			is_relative_pose: whether the pose is relative to the previous pose. By default, it is true so we need to aggregate the pose.
				Assume that this relative transformation is from current to the previous frame
		@Return:
			trajectory
	*/
	std::ifstream traj_file(file_path);
	if (!traj_file.is_open()) {
		std::cerr << "Error: Trajectory file could not be opened!" << std::endl;
		return std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>();
	}
	std::string line;
	std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> trajectory;
	Sophus::SE3d prev_pose(Eigen::Quaterniond(0, 0, 0, 1), Eigen::Vector3d(0, 0, 0));
	while (std::getline(traj_file, line)) {
		std::stringstream ss(line);
		double time, tx, ty, tz, qx, qy, qz, qw;
		ss >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
		Sophus::SE3d pose(Eigen::Quaterniond(qx, qy, qz, qw), Eigen::Vector3d(tx, ty, tz));
		trajectory.push_back(prev_pose * pose);
		prev_pose = pose;
	}
	traj_file.close();
	return trajectory;
}

// Overwrite function
template<typename T>
void io_utils::readTrajectoryFromFile(const std::string& filepath, std::vector<std::vector<T>> & traj){
	/*
	Overwrite function of readTrajectoryFromFile.
	@Args:
		filepath: path to the file
		traj: trajectory
	*/
	std::ifstream traj_file(filepath);
	if (!traj_file.is_open()) {
		std::cerr << "Error: Trajectory file could not be opened!" << std::endl;
		return;
	}
	std::string line;
	std::vector<T> pose;
	while (std::getline(traj_file, line)) {
		std::stringstream ss(line);
		T token;
		while (ss >> token){
			pose.push_back(token);
		}
		traj.push_back(pose);
		pose.clear();
	}
}

// Overwrite function
template<typename T>
void io_utils::readTrajectoryFromFile(const std::string& filepath, std::vector<Eigen::Matrix<T, 4, 4>,  Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>> & traj){
	/*
	Overwrite function of readTrajectoryFromFile.
	@Args:
		filepath: path to the file
		traj: trajectory
	*/
	std::ifstream traj_file(filepath);
	if (!traj_file.is_open()) {
		std::cerr << "Error: Trajectory file could not be opened!" << std::endl;
		return;
	}
	std::string line;

	std::vector<T> pose;
	Eigen::Matrix<T, 4, 4> mat = Eigen::Matrix<T, 4, 4>::Identity();
	while (std::getline(traj_file, line)) {
		std::stringstream ss(line);
		T token;
		while (ss >> token){
			pose.push_back(token);
		}
		for (int i = 0; i < 3; i++){
			for (int j = 0; j < 4; j++){
				mat(i, j) = pose[i * 4 + j];
			}
		}
		traj.push_back(mat);
		pose.clear();
	}
}

void io_utils::readTranslationsFromFile(const std::string& filepath, std::vector<std::vector<double>> & traj){
	/*
	Read trajectory ground truth from a file. Assume that poses are given in the absolute format (current frame w.r.t the world)
	@Args:
		filepath: path to the ground truth file
		traj: container that contains the poses
	*/
	std::string line;
	std::ifstream file(filepath);
	if (!file.is_open()) {
		std::cerr << "Error: GT pose file could not be opened!" << std::endl;
		return;
	}

	// Parse each line to get the translation vector prev_t
	std::vector<int> value_indices = { 3, 7, 11};
	while (std::getline(file, line)){
		std::vector<double> trans = io_utils::line2ValuesByIndices(line, value_indices);
		traj.push_back(trans);
	}
	file.close();
}

template<typename T>
void io_utils::find_center_of_vec_points(const std::vector<std::vector<T>> & points, std::vector<T> & center){
	/*
	Find the center of points given in a vector
	@Args:
		points: a vector of points
		center: the center of the points
	*/

	if (points.size() == 0){
		throw std::runtime_error("Empty points vector!");
		return;
	}

	int point_dim = points[0].size();
	for (int i = 0; i < points.size(); i++){
		for (int j = 0; j < point_dim; j++){
			center[j] += points[i][j];
		}
	}
	for (int j = 0; j < point_dim; j++){
		center[j] /= points.size();
	}

}

template<typename T>
void  io_utils::writeTrajectoryToFile(const std::string& file_path, const std::vector<std::vector<T>> & traj){
	/*
	Write trajectory to a file
	@Args:
		file_path: path to the file
		traj: trajectory, given in the format of a vector of vectors. Each vector is a pose in the format of [R | t].flatten()
	*/
	std::ofstream file(file_path);
	if (!file.is_open()) {
		std::cerr << "Error: Trajectory file could not be opened!" << std::endl;
		return;
	}

	for (auto pose : traj){
		for (auto val : pose){
			file << val << " ";
		}
		file << std::endl;
	}
	file.close();
}
