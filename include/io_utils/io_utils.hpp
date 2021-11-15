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

	std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> readTrajectoryFromFile(const std::string& file_path);
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

std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> io_utils::readTrajectoryFromFile(const std::string& file_path){
	/*
		@Brief: read the trajectory from the file and return a vector of SE3d. Only work with the kitti odometry dataset
		@Args:
			file_path: path to the file
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
	while (std::getline(traj_file, line)) {
		std::stringstream ss(line);
		double time, tx, ty, tz, qx, qy, qz, qw;
		ss >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
		Sophus::SE3d pose(Eigen::Quaterniond(qx, qy, qz, qw), Eigen::Vector3d(tx, ty, tz));
		trajectory.push_back(pose);
	}
	return trajectory;
}