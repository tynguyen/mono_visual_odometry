#pragma once

#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>

namespace draw_utils {
	void add_pose_to_trajectory_plot(cv::Mat& img, const cv::Mat& trans_vec, cv::Scalar color, int line_thickness);
	// Overwrite
	void add_pose_to_trajectory_plot(cv::Mat& img, const std::vector<double>& trans_vec, cv::Scalar color, int line_thickness);
	// Overwrite
	void add_pose_to_trajectory_plot(cv::Mat& img, const Sophus::SE3d& pose_se3d, cv::Scalar color, int line_thickness);
	// Overwrite
	void add_pose_to_trajectory_plot(cv::Mat& img, const cv::Point2d & trans_point, cv::Scalar color, int line_thickness);
};

void draw_utils::add_pose_to_trajectory_plot(cv::Mat& img, const cv::Mat& trans_vec, cv::Scalar color=cv::Scalar(100, 100, 100), int line_thickness=2){
	/*
		Add pose to trajectory plot
		Input:
			img: image to draw on
			trans_vec: translation vector (2,)
			color: color of the pose
			line_thickness: thickness of the line
		Output:
			img: image with pose added
	*/
	// Plot dims
	int plot_width = img.cols, plot_height = img.rows;
	// Find x, y coordinates
	int x = int(trans_vec.at<double>(0));
	int y = int(trans_vec.at<double>(1));
	cv::circle(img, cv::Point(x, y), 1, color, line_thickness);
}

// overwrite
void draw_utils::add_pose_to_trajectory_plot(cv::Mat& img, const Sophus::SE3d& pose_se3d, cv::Scalar color, int line_thickness){
	/*
		Add pose to trajectory plot
		Input:
			img: image to draw on
			pose_se3d: pose to draw (Sophus::SE3d)
			color: color of the pose
			line_thickness: thickness of the line
		Output:
			img: image with pose added
	*/
}

// Overwrite
void draw_utils::add_pose_to_trajectory_plot(cv::Mat& img, const std::vector<double>& trans_vec, cv::Scalar color, int line_thickness){
	/*
		Add pose to trajectory plot
		Input:
			img: image to draw on
			trans_vec: translation vector (2,)
			color: color of the pose
			line_thickness: thickness of the line
		Output:
			img: image with pose added
	*/
	// Plot dims
	int plot_width = img.cols, plot_height = img.rows;
	// Find x, y coordinates
	int x = int(trans_vec.at(0));
	int y = int(trans_vec.at(1));
	cv::circle(img, cv::Point(x, y), 1, color, line_thickness);
}

// Overwrite
void draw_utils::add_pose_to_trajectory_plot(cv::Mat& img, const cv::Point2d & trans_point, cv::Scalar color, int line_thickness){
	/*
		Add pose to trajectory plot
		Input:
			img: image to draw on
			trans_point: translational vector on the ground plane (x, y)
			color: color of the pose
			line_thickness: thickness of the line
		Output:
			img: image with pose added
	*/
	// Plot dims
	int plot_width = img.cols, plot_height = img.rows;
	// Find x, y coordinates
	int x = int(trans_point.x);
	int y = int(trans_point.y);
	cv::circle(img, cv::Point(x, y), 1, color, line_thickness);
}