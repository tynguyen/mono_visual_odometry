#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>
#include <chrono>

#include "feature_utils/feature_utils.hpp"
#include "monoVO_KLT/monoVO_KLT.hpp"
#include "io_utils/io_utils.hpp"

// Global variables
#ifndef MAX_FRAMES
#define MAX_FRAMES 8000
#endif

#ifndef MIN_NUM_FEATURES
#define MIN_NUM_FEATURES 2000
#endif

// Use a MACRO to let the compiler know when to compile this main function (for application) or not (for unit testing)
// To compile this application, define the macro -DTESTING=0 or (OFF) or False
#ifndef TESTING
int main(int argc, char** argv){
	// Visualization
	cv::namedWindow("Cam Video",  cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Trajectory", cv::WINDOW_AUTOSIZE);


	// Dump result to a fileTHIRD_PARTY_LIBS
	std::ofstream outfile;
	if (argc == 3)
		outfile.open(argv[2]);
	else
		outfile.open("results/pred_poses_01.txt");

	std::cout << "Reading images..." << std::endl;

	std::string sequence_path = "/media/tynguyen/docker_folder/kitti/dataset/sequences/01";
	if (argc >= 2)
		sequence_path = std::string(argv[1]);
	std::string src_img_path = sequence_path + "/image_1";

	// Predicted trajecatory
	std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> pred_trajectory;

	// Ground truth trajectory
	char gt_traj_filename[256];
	std::sprintf(gt_traj_filename, "%s/pose.txt", sequence_path.c_str());
	std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> gt_trajectory = io_utils::readTrajectoryFromFile(gt_traj_filename);

	char img_name[256];
	std::sprintf(img_name, "%s/%06d.png", src_img_path.c_str(), 0);
	std::string img_name_1(img_name);
	std::cout << "Reading image 0 from " << img_name << std::endl;
	std::sprintf(img_name, "%s/%06d.png", src_img_path.c_str(), 1);
	std::cout << "Reading image 1 from " << img_name << std::endl;
	std::string img_name_2(img_name);

	cv::Mat img_prev_color = cv::imread(img_name_1, cv::IMREAD_COLOR);
	cv::Mat img_curr_color = cv::imread(img_name_2, cv::IMREAD_COLOR);
	cv::Mat img_prev, img_curr;
	cv::cvtColor(img_prev_color, img_prev, cv::COLOR_BGR2GRAY);
	cv::cvtColor(img_curr_color, img_curr, cv::COLOR_BGR2GRAY);

	if (img_prev.data == nullptr || img_curr.data == nullptr) {
		std::cout << "Error: Images could not be read!" << std::endl;
		return -1;
	}

	// Timer
	std::chrono::steady_clock::time_point t1, t2;
	std::chrono::duration<double> time_span;

	// Feature detection
	std::vector<cv::Point2f> points_prev;
	std::vector<cv::KeyPoint> keypoints_prev;
	// It turns out that ORB takes 0.117 secs while FAST takes 0.0017 secs
	// t1 = std::chrono::steady_clock::now();
	// feature_utils::ORBFeatureDetection(img_prev, points_prev);
	// t2 = std::chrono::steady_clock::now();
	// time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
	// std::cout << "[INFO] ORB: Found "<< points_prev.size() << " features in " << time_span.count() << " seconds." << std::endl;
	t1 = std::chrono::steady_clock::now();
	feature_utils::FASTFeatureDetection(img_prev, points_prev);
	t2 = std::chrono::steady_clock::now();
	time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
	std::cout << "[INFO] FAST: Found "<< points_prev.size() << " features in " << time_span.count() << " seconds." << std::endl;

	// Feature tracking
	t1 = std::chrono::steady_clock::now();
	std::vector<uchar> status; // status of tracked features
	std::vector<cv::Point2f> points_curr;
	std::vector<cv::KeyPoint> keypoints_curr;
	feature_utils::KLTFeatureTracking(img_prev, img_curr, points_prev, points_curr, status);
	t2 = std::chrono::steady_clock::now();
	time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
	std::cout << "[INFO] KLT feature matching time: "<< time_span.count() << " seconds." << std::endl;
	std::cout << "[INFO] Number of matched points: " << points_prev.size() << std::endl;

	// Visualization
	cv::Mat frames_combined;
	feature_utils::drawMatchesUsingPoint2f(img_prev_color, points_prev, img_curr_color, points_curr, frames_combined);
	cv::resize(frames_combined, frames_combined, cv::Size(), 0.5, 0.5);
	cv::imshow("Cam Video", frames_combined);

	// Computer essential matrix
	// Get the camera matrix
	std::string cam_path = sequence_path + "/calib.txt";
	double fx, fy, cx, cy;
	io_utils::readIthCamMatFromFile(cam_path, 1, fx, fy, cx, cy); // start from 1
	std::cout << "[INFO] Camera matrix: " << std::endl;
	std::cout << "fx: " << fx << std::endl;
	std::cout << "fy: " << fy << std::endl;
	std::cout << "cx: " << cx << std::endl;
	std::cout << "cy: " << cy << std::endl;

	cv::Mat mask, E, R, t;
	E = cv::findEssentialMat(points_prev, points_curr, fx, cv::Point2d(cx, cy), cv::RANSAC, 0.999, 1.0, mask);

	// Recover relative pose
	cv::recoverPose(E, points_prev, points_curr, R, t, fx, cv::Point2d(cx, cy), mask);


	// Prepare for the loop
	points_prev = points_curr;
	keypoints_prev = keypoints_curr;
	img_prev = img_curr.clone();
	img_prev_color = img_curr_color.clone();

	// Final rotation and translation matrices
	cv::Mat R_f = R.clone();
	cv::Mat t_f = t.clone();

	// Get the scale
	std::vector<double> pred_t_vec = {t.at<double>(0), t.at<double>(1), t.at<double>(2)};
	double scale = io_utils::getAbsoluteScaleFromFile(gt_traj_filename, 1, pred_t_vec);
	t_f *= scale;
	// Append the predicted pose to the history
	Eigen::Matrix3d R_eigen_pred;
	Eigen::Vector3d t_eigen_pred;
	cv::cv2eigen(R_f, R_eigen_pred);
	cv::cv2eigen(t_f, t_eigen_pred);
	Sophus::SE3d T_se3d_pred(R_eigen_pred, t_eigen_pred);
	pred_trajectory.push_back(T_se3d_pred);

	int traj_width = 1400, traj_height = 1400;
	cv::Mat traj_mat = cv::Mat::zeros(traj_width, traj_height, CV_8UC3);
	char img_filename[256];
	for (int frame_curr = 2; frame_curr < MAX_FRAMES; frame_curr++) {
		// If the number of features < MIN_NUM_FEATURES, find more features
		if (points_prev.size() < MIN_NUM_FEATURES) {
			std::cout << "[INFO] Number of features < " << MIN_NUM_FEATURES << " in frame " << frame_curr << std::endl;
			std::cout << "[INFO] Finding more features..." << std::endl;
			t1 = std::chrono::steady_clock::now();
			feature_utils::FASTFeatureDetection(img_prev, points_prev);
			t2 = std::chrono::steady_clock::now();
			time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
			std::cout << "[INFO] FAST: Found "<< points_prev.size() << " features in " << time_span.count() << " seconds." << std::endl;
		}

		// Read next frame
		std::sprintf(img_filename, "%s/%06d.png", src_img_path.c_str(), frame_curr);
		img_curr_color = cv::imread(img_filename, cv::IMREAD_COLOR);
		if (!img_curr_color.data)
			break;
		cv::cvtColor(img_curr_color, img_curr, cv::COLOR_BGR2GRAY);

		// Find R, T from current frame to previous frame (basically the translation vector from previous frame to current frame in the previous frame)
		feature_utils::KLTFeatureTracking(img_prev, img_curr, points_prev, points_curr, status);
		E = cv::findEssentialMat(points_prev, points_curr, fx, cv::Point2d(cx, cy), cv::RANSAC, 0.999, 1.0, mask);
		cv::recoverPose(E, points_prev, points_curr, R, t, fx, cv::Point2d(cx, cy), mask);

		// Get the scale
		std::vector<double> pred_t_vec = {t.at<double>(0), t.at<double>(1), t.at<double>(2)};
		scale = io_utils::getAbsoluteScaleFromFile(gt_traj_filename, frame_curr, pred_t_vec);

		// Update the trajectory
		t_f = t_f + scale*R_f * t;
		R_f = R_f*R;

		// Visualization
		int x = int(t_f.at<double>(0)) + traj_width/2;
		int y = int(t_f.at<double>(2)) + traj_height - 200;

		cv::circle(traj_mat, cv::Point(x, y), 1, cv::Scalar(0, 255, 0), 1);
		cv::Mat resized_traj_mat;
		cv::resize(traj_mat, resized_traj_mat, cv::Size(traj_width/2, traj_height/2));
		cv::imshow("Trajectory", resized_traj_mat);
		cv::rectangle(traj_mat, cv::Point(10, 30), cv::Point(550, 50), cv::Scalar(0, 0, 0), -1);
		// Visualization
		feature_utils::drawMatchesUsingPoint2f(img_prev_color, points_prev, img_curr_color, points_curr, frames_combined, 20);
		cv::resize(frames_combined, frames_combined, cv::Size(), 0.5, 0.5);
		cv::imshow("Cam Video", frames_combined);

		if (cv::waitKey(1) == 27) {
			break;
		}
		// cv::waitKey(30);

		// Assign the current frame to the previous frame
		img_prev = img_curr.clone();
		points_prev = points_curr;
		keypoints_prev = keypoints_curr;
		img_prev_color = img_curr_color.clone();

		// Convert the predicted R, t to SE3d
		cv::cv2eigen(R_f, R_eigen_pred);
		cv::cv2eigen(t_f, t_eigen_pred);
		Sophus::SE3d T_se3d_pred(R_eigen_pred, t_eigen_pred);
		pred_trajectory.push_back(T_se3d_pred);
	}

	cv::destroyAllWindows();
	return 0;
}
#endif