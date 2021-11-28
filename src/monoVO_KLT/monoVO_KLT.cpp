#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <vector>
#include <chrono>

#include "feature_utils/feature_utils.hpp"
#include "monoVO_KLT/monoVO_KLT.hpp"
#include "io_utils/io_utils.hpp"
#include "trajectory_utils/trajectory_utils.hpp"
#include "draw_utils/draw_utils.hpp"

// Global variables
#ifndef MIN_NUM_FEATURES
#define MIN_NUM_FEATURES 3000
#endif

#ifndef CV_RED
#define CV_RED cv::Scalar(0, 0, 255)
#endif

#ifndef CV_GREEN
#define CV_GREEN cv::Scalar(0, 255, 0)
#endif

// Use a MACRO to let the compiler know when to compile this main function (for application) or not (for unit testing)
// To compile this application, define the macro -DTESTING=0 or (OFF) or False
#ifndef TESTING
int main(int argc, char** argv){
	// Trajectory plot (using cv::Mat)
	int traj_width = 1400, traj_height = 1400;
	cv::Mat traj_mat;
	cv::Mat resized_traj_mat; // Smaller for visualization



	std::cout << "Reading images..." << std::endl;

	std::string sequence_path = "/media/tynguyen/docker_folder/kitti/dataset/sequences/01";
	if (argc >= 2)
		sequence_path = std::string(argv[1]);
	std::string src_img_path = sequence_path + "/image_1";

	// Extract the sequence ID
	std::string sequence_id = sequence_path.substr(sequence_path.find_last_of("/") + 1);
	std::cout << "Sequence ID: " << sequence_id << std::endl;

	// Dump result to a file
	std::string outfile_path;
	if (argc == 3)
		outfile_path = std::string(argv[2]);
	else

		outfile_path = "../results/pred_poses_" + sequence_id + ".txt";

	// Predicted trajecatory
	std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> pred_trajectory;
	std::vector<std::vector<double>> pred_translations;
	std::vector<std::vector<double>> pred_traj_so3;

	// Ground truth trajectory
	char gt_traj_filename[256];
	std::sprintf(gt_traj_filename, "%s/pose.txt", sequence_path.c_str());
	std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> gt_trajectory = io_utils::readTrajectoryFromFile(gt_traj_filename);
	std::vector<std::vector<double>> gt_translations;
	io_utils::readTranslationsFromFile(gt_traj_filename, gt_translations);

	// Get the dimension of the trajectory
	std::vector<double> gt_traj_dim, gt_traj_dim_min_val, gt_traj_dim_max_val;
	trajectory_utils::get_traj_dim_Xd(gt_translations, gt_traj_dim, gt_traj_dim_min_val, gt_traj_dim_max_val);
	traj_width = gt_traj_dim[2] + 20;
	traj_height = gt_traj_dim[0] + 20; // add 20 here to avoid the border
	std::cout << "Trajectory dimension: " << gt_traj_dim[0] << " x " << "| z:  " << gt_traj_dim[2] << std::endl;
	traj_mat = cv::Mat::zeros(traj_width, traj_height, CV_8UC3);


	// Add T0 = [I, [0, 0, 0]] to the predicted trajectory
	Sophus::SE3d _T0(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
	pred_trajectory.push_back(_T0);
	std::vector<double> _t0(3, 0);
	pred_translations.push_back(_t0);
	std::vector<double> t_vec_pred_traj = {1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0};
	pred_traj_so3.push_back(t_vec_pred_traj);

	int max_frames = gt_trajectory.size();

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

	// Transformation from the prev frame to the curr frame
	cv::Mat mask, E, R, t;
	E = cv::findEssentialMat(points_curr, points_prev, fx, cv::Point2d(cx, cy), cv::RANSAC, 0.999, 1.0, mask);

	// Recover relative pose
	cv::recoverPose(E, points_curr, points_prev, R, t, fx, cv::Point2d(cx, cy), mask);


	// Prepare for the loop
	points_prev = points_curr;
	keypoints_prev = keypoints_curr;
	img_prev = img_curr.clone();
	img_prev_color = img_curr_color.clone();

	// Final rotation and translation matrices
	// cv::Mat rot_cam_to_odom = (cv::Mat_<double>(3,3) << 0, -1, 0, 0, 0, -1, 1, 0, 0);
	cv::Mat R_f = R.clone();
	cv::Mat t_f = t.clone();

	// Get the scale
	std::vector<double> pred_t_vec = {t.at<double>(0), t.at<double>(1), t.at<double>(2)};
	double scale = io_utils::getAbsoluteScaleFromFile(gt_traj_filename, 1, pred_t_vec);
	t_f *= scale;
	// Visualize the trajectory. Substract the translation vector from the min value along each dimension of the ground truth trajectory to center the display
	draw_utils::add_pose_to_trajectory_plot(traj_mat, cv::Point2d(t_f.at<double>(0) - gt_traj_dim_min_val[0], t_f.at<double>(2) - gt_traj_dim_min_val[2]), CV_RED, 1); // Ground truth
	draw_utils::add_pose_to_trajectory_plot(traj_mat, cv::Point2d(gt_translations[1][0] - gt_traj_dim_min_val[0], gt_translations[1][2] - gt_traj_dim_min_val[2]), CV_GREEN, 1); // Ground truth

	// Visualization
	cv::Mat frames_combined;
	resized_traj_mat = traj_mat.clone();
	// Visualize feature matches
	feature_utils::drawMatchesUsingPoint2f(img_prev_color, points_prev, img_curr_color, points_curr, frames_combined, 20);
	cv::resize(frames_combined, frames_combined, cv::Size(), 0.5, 0.5);
	// Merge the two images
	cv::Mat merged_plot;
	if (resized_traj_mat.rows > resized_traj_mat.cols) {
		cv::rotate(resized_traj_mat, resized_traj_mat, cv::ROTATE_90_CLOCKWISE);
	}
	// cv::resize(resized_traj_mat, resized_traj_mat, cv::Size(frames_combined.cols, (int)(resized_traj_mat.rows * frames_combined.cols / resized_traj_mat.cols)));
	cv::resize(resized_traj_mat, resized_traj_mat, cv::Size(frames_combined.cols, (int)(resized_traj_mat.rows * frames_combined.cols / resized_traj_mat.cols)));
	cv::vconcat(frames_combined, resized_traj_mat, merged_plot);
	cv::imshow("Front cam [prev_frame, curr_fram] v.s. Trajectory", merged_plot);

	// Save video
	cv::VideoWriter video_writer;
	std::string video_filename = "../media/monoVO_KLT_video_" + sequence_id + ".avi";
	int fps = 30;
	int out_video_width = merged_plot.cols, out_video_height= merged_plot.rows;
	// resize by half to make the video small
	video_writer.open(video_filename, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(out_video_width, out_video_height));
	if (!video_writer.isOpened()) {
		std::cout << "[ERROR] Cannot open video writer." << std::endl;
		return -1;
	}
	video_writer.write(merged_plot);

	// Append the predicted pose to the history
	Eigen::Matrix3d R_eigen_pred;
	Eigen::Vector3d t_eigen_pred;
	cv::cv2eigen(R_f, R_eigen_pred);
	cv::cv2eigen(t_f, t_eigen_pred);
	Sophus::SE3d T_se3d_pred(R_eigen_pred, t_eigen_pred);
	pred_trajectory.push_back(T_se3d_pred);
	std::vector<double> t_vec_pred = {t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2)};
	pred_translations.push_back(t_vec_pred);
	t_vec_pred_traj = {R_f.at<double>(0,0), R_f.at<double>(0,1), R_f.at<double>(0,2), t_f.at<double>(0), R_f.at<double>(1,0), R_f.at<double>(1,1), R_f.at<double>(1,2), t_f.at<double>(1), R_f.at<double>(2,0), R_f.at<double>(2,1), R_f.at<double>(2,2), t_f.at<double>(2)};
	pred_traj_so3.push_back(t_vec_pred_traj);

	char img_filename[256];
	for (int frame_curr = 2; frame_curr < max_frames; frame_curr++) {
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
		std::cout << "[INFO] Reading frame " << frame_curr << "/" << max_frames << " from " << img_filename << std::endl;
		img_curr_color = cv::imread(img_filename, cv::IMREAD_COLOR);
		if (!img_curr_color.data)
			break;
		cv::cvtColor(img_curr_color, img_curr, cv::COLOR_BGR2GRAY);

		// Find R, T from the prev frame to the current frame
		feature_utils::KLTFeatureTracking(img_prev, img_curr, points_prev, points_curr, status);
		E = cv::findEssentialMat(points_curr, points_prev, fx, cv::Point2d(cx, cy), cv::RANSAC, 0.999, 1.0, mask);
		cv::recoverPose(E, points_curr, points_prev, R, t, fx, cv::Point2d(cx, cy), mask);

		// Get the scale
		std::vector<double> pred_t_vec = {t.at<double>(0), t.at<double>(1), t.at<double>(2)};
		scale = io_utils::getAbsoluteScaleFromFile(gt_traj_filename, frame_curr, pred_t_vec);

		// Update the trajectory
		t_f = t_f + scale*R_f * t; // t_f: from the current frame to the world
		R_f = R * R_f; // R_f: from the world to the current frame

		// Visualize the trajectory
		draw_utils::add_pose_to_trajectory_plot(traj_mat, cv::Point2d(t_f.at<double>(0) - gt_traj_dim_min_val[0], t_f.at<double>(2) - gt_traj_dim_min_val[2]), CV_RED, 1); // Ground truth
		draw_utils::add_pose_to_trajectory_plot(traj_mat, cv::Point2d(gt_translations[frame_curr][0] - gt_traj_dim_min_val[0], gt_translations[frame_curr][2] - gt_traj_dim_min_val[2]), CV_GREEN, 1); // Ground truth
		std::cout << "--Pred: " << t_f.t() << std::endl;
		std::cout << "--GT: "
			<< gt_translations[frame_curr][0] << " "
			<< gt_translations[frame_curr][1] << " "
			<< gt_translations[frame_curr][2] << " "
		 	<< std::endl;

		resized_traj_mat = traj_mat.clone();
		// Visualize feature matches
		feature_utils::drawMatchesUsingPoint2f(img_prev_color, points_prev, img_curr_color, points_curr, frames_combined, 20);
		cv::resize(frames_combined, frames_combined, cv::Size(), 0.5, 0.5); // Resize by half to make the video small
		// Merge the two images
		cv::Mat merged_plot;
		if (resized_traj_mat.rows > resized_traj_mat.cols) {
			cv::rotate(resized_traj_mat, resized_traj_mat, cv::ROTATE_90_CLOCKWISE);
		}
		cv::resize(resized_traj_mat, resized_traj_mat, cv::Size(frames_combined.cols, (int)(resized_traj_mat.rows * frames_combined.cols / resized_traj_mat.cols)));
		cv::vconcat(frames_combined, resized_traj_mat, merged_plot);
		cv::imshow("Front cam [prev_frame, curr_fram] v.s. Trajectory", merged_plot);
		video_writer.write(merged_plot);
		assert (merged_plot.cols == out_video_width && merged_plot.rows == out_video_height);
		if (cv::waitKey(1) == 27) {
			break;
		}

		// Assign the current frame to the previous frame
		img_prev = img_curr.clone();
		points_prev = points_curr;
		keypoints_prev = keypoints_curr;
		img_prev_color = img_curr_color.clone();

		// Convert the predicted R, t to So3
		cv::cv2eigen(R_f, R_eigen_pred);
		cv::cv2eigen(t_f, t_eigen_pred);
		Sophus::SE3d T_se3d_pred(R_eigen_pred, t_eigen_pred);
		pred_trajectory.push_back(T_se3d_pred);
		t_vec_pred = {t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2)};
		pred_translations.push_back(t_vec_pred);
		t_vec_pred_traj = {R_f.at<double>(0,0), R_f.at<double>(0,1), R_f.at<double>(0,2), t_f.at<double>(0), R_f.at<double>(1,0), R_f.at<double>(1,1), R_f.at<double>(1,2), t_f.at<double>(1), R_f.at<double>(2,0), R_f.at<double>(2,1), R_f.at<double>(2,2), t_f.at<double>(2)};
		pred_traj_so3.push_back(t_vec_pred_traj);

	}

	// Dumb the predicted trajectory to a file
	io_utils::writeTrajectoryToFile(outfile_path, pred_traj_so3);

	// // Evaluate the trajectory estimation
	// std::cout << "[INFO] Evaluating the trajectory..." << std::endl;
	// double ATE_all = trajectory_utils::find_absolute_trajectory_error(pred_trajectory, gt_trajectory);
	// std::cout << "[INFO] Absolute traj error: " << ATE_all << std::endl;
	// double ATE_trans = trajectory_utils::find_average_translational_error(pred_trajectory, gt_trajectory);
	// std::cout << "[INFO] Average translational error: " << ATE_trans << std::endl;

	cv::destroyAllWindows();
	return 0;
}
#endif