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
	// Extract the sequence ID
	std::string sequence_id = sequence_path.substr(sequence_path.find_last_of("/") + 1);
	std::cout << "Sequence ID: " << sequence_id << std::endl;
	// Src images
	std::string src_img_path = sequence_path + "/image_1";

	// Dump result to a file
	std::string outfile_path;
	if (argc == 3)
		outfile_path = std::string(argv[2]);
	else

		outfile_path = "./results/pred_poses_" + sequence_id + ".txt";

	// Get the camera matrix
	std::string cam_path = sequence_path + "/calib.txt";
	double fx, fy, cx, cy;

	// Ground truth trajectory
	char gt_traj_filename[256];
	std::sprintf(gt_traj_filename, "%s/pose.txt", sequence_path.c_str());
	std::vector<std::vector<double>> gt_translations;
	io_utils::readTranslationsFromFile(gt_traj_filename, gt_translations);
	int max_frames = gt_translations.size();

	// Get the dimension of the trajectory
	std::vector<double> gt_traj_dim, gt_traj_dim_min_val, gt_traj_dim_max_val;
	trajectory_utils::get_traj_dim_Xd(gt_translations, gt_traj_dim, gt_traj_dim_min_val, gt_traj_dim_max_val);
	traj_width = gt_traj_dim[2] + 20;
	traj_height = gt_traj_dim[0] + 20; // add 20 here to avoid the border
	std::cout << "Trajectory dimension: " << gt_traj_dim[0] << " x " << "| z:  " << gt_traj_dim[2] << std::endl;
	traj_mat = cv::Mat::zeros(traj_width, traj_height, CV_8UC3);

	// Save video
	cv::VideoWriter video_writer;
	std::string video_filename = "./media/monoVO_KLT_video_" + sequence_id + ".avi";
	int fps = 30;

	// Initialize a MonoVO_KLT object
	MonoVO_KLT vo_klt(MIN_NUM_FEATURES);
	// Get cam intrinsics
	vo_klt.readCameraIntrinsicsFromFile(cam_path);
	std::cout << "T0 rotation:  " << vo_klt.absRotation_curr() << std::endl;;
	std::cout << "T0 translation: " << vo_klt.absTranslation_curr() << std::endl;

	// Loop
	for (auto frame_id = 0; frame_id < max_frames; frame_id++){
		std::cout << "[INFO]--------Frame " << frame_id << " --------------" << std::endl;
		// Register this frame
		vo_klt.registerImage(frame_id, src_img_path);

		// Set the scale
		double scale_curr = 1.0;
		if (frame_id > 0)
			scale_curr = io_utils::getAbsoluteScaleFromFile(gt_traj_filename, frame_id, vo_klt.relTranslation_curr());

		// If this is the 1st frame, there is no visualization
		if (frame_id == 0) continue;

		vo_klt.Scale(scale_curr);
		// Adjust the scale of the pose
		vo_klt.poseScaleAdjustment();

		// Visualize the trajectory. Substract the translation vector from the min value along each dimension of the ground truth trajectory to center the display
		cv::Mat t_f = vo_klt.absTranslation_curr();
		draw_utils::add_pose_to_trajectory_plot(traj_mat, cv::Point2d(t_f.at<double>(0) - gt_traj_dim_min_val[0], t_f.at<double>(2) - gt_traj_dim_min_val[2]), CV_RED, 1); // Ground truth
		draw_utils::add_pose_to_trajectory_plot(traj_mat, cv::Point2d(gt_translations[frame_id][0] - gt_traj_dim_min_val[0], gt_translations[frame_id][2] - gt_traj_dim_min_val[2]), CV_GREEN, 1); // Ground truth

		// Visualization
		cv::Mat frames_combined;
		resized_traj_mat = traj_mat.clone();
		// Visualize feature matches
		cv::Mat img_prev_color = vo_klt.colorFramePrev();
		cv::Mat img_curr_color = vo_klt.colorFrameCurr();
		std::vector<cv::Point2f> points_prev = vo_klt.pointsPrev();
		std::vector<cv::Point2f> points_curr = vo_klt.pointsCurr();

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
		if (cv::waitKey(1) == 27) {
			break;
		}

		if (frame_id  == 1){
			int out_video_width = merged_plot.cols, out_video_height= merged_plot.rows;
			// resize by half to make the video small
			video_writer.open(video_filename, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(out_video_width, out_video_height));
			if (!video_writer.isOpened()) {
				std::cout << "[ERROR] Cannot open video writer." << std::endl;
				return -1;
			}
		}

		video_writer.write(merged_plot);

	}

	// Dumb the predicted trajectory to a file
	io_utils::writeTrajectoryToFile(outfile_path, vo_klt.predictedTrajSO3());
	cv::destroyAllWindows();
	return 0;
}
#endif