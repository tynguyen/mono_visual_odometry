#pragma once

#include <iostream>
#include <opencv2/core/core.hpp>
#include <vector>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sstream>
#include <fstream>
#include "io_utils/io_utils.hpp"

class MonoVO_KLT{
	public:
		/*
		Methods
		*/
		MonoVO_KLT(int min_num_features):_min_num_features(min_num_features){};
		// Overloaded constructor
		MonoVO_KLT(){_min_num_features = 2000;};
		// Get camera intrinsics
		void readCameraIntrinsicsFromFile(const std::string &calib_file, int cam_id);

		// Add a new image
		void registerImage(const cv::Mat &frame);
		// Overloaded method
		void registerImage(const std::string &img_path);
		// Overloaded method
		void registerImage(const int &img_id, const std::string &src_path);


		// Initialize features
		void initFeatures(const cv::Mat &frame, std::vector<cv::Point2f> &features);

		// // Relative pose prediction
		// void predictPose(const cv::Mat &frame_curr);

		// // Run a loop of pose prediction over a sequence of images
		// void runImageSeqPosePrediction(const int &start_id, const int &end_id, const std::string &img_path);


		// Set min number of features
		void minNumFeatures(int min_num_features){_min_num_features = min_num_features;};
		int minNUmFeatures(){return _min_num_features;};

		// Set camera intrinsics
		cv::Mat cameraIntrinsicsMat(){return  _cam_intrinsics_mat;};
		void cameraIntrinsicsMat(cv::Mat &cam_intrinsics_mat){_cam_intrinsics_mat = cam_intrinsics_mat;};

		// Get predicted trajectory
		std::vector<std::vector<double>> predictedTrajSO3(){
			return _pred_traj_so3;
		};

		// Get the pose of the current frame w.r.t the world
		cv::Mat absRotation_curr(){return _R_f;};
		cv::Mat absTranslation_curr(){return _t_f;};

		// Get the relative pose of the current frame
		cv::Mat relRotation_curr(){return _R;};
		cv::Mat relTranslation_curr(){return _t;};

		// Set, get the scale factor
		void Scale(const double &scale){
			_scale_curr = scale;
			_scale_is_reset = true;
			};
		double Scale(){return _scale_curr;};

		// Adjust the scale of the pose
		void poseScaleAdjustment();

		// Get the previous color image
		cv::Mat colorFramePrev(){return  _rgb_prev;};

		// Get the current color image
		cv::Mat colorFrameCurr(){return  _rgb_curr;};

		// Get the previous points_2f
		std::vector<cv::Point2f> pointsPrev(){return _points_prev;};

		// Get the current points_2f
		std::vector<cv::Point2f> pointsCurr(){return _points_curr;};


	protected:
		// Read image frame from file
		void _readImageFrame(const std::string &img_file, cv::Mat &frame);
		// Overload operator
		// Read image from file given by its ID and a source image path
		void _readImageFrame(const int &img_id, const std::string &src_path, cv::Mat &frame);

		// Initialize the initial pose (T0)
		void _T0(){
			// This function should execute only once
			if (_frame_id > 0) return;
			if (_pred_traj_so3.size() > 0) return;
			std::vector<double> pose_vec = {1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0};
			_pred_traj_so3.push_back(pose_vec);
			_t = (cv::Mat_<double>(3,1) << 0., 0., 0.);
			_R = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1 );
			_R_f = _R.clone();
			_t_f = _t.clone();
			std::cout << "[INFO] Set _R_f " << _R_f << std::endl;
			std::cout << "[INFO] Set _t_f " << _t_f << std::endl;
		};
		// Estimate relative pose using Essential matrix
		void _EssentialMatBasedPosePrediction(std::vector<cv::Point2f> &points_curr, std::vector<cv::Point2f> &points_prev, cv::Mat &cam_intrinsics_mat, cv::Mat &R, cv::Mat &t);


	private:
		// Minimum number of tracked features. If smaller -> reinitialize
		int _min_num_features;
		// Camera intrinsics
		cv::Mat _cam_intrinsics_mat;
		// Current frame distance scale factor
		double _scale_curr;
		bool _scale_is_reset = false; // This to ensure that a proper scale value is set per each frame

		// Frames
		cv::Mat _frame_prev, _frame_curr;
		cv::Mat _rgb_prev, _rgb_curr;

		// Keep the status of tracked features
		std::vector<u_char> _status_vec;

		// Predicted trajectory
		std::vector<std::vector<double>> _pred_traj_so3;

		// Frame ID
		int _frame_id = -1; // -1 because we need to initialize the first frame and set _frame_id to 0
		// Keypoints and Points to be tracked
		std::vector<cv::Point2f> _points_prev, _points_curr;
		std::vector<cv::KeyPoint> _keypoints_prev, _keypoints_curr;

		// Absolute pose (current frame w.r.t the world)
		cv::Mat _R_f, _t_f;

		// Relative pose (prev frame w.r.t the current frame)
		cv::Mat _R, _t;

};

// Get camera intrinsics
void MonoVO_KLT::readCameraIntrinsicsFromFile(const std::string &calib_file, int cam_id=1){
	/*
		Read camera intrinsics from a calibration file
		Input:
			calib_file: calibration file
			cam_id: camera id
		Output:
			_cam_intrinsic_mat: camera intrinsics
	*/
	double fx, fy, cx, cy;
	io_utils::readIthCamMatFromFile(calib_file, cam_id, fx, fy, cx, cy); // start from 1
	_cam_intrinsics_mat = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
}

// Read image frame from file
void MonoVO_KLT::_readImageFrame(const std::string &img_file, cv::Mat &frame){
	// _rgb_curr = cv::imread(img_file, -1);
	frame = cv::imread(img_file, cv::IMREAD_COLOR);
	// cv::cvtColor(_rgb_curr, _frame_curr, cv::COLOR_BGR2GRAY);
}

// Overload operator
// Read image from file given by its ID and a source image path
void MonoVO_KLT::_readImageFrame(const int &img_id, const std::string &src_path, cv::Mat &frame){
	char filename[256];
	std::sprintf(filename, "%s/%06d.png", src_path.c_str(), img_id);
	std::cout << "[INFO] Reading image " << filename << std::endl;
	std::string filename_str(filename);
	_readImageFrame(filename_str, frame);
}

// Add a new image
void MonoVO_KLT::registerImage(const cv::Mat &frame){
	/*
		Register a new image
		Input:
			frame: image frame to be registered
	*/
	_frame_id += 1;

	if (frame.data == nullptr){
		std::cerr << "Error: Image frame is empty" << std::endl;
		return;
	}


	// If the first frame, initialize the first pose
	if (_frame_id == 0){
		_rgb_prev = frame.clone();
		_rgb_curr = frame.clone();
		cv::cvtColor(frame, _frame_prev, cv::COLOR_BGR2GRAY);
		_frame_curr = _frame_prev.clone();
		_scale_curr = 1.;
		// Initialize the first pose
		_T0();
		// Find features
		feature_utils::FASTFeatureDetection(_frame_prev, _points_prev);
		return;
	}

	// If the current frame is the second frame and afterwards
	_rgb_curr = frame.clone();
	cv::cvtColor(frame, _frame_curr, cv::COLOR_BGR2GRAY);

	// Get features from previous frame
	if (_points_prev.size() < _min_num_features){
		std::cout << "[INFO] Number of tracking features " << _points_prev.size() << " < threshold = " << _min_num_features  << std::endl;
		feature_utils::FASTFeatureDetection(_frame_prev, _points_prev);
		std::cout << "[INFO] Found new " << _points_prev.size() << " points" << std::endl;
	}

	// Track features
	feature_utils::KLTFeatureTracking(_frame_prev, _frame_curr, _points_prev, _points_curr, _status_vec);
	std::cout << "[INFO] Number of matched points: " << _points_prev.size() << std::endl;

	// Predict the relative pose
	_EssentialMatBasedPosePrediction(_points_curr, _points_prev, _cam_intrinsics_mat, _R, _t);


	// Prepare for the next frame registration
	_rgb_prev = _rgb_curr.clone();
	_frame_prev = _frame_curr.clone();

}

// Overloaded method add a new image
void MonoVO_KLT::registerImage(const std::string &img_path){
	/*
		Register a new image
		Input:
			img_path: image absolute path
	*/
	// Read image
	cv::Mat frame;
	_readImageFrame(img_path, frame);
	registerImage(frame);
}

// Overloaded method add a new image
void MonoVO_KLT::registerImage(const int &img_id, const std::string &src_path){
	/*
		Register a new image
		Input:
			img_id: image id
			src_path: source image path
	*/
	// Read image
	cv::Mat frame;
	_readImageFrame(img_id, src_path, frame);
	registerImage(frame);
}

// Estimate relative pose using Essential matrix
void MonoVO_KLT::_EssentialMatBasedPosePrediction(std::vector<cv::Point2f> &points_curr, std::vector<cv::Point2f> &points_prev, cv::Mat &cam_intrinsics_mat, cv::Mat &R, cv::Mat &t){
	cv::Mat E, mask;
	double fx = cam_intrinsics_mat.at<double>(0,0);
	int cx = cam_intrinsics_mat.at<double>(0, 2);
	int cy = cam_intrinsics_mat.at<double>(1, 2);
	// Find essential matrix
	E = cv::findEssentialMat(points_curr, _points_prev, fx, cv::Point2d(cx, cy), cv::RANSAC, 0.999, 1.0, mask);
	// Recover relative pose
	cv::recoverPose(E, points_curr, points_prev, R, t, fx, cv::Point2d(cx, cy), mask);
}

// Scale adjustment
void MonoVO_KLT::poseScaleAdjustment(){
	// Ensure a proper scale value is calculated for this frame
	if (!_scale_is_reset){
		throw std::runtime_error("[ERROR] A new scale value is not calculated for this frame yet! Please run scale(const double &scale) before registering this frame.");
	}

	// Update the absolute pose (w.r.t the world)
	_t_f = _t_f + _scale_curr * _R_f * _t;
	_R_f = _R * _R_f;

	// Add to the trajectory
	std::vector<double> pose_vec = {_R_f.at<double>(0,0), _R_f.at<double>(0,1), _R_f.at<double>(0,2), _t_f.at<double>(0),
								    _R_f.at<double>(1,0), _R_f.at<double>(1,1), _R_f.at<double>(1,2), _t_f.at<double>(1),
									_R_f.at<double>(2,0), _R_f.at<double>(2,1), _R_f.at<double>(2,2), _t_f.at<double>(2)};

	_pred_traj_so3.push_back(pose_vec);

	// A new scale is required for the next frame
	_scale_is_reset = false;

}
