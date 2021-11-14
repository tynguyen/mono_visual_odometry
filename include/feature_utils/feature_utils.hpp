#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

namespace feature_utils {
	void ORBFeatureDetection(
		const cv::Mat&,
		std::vector<cv::Point2f>&
	);

	void FASTFeatureDetection(
		const cv::Mat&,
		std::vector<cv::Point2f>&
	);

	void KLTFeatureTracking(
		const cv::Mat&,
		const cv::Mat&,
		std::vector<cv::Point2f>&,
		std::vector<cv::Point2f>&,
		std::vector<uchar>&
	);

	void vecPoint2fTovecKeyPoint(
		const std::vector<cv::Point2f>&,
		std::vector<cv::KeyPoint>&
	);

	void drawMatchesUsingPoint2f(
		const cv::Mat&,
		const std::vector<cv::Point2f>&,
		const cv::Mat&,
		const std::vector<cv::Point2f>&,
		cv::Mat&,
		int
	);
}

void vecPoint2fTovecKeyPoint();

void feature_utils::ORBFeatureDetection(const cv::Mat &image,
	std::vector<cv::Point2f> &points) {
	std::vector<cv::KeyPoint> keypoints;
	cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
	detector->detect(image, keypoints);
	// for (size_t i = 0; i < keypoints.size(); i++) {
	// 	points.push_back(keypoints[i].pt);
	// }
	cv::KeyPoint::convert(keypoints, points, std::vector<int>());
}


void feature_utils::FASTFeatureDetection(const cv::Mat &image,
		std::vector<cv::Point2f> &points) {
	std::vector<cv::KeyPoint> keypoints;
	int fast_threshold = 20;
	bool nonmaxSuppression = true;
	// cv::FAST(image, keypoints, fast_threshold, nonmaxSuppression);
	cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create(fast_threshold, nonmaxSuppression);
	detector->detect(image, keypoints);
	cv::KeyPoint::convert(keypoints, points, std::vector<int>());
}

void feature_utils::KLTFeatureTracking(const cv::Mat &image1, const cv::Mat &image2, std::vector<cv::Point2f> &points1, std::vector<cv::Point2f> &points2, std::vector<uchar> & status) {
	/*
	Feature matching based on KLT.
	@Args:
		image1: first image
		image2: second image
		points1: keypoints in first image (known)
		points2: keypoints in second image (unknown)
		status: status of keypoints (unknown)
	*/
	std::vector<float> err;
	cv::Size winSize(21, 21);
	cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
	cv::calcOpticalFlowPyrLK(image1, image2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);
	// Remove points that are outside of the image
	int num_deleted = 0;
	for (size_t i = 0; i < status.size(); i++) {
		cv::Point2f pt = points2[i - num_deleted];
		if (pt.x < 0 || pt.y < 0 || pt.x > image2.cols || pt.y > image2.rows)
			status[i] = 0;

		if (status[i] == 0){
			points2.erase(points2.begin() + i - num_deleted);
			points1.erase(points1.begin() + i - num_deleted);
			num_deleted += 1;
		}
	}

	// TODO: Remove points with large error
}

void feature_utils::vecPoint2fTovecKeyPoint(
	const std::vector<cv::Point2f>& points,
	std::vector<cv::KeyPoint>& keypoints){
	/*
	Convert vector of points to vector of keypoints.
	*/
	std::cout << "Converting vector of points to vector of keypoints..." << std::endl;
	for (size_t i = 0; i < points.size(); i++) {
		keypoints.push_back(cv::KeyPoint(points[i], 1.f));
	}
}

void feature_utils::drawMatchesUsingPoint2f(
	const cv::Mat& image1,
	const std::vector<cv::Point2f>& points1,
	const cv::Mat& image2,
	const std::vector<cv::Point2f>& points2,
	cv::Mat& img_matches,
	int max_num_matches = -1
	){
	/*
	Draw matches between two images using vector of Point2f instead of cv::KeyPoint
	*/
	cv::hconcat(image1, image2, img_matches);
	if (max_num_matches == -1)
		max_num_matches = int(points1.size());
	int skip_duration = points1.size()/max_num_matches;

	for (size_t i = 0; i < points1.size(); i++) {
		if (i % skip_duration)
			continue;

		cv::Point2f pt1 = points1[i];
		cv::Point2f pt2 = points2[i];
		pt2.x += image1.cols;
		pt2.y += 0;
		cv::Scalar color(
			(double) std::rand()/RAND_MAX*255,
			(double) std::rand()/RAND_MAX*255,
			(double) std::rand()/RAND_MAX*255
		);
		cv::circle(img_matches, pt1, 3, color, -1, 8);
		cv::circle(img_matches, pt2, 3, color, -1, 8);
		cv::line(img_matches, pt1, pt2, color);
	}
}
