#pragma once
#include "Vehicle.h"
#include <vector>

class BGS
{
private:
    std::pair<cv::Point, cv::Point> p_pLine;
    cv::Rect rRect;
	cv::Ptr<cv::BackgroundSubtractorMOG2> pMOG2;
    cv::Mat mColorFrame, mColorFrame1, mMask, mMaskG, ret[2];
    cv::Mat se1, se2;
	std::vector<cv::Vec4i> hierarchy;
    std::vector<Vehicle> vrPrevVehicles;
    std::vector<Vehicle> measuredVehiclesVehicles;


public:
    BGS() = default;
    BGS(cv::Rect, int history, float varThreshold, int iDetectLineX1, int iDetectLineX2, int iDetectLineY1);
    ~BGS() = default;

	double square(int a);
	cv::Point3d construct_box(cv::Rect r0, double angle, std::vector<cv::Point_<int> > ict,  bool draw);
    cv::Point get_lowpoint(cv::Rect r0, double angle, std::vector<cv::Point_<int> > ict,  bool draw);

private:
	void Refactor(cv::Mat &mArg);
    void splitContour(cv::Point const& p1, cv::Point const& p2, std::vector<cv::Point> const& originalContour, std::vector<std::vector<cv::Point>>& newContours);

public:
    cv::Mat* drawSquare(cv::Mat const& mColorFrameArg);
    void printVehicleInfo(cv::Point3d coeffs);
};

