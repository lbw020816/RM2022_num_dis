#ifndef MARKERSSENSOR_H
#define MARKERSSENSOR_H
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <numeric>

//Eigen头文件必须在opencv2/core/eigen.hpp前
#include<Eigen/Core>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/calib3d/calib3d.hpp>


#include "MarkerParams.h"
#include "utils.h"
#include "tf/tf.h"
//timer
//#include "timer.h"
using namespace std;
using namespace cv;

class RotRect {
public:
	cv::Point2f center;
	cv::Point2f dir;
	float width;
	float height;

	RotRect() : width(0), height(0) {};
	RotRect(const cv::Rect & rect) :
		width(rect.width), height(rect.height)
	{
		center.x = rect.x + rect.width*0.5f;
		center.y = rect.y + rect.height*0.5f;
		dir.x = 1;
		dir.y = 0;
  }
};

class Marker 
{
public:
    Marker()
    {
    old_depth=0;
    depth=0;
    }

    enum MarkerType 
    {
    All = 0,
    SMALL= 1,
    BIG=2
    };

    int ComputeKeyPoints();

	int ComputeBBox()
	{
	float max_x = 0, max_y = 0;
    float min_x = 9999, min_y = 9999;
		for (int i = 0; i < 4; i++)
        {
			Point2f kpt = kpts[i];			// may be wrong
			if (kpt.x < min_x)
			{
				min_x = kpt.x;
			}
			if (kpt.x > max_x) {
				max_x = kpt.x;
			}
			if (kpt.y < min_y) {
				min_y = kpt.y;
			}
			if (kpt.y > max_y) {
				max_y = kpt.y;
			}
		}
		bbox.x = min_x;
		bbox.y = min_y;
		bbox.width = (max_x - min_x);
		bbox.height = (max_y - min_y);
		return 0;
	}

	int Draw(Mat & img) 
    {
		ComputeKeyPoints();
		cv::line(img, kpts[0], kpts[1], cv::Scalar(255, 0, 0), 3);
		cv::line(img, kpts[1], kpts[2], cv::Scalar(0, 255, 0), 3);
		cv::line(img, kpts[2], kpts[3], cv::Scalar(0, 0, 255), 3);
		cv::line(img, kpts[3], kpts[0], cv::Scalar(255, 255, 0), 3);
		return 0;
	}

    RotRect   LEDs[2];
    Point2f kpts[4];
    Rect bbox;
    float decision_points;
    float   old_depth, depth;
    MarkerType armor_type;
};


class MarkSensor {
public :
	enum SensorStatus 
    {
	STATUS_SUCCESS = 0,
	STATUS_TRACKING,
    STATUS_TRACKLOST0,
    STATUS_TRACKLOST1,
    STATUS_TRACKLOST2,
    STATUS_DETECTING,
    STATUS_TOP_REDETECT
	};

   enum EnemyStatus
    {
    STATIC_POS = 0,
    MOVING= 1,
    SWAGGING=2,
    ROTATING=3
    };

    MarkSensor(AlgoriParam &ap_,CamParams &cp_, MarkerParams &mp_);
	//MarkSensor(const string & calibration, const string & config, const string & cascade);
    int ProcessFrameLEDXYZ(const Mat & img, float & Z,  int &pix_x, int &pix_y);
	int DetectLEDMarker(const Mat &img, Marker &res_marker);
	int TrackLEDMarker(const Mat &img, Marker &res_marker);

    Point2i top_center;
    Rect top_ROI;
    int GetTopPos(const Mat & img, int &pix_x, int &pix_y, float& depth);
    int DetectTopCenter(const Mat &img, Rect& ROI, Point2i& res_point,  float& depth);
    int TrackTopCenter(const Mat &img, Rect& ROI, Point2i& res_point, float& depth);

	int GetLEDMarker(cv::Mat &roi_mask, Marker &res_marker);
    int GetLEDStrip(cv::Mat &roi_mask, vector <RotRect >& LEDs);
	int PCALEDStrip(vector<cv::Point> &contour, RotRect & LED);
	float ComputeLengthAlongDir(vector<cv::Point> &contour, cv::Point2f &dir);
	int paraDistance(RotRect &LED1, RotRect &LED2);
    int tgt_selector(vector<Marker> &markers);

    float calcDepth(Marker &marker);
    float calcDepth(RotRect &res_LED);
    float judge_motion();
    int bgr2binary(Mat &srcImg, Mat &img_out,int method);

    AlgoriParam ap;
    CamParams cp;
    MarkerParams mp;
    FilterOutStep *jump_filter;

    Mat cameraMatrix;
    Mat OptcameraMatrix;
    Mat distCoeffs;

    SensorStatus status= STATUS_DETECTING;
    Marker marker;
	Mat img_gray, img_bgr, img_hsv, img_h, led_mask,img_out;
    static Mat img_show, ROI_bgr, coordinate;
    Point2f new_target, target;

    
    int track_fail_cnt[3];
    tf::Transform trans;
    bool got_trans=false;

    tf::Vector3 pos_t2w;
    tf::Vector3 pos_t2c;
    //for motion analyse
    EnemyStatus enemy_stat;
    deque<Point> spd_list;

    //for calc angle
    vector<Point3d> big_armor;
    vector<Point3d> small_armor;
    vector<Point2f> img_points;
    Mat rvec,tvec;
    double angle_yaw,angle_pitch,last_angle_yaw,last_angle_pitch;

    int center_in_rect;

    ros::Time imgTime;
};



void limitRect(Rect &location, Size sz);
#endif
