#include "MarkerSensor.h"

//#include "utils.h"
Mat MarkSensor::img_show, MarkSensor::ROI_bgr, MarkSensor::coordinate,MarkSensor::NUM_bgr;
using namespace cv;
using namespace std;

Point lasttgr(30000,30000);
float spd_x,spd_y;
int fpd=30000;//frame per detected
int tracking_count;
int top_lostcount=0;
int filter_count=0;
int top_detectcount=0;
int last_pix_x=0;
int last_pix_y=0;

Point lastTopCenter=Point();
Point lastTopCenter2=Point();
//deque<Mat> imgs;

int realLeftMost=30000;
int realRightMost;

int Marker::ComputeKeyPoints()
{
  int is_dir0_down=(LEDs[0].dir.dot(Point2f(0,1)))>0?1:-1;
  int is_dir1_down=(LEDs[1].dir.dot(Point2f(0,1)))>0?1:-1;
  kpts[0]=LEDs[0].center -is_dir0_down* LEDs[0].dir*LEDs[0].width*0.5f;
  kpts[2]=LEDs[0].center + is_dir0_down*LEDs[0].dir*LEDs[0].width*0.5f;
  kpts[1]=LEDs[1].center - is_dir1_down*LEDs[1].dir*LEDs[1].width*0.5f;
  kpts[3]=LEDs[1].center + is_dir1_down*LEDs[1].dir*LEDs[1].width*0.5f;

  return 0;
}

float MarkSensor::calcDepth(Marker &res_marker)
{
    // if(abs(res_marker.bbox.x-340)>100)
    // {
    //     tracking_count=0;
    //     return 0;
    // }
    // else
    //     tracking_count++;
    // if(tracking_count>3)
    // {
        vector<Point2f> distortedPoints(res_marker.kpts,res_marker.kpts+4);
        vector<Point2f> UndistortedPoints(4);
        undistortPoints(distortedPoints, UndistortedPoints, cameraMatrix, distCoeffs, cv::noArray(), OptcameraMatrix);
        //undistortPoints(distortedPoints, UndistortedPoints, cameraMatrix, distCoeffs);
        //float height=(abs(UndistortedPoints[2].y-UndistortedPoints[0].y)+abs(UndistortedPoints[3].y-UndistortedPoints[1].y))/2;
        float height=(abs(distortedPoints[2].y-distortedPoints[0].y)+abs(distortedPoints[3].y-distortedPoints[1].y))/2;
        res_marker.depth=mp.LED_realheight*cp.fy/height;
    // }
  return res_marker.depth;
}

float MarkSensor::calcDepth(RotRect &res_LED)
{
    // if(abs(res_marker.bbox.x-340)>100)
    // {
    //     tracking_count=0;
    //     return 0;
    // }
    // else
    //     tracking_count++;
    // if(tracking_count>3)
    // {
  vector<Point2f> distortedPoints(2);
  int is_dir0_down=(res_LED.dir.dot(Point2f(0,1)))>0?1:-1;
  distortedPoints[0]=res_LED.center -is_dir0_down* res_LED.dir*res_LED.width*0.5f;
  distortedPoints[1]=res_LED.center + is_dir0_down*res_LED.dir*res_LED.width*0.5f; 
  float height=abs(distortedPoints[1].y-distortedPoints[0].y);
  float real_height=mp.LED_realheight*cp.fy/height;
    // }
  return real_height;
}

///
/// \brief MarkSensor::MarkSensor7
/// structor, initialize some varibles
/// \param ap_
/// \param cp_
/// \param mp_
///
MarkSensor::MarkSensor(AlgoriParam &ap_,CamParams &cp_,MarkerParams &mp_):
  ap(ap_),cp(cp_),mp(mp_)
{
  cameraMatrix = (cv::Mat_<double>(3,3) << cp.fx, 0, cp.cx, 0, cp.fy, cp.cy, 0, 0, 1);
  distCoeffs = (Mat_<double>(1,4) <<cp.distcoef1, cp.distcoef2, 0, 0);
  OptcameraMatrix=getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, Size(640,480),1);
  jump_filter=new FilterOutStep;

  //init real armor size, for solvePnP
  big_armor={Point3d(-0.0675,0.03,0),Point3d(0.0675,0.03,0),Point3d(0.0675,-0.03,0),Point3d(-0.0675,-0.03,0)};
  small_armor={Point3d(-0.0675,0.03,0),Point3d(0.0675,0.03,0),Point3d(0.0675,-0.03,0),Point3d(-0.0675,-0.03,0)};
}

///
/// \brief MarkSensor::bgr2binary change a color img to grayscale
/// \param srcImg
/// \param img_out
/// \param method we have 2 options to do binary
/// \return
///
int MarkSensor::bgr2binary(Mat &srcImg, Mat &img_out,int method)
{
  if (srcImg.empty())
    return -1;
  if(method==1) {
      //method 1: split channels and substract
      vector <Mat> imgChannels;
      split(srcImg, imgChannels);
      Mat red_channel = imgChannels.at(2);
      Mat blue_channel = imgChannels.at(0);
      Mat mid_chn_img;
      if (ap.is_red) {
          mid_chn_img = red_channel - blue_channel;

      } else {
          mid_chn_img = blue_channel - red_channel;
      }
      threshold(mid_chn_img, img_out, 60, 255, THRESH_BINARY);
  }
  else if(method==2)
  {
    //method 2: 3 channel threthold
    cv::inRange(srcImg,cv::Scalar(ap.ch1_min,ap.ch2_min,ap.ch3_min),
                cv::Scalar(ap.ch1_max,ap.ch2_max,ap.ch3_max),img_out);

  }else
    return -1;
  return 0;
}

///
/// \brief MarkSensor::PCALEDStrip
/// convert contours to LED struct
/// \param contour
/// \param LED
/// \return 0 if success
///
int MarkSensor::PCALEDStrip(vector<cv::Point> &contour, RotRect &LED)
{
  int sz = static_cast<int>(contour.size());
  cv::Mat data_pts(sz, 2, CV_64FC1);
  double* _data_pts = (double*)data_pts.data;
  for (int i = 0; i < data_pts.rows; ++i, _data_pts += 2) {
    _data_pts[0] = contour[i].x;
    _data_pts[1] = contour[i].y;
  }
  cv::PCA pca_analysis(data_pts, cv::Mat(), PCA::DATA_AS_ROW);
  LED.center.x = static_cast<float>(pca_analysis.mean.at<double>(0, 0));
  LED.center.y = static_cast<float>(pca_analysis.mean.at<double>(0, 1));
  cv::Point2f dir1, dir2;
  Mat eig = pca_analysis.eigenvectors;
  dir1.x = static_cast<float>(pca_analysis.eigenvectors.at<double>(0, 0));
  dir1.y = static_cast<float>(pca_analysis.eigenvectors.at<double>(0, 1));
  dir2.x = static_cast<float>(pca_analysis.eigenvectors.at<double>(1, 0));
  dir2.y = static_cast<float>(pca_analysis.eigenvectors.at<double>(1, 1));

  dir1 = dir1 * (1 / cv::norm(dir1));// norm(dir1)=1, so make no sense
  dir2 = dir2 * (1 / cv::norm(dir2));

  LED.dir = dir1;
  LED.width = ComputeLengthAlongDir(contour, dir1);
  LED.height = ComputeLengthAlongDir(contour, dir2);

  return 0;

}

///
/// \brief MarkSensor::ComputeLengthAlongDir
/// length of led
/// \param contour
/// \param dir
/// direction of led
/// \return
///
float MarkSensor::ComputeLengthAlongDir(vector<cv::Point> &contour, cv::Point2f &dir)
{
  float max_range = -999999;
  float min_range = 999999;
  for (auto & pt : contour) {
    float x = pt.x*dir.x + pt.y*dir.y;//dot(x,dir)=x*y*cos(theta), project pix on dir
    if (x < min_range) min_range = x;
    if (x > max_range) max_range = x;
  }
  return (max_range - min_range);
}

///
/// \brief MarkSensor::paraDistance
/// calculate distance between 2 parallel lines
/// \param LED1
/// \param LED2
/// \return
///
int MarkSensor::paraDistance(RotRect &LED1, RotRect &LED2)
{
  float distance = 0;
  float tgt_theta = LED1.dir.y / LED1.dir.x;
  float theta = atan(tgt_theta);
  float cx2_para = (LED1.center.y - LED2.center.y)/ tgt_theta + LED2.center.x;
  distance = fabs((LED1.center.x - cx2_para)*sin(theta));
  return distance;
}

///
/// \brief MarkSensor::tgt_Detect
/// when multiple armors detected, select which to shoot.
/// detecting is different from just tracking.
/// \param markers
/// \return
///
int MarkSensor::tgt_selector(vector<Marker> &markers)
{
  int res_idx=0;
  float minDist = 9999;
  for (int i = 0; i < markers.size(); i++) 
  {
    //calc some important params
    markers[i].ComputeKeyPoints();
    markers[i].ComputeBBox();
    //decide marker type
    float wid_div_height=markers[i].bbox.width/markers[i].bbox.height;
    if(wid_div_height>3.5)
    {
      markers[i].armor_type=Marker::BIG;
    }else
    {
      markers[i].armor_type=Marker::SMALL;
    }

    //first decision param: dist to principle point. 400->0.2,0-->1
    Point2f camera_c(cp.cx, cp.cy);
    Point2f marker_c((markers[i].LEDs[0].center + markers[i].LEDs[1].center)*0.5);
    float dist2c = norm(camera_c - marker_c);

    float decide_p1=MAX(1-0.002*dist2c,0);

    if(status!=STATUS_DETECTING)
    {
      markers[i].decision_points=decide_p1;
    }
    else
    {
      // second param: area of marker
      int area= markers[i].bbox.area();
      float decide_p2=MIN(0.001*area+0.1, 1);

      // third param: leaky angle of armor
      float leaky_angle=MIN(markers[i].LEDs[0].width/markers[i].LEDs[1].width,markers[i].LEDs[1].width/markers[i].LEDs[0].width);
      float decide_p3;
      if(markers[i].armor_type==Marker::SMALL)
      {
        decide_p3=1.333*leaky_angle-0.333;  //1-->1, 0.25-->0
      }else
      {
        decide_p3=1.5*leaky_angle-0.5;      //1--->1, 0.5--->0
      }

      markers[i].decision_points=0.5*decide_p1+decide_p2+decide_p3;

    }
    //draw all the markers

    if (mp.ifShow&&status==STATUS_DETECTING)

      rectangle(img_show, markers[i].bbox, Scalar(0, 0, 255),2);

  }
  float max_points=0;
  for  (int j = 0; j < markers.size(); j++) {
    if (markers[j].decision_points > max_points)
    {
      max_points = markers[j].decision_points;
      res_idx = j;
    }

  }
  if (mp.ifShow&&status==STATUS_DETECTING)
    rectangle(img_show, markers[res_idx].bbox, Scalar(0, 128, 128),2);

  return res_idx;

}

///
/// \brief MarkSensor::GetLEDStrip
/// find LED Strips in roi_mask.
/// \param roi_mask
/// \param LEDs
/// \return a vector contain LEDs 
///
 int  MarkSensor::GetLEDStrip(cv::Mat &roi_mask, vector <RotRect >& LEDs)
 {
    vector<vector<Point>> tmp_countours;
    vector<vector<Point>*> pContours;
    // 3 rad
    findContours(roi_mask, tmp_countours,RETR_EXTERNAL, CHAIN_APPROX_NONE);
    for (auto &contour : tmp_countours)
    {
        int cont_sz = static_cast<int>(contour.size());

        if (cont_sz >= mp.contours_lengtch1_min && cont_sz <= mp.contours_lengtch1_max)
            pContours.push_back(&contour);
    }

  //PCA
    for (auto & pContour : pContours)
    {
        RotRect LED;
        if (PCALEDStrip(*pContour, LED) == STATUS_SUCCESS) 
        {
            /// check ratio and length
            if (LED.width < mp.LED_widtch1_min || LED.width > mp.LED_widtch1_max) 
                continue;

            //ADD MORE CONSTRAINTS!!
            if (fabs(LED.dir.dot(cv::Point2f(1, 0))) >mp.cos_marker_direction_radian)// degree
            continue;

            float ratio = LED.width / LED.height;   //>1
            if (ratio < mp.LED_ratio_min || ratio > mp.LED_ratio_max) 
                continue;

            LEDs.push_back(LED);
        }
    }
 }

///
/// \brief MarkSensor::GetLEDMarker
/// find res_marker in roi_mask.
/// \param roi_mask
/// \param res_marker
/// \return
///
int MarkSensor::GetLEDMarker(cv::Mat &roi_mask, Marker &res_marker)
{
    vector <RotRect > LEDs;
   GetLEDStrip(roi_mask,LEDs);

  if (LEDs.size() < 2) 
  {
        printf("LED num < 2 ! \n");
        return -1;
  }
  
  /// search marker
  vector<Marker> markers;

  int LED_sz = LEDs.size();
  vector<bool> matched(LED_sz, false);
  for (size_t i = 0; i < LED_sz; ++i) {
    if (matched[i]) continue;
    for (size_t j = i + 1; j < LED_sz; ++j) {
      if (matched[j]) continue;
      cv::Point2f c2c = LEDs[i].center - LEDs[j].center;//centre to centre
      float para_dist = paraDistance(LEDs[i], LEDs[j]);
      /// check width difference
      float max_width = max(LEDs[i].width, LEDs[j].width);
      float led_diff = fabs(LEDs[i].width - LEDs[j].width) / max_width;

      if (led_diff > mp.LED_diff) {
        //printf("LED difference not satisfied !\n");  491,408,514,510// 56,170 ,91,175
        continue;
      }
      //check distance
      float distance = norm(c2c);

      if (distance > mp.marker_size_max || distance < mp.marker_size_min)
      {
        //printf("LED distance not satisfied !\n");
        continue;
      }
      //check parallel
      if (fabs(LEDs[i].dir.dot(LEDs[j].dir)) < mp.cos_marker_parallel_radian)
      {
        //printf("LED parallel not satisfied !\n");
        continue;
      }
      /// check direction
      cv::Point2f direction=c2c/distance;
      if (fabs(direction.dot(cv::Point2f(1, 0))) < mp.cos_marker_direction_radian) {
        //printf("Marker direction not satisfied !\n");
        continue;
      }
      // check hori distance
      float distance_hori = para_dist;
      if (distance_hori > mp.marker_size_max || distance_hori < mp.marker_size_min)
      {
        //printf("LED horizontal not satisfied !\n");
        continue;
      }

      /// build marker
      Marker tmp_marker;
      float marker_width = distance;
      float marker_height = (LEDs[i].width + LEDs[j].width)*0.5f;
      /// check marker width/height ratio
      float marker_size_ratio = marker_width / marker_height;
      if (marker_size_ratio > mp.marker_ratio_max || marker_size_ratio < mp.marker_ratio_min) {
        //printf("Marker size ratio not satisfied !\n");
        continue;
      }
      matched[i] = matched[j] = true;
      if (c2c.x > 0) {
        tmp_marker.LEDs[0] = LEDs[j];//0 on the left
        tmp_marker.LEDs[1] = LEDs[i];
      }
      else {
        tmp_marker.LEDs[0] = LEDs[i];
        tmp_marker.LEDs[1] = LEDs[j];
      }
      markers.push_back(tmp_marker);

    }

  }

  if (markers.empty())
  {
    return -1;
  }
  // decide which marker to shoot
  int res_idx = tgt_selector(markers);
  res_marker = markers[res_idx];
  res_marker.old_depth=0;
  res_marker.depth=0;


  return 0;
}
///
/// \brief MarkSensor::DetectLEDMarker
/// detect all armors and choose best tgt_pos.status=0;
/// \param img
/// \param res_marker
/// \return
///
int MarkSensor::DetectLEDMarker(const Mat &img, Marker &res_marker)
{
  //cvtColor(img_bgr, img_hsv, CV_BGR2HSV);
  img.copyTo(img_hsv);
  /*actually we use bgr*/
  
  //llj:here, bgr2binary
  bgr2binary(img_hsv,led_mask,1);


    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(led_mask, led_mask, MORPH_CLOSE, element, Point(-1, -1), 1);
    bool is_detected=GetLEDMarker(led_mask,res_marker);

    // if(!is_detected)
    // {
    //     dbg_save(img,ap.dbg_path,status);
    // }
    return is_detected;
}
///
/// \brief MarkSensor::TrackLEDMarker
/// track target in small img patch
/// \param img
/// \param res_marker
/// \return
///
int MarkSensor::TrackLEDMarker(const Mat &img, Marker &res_marker)
{
  Rect &box = res_marker.bbox;

  float left = box.x - status*box.width;
  float right = box.x + box.width * (status+1);
  float top = box.y - status*box.height;
  float bot = box.y + box.height * (status+1);
  left = left < 0 ? 0 : left;
  right = right >= img.cols ? img.cols : right;
  top = top < 0 ? 0 : top;
  bot = bot >= img.rows ? img.rows : bot;
  Rect ROI(left, top, (right - left), (bot - top));
  /// Get Mask
  ROI_bgr = img(ROI).clone();
  NUM_bgr = img(ROI).clone();
  cv::Mat ROI_led_mask;
  ///check if empty
  if (ROI_bgr.empty())
  {
    printf("no marker for tracking!!");
    status = STATUS_DETECTING;
    marker=Marker();
    return -1;
  }
  bgr2binary(ROI_bgr,ROI_led_mask,1);

  /// Get Marker
  if (GetLEDMarker(ROI_led_mask, res_marker) != STATUS_SUCCESS) {
    printf("Get no marker!\n");
//    dbg_save(ROI_bgr,ap.dbg_path,status);
    return -1;
  }
// add coordinate bias
  res_marker.LEDs[0].center.x += ROI.x;
  res_marker.LEDs[0].center.y += ROI.y;
  res_marker.LEDs[1].center.x += ROI.x;
  res_marker.LEDs[1].center.y += ROI.y;
  ///draw the best marker

  res_marker.ComputeKeyPoints();
  res_marker.ComputeBBox();
  if (mp.ifShow)
  {
    //    img_out=img_show(res_marker.bbox);
    rectangle(img_show, res_marker.bbox, Scalar(0, 255, 0), 1);
  }

  return 0;

}

///
/// \brief MarkSensor::ProcessFrameLEDXYZ
/// \param img: input src
/// \param angX: yaw error
/// \param angY: pitch error
/// \param Z: target depth
/// \param pix_x: x error
/// \param pix_y: y error
/// \return :-1  fail
///
///
int MarkSensor::ProcessFrameLEDXYZ(const Mat &img,  float &Z, int &pix_x,int &pix_y)
{
  img.copyTo(img_show);
  if (status == STATUS_DETECTING)
   {
        if (DetectLEDMarker(img, marker) == STATUS_SUCCESS) 
        {
            status = STATUS_TRACKING;
        }
        else 
        {
            printf("Detect No target!\n");
            return -1;
        }
   }
  else if(status==STATUS_TRACKING)
  {
        if (TrackLEDMarker(img, marker) == STATUS_SUCCESS) 
        {
          printf("Track Success!\n");
        }
        else 
        {
            status = STATUS_TRACKLOST0;
            printf("Track No target!\n");
            track_fail_cnt[0]=0;
            //      return -1;
        }
    }
  else if(status==STATUS_TRACKLOST0)
  {
        if (TrackLEDMarker(img, marker) == STATUS_SUCCESS) 
        {
            printf("Track 0 Success!\n");
            status = STATUS_TRACKING;
        }
        else 
        {
            printf("Track 0 No target!\n");
            track_fail_cnt[0]++;
            if(track_fail_cnt[0]>10)   // you can modify this constant
            {
                status=STATUS_TRACKLOST1;
                printf("enlarge ROI!");
                track_fail_cnt[0]=0;
                track_fail_cnt[1]=0;
            }
            //return -1;
        }
  }
  else if(status==STATUS_TRACKLOST1)
  {
        if (TrackLEDMarker(img, marker) == STATUS_SUCCESS) 
        {
            printf("Track 0 Success!\n");
            status = STATUS_TRACKING;
        }
        else 
        {
            printf("Track 1 No target!\n");
            track_fail_cnt[1]++;
             if(track_fail_cnt[1]>20)// you can modify this constant
            {
                status=STATUS_TRACKLOST2;
                printf("ROI enlarge again!");
                track_fail_cnt[1]=0;
                track_fail_cnt[2]=0;
            }
            // marker=Marker();
            //return -1;
        }
    }
  else if(status==STATUS_TRACKLOST2)
  {
        if (TrackLEDMarker(img, marker) == STATUS_SUCCESS) 
        {
            printf("Track 0 Success!\n");
            status = STATUS_TRACKING;
        }
        else 
        {
            printf("Track 0 No target!\n");
            track_fail_cnt[2]++;
            if(track_fail_cnt[2]>20)
            {
                status=STATUS_DETECTING;
                printf("failed to find marker in ROI");
                track_fail_cnt[2]=0;
                marker=Marker();
            }
            return -1;
        }
  }

  if(mp.if_get_angle)
  {
    //将像素坐标转为相机坐标系下的真实坐标
    target=(marker.LEDs[0].center + marker.LEDs[1].center)*0.5f;

    if(mp.ifShow)
    {
    if(status==STATUS_TRACKING)
        circle(img_show,target,4,Scalar(20,20,255),3);
    else
        circle(img_show,target,4,Scalar(255,20,20),3);
    }

    //slovePnP
    marker.ComputeKeyPoints();
    img_points={marker.kpts[2],marker.kpts[3],marker.kpts[1],marker.kpts[0]};
    circle(img_show,marker.kpts[2],4,Scalar(255,20,20),3);
    circle(img_show,marker.kpts[3],4,Scalar(30,200,20),3);
    circle(img_show,marker.kpts[1],4,Scalar(30,20,220),3);
    circle(img_show,marker.kpts[0],4,Scalar(255,255,255),3);
    if(marker.armor_type==Marker::BIG)
    {
      solvePnP(big_armor,img_points,cameraMatrix,distCoeffs,rvec,tvec,false,SOLVEPNP_ITERATIVE);
    }
    else
    {
      solvePnP(small_armor,img_points,cameraMatrix,distCoeffs,rvec,tvec,false,SOLVEPNP_ITERATIVE);
    }

    //Mat Pcam=rvec.t()*tvec;

    //旋转向量化为旋转矩阵
    Rodrigues(rvec,rvec);

    Mat ArmorCord=tvec;

    angle_yaw=ArmorCord.at<double>(0,0);
    cout<<"X ="<<angle_yaw<<endl;
    angle_pitch=ArmorCord.at<double>(0,1);
    cout<<"Y ="<<angle_pitch<<endl;

    coordinate=Mat::zeros(Size(500,500),CV_8UC3);
    line(coordinate,Point(250,0),Point(250,500),Scalar(255,255,255),1);
    line(coordinate,Point(0,250),Point(500,250),Scalar(255,255,255),1);
    int x4show, y4show;

    //如果点过于靠近坐标轴/没有点出现，则增加/减小scale_factor
    int scale_factor=25;
    x4show=angle_yaw*scale_factor*5+250;
    y4show=250-angle_pitch*scale_factor*5;

    if(ap.is_red==false)
      circle(coordinate,Point(x4show,y4show),2,Scalar(255,114,5),2);
    else
      circle(coordinate,Point(x4show,y4show),2,Scalar(5,114,255),2);
    

    /*
    //calc gimbal angle(暂不用)
    angle_yaw=atan2(rvec.ptr<double>(1)[0], rvec.ptr<double>(0)[0]) / CV_PI * 180;
    //check which is right
    angle_pitch=atan2(rvec.ptr<double>(2)[1], rvec.ptr<double>(2)[2]) / CV_PI * 180;
    angle_pitch = atan2(-1 * rvec.ptr<double>(2)[0], sqrt(rvec.ptr<double>(2)[1]*rvec.ptr<double>(2)[1] +rvec.ptr<double>(2)[2]*rvec.ptr<double>(2)[2])) / CV_PI * 180;

    //filter for blink
    if(status==STATUS_TRACKING || status==STATUS_DETECTING)
    {
      last_angle_yaw=angle_yaw;
      last_angle_pitch=angle_pitch;
    }
    else
    {
      angle_yaw=last_angle_yaw;
      angle_pitch=last_angle_pitch;
    }
    */

    if(mp.if_calc_depth)
    {
      //float depth=calcDepth(marker);
      //Z = depth;
      Z=ArmorCord.at<double>(0,2);
      cout<<"Z ="<<Z<<endl;
    }

    pix_x=angle_yaw*1000;
    pix_y=angle_pitch*1000;

    cout<<"angle_yaw"<<pix_x<<endl;
    cout<<"angle_pitch"<<pix_y<<endl;
     cout<<"angle_pitch_"<<Z<<endl;
    return 0;
  }


  ///update pix position"angle_yaw"
  ///  this filter is important, coz the leds on armors will blink if pressed by bullet
  //need to rewrite for solvePnP
  /*
  if(status==STATUS_TRACKING)
  {
    new_target = (marker.LEDs[0].center + marker.LEDs[1].center)*0.5f;
    if(lasttgr.x-new_target.x>100)
        lasttgr=new_target;
    spd_x=(new_target.x-lasttgr.x)/fpd;
    spd_y=(new_target.y-lasttgr.y)/fpd;
    fpd=1;
    jump_filter->setInitPix(new_target);
    target=new_target;
    lasttgr=new_target;
  }
  else
  {
    fpd++;
    new_target =Point(30000,30000);
    target=jump_filter->Filter(new_target,10000,20,spd_x,spd_y);
  }

if(mp.if_calc_depth)
  {
    float depth=calcDepth(marker);
    Z = depth;
  }

  if(mp.if_predict && mp.if_calc_depth)
  {
    int FT_frame=Z/15*200;
    pix_x = target.x+spd_x*FT_frame;
    pix_y = target.y+spd_y*FT_frame;
  }
  else
  {
    pix_x = target.x;
    pix_y = target.y;
  }

  if(pix_x==30000&&pix_y==30000)
  {
    return -1;
  }

  */


 //only for new infantry
  // if(cp.idx==2)
  // {
  //   if(status!=STATUS_TRACKING)
  //   {
  //     last_pix_x=0;
  //     last_pix_y=0;
  //   }
  //     if(last_pix_x!=0 && last_pix_y!=0)
  //     {
  //       pix_x=last_pix_x/5+pix_x*4/5;
  //       pix_y=last_pix_y/5+pix_y*4/5;
  //     }
  //     last_pix_x=pix_x;
  //     last_pix_y=pix_y;
  // }

  if(mp.ifShow)
  {
    if(status==STATUS_TRACKING)
        circle(img_show,target,4,Scalar(20,20,255),3);
    else
        circle(img_show,target,4,Scalar(255,20,20),3);
  }

  //tell if it is time to shoot
  float extend_yaw; //  you will modify this var on different machines
  if(marker.armor_type==Marker::BIG)
  {
      extend_yaw=0.6;
  }else
  {
      extend_yaw=1.2;
  }
  Point img_center=Point(0.5*img.cols,0.5*img.rows+ap.pitch_bias);
  if(img_center.y>(marker.kpts[0].y-marker.bbox.height)&&img_center.y<(marker.kpts[2].y+marker.bbox.height)&&
     img_center.x>(marker.LEDs[0].center.x-extend_yaw*marker.bbox.width)&&img_center.x<(marker.LEDs[1].center.x+extend_yaw*marker.bbox.width))
  {
    center_in_rect=2;
    ROS_WARN("shoot the target!");
  }else
  {
    center_in_rect=1;
  }

  

  return 0;
}

///
/// \brief MarkSensor::DetectTopCenter
/// \param img: input src
/// \param res_point: the point gimbal should point to
/// \return :-1  fail
///
///
int MarkSensor::DetectTopCenter(const Mat &img, Rect& ROI, Point2i& res_point, float& depth)
{
    cout<<"enter detecting!!!"<<endl;
    img.copyTo(img_hsv);
  
    //llj:here, bgr2binary
    bgr2binary(img_hsv,led_mask,1);  /*actually we use bgr*/


    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(led_mask, led_mask, MORPH_CLOSE, element, Point(-1, -1), 1);

    Marker top_marker;
    int is_detected=GetLEDMarker(led_mask,top_marker);
    if(is_detected==-1)
        return -1;

    top_marker.ComputeKeyPoints();
    top_marker.ComputeBBox();
    res_point.x=top_marker.bbox.x+top_marker.bbox.width/2;
    res_point.y=top_marker.bbox.y+top_marker.bbox.height/2;

    //get ROI for tracking
    float left = res_point.x-3*top_marker.bbox.width;
    float right = res_point.x + 3*top_marker.bbox.width;;
    float top = res_point.y-2*top_marker.bbox.height;
    float bot = res_point.y + 2*top_marker.bbox.height;
    left = left < 0 ? 0 : left;
    right = right >= img.cols ? img.cols : right;
    top = top < 0 ? 0 : top;
    bot = bot >= img.rows ? img.rows : bot;
    Rect ROI_(left, top, (right - left), (bot - top));
    
    ROI=ROI_;
    

    if(mp.ifShow)
        circle(img_show,res_point,4,Scalar(20,20,255),3);

    if(mp.if_calc_depth)
    {
      depth=calcDepth(top_marker);
    }

    return 0;
}

///
/// \brief MarkSensor::TrackTopCenter
/// \param img: input src
/// \param res_point: the point gimbal should point to
/// \return :-1  fail
///
///
int MarkSensor::TrackTopCenter(const Mat &img, Rect& ROI, Point2i& res_point, float& depth) 
{
    ROS_WARN("enter tracking!");


    //get ROI
    float left = res_point.x - ROI.width*(top_lostcount/3+1);
    float right = res_point.x + ROI.width*(top_lostcount/3+1);
    float top = res_point.y - ROI.height*(top_lostcount/3+1);
    float bot = res_point.y + ROI.height*(top_lostcount/3+1);
    left = left < 0 ? 0 : left;
    right = right >= img.cols ? img.cols : right;
    top = top < 0 ? 0 : top;
    bot = bot >= img.rows ? img.rows : bot;

    Rect ROI_enlarged(left, top, (right - left), (bot - top));
    ROI_bgr = img(ROI_enlarged).clone();
    //check if empty
    if (ROI_bgr.empty())
    {
        printf("no marker for tracking!!");
        status = STATUS_TOP_REDETECT;
        res_point=Point();
        return -1;
    }

    //BGR to binary
    cv::Mat ROI_led_mask;
    bgr2binary(ROI_bgr,ROI_led_mask,1);

    //get LED strip
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(led_mask, led_mask, MORPH_CLOSE, element, Point(-1, -1), 1);
    vector <RotRect> LEDs;
    GetLEDStrip(ROI_led_mask,LEDs);

    //get x coordinate
    if (LEDs.size() <1) 
    {
        status=STATUS_TOP_REDETECT;
        cout<<"no LEDs"<<endl;
        return -1;
    }

    if (LEDs.size() < 2) 
    {
        //status=STATUS_TOP_REDETECT;
        printf("LED num < 2 ! \n");
        res_point=lastTopCenter;//discuss with control(give last pose or zero)
        if(mp.ifShow)
        circle(img_show,res_point,4,Scalar(255,255,20),3);
        return -1;
    }

    Point2i left_most,right_most;
    left_most=Point(30000,30000);

    for(auto LED:LEDs)
    {
        if(LED.center.x<left_most.x)
        {
            left_most=LED.center;
            //continue;
        }
        if(LED.center.x>right_most.x)
            right_most=LED.center;
    }

    if(left_most.x<realLeftMost || right_most.x>realRightMost)
    {
        realLeftMost=left_most.x;
        realRightMost=right_most.x;
    }

    Point origin(left,top);
    circle(img_show,Point(realLeftMost,left_most.y)+origin,3,Scalar(255,20,255),3);
    circle(img_show,Point(realRightMost,right_most.y)+origin,3,Scalar(20,255,255),3);

    if(abs(left_most.y-right_most.y)>100 || abs(right_most.x-left_most.x)>500)//TODO:this param need to be modify
    {
        res_point=lastTopCenter;//discuss with teammate
        circle(img_show,res_point,4,Scalar(20,20,255),3);
        cout<<"LED strip not form one car!"<<endl;
        return -1;
    }

    res_point.x=(realLeftMost+realRightMost)/2;

    if(res_point.x<left_most.x || res_point.x>right_most.x)
    {
        realLeftMost=30000;
        realRightMost=0;
        res_point=lastTopCenter;
        circle(img_show,res_point,4,Scalar(20,20,255),3);
        cout<<"center not in car!!"<<endl;
        return -1;
    }

    //get y coordinate
    int min_diff_x=10000;
    for(auto LED:LEDs)
    {
        if(abs(res_point.x-LED.center.x)<min_diff_x)
        {
            min_diff_x=abs(res_point.x-LED.center.x);
            res_point.y=LED.center.y;
              if(mp.if_calc_depth)
              {
                  depth=calcDepth(LED);
              }
        }   
    }

    //add ROI bias
    res_point.x+=ROI_enlarged.x;
    res_point.y+=ROI_enlarged.y;

    //filter,avoid shaking
    if(lastTopCenter2!=Point())
    {
        res_point.x=(res_point.x+lastTopCenter.x*2+lastTopCenter2.x*2)/5;
        res_point.y=(res_point.y+lastTopCenter.y*2+lastTopCenter2.y*2)/5;
    }
    lastTopCenter=res_point;
    lastTopCenter2=lastTopCenter;

    //draw result
    if(mp.ifShow)
        circle(img_show,res_point,4,Scalar(255,20,20),3);
    return 0;


   
}
///
/// \brief MarkSensor::GetTopPos
/// \param img: input src
/// \param pix_x: x error(control yaw pos)
/// \param pix_y: y error(control pitch pos)
/// \return :-1  fail
///right_most.x
///
int MarkSensor::GetTopPos(const Mat & img, int &pix_x, int &pix_y, float& depth)
{
    img.copyTo(img_show);
    if(status==STATUS_TOP_REDETECT)
    {
        top_detectcount=0;
        top_center=Point();
        lastTopCenter=Point();
        lastTopCenter2=Point();
        top_ROI=Rect();
        realLeftMost=30000;
        realRightMost=0;
        status=STATUS_DETECTING;
    }
    if (status == STATUS_DETECTING)
   {
       //top_lostcount=0;
        if (DetectTopCenter(img, top_ROI, top_center,depth) == STATUS_SUCCESS) 
        {
            top_lostcount=0;
            status = STATUS_TRACKING;
        }
        else 
        {
            printf("Top_Detect No target!\n");
            return -1;
        }
   }
  else if(status==STATUS_TRACKING)
  {
        if (TrackTopCenter(img, top_ROI, top_center, depth)  == STATUS_SUCCESS) 
        {
            top_lostcount=0;
            //imgs.clear();  
            printf("Top_Track Success!\n");
        }
        else 
        {
            top_lostcount++;
            if(top_lostcount>15)//TODO:this param need to be modify
            {
                status = STATUS_TOP_REDETECT;
                printf("Top_Track No target!\n");
            }
        }
    }
    
    if(top_center!=Point(0,0))
    {
        pix_x=top_center.x;
        pix_y=top_center.y;
        return 1;
    }

    

    return 0;
}

