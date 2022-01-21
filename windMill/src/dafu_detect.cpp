/*
windMill/dafu/PR code here
*/

#include "dafu_detect.h"


///
/// \brief Dafu_Detector::Dafu_Detector
/// structor of the class. It pass  parameters from ap and cp to those Zhangsheng defines.
/// \param _ap    algorighm parameters.
/// \param _cp    camera parameters.
///
Dafu_Detector::Dafu_Detector(AlgoriParam &_ap,CamParams &_cp ):ap(_ap),cp(_cp)
{
  Camera_fx=cp.fx;
  Camera_fy=cp.fy;
  Camera_fxy=(Camera_fx+Camera_fy)/2;
  Camera_frame_width=cp.rows;
  Camera_frame_height=cp.cols;

  size=Size(cp.rows,cp.cols);
  gray_threthold=ap.gray_threthold;
  std::cout<<"gray threthold is"<<gray_threthold<<std::endl;
}

///
/// \brief Dafu_Detector::bgr2binary
/// input color image and output binary image. It can use 2 methods to acheive that.
/// \param srcImg   input
/// \param img_out    output
/// \param method --1: split channels --2: use threthold
/// \return
///
int Dafu_Detector::bgr2binary(Mat &srcImg, Mat &img_out,int method)
{
  ROS_WARN("=====BGR to binary===========");
    if (srcImg.empty())
    return -1;
    // //method 1: split channels and substract
    vector<Mat> imgChannels;
    split(srcImg, imgChannels);
    Mat red_channel = imgChannels.at(2);
    Mat blue_channel = imgChannels.at(0);
    Mat mid_chn_img;
    if(!ap.is_red)
    {
        mid_chn_img = red_channel - blue_channel;

    }else
    {
        mid_chn_img = blue_channel-red_channel;
    }
    threshold(mid_chn_img, img_out, gray_threthold, 255, THRESH_BINARY);
  return 0;
}

///
/// \brief Dafu_Detector::myFilter   滤除少量的跳变数据，请不要将数据迭代输入进此函数
/// \param InputPixel               输入为目标的像素坐标
/// \param InterframeError          帧之间的误差大于InterframeError则认为数据发生跳变否则滤除
/// \param FilterLength             连续FilterLength帧发生跳变则认为是真实跳变
/// \return                         返回滤波后的数据
///
Point2f Dafu_Detector::myFilter(Point2f InputPixel,float InterframeError,int FilterLength )
{
  static int jump_cnt=0;
  static Point2f LastInputPixel=Point2f(320,200);
  static Point2f CurrentInputPixel=Point2f(320,200);
  static Point2f PixelRecord=Point2f(320,200);

  LastInputPixel=CurrentInputPixel;
  CurrentInputPixel=InputPixel;

  float PixelLength=GetPixelLength(LastInputPixel,CurrentInputPixel);
  cout<<PixelLength<<endl;

  PixelLength=GetPixelLength(PixelRecord,CurrentInputPixel);
  cout<<PixelLength<<"               "<<endl<<endl;


  if( PixelLength > InterframeError)
  {
      if(jump_cnt==0)
      {
        PixelRecord=LastInputPixel;
      }
      jump_cnt++;
      if(jump_cnt>=FilterLength)
      {
        jump_cnt=0;
        return CurrentInputPixel;
      }
    else
      {
        return PixelRecord;
      }
  }
  else
  {
    jump_cnt=0;
    PixelRecord=CurrentInputPixel;
    return CurrentInputPixel;
  }
}

///
/// \brief Dafu_Detector::getRotatePoint 将坐标围绕圆心旋转一定角度，用于补偿弹丸在空中飞行的时间
/// \param srcImage          源图像
/// \param Points                 旋转后的坐标将画在的图像     
/// \param rotate_center          旋转中心（大符中心）
/// \param  angle                 旋转的角度值，是经验值      
/// \return                      旋转后的坐标
///
Point2f Dafu_Detector::getRotatePoint(cv::Mat srcImage, cv::Point Points, const cv::Point rotate_center, const double angle) 
{ 
	int x1 = 0, y1 = 0;	
	int row = srcImage.rows;	
	x1 = Points.x;		
	y1 = row - Points.y;		
	int x2 = rotate_center.x;		
	int y2 = row - rotate_center.y;	
    int x,y;
    x = cvRound((x1 - x2)*cos(pi / 180.0 * angle) - (y1 - y2)*sin(pi / 180.0 * angle) + x2);		
	y = cvRound((x1 - x2)*sin(pi / 180.0 * angle) + (y1 - y2)*cos(pi / 180.0 * angle) + y2);	
	y = row - y;
	cv::Point2f dstPoints;			
	dstPoints.x = x;
	dstPoints.y = y;
	return dstPoints; 
}

///
/// \brief Dafu_Detector::predcit 将坐标围绕圆形旋转一定角度，用于补偿弹丸在空中飞行的时间
/// \param angle_degree          旋转的角度值，单位为度
/// \param frame                 旋转后的坐标将画在的图像     
/// \return                      旋转后的坐标
///
Point2f  Dafu_Detector::predcit(float angle_degree,Mat frame) //calculate  predcit
{
  float theta=angle_degree/180*3.14;
  float cos_theta = cos (theta);
  float sin_theta = sin (theta);
  Point2f tgt2center =   shootArmour - DafuCenter;
  float x =cos_theta * tgt2center.x-sin_theta * tgt2center.y;
  float y = sin_theta * tgt2center.x + cos_theta * tgt2center.y;
  Point2f pred2center = Point2f(x,y);
  Point2f pred=pred2center + DafuCenter;

  circle(frame, pred, 5, (0, 255, 255), 2);

  return pred;
}

///
/// \brief Dafu_Detector::UnlockEngGear    主函数
/// \param srcImg                     摄像机得到的原图像
/// \param is_red                     大符的颜色是否是红色
/// \param is_cw                      大符是否顺时钟旋转
/// \return                           返回传给下位机的坐标，作为云台反馈
///
Point Dafu_Detector::UnlockPR(Mat &srcImg, bool is_red,PRStatus status)
{
  bgr2binary(srcImg,threshold_frame,1);

  //连接连通域
   static Mat kernel_close = getStructuringElement(MORPH_RECT, Size(1,1), Point(-1, -1));
   morphologyEx(threshold_frame, threshold_frame, MORPH_DILATE, kernel_close);

  //去除噪点
  //static Mat kernel_open = getStructuringElement(MORPH_RECT, Size(2, 2), Point(-1, -1));
  //morphologyEx(threshold_frame, threshold_frame, MORPH_OPEN, kernel_open);

  //imshow("threshold_frame2",threshold_frame);
  DetectDafuArmor(threshold_frame, srcImg,status);
  std::cout<<"dafu tgt is"<<PredcitShootArmourCenter<<std::endl;
  if(IsDetectDafuCenter==0)
  {
    return Point(0,0);
  }
  else
  {
    return Point(100*(PredcitShootArmourCenter.x-320),100*(PredcitShootArmourCenter.y-240));
  }
}


///
/// \brief Dafu_Detector::DetectDafuArmor   检测大符的中心以及各个装甲板
/// \param grayImage  经过去噪以及二值化后的图像
/// \param dstImage   将在这幅图像上画出检测出的中心以及装甲板的中心，并圈出需要打击的装甲板
/// \param is_cw      大符是否顺时针旋转
///
void Dafu_Detector::DetectDafuArmor(Mat &grayImage, Mat &dstImage,PRStatus status)
{
    //Size size=Size(cp.rows,cp.cols);
    Mat srcImg= Mat::zeros(size, CV_8UC3);
	Mat threadImg, cutImg;
	//namedWindow("threadImg", CV_WINDOW_AUTOSIZE);
	namedWindow("circle", WINDOW_AUTOSIZE);

	if (grayImage.empty())//check if empty
	{
		cout << "no img for detect!" << endl;
		IsDetectDafuCenter=0;
        return;
	}

	//write video
	//VideoWriter writer;
	//writer.open("E:\\RM\\tx2\\2019_win_dafu\\dafu\\dafu\\video\\dafu_red_detect.avi", CV_FOURCC('M', 'J', 'P', 'G'), rate, size_cut, true);//color-true  gray-false

    //show original img
    
  //   imshow("binary image", grayImage);
	// waitKey(5);

    //find contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(grayImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
	if (contours.size() <= 0) 
    {
        cout << "can't find contours!" << endl;
		IsDetectDafuCenter=0;
        return;
    }
	//cout << "contours:" << contours.size() << "hierarchy" << hierarchy.size() << endl;

	// draw contours
	Mat drawing = Mat::zeros(grayImage.size(), CV_8UC3);
	Mat drawing_hou = Mat::zeros(grayImage.size(), CV_8UC3);
	Mat drawing_qian = Mat::zeros(grayImage.size(), CV_8UC3);
	for (int i = 0; i < contours.size(); i++)
	{
		Scalar color = Scalar(0, 0, 255);
		drawContours(drawing, contours, i, color, 1, 8, hierarchy, 0, Point());
	}
	imshow("drawing", drawing);
  waitKey(1);
		/*for (int i = 0; i < contours.size(); i++)
		{
			Scalar color = Scalar(0, 0, 255);
			drawContours(drawing_hou, contours, i, color, 2, 8, hierarchy[2], 0, Point());
		}
		imshow("drawing_hou", drawing_hou);*/
		/*for (int i = 0; i < contours.size(); i++)
		{
			Scalar color = Scalar(0, 255, 0);
			drawContours(drawing_qian, contours, i, color, 2, 8, hierarchy[3], 0, Point());
		}
		imshow("drawing_qian", drawing_qian);*/

//--------------------------------------------find armour------------------------------------------------------//
	Mat armourImg = Mat::zeros(size, CV_8UC3);
	vector<RotatedRect> box(contours.size()); //flag of every contours:whether is it an armor
	//armor feature：no child contours, but has parent contours
	for (int i = 0; i < hierarchy.size(); i++)
	{
		if (hierarchy[i].val[2] == -1 && hierarchy[i].val[3] != -1)
		{
			Scalar color = Scalar(0,0,255);
			drawContours(srcImg, contours, i, color, 1, 8, hierarchy);
			box[i].center.x = 1;
		}
		else
		{
			box[i].center.x = -1;
		}
	}

	//box the contours
	Point2f rect[4];
	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() <= 0) 
            continue;
		if (box[i].center.x != -1)
		{
			box[i] = minAreaRect(Mat(contours[i]));
			box[i].points(rect);
		}
	}

    

    //armor  feature:area in a range
	//armor  feature:height/width ratio in a range
	armor_cnt = 0;
    vector <Point2f> DetectArmourCenter(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		if (box[i].center.x != -1)
		{
			//use pixel width  and pixel height
			cout << "width:" << box[i].size.width << "  height:" << box[i].size.height << endl;	
            
            //judge by simple area
			if ((box[i].size.width * box[i].size.height>300)&& (box[i].size.width * box[i].size.height <1000)) //60 180 //100 500
			{
                //judge by height and width ratio
				if (box[i].size.width > box[i].size.height)
				{
					detect_dafu_armour_pixel_width = box[i].size.width;
					detect_dafu_armour_pixel_height = box[i].size.height;

				}
				else
				{
					detect_dafu_armour_pixel_width = box[i].size.height;
					detect_dafu_armour_pixel_height = box[i].size.width;
				}  
				if ((detect_dafu_armour_pixel_width*1.0 / detect_dafu_armour_pixel_height) < 2.5) //2.2
				{
					Scalar color = Scalar(255, 0, 0);
					drawContours(srcImg, contours, i, color, 2, 8, hierarchy);
					DetectArmourCenter[armor_cnt] = box[i].center;
					armor_cnt += 1;
				}
                else
			    {
				box[i].center.x = -1;
			    }
            }
			else
		    {
			box[i].center.x = 0;
		    }
		}	
	}

    
	//imshow("armourImg", armourImg);

	if (armor_cnt == 0)
	{
    cout << "can't find armor!" << endl;
	IsDetectDafuCenter=0;
    return;
    }

	for (int i = 0; i < armor_cnt; i++)
	{
		//circle(srcImg, Point(DetectArmourCenter[i].x, DetectArmourCenter[i].y), 2, Scalar(0, 0, 255), 1);
	}

//-------------------------------------------------find center------------------------------------------------------//
	dafu_center_cnt = 0;
	IsDafuCenter = 0;
	DafuCenter.x = size.width / 2;
	DafuCenter.y = size.height / 2;
	vector <Point2f> DetectDafuCenter(contours.size());     //flag of every contours:whether is it the center
	for (int i = 0; i < hierarchy.size(); i++)
	{
		//detect possible center
		float radius;
        //center feature:no child contours, no parent contours
		if ((hierarchy[i].val[2] == -1 )&& (hierarchy[i].val[3] == -1))
		{	
			minEnclosingCircle(contours[i], DetectDafuCenter[i], radius);
			//circle(srcImg, Point(DetectDafuCenter[i].x, DetectDafuCenter[i].y), radius, (255, 255,0), 2);
            //judge by radius
			if (radius > 10 || radius < 5)
			{
				DetectDafuCenter[i].x = -1;
			}
			else
			{
				dafu_center_cnt++;
				circle(srcImg, Point(DetectDafuCenter[i].x, DetectDafuCenter[i].y), radius, Scalar(255, 0, 0), 2);		
				cout << DetectDafuCenter[i].x << "  " << DetectDafuCenter[i].y << endl;
				drawContours(dstImage, contours, i, Scalar(0, 255, 0), 1, 8);
			}
		}
		else
		{
			DetectDafuCenter[i].x = -1;
		}
	}
	if (!dafu_center_cnt)
	{
		cout << "!!! can not find center in first stage!" << endl;
		IsDetectDafuCenter=0;
        return;
	}

	//confirm a real center
	
	for (int i = 0; i < contours.size(); i++)
	{
		if (DetectDafuCenter[i].x != -1)
		{
			int flag_error_center = 1;
			for (int j = 0; j < armor_cnt; j++)
			{
				float pixel_length = GetPixelLength(DetectDafuCenter[i], DetectArmourCenter[j]);
				cout << "pixel_length: " << pixel_length << endl;
				float transformation;
				//if (armor_cnt <= 2)
				//transformation = 0.15;
				//else
				transformation = 0.2;
                //judge by distance:dafu center to armor center
				if (pixel_length > real_armour_dafuCenter_pixel_length*(1 + transformation) || pixel_length < real_armour_dafuCenter_pixel_length*(1 - transformation))
				{
					flag_error_center = 0;
					continue;
				}
			}
			if (flag_error_center)
			{
				circle(srcImg, Point(DetectDafuCenter[i].x, DetectDafuCenter[i].y), 3, Scalar(0, 0, 255), 2);
				DafuCenter = DetectDafuCenter[i];
				IsDafuCenter = 1;
				continue;
			}
		}
    }

	if (!IsDafuCenter)
	{
		cout << "!!! can't find Dafu center!" << endl;
	    IsDetectDafuCenter=0;
        return;
	}
//--------------------------------------------find smallest circle-----------------------------------------------------//
    //find activate armor by flow light
	max_distance_Center=DafuCenter;
	IsDetectFlowLight = 0;
	//cout << "center:" << max_distance_Center.x << "  " << max_distance_Center.y << endl;
	max_distance = 0;
    float rad;
    vector<Point2f> DetectFlowLight(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		if (box[i].center.x==0)
		{
            Scalar color = Scalar(0,0,255);
			drawContours(armourImg, contours, i, color, 1, 8, hierarchy);
            minEnclosingCircle(contours[i], DetectFlowLight[i], rad);
            cout<<"rad is"<<rad<<endl;
            if(rad<7) 
            {
                float center_distance = GetPixelLength(DetectFlowLight[i], DafuCenter);
			    if (center_distance > max_distance)
			    {
				    max_distance = center_distance;//flow light flow from center to armor, so find the farest one can lock the flow light
				    max_distance_Center = DetectFlowLight[i];//this is cur flow light
				    IsDetectFlowLight = 1;
			    }
            }
		}
	}

    imshow("flowlight",armourImg);
    waitKey(5);

    if(!IsDetectFlowLight)
    {
        cout<<"can't comfirm which armor to shoot!"<<endl;
        return;
    }

	//circle(srcImg, Point(max_distance_Center.x, max_distance_Center.y), 3, Scalar(0, 255, 255), 2);
    shootArmour = DafuCenter;
	min_distance = size.width;
	IsShootArmor = 0;
	if (IsDetectFlowLight)
	{
		for (int i = 0; i < armor_cnt; i++)
		{
			float armour_circle_distance = GetPixelLength(DetectArmourCenter[i], max_distance_Center);
            //find the closest armor to flow light, which is the armor to shoot
			if (armour_circle_distance < min_distance)
			{
				min_distance = armour_circle_distance;
				shootArmour = DetectArmourCenter[i];
				IsShootArmor = 1;
			}	
		}
    }
    if (IsShootArmor)
	{
		cout << "shoot_armour: " << shootArmour.x << "  " << shootArmour.y << endl;
		circle(srcImg, shootArmour, 3, Scalar(0, 255, 0), 4);
	}
    
//--------------------------------------------predict-----------------------------------------------------//
    double angle;
    //if small
    if(status==PR_cw)
	    angle = -10;
    else if(status==PR_ccw)
        angle = 10;
    else if(status==PR_big) 
    {
        spd_cnt++;
        if(spd_cnt > 10)
        {
            Point repos=shootArmour-DafuCenter;
            curang=atan2(repos.y,repos.x);
            if(curang*lastang>=0)
                ang_spd=curang-lastang;
            lastang=curang;
            spd_cnt=0;
        }
        angle=ang_spd*shootDelay;
    }
        
	//cv::Mat rot_mat = cv::getRotationMatrix2D(DafuCenter, angle, 1.0);
	cv::Point2f dstPoints = getRotatePoint(grayImage, shootArmour, DafuCenter, angle);
	//cv::circle(srcImg, dstPoints, 2, Scalar(255, 255, 0), 2);
    if(dstPoints.x>480 || dstPoints.x<0 || dstPoints.y>640 || dstPoints.y<0)
    {
        IsDetectDafuCenter=0;
        cout<<"result out of range"<<endl;
        return;
    }
    
    //final result
    IsDetectDafuCenter=1;
    PredcitShootArmourCenter=myFilter(dstPoints,50,10);

    cv::circle(srcImg, PredcitShootArmourCenter, 2, Scalar(255, 255, 0), 2);
    imshow("circle", srcImg);
	waitKey(5);
	/*	writer << srcImg;
		waitKey(10);*/

}


//
///
/// \brief Dafu_Detector::CaculatePitchYawError  计算云台需要转的Yaw和Pitch使得摄像头的中轴线到指定点
/// \param Pixel_x          目标的像素横坐标
/// \param Pixel_y          目标的像素纵坐标
/// \return                 云台需要转的Yaw和Pitch角度
///
Point2f Dafu_Detector::CaculatePitchYawError(float Pixel_x, float Pixel_y)
{
  float PitchAngle = 0;
  float YawAngle = 0;
  float tan_pitch = (Pixel_y - myVideoCaptureProperties[CAP_PROP_FRAME_HEIGHT] / 2) / Camera_fy;
  float tan_yaw = (Pixel_x - myVideoCaptureProperties[CAP_PROP_FRAME_WIDTH] / 2) / Camera_fx;

  PitchAngle = atan(tan_pitch);
  YawAngle = atan(tan_yaw);

  PitchAngle = -PitchAngle / 3.14 * 180; //转化成单位度
  YawAngle = -YawAngle / 3.14 * 180; //转化成单位度
  return Point2f(YawAngle, PitchAngle);
}


///
/// \brief Dafu_Detector::GetPixelLength      计算像素坐标距离
/// \param PixelPointO                       第一个点的像素坐标
/// \param PixelPointA                       第二个点的像素坐标
/// \return                                  像素坐标距离
///
float Dafu_Detector::GetPixelLength(Point PixelPointO, Point PixelPointA)
{
  float PixelLength;
  PixelLength = powf((PixelPointO.x - PixelPointA.x), 2) + powf((PixelPointO.y - PixelPointA.y), 2);
  PixelLength = sqrtf(PixelLength);
  return PixelLength;
}


//Distance: the distance of camera and object
//简单的长尺度转换，未考虑相机的畸变和旋转
double Dafu_Detector::CvtRealLenghth2PixelLenghth(double RealLenghth_mm, double Distance_mm)
{
  double PixelLenghth_mm = 0;
  PixelLenghth_mm = Camera_fxy / Distance_mm * RealLenghth_mm;
  return PixelLenghth_mm;
}

//Distance: the distance of camera and object
///
/// \brief Dafu_Detector::CvtPixelLenghth2RealLenghth      在已知物体像素长度和相机与目标之间的距离的时候计算物体的真实长度
/// \param PixelLenghth     物体的像素长度，物体所在平面应尽量垂直于相机中轴线
/// \param Distance_mm      相机与目标之间的距离，单位mm
/// \return                 物体的真实长度，单位mm  
///
double Dafu_Detector::CvtPixelLenghth2RealLenghth(double PixelLenghth, double Distance_mm)
{
  double RealLenghth = 0;
  RealLenghth = Distance_mm * PixelLenghth / Camera_fxy;
  return RealLenghth;
}

//计算弹道下坠距离 单位mm
// HorizontalDistance 水平距离 单位mm
// PitchDegree        云台仰角 单位度 抬头为正
// BulletVelocity     子弹飞行速度
//CorrectionFactor    由于空气阻力影响，乘以修正系数
float  Dafu_Detector::CalculateBallisticDrop(float HorizontalDistance, float PitchDegree,float  BulletVelocity,float CorrectionFactor)
{
  CorrectionFactor=1;
  float PitchRad = PitchDegree / 180.0*3.14;   //把角度转换成弧度
  float BulletFlightTime = HorizontalDistance / ( BulletVelocity * cos(PitchRad) ); //子弹飞行时间为水平距离除以水平速度
  float BallisticDrop = 0.5*9.8*BulletFlightTime*BulletFlightTime;   //下坠为1/2*gt^2

  return BallisticDrop;
}

//计算云台转角增量
void Dafu_Detector::CalculateShootingPitch(Point2f CurrentPixel, Point2f &TargetPixel,float PitchDegree,float HorizontalDistance)
{
  float PitchRad = PitchDegree / 180.0*3.14;   //把角度转换成弧度
  float Length = HorizontalDistance / cos(abs(PitchRad)); //画幅中心到相机的距离

  float tanCurrentPixelPitch = tan(Camera_vertical_halfangle / 180.0*3.14) * (float)(Camera_frame_height / 2 - CurrentPixel.y) / (Camera_frame_height / 2);
  float CurrentPixelPitch = atan(tanCurrentPixelPitch);



}

///
/// \brief Dafu_Detector::GetROI   得到一幅大图中某个小斜矩形框内的图像
/// \param rotate_recte_rect      图像的范围是一个斜矩形
/// \param grayImage              输入的图像要求是二值化后的图像
/// \return                       输出斜矩形框内的图像
///
Mat Dafu_Detector::GetROI(RotatedRect rotate_recte_rect, Mat &grayImage)
{
  Mat ROI;
  Mat image = grayImage;
  Mat mask = Mat::zeros(image.size(), CV_8UC1);
  //画矩形
  Point2f rect[4];
  rotate_recte_rect.points(rect);
  for (int j = 0; j < 4; j++)
  {
    line(mask, rect[j], rect[(j + 1) % 4], Scalar(255), 2, 8);  //绘制最小外接矩形每条边
  }
  //设置种子点位置
  Point seed;
  seed.x = rotate_recte_rect.center.x;
  seed.y = rotate_recte_rect.center.y;
  //pi的值表示为 v(pi),if  v(seed)-loDiff<v(pi)<v(seed)+upDiff,将pi的值设置为newVal
  //使用漫水填充算法填充
  floodFill(mask, seed, 255, NULL, Scalar(0), Scalar(0), FLOODFILL_FIXED_RANGE);
  //mask(rect).setTo(255);
  Mat img2;
  image.copyTo(img2, mask);
  //imshow("mask", mask);
  //imshow("img2", img2);
  //设置ROI区域
  Rect rect2;
  rect2.x = rotate_recte_rect.center.x - 50, rect2.y = rotate_recte_rect.center.y - 50, rect2.width = 100, rect2.height = 100;//ROI0 的坐标
  if(rect2.x>(Camera_frame_width-100-5) ||rect2.x<10 ||rect2.y>(Camera_frame_height-100-5)||rect2.y<10)
    return Mat::zeros(100,100, CV_8UC1);
//------ZTL
  if(rect2.x<=0) rect2.x=1;
  if(rect2.y<=0) rect2.y=1;
  if(rect2.width<=0) rect2.width=1;
  if(rect2.height<=0) rect2.height=1;
  if((rect2.x+rect2.width)>=img2.cols) rect2.width=img2.cols-rect2.x;
  if((rect2.y+rect2.height)>=img2.rows) rect2.height=img2.rows-rect2.y;
  ROI = img2(rect2);

  return ROI;
}
///
/// \brief Dafu_Detector::PointRotate   将一个点绕中心旋转一定的角度
/// \param angle_degree     旋转的角度         
/// \param frame            需要将旋转后的坐标绘制在此图像上
/// \param Rotatecenter     旋转中心
/// \param RotatePoint      所需旋转的点坐标
/// \return                 旋转后的点坐标
///
Point2f Dafu_Detector::PointRotate(float angle_degree, Mat frame,Point2f Rotatecenter,Point2f RotatePoint)
{
    float theta = angle_degree / 180 * 3.14;
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    Point2f tgt2center = RotatePoint - Rotatecenter;
    float x = cos_theta * tgt2center.x - sin_theta * tgt2center.y;
    float y = sin_theta * tgt2center.x + cos_theta * tgt2center.y;
    Point2f pred2center = Point2f(x, y);
    Point2f pred = pred2center + Rotatecenter;
    //circle(frame, pred, 5, (0, 255, 255), 2);
    return pred;
}
