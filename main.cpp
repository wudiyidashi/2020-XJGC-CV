#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "armor_detect.hpp"
#include "buff_detect.hpp"

using namespace cv;
using namespace std;



#define VIDEO

int main()
{	
	#ifdef VIDEO
	VideoCapture capture("buff.avi");
	// VideoCapture capture("test.mp4");
	Size size = Size(capture.get(CAP_PROP_FRAME_WIDTH), capture.get(CAP_PROP_FRAME_HEIGHT));
	int myFourCC = VideoWriter::fourcc('M', 'J', 'P', 'G');
	double rate = capture.get(CAP_PROP_FPS);
	VideoWriter writer("./buffer.avi", myFourCC, rate, size, true);
	if(!capture.isOpened())
	{
		std::cout <<  "video not open" << std::endl;
	}
	Mat frame,finalVideo;
	#else
	// //读取视频或摄像头
	// VideoCapture capture;
	Mat frame;
	VideoCapture capture("/dev/video2");
	//设定capture取图像的帧率，默认是30
	capture.set(CAP_PROP_FPS,60);

	#endif
	// ArmorDetector armor_detector;
	BuffDetector buff_detector;
	OtherParam other_param;
	    other_param.mode = 0;
        other_param.color = 1;
        other_param.buff_offset_x = 0;
        other_param.buff_offset_y = 0;
//        GimDataPro.ProcessGimbalData(raw_gimbal_ya
	while (capture.isOpened())
	{	
		capture >> frame;
		// bool flag=armor_detector.ArmorDetectTask(frame);
		// imshow("读取视频",frame);
		bool command = buff_detector.BuffDetectTask(frame, other_param);
		//  namedWindow("读取视频",WINDOW_NORMAL);
		 imshow("读取视频",frame);
		 writer << frame;
		 waitKey(5);
	}
	return 0;
}

