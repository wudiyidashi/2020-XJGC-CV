#include <stdio.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "buff_detect.hpp"

using namespace cv;
using namespace std;

double clockToMilliseconds(clock_t ticks){
    // units/(units/time) => time (seconds) * 1000 = milliseconds
    return (ticks/(double)CLOCKS_PER_SEC)*1000.0;
}

#define VIDEO

int main()
{	
    clock_t current_ticks, delta_ticks;
    clock_t fps = 0;
	#ifdef VIDEO
    VideoCapture capture("bufftest.avi");
	// VideoCapture capture("test.mp4");
//	Size size = Size(capture.get(CAP_PROP_FRAME_WIDTH), capture.get(CAP_PROP_FRAME_HEIGHT));
//	int myFourCC = VideoWriter::fourcc('M', 'J', 'P', 'G');
//	double rate = capture.get(CAP_PROP_FPS);
//	VideoWriter writer("./buffer.avi", myFourCC, rate, size, true);
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

    //...

    clock_t deltaTime = 0;
    clock_t deltaTimes =0;
    unsigned int frames = 0;
    double  frameRate = 30;
    double  averageFrameTimeMilliseconds = 33.333;

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
        clock_t beginFrame = clock();
		capture >> frame;
		// imshow("读取视频",frame);
		bool command = buff_detector.BuffDetectTask(frame, other_param);
		//  namedWindow("读取视频",WINDOW_NORMAL);
//		 imshow("读取视频",frame);
//		 waitKey(5);
        clock_t endFrame = clock();
        deltaTimes = endFrame - beginFrame;
        deltaTime += endFrame - beginFrame;
        std::cout<<"every time used for delta:"<<deltaTimes/1000<<"ms"<<std::endl;
            frames ++;

            if( clockToMilliseconds(deltaTime)>1000.0){ //every second
                frameRate = (double)frames*0.5 +  frameRate*0.5; //more stable
                frames = 0;
                deltaTime -= CLOCKS_PER_SEC;
                averageFrameTimeMilliseconds  = 1000.0/(frameRate==0?0.001:frameRate);

                std::cout<<"FPS:"<<averageFrameTimeMilliseconds<<std::endl;
            }

	}
	return 0;
}

