#include "armor_detect.hpp"

// #define DEBUG
#define SHOW_ARMOR_PUT_TEXT
#define SHOW_LIGHT_PUT_TEXT
#define SHOW_ROI_RECTANGLE
// #define SHOW_DRAW_RECT
// #define SHOW_DRAW_SPOT
#define SHOW_LIGHT_CONTOURS


    /**
     * 图像预处理+
     */
bool ArmorDetector::DetectArmor(Mat &img, Rect roi_rect)
{
    //提取矩阵
    Mat roi_image = img(roi_rect);
    //
    Point2f offset_roi_point(roi_rect.x, roi_rect.y);
    /**
     * @param binary_brightness_img 二值化亮度图
     * @param binary_color_img 二值化颜色图
     * @param gray  灰度图
     */
    Mat binary_brightness_img, binary_color_img;
    vector<cv::Mat> bgr;
    //按蓝 绿 红 分割出单通道图像
    split(roi_image, bgr);
    Mat result_img,gray;
// #ifdef DEBUG
//     Mat result_img2;
//     subtract(bgr[2], bgr[1], result_img);
//     subtract(bgr[0], bgr[2], result_img2);
//     imshow("灰度",gray);
//     imshow("红色通道",result_img);
//     imshow("蓝色通道",result_img2);
// #endif
    // 0 留下红色通道
    // 1 留下蓝色通道
    if(color_ == 0)
    {
        subtract(bgr[2], bgr[1], result_img);
    }else
    {
        subtract(bgr[0], bgr[2], result_img);
    }
    cuda::GpuMat roiImageGpu(roi_image);
    cuda::GpuMat grayGpu;
    cuda::cvtColor(roiImageGpu,grayGpu,COLOR_BGR2GRAY);
    cv::Ptr<cv::cuda::CannyEdgeDetector> canny_edge = cuda::createCannyEdgeDetector(10, 350, 3, false);
    cuda::GpuMat edge;
    cuda::GpuMat resultImgGpu(result_img);
    canny_edge->detect(resultImgGpu,edge);
    edge.download(binary_color_img);

    // canny_edge->detect(grayGpu, edge);
    // edge.download(binary_brightness_img);
    // grayGpu.download(gray);
    
    // Canny(gray,binary_brightness_img,50,350);
    // Canny(result_img,binary_color_img,50,350);
    // threshold(gray, binary_brightness_img, gray_th_, 255, THRESH_BINARY);
    // threshold(result_img, binary_color_img, color_th_, 255, THRESH_BINARY);
    
    // imshow("result_img", binary_color_img);//适合找灯条
    // imshow("binary_brightness_img", binary_brightness_img);//适合找灯条
    
#ifdef DEBUG
   // Mat binary_brightness_img2,binary_color_img2;
   // bilateralFilter(result_img,binary_color_img,25,50,10);
    //GaussianBlur(result_img,binary_color_img,cv::Size(3, 3), 0, 0);
    imshow("result_img", result_img);//适合找灯条
    // imshow("binary_brightness_img", binary_brightness_img);//适合数字识别
    imshow("binary_color_img", binary_color_img);
    //imshow("binary_brightness_img2", binary_brightness_img2);
   // imshow("binary_color_img2", binary_color_img2);

#endif
    // // **提取可能的灯条** -利用灯条（灰度）周围有相应颜色的光圈包围
    vector<vector<Point>> contours_light;
    //vector<vector<Point>> contours_brightness;
    findContours(binary_color_img, contours_light, RETR_EXTERNAL, CHAIN_APPROX_NONE);
   // findContours(binary_brightness_img, contours_brightness, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    //   Mat hole(binary_color_img.size(),CV_8U,Scalar(0));
    // Mat hole2(binary_brightness_img.size(),CV_8U,Scalar(0));
    //  drawContours(hole,contours_light,-1,Scalar(255),1);
    // drawContours(hole2,contours_brightness,-1,(0,255,0),1);
    //  imshow("contours_light", hole);//适合找灯条
    // imshow("binary_brightness_img", hole2);//适合数字识别
    vector<LED_Stick> LED_Stick_v;  // 声明所有可能的灯条容器
      
    //轮廓查找测试
    // for(size_t i = 0; i < contours_light.size(); i++)
    // {
    //     double length = arcLength(contours_light[i], true); // 灯条周长
    //             if (length > 15 && length <200)
    //             {
    //                 RotatedRect RRect = fitEllipse(contours_light[i]);
    //     // 旋转矩形提取四个点
    //                 Point2f rect_point[4];
    //                 RRect.points(rect_point);
    //                 for (int i = 0; i < 4 ; i++)
    //                 {
    //                     line(img, rect_point[i]+offset_roi_point, rect_point[(i+1)%4]+offset_roi_point, Scalar(255,0,255),1);
    //                 }
    //             }
        
    // }

    //#pragma omp for

        for(size_t i = 0; i < contours_light.size(); i++)
        {       
                double area = contourArea(contours_light[i]);
                if (area < 20.0 || 300 < area) continue;
                double length = arcLength(contours_light[i], true); // 灯条周长
                if (length > 15 && length <150)
                {                    // 使用拟合椭圆的方法要比拟合最小矩形提取出来的角度更精确
                    RotatedRect RRect = fitEllipse(contours_light[i]);
#ifdef SHOW_LIGHT_CONTOURS
                    // 旋转矩形提取四个点
                    Point2f rect_point[4];
                    RRect.points(rect_point);
                    for (int i = 0; i < 4 ; i++)
                    {
                        line(img, rect_point[i]+offset_roi_point, rect_point[(i+1)%4]+offset_roi_point, Scalar(255,0,255),2);
                    }
#endif
                    // 角度换算，将拟合椭圆0~360 -> -180~180
                    if(RRect.angle>90.0f)
                        RRect.angle =  RRect.angle - 180.0f;
#ifdef SHOW_LIGHT_PUT_TEXT
                    // putText(img, to_string(RRect.angle), RRect.center + Point2f(2,2) + offset_roi_point, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
#endif
                    if (fabs(RRect.angle) <= 30)  // 超过一定角度的灯条不要
                    {
                        LED_Stick r(RRect);
                        LED_Stick_v.push_back(r);
                    }
                }
                // break;
            
        }

   // **寻找可能的装甲板** -遍历每个可能的灯条, 两两灯条拟合成装甲板进行逻辑判断
    for(size_t i = 0; i < LED_Stick_v.size() ; i++)
    {
        for(size_t j = i + 1; j < LED_Stick_v.size() ; j++)
        {
            armor arm_tmp( LED_Stick_v.at(i), LED_Stick_v.at(j) );
            if (arm_tmp.error_angle < 8.0f)
            {
#ifdef SHOW_ARMOR_PUT_TEXT
                // putText(img, to_string(arm_tmp.rect.width/(arm_tmp.rect.height+0.0001)), arm_tmp.center + Point_<int>(offset_roi_point) , FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
#endif
                // TODO(cz): 推荐加入灯条宽度要小于装甲板宽度的条件
                if(arm_tmp.is_suitable_size())
                {
                    // TODO(cz): 推荐使用255值的面积进行判断
                    if(arm_tmp.get_average_intensity(gray)< 300 )
                    {
                        arm_tmp.max_match(LED_Stick_v, i, j);

                    }
                }
            }
        }
    }

    // **分类装甲板** -根据灯条匹配状态得到最终装甲板
    vector<armor> final_armor_list;
    for(size_t i = 0; i < LED_Stick_v.size() ; i++)
    {
        if(LED_Stick_v.at(i).matched)
        {
            LED_Stick_v.at(LED_Stick_v.at(i).match_index).matched = false; //clear another matching flag
            armor arm_tmp( LED_Stick_v.at(i), LED_Stick_v.at(LED_Stick_v.at(i).match_index));
            final_armor_list.push_back(arm_tmp);
        }
    }

    // **选择装甲板** -根据距离图像中心最短选择
    float dist=1e8;
    bool found_flag = false;
    armor target;
    Point2f roi_center(roi_rect.width/2, roi_rect.height/2);
    float dx,dy;
    for (size_t i = 0; i < final_armor_list.size() ; i++ )
    {
#ifdef FAST_DISTANCE
        dx = fabs(final_armor_list.at(i).center.x - roi_center.x);
        dy = fabs(final_armor_list.at(i).center.y - roi_center.y);
#else
        dx = pow((final_armor_list.at(i).center.x - roi_center.x), 2.0f);
        dy = pow((final_armor_list.at(i).center.y - roi_center.y), 2.0f);
#endif
        if( dx + dy < dist){
            target = final_armor_list.at(i);
            dist = dx + dy;
        }
#ifdef SHOW_DRAW_RECT
        final_armor_list.at(i).draw_rect(img, offset_roi_point);
#endif
        found_flag = true;
    }
#ifdef SHOW_ROI_RECTANGLE
    rectangle(img, roi_rect,Scalar(255, 0, 255),1);
#endif
    // **计算装甲板四个点顶点** -用于pnp姿态结算
    // TODO(cz): 四个点的不同的bug修复
    RotatedRect target_rect;
    if(found_flag)
    {
        cout<<"found"<<endl;
#ifdef SHOW_DRAW_SPOT
        target.draw_spot(img, offset_roi_point);
#endif
        Point2f point_tmp[4];
        Point2f point_2d[4];
        // 左右灯条分类，本别提取装甲板四个外角点
        RotatedRect R, L;
        if(target.Led_stick[0].rect.center.x > target.Led_stick[1].rect.center.x)
        {
            R = target.Led_stick[0].rect;
            L = target.Led_stick[1].rect;
        }else
        {
            R = target.Led_stick[1].rect;
            L = target.Led_stick[0].rect;
        }
        L.points(point_tmp);
        point_2d[0] = point_tmp[1];
        point_2d[3] = point_tmp[0];
        R.points(point_tmp);
        point_2d[1] = point_tmp[2];
        point_2d[2] = point_tmp[3];
        // 计算补偿，用于调试调整准心
        Point2f offset_point;
        if(cap_mode_ == 0)
        {
            offset_point = Point2f(100, 100) - Point2f(short_offset_x_,short_offset_y_);
        }
        else
        {
            offset_point = Point2f(100, 100) - Point2f(long_offset_x_,long_offset_y_);
        }

        points_2d_.clear();
        vector<Point2f> points_roi_tmp;
        for(int i=0;i<4;i++)
        {
            points_roi_tmp.push_back(point_2d[i] + offset_roi_point);
            points_2d_.push_back(point_2d[i] + offset_roi_point +offset_point);
#ifdef SHOW_LIGHT_PUT_TEXT
           putText(img, to_string(i), points_2d_.at(i), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
           circle(img, points_2d_.at(i), 5, Scalar(255, 255, 255), -1);
           circle(img, points_2d_.at(i), 3, Scalar(i*50, i*50, 255), -1);
#endif
        }
        // 计算当前装甲板类型，到后面task中还有滤波，可以有误差
        float armor_h = target.rect.height;
        float armor_w = target.rect.width;
        if(armor_w / armor_h < 3.3f)
            is_small_ = 1;
        else
            is_small_ = 0;

        //计算ROI的相关参数
        last_target_ = boundingRect(points_roi_tmp);
        rectangle(img, last_target_,Scalar(255,255,255), 1);
        lost_cnt_ = 0;
    }else {
        //计算ROI的相关参数
        lost_cnt_ ++;
    }
    detect_cnt_++;
    return found_flag;
}
    
    /**
     * 
     */
Rect ArmorDetector::GetRoi(const Mat &img)
{
    Size img_size = img.size();
    Rect rect_tmp = last_target_;
    Rect rect_roi;
    
    //初始条件下，rect_tmp为0，所以获得的ROI是输入图像大小
    if(rect_tmp.x == 0 || rect_tmp.y == 0
            || rect_tmp.width == 0 || rect_tmp.height == 0
            || lost_cnt_ >= 15 || detect_cnt_%100 == 0
            )
    {
        last_target_ = Rect(0,0,img_size.width, img_size.height);
        rect_roi = Rect(0,0,img_size.width, img_size.height);
        return rect_roi;
    }
    else
    {
        float scale = 2;
        if (lost_cnt_ < 30)
            scale = 3;
        else if(lost_cnt_ <= 60)
            scale = 4;
        else if(lost_cnt_ <= 120)
            scale = 5;

        int w = int(rect_tmp.width * scale);
        int h = int(rect_tmp.height * scale);
        int x = int(rect_tmp.x - (w - rect_tmp.width)*0.5f);
        int y = int(rect_tmp.y - (h - rect_tmp.height)*0.5f);

        rect_roi = Rect(x, y, w, h);
        
        if (rect_roi.x < 0)
            rect_roi.x = 0;
        if (rect_roi.x + rect_roi.width > img_size.width)
            rect_roi.width = img_size.width - rect_roi.x;
        if (rect_roi.y < 0)
            rect_roi.y = 0;
        if (rect_roi.y + rect_roi.height > img_size.height)
            rect_roi.height = img_size.height - rect_roi.y;
        if (rect_roi.width <= 0 || rect_roi.height <= 0)
            rect_roi = Rect(0,0,img_size.width, img_size.height);
        
    }
    return rect_roi;
}
/**
 * @brief class armor::armor的实现
 */
armor::armor(){

}
/**
 * @brief class armor::armor的实现
 */
armor::armor(const LED_Stick& L1, const LED_Stick& L2){
    Led_stick[0]= L1;
    Led_stick[1]= L2;
    error_angle = fabs(L1.rect.angle - L2.rect.angle);

    rect.width = abs(static_cast<int>(L1.rect.center.x - L2.rect.center.x));
    rect.height = static_cast<int>((L1.rect.size.height + L1.rect.size.height)/2);
    center.x = static_cast<int>((L1.rect.center.x + L2.rect.center.x)/2);
    center.y = static_cast<int>((L1.rect.center.y + L2.rect.center.y)/2);
    rect.x = center.x - rect.width/3;
    rect.y = center.y - rect.height/3;
    rect.width*= 2.0/3;
    rect.height*= 2.0/3;
}

/**
 * @brief 判断可能的装甲板是否符合尺寸
 */
bool armor::is_suitable_size(void) const
{
    // 两个灯条体型相似
    if(Led_stick[0].rect.size.height*0.7f < Led_stick[1].rect.size.height
            && Led_stick[0].rect.size.height*1.3f > Led_stick[1].rect.size.height)
    {
        float armor_width = fabs(Led_stick[0].rect.center.x - Led_stick[1].rect.center.x);
        if(armor_width > Led_stick[0].rect.size.width
                && armor_width > Led_stick[1].rect.size.width
                && armor_width > (Led_stick[0].rect.size.width+Led_stick[1].rect.size.width)*3)
        {
            float h_max = (Led_stick[0].rect.size.height + Led_stick[1].rect.size.height)/2.0f;
            // 两个灯条高度差不大
            if(fabs(Led_stick[0].rect.center.y - Led_stick[1].rect.center.y) < 0.8f* h_max )
            {
                // 长宽比判断
                if(h_max*4.0f > rect.width && h_max < 1.2f* rect.width)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

void armor::draw_rect( Mat& img, Point2f roi_offset_point) const
{
    rectangle(img, rect + Point_<int>(roi_offset_point), Scalar(255,255,255), 1);
}

void armor::draw_spot(Mat &img, Point2f roi_offset_point) const
{
    circle(img, center + Point_<int>(roi_offset_point), int(rect.height/4), Scalar(0,0,255), -1);
}


int armor::get_average_intensity(const Mat& img) {
    if(rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y < 1
            || rect.width + rect.x > img.cols || rect.height + rect.y > img.rows)
        return 255;
    Mat roi = img(Range(rect.y, rect.y + rect.height), Range(rect.x, rect.x + rect.width) );
            imshow("roi ", roi);
    average_intensity = static_cast<int>(mean(roi).val[0]);
    return average_intensity;
}


void armor::max_match(vector<LED_Stick>& LED,size_t i,size_t j){
    RotatedRect R, L;
    if(Led_stick[0].rect.center.x > Led_stick[1].rect.center.x)
    {
        R = Led_stick[0].rect;
        L = Led_stick[1].rect;
    }else
    {
        R = Led_stick[1].rect;
        L = Led_stick[0].rect;
    }

    float angle_8 = L.angle - R.angle;
    //    cout << L.angle << " "<< R.angle << endl;
    if(angle_8 < 1e-3f)
        angle_8 = 0.0f;
    float f = error_angle + 0.5 * angle_8;
    if(!LED.at(i).matched && !LED.at(j).matched )
    {

        LED.at(i).matched = true;
        LED.at(i).match_index = j;
        LED.at(j).matched = true;
        LED.at(j).match_index = i;
        LED.at(i).match_factor = f;
        LED.at(j).match_factor = f;
    }
    if(LED.at(i).matched && !LED.at(j).matched)
    {
        if(f < LED.at(i).match_factor)
        {
            LED.at(LED.at(i).match_index).matched = false;
            LED.at(i).match_factor = f;
            LED.at(i).match_index = j;
            LED.at(j).matched = true;
            LED.at(j).match_factor = f;
            LED.at(j).match_index = i;

        }
    }
    if(LED.at(j).matched && !LED.at(i).matched)
    {
        if(f < LED.at(j).match_factor )
        {
            LED.at(LED.at(j).match_index).matched = false;
            LED.at(j).match_factor = f;
            LED.at(j).match_index = i;
            LED.at(i).matched = true;
            LED.at(i).match_factor = f;
            LED.at(i).match_index = j;
        }
    }
    if(LED.at(j).matched && LED.at(i).matched
            && LED.at(i).match_factor > f && LED.at(j).match_factor > f)
    {
        LED.at(LED.at(j).match_index).matched = false;
        LED.at(LED.at(i).match_index).matched = false;
        LED.at(i).matched = true;
        LED.at(i).match_factor = f;
        LED.at(i).match_index = j;
        LED.at(j).matched = true;
        LED.at(j).match_factor = f;
        LED.at(j).match_index = i;
    }
}