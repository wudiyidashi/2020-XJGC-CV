#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>


using namespace cv;
using namespace std;

/**
 * @brief 装甲板检测器
 */
class ArmorDetector
{
public:
    int ArmorDetectTask(Mat &img)
{   
    Rect roi = GetRoi(img);
    DetectArmor(img,roi);
    
    return 1;
}
private:
    /**
     * @brief GetRoi 获取图像ROI区域
     * @param img 输入图像
     * @return 返回感兴趣区域的矩形
     */
    Rect GetRoi(const Mat &img);
    /**
     * @brief DetectArmor 装甲板识别函数
     * @param img 输入图像
     * @param roi_rect ROI_矩形区域
     * @return
     */
    bool DetectArmor(Mat &img, Rect roi_rect);

public:
    // ｒｏｉ参数
    Rect last_target_;
    int lost_cnt_ = 0;
    int detect_cnt_ = 0;

public:
     // 外部参数
    int color_= 1;
    int cap_mode_;
    
public:
    // 调试参数
    int short_offset_x_ = 100;
    int short_offset_y_ = 100;
    int long_offset_x_ = 100;
    int long_offset_y_ = 100;
    int color_th_ = 16;
    int gray_th_ = 60;
    
private:
    float dist_ = 3000; // 通过距离更换相机
    float r_ = 0.5; // 距离刷新率 (0-1)
    int update_cap_cnt = 0; // 用于强制限制相机更新频率
    float distance_ = 0;
    float angle_x_ = 0;
    float angle_y_ = 0;
    vector<Point2f> points_2d_;

private:
    // 判断大小装甲板类型相关参数
    std::list<bool> history_;
    int filter_size_ = 5;
    bool is_small_;
};
/**
 * @brief 装甲板灯条相关数据信息
 */
class LED_Stick{
public:

    LED_Stick():matched(false){}

    LED_Stick(const RotatedRect& R){
        rect.angle = R.angle;
        rect.center = R.center;
        rect.size = R.size;
        matched = false;
    }

    RotatedRect rect;   // 装甲板灯条相关数据信息
    bool matched;       // 匹配状态， 用于灯条匹配
    size_t match_index; // 匹配对应的灯条序号， 用于灯条匹配
    float match_factor; // 匹配强度， 用于灯条匹配
};
/**
 * @brief 装甲板相关数据信息
 */
class armor{
public:
    armor();
    armor(const LED_Stick& L1, const LED_Stick& L2);
    // 画出装甲板
    void draw_rect( Mat& img, Point2f roi_offset_poin) const;    
    void draw_spot(Mat &img, Point2f roi_offset_point) const;
    // 计算装甲板roi平均色彩强度，用于筛选装甲板中心有灯条
    int get_average_intensity(const Mat& img) ; 
    // 灯条匹配算法
    void max_match(vector<LED_Stick>& LED, size_t i, size_t j); 
    // 判断可能的装甲板是否符合尺寸
    bool is_suitable_size(void) const;          

    LED_Stick Led_stick[2];  // 装甲板的两个灯条
    float error_angle;       // 两个灯条的误差的角度
    Point2i center;          // 装甲板中心点
    Rect2i rect;             // 装甲板roi矩形
    int average_intensity;   // 装甲板roi的平均色彩强度
};